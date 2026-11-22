"""
glb_to_rzobj.py  -  converte .glb (Blender, etc.) para .rzobj v3.

Formato .rzobj v3 (little-endian)
  Cabecalho (32 bytes):
    0   4  magic "RZOB"
    4   2  versao (3)
    6   2  flags: bit0 = tem UVs (sempre 1) | bit2 = indices 32 bits
    8   4  vertCount        (pool unico)
   12   4  indexCount       (pool unico)
   16   2  meshCount
   18   2  texCount
   20  12  reservado
  Em sequencia:
    vertices : float32 * 3 * vertCount   (transformacao dos nodes ja aplicada)
    uvs      : float32 * 2 * vertCount
    indices  : uint16/uint32 * indexCount
    sub-malhas (32 bytes cada):
        u32 indexStart | u32 indexCount | i16 texIndex | u16 flags | char name[20]
        flags: bit0 = double sided (sem culling) | bit1 = color-key (pixel 0x0000 = transparente)
    texturas (8 bytes cada):
        u16 w | u16 h | u8 wrap (0 = repeat, 1 = clamp) | 3 bytes reservados
    dados das texturas na ordem da tabela:
        uint16 * w * h  RGB565 com bytes trocados (ordem do painel). w e h sao potencias de 2.

Novidades da v3 (corrigem texturas esticadas / partes sumindo):
  - texturas em potencia de 2 + wrap REPEAT/CLAMP vindo do sampler do GLB (tiling funciona)
  - baseColorFactor do material aplicado (materiais sem textura viram uma cor solida)
  - alphaMode MASK/BLEND vira color-key (cabelo, folhas, cilios...)
  - doubleSided do material vira flag (o culling so vale para faces de lado unico)
  - TEXCOORD_n correto quando o material usa outro canal de UV
  - KHR_texture_transform (offset / rotation / scale / texCoord) do baseColorTexture e embutido nas UVs
    (sem isso o tiling do material era ignorado e a textura aparecia esticada)

Novidades da v4 (skinning + clipes de animacao; so e gravada quando o GLB tem skin, senao continua v3):
  flags bit3 (8) = tem skin.  Ordem no arquivo:
    cabecalho | vertices | uvs | indices |
    [skin] pesos por vertice : vertCount * 8 bytes = u8 joint[4] + u8 weight[4] (soma 255; weight[0]==0 => vertice estatico)
    sub-malhas | tabela de texturas | dados das texturas |
    [skin] cabecalho (56 B): u16 boneCount | u16 clipCount | u32 reservado | float rootMat[12] (3x4 por linhas)
           ossos (92 B cada, pai sempre ANTES do filho):
               i16 parent (-1 = raiz) | u16 pad | float t[3] | float q[4] (xyzw) | float s[3] | float invBind[12] (3x4 por linhas)
           clipes (28 B cada): char name[20] | u16 nFrames | u16 flags | float duracao(s)
           keyframes de cada clipe, na ordem da tabela, [frame][osso] (32 B cada):
               float t[3] | i16 q[4] (quaternion * 32767) | float s[3]
  Os clipes sao REAMOSTRADOS a taxa fixa (parametro fps): STEP / LINEAR / CUBICSPLINE do GLB viram quadros uniformes,
  e o firmware so interpola linearmente entre dois quadros.
  Pose do osso b: pose[b] = mundo[b] * invBind[b], com mundo[b] = (pai ou rootMat) * T*R*S local.
  Vertice skinned = soma(peso * pose[osso] * posicao_bind)  (posicao NAO recebe a matriz do node, como manda a spec).
"""
import base64
import io
import math
import os
import struct
import sys

import numpy as np
from PIL import Image
from pygltflib import GLTF2

COMP = {5120: np.int8, 5121: np.uint8, 5122: np.int16,
        5123: np.uint16, 5125: np.uint32, 5126: np.float32}
NCOMP = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT4": 16}

FLAG_UV, FLAG_IDX32, FLAG_SKIN = 1, 4, 8
MAX_BONES = 48          # tem que bater com MAX_BONES do firmware
MF_DOUBLE, MF_KEY = 1, 2
CLAMP_TO_EDGE = 33071


# ---------------------------------------------------------------- glTF helpers
def read_accessor(gltf, idx, blob):
    acc = gltf.accessors[idx]
    dt = np.dtype(COMP[acc.componentType])
    n = NCOMP[acc.type]
    elem = dt.itemsize * n
    if acc.bufferView is None:
        return np.zeros((acc.count, n), dtype=dt)
    bv = gltf.bufferViews[acc.bufferView]
    base = (bv.byteOffset or 0) + (acc.byteOffset or 0)
    stride = bv.byteStride or elem
    if stride == elem:
        arr = np.frombuffer(blob, dtype=dt, count=acc.count * n, offset=base).reshape(acc.count, n)
    else:
        raw = np.frombuffer(blob, dtype=np.uint8, offset=base)
        view = np.lib.stride_tricks.as_strided(raw, shape=(acc.count, elem), strides=(stride, 1))
        arr = np.ascontiguousarray(view).view(dt).reshape(acc.count, n)
    if acc.normalized and dt.kind in "iu":
        arr = arr.astype(np.float32) / float(np.iinfo(dt).max)
    return arr


def get_image_bytes(gltf, image, blob, base_dir):
    if image.uri:
        if image.uri.startswith("data:"):
            return base64.b64decode(image.uri.split(",", 1)[1])
        with open(os.path.join(base_dir, image.uri), "rb") as f:
            return f.read()
    bv = gltf.bufferViews[image.bufferView]
    start = bv.byteOffset or 0
    return blob[start:start + bv.byteLength]


def lin2srgb(c):
    c = max(0.0, min(1.0, float(c)))
    return 12.92 * c if c <= 0.0031308 else 1.055 * (c ** (1 / 2.4)) - 0.055


def uv_transform(tex_info):
    """KHR_texture_transform -> (matriz 3x3 ou None, texCoord ou None). Ordem da spec: T * R * S."""
    ext = (getattr(tex_info, "extensions", None) or {}).get("KHR_texture_transform")
    if not ext:
        return None, None
    ox, oy = ext.get("offset", [0.0, 0.0])
    sx, sy = ext.get("scale", [1.0, 1.0])
    r = float(ext.get("rotation", 0.0))
    c, s = math.cos(r), math.sin(r)
    T = np.array([[1, 0, ox], [0, 1, oy], [0, 0, 1]], dtype=np.float64)
    R = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]], dtype=np.float64)
    S = np.array([[sx, 0, 0], [0, sy, 0], [0, 0, 1]], dtype=np.float64)
    return T @ R @ S, ext.get("texCoord")


def material_info(gltf, prim):
    """-> (img_idx, texcoord, tint(r,g,b), clamp, keyed, double_sided, uv_matrix)"""
    if prim.material is None:
        return (0 if len(gltf.images) == 1 else None), 0, (255, 255, 255), False, False, True, None
    mat = gltf.materials[prim.material]
    pbr = mat.pbrMetallicRoughness
    fac = list(pbr.baseColorFactor) if (pbr and pbr.baseColorFactor) else [1, 1, 1, 1]
    tint = tuple(int(round(255 * lin2srgb(fac[i]))) for i in range(3))
    img_idx, tc, clamp, xf = None, 0, False, None
    bct = pbr.baseColorTexture if pbr else None
    if bct is not None:
        try:
            t = gltf.textures[bct.index]
            img_idx = t.source
            tc = bct.texCoord or 0
            xf, tc_ext = uv_transform(bct)
            if tc_ext is not None:
                tc = tc_ext
            if t.sampler is not None and gltf.samplers:
                s = gltf.samplers[t.sampler]
                clamp = (s.wrapS == CLAMP_TO_EDGE and s.wrapT == CLAMP_TO_EDGE)
        except Exception:
            img_idx = None
    keyed = (mat.alphaMode or "OPAQUE") in ("MASK", "BLEND")
    return img_idx, tc, tint, clamp, keyed, bool(mat.doubleSided), xf


def node_local(node):
    if node.matrix:
        return np.array(node.matrix, dtype=np.float64).reshape(4, 4).T
    T = np.eye(4)
    if node.translation:
        T[:3, 3] = node.translation
    R = np.eye(4)
    if node.rotation:
        x, y, z, w = node.rotation
        R[:3, :3] = [[1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
                     [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
                     [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)]]
    S = np.diag([*(node.scale or [1.0, 1.0, 1.0]), 1.0])
    return T @ R @ S


def collect_instances(gltf):
    out = []

    def walk(i, parent):
        n = gltf.nodes[i]
        W = parent @ node_local(n)
        if n.mesh is not None:
            out.append((n.mesh, W, n.name, n.skin))
        for c in (n.children or []):
            walk(c, W)

    if gltf.scenes:
        for r in gltf.scenes[gltf.scene or 0].nodes:
            walk(r, np.eye(4))
    if not out:
        out = [(i, np.eye(4), None, None) for i in range(len(gltf.meshes))]
    return out


def pow2_round(n):
    return max(1, 1 << int(round(math.log2(max(1, n)))))


def to_rgb565_be(img, keyed):
    a = np.asarray(img.convert("RGBA"), dtype=np.uint16)
    r, g, b, al = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
    c = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
    if keyed:                                   # 0x0000 = transparente; preto real vira 0x0001
        c = np.where(al < 128, 0, np.where(c == 0, 1, c)).astype(np.uint16)
    return c.astype(">u2").tobytes()


# ---------------------------------------------------------------- skin / animacao
KEY_DT = np.dtype([("t", "<f4", (3,)), ("q", "<i2", (4,)), ("s", "<f4", (3,))])   # 32 bytes por (quadro, osso)
assert KEY_DT.itemsize == 32


def mat_to_quat(R):
    """matriz de rotacao 3x3 (vetores coluna) -> quaternion xyzw normalizado"""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * S, (R[2, 1] - R[1, 2]) / S, (R[0, 2] - R[2, 0]) / S, (R[1, 0] - R[0, 1]) / S
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x, y, z = (R[2, 1] - R[1, 2]) / S, 0.25 * S, (R[0, 1] + R[1, 0]) / S, (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x, y, z = (R[0, 2] - R[2, 0]) / S, (R[0, 1] + R[1, 0]) / S, 0.25 * S, (R[1, 2] + R[2, 1]) / S
    else:
        S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x, y, z = (R[1, 0] - R[0, 1]) / S, (R[0, 2] + R[2, 0]) / S, (R[1, 2] + R[2, 1]) / S, 0.25 * S
    q = np.array([x, y, z, w], dtype=np.float64)
    return q / np.linalg.norm(q)


def node_trs(node):
    """TRS local do node (decompoe node.matrix se existir) -> (t[3], q[4] xyzw, s[3])"""
    if node.matrix:
        M = np.array(node.matrix, dtype=np.float64).reshape(4, 4).T
        M3 = M[:3, :3]
        s = np.linalg.norm(M3, axis=0)
        if np.linalg.det(M3) < 0:
            s[0] = -s[0]
        s = np.where(np.abs(s) < 1e-12, 1.0, s)
        return M[:3, 3].copy(), mat_to_quat(M3 / s[None, :]), s
    t = np.array(node.translation or [0.0, 0.0, 0.0], dtype=np.float64)
    q = np.array(node.rotation or [0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    q = q / max(np.linalg.norm(q), 1e-12)
    s = np.array(node.scale or [1.0, 1.0, 1.0], dtype=np.float64)
    return t, q, s


def node_parents(gltf):
    par = {}
    for i, n in enumerate(gltf.nodes):
        for c in (n.children or []):
            par[c] = i
    return par


def node_world(gltf, par, i, cache):
    if i in cache:
        return cache[i]
    W = node_local(gltf.nodes[i])
    if i in par:
        W = node_world(gltf, par, par[i], cache) @ W
    cache[i] = W
    return W


def build_skeleton(gltf, skin_idx, blob):
    """Esqueleto do skin escolhido. Os ossos sao REORDENADOS (pai sempre antes do filho);
    `remap` converte o indice antigo (JOINTS_0) para o novo."""
    skin = gltf.skins[skin_idx]
    joints = list(skin.joints)
    if not joints:
        raise SystemExit("Skin sem joints.")
    if len(joints) > MAX_BONES:
        raise SystemExit(f"Skin com {len(joints)} ossos (maximo {MAX_BONES}). Reduza o esqueleto "
                         f"ou aumente MAX_BONES aqui e no firmware.")
    par = node_parents(gltf)
    cache = {}
    jidx = {n: k for k, n in enumerate(joints)}

    def depth(n):
        d = 0
        while n in par:
            n = par[n]
            d += 1
        return d

    order = sorted(range(len(joints)), key=lambda k: (depth(joints[k]), k))     # indices antigos, na ordem nova
    remap = np.zeros(len(joints), dtype=np.int64)
    for new, old in enumerate(order):
        remap[old] = new

    if skin.inverseBindMatrices is not None:
        ibm = read_accessor(gltf, skin.inverseBindMatrices, blob).astype(np.float64).reshape(-1, 4, 4)
        ibm = np.transpose(ibm, (0, 2, 1))                      # glTF e coluna-maior -> linha-maior
    else:
        ibm = np.array([np.linalg.inv(node_world(gltf, par, n, cache)) for n in joints])
    ibm = ibm[order]

    bones, root = [], None
    for nb, old in enumerate(order):
        node = joints[old]
        a, pj, skipped = node, -1, False
        while a in par:
            a = par[a]
            if a in jidx:
                pj = int(remap[jidx[a]])
                break
            skipped = True
        if pj == -1:
            rw = node_world(gltf, par, par[node], cache) if node in par else np.eye(4)
            if root is None:
                root = rw
            elif not np.allclose(root, rw, atol=1e-5):
                print(f"[aviso] osso '{gltf.nodes[node].name}': raiz com pai diferente das outras; usando a 1a")
        elif skipped:
            print(f"[aviso] osso '{gltf.nodes[node].name}': ha node(s) que nao sao joint entre ele e o pai; "
                  f"a transformacao deles e ignorada")
        t, q, s = node_trs(gltf.nodes[node])
        bones.append({"parent": pj, "t": t, "q": q, "s": s, "name": gltf.nodes[node].name or f"bone{nb}"})
    if root is None:
        root = np.eye(4)
    W0 = node_world(gltf, par, joints[order[0]], cache) @ ibm[0]
    return {"bones": bones, "joint_nodes": [joints[o] for o in order], "remap": remap, "inv": ibm,
            "root": root, "flip": bool(np.linalg.det(W0[:3, :3]) < 0)}


def pack_skin(j, w, remap):
    """JOINTS_0/WEIGHTS_0 (N,4) -> (N,8) u8: 4 ossos (indices novos) + 4 pesos somando 255 (ordem decrescente)"""
    j = np.asarray(j, dtype=np.int64)
    w = np.maximum(np.asarray(w, dtype=np.float64), 0.0)
    o = np.argsort(-w, axis=1, kind="stable")
    j = np.take_along_axis(j, o, axis=1)
    w = np.take_along_axis(w, o, axis=1)
    j = remap[np.clip(j, 0, len(remap) - 1)]
    tot = w.sum(axis=1, keepdims=True)
    ok = tot[:, 0] > 1e-8
    wn = np.where(tot > 1e-8, w / np.maximum(tot, 1e-8), 0.0)
    w8 = np.rint(wn * 255.0).astype(np.int64)
    w8[:, 0] += np.where(ok, 255 - w8.sum(axis=1), 0)
    w8 = np.clip(w8, 0, 255)
    j = np.where(w8 > 0, j, 0)
    out = np.zeros((len(j), 8), dtype=np.uint8)
    out[:, :4] = j
    out[:, 4:] = w8
    return out


def slerp(q0, q1, a):
    d = np.sum(q0 * q1, axis=1, keepdims=True)
    q1 = np.where(d < 0, -q1, q1)
    theta = np.arccos(np.clip(np.abs(d), -1.0, 1.0))
    st = np.sin(theta)
    small = st < 1e-6
    sn = np.where(small, 1.0, st)
    w0 = np.where(small, 1.0 - a, np.sin((1.0 - a) * theta) / sn)
    w1 = np.where(small, a, np.sin(a * theta) / sn)
    q = w0 * q0 + w1 * q1
    return q / np.maximum(np.linalg.norm(q, axis=1, keepdims=True), 1e-12)


def sample_track(times, vals, interp, T, quat):
    """amostra um canal do glTF nos instantes T (STEP / LINEAR / CUBICSPLINE)"""
    n = len(times)
    if interp == "CUBICSPLINE":
        vin, vv, vout = vals[0::3], vals[1::3], vals[2::3]
    else:
        vv = vals
    if n == 1:
        out = np.repeat(vv[:1], len(T), axis=0)
    elif interp == "STEP":
        out = vv[np.clip(np.searchsorted(times, T, side="right") - 1, 0, n - 1)]
    else:
        i = np.clip(np.searchsorted(times, T, side="right") - 1, 0, n - 2)
        t0, t1 = times[i], times[i + 1]
        dt = t1 - t0
        a = np.clip(np.where(dt > 1e-9, (T - t0) / np.where(dt > 1e-9, dt, 1.0), 0.0), 0.0, 1.0)[:, None]
        if interp == "CUBICSPLINE":
            td = dt[:, None]
            h00, h10 = 2 * a**3 - 3 * a**2 + 1, a**3 - 2 * a**2 + a
            h01, h11 = -2 * a**3 + 3 * a**2, a**3 - a**2
            out = h00 * vv[i] + h10 * td * vout[i] + h01 * vv[i + 1] + h11 * td * vin[i + 1]
        elif quat:
            out = slerp(vv[i], vv[i + 1], a)
        else:
            out = vv[i] * (1.0 - a) + vv[i + 1] * a
    if quat:
        out = out / np.maximum(np.linalg.norm(out, axis=1, keepdims=True), 1e-12)
    return out


def build_clips(gltf, blob, skel, fps):
    """cada animation do GLB -> clipe reamostrado a `fps` (quadros uniformes, todos os ossos)"""
    nb = len(skel["bones"])
    node2bone = {n: i for i, n in enumerate(skel["joint_nodes"])}
    clips = []
    for ai, anim in enumerate(gltf.animations or []):
        name = anim.name or f"anim{ai}"
        tracks, dur, skipped = {}, 0.0, 0
        for ch in anim.channels:
            tgt = ch.target
            if tgt.node not in node2bone or tgt.path not in ("translation", "rotation", "scale"):
                skipped += 1
                continue
            smp = anim.samplers[ch.sampler]
            times = read_accessor(gltf, smp.input, blob).astype(np.float64).reshape(-1)
            vals = read_accessor(gltf, smp.output, blob).astype(np.float64)
            tracks[(node2bone[tgt.node], tgt.path)] = (times, vals, smp.interpolation or "LINEAR")
            dur = max(dur, float(times[-1]))
        if not tracks:
            print(f"[aviso] animacao '{name}' nao mexe em nenhum osso do skin: ignorada")
            continue
        if skipped:
            print(f"[aviso] animacao '{name}': {skipped} canal(is) ignorado(s) (node fora do skin / weights)")
        nf = max(2, int(round(dur * fps)) + 1) if dur > 1e-6 else 1
        if nf > 65535:
            raise SystemExit(f"Animacao '{name}' longa demais ({nf} quadros). Use um fps menor.")
        T = np.linspace(0.0, dur, nf) if nf > 1 else np.zeros(1)
        keys = np.zeros((nf, nb), dtype=KEY_DT)
        for b, bone in enumerate(skel["bones"]):
            def get(path, base, quat):
                tr = tracks.get((b, path))
                if tr is None:
                    return np.tile(base, (nf, 1))
                return sample_track(tr[0], tr[1], tr[2], T, quat)
            t = get("translation", bone["t"], False)
            q = get("rotation", bone["q"], True)
            s = get("scale", bone["s"], False)
            for k in range(1, nf):                       # continuidade de sinal: o firmware interpola por nlerp
                if np.dot(q[k], q[k - 1]) < 0:
                    q[k] = -q[k]
            keys["t"][:, b] = t
            keys["q"][:, b] = np.rint(q * 32767.0).astype(np.int16)
            keys["s"][:, b] = s
        clips.append({"name": name, "nf": nf, "dur": dur if nf > 1 else 0.0, "keys": keys})
    return clips


def write_skin_chunk(f, skel, clips):
    bones = skel["bones"]
    f.write(struct.pack("<HHI12f", len(bones), len(clips), 0, *skel["root"][:3, :].reshape(-1)))
    for b, inv in zip(bones, skel["inv"]):
        f.write(struct.pack("<hH3f4f3f12f", b["parent"], 0, *b["t"], *b["q"], *b["s"], *inv[:3, :].reshape(-1)))
    for c in clips:
        f.write(struct.pack("<20sHHf", c["name"].encode("ascii", "replace")[:19], c["nf"], 1, c["dur"]))
    for c in clips:
        f.write(c["keys"].tobytes())


# ---------------------------------------------------------------- conversao
def convert(glb_path, out_path, max_tex, fps=30.0):
    gltf = GLTF2().load(glb_path)
    used = list(gltf.extensionsUsed or [])
    ignored = [e for e in used if e not in ("KHR_texture_transform", "KHR_materials_emissive_strength")]
    if used:
        print(f"[info] extensoes no GLB: {', '.join(used)}")
    if ignored:
        print(f"[aviso] extensoes NAO suportadas pelo conversor (podem mudar a aparencia): {', '.join(ignored)}")
    blob = gltf.binary_blob()
    base_dir = os.path.dirname(os.path.abspath(glb_path))

    instances = collect_instances(gltf)
    skin_idx = next((s for _, _, _, s in instances if s is not None), None)
    skel = build_skeleton(gltf, skin_idx, blob) if skin_idx is not None else None
    if skel is not None and len({s for _, _, _, s in instances if s is not None}) > 1:
        print("[aviso] varios skins no GLB: so o primeiro e usado; as malhas dos outros viram estaticas")

    verts, uvs, idxs, sks, meshes = [], [], [], [], []
    tex_map, tex_dims, tex_wrap, tex_data = {}, [], [], []
    voff = 0
    ioff = 0

    def get_tex(img_idx, tint, keyed, clamp):
        key = (img_idx, tint, keyed, clamp)
        if key in tex_map:
            return tex_map[key]
        if img_idx is None or not gltf.images:
            img = Image.new("RGBA", (1, 1), (*tint, 255))          # cor solida do material
            clamp_f = False                                          # sempre repeat
        else:
            img = Image.open(io.BytesIO(get_image_bytes(gltf, gltf.images[img_idx], blob, base_dir))).convert("RGBA")
            if max_tex > 0 and max(img.size) > max_tex:
                s = max_tex / max(img.size)
                img = img.resize((max(1, int(img.width * s)), max(1, int(img.height * s))), Image.LANCZOS)
            w2, h2 = pow2_round(img.width), pow2_round(img.height)
            if (w2, h2) != img.size:
                img = img.resize((w2, h2), Image.LANCZOS)
            if tint != (255, 255, 255):                              # baseColorFactor
                arr = np.asarray(img, dtype=np.float32)
                arr[..., :3] *= np.array(tint, dtype=np.float32) / 255.0
                img = Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8), "RGBA")
            clamp_f = False                                          # sempre repeat (ignora o sampler do GLB)
        tex_map[key] = len(tex_data)
        tex_dims.append(img.size)
        tex_wrap.append(1 if clamp_f else 0)
        tex_data.append(to_rgb565_be(img, keyed and img_idx is not None))
        return tex_map[key]

    for mesh_i, W, node_name, node_skin in instances:
        mesh = gltf.meshes[mesh_i]
        for p_i, prim in enumerate(mesh.primitives):
            if prim.mode not in (None, 4) or prim.attributes.POSITION is None:
                continue
            pos = read_accessor(gltf, prim.attributes.POSITION, blob).astype(np.float64)
            skinned = (skel is not None and node_skin == skin_idx and
                       prim.attributes.JOINTS_0 is not None and prim.attributes.WEIGHTS_0 is not None)
            if skinned:
                # malha skinned: a matriz do node e IGNORADA (spec do glTF); quem posiciona sao as joints
                pos = pos.astype(np.float32)
                flip = skel["flip"]
                sk = pack_skin(read_accessor(gltf, prim.attributes.JOINTS_0, blob),
                               read_accessor(gltf, prim.attributes.WEIGHTS_0, blob), skel["remap"])
                if getattr(prim.attributes, "JOINTS_1", None) is not None:
                    print(f"[aviso] '{mesh.name}': JOINTS_1/WEIGHTS_1 (mais de 4 influencias) ignorados")
            else:
                pos = (np.c_[pos, np.ones(len(pos))] @ W.T)[:, :3].astype(np.float32)
                flip = np.linalg.det(W[:3, :3]) < 0
                sk = np.zeros((len(pos), 8), dtype=np.uint8)           # peso 0 => vertice estatico
                if skel is not None and node_skin is not None:
                    print(f"[aviso] '{mesh.name}': malha com skin sem JOINTS_0/WEIGHTS_0 (ou de outro skin): estatica")

            img_idx, tc, tint, clamp, keyed, double, uvxf = material_info(gltf, prim)
            uv_acc = getattr(prim.attributes, f"TEXCOORD_{tc}", None)
            if uv_acc is None:
                uv_acc = prim.attributes.TEXCOORD_0
            if uv_acc is not None:
                uv = read_accessor(gltf, uv_acc, blob).astype(np.float32)
            else:
                uv = np.zeros((len(pos), 2), dtype=np.float32)
            if uvxf is not None:                                   # KHR_texture_transform
                uvh = np.c_[uv.astype(np.float64), np.ones(len(uv))] @ uvxf.T
                uv = uvh[:, :2].astype(np.float32)

            if prim.indices is not None:
                ind = read_accessor(gltf, prim.indices, blob).astype(np.uint32).reshape(-1)
            else:
                ind = np.arange(len(pos), dtype=np.uint32)
            ind = ind[: (len(ind) // 3) * 3].reshape(-1, 3)
            if flip:
                ind = ind[:, [0, 2, 1]]
            ind = ind.reshape(-1) + voff

            name = (mesh.name or node_name or f"mesh{mesh_i}")
            if len(mesh.primitives) > 1:
                name += f".{p_i}"
            mflags = (MF_DOUBLE if double else 0) | (MF_KEY if (keyed and img_idx is not None) else 0)
            meshes.append({"start": ioff, "count": len(ind), "flags": mflags,
                           "tex": get_tex(img_idx, tint, keyed, clamp), "name": name})
            verts.append(pos); uvs.append(uv); idxs.append(ind); sks.append(sk)
            voff += len(pos); ioff += len(ind)

    if not meshes:
        raise SystemExit("Nenhuma primitiva de triangulos encontrada.")
    if len(meshes) > 65535:
        raise SystemExit("Sub-malhas demais.")

    V = np.concatenate(verts); U = np.concatenate(uvs); I = np.concatenate(idxs); SK = np.concatenate(sks)
    flags = FLAG_UV | (FLAG_SKIN if skel is not None else 0)
    version = 4 if skel is not None else 3
    clips = build_clips(gltf, blob, skel, fps) if skel is not None else []
    if len(V) > 65535:
        flags |= FLAG_IDX32
        ind_bytes = I.astype("<u4").tobytes()
    else:
        ind_bytes = I.astype("<u2").tobytes()

    with open(out_path, "wb") as f:
        f.write(struct.pack("<4sHHIIHH12x", b"RZOB", version, flags, len(V), len(I), len(meshes), len(tex_data)))
        f.write(V.astype("<f4").tobytes())
        f.write(U.astype("<f4").tobytes())
        f.write(ind_bytes)
        if skel is not None:
            f.write(SK.tobytes())                                   # 8 bytes por vertice
        for m in meshes:
            nm = m["name"].encode("ascii", "replace")[:19]
            f.write(struct.pack("<IIhH20s", m["start"], m["count"], m["tex"], m["flags"], nm))
        for (w, h), wr in zip(tex_dims, tex_wrap):
            f.write(struct.pack("<HHB3x", w, h, wr))
        for d in tex_data:
            f.write(d)
        if skel is not None:
            write_skin_chunk(f, skel, clips)

    print(f"[OK] {out_path}  (v{version})")
    print(f"     vertices: {len(V)} | triangulos: {len(I)//3} | sub-malhas: {len(meshes)} | texturas: {len(tex_data)}")
    if skel is not None:
        nskin = int((SK[:, 4] > 0).sum())
        print(f"     skin: {len(skel['bones'])} ossos | {nskin}/{len(V)} vertices com peso | {len(clips)} clipe(s) a {fps:g} fps")
        for c in clips:
            kb = c["nf"] * len(skel["bones"]) * 32 / 1024
            print(f"       clipe '{c['name']}': {c['dur']:.2f}s, {c['nf']} quadros, {kb:.1f} KB")
    for i, m in enumerate(meshes):
        t = m["tex"]
        ts = f"{tex_dims[t][0]}x{tex_dims[t][1]} {'clamp' if tex_wrap[t] else 'repeat'}"
        fl = ("double " if m["flags"] & MF_DOUBLE else "") + ("key" if m["flags"] & MF_KEY else "")
        print(f"       [{i}] {m['name']:<20} {m['count']//3:>6} tris | tex {t} ({ts}) {fl}")
    print(f"     tamanho: {os.path.getsize(out_path)/1024:.1f} KB")


if __name__ == "__main__":
    if len(sys.argv) >= 3:
        src, dst = sys.argv[1], sys.argv[2]
        mt = int(sys.argv[3]) if len(sys.argv) > 3 else 256
        fps = float(sys.argv[4]) if len(sys.argv) > 4 else 30.0
    else:
        src = input("INPUT (.glb): ").strip().strip('"')
        dst = input("OUTPUT (name.rzobj): ").strip().strip('"')
        t = input("TEXTURA MAX em px (ENTER = 256, 0 = original): ").strip()
        mt = int(t) if t else 256
        t = input("FPS dos clipes de animacao (ENTER = 30; menor = arquivo menor): ").strip()
        fps = float(t) if t else 30.0
    if not dst.lower().endswith(".rzobj"):
        dst += ".rzobj"
    convert(src, dst, mt, max(1.0, fps))
