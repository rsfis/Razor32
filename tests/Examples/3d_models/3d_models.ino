#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>
#include <SPI.h>
#include <FS.h>
#include <SD.h>
#include <LovyanGFX.hpp>
#include <math.h>
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// O Arduino compila em -Os (tamanho). Para o laço de pixels queremos velocidade.
#pragma GCC optimize ("O3")

// HOT: coloca o código quente (raster, projeção, blit) na IRAM. O código normal roda da flash via cache de instruções
// (pequena e DISPUTADA pelos 2 núcleos + blit + FreeRTOS): cada miss custa dezenas de ciclos no meio do laço de pixels.
// Se o linker reclamar de estouro da IRAM ("iram0_0_seg overflowed"), compile com -DHOT_IRAM=0.
#ifndef HOT_IRAM
#define HOT_IRAM 1
#endif
#if HOT_IRAM
#define HOT IRAM_ATTR
#else
#define HOT
#endif
// helpers minúsculos: forçar inline garante que NUNCA viram chamada para uma cópia na flash
#define FORCE_INLINE static inline __attribute__((always_inline))

// ============================ CONFIG ============================
#define DRAW_SKY      1     // 0 desliga o céu
#define RENDER_DIV    2     // resolução interna = tela / RENDER_DIV (o blit assume 2 => 240x160)
#define CHUNK_HR      8     // linhas internas por bloco de DMA
#define INTERNAL_RESERVE (40 * 1024) // RAM interna que nunca é consumida pelos buffers
#define FORCE_PSRAM   0     // 0 = buffers "quentes" (z, cores) tentam RAM interna e caem pra PSRAM sozinhos
                            // 1 = tudo na PSRAM (comportamento antigo, bem mais lento)
#define SPAN_N        16    // a cada N pixels a textura é corrigida em perspectiva (menor = mais exato, mais lento)
#define SPAN_FAR      32    // trecho maior, usado quando 1/w varia pouco no trecho (superfícies distantes/frontais)
static_assert(SPAN_N >= 1 && SPAN_N <= SPAN_FAR, "SPAN_N deve estar entre 1 e SPAN_FAR");
#define RENDER_DUALCORE 1   // 1 = raster dividido entre os 2 núcleos (faixas horizontais); 0 = só o núcleo do loop()
#define RENDER_BANDS  6     // nº de faixas horizontais por quadro (distribuídas dinamicamente entre os núcleos)
                            // (com meshlets a duplicação de projeção por faixa é pequena: mais faixas = melhor balanço)

// ---- NOVO: otimizações (todas desligáveis pra comparar FPS) ----
#ifndef AFFINE_TOL
#define AFFINE_TOL    1.0f  // triângulo com 1/w quase constante usa textura AFIM (zero divisões por span).
#endif
                            // erro máx. de textura ≈ AFFINE_TOL/4 texel. 0 desliga (volta ao 100% perspectiva)
#ifndef ML_MAXV
#define ML_MAXV       64    // vértices por meshlet (índices locais cabem em 1 byte => ≤ 255)
#endif
#ifndef ML_MAXT
#define ML_MAXT       126   // triângulos por meshlet
#endif
#ifndef ML_SORT
#define ML_SORT       1     // ordena meshlets frente->trás por quadro (early-z). 0 = ordem do arquivo
#endif
#ifndef BLIT_SLEEP
#define BLIT_SLEEP    1     // 1 = o blit DORME enquanto o DMA corre (libera o núcleo 0 pro worker de raster)
#endif
#ifndef SPI_WRITE_HZ
#define SPI_WRITE_HZ  80000000
#endif
#ifndef FAST_RCP
#define FAST_RCP      1     // 1 = recíproco por Newton em vez de divisão (o ESP32-S3 não tem divisão de float em hardware)
#endif
// ---- filtro de textura estilo Nintendo 64 (bilinear de 3 pontos) ----
#ifndef TEX_FILTER
#define TEX_FILTER    1     // 1 = filtra textura AMPLIADA (o "blurzinho" do Ocarina of Time). 0 = amostragem por ponto (original, bit a bit)
#endif
#ifndef TEXFILTER_MAXSTEP
#define TEXFILTER_MAXSTEP 1.0f  // só filtra onde a textura anda <= este nº de texels por pixel (1.0 = ampliada / 1:1). Menor = mais barato
#endif
#ifndef TEX_BYTESWAPPED
#define TEX_BYTESWAPPED 1   // texturas em RGB565 com os bytes trocados (formato do painel, como o buffer de cor). Cores estranhas => 0
#endif
#define FILT_LIM ((int32_t)(TEXFILTER_MAXSTEP * 65536.0f))
#ifndef COLORB_INTERNAL
#define COLORB_INTERNAL 1   // 1 = 2º buffer de cor na RAM interna (padrão). 0 = PSRAM: libera ~75 KB internos para promoteTextures()
#endif                      // (escrita de cor é sequencial = cache da PSRAM perdoa; textura é acesso aleatório = não perdoa). Teste os dois!
#ifndef PROMOTE_RESERVE
#define PROMOTE_RESERVE (56 * 1024)   // RAM interna que SEMPRE sobra depois de promover texturas/arrays por quadro
#endif
static_assert(ML_MAXV >= 3 && ML_MAXV <= 255, "ML_MAXV: índices locais são uint8_t");
static_assert(ML_MAXT >= 1 && ML_MAXT <= 255, "ML_MAXT deve caber em uint8_t");

// ---- skinning / animação (.rzobj v4) ----
#ifndef MAX_BONES
#define MAX_BONES     48    // ossos por modelo (tem que bater com MAX_BONES do glb_to_rzobj.py)
#endif
#ifndef MAX_CLIPS
#define MAX_CLIPS     32    // clipes de animação por modelo
#endif

// ---- cenário Kokiri (kokiri.rzobj é uma CENA inteira, não um skybox) ----
// ---- câmera orbital (independente de FPS: velocidades em unidades POR SEGUNDO, multiplicadas por dt) ----
#define ORBIT_SPEED   1.0f    // rad/s  (1.0 => volta completa em ~6,28 s, em qualquer FPS)
#define ORBIT_RADIUS  3.0f
#define DT_MAX        0.100f  // s: trava o dt p/ um engasgo (carga, 1º quadro) não "teleportar" a câmera
#define DT_MIN        0.0001f

#define KOKIRI_SCALE  0.5f
#define KOKIRI_CX     (0.0f)
#define KOKIRI_CZ     (0.0f)
#define KOKIRI_GROUND_Y (-1.4f)

// ---- SD card ----
#define SD_CS         4
#define SD_SCK        14
#define SD_MOSI       13
#define SD_MISO       12
#define SD_FREQ       25000000
#define TFT_CS_PIN    10

#define FILE_GANON    "/ganondorf.rzobj"
#define FILE_LINK     "/link_anim.rzobj"
#define FILE_SKY      "/field.rzobj"

// Tipos ANTES de qualquer função: o Arduino insere protótipos antes da 1ª função do arquivo.
struct Vec3 { float x,y,z; };
struct Model;
struct Mat4 { float m[4][4]; };

enum { CULL_NONE = 0, CULL_POS_AREA = 1, CULL_NEG_AREA = -1 };

struct SV { float x, y, iw, u, v; };   // iw = 1/w (profundidade de view invertida); iw < 0 => atrás do near

struct Target {
  int W, H;
  int y0, y1;          // faixa de linhas [y0,y1) que ESTE chamador pode escrever
  uint16_t *color;     // RGB565, bytes já trocados (formato do painel)
  uint16_t *depth;     // 16 bits de 1/w escalado; 0 = infinito
  SV *sv;              // vértices projetados do meshlet atual (ML_MAXV entradas, RAM interna)
  uint8_t *oc;         // outcode de frustum de cada vértice (bit0 L, 1 R, 2 B, 3 T, 4 near)
  float nearW;
  float depthScale;    // 60000 * near  (1/w máx = 1/near)
};

struct __attribute__((packed)) RzHeader {
  char     magic[4];
  uint16_t version;
  uint16_t flags;
  uint32_t vertCount;
  uint32_t indexCount;
  uint16_t meshCount;
  uint16_t texCount;
  uint8_t  reserved[12];
};
struct __attribute__((packed)) RzMesh {
  uint32_t indexStart;
  uint32_t indexCount;
  int16_t  texIndex;
  uint16_t flags;
  char     name[20];
};
struct __attribute__((packed)) RzTexEntry {
  uint16_t w, h;
  uint8_t  wrap;
  uint8_t  reserved[3];
};
#define RZ_UV     1
#define RZ_IDX32  4
#define RZM_DOUBLE 1
#define RZM_KEY    2
#define RZ_SKIN    8

// Estruturas do bloco de skin (v4). Layout natural (sem packed): os tamanhos batem com o arquivo e o acesso a float é alinhado.
struct RzSkinHdr   { uint16_t boneCount, clipCount; uint32_t reserved; float root[12]; };                      // 56 B
struct SkBone      { int16_t parent; uint16_t pad; float t[3]; float q[4]; float s[3]; float inv[12]; };        // 92 B
struct RzClipEntry { char name[20]; uint16_t nFrames; uint16_t flags; float duration; };                       // 28 B
struct AKey        { float t[3]; int16_t q[4]; float s[3]; };                                                  // 32 B
static_assert(sizeof(RzSkinHdr) == 56 && sizeof(SkBone) == 92 && sizeof(RzClipEntry) == 28 && sizeof(AKey) == 32,
              "layout do bloco de skin difere do .rzobj v4");
struct AnimClip    { char name[20]; uint16_t nFrames; uint16_t flags; float duration; AKey* keys; };

// Meshlet = até ML_MAXV vértices / ML_MAXT triângulos contíguos de UMA sub-malha, com AABB própria.
// Vértices (pos+uv) ficam num pool contíguo e os triângulos usam índices LOCAIS de 1 byte:
// culling fino por frustum e por faixa de linhas, e leitura sequencial da PSRAM.
struct Vtx { float x, y, z, u, v; };
struct Meshlet {
  float    c[3], h[3];     // centro e meia-extensão da AABB (espaço do modelo)
  uint32_t vOff;           // 1º vértice no vpool
  uint32_t tOff;           // 1º triângulo no tpool (3 bytes por triângulo)
  uint8_t  nv, nt;
  int16_t  tex;
  uint16_t flags;
};
struct TexData { uint16_t* data; int w, h; bool repeat; };

// contexto do construtor de meshlets (tipo declarado aqui: o Arduino gera protótipos antes da 1ª função)
struct MeshletBuild {
  const float* pos; const float* uv; const uint32_t* idx; uint32_t V;
  uint16_t* map;                      // id global -> índice local (0xFFFF = fora do meshlet atual)
  Meshlet* ml; Vtx* vp; uint8_t* tp;  // nullptr na passada de contagem
  const uint8_t* sk; uint8_t* sp;     // pesos por vértice (8 B: 4 ossos + 4 pesos) de entrada / saída (sp só na 2ª passada)
  uint32_t nMl, nV, nT;               // totais emitidos
  uint32_t used[ML_MAXV];             // ids globais do meshlet em construção
  uint8_t  ltri[ML_MAXT * 3];         // triângulos locais do meshlet em construção
  uint32_t nu, nt;
  int tex; uint16_t flags;
};

class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ST7796 _panel_instance;
  lgfx::Bus_SPI _bus_instance;
  lgfx::Light_PWM _light_instance;
public:
  LGFX(void){
    { auto cfg = _bus_instance.config();
      cfg.spi_host = SPI2_HOST; cfg.spi_mode = 0;
      cfg.freq_write = SPI_WRITE_HZ; cfg.freq_read = 20000000;
      cfg.spi_3wire = false; cfg.use_lock = true; cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk = 14; cfg.pin_mosi = 13; cfg.pin_miso = 12; cfg.pin_dc = 9;
      _bus_instance.config(cfg); _panel_instance.setBus(&_bus_instance);
    }
    { auto cfg = _panel_instance.config();
      cfg.pin_cs = 10; cfg.pin_rst = 11; cfg.pin_busy = -1;
      cfg.memory_width = 320; cfg.memory_height = 480;
      cfg.panel_width = 320; cfg.panel_height = 480;
      cfg.offset_x = 0; cfg.offset_y = 0;
      cfg.readable = false; cfg.invert = true; cfg.rgb_order = false;
      cfg.dlen_16bit = false; cfg.bus_shared = true;
      _panel_instance.config(cfg);
    }
    { auto cfg = _light_instance.config();
      cfg.pin_bl = 38; cfg.invert = false; cfg.freq = 10000; cfg.pwm_channel = 7;
      _light_instance.config(cfg); _panel_instance.setLight(&_light_instance);
    }
    setPanel(&_panel_instance);
  }
};
static LGFX tft;
static SPIClass sdSPI(HSPI);

static char   gErr[768];
static size_t gErrLen = 0;
static void errf(const char* fmt, ...) {
  char tmp[160];
  va_list ap; va_start(ap, fmt);
  vsnprintf(tmp, sizeof(tmp), fmt, ap);
  va_end(ap);
  Serial.printf("[ERRO] %s\n", tmp);
  size_t n = strlen(tmp);
  if (gErrLen + n + 2 < sizeof(gErr)) {
    memcpy(gErr + gErrLen, tmp, n);
    gErrLen += n;
    gErr[gErrLen++] = '\n';
    gErr[gErrLen] = 0;
  }
}

// Alocador. hot=true  -> buffers acessados pixel a pixel (z, cor): tenta RAM interna primeiro.
//           hot=false -> dados do SD (vértices, texturas): sempre PSRAM, pra não gastar a RAM interna.
static void* allocFast(size_t bytes, const char* name, bool hot) {
  void* p = nullptr;
  const char* where = "RAM interna";
#if !FORCE_PSRAM
  if (hot && heap_caps_get_free_size(MALLOC_CAP_INTERNAL) > bytes + INTERNAL_RESERVE)
    p = heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
#endif
  if (!p) { p = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT); where = "PSRAM"; }
  Serial.printf("  %-14s %7u B -> %s\n", name, (unsigned)bytes, p ? where : "FALHOU!");
  if (!p) errf("sem memoria: %s (%u B)", name, (unsigned)bytes);
  return p;
}

// buffer temporário (só durante o load): libera sozinho em qualquer saída da função
struct Scoped {
  void* p = nullptr;
  ~Scoped() { release(); }
  void release() { if (p) { heap_caps_free(p); p = nullptr; } }
};

static bool readAll(File &f, void* dst, size_t n) {
  uint8_t* p = (uint8_t*)dst;
  while (n) {
    size_t chunk = n > 16384 ? 16384 : n;
    int r = f.read(p, chunk);
    if (r <= 0) return false;
    p += r; n -= (size_t)r;
  }
  return true;
}

// ---------------- 3D math ----------------

inline Mat4 matIdentity(){ Mat4 I={}; for(int i=0;i<4;i++) for(int j=0;j<4;j++) I.m[i][j]=(i==j)?1.0f:0.0f; return I; }
inline Mat4 matTranslation(float tx,float ty,float tz){ Mat4 M=matIdentity(); M.m[0][3]=tx; M.m[1][3]=ty; M.m[2][3]=tz; return M; }
inline Mat4 matScale(float sx,float sy,float sz){ Mat4 M={}; M.m[0][0]=sx; M.m[1][1]=sy; M.m[2][2]=sz; M.m[3][3]=1.0f; return M; }
inline Mat4 matRotX(float a){ Mat4 M=matIdentity(); float c=cosf(a), s=sinf(a); M.m[1][1]=c; M.m[1][2]=-s; M.m[2][1]=s; M.m[2][2]=c; return M; }
inline Mat4 matRotY(float a){ Mat4 M=matIdentity(); float c=cosf(a), s=sinf(a); M.m[0][0]=c; M.m[0][2]=s; M.m[2][0]=-s; M.m[2][2]=c; return M; }
inline Mat4 matRotZ(float a){ Mat4 M=matIdentity(); float c=cosf(a), s=sinf(a); M.m[0][0]=c; M.m[0][1]=-s; M.m[1][0]=s; M.m[1][1]=c; return M; }
inline Mat4 matMul(const Mat4 &A,const Mat4 &B){ Mat4 R={}; for(int i=0;i<4;i++) for(int j=0;j<4;j++){ float s=0; for(int k=0;k<4;k++) s+=A.m[i][k]*B.m[k][j]; R.m[i][j]=s; } return R; }

// ---- helpers de skinning (matrizes afins 3x4, linha-maior, última linha implícita 0 0 0 1) ----
static float gWorld[MAX_BONES][12];   // scratch do cálculo de pose: só o fluxo do loop() usa (updateAnim roda fora do quadro)

// O = A * B (O não pode alias A nem B)
FORCE_INLINE void mul34(const float* A, const float* B, float* O) {
  for (int r = 0; r < 3; ++r) {
    const float a0 = A[4*r], a1 = A[4*r+1], a2 = A[4*r+2], a3 = A[4*r+3];
    O[4*r  ] = a0*B[0] + a1*B[4] + a2*B[8];
    O[4*r+1] = a0*B[1] + a1*B[5] + a2*B[9];
    O[4*r+2] = a0*B[2] + a1*B[6] + a2*B[10];
    O[4*r+3] = a0*B[3] + a1*B[7] + a2*B[11] + a3;
  }
}
FORCE_INLINE void qnorm4(float* q) {
  const float l = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
  if (l < 1e-12f) { q[0] = q[1] = q[2] = 0.0f; q[3] = 1.0f; return; }
  const float k = 1.0f / sqrtf(l);
  q[0] *= k; q[1] *= k; q[2] *= k; q[3] *= k;
}

Mat4 lookAt(const Vec3 &eye,const Vec3 &center,const Vec3 &up){
  Vec3 f={center.x-eye.x, center.y-eye.y, center.z-eye.z};
  float fl=sqrtf(f.x*f.x+f.y*f.y+f.z*f.z); if(fl==0) fl=1; f.x/=fl; f.y/=fl; f.z/=fl;
  Vec3 upn=up; float ul=sqrtf(upn.x*upn.x+upn.y*upn.y+upn.z*upn.z); if(ul==0) ul=1; upn.x/=ul; upn.y/=ul; upn.z/=ul;
  Vec3 s={ f.y*upn.z - f.z*upn.y, f.z*upn.x - f.x*upn.z, f.x*upn.y - f.y*upn.x };
  float sl = sqrtf(s.x*s.x+s.y*s.y+s.z*s.z); if(sl==0) sl=1; s.x/=sl; s.y/=sl; s.z/=sl;
  Vec3 u={ s.y*f.z - s.z*f.y, s.z*f.x - s.x*f.z, s.x*f.y - s.y*f.x };
  Mat4 M = matIdentity();
  M.m[0][0]=s.x; M.m[0][1]=s.y; M.m[0][2]=s.z; M.m[0][3]=-(s.x*eye.x + s.y*eye.y + s.z*eye.z);
  M.m[1][0]=u.x; M.m[1][1]=u.y; M.m[1][2]=u.z; M.m[1][3]=-(u.x*eye.x + u.y*eye.y + u.z*eye.z);
  M.m[2][0]=-f.x; M.m[2][1]=-f.y; M.m[2][2]=-f.z; M.m[2][3]=(f.x*eye.x + f.y*eye.y + f.z*eye.z);
  M.m[3][0]=0; M.m[3][1]=0; M.m[3][2]=0; M.m[3][3]=1;
  return M;
}

Mat4 perspective(float fovY,float aspect,float zn,float zf){
  float f = 1.0f / tanf(fovY * 0.5f);
  Mat4 P={};
  P.m[0][0] = f / aspect;
  P.m[1][1] = f;
  P.m[2][2] = (zf + zn) / (zn - zf);
  P.m[2][3] = (2.0f * zf * zn) / (zn - zf);
  P.m[3][2] = -1.0f;
  return P;
}

// ==== RASTER BEGIN ==========================================================
//  - z-buffer guarda 1/w (maior = mais perto), limpo com 0.
//  - textura com correção de perspectiva a cada SPAN_N pixels (trechos encadeados: 1 ponto exato por trecho).
//  - NOVO: triângulos com 1/w quase constante usam interpolação AFIM de u,v (sem nenhuma divisão por span).
//  - wrap REPEAT (potência de 2, índice por shift) ou CLAMP; color-key 0x0000.
//  - triângulos que cruzam o near são recortados.
//  - todo desenho respeita a faixa de linhas [T.y0, T.y1), permitindo 2 núcleos no mesmo quadro.

#define ZFRAC 12

static float INV_N[SPAN_FAR + 1];   // 1/n; preenchida em setup() por initInvTable()
static void initInvTable() {
  INV_N[0] = 0.0f;
  for (int i = 1; i <= SPAN_FAR; ++i) INV_N[i] = 1.0f / (float)i;
}

FORCE_INLINE int ceilToInt(float f) { int i = (int)f; return (f > (float)i) ? i + 1 : i; }
// floor sem chamar a libm (válido p/ |f| < 2^30; aqui os valores são coordenadas de textura)
FORCE_INLINE float floorFast(float f) { const int i = (int)f; return (float)(i - (f < (float)i)); }
// 1/x sem divisão: chute inicial por truque de bits + 2 (rcpf) ou 3 (rcpf3) iterações de Newton.
// erro relativo máx.: 6.6e-6 (2 it) / 1.5e-7 (3 it). Só p/ x != 0 e finito (todos os usos abaixo garantem isso).
FORCE_INLINE float rcpf(float x) {
#if FAST_RCP
  union { float f; uint32_t i; } u; u.f = x; u.i = 0x7EF311C7u - u.i;
  float y = u.f;
  y = y * (2.0f - x * y);
  y = y * (2.0f - x * y);
  return y;
#else
  return 1.0f / x;
#endif
}
FORCE_INLINE float rcpf3(float x) {
#if FAST_RCP
  union { float f; uint32_t i; } u; u.f = x; u.i = 0x7EF311C7u - u.i;
  float y = u.f;
  y = y * (2.0f - x * y);
  y = y * (2.0f - x * y);
  y = y * (2.0f - x * y);
  return y;
#else
  return 1.0f / x;
#endif
}
// 1/n EXATO p/ n potência de 2 (monta o expoente direto)
FORCE_INLINE float pow2Inv(int n) { union { uint32_t i; float f; } u; u.i = (uint32_t)(127 - __builtin_ctz((unsigned)n)) << 23; return u.f; }
FORCE_INLINE float min3f(float a, float b, float c) { float m = a < b ? a : b; return m < c ? m : c; }
FORCE_INLINE float max3f(float a, float b, float c) { float m = a > b ? a : b; return m > c ? m : c; }

template<bool REP>
FORCE_INLINE uint16_t texel(const uint16_t* __restrict tex, int tx, int ty, int tw, int tws, int twm, int thm) {
  if (REP) {                                   // tw é potência de 2: índice por shift, sem multiplicação
    tx &= twm; ty &= thm;
    return tex[(ty << tws) + tx];
  }
  if ((unsigned)tx > (unsigned)twm) tx = (tx < 0) ? 0 : twm;
  if ((unsigned)ty > (unsigned)thm) ty = (ty < 0) ? 0 : thm;
  return tex[ty * tw + tx];
}

// ---- bilinear de 3 pontos do N64 ----
// Em vez de 4 texels, usa os 3 do TRIÂNGULO da célula onde a amostra cai (superior-esquerdo se fu+fv<1, senão inferior-direito):
//   fu+fv <  1:  t00 + fu*(t10-t00) + fv*(t01-t00)      pesos (1-fu-fv, fu, fv)
//   fu+fv >= 1:  t11 + (1-fu)*(t01-t11) + (1-fv)*(t10-t11)  pesos (fu+fv-1, 1-fu, 1-fv)
// É isso que dá o aspecto "borrado com diagonais" do Ocarina. A amostra é no CENTRO do texel (u-0,5), como no N64.
// Pesos em 5 bits (somam 32) e mistura dos 3 canais de uma vez: cada canal vira um campo de 32 bits com folga p/ o produto.
FORCE_INLINE uint32_t ex565(uint32_t s) {
#if TEX_BYTESWAPPED
  s = ((s << 8) | (s >> 8)) & 0xFFFFu;       // texel do painel (bytes trocados) -> RGB565 nativo
#endif
  return (s | (s << 16)) & 0x07E0F81Fu;      // B em 0..4, R em 11..15, G em 21..26
}
template<bool REP>
FORCE_INLINE uint16_t texel3(const uint16_t* __restrict tex, int32_t u, int32_t v, int tw, int tws, int twm, int thm) {
  u -= 0x8000; v -= 0x8000;
  const int x0 = u >> 16, y0 = v >> 16;
  const int fu = (u >> 11) & 31, fv = (v >> 11) & 31;
  int cx0, cx1, r0, r1;
  if (REP) {
    cx0 = x0 & twm; cx1 = (x0 + 1) & twm;
    r0 = (y0 & thm) << tws; r1 = ((y0 + 1) & thm) << tws;
  } else {
    cx0 = x0 < 0 ? 0 : (x0 > twm ? twm : x0);  cx1 = x0 + 1 < 0 ? 0 : (x0 + 1 > twm ? twm : x0 + 1);
    const int ya = y0 < 0 ? 0 : (y0 > thm ? thm : y0), yb = y0 + 1 < 0 ? 0 : (y0 + 1 > thm ? thm : y0 + 1);
    r0 = ya * tw; r1 = yb * tw;
  }
  uint32_t e0, e1, e2, w0, w1, w2;
  if (fu + fv < 32) {
    e0 = ex565(tex[r0 + cx0]); e1 = ex565(tex[r0 + cx1]); e2 = ex565(tex[r1 + cx0]);
    w0 = 32 - fu - fv; w1 = fu; w2 = fv;
  } else {
    e0 = ex565(tex[r1 + cx1]); e1 = ex565(tex[r1 + cx0]); e2 = ex565(tex[r0 + cx1]);
    w0 = fu + fv - 32; w1 = 32 - fu; w2 = 32 - fv;
  }
  const uint32_t r = ((e0 * w0 + e1 * w1 + e2 * w2) >> 5) & 0x07E0F81Fu;
  const uint32_t n = (r | (r >> 16)) & 0xFFFFu;
#if TEX_BYTESWAPPED
  return (uint16_t)(((n << 8) | (n >> 8)) & 0xFFFFu);
#else
  return (uint16_t)n;
#endif
}

// AFF: uw/vw/duw/dvw já vêm em TEXELS (u,v lineares na tela); senão são u/w, v/w (perspectiva).
template<bool TEX, bool AFF, bool REP, bool KEY, bool FILT>
FORCE_INLINE void fillSpan(uint16_t* __restrict crow, uint16_t* __restrict zrow, int xs, int xe,
                     float iw, float diw, float uw, float duw, float vw, float dvw,
                     const uint16_t* __restrict tex, int tw, int th, float dScale)
{
  const float zf = dScale * (float)(1 << ZFRAC);
  int32_t z = (int32_t)(iw * zf);
  const int32_t dz = (int32_t)(diw * zf);

  if (!TEX) {
    for (int x = xs; x <= xe; ++x) {
      const int zi = z >> ZFRAC;
      if (zi >= (int)zrow[x]) { crow[x] = 0xFFFF; zrow[x] = (uint16_t)zi; }
      z += dz;
    }
    return;
  }

  const int twm = tw - 1, thm = th - 1;
  const int tws = REP ? __builtin_ctz((unsigned)tw) : 0;

  if (AFF) {
    int32_t u = (int32_t)(uw * 65536.0f), v = (int32_t)(vw * 65536.0f);
    const int32_t du = (int32_t)(duw * 65536.0f), dv = (int32_t)(dvw * 65536.0f);
    if (FILT && (du < 0 ? -du : du) <= FILT_LIM && (dv < 0 ? -dv : dv) <= FILT_LIM) {   // textura ampliada: filtra
      for (int x = xs; x <= xe; ++x) {
        const int zi = z >> ZFRAC;
        if (zi >= (int)zrow[x]) { crow[x] = texel3<REP>(tex, u, v, tw, tws, twm, thm); zrow[x] = (uint16_t)zi; }
        z += dz; u += du; v += dv;
      }
      return;
    }
    for (int x = xs; x <= xe; ++x) {
      const int zi = z >> ZFRAC;
      if (zi >= (int)zrow[x]) {
        const uint16_t c = texel<REP>(tex, u >> 16, v >> 16, tw, tws, twm, thm);
        if (!(KEY && c == 0)) { crow[x] = c; zrow[x] = (uint16_t)zi; }
      }
      z += dz; u += du; v += dv;
    }
    return;
  }

  const float ftw = (float)tw, fth = (float)th;
  const float ftwi = REP ? pow2Inv(tw) : 0.0f, fthi = REP ? pow2Inv(th) : 0.0f;
  // ponto exato (u,v) no 1º pixel; depois cada trecho termina num ponto exato que vira o início do próximo
  float rw = rcpf(iw > 1e-7f ? iw : 1e-7f);
  float ucR = uw * rw, vcR = vw * rw;
  int x = xs;
  while (x <= xe) {
    // trecho adaptativo: se 1/w muda menos de ~6% em SPAN_FAR pixels, interpolar u,v linearmente é quase exato
    int n = xe - x + 1;
    const int lim = (fabsf(diw) * (float)SPAN_FAR < 0.06f * iw) ? SPAN_FAR : SPAN_N;
    int steps;
    if (n <= lim) steps = n - 1;        // último trecho: termina no ÚLTIMO pixel (sempre dentro do triângulo)
    else { n = lim; steps = lim; }      // intermediário: termina no 1º pixel do próximo trecho
    const float iwE = iw + diw * (float)steps;
    rw = rcpf(iwE > 1e-7f ? iwE : 1e-7f);
    const float unR = (uw + duw * (float)steps) * rw, vnR = (vw + dvw * (float)steps) * rw;
    float uc = ucR, vc = vcR, un = unR, vn = vnR;
    if (REP) {   // traz as coordenadas p/ perto da origem: o ponto fixo 16.16 não estoura
      const float fu = floorFast(uc * ftwi) * ftw, fv = floorFast(vc * fthi) * fth;
      uc -= fu; un -= fu; vc -= fv; vn -= fv;
    }
    const float inv = INV_N[steps] * 65536.0f;
    int32_t u = (int32_t)(uc * 65536.0f), v = (int32_t)(vc * 65536.0f);
    const int32_t du = (int32_t)((un - uc) * inv), dv = (int32_t)((vn - vc) * inv);

    if (FILT && (du < 0 ? -du : du) <= FILT_LIM && (dv < 0 ? -dv : dv) <= FILT_LIM) {   // este trecho está ampliado: filtra
      for (int i = 0; i < n; ++i, ++x) {
        const int zi = z >> ZFRAC;
        if (zi >= (int)zrow[x]) { crow[x] = texel3<REP>(tex, u, v, tw, tws, twm, thm); zrow[x] = (uint16_t)zi; }
        z += dz; u += du; v += dv;
      }
    } else {
      for (int i = 0; i < n; ++i, ++x) {
        const int zi = z >> ZFRAC;
        if (zi >= (int)zrow[x]) {
          const uint16_t c = texel<REP>(tex, u >> 16, v >> 16, tw, tws, twm, thm);
          if (!(KEY && c == 0)) { crow[x] = c; zrow[x] = (uint16_t)zi; }
        }
        z += dz; u += du; v += dv;
      }
    }
    iw = iwE; uw += duw * (float)n; vw += dvw * (float)n;   // (só usados se houver próximo trecho, onde steps == n)
    ucR = unR; vcR = vnR;
  }
}

typedef void (*SpanFn)(uint16_t*, uint16_t*, int, int, float, float, float, float, float, float,
                       const uint16_t*, int, int, float);
// índice: bit3 TEX | bit2 AFIM | bit1 REPEAT | bit0 KEY
#define SPAN_THUNK(F)                                                                                        \
  static void HOT spanThunk##F(uint16_t* c, uint16_t* z, int xs, int xe,                                     \
      float iw, float diw, float uw, float duw, float vw, float dvw,                                         \
      const uint16_t* t, int tw, int th, float ds)                                                           \
  { fillSpan<((F)&8)!=0,((F)&4)!=0,((F)&2)!=0,((F)&1)!=0,((F)&16)!=0>(c, z, xs, xe, iw, diw, uw, duw, vw, dvw, t, tw, th, ds); }
SPAN_THUNK(0)  SPAN_THUNK(8)  SPAN_THUNK(9)  SPAN_THUNK(10) SPAN_THUNK(11)
SPAN_THUNK(12) SPAN_THUNK(13) SPAN_THUNK(14) SPAN_THUNK(15)
#if TEX_FILTER
SPAN_THUNK(24) SPAN_THUNK(26) SPAN_THUNK(28) SPAN_THUNK(30)   // com filtro: só textura (8), sem color-key (bit0 = 0)
#endif

// sem textura (bit3 = 0) AFIM/REPEAT/KEY são ignorados e rasterTri só gera fi == 0: as entradas 1..7 reaproveitam a variante 0
// índice: bit4 FILTRO | bit3 TEX | bit2 AFIM | bit1 REPEAT | bit0 KEY.  Com color-key não filtra (misturaria a cor-chave preta nas bordas)
static const SpanFn SPAN_FNS[32] = {
  spanThunk0,  spanThunk0,  spanThunk0,  spanThunk0,  spanThunk0,  spanThunk0,  spanThunk0,  spanThunk0,
  spanThunk8,  spanThunk9,  spanThunk10, spanThunk11, spanThunk12, spanThunk13, spanThunk14, spanThunk15,
  spanThunk0,  spanThunk0,  spanThunk0,  spanThunk0,  spanThunk0,  spanThunk0,  spanThunk0,  spanThunk0,
#if TEX_FILTER
  spanThunk24, spanThunk9,  spanThunk26, spanThunk11, spanThunk28, spanThunk13, spanThunk30, spanThunk15
#else
  spanThunk8,  spanThunk9,  spanThunk10, spanThunk11, spanThunk12, spanThunk13, spanThunk14, spanThunk15
#endif
};

// fi: bit3 TEX | bit1 REPEAT | bit0 KEY  (o bit2 AFIM é decidido aqui, por triângulo)
static void HOT rasterTri(const Target &T, int fi, int cull, const uint16_t* tdata, int tw, int th,
                      const SV &a, const SV &b, const SV &c)
{
  const int W = T.W;
  const float fW = (float)W;
  const float x0 = a.x, y0 = a.y, x1 = b.x, y1 = b.y, x2 = c.x, y2 = c.y;

  float minx = x0 < x1 ? x0 : x1; if (x2 < minx) minx = x2;
  float maxx = x0 > x1 ? x0 : x1; if (x2 > maxx) maxx = x2;
  float miny = y0 < y1 ? y0 : y1; if (y2 < miny) miny = y2;
  float maxy = y0 > y1 ? y0 : y1; if (y2 > maxy) maxy = y2;
  if (maxx < 0.0f || minx >= fW || maxy < (float)T.y0 || miny >= (float)T.y1) return;

  const float ax = x1 - x0, ay = y1 - y0, bx = x2 - x0, by = y2 - y0;
  const float area = ax*by - bx*ay;
  if (area > -1e-3f && area < 1e-3f) return;
  if (cull == CULL_POS_AREA && area > 0.0f) return;
  if (cull == CULL_NEG_AREA && area < 0.0f) return;
  // Ordena os vértices por y e acha as linhas cobertas ANTES de montar os gradientes:
  // triângulos que não cobrem nenhum centro de pixel (muito comum a 240x160) saem aqui, baratos.
  float xt=x0, yt=y0, xm=x1, ym=y1, xb=x2, yb=y2;
  if (yt > ym) { std::swap(xt,xm); std::swap(yt,ym); }
  if (ym > yb) { std::swap(xm,xb); std::swap(ym,yb); }
  if (yt > ym) { std::swap(xt,xm); std::swap(yt,ym); }

  int yStart = ceilToInt(yt - 0.5f);
  int yEnd   = ceilToInt(yb - 0.5f) - 1;
  const int nRows = yEnd - yStart + 1;          // linhas do triângulo INTEIRO (independe da faixa => mesma decisão em toda faixa)
  if (yStart < T.y0) yStart = T.y0;
  if (yEnd > T.y1 - 1) yEnd = T.y1 - 1;
  if (yEnd < yStart) return;

  const float invA = rcpf3(area);

  // a.iw/b.iw/c.iw = 1/w já calculado na projeção
  const float i0 = a.iw, i1 = b.iw, i2 = c.iw;
  const float di1 = i1 - i0, di2 = i2 - i0;
  const float diwdx = (di1*by - di2*ay) * invA, diwdy = (di2*ax - di1*bx) * invA;

  float uw0 = 0, vw0 = 0, duwdx = 0, duwdy = 0, dvwdx = 0, dvwdy = 0;
  if (fi & 8) {
    const float su = (float)tw, sv = (float)th;
    bool aff = false;
    if (AFFINE_TOL > 0.0f && nRows >= 3) {   // (constante: ramo removido se AFFINE_TOL=0) triângulo de 1-2 linhas não paga o teste
      // Se 1/w quase não varia no triângulo, u,v são (quase) lineares na tela: o erro da versão afim é
      // ≈ (extensão da textura em texels) * (variação relativa de 1/w) / 4. Limitamos isso a AFFINE_TOL/4 texel.
      const float U0 = a.u * su, V0 = a.v * sv, U1 = b.u * su, V1 = b.v * sv, U2 = c.u * su, V2 = c.v * sv;
      const float imin = min3f(i0, i1, i2), imax = max3f(i0, i1, i2);
      const float eu = max3f(fabsf(U1-U0), fabsf(U2-U0), fabsf(U2-U1));
      const float ev = max3f(fabsf(V1-V0), fabsf(V2-V0), fabsf(V2-V1));
      const float ext = eu > ev ? eu : ev;
      if ((imax - imin) * ext <= AFFINE_TOL * imin && ext < 8000.0f) {
        float oU = 0.0f, oV = 0.0f;
        bool ok = true;
        if (fi & 2) {     // REPEAT (tw,th potências de 2): move o triângulo p/ perto da origem; o wrap é por máscara
          oU = floorFast(min3f(U0, U1, U2) * pow2Inv(tw)) * su;
          oV = floorFast(min3f(V0, V1, V2) * pow2Inv(th)) * sv;
        } else {          // CLAMP: coordenadas absolutas precisam caber no ponto fixo 16.16
          ok = max3f(fabsf(U0), fabsf(U1), fabsf(U2)) < 30000.0f && max3f(fabsf(V0), fabsf(V1), fabsf(V2)) < 30000.0f;
        }
        if (ok) {
          const float dU1 = U1-U0, dU2 = U2-U0, dV1 = V1-V0, dV2 = V2-V0;
          duwdx = (dU1*by - dU2*ay) * invA;  duwdy = (dU2*ax - dU1*bx) * invA;
          dvwdx = (dV1*by - dV2*ay) * invA;  dvwdy = (dV2*ax - dV1*bx) * invA;
          uw0 = U0 - oU; vw0 = V0 - oV;
          aff = true;
        }
      }
    }
    if (aff) {
      fi |= 4;
    } else {
      uw0 = a.u * su * i0; vw0 = a.v * sv * i0;
      const float uw1 = b.u * su * i1, vw1 = b.v * sv * i1;
      const float uw2 = c.u * su * i2, vw2 = c.v * sv * i2;
      duwdx = ((uw1-uw0)*by - (uw2-uw0)*ay) * invA;  duwdy = ((uw2-uw0)*ax - (uw1-uw0)*bx) * invA;
      dvwdx = ((vw1-vw0)*by - (vw2-vw0)*ay) * invA;  dvwdy = ((vw2-vw0)*ax - (vw1-vw0)*bx) * invA;
    }
  }

  const float slopeL  = (xb - xt) * rcpf3(yb - yt);
  const float slopeTM = (ym > yt) ? (xm - xt) * rcpf3(ym - yt) : 0.0f;
  const float slopeMB = (yb > ym) ? (xb - xm) * rcpf3(yb - ym) : 0.0f;
  const SpanFn fn = SPAN_FNS[fi & 31];

  for (int y = yStart; y <= yEnd; ++y) {
    const float yc = (float)y + 0.5f;
    const float xa = xt + slopeL * (yc - yt);
    const float xe2 = (yc < ym) ? xt + slopeTM * (yc - yt) : xm + slopeMB * (yc - ym);
    const float xl = xa < xe2 ? xa : xe2;
    const float xr = xa < xe2 ? xe2 : xa;

    int xs = ceilToInt(xl - 0.5f);
    int xe = ceilToInt(xr - 0.5f) - 1;
    if (xs < 0) xs = 0;
    if (xe > W - 1) xe = W - 1;
    if (xe < xs) continue;

    const float fx = ((float)xs + 0.5f) - x0;
    const float fy = yc - y0;
    fn(T.color + y * W, T.depth + y * W, xs, xe,
       i0 + diwdx*fx + diwdy*fy, diwdx,
       uw0 + duwdx*fx + duwdy*fy, duwdx,
       vw0 + dvwdx*fx + dvwdy*fy, dvwdx,
       tdata, tw, th, T.depthScale);
  }
}

// ---- construtor de meshlets (roda no load, 2 passadas: 1ª conta, 2ª grava) ----
static void mbFlush(MeshletBuild& b) {
  if (b.nt == 0) return;
  if (b.ml) {
    Meshlet& m = b.ml[b.nMl];
    Vtx* out = b.vp + b.nV;
    float mn[3] = { 1e30f, 1e30f, 1e30f }, mx[3] = { -1e30f, -1e30f, -1e30f };
    for (uint32_t i = 0; i < b.nu; ++i) {
      const uint32_t id = b.used[i];
      const float* p = b.pos + 3 * (size_t)id;
      out[i].x = p[0]; out[i].y = p[1]; out[i].z = p[2];
      out[i].u = b.uv ? b.uv[2 * (size_t)id]     : 0.0f;
      out[i].v = b.uv ? b.uv[2 * (size_t)id + 1] : 0.0f;
      if (b.sp) {
        uint8_t* d = b.sp + 8 * (size_t)(b.nV + i);
        if (b.sk) memcpy(d, b.sk + 8 * (size_t)id, 8); else memset(d, 0, 8);
      }
      for (int a = 0; a < 3; ++a) { if (p[a] < mn[a]) mn[a] = p[a]; if (p[a] > mx[a]) mx[a] = p[a]; }
    }
    for (int a = 0; a < 3; ++a) { m.c[a] = 0.5f * (mn[a] + mx[a]); m.h[a] = 0.5f * (mx[a] - mn[a]); }
    m.vOff = b.nV; m.tOff = b.nT; m.nv = (uint8_t)b.nu; m.nt = (uint8_t)b.nt;
    m.tex = (int16_t)b.tex; m.flags = b.flags;
    memcpy(b.tp + 3 * (size_t)b.nT, b.ltri, 3 * (size_t)b.nt);
  }
  for (uint32_t i = 0; i < b.nu; ++i) b.map[b.used[i]] = 0xFFFF;
  b.nMl++; b.nV += b.nu; b.nT += b.nt;
  b.nu = 0; b.nt = 0;
}

static void mbRun(MeshletBuild& b, const RzMesh* rm, uint32_t meshCount) {
  b.nMl = b.nV = b.nT = b.nu = b.nt = 0;
  for (uint32_t mi = 0; mi < meshCount; ++mi) {
    b.tex = rm[mi].texIndex; b.flags = rm[mi].flags;
    const uint32_t s = rm[mi].indexStart, e = s + (rm[mi].indexCount / 3) * 3;
    for (uint32_t k = s; k < e; k += 3) {
      const uint32_t id[3] = { b.idx[k], b.idx[k + 1], b.idx[k + 2] };
      if (id[0] >= b.V || id[1] >= b.V || id[2] >= b.V) continue;
      uint32_t need = 0;
      for (int a = 0; a < 3; ++a) {
        if (b.map[id[a]] != 0xFFFF) continue;
        bool dup = false;
        for (int q = 0; q < a; ++q) if (id[q] == id[a]) dup = true;
        if (!dup) ++need;
      }
      if (b.nu + need > ML_MAXV || b.nt >= ML_MAXT) mbFlush(b);
      for (int a = 0; a < 3; ++a) {
        uint16_t &mp = b.map[id[a]];
        if (mp == 0xFFFF) { mp = (uint16_t)b.nu; b.used[b.nu++] = id[a]; }
        b.ltri[3 * b.nt + a] = (uint8_t)mp;
      }
      ++b.nt;
    }
    mbFlush(b);     // um meshlet nunca mistura sub-malhas (textura/flags próprios)
  }
}

// tenta mover um bloco da PSRAM p/ RAM interna (só se sobrar PROMOTE_RESERVE depois). Devolve o ponteiro novo (ou o antigo).
static void* moveInternal(void* p, size_t bytes) {
  if (!p || !bytes) return p;
  if (heap_caps_get_free_size(MALLOC_CAP_INTERNAL) <= bytes + PROMOTE_RESERVE) return p;
  void* q = heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!q) return p;
  memcpy(q, p, bytes);
  heap_caps_free(p);
  return q;
}

struct Model {
  int vertCount;
  int triCount;
  Meshlet* ml;
  Vtx* vpool;
  uint8_t* tpool;
  int mlCount;
  TexData* texs;
  int texCount;
  int cull;
  uint16_t* order;         // meshlets frente -> trás (persistente: insertion sort quase O(n) entre quadros)
  float* skey;             // profundidade de cada meshlet neste quadro
  uint8_t* vflag;          // por meshlet, neste quadro: 0 = fora do frustum, 1 = dentro, 2 = dentro e todo na frente do near
  uint16_t* vis;           // meshlets visíveis no frustum, na ordem de desenho; bit15 = "inteiro na frente do near"
  int visCount;

  // ---- skinning / animação (só usados se o .rzobj for v4 com esqueleto) ----
  Vtx*      vdraw = nullptr;     // vértices que o draw() lê: vpool (modelo estático) ou vskin (skinned)
  Vtx*      vskin = nullptr;     // cópia do vpool; as POSIÇÕES são recalculadas quando a pose muda (u,v ficam)
  uint8_t*  vsk   = nullptr;     // por vértice do vpool: 4 índices de osso + 4 pesos (u8, somam 255; peso[0]==0 => estático)
  int       poolV = 0;           // nº de vértices do vpool
  SkBone*   bones = nullptr;
  int       boneCount = 0;
  float     rootM[12];           // matriz do pai das raízes do esqueleto (3x4)
  AnimClip* clips = nullptr;
  int       clipCount = 0;
  AKey*     keyPool = nullptr;
  float*    animMem = nullptr;   // bloco único: cur | from | out (10 floats por osso: t[3] q[4] s[3]) | pose (12 floats por osso)
  float*    cur = nullptr;       // TRS locais amostrados do clipe atual
  float*    from = nullptr;      // TRS locais exibidos quando o crossfade começou
  float*    out = nullptr;       // TRS locais exibidos (resultado do crossfade)
  float*    pose = nullptr;      // matrizes de skin por osso: mundo * invBind
  int       animClip = -1;
  float     animT = 0.0f, animSpeed = 1.0f;
  bool      animLoop = true, animPlay = false, blending = false;
  float     blendT = 0.0f, blendDur = 0.0f;

  Vec3 position;
  Vec3 rotation;
  Vec3 scale;

  Model()
    : vertCount(0), triCount(0),
      ml(nullptr), vpool(nullptr), tpool(nullptr), mlCount(0),
      texs(nullptr), texCount(0), cull(CULL_NONE),
      order(nullptr), skey(nullptr), vflag(nullptr), vis(nullptr), visCount(0),
      position({0,0,0}), rotation({0,0,0}), scale({1,1,1})
  {}

  bool loaded() const { return ml != nullptr; }

  // Libera TUDO que o modelo ocupa (meshlets, vértices, triângulos, ordem, texturas) e o deixa inerte:
  // prepare()/draw() viram no-op, então dá pra chamar de novo load/unload quando quiser.
  // SEGURANÇA: só chame fora de um quadro em andamento (ex.: no loop(), antes de renderScene, ou dentro
  // do próprio loop depois do xQueueReceive) — os 2 núcleos leem esses buffers durante o render.
  void unload() {
    visCount = 0;                      // 1º: qualquer draw/prepare passa a sair cedo
    mlCount = 0;
    if (texs) {
      for (int i = 0; i < texCount; ++i) if (texs[i].data) heap_caps_free(texs[i].data);
      heap_caps_free(texs);
    }
    if (ml)     heap_caps_free(ml);
    if (vpool)  heap_caps_free(vpool);
    if (tpool)  heap_caps_free(tpool);
    if (order)  heap_caps_free(order);
    if (skey)   heap_caps_free(skey);
    if (vis)    heap_caps_free(vis);
    if (vflag)  heap_caps_free(vflag);
    texs = nullptr; texCount = 0;
    ml = nullptr; vpool = nullptr; tpool = nullptr;
    order = nullptr; skey = nullptr; vis = nullptr; vflag = nullptr;
    vertCount = 0; triCount = 0;

    // skinning / animação
    animPlay = false; blending = false; animClip = -1; animT = 0.0f;
    if (vskin)   heap_caps_free(vskin);
    if (vsk)     heap_caps_free(vsk);
    if (bones)   heap_caps_free(bones);
    if (clips)   heap_caps_free(clips);
    if (keyPool) heap_caps_free(keyPool);
    if (animMem) heap_caps_free(animMem);
    vskin = nullptr; vsk = nullptr; vdraw = nullptr; poolV = 0;
    bones = nullptr; boneCount = 0; clips = nullptr; clipCount = 0; keyPool = nullptr;
    animMem = nullptr; cur = from = out = pose = nullptr;
  }

  // Carrega do SD. Se já havia algo carregado, descarrega antes; se o load falhar, não deixa lixo alocado.
  bool loadRzobj(const char* path) {
    unload();
    const bool ok = loadImpl(path);
    if (!ok) unload();
    return ok;
  }

  bool loadImpl(const char* path) {
    File f = SD.open(path, FILE_READ);
    if (!f) { errf("nao abriu %s (existe no SD?)", path); return false; }

    RzHeader h;
    if (!readAll(f, &h, sizeof(h)) || memcmp(h.magic, "RZOB", 4) != 0 || (h.version != 3 && h.version != 4)) {
      errf("cabecalho invalido em %s (versao != 3? reconverta o .glb)", path);
      f.close(); return false;
    }
    if (h.vertCount == 0 || h.indexCount < 3 || h.meshCount == 0) { f.close(); return false; }
    Serial.printf("Carregando %s: %u verts, %u tris, %u sub-malhas, %u texturas\n", path,
                  (unsigned)h.vertCount, (unsigned)(h.indexCount / 3),
                  (unsigned)h.meshCount, (unsigned)h.texCount);

    // ---- dados brutos (temporários: viram meshlets e são liberados antes das texturas) ----
    Scoped sPos, sUv, sIdx, sMesh, sMap, sSk;
    const float* rawUv = nullptr;
    float* rawPos = (float*)(sPos.p = allocFast((size_t)h.vertCount * 12, "vertices (tmp)", false));
    if (!rawPos || !readAll(f, rawPos, (size_t)h.vertCount * 12)) { f.close(); return false; }

    if (h.flags & RZ_UV) {
      float* u = (float*)(sUv.p = allocFast((size_t)h.vertCount * 8, "uvs (tmp)", false));
      if (!u || !readAll(f, u, (size_t)h.vertCount * 8)) { f.close(); return false; }
      rawUv = u;
    }

    uint32_t* rawIdx = (uint32_t*)(sIdx.p = allocFast((size_t)h.indexCount * 4, "indices (tmp)", false));
    if (!rawIdx) { f.close(); return false; }
    if (h.flags & RZ_IDX32) {
      if (!readAll(f, rawIdx, (size_t)h.indexCount * 4)) { f.close(); return false; }
    } else {
      uint16_t tmp[1024];
      uint32_t done = 0;
      while (done < h.indexCount) {
        uint32_t n = h.indexCount - done; if (n > 1024) n = 1024;
        if (!readAll(f, tmp, n * 2)) { f.close(); return false; }
        for (uint32_t i = 0; i < n; ++i) rawIdx[done + i] = tmp[i];
        done += n;
      }
    }

    // v4 com skin: 4 ossos + 4 pesos por vértice, logo depois dos índices
    const uint8_t* rawSk = nullptr;
    if (h.version >= 4 && (h.flags & RZ_SKIN)) {
      uint8_t* k = (uint8_t*)(sSk.p = allocFast((size_t)h.vertCount * 8, "pesos (tmp)", false));
      if (!k || !readAll(f, k, (size_t)h.vertCount * 8)) { f.close(); return false; }
      rawSk = k;
    }

    RzMesh* rm = (RzMesh*)(sMesh.p = allocFast(sizeof(RzMesh) * h.meshCount, "submalhas (tmp)", false));
    if (!rm) { f.close(); return false; }
    for (uint32_t i = 0; i < h.meshCount; ++i) {
      if (!readAll(f, &rm[i], sizeof(RzMesh))) { f.close(); return false; }
      if ((uint64_t)rm[i].indexStart + rm[i].indexCount > h.indexCount) {
        errf("%s: sub-malha %u fora do pool de indices", path, (unsigned)i);
        f.close(); return false;
      }
    }

    // ---- meshlets ----
    uint16_t* map = (uint16_t*)(sMap.p = allocFast((size_t)h.vertCount * 2, "mapa (tmp)", false));
    if (!map) { f.close(); return false; }
    memset(map, 0xFF, (size_t)h.vertCount * 2);

    MeshletBuild b = {};
    b.pos = rawPos; b.uv = rawUv; b.idx = rawIdx; b.V = h.vertCount; b.map = map;
    mbRun(b, rm, h.meshCount);                         // passada 1: só conta
    if (b.nMl == 0) { errf("%s: nenhum triangulo valido", path); f.close(); return false; }
    if (b.nMl > 32767) { errf("%s: meshlets demais (%u)", path, (unsigned)b.nMl); f.close(); return false; }
    const uint32_t nMl = b.nMl, nV = b.nV, nT = b.nT;

    ml     = (Meshlet*)allocFast(sizeof(Meshlet) * (size_t)nMl, "meshlets", false);
    vpool  = (Vtx*)    allocFast(sizeof(Vtx) * (size_t)nV,       "vertices", false);
    tpool  = (uint8_t*)allocFast(3 * (size_t)nT + 4,             "triangulos", false);
    if (!ml || !vpool || !tpool) { f.close(); return false; }
    b.ml = ml; b.vp = vpool; b.tp = tpool;
    if (rawSk) {
      vsk   = (uint8_t*)allocFast(8 * (size_t)nV,             "pesos",         false);
      vskin = (Vtx*)    allocFast(sizeof(Vtx) * (size_t)nV,   "vertices skin", false);
      if (!vsk || !vskin) { f.close(); return false; }
      b.sk = rawSk; b.sp = vsk;
    }
    mbRun(b, rm, h.meshCount);                         // passada 2: grava
    mlCount = (int)nMl;
    poolV = (int)nV;
    if (vskin) memcpy(vskin, vpool, sizeof(Vtx) * (size_t)nV);   // u,v copiados 1x; a cada pose só as posições mudam
    vdraw = vskin ? vskin : vpool;
    Serial.printf("  %u meshlets, %u vertices (%.2fx do original), %u triangulos\n", (unsigned)nMl, (unsigned)nV,
                  (double)nV / (double)h.vertCount, (unsigned)nT);

    sPos.release(); sUv.release(); sIdx.release(); sMesh.release(); sMap.release(); sSk.release();

    // ordem de desenho / lista de visíveis (RAM interna se sobrar; senão PSRAM)
    order = (uint16_t*)allocFast(sizeof(uint16_t) * (size_t)mlCount, "ordem", false);
    skey  = (float*)   allocFast(sizeof(float)    * (size_t)mlCount, "chaves z", false);
    vis   = (uint16_t*)allocFast(sizeof(uint16_t) * (size_t)mlCount, "visiveis", false);
    vflag = (uint8_t*) allocFast((size_t)mlCount,                    "flags vis.", false);
    if (!vis || !vflag) { f.close(); return false; }
    if (!order || !skey) { order = nullptr; skey = nullptr; }
    else for (int i = 0; i < mlCount; ++i) order[i] = (uint16_t)i;

    if (h.texCount > 0) {
      texs = (TexData*)allocFast(sizeof(TexData) * h.texCount, "tab. texturas", false);
      if (!texs) { f.close(); return false; }
      for (uint32_t i = 0; i < h.texCount; ++i) {
        RzTexEntry te;
        if (!readAll(f, &te, sizeof(te))) { f.close(); return false; }
        const bool pow2 = te.w && te.h && !(te.w & (te.w - 1)) && !(te.h & (te.h - 1));
        texs[i].w = te.w; texs[i].h = te.h; texs[i].data = nullptr;
        texs[i].repeat = pow2;
      }
      texCount = (int)h.texCount;
      for (int i = 0; i < texCount; ++i) {
        const size_t bytes = (size_t)texs[i].w * texs[i].h * 2;
        texs[i].data = (uint16_t*)allocFast(bytes, "textura", false);
        if (!texs[i].data || !readAll(f, texs[i].data, bytes)) { f.close(); return false; }
      }
    }

    // bloco de skin (ossos, clipes, keyframes): depois das texturas
    if (rawSk && !loadSkin(f)) { f.close(); return false; }

    f.close();
    vertCount = (int)h.vertCount;
    triCount  = (int)(h.indexCount / 3);
    if (boneCount > 0) rebindPose();       // pose de repouso: posiciona vskin e as AABBs dos meshlets
    return true;
  }

  // ---------------------------------------------------------------------------------------------------------------
  //  Skinning + animação
  //  Por quadro (só se a pose mudou): TRS local de cada osso (amostrado do clipe, com crossfade) -> matriz de mundo
  //  (hierarquia) -> pose = mundo * invBind (~10 ossos: desprezível) -> cada vértice vira a soma ponderada de até 4
  //  ossos, gravada em vskin. As AABBs dos meshlets são recalculadas junto, então frustum culling e z-order seguem
  //  exatos na pose atual. O raster não muda: o draw() só lê vdraw. Tudo roda no fluxo do loop(), FORA do quadro
  //  (os 2 núcleos só leem esses buffers durante renderScene), por isso não há lock.
  // ---------------------------------------------------------------------------------------------------------------
  bool loadSkin(File &f) {
    RzSkinHdr sh;
    if (!readAll(f, &sh, sizeof(sh))) return false;
    if (sh.boneCount == 0 || sh.boneCount > MAX_BONES || sh.clipCount > MAX_CLIPS) {
      errf("esqueleto invalido: %u ossos (max %d), %u clipes (max %d)", (unsigned)sh.boneCount, MAX_BONES,
           (unsigned)sh.clipCount, MAX_CLIPS);
      return false;
    }
    const int bc = (int)sh.boneCount;
    memcpy(rootM, sh.root, sizeof(rootM));

    bones = (SkBone*)allocFast(sizeof(SkBone) * (size_t)bc, "ossos", false);
    if (!bones || !readAll(f, bones, sizeof(SkBone) * (size_t)bc)) return false;
    for (int i = 0; i < bc; ++i)
      if (bones[i].parent >= i) { errf("osso %d: pai (%d) deve vir antes do filho", i, (int)bones[i].parent); return false; }

    animMem = (float*)allocFast(sizeof(float) * (size_t)bc * (10 * 3 + 12), "pose/anim", true);
    if (!animMem) return false;
    cur = animMem; from = cur + 10 * bc; out = from + 10 * bc; pose = out + 10 * bc;
    boneCount = bc;

    if (sh.clipCount) {
      clips = (AnimClip*)allocFast(sizeof(AnimClip) * (size_t)sh.clipCount, "clipes", false);
      if (!clips) return false;
      size_t totalKeys = 0;
      for (int i = 0; i < (int)sh.clipCount; ++i) {
        RzClipEntry e;
        if (!readAll(f, &e, sizeof(e))) return false;
        if (e.nFrames == 0) { errf("clipe %d sem quadros", i); return false; }
        memcpy(clips[i].name, e.name, sizeof(e.name));
        clips[i].name[sizeof(clips[i].name) - 1] = 0;
        clips[i].nFrames = e.nFrames; clips[i].flags = e.flags; clips[i].duration = e.duration;
        clips[i].keys = nullptr;
        totalKeys += (size_t)e.nFrames * (size_t)bc;
      }
      keyPool = (AKey*)allocFast(totalKeys * sizeof(AKey), "keyframes", false);
      if (!keyPool || !readAll(f, keyPool, totalKeys * sizeof(AKey))) return false;
      AKey* p = keyPool;
      for (int i = 0; i < (int)sh.clipCount; ++i) { clips[i].keys = p; p += (size_t)clips[i].nFrames * (size_t)bc; }
      clipCount = (int)sh.clipCount;
    }
    Serial.printf("  esqueleto: %d ossos, %d clipes\n", boneCount, clipCount);
    for (int i = 0; i < clipCount; ++i)
      Serial.printf("    [%d] %-19s %.2f s, %u quadros\n", i, clips[i].name, (double)clips[i].duration, (unsigned)clips[i].nFrames);
    return true;
  }

  // ---- consulta ----
  bool        skinned() const        { return boneCount > 0; }
  int         animCount() const      { return clipCount; }
  const char* animName(int i) const  { return (i >= 0 && i < clipCount) ? clips[i].name : nullptr; }
  float       animDuration(int i) const { return (i >= 0 && i < clipCount) ? clips[i].duration : 0.0f; }
  int         findAnim(const char* name) const {
    for (int i = 0; i < clipCount; ++i) if (strcmp(clips[i].name, name) == 0) return i;
    return -1;
  }
  bool  animPlaying() const  { return animPlay; }                       // false quando um clipe sem loop chegou ao fim
  int   currentAnim() const  { return animClip; }                       // -1 = nenhum (após stopAnim)
  float animTime() const     { return animT; }

  // Toca o clipe `clip`. loop=false: congela no último quadro e animPlaying() vira false. speed < 0 toca ao contrário.
  // blendSec > 0: crossfade a partir da pose que estava na tela (0 = corte seco). Chame de fora de um quadro (ex.: setup/loop).
  bool playAnim(int clip, bool loop = true, float speed = 1.0f, float blendSec = 0.15f) {
    if (!pose || clip < 0 || clip >= clipCount) return false;
    if (blendSec > 0.0f) {
      memcpy(from, out, sizeof(float) * 10 * (size_t)boneCount);
      blending = true; blendT = 0.0f; blendDur = blendSec;
    } else {
      blending = false;
    }
    animClip = clip; animLoop = loop; animSpeed = speed; animPlay = true;
    animT = (speed < 0.0f) ? clips[clip].duration : 0.0f;
    sampleClip(clips[clip], animT, cur);
    return true;
  }
  bool playAnim(const char* name, bool loop = true, float speed = 1.0f, float blendSec = 0.15f) {
    return playAnim(findAnim(name), loop, speed, blendSec);
  }

  // Para a animação. resetPose=true volta ao bind pose (pose de repouso do .glb); false congela na pose atual.
  void stopAnim(bool resetPose = true) {
    animPlay = false; blending = false; animClip = -1; animT = 0.0f;
    if (resetPose) rebindPose();
  }

  // Avança o relógio da animação em dt segundos e, se algo mudou, refaz a pose e os vértices. Custo zero com tudo parado.
  // Chame 1x por quadro ANTES de renderScene() (ex.: no loop(), logo após calcular o dt).
  void updateAnim(float dt) {
    if (!pose || (!animPlay && !blending)) return;
    const int n = 10 * boneCount;
    if (animPlay) {
      const AnimClip& c = clips[animClip];
      const float d = c.duration;
      animT += dt * animSpeed;
      if (d <= 0.0f) {                                   // clipe de 1 quadro: pose fixa
        animT = 0.0f; animPlay = false;
      } else if (animLoop) {
        if (animT >= d || animT < 0.0f) { animT = fmodf(animT, d); if (animT < 0.0f) animT += d; }
      } else if (animSpeed >= 0.0f ? (animT >= d) : (animT <= 0.0f)) {
        animT = (animSpeed >= 0.0f) ? d : 0.0f; animPlay = false;   // fim: segura o último quadro
      }
      sampleClip(c, animT, cur);
    }
    if (blending) {
      blendT += dt;
      const float a = (blendDur > 0.0f) ? blendT / blendDur : 1.0f;
      if (a >= 1.0f) { blending = false; memcpy(out, cur, sizeof(float) * (size_t)n); }
      else blendLocal(a);
    } else {
      memcpy(out, cur, sizeof(float) * (size_t)n);
    }
    applyPose();
  }

  // volta ao bind pose (TRS de repouso do esqueleto) e atualiza vértices/AABBs
  void rebindPose() {
    if (!pose) return;
    for (int b = 0; b < boneCount; ++b) {
      float* L = cur + 10 * b;
      memcpy(L,     bones[b].t, 3 * sizeof(float));
      memcpy(L + 3, bones[b].q, 4 * sizeof(float));
      memcpy(L + 7, bones[b].s, 3 * sizeof(float));
    }
    memcpy(out,  cur, sizeof(float) * 10 * (size_t)boneCount);
    memcpy(from, cur, sizeof(float) * 10 * (size_t)boneCount);
    applyPose();
  }

  // amostra o clipe no instante t (s): lerp entre 2 quadros vizinhos (t, s: lerp; q: nlerp pelo caminho curto)
  void sampleClip(const AnimClip& c, float t, float* o) const {
    const int bc = boneCount;
    int f0 = 0, f1 = 0;
    float a = 0.0f;
    if (c.nFrames > 1 && c.duration > 0.0f) {
      float fr = t * (float)(c.nFrames - 1) / c.duration;
      if (fr < 0.0f) fr = 0.0f;
      f0 = (int)fr; a = fr - (float)f0;
      if (f0 >= (int)c.nFrames - 1) { f0 = (int)c.nFrames - 1; f1 = f0; a = 0.0f; }
      else f1 = f0 + 1;
    }
    const AKey* k0 = c.keys + (size_t)f0 * bc;
    const AKey* k1 = c.keys + (size_t)f1 * bc;
    const float qs = 1.0f / 32767.0f;
    for (int b = 0; b < bc; ++b) {
      const AKey &A = k0[b], &B = k1[b];
      float* O = o + 10 * b;
      for (int k = 0; k < 3; ++k) O[k] = A.t[k] + (B.t[k] - A.t[k]) * a;
      float qa[4], qb[4], d = 0.0f;
      for (int k = 0; k < 4; ++k) { qa[k] = (float)A.q[k] * qs; qb[k] = (float)B.q[k] * qs; d += qa[k] * qb[k]; }
      const float sg = (d < 0.0f) ? -1.0f : 1.0f, wa = 1.0f - a, wb = sg * a;
      for (int k = 0; k < 4; ++k) O[3 + k] = qa[k] * wa + qb[k] * wb;
      qnorm4(O + 3);
      for (int k = 0; k < 3; ++k) O[7 + k] = A.s[k] + (B.s[k] - A.s[k]) * a;
    }
  }

  // out = mistura(from, cur, a) osso a osso
  void blendLocal(float a) {
    for (int b = 0; b < boneCount; ++b) {
      const float *A = from + 10 * b, *B = cur + 10 * b;
      float* O = out + 10 * b;
      for (int k = 0; k < 3; ++k) { O[k] = A[k] + (B[k] - A[k]) * a; O[7 + k] = A[7 + k] + (B[7 + k] - A[7 + k]) * a; }
      const float d = A[3]*B[3] + A[4]*B[4] + A[5]*B[5] + A[6]*B[6];
      const float sg = (d < 0.0f) ? -1.0f : 1.0f;
      for (int k = 0; k < 4; ++k) O[3 + k] = A[3 + k] * (1.0f - a) + sg * B[3 + k] * a;
      qnorm4(O + 3);
    }
  }

  // TRS locais (out) -> matrizes de pose
  void buildPose() {
    for (int b = 0; b < boneCount; ++b) {
      const float* L = out + 10 * b;
      const float x = L[3], y = L[4], z = L[5], w = L[6];
      const float xx = x*x, yy = y*y, zz = z*z, xy = x*y, xz = x*z, yz = y*z, wx = w*x, wy = w*y, wz = w*z;
      const float sx = L[7], sy = L[8], sz = L[9];
      float m[12];
      m[0] = (1.0f - 2.0f*(yy + zz)) * sx; m[1] = 2.0f*(xy - wz) * sy;          m[2]  = 2.0f*(xz + wy) * sz;          m[3]  = L[0];
      m[4] = 2.0f*(xy + wz) * sx;          m[5] = (1.0f - 2.0f*(xx + zz)) * sy; m[6]  = 2.0f*(yz - wx) * sz;          m[7]  = L[1];
      m[8] = 2.0f*(xz - wy) * sx;          m[9] = 2.0f*(yz + wx) * sy;          m[10] = (1.0f - 2.0f*(xx + yy)) * sz; m[11] = L[2];
      const float* P = (bones[b].parent >= 0) ? gWorld[bones[b].parent] : rootM;
      mul34(P, m, gWorld[b]);
      mul34(gWorld[b], bones[b].inv, pose + 12 * b);
    }
  }

  // cada vértice = soma(peso * pose[osso] * posição_bind), 1 meshlet por vez; recalcula a AABB do meshlet
  void skinVertices() {
    if (!vskin || !vsk) return;
    const int bc = boneCount;
    const float k255 = 1.0f / 255.0f;
    for (int i = 0; i < mlCount; ++i) {
      Meshlet& s = ml[i];
      const Vtx* src = vpool + s.vOff;
      Vtx* dst = vskin + s.vOff;
      const uint8_t* sk = vsk + 8 * (size_t)s.vOff;
      float mn[3] = { 1e30f, 1e30f, 1e30f }, mx[3] = { -1e30f, -1e30f, -1e30f };
      for (int j = 0; j < s.nv; ++j, sk += 8) {
        const float x = src[j].x, y = src[j].y, z = src[j].z;
        float ox = x, oy = y, oz = z;                      // peso[0] == 0: vértice estático (sem osso)
        if (sk[4]) {
          ox = oy = oz = 0.0f;
          for (int k = 0; k < 4; ++k) {
            const unsigned wt = sk[4 + k];
            if (!wt) break;
            unsigned bi = sk[k]; if (bi >= (unsigned)bc) bi = 0;
            const float* P = pose + 12 * bi;
            const float w = (float)wt * k255;
            ox += w * (P[0]*x + P[1]*y + P[2]*z  + P[3]);
            oy += w * (P[4]*x + P[5]*y + P[6]*z  + P[7]);
            oz += w * (P[8]*x + P[9]*y + P[10]*z + P[11]);
          }
        }
        dst[j].x = ox; dst[j].y = oy; dst[j].z = oz;
        if (ox < mn[0]) mn[0] = ox;
        if (ox > mx[0]) mx[0] = ox;
        if (oy < mn[1]) mn[1] = oy;
        if (oy > mx[1]) mx[1] = oy;
        if (oz < mn[2]) mn[2] = oz;
        if (oz > mx[2]) mx[2] = oz;
      }
      for (int a = 0; a < 3; ++a) { s.c[a] = 0.5f * (mn[a] + mx[a]); s.h[a] = 0.5f * (mx[a] - mn[a]); }
    }
  }

  void applyPose() { buildPose(); skinVertices(); }

  Mat4 getModelMatrix() const {
    Mat4 S = matScale(scale.x, scale.y, scale.z);
    Mat4 R = matMul(matRotZ(rotation.z), matMul(matRotY(rotation.y), matRotX(rotation.x)));
    Mat4 T = matTranslation(position.x, position.y, position.z);
    return matMul(T, matMul(R, S));
  }

  // Passo SERIAL do mestre (antes de liberar as faixas): descarta os meshlets fora do frustum, ordena frente->trás
  // e publica a lista `vis`. Os 2 núcleos só LEEM `vis`; cada faixa ainda faz seu teste de linhas.
  // Frente->trás = early-z: o que vem atrás falha no z-test sem amostrar textura.
  // Os cabeçalhos (PSRAM) são lidos UMA vez, em sequência; o sort e a montagem de `vis` só mexem em arrays pequenos
  // (RAM interna, veja promoteFrameData). Insertion sort sobre a ordem do quadro anterior (~O(n)); com orçamento
  // de deslocamentos: se estourar (câmera "teleportou"), cai num std::sort.
  void prepare(const Mat4 &MVP, float nearW) {
    visCount = 0;
    if (mlCount <= 0 || !vis || !vflag) return;
    const float (*M)[4] = MVP.m;
    const bool doSort = (ML_SORT != 0) && order && skey && mlCount > 1;

    // planos do frustum no formato p·(x,y,z,1) >= 0 (= dentro): esq, dir, baixo, cima, near
    float P[5][4], A[5][3];
    for (int k = 0; k < 4; ++k) {
      P[0][k] = M[3][k] + M[0][k];
      P[1][k] = M[3][k] - M[0][k];
      P[2][k] = M[3][k] + M[1][k];
      P[3][k] = M[3][k] - M[1][k];
      P[4][k] = M[3][k];
    }
    P[4][3] -= nearW;
    for (int p = 0; p < 5; ++p) for (int k = 0; k < 3; ++k) A[p][k] = fabsf(P[p][k]);

    // passe sequencial: chave de profundidade + teste de frustum de cada meshlet
    for (int i = 0; i < mlCount; ++i) {
      const Meshlet &s = ml[i];
      if (doSort) skey[i] = M[3][0]*s.c[0] + M[3][1]*s.c[1] + M[3][2]*s.c[2] + M[3][3];   // w = profundidade de view
      // AABB fora de um plano <=> o MAIOR valor de p·X na caixa é < 0 (= os 8 cantos fora do mesmo plano)
      uint8_t f = 1;
      float dN = 0.0f, eN = 0.0f;
      for (int p = 0; p < 5; ++p) {
        const float d = P[p][0]*s.c[0] + P[p][1]*s.c[1] + P[p][2]*s.c[2] + P[p][3];
        const float e = A[p][0]*s.h[0] + A[p][1]*s.h[1] + A[p][2]*s.h[2];
        if (d + e < 0.0f) { f = 0; break; }
        dN = d; eN = e;                         // no último passo (p=4) são os valores do plano near
      }
      if (f && (dN - eN) >= 0.0f) f = 2;        // MENOR w na caixa >= near: a projeção da caixa é confiável
      vflag[i] = f;
    }

    if (doSort) {
      int budget = 16 * mlCount;
      bool bail = false;
      for (int i = 1; i < mlCount && !bail; ++i) {
        const uint16_t o = order[i];
        const float k = skey[o];
        int j = i - 1;
        while (j >= 0 && skey[order[j]] > k) {
          order[j + 1] = order[j]; --j;
          if (--budget < 0) { bail = true; break; }
        }
        order[j + 1] = o;
      }
      if (bail) {
        const float* kk = skey;
        std::sort(order, order + mlCount, [kk](uint16_t a, uint16_t b) { return kk[a] < kk[b]; });
      }
    }

    int n = 0;
    for (int i = 0; i < mlCount; ++i) {
      const int id = order ? order[i] : i;
      const uint8_t f = vflag[id];
      if (f) vis[n++] = (uint16_t)(id | (f == 2 ? 0x8000 : 0));
    }
    visCount = n;
  }

  // Depois que os buffers grandes (z, cores, DMA) e as tarefas já existem: move pra RAM interna os arrays que o
  // mestre e as faixas tocam o tempo todo (pequenos), se sobrar espaço. Chame 1x por modelo, no setup().
  void promoteFrameData() {
    if (mlCount <= 0) return;
    const size_t n = (size_t)mlCount;
    uint16_t* v0 = vis;
    vflag = (uint8_t*) moveInternal(vflag, n);
    vis   = (uint16_t*)moveInternal(vis,   2 * n);
    if (order && skey) {
      order = (uint16_t*)moveInternal(order, 2 * n);
      skey  = (float*)   moveInternal(skey,  4 * n);
    }
    Serial.printf("  arrays por quadro (%u meshlets): %s\n", (unsigned)n, (vis != v0) ? "RAM interna" : "PSRAM (sem espaco)");
    if (vskin && poolV > 0) {                       // vértices skinned: lidos por todas as faixas a cada quadro
      Vtx* v1 = vskin;
      vskin = (Vtx*)moveInternal(vskin, sizeof(Vtx) * (size_t)poolV);
      vdraw = vskin;
      Serial.printf("  vertices skin (%d): %s\n", poolV, (vskin != v1) ? "RAM interna" : "PSRAM");
    }
  }

  // Move texturas pra RAM interna, das mais "densas" (triângulos que a usam / bytes) pras menos, enquanto
  // sobrar PROMOTE_RESERVE. A cache de dados da PSRAM é pequena; textura em RAM interna não sofre miss.
  void promoteTextures() {
    if (!texs || texCount <= 0 || mlCount <= 0) return;
    uint32_t* use = (uint32_t*)heap_caps_malloc(sizeof(uint32_t) * (size_t)texCount, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!use) return;
    memset(use, 0, sizeof(uint32_t) * (size_t)texCount);
    for (int i = 0; i < mlCount; ++i) if (ml[i].tex >= 0 && ml[i].tex < texCount) use[ml[i].tex] += ml[i].nt;
    int moved = 0; size_t movedBytes = 0;
    for (;;) {
      int best = -1; float bestScore = 0.0f;
      for (int i = 0; i < texCount; ++i) {
        if (!use[i] || !texs[i].data) continue;
        const float sc = (float)use[i] / (float)((size_t)texs[i].w * texs[i].h * 2);
        if (sc > bestScore) { bestScore = sc; best = i; }
      }
      if (best < 0) break;
      const size_t bytes = (size_t)texs[best].w * texs[best].h * 2;
      use[best] = 0;                                   // não tenta de novo, caiba ou não
      uint16_t* q = (uint16_t*)moveInternal(texs[best].data, bytes);
      if (q != texs[best].data) { texs[best].data = q; ++moved; movedBytes += bytes; }
    }
    heap_caps_free(use);
    Serial.printf("  texturas em RAM interna: %d de %d (%u B)\n", moved, texCount, (unsigned)movedBytes);
  }

  // definido DEPOIS da classe: função-membro definida dentro da classe é implicitamente inline (seção COMDAT) e, na IRAM,
  // o linker põe os literais float DEPOIS do código => "dangerous relocation: l32r: literal placed after use".
  void draw(const Mat4 &MVP, const Target &T) const;
};

void HOT Model::draw(const Mat4 &MVP, const Target &T) const
  {
    if (visCount <= 0 || !ml) return;

    const float fW = (float)T.W, fH = (float)T.H;
    const float fy0 = (float)T.y0, fy1 = (float)T.y1;
    SV *sv = T.sv;
    uint8_t *oc = T.oc;
    const float (*M)[4] = MVP.m;
    const float nearW = T.nearW;

    // Planos "acima da faixa" / "abaixo da faixa" (margem de 1 linha). Um ponto com w>0 está na linha
    // syc = (0.5 - cy/w*0.5)*H; syc >= fy1+1  <=>  cy - kb*w <= 0 ;  syc < fy0-1  <=>  cy - ka*w > 0.
    const float ka = 1.0f - 2.0f * (fy0 - 1.0f) / fH;
    const float kb = 1.0f - 2.0f * (fy1 + 1.0f) / fH;
    float pa[4], pb[4];
    for (int k = 0; k < 4; ++k) { pa[k] = M[1][k] - ka * M[3][k]; pb[k] = M[1][k] - kb * M[3][k]; }
    const float apa0 = fabsf(pa[0]), apa1 = fabsf(pa[1]), apa2 = fabsf(pa[2]);
    const float apb0 = fabsf(pb[0]), apb1 = fabsf(pb[1]), apb2 = fabsf(pb[2]);

    for (int vi = 0; vi < visCount; ++vi) {
      const uint16_t e = vis[vi];
      const Meshlet &s = ml[e & 0x7FFF];

      // ---- descarta o meshlet se a projeção dele não toca a faixa de linhas deste chamador ----
      if (e & 0x8000) {
        const float cx = s.c[0], cy = s.c[1], cz = s.c[2], hx = s.h[0], hy = s.h[1], hz = s.h[2];
        const float da = pa[0]*cx + pa[1]*cy + pa[2]*cz + pa[3];
        if (da - (apa0*hx + apa1*hy + apa2*hz) > 0.0f) continue;     // tudo acima
        const float db = pb[0]*cx + pb[1]*cy + pb[2]*cz + pb[3];
        if (db + (apb0*hx + apb1*hy + apb2*hz) <= 0.0f) continue;    // tudo abaixo
      }

      const Vtx* vb = vdraw + s.vOff;
      const uint8_t* tp = tpool + 3 * (size_t)s.tOff;
      const int nv = s.nv, nt = s.nt;

      // ---- projeta os vértices do meshlet (≤ ML_MAXV, resultados em RAM interna) ----
      for (int j = 0; j < nv; ++j) {
        const Vtx &p = vb[j];
        const float x = p.x, y = p.y, z = p.z;
        const float w  = M[3][0]*x + M[3][1]*y + M[3][2]*z + M[3][3];
        const float cx = M[0][0]*x + M[0][1]*y + M[0][2]*z + M[0][3];
        const float cy = M[1][0]*x + M[1][1]*y + M[1][2]*z + M[1][3];
        // outcode do vértice contra os 5 planos do frustum, em espaço de clip (vale também atrás da câmera)
        unsigned code = 0;
        if (cx < -w) code |= 1;
        if (cx >  w) code |= 2;
        if (cy < -w) code |= 4;
        if (cy >  w) code |= 8;
        if (w < nearW) code |= 16;
        oc[j] = (uint8_t)code;
        SV &o = sv[j];
        if (w < nearW) { o.iw = -1.0f; continue; }
        const float inv = rcpf(w);
        o.x = (cx * inv * 0.5f + 0.5f) * fW;
        o.y = (0.5f - cy * inv * 0.5f) * fH;
        o.iw = inv;                         // 1/w (positivo); -1 = atrás do near
        o.u = p.u; o.v = p.v;
      }

      const TexData* td = (s.tex >= 0 && s.tex < texCount) ? &texs[s.tex] : nullptr;
      const bool tex = (td && td->data);
      const bool key = tex && (s.flags & RZM_KEY);
      const bool rep = tex && td->repeat;
      const int fi = (tex ? 8 : 0) | (rep ? 2 : 0) | (key ? 1 : 0) | ((TEX_FILTER && tex && !key) ? 16 : 0);
      const uint16_t* tdata = tex ? td->data : nullptr;
      const int tw = tex ? td->w : 1, th = tex ? td->h : 1;
      const int cullMode = (s.flags & RZM_DOUBLE) ? CULL_NONE : cull;

      for (int ti = 0; ti < nt; ++ti, tp += 3) {
        const int l0 = tp[0], l1 = tp[1], l2 = tp[2];

        // ---- frustum culling POR TRIÂNGULO ----
        // Os 3 vértices fora do MESMO plano => o triângulo inteiro está fora. Um único AND de bytes.
        if (oc[l0] & oc[l1] & oc[l2]) continue;

        const SV &A = sv[l0], &B = sv[l1], &C = sv[l2];

        if (A.iw >= 0.0f && B.iw >= 0.0f && C.iw >= 0.0f) {
          // (esq/dir/cima/baixo da tela já foram tratados pelo outcode; falta só a faixa deste núcleo)
          if (max3f(A.y, B.y, C.y) < fy0 || min3f(A.y, B.y, C.y) >= fy1) continue;
          if (cullMode != CULL_NONE) {
            const float area = (B.x - A.x) * (C.y - A.y) - (C.x - A.x) * (B.y - A.y);
            if (cullMode == CULL_POS_AREA && area > 0.0f) continue;
            if (cullMode == CULL_NEG_AREA && area < 0.0f) continue;
          }
          rasterTri(T, fi, CULL_NONE, tdata, tw, th, A, B, C);   // o culling por área já foi feito logo acima
          continue;
        }
        if (A.iw < 0.0f && B.iw < 0.0f && C.iw < 0.0f) continue;

        // ---- recorte contra o plano near (w >= nearW) ----
        struct CV { float cx, cy, w, u, v; };
        const int lid[3] = { l0, l1, l2 };
        CV in[3];
        for (int k = 0; k < 3; ++k) {
          const Vtx &p = vb[lid[k]];
          in[k].cx = M[0][0]*p.x + M[0][1]*p.y + M[0][2]*p.z + M[0][3];
          in[k].cy = M[1][0]*p.x + M[1][1]*p.y + M[1][2]*p.z + M[1][3];
          in[k].w  = M[3][0]*p.x + M[3][1]*p.y + M[3][2]*p.z + M[3][3];
          in[k].u  = p.u;
          in[k].v  = p.v;
        }
        CV poly[4]; int n = 0;
        for (int k = 0; k < 3; ++k) {
          const CV &a = in[k], &b = in[(k + 1) % 3];
          const float da = a.w - nearW, db = b.w - nearW;
          if (da >= 0.0f) poly[n++] = a;
          if ((da >= 0.0f) != (db >= 0.0f)) {
            const float t = da / (da - db);
            poly[n].cx = a.cx + (b.cx - a.cx) * t;
            poly[n].cy = a.cy + (b.cy - a.cy) * t;
            poly[n].w  = nearW;
            poly[n].u  = a.u + (b.u - a.u) * t;
            poly[n].v  = a.v + (b.v - a.v) * t;
            ++n;
          }
        }
        if (n < 3) continue;
        SV pv[4];
        for (int k = 0; k < n; ++k) {
          const float inv = 1.0f / poly[k].w;
          pv[k].x = (poly[k].cx * inv * 0.5f + 0.5f) * fW;
          pv[k].y = (0.5f - poly[k].cy * inv * 0.5f) * fH;
          pv[k].iw = inv; pv[k].u = poly[k].u; pv[k].v = poly[k].v;
        }
        for (int k = 1; k + 1 < n; ++k)
          rasterTri(T, fi, cullMode, tdata, tw, th, pv[0], pv[k], pv[k + 1]);
      }
    }
  }
// ==== RASTER END ============================================================

struct Camera {
  Vec3 position;
  Vec3 target;
  Vec3 up;
  float fov;
  float nearZ;
  float farZ;

  float yaw;
  float pitch;

  Camera() {
    position = {0.0f, 0.0f, -3.0f};
    target   = {0.0f, 0.0f, 0.0f};
    up       = {0.0f, 1.0f, 0.0f};
    fov      = 60.0f * (3.14159265f/180.0f);
    nearZ    = 0.3f;
    farZ     = 100.0f;
    yaw      = 0.0f;
    pitch    = 0.0f;
  }

  void orbitate(float radius) {
    position.x = target.x + radius * cosf(pitch) * sinf(yaw);
    position.y = target.y + radius * sinf(pitch);
    position.z = target.z + radius * cosf(pitch) * cosf(yaw);
  }

  void freecam(){
    Vec3 front;
    front.x = cosf(pitch) * sinf(yaw);
    front.y = sinf(pitch);
    front.z = cosf(pitch) * cosf(yaw);

    target.x = position.x + front.x;
    target.y = position.y + front.y;
    target.z = position.z + front.z;
  }

  void move(const Vec3 &delta) {
    position.x += delta.x;
    position.y += delta.y;
    position.z += delta.z;
  }

  void rotate(float deltaYaw, float deltaPitch) {
    yaw   += deltaYaw;
    pitch += deltaPitch;
    if(pitch >  1.57f) pitch =  1.57f;
    if(pitch < -1.57f) pitch = -1.57f;
  }

  Mat4 getViewMatrix() const {
    return lookAt(position, target, up);
  }
};

// ---------------- Globals & buffers ----------------
int SCR_W, SCR_H;
int HR_W, HR_H;

static uint16_t *colorBuf[2] = {nullptr, nullptr};
static uint16_t *zBuf = nullptr;
static uint16_t *dmaBuf[2] = {nullptr, nullptr};
// scratch de projeção de UM meshlet por contexto de raster (0 = núcleo do loop, 1 = worker): RAM interna estática
static SV      gSV[2][ML_MAXV];
static uint8_t gOC[2][ML_MAXV];

static QueueHandle_t freeQ, fullQ;

Camera camera;
Model m;
Model m2;
Model sky;

unsigned long lastTime = 0;
float dt = 0;
static volatile float gShownFps = 0.0f;
static char gFpsTxt[24] = "FPS: --";
static uint32_t renderUsAcc = 0, renderFrames = 0, prepUsAcc = 0, waitUsAcc = 0;
static unsigned long lastPrint = 0;
static int64_t gLastUs = 0;        // relógio de 64 bits (micros() de 32 bits dá a volta a cada ~71 min)

// ---------------- render: faixas distribuídas entre os núcleos ----------------
// O quadro é dividido em RENDER_BANDS faixas horizontais. Cada núcleo pega a próxima faixa livre
// (contador atômico), limpa as linhas dela e desenha a cena inteira só naquelas linhas.
// Núcleo ocupado com o blit => o outro simplesmente pega mais faixas (balanceamento automático).
struct FrameJob {
  uint16_t* color;
  float nearW, depthScale;
  int n;
  Model* model[3];
  Mat4 mvp[3];
};
static FrameJob gJob;
static std::atomic<int> gNextBand(0);
static std::atomic<int> gBandsDone(0);
static TaskHandle_t gMasterTask = nullptr;
static TaskHandle_t gWorkerTask = nullptr;

static void renderBands(int ctx) {
  for (;;) {
    const int b = gNextBand.fetch_add(1);
    if (b >= RENDER_BANDS) break;

    // gJob só é lido DEPOIS de reivindicar uma faixa (o mestre publica gJob antes de zerar o contador)
    Target T;
    T.W = HR_W; T.H = HR_H;
    T.y0 = (b * HR_H) / RENDER_BANDS;
    T.y1 = ((b + 1) * HR_H) / RENDER_BANDS;
    T.color = gJob.color; T.depth = zBuf;
    T.sv = gSV[ctx]; T.oc = gOC[ctx];
    T.nearW = gJob.nearW;
    T.depthScale = gJob.depthScale;

    // limpa só as linhas desta faixa: cor preta e profundidade 0 (= infinito, pois guardamos 1/w)
    const size_t off = (size_t)T.y0 * HR_W, cnt = (size_t)(T.y1 - T.y0) * HR_W;
    memset(zBuf + off, 0x00, cnt * 2);
    memset(T.color + off, 0x00, cnt * 2);

    // modelos primeiro, céu por último: pixels do céu atrás de um modelo falham no z-test sem amostrar textura
    for (int i = 0; i < gJob.n; ++i) gJob.model[i]->draw(gJob.mvp[i], T);

    if (gBandsDone.fetch_add(1) + 1 == RENDER_BANDS) xTaskNotifyGive(gMasterTask);
  }
}

#if RENDER_DUALCORE
static void rasterWorker(void*) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    renderBands(1);
  }
}
#endif

static void renderScene(uint16_t* color) {
  Mat4 view = camera.getViewMatrix();
  Mat4 proj = perspective(camera.fov, (float)HR_W/(float)HR_H, camera.nearZ, camera.farZ);
  Mat4 VP = matMul(proj, view);

  gJob.color = color;
  gJob.nearW = camera.nearZ;
  gJob.depthScale = 60000.0f * camera.nearZ;
  int n = 0;
  gJob.model[n] = &m;  gJob.mvp[n++] = matMul(VP, m.getModelMatrix());
  gJob.model[n] = &m2; gJob.mvp[n++] = matMul(VP, m2.getModelMatrix());
#if DRAW_SKY
  gJob.model[n] = &sky; gJob.mvp[n++] = matMul(VP, sky.getModelMatrix());
#endif
  gJob.n = n;

  // ordena + culling por frustum dos meshlets (só o mestre; os núcleos apenas leem a lista de visíveis)
  const uint32_t tp0 = micros();
  for (int i = 0; i < n; ++i) gJob.model[i]->prepare(gJob.mvp[i], camera.nearZ);
  prepUsAcc += micros() - tp0;

  gBandsDone.store(0);
  gNextBand.store(0);            // publica o job (seq_cst)
#if RENDER_DUALCORE
  xTaskNotifyGive(gWorkerTask);
#endif
  renderBands(0);
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);   // quem terminar a última faixa avisa o mestre
}

// ---------------- blit (núcleo 0) ----------------
// O LovyanGFX espera o DMA anterior terminar em BUSY-WAIT (laço apertado). Como o transporte de um quadro inteiro
// dura ~30 ms (480*320*16 bits a 80 MHz), a tarefa de blit (prioridade 2) passava quase todo o tempo girando no núcleo 0
// e o worker de raster (prioridade 1) quase nunca rodava. Agora o blit DORME num timer de alta resolução até um
// pouquinho antes do fim estimado do DMA e só gira o restinho. A margem se auto-ajusta.
static TaskHandle_t gBlitTask = nullptr;
static esp_timer_handle_t gBlitTimer = nullptr;
static int gBlitMarginUs = 250;

static void blitTimerCb(void*) { xTaskNotifyGive(gBlitTask); }

static inline void blitSleepUntil(int64_t deadlineUs) {
#if BLIT_SLEEP
  if (!gBlitTimer) return;
  const int64_t d = deadlineUs - (int64_t)gBlitMarginUs - esp_timer_get_time();
  if (d > 150 && esp_timer_start_once(gBlitTimer, (uint64_t)d) == ESP_OK)
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
#else
  (void)deadlineUs;
#endif
}

// Escala 2x. Lê 2 pixels por vez (32 bits) e grava as duas linhas de saída no mesmo laço.
static void HOT blitFrame(const uint16_t* src) {
  tft.startWrite();
  tft.setAddrWindow(0, 0, SCR_W, SCR_H);
  int cur = 0;
  int64_t tPrev = 0, usPrev = 0;     // quando o DMA anterior começou / quanto ele deve durar
  for (int y0 = 0; y0 < HR_H; y0 += CHUNK_HR) {
    const int rows = std::min(CHUNK_HR, HR_H - y0);
    uint32_t* out = (uint32_t*)dmaBuf[cur];
    for (int r = 0; r < rows; ++r) {
      const uint32_t* s2 = (const uint32_t*)(src + (size_t)(y0 + r) * HR_W);   // HR_W par => alinhado a 4 bytes
      uint32_t* l0 = out + (size_t)(2*r) * HR_W;
      uint32_t* l1 = l0 + HR_W;
      for (int x = 0; x < HR_W / 2; ++x) {
        const uint32_t p = s2[x];
        const uint32_t a = p & 0xFFFFu, b = p >> 16;
        const uint32_t da = a | (a << 16), db = b | (b << 16);
        l0[2*x] = da; l0[2*x+1] = db;
        l1[2*x] = da; l1[2*x+1] = db;
      }
    }
    if (tPrev) blitSleepUntil(tPrev + usPrev);       // dorme enquanto o DMA do bloco anterior corre
    const int64_t t1 = esp_timer_get_time();
    const int pixels = rows * 2 * SCR_W;
    tft.writePixelsDMA((const lgfx::swap565_t*)dmaBuf[cur], pixels);   // (espera o resto do bloco anterior)
    const int64_t t2 = esp_timer_get_time();
    if (tPrev) {                                      // auto-ajuste da margem de despertar
      const int64_t blocked = t2 - t1;
      if (blocked < 40)       { if (gBlitMarginUs < 1200) gBlitMarginUs += 100; }   // acordou tarde: SPI ficou ocioso
      else if (blocked > 450) { if (gBlitMarginUs > 120)  gBlitMarginUs -= 30;  }   // acordou cedo demais: girando à toa
    }
    tPrev = t2;
    usPrev = ((int64_t)pixels * 16 * 1000000) / SPI_WRITE_HZ;      // 16 bits por pixel
    cur ^= 1;
  }
  if (tPrev) blitSleepUntil(tPrev + usPrev);          // idem pro último bloco; o waitDMA() do chamador gira o resto
}

// ---- overlay de FPS desenhado NO quadro (fonte 3x5), em vez de tft.print() entre quadros ----
// tft.print() depois de cada quadro abria uma transação SPI própria (janela + 1 DMA/glifo): ~0,3-0,5 ms de barramento
// parado por quadro, justo no gargalo (o SPI), e o texto piscava porque o quadro seguinte o sobrescrevia.
static const uint8_t GLY[16][5] = {
  {7,5,5,5,7},{2,6,2,2,7},{7,1,7,4,7},{7,1,7,1,7},{5,5,7,1,1},{7,4,7,1,7},{7,4,7,5,7},{7,1,1,1,1},{7,5,7,5,7},{7,5,7,1,7},  // 0-9
  {7,4,6,4,4},{7,5,7,4,4},{3,4,2,1,6},{0,2,0,2,0},{0,0,0,0,2},{0,0,0,0,0}                                                     // F P S : . ' '
};
static void drawFpsOverlay(uint16_t* buf, const char* s) {
  int n = 0;
  while (s[n] && n < 14) ++n;
  if (n == 0 || (n * 4 + 3) > HR_W || HR_H < 9) return;
  for (int y = 0; y < 7; ++y) memset(buf + (size_t)(1 + y) * HR_W + 1, 0, (size_t)(n * 4 + 1) * 2);   // fundo preto
  int px = 2;
  for (int i = 0; i < n; ++i, px += 4) {
    const char ch = s[i];
    const int g = (ch >= '0' && ch <= '9') ? ch - '0' : ch == 'F' ? 10 : ch == 'P' ? 11 : ch == 'S' ? 12 : ch == ':' ? 13 : ch == '.' ? 14 : 15;
    for (int r = 0; r < 5; ++r) {
      const unsigned bits = GLY[g][r];
      uint16_t* row = buf + (size_t)(2 + r) * HR_W + px;
      if (bits & 4) row[0] = 0xFFFF;
      if (bits & 2) row[1] = 0xFFFF;
      if (bits & 1) row[2] = 0xFFFF;
    }
  }
}

static void blitTask(void*) {
  gBlitTask = xTaskGetCurrentTaskHandle();
#if BLIT_SLEEP
  esp_timer_create_args_t ta = {};
  ta.callback = blitTimerCb;
  ta.name = "blit";
  if (esp_timer_create(&ta, &gBlitTimer) != ESP_OK) gBlitTimer = nullptr;   // sem timer => volta ao busy-wait
#endif
  uint32_t frames = 0;
  unsigned long t0 = millis();
  for (;;) {
    int idx;
    xQueueReceive(fullQ, &idx, portMAX_DELAY);
    drawFpsOverlay(colorBuf[idx], gFpsTxt);      // o buffer é nosso até devolvermos em freeQ
    blitFrame(colorBuf[idx]);
    xQueueSend(freeQ, &idx, portMAX_DELAY);
    tft.waitDMA();
    tft.endWrite();

    frames++;
    unsigned long now = millis();
    if (now - t0 >= 1000) {
      gShownFps = frames * 1000.0f / (now - t0);
      snprintf(gFpsTxt, sizeof(gFpsTxt), "FPS:%.1f", (double)gShownFps);   // formata 1x por segundo, não por quadro
      frames = 0; t0 = now;
    }

    // o IDLE0 já ganha CPU quando o blit dorme no DMA; 1 tick por segundo basta p/ o watchdog (antes: 1 ms a cada 8 quadros)
    if ((frames & 31) == 0) vTaskDelay(1);
  }
}

// ---------------- SD ----------------
static bool loadOne(Model &mm, const char* path) {
  const bool r = mm.loadRzobj(path);
  if (!r) errf("FALHOU ao carregar %s", path);
  return r;
}

static bool loadAllModels() {
  pinMode(TFT_CS_PIN, OUTPUT); digitalWrite(TFT_CS_PIN, HIGH);
  pinMode(SD_CS, OUTPUT);      digitalWrite(SD_CS, HIGH);

  sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  bool ok = SD.begin(SD_CS, sdSPI, SD_FREQ);
  if (!ok) {
    errf("SD nao montou (cartao? fiacao CS=%d? FAT32?)", SD_CS);
  } else {
    Serial.printf("[SD] ok, %.1f MB\n", SD.cardSize() / 1048576.0);
    ok = loadOne(m, FILE_GANON);
    ok = loadOne(m2, FILE_LINK) && ok;
#if DRAW_SKY
    ok = loadOne(sky, FILE_SKY) && ok;
#endif
    SD.end();
  }
  sdSPI.end();
  return ok;
}

// ---------------- setup / loop ----------------
void setup() {
  Serial.begin(115200);
  initInvTable();                    // tabela 1/n usada pelos trechos de textura

  bool modelsOk = loadAllModels();   // antes do tft.begin()

  tft.begin();
  tft.setRotation(1);
  tft.setBrightness(255);
  tft.fillScreen(TFT_BLACK);

  if (!modelsOk) {
    tft.fillScreen(TFT_BLACK);
    tft.setTextWrap(true);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.setTextSize(2);
    tft.setCursor(4, 4);
    tft.print("ERRO SD / .rzobj");
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(1);
    tft.setCursor(4, 32);
    tft.print(gErr);
    while (1) delay(1000);
  }

  SCR_W = tft.width();
  SCR_H = tft.height();
  HR_W = SCR_W / RENDER_DIV;
  HR_H = SCR_H / RENDER_DIV;

  Serial.println("Alocando buffers:");
  const size_t total = (size_t)HR_W * HR_H;
  const size_t dmaBytes = (size_t)CHUNK_HR * 2 * SCR_W * 2;
  dmaBuf[0] = (uint16_t*)heap_caps_malloc(dmaBytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  dmaBuf[1] = (uint16_t*)heap_caps_malloc(dmaBytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  Serial.printf("  dma x2     %7u B -> RAM interna (DMA)\n", (unsigned)dmaBytes);

  // Ordem = prioridade pra RAM interna: z-buffer (mais acessado), depois as cores.
  // (o scratch de projeção agora é estático e minúsculo: 2 x ML_MAXV vértices)
  zBuf        = (uint16_t*)allocFast(total * 2, "zbuffer", true);
  colorBuf[0] = (uint16_t*)allocFast(total * 2, "color A", true);
  colorBuf[1] = (uint16_t*)allocFast(total * 2, "color B", COLORB_INTERNAL != 0);

  m.position = {2.0f, -1.8f, 0.0f};
  m.rotation = {0.0f, 0.0f, 0.0f};
  m.scale    = {0.028f, 0.028f, 0.028f};
  m.cull     = CULL_POS_AREA;

  m2.position = {0.0f, -1.4f, 0.0f};
  m2.rotation = {0.0f, 0.0f, 0.0f};
  m2.scale    = {0.25f, 0.25f, 0.25f};
  m2.cull     = CULL_POS_AREA;
  m2.playAnim(1, true, 1.0f, 0.2f);

  sky.position = {-KOKIRI_CX * KOKIRI_SCALE, KOKIRI_GROUND_Y, -KOKIRI_CZ * KOKIRI_SCALE};
  sky.rotation = {0.0f, 0.0f, 0.0f};
  sky.scale    = {KOKIRI_SCALE, KOKIRI_SCALE, KOKIRI_SCALE};
  sky.cull     = CULL_POS_AREA;

  if (!zBuf || !colorBuf[0] || !colorBuf[1] || !dmaBuf[0] || !dmaBuf[1]) {
    tft.fillScreen(TFT_BLACK);
    tft.setTextWrap(true);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.setCursor(4, 4); tft.setTextSize(2);
    tft.print("ERRO: alloc de buffers");
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(1);
    tft.setCursor(4, 32);
    tft.printf("zBuf:%d colorA:%d colorB:%d dmaA:%d dmaB:%d\n(1 = ok, 0 = falhou)\n%s",
               zBuf != nullptr, colorBuf[0] != nullptr, colorBuf[1] != nullptr,
               dmaBuf[0] != nullptr, dmaBuf[1] != nullptr, gErr);
    while (1) delay(1000);
  }

  freeQ = xQueueCreate(2, sizeof(int));
  fullQ = xQueueCreate(2, sizeof(int));
  for (int i = 0; i < 2; ++i) xQueueSend(freeQ, &i, 0);

  gMasterTask = xTaskGetCurrentTaskHandle();
  xTaskCreatePinnedToCore(blitTask, "blit", 5120, nullptr, 2, nullptr, 0);
#if RENDER_DUALCORE
  // prioridade 1 (< blit): quando o blit precisa do núcleo 0 ele passa na frente; o loop() pega as faixas restantes
  xTaskCreatePinnedToCore(rasterWorker, "raster1", 6144, nullptr, 1, &gWorkerTask, 0);
#endif

  // sobras de RAM interna -> arrays por quadro de cada modelo, depois texturas (o céu é o que mais pesa)
  Serial.println("Promovendo dados quentes pra RAM interna:");
  m.promoteFrameData(); m2.promoteFrameData();
#if DRAW_SKY
  sky.promoteFrameData();
  sky.promoteTextures();
#endif
  m.promoteTextures(); m2.promoteTextures();

  gLastUs = esp_timer_get_time();
  lastTime = micros();
  lastPrint = millis();

  Serial.println("========= SYSTEM INFO =========");
  Serial.printf("Heap RAM: %.2f MB livre / %.2f MB total\n", ESP.getFreeHeap() / 1048576.0, ESP.getHeapSize() / 1048576.0);
  if (psramFound())
    Serial.printf("PSRAM: %.2f MB livre / %.2f MB total\n", ESP.getFreePsram() / 1048576.0, ESP.getPsramSize() / 1048576.0);
  else
    Serial.println("PSRAM nao detectada!");
  Serial.printf("CPU freq: %d MHz | render interno %dx%d -> tela %dx%d | %d faixas | %s\n",
                getCpuFrequencyMhz(), HR_W, HR_H, SCR_W, SCR_H, RENDER_BANDS,
                RENDER_DUALCORE ? "2 nucleos" : "1 nucleo");
  Serial.println("================================");
}

void loop() {
  int idx;
  const uint32_t tw0 = micros();
  xQueueReceive(freeQ, &idx, portMAX_DELAY);
  waitUsAcc += micros() - tw0;       // tempo parado esperando o blit liberar um buffer => se for alto, o gargalo é o SPI

  // deltaTime: tempo real desde o início do quadro anterior (inclui render + espera pelo SPI)
  const int64_t nowUs = esp_timer_get_time();
  float d = (float)(nowUs - gLastUs) * 1e-6f;
  gLastUs = nowUs;
  if (d < DT_MIN) d = DT_MIN;
  if (d > DT_MAX) d = DT_MAX;
  dt = d;

  // animações: avançam o relógio e refazem pose/vértices só dos modelos que estão animando (fora do quadro => sem lock)
  m.updateAnim(dt);
  m2.updateAnim(dt);

  // ORBITATE: ângulo += velocidade * dt  => mesma velocidade angular com 10 ou 30 FPS
  // (o camera.move() antigo era no-op: orbitate() recalcula a posição a partir do target)
  camera.rotate(ORBIT_SPEED * dt, 0.0f);
  if (camera.yaw > 6.28318531f) camera.yaw -= 6.28318531f;   // só entra em sin/cos: dá p/ dar a volta e manter a precisão do float
  camera.orbitate(ORBIT_RADIUS);

  uint32_t t0 = micros();
  renderScene(colorBuf[idx]);
  renderUsAcc += micros() - t0;
  renderFrames++;

  xQueueSend(fullQ, &idx, portMAX_DELAY);

  if (millis() - lastPrint >= 1000) {
    const float nf = renderFrames ? (float)renderFrames : 1.0f;
    // render = prepare (serial, 1 núcleo) + raster (2 núcleos). espera = tempo esperando o SPI.
    // Se "espera" ~ 0 => limitado pela CPU (otimize o raster). Se "espera" grande e FPS ~ 30 => limitado pelo SPI (teto ~32 FPS a 80 MHz).
    Serial.printf("tela: %.1f FPS | render: %.1f ms (prepare %.2f) | espera SPI: %.1f ms | margem blit: %d us\n",
                  gShownFps, renderUsAcc / 1000.0f / nf, prepUsAcc / 1000.0f / nf, waitUsAcc / 1000.0f / nf, gBlitMarginUs);
    renderUsAcc = 0; renderFrames = 0; prepUsAcc = 0; waitUsAcc = 0; lastPrint = millis();
  }
}
