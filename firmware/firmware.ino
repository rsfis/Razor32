#include <Arduino.h>
#include <esp_heap_caps.h>
#include <esp_task_wdt.h>
#include <LovyanGFX.hpp>
#include <SD.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <algorithm>
#include <NimBLEDevice.h>
#include "time.h"
#include <vector>
#include <string>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <esp_timer.h>
#include <atomic>
#include <cstring>
#include <stdarg.h>
#include <stdio.h>
#include <math.h>
#include <new>

// O Arduino compila em -Os (tamanho). Os laços de pixel (formas, fontes, cópia do quadro) querem velocidade.
#pragma GCC optimize ("O3")

//Keys Pins (matriz 3 linhas x 10 colunas)
//Y1   8
//Y2   21
//Y3   41
//X1   48
//X2   45
//X3   17
//X4   16
//X5   40
//X6   18
//X7   42
//X8   39
//X9   47
//X10  7

#define KERNEL_VERSION "ALPHA 0.2.9"

#define LUA_HEAP_LIMIT (7 * 1024 * 1024)
#define LUA_TASK_STACK 49152

#define SD_CS 4

// KEYBOARD MATRIX SIZE (#define, e não const int, pra já existir quando a IDE gerar os protótipos das funções)
#define KB_ROWS 3
#define KB_COLS 10

#define LUA_SPRITE_DEFINITION "Display.Sprite"
#define LUA_CLEAR_SCREEN_DEFINITION "Display_ClearScreen"
#define LUA_UPDATE_SCREEN_DEFINITION "Display_UpdateScreen"
#define LUA_GET_FREE_MEMORY_DEFINITION "OS_GetFreeMemory"
#define LUA_GET_MEMORY_SIZE_DEFINITION "OS_GetMemorySize"
#define LUA_GET_FREE_HEAP_MEMORY_DEFINITION "OS_GetFreeHeapMemory"
#define LUA_GET_HEAP_MEMORY_SIZE_DEFINITION "OS_GetHeapMemorySize"
#define LUA_GET_CPU_TEMPERATURE_DEFINITION "OS_GetCPUTemperature"
#define LUA_GET_TIME_DEFINITION "OS_GetTime"
#define LUA_GET_LUA_HEAP_USAGE_DEFINITION "OS_GetLuaVMHeapMemoryUsage"
#define LUA_GET_USER_USERNAME_DEFINITION "OS_GetUserUsername"
#define LUA_GET_USER_PASSWORD_DEFINITION "OS_GetUserPassword"
#define LUA_OS_SLEEP_DEFINITION "OS_Sleep"
#define LUA_OS_RESTART_DEFINITION "OS_Restart"
#define LUA_OS_GET_KERNEL_VERSION_DEFINITION "OS_GetKernelVersion"
#define LUA_OS_SAVE_BIOS_CONFIGURATION_DEFINITION "OS_SaveBiosConfig"
#define LUA_OS_SAVE_SYSTEM_CONFIGURATION_DEFINITION "OS_SaveSystemConfig"
#define LUA_OS_SAVE_USER_CONFIGURATION_DEFINITION "OS_SaveUserConfig"
#define LUA_OS_SET_BIOS_CONFIG_INTEGER_KEY "OS_SetBiosConfigIntegerKey"
#define LUA_OS_SET_BIOS_CONFIG_STRING_KEY "OS_SetBiosConfigStringKey"
#define LUA_OS_SET_SYSTEM_CONFIG_INTEGER_KEY "OS_SetSystemConfigIntegerKey"
#define LUA_OS_SET_SYSTEM_CONFIG_STRING_KEY "OS_SetSystemConfigStringKey"
#define LUA_OS_SET_USER_CONFIG_INTEGER_KEY "OS_SetUserConfigIntegerKey"
#define LUA_OS_SET_USER_CONFIG_STRING_KEY "OS_SetUserConfigStringKey"
#define LUA_OS_SET_KEYBOARD_DELAY_DEFINITION "OS_SetKeyboardDelay"
#define LUA_OS_GET_PRESSED_KEY_DEFINITION "OS_GetPressedKey"
#define LUA_OS_IS_KEY_PRESSED_DEFINITION "OS_IsKeyPressed"
#define LUA_OS_HAS_MOUSE_CONNECTED_DEFINITION "OS_HasMouseConnected"
#define LUA_OS_GET_MOUSE_POSITION_DEFINITION "OS_GetMousePosition"
#define LUA_OS_IS_MOUSE1_PRESSED_DEFINITION "OS_IsMouse1Pressed"
#define LUA_OS_IS_MOUSE2_PRESSED_DEFINITION "OS_IsMouse2Pressed"
#define LUA_OS_GET_MOUSE_SCROLL_DELTA_DEFINITION "OS_GetMouseScrollDelta"
#define LUA_OS_UPDATE_MOUSE_TASK_DEFINITION "OS_UpdateMouseTask"
#define LUA_OS_IS_WIFI_ENABLED_DEFINITION "OS_IsWifiEnabled"
#define LUA_OS_IS_BLUETOOTH_ENABLED_DEFINITION "OS_IsBluetoothEnabled"
#define LUA_OS_ENABLE_WIFI_DEFINITION "OS_EnableWifi"
#define LUA_OS_DISABLE_WIFI_DEFINITION "OS_DisableWifi"
#define LUA_OS_ENABLE_BLUETOOTH_DEFINITION "OS_EnableBluetooth"
#define LUA_OS_GET_CONNECTED_BLUETOOTH_NAME_DEFINITION "OS_GetConnectedBluetoothName"
#define LUA_OS_DISABLE_BLUETOOTH_DEFINITION "OS_DisableBluetooth"
#define LUA_DISPLAY_DRAW_RECT_DEFINITION "Display_DrawRect"
#define LUA_DISPLAY_DRAW_ELIPSE_DEFINITION "Display_DrawElipse"
#define LUA_DISPLAY_DRAW_ARC_DEFINITION "Display_DrawArc"
#define LUA_OS_GET_FOUND_WIFI_NETWORKS_DEFINITION "OS_GetFoundWifiNetworks"
#define LUA_OS_FORGET_WIFI_DEFINITION "OS_ForgetWifi"
#define LUA_OS_CONNECT_WIFI_DEFINITION "OS_ConnectWifi"
#define LUA_OS_GET_WIFI_STATUS_DEFINITION "OS_GetWifiStatus"
#define LUA_OS_GET_CONNECTED_WIFI_SSID_DEFINITION "OS_GetConnectedWifiSSID"
#define LUA_OS_IS_WIFI_SCANNING_DEFINITION "OS_IsWifiScanning"
#define LUA_DISPLAY_DRAW_POLIGON_DEFINITION "Display_DrawPoligon"
#define LUA_DISPLAY_DRAW_LINE_DEFINITION "Display_DrawLine"

// ===== WIFI / BLUETOOTH TOGGLE =====
// Nomes das teclas no kbLayout (tratadas só pelo kernel, o Lua nunca recebe elas)
#define KEY_WIFI_TOGGLE "WifiToggle"
#define KEY_BLUETOOTH_TOGGLE "BluetoothToggle"
#define TOGGLE_KEY_COOLDOWN_MS 2000   // timer de utilização das duas teclas
#define KERNEL_KEY_DEBUG 0            // 1 = imprime no Serial as teclas que o kernel vê (pra diagnóstico)
#define TOAST_DURATION_MS 1500        // tempo da notificação rápida na tela

// ===== DISPLAY (mesmas técnicas de tela do 3d_models) =====
#ifndef SPI_WRITE_HZ
#define SPI_WRITE_HZ 80000000
#endif
#ifndef ASYNC_PRESENT
#define ASYNC_PRESENT 1     // 1 = tarefa de blit no núcleo 0 + 2 buffers de quadro (Lua desenha o quadro N+1 enquanto o N vai pelo SPI)
#endif                      // 0 = 1 buffer; o próprio Display_UpdateScreen envia (ainda por DMA em RAM interna, dormindo durante o DMA)
#ifndef BLIT_SLEEP
#define BLIT_SLEEP 1        // 1 = dorme num timer de alta resolução enquanto o DMA corre (em vez de busy-wait)
#endif
#define DMA_CHUNK_PIXELS (480 * 16)   // pixels por bloco de DMA (2 buffers em RAM interna: ~30 KB)

const char* ntpServer = "pool.ntp.org";
long gmtOffset_sec = -3 * 3600;  // UTC-3
const int daylightOffset_sec = 0;

StaticJsonDocument<1024> BiosConfiguration;
StaticJsonDocument<1024> SystemConfiguration;
StaticJsonDocument<1024> UserConfiguration;

struct Sprite;

// Estilo das formas básicas (Display_DrawRect / Display_DrawElipse).
// Definido aqui no topo pra já existir quando a IDE gerar os protótipos das funções.
struct ShapeStyle {
  bool hasFill = false;
  bool hasBorder = false;
  uint16_t fill = 0;
  uint16_t border = 0;
  int bw = 0;            // espessura da borda (px)
};

#define SHAPE_DEG2RAD 0.017453292f
#define SHAPE_POLY_MAX 64          // máximo de vértices em Display_DrawPoligon
#define SHAPE_JOIN_MITER 0         // quina pontuda (vira bevel se o ângulo for muito fechado)
#define SHAPE_JOIN_BEVEL 1         // quina cortada
#define SHAPE_JOIN_ROUND 2         // quina arredondada


// ===== R3D types early (Arduino auto-prototypes all functions before later structs) =====
struct Vec3 { float x, y, z; };
struct Mat4 { float m[4][4]; };
struct Model;           // defined later in R3D engine
struct MeshletBuild;    // defined later
struct RzMesh;          // packed mesh header from .rzobj
struct LuaModel3D;      // Lua userdata wrapper


// ===== KERNEL SERVICES (forward declarations) =====
void kernelLuaCheckpoint();                 // ponto onde a task Lua pode ser pausada pelo kernel
void kernelShowToast(const char* msg);
bool kernelAskConnectBluetooth(const char* name, const char* addr);
volatile bool toastActive = false;
static void r3dReleaseBuf(int idx);                                   // devolve um buffer de cor 3D ao pool (definida na seção R3D)
static const uint16_t* r3dLayerPtr(int idx, int* w, int* h);          // pixels + tamanho de um buffer de cor 3D

// ===== Lua headers =====
extern "C" {
  #include "lua.h"
  #include "lauxlib.h"
  #include "lualib.h"
}

class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ST7796 _panel_instance;
  lgfx::Bus_SPI _bus_instance;
  lgfx::Light_PWM _light_instance;

public:
  LGFX(void)
  {
    { // SPI
      auto cfg = _bus_instance.config();
      cfg.spi_host = SPI2_HOST;
      cfg.spi_mode = 0;
      cfg.freq_write = SPI_WRITE_HZ;
      cfg.freq_read  = 20000000;
      cfg.spi_3wire  = false;
      cfg.use_lock   = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk = 14;
      cfg.pin_mosi = 13;
      cfg.pin_miso = 12;
      cfg.pin_dc   = 9;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }

    { // Painel ST7796
      auto cfg = _panel_instance.config();
      cfg.pin_cs  = 10;
      cfg.pin_rst = 11;
      cfg.pin_busy = -1;

      cfg.memory_width  = 320;
      cfg.memory_height = 480;
      cfg.panel_width   = 320;
      cfg.panel_height  = 480;

      cfg.offset_x = 0;
      cfg.offset_y = 0;

      cfg.readable   = false;
      cfg.invert     = true;    // ⚠️ deixar true
      cfg.rgb_order  = false;   // ⚠️ força modo BGR (a maioria dessas telas usa)
      cfg.dlen_16bit = false;
      cfg.bus_shared = true;

      _panel_instance.config(cfg);
    }

    { // Backlight
      auto cfg = _light_instance.config();
      cfg.pin_bl = 38;
      cfg.invert = false;
      cfg.freq   = 10000;
      cfg.pwm_channel = 7;
      _light_instance.config(cfg);
      _panel_instance.setLight(&_light_instance);
    }

    setPanel(&_panel_instance);
  }
};

LGFX tft;

// Dois buffers de quadro (PSRAM): o Lua desenha em um enquanto o outro vai pelo SPI (ver DISPLAY ENGINE).
// 'frame' continua sendo o nome usado no código todo: aponta sempre pro buffer em que o Lua está desenhando agora.
static LGFX_Sprite frameBuf0(&tft);
static LGFX_Sprite frameBuf1(&tft);
static LGFX_Sprite* const frameBufs[2] = { &frameBuf0, &frameBuf1 };
static LGFX_Sprite* gDrawFrame = &frameBuf0;
static int gDrawIdx = 0;
#define frame (*gDrawFrame)

// ===== DIRTY RECT TRACKING =====
// Em vez de limpar/enviar a tela inteira a cada ciclo Display_ClearScreen()/Display_UpdateScreen(), rastreamos
// as regioes que efetivamente foram desenhadas (Sprite:draw(), Font:drawString(), formas) entre um clear e o
// update seguinte, e so tocamos essas regioes na PSRAM e no SPI.
// ANTES: um unico retangulo (uniao de tudo). Dois elementos longe um do outro (ex.: barra de status + icone no
// canto oposto) faziam a UNIAO cobrir quase a tela inteira. AGORA: uma pequena lista de ate DIRTY_MAX retangulos;
// dois retangulos so sao fundidos quando o retangulo da uniao custa quase o mesmo que os dois separados.
#define DIRTY_MAX 6
#define DIRTY_MERGE_SLACK 512   // pixels "de graca" tolerados ao fundir (~ custo fixo de mais uma janela + DMA)

struct IRect { int16_t x, y, w, h; };
struct DirtyList {
  IRect r[DIRTY_MAX];
  int n = 0;
};

DirtyList frameDirtyAccum;  // o que foi desenhado NESTE frame (entre clear e update)
DirtyList frameDirtyPrev;   // o que foi desenhado (e portanto ficou visivel) no frame anterior
bool frameEverPushed = false;

void clampRectToScreen(int &x, int &y, int &w, int &h) {
  if (x < 0) { w += x; x = 0; }
  if (y < 0) { h += y; y = 0; }
  if (x + w > tft.width())  w = tft.width()  - x;
  if (y + h > tft.height()) h = tft.height() - y;
  if (w < 0) w = 0;
  if (h < 0) h = 0;
}

static inline int irArea(const IRect& a) { return (int)a.w * (int)a.h; }

static inline IRect irUnion(const IRect& a, const IRect& b) {
  const int x1 = min((int)a.x, (int)b.x), y1 = min((int)a.y, (int)b.y);
  const int x2 = max((int)a.x + a.w, (int)b.x + b.w), y2 = max((int)a.y + a.h, (int)b.y + b.h);
  IRect r;
  r.x = (int16_t)x1; r.y = (int16_t)y1; r.w = (int16_t)(x2 - x1); r.h = (int16_t)(y2 - y1);
  return r;
}

// Adiciona um retangulo (ja recortado na tela) a lista, fundindo com os existentes quando compensa.
static void dlAdd(DirtyList& d, IRect nr) {
  for (;;) {                                   // funde em cascata: o retangulo cresce e pode alcancar outros
    int hit = -1;
    IRect u = nr;
    for (int i = 0; i < d.n; i++) {
      u = irUnion(d.r[i], nr);
      if (irArea(u) <= irArea(d.r[i]) + irArea(nr) + DIRTY_MERGE_SLACK) { hit = i; break; }
    }
    if (hit < 0) break;
    nr = u;
    d.r[hit] = d.r[--d.n];                     // remove o absorvido
  }
  if (d.n < DIRTY_MAX) { d.r[d.n++] = nr; return; }

  // Lista cheia: funde o par (contando o novo) que menos pixels extras gera
  IRect t[DIRTY_MAX + 1];
  for (int i = 0; i < d.n; i++) t[i] = d.r[i];
  t[d.n] = nr;
  const int n = d.n + 1;
  int bi = 0, bj = 1, bw = 0x7FFFFFFF;
  for (int i = 0; i < n; i++) {
    for (int j = i + 1; j < n; j++) {
      const int waste = irArea(irUnion(t[i], t[j])) - irArea(t[i]) - irArea(t[j]);
      if (waste < bw) { bw = waste; bi = i; bj = j; }
    }
  }
  t[bi] = irUnion(t[bi], t[bj]);
  t[bj] = t[n - 1];
  d.n = n - 1;
  for (int i = 0; i < d.n; i++) d.r[i] = t[i];
}

void markDirty(int x, int y, int w, int h) {
  if (w <= 0 || h <= 0) return;
  clampRectToScreen(x, y, w, h);               // a lista so guarda retangulos dentro da tela
  if (w <= 0 || h <= 0) return;
  IRect r;
  r.x = (int16_t)x; r.y = (int16_t)y; r.w = (int16_t)w; r.h = (int16_t)h;
  dlAdd(frameDirtyAccum, r);
}

// ---- COMANDOS DE DESENHO: hash + retangulo de cada um, comparados quadro a quadro ----
// Cada Sprite:draw / Font:drawString / Display_Draw* registra {hash dos argumentos, retangulo}. O quadro e uma funcao
// deterministica da lista de comandos; se o comando i e igual ao do quadro anterior, so os pixels dentro do retangulo
// dos comandos que MUDARAM (antigo + novo) podem ter mudado. Entao o que vai pro SPI e so isso (um wallpaper
// redesenhado todo quadro nao custa mais nada no barramento). Qualquer duvida => cai na regra antiga (anterior + atual).
static inline uint32_t hmix(uint32_t h, uint32_t v) {
  h ^= v; h *= 0x85EBCA6Bu; h ^= h >> 15; h *= 0xC2B2AE35u; h ^= h >> 13;
  return h;
}
static inline uint32_t hmixf(uint32_t h, float f) { uint32_t u; memcpy(&u, &f, 4); return hmix(h, u); }
static uint32_t hmixStr(uint32_t h, const char* t) {
  while (*t) h = hmix(h, (uint8_t)*t++);
  return hmix(h, 0xFFu);
}

#define CMD_MAX 256
struct CmdRec { uint32_t h; IRect r; };
static CmdRec gCmdBufA[CMD_MAX], gCmdBufB[CMD_MAX];
static CmdRec* gCmdCur = gCmdBufA;       // comandos do quadro em desenho
static CmdRec* gCmdPrev = gCmdBufB;      // comandos do quadro que o painel mostra
static int gCmdN = 0, gCmdPrevN = 0;
static bool gCmdOverflow = false;        // estourou CMD_MAX neste quadro => sem diff
static bool gCmdPrevValid = false;       // painel == render(gCmdPrev)
static bool gClearedSinceUpdate = false; // o app chamou Display_ClearScreen neste quadro (sem clear nao ha garantia de diff)

static void cmdRecord(uint32_t hash, int x, int y, int w, int h) {
  IRect r; r.x = 0; r.y = 0; r.w = 0; r.h = 0;
  if (w > 0 && h > 0) {
    clampRectToScreen(x, y, w, h);
    if (w > 0 && h > 0) { r.x = (int16_t)x; r.y = (int16_t)y; r.w = (int16_t)w; r.h = (int16_t)h; }
  }
  if (gCmdN < CMD_MAX) { gCmdCur[gCmdN].h = hash; gCmdCur[gCmdN].r = r; gCmdN++; }
  else gCmdOverflow = true;
  if (r.w > 0) dlAdd(frameDirtyAccum, r);
}

// ---- LIMPEZA PREGUICOSA ----
// Display_ClearScreen so anota o que precisa voltar pra preto. O memset acontece no 1o desenho que nao cobre tudo
// (ou no Display_UpdateScreen). Um sprite opaco que cobre a regiao (wallpaper) torna o memset inutil.
static DirtyList gPendClear;

static void clearRectBlack(const IRect& r) {
  uint16_t* b = (uint16_t*)frame.getBuffer();
  if (!b) return;
  const int fw = (int)frame.width();
  if ((int)r.w == fw) {
    memset(b + (size_t)r.y * fw, 0, (size_t)fw * r.h * 2);
  } else {
    for (int j = 0; j < r.h; ++j)
      memset(b + (size_t)(r.y + j) * fw + r.x, 0, (size_t)r.w * 2);
  }
}

static void flushClear() {
  for (int i = 0; i < gPendClear.n; i++) clearRectBlack(gPendClear.r[i]);
  gPendClear.n = 0;
}

// Chamar antes de um desenho OPACO em [x,x+w) x [y,y+h): se cobre todas as regioes pendentes, descarta a limpeza
static void flushClearUnless(int x, int y, int w, int h) {
  if (gPendClear.n == 0) return;
  clampRectToScreen(x, y, w, h);
  bool covered = (w > 0 && h > 0);
  for (int i = 0; covered && i < gPendClear.n; i++) {
    const IRect& r = gPendClear.r[i];
    if (!(r.x >= x && r.y >= y && r.x + r.w <= x + w && r.y + r.h <= y + h)) covered = false;
  }
  if (covered) gPendClear.n = 0; else flushClear();
}

static uint32_t gSpriteSerial = 0;

// ===== DISPLAY ENGINE =====
// Mesmas ideias do blit do 3d_models:
//  - o retângulo sujo do quadro (PSRAM) vai pro painel em BLOCOS de linhas copiados pra 2 buffers de DMA em RAM INTERNA
//    (alternados: enquanto um bloco corre pelo SPI, o próximo já está sendo copiado);
//  - enquanto o DMA corre, a tarefa DORME num timer de alta resolução em vez de girar em busy-wait e só gira o restinho
//    (a margem de despertar se auto-ajusta): sobra CPU pro Wi-Fi / BLE / kernel;
//  - ASYNC_PRESENT: 2 buffers de quadro + tarefa de blit no núcleo 0 (filas free/full);
//  - o toast é composto direto nos blocos de DMA (como o overlay de FPS do 3d_models): sem transação SPI extra nem flicker.
static const int TOAST_X = 140, TOAST_Y = 6, TOAST_W = 200, TOAST_H = 26;

struct PresentJob { int16_t idx, n, layer; IRect r[DIRTY_MAX]; };

static DirtyList bufDirty[2];              // por buffer de quadro: região desenhada NELE (fora dela ele é preto)
static bool gAsync = false;
static uint16_t* gDmaBuf[2] = { nullptr, nullptr };
static QueueHandle_t gFreeQ = nullptr, gFullQ = nullptr;
static SemaphoreHandle_t gPresentMutex = nullptr;   // serializa TODO acesso de escrita ao painel + os buffers de DMA
static std::atomic<int> gJobsPending(0);
// CAMADA 3D: Composite3DToFrame() não copia mais o 3D pro buffer 2D. Ele só "reivindica" o buffer de cor 3D como FUNDO do
// quadro (gLayerSrcIdx); o motor de tela amplia 2x direto nos blocos de DMA e põe o HUD (pixels não-pretos do buffer 2D,
// só dentro dos retângulos sujos) por cima. Sem a escrita de 307 KB na PSRAM + a releitura deles.
static int gLayerSrcIdx = -1;      // camada pendente do quadro em desenho (-1 = nenhuma)
static int gLayerPrevShown = 0;    // o último quadro enviado teve camada (o painel mostra 3D fora do HUD)
static int gLastLayerIdx = -1;     // camada do último quadro enviado (o kernel repinta o toast com ela)
static std::atomic<int> gR3DPending(0);            // quadros 3D enfileirados / ainda indo pelo SPI (tarefa de blit 3D)
static volatile int gLastShownIdx = -1;             // buffer do último quadro enviado (usado pra repintar toast/dialog)
static esp_timer_handle_t gBlitTimer = nullptr;
static volatile TaskHandle_t gBlitWaiter = nullptr;
static int gBlitMarginUs = 250;
static LGFX_Sprite toastSpr(&tft);                  // toast pré-renderizado (RAM interna), composto nos blocos de DMA
static uint16_t toastKey = 0;                       // valor bruto da cor transparente do toast
static bool gToastOk = false;

static void blitTimerCb(void*) {
  TaskHandle_t t = gBlitWaiter;
  if (t) xTaskNotifyGive(t);
}

static inline void blitSleepUntil(int64_t deadlineUs) {
#if BLIT_SLEEP
  if (!gBlitTimer) return;
  const int64_t d = deadlineUs - (int64_t)gBlitMarginUs - esp_timer_get_time();
  if (d > 150) {
    esp_timer_stop(gBlitTimer);        // garante timer parado e sem notificação velha pendente
    ulTaskNotifyTake(pdTRUE, 0);
    if (esp_timer_start_once(gBlitTimer, (uint64_t)d) == ESP_OK)
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  }
#else
  (void)deadlineUs;
#endif
}

// Sobrepõe o toast nas linhas [y0, y0+rows) do bloco (que cobre as colunas [x0, x0+w)). Pixel == toastKey = transparente.
static void compositeToast(uint16_t* buf, int x0, int y0, int w, int rows) {
  const int ty0 = max(y0, TOAST_Y), ty1 = min(y0 + rows, TOAST_Y + TOAST_H);
  const int tx0 = max(x0, TOAST_X), tx1 = min(x0 + w, TOAST_X + TOAST_W);
  if (ty1 <= ty0 || tx1 <= tx0) return;
  const uint16_t* ts = (const uint16_t*)toastSpr.getBuffer();
  if (!ts) return;
  for (int ty = ty0; ty < ty1; ++ty) {
    uint16_t* d = buf + (size_t)(ty - y0) * w + (tx0 - x0);
    const uint16_t* sp = ts + (size_t)(ty - TOAST_Y) * TOAST_W + (tx0 - TOAST_X);
    for (int i = 0; i < tx1 - tx0; ++i) if (sp[i] != toastKey) d[i] = sp[i];
  }
}

// Amplia 1 linha do 3D (n pixels) pra 2n pixels. PAIR=true grava as DUAS linhas de saída (d32 e e32) no mesmo laço:
// lê a linha 3D 1x e não precisa do memcpy da 1ª pra 2ª linha.
template<bool PAIR>
static inline void layerUpRow(uint32_t* __restrict d32, uint32_t* __restrict e32, const uint16_t* sp, int n) {
  int i = 0;
  if (((uintptr_t)sp & 3) == 0) {                         // lê 2 pixels 3D por vez
    const uint32_t* s32 = (const uint32_t*)sp;
    for (; i + 1 < n; i += 2) {
      const uint32_t p = *s32++;
      const uint32_t a = p & 0xFFFFu, b = p >> 16;
      const uint32_t da = a | (a << 16), db = b | (b << 16);
      d32[i] = da; d32[i + 1] = db;
      if (PAIR) { e32[i] = da; e32[i + 1] = db; }
    }
  }
  for (; i < n; ++i) {
    const uint32_t p = sp[i], dp = p | (p << 16);
    d32[i] = dp;
    if (PAIR) e32[i] = dp;
  }
}

// Fundo 3D (lw x lh, ampliado 2x) nas linhas de tela [y0, y0+rows) x colunas [x0, x0+w) do bloco 'out' (w colunas por linha).
static void layerBlock(uint16_t* out, const uint16_t* ls, int lw, int lh, int x0, int y0, int w, int rows) {
  const bool fast = ((x0 | w) & 1) == 0 && x0 + w <= 2 * lw;   // x e w pares: cada pixel 3D vira 1 palavra de 32 bits
  int r = 0;
  while (r < rows) {
    int sy = (y0 + r) >> 1;
    if (sy >= lh) sy = lh - 1;
    int sy1 = (y0 + r + 1) >> 1;
    if (sy1 >= lh) sy1 = lh - 1;
    const bool pair = (r + 1 < rows) && (sy1 == sy);          // as 2 linhas de saída vêm da mesma linha 3D
    uint16_t* d = out + (size_t)r * w;
    const uint16_t* sr = ls + (size_t)sy * lw;
    if (fast) {
      const uint16_t* sp = sr + (x0 >> 1);
      if (pair) layerUpRow<true>((uint32_t*)d, (uint32_t*)(d + w), sp, w >> 1);
      else      layerUpRow<false>((uint32_t*)d, nullptr, sp, w >> 1);
    } else {
      for (int i = 0; i < w; ++i) {
        int px = (x0 + i) >> 1;
        if (px >= lw) px = lw - 1;
        d[i] = sr[px];
      }
      if (pair) memcpy(d + w, d, (size_t)w * 2);
    }
    r += pair ? 2 : 1;
  }
}

// HUD sobre o fundo: pixels != 0 (preto = transparente, como nos sprites) do buffer 2D dentro do retângulo sujo 'h'.
// Lê 2 pixels por vez (o HUD é quase todo transparente e vem da PSRAM): par todo zero é pulado, par todo opaco vira 1 escrita de 32 bits.
static void hudOverlay(uint16_t* out, const uint16_t* fs, int stride, int x0, int y0, int w, int rows, const IRect& h) {
  const int ya = h.y > y0 ? (int)h.y : y0;
  const int yb = (h.y + h.h) < (y0 + rows) ? (int)(h.y + h.h) : (y0 + rows);
  const int xa = h.x > x0 ? (int)h.x : x0;
  const int xb = (h.x + h.w) < (x0 + w) ? (int)(h.x + h.w) : (x0 + w);
  if (yb <= ya || xb <= xa) return;
  const int n = xb - xa;
  for (int yy = ya; yy < yb; ++yy) {
    const uint16_t* sp = fs + (size_t)yy * stride + xa;
    uint16_t* d = out + (size_t)(yy - y0) * w + (xa - x0);
    int i = 0;
    if (((uintptr_t)sp & 2) && n > 0) { const uint16_t c = sp[0]; if (c) d[0] = c; i = 1; }
    for (; i + 1 < n; i += 2) {
      const uint32_t v = *(const uint32_t*)(sp + i);
      if (v == 0) continue;
      const uint16_t lo = (uint16_t)v, hi = (uint16_t)(v >> 16);
      if (lo && hi && (((uintptr_t)(d + i) & 2) == 0)) *(uint32_t*)(d + i) = v;
      else { if (lo) d[i] = lo; if (hi) d[i + 1] = hi; }
    }
    for (; i < n; ++i) { const uint16_t c = sp[i]; if (c) d[i] = c; }
  }
}

// O CHAMADOR PRECISA ESTAR COM gPresentMutex. Envia os retangulos rs[0..nr) do buffer 'idx' pro painel.
// O sprite 16 bpp guarda RGB565 com bytes trocados = formato do painel, entao e DMA direto (sem converter nada).
// Todos os retangulos vao na MESMA transacao e o pipeline (copia do proximo bloco enquanto o DMA corre) continua
// valendo na troca de retangulo: so a 1a janela de cada retangulo espera o DMA anterior acabar.
static void presentRects(int idx, const IRect* rs, int nr, int li) {
  if (nr <= 0 || !gDmaBuf[0] || !gDmaBuf[1]) return;
  gBlitWaiter = xTaskGetCurrentTaskHandle();

  const uint16_t* src = (const uint16_t*)frameBufs[idx]->getBuffer();
  const int stride = frameBufs[idx]->width();
  const bool toast = toastActive;
  const uint16_t* lsrc = nullptr;           // camada 3D de fundo (li >= 0): o buffer 2D só tem o HUD
  int lw = 0, lh = 0;
  if (li >= 0) lsrc = r3dLayerPtr(li, &lw, &lh);
  const DirtyList* hud = lsrc ? &bufDirty[idx] : nullptr;   // onde o HUD foi desenhado neste buffer

  tft.startWrite();
  int cur = 0;
  int64_t tPrev = 0, usPrev = 0;     // quando o DMA anterior comecou / quanto ele deve durar
  for (int k = 0; k < nr; ++k) {
    int x = rs[k].x, y = rs[k].y, w = rs[k].w, h = rs[k].h;
    clampRectToScreen(x, y, w, h);
    if (w <= 0 || h <= 0) continue;

    int rowsPer = DMA_CHUNK_PIXELS / w;
    if (rowsPer < 1) rowsPer = 1;
    const bool fullW = (w == stride);   // retangulo de largura total: as linhas sao contiguas na PSRAM

    for (int r0 = 0; r0 < h; r0 += rowsPer) {
      const int rows = min(rowsPer, h - r0);
      uint16_t* out = gDmaBuf[cur];
      const uint16_t* s = src + (size_t)(y + r0) * stride + x;
      if (lsrc) {
        layerBlock(out, lsrc, lw, lh, x, y + r0, w, rows);
        for (int k2 = 0; k2 < hud->n; ++k2) hudOverlay(out, src, stride, x, y + r0, w, rows, hud->r[k2]);
      } else if (fullW) {
        memcpy(out, s, (size_t)rows * w * 2);                 // 1 memcpy grande em vez de 1 por linha
      } else {
        for (int r = 0; r < rows; ++r)
          memcpy(out + (size_t)r * w, s + (size_t)r * stride, (size_t)w * 2);
      }
      if (toast) compositeToast(out, x, y + r0, w, rows);

      if (tPrev) blitSleepUntil(tPrev + usPrev);     // dorme enquanto o DMA do bloco anterior corre
      const int64_t t1 = esp_timer_get_time();
      if (r0 == 0) {                                 // 1o bloco do retangulo: abre a janela nova
        if (tPrev) tft.waitDMA();                    // a janela so pode mudar depois do DMA anterior terminar
        tft.setAddrWindow(x, y, w, h);
      }
      const int pixels = rows * w;
      tft.writePixelsDMA((const lgfx::swap565_t*)out, pixels);   // (espera o resto do bloco anterior)
      const int64_t t2 = esp_timer_get_time();
      if (tPrev && r0 > 0) {                         // auto-ajuste da margem de despertar (so blocos "normais")
        const int64_t blocked = t2 - t1;
        if (blocked < 40)       { if (gBlitMarginUs < 1200) gBlitMarginUs += 100; }   // acordou tarde: SPI ficou ocioso
        else if (blocked > 450) { if (gBlitMarginUs > 120)  gBlitMarginUs -= 30;  }   // acordou cedo: girando a toa
      }
      tPrev = t2;
      usPrev = ((int64_t)pixels * 16 * 1000000) / SPI_WRITE_HZ;   // 16 bits por pixel
      cur ^= 1;
    }
  }
  if (tPrev) blitSleepUntil(tPrev + usPrev);
  tft.waitDMA();
  tft.endWrite();
}

static inline void presentRect(int idx, int x, int y, int w, int h, int li) {
  IRect r;
  r.x = (int16_t)x; r.y = (int16_t)y; r.w = (int16_t)w; r.h = (int16_t)h;
  presentRects(idx, &r, 1, li);
}

// Espera a fila de blit esvaziar (o kernel usa antes de desenhar direto no painel)
static void displayWaitIdle() {
  const uint32_t t0 = millis();
  while ((gJobsPending.load() > 0 || gR3DPending.load() > 0) && (millis() - t0) < 300) vTaskDelay(1);
}

static void blitTask(void*) {
  for (;;) {
    PresentJob j;
    xQueueReceive(gFullQ, &j, portMAX_DELAY);
    xSemaphoreTake(gPresentMutex, portMAX_DELAY);
    presentRects(j.idx, j.r, j.n, j.layer);
    xSemaphoreGive(gPresentMutex);
    if (j.layer >= 0) r3dReleaseBuf(j.layer);   // o fundo 3D já foi lido: o buffer volta pro pool do 3D
    gJobsPending--;
    int idx = j.idx;
    xQueueSend(gFreeQ, &idx, portMAX_DELAY);   // devolve o buffer pro Lua
  }
}

// Chamada pelo Display_UpdateScreen: manda os retangulos do buffer atual e (modo async) pega o proximo buffer livre.
static void displaySubmit(const DirtyList& pl) {
  if (gR3DPending.load() > 0) {                    // quadro 3D ainda em voo: o HUD tem que ir DEPOIS dele
    const uint32_t t0 = millis();
    while (gR3DPending.load() > 0 && (millis() - t0) < 300) vTaskDelay(1);
  }
  const int idx = gDrawIdx;
  gLastShownIdx = idx;
  const int li = gLayerSrcIdx;                     // camada 3D deste quadro (a posse passa pro job)
  gLayerSrcIdx = -1;
  gLastLayerIdx = li;
  if (gAsync) {
    PresentJob j;
    j.idx = (int16_t)idx; j.n = (int16_t)pl.n; j.layer = (int16_t)li;
    for (int i = 0; i < pl.n; i++) j.r[i] = pl.r[i];
    gJobsPending++;
    xQueueSend(gFullQ, &j, portMAX_DELAY);
    int nxt = 0;
    xQueueReceive(gFreeQ, &nxt, portMAX_DELAY);   // se o blit esta atrasado, e aqui que o Lua espera o SPI
    gDrawIdx = nxt;
    gDrawFrame = frameBufs[nxt];
  } else {
    xSemaphoreTake(gPresentMutex, portMAX_DELAY);
    presentRects(idx, pl.r, pl.n, li);
    xSemaphoreGive(gPresentMutex);
    if (li >= 0) r3dReleaseBuf(li);
  }
}

// Liga o motor depois do boot (os sprites da tela de boot já foram liberados, sobra PSRAM pro 2º buffer de quadro).
static void displayEngineBegin() {
  const int W = tft.width(), H = tft.height();

  gDmaBuf[0] = (uint16_t*)heap_caps_malloc((size_t)DMA_CHUNK_PIXELS * 2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  gDmaBuf[1] = (uint16_t*)heap_caps_malloc((size_t)DMA_CHUNK_PIXELS * 2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  if (!gDmaBuf[0] || !gDmaBuf[1]) {
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1.5);
    tft.setCursor(5, 5);
    tft.print("ERR 0x013 - Couldn't allocate DMA buffers.");
    while (1);
  }

  gPresentMutex = xSemaphoreCreateMutex();

  // toast pré-renderizado: 200x26 em RAM interna
  toastSpr.setPsram(false);
  toastSpr.setColorDepth(16);
  gToastOk = (toastSpr.createSprite(TOAST_W, TOAST_H) != nullptr);
  if (!gToastOk) Serial.println("WARN: sem RAM interna pro toast (notificacoes desativadas)");

  gAsync = false;
#if ASYNC_PRESENT
  frameBuf1.setPsram(true);
  frameBuf1.setColorDepth(16);
  if (frameBuf1.createSprite(W, H)) gAsync = true;
  else Serial.println("WARN: sem PSRAM pro 2o buffer de quadro; apresentando em modo sincrono");
#endif

  frameBuf0.fillScreen(TFT_BLACK);
  if (gAsync) frameBuf1.fillScreen(TFT_BLACK);
  bufDirty[0] = DirtyList();
  bufDirty[1] = DirtyList();
  frameDirtyAccum = DirtyList();
  frameDirtyPrev = DirtyList();
  gPendClear = DirtyList();
  gCmdN = 0; gCmdPrevN = 0; gCmdOverflow = false; gCmdPrevValid = false; gClearedSinceUpdate = false;
  frameEverPushed = false;               // o 1º update manda a tela inteira (a tela ainda mostra o boot)
  gDrawIdx = 0;
  gDrawFrame = &frameBuf0;
  gLastShownIdx = -1;

#if BLIT_SLEEP
  esp_timer_create_args_t ta = {};
  ta.callback = blitTimerCb;
  ta.name = "blit";
  if (esp_timer_create(&ta, &gBlitTimer) != ESP_OK) gBlitTimer = nullptr;   // sem timer => busy-wait
#endif

  if (gAsync) {
    gFreeQ = xQueueCreate(2, sizeof(int));
    gFullQ = xQueueCreate(2, sizeof(PresentJob));
    int one = 1;
    xQueueSend(gFreeQ, &one, 0);          // buffer 0 é o do Lua; o 1 começa livre
    xTaskCreatePinnedToCore(blitTask, "BLIT", 5120, NULL, 2, NULL, 0);
  }

  Serial.printf("Display engine: %s | DMA %d px x2 (RAM interna) | SPI %d MHz | blit sleep %d\n",
                gAsync ? "async, 2 frame buffers" : "sync, 1 frame buffer",
                DMA_CHUNK_PIXELS, SPI_WRITE_HZ / 1000000, (int)BLIT_SLEEP);
}

// TIME SYNC
void syncTime(const char* ssid, const char* password) {

  if (ssid != nullptr && password != nullptr &&
      strlen(ssid) > 0 && strlen(password) > 0) {

    WiFi.begin(ssid, password);
    Serial.println("Wifi Begin");

    unsigned long startAttemptTime = millis();

    while (WiFi.status() != WL_CONNECTED) {
      if (millis() - startAttemptTime > 60000) {
        Serial.println("ERR 0x005 - WiFi timeout");
        goto WIFI_FAIL;
      }
      delay(300);
    }

    Serial.println("Wifi Connected");

    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    struct tm timeinfo;
    startAttemptTime = millis();

    while (!getLocalTime(&timeinfo)) {
      if (millis() - startAttemptTime > 60000) {
        Serial.println("ERR 0x006 - NTP timeout");
        goto WIFI_FAIL;
      }
      delay(100);
    }

    Serial.println("Time Synchronized");

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    return;
  }

WIFI_FAIL:

  tft.print("ERR 0x004 - Couldn't start WIFI and configure time. Using 1/1/1970 - 00:00:00");
  Serial.println("ERR 0x004 - Couldn't start WIFI and configure time. Using 1/1/1970 - 00:00:00");

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(3000);
}

static size_t luaHeapUsed = 0;
volatile bool luaRequestExit = false;

// ===== Alocador Lua usando PSRAM =====
void* lua_psram_alloc(void* ud, void* ptr, size_t osize, size_t nsize) {
  (void)ud;
  (void)osize; // NÃO confiamos mais nisso

  const size_t HEADER_SIZE = sizeof(size_t);

  // =========================
  // FREE
  // =========================
  if (nsize == 0) {
    if (ptr) {
      uint8_t* realPtr = (uint8_t*)ptr - HEADER_SIZE;
      size_t storedSize = *((size_t*)realPtr);

      if (luaHeapUsed >= storedSize)
        luaHeapUsed -= storedSize;
      else
        luaHeapUsed = 0;

      heap_caps_free(realPtr);
    }
    return NULL;
  }

  // =========================
  // REALLOC
  // =========================
  if (ptr) {
    uint8_t* realPtr = (uint8_t*)ptr - HEADER_SIZE;
    size_t oldSize = *((size_t*)realPtr);

    size_t futureUsed = luaHeapUsed - oldSize + nsize;

    if (futureUsed > LUA_HEAP_LIMIT) {
      Serial.println("ERR: Lua HEAP LIMIT REACHED");
      return NULL;
    }

    uint8_t* newRealPtr = (uint8_t*)heap_caps_realloc(realPtr, nsize + HEADER_SIZE,
                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (!newRealPtr) return NULL;

    *((size_t*)newRealPtr) = nsize;
    luaHeapUsed = futureUsed;

    return newRealPtr + HEADER_SIZE;
  }

  // =========================
  // MALLOC
  // =========================
  if (luaHeapUsed + nsize > LUA_HEAP_LIMIT) {
    Serial.println("ERR: Lua HEAP LIMIT REACHED");
    return NULL;
  }

  uint8_t* realPtr = (uint8_t*)heap_caps_malloc(
    nsize + HEADER_SIZE,
    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
  );

  if (!realPtr) return NULL;

  *((size_t*)realPtr) = nsize;
  luaHeapUsed += nsize;

  return realPtr + HEADER_SIZE;
}

// KERNEL FUNCTIONS
bool saveBiosPreferences() {

  if (SD.exists("/boot/boot.json")) {
    SD.remove("/boot/boot.json");
  }

  File file = SD.open("/boot/boot.json", FILE_WRITE);
  if (!file) {
    Serial.println("ERR 0x101 - Couldn't open boot.json for writing.");
    return false;
  }

  if (serializeJson(BiosConfiguration, file) == 0) {
    Serial.println("ERR 0x102 - Failed to write boot.json.");
    file.close();
    return false;
  }

  file.close();
  Serial.println("Saved boot.json");
  return true;
}

bool saveSystemPreferences() {

  if (SD.exists("/usr/sys/general.json")) {
    SD.remove("/usr/sys/general.json");
  }

  File file = SD.open("/usr/sys/general.json", FILE_WRITE);
  if (!file) {
    Serial.println("ERR 0x201 - Couldn't open general.json for writing.");
    return false;
  }

  if (serializeJson(SystemConfiguration, file) == 0) {
    Serial.println("ERR 0x202 - Failed to write general.json.");
    file.close();
    return false;
  }

  file.close();
  Serial.println("Saved system.json");
  return true;
}

bool saveUserPreferences() {

  if (SD.exists("/usr/sys/user.json")) {
    SD.remove("/usr/sys/user.json");
  }

  File file = SD.open("/usr/sys/user.json", FILE_WRITE);
  if (!file) {
    Serial.println("ERR 0x301 - Couldn't open user.json for writing.");
    return false;
  }

  if (serializeJson(UserConfiguration, file) == 0) {
    Serial.println("ERR 0x302 - Failed to write user.json.");
    file.close();
    return false;
  }

  file.close();
  Serial.println("Saved user.json");
  return true;
}

struct RowSpan { int16_t x0, x1, solid; };   // por linha do sprite: extensao dos pixels nao-chave (x1 < x0 = linha vazia)

// Copia n pixels com cor-chave 0x0000 (pixel 0 = transparente). Quando origem e destino têm o mesmo alinhamento de 16 bits,
// processa 2 pixels por vez (par todo transparente é pulado, par todo opaco é 1 escrita de 32 bits).
static inline void blitKeyedRow(const uint16_t* sp, uint16_t* dp, int n) {
  int i = 0;
  if ((((uintptr_t)sp ^ (uintptr_t)dp) & 2) == 0) {
    if (((uintptr_t)dp & 2) && n > 0) { const uint16_t c = sp[0]; if (c) dp[0] = c; i = 1; }
    for (; i + 1 < n; i += 2) {
      const uint32_t v = *(const uint32_t*)(sp + i);
      if (v == 0) continue;
      if ((v & 0xFFFFu) && (v >> 16)) *(uint32_t*)(dp + i) = v;
      else {
        if (v & 0xFFFFu) dp[i] = (uint16_t)v;
        if (v >> 16) dp[i + 1] = (uint16_t)(v >> 16);
      }
    }
  }
  for (; i < n; ++i) { const uint16_t c = sp[i]; if (c) dp[i] = c; }
}

struct Sprite {
  LGFX_Sprite spr;
  RowSpan* rows = nullptr;   // calculado 1x no load: o blit pula linhas vazias e copia trechos solidos com memcpy
  bool opaque = false;       // nenhum pixel-chave: cobre o retangulo inteiro (permite pular a limpeza)
  uint32_t serial = 0;

  float rotation = 0.0f;   // graus
  int width = 0;
  int height = 0;

  Sprite() : spr(&tft) {
    spr.setColorDepth(16);
  }

  bool load(const char* caminho, int largura, int altura) {
    width = largura;
    height = altura;

    spr.setPsram(true);

    if (!spr.createSprite(largura, altura)) {
      tft.setTextColor(TFT_WHITE);
      tft.setCursor(5, 5);
      tft.printf("ERR 0x007 - Sprite creation failed at %s", caminho);
      while (1);
    }

    File f = SD.open(caminho);
    if (!f) {
      tft.setTextColor(TFT_WHITE);
      tft.setCursor(5, 5);
      tft.printf("ERR 0x008 - Couldn't open file at %s", caminho);
      while (1);
    }

    size_t tamanho = f.size();
    uint8_t* buffer = (uint8_t*)malloc(tamanho);
    if (!buffer) {
      tft.setTextColor(TFT_WHITE);
      tft.setCursor(5, 5);
      tft.printf("ERR 0x009 - Malloc failed");
      f.close();
      while (1);
    }

    f.read(buffer, tamanho);
    f.close();

    spr.drawPng(buffer, tamanho, 0, 0);
    free(buffer);
    serial = ++gSpriteSerial;
    analyze();

    return true;
  }

  ~Sprite() { freeRows(); }

  void freeRows() {
    if (rows) { heap_caps_free(rows); rows = nullptr; }
    opaque = false;
  }

  // libera pixels e tabela de linhas (use no lugar de spr.deleteSprite())
  void freeAll() {
    freeRows();
    spr.deleteSprite();
  }

  void analyze() {
    freeRows();
    const uint16_t* s = (const uint16_t*)spr.getBuffer();
    if (!s || width <= 0 || height <= 0) return;
    RowSpan* rs = (RowSpan*)heap_caps_malloc(sizeof(RowSpan) * (size_t)height, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!rs) rs = (RowSpan*)heap_caps_malloc(sizeof(RowSpan) * (size_t)height, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!rs) return;
    const int stride = (int)spr.width();
    bool all = true;
    for (int y = 0; y < height; ++y) {
      const uint16_t* row = s + (size_t)y * stride;
      int x0 = -1, x1 = -1, cnt = 0;
      for (int x = 0; x < width; ++x) {
        if (row[x]) { if (x0 < 0) x0 = x; x1 = x; ++cnt; }
      }
      if (x0 < 0) {
        rs[y].x0 = 0; rs[y].x1 = -1; rs[y].solid = 0;
        all = false;
      } else {
        const bool solid = (cnt == x1 - x0 + 1);
        rs[y].x0 = (int16_t)x0; rs[y].x1 = (int16_t)x1; rs[y].solid = solid ? 1 : 0;
        if (!(solid && x0 == 0 && x1 == width - 1)) all = false;
      }
    }
    rows = rs;
    opaque = all;
  }

  void setRotation(float graus) {
    rotation = graus;
  }

  // 16 bpp -> 16 bpp com cor-chave 0x0000, direto nos buffers (mesmo formato: RGB565 com bytes trocados; preto = 0
  // nos dois), sem passar pelo pipeline generico do LovyanGFX. Recorta na mao contra o quadro.
  void blitKeyed(int x, int y) {
    const uint16_t* s = (const uint16_t*)spr.getBuffer();
    uint16_t* d = (uint16_t*)frame.getBuffer();
    if (!s || !d) { spr.pushSprite(&frame, x, y, 0x0000); return; }

    const int fw = (int)frame.width(), fh = (int)frame.height();
    const int sstride = (int)spr.width();
    const int sx0 = x < 0 ? -x : 0, sy0 = y < 0 ? -y : 0;
    const int sx1 = (x + width > fw) ? (fw - x) : width;
    const int sy1 = (y + height > fh) ? (fh - y) : height;
    if (sx1 <= sx0 || sy1 <= sy0) return;

    if (rows) {
      if (opaque && x == 0 && width == fw && sstride == fw) {     // opaco e da largura do quadro: 1 memcpy so
        memcpy(d + (size_t)(y + sy0) * fw, s + (size_t)sy0 * sstride, (size_t)(sy1 - sy0) * fw * 2);
        return;
      }
      for (int sy = sy0; sy < sy1; ++sy) {
        const RowSpan& rs = rows[sy];
        if (rs.x1 < rs.x0) continue;                              // linha toda transparente
        const int a = rs.x0 > sx0 ? rs.x0 : sx0;
        const int b = (rs.x1 + 1) < sx1 ? (rs.x1 + 1) : sx1;
        if (b <= a) continue;
        const uint16_t* sp = s + (size_t)sy * sstride + a;
        uint16_t* dp = d + (size_t)(y + sy) * fw + (x + a);
        const int n = b - a;
        if (rs.solid) {
          memcpy(dp, sp, (size_t)n * 2);
        } else {
          blitKeyedRow(sp, dp, n);
        }
      }
      return;
    }

    const int n = sx1 - sx0;
    for (int sy = sy0; sy < sy1; ++sy) {
      const uint16_t* sp = s + (size_t)sy * sstride + sx0;
      uint16_t* dp = d + (size_t)(y + sy) * fw + (x + sx0);
      blitKeyedRow(sp, dp, n);
    }
  }

  void draw(int x, int y) {
    uint32_t hh = hmix(0x53505201u, serial);
    hh = hmix(hh, spr.getBuffer() ? 1u : 0u);
    hh = hmix(hh, (uint32_t)x); hh = hmix(hh, (uint32_t)y); hh = hmixf(hh, rotation);

    if (rotation == 0.0f) {
      if (opaque) flushClearUnless(x, y, width, height); else flushClear();
      blitKeyed(x, y);
      cmdRecord(hh, x, y, width, height);
      return;
    }

    flushClear();

    spr.setPivot(width / 2, height / 2);
    frame.setPivot(x + width / 2, y + height / 2);

    spr.pushRotateZoom(&frame, rotation, 1.0f, 1.0f, 0x0000);

    // Com rotação a sprite pode ocupar mais área que width x height original.
    // Marca com margem de segurança usando a diagonal como lado do quadrado
    // que envolve qualquer ângulo de rotação.
    int diag = (int)ceilf(sqrtf((float)(width * width + height * height)));
    int cx = x + width / 2;
    int cy = y + height / 2;
    cmdRecord(hh, cx - diag / 2, cy - diag / 2, diag, diag);
  }
};

// TIME
typedef struct {
    time_t timestamp;
} LuaTime;

// ===== EXPORTED FUNCTIONS =====
//int l_teste(lua_State* L) {
//  int numero = luaL_checkinteger(L, 1);
//  Serial.printf("[Lua] Teste: %d (core %d)\n", numero, xPortGetCoreID());
//  return 0;
//}

int l_endProgram(lua_State* L) {
  Serial.println("[Lua] Requesting VM shutdown");
  luaRequestExit = true;
  return 0;
}

int l_clearScreen(lua_State* L) {
  // Preguicoso: so anota as regioes que precisam voltar pra preto (as do buffer ha 2 quadros + as ja desenhadas neste
  // ciclo). O memset roda no 1o desenho que nao as cobre por inteiro, ou no Display_UpdateScreen.
  DirtyList& old = bufDirty[gDrawIdx];
  for (int i = 0; i < old.n; i++) dlAdd(gPendClear, old.r[i]);
  for (int i = 0; i < frameDirtyAccum.n; i++) dlAdd(gPendClear, frameDirtyAccum.r[i]);

  old.n = 0;
  frameDirtyAccum.n = 0;  // comeca a acumular o que for desenhado neste novo frame

  gCmdN = 0;              // comeca a lista de comandos do novo quadro
  gCmdOverflow = false;
  gClearedSinceUpdate = true;
  return 0;
}

int l_updateScreen(lua_State* L){
  flushClear();           // o buffer precisa estar preto fora do que foi desenhado antes de ir pro painel

  DirtyList push;
  const bool diffMode = frameEverPushed && gClearedSinceUpdate && !gCmdOverflow && gCmdPrevValid;

  if (diffMode) {
    // So o que mudou: pra cada comando diferente do quadro anterior (mesmo indice), o retangulo antigo + o novo.
    const int n = gCmdN > gCmdPrevN ? gCmdN : gCmdPrevN;
    for (int i = 0; i < n; i++) {
      const bool hc = i < gCmdN, hp = i < gCmdPrevN;
      if (hc && hp && gCmdCur[i].h == gCmdPrev[i].h &&
          gCmdCur[i].r.x == gCmdPrev[i].r.x && gCmdCur[i].r.y == gCmdPrev[i].r.y &&
          gCmdCur[i].r.w == gCmdPrev[i].r.w && gCmdCur[i].r.h == gCmdPrev[i].r.h) continue;
      if (hp && gCmdPrev[i].r.w > 0) dlAdd(push, gCmdPrev[i].r);
      if (hc && gCmdCur[i].r.w > 0)  dlAdd(push, gCmdCur[i].r);
    }
  } else {
    // Regra antiga: o painel mostra o quadro anterior (fora dele e preto), entao envia a uniao dos dois.
    push = frameDirtyPrev;
    for (int i = 0; i < frameDirtyAccum.n; i++) dlAdd(push, frameDirtyAccum.r[i]);

    if (!frameEverPushed) {
      // Primeiro update depois do boot: manda a tela inteira uma vez pra sincronizar o display.
      push.n = 1;
      push.r[0].x = 0; push.r[0].y = 0;
      push.r[0].w = (int16_t)tft.width(); push.r[0].h = (int16_t)tft.height();
    }
  }

  // Depois deste envio, painel == render dos comandos deste quadro (se houve clear e a lista nao estourou)
  gCmdPrevValid = gClearedSinceUpdate && !gCmdOverflow;
  { CmdRec* t = gCmdCur; gCmdCur = gCmdPrev; gCmdPrev = t; }
  gCmdPrevN = gCmdN;
  gCmdN = 0;
  gCmdOverflow = false;
  gClearedSinceUpdate = false;

  bufDirty[gDrawIdx] = frameDirtyAccum;   // o que ficou desenhado NESTE buffer (o proximo clear dele apaga isso)
  frameDirtyPrev = frameDirtyAccum;
  frameDirtyAccum.n = 0;

  // Com camada 3D (ou logo depois de um quadro que teve): o fundo muda em todo pixel => tela inteira
  const bool layerNow = (gLayerSrcIdx >= 0);
  if (layerNow || gLayerPrevShown) {
    push.n = 1;
    push.r[0].x = 0; push.r[0].y = 0;
    push.r[0].w = (int16_t)tft.width(); push.r[0].h = (int16_t)tft.height();
  }
  gLayerPrevShown = layerNow ? 1 : 0;

  if (push.n > 0) {
    frameEverPushed = true;
    displaySubmit(push);          // modo async: troca o buffer de desenho
  } else {
    vTaskDelay(1);                // quadro identico: nada vai pro SPI, cede a CPU em vez de girar
  }

  // Frame completo e tela em sincronia: momento seguro pro kernel pausar a task Lua
  kernelLuaCheckpoint();
  return 0;
}

// ===== SHAPES: Display_DrawRect / Display_DrawElipse / Display_DrawArc =====
// Rasterizadas em scanlines direto no 'frame' (sem anti-alias) e reportadas ao
// dirty-rect tracker. Cores são tabelas Lua {r, g, b} (0-255).

static inline int shpMin(int a, int b) { return a < b ? a : b; }
static inline int shpMax(int a, int b) { return a > b ? a : b; }
static inline int shpClamp(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

static inline int shapeArgInt(lua_State* L, int idx) {
  return (int)lroundf((float)luaL_checknumber(L, idx));
}

// true se existe uma tabela de opções nessa posição (nil/ausente = sem opções)
static bool shapeHasOpts(lua_State* L, int idx) {
  if (lua_isnoneornil(L, idx)) return false;
  luaL_checktype(L, idx, LUA_TTABLE);
  return true;
}

// Lê opts[field] = {r, g, b}. 'idx' precisa ser um índice absoluto (positivo).
static bool shapeReadColor(lua_State* L, int idx, const char* field, uint16_t& out) {
  lua_getfield(L, idx, field);
  if (!lua_istable(L, -1)) {
    lua_pop(L, 1);
    return false;
  }
  int c[3] = {0, 0, 0};
  for (int i = 0; i < 3; i++) {
    lua_rawgeti(L, -1, i + 1);
    c[i] = shpClamp((int)lua_tonumber(L, -1), 0, 255);
    lua_pop(L, 1);
  }
  lua_pop(L, 1);
  out = tft.color565(c[0], c[1], c[2]);
  return true;
}

static int shapeReadInt(lua_State* L, int idx, const char* field, int def) {
  lua_getfield(L, idx, field);
  int v = def;
  if (lua_isnumber(L, -1)) v = (int)lroundf((float)lua_tonumber(L, -1));
  lua_pop(L, 1);
  return v;
}

static bool shapeReadBool(lua_State* L, int idx, const char* field, bool def) {
  lua_getfield(L, idx, field);
  bool v = lua_isnil(L, -1) ? def : (lua_toboolean(L, -1) != 0);
  lua_pop(L, 1);
  return v;
}

static inline uint32_t hmixStyle(uint32_t h, const ShapeStyle& st) {
  h = hmix(h, (st.hasFill ? 1u : 0u) | (st.hasBorder ? 2u : 0u));
  h = hmix(h, st.fill); h = hmix(h, st.border); h = hmix(h, (uint32_t)st.bw);
  return h;
}

// defaultFill: true = sem nenhuma cor nas opções, preenche de branco;
//              false = sem nenhuma cor nas opções, usa uma borda branca de 1 px
static void shapeReadStyle(lua_State* L, int idx, bool hasOpts, int maxBorder, bool defaultFill, ShapeStyle& st) {
  st = ShapeStyle();
  if (hasOpts) {
    st.hasFill = shapeReadColor(L, idx, "fill", st.fill);
    st.hasBorder = shapeReadColor(L, idx, "border", st.border);
    if (st.hasBorder) {
      st.bw = shpClamp(shapeReadInt(L, idx, "borderWidth", 1), 0, maxBorder);
      if (st.bw == 0) st.hasBorder = false;
    }
  }
  // Sem nenhuma opção de cor: padrão branco
  if (!st.hasFill && !st.hasBorder) {
    if (defaultFill) {
      st.hasFill = true;
      st.fill = tft.color565(255, 255, 255);
    } else {
      st.hasBorder = true;
      st.border = tft.color565(255, 255, 255);
      st.bw = 1;
    }
  }
}

// Preenchimento direto no buffer 16 bpp (bytes trocados = formato do painel), sem passar pelo pipeline genérico do
// LovyanGFX (conversão de cor + recorte + startWrite/endWrite a CADA scanline). Escreve 2 pixels por vez.
static inline void rawFillSpan(uint16_t* p, int n, uint16_t raw) {
  if (n <= 0) return;
  if ((uintptr_t)p & 2) { *p++ = raw; --n; }
  const uint32_t v = ((uint32_t)raw << 16) | raw;
  uint32_t* q = (uint32_t*)p;
  for (int m = n >> 1; m > 0; --m) *q++ = v;
  if (n & 1) *(uint16_t*)q = raw;
}

static void shapeFillRectRaw(int x, int y, int w, int h, uint16_t color) {
  uint16_t* b = (uint16_t*)frame.getBuffer();
  if (!b) return;
  const int fw = (int)frame.width(), fh = (int)frame.height();
  if (x < 0) { w += x; x = 0; }
  if (y < 0) { h += y; y = 0; }
  if (x + w > fw) w = fw - x;
  if (y + h > fh) h = fh - y;
  if (w <= 0 || h <= 0) return;
  const uint16_t raw = __builtin_bswap16(color);
  if (w == fw) { rawFillSpan(b + (size_t)y * fw, w * h, raw); return; }   // largura total: linhas contíguas
  for (int j = 0; j < h; ++j) rawFillSpan(b + (size_t)(y + j) * fw + x, w, raw);
}

static inline void shapeHLine(int x1, int x2, int y, uint16_t color) {
  if (x2 < x1) return;
  uint16_t* b = (uint16_t*)frame.getBuffer();
  if (!b) return;
  const int fw = (int)frame.width(), fh = (int)frame.height();
  if (y < 0 || y >= fh) return;
  if (x1 < 0) x1 = 0;
  if (x2 >= fw) x2 = fw - 1;
  if (x2 < x1) return;
  rawFillSpan(b + (size_t)y * fw + x1, x2 - x1 + 1, __builtin_bswap16(color));
}

// ----- Retângulo (raios por canto: 0=TL, 1=TR, 2=BR, 3=BL) -----
// Intervalo [l, r] (coords locais) ocupado pela linha j de um retângulo w x h com cantos arredondados
static void shapeRectRowSpan(int w, int h, const int rad[4], int j, int& l, int& r) {
  l = 0;
  r = w - 1;
  const float jc = j + 0.5f;

  for (int k = 0; k < 4; k++) {
    int rd = rad[k];
    if (rd <= 0) continue;

    float dy;
    if (k == 0 || k == 1) {            // cantos de cima
      if (j >= rd) continue;
      dy = rd - jc;
    } else {                           // cantos de baixo
      if (j < h - rd) continue;
      dy = jc - (h - rd);
    }

    float dx2 = (float)rd * rd - dy * dy;
    float dx = dx2 > 0 ? sqrtf(dx2) : 0.0f;
    int inset = (int)ceilf(rd - 0.5f - dx);
    if (inset < 0) inset = 0;

    if (k == 0 || k == 3) l = shpMax(l, inset);        // TL, BL
    else                  r = shpMin(r, w - 1 - inset); // TR, BR
  }
}

static void shapeDrawRoundedRect(int x, int y, int w, int h, int rad[4], const ShapeStyle& st) {
  const int maxR = shpMin(w, h) / 2;
  for (int k = 0; k < 4; k++) rad[k] = shpClamp(rad[k], 0, maxR);

  const int bw = st.hasBorder ? st.bw : 0;
  const int iw = w - 2 * bw;
  const int ih = h - 2 * bw;

  int irad[4];
  const int imaxR = shpMax(0, shpMin(iw, ih) / 2);
  for (int k = 0; k < 4; k++) irad[k] = shpClamp(rad[k] - bw, 0, imaxR);

  // Sem cantos arredondados (o caso comum de UI): fillRect do LovyanGFX no lugar de 1 drawFastHLine por linha
  if (rad[0] == 0 && rad[1] == 0 && rad[2] == 0 && rad[3] == 0 && (bw == 0 || st.hasFill)) {
    if (bw == 0) {
      shapeFillRectRaw(x, y, w, h, st.fill);
    } else {
      shapeFillRectRaw(x, y, w, h, st.border);
      if (iw > 0 && ih > 0) shapeFillRectRaw(x + bw, y + bw, iw, ih, st.fill);
    }
    return;
  }

  const int fh = (int)frame.height();
  const int j0 = shpMax(0, -y);
  const int j1 = shpMin(h, fh - y);

  for (int j = j0; j < j1; j++) {
    int ol, orr;
    shapeRectRowSpan(w, h, rad, j, ol, orr);
    ol += x;
    orr += x;
    const int py = y + j;

    if (bw == 0) {
      shapeHLine(ol, orr, py, st.fill);
      continue;
    }

    const int jin = j - bw;
    bool hasInner = (iw > 0 && ih > 0 && jin >= 0 && jin < ih);
    int il = 0, ir = 0;
    if (hasInner) {
      shapeRectRowSpan(iw, ih, irad, jin, il, ir);
      il = shpMax(il + x + bw, ol);
      ir = shpMin(ir + x + bw, orr);
      if (ir < il) hasInner = false;
    }

    if (!hasInner) {
      shapeHLine(ol, orr, py, st.border);
    } else {
      shapeHLine(ol, il - 1, py, st.border);
      shapeHLine(ir + 1, orr, py, st.border);
      if (st.hasFill) shapeHLine(il, ir, py, st.fill);
    }
  }
}

// ----- Elipse (centro cx,cy; ocupa [cx-rx, cx+rx) x [cy-ry, cy+ry)) -----
static bool shapeEllipseRowSpan(int rx, int ry, int j, int& l, int& r) {
  const float py = (j + 0.5f) - ry;
  const float t = 1.0f - (py * py) / ((float)ry * ry);
  if (t < 0.0f) return false;
  const float half = rx * sqrtf(t);
  l = (int)ceilf(rx - 0.5f - half);
  r = (int)floorf(rx - 0.5f + half);
  return r >= l;
}

static void shapeDrawEllipse(int cx, int cy, int rx, int ry, const ShapeStyle& st) {
  const int bw = st.hasBorder ? st.bw : 0;
  const int left = cx - rx;
  const int top = cy - ry;
  const int irx = rx - bw;
  const int iry = ry - bw;

  const int fh = (int)frame.height();
  const int j0 = shpMax(0, -top);
  const int j1 = shpMin(2 * ry, fh - top);

  for (int j = j0; j < j1; j++) {
    int ol, orr;
    if (!shapeEllipseRowSpan(rx, ry, j, ol, orr)) continue;
    ol += left;
    orr += left;
    const int py = top + j;

    if (bw == 0) {
      shapeHLine(ol, orr, py, st.fill);
      continue;
    }

    const int jin = j - bw;
    bool hasInner = (irx > 0 && iry > 0 && jin >= 0 && jin < 2 * iry);
    int il = 0, ir = 0;
    if (hasInner) {
      hasInner = shapeEllipseRowSpan(irx, iry, jin, il, ir);
      if (hasInner) {
        il = shpMax(il + left + bw, ol);
        ir = shpMin(ir + left + bw, orr);
        if (ir < il) hasInner = false;
      }
    }

    if (!hasInner) {
      shapeHLine(ol, orr, py, st.border);
    } else {
      shapeHLine(ol, il - 1, py, st.border);
      shapeHLine(ir + 1, orr, py, st.border);
      if (st.hasFill) shapeHLine(il, ir, py, st.fill);
    }
  }
}

// ----- Arco -----
// Ângulos em graus, sentido horário, 0° = 12 horas (topo), 90° = 3 horas...
// Anel entre rin e R (rin = 0 vira "fatia de pizza"). Centro no canto do pixel (cx, cy).
static void shapeDrawArc(int cx, int cy, int R, int rin, float startDeg, float sweepDeg, bool full, uint16_t color) {
  const float a0 = startDeg * SHAPE_DEG2RAD;
  const float a1 = (startDeg + sweepDeg) * SHAPE_DEG2RAD;
  const float sx = sinf(a0), sy = -cosf(a0);   // direção inicial (y da tela cresce pra baixo)
  const float ex = sinf(a1), ey = -cosf(a1);   // direção final
  const bool small = (sweepDeg <= 180.0f);

  const float R2 = (float)R * R;
  const float rin2 = (float)rin * rin;
  const int fw = (int)frame.width();
  const int fh = (int)frame.height();

  const int y0 = shpMax(cy - R, 0);
  const int y1 = shpMin(cy + R, fh);

  for (int y = y0; y < y1; y++) {
    const float py = y + 0.5f - cy;
    const float t = R2 - py * py;
    if (t <= 0.0f) continue;

    const float half = sqrtf(t);
    const int xs = shpMax((int)ceilf(cx - 0.5f - half), 0);
    const int xe = shpMin((int)floorf(cx - 0.5f + half), fw - 1);
    if (xe < xs) continue;

    // Varre [xa, xb], desenhando só os trechos que estão dentro do ângulo
    auto scan = [&](int xa, int xb) {
      int run = -1;
      for (int x = xa; x <= xb; x++) {
        const float px = x + 0.5f - cx;
        bool in = true;
        if (!full) {
          const float cs = sx * py - sy * px;   // produto vetorial (início, ponto)
          const float ce = px * ey - py * ex;   // produto vetorial (ponto, fim)
          in = small ? (cs >= 0.0f && ce >= 0.0f) : !(cs < 0.0f && ce < 0.0f);
        }
        if (in) {
          if (run < 0) run = x;
        } else if (run >= 0) {
          shapeHLine(run, x - 1, y, color);
          run = -1;
        }
      }
      if (run >= 0) shapeHLine(run, xb, y, color);
    };

    if (rin > 0 && py * py < rin2) {
      // Pula o buraco do meio do anel
      const float hh = sqrtf(rin2 - py * py);
      const int hs = (int)floorf(cx - 0.5f - hh) + 1;
      const int he = (int)ceilf(cx - 0.5f + hh) - 1;
      scan(xs, shpMin(hs - 1, xe));
      scan(shpMax(he + 1, xs), xe);
    } else {
      scan(xs, xe);
    }
  }
}

// Display_DrawRect(x, y, w, h [, opts])
// opts: fill={r,g,b}, border={r,g,b}, borderWidth=n (padrão 1),
//       radius=n (todos os cantos), radiusTL / radiusTR / radiusBR / radiusBL (por canto)
// Sem fill e sem border: preenche de branco. Raio grande (>= min(w,h)/2) vira "pílula".
int l_drawRect(lua_State* L) {
  const int x = shapeArgInt(L, 1);
  const int y = shapeArgInt(L, 2);
  const int w = shapeArgInt(L, 3);
  const int h = shapeArgInt(L, 4);
  const bool hasOpts = shapeHasOpts(L, 5);

  if (w <= 0 || h <= 0) return 0;

  ShapeStyle st;
  shapeReadStyle(L, 5, hasOpts, shpMin(w, h) / 2, true, st);

  int rad[4] = {0, 0, 0, 0};
  if (hasOpts) {
    const int base = shapeReadInt(L, 5, "radius", 0);
    rad[0] = shapeReadInt(L, 5, "radiusTL", base);
    rad[1] = shapeReadInt(L, 5, "radiusTR", base);
    rad[2] = shapeReadInt(L, 5, "radiusBR", base);
    rad[3] = shapeReadInt(L, 5, "radiusBL", base);
  }

  uint32_t hh = hmix(0x52454354u, (uint32_t)x);
  hh = hmix(hh, (uint32_t)y); hh = hmix(hh, (uint32_t)w); hh = hmix(hh, (uint32_t)h);
  hh = hmixStyle(hh, st);
  for (int k = 0; k < 4; k++) hh = hmix(hh, (uint32_t)rad[k]);

  flushClear();
  shapeDrawRoundedRect(x, y, w, h, rad, st);
  cmdRecord(hh, x, y, w, h);
  return 0;
}

// Display_DrawElipse(cx, cy, rx, ry [, opts])   (cx, cy = centro)
// opts: fill={r,g,b}, border={r,g,b}, borderWidth=n (padrão 1)
// rx == ry desenha um círculo.
int l_drawElipse(lua_State* L) {
  const int cx = shapeArgInt(L, 1);
  const int cy = shapeArgInt(L, 2);
  const int rx = shapeArgInt(L, 3);
  const int ry = shapeArgInt(L, 4);
  const bool hasOpts = shapeHasOpts(L, 5);

  if (rx <= 0 || ry <= 0) return 0;

  ShapeStyle st;
  shapeReadStyle(L, 5, hasOpts, shpMin(rx, ry), true, st);

  uint32_t hh = hmix(0x454C4950u, (uint32_t)cx);
  hh = hmix(hh, (uint32_t)cy); hh = hmix(hh, (uint32_t)rx); hh = hmix(hh, (uint32_t)ry);
  hh = hmixStyle(hh, st);

  flushClear();
  shapeDrawEllipse(cx, cy, rx, ry, st);
  cmdRecord(hh, cx - rx, cy - ry, 2 * rx, 2 * ry);
  return 0;
}

// Display_DrawArc(cx, cy, radius, startAngle, endAngle [, opts])
// Ângulos em graus, sempre no sentido horário de startAngle até endAngle; 0° = topo.
// |end - start| >= 360 desenha o anel completo.
// opts: color={r,g,b} (padrão branco), thickness=n (padrão 1, para dentro a partir de radius),
//       pie=true (fatia preenchida até o centro), roundCaps=true (pontas arredondadas),
//       track={r,g,b} (anel completo desenhado atrás, estilo barra de progresso)
int l_drawArc(lua_State* L) {
  const int cx = shapeArgInt(L, 1);
  const int cy = shapeArgInt(L, 2);
  const int R = shapeArgInt(L, 3);
  const float startDeg = (float)luaL_checknumber(L, 4);
  const float endDeg = (float)luaL_checknumber(L, 5);
  const bool hasOpts = shapeHasOpts(L, 6);

  if (R <= 0) return 0;

  uint16_t color = tft.color565(255, 255, 255);
  uint16_t trackColor = 0;
  bool hasTrack = false;
  int thick = 1;
  bool pie = false, caps = false;

  if (hasOpts) {
    shapeReadColor(L, 6, "color", color);
    hasTrack = shapeReadColor(L, 6, "track", trackColor);
    thick = shapeReadInt(L, 6, "thickness", 1);
    pie = shapeReadBool(L, 6, "pie", false);
    caps = shapeReadBool(L, 6, "roundCaps", false);
  }

  thick = shpClamp(thick, 1, R);
  const int rin = pie ? 0 : (R - thick);

  const float diff = endDeg - startDeg;
  const bool full = fabsf(diff) >= 360.0f;
  float sweep = fmodf(diff, 360.0f);
  if (sweep < 0.0f) sweep += 360.0f;

  uint32_t hh = hmix(0x41524331u, (uint32_t)cx);
  hh = hmix(hh, (uint32_t)cy); hh = hmix(hh, (uint32_t)R);
  hh = hmixf(hh, startDeg); hh = hmixf(hh, endDeg);
  hh = hmix(hh, color); hh = hmix(hh, trackColor); hh = hmix(hh, hasTrack ? 1u : 0u);
  hh = hmix(hh, (uint32_t)thick); hh = hmix(hh, (pie ? 1u : 0u) | (caps ? 2u : 0u));

  flushClear();
  if (hasTrack) shapeDrawArc(cx, cy, R, rin, 0.0f, 360.0f, true, trackColor);

  if (full || sweep > 0.0f) {
    shapeDrawArc(cx, cy, R, rin, startDeg, full ? 360.0f : sweep, full, color);

    if (caps && !pie && !full && thick >= 3) {
      const float mid = (R + rin) * 0.5f;
      const int capR = (thick - 1) / 2;
      for (int k = 0; k < 2; k++) {
        const float a = (startDeg + (k ? sweep : 0.0f)) * SHAPE_DEG2RAD;
        const int px = (int)lroundf(cx + mid * sinf(a) - 0.5f);
        const int py = (int)lroundf(cy - mid * cosf(a) - 0.5f);
        frame.fillCircle(px, py, capR, color);
      }
    }
  }

  cmdRecord(hh, cx - R, cy - R, 2 * R, 2 * R);
  return 0;
}

// ----- Linhas e polígonos -----
// Coordenadas de linhas e vértices são coordenadas de pixel (o centro do pixel é usado).

// Disco preenchido com centro/raio em float (usado em pontas e juntas arredondadas)
static void shapeFillDisc(float cx, float cy, float r, uint16_t color) {
  const int fw = (int)frame.width();
  const int fh = (int)frame.height();
  const int y0 = shpMax((int)floorf(cy - r), 0);
  const int y1 = shpMin((int)ceilf(cy + r), fh - 1);
  const float r2 = r * r;

  for (int y = y0; y <= y1; y++) {
    const float dy = y + 0.5f - cy;
    const float t = r2 - dy * dy;
    if (t < 0.0f) continue;
    const float half = sqrtf(t);
    const int xa = shpMax((int)ceilf(cx - 0.5f - half), 0);
    const int xb = shpMin((int)floorf(cx - 0.5f + half), fw - 1);
    shapeHLine(xa, xb, y, color);
  }
}

// Preenche um polígono qualquer (côncavo também, regra par-ímpar) com vértices em float
static void shapeFillPolyF(const float* px, const float* py, int n, uint16_t color) {
  if (n < 3 || n > SHAPE_POLY_MAX) return;

  const int fw = (int)frame.width();
  const int fh = (int)frame.height();

  float ymin = py[0], ymax = py[0];
  for (int i = 1; i < n; i++) {
    if (py[i] < ymin) ymin = py[i];
    if (py[i] > ymax) ymax = py[i];
  }

  const int y0 = shpMax((int)floorf(ymin), 0);
  const int y1 = shpMin((int)ceilf(ymax), fh - 1);

  float xs[SHAPE_POLY_MAX];

  for (int y = y0; y <= y1; y++) {
    const float yc = y + 0.5f;
    int cnt = 0;

    for (int i = 0; i < n; i++) {
      const int j = (i + 1 == n) ? 0 : i + 1;
      const float ya = py[i], yb = py[j];
      if ((ya <= yc) != (yb <= yc)) {
        if (cnt < SHAPE_POLY_MAX) {
          xs[cnt++] = px[i] + (yc - ya) * (px[j] - px[i]) / (yb - ya);
        }
      }
    }

    // ordena (poucos elementos: inserção)
    for (int a = 1; a < cnt; a++) {
      float v = xs[a];
      int b = a - 1;
      while (b >= 0 && xs[b] > v) { xs[b + 1] = xs[b]; b--; }
      xs[b + 1] = v;
    }

    for (int k = 0; k + 1 < cnt; k += 2) {
      const int xa = shpMax((int)ceilf(xs[k] - 0.5f), 0);
      const int xb = shpMin((int)ceilf(xs[k + 1] - 0.5f) - 1, fw - 1);
      shapeHLine(xa, xb, y, color);
    }
  }
}

// Retângulo girado que representa um segmento grosso (h = meia espessura).
// ext0/ext1 estendem as pontas ao longo da direção do segmento.
static void shapeStrokeQuad(float x0, float y0, float x1, float y1, float h, float ext0, float ext1, uint16_t color) {
  const float dx = x1 - x0, dy = y1 - y0;
  const float len = sqrtf(dx * dx + dy * dy);
  if (len < 1e-4f) return;

  const float ux = dx / len, uy = dy / len;
  x0 -= ux * ext0;
  y0 -= uy * ext0;
  x1 += ux * ext1;
  y1 += uy * ext1;

  const float nx = -uy * h, ny = ux * h;
  const float qx[4] = {x0 + nx, x1 + nx, x1 - nx, x0 - nx};
  const float qy[4] = {y0 + ny, y1 + ny, y1 - ny, y0 - ny};
  shapeFillPolyF(qx, qy, 4, color);
}

// Segmento com espessura e pontas opcionalmente arredondadas
static void shapeStrokeSegment(float x0, float y0, float x1, float y1, int thick, bool roundCaps, bool extend, uint16_t color) {
  const bool rounded = roundCaps && thick >= 3;
  const float h = thick * 0.5f;

  if (thick <= 1 && !rounded) {
    frame.drawLine((int)floorf(x0), (int)floorf(y0), (int)floorf(x1), (int)floorf(y1), color);
    return;
  }

  const float dx = x1 - x0, dy = y1 - y0;
  if (dx * dx + dy * dy < 1e-6f) {   // ponto
    shapeFillDisc(x0, y0, h, color);
    return;
  }

  const float ext = (extend && !rounded) ? 0.5f : 0.0f;
  shapeStrokeQuad(x0, y0, x1, y1, h, ext, ext, color);

  if (rounded) {
    shapeFillDisc(x0, y0, h, color);
    shapeFillDisc(x1, y1, h, color);
  }
}

// Junta entre dois segmentos grossos que se encontram no vértice (vx, vy)
static void shapeStrokeJoin(float ax, float ay, float vx, float vy, float bx, float by, float h, int join, uint16_t color) {
  float d1x = vx - ax, d1y = vy - ay;
  float d2x = bx - vx, d2y = by - vy;
  const float l1 = sqrtf(d1x * d1x + d1y * d1y);
  const float l2 = sqrtf(d2x * d2x + d2y * d2y);
  if (l1 < 1e-4f || l2 < 1e-4f) return;

  d1x /= l1; d1y /= l1;
  d2x /= l2; d2y /= l2;

  const float cross = d1x * d2y - d1y * d2x;
  const float dot = d1x * d2x + d1y * d2y;
  if (fabsf(cross) < 1e-3f) return;   // reto: não tem fresta pra preencher

  if (join == SHAPE_JOIN_ROUND) {
    shapeFillDisc(vx, vy, h, color);
    return;
  }

  // Lado de fora da curva
  const float side = (cross > 0.0f) ? -1.0f : 1.0f;
  const float o1x = -d1y * side * h, o1y = d1x * side * h;
  const float o2x = -d2y * side * h, o2y = d2x * side * h;

  float qx[4], qy[4];
  int m = 0;
  qx[m] = vx;       qy[m] = vy;       m++;
  qx[m] = vx + o1x; qy[m] = vy + o1y; m++;

  // Ponta da quina (limitada: em ângulos muito fechados vira bevel)
  if (join == SHAPE_JOIN_MITER && (1.0f + dot) >= 0.2222f) {
    const float k = 1.0f / (1.0f + dot);
    qx[m] = vx + (o1x + o2x) * k;
    qy[m] = vy + (o1y + o2y) * k;
    m++;
  }

  qx[m] = vx + o2x; qy[m] = vy + o2y; m++;
  shapeFillPolyF(qx, qy, m, color);
}

static void shapeStrokePolygon(const float* px, const float* py, int n, bool closed, int bw, int join, bool roundCaps, uint16_t color) {
  const float h = bw * 0.5f;
  const bool rounded = roundCaps && bw >= 3;
  const int edges = closed ? n : n - 1;

  for (int e = 0; e < edges; e++) {
    const int i = e;
    const int j = (e + 1) % n;
    const float ext0 = (!closed && e == 0 && !rounded) ? 0.5f : 0.0f;
    const float ext1 = (!closed && e == edges - 1 && !rounded) ? 0.5f : 0.0f;
    shapeStrokeQuad(px[i], py[i], px[j], py[j], h, ext0, ext1, color);
  }

  const int vStart = closed ? 0 : 1;
  const int vEnd = closed ? n : n - 1;
  for (int v = vStart; v < vEnd; v++) {
    const int p = (v - 1 + n) % n;
    const int q = (v + 1) % n;
    shapeStrokeJoin(px[p], py[p], px[v], py[v], px[q], py[q], h, join, color);
  }

  if (!closed && rounded) {
    shapeFillDisc(px[0], py[0], h, color);
    shapeFillDisc(px[n - 1], py[n - 1], h, color);
  }
}

static void shapeDrawPolygon(const int* xs, const int* ys, int n, bool closed, const ShapeStyle& st, int join, bool roundCaps) {
  float px[SHAPE_POLY_MAX], py[SHAPE_POLY_MAX];
  for (int i = 0; i < n; i++) {
    px[i] = xs[i] + 0.5f;
    py[i] = ys[i] + 0.5f;
  }

  if (st.hasFill) shapeFillPolyF(px, py, n, st.fill);

  if (st.hasBorder) {
    if (st.bw <= 1) {
      const int edges = closed ? n : n - 1;
      for (int e = 0; e < edges; e++) {
        const int j = (e + 1) % n;
        frame.drawLine(xs[e], ys[e], xs[j], ys[j], st.border);
      }
    } else {
      shapeStrokePolygon(px, py, n, closed, st.bw, join, roundCaps, st.border);
    }
  }
}

// Lê pontos de uma tabela Lua: {{x,y}, {x,y}, ...} ou {x1,y1, x2,y2, ...}
static int shapeReadPoints(lua_State* L, int idx, int* xs, int* ys, int maxN) {
  luaL_checktype(L, idx, LUA_TTABLE);
  const int len = (int)luaL_len(L, idx);
  if (len <= 0) return 0;

  lua_rawgeti(L, idx, 1);
  const bool nested = lua_istable(L, -1);
  lua_pop(L, 1);

  int n = 0;
  if (nested) {
    if (len > maxN) luaL_error(L, "polygon has too many points (max %d)", maxN);
    for (int i = 1; i <= len; i++) {
      lua_rawgeti(L, idx, i);
      luaL_checktype(L, -1, LUA_TTABLE);
      lua_rawgeti(L, -1, 1);
      lua_rawgeti(L, -2, 2);
      xs[n] = (int)lroundf((float)lua_tonumber(L, -2));
      ys[n] = (int)lroundf((float)lua_tonumber(L, -1));
      lua_pop(L, 3);
      n++;
    }
  } else {
    if (len / 2 > maxN) luaL_error(L, "polygon has too many points (max %d)", maxN);
    for (int i = 1; i + 1 <= len; i += 2) {
      lua_rawgeti(L, idx, i);
      lua_rawgeti(L, idx, i + 1);
      xs[n] = (int)lroundf((float)lua_tonumber(L, -2));
      ys[n] = (int)lroundf((float)lua_tonumber(L, -1));
      lua_pop(L, 2);
      n++;
    }
  }
  return n;
}

static int shapeReadJoin(lua_State* L, int idx) {
  int join = SHAPE_JOIN_MITER;
  lua_getfield(L, idx, "join");
  if (lua_type(L, -1) == LUA_TSTRING) {
    const char* s = lua_tostring(L, -1);
    if (strcmp(s, "round") == 0) join = SHAPE_JOIN_ROUND;
    else if (strcmp(s, "bevel") == 0) join = SHAPE_JOIN_BEVEL;
  }
  lua_pop(L, 1);
  return join;
}

// Display_DrawPoligon(points [, opts])
// points: {{x,y}, {x,y}, ...} ou {x1,y1, x2,y2, ...}  (até SHAPE_POLY_MAX = 64 vértices)
// opts: fill={r,g,b}, border={r,g,b}, borderWidth=n (padrão 1, centrada na aresta),
//       join="miter"|"bevel"|"round" (quinas da borda grossa, padrão "miter"),
//       closed=false (linha poligonal aberta), roundCaps=true (pontas redondas se aberta)
// Sem fill e sem border: polígono fechado é preenchido de branco; aberto vira linha branca.
int l_drawPoligon(lua_State* L) {
  int xs[SHAPE_POLY_MAX], ys[SHAPE_POLY_MAX];
  int n = shapeReadPoints(L, 1, xs, ys, SHAPE_POLY_MAX);
  const bool hasOpts = shapeHasOpts(L, 2);

  // Remove vértices repetidos em sequência
  int m = 0;
  for (int i = 0; i < n; i++) {
    if (m == 0 || xs[i] != xs[m - 1] || ys[i] != ys[m - 1]) {
      xs[m] = xs[i];
      ys[m] = ys[i];
      m++;
    }
  }
  n = m;

  bool closed = hasOpts ? shapeReadBool(L, 2, "closed", true) : true;
  if (closed && n > 1 && xs[0] == xs[n - 1] && ys[0] == ys[n - 1]) n--;
  if (n < 2) return 0;
  if (n == 2) closed = false;

  ShapeStyle st;
  shapeReadStyle(L, 2, hasOpts, 64, closed, st);

  int join = SHAPE_JOIN_MITER;
  bool roundCaps = false;
  if (hasOpts) {
    join = shapeReadJoin(L, 2);
    roundCaps = shapeReadBool(L, 2, "roundCaps", false);
  }

  uint32_t hh = hmix(0x504F4C59u, (uint32_t)n);
  for (int i = 0; i < n; i++) { hh = hmix(hh, (uint32_t)xs[i]); hh = hmix(hh, (uint32_t)ys[i]); }
  hh = hmix(hh, (closed ? 1u : 0u) | (roundCaps ? 2u : 0u));
  hh = hmix(hh, (uint32_t)join);
  hh = hmixStyle(hh, st);

  flushClear();
  shapeDrawPolygon(xs, ys, n, closed, st, join, roundCaps);

  int minX = xs[0], maxX = xs[0], minY = ys[0], maxY = ys[0];
  for (int i = 1; i < n; i++) {
    minX = shpMin(minX, xs[i]);
    maxX = shpMax(maxX, xs[i]);
    minY = shpMin(minY, ys[i]);
    maxY = shpMax(maxY, ys[i]);
  }
  const int pad = st.hasBorder ? (st.bw * 2 + 2) : 1;   // folga pra espessura e pontas das quinas
  cmdRecord(hh, minX - pad, minY - pad, (maxX - minX + 1) + 2 * pad, (maxY - minY + 1) + 2 * pad);
  return 0;
}

// Display_DrawLine(x1, y1, x2, y2 [, opts])
// opts: color={r,g,b} (padrão branco), thickness=n (padrão 1), roundCaps=true (pontas redondas),
//       dash=n (comprimento do traço, liga o tracejado), gap=n (espaço entre traços, padrão = dash)
// Dica: dash=1 + gap grande + roundCaps + thickness>=3 faz uma linha pontilhada.
int l_drawLine(lua_State* L) {
  const int x1 = shapeArgInt(L, 1);
  const int y1 = shapeArgInt(L, 2);
  const int x2 = shapeArgInt(L, 3);
  const int y2 = shapeArgInt(L, 4);
  const bool hasOpts = shapeHasOpts(L, 5);

  uint16_t color = tft.color565(255, 255, 255);
  int thick = 1, dash = 0, gap = -1;
  bool caps = false;

  if (hasOpts) {
    shapeReadColor(L, 5, "color", color);
    thick = shapeReadInt(L, 5, "thickness", 1);
    caps = shapeReadBool(L, 5, "roundCaps", false);
    dash = shapeReadInt(L, 5, "dash", 0);
    gap = shapeReadInt(L, 5, "gap", -1);
  }
  thick = shpClamp(thick, 1, 255);

  uint32_t hh = hmix(0x4C494E45u, (uint32_t)x1);
  hh = hmix(hh, (uint32_t)y1); hh = hmix(hh, (uint32_t)x2); hh = hmix(hh, (uint32_t)y2);
  hh = hmix(hh, color); hh = hmix(hh, (uint32_t)thick); hh = hmix(hh, caps ? 1u : 0u);
  hh = hmix(hh, (uint32_t)dash); hh = hmix(hh, (uint32_t)gap);

  flushClear();

  const float ax = x1 + 0.5f, ay = y1 + 0.5f;
  const float bx = x2 + 0.5f, by = y2 + 0.5f;

  if (dash <= 0) {
    shapeStrokeSegment(ax, ay, bx, by, thick, caps, true, color);
  } else {
    if (gap < 0) gap = dash;
    const float dx = bx - ax, dy = by - ay;
    const float len = sqrtf(dx * dx + dy * dy);

    if (len < 1e-4f) {
      shapeStrokeSegment(ax, ay, bx, by, thick, caps, false, color);
    } else {
      const float ux = dx / len, uy = dy / len;
      const float period = (float)(dash + gap);
      // Com 1 px de espessura o drawLine inclui as duas pontas, então o traço é 1 px mais curto
      const float dashLen = (thick <= 1) ? (float)(dash - 1) : (float)dash;

      for (float pos = 0.0f; pos < len; pos += period) {
        const float e = fminf(pos + dashLen, len);
        shapeStrokeSegment(ax + ux * pos, ay + uy * pos, ax + ux * e, ay + uy * e, thick, caps, false, color);
      }
    }
  }

  const int pad = thick / 2 + 2;
  const int minX = shpMin(x1, x2), minY = shpMin(y1, y2);
  const int w = (x1 > x2 ? x1 - x2 : x2 - x1) + 1;
  const int h = (y1 > y2 ? y1 - y2 : y2 - y1) + 1;
  cmdRecord(hh, minX - pad, minY - pad, w + 2 * pad, h + 2 * pad);
  return 0;
}

// Sprites
static Sprite* checkSprite(lua_State* L, int index) {
  return (Sprite*)luaL_checkudata(L, index, LUA_SPRITE_DEFINITION);
}

int l_sprite_free(lua_State* L) {
  Sprite* spr = checkSprite(L, 1);
  spr->freeAll();   // libera sprite interno + tabela de linhas
  return 0;
}

int l_sprite_constructor(lua_State* L) {
  const char* path = luaL_checkstring(L, 2);
  int w = luaL_checkinteger(L, 3);
  int h = luaL_checkinteger(L, 4);

  Sprite* spr = (Sprite*)lua_newuserdata(L, sizeof(Sprite));
  new (spr) Sprite();

  if (!spr->load(path, w, h)) {
    return luaL_error(L, "Sprite load failed");
  }

  luaL_getmetatable(L, LUA_SPRITE_DEFINITION);
  lua_setmetatable(L, -2);

  return 1;
}

int l_sprite_draw(lua_State* L) {
  Sprite* spr = checkSprite(L, 1);
  int x = luaL_checkinteger(L, 2);
  int y = luaL_checkinteger(L, 3);

  spr->draw(x, y);
  return 0;
}

int l_sprite_gc(lua_State* L) {
  Sprite* spr = checkSprite(L, 1);
  spr->~Sprite();
  return 0;
}

void registerSprite(lua_State* L) {

  // cria metatable do userdata
  luaL_newmetatable(L, LUA_SPRITE_DEFINITION);

  // __gc
  lua_pushcfunction(L, l_sprite_gc);
  lua_setfield(L, -2, "__gc");

  // métodos
  lua_newtable(L);

  lua_pushcfunction(L, l_sprite_draw);
  lua_setfield(L, -2, "draw");

  lua_pushcfunction(L, l_sprite_free);
  lua_setfield(L, -2, "free");

  lua_setfield(L, -2, "__index");

  lua_pop(L, 1);

  // ===== Tabela global Sprite =====
  lua_newtable(L);

  // metatable da tabela para suportar __call
  lua_newtable(L);
  lua_pushcfunction(L, l_sprite_constructor);
  lua_setfield(L, -2, "__call");
  lua_setmetatable(L, -2);

  lua_setglobal(L, "Sprite");
}

// OS FUNCTIONS
int l_getFreeMemory(lua_State* L){
  lua_pushinteger(L, ESP.getFreePsram());
  return 1; // number of returned values
}

int l_getMemorySize(lua_State* L){
  lua_pushinteger(L, ESP.getPsramSize());
  return 1;
}

int l_getFreeHeapMemory(lua_State* L){
  lua_pushinteger(L, ESP.getFreeHeap());
  return 1; // number of returned values
}

int l_getHeapMemorySize(lua_State* L){
  lua_pushinteger(L, ESP.getHeapSize());
  return 1;
}

int l_getCpuTemperature(lua_State* L){
    lua_pushnumber(L, temperatureRead());
    return 1;
}

int l_getLuaHeapUsage(lua_State* L){
  lua_pushnumber(L, luaHeapUsed);
  return 1;
}

int l_getUserUsername(lua_State* L){
  const char* username = UserConfiguration["username"];
  lua_pushstring(L, username);
  return 1;
}

int l_getUserPassword(lua_State* L){
  const char* password = UserConfiguration["password"];
  lua_pushstring(L, password);
  return 1;
}

// MOUSE SUPPORT
// ===== MOUSE BLUETOOTH (BLE HID) =====
#define BLE_MOUSE_DEBUG 0              // 1 = imprime no Serial os bytes que o mouse manda
#define BLE_FORCE_BOOT_PROTOCOL 0      // 1 = pede o formato simples "boot" (alguns mouses não aceitam)
#define BLE_MOUSE_NAME ""              // opcional: parte do nome do mouse pra forçar a escolha

portMUX_TYPE mouseMux = portMUX_INITIALIZER_UNLOCKED;

int  mouseX = 240;                 // começa no centro (480x320)
int  mouseY = 160;
bool mouseBtn1 = false;            // esquerdo
bool mouseBtn2 = false;            // direito
int  mouseScroll = 0;              // acumulado desde a última leitura
volatile bool mouseConnected = false;

static NimBLEClient* bleClient = nullptr;
static NimBLEAddress bleTargetAddr;
static volatile bool bleFound = false;
static volatile bool bleDisconnected = false;

volatile bool bluetoothEnabled = false;   // estado inicial do Bluetooth (true = igual ao comportamento anterior)
static bool bleStackUp = false;
static std::string bleTargetName;
static char bleConnectedName[48] = "";   // nome do dispositivo conectado (cópia segura pro Lua)

// Blacklist: dispositivos recusados com "N". Só é esvaziada quando o Bluetooth é desligado/ligado.
static std::vector<std::string> bleBlacklist;
static SemaphoreHandle_t bleBlacklistMutex = nullptr;

static bool bleIsBlacklisted(const std::string& addr) {
  bool found = false;
  if (bleBlacklistMutex && xSemaphoreTake(bleBlacklistMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    for (auto& a : bleBlacklist) {
      if (a == addr) { found = true; break; }
    }
    xSemaphoreGive(bleBlacklistMutex);
  }
  return found;
}

static void bleBlacklistAdd(const std::string& addr) {
  if (bleBlacklistMutex && xSemaphoreTake(bleBlacklistMutex, portMAX_DELAY) == pdTRUE) {
    bleBlacklist.push_back(addr);
    xSemaphoreGive(bleBlacklistMutex);
  }
}

static void bleBlacklistClear() {
  if (bleBlacklistMutex && xSemaphoreTake(bleBlacklistMutex, portMAX_DELAY) == pdTRUE) {
    bleBlacklist.clear();
    xSemaphoreGive(bleBlacklistMutex);
  }
}

// Chega um relatório do mouse
static void bleMouseNotify(NimBLERemoteCharacteristic* chr, uint8_t* d, size_t len, bool isNotify) {
#if BLE_MOUSE_DEBUG
  Serial.printf("[BLE] report (%s) len %d:", chr->getUUID().toString().c_str(), (int)len);
  for (size_t i = 0; i < len; i++) Serial.printf(" %02X", d[i]);
  Serial.println();
#endif

  int btn = 0, dx = 0, dy = 0, wh = 0;

  if (len == 3 || len == 4) {
    // Boot protocol: [botões][dx][dy][wheel]
    btn = d[0];
    dx = (int8_t)d[1];
    dy = (int8_t)d[2];
    if (len == 4) wh = (int8_t)d[3];
  } else if (len == 6 || len == 7) {
    // Formato comum com 16 bits: [botões][x lo][x hi][y lo][y hi][wheel]...
    btn = d[0];
    dx = (int16_t)(d[1] | (d[2] << 8));
    dy = (int16_t)(d[3] | (d[4] << 8));
    wh = (int8_t)d[5];
  } else {
    return;   // formato desconhecido: só o log acima
  }

  portENTER_CRITICAL(&mouseMux);

  mouseBtn1 = (btn & 0x01) != 0;
  mouseBtn2 = (btn & 0x02) != 0;

  mouseX += dx;
  mouseY += dy;   // y positivo = pra baixo, igual à tela

  if (mouseX < 0) mouseX = 0;
  if (mouseY < 0) mouseY = 0;
  if (mouseX > tft.width()  - 1) mouseX = tft.width()  - 1;
  if (mouseY > tft.height() - 1) mouseY = tft.height() - 1;

  mouseScroll += wh;

  portEXIT_CRITICAL(&mouseMux);
}

class MouseClientCb : public NimBLEClientCallbacks {
  void onDisconnect(NimBLEClient* c, int reason) override {
    bleDisconnected = true;
  }
};
static MouseClientCb bleClientCb;

class MouseScanCb : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* dev) override {
    // Já tem mouse conectado (ou BT desligado): não procura nem atualiza mais nada
    if (bleFound || mouseConnected || !bluetoothEnabled) return;

    std::string name = dev->getName();
    bool hasHid = dev->isAdvertisingService(NimBLEUUID((uint16_t)0x1812));
    uint16_t app = dev->haveAppearance() ? dev->getAppearance() : 0;

    bool match = false;
    if (strlen(BLE_MOUSE_NAME) > 0) {
      match = (name.find(BLE_MOUSE_NAME) != std::string::npos);
    } else {
      // Mouse pela aparência, ou qualquer HID que não seja teclado
      match = (app == 0x03C2) || (app == 0x03C0) || (hasHid && app != 0x03C1);
    }
    if (!match) return;

    // Recusado antes (N): ignora até desligar e ligar o Bluetooth
    std::string addr = dev->getAddress().toString();
    if (bleIsBlacklisted(addr)) return;

    bleTargetAddr = dev->getAddress();
    bleTargetName = name;
    bleFound = true;
    Serial.printf("[BLE] Mouse candidate: '%s' (%s)\n", name.c_str(), addr.c_str());
    NimBLEDevice::getScan()->stop();
  }
};
static MouseScanCb bleScanCb;

static bool bleConnectMouse() {
  if (!bleClient) {
    bleClient = NimBLEDevice::createClient();
    bleClient->setClientCallbacks(&bleClientCb, false);
    bleClient->setConnectTimeout(8000);   // ms
  }

  bleDisconnected = false;

  if (!bleClient->connect(bleTargetAddr)) {
    Serial.println("[BLE] Connect failed");
    return false;
  }

  // Pareamento/criptografia (HID exige). O pareamento fica salvo na NVS.
  if (!bleClient->secureConnection()) {
    Serial.println("[BLE] Secure connection failed");
    bleClient->disconnect();
    return false;
  }

  NimBLERemoteService* hid = bleClient->getService(NimBLEUUID((uint16_t)0x1812));
  if (!hid) {
    Serial.println("[BLE] HID service not found");
    bleClient->disconnect();
    return false;
  }

#if BLE_FORCE_BOOT_PROTOCOL
  // Pede o "boot protocol" (formato simples de 3-4 bytes)
  NimBLERemoteCharacteristic* proto = hid->getCharacteristic(NimBLEUUID((uint16_t)0x2A4E));
  if (proto && proto->canWriteNoResponse()) {
    uint8_t boot = 0;
    proto->writeValue(&boot, 1, false);
  }
#endif

  // Assina o relatório boot (0x2A33) e todos os relatórios normais (0x2A4D)
  int subs = 0;
  for (auto chr : hid->getCharacteristics(true)) {
    NimBLEUUID u = chr->getUUID();
    if (u == NimBLEUUID((uint16_t)0x2A33) || u == NimBLEUUID((uint16_t)0x2A4D)) {
      if (chr->canNotify() && chr->subscribe(true, bleMouseNotify)) {
        subs++;
        Serial.printf("[BLE] subscribed to %s\n", u.toString().c_str());
      }
    }
  }

  if (subs == 0) {
    Serial.println("[BLE] No report characteristic to subscribe");
    bleClient->disconnect();
    return false;
  }

  portENTER_CRITICAL(&mouseMux);
  mouseScroll = 0;
  portEXIT_CRITICAL(&mouseMux);

  strlcpy(bleConnectedName, bleTargetName.empty() ? "Mouse" : bleTargetName.c_str(), sizeof(bleConnectedName));
  mouseConnected = true;
  Serial.println("[BLE] Mouse connected");
  return true;
}

static void bleStackStart() {
  NimBLEDevice::init("RazorOS");
  NimBLEDevice::setSecurityAuth(true, false, true);               // bonding, sem MITM, Secure Connections
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);      // "Just Works"

  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(&bleScanCb, false);
  scan->setActiveScan(false);   // passivo: bem mais leve pro rádio compartilhado com o Wi-Fi
  scan->setInterval(160);
  scan->setWindow(30);          // ~19% de ocupação (antes 99%: matava o scan Wi-Fi)

  bleClient = nullptr;
  bleDisconnected = false;
  bleFound = false;
  bleStackUp = true;
  Serial.println("[BLE] Bluetooth ON");
}

static void bleStackStop() {
  NimBLEScan* scan = NimBLEDevice::getScan();
  if (scan->isScanning()) scan->stop();
  if (bleClient && bleClient->isConnected()) bleClient->disconnect();

  mouseConnected = false;
  portENTER_CRITICAL(&mouseMux);
  mouseBtn1 = false;
  mouseBtn2 = false;
  mouseScroll = 0;
  portEXIT_CRITICAL(&mouseMux);

  NimBLEDevice::deinit(true);   // libera client/scan; o pareamento (bond) continua na NVS
  bleClient = nullptr;
  bleFound = false;
  bleStackUp = false;
  Serial.println("[BLE] Bluetooth OFF");
}

// Task de fundo: liga/desliga a pilha BLE, procura mouse, pergunta pro usuário, conecta e reconecta se cair
static void bleMouseTask(void* arg) {
  for (;;) {
    // ----- Bluetooth desligado -----
    if (!bluetoothEnabled) {
      if (bleStackUp) bleStackStop();
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    if (!bleStackUp) bleStackStart();

    // ----- Mouse conectado: NÃO procura nem atualiza lista de dispositivos -----
    if (mouseConnected) {
      if (bleDisconnected || !bleClient || !bleClient->isConnected()) {
        mouseConnected = false;
        portENTER_CRITICAL(&mouseMux);
        mouseBtn1 = false;
        mouseBtn2 = false;
        mouseScroll = 0;
        portEXIT_CRITICAL(&mouseMux);
        Serial.println("[BLE] Mouse disconnected");
      }
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    // ----- Procurando -----
    NimBLEScan* scan = NimBLEDevice::getScan();
    bleFound = false;
    scan->start(5000, false);                // 5 s, não bloqueia

    uint32_t t0 = millis();
    while (!bleFound && bluetoothEnabled && (millis() - t0) < 6000) vTaskDelay(pdMS_TO_TICKS(50));
    scan->stop();

    if (!bluetoothEnabled) continue;         // o topo do loop desliga a pilha

    if (bleFound) {
      std::string addr = bleTargetAddr.toString();
      std::string name = bleTargetName;

      // O kernel pausa o Lua e mostra o dialog; só volta quando o usuário apertar S ou N
      bool wants = kernelAskConnectBluetooth(name.c_str(), addr.c_str());
      if (!bluetoothEnabled) continue;

      if (wants) {
        if (!bleConnectMouse()) {
          vTaskDelay(pdMS_TO_TICKS(10000));  // falhou: espera antes de oferecer de novo
        }
      } else {
        bleBlacklistAdd(addr);
        Serial.printf("[BLE] %s blacklisted until Bluetooth is toggled\n", addr.c_str());
      }
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void bleMouseBegin() {
  if (!bleBlacklistMutex) bleBlacklistMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(bleMouseTask, "BLE_MOUSE", 8192, NULL, 1, NULL, 0);
  Serial.println("BLE Mouse task started");
}

// OS_UpdateMouseTask()
// Não é mais necessário (o BLE roda numa task própria), mas fica exportada
// pra não quebrar os apps Lua que já chamam essa função.
int l_OS_UpdateMouseTask(lua_State* L) {
  return 0;
}

// OS_HasMouseConnected() -> boolean
int l_OS_HasMouseConnected(lua_State* L) {
  lua_pushboolean(L, mouseConnected);
  return 1;
}

// OS_GetMousePosition() -> { x, y }
int l_OS_GetMousePosition(lua_State* L) {
  portENTER_CRITICAL(&mouseMux);
  int x = mouseX;
  int y = mouseY;
  portEXIT_CRITICAL(&mouseMux);

  lua_createtable(L, 2, 0);
  lua_pushinteger(L, x);
  lua_rawseti(L, -2, 1);
  lua_pushinteger(L, y);
  lua_rawseti(L, -2, 2);
  return 1;
}

// OS_IsMouse1Pressed() -> boolean (esquerdo)
int l_OS_IsMouse1Pressed(lua_State* L) {
  portENTER_CRITICAL(&mouseMux);
  bool b = mouseBtn1;
  portEXIT_CRITICAL(&mouseMux);
  lua_pushboolean(L, b);
  return 1;
}

// OS_IsMouse2Pressed() -> boolean (direito)
int l_OS_IsMouse2Pressed(lua_State* L) {
  portENTER_CRITICAL(&mouseMux);
  bool b = mouseBtn2;
  portEXIT_CRITICAL(&mouseMux);
  lua_pushboolean(L, b);
  return 1;
}

// OS_GetMouseScrollDelta() -> integer (acumulado desde a última chamada; zera ao ler)
int l_OS_GetMouseScrollDelta(lua_State* L) {
  portENTER_CRITICAL(&mouseMux);
  int s = mouseScroll;
  mouseScroll = 0;
  portEXIT_CRITICAL(&mouseMux);
  lua_pushinteger(L, s);
  return 1;
}

// ===== KEYBOARD (matriz 3x10 + 2 páginas; a 2ª muda com CAPSLOCK) =====
// KB_ROWS / KB_COLS agora são #define no topo do arquivo (ver "KEYBOARD MATRIX SIZE")

const int kbRowPins[KB_ROWS] = {8, 21, 41};                           // Y1..Y3 (INPUT_PULLUP)
const int kbColPins[KB_COLS] = {48, 45, 17, 16, 40, 18, 42, 39, 47, 7}; // X1..X10

// Nomes exatamente como retornados por OS_GetPressedKey ("" = sem tecla)
// Tabela 0: página 1
// Tabela 1: página 2
// Tabela 2: página 2 com CAPSLOCK ativo
const char* const kbLayout[3][KB_ROWS][KB_COLS] = {
  {
    {"Q", "W", "E", "R", "T", "Y", "U", "I", "O", "P"},
    {"A", "S", "D", "F", "G", "H", "J", "K", "L", "RETURN"},
    {"Z", "X", "C", "V", "SPACE", "B", "N", "M", "FUNCTION", "ENTER"}
  },
  {
    {"1", "2", "3", "4", "5", "6", "7", "8", "9", "0"},
    {"CAPSLOCK", "!", "?", "+", "-", "*", "=", "~", ":", "RETURN"},
    {"%", "/", "\"", "(", ")", "^", ",", ".", "fun", "ENTER"}
  },
  {
    {"ESC", "F1", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9"},
    {"CAPSLOCK", "@", "$", "", "", "", "WifiToggle", "BluetoothToggle", "", "RETURN"},
    {"HOME", "", "", "[", "]", "<", ">", "", "fun", "ENTER"}
  }
};

bool kbMatrix[KB_ROWS][KB_COLS];       // [linha][coluna], true = pressionada
volatile uint8_t kbLayer = 0;                   // página atual (0 ou 1)
volatile bool kbCapsLock = false;               // CAPSLOCK ativo -> letras maiúsculas
volatile uint32_t kbDelayMs = 300;     // delay global anti-flood (ms)
uint32_t kbLastAcceptMs = 0;
bool kbTimerStarted = false;
SemaphoreHandle_t kbScanMutex = nullptr;   // o Lua e a task do kernel escaneiam a mesma matriz

// Camada realmente usada em kbLayout:
// página 0 -> tabela 0
// página 1 -> tabela 1, ou tabela 2 se o CAPSLOCK estiver ativo
uint8_t keyboardEffectiveLayer() {
  if (kbLayer == 1 && kbCapsLock) return 2;
  return kbLayer;
}

// Teclas que pertencem ao kernel (nunca chegam ao Lua)
static inline bool kernelIsReservedKey(const char* name) {
  return strcmp(name, KEY_WIFI_TOGGLE) == 0 || strcmp(name, KEY_BLUETOOTH_TOGGLE) == 0;
}

void keyboardInit() {
  if (!kbScanMutex) kbScanMutex = xSemaphoreCreateMutex();
  for (int l = 0; l < KB_ROWS; l++) pinMode(kbRowPins[l], INPUT_PULLUP);
  // Colunas começam em alta impedância
  for (int c = 0; c < KB_COLS; c++) pinMode(kbColPins[c], INPUT);
}

// true quando o timer anti-flood já terminou (ou nunca foi iniciado)
bool keyboardTimerReady() {
  if (!kbTimerStarted) return true;
  return (uint32_t)(millis() - kbLastAcceptMs) >= kbDelayMs;
}

void keyboardStartTimer() {
  kbLastAcceptMs = millis();
  kbTimerStarted = true;
}

// Escaneia a matriz para dentro de 'm' (protegido por mutex: Lua e kernel podem escanear ao mesmo tempo)
void keyboardScanInto(bool m[KB_ROWS][KB_COLS]) {
  if (kbScanMutex) xSemaphoreTake(kbScanMutex, portMAX_DELAY);

  for (int c = 0; c < KB_COLS; c++) {
    pinMode(kbColPins[c], OUTPUT);
    digitalWrite(kbColPins[c], LOW);
    delayMicroseconds(5);

    for (int l = 0; l < KB_ROWS; l++) {
      m[l][c] = (digitalRead(kbRowPins[l]) == LOW);
    }

    pinMode(kbColPins[c], INPUT);
  }

  if (kbScanMutex) xSemaphoreGive(kbScanMutex);
}

void keyboardScan() {
  // Um app costuma chamar OS_IsKeyPressed/OS_GetPressedKey várias vezes por quadro: cada chamada varria a matriz
  // inteira (10 colunas x pinMode/digitalWrite/5us). Reaproveita a última varredura por até 1,5 ms.
  static uint32_t lastScanUs = 0;
  static bool haveScan = false;
  const uint32_t now = (uint32_t)esp_timer_get_time();
  if (haveScan && (uint32_t)(now - lastScanUs) < 1500u) return;
  keyboardScanInto(kbMatrix);
  lastScanUs = (uint32_t)esp_timer_get_time();
  haveScan = true;
}

// Posição física da tecla FUNCTION (a mesma em todas as páginas)
bool keyboardIsFunctionCell(int layer, int l, int c) {
  (void)layer;
  return (l == 2 && c == 8);
}

// true se FUNCTION (ou CAPSLOCK, na página 2) está pressionada: a tecla é consumida e não é
// reportada ao Lua. Quem troca a página / liga o CAPSLOCK é a task do kernel (kernelTask),
// que escaneia o teclado o tempo todo, mesmo quando o app Lua não lê o teclado.
bool keyboardHandleFunction() {
  for (int c = 0; c < KB_COLS; c++) {
    if (keyboardIsFunctionCell(kbLayer, 2, c) && kbMatrix[2][c]) return true;
  }
  if (kbLayer == 1 && kbMatrix[1][0]) return true;
  return false;
}

// Letras são minúsculas por padrão; maiúsculas com CAPSLOCK ativo.
// Teclas que não são uma única letra (números, símbolos, RETURN, F1...) ficam como estão.
const char* keyboardApplyCase(const char* name, char* buf) {
  if (name[0] != '\0' && name[1] == '\0' && isalpha((unsigned char)name[0])) {
    buf[0] = kbCapsLock ? toupper((unsigned char)name[0]) : tolower((unsigned char)name[0]);
    buf[1] = '\0';
    return buf;
  }
  return name;
}

// OS_SetKeyboardDelay(ms) - delay global (em ms) entre teclas aceitas
int l_OS_SetKeyboardDelay(lua_State* L) {
  lua_Integer ms = luaL_checkinteger(L, 1);
  if (ms < 0) ms = 0;
  kbDelayMs = (uint32_t)ms;
  return 0;
}

// OS_GetPressedKey() -> string | nil
// Retorna o nome da tecla pressionada (apenas uma) ou nil se nenhuma / timer em andamento.
int l_OS_GetPressedKey(lua_State* L) {
  kernelLuaCheckpoint();

  if (!keyboardTimerReady()) {
    lua_pushnil(L);
    return 1;
  }

  keyboardScan();

  if (keyboardHandleFunction()) {
    lua_pushnil(L);
    return 1;
  }

  for (int l = 0; l < KB_ROWS; l++) {
    for (int c = 0; c < KB_COLS; c++) {
      const char* name = kbLayout[keyboardEffectiveLayer()][l][c];
      if (name[0] != '\0' && kbMatrix[l][c] && !kernelIsReservedKey(name)) {
        keyboardStartTimer();
        char caseBuf[2];
        lua_pushstring(L, keyboardApplyCase(name, caseBuf));
        return 1;
      }
    }
  }

  lua_pushnil(L);
  return 1;
}

// OS_IsKeyPressed(key) -> boolean
// true se a tecla pedida está pressionada (e o timer anti-flood permitiu a leitura).
// O timer só é reiniciado quando a tecla pedida é a que foi detectada.
int l_OS_IsKeyPressed(lua_State* L) {
  const char* key = luaL_checkstring(L, 1);

  kernelLuaCheckpoint();

  if (!keyboardTimerReady()) {
    lua_pushboolean(L, 0);
    return 1;
  }

  keyboardScan();

  if (keyboardHandleFunction()) {
    lua_pushboolean(L, 0);
    return 1;
  }

  for (int l = 0; l < KB_ROWS; l++) {
    for (int c = 0; c < KB_COLS; c++) {
      const char* name = kbLayout[keyboardEffectiveLayer()][l][c];
      if (name[0] != '\0' && kbMatrix[l][c] && !kernelIsReservedKey(name) && strcasecmp(name, key) == 0) {
        keyboardStartTimer();
        lua_pushboolean(L, 1);
        return 1;
      }
    }
  }

  lua_pushboolean(L, 0);
  return 1;
}

// ===== KERNEL SERVICES: pausa do Lua, notificação, dialog BT, toggles Wi-Fi/BT =====
static TaskHandle_t luaTaskHandle = nullptr;
static volatile bool luaTaskAlive = false;
static volatile bool luaPauseReq = false;
static volatile bool luaParked = false;

// Chamado pela task Lua em pontos seguros (sem segurar SPI/SD). Se o kernel pediu pausa, bloqueia aqui.
void kernelLuaCheckpoint() {
  if (!luaPauseReq) return;
  if (xTaskGetCurrentTaskHandle() != luaTaskHandle) return;

  luaParked = true;
  while (luaPauseReq) vTaskDelay(pdMS_TO_TICKS(5));
  luaParked = false;
}

static void kernelPauseLua() {
  luaPauseReq = true;
  if (!luaTaskAlive) return;

  // Espera o Lua chegar no próximo checkpoint (não trava pra sempre se o Lua estiver numa conta longa)
  uint32_t t0 = millis();
  while (!luaParked && (millis() - t0) < 400) vTaskDelay(pdMS_TO_TICKS(2));
}

static void kernelResumeLua() {
  luaPauseReq = false;
  uint32_t t0 = millis();
  while (luaParked && (millis() - t0) < 50) vTaskDelay(pdMS_TO_TICKS(1));
}

// ----- Acesso do kernel ao display -----
// O display agora é escrito pela tarefa de blit. Pra desenhar direto nele o kernel pausa o Lua, espera a fila de blit
// esvaziar e toma o gPresentMutex.
static void kernelDisplayBegin() {
  kernelPauseLua();
  displayWaitIdle();
  if (gPresentMutex) xSemaphoreTake(gPresentMutex, portMAX_DELAY);
}

static void kernelDisplayEnd() {
  if (gPresentMutex) xSemaphoreGive(gPresentMutex);
  kernelResumeLua();
}

// Repinta uma região a partir do último quadro enviado pelo Lua (com o toast por cima, se estiver ativo)
static void kernelRepaint(int x, int y, int w, int h) {
  presentRect(gLastShownIdx >= 0 ? gLastShownIdx : 0, x, y, w, h, gLastLayerIdx);
}

// ----- Notificação rápida (toast) -----
// Renderizada 1x num sprite pequeno; o presentRect() compõe nos blocos de DMA (cantos arredondados = cor-chave).
static char toastText[40] = "";
static volatile uint32_t toastUntilMs = 0;

static void renderToastSprite() {
  const uint16_t bg = tft.color565(25, 25, 25);
  toastSpr.fillScreen(tft.color565(255, 0, 255));                 // cor-chave (transparente)
  toastKey = ((const uint16_t*)toastSpr.getBuffer())[0];
  toastSpr.fillRoundRect(0, 0, TOAST_W, TOAST_H, 6, bg);
  toastSpr.drawRoundRect(0, 0, TOAST_W, TOAST_H, 6, TFT_WHITE);
  toastSpr.setFont(&fonts::Font2);
  toastSpr.setTextSize(1);
  toastSpr.setTextColor(TFT_WHITE, bg);
  toastSpr.setTextDatum(textdatum_t::middle_center);
  toastSpr.drawString(toastText, TOAST_W / 2, TOAST_H / 2);
}

void kernelShowToast(const char* msg) {
  if (!gToastOk) return;
  kernelDisplayBegin();
  strlcpy(toastText, msg, sizeof(toastText));
  renderToastSprite();
  toastUntilMs = millis() + TOAST_DURATION_MS;
  toastActive = true;
  kernelRepaint(TOAST_X, TOAST_Y, TOAST_W, TOAST_H);
  kernelDisplayEnd();
}

// Some com a notificação: repõe a área a partir do último quadro do Lua
static void kernelHideToast() {
  kernelDisplayBegin();
  toastActive = false;
  kernelRepaint(TOAST_X, TOAST_Y, TOAST_W, TOAST_H);
  kernelDisplayEnd();
}

// ----- Wi-Fi (gerenciador: varredura, conexão e sincronização de hora em background) -----
volatile bool wifiEnabled = false;   // o syncTime() já deixa o Wi-Fi desligado depois do boot

#define WIFI_CONNECT_TIMEOUT_MS 20000   // tempo máximo tentando conectar
#define WIFI_FAILED_SHOW_MS 3000        // por quanto tempo o status "failed" fica visível pro Lua
#define WIFI_SCAN_INTERVAL_MS 8000      // intervalo entre varreduras (enquanto alguém pede a lista)
#define WIFI_SCAN_KEEPALIVE_MS 15000    // varre por esse tempo depois da última chamada de OS_GetFoundWifiNetworks
#define WIFI_SCAN_TIMEOUT_MS 10000
#define WIFI_MAX_NETWORKS 24
#define WIFI_TIME_RETRY_MS 15000        // espera entre tentativas de sincronizar a hora

// Credenciais salvas (cópia de SystemConfiguration["Wifi"]). O JSON só é atualizado e gravado
// em wifiFlushPendingSave(), que roda na task Lua (a mesma que usa o SD nos outros saves).
static char wifiSavedSsid[33] = "";
static char wifiSavedPass[65] = "";
static volatile bool wifiSavePending = false;

static volatile bool wifiConnecting = false;
static volatile bool wifiFailed = false;
static volatile uint32_t wifiFailedAtMs = 0;
static volatile bool wifiTimeSynced = false;

static volatile bool wifiScanStarted = false;
static volatile bool wifiScanning = false;
static volatile bool wifiScanNow = false;
static volatile uint32_t wifiScanKeepUntilMs = 0;

static volatile bool wifiConnReq = false;     // pedido de conexão vindo do Lua
static char wifiReqSsid[33] = "";
static char wifiReqPass[65] = "";

static std::vector<std::string> wifiNetworks;           // SSIDs da última varredura (mais forte primeiro)
static SemaphoreHandle_t wifiNetMutex = nullptr;        // protege wifiNetworks
static SemaphoreHandle_t wifiDrvMutex = nullptr;        // protege chamadas curtas ao driver (WiFi.*)

static inline void wifiLock()   { if (wifiDrvMutex) xSemaphoreTake(wifiDrvMutex, portMAX_DELAY); }
static inline void wifiUnlock() { if (wifiDrvMutex) xSemaphoreGive(wifiDrvMutex); }

#define WIFI_AUTOCONNECT_MS 12000   // tempo dado pra rede salva conectar antes de liberar scan/retry
#define WIFI_RECONNECT_MS 15000     // intervalo entre novas tentativas na rede salva
static volatile uint32_t wifiLastBeginMs = 0;

// Chamar com o lock do driver
static void wifiBeginSaved() {
  if (wifiSavedSsid[0] != '\0') {
    WiFi.begin(wifiSavedSsid, wifiSavedPass[0] ? wifiSavedPass : nullptr);   // não bloqueia
    wifiLastBeginMs = millis();
    Serial.printf("[WIFI] Auto-connecting to saved '%s'\n", wifiSavedSsid);
  }
}

static void wifiApply(bool on) {
  wifiLock();
  if (on) {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.persistent(false);
    wifiEnabled = true;
    wifiFailed = false;
    wifiTimeSynced = false;
    wifiScanNow = true;
    wifiBeginSaved();          // conecta sozinho na rede salva no SystemConfiguration
    Serial.println("[WIFI] ON");
  } else {
    wifiEnabled = false;       // antes de derrubar o driver: a task Wi-Fi vê isso e abandona o que estava fazendo
    wifiConnReq = false;
    wifiConnecting = false;
    wifiFailed = false;
    wifiTimeSynced = false;
    if (wifiScanStarted) esp_wifi_scan_stop();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    Serial.println("[WIFI] OFF");
  }
  wifiUnlock();

  if (!on && wifiNetMutex && xSemaphoreTake(wifiNetMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    wifiNetworks.clear();
    xSemaphoreGive(wifiNetMutex);
  }
}

// Atualiza o JSON do sistema com as credenciais que funcionaram e grava no SD.
// Roda no contexto da task Lua (chamada por OS_GetWifiStatus).
static void wifiFlushPendingSave() {
  if (!wifiSavePending) return;
  wifiSavePending = false;

  // const char* explícito: o ArduinoJson guarda o ponteiro (sem copiar), então o JSON sempre
  // reflete o buffer e o documento não vai enchendo a cada nova conexão
  SystemConfiguration["Wifi"]["ssid"] = (const char*)wifiSavedSsid;
  SystemConfiguration["Wifi"]["password"] = (const char*)wifiSavedPass;
  saveSystemPreferences();
}

static void wifiCollectScanResults(int n) {
  std::vector<std::pair<int, std::string>> found;   // (rssi, ssid)

  for (int i = 0; i < n; i++) {
    String s = WiFi.SSID(i);
    if (s.length() == 0) continue;   // rede oculta
    const int rssi = WiFi.RSSI(i);

    bool dup = false;
    for (auto& f : found) {
      if (f.second == s.c_str()) {
        if (rssi > f.first) f.first = rssi;
        dup = true;
        break;
      }
    }
    if (!dup) found.push_back(std::make_pair(rssi, std::string(s.c_str())));
  }

  std::sort(found.begin(), found.end(),
            [](const std::pair<int, std::string>& a, const std::pair<int, std::string>& b) { return a.first > b.first; });
  if (found.size() > WIFI_MAX_NETWORKS) found.resize(WIFI_MAX_NETWORKS);

  if (wifiNetMutex && xSemaphoreTake(wifiNetMutex, portMAX_DELAY) == pdTRUE) {
    wifiNetworks.clear();
    for (auto& f : found) wifiNetworks.push_back(f.second);
    xSemaphoreGive(wifiNetMutex);
  }
  Serial.printf("[WIFI] Scan: %d networks\n", (int)found.size());
}

// Conecta (bloqueia só a task Wi-Fi). Em caso de sucesso guarda as credenciais e agenda o save;
// a hora é sincronizada em seguida pelo loop da task.
static void wifiConnectNow(const char* ssid, const char* pass) {
  Serial.printf("[WIFI] Connecting to '%s'\n", ssid);

  wifiLock();
  if (wifiScanStarted) {
    esp_wifi_scan_stop();
    WiFi.scanDelete();
    wifiScanStarted = false;
    wifiScanning = false;
  }
  if (!wifiEnabled) {
    WiFi.mode(WIFI_STA);
    wifiEnabled = true;
  }
  wifiFailed = false;
  wifiTimeSynced = false;
  WiFi.disconnect();
  WiFi.begin(ssid, pass[0] ? pass : nullptr);
  wifiUnlock();

  const uint32_t t0 = millis();
  bool ok = false;
  while ((millis() - t0) < WIFI_CONNECT_TIMEOUT_MS) {
    if (!wifiEnabled) break;      // desligaram o Wi-Fi
    if (wifiConnReq) break;       // veio outro pedido: abandona este
    const wl_status_t st = WiFi.status();
    if (st == WL_CONNECTED) { ok = true; break; }
    if ((millis() - t0) > 1500 && (st == WL_CONNECT_FAILED || st == WL_NO_SSID_AVAIL)) break;
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  if (!wifiEnabled || wifiConnReq) return;   // o próximo pedido (ou o desligamento) cuida do estado

  if (ok) {
    strlcpy(wifiSavedSsid, ssid, sizeof(wifiSavedSsid));
    strlcpy(wifiSavedPass, pass, sizeof(wifiSavedPass));
    wifiSavePending = true;
    Serial.println("[WIFI] Connected");
  } else {
    Serial.println("[WIFI] Connection failed");
    wifiFailedAtMs = millis();
    wifiFailed = true;
    // Volta pra rede que estava salva antes
    wifiLock();
    WiFi.disconnect();
    wifiBeginSaved();
    wifiUnlock();
  }
  wifiConnecting = false;
}

static void wifiTask(void* arg) {
  uint32_t lastScanMs = 0, scanStartMs = 0, nextTimeSyncMs = 0;

  for (;;) {
    // ----- pedido de conexão -----
    if (wifiConnReq) {
      char ssid[33], pass[65];
      strlcpy(ssid, wifiReqSsid, sizeof(ssid));
      strlcpy(pass, wifiReqPass, sizeof(pass));
      wifiConnReq = false;
      wifiConnectNow(ssid, pass);
      continue;
    }

    const uint32_t now = millis();

    // ----- varredura (só enquanto alguém está pedindo a lista) -----
    const bool wifiLinked = (WiFi.status() == WL_CONNECTED);
    const bool haveSaved = (wifiSavedSsid[0] != '\0');
    // Enquanto a rede salva está tentando conectar, não varre (o scan derrubaria a tentativa)
    const bool inAttempt = wifiEnabled && haveSaved && !wifiLinked && !wifiConnecting && !wifiScanStarted &&
                           (now - wifiLastBeginMs) < WIFI_AUTOCONNECT_MS;
    // Conectado a uma rede: NÃO varre mais (a lista fica com o último resultado)
    const bool wantScan = wifiEnabled && !wifiConnecting && !inAttempt && !wifiLinked && (int32_t)(wifiScanKeepUntilMs - now) > 0;

    // ----- mantém conectado na rede salva: se caiu / não conectou, tenta de novo -----
    if (wifiEnabled && haveSaved && !wifiLinked && !wifiConnecting && !wifiScanStarted && !inAttempt &&
        (now - wifiLastBeginMs) >= WIFI_RECONNECT_MS) {
      wifiLock();
      WiFi.disconnect(false);
      wifiBeginSaved();
      wifiUnlock();
    }

    if (wifiScanStarted) {
      if (!wifiEnabled) {
        wifiScanStarted = false;
        wifiScanning = false;
      } else {
        const int n = WiFi.scanComplete();
        if (n == WIFI_SCAN_RUNNING) {
          if ((now - scanStartMs) > WIFI_SCAN_TIMEOUT_MS) {
            wifiLock();
            esp_wifi_scan_stop();
            WiFi.scanDelete();
            wifiUnlock();
            wifiScanStarted = false;
            wifiScanning = false;
          }
        } else {
          if (n >= 0) wifiCollectScanResults(n);
          wifiLock();
          WiFi.scanDelete();
          if (!wifiConnecting && WiFi.status() != WL_CONNECTED) wifiBeginSaved();
          wifiUnlock();
          wifiScanStarted = false;
          wifiScanning = false;
        }
      }
    } else if (wantScan && (wifiScanNow || (now - lastScanMs) >= WIFI_SCAN_INTERVAL_MS)) {
      wifiScanNow = false;
      lastScanMs = now;
      wifiLock();
      WiFi.scanDelete();
      if (WiFi.status() != WL_CONNECTED) WiFi.disconnect(false);   // cancela "connecting" pendente que derruba o scan
      const int r = WiFi.scanNetworks(true, false);   // assíncrono, sem redes ocultas
      wifiUnlock();
      Serial.printf("[WIFI] scanNetworks -> %d\n", r);
      if (r == WIFI_SCAN_RUNNING || r >= 0) {
        wifiScanStarted = true;
        wifiScanning = true;
        scanStartMs = now;
      } else {
        lastScanMs = now - WIFI_SCAN_INTERVAL_MS + 1000;   // falhou: tenta de novo em ~1 s
      }
    }

    // ----- hora: sincroniza sempre que conectar -----
    if (wifiEnabled && !wifiConnecting) {
      if (WiFi.status() == WL_CONNECTED) {
        if (!wifiTimeSynced && (int32_t)(now - nextTimeSyncMs) >= 0) {
          configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
          struct tm ti;
          if (getLocalTime(&ti, 5000)) {
            wifiTimeSynced = true;
            Serial.println("[WIFI] Time synchronized");
          } else {
            nextTimeSyncMs = millis() + WIFI_TIME_RETRY_MS;
            Serial.println("[WIFI] NTP sync failed, retrying later");
          }
        }
      } else {
        wifiTimeSynced = false;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void wifiManagerBegin() {
  if (!wifiDrvMutex) wifiDrvMutex = xSemaphoreCreateMutex();
  if (!wifiNetMutex) wifiNetMutex = xSemaphoreCreateMutex();

  const char* ssid = SystemConfiguration["Wifi"]["ssid"] | "";
  const char* pass = SystemConfiguration["Wifi"]["password"] | "";
  strlcpy(wifiSavedSsid, ssid, sizeof(wifiSavedSsid));
  strlcpy(wifiSavedPass, pass, sizeof(wifiSavedPass));

  xTaskCreatePinnedToCore(wifiTask, "WIFI_MGR", 8192, NULL, 1, NULL, 0);
  Serial.println("Wifi manager task started");
}

static void bluetoothApply(bool on) {
  // Ligar ou desligar sempre zera a blacklist (nova chance de autorizar dispositivos recusados)
  bleBlacklistClear();
  if (!on) mouseConnected = false;
  bluetoothEnabled = on;   // a task BLE liga/desliga a pilha e a busca
}

int l_OS_IsWifiEnabled(lua_State* L) {
  lua_pushboolean(L, wifiEnabled);
  return 1;
}

// OS_GetConnectedBluetoothName() -> string | nil
int l_OS_GetConnectedBluetoothName(lua_State* L) {
  if (bluetoothEnabled && mouseConnected) lua_pushstring(L, bleConnectedName);
  else lua_pushnil(L);
  return 1;
}

int l_OS_IsBluetoothEnabled(lua_State* L) {
  lua_pushboolean(L, bluetoothEnabled);
  return 1;
}

// OS_EnableWifi() / OS_DisableWifi()
// Só age se o estado realmente mudar. Não mostra toast: o toast pausa a task Lua
// e esperaria o próprio checkpoint dela (travaria ~400 ms se chamado daqui).
int l_OS_EnableWifi(lua_State* L) {
  if (!wifiEnabled) wifiApply(true);
  return 0;
}

int l_OS_DisableWifi(lua_State* L) {
  if (wifiEnabled) wifiApply(false);
  return 0;
}

// OS_EnableBluetooth() / OS_DisableBluetooth()
// Só age se o estado mudar (bluetoothApply também zera a blacklist de mouses recusados).
int l_OS_EnableBluetooth(lua_State* L) {
  if (!bluetoothEnabled) bluetoothApply(true);
  return 0;
}

int l_OS_DisableBluetooth(lua_State* L) {
  if (bluetoothEnabled) bluetoothApply(false);
  return 0;
}

// OS_GetFoundWifiNetworks() -> { "ssid1", "ssid2", ... }
// Lista (só SSIDs, sem repetição, sinal mais forte primeiro) da última varredura.
// Enquanto for chamada de tempos em tempos, o kernel mantém varreduras em background;
// ~15 s sem chamadas e ele para de varrer. Vazia se o Wi-Fi estiver desligado.
int l_OS_GetFoundWifiNetworks(lua_State* L) {
  const uint32_t now = millis();
  if ((int32_t)(wifiScanKeepUntilMs - now) <= 0) wifiScanNow = true;   // ninguém pedia: varre já
  wifiScanKeepUntilMs = now + WIFI_SCAN_KEEPALIVE_MS;

  // Copia sob o mutex e só depois mexe na pilha do Lua (um erro de memória do Lua não pode deixar o mutex preso)
  std::vector<std::string> copy;
  if (wifiNetMutex && xSemaphoreTake(wifiNetMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    copy = wifiNetworks;
    xSemaphoreGive(wifiNetMutex);
  }

  // Conectado e sem varredura: garante que a rede atual apareça na lista
  if (wifiEnabled && !wifiConnecting && WiFi.status() == WL_CONNECTED) {
    std::string cur = WiFi.SSID().c_str();
    bool has = false;
    for (auto& e : copy) if (e == cur) { has = true; break; }
    if (!has && !cur.empty()) copy.insert(copy.begin(), cur);
  }

  lua_createtable(L, (int)copy.size(), 0);
  for (size_t i = 0; i < copy.size(); i++) {
    lua_pushstring(L, copy[i].c_str());
    lua_rawseti(L, -2, (lua_Integer)(i + 1));
  }
  return 1;
}

// OS_ConnectWifi(ssid [, password])
// Liga o Wi-Fi se preciso, conecta (em background), e se der certo grava ssid/senha em
// SystemConfiguration["Wifi"] (general.json) e sincroniza a hora. Acompanhe com OS_GetWifiStatus().
int l_OS_ConnectWifi(lua_State* L) {
  const char* ssid = luaL_checkstring(L, 1);
  const char* pass = luaL_optstring(L, 2, "");
  if (ssid[0] == '\0') return 0;

  strlcpy(wifiReqSsid, ssid, sizeof(wifiReqSsid));
  strlcpy(wifiReqPass, pass, sizeof(wifiReqPass));
  wifiFailed = false;
  wifiConnecting = true;    // já vale antes da task acordar, pro Lua não ler um status velho
  wifiConnReq = true;
  return 0;
}

// OS_GetWifiStatus() -> "off" | "idle" | "connecting" | "connected" | "failed"
// "failed" aparece por ~3 s depois de uma tentativa de OS_ConnectWifi que não deu certo.
// Também é aqui que as credenciais novas são gravadas no SD (precisa ser chamada de vez em quando).
int l_OS_GetWifiStatus(lua_State* L) {
  wifiFlushPendingSave();

  const char* s;
  if (!wifiEnabled) s = "off";
  else if (wifiConnecting) s = "connecting";
  else if (wifiFailed && (uint32_t)(millis() - wifiFailedAtMs) < WIFI_FAILED_SHOW_MS) s = "failed";
  else if (WiFi.status() == WL_CONNECTED) s = "connected";
  else s = "idle";

  lua_pushstring(L, s);
  return 1;
}

// OS_GetConnectedWifiSSID() -> string | nil
int l_OS_GetConnectedWifiSSID(lua_State* L) {
  if (wifiEnabled && !wifiConnecting && WiFi.status() == WL_CONNECTED) {
    lua_pushstring(L, WiFi.SSID().c_str());
  } else {
    lua_pushnil(L);
  }
  return 1;
}

// OS_ForgetWifi()
// Desconecta da rede atual e esquece as credenciais salvas (ssid/senha em SystemConfiguration["Wifi"]).
// O Wi-Fi continua ligado (sem reconectar sozinho) e a varredura volta a funcionar.
int l_OS_ForgetWifi(lua_State* L) {
  wifiConnReq = false;
  wifiConnecting = false;
  wifiFailed = false;
  wifiTimeSynced = false;

  wifiLock();
  if (wifiScanStarted) {
    esp_wifi_scan_stop();
    WiFi.scanDelete();
    wifiScanStarted = false;
    wifiScanning = false;
  }
  WiFi.disconnect(false);
  wifiSavedSsid[0] = '\0';
  wifiSavedPass[0] = '\0';
  wifiUnlock();

  wifiSavePending = true;
  wifiFlushPendingSave();   // grava no SD agora (esta função roda na task Lua)
  wifiScanNow = true;
  Serial.println("[WIFI] Network forgotten");
  return 0;
}

// OS_IsWifiScanning() -> boolean
int l_OS_IsWifiScanning(lua_State* L) {
  lua_pushboolean(L, wifiScanning);
  return 1;
}

// ----- Dialog "Conectar ao mouse?" (S / N) -----
static volatile int bleAskState = 0;   // 0 = nada, 1 = pergunta pendente, 2 = S, 3 = N
static char bleAskName[40] = "";
static char bleAskAddr[20] = "";

// Chamada pela task BLE: bloqueia até o kernel receber S ou N
bool kernelAskConnectBluetooth(const char* name, const char* addr) {
  strlcpy(bleAskName, (name && name[0]) ? name : "(sem nome)", sizeof(bleAskName));
  strlcpy(bleAskAddr, addr ? addr : "", sizeof(bleAskAddr));
  bleAskState = 1;
  while (bleAskState == 1) vTaskDelay(pdMS_TO_TICKS(20));
  bool yes = (bleAskState == 2);
  bleAskState = 0;
  return yes;
}

// Lê a posição física das teclas S/N da página 1 (funciona em qualquer página/CAPSLOCK)
static bool kbKeyDownLayer0(bool m[][KB_COLS], const char* key) {
  for (int l = 0; l < KB_ROWS; l++) {
    for (int c = 0; c < KB_COLS; c++) {
      if (strcasecmp(kbLayout[0][l][c], key) == 0) return m[l][c];
    }
  }
  return false;
}

static void drawBluetoothDialog(const char* name, const char* addr) {
  const int W = 380, H = 130;
  const int x = (tft.width() - W) / 2;
  const int y = (tft.height() - H) / 2;
  const int cx = x + W / 2;
  uint16_t bg = tft.color565(20, 20, 20);

  tft.fillRoundRect(x, y, W, H, 8, bg);
  tft.drawRoundRect(x, y, W, H, 8, TFT_WHITE);
  tft.setFont(&fonts::Font2);
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, bg);
  tft.setTextDatum(textdatum_t::top_center);
  tft.drawString("Found Bluetooth Mouse", cx, y + 12);
  tft.drawString("Connect to this device?", cx, y + 32);
  tft.drawString(name, cx, y + 58);
  tft.drawString(addr, cx, y + 76);
  tft.drawString("[S] Yes        [N] No", cx, y + 102);
}

static void kernelRunBluetoothDialog() {
  char name[40], addr[20];
  strlcpy(name, bleAskName, sizeof(name));
  strlcpy(addr, bleAskAddr, sizeof(addr));

  kernelDisplayBegin();     // pausa o Lua, esvazia o blit e segura o display até o fim do dialog
  toastActive = false;
  drawBluetoothDialog(name, addr);

  bool m[KB_ROWS][KB_COLS];
  bool answer = false;
  bool armed = false;   // só aceita S/N depois de ver as duas soltas (evita responder com tecla já apertada)

  for (;;) {
    keyboardScanInto(m);
    bool s = kbKeyDownLayer0(m, "S");
    bool n = kbKeyDownLayer0(m, "N");

    if (!s && !n) {
      armed = true;
    } else if (armed) {
      answer = s;   // S = sim, N = não
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(15));
  }

  // Espera soltar a tecla pra ela não vazar pro Lua quando ele voltar
  uint32_t t0 = millis();
  while ((millis() - t0) < 1500) {
    keyboardScanInto(m);
    if (!kbKeyDownLayer0(m, "S") && !kbKeyDownLayer0(m, "N")) break;
    vTaskDelay(pdMS_TO_TICKS(15));
  }

  // Restaura a tela inteira a partir do último quadro do Lua
  kernelRepaint(0, 0, tft.width(), tft.height());
  kernelDisplayEnd();
  bleAskState = answer ? 2 : 3;
}

// ----- Task do kernel: teclas WifiToggle/BluetoothToggle, toast e dialog -----
static void kernelTask(void* arg) {
  bool m[KB_ROWS][KB_COLS];
  bool wifiHeld = false, btHeld = false;
  bool funcHeld = false, capsHeld = false;
  bool wifiUsed = false, btUsed = false;
  uint32_t wifiLastMs = 0, btLastMs = 0;
#if KERNEL_KEY_DEBUG
  bool dbgPrev[KB_ROWS][KB_COLS] = {};
#endif

  for (;;) {
    if (bleAskState == 1) {
      kernelRunBluetoothDialog();
    }

    if (toastActive && (int32_t)(millis() - toastUntilMs) >= 0) {
      kernelHideToast();
    }

    keyboardScanInto(m);

    // FUNCTION: alterna página 1 <-> 2 | CAPSLOCK (só na página 2): liga/desliga maiúsculas.
    // Uma ação por aperto (detecta a borda), independente do app Lua estar lendo teclado.
    bool funcDown = false;
    for (int c = 0; c < KB_COLS; c++) {
      if (keyboardIsFunctionCell(kbLayer, 2, c) && m[2][c]) funcDown = true;
    }
    if (funcDown && !funcHeld) kbLayer = (kbLayer + 1) % 2;
    funcHeld = funcDown;

    bool capsDown = (kbLayer == 1) && m[1][0];
    if (capsDown && !capsHeld) kbCapsLock = !kbCapsLock;
    capsHeld = capsDown;

    const uint8_t layer = keyboardEffectiveLayer();
    bool wifiDown = false, btDown = false;

    for (int l = 0; l < KB_ROWS; l++) {
      for (int c = 0; c < KB_COLS; c++) {
        if (!m[l][c]) continue;
        const char* name = kbLayout[layer][l][c];
#if KERNEL_KEY_DEBUG
        // Imprime cada tecla nova que o kernel enxerga (mostra em que camada ele acha que está)
        if (!dbgPrev[l][c]) {
          Serial.printf("[KERNEL KEY] cell(%d,%d) layer=%d kbLayer=%d caps=%d name='%s'\n",
                        l, c, layer, (int)kbLayer, (int)kbCapsLock, name);
        }
#endif
        if (strcmp(name, KEY_WIFI_TOGGLE) == 0) wifiDown = true;
        else if (strcmp(name, KEY_BLUETOOTH_TOGGLE) == 0) btDown = true;
      }
    }

#if KERNEL_KEY_DEBUG
    memcpy(dbgPrev, m, sizeof(dbgPrev));
#endif
    uint32_t now = millis();

    // Cada tecla: 1 ação por aperto e timer de 2 s entre usos
    if (wifiDown && !wifiHeld && (!wifiUsed || (now - wifiLastMs) >= TOGGLE_KEY_COOLDOWN_MS)) {
      wifiUsed = true;
      wifiLastMs = now;
      bool on = !wifiEnabled;
      wifiApply(on);
      kernelShowToast(on ? "Wi-Fi ativado" : "Wi-Fi desativado");
    }
    wifiHeld = wifiDown;

    if (btDown && !btHeld && (!btUsed || (now - btLastMs) >= TOGGLE_KEY_COOLDOWN_MS)) {
      btUsed = true;
      btLastMs = now;
      bool on = !bluetoothEnabled;
      bluetoothApply(on);
      kernelShowToast(on ? "Bluetooth activated" : "Bluetooth deactivated");
    }
    btHeld = btDown;

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void kernelServicesBegin() {
  xTaskCreatePinnedToCore(kernelTask, "KERNEL", 8192, NULL, 2, NULL, 0);
  Serial.println("Kernel services task started");
}

int l_OS_Sleep(lua_State* L){
  kernelLuaCheckpoint();
  delay(luaL_checkinteger(L, 1));
  kernelLuaCheckpoint();
  return 0;
}

int l_OS_Restart(lua_State* L){
  esp_restart();
  return 0;
}

int l_OS_GetKernelVersion(lua_State* L){
  lua_pushstring(L, KERNEL_VERSION);
  return 1;
}

int l_OS_SetBiosConfigIntegerKey(lua_State* L){
    const char* key = luaL_checkstring(L, 1);
    int val = luaL_checkinteger(L, 2);

    BiosConfiguration[key] = val;

    return 0;
}

int l_OS_SetBiosConfigStringKey(lua_State* L){
    const char* key = luaL_checkstring(L, 1);
    const char* val = luaL_checkstring(L, 2);

    BiosConfiguration[key] = val;

    return 0;
}

int l_OS_SetSystemConfigIntegerKey(lua_State* L){
    const char* key = luaL_checkstring(L, 1);
    int val = luaL_checkinteger(L, 2);

    SystemConfiguration[key] = val;

    return 0;
}

int l_OS_SetSystemConfigStringKey(lua_State* L){
    const char* key = luaL_checkstring(L, 1);
    const char* val = luaL_checkstring(L, 2);

    SystemConfiguration[key] = val;

    return 0;
}

int l_OS_SetUserConfigIntegerKey(lua_State* L){
    const char* key = luaL_checkstring(L, 1);
    int val = luaL_checkinteger(L, 2);

    UserConfiguration[key] = val;

    return 0;
}

int l_OS_SetUserConfigStringKey(lua_State* L){
    const char* key = luaL_checkstring(L, 1);
    const char* val = luaL_checkstring(L, 2);

    UserConfiguration[key] = val;

    return 0;
}

int l_OS_SaveBiosConfig(lua_State* L){
  saveBiosPreferences();
  return 0;
}

int l_OS_SaveSystemConfig(lua_State* L){
  saveSystemPreferences();
  return 0;
}

int l_OS_SaveUserConfig(lua_State* L){
  saveUserPreferences();
  return 0;
}

int l_time_toString(lua_State* L) {
    LuaTime* t = (LuaTime*)luaL_checkudata(L, 1, "LuaTime");
    const char* format = luaL_optstring(L, 2, "%d/%m/%Y %H:%M:%S");

    struct tm* tm_info = localtime(&t->timestamp);

    char buffer[64];
    strftime(buffer, sizeof(buffer), format, tm_info);

    lua_pushstring(L, buffer);
    return 1;
}

int l_getTime(lua_State* L) {

    LuaTime* t = (LuaTime*)lua_newuserdata(L, sizeof(LuaTime));
    t->timestamp = time(nullptr);

    luaL_getmetatable(L, "LuaTime");
    lua_setmetatable(L, -2);

    return 1;
}

void registerTime(lua_State* L) {

    luaL_newmetatable(L, "LuaTime");

    lua_pushvalue(L, -1);
    lua_setfield(L, -2, "__index");

    lua_pushcfunction(L, l_time_toString);
    lua_setfield(L, -2, "toString");

    lua_pop(L, 1);
}

struct Font {

#pragma pack(push, 1)
    struct Glyph {
        uint32_t code;
        uint16_t atlas_x;
        uint16_t atlas_y;
        uint16_t w;
        uint16_t h;
        int16_t  xOff;
        int16_t  yOff;
        uint16_t advance;
    };
#pragma pack(pop)

    uint8_t* data = nullptr;
    size_t size = 0;

    uint16_t fontSize = 0;
    uint16_t ascent = 0;
    uint16_t glyphCount = 0;
    uint16_t atlasW = 0;
    uint16_t atlasH = 0;

    Glyph* glyphs = nullptr;
    Glyph* lut[256] = {};          // código (0..255) -> glifo, montada no load (evita a busca linear por caractere)
    uint8_t* bitmap = nullptr;

    Font(const char* path){
        load(path);
    }

    bool load(const char* path){

        File f = SD.open(path, FILE_READ);
        if(!f){
            Serial.println("0x010 - Font open error");
            return false;
        }

        size = f.size();

        data = (uint8_t*)heap_caps_malloc(
            size,
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
        );

        if(!data){
            Serial.println("0x011 - Font alloc error");
            f.close();
            return false;
        }

        f.read(data, size);
        f.close();

        uint8_t* ptr = data;

        if(memcmp(ptr, "FNT1", 4) != 0){
            Serial.println("0x012 - Invalid font format");
            return false;
        }

        ptr += 4;

        fontSize   = *(uint16_t*)ptr; ptr += 2;
        ascent     = *(uint16_t*)ptr; ptr += 2;
        glyphCount = *(uint16_t*)ptr; ptr += 2;
        atlasW     = *(uint16_t*)ptr; ptr += 2;
        atlasH     = *(uint16_t*)ptr; ptr += 2;

        glyphs = (Glyph*)ptr;
        ptr += sizeof(Glyph) * glyphCount;

        uint32_t bitmapSize = *(uint32_t*)ptr;
        ptr += 4;

        bitmap = ptr;

        for (int i = 0; i < 256; i++) lut[i] = nullptr;
        for (int i = 0; i < glyphCount; i++) {
            const uint32_t cd = glyphs[i].code;
            if (cd < 256 && !lut[cd]) lut[cd] = &glyphs[i];
        }

        //Serial.printf("Font loaded: %d glyphs | sizeof(Glyph)=%d\n",
        //              glyphCount, sizeof(Glyph));

        return true;
    }

    Glyph* findGlyph(uint32_t code){
        if(code < 256) return lut[code];
        for(int i = 0; i < glyphCount; i++){
            if(glyphs[i].code == code)
                return &glyphs[i];
        }
        return nullptr;
    }

    // Escreve direto no buffer do sprite (16 bpp, bytes trocados = formato do painel) recortando na mão, em vez de um
    // drawPixel() por pixel; a divisão float por pixel virou uma tabela por coluna (1 divisão por coluna/linha do glifo).
    // O dirty-rect é reportado 1x no fim, com a união das células dos glifos.
    void drawString(LGFX_Sprite &spr,
                    const char* text,
                    int x,
                    int y,
                    int pts,
                    int r,
                    int g,
                    int b){

        uint32_t hh = hmix(0x464E5401u, (uint32_t)(uintptr_t)this);
        hh = hmix(hh, (uint32_t)(uintptr_t)data); hh = hmix(hh, fontSize); hh = hmix(hh, glyphCount);
        hh = hmixStr(hh, text);
        hh = hmix(hh, (uint32_t)x); hh = hmix(hh, (uint32_t)y); hh = hmix(hh, (uint32_t)pts);
        hh = hmix(hh, ((uint32_t)(r & 255) << 16) | ((uint32_t)(g & 255) << 8) | (uint32_t)(b & 255));

        if(!data || fontSize == 0 || pts <= 0) { cmdRecord(hh, 0, 0, 0, 0); return; }

        uint16_t* const dst = (uint16_t*)spr.getBuffer();
        if(!dst) { cmdRecord(hh, 0, 0, 0, 0); return; }
        flushClear();
        const int sw = spr.width(), sh = spr.height();
        const uint16_t raw = __builtin_bswap16((uint16_t)tft.color565(r, g, b));
        const float scale = (float)pts / (float)fontSize;

        enum { GLYPH_COL_MAX = 256 };
        uint16_t colMap[GLYPH_COL_MAX];

        int cursorX = x;
        int dx1 = 0x7FFFFFFF, dy1 = 0x7FFFFFFF, dx2 = -1, dy2 = -1;

        while(*text){

            const uint8_t c = *text++;
            const Glyph* gptr = findGlyph(c);

            if(!gptr){
                cursorX += pts / 2;
                continue;
            }

            const int drawX = cursorX + (int)(gptr->xOff * scale);
            const int drawY = y + (int)((ascent + gptr->yOff) * scale);

            int scaledW = (int)(gptr->w * scale);
            const int scaledH = (int)(gptr->h * scale);
            if(scaledW > GLYPH_COL_MAX) scaledW = GLYPH_COL_MAX;

            // recorte contra o sprite
            const int sx0 = drawX < 0 ? -drawX : 0;
            const int sx1 = (drawX + scaledW > sw) ? (sw - drawX) : scaledW;
            const int sy0 = drawY < 0 ? -drawY : 0;
            const int sy1 = (drawY + scaledH > sh) ? (sh - drawY) : scaledH;

            if(sx1 > sx0 && sy1 > sy0){
                for(int sx = sx0; sx < sx1; ++sx)
                    colMap[sx] = (uint16_t)(gptr->atlas_x + (sx * (int)fontSize) / pts);

                for(int sy = sy0; sy < sy1; ++sy){
                    const uint8_t* arow = bitmap + (size_t)(gptr->atlas_y + (sy * (int)fontSize) / pts) * atlasW;
                    uint16_t* drow = dst + (size_t)(drawY + sy) * sw + drawX;
                    for(int sx = sx0; sx < sx1; ++sx){
                        if(arow[colMap[sx]] > 128) drow[sx] = raw;
                    }
                }

                if(drawX + sx0 < dx1) dx1 = drawX + sx0;
                if(drawX + sx1 - 1 > dx2) dx2 = drawX + sx1 - 1;
                if(drawY + sy0 < dy1) dy1 = drawY + sy0;
                if(drawY + sy1 - 1 > dy2) dy2 = drawY + sy1 - 1;
            }

            cursorX += (int)(gptr->advance * scale);
        }

        if(dx2 >= dx1 && dy2 >= dy1){
            cmdRecord(hh, dx1, dy1, dx2 - dx1 + 1, dy2 - dy1 + 1);
        } else {
            cmdRecord(hh, 0, 0, 0, 0);
        }
    }

    // Largura (px) que drawString() vai ocupar com esse texto e tamanho
    int measure(const char* text, int pts){
        if(!data || fontSize == 0) return 0;

        float scale = (float)pts / (float)fontSize;
        int w = 0;

        while(*text){
            uint8_t c = *text++;
            Glyph* gptr = findGlyph(c);

            if(!gptr){
                w += pts / 2;
                continue;
            }

            w += (int)(gptr->advance * scale);
        }
        return w;
    }

    void freeMemory(){
        if(data){
            heap_caps_free(data);
            data = nullptr;
            size = 0;
        }
    }

    ~Font(){
        freeMemory();
    }
};

int l_font_constructor(lua_State* L){

    const char* path = luaL_checkstring(L, 2);

    Font* font = (Font*)lua_newuserdata(L, sizeof(Font));
    new (font) Font(path);

    if(!font->data){
        return luaL_error(L, "Font load failed");
    }

    luaL_getmetatable(L, "Font");
    lua_setmetatable(L, -2);

    return 1;
}

int l_font_draw(lua_State* L){

    Font* font = (Font*)luaL_checkudata(L, 1, "Font");

    const char* text = luaL_checkstring(L, 2);
    int x = luaL_checkinteger(L, 3);
    int y = luaL_checkinteger(L, 4);
    int pts = luaL_checkinteger(L, 5);
    int r = luaL_checkinteger(L, 6);
    int g = luaL_checkinteger(L, 7);
    int b = luaL_checkinteger(L, 8);

    font->drawString(frame, text, x, y, pts, r, g, b);

    return 0;
}

// font:getTextWidth(text, pts) -> integer
int l_font_measure(lua_State* L){

    Font* font = (Font*)luaL_checkudata(L, 1, "Font");
    const char* text = luaL_checkstring(L, 2);
    int pts = (int)luaL_checknumber(L, 3);

    lua_pushinteger(L, font->measure(text, pts));
    return 1;
}

int l_font_gc(lua_State* L){

    Font* font = (Font*)luaL_checkudata(L, 1, "Font");
    font->freeMemory();

    return 0;
}

void registerFont(lua_State* L){
    // metatable das instâncias
    luaL_newmetatable(L, "Font");               // [mt]

    lua_pushcfunction(L, l_font_gc);
    lua_setfield(L, -2, "__gc");                // mt.__gc

    lua_newtable(L);                            // [mt, idx]
    lua_pushcfunction(L, l_font_draw);
    lua_setfield(L, -2, "drawString");
    lua_pushcfunction(L, l_font_measure);
    lua_setfield(L, -2, "getTextWidth");
    lua_setfield(L, -2, "__index");             // mt.__index = idx   -> [mt]
    lua_pop(L, 1);                              // []

    // tabela global Font com __call
    lua_newtable(L);                            // [Font]
    lua_newtable(L);                            // [Font, meta]
    lua_pushcfunction(L, l_font_constructor);
    lua_setfield(L, -2, "__call");              // meta.__call = ctor
    lua_setmetatable(L, -2);                    // setmetatable(Font, meta) -> [Font]
    lua_setglobal(L, "Font");                   // []
}

// ===== R3D ENGINE + LUA BINDINGS (inlined) =====
// =============================================================================
// R3D Engine - extracted from 3d_models.ino, adapted for RazorOS firmware
// Do NOT define LGFX/tft here: uses the global `tft` from firmware.ino
// =============================================================================
#ifndef R3D_ENGINE_INCLUDED
#define R3D_ENGINE_INCLUDED

#include <stdarg.h>
#include <stdio.h>
#include <math.h>
#include <cstdint>
#include <esp_heap_caps.h>
#include <esp_timer.h>

#pragma GCC optimize ("O3")

// ---- engine config & types (from 3d_models) ----
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
#ifndef R3D_FPS_OVERLAY
#define R3D_FPS_OVERLAY 0   // 1 = desenha "FPS:xx.x" no canto do quadro 3D (na tarefa de blit)
#endif
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
#ifndef SD_CS
#define SD_CS 4
#endif
#ifndef SD_SCK
#define SD_SCK 14
#endif
#ifndef SD_MOSI
#define SD_MOSI 13
#endif
#ifndef SD_MISO
#define SD_MISO 12
#endif
#ifndef SD_FREQ
#define SD_FREQ 25000000
#endif
#ifndef TFT_CS_PIN
#define TFT_CS_PIN 10
#endif

#define FILE_GANON    "/ganondorf.rzobj"
#define FILE_LINK     "/link_anim.rzobj"
#define FILE_SKY      "/field.rzobj"

// Tipos ANTES de qualquer função: o Arduino insere protótipos antes da 1ª função do arquivo.
// Vec3, Mat4, Model forward: already defined at top of firmware (Arduino prototypes)

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


// Use firmware global: extern LGFX tft; (already defined in firmware.ino)
extern LGFX tft;

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

// Constantes do triângulo (iguais em todas as linhas) ficam numa struct na pilha do rasterTri: a chamada por linha
// passa só 6 argumentos (Xtensa: a2..a7), em vez de 14 com 8 deles empilhados e relidos. xs/xe vão empacotados.
struct SpanTri { float diw, duw, dvw, dScale; const uint16_t* tex; int tw, th; uintptr_t zd; };   // zd = depth - color (bytes, com wrap)
typedef void (*SpanFn)(uint16_t* crow, int xsxe, float iw, float uw, float vw, const SpanTri* t);
// índice: bit3 TEX | bit2 AFIM | bit1 REPEAT | bit0 KEY
#define SPAN_THUNK(F)                                                                                        \
  static void HOT spanThunk##F(uint16_t* c, int xx, float iw, float uw, float vw, const SpanTri* t)          \
  { fillSpan<((F)&8)!=0,((F)&4)!=0,((F)&2)!=0,((F)&1)!=0,((F)&16)!=0>(c, (uint16_t*)((uintptr_t)c + t->zd),  \
        xx & 0xFFFF, xx >> 16, iw, t->diw, uw, t->duw, vw, t->dvw, t->tex, t->tw, t->th, t->dScale); }
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
  SpanTri st;
  st.diw = diwdx; st.duw = duwdx; st.dvw = dvwdx; st.dScale = T.depthScale;
  st.tex = tdata; st.tw = tw; st.th = th;
  st.zd = (uintptr_t)T.depth - (uintptr_t)T.color;

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
    fn(T.color + y * W, xs | (xe << 16),
       i0 + diwdx*fx + diwdy*fy,
       uw0 + duwdx*fx + duwdy*fy,
       vw0 + dvwdx*fx + dvwdy*fy, &st);
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

  // ---- instâncias ----
  // Uma instância é um Model "clone": compartilha (somente leitura) vértices, triângulos, texturas, esqueleto e clipes
  // com o modelo original e tem só o que muda por instância: transformação, estado de animação, vértices skinned,
  // AABBs dos meshlets (cópia própria se skinned) e as listas de visibilidade/ordem.
  bool isClone = false;
  bool ownMl   = false;          // ml é cópia própria (modelo skinned) e deve ser liberado
  bool isVisible = true;         // false => o draw() ignora este modelo/instância

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
    if (isClone) { unloadClone(); return; }
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

  // Libera só o que pertence à instância; os dados compartilhados continuam com o modelo original.
  void unloadClone() {
    visCount = 0;
    mlCount = 0;
    if (ownMl && ml) heap_caps_free(ml);
    if (order)   heap_caps_free(order);
    if (skey)    heap_caps_free(skey);
    if (vis)     heap_caps_free(vis);
    if (vflag)   heap_caps_free(vflag);
    if (vskin)   heap_caps_free(vskin);
    if (animMem) heap_caps_free(animMem);
    ml = nullptr; ownMl = false;
    order = nullptr; skey = nullptr; vis = nullptr; vflag = nullptr;
    vskin = nullptr; animMem = nullptr; cur = from = out = pose = nullptr;
    vpool = nullptr; tpool = nullptr; vsk = nullptr; vdraw = nullptr; poolV = 0;
    texs = nullptr; texCount = 0;
    bones = nullptr; boneCount = 0; clips = nullptr; clipCount = 0; keyPool = nullptr;
    animPlay = false; blending = false; animClip = -1; animT = 0.0f;
    vertCount = 0; triCount = 0;
    isClone = false;
  }

  // Vira uma instância do modelo `s` (que precisa estar carregado e não ser ele mesmo uma instância).
  // Só aloca o estado por instância. Chame promoteFrameData() depois pra mover os arrays pequenos pra RAM interna.
  bool cloneFrom(const Model& s) {
    if (!s.loaded() || s.isClone) return false;
    isClone = true;
    vertCount = s.vertCount; triCount = s.triCount; cull = s.cull;
    vpool = s.vpool; tpool = s.tpool; mlCount = s.mlCount; poolV = s.poolV;
    texs = s.texs; texCount = s.texCount;
    vsk = s.vsk; bones = s.bones; boneCount = s.boneCount;
    memcpy(rootM, s.rootM, sizeof(rootM));
    clips = s.clips; clipCount = s.clipCount; keyPool = s.keyPool;
    position = {0, 0, 0}; rotation = {0, 0, 0}; scale = {1, 1, 1};
    isVisible = true;

    const size_t n = (size_t)mlCount;
    if (s.vskin) {
      // skinned: AABBs dos meshlets e vértices mudam com a pose => cópia própria
      ml = (Meshlet*)allocFast(sizeof(Meshlet) * n, "meshlets (inst)", false);
      if (!ml) { unloadClone(); return false; }
      ownMl = true;
      memcpy(ml, s.ml, sizeof(Meshlet) * n);
      vskin = (Vtx*)allocFast(sizeof(Vtx) * (size_t)poolV, "vertices skin (inst)", false);
      if (!vskin) { unloadClone(); return false; }
      memcpy(vskin, s.vpool, sizeof(Vtx) * (size_t)poolV);
      vdraw = vskin;
      animMem = (float*)allocFast(sizeof(float) * (size_t)boneCount * (10 * 3 + 12), "pose/anim (inst)", true);
      if (!animMem) { unloadClone(); return false; }
      cur = animMem; from = cur + 10 * boneCount; out = from + 10 * boneCount; pose = out + 10 * boneCount;
    } else {
      ml = s.ml;                      // estático: tudo somente leitura, compartilha
      vdraw = s.vdraw;
    }

    order = (uint16_t*)allocFast(sizeof(uint16_t) * n, "ordem (inst)", false);
    skey  = (float*)   allocFast(sizeof(float)    * n, "chaves z (inst)", false);
    vis   = (uint16_t*)allocFast(sizeof(uint16_t) * n, "visiveis (inst)", false);
    vflag = (uint8_t*) allocFast(n,                    "flags vis. (inst)", false);
    if (!vis || !vflag) { unloadClone(); return false; }
    if (!order || !skey) {
      if (order) heap_caps_free(order);
      if (skey)  heap_caps_free(skey);
      order = nullptr; skey = nullptr;
    } else {
      for (int i = 0; i < mlCount; ++i) order[i] = (uint16_t)i;
    }

    if (boneCount > 0) rebindPose();  // pose de repouso na instância
    return true;
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
    // = T * Rz * Ry * Rx * S, expandido na mão (antes: 6 matMul 4x4 + 3 matrizes de rotação por modelo por quadro)
    const float cx = cosf(rotation.x), sx = sinf(rotation.x);
    const float cy = cosf(rotation.y), sy = sinf(rotation.y);
    const float cz = cosf(rotation.z), sz = sinf(rotation.z);
    const float r00 = cz * cy,  r01 = cz * sy * sx - sz * cx,  r02 = cz * sy * cx + sz * sx;
    const float r10 = sz * cy,  r11 = sz * sy * sx + cz * cx,  r12 = sz * sy * cx - cz * sx;
    const float r20 = -sy,      r21 = cy * sx,                 r22 = cy * cx;
    Mat4 M;
    M.m[0][0] = r00 * scale.x; M.m[0][1] = r01 * scale.y; M.m[0][2] = r02 * scale.z; M.m[0][3] = position.x;
    M.m[1][0] = r10 * scale.x; M.m[1][1] = r11 * scale.y; M.m[1][2] = r12 * scale.z; M.m[1][3] = position.y;
    M.m[2][0] = r20 * scale.x; M.m[2][1] = r21 * scale.y; M.m[2][2] = r22 * scale.z; M.m[2][3] = position.z;
    M.m[3][0] = 0.0f; M.m[3][1] = 0.0f; M.m[3][2] = 0.0f; M.m[3][3] = 1.0f;
    return M;
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
#define R3D_MAX_DRAW 32       // modelos/instâncias desenhados por quadro (fila de desenho)
struct FrameJob {
  uint16_t* color;
  float nearW, depthScale;
  int n;
  Model* model[R3D_MAX_DRAW];
  Mat4 mvp[R3D_MAX_DRAW];
};
static FrameJob gJob;
static int gBands = RENDER_BANDS;     // faixas do quadro atual: RENDER_BANDS com worker, 1 sem (faixa serve só p/ dividir entre núcleos)
static std::atomic<int> gNextBand(0);
static std::atomic<int> gBandsDone(0);
static TaskHandle_t gMasterTask = nullptr;
static TaskHandle_t gWorkerTask = nullptr;

static void renderBands(int ctx) {
  for (;;) {
    const int b = gNextBand.fetch_add(1);
    if (b >= gBands) break;

    // gJob só é lido DEPOIS de reivindicar uma faixa (o mestre publica gJob antes de zerar o contador)
    Target T;
    T.W = HR_W; T.H = HR_H;
    T.y0 = (b * HR_H) / gBands;
    T.y1 = ((b + 1) * HR_H) / gBands;
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

    if (gBandsDone.fetch_add(1) + 1 == gBands) xTaskNotifyGive(gMasterTask);
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

// renderScene replaced by r3dRenderTo

// ---------------- blit (núcleo 0) ----------------
// O LovyanGFX espera o DMA anterior terminar em BUSY-WAIT (laço apertado). Como o transporte de um quadro inteiro
// dura ~30 ms (480*320*16 bits a 80 MHz), a tarefa de blit (prioridade 2) passava quase todo o tempo girando no núcleo 0
// e o worker de raster (prioridade 1) quase nunca rodava. Agora o blit DORME num timer de alta resolução até um
// pouquinho antes do fim estimado do DMA e só gira o restinho. A margem se auto-ajusta.
static TaskHandle_t gR3DBlitTask = nullptr;
static esp_timer_handle_t gR3DBlitTimer = nullptr;
static int gR3DBlitMarginUs = 250;

static void r3dBlitTimerCb(void*) { xTaskNotifyGive(gR3DBlitTask); }

static inline void r3dBlitSleepUntil(int64_t deadlineUs) {
#if BLIT_SLEEP
  if (!gR3DBlitTimer) return;
  const int64_t d = deadlineUs - (int64_t)gR3DBlitMarginUs - esp_timer_get_time();
  if (d > 150 && esp_timer_start_once(gR3DBlitTimer, (uint64_t)d) == ESP_OK)
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
#else
  (void)deadlineUs;
#endif
}

// Escala 2x. Lê 2 pixels por vez (32 bits) e grava as duas linhas de saída no mesmo laço.
static void HOT r3dBlitFrame(const uint16_t* src) {
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
    if (tPrev) r3dBlitSleepUntil(tPrev + usPrev);       // dorme enquanto o DMA do bloco anterior corre
    const int64_t t1 = esp_timer_get_time();
    const int pixels = rows * 2 * SCR_W;
    tft.writePixelsDMA((const lgfx::swap565_t*)dmaBuf[cur], pixels);   // (espera o resto do bloco anterior)
    const int64_t t2 = esp_timer_get_time();
    if (tPrev) {                                      // auto-ajuste da margem de despertar
      const int64_t blocked = t2 - t1;
      if (blocked < 40)       { if (gR3DBlitMarginUs < 1200) gR3DBlitMarginUs += 100; }   // acordou tarde: SPI ficou ocioso
      else if (blocked > 450) { if (gR3DBlitMarginUs > 120)  gR3DBlitMarginUs -= 30;  }   // acordou cedo demais: girando à toa
    }
    tPrev = t2;
    usPrev = ((int64_t)pixels * 16 * 1000000) / SPI_WRITE_HZ;      // 16 bits por pixel
    cur ^= 1;
  }
  if (tPrev) r3dBlitSleepUntil(tPrev + usPrev);          // idem pro último bloco; o waitDMA() do chamador gira o resto
}

// ---- overlay de FPS desenhado NO quadro (fonte 3x5), em vez de tft.print() entre quadros ----
// tft.print() depois de cada quadro abria uma transação SPI própria (janela + 1 DMA/glifo): ~0,3-0,5 ms de barramento
// parado por quadro, justo no gargalo (o SPI), e o texto piscava porque o quadro seguinte o sobrescrevia.
#if R3D_FPS_OVERLAY
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
#endif

static void r3dBlitTask(void*) {
  gR3DBlitTask = xTaskGetCurrentTaskHandle();
#if BLIT_SLEEP
  esp_timer_create_args_t ta = {};
  ta.callback = r3dBlitTimerCb;
  ta.name = "blit3d";
  if (esp_timer_create(&ta, &gR3DBlitTimer) != ESP_OK) gR3DBlitTimer = nullptr;   // sem timer => volta ao busy-wait
#endif
  uint32_t frames = 0;
  unsigned long t0 = millis();
  for (;;) {
    int idx;
    xQueueReceive(fullQ, &idx, portMAX_DELAY);
    xSemaphoreTake(gPresentMutex, portMAX_DELAY);   // o painel e dividido com o motor de tela 2D e o kernel
#if R3D_FPS_OVERLAY
    drawFpsOverlay(colorBuf[idx], gFpsTxt);      // o buffer é nosso até devolvermos em freeQ
#endif
    r3dBlitFrame(colorBuf[idx]);
    xQueueSend(freeQ, &idx, portMAX_DELAY);      // r3dBlitFrame já copiou tudo p/ os buffers de DMA: o Lua pode reusar o quadro
    tft.waitDMA();
    tft.endWrite();
    xSemaphoreGive(gPresentMutex);
    gR3DPending--;

    frames++;
    unsigned long now = millis();
    if (now - t0 >= 1000) {
      gShownFps = frames * 1000.0f / (now - t0);
#if R3D_FPS_OVERLAY
      snprintf(gFpsTxt, sizeof(gFpsTxt), "FPS:%.1f", (double)gShownFps);   // formata 1x por segundo, não por quadro
#endif
      frames = 0; t0 = now;
    }
  }
}

// SD loading is done by Model::loadRzobj via Lua (firmware already mounted SD)

// ---------------- setup / loop ----------------

// ---- R3D public control API (called from Lua bindings) ----
static bool gR3DReady = false;
static bool gR3DAsync = false;      // true = 2 buffers de cor + tarefa de blit 3D (raster do quadro N+1 durante o SPI do N)
static int  gR3DHeld = -1;         // buffer de cor 3D com o Lua (Render3D(false) ainda sem Composite3DToFrame)
static int  gR3DLast = 0;           // buffer de cor do ÚLTIMO quadro renderizado (Composite3DToFrame lê daqui)
static Model* gR3DQueue[R3D_MAX_DRAW];
static int gR3DQueueN = 0;

static void r3dReleaseBuf(int idx) {
  if (freeQ && idx >= 0 && idx < 2) xQueueSend(freeQ, &idx, 0);
}
static const uint16_t* r3dLayerPtr(int idx, int* w, int* h) {
  if (idx < 0 || idx > 1 || !colorBuf[idx]) return nullptr;
  *w = HR_W; *h = HR_H;
  return colorBuf[idx];
}
// devolve ao pool o buffer que o Lua segurou sem usar (Render3D(false) sem Composite3DToFrame)
static void r3dDropHeld() {
  if (gR3DHeld >= 0) { r3dReleaseBuf(gR3DHeld); gR3DHeld = -1; }
}
// um app novo não pode herdar buffers presos pelo anterior
static void r3dResetLayers() {
  r3dDropHeld();
  if (gLayerSrcIdx >= 0) { r3dReleaseBuf(gLayerSrcIdx); gLayerSrcIdx = -1; }
  gLastLayerIdx = -1;
}

static bool r3dInit(int renderDiv, bool dualCore) {
  if (gR3DReady) { r3dResetLayers(); return true; }
  Serial.printf("[R3D] init div=%d dual=%d free heap=%u free psram=%u\n",
                renderDiv, (int)dualCore,
                (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getFreePsram());

  SCR_W = tft.width();
  SCR_H = tft.height();
  HR_W = SCR_W / (renderDiv > 0 ? renderDiv : RENDER_DIV);
  HR_H = SCR_H / (renderDiv > 0 ? renderDiv : RENDER_DIV);
  if (HR_W < 8 || HR_H < 8) {
    Serial.println("[R3D] bad resolution");
    return false;
  }

  initInvTable();

  const size_t total = (size_t)HR_W * HR_H;
  const size_t dmaBytes = (size_t)CHUNK_HR * 2 * SCR_W * 2;
  // DMA scratch must be internal; frame color/z can be PSRAM
  // Os blocos de DMA do motor de tela 2D (mesmo tamanho, RAM interna) servem aqui também: o acesso ao painel já é
  // serializado por gPresentMutex, então não precisa de mais 30 KB de RAM interna.
  if (gDmaBuf[0] && gDmaBuf[1] && dmaBytes <= (size_t)DMA_CHUNK_PIXELS * 2) {
    dmaBuf[0] = gDmaBuf[0];
    dmaBuf[1] = gDmaBuf[1];
  } else {
    dmaBuf[0] = (uint16_t*)heap_caps_malloc(dmaBytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    dmaBuf[1] = (uint16_t*)heap_caps_malloc(dmaBytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  }
  zBuf        = (uint16_t*)allocFast(total * 2, "r3d zbuffer", false);  // PSRAM: internal is tight with OS framebufs
  colorBuf[0] = (uint16_t*)allocFast(total * 2, "r3d color A", false);
  colorBuf[1] = (uint16_t*)heap_caps_malloc(total * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);   // 2o quadro: permite o blit assíncrono
  if (!colorBuf[1]) colorBuf[1] = colorBuf[0];   // sem PSRAM: cai no caminho síncrono (1 buffer)
  if (!zBuf || !colorBuf[0] || !dmaBuf[0] || !dmaBuf[1]) {
    Serial.println("[R3D] buffer alloc FAILED");
    errf("r3dInit: buffer alloc failed");
    return false;
  }

  freeQ = xQueueCreate(2, sizeof(int));
  fullQ = xQueueCreate(2, sizeof(int));
  gR3DAsync = false;
  if (freeQ && fullQ && colorBuf[1] != colorBuf[0] && gPresentMutex) {
    for (int i = 0; i < 2; ++i) xQueueSend(freeQ, &i, 0);
    gR3DAsync = (xTaskCreatePinnedToCore(r3dBlitTask, "R3DBLIT", 4096, nullptr, 2, nullptr, 0) == pdPASS);
    if (!gR3DAsync) Serial.println("[R3D] WARN: blit task not created, sync present");
  }

  gMasterTask = xTaskGetCurrentTaskHandle();
  gWorkerTask = nullptr;
#if RENDER_DUALCORE
  if (dualCore) {
    BaseType_t ok = xTaskCreatePinnedToCore(rasterWorker, "r3d_raster", 8192, nullptr, 1, &gWorkerTask, 0);
    if (ok != pdPASS) {
      Serial.println("[R3D] WARN: raster worker not created, single-core");
      gWorkerTask = nullptr;
    }
  }
#endif
  gR3DReady = true;
  gLastUs = esp_timer_get_time();
  Serial.printf("[R3D] ready %dx%d -> %dx%d bands=%d dual=%d async=%d heap=%u psram=%u\n",
                HR_W, HR_H, SCR_W, SCR_H, RENDER_BANDS, (int)(gWorkerTask != nullptr), (int)gR3DAsync,
                (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getFreePsram());
  return true;
}

static void r3dQueueClear() { gR3DQueueN = 0; }
static bool r3dQueueModel(Model* m) {
  if (!m || !m->loaded() || gR3DQueueN >= R3D_MAX_DRAW) return false;
  gR3DQueue[gR3DQueueN++] = m;
  return true;
}

// Renders queued models into colorBuf[idx], using current camera
static void r3dRenderTo(uint16_t* color) {
  gMasterTask = xTaskGetCurrentTaskHandle();  // notify must reach the task that waits below
  Mat4 view = camera.getViewMatrix();
  Mat4 proj = perspective(camera.fov, (float)HR_W/(float)HR_H, camera.nearZ, camera.farZ);
  Mat4 VP = matMul(proj, view);

  gJob.color = color;
  gJob.nearW = camera.nearZ;
  gJob.depthScale = 60000.0f * camera.nearZ;
  int n = 0;
  for (int i = 0; i < gR3DQueueN && n < R3D_MAX_DRAW; ++i) {
    gJob.model[n] = gR3DQueue[i];
    gJob.mvp[n] = matMul(VP, gR3DQueue[i]->getModelMatrix());
    n++;
  }
    gJob.n = n;

  for (int i = 0; i < n; ++i) gJob.model[i]->prepare(gJob.mvp[i], camera.nearZ);

  gBands = gWorkerTask ? RENDER_BANDS : 1;   // publicado antes de liberar o contador
  gBandsDone.store(0);
  gNextBand.store(0);
#if RENDER_DUALCORE
  if (gWorkerTask) xTaskNotifyGive(gWorkerTask);
#endif
  renderBands(0);
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}

static void r3dPresent(uint16_t* color) {
  r3dBlitFrame(color);
  tft.waitDMA();
  tft.endWrite();
}

#endif // R3D_ENGINE_INCLUDED


// =============================================================================
// R3D Lua bindings for RazorOS
// Include AFTER R3D_Engine.inc and AFTER lua headers / registerApis helpers
// =============================================================================

#define LUA_MODEL3D_MT "Model3D"
#define LUA_CAMERA3D_MT "Camera3D"
#define LUA_MODEL3D_INST_MT "Model3DInstance"
#define LUA_MODEL3D_INSTLIST_MT "Model3DInstances"
#define R3D_MAX_INSTANCES 32

// ---- Model3D userdata: owns a heap Model* so large object is not on Lua stack ----
struct LuaModel3D {
  Model* model;
  Model* inst[R3D_MAX_INSTANCES];   // instâncias criadas com :createInstance() (índice Lua 0-based = índice aqui)
  int instCount;
};

// model.instances[i] -> referência a uma instância (mantém o Model3D dono vivo via uservalue)
struct LuaModelInst {
  LuaModel3D* parent;
  int index;
};

// model.instances -> lista (suporta [i] 0-based e #)
struct LuaInstList {
  LuaModel3D* parent;
};


static int r3d_stricmp(const char* a, const char* b) {
  while (*a && *b) {
    char ca = *a, cb = *b;
    if (ca >= 'A' && ca <= 'Z') ca = (char)(ca - 'A' + 'a');
    if (cb >= 'A' && cb <= 'Z') cb = (char)(cb - 'A' + 'a');
    if (ca != cb) return (int)(unsigned char)ca - (int)(unsigned char)cb;
    ++a; ++b;
  }
  return (int)(unsigned char)*a - (int)(unsigned char)*b;
}

LuaModel3D* checkModel3D(lua_State* L, int idx) {
  return (LuaModel3D*)luaL_checkudata(L, idx, LUA_MODEL3D_MT);
}

// Aceita um Model3D OU uma instância (os mesmos métodos servem pros dois)
Model* checkModelPtr(lua_State* L, int idx) {
  LuaModelInst* r = (LuaModelInst*)luaL_testudata(L, idx, LUA_MODEL3D_INST_MT);
  if (r) {
    Model* im = (r->parent && r->index >= 0 && r->index < r->parent->instCount) ? r->parent->inst[r->index] : nullptr;
    if (!im) luaL_error(L, "Model3D instance no longer exists");
    return im;
  }
  LuaModel3D* u = checkModel3D(L, idx);
  if (!u->model) luaL_error(L, "Model3D is unloaded");
  return u->model;
}

// Libera todas as instâncias de um Model3D (tem que acontecer antes de descarregar/recarregar o modelo-base,
// porque elas apontam pros dados dele). Também tira da fila de desenho qualquer uma que ficou lá.
static void model3dDestroyInstances(LuaModel3D* u) {
  for (int i = 0; i < u->instCount; ++i) {
    Model* m = u->inst[i];
    if (!m) continue;
    int k = 0;
    for (int q = 0; q < gR3DQueueN; ++q) if (gR3DQueue[q] != m) gR3DQueue[k++] = gR3DQueue[q];
    gR3DQueueN = k;
    m->unload();
    m->~Model();
    heap_caps_free(m);
    u->inst[i] = nullptr;
  }
  u->instCount = 0;
}

// Quem recebe a operação: uma instância => só ela; Model3D com instâncias => todas; Model3D sem instâncias => o próprio.
static int model3dTargets(lua_State* L, int idx, Model** out) {
  if (luaL_testudata(L, idx, LUA_MODEL3D_INST_MT)) { out[0] = checkModelPtr(L, idx); return 1; }
  LuaModel3D* u = checkModel3D(L, idx);
  if (!u->model) luaL_error(L, "Model3D is unloaded");
  if (u->instCount > 0) {
    int n = 0;
    for (int i = 0; i < u->instCount; ++i) if (u->inst[i]) out[n++] = u->inst[i];
    return n;
  }
  out[0] = u->model;
  return 1;
}

// {x, y, z} -> Vec3 (campos ausentes ficam como estão em `v`)
static void model3dReadVec3(lua_State* L, int idx, Vec3& v) {
  luaL_checktype(L, idx, LUA_TTABLE);
  float* c[3] = { &v.x, &v.y, &v.z };
  for (int i = 0; i < 3; ++i) {
    lua_rawgeti(L, idx, i + 1);
    if (lua_isnumber(L, -1)) *c[i] = (float)lua_tonumber(L, -1);
    lua_pop(L, 1);
  }
}

static void model3dPushInstanceRef(lua_State* L, int parentIdx, int index) {
  parentIdx = lua_absindex(L, parentIdx);
  LuaModelInst* r = (LuaModelInst*)lua_newuserdatauv(L, sizeof(LuaModelInst), 1);
  r->parent = (LuaModel3D*)lua_touserdata(L, parentIdx);
  r->index = index;
  lua_pushvalue(L, parentIdx);
  lua_setiuservalue(L, -2, 1);          // a instância mantém o Model3D dono vivo
  luaL_setmetatable(L, LUA_MODEL3D_INST_MT);
}

// Model3D(path)  or  Model3D() then :load(path)
int l_model3d_new(lua_State* L) {
  LuaModel3D* u = (LuaModel3D*)lua_newuserdata(L, sizeof(LuaModel3D));
  memset(u, 0, sizeof(LuaModel3D));      // model = nullptr, instCount = 0, inst[] = nullptr
  luaL_getmetatable(L, LUA_MODEL3D_MT);
  lua_setmetatable(L, -2);

  void* mem = heap_caps_malloc(sizeof(Model), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!mem) return luaL_error(L, "Model3D: out of memory");
  u->model = new (mem) Model();

  if (lua_gettop(L) >= 2 && lua_isstring(L, 2)) {
    const char* path = lua_tostring(L, 2);
    Serial.printf("[R3D] loading %s ...\n", path);
    // SPI bus shared with TFT: hold present mutex + TFT CS high while reading SD
    if (gPresentMutex) xSemaphoreTake(gPresentMutex, portMAX_DELAY);
    pinMode(10, OUTPUT);
    digitalWrite(10, HIGH);
    bool ok = u->model->loadRzobj(path);
    if (gPresentMutex) xSemaphoreGive(gPresentMutex);
    if (!ok) {
      return luaL_error(L, "Model3D load failed: %s", path);
    }
    Serial.printf("[R3D] loaded %s, promoting...\n", path);
    u->model->promoteFrameData();
    u->model->promoteTextures();
    Serial.printf("[R3D] ready %s\n", path);
  }
  return 1;
}

int l_model3d_load(lua_State* L) {
  LuaModel3D* u = checkModel3D(L, 1);
  const char* path = luaL_checkstring(L, 2);
  if (!u->model) {
    void* mem = heap_caps_malloc(sizeof(Model), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!mem) return luaL_error(L, "Model3D: out of memory");
    u->model = new (mem) Model();
  }
  model3dDestroyInstances(u);            // instâncias apontam pros dados do modelo antigo
  Serial.printf("[R3D] loading %s ...\n", path);
  if (gPresentMutex) xSemaphoreTake(gPresentMutex, portMAX_DELAY);
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  bool ok = u->model->loadRzobj(path);
  if (gPresentMutex) xSemaphoreGive(gPresentMutex);
  if (!ok) {
    lua_pushboolean(L, 0);
    return 1;
  }
  u->model->promoteFrameData();
  u->model->promoteTextures();
  Serial.printf("[R3D] ready %s\n", path);
  lua_pushboolean(L, 1);
  return 1;
}

int l_model3d_unload(lua_State* L) {
  LuaModel3D* u = checkModel3D(L, 1);
  model3dDestroyInstances(u);
  if (u->model) {
    u->model->unload();
  }
  return 0;
}

int l_model3d_gc(lua_State* L) {
  LuaModel3D* u = checkModel3D(L, 1);
  model3dDestroyInstances(u);
  if (u->model) {
    u->model->unload();
    u->model->~Model();
    heap_caps_free(u->model);
    u->model = nullptr;
  }
  return 0;
}

int l_model3d_loaded(lua_State* L) {
  LuaModel3D* u = checkModel3D(L, 1);
  lua_pushboolean(L, u->model && u->model->loaded());
  return 1;
}

int l_model3d_setPosition(lua_State* L) {
  Model* m = checkModelPtr(L, 1);
  m->position.x = (float)luaL_checknumber(L, 2);
  m->position.y = (float)luaL_checknumber(L, 3);
  m->position.z = (float)luaL_checknumber(L, 4);
  return 0;
}

int l_model3d_getPosition(lua_State* L) {
  Model* m = checkModelPtr(L, 1);
  lua_pushnumber(L, m->position.x);
  lua_pushnumber(L, m->position.y);
  lua_pushnumber(L, m->position.z);
  return 3;
}

int l_model3d_setRotation(lua_State* L) {
  Model* m = checkModelPtr(L, 1);
  m->rotation.x = (float)luaL_checknumber(L, 2);
  m->rotation.y = (float)luaL_checknumber(L, 3);
  m->rotation.z = (float)luaL_checknumber(L, 4);
  return 0;
}

int l_model3d_getRotation(lua_State* L) {
  Model* m = checkModelPtr(L, 1);
  lua_pushnumber(L, m->rotation.x);
  lua_pushnumber(L, m->rotation.y);
  lua_pushnumber(L, m->rotation.z);
  return 3;
}

int l_model3d_setScale(lua_State* L) {
  Model* m = checkModelPtr(L, 1);
  if (lua_gettop(L) >= 4) {
    m->scale.x = (float)luaL_checknumber(L, 2);
    m->scale.y = (float)luaL_checknumber(L, 3);
    m->scale.z = (float)luaL_checknumber(L, 4);
  } else {
    float s = (float)luaL_checknumber(L, 2);
    m->scale = {s, s, s};
  }
  return 0;
}

int l_model3d_getScale(lua_State* L) {
  Model* m = checkModelPtr(L, 1);
  lua_pushnumber(L, m->scale.x);
  lua_pushnumber(L, m->scale.y);
  lua_pushnumber(L, m->scale.z);
  return 3;
}

// setCull("none"|"pos"|"neg") or integer 0/1/-1
// Num Model3D com instâncias vale pra TODAS; numa instância (model.instances[i]) vale só pra ela.
int l_model3d_setCull(lua_State* L) {
  Model* t[R3D_MAX_INSTANCES + 1];
  const int n = model3dTargets(L, 1, t);
  int cull;
  if (lua_isstring(L, 2)) {
    const char* s = lua_tostring(L, 2);
    if (!r3d_stricmp(s, "none")) cull = CULL_NONE;
    else if (!r3d_stricmp(s, "pos")) cull = CULL_POS_AREA;
    else if (!r3d_stricmp(s, "neg")) cull = CULL_NEG_AREA;
    else return luaL_error(L, "setCull: use none/pos/neg");
  } else {
    cull = (int)luaL_checkinteger(L, 2);
  }
  // o modelo-base também guarda o valor: instâncias criadas depois herdam
  if (!luaL_testudata(L, 1, LUA_MODEL3D_INST_MT)) checkModelPtr(L, 1)->cull = cull;
  for (int i = 0; i < n; ++i) t[i]->cull = cull;
  return 0;
}

int l_model3d_playAnim(lua_State* L) {
  Model* t[R3D_MAX_INSTANCES + 1];
  const int n = model3dTargets(L, 1, t);
  bool loop   = lua_isnoneornil(L, 3) ? true : lua_toboolean(L, 3);
  float speed = (float)luaL_optnumber(L, 4, 1.0);
  float blend = (float)luaL_optnumber(L, 5, 0.15);

  const bool byName = (lua_type(L, 2) == LUA_TSTRING);
  const char* name = byName ? lua_tostring(L, 2) : nullptr;
  const int clip = byName ? 0 : (int)luaL_checkinteger(L, 2);

  bool ok = (n > 0);
  for (int i = 0; i < n; ++i) {
    const bool r = byName ? t[i]->playAnim(name, loop, speed, blend) : t[i]->playAnim(clip, loop, speed, blend);
    ok = ok && r;
  }
  lua_pushboolean(L, ok);
  return 1;
}

int l_model3d_stopAnim(lua_State* L) {
  Model* t[R3D_MAX_INSTANCES + 1];
  const int n = model3dTargets(L, 1, t);
  bool reset = lua_isnoneornil(L, 2) ? true : lua_toboolean(L, 2);
  for (int i = 0; i < n; ++i) t[i]->stopAnim(reset);
  return 0;
}

int l_model3d_updateAnim(lua_State* L) {
  Model* t[R3D_MAX_INSTANCES + 1];
  const int n = model3dTargets(L, 1, t);
  float dt = (float)luaL_checknumber(L, 2);
  for (int i = 0; i < n; ++i) t[i]->updateAnim(dt);
  return 0;
}

// true se alguma (instância) estiver tocando
int l_model3d_isPlaying(lua_State* L) {
  Model* t[R3D_MAX_INSTANCES + 1];
  const int n = model3dTargets(L, 1, t);
  bool any = false;
  for (int i = 0; i < n; ++i) any = any || t[i]->animPlaying();
  lua_pushboolean(L, any);
  return 1;
}

int l_model3d_currentAnim(lua_State* L) {
  Model* m = checkModelPtr(L, 1);
  lua_pushinteger(L, m->currentAnim());
  return 1;
}

int l_model3d_animCount(lua_State* L) {
  Model* m = checkModelPtr(L, 1);
  lua_pushinteger(L, m->animCount());
  return 1;
}

int l_model3d_animName(lua_State* L) {
  Model* m = checkModelPtr(L, 1);
  int i = (int)luaL_checkinteger(L, 2);
  const char* n = m->animName(i);
  if (n) lua_pushstring(L, n); else lua_pushnil(L);
  return 1;
}

int l_model3d_findAnim(lua_State* L) {
  Model* m = checkModelPtr(L, 1);
  lua_pushinteger(L, m->findAnim(luaL_checkstring(L, 2)));
  return 1;
}

int l_model3d_skinned(lua_State* L) {
  Model* m = checkModelPtr(L, 1);
  lua_pushboolean(L, m->skinned());
  return 1;
}

// Queues model for the next Render3D()
// Num Model3D com instâncias desenha todas as visíveis; numa instância desenha só ela (se visível).
int l_model3d_draw(lua_State* L) {
  Model* t[R3D_MAX_INSTANCES + 1];
  const int n = model3dTargets(L, 1, t);
  if (!gR3DReady) return luaL_error(L, "Init3DRenderer() first");
  // Model3D sem nenhuma instância não desenha nada (o modelo-base só guarda os dados compartilhados)
  LuaModel3D* owner = (LuaModel3D*)luaL_testudata(L, 1, LUA_MODEL3D_MT);
  if (owner && owner->instCount == 0) {
    lua_pushboolean(L, 0);
    return 1;
  }
  bool any = false;
  for (int i = 0; i < n; ++i) {
    if (t[i]->isVisible) any = r3dQueueModel(t[i]) || any;
  }
  lua_pushboolean(L, any);
  return 1;
}

// model:createInstance(position, scale) -> instance
//   position = {x, y, z}  (padrão: posição do modelo-base)
//   scale    = {sx, sy, sz} ou um número (escala uniforme)  (padrão: escala do modelo-base)
// A instância herda o cull do modelo-base. Acesso: model.instances[0], model.instances[1], ... (#model.instances = quantidade)
int l_model3d_createInstance(lua_State* L) {
  LuaModel3D* u = checkModel3D(L, 1);
  if (!u->model || !u->model->loaded()) return luaL_error(L, "createInstance: model not loaded");
  if (u->instCount >= R3D_MAX_INSTANCES) return luaL_error(L, "createInstance: too many instances (max %d)", R3D_MAX_INSTANCES);

  Vec3 pos = u->model->position;
  Vec3 scl = u->model->scale;
  if (!lua_isnoneornil(L, 2)) model3dReadVec3(L, 2, pos);
  if (!lua_isnoneornil(L, 3)) {
    if (lua_istable(L, 3)) model3dReadVec3(L, 3, scl);
    else { const float s = (float)luaL_checknumber(L, 3); scl = {s, s, s}; }
  }

  void* mem = heap_caps_malloc(sizeof(Model), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!mem) return luaL_error(L, "createInstance: out of memory");
  Model* m = new (mem) Model();
  if (!m->cloneFrom(*u->model)) {
    m->~Model();
    heap_caps_free(mem);
    return luaL_error(L, "createInstance: out of memory");
  }
  m->position = pos;
  m->scale = scl;
  m->promoteFrameData();

  const int index = u->instCount;
  u->inst[u->instCount++] = m;
  model3dPushInstanceRef(L, 1, index);
  return 1;
}

// model:clearInstances()  - remove todas as instâncias (o modelo-base continua carregado)
int l_model3d_clearInstances(lua_State* L) {
  LuaModel3D* u = checkModel3D(L, 1);
  model3dDestroyInstances(u);
  return 0;
}

// model:getInstanceCount()
int l_model3d_getInstanceCount(lua_State* L) {
  LuaModel3D* u = checkModel3D(L, 1);
  lua_pushinteger(L, u->instCount);
  return 1;
}

// ---- metamétodos ----
// Model3D.__index: "instances" devolve a lista; o resto vem da tabela de métodos (upvalue 1)
static int l_model3d_index(lua_State* L) {
  LuaModel3D* u = checkModel3D(L, 1);
  if (lua_type(L, 2) == LUA_TSTRING && strcmp(lua_tostring(L, 2), "instances") == 0) {
    lua_getiuservalue(L, 1, 1);                 // lista em cache no próprio userdata
    if (lua_type(L, -1) != LUA_TUSERDATA) {
      lua_pop(L, 1);
      LuaInstList* l = (LuaInstList*)lua_newuserdatauv(L, sizeof(LuaInstList), 1);
      l->parent = u;
      lua_pushvalue(L, 1);
      lua_setiuservalue(L, -2, 1);              // mantém o Model3D vivo (o ciclo é coletado normalmente pelo GC)
      luaL_setmetatable(L, LUA_MODEL3D_INSTLIST_MT);
      lua_pushvalue(L, -1);
      lua_setiuservalue(L, 1, 1);
    }
    return 1;
  }
  lua_pushvalue(L, 2);
  lua_gettable(L, lua_upvalueindex(1));
  return 1;
}

// instances[i] (0-based) -> instância | nil
static int l_instlist_index(lua_State* L) {
  LuaInstList* l = (LuaInstList*)luaL_checkudata(L, 1, LUA_MODEL3D_INSTLIST_MT);
  if (lua_type(L, 2) == LUA_TNUMBER) {
    int isnum = 0;
    const lua_Integer i = lua_tointegerx(L, 2, &isnum);
    if (isnum && i >= 0 && i < l->parent->instCount && l->parent->inst[i]) {
      // o userdata dono é o uservalue da lista
      lua_getiuservalue(L, 1, 1);
      model3dPushInstanceRef(L, -1, (int)i);
      return 1;
    }
  }
  lua_pushnil(L);
  return 1;
}

// #instances
static int l_instlist_len(lua_State* L) {
  LuaInstList* l = (LuaInstList*)luaL_checkudata(L, 1, LUA_MODEL3D_INSTLIST_MT);
  lua_pushinteger(L, l->parent->instCount);
  return 1;
}

// instance.isVisible / métodos
static int l_inst_index(lua_State* L) {
  Model* m = checkModelPtr(L, 1);
  if (lua_type(L, 2) == LUA_TSTRING && strcmp(lua_tostring(L, 2), "isVisible") == 0) {
    lua_pushboolean(L, m->isVisible);
    return 1;
  }
  lua_pushvalue(L, 2);
  lua_gettable(L, lua_upvalueindex(1));
  return 1;
}

// instance.isVisible = true/false
static int l_inst_newindex(lua_State* L) {
  Model* m = checkModelPtr(L, 1);
  const char* k = luaL_checkstring(L, 2);
  if (strcmp(k, "isVisible") == 0) {
    luaL_checkany(L, 3);
    m->isVisible = lua_toboolean(L, 3) != 0;
    return 0;
  }
  return luaL_error(L, "Model3D instance has no writable field '%s'", k);
}

void registerModel3D(lua_State* L) {
  // Espelha registerSprite() exatamente — se ainda abortar, o problema nao e a API Model3D
  Serial.printf("[R3D] top before=%d\n", lua_gettop(L)); Serial.flush();

  Serial.println("[R3D] luaL_newmetatable..."); Serial.flush();
  luaL_newmetatable(L, "Model3D");
  Serial.printf("[R3D] after newmetatable top=%d\n", lua_gettop(L)); Serial.flush();

  Serial.println("[R3D] push gc..."); Serial.flush();
  lua_pushcfunction(L, l_model3d_gc);
  Serial.println("[R3D] setfield __gc..."); Serial.flush();
  lua_setfield(L, -2, "__gc");

  Serial.println("[R3D] index table..."); Serial.flush();
  lua_newtable(L);
  lua_pushcfunction(L, l_model3d_load);        lua_setfield(L, -2, "load");
  lua_pushcfunction(L, l_model3d_unload);      lua_setfield(L, -2, "unload");
  lua_pushcfunction(L, l_model3d_loaded);      lua_setfield(L, -2, "loaded");
  lua_pushcfunction(L, l_model3d_setPosition); lua_setfield(L, -2, "setPosition");
  lua_pushcfunction(L, l_model3d_getPosition); lua_setfield(L, -2, "getPosition");
  lua_pushcfunction(L, l_model3d_setRotation); lua_setfield(L, -2, "setRotation");
  lua_pushcfunction(L, l_model3d_getRotation); lua_setfield(L, -2, "getRotation");
  lua_pushcfunction(L, l_model3d_setScale);    lua_setfield(L, -2, "setScale");
  lua_pushcfunction(L, l_model3d_getScale);    lua_setfield(L, -2, "getScale");
  lua_pushcfunction(L, l_model3d_setCull);     lua_setfield(L, -2, "setCull");
  lua_pushcfunction(L, l_model3d_playAnim);    lua_setfield(L, -2, "playAnim");
  lua_pushcfunction(L, l_model3d_stopAnim);    lua_setfield(L, -2, "stopAnim");
  lua_pushcfunction(L, l_model3d_updateAnim);  lua_setfield(L, -2, "updateAnim");
  lua_pushcfunction(L, l_model3d_isPlaying);   lua_setfield(L, -2, "isPlaying");
  lua_pushcfunction(L, l_model3d_currentAnim); lua_setfield(L, -2, "currentAnim");
  lua_pushcfunction(L, l_model3d_animCount);   lua_setfield(L, -2, "animCount");
  lua_pushcfunction(L, l_model3d_animName);    lua_setfield(L, -2, "animName");
  lua_pushcfunction(L, l_model3d_findAnim);    lua_setfield(L, -2, "findAnim");
  lua_pushcfunction(L, l_model3d_skinned);     lua_setfield(L, -2, "skinned");
  lua_pushcfunction(L, l_model3d_draw);        lua_setfield(L, -2, "draw");
  lua_pushcfunction(L, l_model3d_createInstance);   lua_setfield(L, -2, "createInstance");
  lua_pushcfunction(L, l_model3d_clearInstances);   lua_setfield(L, -2, "clearInstances");
  lua_pushcfunction(L, l_model3d_getInstanceCount); lua_setfield(L, -2, "getInstanceCount");
  // stack: [Model3D mt, methods]
  lua_pushvalue(L, -1);                              // [mt, methods, methods]
  lua_pushcclosure(L, l_model3d_index, 1);           // [mt, methods, index_fn]  (methods = upvalue)
  lua_setfield(L, -3, "__index");                    // mt.__index = index_fn -> [mt, methods]

  // metatable das instâncias: mesmos métodos + propriedade isVisible
  luaL_newmetatable(L, LUA_MODEL3D_INST_MT);         // [mt, methods, imt]
  lua_pushvalue(L, -2);                              // [mt, methods, imt, methods]
  lua_pushcclosure(L, l_inst_index, 1);              // [mt, methods, imt, fn]
  lua_setfield(L, -2, "__index");
  lua_pushcfunction(L, l_inst_newindex);
  lua_setfield(L, -2, "__newindex");
  lua_pop(L, 1);                                     // [mt, methods]

  // metatable da lista model.instances
  luaL_newmetatable(L, LUA_MODEL3D_INSTLIST_MT);
  lua_pushcfunction(L, l_instlist_index);
  lua_setfield(L, -2, "__index");
  lua_pushcfunction(L, l_instlist_len);
  lua_setfield(L, -2, "__len");
  lua_pop(L, 1);

  lua_pop(L, 2);                                     // []

  Serial.println("[R3D] ctor table..."); Serial.flush();
  lua_newtable(L);
  lua_newtable(L);
  lua_pushcfunction(L, l_model3d_new);
  lua_setfield(L, -2, "__call");
  lua_setmetatable(L, -2);
  lua_setglobal(L, "Model3D");
  Serial.printf("[R3D] Model3D ok top=%d\n", lua_gettop(L)); Serial.flush();
}

// ---- Camera (singleton exposed as table of functions + optional userdata) ----
int l_Init3DRenderer(lua_State* L) {
  int div = 2, bands = 6;
  bool dual = false;  // safer default; enable dualCore=true after stable
  if (lua_istable(L, 1)) {
    lua_getfield(L, 1, "renderDiv"); if (lua_isnumber(L, -1)) div = (int)lua_tointeger(L, -1); lua_pop(L, 1);
    lua_getfield(L, 1, "bands"); if (lua_isnumber(L, -1)) bands = (int)lua_tointeger(L, -1); lua_pop(L, 1);
    lua_getfield(L, 1, "dualCore"); if (!lua_isnil(L, -1)) dual = lua_toboolean(L, -1); lua_pop(L, 1);
  }
  (void)bands; // compile-time RENDER_BANDS; documented for future dynamic use
  bool ok = r3dInit(div, dual);
  lua_pushboolean(L, ok);
  return 1;
}

int l_Camera3D_setPosition(lua_State* L) {
  camera.position.x = (float)luaL_checknumber(L, 1);
  camera.position.y = (float)luaL_checknumber(L, 2);
  camera.position.z = (float)luaL_checknumber(L, 3);
  return 0;
}
int l_Camera3D_getPosition(lua_State* L) {
  lua_pushnumber(L, camera.position.x);
  lua_pushnumber(L, camera.position.y);
  lua_pushnumber(L, camera.position.z);
  return 3;
}
int l_Camera3D_setTarget(lua_State* L) {
  camera.target.x = (float)luaL_checknumber(L, 1);
  camera.target.y = (float)luaL_checknumber(L, 2);
  camera.target.z = (float)luaL_checknumber(L, 3);
  return 0;
}
int l_Camera3D_getTarget(lua_State* L) {
  lua_pushnumber(L, camera.target.x);
  lua_pushnumber(L, camera.target.y);
  lua_pushnumber(L, camera.target.z);
  return 3;
}
int l_Camera3D_setFov(lua_State* L) {
  // degrees
  camera.fov = (float)luaL_checknumber(L, 1) * (3.14159265f / 180.0f);
  return 0;
}
int l_Camera3D_setNearFar(lua_State* L) {
  camera.nearZ = (float)luaL_checknumber(L, 1);
  camera.farZ = (float)luaL_checknumber(L, 2);
  return 0;
}
int l_Camera3D_setYawPitch(lua_State* L) {
  camera.yaw = (float)luaL_checknumber(L, 1);
  camera.pitch = (float)luaL_checknumber(L, 2);
  if (camera.pitch > 1.57f) camera.pitch = 1.57f;
  if (camera.pitch < -1.57f) camera.pitch = -1.57f;
  return 0;
}
int l_Camera3D_getYawPitch(lua_State* L) {
  lua_pushnumber(L, camera.yaw);
  lua_pushnumber(L, camera.pitch);
  return 2;
}
int l_Camera3D_rotate(lua_State* L) {
  camera.rotate((float)luaL_checknumber(L, 1), (float)luaL_optnumber(L, 2, 0.0));
  return 0;
}
int l_Camera3D_orbit(lua_State* L) {
  camera.orbitate((float)luaL_checknumber(L, 1));
  return 0;
}
int l_Camera3D_freecam(lua_State* L) {
  camera.freecam();
  return 0;
}
int l_Camera3D_move(lua_State* L) {
  Vec3 d{(float)luaL_checknumber(L, 1), (float)luaL_checknumber(L, 2), (float)luaL_checknumber(L, 3)};
  camera.move(d);
  return 0;
}

// Begin3DFrame: clear draw queue
int l_Begin3DFrame(lua_State* L) {
  if (!gR3DReady) return luaL_error(L, "Init3DRenderer() first");
  r3dQueueClear();
  return 0;
}

// Render3D(): rasterize all models queued via :draw(), present full-screen via DMA blit
// Optional: Render3D(false) skips present (leaves pixels in internal buffer)
int l_Render3D(lua_State* L) {
  if (!gR3DReady) return luaL_error(L, "Init3DRenderer() first");
  bool doPresent = lua_isnoneornil(L, 1) ? true : lua_toboolean(L, 1);

  if (gR3DAsync) {
    r3dDropHeld();
    // Pega um buffer de cor livre (se os 2 estão ocupados, o SPI é o gargalo e é aqui que o Lua espera),
    // rasteriza nele e entrega pra tarefa de blit: o próximo quadro já começa a ser rasterizado durante o envio.
    int idx = 0;
    xQueueReceive(freeQ, &idx, portMAX_DELAY);
    r3dRenderTo(colorBuf[idx]);
    gR3DLast = idx;
    if (doPresent) {
      gR3DPending++;
      xQueueSend(fullQ, &idx, portMAX_DELAY);
    } else {
      gR3DHeld = idx;   // sem envio: fica com o Lua até o Composite3DToFrame (vira camada) ou o próximo Render3D
    }
    r3dQueueClear();
    return 0;
  }

  uint16_t* color = colorBuf[0];
  r3dRenderTo(color);
  gR3DLast = 0;

  if (doPresent) {
    if (gPresentMutex) xSemaphoreTake(gPresentMutex, portMAX_DELAY);
    r3dPresent(color);
    if (gPresentMutex) xSemaphoreGive(gPresentMutex);
  }
  r3dQueueClear();
  return 0;
}

// Composite3DToFrame(): scale internal 3D buffer into the current Lua frame sprite (for HUD overlays)
int l_Composite3DToFrame(lua_State* L) {
  if (!gR3DReady) return luaL_error(L, "Init3DRenderer() first");
  uint16_t* dst = (uint16_t*)frame.getBuffer();
  if (!dst) return 0;
  const int dw = frame.width(), dh = frame.height();

  // Caminho rápido: o 3D vira a camada de fundo do quadro, sem copiar nada (o motor de tela amplia no envio)
  if (gR3DAsync && gR3DHeld >= 0 && HR_W * 2 == dw && HR_H * 2 == dh) {
    if (gLayerSrcIdx >= 0) r3dReleaseBuf(gLayerSrcIdx);    // camada anterior que nunca chegou a ser enviada
    gLayerSrcIdx = gR3DHeld;
    gR3DHeld = -1;
    static uint32_t layerSerial = 0;                       // conteúdo muda a cada chamada => o diff nunca pula o quadro
    if (gCmdN < CMD_MAX) {
      gCmdCur[gCmdN].h = hmix(0x33445202u, ++layerSerial);
      gCmdCur[gCmdN].r.x = 0; gCmdCur[gCmdN].r.y = 0; gCmdCur[gCmdN].r.w = (int16_t)dw; gCmdCur[gCmdN].r.h = (int16_t)dh;
      gCmdN++;
    } else gCmdOverflow = true;                            // (não entra no dirty do HUD: o buffer 2D só guarda o HUD)
    return 0;
  }

  const uint16_t* src = colorBuf[gR3DLast];

  // Assume RENDER_DIV==2: cada pixel interno vira um bloco 2x2. Cobre [0,cw) x [0,ch) por inteiro (opaco).
  const int cw = (HR_W * 2 < dw) ? HR_W * 2 : dw;
  const int ch = (HR_H * 2 < dh) ? HR_H * 2 : dh;
  flushClearUnless(0, 0, cw, ch);          // opaco: se cobre a limpeza pendente, descarta; senão limpa ANTES (não apaga o 3D depois)

  const int cols = cw >> 1;                // pares completos de saída; cw ímpar => última coluna avulsa (abaixo)
  for (int y = 0; y < HR_H && (y * 2) < dh; ++y) {
    const uint16_t* srow = src + (size_t)y * HR_W;
    uint16_t* d0 = dst + (size_t)(y * 2) * dw;
    uint16_t* d1 = d0 + dw;
    const bool two = (y * 2 + 1) < dh;
    // dw par (480) => d0 e d1 alinhados a 4 bytes: 1 escrita de 32 bits por pixel interno (antes: 4 de 16 bits)
    if ((dw & 1) == 0) {
      uint32_t* w0 = (uint32_t*)d0;
      uint32_t* w1 = (uint32_t*)d1;
      for (int x = 0; x < cols; ++x) {
        const uint32_t p = srow[x];
        const uint32_t pp = p | (p << 16);
        w0[x] = pp;
        if (two) w1[x] = pp;
      }
    } else {
      for (int x = 0; x < cols; ++x) {
        const uint16_t p = srow[x];
        d0[x * 2] = p; d0[x * 2 + 1] = p;
        if (two) { d1[x * 2] = p; d1[x * 2 + 1] = p; }
      }
    }
  }

  // O conteúdo muda a cada chamada: hash único => o diff de comandos sempre reenvia o retângulo inteiro
  static uint32_t compSerial = 0;
  cmdRecord(hmix(0x33445201u, ++compSerial), 0, 0, cw, ch);
  return 0;
}


// Print Lua panic message before abort (default panic is abort() with no text)
static int r3d_lua_panic(lua_State* L) {
  const char* msg = lua_tostring(L, -1);
  Serial.printf("LUA PANIC: %s\n", msg ? msg : "(null)");
  Serial.flush();
  return 0; // lua still aborts after this returns in some builds; we also abort explicitly
}

void registerR3D(lua_State* L) {
  Serial.println("[R3D] registerModel3D"); Serial.flush();
  registerModel3D(L);
  Serial.println("[R3D] lua_register globals"); Serial.flush();

  lua_register(L, "Init3DRenderer", l_Init3DRenderer);
  lua_register(L, "Begin3DFrame", l_Begin3DFrame);
  lua_register(L, "Render3D", l_Render3D);
  lua_register(L, "Composite3DToFrame", l_Composite3DToFrame);

  lua_register(L, "Camera3D_SetPosition", l_Camera3D_setPosition);
  lua_register(L, "Camera3D_GetPosition", l_Camera3D_getPosition);
  lua_register(L, "Camera3D_SetTarget", l_Camera3D_setTarget);
  lua_register(L, "Camera3D_GetTarget", l_Camera3D_getTarget);
  lua_register(L, "Camera3D_SetFov", l_Camera3D_setFov);
  lua_register(L, "Camera3D_SetNearFar", l_Camera3D_setNearFar);
  lua_register(L, "Camera3D_SetYawPitch", l_Camera3D_setYawPitch);
  lua_register(L, "Camera3D_GetYawPitch", l_Camera3D_getYawPitch);
  lua_register(L, "Camera3D_Rotate", l_Camera3D_rotate);
  lua_register(L, "Camera3D_Orbit", l_Camera3D_orbit);
  lua_register(L, "Camera3D_Freecam", l_Camera3D_freecam);
  lua_register(L, "Camera3D_Move", l_Camera3D_move);
  Serial.println("[R3D] all globals ok"); Serial.flush();
}



// ===== RESGISTER EXPORTED FUNCTIONS =====
void registerApis(lua_State* L) {
  //lua_register(L, "teste", l_teste);
  lua_register(L, "endProgram", l_endProgram);
  lua_register(L, LUA_CLEAR_SCREEN_DEFINITION, l_clearScreen);
  lua_register(L, LUA_UPDATE_SCREEN_DEFINITION, l_updateScreen);
  lua_register(L, LUA_GET_FREE_MEMORY_DEFINITION, l_getFreeMemory);
  lua_register(L, LUA_GET_MEMORY_SIZE_DEFINITION, l_getMemorySize);
  lua_register(L, LUA_GET_FREE_HEAP_MEMORY_DEFINITION, l_getFreeHeapMemory);
  lua_register(L, LUA_GET_HEAP_MEMORY_SIZE_DEFINITION, l_getHeapMemorySize);
  lua_register(L, LUA_GET_CPU_TEMPERATURE_DEFINITION, l_getCpuTemperature);
  lua_register(L, LUA_GET_LUA_HEAP_USAGE_DEFINITION, l_getLuaHeapUsage);
  lua_register(L, LUA_GET_TIME_DEFINITION, l_getTime);
  lua_register(L, LUA_GET_USER_USERNAME_DEFINITION, l_getUserUsername);
  lua_register(L, LUA_GET_USER_PASSWORD_DEFINITION, l_getUserPassword);
  lua_register(L, LUA_OS_SLEEP_DEFINITION, l_OS_Sleep);
  lua_register(L, LUA_OS_RESTART_DEFINITION, l_OS_Restart);
  lua_register(L, LUA_OS_GET_KERNEL_VERSION_DEFINITION, l_OS_GetKernelVersion);
  lua_register(L, LUA_OS_SET_KEYBOARD_DELAY_DEFINITION, l_OS_SetKeyboardDelay);
  lua_register(L, LUA_OS_GET_PRESSED_KEY_DEFINITION, l_OS_GetPressedKey);
  lua_register(L, LUA_OS_IS_KEY_PRESSED_DEFINITION, l_OS_IsKeyPressed);
  lua_register(L, LUA_OS_SAVE_BIOS_CONFIGURATION_DEFINITION, l_OS_SaveBiosConfig);
  lua_register(L, LUA_OS_SAVE_SYSTEM_CONFIGURATION_DEFINITION, l_OS_SaveSystemConfig);
  lua_register(L, LUA_OS_SAVE_USER_CONFIGURATION_DEFINITION, l_OS_SaveUserConfig);
  lua_register(L, LUA_OS_SET_BIOS_CONFIG_INTEGER_KEY, l_OS_SetBiosConfigIntegerKey);
  lua_register(L, LUA_OS_SET_BIOS_CONFIG_STRING_KEY, l_OS_SetBiosConfigStringKey);
  lua_register(L, LUA_OS_SET_SYSTEM_CONFIG_INTEGER_KEY, l_OS_SetSystemConfigIntegerKey);
  lua_register(L, LUA_OS_SET_SYSTEM_CONFIG_STRING_KEY, l_OS_SetSystemConfigStringKey);
  lua_register(L, LUA_OS_SET_USER_CONFIG_INTEGER_KEY, l_OS_SetUserConfigIntegerKey);
  lua_register(L, LUA_OS_SET_USER_CONFIG_STRING_KEY, l_OS_SetUserConfigStringKey);
  
  #define STACKCHK(tag) do { Serial.printf("[LUA] %s top=%d\n", tag, lua_gettop(L)); Serial.flush(); } while(0)
  
  registerTime(L); STACKCHK("time");
  registerSprite(L); STACKCHK("spr");
  registerFont(L); STACKCHK("font");
  lua_register(L, LUA_OS_HAS_MOUSE_CONNECTED_DEFINITION, l_OS_HasMouseConnected);
  lua_register(L, LUA_OS_GET_MOUSE_POSITION_DEFINITION, l_OS_GetMousePosition);
  lua_register(L, LUA_OS_IS_MOUSE1_PRESSED_DEFINITION, l_OS_IsMouse1Pressed);
  lua_register(L, LUA_OS_IS_MOUSE2_PRESSED_DEFINITION, l_OS_IsMouse2Pressed);
  lua_register(L, LUA_OS_GET_MOUSE_SCROLL_DELTA_DEFINITION, l_OS_GetMouseScrollDelta);
  lua_register(L, LUA_OS_UPDATE_MOUSE_TASK_DEFINITION, l_OS_UpdateMouseTask);
  lua_register(L, LUA_OS_IS_WIFI_ENABLED_DEFINITION, l_OS_IsWifiEnabled);
  lua_register(L, LUA_OS_IS_BLUETOOTH_ENABLED_DEFINITION, l_OS_IsBluetoothEnabled);
  lua_register(L, LUA_OS_ENABLE_WIFI_DEFINITION, l_OS_EnableWifi);
  lua_register(L, LUA_OS_DISABLE_WIFI_DEFINITION, l_OS_DisableWifi);
  lua_register(L, LUA_OS_ENABLE_BLUETOOTH_DEFINITION, l_OS_EnableBluetooth);
  lua_register(L, LUA_OS_GET_CONNECTED_BLUETOOTH_NAME_DEFINITION, l_OS_GetConnectedBluetoothName);
  lua_register(L, LUA_OS_DISABLE_BLUETOOTH_DEFINITION, l_OS_DisableBluetooth);
  lua_register(L, LUA_DISPLAY_DRAW_RECT_DEFINITION, l_drawRect);
  lua_register(L, LUA_DISPLAY_DRAW_ELIPSE_DEFINITION, l_drawElipse);
  lua_register(L, LUA_DISPLAY_DRAW_ARC_DEFINITION, l_drawArc);
  lua_register(L, LUA_DISPLAY_DRAW_POLIGON_DEFINITION, l_drawPoligon);
  lua_register(L, LUA_DISPLAY_DRAW_LINE_DEFINITION, l_drawLine);
  lua_register(L, LUA_OS_GET_FOUND_WIFI_NETWORKS_DEFINITION, l_OS_GetFoundWifiNetworks);
  lua_register(L, LUA_OS_FORGET_WIFI_DEFINITION, l_OS_ForgetWifi);
  lua_register(L, LUA_OS_CONNECT_WIFI_DEFINITION, l_OS_ConnectWifi);
  lua_register(L, LUA_OS_GET_WIFI_STATUS_DEFINITION, l_OS_GetWifiStatus);
  lua_register(L, LUA_OS_GET_CONNECTED_WIFI_SSID_DEFINITION, l_OS_GetConnectedWifiSSID);
  lua_register(L, LUA_OS_IS_WIFI_SCANNING_DEFINITION, l_OS_IsWifiScanning);
  Serial.println("[LUA] registerR3D..."); Serial.flush();
  registerR3D(L);
  Serial.println("[LUA] registerR3D ok"); Serial.flush();
}

// ===== Task Lua (Core 1) =====
void taskLuaApp(void* arg) {
  Serial.printf("== Lua Task Started at Core: %d ==\n", xPortGetCoreID());
  Serial.printf("[LUA] stack high water will print after; free heap=%u psram=%u\n",
                (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getFreePsram());
  Serial.flush();

  luaHeapUsed = 0;

  Serial.println("[LUA] lua_newstate..."); Serial.flush();
  lua_State* L = lua_newstate(lua_psram_alloc, NULL, esp_random());
  if (!L) {
    Serial.println("ERR: Couldn't create LUA VM");
    Serial.flush();
    vTaskDelete(NULL);
    return;
  }
  Serial.println("[LUA] newstate ok"); Serial.flush();

  Serial.println("[LUA] openlibs..."); Serial.flush();
  luaL_openlibs(L);
  lua_atpanic(L, r3d_lua_panic);
  Serial.println("[LUA] openlibs ok"); Serial.flush();

  Serial.println("[LUA] registerApis..."); Serial.flush();
  registerApis(L);
  Serial.println("[LUA] registerApis ok"); Serial.flush();

  const char* scriptPath = "/sys/src/sys.lua"; // /sys/src/sys.lua and /home/apps/3d_test/test_3d.lua
  Serial.printf("[LUA] opening %s\n", scriptPath); Serial.flush();
  File f = SD.open(scriptPath);
  if (!f) {
    Serial.printf("ERR: %s not found\n", scriptPath);
    Serial.flush();
    lua_close(L);
    vTaskDelete(NULL);
    return;
  }

  size_t size = f.size();
  Serial.printf("[LUA] script size=%u\n", (unsigned)size); Serial.flush();
  char* buffer = (char*)malloc(size + 1);

  if (!buffer) {
    Serial.println("ERR: No RAM available for script");
    Serial.flush();
    f.close();
    lua_close(L);
    vTaskDelete(NULL);
    return;
  }

  f.readBytes(buffer, size);
  buffer[size] = '\0';
  f.close();
  Serial.println("[LUA] script read ok, running..."); Serial.flush();

  luaTaskHandle = xTaskGetCurrentTaskHandle();
  luaTaskAlive = true;

  // EXECUTA ANTES DE DAR FREE
  if (luaL_dostring(L, buffer) != LUA_OK) {
    Serial.printf("LUA ERROR: %s\n", lua_tostring(L, -1));
    Serial.flush();
  }


  luaTaskAlive = false;
  free(buffer);

  // LOOP
  while (!luaRequestExit) {
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }

  luaRequestExit = false;
  lua_close(L);

  Serial.println("[LUA] VM Shutdown");
  vTaskDelete(NULL);
}

// Boot
Sprite* boot_bg = new Sprite();
Sprite* boot_loadingicon = new Sprite();

TaskHandle_t bootTaskHandle = NULL;
bool bootTaskExit = false;

void taskDrawBootImages(void* arg) {
  while (bootTaskExit != true) {
    boot_loadingicon->rotation += 8;

    boot_bg->draw(0,0);
    boot_loadingicon->draw(204,118);

    frame.pushSprite(0, 0);

    vTaskDelay(pdMS_TO_TICKS(16));
  }

  vTaskDelete(bootTaskHandle);
}

// ===== Kernel (Core 0) =====
void setup() {
  esp_task_wdt_deinit();

  Serial.begin(115200);

  Serial.printf("\n\n====== RazorOS ======\n");

  Serial.printf("Starting Kernel at core %d\n", xPortGetCoreID());

  if (!psramFound()) {
    Serial.println("PSRAM NOT FOUND!");
    return;
  }

  // SCREEN START
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setColorDepth(16);
  tft.setBrightness(255);
  tft.setSwapBytes(true);
  Serial.printf("Screen Started\n");

  // SD BEGIN
  if (!SD.begin(SD_CS)) {
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1.5);
    tft.setCursor(5, 5);
    tft.print("ERR 0x001 - Couldn't initiate SD Card module.");
    while (1);
  }
  Serial.printf("Started SD module\n");

  // LOAD BIOS CONFIG
  File biosConfigFile = SD.open("/boot/boot.json");
  if (!biosConfigFile){
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1.5);
    tft.setCursor(5, 5);
    tft.print("ERR 0x003 - Couldn't open bios settings bios.json.");
  }
  DeserializationError error = deserializeJson(BiosConfiguration, biosConfigFile);

  if (error) {
    Serial.print("Erro parse: ");
    Serial.println(error.c_str());
    biosConfigFile.close();
    return;
  }

  biosConfigFile.close();
  Serial.printf("Loaded /boot/boot.json configs\n");

  // CREATING SCREEN FRAME
  frame.setPsram(true);
  frame.setColorDepth(16);
  if (!frame.createSprite(tft.width(), tft.height())) {
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1.5);
    tft.setCursor(5, 5);
    tft.print("ERR 0x002 - Couldn't create drawing frame.");
    while (1);
  }
  Serial.printf("Screen Frame created\n");

  // Colors Test
  if (BiosConfiguration["TestScreenColorsWhenStartingUp"] == true){
    Serial.printf("Testing screen colours\n");
    tft.fillScreen(tft.color565(255, 0, 255));
    delay(1000);
    tft.fillScreen(tft.color565(0, 255, 0));
    delay(1000);
    tft.fillScreen(tft.color565(0, 0, 255));
    delay(1000);
    tft.fillScreen(TFT_BLACK);
  }

  // DRAW BOOT IMAGES
  boot_bg->load("/boot/img/boot_bg.png", 480, 320);
  boot_loadingicon->load("/boot/img/boot_loading.png", 72, 72);
  xTaskCreatePinnedToCore(
    taskDrawBootImages,
    "DrawBootImages",
    LUA_TASK_STACK,
    NULL,
    2,
    &bootTaskHandle,
    1
  );

  // LOAD SYSTEM CONFIGS
  File generalConfigFile = SD.open("/usr/sys/general.json");
  if (!generalConfigFile) {
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1.5);
    tft.setCursor(5, 5);
    tft.print("ERR 0x003 - Couldn't open system settings general.json.");
    while (1);
  }

  DeserializationError _error = deserializeJson(SystemConfiguration, generalConfigFile);

  if (_error) {
    Serial.print("Erro parse: ");
    Serial.println(_error.c_str());
    generalConfigFile.close();
    return;
  }

  generalConfigFile.close();
  Serial.printf("Loaded /usr/sys/general.json configs\n");

  const char* ssid = SystemConfiguration["Wifi"]["ssid"];
  const char* password = SystemConfiguration["Wifi"]["password"];

  Serial.println("Synchronizing Time");
  gmtOffset_sec = int(SystemConfiguration["Time"]["timezone"]) * 3600;
  syncTime(ssid, password);
  //struct tm timeinfo;
  //getLocalTime(&timeinfo);
  //Serial.println(&timeinfo, "%d/%m/%Y %H:%M:%S");

  //vTaskDelete(bootTaskHandle);
  //bootTaskHandle = NULL;
  bootTaskExit = true;
  delay(2000);
  bootTaskExit = false;

  // Libera os sprites da tela de boot (~310 KB de PSRAM, nunca mais usados) e liga o motor de apresentação
  // (buffers de DMA em RAM interna, 2º buffer de quadro, tarefa de blit).
  boot_bg->freeAll();
  boot_loadingicon->freeAll();
  displayEngineBegin();

  // LOAD USER
  // LOAD BIOS CONFIG
  File userConfigFile = SD.open("/usr/sys/user.json");
  if (!userConfigFile){
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1.5);
    tft.setCursor(5, 5);
    tft.print("ERR 0x003 - Couldn't open user user.json.");
  }
  DeserializationError _errorUser = deserializeJson(UserConfiguration, userConfigFile);

  if (_errorUser) {
    Serial.print("Erro parse: ");
    Serial.println(_errorUser.c_str());
    userConfigFile.close();
    return;
  }

  userConfigFile.close();
  Serial.printf("Loaded /usr/sys/user.json user configs\n");

  // EXAMPLE
  //BiosConfiguration["TestScreenColorsWhenStartingUp"] = true;
  //UserConfiguration["username"] = "rsfiscina";
  //SystemConfiguration["Time"]["timezone"] = -4;
  //saveBiosPreferences();
  //saveUserPreferences();
  //saveSystemPreferences();

  // ONLY IF APP PRESENT:
  keyboardInit();
  Serial.printf("Keyboard Started\n");

  bleMouseBegin();
  Serial.printf("BLE Mouse Started\n");

  wifiManagerBegin();
  Serial.printf("Wifi Manager Started\n");

  kernelServicesBegin();
  Serial.printf("Kernel Services Started\n");

  Serial.printf("Starting LUA VM\n");

  xTaskCreatePinnedToCore(
    taskLuaApp,
    "LUA_APP",
    LUA_TASK_STACK,
    NULL,
    2,
    NULL,
    1
  );
}

void loop() {
  // KERNEL
}