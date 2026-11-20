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

#define KERNEL_VERSION "ALPHA 0.2.6"

#define LUA_HEAP_LIMIT (7 * 1024 * 1024)
#define LUA_TASK_STACK 20000

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

// ===== KERNEL SERVICES (forward declarations) =====
void kernelLuaCheckpoint();                 // ponto onde a task Lua pode ser pausada pelo kernel
void kernelDrawToastOverlay();              // desenha a notificação por cima da tela
void kernelShowToast(const char* msg);
bool kernelAskConnectBluetooth(const char* name, const char* addr);
volatile bool toastActive = false;

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
      cfg.freq_write = 80000000;
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
LGFX_Sprite frame(&tft);

// ===== DIRTY RECT TRACKING =====
// Em vez de limpar/enviar a tela inteira (fillScreen + pushSprite completo) a
// cada ciclo Display_ClearScreen()/Display_UpdateScreen(), rastreamos apenas a
// região que efetivamente foi desenhada (por Sprite:draw() e Font:drawString())
// entre um clear e o update seguinte, e só tocamos essa região na PSRAM e no SPI.
struct DirtyRect {
  int x = 0, y = 0, w = 0, h = 0;
  bool valid = false;
};

DirtyRect frameDirtyAccum;  // o que foi desenhado NESTE frame (entre clear e update)
DirtyRect frameDirtyPrev;   // o que foi desenhado (e portanto ficou visível) no frame anterior
bool frameEverPushed = false;

void markDirty(int x, int y, int w, int h) {
  if (w <= 0 || h <= 0) return;

  if (!frameDirtyAccum.valid) {
    frameDirtyAccum.x = x;
    frameDirtyAccum.y = y;
    frameDirtyAccum.w = w;
    frameDirtyAccum.h = h;
    frameDirtyAccum.valid = true;
    return;
  }

  int x1 = min(frameDirtyAccum.x, x);
  int y1 = min(frameDirtyAccum.y, y);
  int x2 = max(frameDirtyAccum.x + frameDirtyAccum.w, x + w);
  int y2 = max(frameDirtyAccum.y + frameDirtyAccum.h, y + h);
  frameDirtyAccum.x = x1;
  frameDirtyAccum.y = y1;
  frameDirtyAccum.w = x2 - x1;
  frameDirtyAccum.h = y2 - y1;
}

void clampRectToScreen(int &x, int &y, int &w, int &h) {
  if (x < 0) { w += x; x = 0; }
  if (y < 0) { h += y; y = 0; }
  if (x + w > tft.width())  w = tft.width()  - x;
  if (y + h > tft.height()) h = tft.height() - y;
  if (w < 0) w = 0;
  if (h < 0) h = 0;
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

struct Sprite {
  LGFX_Sprite spr;

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

    return true;
  }

  void setRotation(float graus) {
    rotation = graus;
  }

  void draw(int x, int y) {
    if (rotation == 0.0f) {
      spr.pushSprite(&frame, x, y, 0x0000);
      markDirty(x, y, width, height);
      return;
    }

    spr.setPivot(width / 2, height / 2);
    frame.setPivot(x + width / 2, y + height / 2);

    spr.pushRotateZoom(&frame, rotation, 1.0f, 1.0f, 0x0000);

    // Com rotação a sprite pode ocupar mais área que width x height original.
    // Marca com margem de segurança usando a diagonal como lado do quadrado
    // que envolve qualquer ângulo de rotação.
    int diag = (int)ceilf(sqrtf((float)(width * width + height * height)));
    int cx = x + width / 2;
    int cy = y + height / 2;
    markDirty(cx - diag / 2, cy - diag / 2, diag, diag);
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
  // Só precisamos apagar (voltar pra preto) a região que ficou visível no
  // frame anterior -- o resto da tela já está preto. Isso troca um
  // fillScreen() de 307KB por um fillRect() bem menor na maioria dos casos.
  if (frameDirtyPrev.valid) {
    frame.fillRect(frameDirtyPrev.x, frameDirtyPrev.y,
                    frameDirtyPrev.w, frameDirtyPrev.h, TFT_BLACK);
  } else {
    // Primeiro clear (ou sem histórico confiável ainda): limpa tudo por segurança.
    frame.fillScreen(TFT_BLACK);
  }

  frameDirtyAccum = DirtyRect();  // começa a acumular o que for desenhado neste novo frame
  return 0;
}

int l_updateScreen(lua_State* L){
  // A região que precisa ir pro display é a união de:
  //  - o que foi apagado agora (frameDirtyPrev, do clear deste ciclo)
  //  - o que foi desenhado agora (frameDirtyAccum)
  DirtyRect pushRect;

  if (frameDirtyPrev.valid && frameDirtyAccum.valid) {
    int x1 = min(frameDirtyPrev.x, frameDirtyAccum.x);
    int y1 = min(frameDirtyPrev.y, frameDirtyAccum.y);
    int x2 = max(frameDirtyPrev.x + frameDirtyPrev.w, frameDirtyAccum.x + frameDirtyAccum.w);
    int y2 = max(frameDirtyPrev.y + frameDirtyPrev.h, frameDirtyAccum.y + frameDirtyAccum.h);
    pushRect = { x1, y1, x2 - x1, y2 - y1, true };
  } else if (frameDirtyAccum.valid) {
    pushRect = frameDirtyAccum;
  } else if (frameDirtyPrev.valid) {
    pushRect = frameDirtyPrev;
  }

  if (!frameEverPushed) {
    // Primeiro update depois de ligar/trocar de app: manda a tela inteira
    // uma vez pra garantir que o display fique sincronizado com o frame.
    frame.pushSprite(0, 0);
    frameEverPushed = true;
  } else if (pushRect.valid) {
    clampRectToScreen(pushRect.x, pushRect.y, pushRect.w, pushRect.h);
    if (pushRect.w > 0 && pushRect.h > 0) {
      frame.setClipRect(pushRect.x, pushRect.y, pushRect.w, pushRect.h);
      frame.pushSprite(0, 0);
      frame.setClipRect(0, 0, frame.width(), frame.height());  // limpa o clip
    }
  }
  // Se pushRect não é válido e já houve push antes, nada mudou desde o
  // último update -- não há necessidade de gastar SPI reenviando a mesma coisa.

  // Notificação ativa: redesenha por cima, senão um push que cruze a área dela apagaria o aviso
  if (toastActive) kernelDrawToastOverlay();

  frameDirtyPrev = frameDirtyAccum;
  frameDirtyAccum = DirtyRect();

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

static inline void shapeHLine(int x1, int x2, int y, uint16_t color) {
  if (x2 < x1) return;
  frame.drawFastHLine(x1, y, x2 - x1 + 1, color);
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
          frame.drawFastHLine(run, y, x - run, color);
          run = -1;
        }
      }
      if (run >= 0) frame.drawFastHLine(run, y, xb - run + 1, color);
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

  shapeDrawRoundedRect(x, y, w, h, rad, st);
  markDirty(x, y, w, h);
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

  shapeDrawEllipse(cx, cy, rx, ry, st);
  markDirty(cx - rx, cy - ry, 2 * rx, 2 * ry);
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

  markDirty(cx - R, cy - R, 2 * R, 2 * R);
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

  shapeDrawPolygon(xs, ys, n, closed, st, join, roundCaps);

  int minX = xs[0], maxX = xs[0], minY = ys[0], maxY = ys[0];
  for (int i = 1; i < n; i++) {
    minX = shpMin(minX, xs[i]);
    maxX = shpMax(maxX, xs[i]);
    minY = shpMin(minY, ys[i]);
    maxY = shpMax(maxY, ys[i]);
  }
  const int pad = st.hasBorder ? (st.bw * 2 + 2) : 1;   // folga pra espessura e pontas das quinas
  markDirty(minX - pad, minY - pad, (maxX - minX + 1) + 2 * pad, (maxY - minY + 1) + 2 * pad);
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
  markDirty(minX - pad, minY - pad, w + 2 * pad, h + 2 * pad);
  return 0;
}

// Sprites
static Sprite* checkSprite(lua_State* L, int index) {
  return (Sprite*)luaL_checkudata(L, index, LUA_SPRITE_DEFINITION);
}

int l_sprite_free(lua_State* L) {
  Sprite* spr = checkSprite(L, 1);
  spr->spr.deleteSprite();   // libera sprite interno
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
  keyboardScanInto(kbMatrix);
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

// ----- Notificação rápida (toast) -----
static const int TOAST_X = 140, TOAST_Y = 6, TOAST_W = 200, TOAST_H = 26;
static char toastText[40] = "";
static volatile uint32_t toastUntilMs = 0;

void kernelDrawToastOverlay() {
  uint16_t bg = tft.color565(25, 25, 25);
  tft.fillRoundRect(TOAST_X, TOAST_Y, TOAST_W, TOAST_H, 6, bg);
  tft.drawRoundRect(TOAST_X, TOAST_Y, TOAST_W, TOAST_H, 6, TFT_WHITE);
  tft.setFont(&fonts::Font2);
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, bg);
  tft.setTextDatum(textdatum_t::middle_center);
  tft.drawString(toastText, TOAST_X + TOAST_W / 2, TOAST_Y + TOAST_H / 2);
}

void kernelShowToast(const char* msg) {
  kernelPauseLua();
  strlcpy(toastText, msg, sizeof(toastText));
  toastUntilMs = millis() + TOAST_DURATION_MS;
  toastActive = true;
  kernelDrawToastOverlay();
  kernelResumeLua();
}

// Some com a notificação: repõe a área a partir do frame do Lua
static void kernelHideToast() {
  kernelPauseLua();
  toastActive = false;
  frame.setClipRect(TOAST_X, TOAST_Y, TOAST_W, TOAST_H);
  frame.pushSprite(0, 0);
  frame.setClipRect(0, 0, frame.width(), frame.height());
  kernelResumeLua();
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

  kernelPauseLua();
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

  // Restaura a tela inteira a partir do frame do Lua
  frame.setClipRect(0, 0, frame.width(), frame.height());
  frame.pushSprite(0, 0);

  kernelResumeLua();
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

        //Serial.printf("Font loaded: %d glyphs | sizeof(Glyph)=%d\n",
        //              glyphCount, sizeof(Glyph));

        return true;
    }

    Glyph* findGlyph(uint32_t code){
        for(int i = 0; i < glyphCount; i++){
            if(glyphs[i].code == code)
                return &glyphs[i];
        }
        return nullptr;
    }

    void drawString(LGFX_Sprite &spr,
                    const char* text,
                    int x,
                    int y,
                    int pts,
                    int r,
                    int g,
                    int b){

        if(!data) return;

        float scale = (float)pts / (float)fontSize;
        uint16_t color = tft.color565(r,g,b);

        int cursorX = x;

        // bbox real dos pixels desenhados, pra reportar ao dirty-rect tracker
        int bboxMinX = x, bboxMaxX = x;
        int bboxMinY = y, bboxMaxY = y;
        bool drewAnything = false;

        while(*text){

            uint8_t c = *text++;
            Glyph* gptr = findGlyph(c);

            if(!gptr){
                cursorX += pts / 2;
                continue;
            }

            int drawX = cursorX + (int)(gptr->xOff * scale);
            int drawY = y + (int)((ascent + gptr->yOff) * scale);

            int scaledW = (int)(gptr->w * scale);
            int scaledH = (int)(gptr->h * scale);

            for(int sy = 0; sy < scaledH; sy++){
                for(int sx = 0; sx < scaledW; sx++){

                    int srcX = (int)(sx / scale);
                    int srcY = (int)(sy / scale);

                    int atlasIndex =
                        (gptr->atlas_y + srcY) * atlasW +
                        (gptr->atlas_x + srcX);

                    uint8_t alpha = bitmap[atlasIndex];

                    if(alpha > 128){
                        spr.drawPixel(
                            drawX + sx,
                            drawY + sy,
                            color
                        );
                        drewAnything = true;
                        if (drawX + sx < bboxMinX) bboxMinX = drawX + sx;
                        if (drawX + sx > bboxMaxX) bboxMaxX = drawX + sx;
                        if (drawY + sy < bboxMinY) bboxMinY = drawY + sy;
                        if (drawY + sy > bboxMaxY) bboxMaxY = drawY + sy;
                    }
                }
            }

            cursorX += (int)(gptr->advance * scale);
        }

        if (drewAnything) {
            markDirty(bboxMinX, bboxMinY, bboxMaxX - bboxMinX + 1, bboxMaxY - bboxMinY + 1);
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

    if(!font->load(path)){
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

    luaL_newmetatable(L, "Font");

    lua_pushcfunction(L, l_font_gc);
    lua_setfield(L, -2, "__gc");

    lua_newtable(L);

    lua_pushcfunction(L, l_font_draw);
    lua_setfield(L, -2, "drawString");

    lua_pushcfunction(L, l_font_measure);
    lua_setfield(L, -2, "getTextWidth");

    lua_setfield(L, -2, "__index");

    lua_pop(L, 1);

    // Global Font constructor
    lua_newtable(L);

    lua_pushcfunction(L, l_font_constructor);
    lua_setfield(L, -2, "__call");

    lua_setmetatable(L, -2);

    lua_setglobal(L, "Font");
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
  registerTime(L);
  registerSprite(L);
  registerFont(L);
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
}

// ===== Task Lua (Core 1) =====
void taskLuaApp(void* arg) {
  Serial.printf("== Lua Task Started at Core: %d ==\n", xPortGetCoreID());

  luaHeapUsed = 0;

  lua_State* L = lua_newstate(lua_psram_alloc, NULL, esp_random());
  if (!L) {
    Serial.println("ERR: Couldn't create LUA VM");
    vTaskDelete(NULL);
    return;
  }

  luaL_openlibs(L);
  registerApis(L);

  File f = SD.open("/sys/src/sys.lua");
  if (!f) {
    Serial.println("ERR: /sys/src/sys.lua not found");
    lua_close(L);
    vTaskDelete(NULL);
    return;
  }

  size_t size = f.size();
  char* buffer = (char*)malloc(size + 1);

  if (!buffer) {
    Serial.println("ERR: No RAM available");
    f.close();
    lua_close(L);
    vTaskDelete(NULL);
    return;
  }

  f.readBytes(buffer, size);
  buffer[size] = '\0';
  f.close();

  luaTaskHandle = xTaskGetCurrentTaskHandle();
  luaTaskAlive = true;

  // EXECUTA ANTES DE DAR FREE
  if (luaL_dostring(L, buffer) != LUA_OK) {
    Serial.printf("LUA ERROR: %s\n", lua_tostring(L, -1));
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