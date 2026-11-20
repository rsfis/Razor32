#include <Arduino.h>
#include <esp_heap_caps.h>
#include <esp_task_wdt.h>
#include <LovyanGFX.hpp>
#include <SD.h>
#include <ArduinoJson.h>
#include <WiFi.h>
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

// ===== WIFI / BLUETOOTH TOGGLE =====
// Nomes das teclas no kbLayout (tratadas só pelo kernel, o Lua nunca recebe elas)
#define KEY_WIFI_TOGGLE "WifiToggle"
#define KEY_BLUETOOTH_TOGGLE "BluetoothToggle"
#define TOGGLE_KEY_COOLDOWN_MS 2000   // timer de utilização das duas teclas
#define KERNEL_KEY_DEBUG 1            // 1 = imprime no Serial as teclas que o kernel vê (pra diagnóstico)
#define TOAST_DURATION_MS 1500        // tempo da notificação rápida na tela

const char* ntpServer = "pool.ntp.org";
long gmtOffset_sec = -3 * 3600;  // UTC-3
const int daylightOffset_sec = 0;

StaticJsonDocument<1024> BiosConfiguration;
StaticJsonDocument<1024> SystemConfiguration;
StaticJsonDocument<1024> UserConfiguration;

struct Sprite;

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
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(99);

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
    {"$", "/", "\"", "(", ")", "@", ",", ".", "fun", "ENTER"}
  },
  {
    {"ESC", "F1", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9"},
    {"CAPSLOCK", "", "", "", "", "", "WifiToggle", "BluetoothToggle", "", "RETURN"},
    {"HOME", "", "", "[", "]", "", "", "", "fun", "ENTER"}
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

// ----- Wi-Fi / Bluetooth -----
volatile bool wifiEnabled = false;   // o syncTime() já deixa o Wi-Fi desligado depois do boot

static void wifiApply(bool on) {
  if (on) {
    const char* ssid = SystemConfiguration["Wifi"]["ssid"] | "";
    const char* password = SystemConfiguration["Wifi"]["password"] | "";
    WiFi.mode(WIFI_STA);
    if (strlen(ssid) > 0) WiFi.begin(ssid, password);   // não bloqueia
    wifiEnabled = true;
    Serial.println("[WIFI] ON");
  } else {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    wifiEnabled = false;
    Serial.println("[WIFI] OFF");
  }
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

int l_OS_IsBluetoothEnabled(lua_State* L) {
  lua_pushboolean(L, bluetoothEnabled);
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
  tft.drawString("Mouse Bluetooth encontrado", cx, y + 12);
  tft.drawString("Conectar a este dispositivo?", cx, y + 32);
  tft.drawString(name, cx, y + 58);
  tft.drawString(addr, cx, y + 76);
  tft.drawString("[S] Sim        [N] Nao", cx, y + 102);
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
      kernelShowToast(on ? "Bluetooth ativado" : "Bluetooth desativado");
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