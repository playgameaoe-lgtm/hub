/*
  Enji - ESP32-S3 Super Mini
  Firmware V0.7 - TFT_eSPI + Smooth Font Roboto, sprite 128x128, HLP/1 qua BLE, Mochi tu the SD

  So voi V0.6:
    - Doi thu vien man hinh sang TFT_eSPI (khong con Adafruit GFX / U8g2). Cau hinh man hinh nam trong
      build_opt.h cung thu muc sketch, KHONG phai sua file trong thu vien TFT_eSPI.
    - Chu: Roboto nhung trong flash (fonts.h), khu rang cua, tieng Viet co dau cho ten duong va "phut".
      Con lai tieng Anh: TURN RIGHT, 300 m, 4,5km.
    - Mui ten chi duong: bieu tuong 30x30 dang mat na alpha (icons.h), khu rang cua.
    - Bo cuc theo toa do nguoi dung dat (xem phan "BO CUC" ben duoi), nen RGB(28,34,42).
    - Ve vao sprite 16-bit 128x128 roi day len man mot lan (khong nhap nhay).

  Thu vien (Library Manager):  TFT_eSPI (2.5.4x), NimBLE-Arduino 2.x, ArduinoJson 7.x
  Tools: ESP32S3 Dev Module | Flash 4MB | PSRAM QSPI | USB CDC On Boot: Enabled
         Partition Scheme: "Huge APP (3MB No OTA/1MB SPIFFS)"
  Cac file trong thu muc sketch: Enji_v0_7.ino, build_opt.h, fonts.h, icons.h

  Cau truc the SD (chi dung cho Mochi):
    /1/frame_000.bmp ...  BMP 24-bit 128x128, hoac /1/anim.r565 (tao bang tools/mochi_pack.py, uu tien dung)
    Folder 3, 6, 7, 10 la Mochi "buon" (khi chua ket noi dien thoai).

  LUU Y:
    1. SSID/mat khau WiFi chi de lay gio NTP (tuy chon). Bo trong = khong dung WiFi, gio se hien --:--.
    2. UUID ghi/thong bao theo MA NGUON firmware mau (0002 = dien thoai ghi, 0003 = HUD bao ve).
    3. build_opt.h co -DUSE_FSPI_PORT: BAT BUOC voi TFT_eSPI 2.5.43/2.5.44 tren ESP32-S3 + Arduino-ESP32 core 3.x.
       Thieu co nay chip crash ngay luc khoi dong (StoreProhibited, EXCVADDR 0x10) vi TFT_eSPI chon SPI_PORT = FSPI = 0.
       (Ban TFT_eSPI tren GitHub tu 20/2/2026 da sua, khi do co nay van vo hai.)
    4. Neu man hinh lech / nhieu vien: doi ST7735_GREENTAB3 trong build_opt.h sang GREENTAB2 hoac GREENTAB128.
       Neu do <-> xanh bi dao: them -DTFT_RGB_ORDER=TFT_BGR vao build_opt.h.
*/

#include <WiFi.h>
#include <NimBLEDevice.h>
#include <ArduinoJson.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <esp_system.h>

#include <SPI.h>
#include <SD.h>
#include <TFT_eSPI.h>             // cau hinh man hinh: build_opt.h

#include "fonts.h"
#include "icons.h"

// ============================================================
// CAU HINH
// ============================================================

#define DEVICE_NAME     "Enji"
#define FIRMWARE_VER    "0.7.1"

// WiFi chi de dong bo gio NTP. De "" de tat han.
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

const long GMT_OFFSET_SEC      = 7 * 3600;      // Viet Nam UTC+7
const int  DAYLIGHT_OFFSET_SEC = 0;

#define SD_CS     11
#define SD_SPI_HZ 20000000                      // ha xuong 10000000 neu doc the bi loi
#define PIN_BOOT  0                             // nut BOOT tren board

#define SCREEN_W          128
#define SCREEN_H          128
#define SCREEN_ROTATION   1                     // giong V0.6; doi 0..3 neu gan nguoc

#define HLP_MAX_FRAME     512                   // §1: toi da 512 byte moi dong
#define HLP_MAX_PAYLOAD   (HLP_MAX_FRAME - 1)
#define PACKET_TIMEOUT_MS 5000
#define WANT_RATE_HZ      4                     // BLE: 1..4 Hz

#define DEBUG_STATE       1                     // in tom tat moi state ra Serial
#define DEBUG_RAW         0                     // 1 = in nguyen van dong JSON
#define DEMO_ON_BOOT      0                     // 1 = vao demo ngay khi bat nguon (giu nut BOOT 1,5 s de bat/tat luc dang chay)

// Luong man hinh cho / Mochi
#define IDLE_BEFORE_MOCHI_MS          10000
#define MOCHI_UPSET_WHEN_DISCONNECTED 1

// UUID (theo ma nguon firmware cong dong)
#define HLP_SERVICE_UUID  "8a7e0001-4d6e-4c48-9a9d-484c504c0001"
#define HLP_WRITE_UUID    "8a7e0002-4d6e-4c48-9a9d-484c504c0001"   // dien thoai -> HUD
#define HLP_NOTIFY_UUID   "8a7e0003-4d6e-4c48-9a9d-484c504c0001"   // HUD -> dien thoai
#define HLP_CAPS_UUID     "8a7e0004-4d6e-4c48-9a9d-484c504c0001"   // doc duoc

// ============================================================
// BO CUC (px, goc tren-trai man hinh). Vi tri chu = mep trai cua net chu (ink) va dinh chu HOA / chu so.
// ============================================================

#define BG_R 28
#define BG_G 34
#define BG_B 42

#define POS_BLE_X         6      // chu BLE 8 px
#define POS_BLE_Y         4
#define POS_TOPR_X        103    // pin (%) hoac gio, 8 px
#define POS_TOPR_Y        4
#define POS_LABEL_X       9      // chu dan duong tieng Anh, 10 px
#define POS_LABEL_Y       25
#define POS_ICON_X        22     // mui ten 30x30
#define POS_ICON_Y        37
#define POS_DIST_CX       37     // khoang cach (300 m), can giua duoi mui ten; 16 px
#define POS_DIST_Y        71
#define POS_SIGN_X        78     // bien toc do 40x40
#define POS_SIGN_Y        33
#define SIGN_SIZE         40
#define SIGN_RING         4      // vien do day 4 px
#define POS_STREET_CX     64     // ten duong: can giua man hinh
#define POS_STREET_Y      93
#define POS_KM_X          22     // so km con lai, 8 px
#define POS_MIN_X         77     // so phut con lai, 8 px
#define POS_BOTTOM_Y      116    // km va phut chung mot duong co so

#define LINE_Y1           15     // 3 net ngang
#define LINE_Y2           90
#define LINE_Y3           110
#define LINE_X0           7
#define LINE_X1           122
#define VLINE_X           64     // net doc giua hai o km | phut
#define VLINE_Y0          113
#define VLINE_Y1          125
#define LINE_ALPHA        77     // 77/255 ~ 0,3 : net 1 px o do dac 30% = nhin nhu day 0,3 px

#define C565(r, g, b) ((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))
#define COL_BG        C565(BG_R, BG_G, BG_B)
#define COL_WHITE     0xFFFF
#define COL_BLACK     0x0000
#define COL_BLUE      C565(45, 125, 255)
#define COL_RED       C565(235, 50, 50)
#define COL_AMBER     C565(236, 176, 24)
#define COL_GREEN     C565(40, 200, 100)
#define COL_SIGN_RED  C565(226, 20, 36)
#define COL_YEL_LT    C565(255, 242, 170)       // nen bien khi vuot 1-4 km/h (nhap nhay voi trang)
#define COL_YEL_DK    C565(255, 196, 0)         // nen bien khi vuot tu 5 km/h (nhap nhay voi trang)
#define COL_GREY      C565(110, 118, 130)
#define COL_LINE_SRC  C565(235, 238, 245)       // mau goc cua cac net ke (se tron 30% voi nen)

// Kieu dung trong chu ky ham: phai khai bao truoc moi ham, vi Arduino IDE tu sinh prototype o dau file
enum MochiMode {
  MOCHI_NORMAL,
  MOCHI_UPSET
};

// Man hinh dang hien thi
enum Screen { SCR_WAIT, SCR_MOCHI, SCR_HUD };

// Thong so cua mot font .vlw (doc tu chinh mang font) de can chu chinh xac den tung pixel
struct FontRef { const uint8_t* vlw; int count; int ascent; int descent; int spaceW; int capH; };
struct Ink { int left; int width; int top; int bottom; int adv; };   // top/bottom: tu duong co so (+ la len)

// ============================================================
// DOI TUONG
// ============================================================

TFT_eSPI    tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft);            // 128x128x16 bit = 32 KB, ve xong moi day len man

struct AlertEntry {
  int  kind = 0;
  int  distance = -1;
  int  value = 0;
  bool hasValue = false;
  int  severity = 0;        // ket xe 1..5
  int  delayMin = -1;       // ket xe: tre bao nhieu phut
};

struct HudState {
  bool nav = false;
  int  speed = 0, limit = 0;
  bool over = false;
  int  turn = 0, turnNext = 0, distance = -1, exitNo = 0;
  String street, streetNext, eta;
  int  remainMin = 0, remainM = -1;
  int  battery = -1;        // % pin dien thoai neu Waze co gui (truong "bat"), -1 = khong co
  bool avgZone = false;
  int  avgLeft = 0, avgSpeed = 0, avgPct = 0;
  AlertEntry alerts[4];     // alerts[0] la canh bao chinh
  int  alertCount = 0;
  uint32_t ts = 0;
};

HudState hud;

bool sdOK = false;
bool overspeedBlinkOn = false;
bool dirty = true;
bool demoMode = false;
bool timeSynced = false;

uint32_t lastPacketTime = 0;
uint32_t lastRender = 0;
uint32_t lastBlink = 0;

// ============================================================
// BLE
// ============================================================

struct RxChunk { uint16_t len; uint8_t data[HLP_MAX_FRAME]; };

static QueueHandle_t rxQueue = nullptr;
static NimBLEServer* bleServer = nullptr;
static NimBLECharacteristic* chrNotify = nullptr;

static volatile bool bleConnected = false;
static volatile bool bleSubscribed = false;
static volatile bool devPending = false;
static volatile bool pongPending = false;
static volatile bool rxResetPending = false;
static volatile uint16_t activeConn = 0xFFFF;
static volatile bool resetWaitPending = false;   // BLE vua ket noi/ngat -> cho lai 10 s truoc khi chay Mochi
static Screen screen = SCR_WAIT;

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s, NimBLEConnInfo& info) override {
    bleConnected = true;
    // Moi ket noi moi: cho packet Waze moi; trong luc cho van chay Mochi.
    lastPacketTime = 0;
    resetWaitPending = true;
    dirty = true;
    Serial.printf("Enji BLE connected (conn %u)\n", info.getConnHandle());
  }
  void onDisconnect(NimBLEServer* s, NimBLEConnInfo& info, int reason) override {
    Serial.printf("BLE disconnected (reason %d)\n", reason);
    bleConnected = (s->getConnectedCount() > 0);
    if (!bleConnected) {
      bleSubscribed = false;
      activeConn = 0xFFFF;
    }
    rxResetPending = true;
    resetWaitPending = true;
    dirty = true;
    NimBLEDevice::startAdvertising();
  }
} serverCallbacks;

class ChrCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& info) override {
    // Chi chep byte roi thoat ngay, phan xu ly nang de loop() lam
    const NimBLEAttValue& v = c->getValue();
    size_t n = v.size();
    if (n == 0 || n > HLP_MAX_FRAME || !rxQueue) return;
    static RxChunk chunk;
    chunk.len = (uint16_t)n;
    memcpy(chunk.data, v.data(), n);
    xQueueSend(rxQueue, &chunk, 0);
  }
  void onSubscribe(NimBLECharacteristic* c, NimBLEConnInfo& info, uint16_t subValue) override {
    if (c != chrNotify) return;
    bleSubscribed = (subValue & 1) != 0;
    if (bleSubscribed) {
      activeConn = info.getConnHandle();
      devPending = true;                         // §8: gui "dev" ngay khi dien thoai bat notify
      rxResetPending = true;
    }
    Serial.printf("BLE notify %s\n", bleSubscribed ? "ON" : "OFF");
    dirty = true;
  }
} chrCallbacks;

// Gui mot dong JSON, tu them '\n', cat theo MTU
bool sendLine(const char* line) {
  if (!bleSubscribed || activeConn == 0xFFFF || !chrNotify || !bleServer) return false;
  size_t n = strlen(line);
  if (n + 1 > HLP_MAX_FRAME) return false;
  static char frame[HLP_MAX_FRAME + 1];
  memcpy(frame, line, n);
  frame[n++] = '\n';

  uint16_t mtu = bleServer->getPeerMTU(activeConn);
  size_t payload = mtu > 23 ? (size_t)(mtu - 3) : 20;
  for (size_t off = 0; off < n; off += payload) {
    size_t c = (n - off < payload) ? (n - off) : payload;
    bool ok = false;
    for (int tries = 0; tries < 4 && !ok; tries++) {
      ok = chrNotify->notify(reinterpret_cast<const uint8_t*>(frame + off), c, activeConn);
      if (!ok) delay(5);
    }
    if (!ok) return false;
    delay(2);
  }
  return true;
}

void sendDev() {
  char line[480];
  snprintf(line, sizeof(line),
    "{\"v\":1,\"t\":\"dev\",\"name\":\"%s\",\"fw\":\"%s\",\"proto\":[1],"
    "\"want\":{\"rate\":%d,\"fields\":[\"nav\",\"spd\",\"lim\",\"over\",\"trn\",\"trn2\",\"dst\","
    "\"exit\",\"st\",\"st2\",\"eta\",\"rmin\",\"rkm\",\"rm\",\"avg\",\"avgL\",\"avgR\",\"avgP\","
    "\"alr\",\"alrD\",\"alrV\",\"alrS\",\"alrM\",\"alrs\"]},"
    "\"can\":[\"speed\",\"limit\",\"turn\",\"street\",\"eta\",\"avgzone\",\"alerts\"],"
    "\"transport\":\"ble\"}",
    DEVICE_NAME, FIRMWARE_VER, WANT_RATE_HZ);
  bool ok = sendLine(line);
  Serial.printf("dev sent: %s\n", ok ? "OK" : "FAILED");
}

void initBLE() {
  rxQueue = xQueueCreate(6, sizeof(RxChunk));
  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setMTU(247);
  bleServer = NimBLEDevice::createServer();
  bleServer->setCallbacks(&serverCallbacks);

  NimBLEService* svc = bleServer->createService(HLP_SERVICE_UUID);
  NimBLECharacteristic* chrWrite =
      svc->createCharacteristic(HLP_WRITE_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  chrWrite->setCallbacks(&chrCallbacks);
  chrNotify = svc->createCharacteristic(HLP_NOTIFY_UUID, NIMBLE_PROPERTY::NOTIFY);
  chrNotify->setCallbacks(&chrCallbacks);
  NimBLECharacteristic* caps = svc->createCharacteristic(HLP_CAPS_UUID, NIMBLE_PROPERTY::READ);
  caps->setValue("{\"v\":1,\"caps\":{\"transport\":\"ble\",\"maxFrame\":512}}\n");

  // Ten + UUID nam trong goi quang ba (30 byte) nhu firmware mau
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->setName(DEVICE_NAME);
  adv->addServiceUUID(HLP_SERVICE_UUID);
  adv->start();
  Serial.println("BLE advertising as " DEVICE_NAME);
}

// ============================================================
// GIAI MA HLP/1
// ============================================================

// Chap nhan so nguyen lan so thuc (52 hoac 52.4); "x | 0" cua ArduinoJson tra 0 voi so thuc
template <typename T> int jInt(const T& v, int def)    { return v.isNull() ? def : v.template as<int>(); }
template <typename T> bool jBool(const T& v, bool def) { return v.isNull() ? def : v.template as<bool>(); }

void decodeState(JsonDocument& doc) {
  // §3: moi goi "s" la snapshot day du; Android bo han field o gia tri mac dinh,
  // nen phai dung lai tu dau chu khong va len state cu.
  HudState s;
  s.nav       = jBool(doc["nav"], false);
  s.speed     = jInt(doc["spd"], 0);
  s.limit     = jInt(doc["lim"], 0);
  s.over      = jBool(doc["over"], false);
  s.turn      = jInt(doc["trn"], 0);
  s.turnNext  = jInt(doc["trn2"], 0);
  s.distance  = jInt(doc["dst"], -1);
  s.exitNo    = jInt(doc["exit"], 0);
  int bt = jInt(doc["bat"], -1);                           // % pin dien thoai (neu Waze Mod co gui)
  if (bt >= 0 && bt <= 100) s.battery = bt;

  const char* st  = doc["st"]  | "";
  const char* st2 = doc["st2"] | "";
  const char* eta = doc["eta"] | "";
  s.street     = String(st);
  s.streetNext = String(st2);
  s.eta        = String(eta);

  s.remainMin = jInt(doc["rmin"], 0);
  if (!doc["rm"].isNull())       s.remainM = jInt(doc["rm"], -1);          // "rm" co tham quyen
  else if (!doc["rkm"].isNull()) s.remainM = (int)(doc["rkm"].as<double>() * 1000.0 + 0.5);

  s.avgZone  = jBool(doc["avg"], false);
  s.avgLeft  = jInt(doc["avgL"], 0);
  s.avgSpeed = jInt(doc["avgR"], 0);
  s.avgPct   = jInt(doc["avgP"], 0);

  // Danh sach canh bao (toi da 4); khoa loai la "k", KHONG phai "t"
  JsonArray alrs = doc["alrs"].as<JsonArray>();
  for (JsonObject e : alrs) {
    if (s.alertCount >= 4) break;
    if (e.isNull()) continue;
    AlertEntry& a = s.alerts[s.alertCount++];
    a.kind     = jInt(e["k"], 0);
    a.distance = jInt(e["d"], -1);
    a.hasValue = !e["v"].isNull();
    a.value    = a.hasValue ? jInt(e["v"], 0) : 0;
    if (a.kind == 6) {
      a.severity = jInt(e["s"], 0);
      a.delayMin = jInt(e["m"], -1);
    }
  }
  // Canh bao don (cu): alr / alrD / alrV, mirror cua alrs[0]
  if (s.alertCount == 0) {
    int k = jInt(doc["alr"], 0);
    if (k > 0) {
      AlertEntry& a = s.alerts[s.alertCount++];
      a.kind     = k;
      a.distance = jInt(doc["alrD"], -1);
      a.hasValue = !doc["alrV"].isNull();
      a.value    = a.hasValue ? jInt(doc["alrV"], 0) : 0;
      if (k == 6) { a.severity = jInt(doc["alrS"], 0); a.delayMin = jInt(doc["alrM"], -1); }
    }
  }

  double ts = doc["ts"] | 0.0;
  s.ts = ts > 0 ? (uint32_t)ts : 0;

  hud = s;
  lastPacketTime = millis();
  dirty = true;

#if DEBUG_STATE
  static uint32_t prevPacketMs = 0;
  Serial.printf("S +%lums ", (unsigned long)(prevPacketMs ? lastPacketTime - prevPacketMs : 0));
  prevPacketMs = lastPacketTime;
  Serial.printf("nav=%d spd=%d lim=%d trn=%d exit=%d dst=%d alr=%d(%d m) avg=%d rm=%d st2=%s\n",
                hud.nav, hud.speed, hud.limit, hud.turn, hud.exitNo, hud.distance,
                hud.alertCount ? hud.alerts[0].kind : 0,
                hud.alertCount ? hud.alerts[0].distance : -1,
                hud.avgZone, hud.remainM, hud.streetNext.c_str());
#endif
}

void handleLine(const char* line, size_t len) {
#if DEBUG_RAW
  Serial.print("HLP RX: ");
  Serial.println(line);
#endif
  JsonDocument doc;
  if (deserializeJson(doc, line, len)) return;
  if (!doc.is<JsonObject>()) return;
  if (!doc["v"].is<double>() || doc["v"].as<int>() != 1) return;   // §2: v khac 1 bi bo qua
  const char* t = doc["t"] | (const char*)nullptr;
  if (!t) return;

  if (strcmp(t, "ping") == 0) pongPending = true;                   // tra pong truoc, ve sau
  else if (strcmp(t, "s") == 0) decodeState(doc);
  // loai khac (cau hinh thiet bi...) khong ho tro: bo qua, khong coi la loi
}

// Ghep cac manh BLE thanh dong, ket thuc bang '\n'
static uint8_t rxBuf[HLP_MAX_FRAME];
static size_t rxLen = 0;
static bool rxOverflow = false;

void feedBytes(const uint8_t* d, size_t n) {
  for (size_t i = 0; i < n; i++) {
    uint8_t c = d[i];
    if (c == '\n') {
      if (!rxOverflow) {
        size_t L = rxLen;
        if (L && rxBuf[L - 1] == '\r') L--;                          // §1: xoa '\r' cuoi dong
        rxBuf[L] = 0;
        handleLine((const char*)rxBuf, L);
      }
      rxLen = 0;
      rxOverflow = false;
    } else if (!rxOverflow) {
      if (rxLen >= HLP_MAX_PAYLOAD) rxOverflow = true;               // dong tran: bo, dong bo lai o '\n' ke tiep
      else rxBuf[rxLen++] = c;
    }
  }
}

void pollBLE() {
  if (rxResetPending) { rxResetPending = false; rxLen = 0; rxOverflow = false; }
  if (pongPending)    { pongPending = false; sendLine("{\"v\":1,\"t\":\"pong\"}"); }
  if (devPending)     { devPending = false; sendDev(); }
  if (!rxQueue) return;
  static RxChunk chunk;
  for (int i = 0; i < 8 && xQueueReceive(rxQueue, &chunk, 0) == pdTRUE; i++) {
    feedBytes(chunk.data, chunk.len);
  }
}

// ============================================================
// DEMO (khong can dien thoai) - dong mau lay/ chinh tu bo test cua firmware cong dong
// ============================================================

static const char* const DEMO_LINES[] = {
  "{\"v\":1,\"t\":\"s\",\"nav\":0,\"spd\":38,\"lim\":50,\"st\":\"Đường Láng\"}",
  "{\"v\":1,\"t\":\"s\",\"nav\":1,\"spd\":32,\"lim\":50,\"trn\":2,\"dst\":120,\"st\":\"Nguyễn Trãi\",\"st2\":\"Khuất Duy Tiến\",\"eta\":\"12:48\",\"rmin\":14,\"rm\":5200}",
  "{\"v\":1,\"t\":\"s\",\"nav\":1,\"spd\":41,\"lim\":50,\"trn\":10,\"exit\":3,\"dst\":250,\"st\":\"Đường Láng\",\"st2\":\"Phạm Văn Đồng\",\"eta\":\"12:50\",\"rmin\":12,\"rm\":4800}",
  "{\"v\":1,\"t\":\"s\",\"nav\":1,\"spd\":55,\"lim\":60,\"trn\":5,\"dst\":800,\"st\":\"Nguyễn Trãi\",\"st2\":\"Lê Văn Lương\",\"eta\":\"12:55\",\"rmin\":9,\"rm\":3900,\"alr\":2,\"alrD\":300,\"alrs\":[{\"k\":2,\"d\":300},{\"k\":8,\"d\":800,\"v\":40}]}",
  "{\"v\":1,\"t\":\"s\",\"nav\":1,\"spd\":72,\"lim\":60,\"over\":1,\"trn\":3,\"dst\":1500,\"st\":\"Quốc lộ 5\",\"st2\":\"Ngõ 123 Trần Duy Hưng\",\"eta\":\"13:05\",\"rmin\":25,\"rm\":18200}",
  "{\"v\":1,\"t\":\"s\",\"nav\":1,\"spd\":58,\"lim\":60,\"trn\":1,\"dst\":900,\"st\":\"Nguyễn Trãi\",\"st2\":\"Nguyễn Trãi\",\"eta\":\"18:43\",\"rmin\":10,\"rm\":7200,\"avg\":1,\"avgL\":500,\"avgR\":60,\"avgP\":40}",
  "{\"v\":1,\"t\":\"s\",\"nav\":1,\"spd\":64,\"lim\":60,\"st\":\"ĐT.281\",\"eta\":\"18:43\",\"rmin\":10,\"rm\":7200,\"alr\":1,\"alrD\":180,\"st2\":\"Khuất Duy Tiến\",\"trn\":4,\"dst\":180}",
  "{\"v\":1,\"t\":\"s\",\"nav\":1,\"spd\":22,\"lim\":40,\"trn\":8,\"dst\":47,\"st\":\"Nguyễn Trãi\",\"st2\":\"Khuất Duy Tiến\",\"eta\":\"20:01\",\"rmin\":23,\"rm\":6742}",
  "{\"v\":1,\"t\":\"s\",\"nav\":1,\"spd\":47,\"lim\":120,\"trn\":14,\"dst\":2300,\"st\":\"CT. Cầu Giẽ - Ninh Bình\",\"eta\":\"05:58\",\"rmin\":72,\"rm\":50877,\"alrs\":[{\"k\":6,\"d\":600,\"s\":3,\"m\":8}]}",
  "{\"v\":1,\"t\":\"s\",\"nav\":1,\"spd\":12,\"lim\":30,\"trn\":17,\"dst\":40,\"st\":\"Phố Huế\",\"eta\":\"13:20\",\"rmin\":1,\"rm\":40}",
};
static const int DEMO_COUNT = sizeof(DEMO_LINES) / sizeof(DEMO_LINES[0]);

void demoTick() {
  static uint32_t last = 0;
  static int idx = 0;
  uint32_t now = millis();
  if (last == 0 || now - last >= 4000) {
    last = now;
    const char* l = DEMO_LINES[idx];
    idx = (idx + 1) % DEMO_COUNT;
    handleLine(l, strlen(l));
  }
}

void pollBootButton() {
  static uint32_t pressedAt = 0;
  static bool fired = false;
  if (digitalRead(PIN_BOOT) == LOW) {
    if (pressedAt == 0) pressedAt = millis();
    if (!fired && millis() - pressedAt >= 1500) {
      fired = true;
      demoMode = !demoMode;
      Serial.printf("Demo mode %s\n", demoMode ? "ON" : "OFF");
      if (!demoMode) lastPacketTime = 0;
      dirty = true;
    }
  } else {
    pressedAt = 0;
    fired = false;
  }
}

// ============================================================
// THOI GIAN
// ============================================================

bool getNow(struct tm& t) {
  time_t now = time(nullptr);
  if (now < 1609459200) return false;
  localtime_r(&now, &t);
  return true;
}

String getClockText() {
  struct tm t;
  if (!getNow(t)) return "--:--";
  char buf[8];
  strftime(buf, sizeof(buf), "%H:%M", &t);
  return String(buf);
}

String getDateText(bool shortForm) {
  struct tm t;
  if (!getNow(t)) return shortForm ? "--/--" : "--/--/----";
  char buf[40];
  if (shortForm) {
    snprintf(buf, sizeof(buf), "%02d/%02d", t.tm_mday, t.tm_mon + 1);
  } else {
    static const char* wd[] = { "Chu nhat", "Thu hai", "Thu ba", "Thu tu", "Thu nam", "Thu sau", "Thu bay" };
    snprintf(buf, sizeof(buf), "%s, %02d/%02d/%04d", wd[t.tm_wday], t.tm_mday, t.tm_mon + 1, t.tm_year + 1900);
  }
  return String(buf);
}

// ============================================================
// TRANG THAI
// ============================================================

bool dataFresh() { return lastPacketTime > 0 && (millis() - lastPacketTime) <= PACKET_TIMEOUT_MS; }
bool navActive() { return dataFresh() && hud.nav; }
bool cruiseActive() { return dataFresh() && !hud.nav; }

bool isOverspeed() {
  return dataFresh() && (hud.over || (hud.limit > 0 && hud.speed > hud.limit));
}

// ============================================================
// DO HOA: TFT_eSPI + Smooth Font Roboto + bieu tuong alpha, ve vao sprite roi day len man
// ============================================================

static FontRef FR_LABEL, FR_SMALL, FR_STREET, FR_SPEED, FR_SPEED3, FR_DIST, FR_CLOCK;

// ---- doc thong so tu chinh mang .vlw (dinh dang big-endian, xem tools/make_vlw.py) ----
static uint32_t be32(const uint8_t* p) {
  return ((uint32_t)pgm_read_byte(p) << 24) | ((uint32_t)pgm_read_byte(p + 1) << 16) |
         ((uint32_t)pgm_read_byte(p + 2) << 8) | (uint32_t)pgm_read_byte(p + 3);
}

static bool fontGlyph(const FontRef& f, uint16_t cp, int* adv, int* w, int* h, int* dy, int* dx) {
  const uint8_t* g = f.vlw + 24;
  for (int i = 0; i < f.count; i++, g += 28) {
    if ((uint16_t)be32(g) == cp) {
      if (h)   *h   = (int)(int32_t)be32(g + 4);
      if (w)   *w   = (int)(int32_t)be32(g + 8);
      if (adv) *adv = (int)(int32_t)be32(g + 12);
      if (dy)  *dy  = (int)(int32_t)be32(g + 16);
      if (dx)  *dx  = (int)(int32_t)be32(g + 20);
      return true;
    }
  }
  return false;
}

static FontRef fontMake(const uint8_t* vlw) {
  FontRef f;
  f.vlw = vlw;
  f.count = (int)be32(vlw);
  f.ascent = (int)be32(vlw + 16);
  f.descent = (int)be32(vlw + 20);
  f.spaceW = (f.ascent + f.descent) * 2 / 7;            // giong gFont.spaceWidth cua TFT_eSPI
  int dy = f.ascent;
  if (!fontGlyph(f, 'H', nullptr, nullptr, nullptr, &dy, nullptr)) fontGlyph(f, '0', nullptr, nullptr, nullptr, &dy, nullptr);
  f.capH = dy;                                          // chieu cao chu hoa / chu so (px tren duong co so)
  return f;
}

// Doc 1 ky tu UTF-8 (toi da 3 byte, giong TFT_eSPI)
static uint16_t nextCp(const char*& s) {
  uint8_t c = (uint8_t)*s++;
  if (c < 0x80) return c;
  if ((c & 0xE0) == 0xC0 && *s) { uint8_t c2 = (uint8_t)*s++; return (uint16_t)(((c & 0x1F) << 6) | (c2 & 0x3F)); }
  if ((c & 0xF0) == 0xE0 && s[0] && s[1]) {
    uint8_t c2 = (uint8_t)*s++, c3 = (uint8_t)*s++;
    return (uint16_t)(((c & 0x0F) << 12) | ((c2 & 0x3F) << 6) | (c3 & 0x3F));
  }
  if ((c & 0xF8) == 0xF0) { for (int i = 0; i < 3 && *s; i++) s++; }
  return 0xFFFD;
}

// Kich thuoc "muc" (ink) that cua chuoi khi ve bang font f (co tinh khoang trang giong TFT_eSPI)
static Ink fontInk(const FontRef& f, const char* s) {
  int x = 0, minL = 100000, maxR = -100000, maxTop = -100000, minBot = 100000;
  while (*s) {
    uint16_t cp = nextCp(s);
    if (cp == 0) continue;
    if (cp == ' ') { x += f.spaceW; continue; }
    int adv, w, h, dy, dx;
    if (!fontGlyph(f, cp, &adv, &w, &h, &dy, &dx)) continue;
    if (w > 0 && h > 0) {
      int l = x + dx;
      if (l < minL) minL = l;
      if (l + w > maxR) maxR = l + w;
      if (dy > maxTop) maxTop = dy;
      if (dy - h < minBot) minBot = dy - h;
    }
    x += adv;
  }
  Ink k;
  k.adv = x;
  if (maxR < minL) { k.left = 0; k.width = 0; k.top = 0; k.bottom = 0; }
  else { k.left = minL; k.width = maxR - minL; k.top = maxTop; k.bottom = minBot; }
  return k;
}

static void appendUtf8(String& out, uint16_t cp) {
  if (cp < 0x80) out += (char)cp;
  else if (cp < 0x800) { out += (char)(0xC0 | (cp >> 6)); out += (char)(0x80 | (cp & 0x3F)); }
  else { out += (char)(0xE0 | (cp >> 12)); out += (char)(0x80 | ((cp >> 6) & 0x3F)); out += (char)(0x80 | (cp & 0x3F)); }
}

// Bo ky tu font khong co (tranh o vuong "tofu"); doi dau gach/nhay kieu Unicode ve ASCII
static String fontClean(const FontRef& f, const String& in) {
  String out;
  out.reserve(in.length());
  const char* s = in.c_str();
  while (*s) {
    uint16_t cp = nextCp(s);
    if (cp == 0 || cp == 0xFFFD) continue;
    if (cp == 0x2013 || cp == 0x2014) cp = '-';
    else if (cp == 0x2018 || cp == 0x2019) cp = '\'';
    else if (cp == 0x201C || cp == 0x201D) cp = '"';
    else if (cp == 0x00A0) cp = ' ';
    if (cp == ' ' || fontGlyph(f, cp, nullptr, nullptr, nullptr, nullptr, nullptr)) appendUtf8(out, cp);
  }
  return out;
}

// Gioi han doc cua cac chu so 0-9 (de can giua so toc do luon cung mot do cao, du la 5, 99 hay 100)
static void digitBounds(const FontRef& f, int* top, int* bottom) {
  int t = -100000, b = 100000;
  for (char c = '0'; c <= '9'; c++) {
    int h, dy;
    if (fontGlyph(f, c, nullptr, nullptr, &h, &dy, nullptr) && h > 0) {
      if (dy > t) t = dy;
      if (dy - h < b) b = dy - h;
    }
  }
  if (t < b) { t = f.capH; b = 0; }
  *top = t; *bottom = b;
}

// ---- ve chu ----
static const uint8_t* g_loaded = nullptr;
static void useFont(const FontRef& f) {
  if (g_loaded == f.vlw) return;
  if (g_loaded) spr.unloadFont();
  spr.loadFont(f.vlw);
  g_loaded = f.vlw;
}
static void endFonts() {
  if (g_loaded) { spr.unloadFont(); g_loaded = nullptr; }
}

// Mep trai cua net chu o x, dinh chu hoa/chu so o capTop (px)
static void drawInkLeft(const FontRef& f, const String& str, int x, int capTop, uint16_t color) {
  if (!str.length()) return;
  Ink k = fontInk(f, str.c_str());
  useFont(f);
  spr.setTextColor(color);
  spr.setTextDatum(TL_DATUM);
  spr.drawString(str, x - k.left, capTop + f.capH - f.ascent);
}

// Can giua theo chieu ngang quanh cx (toa do mep pixel, vd 64 = giua man hinh 128 px)
static void drawInkCenterX(const FontRef& f, const String& str, int cx, int capTop, uint16_t color) {
  if (!str.length()) return;
  Ink k = fontInk(f, str.c_str());
  drawInkLeft(f, str, cx - (k.width + 1) / 2, capTop, color);
}

// Can giua ca hai chieu quanh (cx, cy) (toa do mep pixel): dung de dat so vao giua bien toc do
static void drawInkCenterXY(const FontRef& f, const String& str, int cx, int cy, uint16_t color) {
  if (!str.length()) return;
  Ink k = fontInk(f, str.c_str());
  int dTop, dBot;
  digitBounds(f, &dTop, &dBot);                              // theo ca bo chu so de vi tri doc khong nhay giua cac so
  int baseline = (int)floorf(cy + (dTop + dBot) * 0.5f + 0.5f);
  useFont(f);
  spr.setTextColor(color);
  spr.setTextDatum(TL_DATUM);
  spr.drawString(str, cx - (k.width + 1) / 2 - k.left, baseline - f.ascent);
}

// Ten duong: can giua; qua dai thi chay chu trong vung cat (viewport)
static void drawStreet(const String& raw, uint16_t color, uint32_t tick) {
  String s = fontClean(FR_STREET, raw);
  if (!s.length()) return;
  Ink k = fontInk(FR_STREET, s.c_str());
  const int areaX = 5, areaW = SCREEN_W - 10;
  if (k.width <= areaW) { drawInkCenterX(FR_STREET, s, POS_STREET_CX, POS_STREET_Y, color); return; }
  int extra = k.width - areaW;
  uint32_t period = (uint32_t)extra * 33 + 2400;             // ~30 px/s, dung ~1,2 s o hai dau
  uint32_t t = tick % period;
  int off = (t >= 1200) ? (int)((t - 1200) / 33) : 0;
  if (off > extra) off = extra;
  spr.setViewport(areaX, POS_STREET_Y - 6, areaW, 22, false);
  drawInkLeft(FR_STREET, s, areaX - off, POS_STREET_Y, color);
  spr.resetViewport();
}

// ---- bieu tuong alpha 8-bit: to mau luc chay, tron muot voi nen ----
static void drawIcon(const uint8_t* mask, int x0, int y0, uint16_t color) {
  if (!mask) return;
  for (int j = 0; j < ICON_SIZE; j++) {
    for (int i = 0; i < ICON_SIZE; i++) {
      uint8_t a = pgm_read_byte(mask + j * ICON_SIZE + i);
      if (!a) continue;
      int px = x0 + i, py = y0 + j;
      spr.drawPixel(px, py, a == 255 ? color : spr.alphaBlend(a, color, spr.readPixel(px, py)));
    }
  }
}

// ---- dinh dang ----
static String fmtDistBig(int m) {                            // "300 m", "1,2 km", "12 km"
  if (m < 0) return String("--");
  if (m >= 10000) return String(m / 1000) + " km";
  if (m >= 1000) return String(m / 1000) + "," + String((m % 1000) / 100) + " km";
  return String(m) + " m";
}
static String fmtKm(int m) {                                 // "4,5km", "12km", "850m"
  if (m < 0) return String("--");
  if (m >= 10000) return String(m / 1000) + "km";
  if (m >= 1000) return String(m / 1000) + "," + String((m % 1000) / 100) + "km";
  return String(m) + "m";
}
static String fmtMin(int mins) {                             // "15 phút"
  if (mins <= 0) return String("--");
  return String(mins) + " phút";
}
static String fmtDate(bool shortForm) {
  struct tm t;
  if (!getNow(t)) return String(shortForm ? "--/--" : "--/--/----");
  char b[16];
  if (shortForm) snprintf(b, sizeof b, "%02d/%02d", t.tm_mday, t.tm_mon + 1);
  else snprintf(b, sizeof b, "%02d/%02d/%04d", t.tm_mday, t.tm_mon + 1, t.tm_year + 1900);
  return String(b);
}

static const char* turnLabel(int t) {                        // chu dan duong tieng Anh
  switch (t) {
    case 1:  return "GO STRAIGHT";
    case 2:  return "TURN LEFT";
    case 3:  return "TURN RIGHT";
    case 4:  return "SLIGHT LEFT";
    case 5:  return "SLIGHT RIGHT";
    case 6:  return "SHARP LEFT";
    case 7:  return "SHARP RIGHT";
    case 8: case 9: return "U-TURN";
    case 10: case 11: case 12: case 19: return "ROUNDABOUT";
    case 13: return "KEEP LEFT";
    case 14: return "KEEP RIGHT";
    case 15: return "EXIT LEFT";
    case 16: return "EXIT RIGHT";
    case 17: return "ARRIVE";
    case 18: return "FERRY";
    default: return "";
  }
}

// Ten canh bao (tieng Anh) theo bang ma HLP; ma la -> "ALERT"
static const char* alertName(int k) {
  switch (k) {
    case 1:  return "POLICE";
    case 2:  return "SPEED CAMERA";
    case 3:  return "RED LIGHT CAMERA";
    case 4:  return "HAZARD";
    case 5:  return "ACCIDENT";
    case 6:  return "TRAFFIC JAM";
    case 7:  return "ROAD CLOSED";
    case 8:  return "SLOW DOWN";
    case 9:  return "NO OVERTAKING";
    case 10: return "END NO OVERTAKING";
    case 11: return "RAILWAY";
    case 12: return "TOLL";
    case 13: return "STOPPED VEHICLE";
    case 14: return "ROADWORKS";
    case 15: return "POTHOLE";
    case 16: return "BAD WEATHER";
    case 17: return "LANE BLOCKED";
    case 18: return "DANGEROUS ROAD";
    case 40: case 41: case 42: case 43: case 44: case 45: case 46: return "CAMERA";
    case 56: return "SPEED BUMP";
    case 57: return "SCHOOL ZONE";
    default: return "ALERT";
  }
}
static bool alertIsRed(int k) {
  return k == 4 || k == 5 || k == 7 || k == 9 || k == 11 || k == 13 || k == 14 || k == 17 || k == 18;
}

// ---- cac phan tu chung ----
static void drawDividers(bool withVertical) {
  uint16_t c = spr.alphaBlend(LINE_ALPHA, COL_LINE_SRC, COL_BG);
  spr.drawFastHLine(LINE_X0, LINE_Y1, LINE_X1 - LINE_X0 + 1, c);
  spr.drawFastHLine(LINE_X0, LINE_Y2, LINE_X1 - LINE_X0 + 1, c);
  spr.drawFastHLine(LINE_X0, LINE_Y3, LINE_X1 - LINE_X0 + 1, c);
  if (withVertical) spr.drawFastVLine(VLINE_X, VLINE_Y0, VLINE_Y1 - VLINE_Y0 + 1, c);
}

static String topRightText() {
  if (hud.battery >= 0 && dataFresh()) return String(hud.battery) + "%";
  return getClockText();
}

static void drawTopBar() {                                   // BLE (xanh = da ket noi, do = chua) + pin hoac gio
  drawInkLeft(FR_SMALL, "BLE", POS_BLE_X, POS_BLE_Y, bleConnected ? COL_BLUE : COL_RED);
  drawInkLeft(FR_SMALL, topRightText(), POS_TOPR_X, POS_TOPR_Y, COL_WHITE);
}

// Bien toc do: nen trang, vien do day 4 px, 40x40. Vuot 1-4 km/h: nen nhap nhay vang nhat/trang; tu 5 km/h: vang dam/trang.
static uint16_t signFill() {
  int d = hud.speed - hud.limit;
  if (d >= 5) return overspeedBlinkOn ? COL_YEL_DK : COL_WHITE;
  if (d >= 1) return overspeedBlinkOn ? COL_YEL_LT : COL_WHITE;
  return COL_WHITE;
}

static void drawSignShape() {
  const float cx = POS_SIGN_X + SIGN_SIZE / 2.0f - 0.5f, cy = POS_SIGN_Y + SIGN_SIZE / 2.0f - 0.5f;   // tam theo toa do tam pixel
  const float R = SIGN_SIZE / 2.0f;
  if (hud.limit <= 0) {                                      // chua co gioi han: vong xam
    spr.drawSpot(cx, cy, R, COL_GREY, COL_BG);
    spr.drawSpot(cx, cy, R - SIGN_RING, COL_BG, COL_GREY);
    return;
  }
  spr.drawSpot(cx, cy, R, COL_SIGN_RED, COL_BG);
  spr.drawSpot(cx, cy, R - SIGN_RING, signFill(), COL_SIGN_RED);
}

static void drawSignNumber() {                               // so luon can GIUA bien (tam mep pixel = POS + SIZE/2)
  int cx = POS_SIGN_X + SIGN_SIZE / 2, cy = POS_SIGN_Y + SIGN_SIZE / 2;
  if (hud.limit <= 0) drawInkCenterXY(FR_SPEED, "--", cx, cy, COL_GREY);
  else if (hud.limit >= 100) drawInkCenterXY(FR_SPEED3, String(hud.limit), cx, cy, COL_BLACK);   // 3 chu so: co nho hon de khong cham vien do
  else drawInkCenterXY(FR_SPEED, String(hud.limit), cx, cy, COL_BLACK);
}

// ---- man hinh chi duong ----
static void drawNavScreen(uint32_t tick) {
  bool alert = hud.alertCount > 0;
  int t = hud.turn;
  const uint8_t* icon = (t >= 1 && t <= 19) ? MANEUVER_ICON[t] : nullptr;

  drawDividers(!alert);
  drawIcon(icon, POS_ICON_X, POS_ICON_Y, COL_BLUE);
  drawSignShape();

  // chu nho 8 px: BLE, pin/gio, so loi ra vong xuyen, km va phut con lai
  drawTopBar();
  if (icon && t >= 10 && t <= 19 && ICON_RING_CX[t] >= 0 && hud.exitNo > 0) {
    int rcx = POS_ICON_X + ICON_RING_CX[t] + 1, rcy = POS_ICON_Y + ICON_RING_CY[t] + 1;     // tam vanh (toa do mep)
    drawInkCenterXY(FR_SMALL, String(hud.exitNo), rcx, rcy, COL_WHITE);
  }
  if (!alert) {
    drawInkLeft(FR_SMALL, fmtKm(hud.remainM), POS_KM_X, POS_BOTTOM_Y, COL_WHITE);
    drawInkLeft(FR_SMALL, fmtMin(hud.remainMin), POS_MIN_X, POS_BOTTOM_Y, COL_WHITE);
  }

  drawInkLeft(FR_LABEL, turnLabel(t), POS_LABEL_X, POS_LABEL_Y, COL_WHITE);
  if (alert) {
    const AlertEntry& a = hud.alerts[0];
    String d = fmtDistBig(a.distance);
    drawInkCenterX(FR_LABEL, d, POS_STREET_CX, POS_BOTTOM_Y - 1, COL_WHITE);
  }
  drawInkCenterX(FR_DIST, fmtDistBig(hud.distance), POS_DIST_CX, POS_DIST_Y, COL_WHITE);
  drawSignNumber();

  if (alert) drawInkCenterX(FR_STREET, String(alertName(hud.alerts[0].kind)), POS_STREET_CX, POS_STREET_Y,
                            alertIsRed(hud.alerts[0].kind) ? COL_RED : COL_AMBER);
  else drawStreet(hud.streetNext.length() ? hud.streetNext : hud.street, COL_AMBER, tick);
}

// ---- chay tu do (nav = 0): dong ho lon + bien gioi han + ten duong hien tai ----
static void drawCruiseScreen(uint32_t tick) {
  drawDividers(false);
  drawSignShape();
  drawTopBar();
  drawInkLeft(FR_SMALL, fmtDate(true), POS_KM_X, POS_BOTTOM_Y, COL_WHITE);
  drawInkLeft(FR_LABEL, "FREE DRIVE", POS_LABEL_X, POS_LABEL_Y, COL_WHITE);
  drawInkCenterX(FR_CLOCK, getClockText(), POS_ICON_X + 15, 44, COL_WHITE);
  drawSignNumber();
  drawStreet(hud.street.length() ? hud.street : String(DEVICE_NAME), COL_AMBER, tick);
}

// ---- man hinh cho ----
static void drawSdBadge() {                                  // "SD" + cham xanh (co the) / do (khong co)
  const int left = 53;
  drawInkLeft(FR_SMALL, "SD", left, POS_BOTTOM_Y, COL_WHITE);
  spr.drawSpot(left + 11 + 5 + 2.5f, POS_BOTTOM_Y + 2.5f, 3.0f, sdOK ? COL_GREEN : COL_RED, COL_BG);
}

static void drawIdleScreen(uint32_t tick) {
  const char* msg;
  if (!bleConnected)                           msg = "WAITING FOR PHONE";
  else if (!bleSubscribed)                     msg = "CONNECTING...";
  else if (lastPacketTime > 0 && !dataFresh()) msg = "SIGNAL LOST";
  else                                         msg = "WAITING FOR ROUTE";

  drawDividers(false);
  drawTopBar();
  drawInkCenterX(FR_SMALL, fmtDate(false), 64, 56, COL_GREY);
  drawSdBadge();
  drawInkCenterX(FR_LABEL, String(msg), 64, 70, COL_WHITE);
  drawInkCenterX(FR_CLOCK, getClockText(), 64, 30, COL_WHITE);
  drawInkCenterX(FR_STREET, String(DEVICE_NAME), POS_STREET_CX, POS_STREET_Y, COL_AMBER);
}

// ---- ve lai toan bo ----
void renderHUD() {
  spr.fillSprite(COL_BG);
  uint32_t tick = millis();
  if (screen == SCR_HUD && hud.nav) drawNavScreen(tick);
  else if (screen == SCR_HUD)       drawCruiseScreen(tick);
  else                              drawIdleScreen(tick);
  endFonts();
  spr.pushSprite(0, 0);
}

static uint16_t spriteBg(uint16_t x, uint16_t y) { return spr.readPixel(x, y); }   // nen that de tron chu muot

static uint16_t* frameScratch() { return (uint16_t*)spr.getPointer(); }            // bo nho sprite dung tam cho khung Mochi

void drawStartupScreen() {
  spr.fillSprite(COL_BG);
  drawInkCenterX(FR_STREET, String(DEVICE_NAME), 64, 52, COL_AMBER);
  drawInkCenterX(FR_SMALL, String("v" FIRMWARE_VER), 64, 68, COL_GREY);
  drawInkCenterX(FR_LABEL, String("STARTING..."), 64, 88, COL_WHITE);
  endFonts();
  spr.pushSprite(0, 0);
}

// ============================================================
// SD / WIFI (chi de lay gio)
// ============================================================

void scanAnimationFolders();

void initSD() {
  Serial.println("Enji: initializing SD...");
  if (SD.begin(SD_CS, TFT_eSPI::getSPIinstance(), SD_SPI_HZ)) {
    sdOK = true;
    Serial.printf("SD OK, size: %llu MB\n", SD.cardSize() / (1024ULL * 1024ULL));
    scanAnimationFolders();
  } else {
    sdOK = false;
    Serial.println("SD not detected (Enji HUD still works without Mochi)");
  }
}


// ============================================================
// ENJI / DASAI MOCHI (BMP 24-bit hoac goi anim.r565 tu SD)
// ============================================================
//
// Khi KHONG co du lieu Waze:
//   man hinh cho (IDLE_BEFORE_MOCHI_MS) -> 1 folder Mochi bat ky -> man hinh cho -> ...
// Moi folder la mot Mochi va chi chay dung 1 lan. Co du lieu Waze thi HUD hien ngay.
//
// Hai dinh dang (uu tien dinh dang 1):
//   1) /N/anim.r565   tat ca frame RGB565 trong 1 file (mochi_pack.py) -> doc 1 lan/frame, khong mo file
//   2) /N/frame_000.bmp ...   BMP 24-bit 128x128 -> doc theo khoi 16 dong, ve 1 lan/frame

#define MOCHI_FPS              15
#define MOCHI_SKIP_WHEN_LATE   1        // 1 = bi tre thi bo frame de giu dung toc do; 0 = ve du tung frame (co the cham)
#define MAX_ANIMATION_FOLDERS  128
#define MAX_FRAMES_PER_FOLDER  2000
#define BMP_LINE_BYTES         (SCREEN_W * 3)
#define BMP_CHUNK_ROWS         16
#define PACK_HEADER_BYTES      16
#define PACK_FRAME_BYTES       (SCREEN_W * SCREEN_H * 2)

uint16_t animationFolders[MAX_ANIMATION_FOLDERS];
uint16_t animationFolderCount = 0;
uint16_t frameIndices[MAX_FRAMES_PER_FOLDER];     // chi dung cho dinh dang BMP
uint16_t frameCount = 0;
uint16_t currentFolder = 0;

static bool     packedMode = false;
static File     packedFile;
static uint32_t packedNextFrame = 0;
static uint32_t mochiIntervalMs = 1000UL / MOCHI_FPS;
static uint8_t  bmpChunk[BMP_CHUNK_ROWS * BMP_LINE_BYTES];

static uint32_t waitSince = 0;                    // luc bat dau man hinh cho
static bool     mochiPrepared = false;            // da chon folder + nap danh sach frame cho lan chay ke tiep
static bool     mochiPrepareTried = false;
static uint32_t mochiStartMs = 0;
static uint32_t mochiNextPos = 0;
static uint32_t mochiDrawn = 0, mochiSkipped = 0, mochiBusyMs = 0;

uint16_t readLE16(const uint8_t* p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
uint32_t readLE32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
int32_t readLE32S(const uint8_t* p) { return (int32_t)readLE32(p); }

const char* basenameOf(const char* path) {
  const char* slash = strrchr(path, '/');
  return slash ? slash + 1 : path;
}

void scanAnimationFolders() {
  animationFolderCount = 0;
  if (!sdOK) return;
  File root = SD.open("/");
  if (!root) return;
  File entry;
  while ((entry = root.openNextFile())) {
    if (entry.isDirectory() && animationFolderCount < MAX_ANIMATION_FOLDERS) {
      const char* name = basenameOf(entry.name());
      char* end = nullptr;
      long n = strtol(name, &end, 10);
      if (name[0] && end && *end == '\0' && n >= 1 && n <= 65535) {
        animationFolders[animationFolderCount++] = (uint16_t)n;
      }
    }
    entry.close();
  }
  root.close();
  Serial.printf("Enji: found %u numeric animation folders on SD\n", animationFolderCount);
}

int compareFrameIndex(const void* a, const void* b) {
  uint16_t aa = *(const uint16_t*)a;
  uint16_t bb = *(const uint16_t*)b;
  return aa < bb ? -1 : (aa > bb ? 1 : 0);
}

void closePacked() {
  if (packedFile) packedFile.close();
  packedMode = false;
}

// Mo /N/anim.r565: 16 byte header "MCH1", w, h, so frame, fps (little-endian), roi den frame RGB565
bool tryOpenPacked(uint16_t folderNo) {
  char path[32];
  snprintf(path, sizeof(path), "/%u/anim.r565", folderNo);
  File f = SD.open(path, FILE_READ);
  if (!f) return false;
  uint8_t h[PACK_HEADER_BYTES];
  if (f.read(h, sizeof(h)) != sizeof(h) || memcmp(h, "MCH1", 4) != 0) { f.close(); return false; }
  uint16_t w = readLE16(h + 4), hh = readLE16(h + 6), n = readLE16(h + 8), fps = readLE16(h + 10);
  if (w != SCREEN_W || hh != SCREEN_H || n < 1) { f.close(); return false; }
  if ((uint64_t)f.size() < (uint64_t)PACK_HEADER_BYTES + (uint64_t)n * PACK_FRAME_BYTES) { f.close(); return false; }
  if (fps < 1 || fps > 60) fps = MOCHI_FPS;
  closePacked();
  packedFile = f;
  packedMode = true;
  frameCount = n;
  packedNextFrame = 0;
  mochiIntervalMs = 1000UL / fps;
  return true;
}

bool loadFramesForFolder(uint16_t folderNo) {
  frameCount = 0;
  closePacked();
  if (tryOpenPacked(folderNo)) {
    currentFolder = folderNo;
    Serial.printf("Enji: folder %u packed, %u frames @ %lu ms\n", currentFolder, frameCount, (unsigned long)mochiIntervalMs);
    return true;
  }

  char folderPath[20];
  snprintf(folderPath, sizeof(folderPath), "/%u", folderNo);
  File dir = SD.open(folderPath);
  if (!dir || !dir.isDirectory()) { if (dir) dir.close(); return false; }

  File entry;
  while ((entry = dir.openNextFile())) {
    if (!entry.isDirectory() && frameCount < MAX_FRAMES_PER_FOLDER) {
      const char* base = basenameOf(entry.name());
      unsigned int idx = 0;
      char extra = 0;
      if (sscanf(base, "frame_%u.bmp%c", &idx, &extra) == 1 && idx <= 65535) {
        frameIndices[frameCount++] = (uint16_t)idx;
      }
    }
    entry.close();
  }
  dir.close();
  if (frameCount < 1) return false;
  qsort(frameIndices, frameCount, sizeof(frameIndices[0]), compareFrameIndex);
  mochiIntervalMs = 1000UL / MOCHI_FPS;
  currentFolder = folderNo;
  Serial.printf("Enji: folder %u has %u BMP frames (khuyen nghi dong goi anim.r565 de muot hon)\n", currentFolder, frameCount);
  return true;
}

bool isUpsetFolder(uint16_t n) {
  return n == 3 || n == 6 || n == 7 || n == 10;
}

bool chooseRandomFolder(MochiMode mode) {
  if (!sdOK || animationFolderCount == 0) return false;
  // Thu ngau nhien nhieu lan, tranh lap lai folder vua chay neu co lua chon khac.
  for (uint16_t attempt = 0; attempt < animationFolderCount * 3; ++attempt) {
    uint16_t candidate = animationFolders[random(animationFolderCount)];
    if (mode == MOCHI_UPSET && !isUpsetFolder(candidate)) continue;
    if (candidate == currentFolder && animationFolderCount > 1) continue;
    if (loadFramesForFolder(candidate)) return true;
  }
  // Chi co mot folder upset hop le: lap lai folder do thay vi chuyen sang cam xuc khac.
  if (mode == MOCHI_UPSET) {
    if (currentFolder && isUpsetFolder(currentFolder) && loadFramesForFolder(currentFolder)) return true;
    for (uint16_t i = 0; i < animationFolderCount; ++i) {
      uint16_t candidate = animationFolders[i];
      if (isUpsetFolder(candidate) && loadFramesForFolder(candidate)) return true;
    }
    for (uint16_t i = 0; i < animationFolderCount; ++i) {          // khong co folder 3/6/7/10 nao
      if (loadFramesForFolder(animationFolders[i])) return true;
    }
  }
  // MOCHI_NORMAL: neu lan thu ngau nhien khong nap duoc, thu lan luot de khong bo sot folder hop le
  if (mode == MOCHI_NORMAL) {
    for (uint16_t i = 0; i < animationFolderCount; ++i) {
      if (animationFolders[i] == currentFolder && animationFolderCount > 1) continue;
      if (loadFramesForFolder(animationFolders[i])) return true;
    }
  }
  return false;
}

// ---- ve 1 frame: doc ca frame vao bo nho sprite roi day len TFT mot lan (nhanh hon ve tung dong) ----

bool drawPackedFrame(uint32_t idx) {
  if (!packedFile) return false;
  if (idx != packedNextFrame) {
    if (!packedFile.seek(PACK_HEADER_BYTES + idx * (uint32_t)PACK_FRAME_BYTES)) return false;
  }
  uint16_t* fb = frameScratch();
  if (packedFile.read((uint8_t*)fb, PACK_FRAME_BYTES) != (size_t)PACK_FRAME_BYTES) { packedNextFrame = 0xFFFFFFFFUL; return false; }
  packedNextFrame = idx + 1;
  tft.pushImage(0, 0, SCREEN_W, SCREEN_H, fb);
  return true;
}

bool drawBmpFrame(uint16_t folderNo, uint16_t frameNo) {
  char path[48];
  snprintf(path, sizeof(path), "/%u/frame_%03u.bmp", folderNo, frameNo);
  File bmp = SD.open(path, FILE_READ);
  if (!bmp) return false;

  uint8_t header[54];
  if (bmp.read(header, sizeof(header)) != sizeof(header) || header[0] != 'B' || header[1] != 'M') {
    bmp.close(); return false;
  }
  uint32_t pixelOffset = readLE32(header + 10);
  int32_t width = readLE32S(header + 18);
  int32_t height = readLE32S(header + 22);
  uint16_t planes = readLE16(header + 26);
  uint16_t bpp = readLE16(header + 28);
  uint32_t compression = readLE32(header + 30);
  int32_t absH = height < 0 ? -height : height;
  if (width != SCREEN_W || absH != SCREEN_H || planes != 1 || bpp != 24 || compression != 0) {
    bmp.close(); return false;
  }

  const uint32_t rowSize = ((uint32_t)width * 3 + 3) & ~3UL;       // 384 B, da chia het cho 4
  uint16_t* fb = frameScratch();
  bool bottomUp = height > 0;
  if (!bmp.seek(pixelOffset)) { bmp.close(); return false; }
  for (int r0 = 0; r0 < SCREEN_H; r0 += BMP_CHUNK_ROWS) {
    int rows = (SCREEN_H - r0 < BMP_CHUNK_ROWS) ? (SCREEN_H - r0) : BMP_CHUNK_ROWS;
    size_t need = (size_t)rows * rowSize;
    if (bmp.read(bmpChunk, need) != need) { bmp.close(); return false; }
    for (int i = 0; i < rows; ++i) {
      const uint8_t* src = bmpChunk + (size_t)i * rowSize;
      int y = bottomUp ? (SCREEN_H - 1 - (r0 + i)) : (r0 + i);
      uint16_t* dst = fb + (size_t)y * SCREEN_W;
      for (int x = 0; x < SCREEN_W; ++x, src += 3) {
        dst[x] = ((uint16_t)(src[2] & 0xF8) << 8) | ((uint16_t)(src[1] & 0xFC) << 3) | (src[0] >> 3);
      }
    }
  }
  bmp.close();
  tft.pushImage(0, 0, SCREEN_W, SCREEN_H, fb);
  return true;
}

bool drawMochiFrameAt(uint32_t pos) {
  if (packedMode) return drawPackedFrame(pos);
  return drawBmpFrame(currentFolder, frameIndices[pos]);
}

// ---- luong man hinh: HUD / cho / Mochi ----

// Chon loai Mochi theo tinh trang ket noi (giu logic V0.5)
MochiMode pickMochiMode() {
  if (!bleConnected) return MOCHI_UPSET_WHEN_DISCONNECTED ? MOCHI_UPSET : MOCHI_NORMAL;
  if (lastPacketTime > 0 && !dataFresh()) return MOCHI_UPSET;       // dang co du lieu roi mat tin hieu
  return MOCHI_NORMAL;                                              // da ket noi, chua nhan goi nao
}

void stopMochi() {
  closePacked();
}

void enterWait(uint32_t now, const char* why) {
  if (screen == SCR_MOCHI) stopMochi();
  screen = SCR_WAIT;
  waitSince = now;
  mochiPrepared = false;
  mochiPrepareTried = false;
  resetWaitPending = false;
  dirty = true;
  Serial.printf("Enji: man hinh cho (%s), Mochi sau %lu s\n", why, (unsigned long)(IDLE_BEFORE_MOCHI_MS / 1000));
}

// Chon folder va nap danh sach frame san trong luc dang o man hinh cho (tranh khung hinh khi bat dau chay)
bool prepareMochi() {
  if (!sdOK || animationFolderCount == 0) return false;
  MochiMode mode = pickMochiMode();
  bool ok = chooseRandomFolder(mode);
  Serial.printf("Enji: chuan bi Mochi (%s): %s\n", mode == MOCHI_UPSET ? "UPSET" : "NORMAL",
                ok ? "san sang" : "khong co folder phat duoc");
  return ok;
}

void startMochi(uint32_t now) {
  screen = SCR_MOCHI;
  mochiStartMs = now;
  mochiNextPos = 0;
  mochiDrawn = mochiSkipped = mochiBusyMs = 0;
  packedNextFrame = 0;
  if (packedMode && packedFile) packedFile.seek(PACK_HEADER_BYTES);
  Serial.printf("Enji: chay Mochi folder %u (%u frame, %s)\n", currentFolder, frameCount, packedMode ? "goi anim.r565" : "BMP");
}

void finishMochi() {
  uint32_t avg = mochiDrawn ? mochiBusyMs / mochiDrawn : 0;
  Serial.printf("Enji: Mochi folder %u xong: ve %lu frame, bo %lu, TB %lu ms/frame (~%lu fps toi da)\n",
                currentFolder, (unsigned long)mochiDrawn, (unsigned long)mochiSkipped,
                (unsigned long)avg, (unsigned long)(avg ? 1000 / avg : 0));
}

// Tra ve false khi het 1 folder
bool stepMochi(uint32_t now) {
  if (mochiNextPos >= frameCount) {                                 // frame cuoi van giu du thoi gian cua no
    return (now - mochiStartMs) < (uint32_t)frameCount * mochiIntervalMs;
  }
  uint32_t target = (now - mochiStartMs) / mochiIntervalMs;         // frame "ly tuong" tai thoi diem nay
  if (target < mochiNextPos) return true;                           // chua den gio frame ke tiep
  uint32_t pos = mochiNextPos;
#if MOCHI_SKIP_WHEN_LATE
  if (target > pos) pos = (target < frameCount) ? target : (uint32_t)frameCount - 1;
#endif
  if (pos > mochiNextPos) mochiSkipped += pos - mochiNextPos;
  uint32_t t0 = millis();
  bool ok = drawMochiFrameAt(pos);
  mochiBusyMs += millis() - t0;
  mochiDrawn++;
  mochiNextPos = pos + 1;
  if (!ok) Serial.printf("Enji: bo qua frame khong doc duoc (folder %u, frame %lu)\n", currentFolder, (unsigned long)pos);
  return true;
}

void updateScreen(uint32_t now) {
  // Co du lieu Waze (hoac demo) -> HUD ngay, ngat Mochi neu dang chay
  bool hudData = dataFresh() && (bleConnected || demoMode);
  if (hudData) {
    if (screen != SCR_HUD) {
      if (screen == SCR_MOCHI) stopMochi();
      screen = SCR_HUD;
      resetWaitPending = false;
      dirty = true;
      Serial.println("Enji: co du lieu Waze -> HUD");
    }
    return;
  }

  if (screen == SCR_HUD) { enterWait(now, "het du lieu"); return; }
  if (resetWaitPending)  { enterWait(now, "BLE doi trang thai"); return; }

  if (screen == SCR_WAIT) {
    // Sau 0.5 s (idle da ve xong) moi chuan bi Mochi de khong lam tre khung dau tien
    if (!mochiPrepareTried && now - waitSince >= 500) {
      mochiPrepareTried = true;
      mochiPrepared = prepareMochi();
    }
    if (now - waitSince >= IDLE_BEFORE_MOCHI_MS) {
      if (mochiPrepared) {
        startMochi(now);
      } else {                                                      // khong co Mochi de chay: cho them roi thu lai
        waitSince = now;
        mochiPrepareTried = false;
      }
    }
  } else if (screen == SCR_MOCHI) {
    if (!stepMochi(now)) {
      finishMochi();
      enterWait(now, "het 1 Mochi");
    }
  }
}


bool wifiConfigured() {
  return WIFI_SSID && WIFI_SSID[0] && strcmp(WIFI_SSID, "YOUR_WIFI_SSID") != 0;
}

void startWiFiForTime() {
  if (!wifiConfigured()) return;
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, "pool.ntp.org", "time.google.com", "time.cloudflare.com");
  Serial.println("WiFi: lay gio NTP...");
}

void pollTimeSync() {
  if (!wifiConfigured() || timeSynced) return;
  static uint32_t lastTry = 0;
  struct tm t;
  if (getNow(t)) {
    timeSynced = true;
    Serial.println("Da dong bo gio, tat WiFi de nhuong song cho BLE");
    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_OFF);
    dirty = true;
    return;
  }
  if (WiFi.status() != WL_CONNECTED && millis() - lastTry > 20000) {
    lastTry = millis();
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
}

// ============================================================
// SETUP / LOOP
// ============================================================

// Moc chan doan: in tung buoc khoi dong; neu bi treo/khoi dong lai thi dong "[setup] ..." cuoi cung cho biet buoc nao chet
#define STEP(msg) do { Serial.println("[setup] " msg); Serial.flush(); delay(30); } while (0)

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("==========================");
  Serial.println(" ENJI ESP32-S3 V" FIRMWARE_VER);
  Serial.println("==========================");

  pinMode(SD_CS, OUTPUT);  digitalWrite(SD_CS, HIGH);       // CS cua SD len HIGH truoc khi khoi tao man hinh
  pinMode(PIN_BOOT, INPUT_PULLUP);
  STEP("1 chan GPIO xong");

  tft.init();
  STEP("2 tft.init() xong");                                               // TFT_eSPI tu bat SPI theo build_opt.h (MOSI 7, SCLK 6, MISO 8)
  tft.setRotation(SCREEN_ROTATION);
  tft.setSwapBytes(true);                                   // cho tft.pushImage (Mochi): du lieu RGB565 little-endian
  STEP("3 setRotation/setSwapBytes xong");
  tft.fillScreen(COL_BG);
  STEP("4 fillScreen xong");

  spr.setColorDepth(16);
  if (!spr.createSprite(SCREEN_W, SCREEN_H)) {
    Serial.println("Khong tao duoc sprite 128x128 (thieu RAM?)");
    tft.fillScreen(COL_RED);
    while (true) delay(1000);
  }
  STEP("5 sprite 128x128 da tao");
  spr.setCallback(spriteBg);                                // chu duoc tron voi diem anh nen that trong sprite
  spr.setTextWrap(false);

  FR_LABEL  = fontMake(FontLabel10);
  FR_SMALL  = fontMake(FontSmall8);
  FR_STREET = fontMake(FontStreet10);
  FR_SPEED  = fontMake(FontSpeed18);
  FR_SPEED3 = fontMake(FontSpeed14);
  FR_DIST   = fontMake(FontDist16);
  FR_CLOCK  = fontMake(FontClock24);

  STEP("6 font da doc thong so");
  drawStartupScreen();
  STEP("7 man hinh khoi dong da ve");
  initSD();
  STEP("8 SD xong");
  initBLE();
  STEP("9 BLE xong");
  startWiFiForTime();
  STEP("10 WiFi/NTP xong");

  demoMode = DEMO_ON_BOOT;
  screen = SCR_WAIT;                                        // bat nguon: man hinh cho 10 s roi moi toi Mochi
  waitSince = millis();
  renderHUD();
}

void loop() {
  uint32_t now = millis();

  pollBootButton();
  pollBLE();
  pollTimeSync();
  if (demoMode) demoTick();

  // Nhap nhay chu "QUA TOC DO!" (chi khi dang o man hinh HUD)
  if (screen == SCR_HUD && isOverspeed()) {
    if (now - lastBlink >= 400) {
      lastBlink = now;
      overspeedBlinkOn = !overspeedBlinkOn;
      dirty = true;
    }
  } else if (overspeedBlinkOn) {
    overspeedBlinkOn = false;
    dirty = true;
  }

  updateScreen(now);                              // chuyen HUD / man hinh cho / Mochi, ve 1 frame Mochi neu den gio

  // Mochi ve thang len TFT; man hinh cho va HUD ve qua canvas
  if (screen != SCR_MOCHI && ((dirty && now - lastRender >= 60) || now - lastRender >= 250)) {
    lastRender = now;
    dirty = false;
    renderHUD();
  }

  delay(1);
}
