#include <WiFi.h>
#include <WebServer.h>
#include <NimBLEDevice.h>
#include <Update.h>
#include <ArduinoOTA.h>
#include <Preferences.h>

#if defined(__has_include)
  #if __has_include(<WebSocketsServer.h>)
    #define WITH_WS 1
  #endif
#endif
#ifndef WITH_WS
  #define WITH_WS 0
#endif
#if WITH_WS
  #include <WebSocketsServer.h>
#endif

#if defined(__has_include)
  #if __has_include("NimBLEConnInfo.h")
    #define NIMBLE2 1
  #endif
#endif
#ifndef NIMBLE2
  #define NIMBLE2 0
#endif

// نقشه‌ی کلیدها مثل قبل است تا جفت‌شدن تلویزیون معتبر بماند
#define WITH_MEDIA 1
#define CLEAR_BONDS_ON_BOOT 0
#define WITH_DIS 1
#define KEYDIST 1
#define SEC_REQ 1
#define ENC_MAP 1

const char* AP_SSID = "ESP-Touchpad";
const char* AP_PASS = "12345678";

WebServer server(80);
NimBLEServer* bleServer;
NimBLEAdvertising* adv;
NimBLECharacteristic* kbIn;
NimBLECharacteristic* msIn;
#if WITH_MEDIA
NimBLECharacteristic* mdIn;
#endif

volatile uint16_t secHandle = 0xFFFF;
volatile unsigned long secAt = 0;

uint8_t heldBtn = 0;          // دکمه‌ی نگه‌داشته‌شده‌ی ماوس (Drag)
bool jiggle = false;          // ماوس‌جیگلر
unsigned long lastJig = 0;

static const uint8_t reportMap[] = {
  // Keyboard (ID 1)
  0x05,0x01, 0x09,0x06, 0xA1,0x01, 0x85,0x01,
  0x05,0x07, 0x19,0xE0, 0x29,0xE7, 0x15,0x00, 0x25,0x01,
  0x75,0x01, 0x95,0x08, 0x81,0x02,
  0x95,0x01, 0x75,0x08, 0x81,0x01,
  0x95,0x06, 0x75,0x08, 0x15,0x00, 0x25,0x65,
  0x05,0x07, 0x19,0x00, 0x29,0x65, 0x81,0x00,
  0xC0,
  // Mouse (ID 2)
  0x05,0x01, 0x09,0x02, 0xA1,0x01, 0x85,0x02,
  0x09,0x01, 0xA1,0x00,
  0x05,0x09, 0x19,0x01, 0x29,0x03, 0x15,0x00, 0x25,0x01,
  0x95,0x03, 0x75,0x01, 0x81,0x02,
  0x95,0x01, 0x75,0x05, 0x81,0x03,
  0x05,0x01, 0x09,0x30, 0x09,0x31, 0x09,0x38,
  0x15,0x81, 0x25,0x7F, 0x75,0x08, 0x95,0x03, 0x81,0x06,
  0xC0, 0xC0
#if WITH_MEDIA
  ,
  // Consumer / Media (ID 3)
  0x05,0x0C, 0x09,0x01, 0xA1,0x01, 0x85,0x03,
  0x15,0x00, 0x26,0xFF,0x03,
  0x19,0x00, 0x2A,0xFF,0x03,
  0x75,0x10, 0x95,0x01, 0x81,0x00,
  0xC0
#endif
};

// ---------- ثبت رویدادهای بلوتوث (صفحه‌ی /log) ----------
const int LOGN = 24;
char logs[LOGN][80];
volatile int logHead = 0;

void logEv(const String& s) {
  int i = logHead % LOGN;
  snprintf(logs[i], sizeof(logs[i]), "%lu.%lus %s",
           millis() / 1000UL, (millis() / 100UL) % 10UL, s.c_str());
  logHead = logHead + 1;
}

class SrvCb : public NimBLEServerCallbacks {
#if NIMBLE2
  void onConnect(NimBLEServer* s, NimBLEConnInfo& ci) {
    logEv("CONNECT");
#if SEC_REQ
    secHandle = ci.getConnHandle();
    secAt = millis() + 400;
#endif
  }
  void onDisconnect(NimBLEServer* s, NimBLEConnInfo& ci, int reason) {
    secHandle = 0xFFFF;
    logEv(String("DISCONNECT reason=") + reason);
  }
  void onAuthenticationComplete(NimBLEConnInfo& ci) {
    logEv(String("AUTH done enc=") + (int)ci.isEncrypted() +
          " bond=" + (int)ci.isBonded() + " auth=" + (int)ci.isAuthenticated());
  }
  void onConfirmPassKey(NimBLEConnInfo& ci, uint32_t pin) {
    logEv(String("CONFIRM requested pin=") + pin);
    NimBLEDevice::injectConfirmPasskey(ci, true);
  }
#else
  void onConnect(NimBLEServer* s) {
    logEv("CONNECT");
  }
  void onConnect(NimBLEServer* s, ble_gap_conn_desc* d) {
#if SEC_REQ
    secHandle = d->conn_handle;
    secAt = millis() + 400;
#endif
  }
  void onDisconnect(NimBLEServer* s) {
    secHandle = 0xFFFF;
    logEv("DISCONNECT");
  }
  void onAuthenticationComplete(ble_gap_conn_desc* d) {
    logEv(String("AUTH done enc=") + (int)d->sec_state.encrypted +
          " bond=" + (int)d->sec_state.bonded + " auth=" + (int)d->sec_state.authenticated);
  }
  bool onConfirmPIN(uint32_t pin) {
    logEv(String("CONFIRM requested pin=") + pin);
    return true;
  }
#endif
};

class ChrCb : public NimBLECharacteristicCallbacks {
#if NIMBLE2
  void onRead(NimBLECharacteristic* c, NimBLEConnInfo& ci) {
    logEv(String("READ ") + c->getUUID().toString().c_str());
  }
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& ci) {
    logEv(String("WRITE ") + c->getUUID().toString().c_str());
  }
  void onSubscribe(NimBLECharacteristic* c, NimBLEConnInfo& ci, uint16_t subValue) {
    logEv(String("SUBSCRIBE ") + c->getUUID().toString().c_str() + " v=" + subValue);
  }
#else
  void onRead(NimBLECharacteristic* c) {
    logEv(String("READ ") + c->getUUID().toString().c_str());
  }
  void onWrite(NimBLECharacteristic* c) {
    logEv(String("WRITE ") + c->getUUID().toString().c_str());
  }
  void onSubscribe(NimBLECharacteristic* c, ble_gap_conn_desc* d, uint16_t subValue) {
    logEv(String("SUBSCRIBE ") + c->getUUID().toString().c_str() + " v=" + subValue);
  }
#endif
};
static ChrCb chrCb;

bool isConnected() {
  return bleServer->getConnectedCount() > 0;
}

void blinkN(int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(2, HIGH); delay(200);
    digitalWrite(2, LOW);  delay(200);
  }
  delay(600);
}

void sendMouse(uint8_t b, int8_t x, int8_t y, int8_t w) {
  if (!isConnected()) return;
  uint8_t r[4] = { b, (uint8_t)x, (uint8_t)y, (uint8_t)w };
  msIn->setValue(r, 4);
  msIn->notify();
}

void sendKey(uint8_t mod, uint8_t code) {
  if (!isConnected()) return;
  uint8_t r[8] = { mod, 0, code, 0, 0, 0, 0, 0 };
  kbIn->setValue(r, 8);
  kbIn->notify();
  delay(25);
  uint8_t z[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
  kbIn->setValue(z, 8);
  kbIn->notify();
  delay(25);
}

const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no,viewport-fit=cover">
<meta name="theme-color" content="#000000">
<title>ESP Mouse v1</title>
<style>
:root{--bg:#000;--card:rgba(255,255,255,.05);--line:rgba(255,255,255,.1);--acc:#3e91ff;--acc2:#1a6fe0;--ok:#3e91ff;--txt:#ece9e2;--mut:#9a968c}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
html,body{margin:0}
body{background:var(--bg);color:var(--txt);
font-family:system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;touch-action:manipulation;
user-select:none;-webkit-user-select:none;min-height:100vh}
#app{max-width:900px;margin:0 auto;
padding:max(18px,env(safe-area-inset-top)) max(22px,env(safe-area-inset-right)) max(32px,env(safe-area-inset-bottom)) max(22px,env(safe-area-inset-left))}
button{font:inherit;color:inherit;border:1px solid var(--line);background:var(--card);border-radius:14px;padding:13px 10px;font-size:15px}
button:active{filter:brightness(1.5)}
.tp{touch-action:none}

.padwrap{display:flex;gap:8px;padding-top:2px}
#pad{position:relative;flex:1;height:clamp(220px,42vh,460px);border-radius:20px;border:1px solid var(--line);
background-image:radial-gradient(rgba(255,255,255,.07) 1px,transparent 1px),radial-gradient(circle at 50% 35%,#2b2a28,#232220);
background-size:20px 20px,100% 100%;display:flex;align-items:center;justify-content:center;text-align:center;padding:10px;
transition:box-shadow .15s,border-color .15s}
#pad.act{border-color:var(--acc);box-shadow:0 0 0 2px rgba(62,145,255,.35),inset 0 0 40px rgba(62,145,255,.15)}
.padtxt{pointer-events:none}
.padtxt b{display:block;font-size:17px;margin-bottom:6px;color:#cbd5ff}
.padtxt small{display:block;font-size:12px;line-height:1.7;color:var(--mut)}
#sc{flex:0 0 56px;height:auto;border-radius:20px;border:1px solid var(--line);
background:repeating-linear-gradient(0deg,rgba(255,255,255,.18) 0 2px,transparent 2px 14px),linear-gradient(#18203f,#10152e);background-size:100% 14px,100% 100%;background-position:0 0,0 0;display:flex;flex-direction:column;align-items:center;
justify-content:space-between;padding:12px 0;color:var(--mut);font-size:12px}
#sc em{font-style:normal;writing-mode:vertical-rl;letter-spacing:2px}
#hs{margin:10px 0;height:48px;border-radius:14px;border:1px solid var(--line);
background:repeating-linear-gradient(90deg,rgba(255,255,255,.18) 0 2px,transparent 2px 14px),linear-gradient(90deg,#10152e,#18203f,#10152e);background-size:14px 100%,100% 100%;background-position:0 0,0 0;display:flex;align-items:center;
justify-content:center;color:var(--mut);font-size:12px;letter-spacing:2px}

.seg3{display:grid;grid-template-columns:1fr 1fr 1fr;border-radius:14px;overflow:hidden;border:1px solid var(--line)}
.seg3 button{border:0;border-radius:0;padding:16px 0;font-weight:600}
.seg3 button+button{border-left:1px solid var(--line)}
.row2{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin:10px 0}
.card{background:var(--card);border:1px solid var(--line);border-radius:16px;padding:10px;margin:10px 0}
.opts{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.tgl{display:flex;align-items:center;justify-content:space-between;gap:8px;text-align:left;font-size:14px}
.tgl i{width:38px;height:22px;border-radius:12px;background:#3a3937;position:relative;flex:none;transition:background .15s}
.tgl i::after{content:'';position:absolute;top:3px;left:3px;width:16px;height:16px;border-radius:50%;background:#fff;transition:left .15s}
.tgl.on i{background:var(--ok)}
.tgl.on i::after{left:19px}
.sl{display:flex;align-items:center;gap:10px;padding:6px 4px;font-size:14px}
.sl span{width:52px;color:var(--mut)}
.sl b{width:38px;text-align:right;font-weight:600}
input[type=range]{flex:1;-webkit-appearance:none;appearance:none;height:6px;border-radius:4px;background:#3a3937;outline:none}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:22px;height:22px;border-radius:50%;
background:linear-gradient(135deg,var(--acc),var(--acc2));border:2px solid #fff}
#sc,#hs{position:relative;background-image:none;background-color:#232220;overflow:hidden}
#sc canvas,#hs canvas{position:absolute;left:0;top:0;width:100%;height:100%;pointer-events:none}
#sc span,#sc em,#hs{z-index:1}
.dsc{display:none}
.hi{display:inline-flex;align-items:center;justify-content:center;width:20px;height:20px;border-radius:50%;border:1px solid var(--line);background:var(--card);color:var(--mut);font-size:12px;font-weight:700;flex:none}
.fr{display:flex;align-items:center;gap:4px;min-width:0}
.fr .tgl{flex:1;min-width:0;padding:8px 6px;font-size:12px;gap:4px}
.fr .tgl i{width:30px;height:18px}
.fr .tgl i::after{width:12px;height:12px}
.fr .tgl.on i::after{left:15px}
#trio{display:grid;grid-template-columns:1fr 1fr 1fr;gap:6px;margin-bottom:8px}
#tip{position:absolute;left:0;right:0;background:#1f1e1d;border:1px solid #3a3937;border-radius:14px;padding:14px;font-size:13px;line-height:1.5;display:none;z-index:9;box-shadow:0 8px 24px rgba(0,0,0,.5)}
#fb{position:relative}
.nav svg{width:26px;height:26px;fill:none;stroke:#fff;stroke-width:2;stroke-linecap:round;stroke-linejoin:round;pointer-events:none}
.g5{display:grid;grid-template-columns:1fr 1fr 1.5fr 1fr 1fr;gap:8px}
.g2{display:grid;grid-template-columns:1fr 1fr;gap:8px}
#md button{padding:12px 2px;display:flex;flex-direction:column;align-items:center;gap:4px;font-size:11px;color:var(--mut)}
#md svg{width:24px;height:24px;fill:#fff;pointer-events:none}
#md .card{margin:8px 0 0}
.nav button{padding:12px 0}
#pad.drag{border-color:#fff;box-shadow:0 0 0 3px rgba(62,145,255,.7),0 0 28px rgba(62,145,255,.8),inset 0 0 50px rgba(62,145,255,.35)}
#pad.drag::after{content:'DRAG';position:absolute;top:10px;left:50%;transform:translateX(-50%);padding:3px 12px;border-radius:12px;background:var(--acc);color:#fff;font-size:12px;font-weight:700;letter-spacing:2px}
.lower{display:flex;gap:8px;margin-top:8px;align-items:stretch}
.lcol{flex:1;min-width:0}
.padwrap{position:sticky;top:0;z-index:20;background:var(--bg);padding-bottom:8px;margin-top:-2px}
#fsb{position:absolute;top:12px;right:12px;z-index:25;width:32px;height:28px;padding:0;border:0;border-radius:8px;background:transparent;display:flex;align-items:center;justify-content:center}
#fsb svg{width:20px;height:20px;fill:none;stroke:#fff;stroke-width:2;stroke-linecap:round;stroke-linejoin:round;pointer-events:none}
#sdots{position:absolute;right:12px;top:50%;transform:translateY(-50%);display:flex;gap:6px;pointer-events:none}
#sdots i{width:9px;height:9px;border-radius:50%;background:#e5484d}
#sdots i.ok{background:#30d158}
.sech{width:100%;display:flex;align-items:center;justify-content:space-between;background:transparent;border:0;border-radius:10px;padding:14px 8px;font-size:16px;font-weight:700}
.sech svg{width:22px;height:22px;fill:none;stroke:#fff;stroke-width:2;stroke-linecap:round;stroke-linejoin:round;transition:transform .2s;pointer-events:none}
.sech.closed svg{transform:rotate(180deg)}
#morebody{margin-top:6px;overflow:hidden;max-height:1600px;opacity:1;transition:max-height .45s ease,opacity .3s ease,margin .45s ease,visibility .45s}
#morebody.hide{max-height:0;opacity:0;margin-top:0;visibility:hidden}
.gt{margin:16px 0 8px;padding:14px 4px 0;border-top:2px solid rgba(255,255,255,.3);font-size:12px;letter-spacing:1px;color:var(--acc);font-weight:700}
.gt:first-child{margin-top:4px;padding-top:0;border-top:0}
.gb{display:grid;grid-template-columns:1fr;gap:8px}
.gb>button{width:60%}
#morebody button{text-align:left;transition:background .25s,border-color .25s}
#morebody button.sel{border-color:var(--acc);background:rgba(62,145,255,.18)}
.ptitle{margin:4px 4px 8px;font-size:13px;font-weight:700;letter-spacing:1px;color:var(--acc)}
.gb>.pn{max-height:0;opacity:0;overflow:hidden;margin:0;visibility:hidden;transition:max-height .4s ease,opacity .3s ease,margin .4s ease,padding .4s ease,border-width .4s ease,visibility .4s}
.gb>.pn.open{max-height:900px;opacity:1;margin:8px 0 4px;visibility:visible}
.card.pn{padding-top:0;padding-bottom:0;border-width:0}
.card.pn.open{padding-top:10px;padding-bottom:10px;border-width:1px}
#sc::after{content:'';position:absolute;left:0;right:0;top:50%;height:2px;margin-top:-1px;background:var(--acc);box-shadow:0 0 8px var(--acc);z-index:2;pointer-events:none}
#sc::before{content:'';position:absolute;left:0;top:50%;margin-top:-6px;border-style:solid;border-width:6px 0 6px 9px;border-color:transparent transparent transparent #fff;z-index:3;pointer-events:none}
.ic{width:18px;height:18px;margin-right:8px;vertical-align:-4px;fill:none;stroke:#fff;stroke-width:2;stroke-linecap:round;stroke-linejoin:round;pointer-events:none}
#moreh{position:relative}
#moreh #sdots{right:48px}
.sl{display:flex;flex-direction:column;align-items:stretch;gap:0;padding:6px 4px 2px;border-top:1px solid var(--line)}
.sl .sh{display:flex;align-items:center;gap:8px}
.sl .sh span{flex:1;width:auto;color:var(--txt);font-size:13px}
.sl .sh b{width:auto;min-width:36px;text-align:right;color:var(--acc);font-weight:700;font-size:13px}
.sl .sh b.hi{width:18px;height:18px;min-width:0;font-size:11px;color:var(--mut)}
input[type=range]{--r:.5;flex:none;width:100%;height:22px;margin:0;background:transparent;pointer-events:none}
input[type=range]::-webkit-slider-runnable-track{height:4px;border-radius:2px;background:linear-gradient(to right,var(--acc) calc(10px + var(--r)*(100% - 20px)),#3a3937 calc(10px + var(--r)*(100% - 20px)))}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;pointer-events:auto;width:20px;height:20px;margin-top:-8px;border:0;border-radius:50%;background:var(--acc);box-shadow:0 0 0 0 rgba(62,145,255,.3);transition:box-shadow .15s}
input[type=range]:active::-webkit-slider-thumb{box-shadow:0 0 0 8px rgba(62,145,255,.3)}
input[type=range]::-moz-range-track{height:4px;border-radius:2px;background:#3a3937}
input[type=range]::-moz-range-progress{height:4px;border-radius:2px;background:var(--acc)}
input[type=range]::-moz-range-thumb{pointer-events:auto;width:20px;height:20px;border:0;border-radius:50%;background:var(--acc)}
#trio{grid-template-columns:1fr 1fr 1fr;gap:4px;margin-bottom:4px}
.fr{border-top:0}
.fr .tgl{padding:6px 4px;font-size:11px;gap:3px;border-radius:10px}
.fr .tgl i{width:28px;height:16px}
.fr .tgl i::after{top:2px;left:2px;width:12px;height:12px}
.fr .tgl.on i::after{left:14px}
.card.pn.open{padding-top:6px;padding-bottom:6px}
.ptitle{margin:2px 4px 4px;font-size:12px}
#tr{display:block;margin:8px 0 0 auto;padding:4px 8px;border-radius:8px;border:1px solid var(--line);background:rgba(255,255,255,.08)}
#tr svg{width:24px;height:24px;display:block;pointer-events:none}
#tt.fa{direction:rtl;text-align:right}
#tip{position:absolute;left:14px;right:14px;top:0;z-index:50}
#ta{position:absolute;width:12px;height:12px;background:#1f1e1d;border:1px solid #3a3937;transform:rotate(45deg)}
#tip.up #ta{bottom:-7px;border-top:0;border-left:0}
#tip.dn #ta{top:-7px;border-bottom:0;border-right:0}
.tw{display:flex;align-items:center;gap:6px;width:72%}
.tw>button{flex:1;min-width:0}
.padtxt{transition:opacity .6s ease,transform .6s ease;will-change:opacity,transform}
.padtxt.gone{opacity:0;transform:scale(.94)}
</style></head><body>
<div id="app">
<div id="tip"><div id="ta"></div><div id="tt"></div><button id="tr" aria-label="Translate"><svg viewBox="0 0 24 24"><text x="1" y="12" font-size="12" fill="#fff">文</text><text x="11" y="22" font-size="13" font-weight="700" fill="#fff">A</text></svg></button></div>

<div id="mouse">
<div class="padwrap">
<button id="fsb" aria-label="Fullscreen"><svg viewBox="0 0 24 24"><path d="M6 5h13v13"/></svg></button>
<div id="pad" class="tp"><div class="padtxt" id="ph"><b>Touchpad</b>
<small>tap = click &middot; hold = drag<br>2 fingers = scroll / pinch zoom<br>3 fingers = gestures</small></div></div>
</div>

<div class="lower">
<div class="lcol">
<div class="seg3">
<button id="bl">Left</button>
<button id="bm">Middle</button>
<button id="br">Right</button>
</div>

<div class="seg3 nav" style="margin-top:8px">
<button id="nr"><svg viewBox="0 0 24 24"><path d="M7 5v14M12 5v14M17 5v14"/></svg></button>
<button id="nh"><svg viewBox="0 0 24 24"><rect x="5" y="5" width="14" height="14" rx="5"/></svg></button>
<button id="nb"><svg viewBox="0 0 24 24"><path d="M15 5l-7 7 7 7"/></svg></button>
</div>
<div class="nav" style="margin-top:8px;display:flex;justify-content:flex-start"><button id="np" style="width:33%;padding:12px 0"><svg viewBox="0 0 24 24" style="fill:#fff;stroke:none"><circle cx="6" cy="6" r="2"/><circle cx="12" cy="6" r="2"/><circle cx="18" cy="6" r="2"/><circle cx="6" cy="12" r="2"/><circle cx="12" cy="12" r="2"/><circle cx="18" cy="12" r="2"/><circle cx="6" cy="18" r="2"/><circle cx="12" cy="18" r="2"/><circle cx="18" cy="18" r="2"/></svg></button></div>
</div>
<div id="sc" class="tp"></div>
</div>

<div class="card">
<button id="moreh" class="sech closed"><span>Tools</span><span id="sdots"><i id="dwf"></i><i id="dbt"></i></span><svg viewBox="0 0 24 24"><path d="M6 15l6-6 6 6"/></svg></button>
<div class="hide" id="morebody">

<div class="gt">SCROLL</div>
<div class="gb">
<div class="tw"><button class="tgl" id="rev"><span><svg class="ic" viewBox="0 0 24 24"><path d="M8 4v16M8 4L5 7M8 4l3 3M16 20V4M16 20l-3-3M16 20l3-3"/></svg>Reverse scroll</span><i></i></button><b class="hi" data-t="rev">!</b></div>
<div class="tw"><button id="fbb"><svg class="ic" viewBox="0 0 24 24"><path d="M4 7h10M18 7h2M4 17h2M10 17h10"/><circle cx="16" cy="7" r="2"/><circle cx="8" cy="17" r="2"/></svg>Scroll settings</button><b class="hi" data-t="sset">!</b></div>
<div class="card pn" id="fb">
<div class="ptitle">SCROLL SETTINGS</div>
<div id="trio">
<div class="fr"><button class="tgl" id="vib"><span>Vibrate</span><i></i></button><b class="hi" data-t="vib">!</b></div>
<div class="fr"><button class="tgl" id="snd"><span>Sound</span><i></i></button><b class="hi" data-t="snd">!</b></div>
<div class="fr"><button class="tgl" id="fl"><span>Inertia</span><i></i></button><b class="hi" data-t="fl">!</b></div>
</div>
<div class="sl"><div class="sh"><span>Scroll</span><b id="vss"></b><b class="hi" data-t="ss">!</b></div><input type="range" id="ss" min="0.05" max="0.4" step="0.01" value="0.125"></div>
<div class="sl"><div class="sh"><span>Boost</span><b id="vgn"></b><b class="hi" data-t="gn">!</b></div><input type="range" id="gn" min="0" max="4" step="0.1" value="1.5"></div>
<div class="sl"><div class="sh"><span>Max</span><b id="vgm"></b><b class="hi" data-t="gm">!</b></div><input type="range" id="gm" min="0" max="8" step="0.5" value="3"></div>
<div class="sl" id="rg"><div class="sh"><span>Glide</span><b id="vfr"></b><b class="hi" data-t="fr">!</b></div><input type="range" id="fr" min="0.85" max="0.99" step="0.01" value="0.95"></div>
<div class="sl" id="rv"><div class="sh"><span>Volume</span><b id="vvl"></b><b class="hi" data-t="vl">!</b></div><input type="range" id="vl" min="0.01" max="0.3" step="0.01" value="0.08"></div>
</div>

<div class="gt">MOUSE</div>
<div class="gb">
<div class="tw"><button id="tpb"><svg class="ic" viewBox="0 0 24 24"><rect x="3" y="5" width="18" height="14" rx="3"/><path d="M3 14h18M12 14v5"/></svg>Touchpad settings</button><b class="hi" data-t="tset">!</b></div>
<div class="card pn" id="tps">
<div class="ptitle">TOUCHPAD SETTINGS</div>
<div class="sl"><div class="sh"><span>Speed</span><b id="vsp"></b><b class="hi" data-t="sp">!</b></div><input type="range" id="sp" min="0.5" max="4" step="0.1" value="1.6"></div>
<div class="sl"><div class="sh"><span>Accel</span><b id="vac"></b><b class="hi" data-t="ac">!</b></div><input type="range" id="ac" min="0" max="1" step="0.05" value="0.3"></div>
</div>
<div class="tw"><button class="tgl" id="jig"><span><svg class="ic" viewBox="0 0 24 24"><path d="M3 12h3l2-5 4 10 2-5h7"/></svg>Jiggler</span><i></i></button><b class="hi" data-t="jig">!</b></div>
</div>

<div class="gt">MORE</div>
<div class="gb">
<div class="tw"><button id="mdb"><svg class="ic" viewBox="0 0 24 24"><circle cx="12" cy="12" r="9"/><path d="M10 8.5v7l6-3.5z"/></svg>Media</button><b class="hi" data-t="med">!</b></div>
<div id="md" class="pn">
<div class="ptitle">MEDIA</div>
<div class="card g5">
<button data-u="182"><svg viewBox="0 0 24 24"><path d="M6 5h2v14H6zM20 5v14L9 12z"/></svg>Prev</button>
<button data-u="1000"><svg viewBox="0 0 24 24"><path d="M11 5v14L2 12zM22 5v14l-9-7z"/></svg>Back</button>
<button data-u="205"><svg viewBox="0 0 24 24"><path d="M4 5v14l9-7zM15 5h4v14h-4z"/></svg>Play/Pause</button>
<button data-u="1001"><svg viewBox="0 0 24 24"><path d="M13 5v14l9-7zM2 5v14l9-7z"/></svg>Fwd</button>
<button data-u="181"><svg viewBox="0 0 24 24"><path d="M16 5h2v14h-2zM4 5v14l11-7z"/></svg>Next</button>
</div>
<div class="card g2">
<button data-u="234"><svg viewBox="0 0 24 24"><path d="M3 9v6h4l5 4V5L7 9z"/><path d="M15 12h6" stroke="#fff" stroke-width="2" stroke-linecap="round"/></svg>Vol -</button>
<button data-u="233"><svg viewBox="0 0 24 24"><path d="M3 9v6h4l5 4V5L7 9z"/><path d="M15 12h6M18 9v6" stroke="#fff" stroke-width="2" stroke-linecap="round"/></svg>Vol +</button>
</div>
</div>
<div class="tw"><button id="stbtn"><svg class="ic" viewBox="0 0 24 24"><path d="M5 12.5a10 10 0 0 1 14 0M8.5 16a5 5 0 0 1 7 0M12 19.5h.01"/></svg>Status</button><b class="hi" data-t="sta">!</b></div>
</div>

</div>
</div>

</div>

</div>

<script>
var ws=null,wsOk=false,wsFail=0;
function wsConnect(){
try{ws=new WebSocket('ws://'+location.hostname+':81/');}catch(e){return;}
ws.onopen=function(){wsOk=true;wsFail=0;};
ws.onclose=function(){wsOk=false;wsFail++;setTimeout(wsConnect,Math.min(1000*(wsFail+1),8000));};
ws.onerror=function(){try{ws.close();}catch(e){}};
}
wsConnect();
function send(u){if(wsOk){ws.send(u);}else{fetch(u).catch(function(){});}}
var $=function(i){return document.getElementById(i)};

/* ---- settings (saved in browser) ---- */
var S={sens:1.6,acc:0.3,ssp:0.125,rev:false,vib:true,snd:true,jig:false,fl:true,gn:1.5,gm:3,fr:0.95,vol:0.08,more:false};
try{var o=JSON.parse(localStorage.getItem('epad')||'{}');for(var q in o)S[q]=o[q];}catch(e){}
S.more=false;
var svT=null;
function save(){
try{localStorage.setItem('epad',JSON.stringify(S));}catch(e){}
clearTimeout(svT);
svT=setTimeout(function(){fetch('/cfgs?v='+encodeURIComponent(JSON.stringify(S))).catch(function(){});},800);
}
function buzz(){if(S.vib&&navigator.vibrate)navigator.vibrate(12);}
function sgn(){return S.rev?-1:1}
function cl(b){
send('/c?b='+b);buzz();
}
function lab(id,on){$(id).classList.toggle('on',on);}
function fillAll(){
var r=document.querySelectorAll('input[type=range]');
for(var i=0;i<r.length;i++){var e=r[i];e.style.setProperty('--r',(e.value-e.min)/(e.max-e.min));}
}
function vals(){
$('vsp').textContent=S.sens.toFixed(1);
$('vac').textContent=S.acc.toFixed(2);
$('vss').textContent=S.ssp.toFixed(2);
$('vgn').textContent=S.gn.toFixed(1);
$('vgm').textContent=S.gm.toFixed(1);
$('vfr').textContent=S.fr.toFixed(2);
$('vvl').textContent=S.vol.toFixed(2);
fillAll();
}
function ui(){
$('sp').value=S.sens;$('ac').value=S.acc;$('ss').value=S.ssp;
$('gn').value=S.gn;$('gm').value=S.gm;$('fr').value=S.fr;$('vl').value=S.vol;
vals();
lab('rev',S.rev);lab('vib',S.vib);lab('snd',S.snd);lab('jig',S.jig);lab('fl',S.fl);
$('rv').style.display=S.snd?'flex':'none';
$('rg').style.display=S.fl?'flex':'none';
$('morebody').classList.toggle('hide',!S.more);
$('moreh').classList.toggle('closed',!S.more);
}

/* ---- mouse ---- */
var pad=$('pad'),lx=0,ly=0,dx=0,dy=0,sv=0,hv=0,t0=0,moved=0,maxF=0;
var lp=null,lpFired=false,dragging=false,lastTap=0,held=false;
var gmode='',zacc=0,pd=0,pmy=0,mvz=0,mvs=0,g3x0=0,g3x=0,g3t=0;
function dist2(e){var a=e.touches[0],b=e.touches[1];return Math.hypot(a.clientX-b.clientX,a.clientY-b.clientY);}
function midY(e){return (e.touches[0].clientY+e.touches[1].clientY)/2;}
function avgX(e){var s=0;for(var i=0;i<e.touches.length;i++)s+=e.touches[i].clientX;return s/e.touches.length;}

pad.addEventListener('touchstart',function(e){
e.preventDefault();
pad.classList.add('act');$('ph').classList.add('gone');
var n=e.touches.length;
if(n==1){
maxF=1;moved=0;t0=Date.now();lpFired=false;gmode='';
lx=e.touches[0].clientX;ly=e.touches[0].clientY;
clearTimeout(lp);
if(Date.now()-lastTap<280){dragging=true;send('/d?s=1');pad.classList.add('drag');}
else{lp=setTimeout(function(){
if(moved<10&&maxF==1&&!dragging){dragging=true;send('/d?s=1');pad.classList.add('drag');lpFired=true;buzz();}
},550);}
}else{
maxF=Math.max(maxF,n);clearTimeout(lp);
if(n==2){pd=dist2(e);pmy=midY(e);gmode='';zacc=0;mvz=0;mvs=0;}
if(n>=3){g3x0=avgX(e);g3x=g3x0;g3t=Date.now();}
}
},{passive:false});

pad.addEventListener('touchmove',function(e){
e.preventDefault();
var n=e.touches.length;
if(n>=3){g3x=avgX(e);return;}
if(n==2){
var d=dist2(e),my=midY(e);
var dd=d-pd,dm=pmy-my;
pd=d;pmy=my;
mvz+=Math.abs(dd);mvs+=Math.abs(dm);
moved+=Math.abs(dd)+Math.abs(dm);
if(gmode==''&&(mvz+mvs)>12){gmode=(mvz>mvs*1.2)?'z':'s';}
if(gmode=='z'){
zacc+=dd;
if(Math.abs(zacc)>=25){send('/w?v='+(zacc>0?1:-1)+'&m=1');zacc=0;}
}else if(gmode=='s'){
sv+=dm*S.ssp*sgn();
}
return;
}
if(maxF>1)return;
var x=e.touches[0].clientX,y=e.touches[0].clientY;
var ddx=x-lx,ddy=y-ly;
var dist=Math.hypot(ddx,ddy);
var f=S.sens*(1+S.acc*Math.min(dist,40)/10);
dx+=ddx*f;dy+=ddy*f;
moved+=Math.abs(ddx)+Math.abs(ddy);
if(moved>=10)clearTimeout(lp);
lx=x;ly=y;
},{passive:false});

function padEnd(e){
e.preventDefault();
if(e.touches.length>0)return;
pad.classList.remove('act');
pad.classList.remove('drag');
clearTimeout(lp);
var dur=Date.now()-t0;
if(dragging){dragging=false;send('/d?s='+(held?1:0));lastTap=0;maxF=0;return;}
if(maxF>=3){
var d3=g3x-g3x0;
if(Math.abs(d3)>60){send('/h?c=43&m='+(d3>0?4:6));buzz();}
else if(Date.now()-g3t<300){cl('m');}
maxF=0;return;
}
if(lpFired){lpFired=false;maxF=0;return;}
if(moved<10&&dur<250){
if(maxF>=2){cl('r');}
else{cl('l');lastTap=Date.now();}
}
maxF=0;
}
pad.addEventListener('touchend',padEnd,{passive:false});
pad.addEventListener('touchcancel',function(e){
pad.classList.remove('act');
pad.classList.remove('drag');
clearTimeout(lp);
if(dragging){dragging=false;send('/d?s='+(held?1:0));}
maxF=0;
},{passive:false});

/* ---- دو نوار اسکرول: هر دو در هر چهار جهت اسکرول می‌کنند ---- */
var actx=null;
function tick(){
buzz();
if(!S.snd)return;
try{
if(!actx)actx=new (window.AudioContext||window.webkitAudioContext)();
var o=actx.createOscillator(),g=actx.createGain();
o.type='square';
o.frequency.value=900;
g.gain.setValueAtTime(S.vol,actx.currentTime);
g.gain.exponentialRampToValueAtTime(0.001,actx.currentTime+0.015);
o.connect(g);g.connect(actx.destination);
o.start();o.stop(actx.currentTime+0.02);
}catch(e){}
}
function strip(el,h){
var px=0,py=0,off=0,lastN=0,vel=0,lt=0,raf=0,W=0,H=0,cv=document.createElement('canvas');
el.insertBefore(cv,el.firstChild);
var cx=cv.getContext('2d');
function draw(){
var r=el.getBoundingClientRect(),d=window.devicePixelRatio||1;
if(W!=r.width||H!=r.height){W=r.width;H=r.height;cv.width=W*d;cv.height=H*d;}
cx.setTransform(d,0,0,d,0,0);
cx.clearRect(0,0,W,H);
var L=h?W:H,C=L/2,R=L/2-4;
var g=h?cx.createLinearGradient(0,0,W,0):cx.createLinearGradient(0,0,0,H);
g.addColorStop(0,'rgba(0,0,0,.65)');
g.addColorStop(0.5,'rgba(255,255,255,.07)');
g.addColorStop(1,'rgba(0,0,0,.65)');
cx.fillStyle=g;cx.fillRect(0,0,W,H);
var rot=off/1.2,n=Math.floor(rot/10),base=rot-n*10;
for(var k=-10;k<=10;k++){
var a=(k*10+base)*Math.PI/180;
if(Math.abs(a)>=Math.PI/2)continue;
var c=Math.cos(a),p=C+R*Math.sin(a);
var major=(((k-n)%6)+6)%6==0;
var len=(h?H:W)*(major?0.45:0.28)*c;
cx.strokeStyle='rgba(236,233,226,'+(c*(major?0.95:0.6))+')';
cx.lineWidth=(major?2.5:1.5)*c+0.5;
cx.beginPath();
if(h){cx.moveTo(p,H/2-len/2);cx.lineTo(p,H/2+len/2);}
else{cx.moveTo(W/2-len/2,p);cx.lineTo(W/2+len/2,p);}
cx.stroke();
if(major){
var fd=Math.min(1,Math.abs(p-C)/36);fd=fd*fd*(3-2*fd);
cx.fillStyle='rgba(236,233,226,'+(c*0.85*fd)+')';
cx.font=(8+3*c)+'px system-ui';
cx.textBaseline='middle';
if(h){cx.textAlign='center';cx.fillText(k-n,p,7);}
else{cx.textAlign='left';cx.fillText(k-n,3,p);}
}
}
var nn=Math.floor(off/12);
if(nn!=lastN){lastN=nn;tick();}
}
function snap(){
var t=Math.round(off/12)*12;
(function step(){
off+=(t-off)*0.35;
draw();
if(Math.abs(t-off)>0.2)requestAnimationFrame(step);
else{off=t;draw();}
})();
}
function fling(){
vel*=S.fr;
if(Math.abs(vel)<0.05){snap();return;}
var raw=vel*16;
var amt=raw*S.ssp*sgn();
if(h)hv+=amt;else sv+=amt;
off-=raw;
draw();
raf=requestAnimationFrame(fling);
}
el.addEventListener('touchstart',function(e){
e.preventDefault();
cancelAnimationFrame(raf);
vel=0;lt=Date.now();
px=e.touches[0].clientX;py=e.touches[0].clientY;
if(!actx){try{actx=new (window.AudioContext||window.webkitAudioContext)();}catch(er){}}
},{passive:false});
el.addEventListener('touchmove',function(e){
e.preventDefault();
var x=e.touches[0].clientX,y=e.touches[0].clientY;
var t=Date.now(),dt=Math.max(t-lt,1);
lt=t;
var raw=h?(px-x):(py-y);
var gain=1+Math.min(Math.abs(raw)/dt*S.gn,S.gm);
var amt=raw*S.ssp*sgn()*gain;
if(h)hv+=amt;else sv+=amt;
off-=raw;
vel=raw/dt;
px=x;py=y;
draw();
},{passive:false});
el.addEventListener('touchend',function(e){
e.preventDefault();
if(Date.now()-lt>80)vel=0;
if(S.fl&&Math.abs(vel)>0.3)raf=requestAnimationFrame(fling);
else snap();
},{passive:false});
el.addEventListener('touchcancel',function(e){snap();},{passive:false});
draw();
}
strip($('sc'),false);

var busy=0;
function sendB(u){
if(wsOk){ws.send(u);return;}
busy=1;fetch(u).then(function(){busy=0;},function(){busy=0;});
}
setInterval(function(){
if(busy)return;
var x=Math.round(dx),y=Math.round(dy),s=Math.round(sv);
if(x||y){sendB('/m?x='+x+'&y='+y);dx-=x;dy-=y;}
else if(s){sendB('/s?v='+s);sv-=s;}
},20);

$('bl').onclick=function(){cl('l')};
$('bm').onclick=function(){cl('m')};
$('br').onclick=function(){cl('r')};
$('fsb').onclick=function(){
var d=document,el=d.documentElement;
if(d.fullscreenElement||d.webkitFullscreenElement){
(d.exitFullscreen||d.webkitExitFullscreen).call(d);
}else{
var f=el.requestFullscreen||el.webkitRequestFullscreen;
if(f)f.call(el);
}
};
var PANELS=[['mdb','md'],['tpb','tps'],['fbb','fb']];
function panel(btn,id){
var f=$(id),open=!f.classList.contains('open');
for(var i=0;i<PANELS.length;i++){
$(PANELS[i][1]).classList.remove('open');
$(PANELS[i][0]).classList.remove('sel');
}
if(open){
f.classList.add('open');
$(btn).classList.add('sel');
setTimeout(function(){f.scrollIntoView({behavior:'smooth',block:'nearest'});},200);
}
}
$('mdb').onclick=function(){panel('mdb','md');};
$('tpb').onclick=function(){panel('tpb','tps');};
$('fbb').onclick=function(){panel('fbb','fb');};
var mk=document.querySelectorAll('#md button');
for(var i=0;i<mk.length;i++){mk[i].onclick=function(){send('/md?u='+this.getAttribute('data-u'));buzz();};}
$('nr').onclick=function(){send('/n?k=r');buzz();};
$('nb').addEventListener('touchstart',function(){send('/n?k=x');});
var hT=null,hLong=false;
$('nh').addEventListener('touchstart',function(){
hLong=false;
hT=setTimeout(function(){hLong=true;send('/n?k=a');buzz();},500);
});
$('nh').addEventListener('touchend',function(e){
clearTimeout(hT);
e.preventDefault();
if(!hLong){send('/n?k=h');buzz();}
});
$('nh').addEventListener('touchcancel',function(){clearTimeout(hT);});
$('nb').onclick=function(){send('/n?k=b');buzz();};
$('np').onclick=function(){send('/n?k=p');buzz();};
$('rev').onclick=function(){S.rev=!S.rev;save();ui();};
$('moreh').onclick=function(){S.more=!S.more;save();ui();};
$('vib').onclick=function(){S.vib=!S.vib;save();ui();buzz();};
$('snd').onclick=function(){S.snd=!S.snd;save();ui();tick();};
var TIPS={
vib:'Gives a short haptic pulse on your phone every time the wheel passes a tick mark while scrolling. Also used for button taps and gesture feedback. Turn off to disable all vibration.',
snd:'Plays a short mechanical click sound for every tick mark the wheel passes, like a real mouse wheel. Works after your first touch on the page because browsers block sound before that.',
fl:'When on, the wheel keeps spinning after you lift your finger, slowing down gradually. A fast flick scrolls far, a slow one stops quickly. When off, scrolling stops the moment you release.',
ss:'Base scroll speed. Controls how many scroll steps are sent to the computer for each pixel your finger moves. Higher means the page scrolls farther with the same finger movement.',
gn:'Speed-based acceleration strength. The faster your finger moves, the more extra scrolling is added on top of the base speed. At 0 there is no acceleration and scrolling is perfectly linear. Higher values make fast swipes cover much more distance.',
gm:'Acceleration limit. Caps how much the Boost can multiply your scrolling, so very fast swipes do not jump too far. Lower values keep scrolling controlled, higher values allow huge jumps on fast swipes.',
fr:'Inertia length. Controls how slowly the wheel loses speed after release. Values near 0.99 glide for a long time, values near 0.85 stop quickly. Only has an effect when Inertia is on.',
vl:'Loudness of the tick sound. Plays a sample click when you change it so you can hear the level. Only has an effect when Sound is on.'
};
TIPS.rev='Flips the scroll direction. Turn it on if the page moves the opposite way to your finger.';
TIPS.sset='Fine-tune the scroll wheel: haptics, sound, inertia, speed and acceleration. The panel opens right below this option.';
TIPS.tset='Cursor movement settings for the touchpad: speed and acceleration. The panel opens right below this option.';
TIPS.jig='Nudges the mouse by 1 pixel every 30 seconds and back, so the computer thinks you are active and does not sleep or lock. Works while Bluetooth is connected.';
TIPS.med='Media remote: previous, next, play/pause, rewind, forward and volume for the connected computer or TV.';
TIPS.sta='Opens the status page: WiFi and Bluetooth state, addresses, uptime and free memory. The two dots next to Tools show the same thing: first WiFi, second Bluetooth. Green means connected, red means disconnected.';
TIPS.sp='How far the cursor moves for each bit of finger movement on the touchpad. Higher means the cursor travels farther.';
TIPS.ac='Cursor acceleration. At 0 the cursor always moves at a fixed ratio. Higher values make fast swipes fly farther while slow movements stay precise.';
var TIPS_FA={
vib:'هر بار که چرخ اسکرول از یه خط نشانه رد می‌شه، گوشیت یه لرزش کوچیک می‌ده. برای لمس دکمه‌ها و حرکت‌های چندانگشتی هم همین لرزش هست. خاموشش کنی دیگه هیچ لرزشی نداری.',
snd:'با هر خطی که چرخ رد می‌کنه یه صدای «تیک» کوتاه پخش می‌شه، درست مثل چرخ ماوس واقعی. چون مرورگر قبل از اولین لمس اجازه‌ی صدا نمی‌ده، یه بار صفحه رو لمس کن تا کار کنه.',
fl:'روشن باشه، بعد از اینکه انگشتت رو برمی‌داری چرخ یه مدت می‌چرخه و کم‌کم آروم می‌گیره. تند بکشی خیلی می‌ره جلو، آروم بکشی زود می‌ایسته. خاموش باشه، به محض رها کردن اسکرول می‌ایسته.',
ss:'سرعت اصلی اسکرول. مشخص می‌کنه با هر کم‌وزیاد حرکت انگشتت، صفحه‌ی کامپیوتر چقدر بالا یا پایین بره. عدد بیشتر یعنی با همون حرکت انگشت، صفحه بیشتر جابه‌جا می‌شه.',
gn:'شتاب بر اساس سرعت انگشتت. هرچی تندتر بکشی، اسکرول اضافه‌تری روی سرعت اصلی اضافه می‌شه. روی صفر باشه شتابی نیست و همه‌چیز یکنواخته. عدد بالا باعث می‌شه کشیدن‌های تند صفحه رو خیلی دورتر ببرن.',
gm:'سقف شتاب. مشخص می‌کنه Boost حداکثر چند برابر اسکرول رو زیاد کنه تا کشیدن‌های خیلی تند صفحه رو بیش از حد نپرونن. عدد کمتر یعنی کنترل بیشتر، عدد بیشتر یعنی پرش‌های بزرگ‌تر.',
fr:'مقدار لغزیدن چرخ بعد از رها کردن. عددهای نزدیک ۰٫۹۹ یعنی خیلی طولانی می‌لغزه، نزدیک ۰٫۸۵ یعنی زود می‌ایسته. فقط وقتی Inertia روشنه اثر داره.',
vl:'بلندی صدای تیک. وقتی تغییرش می‌دی یه تیک نمونه پخش می‌شه تا بشنوی. فقط وقتی Sound روشنه اثر داره.',
rev:'جهت اسکرول رو برعکس می‌کنه. اگه حس می‌کنی صفحه خلاف جهت انگشتت می‌ره، این رو روشن کن.',
sset:'تنظیمات دقیق نوار اسکرول: لرزش، صدا، اینرسی، سرعت و شتاب. با لمس این گزینه پنلش همین‌جا زیرش باز می‌شه و با لمس دوباره بسته می‌شه.',
tset:'تنظیمات حرکت مکان‌نما روی لمس‌پد، یعنی سرعت و شتاب. پنلش زیر همین گزینه باز می‌شه.',
jig:'Jiggler هر ۳۰ ثانیه ماوس رو یه پیکسل تکون می‌ده و برمی‌گردونه تا کامپیوتر فکر کنه داری ازش استفاده می‌کنی و خوابش نبره یا صفحه‌ش قفل نشه. وقتی بلوتوث وصل باشه کار می‌کنه.',
med:'دکمه‌های کنترل پخش: قبلی، بعدی، پخش و توقف، عقب و جلو بردن و کم و زیاد کردن صدا، برای کامپیوتر یا تلویزیونی که بهش وصلی.',
sta:'صفحه‌ی وضعیت رو باز می‌کنه: وضعیت وای‌فای و بلوتوث، آدرس‌ها، مدت روشن بودن برد و حافظه‌ی آزاد. دو نقطه‌ی کنار اسم Tools هم همین وضعیته: اولی وای‌فای و دومی بلوتوث. سبز یعنی وصل، قرمز یعنی قطع.',
sp:'سرعت حرکت مکان‌نما نسبت به حرکت انگشتت روی لمس‌پد. عدد بیشتر یعنی با یه حرکت کوچیک، مکان‌نما بیشتر می‌ره.',
ac:'شتاب مکان‌نما. روی صفر باشه مکان‌نما همیشه با یه نسبت ثابت می‌ره. هرچی بالاتر ببری، کشیدن تند مکان‌نما رو خیلی بیشتر پرت می‌کنه و کشیدن آروم دقیق می‌مونه.'
};
var curTip='',tipFa=false,curEl=null;
function showTip(){
var t=$('tip'),tt=$('tt'),a=$('ta');
tt.textContent=tipFa?TIPS_FA[curTip]:TIPS[curTip];
tt.className=tipFa?'fa':'';
t.style.display='block';
var r=curEl.getBoundingClientRect(),sy=window.pageYOffset;
var pb=document.querySelector('.padwrap').getBoundingClientRect().bottom;
var h=t.offsetHeight;
var above=(r.top-h-14)>pb;
t.className=above?'up':'dn';
t.style.top=(above?r.top+sy-h-12:r.bottom+sy+12)+'px';
var ax=r.left+r.width/2-t.getBoundingClientRect().left-7;
a.style.left=Math.max(10,Math.min(ax,t.offsetWidth-24))+'px';
}
$('tr').onclick=function(e){e.stopPropagation();tipFa=!tipFa;showTip();};
var hi=document.querySelectorAll('.hi');
for(var i=0;i<hi.length;i++){hi[i].onclick=function(e){
e.stopPropagation();
curTip=this.getAttribute('data-t');curEl=this;tipFa=false;showTip();
};}
document.addEventListener('touchstart',function(e){
var t=$('tip');
if(t.style.display=='block'&&!e.target.classList.contains('hi')&&!t.contains(e.target))t.style.display='none';
});
$('fl').onclick=function(){S.fl=!S.fl;save();ui();};
$('gn').oninput=function(){S.gn=parseFloat(this.value);save();vals();};
$('gm').oninput=function(){S.gm=parseFloat(this.value);save();vals();};
$('fr').oninput=function(){S.fr=parseFloat(this.value);save();vals();};
$('vl').oninput=function(){S.vol=parseFloat(this.value);save();vals();tick();};
$('jig').onclick=function(){S.jig=!S.jig;save();ui();send('/j?s='+(S.jig?1:0));};
$('sp').oninput=function(){S.sens=parseFloat(this.value);save();vals();};
$('ac').oninput=function(){S.acc=parseFloat(this.value);save();vals();};
$('ss').oninput=function(){S.ssp=parseFloat(this.value);save();vals();};
$('stbtn').onclick=function(){location.href='/status';};
function dots(){
var t=setTimeout(function(){$('dwf').classList.remove('ok');$('dbt').classList.remove('ok');},2500);
fetch('/st').then(function(r){return r.json();}).then(function(d){
clearTimeout(t);
$('dwf').classList.add('ok');
$('dbt').classList.toggle('ok',d.bt==1);
}).catch(function(){
clearTimeout(t);
$('dwf').classList.remove('ok');$('dbt').classList.remove('ok');
});
}
dots();setInterval(dots,4000);
ui();
if(S.jig)send('/j?s=1');
fetch('/cfg').then(function(r){return r.json();}).then(function(d){
var has=false,mo=S.more;
for(var q in d){S[q]=d[q];has=true;}
S.more=mo;
if(!has)save();
ui();
send('/j?s='+(S.jig?1:0));
}).catch(function(){});
</script></body></html>
)rawliteral";

int clampv(int v) {
  if (v > 127) return 127;
  if (v < -127) return -127;
  return v;
}

// چرخ ماوس همراه با مادیفایر: Ctrl = زوم، Shift = اسکرول افقی
void wheelMod(uint8_t mod, int v) {
  if (!isConnected()) return;
  uint8_t r[8] = { mod, 0, 0, 0, 0, 0, 0, 0 };
  kbIn->setValue(r, 8);
  kbIn->notify();
  delay(8);
  sendMouse(heldBtn, 0, 0, clampv(v));
  delay(8);
  uint8_t z[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
  kbIn->setValue(z, 8);
  kbIn->notify();
  delay(8);
}

void clickBtn(uint8_t b) {
  sendMouse(b | heldBtn, 0, 0, 0);
  delay(60);
  sendMouse(heldBtn, 0, 0, 0);
  delay(20);
}

void handleRoot() { server.send_P(200, "text/html", PAGE); }

void handleLog() {
  String out = "<meta charset='utf-8'><meta http-equiv='refresh' content='2'>";
  out += "<body style='background:#111;color:#eee;font-family:monospace;font-size:14px'><pre>";
  out += "MOUSE v1  MEDIA=" + String(WITH_MEDIA) + " DIS=" + String(WITH_DIS) +
         " KEYDIST=" + String(KEYDIST) + " SECREQ=" + String(SEC_REQ) +
         " ENCMAP=" + String(ENC_MAP) + " NIMBLE2=" + String(NIMBLE2) + " WS=" + String(WITH_WS) + "\n\n";
  if (logHead == 0) out += "(no events yet)\n";
  int start = (logHead > LOGN) ? (logHead - LOGN) : 0;
  for (int k = logHead - 1; k >= start; k--) {
    out += logs[k % LOGN];
    out += "\n";
  }
  out += "</pre>";
  server.send(200, "text/html", out);
}

void handleMove() {
  int x = server.arg("x").toInt();
  int y = server.arg("y").toInt();
  while (x != 0 || y != 0) {
    int sx = clampv(x), sy = clampv(y);
    sendMouse(heldBtn, sx, sy, 0);
    x -= sx;
    y -= sy;
  }
  server.send(200, "text/plain", "ok");
}

void handleScroll() {
  sendMouse(heldBtn, 0, 0, clampv(server.arg("v").toInt()));
  server.send(200, "text/plain", "ok");
}

void handleWheelMod() {
  int v = server.arg("v").toInt();
  int m = server.arg("m").toInt();
  wheelMod((uint8_t)m, v);
  server.send(200, "text/plain", "ok");
}

void handleClick() {
  String b = server.arg("b");
  if (b == "r") clickBtn(2);
  else if (b == "m") clickBtn(4);
  else if (b == "d") { clickBtn(1); delay(40); clickBtn(1); }
  else clickBtn(1);
  server.send(200, "text/plain", "ok");
}

void handleDrag() {
  heldBtn = (server.arg("s") == "1") ? 1 : 0;
  sendMouse(heldBtn, 0, 0, 0);
  server.send(200, "text/plain", "ok");
}

void sendConsumer(uint16_t u) {
  if (!isConnected()) return;
  uint8_t r[2] = { (uint8_t)(u & 0xFF), (uint8_t)(u >> 8) };
  mdIn->setValue(r, 2);
  mdIn->notify();
  delay(25);
  uint8_t z[2] = { 0, 0 };
  mdIn->setValue(z, 2);
  mdIn->notify();
  delay(25);
}

void sendConsumerHold(uint16_t u, int ms) {
  if (!isConnected()) return;
  uint8_t r[2] = { (uint8_t)(u & 0xFF), (uint8_t)(u >> 8) };
  mdIn->setValue(r, 2);
  mdIn->notify();
  delay(ms);
  uint8_t z[2] = { 0, 0 };
  mdIn->setValue(z, 2);
  mdIn->notify();
  delay(30);
}

void altTab() {
  sendKey(4, 43);
}

void handleNav() {
  String k = server.arg("k");
  if (k == "h") sendConsumer(0x0223);
  else if (k == "b") sendConsumer(0x0224);
  else if (k == "a") sendKey(8, 0);
  else if (k == "r") altTab();
  else if (k == "p") sendConsumer(0x01A2);
  server.send(200, "text/plain", "ok");
}

void handleMedia() {
  int u = server.arg("u").toInt();
  if (u == 1000) sendKey(0, 0x50);
  else if (u == 1001) sendKey(0, 0x4F);
  else if (u == 1002) sendKey(0, 0x50 + 0);
  else if (u > 0) sendConsumer((uint16_t)u);
  server.send(200, "text/plain", "ok");
}

const char STPAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="theme-color" content="#000000">
<title>Status</title>
<style>
body{margin:0;background:#000;color:#ece9e2;font-family:system-ui,sans-serif;padding:18px}
a{color:#3e91ff;text-decoration:none;font-size:15px}
h3{margin:18px 4px 6px;font-size:12px;letter-spacing:1px;color:#3e91ff}
.c{background:rgba(255,255,255,.05);border:1px solid rgba(255,255,255,.1);border-radius:16px;padding:4px 12px}
.r{display:flex;justify-content:space-between;padding:11px 0;font-size:14px;border-bottom:1px solid rgba(255,255,255,.08)}
.r:last-child{border:0}
.r span{color:#9a968c}
.dot{display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:6px;background:#e5484d}
.dot.ok{background:#30d158}
</style></head><body>
<a href="/">&#8249; Back</a>
<h3>WIFI</h3><div class="c">
<div class="r"><span>Status</span><b><i class="dot" id="dw"></i><em id="sws" style="font-style:normal"></em></b></div>
<div class="r"><span>Network</span><b id="s1"></b></div>
<div class="r"><span>IP address</span><b id="s2"></b></div>
<div class="r"><span>Connected devices</span><b id="s3"></b></div>
<div class="r"><span>MAC</span><b id="s4"></b></div>
</div>
<h3>BLUETOOTH</h3><div class="c">
<div class="r"><span>Status</span><b><i class="dot" id="db"></i><em id="sbs" style="font-style:normal"></em></b></div>
<div class="r"><span>Device name</span><b id="s5"></b></div>
<div class="r"><span>Address</span><b id="s6"></b></div>
</div>
<h3>SYSTEM</h3><div class="c">
<div class="r"><span>Uptime</span><b id="s7"></b></div>
<div class="r"><span>Free memory</span><b id="s8"></b></div>
</div>
<script>
var $=function(i){return document.getElementById(i)};
function fmt(s){var h=Math.floor(s/3600),m=Math.floor(s%3600/60);return h+'h '+m+'m '+(s%60)+'s';}
function stat(){
var t=setTimeout(function(){$('dw').classList.remove('ok');$('sws').textContent='No response';},2500);
fetch('/st').then(function(r){return r.json();}).then(function(d){
clearTimeout(t);
$('dw').classList.add('ok');$('sws').textContent='Connected';
$('s1').textContent=d.ssid;$('s2').textContent=d.ip;$('s3').textContent=d.clients;$('s4').textContent=d.mac;
$('db').classList.toggle('ok',d.bt==1);$('sbs').textContent=d.bt==1?'Connected':'Not connected';
$('s5').textContent=d.btname;$('s6').textContent=d.btmac;
$('s7').textContent=fmt(d.up);$('s8').textContent=Math.round(d.heap/1024)+' KB';
}).catch(function(){clearTimeout(t);$('dw').classList.remove('ok');$('sws').textContent='No response';});
}
stat();setInterval(stat,3000);
</script></body></html>
)rawliteral";

void handleStatus() { server.send_P(200, "text/html", STPAGE); }

void handleStat() {
  String o = "{";
  o += "\"ssid\":\"" + String(AP_SSID) + "\",";
  o += "\"ip\":\"" + WiFi.softAPIP().toString() + "\",";
  o += "\"clients\":" + String(WiFi.softAPgetStationNum()) + ",";
  o += "\"mac\":\"" + WiFi.softAPmacAddress() + "\",";
  o += "\"bt\":" + String(isConnected() ? 1 : 0) + ",";
  o += "\"btname\":\"ESP32 Touchpad\",";
  o += "\"btmac\":\"" + String(NimBLEDevice::getAddress().toString().c_str()) + "\",";
  o += "\"up\":" + String(millis() / 1000UL) + ",";
  o += "\"heap\":" + String(ESP.getFreeHeap());
  o += "}";
  server.send(200, "application/json", o);
}

void handleJig() {
  jiggle = (server.arg("s") == "1");
  lastJig = millis();
  server.send(200, "text/plain", "ok");
}

void handleHid() {
  int c = server.arg("c").toInt();
  int m = server.arg("m").toInt();
  if (c > 0) sendKey((uint8_t)m, (uint8_t)c);
  server.send(200, "text/plain", "ok");
}

void setupBle() {
  NimBLEDevice::init("ESP32 Touchpad");
  NimBLEDevice::setSecurityAuth(true, false, true);

#if KEYDIST
  NimBLEDevice::setSecurityInitKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);
  NimBLEDevice::setSecurityRespKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);
#endif

#if CLEAR_BONDS_ON_BOOT
  NimBLEDevice::deleteAllBonds();
#endif

  bleServer = NimBLEDevice::createServer();
  bleServer->setCallbacks(new SrvCb());

#if WITH_DIS
  NimBLEService* disSvc = bleServer->createService("180A");
  NimBLECharacteristic* mfr = disSvc->createCharacteristic("2A29", NIMBLE_PROPERTY::READ);
  uint8_t mfrVal[9] = { 'E', 's', 'p', 'r', 'e', 's', 's', 'i', 'f' };
  mfr->setValue(mfrVal, 9);
  NimBLECharacteristic* pnp = disSvc->createCharacteristic("2A50", NIMBLE_PROPERTY::READ);
  uint8_t pnpVal[7] = { 0x02, 0xAC, 0x05, 0x0A, 0x82, 0x10, 0x02 };
  pnp->setValue(pnpVal, 7);
  disSvc->start();
#endif

  NimBLEService* hidSvc = bleServer->createService("1812");

  NimBLECharacteristic* info = hidSvc->createCharacteristic("2A4A", NIMBLE_PROPERTY::READ);
  uint8_t infoVal[4] = { 0x11, 0x01, 0x00, 0x01 };
  info->setValue(infoVal, 4);
  info->setCallbacks(&chrCb);

#if ENC_MAP
  NimBLECharacteristic* map = hidSvc->createCharacteristic(
      "2A4B", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::READ_ENC);
#else
  NimBLECharacteristic* map = hidSvc->createCharacteristic("2A4B", NIMBLE_PROPERTY::READ);
#endif
  map->setValue((uint8_t*)reportMap, sizeof(reportMap));
  map->setCallbacks(&chrCb);

  NimBLECharacteristic* ctrl = hidSvc->createCharacteristic("2A4C", NIMBLE_PROPERTY::WRITE_NR);
  ctrl->setCallbacks(&chrCb);

  NimBLECharacteristic* proto = hidSvc->createCharacteristic(
      "2A4E", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE_NR);
  uint8_t protoVal[1] = { 1 };
  proto->setValue(protoVal, 1);
  proto->setCallbacks(&chrCb);

  kbIn = hidSvc->createCharacteristic(
      "2A4D", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::READ_ENC | NIMBLE_PROPERTY::NOTIFY);
  NimBLEDescriptor* kd = kbIn->createDescriptor("2908", NIMBLE_PROPERTY::READ);
  uint8_t kref[2] = { 1, 1 };
  kd->setValue(kref, 2);
  kbIn->setCallbacks(&chrCb);

  msIn = hidSvc->createCharacteristic(
      "2A4D", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::READ_ENC | NIMBLE_PROPERTY::NOTIFY);
  NimBLEDescriptor* md = msIn->createDescriptor("2908", NIMBLE_PROPERTY::READ);
  uint8_t mref[2] = { 2, 1 };
  md->setValue(mref, 2);
  msIn->setCallbacks(&chrCb);

#if WITH_MEDIA
  mdIn = hidSvc->createCharacteristic(
      "2A4D", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::READ_ENC | NIMBLE_PROPERTY::NOTIFY);
  NimBLEDescriptor* cd = mdIn->createDescriptor("2908", NIMBLE_PROPERTY::READ);
  uint8_t cref[2] = { 3, 1 };
  cd->setValue(cref, 2);
  mdIn->setCallbacks(&chrCb);
#endif

  hidSvc->start();

  NimBLEService* batSvc = bleServer->createService("180F");
  NimBLECharacteristic* bat = batSvc->createCharacteristic(
      "2A19", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  uint8_t level[1] = { 100 };
  bat->setValue(level, 1);
  batSvc->start();

  bleServer->start();

  adv = NimBLEDevice::getAdvertising();
  adv->setName("ESP32 Touchpad");
  adv->setAppearance(0x03C1);
  adv->addServiceUUID(hidSvc->getUUID());
  adv->start();
}

const char UPPAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"></head>
<body style="background:#000;color:#ece9e2;font-family:sans-serif;padding:20px">
<h3>Firmware update</h3>
<form method="POST" action="/update" enctype="multipart/form-data">
<input type="file" name="fw" accept=".bin"><br><br>
<input type="submit" value="Upload">
</form></body></html>
)rawliteral";

void handleUpdateDone() {
  bool ok = !Update.hasError();
  server.send(200, "text/plain", ok ? "OK, restarting..." : "FAILED");
  delay(500);
  if (ok) ESP.restart();
}

void handleUpdateUpload() {
  HTTPUpload& u = server.upload();
  if (u.status == UPLOAD_FILE_START) {
    Update.begin(UPDATE_SIZE_UNKNOWN);
  } else if (u.status == UPLOAD_FILE_WRITE) {
    Update.write(u.buf, u.currentSize);
  } else if (u.status == UPLOAD_FILE_END) {
    Update.end(true);
  }
}

Preferences prefs;

void handleCfgGet() {
  prefs.begin("tp", true);
  String s = prefs.getString("cfg", "{}");
  prefs.end();
  server.send(200, "application/json", s);
}

void handleCfgSet() {
  String v = server.arg("v");
  if (v.length() > 0 && v.length() < 600) {
    prefs.begin("tp", false);
    if (prefs.getString("cfg", "") != v) prefs.putString("cfg", v);
    prefs.end();
  }
  server.send(200, "text/plain", "ok");
}

#if WITH_WS
String qa(const String& q, const char* k) {
  String key = String(k) + "=";
  int i = 0;
  while (i < (int)q.length()) {
    int e = q.indexOf('&', i);
    if (e < 0) e = q.length();
    if (q.startsWith(key, i)) return q.substring(i + key.length(), e);
    i = e + 1;
  }
  return "";
}

void wsCmd(const String& u) {
  int qi = u.indexOf('?');
  String p = (qi < 0) ? u : u.substring(0, qi);
  String q = (qi < 0) ? "" : u.substring(qi + 1);
  if (p == "/m") {
    int x = qa(q, "x").toInt(), y = qa(q, "y").toInt();
    while (x != 0 || y != 0) {
      int sx = clampv(x), sy = clampv(y);
      sendMouse(heldBtn, sx, sy, 0);
      x -= sx;
      y -= sy;
    }
  } else if (p == "/s") {
    sendMouse(heldBtn, 0, 0, clampv(qa(q, "v").toInt()));
  } else if (p == "/w") {
    wheelMod((uint8_t)qa(q, "m").toInt(), qa(q, "v").toInt());
  } else if (p == "/c") {
    String b = qa(q, "b");
    if (b == "r") clickBtn(2);
    else if (b == "m") clickBtn(4);
    else clickBtn(1);
  } else if (p == "/d") {
    heldBtn = (qa(q, "s") == "1") ? 1 : 0;
    sendMouse(heldBtn, 0, 0, 0);
  } else if (p == "/h") {
    int c = qa(q, "c").toInt(), m = qa(q, "m").toInt();
    if (c > 0) sendKey((uint8_t)m, (uint8_t)c);
  } else if (p == "/n") {
    String k = qa(q, "k");
    if (k == "h") sendConsumer(0x0223);
    else if (k == "b") sendConsumer(0x0224);
    else if (k == "a") sendKey(8, 0);
    else if (k == "r") altTab();
    else if (k == "p") sendConsumer(0x01A2);
  } else if (p == "/md") {
    int v = qa(q, "u").toInt();
    if (v == 1000) sendKey(0, 0x50);
    else if (v == 1001) sendKey(0, 0x4F);
    else if (v > 0) sendConsumer((uint16_t)v);
  } else if (p == "/j") {
    jiggle = (qa(q, "s") == "1");
    lastJig = millis();
  }
}

WebSocketsServer wsSrv(81);

void onWs(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_TEXT && length < 120) {
    String s;
    s.concat((const char*)payload, length);
    wsCmd(s);
  }
}
#endif

void setup() {
  pinMode(2, OUTPUT);
  blinkN(3);                 // ۳ چشمک: برد بوت شد

  setupBle();
  logEv("BOOT ok, advertising");
  blinkN(2);                 // ۲ چشمک: بلوتوث آماده شد

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS, 1, 0, 2);
  server.on("/", handleRoot);
  server.on("/log", handleLog);
  server.on("/m", handleMove);
  server.on("/s", handleScroll);
  server.on("/w", handleWheelMod);
  server.on("/c", handleClick);
  server.on("/d", handleDrag);
  server.on("/j", handleJig);
  server.on("/h", handleHid);
server.on("/n", handleNav);
server.on("/md", handleMedia);
server.on("/st", handleStat);
server.on("/status", handleStatus);
server.on("/update", HTTP_GET, []() { server.send_P(200, "text/html", UPPAGE); });
  server.on("/update", HTTP_POST, handleUpdateDone, handleUpdateUpload);
  ArduinoOTA.setPort(3232);
  ArduinoOTA.begin();
  server.on("/cfg", handleCfgGet);
  server.on("/cfgs", handleCfgSet);
#if WITH_WS
  wsSrv.begin();
  wsSrv.onEvent(onWs);
#endif
  server.begin();
  blinkN(1);                 // ۱ چشمک: وای‌فای آماده شد
}

void loop() {
  server.handleClient();
  ArduinoOTA.handle();
#if WITH_WS
  wsSrv.loop();
#endif

#if SEC_REQ
  if (secHandle != 0xFFFF && millis() > secAt) {
    uint16_t h = secHandle;
    secHandle = 0xFFFF;
    bool ok = NimBLEDevice::startSecurity(h);
    logEv(String("SECURITY REQUEST sent ok=") + (int)ok);
  }
#endif

  // ماوس‌جیگلر: هر ۳۰ ثانیه یک حرکت ۱ پیکسلی رفت‌وبرگشتی
  if (jiggle && isConnected() && millis() - lastJig > 30000) {
    lastJig = millis();
    sendMouse(heldBtn, 1, 0, 0);
    delay(30);
    sendMouse(heldBtn, -1, 0, 0);
  }

  if (isConnected()) {
    digitalWrite(2, HIGH);
  } else {
    digitalWrite(2, (millis() / 150) % 2);
    if (!adv->isAdvertising()) adv->start();
  }
}