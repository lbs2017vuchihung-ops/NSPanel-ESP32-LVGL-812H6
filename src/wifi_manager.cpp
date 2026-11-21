#include "wifi_manager.h"

#include <Preferences.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include "config.h"
#include "hmi.h"

namespace wifimgr {
namespace {

constexpr uint8_t kMaxKnown = storage::kMaxWifi + 2;
constexpr uint32_t kAttemptMs = 12000;   // one attempt per known network
constexpr uint32_t kRetryMinMs = 30000;  // nghỉ tối thiểu 30s giữa các lần thử khi mất mạng
constexpr uint32_t kRetryMaxMs = 60000;  // tối đa 60s để nhường radio cho ESP-NOW
constexpr uint32_t kJoinMs = 15000;      // user join timeout
constexpr uint8_t kMaxScan = 20;

struct Cred {
    char ssid[33];
    char pass[65];
};

Cred s_known[kMaxKnown];
uint8_t s_knownCount = 0;
uint8_t s_next = 0;           // next known network to try
uint32_t s_attemptAt = 0;     // when the current automatic attempt started
bool s_attempting = false;
uint32_t s_nextAttempt = 0;
uint32_t s_retryDelay = kRetryMinMs;
bool s_scanning = false;
bool s_joining = false;
uint32_t s_joinAt = 0;
Cred s_join;
volatile uint8_t s_lastReason = 0;  // WIFI_REASON_* of the last disconnect

char s_open[kMaxScan][33];
bool s_bootScan = false;
uint8_t s_openCount = 0;

// Kéo sóng radio về Kênh 1 và tắt chế độ ngủ để giữ ổn định ESP-NOW khi chưa có Wi-Fi
void lockEspNowChannel() {
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
}

bool isOpen(const char* ssid) {
    for (uint8_t i = 0; i < s_openCount; i++) {
        if (strcmp(s_open[i], ssid) == 0) return true;
    }
    return false;
}

void addKnown(const char* ssid, const char* pass) {
    if (!ssid || !ssid[0] || s_knownCount >= kMaxKnown) return;
    for (uint8_t i = 0; i < s_knownCount; i++) {
        if (strcmp(s_known[i].ssid, ssid) == 0) return;
    }
    strlcpy(s_known[s_knownCount].ssid, ssid, sizeof(Cred::ssid));
    strlcpy(s_known[s_knownCount].pass, pass ? pass : "", sizeof(Cred::pass));
    s_knownCount++;
}

const Cred* findKnown(const char* ssid) {
    for (uint8_t i = 0; i < s_knownCount; i++) {
        if (strcmp(s_known[i].ssid, ssid) == 0) return &s_known[i];
    }
    return nullptr;
}

void connectTo(const char* ssid, const char* pass) {
    WiFi.disconnect(false, false);
    s_lastReason = 0;
    const bool usePass = pass && pass[0] && !isOpen(ssid);
    Serial.printf("WiFi: begin %s (%s)\n", ssid, usePass ? "with password" : "open, no password");
    if (usePass) WiFi.begin(ssid, pass);
    else WiFi.begin(ssid);
}

void tryNextKnown() {
    if (!s_knownCount) return;
    const Cred& c = s_known[s_next];
    s_next = (s_next + 1) % s_knownCount;
    Serial.printf("WiFi: trying %s\n", c.ssid);
    connectTo(c.ssid, c.pass);
    s_attemptAt = millis();
    s_attempting = true;
}

const char* failMessage(uint8_t reason) {
    switch (reason) {
        case WIFI_REASON_AUTH_FAIL:
        case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
        case WIFI_REASON_HANDSHAKE_TIMEOUT:
            return "Sai mật khẩu";
        case WIFI_REASON_AUTH_EXPIRE:
        case WIFI_REASON_ASSOC_EXPIRE:
        case WIFI_REASON_BEACON_TIMEOUT:
            return "Sóng yếu, router không phản hồi";
        case WIFI_REASON_NO_AP_FOUND:
            return "Không tìm thấy mạng";
        default:
            return "Không kết nối được";
    }
}

void finishScan(int16_t n) {
    hmi::WifiNetwork list[kMaxScan];
    uint8_t count = 0;
    s_openCount = 0;
    for (int16_t i = 0; i < n && count < kMaxScan; i++) {
        const String ssid = WiFi.SSID(i);
        if (ssid.isEmpty()) continue;  // hidden networks
        bool dup = false;              // results are sorted by RSSI: keep the first
        for (uint8_t j = 0; j < count && !dup; j++) dup = strcmp(list[j].ssid, ssid.c_str()) == 0;
        if (dup) continue;
        strlcpy(list[count].ssid, ssid.c_str(), sizeof(list[count].ssid));
        list[count].rssi = WiFi.RSSI(i);
        list[count].secure = WiFi.encryptionType(i) != WIFI_AUTH_OPEN;
        if (!list[count].secure) strlcpy(s_open[s_openCount++], list[count].ssid, sizeof(s_open[0]));
        if (s_bootScan)
            Serial.printf("WiFi:   %4d dBm ch%-2d %s %s\n", list[count].rssi, WiFi.channel(i),
                          list[count].secure ? "WPA " : "open", list[count].ssid);
        const Cred* k = findKnown(list[count].ssid);
        list[count].saved = k && (k->pass[0] || !list[count].secure);
        count++;
    }
    WiFi.scanDelete();
    s_scanning = false;
    hmi::setWifiNetworks(list, count);
    hmi::setWifiScanning(false);
    Serial.printf("WiFi: scan found %u networks\n", count);

    if (WiFi.status() != WL_CONNECTED) {
        lockEspNowChannel();
    }
}

}  // namespace

void begin(const storage::Config& sd) {
    Preferences p;
    p.begin("wifi", false);
    const String ssid = p.getString("ssid", "");
    const String pass = p.getString("pass", "");
    p.end();
    addKnown(ssid.c_str(), pass.c_str());
    for (uint8_t i = 0; i < sd.wifiCount; i++) addKnown(sd.wifiSsid[i], sd.wifiPass[i]);
    addKnown(WIFI_SSID, WIFI_PASSWORD);

    WiFi.onEvent([](WiFiEvent_t, WiFiEventInfo_t info) {
        s_lastReason = info.wifi_sta_disconnected.reason;
        lockEspNowChannel();
    }, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

    WiFi.onEvent([](WiFiEvent_t, WiFiEventInfo_t) {
        esp_wifi_set_ps(WIFI_PS_NONE);
    }, ARDUINO_EVENT_WIFI_STA_CONNECTED);

    WiFi.setHostname(WIFI_HOSTNAME);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    esp_wifi_set_ps(WIFI_PS_NONE);
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    WiFi.setAutoReconnect(false);
    Serial.printf("WiFi: %u known network(s)\n", s_knownCount);
    s_bootScan = true;
    finishScan(WiFi.scanNetworks(false, false));
    s_bootScan = false;
    tryNextKnown();
}

void loop() {
    const uint32_t now = millis();

    if (s_scanning) {
        const int16_t n = WiFi.scanComplete();
        if (n >= 0) finishScan(n);
        else if (n == WIFI_SCAN_FAILED) {
            s_scanning = false;
            hmi::setWifiScanning(false);
            if (WiFi.status() != WL_CONNECTED) lockEspNowChannel();
        }
        return;
    }

    if (s_joining) {
        const bool fatal = s_lastReason == WIFI_REASON_AUTH_FAIL || s_lastReason == WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT ||
                           s_lastReason == WIFI_REASON_HANDSHAKE_TIMEOUT || s_lastReason == WIFI_REASON_NO_AP_FOUND;
        if (WiFi.status() == WL_CONNECTED) {
            s_joining = false;
            esp_wifi_set_ps(WIFI_PS_NONE);
            Preferences p;
            p.begin("wifi", false);
            p.putString("ssid", s_join.ssid);
            p.putString("pass", s_join.pass);
            p.end();
            memmove(&s_known[1], &s_known[0], sizeof(Cred) * (kMaxKnown - 1));
            s_known[0] = s_join;
            if (s_knownCount < kMaxKnown) s_knownCount++;
            for (uint8_t i = 1; i < s_knownCount; i++) {
                if (strcmp(s_known[i].ssid, s_join.ssid) == 0) {
                    memmove(&s_known[i], &s_known[i + 1], sizeof(Cred) * (s_knownCount - i - 1));
                    s_knownCount--;
                    break;
                }
            }
            s_next = 0;
            hmi::setWifiJoin(hmi::WifiJoin::Connected, WiFi.localIP().toString().c_str());
            storage::logEvent("Wi-Fi joined from panel: %s", s_join.ssid);
        } else if ((fatal && now - s_joinAt > 3000) || now - s_joinAt > kJoinMs) {
            s_joining = false;
            const char* msg = failMessage(s_lastReason);
            hmi::setWifiJoin(hmi::WifiJoin::Failed, msg);
            storage::logEvent("Wi-Fi join %s failed: %s (reason %u)", s_join.ssid, msg, s_lastReason);
            lockEspNowChannel();
            s_attemptAt = now;
            tryNextKnown();
        }
        return;
    }

    // Automatic connection: one attempt per known network, then back off.
    if (WiFi.status() == WL_CONNECTED) {
        s_attempting = false;
        s_retryDelay = kRetryMinMs;
        return;
    }
    if (s_attempting) {
        // Thoát ngay sau ~2.5s nếu báo NO_AP_FOUND thay vì chờ đủ 12s làm lệch kênh ESP-NOW
        const bool fastAbort = (s_lastReason == WIFI_REASON_NO_AP_FOUND) && (now - s_attemptAt > 2500);
        if (!fastAbort && (now - s_attemptAt < kAttemptMs)) return;

        // VÔ HIỆU HÓA: Không gọi disconnect để tránh nhảy kênh vô ích
        // WiFi.disconnect(false, false); 
        lockEspNowChannel();
        s_attempting = false;
        s_nextAttempt = now + s_retryDelay;
        // Serial.printf("WiFi: not connected, next try in %lus\n", (unsigned long)(s_retryDelay / 1000));
        s_retryDelay = s_retryDelay * 2 > kRetryMaxMs ? kRetryMaxMs : s_retryDelay * 2;
        return;
    }
    
    // VÔ HIỆU HÓA: Không tự động quét lại Wi-Fi theo chu kỳ
    // if (s_knownCount && (int32_t)(now - s_nextAttempt) >= 0) tryNextKnown();
}

void startScan() {
    if (s_scanning) return;
    if (WiFi.status() != WL_CONNECTED) WiFi.disconnect(false, false);
    s_scanning = WiFi.scanNetworks(true, false) == WIFI_SCAN_RUNNING;
    hmi::setWifiScanning(s_scanning);
    if (!s_scanning) Serial.println("WiFi: scan could not start");
}

void join(const char* ssid, const char* password) {
    strlcpy(s_join.ssid, ssid, sizeof(Cred::ssid));
    strlcpy(s_join.pass, password ? password : "", sizeof(Cred::pass));
    if (strcmp(s_join.pass, kUseSaved) == 0) {
        const Cred* k = findKnown(ssid);
        strlcpy(s_join.pass, k ? k->pass : "", sizeof(Cred::pass));
    }
    s_joining = true;
    s_joinAt = millis();
    hmi::setWifiJoin(hmi::WifiJoin::Connecting, "");
    Serial.printf("WiFi: joining %s\n", ssid);
    connectTo(s_join.ssid, s_join.pass);
}

void joinBssid(const char* ssid, const uint8_t* bssid, int32_t channel) {
    join(ssid, kUseSaved);
    WiFi.disconnect(false, false);
    WiFi.begin(s_join.ssid, s_join.pass[0] ? s_join.pass : nullptr, channel, bssid);
    Serial.printf("WiFi: joining %s via %02x:%02x:%02x:%02x:%02x:%02x ch%ld, panel MAC %s\n", ssid, bssid[0], bssid[1],
                  bssid[2], bssid[3], bssid[4], bssid[5], (long)channel, WiFi.macAddress().c_str());
}

void retryWithNewMac(const char* ssid) {
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    mac[0] = (mac[0] | 0x02) & 0xFE;
    mac[5] ^= 0x5A;
    WiFi.disconnect(false, false);
    esp_wifi_stop();
    const esp_err_t err = esp_wifi_set_mac(WIFI_IF_STA, mac);
    esp_wifi_start();
    Serial.printf("WiFi: station MAC now %s (%s)\n", WiFi.macAddress().c_str(), esp_err_to_name(err));
    join(ssid, kUseSaved);
}

void retry11b(const char* ssid) {
    WiFi.disconnect(false, false);
    const esp_err_t err = esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B);
    Serial.printf("WiFi: protocol 802.11b only (%s)\n", esp_err_to_name(err));
    join(ssid, kUseSaved);
}

bool connected() { return WiFi.status() == WL_CONNECTED; }

}  // namespace wifimgr