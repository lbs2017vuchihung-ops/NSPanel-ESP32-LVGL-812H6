// Business logic as FreeRTOS tasks.  The LVGL HMI (src/hmi/) runs in its own
// "ui" task and is driven only through include/hmi.h.
//
//   task       core prio  owns
//   ui          1   3     LVGL, touch, backlight              (src/hmi/hmi_ui.cpp)
//   app         1   4     relays, sensors, settings (NVS)     queue: AppMsg
//   net         0   2     Wi-Fi manager, MQTT, OTA, NTP       queue: NetMsg
//   devices     0   2     Tasmota relays over HTTP            (src/tasmota.cpp)
//   storage     0   1     SD card: logs, stats, fw update     (src/storage.cpp)
//   console     0   1     serial commands
//
// Tasks never call each other's code directly: they post messages.  Shared
// state is either in the HMI model (locked) or std::atomic.

#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <esp_task_wdt.h>
#include <time.h>

#include <atomic>

#include "config.h"
#include "hmi.h"
#include "lgfx_4848s040_reference.h"
#include "datalog.h"
#include "storage.h"
#include "tasmota.h"
#include "wifi_manager.h"
#include "espnow_manager.h"

// Touch is read through LGFX (Touch_GT911 on I2C_NUM_1).  Do not also start
// Wire on GPIO19/45 - two drivers on the same pins corrupt touch reads.
LGFX display;

namespace
{

    // ===================== shared state =====================

    const uint8_t kRelayPins[RELAY_COUNT] = {RELAY_PIN_1, RELAY_PIN_2, RELAY_PIN_3};
    std::atomic<bool> g_relay[RELAY_COUNT]; // written by app, read by net (MQTT state)
    std::atomic<uint32_t> g_relayChange[RELAY_COUNT];
    std::atomic<bool> g_publishPending{false}; // app -> net: publish MQTT state
    std::atomic<uint32_t> g_minFreeHeap{UINT32_MAX};
    uint32_t g_bootTime = 0;
    float g_currentTemp = 0.0;
    float g_currentHum = 0.0;

    storage::Config g_sdConfig; // /config.json, read once in setup()
    char g_mqttHost[64] = "app.coreiot.io";
    uint16_t g_mqttPort = 1883;
    char g_mqttUser[32] = "IxeF1gv7u8t2AOLQvoZP";
    char g_mqttPass[64] = "";

    // ===================== messages =====================

    std::atomic<bool> g_acOn[hmi::kAcCount];
    std::atomic<uint8_t> g_acTemp[hmi::kAcCount];
    std::atomic<uint8_t> g_acFan[hmi::kAcCount];   // 0=Auto, 1=Yên tĩnh, 2..6=Mức 1..5
    std::atomic<uint8_t> g_acSwing[hmi::kAcCount]; // 0=Off, 1=On

    enum class AppCmd : uint8_t
    {
        RelayToggle,
        RelaySet,
        AcSetPower, // flag = power
        AcSetTemp,  // v[0] = temp (18..32)
        AcSetFan,   // v[0] = fan (0..6)
        AcSetSwing, // flag = swing (0/1)
        AcSetFull,  // flag = power, v[0] = temp, v[1] = fan (-1 giữ nguyên), v[2] = swing (-1 giữ nguyên)
        Sensors,
        SensorPreview,
        SaveBrightness,
        SaveAutoDim,
        SaveTheme
    };
    struct AppMsg
    {
        AppCmd cmd;
        uint8_t index;
        bool flag;
        float v[3];
        char text[12];
    };

    enum class NetCmd : uint8_t
    {
        Scan,
        Join
    };
    struct NetMsg
    {
        NetCmd cmd;
        char ssid[33];
        char pass[65];
    };

    QueueHandle_t g_appQ = nullptr;
    QueueHandle_t g_netQ = nullptr;

    void postApp(const AppMsg &m) { xQueueSend(g_appQ, &m, 0); }
    void postNet(const NetMsg &m) { xQueueSend(g_netQ, &m, 0); }

    // ===================== app task =====================
    Preferences g_prefs; // Đặt trước applyAcControl để tránh lỗi biên dịch

    uint32_t g_acLastPowerMs[hmi::kAcCount] = {};  // Mốc thời gian bật/tắt gần nhất
    bool g_acTempPending[hmi::kAcCount] = {};      // Cờ chờ chốt nhiệt độ sau khi ngừng kéo
    uint32_t g_acTempChangeMs[hmi::kAcCount] = {}; // Mốc thời gian kéo thanh trượt gần nhất

    constexpr uint32_t kAcPowerCooldownMs = 1500; // Khóa chống bấm bật/tắt liên tục dưới 1.5 giây
    constexpr uint32_t kAcTempDebounceMs = 450;   // Chờ ngừng kéo 450ms mới gửi ESP-NOW & ghi NVS

    void saveAcPrefs(uint8_t index, uint8_t temp, uint8_t fan, uint8_t swing)
    {
        char key[12];
        snprintf(key, sizeof(key), "ac_t%u", index);
        g_prefs.putUChar(key, temp);
        snprintf(key, sizeof(key), "ac_f%u", index);
        g_prefs.putUChar(key, fan);
        snprintf(key, sizeof(key), "ac_s%u", index);
        g_prefs.putUChar(key, swing);
    }

    void applyAcControl(uint8_t index, bool on, uint8_t temp, int8_t fan = -1, int8_t swing = -1, bool persist = false)
    {
        if (index >= hmi::kAcCount)
            return;
        temp = constrain(temp, 18, 32);
        const uint8_t finalFan = (fan >= 0) ? static_cast<uint8_t>(constrain(fan, 0, 6)) : g_acFan[index].load();
        const uint8_t finalSwing = (swing >= 0) ? (swing ? 1 : 0) : g_acSwing[index].load();

        g_acOn[index] = on;
        g_acTemp[index] = temp;
        g_acFan[index] = finalFan;
        g_acSwing[index] = finalSwing;

        if (persist)
            saveAcPrefs(index, temp, finalFan, finalSwing);

        // 1. Đồng bộ Model -> HMI UI
        hmi::setAc(index, on, static_cast<float>(temp), true);
        hmi::setAcFanSwing(index, finalFan, finalSwing);

        // 2. Gửi gói tin ESP-NOW tới Slave trung tâm (Node #1) kèm đủ Temp, Fan, Swing
        espnowSendAcCommand(index + 1, on, temp, finalFan, finalSwing);

        // 3. Ghi log & báo netTask đẩy telemetry lên CoreIoT
        storage::logEvent("AC %u %s (%uC F:%u S:%u)", index + 1, on ? "ON" : "OFF", temp, finalFan, finalSwing);
        g_publishPending = true;
    }

    struct SensorValues
    {
        float primary = 0, metric1 = 0, metric2 = 0;
        bool primaryError = false;
        bool stale = false;
    };

    void setRelay(int index, bool on)
    {
        if (index < 0 || index >= RELAY_COUNT || g_relay[index] == on)
            return;
        g_relay[index] = on;
        g_relayChange[index] = millis();
        digitalWrite(kRelayPins[index], on ? HIGH : LOW);
        hmi::setRelay(index, on);
        storage::logEvent("Relay %d %s", index + 1, on ? "ON" : "OFF");
        g_publishPending = true;
    }

    void pushSensors(const SensorValues &s, hmi::SensorState state)
    {
        g_currentTemp = s.primary; // Lấy nhiệt độ
        g_currentHum = s.metric1;  // Lấy độ ẩm
        hmi::setPrimary(s.primary, s.primaryError ? hmi::SensorState::Error : state);
        hmi::setMetric(0, s.metric1, state);
        hmi::setMetric(1, s.metric2, state);
        static uint32_t lastLog = 0;
        if (state == hmi::SensorState::Ok && !s.primaryError && (!lastLog || millis() - lastLog >= 60000))
        {
            lastLog = millis();
            storage::logSensors(s.primary, s.metric1, s.metric2);
        }
        if (state == hmi::SensorState::Ok && !s.primaryError)
            datalog::addSample(s.primary, s.metric1, s.metric2);
    }

    void appTask(void *)
    {
        SensorValues sensors;
        bool simulate = HMI_SIMULATE_SENSORS;
        uint32_t lastSim = 0, lastReal = 0;
        esp_task_wdt_add(nullptr);

        for (;;)
        {
            AppMsg m;
            datalog::tick(); // close finished history slots (10 s / 1 min)
            if (xQueueReceive(g_appQ, &m, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                switch (m.cmd)
                {
                case AppCmd::RelayToggle:
                    if (m.index < RELAY_COUNT)
                        setRelay(m.index, !g_relay[m.index]);
                    break;
                case AppCmd::RelaySet:
                    setRelay(m.index, m.flag);
                    break;
                case AppCmd::Sensors: // real data (MQTT): simulation off for good
                    simulate = false;
                    lastReal = millis();
                    sensors.stale = false;
                    sensors.primaryError = m.flag;
                    if (!isnan(m.v[0]))
                        sensors.primary = m.v[0];
                    if (!isnan(m.v[1]))
                        sensors.metric1 = m.v[1];
                    if (!isnan(m.v[2]))
                        sensors.metric2 = m.v[2];
                    pushSensors(sensors, hmi::SensorState::Ok);
                    break;
                case AppCmd::SensorPreview:
                { // console "sensor ok|warn|error|loading|stale|sim"
                    const char *s = m.text;
                    simulate = strcmp(s, "sim") == 0;
                    if (strcmp(s, "ok") == 0)
                        hmi::setPrimary(26.4f);
                    else if (strcmp(s, "warn") == 0)
                        hmi::setPrimary(31.8f);
                    else if (strcmp(s, "error") == 0)
                        hmi::setPrimary(0, hmi::SensorState::Error);
                    else if (strcmp(s, "loading") == 0)
                        hmi::setPrimary(0, hmi::SensorState::Loading);
                    else if (strcmp(s, "stale") == 0)
                        hmi::setPrimary(26.4f, hmi::SensorState::Stale);
                    break;
                }
                case AppCmd::SaveBrightness:
                    g_prefs.putUChar("bright", m.index);
                    break;
                case AppCmd::SaveAutoDim:
                    g_prefs.putBool("autodim", m.flag);
                    break;
                case AppCmd::SaveTheme:
                    g_prefs.putUChar("theme", m.index);
                    break;
                case AppCmd::AcSetPower:
                    if (m.index < hmi::kAcCount)
                    {
                        if (g_acOn[m.index].load() == m.flag)
                            break;

                        const uint32_t nowMs = millis();
                        if (g_acLastPowerMs[m.index] != 0 && (nowMs - g_acLastPowerMs[m.index] < kAcPowerCooldownMs))
                        {
                            Serial.printf("[AC%u] Chan thao tac bat/tat qua nhanh (<%lums)\n",
                                          m.index + 1, (unsigned long)kAcPowerCooldownMs);
                            hmi::setAc(m.index, g_acOn[m.index].load(), static_cast<float>(g_acTemp[m.index].load()), true);
                            break;
                        }

                        g_acLastPowerMs[m.index] = nowMs;
                        g_acTempPending[m.index] = false;
                        applyAcControl(m.index, m.flag, g_acTemp[m.index].load(), -1, -1, false);
                    }
                    break;

                case AppCmd::AcSetTemp:
                    if (m.index < hmi::kAcCount)
                    {
                        uint8_t newTemp = static_cast<uint8_t>(constrain(lroundf(m.v[0]), 18, 32));
                        if (g_acTemp[m.index].load() != newTemp)
                        {
                            // Cập nhật ngay UI & biến trạng thái, hoãn gửi ESP-NOW/NVS 450ms để chống kéo liên tục
                            g_acTemp[m.index] = newTemp;
                            hmi::setAc(m.index, g_acOn[m.index].load(), static_cast<float>(newTemp));
                            g_acTempPending[m.index] = true;
                            g_acTempChangeMs[m.index] = millis();
                        }
                    }
                    break;

                case AppCmd::AcSetFan:
                    if (m.index < hmi::kAcCount)
                    {
                        uint8_t newFan = static_cast<uint8_t>(constrain(lroundf(m.v[0]), 0, 6));
                        if (g_acFan[m.index].load() != newFan)
                        {
                            g_acFan[m.index] = newFan;
                            char key[12];
                            snprintf(key, sizeof(key), "ac_f%u", m.index);
                            g_prefs.putUChar(key, newFan);

                            hmi::setAcFanSwing(m.index, newFan, g_acSwing[m.index].load());

                            const bool isOn = g_acOn[m.index].load();
                            if (isOn)
                            {
                                espnowSendAcCommand(m.index + 1, true, g_acTemp[m.index].load(),
                                                    newFan, g_acSwing[m.index].load());
                            }
                            storage::logEvent("AC %u fan -> %u (%s)", m.index + 1, newFan, isOn ? "TX" : "SAVED");
                            g_publishPending = true;
                        }
                    }
                    break;

                case AppCmd::AcSetSwing:
                    if (m.index < hmi::kAcCount)
                    {
                        uint8_t newSwing = m.flag ? 1 : 0;
                        if (g_acSwing[m.index].load() != newSwing)
                        {
                            g_acSwing[m.index] = newSwing;
                            char key[12];
                            snprintf(key, sizeof(key), "ac_s%u", m.index);
                            g_prefs.putUChar(key, newSwing);

                            hmi::setAcFanSwing(m.index, g_acFan[m.index].load(), newSwing);

                            const bool isOn = g_acOn[m.index].load();
                            if (isOn)
                            {
                                espnowSendAcCommand(m.index + 1, true, g_acTemp[m.index].load(),
                                                    g_acFan[m.index].load(), newSwing);
                            }
                            storage::logEvent("AC %u swing -> %u (%s)", m.index + 1, newSwing, isOn ? "TX" : "SAVED");
                            g_publishPending = true;
                        }
                    }
                    break;

                case AppCmd::AcSetFull:
                    if (m.index < hmi::kAcCount)
                    {
                        uint8_t newTemp = static_cast<uint8_t>(constrain(lroundf(m.v[0]), 18, 32));
                        int8_t newFan = (m.v[1] >= 0.0f) ? static_cast<int8_t>(constrain(lroundf(m.v[1]), 0, 6)) : -1;
                        int8_t newSwing = (m.v[2] >= 0.0f) ? (lroundf(m.v[2]) ? 1 : 0) : -1;

                        g_acLastPowerMs[m.index] = millis();
                        g_acTempPending[m.index] = false;
                        applyAcControl(m.index, m.flag, newTemp, newFan, newSwing, true);
                    }
                    break;
                }
            }

            const uint32_t now = millis();
            for (uint8_t i = 0; i < hmi::kAcCount; i++) // chốt giá trị nhiệt độ sau khi ngừng kéo thanh trượt
            {
                if (g_acTempPending[i] && (now - g_acTempChangeMs[i] >= kAcTempDebounceMs))
                {
                    g_acTempPending[i] = false;
                    const uint8_t finalTemp = g_acTemp[i].load();
                    const uint8_t curFan = g_acFan[i].load();
                    const uint8_t curSwing = g_acSwing[i].load();
                    const bool isOn = g_acOn[i].load();

                    // 1. Lưu Flash NVS 1 lần duy nhất sau khi ngừng thao tác
                    char key[12];
                    snprintf(key, sizeof(key), "ac_t%u", i);
                    g_prefs.putUChar(key, finalTemp);

                    // 2. Chỉ phát gói ESP-NOW sang Slave khi máy lạnh đang BẬT (kèm đúng Fan/Swing hiện tại)
                    if (isOn)
                        espnowSendAcCommand(i + 1, true, finalTemp, curFan, curSwing);

                    storage::logEvent("AC %u temp -> %uC (%s)", i + 1, finalTemp, isOn ? "TX" : "SAVED");
                    g_publishPending = true;
                }
            }
            if (simulate && now - g_bootTime > 1500 && (!lastSim || now - lastSim >= 2000))
            {
                if (!lastSim)
                    sensors = SensorValues{26.4f, 62, 412};
                lastSim = now;
                sensors.primary = constrain(sensors.primary + random(-60, 61) / 100.0f, 23.5f, 28.5f);
                sensors.metric1 = constrain(sensors.metric1 + random(-2, 3), 55.0f, 70.0f);
                sensors.metric2 = constrain(sensors.metric2 + random(-30, 31), 320.0f, 520.0f);
                pushSensors(sensors, hmi::SensorState::Ok);
            }
            if (!simulate && lastReal && !sensors.stale && now - lastReal > HMI_SENSOR_STALE_MS)
            {
                sensors.stale = true;
                storage::logEvent("Sensor data stale");
                pushSensors(sensors, hmi::SensorState::Stale);
            }
            esp_task_wdt_reset();
        }
    }

    // ===================== net task =====================

    WiFiClient g_wifiClient;
    PubSubClient g_mqtt(g_wifiClient);

    // Chuyển đổi params Fan từ CoreIoT (số 0..6 hoặc chuỗi "Auto"/"Quiet"/"Muc 1".."Muc 5")
    int8_t parseFanParam(JsonVariantConst v)
    {
        if (v.is<int>())
            return static_cast<int8_t>(constrain(v.as<int>(), 0, 6));
        if (v.is<const char *>())
        {
            const char *s = v.as<const char *>();
            if (strcasecmp(s, "Auto") == 0)
                return 0;
            if (strcasecmp(s, "Quiet") == 0 || strcasecmp(s, "Yen tinh") == 0)
                return 1;
            if (strcasecmp(s, "Muc 1") == 0 || strcmp(s, "1") == 0)
                return 2;
            if (strcasecmp(s, "Muc 2") == 0 || strcmp(s, "2") == 0)
                return 3;
            if (strcasecmp(s, "Muc 3") == 0 || strcmp(s, "3") == 0)
                return 4;
            if (strcasecmp(s, "Muc 4") == 0 || strcmp(s, "4") == 0)
                return 5;
            if (strcasecmp(s, "Muc 5") == 0 || strcmp(s, "5") == 0)
                return 6;
        }
        return -1;
    }

    void mqttCallback(char *topic, byte *payload, unsigned int length)
    {
        Serial.printf("MQTT [%s]: %.*s\n", topic, length, (const char *)payload);
        JsonDocument doc;
        if (deserializeJson(doc, payload, length))
            return;

        AppMsg m{};
        if (strcmp(topic, "v1/devices/me/telemetry") == 0)
        {
            m.cmd = AppCmd::Sensors;
            m.flag = doc["error"] | false;
            m.v[0] = doc["primary"].is<float>() ? doc["primary"].as<float>() : NAN;
            m.v[1] = doc["metric1"].is<float>() ? doc["metric1"].as<float>() : NAN;
            m.v[2] = doc["metric2"].is<float>() ? doc["metric2"].as<float>() : NAN;
            postApp(m);
            return;
        }
        if (strncmp(topic, "v1/devices/me/rpc/request/", 26) != 0)
            return;

        if (doc["method"].is<const char *>())
        {
            const char *method = doc["method"];
            bool state = doc["params"] | false;

            if (strcmp(method, "setLight") == 0)
            {
                m.cmd = AppCmd::RelaySet;
                m.index = 0;
                m.flag = state;
                postApp(m);
                Serial.printf("Received setLight: %d\n", state);
            }
            else if (strncmp(method, "setAC", 5) == 0 && method[5] >= '1' && method[5] <= '3')
            {
                const uint8_t idx = static_cast<uint8_t>(method[5] - '1'); // '1'..'3' -> 0..2
                const char *sub = method + 6;

                if (strcmp(sub, "Temp") == 0)
                {
                    // RPC "setAC1Temp" -> params: 18..32
                    float t = doc["params"].as<float>();
                    if (t >= 18.0f && t <= 32.0f)
                        postApp(AppMsg{AppCmd::AcSetTemp, idx, false, {t, 0, 0}, {}});
                }
                else if (strcmp(sub, "Fan") == 0)
                {
                    // RPC "setAC1Fan" -> params: 0..6 hoặc "Auto"/"Quiet"/"Muc 1".."Muc 5"
                    int8_t f = parseFanParam(doc["params"]);
                    if (f >= 0)
                        postApp(AppMsg{AppCmd::AcSetFan, idx, false, {static_cast<float>(f), 0, 0}, {}});
                }
                else if (strcmp(sub, "Swing") == 0)
                {
                    // RPC "setAC1Swing" -> params: true/false hoặc 1/0
                    bool sw = doc["params"].is<bool>() ? doc["params"].as<bool>() : (doc["params"].as<int>() != 0);
                    postApp(AppMsg{AppCmd::AcSetSwing, idx, sw, {}, {}});
                }
                else if (*sub == '\0')
                {
                    // RPC "setAC1" -> params: bool hoặc {"power":true,"temp":24,"fan":2,"swing":true}
                    if (doc["params"].is<JsonObject>())
                    {
                        JsonObjectConst p = doc["params"].as<JsonObjectConst>();
                        bool pwr = p["power"].is<bool>() ? p["power"].as<bool>() : g_acOn[idx].load();
                        float t = p["temp"].is<float>() ? p["temp"].as<float>() : static_cast<float>(g_acTemp[idx].load());
                        float f = p["fan"].isNull() ? -1.0f : static_cast<float>(parseFanParam(p["fan"]));
                        float sw = p["swing"].isNull() ? -1.0f : ((p["swing"].is<bool>() ? p["swing"].as<bool>() : (p["swing"].as<int>() != 0)) ? 1.0f : 0.0f);
                        postApp(AppMsg{AppCmd::AcSetFull, idx, pwr, {t, f, sw}, {}});
                    }
                    else
                    {
                        bool pwr = doc["params"] | false;
                        postApp(AppMsg{AppCmd::AcSetPower, idx, pwr, {}, {}});
                    }
                }
            }
        }
        else if (doc["relays"].is<JsonArray>())
        {
            JsonArray relays = doc["relays"];
            for (size_t i = 0; i < relays.size() && i < RELAY_COUNT; i++)
            {
                postApp(AppMsg{AppCmd::RelaySet, static_cast<uint8_t>(i), relays[i].as<bool>(), {}, {}});
            }
        }
        else if (doc["brightness"].is<int>())
        {
            hmi::setBrightness(map(doc["brightness"].as<int>(), 0, 255, 0, 100));
        }
    }

    void publishState()
    {
        if (!g_mqtt.connected())
            return;
        JsonDocument doc;
        doc["device"] = DEVICE_NAME;
        doc["type"] = DEVICE_TYPE;
        doc["fw_version"] = FIRMWARE_VERSION;
        doc["hw_version"] = HARDWARE_VERSION;
        doc["uptime"] = (millis() - g_bootTime) / 1000;
        doc["free_heap"] = ESP.getFreeHeap();
        doc["min_free_heap"] = g_minFreeHeap.load();
        doc["wifi_rssi"] = WiFi.RSSI();
        doc["ip"] = WiFi.localIP().toString();
        doc["temperature"] = g_currentTemp;
        doc["humidity"] = g_currentHum;

        JsonArray relays = doc["relays"].to<JsonArray>();
        for (int i = 0; i < RELAY_COUNT; i++)
        {
            JsonObject r = relays.add<JsonObject>();
            r["index"] = i;
            r["state"] = g_relay[i].load();
            r["last_change"] = (millis() - g_relayChange[i]) / 1000;
        }

        // Đẩy trạng thái đầy đủ 3 máy lạnh (Power, Temp, Fan, Swing) lên CoreIoT
        for (uint8_t i = 0; i < hmi::kAcCount; i++)
        {
            char kPwr[16], kTmp[16], kFan[16], kSwg[16];
            snprintf(kPwr, sizeof(kPwr), "ac%u_power", i + 1);
            snprintf(kTmp, sizeof(kTmp), "ac%u_temp", i + 1);
            snprintf(kFan, sizeof(kFan), "ac%u_fan", i + 1);
            snprintf(kSwg, sizeof(kSwg), "ac%u_swing", i + 1);

            doc[kPwr] = g_acOn[i].load();
            doc[kTmp] = g_acTemp[i].load();
            doc[kFan] = g_acFan[i].load();
            doc[kSwg] = g_acSwing[i].load() != 0;
        }

        String out;
        serializeJson(doc, out);
        g_mqtt.publish("v1/devices/me/telemetry", out.c_str(), true);
    }

    void setupOTA()
    {
        ArduinoOTA.setHostname(WIFI_HOSTNAME);
        ArduinoOTA.setPassword(OTA_PASSWORD);
        ArduinoOTA.setPort(OTA_PORT);
        ArduinoOTA.onStart([]()
                           { storage::logEvent("OTA start"); });
        ArduinoOTA.onError([](ota_error_t e)
                           { storage::logEvent("OTA error %u", e); });
        ArduinoOTA.begin();
    }

    // MQTT with exponential backoff (5..60 s) and a 1 s TCP timeout.
    void maintainMqtt(uint32_t now)
    {
        static uint32_t lastTry = 0, retryDelay = MQTT_RECONNECT_INTERVAL;
        if (!wifimgr::connected() || g_mqtt.connected())
            return;
        if (lastTry && now - lastTry < retryDelay)
            return;
        lastTry = now;
        String id = String(MQTT_CLIENT_ID) + "-" + String(random(0xffff), HEX);
        if (g_mqtt.connect(id.c_str(), g_mqttUser[0] ? g_mqttUser : nullptr, g_mqttPass[0] ? g_mqttPass : nullptr,
                           "v1/devices/me/telemetry", 1, true, "{\"status\":\"offline\"}"))
        {
            storage::logEvent("MQTT connected to %s:%u", g_mqttHost, g_mqttPort);
            g_mqtt.subscribe("v1/devices/me/rpc/request/+");
            g_mqtt.publish("v1/devices/me/telemetry", "{\"status\":\"online\"}", true);
            publishState();
            retryDelay = MQTT_RECONNECT_INTERVAL;
        }
        else
        {
            Serial.printf("MQTT connect failed rc=%d, retry in %lus\n", g_mqtt.state(), retryDelay / 1000);
            retryDelay = min<uint32_t>(retryDelay * 2, 60000);
        }
    }

    // Link, clock and system info for the header / Settings, once a second.
    void feedHmiStatus(uint32_t now)
    {
        static uint32_t last = 0;
        static uint32_t lastSystemInfo = 0;
        if (last && now - last < 1000)
            return;
        last = now;
        const bool up = wifimgr::connected();
        hmi::Link link = hmi::Link::Offline;
        if (up)
            link = g_mqtt.connected() ? hmi::Link::Online : hmi::Link::BrokerDown;
        else if (now - g_bootTime < 20000)
            link = hmi::Link::Connecting;
        hmi::setLink(link);

        struct tm t;
        char hhmm[6] = "--:--";
        if (getLocalTime(&t, 0))
            strftime(hhmm, sizeof(hhmm), "%H:%M", &t);
        hmi::setClock(hhmm);

        hmi::SystemInfo info;
        if (up)
        {
            strlcpy(info.ssid, WiFi.SSID().c_str(), sizeof(info.ssid));
            strlcpy(info.ip, WiFi.localIP().toString().c_str(), sizeof(info.ip));
            info.rssi = WiFi.RSSI();
            strlcpy(info.bssid, WiFi.BSSIDstr().c_str(), sizeof(info.bssid));
            info.channel = WiFi.channel();
            strlcpy(info.gateway, WiFi.gatewayIP().toString().c_str(), sizeof(info.gateway));
            strlcpy(info.subnet, WiFi.subnetMask().toString().c_str(), sizeof(info.subnet));
            strlcpy(info.dns, WiFi.dnsIP().toString().c_str(), sizeof(info.dns));
        }
        strlcpy(info.mac, WiFi.macAddress().c_str(), sizeof(info.mac));
        strlcpy(info.hostname, WIFI_HOSTNAME, sizeof(info.hostname));
        strlcpy(info.mqttHost, g_mqttHost, sizeof(info.mqttHost));
        info.mqttPort = g_mqttPort;
        info.mqttState = g_mqtt.state();
        info.mqttAuth = g_mqttUser[0] != 0;

        const uint32_t heap = ESP.getFreeHeap();
        if (heap < g_minFreeHeap)
            g_minFreeHeap = heap;
        info.heapKB = heap / 1024;
        info.heapMinKB = ESP.getMinFreeHeap() / 1024;
        info.heapTotalKB = ESP.getHeapSize() / 1024;
        info.heapLargestKB = ESP.getMaxAllocHeap() / 1024;
        info.psramKB = ESP.getFreePsram() / 1024;
        info.psramTotalKB = ESP.getPsramSize() / 1024;
        info.flashMB = ESP.getFlashChipSize() / (1024 * 1024);
        static const uint32_t appKB = ESP.getSketchSize() / 1024; // hashes the image: once
        info.appKB = appKB;
        info.appSlotKB = ESP.getFreeSketchSpace() / 1024;

        info.uptimeS = (now - g_bootTime) / 1000;
        info.resetReason = static_cast<uint8_t>(esp_reset_reason());
        info.timeSynced = getLocalTime(&t, 0);
        if (info.timeSynced)
            strftime(info.date, sizeof(info.date), "%d/%m/%Y", &t);
        strlcpy(info.chip, ESP.getChipModel(), sizeof(info.chip));
        info.chipRev = ESP.getChipRevision();
        info.cores = ESP.getChipCores();
        info.cpuMHz = ESP.getCpuFreqMHz();
        strlcpy(info.sdk, ESP.getSdkVersion(), sizeof(info.sdk));
        if (!lastSystemInfo || now - lastSystemInfo >= 5000)
        {
            lastSystemInfo = now;
            hmi::setSystemInfo(info);
        }
    }

    void onEspNowSensor(uint8_t, float temp, float hum, float power)
    {
        AppMsg m{};
        m.cmd = AppCmd::Sensors;
        m.flag = isnan(temp);
        m.v[0] = temp;
        m.v[1] = hum;
        m.v[2] = power;
        postApp(m);
    }

    void netTask(void *)
    {
        g_wifiClient.setTimeout(1); // bound the blocking TCP connect
        g_mqtt.setServer(g_mqttHost, g_mqttPort);
        g_mqtt.setCallback(mqttCallback);
        g_mqtt.setBufferSize(2048);
        g_mqtt.setKeepAlive(MQTT_KEEPALIVE);
        g_mqtt.setSocketTimeout(2);
        wifimgr::begin(g_sdConfig);
        espnowInit(onEspNowSensor); // Khởi tạo esp_now

        bool wasUp = false, otaStarted = false;
        for (;;)
        {
            NetMsg m;
            if (xQueueReceive(g_netQ, &m, pdMS_TO_TICKS(20)) == pdTRUE)
            {
                if (m.cmd == NetCmd::Scan)
                    wifimgr::startScan();
                else
                    wifimgr::join(m.ssid, m.pass);
            }
            const uint32_t now = millis();
            wifimgr::loop();

            const bool up = wifimgr::connected();
            if (up != wasUp)
            {
                wasUp = up;
                if (up)
                {
                    storage::logEvent("Wi-Fi connected: %s %s RSSI %d", WiFi.SSID().c_str(),
                                      WiFi.localIP().toString().c_str(), WiFi.RSSI());
                    configTime(HMI_TZ_OFFSET_SECONDS, 0, "pool.ntp.org", "time.google.com");
                    if (!otaStarted)
                    {
                        setupOTA();
                        otaStarted = true;
                    }
                }
                else
                {
                    storage::logEvent("Wi-Fi disconnected");
                }
            }
            maintainMqtt(now);
            g_mqtt.loop();
            static uint32_t lastPub = 0;
            if (g_publishPending.exchange(false) || (now - lastPub >= 5000))
            {
                lastPub = now;
                publishState();
            }
            if (otaStarted)
                ArduinoOTA.handle();
            feedHmiStatus(now);
        }
    }

    // ===================== console task =====================

    void printTasks()
    {
        static const char *const names[] = {"ui", "app", "net", "devices", "storage", "console"};
        Serial.printf("heap %u KB (min %u KB), psram free %u KB\n", ESP.getFreeHeap() / 1024,
                      ESP.getMinFreeHeap() / 1024, ESP.getFreePsram() / 1024);
        for (const char *n : names)
        {
            TaskHandle_t h = xTaskGetHandle(n);
            if (h)
                Serial.printf("  %-8s stack free %5u B  prio %u\n", n, uxTaskGetStackHighWaterMark(h), uxTaskPriorityGet(h));
        }
    }

    void consoleTask(void *)
    {
        char line[80];
        uint8_t len = 0;
        for (;;)
        {
            while (Serial.available())
            {
                const char c = Serial.read();
                if (c != '\n' && c != '\r')
                {
                    if (len < sizeof(line) - 1)
                        line[len++] = c;
                    continue;
                }
                line[len] = 0;
                len = 0;
                if (!line[0])
                    continue;
                if (strcmp(line, "shot") == 0)
                {
                    Serial.setDebugOutput(false);
                    hmi::screenshot(Serial);
                    Serial.flush();
                    Serial.setDebugOutput(true);
                }
                else if (strncmp(line, "shot fb", 7) == 0)
                {
                    Serial.setDebugOutput(false);
                    hmi::screenshotFramebuffer(Serial, line[7] ? atoi(line + 8) : 0);
                    Serial.flush();
                    Serial.setDebugOutput(true);
                }
                else if (strcmp(line, "shot sd") == 0)
                {
                    storage::requestScreenshot();
                }
                else if (strcmp(line, "sd tree") == 0)
                {
                    storage::requestTree();
                }
                else if (strcmp(line, "sd wipe") == 0)
                {
                    storage::requestWipe();
                }
                else if (strcmp(line, "sd restore") == 0)
                {
                    storage::requestRestore();
                }
                else if (strcmp(line, "history save") == 0)
                {
                    storage::requestHistorySave();
                }
                else if (strcmp(line, "info") == 0 || strcmp(line, "tasks") == 0)
                {
                    printTasks();
                }
                else if (strncmp(line, "toggle ", 7) == 0)
                {
                    postApp(AppMsg{AppCmd::RelayToggle, static_cast<uint8_t>(atoi(line + 7) - 1), false, {}, {}});
                }
                else if (strncmp(line, "page ", 5) == 0)
                {
                    hmi::showPage(atoi(line + 5));
                }
                else if (strncmp(line, "scroll ", 7) == 0)
                {
                    hmi::scrollPage(atoi(line + 7));
                }
                else if (strncmp(line, "drag ", 5) == 0)
                {
                    int x = 240, y0 = 300, y1 = 120, ms = 300;
                    sscanf(line + 5, "%d %d %d %d", &x, &y0, &y1, &ms);
                    hmi::injectDrag(x, y0, y1, ms);
                }
                else if (strncmp(line, "detail ", 7) == 0)
                {
                    hmi::showDetail(atoi(line + 7));
                }
                else if (strncmp(line, "bench", 5) == 0)
                {
                    hmi::benchmark(strstr(line, "psram") ? 1 : strstr(line, "hybrid") ? 2 : 0);
                }
                else if (strncmp(line, "view ", 5) == 0)
                {
                    hmi::previewFile(line + 5);
                }
                else if (strncmp(line, "kbd", 3) == 0)
                {
                    hmi::previewKeyboard(strstr(line, "open") != nullptr);
                }
                else if (strcmp(line, "devices") == 0)
                {
                    tasmota::print(Serial);
                }
                else if (strncmp(line, "wifi 11b ", 9) == 0)
                {
                    wifimgr::retry11b(line + 9);
                }
                else if (strncmp(line, "wifi mac ", 9) == 0)
                {
                    wifimgr::retryWithNewMac(line + 9);
                }
                else if (strcmp(line, "nodes") == 0)
                    espnowPrintNodes(Serial);
                else if (strncmp(line, "wifi bssid ", 11) == 0)
                {
                    unsigned b[6];
                    int ch = 0, off = 0;
                    if (sscanf(line + 11, "%x:%x:%x:%x:%x:%x %d %n", &b[0], &b[1], &b[2], &b[3], &b[4], &b[5], &ch, &off) >= 7)
                    {
                        const uint8_t bssid[6] = {(uint8_t)b[0], (uint8_t)b[1], (uint8_t)b[2], (uint8_t)b[3], (uint8_t)b[4], (uint8_t)b[5]};
                        wifimgr::joinBssid(line + 11 + off, bssid, ch);
                    }
                }
                else if (strncmp(line, "wifi join ", 10) == 0)
                {
                    NetMsg m{NetCmd::Join, {}, {}};
                    strlcpy(m.ssid, line + 10, sizeof(m.ssid));
                    strlcpy(m.pass, wifimgr::kUseSaved, sizeof(m.pass));
                    postNet(m);
                }
                else if (strcmp(line, "wifi") == 0)
                {
                    hmi::showPage(3);
                    postNet(NetMsg{NetCmd::Scan, {}, {}});
                }
                else if (strncmp(line, "theme ", 6) == 0)
                {
                    hmi::setTheme(atoi(line + 6));
                }
                else if (strncmp(line, "sensor ", 7) == 0)
                {
                    AppMsg m{AppCmd::SensorPreview, 0, false, {}, {}};
                    strlcpy(m.text, line + 7, sizeof(m.text));
                    postApp(m);
                }
                else if (strncmp(line, "ac ", 3) == 0)
                {
                    // Cú pháp Serial test: "ac <id> <pwr> [temp] [fan] [swing]"
                    int id = 1, pwr = 1, temp = -1, fan = -1, swing = -1;
                    int n = sscanf(line + 3, "%d %d %d %d %d", &id, &pwr, &temp, &fan, &swing);
                    if (n >= 2 && id >= 1 && id <= hmi::kAcCount)
                    {
                        const uint8_t idx = static_cast<uint8_t>(id - 1);
                        if (n >= 3)
                        {
                            postApp(AppMsg{AppCmd::AcSetFull, idx, pwr != 0,
                                           {static_cast<float>(temp), static_cast<float>(fan), static_cast<float>(swing)}, {}});
                        }
                        else
                        {
                            postApp(AppMsg{AppCmd::AcSetPower, idx, pwr != 0, {}, {}});
                        }
                    }
                }
                else
                {
                    Serial.printf("? %s\n", line);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }

    // ===================== HMI callbacks (run in the ui task: post only) =====================

    void onRelayToggle(uint8_t i) { postApp(AppMsg{AppCmd::RelayToggle, i, false, {}, {}}); }
    void onAcToggle(uint8_t i, bool on) { postApp(AppMsg{AppCmd::AcSetPower, i, on, {}, {}}); }
    void onAcTemp(uint8_t i, uint8_t rawTemp)
    {
        const float tempC = static_cast<float>(rawTemp) / 2.0f; // slider 36..64 -> 18.0..32.0°C
        postApp(AppMsg{AppCmd::AcSetTemp, i, false, {tempC, 0, 0}, {}});
    }
    void onAcFan(uint8_t i, uint8_t fan)
    {
        postApp(AppMsg{AppCmd::AcSetFan, i, false, {static_cast<float>(fan), 0, 0}, {}});
    }
    void onAcSwing(uint8_t i, uint8_t swing)
    {
        postApp(AppMsg{AppCmd::AcSetSwing, i, swing != 0, {}, {}});
    }
    void onBrightnessChanged(uint8_t pct) { postApp(AppMsg{AppCmd::SaveBrightness, pct, false, {}, {}}); }
    void onAutoDimChanged(bool on) { postApp(AppMsg{AppCmd::SaveAutoDim, 0, on, {}, {}}); }
    void onThemeChanged(uint8_t i) { postApp(AppMsg{AppCmd::SaveTheme, i, false, {}, {}}); }
    void onBacklight(uint8_t pct)
    {
        pct = constrain(pct, 0, 100);
        display.setBrightness(map(pct, 0, 100, HMI_BACKLIGHT_MIN_DUTY, 255));
    }
    void onWifiScan() { postNet(NetMsg{NetCmd::Scan, {}, {}}); }
    void onWifiJoin(const char *ssid, const char *pass)
    {
        NetMsg m{NetCmd::Join, {}, {}};
        strlcpy(m.ssid, ssid, sizeof(m.ssid));
        strlcpy(m.pass, pass, sizeof(m.pass));
        postNet(m);
    }
    void onFirmwareUpdate() { storage::requestFirmwareUpdate(); }
    void onDeviceToggle(uint8_t device, uint8_t channel) { tasmota::toggle(device, channel); }
    void onFileList(const char *path) { storage::requestList(path); }
    void onFileRead(const char *path) { storage::requestRead(path); }
    void onFileDelete(const char *path) { storage::requestDelete(path); }

    uint8_t deviceList(hmi::DeviceInfo *out)
    {
        (void)out;
        return 0;
    }

} // namespace

void setup()
{
    Serial.begin(SERIAL_BAUDRATE);
    delay(100);
    Serial.println("\n\n=== ESP32-4848S040 Smart Home Panel ===");
    Serial.printf("Firmware: %s  Hardware: %s\n", FIRMWARE_VERSION, HARDWARE_VERSION);
    Serial.printf("Reset reason: %d\n", (int)esp_reset_reason());
    g_bootTime = millis();

    for (int i = 0; i < RELAY_COUNT; i++)
    {
        pinMode(kRelayPins[i], OUTPUT);
        digitalWrite(kRelayPins[i], LOW);
        g_relay[i] = false;
        g_relayChange[i] = millis();
    }

    if (!display.begin())
        Serial.println("!!! display.begin() FAILED !!!");
    display.setBrightness(255);

    datalog::begin();

    g_prefs.begin("hmi", false);
    for (uint8_t i = 0; i < hmi::kAcCount; i++)
    {
        char key[12];
        snprintf(key, sizeof(key), "ac_t%u", i);
        uint8_t savedTemp = constrain(g_prefs.getUChar(key, 27), 18, 32);

        snprintf(key, sizeof(key), "ac_f%u", i);
        uint8_t savedFan = constrain(g_prefs.getUChar(key, 1), 0, 6); // Mặc định 1 (Yên tĩnh)

        snprintf(key, sizeof(key), "ac_s%u", i);
        uint8_t savedSwing = g_prefs.getUChar(key, 0) ? 1 : 0; // Mặc định 0 (Off)

        g_acOn[i] = false;
        g_acTemp[i] = savedTemp;
        g_acFan[i] = savedFan;
        g_acSwing[i] = savedSwing;

        hmi::setAc(i, false, static_cast<float>(savedTemp));
        hmi::setAcFanSwing(i, savedFan, savedSwing);
    }
    hmi::setBrightness(g_prefs.getUChar("bright", 80));
    hmi::setAutoDim(g_prefs.getBool("autodim", true));
    hmi::setTheme(g_prefs.getUChar("theme", 0));

    esp_task_wdt_init(10, true);

    g_appQ = xQueueCreate(16, sizeof(AppMsg));
    g_netQ = xQueueCreate(4, sizeof(NetMsg));

    hmi::Callbacks cb;
    cb.relayToggle = onRelayToggle;
    cb.acToggle = onAcToggle;
    cb.acTemp = onAcTemp;
    cb.acFan = onAcFan;
    cb.acSwing = onAcSwing;
    cb.brightnessChanged = onBrightnessChanged;
    cb.autoDimChanged = onAutoDimChanged;
    cb.themeChanged = onThemeChanged;
    cb.backlight = onBacklight;
    cb.wifiScan = onWifiScan;
    cb.wifiJoin = onWifiJoin;
    cb.firmwareUpdate = onFirmwareUpdate;
    cb.deviceToggle = onDeviceToggle;
    cb.fileList = onFileList;
    cb.fileRead = onFileRead;
    cb.fileDelete = onFileDelete;

    static hmi::DeviceInfo devices[hmi::kMaxDevices];
    const uint8_t deviceCount = deviceList(devices);
    hmi::configureDevices(devices, deviceCount);
    hmi::begin(display, cb);

    xTaskCreatePinnedToCore(appTask, "app", 4 * 1024, nullptr, 4, nullptr, 1);
    xTaskCreatePinnedToCore(netTask, "net", 8 * 1024, nullptr, 2, nullptr, 0);
    xTaskCreatePinnedToCore(consoleTask, "console", 4 * 1024, nullptr, 1, nullptr, 0);
    Serial.println("Tasks started: ui app net devices storage console");
}

void loop() { vTaskDelete(nullptr); }