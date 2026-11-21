#include "espnow_manager.h"
#include "smart_room_ctrl.h"

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>
#include <math.h>

static volatile bool s_txDone = false;
static volatile bool s_txAcked = false;

struct SlaveRecord
{
    bool paired = false;     // Kiểm tra đã từng ghép đôi chưa
    uint8_t mac[6] = {0};    // Địa chỉ MAC phần cứng 6 byte
    uint32_t lastSeenMs = 0; // Mốc thời gian millis() cuối cùng nhận được gói tin
};

static SensorDataCallback s_sensorCb = nullptr;

// Hardcode sẵn 2 địa chỉ MAC của Slave #1 và Slave #2
static SlaveRecord s_nodes[kMaxSlaves + 1] = {
    // Index 0: Không dùng
    {false, {0}, 0},
    // Index 1: Slave #1 (10:20:BA:06:18:00)
    {true, {0x10, 0x20, 0xBA, 0x06, 0x18, 0x00}, 0},
    // Index 2: Slave #2 (10:20:BA:06:57:2C)
    {true, {0x10, 0x20, 0xBA, 0x06, 0x57, 0x2C}, 0}
};

static void registerOrUpdatePeer(const uint8_t *mac)
{
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, mac, 6);
    peerInfo.channel = 0; // Theo kênh hiện tại của Wi-Fi STA
    peerInfo.ifidx = WIFI_IF_STA;
    peerInfo.encrypt = false;

    if (esp_now_is_peer_exist(mac))
        esp_now_mod_peer(&peerInfo);
    else
        esp_now_add_peer(&peerInfo);
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static void onDataSent(const esp_now_tx_info_t *info, esp_now_send_status_t status)
{
    (void)info;
#else
static void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    (void)mac_addr;
#endif
    s_txAcked = (status == ESP_NOW_SEND_SUCCESS);
    s_txDone = true;
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    const uint8_t *mac = info->src_addr;
#else
static void onDataRecv(const uint8_t *mac, const uint8_t *data, int len)
{
#endif
    if (len != sizeof(EspNowPacket))
        return;

    EspNowPacket pkt;
    memcpy(&pkt, data, sizeof(EspNowPacket));

    if (pkt.node_id == 0 || pkt.node_id > kMaxSlaves)
        return;

    // Do đã khóa cứng MAC nên chỉ cần cập nhật mốc thời gian lastSeenMs
    SlaveRecord &node = s_nodes[pkt.node_id];
    node.lastSeenMs = millis();

    if (pkt.msg_type == MSG_SENSOR_DATA)
    {
        // Chỉ đẩy số liệu lên đồng hồ chính của NSPanel nếu là Node 1 và có dữ liệu thật (khác NAN)
        if (pkt.node_id == 1 && !isnan(pkt.temperature) && s_sensorCb)
        {
            s_sensorCb(pkt.node_id, pkt.temperature, pkt.humidity, pkt.power_w);
        }

        // Gửi duy nhất 1 gói PING_ACK kèm trạng thái đèn hiện tại để Slave đồng bộ
        uint8_t primaryChan = 1;
        wifi_second_chan_t secondChan;
        esp_wifi_get_channel(&primaryChan, &secondChan);

        EspNowPacket ack = {};
        ack.msg_type = MSG_PING_ACK;
        ack.node_id = pkt.node_id;
        ack.ac_temp = primaryChan; // Báo chính xác kênh thật của NSPanel cho Slave để tránh khóa nhầm kênh
        ack.traffic_state = roomCtrlIsMasterBreakerOn() ? 3
                        : roomCtrlIsLightOn()         ? 2
                        : roomCtrlIsYellowLightOn()   ? 1
                        : 0;

        esp_now_send(node.mac, reinterpret_cast<const uint8_t *>(&ack), sizeof(ack));
    }
}

// Đã giảm số lần thử lại tối đa xuống còn 3 lần
static bool sendPacketWithAck(const uint8_t *mac, const EspNowPacket &pkt, uint8_t maxRetries = 4)
{
    for (uint8_t attempt = 1; attempt <= maxRetries; attempt++)
    {
        s_txDone = false;
        s_txAcked = false;

        if (esp_now_send(mac, reinterpret_cast<const uint8_t *>(&pkt), sizeof(pkt)) == ESP_OK)
        {
            const uint32_t t0 = millis();
            while (!s_txDone && (millis() - t0 < 40))
            {
                vTaskDelay(pdMS_TO_TICKS(2));
            }
            if (s_txAcked)
                vTaskDelay(pdMS_TO_TICKS(30));
                return true; // Slave đã nhận gói tin và trả ACK phần cứng
        }

        // Nghỉ ngắn tránh đụng độ gói Wi-Fi/MQTT rồi thử lại
        if (attempt < maxRetries)
            vTaskDelay(pdMS_TO_TICKS(20));
    }
    vTaskDelay(pdMS_TO_TICKS(30));
    return false;
}

void espnowInit(SensorDataCallback onSensorCb)
{
    s_sensorCb = onSensorCb;
    WiFi.setSleep(false);
    esp_wifi_set_ps(WIFI_PS_NONE);

    // Tự động kéo radio về Kênh 1 ngay khi Wi-Fi rớt hoặc báo NO_AP_FOUND,
    // và tắt chế độ ngủ radio mỗi khi kết nối lại Router thành công
    WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info)
                 {
        (void)info;
        if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED)
        {
            esp_wifi_set_promiscuous(true);
            esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
            esp_wifi_set_promiscuous(false);

            // Ép công suất phát lên mức tối đa (khoảng 19.5 - 20dBm)
            WiFi.setTxPower(WIFI_POWER_19_5dBm);

            // Kích hoạt chuẩn Long Range kết hợp các chuẩn Wi-Fi cơ bản
            esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);
        }
        else if (event == ARDUINO_EVENT_WIFI_STA_CONNECTED)
        {
            esp_wifi_set_ps(WIFI_PS_NONE);
        } });

    // Khóa mặc định ở Kênh 1 ngay lúc khởi động
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    if (esp_now_init() != ESP_OK)
    {
        Serial.println("[ESP-NOW] Init FAILED!");
        return;
    }
    esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_1M_L);

    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);

    // Tự động đăng ký trước 2 Slave dựa trên mảng MAC cứng đã khai báo
    for (uint8_t i = 1; i <= 2; i++) {
        registerOrUpdatePeer(s_nodes[i].mac);
        s_nodes[i].lastSeenMs = millis(); // Khởi tạo mốc thời gian để tránh bị tính là offline ngay lúc đầu
        Serial.printf("[ESP-NOW] Hardcoded Peer #%u -> %02X:%02X:%02X:%02X:%02X:%02X\n",
            i, s_nodes[i].mac[0], s_nodes[i].mac[1], s_nodes[i].mac[2],
            s_nodes[i].mac[3], s_nodes[i].mac[4], s_nodes[i].mac[5]);
    }

    Serial.printf("[ESP-NOW] Multi-Slave Master Ready | MAC: %s\n", WiFi.macAddress().c_str());
}

bool espnowSendAcCommand(uint8_t acId, bool power, uint8_t temp, uint8_t fan, uint8_t swing)
{
    if (acId < 1 || acId > 3)
        return false;

    // AC #1, #2 -> Slave #1 | AC #3 -> Slave #2
    const uint8_t targetNode = (acId == 3) ? 2 : 1;
    if (!s_nodes[targetNode].paired)
    {
        Serial.printf("[ESP-NOW] Error: Slave #%u chua ket noi!\n", targetNode);
        return false;
    }

    EspNowPacket pkt = {};
    pkt.msg_type = MSG_AC_CMD;
    pkt.node_id = targetNode;
    pkt.ac_id = acId;
    pkt.ac_power = power;
    pkt.ac_temp = temp;
    pkt.ac_fan = fan;
    pkt.ac_swing = swing;

    const bool ok = sendPacketWithAck(s_nodes[targetNode].mac, pkt);
    Serial.printf("[ESP-NOW] TX -> Node #%u (AC#%u: %s, %uC, Fan:%u, Swing:%u) : %s\n",
                  targetNode, acId, power ? "ON" : "OFF", temp, fan, swing,
                  ok ? "ACK OK" : "NO ACK (FAIL)");
    return ok;
}

bool espnowSendTrafficCommand(uint8_t state)
{
    const uint8_t targetNode = 1;
    if (!s_nodes[targetNode].paired)
    {
        Serial.printf("[ESP-NOW] Error: Slave #1 chua ket noi!\n");
        return false;
    }

    EspNowPacket pkt = {};
    pkt.msg_type = MSG_TRAFFIC_CMD;
    pkt.node_id = targetNode;
    pkt.traffic_state = state & 0x03;

    const bool ok = sendPacketWithAck(s_nodes[targetNode].mac, pkt);
    Serial.printf("[ESP-NOW] TX -> Node #1 (Traffic LED AB=%u%u) : %s\n",
                  (pkt.traffic_state >> 1) & 1,
                  pkt.traffic_state & 1,
                  ok ? "ACK OK" : "NO ACK (FAIL)");
    return ok;
}

bool espnowIsNodeOnline(uint8_t nodeId)
{
    if (nodeId == 0 || nodeId > kMaxSlaves)
        return false;
    return s_nodes[nodeId].paired && (millis() - s_nodes[nodeId].lastSeenMs < 15000);
}

void espnowPrintNodes(Print &out)
{
    out.println("=== DANH SACH ESP32 SLAVE ===");
    for (uint8_t i = 1; i <= kMaxSlaves; i++)
    {
        if (s_nodes[i].paired)
        {
            const uint32_t ago = (millis() - s_nodes[i].lastSeenMs) / 1000;
            out.printf("  Node #%u | MAC %02X:%02X:%02X:%02X:%02X:%02X | %s (cach day %lus)\n",
                       i, s_nodes[i].mac[0], s_nodes[i].mac[1], s_nodes[i].mac[2],
                       s_nodes[i].mac[3], s_nodes[i].mac[4], s_nodes[i].mac[5],
                       espnowIsNodeOnline(i) ? "ONLINE" : "OFFLINE", (unsigned long)ago);
        }
    }
}