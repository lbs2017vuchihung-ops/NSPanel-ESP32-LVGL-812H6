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
static SlaveRecord s_nodes[kMaxSlaves + 1]; // Chỉ số mảng 1..kMaxSlaves khớp trực tiếp với node_id

static void registerOrUpdatePeer(const uint8_t *mac)
{
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, mac, 6);
    peerInfo.channel = 0; // Theo kênh hiện tại của Wi-Fi STA
    peerInfo.encrypt = false;

    if (esp_now_is_peer_exist(mac))
        esp_now_mod_peer(&peerInfo);
    else
        esp_now_add_peer(&peerInfo);
}

static void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    (void)mac_addr;
    s_txAcked = (status == ESP_NOW_SEND_SUCCESS);
    s_txDone = true;
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    const uint8_t *mac = info->src_addr;
    const uint8_t *dst = info->des_addr;
#else
static void onDataRecv(const uint8_t *mac, const uint8_t *data, int len)
{
    const uint8_t *dst = nullptr;
#endif
    if (len != sizeof(EspNowPacket))
        return;

    EspNowPacket pkt;
    memcpy(&pkt, data, sizeof(EspNowPacket));

    if (pkt.node_id == 0 || pkt.node_id > kMaxSlaves)
        return;

    // Lưu địa chỉ MAC vào đúng ngăn của node_id (Node 1, Node 2 không đè lên nhau)
    SlaveRecord &node = s_nodes[pkt.node_id];
    if (!node.paired || memcmp(node.mac, mac, 6) != 0)
    {
        memcpy(node.mac, mac, 6);
        registerOrUpdatePeer(node.mac);
        node.paired = true;
        Serial.printf("[ESP-NOW] Paired Node #%u -> MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                      pkt.node_id, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
    node.lastSeenMs = millis();

    if (pkt.msg_type == MSG_SENSOR_DATA)
    {
        bool isBroadcast = (dst == nullptr) || (dst[0] == 0xFF && dst[1] == 0xFF);
        if (isBroadcast)
        {
            EspNowPacket ack = {};
            ack.msg_type = MSG_PING_ACK;
            ack.node_id = pkt.node_id;
            esp_now_send(node.mac, reinterpret_cast<const uint8_t *>(&ack), sizeof(ack));
        }

        // Chỉ đẩy số liệu lên đồng hồ chính của NSPanel nếu là Node 1 và có dữ liệu thật (khác NAN)
        if (pkt.node_id == 1 && !isnan(pkt.temperature) && s_sensorCb)
        {
            s_sensorCb(pkt.node_id, pkt.temperature, pkt.humidity, pkt.power_w);
        }

        // Tạo gói phản hồi PING_ACK gửi ngược lại cho Slave
        EspNowPacket ack = {};
        ack.msg_type = MSG_PING_ACK;
        ack.node_id = pkt.node_id;
        ack.traffic_state = roomCtrlIsMasterBreakerOn() ? 3
                          : roomCtrlIsLightOn()         ? 2
                          : roomCtrlIsYellowLightOn()   ? 1
                          : 0;

        esp_now_send(mac, reinterpret_cast<const uint8_t *>(&ack), sizeof(ack));
    }
}

static bool sendPacketWithAck(const uint8_t *mac, const EspNowPacket &pkt, uint8_t maxRetries = 10)
{
    for (uint8_t attempt = 1; attempt <= maxRetries; attempt++)
    {
        s_txDone = false;
        s_txAcked = false;

        if (esp_now_send(mac, reinterpret_cast<const uint8_t *>(&pkt), sizeof(pkt)) == ESP_OK)
        {
            uint32_t t0 = millis();
            while (!s_txDone && (millis() - t0 < 35))
            {
                vTaskDelay(pdMS_TO_TICKS(2));
            }
            if (s_txAcked)
                return true; // Slave đã nhận gói tin và trả ACK phần cứng
        }

        // Giãn 200ms trước khi gửi lại để vượt qua nhịp quét Wi-Fi nếu đang bị lệch kênh
        if (attempt < maxRetries)
            vTaskDelay(pdMS_TO_TICKS(200));
    }
    return false;
}

void espnowInit(SensorDataCallback onSensorCb)
{
    s_sensorCb = onSensorCb;
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("[ESP-NOW] Init FAILED!");
        return;
    }
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);
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
            uint32_t ago = (millis() - s_nodes[i].lastSeenMs) / 1000;
            out.printf("  Node #%u | MAC %02X:%02X:%02X:%02X:%02X:%02X | %s (cach day %lus)\n",
                       i, s_nodes[i].mac[0], s_nodes[i].mac[1], s_nodes[i].mac[2],
                       s_nodes[i].mac[3], s_nodes[i].mac[4], s_nodes[i].mac[5],
                       espnowIsNodeOnline(i) ? "ONLINE" : "OFFLINE", (unsigned long)ago);
        }
    }
}