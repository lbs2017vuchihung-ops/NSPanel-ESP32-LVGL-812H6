#include "espnow_manager.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>
#include <math.h>

struct SlaveRecord
{
    bool paired = false;     // check đã từng ghép đôi chưa
    uint8_t mac[6] = {0};    // Địa chỉ mac phần cứng 6byte
    uint32_t lastSeenMs = 0; // Mốc thời gian millis() cuối cùng nhận được từ gói tin này
};

static SensorDataCallback s_sensorCb = nullptr;
static SlaveRecord s_nodes[kMaxSlaves + 1]; // Chỉ số mảng từ 1..kMaxSlaves khớp trực tiếp với node_id

static void registerOrUpdatePeer(const uint8_t *mac)
{
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, mac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_is_peer_exist(mac))
        esp_now_mod_peer(&peerInfo);
    else
        esp_now_add_peer(&peerInfo);
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

    // Lưu địa chỉ MAC vào đúng ngăn của node_id (Node 1, Node 2, Node 3 không đè lên nhau)
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
            esp_now_send(node.mac, reinterpret_cast<const uint8_t *>(&ack), sizeof(ack));
        }

        // Chỉ đẩy số liệu lên đồng hồ chính của NSPanel nếu là Node 1 (hoặc Node có gắn cảm biến thật, khác NAN)
        if (pkt.node_id == 1 && !isnan(pkt.temperature) && s_sensorCb)
        {
            s_sensorCb(pkt.node_id, pkt.temperature, pkt.humidity, pkt.power_w);
        }
    }
}

/////////////////////////////// Cơ chế hoạt động //////////////////////////////|
// - Mục tiêu hàm: kích hoạt giao thức ESP_NOW trên hạ tầng phần cứng có sẵn   |
//                 của ESP32-S3 -> đồng thời thiết lập cầu nối dữ liệu an toàn |
//                 giữa các tác vụ mạng ở Core 0 và đồ họa interface ở COre 1  |
// - Wifi.mode(WIFI_STA): Bắt buộc phải có trước espnowInit() -> vì ESP32 không|
//                        có chip radio riêng cho ESP_NOW mà sẽ dùng chung tầng|
//                        phần cứng MAC 802.11 của Wifi và anten 2.4GHz -> khi |
//                        gọi hàm wifi này mới kích hoạt tầng phần cứng ở dưới |
//                        => từ đó mới kích hoạt ESP_now qua các hàm init và   |
//                        esp_wifi_start().                                    |
// - esp_now_init(): -> CV1: Cấp phát bảng quản lý PEER trong sram nội: tạo    |
//                           vùng nhớ quản lý danh sách các thiết bị kết nối   |
//                           ESP_NOW (tối đa 20 thiết bị không mã hóa)         |
//                   -> CV2: Dù cấu hình giao diện LVGL đã được đẩy ra 8MB     |
//                           PSRAM -> tuy nhiên vùng SRAM nội vẫn phải chia sẻ |
//                           cho bộ đệm wifi và MQTT => kiểm tra != ESP_OK giúp|
//                           thoát sớm nếu hệ thống bị thiếu RAM để khơi tạo   |
//                           ESPNOW -> tránh gây treo chip.                    |
//                   -> CV3: Đăng ký mã định danh của nhà sản xuất vào bộ lọc  |
//                           phần cứng Wifi -> sau khi thu nhận được gói tin   |
//                           espnow -> đẩy thẳng sang bộ xử lý espnow thay vì  |
//                           xếp vào ngắn xếp mạng TCP/IP.                     |
// - esp_now_register_recv_cb(onDataRecv): Hàng đợi đăng ký onDataRecv vào driver |
//                                         wifi lúc nào có gói tin thì ondatarecv |
//                                         tự động kích hoạt và thêm vào hàng đợi |
//                   -> Hàm callback onDataRecv không chạy trong ngữ cảnh ngắt ISR|
//                      mà chạy trong tác vụ hệ thống FreeRTOS wifi trên core 0   |
//                   -> Trong hàm callback onDataRecv có các hàm như là printf,   |
//                      send, ping_ack (không được delay()).                      |
// - Serial.printf("[ESP-NOW] Multi-Slave Master Ready | MAC: %s\n", WiFi.macAddress().c_str()): |
//             -> in ra địa chỉ Mac byte của master để kiểm tra                                  |
/////////////////////////////////////////////////////////////////////////////////////////////////|

void espnowInit(SensorDataCallback onSensorCb)
{
    s_sensorCb = onSensorCb;
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("[ESP-NOW] Init FAILED!");
        return;
    }
    esp_now_register_recv_cb(onDataRecv);
    Serial.printf("[ESP-NOW] Multi-Slave Master Ready | MAC: %s\n", WiFi.macAddress().c_str());
}

//////////////////////// Cơ chế hoạt động ////////////////////////|
// Hàm chịu trách nhiệm đóng gói lệnh điều khiển máy lạnh từ màn  |
// hình cảm ứng hoặc server core IOT thành khung data 21byte để   |
// gửi đến ESP32 slave.                                           |
//////////////////////////////////////////////////////////////////|
bool espnowSendAcCommand(uint8_t acId, bool power, uint8_t temp, uint8_t mode, uint8_t fan)
{
    // Ánh xạ mặc định: Máy lạnh 1 -> gửi cho Node 1; Máy lạnh 2 -> gửi cho Node 2; Máy lạnh 3 -> gửi cho Node 3
    // (Nếu Node 2 chưa bật, có thể fallback gửi qua Node 1 nếu chung phòng)
    uint8_t targetNode = (acId >= 1 && acId <= kMaxSlaves) ? acId : 1;
    if (!s_nodes[targetNode].paired)
    {
        Serial.printf("[ESP-NOW] Error: Node #%u (phu trach May lanh %u) chua ket noi!\n", targetNode, acId);
        return false;
    }

    EspNowPacket pkt = {};
    pkt.msg_type = MSG_AC_CMD;
    pkt.node_id = targetNode;
    pkt.ac_id = acId;
    pkt.ac_power = power;
    pkt.ac_temp = temp;
    pkt.ac_mode = mode;
    pkt.ac_fan = fan;

    esp_err_t err = esp_now_send(s_nodes[targetNode].mac, reinterpret_cast<const uint8_t *>(&pkt), sizeof(pkt));
    Serial.printf("[ESP-NOW] TX -> Node #%u (AC#%u: %s, %uC) : %s\n",
                  targetNode, acId, power ? "ON" : "OFF", temp, (err == ESP_OK) ? "OK" : "FAIL");
    return (err == ESP_OK);
}

/////////////////////////////////// Cơ chế hoạt động ///////////////////////////////////|
// - Mục tiêu hàm: Giám sát thụ động giúp xác định xem slave còn hoạt động hay là không |
//                 mà không cân tốn băng thông hỏi thăm liên tục.                       |
////////////////////////////////////////////////////////////////////////////////////////|
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