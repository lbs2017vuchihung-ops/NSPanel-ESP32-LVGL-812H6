#pragma once
#include <Arduino.h>

constexpr uint8_t kMaxSlaves = 6; // Hỗ trợ mở rộng tối đa 6 mạch ESP32 Slave (Node 1..6)

enum EspNowMsgType : uint8_t
{
    MSG_SENSOR_DATA = 1, // Slave -> NSPanel: Gửi cảm biến / nhịp tim (Heartbeat)
    MSG_AC_CMD = 2,      // NSPanel -> Slave: Lệnh điều khiển theo node_id / ac_id
    MSG_PING_ACK = 3     // NSPanel -> Slave: Báo xác nhận kênh Wi-Fi
};

typedef struct __attribute__((packed))
{
    uint8_t msg_type;
    uint8_t node_id;   // Định danh mạch Slave: 1, 2, 3...
    float temperature; // °C (Gửi NAN nếu node đó không gắn cảm biến)
    float humidity;    // %
    float power_w;     // W
    uint8_t ac_id;     // Máy lạnh số 1, 2, 3
    bool ac_power;     // true = ON, false = OFF
    uint8_t ac_temp;   // 16..30 °C
    uint8_t ac_mode;   // 0=Auto, 1=Cool, 2=Dry, 3=Fan
    uint8_t ac_fan;    // 0=Auto, 1=Low, 2=Med, 3=High
} EspNowPacket;

typedef void (*SensorDataCallback)(uint8_t nodeId, float temp, float hum, float power);

void espnowInit(SensorDataCallback onSensorCb);
// Gửi lệnh điều khiển đích danh tới mạch Slave phụ trách máy lạnh acId (Mặc định acId=1 gửi tới Node 1, acId=2 -> Node 2...)
bool espnowSendAcCommand(uint8_t acId, bool power, uint8_t temp, uint8_t mode = 1, uint8_t fan = 0);
bool espnowIsNodeOnline(uint8_t nodeId);
void espnowPrintNodes(Print &out); // Gõ lệnh "nodes" trên Terminal để xem danh sách các Slave đang kết nối