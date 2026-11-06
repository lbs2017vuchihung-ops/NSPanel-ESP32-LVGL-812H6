#include <Arduino.h>
#include "include/app_config.h"

void setup() {
    Serial.begin(115200);
    Serial.println("[NSPanel 812H6] Booting system...");
    // TODO: Khoi tao Relay Driver (Hung)
    // TODO: Khoi tao Giao dien LVGL (Bao)
    // TODO: Khoi tao Wi-Fi & CORE IOT MQTT (Huan)
}

void loop() {
    // Vong lap xu ly LVGL & MQTT
    delay(5);
}
