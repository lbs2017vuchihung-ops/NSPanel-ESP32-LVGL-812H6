#pragma once

// 1. PINOUT PHAN CUNG NSPANEL PHONG 812H6 (Vu Chi Hung)
#define PIN_RELAY_1         22
#define PIN_RELAY_2         19
#define PIN_BUZZER          21

// 2. MQTT TOPICS TREN SERVER CORE IOT (Truong Dinh Huan)
#define TOPIC_TELEMETRY     "v1/devices/me/telemetry"
#define TOPIC_ATTRIBUTES    "v1/devices/me/attributes"
#define TOPIC_RPC_REQUEST   "v1/devices/me/rpc/request/+"
#define TOPIC_RPC_RESPONSE  "v1/devices/me/rpc/response/"
