# NSPanel Smart Home Control Panel (ESP32 + LVGL + CORE IOT) — Lab 812H6

> **Project overall:** Hiện thực màn hình điều khiển nhà thông minh NSPanel sử dụng chip **ESP32** và thư viện đồ họa **LVGL**, tích hợp điều khiển hai chiều từ xa qua **Server CORE IOT** và triển khai thực tế tại **Phòng Lab 812H6**.

---

## Mục tiêu Dự án
1. **Làm chủ quy trình phát triển Firmware & GUI:** Xây dựng giao diện cảm ứng mượt mà trên nền tảng ESP32 và thư viện đồ họa **LVGL**.
2. **Ứng dụng AI trong phát triển nhúng (AI-Driven Development):** Sử dụng công cụ AI để thiết kế bố cục giao diện LVGL và tối ưu hóa mã nguồn điều khiển.
3. **Đồng bộ Đám mây 2 chiều:** Kết nối và điều khiển thiết bị (Đèn, Máy lạnh) thời gian thực thông qua **Server CORE IOT** bằng giao thức MQTT/RPC.
4. **Triển khai Thực tế:** Lắp đặt và vận hành ổn định tại **Lab 812H6**, kèm bộ tài liệu **Quick Start Guide** và mã nguồn mở chuẩn hóa.

---

## Thành viên Thực hiện & Phân công

| Thành viên | Vai trò chính | Module phụ trách (`src/`) |
| :--- | :--- | :--- |
| **Thành viên 1** | Team Leader & Hardware/Driver | Khảo sát 812H6, BOM, Relay/Sensor Drivers (`src/drivers/`, `hardware/`) |
| **Thành viên 2** | UI/UX & AI-LVGL Developer | Thiết kế giao diện LVGL & AI Prompting (`src/ui/`, `ui_design/`) |
| **Thành viên 3** | IoT Network & Cloud Engineer | Giao thức MQTT, JSON Parser & Đồng bộ CORE IOT (`src/network/`) |
| **Thành viên 4** | System Integration, QA & Docs | Tích hợp hệ thống tại 812H6, Kiểm thử & Tài liệu (`docs/`) |

* **Người hướng dẫn (Mentor):** Anh Lê Bảo Long
* **Địa điểm triển khai:** Phòng Lab **812H6**

---

## Kiến trúc Thư mục Mã nguồn

```text
├── docs/                  # Tài liệu Quick Start, báo cáo & nhật ký AI Prompts
├── hardware/              # Sơ đồ chân Pinout NSPanel & firmware node hồng ngoại (IR)
├── ui_design/             # Dự án thiết kế giao diện gốc (SquareLine Studio / Assets)
├── src/
│   ├── main.cpp           # Khởi tạo hệ thống & quản lý tác vụ đa luồng FreeRTOS
│   ├── include/           # File cấu hình chân GPIO, MQTT Topics & Wi-Fi/Token
│   ├── ui/                # Mã nguồn giao diện LVGL và xử lý sự kiện cảm ứng (Touch Callbacks)
│   ├── network/           # Module kết nối Wi-Fi, MQTT Client và đồng bộ Server CORE IOT
│   └── drivers/           # Driver điều khiển 2 kênh Relay nội bộ, cảm biến nhiệt độ & còi báo
└── platformio.ini         # Cấu hình môi trường biên dịch PlatformIO và thư viện phụ thuộc