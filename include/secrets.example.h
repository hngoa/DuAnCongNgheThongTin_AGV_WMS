#pragma once
/*
 * ===================================================================================
 * MẪU CẤU HÌNH BẢO MẬT RIÊNG TƯ CHO ESP32-S3 (SECRETS CONFIGURATION TEMPLATE)
 * Hướng dẫn: Sao chép file này thành `include/secrets.h` (đã được .gitignore bảo vệ)
 * để cấu hình mật khẩu Wi-Fi và khóa bảo mật mà không đẩy thông tin nhạy cảm lên Git.
 * ===================================================================================
 */

#define WIFI_SSID_CFG        "LAU 3"
#define WIFI_PASS_CFG        "777777777"
#define MDNS_HOSTNAME_CFG    "agv-wms"
#define AGV_DEVICE_TOKEN_CFG "agv_sec_robot_v2_7f8a9b"
#define AGV_HMAC_SECRET_CFG  "agv_hmac_shared_key_v2_9d8e7f6a5b4c"
