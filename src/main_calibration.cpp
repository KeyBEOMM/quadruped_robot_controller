#ifdef CALIBRATION_MODE
// === Calibration Firmware (AP Mode, Lightweight, No Kinematics) ===

// ============================================================
// main_calibration.cpp — Phase 5 Calibration Firmware
// ============================================================
// Standalone calibration firmware for servo motor testing.
// Uses UDP text commands from PC to control servos in real-time.
//
// Build:  pio run -e esp32dev_calibration
// Flash:  pio run -e esp32dev_calibration -t upload
// Monitor: pio device monitor -b 115200
// ============================================================

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"

#include "HardwareOutput.h"

// WiFi credentials (injected from secrets.ini via build_flags)
#ifndef WIFI_SSID
  #error "WIFI_SSID is not defined! Copy secrets.ini.example to secrets.ini and fill in your credentials."
#endif
#ifndef WIFI_PASSWORD
  #error "WIFI_PASSWORD is not defined! Copy secrets.ini.example to secrets.ini and fill in your credentials."
#endif

static const char* TAG = "CALIB";
static const int CALIB_PORT = 9871;

// ============================================================
// WiFi STA Init (simplified, no CommTask dependency)
// ============================================================
// AP 모드이므로 외부 공유기 접속 실패 이벤트를 처리할 필요가 거의 없으므로 간소화합니다.
static void wifi_init_softap() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // AP 용 netif 생성
    esp_netif_t* ap_netif = esp_netif_create_default_wifi_ap();
    
    // AP의 기본 IP를 192.168.4.1로 설정
    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(ap_netif, &ip_info);
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_cfg = {};
    strcpy((char*)wifi_cfg.ap.ssid, "ESP32_CALIB");
    strcpy((char*)wifi_cfg.ap.password, "12345678");
    wifi_cfg.ap.ssid_len = strlen("ESP32_CALIB");
    wifi_cfg.ap.channel = 1;
    wifi_cfg.ap.max_connection = 4;
    wifi_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  WiFi SoftAP Mode Started!");
    ESP_LOGI(TAG, "  SSID: ESP32_CALIB");
    ESP_LOGI(TAG, "  PASS: 12345678");
    ESP_LOGI(TAG, "  ESP32 IP Address: 192.168.4.1");
    ESP_LOGI(TAG, "========================================");
}

// ============================================================
// Calibration Task (Core 1)
// ============================================================
void CalibrationTask(void* pvParameters) {
    ESP_LOGI(TAG, "CalibrationTask started (core: %d)", xPortGetCoreID());

    // --- I2C + PCA9685 init ---
    HardwareOutput hw_out;
    if (!hw_out.init()) {
        ESP_LOGE(TAG, "HardwareOutput init FAILED! Check I2C wiring (SDA=21, SCL=22).");
        ESP_LOGE(TAG, "Halting calibration task.");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "HardwareOutput (PCA9685) initialized OK.");

    // Wait for WiFi to connect
    ESP_LOGI(TAG, "Waiting 4s for WiFi connection...");
    vTaskDelay(pdMS_TO_TICKS(4000));

    // --- UDP socket ---
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket() failed: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in addr = {};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(CALIB_PORT);

    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind() failed: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  CALIBRATION MODE READY");
    ESP_LOGI(TAG, "  UDP port: %d", CALIB_PORT);
    ESP_LOGI(TAG, "  Commands: set_angle / set_pwm / get_status / save_offset");
    ESP_LOGI(TAG, "========================================");

    // Per-channel state (max 16 PCA9685 channels)
    float cur_angle_deg[16];
    float cur_pwm_us[16];
    for (int i = 0; i < 16; i++) {
        cur_angle_deg[i] = 90.0f;
        cur_pwm_us[i] = 1500.0f;
    }

    char rx_buf[128];
    struct sockaddr_in sender;
    socklen_t sender_len = sizeof(sender);

    // ============================================================
    // Main command loop
    // ============================================================
    while (true) {
        sender_len = sizeof(sender);
        int n = recvfrom(sock, rx_buf, sizeof(rx_buf) - 1, 0,
                         (struct sockaddr*)&sender, &sender_len);
        if (n <= 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        rx_buf[n] = '\0';

        // Strip newlines
        for (int i = 0; i < n; i++) {
            if (rx_buf[i] == '\n' || rx_buf[i] == '\r') rx_buf[i] = '\0';
        }

        char reply[256] = {0};
        int ch = 0;
        float val = 0.0f;

        // --- set_angle <ch> <deg> ---
        if (sscanf(rx_buf, "set_angle %d %f", &ch, &val) == 2) {
            if (ch < 0 || ch > 15) {
                snprintf(reply, sizeof(reply), "ERR: ch %d out of range (0-15)", ch);
            } else {
                float clamped = val;
                if (clamped < 0.0f) clamped = 0.0f;
                if (clamped > 175.0f) clamped = 175.0f;

                float pulse_us = 500.0f + (clamped / 180.0f) * 2000.0f;
                uint16_t ticks = (uint16_t)((pulse_us / 10000.0f) * 4096.0f + 0.5f);
                hw_out.writeCalibrationDeg(ch, clamped);

                cur_angle_deg[ch] = clamped;
                cur_pwm_us[ch] = pulse_us;

                snprintf(reply, sizeof(reply),
                         "OK servo_%d: angle=%.1fdeg -> pwm=%.0fus (ticks=%d)%s",
                         ch, clamped, pulse_us, ticks,
                         (clamped != val) ? " [CLAMPED]" : "");
            }
        }
        // --- set_pwm <ch> <usec> (raw) ---
        else if (sscanf(rx_buf, "set_pwm %d %f", &ch, &val) == 2) {
            if (ch < 0 || ch > 15) {
                snprintf(reply, sizeof(reply), "ERR: ch %d out of range (0-15)", ch);
            } else {
                uint16_t ticks = (uint16_t)((val / 10000.0f) * 4096.0f + 0.5f);
                float equiv_deg = ((val - 500.0f) / 2000.0f) * 180.0f;
                hw_out.writeCalibrationDeg(ch, equiv_deg);

                cur_pwm_us[ch] = val;
                cur_angle_deg[ch] = equiv_deg;

                snprintf(reply, sizeof(reply),
                         "OK servo_%d: pwm=%.0fus (ticks=%d, equiv=%.1fdeg) [RAW]",
                         ch, val, ticks, equiv_deg);
            }
        }
        // --- get_status ---
        else if (strncmp(rx_buf, "get_status", 10) == 0) {
            int off = 0;
            off += snprintf(reply + off, sizeof(reply) - off, "STATUS:\n");
            for (int i = 0; i < 12; i++) {
                int leg = i / 3;
                int jnt = i % 3;
                const char* lname[] = {"LF", "RF", "LH", "RH"};
                const char* jname[] = {"HAA", "HFE", "KNE"};
                off += snprintf(reply + off, sizeof(reply) - off,
                                "  CH%02d (%s_%s): %.1fdeg / %.0fus\n",
                                i, lname[leg], jname[jnt],
                                cur_angle_deg[i], cur_pwm_us[i]);
            }
        }
        // --- save_offset <ch> <usec> ---
        else if (sscanf(rx_buf, "save_offset %d %f", &ch, &val) == 2) {
            float nominal_us = 1500.0f;  // 90deg nominal
            float delta_us = val - nominal_us;
            float delta_deg = (delta_us / 2000.0f) * 180.0f;

            snprintf(reply, sizeof(reply),
                     "OFFSET servo_%d: measured=%.0fus nominal=%.0fus delta=%+.0fus (%+.1fdeg)",
                     ch, val, nominal_us, delta_us, delta_deg);

            ESP_LOGI(TAG, "save_offset ch=%d measured=%.0fus delta=%.1fdeg",
                     ch, val, delta_deg);
        }
        // --- set_board_freq <freq> ---
        else if (strncmp(rx_buf, "set_board_freq", 14) == 0) {
            uint32_t bfreq = 25000000;
            if (sscanf(rx_buf + 14, "%lu", &bfreq) == 1) {
                hw_out.setBoardFreq(bfreq);
                snprintf(reply, sizeof(reply), "FREQ: Board oscillator updated to %lu Hz", bfreq);
            } else {
                snprintf(reply, sizeof(reply), "ERR: Invalid board freq format");
            }
        }
        // --- unknown ---
        else {
            snprintf(reply, sizeof(reply),
                     "ERR: Unknown '%s' | "
                     "Usage: set_angle <ch> <deg> | set_pwm <ch> <us> | get_status | save_offset <ch> <us> | set_board_freq <us>",
                     rx_buf);
        }

        // Reply to sender
        sendto(sock, reply, strlen(reply), 0,
               (struct sockaddr*)&sender, sender_len);

        // Serial echo
        ESP_LOGI(TAG, "[CMD] %s -> %s", rx_buf, reply);
    }
}

// ============================================================
// Entry point
// ============================================================
extern "C" void app_main(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  Phase 5 Calibration Firmware");
    ESP_LOGI(TAG, "  Locomotion pipeline: DISABLED");
    ESP_LOGI(TAG, "========================================");

    // WiFi init
    wifi_init_softap();

    // Single task — calibration only (stack 8192 is sufficient: no Eigen objects)
    xTaskCreatePinnedToCore(
        CalibrationTask, "CalibTask", 8192, NULL, 2, NULL, 1);

    ESP_LOGI(TAG, "CalibrationTask created. Waiting for UDP commands...");
}

#endif // CALIBRATION_MODE
