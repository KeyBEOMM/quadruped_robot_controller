#pragma once

// ============================================================
// CommTask.h  —  Phase 2: Core 0 통신 인프라
// ============================================================
// 역할:
//   1. WiFi STA 모드로 초기화하고 호스트 네트워크에 접속한다.
//   2. UDP 소켓을 열고 명령 패킷을 블로킹 수신한다.
//   3. 수신된 패킷을 RobotCommand로 파싱하고 esp_timer 기반 타임스탬프를 부여한다.
//   4. Mutex로 보호된 SharedData 영역에 파싱 결과를 기록한다.
//   5. "reset" 명령을 수신한 경우 ERROR -> INIT 상태 전이를 트리거한다.
//
// UDP 패킷 포맷 (호스트 → ESP32, 고정 28바이트 리틀엔디언):
//   [0..3]   float  vx    (전진 속도, m/s)
//   [4..7]   float  vy    (측면 속도, m/s)
//   [8..11]  float  wz    (회전 속도, rad/s)
//   [12..15] float  roll  (몸통 롤,  rad)
//   [16..19] float  pitch (몸통 피치, rad)
//   [20..23] float  yaw   (몸통 요,  rad)
//   [24]     uint8  flags  bit0 = reset_requested (1이면 리셋 요청)
//   [25..27] uint8  padding (무시)
//
// 멀티바이트 필드는 모두 리틀엔디언 (x86/ARM 호스트 기본값과 동일).
// ============================================================

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
#include "lwip/netdb.h"

#include "quadruped_types.h"

// ============================================================
// 네트워크 설정
// ============================================================
// WIFI_SSID / WIFI_PASSWORD는 소스 코드에 직접 쓰지 않는다.
// platformio.ini의 build_flags를 통해 secrets.ini에서 주입된다.
// 설정 방법: secrets.ini.example 파일을 secrets.ini로 복사 후 실제 값 입력
// ============================================================
#ifndef WIFI_SSID
  #error "WIFI_SSID is not defined! Copy secrets.ini.example to secrets.ini and fill in your credentials."
#endif
#ifndef WIFI_PASSWORD
  #error "WIFI_PASSWORD is not defined! Copy secrets.ini.example to secrets.ini and fill in your credentials."
#endif
#define UDP_PORT       9870              // ESP32가 수신 대기할 UDP 포트
#define UDP_PACKET_LEN 28               // 패킷 고정 길이 (바이트)

static const char* TAG_COMM = "CommTask";

// ============================================================
// 패킷 파싱 헬퍼 함수
// ============================================================

// strict-aliasing 미정의 동작(UB)을 방지하기 위해
// 포인터 캐스팅 대신 memcpy로 float을 안전하게 꺼낸다.
static inline float bytes_to_float(const uint8_t* buf, int offset) {
    float v;
    memcpy(&v, buf + offset, sizeof(float));
    return v;
}

// 수신 버퍼를 RobotCommand로 파싱한다.
// 패킷 길이가 부족하면 false를 반환하고 해당 패킷을 버린다.
static bool parsePacket(const uint8_t* buf, int len, RobotCommand& out_cmd, bool& out_reset) {
    if (len < UDP_PACKET_LEN) {
        return false;   // 짧은 패킷 — 무효 처리
    }
    out_cmd.vx    = bytes_to_float(buf,  0);
    out_cmd.vy    = bytes_to_float(buf,  4);
    out_cmd.wz    = bytes_to_float(buf,  8);
    out_cmd.roll  = bytes_to_float(buf, 12);
    out_cmd.pitch = bytes_to_float(buf, 16);
    out_cmd.yaw   = bytes_to_float(buf, 20);
    out_reset = (buf[24] & 0x01) != 0;  // flags 최하위 비트 = 리셋 요청
    return true;
}

// ============================================================
// WiFi 이벤트 핸들러
// 최소 구현: 연결/해제/IP 획득 이벤트만 로그 출력
// ============================================================
static void wifiEventHandler(void* arg, esp_event_base_t base,
                             int32_t id, void* data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        // STA 모드 시작 → 즉시 접속 시도
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        // 연결 끊어짐 → 자동 재접속
        ESP_LOGW(TAG_COMM, "WiFi disconnected — retrying...");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        // IP 할당 완료 → 할당받은 IP 출력
        ip_event_got_ip_t* ev = (ip_event_got_ip_t*)data;
        ESP_LOGI(TAG_COMM, "WiFi connected. IP: " IPSTR, IP2STR(&ev->ip_info.ip));
    }
}

// ============================================================
// WiFi STA 초기화
// ============================================================
static void wifiInit() {
    // WiFi 드라이버 내부적으로 NVS(비휘발성 저장소)를 사용하므로 반드시 먼저 초기화
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS 파티션이 꽉 찼거나 버전이 맞지 않으면 초기화 후 재시도
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_ERROR_CHECK(esp_netif_init());                  // TCP/IP 스택 초기화
    ESP_ERROR_CHECK(esp_event_loop_create_default());   // 이벤트 루프 생성
    esp_netif_create_default_wifi_sta();                // STA 인터페이스 생성

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // WiFi 이벤트 및 IP 이벤트 핸들러 등록
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID,     &wifiEventHandler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT,   IP_EVENT_STA_GOT_IP, &wifiEventHandler, NULL, NULL));

    // SSID / 비밀번호 설정
    wifi_config_t wifi_cfg = {};
    strncpy((char*)wifi_cfg.sta.ssid,     WIFI_SSID,     sizeof(wifi_cfg.sta.ssid)     - 1);
    strncpy((char*)wifi_cfg.sta.password, WIFI_PASSWORD, sizeof(wifi_cfg.sta.password) - 1);
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;  // WPA2 이상만 허용

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());  // 시작 → WIFI_EVENT_STA_START 발생 → 핸들러에서 connect()

    ESP_LOGI(TAG_COMM, "WiFi init done. Connecting to SSID: %s", WIFI_SSID);
}

// ============================================================
// CommTask 파라미터 구조체
// ============================================================
// main.cpp에서 할당하여 xTaskCreatePinnedToCore()의 pvParameters로 전달한다.
// sm_ptr: StateMachine*를 void*로 보관 — 헤더 순환 포함 방지
// ============================================================
struct CommTaskParams {
    SharedData*       shared;   // Core 0이 쓰고 Core 1이 읽는 공유 데이터 cmd, time, flag 포함한 데이터
    SemaphoreHandle_t mutex;    // shared 보호용 Mutex  // SemaphoreHandle_t와 뮤텍스가 무엇인지 (freertos에서 여러 테스크 간의 동기화와 공유 자원 보호를 위한 동기화 도구, 세마포어는 테스크 간의 동기화 제공, 다수 테스크가 자원에 접근할 수있는지 관리, 뮤텍스는 상보 배제를 통해 한번해 하나의 테스크만 특져ㅓㅇ 자원에 접근할 수 있도록함)
                                // xSemaphoreCreateMutex로 뮤텍스 생성, xSemaphoreTake로 뮤텍스 획득, xSemaphoreGive로 뮤텍스 반환
    void*             sm_ptr;   // StateMachine* (void* 캐스트)
};

// StateMachine을 포함하는 위치는 구조체 선언 이후 — 순서 주의
#include "StateMachine.h"

// ============================================================
// CommTask 본체 (무한 수신 루프)
// ============================================================
// pvParameters: CommTaskParams* 포인터 // main에서 g_comm_params의 주소를 받음 p = &g_comm_params
// ============================================================
inline void commTaskRun(void* pvParameters) {
    CommTaskParams*   p      = static_cast<CommTaskParams*>(pvParameters); // casting이 무엇인지, 왜 하는지 왜 void 타입인지
    SharedData*       shared = p->shared;
    SemaphoreHandle_t mutex  = p->mutex;
    StateMachine*     sm     = static_cast<StateMachine*>(p->sm_ptr);

    ESP_LOGI(TAG_COMM, "Core 0: CommTask started (core: %d)", xPortGetCoreID());

    // --- WiFi 초기화 ---
    wifiInit();

    // --- UDP 소켓 생성 ---
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP); // 이게 뭔지
    if (sock < 0) {
        ESP_LOGE(TAG_COMM, "socket() failed: errno %d", errno);
        vTaskDelete(NULL); // 이거 뭔지
        return;
    }

    // --- 로컬 포트에 바인딩 ---
    struct sockaddr_in local_addr = {};
    local_addr.sin_family      = AF_INET;
    local_addr.sin_addr.s_addr = INADDR_ANY;  // 모든 인터페이스 수신
    local_addr.sin_port        = htons(UDP_PORT);

    if (bind(sock, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
        ESP_LOGE(TAG_COMM, "bind() failed: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG_COMM, "UDP socket bound. Listening on port %d", UDP_PORT);

    // 수신 버퍼: 고정 패킷(28B) + 오버사이즈 패킷 가드용 여분 4B
    uint8_t rx_buf[UDP_PACKET_LEN + 4];
    struct sockaddr_in sender_addr;
    socklen_t sender_len = sizeof(sender_addr);

    // ============================================================
    // 메인 수신 루프 — recvfrom은 패킷이 올 때까지 블로킹
    // ============================================================
    while (true) {
        int received = recvfrom(sock, rx_buf, sizeof(rx_buf), 0,
                                (struct sockaddr*)&sender_addr, &sender_len);

        if (received < 0) {
            // 일시적 에러는 무시하고 계속 수신 (소켓 닫힘 등 치명적 에러는 추후 처리)
            ESP_LOGW(TAG_COMM, "recvfrom() error: errno %d", errno);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // --- 패킷 파싱 ---
        RobotCommand cmd; // mutex 걸린 
        bool reset_requested = false;
        if (!parsePacket(rx_buf, received, cmd, reset_requested)) {
            ESP_LOGW(TAG_COMM, "Malformed packet (%d bytes) — discarded", received);
            continue;
        }

        // --- 수신 타임스탬프 부여 (마이크로초 → 밀리초 변환) ---
        // esp_timer_get_time()은 부팅 이후 경과 마이크로초를 반환한다.
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

        // --- Mutex 획득 → SharedData 갱신 → Mutex 해제 ---
        // 락을 쥐고 있는 시간을 최소화하기 위해 연산은 락 밖에서 수행했다.
        // (이미 파싱이 완료된 cmd를 구조체 복사만 한다)
        if (xSemaphoreTake(mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            shared->cmd             = cmd;
            shared->timestamp_ms    = now_ms;
            shared->reset_requested = reset_requested;
            xSemaphoreGive(mutex);
        } else {
            // 5ms 이내에 Mutex를 얻지 못하면 패킷을 버린다.
            // Core 1이 락을 과도하게 오래 쥐고 있다면 Phase 3에서 조사 필요.
            ESP_LOGW(TAG_COMM, "Mutex timeout — packet dropped");
        }

        // --- 리셋 명령 처리 ---
        // StateMachine::onResetCommand()는 ERROR 상태가 아니면 자체적으로 무시한다.
        if (reset_requested) {
            ESP_LOGI(TAG_COMM, "Reset command received — triggering INIT transition");
            sm->onResetCommand();
        }

        // --- 디버그 로그 (배포 시 ESP_LOGD 또는 #ifdef DEBUG로 비활성화) ---
        ESP_LOGD(TAG_COMM,
                 "RX vx=%.3f vy=%.3f wz=%.3f | roll=%.3f pitch=%.3f yaw=%.3f | reset=%d | t=%lu ms",
                 cmd.vx, cmd.vy, cmd.wz, cmd.roll, cmd.pitch, cmd.yaw,
                 (int)reset_requested, (unsigned long)now_ms);
    }
}
