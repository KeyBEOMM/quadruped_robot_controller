#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "quadruped_types.h"
#include "StateMachine.h"
#include "GaitSequencer.h"
#include "FootPlanner.h"
#include "TrajectoryGenerator.h"
#include "BodyKinematics.h"
#include "IK.h"
#include "CommTask.h"   // Phase 2: CommTaskParams 구조체 + commTaskRun()

static const char* TAG = "ROBOT_MAIN";

// ============================================================
// 전역 공유 자원
// ============================================================
// 정적(static) 스토리지에 할당하여 힙 파편화 없이 태스크 간 공유한다.
// ============================================================
static StateMachine      g_sm(RobotState::INIT);  // 글로벌 상태 머신
static SharedData        g_shared;                // Core 0 Write / Core 1 Read 공유 데이터
static SemaphoreHandle_t g_mutex = nullptr;       // g_shared 보호용 Mutex

// ============================================================
// [Phase 1] StateMachine 전이 테스트
//   Phase 1 검증 완료 → #if 0으로 비활성화
// ============================================================
#if 0
static void runStateMachineTest() {
    vTaskDelay(pdMS_TO_TICKS(500)); // 부팅 안정화 대기

    ESP_LOGI(TAG, "====================================");
    ESP_LOGI(TAG, "[Phase 1] StateMachine Transition Test START");
    ESP_LOGI(TAG, "====================================");
    vTaskDelay(pdMS_TO_TICKS(500));

    StateMachine sm(RobotState::INIT);

    // 각 상태 진입 시 로그를 출력하는 콜백 등록
    sm.on_enter_idle_       = []() { ESP_LOGI("SM_CB", "  -> Enter: IDLE"); };
    sm.on_enter_trot_       = []() { ESP_LOGI("SM_CB", "  -> Enter: TROT"); };
    sm.on_enter_transition_ = []() { ESP_LOGI("SM_CB", "  -> Enter: TRANSITION"); };
    sm.on_enter_error_      = []() { ESP_LOGI("SM_CB", "  -> Enter: ERROR"); };
    sm.on_enter_init_       = []() { ESP_LOGI("SM_CB", "  -> Enter: INIT"); };

    // 1. INIT -> IDLE
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "[1] INIT -> IDLE : onInitComplete()");
    sm.onInitComplete();
    ESP_LOGI(TAG, "    State: %s  (expected: IDLE)",
             sm.getState() == RobotState::IDLE ? "IDLE  [PASS]" : "[FAIL]");

    // 2. IDLE -> TROT
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "[2] IDLE -> TROT : onVelocityCommand()");
    sm.onVelocityCommand();
    ESP_LOGI(TAG, "    State: %s  (expected: TROT)",
             sm.getState() == RobotState::TROT ? "TROT  [PASS]" : "[FAIL]");

    // 3. TROT -> TRANSITION
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "[3] TROT -> TRANSITION : onStopCommand()");
    sm.onStopCommand();
    ESP_LOGI(TAG, "    State: %s  (expected: TRANSITION)",
             sm.getState() == RobotState::TRANSITION ? "TRANSITION  [PASS]" : "[FAIL]");

    // 4. TRANSITION -> IDLE
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "[4] TRANSITION -> IDLE : onTransitionComplete()");
    sm.onTransitionComplete();
    ESP_LOGI(TAG, "    State: %s  (expected: IDLE)",
             sm.getState() == RobotState::IDLE ? "IDLE  [PASS]" : "[FAIL]");

    // 5. ANY -> ERROR (Watchdog 타임아웃)
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "[5] IDLE -> ERROR : onWatchdogTimeout()");
    sm.onWatchdogTimeout();
    ESP_LOGI(TAG, "    State: %s  (expected: ERROR)",
             sm.getState() == RobotState::ERROR ? "ERROR  [PASS]" : "[FAIL]");

    // 6. ERROR 상태에서 비정상 이벤트 → 무시 확인
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "[6] ERROR  onInitComplete() -> must be ignored");
    sm.onInitComplete();
    ESP_LOGI(TAG, "    State: %s  (expected: ERROR)",
             sm.getState() == RobotState::ERROR ? "ERROR  [PASS]" : "[FAIL]");

    // 7. ERROR -> INIT (리셋 명령)
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "[7] ERROR -> INIT : onResetCommand()");
    sm.onResetCommand();
    ESP_LOGI(TAG, "    State: %s  (expected: INIT)",
             sm.getState() == RobotState::INIT ? "INIT  [PASS]" : "[FAIL]");

    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "====================================");
    ESP_LOGI(TAG, "[Phase 1] StateMachine Test DONE");
    ESP_LOGI(TAG, "====================================");
}
#endif  // Phase 1 테스트 — 검증 완료 후 비활성화

// ============================================================
// Core 0: 통신 태스크 진입점  (Phase 2)
// ============================================================
// CommTask.h의 commTaskRun()으로 바로 위임한다.
// 스택 6144B: WiFi 드라이버 + lwIP + UDP 수신 버퍼 + 지역변수 여유분
// ============================================================
static CommTaskParams g_comm_params;  // 정적 할당 — 태스크 수명과 동일

void CommTaskEntry(void* pvParameters) {
    commTaskRun(pvParameters);  // CommTask.h 에 정의된 무한 수신 루프
}

// ============================================================
// Core 1: 제어 태스크  (Phase 3+ 골격)
// ============================================================
// 현재: vTaskDelayUntil 기반 20ms 루프 + Watchdog 감시만 구현.
// 보행 파이프라인은 Phase 3~4에서 이 루프 안에 채운다.
// ============================================================
void ControlTask(void* pvParameters) {
    ESP_LOGI(TAG, "Core 1: ControlTask started (core: %d)", xPortGetCoreID());

    // --- 기구학 모듈 인스턴스 (Phase 4에서 실제 파이프라인에 사용) ---
    RobotParams         PARAMS_;
    GaitSequencer       gait_sequencer_(PARAMS_);
    FootPosPlanner      foot_planner_(PARAMS_);
    TrajectoryGenerator traj_gen_(PARAMS_);
    BodyKinematics      body_kinematics_(PARAMS_);
    IK                  ik_solver_(PARAMS_);

    // vTaskDelayUntil을 위한 기준 시각 초기화
    // vTaskDelayUntil은 "마지막 깨어난 시각 + 주기"를 절대 시각으로 지정하여
    // 루프 처리 시간이 길어져도 누적 드리프트가 발생하지 않는다.
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true) {
        // -------------------------------------------------------
        // 20ms 정주기 대기 (50Hz)
        // vTaskDelay와 달리 연산 시간을 제외한 나머지만 대기하므로
        // 루프 주기가 일정하게 유지된다.
        // -------------------------------------------------------
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CONTROL_DT_MS));

        // -------------------------------------------------------
        // Step 1. SharedData 스냅샷 복사
        // Mutex를 잡는 구간을 최소화한다.
        // 기구학 연산 중에 락을 쥐고 있으면 CommTask를 블로킹하게 된다.
        // -------------------------------------------------------
        SharedData snapshot;
        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            snapshot = g_shared;    // 구조체 복사 (얕은 복사, 포인터 없음)
            xSemaphoreGive(g_mutex);
        } else {
            // 2ms 안에 Mutex를 얻지 못하면 이번 사이클 건너뜀
            // (Core 0이 SharedData를 매우 오래 쥐고 있다면 CommTask 구조 재검토 필요)
            ESP_LOGW(TAG, "ControlTask: mutex timeout — cycle skipped");
            continue;
        }

        // -------------------------------------------------------
        // Step 2. Watchdog — 통신 두절 감시
        // 마지막 수신 타임스탬프와 현재 시각의 차이가
        // WATCHDOG_TIMEOUT_MS(100ms)를 초과하면 ERROR로 강제 전이한다.
        // -------------------------------------------------------
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
        uint32_t age_ms = now_ms - snapshot.timestamp_ms;

        // timestamp_ms == 0이면 아직 패킷을 한 번도 받지 못한 상태 → 감시 스킵
        if (snapshot.timestamp_ms > 0 && age_ms > WATCHDOG_TIMEOUT_MS) {
            // StateMachine::onWatchdogTimeout()은 이미 ERROR면 내부에서 무시한다.
            if (g_sm.getState() != RobotState::ERROR) {
                ESP_LOGE(TAG, "[Watchdog] TIMEOUT (%lu ms > %lu ms) -> ERROR",
                         (unsigned long)age_ms,
                         (unsigned long)WATCHDOG_TIMEOUT_MS);
                g_sm.onWatchdogTimeout();
            }
        }

        // -------------------------------------------------------
        // Step 3. 리셋 명령 처리 (안전 이중 처리)
        // CommTask가 수신 즉시 onResetCommand()를 호출하지만,
        // 타이밍 경합으로 누락될 경우를 대비한 폴백이다.
        // onResetCommand()는 ERROR 상태가 아니면 자체적으로 무시한다.
        // -------------------------------------------------------
        if (snapshot.reset_requested && g_sm.getState() == RobotState::ERROR) {
            g_sm.onResetCommand();
        }

        // -------------------------------------------------------
        // Step 4. 상태별 분기 (Phase 3~ 에서 채울 보행 파이프라인)
        // -------------------------------------------------------
        // switch (g_sm.getState()) {
        //     case RobotState::INIT:
        //         // 보간 생성기 호출 → 엎드린 자세에서 IDLE 자세로 Soft-Start
        //         break;
        //     case RobotState::IDLE:
        //         // Home Stance 유지 + 속도 임계 감시
        //         break;
        //     case RobotState::TROT:
        //         // 보행 파이프라인: GaitSequencer → FootPlanner → TrajGen → BodyKin → IK → 출력
        //         break;
        //     case RobotState::TRANSITION:
        //         // SWING 다리를 Home으로 부드럽게 착지 보간
        //         break;
        //     case RobotState::ERROR:
        //         // 안전 자세 보간 + 모터 잠금
        //         break;
        // }
    }
}

// ============================================================
// 엔트리 포인트
// ============================================================
extern "C" void app_main(void) {
    ESP_LOGI(TAG, "ESP32 Quadruped Controller booting...");

    // --- Mutex 생성 ---
    // SharedData는 두 코어에서 동시에 접근하므로 반드시 Mutex로 보호해야 한다.
    g_mutex = xSemaphoreCreateMutex();
    if (g_mutex == nullptr) {
        ESP_LOGE(TAG, "FATAL: Failed to create mutex. Halting.");
        return;
    }
    ESP_LOGI(TAG, "Mutex created OK");

    // --- Phase 1 테스트 비활성화 ---
    // runStateMachineTest();  // #if 0 블록 안으로 이동

    // --- Phase 2: CommTask 파라미터 연결 ---
    g_comm_params.shared  = &g_shared;
    g_comm_params.mutex   = g_mutex;
    g_comm_params.sm_ptr  = &g_sm;

    // --- Core 0: 통신 태스크 생성 ---
    // 스택 6144B: WiFi 스택 + lwIP + UDP 파싱 버퍼 + 지역변수
    // 우선순위 1 (낮음): 네트워크 패킷은 비주기적이므로 제어 루프보다 낮게 설정
    xTaskCreatePinnedToCore(
        CommTaskEntry, "CommTask", 6144, &g_comm_params, 1, NULL, 0);

    // --- Core 1: 제어 태스크 생성 ---
    // 스택 8192B: Eigen 행렬 연산 + 기구학 모듈 지역 변수 여유분
    // 우선순위 2 (높음): 20ms 정주기 보장이 최우선
    xTaskCreatePinnedToCore(
        ControlTask, "ControlTask", 8192, NULL, 2, NULL, 1);

    ESP_LOGI(TAG, "All tasks created. Scheduler running.");
}