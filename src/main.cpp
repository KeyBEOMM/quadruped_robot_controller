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
#include "LocomotionController.h"
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

    LocomotionController loco_ctrl_(PARAMS_, gait_sequencer_, foot_planner_, traj_gen_, body_kinematics_, ik_solver_);

    // vTaskDelayUntil을 위한 기준 시각 초기화
    // vTaskDelayUntil은 "마지막 깨어난 시각 + 주기"를 절대 시각으로 지정하여
    // 루프 처리 시간이 길어져도 누적 드리프트가 발생하지 않는다.
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint64_t prev_time = esp_timer_get_time();

    while (true) {
        // -------------------------------------------------------
        // 20ms 정주기 대기 (50Hz)
        // vTaskDelay와 달리 연산 시간을 제외한 나머지만 대기하므로
        // 루프 주기가 일정하게 유지된다.
        // -------------------------------------------------------
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CONTROL_DT_MS));

        uint64_t current_time = esp_timer_get_time();
        float dt = (current_time - prev_time) / 1000000.0f; // dt는 제어 주기 즉, 무조건 0.02초이어야함
        float dt_ms = (current_time - prev_time) / 1000.0f;
        prev_time = current_time;

        // Deadline Miss Check (±1ms tolerance)
        if (dt_ms > CONTROL_DT_MS + 1.0f) {
            ESP_LOGW(TAG, "[!] Deadline Miss: dt = %.1f ms (Expected: %lu ms)", dt_ms, (unsigned long)CONTROL_DT_MS);
        }

        uint64_t loop_start_time = esp_timer_get_time();

        // -------------------------------------------------------
        // Step 1. SharedData 스냅샷 복사
        // Mutex를 잡는 구간을 최소화한다. // 현재 g_comm
        // 기구학 연산 중에 락을 쥐고 있으면 CommTask를 블로킹하게 된다.
        // -------------------------------------------------------
        SharedData snapshot;
        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            snapshot = g_shared;    // 구조체 복사 (얕은 복사, 포인터 없음)
            xSemaphoreGive(g_mutex);
        } else {
            // 2ms 안에 Mutex 단기간 점유 실패 -> 이번 사이클 스킵
            ESP_LOGW(TAG, "ControlTask: mutex timeout — cycle skipped");
            continue;
        }

        // -------------------------------------------------------
        // Step 2. Watchdog — 통신 두절 감시
        // 마지막 수신 타임스탬프와 현재 시각의 차이가
        // WATCHDOG_TIMEOUT_MS(100ms)를 초과하면 ERROR로 강제 전이한다.
        // -------------------------------------------------------
        uint32_t now_ms = (uint32_t)(current_time / 1000ULL);
        uint32_t age_ms = now_ms - snapshot.timestamp_ms;

        if (snapshot.timestamp_ms > 0 && age_ms > WATCHDOG_TIMEOUT_MS) {
            if (g_sm.getState() != RobotState::ERROR) {
                ESP_LOGE(TAG, "[Watchdog] TIMEOUT (%lu ms > %lu ms) -> ERROR",
                         (unsigned long)age_ms,
                         (unsigned long)WATCHDOG_TIMEOUT_MS);
                g_sm.onWatchdogTimeout();
            }
        }

        // -------------------------------------------------------
        // Step 3. 리셋 명령 처리 (안전 이중 처리)
        // -------------------------------------------------------
        if (snapshot.reset_requested) {
            if (g_sm.getState() == RobotState::ERROR) {
                g_sm.onResetCommand();
            }
            if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                g_shared.reset_requested = false;
                xSemaphoreGive(g_mutex);
            }
        }

        // -------------------------------------------------------
        // Step 4. 상태별 분기 (Phase 3~ 에서 채울 보행 파이프라인)
        // -------------------------------------------------------
        switch (g_sm.getState()) {
            case RobotState::INIT:
                // 보간 생성기 호출 → 엎드린 자세에서 IDLE 자세로 Soft-Start
                break;
            case RobotState::IDLE:
                // Home Stance 유지
                loco_ctrl_.initHomeStance();
                break;
            case RobotState::TROT:
            {
                // 보행 파이프라인 연산 (Phase 4)
                auto target_angles = loco_ctrl_.processTrot(dt, snapshot.cmd);
                
                // 임시: 매 초마다 첫 번째 다리의 연산된 첫 번째 관절(HAA) 로그 출력 (수치 검증용)
                static uint64_t last_ik_log = 0;
                if (current_time - last_ik_log > 1000000ULL) {
                    ESP_LOGI(TAG, "IK Out (LF): %.2f, %.2f, %.2f rad", 
                             target_angles[0].x(), target_angles[0].y(), target_angles[0].z());
                    last_ik_log = current_time;
                }
                break;
            }
            case RobotState::TRANSITION:
                // SWING 다리를 Home으로 부드럽게 착지 보간
                break;
            case RobotState::ERROR:
                // 안전 자세 보간 + 모터 잠금
                break;
        }

        uint64_t loop_end_time = esp_timer_get_time();
        float execution_time_ms = (loop_end_time - loop_start_time) / 1000.0f;
        
        // 디버그/검증용: 1초에 한 번만 출력하도록 하면 로그 도배를 막을 수 있지만, 
        // Phase 3 타이밍 측정 목적이므로 지속적으로 실행시간 관찰이 필요하다면 아래처럼 출력할 수 있습니다.
        // (단, printf 자체가 1ms 정도 소요될 수 있으므로 측정 이후 마지막에 실행해야 합니다)
        static uint64_t last_print_time = 0;
        if (loop_end_time - last_print_time > 1000000ULL) { // 1초에 한번만 출력
            ESP_LOGI(TAG, "ControlTask Alive | dt: %.2fms | Execution: %.2fms | State: %d", 
                     dt * 1000.0f, execution_time_ms, (int)g_sm.getState()); // execution_time_ms는 실제 제어 루프 실행 시간 지표 확인 후 제어 주기 낮춰도됨
            last_print_time = loop_end_time;
        }
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

    // --- Phase 2: CommTask 파라미터 연결 --- // g_comm_params는 통신 파라미터를 담은 구조체(shared(Core 0 Write / Core 1 Read 공유 데이터), mutex, sm_ptr(전역 상태 머신))
    g_comm_params.shared  = &g_shared; // 정적 전역 변수 g_shared의 주소를 CommTaskParams(static 정적변수) 구조체에 할당
    g_comm_params.mutex   = g_mutex; // 정적 전역 변수 g_mutex의 주소(g_mutex는 이미 포인터임)를 CommTaskParams(static 정적변수) 구조체에 할당
    g_comm_params.sm_ptr  = &g_sm; // 정적 전역 변수 g_sm의 주소를 CommTaskParams(static 정적변수) 구조체에 할당

    // --- Core 0: 통신 태스크 생성 ---
    // 스택 6144B: WiFi 스택 + lwIP + UDP 파싱 버퍼 + 지역변수
    // 우선순위 1 (낮음): 네트워크 패킷은 비주기적이므로 제어 루프보다 낮게 설정
    xTaskCreatePinnedToCore(
        CommTaskEntry, "CommTask", 6144, &g_comm_params, 1, NULL, 0); // 무슨 함수임

    // --- Core 1: 제어 태스크 생성 ---
    // 스택 8192B: Eigen 행렬 연산 + 기구학 모듈 지역 변수 여유분
    // 우선순위 2 (높음): 20ms 정주기 보장이 최우선
    xTaskCreatePinnedToCore(
        ControlTask, "ControlTask", 8192, NULL, 2, NULL, 1); // 무슨함수임

    ESP_LOGI(TAG, "All tasks created. Scheduler running.");
}


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