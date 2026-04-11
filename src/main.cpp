#ifndef CALIBRATION_MODE
// === Normal Locomotion Firmware ===

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include "quadruped_types.h"
#include "StateMachine.h"
#include "GaitSequencer.h"
#include "FootPlanner.h"
#include "TrajectoryGenerator.h"
#include "BodyKinematics.h"
#include "IK.h"
#include "LocomotionController.h"
#include "HardwareOutput.h" // Phase 5: Hardware Abstraction Layer
#include "CommTask.h"   // Phase 2: CommTaskParams 구조체 + commTaskRun()

// --- [DEBUG DWT PROBE START] ---
// 추후 실제 작동 시 주석 처리할 DWT 타이머 헤더
#include "dwt_timer.h"  // Phase 3: DWT 타이머 유틸리티
// --- [DEBUG DWT PROBE END] ---

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

    // --- [DEBUG DWT PROBE START] ---
    // 추후 실제 작동 시 아래 버퍼 할당 및 초기화 코드를 전체 주석 처리하세요.
    // --- Phase 3 DWT 타이머 및 버퍼 초기화 ---
    DWT::init();
    struct TimingRecord {
        uint32_t loop_idx;
        float dt_ms;
        uint32_t exec_us;
        uint32_t mutex_us;
        uint32_t pipeline_us;
        uint8_t deadline_miss;
    };
    static TimingRecord timing_buf[400];
    static int timing_idx = 0;
    static bool buf_dumped = false;
    uint32_t loop_counter = 0;
    // --- [DEBUG DWT PROBE END] ---

    // --- 기구학 모듈 인스턴스 (Phase 4에서 실제 파이프라인에 사용) ---
    RobotParams         PARAMS_;
    GaitSequencer       gait_sequencer_(PARAMS_);
    FootPosPlanner      foot_planner_(PARAMS_);
    TrajectoryGenerator traj_gen_(PARAMS_);
    BodyKinematics      body_kinematics_(PARAMS_);
    IK                  ik_solver_(PARAMS_);

    LocomotionController loco_ctrl_(PARAMS_, gait_sequencer_, foot_planner_, traj_gen_, body_kinematics_, ik_solver_);

    // --- [Phase 5] 하드웨어 출력 레이어 플러그인 ---
    HardwareOutput hw_out;
    if (!hw_out.init()) {
        ESP_LOGE(TAG, "HardwareOutput init failed! Please check I2C wiring.");
    }


    // -------------------------------------------------------
    // StateMachine 전이 콜백 등록
    // 캡처 람다로 loco_ctrl_을 참조 → 전이 시점에 1회만 실행
    // [주의] loco_ctrl_은 ControlTask 스택에 있으므로,
    //        이 콜백은 반드시 Core 1 (ControlTask) 에서만 호출되어야 한다.
    //        Core 0 (CommTask)의 onResetCommand()는 INIT 콜백만 트리거하므로
    //        현재 구조에서 cross-core 접근 위험은 없다.
    // -------------------------------------------------------
    // IDLE 진입 시: foot_pos_global_ 및 stance_start_pos_를 Home Stance로 초기화
    // (INIT->IDLE, TRANSITION->IDLE 양쪽에서 모두 호출됨)
    g_sm.on_enter_idle_ = [&loco_ctrl_]() {
        loco_ctrl_.initHomeStance();
        ESP_LOGI("SM_CB", "on_enter_idle_: Home Stance initialized");
    };

    // TROT 진입 시: foot_pos_global_ 및 stance_start_pos_를 Home Stance로 초기화
    // rising Edge Latch의 swing_start_pos_가 Home 좌표를 기준으로 시작되도록 보장.
    // gait_sequencer_.reset()으로 current_time_=0 초기화:
    //   이전 보행에서 누적된 시간이 남아 s가 중간값(예: 0.6)에서
    //   시작되는 것을 방지하고, 항상 s=0부터 깨끗하게 보행 시작.
    g_sm.on_enter_trot_ = [&loco_ctrl_, &gait_sequencer_]() {
        gait_sequencer_.reset();        // s=0부터 시작 보장 (위상 리셋)
        loco_ctrl_.initHomeStance();    // 발 좌표 Home으로 초기화
        ESP_LOGI("SM_CB", "on_enter_trot_: GaitSequencer reset + Home Stance initialized");
    };

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


        // --- [DEBUG DWT PROBE START] ---
        // for debug
        // [Phase 3 Probe] 루프 진입
        uint32_t t_loop_start = DWT::now_cycles();
        uint32_t current_pipeline_us = 0;
        // --- [DEBUG DWT PROBE END] ---

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

        // --- [DEBUG DWT PROBE START] ---
        // for debug    
        uint32_t t_mutex_start = DWT::now_cycles(); // [Phase 3 Probe] Mutex 시작
        // --- [DEBUG DWT PROBE END] ---
        
        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            snapshot = g_shared;    // 구조체 복사 (얕은 복사, 포인터 없음)
            xSemaphoreGive(g_mutex);
        } else {
            // 2ms 안에 Mutex 단기간 점유 실패 -> 이번 사이클 스킵
            ESP_LOGW(TAG, "ControlTask: mutex timeout — cycle skipped");
            continue;
        }

        // --- [DEBUG DWT PROBE START] ---
        uint32_t t_mutex_end = DWT::now_cycles();   // [Phase 3 Probe] Mutex 종료
        // --- [DEBUG DWT PROBE END] ---

        // -------------------------------------------------------
        // Step 2. Watchdog — 통신 두절 감시
        // -------------------------------------------------------
        uint32_t now_ms = (uint32_t)(current_time / 1000ULL);
        // CommTask가 방금 기록한 timestamp가 now_ms보다 미래일 수 있으므로(언더플로우 방지)
        uint32_t age_ms = (now_ms > snapshot.timestamp_ms) ? (now_ms - snapshot.timestamp_ms) : 0;

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
                // [TBD: Phase 6] 보간 생성기 호출 → 엎드린 자세에서 IDLE 자세로 Soft-Start
                // 현재는 테스트를 위해 즉시 IDLE로 강제 전이합니다.
                g_sm.onInitComplete();
                break;
            case RobotState::IDLE:
            {
                // Home Stance 유지는 on_enter_idle_ 콜백에서 전이 시 1회 수행
                // (매 프레임 호출 불필요)

                // 속도 명령 감시: 임계(0.01) 초과 시 TROT으로 전이
                // onVelocityCommand() 내부에서 on_enter_trot_ 콜백이 호출된다.
                bool has_velocity = (fabsf(snapshot.cmd.vx) > 0.01f ||
                                     fabsf(snapshot.cmd.vy) > 0.01f ||
                                     fabsf(snapshot.cmd.wz) > 0.01f);

                if (has_velocity) {
                    ESP_LOGI(TAG, "[StateMachine] IDLE -> TROT (vx=%.2f vy=%.2f wz=%.2f)",
                             snapshot.cmd.vx, snapshot.cmd.vy, snapshot.cmd.wz);
                    g_sm.onVelocityCommand(); // → on_enter_trot_ 콜백 자동 호출
                }
                break;
            }
            case RobotState::TROT:
            {
                // -------------------------------------------------------
                // 보행 파이프라인 연산 (Phase 4)
                // -------------------------------------------------------

                // --- [DEBUG DWT PROBE START] ---
                uint32_t t_pipe_start = DWT::now_cycles(); // [Phase 3 Probe] 파이프라인 시작
                // --- [DEBUG DWT PROBE END] ---

                auto target_angles = loco_ctrl_.processTrot(dt, snapshot.cmd);

                // --- [DEBUG DWT PROBE START] ---
                uint32_t t_pipe_end = DWT::now_cycles();   // [Phase 3 Probe] 파이프라인 종료
                current_pipeline_us = (uint32_t)DWT::cycles_to_us(t_pipe_end - t_pipe_start);
                // --- [DEBUG DWT PROBE END] ---

                // --- [Phase 5] 하드웨어 펄스 출력 (Hardware Abstraction Layer) ---
                float target_math_angles[4][3];
                for (int i = 0; i < 4; i++) {
                    target_math_angles[i][0] = target_angles[i].x(); // HAA
                    target_math_angles[i][1] = target_angles[i].y(); // HFE
                    target_math_angles[i][2] = target_angles[i].z(); // KNE
                }
                hw_out.writeJoints(target_math_angles);


                // [Phase 4 검증] 1초마다: IK 각도 스냅샷 + 범위 검사 + T_cycle 출력
                // 루프 지연(Deadline Miss) 방지를 위해 로그는 1초에 한 번만 출력합니다.
                static uint64_t last_ik_log = 0;
                if (current_time - last_ik_log > 1000000ULL) {
                    // 1. IK 범위 결함이 있는지 검사 (Phase 5 이전이므로 음수 각도 정상)
                    // (완전한 물리 이탈(예: -90~90도 초과) 여부만 가볍게 출력)
                    const float RAD_MIN = 0.0; 
                    const float RAD_MAX =  M_PI;      
                    for (int i = 0; i < 4; ++i) {
                        for (int j = 0; j < 3; ++j) {
                            float a = target_angles[i](j);
                            if (a < RAD_MIN || a > RAD_MAX) {
                                ESP_LOGW(TAG, "[Phase4/test1] Leg%d joint%d OUT_OF_RANGE %.1f deg",
                                         i, j, a * (180.0f / M_PI));
                            }
                        }
                    }

                    // 2. T_cycle 판정
                    float t_cycle = gait_sequencer_.getCycleTime();
                    // velocity 로깅 주석 처리
                    // ESP_LOGI(TAG, "[Phase4] cmd(vx=%.2f vy=%.2f wz=%.2f) T_cycle=%.3fs",
                    //          snapshot.cmd.vx, snapshot.cmd.vy, snapshot.cmd.wz,
                    //          t_cycle); 
                             
                    // // 3. IK 각도 출력
                    // ESP_LOGI(TAG, "IK[LF]: %.2f %.2f %.2f | IK[RF]: %.2f %.2f %.2f",
                    //          target_angles[0].x(), target_angles[0].y(), target_angles[0].z(),
                    //          target_angles[1].x(), target_angles[1].y(), target_angles[1].z());
                    // ESP_LOGI(TAG, "IK[LH]: %.2f %.2f %.2f | IK[RH]: %.2f %.2f %.2f",
                    //          target_angles[2].x(), target_angles[2].y(), target_angles[2].z(),
                    //          target_angles[3].x(), target_angles[3].y(), target_angles[3].z());
                             
                    last_ik_log = current_time;
                }

                // 속도=0 감시: 정지 명령 수신 시 TRANSITION으로 전이
                bool is_stopped = (fabsf(snapshot.cmd.vx) <= 0.01f &&
                                   fabsf(snapshot.cmd.vy) <= 0.01f &&
                                   fabsf(snapshot.cmd.wz) <= 0.01f);
                if (is_stopped) {
                    ESP_LOGI(TAG, "[StateMachine] TROT -> TRANSITION (velocity zero)");
                    g_sm.onStopCommand();
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
         
        // --- [DEBUG DWT PROBE START] ---
        // for debug
        uint32_t t_loop_end = DWT::now_cycles(); // [Phase 3 Probe] 루프 종료

        if (!buf_dumped && timing_idx < 400 && g_sm.getState() == RobotState::TROT) {
            TimingRecord& r = timing_buf[timing_idx++];
            r.loop_idx = loop_counter;
            r.dt_ms = dt_ms;
            r.exec_us = (uint32_t)DWT::cycles_to_us(t_loop_end - t_loop_start);
            r.mutex_us = (uint32_t)DWT::cycles_to_us(t_mutex_end - t_mutex_start);
            r.pipeline_us = current_pipeline_us;
            r.deadline_miss = (dt_ms > CONTROL_DT_MS + 1.0f) ? 1 : 0;
            
            if (timing_idx == 400) { // 버퍼 포화 시 1회 일괄 덤프
                buf_dumped = true;
                ESP_LOGI(TAG, "--- DWT Timing Buffer Dump ---");
                printf("idx, dt_ms, exec_us, mutex_us, pipeline_us, miss\n");
                for (int i = 0; i < 400; i++) {
                    TimingRecord& tr = timing_buf[i];
                    printf("%lu,%.2f,%lu,%lu,%lu,%d\n", 
                        (unsigned long)tr.loop_idx, 
                        tr.dt_ms, 
                        (unsigned long)tr.exec_us, 
                        (unsigned long)tr.mutex_us, 
                        (unsigned long)tr.pipeline_us, 
                        tr.deadline_miss);
                    if ((i + 1) % 32 == 0) {
                        vTaskDelay(pdMS_TO_TICKS(1)); // UART FIFO 넘침 방지
                    }
                }
                ESP_LOGI(TAG, "--- Dump Complete ---");
            }
        }
        loop_counter++;
        // --- [DEBUG DWT PROBE END] ---

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
        ControlTask, "ControlTask", 12288, NULL, 2, NULL, 1); // 무슨함수임

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

#endif // !CALIBRATION_MODE