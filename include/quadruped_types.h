#pragma once
#include <Eigen/Dense>

// 다리의 보행 위상 상태
enum class LegPhase
{
    SWING,
    STANCE
};

// 단일 다리 상태 
struct LegState
{
    LegPhase phase;           // 현재 보행 위상
    float s = 0.0f;           // 보행 위상 진행 정도 (0.0 ~ 1.0)
};

// 상위 제어기로 전달되는 속도 명령
struct RobotCommand
{
    float vx = 0.0f;  // 전진 속도 (m/s)
    float vy = 0.0f;  // 측면 속도 (m/s)
    float wz = 0.0f;  // 회전 속도 (rad/s)

    float roll = 0.0f;   // [rad] 몸통 좌우 기울기
    float pitch = 0.0f;  // [rad] 몸통 앞뒤 들림/숙임 각도
    float yaw = 0.0f;    // [rad] 몸통 절대 헤딩 방향 오프셋 (보통 0으로 둠)

    // float body_height = 0.164f; // [m] 몸통 높이 (기본값은 end_leg_length * cos(30 degrees))

    // (추가 확장 가능성) 험지 돌파 시 동적 스윙 높이 조절용 파라미터
    // float swing_height_multiplier = 1.0f;
};  

// robot parameters

// 서보 속도 :
// 1. 20kg - 4.8V / 0.15sec / 60 deg || 6.0V / 0.13 초 / 60 deg 
// 2. 25kbg - 0.18 초 / 60 deg ~ 0.14 초 / 60 deg 

// 20kgfcm 선정 - 안전 속도(약 150%) : 0.2 초 / 60 deg == 1 s / 300 deg  == 5.23598 rad/s  

// 보행 파라미터 : 
// 1. default 보행 각도 60 deg
// 2. leg 굽힘 정도 : 30 deg 
// 3. end leg length : 0.19 m (leg length 0.22 m * cos(30 deg))
// 4. default height : 0.164 m (end leg length * cos(30 deg))
// 5. default stride : 0.095 m (end leg length * sin(30 deg))
// 6. default cycle  time : 2.0 s (보행 주기) -> 속도 명령에 따라 조절 (최소 0.4 s) 실제로 테스트해봐야할듯
// 7. duty factor : 0.5 (trot gait 기준) -> 위상 오프셋 0.5로 설정 (LF, RF, LH, RH 순서로 0.0, 0.5, 0.0, 0.5)
        // duty factor = stance time / cycle time

struct RobotParams 
{
    float leg_length = 0.22f;   // 다리 길이 (m)
    float end_leg_length = 0.19f; // 발 길이 (m) leg_length * cos(30 degrees)
    
    float default_height = 0.164f; // 기본 높이 (m) end_leg_length * cos(30 degrees)
    float default_stride = 0.095f; // 기본 반보폭 (m) end_leg_length * sin(30 degrees)
    
    float min_cycle_time = 0.4f; // 최소 보행 주기 시간 (s)
    float default_cycle_time = 2.0f; // 기본 보행 주기 시간 (s) -> default_velocity -> 0.095 * 4 / 2.0 = 0.19 m/s
    float max_cycle_time = 3.0f; // 최대 보행 주기 시간 (s)

    float THRESHOLD_SPEED = default_stride * 4.0f / default_cycle_time; // 정지로 간주되는 속도 임계값 (m/s)

    float DUTY_FACTOR = 0.5f;

    const float MIN_STEP_HEIGHT = 0.02f;  // 제자리 걸음이어도 무조건 2cm는 든다.
    const float HEIGHT_RATIO = 0.2f;      // 보폭의 20%만큼 추가로 더 든다.
    const float MAX_STEP_HEIGHT = 0.05f;  // 아무리 빨리 뛰어도 5cm 이상은 들지 않는다.
    
    float body_length = 0.255f; // 몸체 길이 (m) 255mm
    float body_width = 0.078f;  // 몸체 너비 (m) 78mm

    Eigen::Vector3f shoulder_offsets[4] = {
        { body_length/2,  body_width/2 , 0.0f},  // LF
        { body_length/2, -body_width/2, 0.0f},  // RF
        {-body_length/2,  body_width/2, 0.0f},  // LH
        {-body_length/2, -body_width/2, 0.0f}   // RH
    };  

    const float HAA_OFFSET_Y = 0.0605f;   // HAA Offset
    const float HAA_OFFSET_Z = 0.01f;
    const float HFE_OFFSET = 0.1111f; // Upper Link
    const float KNE_OFFSET = 0.1185f;  // Lower Link
    const float FOOT_OFFSET = 0.02f; // Foot Link
};

// ============================================================
// 글로벌 로봇 상태 (State Machine 전이 대상)
// ============================================================
enum class RobotState
{
    INIT,        // 전원 인가 직후: Soft-Start 보간으로 기립 중
    IDLE,        // 대기: 4발 지면, Home Stance Position 유지
    TROT,        // 트롯 보행: 보행 파이프라인 활성
    TRANSITION,  // 정지 보간: SWING 다리를 Home Stance로 부드럽게 착지
    ERROR        // 안전 잠금: Watchdog 타임아웃, 수동 리셋 필요
};

// ============================================================
// Core 간 공유 데이터 (Mutex 보호 필수)
// Core 0(CommTask)가 Write, Core 1(ControlTask)가 Read
// ============================================================
struct SharedData
{
    RobotCommand cmd;                 // 최신 속도/자세 명령
    uint32_t timestamp_ms = 0;        // 수신 시각 (esp_timer 기반, ms)
    bool reset_requested = false;     // ERROR→INIT 복귀 트리거
};

// ============================================================
// 시스템 타이밍 상수
// ============================================================
constexpr uint32_t CONTROL_DT_MS = 20;          // 제어 루프 목표 주기 (ms) → 50Hz
constexpr float    CONTROL_DT_S  = 0.020f;      // 제어 루프 주기 (초)
constexpr uint32_t WATCHDOG_TIMEOUT_MS = 500;    // 통신 두절 판정 임계 시간 (ms) - WiFi 환경 고려 완화
constexpr float    INIT_DURATION_S = 3.0f;       // Soft-Start 기립 보간 시간 (초)
