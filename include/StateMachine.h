#pragma once
#include "esp_log.h"
#include "quadruped_types.h"

static const char* TAG_SM = "StateMachine";

// ============================================================
// StateMachine
// ============================================================
// 로봇의 글로벌 상태(RobotState)를 관리하는 클래스.
// 각 전이(Transition)마다 진입/이탈 콜백이 호출되어
// 필요한 초기화 로직을 상태별로 격리한다.
//
// [전이 규칙]
//   INIT       → IDLE        : onInitComplete() 호출 시 (보간 완료)
//   IDLE       → TROT        : onVelocityCommand() 호출 시 (속도 임계 초과)
//   TROT       → TRANSITION  : onStopCommand() 호출 시 (속도 = 0)
//   TRANSITION → IDLE        : onTransitionComplete() 호출 시 (착지 완료)
//   ANY        → ERROR       : onWatchdogTimeout() 호출 시
//   ERROR      → INIT        : onResetCommand() 호출 시 (명시적 리셋)
// ============================================================
class StateMachine {
public:
    // ------------------------------------------------------------
    // 콜백 함수 포인터 타입 정의
    // ------------------------------------------------------------
    using StateCallback = void (*)();

    // 각 상태 진입 시 호출할 콜백 (nullptr 가능 = 콜백 없음)
    StateCallback on_enter_init_       = nullptr;
    StateCallback on_enter_idle_       = nullptr;
    StateCallback on_enter_trot_       = nullptr;
    StateCallback on_enter_transition_ = nullptr;
    StateCallback on_enter_error_      = nullptr;

    // ------------------------------------------------------------
    // 생성자
    // ------------------------------------------------------------
    explicit StateMachine(RobotState initial_state = RobotState::INIT)
        : state_(initial_state)
    {
        ESP_LOGI(TAG_SM, "StateMachine initialized. Initial state: %s", stateToStr(state_));
    }

    // 현재 상태 반환
    RobotState getState() const { return state_; }

    // ============================================================
    // 이벤트 트리거 함수들
    // ============================================================

    // INIT 보간 완료 → IDLE
    void onInitComplete() {
        if (state_ != RobotState::INIT) {
            ESP_LOGW(TAG_SM, "onInitComplete(): Ignored in current state (%s)", stateToStr(state_));
            return;
        }
        transitionTo(RobotState::IDLE);
    }

    // 유효 속도 명령 수신 → TROT
    void onVelocityCommand() {
        if (state_ != RobotState::IDLE) {
            return; // IDLE에서만 TROT 전이
        }
        transitionTo(RobotState::TROT);
    }

    // 속도 0 / 정지 명령 → TRANSITION
    void onStopCommand() {
        if (state_ != RobotState::TROT) {
            return;
        }
        transitionTo(RobotState::TRANSITION);
    }

    // TRANSITION 보간 완료 (모든 다리 착지) → IDLE
    void onTransitionComplete() {
        if (state_ != RobotState::TRANSITION) {
            ESP_LOGW(TAG_SM, "onTransitionComplete(): Ignored in current state (%s)", stateToStr(state_));
            return;
        }
        transitionTo(RobotState::IDLE);
    }

    // Watchdog 타임아웃 → ERROR (어느 상태에서든 즉시 전이)
    void onWatchdogTimeout() {
        if (state_ == RobotState::ERROR) return; // 이미 ERROR면 무시
        ESP_LOGE(TAG_SM, "[!] Watchdog Timeout! Forced ERROR transition");
        transitionTo(RobotState::ERROR);
    }

    // 명시적 리셋 명령 수신 → INIT (ERROR 상태에서만)
    void onResetCommand() {
        if (state_ != RobotState::ERROR) {
            ESP_LOGW(TAG_SM, "onResetCommand(): Not in ERROR state (%s). Ignored", stateToStr(state_));
            return;
        }
        transitionTo(RobotState::INIT);
    }

private:
    RobotState state_;

    // ------------------------------------------------------------
    // 실제 상태 전이 실행기
    // ------------------------------------------------------------
    void transitionTo(RobotState next) {
        ESP_LOGI(TAG_SM, "[Transition] %s -> %s", stateToStr(state_), stateToStr(next));
        state_ = next;

        // 진입 콜백 호출
        switch (state_) {
            case RobotState::INIT:       if (on_enter_init_)       on_enter_init_();       break;
            case RobotState::IDLE:       if (on_enter_idle_)       on_enter_idle_();       break;
            case RobotState::TROT:       if (on_enter_trot_)       on_enter_trot_();       break;
            case RobotState::TRANSITION: if (on_enter_transition_) on_enter_transition_(); break;
            case RobotState::ERROR:      if (on_enter_error_)      on_enter_error_();      break;
        }
    }

    // 로깅용 상태 → 문자열 변환
    static const char* stateToStr(RobotState s) {
        switch (s) {
            case RobotState::INIT:       return "INIT";
            case RobotState::IDLE:       return "IDLE";
            case RobotState::TROT:       return "TROT";
            case RobotState::TRANSITION: return "TRANSITION";
            case RobotState::ERROR:      return "ERROR";
            default:                     return "UNKNOWN";
        }
    }
};
