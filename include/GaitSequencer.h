#pragma once
#include <array>    
#include <cmath>
#include <Eigen/Dense>
#include "quadruped_types.h"

// 상위 제어 명령 RobotCommand를 기반으로 사이클 타임 조절, 위상에 따른 각 다리의 상태 및 진행률 s 업데이트
// 1. 속도 명령에 따른 사이클 타임 조절
// 2. 각 다리의 위상 계산(trot -> 0, 0.5, 0, 0.5)
// 3. 각 다리의 상태 업데이트 (SWING/STANCE) 및 진행 정도(s) 계산 and return leg states
class GaitSequencer {

private:
    RobotParams params_;
    std::array<LegState, 4> leg_states_; // 4개의 다리 상태
    
    float T_cycle_; // 현재 사이클 타임
    const float phase_OFFSET_[4] = {0.0f, 0.5f, 0.0f, 0.5f}; // trot gait의 위상 오프셋 (LF, RF, LH, RH)
    // float TUNNING_PARAM; // 튜닝 파라미터 cycle == 2s 일때, speed == 0.19 m/s가 되도록 설정 (실험적으로 조절 필요)
    float current_time_; // 현재 시간 (s)

public:
    GaitSequencer(const RobotParams& params) 
        :   params_(params),
            T_cycle_(params_.max_cycle_time),
            // TUNNING_PARAM(params_.default_cycle_time / params_.default_stride),
            current_time_(0.0f) 
    {
     
        // 초기 다리 상태 세팅 (모두 지면에 닿아있는 상태로 시작)
        for(auto& state : leg_states_) {
            state.phase = LegPhase::STANCE;
            state.s = 0.0f;
        }
    }
    
    void updateCycleTime(const RobotCommand& cmd) {
        // 4개 다리 각각의 합성 선속도(V + w × r) 중 최대값을 기준으로 사이클 타임 결정
        // 제자리 회전(vx=vy=0, wz≠0) 시에도 올바른 보행 주기가 산출됨
        float max_foot_speed = 0.0f;
        for (int i = 0; i < 4; ++i) {
            float v_foot_x = cmd.vx - (cmd.wz * params_.shoulder_offsets[i].y());
            float v_foot_y = cmd.vy + (cmd.wz * params_.shoulder_offsets[i].x());
            float foot_speed = std::sqrt(v_foot_x * v_foot_x + v_foot_y * v_foot_y);
            if (foot_speed > max_foot_speed) {
                max_foot_speed = foot_speed;
            }
        }

        if (max_foot_speed < 0.01f) {
            T_cycle_ = params_.max_cycle_time; // 완전 정지 시 여유 있게
            return;
        }

        if (max_foot_speed < params_.THRESHOLD_SPEED) {
            // [저속/중속 구간] 다리가 충분히 뻗을 수 있으므로 사이클 시간 고정
            T_cycle_ = params_.default_cycle_time;
        } 
        else {
            // [고속 구간] 목표 보폭이 다리 길이를 초과함 - 보폭 제한, 사이클 시간을 줄임
            T_cycle_ = params_.default_stride * 4.0f / max_foot_speed;
        }

        T_cycle_ = std::clamp(T_cycle_, params_.min_cycle_time, params_.max_cycle_time);    
    }

    void updatePhase(float dt) {
        current_time_ += dt;
        const float df = params_.DUTY_FACTOR; // Stance 비율 (Trot 기본값: 0.5)
        for (int i = 0; i < 4; ++i) {
            float phase = std::fmod((current_time_/T_cycle_) + phase_OFFSET_[i], 1.0f); // 0.0 ~ 1.0 사이의 위상 계산
            
            leg_states_[i].phase = (phase < df) ? LegPhase::STANCE : LegPhase::SWING;
            leg_states_[i].s = (phase < df) ? (phase / df) : ((phase - df) / (1.0f - df)); // 각 위상 내 0.0~1.0 정규화
        }                                               
    }
    const std::array<LegState, 4>& getLegStates() const { return leg_states_; }

    void update(float dt, const RobotCommand& cmd) {
    updateCycleTime(cmd);
    updatePhase(dt);
}
    
    const float getCycleTime() const { return T_cycle_; }
};