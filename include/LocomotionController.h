#pragma once
#include <array>
#include <Eigen/Dense>
#include "quadruped_types.h"
#include "GaitSequencer.h"
#include "FootPlanner.h"
#include "TrajectoryGenerator.h"
#include "BodyKinematics.h"
#include "IK.h"

// ============================================================
// LocomotionController
// ============================================================
// 보행 파이프라인의 핵심 제어기. 각 다리의 상태 전이(Edge 검출)와
// Latch된 파라미터(swing 목표점, Stance 시작점 등)를 관리하고,
// 하위 기구학 모듈들을 순차적으로 호출하여 최종 IK 관절 각도를 산출합니다.
// ============================================================
class LocomotionController {
private:
    RobotParams& params_;
    GaitSequencer& gait_sequencer_;
    FootPosPlanner& foot_planner_;
    TrajectoryGenerator& traj_gen_;
    BodyKinematics& body_kinematics_;
    IK& ik_solver_;

    // --- 런타임 상태 버퍼 (Latch용) ---
    LegPhase prev_phase_[4];
    Eigen::Vector3f swing_start_pos_[4];
    Eigen::Vector3f swing_end_pos_[4];
    Eigen::Vector3f stance_start_pos_[4];
    float latched_step_height_[4];
    Eigen::Vector3f last_valid_angles_[4];

public:
    // 현재 프레임에서 도출된 4다리의 글로벌(지면) 기준 발 좌표
    std::array<Eigen::Vector3f, 4> foot_pos_global_;

    LocomotionController(
        RobotParams& params,
        GaitSequencer& seq,
        FootPosPlanner& planner,
        TrajectoryGenerator& traj,
        BodyKinematics& kin,
        IK& ik
    ) : params_(params),
        gait_sequencer_(seq),
        foot_planner_(planner),
        traj_gen_(traj),
        body_kinematics_(kin),
        ik_solver_(ik)
    {
        // 최기 상태 세팅
        for (int i = 0; i < 4; ++i) {
            prev_phase_[i] = LegPhase::STANCE;
            swing_start_pos_[i].setZero();
            swing_end_pos_[i].setZero();
            stance_start_pos_[i].setZero();
            latched_step_height_[i] = params_.MIN_STEP_HEIGHT;
            last_valid_angles_[i].setZero();
            foot_pos_global_[i].setZero();
        }
    }

    // TROT 상태에서 호출되는 보행 처리기
    // 반환값: 4개 다리(LF, RF, LH, RH) 각각의 [HAA, HFE, KFE] 모터 목표 각도 블록
    std::array<Eigen::Vector3f, 4> processTrot(float dt, const RobotCommand& cmd) {
        std::array<Eigen::Vector3f, 4> target_angles;

        // 1. 위상 및 시간 업데이트
        gait_sequencer_.update(dt, cmd);
        const auto& leg_states = gait_sequencer_.getLegStates();
        float t_cycle = gait_sequencer_.getCycleTime();
        float t_stance = t_cycle * params_.DUTY_FACTOR;

        // 2. 다리별 Edge Detection, Latch 및 글로벌 궤적 산출
        for (int i = 0; i < 4; ++i) {
            LegPhase current_phase = leg_states[i].phase;
            float s = leg_states[i].s;

            // --- Rising Edge: STANCE -> SWING ---
            // 코멘트 주신 대로: 스윙 궤적의 목표 위치와 높이는 여기서 단 1번 계산 후(Latching),
            // 스윙이 끝날 때까지 바뀌지 않습니다. (cmd가 중간에 변해도 무관함)
            if (prev_phase_[i] == LegPhase::STANCE && current_phase == LegPhase::SWING) {
                swing_start_pos_[i] = foot_pos_global_[i];
                
                // 2D 어깨 오프셋
                Eigen::Vector2f shoulder_2d(params_.shoulder_offsets[i].x(), params_.shoulder_offsets[i].y());
                
                // 목표 착지점 (Local, Base Frame) -> Global 관점이긴 하나 회전을 무시한 로컬 평면
                // 하지만 FootPlanner의 calculateTargetFootPosition는 shoulder_offset 기준으로 계산됩니다.
                Eigen::Vector3f target_offset = foot_planner_.calculateTargetFootPosition(cmd, t_cycle, shoulder_2d);
                
                // Home Stance(어깨 바로 아래 글로벌 좌표) 
                Eigen::Vector3f home_pos(params_.shoulder_offsets[i].x(), params_.shoulder_offsets[i].y(), 0.0f);
                
                // 최종 스윙 목표 지점: 기본 착지점 + 스텝 이동량
                swing_end_pos_[i] = home_pos + target_offset;
                
                // 스텝 높이 Latch
                latched_step_height_[i] = traj_gen_.calculateStepHeight(swing_start_pos_[i], swing_end_pos_[i]);
                
                ESP_LOGI("LocoCtrl", "[Leg%d] STANCE->SWING Latched. Target: (%.3f, %.3f)", i, swing_end_pos_[i].x(), swing_end_pos_[i].y());
            }

            // --- Falling Edge: SWING -> STANCE ---
            // 지면에 닿는 순간의 발 좌표를 스탠스의 시작 좌표로 Latch
            if (prev_phase_[i] == LegPhase::SWING && current_phase == LegPhase::STANCE) {
                stance_start_pos_[i] = foot_pos_global_[i];
                ESP_LOGI("LocoCtrl", "[Leg%d] SWING->STANCE Latched. Start: (%.3f, %.3f)", i, stance_start_pos_[i].x(), stance_start_pos_[i].y());
            }

            // --- 궤적 산출 (발 글로벌 좌표 갱신) ---
            if (current_phase == LegPhase::SWING) {
                foot_pos_global_[i] = traj_gen_.getSwingTrajectory(
                    swing_start_pos_[i], 
                    swing_end_pos_[i], 
                    s, 
                    latched_step_height_[i]
                );
            } else {
                // STANCE 상태: 실시간으로 변하는 cmd가 바로 궤적(역방향 밀기)에 반영됩니다.
                foot_pos_global_[i] = traj_gen_.getStanceTrajectory(
                    stance_start_pos_[i],
                    cmd,
                    s,
                    t_stance,
                    params_.shoulder_offsets[i]
                );
            }

            // 상태 업데이트
            prev_phase_[i] = current_phase;
        }

        // 3. 글로벌 -> 발 로컬 (BodyKinematics) 변환
        std::array<Eigen::Vector3f, 4> foot_pos_local;
        body_kinematics_.transformToLocal(cmd, foot_pos_global_, foot_pos_local);

        // 4. IK 연산 수행 (Hold 버퍼 포함)
        for (int i = 0; i < 4; ++i) {
            // leg_side: 왼쪽 다리(LF=0, LH=2)는 1.0f, 오른쪽 다리(RF=1, RH=3)는 -1.0f
            float leg_side = (i == 0 || i == 2) ? 1.0f : -1.0f;
            
            // knee_dir: 현재 무릎 굽힘 방향 고정 가정 (기구 모델에 따라 1.0f로 세팅)
            float knee_dir = 1.0f; 

            Eigen::Vector3f angles;
            bool ok = ik_solver_.IKsolver(foot_pos_local[i], leg_side, knee_dir, angles);

            if (!ok) {
                // IK 실패 (특이점/도달 불가) -> 직전 정상 관절 각도로 Hold
                target_angles[i] = last_valid_angles_[i];
            } else {
                // 갱신 및 정상 각도로 설정
                last_valid_angles_[i] = angles;
                target_angles[i] = angles;
            }
        }

        return target_angles;
    }

    // IDLE 상태 등에서 4개 다리를 초기 위치(Home Stance)로 리셋함.
    void initHomeStance() {
        for (int i = 0; i < 4; ++i) {
            Eigen::Vector3f home(params_.shoulder_offsets[i].x(), params_.shoulder_offsets[i].y(), 0.0f);
            foot_pos_global_[i] = home;
            prev_phase_[i] = LegPhase::STANCE;
            stance_start_pos_[i] = home;
        }
    }
};
