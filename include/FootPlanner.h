#pragma once
#include <Eigen/Dense>
#include "quadruped_types.h"
// #include "GaitSequencer.h"

// 보행 시 Foot Position을 계획하는 클래스
// 속도 명령과 보행 파라미터 및 다리 위상 상태를 기반으로 각 다리의 목표 위치 계산

class FootPosPlanner {
private:
    RobotParams params_;

public:
    FootPosPlanner(const RobotParams& params) : params_(params) {}

    // [핵심] 각 다리의 합성 속도(Kinematic Velocity) V = v + (w x r)을 기반으로
    // Raibert Heuristic을 적용하여 스윙 목표 착지 오프셋을 계산합니다.
    // shoulder_offset: 2D 어깨 위치 (각 다리별 회전 반경 계산에 사용)
    Eigen::Vector3f calculateTargetFootPosition(const RobotCommand& cmd, float t_cycle, Eigen::Vector2f& shoulder_offset) {
        
        float t_stance = t_cycle * params_.DUTY_FACTOR;

        // 1. [핵심] 해당 다리의 합성 속도(Kinematic Velocity) 계산: V = v + (w x r)
        // 외적을 2D 평면으로 풀면 아래와 같이 교차 성분이 나옵니다.
        float v_foot_x = cmd.vx - (cmd.wz * shoulder_offset.y());
        float v_foot_y = cmd.vy + (cmd.wz * shoulder_offset.x());

        // Raibert Heuristic
        float step_x = (v_foot_x * t_stance) / 2.0f;
        float step_y = (v_foot_y * t_stance) / 2.0f;

        // 3. 벡터 크기 기반의 안전 Clamping (Heading 왜곡 방지)        
        float current_stride = std::sqrt(step_x * step_x + step_y * step_y);

        if (current_stride > params_.default_stride) {
            // 뻗으려던 방향(각도)은 그대로 유지한 채, 크기만 MAX_STRIDE로 축소(스케일링)
            float scale = params_.default_stride / current_stride;
            step_x *= scale;
            step_y *= scale;
        }

        return Eigen::Vector3f(step_x, step_y, 0.0f);
    }
};

