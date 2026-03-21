#pragma once
#include <Eigen/Dense>
#include "quadruped_types.h"

class TrajectoryGenerator {
private:
    RobotParams params_;

public:
    TrajectoryGenerator(const RobotParams& params) : params_(params) {}

    // [Step Height Latching] 상승 에지(Rising Edge) 시점에 한 번만 호출하여 Latch할 것 - swing 한 사이클에 높이 고정
    float calculateStepHeight(const Eigen::Vector3f& start_pos, const Eigen::Vector3f& end_pos) {
        float dx = end_pos.x() - start_pos.x();
        float dy = end_pos.y() - start_pos.y();
        float stride = std::sqrt(dx * dx + dy * dy);
        float calculated_height = params_.MIN_STEP_HEIGHT + (stride * params_.HEIGHT_RATIO);
        return std::min(calculated_height, params_.MAX_STEP_HEIGHT);
    }

    // 1. 스윙(Swing) 궤적 — step_height는 상승 에지 시 calculateStepHeight()로 Latch된 상수값
    Eigen::Vector3f getSwingTrajectory(const Eigen::Vector3f& start_pos, 
                                       const Eigen::Vector3f& end_pos, 
                                       float s,
                                       float step_height) {
        
        // [시간 스케일링] 수평 보간 변수 k (Quintic Smoothstep: 6s^5 - 15s^4 + 10s^3)
        float s2 = s * s;
        float s3 = s * s * s;
        float s4 = s3 * s;
        float s5 = s4 * s;
        float k = 10.0f * s3 - 15.0f * s4 + 6.0f * s5;

        // X, Y축 평면 직선 이동
        float current_x = start_pos.x() + (end_pos.x() - start_pos.x()) * k;
        float current_y = start_pos.y() + (end_pos.y() - start_pos.y()) * k;

        // [Z축 포물선] 수직 보간 변수 (Bézier 근사: 16 * s^2 * (1-s)^2)
        float one_minus_s = 1.0f - s;
        float z_curve = 16.0f * s2 * (one_minus_s * one_minus_s);

        // Z축: 바닥에서 출발해 부드럽게 상승했다가 하강하여 착지
        float current_z = start_pos.z() + (step_height * z_curve);

        return Eigen::Vector3f(current_x, current_y, current_z);
    }

    // 2. 스탠스(Stance) 궤적: 가감속 없는 순수한 선형 등속 이동
    Eigen::Vector3f getStanceTrajectory(const Eigen::Vector3f& start_pos, 
                                        const RobotCommand& cmd, 
                                        float s, float t_stance,
                                        const Eigen::Vector3f& shoulder_offset) {
        
        float v_foot_x = cmd.vx - (cmd.wz * shoulder_offset.y());
        float v_foot_y = cmd.vy + (cmd.wz * shoulder_offset.x());

        // s는 순수한 선형 비율이므로, (s * t_stance)는 '스탠스 시작 후 지금까지 흐른 시간'
        // 발이 땅을 딛고 몸을 밀어야 하므로, 전진 속도(v)에 비례하여 발은 뒤로(-) 이동
        float current_x = start_pos.x() - (v_foot_x * s * t_stance);
        float current_y = start_pos.y() - (v_foot_y * s * t_stance);

        // 스탠스 중이므로 Z축은 지면에 단단히 고정
        float current_z = start_pos.z();

        return Eigen::Vector3f(current_x, current_y, current_z);
    }
};
