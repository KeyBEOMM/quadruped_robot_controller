#pragma once 
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include "quadruped_types.h"

class BodyKinematics {
public:
    RobotParams params_;
    // 몸통 중심(CoM)에서 4개 다리의 어깨(HAA 관절)까지의 물리적 오프셋

public:
    BodyKinematics(const RobotParams& params) : params_(params) {};

    // 1. 몸통 자세 변화에 따른 어깨 위치 계산
    // 동체 변환 적용: 4개의 글로벌 궤적 좌표를 어깨 기준 로컬 좌표로 변환
    void transformToLocal(const RobotCommand& cmd, 
                          const std::array<Eigen::Vector3f, 4>& foot_pos_global,
                          std::array<Eigen::Vector3f, 4>& out_foot_pos_local) {
        
        // [Zero-Rotation Early Exit] roll/pitch/yaw가 모두 영에 가까웈다면
        // AngleAxisf 및 matrix()의 sin/cos 6회 호출을 생략하고 직접 번환
        // 일반 보행 중 50Hz 루프에서 ~30μs 절약
        constexpr float kAngleEpsilon = 1e-4f;
        const Eigen::Vector3f P_CoM(0.0f, 0.0f, params_.default_height);

        if (std::abs(cmd.roll)  < kAngleEpsilon &&
            std::abs(cmd.pitch) < kAngleEpsilon &&
            std::abs(cmd.yaw)   < kAngleEpsilon) {
            // 회전 없음: 단순 평행이동 + 어깨 오프셋만 적용
            for (int i = 0; i < 4; ++i) {
                out_foot_pos_local[i] = (foot_pos_global[i] - P_CoM) - params_.shoulder_offsets[i];
            }
            return;
        }

        // 1. Roll, Pitch, Yaw 지령을 기반으로 회전 행렬 R 생성 (Z-Y-X 순서) - body frame 기준
        Eigen::AngleAxisf rollAngle(cmd.roll, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAngle(cmd.pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yawAngle(cmd.yaw, Eigen::Vector3f::UnitZ());

        Eigen::Matrix3f R_body = (yawAngle * pitchAngle * rollAngle).matrix();
        
        // 로컬로 가져오기 위해 전치 행렬(Transpose == Inverse in SO(3)) 사용 - 
        Eigen::Matrix3f R_body_T = R_body.transpose();

        // 3. 4개의 다리에 대해 좌표 변환 수행
        for (int i = 0; i < 4; ++i) {
            // CoM에서 Foot Pos까지의 벡터 계산 / 파라미터로 받은 foot_pos는 몸체의 하단 평면의 global frame 기준
            Eigen::Vector3f pos_relative_to_CoM = foot_pos_global[i] - P_CoM;
            
            // 회전행렬 R은 R_gb (global to body) 임(global frame에서 body frame으로 회전) 
            // -> IK Solver는 body frame 기준(정확히는 shoulder frame) 좌표를 원하므로 R_gb의 전치행렬 R_bg를 사용하여 회전 변환 적용
            Eigen::Vector3f pos_rotated = R_body_T * pos_relative_to_CoM;

            // shoulder offset 적용하여 body frame to shoulder frame
            out_foot_pos_local[i] = pos_rotated - params_.shoulder_offsets[i];
        }
    }
};