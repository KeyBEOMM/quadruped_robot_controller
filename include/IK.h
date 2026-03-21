#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>
#include "quadruped_types.h"

class IK {
private:
    RobotParams params_;

    const float leg_side_signs_[4] = {1.0f, -1.0f, 1.0f, -1.0f}; //LF, RF, LH, RH
    const Eigen::Vector3f motor_dir_signs_[4] = {
        { 1.0f,  1.0f,  1.0f},  // LF
        {-1.0f, -1.0f, -1.0f},  // RF (모터가 뒤집혀 장착되었다고 가정)
        { 1.0f,  1.0f,  1.0f},  // LH
        {-1.0f, -1.0f, -1.0f}   // RH
    };
    float L_1;      // 허벅지 길이
    float L_2;      // 종아리 길이
    float HAA_Y;    // HAA 오프셋
    float HAA_Z;    // HAA 오프셋

    float L_1_sq;
    float L_2_sq;
    float HAA_Y_sq ;
    float HAA_Z_sq;

    float L_HAA_sq;


public:
    // 파라미터 설명:
    // p_local: 어깨 기준 목표 좌표 (x, y, z)
    // leg_side: 왼쪽 다리(1.0), 오른쪽 다리(-1.0) -> HAA 오프셋 부호 결정
    // knee_dir: 무릎 굽힘 방향. '<' 모양이면 1.0, '>' 모양이면 -1.0 (다리 조립 상태에 따라 고정)
    // out_angles: 계산된 3개의 모터 각도를 담을 참조 변수
    // 반환값: 계산 성공 시 true, 도달 불가/특이점 시 false
    IK(const RobotParams& params) : params_(params) {
        
        L_1 = params_.HFE_OFFSET; // 허벅지 길이
        L_2 = params_.KNE_OFFSET + params_.FOOT_OFFSET;   // 종아리 길이
        HAA_Y = params_.HAA_OFFSET_Y; // HAA 오프셋
        HAA_Z = params_.HAA_OFFSET_Z; // HAA 오프셋

        L_1_sq = L_1 * L_1;
        L_2_sq = L_2 * L_2;
        HAA_Y_sq = HAA_Y * HAA_Y;
        HAA_Z_sq = HAA_Z * HAA_Z;

        L_HAA_sq = HAA_Y_sq + HAA_Z_sq;
    }

    bool IKsolver(const Eigen::Vector3f& p_local, float leg_side, float knee_dir, Eigen::Vector3f& out_angles) {
        float x = p_local.x();
        float y = p_local.y();
        float z = p_local.z();

        // length of offset from HAA to HFE joint
        float L_HAA = std::sqrt(L_HAA_sq);
        float yz_dist_sq = y*y + z*z;
        float yz_dist = std::sqrt(yz_dist_sq);

        // [HAA offset 내부 및 굽힙 sigularity 방어] 목표점이 HAA 오프셋보다 안쪽에 있으면 계산 불가
        if (yz_dist < std::abs(L_HAA)) return false; 
        
        float h = std::sqrt(yz_dist_sq - HAA_Y_sq) - HAA_Z;
        float phi = std::atan2(y, -z);   
        float alpha = std::atan2(HAA_Y * leg_side, h + HAA_Z);

        float theta0 = phi - alpha; 

        // HFE, KFE in Leg plane
        float d_xz_sq = x*x + h*h; 
        float d_xz = std::sqrt(d_xz_sq);

        // pre-check for reachability
        if (d_xz > (L_1 + L_2) || d_xz < std::abs(L_1 - L_2)) return false;

        // 4. NaN 폭발 방지: acos 내부 값을 [-1.0, 1.0]으로 강제 클램핑
        float cos_delta = (L_1_sq + d_xz_sq - L_2_sq) / (2.0f * L_1 * d_xz);
        cos_delta = std::clamp(cos_delta, -1.0f, 1.0f);
        
        float cos_beta = (L_1_sq + L_2_sq - d_xz_sq) / (2.0f * L_1 * L_2);
        cos_beta = std::clamp(cos_beta, -1.0f, 1.0f);

        float gamma = std::atan2(-x, h);
        float delta = std::acos(cos_delta);       
        float theta1 = gamma - (knee_dir * delta);

        float beta = std::acos(cos_beta);    
        // knee_dir이 -1.0일 때 -255도처럼 수학적 클램핑 범위를 이탈하지 않도록 괄호로 묶어 부호 전체를 반전시킵니다.
        float theta2 = knee_dir * (beta - (float)M_PI);
        
        // (선택 사항) 물리적 모터 방향까지 여기서 적용하고 싶다면 cwiseProduct 사용
        // out_angles = Eigen::Vector3f(theta0, theta1, theta2).cwiseProduct(motor_dir_signs_[...]);

        // 결과 저장
        out_angles << theta0, theta1, theta2;
        return true; 
    }

};