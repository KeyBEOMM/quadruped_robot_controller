#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>
#include "quadruped_types.h"

class IK {
private:
    RobotParams params_;

    const Eigen::Vector3f motor_dir_signs_[4] = {
        { 1.0f,  1.0f,  1.0f},  // LF
        {-1.0f, -1.0f, -1.0f},  // RF (모터가 뒤집혀 장착됨)
        {-1.0f,  1.0f,  1.0f},  // LH (HAA: YZ 평면 대칭 장착으로 반전)
        { 1.0f, -1.0f, -1.0f}   // RH (HAA: YZ 평면 대칭 장착으로 반전)
    };
    // leg_side는 leg_index로부터 결정론적으로 도출: 좌측(0,2)=+1.0, 우측(1,3)=-1.0
    static constexpr float deriveLegSide(int leg_index) {
        return (leg_index % 2 == 0) ? 1.0f : -1.0f;
    }
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
    // p_local:    어깨 기준 목표 좌표 (x, y, z)
    // leg_index:  다리 인덱스 (0=LF, 1=RF, 2=LH, 3=RH)
    //             → motor_dir_signs_ 선택 및 leg_side 자동 도출에 사용
    // knee_dir:   무릅 굽힘 방향. '<' 모양이면 1.0, '>' 모양이면 -1.0 (다리 조립 상태에 따라 고정)
    // out_angles: 계산된 3개의 모터 각도 (물리 모터 방향 부호 적용 완료)
    // 반환값:     계산 성공 시 true, 도달 불가/특이점 시 false
    IK(const RobotParams& params) : params_(params) {
        L_1 = params_.HFE_OFFSET;
        L_2 = params_.KNE_OFFSET + params_.FOOT_OFFSET;
        HAA_Y = params_.HAA_OFFSET_Y;
        HAA_Z = params_.HAA_OFFSET_Z;

        L_1_sq = L_1 * L_1;
        L_2_sq = L_2 * L_2;
        HAA_Y_sq = HAA_Y * HAA_Y;
        HAA_Z_sq = HAA_Z * HAA_Z;
        L_HAA_sq = HAA_Y_sq + HAA_Z_sq;
    }

    bool IKsolver(const Eigen::Vector3f& p_local, int leg_index, float knee_dir, Eigen::Vector3f& out_angles) {
        // leg_side는 leg_index로부터 자동 도출 (A-1: 파라미터 중복 제거)
        const float leg_side = deriveLegSide(leg_index);
        float x = p_local.x();
        float y = p_local.y();
        float z = p_local.z();

        float L_HAA = std::sqrt(L_HAA_sq);
        float yz_dist_sq = y*y + z*z;
        float yz_dist = std::sqrt(yz_dist_sq);

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
        
        // 물리적 모터 장착 방향 부호 반영:
        // LF/LH(0,2)는 기준 방향(+1), RF/RH(1,3)는 뒤집혀 장착(-1)
        out_angles = Eigen::Vector3f(theta0, theta1, theta2).cwiseProduct(motor_dir_signs_[leg_index]);
        return true; 
    }

};