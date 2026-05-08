#pragma once
#include <array>
#include <cmath>
#include <cstring>
#include <Eigen/Dense>
#include "quadruped_types.h"
#include "IK.h"
#include "BodyKinematics.h"

// ============================================================
// InterpolationGenerator
// ============================================================
// 비보행 상태(INIT / TRANSITION / ERROR)의 보간 각도를 생성한다.
//
// [INIT]       Cartesian. 발을 home_pos에 고정하고 CoM 높이를
//              PRONE_BODY_HEIGHT_M → default_height 로 Smoothstep 보간.
//              → 로봇이 엎드린 자세에서 기립하는 효과.
//
// [TRANSITION] Cartesian. 현재 발 위치 → home_pos 를 XY Smoothstep 보간.
//              Z 에 TRANS_LIFT_HEIGHT_M × sin(π·t) 아크를 얹어 발끌림 방지.
//
// [ERROR]      Joint-space. 현재 관절각 → 0(prone) 을 Smoothstep 보간 후 Lock.
//              Lock 후에는 update() 가 cur_ 를 갱신하지 않아 자세를 고정한다.
//
// 출력 형식: writeJoints() 에 바로 넘길 수 있는 float[4][3] (IK output 공간).
// ============================================================
class InterpolationGenerator {
public:
    enum class Mode { IDLE, RUNNING, LOCKED };

private:
    IK&             ik_;
    BodyKinematics& body_kinematics_;
    const RobotParams& params_;

    enum class Target { NONE, INIT, TRANSITION, ERROR_COLLAPSE };

    Target  target_ = Target::NONE;
    Mode    mode_   = Mode::IDLE;
    float   t_      = 0.0f;
    float   dur_    = 1.0f;

    std::array<Eigen::Vector3f, 4> home_pos_;     // setHomePositions() 으로 1회 설정
    float   h_start_ = 0.0f, h_end_ = 0.0f;      // INIT body height 보간 구간
    std::array<Eigen::Vector3f, 4> trans_start_;  // TRANSITION 시작 발 위치
    float   err_start_[4][3];                     // ERROR 시작 관절각

    float cur_[4][3];        // 현재 프레임 출력 버퍼
    float last_valid_[4][3]; // IK 실패 시 hold 용

    // ---- 수학 유틸리티 ----
    static float smoothstep(float t) noexcept {
        t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
        return t * t * (3.0f - 2.0f * t);
    }
    static float lerpf(float a, float b, float t) noexcept {
        return a + (b - a) * t;
    }

    // ---- 내부 IK 연산 (Cartesian 모드 공통) ----
    void runIK(const std::array<Eigen::Vector3f, 4>& foot_global, float body_h) {
        RobotCommand zero_cmd;
        std::array<Eigen::Vector3f, 4> foot_local;
        body_kinematics_.transformToLocal(zero_cmd, foot_global, foot_local, body_h);

        for (int i = 0; i < 4; ++i) {
            Eigen::Vector3f angles;
            if (ik_.IKsolver(foot_local[i], i, 1.0f, angles)) {
                cur_[i][0]        = angles.x();
                cur_[i][1]        = angles.y();
                cur_[i][2]        = angles.z();
                last_valid_[i][0] = cur_[i][0];
                last_valid_[i][1] = cur_[i][1];
                last_valid_[i][2] = cur_[i][2];
            } else {
                // IK 실패: hold
                cur_[i][0] = last_valid_[i][0];
                cur_[i][1] = last_valid_[i][1];
                cur_[i][2] = last_valid_[i][2];
            }
        }
    }

public:
    InterpolationGenerator(IK& ik, BodyKinematics& bk, const RobotParams& params)
        : ik_(ik), body_kinematics_(bk), params_(params)
    {
        memset(cur_,        0, sizeof(cur_));
        memset(last_valid_, 0, sizeof(last_valid_));
        memset(err_start_,  0, sizeof(err_start_));
        home_pos_.fill(Eigen::Vector3f::Zero());
        trans_start_.fill(Eigen::Vector3f::Zero());
    }

    // ControlTask 초기화 시 1회 호출 — home 발 위치 등록
    void setHomePositions(const std::array<Eigen::Vector3f, 4>& home) {
        home_pos_ = home;
    }

    // ---- 상태 진입 시 호출 ----

    void startINIT(float duration_s) {
        target_  = Target::INIT;
        mode_    = Mode::RUNNING;
        t_       = 0.0f;
        dur_     = duration_s;
        h_start_ = PRONE_BODY_HEIGHT_M;
        h_end_   = params_.default_height;
        // prone 높이에서의 IK 결과로 last_valid_ 초기화 → 첫 프레임 hold 방지
        runIK(home_pos_, h_start_);
    }

    void startTRANSITION(const std::array<Eigen::Vector3f, 4>& start_foot, float duration_s) {
        target_      = Target::TRANSITION;
        mode_        = Mode::RUNNING;
        t_           = 0.0f;
        dur_         = duration_s;
        trans_start_ = start_foot;
    }

    // start_angles: 현재 IK output-space 관절각 (float[4][3])
    void startERROR(const float start_angles[4][3], float duration_s) {
        target_ = Target::ERROR_COLLAPSE;
        mode_   = Mode::RUNNING;
        t_      = 0.0f;
        dur_    = duration_s;
        memcpy(err_start_, start_angles, sizeof(err_start_));
        memcpy(cur_,       start_angles, sizeof(cur_));
    }

    // ---- 매 루프 호출 ----
    // 반환값: true = 완료(or locked). false = 아직 진행 중.
    bool update(float dt) {
        if (mode_ == Mode::LOCKED) return true;
        if (mode_ == Mode::IDLE)   return false;

        t_ += dt / dur_;
        const bool done = (t_ >= 1.0f);
        if (done) {
            t_    = 1.0f;
            mode_ = (target_ == Target::ERROR_COLLAPSE) ? Mode::LOCKED : Mode::IDLE;
        }

        switch (target_) {
        case Target::INIT: {
            const float h = lerpf(h_start_, h_end_, smoothstep(t_));
            runIK(home_pos_, h);
            break;
        }
        case Target::TRANSITION: {
            const float s     = smoothstep(t_);
            const float z_arc = TRANS_LIFT_HEIGHT_M * sinf(static_cast<float>(M_PI) * t_);
            std::array<Eigen::Vector3f, 4> fp;
            for (int i = 0; i < 4; ++i) {
                fp[i] = Eigen::Vector3f(
                    lerpf(trans_start_[i].x(), home_pos_[i].x(), s),
                    lerpf(trans_start_[i].y(), home_pos_[i].y(), s),
                    lerpf(trans_start_[i].z(), home_pos_[i].z(), s) + z_arc
                );
            }
            runIK(fp, params_.default_height);
            break;
        }
        case Target::ERROR_COLLAPSE: {
            const float s = smoothstep(t_);
            for (int i = 0; i < 4; ++i)
                for (int j = 0; j < 3; ++j)
                    cur_[i][j] = lerpf(err_start_[i][j], 0.0f, s);
            break;
        }
        default: break;
        }

        return done;
    }

    // 현재 보간 관절각을 out 으로 복사 (writeJoints() 에 직접 전달 가능)
    void getCurrentAngles(float out[4][3]) const {
        memcpy(out, cur_, sizeof(cur_));
    }

    bool isLocked() const { return mode_ == Mode::LOCKED; }
};
