# 6. Progress Tracker

> 이 파일은 Phase 검증 완료 또는 유의미한 작업 완료 시 에이전트가 자동으로 갱신한다.

## Phase 진행 현황

- [x] **Phase 1**: 기반 타입 및 상태 머신 정의 — PASS
- [x] **Phase 2**: Core 0 통신 인프라 — PASS
- [x] **Phase 3**: Core 1 제어 루프 골격 — PERFECT PASS
  - `dt_ms`: 20.00ms 칼주기, `pipeline_us`: 50~90 μs, `mutex_us`: 4 μs
- [x] **Phase 4**: 보행 파이프라인 통합 — PASS
  - Edge/Latch 동기화 완벽, Watchdog 복구 시퀀스 정상
- [x] **Phase 5**: Hardware Abstraction Layer — PASS
  - [x] HardwareOutput.h 구현
  - [x] PCA9685.h 드라이버 구현
  - [x] 5-3a: ESP32 캘리브레이션 펌웨어 (`src/main_calibration.cpp`, `#ifdef CALIBRATION_MODE` 가드, `esp32dev_calibration` env)
  - [x] 5-3b: `test/phase5_calibration.py` PC 스크립트 (sweep_pwm 기능 추가)
  - [x] 기구학 파라미터 수정: `body_length` 0.255→0.2075m, HFE/FOOT 주석 보강
  - [x] 보드 클럭 vs 모터 영점 오차 분리(Decoupling) 아키텍처 확정 및 문서화
  - [x] 파이썬 파이프라인 리팩토링: `set_board_freq` 구현 및 `include/servo_config.h` C++ 자동 생성
  - [x] HITL-1: Zero-Offset 교정 및 C++ 헤더 생성 확인
  - [x] HITL-2: 서보별 동작 범위 육안 확인 (45°~135° 범위, 12채널)
  - [x] HITL-3: Clamping 검증 (175° 클램핑 확인) — `max_angle_deg` 180→175 버그픽스 포함
  - [~] HITL-4/5: Rate Limiting / Deadband → Phase 6 INIT 기립 시퀀스에서 자연 검증으로 대체
- [~] **Phase 6**: INIT / TRANSITION / ERROR 보간 — 구현 완료, HITL 진행 중
  - [x] InterpolationGenerator.h 구현 (INIT/TRANSITION/ERROR 3종 보간)
  - [x] main.cpp 콜백 등록 완료
  - [x] LF/LH HFE motor_dir/mount_offset 실측 기반 수정 (motor_dir=-1, HFE=53.8°, KFE=201.9°)
  - [x] 좌표계 분석 및 IK 체인 수치 검증 완료
  - [ ] **보행 파이프라인 데이터 흐름 재검증 필요** — State별 사용 모듈이 달라 호환성 체크 미완료
    - INIT: InterpolationGenerator (Cartesian IK) vs TROT: LocomotionController (GaitSequencer+Traj+IK)
    - TRANSITION: InterpolationGenerator (현재 발→Home) — foot_pos_global_ 캡처 시점 정합성 확인 필요
    - IDLE: current_hw_angles 고정 (processTrot 미호출) — TROT 재진입 시 foot_pos_global_ 초기화 경로 확인 필요
  - [ ] RF/RH KFE mount_offset 코드 수정 (180°→205.4°) — 미적용
  - [ ] LH/RH 서보 독립 교정 측정
  - [ ] INIT 초기 스냅 현상 해결 (prone start 제거 또는 완화)
  - [ ] Phase 6 HITL 전 항목 검증
- [ ] **Phase 7**: Home Stance 초기화 & 첫 실물 보행
- [ ] **Phase 8**: 최종 통합 검증 & 파라미터 튜닝
- [ ] **Phase 9**: 모듈화 (Header/CPP 분리)

## 구현 완료 모듈

| 모듈 | 헤더 | 상태 |
|---|---|---|
| quadruped_types | `quadruped_types.h` | ✅ 완료 |
| StateMachine | `StateMachine.h` | ✅ 완료 |
| CommTask | `CommTask.h` | ✅ 완료 |
| GaitSequencer | `GaitSequencer.h` | ✅ 완료 |
| FootPlanner | `FootPlanner.h` | ✅ 완료 |
| TrajectoryGenerator | `TrajectoryGenerator.h` | ✅ 완료 |
| BodyKinematics | `BodyKinematics.h` | ✅ 완료 |
| IK | `IK.h` | ✅ 완료 |
| LocomotionController | `LocomotionController.h` | ✅ 완료 |
| InterpolationGenerator | `InterpolationGenerator.h` | ✅ 구현 완료, HITL 진행 중 |
| HardwareOutput | `HardwareOutput.h` | ✅ 구현 완료, RF/RH KFE mount_offset 수정 대기 |
| PCA9685 | `PCA9685.h` | ✅ 구현 완료 |
| DWT Timer | `dwt_timer.h` | ✅ 완료 |

## 핵심 교정 데이터 (실측)

| 관절 | 스탠스 서보각 | mount_offset | motor_dir(HW) | 상태 |
|---|---|---|---|---|
| LF HFE | 114.3° | 53.8° | -1 | ✅ 적용 |
| LF KFE | 97.2° | 201.9° | +1 | ✅ 적용 |
| RF HFE | 59.6° (계산) | 120.0° | -1 | ✅ 정상 (클램핑 없음) |
| RF KFE | 100.8° (실측) | **205.4° 필요** | -1 | ❌ 미적용 (현재 180°) |
| LH HFE | LF 기준 적용 | 53.8° | -1 | ⚠️ 독립 검증 필요 |
| LH KFE | 목표 97.2° | 201.9° | +1 | ⚠️ 독립 검증 필요 |
| RH HFE | RF 기준 적용 | 120.0° | -1 | ⚠️ 독립 검증 필요 |
| RH KFE | 목표 100.8° | **205.4° 필요** | -1 | ❌ 미적용 (현재 180°) |
