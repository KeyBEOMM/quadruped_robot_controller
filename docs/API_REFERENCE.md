# 📖 Quadruped Robot Controller — API Reference & Architecture Guide

> **최종 갱신:** 2026-05-07  
> **대상 펌웨어:** ESP32 / ESP-IDF (FreeRTOS)  
> **이 문서의 목적:** 프로젝트에 새로 참여하거나 잠시 떠났다 돌아온 개발자가 **코드를 보지 않고도** 각 모듈의 역할, 입출력, 호출 규칙을 즉시 파악할 수 있도록 작성된 단일 참조 문서다.

---

> ⚠️ **문서 최신화 규칙**  
> 새 기능 추가, 파라미터 변경, 인터페이스 수정이 발생하면 **해당 커밋과 동시에 이 문서를 갱신**한다.  
> 코드는 진실이고, 이 문서는 코드의 거울이어야 한다. 둘이 어긋나는 순간 이 문서는 쓰레기가 된다.

> 📝 **로깅 규칙 (Logging Policy)**
> 향후 모든 로그 메시지(`Serial.print`, `ESP_LOG` 등)는 문자 깨짐 방지를 위해 반드시 **영어(English)**로 작성한다.

---

## 목차

1. [데이터 타입 및 전역 상수 (`quadruped_types.h`)](#1-데이터-타입-및-전역-상수)
2. [StateMachine (`StateMachine.h`)](#2-statemachine)
3. [GaitSequencer (`GaitSequencer.h`)](#3-gaitsequencer)
4. [FootPosPlanner (`FootPlanner.h`)](#4-footposplanner)
5. [TrajectoryGenerator (`TrajectoryGenerator.h`)](#5-trajectorygenerator)
6. [BodyKinematics (`BodyKinematics.h`)](#6-bodykinematics)
7. [IK (`IK.h`)](#7-ik-역운동학)
8. [LocomotionController (`LocomotionController.h`)](#8-locomotioncontroller)
9. [InterpolationGenerator (`InterpolationGenerator.h`)](#9-interpolationgenerator)
10. [PCA9685 & HardwareOutput (`HardwareOutput.h`)](#10-pca9685--hardwareoutput-하드웨어-출력)
11. [CommTask (`CommTask.h`)](#11-commtask-통신-인프라)
12. [FreeRTOS 핵심 API 정리](#12-freertos-핵심-api-정리)
13. [전체 데이터 흐름도](#13-전체-데이터-흐름도)

---

## 1. 데이터 타입 및 전역 상수

**파일:** [`include/quadruped_types.h`](../include/quadruped_types.h)  
**역할:** 프로젝트 전체에서 공유되는 구조체, 열거형, 상수를 한 곳에 모은 "공용어 사전". 모든 모듈이 이 파일에 의존하므로, 수정 시 파급 범위를 반드시 확인할 것.

---

### 열거형 (Enums)

#### `enum class LegPhase`
다리 한 쪽의 현재 보행 위상 상태.

| 값 | 의미 |
|---|---|
| `SWING` | 발이 공중에 있음 (이동 중) |
| `STANCE` | 발이 지면에 닿아 있음 (지지 중) |

---

#### `enum class RobotState`
로봇 전체의 글로벌 상태. `StateMachine`이 이 값을 읽고 전이(Transition)를 관리한다.

| 값 | 진입 조건 | 의미 |
|---|---|---|
| `INIT` | 전원 인가 직후 / 리셋 명령 | Soft-Start 기립 보간 진행 중 |
| `IDLE` | INIT 보간 완료 / TRANSITION 완료 | 4발 지면, 정지 대기 자세 |
| `TROT` | IDLE에서 속도 명령 수신 | 트롯 보행 파이프라인 활성 |
| `TRANSITION` | TROT에서 속도 = 0 | SWING 다리를 Home으로 착지 보간 중 |
| `ERROR` | Watchdog 타임아웃(어느 상태에서든) | 안전 잠금. 수동 리셋 명령만 탈출 가능 |

---

### 구조체 (Structs)

#### `struct LegState`
다리 1개의 실시간 상태 스냅샷.

| 필드 | 타입 | 의미 |
|---|---|---|
| `phase` | `LegPhase` | 현재 SWING / STANCE |
| `s` | `float` [0.0 ~ 1.0] | 위상 내 진행 정도. 0이면 위상 시작, 1이면 위상 끝 |

---

#### `struct RobotCommand`
상위 제어기(PC/조이스틱)에서 내려오는 속도 및 자세 명령.

| 필드 | 타입 | 단위 | 기본값 | 의미 |
|---|---|---|---|---|
| `vx` | `float` | m/s | 0.0 | 전진(+) / 후진(-) 속도 |
| `vy` | `float` | m/s | 0.0 | 좌(+) / 우(-) 측면 속도 |
| `wz` | `float` | rad/s | 0.0 | 좌회전(+) / 우회전(-) |
| `roll` | `float` | rad | 0.0 | 몸통 좌우 기울기 |
| `pitch` | `float` | rad | 0.0 | 몸통 앞뒤 들림/숙임 |
| `yaw` | `float` | rad | 0.0 | 몸통 헤딩 방향 오프셋 |

---

#### `struct RobotParams`
하드웨어 물리 치수 및 보행 파라미터. 모든 기구학 모듈이 생성자에서 이 구조체를 복사해 보관한다.

| 필드 | 단위 | 값 | 의미 |
|---|---|---|---|
| `leg_length` | m | 0.22 | 다리 전체 길이 |
| `end_leg_length` | m | 0.19 | 발 길이 (leg × cos30°) |
| `default_height` | m | 0.164 | 기본 몸통 높이 |
| `default_stride` | m | 0.095 | 기본 반보폭 (end_leg × sin30°) |
| `min/default/max_cycle_time` | s | 0.4 / 2.0 / 3.0 | 보행 1사이클 시간 범위 |
| `DUTY_FACTOR` | — | 0.5 | Stance 비율 (Trot 기준) |
| `MIN_STEP_HEIGHT` | m | 0.02 | 최소 스윙 높이 |
| `HEIGHT_RATIO` | — | 0.2 | 보폭 대비 높이 증가율 |
| `MAX_STEP_HEIGHT` | m | 0.05 | 최대 스윙 높이 |
| `body_length / width` | m | 0.255 / 0.078 | 동체 치수 |
| `shoulder_offsets[4]` | `Eigen::Vector3f` | — | 각 다리(LF/RF/LH/RH)의 어깨 위치, 동체 중심 기준 |
| `HAA_OFFSET_Y/Z` | m | 0.0605 / 0.01 | HAA 관절 오프셋 |
| `HFE_OFFSET` | m | 0.1111 | 허벅지 링크 길이 |
| `KNE_OFFSET` | m | 0.1185 | 종아리 링크 길이 |
| `FOOT_OFFSET` | m | 0.02 | 발 링크 길이 |

> **다리 인덱스 규칙:** `[0]=LF, [1]=RF, [2]=LH, [3]=RH`

---

#### `struct SharedData`
Core 0(CommTask)이 쓰고 Core 1(ControlTask)이 읽는 공유 메모리. **반드시 Mutex로 보호**해야 한다.

| 필드 | 타입 | 의미 |
|---|---|---|
| `cmd` | `RobotCommand` | 가장 최근 수신된 속도/자세 명령 |
| `timestamp_ms` | `uint32_t` | 마지막 패킷 수신 시각 (ms, esp_timer 기준) |
| `reset_requested` | `bool` | 상위 제어기에서 리셋 요청 플래그 |

> **Watchdog 규칙:** `현재시각 - timestamp_ms > WATCHDOG_TIMEOUT_MS(100ms)` 이면 ERROR 전이.

---

### 전역 상수

| 상수 | 값 | 의미 |
|---|---|---|
| `CONTROL_DT_MS` | `10` ms | 제어 루프 목표 주기 (100Hz) |
| `CONTROL_DT_S` | `0.010f` s | 제어 루프 주기 (소수 표현) |
| `WATCHDOG_TIMEOUT_MS` | `500` ms | 통신 두절 판정 임계 시간 |
| `INIT_DURATION_S` | `3.0f` s | INIT 보간 총 시간 |
| `PRONE_BODY_HEIGHT_M` | `0.05f` m | INIT 시작 CoM 높이 (엎드린 자세). IK 안전 하한 ≈ 0.038m |
| `TRANS_LIFT_HEIGHT_M` | `0.02f` m | TRANSITION 발 Z 리프트 아크 최대 높이 (발끌림 방지) |
| `TRANSITION_DURATION_S` | `0.8f` s | TRANSITION 보간 총 시간 |
| `ERROR_DURATION_S` | `1.2f` s | ERROR 보간 총 시간 |

---

## 2. StateMachine

**파일:** [`include/StateMachine.h`](../include/StateMachine.h)  
**역할:** 로봇의 글로벌 상태(`RobotState`)를 관리하는 유한 상태 기계(FSM). 외부에서 "이벤트 함수"를 호출하면 전이 규칙에 따라 상태를 바꾸고 진입 콜백을 실행한다.

```
                onInitComplete()
  [INIT] ─────────────────────────► [IDLE]
                                       │
                  onVelocityCommand()  │  onTransitionComplete()
                 ◄─────────────────────┤◄──────────────────────
                 │                                             │
              [TROT] ──onStopCommand()──► [TRANSITION]────────┘
                 │
                 │  onWatchdogTimeout() (어느 상태에서든)
                 ▼
              [ERROR] ──onResetCommand()──► [INIT]
```

### 생성자

```cpp
explicit StateMachine(RobotState initial_state = RobotState::INIT);
```
- **입력:** 초기 상태 (기본값: `INIT`)
- **동작:** 초기 상태를 설정하고 ESP 로그에 출력한다.

---

### 이벤트 함수 (상태 전이 트리거)

모든 이벤트 함수는 **전이가 허용된 상태가 아니라면 조용히 무시**한다 (에러를 던지지 않음).

| 함수 | 유효 현재 상태 | 전이 후 상태 | 언제 호출하나 |
|---|---|---|---|
| `onInitComplete()` | `INIT` | `IDLE` | Soft-Start 기립 보간이 완료됐을 때 |
| `onVelocityCommand()` | `IDLE` | `TROT` | 속도 명령이 임계값 이상일 때 |
| `onStopCommand()` | `TROT` | `TRANSITION` | 속도 명령이 0이 됐을 때 |
| `onTransitionComplete()` | `TRANSITION` | `IDLE` | SWING 다리가 모두 착지 완료됐을 때 |
| `onWatchdogTimeout()` | **어디서든** | `ERROR` | 통신 타임아웃 감지 시 (ControlTask) |
| `onResetCommand()` | `ERROR` | `INIT` | 상위 제어기에서 리셋 명령 수신 시 |

---

### 조회 함수

```cpp
RobotState getState() const;
```
- **반환:** 현재 `RobotState`

---

### 콜백 (선택 사항)

상태 진입 시 실행할 함수 포인터를 등록할 수 있다. `nullptr`이면 무시된다.

```cpp
// 사용 예시
sm.on_enter_trot_ = []() { ESP_LOGI("CB", "보행 파이프라인 초기화"); };
```

| 공개 멤버 | 실행 시점 |
|---|---|
| `on_enter_init_` | `INIT` 진입 시 |
| `on_enter_idle_` | `IDLE` 진입 시 |
| `on_enter_trot_` | `TROT` 진입 시 |
| `on_enter_transition_` | `TRANSITION` 진입 시 |
| `on_enter_error_` | `ERROR` 진입 시 |

---

## 3. GaitSequencer

**파일:** [`include/GaitSequencer.h`](../include/GaitSequencer.h)  
**역할:** 속도 명령을 받아 보행 사이클 시간(`T_cycle`)을 결정하고, 매 제어 주기마다 4개 다리 각각의 위상 상태(`LegState`)를 업데이트한다. **"보행의 심장박동기"**.

### 생성자

```cpp
GaitSequencer(const RobotParams& params);
```
- 모든 다리를 `STANCE`, `s=0`으로 초기화.
- `T_cycle`은 `max_cycle_time(3.0s)`으로 시작.

---

### 주요 함수

#### `update(float dt, const RobotCommand& cmd)` — 매 루프 1회 호출
```
입력: dt [s] — 이전 루프 이후 경과 시간
      cmd    — 현재 속도 명령
동작: updateCycleTime(cmd) → updatePhase(dt) 순서로 호출
출력: 없음 (내부 leg_states_ 갱신)
```

#### `updateCycleTime(const RobotCommand& cmd)` — update() 내부 호출
각 다리의 합성 선속도 `V_foot = sqrt((vx - wz*r_y)² + (vy + wz*r_x)²)` 중 최대값을 기준으로 `T_cycle`을 산출한다.  
제자리 회전(`vx=vy=0, wz≠0`)도 올바르게 처리된다.

```
저속 : T_cycle = default_cycle_time (2.0s)
고속 : T_cycle = default_stride × 4 / max_foot_speed (보폭 제한선)
정지 : T_cycle = max_cycle_time (3.0s)
→ 항상 [min_cycle_time, max_cycle_time] 범위로 클램핑
```

#### `updatePhase(float dt)` — update() 내부 호출
`current_time_`을 `dt`만큼 누적하고, 각 다리의 위상 `φ = fmod((t/T_cycle + offset), 1.0)`을 계산해 `LegState`를 갱신한다.

| 다리 | 위상 오프셋 | Trot에서의 쌍 |
|---|---|---|
| LF (0) | 0.0 | LF + RH = 동시 SWING |
| RF (1) | 0.5 | RF + LH = 동시 SWING |
| LH (2) | 0.0 | |
| RH (3) | 0.5 | |

`φ < DUTY_FACTOR(0.5)` → `STANCE`, 나머지 → `SWING`  
`s` = 각 위상 내 [0, 1] 정규화된 진행 비율.

---

### 조회 함수

```cpp
const std::array<LegState, 4>& getLegStates() const;
const float getCycleTime() const;
```

---

## 4. FootPosPlanner

**파일:** [`include/FootPlanner.h`](../include/FootPlanner.h)  
**역할:** **Raibert Heuristic**을 이용해 스윙 다리의 목표 착지점(Landing Target)을 계산한다. 상승 에지(Rising Edge) 시점에 **딱 한 번** 호출하고 결과를 Latch해야 한다 (스윙 도중 재호출 금지).

---

### 주요 함수

#### `calculateTargetFootPosition(cmd, t_cycle, shoulder_offset)` — 권장
```
입력: cmd             — 현재 속도 명령 (vx, vy, wz)
      t_cycle         — 현재 사이클 시간 (GaitSequencer에서 획득)
      shoulder_offset — 해당 다리의 어깨 2D 오프셋 (Eigen::Vector2f)
반환: Eigen::Vector3f — 어깨 기준 목표 착지점 (x, y, z=0)
```

**내부 계산:**
1. 다리별 합성 속도: `v_foot = (vx - wz*r_y, vy + wz*r_x)` — 제자리 회전 대응
2. Raibert Heuristic: `step = v_foot * t_stance / 2` (`t_stance = T_cycle * DUTY_FACTOR`)
3. 보폭 방향 유지 Clamping: 크기가 `default_stride`를 초과하면 방향은 그대로, 크기만 축소

#### `calculateTargetFootPositions(cmd, t_cycle, out_targets)` — 단순화 버전 (회전 미고려)
어깨 오프셋 없이 `vx/vy`만으로 보폭 계산. 제자리 회전 시 부정확하므로 **위 함수를 권장**.

---

## 5. TrajectoryGenerator

**파일:** [`include/TrajectoryGenerator.h`](../include/TrajectoryGenerator.h)  
**역할:** 상승 에지에서 Latch된 시작/종료 위치와 위상 `s`를 받아, 각 프레임의 발 위치를 보간 궤적으로 계산한다.

---

### 주요 함수

#### `calculateStepHeight(start_pos, end_pos)` — 상승 에지에서 1회만 호출
```
입력: start_pos, end_pos — Eigen::Vector3f, 스윙 시작/목표 위치
반환: float — 스윙 최고 높이 (m)
계산: MIN_STEP_HEIGHT + stride_2d * HEIGHT_RATIO, MAX_STEP_HEIGHT로 클램핑
```
> ⚠️ **Latching 규칙:** 연산 결과를 변수에 저장하고, 해당 스윙이 끝날 때까지 재호출하지 않는다 (상승 에지에서만 호출).

---

#### `getSwingTrajectory(start_pos, end_pos, s, step_height)` — 매 루프(SWING 다리)
```
입력: start_pos, end_pos — Latch된 위치
      s                  — 스윙 위상 [0, 1]
      step_height        — Latch된 스텝 높이
반환: Eigen::Vector3f — 이번 프레임의 발 위치
```

**보간 방식:**
- **XY (수평):** Quintic Smoothstep `k = 10s³ - 15s⁴ + 6s⁵` — 시작/끝 속도 = 0 보장 (부드러운 가감속)
- **Z (수직):** Bézier 근사 `z = step_height × 16s²(1-s)²` — 중간에 정점, 시작/끝에서 0

---

#### `getStanceTrajectory(start_pos, cmd, s, t_stance, shoulder_offset)` — 매 루프(STANCE 다리)
```
입력: start_pos      — Falling Edge에서 Latch된 착지 위치
      cmd            — 현재 속도 명령
      s              — 스탠스 위상 [0, 1] (선형)
      t_stance       — T_cycle * DUTY_FACTOR
      shoulder_offset — 어깨 3D 오프셋
반환: Eigen::Vector3f — 이번 프레임의 발 위치 (지면에 고정)
```

**내부 계산:** 발이 땅을 밀어 몸을 앞으로 이동시키므로, 발은 `v_foot * s * t_stance` 만큼 **뒤로** 이동. Z는 고정.

---

## 6. BodyKinematics

**파일:** [`include/BodyKinematics.h`](../include/BodyKinematics.h)  
**역할:** 보행 파이프라인이 계산한 4개 발의 **글로벌 좌표**를, 동체의 Roll/Pitch/Yaw 자세를 반영하여 **각 어깨(HAA 관절) 기준 로컬 좌표**로 변환한다. IK Solver 직전에 호출해야 한다.

---

### 주요 함수

#### `transformToLocal(cmd, foot_pos_global, out_foot_pos_local, body_height = -1.0f)`
```
입력: cmd                — roll, pitch, yaw 명령값 포함
      foot_pos_global[4] — 발 위치 (글로벌 좌표, Eigen::Vector3f 배열)
      body_height        — CoM 높이 오버라이드 (m). 0 이하이면 params_.default_height 사용.
                           기본값 -1.0f (= default_height 사용). 하위 호환 유지.
출력: out_foot_pos_local[4] — 어깨 기준 로컬 좌표 (IK 입력용)
```

> **body_height 사용처:** `InterpolationGenerator`의 INIT 보간에서 `PRONE_BODY_HEIGHT_M → default_height` 로 넘겨 기립 효과를 낸다. 보행 파이프라인(`processTrot`)은 기본값을 그대로 사용한다.

**좌표 변환 순서:**
1. Z-Y-X 오일러각으로 회전 행렬 `R_body` 생성
2. `R_body_T = R_body.transpose()` (역회전, SO(3)에서 전치 = 역행렬)
3. `pos_relative_to_CoM = foot_pos_global[i] - P_CoM` (CoM은 Z=`body_height`)
4. `pos_rotated = R_body_T * pos_relative_to_CoM`
5. `out_local[i] = pos_rotated - shoulder_offsets[i]` (어깨 기준화)

---

## 7. IK (역운동학)

**파일:** [`include/IK.h`](../include/IK.h)  
**역할:** 어깨 기준 목표 발 좌표 `(x, y, z)`로부터 3개 관절 `(HAA, HFE, KFE)`의 목표 각도를 기하학적으로 역산한다.

```
관절 순서: HAA(Hip Abduction/Adduction) → HFE(Hip Flexion/Extension) → KFE(Knee Flexion/Extension)
출력 각도 단위: radian
```

---

### 생성자

```cpp
IK(const RobotParams& params);
```
`RobotParams`에서 링크 길이(`L_1=HFE_OFFSET`, `L_2=KNE+FOOT_OFFSET`), HAA 오프셋 등을 추출해 제곱값까지 미리 계산해 둔다 (런타임 연산 절약).

---

### 주요 함수

#### `IKsolver(p_local, leg_side, knee_dir, out_angles)`
```
입력: p_local    — Eigen::Vector3f, 어깨 기준 목표 발 좌표 (BodyKinematics 출력)
      leg_side   — +1.0f (왼쪽 다리), -1.0f (오른쪽 다리)
      knee_dir   — +1.0f ('<' 모양 굽힘), -1.0f ('>' 모양 굽힘)
출력: out_angles — Eigen::Vector3f & (θ_HAA, θ_HFE, θ_KFE) [rad]
반환: bool       — true: 성공, false: 도달 불가 / 특이점
```

**반환값이 `false`일 때:** 절대 `out_angles`를 모터에 적용하지 말 것. **Hold Buffer**(이전 프레임의 정상 각도)를 사용해야 한다.

**특이점 방어:**
- `yz_dist < |L_HAA|` → HAA 오프셋 내부: 즉시 `false`
- `d_xz > L_1 + L_2` or `d_xz < |L_1 - L_2|` → 도달 범위 이탈: 즉시 `false`
- `acos` 인자 → `[-1, 1]` 강제 클램핑 (NaN 폭발 방지)

---

### 다리별 고정 파라미터 (호출 시 사용)

| 다리 | 인덱스 | `leg_side` | `knee_dir` |
|---|---|---|---|
| LF | 0 | +1.0 | +1.0 |
| RF | 1 | -1.0 | +1.0 |
| LH | 2 | +1.0 | +1.0 |
| RH | 3 | -1.0 | +1.0 |

> `motor_dir_signs_`(하드웨어 부착 방향 보정)는 IK.h 내부에 정의되어 있으며 `IKsolver` 출력에 적용된다. HardwareOutput의 `motor_dir`와 동일한 부호를 가지므로 두 번 적용되어 상쇄 → 최종 물리 변환은 `mount_offset + theta_raw + zero_offset` 으로 귀결된다.

---

## 8. LocomotionController

**파일:** [`include/LocomotionController.h`](../include/LocomotionController.h)  
**역할:** 보행 파이프라인의 핵심 제어기. 하위 기구학 모듈들을 묶어, 에지 검출(STANCE↔SWING)에 따른 Latch(고정)를 처리하고 최종 4다리의 역운동학(IK) 관절 각도를 도출합니다.

---

### 생성자

```cpp
LocomotionController(RobotParams& params, GaitSequencer& seq, FootPosPlanner& planner,
                     TrajectoryGenerator& traj, BodyKinematics& kin, IK& ik);
```
모든 하위 기구학 모듈들의 참조를 받아 내부 파이프라인을 구축합니다.

---

### 주요 함수

#### `processTrot(dt, cmd)`
```
입력: dt     — 이전 루프 이후 경과 시간
      cmd    — 현재 속도/자세 명령
반환: std::array<Eigen::Vector3f, 4> — 4개 다리의 [HAA, HFE, KFE] 관절 목표 각도
```

**파이프라인 실행 순서:**
1. `gait_sequencer_.update(dt, cmd)`: 위상 및 시간 갱신
2. 다리별 위상 에지 검출:
   - `STANCE -> SWING (Rising Edge)`: `swing_start_pos`, `swing_end_pos`, `step_height` 산출 및 Latch (스윙 공중 궤적 고정)
   - `SWING -> STANCE (Falling Edge)`: `stance_start_pos` Latch
3. 궤적 산출: `getSwingTrajectory` / `getStanceTrajectory` 호출하여 `foot_pos_global_` 결정
4. `transformToLocal()`: 글로벌 발 좌표 → 어깨 기준 로컬 좌표 변환
5. `IKsolver()`: 3개 관절 각도 산출. 도달 불가 시 `last_valid_angles_` (Hold Buffer) 사용

#### `getHomePos(int i)` — public
```
입력: i — 다리 인덱스 (0=LF, 1=RF, 2=LH, 3=RH)
반환: Eigen::Vector3f — 다리 i 의 Home Stance 글로벌 발 위치
      = (shoulder_x, shoulder_y ± HAA_OFFSET_Y, 0.0f)
```
`InterpolationGenerator::setHomePositions()` 호출 시 사용. `processTrot` 내부의 `swing_end_pos` 계산과 동일한 단일 지점(DRY).

#### `initHomeStance()`
모든 다리를 기본 착지점(Home Stance) 좌표로 리셋합니다. `IDLE` 진입 콜백에서 호출.
> Home 좌표는 각 다리의 어깨 관절 직하방 `Z = 0.0f` 지점. `default_height = 0.164m` 가 IK에서 결합되어 무릎이 약 30° 굽혀진 자연스러운 대기 자세가 산출된다.

---

## 9. InterpolationGenerator

**파일:** [`include/InterpolationGenerator.h`](../include/InterpolationGenerator.h)  
**역할:** `INIT` / `TRANSITION` / `ERROR` 세 가지 비보행 상태에서 매 루프 호출되어 관절 목표각을 보간 생성한다. 출력 형식은 `HardwareOutput::writeJoints()`에 직접 넘길 수 있는 `float[4][3]` (IK output 공간).

---

### 생성자

```cpp
InterpolationGenerator(IK& ik, BodyKinematics& bk, const RobotParams& params);
```
세 모듈의 참조를 받아 내부 IK 연산에 재사용한다. 힙 할당 없음.

---

### 초기화

#### `setHomePositions(const std::array<Eigen::Vector3f, 4>& home)`
ControlTask 초기화 단계에서 **1회만** 호출. `LocomotionController::getHomePos(i)` 로 계산한 Home 발 위치를 등록한다. 이후 모든 보간의 종점으로 사용된다.

---

### 상태 진입 시 호출 (StateMachine 콜백에서 호출)

#### `startINIT(float duration_s)`
- 엎드린 자세(CoM 높이 `PRONE_BODY_HEIGHT_M = 0.05m`)에서 기립(`default_height = 0.164m`)하는 Cartesian 보간 시작.
- 발은 `home_pos` 에 고정, CoM 높이만 Smoothstep 변화 → `transformToLocal`에 높이 주입.

#### `startTRANSITION(const std::array<Eigen::Vector3f, 4>& start_foot, float duration_s)`
- 현재 발 위치(`loco_ctrl_.foot_pos_global_`) → `home_pos` 로 이동하는 Cartesian 보간 시작.
- XY: Smoothstep 보간 / Z: `lerp + TRANS_LIFT_HEIGHT_M × sin(π·t)` 아크 추가 (발끌림 방지).

#### `startERROR(const float start_angles[4][3], float duration_s)`
- 현재 관절각(`current_hw_angles`) → `[0, 0, 0]` (prone) 로 Joint-space Smoothstep 보간 시작.
- 완료 후 `LOCKED` 상태 진입 → 이후 `update()` 호출 시 `cur_` 갱신 없이 마지막 자세 유지.

---

### 매 루프 호출

#### `update(float dt)` → `bool`
```
입력: dt — 루프 경과 시간 (초)
반환: true  — 보간 완료 (또는 LOCKED 상태)
      false — 보간 진행 중
```

| 내부 분기 | 동작 |
|---|---|
| `INIT` | `h = lerp(h_start, h_end, smoothstep(t))` → `runIK(home_pos, h)` |
| `TRANSITION` | `fp[i] = lerp(start, home, s) + z_arc` → `runIK(fp, default_height)` |
| `ERROR_COLLAPSE` | `cur_[i][j] = lerp(start, 0, s)` (IK 없음) |

> **IK 실패 보호:** `runIK` 내부에서 IK가 `false`를 반환하면 `last_valid_` (직전 유효각)를 `cur_`에 복사(Hold).

#### `getCurrentAngles(float out[4][3]) const`
현재 보간 결과를 `out` 에 복사. `hw_out.writeJoints(out)` 에 바로 전달 가능.

#### `isLocked() const` → `bool`
ERROR 보간 완료 후 Lock 상태 여부. `true` 이면 `update()` 를 호출하지 않아도 되며, 서보 토크 유지를 위해 `writeJoints()` 는 계속 호출해야 한다.

---

### 보간 곡선

| 함수 | 수식 | 특성 |
|---|---|---|
| Smoothstep | `t²(3−2t)` | 시작/끝 속도 = 0, 부드러운 S-커브 |
| Z arc (TRANSITION) | `LIFT_H × sin(π·t)` | 중간 정점, 시작/끝 Z = 0 |

---

### main.cpp 통합 패턴

```cpp
// 초기화
InterpolationGenerator interp_(ik_solver_, body_kinematics_, PARAMS_);
interp_.setHomePositions(home_arr);

// 콜백
g_sm.on_enter_init_       = [&]() { interp_.startINIT(INIT_DURATION_S); };
g_sm.on_enter_transition_ = [&]() { interp_.startTRANSITION(loco_ctrl_.foot_pos_global_, ...); };
g_sm.on_enter_error_      = [&]() { interp_.startERROR(current_hw_angles, ...); };

// 루프 (INIT / TRANSITION 동일 패턴)
if (interp_.update(dt)) g_sm.onInitComplete();
interp_.getCurrentAngles(current_hw_angles);
hw_out.writeJoints(current_hw_angles);

// ERROR (Lock 후에도 writeJoints 유지)
if (!interp_.isLocked()) interp_.update(dt);
interp_.getCurrentAngles(current_hw_angles);
hw_out.writeJoints(current_hw_angles);
```

---

## 10. PCA9685 & HardwareOutput (하드웨어 출력)

**파일:** [`include/PCA9685.h`](../include/PCA9685.h), [`include/HardwareOutput.h`](../include/HardwareOutput.h)  
**역할:** IK에서 도출된 수학적 타겟 각도를 물리적 PWM 신호로 변환하여 서보 모터를 제어하고, 기구 보호를 위한 안전장치를 수행한다.

---

### PCA9685 (저수준 I2C 통신)
- **주요 함수:** `begin(sda, scl, clk_speed)`, `setPWMFreq(freqHz)`, `setPWM(channel, on, off)`
- **특징:** ESP-IDF 기반 I2C 마스터를 생성하여 통신(기본 SDA=21, SCL=22). 디지털 서보 대응을 위해 주파수를 100Hz로 설정. 

추후 최대 : 내부 연산 주기를 250 Hz로 잡고 있음 & 그 사이의 명령은 spline 보간

---

### HardwareOutput (고수준 매핑 & 안전 제어)
역기구학 연산 결과 `target_math_angles[4][3]`를 받아 물리적 제어로 안전하게 이관.

- **Zero-Offset Matrix (`zero_offset`)**: 서보 조립 후 발생하는 물리적 영점 오차 보정 배열.
- **Direction Reversal (`motor_dir`)**: 좌우 대칭 조립으로 인한 서보 회전 방향(부호) 역전 보정 배열.
- **Rate Limiting (`max_delta_rad`)**: `|Δθ| > max_delta_rad` 시 변화량을 강제 제한하여 과도한 전류 소모 및 기어 파손 방지 (최대 4.65 rad/s 제한 보장).
- **Deadband (`deadband_rad`)**: `|Δθ| < 0.5°` 미만일 경우 이전 각도 유지 (미세 진동/채터링 원천 억제).
- **Safety Clamping**: 모터 구동 범위를 `5° ~ 175°` 로 강제 제한하여 물리적 프레임 충돌 방지.
- **PWM 변환**: 최종 물리 각도를 `500μs ~ 2500μs` 로 선형 변환하여 PCA9685 채널 `0~11`에 순차 송신.

---

## 11. CommTask (통신 인프라)

**파일:** [`include/CommTask.h`](../include/CommTask.h)  
**역할:** Core 0에서 실행되는 WiFi UDP 수신 루프. 패킷 수신 → 파싱 → `SharedData` 갱신 → 리셋 명령 처리의 전 과정을 담당한다.

atomic이나 고성능 Queue 방식 도입도 검토
---

### UDP 패킷 포맷 (Host → ESP32, 28바이트 고정, 리틀엔디언)

| 오프셋 | 크기 | 내용 |
|---|---|---|
| 0 | 4B `float` | `vx` (전진 속도, m/s) |
| 4 | 4B `float` | `vy` (측면 속도, m/s) |
| 8 | 4B `float` | `wz` (회전 속도, rad/s) |
| 12 | 4B `float` | `roll` (rad) |
| 16 | 4B `float` | `pitch` (rad) |
| 20 | 4B `float` | `yaw` (rad) |
| 24 | 1B `uint8` | `flags`: bit0 = reset_requested |
| 25~27 | 3B | padding (무시) |

---

### 네트워크 설정 상수 (수정 필요)

```cpp
#define WIFI_SSID      "YOUR_SSID"    // ← 실제 SSID로 변경
#define WIFI_PASSWORD  "YOUR_PASSWORD" // ← 실제 비밀번호로 변경
#define UDP_PORT        9870
#define UDP_PACKET_LEN  28
```

---

### 핵심 함수 및 구조체

#### `struct CommTaskParams`
`main.cpp`에서 정적으로 할당하고 `xTaskCreatePinnedToCore()`에 전달하는 파라미터 묶음.

| 필드 | 타입 | 의미 |
|---|---|---|
| `shared` | `SharedData*` | 공유 데이터 영역 포인터 |
| `mutex` | `SemaphoreHandle_t` | Mutex 핸들 |
| `sm_ptr` | `void*` | `StateMachine*` (헤더 순환 방지를 위해 void로 보관) |

> **왜 `void*`인가?** `CommTask.h`와 `StateMachine.h`가 서로를 `include`하면 순환 의존성이 생긴다. `void*`로 타입을 지우고, `commTaskRun()` 내부에서 `static_cast<StateMachine*>`로 복원한다. C의 전통적인 "타입 소거(Type Erasure)" 패턴이다.

#### `inline void commTaskRun(void* pvParameters)`
태스크 본체. `xTaskCreatePinnedToCore()`에 태스크 함수로 등록한다. 내부 동작:
1. `wifiInit()` 호출 (NVS, TCP/IP, WiFi STA 초기화)
2. UDP 소켓 생성 (`socket()`) 및 포트 바인딩 (`bind()`)
3. 무한 루프: `recvfrom()` 블로킹 수신 → `parsePacket()` → 타임스탬프 → Mutex로 `SharedData` 갱신 → 리셋 처리

#### `static bool parsePacket(buf, len, cmd, reset)`
```
입력: buf[28] — 수신 버퍼
      len     — 수신 바이트 수
출력: cmd     — 파싱된 RobotCommand
      reset   — 리셋 플래그
반환: false면 길이 부족 (패킷 버림)
```

---

## 12. FreeRTOS 핵심 API 정리

이 프로젝트에서 자주 등장하는 FreeRTOS / ESP-IDF API를 한 곳에 정리한다.

---

### 태스크 생성

```cpp
xTaskCreatePinnedToCore(
    TaskFunction_t pvTaskCode,  // 태스크 함수 포인터
    const char*    pcName,      // 디버그용 이름
    uint32_t       usStackDepth,// 스택 크기 (바이트)
    void*          pvParameters,// 태스크로 전달할 인자 (void* 타입 소거)
    UBaseType_t    uxPriority,  // 우선순위 (높을수록 먼저 실행)
    TaskHandle_t*  pvCreatedTask, // 태스크 핸들 (불필요하면 NULL)
    BaseType_t     xCoreID      // 고정할 코어 (0 또는 1)
);
```

**이 프로젝트의 태스크 배치:**

| 태스크 | 코어 | 우선순위 | 스택 | 이유 |
|---|---|---|---|---|
| `CommTask` | 0 | 1 | 6144B | WiFi 드라이버가 Core 0에 묶여 있음 |
| `ControlTask` | 1 | 2 | 8192B | 20ms 정주기 최우선, Eigen 연산 스택 필요 |

---

### 정주기 타이밍

#### ❌ `vTaskDelay(pdMS_TO_TICKS(20))`
"지금부터 20ms 후에 깨워라." 루프 내 연산 시간만큼 **주기가 밀린다** (Drift 누적).

#### ✅ `vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20))`
"마지막으로 깨어난 시각 + 20ms에 깨워라." 루프 내 연산 시간을 **자동 보상**하여 절대 주기를 유지한다.

```cpp
// 사용 패턴 (ControlTask 첫 줄에서 초기화)
TickType_t xLastWakeTime = xTaskGetTickCount(); // 현재 틱 카운트를 기준 시각으로 저장

while(true) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20)); // 여기서 정확히 20ms 주기 보장
    // ... 연산 ...
}
```

> **`xTaskGetTickCount()`:** RTOS 부팅 이후 경과한 Tick 수를 반환한다. 1 Tick = `1000 / configTICK_RATE_HZ`ms (기본 10ms).

---

### 뮤텍스 (Mutex)

**목적:** 두 코어(태스크)가 동시에 같은 변수(`SharedData`)를 읽고 쓰면 **Data Tearing**(중간 상태의 쓰레기 값 읽기)이 발생한다. Mutex는 한 번에 하나의 태스크만 임계 구역(Critical Section)에 진입하도록 잠근다.

```cpp
// 생성 (app_main에서 1회)
SemaphoreHandle_t g_mutex = xSemaphoreCreateMutex();

// Core 0: SharedData 쓰기
if (xSemaphoreTake(mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    shared->cmd = cmd;          // 임계 구역 진입
    xSemaphoreGive(mutex);      // 잠금 해제 (반드시 호출!)
}

// Core 1: SharedData 읽기 (스냅샷)
SharedData snapshot;
if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
    snapshot = g_shared;        // 구조체 전체 복사
    xSemaphoreGive(g_mutex);    // 즉시 해제 (연산은 락 밖에서)
}
```

> **핵심 규칙:** `xSemaphoreTake()` 이후 반드시 `xSemaphoreGive()`를 호출해야 한다. 호출하지 않으면 다른 태스크가 영원히 블로킹된다 (Deadlock).

---

### UDP 소켓 API

```cpp
// 소켓 생성 — OS 수준의 네트워크 파이프 생성
// AF_INET: IPv4, SOCK_DGRAM: UDP (비연결, 빠름), IPPROTO_UDP: 프로토콜 명시
int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
// 반환: 파일 디스크립터(정수). 음수면 생성 실패.

// 포트 바인딩 — 어떤 포트에서 수신할지 지정
bind(sock, (struct sockaddr*)&local_addr, sizeof(local_addr));

// 패킷 수신 (블로킹) — 패킷이 올 때까지 이 함수에서 잠들어 있음
int received = recvfrom(sock, buf, buf_len, 0, &sender_addr, &sender_len);
// 반환: 수신한 바이트 수. 음수면 에러.
```

---

### 타임스탬프

```cpp
// ESP-IDF 고정밀 타이머 (마이크로초 단위)
int64_t us = esp_timer_get_time();

// 밀리초로 변환
uint32_t ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
```

> ⚠️ Arduino의 `millis()`는 ESP-IDF 환경에서 사용 불가. 반드시 `esp_timer_get_time()`을 사용한다.

---

## 13. 전체 데이터 흐름도

```
[상위 제어기 (PC/조이스틱)]
    │ UDP 28byte
    ▼
[Core 0: CommTask — CommTask.h]
    recvfrom() 블로킹 수신
    → parsePacket() 파싱
    → esp_timer 타임스탬프
    → xSemaphoreTake(mutex)
    → SharedData 갱신 { cmd, timestamp_ms, reset_requested }
    → xSemaphoreGive(mutex)
    → reset_requested? → sm.onResetCommand()

[Core 1: ControlTask — main.cpp, 20ms 정주기]
    vTaskDelayUntil(&xLastWakeTime, 20ms)       ← 절대 주기 보장
    │
    ├─ [Step 1] xSemaphoreTake → SharedData 스냅샷 복사 → xSemaphoreGive
    ├─ [Step 2] Watchdog: now - snapshot.timestamp_ms > 100ms → sm.onWatchdogTimeout()
    ├─ [Step 3] reset_requested → sm.onResetCommand() (폴백)
    │
    └─ [Step 4] switch(sm.getState())
          │
          ├─ TROT: 보행 파이프라인 (LocomotionController 통합 완료)
          │    gait_sequencer_.update(dt, cmd)         [GaitSequencer]
          │    → getLegStates() for each leg:
          │         Edge Detection (STANCE→SWING? SWING→STANCE?)
          │         SWING: Latch start/end/height      [FootPosPlanner, TrajectoryGenerator]
          │               getSwingTrajectory(s)       [TrajectoryGenerator]
          │         STANCE: getStanceTrajectory(s)    [TrajectoryGenerator]
          │    → body_kinematics_.transformToLocal()   [BodyKinematics]
          │    → ik_solver_.IKsolver() × 4             [IK]
          │         ↳ false → Hold Buffer (직전 정상 각도 유지)
          │    → HardwareOutput (Phase 5~)
          │         Zero-Offset → Clamping → Rate Limit → Deadband → PWM
          │
          ├─ INIT (3s, Cartesian, CoM 높이 보간):
          │    InterpolationGenerator::update(dt)
          │    → runIK(home_pos, h)  h: 0.05m→0.164m Smoothstep
          │    → getCurrentAngles() → hw_out.writeJoints()
          │    → done → sm.onInitComplete() → IDLE
          │
          ├─ TRANSITION (0.8s, Cartesian, XY+Z arc):
          │    InterpolationGenerator::update(dt)
          │    → fp[i] = lerp(start, home, s) + LIFT×sin(πt)
          │    → runIK(fp, default_height)
          │    → getCurrentAngles() → hw_out.writeJoints()
          │    → done → sm.onTransitionComplete() → IDLE
          │
          ├─ ERROR (1.2s, Joint-space → prone → LOCK):
          │    InterpolationGenerator::update(dt) (isLocked 아닌 동안)
          │    → cur_[i][j] = lerp(start, 0, s)  [IK 없음]
          │    → getCurrentAngles() → hw_out.writeJoints()  (LOCK 후에도 유지)
          │
          └─ IDLE: hw_out.writeJoints(current_hw_angles) 매 프레임 유지
```

---

> 📝 **문서 기여 가이드**  
> - 새 모듈 추가 시: 해당 섹션을 동일한 포맷(`역할 → 생성자 → 주요 함수 → 입출력 표`)으로 추가한다.  
> - 파라미터 변경 시: §1의 `RobotParams` 표를 함께 갱신한다.  
> - 검증 완료된 Phase가 있다면: `§13 데이터 흐름도`에서 해당 파이프라인 주석을 업데이트한다.
