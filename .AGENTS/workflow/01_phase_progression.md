# 01. Phase Progression (상세 구현 계획 및 현황)

**전략: 기계적 통합 및 검증 (Mechanical Integration & Verification)**
> 본 문서는 설계와 코딩을 분리하여, "무엇을 어떻게 짤지 고민하는 단계"를 배제하고 **"체크리스트를 단순히 기계적으로 코드로 번역하는 과정"**이 되도록 작성된 극도로 구체적인 구현 지침서입니다.
> 
> **중요 원칙:** 각 Phase는 `[🛠️ Implementation]` 과정을 모두 마친 직후, 반드시 지정된 `[✅ Testing & Verification]` 검증 절차를 통과해야만 다음 Phase로 진입합니다.

---

> **📁 Phase 1~4 상세 구현 지침 및 검증 결과 아카이브:** [`docs/phase_progression_full.md`](../../docs/phase_progression_full.md)
>
> | Phase | 요약 | 판정 |
> |---|---|---|
> | Phase 1 | 기반 타입 (`quadruped_types.h`) 및 상태 머신 (`StateMachine.h`) 정의 | ✔ PASS |
> | Phase 2 | Core 0 통신 인프라 (`CommTask`), Mutex, Software Watchdog | ✔ PASS |
> | Phase 3 | Core 1 제어 루프 골격 (50Hz/20ms 정주기), DWT 타이밍 검증 | ✔ PERFECT PASS |
> | Phase 4 | 보행 파이프라인 통합 (Edge Detection + Latch + IK Hold Buffer) | ✔ PASS |

---

### Phase 5: Hardware Abstraction Layer (출력 계층)
**목표:** 설계서 §4.6의 보정/제한/안정화 계층을 구현한다.
**구현 상태:** 5-1~5-3b 완료 ✅ · HITL-1 진행 중 🔄 · HITL-3~5 미시작

#### 5-1. `HardwareOutput.h` [완료]
- **Zero-Offset 배열:** `float zero_offset[4][3]` (다리×관절) 
  - *조립체 기구학적 영점 편차 전용*
- **글로벌 클럭 보정:** `PCA9685_OSC_FREQ` 상수 기반 정확한 500~2500us 출력 보장
- **Hardware Limit Clamping:** `joint_min = 0°`, `joint_max = 180°`
- **Angular Velocity Rate Limiting:** 프레임 간 `|Δθ| > 4.65 rad/s * dt` 시 제한
- **Deadband:** `|θ_new - θ_prev| < 0.5°` 시 이전 값 유지 (하드웨어 데드밴드 0.45° 이상)
- **PWM 변환 및 출력:** `usec = 500 + (angle_deg / 180.0) × 2000` → PCA9685 출력

#### 5-2. 서보 드라이버 통합 [완료]
- PCA9685 기반 PWM 출력 (초기 50Hz, Phase 8에서 100Hz 상향 검토)
- 모터 방향 부호 적용 (`motor_dir_signs_`, 기본 CCW 기준)

#### 5-3. Calibration Tool [완료] — 실시간 서보 제어 도구

> **원칙:** 각도를 펌웨어에 하드코딩하여 빌드/업로드를 반복하는 방식은 **금지**한다.
> PC에서 실시간으로 명령하고, 결과를 기록하며, 코드에 자동 반영하는 도구를 먼저 만든다.

##### 5-3a. ESP32 캘리브레이션 모드 (임시 펌웨어 분기) [완료]
- `StateMachine`에 `CALIBRATION` 상태를 추가하거나, 빌드 플래그(`-D CALIBRATION_MODE`)로 분리
- UDP 수신 루프에서 **텍스트 커맨드 파싱** 지원:
  - `set_angle <servo_index> <angle_deg>` → 해당 서보를 지정 각도(0~180)로 이동, 적용된 PWM(usec) 반환
  - `set_pwm <servo_index> <usec>` → PWM 직접 제어
  - `get_status` → 전 채널의 현재 각도/PWM 상태 일괄 출력
  - `save_offset <servo_index> <measured_usec>` → 90도 센터의 영점 편차 기록
  - `set_board_freq <freq>` → PCA9685 내부 오실레이터 주파수 튜닝 (Board Error 보정용)

##### 5-3b. PC 캘리브레이션 스크립트 (`test/phase5_calibration.py`) [완료]
- UDP 소켓으로 ESP32에 텍스트 커맨드 전송
- **인터랙티브 CLI 모드:**
  ```
  > set_angle 0 90.0
  [ESP32] servo_0: angle=90.0° → pwm=1500us
  > save_offset 0 1520
  [LOG] servo_0 offset recorded: 1520us (Δ=+20us → +1.8°)
  ```
- `save_offset` 실행 시:
  1. 입력받은 `usec`를 90도 기준의 '각도 오차(Degree)'로 변환
  2. 스크립트가 기록된 데이터를 모아 **직접 `include/servo_config.h` 파일을 생성/덮어쓴다.** (에이전트 개입 불필요)
- `set_board_freq` 실행 시: 보드 주파수 상수를 포함하여 `include/servo_config.h`를 즉시 갱신

#### ✅ Phase 5 검증 체크포인트 — Human-in-the-Loop (HITL) 워크플로우

> **원칙:** "육안 확인" 같은 애매한 표현은 사용하지 않는다.
> 에이전트가 캘리브레이션 스크립트를 통해 테스트를 실행하고, 각 단계마다 사용자에게 **명시적 승인(`Y/N`)**을 요청한다.
> 사용자의 `Y` 응답이 없으면 다음 테스트로 넘어가지 않는다.

##### HITL-1. Zero-Offset 교정 (서보 1개씩 순차)

```
에이전트: set_angle 0 90.0 전송
에이전트: "서보 0이 정확히 90° 위치에 있습니까? [Y/N]"
사용자:   N
에이전트: "현재 서보의 실제 각도를 측정하거나 편차 방향을 알려주세요."
사용자:   "약 2도 시계방향으로 치우쳐있어"
에이전트: set_angle 0 88.0 전송 → 재확인 요청
사용자:   Y
에이전트: save_offset 0 <측정값> → 로그 저장 → HardwareOutput.h 자동 갱신
에이전트: "서보 0 교정 완료. 서보 1로 이동합니다."
```

##### HITL-2. 기본 동작 범위 확인 (서보별)

| 단계 | 에이전트 행동 | 사용자 응답 | 통과 기준 |
|---|---|---|---|
| 1 | `set_angle <i> 0` 전송 | "0° 위치가 맞습니까? [Y/N]" | `Y` 필수 |
| 2 | `set_angle <i> 90` 전송 | "90° 중립 위치가 맞습니까? [Y/N]" | `Y` 필수 |
| 3 | `set_angle <i> 180` 전송 | "180° 끝단 위치가 맞습니까? [Y/N]" | `Y` 필수 |

##### HITL-3. Clamping 검증

```
에이전트: set_angle <i> 200 전송 (의도적 범위 초과)
에이전트: "서보가 175° 부근에서 정지했습니까? [Y/N]"
사용자:   Y → PASS
```

##### HITL-4. Rate Limiting 검증

```
에이전트: set_angle <i> 0 전송 → 1초 대기 → set_angle <i> 180 전송
에이전트: "서보가 급격히 점프하지 않고 약 3~4초에 걸쳐 부드럽게 이동했습니까? [Y/N]"
사용자:   Y → PASS
```

##### HITL-5. Deadband 검증

```
에이전트: set_angle <i> 90.0 전송 → 1초 대기 → set_angle <i> 90.3 전송
에이전트: "서보가 0.3° 미세 변화에 반응하지 않고 정지 상태를 유지했습니까? [Y/N]"
사용자:   Y → PASS
```

##### 합격 기준

| 항목 | 조건 |
|---|---|
| Zero-Offset 교정 | 테스트 대상 서보 전체 `Y` 확인 완료 |
| 동작 범위 (0°/90°/180°) | 전 단계 `Y` |
| Clamping (175° 상한) | `Y` |
| Rate Limiting (부드러운 이동) | `Y` |
| Deadband (0.3° 무반응) | `Y` |
| `calibration_log.json` 생성 | 파일 존재 + zero_offset 값 기록 |
| `HardwareOutput.h` 자동 갱신 | `zero_offset` 배열이 측정값과 일치 |

- **진입 조건**: 위 전항목 `Y` + 코드 자동 갱신 확인 후 Phase 6 진입.

---

### Phase 6: INIT / TRANSITION / ERROR 상태 구현
**목표:** 비보행 상태의 보간 생성기를 구현한다.
**구현 상태:** 6-1~6-4 완료 ✅ · 빌드 성공 ✅ · HITL 미시작 🔄

#### 6-1. `quadruped_types.h` 상수 추가 [완료]
- `PRONE_BODY_HEIGHT_M = 0.05f` — INIT 시작 CoM 높이 (IK 안전 최솟값 ~0.038m 기준)
- `TRANS_LIFT_HEIGHT_M = 0.02f` — TRANSITION Z 사인 아크 최대 리프트 높이
- `TRANSITION_DURATION_S = 0.8f`, `ERROR_DURATION_S = 1.2f`

#### 6-2. `BodyKinematics.h` 수정 [완료]
- `transformToLocal()` 에 `float body_height = -1.0f` 파라미터 추가 (하위 호환 기본값 유지)
- 0 이하이면 `params_.default_height` 사용, 양수이면 오버라이드 → INIT 보간 시 CoM 높이 가변 처리

#### 6-3. `LocomotionController.h` 수정 [완료]
- `getHomePos(int i)` 를 `private` → `public` 으로 이동
- `InterpolationGenerator` 초기화 및 TRANSITION 콜백에서 공용으로 호출 (DRY)

#### 6-4. `InterpolationGenerator.h` [NEW · 완료]
- **INIT (Cartesian):** 발을 `home_pos` 고정, CoM 높이를 `PRONE_BODY_HEIGHT_M → default_height` Smoothstep 보간
  → `transformToLocal(body_height=h)` + IK 연산으로 매 프레임 관절각 산출 (기립 효과)
- **TRANSITION (Cartesian + Z 사인 아크):** 현재 발 위치 → `home_pos` XY Smoothstep 보간
  + `TRANS_LIFT_HEIGHT_M × sin(π·t)` Z 아크 중첩 → 발끌림 방지
- **ERROR (Joint-space + Lock):** 현재 관절각 → 0(prone) Smoothstep 보간 후 `Mode::LOCKED`
  → Lock 후 `update()` no-op, `cur_` 동결 (자세 고정)
- `Mode::IDLE / RUNNING / LOCKED` 상태 머신 내장
- IK 실패 시 `last_valid_[4][3]` hold 버퍼로 각도 유지

#### 6-5. `src/main.cpp` 통합 [완료]
- `InterpolationGenerator interp_` 선언 및 `setHomePositions()` 초기화
- `float current_hw_angles[4][3]` 버퍼 추가 — TROT/INIT/TRANSITION/ERROR 매 프레임 갱신
- 5개 상태 콜백 등록:
  - `on_enter_init_` → `interp_.startINIT(INIT_DURATION_S)`
  - `on_enter_idle_` → `loco_ctrl_.initHomeStance()`
  - `on_enter_trot_` → GaitSequencer 리셋
  - `on_enter_transition_` → `interp_.startTRANSITION(loco_ctrl_.foot_pos_global_, ...)`
  - `on_enter_error_` → `interp_.startERROR(current_hw_angles, ...)`
- 부트 직후 INIT 수동 트리거 (`g_sm.on_enter_init_()`) — 콜백 등록 전 상태 진입 대응

#### ✅ Phase 6 검증 체크포인트 (4다리 하드웨어 기립 테스트)
- **4개 서보 모두 연결** 후 전원 인가:
  - `INIT` 보간: `PRONE_BODY_HEIGHT_M(0.05m)` 높이에서 `default_height(0.164m)`까지 3초 기립, Snap 없음 확인
  - 모터 충격음(Snap) 없이 부드럽게 기립하면 ✅
- `ERROR` 트리거 (통신 중단) → 1.2초 내 prone(관절각 0) 전환 후 자세 고정 확인
- **진입 조건**: INIT 기립 + ERROR 안전 자세 전환이 실물에서 검증된 뒤 Phase 7 진입.

---

### Phase 7: Home Stance Position 초기화 & 보행 통합
**목표:** `IDLE` 진입 시 4개 다리의 기본 착지점을 확정하고, 전체 보행 파이프라인을 처음으로 실물에서 구동한다.

#### 7-1. Home Stance 계산
- 각 어깨 오프셋 직하방, Z=0 (지면) 기준 좌표
- `home_pos[i] = Vector3f(shoulder_offsets[i].x(), shoulder_offsets[i].y(), 0.0f)`
- `IDLE` 상태 진입 시 `foot_pos_global[i] = home_pos[i]`로 초기화

#### ✅ Phase 7 검증 체크포인트 (첫 실물 보행 테스트)
- `IDLE` → 저속 `vx` 명령 → `TROT` 진입 후 제자리 트롯 보행 구동:
  - 4개 다리가 대각선 쌍(Diagonal Pair)으로 교번하는지 육안 확인
  - 보행 중 로봇 동체가 앞으로 이동하는지 확인
- 속도 0 명령 → `TROT→TRANSITION→IDLE` 전이 후 안정적으로 정지하는지 확인
- 보행 중 통신 차단 → `ERROR` 진입 및 안전 자세 전환 확인
- **진입 조건**: 실물에서 기본 트롯 보행 + 정지 + 긴급 정지가 모두 동작한 뒤 Phase 8 진입.

---

### Phase 8: 최종 통합 검증 및 파라미터 튜닝
**목표:** 전체 시스템 안정성 장시간 검증 및 하드웨어 파라미터 최종 확정.

#### 8-1. 연속 보행 내구 테스트
- 5분 이상 연속 트롯 보행 → Deadline Miss 누적 횟수, Error 발생 여부 로그 확인
- 저속(vx=0.05) / 중속(vx=0.15) / 제자리 회전(wz) 각 모드 전환 테스트

#### 8-2. 파라미터 최종 튜닝
- **Rate Limit 값:** 실측 모터 응답 속도 기반으로 `4.65 rad/s` 재검증 및 조정
- **Deadband 값:** 0.5° 기준으로 채터링 여부 및 모터 발열 관찰 후 조정
- **RTOS 연산 예산:** 실측 연산 시간 < 10ms 확인 시 PCA9685 주파수 100Hz 상향 시도

#### 8-3. Angular Velocity Rate Limiting & Deadband 최종 확정
- 시뮬레이션과 실물 양쪽에서 검증된 값으로 파라미터 확정
- `// 먼저 시뮬레이션 환경에서 테스트 후 및 실제 하드웨어 테스트 후 조정 필요` 조건 이행 완료 처리

---

### Phase 9: 모듈화 및 물리적 의존성 분리 (Header/CPP 분리)
**목표:** 빠른 디버깅과 통합을 위해 Header-Only로 작성된 제어 모듈들을 `.AGENTS` 정책에 맞게 `.h` (인터페이스)와 `.cpp` (구현 로직)로 분리한다.

#### 9-1. 헤더/구현체 분리 작업
- `IK`, `FootPlanner`, `TrajectoryGenerator`, `BodyKinematics`, `GaitSequencer`, `LocomotionController` 클래스의 멤버 함수 선언만 헤더에 남긴다.
- 모든 내부 구현 로직은 `src/` 폴더 내의 각 `.cpp` 파일로 이동시킨다.

#### ✅ Phase 9 검증 체크포인트 (빌드 최적화 및 의존성 검증)
- 전체 프로젝트 클린 빌드 성공 확인 (`pio run -e esp32dev`)
- Header-Only 시절과 비교하여 50Hz 제어 루프의 실행 속도(실측 연산 시간) 유지 또는 향상 여부 벤치마킹 확인0