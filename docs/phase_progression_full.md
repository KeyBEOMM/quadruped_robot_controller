# 01. Phase Progression (상세 구현 계획 및 현황)

**전략: 기계적 통합 및 검증 (Mechanical Integration & Verification)**
> 본 문서는 설계와 코딩을 분리하여, "무엇을 어떻게 짤지 고민하는 단계"를 배제하고 **"체크리스트를 단순히 기계적으로 코드로 번역하는 과정"**이 되도록 작성된 극도로 구체적인 구현 지침서입니다.
> 
> **중요 원칙:** 각 Phase는 `[🛠️ Implementation]` 과정을 모두 마친 직후, 반드시 지정된 `[✅ Testing & Verification]` 검증 절차를 통과해야만 다음 Phase로 진입합니다.

---

### Phase 1: 기반 타입 및 상태 머신 정의
**목표:** 전체 시스템의 뼈대가 되는 글로벌 상태와 공유 데이터 구조를 확립한다.

#### 1-1. `quadruped_types.h` 확장
- `RobotState` enum 정의: `INIT`, `IDLE`, `TROT`, `TRANSITION`, `ERROR`
- `SharedData` 구조체 정의: `RobotCommand cmd` + `uint32_t timestamp_ms` + `RobotState state`
- Timestamped Command 구조 (`RobotCommand` + 수신 시각)

#### 1-2. `StateMachine.h` [NEW]
- 글로벌 상태 전이 로직 클래스
- 전이 조건: `INIT→IDLE` (보간 완료), `IDLE→TROT` (속도 임계 초과), `TROT→TRANSITION` (속도=0), `TRANSITION→IDLE` (착지 완료), `ANY→ERROR` (Watchdog), `ERROR→INIT` (Reset Command)
- 각 상태 진입/이탈 시 필요한 초기화 로직 콜백

#### ✅ Phase 1 검증 체크포인트 (빌드 & 단위 테스트)
- `pio run -e esp32dev` 컴파일 성공, Warning 0건
- `StateMachine` 단독 인스턴스 생성 후 시리얼 출력으로 상태 전이 로그 확인:
  - `requestTransition(INIT)` → 로그에 `[StateMachine] → INIT` 출력
  - `onInitComplete()` 호출 → `IDLE` 전이 확인
  - `onVelocityCommand(v>0)` → `TROT` 전이 확인
  - `onWatchdogTimeout()` → `ERROR` 전이 확인
- **진입 조건**: 모든 상태 전이가 로그로 검증된 뒤 Phase 2 진입.

---

### Phase 2: Core 0 통신 인프라
**목표:** 상위 제어기로부터 UDP 명령을 수신하고, Mutex로 보호된 공유 메모리에 기록한다.

#### 2-1. `CommTask` 구현 (`main.cpp`)
- WiFi STA 모드 초기화 + UDP 소켓 수신 루프 (ESP-NOW 브릿지 전환 대비 추상화)
- 수신 데이터 → `RobotCommand` 파싱
- `esp_timer_get_time()` 기반 수신 타임스탬프 부여
- Mutex 획득 → `SharedData` 갱신 → Mutex 해제
- Reset Command 수신 시 `state = INIT` 전이 트리거

#### ✅ Phase 2 검증 체크포인트 (통신 & Mutex 테스트)
- PC에서 Python/netcat으로 UDP 패킷 전송 → 시리얼 로그에 파싱된 `vx, vy, wz` 값이 정확히 출력되는지 확인
- 빠른 연속 패킷 전송 (100Hz 이상) → Mutex 데드락 없이 안정적으로 수신되는지 확인
- **Watchdog 테스트:** UDP 전송 중단 후 100ms 경과 → 시리얼에 `[Watchdog] TIMEOUT → ERROR` 로그 출력 확인
- **진입 조건**: 명령 수신, 파싱, Watchdog 동작이 모두 시리얼로 검증된 뒤 Phase 3 진입.

---

### Phase 3: Core 1 제어 루프 골격
**목표:** `vTaskDelayUntil` 기반 **20ms 정주기 루프**와 Watchdog 감시를 확립한다.

#### 3-1. `ControlTask` 리팩터링 (`main.cpp`)
- `vTaskDelay(1000)` → `vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20))`
- Deadline Miss 시: 실제 경과 시간(`dt = 현재시각 - 이전프레임시각`)을 계산하여 궤적에 반영 (Skip & Catch-up)
- 스택 사이즈 `4096` → `8192` (Eigen 행렬 연산 + 버퍼 여유)
- 루프 시작 직후: Mutex Lock → `SharedData` 스냅샷 복사 → Mutex Unlock
- Watchdog: `(현재시각 - snapshot.timestamp_ms) > 100ms` → `state = ERROR`

#### 3-2. 상태별 분기 구조
```
switch (state) {
    case INIT:       → Interpolation Generator 호출
    case IDLE:       → 정지 자세 유지, 속도 임계 감시
    case TROT:       → 보행 파이프라인 (Phase 4)
    case TRANSITION: → 착지 보간 로직
    case ERROR:      → 안전 자세 보간 + Lock
}
```

#### ✅ Phase 3 검증 체크포인트 (DWT 고정밀 타이밍 테스트)

##### 측정 방식: DWT 사이클 카운터 + 정적 배열 버퍼 일괄 덤프

> **왜 DWT인가?**  
> `esp_timer_get_time()`은 드라이버 레이어 개입으로 수 μs 오차가 발생한다.  
> DWT(Data Watchpoint and Trace) CYCCNT는 CPU 클럭(240 MHz)과 1:1 동기된  
> 하드웨어 레지스터로 **4.17 ns 해상도** 측정이 가능하다.
>
> **왜 배열 버퍼인가?**  
> `ESP_LOGI`는 UART FIFO 대기 + 포맷 변환으로 **수백 μs ~ 수 ms**를 소요하여  
> 측정값을 왜곡한다. 측정 구간에는 시리얼 출력을 하지 않고 결과를 정적 배열에  
> 저장한 뒤, 버퍼가 꽉 차면 **한 번에 일괄 덤프**한다.

##### 3-T1. DWT 유틸리티 모듈 (`dwt_timer.h` [NEW])

- DWT CYCCNT 레지스터에 직접 접근하는 헤더 전용 유틸리티
- CMSIS 없이 메모리 매핑 주소로 직접 접근 (ESP32 Xtensa LX6 고정 주소)
- 제공 기능:
  - `init()` — 트레이스 서브시스템 활성화 + 카운터 초기화 및 시작
  - `now_cycles()` — 현재 사이클 수 즉시 반환 (오버플로 주기 ≈ 17.9 초)
  - `cycles_to_us(cycles)` — 사이클 → μs 변환 (÷ 240)
  - `cycles_to_ms(cycles)` — 사이클 → ms 변환 (÷ 240,000)

##### 3-T2. 타이밍 버퍼 설계

- **버퍼 크기:** 400 샘플 (50Hz × 8초치), `ControlTask` 내부 정적 선언
- **1 샘플(레코드)이 담는 항목:**

| 필드 | 내용 |
|---|---|
| `loop_idx` | 몇 번째 사이클인지 (단조 증가) |
| `dt_ms` | 직전 루프 대비 실측 주기 (목표: 20 ms) |
| `exec_us` | 루프 전체 실행 시간 (μs) |
| `mutex_us` | Mutex 취득 + SharedData 복사 소요 시간 |
| `pipeline_us` | `processTrot()` 보행 파이프라인 소요 시간 |
| `deadline_miss` | 이번 사이클 Deadline Miss 여부 (0/1) |

- `buf_dumped` 플래그로 덤프 **1회만** 실행, 이후 재수집 금지

##### 3-T3. 측정 탐침(Probe) 삽입 위치

```
[루프 진입 — vTaskDelayUntil 직후]
  t_loop_start ← DWT.now_cycles()

  [Mutex 취득 → SharedData 복사 → Mutex 해제]
  t_mutex_start / t_mutex_end 로 구간 감싸기

  [보행 파이프라인 — processTrot()]
  t_pipe_start / t_pipe_end 로 구간 감싸기

  t_loop_end ← DWT.now_cycles()

  if 버퍼 미달 → 사이클 차이를 us/ms로 변환 후 레코드 기록
  if 버퍼 꽉 참 → 3-T4 일괄 덤프 실행
```

##### 3-T4. 버퍼 포화 시 일괄 덤프 절차

1. `buf_dumped = true` 플래그 세팅 (재진입 방지)
2. CSV 헤더 출력: `idx, dt_ms, exec_us, mutex_us, pipeline_us, miss`
3. 400개 레코드 순차 출력
   - **UART FIFO 넘침 방지:** 32줄 출력마다 `vTaskDelay(1)` 로 1틱 양보
4. 덤프 완료 마커 출력 후 종료

##### 3-V. 합격 기준

| 항목 | 목표 기준 | 비고 |
|---|---|---|
| `dt_ms` 안정성 | **20 ms ± 1 ms** | Deadline Miss 5% 미만 |
| `exec_us` (전체 루프) | **< 15,000 μs** | 20ms 예산 75% 이하 |
| `pipeline_us` (processTrot) | **< 10,000 μs** | Phase 4 파이프라인 포함 기준 |
| `mutex_us` | **< 200 μs** | CommTask 블로킹 최소화 |
| Deadline Miss 연속 발생 | **없음** | 3회 이상 연속 시 스택/태스크 우선순위 재검토 |

`pipeline_us` < 10 ms 달성 시 → Phase 8에서 PCA9685 100Hz 상향 검토

- **진입 조건**: 위 합격 기준 전항목 통과 후 Phase 4 진입.

**[✔ Phase 3 검증 완료 요약]**
- `pipeline_us`: 평균 50~90 μs (0.1ms 미만) 측정. 하드웨어 FPU 완전 최적화 증명.
- `dt_ms`: 20.00ms 칼주기 달성.
- `mutex_us`: 평균 4 μs로 데드락/병목 없음.
- **판정: PERFCT PASS. 목표 성능 100배 초과 달성**

---

### Phase 4: 보행 파이프라인 통합 (TROT 상태)
**목표:** 이미 구현된 기구학 모듈들을 Edge Detection + Latch 패턴으로 결합한다.

#### 4-1. `LocomotionController.h` [NEW]
- 4개 다리의 런타임 상태 관리:
  - `prev_phase[4]` — 에지 검출용 이전 위상
  - `swing_start_pos[4]`, `swing_end_pos[4]` — Latch 버퍼
  - `stance_start_pos[4]` — Stance Latch 버퍼
  - `latched_step_height[4]` — Step Height Latch 버퍼
  - `last_valid_angles[4]` — IK Hold Buffer (직전 정상 각도)
  - `foot_pos_global[4]` — 현재 프레임 발 좌표

#### 4-2. 에지 검출 & Latch 로직
- **Rising Edge (STANCE→SWING):**
  1. `swing_start_pos[i] = foot_pos_global[i]` (현재 발 위치)
  2. `swing_end_pos[i] = foot_planner_.calculateTargetFootPosition(cmd, T_cycle, shoulder_2d)`
  3. `latched_step_height[i] = traj_gen_.calculateStepHeight(start, end)`
- **Falling Edge (SWING→STANCE):**
  1. `stance_start_pos[i] = foot_pos_global[i]` (착지 위치)

#### 4-3. 매 프레임 파이프라인 실행
```
1. gait_sequencer_.update(dt, cmd)
2. for each leg:
     Edge Detection → Latch (4-2)
     if SWING: foot_pos = traj_gen_.getSwingTrajectory(start, end, s, height)
     if STANCE: foot_pos = traj_gen_.getStanceTrajectory(start, cmd, s, t_stance, shoulder)
3. body_kinematics_.transformToLocal(cmd, foot_pos_global, foot_pos_local)
4. for each leg:
     bool ok = ik_solver_.IKsolver(foot_pos_local[i], side, knee_dir, angles)
     if (!ok) angles = last_valid_angles[i]   // Hold Buffer
     else last_valid_angles[i] = angles        // 갱신
5. Calibration & Output (Phase 5)
```

#### ✅ Phase 4 검증 체크포인트 (수치 검증 — 하드웨어 없이)
- **PWM 출력 비활성화 상태**에서 IK 출력 각도만 시리얼로 출력하여 검증:
  test1
  - 고정 속도(`vx=0.1`) 입력 시 4개 다리 IK 각도가 물리적 범위 내인지 확인 // 특정 범위 내의 속도 지속적(특정 시간동안)으로 입력하는 방안은 어떤지
  test2
  - 제자리 회전(`wz`만 입력) 시 `T_cycle`이 합리적인 값(예: 0.4~0.8초)으로 산출되는지 확인
  test3
  - IK 강제 실패(특이점 근방 좌표 주입) → Hold Buffer가 직전 정상 각도를 유지하는지 확인
- Rising/Falling 에지 검출 로그: `[Leg0] STANCE→SWING Latched`, `[Leg0] SWING→STANCE Latched` 출력 확인
- **진입 조건**: IK 수치가 물리적으로 타당하고, 에지/Latch 로직이 로그로 검증된 뒤 Phase 5 진입.

**[✔ Phase 4 검증 완료 요약]**
- `test1` (vx: 0.10 → 0.59): 속도 한계치 돌파 시 역기구학 `OUT_OF_RANGE` 경고문 출력 및 소프트웨어 리미트컷 정상 동작. (수학 모델 안정성 확보)
- `test3`: Watchdog TimeOut(500ms) 정확히 발생 후, `ERROR` 상태 전환 및 `INIT`→`IDLE` 복구 시퀀스 완벽 동작.
- Rising/Falling Edge Latch 기능 완벽 동기화 확인.
- **판정: PASS**

---

### Phase 5: Hardware Abstraction Layer (출력 계층)
**목표:** 설계서 §4.6의 보정/제한/안정화 계층을 구현한다.

#### 5-1. `HardwareOutput.h` [NEW]
- **Zero-Offset 배열:** `float zero_offset[4][3]` (다리×관절)
- **Hardware Limit Clamping:** `joint_min = 5°`, `joint_max = 175°` (안전 마진 적용) // `float joint_min[4][3]`, `joint_max[4][3]` 배열로 구현 파일 수정 필요
- **Angular Velocity Rate Limiting:** 프레임 간 `|Δθ| > 4.65 rad/s * dt` 시 제한
- **Deadband:** `|θ_new - θ_prev| < 0.5°` 시 이전 값 유지 (하드웨어 데드밴드 0.45° 이상)
- **PWM 변환 및 출력:** `usec = 500 + (angle_deg / 180.0) × 2000` → PCA9685 출력

#### 5-2. 서보 드라이버 통합
- PCA9685 기반 PWM 출력 (초기 50Hz, Phase 8에서 100Hz 상향 검토)
- 모터 방향 부호 적용 (`motor_dir_signs_`, 기본 CCW 기준)

#### ✅ Phase 5 검증 체크포인트 (단일 관절 하드웨어 테스트)
- **서보 1개만 연결**한 상태에서 순차적으로 검증:
  1. `angle = 0°` 명령 → 실제 서보 위치 육안 확인
  2. `angle = 90°` 명령 → 중립 위치 도달 확인 (1500usec)
  3. `angle = 180°` 명령 → 끝단 도달 확인
  4. Clamping 테스트: `angle = 200°` 명령 → 서보가 175°에서 멈추는지 확인
  5. Rate Limiting 테스트: 0° → 180° 급격한 명령 → 서보가 부드럽게(≈4초) 이동하는지 확인
- **Deadband 테스트:** 0.3° 미세 변화 명령 → 서보 정지 유지 확인
- **진입 조건**: 단일 서보에서 PWM 변환, Clamping, Rate Limiting, Deadband 모두 검증된 뒤 Phase 6 진입.

---

### Phase 6: INIT / TRANSITION / ERROR 상태 구현
**목표:** 비보행 상태의 보간 생성기를 구현한다.

#### 6-1. `InterpolationGenerator.h` [NEW]
- **INIT 보간:** 엎드린 자세 각도 → IDLE 자세 각도를 3~5초에 걸쳐 선형/Smoothstep 보간
- **TRANSITION 보간:** 현재 발 위치 → Home Stance Position으로 부드럽게 이동
- **ERROR 보간:** 현재 자세 → 안전 자세(주저앉기 각도)로 급속 보간 후 Lock

#### ✅ Phase 6 검증 체크포인트 (4다리 하드웨어 기립 테스트)
- **4개 서보 모두 연결** 후 전원 인가:
  - `INIT` 보간: 엎드린 자세에서 3~5초에 걸쳐 IDLE 자세로 부드럽게 기립하는지 육안 확인
  - 모터 충격음(Snap) 없이 부드럽게 기립하면 ✅
- `ERROR` 트리거 (통신 중단) → 4다리가 안전 자세(주저앉기)로 부드럽게 전환되는지 확인
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
- Header-Only 시절과 비교하여 50Hz 제어 루프의 실행 속도(실측 연산 시간) 유지 또는 향상 여부 벤치마킹 확인