# 3. Locomotion Kinematics & Data Pipeline

센서 피드백(IMU 등)이 배제된 ESP32의 개루프(Open-loop) 제어 한계를 극복하기 위해, 본 프로젝트에서는 `TROT` 상태에서의 보행 주기 및 궤적 관리에 있어 다음과 같은 강경한 물리적 방어 기전과 파이프라인(Pipeline)을 상정합니다.

Core 1에서 엄격한 주기로 실행되는 핵심 처리 순서이자, 도메인 지식입니다.

---

## 3.1. Core Algorithmic Mechanics (핵심 보행 방어 논리)

1.  **스윙 타겟 고정 (Static Target Latching)**
    *   **과제:** 센서가 없는 상태에서 보행 중 스윙 궤적의 목표점(목표 착지점)이 시시각각 변하면 모터 제어의 불연속성과 기구적 급기동이 발생합니다.
    *   **해결:** 다리가 땅에서 떨어지는 **상승 에지(Rising Edge, STANCE $\rightarrow$ SWING)** 검출 순간에, 당시의 '현재 명령 속도'를 기반으로 `FootPlanner`를 호출해 목표 착지점을 단 한 번만 계산하여 로컬 버퍼에 고정(Latch)합니다. 공중 체공 중에는 상위의 목표 명령어에 즉시 대응하지 않고 현재 스윙을 끝까지 완수합니다.

2.  **연산 병목 제거 (Step Height Latching)**
    *   **과제:** `sqrt()` 등 고비용 연산을 매 루프마다 4다리에 대해 수행하면 Deadline Miss가 발생합니다.
    *   **해결:** 동적 스텝 높이 산출(지형이나 이동 방향에 대비한 높이) 역시 상승 에지 순간에 **단 한 번만 계산**하고, 스윙 궤적 내내 해당 상수값을 재사용합니다.

3.  **착지 충격 방어 (Touchdown Retraction)**
    *   **과제:** 하드웨어 마찰력에 의해 바닥면에 발이 닿는 순간 속도가 `0`이 아니면 다리가 밀리거나 로봇이 요동칩니다.
    *   **해결:** 스윙 궤적의 마지막 착지 직전 타이밍(phase $s=1$)에 진입할 때, 지면 기준 발의 상대 선속도를 역산하여 `0`으로 일치시킵니다. 즉, 동체의 전진 이동 속도(`+V`)와 동일한 후퇴 속도(`-V`)를 매칭시키는 보간 다항식(Retraction velocity)을 스윙 끝단에 적용합니다.

4.  **회전 보행 주기 보정 (Rotation Cycle Compensation)**
    *   **과제:** 제자리 회전 시 동체의 회전 속도(`wz`) 자체는 낮아도, 어깨 오프셋 회전 반경(`r`)에 의해 가장 바깥쪽 발단(End-effector)의 선속도는 모터 한계를 돌파할 수 있습니다.
    *   **해결:** 보행 주기(Cycle Time) 산출 시 $Speed = \sqrt{vx^2 + vy^2}$의 평행 이동량만 보지 않습니다. 4개 다리 각각의 합성 선속도($V + wz \times r$) 중 **최댓값(Maxiumum speed)**을 찾고, 그 최대 선속도를 모터 한계와 비교하여 전체 시스템의 보행 주기를 갱신합니다.

---

## 3.2. Execution Pipeline (매 주기 실행 파이프라인 6단계)

Core 1 `ControlTask`에서 매 (20ms) 주기마다 **반드시** 이 순서대로 실행되어야 합니다.

### Step 1. Data Snapshot & Watchdog
가장 먼저 Mutex를 찰나의 순간 동안만 잠그고, 목표 속도 커맨드와 타임스탬프를 복사합니다. 100ms 이상 지연되면 `state = ERROR`로 직행합니다.

### Step 2. State Branching (상태별 분기)
`INIT`, `ERROR`, `TRANSITION` 상태는 3~4번 보행 궤적 생성을 완전히 건너뛰고 **보간 생성기(Interpolation Generator)**를 호출하여 목표 자세를 만든 뒤 바로 Step 5(IK)로 직행합니다. `TROT`일 때만 아래 절차를 밟습니다.

### Step 3. Trajectory Generation (독립 궤적 생성)
각 다리별로 고유 위상(Phase, `s`)을 계산하고 에지를 감지합니다.
*   **상승 에지 검출 시:** `start_pos`와 `end_pos`(FootPlanner 착지점), `step_height`를 Latching.
*   **하강 에지 검출 시:** 현재 착지 위치를 기반으로 새로운 STANCE `start_pos`를 Latching. 이후 속도의 반대(후방) 방향으로 선형 이동 생성.

### Step 4. Body Kinematics (동체 자세 변환)
계산된 4개의 발 지면 좌표에, 목표 자세(Roll, Pitch, Yaw) 필터를 입혀 무게 중심의 이동을 반영합니다. 최종 결과물을 각 다리의 어깨 장착점 기준 로컬 좌표(Local Coordinates)로 변환합니다.

### Step 5. Inverse Kinematics (역운동학 도출 및 방어)
루트(기저)부터 각 끝단까지 3개의 관절 각도(HAA, HFE, KFE)를 역산합니다.
*   **IK Hold Buffer 방어 체계:** 계산 중 범위를 벗어나 `NaN`이나 허수가 발생(예: 너무 멀리 뻗음)하여 IK Solver 로직이 `false`를 반환할 때, 시스템이 붕괴하지 않도록 **버퍼에 저장된 직전 프레임의 유효 각도**를 유지(Hold)하여 반환합니다.

### Step 6. Hardware Abstraction & Clamping (출력 안전망)
1.  **Calibration Decoupling:** 
    *   **Board Error (Global Clock):** PCA9685 보드의 RC 오실레이터 주파수 변동은 글로벌 `PCA9685_OSC_FREQ` 상수를 통해 보정하여, 칩 내 모든 핀이 정확한 펄스폭(500~2500us)을 출력하도록 보장합니다.
    *   **Motor Error (Zero-Offset):** 산출된 이상적인 모터 각도에 실제 하드웨어의 미세 조립 오차(90도 물리적 편차)만을 반영하기 위해 `zero_offset` 상수를 가산합니다. 개루프 제어 특성 상 끝단의 Gain 오차는 무시합니다.
2.  **HW Limit Clamping:** 기계적 한계 상하한선(예: $5^{\circ}$ ~ $175^{\circ}$)을 넘지 않게 자릅니다.
3.  **Velocity Rate Limiting:** 모터 성능을 초과하는 각속도 요구를 무시하고 1틱당 허용 한계(`dt` $\times$ 최대각속도)까지만 움직이게 제한합니다.
4.  **Deadband Filter:** $0.5^{\circ}$ 이하의 미세한 떨림은 PWM으로 번역하지 않고 무시하여 모터 채터링 및 발열을 막습니다. (이후 PCA9685 통신 수행)

---

## 3.3. IK 좌표계 및 관절 각도 규칙

### 어깨 기준 로컬 좌표계

IK solver에 입력되는 `p_local`은 **각 다리의 어깨(HAA 관절) 기준 좌표**입니다.

| 축 | 방향 |
|---|---|
| x | 전방 (rostral) |
| y | 측방 (좌측 양수) |
| z | 하방 (지면 양수, 발이 z>0) |

---

### 관절 수학 각도(Math Angle) 영점 정의

| 관절 | theta = 0° 의 물리적 의미 | IK 수식 |
|---|---|---|
| theta0 (HAA) | 다리가 시상면(x-z 평면)과 일치, 외전 없음 | `phi - alpha` |
| theta1 (HFE) | 허벅지가 수직 하방 | `gamma - knee_dir × delta` |
| theta2 (KFE) | 허벅지-종아리 완전 신장 (특이점) | `knee_dir × (beta - π)` |

> **KFE 특이점 설계 의도:** `theta2 = 0`을 완전 신장으로 정의했기 때문에, `mount_offset = 180°`으로 특이점을 서보 상한(175° 클램핑)에 가두어 하드웨어적으로 차단합니다.

---

### knee_dir 규칙

```
knee_dir = +1.0f  →  '<' 형태 (후방 무릎, 포유류형, MIT Mini Cheetah 방식) ← 채택
knee_dir = -1.0f  →  '>' 형태 (전방 무릎, Spot 방식)
```

**'<' 형태 트롯 스탠딩 기구학 (발이 어깨 바로 아래, x=0):**
```
L1 = 0.1111m,  L2 = 0.1385m,  h = 0.154m

delta = acos((L1² + h² - L2²) / (2·L1·h)) ≈ 60°
theta1 = 0 - 60° = -60°  (허벅지 60° 후방 기울기)
theta2 = -(180° - 75°)  = -105°  (약 105° 굽힘)
```

---

### 다리별 부호 규칙 (motor_dir_signs_)

IK solver는 내부적으로 모터 장착 방향 보정을 적용한 뒤 출력합니다.

```
LF / LH  (leg_index 0, 2):  × {+1, +1, +1}  — 기준 방향
RF / RH  (leg_index 1, 3):  × {-1, -1, -1}  — 좌우 대칭 반전
```

---

### 물리 서보 각도 변환 공식

`HardwareOutput::writeJoints()` 내부 변환 순서:

```
phys_rad = mount_offset_rad[i][j]
         + theta_math[i][j] × motor_dir[i][j]
         + ZERO_OFFSET[i][j] × (π/180)
```

| 항목 | 역할 |
|---|---|
| `mount_offset_rad` | IK math 0°에 대응하는 서보 물리 각도 (조립 편향 인코딩) |
| `motor_dir` | HardwareOutput 수준의 방향 보정 (+1 or -1) |
| `ZERO_OFFSET` | HITL-1 교정으로 측정된 혼 조립 오차 (단위: 도) |

---

### mount_offset 확정값 및 근거

| 관절 | mount_offset | 서보 90°(조립) 시 math angle | 근거 |
|---|---|---|---|
| HAA | `π/2` (90°) | 0° (수직 중립) | 대칭 ±90° 확보 |
| HFE | `2π/3` (120°) | -30° (허벅지 30° 후방) | 트롯 스탠딩(θ=-60°) → 서보 60°, bowing 범위 확보 |
| KFE | `π` (180°) | -90° (직각 굽힘) | 특이점(θ=0)을 서보 180°→175° 클램핑으로 차단 |

**HFE 유효 서보 범위 (mount_offset=120° 기준):**
```
서보:   5°      60°      120°      175°
         |        |         |         |
theta1: -115°   -60°      0°       +55°
               (트롯)  (수직기준)  (최대전방)
```

---

### motor_dir 이중 적용과 상쇄

IK의 `motor_dir_signs_`와 HardwareOutput의 `motor_dir`은 **동일한 값**이므로 RF/RH에서 이중으로 적용되어 결과적으로 상쇄됩니다. 양쪽 다리가 동일한 물리 서보 각도로 대칭 동작합니다.

| 다리 | IK ×signs | HW ×motor_dir | 합성 |
|---|---|---|---|
| LF/LH | ×(+1) | ×(+1) | net +1 |
| RF/RH | ×(-1) | ×(-1) | net +1 (상쇄) |
