# Quadruped Robot — Full System Architecture
> 작성 : 2026-04-13 | 범위: 하위 제어기(ESP32) Phase 5→END + 상위 제어기 병행 설계
> 개요 : 
    1. 현재 진행상황 체크
    2. 상위 제어 controller와 하위 제어 controller 병행 개발을 위한 아키텍처 점검 및 가능성 여부 판단
> 결과 요약 및 추가 필요 사항 :
    1. 병행 개발 가능
    2. 상위 아키텍처 기획단계 필요 : 설계를 위한 인터페이스 정의, 필요 기술 스택 검토 및 선택 가이드 제공.
    3. 상위 제어기 테스트는 하위제어기 phase 8 단계 이후 진행
---

## 0. 한눈에 보는 전체 그림

```
┌─────────────────────────────────────────────────────────────────────┐
│                        UPPER CONTROLLER                             │
│                   (PC / Jetson / RPi — ROS 2)                       │
│                                                                     │
│  ┌──────────────┐   ┌───────────────┐   ┌─────────────────────┐   │
│  │ High-Level   │   │  Path/Mission │   │  Joystick / UI      │   │
│  │  Behavior    │→  │   Planner     │→  │  Command Source     │   │
│  └──────────────┘   └───────────────┘   └─────────────────────┘   │
│             │                │                    │                │
│             └────────────────┴────────────────────┘                │
│                              │                                      │
│                   ┌──────────▼──────────┐                          │
│                   │   CommandPublisher   │   (UDP / ROS 2 topic)   │
│                   │  vx, vy, wz          │   port 12345            │
│                   │  roll, pitch, yaw    │                          │
│                   └──────────┬──────────┘                          │
└──────────────────────────────│──────────────────────────────────────┘
                               │  WiFi UDP (20-byte packet)
                               ▼
┌─────────────────────────────────────────────────────────────────────┐
│                   LOWER CONTROLLER (ESP32)                          │
│                                                                     │
│  Core 0 (CommTask)           Core 1 (ControlTask — 50Hz)           │
│  ┌──────────────────┐        ┌────────────────────────────────┐    │
│  │ UDP Recv         │        │ State Machine                  │    │
│  │ Parse RobotCmd   │ Mutex  │  INIT → IDLE → TROT            │    │
│  │ Timestamp attach │──────→│  TRANSITION → ERROR            │    │
│  │ SharedData write │        │                                │    │
│  └──────────────────┘        │ Locomotion Pipeline            │    │
│                              │  GaitSequencer                 │    │
│                              │  FootPlanner                   │    │
│                              │  TrajectoryGenerator           │    │
│                              │  BodyKinematics                │    │
│                              │  IK Solver                     │    │
│                              │  HardwareOutput                │    │
│                              │   └→ PCA9685 (I2C)             │    │
│                              └────────────────────────────────┘    │
│                                          │                          │
└──────────────────────────────────────────│──────────────────────────┘
                                           │ 12× PWM
                                           ▼
                              ┌────────────────────────┐
                              │  12× Servo (4 legs × 3) │
                              │  HAA / HFE / KFE        │
                              └────────────────────────┘
```

---

## 1. 하위 제어기 (ESP32) — Phase 5→END 로드맵

### 1.1. 잔여 Phase 개요

| Phase | 목표 | 입력 조건 |
|---|---|---|
| **5 (완료 잔여)** | HITL-3~5: Clamping / Rate Limit / Deadband 하드웨어 검증 | 서보 1개 연결 |
| **6** | INIT/TRANSITION/ERROR 보간 (`InterpolationGenerator`) | 4 서보 연결 |
| **7** | Home Stance 확정 + 첫 실물 트롯 보행 | 전체 로봇 조립 |
| **8** | 5분 내구 테스트 + 파라미터 최종 튜닝 | Phase 7 Pass |
| **9** | 모듈화: Header/CPP 분리 | Phase 8 Pass |

### 1.2. Phase 5 잔여 작업 (HITL-3~5)

```
HITL-3: Clamping
  - 명령: angle = 200° → 실물 서보 175°에서 멈춤 확인

HITL-4: Rate Limiting
  - 명령: 0° → 180° 즉시 전환 → 서보가 ≈4.65 rad/s 속도로 부드럽게 이동 확인
  - 4.65 rad/s = 전체 180°를 ≈0.67초에 이동

HITL-5: Deadband
  - 명령: 90.0° → 90.3° → 서보 무반응 확인 (0.3° < 0.5° deadband)
```

### 1.3. Phase 6 설계 포인트 (`InterpolationGenerator.h`)

```
INIT 보간:
  - 엎드린 기본 자세 → IDLE 자세 (Home Stance)
  - 시간: 3~5초, Smoothstep (S-curve) 보간
  - 완료 시: StateMachine.onInitComplete() 호출 → IDLE 전이

TRANSITION 보간:
  - 현재 발 위치 → Home Stance Position
  - 공중 다리 있을 경우: 현재 궤적 완료 후 보간 시작
  - 완료 시: StateMachine.onTransitionComplete() → IDLE 전이

ERROR 보간:
  - 현재 자세 → 안전 자세 (주저앉기 각도)
  - 시간: 0.5~1초 (신속 대피)
  - 완료 후 Lock (추가 명령 무시)
```

### 1.4. Phase 7 Home Stance 좌표

```cpp
// 어깨 오프셋 직하방, Z = default_height
home_pos[LF] = Vector3f( body_length/2,  body_width/2, default_height)
home_pos[RF] = Vector3f( body_length/2, -body_width/2, default_height)
home_pos[LH] = Vector3f(-body_length/2,  body_width/2, default_height)
home_pos[RH] = Vector3f(-body_length/2, -body_width/2, default_height)
```

### 1.5. Phase 8 튜닝 대상 파라미터

| 파라미터 | 현재값 | 튜닝 방향 |
|---|---|---|
| Rate Limit | 4.65 rad/s | 실측 모터 응답 기반 재확인 |
| Deadband | 0.5° | 채터링/발열 관찰 후 조정 |
| PCA9685 주파수 | 50Hz | 100Hz 상향 시도 (pipeline_us < 10ms 확인됨) |
| L2 IK 유효값 | ~133.4mm | Phase 8 기구학 재검증 |

---

## 2. 상위 제어기 설계

### 2.1. 역할 정의

하위 제어기(ESP32)는 **"어떻게 걷는가"** 를 담당한다.  
상위 제어기는 **"어디로, 무엇을 위해 걷는가"** 를 담당한다.

```
상위 제어기 책임 범위:
  ✅ 속도 명령 생성 (vx, vy, wz)
  ✅ 자세 명령 생성 (roll, pitch, yaw)
  ✅ 미션/경로 계획 (목표 지점 이동)
  ✅ 조이스틱 입력 → 명령 변환
  ✅ 텔레메트리 수신 및 시각화 (미래)
  ❌ 개별 관절 각도 계산 — 하위 제어기 담당
  ❌ 보행 궤적 생성 — 하위 제어기 담당
```

### 2.2. 통신 프로토콜 (UDP)

#### 하향 패킷 (상위→하위, 20 bytes)

```
Offset  Bytes  Field       Type    Unit    Range
──────────────────────────────────────────────────
0       4      vx          float   m/s     [-0.5, 0.5]
4       4      vy          float   m/s     [-0.5, 0.5]
8       4      wz          float   rad/s   [-2.0, 2.0]
12      4      roll        float   rad     [-0.3, 0.3]
16      4      pitch       float   rad     [-0.3, 0.3]
20      (yaw — 현재 미사용, 향후 확장)
```

> **현재 구현 기준:** `quadruped_types.h RobotCommand` 구조체와 1:1 매핑.  
> 패킷 손실 시 100ms 이내 재전송 없으면 ESP32 Software Watchdog → ERROR.

#### 상향 패킷 (하위→상위, 미래 텔레메트리 — Phase 8 이후)

```
추후 설계 예정:
  - 현재 RobotState (INIT/IDLE/TROT/TRANSITION/ERROR)
  - 루프 성능 지표 (pipeline_us, deadline_miss_count)
  - 각 관절 실제 출력 각도 (12 floats)
```

### 2.3. 상위 제어기 모듈 구조

```
upper_controller/
├── src/
│   ├── main.py                    # 진입점, 이벤트 루프
│   ├── command_publisher.py       # UDP 송신 (20Hz 기본)
│   ├── input/
│   │   ├── joystick_input.py      # pygame / evdev 조이스틱
│   │   └── keyboard_input.py      # 개발/디버그용 키보드
│   ├── planner/
│   │   ├── velocity_smoother.py   # 입력 가속도 제한 (Slew Rate)
│   │   └── mission_planner.py     # 목표 지점 → vx/vy/wz 변환
│   └── telemetry/
│       └── monitor.py             # ESP32 상태 수신 및 로깅
├── config/
│   └── robot_config.yaml          # IP, 포트, 속도 한계 등
└── tests/
    └── test_publisher.py          # 기존 phase4_test.py 계승
```

### 2.4. 상위-하위 동시 개발 전략

```
Phase 7 (첫 실물 보행):
  └─ 조이스틱 입력 → 상위 제어기 → UDP → ESP32 실물 트롯
     → 최초 end-to-end 완전 체계 검증

Phase 8 (파라미터 튜닝):
  └─ 상위 텔레메트리 수신 → 루프 성능 실시간 모니터링
```

---

## 3. 핵심 인터페이스 규격

### 3.1. ESP32 접속 정보

| 항목 | 값 |
|---|---|
| ESP32 IP | `secrets.ini` 참조 (Wi-Fi STA) |
| UDP 수신 포트 | 12345 |
| 송신 주기 (상위) | 20Hz (50ms) — 하위 Watchdog 100ms 기준 여유 |
| 패킷 포맷 | Little-endian float×5 (=20 bytes) |

### 3.2. RobotCommand 속도 한계

| 필드 | 소프트웨어 제한 | 하위 제어기 내 클램핑 |
|---|---|---|
| vx | ±0.5 m/s | GaitSequencer T_cycle 자동 조정 |
| vy | ±0.5 m/s | 동일 |
| wz | ±2.0 rad/s | 회전 반경 기반 T_cycle 조정 |
| roll/pitch | ±0.3 rad | BodyKinematics 변환 입력 |

---

## 4. 작업 분기 계획 (새 워크스페이스 구성)

### 4.1. 리포지토리 구조 (안)

```
Quadruped_Robot/
├── quadruped_robot_controller/   ← 현재 워크스페이스 (ESP32 하위 제어기)
│   └── (Phase 5→9 진행)
└── quadruped_upper_controller/   ← 신규 워크스페이스 (상위 제어기)
    ├── src/
    ├── config/
    └── tests/
```

### 4.2. 병행 작업 원칙

1. **인터페이스 고정 우선:** UDP 패킷 포맷 변경 시 양측 동시 수정 필요 → `§3.1` 규격을 Freeze 기준으로 삼는다.
2. **상위는 하위를 블로킹하지 않는다:** 상위 제어기 미완성이어도 하위 Phase 진행은 독립 진행.
3. **테스트 스크립트 통합:** 기존 `test/phase*.py` → 상위 제어기 `tests/` 로 점진 이관.
4. **상위 제어기 언어:** Python 3 (빠른 프로토타이핑), 미래에 필요 시 ROS 2 노드로 전환.
5. **개선 방안 제안** 개발 중 현재 구조에 대한 문제가 발견되면 적극적으로 제안. 동의 없이 임의 수정은 금물

---

## 5. 결정 보류 항목 (Decision Points)

> 아래 항목은 설계 방향이 확정되지 않아 진행 전 결정이 필요하다.

| # | 항목 | 옵션 A | 옵션 B | 선택안 |
|---|---|---|---|---|
| D1 | 상위 제어기 실행 플랫폼 | PC (개발 편의) | Jetson / RPi (탑재형) | - 우선 pc에서 개발 향후 탑재형 고려 |
| D2 | 통신 방식 장기 | WiFi UDP (현재) | ESP-NOW 브릿지 (저지연) | - 우선 udp 향후 가능하면 esp-now 고려 |
| D3 | Phase 9 모듈화 시점 | Phase 8 완료 후 | Phase 7 완료 후 병행 | 우선순위 낮음 일단 하위 제어기 개발 후 리팩토링 |
| D4 | 텔레메트리 방향 | ESP32→PC 직접 UDP | ROS 2 토픽 | 우선 udp로 진행 |
| D5 | 상위 제어기 언어 | Python | C++ (ROS 2 노드) | 개발 전 기획 진행하여 결정 필요 |
