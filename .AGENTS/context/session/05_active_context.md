# 5. Active Context (현재 작업 상태)

> 이 파일은 매 작업 세션 종료 시 에이전트가 자동으로 갱신한다.
> 새 대화 시작 시 반드시 이 파일을 가장 먼저 읽어야 한다.

## 현재 Phase
Phase 6: INIT / TRANSITION / ERROR 보간

## 마지막 작업 요약 (2026-05-07)

### 후방 HAA 모터 YZ 평면 대칭 장착 반영 ✅
- **하드웨어 설계 확정:** LH/RH HAA 모터 샤프트가 YZ 평면 대칭 (전방 +X, 후방 -X) — 변경 불가
- `include/IK.h` `motor_dir_signs_`: LH→{-1,+1,+1}, RH→{+1,-1,-1}
- `include/HardwareOutput.h` `motor_dir`: LH→{-1,+1,+1}, RH→{+1,-1,-1}

### HITL-1 Zero-Offset 재교정 완료 ✅
- 12채널 전 서보 재교정 완료 (HAA 방향 반전 후 재측정). `test/calibration_log.json` 갱신.
- `include/servo_config.h` 파이썬 스크립트 자동 갱신 완료.

```
ZERO_OFFSET[4][3] (단위: 도):
  LF: { 0.00,  0.00,  0.36}
  RF: { 0.00, -2.34,  0.72}
  LH: { 1.44,  0.00, -0.54}
  RH: { 3.60, -6.12, -2.70}
```

### 기구학 설계 확정 및 코드 수정 ✅

**1. knee_dir 버그 수정 (`include/LocomotionController.h`)**
- `knee_dir = -1.0f` (> 형태, 오설정) → `+1.0f` (< 형태, 포유류형) 수정
- API_REFERENCE.md 설계 기준과 일치

**2. mount_offset_rad 확정 (`include/HardwareOutput.h`)**

| 관절 | mount_offset | 근거 |
|---|---|---|
| HAA | `π/2` (90°) | 대칭 중립 |
| HFE | `2π/3` (120°) | 30° 후방 편향 — 트롯 스탠딩(θ=-60°) → 서보 60° |
| KFE | `π` (180°) | 특이점(완전 신장=θ=0) → 서보 180°→175° 클램핑 차단 |

**3. `pio run -e esp32dev` 빌드 검증 ✅ SUCCESS**

### 기구학 지식 정리
- `<` 형태(후방 무릎) 트롯 스탠딩: theta1 ≈ -60°, theta2 ≈ -105°
- 트롯 보행 중 theta1 실사용 범위: **-78° ~ -42°** (항상 음수, 전방 범위 미사용)
- motor_dir_signs_(IK) × motor_dir(HW) = RF/RH에서 상쇄 → 양쪽 대칭 서보 90°
- LH/RH HAA는 전방과 달리 양쪽 모두 부호 반전 → 후방 다리 HAA 단독 반전 구조
- `03_locomotion_kinematics.md` 섹션 3.3에 전체 정리 완료

## Phase 5 HITL 완료 요약 (2026-05-07)

### HITL-2: 서보 동작 범위 육안 확인 ✅
- 12채널 45°~135° 범위 육안 확인 완료
- 교정 모드에서 좌우 반전 동작 확인 (motor_dir 미적용 상태 — 정상)

### HITL-3: Clamping 검증 ✅ (버그픽스 포함)
- **버그:** `src/main_calibration.cpp` 상한 180.0f → **175.0f 수정** 완료
- **버그:** `include/HardwareOutput.h` `max_angle_deg` 180.0f → **175.0f 수정** 완료
- 200° 명령 → 175° 정지 확인

### HITL-4/5: Rate Limiting / Deadband
- 교정 펌웨어에서는 `writeJoints()` 미사용으로 검증 불가
- **Phase 6 INIT 기립 시퀀스에서 자연 검증으로 대체 결정**

### 기타 정리
- `src_calibration/` 디렉토리 삭제 (구 버전 — 스테일)
- 실제 빌드 타겟: `src/main_calibration.cpp` (`#ifdef CALIBRATION_MODE`)

## 현재 블로커 / 미해결 이슈
- 없음

## 다음 예상 작업
1. **Phase 6** `InterpolationGenerator.h` 구현
   - INIT: prone → IDLE standing (3~5s, Smoothstep 보간)
   - TRANSITION: 현재 foot position → Home Stance
   - ERROR: 현재 pose → safe pose (빠른 collapse → Lock)
2. `[WRITE_CODE]` 지시어 수신 후 구현 착수
