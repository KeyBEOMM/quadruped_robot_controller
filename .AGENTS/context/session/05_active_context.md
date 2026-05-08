# 5. Active Context (현재 작업 상태)

> 이 파일은 매 작업 세션 종료 시 에이전트가 자동으로 갱신한다.
> 새 대화 시작 시 반드시 이 파일을 가장 먼저 읽어야 한다.

## 현재 Phase
Phase 6: INIT / TRANSITION / ERROR 보간 — 구현 완료, HITL 테스트 진행 중

## 마지막 작업 요약 (2026-05-08)

### Phase 6 구현 완료 ✅
- `include/InterpolationGenerator.h` 구현 완료
  - INIT: Cartesian, h=PRONE→default_height Smoothstep 보간
  - TRANSITION: 현재 발→Home XY Smoothstep + Z sine-arc
  - ERROR: Joint-space, 현재→0 Smoothstep 후 LOCK
- `src/main.cpp` 콜백 등록 완료 (on_enter_init_, on_enter_idle_, on_enter_trot_, on_enter_transition_, on_enter_error_)
- `PRONE_BODY_HEIGHT_M = 0.10m` (LF/LH HFE 클램핑 방지)
- 빌드 성공 ✅

### LF/LH HFE 서보 혼 장착 오차 수정 ✅
- **실측 기반 재교정**: 캘리브레이션 툴로 실제 스탠스 서보각 측정
  - LF HFE 스탠스 = 114.3° → `mount_offset = 53.8°` (이전: 120°)
  - LF KFE 스탠스 = 97.2° → `mount_offset = 201.9°` (이전: 180°)
- LH에 동일값 적용 (독립 검증 미완료 — 다음 세션에서 확인 필요)
- `HardwareOutput.h` motor_dir LF/LH HFE: -1 확정 (CCW=뒤, 실측 확인)

**현재 HardwareOutput.h 확정값:**
```cpp
float motor_dir[4][3] = {
    { 1.0f, -1.0f,  1.0f}, // LF [HAA, HFE, KFE]
    {-1.0f, -1.0f, -1.0f}, // RF
    {-1.0f, -1.0f,  1.0f}, // LH
    { 1.0f, -1.0f, -1.0f}  // RH
};

float mount_offset_rad[4][3] = {
    {π/2, 53.8°,  201.9°},  // LF [HAA, HFE, KFE]
    {π/2, 120.0°, 180.0°},  // RF ← KFE 205.4°로 수정 필요
    {π/2, 53.8°,  201.9°},  // LH (LF 기준, 독립 검증 필요)
    {π/2, 120.0°, 180.0°}   // RH ← KFE 205.4°로 수정 필요
};
```

### RF/RH KFE mount_offset 소프트웨어 수정 필요 (미적용) ❌
- **실측**: RF KFE 스탠스 = 100.8° (사용자 제공)
- **계산**:
  - IK 체인: theta2_raw=-104.6°, motor_dir_signs_[RF][KFE]=-1 → IK_out=+104.6°
  - 현재 코드: 180° + 104.6°×(-1) = 75.4° (오차 25.4°)
  - 필요값: `mount_offset = 100.8° + 104.6° = 205.4°`
- RH도 동일 가정 (독립 측정 필요)

### RF/RH HFE 재장착 불필요 (오분석 정정)
- IK motor_dir_signs_[RF][HFE]=-1이 이미 방향 보상
- RF HFE 스탠스 서보 = 120° + 60.4°×(-1) = **59.6°** (정상 범위, 클램핑 없음)
- 이전 세션의 "RF HFE 175° 클램핑" 분석은 IK motor_dir_signs_ 미적용으로 인한 계산 오류

### 좌표계 분석 완료 ✅
- **전역**: X-Forward, Y-Left, Z-Up (ROS REP 103), Origin=CoM at z=h
- **어깨 프레임**: `p_local = R_body^T × (foot_global - P_CoM) - shoulder_offset`
- **IK 검증** (LF IDLE 스탠스 p_local=(0, 0.0605, -0.164)):
  - theta0=0°, theta1=-60.4°, theta2=-104.6° → 서보 114.2°/97.3° ✅ 실측 일치
- **RF 수치 체인**: theta2_IK_out=+104.6° → 서보=75.4° (실측 100.8°와 25.4° 불일치 → mount_offset 수정으로 해결)

### 보행 파이프라인 데이터 흐름 검증 ✅
- `getSwingTrajectory()/getStanceTrajectory()` → `foot_pos_global_` → `transformToLocal()` → `IKsolver()` → `writeJoints()` 정상 연결 확인
- IDLE 상태: `processTrot()` 미호출 → `current_hw_angles` 고정 (INIT 마지막 값 유지) — 설계상 정상

## 현재 블로커 / 미해결 이슈

### 1. RF/RH KFE mount_offset 코드 미적용
- 수정값: `mount_offset_rad[1][2] = mount_offset_rad[3][2] = 205.4° × (π/180)`
- `[WRITE_CODE]` 수신 시 즉시 적용 가능
- 추가 검증 후 적용 예정

### 2. INIT 초기 스냅 현상 (first_update bypass)
- 증상: 부팅 시 서보가 prone 위치로 즉시 스냅 후 3초 보간 (사용자: "이상한 값으로 튀었다가 돌아온다")
- 원인: `HardwareOutput::writeJoints()` 첫 호출에서 `first_update=true`가 Rate Limiter 우회
  - LF HFE: 90°→154° (스냅), RF HFE: 90°→20° (반대 방향 스냅)
  - 이후 3초간 보간으로 스탠스로 복귀
- 미적용 해결방안: prone 시작 제거 (`h_start_=default_height`) or PRONE_BODY_HEIGHT 완화

### 3. LH/RH 독립 교정 미완료
- LH KFE 스탠스 목표: 97.2° (LF 기준, 독립 미측정)
- RH KFE 스탠스 목표: 100.8° (RF 기준 가정, 독립 미측정)
- LH HFE: LF 기준 적용 중, 별도 검증 필요

## 다음 예상 작업
1. 사용자 자체 코드 분석 후 문제점 파악 (사용자 주도)
2. RF/RH KFE mount_offset 코드 수정 적용
3. LH/RH 독립 교정 측정
4. INIT 초기 스냅 해결 (prone 시작 제거 또는 완화)
5. Phase 6 HITL 본격 검증
