# 4. RTOS Scheduling & Timing Management

개루프(Open-loop) 제어로 구동되는 현재 버전의 하드웨어 특성 상, 연산 주기(`dt`)의 미세한 오차는 곧 모터 궤적 위치의 치명적 적분 오차로 직결됩니다. 이를 방어하기 위한 `ControlTask`의 타이밍 제약입니다.

---

## 4.1. Core Timing Principles (절대 타이밍 보장)

1.  **목표 제어 주기 (100Hz / 10ms)**
    *   **근거:** Phase 4 검증 결과 `pipeline_us < 10,000μs` 확인 완료. 현재 펌웨어는 50Hz(20ms)로 운용 중이며, Phase 8에서 `pdMS_TO_TICKS(10)`으로 전환 예정. 실측값: `context/knowledge/05_hardware_params.md §5.3` 참조.

2.  **타이밍 보장 API 사용 (`vTaskDelayUntil`)**
    *   **규칙:** 단순 상대 시간 지연인 `vTaskDelay()`를 절대 사용해서는 안 됩니다. 지정된 절대 시각부터 정확히 대기하는 `vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10))`을 사용하여, 루틴과 루틴 사이의 누적 지연(Drift) 오차를 완전히 차단합니다.

## 4.2. Deadline Miss (오버런) 대응 방안

어떠한 이유로 실연산 시간이 10ms 예산을 초과했을 경우 무너진 프레임 간격을 동기화하기 위한 명세입니다.

*   **설계 결정 정책:** `Skip & Catch-up` (글로벌 시간 우선 타임라인 보존)
*   **배경:** 프레임을 놓쳤을 때 해당 위치부터 늦게 다시 시작(Delay-shift)하면 4개 다리의 위상(Phase)이 완전히 어긋납니다.
*   **해결책:** 데드라인을 놓친 틱(Tick)을 미련 없이 버리고, 실제 경과 시간(`dt = 현재시각 - 이전시각`)만큼 보행 궤적 진행률 변수(`s`)를 한 번에 크게 점프시킵니다.
*   **충격 완화:** 점프 폭이 너무 커서 모터가 따라가기 이전에 기구적 파손이 생길 수 있으므로, [Hardware Clamping 제한자](03_locomotion_kinematics.md)의 "Angular Velocity Rate Limiting" 룰이 최종 PWM 방출 직전에 동작하여 모터의 가파른 기동을 억제합니다.

## 4.3. PWM / 서보 파라미터

> 확정 수치 (클램핑 범위, Rate Limit, Deadband, PWM 변환식, PCA9685 오실레이터 주파수 등):
> `context/knowledge/05_hardware_params.md §5.2` 참조.
