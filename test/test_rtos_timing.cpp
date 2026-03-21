#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h" // For esp_timer_get_time()

// 두 태스크의 주기를 저장할 변수
volatile int period_delay = 0;
volatile int period_delay_until = 0;

// 밀리초 단위 시간 반환 헬퍼 (Arduino의 millis() 대체)
uint32_t get_millis() {
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

void TaskDelay(void *pvParameters) {
    uint32_t lastTime = get_millis();
    while (1) {
        uint32_t currentTime = get_millis();
        period_delay = currentTime - lastTime;    // 현재 주기 계산
        lastTime = currentTime;
        
        // 눈으로 보기 쉽도록 200ms짜리 막대한 연산 부하 주입
        uint32_t startDelay = get_millis();
        while(get_millis() - startDelay < 200) {
            // CPU를 200ms 동안 100% 점유하는 루프
        }

        // 500ms 제어 주기를 원했으나, 위의 200ms가 더해져 총 700ms로 영원히 밀리게 됨
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void TaskDelayUntil(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(500);

    // 첫 시작 시간 기록
    xLastWakeTime = xTaskGetTickCount();
    uint32_t lastTime = get_millis();

    while (1) {
        uint32_t currentTime = get_millis();
        period_delay_until = currentTime - lastTime;  // 현재 주기 계산
        lastTime = currentTime;
        
        // 똑같이 200ms짜리 막대한 연산 부하 주입
        uint32_t startDelay = get_millis();
        while(get_millis() - startDelay < 200) {
            // CPU를 200ms 동안 100% 점유하는 루프
        }

        // 목표 주파수(500ms) 달성을 위해, 부하 시간(200ms)을 뺀 300ms만 스마트하게 쉼!
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskPrint(void *pvParameters) {
    while (1) {
        // 시리얼 모니터에 보기 편한 속도로 출력
        printf("vTaskDelay_Drift:%d, vTaskDelayUntil_Absolute:%d\n", period_delay, period_delay_until);
        
        // 500ms 마다 갱신 (스크롤이 너무 빠르지 않게 조절)
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

extern "C" void app_main(void) {
    // Core 1에 타이밍 감시를 위한 Task 생성
    xTaskCreatePinnedToCore(TaskDelay, "TaskDelay", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(TaskDelayUntil, "TaskDelayUntil", 2048, NULL, 1, NULL, 1);
    
    // Core 0에 인쇄 Task 생성 (타이밍 테스트에 영향을 주지 않도록 분리)
    xTaskCreatePinnedToCore(TaskPrint, "TaskPrint", 2048, NULL, 1, NULL, 0);

    // main 태스크 부하 방지를 위해 삭제
    vTaskDelete(NULL);
}
