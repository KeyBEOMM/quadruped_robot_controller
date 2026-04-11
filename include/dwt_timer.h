#pragma once

#include <stdint.h>
#include "esp_attr.h"

// ESP32 Xtensa LX6 CPU Cycle Counter Access
#if __has_include("esp_cpu.h")
    #include "esp_cpu.h"
#elif __has_include("xtensa/hal.h")
    #include <xtensa/hal.h>
#endif

namespace DWT {

    /**
     * @brief Initialize the cycle counter (if needed).
     * On ESP32 Xtensa, the CCOUNT register is always running.
     * We provide this for interface compatibility.
     */
    static inline IRAM_ATTR void init() {
        // Nothing to specifically initialize for ESP32 CCOUNT,
        // it starts at boot.
    }

    /**
     * @brief Get current CPU cycle count directly.
     * @return 32-bit cycle count (overflows every ~17.9 seconds at 240MHz)
     */
    static inline IRAM_ATTR uint32_t now_cycles() {
#if __has_include("esp_cpu.h")
        return esp_cpu_get_cycle_count();
#elif __has_include("xtensa/hal.h")
        return xthal_get_ccount();
#else
        uint32_t ccount;
        __asm__ __volatile__("rsr %0, ccount" : "=a"(ccount));
        return ccount;
#endif
    }

    /**
     * @brief Convert cycles to microseconds (assuming 240 MHz clock)
     */
    static inline IRAM_ATTR float cycles_to_us(uint32_t cycles) {
        return (float)cycles / 240.0f;
    }

    /**
     * @brief Convert cycles to milliseconds (assuming 240 MHz clock)
     */
    static inline IRAM_ATTR float cycles_to_ms(uint32_t cycles) {
        return (float)cycles / 240000.0f;
    }

} // namespace DWT
