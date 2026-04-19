#include "vl53l1x_simple.h"

volatile uint16_t* disp = (uint16_t*)0x20;

void sleep_ms(uint32_t ms)
{
    const volatile uint32_t loops_per_ms = 50000; // calibrate experimentally

    for (uint32_t m = 0; m < ms; ++m) {
        for (volatile uint32_t i = 0; i < loops_per_ms; ++i) {
            __asm__ volatile ("" ::: "memory");
        }
    }
}

int main(void)
{
    int a = 0;
    uint16_t distance_mm;

    if (!vl53l1x_init()) {
        while (1) {
            sleep_ms(2500);
            *disp = 0x3124;
        }
    }

    while (1) {
        vl53l1x_poll_result_t r = vl53l1x_poll(&distance_mm);

        if (r == VL53L1X_POLL_OK) {
            *disp = distance_mm;
        } else if (r == VL53L1X_POLL_INVALID) {
            *disp = 0xFFF2;
        } else if (r == VL53L1X_POLL_ERROR) {
            *disp = 0xFFF3;
        }

        sleep_ms(250);
    }
}
