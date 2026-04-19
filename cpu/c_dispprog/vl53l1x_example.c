#include "vl53l1x_simple.h"

#define disp (*(volatile uint16_t*)0x20u)
#define LOOPS_PER_MS 50000u

void sleep_ms(uint32_t ms)
{
    for (uint32_t m = 0u; m < ms; ++m) {
        for (volatile uint32_t i = 0u; i < LOOPS_PER_MS; ++i) {
            __asm__ volatile ("" ::: "memory");
        }
    }
}

int main(void)
{
    uint16_t distance_mm = 0u;
    uint16_t raw_range = 0u;
    uint8_t raw_status = 0xFFu;

    if (!vl53l1x_init()) {
        while (1) {
            disp = 0xE100u;
            sleep_ms(1000u);
        }
    }

    while (1) {
        vl53l1x_poll_result_t r = vl53l1x_poll_ex(&distance_mm,
                                                  &raw_status,
                                                  &raw_range);

        (void)raw_range;

        if (r == VL53L1X_POLL_OK) {
            disp = distance_mm;
        } else if (r == VL53L1X_POLL_INVALID) {
            /* Show raw status in hex: F0ss */
            disp = (uint16_t)(0xF000u | raw_status);
        } else if (r == VL53L1X_POLL_ERROR) {
            disp = 0xFFF3u;
        }

        sleep_ms(250u);
    }
}
