#include "vl53l1x_simple.h"

volatile uint16_t* disp = (uint16_t*)0x20;

extern volatile uint8_t  vl53l1x_last_raw_status;
extern volatile uint16_t vl53l1x_last_raw_range;
extern volatile uint8_t  vl53l1x_last_result_block[17];

void sleep_ms(uint32_t ms)
{
    const volatile uint32_t loops_per_ms = 50000;

    for (uint32_t m = 0; m < ms; ++m) {
        for (volatile uint32_t i = 0; i < loops_per_ms; ++i) {
            __asm__ volatile ("" ::: "memory");
        }
    }
}

static uint16_t pack_block_pair(uint8_t pair_index)
{
    uint8_t i0 = (uint8_t)(pair_index * 2u);
    uint8_t i1 = (uint8_t)(i0 + 1u);

    uint8_t hi = 0u;
    uint8_t lo = 0u;

    if (i0 < 17u) {
        hi = vl53l1x_last_result_block[i0];
    }

    if (i1 < 17u) {
        lo = vl53l1x_last_result_block[i1];
    }

    return (uint16_t)(((uint16_t)hi << 8) | lo);
}

int main(void)
{
    uint16_t distance_mm;
    uint8_t pair_index = 0u;

    if (!vl53l1x_init()) {
        while (1) {
            *disp = 0xE100;
            sleep_ms(1000);
        }
    }

    while (1) {
        /* keep polling until we get a fresh block or an error */
        for (uint32_t k = 0; k < 20u; ++k) {
            vl53l1x_poll_result_t r = vl53l1x_poll(&distance_mm);

            if (r == VL53L1X_POLL_ERROR) {
                *disp = 0xFFF3;
                sleep_ms(1000);
                break;
            }

            sleep_ms(50);
        }

        *disp = pack_block_pair(pair_index);

        pair_index++;
        if (pair_index >= 9u) {
            pair_index = 0u;
        }

        sleep_ms(1000);
    }
}