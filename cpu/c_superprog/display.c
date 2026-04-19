#include "stdint.h"

volatile uint16_t* disp = (uint16_t*)0x20;

void sleep_ms(uint32_t ms)
{
    const volatile uint32_t loops_per_ms = 50000 / 7; // calibrate experimentally

    for (uint32_t m = 0; m < ms; ++m) {
        for (volatile uint32_t i = 0; i < loops_per_ms; ++i) {
            __asm__ volatile ("" ::: "memory");
        }
    }
}

int main(void)
{
    uint16_t i = 0x0000;

    while (1) {
        *disp = i++;
        sleep_ms(1000);
    }
}

