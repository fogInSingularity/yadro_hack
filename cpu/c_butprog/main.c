#include <stdint.h>

volatile uint16_t* disp = (uint16_t*)0x20;
volatile const uint16_t* but  = (uint16_t*)0x4;

void main(void)
{
    // uint32_t i = 1u << 31;
    while (1) {
        *disp = *but;
    }
}
