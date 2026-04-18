#include "vl53l1x_simple.h"

volatile uint16_t* disp = (uint16_t*)0x20;

int main(void)
{
    int a = 0;
    uint16_t distance_mm;

    if (!vl53l1x_init()) {
        while (1) {
            a = a + 1;
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
    }
}
