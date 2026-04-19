#include <stdint.h>

#include "rover.h"

#define disp ((volatile uint16_t*)0x20)

#define LOOPS_PER_MS 50000u
void sleep_ms(uint32_t ms)
{
    for (uint32_t m = 0u; m < ms; ++m) {
        for (volatile uint32_t i = 0u; i < LOOPS_PER_MS; ++i) {
            __asm__ volatile ("" ::: "memory");
        }
    }
}

/* ---------- Tunable knobs ---------- */

#define TEST_SPEED     70
#define SPIN_TIME_MS   1200u
#define PAUSE_TIME_MS   500u

static void fatal_stop(const rover_t* rover, uint16_t code)
{
    if (rover != 0) {
        (void)rover_stop(rover);
    }

    while (1) {
        *disp = code;
    }
}

static void spin_one_motor(const rover_t* rover,
                           rover_motor_t motor,
                           uint16_t index)
{
    if (!rover_stop(rover)) {
        fatal_stop(rover, 0xFFF6);
    }

    *disp = index;
    sleep_ms(PAUSE_TIME_MS);

    if (!rover_set_motor(rover, motor, TEST_SPEED)) {
        fatal_stop(rover, 0xFFF7);
    }

    *disp = index;
    sleep_ms(SPIN_TIME_MS);

    if (!rover_stop(rover)) {
        fatal_stop(rover, 0xFFF8);
    }

    *disp = 0u;
    sleep_ms(PAUSE_TIME_MS);
}

int main(void)
{
    rover_t rover;

    if (!rover_init(&rover, ROVER_I2C_ADDR_DEFAULT)) {
        fatal_stop(0, 0xFFF4);
    }

    if (!rover_probe(&rover)) {
        fatal_stop(&rover, 0xFFF5);
    }

    if (!rover_stop(&rover)) {
        fatal_stop(&rover, 0xFFF6);
    }

    while (1) {
        spin_one_motor(&rover, ROVER_MOTOR_1, 1u);
        spin_one_motor(&rover, ROVER_MOTOR_2, 2u);
        spin_one_motor(&rover, ROVER_MOTOR_3, 3u);
        spin_one_motor(&rover, ROVER_MOTOR_4, 4u);
    }
}