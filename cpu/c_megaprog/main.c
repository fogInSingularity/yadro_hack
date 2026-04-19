#include <stdint.h>

#include "rover.h"
#include "rover_high.h"

#include "vl53l1x_simple.h"

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

/* Opening detected when distance >= this */
#define OPEN_THRESHOLD_MM 300

/* Must fall below this before we arm the next opening detect */
#define REARM_THRESHOLD_MM 220

/* Require N consecutive samples before triggering */
#define OPEN_DEBOUNCE_SAMPLES 3
#define REARM_DEBOUNCE_SAMPLES 3

/*
 * Turn duration in main-loop iterations.
 * Tune these on hardware until the rover makes the angle you want.
 */
#define RIGHT_TURN_TICKS 18000
#define LEFT_TURN_TICKS  18000

/* ---------- State machine ---------- */

typedef enum {
    STATE_FIND_FIRST_OPEN = 0,
    STATE_TURN_RIGHT,
    STATE_REARM_SECOND_OPEN,
    STATE_FIND_SECOND_OPEN,
    STATE_TURN_LEFT,
    STATE_FINAL_STRAIGHT
} robot_state_t;

static void fatal(uint16_t code)
{
    while (1) {
        *disp = code;
    }
}

int main(void)
{
    uint16_t distance_mm = 0;
    uint8_t hit_count = 0;
    uint8_t rearm_count = 0;
    uint32_t turn_ticks = 0;
    robot_state_t state = STATE_FIND_FIRST_OPEN;

    rover_t rover;
    rover_high_t high;
    rover_high_config_t cfg;

    if (!vl53l1x_init()) {
        fatal(0xFFF1);
    }

    if (!rover_init(&rover, ROVER_I2C_ADDR_DEFAULT)) {
        fatal(0xFFF4);
    }

    if (!rover_probe(&rover)) {
        fatal(0xFFF5);
    }

    rover_high_default_config(&cfg);

    /*
     * Tune these for your robot.
     * These are examples only.
     */

    /* Fix wheel direction if needed */
    cfg.invert_front_left  = false;
    cfg.invert_rear_left   = false;
    cfg.invert_front_right = true;
    cfg.invert_rear_right  = true;

    /* Straight */
    cfg.straight_left_speed  = 70;
    cfg.straight_right_speed = 68;

    /* Right turn */
    cfg.turn_right_left_speed  = 75;
    cfg.turn_right_right_speed = -75;

    /* Left turn */
    cfg.turn_left_left_speed  = -75;
    cfg.turn_left_right_speed = 75;

    if (!rover_high_init(&high, &rover, &cfg)) {
        fatal(0xFFF6);
    }

    while (1) {
        vl53l1x_poll_result_t r = vl53l1x_poll(&distance_mm);
        volatile robot_state_t current_state;

        sleep_ms(15);

        if (r == VL53L1X_POLL_ERROR) {
            rover_high_stop(&high);
            fatal(0xFFF3);
        }

        current_state = state;

        if (current_state == STATE_FIND_FIRST_OPEN) {
            rover_high_go_straight(&high);

            if (r == VL53L1X_POLL_OK) {
                *disp = distance_mm;

                if (distance_mm >= OPEN_THRESHOLD_MM) {
                    if (hit_count < 255) {
                        ++hit_count;
                    }
                    if (hit_count >= OPEN_DEBOUNCE_SAMPLES) {
                        state = STATE_TURN_RIGHT;
                        turn_ticks = RIGHT_TURN_TICKS;
                        hit_count = 0;
                    }
                } else {
                    hit_count = 0;
                }
            } else {
                *disp = 0xFFF2;
            }
            continue;
        }

        if (current_state == STATE_TURN_RIGHT) {
            rover_high_turn_right(&high);
            *disp = 0x9001;

            if (turn_ticks > 0) {
                --turn_ticks;
                asm volatile("" : : "g"(turn_ticks) : "memory");
            } else {
                state = STATE_REARM_SECOND_OPEN;
                rearm_count = 0;
            }
            continue;
        }

        if (current_state == STATE_REARM_SECOND_OPEN) {
            rover_high_go_straight(&high);

            if (r == VL53L1X_POLL_OK) {
                *disp = distance_mm;

                if (distance_mm <= REARM_THRESHOLD_MM) {
                    if (rearm_count < 255) {
                        ++rearm_count;
                    }
                    if (rearm_count >= REARM_DEBOUNCE_SAMPLES) {
                        state = STATE_FIND_SECOND_OPEN;
                        rearm_count = 0;
                    }
                } else {
                    rearm_count = 0;
                }
            } else {
                *disp = 0xFFF2;
            }
            continue;
        }

        if (current_state == STATE_FIND_SECOND_OPEN) {
            rover_high_go_straight(&high);

            if (r == VL53L1X_POLL_OK) {
                *disp = distance_mm;

                if (distance_mm >= OPEN_THRESHOLD_MM) {
                    if (hit_count < 255) {
                        ++hit_count;
                    }
                    if (hit_count >= OPEN_DEBOUNCE_SAMPLES) {
                        state = STATE_TURN_LEFT;
                        turn_ticks = LEFT_TURN_TICKS;
                        hit_count = 0;
                    }
                } else {
                    hit_count = 0;
                }
            } else {
                *disp = 0xFFF2;
            }
            continue;
        }

        if (current_state == STATE_TURN_LEFT) {
            rover_high_turn_left(&high);
            *disp = 0x9002;

            if (turn_ticks > 0) {
                --turn_ticks;
            } else {
                state = STATE_FINAL_STRAIGHT;
            }
            continue;
        }

        rover_high_go_straight(&high);

        if (r == VL53L1X_POLL_OK) {
            *disp = distance_mm;
        } else {
            *disp = 0xFFF2;
        }
    }
}