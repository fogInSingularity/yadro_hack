#include <stdint.h>

#include "rover.h"
#include "rover_high.h"
#include "vl53l1x_simple.h"

#define disp   ((volatile uint16_t*)0x20)
#define button ((volatile uint16_t*)0x4)

#define LOOPS_PER_MS 7000u
static void sleep_ms(uint32_t ms)
{
    uint32_t m;
    for (m = 0u; m < ms; ++m) {
        volatile uint32_t i;
        for (i = 0u; i < LOOPS_PER_MS; ++i) {
            __asm__ volatile ("" ::: "memory");
        }
    }
}

/* ---------- States ---------- */
#define STATE_STRAIGHT 0u
#define STATE_LEFT     1u
#define STATE_RIGHT    2u
#define STATE_FINISH   3u

/* ---------- Tunable knobs ---------- */

/* Full speed straight */
#define STRAIGHT_SPEED 127

/* Curved turns: outer side fast, inner side slower */
#define TURN_OUTER_SPEED 127
#define TURN_INNER_SPEED 55

/* Start the first turn when something is close in front */
#define STRAIGHT_TRIGGER_MM 180u

/* Consider the path "open" while turning when distance rises above this */
#define TURN_OPEN_THRESHOLD_MM 320u

/* Small re-arm threshold to stop RIGHT from finishing instantly */
#define TURN_ARM_THRESHOLD_MM 220u

/* Debounce */
#define CLOSE_DEBOUNCE_SAMPLES 2u
#define OPEN_DEBOUNCE_SAMPLES  2u
#define ARM_DEBOUNCE_SAMPLES   2u

/* Keep turning a bit after the opening is detected */
#define LEFT_SETTLE_MS  120u
#define RIGHT_SETTLE_MS 120u

static void fatal(uint16_t code)
{
    while (1) {
        *disp = code;
    }
}

int main(void)
{
    while (!button) {
        sleep_ms(50);
    }

    uint16_t distance_mm = 0u;
    uint8_t state = STATE_STRAIGHT;

    uint8_t close_count = 0u;
    uint8_t open_count = 0u;
    uint8_t arm_count = 0u;
    uint8_t turn_armed = 0u;

    vl53l1x_poll_result_t r;

    rover_t rover;
    rover_high_t high;
    rover_high_config_t cfg;

    if (!vl53l1x_init()) {
        fatal(0xFFF1u);
    }

    if (!rover_init(&rover, ROVER_I2C_ADDR_DEFAULT)) {
        fatal(0xFFF4u);
    }

    if (!rover_probe(&rover)) {
        fatal(0xFFF5u);
    }

    rover_high_default_config(&cfg);

    /* Fix wheel direction if needed */
    cfg.invert_front_left  = false;
    cfg.invert_rear_left   = false;
    cfg.invert_front_right = false;
    cfg.invert_rear_right  = false;

    /* Straight */
    cfg.straight_left_speed  = STRAIGHT_SPEED;
    cfg.straight_right_speed = STRAIGHT_SPEED;

    /* Curved right: left side faster than right side */
    cfg.turn_right_left_speed  = TURN_OUTER_SPEED;
    cfg.turn_right_right_speed = TURN_INNER_SPEED;

    /* Curved left: right side faster than left side */
    cfg.turn_left_left_speed  = TURN_INNER_SPEED;
    cfg.turn_left_right_speed = TURN_OUTER_SPEED;

    if (!rover_high_init(&high, &rover, &cfg)) {
        fatal(0xFFF6u);
    }

    rover_high_go_straight(&high);

    while (1) {
        sleep_ms(15u);
        r = vl53l1x_poll(&distance_mm);
        sleep_ms(15u);

        if (r == VL53L1X_POLL_ERROR) {
            rover_high_stop(&high);
            fatal(0xFFF3u);
        }

        if (state == STATE_STRAIGHT) {
            rover_high_go_straight(&high);

            if (r == VL53L1X_POLL_OK) {
                *disp = distance_mm;

                if (distance_mm <= STRAIGHT_TRIGGER_MM) {
                    if (close_count < 255u) {
                        ++close_count;
                    }

                    if (close_count >= CLOSE_DEBOUNCE_SAMPLES) {
                        state = STATE_LEFT;
                        close_count = 0u;
                        open_count = 0u;
                        arm_count = 0u;
                        turn_armed = 1u;
                        rover_high_turn_left(&high);
                    }
                } else {
                    close_count = 0u;
                }
            } else {
                *disp = 0xFFF2u;
            }

            continue;
        }

        if (state == STATE_LEFT) {
            rover_high_turn_left(&high);

            if (r == VL53L1X_POLL_OK) {
                *disp = distance_mm;

                if (turn_armed == 0u) {
                    if (distance_mm <= TURN_ARM_THRESHOLD_MM) {
                        if (arm_count < 255u) {
                            ++arm_count;
                        }

                        if (arm_count >= ARM_DEBOUNCE_SAMPLES) {
                            turn_armed = 1u;
                            arm_count = 0u;
                            open_count = 0u;
                        }
                    } else {
                        arm_count = 0u;
                    }
                } else {
                    if (distance_mm >= TURN_OPEN_THRESHOLD_MM) {
                        if (open_count < 255u) {
                            ++open_count;
                        }

                        if (open_count >= OPEN_DEBOUNCE_SAMPLES) {
                            sleep_ms(LEFT_SETTLE_MS);
                            state = STATE_RIGHT;
                            open_count = 0u;
                            arm_count = 0u;
                            turn_armed = 0u;
                            rover_high_turn_right(&high);
                        }
                    } else {
                        open_count = 0u;
                    }
                }
            } else {
                *disp = 0x9002u;
            }

            continue;
        }

        if (state == STATE_RIGHT) {
            rover_high_turn_right(&high);

            if (r == VL53L1X_POLL_OK) {
                *disp = distance_mm;

                if (turn_armed == 0u) {
                    if (distance_mm <= TURN_ARM_THRESHOLD_MM) {
                        if (arm_count < 255u) {
                            ++arm_count;
                        }

                        if (arm_count >= ARM_DEBOUNCE_SAMPLES) {
                            turn_armed = 1u;
                            arm_count = 0u;
                            open_count = 0u;
                        }
                    } else {
                        arm_count = 0u;
                    }
                } else {
                    if (distance_mm >= TURN_OPEN_THRESHOLD_MM) {
                        if (open_count < 255u) {
                            ++open_count;
                        }

                        if (open_count >= OPEN_DEBOUNCE_SAMPLES) {
                            sleep_ms(RIGHT_SETTLE_MS);
                            state = STATE_FINISH;
                            open_count = 0u;
                            arm_count = 0u;
                            turn_armed = 0u;
                            rover_high_go_straight(&high);
                        }
                    } else {
                        open_count = 0u;
                    }
                }
            } else {
                *disp = 0x9001u;
            }

            continue;
        }

        rover_high_go_straight(&high);

        if (r == VL53L1X_POLL_OK) {
            *disp = distance_mm;
        } else {
            *disp = 0xFFF2u;
        }
    }
}