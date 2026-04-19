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
#define STATE_STRAIGHT_1 0u
#define STATE_LEFT       1u
#define STATE_STRAIGHT_2 2u

/* ---------- Tunable knobs ---------- */

/* Forward speed */
#define STRAIGHT_SPEED 127

/* Start sliding left when wall is close */
#define STRAIGHT_TRIGGER_MM 50u

/* While sliding left, treat this as "no more wall" */
#define LEFT_OPEN_THRESHOLD_MM 100u

/* Debounce */
#define CLOSE_DEBOUNCE_SAMPLES 2u
#define OPEN_DEBOUNCE_SAMPLES  2u

/* Debug pause at every state transition.
   Set to 0u when you're done debugging. */
#define DEBUG_TRANSITION_MS 3000u

static void fatal(uint16_t code)
{
    while (1) {
        *disp = code;
    }
}

static void transition_pause(rover_high_t *high)
{
    rover_high_stop(high);
#if DEBUG_TRANSITION_MS > 0u
    sleep_ms(DEBUG_TRANSITION_MS);
#endif
}

static inline void cmd_straight(rover_high_t *high)
{
    rover_high_go_straight(high);
}

static inline void cmd_left_sideways(rover_high_t *high)
{
    /* Replace this line if your sideways helper has a different name.
       This must be a PURE LEFT STRAFE, not a rotation. */
    rover_high_go_left(high);
}

int main(void)
{
    /* Fix: must dereference button */
    while (!*button) {
        sleep_ms(50u);
    }

    uint16_t distance_mm = 0u;
    uint8_t state = STATE_STRAIGHT_1;

    uint8_t close_count = 0u;
    uint8_t open_count = 0u;

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

    /* Straight speed */
    cfg.straight_left_speed  = STRAIGHT_SPEED;
    cfg.straight_right_speed = STRAIGHT_SPEED;

    /* If your rover_high config has dedicated sideways/strafe speed fields,
       set them here too. */

    if (!rover_high_init(&high, &rover, &cfg)) {
        fatal(0xFFF6u);
    }

    cmd_straight(&high);

    while (1) {
        sleep_ms(15u);
        r = vl53l1x_poll(&distance_mm);
        sleep_ms(15u);

        if (r == VL53L1X_POLL_ERROR) {
            rover_high_stop(&high);
            fatal(0xFFF3u);
        }

        if (state == STATE_STRAIGHT_1) {
            cmd_straight(&high);

            if (r == VL53L1X_POLL_OK) {
                *disp = distance_mm;

                if (distance_mm <= STRAIGHT_TRIGGER_MM) {
                    if (close_count < 255u) {
                        ++close_count;
                    }

                    if (close_count >= CLOSE_DEBOUNCE_SAMPLES) {
                        close_count = 0u;
                        open_count = 0u;

                        transition_pause(&high);
                        state = STATE_LEFT;
                        cmd_left_sideways(&high);
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
            cmd_left_sideways(&high);

            if (r == VL53L1X_POLL_OK) {
                *disp = distance_mm;

                if (distance_mm >= LEFT_OPEN_THRESHOLD_MM) {
                    if (open_count < 255u) {
                        ++open_count;
                    }

                    if (open_count >= OPEN_DEBOUNCE_SAMPLES) {
                        open_count = 0u;

                        transition_pause(&high);
                        state = STATE_STRAIGHT_2;
                        cmd_straight(&high);
                    }
                } else {
                    open_count = 0u;
                }
            } else {
                *disp = 0x9002u;
            }

            continue;
        }

        /* Final state: keep going straight */
        cmd_straight(&high);

        if (r == VL53L1X_POLL_OK) {
            *disp = distance_mm;
        } else {
            *disp = 0xFFF2u;
        }
    }
}