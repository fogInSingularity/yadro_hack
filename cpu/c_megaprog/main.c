#include <stdint.h>

#include "rover.h"
#include "rover_high.h"
#include "i2c.h"
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
#define STRAIGHT_SPEED 127
#define STRAFE_LEFT_SPEED 110

#define STRAIGHT_TRIGGER_MM    250u
#define LEFT_OPEN_THRESHOLD_MM 400u

#define CLOSE_DEBOUNCE_SAMPLES 2u
#define OPEN_DEBOUNCE_SAMPLES  2u

/* First transition pause: STRAIGHT -> LEFT */
#define DEBUG_TRANSITION_MS 400u

/* Second transition pause: LEFT -> STRAIGHT */
#define RETURN_TRANSITION_MS 200u

/* Recovery timing for ToF4M sensor re-init */
#define TOF4M_RECOVERY_DELAY_MS 10u
#define TOF4M_RECOVERY_FAIL_CODE 0xFFF1u
#define TOF4M_RECOVERY_ACTIVE_CODE 0xFFF3u

static void sleep_transition_first(void)
{
#if DEBUG_TRANSITION_MS > 0u
    sleep_ms(DEBUG_TRANSITION_MS);
#endif
}

static void sleep_transition_second(void)
{
#if RETURN_TRANSITION_MS > 0u
    sleep_ms(RETURN_TRANSITION_MS);
#endif
}

static void fatal(uint16_t code)
{
    while (1) {
        *disp = code;
    }
}

static bool recover_tof4m(void)
{
    /*
     * Sensor-only recovery path.
     * Motors are intentionally not touched here.
     */
    *disp = TOF4M_RECOVERY_ACTIVE_CODE;

    (void)vl53l1x_stop();
    sleep_ms(TOF4M_RECOVERY_DELAY_MS);

    if (!vl53l1x_init()) {
        *disp = TOF4M_RECOVERY_FAIL_CODE;
        return false;
    }

    return true;
}

int main(void)
{
    {
        uint8_t stop_motors[4];
        stop_motors[0] = 0u;
        stop_motors[1] = 0u;
        stop_motors[2] = 0u;
        stop_motors[3] = 0u;

        (void)i2c_write_reg8_burst(ROVER_I2C_ADDR_DEFAULT,
                                   0x00u,
                                   stop_motors,
                                   sizeof(stop_motors));
    }

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

    cfg.invert_front_left  = false;
    cfg.invert_rear_left   = false;
    cfg.invert_front_right = false;
    cfg.invert_rear_right  = false;

    cfg.straight_left_speed  = STRAIGHT_SPEED;
    cfg.straight_right_speed = STRAIGHT_SPEED;
    cfg.strafe_left_speed    = STRAFE_LEFT_SPEED;

    if (!rover_high_init(&high, &rover, &cfg)) {
        fatal(0xFFF6u);
    }

    rover_high_go_straight(&high);

    bool sensor_needed = true;
    while (1) {
        sleep_ms(15u);
        r = vl53l1x_poll(&distance_mm);
        sleep_ms(15u);

        if (r == VL53L1X_POLL_ERROR && sensor_needed) {
            /*
             * Re-initialize only the ToF4M sensor and preserve the current
             * state machine state and debounce counters.
             */
            (void)recover_tof4m();
            continue;
        }

        if (state == STATE_STRAIGHT_1) {
            rover_high_go_straight(&high);

            if (r == VL53L1X_POLL_OK) {
                *disp = distance_mm;

                if (distance_mm <= STRAIGHT_TRIGGER_MM) {
                    if (close_count < 255u) {
                        ++close_count;
                    }

                    if (close_count >= CLOSE_DEBOUNCE_SAMPLES) {
                        close_count = 0u;
                        open_count = 0u;

                        sensor_needed = false;
                        rover_high_stop(&high);
                        sleep_transition_first();

                        state = STATE_LEFT;
                        rover_high_go_left(&high);
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
            rover_high_go_left(&high);

            if (r == VL53L1X_POLL_OK) {
                *disp = distance_mm;

                if (distance_mm >= LEFT_OPEN_THRESHOLD_MM) {
                    if (open_count < 255u) {
                        ++open_count;
                    }

                    if (open_count >= OPEN_DEBOUNCE_SAMPLES) {
                        open_count = 0u;

                        rover_high_stop(&high);
                        sleep_transition_second();

                        state = STATE_STRAIGHT_2;
                        rover_high_go_straight(&high);
                    }
                } else {
                    open_count = 0u;
                }
            } else {
                *disp = 0x9002u;
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