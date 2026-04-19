#include <stdint.h>
#include <stdbool.h>

#include "rover.h"
#include "i2c.h"
#include "vl53l1x_simple.h"

#define disp   ((volatile uint16_t*)0x20)
#define button ((volatile uint16_t*)0x4)

#define TOF4M_KEEP_SYMBOL __attribute__((used, noinline, externally_visible))

static uint32_t udiv32(uint32_t n, uint32_t d)
{
    uint32_t q = 0u;
    uint32_t r = 0u;
    uint32_t i;

    if (d == 0u) {
        return 0xFFFFFFFFu;
    }

    i = 32u;
    while (i != 0u) {
        --i;

        r = (r << 1) | ((n >> i) & 1u);

        if (r >= d) {
            r = r - d;
            q |= (1u << i);
        }
    }

    return q;
}

TOF4M_KEEP_SYMBOL int __divsi3(int a, int b)
{
    uint32_t ua;
    uint32_t ub;
    uint32_t uq;
    int negate = 0;

    if (b == 0) {
        return 0;
    }

    if (a < 0) {
        ua = (uint32_t)(-(a + 1)) + 1u;
        negate = !negate;
    } else {
        ua = (uint32_t)a;
    }

    if (b < 0) {
        ub = (uint32_t)(-(b + 1)) + 1u;
        negate = !negate;
    } else {
        ub = (uint32_t)b;
    }

    uq = udiv32(ua, ub);

    if (negate) {
        return (int)(0u - uq);
    }

    return (int)uq;
}


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
#define STATE_STRAIGHT_1      0u
#define STATE_DIAG_LEFT_1     1u
#define STATE_LEFT            2u
#define STATE_DIAG_LEFT_2     3u
#define STATE_STRAIGHT_2      4u

/* ---------- Timing knobs ---------- */
#define CONTROL_PERIOD_MS            15u
#define MOTION_RAMP_MS              180u
#define EXIT_DIAGONAL_HOLD_MS       250u

/* Set to 0u in normal use. */
#define DEBUG_TRANSITION_MS           0u

/* ---------- Distance knobs ---------- */
#define DIAG_TRIGGER_MM             350u
#define LEFT_TRIGGER_MM             150u
#define OPEN_THRESHOLD_MM           400u

/* ---------- Debounce knobs ---------- */
#define DIAG_DEBOUNCE_SAMPLES         2u
#define LEFT_DEBOUNCE_SAMPLES         2u
#define OPEN_DEBOUNCE_SAMPLES         2u

/* ---------- Speed knobs ---------- */
#define STRAIGHT_SPEED              127
#define LEFT_SPEED                   90

/*
 * Diagonal forward-left command.
 *
 * Assumed wheel order:
 *   motor1 = front-left
 *   motor2 = front-right
 *   motor3 = rear-left
 *   motor4 = rear-right
 *
 * For a mecanum-style forward-left diagonal, a common starting point is:
 *   FL = 0
 *   FR = +
 *   RL = +
 *   RR = 0
 *
 * Leave these as knobs for tuning.
 */
#define DIAG_LEFT_FRONT_LEFT_SPEED     0
#define DIAG_LEFT_FRONT_RIGHT_SPEED  110
#define DIAG_LEFT_REAR_LEFT_SPEED   110
#define DIAG_LEFT_REAR_RIGHT_SPEED     0

/* ---------- Motor inversion knobs ---------- */
#define INVERT_FRONT_LEFT           false
#define INVERT_FRONT_RIGHT          false
#define INVERT_REAR_LEFT            false
#define INVERT_REAR_RIGHT           false

/* ---------- Error / debug codes ---------- */
#define TOF4M_INIT_FAIL_CODE         0xFFF1u
#define SENSOR_POLL_FAIL_CODE        0xFFF2u
#define TOF4M_RECOVERY_ACTIVE_CODE   0xFFF3u
#define ROVER_INIT_FAIL_CODE         0xFFF4u
#define ROVER_PROBE_FAIL_CODE        0xFFF5u
#define MOTOR_WRITE_FAIL_CODE        0xFFF6u
#define TOF4M_RECOVERY_FAIL_CODE     0xFFF7u

/* Recovery timing for ToF4M sensor re-init */
#define TOF4M_RECOVERY_DELAY_MS        10u

typedef struct {
    int16_t m1;
    int16_t m2;
    int16_t m3;
    int16_t m4;
} motor_cmd_t;

typedef struct {
    motor_cmd_t current;
    motor_cmd_t start;
    motor_cmd_t target;
    uint32_t start_ms;
    uint32_t duration_ms;
    bool active;
} motion_ramp_t;

static void sleep_transition(uint32_t* now_ms)
{
#if DEBUG_TRANSITION_MS > 0u
    sleep_ms(DEBUG_TRANSITION_MS);
    if (now_ms != NULL) {
        *now_ms += DEBUG_TRANSITION_MS;
    }
#else
    (void)now_ms;
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
    *disp = TOF4M_RECOVERY_ACTIVE_CODE;

    (void)vl53l1x_stop();
    sleep_ms(TOF4M_RECOVERY_DELAY_MS);

    if (!vl53l1x_init()) {
        *disp = TOF4M_RECOVERY_FAIL_CODE;
        return false;
    }

    return true;
}

static int16_t apply_invert(int16_t speed, bool invert)
{
    if (invert) {
        return (int16_t)(-speed);
    }

    return speed;
}

static motor_cmd_t make_motor_cmd(int16_t fl,
                                  int16_t fr,
                                  int16_t rl,
                                  int16_t rr)
{
    motor_cmd_t cmd;

    cmd.m1 = apply_invert(fl, INVERT_FRONT_LEFT);
    cmd.m2 = apply_invert(fr, INVERT_FRONT_RIGHT);
    cmd.m3 = apply_invert(rl, INVERT_REAR_LEFT);
    cmd.m4 = apply_invert(rr, INVERT_REAR_RIGHT);

    return cmd;
}

static motor_cmd_t motion_stop(void)
{
    return make_motor_cmd(0, 0, 0, 0);
}

static motor_cmd_t motion_straight(void)
{
    return make_motor_cmd(STRAIGHT_SPEED,
                          STRAIGHT_SPEED,
                          STRAIGHT_SPEED,
                          STRAIGHT_SPEED);
}

static motor_cmd_t motion_left(void)
{
    /*
     * Strafe left.
     * Common mecanum pattern:
     *   FL = -S
     *   FR = +S
     *   RL = +S
     *   RR = -S
     */
    return make_motor_cmd(-LEFT_SPEED,
                           LEFT_SPEED,
                           LEFT_SPEED,
                          -LEFT_SPEED);
}

static motor_cmd_t motion_diag_left(void)
{
    return make_motor_cmd(DIAG_LEFT_FRONT_LEFT_SPEED,
                          DIAG_LEFT_FRONT_RIGHT_SPEED,
                          DIAG_LEFT_REAR_LEFT_SPEED,
                          DIAG_LEFT_REAR_RIGHT_SPEED);
}

static motor_cmd_t motion_for_state(uint8_t state)
{
    switch (state) {
    case STATE_STRAIGHT_1:
    case STATE_STRAIGHT_2:
        return motion_straight();

    case STATE_DIAG_LEFT_1:
    case STATE_DIAG_LEFT_2:
        return motion_diag_left();

    case STATE_LEFT:
        return motion_left();

    default:
        return motion_stop();
    }
}

static bool motor_cmd_equal(const motor_cmd_t* a, const motor_cmd_t* b)
{
    if (a == NULL || b == NULL) {
        return false;
    }

    return (a->m1 == b->m1) &&
           (a->m2 == b->m2) &&
           (a->m3 == b->m3) &&
           (a->m4 == b->m4);
}

static int16_t lerp_i16(int16_t a,
                        int16_t b,
                        uint32_t elapsed_ms,
                        uint32_t duration_ms)
{
    int32_t delta;
    int32_t value;

    if (duration_ms == 0u || elapsed_ms >= duration_ms) {
        return (int16_t)rover_clamp_speed((int16_t)b);
    }

    delta = (int32_t)b - (int32_t)a;
    value = (int32_t)a + (delta * (int32_t)elapsed_ms) / (int32_t)duration_ms;

    return (int16_t)rover_clamp_speed((int16_t)value);
}

static motor_cmd_t motor_cmd_lerp(const motor_cmd_t* from,
                                  const motor_cmd_t* to,
                                  uint32_t elapsed_ms,
                                  uint32_t duration_ms)
{
    motor_cmd_t out;

    out.m1 = lerp_i16(from->m1, to->m1, elapsed_ms, duration_ms);
    out.m2 = lerp_i16(from->m2, to->m2, elapsed_ms, duration_ms);
    out.m3 = lerp_i16(from->m3, to->m3, elapsed_ms, duration_ms);
    out.m4 = lerp_i16(from->m4, to->m4, elapsed_ms, duration_ms);

    return out;
}

static void motion_ramp_init(motion_ramp_t* ramp, motor_cmd_t initial)
{
    if (ramp == NULL) {
        return;
    }

    ramp->current = initial;
    ramp->start = initial;
    ramp->target = initial;
    ramp->start_ms = 0u;
    ramp->duration_ms = 0u;
    ramp->active = false;
}

static void motion_ramp_update(motion_ramp_t* ramp, uint32_t now_ms)
{
    uint32_t elapsed_ms;

    if (ramp == NULL) {
        return;
    }

    if (!ramp->active) {
        ramp->current = ramp->target;
        return;
    }

    elapsed_ms = now_ms - ramp->start_ms;

    if (elapsed_ms >= ramp->duration_ms) {
        ramp->current = ramp->target;
        ramp->active = false;
        return;
    }

    ramp->current = motor_cmd_lerp(&ramp->start,
                                   &ramp->target,
                                   elapsed_ms,
                                   ramp->duration_ms);
}

static void motion_ramp_set_target(motion_ramp_t* ramp,
                                   const motor_cmd_t* target,
                                   uint32_t now_ms,
                                   uint32_t duration_ms)
{
    if (ramp == NULL || target == NULL) {
        return;
    }

    if (motor_cmd_equal(&ramp->target, target)) {
        return;
    }

    ramp->start = ramp->current;
    ramp->target = *target;
    ramp->start_ms = now_ms;
    ramp->duration_ms = duration_ms;

    if (duration_ms == 0u || motor_cmd_equal(&ramp->start, &ramp->target)) {
        ramp->current = ramp->target;
        ramp->active = false;
        return;
    }

    ramp->active = true;
}

static bool apply_motor_cmd(const rover_t* rover, const motor_cmd_t* cmd)
{
    if (rover == NULL || cmd == NULL) {
        return false;
    }

    return rover_set_motors(rover,
                            cmd->m1,
                            cmd->m2,
                            cmd->m3,
                            cmd->m4);
}

int main(void)
{
    uint16_t distance_mm = 0u;
    uint8_t state = STATE_STRAIGHT_1;
    uint8_t diag_count = 0u;
    uint8_t left_count = 0u;
    uint8_t open_count = 0u;
    uint32_t now_ms = 0u;
    uint32_t state_entry_ms = 0u;

    vl53l1x_poll_result_t r;
    rover_t rover;
    motion_ramp_t ramp;
    motor_cmd_t desired;

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

    if (!vl53l1x_init()) {
        fatal(TOF4M_INIT_FAIL_CODE);
    }

    if (!rover_init(&rover, ROVER_I2C_ADDR_DEFAULT)) {
        fatal(ROVER_INIT_FAIL_CODE);
    }

    if (!rover_probe(&rover)) {
        fatal(ROVER_PROBE_FAIL_CODE);
    }

    motion_ramp_init(&ramp, motion_stop());

    /* Smooth startup into straight motion */
    desired = motion_for_state(state);
    motion_ramp_set_target(&ramp, &desired, now_ms, MOTION_RAMP_MS);
    motion_ramp_update(&ramp, now_ms);

    if (!apply_motor_cmd(&rover, &ramp.current)) {
        fatal(MOTOR_WRITE_FAIL_CODE);
    }

    while (1) {
        sleep_ms(CONTROL_PERIOD_MS);
        now_ms += CONTROL_PERIOD_MS;

        r = vl53l1x_poll(&distance_mm);

        if (r == VL53L1X_POLL_ERROR) {
            (void)recover_tof4m();
        }

        /* Bring ramp to "now" before possibly changing target */
        motion_ramp_update(&ramp, now_ms);

        switch (state) {
        case STATE_STRAIGHT_1:
            if (r == VL53L1X_POLL_OK) {
                if (distance_mm <= DIAG_TRIGGER_MM) {
                    if (diag_count < 255u) {
                        ++diag_count;
                    }

                    if (diag_count >= DIAG_DEBOUNCE_SAMPLES) {
                        diag_count = 0u;
                        left_count = 0u;
                        open_count = 0u;

                        sleep_transition(&now_ms);
                        state = STATE_DIAG_LEFT_1;
                        state_entry_ms = now_ms;
                    }
                } else {
                    diag_count = 0u;
                }
            } else {
                diag_count = 0u;
            }
            break;

        case STATE_DIAG_LEFT_1:
            if (r == VL53L1X_POLL_OK) {
                if (distance_mm <= LEFT_TRIGGER_MM) {
                    if (left_count < 255u) {
                        ++left_count;
                    }

                    if (left_count >= LEFT_DEBOUNCE_SAMPLES) {
                        diag_count = 0u;
                        left_count = 0u;
                        open_count = 0u;

                        sleep_transition(&now_ms);
                        state = STATE_LEFT;
                        state_entry_ms = now_ms;
                    }
                } else {
                    left_count = 0u;
                }
            } else {
                left_count = 0u;
            }
            break;

        case STATE_LEFT:
            if (r == VL53L1X_POLL_OK) {
                if (distance_mm >= OPEN_THRESHOLD_MM) {
                    if (open_count < 255u) {
                        ++open_count;
                    }

                    if (open_count >= OPEN_DEBOUNCE_SAMPLES) {
                        diag_count = 0u;
                        left_count = 0u;
                        open_count = 0u;

                        sleep_transition(&now_ms);
                        state = STATE_DIAG_LEFT_2;
                        state_entry_ms = now_ms;
                    }
                } else {
                    open_count = 0u;
                }
            } else {
                open_count = 0u;
            }
            break;

        case STATE_DIAG_LEFT_2:
            if ((now_ms - state_entry_ms) >= EXIT_DIAGONAL_HOLD_MS) {
                diag_count = 0u;
                left_count = 0u;
                open_count = 0u;

                sleep_transition(&now_ms);
                state = STATE_STRAIGHT_2;
                state_entry_ms = now_ms;
            }
            break;

        case STATE_STRAIGHT_2:
        default:
            /* Final straight: no more obstacle-triggered transitions */
            break;
        }

        desired = motion_for_state(state);
        motion_ramp_set_target(&ramp, &desired, now_ms, MOTION_RAMP_MS);
        motion_ramp_update(&ramp, now_ms);

        if (!apply_motor_cmd(&rover, &ramp.current)) {
            fatal(MOTOR_WRITE_FAIL_CODE);
        }

        if (r == VL53L1X_POLL_OK) {
            *disp = distance_mm;
        } else if (r != VL53L1X_POLL_ERROR) {
            *disp = SENSOR_POLL_FAIL_CODE;
        }
    }
}