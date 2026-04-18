#include "rover_high.h"

#define ROVER_HIGH_DEFAULT_STRAIGHT_LEFT_SPEED    70
#define ROVER_HIGH_DEFAULT_STRAIGHT_RIGHT_SPEED   70

#define ROVER_HIGH_DEFAULT_TURN_RIGHT_LEFT_SPEED   75
#define ROVER_HIGH_DEFAULT_TURN_RIGHT_RIGHT_SPEED (-75)

#define ROVER_HIGH_DEFAULT_TURN_LEFT_LEFT_SPEED   (-75)
#define ROVER_HIGH_DEFAULT_TURN_LEFT_RIGHT_SPEED   75

static bool rover_high_valid_motor(rover_motor_t motor)
{
    return (motor >= ROVER_MOTOR_1) && (motor <= ROVER_MOTOR_4);
}

static bool rover_high_valid_map(const rover_high_config_t* cfg)
{
    bool used[ROVER_MOTOR_COUNT] = { false, false, false, false };
    rover_motor_t motors[ROVER_MOTOR_COUNT];
    uint8_t i;

    motors[0] = cfg->front_left_motor;
    motors[1] = cfg->rear_left_motor;
    motors[2] = cfg->front_right_motor;
    motors[3] = cfg->rear_right_motor;

    for (i = 0u; i < ROVER_MOTOR_COUNT; ++i) {
        uint8_t idx;

        if (!rover_high_valid_motor(motors[i])) {
            return false;
        }

        idx = (uint8_t)motors[i] - 1u;
        if (used[idx]) {
            return false;
        }

        used[idx] = true;
    }

    return true;
}

static int8_t rover_high_fix_speed(int16_t speed)
{
    return rover_clamp_speed(speed);
}

static int8_t rover_high_apply_invert(int8_t speed, bool invert)
{
    if (invert) {
        return (int8_t)(-speed);
    }
    return speed;
}

static bool rover_high_drive(rover_high_t* high,
                             int16_t left_speed,
                             int16_t right_speed)
{
    int8_t fl;
    int8_t rl;
    int8_t fr;
    int8_t rr;

    int16_t motor1 = 0;
    int16_t motor2 = 0;
    int16_t motor3 = 0;
    int16_t motor4 = 0;

    if ((high == NULL) || (high->rover == NULL)) {
        return false;
    }

    if (!rover_high_valid_map(&high->cfg)) {
        return false;
    }

    fl = rover_high_fix_speed(left_speed);
    rl = rover_high_fix_speed(left_speed);
    fr = rover_high_fix_speed(right_speed);
    rr = rover_high_fix_speed(right_speed);

    fl = rover_high_apply_invert(fl, high->cfg.invert_front_left);
    rl = rover_high_apply_invert(rl, high->cfg.invert_rear_left);
    fr = rover_high_apply_invert(fr, high->cfg.invert_front_right);
    rr = rover_high_apply_invert(rr, high->cfg.invert_rear_right);

    switch (high->cfg.front_left_motor) {
        case ROVER_MOTOR_1: motor1 = fl; break;
        case ROVER_MOTOR_2: motor2 = fl; break;
        case ROVER_MOTOR_3: motor3 = fl; break;
        case ROVER_MOTOR_4: motor4 = fl; break;
        default: return false;
    }

    switch (high->cfg.rear_left_motor) {
        case ROVER_MOTOR_1: motor1 = rl; break;
        case ROVER_MOTOR_2: motor2 = rl; break;
        case ROVER_MOTOR_3: motor3 = rl; break;
        case ROVER_MOTOR_4: motor4 = rl; break;
        default: return false;
    }

    switch (high->cfg.front_right_motor) {
        case ROVER_MOTOR_1: motor1 = fr; break;
        case ROVER_MOTOR_2: motor2 = fr; break;
        case ROVER_MOTOR_3: motor3 = fr; break;
        case ROVER_MOTOR_4: motor4 = fr; break;
        default: return false;
    }

    switch (high->cfg.rear_right_motor) {
        case ROVER_MOTOR_1: motor1 = rr; break;
        case ROVER_MOTOR_2: motor2 = rr; break;
        case ROVER_MOTOR_3: motor3 = rr; break;
        case ROVER_MOTOR_4: motor4 = rr; break;
        default: return false;
    }

    return rover_set_motors(high->rover, motor1, motor2, motor3, motor4);
}

void rover_high_default_config(rover_high_config_t* cfg)
{
    if (cfg == NULL) {
        return;
    }

    cfg->front_left_motor  = ROVER_MOTOR_1;
    cfg->rear_left_motor   = ROVER_MOTOR_2;
    cfg->front_right_motor = ROVER_MOTOR_3;
    cfg->rear_right_motor  = ROVER_MOTOR_4;

    cfg->invert_front_left  = false;
    cfg->invert_rear_left   = false;
    cfg->invert_front_right = false;
    cfg->invert_rear_right  = false;

    cfg->straight_left_speed  = ROVER_HIGH_DEFAULT_STRAIGHT_LEFT_SPEED;
    cfg->straight_right_speed = ROVER_HIGH_DEFAULT_STRAIGHT_RIGHT_SPEED;

    cfg->turn_right_left_speed  = ROVER_HIGH_DEFAULT_TURN_RIGHT_LEFT_SPEED;
    cfg->turn_right_right_speed = ROVER_HIGH_DEFAULT_TURN_RIGHT_RIGHT_SPEED;

    cfg->turn_left_left_speed  = ROVER_HIGH_DEFAULT_TURN_LEFT_LEFT_SPEED;
    cfg->turn_left_right_speed = ROVER_HIGH_DEFAULT_TURN_LEFT_RIGHT_SPEED;
}

bool rover_high_init(rover_high_t* high,
                     rover_t* rover,
                     const rover_high_config_t* cfg)
{
    if ((high == NULL) || (rover == NULL)) {
        return false;
    }

    high->rover = rover;

    if (cfg != NULL) {
        high->cfg = *cfg;
    } else {
        rover_high_default_config(&high->cfg);
    }

    return rover_high_valid_map(&high->cfg);
}

bool rover_high_go_straight(rover_high_t* high)
{
    if (high == NULL) {
        return false;
    }

    return rover_high_drive(high,
                            high->cfg.straight_left_speed,
                            high->cfg.straight_right_speed);
}

bool rover_high_turn_right(rover_high_t* high)
{
    if (high == NULL) {
        return false;
    }

    return rover_high_drive(high,
                            high->cfg.turn_right_left_speed,
                            high->cfg.turn_right_right_speed);
}

bool rover_high_turn_left(rover_high_t* high)
{
    if (high == NULL) {
        return false;
    }

    return rover_high_drive(high,
                            high->cfg.turn_left_left_speed,
                            high->cfg.turn_left_right_speed);
}

bool rover_high_stop(rover_high_t* high)
{
    if ((high == NULL) || (high->rover == NULL)) {
        return false;
    }

    return rover_stop(high->rover);
}

void rover_high_set_straight(rover_high_t* high,
                             int16_t left_speed,
                             int16_t right_speed)
{
    if (high == NULL) {
        return;
    }

    high->cfg.straight_left_speed = rover_high_fix_speed(left_speed);
    high->cfg.straight_right_speed = rover_high_fix_speed(right_speed);
}

void rover_high_set_turn_right(rover_high_t* high,
                               int16_t left_speed,
                               int16_t right_speed)
{
    if (high == NULL) {
        return;
    }

    high->cfg.turn_right_left_speed = rover_high_fix_speed(left_speed);
    high->cfg.turn_right_right_speed = rover_high_fix_speed(right_speed);
}

void rover_high_set_turn_left(rover_high_t* high,
                              int16_t left_speed,
                              int16_t right_speed)
{
    if (high == NULL) {
        return;
    }

    high->cfg.turn_left_left_speed = rover_high_fix_speed(left_speed);
    high->cfg.turn_left_right_speed = rover_high_fix_speed(right_speed);
}

void rover_high_set_motor_map(rover_high_t* high,
                              rover_motor_t front_left_motor,
                              rover_motor_t rear_left_motor,
                              rover_motor_t front_right_motor,
                              rover_motor_t rear_right_motor)
{
    if (high == NULL) {
        return;
    }

    high->cfg.front_left_motor = front_left_motor;
    high->cfg.rear_left_motor = rear_left_motor;
    high->cfg.front_right_motor = front_right_motor;
    high->cfg.rear_right_motor = rear_right_motor;
}

void rover_high_set_invert(rover_high_t* high,
                           bool invert_front_left,
                           bool invert_rear_left,
                           bool invert_front_right,
                           bool invert_rear_right)
{
    if (high == NULL) {
        return;
    }

    high->cfg.invert_front_left = invert_front_left;
    high->cfg.invert_rear_left = invert_rear_left;
    high->cfg.invert_front_right = invert_front_right;
    high->cfg.invert_rear_right = invert_rear_right;
}
