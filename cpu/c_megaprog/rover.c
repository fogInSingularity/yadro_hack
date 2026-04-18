#include "rover.h"
#include "i2c.h"

#include <stddef.h>

#define ROVER_REG_MOTOR_BASE  0x00u

static bool rover_is_valid(const rover_t* rover)
{
    return (rover != NULL) && ((rover->addr & 0x80u) == 0u);
}

static bool rover_motor_to_reg(rover_motor_t motor, uint8_t* reg)
{
    if (reg == NULL) {
        return false;
    }

    switch (motor) {
    case ROVER_MOTOR_1:
        *reg = ROVER_REG_MOTOR_BASE + 0u;
        return true;

    case ROVER_MOTOR_2:
        *reg = ROVER_REG_MOTOR_BASE + 1u;
        return true;

    case ROVER_MOTOR_3:
        *reg = ROVER_REG_MOTOR_BASE + 2u;
        return true;

    case ROVER_MOTOR_4:
        *reg = ROVER_REG_MOTOR_BASE + 3u;
        return true;

    default:
        return false;
    }
}

static uint8_t rover_speed_to_u8(int16_t speed)
{
    int8_t clamped = rover_clamp_speed(speed);

    /*
     * Conversion to uint8_t is defined modulo 256.
     * RoverC-Pro firmware interprets the byte as signed speed.
     */
    return (uint8_t)clamped;
}

static int8_t rover_u8_to_speed(uint8_t raw)
{
    /*
     * Avoid implementation-defined uint8_t -> int8_t conversion for values
     * above 127. int8_t exists here because rover.h includes <stdint.h>.
     */
    if (raw <= 127u) {
        return (int8_t)raw;
    }

    return (int8_t)((int16_t)raw - 256);
}

bool rover_init(rover_t* rover, uint8_t addr)
{
    if (rover == NULL) {
        return false;
    }

    if ((addr & 0x80u) != 0u) {
        return false;
    }

    rover->addr = addr;
    return true;
}

bool rover_probe(const rover_t* rover)
{
    if (!rover_is_valid(rover)) {
        return false;
    }

    return i2c_probe(rover->addr);
}

int8_t rover_clamp_speed(int16_t speed)
{
    if (speed > ROVER_SPEED_MAX) {
        return (int8_t)ROVER_SPEED_MAX;
    }

    if (speed < ROVER_SPEED_MIN) {
        return (int8_t)ROVER_SPEED_MIN;
    }

    return (int8_t)speed;
}

bool rover_set_motor(const rover_t* rover,
                     rover_motor_t motor,
                     int16_t speed)
{
    uint8_t reg;
    uint8_t value;

    if (!rover_is_valid(rover)) {
        return false;
    }

    if (!rover_motor_to_reg(motor, &reg)) {
        return false;
    }

    value = rover_speed_to_u8(speed);

    return i2c_write_reg8(rover->addr, reg, value);
}

bool rover_get_motor(const rover_t* rover,
                     rover_motor_t motor,
                     int8_t* speed)
{
    uint8_t reg;
    uint8_t raw;

    if (!rover_is_valid(rover) || speed == NULL) {
        return false;
    }

    if (!rover_motor_to_reg(motor, &reg)) {
        return false;
    }

    if (!i2c_read_reg8(rover->addr, reg, &raw)) {
        return false;
    }

    *speed = rover_u8_to_speed(raw);
    return true;
}

bool rover_set_motors(const rover_t* rover,
                      int16_t motor1,
                      int16_t motor2,
                      int16_t motor3,
                      int16_t motor4)
{
    uint8_t data[ROVER_MOTOR_COUNT];

    if (!rover_is_valid(rover)) {
        return false;
    }

    data[0] = rover_speed_to_u8(motor1);
    data[1] = rover_speed_to_u8(motor2);
    data[2] = rover_speed_to_u8(motor3);
    data[3] = rover_speed_to_u8(motor4);

    return i2c_write_reg8_burst(rover->addr,
                                ROVER_REG_MOTOR_BASE,
                                data,
                                sizeof(data));
}

bool rover_get_motors(const rover_t* rover,
                      int8_t* motor1,
                      int8_t* motor2,
                      int8_t* motor3,
                      int8_t* motor4)
{
    uint8_t data[ROVER_MOTOR_COUNT];

    if (!rover_is_valid(rover)) {
        return false;
    }

    if (motor1 == NULL || motor2 == NULL || motor3 == NULL || motor4 == NULL) {
        return false;
    }

    if (!i2c_read_reg8_burst(rover->addr,
                             ROVER_REG_MOTOR_BASE,
                             data,
                             sizeof(data))) {
        return false;
    }

    *motor1 = rover_u8_to_speed(data[0]);
    *motor2 = rover_u8_to_speed(data[1]);
    *motor3 = rover_u8_to_speed(data[2]);
    *motor4 = rover_u8_to_speed(data[3]);

    return true;
}

bool rover_stop(const rover_t* rover)
{
    return rover_set_motors(rover, 0, 0, 0, 0);
}
