#ifndef ROVER_H
#define ROVER_H

#include <stdint.h>
#include <stdbool.h>

#define ROVER_I2C_ADDR_DEFAULT  0x38u

#define ROVER_MOTOR_COUNT       4u
#define ROVER_SPEED_MIN        (-127)
#define ROVER_SPEED_MAX         127

/*
 * RoverC-Pro motor registers:
 *   Motor 1 -> 0x00
 *   Motor 2 -> 0x01
 *   Motor 3 -> 0x02
 *   Motor 4 -> 0x03
 *
 * This driver intentionally exposes motor numbers/registers directly.
 * Physical wheel placement should be handled by the higher-level wrapper.
 */
typedef enum {
    ROVER_MOTOR_1 = 1u,
    ROVER_MOTOR_2 = 2u,
    ROVER_MOTOR_3 = 3u,
    ROVER_MOTOR_4 = 4u
} rover_motor_t;

typedef struct {
    uint8_t addr;
} rover_t;

/*
 * Initializes only the software handle.
 * It does not touch the I2C bus.
 */
bool rover_init(rover_t* rover, uint8_t addr);

/*
 * Checks whether a device ACKs the configured RoverC-Pro address.
 */
bool rover_probe(const rover_t* rover);

/*
 * Clamp helper for higher-level movement code.
 */
int8_t rover_clamp_speed(int16_t speed);

/*
 * Low-level motor control.
 *
 * Speed range:
 *   -127..-1  reverse
 *      0      stop
 *    1..127   forward
 */
bool rover_set_motor(const rover_t* rover,
                     rover_motor_t motor,
                     int16_t speed);

bool rover_get_motor(const rover_t* rover,
                     rover_motor_t motor,
                     int8_t* speed);

bool rover_set_motors(const rover_t* rover,
                      int16_t motor1,
                      int16_t motor2,
                      int16_t motor3,
                      int16_t motor4);

bool rover_get_motors(const rover_t* rover,
                      int8_t* motor1,
                      int8_t* motor2,
                      int8_t* motor3,
                      int8_t* motor4);

bool rover_stop(const rover_t* rover);

#endif /* ROVER_H */
