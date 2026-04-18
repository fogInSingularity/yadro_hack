#ifndef ROVER_HIGH_H
#define ROVER_HIGH_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "rover.h"

typedef struct {
    /* Physical wheel -> low-level motor register mapping */
    rover_motor_t front_left_motor;
    rover_motor_t rear_left_motor;
    rover_motor_t front_right_motor;
    rover_motor_t rear_right_motor;

    /* Set true if that wheel must be flipped in software */
    bool invert_front_left;
    bool invert_rear_left;
    bool invert_front_right;
    bool invert_rear_right;

    /* Straight-line tuning */
    int8_t straight_left_speed;
    int8_t straight_right_speed;

    /* Right turn tuning */
    int8_t turn_right_left_speed;
    int8_t turn_right_right_speed;

    /* Left turn tuning */
    int8_t turn_left_left_speed;
    int8_t turn_left_right_speed;
} rover_high_config_t;

typedef struct {
    rover_t* rover;
    rover_high_config_t cfg;
} rover_high_t;

/* Fill config with simple defaults */
void rover_high_default_config(rover_high_config_t* cfg);

/* Initialize wrapper, cfg==NULL uses defaults */
bool rover_high_init(rover_high_t* high,
                     rover_t* rover,
                     const rover_high_config_t* cfg);

/* High-level moves */
bool rover_high_go_straight(rover_high_t* high);
bool rover_high_turn_right(rover_high_t* high);
bool rover_high_turn_left(rover_high_t* high);
bool rover_high_stop(rover_high_t* high);

/* Knobs */
void rover_high_set_straight(rover_high_t* high,
                             int16_t left_speed,
                             int16_t right_speed);

void rover_high_set_turn_right(rover_high_t* high,
                               int16_t left_speed,
                               int16_t right_speed);

void rover_high_set_turn_left(rover_high_t* high,
                              int16_t left_speed,
                              int16_t right_speed);

/* Setup helpers */
void rover_high_set_motor_map(rover_high_t* high,
                              rover_motor_t front_left_motor,
                              rover_motor_t rear_left_motor,
                              rover_motor_t front_right_motor,
                              rover_motor_t rear_right_motor);

void rover_high_set_invert(rover_high_t* high,
                           bool invert_front_left,
                           bool invert_rear_left,
                           bool invert_front_right,
                           bool invert_rear_right);

#endif /* ROVER_HIGH_H */
