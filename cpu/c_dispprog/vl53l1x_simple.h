#ifndef VL53L1X_SIMPLE_H
#define VL53L1X_SIMPLE_H

#include <stdbool.h>
#include <stdint.h>

/*
 * This header name is kept for compatibility with your existing main.c.
 * The target module here is M5Stack ToF4M, I2C address 0x29.
 */
#ifndef TOF4M_ADDR
#define TOF4M_ADDR 0x29u
#endif

#ifndef VL53L1X_ADDR
#define VL53L1X_ADDR TOF4M_ADDR
#endif

typedef enum {
    VL53L1X_POLL_NONE = 0,
    VL53L1X_POLL_OK,
    VL53L1X_POLL_INVALID,
    VL53L1X_POLL_ERROR
} vl53l1x_poll_result_t;

bool vl53l1x_init(void);
bool vl53l1x_start(void);
bool vl53l1x_stop(void);
vl53l1x_poll_result_t vl53l1x_poll(uint16_t *range_mm);

extern volatile uint8_t  vl53l1x_last_raw_status;
extern volatile uint16_t vl53l1x_last_raw_range;

#endif