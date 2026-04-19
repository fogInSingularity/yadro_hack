#ifndef VL53L1X_SIMPLE_H
#define VL53L1X_SIMPLE_H

#include <stdbool.h>
#include <stdint.h>

#ifndef VL53L1X_ADDR
#define VL53L1X_ADDR 0x29u
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

/*
 * Kept for compatibility with your existing main/debug code.
 * This minimal driver does not use range status to reject samples.
 */
extern volatile uint8_t  vl53l1x_last_raw_status;
extern volatile uint16_t vl53l1x_last_raw_range;

#endif