#ifndef VL53L1X_SIMPLE_H
#define VL53L1X_SIMPLE_H

#include <stdint.h>
#include <stdbool.h>

#ifndef VL53L1X_ADDR
#define VL53L1X_ADDR 0x29u
#endif

#ifndef VL53L1X_USE_2V8
#define VL53L1X_USE_2V8 1
#endif

#ifndef VL53L1X_TIMING_BUDGET_US
#define VL53L1X_TIMING_BUDGET_US 50000u
#endif

#ifndef VL53L1X_INTERMEASUREMENT_MS
#define VL53L1X_INTERMEASUREMENT_MS 50u
#endif

typedef enum {
    VL53L1X_POLL_ERROR   = -1,
    VL53L1X_POLL_NONE    =  0,
    VL53L1X_POLL_OK      =  1,
    VL53L1X_POLL_INVALID =  2
} vl53l1x_poll_result_t;

bool vl53l1x_init(void);
vl53l1x_poll_result_t vl53l1x_poll(uint16_t *range_mm);

#endif