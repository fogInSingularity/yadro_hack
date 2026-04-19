#include "vl53l1x_simple.h"
#include "i2c.h"

#include <stddef.h>

/*
 * Minimal VL53L1X driver matching the working SV flow:
 *
 *   boot ready
 *   read model ID
 *   write default config 0x002D..0x0087
 *   read interrupt polarity
 *   start once
 *   wait first ready
 *   clear interrupt
 *   stop
 *   apply VHV/phase post-init tweaks
 *   start continuous ranging
 *
 * Polling:
 *   read GPIO data-ready
 *   if ready, read range at 0x0096
 *   clear interrupt
 *   accept the range
 *
 * Important:
 *   We intentionally do NOT reject samples based on RESULT__RANGE_STATUS.
 *   Your working HDL does not read that byte. For approximate distance,
 *   data-ready + range register + clear interrupt is enough.
 */

#define REG_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND           0x0008u
#define REG_VHV_CONFIG__INIT                                0x000Bu
#define REG_PAD_I2C_HV__EXTSUP_CONFIG                       0x002Eu
#define REG_GPIO_HV_MUX__CTRL                               0x0030u
#define REG_GPIO__TIO_HV_STATUS                             0x0031u
#define REG_SYSTEM__INTERRUPT_CLEAR                         0x0086u
#define REG_SYSTEM__MODE_START                              0x0087u
#define REG_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0  0x0096u
#define REG_FIRMWARE__SYSTEM_STATUS                         0x00E5u
#define REG_IDENTIFICATION__MODEL_ID                        0x010Fu

#define REG_DEFAULT_CONFIG_BASE                             0x002Du

#define VL53L1X_MODE_START_BACK_TO_BACK                     0x40u
#define VL53L1X_MODE_STOP                                   0x00u
#define VL53L1X_CLEAR_INTERRUPT                             0x01u

#ifndef VL53L1X_BOOT_POLL_COUNT
#define VL53L1X_BOOT_POLL_COUNT 10000u
#endif

#ifndef VL53L1X_DATAREADY_POLL_COUNT
#define VL53L1X_DATAREADY_POLL_COUNT 20000u
#endif

/*
 * Set to 0 if you want init to exactly mimic your SV sequence:
 * start -> clear -> stop without waiting for first data-ready.
 *
 * ST/ULD-style init normally waits for the first ready event here,
 * and your existing C already did this successfully, so default is 1.
 */
#ifndef VL53L1X_INIT_WAIT_FIRST_READY
#define VL53L1X_INIT_WAIT_FIRST_READY 1
#endif

#ifndef VL53L1X_DELAY_US
static void vl53l1x_delay_us_default(uint32_t us)
{
    uint32_t count;

    if (us == 0u) {
        return;
    }

    /*
     * 50 MHz core, approximate 25 loop iterations/us.
     * Written as shifts/adds to avoid needing RV32I multiply helpers.
     */
    count = (us << 4) + (us << 3) + us; /* us * 25 */

    if (count == 0u) {
        return;
    }

#if defined(__riscv)
    __asm__ volatile (
        "1:\n"
        "addi %[cnt], %[cnt], -1\n"
        "bnez %[cnt], 1b\n"
        : [cnt] "+r" (count)
        :
        : "memory"
    );
#else
    while (count-- != 0u) {
        __asm__ volatile ("" ::: "memory");
    }
#endif
}
#define VL53L1X_DELAY_US(us_) vl53l1x_delay_us_default((uint32_t)(us_))
#endif

/*
 * ST/ULD default configuration block, 0x002D..0x0087 inclusive.
 * Same bytes as your working SV module.
 */
static const uint8_t vl53l1x_default_cfg[] = {
    0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x02, 0x08,
    0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00,
    0x00, 0xFF, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x20, 0x0B, 0x00, 0x00, 0x02, 0x0A, 0x21,
    0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xC8,
    0x00, 0x00, 0x38, 0xFF, 0x01, 0x00, 0x08, 0x00,
    0x00, 0x01, 0xCC, 0x0F, 0x01, 0xF1, 0x0D, 0x01,
    0x68, 0x00, 0x80, 0x08, 0xB8, 0x00, 0x00, 0x00,
    0x00, 0x0F, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x0F, 0x0D, 0x0E, 0x0E, 0x00,
    0x00, 0x02, 0xC7, 0xFF, 0x9B, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00
};

static uint8_t vl53l1x_irq_ready_level = 1u;
static bool vl53l1x_discard_next_range = false;

volatile uint8_t  vl53l1x_last_raw_status = 0xFFu;
volatile uint16_t vl53l1x_last_raw_range  = 0u;

static bool vl53l1x_write_multi(uint16_t reg, const uint8_t *data, uint8_t len)
{
    if ((data == NULL) || (len == 0u)) {
        return false;
    }

    if (!i2c_start()) {
        return false;
    }

    if (!i2c_write_byte((uint8_t)((VL53L1X_ADDR << 1) | 0u))) {
        (void)i2c_stop();
        return false;
    }

    if (!i2c_write_byte((uint8_t)(reg >> 8))) {
        (void)i2c_stop();
        return false;
    }

    if (!i2c_write_byte((uint8_t)reg)) {
        (void)i2c_stop();
        return false;
    }

    for (uint8_t i = 0u; i < len; ++i) {
        if (!i2c_write_byte(data[i])) {
            (void)i2c_stop();
            return false;
        }
    }

    return i2c_stop();
}

static bool vl53l1x_read_multi(uint16_t reg, uint8_t *data, uint8_t len)
{
    if ((data == NULL) || (len == 0u)) {
        return false;
    }

    if (!i2c_start()) {
        return false;
    }

    if (!i2c_write_byte((uint8_t)((VL53L1X_ADDR << 1) | 0u))) {
        (void)i2c_stop();
        return false;
    }

    if (!i2c_write_byte((uint8_t)(reg >> 8))) {
        (void)i2c_stop();
        return false;
    }

    if (!i2c_write_byte((uint8_t)reg)) {
        (void)i2c_stop();
        return false;
    }

    if (!i2c_restart()) {
        (void)i2c_stop();
        return false;
    }

    if (!i2c_write_byte((uint8_t)((VL53L1X_ADDR << 1) | 1u))) {
        (void)i2c_stop();
        return false;
    }

    for (uint8_t i = 0u; i < len; ++i) {
        bool send_nack = (i == (uint8_t)(len - 1u));

        if (!i2c_read_byte(&data[i], send_nack)) {
            (void)i2c_stop();
            return false;
        }
    }

    return i2c_stop();
}

static bool vl53l1x_write8(uint16_t reg, uint8_t value)
{
    return vl53l1x_write_multi(reg, &value, 1u);
}

static bool vl53l1x_read8(uint16_t reg, uint8_t *value)
{
    return vl53l1x_read_multi(reg, value, 1u);
}

static bool vl53l1x_read16(uint16_t reg, uint16_t *value)
{
    uint8_t data[2];

    if (value == NULL) {
        return false;
    }

    if (!vl53l1x_read_multi(reg, data, 2u)) {
        return false;
    }

    *value = ((uint16_t)data[0] << 8) | data[1];
    return true;
}

static bool vl53l1x_wait_boot(void)
{
    uint8_t status = 0u;

    for (uint32_t i = 0u; i < VL53L1X_BOOT_POLL_COUNT; ++i) {
        if (vl53l1x_read8(REG_FIRMWARE__SYSTEM_STATUS, &status) &&
            ((status & 0x01u) != 0u)) {
            return true;
        }

        VL53L1X_DELAY_US(100u);
    }

    return false;
}

static bool vl53l1x_get_interrupt_ready_level(uint8_t *level)
{
    uint8_t mux;

    if (level == NULL) {
        return false;
    }

    if (!vl53l1x_read8(REG_GPIO_HV_MUX__CTRL, &mux)) {
        return false;
    }

    /*
     * ULD logic:
     *   IntPol = !((GPIO_HV_MUX__CTRL & 0x10) >> 4)
     */
    *level = (uint8_t)(!((mux >> 4) & 0x01u));
    return true;
}

static bool vl53l1x_data_ready(bool *ready)
{
    uint8_t gpio_status;

    if (ready == NULL) {
        return false;
    }

    if (!vl53l1x_read8(REG_GPIO__TIO_HV_STATUS, &gpio_status)) {
        return false;
    }

    *ready = ((gpio_status & 0x01u) == vl53l1x_irq_ready_level);
    return true;
}

static bool vl53l1x_wait_data_ready(uint32_t max_polls)
{
    bool ready = false;

    while (max_polls-- != 0u) {
        if (!vl53l1x_data_ready(&ready)) {
            return false;
        }

        if (ready) {
            return true;
        }
    }

    return false;
}

static bool vl53l1x_write_default_config(void)
{
    for (uint16_t i = 0u; i < (uint16_t)sizeof(vl53l1x_default_cfg); ++i) {
        if (!vl53l1x_write8((uint16_t)(REG_DEFAULT_CONFIG_BASE + i),
                            vl53l1x_default_cfg[i])) {
            return false;
        }
    }

    return true;
}

bool vl53l1x_start(void)
{
    return vl53l1x_write8(REG_SYSTEM__MODE_START,
                          VL53L1X_MODE_START_BACK_TO_BACK);
}

bool vl53l1x_stop(void)
{
    return vl53l1x_write8(REG_SYSTEM__MODE_START, VL53L1X_MODE_STOP);
}

static bool vl53l1x_clear_interrupt(void)
{
    return vl53l1x_write8(REG_SYSTEM__INTERRUPT_CLEAR,
                          VL53L1X_CLEAR_INTERRUPT);
}

bool vl53l1x_init(void)
{
    uint16_t model_id = 0u;

    vl53l1x_irq_ready_level = 1u;
    vl53l1x_discard_next_range = false;
    vl53l1x_last_raw_status = 0xFFu;
    vl53l1x_last_raw_range = 0u;

    if (!vl53l1x_wait_boot()) {
        return false;
    }

    if (!vl53l1x_read16(REG_IDENTIFICATION__MODEL_ID, &model_id)) {
        return false;
    }

    /*
     * Real VL53L1X commonly reads 0xEACC here.
     * Your HDL path accepted both, so keep both.
     */
    if ((model_id != 0xEACCu) && (model_id != 0xEBAAu)) {
        return false;
    }

    /*
     * Write as single-byte register writes to match your working SV module.
     * A block write also works on many platforms, but this mirrors the HDL.
     */
    if (!vl53l1x_write_default_config()) {
        return false;
    }

#if defined(VL53L1X_USE_2V8) && VL53L1X_USE_2V8
    if (!vl53l1x_write8(REG_PAD_I2C_HV__EXTSUP_CONFIG, 0x01u)) {
        return false;
    }
#endif

    if (!vl53l1x_get_interrupt_ready_level(&vl53l1x_irq_ready_level)) {
        return false;
    }

    /*
     * SensorInit-style first start/clear/stop.
     */
    if (!vl53l1x_start()) {
        return false;
    }

#if VL53L1X_INIT_WAIT_FIRST_READY
    if (!vl53l1x_wait_data_ready(VL53L1X_DATAREADY_POLL_COUNT)) {
        return false;
    }
#endif

    if (!vl53l1x_clear_interrupt()) {
        return false;
    }

    if (!vl53l1x_stop()) {
        return false;
    }

    /*
     * ST ULD post-init tweaks.
     */
    if (!vl53l1x_write8(REG_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09u)) {
        return false;
    }

    if (!vl53l1x_write8(REG_VHV_CONFIG__INIT, 0x00u)) {
        return false;
    }

    /*
     * Final continuous ranging start.
     */
    if (!vl53l1x_start()) {
        return false;
    }

    /*
     * First result after start can be stale/sync garbage.
     */
    vl53l1x_discard_next_range = true;

    return true;
}

vl53l1x_poll_result_t vl53l1x_poll(uint16_t *range_mm)
{
    bool ready = false;
    uint16_t range = 0u;

    if (range_mm == NULL) {
        return VL53L1X_POLL_ERROR;
    }

    if (!vl53l1x_data_ready(&ready)) {
        return VL53L1X_POLL_ERROR;
    }

    if (!ready) {
        return VL53L1X_POLL_NONE;
    }

    /*
     * This is the important part:
     * read only the distance register, like the working SV module.
     * Do not reject raw status 0x00.
     */
    if (!vl53l1x_read16(REG_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0,
                        &range)) {
        return VL53L1X_POLL_ERROR;
    }

    vl53l1x_last_raw_range = range;
    vl53l1x_last_raw_status = 0x00u; /* status intentionally ignored */

    if (!vl53l1x_clear_interrupt()) {
        return VL53L1X_POLL_ERROR;
    }

    if (vl53l1x_discard_next_range) {
        vl53l1x_discard_next_range = false;
        return VL53L1X_POLL_NONE;
    }

    *range_mm = range;
    return VL53L1X_POLL_OK;
}