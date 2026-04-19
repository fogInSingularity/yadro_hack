#include "vl53l1x_simple.h"
#include "i2c.h"

#include <stddef.h>

/*
 * Minimal ToF4M driver.
 *
 * The public names still say vl53l1x because your main.c already uses them.
 * Internally this is written to match your working SystemVerilog transaction
 * sequence for the M5Stack ToF4M / VL53L1CX unit.
 */

#define TOF4M_KEEP_SYMBOL __attribute__((used, noinline, externally_visible))

/*
 * Small RV32I software helpers.
 * Keep these if your bare-metal RV32I build has no libgcc mul/div support.
 */

typedef union {
    uint64_t u64;
    struct {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
        uint32_t lo;
        uint32_t hi;
#else
        uint32_t hi;
        uint32_t lo;
#endif
    } w;
} tof4m_u64_words_t;

TOF4M_KEEP_SYMBOL uint64_t __ashldi3(uint64_t value, int shift)
{
    tof4m_u64_words_t in;
    tof4m_u64_words_t out;
    uint32_t s;

    in.u64 = value;
    out.w.lo = 0u;
    out.w.hi = 0u;

    if (shift <= 0) {
        return value;
    }

    s = (uint32_t)shift;

    if (s >= 64u) {
        return 0u;
    }

    if (s >= 32u) {
        out.w.hi = in.w.lo << (s - 32u);
        return out.u64;
    }

    out.w.hi = (in.w.hi << s) | (in.w.lo >> (32u - s));
    out.w.lo = in.w.lo << s;

    return out.u64;
}

TOF4M_KEEP_SYMBOL uint64_t __lshrdi3(uint64_t value, int shift)
{
    tof4m_u64_words_t in;
    tof4m_u64_words_t out;
    uint32_t s;

    in.u64 = value;
    out.w.lo = 0u;
    out.w.hi = 0u;

    if (shift <= 0) {
        return value;
    }

    s = (uint32_t)shift;

    if (s >= 64u) {
        return 0u;
    }

    if (s >= 32u) {
        out.w.lo = in.w.hi >> (s - 32u);
        return out.u64;
    }

    out.w.lo = (in.w.lo >> s) | (in.w.hi << (32u - s));
    out.w.hi = in.w.hi >> s;

    return out.u64;
}

static uint32_t tof4m_soft_mul_u32(uint32_t a, uint32_t b)
{
    uint32_t result = 0u;

    while (b != 0u) {
        if ((b & 1u) != 0u) {
            result += a;
        }

        a <<= 1;
        b >>= 1;
    }

    return result;
}

static uint32_t tof4m_soft_udiv_u32(uint32_t numerator, uint32_t denominator)
{
    uint32_t quotient = 0u;
    uint32_t remainder = 0u;

    if (denominator == 0u) {
        return 0u;
    }

    for (int bit = 31; bit >= 0; --bit) {
        remainder <<= 1;
        remainder |= (numerator >> bit) & 1u;

        if (remainder >= denominator) {
            remainder -= denominator;
            quotient |= ((uint32_t)1u << bit);
        }
    }

    return quotient;
}

static uint64_t tof4m_soft_udiv_u64(uint64_t numerator, uint64_t denominator)
{
    uint64_t quotient = 0u;
    uint64_t remainder = 0u;

    if (denominator == 0u) {
        return 0u;
    }

    for (int bit = 63; bit >= 0; --bit) {
        remainder <<= 1;
        remainder |= (numerator >> bit) & 1u;

        if (remainder >= denominator) {
            remainder -= denominator;
            quotient |= (((uint64_t)1u) << bit);
        }
    }

    return quotient;
}

TOF4M_KEEP_SYMBOL uint32_t __mulsi3(uint32_t a, uint32_t b)
{
    return tof4m_soft_mul_u32(a, b);
}

TOF4M_KEEP_SYMBOL uint32_t __udivsi3(uint32_t numerator, uint32_t denominator)
{
    return tof4m_soft_udiv_u32(numerator, denominator);
}

TOF4M_KEEP_SYMBOL uint64_t __udivdi3(uint64_t numerator, uint64_t denominator)
{
    return tof4m_soft_udiv_u64(numerator, denominator);
}

#ifndef VL53L1X_DELAY_US
static void tof4m_delay_us(uint32_t us)
{
    uint32_t count;

    if (us == 0u) {
        return;
    }

    /*
     * Approximate delay for 50 MHz core.
     * 25 loop iterations/us, written without multiplication.
     */
    count = (us << 4) + (us << 3) + us;

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
#define VL53L1X_DELAY_US(us_) tof4m_delay_us((uint32_t)(us_))
#endif

#ifndef TOF4M_BOOT_POLL_COUNT
#define TOF4M_BOOT_POLL_COUNT 10000u
#endif

#ifndef TOF4M_STATUS_POLL_LIMIT
#define TOF4M_STATUS_POLL_LIMIT 512u
#endif

#define REG_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND           0x0008u
#define REG_VHV_CONFIG__INIT                                0x000Bu
#define REG_PAD_I2C_HV__EXTSUP_CONFIG                       0x002Eu
#define REG_GPIO_HV_MUX__CTRL                               0x0030u
#define REG_GPIO__TIO_HV_STATUS                             0x0031u
#define REG_SYSTEM__INTERRUPT_CLEAR                         0x0086u
#define REG_SYSTEM__MODE_START                              0x0087u
#define REG_RESULT__RANGE_STATUS                            0x0089u
#define REG_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0  0x0096u
#define REG_FIRMWARE__SYSTEM_STATUS                         0x00E5u
#define REG_IDENTIFICATION__MODEL_ID                        0x010Fu

#define REG_DEFAULT_CONFIG_BASE                             0x002Du

#define TOF4M_START_RANGING_BACK_TO_BACK                    0x40u
#define TOF4M_STOP_RANGING                                  0x00u
#define TOF4M_CLEAR_RANGE_INTERRUPT                         0x01u

#define TOF4M_RESULT_BUF_LEN                                17u

typedef struct {
    uint8_t  range_status;
    uint8_t  stream_count;
    uint16_t range_mm;
} tof4m_result_t;

/*
 * Same 0x002D..0x0087 block as your working SV module.
 */
static const uint8_t tof4m_default_cfg[] = {
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

static uint8_t tof4m_irq_ready_level = 1u;
static bool tof4m_discard_next_range = false;

volatile uint8_t  vl53l1x_last_raw_status = 0xFFu;
volatile uint16_t vl53l1x_last_raw_range  = 0u;

static bool tof4m_write_multi(uint16_t reg, const uint8_t *data, uint8_t len)
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

static bool tof4m_read_multi(uint16_t reg, uint8_t *data, uint8_t len)
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

static bool tof4m_write8(uint16_t reg, uint8_t value)
{
    return tof4m_write_multi(reg, &value, 1u);
}

static bool tof4m_read8(uint16_t reg, uint8_t *value)
{
    return tof4m_read_multi(reg, value, 1u);
}

static bool tof4m_read16(uint16_t reg, uint16_t *value)
{
    uint8_t data[2];

    if (value == NULL) {
        return false;
    }

    if (!tof4m_read_multi(reg, data, 2u)) {
        return false;
    }

    *value = ((uint16_t)data[0] << 8) | data[1];
    return true;
}

static bool tof4m_clear_interrupt(void)
{
    return tof4m_write8(REG_SYSTEM__INTERRUPT_CLEAR,
                        TOF4M_CLEAR_RANGE_INTERRUPT);
}

static bool tof4m_wait_boot(void)
{
    uint8_t status = 0u;

    for (uint32_t i = 0u; i < TOF4M_BOOT_POLL_COUNT; ++i) {
        if (tof4m_read8(REG_FIRMWARE__SYSTEM_STATUS, &status) &&
            ((status & 0x01u) != 0u)) {
            return true;
        }

        VL53L1X_DELAY_US(100u);
    }

    return false;
}

static bool tof4m_get_interrupt_ready_level(uint8_t *level)
{
    uint8_t mux;

    if (level == NULL) {
        return false;
    }

    if (!tof4m_read8(REG_GPIO_HV_MUX__CTRL, &mux)) {
        return false;
    }

    /*
     * Same as your SV:
     * irq_ready_level = ~GPIO_HV_MUX__CTRL[4]
     */
    *level = (uint8_t)(!((mux >> 4) & 0x01u));
    return true;
}

static bool tof4m_data_ready(bool *ready)
{
    uint8_t gpio_status;

    if (ready == NULL) {
        return false;
    }

    if (!tof4m_read8(REG_GPIO__TIO_HV_STATUS, &gpio_status)) {
        return false;
    }

    *ready = ((gpio_status & 0x01u) == tof4m_irq_ready_level);
    return true;
}

static bool tof4m_wait_data_ready(uint32_t max_polls, bool *timed_out)
{
    bool ready = false;

    if (timed_out != NULL) {
        *timed_out = false;
    }

    while (max_polls-- != 0u) {
        if (!tof4m_data_ready(&ready)) {
            return false;
        }

        if (ready) {
            return true;
        }
    }

    if (timed_out != NULL) {
        *timed_out = true;
    }

    return true;
}

static bool tof4m_write_default_config(void)
{
    /*
     * Write byte-by-byte to match the working SV transaction stream exactly.
     */
    for (uint16_t i = 0u; i < (uint16_t)sizeof(tof4m_default_cfg); ++i) {
        if (!tof4m_write8((uint16_t)(REG_DEFAULT_CONFIG_BASE + i),
                          tof4m_default_cfg[i])) {
            return false;
        }
    }

    return true;
}

static bool tof4m_read_result(tof4m_result_t *result)
{
    uint8_t buf[TOF4M_RESULT_BUF_LEN];
    uint16_t direct_range = 0u;

    if (result == NULL) {
        return false;
    }

    result->range_status = 0xFFu;
    result->stream_count = 0u;
    result->range_mm = 0u;

    /*
     * Read the normal result block beginning at 0x0089.
     *
     * Offsets:
     *   0  = range status
     *   2  = stream count
     *   13 = range high byte, register 0x0096
     *   14 = range low byte,  register 0x0097
     */
    if (tof4m_read_multi(REG_RESULT__RANGE_STATUS,
                         buf,
                         TOF4M_RESULT_BUF_LEN)) {
        result->range_status = buf[0];
        result->stream_count = buf[2];
        result->range_mm = ((uint16_t)buf[13] << 8) | buf[14];
    } else {
        return false;
    }

    /*
     * Safety fallback: if the block read reports zero, try the exact register
     * used by your SV module.
     */
    if (result->range_mm == 0u) {
        if (!tof4m_read16(REG_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0,
                          &direct_range)) {
            return false;
        }

        if (direct_range != 0u) {
            result->range_mm = direct_range;
        }
    }

    return true;
}

bool vl53l1x_start(void)
{
    /*
     * Clear before start so a stale latched interrupt does not cause the first
     * host poll to read an old zero result.
     */
    if (!tof4m_clear_interrupt()) {
        return false;
    }

    return tof4m_write8(REG_SYSTEM__MODE_START,
                        TOF4M_START_RANGING_BACK_TO_BACK);
}

bool vl53l1x_stop(void)
{
    return tof4m_write8(REG_SYSTEM__MODE_START, TOF4M_STOP_RANGING);
}

bool vl53l1x_init(void)
{
    uint16_t model_id = 0u;

    tof4m_irq_ready_level = 1u;
    tof4m_discard_next_range = false;
    vl53l1x_last_raw_status = 0xFFu;
    vl53l1x_last_raw_range = 0u;

    /*
     * Match your working SV order:
     *   boot first, then model ID.
     */
    if (!tof4m_wait_boot()) {
        return false;
    }

    if (!tof4m_read16(REG_IDENTIFICATION__MODEL_ID, &model_id)) {
        return false;
    }

    /*
     * Your HDL accepts both.
     */
    if ((model_id != 0xEACCu) && (model_id != 0xEBAAu)) {
        return false;
    }

    if (!tof4m_write_default_config()) {
        return false;
    }

#if defined(VL53L1X_USE_2V8) && VL53L1X_USE_2V8
    if (!tof4m_write8(REG_PAD_I2C_HV__EXTSUP_CONFIG, 0x01u)) {
        return false;
    }
#endif

    if (!tof4m_get_interrupt_ready_level(&tof4m_irq_ready_level)) {
        return false;
    }

    /*
     * SensorInit-style sequence matching your SV:
     *
     *   MODE_START = 0x40
     *   INTERRUPT_CLEAR = 0x01
     *   MODE_START = 0x00
     *
     * Do not wait for data-ready here. Your working SV does not wait here.
     */
    if (!tof4m_write8(REG_SYSTEM__MODE_START,
                      TOF4M_START_RANGING_BACK_TO_BACK)) {
        return false;
    }

    if (!tof4m_clear_interrupt()) {
        return false;
    }

    if (!tof4m_write8(REG_SYSTEM__MODE_START, TOF4M_STOP_RANGING)) {
        return false;
    }

    /*
     * Post-init tweaks from your SV.
     */
    if (!tof4m_write8(REG_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09u)) {
        return false;
    }

    if (!tof4m_write8(REG_VHV_CONFIG__INIT, 0x00u)) {
        return false;
    }

    /*
     * Final continuous start.
     */
    if (!vl53l1x_start()) {
        return false;
    }

    /*
     * Your SV discards the first sample after final start.
     */
    tof4m_discard_next_range = true;

    return true;
}

vl53l1x_poll_result_t vl53l1x_poll(uint16_t *range_mm)
{
    bool timed_out = false;
    tof4m_result_t result;

    if (range_mm == NULL) {
        return VL53L1X_POLL_ERROR;
    }

    /*
     * Match your SV sample path:
     * poll GPIO status internally before reading the range.
     */
    if (!tof4m_wait_data_ready(TOF4M_STATUS_POLL_LIMIT, &timed_out)) {
        return VL53L1X_POLL_ERROR;
    }

    if (timed_out) {
        return VL53L1X_POLL_NONE;
    }

    if (!tof4m_read_result(&result)) {
        return VL53L1X_POLL_ERROR;
    }

    vl53l1x_last_raw_status = result.range_status;
    vl53l1x_last_raw_range = result.range_mm;

    if (!tof4m_clear_interrupt()) {
        return VL53L1X_POLL_ERROR;
    }

    /*
     * Status 18 is the common sync/first-stream interrupt.
     * Also discard the first post-start sample, like your SV.
     */
    if (tof4m_discard_next_range || (result.range_status == 18u)) {
        tof4m_discard_next_range = false;
        return VL53L1X_POLL_NONE;
    }

    /*
     * ToF4M minimum useful range is around 40 mm, so zero is not useful data.
     * Returning INVALID here makes your existing main display F0ss instead of
     * silently showing a bogus zero.
     */
    if (result.range_mm == 0u) {
        return VL53L1X_POLL_INVALID;
    }

    *range_mm = result.range_mm;
    return VL53L1X_POLL_OK;
}