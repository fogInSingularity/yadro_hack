#include "vl53l1x_simple.h"
#include "i2c.h"

#include <stddef.h>

#define VL53L1X_KEEP_SYMBOL __attribute__((used, noinline, externally_visible))
/*
 * Small software helpers for RV32I builds without libgcc mul/div support.
 * These symbols intentionally use the libgcc names so GCC/LTO can resolve
 * compiler-generated helper calls during the final link.
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
} vl53l1x_u64_words_t;

VL53L1X_KEEP_SYMBOL uint64_t __ashldi3(uint64_t value, int shift)
{
    vl53l1x_u64_words_t in;
    vl53l1x_u64_words_t out;
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

VL53L1X_KEEP_SYMBOL uint64_t __lshrdi3(uint64_t value, int shift)
{
    vl53l1x_u64_words_t in;
    vl53l1x_u64_words_t out;
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

static uint32_t vl53l1x_soft_mul_u32(uint32_t a, uint32_t b)
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

static uint32_t vl53l1x_soft_udiv_u32(uint32_t numerator, uint32_t denominator)
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

static uint64_t vl53l1x_soft_udiv_u64(uint64_t numerator, uint64_t denominator)
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

/*
 * Export the usual libgcc helper names.
 * Do not make these static.
 */
VL53L1X_KEEP_SYMBOL uint32_t __mulsi3(uint32_t a, uint32_t b)
{
    return vl53l1x_soft_mul_u32(a, b);
}

VL53L1X_KEEP_SYMBOL uint32_t __udivsi3(uint32_t numerator, uint32_t denominator)
{
    return vl53l1x_soft_udiv_u32(numerator, denominator);
}

VL53L1X_KEEP_SYMBOL uint64_t __udivdi3(uint64_t numerator, uint64_t denominator)
{
    return vl53l1x_soft_udiv_u64(numerator, denominator);
}

#ifndef VL53L1X_DELAY_US
static inline void vl53l1x_delay_us(uint32_t us)
{
    /*
     * 50 MHz core => 50 cycles per microsecond.
     *
     * This loop is written in inline asm so the compiler does not
     * optimize it away or change the instruction mix too much.
     *
     * Loop body is typically:
     *   addi  count, count, -1
     *   bnez  count, loop
     *
     * On a simple 1-CPI core, that is about 2 cycles per iteration
     * if branches are not penalized heavily.
     *
     * So iterations per microsecond ~= 50 / 2 = 25.
     */
    uint32_t count = us * 25u;

    __asm__ volatile (
        "1:\n"
        "addi %[cnt], %[cnt], -1\n"
        "bnez %[cnt], 1b\n"
        : [cnt] "+r" (count)
        :
        : "memory"
    );
}
#define VL53L1X_DELAY_US(us) vl53l1x_delay_us((uint32_t)(us))
#endif

#ifndef VL53L1X_BOOT_POLL_COUNT
#define VL53L1X_BOOT_POLL_COUNT 10000u
#endif

#define VL53L1X_TARGET_RATE                 0x0A00u
#define VL53L1X_TIMING_GUARD_US            4528u
#define VL53L1X_RESULT_BUF_LEN             17u

#define REG_SOFT_RESET                                      0x0000u
#define REG_OSC_MEASURED__FAST_OSC__FREQUENCY              0x0006u
#define REG_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND           0x0008u
#define REG_VHV_CONFIG__INIT                                0x000Bu
#define REG_ALGO__PART_TO_PART_RANGE_OFFSET_MM              0x001Eu
#define REG_DSS_CONFIG__TARGET_TOTAL_RATE_MCPS              0x0024u
#define REG_MM_CONFIG__OUTER_OFFSET_MM                      0x0022u
#define REG_PAD_I2C_HV__EXTSUP_CONFIG                       0x002Eu
#define REG_GPIO__TIO_HV_STATUS                             0x0031u
#define REG_SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS       0x0036u
#define REG_SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS     0x0037u
#define REG_ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM    0x0039u
#define REG_ALGO__RANGE_IGNORE_VALID_HEIGHT_MM              0x003Eu
#define REG_ALGO__RANGE_MIN_CLIP                            0x003Fu
#define REG_ALGO__CONSISTENCY_CHECK__TOLERANCE              0x0040u
#define REG_PHASECAL_CONFIG__TIMEOUT_MACROP                 0x004Bu
#define REG_CAL_CONFIG__VCSEL_START                         0x0047u
#define REG_PHASECAL_CONFIG__OVERRIDE                       0x004Du
#define REG_DSS_CONFIG__ROI_MODE_CONTROL                    0x004Fu
#define REG_SYSTEM__THRESH_RATE_HIGH                        0x0050u
#define REG_SYSTEM__THRESH_RATE_LOW                         0x0052u
#define REG_DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT       0x0054u
#define REG_DSS_CONFIG__APERTURE_ATTENUATION                0x0057u
#define REG_MM_CONFIG__TIMEOUT_MACROP_A                     0x005Au
#define REG_MM_CONFIG__TIMEOUT_MACROP_B                     0x005Cu
#define REG_RANGE_CONFIG__TIMEOUT_MACROP_A                  0x005Eu
#define REG_RANGE_CONFIG__VCSEL_PERIOD_A                    0x0060u
#define REG_RANGE_CONFIG__TIMEOUT_MACROP_B                  0x0061u
#define REG_RANGE_CONFIG__VCSEL_PERIOD_B                    0x0063u
#define REG_RANGE_CONFIG__SIGMA_THRESH                      0x0064u
#define REG_RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS     0x0066u
#define REG_RANGE_CONFIG__VALID_PHASE_HIGH                  0x0069u
#define REG_SYSTEM__INTERMEASUREMENT_PERIOD                 0x006Cu
#define REG_SYSTEM__GROUPED_PARAMETER_HOLD_0                0x0071u
#define REG_SYSTEM__SEED_CONFIG                             0x0077u
#define REG_SD_CONFIG__WOI_SD0                              0x0078u
#define REG_SD_CONFIG__WOI_SD1                              0x0079u
#define REG_SD_CONFIG__INITIAL_PHASE_SD0                    0x007Au
#define REG_SD_CONFIG__INITIAL_PHASE_SD1                    0x007Bu
#define REG_SYSTEM__GROUPED_PARAMETER_HOLD_1                0x007Cu
#define REG_SD_CONFIG__QUANTIFIER                           0x007Eu
#define REG_SYSTEM__SEQUENCE_CONFIG                         0x0081u
#define REG_SYSTEM__GROUPED_PARAMETER_HOLD                  0x0082u
#define REG_SYSTEM__INTERRUPT_CLEAR                         0x0086u
#define REG_SYSTEM__MODE_START                              0x0087u
#define REG_RESULT__RANGE_STATUS                            0x0089u
#define REG_PHASECAL_RESULT__VCSEL_START                    0x00D8u
#define REG_RESULT__OSC_CALIBRATE_VAL                       0x00DEu
#define REG_FIRMWARE__SYSTEM_STATUS                         0x00E5u
#define REG_IDENTIFICATION__MODEL_ID                        0x010Fu

#define VL53L1X_RANGE_STATUS_COMPLETE                       9u

struct vl53l1x_results {
    uint8_t  range_status;
    uint8_t  stream_count;
    uint16_t dss_actual_effective_spads_sd0;
    uint16_t ambient_count_rate_mcps_sd0;
    uint16_t final_crosstalk_corrected_range_mm_sd0;
    uint16_t peak_signal_count_rate_crosstalk_corrected_mcps_sd0;
};

static uint16_t fast_osc_frequency;
static uint16_t osc_calibrate_val;
static bool calibrated;
static uint8_t saved_vhv_init;
static uint8_t saved_vhv_timeout;

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

    for (uint8_t i = 0; i < len; i++) {
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

    for (uint8_t i = 0; i < len; i++) {
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

static bool vl53l1x_write16(uint16_t reg, uint16_t value)
{
    uint8_t data[2];

    data[0] = (uint8_t)(value >> 8);
    data[1] = (uint8_t)value;

    return vl53l1x_write_multi(reg, data, 2u);
}

static bool vl53l1x_write32(uint16_t reg, uint32_t value)
{
    uint8_t data[4];

    data[0] = (uint8_t)(value >> 24);
    data[1] = (uint8_t)(value >> 16);
    data[2] = (uint8_t)(value >> 8);
    data[3] = (uint8_t)value;

    return vl53l1x_write_multi(reg, data, 4u);
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

static uint32_t vl53l1x_timeout_us_to_mclks(uint32_t timeout_us,
                                            uint32_t macro_period_us)
{
    if (macro_period_us == 0u) {
        return 0u;
    }

    return (uint32_t)(((((uint64_t)timeout_us << 12) +
                       (macro_period_us >> 1)) / macro_period_us));
}

static uint16_t vl53l1x_encode_timeout(uint32_t timeout_mclks)
{
    uint32_t ls_byte;
    uint16_t ms_byte = 0u;

    if (timeout_mclks == 0u) {
        return 0u;
    }

    ls_byte = timeout_mclks - 1u;

    while ((ls_byte & 0xFFFFFF00u) != 0u) {
        ls_byte >>= 1;
        ms_byte++;
    }

    return (uint16_t)((ms_byte << 8) | (ls_byte & 0xFFu));
}

static uint32_t vl53l1x_calc_macro_period(uint8_t vcsel_period)
{
    uint32_t pll_period_us;
    uint8_t vcsel_period_pclks;
    uint32_t macro_period_us;

    if (fast_osc_frequency == 0u) {
        return 0u;
    }

    pll_period_us = ((uint32_t)1u << 30) / fast_osc_frequency;
    vcsel_period_pclks = (uint8_t)((vcsel_period + 1u) << 1);

    macro_period_us = 2304u * pll_period_us;
    macro_period_us >>= 6;
    macro_period_us *= vcsel_period_pclks;
    macro_period_us >>= 6;

    return macro_period_us;
}

static bool vl53l1x_set_measurement_timing_budget(uint32_t budget_us)
{
    uint8_t vcsel_period;
    uint32_t macro_period_us;
    uint32_t phasecal_timeout_mclks;
    uint32_t range_config_timeout_us;

    if (budget_us <= VL53L1X_TIMING_GUARD_US) {
        return false;
    }

    range_config_timeout_us = budget_us - VL53L1X_TIMING_GUARD_US;

    if (range_config_timeout_us > 1100000u) {
        return false;
    }

    range_config_timeout_us /= 2u;

    if (!vl53l1x_read8(REG_RANGE_CONFIG__VCSEL_PERIOD_A, &vcsel_period)) {
        return false;
    }

    macro_period_us = vl53l1x_calc_macro_period(vcsel_period);

    if (macro_period_us == 0u) {
        return false;
    }

    phasecal_timeout_mclks =
        vl53l1x_timeout_us_to_mclks(1000u, macro_period_us);

    if (phasecal_timeout_mclks > 0xFFu) {
        phasecal_timeout_mclks = 0xFFu;
    }

    if (!vl53l1x_write8(REG_PHASECAL_CONFIG__TIMEOUT_MACROP,
                        (uint8_t)phasecal_timeout_mclks)) {
        return false;
    }

    if (!vl53l1x_write16(
            REG_MM_CONFIG__TIMEOUT_MACROP_A,
            vl53l1x_encode_timeout(
                vl53l1x_timeout_us_to_mclks(1u, macro_period_us)))) {
        return false;
    }

    if (!vl53l1x_write16(
            REG_RANGE_CONFIG__TIMEOUT_MACROP_A,
            vl53l1x_encode_timeout(
                vl53l1x_timeout_us_to_mclks(range_config_timeout_us,
                                            macro_period_us)))) {
        return false;
    }

    if (!vl53l1x_read8(REG_RANGE_CONFIG__VCSEL_PERIOD_B, &vcsel_period)) {
        return false;
    }

    macro_period_us = vl53l1x_calc_macro_period(vcsel_period);

    if (macro_period_us == 0u) {
        return false;
    }

    if (!vl53l1x_write16(
            REG_MM_CONFIG__TIMEOUT_MACROP_B,
            vl53l1x_encode_timeout(
                vl53l1x_timeout_us_to_mclks(1u, macro_period_us)))) {
        return false;
    }

    if (!vl53l1x_write16(
            REG_RANGE_CONFIG__TIMEOUT_MACROP_B,
            vl53l1x_encode_timeout(
                vl53l1x_timeout_us_to_mclks(range_config_timeout_us,
                                            macro_period_us)))) {
        return false;
    }

    return true;
}

static bool vl53l1x_set_long_distance_mode(void)
{
    if (!vl53l1x_write8(REG_RANGE_CONFIG__VCSEL_PERIOD_A, 0x0Fu)) {
        return false;
    }

    if (!vl53l1x_write8(REG_RANGE_CONFIG__VCSEL_PERIOD_B, 0x0Du)) {
        return false;
    }

    if (!vl53l1x_write8(REG_RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8u)) {
        return false;
    }

    if (!vl53l1x_write8(REG_SD_CONFIG__WOI_SD0, 0x0Fu)) {
        return false;
    }

    if (!vl53l1x_write8(REG_SD_CONFIG__WOI_SD1, 0x0Du)) {
        return false;
    }

    if (!vl53l1x_write8(REG_SD_CONFIG__INITIAL_PHASE_SD0, 14u)) {
        return false;
    }

    if (!vl53l1x_write8(REG_SD_CONFIG__INITIAL_PHASE_SD1, 14u)) {
        return false;
    }

    return vl53l1x_set_measurement_timing_budget(VL53L1X_TIMING_BUDGET_US);
}

#include <stdbool.h>
#include <stdint.h>

/* add these if not already present */
#define REG_SOFT_RESET                                      0x0000u
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

#ifndef VL53L1X_BOOT_POLL_COUNT
#define VL53L1X_BOOT_POLL_COUNT 10000u
#endif

#ifndef VL53L1X_DATAREADY_POLL_COUNT
#define VL53L1X_DATAREADY_POLL_COUNT 20000u
#endif

#define VL53L1X_RAW_RANGE_STATUS_GOOD 0x09u

static uint8_t vl53l1x_irq_ready_level = 1u;
static bool vl53l1x_discard_next_range = false;

/*
 * ST/ULD default configuration block, 0x002D..0x0087 inclusive.
 * This matches the working HDL flow and ST's ULD-style implementations.
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

static bool vl53l1x_wait_boot(void)
{
    uint8_t status = 0u;

    for (uint32_t i = 0; i < VL53L1X_BOOT_POLL_COUNT; ++i) {
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

    /* ST logic: IntPol = !((GPIO_HV_MUX__CTRL & 0x10) >> 4) */
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

static volatile uint16_t* disp = (uint16_t*)0x20;
bool vl53l1x_init(void)
{
    uint16_t model_id;

    vl53l1x_irq_ready_level = 1u;
    vl53l1x_discard_next_range = false;

    if (!vl53l1x_read16(REG_IDENTIFICATION__MODEL_ID, &model_id)) {
        *disp = 0x1100;
        return false;
    }

    /* Your HDL accepts both. Keep only 0xEACC if you want to be stricter. */
    if ((model_id != 0xEACCu) && (model_id != 0xEBAAu)) {
        *disp = 0x1101;
        return false;
    }

    /* Optional but generally helpful */
    if (!vl53l1x_write8(REG_SOFT_RESET, 0x00u)) {
        *disp = 0x1102;
        return false;
    }
    VL53L1X_DELAY_US(100u);

    if (!vl53l1x_write8(REG_SOFT_RESET, 0x01u)) {
        *disp = 0x1103;
        return false;
    }
    VL53L1X_DELAY_US(1000u);

    if (!vl53l1x_wait_boot()) {
        *disp = 0x1104;
        return false;
    }

    if (!vl53l1x_write_multi(0x002Du,
                             vl53l1x_default_cfg,
                             (uint8_t)sizeof(vl53l1x_default_cfg))) {
        *disp = 0x1105;
        return false;
    }

#if defined(VL53L1X_USE_2V8) && VL53L1X_USE_2V8
    if (!vl53l1x_write8(REG_PAD_I2C_HV__EXTSUP_CONFIG, 0x01u)) {
        *disp = 0x1106;
        return false;
    }
#endif

    if (!vl53l1x_get_interrupt_ready_level(&vl53l1x_irq_ready_level)) {
        *disp = 0x1107;
        return false;
    }

    /* SensorInit(): start once, wait ready, clear, stop */
    if (!vl53l1x_write8(REG_SYSTEM__MODE_START, 0x40u)) {
        *disp = 0x1108;
        return false;
    }

    if (!vl53l1x_wait_data_ready(VL53L1X_DATAREADY_POLL_COUNT)) {
        *disp = 0x1109;
        return false;
    }

    if (!vl53l1x_write8(REG_SYSTEM__INTERRUPT_CLEAR, 0x01u)) {
        *disp = 0x110A;
        return false;
    }

    if (!vl53l1x_write8(REG_SYSTEM__MODE_START, 0x00u)) {
        *disp = 0x110B;
        return false;
    }

    /* ST ULD post-init tweaks */
    if (!vl53l1x_write8(REG_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09u)) {
        *disp = 0x110C;
        return false;
    }

    if (!vl53l1x_write8(REG_VHV_CONFIG__INIT, 0x00u)) {
        *disp = 0x110D;
        return false;
    }

    /* Final continuous start */
    if (!vl53l1x_write8(REG_SYSTEM__MODE_START, 0x40u)) {
        *disp = 0x110E;
        return false;
    }

    /*
     * First sample after (re)start is often discarded in simple host drivers.
     * Remove this if you truly do not care.
     */
    vl53l1x_discard_next_range = true;
    return true;
}

vl53l1x_poll_result_t vl53l1x_poll(uint16_t *range_mm)
{
    bool ready;
    uint8_t raw_status;
    uint16_t range;

    if (range_mm == NULL) {
        return VL53L1X_POLL_ERROR;
    }

    if (!vl53l1x_data_ready(&ready)) {
        return VL53L1X_POLL_ERROR;
    }

    if (!ready) {
        return VL53L1X_POLL_NONE;
    }

    if (!vl53l1x_read8(REG_RESULT__RANGE_STATUS, &raw_status)) {
        return VL53L1X_POLL_ERROR;
    }

    if (!vl53l1x_read16(REG_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0,
                        &range)) {
        return VL53L1X_POLL_ERROR;
    }

    if (!vl53l1x_write8(REG_SYSTEM__INTERRUPT_CLEAR, 0x01u)) {
        return VL53L1X_POLL_ERROR;
    }

    if (vl53l1x_discard_next_range) {
        vl53l1x_discard_next_range = false;
        return VL53L1X_POLL_NONE;
    }

    *range_mm = range;

    if (raw_status == VL53L1X_RAW_RANGE_STATUS_GOOD) {
        return VL53L1X_POLL_OK;
    }

    return VL53L1X_POLL_INVALID;
}