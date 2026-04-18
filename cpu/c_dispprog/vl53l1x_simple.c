#include "vl53l1x_simple.h"
#include "i2c.h"

#include <stddef.h>

/*
 * Replace this with a real delay if you have one:
 *
 *   #define VL53L1X_DELAY_US(us) delay_us(us)
 *
 * before compiling this file.
 */
#ifndef VL53L1X_DELAY_US
static void vl53l1x_default_delay_us(uint32_t ms)
{
    const volatile uint32_t loops_per_ms = 50000; // calibrate experimentally

    for (uint32_t m = 0; m < ms; ++m) {
        for (volatile uint32_t i = 0; i < loops_per_ms; ++i) {
            __asm__ volatile ("" ::: "memory");
        }
    }
}
#define VL53L1X_DELAY_US(us) vl53l1x_default_delay_us((uint32_t)(us))
#endif

#ifndef VL53L1X_BOOT_POLL_COUNT
#define VL53L1X_BOOT_POLL_COUNT 10000u
#endif

#define VL53L1X_TARGET_RATE                 0x0A00u
#define VL53L1X_TIMING_GUARD_US            4528u
#define VL53L1X_RESULT_BUF_LEN               17u

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

#define VL53L1X_RANGE_STATUS_COMPLETE                         9u

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


/* ------------------------------------------------------------------------- */
/* Low-level register access via write_burst/read_burst                      */
/* ------------------------------------------------------------------------- */

static bool vl53l1x_write8(uint16_t reg, uint8_t value)
{
    uint8_t buf[3];

    buf[0] = (uint8_t)(reg >> 8);
    buf[1] = (uint8_t)reg;
    buf[2] = value;

    return write_burst(VL53L1X_ADDR, buf, sizeof(buf));
}

static bool vl53l1x_write16(uint16_t reg, uint16_t value)
{
    uint8_t buf[4];

    buf[0] = (uint8_t)(reg >> 8);
    buf[1] = (uint8_t)reg;
    buf[2] = (uint8_t)(value >> 8);
    buf[3] = (uint8_t)value;

    return write_burst(VL53L1X_ADDR, buf, sizeof(buf));
}

static bool vl53l1x_write32(uint16_t reg, uint32_t value)
{
    uint8_t buf[6];

    buf[0] = (uint8_t)(reg >> 8);
    buf[1] = (uint8_t)reg;
    buf[2] = (uint8_t)(value >> 24);
    buf[3] = (uint8_t)(value >> 16);
    buf[4] = (uint8_t)(value >> 8);
    buf[5] = (uint8_t)value;

    return write_burst(VL53L1X_ADDR, buf, sizeof(buf));
}

/*
 * Register reads are done as:
 *   1) write_burst(addr, reg_hi, reg_lo)
 *   2) read_burst(addr, data...)
 *
 * This is two I2C transactions, not a repeated-start combined transaction.
 * That matches the behavior of the original Arduino code.
 */
static bool vl53l1x_read_multi(uint16_t reg, uint8_t *data, size_t len)
{
    uint8_t reg_buf[2];

    if ((len > 0u) && (data == NULL)) {
        return false;
    }

    reg_buf[0] = (uint8_t)(reg >> 8);
    reg_buf[1] = (uint8_t)reg;

    if (!write_burst(VL53L1X_ADDR, reg_buf, sizeof(reg_buf))) {
        return false;
    }

    return read_burst(VL53L1X_ADDR, data, len);
}

static bool vl53l1x_read8(uint16_t reg, uint8_t *value)
{
    return vl53l1x_read_multi(reg, value, 1u);
}

static bool vl53l1x_read16(uint16_t reg, uint16_t *value)
{
    uint8_t buf[2];

    if (value == NULL) {
        return false;
    }

    if (!vl53l1x_read_multi(reg, buf, sizeof(buf))) {
        return false;
    }

    *value = ((uint16_t)buf[0] << 8) | buf[1];
    return true;
}


/* ------------------------------------------------------------------------- */
/* Software arithmetic helpers for RV32I (no MUL/DIV instructions required)  */
/* ------------------------------------------------------------------------- */

static uint32_t vl53l1x_mul_u32_soft(uint32_t a, uint32_t b)
{
    uint32_t r = 0u;

    while (b != 0u) {
        if ((b & 1u) != 0u) {
            r += a;
        }

        a <<= 1;
        b >>= 1;
    }

    return r;
}

static uint32_t vl53l1x_divmod_u32_soft(uint32_t num,
                                        uint32_t den,
                                        uint32_t *rem_out)
{
    uint32_t q = 0u;
    uint32_t bit = 1u;

    if (den == 0u) {
        if (rem_out != NULL) {
            *rem_out = 0u;
        }
        return 0u;
    }

    while ((den < num) && ((den & 0x80000000u) == 0u)) {
        den <<= 1;
        bit <<= 1;
    }

    while (bit != 0u) {
        if (num >= den) {
            num -= den;
            q |= bit;
        }

        den >>= 1;
        bit >>= 1;
    }

    if (rem_out != NULL) {
        *rem_out = num;
    }

    return q;
}

static uint32_t vl53l1x_div_shift12_round_u32(uint32_t num, uint32_t den)
{
    uint32_t q;
    uint32_t rem;
    uint32_t frac = 0u;
    uint32_t half_floor;
    uint32_t half_ceil;
    uint8_t i;

    if (den == 0u) {
        return 0u;
    }

    q = vl53l1x_divmod_u32_soft(num, den, &rem);

    if (q > 0x000FFFFFu) {
        return 0xFFFFFFFFu;
    }

    q <<= 12;

    half_floor = den >> 1;
    half_ceil = half_floor + (den & 1u);

    for (i = 0u; i < 12u; ++i) {
        frac <<= 1;

        if (rem >= half_ceil) {
            frac |= 1u;

            if ((den & 1u) == 0u) {
                rem = (rem - half_floor) << 1;
            } else {
                rem = ((rem - half_ceil) << 1) | 1u;
            }
        } else {
            rem <<= 1;
        }
    }

    q += frac;

    if (rem >= half_ceil) {
        if (q != 0xFFFFFFFFu) {
            q++;
        }
    }

    return q;
}

static uint32_t vl53l1x_mul_u32_2011(uint32_t x)
{
    return (x << 10) + (x << 9) + (x << 8) + (x << 7) +
           (x << 6) + (x << 4) + (x << 3) + (x << 1) + x;
}


/* ------------------------------------------------------------------------- */
/* Timing helpers                                                            */
/* ------------------------------------------------------------------------- */

static uint32_t vl53l1x_timeout_us_to_mclks(uint32_t timeout_us,
                                            uint32_t macro_period_us)
{
    return vl53l1x_div_shift12_round_u32(timeout_us, macro_period_us);
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

    pll_period_us = vl53l1x_divmod_u32_soft(((uint32_t)1u << 30),
                                            fast_osc_frequency,
                                            NULL);

    vcsel_period_pclks = (uint8_t)((vcsel_period + 1u) << 1);

    macro_period_us = (pll_period_us << 11) + (pll_period_us << 8);
    macro_period_us >>= 6;
    macro_period_us = vl53l1x_mul_u32_soft(macro_period_us,
                                           (uint32_t)vcsel_period_pclks);
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

    range_config_timeout_us >>= 1;

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


/* ------------------------------------------------------------------------- */
/* Sensor helpers                                                            */
/* ------------------------------------------------------------------------- */

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

static bool vl53l1x_data_ready(bool *ready)
{
    uint8_t gpio_status;

    if (ready == NULL) {
        return false;
    }

    if (!vl53l1x_read8(REG_GPIO__TIO_HV_STATUS, &gpio_status)) {
        return false;
    }

    *ready = ((gpio_status & 0x01u) == 0u);
    return true;
}

static bool vl53l1x_read_results(struct vl53l1x_results *r)
{
    uint8_t b[VL53L1X_RESULT_BUF_LEN];

    if (r == NULL) {
        return false;
    }

    if (!vl53l1x_read_multi(REG_RESULT__RANGE_STATUS, b, sizeof(b))) {
        return false;
    }

    r->range_status = b[0];
    r->stream_count = b[2];

    r->dss_actual_effective_spads_sd0 =
        ((uint16_t)b[3] << 8) | b[4];

    r->ambient_count_rate_mcps_sd0 =
        ((uint16_t)b[7] << 8) | b[8];

    r->final_crosstalk_corrected_range_mm_sd0 =
        ((uint16_t)b[13] << 8) | b[14];

    r->peak_signal_count_rate_crosstalk_corrected_mcps_sd0 =
        ((uint16_t)b[15] << 8) | b[16];

    return true;
}

static bool vl53l1x_setup_manual_calibration(void)
{
    uint8_t phasecal_vcsel_start;

    if (!vl53l1x_read8(REG_VHV_CONFIG__INIT, &saved_vhv_init)) {
        return false;
    }

    if (!vl53l1x_read8(REG_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,
                       &saved_vhv_timeout)) {
        return false;
    }

    if (!vl53l1x_write8(REG_VHV_CONFIG__INIT,
                        (uint8_t)(saved_vhv_init & 0x7Fu))) {
        return false;
    }

    if (!vl53l1x_write8(
            REG_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,
            (uint8_t)((saved_vhv_timeout & 0x03u) + (3u << 2)))) {
        return false;
    }

    if (!vl53l1x_write8(REG_PHASECAL_CONFIG__OVERRIDE, 0x01u)) {
        return false;
    }

    if (!vl53l1x_read8(REG_PHASECAL_RESULT__VCSEL_START,
                       &phasecal_vcsel_start)) {
        return false;
    }

    if (!vl53l1x_write8(REG_CAL_CONFIG__VCSEL_START,
                        phasecal_vcsel_start)) {
        return false;
    }

    return true;
}

static bool vl53l1x_update_dss(const struct vl53l1x_results *r)
{
    uint16_t spad_count;

    if (r == NULL) {
        return false;
    }

    spad_count = r->dss_actual_effective_spads_sd0;

    if (spad_count != 0u) {
        uint32_t total_rate_per_spad =
            (uint32_t)r->peak_signal_count_rate_crosstalk_corrected_mcps_sd0 +
            r->ambient_count_rate_mcps_sd0;

        if (total_rate_per_spad > 0xFFFFu) {
            total_rate_per_spad = 0xFFFFu;
        }

        total_rate_per_spad =
            vl53l1x_divmod_u32_soft(total_rate_per_spad << 16,
                                    spad_count,
                                    NULL);

        if (total_rate_per_spad != 0u) {
            uint32_t required_spads =
                vl53l1x_divmod_u32_soft(((uint32_t)VL53L1X_TARGET_RATE << 16),
                                        total_rate_per_spad,
                                        NULL);

            if (required_spads > 0xFFFFu) {
                required_spads = 0xFFFFu;
            }

            return vl53l1x_write16(
                REG_DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT,
                (uint16_t)required_spads);
        }
    }

    return vl53l1x_write16(REG_DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT,
                           0x8000u);
}


/* ------------------------------------------------------------------------- */
/* Public API                                                                */
/* ------------------------------------------------------------------------- */

bool vl53l1x_init(void)
{
    uint8_t tmp8;
    uint16_t tmp16;

    calibrated = false;
    saved_vhv_init = 0u;
    saved_vhv_timeout = 0u;
    fast_osc_frequency = 0u;
    osc_calibrate_val = 0u;

    if (!vl53l1x_read16(REG_IDENTIFICATION__MODEL_ID, &tmp16)) {
        return false;
    }

    if (tmp16 != 0xEACCu) {
        return false;
    }

    if (!vl53l1x_write8(REG_SOFT_RESET, 0x00u)) {
        return false;
    }

    VL53L1X_DELAY_US(100u);

    if (!vl53l1x_write8(REG_SOFT_RESET, 0x01u)) {
        return false;
    }

    VL53L1X_DELAY_US(1000u);

    if (!vl53l1x_wait_boot()) {
        return false;
    }

#if VL53L1X_USE_2V8
    if (!vl53l1x_read8(REG_PAD_I2C_HV__EXTSUP_CONFIG, &tmp8)) {
        return false;
    }

    if (!vl53l1x_write8(REG_PAD_I2C_HV__EXTSUP_CONFIG,
                        (uint8_t)(tmp8 | 0x01u))) {
        return false;
    }
#endif

    if (!vl53l1x_read16(REG_OSC_MEASURED__FAST_OSC__FREQUENCY,
                        &fast_osc_frequency)) {
        return false;
    }

    if (!vl53l1x_read16(REG_RESULT__OSC_CALIBRATE_VAL,
                        &osc_calibrate_val)) {
        return false;
    }

    if ((fast_osc_frequency == 0u) || (osc_calibrate_val == 0u)) {
        return false;
    }

    if (!vl53l1x_write16(REG_DSS_CONFIG__TARGET_TOTAL_RATE_MCPS,
                         VL53L1X_TARGET_RATE)) {
        return false;
    }

    if (!vl53l1x_write8(REG_GPIO__TIO_HV_STATUS, 0x02u)) {
        return false;
    }

    if (!vl53l1x_write8(REG_SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS, 8u)) {
        return false;
    }

    if (!vl53l1x_write8(REG_SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS, 16u)) {
        return false;
    }

    if (!vl53l1x_write8(REG_ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM,
                        0x01u)) {
        return false;
    }

    if (!vl53l1x_write8(REG_ALGO__RANGE_IGNORE_VALID_HEIGHT_MM, 0xFFu)) {
        return false;
    }

    if (!vl53l1x_write8(REG_ALGO__RANGE_MIN_CLIP, 0u)) {
        return false;
    }

    if (!vl53l1x_write8(REG_ALGO__CONSISTENCY_CHECK__TOLERANCE, 2u)) {
        return false;
    }

    if (!vl53l1x_write16(REG_SYSTEM__THRESH_RATE_HIGH, 0x0000u)) {
        return false;
    }

    if (!vl53l1x_write16(REG_SYSTEM__THRESH_RATE_LOW, 0x0000u)) {
        return false;
    }

    if (!vl53l1x_write8(REG_DSS_CONFIG__APERTURE_ATTENUATION, 0x38u)) {
        return false;
    }

    if (!vl53l1x_write16(REG_RANGE_CONFIG__SIGMA_THRESH, 360u)) {
        return false;
    }

    if (!vl53l1x_write16(REG_RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS,
                         192u)) {
        return false;
    }

    if (!vl53l1x_write8(REG_SYSTEM__GROUPED_PARAMETER_HOLD_0, 0x01u)) {
        return false;
    }

    if (!vl53l1x_write8(REG_SYSTEM__GROUPED_PARAMETER_HOLD_1, 0x01u)) {
        return false;
    }

    if (!vl53l1x_write8(REG_SD_CONFIG__QUANTIFIER, 2u)) {
        return false;
    }

    if (!vl53l1x_write8(REG_SYSTEM__GROUPED_PARAMETER_HOLD, 0x00u)) {
        return false;
    }

    if (!vl53l1x_write8(REG_SYSTEM__SEED_CONFIG, 1u)) {
        return false;
    }

    if (!vl53l1x_write8(REG_SYSTEM__SEQUENCE_CONFIG, 0x8Bu)) {
        return false;
    }

    if (!vl53l1x_write16(REG_DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT,
                         (uint16_t)(200u << 8))) {
        return false;
    }

    if (!vl53l1x_write8(REG_DSS_CONFIG__ROI_MODE_CONTROL, 2u)) {
        return false;
    }

    if (!vl53l1x_set_long_distance_mode()) {
        return false;
    }

    if (!vl53l1x_read16(REG_MM_CONFIG__OUTER_OFFSET_MM, &tmp16)) {
        return false;
    }

    if (!vl53l1x_write16(REG_ALGO__PART_TO_PART_RANGE_OFFSET_MM,
                         (uint16_t)((uint32_t)tmp16 << 2))) {
        return false;
    }

    if (!vl53l1x_write32(
            REG_SYSTEM__INTERMEASUREMENT_PERIOD,
            vl53l1x_mul_u32_soft((uint32_t)VL53L1X_INTERMEASUREMENT_MS,
                                 (uint32_t)osc_calibrate_val))) {
        return false;
    }

    if (!vl53l1x_write8(REG_SYSTEM__INTERRUPT_CLEAR, 0x01u)) {
        return false;
    }

    if (!vl53l1x_write8(REG_SYSTEM__MODE_START, 0x40u)) {
        return false;
    }

    return true;
}

vl53l1x_poll_result_t vl53l1x_poll(uint16_t *range_mm)
{
    bool ready;
    struct vl53l1x_results r;
    uint32_t corrected_range;

    if (range_mm == NULL) {
        return VL53L1X_POLL_ERROR;
    }

    if (!vl53l1x_data_ready(&ready)) {
        return VL53L1X_POLL_ERROR;
    }

    if (!ready) {
        return VL53L1X_POLL_NONE;
    }

    if (!vl53l1x_read_results(&r)) {
        return VL53L1X_POLL_ERROR;
    }

    if (!calibrated) {
        if (!vl53l1x_setup_manual_calibration()) {
            return VL53L1X_POLL_ERROR;
        }

        calibrated = true;
    }

    if (!vl53l1x_update_dss(&r)) {
        return VL53L1X_POLL_ERROR;
    }

    corrected_range = vl53l1x_mul_u32_2011(
                          (uint32_t)r.final_crosstalk_corrected_range_mm_sd0)
                      + 0x0400u;
    corrected_range >>= 11;

    if (corrected_range > 0xFFFFu) {
        corrected_range = 0xFFFFu;
    }

    *range_mm = (uint16_t)corrected_range;

    if (!vl53l1x_write8(REG_SYSTEM__INTERRUPT_CLEAR, 0x01u)) {
        return VL53L1X_POLL_ERROR;
    }

    if (r.range_status == VL53L1X_RANGE_STATUS_COMPLETE) {
        return VL53L1X_POLL_OK;
    }

    return VL53L1X_POLL_INVALID;
}
