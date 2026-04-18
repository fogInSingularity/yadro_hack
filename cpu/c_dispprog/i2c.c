#include "i2c.h"

/* MMIO registers */
static volatile uint8_t* const cmd_valid_i      = (uint8_t*)0x24;
static volatile uint8_t* const cmd_i            = (uint8_t*)0x28;
static volatile uint8_t* const din_i            = (uint8_t*)0x2C;
static volatile uint8_t* const dout_o           = (uint8_t*)0x30;
static volatile uint8_t* const rx_ack_o         = (uint8_t*)0x34;
static volatile uint8_t* const ready_o          = (uint8_t*)0x38;
static volatile uint8_t* const arb_lost_o       = (uint8_t*)0x3C;
static volatile uint8_t* const arb_lost_clear_i = (uint8_t*)0x40;
static volatile uint8_t* const busy_o           = (uint8_t*)0x44;

/* Core commands */
#define I2C_CMD_START    1u
#define I2C_CMD_WRITE    2u
#define I2C_CMD_READ     3u
#define I2C_CMD_STOP     4u
#define I2C_CMD_RESTART  5u

#ifndef I2C_TIMEOUT_ITER
#define I2C_TIMEOUT_ITER 1000000u
#endif

static bool i2c_wait_ready(void)
{
    uint32_t timeout = I2C_TIMEOUT_ITER;

    while (((*ready_o) & 1u) == 0u) {
        if (((*arb_lost_o) & 1u) != 0u) {
            return false;
        }

        if (--timeout == 0u) {
            return false;
        }
    }

    return true;
}

static bool i2c_wait_not_busy(void)
{
    uint32_t timeout = I2C_TIMEOUT_ITER;

    while (((*busy_o) & 1u) != 0u) {
        if (((*arb_lost_o) & 1u) != 0u) {
            return false;
        }

        if (--timeout == 0u) {
            return false;
        }
    }

    return true;
}

static bool i2c_wait_arb_lost_clear(void)
{
    uint32_t timeout = I2C_TIMEOUT_ITER;

    while (((*arb_lost_o) & 1u) != 0u) {
        if (--timeout == 0u) {
            return false;
        }
    }

    return true;
}

static bool i2c_clear_arb_lost(void)
{
    *arb_lost_clear_i = 1u;
    (void)*arb_lost_o;
    *arb_lost_clear_i = 0u;

    return i2c_wait_arb_lost_clear();
}

static bool i2c_exec_cmd(uint8_t cmd, uint8_t din)
{
    uint32_t timeout;

    *cmd_valid_i = 0u;

    if (((*arb_lost_o) & 1u) != 0u) {
        return false;
    }

    if (!i2c_wait_ready()) {
        return false;
    }

    *din_i = din;
    *cmd_i = cmd;
    *cmd_valid_i = 1u;

    timeout = I2C_TIMEOUT_ITER;

    while (((*ready_o) & 1u) != 0u) {
        if (((*arb_lost_o) & 1u) != 0u) {
            *cmd_valid_i = 0u;
            return false;
        }

        if (--timeout == 0u) {
            *cmd_valid_i = 0u;
            return false;
        }
    }

    *cmd_valid_i = 0u;

    if (!i2c_wait_ready()) {
        return false;
    }

    if (((*arb_lost_o) & 1u) != 0u) {
        return false;
    }

    return true;
}

static bool i2c_fail_with_stop_if_possible(void)
{
    *cmd_valid_i = 0u;

    /*
     * If arbitration was lost, the core has already released the bus and
     * will not accept new commands until the flag is cleared.
     */
    if (((*arb_lost_o) & 1u) == 0u && ((*ready_o) & 1u) != 0u) {
        (void)i2c_stop();
    }

    return false;
}

static bool i2c_prepare_transaction(void)
{
    *cmd_valid_i = 0u;

    if (((*arb_lost_o) & 1u) != 0u) {
        if (!i2c_clear_arb_lost()) {
            return false;
        }
    }

    if (!i2c_wait_ready()) {
        return false;
    }

    if (!i2c_wait_not_busy()) {
        return false;
    }

    return true;
}

/* =========================================================================
 * Level 1: private byte-level primitives
 * ========================================================================= */

bool i2c_start(void)
{
    return i2c_exec_cmd(I2C_CMD_START, 0u);
}

bool i2c_restart(void)
{
    return i2c_exec_cmd(I2C_CMD_RESTART, 0u);
}

bool i2c_stop(void)
{
    return i2c_exec_cmd(I2C_CMD_STOP, 0u);
}

bool i2c_write_byte(uint8_t byte)
{
    if (!i2c_exec_cmd(I2C_CMD_WRITE, byte)) {
        return false;
    }

    /* rx_ack_o: 0 = ACK, 1 = NACK */
    return (((*rx_ack_o) & 1u) == 0u);
}

bool i2c_read_byte(uint8_t* byte, bool send_nack)
{
    if (byte == NULL) {
        return false;
    }

    /*
     * For READ:
     * din_i[0] = 0 -> ACK
     * din_i[0] = 1 -> NACK
     */
    if (!i2c_exec_cmd(I2C_CMD_READ, send_nack ? 1u : 0u)) {
        return false;
    }

    *byte = *dout_o;
    return true;
}

/* =========================================================================
 * Level 2: blocking public transactions
 * ========================================================================= */

bool i2c_probe(uint8_t addr)
{
    uint8_t addr_w = (uint8_t)((addr & 0x7Fu) << 1);

    if (!i2c_prepare_transaction()) {
        return false;
    }

    if (!i2c_start()) {
        return false;
    }

    if (!i2c_write_byte(addr_w)) {
        return i2c_fail_with_stop_if_possible();
    }

    if (!i2c_stop()) {
        return false;
    }

    return true;
}

bool i2c_write(uint8_t addr, uint8_t data)
{
    return i2c_write_burst(addr, &data, 1u);
}

bool i2c_read(uint8_t addr, uint8_t* data)
{
    return i2c_read_burst(addr, data, 1u);
}

bool i2c_write_burst(uint8_t addr, const uint8_t* data, size_t len)
{
    uint8_t addr_w = (uint8_t)((addr & 0x7Fu) << 1);
    size_t i;

    if (len > 0u && data == NULL) {
        return false;
    }

    /*
     * Zero length is a successful no-op.
     * The bus is not touched.
     */
    if (len == 0u) {
        return true;
    }

    if (!i2c_prepare_transaction()) {
        return false;
    }

    if (!i2c_start()) {
        return false;
    }

    if (!i2c_write_byte(addr_w)) {
        return i2c_fail_with_stop_if_possible();
    }

    for (i = 0u; i < len; ++i) {
        if (!i2c_write_byte(data[i])) {
            return i2c_fail_with_stop_if_possible();
        }
    }

    if (!i2c_stop()) {
        return false;
    }

    return true;
}

bool i2c_read_burst(uint8_t addr, uint8_t* data, size_t len)
{
    uint8_t addr_r = (uint8_t)(((addr & 0x7Fu) << 1) | 1u);
    size_t i;

    if (len > 0u && data == NULL) {
        return false;
    }

    /*
     * Zero length is a successful no-op.
     * The bus is not touched.
     */
    if (len == 0u) {
        return true;
    }

    if (!i2c_prepare_transaction()) {
        return false;
    }

    if (!i2c_start()) {
        return false;
    }

    if (!i2c_write_byte(addr_r)) {
        return i2c_fail_with_stop_if_possible();
    }

    for (i = 0u; i < len; ++i) {
        bool last = (i == (len - 1u));

        if (!i2c_read_byte(&data[i], last)) {
            return i2c_fail_with_stop_if_possible();
        }
    }

    if (!i2c_stop()) {
        return false;
    }

    return true;
}

bool write_burst(uint8_t addr, const uint8_t* data, size_t len)
{
    return i2c_write_burst(addr, data, len);
}

bool read_burst(uint8_t addr, uint8_t* data, size_t len)
{
    return i2c_read_burst(addr, data, len);
}

bool i2c_write_reg8(uint8_t addr, uint8_t reg, uint8_t data)
{
    return i2c_write_reg8_burst(addr, reg, &data, 1u);
}

bool i2c_read_reg8(uint8_t addr, uint8_t reg, uint8_t* data)
{
    return i2c_read_reg8_burst(addr, reg, data, 1u);
}

bool i2c_write_reg8_burst(uint8_t addr,
                          uint8_t reg,
                          const uint8_t* data,
                          size_t len)
{
    uint8_t addr_w = (uint8_t)((addr & 0x7Fu) << 1);
    size_t i;

    if (len > 0u && data == NULL) {
        return false;
    }

    /*
     * len == 0 writes only the register byte. This is useful for devices
     * where a register-pointer write is a valid operation.
     */
    if (!i2c_prepare_transaction()) {
        return false;
    }

    if (!i2c_start()) {
        return false;
    }

    if (!i2c_write_byte(addr_w)) {
        return i2c_fail_with_stop_if_possible();
    }

    if (!i2c_write_byte(reg)) {
        return i2c_fail_with_stop_if_possible();
    }

    for (i = 0u; i < len; ++i) {
        if (!i2c_write_byte(data[i])) {
            return i2c_fail_with_stop_if_possible();
        }
    }

    if (!i2c_stop()) {
        return false;
    }

    return true;
}

bool i2c_read_reg8_burst(uint8_t addr,
                         uint8_t reg,
                         uint8_t* data,
                         size_t len)
{
    uint8_t addr_w = (uint8_t)((addr & 0x7Fu) << 1);
    uint8_t addr_r = (uint8_t)(((addr & 0x7Fu) << 1) | 1u);
    size_t i;

    if (len > 0u && data == NULL) {
        return false;
    }

    /*
     * Zero length is a successful no-op.
     * The bus is not touched.
     */
    if (len == 0u) {
        return true;
    }

    if (!i2c_prepare_transaction()) {
        return false;
    }

    if (!i2c_start()) {
        return false;
    }

    if (!i2c_write_byte(addr_w)) {
        return i2c_fail_with_stop_if_possible();
    }

    if (!i2c_write_byte(reg)) {
        return i2c_fail_with_stop_if_possible();
    }

    if (!i2c_restart()) {
        return i2c_fail_with_stop_if_possible();
    }

    if (!i2c_write_byte(addr_r)) {
        return i2c_fail_with_stop_if_possible();
    }

    for (i = 0u; i < len; ++i) {
        bool last = (i == (len - 1u));

        if (!i2c_read_byte(&data[i], last)) {
            return i2c_fail_with_stop_if_possible();
        }
    }

    if (!i2c_stop()) {
        return false;
    }

    return true;
}
