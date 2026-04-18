#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/*
 * Blocking I2C transactions.
 *
 * addr is always a 7-bit I2C address, not shifted.
 */

/* Raw byte transactions: no register byte is sent. */
bool i2c_probe(uint8_t addr);
bool i2c_write(uint8_t addr, uint8_t data);
bool i2c_read(uint8_t addr, uint8_t* data);

/* Raw burst transactions: no register byte is sent. */
bool i2c_write_burst(uint8_t addr, const uint8_t* data, size_t len);
bool i2c_read_burst(uint8_t addr, uint8_t* data, size_t len);

/* Backward-compatible names from the original API. */
bool write_burst(uint8_t addr, const uint8_t* data, size_t len);
bool read_burst(uint8_t addr, uint8_t* data, size_t len);

/*
 * 8-bit register transactions.
 *
 * Write format:
 *   START, addr+W, reg, data..., STOP
 *
 * Read format:
 *   START, addr+W, reg, RESTART, addr+R, data..., STOP
 */
bool i2c_write_reg8(uint8_t addr, uint8_t reg, uint8_t data);
bool i2c_read_reg8(uint8_t addr, uint8_t reg, uint8_t* data);
bool i2c_write_reg8_burst(uint8_t addr,
                          uint8_t reg,
                          const uint8_t* data,
                          size_t len);
bool i2c_read_reg8_burst(uint8_t addr,
                         uint8_t reg,
                         uint8_t* data,
                         size_t len);

#endif /* I2C_H */
