#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include <stdbool.h>

/* Уровень 1: одна функция = одна команда I2C Master Core */
bool i2c_start(void);
bool i2c_restart(void);
bool i2c_stop(void);
bool i2c_write_byte(uint8_t byte);
bool i2c_read_byte(uint8_t* byte, bool send_nack);

/* Уровень 2: блокирующие транзакции */
bool i2c_write(uint8_t addr, uint8_t data);
bool i2c_read(uint8_t addr, uint8_t* data);

#endif /* I2C_H */
