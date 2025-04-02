#pragma once

#include <stdint.h>

typedef enum { ST_OK, ST_FAIL } STAT;

STAT i2c_init();
STAT i2c_write(uint8_t addr, uint8_t* buffer, uint16_t len);
STAT i2c_deinit();