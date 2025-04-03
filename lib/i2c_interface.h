#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { ST_OK, ST_FAIL } STAT;

STAT i2c_init();
STAT i2c_write(uint8_t addr, uint8_t* buffer, unsigned len);
STAT i2c_deinit();

#ifdef __cplusplus
}
#endif
