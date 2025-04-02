#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <stdarg.h>
#include <malloc.h>
#include "i2c_interface.h"
#include "mcp2221_dll_um.h"

#define DEFAULT_VID                     0x04D8
#define DEFAULT_PID                     0x00DD
#define SSD1306_COMMAND           0x80  // Continuation bit=1, D/C=0; 1000 0000

void* mcpHandle = NULL;
unsigned char use7bitAddress = 1;


/**
 * @brief  interface iic bus init
 * @return status code
 *         - 0 success
 *         - 1 iic init failed
 * @note   none
 */
STAT i2c_init()
{
    wchar_t libVersion[64];
    unsigned int numberOfDevices = 0;
    int result = 1;

    if ((result = Mcp2221_GetLibraryVersion(libVersion)) != E_NO_ERR)
        return ST_FAIL;
    if ((result = Mcp2221_GetConnectedDevices(DEFAULT_VID, DEFAULT_PID, &numberOfDevices)) != E_NO_ERR)
        return ST_FAIL;
    mcpHandle = Mcp2221_OpenByIndex(DEFAULT_VID, DEFAULT_PID, 0);
    if ((result = Mcp2221_GetLastError()) != ST_OK)
        return ST_FAIL;
    if ((result = Mcp2221_SetSpeed(mcpHandle, 100000)) != E_NO_ERR)
        return ST_OK;
    return ST_OK;
}

/**
 * @brief  interface iic bus deinit
 * @return status code
 *         - 0 success
 *         - 1 iic deinit failed
 * @note   none
 */
STAT i2c_deinit()
{
    int result;

    if ((result = Mcp2221_Close(mcpHandle)) != E_NO_ERR)
        return ST_FAIL;
    return S_OK;
}

/**
 * @brief     interface iic bus write
 * @param[in] addr iic device write address
 * @param[in] reg iic register address
 * @param[in] *buf pointer to a data buffer
 * @param[in] len length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
STAT i2c_write(uint8_t addr, uint8_t *buf, uint16_t len)
{
    int result = 1;
    result = Mcp2221_I2cWrite(mcpHandle, len, addr, use7bitAddress, buf);
    
    return (result == E_NO_ERR ? ST_OK : ST_FAIL);
}
