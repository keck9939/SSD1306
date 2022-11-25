/** 
 * --------------------------------------------------------------------------------------+  
 * @desc        SSD1306 OLED Driver
 * --------------------------------------------------------------------------------------+ 
 *              Copyright (C) 2020 Marian Hrinko.
 *              Written by Marian Hrinko (mato.hrinko@gmail.com)
 *
 * @author      Marian Hrinko
 * @datum       06.10.2020
 * @update      24.11.2022
 * @file        ssd1306.c
 * @version     3.0
 * @tested      AVR Atmega328p
 *
 * @depend      font.h, twi.h
 * --------------------------------------------------------------------------------------+ 
 * @descr       Version 1.0 -> applicable for 1 display
 *              Version 2.0 -> rebuild to 'cacheMemLcd' array
 *              Version 3.0 -> remove 'cacheMemLcd' approach
 * --------------------------------------------------------------------------------------+ 
 * @usage       Basic Setup for OLED Display
 */
 
#include "ssd1306.h"

// +---------------------------+
// |      Set MUX Ratio        |
// +---------------------------+
// |        0xA8, 0x3F         |
// +---------------------------+
//              |
// +---------------------------+
// |    Set Display Offset     |
// +---------------------------+
// |        0xD3, 0x00         |
// +---------------------------+
//              |
// +---------------------------+
// |  Set Display Start Line   |
// +---------------------------+
// |          0x40             |
// +---------------------------+
//              |
// +---------------------------+
// |     Set Segment Remap     |
// +---------------------------+
// |       0xA0 / 0xA1         |
// +---------------------------+
//              |
// +---------------------------+
// |   Set COM Output Scan     |
// |        Direction          |
// +---------------------------+
// |       0xC0 / 0xC8         |
// +---------------------------+
//              |
// +---------------------------+
// |   Set COM Pins hardware   |
// |       configuration       |
// +---------------------------+
// |        0xDA, 0x02         |
// +---------------------------+
//              |
// +---------------------------+
// |   Set Contrast Control    |
// +---------------------------+
// |        0x81, 0x7F         |
// +---------------------------+
//              |
// +---------------------------+
// | Disable Entire Display On |
// +---------------------------+
// |          0xA4             |
// +---------------------------+
//              |
// +---------------------------+
// |    Set Normal Display     |
// +---------------------------+
// |          0xA6             |
// +---------------------------+
//              |
// +---------------------------+
// |    Set Osc Frequency      |
// +---------------------------+
// |       0xD5, 0x80          |
// +---------------------------+
//              |
// +---------------------------+
// |    Enable charge pump     |
// |        regulator          |
// +---------------------------+
// |       0x8D, 0x14          |
// +---------------------------+
//              |
// +---------------------------+
// |        Display On         |
// +---------------------------+
// |          0xAF             |
// +---------------------------+

// @array Init command
const uint8_t INIT_SSD1306[] PROGMEM = {
  18,                                                             // number of initializers
  0, SSD1306_DISPLAY_OFF,                                         // 0xAE = Set Display OFF
  1, SSD1306_SET_MUX_RATIO, 0x3F,                                 // 0xA8 - 64MUX
  1, SSD1306_MEMORY_ADDR_MODE, 0x00,                              // 0x20 = Set Memory Addressing Mode
                                                                  // 0x00 - Horizontal Addressing Mode
                                                                  // 0x01 - Vertical Addressing Mode
                                                                  // 0x02 - Page Addressing Mode (RESET)
  2, SSD1306_SET_COLUMN_ADDR, START_COLUMN_ADDR, END_COLUMN_ADDR, // 0x21 = Set Column Address, 0 - 127
  2, SSD1306_SET_PAGE_ADDR, START_PAGE_ADDR, END_PAGE_ADDR,       // 0x22 = Set Page Address, 0 - 7
  0, SSD1306_SET_START_LINE,                                      // 0x40
  1, SSD1306_DISPLAY_OFFSET, 0x00,                                // 0xD3
  0, SSD1306_SEG_REMAP_OP,                                        // 0xA0 / remap 0xA1
  0, SSD1306_COM_SCAN_DIR_OP,                                     // 0xC0 / remap 0xC8
  1, SSD1306_COM_PIN_CONF, 0x12,                                  // 0xDA, 0x12 - Disable COM Left/Right remap, Alternative COM pin configuration
  1, SSD1306_SET_CONTRAST, 0x7F,                                  // 0x81, 0x7F - reset value (max 0xFF)
  0, SSD1306_DIS_ENT_DISP_ON,                                     // 0xA4
  0, SSD1306_DIS_NORMAL,                                          // 0xA6
  1, SSD1306_SET_OSC_FREQ, 0x80,                                  // 0xD5, 0x80 => D=1; DCLK = Fosc / D <=> DCLK = Fosc
  1, SSD1306_SET_PRECHARGE, 0xc2,                                 // 0xD9, higher value less blinking
                                                                  // 0xC2, 1st phase = 2 DCLK,  2nd phase = 13 DCLK
  1, SSD1306_VCOM_DESELECT, 0x20,                                 // Set V COMH Deselect, reset value 0x22 = 0,77xUcc
  1, SSD1306_SET_CHAR_REG, 0x14,                                  // 0x8D, Enable charge pump during display on
  0, SSD1306_DISPLAY_ON                                           // 0xAF = Set Display ON
};

unsigned short int _indexCol = START_COLUMN_ADDR;                 // @var global - cache index column
unsigned short int _indexPage = START_PAGE_ADDR;                  // @var global - cache index page

/**
 * @desc    SSD1306 Init
 *
 * @param   void
 *
 * @return  uint8_t
 */
uint8_t SSD1306_Init (void)
{ 
  const uint8_t *commands = INIT_SSD1306;                         // variables
  unsigned short int no_of_commands = pgm_read_byte(commands++);  // number of commands
  uint8_t no_of_arguments;                                        // number od arguments
  uint8_t command;                                                // command
  uint8_t status = INIT_STATUS;                                   // TWI init status 0xFF

  // TWI INIT
  // -------------------------------------------------------------------------------------
  TWI_Init ();

  // TWI Start & SLAW
  // -------------------------------------------------------------------------------------
  status = SSD1306_Send_StartAndSLAW (SSD1306_ADDR);              // start & SLAW
  if (SSD1306_SUCCESS != status) {                                // check status
    return status;                                                // error
  }
  // MAIN LOOP
  // -------------------------------------------------------------------------------------
  while (no_of_commands) {                                        // commands loop
    no_of_arguments = pgm_read_byte (commands++);                 // number of arguments
    command = pgm_read_byte (commands++);                         // command
    // SEND COMMAND
    // -----------------------------------------------------------------------------------
    status = SSD1306_Send_Command (command);                      // send command
    if (SSD1306_SUCCESS != status) {                              // check status
      return status;                                              // error
    }
    // SEND ARGUMENTS
    // -----------------------------------------------------------------------------------
    while (no_of_arguments--) {
      status = SSD1306_Send_Command (pgm_read_byte(commands++));  // send argument
      if (SSD1306_SUCCESS != status) {                            // check status
        return status;                                            // error
      }
    }

    no_of_commands--;                                             // next command
  }
  // TWI STOP
  // -------------------------------------------------------------------------------------
  TWI_Stop ();

  return SSD1306_SUCCESS;                                         // success
}

/**
 * @desc    SSD1306 Send Start and SLAW request
 *
 * @param   uint8_t
 *
 * @return  uint8_t
 */
uint8_t SSD1306_Send_StartAndSLAW (uint8_t address)
{
  uint8_t status = INIT_STATUS;                                   // TWI init status 0xFF

  // TWI START
  // -------------------------------------------------------------------------------------
  status = TWI_MT_Start ();                                       // start
  if (SSD1306_SUCCESS != status) {                                // check status
    return status;                                                // error
  }
  // TWI start & SLAW
  // -------------------------------------------------------------------------------------
  status = TWI_MT_Send_SLAW (address);                            // start & SLAW
  if (SSD1306_SUCCESS != status) {                                // check status
    return status;                                                // error
  }
  
  return SSD1306_SUCCESS;                                         // success
}

/**
 * @desc    SSD1306 Send command
 *
 * @param   uint8_t command
 *
 * @return  uint8_t
 */
uint8_t SSD1306_Send_Command (uint8_t command)
{
  uint8_t status = INIT_STATUS;                                   // TWI init status 0xFF

  // TWI send control byte
  // -------------------------------------------------------------------------------------  
  status = TWI_MT_Send_Data (SSD1306_COMMAND);                    // send control byte
  if (SSD1306_SUCCESS != status) {                                // check status
    return status;                                                // error
  }
  // TWI send command
  // -------------------------------------------------------------------------------------
  status = TWI_MT_Send_Data (command);                            // send command
  if (SSD1306_SUCCESS != status) {                                // check status
    return status;                                                // error
  }

  return SSD1306_SUCCESS;                                         // success
}

/**
 * @desc    SSD1306 Normal colors
 *
 * @param   void
 *
 * @return  uint8_t
 */
uint8_t SSD1306_NormalScreen (void)
{
  uint8_t status = INIT_STATUS;                                   // TWI init status 0xFF

  // TWI start & SLAW
  // -------------------------------------------------------------------------------------
  status = SSD1306_Send_StartAndSLAW (SSD1306_ADDR);              // start & SLAW
  if (SSD1306_SUCCESS != status) {                                // check status
    return status;                                                // error
  }
  // TWI send command
  // -------------------------------------------------------------------------------------
  status = SSD1306_Send_Command (SSD1306_DIS_NORMAL);             // command 0xA6
  if (SSD1306_SUCCESS != status) {                                // check status
    return status;                                                // error
  }

  return SSD1306_SUCCESS;                                         // success
}

/**
 * @desc    SSD1306 Inverse colors
 *
 * @param   void
 *
 * @return  uint8_t
 */
uint8_t SSD1306_InverseScreen (void)
{
  uint8_t status = INIT_STATUS;                                   // TWI init status 0xFF

  // TWI start & SLAW
  // -------------------------------------------------------------------------------------
  status = SSD1306_Send_StartAndSLAW (SSD1306_ADDR);              // start & SLAW
  if (SSD1306_SUCCESS != status) {                                // check status
    return status;                                                // error
  }
  // TWI send command
  // -------------------------------------------------------------------------------------
  status = SSD1306_Send_Command (SSD1306_DIS_INVERSE);            // command 0xA7
  if (SSD1306_SUCCESS != status) {                                // check status
    return status;                                                // error
  }

  return SSD1306_SUCCESS;                                         // success
}

/**
 * @desc    SSD1306 Clear screen
 *
 * @param   void
 *
 * @return  uint8_t
 */
uint8_t SSD1306_ClearScreen (void)
{
  uint8_t status = INIT_STATUS;                                   // TWI init status 0xFF
  uint16_t i = 0;                                                 // counter

  // TWI start & SLAW
  // -------------------------------------------------------------------------------------
  status = SSD1306_Send_StartAndSLAW (SSD1306_ADDR);              // start & SLAW
  if (SSD1306_SUCCESS != status) {                                // check status
    return status;                                                // error
  }
  // TWI control byte data stream
  // -------------------------------------------------------------------------------------
  status = TWI_MT_Send_Data (SSD1306_DATA_STREAM);                // send data 0x40
  if (SSD1306_SUCCESS != status) {                                // check status
    return status;                                                // error
  }
  //  send clear byte to memory lcd
  // -------------------------------------------------------------------------------------
  while (i < CACHE_SIZE_MEM) {
    status = TWI_MT_Send_Data (CLEAR_COLOR);                      // send data 0x00
    if (SSD1306_SUCCESS != status) {                              // check status
      return status;                                              // error
    }
    i++;                                                          // update counter
  }
  // TWI stop
  // -------------------------------------------------------------------------------------
  TWI_Stop ();

  return SSD1306_SUCCESS;                                         // success
}

/**
 * @desc    SSD1306 Set position
 *
 * @param   uint8_t column -> 0 ... 127
 * @param   uint8_t page -> 0 ... 7
 *
 * @return  void
 */
uint8_t SSD1306_SetPosition (uint8_t x, uint8_t y) 
{
  uint8_t status = INIT_STATUS;                                   // TWI init status 0xFF

  // TWI start & SLAW
  // -------------------------------------------------------------------------------------
  status = SSD1306_Send_StartAndSLAW (SSD1306_ADDR);              // start & SLAW
  if (SSD1306_SUCCESS != status) {                                // check status
    return status;                                                // error
  }
  // COLUMN
  // ------------------------------------------------------------------------------------- 
  status = SSD1306_Send_Command (SSD1306_SET_COLUMN_ADDR);        // 0x21
  if (SSD1306_SUCCESS != status) {                                // check status
    return status;                                                // error
  }
  // start COLUMN
  status = SSD1306_Send_Command (x);
  if (SSD1306_SUCCESS != status) {                                // check status
    return status;                                                // error
  }
  // end COLUMN
  status = SSD1306_Send_Command (END_COLUMN_ADDR);                // 127
  if (SSD1306_SUCCESS != status) {                                // check status
    return status;                                                // error
  }
  // update column index
  _indexCol = x;

  // PAGE
  // -------------------------------------------------------------------------------------
  status = SSD1306_Send_Command (SSD1306_SET_PAGE_ADDR);          // 0x22
  if (SSD1306_SUCCESS != status) {                                // check status
    return status;                                                // error
  }
  // start PAGE
  status = SSD1306_Send_Command (y);
  if (SSD1306_SUCCESS != status) {                                // check status
    return status;                                                // error
  }
  // end PAGE
  status = SSD1306_Send_Command (END_PAGE_ADDR);                  // 7 for 128x64
  if (SSD1306_SUCCESS != status) {                                // check status
    return status;                                                // error
  }
  // update column index
  _indexPage = y;

  // TWI stop
  // -------------------------------------------------------------------------------------
  TWI_Stop ();

  return SSD1306_SUCCESS;                                         // success
}

/**
 * @desc    SSD1306 Update text poisition - this ensure that character will not be divided
 *          at the end of row, the whole character will be depicted on the new row
 *
 * @param   void
 *
 * @return  uint8_t
 */
uint8_t SSD1306_UpdatePosition (void) 
{
  uint8_t status = INIT_STATUS;                                   // TWI init status 0xFF
  uint8_t x = _indexCol + CHARS_COLS_LENGTH + 1;                  // check end col position

  // check position
  // -------------------------------------------------------------------------------------
  if (x > END_COLUMN_ADDR) {
    // last page not reached
    // -----------------------------------------------------------------------------------
    if (_indexPage < END_PAGE_ADDR) {
      _indexCol = 0;                                              // update column
      _indexPage = _indexPage + 1;                                // update page
      status = SSD1306_SetPosition (_indexCol, _indexPage);       // update position
      if (SSD1306_SUCCESS != status) {                            // check status
        return status;                                            // error
      }
    }
    // last page reached
    // -----------------------------------------------------------------------------------
    if (_indexPage >= END_PAGE_ADDR) {
      return SSD1306_ERROR;                                       // return out of range     
    }
  }
 
  return SSD1306_SUCCESS;                                         // success
}

/**
 * @desc    SSD1306 Draw character
 *
 * @param   char character
 *
 * @return  uint8_t
 */
uint8_t SSD1306_DrawChar (char ch)
{
  uint8_t status = INIT_STATUS;                                   // TWI init status 0xFF
  uint16_t i = 0;                                                 // counter

  // update text position
  // -------------------------------------------------------------------------------------
  status = SSD1306_UpdatePosition();
  if (SSD1306_SUCCESS != status) {                                // check status
    return status;                                                // error
  }
  // TWI start & SLAW
  // -------------------------------------------------------------------------------------
  status = SSD1306_Send_StartAndSLAW (SSD1306_ADDR);              // start & SLAW
  if (SSD1306_SUCCESS != status) {                                // check status
    return status;                                                // error
  }
  // TWI control byte data stream
  // -------------------------------------------------------------------------------------
  status = TWI_MT_Send_Data (SSD1306_DATA_STREAM);                // send data 0x40
  if (SSD1306_SUCCESS != status) {                                // check status
    return status;                                                // error
  }
  //  send character byte to memory lcd
  // -------------------------------------------------------------------------------------
  while (i < CHARS_COLS_LENGTH) {
    status = TWI_MT_Send_Data (pgm_read_byte (&FONTS[ch-32][i])); // send data col
    if (SSD1306_SUCCESS != status) {                              // check status
      return status;                                              // error
    }
    i++;                                                          // update counter
  }
  //  send empty column to memory lcd
  // -------------------------------------------------------------------------------------
  status = TWI_MT_Send_Data (CLEAR_COLOR);                        // ONE empty column
  if (SSD1306_SUCCESS != status) {                                // check status
    return status;                                                // error
  }

  _indexCol = _indexCol + CHARS_COLS_LENGTH + 1;                  // update global col

  // TWI stop
  // -------------------------------------------------------------------------------------
  TWI_Stop ();

  return SSD1306_SUCCESS;                                         // success
}

/**
 * @desc    SSD1306 Draw String
 *
 * @param   char * string
 *
 * @return  uint8_t
 */
uint8_t SSD1306_DrawString (char *str)
{
  uint8_t i = 0;                                                  // char counter

  // send characters of string
  // ------------------------------------------------------------------------------------- 
  while (str[i] != '\0') {
    SSD1306_DrawChar (str[i++]);                                  // send char
  }

  return SSD1306_SUCCESS;                                         // success
}

/**
 * @desc    Draw pixel
 *
 * @param   uint8_t x -> 0 ... MAX_X
 * @param   uint8_t y -> 0 ... MAX_Y
 *
 * @return  uint8_t
 */
uint8_t SSD1306_DrawPixel (uint8_t x, uint8_t y)
{
  uint8_t page = 0;                                               // page
  uint8_t pixel = 0;                                              // pixel
  uint8_t status = INIT_STATUS;                                   // TWI init status 0xFF
  
  if ((x > MAX_X) && (y > MAX_Y)) {                               // if out of range
    return SSD1306_ERROR;                                         // error
  }

  page = y >> 3;                                                  // find page (y / 8)
  pixel = 1 << (y - (page << 3));                                 // which pixel (y % 8)

  // set position
  // -------------------------------------------------------------------------------------
  status = SSD1306_SetPosition (x, page);                         // update position
  if (SSD1306_SUCCESS != status) {                                // check status
    return status;                                                // error
  }

  // TWI start & SLAW
  // -------------------------------------------------------------------------------------
  status = SSD1306_Send_StartAndSLAW (SSD1306_ADDR);              // start & SLAW
  if (SSD1306_SUCCESS != status) {                                // check status
    return status;                                                // error
  }
  // TWI control byte data stream
  // -------------------------------------------------------------------------------------
  status = TWI_MT_Send_Data (SSD1306_DATA_STREAM);                // send data 0x40
  if (SSD1306_SUCCESS != status) {                                // check status
    return status;                                                // error
  }
  //  send clear byte to memory lcd
  // -------------------------------------------------------------------------------------
  status = TWI_MT_Send_Data (pixel);                              // send pixel
  if (SSD1306_SUCCESS != status) {                                // check status
    return status;                                                // error
  }

  // TWI stop
  // -------------------------------------------------------------------------------------
  TWI_Stop ();

  return SSD1306_SUCCESS;                                         // success
}

/**
 * @desc    Draw line by Bresenham algoritm
 *
 * @param   uint8_t x start position / 0 <= cols <= MAX_X-1
 * @param   uint8_t x end position   / 0 <= cols <= MAX_X-1
 * @param   uint8_t y start position / 0 <= rows <= MAX_Y-1
 * @param   uint8_t y end position   / 0 <= rows <= MAX_Y-1
 *
 * @return  uint8_t
 */
uint8_t SSD1306_DrawLine (uint8_t x1, uint8_t x2, uint8_t y1, uint8_t y2)
{
  int16_t D;                                                      // determinant
  int16_t delta_x, delta_y;                                       // deltas
  int16_t trace_x = 1, trace_y = 1;                               // steps

  delta_x = x2 - x1;                                              // delta x
  delta_y = y2 - y1;                                              // delta y

  // Vertical line
  // -------------------------------------------------------------------------------------
  if (delta_x == 0) {
    status = SSD1306_DrawLineVert (x1, y1, y2);                   // draw vertical line
    if (SSD1306_SUCCESS != status) {                              // check status
      return status;                                              // error
    }
    return SSD1306_SUCCESS;                                       // success return    
  }
  //  Horizontal line
  // -------------------------------------------------------------------------------------
  if (delta_y == 0) {
    status = SSD1306_DrawLineHorz (y1, x1, x2);                   // draw horizontal line
    if (SSD1306_SUCCESS != status) {                              // check status
      return status;                                              // error
    }
    return SSD1306_SUCCESS;                                       // success return    
  } 

  if (delta_x < 0) {                                              // check if x2 > x1
    delta_x = -delta_x;                                           // negate delta x
    trace_x = -trace_x;                                           // negate step x
  }

  if (delta_y < 0) {                                              // check if y2 > y1
    delta_y = -delta_y;                                           // negate delta y
    trace_y = -trace_y;                                           // negate step y
  }
  // condition for m < 1 (dy < dx)
  // -------------------------------------------------------------------------------------
  if (delta_y < delta_x) {                                        //
    D = (delta_y << 1) - delta_x;                                 // calculate determinant
    SSD1306_DrawPixel (x1, y1);                                   // draw first pixel
    while (x1 != x2) {                                            // check if x1 equal x2
      x1 += trace_x;                                              // update x1
      if (D >= 0) {                                               // positive?
        y1 += trace_y;                                            // update y1
        D -= 2*delta_x;                                           // update determinant
      }
      D += 2*delta_y;                                             // update deteminant
      SSD1306_DrawPixel (x1, y1);                                 // draw next pixel
    }
  // condition for m >= 1 (dy >= dx)
  // -------------------------------------------------------------------------------------
  } else {
    D = delta_y - (delta_x << 1);                                 // calculate determinant
    SSD1306_DrawPixel (x1, y1);                                   // draw first pixel
    while (y1 != y2) {                                            // check if y2 equal y1
      y1 += trace_y;                                              // update y1
      if (D <= 0) {                                               // positive?
        x1 += trace_x;                                            // update y1
        D += 2*delta_y;                                           // update determinant
      }
      D -= 2*delta_x;                                             // update deteminant
      SSD1306_DrawPixel (x1, y1);                                 // draw next pixel
    }
  }

  return SSD1306_SUCCESS;                                         // success return
}
