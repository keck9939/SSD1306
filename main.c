/** 
 * --------------------------------------------------------------------------------------+  
 * @name        OLED SSD1306 example
 * --------------------------------------------------------------------------------------+ 
 *              Copyright (C) 2020 Marian Hrinko.
 *              Written by Marian Hrinko (mato.hrinko@gmail.com)
 *
 * @author      Marian Hrinko
 * @date        06.10.2020
 * @file        main.c
 * @version     2.0.0
 * @test        AVR Atmega328p
 *
 * @depend      lib/ssd1306.h
 * --------------------------------------------------------------------------------------+ 
 * @brief       Version 1.0.0 -> applicable for 1 display
 *              Version 2.0.0 -> rebuild to 'cacheMemLcd' array
 *              Version 3.0.0 -> simplified alphanumeric version for 1 display
 * --------------------------------------------------------------------------------------+ 
 */

// include libraries
#include "ssd1306.h"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#define _delay_ms(a) Sleep(a)
#else
#include <util/delay.h>
#endif

/**
 * @desc    Main function
 *
 * @param   void
 *
 * @return  int
 */
int main(void)
{
  // LCD INIT
  // ------------------------------------------------------------------------------------- 
  SSD1306_Init(SSD1306_ADDR);                                    // 0x3C
  _delay_ms(100);
  // DRAWING
  // -------------------------------------------------------------------------------------
  SSD1306_ClearScreen();                                         // clear screen
  //SSD1306_DrawLine(0, MAX_X, 4, 4);                              // draw line
  SSD1306_SetPosition(7, 0);                                     // set position
  SSD1306_DrawString("SSD1306 OLED DRIVER");                     // draw string
  //SSD1306_DrawLine(0, MAX_X, 18, 18);                            // draw line
  SSD1306_SetPosition(40, 1);                                    // set position
  SSD1306_DrawString("MATIASUS");                                // draw string
  SSD1306_SetPosition(53, 2);                                    // set position
  SSD1306_DrawString("2021");                                    // draw string
  SSD1306_SetPosition(48, 3);                                    // set position
  SSD1306_DrawString("Line 4");                                    // draw string

  SSD1306_UpdateScreen(SSD1306_ADDR);                            // update
  _delay_ms(1000);

  SSD1306_ScrollLine();
  SSD1306_UpdateScreen(SSD1306_ADDR);                            // update

  _delay_ms(1000);
  SSD1306_InverseScreen(SSD1306_ADDR);

  _delay_ms(1000);
  SSD1306_NormalScreen(SSD1306_ADDR);

  // RETURN
  // -------------------------------------------------------------------------------------
  return 0;
}