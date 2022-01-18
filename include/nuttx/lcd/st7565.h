/****************************************************************************
 * include/nuttx/lcd/st7565.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_LCD_ST7565_H
#define __INCLUDE_NUTTX_LCD_ST7565_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ST7565 Configuration Settings:
 *
 * CONFIG_ST7565_NINTERFACES - Specifies the number of physical
 *   ST7565 devices that will be supported.  NOTE:  At present, this
 *   must be undefined or defined to be 1.
 *
 * Required LCD driver settings:
 * CONFIG_LCD_ST7565 - Enable ST7565 support
 * CONFIG_LCD_MAXCONTRAST should be 255, but any value >0 and <=255 will be
 * accepted.
 */

/* Some important "colors" */

#define ST7565_Y1_BLACK  0
#define ST7565_Y1_WHITE  1

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct st7565_lcd_s
{
  /* Interface to control the st7565 like lcd driver
   *
   *  - reset       Switch reset pin of LCD (optional but as your risk).
   *  - select      Select the device (as necessary) before performing any
   *                operations.
   *  - deselect    Deselect the device (as necessary).
   *  - cmddata     Select command (A0 = 0) or data (A0 = 1) mode .
   *  - senddata    Send data to the LCD driver.
   *  - backlight   Change the backlight level of the connected display.
   *                In the context of the ili9341 that means change the
   *                backlight level of the connected LED driver.
   *                The implementation in detail is part of the platform
   *                specific sub driver.
   */

  void (*reset)(FAR struct st7565_lcd_s *lcd, bool on);
  void (*select)(FAR struct st7565_lcd_s *lcd);
  void (*deselect)(FAR struct st7565_lcd_s *lcd);
  void (*cmddata)(FAR struct st7565_lcd_s *lcd, const uint8_t cmd);
  int  (*senddata)(FAR struct st7565_lcd_s *lcd, FAR const uint8_t *data,
                   int size);
  int  (*backlight)(FAR struct st7565_lcd_s *lcd, int level);

  /* MCU interface specific data following */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name:  st7565_initialize
 *
 * Description:
 *   Initialize the ST7565 video hardware.  The initial state of the
 *   LCD is fully initialized, display memory cleared, and the LCD ready
 *   to use, but with the power setting at 0 (full off == sleep mode).
 *
 * Input Parameters:
 *
 *   spi - A reference to the SPI driver instance.
 *   devno - A value in the range of 0 throw CONFIG_ST7565_NINTERFACES-1.
 *     This allows support for multiple LCD devices.
 *
 * Returned Value:
 *
 *   On success, this function returns a reference to the LCD object for
 *   the specified LCD.  NULL is returned on any failure.
 *
 ****************************************************************************/

FAR struct lcd_dev_s *st7565_initialize(FAR struct st7565_lcd_s *lcd,
                                        unsigned int devno);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_LCD_ST7567_H */
