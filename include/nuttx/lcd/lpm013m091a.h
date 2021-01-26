/****************************************************************************
 * include/nuttx/lcd/lpm013m091a.h
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

#ifndef __INCLUDE_NUTTX_LCD_LPM013M091A_H
#define __INCLUDE_NUTTX_LCD_LPM013M091A_H

/* Command set. */

#define LPM013M091A_SWRESET 0x01

#define LPM013M091A_SLPIN   0x10
#define LPM013M091A_SLPOUT  0x11

#define LPM013M091A_DISPOFF 0x28
#define LPM013M091A_DISPON  0x29
#define LPM013M091A_CASET   0x2A
#define LPM013M091A_PASET   0x2B
#define LPM013M091A_RAMWR   0x2C

#define LPM013M091A_PIXFMT  0x3A

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Response CODE
 ****************************************************************************/

struct lpm013m091a_lcd_s
{
  /* Interface to control the LPM013M091A lcd driver
   *
   *  - select      Select the device (as necessary) before performing
   *                any operations.
   *  - deselect    Deselect the device (as necessary).
   *  - sendcmd     Send specific command to the LCD driver.
   *  - sendparam   Send specific parameter to the LCD driver.
   *  - recvparam   Receive specific parameter from the LCD driver.
   *  - sendgram    Send pixel data to the LCD drivers gram.
   *  - recvgram    Receive pixel data from the LCD drivers gram.
   *  - backlight   Change the backlight level of the connected display.
   *                In the context of the lpm013m091a that means change the
   *                backlight level of the connected LED driver.
   *                The implementation in detail is part of the platform
   *                specific sub driver.
   *
   */

  void (*select)(FAR struct lpm013m091a_lcd_s *lcd);
  void (*deselect)(FAR struct lpm013m091a_lcd_s *lcd);
  int (*sendcmd)(FAR struct lpm013m091a_lcd_s *lcd, const uint8_t cmd);
  int (*sendparam)(FAR struct lpm013m091a_lcd_s *lcd, const uint8_t param);
  int (*recvparam)(FAR struct lpm013m091a_lcd_s *lcd, uint8_t *param);
  int (*recvgram)(FAR struct lpm013m091a_lcd_s *lcd,
                  uint16_t *wd, uint32_t nwords);
  int (*sendgram)(FAR struct lpm013m091a_lcd_s *lcd,
                  const uint16_t *wd, uint32_t nwords);
  int (*backlight)(FAR struct lpm013m091a_lcd_s *lcd, int level);

  /* mcu interface specific data following */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name:  lpm013m091a_initialize
 *
 * Description:
 *   Initialize the LCD video driver internal structure. Also initialize the
 *   lcd hardware if not done. The control of the LCD driver is depend on the
 *   selected MCU interface and part of the platform specific subdriver
 *
 *
 * Input Parameters:
 *
 *   lcd - A reference to the platform specific driver instance to control
 *         the lpm013m091a display driver.
 *   devno - This is for compat. must be zero.
 *
 * Returned Value:
 *
 *   On success, this function returns a reference to the LCD driver object
 *   for the specified LCD driver. NULL is returned on any failure.
 *
 ****************************************************************************/

FAR struct lcd_dev_s *
  lpm013m091a_initialize(FAR struct lpm013m091a_lcd_s *lcd, int devno);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_LCD_LPM013M091A_H */
