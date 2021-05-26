/****************************************************************************
 * include/nuttx/lcd/lcddrv_spiif.h
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

#ifndef __INCLUDE_NUTTX_LCD_LCDDRV_SPIIF_H
#define __INCLUDE_NUTTX_LCD_LCDDRV_SPIIF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct lcddrv_lcd_s
{
  /* Interface to control the ILI9341 lcd driver
   *
   *  - select      Select the device (as necessary) before performing
   *                any operations.
   *  - deselect    Deselect the device (as necessary).
   *  - send        Send specific parameter to the LCD driver.
   *  - recv        Receive specific parameter from the LCD driver.
   *  - sendmulti   Send pixel data to the LCD drivers gram.
   *  - recvmulti   Receive pixel data from the LCD drivers gram.
   *  - backlight   Change the backlight level of the connected display.
   *                In the context of the ili9341 that means change the
   *                backlight level of the connected LED driver.
   *                The implementation in detail is part of the platform
   *                specific sub driver.
   */

  CODE void (*select)(FAR struct lcddrv_lcd_s *lcd);
  CODE void (*deselect)(FAR struct lcddrv_lcd_s *lcd);
  CODE int  (*sendcmd)(FAR struct lcddrv_lcd_s *lcd, const uint8_t cmd);
  CODE int  (*sendparam)(FAR struct lcddrv_lcd_s *lcd, const uint8_t param);
  CODE int  (*recvparam)(FAR struct lcddrv_lcd_s *lcd, FAR uint8_t *param);
  CODE int  (*recvgram)(FAR struct lcddrv_lcd_s *lcd,
                        FAR uint16_t *wd, uint32_t nwords);
  CODE int  (*sendgram)(FAR struct lcddrv_lcd_s *lcd,
                        FAR const uint16_t *wd, uint32_t nwords);
  CODE int  (*backlight)(FAR struct lcddrv_lcd_s *lcd, int level);

  /* MCU interface specific data following */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: spiif_backlight
 *       (Provided by integrating platform)
 *
 * Description:
 *   Set the backlight level of the connected display.
 *
 * Input Parameters:
 *   spi   - Reference to the public driver structure
 *   level - backlight level
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/

extern int spiif_backlight(FAR struct lcddrv_lcd_s *lcd, int level);

/****************************************************************************
 * Name:  FAR struct lcddrv_lcd_s *lcddrv_spiif_initialize
 *
 * Description:
 *   Initialize the device structure to control the LCD Single chip driver.
 *
 * Input Parameters:
 *   path : path to SPI device to use
 *
 * Returned Value:
 *   On success, this function returns a reference to the LCD control object
 *   for the specified LCDDRV LCD Single chip driver.
 *   NULL is returned on failure.
 *
 ****************************************************************************/

FAR struct lcddrv_lcd_s *lcddrv_spiif_initialize(FAR struct spi_dev_s *spi);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_LCD_LCDDRV_SPIIF_H */
