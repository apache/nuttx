/****************************************************************************
 * include/nuttx/lcd/lcddrv_spiif.h
 *
 *   Copyright (C) 2020 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Dave Marples <dave@marples.net>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
