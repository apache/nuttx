/****************************************************************************
 * include/nuttx/lcd/ssd1680.h
 *
 * Driver for Solomon Systech SSD1680 e-paper controller
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

#ifndef __INCLUDE_NUTTX_LCD_SSD1680_H
#define __INCLUDE_NUTTX_LCD_SSD1680_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include <nuttx/arch.h>

#ifdef CONFIG_LCD_SSD1680
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* SSD1680 configuration settings:
 *
 * CONFIG_SSD1680_SPIMODE - Controls the SPI mode
 * CONFIG_SSD1680_FREQUENCY - Define to use a different bus frequency
 *
 * Required LCD driver settings:
 *
 * CONFIG_LCD_MAXCONTRAST should be 255, but any value >0 and <=255
 *   will be accepted.
 * CONFIG_LCD_MAXPOWER must be 1
 *
 * Optional LCD driver settings:
 * CONFIG_LCD_LANDSCAPE, CONFIG_LCD_PORTRAIT, CONFIG_LCD_RLANDSCAPE, and
 *   CONFIG_LCD_RPORTRAIT - Display orientation.
 *
 * Required SPI driver settings:
 * CONFIG_SPI_CMDDATA - Include support for cmd/data selection.
 */

/* SPI Interface
 *
 * "The serial interface consists of serial clock SCL, serial data SI, CS and
 *  CMD/!DTA. SI is shifted into an 8-bit shift register on every rising edge
 *  of SCL in the order of D7, D6, ... and D0. CMD/!DTA is sampled on every
 *  eighth clock and the data byte in the shift register is written to the
 *  display data RAM or command register in the same clock."
 *
 * "This module determines whether the input data is interpreted as data or
 * command. When CMD/!DTA = "H," the inputs at D7 - D0 are interpreted as
 * data and be written to display RAM. When CMD/!DTA = "L", the inputs at
 * D7 - D0 are interpreted as command, they will be decoded and be written
 * to the corresponding command registers."
 */

#ifndef CONFIG_SPI_CMDDATA
#  error "CONFIG_SPI_CMDDATA must be defined in your NuttX configuration"
#endif

/* Check contrast selection */

#if !defined(CONFIG_LCD_MAXCONTRAST)
#  define CONFIG_LCD_MAXCONTRAST 255
#endif

#if CONFIG_LCD_MAXCONTRAST <= 0 || CONFIG_LCD_MAXCONTRAST > 255
#  error "CONFIG_LCD_MAXCONTRAST exceeds supported maximum"
#endif

/* Check power setting */

#if !defined(CONFIG_LCD_MAXPOWER)
#  define CONFIG_LCD_MAXPOWER 1
#endif

#if CONFIG_LCD_MAXPOWER != 1
#  warning "CONFIG_LCD_MAXPOWER exceeds supported maximum"
#  undef CONFIG_LCD_MAXPOWER
#  define CONFIG_LCD_MAXPOWER 1
#endif

/* Color is 1bpp monochrome with leftmost column contained in bits 0  */

#ifdef CONFIG_NX_DISABLE_1BPP
#  warning "1 bit-per-pixel support needed"
#endif

/* Orientation */

#if defined(CONFIG_LCD_LANDSCAPE)
#  undef CONFIG_LCD_PORTRAIT
#  undef CONFIG_LCD_RLANDSCAPE
#  undef CONFIG_LCD_RPORTRAIT
#elif defined(CONFIG_LCD_PORTRAIT)
#  undef CONFIG_LCD_LANDSCAPE
#  undef CONFIG_LCD_RLANDSCAPE
#  undef CONFIG_LCD_RPORTRAIT
#elif defined(CONFIG_LCD_RLANDSCAPE)
#  undef CONFIG_LCD_LANDSCAPE
#  undef CONFIG_LCD_PORTRAIT
#  undef CONFIG_LCD_RPORTRAIT
#elif defined(CONFIG_LCD_RPORTRAIT)
#  undef CONFIG_LCD_LANDSCAPE
#  undef CONFIG_LCD_PORTRAIT
#  undef CONFIG_LCD_RLANDSCAPE
#else
#  define CONFIG_LCD_LANDSCAPE 1
#  warning "Assuming landscape orientation"
#endif

/* Some important "colors" */

#define SSD1680_Y1_BLACK  0
#define SSD1680_Y1_WHITE  1

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct ssd1680_priv_s
{
  bool (*set_vcc) (bool on);    /* Allow board to control display power. Return
                                 * true if request state set successfully. */
  bool (*set_rst) (bool on);    /* Hardware reset support */
  bool (*check_busy) (void);    /* Checks the state of busy pin */
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
 * Name:  ssd1680initialize
 *
 * Description:
 *   Initialize the video hardware.  The initial state of the OLED is
 *   fully initialized, display memory cleared, and the OLED ready to
 *   use, but with the power setting at 0 (full off == sleep mode).
 *
 * Input Parameters:
 *
 *   dev - A reference to the SPI driver instance.
 *   board_priv - Board specific structure.
 *
 * Returned Value:
 *
 *   On success, this function returns a reference to the LCD object for
 *   the specified OLED.  NULL is returned on any failure.
 *
 ****************************************************************************/

struct lcd_dev_s;    /* See include/nuttx/lcd/lcd.h */
struct spi_dev_s;    /* See include/nuttx/spi/spi.h */

FAR struct lcd_dev_s *ssd1680_initialize(FAR struct spi_dev_s *dev,
                          FAR const struct ssd1680_priv_s *board_priv);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_LCD_SSD1680 */
#endif /* __INCLUDE_NUTTX_LCD_SSD1680_H */
