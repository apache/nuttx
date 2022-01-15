/****************************************************************************
 * include/nuttx/lcd/ssd1306.h
 *
 * Driver for Univision UG-2864HSWEG01 OLED display or UG-2832HSWEG04 both
 * with the Univision SSD1306 controller in SPI mode and Densitron
 * DD-12864WO-4A with SSD1309 in SPI mode.
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

#ifndef __INCLUDE_NUTTX_LCD_SSD1306_H
#define __INCLUDE_NUTTX_LCD_SSD1306_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include <nuttx/arch.h>

#ifdef CONFIG_LCD_SSD1306

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* SSD1306 configuration settings:
 *
 * CONFIG_SSD1306_SPIMODE - Controls the SPI mode
 * CONFIG_SSD1306_FREQUENCY - Define to use a different bus frequency
 *
 * LCD panel selection settings:
 *
 * CONFIG_LCD_UG2864HSWEG01   - Enable UG-2864HSWEG01 support
 * CONFIG_LCD_UG2832HSWEG04   - Enable UG-2832HSWEG04 support
 * CONFIG_LCD_DD12864WO4A     - Enable DD-12864WO-4A support
 * CONFIG_LCD_HILETGO         - Enable HiletGo 129x64 OLED support
 * CONFIG_LCD_SH1106_OLED_132 - Enable SH1106 132x28 support
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
 * "The serial interface consists of serial clock SCL, serial data SI, A0 and
 *  CS. SI is shifted into an 8-bit shift register on every rising edge of
 *  SCL in the order of D7, D6, ... and D0. A0 is sampled on every eighth
 *  clock and the data byte in the shift register is written to the display
 *  data RAM or command register in the same clock."
 *
 * MODE 3:
 *   Clock polarity:  High (CPOL=1)
 *   Clock phase:     Sample on trailing (rising edge) (CPHA 1)
 */

#ifdef CONFIG_LCD_SSD1306_SPI

/* "This module determines whether the input data is interpreted as data or
 * command. When A0 = "H," the inputs at D7 - D0 are interpreted as data and
 * be written to display RAM. When A0 = "L", the inputs at D7 - D0 are
 * interpreted as command, they will be decoded and be written to the
 * corresponding command registers."
 */

#ifndef CONFIG_SPI_CMDDATA
#  error "CONFIG_SPI_CMDDATA must be defined in your NuttX configuration"
#endif

#endif /* CONFIG_LCD_SSD1306_SPI */

#ifdef CONFIG_LCD_SSD1306_I2C

#ifndef CONFIG_SSD1306_I2CADDR
#  define CONFIG_SSD1306_I2CADDR 0x78 /* 120 in decimal */
#endif

#ifndef CONFIG_SSD1306_I2CFREQ
#  define CONFIG_SSD1306_I2CADDR 400000
#endif

#endif /* CONFIG_LCD_SSD1306_I2C */

/* Check contrast selection */

#if !defined(CONFIG_LCD_MAXCONTRAST)
#  define CONFIG_LCD_MAXCONTRAST 255
#endif

#if CONFIG_LCD_MAXCONTRAST <= 0 || CONFIG_LCD_MAXCONTRAST > 255
#  error "CONFIG_LCD_MAXCONTRAST exceeds supported maximum"
#endif

#if CONFIG_LCD_MAXCONTRAST < 255
#  warning "Optimal setting of CONFIG_LCD_MAXCONTRAST is 255"
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

#define SSD1306_Y1_BLACK  0
#define SSD1306_Y1_WHITE  1

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct ssd1306_priv_s
{
  bool (*set_vcc) (bool on); /* Allow board to control display power. Return
                              * true if request state set successfully. */
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
 * Name:  ssd1306initialize
 *
 * Description:
 *   Initialize the video hardware.  The initial state of the OLED is
 *   fully initialized, display memory cleared, and the OLED ready to
 *   use, but with the power setting at 0 (full off == sleep mode).
 *
 * Input Parameters:
 *
 *   dev - A reference to the SPI/I2C driver instance.
 *   board_priv - Board specific structure.
 *   devno - A device number when there are multiple OLED devices.
 *     Currently must be zero.
 *
 * Returned Value:
 *
 *   On success, this function returns a reference to the LCD object for
 *   the specified OLED.  NULL is returned on any failure.
 *
 ****************************************************************************/

struct lcd_dev_s;    /* See include/nuttx/lcd/lcd.h */
struct spi_dev_s;    /* See include/nuttx/spi/spi.h */
struct i2c_master_s; /* See include/nuttx/i2c/i2c_master.h */

#ifdef CONFIG_LCD_SSD1306_SPI
FAR struct lcd_dev_s *ssd1306_initialize(FAR struct spi_dev_s *dev,
                          FAR const struct ssd1306_priv_s *board_priv,
                          unsigned int devno);
#else
FAR struct lcd_dev_s *ssd1306_initialize(FAR struct i2c_master_s *dev,
                          FAR const struct ssd1306_priv_s *board_priv,
                          unsigned int devno);
#endif

/****************************************************************************
 * Name:  ssd1306_fill
 *
 * Description:
 *   This non-standard method can be used to clear the entire display by
 *   writing one color to the display.  This is much faster than writing a
 *   series of runs.
 *
 * Input Parameters:
 *   dev   - Reference to LCD object
 *   color - Desired color
 *
 * Assumptions:
 *   Caller has selected the OLED section.
 *
 ****************************************************************************/

int ssd1306_fill(FAR struct lcd_dev_s *dev, uint8_t color);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_LCD_SSD1306 */
#endif /* __INCLUDE_NUTTX_LCD_SSD1306_H */
