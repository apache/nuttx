/****************************************************************************
 * include/nuttx/lcd/ssd1351.h
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

#ifndef __INCLUDE_NUTTX_LCD_SSD1351_H
#define __INCLUDE_NUTTX_LCD_SSD1351_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_LCD_SSD1351

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* SSD1351 configuration settings:
 * CONFIG_SSD1351_PARALLEL8BIT - 8-bit parallel interface
 * CONFIG_SSD1351_SPI3WIRE     - 3-wire SPI interface
 * CONFIG_SSD1351_SPI4WIRE     - 4-wire SPI interface
 * CONFIG_SSD1351_SPIMODE      - SPI mode
 * CONFIG_SSD1351_SPIFREQ      - SPI frequency
 * CONFIG_SSD1351_NINTERFACES  - number of physical devices supported
 * CONFIG_SSD1351_XRES         - X resolution
 * CONFIG_SSD1351_YRES         - Y resolution
 * CONFIG_SSD1351_MIRRORX      - mirror along the X axis
 * CONFIG_SSD1351_MIRRORY      - mirror along the Y axis
 * CONFIG_SSD1351_INVERT       - invert the display
 * CONFIG_SSD1351_VDDEXT       - external VDD
 * CONFIG_SSD1351_TRST         - reset period
 * CONFIG_SSD1351_TPRECHG1     - first pre-charge period
 * CONFIG_SSD1351_PERFENHANCE  - enhance display performance
 * CONFIG_SSD1351_CLKDIV       - clock divider
 * CONFIG_SSD1351_OSCFREQ      - oscillator frequency
 * CONFIG_SSD1351_TPRECHG2     - second pre-charge period
 * CONFIG_SSD1351_VPRECHG      - pre-charge voltage level
 * CONFIG_SSD1351_VCOMH        - COM deselect voltage level
 * CONFIG_SSD1351_CONTRASTA    - color A contrast
 * CONFIG_SSD1351_CONTRASTB    - color B contrast
 * CONFIG_SSD1351_CONTRASTC    - color C contrast
 * CONFIG_SSD1351_MSTRCONTRAST - master contrast ratio
 *
 * Required LCD driver settings:
 * CONFIG_LCD_SSD1351          - enables SSD1351 support
 * CONFIG_LCD_MAXPOWER         - maximum power, must be 1
 *
 * Additional LCD driver settings:
 * CONFIG_LCD_LANDSCAPE        - landscape
 * CONFIG_LCD_RLANDSCAPE       - reverse landscape
 * CONFIG_LCD_PORTRAIT         - portrait
 * CONFIG_LCD_RPORTRAIT        - reverse portrait
 *
 * Required SPI driver settings:
 * CONFIG_SPI                  - enables support for SPI
 * CONFIG_SPI_CMDDATA          - enables support for cmd/data selection
 *                               (if using 4-wire SPI)
 *
 * NX settings that must be undefined:
 * CONFIG_NX_DISABLE_16BPP     - disables 16 bpp support
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_SSD1351_PARALLEL8BIT
struct ssd1351_lcd_s
{
  void    (*cmd)(FAR struct ssd1351_lcd_s *lcd, uint8_t cmd);
#ifndef CONFIG_LCD_NOGETRUN
  uint8_t (*read)(FAR struct ssd1351_lcd_s *lcd);
#endif
  void    (*write)(FAR struct ssd1351_lcd_s *lcd, uint8_t data);
};
#elif defined(CONFIG_SSD1351_SPI3WIRE) || defined(CONFIG_SSD1351_SPI4WIRE)
struct spi_dev_s;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: ssd1351_initialize
 *
 * Description:
 *   Initialize the video hardware.  The initial state of the device
 *   is fully initialized, display memory cleared, and ready to use,
 *   but with the power setting at 0 (full off == sleep mode).
 *
 * Input Parameters:
 *   lcd   - A reference to the platform-specific interface.
 *   spi   - A reference to the SPI driver instance.
 *   devno - A value in the range of 0 through CONFIG_SSD1351_NINTERFACES-1.
 *           This allows support for multiple devices.
 *
 * Returned Value:
 *   On success, this function returns a reference to the LCD object for the
 *   specified device.  NULL is returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SSD1351_PARALLEL8BIT
FAR struct lcd_dev_s *ssd1351_initialize(FAR struct ssd1351_lcd_s *lcd,
                                         unsigned int devno);
#elif defined(CONFIG_SSD1351_SPI3WIRE) || defined(CONFIG_SSD1351_SPI4WIRE)
FAR struct lcd_dev_s *ssd1351_initialize(FAR struct spi_dev_s *spi,
                                         unsigned int devno);
#endif

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_LCD_SSD1351 */
#endif /* __INCLUDE_NUTTX_LCD_SSD1351_H */
