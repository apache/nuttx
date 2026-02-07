/****************************************************************************
 * include/nuttx/lcd/st7796.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_LCD_ST7796_H
#define __INCLUDE_NUTTX_LCD_ST7796_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ST7796 Commands - Standard */

#define ST7796_NOP              0x00  /* NOP */
#define ST7796_SWRESET          0x01  /* Software Reset */
#define ST7796_RDDID            0x04  /* Read Display ID */
#define ST7796_RDDST            0x09  /* Read Display Status */
#define ST7796_SLPIN            0x10  /* Sleep In */
#define ST7796_SLPOUT           0x11  /* Sleep Out */
#define ST7796_PTLON            0x12  /* Partial Display Mode On */
#define ST7796_NORON            0x13  /* Normal Display Mode On */
#define ST7796_RDIMGFMT         0x0a  /* Read Display Image Format */
#define ST7796_RDSELFDIAG       0x0f  /* Read Display Self-Diagnostic Result */
#define ST7796_INVOFF           0x20  /* Display Inversion Off */
#define ST7796_INVON            0x21  /* Display Inversion On */
#define ST7796_GAMMASET         0x26  /* Gamma Set */
#define ST7796_DISPOFF          0x28  /* Display Off */
#define ST7796_DISPON           0x29  /* Display On */
#define ST7796_CASET            0x2a  /* Column Address Set */
#define ST7796_RASET            0x2b  /* Row Address Set */
#define ST7796_RAMWR            0x2c  /* Memory Write */
#define ST7796_RAMRD            0x2e  /* Memory Read */
#define ST7796_PTLAR            0x30  /* Partial Area */
#define ST7796_VSCRDEF          0x33  /* Vertical Scrolling Definition */
#define ST7796_TEOFF            0x34  /* Tearing Effect Line Off */
#define ST7796_TEON             0x35  /* Tearing Effect Line On */
#define ST7796_MADCTL           0x36  /* Memory Access Control */
#define ST7796_VSCRSADD         0x37  /* Vertical Scrolling Start Address */
#define ST7796_PIXFMT           0x3a  /* Pixel Format Set */
#define ST7796_WRDISPBRIGHT     0x51  /* Write Display Brightness */
#define ST7796_RDDISPBRIGHT     0x52  /* Read Display Brightness */
#define ST7796_WRCTRLD          0x53  /* Write Control Display */
#define ST7796_RDCTRLD          0x54  /* Read Control Display */
#define ST7796_WRCABC           0x55  /* Write Content Adaptive Brightness */
#define ST7796_RDCABC           0x56  /* Read Content Adaptive Brightness */
#define ST7796_WRCABCMIN        0x5e  /* Write CABC Minimum Brightness */
#define ST7796_RDCABCMIN        0x5f  /* Read CABC Minimum Brightness */

/* ST7796 Commands - Extended */

#define ST7796_INVCTR           0xb4  /* Display Inversion Control */
#define ST7796_DFC              0xb6  /* Display Function Control */
#define ST7796_PWCTRL1          0xc0  /* Power Control 1 */
#define ST7796_PWCTRL2          0xc1  /* Power Control 2 */
#define ST7796_PWCTRL3          0xc2  /* Power Control 3 */
#define ST7796_PWCTRL4          0xc3  /* Power Control 4 */
#define ST7796_PWCTRL5          0xc4  /* Power Control 5 */
#define ST7796_VCOM             0xc5  /* VCOM Control */
#define ST7796_PWCTRL6          0xc6  /* Power Control 6 */
#define ST7796_GAMMAPOS         0xe0  /* Positive Gamma Correction */
#define ST7796_GAMMANEG         0xe1  /* Negative Gamma Correction */
#define ST7796_DOCA             0xe9  /* Set DDB Write Address */
#define ST7796_CSCON            0xf0  /* Command Set Control */

/* ST7796 MADCTL bits */

#define ST7796_MADCTL_MY        0x80  /* Row Address Order */
#define ST7796_MADCTL_MX        0x40  /* Column Address Order */
#define ST7796_MADCTL_MV        0x20  /* Row/Column Exchange */
#define ST7796_MADCTL_ML        0x10  /* Vertical Refresh Order */
#define ST7796_MADCTL_BGR       0x08  /* BGR color filter panel */
#define ST7796_MADCTL_MH        0x04  /* Horizontal Refresh Order */

/* Pre-defined MADCTL values for common orientations */

#define ST7796_MADCTL_PORTRAIT           (ST7796_MADCTL_MX)
#define ST7796_MADCTL_PORTRAIT_BGR       (ST7796_MADCTL_MX | ST7796_MADCTL_BGR)
#define ST7796_MADCTL_RPORTRAIT          (ST7796_MADCTL_MY)
#define ST7796_MADCTL_RPORTRAIT_BGR      (ST7796_MADCTL_MY | ST7796_MADCTL_BGR)
#define ST7796_MADCTL_LANDSCAPE          (ST7796_MADCTL_MV)
#define ST7796_MADCTL_LANDSCAPE_BGR      (ST7796_MADCTL_MV | ST7796_MADCTL_BGR)
#define ST7796_MADCTL_RLANDSCAPE         (ST7796_MADCTL_MY | ST7796_MADCTL_MX | \
                                          ST7796_MADCTL_MV)
#define ST7796_MADCTL_RLANDSCAPE_BGR     (ST7796_MADCTL_MY | ST7796_MADCTL_MX | \
                                          ST7796_MADCTL_MV | ST7796_MADCTL_BGR)

/* Display raw dimensions (before orientation transform) */

#define ST7796_XRES_RAW         320
#define ST7796_YRES_RAW         480

/* Default SPI frequency */

#define ST7796_SPI_MAXFREQUENCY 40000000

/* Rotation ioctl commands (if not defined in fb.h) */

#ifndef FBIOSET_ROTATION
#  define FBIOSET_ROTATION      _FBIOC(0x0100)
#endif

#ifndef FBIOGET_ROTATION
#  define FBIOGET_ROTATION      _FBIOC(0x0101)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Command sequence entry for initialization */

struct st7796_cmd_s
{
  uint8_t cmd;              /* Command byte */
  FAR const uint8_t *data;  /* Parameter data (NULL if no params) */
  uint8_t len;              /* Number of parameter bytes */
  uint16_t delay_ms;        /* Delay after command in milliseconds */
};

/* Board-specific configuration passed to driver at initialization.
 * This structure allows board code to configure the driver without
 * requiring board-specific Kconfig options in the generic driver.
 */

struct st7796_config_s
{
  uint32_t frequency;       /* SPI clock frequency in Hz */
  uint16_t xres;            /* Horizontal resolution (after orientation) */
  uint16_t yres;            /* Vertical resolution (after orientation) */
  uint16_t rotation;        /* Initial rotation: 0, 90, 180, or 270 */
  uint8_t bpp;              /* Bits per pixel: 16 (RGB565) or 18 (RGB666) */
  uint8_t madctl;           /* Base MADCTL register value for orientation */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: st7796_fbinitialize
 *
 * Description:
 *   Initialize the ST7796 LCD controller as a framebuffer device.
 *
 *   This function initializes the ST7796 display controller using the
 *   provided configuration and returns a framebuffer virtual table.
 *   The driver uses CONFIG_SPI_CMDDATA to control the DC (Data/Command)
 *   pin automatically via the SPI driver.
 *
 * Input Parameters:
 *   spi    - SPI device instance configured for the ST7796
 *   config - Board-specific configuration (frequency, resolution, etc.)
 *
 * Returned Value:
 *   Pointer to framebuffer vtable on success; NULL on failure.
 *
 * Assumptions:
 *   - CONFIG_SPI_CMDDATA is enabled
 *   - DC pin has been registered with the SPI driver
 *   - CS pin has been registered with the SPI driver
 *   - RESET and backlight pins configured by board code
 *
 ****************************************************************************/

FAR struct fb_vtable_s *st7796_fbinitialize(FAR struct spi_dev_s *spi,
                                            FAR const struct st7796_config_s
                                            *config);

/****************************************************************************
 * Name: st7796_setrotation
 *
 * Description:
 *   Set display rotation at runtime. Valid rotation values are 0, 90,
 *   180, and 270 degrees. The rotation is applied by modifying the
 *   MADCTL register.
 *
 * Input Parameters:
 *   vtable   - Reference to the framebuffer virtual table
 *   rotation - Rotation angle in degrees (0, 90, 180, or 270)
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int st7796_setrotation(FAR struct fb_vtable_s *vtable, uint16_t rotation);

/****************************************************************************
 * Name: st7796_getrotation
 *
 * Description:
 *   Get current display rotation.
 *
 * Input Parameters:
 *   vtable - Reference to the framebuffer virtual table
 *
 * Returned Value:
 *   Current rotation in degrees (0, 90, 180, or 270).
 *
 ****************************************************************************/

uint16_t st7796_getrotation(FAR struct fb_vtable_s *vtable);

#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_LCD_ST7796_H */
