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

/* Display raw dimensions (before orientation transform)
 *
 * These must be in the public header because board code needs them to
 * calculate the effective xres/yres based on Kconfig orientation settings
 * at compile-time. Without these, boards would need to hardcode dimensions.
 */

#define ST7796_XRES_RAW         320
#define ST7796_YRES_RAW         480

/* Default SPI frequency
 *
 * Must be in the public header because board code uses this as a safe
 * default when the user hasn't specified a custom frequency via Kconfig.
 * Centralizes the datasheet specification to prevent duplication.
 */

#define ST7796_SPI_MAXFREQUENCY 40000000

/* Pre-defined MADCTL values for common orientations
 *
 * These must be in the public header because board code selects the
 * appropriate MADCTL value based on hardware orientation and RGB/BGR
 * panel type configured via Kconfig. This abstracts the internal MADCTL
 * bit manipulation from board code. Values are absolute to eliminate
 * dependency on internal bit definitions.
 */

#define ST7796_MADCTL_PORTRAIT           0x40
#define ST7796_MADCTL_PORTRAIT_BGR       0x48
#define ST7796_MADCTL_RPORTRAIT          0x80
#define ST7796_MADCTL_RPORTRAIT_BGR      0x88
#define ST7796_MADCTL_LANDSCAPE          0x20
#define ST7796_MADCTL_LANDSCAPE_BGR      0x28
#define ST7796_MADCTL_RLANDSCAPE         0xe0
#define ST7796_MADCTL_RLANDSCAPE_BGR     0xe8

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Board-specific configuration passed to driver at initialization.
 *
 * This structure must be in the public header because it defines the
 * driver's configuration API. Board code instantiates and populates this
 * structure to configure the driver without requiring board-specific
 * Kconfig options in the generic driver.
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
