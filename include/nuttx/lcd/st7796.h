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
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

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
 *   This function initializes the ST7796 display controller and returns
 *   a framebuffer virtual table. Display parameters (resolution, color
 *   depth, orientation, frequency) are taken from CONFIG_LCD_ST7796_*
 *   Kconfig options. The driver uses CONFIG_SPI_CMDDATA to control the
 *   DC (Data/Command) pin automatically via the SPI driver.
 *
 * Input Parameters:
 *   spi - SPI device instance configured for the ST7796
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

FAR struct fb_vtable_s *st7796_fbinitialize(FAR struct spi_dev_s *spi);

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

/****************************************************************************
 * Name: st7796_board_power
 *
 * Description:
 *   Board-level callback to control display backlight/power.
 *   Called by the ST7796 driver when FBIOSET_POWER ioctl is
 *   received. The board must implement this function to
 *   control the backlight GPIO pin.
 *
 * Input Parameters:
 *   on - true to turn backlight on, false to turn off
 *
 ****************************************************************************/

void st7796_board_power(bool on);

#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_LCD_ST7796_H */
