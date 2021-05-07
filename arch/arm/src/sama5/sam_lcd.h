/****************************************************************************
 * arch/arm/src/sama5/sam_lcd.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_SAM_LCDC_H
#define __ARCH_ARM_SRC_SAMA5_SAM_LCDC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include <nuttx/nx/nxglib.h>

#include "hardware/sam_lcdc.h"

#ifdef CONFIG_SAMA5_LCDC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* These definitions provide the LCDC framebuffer memory description needed
 * to remap that region to be non-cacheable and non-bufferable
 */

#if (CONFIG_SAMA5_LCDC_FB_VBASE & 0x000fffff) != 0
#  error CONFIG_SAMA5_LCDC_FB_VBASE not aligned to 1MB boundary
#endif

#if (CONFIG_SAMA5_LCDC_FB_PBASE & 0x000fffff) != 0
#  error CONFIG_SAMA5_LCDC_FB_PBASE not aligned to 1MB boundary
#endif

#define SAMA5_LCDC_FBNSECTIONS \
  ((CONFIG_SAMA5_LCDC_FB_SIZE + 0x000fffff) >> 20)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/* The SAMA5 LCD driver uses the common framebuffer interfaces declared in
 * include/nuttx/video/fb.h.
 */

/****************************************************************************
 * Name: sam_lcdclear
 *
 * Description:
 *   This is a non-standard LCD interface just for the SAMA5.  Clearing the
 *   display in the normal way by writing a sequences of runs that covers the
 *   entire display can be slow.  Here the display is cleared by simply
 *   setting all VRAM memory to the specified color.
 *
 ****************************************************************************/

void sam_lcdclear(nxgl_mxpixel_t color);

/****************************************************************************
 * Name: sam_backlight
 *
 * Description:
 *   If CONFIG_SAM_LCD_BACKLIGHT is defined, then the board-specific logic
 *   must provide this interface to turn the backlight on and off.
 *
 ****************************************************************************/

#ifdef CONFIG_SAM_LCD_BACKLIGHT
void sam_backlight(bool blon);
#endif

#endif /* CONFIG_SAMA5_LCDC */
#endif /* __ARCH_ARM_SRC_SAMA5_SAM_LCDC_H */
