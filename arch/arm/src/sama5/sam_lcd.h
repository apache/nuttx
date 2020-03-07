/************************************************************************************
 * arch/arm/src/sama5/sam_lcd.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMA5_SAM_LCDC_H
#define __ARCH_ARM_SRC_SAMA5_SAM_LCDC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include <nuttx/nx/nxglib.h>

#include "hardware/sam_lcdc.h"

#ifdef CONFIG_SAMA5_LCDC

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* These definitions provide the LCDC framebuffer memory description needed to
 * remap that region to be non-cacheable and non-bufferable
 */

#if (CONFIG_SAMA5_LCDC_FB_VBASE & 0x000fffff) != 0
#  error CONFIG_SAMA5_LCDC_FB_VBASE not aligned to 1MB boundary
#endif

#if (CONFIG_SAMA5_LCDC_FB_PBASE & 0x000fffff) != 0
#  error CONFIG_SAMA5_LCDC_FB_PBASE not aligned to 1MB boundary
#endif

#define SAMA5_LCDC_FBNSECTIONS \
  ((CONFIG_SAMA5_LCDC_FB_SIZE + 0x000fffff) >> 20)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/
/* The SAMA5 LCD driver uses the common framebuffer interfaces declared in
 * include/nuttx/video/fb.h.
 */

/************************************************************************************
 * Name: sam_lcdclear
 *
 * Description:
 *   This is a non-standard LCD interface just for the SAMA5.  Clearing the display
 *   in the normal way by writing a sequences of runs that covers the entire display
 *   can be slow.  Here the display is cleared by simply setting all VRAM memory to
 *   the specified color.
 *
 ************************************************************************************/

void sam_lcdclear(nxgl_mxpixel_t color);

/************************************************************************************
 * Name: sam_backlight
 *
 * Description:
 *   If CONFIG_SAM_LCD_BACKLIGHT is defined, then the board-specific logic must
 *   provide this interface to turn the backlight on and off.
 *
 ************************************************************************************/

#ifdef CONFIG_SAM_LCD_BACKLIGHT
void sam_backlight(bool blon);
#endif

#endif /* CONFIG_SAMA5_LCDC */
#endif /* __ARCH_ARM_SRC_SAMA5_SAM_LCDC_H */
