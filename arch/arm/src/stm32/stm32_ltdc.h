/************************************************************************************
 * arch/arm/src/stm32/stm32_ltdc.h
 *
 *   Copyright (C) 2013 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
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

#ifndef __ARCH_ARM_SRC_STM32_STM32_LTDC_H
#define __ARCH_ARM_SRC_STM32_STM32_LTDC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include <nuttx/nx/nxglib.h>

#include "chip/stm32_ltdc.h"

#ifdef CONFIG_STM32_LTDC

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/
/* The STM32 LTDC driver uses the common framebuffer interfaces declared in
 * include/nuttx/fb.h.
 */

/************************************************************************************
 * Name: stm32_lcdclear
 *
 * Description:
 *   This is a non-standard LCD interface just for the STM32.  Clearing the display
 *   in the normal way by writing a sequences of runs that covers the entire display
 *   can be slow.  Here the display is cleared by simply setting all buffer memory to
 *   the specified color.
 *
 ************************************************************************************/

void stm32_lcdclear(nxgl_mxpixel_t color);

/************************************************************************************
 * Name: stm32_lcd_backlight
 *
 * Description:
 *   If CONFIG_STM32_LCD_BACKLIGHT is defined, then the board-specific logic must
 *   provide this interface to turn the backlight on and off.
 *
 ************************************************************************************/

#ifdef CONFIG_STM_LCD_BACKLIGHT
void sam_lcd_backlight(bool blon);
#endif

#endif /* CONFIG_STM32_LTDC */
#endif /* __ARCH_ARM_SRC_STM32_STM32_LTDC_H */
