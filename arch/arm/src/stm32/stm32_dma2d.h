/******************************************************************************
 * arch/arm/src/stm32/stm32_dma2d.h
 *
 *   Copyright (C) 2014-2015 Marco Krahl. All rights reserved.
 *   Author: Marco Krahl <ocram.lhark@gmail.com>
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
 ******************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32_STM32_DMA2D_H
#define __ARCH_ARM_SRC_STM32_STM32_DMA2D_H

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include <nuttx/config.h>
#include <nuttx/video/fb.h>
#include <arch/chip/ltdc.h>

#ifdef CONFIG_STM32_DMA2D
/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/******************************************************************************
 * Public Types
 ******************************************************************************/

/******************************************************************************
 * Public Data
 ******************************************************************************/

/******************************************************************************
 * Public Functions
 ******************************************************************************/

# ifdef CONFIG_STM32_LTDC_INTERFACE
/******************************************************************************
 * Name: stm32_dma2dinitltdc
 *
 * Description:
 *   Get a reference to the dma2d layer coupled with the ltdc layer.
 *   It not intends to use this function by user space applications.
 *   It resolves the following requirements:
 *   1. Share the color lookup table
 *   2. Share the planeinfo information
 *   3. Share the videoinfo information
 *
 * Parameter:
 *   layer  - a valid reference to the low level ltdc layer structure
 *
 * Return:
 *   On success - A valid dma2d layer reference
 *   On error   - NULL and errno is set to
 *                -EINVAL if one of the parameter is invalid
 *
 ******************************************************************************/

FAR struct dma2d_layer_s * stm32_dma2dinitltdc(FAR struct stm32_ltdc_s *layer);
# endif /* CONFIG_STM32_LTDC_INTERFACE */

#endif /* CONFIG_STM32_DMA2D */
#endif /* __ARCH_ARM_SRC_STM32_STM32_DMA2D_H */
