/******************************************************************************
 * arch/arm/src/stm32/stm32_dma2d.h
 *
 *   Copyright (C) 2014 Marco Krahl. All rights reserved.
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
#include "stm32_ltdc.h"

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

/******************************************************************************
 * Name: stm32_dma2dblit
 *
 * Description:
 *   Copy selected area from a background layer to selected position of the
 *   foreground layer. Copies the result to the destination layer.
 *
 * Parameter:
 *   dest     - Valid reference to the destination layer
 *   fore     - Valid reference to the foreground layer
 *   forexpos - Valid selected x target position of the destination layer
 *   foreypos - Valid selected y target position of the destination layer
 *   back     - Valid reference to the background layer
 *   backarea - Valid reference to the selected area of the background layer
 *
 * Return:
 *    OK     - On success
 *   -EINVAL - On error
 *
 ******************************************************************************/

int stm32_dma2dblit(FAR struct stm32_ltdc_s *dest,
                    FAR struct stm32_ltdc_s *fore,
                    fb_coord_t forexpos, fb_coord_t foreypos,
                    FAR struct stm32_ltdc_s *back,
                    FAR const struct ltdc_area_s *backarea);

/******************************************************************************
 *
 * Name: stm32_dma2dblend
 *
 * Description:
 *   Blends the selected area from a background layer with selected position of
 *   the foreground layer. Blends the result with the destination layer.
 *   Note! This is the same as the blit operation but with blending depending on
 *   the blendmode settings of the layer.
 *
 * Parameter:
 *   dest     - Valid reference to the destination layer
 *   fore     - Valid reference to the foreground layer
 *   forexpos - Valid selected x target position of the destination layer
 *   foreypos - Valid selected y target position of the destination layer
 *   back     - Valid reference to the background layer
 *   backarea - Valid reference to the selected area of the background layer
 *
 * Return:
 *    OK     - On success
 *   -EINVAL - On error
 *
 ******************************************************************************/

int stm32_dma2dblend(FAR struct stm32_ltdc_s *dest,
                      FAR struct stm32_ltdc_s *fore,
                      fb_coord_t forexpos, fb_coord_t foreypos,
                      FAR struct stm32_ltdc_s *back,
                      FAR const struct ltdc_area_s *backarea);

/******************************************************************************
 * Name: up_dma2dinitialize
 *
 * Description:
 *   Initialize the dma2d controller
 *
 * Return:
 *   OK - On success
 *   An error if initializing failed.
 *
 ******************************************************************************/

int up_dma2dinitialize(void);

/******************************************************************************
 * Name: up_dma2duninitialize
 *
 * Description:
 *   Uninitialize the dma2d controller
 *
 ******************************************************************************/

void up_dma2duninitialize(void);

#endif /* CONFIG_STM32_DMA2D */
#endif /* __ARCH_ARM_SRC_STM32_STM32_DMA2D_H */
