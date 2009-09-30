/************************************************************************************
 * configs/stm3210e_eval/src/stm3210e_internal.h
 * arch/arm/src/board/stm3210e_internal.n
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __CONFIGS_STM3210E_EVAL_SRC_STM3210E_INTERNAL_H
#define __CONFIGS_STM3210E_EVAL_SRC_STM3210E_INTERNAL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <sys/types.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* How many SPI modules does this chip support? The LM3S6918 supports 2 SPI
 * modules (others may support more -- in such case, the following must be
 * expanded).
 */

#if STM32_NSPI < 1
#  undef CONFIG_STM32_SPI1
#  undef CONFIG_STM32_SPI2
#elif STM32_NSPI < 2
#  undef CONFIG_STM32_SPI2
#endif

/* STM3210E-EVAL GPIOs **************************************************************/

/* LEDs */

#define GPIO_LED1 (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz| GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN6)
#define GPIO_LED2 (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz| GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN7)
#define GPIO_LED3 (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz| GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN8)
#define GPIO_LED4 (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz| GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN9)

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the STM3210E-EVAL board.
 *
 ************************************************************************************/

extern void weak_function stm32_spiinitialize(void);

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_STM3210E_EVAL_SRC_STM3210E_INTERNAL_H */

