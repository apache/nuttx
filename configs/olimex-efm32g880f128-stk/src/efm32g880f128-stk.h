/****************************************************************************
 * configs/olimex-efm32g880f128-stk/src/efm32g880f128-stk.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

#ifndef __CONFIGS_OLIMEX_EFM32G880F128_STK_SRC_EFM32G880F128_STK_H
#define __CONFIGS_OLIMEX_EFM32G880F128_STK_SRC_EFM32G880F128_STK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Buttons:
 *
 * The Olimex board has four buttons, BUT1-4.  Each is grounded and so should
 * have a weak pull-up so that it will be sensed as "1" when open and "0"
 * when closed.
 *
 * --------------------- ---------------------
 * PIN                   CONNECTIONS
 * --------------------- ---------------------
 * PE0/PCNT0_S0IN/U0_TX  BUT1, EXT-18
 * PE1/PCNT0_S1IN/U0_RX  BUT2, EXT-19
 * PE2/ACMP0_O           BUT3, EXT-20
 * PE3/ACMP1_O           BUT4, EXT-21
 * --------------------- ---------------------
 */

#ifdef CONFIG_EFM32_GPIO_IRQ
#  define GPIO_BUTTON_1 (GPIO_INPUT_PULLUP|GPIO_INT_BOTH|GPIO_PORTE|GPIO_PIN0)
#  define GPIO_BUTTON_2 (GPIO_INPUT_PULLUP|GPIO_INT_BOTH|GPIO_PORTE|GPIO_PIN1)
#  define GPIO_BUTTON_3 (GPIO_INPUT_PULLUP|GPIO_INT_BOTH|GPIO_PORTE|GPIO_PIN2)
#  define GPIO_BUTTON_4 (GPIO_INPUT_PULLUP|GPIO_INT_BOTH|GPIO_PORTE|GPIO_PIN3)

#  define GPIO_IRQ_BUTTON_1 EFM32_IRQ_EXTI0
#  define GPIO_IRQ_BUTTON_2 EFM32_IRQ_EXTI1
#  define GPIO_IRQ_BUTTON_3 EFM32_IRQ_EXTI2
#  define GPIO_IRQ_BUTTON_4 EFM32_IRQ_EXTI3
#else
#  define GPIO_BUTTON_1 (GPIO_INPUT_PULLUP|GPIO_PORTE|GPIO_PIN0)
#  define GPIO_BUTTON_2 (GPIO_INPUT_PULLUP|GPIO_PORTE|GPIO_PIN1)
#  define GPIO_BUTTON_3 (GPIO_INPUT_PULLUP|GPIO_PORTE|GPIO_PIN2)
#  define GPIO_BUTTON_4 (GPIO_INPUT_PULLUP|GPIO_PORTE|GPIO_PIN3)
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __CONFIGS_EFM32_DK3650_INCLUDE_BOARD_H */
