/****************************************************************************
 * arch/arm/include/stm32f0l0/stm32f0_irq.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Alan Carvalho de Assis <acassis@gmail.com>
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

/* This file should never be included directed but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_STM32F0L0_STM32F0_IRQ_H
#define __ARCH_ARM_INCLUDE_STM32F0L0_STM32F0_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif
#include <arch/stm32f0l0/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Include STM32F0-specific external interrupt definitions */

#define STM32_IRQ_WWDG          (16) /* Vector 16: WWDG */
#define STM32_IRQ_PVD_VDDIO2    (17) /* Vector 17: PVD_VDDIO2 */
#define STM32_IRQ_RTC           (18) /* Vector 18: RTC */
#define STM32_IRQ_FLASH         (19) /* Vector 19: FLASH */
#define STM32_IRQ_RCC_CRS       (20) /* Vector 20: RCC and CRS */
#define STM32_IRQ_EXTI0_1       (21) /* Vector 21: EXTI0_1 */
#define STM32_IRQ_EXTI2_3       (22) /* Vector 22: EXTI2_3 */
#define STM32_IRQ_EXTI4_15      (23) /* Vector 23: EXTI4_15 */
#define STM32_IRQ_TSC           (24) /* Vector 24: TSC */
#define STM32_IRQ_DMA_CH1       (25) /* Vector 25: DMA_CH1 */
#define STM32_IRQ_DMA_CH23      (26) /* Vector 26: DMA_CH2_3 and DMA2_CH1_2 */
#define STM32_IRQ_DMA_CH4567    (27) /* Vector 27: DMA_CH4_5_6_7 and DMA2_CH3_4_5 */
#define STM32_IRQ_ADC_COMP      (28) /* Vector 28: ADC_COMP */
#define STM32_IRQ_TIM1_BRK      (29) /* Vector 29: TIM1_BRK_UP_TRG_COM */
#define STM32_IRQ_TIM1_CC       (30) /* Vector 30: TIM1_CC */
#define STM32_IRQ_TIM2          (31) /* Vector 31: TIM2 */
#define STM32_IRQ_TIM3          (32) /* Vector 32: TIM3 */
#define STM32_IRQ_TIM6_DAC      (33) /* Vector 33: TIM6 and DAC */
#define STM32_IRQ_TIM7          (34) /* Vector 34: TIM7 */
#define STM32_IRQ_TIM14         (35) /* Vector 35: TIM14 */
#define STM32_IRQ_TIM15         (36) /* Vector 36: TIM15 */
#define STM32_IRQ_TIM16         (37) /* Vector 37: TIM16 */
#define STM32_IRQ_TIM17         (38) /* Vector 38: TIM17 */
#define STM32_IRQ_I2C1          (39) /* Vector 39: I2C1 */
#define STM32_IRQ_I2C2          (40) /* Vector 40: I2C2 */
#define STM32_IRQ_SPI1          (41) /* Vector 41: SPI1 */
#define STM32_IRQ_SPI2          (42) /* Vector 42: SPI2 */
#define STM32_IRQ_USART1        (43) /* Vector 43: USART1 */
#define STM32_IRQ_USART2        (44) /* Vector 44: USART2 */
#define STM32_IRQ_USART345678   (45) /* Vector 45: USART3_4_5_6_7_8 */
#define STM32_IRQ_CEC_CAN       (46) /* Vector 46: HDMI CEC and CAN */
#define STM32_IRQ_USB           (47) /* Vector 47: USB */

#define STM32_IRQ_NEXTINT       (32) /* 32 external interrupts */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
typedef void (*vic_vector_t)(uint32_t *regs);

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_INCLUDE_STM32F0L0_STM32F0_IRQ_H */
