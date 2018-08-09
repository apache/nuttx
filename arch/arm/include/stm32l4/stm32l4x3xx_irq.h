/****************************************************************************************************
 * arch/arm/include/stm32l4/stm32l4x3xx_irq.h
 *
 *   Copyright (C) 2015 Sebastien Lorquet. All rights reserved.
 *   Authors: Sebastien Lorquet <sebastien@lorquet.fr>
 *            Juha Niskanen <juha.niskanen@haltian.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
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
 ****************************************************************************************************/

/* This file should never be included directed but, rather, only indirectly through arch/irq.h */

#ifndef __ARCH_ARM_INCLUDE_STM32L4_STM32L4X3XX_IRQ_H
#define __ARCH_ARM_INCLUDE_STM32L4_STM32L4X3XX_IRQ_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* IRQ numbers.  The IRQ number corresponds vector number and hence map directly to bits in the
 * NVIC.  This does, however, waste several words of memory in the IRQ to handle mapping tables.
 *
 * Processor Exceptions (vectors 0-15).  These common definitions can be found in the file
 * nuttx/arch/arm/include/stm32l4/irq.h which includes this file
 *
 * External interrupts (vectors >= 16)
 *
 * These interrupts vectors was implemented based on RM0394 Table 45 and should work for
 * STM32L431xx, STM32L451xx, STM32L4X2 and STM32L4X3.
 *
 */

#define STM32L4_IRQ_WWDG        (STM32L4_IRQ_FIRST + 0)  /* 0:  Window Watchdog interrupt */
#define STM32L4_IRQ_PVD         (STM32L4_IRQ_FIRST + 1)  /* 1:  PVD through EXTI Line detection interrupt */
#define STM32L4_IRQ_TAMPER      (STM32L4_IRQ_FIRST + 2)  /* 2:  Tamper and time stamp interrupts */
#define STM32L4_IRQ_TIMESTAMP   (STM32L4_IRQ_FIRST + 2)  /* 2:  Tamper and time stamp interrupts */
#define STM32L4_IRQ_RTC_WKUP    (STM32L4_IRQ_FIRST + 3)  /* 3:  RTC global interrupt */
#define STM32L4_IRQ_FLASH       (STM32L4_IRQ_FIRST + 4)  /* 4:  Flash global interrupt */
#define STM32L4_IRQ_RCC         (STM32L4_IRQ_FIRST + 5)  /* 5:  RCC global interrupt */
#define STM32L4_IRQ_EXTI0       (STM32L4_IRQ_FIRST + 6)  /* 6:  EXTI Line 0 interrupt */
#define STM32L4_IRQ_EXTI1       (STM32L4_IRQ_FIRST + 7)  /* 7:  EXTI Line 1 interrupt */
#define STM32L4_IRQ_EXTI2       (STM32L4_IRQ_FIRST + 8)  /* 8:  EXTI Line 2 interrupt */
#define STM32L4_IRQ_EXTI3       (STM32L4_IRQ_FIRST + 9)  /* 9:  EXTI Line 3 interrupt */
#define STM32L4_IRQ_EXTI4       (STM32L4_IRQ_FIRST + 10) /* 10: EXTI Line 4 interrupt */
#define STM32L4_IRQ_DMA1CH1     (STM32L4_IRQ_FIRST + 11) /* 11: DMA1 Channel 1 global interrupt */
#define STM32L4_IRQ_DMA1CH2     (STM32L4_IRQ_FIRST + 12) /* 12: DMA1 Channel 2 global interrupt */
#define STM32L4_IRQ_DMA1CH3     (STM32L4_IRQ_FIRST + 13) /* 13: DMA1 Channel 3 global interrupt */
#define STM32L4_IRQ_DMA1CH4     (STM32L4_IRQ_FIRST + 14) /* 14: DMA1 Channel 4 global interrupt */
#define STM32L4_IRQ_DMA1CH5     (STM32L4_IRQ_FIRST + 15) /* 15: DMA1 Channel 5 global interrupt */
#define STM32L4_IRQ_DMA1CH6     (STM32L4_IRQ_FIRST + 16) /* 16: DMA1 Channel 6 global interrupt */
#define STM32L4_IRQ_DMA1CH7     (STM32L4_IRQ_FIRST + 17) /* 17: DMA1 Channel 7 global interrupt */
#define STM32L4_IRQ_ADC1        (STM32L4_IRQ_FIRST + 18) /* 18: ADC1 global interrupt */
#define STM32L4_IRQ_CAN1TX      (STM32L4_IRQ_FIRST + 19) /* 19: CAN1 TX interrupts */
#define STM32L4_IRQ_CAN1RX0     (STM32L4_IRQ_FIRST + 20) /* 20: CAN1 RX0 interrupts */
#define STM32L4_IRQ_CAN1RX1     (STM32L4_IRQ_FIRST + 21) /* 21: CAN1 RX1 interrupt */
#define STM32L4_IRQ_CAN1SCE     (STM32L4_IRQ_FIRST + 22) /* 22: CAN1 SCE interrupt */
#define STM32L4_IRQ_EXTI95      (STM32L4_IRQ_FIRST + 23) /* 23: EXTI Line[9:5] interrupts */
#define STM32L4_IRQ_TIM1BRK     (STM32L4_IRQ_FIRST + 24) /* 24: TIM1 Break interrupt */
#define STM32L4_IRQ_TIM15       (STM32L4_IRQ_FIRST + 24) /* 24: TIM15 global interrupt */
#define STM32L4_IRQ_TIM1UP      (STM32L4_IRQ_FIRST + 25) /* 25: TIM1 Update interrupt */
#define STM32L4_IRQ_TIM16       (STM32L4_IRQ_FIRST + 25) /* 25: TIM16 global interrupt */
#define STM32L4_IRQ_TIM1TRGCOM  (STM32L4_IRQ_FIRST + 26) /* 26: TIM1 Trigger and Commutation interrupts */
#define STM32L4_IRQ_TIM1CC      (STM32L4_IRQ_FIRST + 27) /* 27: TIM1 Capture Compare interrupt */
#define STM32L4_IRQ_TIM2        (STM32L4_IRQ_FIRST + 28) /* 28: TIM2 global interrupt */
#define STM32L4_IRQ_TIM3        (STM32L4_IRQ_FIRST + 29) /* 29: TIM3 global interrupt */
/* Reserved */                                           /* 30: TIM4 global interrupt */
#define STM32L4_IRQ_I2C1EV      (STM32L4_IRQ_FIRST + 31) /* 31: I2C1 event interrupt */
#define STM32L4_IRQ_I2C1ER      (STM32L4_IRQ_FIRST + 32) /* 32: I2C1 error interrupt */
#define STM32L4_IRQ_I2C2EV      (STM32L4_IRQ_FIRST + 33) /* 33: I2C2 event interrupt */
#define STM32L4_IRQ_I2C2ER      (STM32L4_IRQ_FIRST + 34) /* 34: I2C2 error interrupt */
#define STM32L4_IRQ_SPI1        (STM32L4_IRQ_FIRST + 35) /* 35: SPI1 global interrupt */
#define STM32L4_IRQ_SPI2        (STM32L4_IRQ_FIRST + 36) /* 36: SPI2 global interrupt */
#define STM32L4_IRQ_USART1      (STM32L4_IRQ_FIRST + 37) /* 37: USART1 global interrupt */
#define STM32L4_IRQ_USART2      (STM32L4_IRQ_FIRST + 38) /* 38: USART2 global interrupt */
#define STM32L4_IRQ_USART3      (STM32L4_IRQ_FIRST + 39) /* 39: USART3 global interrupt */
#define STM32L4_IRQ_EXTI1510    (STM32L4_IRQ_FIRST + 40) /* 40: EXTI Line[15:10] interrupts */
#define STM32L4_IRQ_RTCALRM     (STM32L4_IRQ_FIRST + 41) /* 41: RTC alarm through EXTI line interrupt */
/* Reserved */                                           /* 42-48: reserved */
#define STM32L4_IRQ_SDMMC1      (STM32L4_IRQ_FIRST + 49) /* 49: SDMMC1 global interrupt */
/* Reserved */                                           /* 50: TIM5 global interrupt */
#define STM32L4_IRQ_SPI3        (STM32L4_IRQ_FIRST + 51) /* 51: SPI3 global interrupt */
#define STM32L4_IRQ_UART4       (STM32L4_IRQ_FIRST + 52) /* 52: UART4 global interrupt */
/* Reserved */                                         /* 53: UART5 global interrupt */
#define STM32L4_IRQ_TIM6        (STM32L4_IRQ_FIRST + 54) /* 54: TIM6 global interrupt */
#define STM32L4_IRQ_DAC         (STM32L4_IRQ_FIRST + 54) /* 54: DAC1 underrun error interrupts */
#define STM32L4_IRQ_TIM7        (STM32L4_IRQ_FIRST + 55) /* 55: TIM7 global interrupt */
#define STM32L4_IRQ_DMA2CH1     (STM32L4_IRQ_FIRST + 56) /* 56: DMA2 Channel 1 global interrupt */
#define STM32L4_IRQ_DMA2CH2     (STM32L4_IRQ_FIRST + 57) /* 57: DMA2 Channel 2 global interrupt */
#define STM32L4_IRQ_DMA2CH3     (STM32L4_IRQ_FIRST + 58) /* 58: DMA2 Channel 3 global interrupt */
#define STM32L4_IRQ_DMA2CH4     (STM32L4_IRQ_FIRST + 59) /* 59: DMA2 Channel 4 global interrupt */
#define STM32L4_IRQ_DMA2CH5     (STM32L4_IRQ_FIRST + 60) /* 60: DMA2 Channel 5 global interrupt */
#define STM32L4_IRQ_DFSDM0      (STM32L4_IRQ_FIRST + 61) /* 61: DFSDM0 global interrupt */
#define STM32L4_IRQ_DFSDM1      (STM32L4_IRQ_FIRST + 62) /* 62: DFSDM1 global interrupt*/
/* Reserved */                                           /* 63: DFSDM2 global interrupt */
#define STM32L4_IRQ_COMP        (STM32L4_IRQ_FIRST + 64) /* 64: COMP1/COMP2 interrupts */
#define STM32L4_IRQ_LPTIM1      (STM32L4_IRQ_FIRST + 65) /* 65: LPTIM1 global interrupt */
#define STM32L4_IRQ_LPTIM2      (STM32L4_IRQ_FIRST + 66) /* 66: LPTIM2 global interrupt */
#define STM32L4_IRQ_USB_FS      (STM32L4_IRQ_FIRST + 67) /* 67: USB event interrupt through EXTI line 17 */
#define STM32L4_IRQ_DMA2CH6     (STM32L4_IRQ_FIRST + 68) /* 68: DMA2 Channel 6 global interrupt */
#define STM32L4_IRQ_DMA2CH7     (STM32L4_IRQ_FIRST + 69) /* 69: DMA2 Channel 7 global interrupt */
#define STM32L4_IRQ_LPUART1     (STM32L4_IRQ_FIRST + 70) /* 70: Low power UART 1 global interrupt */
#define STM32L4_IRQ_QUADSPI     (STM32L4_IRQ_FIRST + 71) /* 71: QUADSPI global interrupt */
#define STM32L4_IRQ_I2C3EV      (STM32L4_IRQ_FIRST + 72) /* 72: I2C3 event interrupt */
#define STM32L4_IRQ_I2C3ER      (STM32L4_IRQ_FIRST + 73) /* 73: I2C3 error interrupt */
#define STM32L4_IRQ_SAI1        (STM32L4_IRQ_FIRST + 74) /* 74: SAI1 global interrupt */
/* Reserved */                                           /* 75: SAI2 global interrupt */
#define STM32L4_IRQ_SWPMI1      (STM32L4_IRQ_FIRST + 76) /* 76: SWPMI1 global interrupt */
#define STM32L4_IRQ_TSC         (STM32L4_IRQ_FIRST + 77) /* 77: TSC global interrupt */
#define STM32L4_IRQ_LCD         (STM32L4_IRQ_FIRST + 78) /* 78: LCD global interrupt */
#define STM32L4_IRQ_AES         (STM32L4_IRQ_FIRST + 79) /* 79: AES crypto global interrupt */
#define STM32L4_IRQ_RNG         (STM32L4_IRQ_FIRST + 80) /* 80: RNG global interrupt */
#define STM32L4_IRQ_FPU         (STM32L4_IRQ_FIRST + 81) /* 81: FPU global interrupt */
#define STM32L4_IRQ_CRS         (STM32L4_IRQ_FIRST + 82) /* 82: CRS global interrupt */
#define STM32L4_IRQ_I2C4EV      (STM32L4_IRQ_FIRST + 83) /* 83: I2C4 event interrupt */
#define STM32L4_IRQ_I2C4ER      (STM32L4_IRQ_FIRST + 84) /* 84: I2C4 error interrupt */

#if defined(CONFIG_STM32L4_STM32L4X3)
#  define STM32L4_IRQ_NEXTINTS  85
#else
#  error "Unsupported STM32L4 chip"
#endif

/* (EXTI interrupts do not use IRQ numbers) */

#define NR_IRQS                 (STM32L4_IRQ_FIRST + STM32L4_IRQ_NEXTINTS)

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_STM32L4_STM32L4X3XX_IRQ_H */
