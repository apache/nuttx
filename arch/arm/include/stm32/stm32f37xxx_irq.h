/****************************************************************************************************
 * arch/arm/include/stm32/stm32f37xxx_irq.h
 *
 *   Copyright (C) 2012, 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *   Modified for STM32F373 by Marten Svanfeldt <marten@svanfeldt.com>
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
 ****************************************************************************************************/

/* This file should never be included directly but, rather, only indirectly through nuttx/irq.h */

#ifndef __ARCH_ARM_INCLUDE_STM32_STM32F37XXX_IRQ_H
#define __ARCH_ARM_INCLUDE_STM32_STM32F37XXX_IRQ_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* IRQ numbers.  The IRQ number corresponds vector number and hence map directly to
 * bits in the NVIC.  This does, however, waste several words of memory in the IRQ
 * to handle mapping tables.
 *
 * Processor Exceptions (vectors 0-15).  These common definitions can be found
 * in nuttx/arch/arm/include/stm32/irq.h
 *
 * External interrupts (vectors >= 16)
 */

#define STM32_IRQ_WWDG        (STM32_IRQ_FIRST+0)  /* 0:  Window Watchdog interrupt */
#define STM32_IRQ_PVD         (STM32_IRQ_FIRST+1)  /* 1:  PVD through EXTI Line detection interrupt */
#define STM32_IRQ_TAMPER      (STM32_IRQ_FIRST+2)  /* 2:  Tamper interrupt, or */
#define STM32_IRQ_TIMESTAMP   (STM32_IRQ_FIRST+2)  /* 2:  Time stamp interrupt */
#define STM32_IRQ_RTC_WKUP    (STM32_IRQ_FIRST+3)  /* 3:  RTC global interrupt */
#define STM32_IRQ_FLASH       (STM32_IRQ_FIRST+4)  /* 4:  Flash global interrupt */
#define STM32_IRQ_RCC         (STM32_IRQ_FIRST+5)  /* 5:  RCC global interrupt */
#define STM32_IRQ_EXTI0       (STM32_IRQ_FIRST+6)  /* 6:  EXTI Line 0 interrupt */
#define STM32_IRQ_EXTI1       (STM32_IRQ_FIRST+7)  /* 7:  EXTI Line 1 interrupt */
#define STM32_IRQ_EXTI2       (STM32_IRQ_FIRST+8)  /* 8:  EXTI Line 2 interrupt, or */
#define STM32_IRQ_TSC         (STM32_IRQ_FIRST+8)  /* 8:  TSC interrupt */
#define STM32_IRQ_EXTI3       (STM32_IRQ_FIRST+9)  /* 9:  EXTI Line 3 interrupt */
#define STM32_IRQ_EXTI4       (STM32_IRQ_FIRST+10) /* 10: EXTI Line 4 interrupt */
#define STM32_IRQ_DMA1CH1     (STM32_IRQ_FIRST+11) /* 11: DMA1 channel 1 global interrupt */
#define STM32_IRQ_DMA1CH2     (STM32_IRQ_FIRST+12) /* 12: DMA1 channel 2 global interrupt */
#define STM32_IRQ_DMA1CH3     (STM32_IRQ_FIRST+13) /* 13: DMA1 channel 3 global interrupt */
#define STM32_IRQ_DMA1CH4     (STM32_IRQ_FIRST+14) /* 14: DMA1 channel 4 global interrupt */
#define STM32_IRQ_DMA1CH5     (STM32_IRQ_FIRST+15) /* 15: DMA1 channel 5 global interrupt */
#define STM32_IRQ_DMA1CH6     (STM32_IRQ_FIRST+16) /* 16: DMA1 channel 6 global interrupt */
#define STM32_IRQ_DMA1CH7     (STM32_IRQ_FIRST+17) /* 17: DMA1 channel 7 global interrupt */
#define STM32_IRQ_ADC1        (STM32_IRQ_FIRST+18) /* 18: ADC1 global interrupt */
#define STM32_IRQ_CAN1TX      (STM32_IRQ_FIRST+19) /* 19: CAN1 TX interrupts */
#define STM32_IRQ_CAN1RX0     (STM32_IRQ_FIRST+20) /* 20: CAN1 RX0 interrupts*/
#define STM32_IRQ_CAN1RX1     (STM32_IRQ_FIRST+21) /* 21: CAN1 RX1 interrupt */
#define STM32_IRQ_CAN1SCE     (STM32_IRQ_FIRST+22) /* 22: CAN1 SCE interrupt */
#define STM32_IRQ_EXTI95      (STM32_IRQ_FIRST+23) /* 23: EXTI Line[9:5] interrupts */
#define STM32_IRQ_TIM15       (STM32_IRQ_FIRST+24) /* 24: TIM15 global interrupt */
#define STM32_IRQ_TIM16       (STM32_IRQ_FIRST+25) /* 25: TIM16 global interrupt */
#define STM32_IRQ_TIM17       (STM32_IRQ_FIRST+26) /* 26: TIM17 global interrupt */
#define STM32_IRQ_TIM18       (STM32_IRQ_FIRST+27) /* 27: TIM18 global interrupt, or */
#define STM32_IRQ_DAC2        (STM32_IRQ_FIRST+27) /* 27: DAC2 global interrupt */
#define STM32_IRQ_TIM2        (STM32_IRQ_FIRST+28) /* 28: TIM2 global interrupt */
#define STM32_IRQ_TIM3        (STM32_IRQ_FIRST+29) /* 29: TIM3 global interrupt */
#define STM32_IRQ_TIM4        (STM32_IRQ_FIRST+30) /* 30: TIM4 global interrupt */
#define STM32_IRQ_I2C1EV      (STM32_IRQ_FIRST+31) /* 31: I2C1 event interrupt */
#define STM32_IRQ_I2C1ER      (STM32_IRQ_FIRST+32) /* 32: I2C1 error interrupt */
#define STM32_IRQ_I2C2EV      (STM32_IRQ_FIRST+33) /* 33: I2C2 event interrupt */
#define STM32_IRQ_I2C2ER      (STM32_IRQ_FIRST+34) /* 34: I2C2 error interrupt */
#define STM32_IRQ_SPI1        (STM32_IRQ_FIRST+35) /* 35: SPI1 global interrupt */
#define STM32_IRQ_SPI2        (STM32_IRQ_FIRST+36) /* 36: SPI2 global interrupt */
#define STM32_IRQ_USART1      (STM32_IRQ_FIRST+37) /* 37: USART1 global interrupt */
#define STM32_IRQ_USART2      (STM32_IRQ_FIRST+38) /* 38: USART2 global interrupt */
#define STM32_IRQ_USART3      (STM32_IRQ_FIRST+39) /* 39: USART3 global interrupt */
#define STM32_IRQ_EXTI1510    (STM32_IRQ_FIRST+40) /* 40: EXTI Line[15:10] interrupts */
#define STM32_IRQ_RTCALRM     (STM32_IRQ_FIRST+41) /* 41: RTC alarm through EXTI line interrupt */
#define STM32_IRQ_CEC         (STM32_IRQ_FIRST+42) /* 42: CEC Interrupt */
#define STM32_IRQ_TIM12       (STM32_IRQ_FIRST+43) /* 43: TIM12 global interrupt */
#define STM32_IRQ_TIM13       (STM32_IRQ_FIRST+44) /* 44: TIM13 global interrupt */
#define STM32_IRQ_TIM14       (STM32_IRQ_FIRST+45) /* 45: TIM14 global interrupt */
#define STM32_IRQ_RESERVED46  (STM32_IRQ_FIRST+46) /* 46: Reserved */
#define STM32_IRQ_RESERVED47  (STM32_IRQ_FIRST+47) /* 47: Reserved */
#define STM32_IRQ_RESERVED48  (STM32_IRQ_FIRST+48) /* 48: Reserved */
#define STM32_IRQ_RESERVED49  (STM32_IRQ_FIRST+49) /* 49: Reserved */
#define STM32_IRQ_RESERVED50  (STM32_IRQ_FIRST+50) /* 50: Reserved */
#define STM32_IRQ_SPI3        (STM32_IRQ_FIRST+51) /* 51: SPI3 global interrupt */
#define STM32_IRQ_RESERVED52  (STM32_IRQ_FIRST+52) /* 52: Reserved */
#define STM32_IRQ_RESERVED53  (STM32_IRQ_FIRST+53) /* 53: Reserved */
#define STM32_IRQ_TIM6        (STM32_IRQ_FIRST+54) /* 54: TIM6 global interrupt, or */
#define STM32_IRQ_DAC1        (STM32_IRQ_FIRST+54) /* 54: DAC1 underrun error interrupts */
#define STM32_IRQ_TIM7        (STM32_IRQ_FIRST+55) /* 55: TIM7 global interrupt */
#define STM32_IRQ_DMA2CH1     (STM32_IRQ_FIRST+56) /* 56: DMA2 channel 1 global interrupt */
#define STM32_IRQ_DMA2CH2     (STM32_IRQ_FIRST+57) /* 57: DMA2 channel 2 global interrupt */
#define STM32_IRQ_DMA2CH3     (STM32_IRQ_FIRST+58) /* 58: DMA2 channel 3 global interrupt */
#define STM32_IRQ_DMA2CH4     (STM32_IRQ_FIRST+59) /* 59: DMA2 channel 4 global interrupt */
#define STM32_IRQ_DMA2CH5     (STM32_IRQ_FIRST+60) /* 60: DMA2 channel 5 global interrupt */
#define STM32_IRQ_SDADC1      (STM32_IRQ_FIRST+61) /* 61: ADC Sigma Delta 1 global interrupt */
#define STM32_IRQ_SDADC2      (STM32_IRQ_FIRST+62) /* 62: ADC Sigma Delta 2 global interrupt */
#define STM32_IRQ_SDADC3      (STM32_IRQ_FIRST+63) /* 63: ADC Sigma Delta 3 global interrupt */
#define STM32_IRQ_COMP12      (STM32_IRQ_FIRST+64) /* 64: COMP1 & COMP2 interrupts*/
#define STM32_IRQ_RESERVED65  (STM32_IRQ_FIRST+65) /* 65: Reserved */
#define STM32_IRQ_RESERVED66  (STM32_IRQ_FIRST+66) /* 66: Reserved */
#define STM32_IRQ_RESERVED67  (STM32_IRQ_FIRST+67) /* 67: Reserved */
#define STM32_IRQ_RESERVED68  (STM32_IRQ_FIRST+68) /* 68: Reserved */
#define STM32_IRQ_RESERVED69  (STM32_IRQ_FIRST+69) /* 69: Reserved */
#define STM32_IRQ_RESERVED70  (STM32_IRQ_FIRST+70) /* 70: Reserved */
#define STM32_IRQ_RESERVED71  (STM32_IRQ_FIRST+71) /* 71: Reserved */
#define STM32_IRQ_RESERVED72  (STM32_IRQ_FIRST+72) /* 72: Reserved */
#define STM32_IRQ_RESERVED73  (STM32_IRQ_FIRST+73) /* 73: Reserved */
#define STM32_IRQ_USBHP       (STM32_IRQ_FIRST+74) /* 74: USB High priority interrupt */
#define STM32_IRQ_USBLP       (STM32_IRQ_FIRST+75) /* 75: USB Low priority interrupt */
#define STM32_IRQ_USBWKUP     (STM32_IRQ_FIRST+76) /* 76: USB wakeup from suspend */
#define STM32_IRQ_RESERVED77  (STM32_IRQ_FIRST+77) /* 77: Reserved */
#define STM32_IRQ_RESERVED78  (STM32_IRQ_FIRST+78) /* 78: Reserved */
#define STM32_IRQ_RESERVED79  (STM32_IRQ_FIRST+79) /* 79: Reserved */
#define STM32_IRQ_RESERVED80  (STM32_IRQ_FIRST+80) /* 80: Reserved */
#define STM32_IRQ_FPU         (STM32_IRQ_FIRST+81) /* 81: FPU global interrupt */

#define STM32_IRQ_NEXTINT     (82)
#define NR_IRQS               (STM32_IRQ_FIRST+82)

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

#endif /* __ARCH_ARM_INCLUDE_STM32F30XXX_IRQ_H */
