/****************************************************************************************************
 * arch/arm/include/stm32/stm32f20xxx_irq.h
 *
 *   Copyright (C) 2012, 2018 Gregory Nutt. All rights reserved.
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
 ****************************************************************************************************/

/* This file should never be included directly but, rather, only indirectly through nuttx/irq.h */

#ifndef __ARCH_ARM_INCLUDE_STM32_STM32F20XXX_IRQ_H
#define __ARCH_ARM_INCLUDE_STM32_STM32F20XXX_IRQ_H

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
#define STM32_IRQ_TAMPER      (STM32_IRQ_FIRST+2)  /* 2:  Tamper and time stamp interrupts */
#define STM32_IRQ_TIMESTAMP   (STM32_IRQ_FIRST+2)  /* 2:  Tamper and time stamp interrupts */
#define STM32_IRQ_RTC_WKUP    (STM32_IRQ_FIRST+3)  /* 3:  RTC global interrupt */
#define STM32_IRQ_FLASH       (STM32_IRQ_FIRST+4)  /* 4:  Flash global interrupt */
#define STM32_IRQ_RCC         (STM32_IRQ_FIRST+5)  /* 5:  RCC global interrupt */
#define STM32_IRQ_EXTI0       (STM32_IRQ_FIRST+6)  /* 6:  EXTI Line 0 interrupt */
#define STM32_IRQ_EXTI1       (STM32_IRQ_FIRST+7)  /* 7:  EXTI Line 1 interrupt */
#define STM32_IRQ_EXTI2       (STM32_IRQ_FIRST+8)  /* 8:  EXTI Line 2 interrupt */
#define STM32_IRQ_EXTI3       (STM32_IRQ_FIRST+9)  /* 9:  EXTI Line 3 interrupt */
#define STM32_IRQ_EXTI4       (STM32_IRQ_FIRST+10) /* 10: EXTI Line 4 interrupt */
#define STM32_IRQ_DMA1S0      (STM32_IRQ_FIRST+11) /* 11: DMA1 Stream 0 global interrupt */
#define STM32_IRQ_DMA1S1      (STM32_IRQ_FIRST+12) /* 12: DMA1 Stream 1 global interrupt */
#define STM32_IRQ_DMA1S2      (STM32_IRQ_FIRST+13) /* 13: DMA1 Stream 2 global interrupt */
#define STM32_IRQ_DMA1S3      (STM32_IRQ_FIRST+14) /* 14: DMA1 Stream 3 global interrupt */
#define STM32_IRQ_DMA1S4      (STM32_IRQ_FIRST+15) /* 15: DMA1 Stream 4 global interrupt */
#define STM32_IRQ_DMA1S5      (STM32_IRQ_FIRST+16) /* 16: DMA1 Stream 5 global interrupt */
#define STM32_IRQ_DMA1S6      (STM32_IRQ_FIRST+17) /* 17: DMA1 Stream 6 global interrupt */
#define STM32_IRQ_ADC         (STM32_IRQ_FIRST+18) /* 18: ADC1, ADC2, and ADC3 global interrupt */
#define STM32_IRQ_CAN1TX      (STM32_IRQ_FIRST+19) /* 19: CAN1 TX interrupts */
#define STM32_IRQ_CAN1RX0     (STM32_IRQ_FIRST+20) /* 20: CAN1 RX0 interrupts */
#define STM32_IRQ_CAN1RX1     (STM32_IRQ_FIRST+21) /* 21: CAN1 RX1 interrupt */
#define STM32_IRQ_CAN1SCE     (STM32_IRQ_FIRST+22) /* 22: CAN1 SCE interrupt */
#define STM32_IRQ_EXTI95      (STM32_IRQ_FIRST+23) /* 23: EXTI Line[9:5] interrupts */
#define STM32_IRQ_TIM1BRK     (STM32_IRQ_FIRST+24) /* 24: TIM1 Break interrupt */
#define STM32_IRQ_TIM9        (STM32_IRQ_FIRST+24) /* 24: TIM9 global interrupt */
#define STM32_IRQ_TIM1UP      (STM32_IRQ_FIRST+25) /* 25: TIM1 Update interrupt */
#define STM32_IRQ_TIM10       (STM32_IRQ_FIRST+25) /* 25: TIM10 global interrupt */
#define STM32_IRQ_TIM1TRGCOM  (STM32_IRQ_FIRST+26) /* 26: TIM1 Trigger and Commutation interrupts */
#define STM32_IRQ_TIM11       (STM32_IRQ_FIRST+26) /* 26: TIM11 global interrupt */
#define STM32_IRQ_TIM1CC      (STM32_IRQ_FIRST+27) /* 27: TIM1 Capture Compare interrupt */
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
#define STM32_IRQ_OTGFSWKUP   (STM32_IRQ_FIRST+42) /* 42: USB On-The-Go FS Wakeup through EXTI line interrupt */
#define STM32_IRQ_TIM8BRK     (STM32_IRQ_FIRST+43) /* 43: TIM8 Break interrupt */
#define STM32_IRQ_TIM12       (STM32_IRQ_FIRST+43) /* 43: TIM12 global interrupt */
#define STM32_IRQ_TIM8UP      (STM32_IRQ_FIRST+44) /* 44: TIM8 Update interrupt */
#define STM32_IRQ_TIM13       (STM32_IRQ_FIRST+44) /* 44: TIM13 global interrupt */
#define STM32_IRQ_TIM8TRGCOM  (STM32_IRQ_FIRST+45) /* 45: TIM8 Trigger and Commutation interrupts */
#define STM32_IRQ_TIM14       (STM32_IRQ_FIRST+45) /* 45: TIM14 global interrupt */
#define STM32_IRQ_TIM8CC      (STM32_IRQ_FIRST+46) /* 46: TIM8 Capture Compare interrupt */
#define STM32_IRQ_DMA1S7      (STM32_IRQ_FIRST+47) /* 47: DMA1 Stream 7 global interrupt */
#define STM32_IRQ_FSMC        (STM32_IRQ_FIRST+48) /* 48: FSMC global interrupt */
#define STM32_IRQ_SDIO        (STM32_IRQ_FIRST+49) /* 49: SDIO global interrupt */
#define STM32_IRQ_TIM5        (STM32_IRQ_FIRST+50) /* 50: TIM5 global interrupt */
#define STM32_IRQ_SPI3        (STM32_IRQ_FIRST+51) /* 51: SPI3 global interrupt */
#define STM32_IRQ_UART4       (STM32_IRQ_FIRST+52) /* 52: UART4 global interrupt */
#define STM32_IRQ_UART5       (STM32_IRQ_FIRST+53) /* 53: UART5 global interrupt */
#define STM32_IRQ_TIM6        (STM32_IRQ_FIRST+54) /* 54: TIM6 global interrupt */
#define STM32_IRQ_DAC         (STM32_IRQ_FIRST+54) /* 54: DAC1 and DAC2 underrun error interrupts */
#define STM32_IRQ_TIM7        (STM32_IRQ_FIRST+55) /* 55: TIM7 global interrupt */
#define STM32_IRQ_DMA2S0      (STM32_IRQ_FIRST+56) /* 56: DMA2 Stream 0 global interrupt */
#define STM32_IRQ_DMA2S1      (STM32_IRQ_FIRST+57) /* 57: DMA2 Stream 1 global interrupt */
#define STM32_IRQ_DMA2S2      (STM32_IRQ_FIRST+58) /* 58: DMA2 Stream 2 global interrupt */
#define STM32_IRQ_DMA2S3      (STM32_IRQ_FIRST+59) /* 59: DMA2 Stream 3 global interrupt */
#define STM32_IRQ_DMA2S4      (STM32_IRQ_FIRST+60) /* 60: DMA2 Stream 4 global interrupt */
#define STM32_IRQ_ETH         (STM32_IRQ_FIRST+61) /* 61: Ethernet global interrupt */
#define STM32_IRQ_ETHWKUP     (STM32_IRQ_FIRST+62) /* 62: Ethernet Wakeup through EXTI line interrupt */
#define STM32_IRQ_CAN2TX      (STM32_IRQ_FIRST+63) /* 63: CAN2 TX interrupts */
#define STM32_IRQ_CAN2RX0     (STM32_IRQ_FIRST+64) /* 64: CAN2 RX0 interrupts */
#define STM32_IRQ_CAN2RX1     (STM32_IRQ_FIRST+65) /* 65: CAN2 RX1 interrupt */
#define STM32_IRQ_CAN2SCE     (STM32_IRQ_FIRST+66) /* 66: CAN2 SCE interrupt */
#define STM32_IRQ_OTGFS       (STM32_IRQ_FIRST+67) /* 67: USB On The Go FS global interrupt */
#define STM32_IRQ_DMA2S5      (STM32_IRQ_FIRST+68) /* 68: DMA2 Stream 5 global interrupt */
#define STM32_IRQ_DMA2S6      (STM32_IRQ_FIRST+69) /* 69: DMA2 Stream 6 global interrupt */
#define STM32_IRQ_DMA2S7      (STM32_IRQ_FIRST+70) /* 70: DMA2 Stream 7 global interrupt */
#define STM32_IRQ_USART6      (STM32_IRQ_FIRST+71) /* 71: USART6 global interrupt */
#define STM32_IRQ_I2C3EV      (STM32_IRQ_FIRST+72) /* 72: I2C3 event interrupt */
#define STM32_IRQ_I2C3ER      (STM32_IRQ_FIRST+73) /* 73: I2C3 error interrupt */
#define STM32_IRQ_OTGHSEP1OUT (STM32_IRQ_FIRST+74) /* 74: USB On The Go HS End Point 1 Out global interrupt */
#define STM32_IRQ_OTGHSEP1IN  (STM32_IRQ_FIRST+75) /* 75: USB On The Go HS End Point 1 In global interrupt */
#define STM32_IRQ_OTGHSWKUP   (STM32_IRQ_FIRST+76) /* 76: USB On The Go HS Wakeup through EXTI interrupt */
#define STM32_IRQ_OTGHS       (STM32_IRQ_FIRST+77) /* 77: USB On The Go HS global interrupt */
#define STM32_IRQ_DCMI        (STM32_IRQ_FIRST+78) /* 78: DCMI global interrupt */
#define STM32_IRQ_CRYP        (STM32_IRQ_FIRST+79) /* 79: CRYP crypto global interrupt */
#define STM32_IRQ_HASH        (STM32_IRQ_FIRST+80) /* 80: Hash and Rng global interrupt */
#define STM32_IRQ_RNG         (STM32_IRQ_FIRST+80) /* 80: Hash and Rng global interrupt */

#define STM32_IRQ_NEXTINT     (81)
#define NR_IRQS               (STM32_IRQ_FIRST+81)

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

#endif /* __ARCH_ARM_INCLUDE_STM32_STM32F20XXX_IRQ_H */
