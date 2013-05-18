/****************************************************************************************************
 * arch/arm/include/stm32s/stm32l15xxx_irq.h
 * For STM32L100xx, STM32L151xx, STM32L152xx and STM32L162xx advanced ARM-based 32-bit MCUs
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
 ****************************************************************************************************/

/* This file should never be included directed but, rather, only indirectly through nuttx/irq.h */

#ifndef __ARCH_ARM_INCLUDE_STM32L15XXX_IRQ_H
#define __ARCH_ARM_INCLUDE_STM32L15XXX_IRQ_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

/* IRQ numbers.  The IRQ number corresponds vector number and hence map directly to
 * bits in the NVIC.  This does, however, waste several words of memory in the IRQ
 * to handle mapping tables.
 *
 * Processor Exceptions (vectors 0-15).  These common definitions can be found
 * in nuttx/arch/arm/include/stm32/irq.h
 *
 * External interrupts (vectors >= 16) for low and medium density devices
 */

#if defined(CONFIG_STM32_LOWDENSITY) || defined(CONFIG_STM32_MEDIUMDENSITY)
#  define STM32_IRQ_WWDG        (STM32_IRQ_INTERRUPTS+0)  /* 0:  Window Watchdog interrupt */
#  define STM32_IRQ_PVD         (STM32_IRQ_INTERRUPTS+1)  /* 1:  PVD through EXTI Line detection interrupt */
#  define STM32_IRQ_TAMPER      (STM32_IRQ_INTERRUPTS+2)  /* 2:  Tamper through EXTI line interrupt, or */
#  define STM32_IRQ_TIMESTAMP   (STM32_IRQ_INTERRUPTS+2)  /* 2:  Time stamp through EXTI line interrupt */
#  define STM32_IRQ_RTC_WKUP    (STM32_IRQ_INTERRUPTS+3)  /* 3:  RTC Wakeup through EXTI line interrupt */
#  define STM32_IRQ_FLASH       (STM32_IRQ_INTERRUPTS+4)  /* 4:  Flash global interrupt */
#  define STM32_IRQ_RCC         (STM32_IRQ_INTERRUPTS+5)  /* 5:  RCC global interrupt */
#  define STM32_IRQ_EXTI0       (STM32_IRQ_INTERRUPTS+6)  /* 6:  EXTI Line 0 interrupt */
#  define STM32_IRQ_EXTI1       (STM32_IRQ_INTERRUPTS+7)  /* 7:  EXTI Line 1 interrupt */
#  define STM32_IRQ_EXTI2       (STM32_IRQ_INTERRUPTS+8)  /* 8:  EXTI Line 2 interrupt */
#  define STM32_IRQ_EXTI3       (STM32_IRQ_INTERRUPTS+9)  /* 9:  EXTI Line 3 interrupt */
#  define STM32_IRQ_EXTI4       (STM32_IRQ_INTERRUPTS+10) /* 10: EXTI Line 4 interrupt */
#  define STM32_IRQ_DMA1CH1     (STM32_IRQ_INTERRUPTS+11) /* 11: DMA1 channel 1 global interrupt */
#  define STM32_IRQ_DMA1CH2     (STM32_IRQ_INTERRUPTS+12) /* 12: DMA1 channel 2 global interrupt */
#  define STM32_IRQ_DMA1CH3     (STM32_IRQ_INTERRUPTS+13) /* 13: DMA1 channel 3 global interrupt */
#  define STM32_IRQ_DMA1CH4     (STM32_IRQ_INTERRUPTS+14) /* 14: DMA1 channel 4 global interrupt */
#  define STM32_IRQ_DMA1CH5     (STM32_IRQ_INTERRUPTS+15) /* 15: DMA1 channel 5 global interrupt */
#  define STM32_IRQ_DMA1CH6     (STM32_IRQ_INTERRUPTS+16) /* 16: DMA1 channel 6 global interrupt */
#  define STM32_IRQ_DMA1CH7     (STM32_IRQ_INTERRUPTS+17) /* 17: DMA1 channel 7 global interrupt */
#  define STM32_IRQ_ADC1        (STM32_IRQ_INTERRUPTS+18) /* 18: ADC1 global interrupt */
#  define STM32_IRQ_USBHP       (STM32_IRQ_INTERRUPTS+19) /* 19: USB High Priority interrupts */
#  define STM32_IRQ_USBLP       (STM32_IRQ_INTERRUPTS+20) /* 20: USB Low Priority interrupt */
#  define STM32_IRQ_DAC         (STM32_IRQ_INTERRUPTS+21) /* 21: DAC interrupt */
#  define STM32_IRQ_COMP        (STM32_IRQ_INTERRUPTS+22) /* 22: Comparator wakeup through EXTI interrupt */
#  define STM32_IRQ_EXTI95      (STM32_IRQ_INTERRUPTS+23) /* 23: EXTI Line[9:5] interrupts */
#  define STM32_IRQ_LDC         (STM32_IRQ_INTERRUPTS+24) /* 24: LCD global interrupt */
#  define STM32_IRQ_TIM9        (STM32_IRQ_INTERRUPTS+25) /* 25: TIM9 global interrupt */
#  define STM32_IRQ_TIM10       (STM32_IRQ_INTERRUPTS+26) /* 26: TIM10 global interrupt */
#  define STM32_IRQ_TIM11       (STM32_IRQ_INTERRUPTS+27) /* 27: TIM11 global interrupt */
#  define STM32_IRQ_TIM2        (STM32_IRQ_INTERRUPTS+28) /* 28: TIM2 global interrupt */
#  define STM32_IRQ_TIM3        (STM32_IRQ_INTERRUPTS+29) /* 29: TIM3 global interrupt */
#  define STM32_IRQ_TIM4        (STM32_IRQ_INTERRUPTS+30) /* 30: TIM4 global interrupt */
#  define STM32_IRQ_I2C1EV      (STM32_IRQ_INTERRUPTS+31) /* 31: I2C1 event interrupt */
#  define STM32_IRQ_I2C1ER      (STM32_IRQ_INTERRUPTS+32) /* 32: I2C1 error interrupt */
#  define STM32_IRQ_I2C2EV      (STM32_IRQ_INTERRUPTS+33) /* 33: I2C2 event interrupt */
#  define STM32_IRQ_I2C2ER      (STM32_IRQ_INTERRUPTS+34) /* 34: I2C2 error interrupt */
#  define STM32_IRQ_SPI1        (STM32_IRQ_INTERRUPTS+35) /* 35: SPI1 global interrupt */
#  define STM32_IRQ_SPI2        (STM32_IRQ_INTERRUPTS+36) /* 36: SPI2 global interrupt */
#  define STM32_IRQ_USART1      (STM32_IRQ_INTERRUPTS+37) /* 37: USART1 global interrupt */
#  define STM32_IRQ_USART2      (STM32_IRQ_INTERRUPTS+38) /* 38: USART2 global interrupt */
#  define STM32_IRQ_USART3      (STM32_IRQ_INTERRUPTS+39) /* 39: USART3 global interrupt */
#  define STM32_IRQ_EXTI1510    (STM32_IRQ_INTERRUPTS+40) /* 40: EXTI Line[15:10] interrupts */
#  define STM32_IRQ_RTCALRM     (STM32_IRQ_INTERRUPTS+41) /* 41: RTC alarm through EXTI line interrupt */
#  define STM32_IRQ_USBWKUP     (STM32_IRQ_INTERRUPTS+42) /* 42: USB wakeup from suspend through EXTI line interrupt */
#  define STM32_IRQ_TIM6        (STM32_IRQ_INTERRUPTS+43) /* 43: TIM6 global interrupt */
#  define STM32_IRQ_TIM7        (STM32_IRQ_INTERRUPTS+44) /* 44: TIM7 global interrupt */

#  define NR_IRQS               (STM32_IRQ_INTERRUPTS+45)

/* External interrupts (vectors >= 16) medium+ density devices */

#elif defined(CONFIG_STM32_MEDIUMPLUSDENSITY)
#  define STM32_IRQ_WWDG        (STM32_IRQ_INTERRUPTS+0)  /* 0:  Window Watchdog interrupt */
#  define STM32_IRQ_PVD         (STM32_IRQ_INTERRUPTS+1)  /* 1:  PVD through EXTI Line detection interrupt */
#  define STM32_IRQ_TAMPER      (STM32_IRQ_INTERRUPTS+2)  /* 2:  Tamper through EXTI line interrupt, or */
#  define STM32_IRQ_TIMESTAMP   (STM32_IRQ_INTERRUPTS+2)  /* 2:  Time stamp through EXTI line interrupt */
#  define STM32_IRQ_RTC_WKUP    (STM32_IRQ_INTERRUPTS+3)  /* 3:  RTC Wakeup through EXTI line interrupt */
#  define STM32_IRQ_FLASH       (STM32_IRQ_INTERRUPTS+4)  /* 4:  Flash global interrupt */
#  define STM32_IRQ_RCC         (STM32_IRQ_INTERRUPTS+5)  /* 5:  RCC global interrupt */
#  define STM32_IRQ_EXTI0       (STM32_IRQ_INTERRUPTS+6)  /* 6:  EXTI Line 0 interrupt */
#  define STM32_IRQ_EXTI1       (STM32_IRQ_INTERRUPTS+7)  /* 7:  EXTI Line 1 interrupt */
#  define STM32_IRQ_EXTI2       (STM32_IRQ_INTERRUPTS+8)  /* 8:  EXTI Line 2 interrupt */
#  define STM32_IRQ_EXTI3       (STM32_IRQ_INTERRUPTS+9)  /* 9:  EXTI Line 3 interrupt */
#  define STM32_IRQ_EXTI4       (STM32_IRQ_INTERRUPTS+10) /* 10: EXTI Line 4 interrupt */
#  define STM32_IRQ_DMA1CH1     (STM32_IRQ_INTERRUPTS+11) /* 11: DMA1 channel 1 global interrupt */
#  define STM32_IRQ_DMA1CH2     (STM32_IRQ_INTERRUPTS+12) /* 12: DMA1 channel 2 global interrupt */
#  define STM32_IRQ_DMA1CH3     (STM32_IRQ_INTERRUPTS+13) /* 13: DMA1 channel 3 global interrupt */
#  define STM32_IRQ_DMA1CH4     (STM32_IRQ_INTERRUPTS+14) /* 14: DMA1 channel 4 global interrupt */
#  define STM32_IRQ_DMA1CH5     (STM32_IRQ_INTERRUPTS+15) /* 15: DMA1 channel 5 global interrupt */
#  define STM32_IRQ_DMA1CH6     (STM32_IRQ_INTERRUPTS+16) /* 16: DMA1 channel 6 global interrupt */
#  define STM32_IRQ_DMA1CH7     (STM32_IRQ_INTERRUPTS+17) /* 17: DMA1 channel 7 global interrupt */
#  define STM32_IRQ_ADC1        (STM32_IRQ_INTERRUPTS+18) /* 18: ADC1 global interrupt */
#  define STM32_IRQ_USBHP       (STM32_IRQ_INTERRUPTS+19) /* 19: USB High Priority interrupts */
#  define STM32_IRQ_USBLP       (STM32_IRQ_INTERRUPTS+20) /* 20: USB Low Priority interrupt */
#  define STM32_IRQ_DAC         (STM32_IRQ_INTERRUPTS+21) /* 21: DAC interrupt */
#  define STM32_IRQ_COMP        (STM32_IRQ_INTERRUPTS+22) /* 22: Comparator wakeup through EXTI interrupt, or */
#  define STM32_IRQ_CA          (STM32_IRQ_INTERRUPTS+22) /* 22: Channel acquisition interrupt */
#  define STM32_IRQ_EXTI95      (STM32_IRQ_INTERRUPTS+23) /* 23: EXTI Line[9:5] interrupts */
#  define STM32_IRQ_LDC         (STM32_IRQ_INTERRUPTS+24) /* 24: LCD global interrupt */
#  define STM32_IRQ_TIM9        (STM32_IRQ_INTERRUPTS+25) /* 25: TIM9 global interrupt */
#  define STM32_IRQ_TIM10       (STM32_IRQ_INTERRUPTS+26) /* 26: TIM10 global interrupt */
#  define STM32_IRQ_TIM11       (STM32_IRQ_INTERRUPTS+27) /* 27: TIM11 global interrupt */
#  define STM32_IRQ_TIM2        (STM32_IRQ_INTERRUPTS+28) /* 28: TIM2 global interrupt */
#  define STM32_IRQ_TIM3        (STM32_IRQ_INTERRUPTS+29) /* 29: TIM3 global interrupt */
#  define STM32_IRQ_TIM4        (STM32_IRQ_INTERRUPTS+30) /* 30: TIM4 global interrupt */
#  define STM32_IRQ_I2C1EV      (STM32_IRQ_INTERRUPTS+31) /* 31: I2C1 event interrupt */
#  define STM32_IRQ_I2C1ER      (STM32_IRQ_INTERRUPTS+32) /* 32: I2C1 error interrupt */
#  define STM32_IRQ_I2C2EV      (STM32_IRQ_INTERRUPTS+33) /* 33: I2C2 event interrupt */
#  define STM32_IRQ_I2C2ER      (STM32_IRQ_INTERRUPTS+34) /* 34: I2C2 error interrupt */
#  define STM32_IRQ_SPI1        (STM32_IRQ_INTERRUPTS+35) /* 35: SPI1 global interrupt */
#  define STM32_IRQ_SPI2        (STM32_IRQ_INTERRUPTS+36) /* 36: SPI2 global interrupt */
#  define STM32_IRQ_USART1      (STM32_IRQ_INTERRUPTS+37) /* 37: USART1 global interrupt */
#  define STM32_IRQ_USART2      (STM32_IRQ_INTERRUPTS+38) /* 38: USART2 global interrupt */
#  define STM32_IRQ_USART3      (STM32_IRQ_INTERRUPTS+39) /* 39: USART3 global interrupt */
#  define STM32_IRQ_EXTI1510    (STM32_IRQ_INTERRUPTS+40) /* 40: EXTI Line[15:10] interrupts */
#  define STM32_IRQ_RTCALRM     (STM32_IRQ_INTERRUPTS+41) /* 41: RTC alarm through EXTI line interrupt */
#  define STM32_IRQ_USBWKUP     (STM32_IRQ_INTERRUPTS+42) /* 42: USB wakeup from suspend through EXTI line interrupt */
#  define STM32_IRQ_TIM6        (STM32_IRQ_INTERRUPTS+43) /* 43: TIM6 global interrupt */
#  define STM32_IRQ_TIM7        (STM32_IRQ_INTERRUPTS+44) /* 44: TIM7 global interrupt */
#  define STM32_IRQ_TIM5        (STM32_IRQ_INTERRUPTS+45) /* 45: TIM5 global interrupt */
#  define STM32_IRQ_SPI3        (STM32_IRQ_INTERRUPTS+46) /* 46: SPI3 global interrupt */
#  define STM32_IRQ_DMA2CH1     (STM32_IRQ_INTERRUPTS+47) /* 47: DMA2 channel 1 global interrupt */
#  define STM32_IRQ_DMA2CH2     (STM32_IRQ_INTERRUPTS+48) /* 48: DMA2 channel 2 global interrupt */
#  define STM32_IRQ_DMA2CH3     (STM32_IRQ_INTERRUPTS+49) /* 49: DMA2 channel 3 global interrupt */
#  define STM32_IRQ_DMA2CH4     (STM32_IRQ_INTERRUPTS+50) /* 50: DMA2 channel 4 global interrupt */
#  define STM32_IRQ_DMA2CH5     (STM32_IRQ_INTERRUPTS+51) /* 51: DMA2 channel 5 global interrupt */
#  define STM32_IRQ_AES         (STM32_IRQ_INTERRUPTS+52) /* 52: AES global interrupt */
#  define STM32_IRQ_COMPACQ     (STM32_IRQ_INTERRUPTS+53) /* 53: Comparator Channel Acquisition Interrupt */

#  define NR_IRQS               (STM32_IRQ_INTERRUPTS+54)

/* External interrupts (vectors >= 16) high density devices */

#elif defined(CONFIG_STM32_HIGHDENSITY)
#  define STM32_IRQ_WWDG        (STM32_IRQ_INTERRUPTS+0)  /* 0:  Window Watchdog interrupt */
#  define STM32_IRQ_PVD         (STM32_IRQ_INTERRUPTS+1)  /* 1:  PVD through EXTI Line detection interrupt */
#  define STM32_IRQ_TAMPER      (STM32_IRQ_INTERRUPTS+2)  /* 2:  Tamper through EXTI line interrupt, or */
#  define STM32_IRQ_TIMESTAMP   (STM32_IRQ_INTERRUPTS+2)  /* 2:  Time stamp through EXTI line interrupt */
#  define STM32_IRQ_RTC_WKUP    (STM32_IRQ_INTERRUPTS+3)  /* 3:  RTC Wakeup through EXTI line interrupt */
#  define STM32_IRQ_FLASH       (STM32_IRQ_INTERRUPTS+4)  /* 4:  Flash global interrupt */
#  define STM32_IRQ_RCC         (STM32_IRQ_INTERRUPTS+5)  /* 5:  RCC global interrupt */
#  define STM32_IRQ_EXTI0       (STM32_IRQ_INTERRUPTS+6)  /* 6:  EXTI Line 0 interrupt */
#  define STM32_IRQ_EXTI1       (STM32_IRQ_INTERRUPTS+7)  /* 7:  EXTI Line 1 interrupt */
#  define STM32_IRQ_EXTI2       (STM32_IRQ_INTERRUPTS+8)  /* 8:  EXTI Line 2 interrupt */
#  define STM32_IRQ_EXTI3       (STM32_IRQ_INTERRUPTS+9)  /* 9:  EXTI Line 3 interrupt */
#  define STM32_IRQ_EXTI4       (STM32_IRQ_INTERRUPTS+10) /* 10: EXTI Line 4 interrupt */
#  define STM32_IRQ_DMA1CH1     (STM32_IRQ_INTERRUPTS+11) /* 11: DMA1 channel 1 global interrupt */
#  define STM32_IRQ_DMA1CH2     (STM32_IRQ_INTERRUPTS+12) /* 12: DMA1 channel 2 global interrupt */
#  define STM32_IRQ_DMA1CH3     (STM32_IRQ_INTERRUPTS+13) /* 13: DMA1 channel 3 global interrupt */
#  define STM32_IRQ_DMA1CH4     (STM32_IRQ_INTERRUPTS+14) /* 14: DMA1 channel 4 global interrupt */
#  define STM32_IRQ_DMA1CH5     (STM32_IRQ_INTERRUPTS+15) /* 15: DMA1 channel 5 global interrupt */
#  define STM32_IRQ_DMA1CH6     (STM32_IRQ_INTERRUPTS+16) /* 16: DMA1 channel 6 global interrupt */
#  define STM32_IRQ_DMA1CH7     (STM32_IRQ_INTERRUPTS+17) /* 17: DMA1 channel 7 global interrupt */
#  define STM32_IRQ_ADC1        (STM32_IRQ_INTERRUPTS+18) /* 18: ADC1 global interrupt */
#  define STM32_IRQ_USBHP       (STM32_IRQ_INTERRUPTS+19) /* 19: USB High Priority interrupts */
#  define STM32_IRQ_USBLP       (STM32_IRQ_INTERRUPTS+20) /* 20: USB Low Priority interrupt */
#  define STM32_IRQ_DAC         (STM32_IRQ_INTERRUPTS+21) /* 21: DAC interrupt */
#  define STM32_IRQ_COMP        (STM32_IRQ_INTERRUPTS+22) /* 22: Comparator wakeup through EXTI interrupt, or */
#  define STM32_IRQ_CA          (STM32_IRQ_INTERRUPTS+22) /* 22: Channel acquisition interrupt */
#  define STM32_IRQ_EXTI95      (STM32_IRQ_INTERRUPTS+23) /* 23: EXTI Line[9:5] interrupts */
#  define STM32_IRQ_LDC         (STM32_IRQ_INTERRUPTS+24) /* 24: LCD global interrupt */
#  define STM32_IRQ_TIM9        (STM32_IRQ_INTERRUPTS+25) /* 25: TIM9 global interrupt */
#  define STM32_IRQ_TIM10       (STM32_IRQ_INTERRUPTS+26) /* 26: TIM10 global interrupt */
#  define STM32_IRQ_TIM11       (STM32_IRQ_INTERRUPTS+27) /* 27: TIM11 global interrupt */
#  define STM32_IRQ_TIM2        (STM32_IRQ_INTERRUPTS+28) /* 28: TIM2 global interrupt */
#  define STM32_IRQ_TIM3        (STM32_IRQ_INTERRUPTS+29) /* 29: TIM3 global interrupt */
#  define STM32_IRQ_TIM4        (STM32_IRQ_INTERRUPTS+30) /* 30: TIM4 global interrupt */
#  define STM32_IRQ_I2C1EV      (STM32_IRQ_INTERRUPTS+31) /* 31: I2C1 event interrupt */
#  define STM32_IRQ_I2C1ER      (STM32_IRQ_INTERRUPTS+32) /* 32: I2C1 error interrupt */
#  define STM32_IRQ_I2C2EV      (STM32_IRQ_INTERRUPTS+33) /* 33: I2C2 event interrupt */
#  define STM32_IRQ_I2C2ER      (STM32_IRQ_INTERRUPTS+34) /* 34: I2C2 error interrupt */
#  define STM32_IRQ_SPI1        (STM32_IRQ_INTERRUPTS+35) /* 35: SPI1 global interrupt */
#  define STM32_IRQ_SPI2        (STM32_IRQ_INTERRUPTS+36) /* 36: SPI2 global interrupt */
#  define STM32_IRQ_USART1      (STM32_IRQ_INTERRUPTS+37) /* 37: USART1 global interrupt */
#  define STM32_IRQ_USART2      (STM32_IRQ_INTERRUPTS+38) /* 38: USART2 global interrupt */
#  define STM32_IRQ_USART3      (STM32_IRQ_INTERRUPTS+39) /* 39: USART3 global interrupt */
#  define STM32_IRQ_EXTI1510    (STM32_IRQ_INTERRUPTS+40) /* 40: EXTI Line[15:10] interrupts */
#  define STM32_IRQ_RTCALRM     (STM32_IRQ_INTERRUPTS+41) /* 41: RTC alarm through EXTI line interrupt */
#  define STM32_IRQ_USBWKUP     (STM32_IRQ_INTERRUPTS+42) /* 42: USB wakeup from suspend through EXTI line interrupt */
#  define STM32_IRQ_TIM6        (STM32_IRQ_INTERRUPTS+43) /* 43: TIM6 global interrupt */
#  define STM32_IRQ_TIM7        (STM32_IRQ_INTERRUPTS+44) /* 44: TIM7 global interrupt */
#  define STM32_IRQ_SDIO        (STM32_IRQ_INTERRUPTS+45) /* 45: SDIO Global interrupt */
#  define STM32_IRQ_TIM5        (STM32_IRQ_INTERRUPTS+46) /* 46: TIM5 global interrupt */
#  define STM32_IRQ_SPI3        (STM32_IRQ_INTERRUPTS+47) /* 47: SPI3 global interrupt */
#  define STM32_IRQ_USART4      (STM32_IRQ_INTERRUPTS+48) /* 48: USART4 global interrupt */
#  define STM32_IRQ_USART5      (STM32_IRQ_INTERRUPTS+49) /* 49: USART5 global interrupt */
#  define STM32_IRQ_DMA2CH1     (STM32_IRQ_INTERRUPTS+50) /* 50: DMA2 channel 1 global interrupt */
#  define STM32_IRQ_DMA2CH2     (STM32_IRQ_INTERRUPTS+51) /* 51: DMA2 channel 2 global interrupt */
#  define STM32_IRQ_DMA2CH3     (STM32_IRQ_INTERRUPTS+52) /* 52: DMA2 channel 3 global interrupt */
#  define STM32_IRQ_DMA2CH4     (STM32_IRQ_INTERRUPTS+53) /* 53: DMA2 channel 4 global interrupt */
#  define STM32_IRQ_DMA2CH5     (STM32_IRQ_INTERRUPTS+54) /* 54: DMA2 channel 5 global interrupt */
#  define STM32_IRQ_AES         (STM32_IRQ_INTERRUPTS+55) /* 55: AES global interrupt */
#  define STM32_IRQ_COMPACQ     (STM32_IRQ_INTERRUPTS+56) /* 56: Comparator Channel Acquisition Interrupt */

#  define NR_IRQS               (STM32_IRQ_INTERRUPTS+57)
#else
#  error "Unknown STM32L density"
#endif

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Data
****************************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
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

#endif /* __ARCH_ARM_INCLUDE_STM32L15XXX_IRQ_H */

