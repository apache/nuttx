/************************************************************************************
 * arch/arm/src/stm32/chip/stm32f33xxx_vectors.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *   Modified for STM32F334 by Mateusz Szafoni <raiden00@railab.me>
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

/************************************************************************************
 * Pre-processor definitions
 ************************************************************************************/
/* This file is included by stm32_vectors.S.  It provides the macro VECTOR that
 * supplies each STM32F33xxx vector in terms of a (lower-case) ISR label and an
 * (upper-case) IRQ number as defined in arch/arm/include/stm32/stm32f33xxx_irq.h.
 * stm32_vectors.S will defined the VECTOR in different ways in order to generate
 * the interrupt vectors and handlers in their final form.
 */

/* If the common ARMv7-M vector handling is used, then all it needs is the following
 * definition that provides the number of supported vectors.
 */

#ifdef CONFIG_ARMV7M_CMNVECTOR

/* Reserve 82 interrupt table entries for I/O interrupts. */

#  define ARMV7M_PERIPHERAL_INTERRUPTS 82

#else

VECTOR(stm32_wwdg, STM32_IRQ_WWDG)             /* 0:  Window Watchdog interrupt */
VECTOR(stm32_pvd, STM32_IRQ_PVD)               /* 1:  PVD through EXTI Line detection interrupt */
VECTOR(stm32_tamper, STM32_IRQ_TAMPER)         /* 2:  Tamper or Time stamp interrupt */
VECTOR(stm32_rtc_wkup, STM32_IRQ_RTC_WKUP)     /* 3:  RTC global interrupt */
VECTOR(stm32_flash, STM32_IRQ_FLASH)           /* 4:  Flash global interrupt */
VECTOR(stm32_rcc, STM32_IRQ_RCC)               /* 5:  RCC global interrupt */
VECTOR(stm32_exti0, STM32_IRQ_EXTI0)           /* 6:  EXTI Line 0 interrupt */
VECTOR(stm32_exti1, STM32_IRQ_EXTI1)           /* 7:  EXTI Line 1 interrupt */
VECTOR(stm32_exti2, STM32_IRQ_EXTI2)           /* 8:  EXTI Line 2 or TSC interrupt */
VECTOR(stm32_exti3, STM32_IRQ_EXTI3)           /* 9:  EXTI Line 3 interrupt */

VECTOR(stm32_exti4, STM32_IRQ_EXTI4)           /* 10: EXTI Line 4 interrupt */
VECTOR(stm32_dma1ch1, STM32_IRQ_DMA1CH1)       /* 11: DMA1 channel 1 global interrupt */
VECTOR(stm32_dma1ch2, STM32_IRQ_DMA1CH2)       /* 12: DMA1 channel 2 global interrupt */
VECTOR(stm32_dma1ch3, STM32_IRQ_DMA1CH3)       /* 13: DMA1 channel 3 global interrupt */
VECTOR(stm32_dma1ch4, STM32_IRQ_DMA1CH4)       /* 14: DMA1 channel 4 global interrupt */
VECTOR(stm32_dma1ch5, STM32_IRQ_DMA1CH5)       /* 15: DMA1 channel 5 global interrupt */
VECTOR(stm32_dma1ch6, STM32_IRQ_DMA1CH6)       /* 16: DMA1 channel 6 global interrupt */
VECTOR(stm32_dma1ch7, STM32_IRQ_DMA1CH7)       /* 17: DMA1 channel 7 global interrupt */
VECTOR(stm32_adc12, STM32_IRQ_ADC12)           /* 18: ADC1/ADC2 global interrupt */
VECTOR(stm32_can1tx, STM32_IRQ_CAN1TX)         /* 19: USB High Priority or CAN1 TX interrupts */

VECTOR(stm32_can1rx0, STM32_IRQ_CAN1RX0)       /* 20: USB Low Priority or CAN1 RX0 interrupts*/
VECTOR(stm32_can1rx1, STM32_IRQ_CAN1RX1)       /* 21: CAN1 RX1 interrupt */
VECTOR(stm32_can1sce, STM32_IRQ_CAN1SCE)       /* 22: CAN1 SCE interrupt */
VECTOR(stm32_exti95, STM32_IRQ_EXTI95)         /* 23: EXTI Line[9:5] interrupts */
VECTOR(stm32_tim1brk, STM32_IRQ_TIM1BRK)       /* 24: TIM1 Break or TIM15 global interrupt */
VECTOR(stm32_tim1up, STM32_IRQ_TIM1UP)         /* 25: TIM1 Update or TIM16 global interrupt */
VECTOR(stm32_tim1trgcom, STM32_IRQ_TIM1TRGCOM) /* 26: TIM1 Trigger or TIM17 global interrupt */
VECTOR(stm32_tim1cc, STM32_IRQ_TIM1CC)         /* 27: TIM1 Capture Compare interrupt */
VECTOR(stm32_tim2, STM32_IRQ_TIM2)             /* 28: TIM2 global interrupt */
VECTOR(stm32_tim3, STM32_IRQ_TIM3)             /* 29: TIM3 global interrupt */

UNUSED(STM32_IRQ_RESERVED30)                   /* 30: Reserved */
VECTOR(stm32_i2c1ev, STM32_IRQ_I2C1EV)         /* 31: I2C1 event or EXTI Line23 interrupt */
VECTOR(stm32_i2c1er, STM32_IRQ_I2C1ER)         /* 32: I2C1 error interrupt */
UNUSED(STM32_IRQ_RESERVED33)                   /* 33: Reserved */
UNUSED(STM32_IRQ_RESERVED34)                   /* 34: Reserved */
VECTOR(stm32_spi1, STM32_IRQ_SPI1)             /* 35: SPI1 global interrupt */
UNUSED(STM32_IRQ_RESERVED36)                   /* 36: Reserved */
VECTOR(stm32_usart1, STM32_IRQ_USART1)         /* 37: USART1 global or EXTI Line 25 interrupt */
VECTOR(stm32_usart2, STM32_IRQ_USART2)         /* 38: USART2 global or EXTI Line 26 interrupt */
VECTOR(stm32_usart3, STM32_IRQ_USART3)         /* 39: USART3 global or EXTI Line 28 interrupt */

VECTOR(stm32_exti1510, STM32_IRQ_EXTI1510)     /* 40: EXTI Line[15:10] interrupts */
VECTOR(stm32_rtcalrm, STM32_IRQ_RTCALRM)       /* 41: RTC alarm through EXTI line interrupt */
UNUSED(STM32_IRQ_RESERVED42)                   /* 42: Reserved */
UNUSED(STM32_IRQ_RESERVED43)                   /* 43: Reserved */
UNUSED(STM32_IRQ_RESERVED44)                   /* 44: Reserved */
UNUSED(STM32_IRQ_RESERVED45)                   /* 45: Reserved */
UNUSED(STM32_IRQ_RESERVED46)                   /* 46: Reserved */
UNUSED(STM32_IRQ_RESERVED47)                   /* 47: Reserved*/
UNUSED(STM32_IRQ_RESERVED48)                   /* 48: Reserved */
UNUSED(STM32_IRQ_RESERVED49)                   /* 49: Reserved */

UNUSED(STM32_IRQ_RESERVED50)                   /* 50: Reserved */
UNUSED(STM32_IRQ_RESERVED51)                   /* 51: Reserved */
UNUSED(STM32_IRQ_RESERVED51)                   /* 52: Reserved */
UNUSED(STM32_IRQ_RESERVED52)                   /* 53: Reserved */
VECTOR(stm32_dac1, STM32_IRQ_DAC1)             /* 54: TIM6 global or DAC1 underrun interrupts */
VECTOR(stm32_dac2, STM32_IRQ_DAC2)             /* 55: TIM7 global or DAC2 underrun interrupt */
UNUSED(STM32_IRQ_RESERVED56)                   /* 56: Reserved */
UNUSED(STM32_IRQ_RESERVED57)                   /* 57: Reserved */
UNUSED(STM32_IRQ_RESERVED58)                   /* 58: Reserved */
UNUSED(STM32_IRQ_RESERVED59)                   /* 59: Reserved */

UNUSED(STM32_IRQ_RESERVED60)                   /* 60: Reserved */
UNUSED(STM32_IRQ_RESERVED61)                   /* 61: Reserved */
UNUSED(STM32_IRQ_RESERVED62)                   /* 62: Reserved */
UNUSED(STM32_IRQ_RESERVED63)                   /* 63: Reserved */
VECTOR(stm32_comp2, STM32_IRQ_COMP2)           /* 64: COMP2 or EXTI Lines 21-2 and 29 interrupts */
VECTOR(stm32_comp46, STM32_IRQ_COMP46)         /* 65: COMP4/COMP6 or EXTI Lines 30-2 interrupts */
UNUSED(STM32_IRQ_RESERVED66)                   /* 66: Reserved */
VECTOR(stm32_hrtim_tm, STM32_IRQ_HRTIMTM)      /* 67: HRTIM master timer interrutp */
VECTOR(stm32_hrtim_ta, STM32_IRQ_HRTIMTA)      /* 68: HRTIM timer A interrutp */
VECTOR(stm32_hrtim_tb, STM32_IRQ_HRTIMTB)      /* 69: HRTIM timer B interrutp */

VECTOR(stm32_hrtim_tc, STM32_IRQ_HRTIMTC)      /* 70: HRTIM timer C interrutp */
VECTOR(stm32_hrtim_td, STM32_IRQ_HRTIMTD)      /* 71: HRTIM timer D interrutp */
VECTOR(stm32_hrtim_te, STM32_IRQ_HRTIMTE)      /* 72: HRTIM timer E interrutp */
VECTOR(stm32_hrtim_flt, STM32_IRQ_HRTIMFLT)    /* 73: HRTIM fault interrutp */
UNUSED(STM32_IRQ_RESERVED73)                   /* 74: Reserved */
UNUSED(STM32_IRQ_RESERVED74)                   /* 75: Reserved */
UNUSED(STM32_IRQ_RESERVED75)                   /* 76: Reserved */
UNUSED(STM32_IRQ_RESERVED76)                   /* 77: Reserved */
UNUSED(STM32_IRQ_RESERVED77)                   /* 78: Reserved */
UNUSED(STM32_IRQ_RESERVED78)                   /* 79: Reserved */

UNUSED(STM32_IRQ_RESERVED79)                   /* 80: Reserved */
UNUSED(STM32_IRQ_RESERVED80)                   /* 81: Reserved */
VECTOR(stm32_fpu, STM32_IRQ_FPU)               /* 82: FPU global interrupt */

#endif /* CONFIG_ARMV7M_CMNVECTOR */
