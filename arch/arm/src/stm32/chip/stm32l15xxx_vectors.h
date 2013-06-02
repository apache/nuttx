/************************************************************************************
 * arch/arm/src/stm32/chip/stm32l15xxx_vectors.h
 * For STM32L100xx, STM32L151xx, STM32L152xx and STM32L162xx advanced ARM-based
 * 32-bit MCUs
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
 ************************************************************************************/

/************************************************************************************
 * Pre-processor definitions
 ************************************************************************************/
/* This file is included by stm32_vectors.S.  It provides the macro VECTOR that
 * supplies ach STM32F10xxx vector in terms of a (lower-case) ISR label and an
 * (upper-case) IRQ number as defined in arch/arm/include/stm32/stm32f10xxx_irq.h.
 * stm32_vectors.S will defined the VECTOR in different ways in order to generate
 * the interrupt vectors and handlers in their final form.
 *
 *
 * Vectors for low and medium density devices
 */

#if defined(CONFIG_STM32_LOWDENSITY) || defined(CONFIG_STM32_MEDIUMDENSITY)

/* If the common ARMv7-M vector handling is used, then all it needs is the following
 * definition that provides the number of supported vectors.
 */

#ifdef CONFIG_ARMV7M_CMNVECTOR

/* Reserve 45 interrupt table entries for I/O interrupts. */

# define ARMV7M_PERIPHERAL_INTERRUPTS 45

#else
  VECTOR(stm32_wwdg, STM32_IRQ_WWDG)          /* 0:  Window Watchdog interrupt */
  VECTOR(stm32_pvd, STM32_IRQ_PVD)            /* 1:  PVD through EXTI Line detection interrupt */
  VECTOR(stm32_tamper, STM32_IRQ_TAMPER)      /* 2:  Tamper through EXTI line interrupt */
  VECTOR(stm32_rtc_wkup, STM32_IRQ_RTC_WKUP)  /* 3:  RTC Wakeup through EXTI line interrupt */
  VECTOR(stm32_flash, STM32_IRQ_FLASH)        /* 4:  Flash global interrupt */
  VECTOR(stm32_rcc, STM32_IRQ_RCC)            /* 5:  RCC global interrupt */
  VECTOR(stm32_exti0, STM32_IRQ_EXTI0)        /* 6:  EXTI Line 0 interrupt */
  VECTOR(stm32_exti1, STM32_IRQ_EXTI1)        /* 7:  EXTI Line 1 interrupt */
  VECTOR(stm32_exti2, STM32_IRQ_EXTI2)        /* 8:  EXTI Line 2 interrupt */
  VECTOR(stm32_exti3, STM32_IRQ_EXTI3)        /* 9:  EXTI Line 3 interrupt */
  VECTOR(stm32_exti4, STM32_IRQ_EXTI4)        /* 10: EXTI Line 4 interrupt */
  VECTOR(stm32_dma1ch1, STM32_IRQ_DMA1CH1)    /* 11: DMA1 channel 1 global interrupt */
  VECTOR(stm32_dma1ch2, STM32_IRQ_DMA1CH2)    /* 12: DMA1 channel 2 global interrupt */
  VECTOR(stm32_dma1ch3, STM32_IRQ_DMA1CH3)    /* 13: DMA1 channel 3 global interrupt */
  VECTOR(stm32_dma1ch4, STM32_IRQ_DMA1CH4)    /* 14: DMA1 channel 4 global interrupt */
  VECTOR(stm32_dma1ch5, STM32_IRQ_DMA1CH5)    /* 15: DMA1 channel 5 global interrupt */
  VECTOR(stm32_dma1ch6, STM32_IRQ_DMA1CH6)    /* 16: DMA1 channel 6 global interrupt */
  VECTOR(stm32_dma1ch7, STM32_IRQ_DMA1CH7)    /* 17: DMA1 channel 7 global interrupt */
  VECTOR(stm32_adc1, STM32_IRQ_ADC1)          /* 18: ADC1 global interrupt */
  VECTOR(stm32_usbhp, STM32_IRQ_USBHP)        /* 19: USB High Priority interrupts */
  VECTOR(stm32_usblp, STM32_IRQ_USBLP)        /* 20: USB Low Priority interrupt */
  VECTOR(stm32_dac, STM32_IRQ_DAC)            /* 21: DAC interrupt */
  VECTOR(stm32_comp, STM32_IRQ_COMP)          /* 22: Comparator wakeup through EXTI interrupt */
  VECTOR(stm32_exti95, STM32_IRQ_EXTI95)      /* 23: EXTI Line[9:5] interrupts */
  VECTOR(stm32_ldc, STM32_IRQ_LDC)            /* 24: LCD global interrupt */
  VECTOR(stm32_tim9, STM32_IRQ_TIM9)          /* 25: TIM9 global interrupt */
  VECTOR(stm32_tim10, STM32_IRQ_TIM10)        /* 26: TIM10 global interrupt */
  VECTOR(stm32_tim11, STM32_IRQ_TIM11)        /* 27: TIM11 global interrupt */
  VECTOR(stm32_tim2, STM32_IRQ_TIM2)          /* 28: TIM2 global interrupt */
  VECTOR(stm32_tim3, STM32_IRQ_TIM3)          /* 29: TIM3 global interrupt */
  VECTOR(stm32_tim4, STM32_IRQ_TIM4)          /* 30: TIM4 global interrupt */
  VECTOR(stm32_i2c1ev, STM32_IRQ_I2C1EV)      /* 31: I2C1 event interrupt */
  VECTOR(stm32_i2c1er, STM32_IRQ_I2C1ER)      /* 32: I2C1 error interrupt */
  VECTOR(stm32_i2c2ev, STM32_IRQ_I2C2EV)      /* 33: I2C2 event interrupt */
  VECTOR(stm32_i2c2er, STM32_IRQ_I2C2ER)      /* 34: I2C2 error interrupt */
  VECTOR(stm32_spi1, STM32_IRQ_SPI1)          /* 35: SPI1 global interrupt */
  VECTOR(stm32_spi2, STM32_IRQ_SPI2)          /* 36: SPI2 global interrupt */
  VECTOR(stm32_usart1, STM32_IRQ_USART1)      /* 37: USART1 global interrupt */
  VECTOR(stm32_usart2, STM32_IRQ_USART2)      /* 38: USART2 global interrupt */
  VECTOR(stm32_usart3, STM32_IRQ_USART3)      /* 39: USART3 global interrupt */
  VECTOR(stm32_exti1510, STM32_IRQ_EXTI1510)  /* 40: EXTI Line[15:10] interrupts */
  VECTOR(stm32_rtcalrm, STM32_IRQ_RTCALRM)    /* 41: RTC alarm through EXTI line interrupt */
  VECTOR(stm32_usbwkup, STM32_IRQ_USBWKUP)    /* 42: USB wakeup from suspend through EXTI line interrupt */
  VECTOR(stm32_tim6, STM32_IRQ_TIM6)          /* 43: TIM6 global interrupt */
  VECTOR(stm32_TIM7, STM32_IRQ_TIM7)          /* 44: TIM7 global interrupt */
#endif

/* Vectors for medium+ density devices */

#elif defined(CONFIG_STM32_MEDIUMPLUSDENSITY)

/* If the common ARMv7-M vector handling is used, then all it needs is the following
 * definition that provides the number of supported vectors.
 */

#ifdef CONFIG_ARMV7M_CMNVECTOR

/* Reserve 61 interrupt table entries for I/O interrupts. */

# define ARMV7M_PERIPHERAL_INTERRUPTS 54

#else
  VECTOR(stm32_wwdg, STM32_IRQ_WWDG)          /* 0:  Window Watchdog interrupt */
  VECTOR(stm32_pvd, STM32_IRQ_PVD)            /* 1:  PVD through EXTI Line detection interrupt */
  VECTOR(stm32_tamper, STM32_IRQ_TAMPER)      /* 2:  Tamper through EXTI line interrupt */
  VECTOR(stm32_rtc_wkup, STM32_IRQ_RTC_WKUP)  /* 3:  RTC Wakeup through EXTI line interrupt */
  VECTOR(stm32_flash, STM32_IRQ_FLASH)        /* 4:  Flash global interrupt */
  VECTOR(stm32_rcc, STM32_IRQ_RCC)            /* 5:  RCC global interrupt */
  VECTOR(stm32_exti0, STM32_IRQ_EXTI0)        /* 6:  EXTI Line 0 interrupt */
  VECTOR(stm32_exti1, STM32_IRQ_EXTI1)        /* 7:  EXTI Line 1 interrupt */
  VECTOR(stm32_exti2, STM32_IRQ_EXTI2)        /* 8:  EXTI Line 2 interrupt */
  VECTOR(stm32_exti3, STM32_IRQ_EXTI3)        /* 9:  EXTI Line 3 interrupt */
  VECTOR(stm32_exti4, STM32_IRQ_EXTI4)        /* 10: EXTI Line 4 interrupt */
  VECTOR(stm32_dma1ch1, STM32_IRQ_DMA1CH1)    /* 11: DMA1 channel 1 global interrupt */
  VECTOR(stm32_dma1ch2, STM32_IRQ_DMA1CH2)    /* 12: DMA1 channel 2 global interrupt */
  VECTOR(stm32_dma1ch3, STM32_IRQ_DMA1CH3)    /* 13: DMA1 channel 3 global interrupt */
  VECTOR(stm32_dma1ch4, STM32_IRQ_DMA1CH4)    /* 14: DMA1 channel 4 global interrupt */
  VECTOR(stm32_dma1ch5, STM32_IRQ_DMA1CH5)    /* 15: DMA1 channel 5 global interrupt */
  VECTOR(stm32_dma1ch6, STM32_IRQ_DMA1CH6)    /* 16: DMA1 channel 6 global interrupt */
  VECTOR(stm32_dma1ch7, STM32_IRQ_DMA1CH7)    /* 17: DMA1 channel 7 global interrupt */
  VECTOR(stm32_adc1, STM32_IRQ_ADC1)          /* 18: ADC1 global interrupt */
  VECTOR(stm32_usbhp, STM32_IRQ_USBHP)        /* 19: USB High Priority interrupts */
  VECTOR(stm32_usblp, STM32_IRQ_USBLP)        /* 20: USB Low Priority interrupt */
  VECTOR(stm32_dac, STM32_IRQ_DAC)            /* 21: DAC interrupt */
  VECTOR(stm32_comp, STM32_IRQ_COMP)          /* 22: Comparator wakeup through EXTI interrupt */
  VECTOR(stm32_exti95, STM32_IRQ_EXTI95)      /* 23: EXTI Line[9:5] interrupts */
  VECTOR(stm32_ldc, STM32_IRQ_LDC)            /* 24: LCD global interrupt */
  VECTOR(stm32_tim9, STM32_IRQ_TIM9)          /* 25: TIM9 global interrupt */
  VECTOR(stm32_tim10, STM32_IRQ_TIM10)        /* 26: TIM10 global interrupt */
  VECTOR(stm32_tim11, STM32_IRQ_TIM11)        /* 27: TIM11 global interrupt */
  VECTOR(stm32_tim2, STM32_IRQ_TIM2)          /* 28: TIM2 global interrupt */
  VECTOR(stm32_tim3, STM32_IRQ_TIM3)          /* 29: TIM3 global interrupt */
  VECTOR(stm32_tim4, STM32_IRQ_TIM4)          /* 30: TIM4 global interrupt */
  VECTOR(stm32_i2c1ev, STM32_IRQ_I2C1EV)      /* 31: I2C1 event interrupt */
  VECTOR(stm32_i2c1er, STM32_IRQ_I2C1ER)      /* 32: I2C1 error interrupt */
  VECTOR(stm32_i2c2ev, STM32_IRQ_I2C2EV)      /* 33: I2C2 event interrupt */
  VECTOR(stm32_i2c2er, STM32_IRQ_I2C2ER)      /* 34: I2C2 error interrupt */
  VECTOR(stm32_spi1, STM32_IRQ_SPI1)          /* 35: SPI1 global interrupt */
  VECTOR(stm32_spi2, STM32_IRQ_SPI2)          /* 36: SPI2 global interrupt */
  VECTOR(stm32_usart1, STM32_IRQ_USART1)      /* 37: USART1 global interrupt */
  VECTOR(stm32_usart2, STM32_IRQ_USART2)      /* 38: USART2 global interrupt */
  VECTOR(stm32_usart3, STM32_IRQ_USART3)      /* 39: USART3 global interrupt */
  VECTOR(stm32_exti1510, STM32_IRQ_EXTI1510)  /* 40: EXTI Line[15:10] interrupts */
  VECTOR(stm32_rtcalrm, STM32_IRQ_RTCALRM)    /* 41: RTC alarm through EXTI line interrupt */
  VECTOR(stm32_usbwkup, STM32_IRQ_USBWKUP)    /* 42: USB wakeup from suspend through EXTI line interrupt */
  VECTOR(stm32_tim6, STM32_IRQ_TIM6)          /* 43: TIM6 global interrupt */
  VECTOR(stm32_tim7, STM32_IRQ_TIM7)          /* 44: TIM7 global interrupt */
  VECTOR(stm32_tim5, STM32_IRQ_TIM5)          /* 45: TIM5 global interrupt */
  VECTOR(stm32_spi3, STM32_IRQ_SPI3)          /* 46: SPI3 global interrupt */
  VECTOR(stm32_dma2ch1, STM32_IRQ_DMA2CH1)    /* 47: DMA2 channel 1 global interrupt */
  VECTOR(stm32_dma2ch2, STM32_IRQ_DMA2CH2)    /* 48: DMA2 channel 2 global interrupt */
  VECTOR(stm32_dma2ch3, STM32_IRQ_DMA2CH3)    /* 49: DMA2 channel 3 global interrupt */
  VECTOR(stm32_dma2ch4, STM32_IRQ_DMA2CH4)    /* 50: DMA2 channel 4 global interrupt */
  VECTOR(stm32_dma2ch5, STM32_IRQ_DMA2CH5)    /* 51: DMA2 channel 5 global interrupt */
  VECTOR(stm32_aes, STM32_IRQ_AES)            /* 52: AES global interrupt */
  VECTOR(stm32_compacq, STM32_IRQ_COMPACQ)    /* 53: Comparator Channel Acquisition Interrupt */
#endif

/* Vectors for high density devices */

#elif defined(CONFIG_STM32_HIGHDENSITY)

/* If the common ARMv7-M vector handling is used, then all it needs is the following
 * definition that provides the number of supported vectors.
 */

#ifdef CONFIG_ARMV7M_CMNVECTOR

/* Reserve 61 interrupt table entries for I/O interrupts. */

# define ARMV7M_PERIPHERAL_INTERRUPTS 57

#else
  VECTOR(stm32_wwdg, STM32_IRQ_WWDG)          /* 0:  Window Watchdog interrupt */
  VECTOR(stm32_pvd, STM32_IRQ_PVD)            /* 1:  PVD through EXTI Line detection interrupt */
  VECTOR(stm32_tamper, STM32_IRQ_TAMPER)      /* 2:  Tamper through EXTI line interrupt */
  VECTOR(stm32_rtc_wkup, STM32_IRQ_RTC_WKUP)  /* 3:  RTC Wakeup through EXTI line interrupt */
  VECTOR(stm32_flash, STM32_IRQ_FLASH)        /* 4:  Flash global interrupt */
  VECTOR(stm32_rcc, STM32_IRQ_RCC)            /* 5:  RCC global interrupt */
  VECTOR(stm32_exti0, STM32_IRQ_EXTI0)        /* 6:  EXTI Line 0 interrupt */
  VECTOR(stm32_exti1, STM32_IRQ_EXTI1)        /* 7:  EXTI Line 1 interrupt */
  VECTOR(stm32_exti2, STM32_IRQ_EXTI2)        /* 8:  EXTI Line 2 interrupt */
  VECTOR(stm32_exti3, STM32_IRQ_EXTI3)        /* 9:  EXTI Line 3 interrupt */
  VECTOR(stm32_exti4, STM32_IRQ_EXTI4)        /* 10: EXTI Line 4 interrupt */
  VECTOR(stm32_dma1ch1, STM32_IRQ_DMA1CH1)    /* 11: DMA1 channel 1 global interrupt */
  VECTOR(stm32_dma1ch2, STM32_IRQ_DMA1CH2)    /* 12: DMA1 channel 2 global interrupt */
  VECTOR(stm32_dma1ch3, STM32_IRQ_DMA1CH3)    /* 13: DMA1 channel 3 global interrupt */
  VECTOR(stm32_dma1ch4, STM32_IRQ_DMA1CH4)    /* 14: DMA1 channel 4 global interrupt */
  VECTOR(stm32_dma1ch5, STM32_IRQ_DMA1CH5)    /* 15: DMA1 channel 5 global interrupt */
  VECTOR(stm32_dma1ch6, STM32_IRQ_DMA1CH6)    /* 16: DMA1 channel 6 global interrupt */
  VECTOR(stm32_dma1ch7, STM32_IRQ_DMA1CH7)    /* 17: DMA1 channel 7 global interrupt */
  VECTOR(stm32_adc1, STM32_IRQ_ADC1)          /* 18: ADC1 global interrupt */
  VECTOR(stm32_usbhp, STM32_IRQ_USBHP)        /* 19: USB High Priority interrupts */
  VECTOR(stm32_usblp, STM32_IRQ_USBLP)        /* 20: USB Low Priority interrupt */
  VECTOR(stm32_dac, STM32_IRQ_DAC)            /* 21: DAC interrupt */
  VECTOR(stm32_comp, STM32_IRQ_COMP)          /* 22: Comparator wakeup through EXTI interrupt */
  VECTOR(stm32_exti95, STM32_IRQ_EXTI95)      /* 23: EXTI Line[9:5] interrupts */
  VECTOR(stm32_ldc, STM32_IRQ_LDC)            /* 24: LCD global interrupt */
  VECTOR(stm32_tim9, STM32_IRQ_TIM9)          /* 25: TIM9 global interrupt */
  VECTOR(stm32_tim10, STM32_IRQ_TIM10)        /* 26: TIM10 global interrupt */
  VECTOR(stm32_tim11, STM32_IRQ_TIM11)        /* 27: TIM11 global interrupt */
  VECTOR(stm32_tim2, STM32_IRQ_TIM2)          /* 28: TIM2 global interrupt */
  VECTOR(stm32_tim3, STM32_IRQ_TIM3)          /* 29: TIM3 global interrupt */
  VECTOR(stm32_tim4, STM32_IRQ_TIM4)          /* 30: TIM4 global interrupt */
  VECTOR(stm32_i2c1ev, STM32_IRQ_I2C1EV)      /* 31: I2C1 event interrupt */
  VECTOR(stm32_i2c1er, STM32_IRQ_I2C1ER)      /* 32: I2C1 error interrupt */
  VECTOR(stm32_i2c2ev, STM32_IRQ_I2C2EV)      /* 33: I2C2 event interrupt */
  VECTOR(stm32_i2c2er, STM32_IRQ_I2C2ER)      /* 34: I2C2 error interrupt */
  VECTOR(stm32_spi1, STM32_IRQ_SPI1)          /* 35: SPI1 global interrupt */
  VECTOR(stm32_spi2, STM32_IRQ_SPI2)          /* 36: SPI2 global interrupt */
  VECTOR(stm32_usart1, STM32_IRQ_USART1)      /* 37: USART1 global interrupt */
  VECTOR(stm32_usart2, STM32_IRQ_USART2)      /* 38: USART2 global interrupt */
  VECTOR(stm32_usart3, STM32_IRQ_USART3)      /* 39: USART3 global interrupt */
  VECTOR(stm32_exti1510, STM32_IRQ_EXTI1510)  /* 40: EXTI Line[15:10] interrupts */
  VECTOR(stm32_rtcalrm, STM32_IRQ_RTCALRM)    /* 41: RTC alarm through EXTI line interrupt */
  VECTOR(stm32_usbwkup, STM32_IRQ_USBWKUP)    /* 42: USB wakeup from suspend through EXTI line interrupt */
  VECTOR(stm32_tim6, STM32_IRQ_TIM6)          /* 43: TIM6 global interrupt */
  VECTOR(stm32_tim7, STM32_IRQ_TIM7)          /* 44: TIM7 global interrupt */
  VECTOR(stm32_sdio, STM32_IRQ_SDIO)          /* 45: SDIO Global interrupt */
  VECTOR(stm32_tim5, STM32_IRQ_TIM5)          /* 46: TIM5 global interrupt */
  VECTOR(stm32_spi3, STM32_IRQ_SPI3)          /* 47: SPI3 global interrupt */
  VECTOR(stm32_usart4, STM32_IRQ_USART4)      /* 48: USART4 global interrupt */
  VECTOR(stm32_usart5, STM32_IRQ_USART5)      /* 49: USART5 global interrupt */
  VECTOR(stm32_dma2ch1, STM32_IRQ_DMA2CH1)    /* 50: DMA2 channel 1 global interrupt */
  VECTOR(stm32_dma2ch2, STM32_IRQ_DMA2CH2)    /* 51: DMA2 channel 2 global interrupt */
  VECTOR(stm32_dma2ch3, STM32_IRQ_DMA2CH3)    /* 52: DMA2 channel 3 global interrupt */
  VECTOR(stm32_dma2ch4, STM32_IRQ_DMA2CH4)    /* 53: DMA2 channel 4 global interrupt */
  VECTOR(stm32_dma2ch5, STM32_IRQ_DMA2CH5)    /* 54: DMA2 channel 5 global interrupt */
  VECTOR(stm32_aes, STM32_IRQ_AES)            /* 55: AES global interrupt */
  VECTOR(stm32_compacq, STM32_IRQ_COMPACQ)    /* 56: Comparator Channel Acquisition Interrupt */
#endif

#else
#  error "Unknown STM32L density"
#endif
