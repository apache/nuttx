/****************************************************************************
 * arch/arm/include/stm32wl5/stm32wl5xxx_cpu1_irq.h

 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/* This file should never be included directly but, rather, only indirectly
 * through arch/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_STM32WL5_STM32WL5XXX_CPU1_IRQ_H
#define __ARCH_ARM_INCLUDE_STM32WL5_STM32WL5XXX_CPU1_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* IRQ numbers.  The IRQ number corresponds vector number and hence map
 * directly to bits in the NVIC.  This does, however, waste several words of
 * memory in the IRQ to handle mapping tables.
 *
 * Processor Exceptions (vectors 0-15).  These common definitions can be
 * found in the file nuttx/arch/arm/include/stm32wl5/irq.h which includes
 * this file
 *
 * External interrupts (vectors >= 16)
 */

#define STM32WL5_IRQ_WWDG              (STM32WL5_IRQ_FIRST + 0)  /*  0: Window watchdog early wakeup */
#define STM32WL5_IRQ_PVD               (STM32WL5_IRQ_FIRST + 1)  /*  1: PVD through EXTI[16] */
#define STM32WL5_IRQ_PVM               (STM32WL5_IRQ_FIRST + 1)  /*  1: PVM through EXTI[34] */
#define STM32WL5_IRQ_TAMPER            (STM32WL5_IRQ_FIRST + 2)  /*  2: Tamper */
#define STM32WL5_IRQ_LSE_CSS           (STM32WL5_IRQ_FIRST + 2)  /*  2: LSECSS */
#define STM32WL5_IRQ_RTC_STAMP         (STM32WL5_IRQ_FIRST + 2)  /*  2: timestamp */
#define STM32WL5_IRQ_RTC_SSRU          (STM32WL5_IRQ_FIRST + 2)  /*  2: RTC SSR underflow */
#define STM32WL5_IRQ_RTC_WKUP          (STM32WL5_IRQ_FIRST + 3)  /*  3: RTC wakeup interrupt */
#define STM32WL5_IRQ_FLASH             (STM32WL5_IRQ_FIRST + 4)  /*  4: Flash memory global interrupt and Flash memory ECC single error interrupt */
#define STM32WL5_IRQ_RCC               (STM32WL5_IRQ_FIRST + 5)  /*  5: RCC global interrupt */
#define STM32WL5_IRQ_EXTI0             (STM32WL5_IRQ_FIRST + 6)  /*  6: EXTI line 0 interrupt through EXTI[0] */
#define STM32WL5_IRQ_EXTI1             (STM32WL5_IRQ_FIRST + 7)  /*  7: EXTI line 1 interrupt through EXTI[1] */
#define STM32WL5_IRQ_EXTI2             (STM32WL5_IRQ_FIRST + 8)  /*  8: EXTI line 2 interrupt through EXTI[2] */
#define STM32WL5_IRQ_EXTI3             (STM32WL5_IRQ_FIRST + 9)  /*  9: EXTI line 3 interrupt through EXTI[3] */
#define STM32WL5_IRQ_EXTI4             (STM32WL5_IRQ_FIRST + 10) /* 10: EXTI line 4 interrupt through EXTI[4] */
#define STM32WL5_IRQ_DMA1CH1           (STM32WL5_IRQ_FIRST + 11) /* 11: DMA1 channel 1 non-secure interrupt */
#define STM32WL5_IRQ_DMA1CH2           (STM32WL5_IRQ_FIRST + 12) /* 12: DMA1 channel 2 non-secure interrupt */
#define STM32WL5_IRQ_DMA1CH3           (STM32WL5_IRQ_FIRST + 13) /* 13: DMA1 channel 3 non-secure interrupt */
#define STM32WL5_IRQ_DMA1CH4           (STM32WL5_IRQ_FIRST + 14) /* 14: DMA1 channel 4 non-secure interrupt */
#define STM32WL5_IRQ_DMA1CH5           (STM32WL5_IRQ_FIRST + 15) /* 15: DMA1 channel 5 non-secure interrupt */
#define STM32WL5_IRQ_DMA1CH6           (STM32WL5_IRQ_FIRST + 16) /* 16: DMA1 channel 6 non-secure interrupt */
#define STM32WL5_IRQ_DMA1CH7           (STM32WL5_IRQ_FIRST + 17) /* 17: DMA1 channel 7 non-secure interrupt */
#define STM32WL5_IRQ_ADC               (STM32WL5_IRQ_FIRST + 18) /* 18: ADC global interrupt */
#define STM32WL5_IRQ_DAC               (STM32WL5_IRQ_FIRST + 19) /* 19: DAC global interrupt */
#define STM32WL5_IRQ_C2SEV             (STM32WL5_IRQ_FIRST + 20) /* 20: CPU2 SEV through EXTI[40] */
#define STM32WL5_IRQ_PWRC2H            (STM32WL5_IRQ_FIRST + 20) /* 20: PWR CPU2 HOLD wakeup */
#define STM32WL5_IRQ_COMP              (STM32WL5_IRQ_FIRST + 21) /* 21: COMP2 and COMP1 interrupt through EXTI[22:21] */
#define STM32WL5_IRQ_EXTI95            (STM32WL5_IRQ_FIRST + 22) /* 22: EXTI line [9:5] interrupt through EXTI[9:5] */
#define STM32WL5_IRQ_TIM1BRK           (STM32WL5_IRQ_FIRST + 23) /* 23: Timer 1 break interrupt */
#define STM32WL5_IRQ_TIM1UP            (STM32WL5_IRQ_FIRST + 24) /* 24: Timer 1 Update */
#define STM32WL5_IRQ_TIM1TRG_COM       (STM32WL5_IRQ_FIRST + 25) /* 25: Timer 1 trigger and communication */
#define STM32WL5_IRQ_TIM1CC            (STM32WL5_IRQ_FIRST + 26) /* 26: Timer 1 capture compare interrupt */
#define STM32WL5_IRQ_TIM2              (STM32WL5_IRQ_FIRST + 27) /* 27: Timer 2 global interrupt */
#define STM32WL5_IRQ_TIM16             (STM32WL5_IRQ_FIRST + 28) /* 28: Timer 16 global interrupt */
#define STM32WL5_IRQ_TIM17             (STM32WL5_IRQ_FIRST + 29) /* 29: Timer 17 global interrupt */
#define STM32WL5_IRQ_I2C1EV            (STM32WL5_IRQ_FIRST + 30) /* 30: I2C1 event interrupt */
#define STM32WL5_IRQ_I2C1ER            (STM32WL5_IRQ_FIRST + 31) /* 31: I2C1 error interrupt */
#define STM32WL5_IRQ_I2C2EV            (STM32WL5_IRQ_FIRST + 32) /* 32: I2C2 event interrupt */
#define STM32WL5_IRQ_I2C2ER            (STM32WL5_IRQ_FIRST + 33) /* 33: I2C2 error interrupt */
#define STM32WL5_IRQ_SPI1              (STM32WL5_IRQ_FIRST + 34) /* 34: SPI1 global interrupt */
#define STM32WL5_IRQ_SPI2S2            (STM32WL5_IRQ_FIRST + 35) /* 35: SPI2S2 global interrupt */
#define STM32WL5_IRQ_USART1            (STM32WL5_IRQ_FIRST + 36) /* 36: USART1 global interrupt */
#define STM32WL5_IRQ_USART2            (STM32WL5_IRQ_FIRST + 37) /* 37: USART2 global interrupt */
#define STM32WL5_IRQ_LPUART1           (STM32WL5_IRQ_FIRST + 38) /* 38: LPUART1 global interrupt */
#define STM32WL5_IRQ_LPTIM1            (STM32WL5_IRQ_FIRST + 39) /* 39: LP timer 1 global interrupt */
#define STM32WL5_IRQ_LPTIM2            (STM32WL5_IRQ_FIRST + 40) /* 40: LP timer 2 global interrupt */
#define STM32WL5_IRQ_EXTI1510          (STM32WL5_IRQ_FIRST + 41) /* 41: EXTI line [15:10] interrupt through EXTI[15:10] (IMR1[31:26]) */
#define STM32WL5_IRQ_RTCALRM           (STM32WL5_IRQ_FIRST + 42) /* 42: RTC alarms A and B interrupt */
#define STM32WL5_IRQ_LPTIM3            (STM32WL5_IRQ_FIRST + 43) /* 43: LP timer 3 global interrupt */
                                                                 /* 44: Reserved */
#define STM32WL5_IRQ_IPCC_C1_RX_IT     (STM32WL5_IRQ_FIRST + 45) /* 45: IPCC CPU1 RX occupied interrupt */
#define STM32WL5_IRQ_IPCC_C1_TX_IT     (STM32WL5_IRQ_FIRST + 46) /* 46: IPCC CPU1 TX free interrupt */
#define STM32WL5_IRQ_HSEM              (STM32WL5_IRQ_FIRST + 47) /* 47: Semaphore interrupt 0 to CPU1 */
#define STM32WL5_IRQ_I2C3EV            (STM32WL5_IRQ_FIRST + 48) /* 48: I2C3 event interrupt */
#define STM32WL5_IRQ_I2C3ER            (STM32WL5_IRQ_FIRST + 49) /* 49: I2C3 error interrupt */
#define STM32WL5_IRQ_RADIO             (STM32WL5_IRQ_FIRST + 50) /* 50: Radio */
#define STM32WL5_IRQ_RFBUSY            (STM32WL5_IRQ_FIRST + 50) /* 50: RFBUSY interrupt through EXTI[45] */
#define STM32WL5_IRQ_AES               (STM32WL5_IRQ_FIRST + 51) /* 51: AES global interrupt */
#define STM32WL5_IRQ_RNG               (STM32WL5_IRQ_FIRST + 52) /* 52: True random number generator interrupt */
#define STM32WL5_IRQ_PKA               (STM32WL5_IRQ_FIRST + 53) /* 53: Private key accelerator interrupt */
#define STM32WL5_IRQ_DMA2CH1           (STM32WL5_IRQ_FIRST + 54) /* 54: DMA2 channel 1 non-secure interrupt */
#define STM32WL5_IRQ_DMA2CH2           (STM32WL5_IRQ_FIRST + 55) /* 55: DMA2 channel 2 non-secure interrupt */
#define STM32WL5_IRQ_DMA2CH3           (STM32WL5_IRQ_FIRST + 56) /* 56: DMA2 channel 3 non-secure interrupt */
#define STM32WL5_IRQ_DMA2CH4           (STM32WL5_IRQ_FIRST + 57) /* 57: DMA2 channel 4 non-secure interrupt */
#define STM32WL5_IRQ_DMA2CH5           (STM32WL5_IRQ_FIRST + 58) /* 58: DMA2 channel 5 non-secure interrupt */
#define STM32WL5_IRQ_DMA2CH6           (STM32WL5_IRQ_FIRST + 59) /* 59: DMA2 channel 6 non-secure interrupt */
#define STM32WL5_IRQ_DMA2CH7           (STM32WL5_IRQ_FIRST + 60) /* 60: DMA2 channel 7 non-secure interrupt */
#define STM32WL5_IRQ_DMAMUX1_OVR       (STM32WL5_IRQ_FIRST + 61) /* 61: DMAMUX1 overrun interrupt */

#define STM32WL5_IRQ_NEXTINTS  62

/* (EXTI interrupts do not use IRQ numbers) */

#define NR_IRQS                 (STM32WL5_IRQ_FIRST + STM32WL5_IRQ_NEXTINTS)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_STM32WL5_STM32WL5XXX_CPU1_IRQ_H */
