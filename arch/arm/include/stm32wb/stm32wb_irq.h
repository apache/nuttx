/****************************************************************************
 * arch/arm/include/stm32wb/stm32wb_irq.h
 *
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

#ifndef __ARCH_ARM_INCLUDE_STM32WB_STM32WB_IRQ_H
#define __ARCH_ARM_INCLUDE_STM32WB_STM32WB_IRQ_H

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
 * found in the file nuttx/arch/arm/include/stm32wb/irq.h which includes
 * this file
 *
 * External interrupts (vectors >= 16)
 *
 * These interrupts vectors was implemented based on RM0434 Table 61.
 *
 */

#define STM32WB_IRQ_WWDG        (STM32WB_IRQ_FIRST + 0)  /* 0:  Window Watchdog interrupt */
#define STM32WB_IRQ_PVD         (STM32WB_IRQ_FIRST + 1)  /* 1:  PVD through EXTI[16] Line detection interrupt */

#if defined(CONFIG_STM32WB_STM32WB35) || defined(CONFIG_STM32WB_STM32WB55)
#  define STM32WB_IRQ_PVM1      (STM32WB_IRQ_FIRST + 1)  /* 1:  PVM1 through EXTI[31] Line detection interrupt */
#endif

#if defined(CONFIG_STM32WB_STM32WB35) || defined(CONFIG_STM32WB_STM32WB55) \
    || defined(CONFIG_STM32WB_STM32WB15)
#  define STM32WB_IRQ_PVM3      (STM32WB_IRQ_FIRST + 1)  /* 1:  PVM3 through EXTI[33] Line detection interrupt */
#endif

#define STM32WB_IRQ_TAMPER      (STM32WB_IRQ_FIRST + 2)  /* 2:  Tamper through EXTI[18] interrupts */
#define STM32WB_IRQ_TIMESTAMP   (STM32WB_IRQ_FIRST + 2)  /* 2:  Time stamp through EXTI[18] interrupts */
#define STM32WB_IRQ_LSECSS      (STM32WB_IRQ_FIRST + 2)  /* 2:  LSECSS through EXTI[18] interrupts */
#define STM32WB_IRQ_RTC_WKUP    (STM32WB_IRQ_FIRST + 3)  /* 3:  RTC global interrupt */
#define STM32WB_IRQ_FLASH       (STM32WB_IRQ_FIRST + 4)  /* 4:  Flash global interrupt */
#define STM32WB_IRQ_RCC         (STM32WB_IRQ_FIRST + 5)  /* 5:  RCC global interrupt */
#define STM32WB_IRQ_EXTI0       (STM32WB_IRQ_FIRST + 6)  /* 6:  EXTI Line 0 interrupt */
#define STM32WB_IRQ_EXTI1       (STM32WB_IRQ_FIRST + 7)  /* 7:  EXTI Line 1 interrupt */
#define STM32WB_IRQ_EXTI2       (STM32WB_IRQ_FIRST + 8)  /* 8:  EXTI Line 2 interrupt */
#define STM32WB_IRQ_EXTI3       (STM32WB_IRQ_FIRST + 9)  /* 9:  EXTI Line 3 interrupt */
#define STM32WB_IRQ_EXTI4       (STM32WB_IRQ_FIRST + 10) /* 10: EXTI Line 4 interrupt */
#define STM32WB_IRQ_DMA1CH1     (STM32WB_IRQ_FIRST + 11) /* 11: DMA1 Channel 1 global interrupt */
#define STM32WB_IRQ_DMA1CH2     (STM32WB_IRQ_FIRST + 12) /* 12: DMA1 Channel 2 global interrupt */
#define STM32WB_IRQ_DMA1CH3     (STM32WB_IRQ_FIRST + 13) /* 13: DMA1 Channel 3 global interrupt */
#define STM32WB_IRQ_DMA1CH4     (STM32WB_IRQ_FIRST + 14) /* 14: DMA1 Channel 4 global interrupt */
#define STM32WB_IRQ_DMA1CH5     (STM32WB_IRQ_FIRST + 15) /* 15: DMA1 Channel 5 global interrupt */
#define STM32WB_IRQ_DMA1CH6     (STM32WB_IRQ_FIRST + 16) /* 16: DMA1 Channel 6 global interrupt */
#define STM32WB_IRQ_DMA1CH7     (STM32WB_IRQ_FIRST + 17) /* 17: DMA1 Channel 7 global interrupt */
#define STM32WB_IRQ_ADC1        (STM32WB_IRQ_FIRST + 18) /* 18: ADC1 global interrupt */

#if defined(CONFIG_STM32WB_STM32WB35) || defined(CONFIG_STM32WB_STM32WB55)
  #define STM32WB_IRQ_USB_HP    (STM32WB_IRQ_FIRST + 19) /* 19: USB High Priority Interrupt */
  #define STM32WB_IRQ_USB_LP    (STM32WB_IRQ_FIRST + 20) /* 20: USB Low Priority Interrupt */
#endif

#define STM32WB_IRQ_C2SEV       (STM32WB_IRQ_FIRST + 21) /* 21: CPU2 SEV Interrupt */

#if defined(CONFIG_STM32WB_STM32WB35) || defined(CONFIG_STM32WB_STM32WB55) \
    || defined(CONFIG_STM32WB_STM32WB15)
#  define STM32WB_IRQ_COMP      (STM32WB_IRQ_FIRST + 22) /* 22: COMP1/COMP2 Interrupts */
#endif

#define STM32WB_IRQ_EXTI95      (STM32WB_IRQ_FIRST + 23) /* 23: EXTI Lines [9:5] Interrupt */
#define STM32WB_IRQ_TIM1BRK     (STM32WB_IRQ_FIRST + 24) /* 24: TIM1 Break interrupt */
#define STM32WB_IRQ_TIM1UP      (STM32WB_IRQ_FIRST + 25) /* 25: TIM1 Update interrupt */
#define STM32WB_IRQ_TIM1TRGCOM  (STM32WB_IRQ_FIRST + 26) /* 26: TIM1 Trigger and Communication Interrupts */

#if defined(CONFIG_STM32WB_STM32WB30) || defined(CONFIG_STM32WB_STM32WB50) \
    || defined(CONFIG_STM32WB_STM32WB35) || defined(CONFIG_STM32WB_STM32WB55)
#  define STM32WB_IRQ_TIM16     (STM32WB_IRQ_FIRST + 25) /* 25: TIM16 global interrupt */
#  define STM32WB_IRQ_TIM17     (STM32WB_IRQ_FIRST + 26) /* 26: TIM17 global interrupt */
#endif

#define STM32WB_IRQ_TIM1CC      (STM32WB_IRQ_FIRST + 27) /* 27: TIM1 Capture Compare interrupt */
#define STM32WB_IRQ_TIM2        (STM32WB_IRQ_FIRST + 28) /* 28: TIM2 global interrupt */
#define STM32WB_IRQ_PKA         (STM32WB_IRQ_FIRST + 29) /* 29: PKA Interrupt */
#define STM32WB_IRQ_I2C1EV      (STM32WB_IRQ_FIRST + 30) /* 30: I2C1 event interrupt */
#define STM32WB_IRQ_I2C1ER      (STM32WB_IRQ_FIRST + 31) /* 31: I2C1 error interrupt */

#if defined(CONFIG_STM32WB_STM32WB35) || defined(CONFIG_STM32WB_STM32WB55)
#  define STM32WB_IRQ_I2C3EV    (STM32WB_IRQ_FIRST + 32) /* 32: I2C3 event interrupt */
#  define STM32WB_IRQ_I2C3ER    (STM32WB_IRQ_FIRST + 33) /* 33: I2C3 error interrupt */
#endif

#define STM32WB_IRQ_SPI1        (STM32WB_IRQ_FIRST + 34) /* 34: SPI1 global interrupt */

#if defined(CONFIG_STM32WB_STM32WB55)
#  define STM32WB_IRQ_SPI2      (STM32WB_IRQ_FIRST + 35) /* 35: SPI2 global interrupt */
#endif

#define STM32WB_IRQ_USART1      (STM32WB_IRQ_FIRST + 36) /* 36: USART1 global interrupt */

#if defined(CONFIG_STM32WB_STM32WB35) || defined(CONFIG_STM32WB_STM32WB55) \
    || defined(CONFIG_STM32WB_STM32WB15)
#  define STM32WB_IRQ_LPUART1   (STM32WB_IRQ_FIRST + 37) /* 37: LPUART1 global interrupt */
#endif

#if defined(CONFIG_STM32WB_STM32WB35) || defined(CONFIG_STM32WB_STM32WB55)
#  define STM32WB_IRQ_SAI1      (STM32WB_IRQ_FIRST + 38) /* 38: SAI1 A/B global interrupt */
#endif

#if defined(CONFIG_STM32WB_STM32WB10) || defined(CONFIG_STM32WB_STM32WB15) \
    || defined(CONFIG_STM32WB_STM32WB55)
#  define STM32WB_IRQ_TSC       (STM32WB_IRQ_FIRST + 39) /* 39: TSC global interrupt */
#endif

#define STM32WB_IRQ_EXTI1510    (STM32WB_IRQ_FIRST + 40) /* 40: EXTI Line[15:10] interrupts */
#define STM32WB_IRQ_RTCALRM     (STM32WB_IRQ_FIRST + 41) /* 41: RTC alarm A/B interrupt */

#if defined(CONFIG_STM32WB_STM32WB35) || defined(CONFIG_STM32WB_STM32WB55)
#  define STM32WB_IRQ_CRS       (STM32WB_IRQ_FIRST + 42) /* 42: CRS interrupt */
#endif

#define STM32WB_IRQ_PWRSOTF     (STM32WB_IRQ_FIRST + 43) /* 43: PWR switching on the fly interrupt */
#define STM32WB_IRQ_PWRBLEACT   (STM32WB_IRQ_FIRST + 43) /* 43: PWR end of BLE activity interrupt */
#define STM32WB_IRQ_PWRRFPHASE  (STM32WB_IRQ_FIRST + 43) /* 43: PWR end of critical radio phase interrupt */

#if defined(CONFIG_STM32WB_STM32WB30) || defined(CONFIG_STM32WB_STM32WB50) \
    || defined(CONFIG_STM32WB_STM32WB35) || defined(CONFIG_STM32WB_STM32WB55)
#  define STM32WB_IRQ_PWR802ACT (STM32WB_IRQ_FIRST + 43) /* 43: PWR end of 802.15.4 activity interrupt */
#endif

#define STM32WB_IRQ_IPCCRX      (STM32WB_IRQ_FIRST + 44) /* 44: IPCC RX occupied interrupt */
#define STM32WB_IRQ_IPCCTX      (STM32WB_IRQ_FIRST + 45) /* 45: IPCC TX free interrupt */
#define STM32WB_IRQ_HSEM        (STM32WB_IRQ_FIRST + 46) /* 46: Semaphore interrupt 0 to CPU1 */
#define STM32WB_IRQ_LPTIM1      (STM32WB_IRQ_FIRST + 47) /* 47: LPTIM1 global interrupt */
#define STM32WB_IRQ_LPTIM2      (STM32WB_IRQ_FIRST + 48) /* 48: LPTIM2 global interrupt */

#if defined(CONFIG_STM32WB_STM32WB55)
#  define STM32WB_IRQ_LCD       (STM32WB_IRQ_FIRST + 49) /* 49: LCD global interrupt */
#endif

#if defined(CONFIG_STM32WB_STM32WB35) || defined(CONFIG_STM32WB_STM32WB55)
#  define STM32WB_IRQ_QUADSPI   (STM32WB_IRQ_FIRST + 50) /* 50: QUADSPI global interrupt */
#  define STM32WB_IRQ_AES1      (STM32WB_IRQ_FIRST + 51) /* 51: AES1 crypto global interrupt */
#endif

#define STM32WB_IRQ_AES2        (STM32WB_IRQ_FIRST + 52) /* 52: AES2 crypto global interrupt */
#define STM32WB_IRQ_RNG         (STM32WB_IRQ_FIRST + 53) /* 53: RNG global interrupt */
#define STM32WB_IRQ_FPU         (STM32WB_IRQ_FIRST + 54) /* 54: FPU global interrupt */

#if defined(CONFIG_STM32WB_STM32WB35) || defined(CONFIG_STM32WB_STM32WB55)
#  define STM32WB_IRQ_DMA2CH1   (STM32WB_IRQ_FIRST + 55) /* 55: DMA2 Channel 1 global interrupt */
#  define STM32WB_IRQ_DMA2CH2   (STM32WB_IRQ_FIRST + 56) /* 56: DMA2 Channel 2 global interrupt */
#  define STM32WB_IRQ_DMA2CH3   (STM32WB_IRQ_FIRST + 57) /* 57: DMA2 Channel 3 global interrupt */
#  define STM32WB_IRQ_DMA2CH4   (STM32WB_IRQ_FIRST + 58) /* 58: DMA2 Channel 4 global interrupt */
#  define STM32WB_IRQ_DMA2CH5   (STM32WB_IRQ_FIRST + 59) /* 59: DMA2 Channel 5 global interrupt */
#  define STM32WB_IRQ_DMA2CH6   (STM32WB_IRQ_FIRST + 60) /* 60: DMA2 Channel 6 global interrupt */
#  define STM32WB_IRQ_DMA2CH7   (STM32WB_IRQ_FIRST + 61) /* 61: DMA2 Channel 7 global interrupt */
#endif

#define STM32WB_IRQ_DMAMUX1     (STM32WB_IRQ_FIRST + 62) /* 62: DMAMUX1 overrun interrupt */

#define STM32WB_IRQ_NEXTINTS    63

/* (EXTI interrupts do not use IRQ numbers) */

#define NR_IRQS                 (STM32WB_IRQ_FIRST + STM32WB_IRQ_NEXTINTS)

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

#endif /* __ARCH_ARM_INCLUDE_STM32WB_STM32WB_IRQ_H */
