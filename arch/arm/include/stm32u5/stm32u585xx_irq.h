/****************************************************************************
 * arch/arm/include/stm32u5/stm32u585xx_irq.h
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

#ifndef __ARCH_ARM_INCLUDE_STM32U5_STM32U585XX_IRQ_H
#define __ARCH_ARM_INCLUDE_STM32U5_STM32U585XX_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/stm32u5/stm32_irq.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* IRQ numbers.  The IRQ number corresponds vector number and hence map
 * directly to bits in the NVIC.  This does, however, waste several words of
 * memory in the IRQ to handle mapping tables.
 *
 * Processor Exceptions (vectors 0-15). These common definitions can be found
 * in the file nuttx/arch/arm/include/stm32u5/stm32_irq.h, which is
 * included above.
 *
 * External interrupts (vectors >= 16)
 *
 * These interrupt vectors were implemented based on RM0456 Table 153
 * (STM32U575/585 vector table) and should work for STM32U575xx and
 * STL32U585xx.
 */

#define STM32_IRQ_WWDG          (STM32_IRQ_FIRST + 0)   /* 0:   Window Watchdog interrupt */
#define STM32_IRQ_PVD_PVM       (STM32_IRQ_FIRST + 1)   /* 1:   PVD/PVM1/PVM2/PVM3/PVM4 */
#define STM32_IRQ_RTC           (STM32_IRQ_FIRST + 2)   /* 2:   RTC global interrupts */
#define STM32_IRQ_RTC_S         (STM32_IRQ_FIRST + 3)   /* 3:   RTC secure global interrupts */
#define STM32_IRQ_TAMP          (STM32_IRQ_FIRST + 4)   /* 4:   Tamper global interrupt */
#define STM32_IRQ_RAMCFG        (STM32_IRQ_FIRST + 5)   /* 5:   RAM configuration global interrupt */
#define STM32_IRQ_FLASH         (STM32_IRQ_FIRST + 6)   /* 6:   Flash memory global interrupt */
#define STM32_IRQ_FLASH_S       (STM32_IRQ_FIRST + 7)   /* 7:   Flash memory secure global interrupt */
#define STM32_IRQ_GTZC          (STM32_IRQ_FIRST + 8)   /* 8:   TZIC secure global interrupt */
#define STM32_IRQ_RCC           (STM32_IRQ_FIRST + 9)   /* 9:   RCC global interrupt */
#define STM32_IRQ_RCC_S         (STM32_IRQ_FIRST + 10)  /* 10:  RCC secure global interrupt */
#define STM32_IRQ_EXTI0         (STM32_IRQ_FIRST + 11)  /* 11:  EXTI Line 0 interrupt */
#define STM32_IRQ_EXTI1         (STM32_IRQ_FIRST + 12)  /* 12:  EXTI Line 1 interrupt */
#define STM32_IRQ_EXTI2         (STM32_IRQ_FIRST + 13)  /* 13:  EXTI Line 2 interrupt */
#define STM32_IRQ_EXTI3         (STM32_IRQ_FIRST + 14)  /* 14:  EXTI Line 3 interrupt */
#define STM32_IRQ_EXTI4         (STM32_IRQ_FIRST + 15)  /* 15:  EXTI Line 4 interrupt */
#define STM32_IRQ_EXTI5         (STM32_IRQ_FIRST + 16)  /* 16:  EXTI Line 5 interrupt */
#define STM32_IRQ_EXTI6         (STM32_IRQ_FIRST + 17)  /* 17:  EXTI Line 6 interrupt */
#define STM32_IRQ_EXTI7         (STM32_IRQ_FIRST + 18)  /* 18:  EXTI Line 7 interrupt */
#define STM32_IRQ_EXTI8         (STM32_IRQ_FIRST + 19)  /* 19:  EXTI Line 8 interrupt */
#define STM32_IRQ_EXTI9         (STM32_IRQ_FIRST + 20)  /* 20:  EXTI Line 9 interrupt */
#define STM32_IRQ_EXTI10        (STM32_IRQ_FIRST + 21)  /* 21:  EXTI Line 10 interrupt */
#define STM32_IRQ_EXTI11        (STM32_IRQ_FIRST + 22)  /* 22:  EXTI Line 11 interrupt */
#define STM32_IRQ_EXTI12        (STM32_IRQ_FIRST + 23)  /* 23:  EXTI Line 12 interrupt */
#define STM32_IRQ_EXTI13        (STM32_IRQ_FIRST + 24)  /* 24:  EXTI Line 13 interrupt */
#define STM32_IRQ_EXTI14        (STM32_IRQ_FIRST + 25)  /* 25:  EXTI Line 14 interrupt */
#define STM32_IRQ_EXTI15        (STM32_IRQ_FIRST + 26)  /* 26:  EXTI Line 15 interrupt */
#define STM32_IRQ_IWDG          (STM32_IRQ_FIRST + 27)  /* 27:  Independent watchdog interrupt */
#define STM32_IRQ_SAES          (STM32_IRQ_FIRST + 28)  /* 28:  Secure AES */
#define STM32_IRQ_GPDMA1_CH0    (STM32_IRQ_FIRST + 29)  /* 29:  GPDMA1 Channel 0 global interrupt */
#define STM32_IRQ_GPDMA1_CH1    (STM32_IRQ_FIRST + 30)  /* 30:  GPDMA1 Channel 1 global interrupt */
#define STM32_IRQ_GPDMA1_CH2    (STM32_IRQ_FIRST + 31)  /* 31:  GPDMA1 Channel 2 global interrupt */
#define STM32_IRQ_GPDMA1_CH3    (STM32_IRQ_FIRST + 32)  /* 32:  GPDMA1 Channel 3 global interrupt */
#define STM32_IRQ_GPDMA1_CH4    (STM32_IRQ_FIRST + 33)  /* 33:  GPDMA1 Channel 4 global interrupt */
#define STM32_IRQ_GPDMA1_CH5    (STM32_IRQ_FIRST + 34)  /* 34:  GPDMA1 Channel 5 global interrupt */
#define STM32_IRQ_GPDMA1_CH6    (STM32_IRQ_FIRST + 35)  /* 35:  GPDMA1 Channel 6 global interrupt */
#define STM32_IRQ_GPDMA1_CH7    (STM32_IRQ_FIRST + 36)  /* 36:  GPDMA1 Channel 7 global interrupt */
#define STM32_IRQ_ADC1          (STM32_IRQ_FIRST + 37)  /* 37:  ADC1 (14 bits) global interrupt */
#define STM32_IRQ_DAC1          (STM32_IRQ_FIRST + 38)  /* 38:  DAC1 global interrupt */
#define STM32_IRQ_FDCAN1_IT0    (STM32_IRQ_FIRST + 39)  /* 39:  FDCAN1 Interrupt 0 */
#define STM32_IRQ_FDCAN1_IT1    (STM32_IRQ_FIRST + 40)  /* 40:  FDCAN1 Interrupt 1 */
#define STM32_IRQ_TIM1_BRK      (STM32_IRQ_FIRST + 41)  /* 41:  TIM1 break */
#define STM32_IRQ_TIM1_TERR     (STM32_IRQ_FIRST + 41)  /* 41:  TIM1 transition error */
#define STM32_IRQ_TIM1_IERR     (STM32_IRQ_FIRST + 41)  /* 41:  TIM1 index error */
#define STM32_IRQ_TIM1_UP       (STM32_IRQ_FIRST + 42)  /* 42:  TIM1 update */
#define STM32_IRQ_TIM1_TRG_COM  (STM32_IRQ_FIRST + 43)  /* 43:  TIM1 trigger and communication */
#define STM32_IRQ_TIM1_DIR      (STM32_IRQ_FIRST + 43)  /* 43:  TIM1 direction change interrupt */
#define STM32_IRQ_TIM1_IDX      (STM32_IRQ_FIRST + 43)  /* 43:  TIM1 index */
#define STM32_IRQ_TIM1_CC       (STM32_IRQ_FIRST + 44)  /* 44:  TIM1 capture compare interrupt */
#define STM32_IRQ_TIM2          (STM32_IRQ_FIRST + 45)  /* 45:  TIM2 global interrupt */
#define STM32_IRQ_TIM3          (STM32_IRQ_FIRST + 46)  /* 46:  TIM3 global interrupt */
#define STM32_IRQ_TIM4          (STM32_IRQ_FIRST + 47)  /* 47:  TIM4 global interrupt */
#define STM32_IRQ_TIM5          (STM32_IRQ_FIRST + 48)  /* 48:  TIM5 global interrupt */
#define STM32_IRQ_TIM6          (STM32_IRQ_FIRST + 49)  /* 49:  TIM6 global interrupt */
#define STM32_IRQ_TIM7          (STM32_IRQ_FIRST + 50)  /* 50:  TIM7 global interrupt */
#define STM32_IRQ_TIM8_BRK      (STM32_IRQ_FIRST + 51)  /* 51:  TIM8 break */
#define STM32_IRQ_TIM8_TERR     (STM32_IRQ_FIRST + 51)  /* 51:  TIM8 transition error */
#define STM32_IRQ_TIM8_IERR     (STM32_IRQ_FIRST + 51)  /* 51:  TIM8 index error */
#define STM32_IRQ_TIM8_UP       (STM32_IRQ_FIRST + 52)  /* 52:  TIM8 update */
#define STM32_IRQ_TIM8_TRG_COM  (STM32_IRQ_FIRST + 53)  /* 53:  TIM8 trigger and communication */
#define STM32_IRQ_TIM8_DIR      (STM32_IRQ_FIRST + 53)  /* 53:  TIM8 direction change interrupt */
#define STM32_IRQ_TIM8_IDX      (STM32_IRQ_FIRST + 53)  /* 53:  TIM8 index */
#define STM32_IRQ_TIM8_CC       (STM32_IRQ_FIRST + 54)  /* 54:  TIM8 capture compare interrupt */
#define STM32_IRQ_I2C1_EV       (STM32_IRQ_FIRST + 55)  /* 55:  I2C1 event interrupt */
#define STM32_IRQ_I2C1_ER       (STM32_IRQ_FIRST + 56)  /* 56:  I2C1 error interrupt */
#define STM32_IRQ_I2C2_EV       (STM32_IRQ_FIRST + 57)  /* 57:  I2C2 event interrupt */
#define STM32_IRQ_I2C2_ER       (STM32_IRQ_FIRST + 58)  /* 58:  I2C2 error interrupt */
#define STM32_IRQ_SPI1          (STM32_IRQ_FIRST + 59)  /* 59:  SPI1 global interrupt */
#define STM32_IRQ_SPI2          (STM32_IRQ_FIRST + 60)  /* 60:  SPI2 global interrupt */
#define STM32_IRQ_USART1        (STM32_IRQ_FIRST + 61)  /* 61:  USART1 global interrupt */
#define STM32_IRQ_USART2        (STM32_IRQ_FIRST + 62)  /* 62:  USART2 global interrupt */
#define STM32_IRQ_USART3        (STM32_IRQ_FIRST + 63)  /* 63:  USART3 global interrupt */
#define STM32_IRQ_UART4         (STM32_IRQ_FIRST + 64)  /* 64:  UART4 global interrupt */
#define STM32_IRQ_UART5         (STM32_IRQ_FIRST + 65)  /* 65:  UART5 global interrupt */
#define STM32_IRQ_LPUART1       (STM32_IRQ_FIRST + 66)  /* 66:  LPUART 1 global interrupt */
#define STM32_IRQ_LPTIM1        (STM32_IRQ_FIRST + 67)  /* 67:  LPTIM1 global interrupt */
#define STM32_IRQ_LPTIM2        (STM32_IRQ_FIRST + 68)  /* 68:  LPTIM2 global interrupt */
#define STM32_IRQ_TIM15         (STM32_IRQ_FIRST + 69)  /* 69:  TIM15 global interrupt */
#define STM32_IRQ_TIM16         (STM32_IRQ_FIRST + 70)  /* 70:  TIM16 global interrupt */
#define STM32_IRQ_TIM17         (STM32_IRQ_FIRST + 71)  /* 71:  TIM17 global interrupt */
#define STM32_IRQ_COMP          (STM32_IRQ_FIRST + 72)  /* 72:  COMP1/COMP2 interrupts */
#define STM32_IRQ_OTG_FS        (STM32_IRQ_FIRST + 73)  /* 73:  USB OTG FS global interrupt */
#define STM32_IRQ_CRS           (STM32_IRQ_FIRST + 74)  /* 74:  CRS global interrupt */
#define STM32_IRQ_FMC           (STM32_IRQ_FIRST + 75)  /* 75:  FMC global interrupt */
#define STM32_IRQ_OCTOSPI1      (STM32_IRQ_FIRST + 76)  /* 76:  OCTOSPI1 global interrupt */
#define STM32_PWER_S3WU         (STM32_IRQ_FIRST + 77)  /* 77:  PWR wakeup from Stop 3 interrupt */
#define STM32_IRQ_SDMMC1        (STM32_IRQ_FIRST + 78)  /* 78:  SDMMC1 global interrupt */
#define STM32_IRQ_SDMMC2        (STM32_IRQ_FIRST + 79)  /* 79:  SDMMC2 global interrupt */
#define STM32_IRQ_GPDMA1_CH8    (STM32_IRQ_FIRST + 80)  /* 80:  GPDMA1 Channel 8 interrupt */
#define STM32_IRQ_GPDMA1_CH9    (STM32_IRQ_FIRST + 81)  /* 81:  GPDMA1 Channel 9 interrupt */
#define STM32_IRQ_GPDMA1_CH10   (STM32_IRQ_FIRST + 82)  /* 82:  GPDMA1 Channel 10 interrupt */
#define STM32_IRQ_GPDMA1_CH11   (STM32_IRQ_FIRST + 83)  /* 83:  GPDMA1 Channel 11 interrupt */
#define STM32_IRQ_GPDMA1_CH12   (STM32_IRQ_FIRST + 84)  /* 84:  GPDMA1 Channel 12 interrupt */
#define STM32_IRQ_GPDMA1_CH13   (STM32_IRQ_FIRST + 85)  /* 85:  GPDMA1 Channel 13 interrupt */
#define STM32_IRQ_GPDMA1_CH14   (STM32_IRQ_FIRST + 86)  /* 86:  GPDMA1 Channel 14 interrupt */
#define STM32_IRQ_GPDMA1_CH15   (STM32_IRQ_FIRST + 87)  /* 87:  GPDMA1 Channel 15 interrupt */
#define STM32_IRQ_I2C3_EV       (STM32_IRQ_FIRST + 88)  /* 88:  I2C3 event interrupt */
#define STM32_IRQ_I2C3_ER       (STM32_IRQ_FIRST + 89)  /* 89:  I2C3 error interrupt */
#define STM32_IRQ_SAI1          (STM32_IRQ_FIRST + 90)  /* 90:  SAI1 global interrupt */
#define STM32_IRQ_SAI2          (STM32_IRQ_FIRST + 91)  /* 91:  SAI2 global interrupt */
#define STM32_IRQ_TSC           (STM32_IRQ_FIRST + 92)  /* 92:  TSC global interrupt */
#define STM32_IRQ_AES           (STM32_IRQ_FIRST + 93)  /* 93:  AES global interrupt */
#define STM32_IRQ_RNG           (STM32_IRQ_FIRST + 94)  /* 94:  RNG global interrupt */
#define STM32_IRQ_FPU           (STM32_IRQ_FIRST + 95)  /* 95:  FPU global interrupt */
#define STM32_IRQ_HASH          (STM32_IRQ_FIRST + 96)  /* 96:  HASH global interrupt */
#define STM32_IRQ_PKA           (STM32_IRQ_FIRST + 97)  /* 97:  PKA global interrupt */
#define STM32_IRQ_LPTIM3        (STM32_IRQ_FIRST + 98)  /* 98:  LPTIM3 global interrupt */
#define STM32_IRQ_SPI3          (STM32_IRQ_FIRST + 99)  /* 99:  SPI3 global interrupt */
#define STM32_IRQ_I2C4_EV       (STM32_IRQ_FIRST + 100) /* 100: I2C4 event interrupt */
#define STM32_IRQ_I2C4_ER       (STM32_IRQ_FIRST + 101) /* 101: I2C4 error interrupt */
#define STM32_IRQ_MDF1_FLT0     (STM32_IRQ_FIRST + 102) /* 102: MDF1 filter 0 global interrupt */
#define STM32_IRQ_MDF1_FLT1     (STM32_IRQ_FIRST + 103) /* 103: MDF1 filter 1 global interrupt */
#define STM32_IRQ_MDF1_FLT2     (STM32_IRQ_FIRST + 104) /* 104: MDF1 filter 2 global interrupt */
#define STM32_IRQ_MDF1_FLT3     (STM32_IRQ_FIRST + 105) /* 105: MDF1 filter 3 global interrupt */
#define STM32_IRQ_UCPD1         (STM32_IRQ_FIRST + 106) /* 106: UCPD1 global interrupt */
#define STM32_IRQ_ICACHE        (STM32_IRQ_FIRST + 107) /* 107: Instruction cache global interrupt */
#define STM32_IRQ_OTFDEC1       (STM32_IRQ_FIRST + 108) /* 108: OTFDEC1 secure global interrupt */
#define STM32_IRQ_OTFDEC2       (STM32_IRQ_FIRST + 109) /* 109: OTFDEC2 secure global interrupt */
#define STM32_IRQ_LPTIM4        (STM32_IRQ_FIRST + 110) /* 110: LPTIM4 global interrupt */
#define STM32_IRQ_DCACHE1       (STM32_IRQ_FIRST + 111) /* 111: Data cache global interrupt */
#define STM32_IRQ_ADF1_FLT0     (STM32_IRQ_FIRST + 112) /* 112: ADF1 filter 0 global interrupt */
#define STM32_IRQ_ADC4          (STM32_IRQ_FIRST + 113) /* 113: ADC4 (12 bits) global interrupt */
#define STM32_IRQ_LPDMA1_CH0    (STM32_IRQ_FIRST + 114) /* 114: LPDMA1 SmartRun channel 0 global interrupt */
#define STM32_IRQ_LPDMA1_CH1    (STM32_IRQ_FIRST + 115) /* 115: LPDMA1 SmartRun channel 1 global interrupt */
#define STM32_IRQ_LPDMA1_CH2    (STM32_IRQ_FIRST + 116) /* 116: LPDMA1 SmartRun channel 2 global interrupt */
#define STM32_IRQ_LPDMA1_CH3    (STM32_IRQ_FIRST + 117) /* 117: LPDMA1 SmartRun channel 3 global interrupt */
#define STM32_IRQ_DMA2D         (STM32_IRQ_FIRST + 118) /* 118: DMA2D global interrupt */
#define STM32_IRQ_DCMI_PSSI     (STM32_IRQ_FIRST + 119) /* 119: DCMI/PSSI global interrupt */
#define STM32_IRQ_OCTOSPI2      (STM32_IRQ_FIRST + 120) /* 120: OCTOSPI2 global interrupt */
#define STM32_IRQ_MDF1_FLT4     (STM32_IRQ_FIRST + 121) /* 121: MDF1 filter 4 global interrupt */
#define STM32_IRQ_MDF1_FLT5     (STM32_IRQ_FIRST + 122) /* 122: MDF1 filter 5 global interrupt */
#define STM32_IRQ_CORDIC        (STM32_IRQ_FIRST + 123) /* 123: CORDIC interrupt */
#define STM32_IRQ_FMAC          (STM32_IRQ_FIRST + 124) /* 124: FMAC interrupt */

#if defined(CONFIG_STM32U5_STM32U585XX)
#  define STM32_IRQ_NEXTINTS  125
#else
#  error "Unsupported STM32U5 chip"
#endif

/* (EXTI interrupts do not use IRQ numbers) */

#define NR_IRQS                 (STM32_IRQ_FIRST + STM32_IRQ_NEXTINTS)

#endif /* __ARCH_ARM_INCLUDE_STM32U5_STM32U585XX_IRQ_H */
