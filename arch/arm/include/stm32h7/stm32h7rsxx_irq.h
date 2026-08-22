/****************************************************************************
 * arch/arm/include/stm32h7/stm32h7rsxx_irq.h
 *
 * SPDX-License-Identifier: Apache-2.0
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
 * through arch/irq.h.
 */

#ifndef __ARCH_ARM_INCLUDE_STM32H7_STM32H7RSXX_IRQ_H
#define __ARCH_ARM_INCLUDE_STM32H7_STM32H7RSXX_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IRQ numbers.  The IRQ number corresponds to the vector number and maps
 * directly to bits in the NVIC.
 *
 * Processor exceptions (vectors 0-15) are defined in stm32h7/irq.h.
 * External interrupts begin at vector 16.
 */

#define STM32_IRQ_PVDPVM          (STM32_IRQ_FIRST + 0)   /* 0: PVD/PVM through EXTI */
#define STM32_IRQ_RESERVED1       (STM32_IRQ_FIRST + 1)   /* 1: Reserved */
#define STM32_IRQ_DTS             (STM32_IRQ_FIRST + 2)   /* 2: Digital temperature sensor */
#define STM32_IRQ_IWDG            (STM32_IRQ_FIRST + 3)   /* 3: Independent watchdog */
#define STM32_IRQ_WWDG            (STM32_IRQ_FIRST + 4)   /* 4: Window watchdog */
#define STM32_IRQ_RCC             (STM32_IRQ_FIRST + 5)   /* 5: RCC global interrupt */
#define STM32_IRQ_RESERVED6       (STM32_IRQ_FIRST + 6)   /* 6: Reserved */
#define STM32_IRQ_RESERVED7       (STM32_IRQ_FIRST + 7)   /* 7: Reserved */
#define STM32_IRQ_FLASH           (STM32_IRQ_FIRST + 8)   /* 8: FLASH global interrupt */
#define STM32_IRQ_RAMECC          (STM32_IRQ_FIRST + 9)   /* 9: RAM ECC interrupt */
#define STM32_IRQ_FPU             (STM32_IRQ_FIRST + 10)  /* 10: FPU interrupt */
#define STM32_IRQ_RESERVED11      (STM32_IRQ_FIRST + 11)  /* 11: Reserved */
#define STM32_IRQ_RESERVED12      (STM32_IRQ_FIRST + 12)  /* 12: Reserved */
#define STM32_IRQ_TAMP            (STM32_IRQ_FIRST + 13)  /* 13: Tamper and timestamp */
#define STM32_IRQ_RESERVED14      (STM32_IRQ_FIRST + 14)  /* 14: Reserved */
#define STM32_IRQ_RESERVED15      (STM32_IRQ_FIRST + 15)  /* 15: Reserved */
#define STM32_IRQ_EXTI0           (STM32_IRQ_FIRST + 16)  /* 16: EXTI line 0 */
#define STM32_IRQ_EXTI1           (STM32_IRQ_FIRST + 17)  /* 17: EXTI line 1 */
#define STM32_IRQ_EXTI2           (STM32_IRQ_FIRST + 18)  /* 18: EXTI line 2 */
#define STM32_IRQ_EXTI3           (STM32_IRQ_FIRST + 19)  /* 19: EXTI line 3 */
#define STM32_IRQ_EXTI4           (STM32_IRQ_FIRST + 20)  /* 20: EXTI line 4 */
#define STM32_IRQ_EXTI5           (STM32_IRQ_FIRST + 21)  /* 21: EXTI line 5 */
#define STM32_IRQ_EXTI6           (STM32_IRQ_FIRST + 22)  /* 22: EXTI line 6 */
#define STM32_IRQ_EXTI7           (STM32_IRQ_FIRST + 23)  /* 23: EXTI line 7 */
#define STM32_IRQ_EXTI8           (STM32_IRQ_FIRST + 24)  /* 24: EXTI line 8 */
#define STM32_IRQ_EXTI9           (STM32_IRQ_FIRST + 25)  /* 25: EXTI line 9 */
#define STM32_IRQ_EXTI10          (STM32_IRQ_FIRST + 26)  /* 26: EXTI line 10 */
#define STM32_IRQ_EXTI11          (STM32_IRQ_FIRST + 27)  /* 27: EXTI line 11 */
#define STM32_IRQ_EXTI12          (STM32_IRQ_FIRST + 28)  /* 28: EXTI line 12 */
#define STM32_IRQ_EXTI13          (STM32_IRQ_FIRST + 29)  /* 29: EXTI line 13 */
#define STM32_IRQ_EXTI14          (STM32_IRQ_FIRST + 30)  /* 30: EXTI line 14 */
#define STM32_IRQ_EXTI15          (STM32_IRQ_FIRST + 31)  /* 31: EXTI line 15 */
#define STM32_IRQ_RTC             (STM32_IRQ_FIRST + 32)  /* 32: RTC wakeup and alarm */
#define STM32_IRQ_RTCWKUP         STM32_IRQ_RTC           /* 32: RTC wakeup */
#define STM32_IRQ_RTCALARM        STM32_IRQ_RTC           /* 32: RTC alarm */
#define STM32_IRQ_SAES            (STM32_IRQ_FIRST + 33)  /* 33: Secure AES */
#define STM32_IRQ_CRYP            (STM32_IRQ_FIRST + 34)  /* 34: Cryptographic processor */
#define STM32_IRQ_PKA             (STM32_IRQ_FIRST + 35)  /* 35: Public key accelerator */
#define STM32_IRQ_HASH            (STM32_IRQ_FIRST + 36)  /* 36: Hash processor */
#define STM32_IRQ_RNG             (STM32_IRQ_FIRST + 37)  /* 37: Random number generator */
#define STM32_IRQ_ADC12           (STM32_IRQ_FIRST + 38)  /* 38: ADC1 and ADC2 */
#define STM32_IRQ_GPDMA1CH0       (STM32_IRQ_FIRST + 39)  /* 39: GPDMA1 channel 0 */
#define STM32_IRQ_GPDMA1CH1       (STM32_IRQ_FIRST + 40)  /* 40: GPDMA1 channel 1 */
#define STM32_IRQ_GPDMA1CH2       (STM32_IRQ_FIRST + 41)  /* 41: GPDMA1 channel 2 */
#define STM32_IRQ_GPDMA1CH3       (STM32_IRQ_FIRST + 42)  /* 42: GPDMA1 channel 3 */
#define STM32_IRQ_GPDMA1CH4       (STM32_IRQ_FIRST + 43)  /* 43: GPDMA1 channel 4 */
#define STM32_IRQ_GPDMA1CH5       (STM32_IRQ_FIRST + 44)  /* 44: GPDMA1 channel 5 */
#define STM32_IRQ_GPDMA1CH6       (STM32_IRQ_FIRST + 45)  /* 45: GPDMA1 channel 6 */
#define STM32_IRQ_GPDMA1CH7       (STM32_IRQ_FIRST + 46)  /* 46: GPDMA1 channel 7 */
#define STM32_IRQ_TIM1BRK         (STM32_IRQ_FIRST + 47)  /* 47: TIM1 break */
#define STM32_IRQ_TIM1UP          (STM32_IRQ_FIRST + 48)  /* 48: TIM1 update */
#define STM32_IRQ_TIM1TRGCOM      (STM32_IRQ_FIRST + 49)  /* 49: TIM1 trigger/commutation */
#define STM32_IRQ_TIM1CC          (STM32_IRQ_FIRST + 50)  /* 50: TIM1 capture/compare */
#define STM32_IRQ_TIM2            (STM32_IRQ_FIRST + 51)  /* 51: TIM2 global interrupt */
#define STM32_IRQ_TIM3            (STM32_IRQ_FIRST + 52)  /* 52: TIM3 global interrupt */
#define STM32_IRQ_TIM4            (STM32_IRQ_FIRST + 53)  /* 53: TIM4 global interrupt */
#define STM32_IRQ_TIM5            (STM32_IRQ_FIRST + 54)  /* 54: TIM5 global interrupt */
#define STM32_IRQ_TIM6            (STM32_IRQ_FIRST + 55)  /* 55: TIM6 global interrupt */
#define STM32_IRQ_TIM7            (STM32_IRQ_FIRST + 56)  /* 56: TIM7 global interrupt */
#define STM32_IRQ_TIM9            (STM32_IRQ_FIRST + 57)  /* 57: TIM9 global interrupt */
#define STM32_IRQ_SPI1            (STM32_IRQ_FIRST + 58)  /* 58: SPI1 global interrupt */
#define STM32_IRQ_SPI2            (STM32_IRQ_FIRST + 59)  /* 59: SPI2 global interrupt */
#define STM32_IRQ_SPI3            (STM32_IRQ_FIRST + 60)  /* 60: SPI3 global interrupt */
#define STM32_IRQ_SPI4            (STM32_IRQ_FIRST + 61)  /* 61: SPI4 global interrupt */
#define STM32_IRQ_SPI5            (STM32_IRQ_FIRST + 62)  /* 62: SPI5 global interrupt */
#define STM32_IRQ_SPI6            (STM32_IRQ_FIRST + 63)  /* 63: SPI6 global interrupt */
#define STM32_IRQ_HPDMA1CH0       (STM32_IRQ_FIRST + 64)  /* 64: HPDMA1 channel 0 */
#define STM32_IRQ_HPDMA1CH1       (STM32_IRQ_FIRST + 65)  /* 65: HPDMA1 channel 1 */
#define STM32_IRQ_HPDMA1CH2       (STM32_IRQ_FIRST + 66)  /* 66: HPDMA1 channel 2 */
#define STM32_IRQ_HPDMA1CH3       (STM32_IRQ_FIRST + 67)  /* 67: HPDMA1 channel 3 */
#define STM32_IRQ_HPDMA1CH4       (STM32_IRQ_FIRST + 68)  /* 68: HPDMA1 channel 4 */
#define STM32_IRQ_HPDMA1CH5       (STM32_IRQ_FIRST + 69)  /* 69: HPDMA1 channel 5 */
#define STM32_IRQ_HPDMA1CH6       (STM32_IRQ_FIRST + 70)  /* 70: HPDMA1 channel 6 */
#define STM32_IRQ_HPDMA1CH7       (STM32_IRQ_FIRST + 71)  /* 71: HPDMA1 channel 7 */
#define STM32_IRQ_SAI1A           (STM32_IRQ_FIRST + 72)  /* 72: SAI1 block A */
#define STM32_IRQ_SAI1B           (STM32_IRQ_FIRST + 73)  /* 73: SAI1 block B */
#define STM32_IRQ_SAI2A           (STM32_IRQ_FIRST + 74)  /* 74: SAI2 block A */
#define STM32_IRQ_SAI2B           (STM32_IRQ_FIRST + 75)  /* 75: SAI2 block B */
#define STM32_IRQ_I2C1EV          (STM32_IRQ_FIRST + 76)  /* 76: I2C1 event */
#define STM32_IRQ_I2C1ER          (STM32_IRQ_FIRST + 77)  /* 77: I2C1 error */
#define STM32_IRQ_I2C2EV          (STM32_IRQ_FIRST + 78)  /* 78: I2C2 event */
#define STM32_IRQ_I2C2ER          (STM32_IRQ_FIRST + 79)  /* 79: I2C2 error */
#define STM32_IRQ_I2C3EV          (STM32_IRQ_FIRST + 80)  /* 80: I2C3 event */
#define STM32_IRQ_I2C3ER          (STM32_IRQ_FIRST + 81)  /* 81: I2C3 error */
#define STM32_IRQ_USART1          (STM32_IRQ_FIRST + 82)  /* 82: USART1 global interrupt */
#define STM32_IRQ_USART2          (STM32_IRQ_FIRST + 83)  /* 83: USART2 global interrupt */
#define STM32_IRQ_USART3          (STM32_IRQ_FIRST + 84)  /* 84: USART3 global interrupt */
#define STM32_IRQ_UART4           (STM32_IRQ_FIRST + 85)  /* 85: UART4 global interrupt */
#define STM32_IRQ_UART5           (STM32_IRQ_FIRST + 86)  /* 86: UART5 global interrupt */
#define STM32_IRQ_UART7           (STM32_IRQ_FIRST + 87)  /* 87: UART7 global interrupt */
#define STM32_IRQ_UART8           (STM32_IRQ_FIRST + 88)  /* 88: UART8 global interrupt */
#define STM32_IRQ_I3C1EV          (STM32_IRQ_FIRST + 89)  /* 89: I3C1 event */
#define STM32_IRQ_I3C1ER          (STM32_IRQ_FIRST + 90)  /* 90: I3C1 error */
#define STM32_IRQ_OTGHS           (STM32_IRQ_FIRST + 91)  /* 91: USB OTG HS */
#define STM32_IRQ_ETH             (STM32_IRQ_FIRST + 92)  /* 92: Ethernet global interrupt */
#define STM32_IRQ_CORDIC          (STM32_IRQ_FIRST + 93)  /* 93: CORDIC global interrupt */
#define STM32_IRQ_GFXTIM          (STM32_IRQ_FIRST + 94)  /* 94: Graphics timer */
#define STM32_IRQ_DCMIPP          (STM32_IRQ_FIRST + 95)  /* 95: DCMIPP global interrupt */
#define STM32_IRQ_LTDC            (STM32_IRQ_FIRST + 96)  /* 96: LTDC global interrupt */
#define STM32_IRQ_LTDCER          (STM32_IRQ_FIRST + 97)  /* 97: LTDC error interrupt */
#define STM32_IRQ_DMA2D           (STM32_IRQ_FIRST + 98)  /* 98: DMA2D global interrupt */
#define STM32_IRQ_JPEG            (STM32_IRQ_FIRST + 99)  /* 99: JPEG global interrupt */
#define STM32_IRQ_GFXMMU          (STM32_IRQ_FIRST + 100) /* 100: Graphics MMU */
#define STM32_IRQ_I3C1WKUP        (STM32_IRQ_FIRST + 101) /* 101: I3C1 wakeup */
#define STM32_IRQ_MCE1            (STM32_IRQ_FIRST + 102) /* 102: MCE1 global interrupt */
#define STM32_IRQ_MCE2            (STM32_IRQ_FIRST + 103) /* 103: MCE2 global interrupt */
#define STM32_IRQ_MCE3            (STM32_IRQ_FIRST + 104) /* 104: MCE3 global interrupt */
#define STM32_IRQ_XSPI1           (STM32_IRQ_FIRST + 105) /* 105: XSPI1 global interrupt */
#define STM32_IRQ_XSPI2           (STM32_IRQ_FIRST + 106) /* 106: XSPI2 global interrupt */
#define STM32_IRQ_FMC             (STM32_IRQ_FIRST + 107) /* 107: FMC global interrupt */
#define STM32_IRQ_SDMMC1          (STM32_IRQ_FIRST + 108) /* 108: SDMMC1 global interrupt */
#define STM32_IRQ_SDMMC2          (STM32_IRQ_FIRST + 109) /* 109: SDMMC2 global interrupt */
#define STM32_IRQ_RESERVED110     (STM32_IRQ_FIRST + 110) /* 110: Reserved */
#define STM32_IRQ_RESERVED111     (STM32_IRQ_FIRST + 111) /* 111: Reserved */
#define STM32_IRQ_OTGFS           (STM32_IRQ_FIRST + 112) /* 112: USB OTG FS */
#define STM32_IRQ_TIM12           (STM32_IRQ_FIRST + 113) /* 113: TIM12 global interrupt */
#define STM32_IRQ_TIM13           (STM32_IRQ_FIRST + 114) /* 114: TIM13 global interrupt */
#define STM32_IRQ_TIM14           (STM32_IRQ_FIRST + 115) /* 115: TIM14 global interrupt */
#define STM32_IRQ_TIM15           (STM32_IRQ_FIRST + 116) /* 116: TIM15 global interrupt */
#define STM32_IRQ_TIM16           (STM32_IRQ_FIRST + 117) /* 117: TIM16 global interrupt */
#define STM32_IRQ_TIM17           (STM32_IRQ_FIRST + 118) /* 118: TIM17 global interrupt */
#define STM32_IRQ_LPTIM1          (STM32_IRQ_FIRST + 119) /* 119: LPTIM1 global interrupt */
#define STM32_IRQ_LPTIM2          (STM32_IRQ_FIRST + 120) /* 120: LPTIM2 global interrupt */
#define STM32_IRQ_LPTIM3          (STM32_IRQ_FIRST + 121) /* 121: LPTIM3 global interrupt */
#define STM32_IRQ_LPTIM4          (STM32_IRQ_FIRST + 122) /* 122: LPTIM4 global interrupt */
#define STM32_IRQ_LPTIM5          (STM32_IRQ_FIRST + 123) /* 123: LPTIM5 global interrupt */
#define STM32_IRQ_SPDIF           (STM32_IRQ_FIRST + 124) /* 124: SPDIF receiver */
#define STM32_IRQ_MDIOS           (STM32_IRQ_FIRST + 125) /* 125: MDIOS global interrupt */
#define STM32_IRQ_ADF1FLT0        (STM32_IRQ_FIRST + 126) /* 126: ADF1 filter 0 */
#define STM32_IRQ_CRS             (STM32_IRQ_FIRST + 127) /* 127: Clock recovery system */
#define STM32_IRQ_UCPD1           (STM32_IRQ_FIRST + 128) /* 128: USB-C power delivery */
#define STM32_IRQ_CEC             (STM32_IRQ_FIRST + 129) /* 129: HDMI CEC */
#define STM32_IRQ_PSSI            (STM32_IRQ_FIRST + 130) /* 130: Parallel synchronous input */
#define STM32_IRQ_LPUART1         (STM32_IRQ_FIRST + 131) /* 131: LPUART1 global interrupt */
#define STM32_IRQ_WKUP            (STM32_IRQ_FIRST + 132) /* 132: Wakeup pins */
#define STM32_IRQ_GPDMA1CH8       (STM32_IRQ_FIRST + 133) /* 133: GPDMA1 channel 8 */
#define STM32_IRQ_GPDMA1CH9       (STM32_IRQ_FIRST + 134) /* 134: GPDMA1 channel 9 */
#define STM32_IRQ_GPDMA1CH10      (STM32_IRQ_FIRST + 135) /* 135: GPDMA1 channel 10 */
#define STM32_IRQ_GPDMA1CH11      (STM32_IRQ_FIRST + 136) /* 136: GPDMA1 channel 11 */
#define STM32_IRQ_GPDMA1CH12      (STM32_IRQ_FIRST + 137) /* 137: GPDMA1 channel 12 */
#define STM32_IRQ_GPDMA1CH13      (STM32_IRQ_FIRST + 138) /* 138: GPDMA1 channel 13 */
#define STM32_IRQ_GPDMA1CH14      (STM32_IRQ_FIRST + 139) /* 139: GPDMA1 channel 14 */
#define STM32_IRQ_GPDMA1CH15      (STM32_IRQ_FIRST + 140) /* 140: GPDMA1 channel 15 */
#define STM32_IRQ_HPDMA1CH8       (STM32_IRQ_FIRST + 141) /* 141: HPDMA1 channel 8 */
#define STM32_IRQ_HPDMA1CH9       (STM32_IRQ_FIRST + 142) /* 142: HPDMA1 channel 9 */
#define STM32_IRQ_HPDMA1CH10      (STM32_IRQ_FIRST + 143) /* 143: HPDMA1 channel 10 */
#define STM32_IRQ_HPDMA1CH11      (STM32_IRQ_FIRST + 144) /* 144: HPDMA1 channel 11 */
#define STM32_IRQ_HPDMA1CH12      (STM32_IRQ_FIRST + 145) /* 145: HPDMA1 channel 12 */
#define STM32_IRQ_HPDMA1CH13      (STM32_IRQ_FIRST + 146) /* 146: HPDMA1 channel 13 */
#define STM32_IRQ_HPDMA1CH14      (STM32_IRQ_FIRST + 147) /* 147: HPDMA1 channel 14 */
#define STM32_IRQ_HPDMA1CH15      (STM32_IRQ_FIRST + 148) /* 148: HPDMA1 channel 15 */
#define STM32_IRQ_GPU2D           (STM32_IRQ_FIRST + 149) /* 149: GPU2D global interrupt */
#define STM32_IRQ_GPU2DER         (STM32_IRQ_FIRST + 150) /* 150: GPU2D error interrupt */
#define STM32_IRQ_ICACHE          (STM32_IRQ_FIRST + 151) /* 151: ICACHE global interrupt */
#define STM32_IRQ_FDCAN1_0        (STM32_IRQ_FIRST + 152) /* 152: FDCAN1 interrupt 0 */
#define STM32_IRQ_FDCAN1_1        (STM32_IRQ_FIRST + 153) /* 153: FDCAN1 interrupt 1 */
#define STM32_IRQ_FDCAN2_0        (STM32_IRQ_FIRST + 154) /* 154: FDCAN2 interrupt 0 */
#define STM32_IRQ_FDCAN2_1        (STM32_IRQ_FIRST + 155) /* 155: FDCAN2 interrupt 1 */

#define STM32_IRQ_NEXTINTS        156
#define NR_IRQS                   (STM32_IRQ_FIRST + STM32_IRQ_NEXTINTS)

#endif /* __ARCH_ARM_INCLUDE_STM32H7_STM32H7RSXX_IRQ_H */
