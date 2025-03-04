/****************************************************************************
 * arch/arm/include/at32/at32f43xxx_irq.h
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

/* This file should never be included directly but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_AT32_AT32F43XXX_IRQ_H
#define __ARCH_ARM_INCLUDE_AT32_AT32F43XXX_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* IRQ numbers.
 * The IRQ number corresponds vector number and hence map directly to
 * bits in the NVIC.  This does, however, waste several words of memory in
 * the IRQ to handle mapping tables.
 *
 * Processor Exceptions (vectors 0-15). These common definitions can
 * be found in nuttx/arch/arm/include/at32/irq.h
 *
 * External interrupts (vectors >= 16)
 */

#define AT32_IRQ_WWDG           (AT32_IRQ_FIRST+0)   /* 0:  Window Watchdog interrupt */
#define AT32_IRQ_PVD            (AT32_IRQ_FIRST+1)   /* 1:  PVD through EXTI Line detection interrupt */
#define AT32_IRQ_TAMPER         (AT32_IRQ_FIRST+2)   /* 2:  Tamper and time stamp interrupts */
#define AT32_IRQ_TIMESTAMP      (AT32_IRQ_FIRST+2)   /* 2:  Tamper and time stamp interrupts */
#define AT32_IRQ_RTC_WKUP       (AT32_IRQ_FIRST+3)   /* 3:  RTC global interrupt */
#define AT32_IRQ_FLASH          (AT32_IRQ_FIRST+4)   /* 4:  Flash global interrupt */
#define AT32_IRQ_RCC            (AT32_IRQ_FIRST+5)   /* 5:  RCC global interrupt */
#define AT32_IRQ_EXTI0          (AT32_IRQ_FIRST+6)   /* 6:  EXTI Line 0 interrupt */
#define AT32_IRQ_EXTI1          (AT32_IRQ_FIRST+7)   /* 7:  EXTI Line 1 interrupt */
#define AT32_IRQ_EXTI2          (AT32_IRQ_FIRST+8)   /* 8:  EXTI Line 2 interrupt */
#define AT32_IRQ_EXTI3          (AT32_IRQ_FIRST+9)   /* 9:  EXTI Line 3 interrupt */
#define AT32_IRQ_EXTI4          (AT32_IRQ_FIRST+10)  /* 10: EXTI Line 4 interrupt */
#define AT32_IRQ_EDMAS1         (AT32_IRQ_FIRST+11)  /* 11: EDMA Stream 1 global interrupt */
#define AT32_IRQ_EDMAS2         (AT32_IRQ_FIRST+12)  /* 12: EDMA Stream 2 global interrupt */
#define AT32_IRQ_EDMAS3         (AT32_IRQ_FIRST+13)  /* 13: EDMA Stream 3 global interrupt */
#define AT32_IRQ_EDMAS4         (AT32_IRQ_FIRST+14)  /* 14: EDMA Stream 4 global interrupt */
#define AT32_IRQ_EDMAS5         (AT32_IRQ_FIRST+15)  /* 15: EDMA Stream 5 global interrupt */
#define AT32_IRQ_EDMAS6         (AT32_IRQ_FIRST+16)  /* 16: EDMA Stream 6 global interrupt */
#define AT32_IRQ_EDMAS7         (AT32_IRQ_FIRST+17)  /* 17: EDMA Stream 7 global interrupt */
#define AT32_IRQ_ADC            (AT32_IRQ_FIRST+18)  /* 18: ADC1, ADC2, and ADC3 global interrupt */
#define AT32_IRQ_CAN1TX         (AT32_IRQ_FIRST+19)  /* 19: CAN1 TX interrupts */
#define AT32_IRQ_CAN1RX0        (AT32_IRQ_FIRST+20)  /* 20: CAN1 RX0 interrupts */
#define AT32_IRQ_CAN1RX1        (AT32_IRQ_FIRST+21)  /* 21: CAN1 RX1 interrupt */
#define AT32_IRQ_CAN1SCE        (AT32_IRQ_FIRST+22)  /* 22: CAN1 SCE interrupt */
#define AT32_IRQ_EXTI95         (AT32_IRQ_FIRST+23)  /* 23: EXTI Line[9:5] interrupts */
#define AT32_IRQ_TIM1BRK        (AT32_IRQ_FIRST+24)  /* 24: TIM1 Break interrupt */
#define AT32_IRQ_TIM9           (AT32_IRQ_FIRST+24)  /* 24: TIM9 global interrupt */
#define AT32_IRQ_TIM1UP         (AT32_IRQ_FIRST+25)  /* 25: TIM1 Update interrupt */
#define AT32_IRQ_TIM10          (AT32_IRQ_FIRST+25)  /* 25: TIM10 global interrupt */
#define AT32_IRQ_TIM1TRGCOM     (AT32_IRQ_FIRST+26)  /* 26: TIM1 Trigger and Commutation interrupts */
#define AT32_IRQ_TIM11          (AT32_IRQ_FIRST+26)  /* 26: TIM11 global interrupt */
#define AT32_IRQ_TIM1CC         (AT32_IRQ_FIRST+27)  /* 27: TIM1 Capture Compare interrupt */
#define AT32_IRQ_TIM2           (AT32_IRQ_FIRST+28)  /* 28: TIM2 global interrupt */
#define AT32_IRQ_TIM3           (AT32_IRQ_FIRST+29)  /* 29: TIM3 global interrupt */
#define AT32_IRQ_TIM4           (AT32_IRQ_FIRST+30)  /* 30: TIM4 global interrupt */
#define AT32_IRQ_I2C1EV         (AT32_IRQ_FIRST+31)  /* 31: I2C1 event interrupt */
#define AT32_IRQ_I2C1ER         (AT32_IRQ_FIRST+32)  /* 32: I2C1 error interrupt */
#define AT32_IRQ_I2C2EV         (AT32_IRQ_FIRST+33)  /* 33: I2C2 event interrupt */
#define AT32_IRQ_I2C2ER         (AT32_IRQ_FIRST+34)  /* 34: I2C2 error interrupt */
#define AT32_IRQ_SPI1           (AT32_IRQ_FIRST+35)  /* 35: SPI1 global interrupt */
#define AT32_IRQ_SPI2           (AT32_IRQ_FIRST+36)  /* 36: SPI2 global interrupt */
#define AT32_IRQ_USART1         (AT32_IRQ_FIRST+37)  /* 37: USART1 global interrupt */
#define AT32_IRQ_USART2         (AT32_IRQ_FIRST+38)  /* 38: USART2 global interrupt */
#define AT32_IRQ_USART3         (AT32_IRQ_FIRST+39)  /* 39: USART3 global interrupt */
#define AT32_IRQ_EXTI1510       (AT32_IRQ_FIRST+40)  /* 40: EXTI Line[15:10] interrupts */
#define AT32_IRQ_RTCALRM        (AT32_IRQ_FIRST+41)  /* 41: RTC alarm through EXTI line interrupt */
#define AT32_IRQ_OTGFSWKUP      (AT32_IRQ_FIRST+42)  /* 42: USB On-The-Go FS1 Wakeup through EXTI line interrupt */
#define AT32_IRQ_TIM8BRK        (AT32_IRQ_FIRST+43)  /* 43: TIM8 Break interrupt */
#define AT32_IRQ_TIM12          (AT32_IRQ_FIRST+43)  /* 43: TIM12 global interrupt */
#define AT32_IRQ_TIM8UP         (AT32_IRQ_FIRST+44)  /* 44: TIM8 Update interrupt */
#define AT32_IRQ_TIM13          (AT32_IRQ_FIRST+44)  /* 44: TIM13 global interrupt */
#define AT32_IRQ_TIM8TRGCOM     (AT32_IRQ_FIRST+45)  /* 45: TIM8 Trigger and Commutation interrupts */
#define AT32_IRQ_TIM14          (AT32_IRQ_FIRST+45)  /* 45: TIM14 global interrupt */
#define AT32_IRQ_TIM8CC         (AT32_IRQ_FIRST+46)  /* 46: TIM8 Capture Compare interrupt */
#define AT32_IRQ_EDMAS8         (AT32_IRQ_FIRST+47)  /* 47: EDMA Stream 8 global interrupt */
#define AT32_IRQ_XSMC           (AT32_IRQ_FIRST+48)  /* 48: XSMC global interrupt */
#define AT32_IRQ_SDIO           (AT32_IRQ_FIRST+49)  /* 49: SDIO global interrupt */
#define AT32_IRQ_TIM5           (AT32_IRQ_FIRST+50)  /* 50: TIM5 global interrupt */
#define AT32_IRQ_SPI3           (AT32_IRQ_FIRST+51)  /* 51: SPI3 global interrupt */
#define AT32_IRQ_UART4          (AT32_IRQ_FIRST+52)  /* 52: UART4 global interrupt */
#define AT32_IRQ_UART5          (AT32_IRQ_FIRST+53)  /* 53: UART5 global interrupt */
#define AT32_IRQ_TIM6           (AT32_IRQ_FIRST+54)  /* 54: TIM6 global interrupt */
#define AT32_IRQ_DAC            (AT32_IRQ_FIRST+54)  /* 54: DAC1 and DAC2 underrun error interrupts */
#define AT32_IRQ_TIM7           (AT32_IRQ_FIRST+55)  /* 55: TIM7 global interrupt */
#define AT32_IRQ_DMA1CH1        (AT32_IRQ_FIRST+56)  /* 56: DMA1 Stream 1 global interrupt */
#define AT32_IRQ_DMA1CH2        (AT32_IRQ_FIRST+57)  /* 57: DMA1 Stream 2 global interrupt */
#define AT32_IRQ_DMA1CH3        (AT32_IRQ_FIRST+58)  /* 58: DMA1 Stream 3 global interrupt */
#define AT32_IRQ_DMA1CH4        (AT32_IRQ_FIRST+59)  /* 59: DMA1 Stream 4 global interrupt */
#define AT32_IRQ_DMA1CH5        (AT32_IRQ_FIRST+60)  /* 60: DMA1 Stream 5 global interrupt */
#if defined(CONFIG_AT32_AT32F437)
#  define AT32_IRQ_ETH          (AT32_IRQ_FIRST+61)  /* 61: Ethernet global interrupt */
#  define AT32_IRQ_ETHWKUP      (AT32_IRQ_FIRST+62)  /* 62: Ethernet Wakeup through EXTI line interrupt */
#endif
#define AT32_IRQ_CAN2TX         (AT32_IRQ_FIRST+63)  /* 63: CAN2 TX interrupts */
#define AT32_IRQ_CAN2RX0        (AT32_IRQ_FIRST+64)  /* 64: CAN2 RX0 interrupts */
#define AT32_IRQ_CAN2RX1        (AT32_IRQ_FIRST+65)  /* 65: CAN2 RX1 interrupt */
#define AT32_IRQ_CAN2SCE        (AT32_IRQ_FIRST+66)  /* 66: CAN2 SCE interrupt */
#define AT32_IRQ_OTGFS          (AT32_IRQ_FIRST+67)  /* 67: USB On The Go FS global interrupt */
#define AT32_IRQ_DMA1CH6        (AT32_IRQ_FIRST+68)  /* 68: DMA1 Stream 6 global interrupt */
#define AT32_IRQ_DMA1CH7        (AT32_IRQ_FIRST+69)  /* 69: DMA1 Stream 7 global interrupt */
#define AT32_IRQ_USART6         (AT32_IRQ_FIRST+71)  /* 71: USART6 global interrupt */
#define AT32_IRQ_I2C3EV         (AT32_IRQ_FIRST+72)  /* 72: I2C3 event interrupt */
#define AT32_IRQ_I2C3ER         (AT32_IRQ_FIRST+73)  /* 73: I2C3 error interrupt */
#define AT32_IRQ_OTGFS2         (AT32_IRQ_FIRST+77)  /* 77: USB On The Go FS2 global interrupt */
#define AT32_IRQ_DVP            (AT32_IRQ_FIRST+78)  /* 78: DVP global interrupt */
#define AT32_IRQ_FPU            (AT32_IRQ_FIRST+81)  /* 81: FPU global interrupt */
#define AT32_IRQ_UART7          (AT32_IRQ_FIRST+82)  /* 82: UART7 interrupt */
#define AT32_IRQ_UART8          (AT32_IRQ_FIRST+83)  /* 83: UART8 interrupt */
#define AT32_IRQ_SPI4           (AT32_IRQ_FIRST+84)  /* 84: SPI4 interrupt */
#define AT32_IRQ_QUADSPI2       (AT32_IRQ_FIRST+91)  /* 91: SAI2 Global interrupt */
#define AT32_IRQ_QUADSPI1       (AT32_IRQ_FIRST+92)  /* 92: QuadSPI Global interrupt */
#define AT32_IRQ_DMAMUX         (AT32_IRQ_FIRST+94)  /* DMAMUX over interrupt */
#define AT32_IRQ_SDIO2          (AT32_IRQ_FIRST+102) /* SDIO2 interrupt */
#define AT32_IRQ_ACC            (AT32_IRQ_FIRST+103) /* ACC interrupt */
#define AT32_IRQ_TIM20BRK       (AT32_IRQ_FIRST+104) /* TIM20 break up interrupt */
#define AT32_IRQ_TIM20UP        (AT32_IRQ_FIRST+105) /* TIM20 over interrupt */
#define AT32_IRQ_TIM20          (AT32_IRQ_FIRST+106) /* TIM20 global interrupt */
#define AT32_IRQ_TIM20CC        (AT32_IRQ_FIRST+107) /* TIM20 Capture Compare interrupt */
#define AT32_IRQ_DMA2CH1        (AT32_IRQ_FIRST+108) /* DMA2 Stream 1 global interrupt */
#define AT32_IRQ_DMA2CH2        (AT32_IRQ_FIRST+109) /* DMA2 Stream 2 global interrupt */
#define AT32_IRQ_DMA2CH3        (AT32_IRQ_FIRST+110) /* DMA2 Stream 3 global interrupt */
#define AT32_IRQ_DMA2CH4        (AT32_IRQ_FIRST+111) /* DMA2 Stream 4 global interrupt */
#define AT32_IRQ_DMA2CH5        (AT32_IRQ_FIRST+112) /* DMA2 Stream 5 global interrupt */
#define AT32_IRQ_DMA2CH6        (AT32_IRQ_FIRST+113) /* DMA2 Stream 6 global interrupt */
#define AT32_IRQ_DMA2CH7        (AT32_IRQ_FIRST+114) /* DMA2 Stream 7 global interrupt */

#if defined(CONFIG_AT32_AT32F435) || defined(CONFIG_AT32_AT32F437)
#  define AT32_IRQ_NEXTINT      (115)
#  define NR_IRQS               (AT32_IRQ_FIRST+115)
#endif

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

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_AT32_AT32F43XXX_IRQ_H */