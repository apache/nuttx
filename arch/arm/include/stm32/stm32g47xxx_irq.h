/****************************************************************************************************
 *  arch/arm/include/stm32/stm32g47xxx_irq.h
 *
 *  Licensed to the Apache Software Foundation (ASF) under one or more
 *  contributor license agreements.  See the NOTICE file distributed with
 *  this work for additional information regarding copyright ownership.  The
 *  ASF licenses this file to you under the Apache License, Version 2.0 (the
 *  "License"); you may not use this file except in compliance with the
 *  License.  You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 *  License for the specific language governing permissions and limitations
 *  under the License.
 *
 ****************************************************************************************************/

/* This file should never be included directly but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_STM32_STM32G47XXX_IRQ_H
#define __ARCH_ARM_INCLUDE_STM32_STM32G47XXX_IRQ_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* IRQ numbers.  The IRQ numbers correspond to the vector numbers and hence
 * map directly to bits in the NVIC.  This does, however, waste several words
 * of memory in the IRQ to handle mapping tables.
 *
 * Processor Exceptions (vectors 0-15) are common to all STM32 parts and are
 * found in nuttx/arch/arm/include/stm32/irq.h.  They are not repeated here.
 *
 * Other interrupts (vectors >= 16) are defined below.
 */

#define STM32_IRQ_WWDG            (STM32_IRQ_FIRST + 0)   /* 0:  Window Watchdog interrupt */
#define STM32_IRQ_PVD             (STM32_IRQ_FIRST + 1)   /* 1:  PVD through EXTI Line detection interrupt */
#define STM32_IRQ_TAMPER          (STM32_IRQ_FIRST + 2)   /* 2:  Tamper interrupt, or Time Stamp (shared with STM32_IRQ_TIMESTAMP) */
#define STM32_IRQ_TIMESTAMP       (STM32_IRQ_FIRST + 2)   /* 2:  Time stamp interrupt (shared with STM32_IRQ_TAMPER) */
#define STM32_IRQ_RTC_WKUP        (STM32_IRQ_FIRST + 3)   /* 3:  RTC global interrupt */
#define STM32_IRQ_FLASH           (STM32_IRQ_FIRST + 4)   /* 4:  Flash global interrupt */
#define STM32_IRQ_RCC             (STM32_IRQ_FIRST + 5)   /* 5:  RCC global interrupt */
#define STM32_IRQ_EXTI0           (STM32_IRQ_FIRST + 6)   /* 6:  EXTI Line 0 interrupt */
#define STM32_IRQ_EXTI1           (STM32_IRQ_FIRST + 7)   /* 7:  EXTI Line 1 interrupt */
#define STM32_IRQ_EXTI2           (STM32_IRQ_FIRST + 8)   /* 8:  EXTI Line 2 interrupt, or */
#define STM32_IRQ_EXTI3           (STM32_IRQ_FIRST + 9)   /* 9:  EXTI Line 3 interrupt */

#define STM32_IRQ_EXTI4           (STM32_IRQ_FIRST + 10)  /* 10: EXTI Line 4 interrupt */
#define STM32_IRQ_DMA1CH1         (STM32_IRQ_FIRST + 11)  /* 11: DMA1 channel 1 global interrupt */
#define STM32_IRQ_DMA1CH2         (STM32_IRQ_FIRST + 12)  /* 12: DMA1 channel 2 global interrupt */
#define STM32_IRQ_DMA1CH3         (STM32_IRQ_FIRST + 13)  /* 13: DMA1 channel 3 global interrupt */
#define STM32_IRQ_DMA1CH4         (STM32_IRQ_FIRST + 14)  /* 14: DMA1 channel 4 global interrupt */
#define STM32_IRQ_DMA1CH5         (STM32_IRQ_FIRST + 15)  /* 15: DMA1 channel 5 global interrupt */
#define STM32_IRQ_DMA1CH6         (STM32_IRQ_FIRST + 16)  /* 16: DMA1 channel 6 global interrupt */
#define STM32_IRQ_DMA1CH7         (STM32_IRQ_FIRST + 17)  /* 17: DMA1 channel 7 global interrupt */
#define STM32_IRQ_ADC12           (STM32_IRQ_FIRST + 18)  /* 18: ADC1 and ADC2 shared global interrupt */
#define STM32_IRQ_USBHP           (STM32_IRQ_FIRST + 19)  /* 19: USB High priority interrupt */

#define STM32_IRQ_USBLP           (STM32_IRQ_FIRST + 20)  /* 20: USB Low priority interrupt */
#define STM32_IRQ_FDCAN1_0        (STM32_IRQ_FIRST + 21)  /* 21: FDCAN1 interrupt 0 */
#define STM32_IRQ_FDCAN1_1        (STM32_IRQ_FIRST + 22)  /* 22: FDCAN1 interrupt 1 */
#define STM32_IRQ_EXTI95          (STM32_IRQ_FIRST + 23)  /* 23: EXTI Line[9:5] interrupts */
#define STM32_IRQ_TIM15           (STM32_IRQ_FIRST + 24)  /* 24: TIM15 global interrupt (shared with STM32_IRQ_TIM1BRK) */
#define STM32_IRQ_TIM1BRK         (STM32_IRQ_FIRST + 24)  /* 24: TIM1 Break, Transition error, Index error (shared with STM32_IRQ_TIM15) */
#define STM32_IRQ_TIM16           (STM32_IRQ_FIRST + 25)  /* 25: TIM16 global interrupt (shared with STM32_IRQ_TIM1UP) */
#define STM32_IRQ_TIM1UP          (STM32_IRQ_FIRST + 25)  /* 25: TIM1 Update interrupt (shared with STM32_IRQ_TIM16) */
#define STM32_IRQ_TIM17           (STM32_IRQ_FIRST + 26)  /* 26: TIM17 global interrupt (shared with STM32_IRQ_TIM1TRGCOM) */
#define STM32_IRQ_TIM1TRGCOM      (STM32_IRQ_FIRST + 26)  /* 26: TIM1 Trigger, Commutation, Direction Change, and Index interrupt (shared with STM32_IRQ_TIM17) */
#define STM32_IRQ_TIM1CC          (STM32_IRQ_FIRST + 27)  /* 27: TIM1 Capture Compare interrupt */
#define STM32_IRQ_TIM2            (STM32_IRQ_FIRST + 28)  /* 28: TIM2 global interrupt */
#define STM32_IRQ_TIM3            (STM32_IRQ_FIRST + 29)  /* 29: TIM3 global interrupt */

#define STM32_IRQ_TIM4            (STM32_IRQ_FIRST + 30)  /* 30: TIM4 global interrupt */
#define STM32_IRQ_I2C1EV          (STM32_IRQ_FIRST + 31)  /* 31: I2C1 event interrupt */
#define STM32_IRQ_I2C1ER          (STM32_IRQ_FIRST + 32)  /* 32: I2C1 error interrupt */
#define STM32_IRQ_I2C2EV          (STM32_IRQ_FIRST + 33)  /* 33: I2C2 event interrupt */
#define STM32_IRQ_I2C2ER          (STM32_IRQ_FIRST + 34)  /* 34: I2C2 error interrupt */
#define STM32_IRQ_SPI1            (STM32_IRQ_FIRST + 35)  /* 35: SPI1 global interrupt */
#define STM32_IRQ_SPI2            (STM32_IRQ_FIRST + 36)  /* 36: SPI2 global interrupt */
#define STM32_IRQ_USART1          (STM32_IRQ_FIRST + 37)  /* 37: USART1 global interrupt */
#define STM32_IRQ_USART2          (STM32_IRQ_FIRST + 38)  /* 38: USART2 global interrupt */
#define STM32_IRQ_USART3          (STM32_IRQ_FIRST + 39)  /* 39: USART3 global interrupt */

#define STM32_IRQ_EXTI1510        (STM32_IRQ_FIRST + 40)  /* 40: EXTI Line[15:10] interrupts */
#define STM32_IRQ_RTCALRM         (STM32_IRQ_FIRST + 41)  /* 41: RTC alarm through EXTI line interrupt */
#define STM32_IRQ_USBWKUP         (STM32_IRQ_FIRST + 42)  /* 42: 42: USB wakeup from suspend through EXTI line interrupt */
#define STM32_IRQ_TIM8BRK         (STM32_IRQ_FIRST + 43)  /* 43: TIM8 Break, Transition error, Index error */
#define STM32_IRQ_TIM8UP          (STM32_IRQ_FIRST + 44)  /* 44: TIM8 Update interrupt */
#define STM32_IRQ_TIM8TRGCOM      (STM32_IRQ_FIRST + 45)  /* 45: TIM8 Trigger, Commutation, Direction Change, and Index interrupt */
#define STM32_IRQ_TIM8CC          (STM32_IRQ_FIRST + 46)  /* 46: TIM8 Capture Compare interrupt */
#define STM32_IRQ_ADC3            (STM32_IRQ_FIRST + 47)  /* 47: ADC3 global interrupt */
#define STM32_IRQ_FMC             (STM32_IRQ_FIRST + 48)  /* 48: FMC global interrupt */
#define STM32_IRQ_LPTIM1          (STM32_IRQ_FIRST + 49)  /* 49: LPTIM1 interrupt */

#define STM32_IRQ_TIM5            (STM32_IRQ_FIRST + 50)  /* 50: TIM5 global interrupt */
#define STM32_IRQ_SPI3            (STM32_IRQ_FIRST + 51)  /* 51: SPI3 global interrupt */
#define STM32_IRQ_UART4           (STM32_IRQ_FIRST + 52)  /* 52: UART4 global interrupt */
#define STM32_IRQ_UART5           (STM32_IRQ_FIRST + 53)  /* 53: UART5 global interrupt */
#define STM32_IRQ_TIM6            (STM32_IRQ_FIRST + 54)  /* 54: TIM6 global interrupt (shared with STM32_IRQ_DAC1, STM32_IRQ_DAC3) */
#define STM32_IRQ_DAC1            (STM32_IRQ_FIRST + 54)  /* 54: DAC1 underrun error interrupt (shared with STM32_IRQ_TIM6, STM32_IRQ_DAC3) */
#define STM32_IRQ_DAC3            (STM32_IRQ_FIRST + 54)  /* 54: DAC3 underrun error interrupt (shared with STM32_IRQ_TIM6, STM32_IRQ_DAC1) */
#define STM32_IRQ_TIM7            (STM32_IRQ_FIRST + 55)  /* 55: TIM7 global interrupt (shared with STM32_IRQ_DAC2, STM32_IRQ_DAC4) */
#define STM32_IRQ_DAC2            (STM32_IRQ_FIRST + 55)  /* 55: DAC2 underrun error interrupt (shared with STM32_IRQ_TIM7) */
#define STM32_IRQ_DAC4            (STM32_IRQ_FIRST + 55)  /* 55: DAC4 underrun error interrupt (shared with STM32_IRQ_TIM7) */
#define STM32_IRQ_DMA2CH1         (STM32_IRQ_FIRST + 56)  /* 56: DMA2 channel 1 global interrupt */
#define STM32_IRQ_DMA2CH2         (STM32_IRQ_FIRST + 57)  /* 57: DMA2 channel 2 global interrupt */
#define STM32_IRQ_DMA2CH3         (STM32_IRQ_FIRST + 58)  /* 58: DMA2 channel 3 global interrupt */
#define STM32_IRQ_DMA2CH4         (STM32_IRQ_FIRST + 59)  /* 59: DMA2 channel 4 global interrupt */

#define STM32_IRQ_DMA2CH5         (STM32_IRQ_FIRST + 60)  /* 60: DMA2 channel 5 global interrupt */
#define STM32_IRQ_ADC4            (STM32_IRQ_FIRST + 61)  /* 61: ADC4 global interrupt */
#define STM32_IRQ_ADC5            (STM32_IRQ_FIRST + 62)  /* 62: ADC5 global interrupt */
#define STM32_IRQ_UCPD            (STM32_IRQ_FIRST + 63)  /* 63: UCPD global interrupt */
#define STM32_IRQ_COMP123         (STM32_IRQ_FIRST + 64)  /* 64: COMP1, COMP2, and COMP3 interrupts */
#define STM32_IRQ_COMP456         (STM32_IRQ_FIRST + 65)  /* 65: COMP4, COMP5, and COMP6 interrupts */
#define STM32_IRQ_COMP7           (STM32_IRQ_FIRST + 66)  /* 66: COMPP7 interrupt */
#define STM32_IRQ_HRTIM1MST       (STM32_IRQ_FIRST + 67)  /* 67: HRTIM1 master timer interrupt */
#define STM32_IRQ_HRTIM1TIMA      (STM32_IRQ_FIRST + 68)  /* 68: HRTIM1 timer A interrupt */
#define STM32_IRQ_HRTIM1TIMB      (STM32_IRQ_FIRST + 69)  /* 69: HRTIM1 timer B interrupt */

#define STM32_IRQ_HRTIM1TIMC      (STM32_IRQ_FIRST + 70)  /* 70: HRTIM1 timer C interrupt */
#define STM32_IRQ_HRTIM1TIMD      (STM32_IRQ_FIRST + 71)  /* 71: HRTIM1 timer D interrupt */
#define STM32_IRQ_HRTIM1TIME      (STM32_IRQ_FIRST + 72)  /* 72: HRTIM1 timer E interrupt */
#define STM32_IRQ_HRTIM1FLT       (STM32_IRQ_FIRST + 73)  /* 73: HRTIM1 fault interrupt */
#define STM32_IRQ_HRTIM1TIMF      (STM32_IRQ_FIRST + 74)  /* 74: HRTIM1 timer E interrupt */
#define STM32_IRQ_CRS             (STM32_IRQ_FIRST + 75)  /* 75: CRS (Clock Recovery System) global interrupt */
#define STM32_IRQ_SAI1            (STM32_IRQ_FIRST + 76)  /* 76: SAI4 global interrupt */
#define STM32_IRQ_TIM20BRK        (STM32_IRQ_FIRST + 77)  /* 77: TIM20 Break, Transition error, Index error interrupt */
#define STM32_IRQ_TIM20UP         (STM32_IRQ_FIRST + 78)  /* 78: TIM20 Update interrupt */
#define STM32_IRQ_TIM20TRGCOM     (STM32_IRQ_FIRST + 79)  /* 79: TIM20 Trigger, Commutation, Direction Change, and Index interrupt */

#define STM32_IRQ_TIM20CC         (STM32_IRQ_FIRST + 80)  /* 80: TIM20 Capture Compare interrupt */
#define STM32_IRQ_FPU             (STM32_IRQ_FIRST + 81)  /* 81: FPU global interrupt */
#define STM32_IRQ_I2C4EV          (STM32_IRQ_FIRST + 82)  /* 82: I2C4 event interrupt */
#define STM32_IRQ_I2C4ER          (STM32_IRQ_FIRST + 83)  /* 83: I2C4 error interrupt */
#define STM32_IRQ_SPI4            (STM32_IRQ_FIRST + 84)  /* 84: SPI4 global interrupt */
#define STM32_IRQ_AES             (STM32_IRQ_FIRST + 85)  /* 85: AES global interrupt */
#define STM32_IRQ_FDCAN2_0        (STM32_IRQ_FIRST + 86)  /* 86: FDCAN2 interrupt 0 */
#define STM32_IRQ_FDCAN2_1        (STM32_IRQ_FIRST + 87)  /* 87: FDCAN2 interrupt 1 */
#define STM32_IRQ_FDCAN3_0        (STM32_IRQ_FIRST + 88)  /* 88: FDCAN3 interrupt 0 */
#define STM32_IRQ_FDCAN3_1        (STM32_IRQ_FIRST + 89)  /* 89: FDCAN3 interrupt 1 */

#define STM32_IRQ_RNG             (STM32_IRQ_FIRST + 90)  /* 90: RNG global interrupt */
#define STM32_IRQ_LPUART          (STM32_IRQ_FIRST + 91)  /* 91: LPUART global interrupt */
#define STM32_IRQ_I2C3EV          (STM32_IRQ_FIRST + 92)  /* 92: I2C3 event interrupt */
#define STM32_IRQ_I2C3ER          (STM32_IRQ_FIRST + 93)  /* 93: I2C3 error interrupt */
#define STM32_IRQ_DMAMUXOV        (STM32_IRQ_FIRST + 94)  /* 94: DMAMUX overrun interrupt */
#define STM32_IRQ_QUADSPI         (STM32_IRQ_FIRST + 95)  /* 95: QuadSPI global interrupt */
#define STM32_IRQ_DMA1CH8         (STM32_IRQ_FIRST + 96)  /* 96: DMA1 channel 8 global interrupt */
#define STM32_IRQ_DMA2CH6         (STM32_IRQ_FIRST + 97)  /* 97: DMA2 channel 6 global interrupt */
#define STM32_IRQ_DMA2CH7         (STM32_IRQ_FIRST + 98)  /* 98: DMA2 channel 7 global interrupt */
#define STM32_IRQ_DMA2CH8         (STM32_IRQ_FIRST + 99)  /* 99: DMA2 channel 8 global interrupt */

#define STM32_IRQ_CORDIC          (STM32_IRQ_FIRST + 100) /* 100: CORDIC trigonometric accelerator interrupt */
#define STM32_IRQ_FMAC            (STM32_IRQ_FIRST + 101) /* 101: FMAC filter math acclerator interrupt */

#define STM32_IRQ_NEXTINT         (102)
#define NR_IRQS                   (STM32_IRQ_FIRST + 102)

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
 * Public Function Prototypes
 ****************************************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_STM32F30XXX_IRQ_H */
