/************************************************************************************
 * arch/arm/include/stm32s/irq.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

/* This file should never be included directed but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_STM32_IRQ_H
#define __ARCH_ARM_INCLUDE_STM32_IRQ_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <nuttx/irq.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* IRQ numbers.  The IRQ number corresponds vector number and hence map directly to
 * bits in the NVIC.  This does, however, waste several words of memory in the IRQ
 * to handle mapping tables.
 */

/* Processor Exceptions (vectors 0-15) */

#define STM32_IRQ_RESERVED       (0) /* Reserved vector (only used with CONFIG_DEBUG) */
                                     /* Vector  0: Reset stack pointer value */
                                     /* Vector  1: Reset (not handler as an IRQ) */
#define STM32_IRQ_NMI            (2) /* Vector  2: Non-Maskable Interrupt (NMI) */
#define STM32_IRQ_HARDFAULT      (3) /* Vector  3: Hard fault */
#define STM32_IRQ_MPU            (4) /* Vector  4: Memory management (MPU) */
#define STM32_IRQ_BUSFAULT       (5) /* Vector  5: Bus fault */
#define STM32_IRQ_USAGEFAULT     (6) /* Vector  6: Usage fault */
#define STM32_IRQ_SVCALL        (11) /* Vector 11: SVC call */
#define STM32_IRQ_DBGMONITOR    (12) /* Vector 12: Debug Monitor */
                                     /* Vector 13: Reserved */
#define STM32_IRQ_PENDSV        (14) /* Vector 14: Pendable system service request */
#define STM32_IRQ_SYSTICK       (15) /* Vector 15: System tick */

/* External interrupts (vectors >= 16) */

#define STM32_IRQ_INTERRUPTS    (16) /* Vector number of the first external interrupt */
#ifdef CONFIG_STM32_CONNECTIVITY_LINE
#  define STM32_IRQ_WWDG        (16) /* 0:  Window Watchdog interrupt */
#  define STM32_IRQ_PVD         (17) /* 1:  PVD through EXTI Line detection interrupt */
#  define STM32_IRQ_TAMPER      (18) /* 2:  Tamper interrupt */
#  define STM32_IRQ_RTC         (19) /* 3:  RTC global interrupt */
#  define STM32_IRQ_FLASH       (20) /* 4:  Flash global interrupt */
#  define STM32_IRQ_RCC         (21) /* 5:  RCC global interrupt */
#  define STM32_IRQ_EXTI0       (22) /* 6:  EXTI Line 0 interrupt */
#  define STM32_IRQ_EXTI1       (23) /* 7:  EXTI Line 1 interrupt */
#  define STM32_IRQ_EXTI2       (24) /* 8:  EXTI Line 2 interrupt */
#  define STM32_IRQ_EXTI3       (25) /* 9:  EXTI Line 3 interrupt */
#  define STM32_IRQ_EXTI4       (26) /* 10: EXTI Line 4 interrupt */
#  define STM32_IRQ_DMA1CH1     (27) /* 11: DMA1 Channel 1 global interrupt */
#  define STM32_IRQ_DMA1CH2     (28) /* 12: DMA1 Channel 2 global interrupt */
#  define STM32_IRQ_DMA1CH3     (29) /* 13: DMA1 Channel 3 global interrupt */
#  define STM32_IRQ_DMA1CH4     (30) /* 14: DMA1 Channel 4 global interrupt */
#  define STM32_IRQ_DMA1CH5     (31) /* 15: DMA1 Channel 5 global interrupt */
#  define STM32_IRQ_DMA1CH6     (32) /* 16: DMA1 Channel 7 global interrupt */
#  define STM32_IRQ_ADC12       (34) /* 18: ADC1 and ADC2 global interrupt */
#  define STM32_IRQ_CAN1TX      (35) /* 19: CAN1 TX interrupts */
#  define STM32_IRQ_CAN1RX0     (36) /* 20: CAN1 RX0 interrupts */
#  define STM32_IRQ_CAN1RX1     (37) /* 21: CAN1 RX1 interrupt */
#  define STM32_IRQ_CAN1SCE     (38) /* 22: CAN1 SCE interrupt */
#  define STM32_IRQ_EXTI95      (39) /* 23: EXTI Line[9:5] interrupts */
#  define STM32_IRQ_TIM1BRK     (40) /* 24: TIM1 Break interrupt */
#  define STM32_IRQ_TIM1UP      (41) /* 25: TIM1 Update interrupt */
#  define STM32_IRQ_TIM1TRGCOM  (42) /* 26: TIM1 Trigger and Commutation interrupts */
#  define STM32_IRQ_TIM1CC      (43) /* 27: TIM1 Capture Compare interrupt */
#  define STM32_IRQ_TIM2        (44) /* 28: TIM2 global interrupt */
#  define STM32_IRQ_TIM3        (45) /* 29: TIM3 global interrupt */
#  define STM32_IRQ_TIM4        (46) /* 30: TIM4 global interrupt */
#  define STM32_IRQ_I2C1EV      (47) /* 31: I2C1 event interrupt */
#  define STM32_IRQ_I2C1ER      (48) /* 32: I2C1 error interrupt */
#  define STM32_IRQ_I2C2EV      (49) /* 33: I2C2 event interrupt */
#  define STM32_IRQ_I2C2ER      (50) /* 34: I2C2 error interrupt */
#  define STM32_IRQ_SPI1        (51) /* 35: SPI1 global interrupt */
#  define STM32_IRQ_SPI2        (52) /* 36: SPI2 global interrupt */
#  define STM32_IRQ_USART1      (53) /* 37: USART1 global interrupt */
#  define STM32_IRQ_USART2      (54) /* 38: USART2 global interrupt */
#  define STM32_IRQ_USART3      (55) /* 39: USART3 global interrupt */
#  define STM32_IRQ_EXTI1510    (56) /* 40: EXTI Line[15:10] interrupts */
#  define STM32_IRQ_RTCAlR      (57) /* 41: RTC alarm through EXTI line interrupt */
#  define STM32_IRQ_OTGFSWKUP   (58) /* 42: USB On-The-Go FS Wakeup through EXTI line interrupt */
                                     /* 43-49: Reserved */
#  define STM32_IRQ_TIM5        (59) /* 50: TIM5 global interrupt */
#  define STM32_IRQ_SPI3        (60) /* 51: SPI3 global interrupt */
#  define STM32_IRQ_UART4       (71) /* 52: UART4 global interrupt */
#  define STM32_IRQ_UART5       (72) /* 53: UART5 global interrupt */
#  define STM32_IRQ_TIM6        (73) /* 54: TIM6 global interrupt */
#  define STM32_IRQ_TIM7        (74) /* 55: TIM7 global interrupt */
#  define STM32_IRQ_DMA2CH1     (75) /* 56: DMA2 Channel 1 global interrupt */
#  define STM32_IRQ_DMA2CH2     (76) /* 57: DMA2 Channel 2 global interrupt */
#  define STM32_IRQ_DMA2CH3     (77) /* 58: DMA2 Channel 3 global interrupt */
#  define STM32_IRQ_DMA2CH4     (78) /* 59: DMA2 Channel 4 global interrupt */
#  define STM32_IRQ_DMA2CH5     (79) /* 60: DMA2 Channel 5 global interrupt */
#  define STM32_IRQ_ETH E       (80) /* 61: thernet global interrupt */
#  define STM32_IRQ_ETHWKUP     (81) /* 62: Ethernet Wakeup through EXTI line interrupt */
#  define STM32_IRQ_CAN2TX      (82) /* 63: CAN2 TX interrupts */
#  define STM32_IRQ_CAN2RX0     (83) /* 64: CAN2 RX0 interrupts */
#  define STM32_IRQ_CAN2RX1     (84) /* 65: CAN2 RX1 interrupt */
#  define STM32_IRQ_CAN2SCE     (85) /* 66: CAN2 SCE interrupt */
#  define STM32_IRQ_OTGFS       (86) /* 67: USB On The Go FS global interrupt */
#  define NR_IRQS               (87)
#else
#  define STM32_IRQ_WWDG        (16) /* 0:  Window Watchdog interrupt */
#  define STM32_IRQ_PVD         (17) /* 1:  PVD through EXTI Line detection interrupt */
#  define STM32_IRQ_TAMPER      (18) /* 2:  Tamper interrupt */
#  define STM32_IRQ_RTC         (19) /* 3:  RTC global interrupt */
#  define STM32_IRQ_FLASH       (20) /* 4:  Flash global interrupt */
#  define STM32_IRQ_RCC         (21) /* 5:  RCC global interrupt */
#  define STM32_IRQ_EXTI0       (22) /* 6:  EXTI Line 0 interrupt */
#  define STM32_IRQ_EXTI1       (23) /* 7:  EXTI Line 1 interrupt */
#  define STM32_IRQ_EXTI2       (24) /* 8:  EXTI Line 2 interrupt */
#  define STM32_IRQ_EXTI3       (25) /* 9:  EXTI Line 3 interrupt */
#  define STM32_IRQ_EXTI4       (26) /* 10: EXTI Line 4 interrupt */
#  define STM32_IRQ_DMA1CH1     (27) /* 11: DMA1 Channel 1 global interrupt */
#  define STM32_IRQ_DMA1CH2     (28) /* 12: DMA1 Channel 2 global interrupt */
#  define STM32_IRQ_DMA1CH3     (29) /* 13: DMA1 Channel 3 global interrupt */
#  define STM32_IRQ_DMA1CH4     (30) /* 14: DMA1 Channel 4 global interrupt */
#  define STM32_IRQ_DMA1CH5     (31) /* 15: DMA1 Channel 5 global interrupt */
#  define STM32_IRQ_DMA1CH6     (32) /* 16: DMA1 Channel 6 global interrupt */
#  define STM32_IRQ_DMA1CH7     (33) /* 17: DMA1 Channel 7 global interrupt */
#  define STM32_IRQ_ADC12       (34) /* 18: ADC1 and ADC2 global interrupt */
#  define STM32_IRQ_USBHPCANTX  (35) /* 19: USB High Priority or CAN TX interrupts*/
#  define STM32_IRQ_USBLPCANRX0 (36) /* 20: USB Low Priority or CAN RX0 interrupts*/
#  define STM32_IRQ_CAN1RX1     (37) /* 21: CAN1 RX1 interrupt */
#  define STM32_IRQ_CAN1SCE     (38) /* 22: CAN1 SCE interrupt */
#  define STM32_IRQ_EXTI95      (39) /* 23: EXTI Line[9:5] interrupts */
#  define STM32_IRQ_TIM1BRK     (40) /* 24: TIM1 Break interrupt */
#  define STM32_IRQ_TIM1UP      (41) /* 25: TIM1 Update interrupt */
#  define STM32_IRQ_TIM1TRGCOM  (42) /* 26: TIM1 Trigger and Commutation interrupts */
#  define STM32_IRQ_TIM1CC      (43) /* 27: TIM1 Capture Compare interrupt */
#  define STM32_IRQ_TIM2        (44) /* 28: TIM2 global interrupt */
#  define STM32_IRQ_TIM3        (45) /* 29: TIM3 global interrupt */
#  define STM32_IRQ_TIM4        (46) /* 30: TIM4 global interrupt */
#  define STM32_IRQ_I2C1EV      (47) /* 31: I2C1 event interrupt */
#  define STM32_IRQ_I2C1ER      (48) /* 32: I2C1 error interrupt */
#  define STM32_IRQ_I2C2EV      (49) /* 33: I2C2 event interrupt */
#  define STM32_IRQ_I2C2ER      (50) /* 34: I2C2 error interrupt */
#  define STM32_IRQ_SPI1        (51) /* 35: SPI1 global interrupt */
#  define STM32_IRQ_SPI2        (52) /* 36: SPI2 global interrupt */
#  define STM32_IRQ_USART1      (53) /* 37: USART1 global interrupt */
#  define STM32_IRQ_USART2      (54) /* 38: USART2 global interrupt */
#  define STM32_IRQ_USART3      (55) /* 39: USART3 global interrupt */
#  define STM32_IRQ_EXTI1510    (56) /* 40: EXTI Line[15:10] interrupts */
#  define STM32_IRQ_RTCAlR      (57) /* 41: RTC alarm through EXTI line interrupt */
#  define STM32_IRQ_USBWKUP     (58) /* 42: USB wakeup from suspend through EXTI line interrupt*/
#  define STM32_IRQ_TIM8BRK     (59) /* 43: TIM8 Break interrupt */
#  define STM32_IRQ_TIM8UP      (60) /* 44: TIM8 Update interrupt */
#  define STM32_IRQ_TIM8TRGCOM  (61) /* 45: TIM8 Trigger and Commutation interrupts */
#  define STM32_IRQ_TIM8CC      (62) /* 46: TIM8 Capture Compare interrupt */
#  define STM32_IRQ_ADC3        (63) /* 47: ADC3 global interrupt */
#  define STM32_IRQ_FSMC        (64) /* 48: FSMC global interrupt */
#  define STM32_IRQ_SDIO        (65) /* 49: SDIO global interrupt */
#  define STM32_IRQ_TIM5        (66) /* 50: TIM5 global interrupt */
#  define STM32_IRQ_SPI3        (67) /* 51: SPI3 global interrupt */
#  define STM32_IRQ_UART4       (68) /* 52: UART4 global interrupt */
#  define STM32_IRQ_UART5       (69) /* 53: UART5 global interrupt */
#  define STM32_IRQ_TIM6        (70) /* 54: TIM6 global interrupt */
#  define STM32_IRQ_TIM7        (71) /* 55: TIM7 global interrupt */
#  define STM32_IRQ_DMA2CH1     (72) /* 56: DMA2 Channel 1 global interrupt */
#  define STM32_IRQ_DMA2CH2     (73) /* 57: DMA2 Channel 2 global interrupt */
#  define STM32_IRQ_DMA2CH3     (74) /* 58: DMA2 Channel 3 global interrupt */
#  define STM32_IRQ_DMA2CH45    (75) /* 59: DMA2 Channel 4&5 global interrupt */
#  define NR_IRQS               (76)
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_STM32_IRQ_H */

