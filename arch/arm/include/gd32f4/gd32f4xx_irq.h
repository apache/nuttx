/****************************************************************************
 * arch/arm/include/gd32f4/gd32f4xx_irq.h
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

#ifndef __ARCH_ARM_INCLUDE_GD32_GD32F4XX_IRQ_H
#define __ARCH_ARM_INCLUDE_GD32_GD32F4XX_IRQ_H

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
 * be found in nuttx/arch/arm/include/gd32/irq.h
 *
 * External interrupts (vectors >= 16)
 */

#define GD32_IRQ_WWDGT                      (GD32_IRQ_EXINT+0)  /* 0:  window watchdog timer interrupt */
#define GD32_IRQ_LVD                        (GD32_IRQ_EXINT+1)  /* 1:  LVD through EXTI line detect interrupt */
#define GD32_IRQ_TAMPER_STAMP               (GD32_IRQ_EXINT+2)  /* 2:  tamper and timestamp through EXTI line detect */
#define GD32_IRQ_RTC_WKUP                   (GD32_IRQ_EXINT+3)  /* 3:  RTC wakeup through EXTI line interrupt */
#define GD32_IRQ_FMC                        (GD32_IRQ_EXINT+4)  /* 4:  FMC interrupt */
#define GD32_IRQ_RCU_CTC                    (GD32_IRQ_EXINT+5)  /* 5:  RCU and CTC interrupt */
#define GD32_IRQ_EXTI0                      (GD32_IRQ_EXINT+6)  /* 6:  EXTI Line 0 interrupt */
#define GD32_IRQ_EXTI1                      (GD32_IRQ_EXINT+7)  /* 7:  EXTI Line 1 interrupt */
#define GD32_IRQ_EXTI2                      (GD32_IRQ_EXINT+8)  /* 8:  EXTI Line 2 interrupt */
#define GD32_IRQ_EXTI3                      (GD32_IRQ_EXINT+9)  /* 9:  EXTI Line 3 interrupt */
#define GD32_IRQ_EXTI4                      (GD32_IRQ_EXINT+10) /* 10: EXTI Line 4 interrupt */
#define GD32_IRQ_DMA0_CHANNEL0              (GD32_IRQ_EXINT+11) /* 11: DMA0 channel0 Interrupt */
#define GD32_IRQ_DMA0_CHANNEL1              (GD32_IRQ_EXINT+12) /* 12: DMA0 channel1 Interrupt */
#define GD32_IRQ_DMA0_CHANNEL2              (GD32_IRQ_EXINT+13) /* 13: DMA0 channel2 Interrupt */
#define GD32_IRQ_DMA0_CHANNEL3              (GD32_IRQ_EXINT+14) /* 14: DMA0 channel3 Interrupt */
#define GD32_IRQ_DMA0_CHANNEL4              (GD32_IRQ_EXINT+15) /* 15: DMA0 channel4 Interrupt */
#define GD32_IRQ_DMA0_CHANNEL5              (GD32_IRQ_EXINT+16) /* 16: DMA0 channel5 Interrupt */
#define GD32_IRQ_DMA0_CHANNEL6              (GD32_IRQ_EXINT+17) /* 17: DMA0 channel6 Interrupt */
#define GD32_IRQ_ADC                        (GD32_IRQ_EXINT+18) /* 18: ADC interrupt */
#define GD32_IRQ_CAN0_TX                    (GD32_IRQ_EXINT+19) /* 19: CAN0 TX interrupt */
#define GD32_IRQ_CAN0_RX0                   (GD32_IRQ_EXINT+20) /* 20: CAN0 RX0 interrupt */
#define GD32_IRQ_CAN0_RX1                   (GD32_IRQ_EXINT+21) /* 21: CAN0 RX1 interrupt */
#define GD32_IRQ_CAN0_EWMC                  (GD32_IRQ_EXINT+22) /* 22: CAN0 EWMC interrupt */
#define GD32_IRQ_EXTI5_9                    (GD32_IRQ_EXINT+23) /* 23: EXTI[9:5] interrupts */
#define GD32_IRQ_TIMER0_BRK_TIMER8          (GD32_IRQ_EXINT+24) /* 24: TIMER0 break and TIMER8 interrupts */
#define GD32_IRQ_TIMER0_UP_TIMER9           (GD32_IRQ_EXINT+25) /* 25: TIMER0 update and TIMER9 interrupts */
#define GD32_IRQ_TIMER0_TRG_CMT_TIMER10     (GD32_IRQ_EXINT+26) /* 26: TIMER0 trigger and commutation  and TIMER10 interrupts */
#define GD32_IRQ_TIMER0_CHANNEL             (GD32_IRQ_EXINT+27) /* 27: TIMER0 channel capture compare interrupt */
#define GD32_IRQ_TIMER1                     (GD32_IRQ_EXINT+28) /* 28: TIMER1 interrupt */
#define GD32_IRQ_TIMER2                     (GD32_IRQ_EXINT+29) /* 29: TIMER2 interrupt */
#define GD32_IRQ_TIMER3                     (GD32_IRQ_EXINT+30) /* 30: TIMER3 interrupt */
#define GD32_IRQ_I2C0_EV                    (GD32_IRQ_EXINT+31) /* 31: I2C0 event interrupt */
#define GD32_IRQ_I2C0_ER                    (GD32_IRQ_EXINT+32) /* 32: I2C0 error interrupt */
#define GD32_IRQ_I2C1_EV                    (GD32_IRQ_EXINT+33) /* 33: I2C1 event interrupt */
#define GD32_IRQ_I2C1_ER                    (GD32_IRQ_EXINT+34) /* 34: I2C1 error interrupt */
#define GD32_IRQ_SPI0                       (GD32_IRQ_EXINT+35) /* 35: SPI0 interrupt */
#define GD32_IRQ_SPI1                       (GD32_IRQ_EXINT+36) /* 36: SPI1 interrupt */
#define GD32_IRQ_USART0                     (GD32_IRQ_EXINT+37) /* 37: USART0 interrupt */
#define GD32_IRQ_USART1                     (GD32_IRQ_EXINT+38) /* 38: USART1 interrupt */
#define GD32_IRQ_USART2                     (GD32_IRQ_EXINT+39) /* 39: USART2 interrupt */
#define GD32_IRQ_EXTI10_15                  (GD32_IRQ_EXINT+40) /* 40: EXTI[15:10] interrupts */
#define GD32_IRQ_RTC_Alarm                  (GD32_IRQ_EXINT+41) /* 41: RTC alarm interrupt */
#define GD32_IRQ_USBFS_WKUP                 (GD32_IRQ_EXINT+42) /* 42: USBFS wakeup interrupt */
#define GD32_IRQ_TIMER7_BRK_TIMER11         (GD32_IRQ_EXINT+43) /* 43: TIMER7 break and TIMER11 interrupts */
#define GD32_IRQ_TIMER7_UP_TIMER12          (GD32_IRQ_EXINT+44) /* 44: TIMER7 update and TIMER12 interrupts */
#define GD32_IRQ_TIMER7_TRG_CMT_TIMER13     (GD32_IRQ_EXINT+45) /* 45: TIMER7 trigger and commutation and TIMER13 interrupts */
#define GD32_IRQ_TIMER7_CHANNEL             (GD32_IRQ_EXINT+46) /* 46: TIMER7 channel capture compare interrupt */
#define GD32_IRQ_DMA0_CHANNEL7              (GD32_IRQ_EXINT+47) /* 47: DMA0 channel7 interrupt */

#if defined(CONFIG_GD32F4_GD32F450)
#define GD32_IRQ_EXMC                       (GD32_IRQ_EXINT+48) /* 48: EXMC interrupt */
#define GD32_IRQ_SDIO                       (GD32_IRQ_EXINT+49) /* 49: SDIO interrupt */
#define GD32_IRQ_TIMER4                     (GD32_IRQ_EXINT+50) /* 50: TIMER4 interrupt */
#define GD32_IRQ_SPI2                       (GD32_IRQ_EXINT+51) /* 51: SPI2 interrupt */
#define GD32_IRQ_UART3                      (GD32_IRQ_EXINT+52) /* 52: UART3 interrupt */
#define GD32_IRQ_UART4                      (GD32_IRQ_EXINT+53) /* 53: UART4 interrupt */
#define GD32_IRQ_TIMER5_DAC                 (GD32_IRQ_EXINT+54) /* 54: TIMER5 and DAC0 DAC1 underrun error interrupts */
#define GD32_IRQ_TIMER6                     (GD32_IRQ_EXINT+55) /* 55: TIMER6 interrupt */
#define GD32_IRQ_DMA1_CHANNEL0              (GD32_IRQ_EXINT+56) /* 56: DMA1 channel0 interrupt */
#define GD32_IRQ_DMA1_CHANNEL1              (GD32_IRQ_EXINT+57) /* 57: DMA1 channel1 interrupt */
#define GD32_IRQ_DMA1_CHANNEL2              (GD32_IRQ_EXINT+58) /* 58: DMA1 channel2 interruptt */
#define GD32_IRQ_DMA1_CHANNEL3              (GD32_IRQ_EXINT+59) /* 59: DMA1 channel3 interrupt */
#define GD32_IRQ_DMA1_CHANNEL4              (GD32_IRQ_EXINT+60) /* 60: DMA1 channel4 interrupt */
#define GD32_IRQ_ENET                       (GD32_IRQ_EXINT+61) /* 61: ENET interrupt */
#define GD32_IRQ_ENET_WKUP                  (GD32_IRQ_EXINT+62) /* 62: ENET wakeup interrupt */
#define GD32_IRQ_CAN1_TX                    (GD32_IRQ_EXINT+63) /* 63: CAN1 TX interrupt */
#define GD32_IRQ_CAN1_RX0                   (GD32_IRQ_EXINT+64) /* 64: CAN1 RX0 interrupt */
#define GD32_IRQ_CAN1_RX1                   (GD32_IRQ_EXINT+65) /* 65: CAN1 RX1 interrupt */
#define GD32_IRQ_CAN1_EWMC                  (GD32_IRQ_EXINT+66) /* 66: CAN1 EWMC interrupt */
#define GD32_IRQ_USBFS                      (GD32_IRQ_EXINT+67) /* 67: USBFS interrupt */
#define GD32_IRQ_DMA1_CHANNEL5              (GD32_IRQ_EXINT+68) /* 68: DMA1 channel5 interrupt */
#define GD32_IRQ_DMA1_CHANNEL6              (GD32_IRQ_EXINT+69) /* 69: DMA1 channel6 interrupt */
#define GD32_IRQ_DMA1_CHANNEL7              (GD32_IRQ_EXINT+70) /* 70: DMA1 channel7 interrupt */
#define GD32_IRQ_USART5                     (GD32_IRQ_EXINT+71) /* 71: USART5 interrupt */
#define GD32_IRQ_I2C2_EV                    (GD32_IRQ_EXINT+72) /* 72: I2C2 event interrupt */
#define GD32_IRQ_I2C2_ER                    (GD32_IRQ_EXINT+73) /* 73: I2C2 error interrupt */
#define GD32_IRQ_USBHS_EP1_Out              (GD32_IRQ_EXINT+74) /* 74: USBHS endpoint 1 out interrupt */
#define GD32_IRQ_USBHS_EP1_In               (GD32_IRQ_EXINT+75) /* 75: USBHS endpoint 1 in interrupt */
#define GD32_IRQ_USBHS_WKUP                 (GD32_IRQ_EXINT+76) /* 76: USBHS wakeup through EXTI line interrupt */
#define GD32_IRQ_USBHS                      (GD32_IRQ_EXINT+77) /* 77: USBHS interrupt */
#define GD32_IRQ_DCI                        (GD32_IRQ_EXINT+78) /* 78: DCI interrupt */
#define GD32_IRQ_TRNG                       (GD32_IRQ_EXINT+80) /* 80: TRNG interrupt */
#define GD32_IRQ_FPU                        (GD32_IRQ_EXINT+81) /* 81: FPU interrupt */
#define GD32_IRQ_UART6                      (GD32_IRQ_EXINT+82) /* 82: UART6 interrupt */
#define GD32_IRQ_UART7                      (GD32_IRQ_EXINT+83) /* 83: UART7 interrupt */
#define GD32_IRQ_SPI3                       (GD32_IRQ_EXINT+84) /* 84: SPI3 interrupt */
#define GD32_IRQ_SPI4                       (GD32_IRQ_EXINT+85) /* 85: SPI4 interrupt */
#define GD32_IRQ_SPI5                       (GD32_IRQ_EXINT+86) /* 86: SPI5 interrupt */
#define GD32_IRQ_TLI                        (GD32_IRQ_EXINT+88) /* 88: TLI interrupt */
#define GD32_IRQ_TLI_ER                     (GD32_IRQ_EXINT+89) /* 89: TLI error interrupt */
#define GD32_IRQ_IPA                        (GD32_IRQ_EXINT+90) /* 90: IPA interrupt */

#elif defined(CONFIG_GD32F4_GD32F407)
#define GD32_IRQ_EXMC                       (GD32_IRQ_EXINT+48) /* 48: EXMC interrupt */
#define GD32_IRQ_SDIO                       (GD32_IRQ_EXINT+49) /* 49: SDIO interrupt */
#define GD32_IRQ_TIMER4                     (GD32_IRQ_EXINT+50) /* 50: TIMER4 interrupt */
#define GD32_IRQ_SPI2                       (GD32_IRQ_EXINT+51) /* 51: SPI2 interrupt */
#define GD32_IRQ_UART3                      (GD32_IRQ_EXINT+52) /* 52: UART3 interrupt */
#define GD32_IRQ_UART4                      (GD32_IRQ_EXINT+53) /* 53: UART4 interrupt */
#define GD32_IRQ_TIMER5_DAC                 (GD32_IRQ_EXINT+54) /* 54: TIMER5 and DAC0 DAC1 underrun error interrupts */
#define GD32_IRQ_TIMER6                     (GD32_IRQ_EXINT+55) /* 55: TIMER6 interrupt */
#define GD32_IRQ_DMA1_CHANNEL0              (GD32_IRQ_EXINT+56) /* 56: DMA1 channel0 interrupt */
#define GD32_IRQ_DMA1_CHANNEL1              (GD32_IRQ_EXINT+57) /* 57: DMA1 channel1 interrupt */
#define GD32_IRQ_DMA1_CHANNEL2              (GD32_IRQ_EXINT+58) /* 58: DMA1 channel2 interruptt */
#define GD32_IRQ_DMA1_CHANNEL3              (GD32_IRQ_EXINT+59) /* 59: DMA1 channel3 interrupt */
#define GD32_IRQ_DMA1_CHANNEL4              (GD32_IRQ_EXINT+60) /* 60: DMA1 channel4 interrupt */
#define GD32_IRQ_ENET                       (GD32_IRQ_EXINT+61) /* 61: ENET interrupt */
#define GD32_IRQ_ENET_WKUP                  (GD32_IRQ_EXINT+62) /* 62: ENET wakeup interrupt */
#define GD32_IRQ_CAN1_TX                    (GD32_IRQ_EXINT+63) /* 63: CAN1 TX interrupt */
#define GD32_IRQ_CAN1_RX0                   (GD32_IRQ_EXINT+64) /* 64: CAN1 RX0 interrupt */
#define GD32_IRQ_CAN1_RX1                   (GD32_IRQ_EXINT+65) /* 65: CAN1 RX1 interrupt */
#define GD32_IRQ_CAN1_EWMC                  (GD32_IRQ_EXINT+66) /* 66: CAN1 EWMC interrupt */
#define GD32_IRQ_USBFS                      (GD32_IRQ_EXINT+67) /* 67: USBFS interrupt */
#define GD32_IRQ_DMA1_CHANNEL5              (GD32_IRQ_EXINT+68) /* 68: DMA1 channel5 interrupt */
#define GD32_IRQ_DMA1_CHANNEL6              (GD32_IRQ_EXINT+69) /* 69: DMA1 channel6 interrupt */
#define GD32_IRQ_DMA1_CHANNEL7              (GD32_IRQ_EXINT+70) /* 70: DMA1 channel7 interrupt */
#define GD32_IRQ_USART5                     (GD32_IRQ_EXINT+71) /* 71: USART5 interrupt */
#define GD32_IRQ_I2C2_EV                    (GD32_IRQ_EXINT+72) /* 72: I2C2 event interrupt */
#define GD32_IRQ_I2C2_ER                    (GD32_IRQ_EXINT+73) /* 73: I2C2 error interrupt */
#define GD32_IRQ_USBHS_EP1_Out              (GD32_IRQ_EXINT+74) /* 74: USBHS endpoint 1 out interrupt */
#define GD32_IRQ_USBHS_EP1_In               (GD32_IRQ_EXINT+75) /* 75: USBHS endpoint 1 in interrupt */
#define GD32_IRQ_USBHS_WKUP                 (GD32_IRQ_EXINT+76) /* 76: USBHS wakeup through EXTI line interrupt */
#define GD32_IRQ_USBHS                      (GD32_IRQ_EXINT+77) /* 77: USBHS interrupt */
#define GD32_IRQ_DCI                        (GD32_IRQ_EXINT+78) /* 78: DCI interrupt */
#define GD32_IRQ_TRNG                       (GD32_IRQ_EXINT+80) /* 80: TRNG interrupt */
#define GD32_IRQ_FPU                        (GD32_IRQ_EXINT+81) /* 81: FPU interrupt */

#elif defined(CONFIG_GD32F4_GD32F405)
#define GD32_IRQ_SDIO                       (GD32_IRQ_EXINT+49) /* 49: SDIO interrupt */
#define GD32_IRQ_TIMER4                     (GD32_IRQ_EXINT+50) /* 50: TIMER4 interrupt */
#define GD32_IRQ_SPI2                       (GD32_IRQ_EXINT+51) /* 51: SPI2 interrupt */
#define GD32_IRQ_UART3                      (GD32_IRQ_EXINT+52) /* 52: UART3 interrupt */
#define GD32_IRQ_UART4                      (GD32_IRQ_EXINT+53) /* 53: UART4 interrupt */
#define GD32_IRQ_TIMER5_DAC                 (GD32_IRQ_EXINT+54) /* 54: TIMER5 and DAC0 DAC1 underrun error interrupts */
#define GD32_IRQ_TIMER6                     (GD32_IRQ_EXINT+55) /* 55: TIMER6 interrupt */
#define GD32_IRQ_DMA1_CHANNEL0              (GD32_IRQ_EXINT+56) /* 56: DMA1 channel0 interrupt */
#define GD32_IRQ_DMA1_CHANNEL1              (GD32_IRQ_EXINT+57) /* 57: DMA1 channel1 interrupt */
#define GD32_IRQ_DMA1_CHANNEL2              (GD32_IRQ_EXINT+58) /* 58: DMA1 channel2 interruptt */
#define GD32_IRQ_DMA1_CHANNEL3              (GD32_IRQ_EXINT+59) /* 59: DMA1 channel3 interrupt */
#define GD32_IRQ_DMA1_CHANNEL4              (GD32_IRQ_EXINT+60) /* 60: DMA1 channel4 interrupt */
#define GD32_IRQ_CAN1_TX                    (GD32_IRQ_EXINT+63) /* 63: CAN1 TX interrupt */
#define GD32_IRQ_CAN1_RX0                   (GD32_IRQ_EXINT+64) /* 64: CAN1 RX0 interrupt */
#define GD32_IRQ_CAN1_RX1                   (GD32_IRQ_EXINT+65) /* 65: CAN1 RX1 interrupt */
#define GD32_IRQ_CAN1_EWMC                  (GD32_IRQ_EXINT+66) /* 66: CAN1 EWMC interrupt */
#define GD32_IRQ_USBFS                      (GD32_IRQ_EXINT+67) /* 67: USBFS interrupt */
#define GD32_IRQ_DMA1_CHANNEL5              (GD32_IRQ_EXINT+68) /* 68: DMA1 channel5 interrupt */
#define GD32_IRQ_DMA1_CHANNEL6              (GD32_IRQ_EXINT+69) /* 69: DMA1 channel6 interrupt */
#define GD32_IRQ_DMA1_CHANNEL7              (GD32_IRQ_EXINT+70) /* 70: DMA1 channel7 interrupt */
#define GD32_IRQ_USART5                     (GD32_IRQ_EXINT+71) /* 71: USART5 interrupt */
#define GD32_IRQ_I2C2_EV                    (GD32_IRQ_EXINT+72) /* 72: I2C2 event interrupt */
#define GD32_IRQ_I2C2_ER                    (GD32_IRQ_EXINT+73) /* 73: I2C2 error interrupt */
#define GD32_IRQ_USBHS_EP1_Out              (GD32_IRQ_EXINT+74) /* 74: USBHS endpoint 1 out interrupt */
#define GD32_IRQ_USBHS_EP1_In               (GD32_IRQ_EXINT+75) /* 75: USBHS endpoint 1 in interrupt */
#define GD32_IRQ_USBHS_WKUP                 (GD32_IRQ_EXINT+76) /* 76: USBHS wakeup through EXTI line interrupt */
#define GD32_IRQ_USBHS                      (GD32_IRQ_EXINT+77) /* 77: USBHS interrupt */
#define GD32_IRQ_DCI                        (GD32_IRQ_EXINT+78) /* 78: DCI interrupt */
#define GD32_IRQ_TRNG                       (GD32_IRQ_EXINT+80) /* 80: TRNG interrupt */
#define GD32_IRQ_FPU                        (GD32_IRQ_EXINT+81) /* 81: FPU interrupt */

#else
    #error "Unkonwn GD32F4xx chip."
#endif /* CONFIG_GD32F4_GD32F450 */

#if defined(CONFIG_GD32F4_GD32F450)
#  define GD32_IRQ_NEXTINT      (91)
#  define NR_IRQS                (GD32_IRQ_EXINT + GD32_IRQ_NEXTINT)
#elif defined(CONFIG_GD32F4_GD32F407)
#  define GD32_IRQ_NEXTINT      (82)
#  define NR_IRQS                (GD32_IRQ_EXINT + GD32_IRQ_NEXTINT)
#elif defined(CONFIG_GD32F4_GD32F405)
#  define GD32_IRQ_NEXTINT      (82)
#  define NR_IRQS                (GD32_IRQ_EXINT + GD32_IRQ_NEXTINT)
#else
#  error "Unknown GD32F4xx chip!"
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

#endif /* __ARCH_ARM_INCLUDE_GD32_GD32F4XX_IRQ_H */
