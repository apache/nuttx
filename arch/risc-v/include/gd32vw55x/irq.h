/****************************************************************************
 * arch/risc-v/include/gd32vw55x/irq.h
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

#ifndef __ARCH_RISCV_INCLUDE_GD32VW55X_IRQ_H
#define __ARCH_RISCV_INCLUDE_GD32VW55X_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* All interrupt sources (including the core software and timer interrupts)
 * are inputs of the Nuclei ECLIC.  The ECLIC interrupt ID is mapped to the
 * NuttX IRQ number as:  IRQ = RISCV_IRQ_ASYNC + ECLIC ID.
 *
 * ECLIC IDs 3 (software) and 7 (machine timer) therefore land exactly on
 * RISCV_IRQ_MSOFT and RISCV_IRQ_MTIMER.
 */

#define GD32VW55X_IRQ_SFT            (RISCV_IRQ_ASYNC + 3)   /* Core software */
#define GD32VW55X_IRQ_TMR            (RISCV_IRQ_ASYNC + 7)   /* Core timer (mtime) */

/* Peripheral interrupt sources (ECLIC IDs) */

#define GD32VW55X_IRQ_WWDGT          (RISCV_IRQ_ASYNC + 19)  /* Window watchdog */
#define GD32VW55X_IRQ_LVD            (RISCV_IRQ_ASYNC + 20)  /* LVD through EXTI */
#define GD32VW55X_IRQ_TAMPER_STAMP   (RISCV_IRQ_ASYNC + 21)  /* Tamper/timestamp */
#define GD32VW55X_IRQ_RTC_WKUP       (RISCV_IRQ_ASYNC + 22)  /* RTC wakeup */
#define GD32VW55X_IRQ_FMC            (RISCV_IRQ_ASYNC + 23)  /* Flash controller */
#define GD32VW55X_IRQ_RCU            (RISCV_IRQ_ASYNC + 24)  /* Reset/clock unit */
#define GD32VW55X_IRQ_EXTI0          (RISCV_IRQ_ASYNC + 25)  /* EXTI line 0 */
#define GD32VW55X_IRQ_EXTI1          (RISCV_IRQ_ASYNC + 26)  /* EXTI line 1 */
#define GD32VW55X_IRQ_EXTI2          (RISCV_IRQ_ASYNC + 27)  /* EXTI line 2 */
#define GD32VW55X_IRQ_EXTI3          (RISCV_IRQ_ASYNC + 28)  /* EXTI line 3 */
#define GD32VW55X_IRQ_EXTI4          (RISCV_IRQ_ASYNC + 29)  /* EXTI line 4 */
#define GD32VW55X_IRQ_DMA_CH0        (RISCV_IRQ_ASYNC + 30)  /* DMA channel 0 */
#define GD32VW55X_IRQ_DMA_CH1        (RISCV_IRQ_ASYNC + 31)  /* DMA channel 1 */
#define GD32VW55X_IRQ_DMA_CH2        (RISCV_IRQ_ASYNC + 32)  /* DMA channel 2 */
#define GD32VW55X_IRQ_DMA_CH3        (RISCV_IRQ_ASYNC + 33)  /* DMA channel 3 */
#define GD32VW55X_IRQ_DMA_CH4        (RISCV_IRQ_ASYNC + 34)  /* DMA channel 4 */
#define GD32VW55X_IRQ_DMA_CH5        (RISCV_IRQ_ASYNC + 35)  /* DMA channel 5 */
#define GD32VW55X_IRQ_DMA_CH6        (RISCV_IRQ_ASYNC + 36)  /* DMA channel 6 */
#define GD32VW55X_IRQ_DMA_CH7        (RISCV_IRQ_ASYNC + 37)  /* DMA channel 7 */
#define GD32VW55X_IRQ_ADC            (RISCV_IRQ_ASYNC + 38)  /* ADC */
#define GD32VW55X_IRQ_EXTI5_9        (RISCV_IRQ_ASYNC + 42)  /* EXTI lines 5..9 */
#define GD32VW55X_IRQ_TIMER0_BRK     (RISCV_IRQ_ASYNC + 43)  /* TIMER0 break */
#define GD32VW55X_IRQ_TIMER0_UP      (RISCV_IRQ_ASYNC + 44)  /* TIMER0 update */
#define GD32VW55X_IRQ_TIMER0_CMT     (RISCV_IRQ_ASYNC + 45)  /* TIMER0 commutation */
#define GD32VW55X_IRQ_TIMER0_CC      (RISCV_IRQ_ASYNC + 46)  /* TIMER0 capture/compare */
#define GD32VW55X_IRQ_TIMER1         (RISCV_IRQ_ASYNC + 47)  /* TIMER1 (32-bit) */
#define GD32VW55X_IRQ_TIMER2         (RISCV_IRQ_ASYNC + 48)  /* TIMER2 (32-bit) */
#define GD32VW55X_IRQ_I2C0_EV        (RISCV_IRQ_ASYNC + 50)  /* I2C0 event */
#define GD32VW55X_IRQ_I2C0_ER        (RISCV_IRQ_ASYNC + 51)  /* I2C0 error */
#define GD32VW55X_IRQ_I2C1_EV        (RISCV_IRQ_ASYNC + 52)  /* I2C1 event */
#define GD32VW55X_IRQ_I2C1_ER        (RISCV_IRQ_ASYNC + 53)  /* I2C1 error */
#define GD32VW55X_IRQ_SPI            (RISCV_IRQ_ASYNC + 54)  /* SPI */
#define GD32VW55X_IRQ_USART0         (RISCV_IRQ_ASYNC + 56)  /* USART0 */
#define GD32VW55X_IRQ_UART1          (RISCV_IRQ_ASYNC + 57)  /* UART1 */
#define GD32VW55X_IRQ_UART2          (RISCV_IRQ_ASYNC + 58)  /* UART2 */
#define GD32VW55X_IRQ_EXTI10_15      (RISCV_IRQ_ASYNC + 59)  /* EXTI lines 10..15 */
#define GD32VW55X_IRQ_RTC_ALARM      (RISCV_IRQ_ASYNC + 60)  /* RTC alarm */
#define GD32VW55X_IRQ_VLVDF          (RISCV_IRQ_ASYNC + 61)  /* VDDA low voltage */
#define GD32VW55X_IRQ_TIMER15        (RISCV_IRQ_ASYNC + 63)  /* TIMER15 */
#define GD32VW55X_IRQ_TIMER16        (RISCV_IRQ_ASYNC + 64)  /* TIMER16 */
#define GD32VW55X_IRQ_I2C0_WKUP      (RISCV_IRQ_ASYNC + 70)  /* I2C0 wakeup */
#define GD32VW55X_IRQ_USART0_WKUP    (RISCV_IRQ_ASYNC + 71)  /* USART0 wakeup */
#define GD32VW55X_IRQ_TIMER5         (RISCV_IRQ_ASYNC + 73)  /* TIMER5 (basic) */
#define GD32VW55X_IRQ_WIFI_PROT      (RISCV_IRQ_ASYNC + 74)  /* Wi-Fi protocol */
#define GD32VW55X_IRQ_WIFI_INTGEN    (RISCV_IRQ_ASYNC + 75)  /* Wi-Fi general */
#define GD32VW55X_IRQ_WIFI_TX        (RISCV_IRQ_ASYNC + 76)  /* Wi-Fi TX */
#define GD32VW55X_IRQ_WIFI_RX        (RISCV_IRQ_ASYNC + 77)  /* Wi-Fi RX */
#define GD32VW55X_IRQ_LA             (RISCV_IRQ_ASYNC + 83)  /* Logic analyzer */
#define GD32VW55X_IRQ_WIFI_WKUP      (RISCV_IRQ_ASYNC + 84)  /* Wi-Fi wakeup */
#define GD32VW55X_IRQ_BLE_WKUP       (RISCV_IRQ_ASYNC + 85)  /* BLE wakeup */
#define GD32VW55X_IRQ_PLATFORM_WAKE  (RISCV_IRQ_ASYNC + 86)  /* Platform wakeup */
#define GD32VW55X_IRQ_ISO_BT_STAMP0  (RISCV_IRQ_ASYNC + 87)  /* ISO BT timestamp 0 */
#define GD32VW55X_IRQ_ISO_BT_STAMP1  (RISCV_IRQ_ASYNC + 88)  /* ISO BT timestamp 1 */
#define GD32VW55X_IRQ_ISO_BT_STAMP2  (RISCV_IRQ_ASYNC + 89)  /* ISO BT timestamp 2 */
#define GD32VW55X_IRQ_ISO_BT_STAMP3  (RISCV_IRQ_ASYNC + 90)  /* ISO BT timestamp 3 */
#define GD32VW55X_IRQ_ISO_BT_STAMP4  (RISCV_IRQ_ASYNC + 91)  /* ISO BT timestamp 4 */
#define GD32VW55X_IRQ_ISO_BT_STAMP5  (RISCV_IRQ_ASYNC + 92)  /* ISO BT timestamp 5 */
#define GD32VW55X_IRQ_ISO_BT_STAMP6  (RISCV_IRQ_ASYNC + 93)  /* ISO BT timestamp 6 */
#define GD32VW55X_IRQ_ISO_BT_STAMP7  (RISCV_IRQ_ASYNC + 94)  /* ISO BT timestamp 7 */
#define GD32VW55X_IRQ_BLE_POWER_STAT (RISCV_IRQ_ASYNC + 95)  /* BLE power status */
#define GD32VW55X_IRQ_CAU            (RISCV_IRQ_ASYNC + 98)  /* Crypto (CAU) */
#define GD32VW55X_IRQ_HAU_TRNG       (RISCV_IRQ_ASYNC + 99)  /* Hash / TRNG */
#define GD32VW55X_IRQ_WIFI_INT       (RISCV_IRQ_ASYNC + 101) /* Wi-Fi common */
#define GD32VW55X_IRQ_BLE_SW_TRIG    (RISCV_IRQ_ASYNC + 102) /* BLE SW trigger */
#define GD32VW55X_IRQ_BLE_FINE_TGT   (RISCV_IRQ_ASYNC + 103) /* BLE fine timer */
#define GD32VW55X_IRQ_BLE_STAMP_TGT1 (RISCV_IRQ_ASYNC + 104) /* BLE timestamp 1 */
#define GD32VW55X_IRQ_BLE_STAMP_TGT2 (RISCV_IRQ_ASYNC + 105) /* BLE timestamp 2 */
#define GD32VW55X_IRQ_BLE_STAMP_TGT3 (RISCV_IRQ_ASYNC + 106) /* BLE timestamp 3 */
#define GD32VW55X_IRQ_BLE_ENC_ENGINE (RISCV_IRQ_ASYNC + 107) /* BLE encryption */
#define GD32VW55X_IRQ_BLE_SLEEP_MODE (RISCV_IRQ_ASYNC + 108) /* BLE sleep mode */
#define GD32VW55X_IRQ_BLE_HALF_SLOT  (RISCV_IRQ_ASYNC + 109) /* BLE half slot */
#define GD32VW55X_IRQ_BLE_FIFO       (RISCV_IRQ_ASYNC + 110) /* BLE FIFO activity */
#define GD32VW55X_IRQ_BLE_ERROR      (RISCV_IRQ_ASYNC + 111) /* BLE error */
#define GD32VW55X_IRQ_BLE_FREQ_SEL   (RISCV_IRQ_ASYNC + 112) /* BLE freq select */
#define GD32VW55X_IRQ_EFUSE          (RISCV_IRQ_ASYNC + 113) /* EFUSE */
#define GD32VW55X_IRQ_QSPI           (RISCV_IRQ_ASYNC + 114) /* QSPI */
#define GD32VW55X_IRQ_PKCAU          (RISCV_IRQ_ASYNC + 115) /* Public key crypto */

/* Total number of ECLIC interrupt inputs */

#define GD32VW55X_NIRQS              116

#define NR_IRQS                      (RISCV_IRQ_ASYNC + GD32VW55X_NIRQS)

#endif /* __ARCH_RISCV_INCLUDE_GD32VW55X_IRQ_H */
