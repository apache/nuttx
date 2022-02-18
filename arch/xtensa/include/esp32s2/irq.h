/****************************************************************************
 * arch/xtensa/include/esp32s2/irq.h
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
 * through nuttx/irq.h
 */

#ifndef __ARCH_XTENSA_INCLUDE_ESP32S2_IRQ_H
#define __ARCH_XTENSA_INCLUDE_ESP32S2_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/esp32s2/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Interrupt Matrix
 *
 * Features
 * - Accepts 95 peripheral interrupt sources as input.
 * - Generates 26 peripheral interrupt sources as output.
 * - CPU NMI Interrupt Mask.
 * - Queries current interrupt status of peripheral interrupt sources.
 *
 * Peripheral Interrupt Source
 *
 * ESP32S2 has 95 peripheral interrupt sources in total. 67 of 71 ESP32S2
 * peripheral interrupt sources can be allocated to either CPU.  The four
 * remaining peripheral interrupt sources are CPU-specific, two per CPU.
 *
 * - GPIO_INTERRUPT_PRO and GPIO_INTERRUPT_PRO_NMI can only be allocated to
 *   PRO_CPU.
 * - GPIO_INTERRUPT_APP and GPIO_INTERRUPT_APP_NMI can only be allocated to
 *   APP_CPU.
 *
 * As a result, PRO_CPU and APP_CPU each have 69 peripheral interrupt
 * sources.
 */

/* PRO_INTR_STATUS_REG_0 */

#define ESP32S2_PERI_MAC               0  /* INTR_STATUS_REG_0, bit 0 */
#define ESP32S2_PERI_MAC_NMI           1  /* INTR_STATUS_REG_0, bit 1 */
#define ESP32S2_PERI_PWR               2  /* INTR_STATUS_REG_0, bit 2 */
#define ESP32S2_PERI_BB                3  /* INTR_STATUS_REG_0, bit 3 */
#define ESP32S2_PERI_BT_MAC            4  /* INTR_STATUS_REG_0, bit 4 */
#define ESP32S2_PERI_BT_BB             5  /* INTR_STATUS_REG_0, bit 5 */
#define ESP32S2_PERI_BT_BB_NMI         6  /* INTR_STATUS_REG_0, bit 6 */
#define ESP32S2_PERI_RWBT              7  /* INTR_STATUS_REG_0, bit 7 */
#define ESP32S2_PERI_RWBLE             8  /* INTR_STATUS_REG_0, bit 8 */
#define ESP32S2_PERI_RWBT_NMI          9  /* INTR_STATUS_REG_0, bit 9 */

#define ESP32S2_PERI_RWBLE_NMI         10 /* INTR_STATUS_REG_0, bit 10 */
#define ESP32S2_PERI_SLC0              11 /* INTR_STATUS_REG_0, bit 11 */
#define ESP32S2_PERI_SLC1              12 /* INTR_STATUS_REG_0, bit 12 */
#define ESP32S2_PERI_UHCI0             13 /* INTR_STATUS_REG_0, bit 13 */
#define ESP32S2_PERI_UHCI1             14 /* INTR_STATUS_REG_0, bit 14 */
#define ESP32S2_PERI_TG_T0_LEVEL       15 /* INTR_STATUS_REG_0, bit 15 */
#define ESP32S2_PERI_TG_T1_LEVEL       16 /* INTR_STATUS_REG_0, bit 16 */
#define ESP32S2_PERI_TG_WDT_LEVEL      17 /* INTR_STATUS_REG_0, bit 17 */
#define ESP32S2_PERI_TG_LACT_LEVEL     18 /* INTR_STATUS_REG_0, bit 18 */
#define ESP32S2_PERI_TG1_T0_LEVEL      19 /* INTR_STATUS_REG_0, bit 19 */

#define ESP32S2_PERI_TG1_T1_LEVEL      20 /* INTR_STATUS_REG_0, bit 20 */
#define ESP32S2_PERI_TG1_WDT_LEVEL     21 /* INTR_STATUS_REG_0, bit 21 */
#define ESP32S2_PERI_TG1_LACT_LEVEL    22 /* INTR_STATUS_REG_0, bit 22 */
#define ESP32S2_PERI_GPIO_INT_PRO      23 /* INTR_STATUS_REG_0, bit 23 */
#define ESP32S2_PERI_GPIO_INT_PRO_NMI  24 /* INTR_STATUS_REG_0, bit 24 */
#define ESP32S2_PERI_GPIO_INT_APP      25 /* INTR_STATUS_REG_0, bit 25 */
#define ESP32S2_PERI_GPIO_INT_APP_NMI  26 /* INTR_STATUS_REG_0, bit 26 */
#define ESP32S2_PERI_DEDICATED_GPIO_IN 27 /* INTR_STATUS_REG_0, bit 27 */
#define ESP32S2_PERI_INT_FROM_CPU0     28 /* INTR_STATUS_REG_0, bit 28 */
#define ESP32S2_PERI_INT_FROM_CPU1     29 /* INTR_STATUS_REG_0, bit 29 */

#define ESP32S2_PERI_INT_FROM_CPU2     30 /* INTR_STATUS_REG_0, bit 30 */
#define ESP32S2_PERI_INT_FROM_CPU3     31 /* INTR_STATUS_REG_0, bit 31 */

/* PRO_INTR_STATUS_REG_1 */

#define ESP32S2_PERI_SPI1              32 /* INTR_STATUS_REG_1, bit 0 */
#define ESP32S2_PERI_SPI2              33 /* INTR_STATUS_REG_1, bit 1 */
#define ESP32S2_PERI_SPI3              34 /* INTR_STATUS_REG_1, bit 2 */
#define ESP32S2_PERI_I2S0              35 /* INTR_STATUS_REG_1, bit 3 */
#define ESP32S2_PERI_I2S1              36 /* INTR_STATUS_REG_1, bit 4 */
#define ESP32S2_PERI_UART              37 /* INTR_STATUS_REG_1, bit 5 */
#define ESP32S2_PERI_UART1             38 /* INTR_STATUS_REG_1, bit 6 */
#define ESP32S2_PERI_UART2             39 /* INTR_STATUS_REG_1, bit 7 */
#define ESP32S2_PERI_SDIO_HOST         40 /* INTR_STATUS_REG_1, bit 8 */
#define ESP32S2_PERI_PWM0              41 /* INTR_STATUS_REG_1, bit 9 */

#define ESP32S2_PERI_PWM1              42 /* INTR_STATUS_REG_1, bit 10 */
#define ESP32S2_PERI_PWM2              43 /* INTR_STATUS_REG_1, bit 11 */
#define ESP32S2_PERI_PWM3              44 /* INTR_STATUS_REG_1, bit 12 */
#define ESP32S2_PERI_LEDC              45 /* INTR_STATUS_REG_1, bit 13 */
#define ESP32S2_PERI_EFUSE             46 /* INTR_STATUS_REG_1, bit 14 */
#define ESP32S2_PERI_CAN               47 /* INTR_STATUS_REG_1, bit 15 */
#define ESP32S2_PERI_USB               48 /* INTR_STATUS_REG_1, bit 16 */
#define ESP32S2_PERI_RTC_CORE          49 /* INTR_STATUS_REG_1, bit 17 */
#define ESP32S2_PERI_RMT               50 /* INTR_STATUS_REG_1, bit 18 */
#define ESP32S2_PERI_PCNT              51 /* INTR_STATUS_REG_1, bit 19 */

#define ESP32S2_PERI_I2C_EXT0          52 /* INTR_STATUS_REG_1, bit 20 */
#define ESP32S2_PERI_I2C_EXT1          53 /* INTR_STATUS_REG_1, bit 21 */
#define ESP32S2_PERI_RSA               54 /* INTR_STATUS_REG_1, bit 22 */
#define ESP32S2_PERI_SHA               55 /* INTR_STATUS_REG_1, bit 23 */
#define ESP32S2_PERI_AES               56 /* INTR_STATUS_REG_1, bit 24 */
#define ESP32S2_PERI_SPI2_DMA          57 /* INTR_STATUS_REG_1, bit 25 */
#define ESP32S2_PERI_SPI3_DMA          58 /* INTR_STATUS_REG_1, bit 26 */
#define ESP32S2_PERI_WDG               59 /* INTR_STATUS_REG_1, bit 27 */
#define ESP32S2_PERI_TIMER             60 /* INTR_STATUS_REG_1, bit 28 */
#define ESP32S2_PERI_TIMER_INT2        61 /* INTR_STATUS_REG_1, bit 29 */

#define ESP32S2_PERI_TG_T0_EDGE        62 /* INTR_STATUS_REG_1, bit 30 */
#define ESP32S2_PERI_TG_T1_EDGE        63 /* INTR_STATUS_REG_1, bit 31 */

/* PRO_INTR_STATUS_REG_2 */

#define ESP32S2_PERI_TG_WDT_EDGE       64  /* INTR_STATUS_REG_2, bit 0 */
#define ESP32S2_PERI_TG_LACT_EDGE      65  /* INTR_STATUS_REG_2, bit 1 */
#define ESP32S2_PERI_TG1_T0_EDGE       66  /* INTR_STATUS_REG_2, bit 2 */
#define ESP32S2_PERI_TG1_T1_EDGE       67  /* INTR_STATUS_REG_2, bit 3 */
#define ESP32S2_PERI_TG1_WDT_EDGE      68  /* INTR_STATUS_REG_2, bit 4 */
#define ESP32S2_PERI_TG1_LACT_EDGE     69  /* INTR_STATUS_REG_2, bit 5 */
#define ESP32S2_PERI_CACHE_IA          70  /* INTR_STATUS_REG_2, bit 6 */
#define ESP32S2_PERI_SYSTIMER_TARGET0  71  /* INTR_STATUS_REG_2, bit 7 */
#define ESP32S2_PERI_SYSTIMER_TARGET1  72  /* INTR_STATUS_REG_2, bit 8 */
#define ESP32S2_PERI_SYSTIMER_TARGET2  73  /* INTR_STATUS_REG_2, bit 9 */

#define ESP32S2_PERI_ASSIST_DEBUG      74 /* INTR_STATUS_REG_2, bit 10 */
#define ESP32S2_PERI_PMS_PRO_IRAM0_ILG 75 /* INTR_STATUS_REG_2, bit 11 */
#define ESP32S2_PERI_PMS_PRO_DRAM0_ILG 76 /* INTR_STATUS_REG_2, bit 12 */
#define ESP32S2_PERI_PMS_PRO_DPORT_ILG 77 /* INTR_STATUS_REG_2, bit 13 */
#define ESP32S2_PERI_PMS_PRO_AHB_ILG   78 /* INTR_STATUS_REG_2, bit 14 */
#define ESP32S2_PERI_PMS_PRO_CACHE_ILG 79 /* INTR_STATUS_REG_2, bit 15 */
#define ESP32S2_PERI_PMS_DMA_APB_I_ILG 80 /* INTR_STATUS_REG_2, bit 16 */
#define ESP32S2_PERI_PMS_DMA_RX_I_ILG  81 /* INTR_STATUS_REG_2, bit 17 */
#define ESP32S2_PERI_PMS_DMA_TX_I_ILG  82 /* INTR_STATUS_REG_2, bit 18 */
#define ESP32S2_PERI_SPI_MEM_REJECT    83 /* INTR_STATUS_REG_2, bit 19 */

#define ESP32S2_PERI_DMA_COPY          84 /* INTR_STATUS_REG_2, bit 20 */
#define ESP32S2_PERI_SPI4_DMA          85 /* INTR_STATUS_REG_2, bit 21 */
#define ESP32S2_PERI_SPI4              86 /* INTR_STATUS_REG_2, bit 22 */
#define ESP32S2_PERI_DCACHE_PRELOAD    87 /* INTR_STATUS_REG_2, bit 23 */
#define ESP32S2_PERI_ICACHE_PRELOAD    88 /* INTR_STATUS_REG_2, bit 24 */
#define ESP32S2_PERI_APB_ADC           89 /* INTR_STATUS_REG_2, bit 25 */
#define ESP32S2_PERI_CRYPTO_DMA        90 /* INTR_STATUS_REG_2, bit 26 */
#define ESP32S2_PERI_CPU_PERI_ERR      91 /* INTR_STATUS_REG_2, bit 27 */
#define ESP32S2_PERI_APB_PERI_ERR      92 /* INTR_STATUS_REG_2, bit 28 */
#define ESP32S2_PERI_DCACHE_SYNC       93 /* INTR_STATUS_REG_2, bit 29 */
#define ESP32S2_PERI_ICACHE_SYNC       94 /* INTR_STATUS_REG_2, bit 29 */

/* Total number of peripherals */

#define ESP32S2_NPERIPHERALS          95

/* Exceptions
 *
 * IRAM Offset  Description
 *   0x0000     Windows
 *   0x0180     Level 2 interrupt
 *   0x01c0     Level 3 interrupt
 *   0x0200     Level 4 interrupt
 *   0x0240     Level 5 interrupt
 *   0x0280     Debug exception
 *   0x02c0     NMI exception
 *   0x0300     Kernel exception
 *   0x0340     User exception
 *   0x03c0     Double exception
 *
 * REVISIT: In more architectures supported by NuttX, exception errors
 * tie into the normal interrupt handling via special IRQ numbers.
 * It is still to be determined what will be done for the ESP32S2.
 *
 */

/* IRQ numbers for internal interrupts that are dispatched like peripheral
 * interrupts
 */

#define XTENSA_IRQ_TIMER0           0  /* INTERRUPT, bit 6 */
#define XTENSA_IRQ_TIMER1           1  /* INTERRUPT, bit 15 */
#define XTENSA_IRQ_TIMER2           2  /* INTERRUPT, bit 16 */
#define XTENSA_IRQ_SYSCALL          3  /* User interrupt w/EXCCAUSE=syscall */
#define XTENSA_IRQ_SWINT            4  /* Software interrupt */

#define XTENSA_NIRQ_INTERNAL        5  /* Number of dispatch internal interrupts */
#define XTENSA_IRQ_FIRSTPERI        5  /* First peripheral IRQ number */

/* IRQ numbers for peripheral interrupts coming through the Interrupt
 * Matrix.
 */

#define ESP32S2_IRQ2PERIPH(irq)       ((irq)-XTENSA_IRQ_FIRSTPERI)

/* PRO_INTR_STATUS_REG_0 */

#define ESP32S2_IRQ_MAC               (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_MAC)
#define ESP32S2_IRQ_MAC_NMI           (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_MAC_NMI)
#define ESP32S2_IRQ_PWR               (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_PWR)
#define ESP32S2_IRQ_BB                (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_BB)
#define ESP32S2_IRQ_BT_MAC            (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_BT_MAC)
#define ESP32S2_IRQ_BT_BB             (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_BB)
#define ESP32S2_IRQ_BT_BB_NMI         (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_BB_NMI)
#define ESP32S2_IRQ_RWBT              (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_RWBT)
#define ESP32S2_IRQ_RWBLE             (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_RWBLE)
#define ESP32S2_IRQ_RWBT_NMI          (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_RWBT_NMI)

#define ESP32S2_IRQ_RWBLE_NMI         (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_RWBLE_NMI)
#define ESP32S2_IRQ_SLC0              (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_SLC0)
#define ESP32S2_IRQ_SLC1              (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_SLC1)
#define ESP32S2_IRQ_UHCI0             (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_UHCI0)
#define ESP32S2_IRQ_UHCI1             (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_UHCI1)
#define ESP32S2_IRQ_TG_T0_LEVEL       (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_TG_T0_LEVEL)
#define ESP32S2_IRQ_TG_T1_LEVEL       (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_TG_T1_LEVEL)
#define ESP32S2_IRQ_TG_WDT_LEVEL      (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_TG_WDT_LEVEL)
#define ESP32S2_IRQ_TG_LACT_LEVEL     (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_TG_LACT_LEVEL)
#define ESP32S2_IRQ_TG1_T0_LEVEL      (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_TG1_T0_LEVEL)

#define ESP32S2_IRQ_TG1_T1_LEVEL      (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_TG1_T1_LEVEL)
#define ESP32S2_IRQ_TG1_WDT_LEVEL     (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_TG1_WDT_LEVEL)
#define ESP32S2_IRQ_TG1_LACT_LEVEL    (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_TG1_LACT_LEVEL)
#define ESP32S2_IRQ_GPIO_INT_PRO      (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_GPIO_INT_PRO)
#define ESP32S2_IRQ_GPIO_INT_PRO_NMI  (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_GPIO_INT_PRO_NMI)
#define ESP32S2_IRQ_GPIO_INT_APP      (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_GPIO_INT_APP)
#define ESP32S2_IRQ_GPIO_INT_APP_NMI  (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_GPIO_INT_APP_NMI)
#define ESP32S2_IRQ_DEDICATED_GPIO_IN (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_DEDICATED_GPIO_IN)
#define ESP32S2_IRQ_INT_FROM_CPU0     (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_INT_FROM_CPU0)
#define ESP32S2_IRQ_INT_FROM_CPU1     (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_INT_FROM_CPU1)

#define ESP32S2_IRQ_INT_FROM_CPU2     (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_INT_FROM_CPU2)
#define ESP32S2_IRQ_INT_FROM_CPU3     (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_INT_FROM_CPU3)

#define ESP32S2_IRQ_SREG0             ESP32S2_IRQ_MAC
#define ESP32S2_NIRQS_SREG0           32

/* PRO_INTR_STATUS_REG_1 */

#define ESP32S2_IRQ_SPI1              (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_SPI1)
#define ESP32S2_IRQ_SPI2              (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_SPI2)
#define ESP32S2_IRQ_SPI3              (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_SPI3)
#define ESP32S2_IRQ_I2S0              (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_I2S0)
#define ESP32S2_IRQ_I2S1              (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_I2S1)
#define ESP32S2_IRQ_UART              (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_UART)
#define ESP32S2_IRQ_UART1             (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_UART1)
#define ESP32S2_IRQ_UART2             (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_UART2)
#define ESP32S2_IRQ_SDIO_HOST         (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_SDIO_HOST)
#define ESP32S2_IRQ_PWM0              (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_PWM0)

#define ESP32S2_IRQ_PWM1              (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_PWM1)
#define ESP32S2_IRQ_PWM2              (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_PWM2)
#define ESP32S2_IRQ_PWM3              (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_PWM3)
#define ESP32S2_IRQ_LEDC              (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_LEDC)
#define ESP32S2_IRQ_EFUSE             (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_EFUSE)
#define ESP32S2_IRQ_CAN               (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_CAN)
#define ESP32S2_IRQ_USB               (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_USB)
#define ESP32S2_IRQ_RTC_CORE          (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_RTC_CORE)
#define ESP32S2_IRQ_RMT               (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_RMT)
#define ESP32S2_IRQ_PCNT              (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_PCNT)

#define ESP32S2_IRQ_I2C_EXT0          (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_I2C_EXT0)
#define ESP32S2_IRQ_I2C_EXT1          (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_I2C_EXT1)
#define ESP32S2_IRQ_RSA               (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_RSA)
#define ESP32S2_IRQ_SHA               (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_SHA)
#define ESP32S2_IRQ_AES               (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_AES)
#define ESP32S2_IRQ_SPI2_DMA          (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_SPI2_DMA)
#define ESP32S2_IRQ_SPI3_DMA          (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_SPI3_DMA)
#define ESP32S2_IRQ_WDG               (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_WDG)
#define ESP32S2_IRQ_TIMER             (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_TIMER)
#define ESP32S2_IRQ_TIMER_INT2        (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_TIMER_INT2)

#define ESP32S2_IRQ_TG_T0_EDGE        (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_TG_T0_EDGE)
#define ESP32S2_IRQ_TG_T1_EDGE        (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_TG_T1_EDGE)

#define ESP32S2_IRQ_SREG1             ESP32S2_IRQ_SPI1
#define ESP32S2_NIRQS_SREG1           32

/* PRO_INTR_STATUS_REG_2 */

#define ESP32S2_IRQ_TG_WDT_EDGE       (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_TG_WDT_EDGE)
#define ESP32S2_IRQ_TG_LACT_EDGE      (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_TG_LACT_EDGE)
#define ESP32S2_IRQ_TG1_T0_EDGE       (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_TG1_T0_EDGE)
#define ESP32S2_IRQ_TG1_T1_EDGE       (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_TG1_T1_EDGE)
#define ESP32S2_IRQ_TG1_WDT_EDGE      (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_TG1_WDT_EDGE)
#define ESP32S2_IRQ_TG1_LACT_EDGE     (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_TG1_LACT_EDGE)
#define ESP32S2_IRQ_CACHE_IA          (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_CACHE_IA)
#define ESP32S2_IRQ_SYSTIMER_TARGET0  (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_SYSTIMER_TARGET0)
#define ESP32S2_IRQ_SYSTIMER_TARGET1  (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_SYSTIMER_TARGET1)
#define ESP32S2_IRQ_SYSTIMER_TARGET2  (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_SYSTIMER_TARGET2)

#define ESP32S2_IRQ_ASSIST_DEBUG      (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_ASSIST_DEBUG)
#define ESP32S2_IRQ_PMS_PRO_IRAM0_ILG (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_PMS_PRO_IRAM0_ILG)
#define ESP32S2_IRQ_PMS_PRO_DRAM0_ILG (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_PMS_PRO_DRAM0_ILG)
#define ESP32S2_IRQ_PMS_PRO_DPORT_ILG (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_PMS_PRO_DPORT_ILG)
#define ESP32S2_IRQ_PMS_PRO_AHB_ILG   (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_PMS_PRO_AHB_ILG)
#define ESP32S2_IRQ_PMS_PRO_CACHE_ILG (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_PMS_PRO_CACHE_ILG)
#define ESP32S2_IRQ_PMS_DMA_APB_I_ILG (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_PMS_DMA_APB_I_ILG)
#define ESP32S2_IRQ_PMS_DMA_RX_I_ILG  (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_PMS_DMA_RX_I_ILG)
#define ESP32S2_IRQ_PMS_DMA_TX_I_ILG  (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_PMS_DMA_TX_I_ILG)
#define ESP32S2_IRQ_SPI_MEM_REJECT    (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_SPI_MEM_REJECT)

#define ESP32S2_IRQ_DMA_COPY          (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_DMA_COPY)
#define ESP32S2_IRQ_SPI4_DMA          (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_SPI4_DMA)
#define ESP32S2_IRQ_SPI4              (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_SPI4)
#define ESP32S2_IRQ_DCACHE_PRELOAD    (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_DCACHE_PRELOAD)
#define ESP32S2_IRQ_ICACHE_PRELOAD    (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_ICACHE_PRELOAD)
#define ESP32S2_IRQ_APB_ADC           (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_APB_ADC)
#define ESP32S2_IRQ_CRYPTO_DMA        (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_CRYPTO_DMA)
#define ESP32S2_IRQ_CPU_PERI_ERR      (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_CPU_PERI_ERR)
#define ESP32S2_IRQ_APB_PERI_ERE      (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_APB_PERI_ERR)
#define ESP32S2_IRQ_DCACHE_SYNC       (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_DCACHE_SYNC)

#define ESP32S2_IRQ_ICACHE_SYNC       (XTENSA_IRQ_FIRSTPERI + ESP32S2_PERI_ICACHE_SYNC)

#define ESP32S2_IRQ_SREG2             ESP32S2_IRQ_TG_WDT_EDGE
#define ESP32S2_NIRQS_SREG2           31

#define ESP32S2_NIRQ_PERIPH           ESP32S2_NPERIPHERALS

/* Second level GPIO interrupts.  GPIO interrupts are decoded and dispatched
 * as a second level of decoding:  The first level dispatches to the GPIO
 * interrupt handler.  The second to the decoded GPIO interrupt handler.
 */

#ifdef CONFIG_ESP32S2_GPIO_IRQ
#  define ESP32S2_NIRQ_GPIO           40
#  define ESP32S2_FIRST_GPIOIRQ       (XTENSA_NIRQ_INTERNAL+ESP32S2_NIRQ_PERIPH)
#  define ESP32S2_LAST_GPIOIRQ        (ESP32S2_FIRST_GPIOIRQ+ESP32S2_NIRQ_GPIO-1)
#  define ESP32S2_PIN2IRQ(p)          ((p) + ESP32S2_FIRST_GPIOIRQ)
#  define ESP32S2_IRQ2PIN(i)          ((i) - ESP32S2_FIRST_GPIOIRQ)
#else
#  define ESP32S2_NIRQ_GPIO           0
#endif

/* Total number of interrupts */

#define NR_IRQS                     (XTENSA_NIRQ_INTERNAL+ESP32S2_NIRQ_PERIPH+ESP32S2_NIRQ_GPIO)

/* Xtensa CPU Interrupts.
 *
 * Each of the two CPUs (PRO and APP) have 32 interrupts each, of which
 * 26 can be mapped to peripheral interrupts:
 *
 *   Level triggered peripherals (21 total):
 *     0-5, 8-9, 12-13, 17-18 - Priority 1
 *     19-21                  - Priority 2
 *     23, 27                 - Priority 3
 *     24-25                  - Priority 4
 *     26, 31                 - Priority 5
 *   Edge triggered peripherals (4 total):
 *     10                     - Priority 1
 *     22                     - Priority 3
 *     28, 30                 - Priority 4
 *   NMI (1 total):
 *     14                     - NMI
 *
 * CPU peripheral interrupts can be a assigned to a CPU interrupt using the
 * PRO_*_MAP_REG or APP_*_MAP_REG.  There are a pair of these registers for
 * each peripheral source.  Multiple peripheral interrupt sources can be
 * mapped to the same CPU interrupt.
 *
 * The remaining, six, internal CPU interrupts are:
 *
 *   6   Timer0    - Priority 1
 *   7   Software  - Priority 1
 *   11  Profiling - Priority 3
 *   15  Timer1    - Priority 3
 *   16  Timer2    - Priority 5
 *   29  Software  - Priority 3
 *
 * A peripheral interrupt can be disabled
 */

#define ESP32S2_CPUINT_LEVELPERIPH_0  0
#define ESP32S2_CPUINT_LEVELPERIPH_1  1
#define ESP32S2_CPUINT_LEVELPERIPH_2  2
#define ESP32S2_CPUINT_LEVELPERIPH_3  3
#define ESP32S2_CPUINT_LEVELPERIPH_4  4
#define ESP32S2_CPUINT_LEVELPERIPH_5  5
#define ESP32S2_CPUINT_LEVELPERIPH_6  8
#define ESP32S2_CPUINT_LEVELPERIPH_7  9
#define ESP32S2_CPUINT_LEVELPERIPH_8  12
#define ESP32S2_CPUINT_LEVELPERIPH_9  13
#define ESP32S2_CPUINT_LEVELPERIPH_10 17
#define ESP32S2_CPUINT_LEVELPERIPH_11 18
#define ESP32S2_CPUINT_LEVELPERIPH_12 19
#define ESP32S2_CPUINT_LEVELPERIPH_13 20
#define ESP32S2_CPUINT_LEVELPERIPH_14 21
#define ESP32S2_CPUINT_LEVELPERIPH_15 23
#define ESP32S2_CPUINT_LEVELPERIPH_16 24
#define ESP32S2_CPUINT_LEVELPERIPH_17 25
#define ESP32S2_CPUINT_LEVELPERIPH_18 26
#define ESP32S2_CPUINT_LEVELPERIPH_19 27
#define ESP32S2_CPUINT_LEVELPERIPH_20 31

#define ESP32S2_CPUINT_NLEVELPERIPHS  21
#define ESP32S2_CPUINT_LEVELSET       0x8fbe333f

#define ESP32S2_CPUINT_EDGEPERIPH_0   10
#define ESP32S2_CPUINT_EDGEPERIPH_1   22
#define ESP32S2_CPUINT_EDGEPERIPH_2   28
#define ESP32S2_CPUINT_EDGEPERIPH_3   30

#define ESP32S2_CPUINT_NEDGEPERIPHS   4
#define ESP32S2_CPUINT_EDGESET        0x50400400

#define ESP32S2_CPUINT_NNMIPERIPHS    1
#define ESP32S2_CPUINT_NMISET         0x00004000

#define ESP32S2_CPUINT_MAC            0
#define ESP32S2_CPUINT_TIMER0         6
#define ESP32S2_CPUINT_SOFTWARE0      7
#define ESP32S2_CPUINT_PROFILING      11
#define ESP32S2_CPUINT_TIMER1         15
#define ESP32S2_CPUINT_TIMER2         16
#define ESP32S2_CPUINT_SOFTWARE1      29

#define ESP32S2_CPUINT_NINTERNAL      6

#define ESP32S2_NCPUINTS              32
#define ESP32S2_CPUINT_MAX            (ESP32S2_NCPUINTS - 1)
#define ESP32S2_CPUINT_PERIPHSET      0xdffe773f
#define ESP32S2_CPUINT_INTERNALSET    0x200188c0

/* Priority 1:   0-10, 12-13, 17-18    (15)
 * Priority 2:   19-21                 (3)
 * Priority 3:   11, 15, 22-23, 27, 29 (6)
 * Priority 4:   24-25, 28, 30         (4)
 * Priority 5:   16, 26, 31            (3)
 * Priority NMI: 14                    (1)
 */

#define ESP32S2_INTPRI1_MASK          0x000637ff
#define ESP32S2_INTPRI2_MASK          0x00380000
#define ESP32S2_INTPRI3_MASK          0x28c08800
#define ESP32S2_INTPRI4_MASK          0x53000000
#define ESP32S2_INTPRI5_MASK          0x84010000
#define ESP32S2_INTNMI_MASK           0x00004000

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

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

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_INCLUDE_ESP32S2_IRQ_H */
