/****************************************************************************
 * arch/xtensa/include/esp32/irq.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ****************************************************************************/

/* This file should never be included directed but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_XTENSA_INCLUDE_ESP32_IRQ_H
#define __ARCH_XTENSA_INCLUDE_ESP32_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/esp32/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Interrupt Matrix
 *
 * The Interrupt Matrix embedded in the ESP32 independently allocates
 * peripheral interrupt sources to the two CPUs’ peripheral interrupts. This
 * configuration is highly flexible in order to meet many different needs.
 *
 * Features
 * - Accepts 71 peripheral interrupt sources as input.
 * - Generates 26 peripheral interrupt sources per CPU as output (52 total).
 * - CPU NMI Interrupt Mask.
 * - Queries current interrupt status of peripheral interrupt sources.
 *
 * Peripheral Interrupt Source
 *
 * ESP32 has 71 peripheral interrupt sources in total. 67 of 71 ESP32
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

/* PRO_INTR_STATUS_REG_0 / APP_INTR_STATUS_REG_0 */

#define ESP32_PERIPH_MAC            0  /* INTR_STATUS_REG_0, bit 0 */
#define ESP32_PERIPH_MAC_NMI        1  /* INTR_STATUS_REG_0, bit 1 */
#define ESP32_PERIPH_BB             2  /* INTR_STATUS_REG_0, bit 2 */
#define ESP32_PERIPH_BB_MAC         3  /* INTR_STATUS_REG_0, bit 3 */
#define ESP32_PERIPH_BT_BB          4  /* INTR_STATUS_REG_0, bit 4 */
#define ESP32_PERIPH_BT_BB_NMI      5  /* INTR_STATUS_REG_0, bit 5 */
#define ESP32_PERIPH_RWBT_IRQ       6  /* INTR_STATUS_REG_0, bit 6 */
#define ESP32_PERIPH_RWBLE_IRQ      7  /* INTR_STATUS_REG_0, bit 7 */
#define ESP32_PERIPH_RWBT_NMI       8  /* INTR_STATUS_REG_0, bit 8 */
#define ESP32_PERIPH_RWBLE_NMI      9  /* INTR_STATUS_REG_0, bit 9 */

#define ESP32_PERIPH_SLC0           10 /* INTR_STATUS_REG_0, bit 10 */
#define ESP32_PERIPH_SLC1           11 /* INTR_STATUS_REG_0, bit 11 */
#define ESP32_PERIPH_UHCI0          12 /* INTR_STATUS_REG_0, bit 12 */
#define ESP32_PERIPH_UHCI1          13 /* INTR_STATUS_REG_0, bit 13 */
#define ESP32_PERIPH_TG_T0_LEVEL    14 /* INTR_STATUS_REG_0, bit 14 */
#define ESP32_PERIPH_TG_T1_LEVEL    15 /* INTR_STATUS_REG_0, bit 15 */
#define ESP32_PERIPH_TG_WDT_LEVEL   16 /* INTR_STATUS_REG_0, bit 16 */
#define ESP32_PERIPH_TG_LACT_LEVEL  17 /* INTR_STATUS_REG_0, bit 17 */
#define ESP32_PERIPH_TG1_T0_LEVEL   18 /* INTR_STATUS_REG_0, bit 18 */
#define ESP32_PERIPH_TG1_T1_LEVEL   19 /* INTR_STATUS_REG_0, bit 19 */

#define ESP32_PERIPH_TG1_WDT_LEVEL  20 /* INTR_STATUS_REG_0, bit 20 */
#define ESP32_PERIPH_G1_LACT_LEVEL  21 /* INTR_STATUS_REG_0, bit 21 */
#define ESP32_PERIPH_CPU_GPIO       22 /* INTR_STATUS_REG_0, bit 22 */
#define ESP32_PERIPH_CPU_NMI        23 /* INTR_STATUS_REG_0, bit 23 */
#define ESP32_PERIPH_CPU_CPU0       24 /* INTR_STATUS_REG_0, bit 24 */
#define ESP32_PERIPH_CPU_CPU1       25 /* INTR_STATUS_REG_0, bit 25 */
#define ESP32_PERIPH_CPU_CPU2       26 /* INTR_STATUS_REG_0, bit 26 */
#define ESP32_PERIPH_CPU_CPU3       27 /* INTR_STATUS_REG_0, bit 27 */
#define ESP32_PERIPH_SPI0           28 /* INTR_STATUS_REG_0, bit 28 */
#define ESP32_PERIPH_SPI1           29 /* INTR_STATUS_REG_0, bit 29 */

#define ESP32_PERIPH_SPI2           30 /* INTR_STATUS_REG_0, bit 30 */
#define ESP32_PERIPH_SPI3           31 /* INTR_STATUS_REG_0, bit 31 */

/* PRO_INTR_STATUS_REG_1 / APP_INTR_STATUS_REG_1 */

#define ESP32_PERIPH_I2S0           32 /* INTR_STATUS_REG_1, bit 0 */
#define ESP32_PERIPH_I2S1           33 /* INTR_STATUS_REG_1, bit 1 */
#define ESP32_PERIPH_UART           34 /* INTR_STATUS_REG_1, bit 2 */
#define ESP32_PERIPH_UART1          35 /* INTR_STATUS_REG_1, bit 3 */
#define ESP32_PERIPH_UART2          36 /* INTR_STATUS_REG_1, bit 4 */
#define ESP32_PERIPH_SDIO_HOST      37 /* INTR_STATUS_REG_1, bit 5 */
#define ESP32_PERIPH_EMAC           38 /* INTR_STATUS_REG_1, bit 6 */
#define ESP32_PERIPH_PWM0           39 /* INTR_STATUS_REG_1, bit 7 */
#define ESP32_PERIPH_PWM1           40 /* INTR_STATUS_REG_1, bit 8 */
#define ESP32_PERIPH_PWM2           41 /* INTR_STATUS_REG_1, bit 9 */

#define ESP32_PERIPH_PWM3           42 /* INTR_STATUS_REG_1, bit 10 */
#define ESP32_PERIPH_LEDC           43 /* INTR_STATUS_REG_1, bit 11 */
#define ESP32_PERIPH_EFUSE          44 /* INTR_STATUS_REG_1, bit 12 */
#define ESP32_PERIPH_CAN            45 /* INTR_STATUS_REG_1, bit 13 */
#define ESP32_PERIPH_RTC_CORE       46 /* INTR_STATUS_REG_1, bit 14 */
#define ESP32_PERIPH_RMT            47 /* INTR_STATUS_REG_1, bit 15 */
#define ESP32_PERIPH_PCNT           48 /* INTR_STATUS_REG_1, bit 16 */
#define ESP32_PERIPH_I2C_EXT0       49 /* INTR_STATUS_REG_1, bit 17 */
#define ESP32_PERIPH_I2C_EXT1       50 /* INTR_STATUS_REG_1, bit 18 */
#define ESP32_PERIPH_RSA            51 /* INTR_STATUS_REG_1, bit 19 */

#define ESP32_PERIPH_SPI1_DMA       52 /* INTR_STATUS_REG_1, bit 20 */
#define ESP32_PERIPH_SPI2_DMA       53 /* INTR_STATUS_REG_1, bit 21 */
#define ESP32_PERIPH_SPI3_DMA       54 /* INTR_STATUS_REG_1, bit 22 */
#define ESP32_PERIPH_WDG            55 /* INTR_STATUS_REG_1, bit 23 */
#define ESP32_PERIPH_TIMER1         56 /* INTR_STATUS_REG_1, bit 24 */
#define ESP32_PERIPH_TIMER2         57 /* INTR_STATUS_REG_1, bit 25 */
#define ESP32_PERIPH_TG_T0_EDGE     58 /* INTR_STATUS_REG_1, bit 26 */
#define ESP32_PERIPH_TG_T1_EDGE     59 /* INTR_STATUS_REG_1, bit 27 */
#define ESP32_PERIPH_TG_WDT_EDGE    60 /* INTR_STATUS_REG_1, bit 28 */
#define ESP32_PERIPH_TG_LACT_EDGE   61 /* INTR_STATUS_REG_1, bit 29 */

#define ESP32_PERIPH_TG1_T0_EDGE    62 /* INTR_STATUS_REG_1, bit 30 */
#define ESP32_PERIPH_TG1_T1_EDGE    63 /* INTR_STATUS_REG_1, bit 31 */

/* PRO_INTR_STATUS_REG_2 / APP_INTR_STATUS_REG_2 */

#define ESP32_PERIPH_TG1_WDT_EDGE   64 /* INTR_STATUS_REG_2, bit 0 */
#define ESP32_PERIPH_TG1_LACT_EDGE  65 /* INTR_STATUS_REG_2, bit 1 */
#define ESP32_PERIPH_MMU_IA         66 /* INTR_STATUS_REG_2, bit 2 */
#define ESP32_PERIPH_MPU_IA         67 /* INTR_STATUS_REG_2, bit 3 */
#define ESP32_PERIPH_CACHE_IA       68 /* INTR_STATUS_REG_2, bit 4 */

/* Total number of peripherals */

#define ESP32_NPERIPHERALS          69

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
 * tie into the normal interrupt handling via special IRQ numbers.  I
 * is still to be determined what will be done for the ESP32.
 */

/* IRQ numbers for internal interrupts that are dispatched like peripheral
 * interrupts
 */

#define XTENSA_IRQ_TIMER0           0  /* INTERRUPT, bit 6 */
#define XTENSA_IRQ_TIMER1           1  /* INTERRUPT, bit 15 */
#define XTENSA_IRQ_TIMER2           2  /* INTERRUPT, bit 16 */
#define XTENSA_IRQ_SYSCALL          3  /* User interrupt w/EXCCAUSE=syscall */

#define XTENSA_NIRQ_INTERNAL        4  /* Number of dispatch internal interrupts */
#define XTENSA_IRQ_FIRSTPERIPH      4  /* First peripheral IRQ number */

/* IRQ numbers for peripheral interrupts coming throught the Interrupt
 * Matrix.
 */

#define ESP32_IRQ2PERIPH(irq)       ((irq)-XTENSA_IRQ_FIRSTPERIPH)

/* PRO_INTR_STATUS_REG_0 / APP_INTR_STATUS_REG_0 */

#define ESP32_IRQ_MAC               (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_MAC)
#define ESP32_IRQ_MAC_NMI           (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_MAC_NMI)
#define ESP32_IRQ_BB                (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_BB)
#define ESP32_IRQ_BB_MAC            (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_BB_MAC)
#define ESP32_IRQ_BT_BB             (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_BT_BB)
#define ESP32_IRQ_BT_BB_NMI         (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_BT_BB_NMI)
#define ESP32_IRQ_RWBT_IRQ          (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_RWBT_IRQ)
#define ESP32_IRQ_RWBLE_IRQ         (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_RWBLE_IRQ)
#define ESP32_IRQ_RWBT_NMI          (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_RWBT_NMI)
#define ESP32_IRQ_RWBLE_NMI         (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_RWBLE_NMI)
#define ESP32_IRQ_SLC0              (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_SLC0)
#define ESP32_IRQ_SLC1              (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_SLC1)
#define ESP32_IRQ_UHCI0             (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_UHCI0)
#define ESP32_IRQ_UHCI1             (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_UHCI1)
#define ESP32_IRQ_TG_T0_LEVEL       (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_TG_T0_LEVEL)
#define ESP32_IRQ_TG_T1_LEVEL       (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_TG_T1_LEVEL)
#define ESP32_IRQ_TG_WDT_LEVEL      (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_TG_WDT_LEVEL)
#define ESP32_IRQ_TG_LACT_LEVEL     (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_TG_LACT_LEVEL)
#define ESP32_IRQ_TG1_T0_LEVEL      (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_TG1_T0_LEVEL)
#define ESP32_IRQ_TG1_T1_LEVEL      (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_TG1_T1_LEVEL)
#define ESP32_IRQ_TG1_WDT_LEVEL     (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_TG1_WDT_LEVEL)
#define ESP32_IRQ_G1_LACT_LEVEL     (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_G1_LACT_LEVEL)
#define ESP32_IRQ_CPU_GPIO          (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_CPU_GPIO)
#define ESP32_IRQ_CPU_NMI           (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_CPU_NMI)
#define ESP32_IRQ_CPU_CPU0          (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_CPU_CPU0)
#define ESP32_IRQ_CPU_CPU1          (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_CPU_CPU1)
#define ESP32_IRQ_CPU_CPU2          (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_CPU_CPU2)
#define ESP32_IRQ_CPU_CPU3          (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_CPU_CPU3)
#define ESP32_IRQ_SPI0              (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_SPI0)
#define ESP32_IRQ_SPI1              (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_SPI1
#define ESP32_IRQ_SPI2              (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_SPI2)
#define ESP32_IRQ_SPI3              (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_SPI3

#define ESP32_IRQ_SREG0             ESP32_IRQ_MAC
#define ESP32_NIRQS_SREG0           32

/* PRO_INTR_STATUS_REG_1 / APP_INTR_STATUS_REG_1 */

#define ESP32_IRQ_I2S0              (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_I2S0)
#define ESP32_IRQ_I2S1              (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_I2S1)
#define ESP32_IRQ_UART              (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_UART)
#define ESP32_IRQ_UART1             (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_UART1)
#define ESP32_IRQ_UART2             (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_UART2)
#define ESP32_IRQ_SDIO_HOST         (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_SDIO_HOST)
#define ESP32_IRQ_EMAC              (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_EMAC)
#define ESP32_IRQ_PWM0              (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_PWM0)
#define ESP32_IRQ_PWM1              (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_PWM1)
#define ESP32_IRQ_PWM2              (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_PWM2)
#define ESP32_IRQ_PWM3              (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_PWM3)
#define ESP32_IRQ_LEDC              (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_LEDC)
#define ESP32_IRQ_EFUSE             (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_EFUSE)
#define ESP32_IRQ_CAN               (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_CAN)
#define ESP32_IRQ_RTC_CORE          (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_RTC_CORE)
#define ESP32_IRQ_RMT               (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_RMT)
#define ESP32_IRQ_PCNT              (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_PCNT)
#define ESP32_IRQ_I2C_EXT0          (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_I2C_EXT0)
#define ESP32_IRQ_I2C_EXT1          (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_I2C_EXT1)
#define ESP32_IRQ_RSA               (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_RSA)
#define ESP32_IRQ_SPI1_DMA          (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_SPI1_DMA)
#define ESP32_IRQ_SPI2_DMA          (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_SPI2_DMA)
#define ESP32_IRQ_SPI3_DMA          (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_SPI3_DMA)
#define ESP32_IRQ_WDG               (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_WDG)
#define ESP32_IRQ_TIMER1            (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_TIMER1)
#define ESP32_IRQ_TIMER2            (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_TIMER2)
#define ESP32_IRQ_TG_T0_EDGE        (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_TG_T0_EDGE)
#define ESP32_IRQ_TG_T1_EDGE        (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_TG_T1_EDGE)
#define ESP32_IRQ_TG_WDT_EDGE       (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_TG_WDT_EDGE)
#define ESP32_IRQ_TG_LACT_EDGE      (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_TG_LACT_EDGE)
#define ESP32_IRQ_TG1_T0_EDGE       (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_TG1_T0_EDGE)
#define ESP32_IRQ_TG1_T1_EDGE       (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_TG1_T1_EDGE)

#define ESP32_IRQ_SREG1             ESP32_IRQ_I2S0
#define ESP32_NIRQS_SREG1           32

/* PRO_INTR_STATUS_REG_2 / APP_INTR_STATUS_REG_2 */

#define ESP32_IRQ_TG1_WDT_EDGE      (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_TG1_WDT_EDGE)
#define ESP32_IRQ_TG1_LACT_EDGE     (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_TG1_LACT_EDGE)
#define ESP32_IRQ_MMU_IA            (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_MMU_IA)
#define ESP32_IRQ_MPU_IA            (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_MPU_IA)
#define ESP32_IRQ_CACHE_IA          (XTENSA_IRQ_FIRSTPERIPH+ESP32_PERIPH_CACHE_IA)

#define ESP32_IRQ_SREG2             ESP32_IRQ_TG1_WDT_EDGE
#define ESP32_NIRQS_SREG2           5

#define ESP32_NIRQ_PERIPH           ESP32_NPERIPHERALS

/* Second level GPIO interrupts.  GPIO interrupts are decoded and dispatched as
 * a second level of decoding:  The first level dispatches to the GPIO interrupt
 * handler.  The second to the decoded GPIO interrupt handler.
 */

#ifdef CONFIG_ESP32_GPIO_IRQ
#  define ESP32_NIRQ_GPIO           40
#  define ESP32_FIRST_GPIOIRQ       (XTENSA_NIRQ_INTERNAL+ESP32_NIRQ_PERIPH)
#  define ESP32_LAST_GPIOIRQ        (ESP32_FIRST_GPIOIRQ+ESP32_NIRQ_GPIO-1)
#  define ESP32_PIN2IRQ(p)          ((p) + ESP32_FIRST_GPIOIRQ)
#  define ESP32_IRQ2PIN(i)          ((i) - ESP32_FIRST_GPIOIRQ)
#else
#  define ESP32_NIRQ_GPIO           0
#endif

/* Total number of interrupts */

#define NR_IRQS                     (XTENSA_NIRQ_INTERNAL+ESP32_NIRQ_PERIPH+ESP32_NIRQ_GPIO)

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
 * mapped to the same.
 *
 * The remaining, five, internal CPU interrupts are:
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

#define ESP32_CPUINT_LEVELPERIPH_0  0
#define ESP32_CPUINT_LEVELPERIPH_1  1
#define ESP32_CPUINT_LEVELPERIPH_2  2
#define ESP32_CPUINT_LEVELPERIPH_3  3
#define ESP32_CPUINT_LEVELPERIPH_4  4
#define ESP32_CPUINT_LEVELPERIPH_5  5
#define ESP32_CPUINT_LEVELPERIPH_6  8
#define ESP32_CPUINT_LEVELPERIPH_7  9
#define ESP32_CPUINT_LEVELPERIPH_8  12
#define ESP32_CPUINT_LEVELPERIPH_9  13
#define ESP32_CPUINT_LEVELPERIPH_10 17
#define ESP32_CPUINT_LEVELPERIPH_11 18
#define ESP32_CPUINT_LEVELPERIPH_12 19
#define ESP32_CPUINT_LEVELPERIPH_13 20
#define ESP32_CPUINT_LEVELPERIPH_14 21
#define ESP32_CPUINT_LEVELPERIPH_15 23
#define ESP32_CPUINT_LEVELPERIPH_16 24
#define ESP32_CPUINT_LEVELPERIPH_17 25
#define ESP32_CPUINT_LEVELPERIPH_18 26
#define ESP32_CPUINT_LEVELPERIPH_19 27
#define ESP32_CPUINT_LEVELPERIPH_20 31

#define ESP32_CPUINT_NLEVELPERIPHS  21
#define EPS32_CPUINT_LEVELSET       0x8fbe333f

#define ESP32_CPUINT_EDGEPERIPH_0   10
#define ESP32_CPUINT_EDGEPERIPH_1   22
#define ESP32_CPUINT_EDGEPERIPH_2   28
#define ESP32_CPUINT_EDGEPERIPH_3   30

#define ESP32_CPUINT_NEDGEPERIPHS   4
#define EPS32_CPUINT_EDGESET        0x50400400

#define ESP32_CPUINT_NNMIPERIPHS    1
#define EPS32_CPUINT_NMISET         0x00004000

#define ESP32_CPUINT_TIMER0         6
#define ESP32_CPUINT_SOFTWARE0      7
#define ESP32_CPUINT_PROFILING      11
#define ESP32_CPUINT_TIMER1         15
#define ESP32_CPUINT_TIMER2         16
#define ESP32_CPUINT_SOFTWARE1      29

#define ESP32_CPUINT_NINTERNAL      6

#define ESP32_NCPUINTS              32
#define ESP32_CPUINT_MAX            (ESP32_NCPUINTS - 1)
#define EPS32_CPUINT_PERIPHSET      0xdffe773f
#define EPS32_CPUINT_INTERNALSET    0x200188c0

/* Priority 1:   0-10, 12-13, 17-18    (15)
 * Priority 2:   19-21                 (3)
 * Priority 3:   11, 15, 22-23, 27, 29 (6)
 * Priority 4:   24-25, 28, 30         (4)
 * Priority 5:   16, 26, 31            (3)
 * Priority NMI: 14                    (1)
 */

#define ESP32_INTPRI1_MASK          0x000637ff
#define ESP32_INTPRI2_MASK          0x00380000
#define ESP32_INTPRI3_MASK          0x28c08800
#define ESP32_INTPRI4_MASK          0x53000000
#define ESP32_INTPRI5_MASK          0x84010000
#define ESP32_INTNMI_MASK           0x00004000

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
#endif /* __ARCH_XTENSA_INCLUDE_ESP32_IRQ_H */
