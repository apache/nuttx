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

#define ESP32S2_INT_PRIO_DEF        1

/* Interrupt Matrix
 *
 * The Interrupt Matrix embedded in the ESP32-S2 independently allocates
 * peripheral interrupt sources to the CPU peripheral interrupts, so as to
 * timely inform the CPU to process the interrupts once the interrupt signals
 * are generated.
 * Peripheral interrupt sources must be routed to CPU peripheral
 * interrupts via this interrupt matrix due to the following considerations:
 * - ESP32-S2 has 95 peripheral interrupt sources. To map them to 32 CPU
 *   interrupts, this matrix is needed.
 * - Through this matrix, one peripheral interrupt source can be mapped to
 *   multiple CPU interrupts according to application requirements.
 *
 * Features:
 * - Accept 95 peripheral interrupt sources as input.
 * - Generate 26 peripheral interrupts to the CPU output. Note that the
 *   remaining 6 CPU interrupts are internal interrupts.
 * - Support disabling CPU non-maskable interrupt (NMI) sources.
 * - Support querying current interrupt status of peripheral interrupt
 *   sources.
 */

/* RESERVED interrupts: 0, 1, 3, 4, 5, 6, 7, 8, 9 */

#define ESP32S2_PERIPH_PWR               2

/* RESERVED interrupts: 10, 11, 12, 14 */

#define ESP32S2_PERIPH_UHCI0             13
#define ESP32S2_PERIPH_TG_T0_LEVEL       15
#define ESP32S2_PERIPH_TG_T1_LEVEL       16
#define ESP32S2_PERIPH_TG_WDT_LEVEL      17
#define ESP32S2_PERIPH_TG_LACT_LEVEL     18
#define ESP32S2_PERIPH_TG1_T0_LEVEL      19

/* RESERVED interrupts: 25, 26 */

#define ESP32S2_PERIPH_TG1_T1_LEVEL      20
#define ESP32S2_PERIPH_TG1_WDT_LEVEL     21
#define ESP32S2_PERIPH_TG1_LACT_LEVEL    22
#define ESP32S2_PERIPH_GPIO_INT_PRO      23
#define ESP32S2_PERIPH_GPIO_INT_PRO_NMI  24
#define ESP32S2_PERIPH_DEDICATED_GPIO_IN 27
#define ESP32S2_PERIPH_INT_FROM_CPU0     28
#define ESP32S2_PERIPH_INT_FROM_CPU1     29

/* RESERVED interrupts: 39 */

#define ESP32S2_PERIPH_INT_FROM_CPU2     30
#define ESP32S2_PERIPH_INT_FROM_CPU3     31
#define ESP32S2_PERIPH_SPI1              32
#define ESP32S2_PERIPH_SPI2              33
#define ESP32S2_PERIPH_SPI3              34
#define ESP32S2_PERIPH_I2S0              35
#define ESP32S2_PERIPH_UART              37
#define ESP32S2_PERIPH_UART1             38

/* RESERVED interrupts: 40, 41, 42, 43, 44 */

#define ESP32S2_PERIPH_LEDC              45
#define ESP32S2_PERIPH_EFUSE             46
#define ESP32S2_PERIPH_CAN               47
#define ESP32S2_PERIPH_USB               48
#define ESP32S2_PERIPH_RTC_CORE          49

/* RESERVED interrupts: 59 */

#define ESP32S2_PERIPH_RMT               50
#define ESP32S2_PERIPH_PCNT              51
#define ESP32S2_PERIPH_I2C_EXT0          52
#define ESP32S2_PERIPH_I2C_EXT1          53
#define ESP32S2_PERIPH_RSA               54
#define ESP32S2_PERIPH_SHA               55
#define ESP32S2_PERIPH_AES               56
#define ESP32S2_PERIPH_SPI2_DMA          57
#define ESP32S2_PERIPH_SPI3_DMA          58

#define ESP32S2_PERIPH_TIMER             60
#define ESP32S2_PERIPH_TIMER_INT2        61
#define ESP32S2_PERIPH_TG_T0_EDGE        62
#define ESP32S2_PERIPH_TG_T1_EDGE        63
#define ESP32S2_PERIPH_TG_WDT_EDGE       64
#define ESP32S2_PERIPH_TG_LACT_EDGE      65
#define ESP32S2_PERIPH_TG1_T0_EDGE       66
#define ESP32S2_PERIPH_TG1_T1_EDGE       67
#define ESP32S2_PERIPH_TG1_WDT_EDGE      68
#define ESP32S2_PERIPH_TG1_LACT_EDGE     69

#define ESP32S2_PERIPH_CACHE_IA          70
#define ESP32S2_PERIPH_SYSTIMER_TARGET0  71
#define ESP32S2_PERIPH_SYSTIMER_TARGET1  72
#define ESP32S2_PERIPH_SYSTIMER_TARGET2  73
#define ESP32S2_PERIPH_ASSIST_DEBUG      74
#define ESP32S2_PERIPH_PMS_PRO_IRAM0_ILG 75
#define ESP32S2_PERIPH_PMS_PRO_DRAM0_ILG 76
#define ESP32S2_PERIPH_PMS_PRO_DPORT_ILG 77
#define ESP32S2_PERIPH_PMS_PRO_AHB_ILG   78
#define ESP32S2_PERIPH_PMS_PRO_CACHE_ILG 79

/* RESERVED interrupts: 85, 86 */

#define ESP32S2_PERIPH_PMS_DMA_APB_I_ILG 80
#define ESP32S2_PERIPH_PMS_DMA_RX_I_ILG  81
#define ESP32S2_PERIPH_PMS_DMA_TX_I_ILG  82
#define ESP32S2_PERIPH_SPI_MEM_REJECT    83
#define ESP32S2_PERIPH_DMA_COPY          84
#define ESP32S2_PERIPH_DCACHE_PRELOAD    87
#define ESP32S2_PERIPH_ICACHE_PRELOAD    88
#define ESP32S2_PERIPH_APB_ADC           89

#define ESP32S2_PERIPH_CRYPTO_DMA        90
#define ESP32S2_PERIPH_CPU_PERI_ERR      91
#define ESP32S2_PERIPH_APB_PERI_ERR      92
#define ESP32S2_PERIPH_DCACHE_SYNC       93
#define ESP32S2_PERIPH_ICACHE_SYNC       94

/* Total number of peripherals */

#define ESP32S2_NPERIPHERALS             95

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
#define XTENSA_IRQ_FIRSTPERIPH      5  /* First peripheral IRQ number */

/* IRQ numbers for peripheral interrupts coming through the Interrupt
 * Matrix.
 */

#define ESP32S2_IRQ2PERIPH(irq)       ((irq) - XTENSA_IRQ_FIRSTPERIPH)
#define ESP32S2_PERIPH2IRQ(id)        ((id) + XTENSA_IRQ_FIRSTPERIPH)

#define ESP32S2_IRQ_PWR               (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_PWR)

#define ESP32S2_IRQ_UHCI0             (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_UHCI0)
#define ESP32S2_IRQ_TG_T0_LEVEL       (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_TG_T0_LEVEL)
#define ESP32S2_IRQ_TG_T1_LEVEL       (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_TG_T1_LEVEL)
#define ESP32S2_IRQ_TG_WDT_LEVEL      (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_TG_WDT_LEVEL)
#define ESP32S2_IRQ_TG_LACT_LEVEL     (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_TG_LACT_LEVEL)
#define ESP32S2_IRQ_TG1_T0_LEVEL      (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_TG1_T0_LEVEL)

#define ESP32S2_IRQ_TG1_T1_LEVEL      (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_TG1_T1_LEVEL)
#define ESP32S2_IRQ_TG1_WDT_LEVEL     (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_TG1_WDT_LEVEL)
#define ESP32S2_IRQ_TG1_LACT_LEVEL    (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_TG1_LACT_LEVEL)
#define ESP32S2_IRQ_GPIO_INT_PRO      (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_GPIO_INT_PRO)
#define ESP32S2_IRQ_GPIO_INT_PRO_NMI  (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_GPIO_INT_PRO_NMI)
#define ESP32S2_IRQ_DEDICATED_GPIO_IN (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_DEDICATED_GPIO_IN)
#define ESP32S2_IRQ_INT_FROM_CPU0     (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_INT_FROM_CPU0)
#define ESP32S2_IRQ_INT_FROM_CPU1     (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_INT_FROM_CPU1)

#define ESP32S2_IRQ_INT_FROM_CPU2     (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_INT_FROM_CPU2)
#define ESP32S2_IRQ_INT_FROM_CPU3     (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_INT_FROM_CPU3)
#define ESP32S2_IRQ_SPI1              (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_SPI1)
#define ESP32S2_IRQ_SPI2              (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_SPI2)
#define ESP32S2_IRQ_SPI3              (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_SPI3)
#define ESP32S2_IRQ_I2S0              (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_I2S0)
#define ESP32S2_IRQ_I2S1              (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_I2S1)
#define ESP32S2_IRQ_UART              (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_UART)
#define ESP32S2_IRQ_UART1             (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_UART1)

#define ESP32S2_IRQ_LEDC              (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_LEDC)
#define ESP32S2_IRQ_EFUSE             (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_EFUSE)
#define ESP32S2_IRQ_CAN               (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_CAN)
#define ESP32S2_IRQ_USB               (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_USB)
#define ESP32S2_IRQ_RTC_CORE          (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_RTC_CORE)

#define ESP32S2_IRQ_RMT               (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_RMT)
#define ESP32S2_IRQ_PCNT              (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_PCNT)
#define ESP32S2_IRQ_I2C_EXT0          (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_I2C_EXT0)
#define ESP32S2_IRQ_I2C_EXT1          (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_I2C_EXT1)
#define ESP32S2_IRQ_RSA               (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_RSA)
#define ESP32S2_IRQ_SHA               (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_SHA)
#define ESP32S2_IRQ_AES               (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_AES)
#define ESP32S2_IRQ_SPI2_DMA          (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_SPI2_DMA)
#define ESP32S2_IRQ_SPI3_DMA          (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_SPI3_DMA)

#define ESP32S2_IRQ_TIMER             (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_TIMER)
#define ESP32S2_IRQ_TIMER_INT2        (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_TIMER_INT2)
#define ESP32S2_IRQ_TG_T0_EDGE        (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_TG_T0_EDGE)
#define ESP32S2_IRQ_TG_T1_EDGE        (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_TG_T1_EDGE)
#define ESP32S2_IRQ_TG_WDT_EDGE       (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_TG_WDT_EDGE)
#define ESP32S2_IRQ_TG_LACT_EDGE      (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_TG_LACT_EDGE)
#define ESP32S2_IRQ_TG1_T0_EDGE       (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_TG1_T0_EDGE)
#define ESP32S2_IRQ_TG1_T1_EDGE       (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_TG1_T1_EDGE)
#define ESP32S2_IRQ_TG1_WDT_EDGE      (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_TG1_WDT_EDGE)
#define ESP32S2_IRQ_TG1_LACT_EDGE     (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_TG1_LACT_EDGE)

#define ESP32S2_IRQ_CACHE_IA          (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_CACHE_IA)
#define ESP32S2_IRQ_SYSTIMER_TARGET0  (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_SYSTIMER_TARGET0)
#define ESP32S2_IRQ_SYSTIMER_TARGET1  (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_SYSTIMER_TARGET1)
#define ESP32S2_IRQ_SYSTIMER_TARGET2  (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_SYSTIMER_TARGET2)
#define ESP32S2_IRQ_ASSIST_DEBUG      (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_ASSIST_DEBUG)
#define ESP32S2_IRQ_PMS_PRO_IRAM0_ILG (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_PMS_PRO_IRAM0_ILG)
#define ESP32S2_IRQ_PMS_PRO_DRAM0_ILG (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_PMS_PRO_DRAM0_ILG)
#define ESP32S2_IRQ_PMS_PRO_DPORT_ILG (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_PMS_PRO_DPORT_ILG)
#define ESP32S2_IRQ_PMS_PRO_AHB_ILG   (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_PMS_PRO_AHB_ILG)
#define ESP32S2_IRQ_PMS_PRO_CACHE_ILG (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_PMS_PRO_CACHE_ILG)

#define ESP32S2_IRQ_PMS_DMA_APB_I_ILG (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_PMS_DMA_APB_I_ILG)
#define ESP32S2_IRQ_PMS_DMA_RX_I_ILG  (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_PMS_DMA_RX_I_ILG)
#define ESP32S2_IRQ_PMS_DMA_TX_I_ILG  (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_PMS_DMA_TX_I_ILG)
#define ESP32S2_IRQ_SPI_MEM_REJECT    (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_SPI_MEM_REJECT)
#define ESP32S2_IRQ_DMA_COPY          (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_DMA_COPY)
#define ESP32S2_IRQ_DCACHE_PRELOAD    (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_DCACHE_PRELOAD)
#define ESP32S2_IRQ_ICACHE_PRELOAD    (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_ICACHE_PRELOAD)
#define ESP32S2_IRQ_APB_ADC           (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_APB_ADC)

#define ESP32S2_IRQ_CRYPTO_DMA        (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_CRYPTO_DMA)
#define ESP32S2_IRQ_CPU_PERI_ERR      (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_CPU_PERI_ERR)
#define ESP32S2_IRQ_APB_PERI_ERE      (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_APB_PERI_ERR)
#define ESP32S2_IRQ_DCACHE_SYNC       (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_DCACHE_SYNC)
#define ESP32S2_IRQ_ICACHE_SYNC       (XTENSA_IRQ_FIRSTPERIPH + ESP32S2_PERIPH_ICACHE_SYNC)

#define ESP32S2_NIRQ_PERIPH           ESP32S2_NPERIPHERALS

/* Second level GPIO interrupts. GPIO interrupts are decoded and dispatched
 * as a second level of decoding: The first level dispatches to the GPIO
 * interrupt handler. The second to the decoded GPIO interrupt handler.
 */

#ifdef CONFIG_ESP32S2_GPIO_IRQ
#  define ESP32S2_NIRQ_GPIO           40
#  define ESP32S2_FIRST_GPIOIRQ       (XTENSA_NIRQ_INTERNAL + ESP32S2_NIRQ_PERIPH)
#  define ESP32S2_LAST_GPIOIRQ        (ESP32S2_FIRST_GPIOIRQ + ESP32S2_NIRQ_GPIO - 1)
#  define ESP32S2_PIN2IRQ(p)          ((p) + ESP32S2_FIRST_GPIOIRQ)
#  define ESP32S2_IRQ2PIN(i)          ((i) - ESP32S2_FIRST_GPIOIRQ)
#else
#  define ESP32S2_NIRQ_GPIO           0
#endif

/* Total number of interrupts */

#define NR_IRQS                     (XTENSA_NIRQ_INTERNAL + ESP32S2_NIRQ_PERIPH + ESP32S2_NIRQ_GPIO)

/* Xtensa CPU Interrupts.
 *
 * The CPU has 32 interrupts, of which 26 can be mapped to peripheral
 * interrupts:
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
 * INTERRUPT_PRO_*_MAP_REG. There are a pair of these registers for each
 * peripheral source. Multiple peripheral interrupt sources can be mapped to
 * the same CPU interrupt.
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
 * A peripheral interrupt can be disabled.
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
