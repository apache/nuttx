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
 * is still to be determined what will be done for the the ESP32.
 */

#define XTENSA_IRQ_TIMER0        0  /* INTERRUPT, bit 6 */
#define XTENSA_IRQ_TIMER1        1  /* INTERRUPT, bit 15 */
#define XTENSA_IRQ_TIMER2        2  /* INTERRUPT, bit 16 */

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

#define XTENSA_IRQ_SREG0         3
#define XTENSA_IRQ_MAC           3  /* INTR_STATUS_REG_0, bit 0 */
#define XTENSA_IRQ_MAC_NMI       4  /* INTR_STATUS_REG_0, bit 1 */
#define XTENSA_IRQ_BB            5  /* INTR_STATUS_REG_0, bit 2 */
#define XTENSA_IRQ_BB_MAC        6  /* INTR_STATUS_REG_0, bit 3 */
#define XTENSA_IRQ_BT_BB         7  /* INTR_STATUS_REG_0, bit 4 */
#define XTENSA_IRQ_BT_BB_NMI     8  /* INTR_STATUS_REG_0, bit 5 */
#define XTENSA_IRQ_RWBT_IRQ      9  /* INTR_STATUS_REG_0, bit 6 */
#define XTENSA_IRQ_RWBLE_IRQ     10 /* INTR_STATUS_REG_0, bit 7 */
#define XTENSA_IRQ_RWBT_NMI      11 /* INTR_STATUS_REG_0, bit 8 */
#define XTENSA_IRQ_RWBLE_NMI     12 /* INTR_STATUS_REG_0, bit 9 */

#define XTENSA_IRQ_SLC0          13 /* INTR_STATUS_REG_0, bit 10 */
#define XTENSA_IRQ_SLC1          14 /* INTR_STATUS_REG_0, bit 11 */
#define XTENSA_IRQ_UHCI0         15 /* INTR_STATUS_REG_0, bit 12 */
#define XTENSA_IRQ_UHCI1         16 /* INTR_STATUS_REG_0, bit 13 */
#define XTENSA_IRQ_TG_T0_LEVEL   17 /* INTR_STATUS_REG_0, bit 14 */
#define XTENSA_IRQ_TG_T1_LEVEL   18 /* INTR_STATUS_REG_0, bit 15 */
#define XTENSA_IRQ_TG_WDT_LEVEL  19 /* INTR_STATUS_REG_0, bit 16 */
#define XTENSA_IRQ_TG_LACT_LEVEL 20 /* INTR_STATUS_REG_0, bit 17 */
#define XTENSA_IRQ_TG1_T0_LEVEL  21 /* INTR_STATUS_REG_0, bit 18 */
#define XTENSA_IRQ_TG1_T1_LEVEL  22 /* INTR_STATUS_REG_0, bit 19 */

#define XTENSA_IRQ_TG1_WDT_LEVEL 23 /* INTR_STATUS_REG_0, bit 20 */
#define XTENSA_IRQ_G1_LACT_LEVEL 24 /* INTR_STATUS_REG_0, bit 21 */
#define XTENSA_IRQ_CPU_GPIO      25 /* INTR_STATUS_REG_0, bit 22 */
#define XTENSA_IRQ_CPU_NMI       26 /* INTR_STATUS_REG_0, bit 23 */
#define XTENSA_IRQ_CPU_CPU0      27 /* INTR_STATUS_REG_0, bit 24 */
#define XTENSA_IRQ_CPU_CPU1      28 /* INTR_STATUS_REG_0, bit 25 */
#define XTENSA_IRQ_CPU_CPU2      29 /* INTR_STATUS_REG_0, bit 26 */
#define XTENSA_IRQ_CPU_CPU3      30 /* INTR_STATUS_REG_0, bit 27 */
#define XTENSA_IRQ_SPI0          31 /* INTR_STATUS_REG_0, bit 28 */
#define XTENSA_IRQ_SPI1          32 /* INTR_STATUS_REG_0, bit 29 */

#define XTENSA_IRQ_SPI2          33 /* INTR_STATUS_REG_0, bit 30 */
#define XTENSA_IRQ_SPI3          34 /* INTR_STATUS_REG_0, bit 31 */

/* PRO_INTR_STATUS_REG_1 / APP_INTR_STATUS_REG_1 */

#define XTENSA_IRQ_SREG1         35
#define XTENSA_IRQ_I2S0          35 /* INTR_STATUS_REG_1, bit 0 */
#define XTENSA_IRQ_I2S1          36 /* INTR_STATUS_REG_1, bit 1 */
#define XTENSA_IRQ_UART          37 /* INTR_STATUS_REG_1, bit 2 */
#define XTENSA_IRQ_UART1         38 /* INTR_STATUS_REG_1, bit 3 */
#define XTENSA_IRQ_UART2         39 /* INTR_STATUS_REG_1, bit 4 */
#define XTENSA_IRQ_SDIO_HOST     40 /* INTR_STATUS_REG_1, bit 5 */
#define XTENSA_IRQ_EMAC          41 /* INTR_STATUS_REG_1, bit 6 */
#define XTENSA_IRQ_PWM0          42 /* INTR_STATUS_REG_1, bit 7 */
#define XTENSA_IRQ_PWM1          43 /* INTR_STATUS_REG_1, bit 8 */
#define XTENSA_IRQ_PWM2          44 /* INTR_STATUS_REG_1, bit 9 */

#define XTENSA_IRQ_PWM3          45 /* INTR_STATUS_REG_1, bit 10 */
#define XTENSA_IRQ_LEDC          46 /* INTR_STATUS_REG_1, bit 11 */
#define XTENSA_IRQ_EFUSE         47 /* INTR_STATUS_REG_1, bit 12 */
#define XTENSA_IRQ_CAN           48 /* INTR_STATUS_REG_1, bit 13 */
#define XTENSA_IRQ_RTC_CORE      49 /* INTR_STATUS_REG_1, bit 14 */
#define XTENSA_IRQ_RMT           50 /* INTR_STATUS_REG_1, bit 15 */
#define XTENSA_IRQ_PCNT          51 /* INTR_STATUS_REG_1, bit 16 */
#define XTENSA_IRQ_I2C_EXT0      52 /* INTR_STATUS_REG_1, bit 17 */
#define XTENSA_IRQ_I2C_EXT1      53 /* INTR_STATUS_REG_1, bit 18 */
#define XTENSA_IRQ_RSA           54 /* INTR_STATUS_REG_1, bit 19 */

#define XTENSA_IRQ_SPI1_DMA      55 /* INTR_STATUS_REG_1, bit 20 */
#define XTENSA_IRQ_SPI2_DMA      56 /* INTR_STATUS_REG_1, bit 21 */
#define XTENSA_IRQ_SPI3_DMA      57 /* INTR_STATUS_REG_1, bit 22 */
#define XTENSA_IRQ_WDG           58 /* INTR_STATUS_REG_1, bit 23 */
#define XTENSA_IRQ_TIMER1        59 /* INTR_STATUS_REG_1, bit 24 */
#define XTENSA_IRQ_TIMER2        60 /* INTR_STATUS_REG_1, bit 25 */
#define XTENSA_IRQ_TG_T0_EDGE    61 /* INTR_STATUS_REG_1, bit 26 */
#define XTENSA_IRQ_TG_T1_EDGE    62 /* INTR_STATUS_REG_1, bit 27 */
#define XTENSA_IRQ_TG_WDT_EDGE   63 /* INTR_STATUS_REG_1, bit 28 */
#define XTENSA_IRQ_TG_LACT_EDGE  64 /* INTR_STATUS_REG_1, bit 29 */

#define XTENSA_IRQ_TG1_T0_EDGE   65 /* INTR_STATUS_REG_1, bit 30 */
#define XTENSA_IRQ_TG1_T1_EDGE   66 /* INTR_STATUS_REG_1, bit 31 */

/* PRO_INTR_STATUS_REG_2 / APP_INTR_STATUS_REG_2 */

#define XTENSA_IRQ_SREG2         67
#define XTENSA_IRQ_TG1_WDT_EDGE  67 /* INTR_STATUS_REG_2, bit 0 */
#define XTENSA_IRQ_TG1_LACT_EDGE 68 /* INTR_STATUS_REG_2, bit 1 */
#define XTENSA_IRQ_MMU_IA        69 /* INTR_STATUS_REG_2, bit 2 */
#define XTENSA_IRQ_MPU_IA        70 /* INTR_STATUS_REG_2, bit 3 */
#define XTENSA_IRQ_CACHE_IA      71 /* INTR_STATUS_REG_2, bit 4 */

/* Total number of interrupts */

#define NR_IRQS                  72

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
