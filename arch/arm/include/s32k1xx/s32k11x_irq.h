/************************************************************************************
 * arch/arm/include/s32k1xx/s32k11x_irq.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

/* This file should never be included directed but, rather, only indirectly through
 * nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_S32K1XX_S32K11XX_IRQ_H
#define __ARCH_ARM_INCLUDE_S32K1XX_S32K11XX_IRQ_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* IRQ numbers.  The IRQ number corresponds vector number and hence map directly to
 * bits in the NVIC.  This does, however, waste several words of memory in the IRQ
 * to handle mapping tables.
 */

/* Processor Exceptions (vectors 0-15) */

#define S32K1XX_IRQ_RESERVED       (0) /* Reserved vector (only used with CONFIG_DEBUG_FEATURES) */
                                       /* Vector  0: Reset stack pointer value */
                                       /* Vector  1: Reset (not handler as an IRQ) */
#define S32K1XX_IRQ_NMI            (2) /* Vector  2: Non-Maskable Interrupt (NMI) */
#define S32K1XX_IRQ_HARDFAULT      (3) /* Vector  3: Hard fault */
                                       /* Vector  4-10: Reserved */
#define S32K1XX_IRQ_SVCALL        (11) /* Vector 11: SVC call */
                                       /* Vector 12-13: Reserved */
#define S32K1XX_IRQ_PENDSV        (14) /* Vector 14: Pendable system service request */
#define S32K1XX_IRQ_SYSTICK       (15) /* Vector 15: System tick */

/* External interrupts (vectors >= 16).  These definitions are chip-specific */

#define S32K1XX_IRQ_INTERRUPT     (16)

#define S32K1XX_IRQ_DMACH0        (16) /* DMA channel 0 transfer complete */
#define S32K1XX_IRQ_DMACH1        (17) /* DMA channel 1 transfer complete */
#define S32K1XX_IRQ_DMACH2        (18) /* DMA channel 2 transfer complete */
#define S32K1XX_IRQ_DMACH3        (19) /* DMA channel 3 transfer complete */
#define S32K1XX_IRQ_DMACH_ERR     (20) /* DMA error interrupt channels 0-15 */
#define S32K1XX_IRQ_ERM           (21) /* ERM single/double bit error */
#define S32K1XX_IRQ_RTC_ALARM     (22) /* RTC alarm interrupt */
#define S32K1XX_IRQ_RTC_SEC       (23) /* RTC seconds interrupt */
#define S32K1XX_IRQ_LPTIMER       (24) /* LPTIMER interrupt request */
#define S32K1XX_IRQ_PORT          (25) /* PORTA-E Interrupt */
#define S32K1XX_IRQ_CAN0          (26) /* CAN0 OR'ed Interrupt */
#define S32K1XX_IRQ_CAN0_0_31     (27) /* CAN0 OR'ed Message buffer (0-31) */
#define S32K1XX_IRQ_FTM0_CH0_7    (28) /* FTM0 Channel 0..7 Interrupt */
#define S32K1XX_IRQ_FTM0_FAULT    (29) /* FTM0 Fault Interrupt */
#define S32K1XX_IRQ_FTM0_OVERFLOW (30) /* FTM0 Counter Overflow/Reload Interrupt */
#define S32K1XX_IRQ_FTM1_CH0_7    (31) /* FTM1 Channel 0..7 Interrupt */
#define S32K1XX_IRQ_FTM1_FAULT    (32) /* FTM1 Fault Interrupt */
#define S32K1XX_IRQ_FTM1_OVERFLOW (33) /* FTM1 Counter Overflow/Reload Interrupt */
#define S32K1XX_IRQ_FTFC          (34) /* FTFC Command Complete, Collision, Double bit */
#define S32K1XX_IRQ_PDB           (35) /* PDB Interrupt */
#define S32K1XX_IRQ_LPIT          (36) /* LPIT Interrupt */
#define S32K1XX_IRQ_PMC           (37) /* PMC Interrupt */
#define S32K1XX_IRQ_WDOG          (38) /* WDOG interrupt request out before wdg reset out */
#define S32K1XX_IRQ_RCM           (39) /* RCM Asynchronous Interrupt */
#define S32K1XX_IRQ_LPI2C0M       (40) /* LPI2C Master/Slave Interrupt */
#define S32K1XX_IRQ_LPI2C0S       (40) /* LPI2C Master/Slave Interrupt */
#define S32K1XX_IRQ_FLEXIO        (41) /* FlexIO Interrupt */
#define S32K1XX_IRQ_LPSPI0        (42) /* LPSPI0 Interrupt */
#define S32K1XX_IRQ_LPSPI1        (43) /* LPSPI1 Interrupt */
#define S32K1XX_IRQ_ADC0          (44) /* ADC0 Interrupt */
#define S32K1XX_IRQ_CMP0          (45) /* CMP0 Interrupt */
#define S32K1XX_IRQ_LPUART1       (46) /* LPUART1 Interrupt */
#define S32K1XX_IRQ_LPUART0       (47) /* LPUART0 Interrupt */

#define S32K1XX_IRQ_NIRQS         (48)
#define S32K1XX_IRQ_NEXTINT       (S32K1XX_IRQ_NIRQS - S32K1XX_IRQ_INTERRUPT)

/* Total number of IRQ numbers */

#define NR_IRQS                    S32K1XX_IRQ_NIRQS

#endif /* __ARCH_ARM_INCLUDE_S32K1XX_S32K11XX_IRQ_H */
