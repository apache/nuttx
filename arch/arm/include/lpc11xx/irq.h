/****************************************************************************
 * arch/arm/include/lpc11xxx/irq.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_INCLUDE_LPC11XX_IRQ_H
#define __ARCH_ARM_INCLUDE_LPC11XX_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif
#include <arch/lpc11xx/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* IRQ numbers.  The IRQ number corresponds vector number and hence map
 * directly to bits in the NVIC.  This does, however, waste several words of
 * memory in the IRQ to handle mapping tables.
 */

/* Common Processor Exceptions (vectors 0-15) */

#define LPC11_IRQ_RESERVED       (0) /* Reserved vector (only used with CONFIG_DEBUG_FEATURES) */
                                        /* Vector  0: Reset stack pointer value */
                                        /* Vector  1: Reset (not handler as an IRQ) */
#define LPC11_IRQ_NMI            (2) /* Vector  2: Non-Maskable Interrupt (NMI) */
#define LPC11_IRQ_HARDFAULT      (3) /* Vector  3: Hard fault */
                                        /* Vectors 4-10: Reserved */
#define LPC11_IRQ_SVCALL        (11) /* Vector 11: SVC call */
                                        /* Vector 12-13: Reserved */
#define LPC11_IRQ_PENDSV        (14) /* Vector 14: Pendable system service request */
#define LPC11_IRQ_SYSTICK       (15) /* Vector 15: System tick */

/* External interrupts (vectors >= 16) */

#define LPC11_IRQ_EXTINT        (16) /* Vector number of the first external interrupt */

#if defined(CONFIG_ARCH_CHIP_LPC1115)
#define LPC11_IRQ_PIO0_0        (16) /* Vector 16: PIO0_0 */
#define LPC11_IRQ_PIO0_1        (17) /* Vector 17: PIO0_1 */
#define LPC11_IRQ_PIO0_2        (18) /* Vector 18: PIO0_2 */
#define LPC11_IRQ_PIO0_3        (19) /* Vector 19: PIO0_3 */
#define LPC11_IRQ_PIO0_4        (20) /* Vector 20: PIO0_4 */
#define LPC11_IRQ_PIO0_5        (21) /* Vector 21: PIO0_5 */
#define LPC11_IRQ_PIO0_6        (22) /* Vector 22: PIO0_6 */
#define LPC11_IRQ_PIO0_7        (23) /* Vector 23: PIO0_7 */
#define LPC11_IRQ_PIO0_8        (24) /* Vector 24: PIO0_8 */
#define LPC11_IRQ_PIO0_9        (25) /* Vector 25: PIO0_9 */
#define LPC11_IRQ_PIO0_10       (26) /* Vector 26: PIO0_10 */
#define LPC11_IRQ_PIO0_11       (27) /* Vector 27: PIO0_11 */
#define LPC11_IRQ_PIO1_0        (28) /* Vector 28: PIO1_0 */
#define LPC11_IRQ_CCAN          (29) /* Vector 29: C_CAN controller for LPC11Cxx */
#define LPC11_IRQ_SSP1          (30) /* Vector 30: SPI1/SSP1 */
#define LPC11_IRQ_I2C0          (31) /* Vector 31: I2C0 */
#define LPC11_IRQ_CT16B0        (32) /* Vector 32: Clock/Timer0 16 bits */
#define LPC11_IRQ_CT16B1        (33) /* Vector 33: Clock/Timer1 16 bits */
#define LPC11_IRQ_CT32B0        (34) /* Vector 34: Clock/Timer0 32 bits */
#define LPC11_IRQ_CT32B1        (35) /* Vector 35: Clock/Timer1 32 bits */
#define LPC11_IRQ_SSP0          (36) /* Vector 36: SPI0/SSP0 */
#define LPC11_IRQ_UART          (37) /* Vector 37: UART */
                                     /* Vector 38: Reserved */
                                     /* Vector 39: Reserved */
#define LPC11_IRQ_ADC           (40) /* Vector 40: Analog/Digital Converter */
#define LPC11_IRQ_WDT           (41) /* Vector 41: Watchdog timer */
#define LPC11_IRQ_BOD           (42) /* Vector 42: Brownout Detection */
                                     /* Vector 43: Reserved */
#define LPC11_IRQ_PIO3          (44) /* Vector 44: PIO3 */
#define LPC11_IRQ_PIO2          (45) /* Vector 45: PIO2 */
#define LPC11_IRQ_PIO1          (46) /* Vector 46: PIO1 */
#define LPC11_IRQ_PIO0          (47) /* Vector 47: PIO0 */
#endif

#define NR_VECTORS              (48) /* 48 vectors */
#define NR_IRQS                 (48) /* 32 interrupts plus 16 exceptions */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
typedef void (*vic_vector_t)(uint32_t *regs);

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
extern "C"
{
#endif

#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_INCLUDE_LPC11XX_IRQ_H */
