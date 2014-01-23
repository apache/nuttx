/************************************************************************************
 * arch/arm/include/efm32s/irq.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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
 
#ifndef __ARCH_ARM_INCLUDE_EFM32_IRQ_H
#define __ARCH_ARM_INCLUDE_EFM32_IRQ_H

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define EFM32_IRQ_RESERVED       (0) /* Reserved vector (only used with CONFIG_DEBUG) */
                                     /* Vector  0: Reset stack pointer value */
                                     /* Vector  1: Reset (not handler as an IRQ) */
#define EFM32_IRQ_NMI            (2) /* Vector  2: Non-Maskable Interrupt (NMI) */
#define EFM32_IRQ_HARDFAULT      (3) /* Vector  3: Hard fault */
#define EFM32_IRQ_MEMFAULT       (4) /* Vector  4: Memory management (MPU) */
#define EFM32_IRQ_BUSFAULT       (5) /* Vector  5: Bus fault */
#define EFM32_IRQ_USAGEFAULT     (6) /* Vector  6: Usage fault */
#define EFM32_IRQ_SVCALL        (11) /* Vector 11: SVC call */
#define EFM32_IRQ_DBGMONITOR    (12) /* Vector 12: Debug Monitor */
                                     /* Vector 13: Reserved */
#define EFM32_IRQ_PENDSV        (14) /* Vector 14: Pendable system service request */
#define EFM32_IRQ_SYSTICK       (15) /* Vector 15: System tick */

/* External interrupts (vectors >= 16).  These definitions are chip-specific */

#define EFM32_IRQ_INTERRUPTS    (16) /* Vector number of the first external interrupt */

#define ARMV7M_PERIPHERAL_INTERRUPTS 38

#define NR_IRQS (16 + ARMV7M_PERIPHERAL_INTERRUPTS)

#endif /* __ARCH_ARM_INCLUDE_EFM32_IRQ_H */
