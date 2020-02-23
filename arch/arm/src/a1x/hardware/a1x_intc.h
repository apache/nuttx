/************************************************************************************
 * arch/arm/src/a1x/hardware/a1x_intc.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_A1X_HARDWARE_A1X_INTC_H
#define __ARCH_ARM_SRC_A1X_HARDWARE_A1X_INTC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "hardware/a1x_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define A1X_INTC_VECTOR_OFFSET       0x0000 /* Interrupt Vector */
#define A1X_INTC_BASEADDR_OFFSET     0x0004 /* Interrupt Base Address */
#define A1X_INTC_PROTECT_OFFSET      0x0008 /* Interrupt Protection Register */
#define A1X_INTC_NMICTRL_OFFSET      0x000c /* Interrupt Control */

#define A1X_INTC_IRQ_PEND_OFFSET(n)  (0x0010 + (((n) >> 3) & ~3))
#define A1X_INTC_IRQ_PEND0_OFFSET    0x0010 /* Interrupt IRQ Pending 0 Status */
#define A1X_INTC_IRQ_PEND1_OFFSET    0x0014 /* Interrupt IRQ Pending 1 Status */
#define A1X_INTC_IRQ_PEND2_OFFSET    0x0018 /* Interrupt IRQ Pending 2 Status */

#define A1X_INTC_FIQ_PEND_OFFSET(n)  (0x0020 + (((n) >> 3) & ~3))
#define A1X_INTC_FIQ_PEND0_OFFSET    0x0020 /* Interrupt FIQ Pending 0 Status */
#define A1X_INTC_FIQ_PEND1_OFFSET    0x0024 /* Interrupt FIQ Pending 1 Status */
#define A1X_INTC_FIQ_PEND2_OFFSET    0x0028 /* Interrupt FIQ Pending 2 Status */

#define A1X_INTC_FIRQ_SEL_OFFSET(n)  (0x0030 + (((n) >> 3) & ~3))
#define A1X_INTC_IRQ_SEL0_OFFSET     0x0030 /* Interrupt Select 0 */
#define A1X_INTC_IRQ_SEL1_OFFSET     0x0034 /* Interrupt Select 1 */
#define A1X_INTC_IRQ_SEL2_OFFSET     0x0038 /* Interrupt Select 2 */

#define A1X_INTC_EN_OFFSET(n)        (0x0040 + (((n) >> 3) & ~3))
#define A1X_INTC_EN0_OFFSET          0x0040 /* Interrupt Enable 0 */
#define A1X_INTC_EN1_OFFSET          0x0044 /* Interrupt Enable 1 */
#define A1X_INTC_EN2_OFFSET          0x0048 /* Interrupt Enable 2 */

#define A1X_INTC_MASK_OFFSET(n)      (0x0050 + (((n) >> 3) & ~3))
#define A1X_INTC_MASK0_OFFSET        0x0050 /* Interrupt Mask 0 */
#define A1X_INTC_MASK1_OFFSET        0x0054 /* Interrupt Mask 1 */
#define A1X_INTC_MASK2_OFFSET        0x0058 /* Interrupt Mask 2 */

#define A1X_INTC_RESP_OFFSET(n)      (0x0060 + (((n) >> 3) & ~3))
#define A1X_INTC_RESP0_OFFSET        0x0060 /* Interrupt Response 0 */
#define A1X_INTC_RESP1_OFFSET        0x0064 /* Interrupt Response 1 */
#define A1X_INTC_RESP2_OFFSET        0x0068 /* Interrupt Response 2 */

#define A1X_INTC_FF_OFFSET(n)        (0x0070 + (((n) >> 3) & ~3))
#define A1X_INTC_FF0_OFFSET          0x0070 /* Interrupt Fast Forcing 0 */
#define A1X_INTC_FF1_OFFSET          0x0074 /* Interrupt Fast Forcing 1 */
#define A1X_INTC_FF2_OFFSET          0x0078 /* Interrupt Fast Forcing 2 */

#define A1X_INTC_PRIO_OFFSET(n)      (0x0080 + (((n) >> 2) & ~3))
#define A1X_INTC_PRIO0_OFFSET        0x0080 /* Interrupt Source Priority 0 */
#define A1X_INTC_PRIO1_OFFSET        0x0084 /* Interrupt Source Priority 1 */
#define A1X_INTC_PRIO2_OFFSET        0x0088 /* Interrupt Source Priority 2 */
#define A1X_INTC_PRIO3_OFFSET        0x008c /* Interrupt Source Priority 3 */
#define A1X_INTC_PRIO4_OFFSET        0x0090 /* Interrupt Source Priority 4 */

/* Register virtual addresses *******************************************************/

#define A1X_INTC_VECTOR              (A1X_INTC_VADDR+A1X_INTC_VECTOR_OFFSET)
#define A1X_INTC_BASEADDR            (A1X_INTC_VADDR+A1X_INTC_BASEADDR_OFFSET)
#define A1X_INTC_PROTECT             (A1X_INTC_VADDR+A1X_INTC_PROTECT_OFFSET)
#define A1X_INTC_NMICTRL             (A1X_INTC_VADDR+A1X_INTC_NMICTRL_OFFSET)

#define A1X_INTC_IRQ_PEND(n)         (A1X_INTC_VADDR+A1X_INTC_IRQ_PEND_OFFSET(n))
#define A1X_INTC_IRQ_PEND0           (A1X_INTC_VADDR+A1X_INTC_IRQ_PEND0_OFFSET)
#define A1X_INTC_IRQ_PEND1           (A1X_INTC_VADDR+A1X_INTC_IRQ_PEND1_OFFSET)
#define A1X_INTC_IRQ_PEND2           (A1X_INTC_VADDR+A1X_INTC_IRQ_PEND2_OFFSET)

#define A1X_INTC_FIQ_PEND(n)         (A1X_INTC_VADDR+A1X_INTC_FIQ_PEND_OFFSET(n))
#define A1X_INTC_FIQ_PEND0           (A1X_INTC_VADDR+A1X_INTC_FIQ_PEND0_OFFSET)
#define A1X_INTC_FIQ_PEND1           (A1X_INTC_VADDR+A1X_INTC_FIQ_PEND1_OFFSET)
#define A1X_INTC_FIQ_PEND2           (A1X_INTC_VADDR+A1X_INTC_FIQ_PEND2_OFFSET)

#define A1X_INTC_IRQ_SEL(n)          (A1X_INTC_VADDR+A1X_INTC_IRQ_SEL_OFFSET(n))
#define A1X_INTC_IRQ_SEL0            (A1X_INTC_VADDR+A1X_INTC_IRQ_SEL0_OFFSET)
#define A1X_INTC_IRQ_SEL1            (A1X_INTC_VADDR+A1X_INTC_IRQ_SEL1_OFFSET)
#define A1X_INTC_IRQ_SEL2            (A1X_INTC_VADDR+A1X_INTC_IRQ_SEL2_OFFSET)

#define A1X_INTC_EN(n)               (A1X_INTC_VADDR+A1X_INTC_EN_OFFSET(n))
#define A1X_INTC_EN0                 (A1X_INTC_VADDR+A1X_INTC_EN0_OFFSET)
#define A1X_INTC_EN1                 (A1X_INTC_VADDR+A1X_INTC_EN1_OFFSET)
#define A1X_INTC_EN2                 (A1X_INTC_VADDR+A1X_INTC_EN2_OFFSET)

#define A1X_INTC_MASK(n)             (A1X_INTC_VADDR+A1X_INTC_MASK_OFFSET(n))
#define A1X_INTC_MASK0               (A1X_INTC_VADDR+A1X_INTC_MASK0_OFFSET)
#define A1X_INTC_MASK1               (A1X_INTC_VADDR+A1X_INTC_MASK1_OFFSET)
#define A1X_INTC_MASK2               (A1X_INTC_VADDR+A1X_INTC_MASK2_OFFSET)

#define A1X_INTC_RESP(n)             (A1X_INTC_VADDR+A1X_INTC_RESP_OFFSET(n))
#define A1X_INTC_RESP0               (A1X_INTC_VADDR+A1X_INTC_RESP0_OFFSET)
#define A1X_INTC_RESP1               (A1X_INTC_VADDR+A1X_INTC_RESP1_OFFSET)
#define A1X_INTC_RESP2               (A1X_INTC_VADDR+A1X_INTC_RESP2_OFFSET)

#define A1X_INTC_FF(n)               (A1X_INTC_VADDR+A1X_INTC_FF_OFFSET(n))
#define A1X_INTC_FF0                 (A1X_INTC_VADDR+A1X_INTC_FF0_OFFSET)
#define A1X_INTC_FF1                 (A1X_INTC_VADDR+A1X_INTC_FF1_OFFSET)
#define A1X_INTC_FF2                 (A1X_INTC_VADDR+A1X_INTC_FF2_OFFSET)

#define A1X_INTC_PRIO(n)             (A1X_INTC_VADDR+A1X_INTC_PRIO_OFFSET(n))
#define A1X_INTC_PRIO0               (A1X_INTC_VADDR+A1X_INTC_PRIO0_OFFSET)
#define A1X_INTC_PRIO1               (A1X_INTC_VADDR+A1X_INTC_PRIO1_OFFSET)
#define A1X_INTC_PRIO2               (A1X_INTC_VADDR+A1X_INTC_PRIO2_OFFSET)
#define A1X_INTC_PRIO3               (A1X_INTC_VADDR+A1X_INTC_PRIO3_OFFSET)
#define A1X_INTC_PRIO4               (A1X_INTC_VADDR+A1X_INTC_PRIO4_OFFSET)

/* Register bit field definitions ***************************************************/

/* Interrupt Vector */

#define INTC_VECTOR_MASK             0xfffffffc /* Bits 2-31: Vector address */

/* Interrupt Base Address */

#define INTC_BASEADDR_MASK           0xfffffffc /* Bits 2-31: Base address */

/* Interrupt Control */

#define INTC_PROTECT_PROTEN          (1 << 0)  /* Bit 0:  Enabled protected register access */

/* Interrupt Control */

#define INTC_NMICTRL_SRCTYPE_SHIFT   (0)       /* Bits 0-1: External NMI Interrupt Source Type */
#define INTC_NMICTRL_SRCTYPE_MASK    (3 << INTC_NMICTRL_SRCTYPE_SHIFT)
#  define INTC_NMICTRL_SRCTYPE_LOW   (0 << INTC_NMICTRL_SRCTYPE_SHIFT) /* Low level sensitive */
#  define INTC_NMICTRL_SRCTYPE_NEDGE (1 << INTC_NMICTRL_SRCTYPE_SHIFT) /* Negative edge triggered */

/* Interrupt IRQ Pending 0-2 Status */

#define INTC_IRQ_PEND(n)             (1 << ((n) & 0x1f)) /* n=0-95:  Interrupt pending */

/* Interrupt FIQ Pending 0-2 Status */

#define INTC_FIQ_PEND(n)             (1 << ((n) & 0x1f)) /* n=0-95:  Interrupt pending */

/* Interrupt Select 0-2 */

#define INTC_IRQ_SEL(n)              (1 << ((n) & 0x1f)) /* n=0-95:  FIQ (vs IRQ) */

/* Interrupt Enable 0-2 */

#define INTC_EN(n)                   (1 << ((n) & 0x1f)) /* n=0-95:  Interrupt enable */

/* Interrupt Mask 0-2 */

#define INTC_MASK(n)                 (1 << ((n) & 0x1f)) /* n=0-95:  Interrupt mask */

/* Interrupt Response 0-2 */

#define INTC_RESP(n)                 (1 << ((n) & 0x1f)) /* n=0-95:  Interrupt level mask */

/* Interrupt Fast Forcing 0-2 */

#define INTC_FF(n)                   (1 << ((n) & 0x1f)) /* n=0-95:  Enable fast forcing feature */

/* Interrupt Source Priority 0-4 */

#define INTC_PRIO_MIN                0
#define INTC_PRIO_MAX                3

#define INTC_PRIO_SHIFT(n)           (((n) & 15) << 1)   /* n=0-95: Priority level */
#define INTC_PRIO_MASK(n)            (3 << INTC_PRIO_SHIFT(n))
#  define INTC_PRIO(n,p)             ((uint32_t)(p) << INTC_PRIO_SHIFT(n))

#endif /* __ARCH_ARM_SRC_A1X_HARDWARE_A1X_INTC_H */
