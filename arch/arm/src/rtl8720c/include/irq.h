/****************************************************************************
 * arch/arm/include/song/irq.h
 *
 *   Copyright (C) 2017 Pinecone Inc. All rights reserved.
 *   Author: Xiang Xiao <xiaoxiang@pinecone.net>
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

/* This file should never be included directed but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_SONG_IRQ_H
#define __ARCH_ARM_INCLUDE_SONG_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Exception/interrupt vector numbers */

/* Vector  0: Reset stack pointer value */
/* Vector  1: Reset */
#define NVIC_IRQ_NMI               (2)    /* Vector  2: Non-Maskable Interrupt (NMI) */
#define NVIC_IRQ_HARDFAULT         (3)    /* Vector  3: Hard fault */
#define NVIC_IRQ_MEMFAULT          (4)    /* Vector  4: Memory management (MPU) */
#define NVIC_IRQ_BUSFAULT          (5)    /* Vector  5: Bus fault */
#define NVIC_IRQ_USAGEFAULT        (6)    /* Vector  6: Usage fault */
/* Vectors 7-10: Reserved */
#define NVIC_IRQ_SVCALL            (11)   /* Vector 11: SVC call */
#define NVIC_IRQ_DBGMONITOR        (12)   /* Vector 12: Debug Monitor */
/* Vector 13: Reserved */
#define NVIC_IRQ_PENDSV            (14)   /* Vector 14: Pendable system service request */
#define NVIC_IRQ_SYSTICK           (15)   /* Vector 15: System tick */

/* External interrupts (vectors >= 16).  These definitions are chip-specific */

#define NVIC_IRQ_FIRST             (16)   /* Vector number of the first interrupt */
#define NVIC_IRQ_WLAN              (28)   /* Vector 28: Wireless Lan */

/* IRQ numbers.  The IRQ number corresponds vector number and hence map directly to
 * bits in the INTC.  This does, however, waste several words of memory in the IRQ
 * to handle mapping tables.
 */

#define NR_IRQS                    CONFIG_AMEBA_NR_IRQS

/* NVIC priority levels *****************************************************/

#define NVIC_SYSH_PRIORITY_MIN     0xff /* All bits set in minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0xa0 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x40 /* Three bits priority used, bits[7-6] as group */
#define NVIC_SYSH_PRIORITY_SUBSTEP 0x20 /* Three bits priority used, bit[5] as sub */

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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_SONG_IRQ_H */
