/****************************************************************************
 * arch/arm/src/armv7-m/ram_vectors.h
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_COMMON_ARMV7_M_RAM_VECTORS_H
#define __ARCH_ARM_SRC_COMMON_ARMV7_M_RAM_VECTORS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/irq.h>

#include "up_internal.h"

#ifdef CONFIG_ARCH_RAMVECTORS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This is the size of the vector table (in 4-byte entries).  This size
 * includes the (1) the peripheral interrupts, (2) space for 15 Cortex-M
 * exceptions, and (3) IDLE stack pointer which lies at the beginning of the
 * table.
 */

#define ARMV7M_VECTAB_SIZE (NR_VECTORS + 16)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* If CONFIG_ARCH_RAMVECTORS is defined, then the ARM logic must provide
 * ARM-specific implementations of irq_initialize(), irq_attach(), and
 * irq_dispatch.  In this case, it is also assumed that the ARM vector
 * table resides in RAM, has the name up_ram_vectors, and has been
 * properly positioned and aligned in memory by the linker script.
 *
 * REVISIT: Can this alignment requirement vary from core-to-core?  Yes, it
 * depends on the number of vectors supported by the MCU. The safest thing
 * to do is to put the vector table at the beginning of RAM in order toforce
 * the highest alignment possible.
 */

extern up_vector_t g_ram_vectors[ARMV7M_VECTAB_SIZE]
  __attribute__ ((section (".ram_vectors"), aligned (128)));

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: up_ramvec_initialize
 *
 * Description:
 *   Copy vectors to RAM an configure the NVIC to use the RAM vectors.
 *
 ****************************************************************************/

void up_ramvec_initialize(void);

/****************************************************************************
 * Name: exception_common
 *
 * Description:
 *   This is the default, common vector handling entrypoint.
 *
 ****************************************************************************/

void exception_common(void);

/****************************************************************************
 * Name: up_ramvec_attach
 *
 * Description:
 *   Configure the ram vector table so that IRQ number 'irq' will be
 *   dipatched by hardware to 'vector'
 *
 ****************************************************************************/

int up_ramvec_attach(int irq, up_vector_t vector);

#endif /* CONFIG_ARCH_RAMVECTORS */
#endif  /* __ARCH_ARM_SRC_COMMON_ARMV7_M_RAM_VECTORS_H */
