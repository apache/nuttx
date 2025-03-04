/****************************************************************************
 * arch/arm/src/sam34/sam_vectors.c
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: 2012 Michael Smith. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

#include "chip.h"
#include "arm_internal.h"
#include "ram_vectors.h"
#include "nvic.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IDLE_STACK      (_ebss + CONFIG_IDLETHREAD_STACKSIZE)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Chip-specific entrypoint */

extern void __start(void);

static void start(void)
{
  /* Zero lr to mark the end of backtrace */

  asm volatile ("mov lr, #0\n\t"
                "b  __start\n\t");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Common exception entrypoint */

extern void exception_common(void);
extern void exception_direct(void);

/****************************************************************************
 * Public data
 ****************************************************************************/

/* The v7m vector table consists of an array of function pointers, with the
 * first slot (vector zero) used to hold the initial stack pointer.
 *
 * As all exceptions (interrupts) are routed via exception_common, we just
 * need to fill this array with pointers to it.
 *
 * Note that the [ ... ] designated initializer is a GCC extension.
 */

const void * const _vectors[] locate_data(".vectors")
                              aligned_data(VECTAB_ALIGN) =
{
  /* Initial stack */

  IDLE_STACK,

  /* Reset exception handler */

  start,

  /* Vectors 2 - n point directly at the generic handler */

  [2 ... NVIC_IRQ_PENDSV] = &exception_common,
  [(NVIC_IRQ_PENDSV + 1) ... (SAM_IRQ_SMP_CALL0 - 1)]
                          = &exception_direct,
  [SAM_IRQ_SMP_CALL0]     = &exception_common,
  [(SAM_IRQ_SMP_CALL0 + 1) ... (SAM_IRQ_SMP_CALL1 - 1)]
                          = &exception_direct,
  [SAM_IRQ_SMP_CALL1]     = &exception_common,
  [(SAM_IRQ_SMP_CALL1 + 1) ... (15 + ARMV7M_PERIPHERAL_INTERRUPTS)]
                          = &exception_direct
};
