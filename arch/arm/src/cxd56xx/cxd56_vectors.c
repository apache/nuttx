/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_vectors.c
 *
 *   Copyright (C) 2012 Michael Smith. All rights reserved.
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

#include "chip.h"
#include "arm_internal.h"
#include "ram_vectors.h"
#include "nvic.h"

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
 * Pre-processor Definitions
 ****************************************************************************/

#define IDLE_STACK      (_ebss + CONFIG_IDLETHREAD_STACKSIZE)

/****************************************************************************
 * Public data
 ****************************************************************************/

/* The cx56 use CXD56_IRQ_SMP_CALL to do SMP call.
 * When sig dispatch do up_schedule_sigaction, need to make a new frame to
 * run arm_sigdeliver. But the exception_direct cannot handle xcp.regs as we
 * did not update the regs when c-function expection handler is called.
 *
 * We need to use exception_common to handle SMP call.
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
  [NVIC_IRQ_SYSTICK ... (CXD56_IRQ_SMP_CALL - 1)]
                          = &exception_direct,
  [CXD56_IRQ_SMP_CALL]    = &exception_common,
  [(CXD56_IRQ_SMP_CALL + 1) ... (15 + ARMV7M_PERIPHERAL_INTERRUPTS)]
                          = &exception_direct
};
