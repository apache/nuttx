/****************************************************************************
 * arch/sim/src/sim/up_interruptcontext.c
 *
 *   Copyright (C) 2007-2009 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <nuttx/arch.h>
#include "up_internal.h"

#ifdef CONFIG_SIM_PREEMPTIBLE
uint32_t host_signal_save(void);
void host_signal_restore(uint32_t flags);
int up_cpu_simulated_interrupt(void);
#else
#  define host_signal_save()           0
#  define host_signal_restore(f)       (void)(f)
#  define up_cpu_simulated_interrupt() 0
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irq_save
 *
 * Description:
 *   Disable interrupts and returned the mask before disabling them.
 *
 ****************************************************************************/

irqstate_t up_irq_save(void)
{
  return host_signal_save();
}

/****************************************************************************
 * Name: up_irq_restore
 *
 * Input Parameters:
 *   flags - the mask used to restore interrupts
 *
 * Description:
 *   Re-enable interrupts using the specified mask in flags argument.
 *
 ****************************************************************************/

void up_irq_restore(irqstate_t flags)
{
  host_signal_restore(flags);
}

/****************************************************************************
 * Name: up_interrupt_context
 *
 * Description:
 *   Return true is we are currently executing in the interrupt handler
 *   context.
 *
 ****************************************************************************/

bool up_interrupt_context(void)
{
  /* The simulation is never in the interrupt state */

  int is_in_signal = up_cpu_simulated_interrupt();
  return is_in_signal > 0;
}
