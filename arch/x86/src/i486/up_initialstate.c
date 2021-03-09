/****************************************************************************
 * arch/x86/src/i486/up_initialstate.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <string.h>

#include <nuttx/arch.h>
#include <arch/arch.h>

#include "up_internal.h"
#include "up_arch.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_initial_state
 *
 * Description:
 *   A new thread is being started and a new TCB has been created. This
 *   function is called to initialize the processor specific portions of the
 *   new TCB.
 *
 *   This function must setup the initial architecture registers and/or stack
 *   so that execution will begin at tcb->start on the next context switch.
 *
 ****************************************************************************/

void up_initial_state(struct tcb_s *tcb)
{
  struct xcptcontext *xcp = &tcb->xcp;

  /* Initialize the idle thread stack */

  if (tcb->pid == 0)
    {
      tcb->stack_alloc_ptr = (void *)(g_idle_topstack -
                                      CONFIG_IDLETHREAD_STACKSIZE);
      tcb->adj_stack_ptr   = (void *)g_idle_topstack;
      tcb->adj_stack_size  = CONFIG_IDLETHREAD_STACKSIZE;
    }

  /* Initialize the initial exception register context structure */

  memset(xcp, 0, sizeof(struct xcptcontext));

  /* Save the initial stack pointer... the value of the stackpointer before
   * the "interrupt occurs."  We don't know the value of REG_ESP yet..
   * that depends on if a priority change is required or not.
   */

  xcp->regs[REG_SP]      = (uint32_t)tcb->adj_stack_ptr;

  /* Save the task entry point */

  xcp->regs[REG_EIP]     = (uint32_t)tcb->start;

  /* Set up the segment registers... assume the same segment as the caller.
   * That is not a good assumption in the long run.
   */

  xcp->regs[REG_DS]      = up_getds();
  xcp->regs[REG_CS]      = up_getcs();
  xcp->regs[REG_SS]      = up_getss();

  /* Set supervisor- or user-mode, depending on how NuttX is configured and
   * what kind of thread is being started.  Disable FIQs in any event
   *
   * If the kernel build is not selected, then all threads run in
   * supervisor-mode.
   */

#ifdef CONFIG_BUILD_KERNEL
#  error "Missing logic for the CONFIG_BUILD_KERNEL build"
#endif

  /* Enable or disable interrupts, based on user configuration.  If the IF
   * bit is set, maskable interrupts will be enabled.
   */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  xcp->regs[REG_EFLAGS]  = X86_FLAGS_IF;
#endif
}
