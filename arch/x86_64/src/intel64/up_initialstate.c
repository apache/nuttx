/****************************************************************************
 *  arch/x86_64/src/intel64/up_initialstate.c
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
#include "sched/sched.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
 *   This function must setup the intial architecture registers and/or stack
 *   so that execution will begin at tcb->start on the next context switch.
 *
 ****************************************************************************/

extern uintptr_t tux_mm_new_pd1(void);

void up_initial_state(struct tcb_s *tcb)
{
  struct xcptcontext *xcp = &tcb->xcp;
  struct tcb_s *rtcb;

  /* Initialize the initial exception register context structure */

  memset(xcp, 0, sizeof(struct xcptcontext));

  /* set the FCW to 1f80 */
  xcp->regs[1]      = (uint64_t)0x0000037f00000000;

  /* set the MXCSR to 1f80 */
  xcp->regs[3]      = (uint64_t)0x0000000000001f80;

  /* set page table to share space with current process */

  rtcb = this_task();

  /* Save the initial stack pointer... the value of the stackpointer before
   * the "interrupt occurs."
   */

  xcp->regs[REG_RSP]      = (uint64_t)tcb->adj_stack_ptr;
  xcp->regs[REG_RBP]      = (uint64_t)tcb->adj_stack_ptr;

  /* Save the task entry point */

  xcp->regs[REG_RIP]     = (uint64_t)tcb->start;

  /* Set up the segment registers... assume the same segment as the caller.
   * That is not a good assumption in the long run.
   */

  xcp->regs[REG_DS]      = up_getds();
  xcp->regs[REG_CS]      = up_getcs();
  xcp->regs[REG_SS]      = up_getss();
  xcp->regs[REG_ES]      = up_getes();

  /* Aux GS and FS are set to be 0 */
  /* used by some libc for TLS and segment reference */
  xcp->regs[REG_GS]      = 0;
  xcp->regs[REG_FS]      = 0;

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
  xcp->regs[REG_RFLAGS]  = X86_64_RFLAGS_IF;
#endif
}

