/****************************************************************************
 * arch/arm64/src/common/arm64_initialstate.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <arch/limits.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/drivers/drivers.h>
#include <nuttx/fs/loop.h>
#include <nuttx/net/loopback.h>
#include <nuttx/net/tun.h>
#include <nuttx/net/telnet.h>
#include <nuttx/note/note_driver.h>
#include <nuttx/syslog/syslog_console.h>
#include <nuttx/serial/pty.h>
#include <nuttx/crypto/crypto.h>
#include <nuttx/power/pm.h>
#include <arch/chip/chip.h>

#include "addrenv.h"
#include "arm64_arch.h"
#include "arm64_internal.h"
#include "chip.h"
#include "arm64_fatal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void arm64_new_task(struct tcb_s * tcb)
{
  uint64_t stack_ptr = (uintptr_t)tcb->stack_base_ptr + tcb->adj_stack_size;
  struct xcptcontext *xcp = &tcb->xcp;

#ifdef CONFIG_ARCH_KERNEL_STACK
  /* Use the process kernel stack to store context for user processes */

  if (tcb->xcp.kstack)
    {
      tcb->xcp.ustkptr = (uintptr_t *)stack_ptr;
      stack_ptr        = (uintptr_t)tcb->xcp.kstack + ARCH_KERNEL_STACKSIZE;
    }
#endif

  /* Initialize the context registers to stack top */

  xcp->regs = (void *)(stack_ptr - XCPTCONTEXT_SIZE);

  /* Initialize the xcp registers */

  memset(xcp->regs, 0, XCPTCONTEXT_SIZE);

  xcp->regs[REG_ELR]       = (uint64_t)tcb->start;

  /* Keep using SP_EL1 or SP_EL3 */

#if CONFIG_ARCH_ARM64_EXCEPTION_LEVEL == 3
  xcp->regs[REG_SPSR]      = SPSR_MODE_EL3H;
#else
  xcp->regs[REG_SPSR]      = SPSR_MODE_EL1H;
#endif

  xcp->regs[REG_SCTLR_EL1] = read_sysreg(sctlr_el1);
#ifdef CONFIG_ARM64_MTE
  xcp->regs[REG_SCTLR_EL1] |= SCTLR_TCF1_BIT;
#endif

#ifdef CONFIG_SUPPRESS_INTERRUPTS
  xcp->regs[REG_SPSR]     |= (DAIF_IRQ_BIT | DAIF_FIQ_BIT);
#endif /* CONFIG_SUPPRESS_INTERRUPTS */

  xcp->regs[REG_SP_ELX]    = (uint64_t)stack_ptr;
#ifdef CONFIG_ARCH_KERNEL_STACK
  xcp->regs[REG_SP_EL0]    = (uint64_t)tcb->xcp.ustkptr;
#else
  xcp->regs[REG_SP_EL0]    = (uint64_t)stack_ptr - XCPTCONTEXT_SIZE;
#endif
  xcp->regs[REG_EXE_DEPTH] = 0;

#ifndef CONFIG_BUILD_FLAT
  tcb->xcp.initregs        = tcb->xcp.regs;
#endif
}

/****************************************************************************
 * Name: up_initial_state
 *
 * Description:
 *   A new thread is being started and a new TCB has been created. This
 *   function is called to initialize the processor specific portions of
 *   the new TCB.
 *
 *   This function must setup the initial architecture registers and/or
 *   stack so that execution will begin at tcb->start on the next context
 *   switch.
 *
 ****************************************************************************/

void up_initial_state(struct tcb_s *tcb)
{
  struct xcptcontext *xcp = &tcb->xcp;
#ifdef CONFIG_ARCH_KERNEL_STACK
  uint64_t *kstack = xcp->kstack;
#endif

  memset(xcp, 0, sizeof(struct xcptcontext));

#ifdef CONFIG_ARCH_KERNEL_STACK
  xcp->kstack = kstack;
#endif

  if (tcb->pid == IDLE_PROCESS_ID)
    {
      /* Initialize the idle thread stack */
#ifdef CONFIG_SMP
      tcb->stack_alloc_ptr = (void *)(g_cpu_idlestackalloc[0]);
#else
      tcb->stack_alloc_ptr = (void *)(g_idle_stack);
#endif
      tcb->stack_base_ptr  = tcb->stack_alloc_ptr;
      tcb->adj_stack_size  = CONFIG_IDLETHREAD_STACKSIZE;

      /* set initialize idle thread tcb and exception depth
       * core 0, idle0
       */

      write_sysreg(0, tpidrro_el0);

#ifdef CONFIG_STACK_COLORATION

      /* If stack debug is enabled, then fill the stack with a
       * recognizable value that we can use later to test for high
       * water marks.
       */

      arm64_stack_color(tcb->stack_alloc_ptr, 0);
#endif /* CONFIG_STACK_COLORATION */
      return;
    }

  arm64_new_task(tcb);
}
