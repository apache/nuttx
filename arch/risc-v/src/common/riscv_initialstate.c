/****************************************************************************
 * arch/risc-v/src/common/riscv_initialstate.c
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

#include <sys/types.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/tls.h>
#include <nuttx/kmalloc.h>
#include <arch/irq.h>

#include "addrenv.h"
#include "riscv_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_initial_state
 *
 * Description:
 *   A new thread is being started and a new TCB
 *   has been created. This function is called to initialize
 *   the processor specific portions of the new TCB.
 *
 *   This function must setup the initial architecture registers
 *   and/or  stack so that execution will begin at tcb->start
 *   on the next context switch.
 *
 ****************************************************************************/

void up_initial_state(struct tcb_s *tcb)
{
  struct xcptcontext *xcp = &tcb->xcp;
#if defined(CONFIG_ARCH_RV_ISA_V) && (CONFIG_ARCH_RV_VECTOR_BYTE_LENGTH == 0)
  uintptr_t *vregs = tcb->vregs;
#endif
  uintptr_t regval;
  uintptr_t topstack;
#ifdef CONFIG_ARCH_KERNEL_STACK
  uintptr_t *kstack = xcp->kstack;
#endif

  /* Initialize the initial exception register context structure */

  memset(xcp, 0, sizeof(struct xcptcontext));

#if defined(CONFIG_ARCH_RV_ISA_V) && (CONFIG_ARCH_RV_VECTOR_BYTE_LENGTH == 0)

  /* Initialize vector registers */

  if (vregs == NULL)
    {
      regval = READ_CSR(CSR_VLENB);
      if (regval != 0)
        {
          /* There are 32 vector registers(v0 - v31) with vlenb length. */

          xcp->vregs = kmm_calloc(1, regval * 32 + VPU_XCPT_SIZE);
          DEBUGASSERT(xcp->vregs != NULL);
        }
    }
  else
    {
      /* Keep the vector region if task restart */

      xcp->vregs = vregs;
    }
#endif

  /* Initialize the idle thread stack */

  if (tcb->pid == IDLE_PROCESS_ID)
    {
      tcb->stack_alloc_ptr = (void *)g_cpux_idlestack[riscv_mhartid()];
      tcb->stack_base_ptr  = tcb->stack_alloc_ptr;
      tcb->adj_stack_size  = SMP_STACK_SIZE;

#ifdef CONFIG_STACK_COLORATION
      /* If stack debug is enabled, then fill the stack with a
       * recognizable value that we can use later to test for high
       * water marks.
       */

      riscv_stack_color(tcb->stack_alloc_ptr, 0);
#endif /* CONFIG_STACK_COLORATION */

      /* Set idle process' initial interrupt context */

      riscv_set_idleintctx();
      return;
    }

  topstack = (uintptr_t)tcb->stack_base_ptr + tcb->adj_stack_size;

#ifdef CONFIG_ARCH_KERNEL_STACK
  /* Use the process kernel stack to store context for user processes */

  if (kstack)
    {
      xcp->kstack  = kstack;
      xcp->ustkptr = (uintptr_t *)topstack;
      topstack     = (uintptr_t)kstack + ARCH_KERNEL_STACKSIZE;
      xcp->ktopstk = (uintptr_t *)topstack;
      xcp->kstkptr = xcp->ktopstk;
    }
#endif

  xcp->regs = (uintptr_t *)(topstack - XCPTCONTEXT_SIZE);
  memset(xcp->regs, 0, XCPTCONTEXT_SIZE);

  /* Save the initial stack pointer.  Hmmm.. the stack is set to the very
   * beginning of the stack region.  Some functions may want to store data on
   * the caller's stack and it might be good to reserve some space.  However,
   * only the start function would do that and we have control over that one
   */

  xcp->regs[REG_SP]      = topstack;

  /* Save the task entry point */

  xcp->regs[REG_EPC]     = (uintptr_t)tcb->start;

  /* Setup thread local storage pointer */

#ifdef CONFIG_SCHED_THREAD_LOCAL
  xcp->regs[REG_TP]      = (uintptr_t)tcb->stack_alloc_ptr +
                                     sizeof(struct tls_info_s);
#endif

  /* Set the initial value of the interrupt context register.
   *
   * Since various RISC-V platforms use different interrupt
   * methodologies, the value of the interrupt context is
   * part specific.
   *
   */

  regval = riscv_get_newintctx();
  xcp->regs[REG_INT_CTX] = regval;
}
