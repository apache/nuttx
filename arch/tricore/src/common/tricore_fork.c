/****************************************************************************
 * arch/tricore/src/common/tricore_fork.c
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
#include <stdint.h>
#include <string.h>
#include <sys/types.h>

#include <syslog.h>
#include <debug.h>
#include <nuttx/irq.h>

#include "sched/sched.h"
#include "tricore_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TRICORE_CSA_NEXT       0
#define TRICORE_CSA_CHAIN_MASK 0xFFF00000u

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct fork_s
{
  volatile uintptr_t lcsa[TC_CONTEXT_REGS]; /* Generate lcsa and save manually */
  uintptr_t *pucsa;                         /* Pointer to the in-stack csa */
  uintptr_t ra;
  uintptr_t sp;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static pid_t tricore_fork(bool vfork, const struct fork_s *context)
{
  struct tcb_s *parent = this_task();
  uintptr_t *csa_parent;
  uintptr_t *csa_child;
  struct tcb_s *child;
  uintptr_t *csa_prev = NULL;
  uintptr_t oldtop;
  uintptr_t oldsp;
  uintptr_t oldbase;
  uintptr_t stackutil;
  uintptr_t newtop;
  uintptr_t newsp;
  uintptr_t newbase;
  uintptr_t pcxi;
  irqstate_t flags;
  bool is_upper;
  int csa_cnt;
  int i;

  /* Calculate the parent used csa, lcsa copy and freed back */

  sinfo("parent regs=%p\n", context->pucsa);

  oldbase = (uintptr_t)parent->stack_base_ptr;
  oldtop  = oldbase + parent->adj_stack_size;
  oldsp   = context->sp;

  DEBUGASSERT(oldtop > oldsp && oldsp >= oldbase);
  stackutil = oldtop - oldsp;

  csa_cnt = 0;
  for (pcxi = context->lcsa[TRICORE_CSA_NEXT]; (pcxi & FCX_FREE) != 0;
       pcxi = tricore_csa2addr(pcxi)[TRICORE_CSA_NEXT])
    {
      csa_cnt++;
    }

  sinfo("parent_csa_cnt=%d\n", csa_cnt);

  uintptr_t *csa_list[csa_cnt + 1];

  /* Allocate and initialize a TCB & stack for the child task. */

  child = nxtask_setup_fork((start_t)context->ra, vfork);
  if (!child)
    {
      serr("ERROR: nxtask_setup_fork failed\n");
      return (pid_t)ERROR;
    }

  newbase = (uintptr_t)child->stack_base_ptr;
  newtop  = newbase + child->adj_stack_size;
  newsp   = newtop - stackutil;

  sinfo("TCBs: Parent=%p Child=%p\n", parent, child);

  /* Copy Stack directly. */

  memcpy((void *)newsp, (const void *)oldsp, stackutil);

  sinfo("sp from=%zX to=%zX size=%zX\n", oldsp, newsp, stackutil);

  flags = up_irq_save();

  /* Release the initial context, the child resumes from the copied one */

  tricore_reclaim_csa(tricore_addr2csa(child->xcp.regs));

  /* Take csa_cnt + 1 CSAs from the free list, they are already linked */

  UP_DSB();

  pcxi      = tricore_mfcr(TRICORE_CPU_FCX);
  csa_child = tricore_csa2addr(pcxi);

  for (i = 0; i <= csa_cnt; i++)
    {
      DEBUGASSERT((pcxi & FCX_FREE) != 0);
      csa_list[i] = csa_child;
      pcxi      = csa_child[TRICORE_CSA_NEXT];
      csa_child = tricore_csa2addr(pcxi);
    }

  tricore_mtcr(TRICORE_CPU_FCX, pcxi);

  UP_ISB();

  up_irq_restore(flags);

  /* Copy whole csa chain */

  csa_parent = tricore_csa2addr(context->lcsa[TRICORE_CSA_NEXT]);

  for (i = csa_cnt - 1; i >= 0; i--)
    {
      uintptr_t *regs = csa_list[csa_cnt - i];

      memcpy(regs, csa_parent, TC_CONTEXT_SIZE);
      csa_parent = tricore_csa2addr(csa_parent[TRICORE_CSA_NEXT]);
    }

  /* Copy backuped parent csa */

  memcpy(csa_list[0], (void *)context->lcsa, TC_CONTEXT_SIZE);

  /* Clear the Return value of child thread */

  csa_list[0][REG_D2] = 0;

  /* Increase the csa_cnt backuped from lcsa */

  csa_cnt++;

  /* Correct csa chain */

  for (i = 0; i < csa_cnt; i++)
    {
      uintptr_t *regs = csa_list[csa_cnt - 1 - i];
      uintptr_t next_value;

      next_value  = regs[TRICORE_CSA_NEXT] & TRICORE_CSA_CHAIN_MASK;
      next_value |= tricore_addr2csa(csa_prev);
      regs[TRICORE_CSA_NEXT] = next_value;
      csa_prev = regs;
    }

  /* Correct sp */

  is_upper = false;

  for (i = csa_cnt - 1; i >= 0; i--)
    {
      uintptr_t *regs = csa_list[csa_cnt - 1 - i];

      if (is_upper)
        {
          regs[REG_SP] = regs[REG_SP] - oldsp + newsp;
        }

      is_upper = (regs[TRICORE_CSA_NEXT] & PCXI_UL) != 0;
    }

  /* Correct the last regs */

  sinfo("child set regs=%p\n", csa_prev);
  child->xcp.regs = csa_prev;

  /* Correct syscall context */

#ifdef CONFIG_LIB_SYSCALL
  if (parent->xcp.nsyscalls > 0)
    {
      int index;

      child->xcp.nsyscalls = parent->xcp.nsyscalls;
      for (index = 0; index < parent->xcp.nsyscalls; index++)
        {
          csa_parent = (uintptr_t *)context->lcsa;
          for (i = 0; i < csa_cnt; i++)
            {
              if (parent->xcp.syscall_regs[index] == csa_parent)
                {
                  child->xcp.syscall_regs[index] = csa_list[i];
                  break;
                }

              csa_parent =
                tricore_csa2addr(csa_parent[TRICORE_CSA_NEXT]);
            }
        }
    }
#endif

  return nxtask_start_fork(child, vfork);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

pid_t up_fork(bool vfork)
{
  struct fork_s context;
  uintptr_t *csa_addr;

  __asm__ __volatile__ ("svlcx" ::: "memory");
  csa_addr = tricore_csa2addr(tricore_mfcr(TRICORE_CPU_PCXI));
  memcpy((void *)context.lcsa, csa_addr, TC_CONTEXT_SIZE);

  context.pucsa = tricore_csa2addr(csa_addr[TRICORE_CSA_NEXT]);

  context.ra = context.pucsa[REG_RA];
  context.sp = context.pucsa[REG_SP];

  /* If the rslcx instrument before get pucsa will
   * case trap because of csa_addr[0] inconsistent value
   */

  __asm__ __volatile__ ("rslcx" ::: "memory",
                        "d0", "d1", "d2", "d3",
                        "d4", "d5", "d6", "d7",
                        "a2", "a3", "a4", "a5",
                        "a6", "a7");

  sinfo("lcsa=0x%zX pucsa=%p recalc=%p\n",
        context.lcsa[TRICORE_CSA_NEXT], context.pucsa,
        tricore_csa2addr(context.lcsa[TRICORE_CSA_NEXT]));

  return tricore_fork(vfork, &context);
}
