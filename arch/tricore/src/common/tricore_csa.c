/****************************************************************************
 * arch/tricore/src/common/tricore_csa.c
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

#include <sys/types.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/tls.h>
#include <arch/barriers.h>
#include <arch/irq.h>

#include "tricore_internal.h"

#ifdef CONFIG_SMP
#  include <nuttx/spinlock.h>
#  include "tricore_smp.h"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A CSA frame belongs to the CPU whose data scratchpad RAM holds it.  When
 * a task moves to another CPU, or a CPU builds the first context of a task
 * for another CPU, frames of one CPU end up in the free list of another.
 * Such frames are handed back to their owner through its return list.
 */

#ifdef CONFIG_SMP
static spinlock_t g_csa_return_lock[CONFIG_SMP_NCPUS];
static uintptr_t g_csa_return_head[CONFIG_SMP_NCPUS];
static uintptr_t g_csa_return_tail[CONFIG_SMP_NCPUS];
static volatile bool g_csa_reclaim_req[CONFIG_SMP_NCPUS];
static uint32_t g_csa_switches[CONFIG_SMP_NCPUS];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_SMP

/****************************************************************************
 * Name: tricore_csa_owner
 *
 * Description:
 *   Return the CPU that owns the CSA frame of a link word, or -1.
 *
 ****************************************************************************/

static int tricore_csa_owner(uintptr_t link)
{
  int cpu;

  cpu = TRICORE_SMP_CPU_OF_CORE(TRICORE_SMP_DSPR_SEG_TOP -
                                (int)((link & FCX_FCXS_MASK) >> FCX_FCXS));
  if (cpu < 0 || cpu >= CONFIG_SMP_NCPUS)
    {
      return -1;
    }

  return cpu;
}

/****************************************************************************
 * Name: tricore_csa_take_returns
 *
 * Description:
 *   Put the frames that other CPUs gave back to this CPU on its free list.
 *
 ****************************************************************************/

static void tricore_csa_take_returns(int me)
{
  irqstate_t flags;
  uintptr_t head;
  uintptr_t tail;

  if (g_csa_return_head[me] == 0)
    {
      return;
    }

  flags = spin_lock_irqsave(&g_csa_return_lock[me]);
  head = g_csa_return_head[me];
  tail = g_csa_return_tail[me];
  g_csa_return_head[me] = 0;
  g_csa_return_tail[me] = 0;

  if (head != 0)
    {
      tricore_csa2addr(tail)[0] = tricore_mfcr(TRICORE_CPU_FCX) & FCX_FREE;
      tricore_mtcr(TRICORE_CPU_FCX, head);
      UP_ISB();
    }

  spin_unlock_irqrestore(&g_csa_return_lock[me], flags);
}

/****************************************************************************
 * Name: tricore_csa_give_foreign
 *
 * Description:
 *   Take the frames of other CPUs out of the first "limit" own frames of
 *   the free list and queue them on the return lists of their owners.
 *   Return true if fewer than "limit" own frames are free.
 *
 ****************************************************************************/

static bool tricore_csa_give_foreign(int me, unsigned int limit)
{
  uintptr_t head[CONFIG_SMP_NCPUS];
  uintptr_t tail[CONFIG_SMP_NCPUS];
  uintptr_t *prev;
  uintptr_t *addr;
  uintptr_t link;
  uintptr_t next;
  uintptr_t lcx;
  unsigned int count;
  irqstate_t flags;
  int owner;
  int cpu;

  for (cpu = 0; cpu < CONFIG_SMP_NCPUS; cpu++)
    {
      head[cpu] = 0;
      tail[cpu] = 0;
    }

  flags = up_irq_save();

  prev  = NULL;
  link  = tricore_mfcr(TRICORE_CPU_FCX) & FCX_FREE;
  lcx   = tricore_mfcr(TRICORE_CPU_LCX) & FCX_FREE;
  count = 0;

  while (link != 0 && link != lcx && count < limit)
    {
      addr  = tricore_csa2addr(link);
      next  = addr[0] & FCX_FREE;
      owner = tricore_csa_owner(link);

      if (owner >= 0 && owner != me)
        {
          if (prev == NULL)
            {
              tricore_mtcr(TRICORE_CPU_FCX, next);
              UP_ISB();
            }
          else
            {
              prev[0] = next;
            }

          addr[0] = head[owner];
          head[owner] = link;
          if (tail[owner] == 0)
            {
              tail[owner] = link;
            }
        }
      else
        {
          prev = addr;
          count++;
        }

      link = next;
    }

  up_irq_restore(flags);

  for (cpu = 0; cpu < CONFIG_SMP_NCPUS; cpu++)
    {
      if (head[cpu] == 0)
        {
          continue;
        }

      flags = spin_lock_irqsave(&g_csa_return_lock[cpu]);
      tricore_csa2addr(tail[cpu])[0] = g_csa_return_head[cpu];
      if (g_csa_return_tail[cpu] == 0)
        {
          g_csa_return_tail[cpu] = tail[cpu];
        }

      g_csa_return_head[cpu] = head[cpu];
      spin_unlock_irqrestore(&g_csa_return_lock[cpu], flags);
    }

  return count < limit;
}

/****************************************************************************
 * Name: tricore_csa_ask_reclaim
 *
 * Description:
 *   Ask the other CPUs to give back the frames of this CPU.
 *
 ****************************************************************************/

static void tricore_csa_ask_reclaim(int me)
{
  cpu_set_t cpuset = 0;
  int cpu;

  for (cpu = 0; cpu < CONFIG_SMP_NCPUS; cpu++)
    {
      if (cpu != me && !g_csa_reclaim_req[cpu])
        {
          g_csa_reclaim_req[cpu] = true;
          cpuset |= (cpu_set_t)1 << cpu;
        }
    }

  if (cpuset != 0)
    {
      up_trigger_irq(TRICORE_SMP_IPI_IRQ(me, me), cpuset);
    }
}

#endif /* CONFIG_SMP */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_alloc_csa
 ****************************************************************************/

uintptr_t *tricore_alloc_csa(uintptr_t pc, uintptr_t sp,
                             uintptr_t psw, bool irqsave)
{
  uintptr_t *plcsa;
  uintptr_t *pucsa;

  plcsa = (uintptr_t *)tricore_csa2addr(tricore_mfcr(TRICORE_CPU_FCX));

  /* DSYNC instruction should be executed immediately prior to the MTCR */

  UP_DSB();

  pucsa = (uintptr_t *)tricore_csa2addr(plcsa[REG_UPCXI]);

  tricore_mtcr(TRICORE_CPU_FCX, pucsa[REG_UPCXI]);

  /* ISYNC instruction executed immediately following MTCR */

  UP_ISB();

  memset(pucsa, 0, TC_CONTEXT_SIZE);
  memset(plcsa, 0, TC_CONTEXT_SIZE);

  pucsa[REG_SP]  = sp;
  pucsa[REG_PSW] = psw;

  /* Save the task entry point */

  pucsa[REG_UPC] = pc;
  plcsa[REG_LPC] = pc;

  plcsa[REG_LPCXI] = (PCXI_UL | tricore_addr2csa(pucsa));

  if (!irqsave)
    {
      plcsa[REG_LPCXI] |= PCXI_PIE;
    }

  return plcsa;
}

/****************************************************************************
 * Name: tricore_reclaim_csa
 ****************************************************************************/

void tricore_reclaim_csa(uintptr_t pcxi)
{
  uintptr_t head;
  uintptr_t tail;
  uintptr_t free;
  uintptr_t *next;

  /* A pointer to the first CSA in the list of CSAs consumed by the task is
   * stored in the first element of the tasks TCB structure (where the stack
   * pointer would be on a traditional stack based architecture).
   */

  head = pcxi & FCX_FREE;

  /* Mask off everything in the CSA link field other than the address.  If
   * the address is NULL, then the CSA is not linking anywhere and there is
   * nothing to do.
   */

  tail = head;

  /* Convert the link value to contain just a raw address and store this
   * in a local variable.
   */

  next = tricore_csa2addr(tail);

  /* Iterate over the CSAs that were consumed as part of the task.  The
   * first field in the CSA is the pointer to then next CSA.  Mask off
   * everything in the pointer to the next CSA, other than the link address.
   * If this is NULL, then the CSA currently being pointed to is the last in
   * the chain.
   */

  while ((next[0] & FCX_FREE) != 0)
    {
      /* Clear all bits of the pointer to the next in the chain, other
       * than the address bits themselves.
       */

      next[0] = next[0] & FCX_FREE;

      /* Move the pointer to point to the next CSA in the list. */

      tail = next[0];

      /* Update the local pointer to the CSA. */

      next = tricore_csa2addr(tail);
    }

  /* Look up the current free CSA head. */

  free = tricore_mfcr(TRICORE_CPU_FCX);

  /* Join the current Free onto the Tail of what is being reclaimed. */

  tricore_csa2addr(tail)[0] = free;

  /* Move the head of the reclaimed into the Free. */

  tricore_mtcr(TRICORE_CPU_FCX, head);
}

#ifdef CONFIG_SMP

/****************************************************************************
 * Name: tricore_csa_balance
 *
 * Description:
 *   Take back the own frames and give back all foreign ones.  Called by
 *   the idle loop.
 *
 ****************************************************************************/

void tricore_csa_balance(void)
{
  int me = this_cpu();

  tricore_csa_take_returns(me);
  tricore_csa_give_foreign(me, TRICORE_SMP_CSA_SCAN_ALL);
}

/****************************************************************************
 * Name: tricore_csa_switch_balance
 *
 * Description:
 *   Called at every context switch: take back the own frames and give back
 *   the foreign ones among the first few free frames.  Every
 *   TRICORE_SMP_CSA_LOW_EVERY switches check the low water mark and, when
 *   below it, ask the other CPUs to give back the frames of this CPU.
 *
 ****************************************************************************/

void tricore_csa_switch_balance(void)
{
  int me = this_cpu();

  tricore_csa_take_returns(me);

  g_csa_switches[me]++;
  if ((g_csa_switches[me] % TRICORE_SMP_CSA_LOW_EVERY) != 0)
    {
      tricore_csa_give_foreign(me, TRICORE_SMP_CSA_SWITCH_SCAN);

      return;
    }

  if (tricore_csa_give_foreign(me, TRICORE_SMP_CSA_LOW_WATER))
    {
      tricore_csa_ask_reclaim(me);
    }
}

/****************************************************************************
 * Name: tricore_csa_reclaim_ipi
 *
 * Description:
 *   Called from the inter-CPU interrupt: give back foreign frames if
 *   another CPU asked for them.
 *
 ****************************************************************************/

void tricore_csa_reclaim_ipi(void)
{
  int me = this_cpu();

  if (!g_csa_reclaim_req[me])
    {
      return;
    }

  g_csa_reclaim_req[me] = false;
  tricore_csa_balance();
}

#endif /* CONFIG_SMP */
