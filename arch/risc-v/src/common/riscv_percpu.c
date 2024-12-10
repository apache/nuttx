/****************************************************************************
 * arch/risc-v/src/common/riscv_percpu.c
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
#include <nuttx/irq.h>
#include <nuttx/queue.h>
#include <nuttx/spinlock.h>

#include <arch/barriers.h>
#include <arch/mode.h>

#include <assert.h>
#include <stdint.h>

#include "riscv_internal.h"
#include "riscv_percpu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HART_CNT    (CONFIG_SMP_NCPUS)

static_assert(RISCV_PERCPU_TCB == offsetof(riscv_percpu_t, tcb),
              "RISCV_PERCPU_TCB offset is wrong");
static_assert(RISCV_PERCPU_HARTID == offsetof(riscv_percpu_t, hartid),
              "RISCV_PERCPU_HARTID offset is wrong");
static_assert(RISCV_PERCPU_IRQSTACK == offsetof(riscv_percpu_t, irq_stack),
              "RISCV_PERCPU_IRQSTACK offset is wrong");
static_assert(RISCV_PERCPU_USP == offsetof(riscv_percpu_t, usp),
              "RISCV_PERCPU_USP offset is wrong");
static_assert(RISCV_PERCPU_KSP == offsetof(riscv_percpu_t, ksp),
              "RISCV_PERCPU_KSP offset is wrong");

/****************************************************************************
 * Private Data
 ****************************************************************************/

static riscv_percpu_t   g_percpu[HART_CNT];
static sq_queue_t       g_freelist;
static uintptr_t        g_initialized;
static spinlock_t       g_percpu_spin;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_percpu_init
 *
 * Description:
 *   Initialize the per CPU structures, the first to get here does the init.
 *
 ****************************************************************************/

static void riscv_percpu_init(void)
{
  uintptr_t  i;
  uintptr_t  initialized;
  irqstate_t flags;

  /* Need to lock access during configuration */

  flags = spin_lock_irqsave(&g_percpu_spin);

  /* Initialize if not done so already */

  initialized = g_initialized;
  g_initialized = 1;

  if (initialized == 1)
    {
      goto out_with_lock;
    }

  sq_init(&g_freelist);

  for (i = 0; i < HART_CNT; i++)
    {
      /* Set interrupt stack (if any) */

#if CONFIG_ARCH_INTERRUPTSTACK > 15
      g_percpu[i].irq_stack = (uintptr_t)g_intstacktop - i * INT_STACK_SIZE;
#endif

      sq_addlast((struct sq_entry_s *) &g_percpu[i], &g_freelist);
    }

out_with_lock:
  spin_unlock_irqrestore(&g_percpu_spin, flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_percpu_add_hart
 *
 * Description:
 *   Get add a hart to the per CPU area
 *
 * Input Parameters:
 *   hartid - Hart number
 *
 ****************************************************************************/

void riscv_percpu_add_hart(uintptr_t hartid)
{
  riscv_percpu_t *percpu;
  irqstate_t      flags;

  /* Make sure we are initialized */

  riscv_percpu_init();

  /* Get free entry for this hart, this must not fail */

  flags = spin_lock_irqsave(&g_percpu_spin);
  percpu = (riscv_percpu_t *)sq_remfirst(&g_freelist);
  spin_unlock_irqrestore(&g_percpu_spin, flags);
  DEBUGASSERT(percpu);

  /* Assign hartid, stack has already been assigned */

  percpu->hartid = hartid;

  /* Set the scratch register value to point to the scratch area */

  WRITE_CSR(CSR_SCRATCH, percpu);

  /* Make sure it sticks */

  UP_DSB();
}

/****************************************************************************
 * Name: riscv_percpu_get_hartid
 *
 * Description:
 *   Get harts own hartid by reading it from the per CPU area. This is safe
 *   to use from lower privilege modes (than M-mode).
 *
 * Returned Value:
 *   Hart id
 *
 ****************************************************************************/

uintptr_t riscv_percpu_get_hartid(void)
{
  uintptr_t scratch = READ_CSR(CSR_SCRATCH);

  DEBUGASSERT(scratch >= (uintptr_t) &g_percpu &&
              scratch <  (uintptr_t) &g_percpu + sizeof(g_percpu));

  return ((riscv_percpu_t *)scratch)->hartid;
}

/****************************************************************************
 * Name: riscv_percpu_get_irqstack
 *
 * Description:
 *   Get harts own IRQ stack by reading it from the per CPU area.
 *
 * Returned Value:
 *   IRQ stack, or 0 if no IRQ stack is assigned
 *
 ****************************************************************************/

uintptr_t riscv_percpu_get_irqstack(void)
{
  uintptr_t scratch = READ_CSR(CSR_SCRATCH);

  DEBUGASSERT(scratch >= (uintptr_t) &g_percpu &&
              scratch <  (uintptr_t) &g_percpu + sizeof(g_percpu));

  return ((riscv_percpu_t *)scratch)->irq_stack;
}

/****************************************************************************
 * Name: riscv_percpu_set_kstack
 *
 * Description:
 *   Set current kernel stack, so it can be taken quickly into use when a
 *   trap is taken. Do this whenever a new process moves to running state.
 *
 * Input Parameters:
 *   ksp - Pointer to the kernel stack
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void riscv_percpu_set_kstack(uintptr_t ksp)
{
  irqstate_t flags;
  uintptr_t scratch;

  /* This must be done with interrupts disabled */

  flags   = up_irq_save();
  scratch = READ_CSR(CSR_SCRATCH);

  DEBUGASSERT(scratch >= (uintptr_t) &g_percpu &&
              scratch <  (uintptr_t) &g_percpu + sizeof(g_percpu));

  ((riscv_percpu_t *)scratch)->ksp = ksp;
  up_irq_restore(flags);
}

/****************************************************************************
 * Name: riscv_percpu_set_thread
 *
 * Description:
 *   Set current thread (tcb), so it can be found quickly when a trap is
 *   taken.
 *
 * Input Parameters:
 *   tcb - Pointer to the current thread's tcb
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void riscv_percpu_set_thread(struct tcb_s *tcb)
{
  irqstate_t flags;
  uintptr_t scratch;

  /* This must be done with interrupts disabled */

  flags   = up_irq_save();
  scratch = READ_CSR(CSR_SCRATCH);

  DEBUGASSERT(scratch >= (uintptr_t) &g_percpu &&
              scratch <  (uintptr_t) &g_percpu + sizeof(g_percpu));

  ((riscv_percpu_t *)scratch)->tcb = tcb;
  up_irq_restore(flags);
}
