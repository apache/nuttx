/****************************************************************************
 * arch/arm/src/lc823450/lc823450_cpustart.c
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

#pragma GCC optimize ("O0")

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>
#include <string.h>
#include <stdio.h>

#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include <nuttx/sched_note.h>

#include "nvic.h"
#include "sched/sched.h"
#include "init/init.h"
#include "arm_internal.h"
#include "lc823450_syscontrol.h"

#if defined(CONFIG_BUILD_FLAT) && defined(CONFIG_ARM_MPU)
#  include "lc823450_mpuinit2.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if 0
#define DPRINTF(fmt, args...) llinfo(fmt, ##args)
#else
#define DPRINTF(fmt, args...) do {} while (0)
#endif

#define CPU1_VECTOR_ISTACK  0x00000000
#define CPU1_VECTOR_RESETV  0x00000004

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern volatile spinlock_t g_cpu_wait[CONFIG_SMP_NCPUS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

extern int lc823450_pause_handler(int irq, void *c, void *arg);

/****************************************************************************
 * Name: cpu1_boot
 *
 * Description:
 *   This is the boot vector for Cortex-M3 #1
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static void cpu1_boot(void)
{
  int cpu = up_cpu_index();

  DPRINTF("cpu = %d\n", cpu);

  if (cpu == 1)
    {
      putreg32((uint32_t)_stext, NVIC_VECTAB); /* use CPU0 vectors */

#if defined(CONFIG_BUILD_FLAT) && defined(CONFIG_ARM_MPU)
      lc823450_mpuinitialize();

      irq_attach(LC823450_IRQ_MEMFAULT, arm_memfault, NULL);
      up_enable_irq(LC823450_IRQ_MEMFAULT);
#endif

      irq_attach(LC823450_IRQ_CTXM3_01, lc823450_pause_handler, NULL);
      up_enable_irq(LC823450_IRQ_CTXM3_01);
    }

  spin_unlock(&g_cpu_wait[0]);

#ifdef CONFIG_SCHED_INSTRUMENTATION
  /* Notify that this CPU has started */

  sched_note_cpu_started(this_task());
#endif

  /* Then transfer control to the IDLE task */

  nx_idle_trampoline();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_cpu_start
 *
 * Description:
 *   In an SMP configuration, only one CPU is initially active (CPU 0).
 *   System initialization occurs on that single thread. At the completion of
 *   the initialization of the OS, just before beginning normal multitasking,
 *   the additional CPUs would be started by calling this function.
 *
 *   Each CPU is provided the entry point to its IDLE task when started.  A
 *   TCB for each CPU's IDLE task has been initialized and placed in the
 *   CPU's g_assignedtasks[cpu] list.  No stack has been allocated or
 *   initialized.
 *
 *   The OS initialization logic calls this function repeatedly until each
 *   CPU has been started, 1 through (CONFIG_SMP_NCPUS-1).
 *
 * Input Parameters:
 *   cpu - The index of the CPU being started.  This will be a numeric
 *         value in the range of one to (CONFIG_SMP_NCPUS-1).
 *         (CPU 0 is already active)
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_cpu_start(int cpu)
{
  struct tcb_s *tcb = current_task(cpu);
  uint32_t backup[2];

  DPRINTF("cpu=%d\n", cpu);

  if (cpu != 1)
    {
      return -1;
    }

  /* create initial vectors for CPU1 */

  putreg32(0x1, REMAP); /* remap enable */
  backup[0] = getreg32(CPU1_VECTOR_ISTACK);
  backup[1] = getreg32(CPU1_VECTOR_RESETV);
  putreg32((uint32_t)tcb->stack_base_ptr +
           tcb->adj_stack_size, CPU1_VECTOR_ISTACK);
  putreg32((uint32_t)cpu1_boot, CPU1_VECTOR_RESETV);

  spin_lock(&g_cpu_wait[0]);

#ifdef CONFIG_SCHED_INSTRUMENTATION
  /* Notify of the start event */

  sched_note_cpu_start(this_task(), cpu);
#endif

  /* enable clock core #1 */

  modifyreg32(CORECNT, 0, CORECNT_C1CLKEN);

  /* unreset core #1 */

  modifyreg32(CORECNT, 0, CORECNT_C1RSTN);

  /* IRQ setup CPU1->CPU0 */

  irq_attach(LC823450_IRQ_CTXM3_11, lc823450_pause_handler, NULL);
  up_enable_irq(LC823450_IRQ_CTXM3_11);

  spin_lock(&g_cpu_wait[0]);

  /* CPU1 boot done */

  /* restore : after CPU1 boot, CPU1 use normal vectors table. */

  putreg32(backup[0], CPU1_VECTOR_ISTACK);
  putreg32(backup[1], CPU1_VECTOR_RESETV);
  putreg32(0x0, REMAP); /* remap disable */

  spin_unlock(&g_cpu_wait[0]);

  return 0;
}
