/****************************************************************************
 * arch/sparc/src/s698pm/s698pm_cpustart.c
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
#include <assert.h>
#include <debug.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include <nuttx/sched_note.h>

#include "sched/sched.h"
#include "up_internal.h"

#ifdef CONFIG_BUILD_KERNEL
#  include "s698pm_mmu.h"
#endif

#include "s698pm.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile static spinlock_t g_cpu_boot;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s698pm_cpu_boot
 *
 * Description:
 *   Boot handler for cpu[x]
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void s698pm_cpu_boot(void)
{
  struct tcb_s *tcb = this_task();

  _info("CPU%d Started\n", this_cpu());

  /* Initialize CPU interrupts */

  s698pm_cpuint_initialize();

  spin_unlock(&g_cpu_boot);

#ifdef CONFIG_SCHED_INSTRUMENTATION
  /* Notify that this CPU has started */

  sched_note_cpu_started(tcb);
#endif

  /* Reset scheduler parameters */

  nxsched_resume_scheduler(tcb);

  /* And finally, enable cpu interrupts */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  up_irq_enable();
#endif

  /* Then transfer control to the IDLE task */

  nx_idle_trampoline();
}

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
  uintptr_t regaddr;

  _info("CPU=%d\n", cpu);

#ifdef CONFIG_SCHED_INSTRUMENTATION
  /* Notify of the start event */

  sched_note_cpu_start(this_task(), cpu);
#endif

  /* Set the start up address */

  regaddr = S698PM_DSU_BASE + (0x1000000 * cpu) + S698PM_DSU_PC_OFFSET;
  putreg32(CONFIG_RAM_START, regaddr);
  putreg32(0x40001000, regaddr);

  regaddr = S698PM_DSU_BASE + (0x1000000 * cpu) + S698PM_DSU_NPC_OFFSET;
  putreg32(0x40001004, regaddr);

  spin_lock(&g_cpu_boot);

  /* set 1 to bit n of multiprocessor status register to active cpu n */

  putreg32(1 << cpu, S698PM_IRQREG_MPSTATUS);

  spin_lock(&g_cpu_boot);

  /* prev cpu boot done */

  spin_unlock(&g_cpu_boot);

  return 0;
}
