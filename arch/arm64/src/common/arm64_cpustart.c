/****************************************************************************
 * arch/arm64/src/common/arm64_cpustart.c
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

#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/sched_note.h>
#include <sched/sched.h>
#include <nuttx/cache.h>
#include <nuttx/spinlock.h>
#include <nuttx/init.h>

#include "init/init.h"
#include "arm64_arch.h"
#include "arm64_internal.h"
#include "arm64_gic.h"
#include "arm64_arch_timer.h"
#include "arm64_smp.h"
#include "arm64_cpu_psci.h"

#ifdef CONFIG_ARCH_HAVE_MPU
#include "arm64_mpu.h"
#else
#include "arm64_mmu.h"
#endif

/****************************************************************************
 * Public data
 ****************************************************************************/

uint64_t *const g_cpu_int_stacktop[CONFIG_SMP_NCPUS] =
{
  (uint64_t *)(g_interrupt_stacks[0] + INTSTACK_SIZE),
#if CONFIG_SMP_NCPUS > 1
  (uint64_t *)(g_interrupt_stacks[1] + INTSTACK_SIZE),
#if CONFIG_SMP_NCPUS > 2
  (uint64_t *)(g_interrupt_stacks[2] + INTSTACK_SIZE),
#if CONFIG_SMP_NCPUS > 3
  (uint64_t *)(g_interrupt_stacks[3] + INTSTACK_SIZE),
#if CONFIG_SMP_NCPUS > 4
  (uint64_t *)(g_interrupt_stacks[4] + INTSTACK_SIZE),
#if CONFIG_SMP_NCPUS > 5
#  error This logic needs to extended for CONFIG_SMP_NCPUS > 5
#endif /* CONFIG_SMP_NCPUS > 5 */
#endif /* CONFIG_SMP_NCPUS > 4 */
#endif /* CONFIG_SMP_NCPUS > 3 */
#endif /* CONFIG_SMP_NCPUS > 2 */
#endif /* CONFIG_SMP_NCPUS > 1 */
};

#ifdef CONFIG_ARM64_DECODEFIQ
uint64_t *const g_cpu_int_fiq_stacktop[CONFIG_SMP_NCPUS] =
{
  (uint64_t *)(g_interrupt_fiq_stacks[0] + INTSTACK_SIZE),
#if CONFIG_SMP_NCPUS > 1
  (uint64_t *)(g_interrupt_fiq_stacks[1] + INTSTACK_SIZE),
#if CONFIG_SMP_NCPUS > 2
  (uint64_t *)(g_interrupt_fiq_stacks[2] + INTSTACK_SIZE),
#if CONFIG_SMP_NCPUS > 3
  (uint64_t *)(g_interrupt_fiq_stacks[3] + INTSTACK_SIZE),
#if CONFIG_SMP_NCPUS > 4
  (uint64_t *)(g_interrupt_fiq_stacks[4] + INTSTACK_SIZE),
#if CONFIG_SMP_NCPUS > 5
#  error This logic needs to extended for CONFIG_SMP_NCPUS > 5
#endif /* CONFIG_SMP_NCPUS > 5 */
#endif /* CONFIG_SMP_NCPUS > 4 */
#endif /* CONFIG_SMP_NCPUS > 3 */
#endif /* CONFIG_SMP_NCPUS > 2 */
#endif /* CONFIG_SMP_NCPUS > 1 */
};
#endif

/****************************************************************************
 * Private data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void local_delay(void)
{
  for (volatile int i = 0; i < 1000; i++)
    {
    }
}

static void arm64_smp_init_top(void)
{
  struct tcb_s *tcb = this_task();

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  /* And finally, enable interrupts */

  up_irq_enable();
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION

  /* Notify that this CPU has started */

  sched_note_cpu_started(tcb);
#endif

  /* Reset scheduler parameters */

  nxsched_resume_scheduler(tcb);

  /* core n, idle n */

  write_sysreg(0, tpidrro_el0);
  UNUSED(tcb);

  nx_idle_trampoline();
}

static void arm64_start_cpu(int cpu_num)
{
#ifdef CONFIG_ARM64_PSCI
  uint64_t cpu_mpid = arm64_get_mpid(cpu_num);
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION

  /* Notify of the start event */

  sched_note_cpu_start(this_task(), cpu_num);
#endif

#ifdef CONFIG_ARM64_PSCI
  if (psci_cpu_on(cpu_mpid, (uint64_t)__start))
    {
      serr("Failed to boot secondary CPU core %d (MPID:%#lx)\n", cpu_num,
           cpu_mpid);
      return;
    }
#else
  SP_SEV();
#endif
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
  DEBUGASSERT(cpu >= 0 && cpu < CONFIG_SMP_NCPUS && cpu != this_cpu());

#ifdef CONFIG_SCHED_INSTRUMENTATION

  /* Notify of the start event */

  sched_note_cpu_start(this_task(), cpu);
#endif

#ifdef CONFIG_ARM64_SMP_BUSY_WAIT
  uint32_t *address = (uint32_t *)CONFIG_ARM64_SMP_BUSY_WAIT_FLAG_ADDR;
  *address = 1;
  up_flush_dcache((uintptr_t)address, sizeof(address));
#endif

  arm64_start_cpu(cpu);

  return 0;
}

/* the C entry of secondary cores */

void arm64_boot_secondary_c_routine(void)
{
#ifdef CONFIG_ARCH_HAVE_MPU
  arm64_mpu_init(false);
#endif

#ifdef CONFIG_ARCH_HAVE_MMU
  arm64_mmu_init(false);
#endif

  arm64_gic_secondary_init();

  arm64_arch_timer_secondary_init();

  up_perf_init(NULL);

  arm64_smp_init_top();
}

