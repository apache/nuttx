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
#include <arch/spinlock.h>
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

typedef void (*arm64_cpustart_t)(void *data);

struct arm64_boot_params
{
  uint64_t mpid;
  char *boot_sp;
  arm64_cpustart_t func;
  void *arg;
  int cpu_num;
  volatile long cpu_ready_flag;
};

volatile struct arm64_boot_params aligned_data(L1_CACHE_BYTES)
cpu_boot_params =
{
  .mpid    = -1,
  .boot_sp = (char *)g_cpu_idlestackalloc[0],
};

volatile uint64_t *g_cpu_int_stacktop[CONFIG_SMP_NCPUS] =
{
  (uint64_t *)(g_interrupt_stacks[0] + INTSTACK_SIZE),
};

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

#if defined (CONFIG_ARCH_HAVE_MMU) || defined (CONFIG_ARCH_HAVE_MPU)
static void flush_boot_params(void)
{
  uintptr_t flush_start;
  uintptr_t flush_end;

  flush_start   = (uintptr_t)&cpu_boot_params;
  flush_end     = flush_start + sizeof(cpu_boot_params);

  up_flush_dcache(flush_start, flush_end);
}
#endif

static void arm64_smp_init_top(void *arg)
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
  write_sysreg(tcb, tpidr_el1);
  write_sysreg(tcb, tpidr_el0);

  cpu_boot_params.cpu_ready_flag = 1;
  SP_SEV();

  nx_idle_trampoline();
}

static void arm64_start_cpu(int cpu_num, char *stack, int stack_sz,
                            arm64_cpustart_t fn)
{
  uint64_t cpu_mpid;

  cpu_mpid = arm64_get_mpid(cpu_num);

#ifdef CONFIG_SCHED_INSTRUMENTATION

  /* Notify of the start event */

  sched_note_cpu_start(this_task(), cpu);
#endif

  cpu_boot_params.boot_sp   = stack;
  cpu_boot_params.func      = fn;
  cpu_boot_params.arg       = 0;
  cpu_boot_params.cpu_num   = cpu_num;
  g_cpu_int_stacktop[cpu_num] =
            (uint64_t *)(g_interrupt_stacks[cpu_num] + INTSTACK_SIZE);

  ARM64_DSB();

  /* store mpid last as this is our synchronization point */

  cpu_boot_params.mpid = cpu_mpid;

  flush_boot_params();

#ifdef CONFIG_ARCH_HAVE_PSCI
  if (psci_cpu_on(cpu_mpid, (uint64_t)__start))
    {
      sinfo("Failed to boot secondary CPU core %d (MPID:%#lx)\n", cpu_num,
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

#ifdef CONFIG_STACK_COLORATION
  /* If stack debug is enabled, then fill the stack with a
   * recognizable value that we can use later to test for high
   * water marks.
   */

  arm64_stack_color(g_cpu_idlestackalloc[cpu], SMP_STACK_SIZE);
#endif

  cpu_boot_params.cpu_ready_flag = 0;
  arm64_start_cpu(cpu, (char *)g_cpu_idlestackalloc[cpu], SMP_STACK_SIZE,
                  arm64_smp_init_top);

  /* Waiting for this CPU to be boot complete */

  while (!cpu_boot_params.cpu_ready_flag)
    {
      SP_WFE();
      flush_boot_params();
    }

  return 0;
}

/* the C entry of secondary cores */

void arm64_boot_secondary_c_routine(void)
{
  arm64_cpustart_t  func;
  void              *arg;

#ifdef CONFIG_ARCH_HAVE_MPU
  arm64_mpu_init(false);
#endif

#ifdef CONFIG_ARCH_HAVE_MMU
  arm64_mmu_init(false);
#endif

  arm64_gic_secondary_init();

  up_perf_init(NULL);

  up_enable_irq(SGI_CPU_PAUSE);

  func  = cpu_boot_params.func;
  arg   = cpu_boot_params.arg;
  ARM64_DSB();

  /* Secondary core clears .func to announce its presence.
   * Primary core is polling for this. We no longer own
   * arm64_cpu_boot_params afterwards.
   */

  cpu_boot_params.func = NULL;

  ARM64_DSB();
  SP_SEV();

  func(arg);
}

int arm64_smp_sgi_init(void)
{
  irq_attach(SGI_CPU_PAUSE, arm64_pause_handler, 0);
  up_enable_irq(SGI_CPU_PAUSE);

  return 0;
}
