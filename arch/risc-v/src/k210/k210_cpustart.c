/****************************************************************************
 * arch/risc-v/src/k210/k210_cpustart.c
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

#include "riscv_arch.h"
#include "sched/sched.h"
#include "init/init.h"
#include "riscv_internal.h"
#include "chip.h"

#ifdef CONFIG_SMP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if 0
#  define DPRINTF(fmt, args...) _err(fmt, ##args)
#else
#  define DPRINTF(fmt, args...) do {} while (0)
#endif

#ifdef CONFIG_DEBUG_FEATURES
#  define showprogress(c) riscv_lowputc(c)
#else
#  define showprogress(c)
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern volatile bool g_serial_ok;
extern int riscv_pause_handler(int irq, void *c, FAR void *arg);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: k210_cpu_boot
 *
 * Description:
 *   Boot handler for cpu1
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void k210_cpu_boot(int cpu)
{
  if (1 < cpu)
    {
      return;
    }

  /* Wait for g_serial_ok set by cpu0 when booting */

  while (!g_serial_ok)
    {
    }

  /* Clear machine software interrupt for CPU(cpu) */

  putreg32(0, (uintptr_t)K210_CLINT_MSIP + (4 * cpu));

  /* Enable machine software interrupt for IPI to boot */

  up_enable_irq(K210_IRQ_MSOFT);

  /* Wait interrupt */

  asm("WFI");

  showprogress('b');
  DPRINTF("CPU%d Started\n", this_cpu());

  /* TODO: Setup FPU */

  /* Clear machine software interrupt for CPU(cpu) */

  putreg32(0, (uintptr_t)K210_CLINT_MSIP + (4 * cpu));

#ifdef CONFIG_SCHED_INSTRUMENTATION
  /* Notify that this CPU has started */

  sched_note_cpu_started(this_task());
#endif

  up_irq_enable();

  /* Then transfer control to the IDLE task */

  nx_idle_trampoline();
}

/****************************************************************************
 * Name: up_cpu_start
 *
 * Description:
 *   In an SMP configution, only one CPU is initially active (CPU 0). System
 *   initialization occurs on that single thread. At the completion of the
 *   initialization of the OS, just before beginning normal multitasking,
 *   the additional CPUs would be started by calling this function.
 *
 *   Each CPU is provided the entry point to is IDLE task when started.  A
 *   TCB for each CPU's IDLE task has been initialized and placed in the
 *   CPU's g_assignedtasks[cpu] list.  Not stack has been allocated or
 *   initialized.
 *
 *   The OS initialization logic calls this function repeatedly until each
 *   CPU has been started, 1 through (CONFIG_SMP_NCPUS-1).
 *
 * Input Parameters:
 *   cpu - The index of the CPU being started.  This will be a numeric
 *         value in the range of from one to (CONFIG_SMP_NCPUS-1).  (CPU
 *         0 is already active)
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_cpu_start(int cpu)
{
  DPRINTF("cpu=%d\n", cpu);

#ifdef CONFIG_SCHED_INSTRUMENTATION
  /* Notify of the start event */

  sched_note_cpu_start(this_task(), cpu);
#endif

  /* Send IPI to CPU(cpu) */

  putreg32(1, (uintptr_t)K210_CLINT_MSIP + (cpu * 4));

  return 0;
}

#endif /* CONFIG_SMP */
