/****************************************************************************
 * arch/arm/src/sam34/sam4cm_cpustart.c
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

#include "nvic.h"
#include "sched/sched.h"
#include "init/init.h"
#include "arm_internal.h"
#include "hardware/sam_pmc.h"
#include "hardware/sam_rstc.h"
#include "hardware/sam4cm_ipc.h"
#include "sam4cm_periphclks.h"

#ifdef CONFIG_SMP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if 0
#  define DPRINTF(fmt, args...) _err(fmt, ##args)
#else
#  define DPRINTF(fmt, args...) do {} while (0)
#endif

#define CPU1_VECTOR_ISTACK  (SAM_INTSRAM1_BASE)
#define CPU1_VECTOR_RESETV  (SAM_INTSRAM1_BASE + 4)

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile static spinlock_t g_cpu1_boot;
extern int arm_pause_handler(int irq, void *c, void *arg);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cpu1_boot
 *
 * Description:
 *   This is the boot vector for CM4P1
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static void cpu1_boot(void)
{
  int cpu;

  /* Disable CMCC1 */

  putreg32(0, 0x48018008);
  while ((getreg32(0x4801800c) & 0x01) != 0);

  cpu = up_cpu_index();
  DPRINTF("cpu = %d\n", cpu);

  if (cpu == 1)
    {
      /* Use CPU0 vectors */

      putreg32((uint32_t)_stext, NVIC_VECTAB);
      sam_ipc1_enableclk();

      /* Clear : write-only */

      putreg32(0x1, SAM_IPC1_ICCR);

      /* Enable : write-only */

      putreg32(0x1, SAM_IPC1_IECR);
      irq_attach(SAM_IRQ_IPC1, arm_pause_handler, NULL);
      up_enable_irq(SAM_IRQ_IPC1);
    }

  spin_unlock(&g_cpu1_boot);

#ifdef CONFIG_SCHED_INSTRUMENTATION
  /* Notify that this CPU has started */

  sched_note_cpu_started(this_task());
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
  struct tcb_s *tcb = current_task(cpu);

  DPRINTF("cpu=%d\n", cpu);

  if (cpu != 1)
    {
      return -EINVAL;
    }

#ifdef CONFIG_SCHED_INSTRUMENTATION
  /* Notify of the start event */

  sched_note_cpu_start(this_task(), cpu);
#endif

  /* Reset coprocessor */

  putreg32(0x5a000000, SAM_RSTC_CPMR);

  /* Enable Coprocessor Bus Master Clock (write-only) */

  putreg32(PMC_CPKEY | PMC_CPBMCK, SAM_PMC_SCER);

  /* Enable Coprocessor Clock (write-only) */

  putreg32(PMC_CPKEY | PMC_CPCK, SAM_PMC_SCER);

  /* Set Coprocessor Clock Prescalar */

  modifyreg32(SAM_PMC_MCKR, PMC_MCKR_CPPRES_MASK, 0);

  /* Set Coprocessor Clock Source */

  modifyreg32(SAM_PMC_MCKR, PMC_MCKR_CPCSS_MASK, PMC_MCKR_CPCSS_PLLB);

  /* Unreset coprocessor pheripheral */

  putreg32(0x5a000010, SAM_RSTC_CPMR);

  /* Enable clock for SRAM1 where CPU1 starts (write-only) */

  putreg32(PMC_PID42, SAM_PMC_PCER1);

  /* Clear SRAM1 */

  memset((void *)SAM_INTSRAM1_BASE, 0, 16 * 1024);

  /* Copy initial vectors for CPU1 */

  putreg32((uint32_t)tcb->stack_base_ptr +
                     tcb->adj_stack_size, CPU1_VECTOR_ISTACK);
  putreg32((uint32_t)cpu1_boot, CPU1_VECTOR_RESETV);

  spin_lock(&g_cpu1_boot);

  /* Unreset coprocessor */

  putreg32(0x5a000011, SAM_RSTC_CPMR);

  /* IRQ setup CPU1->CPU0 */

  sam_ipc0_enableclk();
  putreg32(0x1, SAM_IPC0_ICCR); /* clear : write-only */
  putreg32(0x1, SAM_IPC0_IECR); /* enable : write-only */
  irq_attach(SAM_IRQ_IPC0, arm_pause_handler, NULL);
  up_enable_irq(SAM_IRQ_IPC0);

  spin_lock(&g_cpu1_boot);

  /* CPU1 boot done */

  spin_unlock(&g_cpu1_boot);

  return 0;
}

#endif /* CONFIG_SMP */
