/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_cpustart.c
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
#include "hardware/cxd56_crg.h"
#include "hardware/cxd5602_memorymap.h"

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
#  define showprogress(c) arm_lowputc(c)
#else
#  define showprogress(c)
#endif

#define CXD56_ACNV_P0_DST0   0x0e012004
#define CXD56_CPU_P2_INT     (CXD56_SWINT_BASE + 0x8)  /* for APP_DSP0 */

#define VECTOR_ISTACK        (CXD56_ADSP_RAM_BASE + 0)
#define VECTOR_RESETV        (CXD56_ADSP_RAM_BASE + 4)

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile static spinlock_t g_appdsp_boot;

extern int arm_pause_handler(int irq, void *c, void *arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: appdsp_boot
 *
 * Description:
 *   This is the boot vector for APP_DSP
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static void appdsp_boot(void)
{
  int cpu;

  cpu = up_cpu_index();
  DPRINTF("cpu = %d\n", cpu);

  /* Setup NVIC */

  up_irqinitialize();

  /* Setup FPU */

  arm_fpuconfig();

  /* Clear SW_INT for APP_DSP(cpu) */

  putreg32(0, CXD56_CPU_P2_INT + (4 * cpu));

  /* Enable SW_INT */

  irq_attach(CXD56_IRQ_SW_INT, arm_pause_handler, NULL);
  up_enable_irq(CXD56_IRQ_SW_INT);

  spin_unlock(&g_appdsp_boot);

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
  int i;
  struct tcb_s *tcb = current_task(cpu);

  DPRINTF("cpu=%d\n", cpu);

#ifdef CONFIG_SCHED_INSTRUMENTATION
  /* Notify of the start event */

  sched_note_cpu_start(this_task(), cpu);
#endif

  /* Reset APP_DSP(cpu) */

  modifyreg32(CXD56_CRG_RESET, 1 << (16 + cpu), 0);

  /* Copy initial stack and reset vector for APP_DSP */

  putreg32((uint32_t)tcb->stack_base_ptr +
                     tcb->adj_stack_size, VECTOR_ISTACK);
  putreg32((uint32_t)appdsp_boot, VECTOR_RESETV);

  spin_lock(&g_appdsp_boot);

  /* See 3.13.4.16.3 ADSP Startup */

  /* 2. Clock supply */

  modifyreg32(CXD56_CRG_CK_GATE_AHB, 0, 1 << (16 + cpu));

  /* 3. Clock stop */

  modifyreg32(CXD56_CRG_CK_GATE_AHB, 1 << (16 + cpu), 0);

  /* 4. APP_DSP(cpu) start preparation */

  /* Copy APP_DSP0 settings to all 12 tiles for APP_DSP(cpu)
   * TODO: need to exclude memory areas for AMP
   */

  for (i = 0; i < 12; i++)
    {
      uint32_t val = getreg32(CXD56_ACNV_P0_DST0 + (4 * i));
      putreg32(val, CXD56_ACNV_P0_DST0 + (4 * i) + (cpu * 0x20));
    }

  /* 5. Reset release */

  modifyreg32(CXD56_CRG_RESET, 0, 1 << (16 + cpu));

  /* 6. Clock supply */

  modifyreg32(CXD56_CRG_CK_GATE_AHB, 0, 1 << (16 + cpu));

  /* Setup SW_INT for APP_DSP0. The caller is APP_DSP0 and
   * it's enough to setup only once. So, here this setup is
   * done in case that we boot APP_DSP1 (cpu=1).
   */

  if (1 == cpu)
    {
      /* Clear SW_INT for this APP_DSP0 */

      putreg32(0, CXD56_CPU_P2_INT);

      /* Setup SW_INT for this APP_DSP0 */

      irq_attach(CXD56_IRQ_SW_INT, arm_pause_handler, NULL);
      up_enable_irq(CXD56_IRQ_SW_INT);
    }

  spin_lock(&g_appdsp_boot);

  /* APP_DSP(cpu) boot done */

  spin_unlock(&g_appdsp_boot);

  return 0;
}

#endif /* CONFIG_SMP */
