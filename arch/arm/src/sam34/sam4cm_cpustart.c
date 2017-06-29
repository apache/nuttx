/****************************************************************************
 * arch/arm/src/sam34/sam4cm_cpustart.c
 *
 *   Copyright (C) 2016 Masayuki Ishikawa. All rights reserved.
 *   Author: Masayuki Ishikawa <masayuki.ishikawa@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
#include "up_arch.h"
#include "sched/sched.h"
#include "init/init.h"
#include "up_internal.h"
#include "chip/sam_pmc.h"
#include "chip/sam_rstc.h"
#include "chip/sam4cm_ipc.h"
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

#define CPU1_VECTOR_RESETV  (SAM_INTSRAM1_BASE)
#define CPU1_VECTOR_ISTACK  (SAM_INTSRAM1_BASE + 4)

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile static spinlock_t g_cpu1_boot;
extern int arm_pause_handler(int irq, void *c, FAR void *arg);

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

      putreg32((uint32_t)&_stext, NVIC_VECTAB);
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

  (void)os_idle_task(0, NULL);
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
 *   CPU's g_assignedtasks[cpu] list.  Not stack has been alloced or
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
  struct tcb_s *tcb = current_task(cpu);

  DPRINTF("cpu=%d\n",cpu);

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

  putreg32((uint32_t)tcb->adj_stack_ptr, CPU1_VECTOR_RESETV);
  putreg32((uint32_t)cpu1_boot, CPU1_VECTOR_ISTACK);

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
