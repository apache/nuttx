/****************************************************************************
 * arch/arm/src/lc823450/lc823450_cpustart.c
 *
 *   Copyright 2016,2017 Sony Video & Sound Products Inc.
 *   Author: Masatoshi Tateishi <Masatoshi.Tateishi@jp.sony.com>
 *   Author: Masayuki Ishikawa <Masayuki.Ishikawa@jp.sony.com>
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

#include "up_arch.h"
#include "nvic.h"
#include "sched/sched.h"
#include "init/init.h"
#include "up_internal.h"

#include "lc823450_syscontrol.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if 0
#define DPRINTF(fmt, args...) llinfo(fmt, ##args)
#else
#define DPRINTF(fmt, args...) do {} while (0)
#endif

#define CPU1_VECTOR_RESETV  0x00000000
#define CPU1_VECTOR_ISTACK  0x00000004

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t cpu1_vector_table[];

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern volatile spinlock_t g_cpu_wait[CONFIG_SMP_NCPUS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

extern int lc823450_pause_handler(int irq, void *c, FAR void *arg);

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
      putreg32((uint32_t)&_stext, NVIC_VECTAB); /* use CPU0 vectors */

      irq_attach(LC823450_IRQ_CTXM3_01, lc823450_pause_handler, NULL);
      up_enable_irq(LC823450_IRQ_CTXM3_01);
    }

  spin_unlock(&g_cpu_wait[0]);

#ifdef CONFIG_SCHED_INSTRUMENTATION
  /* Notify that this CPU has started */

  sched_note_cpu_started(this_task());
#endif

  /* Then transfer control to the IDLE task */

  (void)os_idle_task(0, NULL);

}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
  uint32_t backup[2];

  DPRINTF("cpu=%d\n", cpu);

  if (cpu != 1)
    {
      return -1;
    }

  /* create initial vectors for CPU1 */

  putreg32(0x1, REMAP); /* remap enable */
  backup[0] = getreg32(CPU1_VECTOR_RESETV);
  backup[1] = getreg32(CPU1_VECTOR_ISTACK);
  putreg32((uint32_t)tcb->adj_stack_ptr, CPU1_VECTOR_RESETV);
  putreg32((uint32_t)cpu1_boot, CPU1_VECTOR_ISTACK);

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

  putreg32(backup[0], CPU1_VECTOR_RESETV);
  putreg32(backup[1], CPU1_VECTOR_ISTACK);
  putreg32(0x0, REMAP); /* remap disable */

  spin_unlock(&g_cpu_wait[0]);

  return 0;
}
