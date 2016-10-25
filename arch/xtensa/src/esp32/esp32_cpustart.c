/****************************************************************************
 * arch/xtensa/src/esp32/esp32_cpustart.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
#include <stdbool.h>
#include <semaphore.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>

#include "sched/sched.h"
#include "xtensa.h"
#include "esp32_region.h"

#ifdef CONFIG_SMP

#warning REVISIT Need ets_set_appcpu_boot_addr() prototype
void ets_set_appcpu_boot_addr(uint32_t);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_appcpu_started;
static sem_t g_appcpu_interlock;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xtensa_registerdump
 ****************************************************************************/

#if 0 /* Was useful in solving some startup problems */
static inline void xtensa_registerdump(FAR struct tcb_s *tcb)
{
  _info("CPU%d:\n", up_cpu_index());

  /* Dump the startup registers */
  /* To be provided */
}
#else
# define xtensa_registerdump(tcb)
#endif

/****************************************************************************
 * Name: xtensa_disable_all
 ****************************************************************************/

static inline void xtensa_disable_all(void)
{
  __asm__ __volatile__
  (
    "movi a2, 0\n"
    "xsr a2, INTENABLE\n"
    : : : "a2"
  );
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xtensa_start_handler
 *
 * Description:
 *   This is the handler for SGI1.  This handler simply returns from the
 *   interrupt, restoring the state of the new task at the head of the ready
 *   to run list.
 *
 * Input Parameters:
 *   Standard interrupt handling
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int xtensa_start_handler(int irq, FAR void *context)
{
  FAR struct tcb_s *tcb;

  sinfo("CPU%d Started\n", up_cpu_index());

  /* Handle interlock*/

  g_appcpu_started = true;
  sem_post(&g_appcpu_interlock);

  /* Reset scheduler parameters */

  tcb = this_task();
  sched_resume_scheduler(tcb);

  /* Move CPU0 exception vectors to IRAM */

  asm volatile ("wsr %0, vecbase\n"::"r" (&_init_start));

  /* Make page 0 access raise an exception */

  esp32_region_protection();

  /* Disable all PRO CPU interrupts */

  xtensa_disable_all();

  /* Disable peripheral sources from all PRO CPU interrupt */
#warning Missing logic

  /* Dump registers so that we can see what is going to happen on return */

  xtensa_registerdump(tcb);

  /* Then switch contexts. This instantiates the exception context of the
   * tcb at the head of the assigned task list.  In this case, this should
   * be the CPUs NULL task.
   */

  xtensa_context_restore(tcb->xcp.regs);
  return OK;
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
  DEBUGASSERT(cpu >= 0 && cpu < CONFIG_SMP_NCPUS && cpu != this_cpu());

  if (!g_appcpu_started)
    {
      uint32_t regval;
      int ret;

      /* Start CPU1 */

      sinfo("Starting CPU%d\n", cpu);
      sem_init(&g_appcpu_interlock, 0, 0)

      regval  = getreg32(DPORT_APPCPU_CTRL_B_REG);
      regval |= DPORT_APPCPU_CLKGATE_EN;
      putreg32(regval, DPORT_APPCPU_CTRL_B_REG);

      regval  = getreg32(DPORT_APPCPU_CTRL_C_REG);
      regval ~= DPORT_APPCPU_RUNSTALL;
      putreg32(regval, DPORT_APPCPU_CTRL_C_REG);

      regval  = getreg32(DPORT_APPCPU_CTRL_A_REG);
      regval |= DPORT_APPCPU_RESETTING;
      putreg32(regval, DPORT_APPCPU_CTRL_A_REG);

      regval  = getreg32(DPORT_APPCPU_CTRL_A_REG);
      regval &= ~DPORT_APPCPU_RESETTING;
      putreg32(regval, DPORT_APPCPU_CTRL_A_REG);

      /* Set the CPU1 start address */

      ets_set_appcpu_boot_addr((uint32_t)xtensa_start_handler);

      /* And way for the initial task to run on CPU1 */

      while (!app_cpu_started)
        {
          ret = sem_wait(&g_appcpu_interlock);
          if (ret < 0)
            {
              DEBUGASSERT(errno == EINTR);
            }
        }

      sem_destroy(&g_appcpu_interlock);
    }
}

#endif /* CONFIG_SMP */
