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
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/spinlock.h>
#include <nuttx/sched_note.h>

#include "sched/sched.h"
#include "xtensa.h"
#include "chip/esp32_dport.h"
#include "chip/esp32_rtccntl.h"
#include "esp32_region.h"
#include "esp32_cpuint.h"
#include "esp32_smp.h"

#ifdef CONFIG_SMP

/****************************************************************************
 * Private Data
 ****************************************************************************/

static volatile bool g_appcpu_started;
static volatile spinlock_t g_appcpu_interlock SP_SECTION;

/****************************************************************************
 * ROM function prototypes
 ****************************************************************************/

void Cache_Flush(int cpu);
void Cache_Read_Enable(int cpu);
void ets_set_appcpu_boot_addr(uint32_t start);

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
 * Name: xtensa_attach_fromcpu0_interrupt
 ****************************************************************************/

#ifdef CONFIG_SMP
static inline void xtensa_attach_fromcpu0_interrupt(void)
{
  int cpuint;

  /* Allocate a level-sensitive, priority 1 CPU interrupt for the UART */

  cpuint = esp32_alloc_levelint(1);
  DEBUGASSERT(cpuint >= 0);

  /* Connect all CPU peripheral source to allocated CPU interrupt */

  up_disable_irq(cpuint);
  esp32_attach_peripheral(1, ESP32_PERIPH_CPU_CPU0, cpuint);

  /* Attach the inter-CPU interrupt. */

  (void)irq_attach(ESP32_IRQ_CPU_CPU0, (xcpt_t)esp32_fromcpu0_interrupt, NULL);

  /* Enable the inter 0 CPU interrupts. */

  up_enable_irq(cpuint);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xtensa_appcpu_start
 *
 * Description:
 *   This is the entry point used with the APP CPU was started  via
 *   up_cpu_start().  The actually start-up logic in in ROM and we boot up
 *   in C code.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None, does not return
 *
 ****************************************************************************/

void xtensa_appcpu_start(void)
{
  FAR struct tcb_s *tcb = this_task();
  register uint32_t sp;

#ifdef CONFIG_STACK_COLORATION
  {
    register uint32_t *ptr;
    register int i;

      /* If stack debug is enabled, then fill the stack with a recognizable value
       * that we can use later to test for high water marks.
       */

      for (i = 0, ptr = (uint32_t *)tcb->stack_alloc_ptr;
           i < tcb->adj_stack_size;
           i += sizeof(uint32_t))
        {
          *ptr++ = STACK_COLOR;
        }
  }
#endif

  /* Move to the stack assigned to us by up_smp_start immediately.  Although
   * we were give a stack pointer at start-up, we don't know where that stack
   * pointer is positioned respect to our memory map.  The only safe option
   * is to switch to a well-known IDLE thread stack.
   */

  sp = (uint32_t)tcb->adj_stack_ptr;
  __asm__ __volatile__("mov sp, %0\n" : : "r"(sp));

  sinfo("CPU%d Started\n", up_cpu_index());

#ifdef CONFIG_SCHED_INSTRUMENTATION
  /* Notify that this CPU has started */

  sched_note_cpu_started(tcb);
#endif

  /* Handle interlock*/

  g_appcpu_started = true;
  spin_unlock(&g_appcpu_interlock);

  /* Reset scheduler parameters */

  sched_resume_scheduler(tcb);

  /* Move CPU0 exception vectors to IRAM */

  asm volatile ("wsr %0, vecbase\n"::"r" (&_init_start));

  /* Make page 0 access raise an exception */

  esp32_region_protection();

  /* Initialize CPU interrupts */

  (void)esp32_cpuint_initialize();

  /* Attach and emable internal interrupts */

#ifdef CONFIG_SMP
  /* Attach and enable the inter-CPU interrupt */

  xtensa_attach_fromcpu0_interrupt();
#endif

#if 0 /* Does it make since to have co-processors enabled on the IDLE thread? */
#if XTENSA_CP_ALLSET != 0
  /* Set initial co-processor state */

  xtensa_coproc_enable(struct xtensa_cpstate_s *cpstate, int cpset);
#endif
#endif

  /* Dump registers so that we can see what is going to happen on return */

  xtensa_registerdump(tcb);

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  /* And Enable interrupts */

  up_irq_enable();
#endif

  /* Then switch contexts. This instantiates the exception context of the
   * tcb at the head of the assigned task list.  In this case, this should
   * be the CPUs NULL task.
   */

  xtensa_context_restore(tcb->xcp.regs);
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

      /* Start CPU1 */

      sinfo("Starting CPU%d\n", cpu);

#ifdef CONFIG_SCHED_INSTRUMENTATION
      /* Notify of the start event */

      sched_note_cpu_start(this_task(), cpu);
#endif

      /* The waitsem semaphore is used for signaling and, hence, should not
       * have priority inheritance enabled.
       */

      spin_initialize(&g_appcpu_interlock, SP_LOCKED);

      /* Flush and enable I-cache for APP CPU */

      Cache_Flush(cpu);
      Cache_Read_Enable(cpu);

      /* Unstall the APP CPU */

      regval  = getreg32(RTC_CNTL_SW_CPU_STALL_REG);
      regval &= ~RTC_CNTL_SW_STALL_APPCPU_C1_M;
      putreg32(regval, RTC_CNTL_SW_CPU_STALL_REG);

      regval  = getreg32(RTC_CNTL_OPTIONS0_REG);
      regval &= ~RTC_CNTL_SW_STALL_APPCPU_C0_M;
      putreg32(regval, RTC_CNTL_OPTIONS0_REG);

      /* Enable clock gating for the APP CPU */

      regval  = getreg32(DPORT_APPCPU_CTRL_B_REG);
      regval |= DPORT_APPCPU_CLKGATE_EN;
      putreg32(regval, DPORT_APPCPU_CTRL_B_REG);

      regval  = getreg32(DPORT_APPCPU_CTRL_C_REG);
      regval &= ~DPORT_APPCPU_RUNSTALL;
      putreg32(regval, DPORT_APPCPU_CTRL_C_REG);

      /* Reset the APP CPU */

      regval  = getreg32(DPORT_APPCPU_CTRL_A_REG);
      regval |= DPORT_APPCPU_RESETTING;
      putreg32(regval, DPORT_APPCPU_CTRL_A_REG);

      regval  = getreg32(DPORT_APPCPU_CTRL_A_REG);
      regval &= ~DPORT_APPCPU_RESETTING;
      putreg32(regval, DPORT_APPCPU_CTRL_A_REG);

      /* Set the CPU1 start address */

      ets_set_appcpu_boot_addr((uint32_t)xtensa_appcpu_start);

      /* And wait for the initial task to run on CPU1 */

      spin_lock(&g_appcpu_interlock);
      DEBUGASSERT(g_appcpu_started);
    }

  return OK;
}

#endif /* CONFIG_SMP */
