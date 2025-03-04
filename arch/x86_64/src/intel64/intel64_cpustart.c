/****************************************************************************
 * arch/x86_64/src/intel64/intel64_cpustart.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <assert.h>
#include <debug.h>

#include <arch/arch.h>
#include <arch/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spinlock.h>

#include "sched/sched.h"
#include "init/init.h"

#include "intel64_lowsetup.h"
#include "intel64_cpu.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * External functions
 ****************************************************************************/

extern void __ap_entry(void);
extern int x86_64_smp_call_handler(int irq, void *c, void *arg);
extern int x86_64_smp_sched_handler(int irq, void *c, void *arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: x86_64_ap_startup
 *
 * Description:
 *   Startup AP CPU
 *
 ****************************************************************************/

static int x86_64_ap_startup(int cpu)
{
  uint64_t dest   = 0;
  uint64_t vect   = 0;
  uint64_t regval = 0;

  sinfo("cpu=%d\n", cpu);

  /* Get destination - must be LOAPIC id */

  dest = MSR_X2APIC_DESTINATION((uint64_t)x86_64_cpu_to_loapic(cpu));

  /* Get the AP trampoline from a fixed address */

  vect = (uint32_t)((uintptr_t)&__ap_entry) >> 12;

  /* Send an INIT IPI to the CPU */

  regval = MSR_X2APIC_ICR_INIT | MSR_X2APIC_ICR_ASSERT
           | MSR_X2APIC_ICR_LEVEL | dest;
  write_msr(MSR_X2APIC_ICR, regval);

  /* Wait for 10 ms */

  up_mdelay(10);
  UP_DMB();

  /* Send an STARTUP IPI to the CPU */

  regval = MSR_X2APIC_ICR_STARTUP | dest | vect;
  write_msr(MSR_X2APIC_ICR, regval);

  /* Wait for AP ready */

  do
    {
      up_udelay(300);
      UP_DMB();
      sinfo("wait for startup cpu=%d...\n", cpu);
    }
  while (x86_64_cpu_ready_get(cpu) == false);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: x86_64_cpu_boot
 *
 * Description:
 *   Boot handler for AP core[cpu]
 *
 * Input Parameters:
 *   lapic_id - The local APIC ID index of the CPU being started.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void x86_64_ap_boot(void)
{
  struct tcb_s *tcb;
  uint8_t cpu = 0;

  /* Do some checking on CPU compatibilities at the top of this function */

  x86_64_check_and_enable_capability();

  /* Reload the GDTR with mapped high memory address */

  setgdt((void *)g_gdt64, (uintptr_t)(&g_gdt64_low_end - &g_gdt64_low) - 1);

  /* Get CPU ID */

  cpu = x86_64_cpu_count_get();

  /* Store CPU private data */

  x86_64_cpu_priv_set(cpu);

  tcb = current_task(cpu);
  UNUSED(tcb);

  /* Configure interrupts */

  up_irqinitialize();

#ifdef CONFIG_SCHED_INSTRUMENTATION
  /* Notify that this CPU has started */

  sched_note_cpu_started(tcb);
#endif

  sinfo("cpu=%d\n", cpu);

  /* Connect Pause IRQ to CPU */

  irq_attach(SMP_IPI_CALL_IRQ, x86_64_smp_call_handler, NULL);
  irq_attach(SMP_IPI_SCHED_IRQ, x86_64_smp_sched_handler, NULL);

  /* NOTE: IPC interrupts don't use IOAPIC but interrupts are sent
   * directly to CPU, so we don't use up_enable_irq() API here.
   */

#ifdef CONFIG_STACK_COLORATION
  /* If stack debug is enabled, then fill the stack with a
   * recognizable value that we can use later to test for high
   * water marks.
   */

  x86_64_stack_color(tcb->stack_alloc_ptr, 0);
#endif

  /* CPU ready */

  x86_64_cpu_ready_set(cpu);

  /* Revoke the lower memory if all CPUs are up */

  if (x86_64_cpu_count_get() >= CONFIG_SMP_NCPUS)
    {
      __revoke_low_memory();
    }

  up_update_task(tcb);

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
  int ret = OK;

  if (cpu != 0)
    {
      /* Startup AP core */

      ret = x86_64_ap_startup(cpu);
    }

  return ret;
}
