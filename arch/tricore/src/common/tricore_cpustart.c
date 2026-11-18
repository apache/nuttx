/****************************************************************************
 * arch/tricore/src/common/tricore_cpustart.c
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

#include <errno.h>
#include <stdint.h>
#include <strings.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/sched.h>
#include <nuttx/sched_note.h>

#include "init/init.h"
#include "sched/sched.h"
#include "tricore_internal.h"
#include "tricore_smp.h"
#include "chip.h"

#ifdef CONFIG_ARCH_CHIP_FAMILY_TC4X
#  include "hardware/tc4x_cpu.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TRICORE_SMP_STACK_ALIGN     8
#define TRICORE_START_PC_HALT       1

#if (CONFIG_ARCH_INTERRUPTSTACK % TRICORE_SMP_STACK_ALIGN) != 0 || \
    (CONFIG_IDLETHREAD_STACKSIZE % TRICORE_SMP_STACK_ALIGN) != 0
#  error "The interrupt and idle stack sizes must be multiples of 8"
#endif

#if CONFIG_ARCH_INTERRUPTSTACK < 4
#  error "SMP needs an interrupt stack for every CPU"
#endif

/****************************************************************************
 * External Definitions
 ****************************************************************************/

/* Entry of a secondary CPU in tricore_head.S */

extern void __start_secondary(void);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_smp_ipi_handler
 *
 * Description:
 *   Inter-CPU interrupt: give back CSA frames if asked to, run the SMP
 *   calls queued for this CPU and pick up the tasks given to it.  The
 *   context switch, if any, happens on the way out of tricore_doirq().
 *
 ****************************************************************************/

static int tricore_smp_ipi_handler(int irq, void *context, void *arg)
{
  tricore_csa_reclaim_ipi();
  nxsched_smp_call_handler(irq, context, arg);
  nxsched_process_delivered(this_cpu());

  return OK;
}

/****************************************************************************
 * Name: tricore_smp_grant_access
 *
 * Description:
 *   On TC4x only CPU0 may write the peripherals and the memories of the
 *   other CPUs after reset.  Let every CPU route and trigger interrupts
 *   to every CPU, use the data scratchpad RAM and the DLMU of every CPU,
 *   where the kernel data and the CSA frames are, and program the STM of
 *   CPU0, which is the system timer: every CPU arms its alarm, so they
 *   all need the STM and the SFRs of CPU0 the STM is in.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_CHIP_FAMILY_TC4X
static void tricore_smp_grant_ranges(uintptr_t prot, uintptr_t cfg,
                                     uintptr_t active, int nranges,
                                     int nwords, uint32_t tags)
{
  uintptr_t range;
  int r;
  int i;

  for (r = 0; r < nranges; r++)
    {
      range = active + r * TC4X_ACCEN_RANGE_SIZE;

      putreg32(TC4X_PROT_SET(r, TC4X_PROT_STATE_RUN), prot);
      putreg32(TC4X_PROT_SET(r, TC4X_PROT_STATE_CONFIG), prot);

      for (i = 0; i < nwords; i++)
        {
          putreg32(getreg32(range + i * sizeof(uint32_t)),
                   cfg + i * sizeof(uint32_t));
        }

      modreg32(tags, tags, cfg + TC4X_ACCEN_WRA);
      putreg32(TC4X_PROT_SET(r, TC4X_PROT_STATE_RUN), prot);
    }
}

static void tricore_smp_grant_access(void)
{
  uint32_t tags = TC4X_CPU_TAGS_ALL(CONFIG_SMP_NCPUS);
  int cpu;

  for (cpu = 0; cpu < CONFIG_SMP_NCPUS; cpu++)
    {
      modreg32(tags, tags, TC4X_IR_ACCENSCFG_WRA(TRICORE_SRC_TOS_CPU(cpu)));
      modreg32(tags, tags, TC4X_IR_ACCENSCTRL_WRA(TRICORE_SRC_TOS_CPU(cpu)));
      modreg32(TC4X_CPU_TAGS(cpu), TC4X_CPU_TAGS(cpu),
               TC4X_IR_ACCENSRB_WRA(cpu));

      tricore_smp_grant_ranges(TC4X_CPU_PROTSPRSE(cpu),
                               TC4X_CPU_ACCENSPRCFG(cpu),
                               TC4X_CPU_ACCENSPR(cpu, 0), TC4X_PROT_RANGES,
                               TC4X_ACCEN_WORDS, tags);
      tricore_smp_grant_ranges(TC4X_CPU_PROTDLMUSE(cpu),
                               TC4X_CPU_ACCENDLMUCFG(cpu),
                               TC4X_CPU_ACCENDLMU(cpu, 0), TC4X_PROT_RANGES,
                               TC4X_ACCEN_WORDS, tags);
    }

  tricore_smp_grant_ranges(TC4X_CPU_PROTSFRSE(0), TC4X_CPU_ACCENSFRCFG(0),
                           TC4X_CPU_ACCENSFR(0), TC4X_PROT_SFR_RANGES,
                           TC4X_ACCEN_SFR_WORDS, tags);

  putreg32(TC4X_PROT_STM(TC4X_PROT_STATE_RUN), TC4X_CPU_PROTSTMSE(0));
  putreg32(TC4X_PROT_STM(TC4X_PROT_STATE_CONFIG), TC4X_CPU_PROTSTMSE(0));
  modreg32(tags, tags, TC4X_CPU_ACCENSTMCFG_WRA(0));
  putreg32(TC4X_PROT_STM(TC4X_PROT_STATE_RUN), TC4X_CPU_PROTSTMSE(0));
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_cpu_idlestack
 *
 * Description:
 *   The idle stack of a secondary CPU is in its own data scratchpad RAM,
 *   above its CSA pool and its interrupt stack.
 *
 ****************************************************************************/

int up_cpu_idlestack(int cpu, struct tcb_s *tcb, size_t stack_size)
{
  uintptr_t top;

  if (tcb == NULL || cpu <= 0 || cpu >= CONFIG_SMP_NCPUS ||
      stack_size > CONFIG_IDLETHREAD_STACKSIZE)
    {
      return -EINVAL;
    }

  top = TRICORE_SMP_CPU_DSPR(cpu) + TRICORE_SMP_USTACK_TOP;

  tcb->stack_alloc_ptr = (void *)(top - CONFIG_IDLETHREAD_STACKSIZE);
  tcb->stack_base_ptr  = tcb->stack_alloc_ptr;
  tcb->adj_stack_size  = CONFIG_IDLETHREAD_STACKSIZE;

  return OK;
}

/****************************************************************************
 * Name: up_cpu_start
 *
 * Description:
 *   Start a secondary CPU at __start_secondary: set its PC and clear its
 *   boot halt bit.
 *
 ****************************************************************************/

int up_cpu_start(int cpu)
{
  uintptr_t pc = (uintptr_t)__start_secondary;

  if (cpu <= 0 || cpu >= CONFIG_SMP_NCPUS)
    {
      return -EINVAL;
    }

  if ((getreg32(TRICORE_START_HALT(cpu)) & TRICORE_START_BHALT) == 0)
    {
      return -EBUSY;
    }

  putreg32((getreg32(TRICORE_START_PC(cpu)) & TRICORE_START_PC_HALT) |
           (pc & ~TRICORE_START_PC_HALT), TRICORE_START_PC(cpu));
  putreg32(getreg32(TRICORE_START_HALT(cpu)) & ~TRICORE_START_BHALT,
           TRICORE_START_HALT(cpu));

  return OK;
}

/****************************************************************************
 * Name: up_send_smp_sched
 *
 * Description:
 *   Send the inter-CPU interrupt to one CPU.
 *
 ****************************************************************************/

int up_send_smp_sched(int cpu)
{
  if (cpu < 0 || cpu >= CONFIG_SMP_NCPUS)
    {
      return -EINVAL;
    }

  up_trigger_irq(TRICORE_SMP_IPI_IRQ(this_cpu(), cpu), (cpu_set_t)1 << cpu);

  return OK;
}

/****************************************************************************
 * Name: up_send_smp_call
 *
 * Description:
 *   Send the inter-CPU interrupt to a set of CPUs.  One write to the
 *   software broadcast register of this CPU raises all of them.
 *
 ****************************************************************************/

void up_send_smp_call(cpu_set_t cpuset)
{
  cpuset &= ((cpu_set_t)1 << CONFIG_SMP_NCPUS) - 1;
  if (cpuset != 0)
    {
      up_trigger_irq(TRICORE_SMP_IPI_IRQ(this_cpu(), this_cpu()), cpuset);
    }
}

/****************************************************************************
 * Name: tricore_smp_initialize
 *
 * Description:
 *   Called on CPU0 before the other CPUs start: give them access to the
 *   shared resources and attach and enable the inter-CPU interrupts of
 *   every CPU.  up_irqinitialize() already routed each GPSR to its CPU.
 *
 ****************************************************************************/

void tricore_smp_initialize(void)
{
  int from;
  int to;

#ifdef CONFIG_ARCH_CHIP_FAMILY_TC4X
  tricore_smp_grant_access();
#endif

  for (to = 0; to < CONFIG_SMP_NCPUS; to++)
    {
      for (from = 0; from < CONFIG_SMP_NCPUS; from++)
        {
          irq_attach(TRICORE_SMP_IPI_IRQ(from, to),
                     tricore_smp_ipi_handler, NULL);
          up_enable_irq(TRICORE_SMP_IPI_IRQ(from, to));
        }
    }
}

/****************************************************************************
 * Name: tricore_cpu_boot
 *
 * Description:
 *   C entry of a secondary CPU, called by __start_secondary on the idle
 *   stack of the CPU with its CSA pool and interrupt stack set up.
 *
 ****************************************************************************/

void tricore_cpu_boot(void)
{
#ifdef CONFIG_ARCH_HAVE_FPU
  tricore_fpuinit();
#endif

#ifdef CONFIG_ARCH_USE_MPU
  tricore_mpuinit();
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION
  sched_note_cpu_started(this_task());
#endif

  up_irq_enable();

  nx_idle_trampoline();
}
