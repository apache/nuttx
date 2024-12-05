/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_smpcall.c
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

#include <stdint.h>
#include <assert.h>
#include <debug.h>
#include <string.h>
#include <stdio.h>

#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include <nuttx/sched_note.h>

#include "sched/sched.h"
#include "arm_internal.h"
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

#define CXD56_CPU_P2_INT        (CXD56_SWINT_BASE + 0x8)  /* for APP_DSP0 */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static volatile int g_irq_to_handle[CONFIG_SMP_NCPUS][2];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: handle_irqreq
 *
 * Description:
 *   If an irq handling request is found on cpu, call up_enable_irq() or
 *   up_disable_irq(), then return true.
 *
 * Input Parameters:
 *   cpu - The index of the CPU to be queried
 *
 * Returned Value:
 *   true  = an irq handling request is found
 *   false = no irq handling request is found
 *
 ****************************************************************************/

static bool handle_irqreq(int cpu)
{
  int i;
  bool handled = false;

  /* Check both cases */

  for (i = 0; i < 2; i++)
    {
      int irqreq = g_irq_to_handle[cpu][i];

      if (irqreq)
        {
          /* Clear g_irq_to_handle[cpu][i] */

          g_irq_to_handle[cpu][i] = 0;

          if (0 == i)
            {
              up_enable_irq(irqreq);
            }
          else
            {
              up_disable_irq(irqreq);
            }

          handled = true;

          break;
        }
    }

  return handled;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_smp_call_handler
 *
 * Description:
 *   This is the handler for SMP_CALL.
 *
 ****************************************************************************/

int cxd56_smp_call_handler(int irq, void *c, void *arg)
{
  int cpu = this_cpu();
  int ret = OK;

  handle_irqreq(cpu);

  nxsched_smp_call_handler(irq, c, arg);

  DPRINTF("cpu%d will be paused\n", cpu);

  /* Clear SW_INT for APP_DSP(cpu) */

  putreg32(0, CXD56_CPU_P2_INT + (4 * cpu));

  nxsched_process_delivered(cpu);

  return ret;
}

/****************************************************************************
 * Name: up_send_smp_sched
 *
 * Description:
 *   pause task execution on the CPU
 *   check whether there are tasks delivered to specified cpu
 *   and try to run them.
 *
 * Input Parameters:
 *   cpu - The index of the CPU to be paused.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 * Assumptions:
 *   Called from within a critical section;
 *
 ****************************************************************************/

int up_send_smp_sched(int cpu)
{
  /* Generate IRQ for CPU(cpu) */

  putreg32(1, CXD56_CPU_P2_INT + (4 * cpu));

  return OK;
}

/****************************************************************************
 * Name: up_send_smp_call
 *
 * Description:
 *   Send smp call to target cpu.
 *
 * Input Parameters:
 *   cpuset - The set of CPUs to receive the SGI.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void up_send_smp_call(cpu_set_t cpuset)
{
  int cpu;

  for (; cpuset != 0; cpuset &= ~(1 << cpu))
    {
      cpu = ffs(cpuset) - 1;
      up_send_smp_sched(cpu);
    }
}

/****************************************************************************
 * Name: up_send_irqreq()
 *
 * Description:
 *   Send up_enable_irq() / up_disable_irq() request to the specified cpu
 *
 *   This function is called from up_enable_irq() or up_disable_irq()
 *   to be handled on specified CPU. Locking protocol in the sequence is
 *   the same as up_pause_cpu() plus up_resume_cpu().
 *
 * Input Parameters:
 *   idx - The request index (0: enable, 1: disable)
 *   irq - The IRQ number to be handled
 *   cpu - The index of the CPU which will handle the request
 *
 ****************************************************************************/

void up_send_irqreq(int idx, int irq, int cpu)
{
  DEBUGASSERT(cpu >= 0 && cpu < CONFIG_SMP_NCPUS && cpu != this_cpu());

  /* Set irq for the cpu */

  g_irq_to_handle[cpu][idx] = irq;

  /* Generate IRQ for CPU(cpu) */

  putreg32(1, CXD56_CPU_P2_INT + (4 * cpu));
}

#endif /* CONFIG_SMP */
