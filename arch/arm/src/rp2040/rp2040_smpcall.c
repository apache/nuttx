/****************************************************************************
 * arch/arm/src/rp2040/rp2040_smpcall.c
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
#include "hardware/rp2040_sio.h"

#ifdef CONFIG_SMP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if 0
#define DPRINTF(fmt, args...) llinfo(fmt, ##args)
#else
#define DPRINTF(fmt, args...) do {} while (0)
#endif

/****************************************************************************
 * Name: rp2040_handle_irqreq
 *
 * Description:
 *   If an irq handling request is found on cpu, call up_enable_irq() or
 *   up_disable_irq().
 *
 * Input Parameters:
 *   irqreq - The IRQ number to be handled (>0 : enable / <0 : disable)
 *
 ****************************************************************************/

static void rp2040_handle_irqreq(int irqreq)
{
  DEBUGASSERT(this_cpu() == 0);

  if (irqreq > 0)
    {
      up_enable_irq(irqreq);
    }
  else
    {
      up_disable_irq(-irqreq);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_smp_call_handler
 *
 * Description:
 *   This is the handler for SMP_CALL.
 *
 ****************************************************************************/

int rp2040_smp_call_handler(int irq, void *c, void *arg)
{
  int cpu = this_cpu();
  int irqreq;
  uint32_t stat;

  nxsched_smp_call_handler(irq, c, arg);

  stat = getreg32(RP2040_SIO_FIFO_ST);
  if (stat & (RP2040_SIO_FIFO_ST_ROE | RP2040_SIO_FIFO_ST_WOF))
    {
      /* Clear sticky flag */

      putreg32(0, RP2040_SIO_FIFO_ST);
    }

  if (!(stat & RP2040_SIO_FIFO_ST_VLD))
    {
      /* No data received */

      return OK;
    }

  irqreq = getreg32(RP2040_SIO_FIFO_RD);

  if (irqreq != 0)
    {
      /* Handle IRQ enable/disable request */

      rp2040_handle_irqreq(irqreq);
      return OK;
    }

  DPRINTF("cpu%d will be paused\n", cpu);

  nxsched_process_delivered(cpu);

  return OK;
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

  while (!(getreg32(RP2040_SIO_FIFO_ST) & RP2040_SIO_FIFO_ST_RDY))
    ;
  putreg32(0, RP2040_SIO_FIFO_WR);

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
 * Name: rp2040_send_irqreq()
 *
 * Description:
 *   Send up_enable_irq() / up_disable_irq() request to the Core #0
 *
 *   This function is called from up_enable_irq() or up_disable_irq()
 *   to be handled on specified CPU. Locking protocol in the sequence is
 *   the same as up_pause_cpu() plus up_resume_cpu().
 *
 * Input Parameters:
 *   irqreq - The IRQ number to be handled (>0 : enable / <0 : disable)
 *
 ****************************************************************************/

void rp2040_send_irqreq(int irqreq)
{
  /* Send IRQ number to Core #0 */

  while (!(getreg32(RP2040_SIO_FIFO_ST) & RP2040_SIO_FIFO_ST_RDY))
    ;
  putreg32(irqreq, RP2040_SIO_FIFO_WR);
}

#endif /* CONFIG_SMP */
