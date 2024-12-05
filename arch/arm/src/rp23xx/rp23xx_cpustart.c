/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_cpustart.c
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
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include <nuttx/sched_note.h>

#include "nvic.h"
#include "sched/sched.h"
#include "init/init.h"
#include "arm_internal.h"
#include "hardware/rp23xx_memorymap.h"
#include "hardware/rp23xx_sio.h"
#include "hardware/rp23xx_psm.h"

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

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile static spinlock_t g_core1_boot;

extern int rp23xx_smp_call_handler(int irq, void *c, void *arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fifo_drain
 *
 * Description:
 *   Drain all data in the inter-processor FIFO
 ****************************************************************************/

static void fifo_drain(void)
{
  putreg32(0, RP23XX_SIO_FIFO_ST);

  while (getreg32(RP23XX_SIO_FIFO_ST) & RP23XX_SIO_FIFO_ST_VLD)
    {
      getreg32(RP23XX_SIO_FIFO_RD);
    }

  __asm__ volatile ("sev");
}

/****************************************************************************
 * Name: fifo_comm
 *
 * Description:
 *   Communicate with CPU Core 1 using inter-processor FIFO for boot
 *
 * Input Parameters:
 *   msg - Data to be sent to Core 1
 *
 * Returned Value:
 *   true on success; false on failure.
 *
 ****************************************************************************/

static int fifo_comm(uint32_t msg)
{
  uint32_t rcv;

  while (!(getreg32(RP23XX_SIO_FIFO_ST) & RP23XX_SIO_FIFO_ST_RDY))
    ;
  putreg32(msg, RP23XX_SIO_FIFO_WR);
  __asm__ volatile ("sev");

  while (!(getreg32(RP23XX_SIO_FIFO_ST) & RP23XX_SIO_FIFO_ST_VLD))
    __asm__ volatile ("wfe");

  rcv = getreg32(RP23XX_SIO_FIFO_RD);

  return msg == rcv;
}

/****************************************************************************
 * Name: core1_boot
 *
 * Description:
 *   This is the boot vector for Core #1
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static void core1_boot(void)
{
#if CONFIG_ARCH_INTERRUPTSTACK > 3
  /* Initializes the stack pointer */

  arm_initialize_stack();
#endif

  fifo_drain();

  /* Setup NVIC */

  up_irqinitialize();

  /* Enable inter-processor FIFO interrupt */

  irq_attach(RP23XX_SIO_IRQ_FIFO, rp23xx_smp_call_handler, NULL);
  up_enable_irq(RP23XX_SIO_IRQ_FIFO);

  spin_unlock(&g_core1_boot);

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
  uint32_t core1_boot_msg[5];

  DPRINTF("cpu=%d\n", cpu);

#ifdef CONFIG_SCHED_INSTRUMENTATION
  /* Notify of the start event */

  sched_note_cpu_start(this_task(), cpu);
#endif

  /* Reset Core 1 */

  setbits_reg32(RP23XX_PSM_PROC1, RP23XX_PSM_FRCE_OFF);
  while (!(getreg32(RP23XX_PSM_FRCE_OFF) & RP23XX_PSM_PROC1))
    ;
  clrbits_reg32(RP23XX_PSM_PROC1, RP23XX_PSM_FRCE_OFF);

  spin_lock(&g_core1_boot);

  /* Send initial VTOR, MSP, PC for Core 1 boot */

  core1_boot_msg[0] = 0;
  core1_boot_msg[1] = 1;
  core1_boot_msg[2] = getreg32(NVIC_VECTAB);
  core1_boot_msg[3] = (uint32_t)tcb->stack_base_ptr +
                                tcb->adj_stack_size;
  core1_boot_msg[4] = (uint32_t)core1_boot;

  do
    {
      fifo_drain();
      for (i = 0; i < 5; i++)
        {
          if (!fifo_comm(core1_boot_msg[i]))
            {
              break;
            }
        }
    }
  while (i < 5);

  fifo_drain();

  /* Enable inter-processor FIFO interrupt */

  irq_attach(RP23XX_SIO_IRQ_FIFO, rp23xx_smp_call_handler, NULL);
  up_enable_irq(RP23XX_SIO_IRQ_FIFO);

  spin_lock(&g_core1_boot);

  /* CPU Core 1 boot done */

  spin_unlock(&g_core1_boot);

  return 0;
}

#endif /* CONFIG_SMP */
