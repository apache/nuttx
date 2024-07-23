/****************************************************************************
 * arch/risc-v/src/jh7110/jh7110_timerisr.c
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
#include <stdint.h>
#include <time.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/init.h>
#include <nuttx/spinlock.h>
#include <nuttx/timers/arch_alarm.h>
#include <arch/board/board.h>

#include "riscv_internal.h"
#include "riscv_mtimer.h"
#include "riscv_percpu.h"
#include "hardware/jh7110_memorymap.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_stimer_pending = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: jh7110_ssoft_interrupt
 *
 * Description:
 *   This function is S-mode software interrupt handler to proceed
 *   the OS timer
 *
 ****************************************************************************/

static int jh7110_ssoft_interrupt(int irq, void *context, void *arg)
{
  /* Cleaer Supervisor Software Interrupt */

  CLEAR_CSR(CSR_SIP, SIP_SSIP);

  if (g_stimer_pending)
    {
      g_stimer_pending = false;

      /* Proceed the OS timer */

      nxsched_process_timer();
    }
#ifdef CONFIG_SMP
  else
    {
      /* We assume IPI has been issued */

      riscv_pause_handler(irq, context, arg);
    }
#endif

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize
 *   the timer interrupt.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  irq_attach(RISCV_IRQ_SSOFT, jh7110_ssoft_interrupt, NULL);
  up_enable_irq(RISCV_IRQ_SSOFT);
}
