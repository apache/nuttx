/****************************************************************************
 * arch/risc-v/src/jh7100/jh7100_timerisr.c
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
#include <time.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include <arch/board/board.h>

#include "riscv_arch.h"

#include "jh7100.h"
#include "jh7100_clockconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define getreg64(a)   (*(volatile uint64_t *)(a))
#define putreg64(v,a) (*(volatile uint64_t *)(a) = (v))

#ifdef CONFIG_JH7100_WITH_QEMU
#define TICK_COUNT (10000000UL / TICK_PER_SEC)
#else
#define TICK_COUNT ((jh7100_get_cpuclk()) / TICK_PER_SEC)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool _b_tick_started = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  jh7100_reload_mtimecmp
 ****************************************************************************/

static void jh7100_reload_mtimecmp(void)
{
  irqstate_t flags = spin_lock_irqsave(NULL);
  uint64_t current;
  uint64_t next;

  if (!_b_tick_started)
    {
      _b_tick_started = true;
      current = getreg64(JH7100_CLINT_MTIME);
    }
  else
    {
      current = getreg64(JH7100_CLINT_MTIMECMP);
    }

  uint64_t tick = TICK_COUNT;
  next = current + tick;

  putreg64(next, JH7100_CLINT_MTIMECMP);

  spin_unlock_irqrestore(NULL, flags);
}

/****************************************************************************
 * Name:  jh7100_timerisr
 ****************************************************************************/

static int jh7100_timerisr(int irq, void *context, FAR void *arg)
{
  jh7100_reload_mtimecmp();

  /* Process timer interrupt */

  nxsched_process_timer();
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
  /* Attach timer interrupt handler */
<<<<<<< HEAD

=======
>>>>>>> 577374631f (Add rest of BeagleV Starlight JH7100 and JH7110 work.)
  irq_attach(JH7100_IRQ_MTIMER, jh7100_timerisr, NULL);

  /* Reload CLINT mtimecmp */

  jh7100_reload_mtimecmp();

  /* And enable the timer interrupt */

  up_enable_irq(JH7100_IRQ_MTIMER);
}
