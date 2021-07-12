/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_timerisr.c
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
#include <arch/board/board.h>
#include <nuttx/spinlock.h>

#include "riscv_arch.h"

#include "mpfs.h"
#include "mpfs_clockconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TICK_COUNT (MPFS_MSS_RTC_TOGGLE_CLK / TICK_PER_SEC)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool _b_tick_started = false;
static uint64_t *_mtime_cmp = 0L;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  mpfs_reload_mtimecmp
 ****************************************************************************/

static void mpfs_reload_mtimecmp(void)
{
  irqstate_t flags = spin_lock_irqsave(NULL);

  uint64_t current;
  uint64_t next;

  if (!_b_tick_started)
    {
      _b_tick_started = true;
      current = getreg64(MPFS_CLINT_MTIME);
    }
  else
    {
      current = getreg64(_mtime_cmp);
    }

  uint64_t tick = TICK_COUNT;
  next = current + tick;

  putreg64(next, _mtime_cmp);

  spin_unlock_irqrestore(NULL, flags);
}

/****************************************************************************
 * Name:  mpfs_timerisr
 ****************************************************************************/

static int mpfs_timerisr(int irq, void *context, FAR void *arg)
{
  mpfs_reload_mtimecmp();

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
  /* what is our timecmp address for this hart */

  uint64_t hart_id = READ_CSR(mhartid);
  _mtime_cmp = (uint64_t *)MPFS_CLINT_MTIMECMP0 + hart_id;

  /* Attach timer interrupt handler */

  irq_attach(MPFS_IRQ_MTIMER, mpfs_timerisr, NULL);

  /* Reload CLINT mtimecmp */

  mpfs_reload_mtimecmp();

  /* And enable the timer interrupt */

  up_enable_irq(MPFS_IRQ_MTIMER);
}
