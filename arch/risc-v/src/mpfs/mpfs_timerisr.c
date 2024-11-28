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

#include "riscv_internal.h"

#include "mpfs.h"
#include "mpfs_clockconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TICK_COUNT (MPFS_MSS_RTC_TOGGLE_CLK / TICK_PER_SEC)

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifndef CONFIG_BUILD_KERNEL
static bool _b_tick_started;
static uint64_t *_mtime_cmp;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint64_t get_current(void)
{
#ifndef CONFIG_BUILD_KERNEL
  if (!_b_tick_started)
    {
      _b_tick_started = true;
      return getreg64(MPFS_CLINT_MTIME);
    }
  else
    {
      return getreg64((uintptr_t)_mtime_cmp);
    }
#else
  return riscv_sbi_get_time();
#endif
}

static void set_next(uint64_t next)
{
#ifndef CONFIG_BUILD_KERNEL
  putreg64(next, (uintptr_t)_mtime_cmp);
#else
  riscv_sbi_set_timer(next);
#endif
}

/****************************************************************************
 * Name:  mpfs_reload_mtimecmp
 ****************************************************************************/

static void mpfs_reload_mtimecmp(void)
{
  irqstate_t flags = spin_lock_irqsave(NULL);

  uint64_t current;
  uint64_t next;

  current = get_current();
  uint64_t tick = TICK_COUNT;
  next = current + tick;

  set_next(next);

  spin_unlock_irqrestore(NULL, flags);
}

/****************************************************************************
 * Name:  mpfs_timerisr
 ****************************************************************************/

static int mpfs_timerisr(int irq, void *context, void *arg)
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
#ifndef CONFIG_BUILD_KERNEL
  /* what is our timecmp address for this hart */

  uintptr_t hart_id = riscv_mhartid();
  _mtime_cmp = (uint64_t *)MPFS_CLINT_MTIMECMP0 + hart_id;
#endif

  /* Attach timer interrupt handler */

  irq_attach(RISCV_IRQ_TIMER, mpfs_timerisr, NULL);

  /* Reload CLINT mtimecmp */

  mpfs_reload_mtimecmp();

  /* And enable the timer interrupt */

  up_enable_irq(RISCV_IRQ_TIMER);
}
