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

#include <arch/board/board.h>

#include "mpfs.h"
#include "mpfs_clockconfig.h"

#include "riscv_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TICK_COUNT (MPFS_MSS_RTC_TOGGLE_CLK / TICK_PER_SEC)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  mpfs_timerisr
 ****************************************************************************/

static int mpfs_timerisr(int irq, void *context, void *arg)
{
  /* (Re-)load the timer compare match register */

  riscv_reload_mtimecmp();

  /* Process timer interrupt */

  nxsched_process_timer();

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void mpfs_mtimer_init(void)
{
  riscv_init_mtimer(MPFS_CLINT_MTIMECMP0, MPFS_CLINT_MTIME, TICK_COUNT);
}

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
  mpfs_mtimer_init();

  /* Reload the timer */

  riscv_reload_mtimecmp();

  /* Attach and enable the timer */

  irq_attach(RISCV_IRQ_TIMER, mpfs_timerisr, NULL);
  up_enable_irq(RISCV_IRQ_TIMER);
}
