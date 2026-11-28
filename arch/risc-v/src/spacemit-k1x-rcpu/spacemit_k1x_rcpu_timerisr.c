/****************************************************************************
 * arch/risc-v/src/spacemit-k1x-rcpu/spacemit_k1x_rcpu_timerisr.c
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

#include <nuttx/arch.h>
#include <nuttx/timers/arch_alarm.h>

#include <arch/irq.h>

#include "riscv_internal.h"
#include "riscv_mtimer.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The Nuclei machine timer keeps mtime and mtimecmp eight bytes apart */

#define SPACEMIT_K1X_RCPU_MTIME_OFFSET            0x0000
#define SPACEMIT_K1X_RCPU_MTIMECMP_OFFSET         0x0008

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_timer_initialize
 *
 * Description:
 *   Start the NuttX system tick.
 *
 *   The tick is driven by the RCPU private machine timer, which the core
 *   places inside the Nuclei IRegion at a run time discovered base.  The
 *   generic oneshot lower half takes care of programming mtimecmp, so all
 *   this has to do is hand it the two registers, the interrupt and the
 *   counter frequency.
 *
 * The frequency is measured rather than assumed; see
 * SPACEMIT_K1X_RCPU_SYSTIMER_FREQ in spacemit_k1x_rcpu_memorymap.h and the
 * help text of CONFIG_SPACEMIT_K1X_RCPU_SYSTIMER_FREQ.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  uintptr_t base = spacemit_k1x_rcpu_systimer_base();
  struct oneshot_lowerhalf_s *lower;

  lower = riscv_mtimer_initialize(base + SPACEMIT_K1X_RCPU_MTIME_OFFSET,
                                  base + SPACEMIT_K1X_RCPU_MTIMECMP_OFFSET,
                                  RISCV_IRQ_MTIMER,
                                  CONFIG_SPACEMIT_K1X_RCPU_SYSTIMER_FREQ);

  DEBUGASSERT(lower != NULL);

  up_alarm_set_lowerhalf(lower);
}
