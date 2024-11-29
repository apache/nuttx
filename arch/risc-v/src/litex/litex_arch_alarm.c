/****************************************************************************
 * arch/risc-v/src/litex/litex_arch_alarm.c
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

#include <assert.h>
#include <nuttx/config.h>
#include <nuttx/timers/arch_alarm.h>

#include "chip.h"
#include "litex_clockconfig.h"
#include "riscv_internal.h"
#include "riscv_mtimer.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  up_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize
 *   the timer interrupt.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  struct oneshot_lowerhalf_s *lower
      = riscv_mtimer_initialize(LITEX_CLINT_MTIME, LITEX_CLINT_MTIMECMP,
                                RISCV_IRQ_TIMER, litex_get_hfclk());

  DEBUGASSERT(lower);

#ifdef CONFIG_ARCH_HAVE_PERF_EVENTS
  up_perf_init((void *)CONFIG_LITEX_SYS_CORE_FREQ_HZ);
#endif

  up_alarm_set_lowerhalf(lower);
}
