/****************************************************************************
 * arch/risc-v/src/rp23xx-rv/rp23xx_timerisr.c
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
#include <time.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/timers/arch_alarm.h>
#include <arch/board/board.h>

#include "clock/clock.h"
#include "riscv_internal.h"
#include "riscv_mtimer.h"
#include "chip.h"
#include "hardware/rp23xx_memorymap.h"
#include "hardware/rp23xx_sio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
  putreg32(0, RP23XX_SIO_BASE + RP23XX_SIO_MTIME_CTRL_OFFSET);
  putreg32(0, RP23XX_SIO_BASE + RP23XX_SIO_MTIME_OFFSET);
  putreg32(0, RP23XX_SIO_BASE + RP23XX_SIO_MTIMEH_OFFSET);
  putreg32(RP23XX_SIO_MTIMECMP_MASK,
           RP23XX_SIO_BASE + RP23XX_SIO_MTIMECMP_OFFSET);
  putreg32(RP23XX_SIO_MTIMECMPH_MASK,
           RP23XX_SIO_BASE + RP23XX_SIO_MTIMECMPH_OFFSET);

  struct oneshot_lowerhalf_s *lower = riscv_mtimer_initialize(
    RP23XX_SIO_BASE + RP23XX_SIO_MTIME_OFFSET,
    RP23XX_SIO_BASE + RP23XX_SIO_MTIMECMP_OFFSET,
    RISCV_IRQ_MTIMER, BOARD_SYS_FREQ);

  DEBUGASSERT(lower);

  up_alarm_set_lowerhalf(lower);

  putreg32(RP23XX_SIO_MTIME_CTRL_EN | RP23XX_SIO_MTIME_CTRL_FULLSPEED,
           RP23XX_SIO_BASE + RP23XX_SIO_MTIME_CTRL_OFFSET);
}
