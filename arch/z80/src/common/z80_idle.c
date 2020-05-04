/****************************************************************************
 * arch/z80/src/common/z80_idle.c
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

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "z80_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_ARCH_LEDS) && defined(CONFIG_ARCH_BRINGUP)
static uint8_t g_ledtoggle = 0;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_idle
 *
 * Description:
 *   up_idle() is the logic that will be executed when their
 *   is no other ready-to-run task.  This is processor idle
 *   time and will continue until some interrupt occurs to
 *   cause a context switch from the idle task.
 *
 *   Processing in this state may be processor-specific. e.g.,
 *   this is where power management operations might be
 *   performed.
 *
 ****************************************************************************/

void up_idle(void)
{
#if defined(CONFIG_ARCH_LEDS) && defined(CONFIG_ARCH_BRINGUP)
  g_ledtoggle++;
  if (g_ledtoggle == 0x80)
    {
      board_autoled_on(LED_IDLE);
    }
  else if (g_ledtoggle == 0x00)
    {
      board_autoled_off(LED_IDLE);
    }
#endif

#if defined(CONFIG_SUPPRESS_INTERRUPTS) || defined(CONFIG_SUPPRESS_TIMER_INTS)
  /* If the system is idle and there are no timer interrupts,
   * then process "fake" timer interrupts. Hopefully, something
   * will wake up.
   */

  nxsched_process_timer();
#endif
}
