/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gd32vw55x_idle.c
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

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/irq.h>
#include <nuttx/power/pm.h>
#include <arch/board/board.h>

#include "riscv_internal.h"
#include "gd32vw55x_pm.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_idlepm
 *
 * Description:
 *   Perform the IDLE state power management: the PM_STANDBY state is the
 *   deep-sleep mode of the GD32VW55x.  Return false when the core still has
 *   to wait for an interrupt.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static bool up_idlepm(void)
{
  static enum pm_state_e oldstate = PM_NORMAL;
  enum pm_state_e newstate;
  irqstate_t flags;
  bool slept = false;
  int ret;

  /* Decide which power saving level can be obtained */

  newstate = pm_checkstate(PM_IDLE_DOMAIN);

  if (newstate == oldstate)
    {
      return false;
    }

  flags = enter_critical_section();

  /* Force the global state change */

  ret = pm_changestate(PM_IDLE_DOMAIN, newstate);
  if (ret < 0)
    {
      /* The new state change failed, revert to the preceding state */

      pm_changestate(PM_IDLE_DOMAIN, oldstate);
    }
  else
    {
      oldstate = newstate;

      if (newstate == PM_STANDBY)
        {
          /* Sleep until the next timer event or an interrupt, then tell
           * the drivers that the system is running again.
           */

          gd32vw55x_pmstop();
          slept = true;

          if (pm_changestate(PM_IDLE_DOMAIN, PM_NORMAL) >= 0)
            {
              oldstate = PM_NORMAL;
            }
        }
    }

  leave_critical_section(flags);
  return slept;
}
#else
#  define up_idlepm() false
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_idle
 *
 * Description:
 *   up_idle() is the logic that will be executed when there is no other
 *   ready-to-run task.  This is processor idle time and will continue until
 *   some interrupt occurs to cause a context switch from the idle task.
 *
 ****************************************************************************/

void up_idle(void)
{
#if defined(CONFIG_SUPPRESS_INTERRUPTS) || defined(CONFIG_SUPPRESS_TIMER_INTS)
  /* If the system is idle and there are no timer interrupts, then process
   * "fake" timer interrupts.  Hopefully, something will wake up.
   */

  nxsched_process_timer();
#else

  /* Perform IDLE mode power management */

  if (up_idlepm())
    {
      return;
    }

#ifdef CONFIG_ARCH_LEDS_CPU_ACTIVITY
  board_autoled_off(LED_CPU);
#endif

  /* Sleep until an interrupt occurs.  With frame pointers, clobbering "ra"
   * makes the compiler keep it in the frame chain that sched_backtrace()
   * follows, as in the common riscv_idle.c.
   */

#if defined(CONFIG_FRAME_POINTER) && defined(CONFIG_SCHED_BACKTRACE)
  asm volatile ("wfi" : : : "ra");
#else
  asm volatile ("wfi");
#endif

#endif
}
