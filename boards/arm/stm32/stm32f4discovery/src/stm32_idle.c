/****************************************************************************
 * boards/arm/stm32/stm32f4discovery/src/stm32_idle.c
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

#include <arch/board/board.h>
#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/clock.h>
#include <nuttx/power/pm.h>

#include <nuttx/irq.h>

#include "arm_internal.h"
#include "stm32_pm.h"
#include "stm32_rcc.h"
#include "stm32_exti.h"

#include "stm32f4discovery.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Does the board support an IDLE LED to indicate that the board is in the
 * IDLE state?
 */

#if defined(CONFIG_ARCH_LEDS) && defined(LED_IDLE)
#  define BEGIN_IDLE() board_autoled_on(LED_IDLE)
#  define END_IDLE()   board_autoled_off(LED_IDLE)
#else
#  define BEGIN_IDLE()
#  define END_IDLE()
#endif

/* Values for the RTC Alarm to wake up from the PM_STANDBY mode */

#ifndef CONFIG_PM_ALARM_SEC
#  define CONFIG_PM_ALARM_SEC 3
#endif

#ifndef CONFIG_PM_ALARM_NSEC
#  define CONFIG_PM_ALARM_NSEC 0
#endif

#define PM_IDLE_DOMAIN 0 /* Revisit */

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if 0 /* Not used */
static void up_alarmcb(void);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_idlepm
 *
 * Description:
 *   Perform IDLE state power management.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static void stm32_idlepm(void)
{
  static enum pm_state_e oldstate = PM_NORMAL;
  enum pm_state_e newstate;
  irqstate_t flags;
  int ret;

  /* Decide, which power saving level can be obtained */

  newstate = pm_checkstate(PM_IDLE_DOMAIN);

  /* Check for state changes */

  if (newstate != oldstate)
    {
      sinfo("newstate= %d oldstate=%d\n", newstate, oldstate);

      flags = enter_critical_section();

      /* Force the global state change */

      ret = pm_changestate(PM_IDLE_DOMAIN, newstate);
      if (ret < 0)
        {
          /* The new state change failed, revert to the preceding state */

          pm_changestate(PM_IDLE_DOMAIN, oldstate);

          /* No state change... */

          goto errout;
        }

      /* Then perform board-specific, state-dependent logic here */

      switch (newstate)
        {
        case PM_NORMAL:
          {
          }
          break;

        case PM_IDLE:
          {
          }
          break;

        case PM_STANDBY:
          {
#ifdef CONFIG_RTC_ALARM
            /* Disable RTC Alarm interrupt */

#warning "missing logic"

            /* Configure the RTC alarm to Auto Wake the system */

#warning "missing logic"

            /* The tv_nsec value must not exceed 1,000,000,000. That
             * would be an invalid time.
             */

#warning "missing logic"

            /* Set the alarm */

#warning "missing logic"
#endif
            /* Call the STM32 stop mode */

            stm32_pmstop(true);

            /* We have been re-awakened by some even:  A button press?
             * An alarm?  Cancel any pending alarm and resume the normal
             * operation.
             */

#ifdef CONFIG_RTC_ALARM
#warning "missing logic"
#endif
            /* Resume normal operation */

            pm_changestate(PM_IDLE_DOMAIN, PM_NORMAL);
            newstate = PM_NORMAL;
          }
          break;

        case PM_SLEEP:
          {
            /* We should not return from standby mode.  The only way out
             * of standby is via the reset path.
             */

            stm32_pmstandby();
          }
          break;

        default:
          break;
        }

      /* Save the new state */

      oldstate = newstate;

errout:
      leave_critical_section(flags);
    }
}
#else
#  define stm32_idlepm()
#endif

/****************************************************************************
 * Name: up_alarmcb
 *
 * Description:
 *    RTC alarm service routine
 *
 ****************************************************************************/

#if 0 /* Not used */
static void up_alarmcb(void)
{
  /* This alarm occurs because there wasn't any EXTI interrupt during the
   * PM_STANDBY period. So just go to sleep.
   */

  pm_changestate(PM_IDLE_DOMAIN, PM_SLEEP);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_idle
 *
 * Description:
 *   up_idle() is the logic that will be executed when their is no other
 *   ready-to-run task.  This is processor idle time and will continue until
 *   some interrupt occurs to cause a context switch from the idle task.
 *
 *   Processing in this state may be processor-specific. e.g., this is where
 *   power management operations might be performed.
 *
 ****************************************************************************/

void up_idle(void)
{
#if defined(CONFIG_SUPPRESS_INTERRUPTS) || defined(CONFIG_SUPPRESS_TIMER_INTS)
  /* If the system is idle and there are no timer interrupts, then process
   * "fake" timer interrupts. Hopefully, something will wake up.
   */

  nxsched_process_timer();
#else

  /* Perform IDLE mode power management */

  BEGIN_IDLE();
  stm32_idlepm();
  END_IDLE();
#endif
}
