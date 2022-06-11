/****************************************************************************
 * boards/arm/stm32/stm3210e-eval/src/stm32_idle.c
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

#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/power/pm.h>

#include <arch/irq.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "stm32_pm.h"
#include "stm32_rcc.h"
#include "stm32_exti.h"
#include "stm32_rtc.h"

#include "stm3210e-eval.h"

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

/* Values for the RTC Alarm to wake up from the PM_STANDBY mode
 * (which corresponds to STM32 stop mode).  If this alarm expires,
 * the logic in this file will wakeup from PM_STANDBY mode and
 * transition to PM_SLEEP mode (STM32 standby mode).
 */

#ifndef CONFIG_PM_ALARM_SEC
#  define CONFIG_PM_ALARM_SEC 15
#endif

#ifndef CONFIG_PM_ALARM_NSEC
#  define CONFIG_PM_ALARM_NSEC 0
#endif

/* Values for the RTC Alarm to reset from the PM_SLEEP mode (STM32
 * standby mode).  If CONFIG_PM_SLEEP_WAKEUP is defined in the
 * configuration, then the logic in this file will program the RTC
 * alarm to wakeup the processor after an a delay.
 *
 * This feature might be useful, for example, in a system that needs to
 * use minimal power but awake up to perform some task at periodic
 * intervals.
 */

#ifdef CONFIG_PM_SLEEP_WAKEUP

#  ifndef CONFIG_RTC_ALARM
#    error "CONFIG_RTC_ALARM should be enabled to use CONFIG_PM_SLEEP_WAKEUP"
#  endif

/* If CONFIG_PM_SLEEP_WAKEUP is defined, then CONFIG_PM_SLEEP_WAKEUP_SEC
 * and CONFIG_PM_SLEEP_WAKEUP_NSEC define the delay until the STM32
 * awakens from PM_SLEEP mode.
 */

#  ifndef CONFIG_PM_SLEEP_WAKEUP_SEC
#    define CONFIG_PM_SLEEP_WAKEUP_SEC 10
#  endif

#  ifndef CONFIG_PM_SLEEP_WAKEUP_NSEC
#    define CONFIG_PM_SLEEP_WAKEUP_NSEC 0
#  endif
#endif

#define PM_IDLE_DOMAIN 0 /* Revisit */

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_PM) && defined(CONFIG_RTC_ALARM)
static volatile bool g_alarmwakeup;               /* Wakeup Alarm indicator */
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_alarmcb
 *
 * Description:
 *    RTC alarm callback
 *
 ****************************************************************************/

#if defined(CONFIG_PM) && defined(CONFIG_RTC_ALARM)
static void stm32_alarmcb(void)
{
  /* Note that we were awaken by an alarm */

  g_alarmwakeup = true;
}
#endif

/****************************************************************************
 * Name: stm32_alarm_exti
 *
 * Description:
 *    RTC alarm EXTI interrupt service routine
 *
 ****************************************************************************/

#if defined(CONFIG_PM) && defined(CONFIG_RTC_ALARM)
static int stm32_alarm_exti(int irq, void *context, void *arg)
{
  stm32_alarmcb();
  return OK;
}
#endif

/****************************************************************************
 * Name: stm32_exti_cancel
 *
 * Description:
 *    Disable the ALARM EXTI interrupt
 *
 ****************************************************************************/

#if defined(CONFIG_PM) && defined(CONFIG_RTC_ALARM)
static void stm32_exti_cancel(void)
{
  stm32_exti_alarm(false, false, false, NULL, NULL);
}
#endif

/****************************************************************************
 * Name: stm32_rtc_alarm
 *
 * Description:
 *   Set the alarm
 *
 ****************************************************************************/

#if defined(CONFIG_PM) && defined(CONFIG_RTC_ALARM)
static int stm32_rtc_alarm(time_t tv_sec, time_t tv_nsec, bool exti)
{
  struct timespec alarmtime;
  int ret;

  /* Configure to receive RTC Alarm EXTI interrupt */

  if (exti)
    {
      /* TODO: Make sure that that is no pending EXTI interrupt */

      stm32_exti_alarm(true, true, true, stm32_alarm_exti, NULL);
    }

  /* Configure the RTC alarm to Auto Wake the system */

  up_rtc_gettime(&alarmtime);

  alarmtime.tv_sec  += tv_sec;
  alarmtime.tv_nsec += tv_nsec;

  /* The tv_nsec value must not exceed 1,000,000,000. That
   * would be an invalid time.
   */

  if (alarmtime.tv_nsec >= NSEC_PER_SEC)
    {
      /* Carry to the seconds */

      alarmtime.tv_sec++;
      alarmtime.tv_nsec -= NSEC_PER_SEC;
    }

  /* Set the alarm */

  g_alarmwakeup = false;
  ret = stm32_rtc_setalarm(&alarmtime, stm32_alarmcb);
  if (ret < 0)
    {
      serr("ERROR: Warning: The alarm is already set\n");
    }

  return ret;
}
#endif

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
  int ret;

  /* The following is logic that is done after the wake-up from PM_STANDBY
   * state.  It decides whether to go back to the PM_NORMAL or to the deeper
   * power-saving mode PM_SLEEP:  If the alarm expired with no "normal"
   * wake-up event, then PM_SLEEP is entered.
   *
   * Logically, this code belongs at the end of the PM_STANDBY case below,
   * does not work in the position for some unknown reason.
   */

  if (oldstate == PM_STANDBY)
    {
      /* Were we awakened by the alarm? */

#ifdef CONFIG_RTC_ALARM
      if (g_alarmwakeup)
        {
          /* Yes.. Go to SLEEP mode */

          newstate = PM_SLEEP;
        }
      else
#endif
        {
          /* Resume normal operation */

          newstate = PM_NORMAL;
        }
    }
  else
    {
      /* Let the PM system decide, which power saving level can be obtained */

      newstate = pm_checkstate(PM_IDLE_DOMAIN);
    }

  /* Check for state changes */

  if (newstate != oldstate)
    {
      _info("newstate= %d oldstate=%d\n", newstate, oldstate);

      sched_lock();

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
            /* If we just awakened from PM_STANDBY mode, then reconfigure
             * clocking.
             */

            if (oldstate == PM_STANDBY)
              {
                /* Re-enable clocking */

                stm32_clockenable();

                /* The system timer was disabled while in PM_STANDBY or
                 * PM_SLEEP modes.  But the RTC has still be running:  Reset
                 * the system time the current RTC time.
                 */

#ifdef CONFIG_RTC
                clock_synchronize(NULL);
#endif
              }
          }
          break;

        case PM_IDLE:
          {
          }
          break;

        case PM_STANDBY:
          {
            /* Set the alarm as an EXTI Line */

#ifdef CONFIG_RTC_ALARM
            stm32_rtc_alarm(CONFIG_PM_ALARM_SEC, CONFIG_PM_ALARM_NSEC, true);
#endif
            /* Wait 10ms */

            up_mdelay(10);

            /* Enter the STM32 stop mode */

            stm32_pmstop(false);

            /* We have been re-awakened by some even:  A button press?
             * An alarm?  Cancel any pending alarm and resume the normal
             * operation.
             */

#ifdef CONFIG_RTC_ALARM
            stm32_exti_cancel();
            ret = stm32_rtc_cancelalarm();
            if (ret < 0)
              {
                swarn("WARNING: Cancel alarm failed\n");
              }
#endif

            /* Note:  See the additional PM_STANDBY related logic at the
             * beginning of this function.  That logic is executed after
             * this point.
             */
          }
          break;

        case PM_SLEEP:
          {
            /* We should not return from standby mode.  The only way out
             * of standby is via the reset path.
             */

            /* Configure the RTC alarm to Auto Reset the system */

#ifdef CONFIG_PM_SLEEP_WAKEUP
            stm32_rtc_alarm(CONFIG_PM_SLEEP_WAKEUP_SEC,
                            CONFIG_PM_SLEEP_WAKEUP_NSEC, false);
#endif
            /* Wait 10ms */

            up_mdelay(10);

            /* Enter the STM32 standby mode */

            stm32_pmstandby();
          }
          break;

        default:
          break;
        }

      /* Save the new state */

      oldstate = newstate;

errout:
      sched_unlock();
    }
}
#else
#  define stm32_idlepm()
#endif /* CONFIG_PM */

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
