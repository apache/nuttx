/****************************************************************************
 * arch/arm/src/imxrt/imxrt_hprtc.c
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

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>
#include <nuttx/timers/rtc.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/imxrt_snvs.h"
#include "imxrt_periphclks.h"
#include "imxrt_lpsrtc.h"
#include "imxrt_hprtc.h"

#ifdef CONFIG_IMXRT_SNVS_HPRTC

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Callback to use when the alarm expires */

#if defined(CONFIG_RTC_ALARM) && defined(CONFIG_RTC_DRIVER)
static hprtc_alarm_callback_t g_hprtc_alarmcb;
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Variable determines the state of the RTC module.
 *
 * After initialization value is set to 'true' if RTC starts successfully.
 * The value can be changed to false also during operation if RTC for
 * some reason fails.
 */

volatile bool g_rtc_enabled;

#if !defined(CONFIG_IMXRT_SNVS_LPSRTC) && defined(CONFIG_RTC_DRIVER)
bool g_hprtc_timset;  /* True:  time has been set since power up */
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_snvs_interrupt
 *
 * Description:
 *    RTC interrupt service routine
 *
 * Input Parameters:
 *   irq - The IRQ number that generated the interrupt
 *   context - Architecture specific register save information.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_RTC_ALARM) && defined(CONFIG_RTC_DRIVER)
static int imxrt_snvs_interrupt(int irq, void *context, void *arg)
{
  hprtc_alarm_callback_t cb;

  /* The alarm is the only interrupt enabled */

  DEBUGASSERT((getreg32(IMXRT_SNVS_HPSR) & SNVS_HPSR_HPTA) != 0);

  /* Sample and clear the callback */

  cb              = g_hprtc_alarmcb;
  g_hprtc_alarmcb = NULL;

  /* Disable the alarm, alarm interrupts, clear pending alarm interrupt
   * status
   */

  imxrt_hprtc_alarmdisable();

  /* Perform the callback */

  cb();
  return 0;
}
#endif

/****************************************************************************
 * Name: imxrt_hprtc_enable
 *
 * Description:
 *   Enable/start the HPRTC time counter.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void imxrt_hprtc_enable(void)
{
  uint32_t regval;

  /* Enable the HPRTC */

  regval  = getreg32(IMXRT_SNVS_HPCR);
  regval |= SNVS_HPCR_RTCEN;
  putreg32(regval, IMXRT_SNVS_HPCR);

  while ((getreg32(IMXRT_SNVS_HPCR) & SNVS_HPCR_RTCEN) == 0)
    {
    }
}

/****************************************************************************
 * Name: imxrt_hprtc_alarmenable
 *
 * Description:
 *    Enable alarm interrupts.  This is currently only used internally at the
 *    time that alarm interrupts are enabled.
 *
 * Input Parameters:
 *    None
 *
 * Returned Value:
 *    None
 *
 ****************************************************************************/

#if defined(CONFIG_RTC_ALARM) && defined(CONFIG_RTC_DRIVER)
static void imxrt_hprtc_alarmenable(void)
{
  uint32_t regval;

  /* Enable the alarm */

  regval  = getreg32(IMXRT_SNVS_HPCR);
  regval |= SNVS_HPCR_HPTAEN;
  putreg32(regval, IMXRT_SNVS_HPCR);

  /* Enable alarm interrupts at the NVIC */

  up_enable_irq(IMXRT_IRQ_SNVS);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Functions used only for HPRTC
 ****************************************************************************/

#ifndef CONFIG_IMXRT_SNVS_LPSRTC

/****************************************************************************
 * Name: up_rtc_time
 *
 * Description:
 *   Get the current time in seconds.  This is similar to the standard time()
 *   function.  This interface is only required if the low-resolution
 *   RTC/counter hardware implementation selected.  It is only used by the
 *   RTOS during initialization to set up the system time when CONFIG_RTC
 *   is set but neither CONFIG_RTC_HIRES nor CONFIG_RTC_DATETIME are set.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current time in seconds
 *
 ****************************************************************************/

time_t up_rtc_time(void)
{
  /* Delegate to imxrt_hprtc_time() */

  return imxrt_hprtc_time();
}

/****************************************************************************
 * Name: up_rtc_settime
 *
 * Description:
 *   Set the RTC to the provided time.  All RTC implementations must be able
 *   to set their time based on a standard timespec.
 *
 * Input Parameters:
 *   tp - the time to use
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_settime(const struct timespec *ts)
{
  uint32_t regval;

  DEBUGASSERT(ts != NULL);

  /* Disable the HPRTC */

  regval  = getreg32(IMXRT_SNVS_HPCR);
  regval &= ~SNVS_HPCR_RTCEN;
  putreg32(regval, IMXRT_SNVS_HPCR);

  while ((getreg32(IMXRT_SNVS_HPCR) & SNVS_HPCR_RTCEN) != 0)
    {
    }

  /* Set HPRTC time in seconds.  We could do better by accounting for the
   * ts->tv_nsec unused residual.
   *
   * IMXRT_SNVS_HPTAMR Bits 9-14 = 15-bit MSB of alarm setting.
   * IMXRT_SNVS_HPTALR 32-bit LSB of alarm setting.
   */

  putreg32((uint32_t)ts->tv_sec >> 17, IMXRT_SNVS_HPRTCMR);
  putreg32((uint32_t)ts->tv_sec << 15, IMXRT_SNVS_HPRTCLR);

#ifdef CONFIG_RTC_DRIVER
  /* The time has been set */

  g_hprtc_timset = true;
#endif

  /* Unconditionally re-enable the HPRTC */

  imxrt_hprtc_enable();
  return OK;
}

#endif /* !CONFIG_IMXRT_SNVS_LPSRTC */

/****************************************************************************
 * Logic Common to LPSRTC and HPRTC
 ****************************************************************************/

/****************************************************************************
 * Name: up_rtc_initialize
 *
 * Description:
 *   Initialize the hardware RTC per the selected configuration.
 *   This function is called once during the OS initialization sequence
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_initialize(void)
{
  int ret;

#ifdef CONFIG_IMXRT_SNVS_LPSRTC
  /* Perform LPSRTC initialization */

  ret = imxrt_lpsrtc_initialize();
  if (ret < 0)
    {
      rtcerr("ERROR: imxrt_lpsrtc_initialize failed: %d\n", ret);
      return ret;
    }
#else
  /* Initialize the HPRTC */

  ret = imxrt_hprtc_initialize();
  if (ret < 0)
    {
      rtcerr("ERROR: imxrt_hprtc_initialize failed: %d\n", ret);
      return ret;
    }

  /* Start the HPRTC */

  imxrt_hprtc_enable();
#endif

#if defined(CONFIG_IMXRT_SNVS_HPRTC) && defined(CONFIG_RTC_ALARM) && \
    defined(CONFIG_RTC_DRIVER)
  /* Attach the alarm interrupt handler */

  ret = irq_attach(IMXRT_IRQ_SNVS, imxrt_snvs_interrupt, NULL);
  if (ret < 0)
    {
      rtcerr("ERROR: Failed to attach to IRQ%d\n", IMXRT_IRQ_SNVS);
      return ret;
    }
#endif

  g_rtc_enabled = true;
  return OK;
}

/****************************************************************************
 * Name: imxrt_hprtc_initialize
 *
 * Description:
 *   Initialize the LPSRTC per the selected configuration.  This function
 *   is called via up_rtc_initialize (see imxrt_hprtc.c).
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int imxrt_hprtc_initialize(void)
{
  uint32_t regval;

  /* Enable clocking to the the SNVS HP module.
   * Clock is on in run mode, but off in WAIT and STOP modes.
   */

  imxrt_clockrun_snvs_hp();

  /* Enable non-privileged access */

  regval  = getreg32(IMXRT_SNVS_HPCOMR);
  regval |= (SNVS_HPCOMR_NPSWAEN | SNVS_HPCOMR_SWSV);
  putreg32(regval, IMXRT_SNVS_HPCOMR);

  /* TODO: Set the periodic interrupt frequency */

  putreg32(SNVS_HPCR_PIFREQ(0), IMXRT_SNVS_HPCR);

#ifdef CONFIG_IMXRTC_HPRTC_CALENABLE
  /* Set the HPRTC calibration value */

  regval  = getreg32(IMXRT_SNVS_HPCR);
  regval &= ~SNVS_HPCR_HPCALBVAL_MASK;
  regval |= SNVS_HPCR_HPCALBVAL(CONFIG_IMXRTC_HPRTC_CALVALUE);
  regval |= SNVS_HPCR_HPCALBEN;
  putreg32(regval, IMXRT_SNVS_HPCR);
#endif

  return OK;
}

/****************************************************************************
 * Name: imxrt_hprtc_synchronize
 *
 * Description:
 *   Synchronize the HPRTC to the LPSRTC and enable the HPRTC timer.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_IMXRT_SNVS_LPSRTC
void imxrt_hprtc_synchronize(void)
{
  uint32_t regval;
  uint32_t hpcr;

  /* Make sure that the RTC is disabled (it should be at this point in the
   * LPSRTC initialization sequence).
   */

  hpcr    = getreg32(IMXRT_SNVS_HPCR);
  regval  = hpcr;
  regval &= ~SNVS_HPCR_RTCEN;
  putreg32(regval, IMXRT_SNVS_HPCR);

  while ((getreg32(IMXRT_SNVS_HPCR) & SNVS_HPCR_RTCEN) != 0)
    {
    }

  /* Synchronize to the LPSRTC */

  regval  = getreg32(IMXRT_SNVS_HPCR);
  regval |= SNVS_HPCR_HPTS;
  putreg32(regval, IMXRT_SNVS_HPCR);

  /* Unconditionally enable the HPRTC */

  imxrt_hprtc_enable();
}
#endif

/****************************************************************************
 * Name: imxrt_hprtc_time
 *
 * Description:
 *   Get the current time in seconds.  This is the underlying implementation
 *   of the up_rtc_time() function that is used by the RTOS during
 *   initialization to set up the system time.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current time in seconds
 *
 ****************************************************************************/

uint32_t imxrt_hprtc_time(void)
{
  uint32_t seconds;
  uint32_t verify = 0;

  /* Do consecutive reads until value is correct */

  do
    {
      /* IMXRT_SNVS_HPRTCMR: Bits 9-14 = 15-bit MSB of counter.
       * IMXRT_SNVS_HPRTCLR: 32-bit LSB of counter.
       *
       * REVISIT:  This could be modified to support CONFIG_RTC_HI_RES
       */

      seconds = verify;
      verify  = (getreg32(IMXRT_SNVS_HPRTCMR) << 17) |
                (getreg32(IMXRT_SNVS_HPRTCLR) >> 15);
    }
  while (verify != seconds);

  return seconds;
}

/****************************************************************************
 * Name: imxrt_hprtc_getalarm
 *
 * Description:
 *   Get the current alarm setting in seconds.
 *   This is only used by the lower half RTC driver.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current alarm setting in seconds
 *
 ****************************************************************************/

#if defined(CONFIG_RTC_ALARM) && defined(CONFIG_RTC_DRIVER)
uint32_t imxrt_hprtc_getalarm(void)
{
  /* Return the alarm setting in seconds
   *
   * IMXRT_SNVS_HPTAMR Bits 9-14 = 15-bit MSB of alarm setting.
   * IMXRT_SNVS_HPTALR 32-bit LSB of alarm setting.
   */

  return (getreg32(IMXRT_SNVS_HPTAMR) << 17) |
         (getreg32(IMXRT_SNVS_HPTALR) >> 15);
}
#endif

/****************************************************************************
 * Name: imxrt_hprtc_setalarm
 *
 * Description:
 *   Set the alarm (in seconds) and enable alarm interrupts.
 *   This is only used by the lower half RTC driver.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current alarm setting in seconds
 *
 ****************************************************************************/

#if defined(CONFIG_RTC_ALARM) && defined(CONFIG_RTC_DRIVER)
int imxrt_hprtc_setalarm(struct timespec *ts, hprtc_alarm_callback_t cb)
{
  irqstate_t flags;
  uint32_t regval;
  uint32_t now;

  DEBUGASSERT(ts != NULL && cb != NULL);

  /* Disable interrupts so that the following sequence of events will not be
   * interrupted or preempted.
   */

  flags = spin_lock_irqsave(NULL);

  now = imxrt_hprtc_time();

  /* Return error if the alarm time has passed.
   * NOTES:  (1) This will fail, of course, when the number of seconds since
   * epoch wraps. (2) We could do better by accounting for the ts->tv_nsec
   * unused residual.
   */

  if ((uint32_t)ts->tv_sec <= now)
    {
      rtcwarn("WARNING: time is in the past\n");
      spin_unlock_irqrestore(NULL, flags);
      return -EINVAL;
    }

  /* Disable the RTC alarm interrupt */

  regval  = getreg32(IMXRT_SNVS_HPCR);
  regval  &= ~SNVS_HPCR_HPTAEN;
  putreg32(regval, IMXRT_SNVS_HPCR);

  while ((getreg32(IMXRT_SNVS_HPCR) & SNVS_HPCR_HPTAEN) != 0)
    {
    }

  /* Save the alarm callback */

  g_hprtc_alarmcb = cb;

  /* Set alarm in seconds
   *
   * IMXRT_SNVS_HPTAMR Bits 9-14 = 15-bit MSB of alarm setting.
   * IMXRT_SNVS_HPTALR 32-bit LSB of alarm setting.
   */

  putreg32((uint32_t)ts->tv_sec >> 17, IMXRT_SNVS_HPTAMR);
  putreg32((uint32_t)ts->tv_sec << 15, IMXRT_SNVS_HPTALR);

  /* Unconditionally enable the RTC alarm interrupt */

  imxrt_hprtc_alarmenable();
  spin_unlock_irqrestore(NULL, flags);
  return OK;
}
#endif

/****************************************************************************
 * Name: imxrt_hprtc_alarmdisable
 *
 * Description:
 *    Disable alarm interrupts.  Used internally after the receipt of the
 *    alarm interrupt.  Also called by the lower-half RTC driver in order to
 *    cancel an alarm.
 *
 * Input Parameters:
 *    None
 *
 * Returned Value:
 *    None
 *
 ****************************************************************************/

#if defined(CONFIG_RTC_ALARM) && defined(CONFIG_RTC_DRIVER)
void imxrt_hprtc_alarmdisable(void)
{
  uint32_t regval;

  /* Disable alarm interrupts at the NVIC */

  up_disable_irq(IMXRT_IRQ_SNVS);

  /* Disable the alarm function */

  regval  = getreg32(IMXRT_SNVS_HPCR);
  regval &= ~SNVS_HPCR_HPTAEN;
  putreg32(regval, IMXRT_SNVS_HPCR);

  /* Clear any pending alarm interrupts */

  putreg32(SNVS_HPSR_HPTA, IMXRT_SNVS_HPSR);
}
#endif

#endif /* CONFIG_IMXRT_SNVS_HPRTC */
