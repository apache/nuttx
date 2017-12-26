/****************************************************************************
 * arch/arm/src/lpc54/lpc54_rtc.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
#include <time.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/timers/rtc.h>

#include <arch/board/board.h>

#include "up_arch.h"

#include "chip/lpc54_rtc.h"
#include "lpc54_enableclk.h"
#include "lpc54_rtc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifdef CONFIG_RTC_HIRES
#    error "CONFIG_RTC_HIRES is not supported"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
/* Callback to use when the alarm expires */

static alarmcb_t g_alarmcb;
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Variable determines if the RTC has been initialized and enabled. */

volatile bool g_rtc_enabled = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_rtc_interrupt
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

#ifdef CONFIG_RTC_ALARM
static int lpc54_rtc_interrupt(int irq, void *context, FAR void *arg)
{
  uint32_t status = getreg16(LPC54_RTC_CTRL);

  if ((status & RTC_CTRL_ALARM1HZ) != 0 && g_alarmcb != NULL)
    {
      /* Clear pending status */

      putreg32(status | RTC_CTRL_ALARM1HZ | RTC_CTRL_WAKE1KHZ, LPC54_RTC_CTRL);

      /* Perform the alarm callback */

      g_alarmcb();
      g_alarmcb = NULL;
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_rtc_initialize
 *
 * Description:
 *   Initialize the hardware RTC per the selected configuration.  This function is
 *   called once during the OS initialization sequence
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
  /* Enable the clock to the RTC register interface and peripheral clock. */

  lpc54_rtc_enableclk();

  /* If the 32 kHz output of the RTC is used by another part of the system, enable it
   * via the EN bit in the RTCOSCCTRL register
   */

  putreg32(SYSCON_RTCOSCCTRL_EN, LPC54_SYSCON_RTCOSCCTRL);

  /* The RTC is already running or, perhaps waiting to be enabled if it was never
   * configured.  We will set enable the RTC only if the time if initialized by
   * higher level logic.
   */

  g_rtc_enabled = true;
  return OK;
}

/****************************************************************************
 * Name: up_rtc_time
 *
 * Description:
 *   Get the current time in seconds.  This is similar to the standard time()
 *   function.  This interface is only required if the low-resolution RTC/counter
 *   hardware implementation selected.  It is only used by the RTOS during
 *   initialization to set up the system time when CONFIG_RTC is set.
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
  /* Read and return the 32-bit 1Hz RTC counter value */

  return getreg32(LPC54_RTC_COUNT);
}

/****************************************************************************
 * Name: up_rtc_settime
 *
 * Description:
 *   Set the RTC to the provided time.  All RTC implementations must be
 *   able to set their time based on a standard timespec.
 *
 * Input Parameters:
 *   tp - the time to use
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_settime(FAR const struct timespec *tp)
{
  irqstate_t flags;
  uint32_t regval;

  /* Read the Break out the time values */

  flags = enter_critical_section();

  /* Make sure the the RTC is out of reset, but not enabled. */

  regval  = getreg32(LPC54_RTC_CTRL);
  regval &= ~(RTC_CTRL_SWRESET | RTC_CTRL_RTCEN | RTC_CTRL_RTC1KHZEN |
              RTC_CTRL_OSCPD);
  putreg32(regval, LPC54_RTC_CTRL);

  /* Then write the time in seconds to the counter register.  NOTE that we
   * can only write to this register when the RTC_EN bit in the RTC CTRL
   * Register is 0.
   */

  putreg32(tp->tv_sec, LPC54_RTC_COUNT);

  /* (Re-)enabled the RTC.  The counter increments one second after the
   * RTC_EN bit is set.
   */

  regval |= RTC_CTRL_RTCEN;
  putreg32(regval, LPC54_RTC_CTRL);

  /* Make sure that magic value is in the general purpose register */

  putreg32(RTC_MAGIC, RTC_MAGIC_REG);

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: lpc54_rtc_setalarm
 *
 * Description:
 *   Set up an alarm.
 *
 * Input Parameters:
 *   tp       - The time to set the alarm
 *   callback - The function to call when the alarm expires.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
int lpc54_rtc_setalarm(FAR const struct timespec *tp, alarmcb_t callback)
{
  irqstate_t flags;
  uint32_t regval;
  int ret = -EBUSY;

  /* Is there already something waiting on the ALARM? */

  flags = enter_critical_section();
  if (g_alarmcb == NULL)
    {
      /* No.. Save the callback function pointer */

      g_alarmcb = callback;

      /* Make sure the the RTC is out of reset. */

      regval  = getreg32(LPC54_RTC_CTRL);
      regval &= ~(RTC_CTRL_SWRESET | RTC_CTRL_ALARMDPDEN | RTC_CTRL_RTC1KHZEN |
                  RTC_CTRL_WAKEDPDEN | RTC_CTRL_OSCPD);
      putreg32(regval, LPC54_RTC_CTRL);

      /* Make sure that the ALARM interrupt is attached and enabled. */

      irq_attach(LPC54_IRQ_RTC, lpc54_rtc_interrupt, NULL);
      up_enable_irq(LPC54_IRQ_RTC);

      /* Set the alarm match register */

      putreg32(tp->tv_sec, LPC54_RTC_MATCH);

      /* Enable RTC alarm */

      regval |= RTC_CTRL_ALARMDPDEN;
      putreg32(regval, LPC54_RTC_CTRL);
      ret = OK;
    }

  leave_critical_section(flags);
  return ret;
}
#endif

/****************************************************************************
 * Name: lpc54_rtc_rdalarm
 *
 * Description:
 *   Query an alarm configured in hardware.
 *
 * Input Parameters:
 *  time - Current alarm setting.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
int lpc54_rtc_rdalarm(FAR struct tm *time)
{
  uint32_t match;

  match = getreg32(LPC54_RTC_MATCH);
  (void)gmtime_r((time_t *)&match, time);
  return OK;
}
#endif

/****************************************************************************
 * Name: lpc54_rtc_cancelalarm
 *
 * Description:
 *   Cancel a pending alarm alarm
 *
 * Input Parameters:
 *   none
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
int lpc54_rtc_cancelalarm(void)
{
  irqstate_t flags;
  uint32_t regval;
  int ret = -ENODATA;

  flags = enter_critical_section();
  if (g_alarmcb != NULL)
    {
      /* Cancel the global callback function */

      g_alarmcb = NULL;

      /* Disable the Alarm interrupt */

      up_disable_irq(LPC54_IRQ_RTC);

      /* Unset the alarm */

      regval  = getreg32(LPC54_RTC_CTRL);
      regval &= ~(RTC_CTRL_SWRESET | RTC_CTRL_ALARMDPDEN | RTC_CTRL_RTC1KHZEN |
                  RTC_CTRL_WAKEDPDEN | RTC_CTRL_OSCPD);
      putreg32(regval, LPC54_RTC_CTRL);

      ret = OK;
    }

  leave_critical_section(flags);
  return ret;
}
#endif
