/****************************************************************************
 * arch/arm/src/max326xx/max32660_rtc.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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
#include <fixedmath.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/timers/rtc.h>

#include "arm_arch.h"

#include "hardware/max326_rtc.h"
#include "max326_rtc.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* In hi-res mode, the RTC operates at 256Hz.  In the lo-res mode, the RTC
 * operates at 1Hz.
 */

#ifdef CONFIG_RTC_HIRES
#  ifndef CONFIG_RTC_FREQUENCY
#    error "CONFIG_RTC_FREQUENCY is required for CONFIG_RTC_HIRES"
#  elif CONFIG_RTC_FREQUENCY != 256
#    error "Only hi-res CONFIG_RTC_FREQUENCY of 256Hz is supported"
#  endif
#else
#  ifndef CONFIG_RTC_FREQUENCY
#    define CONFIG_RTC_FREQUENCY 1
#  endif
#  if CONFIG_RTC_FREQUENCY != 1
#    error "Only lo-res CONFIG_RTC_FREQUENCY of 1Hz is supported"
#  endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
/* Callback to use when the alarm expires */

static alm_callback_t g_alarmcb;
static FAR void *g_alarmarg;
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Indicates that the RTC has been initialized */

volatile bool g_rtc_enabled = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max326_rtc_waitbusy
 *
 * Description:
 *   Wait for registers to synchronize with the RTC module.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void max326_rtc_waitbusy(void)
{
  while ((getreg32(MAX326_RTC_CTRL) & RTC_CTRL_BUSY) != 0)
    {
    }
}

/****************************************************************************
 * Name: max326_rtc_enable
 *
 * Description:
 *   Enable/disable the RTC.  RTC must be disabled in order to set the SEC
 *   and SSEC registers.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void max326_rtc_enable(bool enable)
{
  uint32_t regval;

  regval = getreg32(MAX326_RTC_CTRL);
  if (enable)
    {
      regval |= RTC_CTRL_ENABLE;
    }
  else
    {
      regval &= ~RTC_CTRL_ENABLE;
    }

  putreg32(regval, MAX326_RTC_CTRL);
  max326_rtc_waitbusy();
}

/****************************************************************************
 * Name: max326_rtc_wrenable
 *
 * Description:
 *   Enable/disable writing to the TRIM register or to the CTRL enable bit.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void max326_rtc_wrenable(bool enable)
{
  uint32_t regval;

  regval = getreg32(MAX326_RTC_CTRL);
  if (enable)
    {
      regval |= RTC_CTRL_WRITEEN;
    }
  else
    {
      regval &= !RTC_CTRL_WRITEEN;
    }

  putreg32(regval, MAX326_RTC_CTRL);
}

/****************************************************************************
 * Name: max326_rtc_tm2b32
 *
 * Description:
 *   Encode time in the standard struct timespec format to type b32_t.
 *
 * Input Parameters:
 *   tp - the time to use
 *
 * Returned Value:
 *   Encoded time
 *
 ****************************************************************************/

static b32_t max326_rtc_tm2b32(FAR const struct timespec *tp)
{
  b32_t intpart;
  b32_t fracpart;

  DEBUGASSERT(tp != NULL && tp->tv_nsec < NSEC_PER_SEC);

  /* Convert the time to b32_t type:
   *
   * Integer part is easy:
   *
   * intpart:  ssssssss ssssssss 00000000 00000000
   *           00000000 00000001 00000000 00000000 = 1 Sec
   */

  intpart = itob32(tp->tv_sec);

  /* Fractional part:
   *
   * fracpart: 00000000 00000000 ffffffff ffffffff
   *           00000000 00000001 00000000 00000000 = 1 Sec
   */

   if (tp->tv_nsec > 0)
    {
      fracpart = itob32(tp->tv_nsec) / NSEC_PER_SEC;
    }
  else
    {
      fracpart = 0;
    }

  return intpart + fracpart;
}

/****************************************************************************
 * Name: max326_rtc_interrupt
 *
 * Description:
 *    RTC alarm interrupt service routine
 *
 * Input Parameters:
 *   irq    - The IRQ number that generated the interrupt
 *   context - Architecture specific register save information.
 *   arg     - Argument that my accompany the interrupt event
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int max326_rtc_interrupt(int irq, void *context, FAR void *arg)
{
  uint32_t regval;

  /* Get the CTRL register */

  regval = getreg32(MAX326_RTC_CTRL);
  DEBUGASSERT((regval & RTC_CTRL_ALARM_SSFL) != 0);

  /* Clear pending interrupt flags.  Disable alarms */

  regval &= ~(RTC_CTRL_ALARM_TODEN | RTC_CTRL_ALARM_SSEN |
              RTC_CTRL_ALARM_TODFL | RTC_CTRL_ALARM_SSFL);
  putreg32(regval, MAX326_RTC_CTRL);

  /* Disable further RTC interrupts */

  up_disable_irq(MAX326_IRQ_RTC);

  DEBUGASSERT(g_alarmcb != NULL);
  if (g_alarmcb != NULL)
    {
      /* Perform the alarm callback */

      g_alarmcb(g_alarmarg, 0);
      g_alarmcb  = NULL;
      g_alarmarg = NULL;
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
 *   Initialize the hardware RTC per the selected configuration.  This
 *   function is called once during the OS initialization sequence
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
  /* Configure initial the RTC CTRL register
   *
   * - RTC disabled
   * - All interrupts disabled.
   * - No Square Wave output.
   * - Oscillator operates in noise immunity mode
   * - Write disabled.
   */

  putreg32(0, MAX326_RTC_CTRL);
  max326_rtc_waitbusy();

  /* Enable write access */

  max326_rtc_wrenable(false);

  /* No RTC Square Wave Output */

  putreg32(0, MAX326_RTC_OSCCTRL);
  max326_rtc_waitbusy();

#ifdef CONFIG_RTC_ALARM
  /* Attach the ALARM interrupt handler */

  up_disable_irq(MAX326_IRQ_RTC);
  irq_attach(MAX326_IRQ_RTC, max326_rtc_interrupt, NULL);
#endif

  /* Enable the RTC */

  max326_rtc_enable(true);

  /* Disable write access */

  max326_rtc_wrenable(false);

  g_rtc_enabled = true;
  return OK;
}

/****************************************************************************
 * Name: up_rtc_time
 *
 * Description:
 *   Get the current time in seconds.  This is similar to the
 *   standard time() function.   It is only used by the RTOS during
 *   [re-]initialization to set up the system time when CONFIG_RTC is set
 *   but neither CONFIG_RTC_HIRES nor CONFIG_RTC_DATETIME are set.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current time in seconds
 *
 ****************************************************************************/

#ifndef CONFIG_RTC_HIRES
time_t up_rtc_time(void)
{
  uint32_t sec;
  uint32_t ssec;
  uint32_t verify;

  /* Read the time handling rollover to full seconds */

  do
    {
      sec    = getreg32(MAX326_RTC_SEC);
      ssec   = getreg32(MAX326_RTC_SSEC) &RTC_SSEC_MASK;
      verify = getreg32(MAX326_RTC_SEC);
    }
  while (verify != sec);

  /* Round */

  if (ssec > 128)
    {
      sec++;
    }

  return sec;
}
#endif

/****************************************************************************
 * Name: up_rtc_gettime
 *
 * Description:
 *   Get the current time from the high resolution RTC clock/counter.  This
 *   interface is only supported by the high-resolution RTC/counter hardware
 *   implementation.  It is used to replace the system timer.
 *
 * Input Parameters:
 *   tp - The location to return the high resolution time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_HIRES
int up_rtc_gettime(FAR struct timespec *tp)
{
  uint64_t tmp;
  uint32_t sec;
  uint32_t ssec;
  uint32_t verify;

  /* Read the time handling rollover to full seconds */

  do
    {
      sec    = getreg32(MAX326_RTC_SEC);
      ssec   = getreg32(MAX326_RTC_SSEC) & RTC_SSEC_MASK;
      verify = getreg32(MAX326_RTC_SEC);
    }
  while (verify != sec);

  /* Format as a tm */

  tmp = ((uint64_t)ssec * NSEC_PER_SEC) / 256;

  tp->tv_sec  = sec;
  tp->tv_nsec = (long)tmp;

  return OK;
}
#endif

/****************************************************************************
 * Name: up_rtc_settime
 *
 * Description:
 *   Set the RTC to the provided time.  All RTC implementations must be able to
 *   set their time based on a standard tm.
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
  b32_t ftime;
  uint32_t sec;
  uint32_t ssec;
  uint32_t verify;

  /* Get the time as a fixed precision number. */

  ftime = max326_rtc_tm2b32(tp);
  sec   = b32toi(ftime);
  ssec  = (uint32_t)(b32frac(ftime) >> (32 - 8));

  /* Enable write access to RTC configuration registers */

  flags = spin_lock_irqsave();
  max326_rtc_wrenable(true);

  /* We need to disable the RTC in order to write to the SEC and SSEC
   * registers.
   */

  max326_rtc_enable(false);

  /* Then write the time values to the SEC and SSEC registers. */

  do
    {
      putreg32(sec, MAX326_RTC_SEC);
      putreg32(ssec, MAX326_RTC_SSEC);
      verify = getreg32(MAX326_RTC_SEC);
    }
  while (verify != sec);

  /* Re-enable the RTC and disable write access */

  max326_rtc_enable(true);
  max326_rtc_wrenable(false);

  spin_unlock_irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Name: max326_rtc_setalarm
 *
 * Description:
 *   Set an alarm to an absolute time using associated hardware.
 *
 * Input Parameters:
 *  ts  - Alarm time
 *  cb  - Callback invoked when alarm expires
 *  arg - Argument passed with the alarm
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
int max326_rtc_setalarm(FAR struct timespec *ts, alm_callback_t cb, FAR void *arg)
{
  irqstate_t flags;
  b32_t b32now;
  b32_t b32alarm;
  b32_t b32delay;
  uint32_t sec;
  uint32_t ssec;
  uint32_t rssa;
  uint32_t verify;
  uint32_t regval;
  int ret = -EBUSY;

  DEBUGASSERT(alminfo != NULL && alminfo->as_cb != NULL);

  /* Is there already something waiting on the ALARM? */

  flags = spin_lock_irqsave();
  if (g_alarmcb == NULL)
    {
      /* Get the time as a fixed precision number.
       * Form:  ssssssss ssssssss ffffffff ffffff
       *        |                 `- 32-bits of fraction
       *        `- 32-bits of integer seconds
       */

      b32alarm = max326_rtc_tm2b32(ts);

      /* Get the current time */

      do
        {
          sec    = getreg32(MAX326_RTC_SEC);
          ssec   = getreg32(MAX326_RTC_SSEC);
          verify = getreg32(MAX326_RTC_SEC);
        }
      while (verify != sec);

      /* Get the current time as a b32_t value */

      b32now = itob32(sec) | (b32_t)ssec << (32 - 8);
      if (b32now >= b32alarm)
        {
          ret = -EINVAL;
          goto errout_with_lock;
        }

      /* Get the ALARM delay between now and the alarm time */

      b32delay = b32alarm - b32now;

      /* Convert to RSSA value.
       *
       * Writing RSSA sets the starting value for the sub-second alarm
       * counter. Writing the SSEN enables the sub-second alarm. Once
       * enabled, the sub-second alarm begins up-counting from the RSSA
       * value. When the counter rolls over from 0xffffffff to 0x00000000.
       * A 256Hz clock drives the sub-second alarm allowing a maximum
       * interval of 16,777,216 seconds with a resolution of approximately
       * 3.9 msec.
       *
       * The delay is ssss ssff Where ssssss is the ls 24 bits of seconds
       * and ff is the 8bit fractional value.  To get the RSSA, that value
       * has to be subtracted from 1 << 32.
       */

       if ((uint32_t)b32toi(b32delay) >= 16777216)
         {
           rssa = 0;
         }
       else
         {
           uint64_t tmp = ((b32delay >> (32 - 8)) & 0x00000000ffffffff);
           if (tmp == 0)
             {
               rssa = UINT32_MAX;
             }
           else
             {
               tmp  = 0x0000000100000000 - tmp;
               rssa = (uint32_t)tmp;
             }
         }

      /* We need to disable ALARMs in order to write to the RSSA registers. */

      regval  = getreg32(MAX326_RTC_CTRL);
      regval &= ~(RTC_CTRL_ALARM_TODEN | RTC_CTRL_ALARM_SSEN);
      putreg32(regval, MAX326_RTC_CTRL);
      max326_rtc_waitbusy();

      /* Then write the RSSA values. */

      putreg32(rssa, MAX326_RTC_RSSA);

      /* Enable sub-second alarm */

      regval |= RTC_CTRL_ALARM_SSEN;
      putreg32(regval, MAX326_RTC_CTRL);

      /* No.. Save the callback function pointer */

      g_alarmcb  = cb;
      g_alarmarg = arg;

      /* Enable the RTC interrupt at the NVIC */

      up_enable_irq(MAX326_IRQ_RTC);
      ret = OK;
    }

errout_with_lock:
  spin_unlock_irqrestore(flags);
  return ret;
}
#endif

/****************************************************************************
 * Name: max326_rtc_rdalarm
 *
 * Description:
 *   Query an alarm configured in hardware.
 *
 * Input Parameters:
 *  ftime - Location to return the current alarm setting.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
int max326_rtc_rdalarm(FAR b32_t *ftime)
{
  b32_t b32now;
  b32_t b32delay;
  uint32_t sec;
  uint32_t ssec;
  uint32_t rssa;
  uint32_t verify;
  uint32_t regval;

  DEBUGASSERT(ftime != NULL);

  /* Read the SEC and the RSSA registers */

  do
    {
      rssa   = getreg32(MAX326_RTC_RSSA);
      sec    = getreg32(MAX326_RTC_SEC);
      ssec   = getreg32(MAX326_RTC_SSEC);
      verify = getreg32(MAX326_RTC_RSSA);
    }
  while (verify != rssa);

  /* Check if the alarm is enabled */

  regval = getreg32(MAX326_RTC_CTRL);
  if ((regval & RTC_CTRL_ALARM_SSEN) == 0)
    {
      return -EINVAL;
    }

  /* Get the current time as a b32_t value */

  b32now = itob32(sec) | ssec << (32 - 8);

  /* Use the RSSA value to determine the time when the alarm will fire:
   *
   * alarm_time = current_time + rssa_remaining
   *
   * Where current_time is the time from SEC and SSEC registers.
   *       rssa_remaining is the time remaining until RSSA roles
   *       over to zero, that is (1 << 32) - RRSA.
   */

   if (rssa > 0)
     {
       b32delay = 0x0000000100000000 - (uint64_t)rssa;
       b32delay = (b32delay & 0x00000000ffffffff) << (32 - 8);
     }
   else
     {
       b32delay = 0;
     }

   *ftime  = b32now + b32delay;
   return OK;
}
#endif

/****************************************************************************
 * Name: max326_rtc_cancelalarm
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
int max326_rtc_cancelalarm(void)
{
  irqstate_t flags;
  uint32_t regval;
  int ret = -ENODATA;

  flags = spin_lock_irqsave();

  if (g_alarmcb != NULL)
    {
      /* Disable the alarm interrupt */

      up_disable_irq(MAX326_IRQ_RTC);

      /* Disable the alarm */

      regval = getreg32(MAX326_RTC_CTRL);
      regval &= ~(RTC_CTRL_ALARM_TODEN | RTC_CTRL_ALARM_SSEN);
      putreg32(regval, MAX326_RTC_CTRL);
      max326_rtc_waitbusy();

      /* Cancel the global callback function */

      g_alarmcb  = NULL;
      g_alarmarg = NULL;

      /* Unset the alarm (for debug) */

      putreg32(0, MAX326_RTC_RAS);
      putreg32(0, MAX326_RTC_RSSA);

      ret = OK;
    }

  spin_unlock_irqrestore(flags);
  return ret;
}
#endif
