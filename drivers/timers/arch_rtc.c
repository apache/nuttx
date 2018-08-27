/****************************************************************************
 * drivers/timers/arch_rtc.c
 *
 *   Copyright (C) 2017 Pinecone Inc. All rights reserved.
 *   Author: Xiang Xiao <xiaoxiang@pinecone.net>
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

#include <nuttx/arch.h>
#include <nuttx/timers/arch_rtc.h>

#include <string.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct rtc_lowerhalf_s *g_rtc_lower;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Variable determines the state of the RTC module.
 *
 * After initialization value is set to 'true' if RTC starts successfully.
 * The value can be changed to false also during operation if RTC for
 * some reason fails.
 */

volatile bool g_rtc_enabled = false;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_rtc_set_lowerhalf(FAR struct rtc_lowerhalf_s *lower)
{
  g_rtc_lower = lower;
  g_rtc_enabled = true;
}

/************************************************************************************
 * Name: up_rtc_time
 *
 * Description:
 *   Get the current time in seconds.  This is similar to the standard time()
 *   function.  This interface is only required if the low-resolution RTC/counter
 *   hardware implementation selected.  It is only used by the RTOS during
 *   initialization to set up the system time when CONFIG_RTC is set but neither
 *   CONFIG_RTC_HIRES nor CONFIG_RTC_DATETIME are set.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current time in seconds.
 *
 ************************************************************************************/

#ifndef CONFIG_RTC_HIRES
time_t up_rtc_time(void)
{
  struct rtc_time rtctime;
  time_t time = 0;

  if (!g_rtc_lower)
    {
      return 0;
    }

  if (g_rtc_lower->ops->rdtime(g_rtc_lower, &rtctime) == 0)
    {
      time = mktime((FAR struct tm *)&rtctime);
    }

  return time;
}
#endif

/************************************************************************************
 * Name: up_rtc_gettime
 *
 * Description:
 *   Get the current time from the high resolution RTC clock/counter.  This interface
 *   is only supported by the high-resolution RTC/counter hardware implementation.
 *   It is used to replace the system timer.
 *
 * Input Parameters:
 *   tp - The location to return the high resolution time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ************************************************************************************/

#ifdef CONFIG_RTC_HIRES
int up_rtc_gettime(FAR struct timespec *tp)
{
  struct rtc_time rtctime;
  int ret;

  if (!g_rtc_lower)
    {
      memset(tp, 0, sizeof(*tp));
      return 0;
    }

  ret = g_rtc_lower->ops->rdtime(g_rtc_lower, &rtctime);
  if (ret == 0)
    {
      tp->tv_sec = mktime((FAR struct tm *)&rtctime);
      tp->tv_nsec = rtctime.tm_nsec;
    }

  return ret;
}
#endif

/************************************************************************************
 * Name: up_rtc_getdatetime
 *
 * Description:
 *   Get the current date and time from the date/time RTC.  This interface
 *   is only supported by the date/time RTC hardware implementation.
 *   It is used to replace the system timer.  It is only used by the RTOS during
 *   initialization to set up the system time when CONFIG_RTC and CONFIG_RTC_DATETIME
 *   are selected (and CONFIG_RTC_HIRES is not).
 *
 *   NOTE: Some date/time RTC hardware is capability of sub-second accuracy.  That
 *   sub-second accuracy is lost in this interface.  However, since the system time
 *   is reinitialized on each power-up/reset, there will be no timing inaccuracy in
 *   the long run.
 *
 * Input Parameters:
 *   tp - The location to return the high resolution time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ************************************************************************************/

#ifdef CONFIG_RTC_DATETIME
int up_rtc_getdatetime(FAR struct tm *tp)
{
  struct rtc_time rtctime;
  int ret;

  if (!g_rtc_lower)
    {
      memset(tp, 0, sizeof(*tp));
      return 0;
    }

  ret = g_rtc_lower->ops->rdtime(g_rtc_lower, &rtctime);
  if (ret == 0)
    {
      *tp = *((FAR struct tm *)&rtctime);
    }

  return ret;
}
#endif

/************************************************************************************
 * Name: up_rtc_getdatetime_with_subseconds
 *
 * Description:
 *   Get the current date and time from the date/time RTC.  This interface
 *   is only supported by the date/time RTC hardware implementation.
 *   It is used to replace the system timer.  It is only used by the RTOS during
 *   initialization to set up the system time when CONFIG_RTC and CONFIG_RTC_DATETIME
 *   are selected (and CONFIG_RTC_HIRES is not).
 *
 *   NOTE: This interface exposes sub-second accuracy capability of RTC hardware.
 *   This interface allow maintaining timing accuracy when system time needs constant
 *   resynchronization with RTC, for example on MCU with low-power state that
 *   stop system timer.
 *
 * Input Parameters:
 *   tp - The location to return the high resolution time value.
 *   nsec - The location to return the subsecond time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

#if defined(CONFIG_RTC_DATETIME) && defined(CONFIG_ARCH_HAVE_RTC_SUBSECONDS)
int up_rtc_getdatetime_with_subseconds(FAR struct tm *tp, FAR long *nsec)
{
  struct rtc_time rtctime;
  int ret;

  if (!g_rtc_lower)
    {
      memset(tp, 0, sizeof(*tp));
      *nsec = 0;
      return 0;
    }

  ret = g_rtc_lower->ops->rdtime(g_rtc_lower, &rtctime);
  if (ret == 0)
    {
      *tp = *((FAR struct tm *)&rtctime);
      *nsec = rtctime.tm_nsec;
    }

  return ret;
}
#endif

/************************************************************************************
 * Name: up_rtc_settime
 *
 * Description:
 *   Set the RTC to the provided time.  All RTC implementations must be able to
 *   set their time based on a standard timespec.
 *
 * Input Parameters:
 *   tp - the time to use
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ************************************************************************************/

int up_rtc_settime(FAR const struct timespec *tp)
{
  struct rtc_time rtctime;

  if (!g_rtc_lower)
    {
      return -EAGAIN;
    }

  gmtime_r(&tp->tv_sec, (FAR struct tm *)&rtctime);
#if defined(CONFIG_RTC_HIRES) || defined(CONFIG_ARCH_HAVE_RTC_SUBSECONDS)
  rtctime.tm_nsec = tp->tv_nsec;
#endif

  return g_rtc_lower->ops->settime(g_rtc_lower, &rtctime);
}
