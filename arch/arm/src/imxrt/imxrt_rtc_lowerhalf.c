/****************************************************************************
 * arch/arm/src/imxrt/imxrt_rtc_lowerhalf.c
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

#include <sys/types.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/timers/rtc.h>

#include "arm_internal.h"
#include "chip.h"
#include "imxrt_lpsrtc.h"
#include "imxrt_hprtc.h"

#ifdef CONFIG_RTC_DRIVER

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This is the private type for the RTC state.  It must be cast compatible
 * with struct rtc_lowerhalf_s.
 */

struct imxrt_lowerhalf_s
{
  /* This is the contained reference to the read-only, lower-half
   * operations vtable (which may lie in FLASH or ROM)
   */

  const struct rtc_ops_s *ops;

  /* Data following is private to this driver and not visible outside of
   * this file.
   */

  sem_t devsem;                     /* Threads can only exclusively access the RTC */

#ifdef CONFIG_RTC_ALARM
  /* Alarm callback information */

  volatile rtc_alarm_callback_t cb; /* Callback when the alarm expires */
  volatile void *priv;              /* Private argument to accompany callback */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Prototypes for static methods in struct rtc_ops_s */

static int imxrt_rdtime(struct rtc_lowerhalf_s *lower,
                        struct rtc_time *rtctime);
static int imxrt_settime(struct rtc_lowerhalf_s *lower,
                         const struct rtc_time *rtctime);
static bool imxrt_havesettime(struct rtc_lowerhalf_s *lower);

#ifdef CONFIG_RTC_ALARM
static int imxrt_setalarm(struct rtc_lowerhalf_s *lower,
                          const struct lower_setalarm_s *alarminfo);
static int imxrt_setrelative(struct rtc_lowerhalf_s *lower,
                            const struct lower_setrelative_s *alarminfo);
static int imxrt_cancelalarm(struct rtc_lowerhalf_s *lower,
                             int alarmid);
static int imxrt_rdalarm(struct rtc_lowerhalf_s *lower,
                         struct lower_rdalarm_s *alarminfo);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* i.MXRT RTC driver operations */

static const struct rtc_ops_s g_rtc_ops =
{
  .rdtime         = imxrt_rdtime,
  .settime        = imxrt_settime,
  .havesettime    = imxrt_havesettime,
#ifdef CONFIG_RTC_ALARM
  .setalarm       = imxrt_setalarm,
  .setrelative    = imxrt_setrelative,
  .cancelalarm    = imxrt_cancelalarm,
  .rdalarm        = imxrt_rdalarm,
#endif
#ifdef CONFIG_RTC_PERIODIC
  .setperiodic    = NULL,  /* Not implemented */
  .cancelperiodic = NULL,
#endif
#ifdef CONFIG_RTC_IOCTL
  .ioctl          = NULL,
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  .destroy        = NULL,
#endif
};

/* i.MXRT RTC device state */

static struct imxrt_lowerhalf_s g_rtc_lowerhalf =
{
  .ops         = &g_rtc_ops,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_alarm_callback
 *
 * Description:
 *   This is the function that is called from the RTC driver when the alarm
 *   goes off.  It just invokes the upper half drivers callback.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static void imxrt_alarm_callback(void)
{
  struct imxrt_lowerhalf_s *rtc = &g_rtc_lowerhalf;

  /* Sample and clear the callback information to minimize the window in
   * time in which race conditions can occur.
   */

  rtc_alarm_callback_t cb = (rtc_alarm_callback_t)rtc->cb;
  void *arg           = (void *)rtc->priv;

  rtc->cb                 = NULL;
  rtc->priv               = NULL;

  /* Perform the callback */

  if (cb != NULL)
    {
      cb(arg, 0);
    }
}
#endif /* CONFIG_RTC_ALARM */

/****************************************************************************
 * Name: imxrt_rdtime
 *
 * Description:
 *   Implements the rdtime() method of the RTC driver interface
 *
 * Input Parameters:
 *   lower   - A reference to RTC lower half driver state structure
 *   rcttime - The location in which to return the current RTC time.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

static int imxrt_rdtime(struct rtc_lowerhalf_s *lower,
                        struct rtc_time *rtctime)
{
  time_t timer;

  /* The resolution of time is only 1 second */

  timer = up_rtc_time();

  /* Convert the one second epoch time to a struct tm */

  if (gmtime_r(&timer, (struct tm *)rtctime) == 0)
    {
      int errcode = get_errno();
      DEBUGASSERT(errcode > 0);

      rtcerr("ERROR: gmtime_r failed: %d\n", errcode);
      return -errcode;
    }

  return OK;
}

/****************************************************************************
 * Name: imxrt_settime
 *
 * Description:
 *   Implements the settime() method of the RTC driver interface
 *
 * Input Parameters:
 *   lower   - A reference to RTC lower half driver state structure
 *   rcttime - The new time to set
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

static int imxrt_settime(struct rtc_lowerhalf_s *lower,
                         const struct rtc_time *rtctime)
{
  struct timespec ts;

  /* Convert the struct rtc_time to a time_t.  Here we assume that struct
   * rtc_time is cast compatible with struct tm.
   */

  ts.tv_sec  = timegm((struct tm *)rtctime);
  ts.tv_nsec = 0;

  /* Now set the time (to one second accuracy) */

  return up_rtc_settime(&ts);
}

/****************************************************************************
 * Name: imxrt_havesettime
 *
 * Description:
 *   Implements the havesettime() method of the RTC driver interface
 *
 * Input Parameters:
 *   lower   - A reference to RTC lower half driver state structure
 *
 * Returned Value:
 *   Returns true if RTC date-time have been previously set.
 *
 ****************************************************************************/

static bool imxrt_havesettime(struct rtc_lowerhalf_s *lower)
{
#ifdef CONFIG_IMXRT_SNVS_LPSRTC
  return imxrt_lpsrtc_havesettime();
#else
  return g_hprtc_timset;
#endif
}

/****************************************************************************
 * Name: imxrt_setalarm
 *
 * Description:
 *   Set a new alarm.  This function implements the setalarm() method of the
 *   RTC driver interface
 *
 * Input Parameters:
 *   lower     - A reference to RTC lower half driver state structure
 *   alarminfo - Provided information needed to set the alarm
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int imxrt_setalarm(struct rtc_lowerhalf_s *lower,
                          const struct lower_setalarm_s *alarminfo)
{
  struct imxrt_lowerhalf_s *rtc;
  int ret;

  DEBUGASSERT(lower != NULL && alarminfo != NULL && alarminfo->id == 0);
  rtc = (struct imxrt_lowerhalf_s *)lower;

  /* Get exclusive access to the alarm */

  ret = nxsem_wait(&rtc->devsem);
  if (ret < 0)
    {
      rtcerr("ERROR: nxsem_wait failed: %d\n", ret);
      return ret;
    }

  ret = -EINVAL;
  if (alarminfo->id == 0)
    {
      struct timespec ts;

      /* Convert the RTC time to a timespec (1 second accuracy) */

      ts.tv_sec   = timegm((struct tm *)&alarminfo->time);
      ts.tv_nsec  = 0;

      /* Remember the callback information */

      rtc->cb     = alarminfo->cb;
      rtc->priv   = alarminfo->priv;

      /* And set the alarm */

      ret = imxrt_hprtc_setalarm(&ts, imxrt_alarm_callback);
      if (ret < 0)
        {
          rtc->cb   = NULL;
          rtc->priv = NULL;
        }
    }

  nxsem_post(&rtc->devsem);
  return ret;
}
#endif

/****************************************************************************
 * Name: imxrt_setrelative
 *
 * Description:
 *   Set a new alarm relative to the current time.  This function implements
 *   the setrelative() method of the RTC driver interface
 *
 * Input Parameters:
 *   lower - A reference to RTC lower half driver state structure
 *   alarminfo - Provided information needed to set the alarm
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int imxrt_setrelative(struct rtc_lowerhalf_s *lower,
                             const struct lower_setrelative_s *alarminfo)
{
  struct imxrt_lowerhalf_s *rtc;
  struct timespec ts;
  int ret = -EINVAL;

  DEBUGASSERT(lower != NULL && alarminfo != NULL && alarminfo->id == 0);
  rtc = (struct imxrt_lowerhalf_s *)lower;

  /* Get exclusive access to the alarm */

  ret = nxsem_wait(&rtc->devsem);
  if (ret < 0)
    {
      rtcerr("ERROR: nxsem_wait failed: %d\n", ret);
      return ret;
    }

  ret = -EINVAL;
  if (alarminfo->id == 0 && alarminfo->reltime > 0)
    {
      /* Get the current time in seconds */

      /* The resolution of time is only 1 second */

      ts.tv_sec  = up_rtc_time();
      ts.tv_nsec = 0;

      /* Add the seconds offset.  Add one to the number of seconds because
       * we are unsure of the phase of the timer.
       */

      ts.tv_sec += (alarminfo->reltime + 1);

      /* Remember the callback information */

      rtc->cb     = alarminfo->cb;
      rtc->priv   = alarminfo->priv;

      /* And set the alarm */

      ret = imxrt_hprtc_setalarm(&ts, imxrt_alarm_callback);
      if (ret < 0)
        {
          rtc->cb   = NULL;
          rtc->priv = NULL;
        }
    }

  nxsem_post(&rtc->devsem);
  return ret;
}
#endif

/****************************************************************************
 * Name: imxrt_cancelalarm
 *
 * Description:
 *   Cancel the current alarm.  This function implements the cancelalarm()
 *   method of the RTC driver interface
 *
 * Input Parameters:
 *   lower - A reference to RTC lower half driver state structure
 *   alarminfo - Provided information needed to set the alarm
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int imxrt_cancelalarm(struct rtc_lowerhalf_s *lower, int alarmid)
{
  /* We cancel the alarm by alarm by disabling the alarm and the alarm
   * interrupt.
   */

  imxrt_hprtc_alarmdisable();
  return OK;
}
#endif

/****************************************************************************
 * Name: imxrt_rdalarm
 *
 * Description:
 *   Query the RTC alarm.
 *
 * Input Parameters:
 *   lower     - A reference to RTC lower half driver state structure
 *   alarminfo - Provided information needed to query the alarm
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int imxrt_rdalarm(struct rtc_lowerhalf_s *lower,
                         struct lower_rdalarm_s *alarminfo)
{
  int ret = -EINVAL;

  DEBUGASSERT(lower != NULL && alarminfo != NULL && alarminfo->time != NULL);
  DEBUGASSERT(alarminfo->id == 0);

  /* There is only one alarm */

  if (alarminfo->id == 0)
    {
      time_t alarm;

      /* Get the current alarm setting in seconds */

      alarm = (time_t)imxrt_hprtc_getalarm();

      /* Convert the one second epoch time to a struct tm */

      ret = OK;
      if (gmtime_r(&alarm, (struct tm *)alarminfo->time) == 0)
        {
          int errcode = get_errno();
          DEBUGASSERT(errcode > 0);
          ret = -errcode;
        }
    }

  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_rtc_lowerhalf
 *
 * Description:
 *   Instantiate the RTC lower half driver for the i.MXRT.  General usage:
 *
 *     #include <nuttx/timers/rtc.h>
 *
 *     struct rtc_lowerhalf_s *lower;
 *     lower = imxrt_rtc_lowerhalf();
 *     rtc_initialize(0, lower);
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, a non-NULL RTC lower interface is returned.  NULL is
 *   returned on any failure.
 *
 ****************************************************************************/

struct rtc_lowerhalf_s *imxrt_rtc_lowerhalf(void)
{
  nxsem_init(&g_rtc_lowerhalf.devsem, 0, 1);

  return (struct rtc_lowerhalf_s *)&g_rtc_lowerhalf;
}

#endif /* CONFIG_RTC_DRIVER */
