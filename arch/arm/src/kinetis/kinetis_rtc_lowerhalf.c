/****************************************************************************
 * arch/arm/src/kinetis/kinetis_rtc_lowerhalf.c
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

/* REVISIT:  This driver is *not* thread-safe! */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/timers/rtc.h>

#include "chip.h"
#include "kinetis_rtc_if.h"
#include "kinetis_alarm.h"

#ifdef CONFIG_RTC_DRIVER

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
struct kinetis_cbinfo_s
{
  volatile rtc_alarm_callback_t cb; /* Callback when the alarm expires */
  volatile void *priv;              /* Private argurment to accompany callback */
};
#endif

/* This is the private type for the RTC state.  It must be cast compatible
 * with struct rtc_lowerhalf_s.
 */

struct kinetis_lowerhalf_s
{
  /* This is the contained reference to the read-only, lower-half
   * operations vtable (which may lie in FLASH or ROM)
   */

  const struct rtc_ops_s *ops;

  /* Data following is private to this driver and not visible outside of
   * this file.
   */

#ifdef CONFIG_RTC_ALARM
  /* Alarm callback information */

  struct kinetis_cbinfo_s cbinfo;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Prototypes for static methods in struct rtc_ops_s */

static int kinetis_rdtime(struct rtc_lowerhalf_s *lower,
                          struct rtc_time *rtctime);
static int kinetis_settime(struct rtc_lowerhalf_s *lower,
                           const struct rtc_time *rtctime);

#ifdef CONFIG_RTC_ALARM
static int kinetis_setalarm(struct rtc_lowerhalf_s *lower,
                            const struct lower_setalarm_s *alarminfo);
static int kinetis_setrelative(struct rtc_lowerhalf_s *lower,
                            const struct lower_setrelative_s *alarminfo);
static int kinetis_cancelalarm(struct rtc_lowerhalf_s *lower,
                               int alarmid);
static int kinetis_rdalarm(struct rtc_lowerhalf_s *lower,
                           struct lower_rdalarm_s *alarminfo);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Kinetis RTC driver operations */

static const struct rtc_ops_s g_rtc_ops =
{
  .rdtime      = kinetis_rdtime,
  .settime     = kinetis_settime,
#ifdef CONFIG_RTC_ALARM
  .setalarm    = kinetis_setalarm,
  .setrelative = kinetis_setrelative,
  .cancelalarm = kinetis_cancelalarm,
  .rdalarm     = kinetis_rdalarm,
#endif
};

/* Kinetis RTC device operations */

static struct kinetis_lowerhalf_s g_rtc_lowerhalf =
{
  .ops        = &g_rtc_ops,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kinetis_alarm_callback
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
static void kinetis_alarm_callback(void)
{
  struct kinetis_cbinfo_s *cbinfo = &g_rtc_lowerhalf.cbinfo;

  /* Sample and clear the callback information to minimize the window in
   * time in which race conditions can occur.
   */

  rtc_alarm_callback_t cb = (rtc_alarm_callback_t)cbinfo->cb;
  void *arg               = (void *)cbinfo->priv;

  cbinfo->cb              = NULL;
  cbinfo->priv            = NULL;

  /* Perform the callback */

  if (cb != NULL)
    {
      cb(arg, 0);
    }
}
#endif /* CONFIG_RTC_ALARM */

/****************************************************************************
 * Name: kinetis_rdtime
 *
 * Description:
 *   Implements the rdtime() method of the RTC driver interface
 *
 * Input Parameters:
 *   lower   - A reference to RTC lower half driver state structure
 *   rtctime - The location in which to return the current RTC time.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

static int kinetis_rdtime(struct rtc_lowerhalf_s *lower,
                          struct rtc_time *rtctime)
{
#if defined(CONFIG_RTC_DATETIME)
  /* This operation depends on the fact that struct rtc_time is cast
   * compatible with struct tm.
   */

  return up_rtc_getdatetime((struct tm *)rtctime);

#elif defined(CONFIG_RTC_HIRES)
  struct timespec ts;
  int ret;

  /* Get the higher resolution time */

  ret = up_rtc_gettime(&ts);
  if (ret < 0)
    {
      goto errout;
    }

  /* Convert the one second epoch time to a struct tm.  This operation
   * depends on the fact that struct rtc_time and struct tm are cast
   * compatible.
   */

  if (!gmtime_r(&ts.tv_sec, (struct tm *)rtctime))
    {
      ret = -get_errno();
      goto errout;
    }

  return OK;

errout:
  DEBUGASSERT(ret < 0);
  return ret;

#else
  time_t timer;

  /* The resolution of time is only 1 second */

  timer = up_rtc_time();

  /* Convert the one second epoch time to a struct tm */

  if (!gmtime_r(&timer, (struct tm *)rtctime))
    {
      int errcode = get_errno();
      DEBUGASSERT(errcode > 0);
      return -errcode;
    }

  return OK;
#endif
}

/****************************************************************************
 * Name: kinetis_settime
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

static int kinetis_settime(struct rtc_lowerhalf_s *lower,
                           const struct rtc_time *rtctime)
{
  struct timespec ts;

  /* Convert the struct rtc_time to a time_t.  Here we assume that struct
   * rtc_time is cast compatible with struct tm.
   */

  ts.tv_sec  = timegm((struct tm *)rtctime);
  ts.tv_nsec = 0;

  /* Set the time (to one second accuracy) */

  return up_rtc_settime(&ts);
}

/****************************************************************************
 * Name: kinetis_setalarm
 *
 * Description:
 *   Set a new alarm.  This function implements the setalarm() method of the
 *   RTC driver interface
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
static int kinetis_setalarm(struct rtc_lowerhalf_s *lower,
                            const struct lower_setalarm_s *alarminfo)
{
  struct kinetis_lowerhalf_s *priv;
  struct kinetis_cbinfo_s *cbinfo;

  struct timespec tp;

  int ret = -EINVAL;

  /* ID0-> Alarm A supported */

  DEBUGASSERT(lower != NULL && alarminfo != NULL);
  priv = (struct kinetis_lowerhalf_s *)lower;

  if (alarminfo->id == RTC_ALARMA)
    {
      /* Remember the callback information */

      cbinfo           = &priv->cbinfo;
      cbinfo->cb       = alarminfo->cb;
      cbinfo->priv     = alarminfo->priv;

      /* Convert from Julian calendar time to epoch time */

      tp.tv_sec = timegm((struct tm *)&alarminfo->time) ;

      /* And set the alarm */

      ret = kinetis_rtc_setalarm(&tp, kinetis_alarm_callback);
      if (ret < 0)
        {
          cbinfo->cb   = NULL;
          cbinfo->priv = NULL;
        }
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: kinetis_setrelative
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
static int
kinetis_setrelative(struct rtc_lowerhalf_s *lower,
                    const struct lower_setrelative_s *alarminfo)
{
  struct lower_setalarm_s setalarm;
#if defined(CONFIG_RTC_DATETIME)
  struct tm time;
#endif
  time_t seconds;
  struct timespec ts;
  int ret = -EINVAL;

  DEBUGASSERT(lower != NULL && alarminfo != NULL);
  DEBUGASSERT(alarminfo->id == RTC_ALARMA);

  if (alarminfo->id == RTC_ALARMA &&
      alarminfo->reltime > 0)
    {
      /* Disable pre-emption while we do this so that we don't have to worry
       * about being suspended and working on an old time.
       */

      sched_lock();

#if defined(CONFIG_RTC_DATETIME)
      /* Get the broken out time and convert to seconds */

      ret = up_rtc_getdatetime(&time);
      if (ret < 0)
        {
          sched_unlock();
          return ret;
        }

      ts.tv_sec  = timegm(&time);
      ts.tv_nsec = 0;
#else
      /* Get the current time in broken out format */

      ret = up_rtc_gettime(&ts);
      if (ret < 0)
        {
          sched_unlock();
          return ret;
        }
#endif

      /* Convert to seconds since the epoch */

      seconds = ts.tv_sec;

      /* Add the seconds offset.  Add one to the number of seconds
       * because we are unsure of the phase of the timer.
       */

      seconds += (alarminfo->reltime + 1);

      /* And convert the time back to Julian/broken out format */

      gmtime_r(&seconds, (struct tm *)&setalarm.time);

      /* The set the alarm using this absolute time */

      setalarm.id   = alarminfo->id;
      setalarm.cb   = alarminfo->cb;
      setalarm.priv = alarminfo->priv;

      ret = kinetis_setalarm(lower, &setalarm);

      sched_unlock();
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: kinetis_cancelalarm
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
static int
kinetis_cancelalarm(struct rtc_lowerhalf_s *lower, int alarmid)
{
  struct kinetis_lowerhalf_s *priv;
  struct kinetis_cbinfo_s *cbinfo;
  int ret = -EINVAL;

  DEBUGASSERT(lower != NULL);
  DEBUGASSERT(alarmid == RTC_ALARMA);
  priv = (struct kinetis_lowerhalf_s *)lower;

  /* ID0-> Alarm A */

  if (alarmid == RTC_ALARMA)
    {
      /* Nullify callback information to reduce window for race conditions */

      cbinfo       = &priv->cbinfo;
      cbinfo->cb   = NULL;
      cbinfo->priv = NULL;

      /* Then cancel the alarm */

      ret = kinetis_rtc_cancelalarm();
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: kinetis_rdalarm
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
static int kinetis_rdalarm(struct rtc_lowerhalf_s *lower,
                           struct lower_rdalarm_s *alarminfo)
{
  struct timespec ts;
  int ret = -EINVAL;

  DEBUGASSERT(lower != NULL && alarminfo != NULL && alarminfo->time != NULL);
  DEBUGASSERT(alarminfo->id == RTC_ALARMA);

  if (alarminfo->id == RTC_ALARMA)
    {
      /* Disable pre-emption while we do this so that we don't have to worry
       * about being suspended and working on an old time.
       */

      sched_lock();
      ret = kinetis_rtc_rdalarm(&ts);

      localtime_r((const time_t *)&ts.tv_sec,
                  (struct tm *)alarminfo->time);
      sched_unlock();
    }

  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kinetis_rtc_lowerhalf
 *
 * Description:
 *   Instantiate the RTC lower half driver for the Kinetis.  General usage:
 *
 *     #include <nuttx/timers/rtc.h>
 *     #include "kinetis_rtc.h>
 *
 *     struct rtc_lowerhalf_s *lower;
 *     lower = kinetis_rtc_lowerhalf();
 *     rtc_initialize(0, lower);
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, a non-NULL RTC lower interface is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

struct rtc_lowerhalf_s *kinetis_rtc_lowerhalf(void)
{
  return (struct rtc_lowerhalf_s *)&g_rtc_lowerhalf;
}

#endif /* CONFIG_RTC_DRIVER */
