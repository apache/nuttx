/****************************************************************************
 * arch/z80/src/ez80/ez80_rtc_lowerhalf.c
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

#include <nuttx/arch.h>
#include <nuttx/timers/rtc.h>

#include "chip.h"
#include "ez80_rtc.h"

#ifdef CONFIG_RTC_DRIVER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
struct ez80_cbinfo_s
{
  volatile rtc_alarm_callback_t cb;  /* Callback when the alarm expires */
  volatile FAR void *priv;           /* Private argument to accompany callback */
};
#endif

/* This is the private type for the RTC state.  It must be cast compatible
 * with struct rtc_lowerhalf_s.
 */

struct ez80_lowerhalf_s
{
  /* This is the contained reference to the read-only, lower-half
   * operations vtable (which may lie in FLASH or ROM)
   */

  FAR const struct rtc_ops_s *ops;

  /* Data following is private to this driver and not visible outside of
   * this file.
   */

  sem_t devsem;         /* Threads can only exclusively access the RTC */

#ifdef CONFIG_RTC_ALARM
  /* Alarm callback information */

  struct ez80_cbinfo_s cbinfo;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Prototypes for static methods in struct rtc_ops_s */

static int ez80_rdtime(FAR struct rtc_lowerhalf_s *lower,
             FAR struct rtc_time *rtctime);
static int ez80_settime(FAR struct rtc_lowerhalf_s *lower,
             FAR const struct rtc_time *rtctime);
static bool ez80_havesettime(FAR struct rtc_lowerhalf_s *lower);

#ifdef CONFIG_RTC_ALARM
static int ez80_setalarm(FAR struct rtc_lowerhalf_s *lower,
             FAR const struct lower_setalarm_s *alarminfo);
static int ez80_setrelative(FAR struct rtc_lowerhalf_s *lower,
             FAR const struct lower_setrelative_s *alarminfo);
static int ez80_cancelalarm(FAR struct rtc_lowerhalf_s *lower, int alarmid);
static int ez80_rdalarm(FAR struct rtc_lowerhalf_s *lower,
             FAR struct lower_rdalarm_s *alarminfo);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* eZ80 RTC driver operations */

static const struct rtc_ops_s g_rtc_ops =
{
    ez80_rdtime,        /* rdtime */
    ez80_settime,       /* settime */
    ez80_havesettime    /* havesettime */
#ifdef CONFIG_RTC_ALARM
  , ez80_setalarm,      /* setalarm */
    ez80_setrelative,   /* setrelative */
    ez80_cancelalarm,   /* cancelalarm */
    ez80_rdalarm        /* rdalarm */
#endif
#ifdef CONFIG_RTC_PERIODIC
  , NULL,               /* setperiodic */
    NULL                /* cancelperiodic */
#endif
#ifdef CONFIG_RTC_IOCTL
  , NULL                /* ioctl */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL                /* destroy */
#endif
};

/* eZ80 RTC device state */

static struct ez80_lowerhalf_s g_rtc_lowerhalf =
{
  &g_rtc_ops           /* ops */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ez80_alarm_callback
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
static void ez80_alarm_callback(FAR void *arg)
{
  FAR struct ez80_lowerhalf_s *lower;
  FAR struct ez80_cbinfo_s *cbinfo;
  rtc_alarm_callback_t cb;
  FAR void *priv;

  lower        = (struct ez80_lowerhalf_s *)arg;
  cbinfo       = &lower->cbinfo;

  /* Sample and clear the callback information to minimize the window in
   * time in which race conditions can occur.
   */

  cb           = (rtc_alarm_callback_t)cbinfo->cb;
  priv         = (FAR void *)cbinfo->priv;
  DEBUGASSERT(priv != NULL);

  cbinfo->cb   = NULL;
  cbinfo->priv = NULL;

  /* Perform the callback */

  if (cb != NULL)
    {
      cb(priv, 0);
    }
}
#endif

/****************************************************************************
 * Name: ez80_rdtime
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

static int ez80_rdtime(FAR struct rtc_lowerhalf_s *lower,
                        FAR struct rtc_time *rtctime)
{
#if defined(CONFIG_RTC_DATETIME)
  /* This operation depends on the fact that struct rtc_time is cast
   * compatible with struct tm.
   */

  return up_rtc_getdatetime((FAR struct tm *)rtctime);

#elif defined(CONFIG_RTC_HIRES)
  FAR struct timespec ts;
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

  if (!gmtime_r(&ts.tv_sec, (FAR struct tm *)rtctime))
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

  if (!gmtime_r(&timer, (FAR struct tm *)rtctime))
    {
      int errcode = get_errno();
      DEBUGASSERT(errcode > 0);
      return -errcode;
    }

  return OK;
#endif
}

/****************************************************************************
 * Name: ez80_settime
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

static int ez80_settime(FAR struct rtc_lowerhalf_s *lower,
                         FAR const struct rtc_time *rtctime)
{
#ifdef CONFIG_RTC_DATETIME
  /* This operation depends on the fact that struct rtc_time is cast
   * compatible with struct tm.
   */

  return ez80_rtc_setdatetime((FAR const struct tm *)rtctime);

#else
  struct timespec ts;

  /* Convert the struct rtc_time to a time_t.  Here we assume that struct
   * rtc_time is cast compatible with struct tm.
   */

  ts.tv_sec  = timegm((FAR struct tm *)rtctime);
  ts.tv_nsec = 0;

  /* Now set the time (to one second accuracy) */

  return up_rtc_settime(&ts);
#endif
}

/****************************************************************************
 * Name: ez80_havesettime
 *
 * Description:
 *   Implements the havesettime() method of the RTC driver interface
 *
 * Input Parameters:
 *   lower - A reference to RTC lower half driver state structure
 *
 * Returned Value:
 *   Returns true if RTC date-time have been previously set.
 *
 ****************************************************************************/

static bool ez80_havesettime(FAR struct rtc_lowerhalf_s *lower)
{
  return true;
}

/****************************************************************************
 * Name: ez80_setalarm
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
static int ez80_setalarm(FAR struct rtc_lowerhalf_s *lower,
                          FAR const struct lower_setalarm_s *alarminfo)
{
  FAR struct ez80_lowerhalf_s *priv;
  FAR struct ez80_cbinfo_s *cbinfo;
  struct alm_setalarm_s lowerinfo;
  int ret;

  /* ID0-> Alarm A; ID1 -> Alarm B */

  DEBUGASSERT(lower != NULL && alarminfo != NULL);
  DEBUGASSERT(alarminfo->id == 0);

  priv = (FAR struct ez80_lowerhalf_s *)lower;

  ret = nxsem_wait(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Remember the callback information */

  cbinfo            = &priv->cbinfo;
  cbinfo->cb        = alarminfo->cb;
  cbinfo->priv      = alarminfo->priv;

  /* Set the alarm */

  lowerinfo.as_cb   = ez80_alarm_callback;
  lowerinfo.as_arg  = priv;
  memcpy(&lowerinfo.as_time, &alarminfo->time, sizeof(struct tm));

  /* And set the alarm */

  ret = ez80_rtc_setalarm(&lowerinfo);
  if (ret < 0)
    {
      cbinfo->cb   = NULL;
      cbinfo->priv = NULL;
    }

  nxsem_post(&priv->devsem);

  return ret;
}
#endif

/****************************************************************************
 * Name: ez80_setrelative
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
static int ez80_setrelative(FAR struct rtc_lowerhalf_s *lower,
                             FAR const struct lower_setrelative_s *alarminfo)
{
  struct lower_setalarm_s setalarm;
  struct tm time;
  time_t seconds;
  int ret = -EINVAL;

  DEBUGASSERT(lower != NULL && alarminfo != NULL && alarminfo->id == 0);

  /* Disable pre-emption while we do this so that we don't have to worry
   * about being suspended and working on an old time.
   */

  sched_lock();

  /* Get the current time in broken out format */

  ret = up_rtc_getdatetime(&time);
  if (ret >= 0)
    {
      /* Convert to seconds since the epoch */

      seconds = timegm(&time);

      /* Add the seconds offset.  Add one to the number of seconds
       * because we are unsure of the phase of the timer.
       */

      seconds += (alarminfo->reltime + 1);

      /* And convert the time back to broken out format */

      gmtime_r(&seconds, (FAR struct tm *)&setalarm.time);

      /* The set the alarm using this absolute time */

      setalarm.cb   = alarminfo->cb;
      setalarm.priv = alarminfo->priv;

      ret = ez80_setalarm(lower, &setalarm);
    }

  sched_unlock();
  return ret;
}
#endif

/****************************************************************************
 * Name: ez80_cancelalarm
 *
 * Description:
 *   Cancel the current alarm.  This function implements the cancelalarm()
 *   method of the RTC driver interface
 *
 * Input Parameters:
 *   lower   - A reference to RTC lower half driver state structure
 *   alarmid - Must be zero
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int ez80_cancelalarm(FAR struct rtc_lowerhalf_s *lower, int alarmid)
{
  FAR struct ez80_lowerhalf_s *priv;
  FAR struct ez80_cbinfo_s *cbinfo;
  int ret;

  DEBUGASSERT(lower != NULL && alarmid == 0);

  priv = (FAR struct ez80_lowerhalf_s *)lower;

  ret = nxsem_wait(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Nullify callback information to reduce window for race conditions */

  cbinfo       = &priv->cbinfo;
  cbinfo->cb   = NULL;
  cbinfo->priv = NULL;

  /* Then cancel the alarm */

  ret = ez80_rtc_cancelalarm();
  nxsem_post(&priv->devsem);

  return ret;
}
#endif

/****************************************************************************
 * Name: ez80_rdalarm
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
static int ez80_rdalarm(FAR struct rtc_lowerhalf_s *lower,
                        FAR struct lower_rdalarm_s *alarminfo)
{
  int ret = -EINVAL;

  DEBUGASSERT(lower != NULL && alarminfo != NULL &&
              alarminfo->time != NULL && alarminfo->id == 0);

  /* Disable pre-emption while we do this so that we don't have to worry
   * about being suspended and working on an old time.
   */

  sched_lock();
  ret = ez80_rtc_rdalarm((FAR struct tm *)alarminfo->time);
  sched_unlock();

  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ez80_rtc_lowerhalf
 *
 * Description:
 *   Instantiate the RTC lower half driver for the eZ80.  General usage:
 *
 *     #include <nuttx/timers/rtc.h>
 *     #include "ez80_rtc.h>
 *
 *     struct rtc_lowerhalf_s *lower;
 *     lower = ez80_rtc_lowerhalf();
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

FAR struct rtc_lowerhalf_s *ez80_rtc_lowerhalf(void)
{
  nxsem_init(&g_rtc_lowerhalf.devsem, 0, 1);

  return (FAR struct rtc_lowerhalf_s *)&g_rtc_lowerhalf;
}

#endif /* CONFIG_RTC_DRIVER */
