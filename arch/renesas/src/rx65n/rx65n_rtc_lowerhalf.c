/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_rtc_lowerhalf.c
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
#include <nuttx/mutex.h>
#include <nuttx/timers/rtc.h>
#include "chip.h"
#include <rx65n_rtc.h>
#include "up_internal.h"

#ifdef CONFIG_RTC_DRIVER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

# define rx65n_getreg(addr)      getreg8(addr)
# define rx65n_putreg(val,addr)  putreg8(val,addr)
# define RX65N_NALARMS           1
/* Configuration ************************************************************/

#if defined(CONFIG_RTC_ALARM) && !defined(CONFIG_SCHED_WORKQUEUE)
#  error CONFIG_RTC_ALARM requires CONFIG_SCHED_WORKQUEUE
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

 #ifdef CONFIG_RTC_ALARM
struct rx65n_cbinfo_s
{
  volatile rtc_alarm_callback_t cb;  /* Callback when the alarm expires */
  volatile FAR void *priv;           /* Private argument to accompany callback */
  uint8_t id;                        /* Identifies the alarm */
};
#endif

/* This is the private type for the RTC state.  It must be cast compatible
 * with struct rtc_lowerhalf_s.
 */

struct rx65n_lowerhalf_s
{
  /* This is the contained reference to the read-only, lower-half
   * operations vtable (which may lie in FLASH or ROM)
   */

  FAR const struct rtc_ops_s *ops;

  /* Data following is private to this driver and not visible outside of
   * this file.
   */

  mutex_t devlock;      /* Threads can only exclusively access the RTC */

#ifdef CONFIG_RTC_ALARM
  /* Alarm callback information */

  struct rx65n_cbinfo_s cbinfo[RX65N_NALARMS];
#endif

#ifdef CONFIG_RTC_PERIODIC
  /* Periodic wakeup information */

  struct lower_setperiodic_s periodic;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Prototypes for static methods in struct rtc_ops_s */

static int rx65n_rdtime(FAR struct rtc_lowerhalf_s *lower,
                          FAR struct rtc_time *rtctime);
static int rx65n_settime(FAR struct rtc_lowerhalf_s *lower,
                           FAR const struct rtc_time *rtctime);
static bool rx65n_havesettime(FAR struct rtc_lowerhalf_s *lower);

#ifdef CONFIG_RTC_ALARM
static int rx65n_setalarm(FAR struct rtc_lowerhalf_s *lower,
                            FAR const struct lower_setalarm_s *alarminfo);
static int rx65n_setrelative(FAR struct rtc_lowerhalf_s *lower,
                             FAR const struct lower_setrelative_s
                                                         *alarminfo);
static int rx65n_cancelalarm(FAR struct rtc_lowerhalf_s *lower,
                               int alarmid);
static int rx65n_rdalarm(FAR struct rtc_lowerhalf_s *lower,
                           FAR struct lower_rdalarm_s *alarminfo);
#endif

#ifdef CONFIG_RTC_PERIODIC
static int rx65n_setperiodic(FAR struct rtc_lowerhalf_s *lower,
                             FAR const struct lower_setperiodic_s
                                                         *alarminfo);
static int rx65n_cancelperiodic(FAR struct rtc_lowerhalf_s *lower, int id);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* RX65N RTC driver operations */

static const struct rtc_ops_s g_rtc_ops =
{
  .rdtime      = rx65n_rdtime,
  .settime     = rx65n_settime,
  .havesettime = rx65n_havesettime,
#ifdef CONFIG_RTC_ALARM
  .setalarm    = rx65n_setalarm,
  .setrelative = rx65n_setrelative,
  .cancelalarm = rx65n_cancelalarm,
  .rdalarm     = rx65n_rdalarm,
#endif
#ifdef CONFIG_RTC_PERIODIC
  .setperiodic    = rx65n_setperiodic,
  .cancelperiodic = rx65n_cancelperiodic,
#endif
#ifdef CONFIG_RTC_IOCTL
  .ioctl       = NULL,
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  .destroy     = NULL,
#endif
};

/* RX65N RTC device state */

static struct rx65n_lowerhalf_s g_rtc_lowerhalf =
{
  .ops         = &g_rtc_ops,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rx65n_alarm_callback
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
static void rx65n_alarm_callback(FAR void *arg, unsigned int alarmid)
{
  FAR struct rx65n_lowerhalf_s *lower;
  FAR struct rx65n_cbinfo_s *cbinfo;
  rtc_alarm_callback_t cb;
  FAR void *priv;

  DEBUGASSERT(arg != NULL);

  lower        = (struct rx65n_lowerhalf_s *)arg;
  cbinfo       = &lower->cbinfo[alarmid];

  /* Sample and clear the callback information to minimize the window in
   * time in which race conditions can occur.
   */

  cb           = (rtc_alarm_callback_t)cbinfo->cb;
  priv         = (FAR void *)cbinfo->priv;

  cbinfo->cb   = NULL;
  cbinfo->priv = NULL;

  /* Perform the callback */

  if (cb != NULL)
    {
      cb(priv, alarmid);
    }
}
#endif /* CONFIG_RTC_ALARM */

/****************************************************************************
 * Name: rx65n_rdtime
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

static int rx65n_rdtime(FAR struct rtc_lowerhalf_s *lower,
                          FAR struct rtc_time *rtctime)
{
#if defined(CONFIG_RTC_DATETIME)
  /* This operation depends on the fact that struct rtc_time is cast
   * compatible with struct tm.
   */

  int ret;
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
#endif

  return OK;

errout:
  DEBUGASSERT(ret < 0);
  return ret;
}

/****************************************************************************
 * Name: rx65n_settime
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

static int rx65n_settime(FAR struct rtc_lowerhalf_s *lower,
                           FAR const struct rtc_time *rtctime)
{
#ifdef CONFIG_RTC_DATETIME
  /* This operation depends on the fact that struct rtc_time is cast
   * compatible with struct tm.
   */

  return rx65n_rtc_setdatetime((FAR const struct tm *)rtctime);

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
 * Name: rx65n_havesettime
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

static bool rx65n_havesettime(FAR struct rtc_lowerhalf_s *lower)
{
  return true ;
}

/****************************************************************************
 * Name: rx65n_setalarm
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
static int rx65n_setalarm(FAR struct rtc_lowerhalf_s *lower,
                            FAR const struct lower_setalarm_s *alarminfo)
{
  FAR struct rx65n_lowerhalf_s *priv;
  FAR struct rx65n_cbinfo_s *cbinfo;
  struct alm_setalarm_s lowerinfo;
  int ret;

  DEBUGASSERT(lower != NULL && alarminfo != NULL && alarminfo->id == 0);
  priv = (FAR struct rx65n_lowerhalf_s *)lower;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  ret = -EINVAL;
  if (alarminfo->id == 0)
    {
      cbinfo            = &priv->cbinfo[0];
      cbinfo->cb        = alarminfo->cb;
      cbinfo->priv      = alarminfo->priv;
      cbinfo->id        = alarminfo->id;

      /* Set the alarm */

      lowerinfo.as_id   = alarminfo->id;
      lowerinfo.as_cb   = rx65n_alarm_callback;
      lowerinfo.as_arg  = priv;
      memcpy(&lowerinfo.as_time, &alarminfo->time, sizeof(struct tm));

      /* And set the alarm */

      ret = rx65n_rtc_setalarm(&lowerinfo);
      if (ret < 0)
        {
          cbinfo->cb   = NULL;
          cbinfo->priv = NULL;
        }
    }

  nxmutex_unlock(&priv->devlock);
  return ret;
}
#endif

/****************************************************************************
 * Name: rx65n_setrelative
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
static int rx65n_setrelative(FAR struct rtc_lowerhalf_s *lower,
                             FAR const struct lower_setrelative_s
                                                         *alarminfo)
{
  struct lower_setalarm_s setalarm;
  struct tm time;
  struct timespec rtc_time;
  time_t seconds;
  int ret = -EINVAL;

  DEBUGASSERT(lower != NULL && alarminfo != NULL);

  if ((alarminfo->id >= 0) && alarminfo->reltime > 0)
    {
      /* Disable pre-emption while we do this so that we don't have to worry
       * about being suspended and working on an old time.
       */

      sched_lock();

#if defined(CONFIG_RTC_DATETIME)
      /* Get the broken out time and convert to seconds */

      struct timespec ts;
      ret = up_rtc_getdatetime(&time);
      if (ret < 0)
        {
          sched_unlock();
          return ret;
        }

      ts.tv_sec  = timegm(&time);
      ts.tv_nsec = 0;

#elif defined(CONFIG_RTC_HIRES)
      /* Get the higher resolution time */

      ret = up_rtc_gettime(&rtc_time);
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

          setalarm.id   = alarminfo->id;
          setalarm.cb   = alarminfo->cb;
          setalarm.priv = alarminfo->priv;

          ret = rx65n_setalarm(lower, &setalarm);
        }
#else
      /* The resolution of time is only 1 second */

      ts.tv_sec  = up_rtc_time();
      ts.tv_nsec = 0;
#endif

      /* Remember the callback information */

      sched_unlock();
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: rx65n_cancelalarm
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
static int rx65n_cancelalarm(FAR struct rtc_lowerhalf_s *lower, int alarmid)
{
  FAR struct rx65n_lowerhalf_s *priv;
  FAR struct rx65n_cbinfo_s *cbinfo;

  DEBUGASSERT(lower != NULL);
  DEBUGASSERT(alarmid == 0);
  priv = (FAR struct rx65n_lowerhalf_s *)lower;

  /* Nullify callback information to reduce window for race conditions */

  cbinfo       = &priv->cbinfo[0];
  cbinfo->cb   = NULL;
  cbinfo->priv = NULL;

  /* Then cancel the alarm */

  return rx65n_rtc_cancelalarm();
}
#endif

/****************************************************************************
 * Name: rx65n_rdalarm
 *
 * Description:
 *   Query the RTC alarm.
 *
 * Input Parameters:
 *   lower - A reference to RTC lower half driver state structure
 *   alarminfo - Provided information needed to query the alarm
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int rx65n_rdalarm(FAR struct rtc_lowerhalf_s *lower,
                           FAR struct lower_rdalarm_s *alarminfo)
{
  struct alm_rdalarm_s lowerinfo;
  int ret = -EINVAL;

  DEBUGASSERT(lower != NULL && alarminfo != NULL && alarminfo->time != NULL);
  if (alarminfo->id >= 0)
    {
      /* Disable pre-emption while we do this so that we don't have to worry
       * about being suspended and working on an old time.
       */

      sched_lock();

      lowerinfo.ar_id = alarminfo->id;
      lowerinfo.ar_time = alarminfo->time;

      ret = rx65n_rtc_rdalarm(&lowerinfo);

      sched_unlock();
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: rx65n_periodic_callback
 *
 * Description:
 *   This is the function that is called from the RTC driver when the
 *   periodic wakeup goes off.  It just invokes the upper half drivers
 *   callback.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_PERIODIC
static int rx65n_periodic_callback(void)
{
  FAR struct rx65n_lowerhalf_s *lower;
  struct lower_setperiodic_s *cbinfo;
  periodiccb_t cb;
  FAR void *priv;

  lower = (FAR struct rx65n_lowerhalf_s *)&g_rtc_lowerhalf;

  cbinfo = &lower->periodic;
  cb     = (periodiccb_t)cbinfo->cb;
  priv   = (FAR void *)cbinfo->priv;

  /* Perform the callback */

  if (cb != NULL)
    {
      cb(priv, 0);
    }

  return OK;
}
#endif /* CONFIG_RTC_PERIODIC */

/****************************************************************************
 * Name: rx65n_setperiodic
 *
 * Description:
 *   Set a new periodic wakeup relative to the current time, with a given
 *   period. This function implements the setperiodic() method of the RTC
 *   driver interface
 *
 * Input Parameters:
 *   lower - A reference to RTC lower half driver state structure
 *   alarminfo - Provided information needed to set the wakeup activity
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_PERIODIC
static int rx65n_setperiodic(FAR struct rtc_lowerhalf_s *lower,
                             FAR const struct lower_setperiodic_s
                                                         *alarminfo)
{
  FAR struct rx65n_lowerhalf_s *priv;
  int ret;

  DEBUGASSERT(lower != NULL && alarminfo != NULL);
  priv = (FAR struct rx65n_lowerhalf_s *)lower;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  memcpy(&priv->periodic, alarminfo, sizeof(struct lower_setperiodic_s));
  ret = rx65n_rtc_setperiodic(&alarminfo->period,
                              (periodiccb_t)rx65n_periodic_callback);

  nxmutex_unlock(&priv->devlock);
  return ret;
}
#endif

/****************************************************************************
 * Name: rx65n_cancelperiodic
 *
 * Description:
 *   Cancel the current periodic wakeup activity.  This function implements
 *   the cancelperiodic() method of the RTC driver interface
 *
 * Input Parameters:
 *   lower - A reference to RTC lower half driver state structure
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_PERIODIC
static int rx65n_cancelperiodic(FAR struct rtc_lowerhalf_s *lower, int id)
{
  FAR struct rx65n_lowerhalf_s *priv;
  int ret;

  DEBUGASSERT(lower != NULL);
  priv = (FAR struct rx65n_lowerhalf_s *)lower;

  DEBUGASSERT(id == 0);

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  ret = rx65n_rtc_cancelperiodic();

  nxmutex_unlock(&priv->devlock);
  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rx65n_rtc_lowerhalf
 *
 * Description:
 *   Instantiate the RTC lower half driver for the RX65N.  General usage:
 *
 *     #include <nuttx/timers/rtc.h>
 *     #include "rx65n_rtc.h>
 *
 *     struct rtc_lowerhalf_s *lower;
 *     lower = rx65n_rtc_lowerhalf();
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

FAR struct rtc_lowerhalf_s *rx65n_rtc_lowerhalf(void)
{
  nxmutex_init(&g_rtc_lowerhalf.devlock);
  return (FAR struct rtc_lowerhalf_s *)&g_rtc_lowerhalf;
}

#endif /* CONFIG_RTC_DRIVER */

