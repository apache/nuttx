/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_rtc_lowerhalf.c
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
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/timers/rtc.h>

#include "chip.h"
#include "cxd56_rtc.h"

#ifdef CONFIG_RTC_DRIVER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
struct cxd56_cbinfo_s
{
  volatile rtc_alarm_callback_t cb; /* Callback when the alarm expires */
  volatile void *priv;              /* Private argurment to accompany callback */
};
#endif

/* This is the private type for the RTC state.  It must be cast compatible
 * with struct rtc_lowerhalf_s.
 */

struct cxd56_lowerhalf_s
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

  struct cxd56_cbinfo_s cbinfo[RTC_ALARM_LAST];
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Prototypes for static methods in struct rtc_ops_s */

static int cxd56_rdtime(struct rtc_lowerhalf_s *lower,
                        struct rtc_time *rtctime);
static int cxd56_settime(struct rtc_lowerhalf_s *lower,
                         const struct rtc_time *rtctime);

#ifdef CONFIG_RTC_ALARM
static int cxd56_setalarm(struct rtc_lowerhalf_s *lower,
                          const struct lower_setalarm_s *alarminfo);
static int cxd56_setrelative(struct rtc_lowerhalf_s *lower,
                          const struct lower_setrelative_s *alarminfo);
static int cxd56_cancelalarm(struct rtc_lowerhalf_s *lower,
                             int alarmid);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* CXD56 RTC driver operations */

static const struct rtc_ops_s g_rtc_ops =
{
  .rdtime      = cxd56_rdtime,
  .settime     = cxd56_settime,
#ifdef CONFIG_RTC_ALARM
  .setalarm    = cxd56_setalarm,
  .setrelative = cxd56_setrelative,
  .cancelalarm = cxd56_cancelalarm,
#endif
};

/* CXD56 RTC device state */

static struct cxd56_lowerhalf_s g_rtc_lowerhalf =
{
  .ops         = &g_rtc_ops,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_alarm_callback
 *
 * Description:
 *   This is the function that is called from the RTC driver when the alarm
 *   goes off.  It just invokes the upper half drivers callback.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static void cxd56_alarm_callback(void *arg, unsigned int alarmid)
{
  struct cxd56_lowerhalf_s *lower;
  struct cxd56_cbinfo_s *cbinfo;
  rtc_alarm_callback_t cb;
  void *priv;

  DEBUGASSERT((RTC_ALARM0 <= alarmid) && (alarmid < RTC_ALARM_LAST));

  lower        = (struct cxd56_lowerhalf_s *)arg;
  cbinfo       = &lower->cbinfo[alarmid];

  /* Sample and clear the callback information to minimize the window in
   * time in which race conditions can occur.
   */

  cb           = (rtc_alarm_callback_t)cbinfo->cb;
  priv         = (void *)cbinfo->priv;

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
 * Name: cxd56_rdtime
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

static int cxd56_rdtime(struct rtc_lowerhalf_s *lower,
                        struct rtc_time *rtctime)
{
#if defined(CONFIG_RTC_HIRES)
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
 * Name: cxd56_settime
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

static int cxd56_settime(struct rtc_lowerhalf_s *lower,
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
 * Name: cxd56_setalarm
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
static int cxd56_setalarm(struct rtc_lowerhalf_s *lower,
                          const struct lower_setalarm_s *alarminfo)
{
  struct cxd56_lowerhalf_s *priv;
  struct cxd56_cbinfo_s *cbinfo;
  struct alm_setalarm_s lowerinfo;
  int ret = -EINVAL;

  DEBUGASSERT(lower != NULL && alarminfo != NULL);
  DEBUGASSERT(RTC_ALARM0 == alarminfo->id);

  priv = (struct cxd56_lowerhalf_s *)lower;

  if (RTC_ALARM0 == alarminfo->id)
    {
      /* Remember the callback information */

      cbinfo            = &priv->cbinfo[alarminfo->id];
      cbinfo->cb        = alarminfo->cb;
      cbinfo->priv      = alarminfo->priv;

      /* Set the alarm */

      lowerinfo.as_id   = alarminfo->id;
      lowerinfo.as_cb   = cxd56_alarm_callback;
      lowerinfo.as_arg  = priv;

      /* Convert the RTC time to a timespec (1 second accuracy) */

      lowerinfo.as_time.tv_sec  = timegm((struct tm *)&alarminfo->time);
      lowerinfo.as_time.tv_nsec = 0;

      /* And set the alarm */

      ret = cxd56_rtc_setalarm(&lowerinfo);
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
 * Name: cxd56_setrelative
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
static int cxd56_setrelative(struct rtc_lowerhalf_s *lower,
                             const struct lower_setrelative_s *alarminfo)
{
  struct lower_setalarm_s setalarm;
  struct timespec ts;
  time_t seconds;
  int ret = -EINVAL;

  DEBUGASSERT(lower != NULL && alarminfo != NULL);
  DEBUGASSERT((RTC_ALARM0 <= alarminfo->id) &&
              (alarminfo->id < RTC_ALARM_LAST));

  if ((alarminfo->id == RTC_ALARM0) && (alarminfo->reltime > 0))
    {
      /* Disable pre-emption while we do this so that we don't have to worry
       * about being suspended and working on an old time.
       */

      sched_lock();

#if defined(CONFIG_RTC_HIRES)
      /* Get the higher resolution time */

      ret = up_rtc_gettime(&ts);
      if (ret < 0)
        {
          sched_unlock();
          return ret;
        }
#else
      /* The resolution of time is only 1 second */

      ts.tv_sec  = up_rtc_time();
      ts.tv_nsec = 0;
#endif

      /* Add the seconds offset.  Add one to the number of seconds because
       * we are unsure of the phase of the timer.
       */

      seconds = ts.tv_sec + (alarminfo->reltime + 1);

      gmtime_r(&seconds, (struct tm *)&setalarm.time);

      /* The set the alarm using this absolute time */

      setalarm.id   = alarminfo->id;
      setalarm.cb   = alarminfo->cb;
      setalarm.priv = alarminfo->priv;

      ret = cxd56_setalarm(lower, &setalarm);

      sched_unlock();
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: cxd56_cancelalarm
 *
 * Description:
 *   Cancel the current alarm.  This function implements the cancelalarm()
 *   method of the RTC driver interface
 *
 * Input Parameters:
 *   lower - A reference to RTC lower half driver state structure
 *   alarmid - the alarm id
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int cxd56_cancelalarm(struct rtc_lowerhalf_s *lower, int alarmid)
{
  struct cxd56_lowerhalf_s *priv;
  struct cxd56_cbinfo_s *cbinfo;
  int ret = -EINVAL;

  DEBUGASSERT(lower != NULL);
  DEBUGASSERT((RTC_ALARM0 <= alarmid) && (alarmid < RTC_ALARM_LAST));

  priv = (struct cxd56_lowerhalf_s *)lower;

  if ((RTC_ALARM0 <= alarmid) && (alarmid < RTC_ALARM_LAST))
    {
      /* Nullify callback information to reduce window for race conditions */

      cbinfo       = &priv->cbinfo[alarmid];
      cbinfo->cb   = NULL;
      cbinfo->priv = NULL;

      /* Then cancel the alarm */

      ret = cxd56_rtc_cancelalarm((enum alm_id_e)alarmid);
    }

  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_rtc_lowerhalf
 *
 * Description:
 *   Instantiate the RTC lower half driver for the CXD56.  General usage:
 *
 *     #include <nuttx/timers/rtc.h>
 *     #include "cxd56_rtc.h"
 *
 *     struct rtc_lowerhalf_s *lower;
 *     lower = cxd56_rtc_lowerhalf();
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

struct rtc_lowerhalf_s *cxd56_rtc_lowerhalf(void)
{
  return (struct rtc_lowerhalf_s *)&g_rtc_lowerhalf;
}

#endif /* CONFIG_RTC_DRIVER */
