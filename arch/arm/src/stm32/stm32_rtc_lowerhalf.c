/****************************************************************************
 * arch/arm/src/stm32/stm32_rtc_lowerhalf.c
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

#include "arm_internal.h"
#include "chip.h"
#include "stm32_rtc.h"

#ifdef CONFIG_RTC_DRIVER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_STM32_STM32F4XXX) || defined(CONFIG_STM32_STM32L15XX)
#  define STM32_NALARMS 2
#else
#  define STM32_NALARMS 1
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
struct stm32_cbinfo_s
{
  volatile rtc_alarm_callback_t cb; /* Callback when the alarm expires */
  volatile void *priv;              /* Private argument to accompany callback */
#if defined(CONFIG_STM32_STM32F4XXX) || defined(CONFIG_STM32_STM32L15XX)
  uint8_t id;                       /* Identifies the alarm */
#endif
};
#endif

/* This is the private type for the RTC state.  It must be cast compatible
 * with struct rtc_lowerhalf_s.
 */

struct stm32_lowerhalf_s
{
  /* This is the contained reference to the read-only, lower-half
   * operations vtable (which may lie in FLASH or ROM)
   */

  const struct rtc_ops_s *ops;

  /* Data following is private to this driver and not visible outside of
   * this file.
   */

  mutex_t devlock;      /* Threads can only exclusively access the RTC */

#ifdef CONFIG_RTC_ALARM
  /* Alarm callback information */

  struct stm32_cbinfo_s cbinfo[STM32_NALARMS];
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

static int stm32_rdtime(struct rtc_lowerhalf_s *lower,
                        struct rtc_time *rtctime);
static int stm32_settime(struct rtc_lowerhalf_s *lower,
                         const struct rtc_time *rtctime);
static bool stm32_havesettime(struct rtc_lowerhalf_s *lower);

#ifdef CONFIG_RTC_ALARM
static int stm32_setalarm(struct rtc_lowerhalf_s *lower,
                          const struct lower_setalarm_s *alarminfo);
static int stm32_setrelative(struct rtc_lowerhalf_s *lower,
                            const struct lower_setrelative_s *alarminfo);
static int stm32_cancelalarm(struct rtc_lowerhalf_s *lower,
                             int alarmid);
static int stm32_rdalarm(struct rtc_lowerhalf_s *lower,
                         struct lower_rdalarm_s *alarminfo);
#endif

#ifdef CONFIG_RTC_PERIODIC
static int stm32_setperiodic(struct rtc_lowerhalf_s *lower,
                             const struct lower_setperiodic_s *alarminfo);
static int stm32_cancelperiodic(struct rtc_lowerhalf_s *lower, int id);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* STM32 RTC driver operations */

static const struct rtc_ops_s g_rtc_ops =
{
  .rdtime      = stm32_rdtime,
  .settime     = stm32_settime,
  .havesettime = stm32_havesettime,
#ifdef CONFIG_RTC_ALARM
  .setalarm    = stm32_setalarm,
  .setrelative = stm32_setrelative,
  .cancelalarm = stm32_cancelalarm,
  .rdalarm     = stm32_rdalarm,
#endif
#ifdef CONFIG_RTC_PERIODIC
  .setperiodic    = stm32_setperiodic,
  .cancelperiodic = stm32_cancelperiodic,
#endif
#ifdef CONFIG_RTC_IOCTL
  .ioctl       = NULL,
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  .destroy     = NULL,
#endif
};

/* STM32 RTC device state */

static struct stm32_lowerhalf_s g_rtc_lowerhalf =
{
  .ops     = &g_rtc_ops,
  .devlock = NXMUTEX_INITIALIZER,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_alarm_callback
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
#if defined(CONFIG_STM32_STM32F4XXX) || defined(CONFIG_STM32_STM32L15XX)
static void stm32_alarm_callback(void *arg, unsigned int alarmid)
{
  struct stm32_lowerhalf_s *lower;
  struct stm32_cbinfo_s *cbinfo;
  rtc_alarm_callback_t cb;
  void *priv;

  DEBUGASSERT(alarmid == RTC_ALARMA || alarmid == RTC_ALARMB);

  lower        = (struct stm32_lowerhalf_s *)arg;
  cbinfo       = &lower->cbinfo[alarmid];

  /* Sample and clear the callback information to minimize the window in
   * time in which race conditions can occur.
   */

  cb           = (rtc_alarm_callback_t)cbinfo->cb;
  priv         = (void *)cbinfo->priv;
  DEBUGASSERT(priv != NULL);

  cbinfo->cb   = NULL;
  cbinfo->priv = NULL;

  /* Perform the callback */

  if (cb != NULL)
    {
      cb(priv, alarmid);
    }
}

#else
static void stm32_alarm_callback(void)
{
  struct stm32_cbinfo_s *cbinfo = &g_rtc_lowerhalf.cbinfo[0];

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

#endif /* CONFIG_STM32_STM32F4XXX || CONFIG_STM32_STM32L15XX */
#endif /* CONFIG_RTC_ALARM */

/****************************************************************************
 * Name: stm32_rdtime
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

static int stm32_rdtime(struct rtc_lowerhalf_s *lower,
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
 * Name: stm32_settime
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

static int stm32_settime(struct rtc_lowerhalf_s *lower,
                         const struct rtc_time *rtctime)
{
#ifdef CONFIG_RTC_DATETIME
  /* This operation depends on the fact that struct rtc_time is cast
   * compatible with struct tm.
   */

  return stm32_rtc_setdatetime((const struct tm *)rtctime);

#else
  struct timespec ts;

  /* Convert the struct rtc_time to a time_t.  Here we assume that struct
   * rtc_time is cast compatible with struct tm.
   */

  ts.tv_sec  = timegm((struct tm *)rtctime);
  ts.tv_nsec = 0;

  /* Now set the time (to one second accuracy) */

  return up_rtc_settime(&ts);
#endif
}

/****************************************************************************
 * Name: stm32_havesettime
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

static bool stm32_havesettime(struct rtc_lowerhalf_s *lower)
{
#if defined(CONFIG_STM32_STM32F10XX)
  return getreg16(RTC_MAGIC_REG) == RTC_MAGIC_TIME_SET;
#else
  return getreg32(RTC_MAGIC_REG) == RTC_MAGIC_TIME_SET;
#endif
}

/****************************************************************************
 * Name: stm32_setalarm
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
static int stm32_setalarm(struct rtc_lowerhalf_s *lower,
                          const struct lower_setalarm_s *alarminfo)
{
#if defined(CONFIG_STM32_STM32F4XXX) || defined(CONFIG_STM32_STM32L15XX)
  struct stm32_lowerhalf_s *priv;
  struct stm32_cbinfo_s *cbinfo;
  struct alm_setalarm_s lowerinfo;
  int ret;

  /* ID0-> Alarm A; ID1 -> Alarm B */

  DEBUGASSERT(lower != NULL && alarminfo != NULL);
  DEBUGASSERT(alarminfo->id == RTC_ALARMA || alarminfo->id == RTC_ALARMB);
  priv = (struct stm32_lowerhalf_s *)lower;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  ret = -EINVAL;
  if (alarminfo->id == RTC_ALARMA || alarminfo->id == RTC_ALARMB)
    {
      /* Remember the callback information */

      cbinfo            = &priv->cbinfo[alarminfo->id];
      cbinfo->cb        = alarminfo->cb;
      cbinfo->priv      = alarminfo->priv;
      cbinfo->id        = alarminfo->id;

      /* Set the alarm */

      lowerinfo.as_id   = alarminfo->id;
      lowerinfo.as_cb   = stm32_alarm_callback;
      lowerinfo.as_arg  = priv;
      memcpy(&lowerinfo.as_time, &alarminfo->time, sizeof(struct tm));

      /* And set the alarm */

      ret = stm32_rtc_setalarm(&lowerinfo);
      if (ret < 0)
        {
          cbinfo->cb   = NULL;
          cbinfo->priv = NULL;
        }
    }

  nxmutex_unlock(&priv->devlock);
  return ret;

#else
  struct stm32_lowerhalf_s *priv;
  struct stm32_cbinfo_s *cbinfo;
  int ret;

  DEBUGASSERT(lower != NULL && alarminfo != NULL && alarminfo->id == 0);
  priv = (struct stm32_lowerhalf_s *)lower;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
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

      cbinfo            = &priv->cbinfo[0];
      cbinfo->cb        = alarminfo->cb;
      cbinfo->priv      = alarminfo->priv;

      /* And set the alarm */

      ret = stm32_rtc_setalarm(&ts, stm32_alarm_callback);
      if (ret < 0)
        {
          cbinfo->cb   = NULL;
          cbinfo->priv = NULL;
        }
    }

  nxmutex_unlock(&priv->devlock);
  return ret;
#endif
}
#endif

/****************************************************************************
 * Name: stm32_setrelative
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
static int stm32_setrelative(struct rtc_lowerhalf_s *lower,
                             const struct lower_setrelative_s *alarminfo)
{
#if defined(CONFIG_STM32_STM32F4XXX) || defined(CONFIG_STM32_STM32L15XX)
  struct lower_setalarm_s setalarm;
  struct tm time;
  time_t seconds;
  int ret = -EINVAL;

  DEBUGASSERT(lower != NULL && alarminfo != NULL);
  DEBUGASSERT(alarminfo->id == RTC_ALARMA || alarminfo->id == RTC_ALARMB);

  if ((alarminfo->id == RTC_ALARMA || alarminfo->id == RTC_ALARMB) &&
      alarminfo->reltime > 0)
    {
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

          gmtime_r(&seconds, (struct tm *)&setalarm.time);

          /* The set the alarm using this absolute time */

          setalarm.id   = alarminfo->id;
          setalarm.cb   = alarminfo->cb;
          setalarm.priv = alarminfo->priv;

          ret = stm32_setalarm(lower, &setalarm);
        }

      sched_unlock();
    }

  return ret;

#else
  struct stm32_lowerhalf_s *priv;
  struct stm32_cbinfo_s *cbinfo;
#if defined(CONFIG_RTC_DATETIME)
  struct tm time;
#endif
  struct timespec ts;
  int ret = -EINVAL;

  DEBUGASSERT(lower != NULL && alarminfo != NULL && alarminfo->id == 0);
  priv = (struct stm32_lowerhalf_s *)lower;

  if (alarminfo->id == 0 && alarminfo->reltime > 0)
    {
      /* Disable pre-emption while we do this so that we don't have to worry
       * about being suspended and working on an old time.
       */

      sched_lock();

      /* Get the current time in seconds */

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

#elif defined(CONFIG_RTC_HIRES)
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

      ts.tv_sec   += (alarminfo->reltime + 1);

      /* Remember the callback information */

      cbinfo       = &priv->cbinfo[0];
      cbinfo->cb   = alarminfo->cb;
      cbinfo->priv = alarminfo->priv;

      /* And set the alarm */

      ret = stm32_rtc_setalarm(&ts, stm32_alarm_callback);
      if (ret < 0)
        {
          cbinfo->cb   = NULL;
          cbinfo->priv = NULL;
        }

      sched_unlock();
    }

  return ret;
#endif
}
#endif

/****************************************************************************
 * Name: stm32_cancelalarm
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
static int stm32_cancelalarm(struct rtc_lowerhalf_s *lower, int alarmid)
{
#if defined(CONFIG_STM32_STM32F4XXX) || defined(CONFIG_STM32_STM32L15XX)
  struct stm32_lowerhalf_s *priv;
  struct stm32_cbinfo_s *cbinfo;
  int ret;

  DEBUGASSERT(lower != NULL);
  DEBUGASSERT(alarmid == RTC_ALARMA || alarmid == RTC_ALARMB);
  priv = (struct stm32_lowerhalf_s *)lower;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  /* ID0-> Alarm A; ID1 -> Alarm B */

  ret = -EINVAL;
  if (alarmid == RTC_ALARMA || alarmid == RTC_ALARMB)
    {
      /* Nullify callback information to reduce window for race conditions */

      cbinfo       = &priv->cbinfo[alarmid];
      cbinfo->cb   = NULL;
      cbinfo->priv = NULL;

      /* Then cancel the alarm */

      ret = stm32_rtc_cancelalarm((enum alm_id_e)alarmid);
    }

  nxmutex_unlock(&priv->devlock);
  return ret;

#else
  struct stm32_lowerhalf_s *priv;
  struct stm32_cbinfo_s *cbinfo;

  DEBUGASSERT(lower != NULL);
  DEBUGASSERT(alarmid == 0);
  priv = (struct stm32_lowerhalf_s *)lower;

  /* Nullify callback information to reduce window for race conditions */

  cbinfo       = &priv->cbinfo[0];
  cbinfo->cb   = NULL;
  cbinfo->priv = NULL;

  /* Then cancel the alarm */

  return stm32_rtc_cancelalarm();
#endif
}
#endif

/****************************************************************************
 * Name: stm32_rdalarm
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
static int stm32_rdalarm(struct rtc_lowerhalf_s *lower,
                         struct lower_rdalarm_s *alarminfo)
{
  struct alm_rdalarm_s lowerinfo;
  int ret = -EINVAL;

  DEBUGASSERT(lower != NULL && alarminfo != NULL && alarminfo->time != NULL);
  DEBUGASSERT(alarminfo->id == RTC_ALARMA || alarminfo->id == RTC_ALARMB);

  if (alarminfo->id == RTC_ALARMA || alarminfo->id == RTC_ALARMB)
    {
      /* Disable pre-emption while we do this so that we don't have to worry
       * about being suspended and working on an old time.
       */

      sched_lock();

      lowerinfo.ar_id   = alarminfo->id;
      lowerinfo.ar_time = alarminfo->time;

      ret = stm32_rtc_rdalarm(&lowerinfo);

      sched_unlock();
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: stm32_periodic_callback
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
static int stm32_periodic_callback(void)
{
  struct stm32_lowerhalf_s *lower;
  struct lower_setperiodic_s *cbinfo;
  rtc_wakeup_callback_t cb;
  void *priv;

  lower = (struct stm32_lowerhalf_s *)&g_rtc_lowerhalf;

  cbinfo = &lower->periodic;
  cb     = (rtc_wakeup_callback_t)cbinfo->cb;
  priv   = (void *)cbinfo->priv;

  /* Perform the callback */

  if (cb != NULL)
    {
      cb(priv, 0);
    }

  return OK;
}
#endif /* CONFIG_RTC_PERIODIC */

/****************************************************************************
 * Name: stm32_setperiodic
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
static int stm32_setperiodic(struct rtc_lowerhalf_s *lower,
                             const struct lower_setperiodic_s *alarminfo)
{
  struct stm32_lowerhalf_s *priv;
  int ret;

  DEBUGASSERT(lower != NULL && alarminfo != NULL);
  priv = (struct stm32_lowerhalf_s *)lower;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  memcpy(&priv->periodic, alarminfo, sizeof(struct lower_setperiodic_s));
  ret = stm32_rtc_setperiodic(&alarminfo->period, stm32_periodic_callback);

  nxmutex_unlock(&priv->devlock);
  return ret;
}
#endif

/****************************************************************************
 * Name: stm32_cancelperiodic
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
static int stm32_cancelperiodic(struct rtc_lowerhalf_s *lower, int id)
{
  struct stm32_lowerhalf_s *priv;
  int ret;

  DEBUGASSERT(lower != NULL);
  priv = (struct stm32_lowerhalf_s *)lower;

  DEBUGASSERT(id == 0);

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  ret = stm32_rtc_cancelperiodic();

  nxmutex_unlock(&priv->devlock);
  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_rtc_lowerhalf
 *
 * Description:
 *   Instantiate the RTC lower half driver for the STM32.  General usage:
 *
 *     #include <nuttx/timers/rtc.h>
 *     #include "stm32_rtc.h>
 *
 *     struct rtc_lowerhalf_s *lower;
 *     lower = stm32_rtc_lowerhalf();
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

struct rtc_lowerhalf_s *stm32_rtc_lowerhalf(void)
{
  return (struct rtc_lowerhalf_s *)&g_rtc_lowerhalf;
}

#endif /* CONFIG_RTC_DRIVER */
