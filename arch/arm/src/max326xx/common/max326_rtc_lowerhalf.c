/****************************************************************************
 * arch/arm/src/max326xx/max326_rtc_lowerhalf.c
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

#include <sys/types.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/timers/rtc.h>

#include "arm_arch.h"

#include "chip.h"
#include "max326_rtc.h"

#ifdef CONFIG_RTC_DRIVER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RTC_NALARMS 1

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
struct max326_cbinfo_s
{
  volatile rtc_alarm_callback_t cb;  /* Callback when the alarm expires */
  volatile FAR void *priv;           /* Private argument to accompany callback */
};
#endif

/* This is the private type for the RTC state.  It must be cast compatible
 * with struct rtc_lowerhalf_s.
 */

struct max326_lowerhalf_s
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

  struct max326_cbinfo_s cbinfo[RTC_NALARMS];
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

static int max326_rdtime(FAR struct rtc_lowerhalf_s *lower,
                        FAR struct rtc_time *rtctime);
static int max326_settime(FAR struct rtc_lowerhalf_s *lower,
                         FAR const struct rtc_time *rtctime);
static bool max326_havesettime(FAR struct rtc_lowerhalf_s *lower);

#ifdef CONFIG_RTC_ALARM
static int max326_setalarm(FAR struct rtc_lowerhalf_s *lower,
                           FAR const struct lower_setalarm_s *alarminfo);
static int max326_setrelative(FAR struct rtc_lowerhalf_s *lower,
                           FAR const struct lower_setrelative_s *alarminfo);
static int max326_cancelalarm(FAR struct rtc_lowerhalf_s *lower,
                             int alarmid);
static int max326_rdalarm(FAR struct rtc_lowerhalf_s *lower,
                          FAR struct lower_rdalarm_s *alarminfo);
#endif

#ifdef CONFIG_RTC_PERIODIC
static int max326_setperiodic(FAR struct rtc_lowerhalf_s *lower,
                           FAR const struct lower_setperiodic_s *alarminfo);
static int max326_cancelperiodic(FAR struct rtc_lowerhalf_s *lower, int id);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* RTC driver operations */

static const struct rtc_ops_s g_rtc_ops =
{
  .rdtime      = max326_rdtime,
  .settime     = max326_settime,
  .havesettime = max326_havesettime,
#ifdef CONFIG_RTC_ALARM
  .setalarm    = max326_setalarm,
  .setrelative = max326_setrelative,
  .cancelalarm = max326_cancelalarm,
  .rdalarm     = max326_rdalarm,
#endif
#ifdef CONFIG_RTC_PERIODIC
  .setperiodic    = max326_setperiodic,
  .cancelperiodic = max326_cancelperiodic,
#endif
#ifdef CONFIG_RTC_IOCTL
  .ioctl       = NULL,
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  .destroy     = NULL,
#endif
};

/* RTC device state */

static struct max326_lowerhalf_s g_rtc_lowerhalf =
{
  .ops        = &g_rtc_ops,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max326_alarm_callback
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
static void max326_alarm_callback(FAR void *arg, unsigned int alarmid)
{
  FAR struct max326_lowerhalf_s *lower;
  FAR struct max326_cbinfo_s *cbinfo;
  rtc_alarm_callback_t cb;
  FAR void *priv;

  DEBUGASSERT(alarmid < RTC_NALARMS);

  lower        = (struct max326_lowerhalf_s *)arg;
  cbinfo       = &lower->cbinfo[alarmid];

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
      cb(priv, alarmid);
    }
}
#endif /* CONFIG_RTC_ALARM */

/****************************************************************************
 * Name: max326_rdtime
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

static int max326_rdtime(FAR struct rtc_lowerhalf_s *lower,
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
      goto errout_with_errno;
    }

  /* Convert the one second epoch time to a struct tm.  This operation
   * depends on the fact that struct rtc_time and struct tm are cast
   * compatible.
   */

  if (!gmtime_r(&ts.tv_sec, (FAR struct tm *)rtctime))
    {
      goto errout_with_errno;
    }

  return OK;

errout_with_errno:
  ret = get_errno();
  DEBUGASSERT(ret > 0);
  return -ret;

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
 * Name: max326_settime
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

static int max326_settime(FAR struct rtc_lowerhalf_s *lower,
                         FAR const struct rtc_time *rtctime)
{
#ifdef CONFIG_RTC_DATETIME
  /* This operation depends on the fact that struct rtc_time is cast
   * compatible with struct tm.
   */

  return max326_rtc_setdatetime((FAR const struct tm *)rtctime);

#else
  struct timespec ts;

  /* Convert the struct rtc_time to a time_t.  Here we assume that struct
   * rtc_time is cast compatible with struct tm.
   */

  ts.tv_sec  = mktime((FAR struct tm *)rtctime);
  ts.tv_nsec = 0;

  /* Now set the time (to one second accuracy) */

  return up_rtc_settime(&ts);
#endif
}

/****************************************************************************
 * Name: max326_havesettime
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

static bool max326_havesettime(FAR struct rtc_lowerhalf_s *lower)
{
  return true;
}

/****************************************************************************
 * Name: max326_setalarm
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
static int max326_setalarm(FAR struct rtc_lowerhalf_s *lower,
                           FAR const struct lower_setalarm_s *alarminfo)
{
  FAR struct max326_lowerhalf_s *priv;
  FAR struct max326_cbinfo_s *cbinfo;
  int ret;

  DEBUGASSERT(lower != NULL && alarminfo != NULL && alarminfo->id == 0);
  priv = (FAR struct max326_lowerhalf_s *)lower;

  ret = nxsem_wait(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  ret = -EINVAL;
  if (alarminfo->id == 0)
    {
      struct timespec ts;

      /* Convert the RTC time to a timespec (1 second accuracy) */

      ts.tv_sec   = mktime((FAR struct tm *)&alarminfo->time);
      ts.tv_nsec  = 0;

      /* Remember the callback information */

      cbinfo            = &priv->cbinfo[0];
      cbinfo->cb        = alarminfo->cb;
      cbinfo->priv      = alarminfo->priv;

      /* And set the alarm */

      ret = max326_rtc_setalarm(&ts, max326_alarm_callback, lower);
      if (ret < 0)
        {
          cbinfo->cb   = NULL;
          cbinfo->priv = NULL;
        }
    }

  nxsem_post(&priv->devsem);

  return ret;
}
#endif

/****************************************************************************
 * Name: max326_setrelative
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
static int max326_setrelative(FAR struct rtc_lowerhalf_s *lower,
                             FAR const struct lower_setrelative_s *alarminfo)
{
  FAR struct max326_lowerhalf_s *priv;
  FAR struct max326_cbinfo_s *cbinfo;
#if defined(CONFIG_RTC_DATETIME)
  struct tm time;
#endif
  FAR struct timespec ts;
  int ret = -EINVAL;

  DEBUGASSERT(lower != NULL && alarminfo != NULL && alarminfo->id == 0);
  priv = (FAR struct max326_lowerhalf_s *)lower;

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

      ts.tv_sec  = mktime(&time);
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

      ret = max326_rtc_setalarm(&ts, max326_alarm_callback, lower);
      if (ret < 0)
        {
          cbinfo->cb   = NULL;
          cbinfo->priv = NULL;
        }

      sched_unlock();
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: max326_cancelalarm
 *
 * Description:
 *   Cancel the current alarm.  This function implements the cancelalarm()
 *   method of the RTC driver interface
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
static int max326_cancelalarm(FAR struct rtc_lowerhalf_s *lower, int alarmid)
{
  FAR struct max326_lowerhalf_s *priv;
  FAR struct max326_cbinfo_s *cbinfo;

  DEBUGASSERT(lower != NULL);
  DEBUGASSERT(alarmid == 0);
  priv = (FAR struct max326_lowerhalf_s *)lower;

  /* Nullify callback information to reduce window for race conditions */

  cbinfo       = &priv->cbinfo[0];
  cbinfo->cb   = NULL;
  cbinfo->priv = NULL;

  /* Then cancel the alarm */

  return max326_rtc_cancelalarm();
}
#endif

/****************************************************************************
 * Name: max326_rdalarm
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
static int max326_rdalarm(FAR struct rtc_lowerhalf_s *lower,
                         FAR struct lower_rdalarm_s *alarminfo)
{
  int ret = -EINVAL;

  DEBUGASSERT(lower != NULL && alarminfo != NULL);
  DEBUGASSERT(alarminfo->id == 0 && alarminfo->time != NULL);

  if (alarminfo->id == 0)
    {
       b32_t ftime;

      /* Disable pre-emption while we do this so that we don't have to worry
       * about being suspended and working on an old time.
       */

      ret = max326_rtc_rdalarm(&ftime);
      if (ret >= 0)
        {
          /* Extract integer seconds from the b32_t value */

          time_t sec = (time_t)(b32toi(ftime));

          /* Convert to struct rtc_time (aka struct tm) */

          localtime_r(&sec, (FAR struct tm *)alarminfo->time);
          ret = OK;
        }
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: max326_periodic_callback
 *
 * Description:
 *   This is the function that is called from the RTC driver when the
 *   periodic wakeup goes off. It just invokes the upper half drivers
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
static int max326_periodic_callback(void)
{
  FAR struct max326_lowerhalf_s *lower;
  struct lower_setperiodic_s *cbinfo;
  rtc_wakeup_callback_t cb;
  FAR void *priv;

  lower = (FAR struct max326_lowerhalf_s *)&g_rtc_lowerhalf;

  cbinfo = &lower->periodic;
  cb     = (rtc_wakeup_callback_t)cbinfo->cb;
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
 * Name: max326_setperiodic
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
static int max326_setperiodic(FAR struct rtc_lowerhalf_s *lower,
                             FAR const struct lower_setperiodic_s *alarminfo)
{
  FAR struct max326_lowerhalf_s *priv;
  int ret;

  DEBUGASSERT(lower != NULL && alarminfo != NULL);
  priv = (FAR struct max326_lowerhalf_s *)lower;

  ret = nxsem_wait(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  memcpy(&priv->periodic, alarminfo, sizeof(struct lower_setperiodic_s));

  ret = max326_rtc_setperiodic(&alarminfo->period, max326_periodic_callback);

  nxsem_post(&priv->devsem);

  return ret;
}
#endif

/****************************************************************************
 * Name: max326_cancelperiodic
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
static int max326_cancelperiodic(FAR struct rtc_lowerhalf_s *lower, int id)
{
  FAR struct max326_lowerhalf_s *priv;
  int ret;

  DEBUGASSERT(lower != NULL);
  priv = (FAR struct max326_lowerhalf_s *)lower;

  DEBUGASSERT(id == 0);

  ret = nxsem_wait(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  ret = max326_rtc_cancelperiodic();

  nxsem_post(&priv->devsem);

  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max326_rtc_lowerhalf
 *
 * Description:
 *   Instantiate the RTC lower half driver for the MAX326XX.  General usage:
 *
 *     #include <nuttx/timers/rtc.h>
 *     #include "max326_rtc.h"
 *
 *     struct rtc_lowerhalf_s *lower;
 *     lower = max326_rtc_lowerhalf();
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

FAR struct rtc_lowerhalf_s *max326_rtc_lowerhalf(void)
{
  nxsem_init(&g_rtc_lowerhalf.devsem, 0, 1);

  return (FAR struct rtc_lowerhalf_s *)&g_rtc_lowerhalf;
}

#endif /* CONFIG_RTC_DRIVER */
