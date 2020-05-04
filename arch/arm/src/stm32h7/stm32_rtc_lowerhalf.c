/****************************************************************************
 * arch/arm/src/stm32h7/stm32_rtc_lowerhalf.c
 *
 *   Copyright (C) 2015-2016, 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
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
#include "stm32_rtc.h"

#ifdef CONFIG_RTC_DRIVER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STM32_NALARMS 2

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
struct stm32_cbinfo_s
{
  volatile rtc_alarm_callback_t cb;  /* Callback when the alarm expires */
  volatile FAR void *priv;           /* Private argument to accompany callback */
  uint8_t id;                        /* Identifies the alarm */
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

  FAR const struct rtc_ops_s *ops;

  /* Data following is private to this driver and not visible outside of
   * this file.
   */

  sem_t devsem;         /* Threads can only exclusively access the RTC */

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

static int stm32_rdtime(FAR struct rtc_lowerhalf_s *lower,
                        FAR struct rtc_time *rtctime);
static int stm32_settime(FAR struct rtc_lowerhalf_s *lower,
                         FAR const struct rtc_time *rtctime);
static bool stm32_havesettime(FAR struct rtc_lowerhalf_s *lower);

#ifdef CONFIG_RTC_ALARM
static int stm32_setalarm(FAR struct rtc_lowerhalf_s *lower,
                          FAR const struct lower_setalarm_s *alarminfo);
static int stm32_setrelative(FAR struct rtc_lowerhalf_s *lower,
                             FAR const struct lower_setrelative_s *alarminfo);
static int stm32_cancelalarm(FAR struct rtc_lowerhalf_s *lower,
                             int alarmid);
static int stm32_rdalarm(FAR struct rtc_lowerhalf_s *lower,
                         FAR struct lower_rdalarm_s *alarminfo);
#endif

#ifdef CONFIG_RTC_PERIODIC
static int stm32_setperiodic(FAR struct rtc_lowerhalf_s *lower,
                             FAR const struct lower_setperiodic_s *alarminfo);
static int stm32_cancelperiodic(FAR struct rtc_lowerhalf_s *lower, int id);
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
  .ops         = &g_rtc_ops,
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
static void stm32_alarm_callback(FAR void *arg, unsigned int alarmid)
{
  FAR struct stm32_lowerhalf_s *lower;
  FAR struct stm32_cbinfo_s *cbinfo;
  rtc_alarm_callback_t cb;
  FAR void *priv;

  DEBUGASSERT(arg != NULL);
  DEBUGASSERT(alarmid == RTC_ALARMA || alarmid == RTC_ALARMB);

  lower        = (struct stm32_lowerhalf_s *)arg;
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

static int stm32_rdtime(FAR struct rtc_lowerhalf_s *lower,
                        FAR struct rtc_time *rtctime)
{
  FAR struct stm32_lowerhalf_s *priv;
  int ret;

  priv = (FAR struct stm32_lowerhalf_s *)lower;

  ret = nxsem_wait(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

#if defined(CONFIG_RTC_DATETIME)
  /* This operation depends on the fact that struct rtc_time is cast
   * compatible with struct tm.
   */

  ret = up_rtc_getdatetime((FAR struct tm *)rtctime);
#else
  time_t timer;

  /* The resolution of time is only 1 second */

  timer = up_rtc_time();

  /* Convert the one second epoch time to a struct tm */

  if (!gmtime_r(&timer, (FAR struct tm *)rtctime))
    {
      int errcode = get_errno();
      DEBUGASSERT(errcode > 0);
      nxsem_post(&priv->devsem);
      return -errcode;
    }

#endif
  nxsem_post(&priv->devsem);
  return ret;
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

static int stm32_settime(FAR struct rtc_lowerhalf_s *lower,
                         FAR const struct rtc_time *rtctime)
{
  FAR struct stm32_lowerhalf_s *priv;
  int ret;

  priv = (FAR struct stm32_lowerhalf_s *)lower;

  ret = nxsem_wait(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_RTC_DATETIME
  /* This operation depends on the fact that struct rtc_time is cast
   * compatible with struct tm.
   */

  ret = stm32_rtc_setdatetime((FAR const struct tm *)rtctime);

#else
  struct timespec ts;

  /* Convert the struct rtc_time to a time_t.  Here we assume that struct
   * rtc_time is cast compatible with struct tm.
   */

  ts.tv_sec  = mktime((FAR struct tm *)rtctime);
  ts.tv_nsec = 0;

  /* Now set the time (to one second accuracy) */

  ret = up_rtc_settime(&ts);
#endif

  nxsem_post(&priv->devsem);
  return ret;
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

static bool stm32_havesettime(FAR struct rtc_lowerhalf_s *lower)
{
  return stm32_rtc_havesettime();
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
static int stm32_setalarm(FAR struct rtc_lowerhalf_s *lower,
                          FAR const struct lower_setalarm_s *alarminfo)
{
  FAR struct stm32_lowerhalf_s *priv;
  FAR struct stm32_cbinfo_s *cbinfo;
  struct alm_setalarm_s lowerinfo;
  int ret;

  /* ID0-> Alarm A; ID1 -> Alarm B */

  DEBUGASSERT(lower != NULL && alarminfo != NULL);
  DEBUGASSERT(alarminfo->id == RTC_ALARMA || alarminfo->id == RTC_ALARMB);
  priv = (FAR struct stm32_lowerhalf_s *)lower;

  ret = nxsem_wait(&priv->devsem);
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

  nxsem_post(&priv->devsem);

  return ret;
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
static int stm32_setrelative(FAR struct rtc_lowerhalf_s *lower,
                             FAR const struct lower_setrelative_s *alarminfo)
{
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

          seconds = mktime(&time);

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

          ret = stm32_setalarm(lower, &setalarm);
        }

      sched_unlock();
    }

  return ret;
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
 *   lower     - A reference to RTC lower half driver state structure
 *   alarminfo - Provided information needed to set the alarm
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int stm32_cancelalarm(FAR struct rtc_lowerhalf_s *lower, int alarmid)
{
  FAR struct stm32_lowerhalf_s *priv;
  FAR struct stm32_cbinfo_s *cbinfo;
  int ret;

  DEBUGASSERT(lower != NULL);
  DEBUGASSERT(alarmid == RTC_ALARMA || alarmid == RTC_ALARMB);
  priv = (FAR struct stm32_lowerhalf_s *)lower;

  ret = nxsem_wait(&priv->devsem);
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

  nxsem_post(&priv->devsem);

  return ret;
}
#endif

/****************************************************************************
 * Name: stm32_rdalarm
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
static int stm32_rdalarm(FAR struct rtc_lowerhalf_s *lower,
                         FAR struct lower_rdalarm_s *alarminfo)
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

      lowerinfo.ar_id = alarminfo->id;
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
 *   periodic wakeup goes off.  It just invokes the upper half driver's
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
  FAR struct stm32_lowerhalf_s *lower;
  struct lower_setperiodic_s *cbinfo;
  rtc_wakeup_callback_t cb;
  FAR void *priv;

  lower = (FAR struct stm32_lowerhalf_s *)&g_rtc_lowerhalf;

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
static int stm32_setperiodic(FAR struct rtc_lowerhalf_s *lower,
                             FAR const struct lower_setperiodic_s *alarminfo)
{
  FAR struct stm32_lowerhalf_s *priv;
  int ret;

  DEBUGASSERT(lower != NULL && alarminfo != NULL);
  priv = (FAR struct stm32_lowerhalf_s *)lower;

  ret = nxsem_wait(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  memcpy(&priv->periodic, alarminfo, sizeof(struct lower_setperiodic_s));

  ret = stm32_rtc_setperiodic(&alarminfo->period, stm32_periodic_callback);

  nxsem_post(&priv->devsem);

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
static int stm32_cancelperiodic(FAR struct rtc_lowerhalf_s *lower, int id)
{
  FAR struct stm32_lowerhalf_s *priv;
  int ret;

  DEBUGASSERT(lower != NULL);
  priv = (FAR struct stm32_lowerhalf_s *)lower;

  DEBUGASSERT(id == 0);

  ret = nxsem_wait(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  ret = stm32_rtc_cancelperiodic();

  nxsem_post(&priv->devsem);

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

FAR struct rtc_lowerhalf_s *stm32_rtc_lowerhalf(void)
{
  nxsem_init(&g_rtc_lowerhalf.devsem, 0, 1);

  return (FAR struct rtc_lowerhalf_s *)&g_rtc_lowerhalf;
}

#endif /* CONFIG_RTC_DRIVER */
