/****************************************************************************
 * arch/arm/src/lpc54xx/lpc54_rtc_lowerhalf.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#include "hardware/lpc54_rtc.h"
#include "lpc54_rtc.h"

#ifdef CONFIG_RTC_DRIVER

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
struct lpc54_cbinfo_s
{
  volatile rtc_alarm_callback_t cb;  /* Callback when the alarm expires */
  volatile FAR void *priv;           /* Private argument to accompany callback */
};
#endif

/* This is the private type for the RTC state.  It must be cast compatible
 * with struct rtc_lowerhalf_s.
 */

struct lpc54_lowerhalf_s
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

  struct lpc54_cbinfo_s cbinfo;
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

static int lpc54_rdtime(FAR struct rtc_lowerhalf_s *lower,
             FAR struct rtc_time *rtctime);
static int lpc54_settime(FAR struct rtc_lowerhalf_s *lower,
             FAR const struct rtc_time *rtctime);
static bool lpc54_havesettime(FAR struct rtc_lowerhalf_s *lower);

#ifdef CONFIG_RTC_ALARM
static int lpc54_setalarm(FAR struct rtc_lowerhalf_s *lower,
             FAR const struct lower_setalarm_s *alarminfo);
static int lpc54_setrelative(FAR struct rtc_lowerhalf_s *lower,
             FAR const struct lower_setrelative_s *alarminfo);
static int lpc54_cancelalarm(FAR struct rtc_lowerhalf_s *lower,
             int alarmid);
static int lpc54_rdalarm(FAR struct rtc_lowerhalf_s *lower,
             FAR struct lower_rdalarm_s *alarminfo);
#endif

#ifdef CONFIG_RTC_PERIODIC
static int lpc54_setperiodic(FAR struct rtc_lowerhalf_s *lower,
             FAR const struct lower_setperiodic_s *alarminfo);
static int lpc54_cancelperiodic(FAR struct rtc_lowerhalf_s *lower, int id);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* LPC54 RTC driver operations */

static const struct rtc_ops_s g_rtc_ops =
{
  .rdtime         = lpc54_rdtime,
  .settime        = lpc54_settime,
  .havesettime    = lpc54_havesettime,
#ifdef CONFIG_RTC_ALARM
  .setalarm       = lpc54_setalarm,
  .setrelative    = lpc54_setrelative,
  .cancelalarm    = lpc54_cancelalarm,
  .rdalarm        = lpc54_rdalarm,
#endif
#ifdef CONFIG_RTC_PERIODIC
  .setperiodic    = lpc54_setperiodic,
  .cancelperiodic = lpc54_cancelperiodic,
#endif
#ifdef CONFIG_RTC_IOCTL
  .ioctl          = NULL,
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  .destroy        = NULL,
#endif
};

/* LPC54 RTC device state */

static struct lpc54_lowerhalf_s g_rtc_lowerhalf =
{
  .ops           = &g_rtc_ops,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_alarm_callback
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
static void lpc54_alarm_callback(void)
{
  FAR struct lpc54_cbinfo_s *cbinfo = &g_rtc_lowerhalf.cbinfo;

  /* Sample and clear the callback information to minimize the window in
   * time in which race conditions can occur.
   */

  rtc_alarm_callback_t cb = (rtc_alarm_callback_t)cbinfo->cb;
  FAR void *arg           = (FAR void *)cbinfo->priv;

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
 * Name: lpc54_rdtime
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

static int lpc54_rdtime(FAR struct rtc_lowerhalf_s *lower,
                        FAR struct rtc_time *rtctime)
{
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
}

/****************************************************************************
 * Name: lpc54_settime
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

static int lpc54_settime(FAR struct rtc_lowerhalf_s *lower,
                         FAR const struct rtc_time *rtctime)
{
  struct timespec ts;

  /* Convert the struct rtc_time to a time_t.  Here we assume that struct
   * rtc_time is cast compatible with struct tm.
   */

  ts.tv_sec  = mktime((FAR struct tm *)rtctime);
  ts.tv_nsec = 0;

  /* Now set the time (to one second accuracy) */

  return up_rtc_settime(&ts);
}

/****************************************************************************
 * Name: lpc54_havesettime
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

static bool lpc54_havesettime(FAR struct rtc_lowerhalf_s *lower)
{
  return getreg32(RTC_MAGIC_REG) == RTC_MAGIC;
}

/****************************************************************************
 * Name: lpc54_setalarm
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
static int lpc54_setalarm(FAR struct rtc_lowerhalf_s *lower,
                          FAR const struct lower_setalarm_s *alarminfo)
{
  FAR struct lpc54_lowerhalf_s *priv;
  FAR struct lpc54_cbinfo_s *cbinfo;
  int ret;

  DEBUGASSERT(lower != NULL && alarminfo != NULL && alarminfo->id == 0);
  priv = (FAR struct lpc54_lowerhalf_s *)lower;

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

      cbinfo           = &priv->cbinfo;
      cbinfo->cb       = alarminfo->cb;
      cbinfo->priv     = alarminfo->priv;

      /* And set the alarm */

      ret = lpc54_rtc_setalarm(&ts, lpc54_alarm_callback);
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
 * Name: lpc54_setrelative
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
static int lpc54_setrelative(FAR struct rtc_lowerhalf_s *lower,
                             FAR const struct lower_setrelative_s *alarminfo)
{
  FAR struct lpc54_lowerhalf_s *priv;
  FAR struct lpc54_cbinfo_s *cbinfo;
  FAR struct timespec ts;
  int ret = -EINVAL;

  DEBUGASSERT(lower != NULL && alarminfo != NULL && alarminfo->id == 0);
  priv = (FAR struct lpc54_lowerhalf_s *)lower;

  if (alarminfo->id == 0 && alarminfo->reltime > 0)
    {
      /* Disable pre-emption while we do this so that we don't have to worry
       * about being suspended and working on an old time.
       */

      sched_lock();

      /* Get the current time in seconds */

      ts.tv_sec  = up_rtc_time();
      ts.tv_nsec = 0;

      /* Add the seconds offset.  Add one to the number of seconds because
       * we are unsure of the phase of the timer.
       */

      ts.tv_sec   += (alarminfo->reltime + 1);

      /* Remember the callback information */

      cbinfo       = &priv->cbinfo;
      cbinfo->cb   = alarminfo->cb;
      cbinfo->priv = alarminfo->priv;

      /* And set the alarm */

      ret = lpc54_rtc_setalarm(&ts, lpc54_alarm_callback);
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
 * Name: lpc54_cancelalarm
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
static int lpc54_cancelalarm(FAR struct rtc_lowerhalf_s *lower, int alarmid)
{
  FAR struct lpc54_lowerhalf_s *priv;
  FAR struct lpc54_cbinfo_s *cbinfo;

  DEBUGASSERT(lower != NULL);
  DEBUGASSERT(alarmid == 0);
  priv = (FAR struct lpc54_lowerhalf_s *)lower;

  /* Nullify callback information to reduce window for race conditions */

  cbinfo       = &priv->cbinfo;
  cbinfo->cb   = NULL;
  cbinfo->priv = NULL;

  /* Then cancel the alarm */

  return lpc54_rtc_cancelalarm();
}
#endif

/****************************************************************************
 * Name: lpc54_rdalarm
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
static int lpc54_rdalarm(FAR struct rtc_lowerhalf_s *lower,
                         FAR struct lower_rdalarm_s *alarminfo)
{
  int ret = -EINVAL;

  DEBUGASSERT(lower != NULL && alarminfo != NULL && alarminfo->id == 0 &&
              alarminfo->time != NULL);

  if (alarminfo->id == 0)
    {
      ret = lpc54_rtc_rdalarm((FAR struct tm *)alarminfo->time);
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: lpc54_periodic_callback
 *
 * Description:
 *   This is the function that is called from the RTC driver when the periodic
 *   wakeup goes off.  It just invokes the upper half drivers callback.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_PERIODIC
static int lpc54_periodic_callback(void)
{
  FAR struct lpc54_lowerhalf_s *lower;
  struct lower_setperiodic_s *cbinfo;
  rtc_wakeup_callback_t cb;
  FAR void *priv;

  lower = (FAR struct lpc54_lowerhalf_s *)&g_rtc_lowerhalf;

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
 * Name: lpc54_setperiodic
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
static int lpc54_setperiodic(FAR struct rtc_lowerhalf_s *lower,
                             FAR const struct lower_setperiodic_s *alarminfo)
{
  FAR struct lpc54_lowerhalf_s *priv;
  int ret;

  DEBUGASSERT(lower != NULL && alarminfo != NULL);
  priv = (FAR struct lpc54_lowerhalf_s *)lower;

  ret = nxsem_wait(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  memcpy(&priv->periodic, alarminfo, sizeof(struct lower_setperiodic_s));

  ret = lpc54_rtc_setperiodic(&alarminfo->period, lpc54_periodic_callback);

  nxsem_post(&priv->devsem);
  return ret;
}
#endif

/****************************************************************************
 * Name: lpc54_cancelperiodic
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
static int lpc54_cancelperiodic(FAR struct rtc_lowerhalf_s *lower, int id)
{
  FAR struct lpc54_lowerhalf_s *priv;
  int ret;

  DEBUGASSERT(lower != NULL);
  priv = (FAR struct lpc54_lowerhalf_s *)lower;

  DEBUGASSERT(id == 0);

  ret = nxsem_wait(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  ret = lpc54_rtc_cancelperiodic();

  nxsem_post(&priv->devsem);
  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_rtc_lowerhalf
 *
 * Description:
 *   Instantiate the RTC lower half driver for the LPC54.  General usage:
 *
 *     #include <nuttx/timers/rtc.h>
 *     #include "lpc54_rtc.h>
 *
 *     struct rtc_lowerhalf_s *lower;
 *     lower = lpc54_rtc_lowerhalf();
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

FAR struct rtc_lowerhalf_s *lpc54_rtc_lowerhalf(void)
{
  nxsem_init(&g_rtc_lowerhalf.devsem, 0, 1);

  return (FAR struct rtc_lowerhalf_s *)&g_rtc_lowerhalf;
}

#endif /* CONFIG_RTC_DRIVER */
