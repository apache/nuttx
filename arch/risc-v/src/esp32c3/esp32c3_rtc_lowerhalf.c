/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_rtc_lowerhalf.c
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
#include <nuttx/spinlock.h>

#include <sys/types.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/timers/rtc.h>

#include "esp32c3_rtc.h"
#include "hardware/esp32c3_tim.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
struct esp32c3_cbinfo_s
{
  volatile rtc_alarm_callback_t cb;  /* Callback when the alarm expires */
  volatile void *priv;               /* Private argurment to accompany callback */
};
#endif

/* This is the private type for the RTC state.  It must be cast compatible
 * with struct rtc_lowerhalf_s.
 */

struct esp32c3_lowerhalf_s
{
  /* This is the contained reference to the read-only, lower-half
   * operations vtable (which may lie in FLASH or ROM)
   */

  const struct rtc_ops_s *ops;
#ifdef CONFIG_RTC_ALARM
  /* Alarm callback information */

  struct esp32c3_cbinfo_s cbinfo[RTC_ALARM_LAST];
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Prototypes for static methods in struct rtc_ops_s */

static int rtc_lh_rdtime(struct rtc_lowerhalf_s *lower,
                         struct rtc_time *rtctime);
static int rtc_lh_settime(struct rtc_lowerhalf_s *lower,
                          const struct rtc_time *rtctime);
static bool rtc_lh_havesettime(struct rtc_lowerhalf_s *lower);

#ifdef CONFIG_RTC_ALARM
static void rtc_lh_alarm_callback(void *arg, unsigned int alarmid);
static int rtc_lh_setalarm(struct rtc_lowerhalf_s *lower,
                           const struct lower_setalarm_s *alarminfo);
static int rtc_lh_setrelative(struct rtc_lowerhalf_s *lower,
                           const struct lower_setrelative_s *alarminfo);
static int rtc_lh_cancelalarm(struct rtc_lowerhalf_s *lower,
                              int alarmid);
static int rtc_lh_rdalarm(struct rtc_lowerhalf_s *lower,
                          struct lower_rdalarm_s *alarminfo);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* ESP32-C3 RTC driver operations */

static const struct rtc_ops_s g_rtc_ops =
{
  .rdtime      = rtc_lh_rdtime,
  .settime     = rtc_lh_settime,
  .havesettime = rtc_lh_havesettime,
#ifdef CONFIG_RTC_ALARM
  .setalarm    = rtc_lh_setalarm,
  .setrelative = rtc_lh_setrelative,
  .cancelalarm = rtc_lh_cancelalarm,
  .rdalarm     = rtc_lh_rdalarm,
#endif
};

/* ESP32-C3 RTC device state */

static struct esp32c3_lowerhalf_s g_rtc_lowerhalf =
{
  .ops = &g_rtc_ops,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rtc_lh_alarm_callback
 *
 * Description:
 *   This is the function that is called from the RTC driver when the alarm
 *   goes off. It just invokes the upper half drivers callback.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static void rtc_lh_alarm_callback(void *arg, unsigned int alarmid)
{
  struct esp32c3_lowerhalf_s *lower;
  struct esp32c3_cbinfo_s *cbinfo;
  rtc_alarm_callback_t cb;
  void *priv;

  DEBUGASSERT((RTC_ALARM0 <= alarmid) && (alarmid < RTC_ALARM_LAST));

  lower        = (struct esp32c3_lowerhalf_s *)arg;
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
 * Name: rtc_lh_rdtime
 *
 * Description:
 *   Returns the current RTC time.
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

static int rtc_lh_rdtime(struct rtc_lowerhalf_s *lower,
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
  rtcerr("ERROR: failed to get RTC time: %d\n", ret);
  return ret;

#else
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
#endif
}

/****************************************************************************
 * Name: rtc_lh_settime
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

static int rtc_lh_settime(struct rtc_lowerhalf_s *lower,
                          const struct rtc_time *rtctime)
{
  struct timespec ts;

  /* Convert the struct rtc_time to a time_t.  Here we assume that struct
   * rtc_time is cast compatible with struct tm.
   */

  ts.tv_sec  = mktime((struct tm *)rtctime);
  ts.tv_nsec = 0;

  /* Now set the time (with a accuracy of seconds) */

  return up_rtc_settime(&ts);
}

/****************************************************************************
 * Name: rtc_lh_havesettime
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

static bool rtc_lh_havesettime(struct rtc_lowerhalf_s *lower)
{
  if (esp32c3_rtc_get_boot_time() == 0)
    {
      return false;
    }

  return true;
}

/****************************************************************************
 * Name: rtc_lh_setalarm
 *
 * Description:
 *   Set a new alarm. This function implements the setalarm() method of the
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
static int rtc_lh_setalarm(struct rtc_lowerhalf_s *lower,
                           const struct lower_setalarm_s *alarminfo)
{
  struct esp32c3_lowerhalf_s *priv;
  struct esp32c3_cbinfo_s *cbinfo;
  struct alm_setalarm_s lowerinfo;
  int ret;

  DEBUGASSERT(lower != NULL && alarminfo != NULL);
  DEBUGASSERT((RTC_ALARM0 <= alarminfo->id) &&
              (alarminfo->id < RTC_ALARM_LAST));

  priv = (struct esp32c3_lowerhalf_s *)lower;

  /* Remember the callback information */

  cbinfo            = &priv->cbinfo[alarminfo->id];
  cbinfo->cb        = alarminfo->cb;
  cbinfo->priv      = alarminfo->priv;

  /* Set the alarm */

  lowerinfo.as_id   = alarminfo->id;
  lowerinfo.as_cb   = rtc_lh_alarm_callback;
  lowerinfo.as_arg  = priv;

  /* Convert the RTC time to a timespec (1 second accuracy) */

  lowerinfo.as_time.tv_sec  = mktime((struct tm *)&alarminfo->time);
  lowerinfo.as_time.tv_nsec = 0;

  /* And set the alarm */

  ret = up_rtc_setalarm(&lowerinfo);
  if (ret < 0)
    {
      cbinfo->cb   = NULL;
      cbinfo->priv = NULL;
    }

  return ret;
}
#endif /* CONFIG_RTC_ALARM */

/****************************************************************************
 * Name: rtc_lh_setrelative
 *
 * Description:
 *   Set a new alarm relative to the current time. This function implements
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
static int rtc_lh_setrelative(struct rtc_lowerhalf_s *lower,
                            const struct lower_setrelative_s *alarminfo)
{
  struct lower_setalarm_s setalarm;
  time_t seconds;
  int ret = -EINVAL;
  irqstate_t flags;

  DEBUGASSERT(lower != NULL && alarminfo != NULL);
  DEBUGASSERT((RTC_ALARM0 <= alarminfo->id) &&
              (alarminfo->id < RTC_ALARM_LAST));

  if (alarminfo->reltime > 0)
    {
      flags = spin_lock_irqsave(NULL);

      seconds = alarminfo->reltime;
      gmtime_r(&seconds, (struct tm *)&setalarm.time);

      /* The set the alarm using this absolute time */

      setalarm.id   = alarminfo->id;
      setalarm.cb   = alarminfo->cb;
      setalarm.priv = alarminfo->priv;
      ret = rtc_lh_setalarm(lower, &setalarm);

      spin_unlock_irqrestore(NULL, flags);
    }

  return ret;
}
#endif /* CONFIG_RTC_ALARM */

/****************************************************************************
 * Name: rtc_lh_cancelalarm
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
static int rtc_lh_cancelalarm(struct rtc_lowerhalf_s *lower, int alarmid)
{
  struct esp32c3_lowerhalf_s *priv;
  struct esp32c3_cbinfo_s *cbinfo;

  DEBUGASSERT(lower != NULL);
  DEBUGASSERT((RTC_ALARM0 <= alarmid) && (alarmid < RTC_ALARM_LAST));

  priv = (struct esp32c3_lowerhalf_s *)lower;

  /* Nullify callback information to reduce window for race conditions */

  cbinfo       = &priv->cbinfo[alarmid];
  cbinfo->cb   = NULL;
  cbinfo->priv = NULL;

  /* Then cancel the alarm */

  return up_rtc_cancelalarm((enum alm_id_e)alarmid);
}
#endif /* CONFIG_RTC_ALARM */

/****************************************************************************
 * Name: rtc_lh_rdalarm
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
static int rtc_lh_rdalarm(struct rtc_lowerhalf_s *lower,
                          struct lower_rdalarm_s *alarminfo)
{
  struct timespec ts;
  int ret;
  irqstate_t flags;

  DEBUGASSERT(lower != NULL && alarminfo != NULL && alarminfo->time != NULL);
  DEBUGASSERT((RTC_ALARM0 <= alarminfo->id) &&
              (alarminfo->id < RTC_ALARM_LAST));

  flags = spin_lock_irqsave(NULL);

  ret = up_rtc_rdalarm(&ts, alarminfo->id);
  localtime_r((const time_t *)&ts.tv_sec,
              (struct tm *)alarminfo->time);

  spin_unlock_irqrestore(NULL, flags);

  return ret;
}
#endif /* CONFIG_RTC_ALARM */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_rtc_lowerhalf
 *
 * Description:
 *   Instantiate the RTC lower half driver for the ESP32-C3.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, a non-NULL RTC lower interface is returned.  NULL is
 *   returned on any failure.
 *
 ****************************************************************************/

struct rtc_lowerhalf_s *esp32c3_rtc_lowerhalf(void)
{
  return (struct rtc_lowerhalf_s *)&g_rtc_lowerhalf;
}

/****************************************************************************
 * Name: esp32c3_rtc_driverinit
 *
 * Description:
 *   Bind the configuration timer to a timer lower half instance and
 *   register the timer drivers at 'devpath'
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int esp32c3_rtc_driverinit(void)
{
  int ret;
  struct rtc_lowerhalf_s *lower;

  /* Instantiate the ESP32-C3 lower-half RTC driver */

  lower = esp32c3_rtc_lowerhalf();
  if (lower == NULL)
    {
      return ret;
    }
  else
    {
      /* Bind the lower half driver and register the combined RTC driver
       * as /dev/rtc0
       */

      ret = rtc_initialize(0, lower);
    }

  /* Init RTC timer */

  up_rtc_timer_init();

  return ret;
}
