/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_rtc_lowerhalf.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/timers/rtc.h>

#include "rp23xx_rtc.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM

/* Bridges the chip-level alarm callback, which carries only an opaque
 * argument, to the RTC upper-half callback, which also wants an alarm id.
 */

struct rp23xx_cbinfo_s
{
  volatile rtc_alarm_callback_t cb;  /* Upper-half callback */
  volatile FAR void *priv;           /* Upper-half argument */
};
#endif

struct rp23xx_lowerhalf_s
{
  /* This is the contained reference to the read-only, lower-half operations
   * vtable (which may lie in FLASH).
   */

  FAR const struct rtc_ops_s *ops;

  /* Data following is private to this driver and not visible outside of
   * this file.
   */

  mutex_t devlock;                   /* Threads accessing the device */

#ifdef CONFIG_RTC_ALARM
  struct rp23xx_cbinfo_s cbinfo;     /* Alarm callback information */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int rp23xx_rdtime(FAR struct rtc_lowerhalf_s *lower,
                         FAR struct rtc_time *rtctime);
static int rp23xx_settime(FAR struct rtc_lowerhalf_s *lower,
                          FAR const struct rtc_time *rtctime);
static bool rp23xx_havesettime(FAR struct rtc_lowerhalf_s *lower);

#ifdef CONFIG_RTC_ALARM
static int rp23xx_setalarm(FAR struct rtc_lowerhalf_s *lower,
                           FAR const struct lower_setalarm_s *alarminfo);
static int rp23xx_setrelative(FAR struct rtc_lowerhalf_s *lower,
                              FAR const struct lower_setrelative_s *info);
static int rp23xx_cancelalarm(FAR struct rtc_lowerhalf_s *lower,
                              int alarmid);
static int rp23xx_rdalarm(FAR struct rtc_lowerhalf_s *lower,
                          FAR struct lower_rdalarm_s *alarminfo);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* RP2350 RTC driver operations */

static const struct rtc_ops_s g_rtc_ops =
{
  .rdtime      = rp23xx_rdtime,
  .settime     = rp23xx_settime,
  .havesettime = rp23xx_havesettime,
#ifdef CONFIG_RTC_ALARM
  .setalarm    = rp23xx_setalarm,
  .setrelative = rp23xx_setrelative,
  .cancelalarm = rp23xx_cancelalarm,
  .rdalarm     = rp23xx_rdalarm,
#endif
};

/* RP2350 RTC device state */

static struct rp23xx_lowerhalf_s g_rtc_lowerhalf =
{
  .ops     = &g_rtc_ops,
  .devlock = NXMUTEX_INITIALIZER,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM

/****************************************************************************
 * Name: rp23xx_alarm_callback
 *
 * Description:
 *   Invoked from the POWMAN timer interrupt when the alarm expires.
 *
 ****************************************************************************/

static void rp23xx_alarm_callback(FAR void *arg)
{
  FAR struct rp23xx_lowerhalf_s *priv = arg;
  FAR struct rp23xx_cbinfo_s *cbinfo = &priv->cbinfo;
  rtc_alarm_callback_t cb;
  FAR void *cbarg;

  /* Sample and clear the callback before invoking it, so that the callback
   * is free to arm a new alarm.
   */

  cb    = (rtc_alarm_callback_t)cbinfo->cb;
  cbarg = (FAR void *)cbinfo->priv;

  cbinfo->cb   = NULL;
  cbinfo->priv = NULL;

  if (cb != NULL)
    {
      cb(cbarg, RP23XX_ALARM0);
    }
}
#endif /* CONFIG_RTC_ALARM */

/****************************************************************************
 * Name: rp23xx_rdtime
 *
 * Description:
 *   Implements the rdtime() method of the RTC driver interface.
 *
 ****************************************************************************/

static int rp23xx_rdtime(FAR struct rtc_lowerhalf_s *lower,
                         FAR struct rtc_time *rtctime)
{
  time_t secs;

  secs = (time_t)(rp23xx_rtc_getms() / MSEC_PER_SEC);

  if (gmtime_r(&secs, (FAR struct tm *)rtctime) == NULL)
    {
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: rp23xx_settime
 *
 * Description:
 *   Implements the settime() method of the RTC driver interface.
 *
 ****************************************************************************/

static int rp23xx_settime(FAR struct rtc_lowerhalf_s *lower,
                          FAR const struct rtc_time *rtctime)
{
  struct timespec ts;

  /* timegm() normalises its argument, so convert a copy */

  struct tm tm = *(FAR const struct tm *)rtctime;

  ts.tv_sec  = timegm(&tm);
  ts.tv_nsec = 0;

  return up_rtc_settime(&ts);
}

/****************************************************************************
 * Name: rp23xx_havesettime
 *
 * Description:
 *   Implements the havesettime() method of the RTC driver interface.
 *
 ****************************************************************************/

static bool rp23xx_havesettime(FAR struct rtc_lowerhalf_s *lower)
{
  /* POWMAN has no flag that the time was set; report the counter running */

  return g_rtc_enabled;
}

#ifdef CONFIG_RTC_ALARM

/****************************************************************************
 * Name: rp23xx_setalarm
 *
 * Description:
 *   Implements the setalarm() method of the RTC driver interface.
 *
 ****************************************************************************/

static int rp23xx_setalarm(FAR struct rtc_lowerhalf_s *lower,
                           FAR const struct lower_setalarm_s *alarminfo)
{
  FAR struct rp23xx_lowerhalf_s *priv;
  FAR struct rp23xx_cbinfo_s *cbinfo;
  struct tm tm;
  uint64_t ms;
  int ret;

  DEBUGASSERT(lower != NULL && alarminfo != NULL);
  priv = (FAR struct rp23xx_lowerhalf_s *)lower;

  if (alarminfo->id != RP23XX_ALARM0)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  tm = *(FAR const struct tm *)&alarminfo->time;
  ms = (uint64_t)timegm(&tm) * MSEC_PER_SEC;

  /* Remember the callback information before arming: the alarm can in
   * principle fire the moment it is enabled.
   */

  cbinfo       = &priv->cbinfo;
  cbinfo->cb   = alarminfo->cb;
  cbinfo->priv = alarminfo->priv;

  ret = rp23xx_rtc_setalarm(ms, rp23xx_alarm_callback, priv);
  if (ret < 0)
    {
      cbinfo->cb   = NULL;
      cbinfo->priv = NULL;
    }

  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Name: rp23xx_setrelative
 *
 * Description:
 *   Implements the setrelative() method of the RTC driver interface.
 *
 ****************************************************************************/

static int rp23xx_setrelative(FAR struct rtc_lowerhalf_s *lower,
                              FAR const struct lower_setrelative_s *info)
{
  FAR struct rp23xx_lowerhalf_s *priv;
  FAR struct rp23xx_cbinfo_s *cbinfo;
  uint64_t ms;
  int ret;

  DEBUGASSERT(lower != NULL && info != NULL);
  priv = (FAR struct rp23xx_lowerhalf_s *)lower;

  if (info->id != RP23XX_ALARM0 || info->reltime <= 0)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  /* Use the millisecond counter, not the system time in whole seconds */

  ms = rp23xx_rtc_getms() + (uint64_t)info->reltime * MSEC_PER_SEC;

  cbinfo       = &priv->cbinfo;
  cbinfo->cb   = info->cb;
  cbinfo->priv = info->priv;

  ret = rp23xx_rtc_setalarm(ms, rp23xx_alarm_callback, priv);
  if (ret < 0)
    {
      cbinfo->cb   = NULL;
      cbinfo->priv = NULL;
    }

  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Name: rp23xx_cancelalarm
 *
 * Description:
 *   Implements the cancelalarm() method of the RTC driver interface.
 *
 ****************************************************************************/

static int rp23xx_cancelalarm(FAR struct rtc_lowerhalf_s *lower,
                              int alarmid)
{
  FAR struct rp23xx_lowerhalf_s *priv;
  FAR struct rp23xx_cbinfo_s *cbinfo;
  int ret;

  DEBUGASSERT(lower != NULL);
  priv = (FAR struct rp23xx_lowerhalf_s *)lower;

  if (alarmid != RP23XX_ALARM0)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  cbinfo       = &priv->cbinfo;
  cbinfo->cb   = NULL;
  cbinfo->priv = NULL;

  ret = rp23xx_rtc_cancelalarm();

  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Name: rp23xx_rdalarm
 *
 * Description:
 *   Implements the rdalarm() method of the RTC driver interface.
 *
 ****************************************************************************/

static int rp23xx_rdalarm(FAR struct rtc_lowerhalf_s *lower,
                          FAR struct lower_rdalarm_s *alarminfo)
{
  uint64_t ms;
  time_t secs;
  int ret;

  DEBUGASSERT(lower != NULL && alarminfo != NULL &&
              alarminfo->time != NULL);

  if (alarminfo->id != RP23XX_ALARM0)
    {
      return -EINVAL;
    }

  /* An alarm that is not armed is not an error: the upper half reports it
   * as inactive.
   */

  ret = rp23xx_rtc_rdalarm(&ms);
  if (ret == -ENODATA)
    {
      ms = 0;
    }
  else if (ret < 0)
    {
      return ret;
    }

  secs = (time_t)(ms / MSEC_PER_SEC);

  if (gmtime_r(&secs, (FAR struct tm *)alarminfo->time) == NULL)
    {
      return -EINVAL;
    }

  return OK;
}
#endif /* CONFIG_RTC_ALARM */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_rtc_lowerhalf
 *
 * Description:
 *   Instantiate the RTC lower half driver for the POWMAN always-on timer.
 *
 ****************************************************************************/

FAR struct rtc_lowerhalf_s *rp23xx_rtc_lowerhalf(void)
{
  return (FAR struct rtc_lowerhalf_s *)&g_rtc_lowerhalf;
}
