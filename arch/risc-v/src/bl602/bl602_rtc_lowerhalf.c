/****************************************************************************
 * arch/risc-v/src/bl602/bl602_rtc_lowerhalf.c
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

#include "riscv_internal.h"
#include "chip.h"
#include "hardware/bl602_hbn.h"
#include "bl602_hbn.h"
#include "bl602_rtc.h"
#include "time.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
struct bl602_cbinfo_s
{
  rtc_alarm_callback_t cb;  /* Callback when the alarm expires */
  void *priv;               /* Private argument to accompany callback */
};
#endif

/* This is the private type for the RTC state.  It must be cast compatible
 * with struct rtc_lowerhalf_s.
 */

struct bl602_lowerhalf_s
{
  /* This is the contained reference to the read-only, lower-half
   * operations vtable (which may lie in FLASH or ROM)
   */

  const struct rtc_ops_s *ops;

  /* Data following is private to this driver and not visible outside of
   * this file.
   */

  mutex_t devlock;      /* Threads can only exclusively access the RTC */

  struct rtc_time rtc_base;
  struct rtc_time rtc_alarm;

#ifdef CONFIG_RTC_ALARM
  /* Alarm callback information */

  struct bl602_cbinfo_s cbinfo;
#endif

#ifdef CONFIG_RTC_PERIODIC
  /* Periodic wakeup information */

  uint8_t periodic_enable;
  struct lower_setperiodic_s periodic;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Prototypes for static methods in struct rtc_ops_s */

static int bl602_rdtime(struct rtc_lowerhalf_s *lower,
                        struct rtc_time *rtctime);
static int bl602_settime(struct rtc_lowerhalf_s *lower,
                         const struct rtc_time *rtctime);
static bool bl602_havesettime(struct rtc_lowerhalf_s *lower);

#ifdef CONFIG_RTC_ALARM
static int bl602_setalarm(struct rtc_lowerhalf_s *lower,
                          const struct lower_setalarm_s *alarminfo);
static int bl602_setrelative(struct rtc_lowerhalf_s *lower,
                            const struct lower_setrelative_s *alarminfo);
static int bl602_cancelalarm(struct rtc_lowerhalf_s *lower,
                             int alarmid);
static int bl602_rdalarm(struct rtc_lowerhalf_s *lower,
                         struct lower_rdalarm_s *alarminfo);
#endif

#ifdef CONFIG_RTC_PERIODIC
static int bl602_setperiodic(struct rtc_lowerhalf_s *lower,
                            const struct lower_setperiodic_s *alarminfo);
static int bl602_cancelperiodic(struct rtc_lowerhalf_s *lower, int id);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* BL602 RTC driver operations */

static const struct rtc_ops_s g_rtc_ops =
{
  .rdtime      = bl602_rdtime,
  .settime     = bl602_settime,
  .havesettime = bl602_havesettime,
#ifdef CONFIG_RTC_ALARM
  .setalarm    = bl602_setalarm,
  .setrelative = bl602_setrelative,
  .cancelalarm = bl602_cancelalarm,
  .rdalarm     = bl602_rdalarm,
#endif
#ifdef CONFIG_RTC_PERIODIC
  .setperiodic    = bl602_setperiodic,
  .cancelperiodic = bl602_cancelperiodic,
#endif
};

/* BL602 RTC device state */

static struct bl602_lowerhalf_s g_rtc_lowerhalf =
{
  .ops     = &g_rtc_ops,
  .devlock = NXMUTEX_INITIALIZER,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint64_t bl602_rtc_get_timestamp_ms(void)
{
  uint64_t cnt;
  uint32_t val_low;
  uint32_t val_high;

  bl602_hbn_get_rtc_timer_val(&val_low, &val_high);

  cnt = (uint64_t)val_high << 32 | val_low;

  /* cnt * 1000 / 32768 */

  return (cnt >> 5) - (cnt >> 11) - (cnt >> 12);
}

#if defined(CONFIG_RTC_ALARM) || defined(CONFIG_RTC_PERIODIC)
static void bl602_rtc_set_timestamp_ms(uint64_t ms)
{
  ms += bl602_rtc_get_timestamp_ms();
  ms = ms * 32768 / 1000;

  bl602_hbn_set_rtc_timer(BL602_HBN_RTC_INT_DELAY_0T,
                          (uint32_t)ms,
                          (uint32_t)(ms >> 32),
                           BL602_HBN_RTC_COMP_BIT0_39);
}
#endif

/****************************************************************************
 * Name: bl602_alarm_callback
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

#if defined(CONFIG_RTC_ALARM) || defined(CONFIG_RTC_PERIODIC)
static int bl602_alarm_callback(void *arg)
{
  struct bl602_lowerhalf_s *priv = (struct bl602_lowerhalf_s *)arg;
  struct bl602_cbinfo_s *cbinfo = &priv->cbinfo;

  rtc_alarm_callback_t cb = (rtc_alarm_callback_t)cbinfo->cb;
  void *p_arg           = (void *)cbinfo->priv;

#ifdef CONFIG_RTC_PERIODIC
  if (priv->periodic_enable)
    {
      uint64_t time_stamp_s;
      struct timespec *tm_spec = &priv->periodic.period;

      time_stamp_s = tm_spec->tv_sec;

      if (time_stamp_s)
        {
          bl602_rtc_set_timestamp_ms(time_stamp_s * 1000);
          bl602_hbn_out0_int_enable();
        }

      if (priv->periodic.cb)
        {
          priv->periodic.cb(priv->periodic.priv, priv->periodic.id);
        }
    }
#endif

  cbinfo->cb              = NULL;
  cbinfo->priv            = NULL;

  /* Perform the callback */

  if (cb != NULL)
    {
      cb(p_arg, 0);
    }

  return OK;
}
#endif /* CONFIG_RTC_ALARM */

/****************************************************************************
 * Name: bl602_rdtime
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

static int bl602_rdtime(struct rtc_lowerhalf_s *lower,
                        struct rtc_time *rtctime)
{
  struct bl602_lowerhalf_s *priv;
  uint64_t time_stamp_s;
  struct rtc_time tim;

  priv = (struct bl602_lowerhalf_s *)lower;
  time_stamp_s = bl602_rtc_get_timestamp_ms() / 1000;

  tim = priv->rtc_base;

  if (tim.tm_year >= 1900)
    {
      tim.tm_year -= 1900;
    }

  time_stamp_s += mktime((struct tm *)&tim);
  gmtime_r((const time_t *)&time_stamp_s, (struct tm *)&rtctime);

  return OK;
}

/****************************************************************************
 * Name: bl602_settime
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

static int bl602_settime(struct rtc_lowerhalf_s *lower,
                         const struct rtc_time *rtctime)
{
  struct bl602_lowerhalf_s *priv = (struct bl602_lowerhalf_s *)lower;

  if (rtctime->tm_year < 1900)
    {
      return -ETIME;
    }

  priv->rtc_base = *rtctime;

  return OK;
}

/****************************************************************************
 * Name: bl602_havesettime
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

static bool bl602_havesettime(struct rtc_lowerhalf_s *lower)
{
  struct bl602_lowerhalf_s *priv = (struct bl602_lowerhalf_s *)lower;

  return (priv->rtc_base.tm_year != 0);
}

/****************************************************************************
 * Name: bl602_setalarm
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
static int bl602_setalarm(struct rtc_lowerhalf_s *lower,
                          const struct lower_setalarm_s *alarminfo)
{
  struct bl602_lowerhalf_s *priv;
  struct bl602_cbinfo_s *cbinfo;
  int ret;

  DEBUGASSERT(lower != NULL && alarminfo != NULL && alarminfo->id == 0);
  priv = (struct bl602_lowerhalf_s *)lower;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  ret = -EINVAL;
  if (alarminfo->id == 0)
    {
      uint64_t time_stamp_s;

      /* Convert the RTC time to a timespec */

      time_stamp_s    = mktime((struct tm *)&alarminfo->time);
      priv->rtc_alarm = alarminfo->time;

      /* Remember the callback information */

      cbinfo            = &priv->cbinfo;
      cbinfo->cb        = alarminfo->cb;
      cbinfo->priv      = alarminfo->priv;

      /* And set the alarm */

      bl602_rtc_set_timestamp_ms(time_stamp_s * 1000);

      bl602_hbn_out0_int_enable();

      ret = OK;
    }

  nxmutex_unlock(&priv->devlock);
  return ret;
}
#endif

/****************************************************************************
 * Name: bl602_setrelative
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
static int bl602_setrelative(struct rtc_lowerhalf_s *lower,
                             const struct lower_setrelative_s *alarminfo)
{
  struct bl602_lowerhalf_s *priv;
  struct lower_setalarm_s setalarm;
  struct rtc_time *time;
  time_t seconds;
  int ret = -EINVAL;

  priv = (struct bl602_lowerhalf_s *)lower;

  DEBUGASSERT(lower != NULL && alarminfo != NULL);

  if (alarminfo->id == 0 && alarminfo->reltime > 0)
    {
      /* Get the current time in broken out format */

      time = &priv->rtc_base;

      /* Convert to seconds since the epoch */

      seconds = mktime((struct tm *)time);

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

      ret = bl602_setalarm(lower, &setalarm);
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: bl602_cancelalarm
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
static int bl602_cancelalarm(struct rtc_lowerhalf_s *lower, int alarmid)
{
  struct bl602_lowerhalf_s *priv;
  struct bl602_cbinfo_s *cbinfo;

  DEBUGASSERT(lower != NULL);
  DEBUGASSERT(alarmid == 0);
  priv = (struct bl602_lowerhalf_s *)lower;

  /* Nullify callback information to reduce window for race conditions */

  cbinfo       = &priv->cbinfo;
  cbinfo->cb   = NULL;
  cbinfo->priv = NULL;

  memset(&priv->rtc_alarm, 0, sizeof(priv->rtc_alarm));

  /* Then cancel the alarm */

  bl602_hbn_out0_int_unregister(BL602_HBN_OUT0_INT_RTC);
  bl602_hbn_out0_int_disable();

  return OK;
}
#endif

/****************************************************************************
 * Name: bl602_rdalarm
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
static int bl602_rdalarm(struct rtc_lowerhalf_s *lower,
                         struct lower_rdalarm_s *alarminfo)
{
  struct bl602_lowerhalf_s *priv;

  DEBUGASSERT(lower != NULL && alarminfo != NULL && alarminfo->time != NULL);
  priv = (struct bl602_lowerhalf_s *)lower;

  if (alarminfo->id == 0)
    {
      *alarminfo->time = priv->rtc_alarm;
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: bl602_setperiodic
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
static int bl602_setperiodic(struct rtc_lowerhalf_s *lower,
                             const struct lower_setperiodic_s *alarminfo)
{
  struct bl602_lowerhalf_s *priv;
  irqstate_t flags;

  DEBUGASSERT(lower != NULL && alarminfo != NULL);
  priv = (struct bl602_lowerhalf_s *)lower;

  flags = enter_critical_section();
  priv->periodic = *alarminfo;
  priv->periodic_enable = 1;
  memcpy(&priv->periodic, alarminfo, sizeof(struct lower_setperiodic_s));
  leave_critical_section(flags);

  return OK;
}
#endif

/****************************************************************************
 * Name: bl602_cancelperiodic
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
static int bl602_cancelperiodic(struct rtc_lowerhalf_s *lower, int id)
{
  struct bl602_lowerhalf_s *priv;
  irqstate_t flags;

  DEBUGASSERT(lower != NULL);
  priv = (struct bl602_lowerhalf_s *)lower;

  DEBUGASSERT(id == 0);

  flags = enter_critical_section();
  priv->periodic_enable = 0;
  leave_critical_section(flags);

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_rtc_initialize
 *
 * Description:
 *   Initialize the hardware RTC per the selected configuration.  This
 *   function is called once during the OS initialization sequence
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_initialize(void)
{
#ifdef CONFIG_BL602_RTC_USE_XTAL32K
  bl602_hbn_sel(BL602_HBN_32K_XTAL);
#else
  bl602_hbn_sel(BL602_HBN_32K_RC);
#endif

  bl602_hbn_clear_rtc_counter();
  bl602_hbn_enable_rtc_counter();

  bl602_hbn_out0_int_register(BL602_HBN_OUT0_INT_RTC,
                              bl602_alarm_callback,
                              (void *)&g_rtc_lowerhalf);
  return OK;
}

/****************************************************************************
 * Name: bl602_rtc_lowerhalf_initialize
 *
 * Description:
 *   Instantiate the RTC lower half driver for the BL602.  General usage:
 *
 *     #include <nuttx/timers/rtc.h>
 *     #include "bl602_rtc.h"
 *
 *     struct rtc_lowerhalf_s *lower;
 *     lower = bl602_rtc_lowerhalf_initialize();
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

struct rtc_lowerhalf_s *bl602_rtc_lowerhalf_initialize(void)
{
  g_rtc_lowerhalf.rtc_base.tm_year = 70;
  g_rtc_lowerhalf.rtc_base.tm_mday = 1;

  return (struct rtc_lowerhalf_s *)&g_rtc_lowerhalf;
}
