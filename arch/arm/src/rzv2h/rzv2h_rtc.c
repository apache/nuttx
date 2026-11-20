/****************************************************************************
 * arch/arm/src/rzv2h/rzv2h_rtc.c
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
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/spinlock.h>
#include <nuttx/signal.h>
#include <nuttx/clock.h>
#include <nuttx/timers/rtc.h>
#include <nuttx/timers/arch_rtc.h>

#include "arm_internal.h"
#include "rzv2h_fsp_err.h"
#include "rzv2h_irq.h"
#include "rzv2h_rtc.h"

#include "r_rtc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_RTC_HIRES
#  error "CONFIG_RTC_HIRES must NOT be set with the RZ/V2H RTC driver"
#endif

#ifndef CONFIG_RTC_ARCH
#  error "CONFIG_RTC_ARCH must be set for the RZ/V2H RTC driver"
#endif

/* RTC calendar year range: 2000-2099 maps to tm_year 100-199. */
#define RZV2H_RTC_YEAR_MIN      100
#define RZV2H_RTC_YEAR_MAX      199

#ifdef CONFIG_RTC_PERIODIC
/* Periodic rate thresholds */

#  define RZV2H_RTC_PERIOD_NS_1_2      500000000L /* >= 0.5   s */
#  define RZV2H_RTC_PERIOD_NS_1_4      250000000L /* >= 0.25  s */
#  define RZV2H_RTC_PERIOD_NS_1_8      125000000L /* >= 0.125 s */
#  define RZV2H_RTC_PERIOD_NS_1_16      62500000L /* >= 1/16  s */
#  define RZV2H_RTC_PERIOD_NS_1_32      31250000L /* >= 1/32  s */
#  define RZV2H_RTC_PERIOD_NS_1_64      15625000L /* >= 1/64  s */
#  define RZV2H_RTC_PERIOD_NS_1_128      7812500L /* >= 1/128 s */
#endif

#define RZV2H_IRQ_RTC_ALARM \
  RZV2H_FSP_TO_GIC_IRQ(CONFIG_RZV2H_RTC_ALARM_INTSEL)

#define RZV2H_IRQ_RTC_PERIODIC \
  RZV2H_FSP_TO_GIC_IRQ(CONFIG_RZV2H_RTC_PERIODIC_INTSEL)

#define RZV2H_IRQ_RTC_CARRY \
  RZV2H_FSP_TO_GIC_IRQ(CONFIG_RZV2H_RTC_CARRY_INTSEL)

/****************************************************************************
 * Private Types
 ****************************************************************************/

void rtc_alarm_periodic_isr(void);
void rtc_carry_isr(void);

/* Private state -- must be first member for cast compatibility with
 * struct rtc_lowerhalf_s.
 */

struct rzv2h_rtc_lowerhalf_s
{
  FAR const struct rtc_ops_s *ops;  /* MUST be first -- cast compatibility */

  spinlock_t  lock;           /* guards all fields below; taken by ISR and task */
  mutex_t     devlock;        /* NXMUTEX_INITIALIZER; task-context ops only */

  bool        havesettime;    /* written by settime, read by havesettime */

#ifdef CONFIG_RTC_ALARM
  bool        armed;          /* written by setalarm/cancelalarmc, callback */
  uint32_t    generation;     /* diagnostic counter */
  uint8_t     alarm_id;       /* copied from lower_setalarm_s.id at arm time */
  rtc_alarm_callback_t cb;    /* NULL-ed before invoking (NULL-first idiom) */
  FAR void   *priv;           /* upper-half container; freed by rtc_destroy() */
  struct rtc_time alarm_time; /* alarm time for rdalarm and settime re-arm */
#endif

#ifdef CONFIG_RTC_PERIODIC
  bool        periodic_armed;         /* written by setperiodic/cancelperiodic */
  uint8_t     periodic_id;            /* copied from lower_setperiodic_s.id */
  rtc_wakeup_callback_t periodic_cb;  /* NULL-ed before invoking */
  FAR void   *periodic_priv;          /* upper-half container */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  rzv2h_rtc_rdtime(FAR struct rtc_lowerhalf_s *lower,
                              FAR struct rtc_time *rtctime);
static int  rzv2h_rtc_settime(FAR struct rtc_lowerhalf_s *lower,
                               FAR const struct rtc_time *rtctime);
static bool rzv2h_rtc_havesettime(FAR struct rtc_lowerhalf_s *lower);

#ifdef CONFIG_RTC_ALARM
static int  rzv2h_rtc_setalarm_nolock(
              FAR struct rzv2h_rtc_lowerhalf_s *priv, uint8_t id,
              rtc_alarm_callback_t cb, FAR void *alarm_priv,
              FAR const struct rtc_time *alarm_time);
static int  rzv2h_rtc_setalarm(FAR struct rtc_lowerhalf_s *lower,
                      FAR const struct lower_setalarm_s *alarminfo);
static int  rzv2h_rtc_setrelative(FAR struct rtc_lowerhalf_s *lower,
                      FAR const struct lower_setrelative_s *alarminfo);
static int  rzv2h_rtc_cancelalarm(FAR struct rtc_lowerhalf_s *lower,
                                   int alarmid);
static int  rzv2h_rtc_rdalarm(FAR struct rtc_lowerhalf_s *lower,
                      FAR struct lower_rdalarm_s *alarminfo);
static int  rzv2h_rtc_alarm_interrupt(int irq, FAR void *context,
                                       FAR void *arg);
#endif

#ifdef CONFIG_RTC_PERIODIC
static int  rzv2h_rtc_setperiodic(FAR struct rtc_lowerhalf_s *lower,
                      FAR const struct lower_setperiodic_s *alarminfo);
static int  rzv2h_rtc_cancelperiodic(FAR struct rtc_lowerhalf_s *lower,
                                      int id);
static int  rzv2h_rtc_periodic_interrupt(int irq, FAR void *context,
                                          FAR void *arg);
#endif

static int  rzv2h_rtc_carry_interrupt(int irq, FAR void *context,
                                       FAR void *arg);
static void rzv2h_rtc_callback(rtc_callback_args_t *p_args);

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int  rzv2h_rtc_destroy(FAR struct rtc_lowerhalf_s *lower);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rtc_ops_s g_rtc_ops =
{
  .rdtime      = rzv2h_rtc_rdtime,
  .settime     = rzv2h_rtc_settime,
  .havesettime = rzv2h_rtc_havesettime,
#ifdef CONFIG_RTC_ALARM
  .setalarm    = rzv2h_rtc_setalarm,
  .setrelative = rzv2h_rtc_setrelative,
  .cancelalarm = rzv2h_rtc_cancelalarm,
  .rdalarm     = rzv2h_rtc_rdalarm,
#endif
#ifdef CONFIG_RTC_PERIODIC
  .setperiodic    = rzv2h_rtc_setperiodic,
  .cancelperiodic = rzv2h_rtc_cancelperiodic,
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  .destroy        = rzv2h_rtc_destroy,
#endif
};

static rtc_instance_ctrl_t g_rtc_ctrl;

static const rtc_error_adjustment_cfg_t g_rtc_err_adj_cfg =
{
  .adjustment_mode   = RTC_ERROR_ADJUSTMENT_MODE_MANUAL,
  .adjustment_period = RTC_ERROR_ADJUSTMENT_PERIOD_NONE,
  .adjustment_type   = RTC_ERROR_ADJUSTMENT_NONE,
  .adjustment_value  = 0,
};

static const rtc_extended_cfg_t g_rtc_cfg_extend =
{
  /* EVK drives the RTC sub-clock from a 32.768 kHz crystal */

  .clock_bypass_mode = CRYSTAL_OSCILLATOR_MODE,
};

static struct rzv2h_rtc_lowerhalf_s g_rtc_lowerhalf =
{
  .ops     = &g_rtc_ops,
  .devlock = NXMUTEX_INITIALIZER,
};

static const rtc_cfg_t g_rtc_cfg =
{
  /* Software hint only; source is fixed in hardware on rzv2h. */

  .clock_source       = RTC_CLOCK_SOURCE_SUBCLK,
  .freq_compare_value = 0,
  .p_err_cfg          = &g_rtc_err_adj_cfg,
#ifdef CONFIG_RTC_ALARM
  .alarm_ipl          = (uint8_t)CONFIG_RZV2H_RTC_ALARM_PRIORITY,
  .alarm_irq          = (IRQn_Type)CONFIG_RZV2H_RTC_ALARM_INTSEL,
#else
  .alarm_ipl       = (uint8_t)BSP_IRQ_DISABLED,
  .alarm_irq       = (IRQn_Type)(-1),
#endif /* CONFIG_RTC_ALARM */
#ifdef CONFIG_RTC_PERIODIC
  .periodic_ipl       = (uint8_t)CONFIG_RZV2H_RTC_PERIODIC_PRIORITY,
  .periodic_irq       = (IRQn_Type)CONFIG_RZV2H_RTC_PERIODIC_INTSEL,
#else
  .periodic_ipl       = (uint8_t)BSP_IRQ_DISABLED,
  .periodic_irq       = (IRQn_Type)(-1),
#endif /* CONFIG_RTC_PERIODIC */
  .carry_ipl          = (uint8_t)CONFIG_RZV2H_RTC_CARRY_PRIORITY,
  .carry_irq          = (IRQn_Type)CONFIG_RZV2H_RTC_CARRY_INTSEL,
  .p_callback         = rzv2h_rtc_callback,
  .p_context          = &g_rtc_lowerhalf,
  .p_extend           = &g_rtc_cfg_extend,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rzv2h_rtc_rdtime_internal (internal helper, no lower-half wrapper)
 *
 * Description:
 *   Read the current RTC calendar time into *rtctime.
 *   Spinlock-protected; safe pre-scheduler and with IRQs off.
 *
 ****************************************************************************/

static int rzv2h_rtc_rdtime_internal(FAR struct rtc_time *rtctime)
{
  rtc_time_t tmp;
  irqstate_t flags;
  fsp_err_t  err;

  if (rtctime == NULL)
    {
      return -EINVAL;
    }

  /* FSP writes only 7 of 11 struct tm fields; zero the rest first. */

  memset(&tmp, 0, sizeof(tmp));

  flags = spin_lock_irqsave(&g_rtc_lowerhalf.lock);
  err   = R_RTC_CalendarTimeGet(&g_rtc_ctrl, &tmp);
  spin_unlock_irqrestore(&g_rtc_lowerhalf.lock, flags);

  if (err != FSP_SUCCESS)
    {
      return rzv2h_fsp_err_to_errno(err);
    }

  tmp.tm_isdst = 0;
  tmp.tm_yday  = 0;

  /* Field-by-field copy so the tm_nsec assumption is not silently
   * load-bearing if CONFIG_RTC_HIRES is ever selected.
   */

  rtctime->tm_sec  = tmp.tm_sec;
  rtctime->tm_min  = tmp.tm_min;
  rtctime->tm_hour = tmp.tm_hour;
  rtctime->tm_mday = tmp.tm_mday;
  rtctime->tm_mon  = tmp.tm_mon;
  rtctime->tm_year = tmp.tm_year;
  rtctime->tm_wday = tmp.tm_wday;
  rtctime->tm_yday = 0;
  rtctime->tm_isdst = 0;

  return OK;
}

/****************************************************************************
 * Name: rzv2h_rtc_rdtime
 *
 * Description:
 *   rtc_ops_s.rdtime -- return the current RTC time.
 *
 ****************************************************************************/

static int rzv2h_rtc_rdtime(FAR struct rtc_lowerhalf_s *lower,
                             FAR struct rtc_time *rtctime)
{
  return rzv2h_rtc_rdtime_internal(rtctime);
}

/****************************************************************************
 * Name: rzv2h_rtc_settime
 *
 * Description:
 *   rtc_ops_s.settime -- program the RTC calendar.
 *
 ****************************************************************************/

static int rzv2h_rtc_settime(FAR struct rtc_lowerhalf_s *lower,
                              FAR const struct rtc_time *rtctime)
{
  struct rzv2h_rtc_lowerhalf_s *priv =
    (struct rzv2h_rtc_lowerhalf_s *)lower;
  rtc_time_t tmp;
  irqstate_t flags;
  fsp_err_t  err;
  int        ret;
  time_t     t;
#ifdef CONFIG_RTC_ALARM
  bool       was_armed;
  struct rtc_time saved_alarm;
  rtc_alarm_callback_t saved_cb;
  FAR void  *saved_priv;
  uint8_t    saved_id;
#endif

  if (rtctime == NULL)
    {
      return -EINVAL;
    }

  /* Copy immediately -- on the LPWORK path the caller's pointer is
   * &g_rtc_to_set, a static that a later clock_settime() overwrites
   * without waiting for the worker.
   */

  tmp.tm_sec  = rtctime->tm_sec;
  tmp.tm_min  = rtctime->tm_min;
  tmp.tm_hour = rtctime->tm_hour;
  tmp.tm_mday = rtctime->tm_mday;
  tmp.tm_mon  = rtctime->tm_mon;
  tmp.tm_year = rtctime->tm_year;
  tmp.tm_wday = rtctime->tm_wday;
  tmp.tm_yday = rtctime->tm_yday;
  tmp.tm_isdst = 0;

  /* Normalise to guarantee correct tm_wday. */

  t = timegm((FAR struct tm *)&tmp);
  gmtime_r(&t, (FAR struct tm *)&tmp);

  if (tmp.tm_year < RZV2H_RTC_YEAR_MIN ||
      tmp.tm_year > RZV2H_RTC_YEAR_MAX)
    {
      return -EINVAL;
    }

  if (tmp.tm_sec > 59)
    {
      /* Only accepts 0-59. */

      return -EINVAL;
    }

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_RTC_ALARM
  /* Snapshot the armed alarm so we can re-arm after the software reset. */

  flags = spin_lock_irqsave(&priv->lock);
  was_armed  = priv->armed;
  saved_alarm = priv->alarm_time;
  saved_cb   = priv->cb;
  saved_priv = priv->priv;
  saved_id   = priv->alarm_id;
  spin_unlock_irqrestore(&priv->lock, flags);
#endif

  flags = spin_lock_irqsave(&priv->lock);
  err   = R_RTC_CalendarTimeSet(&g_rtc_ctrl, &tmp);
  spin_unlock_irqrestore(&priv->lock, flags);

  if (err != FSP_SUCCESS)
    {
      nxmutex_unlock(&priv->devlock);
      return rzv2h_fsp_err_to_errno(err);
    }

  /* Re-arm the alarm if one was active before the software reset. */

#ifdef CONFIG_RTC_ALARM
  if (was_armed && saved_cb != NULL)
    {
      int rearm_ret = rzv2h_rtc_setalarm_nolock(priv, saved_id, saved_cb,
                                                 saved_priv, &saved_alarm);

      if (rearm_ret < 0)
        {
          rtcerr("settime: alarm re-arm failed: %d\n", rearm_ret);
        }
    }
#endif

  /* Mark time as set. */

  flags = spin_lock_irqsave(&priv->lock);
  priv->havesettime = true;
  spin_unlock_irqrestore(&priv->lock, flags);

  nxmutex_unlock(&priv->devlock);
  return OK;
}

/****************************************************************************
 * Name: rzv2h_rtc_havesettime
 *
 * Description:
 *   rtc_ops_s.havesettime -- report whether the time has been set.
 *
 ****************************************************************************/

static bool rzv2h_rtc_havesettime(FAR struct rtc_lowerhalf_s *lower)
{
  struct rzv2h_rtc_lowerhalf_s *priv =
    (struct rzv2h_rtc_lowerhalf_s *)lower;
  irqstate_t flags;
  bool       result;

  flags  = spin_lock_irqsave(&priv->lock);
  result = priv->havesettime;
  spin_unlock_irqrestore(&priv->lock, flags);

  return result;
}

#ifdef CONFIG_RTC_ALARM

/****************************************************************************
 * Name: rzv2h_rtc_setalarm_nolock
 *
 * Description:
 *   Shared _nolock arm core used by setalarm, setrelative, and the
 *   settime re-arm path.  Caller must hold devlock.
 *
 ****************************************************************************/

static int rzv2h_rtc_setalarm_nolock(
  FAR struct rzv2h_rtc_lowerhalf_s *priv,
  uint8_t id,
  rtc_alarm_callback_t cb,
  FAR void *alarm_priv,
  FAR const struct rtc_time *alarm_time)
{
  rtc_alarm_time_t a;
  irqstate_t       flags;
  fsp_err_t        err;

  memset(&a, 0, sizeof(a));
  a.time.tm_sec  = alarm_time->tm_sec;
  a.time.tm_min  = alarm_time->tm_min;
  a.time.tm_hour = alarm_time->tm_hour;
  a.time.tm_mday = alarm_time->tm_mday;
  a.time.tm_mon  = alarm_time->tm_mon;
  a.time.tm_year = alarm_time->tm_year;
  a.time.tm_wday = alarm_time->tm_wday;

  a.sec_match   = true;
  a.min_match   = true;
  a.hour_match  = true;
  a.mday_match  = true;
  a.mon_match   = true;
  a.year_match  = true;
  a.dayofweek_match = false;

  /* Publish state BEFORE the FSP call so a spurious mid-rewrite callback
   * finds armed == true and a matching generation.
   */

  flags = spin_lock_irqsave(&priv->lock);
  priv->generation++;
  priv->armed      = true;
  priv->cb         = cb;
  priv->priv       = alarm_priv;
  priv->alarm_id   = id;
  memcpy(&priv->alarm_time, alarm_time, sizeof(priv->alarm_time));
  spin_unlock_irqrestore(&priv->lock, flags);

  flags = spin_lock_irqsave(&priv->lock);
  err   = R_RTC_CalendarAlarmSet(&g_rtc_ctrl, &a);
  spin_unlock_irqrestore(&priv->lock, flags);

  if (err != FSP_SUCCESS)
    {
      /* Undo the state publication on failure. */

      flags = spin_lock_irqsave(&priv->lock);
      priv->armed    = false;
      priv->cb       = NULL;
      priv->priv     = NULL;
      priv->generation++;
      spin_unlock_irqrestore(&priv->lock, flags);

      up_disable_irq(RZV2H_IRQ_RTC_ALARM);
      return rzv2h_fsp_err_to_errno(err);
    }

  return OK;
}

/****************************************************************************
 * Name: rzv2h_rtc_setalarm
 *
 * Description:
 *   rtc_ops_s.setalarm -- arm a one-shot alarm at an absolute time.
 *
 ****************************************************************************/

static int rzv2h_rtc_setalarm(FAR struct rtc_lowerhalf_s *lower,
                               FAR const struct lower_setalarm_s *alarminfo)
{
  struct rzv2h_rtc_lowerhalf_s *priv =
    (struct rzv2h_rtc_lowerhalf_s *)lower;
  struct rtc_time now;
  struct rtc_time alarm_copy;
  irqstate_t flags;
  time_t     t;
  int        ret;

  if (alarminfo == NULL)
    {
      return -EINVAL;
    }

  if (alarminfo->id != 0)
    {
      /* Only one alarm channel on rzv2h. */

      return -EINVAL;
    }

  /* Copy from the stack-local struct immediately. */

  memcpy(&alarm_copy, &alarminfo->time, sizeof(alarm_copy));

  /* Normalise tm_wday. */

  t = timegm((FAR struct tm *)&alarm_copy);
  gmtime_r(&t, (FAR struct tm *)&alarm_copy);

  /* Year range check. */

  if (alarm_copy.tm_year < RZV2H_RTC_YEAR_MIN ||
      alarm_copy.tm_year > RZV2H_RTC_YEAR_MAX)
    {
      return -EINVAL;
    }

  /* Calendar must be running before an alarm can be set. */

  flags = spin_lock_irqsave(&priv->lock);
  if (!priv->havesettime)
    {
      spin_unlock_irqrestore(&priv->lock, flags);
      return -EPERM;
    }

  spin_unlock_irqrestore(&priv->lock, flags);

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  /* Refuse non-future alarms. */

  ret = rzv2h_rtc_rdtime_internal(&now);
  if (ret < 0)
    {
      nxmutex_unlock(&priv->devlock);
      return ret;
    }

  if (timegm((FAR struct tm *)&alarm_copy) <=
      timegm((FAR struct tm *)&now))
    {
      nxmutex_unlock(&priv->devlock);
      return -EINVAL;
    }

  ret = rzv2h_rtc_setalarm_nolock(priv, alarminfo->id,
                                   alarminfo->cb, alarminfo->priv,
                                   &alarm_copy);

  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Name: rzv2h_rtc_setrelative
 *
 * Description:
 *   rtc_ops_s.setrelative -- arm an alarm reltime seconds from now.
 *
 ****************************************************************************/

static int rzv2h_rtc_setrelative(FAR struct rtc_lowerhalf_s *lower,
                          FAR const struct lower_setrelative_s *alarminfo)
{
  struct rzv2h_rtc_lowerhalf_s *priv =
    (struct rzv2h_rtc_lowerhalf_s *)lower;
  struct rtc_time now;
  struct rtc_time target;
  time_t          t;
  time_t          reltime;
  irqstate_t      flags;
  int             ret;

  if (alarminfo == NULL)
    {
      return -EINVAL;
    }

  if (alarminfo->id != 0)
    {
      return -EINVAL;
    }

  reltime = alarminfo->reltime;
  if (reltime <= 0)
    {
      return -EINVAL;
    }

  flags = spin_lock_irqsave(&priv->lock);
  if (!priv->havesettime)
    {
      spin_unlock_irqrestore(&priv->lock, flags);
      return -EPERM;
    }

  spin_unlock_irqrestore(&priv->lock, flags);

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  /* Read current time under the devlock so a concurrent settime cannot
   * land between the read and the arm.
   */

  ret = rzv2h_rtc_rdtime_internal(&now);
  if (ret < 0)
    {
      nxmutex_unlock(&priv->devlock);
      return ret;
    }

  t = timegm((FAR struct tm *)&now) + reltime;
  gmtime_r(&t, (FAR struct tm *)&target);

  /* Year overflow check. */

  if (target.tm_year < RZV2H_RTC_YEAR_MIN ||
      target.tm_year > RZV2H_RTC_YEAR_MAX)
    {
      nxmutex_unlock(&priv->devlock);
      return -EINVAL;
    }

  /* Upstream +1 guard: if the target is already past (read-arm gap),
   * add one second.
   */

  if (timegm((FAR struct tm *)&target) <=
      timegm((FAR struct tm *)&now))
    {
      t += 1;
      gmtime_r(&t, (FAR struct tm *)&target);
    }

  ret = rzv2h_rtc_setalarm_nolock(priv, alarminfo->id,
                                   alarminfo->cb, alarminfo->priv,
                                   &target);

  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Name: rzv2h_rtc_cancelalarm
 *
 * Description:
 *   rtc_ops_s.cancelalarm -- disarm the alarm.  IDEMPOTENT.
 *
 ****************************************************************************/

static int rzv2h_rtc_cancelalarm(FAR struct rtc_lowerhalf_s *lower,
                                  int alarmid)
{
  struct rzv2h_rtc_lowerhalf_s *priv =
    (struct rzv2h_rtc_lowerhalf_s *)lower;
  irqstate_t flags;
  int        ret;

  if (alarmid != 0)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  /* NULL-first ordering: clear cb/priv before releasing the spinlock so
   * a racing ISR that fires between the GIC mask and the state clear finds
   * armed == false and drops the callback.
   */

  flags = spin_lock_irqsave(&priv->lock);
  priv->generation++;
  priv->armed = false;
  priv->cb    = NULL;
  priv->priv  = NULL;
  spin_unlock_irqrestore(&priv->lock, flags);

  /* GIC mask first, then the native hardware cancel. */

  up_disable_irq(RZV2H_IRQ_RTC_ALARM);

  flags = spin_lock_irqsave(&priv->lock);
  R_RTC->RCR1 &= (uint8_t) ~R_RTC_RCR1_AIE_Msk;
  FSP_HARDWARE_REGISTER_WAIT((R_RTC->RCR1 & R_RTC_RCR1_AIE_Msk), 0U);
  R_RTC->RSR = (uint8_t)(R_RTC->RSR & ~R_RTC_RSR_AF_Msk);
  spin_unlock_irqrestore(&priv->lock, flags);

  nxmutex_unlock(&priv->devlock);
  return OK;
}

/****************************************************************************
 * Name: rzv2h_rtc_rdalarm
 *
 * Description:
 *   rtc_ops_s.rdalarm -- query the current alarm time.
 *
 ****************************************************************************/

static int rzv2h_rtc_rdalarm(FAR struct rtc_lowerhalf_s *lower,
                              FAR struct lower_rdalarm_s *alarminfo)
{
  struct rzv2h_rtc_lowerhalf_s *priv =
    (struct rzv2h_rtc_lowerhalf_s *)lower;
  rtc_alarm_time_t a;
  irqstate_t       flags;
  fsp_err_t        err;
  int              ret;

  if (alarminfo == NULL || alarminfo->id != 0 ||
      alarminfo->time == NULL)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  /* When armed, return the cached value -- it is authoritative and avoids
   * the case where a settime software reset cleared the alarm registers.
   */

  flags = spin_lock_irqsave(&priv->lock);
  if (priv->armed)
    {
      memcpy(alarminfo->time, &priv->alarm_time,
             sizeof(*alarminfo->time));
      spin_unlock_irqrestore(&priv->lock, flags);
      nxmutex_unlock(&priv->devlock);
      return OK;
    }

  spin_unlock_irqrestore(&priv->lock, flags);

  err = R_RTC_CalendarAlarmGet(&g_rtc_ctrl, &a);
  if (err != FSP_SUCCESS)
    {
      nxmutex_unlock(&priv->devlock);
      return rzv2h_fsp_err_to_errno(err);
    }

  /* Convert FSP rtc_alarm_time_t -> struct rtc_time.
   * Sanitise tm_mon underflow: rtc_bcd_to_dec returns uint8_t, so
   * 0 - 1 promotes to int yielding -1.
   */

  alarminfo->time->tm_sec  = a.time.tm_sec;
  alarminfo->time->tm_min  = a.time.tm_min;
  alarminfo->time->tm_hour = a.time.tm_hour;
  alarminfo->time->tm_mday = a.time.tm_mday;
  alarminfo->time->tm_mon  = (a.time.tm_mon < 0) ? 0 : a.time.tm_mon;
  alarminfo->time->tm_year = a.time.tm_year;
  alarminfo->time->tm_wday = a.time.tm_wday;
  alarminfo->time->tm_yday = 0;
  alarminfo->time->tm_isdst = 0;

  nxmutex_unlock(&priv->devlock);
  return OK;
}

/****************************************************************************
 * Name: rzv2h_rtc_alarm_interrupt
 *
 * Description:
 *   NuttX IRQ handler for the RTC alarm (GIC INTID 385, SEL0_IRQn).
 *   Reconstructs the FSP interrupt context and calls the FSP ISR.
 *
 ****************************************************************************/

static int rzv2h_rtc_alarm_interrupt(int irq, FAR void *context,
                                      FAR void *arg)
{
  IRQn_Type fsp_irq = (IRQn_Type)(irq - BSP_CORTEX_VECTOR_TABLE_ENTRIES);

  rzv2h_interrupt_common_handler(fsp_irq, rtc_alarm_periodic_isr);

  return OK;
}

#endif /* CONFIG_RTC_ALARM */

#ifdef CONFIG_RTC_PERIODIC

/****************************************************************************
 * Name: rzv2h_rtc_period_to_rate
 *
 * Description:
 *   Map a struct timespec period to an rtc_periodic_irq_select_t rate.
 *   rzv2h supports 1/128 s .. 2 s; 1/256 s is unavailable.
 *
 ****************************************************************************/

static int rzv2h_rtc_period_to_rate(FAR const struct timespec *period,
                                    FAR rtc_periodic_irq_select_t *rate)
{
  if (period->tv_sec >= 2)
    {
      *rate = RTC_PERIODIC_IRQ_SELECT_2_SECOND;
    }
  else if (period->tv_sec == 1)
    {
      *rate = RTC_PERIODIC_IRQ_SELECT_1_SECOND;
    }
  else if (period->tv_nsec >= RZV2H_RTC_PERIOD_NS_1_2)
    {
      *rate = RTC_PERIODIC_IRQ_SELECT_1_DIV_BY_2_SECOND;
    }
  else if (period->tv_nsec >= RZV2H_RTC_PERIOD_NS_1_4)
    {
      *rate = RTC_PERIODIC_IRQ_SELECT_1_DIV_BY_4_SECOND;
    }
  else if (period->tv_nsec >= RZV2H_RTC_PERIOD_NS_1_8)
    {
      *rate = RTC_PERIODIC_IRQ_SELECT_1_DIV_BY_8_SECOND;
    }
  else if (period->tv_nsec >= RZV2H_RTC_PERIOD_NS_1_16)
    {
      *rate = RTC_PERIODIC_IRQ_SELECT_1_DIV_BY_16_SECOND;
    }
  else if (period->tv_nsec >= RZV2H_RTC_PERIOD_NS_1_32)
    {
      *rate = RTC_PERIODIC_IRQ_SELECT_1_DIV_BY_32_SECOND;
    }
  else if (period->tv_nsec >= RZV2H_RTC_PERIOD_NS_1_64)
    {
      *rate = RTC_PERIODIC_IRQ_SELECT_1_DIV_BY_64_SECOND;
    }
  else if (period->tv_nsec >= RZV2H_RTC_PERIOD_NS_1_128)
    {
      *rate = RTC_PERIODIC_IRQ_SELECT_1_DIV_BY_128_SECOND;
    }
  else
    {
      /* Below 1/128 s (or the unavailable 1/256 s) -- refused. */

      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: rzv2h_rtc_setperiodic
 *
 * Description:
 *   rtc_ops_s.setperiodic -- start a periodic wakeup.  The callback fires
 *   every period until cancelperiodic.
 *
 ****************************************************************************/

static int rzv2h_rtc_setperiodic(FAR struct rtc_lowerhalf_s *lower,
                    FAR const struct lower_setperiodic_s *alarminfo)
{
  struct rzv2h_rtc_lowerhalf_s *priv =
    (struct rzv2h_rtc_lowerhalf_s *)lower;
  rtc_periodic_irq_select_t rate;
  irqstate_t flags;
  fsp_err_t  err;
  int        ret;

  if (alarminfo == NULL || alarminfo->id != 0)
    {
      return -EINVAL;
    }

  ret = rzv2h_rtc_period_to_rate(&alarminfo->period, &rate);
  if (ret < 0)
    {
      return ret;
    }

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  /* Publish state before the FSP call so a spurious callback finds
   * periodic_armed == true.
   */

  flags = spin_lock_irqsave(&priv->lock);
  priv->periodic_armed = true;
  priv->periodic_id    = alarminfo->id;
  priv->periodic_cb    = alarminfo->cb;
  priv->periodic_priv  = alarminfo->priv;
  spin_unlock_irqrestore(&priv->lock, flags);

  flags = spin_lock_irqsave(&priv->lock);
  err   = R_RTC_PeriodicIrqRateSet(&g_rtc_ctrl, rate);
  spin_unlock_irqrestore(&priv->lock, flags);

  if (err != FSP_SUCCESS)
    {
      flags = spin_lock_irqsave(&priv->lock);
      priv->periodic_armed = false;
      priv->periodic_cb    = NULL;
      priv->periodic_priv  = NULL;
      spin_unlock_irqrestore(&priv->lock, flags);

      up_disable_irq(RZV2H_IRQ_RTC_PERIODIC);
      nxmutex_unlock(&priv->devlock);
      return rzv2h_fsp_err_to_errno(err);
    }

  nxmutex_unlock(&priv->devlock);
  return OK;
}

/****************************************************************************
 * Name: rzv2h_rtc_cancelperiodic
 *
 * Description:
 *   rtc_ops_s.cancelperiodic -- stop the periodic wakeup.  IDEMPOTENT.
 *
 ****************************************************************************/

static int rzv2h_rtc_cancelperiodic(FAR struct rtc_lowerhalf_s *lower,
                                     int id)
{
  struct rzv2h_rtc_lowerhalf_s *priv =
    (struct rzv2h_rtc_lowerhalf_s *)lower;
  irqstate_t flags;
  int        ret;

  if (id != 0)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  flags = spin_lock_irqsave(&priv->lock);
  priv->periodic_armed = false;
  priv->periodic_cb    = NULL;
  priv->periodic_priv  = NULL;
  spin_unlock_irqrestore(&priv->lock, flags);

  up_disable_irq(RZV2H_IRQ_RTC_PERIODIC);

  flags = spin_lock_irqsave(&priv->lock);
  R_RTC->RCR1 &= (uint8_t) ~(R_RTC_RCR1_PIE_Msk | R_RTC_RCR1_PES_Msk);
  FSP_HARDWARE_REGISTER_WAIT((R_RTC->RCR1 & R_RTC_RCR1_PIE_Msk), 0U);
  R_RTC->RSR = (uint8_t)(R_RTC->RSR & ~R_RTC_RSR_PF_Msk);
  spin_unlock_irqrestore(&priv->lock, flags);

  nxmutex_unlock(&priv->devlock);
  return OK;
}

/****************************************************************************
 * Name: rzv2h_rtc_periodic_interrupt
 *
 * Description:
 *   NuttX IRQ handler for the RTC periodic wakeup.
 *
 ****************************************************************************/

static int rzv2h_rtc_periodic_interrupt(int irq, FAR void *context,
                                         FAR void *arg)
{
  IRQn_Type fsp_irq = (IRQn_Type)(irq - BSP_CORTEX_VECTOR_TABLE_ENTRIES);

  rzv2h_interrupt_common_handler(fsp_irq, rtc_alarm_periodic_isr);

  return OK;
}

#endif /* CONFIG_RTC_PERIODIC */

/****************************************************************************
 * Name: rzv2h_rtc_callback
 *
 * Description:
 *   FSP callback invoked from rtc_alarm_periodic_isr() in hardware interrupt
 *   context.  Delivers the alarm expiry to the NuttX upper half.
 *
 ****************************************************************************/

static void rzv2h_rtc_callback(rtc_callback_args_t *p_args)
{
#if defined(CONFIG_RTC_ALARM) || defined(CONFIG_RTC_PERIODIC)
  struct rzv2h_rtc_lowerhalf_s *priv = &g_rtc_lowerhalf;
  rtc_event_t event;
  irqstate_t  flags;

  if (p_args == NULL)
    {
      return;
    }

  event = p_args->event;

  switch (event)
    {
#ifdef CONFIG_RTC_ALARM
      case RTC_EVENT_ALARM_IRQ:
        {
          rtc_alarm_callback_t cb;
          FAR void            *alarm_priv;
          int                  alarm_id;

          /* NULL-first ordering: clear cb/priv before releasing. */

          flags = spin_lock_irqsave(&priv->lock);
          if (!priv->armed)
            {
              spin_unlock_irqrestore(&priv->lock, flags);
              return;
            }

          cb         = priv->cb;
          alarm_priv = priv->priv;
          alarm_id   = (int)priv->alarm_id;

          priv->armed = false;
          priv->generation++;
          priv->cb    = NULL;
          priv->priv  = NULL;
          spin_unlock_irqrestore(&priv->lock, flags);

          /* One-shot disarm: mask the GIC line.  RCR1.AIE is still set but
           * the GIC mask prevents further delivery until the next arm.
           */

          up_disable_irq(RZV2H_IRQ_RTC_ALARM);

          if (cb != NULL)
            {
              cb(alarm_priv, alarm_id);   /* legal from ISR context */
            }
        }
        break;
#endif /* CONFIG_RTC_ALARM */

#ifdef CONFIG_RTC_PERIODIC
      case RTC_EVENT_PERIODIC_IRQ:
        {
          rtc_wakeup_callback_t pcb;
          FAR void             *ppriv;
          int                   pid;

          flags = spin_lock_irqsave(&priv->lock);
          if (!priv->periodic_armed)
            {
              spin_unlock_irqrestore(&priv->lock, flags);
              return;
            }

          /* Do NOT clear periodic_armed -- it fires every period until
           * cancelperiodic.
           */

          pcb   = priv->periodic_cb;
          ppriv = priv->periodic_priv;
          pid   = (int)priv->periodic_id;
          spin_unlock_irqrestore(&priv->lock, flags);

          if (pcb != NULL)
            {
              pcb(ppriv, pid);            /* legal from ISR context */
            }
        }
        break;
#endif /* CONFIG_RTC_PERIODIC */

      default:
        break;
    }

#endif /* CONFIG_RTC_ALARM || CONFIG_RTC_PERIODIC */
}

/****************************************************************************
 * Name: rzv2h_rtc_carry_interrupt
 *
 * Description:
 *   NuttX IRQ handler for the RTC carry (GIC INTID 386, SEL1_IRQn).
 *   Reconstructs the FSP interrupt context and calls the FSP ISR.
 *
 ****************************************************************************/

static int rzv2h_rtc_carry_interrupt(int irq, FAR void *context,
                                      FAR void *arg)
{
  IRQn_Type fsp_irq = (IRQn_Type)(irq - BSP_CORTEX_VECTOR_TABLE_ENTRIES);

  rzv2h_interrupt_common_handler(fsp_irq, rtc_carry_isr);

  return OK;
}

/****************************************************************************
 * Name: rzv2h_rtc_destroy
 *
 * Description:
 *   rtc_ops_s.destroy -- tear the RTC down and close the RTC HW.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int rzv2h_rtc_destroy(FAR struct rtc_lowerhalf_s *lower)
{
  FAR struct rzv2h_rtc_lowerhalf_s *priv =
    (FAR struct rzv2h_rtc_lowerhalf_s *)lower;
  irqstate_t flags;
  int        ret = OK;

  /* 1. Mask every RTC GIC line first so no new request can be delivered */

  up_disable_irq(RZV2H_IRQ_RTC_CARRY);
#ifdef CONFIG_RTC_ALARM
  up_disable_irq(RZV2H_IRQ_RTC_ALARM);
#endif
#ifdef CONFIG_RTC_PERIODIC
  up_disable_irq(RZV2H_IRQ_RTC_PERIODIC);
#endif

  /* 2. Clear the adapter's own state. */

  flags = spin_lock_irqsave(&priv->lock);

#ifdef CONFIG_RTC_ALARM
  priv->armed = false;
  priv->cb    = NULL;
  priv->priv  = NULL;
  priv->generation++;
#endif

#ifdef CONFIG_RTC_PERIODIC
  priv->periodic_armed = false;
  priv->periodic_cb    = NULL;
  priv->periodic_priv  = NULL;
#endif

  spin_unlock_irqrestore(&priv->lock, flags);

  /* 3. Stops the counter, disables the GIC lines and RTC clock. */

  R_RTC_Close(&g_rtc_ctrl);

  /* 4. De-initializte the NuttX IRQ handlers. */

  irq_detach(RZV2H_IRQ_RTC_CARRY);
  ret = rzv2h_intsel_disconnect_event(CONFIG_RZV2H_RTC_CARRY_INTSEL);
  if (ret < 0)
    {
      rtcerr("IRQSEL un-route carry failed: %d\n", ret);
    }
#ifdef CONFIG_RTC_ALARM

  irq_detach(RZV2H_IRQ_RTC_ALARM);
  ret = rzv2h_intsel_disconnect_event(CONFIG_RZV2H_RTC_ALARM_INTSEL);
  if (ret < 0)
    {
      rtcerr("IRQSEL un-route alarm failed: %d\n", ret);
    }
#endif
#ifdef CONFIG_RTC_PERIODIC

  irq_detach(RZV2H_IRQ_RTC_PERIODIC);
  ret = rzv2h_intsel_disconnect_event(CONFIG_RZV2H_RTC_PERIODIC_INTSEL);
  if (ret < 0)
    {
      rtcerr("IRQSEL un-route periodic failed: %d\n", ret);
    }
#endif

  return ret;
}
#endif /* CONFIG_DISABLE_PSEUDOFS_OPERATIONS */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rzv2h_rtc_lowerhalf
 *
 * Description:
 *   Return a pointer to the RTC lower-half driver instance.
 *   Called from rzv2h_bringup() to pass to rtc_initialize() and
 *   up_rtc_set_lowerhalf().
 *
 ****************************************************************************/

FAR struct rtc_lowerhalf_s *rzv2h_rtc_lowerhalf(void)
{
  return (FAR struct rtc_lowerhalf_s *)&g_rtc_lowerhalf;
}

/****************************************************************************
 * Name: up_rtc_initialize
 *
 * Description:
 *   Initialize the builtin MCU RTC.  Called once very early in the OS
 *   initialization sequence by clock_initialize().
 *
 ****************************************************************************/

int up_rtc_initialize(void)
{
#ifdef CONFIG_RTC_DATETIME
  /* Turn on the clock power */

  R_BSP_BypassModeCfg(BSP_BYPASS_OSCILLATOR_RTC,
                                          BSP_BYPASS_MODE_CRYSTAL_OSC, 0);
  R_BSP_MODULE_START(FSP_IP_RTC, 0);
#endif
  return OK;
}

/****************************************************************************
 * Name: up_rtc_getdatetime
 *
 * Description:
 *   CONFIG_RTC_DATETIME arch hook used by clock_basetime() to seed the
 *   system clock.  Strong definition overriding the weak wrapper in
 *   drivers/timers/arch_rtc.c.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_DATETIME
static uint8_t rzv2h_bcd2dec(uint8_t v)
{
  return (uint8_t)((((v & 0xf0) >> 4) * 10) + (v & 0x0f));
}

int up_rtc_getdatetime(FAR struct tm *tp)
{
  /* Only seed the system clock when the RTC has been started */

  if (R_RTC->RCR2_b.START == 1U)
    {
      tp->tm_sec  = (int)rzv2h_bcd2dec((uint8_t)R_RTC->RSECCNT);
      tp->tm_min  = (int)rzv2h_bcd2dec((uint8_t)R_RTC->RMINCNT);
      tp->tm_hour = (int)rzv2h_bcd2dec((uint8_t)(R_RTC->RHRCNT & 0x3f));
      tp->tm_wday = (int)rzv2h_bcd2dec((uint8_t)R_RTC->RWKCNT);
      tp->tm_mday = (int)rzv2h_bcd2dec((uint8_t)R_RTC->RDAYCNT);
      tp->tm_mon  = (int)rzv2h_bcd2dec((uint8_t)R_RTC->RMONCNT) - 1;
      tp->tm_year = (int)rzv2h_bcd2dec((uint8_t)R_RTC->RYRCNT) + 100;
      tp->tm_yday = 0;
      tp->tm_isdst = 0;

      g_rtc_lowerhalf.havesettime = true;
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: rzv2h_rtc_initialize
 *
 * Description:
 *   Initialize the RTC lower-half driver.  Called from rzv2h_bringup()
 *   before rtc_initialize().
 *
 ****************************************************************************/

int rzv2h_rtc_initialize(void)
{
  fsp_err_t  err;
  int        ret;

  /* Step 1: Route SELECT slots and attach IRQs. */

  ret = rzv2h_intsel_connect_event(CONFIG_RZV2H_RTC_CARRY_INTSEL,
                    RZV2H_IRQSEL_RTC_CUP, BSP_GIC_SPI_DETECT_EDGE);
  if (ret < 0)
    {
      rtcerr("IRQSEL route carry failed: %d\n", ret);
      goto err_close;
    }

  ret = irq_attach(RZV2H_IRQ_RTC_CARRY, rzv2h_rtc_carry_interrupt,
                   &g_rtc_lowerhalf);
  if (ret < 0)
    {
      rtcerr("irq_attach carry failed: %d\n", ret);
      goto err_close;
    }

#ifdef CONFIG_RTC_ALARM
  ret = rzv2h_intsel_connect_event(CONFIG_RZV2H_RTC_ALARM_INTSEL,
                      RZV2H_IRQSEL_RTC_ALM, BSP_GIC_SPI_DETECT_EDGE);
  if (ret < 0)
    {
      rtcerr("IRQSEL route alarm failed: %d\n", ret);
      goto err_close;
    }

  ret = irq_attach(RZV2H_IRQ_RTC_ALARM, rzv2h_rtc_alarm_interrupt,
                   &g_rtc_lowerhalf);
  if (ret < 0)
    {
      rtcerr("irq_attach alarm failed: %d\n", ret);
      goto err_close;
    }

#endif

#ifdef CONFIG_RTC_PERIODIC
  ret = rzv2h_intsel_connect_event(CONFIG_RZV2H_RTC_PERIODIC_INTSEL,
                        RZV2H_IRQSEL_RTC_PRD, BSP_GIC_SPI_DETECT_EDGE);
  if (ret < 0)
    {
      rtcerr("IRQSEL route periodic failed: %d\n", ret);
      goto err_close;
    }

  ret = irq_attach(RZV2H_IRQ_RTC_PERIODIC, rzv2h_rtc_periodic_interrupt,
                   &g_rtc_lowerhalf);
  if (ret < 0)
    {
      rtcerr("irq_attach periodic failed: %d\n", ret);
      goto err_close;
    }

#endif

  /* Step 2: Open the FSP driver. */

  err = R_RTC_Open(&g_rtc_ctrl, &g_rtc_cfg);
  if (err != FSP_SUCCESS)
    {
      rtcerr("R_RTC_Open failed: %d\n", (int)err);
      return rzv2h_fsp_err_to_errno(err);
    }

  return OK;

err_close:
  R_RTC_Close(&g_rtc_ctrl);

  /* De-initializte the NuttX IRQ handlers. */

  irq_detach(RZV2H_IRQ_RTC_CARRY);
  ret = rzv2h_intsel_disconnect_event(CONFIG_RZV2H_RTC_CARRY_INTSEL);
  if (ret < 0)
    {
      rtcerr("IRQSEL un-route carry failed: %d\n", ret);
    }
#ifdef CONFIG_RTC_ALARM

  irq_detach(RZV2H_IRQ_RTC_ALARM);
  ret = rzv2h_intsel_disconnect_event(CONFIG_RZV2H_RTC_ALARM_INTSEL);
  if (ret < 0)
    {
      rtcerr("IRQSEL un-route alarm failed: %d\n", ret);
    }
#endif
#ifdef CONFIG_RTC_PERIODIC

  irq_detach(RZV2H_IRQ_RTC_PERIODIC);
  ret = rzv2h_intsel_disconnect_event(CONFIG_RZV2H_RTC_PERIODIC_INTSEL);
  if (ret < 0)
    {
      rtcerr("IRQSEL un-route periodic failed: %d\n", ret);
    }
#endif

  return ret;
}
