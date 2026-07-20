/****************************************************************************
 * arch/arm/src/common/ameba/ameba_rtc.c
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

/* NuttX RTC lower half for the Realtek Ameba RTC.  It exposes the on-chip
 * real-time clock as a NuttX date/time RTC at /dev/rtc0 (rdtime/settime)
 * and, when CONFIG_RTC_ALARM is set, a single one-shot alarm (setalarm/
 * rdalarm/cancelalarm/setrelative) whose expiry fires the RTC interrupt and
 * calls the upper-half callback.  The same hardware also backs the arch RTC
 * hooks (up_rtc_*) so the NuttX system time is seeded from it.
 *
 * The RTC is driven through the SDK fwlib RTC API.  Those routines are
 * marked _LONG_CALL_ but are linked from the fwlib RAM source ameba_rtc.c
 * (added to the board build), and none of them take a register base: the
 * fwlib selects the secure/non-secure RTC alias itself, so this driver
 * never touches a base address.  To keep the vendor headers out of the
 * NuttX include world the few fwlib symbols, constants and structures used
 * here are declared locally (layout-compatible mirrors) rather than pulled
 * in from the SDK <ameba_rtc.h>.
 *
 * Calendar model: the Ameba RTC stores a year (1900..2155) plus a
 * day-of-year (0..511) and h:m:s -- it has no month/day-of-month register.
 * NuttX works in struct rtc_time (cast-compatible with struct tm: month +
 * day-of-month).  The two are bridged with the libc calendar routines: to
 * program the RTC we normalise the incoming time and take tm_yday; to read
 * it back we place the day-of-year as "day tm_yday+1 of month 0" and let
 * timegm()/gmtime_r() carry it into the correct month and day.  No timezone
 * is involved (timegm() is UTC), so the conversion is exact and reversible.
 *
 * The only genuinely chip-specific fact -- the RTC interrupt vector -- lives
 * in the per-chip ameba_rtc_chip.h, together with the clock masks and the
 * calendar base year.  The shared driver here is never edited per chip.
 */

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/timers/rtc.h>

#include "ameba_rtc.h"
#include "ameba_rtc_chip.h"

#ifdef CONFIG_RTC_DRIVER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* "state" argument for fwlib enable/disable style APIs. */

#define AMEBA_DISABLE               0x0
#define AMEBA_ENABLE                0x1

/* fwlib RTC constants (from the SDK ameba_rtc.h; identical on every current
 * Ameba chip).  BIN input format, 24-hour clock, AM indicator, and the alarm
 * mask values that mean "match every field" (nothing masked out).
 */

#define AMEBA_RTC_FORMAT_BIN        0x0        /* RTC_Format_BIN          */
#define AMEBA_RTC_HOURFMT_24        0x0        /* RTC_HourFormat_24       */
#define AMEBA_RTC_H12_AM            0x0        /* RTC_H12_AM              */
#define AMEBA_RTC_ALARMMASK_NONE    0x0        /* RTC_AlarmMask_None      */
#define AMEBA_RTC_ALARM2MASK_NONE   0x0        /* RTC_Alarm2Mask_None     */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Layout-compatible mirrors of the fwlib RTC structures (same field order
 * and types); passed by address to the fwlib RTC API.
 */

/* RTC_InitTypeDef */

struct ameba_rtc_init_s
{
  uint32_t hourformat;               /* RTC_HourFormat      */
  uint32_t asynchprediv;             /* RTC_AsynchPrediv    */
  uint32_t synchprediv;              /* RTC_SynchPrediv     */
  uint32_t daythreshold;             /* RTC_DayThreshold    */
};

/* RTC_TimeTypeDef: year + day-of-year, not month/day. */

struct ameba_rtc_time_s
{
  uint16_t year;                     /* 1900..2155          */
  uint16_t days;                     /* day-of-year, 0..511 */
  uint8_t  hours;                    /* 0..23 (24h format)  */
  uint8_t  minutes;                  /* 0..59               */
  uint8_t  seconds;                  /* 0..59               */
  uint8_t  h12_pmam;                 /* RTC_H12_AM/PM       */
};

/* RTC_AlarmTypeDef */

struct ameba_rtc_alarm_s
{
  struct ameba_rtc_time_s time;      /* Alarm match time    */
  uint32_t mask;                     /* RTC_AlarmMask  (HMS) */
  uint32_t mask2;                    /* RTC_Alarm2Mask (Day) */
};

/* This is the private RTC lower-half state.  It must be cast-compatible with
 * struct rtc_lowerhalf_s (the ops pointer is first).
 */

struct ameba_rtc_lowerhalf_s
{
  const struct rtc_ops_s *ops;       /* Lower-half operations vtable */

  mutex_t devlock;                   /* Exclusive access to the alarm */

#ifdef CONFIG_RTC_ALARM
  volatile rtc_alarm_callback_t cb;  /* Callback when the alarm expires */
  volatile void *priv;               /* Argument to accompany the callback */
  bool irqattached;                  /* RTC IRQ has been attached */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SDK fwlib RTC / clock API (linked from fwlib RAM ameba_rtc.c).  None of
 * the RTC calls take a register base -- the fwlib picks the secure/non-
 * secure alias internally.
 */

extern void RCC_PeriphClockCmd(uint32_t periph, uint32_t clock,
                               uint8_t newstate);
extern void RTC_StructInit(struct ameba_rtc_init_s *init);
extern uint32_t RTC_Init(struct ameba_rtc_init_s *init);
extern uint32_t RTC_SetTime(uint32_t format,
                            struct ameba_rtc_time_s *time);
extern void RTC_GetTime(uint32_t format, struct ameba_rtc_time_s *time);
extern uint32_t RTC_SetAlarm(uint32_t format,
                             struct ameba_rtc_alarm_s *alarm);
extern void RTC_GetAlarm(uint32_t format, struct ameba_rtc_alarm_s *alarm);
extern void RTC_AlarmStructInit(struct ameba_rtc_alarm_s *alarm);
extern void RTC_AlarmCmd(uint32_t newstate);
extern void RTC_AlarmClear(void);

/* Internal helpers. */

static void ameba_time_to_hw(const struct rtc_time *rtctime,
                             struct ameba_rtc_time_s *hw);

/* RTC lower-half operations. */

static int  ameba_rdtime(struct rtc_lowerhalf_s *lower,
                         struct rtc_time *rtctime);
static int  ameba_settime(struct rtc_lowerhalf_s *lower,
                          const struct rtc_time *rtctime);
static bool ameba_havesettime(struct rtc_lowerhalf_s *lower);

#ifdef CONFIG_RTC_ALARM
static int  ameba_setalarm(struct rtc_lowerhalf_s *lower,
                           const struct lower_setalarm_s *alarminfo);
static int  ameba_setrelative(struct rtc_lowerhalf_s *lower,
                              const struct lower_setrelative_s *alarminfo);
static int  ameba_cancelalarm(struct rtc_lowerhalf_s *lower, int alarmid);
static int  ameba_rdalarm(struct rtc_lowerhalf_s *lower,
                          struct lower_rdalarm_s *alarminfo);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rtc_ops_s g_rtc_ops =
{
  .rdtime      = ameba_rdtime,
  .settime     = ameba_settime,
  .havesettime = ameba_havesettime,
#ifdef CONFIG_RTC_ALARM
  .setalarm    = ameba_setalarm,
  .setrelative = ameba_setrelative,
  .cancelalarm = ameba_cancelalarm,
  .rdalarm     = ameba_rdalarm,
#endif
};

static struct ameba_rtc_lowerhalf_s g_rtc_lowerhalf =
{
  .ops     = &g_rtc_ops,
  .devlock = NXMUTEX_INITIALIZER,
};

/* Set true once the clock/RTC has been brought up (from either the early
 * arch up_rtc_initialize() or the later ameba_rtc_lowerhalf()), and true by
 * settime() so havesettime() can report it.
 */

static bool g_rtc_started;
static bool g_rtc_timeset;

/* Consumed by the NuttX clock layer: set once the RTC is running so system
 * time updates are written through to the RTC (see sched/clock).
 */

volatile bool g_rtc_enabled = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ameba_rtc_hwstart
 *
 * Description:
 *   Gate the RTC clock and initialise the fwlib RTC (24-hour format) exactly
 *   once.  Called from both the early arch up_rtc_initialize() and the later
 *   ameba_rtc_lowerhalf(); whichever runs first does the work.
 *
 ****************************************************************************/

static void ameba_rtc_hwstart(void)
{
  struct ameba_rtc_init_s init;
  struct ameba_rtc_time_s hw;

  if (g_rtc_started)
    {
      return;
    }

  RCC_PeriphClockCmd(AMEBA_RTC_APBPERIPH, AMEBA_RTC_APBPERIPH_CLK,
                     AMEBA_ENABLE);

  RTC_StructInit(&init);
  init.hourformat = AMEBA_RTC_HOURFMT_24;
  RTC_Init(&init);

  /* On a cold start the RTC year sits at its power-on default (near the
   * calendar base year, 1900), which predates the POSIX epoch and cannot be
   * represented as a time_t -- reads would return a garbage negative time.
   * Seed it with the configured build date so the clock is sane before any
   * application sets the time.  A warm RTC that already holds a later year
   * (kept running across a reset) is left untouched.
   */

  RTC_GetTime(AMEBA_RTC_FORMAT_BIN, &hw);
  if (hw.year < CONFIG_START_YEAR)
    {
      struct rtc_time rtctime;

      memset(&rtctime, 0, sizeof(rtctime));
      rtctime.tm_year = CONFIG_START_YEAR - AMEBA_RTC_BASE_YEAR;
      rtctime.tm_mon  = CONFIG_START_MONTH - 1;
      rtctime.tm_mday = CONFIG_START_DAY;

      ameba_time_to_hw(&rtctime, &hw);
      RTC_SetTime(AMEBA_RTC_FORMAT_BIN, &hw);
    }

  g_rtc_started = true;
}

/****************************************************************************
 * Name: ameba_time_to_hw
 *
 * Description:
 *   Convert a struct rtc_time (month + day-of-month, tm-compatible) into the
 *   fwlib year + day-of-year form.  The incoming time is normalised
 *   through timegm()/gmtime_r() (UTC, no timezone) so tm_yday is exact.
 *
 ****************************************************************************/

static void ameba_time_to_hw(const struct rtc_time *rtctime,
                             struct ameba_rtc_time_s *hw)
{
  struct tm tmp;
  time_t secs;

  /* struct rtc_time is cast-compatible with struct tm. */

  memcpy(&tmp, rtctime, sizeof(struct tm));
  tmp.tm_isdst = 0;

  /* Normalise and recover the day-of-year. */

  secs = timegm(&tmp);
  gmtime_r(&secs, &tmp);

  hw->year     = (uint16_t)(tmp.tm_year + AMEBA_RTC_BASE_YEAR);
  hw->days     = (uint16_t)tmp.tm_yday;
  hw->hours    = (uint8_t)tmp.tm_hour;
  hw->minutes  = (uint8_t)tmp.tm_min;
  hw->seconds  = (uint8_t)tmp.tm_sec;
  hw->h12_pmam = AMEBA_RTC_H12_AM;
}

/****************************************************************************
 * Name: ameba_hw_to_time
 *
 * Description:
 *   Convert the fwlib year + day-of-year representation back into a struct
 *   rtc_time.  The day-of-year is placed as "day (days + 1) of month 0" and
 *   timegm()/gmtime_r() carry it into the correct month, day and (on a
 *   day-of-year overflow) year.
 *
 ****************************************************************************/

static void ameba_hw_to_time(const struct ameba_rtc_time_s *hw,
                             struct rtc_time *rtctime)
{
  struct tm tmp;
  time_t secs;

  memset(&tmp, 0, sizeof(tmp));
  tmp.tm_year  = (int)hw->year - AMEBA_RTC_BASE_YEAR;
  tmp.tm_mon   = 0;
  tmp.tm_mday  = (int)hw->days + 1;
  tmp.tm_hour  = hw->hours;
  tmp.tm_min   = hw->minutes;
  tmp.tm_sec   = hw->seconds;
  tmp.tm_isdst = 0;

  secs = timegm(&tmp);
  gmtime_r(&secs, (struct tm *)rtctime);
}

/****************************************************************************
 * Name: ameba_rtc_interrupt
 *
 * Description:
 *   RTC interrupt handler.  Clears the alarm flag, disables the (one-shot)
 *   alarm and invokes the upper-half callback recorded at setalarm() time.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int ameba_rtc_interrupt(int irq, void *context, void *arg)
{
  struct ameba_rtc_lowerhalf_s *priv =
    (struct ameba_rtc_lowerhalf_s *)arg;
  rtc_alarm_callback_t cb;
  void *cbarg;

  /* Acknowledge and disable the alarm (this driver's alarm is one-shot). */

  RTC_AlarmClear();
  RTC_AlarmCmd(AMEBA_DISABLE);

  /* Sample and clear the callback so a re-arm from within it is clean. */

  cb    = (rtc_alarm_callback_t)priv->cb;
  cbarg = (void *)priv->priv;

  priv->cb   = NULL;
  priv->priv = NULL;

  if (cb != NULL)
    {
      cb(cbarg, 0);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: ameba_rdtime
 *
 * Description:
 *   Implement the rdtime() method: return the current RTC date/time.
 *
 ****************************************************************************/

static int ameba_rdtime(struct rtc_lowerhalf_s *lower,
                         struct rtc_time *rtctime)
{
  struct ameba_rtc_time_s hw;

  RTC_GetTime(AMEBA_RTC_FORMAT_BIN, &hw);
  ameba_hw_to_time(&hw, rtctime);
  return OK;
}

/****************************************************************************
 * Name: ameba_settime
 *
 * Description:
 *   Implement the settime() method: set the RTC date/time.
 *
 ****************************************************************************/

static int ameba_settime(struct rtc_lowerhalf_s *lower,
                          const struct rtc_time *rtctime)
{
  struct ameba_rtc_time_s hw;

  ameba_time_to_hw(rtctime, &hw);

  if (RTC_SetTime(AMEBA_RTC_FORMAT_BIN, &hw) != AMEBA_ENABLE)
    {
      return -EIO;
    }

  g_rtc_timeset = true;
  return OK;
}

/****************************************************************************
 * Name: ameba_havesettime
 *
 * Description:
 *   Implement the havesettime() method: has the RTC time ever been set?
 *
 ****************************************************************************/

static bool ameba_havesettime(struct rtc_lowerhalf_s *lower)
{
  return g_rtc_timeset;
}

/****************************************************************************
 * Name: ameba_setalarm
 *
 * Description:
 *   Implement the setalarm() method: program a one-shot alarm at an absolute
 *   time and arm the RTC interrupt.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int ameba_setalarm(struct rtc_lowerhalf_s *lower,
                          const struct lower_setalarm_s *alarminfo)
{
  struct ameba_rtc_lowerhalf_s *priv =
    (struct ameba_rtc_lowerhalf_s *)lower;
  struct ameba_rtc_alarm_s alarm;
  struct ameba_rtc_time_s hw;
  int ret;

  DEBUGASSERT(priv != NULL && alarminfo != NULL);

  /* This RTC has a single alarm. */

  if (alarminfo->id != 0)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  /* Attach the RTC interrupt on first use. */

  if (!priv->irqattached)
    {
      ret = irq_attach(AMEBA_RTC_IRQ, ameba_rtc_interrupt, priv);
      if (ret < 0)
        {
          nxmutex_unlock(&priv->devlock);
          return ret;
        }

      up_enable_irq(AMEBA_RTC_IRQ);
      priv->irqattached = true;
    }

  /* Remember the callback, then program the alarm.  The alarm hardware
   * matches day-of-year + h:m:s, so nothing is masked out.
   */

  priv->cb   = alarminfo->cb;
  priv->priv = alarminfo->priv;

  ameba_time_to_hw(&alarminfo->time, &hw);

  RTC_AlarmStructInit(&alarm);
  alarm.time  = hw;
  alarm.time.h12_pmam = AMEBA_RTC_H12_AM;
  alarm.mask  = AMEBA_RTC_ALARMMASK_NONE;
  alarm.mask2 = AMEBA_RTC_ALARM2MASK_NONE;

  if (RTC_SetAlarm(AMEBA_RTC_FORMAT_BIN, &alarm) != AMEBA_ENABLE)
    {
      priv->cb   = NULL;
      priv->priv = NULL;
      nxmutex_unlock(&priv->devlock);
      return -EIO;
    }

  RTC_AlarmCmd(AMEBA_ENABLE);

  nxmutex_unlock(&priv->devlock);
  return OK;
}
#endif

/****************************************************************************
 * Name: ameba_setrelative
 *
 * Description:
 *   Implement the setrelative() method: program a one-shot alarm relative to
 *   the current time by folding it onto setalarm().
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int ameba_setrelative(struct rtc_lowerhalf_s *lower,
                             const struct lower_setrelative_s *alarminfo)
{
  struct lower_setalarm_s setalarm;
  struct ameba_rtc_time_s hw;
  struct tm tmp;
  time_t secs;

  DEBUGASSERT(alarminfo != NULL);

  if (alarminfo->id != 0 || alarminfo->reltime == 0)
    {
      return -EINVAL;
    }

  /* Current time + reltime, via the calendar routines. */

  RTC_GetTime(AMEBA_RTC_FORMAT_BIN, &hw);
  ameba_hw_to_time(&hw, (struct rtc_time *)&tmp);
  secs = timegm(&tmp) + (time_t)alarminfo->reltime;
  gmtime_r(&secs, (struct tm *)&setalarm.time);

  setalarm.id   = alarminfo->id;
  setalarm.cb   = alarminfo->cb;
  setalarm.priv = alarminfo->priv;

  return ameba_setalarm(lower, &setalarm);
}
#endif

/****************************************************************************
 * Name: ameba_cancelalarm
 *
 * Description:
 *   Implement the cancelalarm() method: disable the alarm and forget the
 *   callback.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int ameba_cancelalarm(struct rtc_lowerhalf_s *lower, int alarmid)
{
  struct ameba_rtc_lowerhalf_s *priv =
    (struct ameba_rtc_lowerhalf_s *)lower;
  int ret;

  if (alarmid != 0)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  RTC_AlarmCmd(AMEBA_DISABLE);
  RTC_AlarmClear();

  priv->cb   = NULL;
  priv->priv = NULL;

  nxmutex_unlock(&priv->devlock);
  return OK;
}
#endif

/****************************************************************************
 * Name: ameba_rdalarm
 *
 * Description:
 *   Implement the rdalarm() method: return the programmed alarm time.  The
 *   hardware alarm stores only day-of-year + h:m:s, so the year is borrowed
 *   from the current RTC time to build a full calendar time.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int ameba_rdalarm(struct rtc_lowerhalf_s *lower,
                         struct lower_rdalarm_s *alarminfo)
{
  struct ameba_rtc_alarm_s alarm;
  struct ameba_rtc_time_s now;

  DEBUGASSERT(alarminfo != NULL && alarminfo->time != NULL);

  if (alarminfo->id != 0)
    {
      return -EINVAL;
    }

  RTC_GetAlarm(AMEBA_RTC_FORMAT_BIN, &alarm);

  /* The alarm carries no year; borrow it from the current RTC time. */

  RTC_GetTime(AMEBA_RTC_FORMAT_BIN, &now);
  alarm.time.year = now.year;

  ameba_hw_to_time(&alarm.time, alarminfo->time);
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ameba_rtc_lowerhalf
 ****************************************************************************/

struct rtc_lowerhalf_s *ameba_rtc_lowerhalf(void)
{
  /* Bring the RTC up exactly once (arch up_rtc_initialize() may have done it
   * already during the early clock init).
   */

  ameba_rtc_hwstart();

  return (struct rtc_lowerhalf_s *)&g_rtc_lowerhalf;
}

/****************************************************************************
 * Name: up_rtc_initialize
 *
 * Description:
 *   Arch RTC hook: bring the RTC up during the early clock initialisation so
 *   the NuttX system time can be seeded from it.  The /dev/rtc0 lower half
 *   is layered on top of this same hardware.
 *
 ****************************************************************************/

int up_rtc_initialize(void)
{
  ameba_rtc_hwstart();
  g_rtc_enabled = true;
  return OK;
}

/****************************************************************************
 * Name: up_rtc_getdatetime
 *
 * Description:
 *   Arch RTC hook (date/time RTC): return the current RTC time as a broken-
 *   out struct tm.  Used by the clock layer to seed the system time.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_DATETIME
int up_rtc_getdatetime(struct tm *tp)
{
  struct ameba_rtc_time_s hw;

  RTC_GetTime(AMEBA_RTC_FORMAT_BIN, &hw);
  ameba_hw_to_time(&hw, (struct rtc_time *)tp);
  return OK;
}
#endif

/****************************************************************************
 * Name: up_rtc_settime
 *
 * Description:
 *   Arch RTC hook: set the RTC from a timespec (one-second resolution).
 *
 ****************************************************************************/

int up_rtc_settime(const struct timespec *tp)
{
  struct ameba_rtc_time_s hw;
  struct tm tmp;
  time_t secs = tp->tv_sec;

  gmtime_r(&secs, &tmp);
  ameba_time_to_hw((struct rtc_time *)&tmp, &hw);

  if (RTC_SetTime(AMEBA_RTC_FORMAT_BIN, &hw) != AMEBA_ENABLE)
    {
      return -EIO;
    }

  g_rtc_timeset = true;
  return OK;
}

/****************************************************************************
 * Name: ameba_rtc_initialize
 ****************************************************************************/

int ameba_rtc_initialize(void)
{
  struct rtc_lowerhalf_s *lower;

  lower = ameba_rtc_lowerhalf();
  if (lower == NULL)
    {
      return -ENODEV;
    }

  return rtc_initialize(0, lower);
}

#endif /* CONFIG_RTC_DRIVER */
