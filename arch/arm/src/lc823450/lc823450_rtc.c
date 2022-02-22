/****************************************************************************
 * arch/arm/src/lc823450/lc823450_rtc.c
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

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/timers/rtc.h>

#ifdef CONFIG_RTC_ALARM
#  include <nuttx/alarm.h>
#endif

#include <nuttx/power/pm.h>

#include <time.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include "lc823450_syscontrol.h"
#include "arm_arch.h"
#include "clock/clock.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RTC_REGBASE         0x4008e000
#define RTC_SEC             (RTC_REGBASE + 0x000)
#define RTC_MIN             (RTC_REGBASE + 0x004)
#define RTC_HOUR            (RTC_REGBASE + 0x008)
#define RTC_WEEK            (RTC_REGBASE + 0x00c)
#define RTC_DAY             (RTC_REGBASE + 0x010)
#define RTC_MONTH           (RTC_REGBASE + 0x014)
#define RTC_YEAR            (RTC_REGBASE + 0x018)
#define RTC_CENT            (RTC_REGBASE + 0x01c)
#define RTC_SECALM          (RTC_REGBASE + 0x020)
#define RTC_MINALM          (RTC_REGBASE + 0x024)
#define RTC_HOURALM         (RTC_REGBASE + 0x028)
#define RTC_WEEKALM         (RTC_REGBASE + 0x02c)
#define RTC_DAYALM          (RTC_REGBASE + 0x030)
#define RTC_MONTHALM        (RTC_REGBASE + 0x034)
#define RTC_PRSEL           (RTC_REGBASE + 0x038)
#define RTC_RTCINT          (RTC_REGBASE + 0x03c)
#define RTC_RTCINT_AIE      5
#define RTC_RTCINT_SET      7
#define RTC_DST             (RTC_REGBASE + 0x040)
#define RTC_RTCSTAT         (RTC_REGBASE + 0x044)
#define RTC_RTCSTAT_UIP     0
#define RTC_RTCSTAT_UF      4
#define RTC_RTCSTAT_PCLR    7
#define RTC_TEST            (RTC_REGBASE + 0x048)
#define RTC_PTN32BIT0       (RTC_REGBASE + 0x04c)
#define RTC_PTN32BIT1       (RTC_REGBASE + 0x050)
#define RTC_PTN32BIT2       (RTC_REGBASE + 0x054)
#define RTC_PTN32BIT3       (RTC_REGBASE + 0x058)
#define RTC_PTN32BIT0_VAL   0x12
#define RTC_PTN32BIT1_VAL   0x34
#define RTC_PTN32BIT2_VAL   0x56
#define RTC_PTN32BIT3_VAL   0x78
#define RTC_VPTN32BIT0      (RTC_REGBASE + 0x05c)
#define RTC_VPTN32BIT1      (RTC_REGBASE + 0x060)
#define RTC_VPTN32BIT2      (RTC_REGBASE + 0x064)
#define RTC_VPTN32BIT3      (RTC_REGBASE + 0x068)
#define RTC_VPTN32BIT0_VAL  0xa7
#define RTC_VPTN32BIT1_VAL  0x58
#define RTC_VPTN32BIT2_VAL  0x3c
#define RTC_VPTN32BIT3_VAL  0xf1
#define RTC_VDET           (RTC_REGBASE + 0x06c)
#define RTC_VDET_VDET       0x01
#define RTC_RTCINTCNT       (RTC_REGBASE + 0x070)

#ifndef timespec_sub
#define timespec_sub(a, b, c)   /* c = a - b */ \
  do \
    {\
      (c)->tv_nsec = (a)->tv_nsec - (b)->tv_nsec; \
      (c)->tv_sec = (a)->tv_sec - (b)->tv_sec; \
      if ((c)->tv_nsec < 0) \
        {\
          (c)->tv_nsec += (1000 * 1000 * 1000); \
          (c)->tv_sec--; \
        }\
    }\
  while (0)
#endif /* timespec_sub */

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_RTC_SAVE_DEFAULT
struct rtc_default
{
  int sig;
#define RTC_DEFAULT_SIGNATURE 0x12345678
  struct tm tm;
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_RTC_SAVE_DEFAULT
static void rtc_pmnotify(struct pm_callback_s *cb, enum pm_state_e pmstate);
#endif /* CONFIG_RTC_SAVE_DEFAULT */

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static alarmcb_t g_alarmcb;
#endif

#ifdef CONFIG_RTC_SAVE_DEFAULT
static struct pm_callback_s pm_cb =
{
  .notify = rtc_pmnotify,
};
#endif /* CONFIG_RTC_SAVE_DEFAULT */

#ifdef CONFIG_RTC_DIV
static int cboot = 1;
#endif /* CONFIG_RTC_DIV */

static struct timespec lastupdate_mono;
static struct timespec lastupdate_rtc;

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile bool g_rtc_enabled = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tm_divider
 ****************************************************************************/

#ifdef CONFIG_RTC_DIV
static void tm_divider(struct tm *tm, int divn, int divm)
{
  time_t tt;
  tt = timegm(tm);
  tt = (time_t) ((uint64_t)tt * divn / divm);
  gmtime_r(&tt, tm);
}
#endif /* CONFIG_RTC_DIV */

/****************************************************************************
 * Name: rtc_pmnotify
 ****************************************************************************/

#ifdef CONFIG_RTC_SAVE_DEFAULT
static void rtc_pmnotify(struct pm_callback_s *cb, enum pm_state_e pmstate)
{
  struct tm tm;

  switch (pmstate)
    {
      case PM_SLEEP:
      case PM_SHUTDOWN:

        /* RTC is not set yet. */

        if (getreg8(RTC_PTN32BIT0) != RTC_PTN32BIT0_VAL ||
            getreg8(RTC_PTN32BIT1) != RTC_PTN32BIT1_VAL ||
            getreg8(RTC_PTN32BIT2) != RTC_PTN32BIT2_VAL ||
            getreg8(RTC_PTN32BIT3) != RTC_PTN32BIT3_VAL)
          {
            break;
          }

        up_rtc_getdatetime(&tm);
        up_rtc_set_default_datetime(&tm);
        break;

    default:
      break;
    }

  return;
}
#endif /* CONFIG_RTC_SAVE_DEFAULT */

/****************************************************************************
 * Name: rtc_interrupt
 *
 * Description:
 *   RTC interrupt service routine
 *
 * Input Parameters:
 *   irq - The IRQ number that generated the interrupt
 *   context - Architecture specific register save information.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int rtc_interrupt(int irq, void *context, FAR void *arg)
{
  struct tm tm;
  up_rtc_getdatetime(&tm);

  rtcinfo("RTCSTAT = 0x%02x (%04d/%02d/%02d %02d:%02d:%02d)\n",
          getreg8(RTC_RTCSTAT),
          tm.tm_year + 1900,
          tm.tm_mon + 1,
          tm.tm_mday,
          tm.tm_hour,
          tm.tm_min,
          tm.tm_sec);

  /* Disable IRQ */

  putreg8(0, RTC_RTCINT);

  /* Clear interrupt status */

  putreg8(0, RTC_RTCSTAT);
  putreg8(1 << RTC_RTCSTAT_PCLR, RTC_RTCSTAT);

  if (g_alarmcb)
    {
      g_alarmcb();
    }

  return OK;
}
#endif /* CONFIG_RTC_ALARM */

/****************************************************************************
 * Name: up_rtc_getdatetime
 *
 * Description:
 *   Get the current date and time from the date/time RTC.  This interface
 *   is only supported by the date/time RTC hardware implementation.
 *   It is used to replace the system timer.
 *   It is only used by the RTOS during initialization to
 *   set up the system time when CONFIG_RTC and CONFIG_RTC_DATETIME
 *   are selected (and CONFIG_RTC_HIRES is not).
 *
 *   NOTE: Some date/time RTC hardware is capability of sub-second accuracy.
 *   That sub-second accuracy is lost in this interface.
 *   However, since the system time is reinitialized on each power-up/reset,
 *   there will be no timing inaccuracy in the long run.
 *
 * Input Parameters:
 *   tp - The location to return the high resolution time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

static int up_rtc_getdatetime_main(FAR struct tm *tp)
{
#ifdef CONFIG_RTC_DIV
  tp->tm_sec  = getreg8(RTC_SEC);
  tp->tm_min  = getreg8(RTC_MIN);
  tp->tm_hour = getreg8(RTC_HOUR);
  tp->tm_mday = getreg8(RTC_DAY);

  /* this RTC count month from 1 */

  tp->tm_mon  = getreg8(RTC_MONTH) - 1;

  /* int tm_year: years since 1900  */

  tp->tm_year = getreg8(RTC_YEAR) + (getreg8(RTC_CENT) - 19) * 100;
  tm_divider(tp, CONFIG_RTC_DIV_N, CONFIG_RTC_DIV_M);
#else /* CONFIG_RTC_DIV */
  if (getreg8(RTC_PTN32BIT0) != RTC_PTN32BIT0_VAL ||
      getreg8(RTC_PTN32BIT1) != RTC_PTN32BIT1_VAL ||
      getreg8(RTC_PTN32BIT2) != RTC_PTN32BIT2_VAL ||
      getreg8(RTC_PTN32BIT3) != RTC_PTN32BIT3_VAL)
    {
      tp->tm_sec  = 0;
      tp->tm_min  = 0;
      tp->tm_hour = 0;
      tp->tm_mday = CONFIG_START_DAY;           /* 1 to 31 */
      tp->tm_mon  = CONFIG_START_MONTH - 1;     /* 0 to 11 */
      tp->tm_year = (CONFIG_START_YEAR - 1900); /* since 1900 */
    }
  else
    {
      tp->tm_sec  = getreg8(RTC_SEC);
      tp->tm_min  = getreg8(RTC_MIN);
      tp->tm_hour = getreg8(RTC_HOUR);
      tp->tm_mday = getreg8(RTC_DAY);
      tp->tm_mon  = getreg8(RTC_MONTH) - 1; /* this RTC count month from 1 */

      /* int tm_year: years since 1900  */

      tp->tm_year = getreg8(RTC_YEAR) + (getreg8(RTC_CENT) - 19) * 100;
    }
#endif /* CONFIG_RTC_DIV */

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_rtcinitialize
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
  /* RTC clock / reset */

  modifyreg32(MCLKCNTAPB, 0, MCLKCNTAPB_RTC_CLKEN);
  modifyreg32(MRSTCNTAPB, 0, MRSTCNTAPB_RTC_RSTB);

  putreg8(0xa7, RTC_VPTN32BIT0);
  putreg8(0x58, RTC_VPTN32BIT1);
  putreg8(0x3c, RTC_VPTN32BIT2);
  putreg8(0xf1, RTC_VPTN32BIT3);

#ifdef CONFIG_RTC_DIV
  struct tm tm;

  /* Stop rtc update */

  putreg8(1 << RTC_RTCINT_SET, RTC_RTCINT);

  tm.tm_sec  = 0;
  tm.tm_min  = 0;
  tm.tm_hour = 0;
  tm.tm_mday = CONFIG_START_DAY;
  tm.tm_mon  = CONFIG_START_MONTH - 1;
  tm.tm_year = CONFIG_START_YEAR - 1900;

  tm_divider(&tm, CONFIG_RTC_DIV_M, CONFIG_RTC_DIV_N);

  putreg8(tm.tm_sec, RTC_SEC);
  putreg8(tm.tm_min, RTC_MIN);
  putreg8(tm.tm_hour, RTC_HOUR);
  putreg8(tm.tm_mday, RTC_DAY);
  putreg8(tm.tm_mon + 1, RTC_MONTH);
  putreg8((tm.tm_year + 1900) / 100, RTC_CENT);
  putreg8(tm.tm_year % 100, RTC_YEAR);

#else /* CONFIG_RTC_DIV */

  if (!(getreg8(RTC_VDET) & RTC_VDET_VDET))
    {
      rtcinfo("VDET Detect\n");
      putreg8(0, RTC_PTN32BIT0);
      putreg8(0, RTC_PTN32BIT1);
      putreg8(0, RTC_PTN32BIT2);
      putreg8(0, RTC_PTN32BIT3);

      /* VDET decahtter */

      putreg8(RTC_VPTN32BIT0_VAL, RTC_VPTN32BIT0);
      putreg8(RTC_VPTN32BIT1_VAL, RTC_VPTN32BIT1);
      putreg8(RTC_VPTN32BIT2_VAL, RTC_VPTN32BIT2);
      putreg8(RTC_VPTN32BIT3_VAL, RTC_VPTN32BIT3);

      /* Reset VDET */

      putreg32(RTC_VDET_VDET, RTC_VDET);

      /* 32.768kHz * 2 clock */

      up_udelay(62);
      putreg32(0, RTC_VDET);
    }

  if (getreg8(RTC_PTN32BIT0) != RTC_PTN32BIT0_VAL ||
      getreg8(RTC_PTN32BIT1) != RTC_PTN32BIT1_VAL ||
      getreg8(RTC_PTN32BIT2) != RTC_PTN32BIT2_VAL ||
      getreg8(RTC_PTN32BIT3) != RTC_PTN32BIT3_VAL)
    {
      /* DETECT RTC reset. Set temporary timevalue for RTC alarm */

      /* Stop rtc update */

      putreg8(1 << RTC_RTCINT_SET, RTC_RTCINT);

      putreg8(0, RTC_SEC);
      putreg8(0, RTC_MIN);
      putreg8(0, RTC_HOUR);
      putreg8(CONFIG_START_DAY, RTC_DAY);
      putreg8(CONFIG_START_MONTH, RTC_MONTH);
      putreg8(CONFIG_START_YEAR % 100, RTC_YEAR);
      putreg8(CONFIG_START_YEAR / 100, RTC_CENT);
    }
#endif /* CONFIG_RTC_DIV */

  /* RTC start, if not run */

  putreg8(0, RTC_RTCINT);

  putreg8(0, RTC_RTCSTAT);
  putreg8(1 << RTC_RTCSTAT_PCLR, RTC_RTCSTAT);

#ifdef CONFIG_RTC_ALARM
  irq_attach(LC823450_IRQ_RTC, rtc_interrupt, NULL);
  up_enable_irq(LC823450_IRQ_RTC);
#endif

#ifdef CONFIG_RTC_SAVE_DEFAULT
  pm_register(&pm_cb);
#endif /* CONFIG_RTC_SAVE_DEFAULT */

  g_rtc_enabled = true;

  return OK;
}

int up_rtc_getdatetime(FAR struct tm *tp)
{
  struct tm tm1;
  struct tm tm2;

#ifdef CONFIG_RTC_DIV
  /* WA: time registers cannot be read within one second(from set) */

  if (cboot)
    {
      tp->tm_sec  = 0;
      tp->tm_min  = 0;
      tp->tm_hour = 0;
      tp->tm_mday = CONFIG_START_DAY;
      tp->tm_mon  = CONFIG_START_MONTH - 1;
      tp->tm_year = CONFIG_START_YEAR - 1900;
      cboot = 0;
      return OK;
    }
#endif /* CONFIG_RTC_DIV */

  up_rtc_getdatetime_main(&tm1);
  up_rtc_getdatetime_main(&tm2);

  if (tm1.tm_sec == tm2.tm_sec)
    {
      /* if tm1 == tm2, could read registers atomically  */

      *tp = tm1;
    }
  else
    {
      /* if tm1 != tm2, can read registers atomically in 1 sec */

      up_rtc_getdatetime_main(tp);
    }

  return OK;
}

/****************************************************************************
 * Name: up_rtc_settime
 *
 * Description:
 *   Set the RTC to the provided time.
 *   All RTC implementations must be able to set their time
 *   based on a standard timespec.
 *
 * Input Parameters:
 *   tp - the time to use
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_settime(FAR const struct timespec *ts)
{
  struct tm *tp;

#ifdef CONFIG_ALARM_DEV
  alarm_before_rtc_change();
#endif /* CONFIG_ALARM_DEV */

  tp = gmtime(&ts->tv_sec);

#ifdef CONFIG_RTC_DIV
  tm_divider(tp, CONFIG_RTC_DIV_M, CONFIG_RTC_DIV_N);
#endif /* CONFIG_RTC_DIV */

  /* Stop rtc update */

  putreg8(1 << RTC_RTCINT_SET, RTC_RTCINT);

  putreg8(tp->tm_sec, RTC_SEC);
  putreg8(tp->tm_min, RTC_MIN);
  putreg8(tp->tm_hour, RTC_HOUR);
  putreg8(tp->tm_mday, RTC_DAY);
  putreg8(tp->tm_mon + 1, RTC_MONTH); /* this RTC count month from 1 */

  /* int tm_year;     years since 1900  */

  putreg8((tp->tm_year + 1900) / 100, RTC_CENT);
  putreg8(tp->tm_year % 100, RTC_YEAR);

  /* mark RTC has valid timevalue */

  putreg8(RTC_PTN32BIT0_VAL, RTC_PTN32BIT0);
  putreg8(RTC_PTN32BIT1_VAL, RTC_PTN32BIT1);
  putreg8(RTC_PTN32BIT2_VAL, RTC_PTN32BIT2);
  putreg8(RTC_PTN32BIT3_VAL, RTC_PTN32BIT3);

#ifdef CONFIG_RTC_SAVE_DEFAULT
  up_rtc_set_default_datetime(tp);
#endif /* CONFIG_RTC_SAVE_DEFAULT */

  clock_systime_timespec(&lastupdate_mono);
  lastupdate_rtc = *ts;

  /* Start rtc update */

  putreg8(0, RTC_RTCINT);

#ifdef CONFIG_ALARM_DEV
  alarm_after_rtc_change();
#endif /* CONFIG_ALARM_DEV */

  return OK;
}

/****************************************************************************
 * Name: up_rtc_setalarm
 *
 * Description:
 *   Set up an alarm.
 *   NOTE: Only one alarm is supported.
 *
 * Input Parameters:
 *   tp - the time to set the alarm
 *   callback - the function to call when the alarm expires.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
int up_rtc_setalarm(FAR const struct timespec *ts, alarmcb_t callback)
{
  struct tm *tp;

  if (g_alarmcb)
    {
      return -EBUSY;
    }

  tp = gmtime(&ts->tv_sec);
#ifdef CONFIG_RTC_DIV
  tm_divider(tp, CONFIG_RTC_DIV_M, CONFIG_RTC_DIV_N);
#endif /* CONFIG_RTC_DIV */
  g_alarmcb = callback;
#if 0
  llinfo("SETALARM (%04d/%02d/%02d %02d:%02d:%02d)\n",
    tp->tm_year + 1900,
    tp->tm_mon + 1,
    tp->tm_mday,
    tp->tm_hour,
    tp->tm_min,
    tp->tm_sec);
#endif

  /* Stop rtc update */

  putreg8(1 << RTC_RTCINT_SET, RTC_RTCINT);

  putreg8(tp->tm_sec, RTC_SECALM);
  putreg8(tp->tm_min, RTC_MINALM);
  putreg8(tp->tm_hour, RTC_HOURALM);
  putreg8(0, RTC_WEEKALM); /* 0 == not use */
  putreg8(tp->tm_mday, RTC_DAYALM);
  putreg8(tp->tm_mon + 1, RTC_MONTHALM); /* this RTC count month from 1 */

  /* tp->tm_year : this rtc don't support. */

  putreg8(0, RTC_RTCSTAT);
  putreg8(1 << RTC_RTCSTAT_PCLR, RTC_RTCSTAT);

  /* Start rtc update & interrupt enable */

  modifyreg8(RTC_RTCINT, 1 << RTC_RTCINT_SET, 1 << RTC_RTCINT_AIE);

  return OK;
}

/****************************************************************************
 * Name: up_rtc_cancelalarm
 ****************************************************************************/

int up_rtc_cancelalarm(void)
{
  irqstate_t   flags;
  flags = enter_critical_section();
  g_alarmcb = NULL;

  /* Disable IRQ */

  putreg8(0, RTC_RTCINT);

  leave_critical_section(flags);
  return 0;
}

#endif

/****************************************************************************
 * Name: up_rtc_getrawtime
 *
 * Description:
 *  This function is used to get timespec of raw hardware value.
 *
 ****************************************************************************/

int up_rtc_getrawtime(FAR struct timespec *ts)
{
  struct tm tm;
  struct timespec now;
  struct timespec diff;

  clock_systime_timespec(&now);
  timespec_sub(&now, &lastupdate_mono, &diff);

  if (lastupdate_rtc.tv_sec != 0 && diff.tv_sec < 1)
    {
      /* Can not read RTC value until the end of first count (<1s)  */

      *ts = lastupdate_rtc;
      return 0;
    }

  tm.tm_sec  = getreg8(RTC_SEC);
  tm.tm_min  = getreg8(RTC_MIN);
  tm.tm_hour = getreg8(RTC_HOUR);
  tm.tm_mday = getreg8(RTC_DAY);
  tm.tm_mon  = getreg8(RTC_MONTH) - 1; /* this RTC count month from 1 */
  tm.tm_year = getreg8(RTC_YEAR) + (getreg8(RTC_CENT) - 19) * 100;

#ifdef CONFIG_RTC_DIV
  tm_divider(&tm, CONFIG_RTC_DIV_N, CONFIG_RTC_DIV_M);
#endif /* CONFIG_RTC_DIV */

  ts->tv_nsec = 0;
  ts->tv_sec = timegm(&tm);
  return 0;
}

#ifdef CONFIG_RTC_SAVE_DEFAULT

/****************************************************************************
 * Name: up_rtc_set_default_datetime
 ****************************************************************************/

void up_rtc_set_default_datetime(struct tm *tp)
{
  int ret;
  void *handle;
  struct rtc_default rtc_def;

  rtc_def.tm = *tp;
  rtc_def.sig = RTC_DEFAULT_SIGNATURE;

  ret = bchlib_setup(CONFIG_RTC_SAVE_BLOCKNAME, true, &handle);
  if (ret)
    {
      llerr("error: %d\n");
      return;
    }

  bchlib_write(handle, (void *)&rtc_def,
               CONFIG_RTC_SAVE_SECTOR_OFFSET * 512, sizeof(rtc_def));
  bchlib_teardown(handle);
}

/****************************************************************************
 * Name: up_rtc_get_default_datetime
 ****************************************************************************/

int up_rtc_get_default_datetime(struct tm *tp)
{
  void *handle;
  struct rtc_default rtc_def;
  int ret;

  ret = bchlib_setup(CONFIG_RTC_SAVE_BLOCKNAME, true, &handle);
  if (ret)
    {
      llerr("error: %d\n");
      return -1;
    }

  bchlib_read(handle, (void *)&rtc_def,
              CONFIG_RTC_SAVE_SECTOR_OFFSET * 512, sizeof(rtc_def));
  bchlib_teardown(handle);
  if (rtc_def.sig != RTC_DEFAULT_SIGNATURE)
    {
      return -ENOENT;
    }

  *tp = rtc_def.tm;
  return 0;
}

/****************************************************************************
 * Name: up_rtc_clear_default
 ****************************************************************************/

void up_rtc_clear_default(void)
{
  int ret;
  struct rtc_default rtc_def;
  void *handle;

  ret = bchlib_setup(CONFIG_RTC_SAVE_BLOCKNAME, true, &handle);
  if (ret)
    {
      llerr("error: %d\n");
      return;
    }

  memset(&rtc_def, 0, sizeof(rtc_def));
  bchlib_write(handle, (void *)&rtc_def,
               CONFIG_RTC_SAVE_SECTOR_OFFSET * 512, sizeof(rtc_def));
  bchlib_teardown(handle);
  return;
}

#endif /* CONFIG_RTC_SAVE_DEFAULT */
