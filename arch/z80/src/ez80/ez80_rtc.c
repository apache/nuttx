/****************************************************************************
 * arch/z80/src/ez80/ez80_rtc.c
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

#include <stdint.h>
#include <stdbool.h>
#include <sched.h>
#include <time.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/timers/rtc.h>

#include <arch/io.h>

#include "chip.h"
#include "ez80_rtc.h"

#include <arch/board/board.h>

#ifdef CONFIG_EZ80_RTC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_RTC_DATETIME
#  error "CONFIG_RTC_DATETIME must be set to use this driver"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Raw time from the RTC.  All are binary. */

struct rtc_timeregs_s
{
  uint8_t sec;   /* Seconds      Range 0-59 */
  uint8_t min;   /* Minutes      Range 0-59 */
  uint8_t hrs;   /* Hours        Range 0-23 */
  uint8_t dow;   /* Day of week  Range 1-7 */
  uint8_t dom;   /* Day of month Range 1-31 */
  uint8_t mon;   /* Month        Range 1-12 */
  uint8_t yr;    /* Year         Range 0-99 */
  uint8_t cen;   /* Century      Range 0-99 */
};

/* Alarm time from the RTC.  All are BCD encoded. */

struct rtc_almregs_s
{
  uint8_t sec;   /* Seconds */
  uint8_t min;   /* Minutes */
  uint8_t hrs;   /* Hours */
  uint8_t dow;   /* Day of week */
};

#ifdef CONFIG_RTC_ALARM
struct alm_cbinfo_s
{
  volatile alm_callback_t ac_cb; /* Client callback function */
  volatile FAR void *ac_arg;     /* Argument to pass with the callback function */
};
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
/* Callback to use when an EXTI is activated  */

static struct alm_cbinfo_s g_alarmcb;
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_rtc_enabled is set true after the RTC has successfully initialized */

volatile bool g_rtc_enabled = false;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Debug */

#ifdef CONFIG_DEBUG_RTC_INFO
static void rtc_dumpregs(FAR const char *msg);
static void rtc_dumptime(FAR const struct tm *tp, FAR const char *msg);
#endif

/* Register access */

static void rtc_unlock(void);
static void rtc_lock(void);

static void get_raw_time(FAR struct rtc_timeregs_s *rtcregs);
static void set_raw_time(FAR const struct rtc_timeregs_s *rtcregs);

#ifdef CONFIG_RTC_ALARM
static void get_raw_alarm(FAR struct rtc_almregs_s *almregs);
static void set_raw_alarm(FAR const struct rtc_almregs_s *almregs);
static int ez80_alarm_interrupt(int irq, FAR void *context, FAR void *arg);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rtc_dumpregs
 ****************************************************************************/

#ifdef CONFIG_DEBUG_RTC_INFO
static void rtc_dumpregs(FAR const char *msg)
{
  int rtc_state;

  rtcinfo("%s:\n", msg);
  rtcinfo("     SEC: %02x\n", inp(EZ80_RTC_SEC));
  rtcinfo("     MIN: %02x\n", inp(EZ80_RTC_MIN));
  rtcinfo("     HRS: %02x\n", inp(EZ80_RTC_HRS));
  rtcinfo("     DOW: %02x\n", inp(EZ80_RTC_DOW));
  rtcinfo("     DOM: %02x\n", inp(EZ80_RTC_DOM));
  rtcinfo("     MON: %02x\n", inp(EZ80_RTC_MON));
  rtcinfo("      YR: %02x\n", inp(EZ80_RTC_YR));
  rtcinfo("     CEN: %02x\n", inp(EZ80_RTC_CEN));
  rtcinfo("    ASEC: %02x\n", inp(EZ80_RTC_ASEC));
  rtcinfo("    AMIN: %02x\n", inp(EZ80_RTC_AMIN));
  rtcinfo("    AHRS: %02x\n", inp(EZ80_RTC_AHRS));
  rtcinfo("    ADOW: %02x\n", inp(EZ80_RTC_ADOW));
  rtcinfo("   ACTRL: %02x\n", inp(EZ80_RTC_ACTRL));
  rtcinfo("    CTRL: %02x\n", inp(EZ80_RTC_CTRL));
}
#else
#  define rtc_dumpregs(msg)
#endif

/****************************************************************************
 * Name: rtc_dumptime
 ****************************************************************************/

#ifdef CONFIG_DEBUG_RTC_INFO
static void rtc_dumptime(FAR const struct tm *tp, FAR const char *msg)
{
  rtcinfo("%s:\n", msg);
  rtcinfo("  tm_sec: %08x\n", tp->tm_sec);
  rtcinfo("  tm_min: %08x\n", tp->tm_min);
  rtcinfo(" tm_hour: %08x\n", tp->tm_hour);
  rtcinfo(" tm_wday: %08x\n", tp->tm_wday);
  rtcinfo(" tm_mday: %08x\n", tp->tm_mday);
  rtcinfo("  tm_mon: %08x\n", tp->tm_mon);
  rtcinfo(" tm_year: %08x\n", tp->tm_year);
}
#else
#  define rtc_dumptime(tp, msg)
#endif

/****************************************************************************
 * Name: rtc_unlock
 *
 * Description:
 *    Disable RTC write protection
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void rtc_unlock(void)
{
  uint8_t regval;

  regval  = inp(EZ80_RTC_CTRL);
  regval |= EZ80_RTC_UNLOCK;
  outp(EZ80_RTC_CTRL, regval);
}

/****************************************************************************
 * Name: rtc_lock
 *
 * Description:
 *    Enable RTC write protection
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void rtc_lock(void)
{
  uint8_t regval;

  regval  = inp(EZ80_RTC_CTRL);
  regval &= ~EZ80_RTC_UNLOCK;
  outp(EZ80_RTC_CTRL, regval);
}

/****************************************************************************
 * Name: get_raw_time
 *
 * Description:
 *    Read all of the RTC time registers
 *
 * Input Parameters:
 *   rtcregs - Location to return the raw RTC time registers
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void get_raw_time(FAR struct rtc_timeregs_s *rtcregs)
{
  rtcregs->sec = inp(EZ80_RTC_SEC);
  rtcregs->min = inp(EZ80_RTC_MIN);
  rtcregs->hrs = inp(EZ80_RTC_HRS);
  rtcregs->dow = inp(EZ80_RTC_DOW);
  rtcregs->dom = inp(EZ80_RTC_DOM);
  rtcregs->mon = inp(EZ80_RTC_MON);
  rtcregs->yr  = inp(EZ80_RTC_YR);
  rtcregs->cen = inp(EZ80_RTC_CEN);
}

/****************************************************************************
 * Name: set_raw_time
 *
 * Description:
 *    Write all of the RTC time registers
 *
 * Input Parameters:
 *   almregs - New RTC register values
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void set_raw_time(FAR const struct rtc_timeregs_s *rtcregs)
{
  rtc_unlock();
  outp(EZ80_RTC_SEC, rtcregs->sec);
  outp(EZ80_RTC_MIN, rtcregs->min);
  outp(EZ80_RTC_HRS, rtcregs->hrs);
  outp(EZ80_RTC_DOW, rtcregs->dow);
  outp(EZ80_RTC_DOM, rtcregs->dom);
  outp(EZ80_RTC_MON, rtcregs->mon);
  outp(EZ80_RTC_YR,  rtcregs->yr);
  outp(EZ80_RTC_CEN, rtcregs->cen);
  rtc_lock();
}

/****************************************************************************
 * Name: get_raw_alarm
 *
 * Description:
 *    Read all of the RTC alarm registers
 *
 * Input Parameters:
 *   almregs - Location to return the raw RTC alarm registers
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static void get_raw_alarm(FAR struct rtc_almregs_s *almregs)
{
  almregs->sec = inp(EZ80_RTC_ASEC);
  almregs->min = inp(EZ80_RTC_AMIN);
  almregs->hrs = inp(EZ80_RTC_AHRS);
  almregs->dow = inp(EZ80_RTC_ADOW);
}
#endif

/****************************************************************************
 * Name: set_raw_alarm
 *
 * Description:
 *    Write all of the RTC alarm registers
 *
 * Input Parameters:
 *   almregs - New RTC register values
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static void set_raw_alarm(FAR const struct rtc_almregs_s *almregs)
{
  rtc_unlock();
  outp(EZ80_RTC_ASEC, almregs->sec);
  outp(EZ80_RTC_AMIN, almregs->min);
  outp(EZ80_RTC_AHRS, almregs->hrs);
  outp(EZ80_RTC_ADOW, almregs->dow);
  rtc_lock();
}
#endif

/****************************************************************************
 * Name: ez80_alarm_interrupt
 *
 * Description:
 *   RTC ALARM interrupt service routine through the EXTI line
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
static int ez80_alarm_interrupt(int irq, FAR void *context, FAR void *arg)
{
  uint8_t regval;

  /* Verify that the alarm interrupt is pending */

  regval = inp(EZ80_RTC_CTRL);
  if ((regval & EZ80_RTC_ALARM) != 0)
    {
      alm_callback_t cb;
      FAR void *cb_arg;

      /* Disable the alarm and disable the alarm interrupt */

      rtc_unlock();
      outp(EZ80_RTC_ACTRL, 0);

      regval  = inp(EZ80_RTC_CTRL);
      regval &= ~EZ80_RTC_INTEN;
      outp(EZ80_RTC_CTRL, regval);
      rtc_lock();

      up_disable_irq(EZ80_RTC_IRQ);

      /* Perform the alarm callback */

      cb               = g_alarmcb.ac_cb;
      cb_arg           = g_alarmcb.ac_arg;

      g_alarmcb.ac_cb  = NULL;
      g_alarmcb.ac_arg = NULL;

      cb(cb_arg);
    }

  return OK;
}
#endif

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
  uint8_t regval;

  /* Some boards do not have the external 32khz oscillator installed,
   * for those boards we must fallback to the crummy line frequency
   * clock source.
   */

  regval = 0;

#ifndef CONFIG_EZ80_RTC_32KHZ
#ifdef EZ80_RTC_LINEFREQ50
  /* Use the internal 50/60 Hz clock as the input to the RTC block */

  regval |= (EZ80_RTC_FREQSEL | EZ80_RTC_CLKSEL);
#else
  regval |= EZ80_RTC_CLKSEL;
#endif
#endif

  outp(EZ80_RTC_CTRL, regval);

#ifdef CONFIG_RTC_ALARM
  irq_attach(EZ80_RTC_IRQ, ez80_alarm_interrupt, NULL);
#endif

  rtc_dumpregs("After Initialization");
  g_rtc_enabled = true;
  return OK;
}

/****************************************************************************
 * Name: up_rtc_getdatetime
 *
 * Description:
 *   Get the current date and time from the date/time RTC.  This is used to
 *   replace the system timer.  It is only used by the RTOS during
 *   initialization to set up the system time when CONFIG_RTC and
 *   CONFIG_RTC_DATETIME are selected (and CONFIG_RTC_HIRES is not).
 *
 * Input Parameters:
 *   tp - The location to return the high resolution time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_getdatetime(FAR struct tm *tp)
{
  struct rtc_timeregs_s timeregs;
  struct rtc_timeregs_s tmpregs;

  rtc_dumpregs("Reading Time");

  /* Sample the data time registers.  There is a race condition here... ,
   * for example, we sample the time just before midnight on December 31,
   * the date could be wrong because the day rolled over while were
   * sampling. Thus loop for checking wrap here is needed.
   */

  do
    {
       get_raw_time(&tmpregs);
       get_raw_time(&timeregs);
    }
  while (tmpregs.min != timeregs.min ||
         tmpregs.hrs != timeregs.hrs ||
         tmpregs.dom != timeregs.dom ||
         tmpregs.mon != timeregs.mon ||
         tmpregs.yr  != timeregs.yr ||
         tmpregs.cen != timeregs.cen);

  /* Convert the RTC time to fields in struct tm format.  Most of the EZ80
   * ranges of values correspond between struct tm and the time registers.
   * Exceptions:  Month and year.
   */

  tp->tm_sec  = timeregs.sec;
  tp->tm_min  = timeregs.min;
  tp->tm_hour = timeregs.hrs;
  tp->tm_mday = timeregs.dom;
  tp->tm_mon  = timeregs.mon - 1;  /* Range is 0-11 */

  /* Years since 1900 */

  tp->tm_year = (uint16_t)timeregs.cen * 100 + (uint16_t)timeregs.yr - 1900;

  rtc_dumptime((FAR const struct tm *)tp, "Returning");
  return OK;
}

/****************************************************************************
 * Name: ez80_rtc_setdatetime
 *
 * Description:
 *   Set the RTC to the provided time. RTC implementations which provide
 *   up_rtc_getdatetime() (CONFIG_RTC_DATETIME is selected) should provide
 *   this function.
 *
 * Input Parameters:
 *   tp - the time to use
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int ez80_rtc_setdatetime(FAR const struct tm *tp)
{
  struct rtc_timeregs_s timeregs;
  uint16_t year;
  uint16_t cen;

  rtc_dumptime(tp, "Requested time");

  timeregs.sec = tp->tm_sec;
  timeregs.min = tp->tm_min;
  timeregs.hrs = tp->tm_hour;
  timeregs.dom = tp->tm_mday;
  timeregs.mon = tp->tm_mon + 1;  /* Range is 1-12 */

  /* Years AD */

  year         = tp->tm_year + 1900;
  cen          = year / 100;

  timeregs.cen = cen;
  timeregs.yr  = year - 100 * cen;

  set_raw_time(&timeregs);

  rtc_dumpregs("New time setting");
  return OK;
}

/****************************************************************************
 * Name: up_rtc_settime
 *
 * Description:
 *   Set the RTC to the provided time.  All RTC implementations must be able
 *   to set their time based on a standard timespec.
 *
 * Input Parameters:
 *   tp - the time to use
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_settime(FAR const struct timespec *tp)
{
  FAR struct tm newtime;

  /* Break out the time values (not that the time is set only to units of
   * seconds)
   */

  gmtime_r(&tp->tv_sec, &newtime);
  return ez80_rtc_setdatetime(&newtime);
}

/****************************************************************************
 * Name: ez80_rtc_setalarm
 *
 * Description:
 *   Set an alarm to an absolute time using associated hardware.
 *
 * Input Parameters:
 *  alminfo - Information about the alarm configuration.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
int ez80_rtc_setalarm(FAR struct alm_setalarm_s *alminfo)
{
  struct rtc_almregs_s almregs;
  uint8_t regval;
  int ret = -EINVAL;

  DEBUGASSERT(alminfo != NULL);

  rtc_dumptime(&alminfo->as_time, "New alarm time");

  /* Save the callback info */

  g_alarmcb.ac_cb  = alminfo->as_cb;
  g_alarmcb.ac_arg = alminfo->as_arg;

  /* Set the alarm time */

  almregs.sec = alminfo->as_time.tm_sec;
  almregs.min = alminfo->as_time.tm_min;
  almregs.hrs = alminfo->as_time.tm_hour;
  almregs.dow = alminfo->as_time.tm_wday;

  set_raw_alarm(&almregs);

  /* Enable the alarm */

  rtc_unlock();
  outp(EZ80_RTC_ACTRL, EZ80_RTC_AALL);

  regval  = inp(EZ80_RTC_CTRL);
  regval |= ~EZ80_RTC_INTEN;
  outp(EZ80_RTC_CTRL, regval);
  rtc_lock();

  rtc_dumpregs("Set Alarm");

  /* Enable the alarm interrupt at the interrupt controller */

  up_enable_irq(EZ80_RTC_IRQ);
  return OK;
}
#endif

/****************************************************************************
 * Name: ez80_rtc_cancelalarm
 *
 * Description:
 *   Cancel an alarm.
 *
 * Input Parameters:
 *  None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
int ez80_rtc_cancelalarm(void)
{
  uint8_t regval;

  /* Cancel the global callback function */

  g_alarmcb.ac_cb  = NULL;
  g_alarmcb.ac_arg = NULL;

  /* Disable RTC alarm and and the alarm interrupt */

  rtc_unlock();
  outp(EZ80_RTC_ACTRL, 0);

  regval  = inp(EZ80_RTC_CTRL);
  regval &= ~EZ80_RTC_INTEN;
  outp(EZ80_RTC_CTRL, regval);
  rtc_lock();

  up_disable_irq(EZ80_RTC_IRQ);
  return OK;
}
#endif

/****************************************************************************
 * Name: ez80_rtc_rdalarm
 *
 * Description:
 *   Return the current alarm setting.
 *
 * Input Parameters:
 *  almtime - Location to return the current alarm time.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
int ez80_rtc_rdalarm(FAR struct tm *almtime)
{
  struct rtc_almregs_s almregs;
  int ret = -EINVAL;

  rtc_dumpregs("Reading Alarm");

  /* Get the current time for the month and year */

  ret = up_rtc_getdatetime(almtime);
  if (ret < 0)
    {
      return ret;
    }

  /* Sample the alarm time registers.  There is no race condition in this
   * case.
   */

  get_raw_alarm(&almregs);

  /* Overwrite to get the full alarm time */

  almtime->tm_sec  = almregs.sec;
  almtime->tm_min  = almregs.min;
  almtime->tm_hour = almregs.hrs;
  almtime->tm_wday = almregs.dow;

  rtc_dumptime((FAR const struct tm *)almtime, "Returning");
  return OK;
}
#endif

#endif /* CONFIG_EZ80_RTC */
