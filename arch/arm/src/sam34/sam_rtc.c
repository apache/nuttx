/************************************************************************************
 * arch/arm/src/sam34/sam_rtc.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Bob Doiron
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <time.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/wqueue.h>

#include <arch/board/board.h>

#include "arm_arch.h"
#include "sam_rtc.h"

#if defined(CONFIG_RTC_HIRES) && defined (CONFIG_SAM34_RTT)
#  include "sam_rtt.h"
#  include "sam_periphclks.h"
#endif

#ifdef CONFIG_RTC

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/

#ifdef CONFIG_RTC_HIRES
# if !defined(CONFIG_SAM34_RTT)
#  error RTT is required to emulate high resolution RTC
# endif
# if (CONFIG_RTC_FREQUENCY > 32768) || ((32768 % CONFIG_RTC_FREQUENCY) != 0)
#  error CONFIG_RTC_FREQUENCY must be an integer division of 32768
# endif
#endif

#if defined(CONFIG_RTC_ALARM) && !defined(CONFIG_SCHED_WORKQUEUE)
#  error CONFIG_RTC_ALARM requires CONFIG_SCHED_WORKQUEUE
#endif

#define RTC_MAGIC 0xdeadbeef

/************************************************************************************
 * Private Data
 ************************************************************************************/

/* Callback to use when the alarm expires */

#ifdef CONFIG_RTC_ALARM
static alarmcb_t g_alarmcb;
struct work_s g_alarmwork;
#endif

/************************************************************************************
 * Public Data
 ************************************************************************************/

/* g_rtc_enabled is set true after the RTC has successfully initialized */

volatile bool g_rtc_enabled = false;

/* g_rtt_offset holds the rtt->rtc count offset */

#if defined(CONFIG_RTC_HIRES) && defined (CONFIG_SAM34_RTT)
uint32_t g_rtt_offset = 0;
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/
/************************************************************************************
 * Name: rtc_dumpregs
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
 ************************************************************************************/

#ifdef CONFIG_DEBUG_RTC_INFO
static void rtc_dumpregs(FAR const char *msg)
{
  rtcinfo("%s:\n", msg);
  rtcinfo("      CR: %08x\n", getreg32(SAM_RTC_CR));
  rtcinfo("      MR: %08x\n", getreg32(SAM_RTC_MR));
  rtcinfo("    TIMR: %08x\n", getreg32(SAM_RTC_TIMR));
  rtcinfo("    CALR: %08x\n", getreg32(SAM_RTC_CALR));
  rtcinfo("  TIMALR: %08x\n", getreg32(SAM_RTC_TIMALR));
  rtcinfo("  CALALR: %08x\n", getreg32(SAM_RTC_CALALR));
  rtcinfo("      SR: %08x\n", getreg32(SAM_RTC_SR));
  rtcinfo("     IMR: %08x\n", getreg32(SAM_RTC_IMR));
  rtcinfo("     VER: %08x\n", getreg32(SAM_RTC_VER));
}
#else
#  define rtc_dumpregs(msg)
#endif

/************************************************************************************
 * Name: rtc_dumptime
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
 ************************************************************************************/

#ifdef CONFIG_DEBUG_RTC_INFO
static void rtc_dumptime(FAR struct tm *tp, FAR const char *msg)
{
  rtcinfo("%s:\n", msg);
  rtcinfo("  tm_sec: %08x\n", tp->tm_sec);
  rtcinfo("  tm_min: %08x\n", tp->tm_min);
  rtcinfo(" tm_hour: %08x\n", tp->tm_hour);
  rtcinfo(" tm_mday: %08x\n", tp->tm_mday);
  rtcinfo("  tm_mon: %08x\n", tp->tm_mon);
  rtcinfo(" tm_year: %08x\n", tp->tm_year);
}
#else
#  define rtc_dumptime(tp, msg)
#endif

/************************************************************************************
 * Name: rtc_bin2bcd
 *
 * Description:
 *   Converts a 2 digit binary to BCD format
 *
 * Input Parameters:
 *   value - The byte to be converted.
 *
 * Returned Value:
 *   The value in BCD representation
 *
 ************************************************************************************/

static uint32_t rtc_bin2bcd(int value)
{
  uint32_t msbcd = 0;

  while (value >= 10)
    {
      msbcd++;
      value -= 10;
    }

  return (msbcd << 4) | value;
}

/************************************************************************************
 * Name: rtc_bin2bcd
 *
 * Description:
 *   Convert from 2 digit BCD to binary.
 *
 * Input Parameters:
 *   value - The BCD value to be converted.
 *
 * Returned Value:
 *   The value in binary representation
 *
 ************************************************************************************/

static int rtc_bcd2bin(uint32_t value)
{
  uint32_t tens = (value >> 4) * 10;
  return (int)(tens + (value & 0x0f));
}

/************************************************************************************
 * Name: rtc_worker
 *
 * Description:
 *    Perform alarm callback
 *
 * Input Parameters:
 *   Standard work callbacks
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value on failure.
 *
 ************************************************************************************/

#ifdef CONFIG_RTC_ALARM
static void rtc_worker(FAR void *arg)
{
  /* Sample once (atomically) */

  alarmcb_t alarmcb = g_alarmcb;

  /* Is there a subscriber to the alarm? */

  if (alarmcb)
    {
      /* Yes.. perform the callback */

      alarmcb();
    }
}
#endif

/************************************************************************************
 * Name: rtc_interrupt
 *
 * Description:
 *    RTC interrupt service routine
 *
 * Input Parameters:
 *   irq - The IRQ number that generated the interrupt
 *   context - Architecture specific register save information.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value on failure.
 *
 ************************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int rtc_interrupt(int irq, void *context, FAR void *arg)
{
  int ret;

  /* Schedule the callback to occur on the low-priority worker thread */

  DEBUGASSERT(work_available(&g_alarmwork));
  ret = work_queue(LPWORK, &g_alarmwork, rtc_worker, NULL, 0);
  if (ret < 0)
    {
      rtcerr("ERROR: work_queue failed: %d\n", ret);
    }

  /* Disable any further alarm interrupts */

  putreg32(RTC_IDR_ALRDIS, SAM_RTC_IDR);

  /* Clear any pending alarm interrupts */

  putreg32(RTC_SCCR_ALRCLR, SAM_RTC_SCCR);
  return OK;
}
#endif

/************************************************************************************
 * Name: rtc_sync
 *
 * Description:
 *   Waits and returns immediately after 1 sec tick. For best accuracy,
 *   call with interrupts disabled.
 *
 * Returns value of the TIMR register
 *
 ************************************************************************************/

static uint32_t rtc_sync(void)
{
  uint32_t r0, r1;

  /* Get start second (stable) */

  do
    {
      r0 = getreg32(SAM_RTC_TIMR);
    }
  while (r0 != getreg32(SAM_RTC_TIMR));

  /* Now read until it changes */

  do
    {
      r1 = getreg32(SAM_RTC_TIMR);
    }
  while (r1 == r0);

  return r1;
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: up_rtc_initialize
 *
 * Description:
 *   Initialize the hardware RTC per the selected configuration.  This function is
 *   called once during the OS initialization sequence
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

int up_rtc_initialize(void)
{
  uint32_t ver;

  rtc_dumpregs("On reset");

  /* No clocking setup need be performed. The Real-time Clock is continuously clocked
   * at 32768 Hz (SCLK). The Power Management Controller has no effect on RTC
   * behavior.
   */

  /* Set the 24 hour format */

  putreg32(0, SAM_RTC_MR);

  /* Has the RTC been initialized? */

  ver = getreg32(SAM_RTC_VER);
  g_rtc_enabled = ((ver & (RTC_VER_NVTIM | RTC_VER_NVCAL)) == 0);

#ifdef CONFIG_RTC_ALARM
  /* Then attach the ALARM interrupt handler */

  irq_attach(SAM_IRQ_RTC, rtc_interrupt, NULL);

  /* Should RTC alarm interrupt be enabled at the peripheral?  Let's assume so
   * for now.  Let's say yes if the time is valid and a valid alarm has been
   * programmed.
   */

  if (g_rtc_enabled && (ver & (RTC_VER_NVTIMALR | RTC_VER_NVCALALR)) == 0)
    {
      /* Enable the alarm interrupt at the RTC */

      putreg32(RTC_IER_ALREN, SAM_RTC_IER);
    }
  else
    {
      /* Disable the alarm interrupt at the RTC */

      putreg32(RTC_IDR_ALRDIS, SAM_RTC_IDR);
    }

  /* Enable RTC interrupts */

  up_enable_irq(SAM_IRQ_RTC);

#endif

#if defined(CONFIG_RTC_HIRES) && defined(CONFIG_SAM34_RTT)
  /* Using the RTT for subsecond ticks. */

  sam_rtt_enableclk();

  /* Disable ints, set prescaler, start counter */

  putreg32(RTT_MR_RTPRES(32768/CONFIG_RTC_FREQUENCY) | RTT_MR_RTTRST, SAM_RTT_MR);

  /* wait for a second tick to get the RTT offset.
   * Interrupts are assumed to still be off at this point.
   */

  rtc_sync();

  /* Probably safe to read the RTT_VR register now since the clock just ticked,
   * but we'll be careful anyway.
   */

  do
    {
      g_rtt_offset = getreg32(SAM_RTT_VR);
    }
  while (getreg32(SAM_RTT_VR) != g_rtt_offset);
#endif

  rtc_dumpregs("After Initialization");
  return OK;
}

/************************************************************************************
 * Name: up_rtc_getdatetime
 *
 * Description:
 *   Get the current date and time from the date/time RTC.  This interface
 *   is only supported by the date/time RTC hardware implementation.
 *   It is used to replace the system timer.  It is only used by the RTOS during
 *   initialization to set up the system time when CONFIG_RTC and CONFIG_RTC_DATETIME
 *   are selected (and CONFIG_RTC_HIRES is not).
 *
 *   NOTE: Some date/time RTC hardware is capability of sub-second accuracy.  That
 *   sub-second accuracy is lost in this interface.  However, since the system time
 *   is reinitialized on each power-up/reset, there will be no timing inaccuracy in
 *   the long run.
 *
 * Input Parameters:
 *   tp - The location to return the high resolution time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

int up_rtc_getdatetime(FAR struct tm *tp)
{
  uint32_t timr;
  uint32_t calr;
  uint32_t cent;
  uint32_t year;
  uint32_t tmp;

  /* Sample the data time registers.  There is a race condition here... If we sample
   * the time just before midnight on December 31, the date could be wrong because
   * the day rolled over while were sampling.
   */

  do
    {
      calr  = getreg32(SAM_RTC_CALR);
      timr  = getreg32(SAM_RTC_TIMR);
      tmp   = getreg32(SAM_RTC_CALR);
    }
  while (tmp != calr);

  rtc_dumpregs("Reading Time");

  /* Convert the RTC time register fields to struct tm format.
   *
   *   struct tm       TIMR register
   *   tm_sec    0-61* SEC    (0-59)
   *   tm_min    0-59  MIN    (0-59)
   *   tm_hour   0-23  HOUR   (0-23)
   *
   *  *To allow for leap seconds.  But these never actually happen.
   */

  tmp = (timr & RTC_TIMR_SEC_MASK) >> RTC_TIMR_SEC_SHIFT;
  tp->tm_sec = rtc_bcd2bin(tmp);

  tmp = (timr & RTC_TIMR_MIN_MASK) >> RTC_TIMR_MIN_SHIFT;
  tp->tm_min = rtc_bcd2bin(tmp);

  tmp = (timr & RTC_TIMR_HOUR_MASK) >> RTC_TIMR_HOUR_SHIFT;
  tp->tm_hour = rtc_bcd2bin(tmp);

  /* Convert the RTC date register fields to struct tm format.
   *
   *   struct tm       TIMR register
   *   tm_mday   1-31  DATE   (1-31)
   *   tm_wday   0-6   DAY    (1-7)  **
   *   tm_mon    0-11  MONTH: (1-12)
   *   tm_year   *     YEAR   (0-99)
   *                   CENT   (19-20)
   *
   *  *Years since 1900
   * **Day of the week is not supported
   */

  tmp  = (calr & RTC_CALR_DATE_MASK) >> RTC_CALR_DATE_SHIFT;
  tp->tm_mday = rtc_bcd2bin(tmp);

  tmp  = (calr & RTC_CALR_MONTH_MASK) >> RTC_CALR_MONTH_SHIFT;
  tp->tm_mon = rtc_bcd2bin(tmp) - 1;

  tmp  = (calr & RTC_CALR_CENT_MASK) >> RTC_CALR_CENT_SHIFT;
  cent =  rtc_bcd2bin(tmp);
  tmp  = (calr & RTC_CALR_YEAR_MASK) >> RTC_CALR_YEAR_SHIFT;
  year =  rtc_bcd2bin(tmp);
  tp->tm_year = cent * 100 + year - 1900;

  rtc_dumptime(tp, "Returning");
  return OK;
}

/************************************************************************************
 * Name: up_rtc_settime
 *
 * Description:
 *   Set the RTC to the provided time.  All RTC implementations must be able to
 *   set their time based on a standard timespec.
 *
 * Input Parameters:
 *   tp - the time to use
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

int up_rtc_settime(FAR const struct timespec *tp)
{
  FAR struct tm newtime;
  uint32_t regval;
  uint32_t timr;
  uint32_t calr;
  uint32_t cent;
  uint32_t year;

  /* Break out the time values (note that the time is set only to units of seconds) */

  gmtime_r(&tp->tv_sec, &newtime);
  rtc_dumptime(&newtime, "Setting time");

  /* Then write the broken out values to the RTC */

  /* Convert the struct tm format to RTC time register fields.
   *
   *   struct tm       TIMR register
   *   tm_sec    0-61* SEC    (0-59)
   *   tm_min    0-59  MIN    (0-59)
   *   tm_hour   0-23  HOUR   (0-23)
   *
   *  *To allow for leap seconds.  But these never actually happen.
   */

  timr  = (rtc_bin2bcd(newtime.tm_sec)  << RTC_TIMR_SEC_SHIFT)  & RTC_TIMR_SEC_MASK;
  timr |= (rtc_bin2bcd(newtime.tm_min)  << RTC_TIMR_MIN_SHIFT)  & RTC_TIMR_MIN_MASK;
  timr |= (rtc_bin2bcd(newtime.tm_hour) << RTC_TIMR_HOUR_SHIFT) & RTC_TIMR_HOUR_MASK;

  /* Convert the struct tm format to RTC date register fields.
   *
   *   struct tm       CALR register
   *   tm_mday   1-31  DATE   (1-31)
   *   tm_wday   0-6   DAY    (1-7)  **
   *   tm_mon    0-11  MONTH: (1-12)
   *   tm_year   *     YEAR   (0-99)
   *                   CENT   (19-20)
   *
   *  *Years since 1900
   * **Day of the week is not supported.  Set to Monday.
   */

  calr  = (rtc_bin2bcd(newtime.tm_mday)  << RTC_CALR_DATE_SHIFT)  & RTC_CALR_DATE_MASK;
  calr |= (rtc_bin2bcd(1)                << RTC_CALR_DAY_SHIFT)   & RTC_CALR_DAY_MASK;
  calr |= (rtc_bin2bcd(newtime.tm_mon+1) << RTC_CALR_MONTH_SHIFT) & RTC_CALR_MONTH_MASK;

  cent  = newtime.tm_year / 100 + 19;
  year  = newtime.tm_year % 100;

  calr |= (rtc_bin2bcd(year)             << RTC_CALR_YEAR_SHIFT) & RTC_CALR_YEAR_MASK;
  calr |= (rtc_bin2bcd(cent)             << RTC_CALR_CENT_SHIFT) & RTC_CALR_CENT_MASK;

  /* Stop RTC time and date counting */

  regval  = getreg32(SAM_RTC_CR);
  regval |= (RTC_CR_UPDTIM | RTC_CR_UPDCAL);
  putreg32(regval, SAM_RTC_CR);

  /* Wait until the RTC has stopped so that we can update the time */

  while ((getreg32(SAM_RTC_SR) & RTC_SR_ACKUPD) != RTC_SR_ACKUPD);

  /* Clear the ACKUPD bit in the status register */

  putreg32(RTC_SCCR_ACKCLR, SAM_RTC_SCCR);

  /* Set the new date */

  putreg32(calr, SAM_RTC_CALR);

  /* Write the new time */

  putreg32(timr, SAM_RTC_TIMR);

  /* Resume RTC date/time counting */

  regval  = getreg32(SAM_RTC_CR);
  regval &= ~(RTC_CR_UPDTIM | RTC_CR_UPDCAL);
  putreg32(regval, SAM_RTC_CR);

  /* Clear the SEC status in the SR */

  regval = getreg32(SAM_RTC_SCCR);
  regval = RTC_SCCR_SECCLR;
  putreg32(regval, SAM_RTC_SCCR);

  /* The RTC should now be enabled */

  g_rtc_enabled = ((getreg32(SAM_RTC_VER) & (RTC_VER_NVTIM | RTC_VER_NVCAL)) == 0);
  DEBUGASSERT(g_rtc_enabled);

  rtc_dumpregs("New time setting");
  return OK;
}

/************************************************************************************
 * Name: sam_rtc_setalarm
 *
 * Description:
 *   Set up an alarm.  Up to two alarms can be supported (ALARM A and ALARM B).
 *
 * Input Parameters:
 *   tp - the time to set the alarm
 *   callback - the function to call when the alarm expires.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

#ifdef CONFIG_RTC_ALARM
int sam_rtc_setalarm(FAR const struct timespec *tp, alarmcb_t callback)
{
  FAR struct tm newalarm;
  irqstate_t flags;
  uint32_t timalr;
  uint32_t calalr;
  int ret = -EBUSY;

  /* Is there already something waiting on the ALARM? */

  flags = enter_critical_section();
  if (g_alarmcb == NULL)
    {
      /* No.. Save the callback function pointer */

      g_alarmcb = callback;

      /* Clear any pending alarm interrupts */

      putreg32(RTC_SCCR_ALRCLR, SAM_RTC_SCCR);

      /* Break out the time values (note that the time is set only to units
       * of seconds)
       */

      gmtime_r(&tp->tv_sec, &newalarm);
      rtc_dumptime(&newalarm, "Setting alarm");

      /* Then write the broken out values to the RTC */

      /* Convert the struct tm format to RTC time register fields.
       *
       *   struct tm       TIMALR register
       *   tm_sec    0-61* SEC    (0-59)
       *   tm_min    0-59  MIN    (0-59)
       *   tm_hour   0-23  HOUR   (0-23)
       *
       *  *To allow for leap seconds.  But these never actually happen.
       */

      timalr  = (rtc_bin2bcd(newalarm.tm_sec)  << RTC_TIMALR_SEC_SHIFT)  & RTC_TIMALR_SEC_MASK;
      timalr |= (rtc_bin2bcd(newalarm.tm_min)  << RTC_TIMALR_MIN_SHIFT)  & RTC_TIMALR_MIN_MASK;
      timalr |= (rtc_bin2bcd(newalarm.tm_hour) << RTC_TIMALR_HOUR_SHIFT) & RTC_TIMALR_HOUR_MASK;
      timalr |= (RTC_TIMALR_SECEN | RTC_TIMALR_MINEN | RTC_TIMALR_HOUREN);

      /* Convert the struct tm format to RTC date register fields.
       *
       *   struct tm       CALALR register
       *   tm_mday   1-31  DATE   (1-31)
       *   tm_wday   0-6   DAY    (1-7)  **
       *   tm_mon    0-11  MONTH: (1-12)
       *   tm_year   *     YEAR   (0-99)
       *                   CENT   (19-20)
       *
       *  *Years since 1900
       * **Day of the week is not supported
       */

      calalr  = (rtc_bin2bcd(newalarm.tm_mday)  << RTC_CALALR_DATE_SHIFT)  & RTC_CALALR_DATE_MASK;
      calalr |= (rtc_bin2bcd(newalarm.tm_mon+1) << RTC_CALALR_MONTH_SHIFT) & RTC_CALALR_MONTH_MASK;
      calalr |= (RTC_CALALR_MTHEN | RTC_CALALR_DATEEN);

      /* Set the new date */

      putreg32(calalr, SAM_RTC_CALALR);

      /* Write the new time */

      putreg32(timalr, SAM_RTC_TIMALR);

      DEBUGASSERT((getreg32(SAM_RTC_VER) & RTC_VER_NVTIMALR) == 0);
      DEBUGASSERT((getreg32(SAM_RTC_VER) & RTC_VER_NVCALALR) == 0);

      rtc_dumpregs("New alarm setting");

      /* Enable alarm interrupts */

      putreg32(RTC_IER_ALREN, SAM_RTC_IER);
      ret = OK;
    }

  leave_critical_section(flags);
  return ret;
}
#endif


/************************************************************************************
 * Name: up_rtc_gettime
 *
 * Description:
 *   Get the current time from the high resolution RTC clock/counter.  This interface
 *   is only supported by the high-resolution RTC/counter hardware implementation.
 *   It is used to replace the system timer.
 *
 * Input Parameters:
 *   tp - The location to return the high resolution time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

#if defined(CONFIG_RTC_HIRES) && defined (CONFIG_SAM34_RTT)
int up_rtc_gettime(FAR struct timespec *tp)
{
  /* This is a hack to emulate a high resolution rtc using the rtt */

  uint32_t rtc_cal, rtc_tim, rtt_val;
  struct tm t;

  do
    {
      rtc_cal = getreg32(SAM_RTC_CALR);
      rtc_tim = getreg32(SAM_RTC_TIMR);
      rtt_val = getreg32(SAM_RTT_VR);
    }
  while (rtc_cal != getreg32(SAM_RTC_CALR) ||
         rtc_tim != getreg32(SAM_RTC_TIMR) ||
         rtt_val != getreg32(SAM_RTT_VR));

  t.tm_sec  = rtc_bcd2bin((rtc_tim & RTC_TIMR_SEC_MASK)   >> RTC_TIMR_SEC_SHIFT);
  t.tm_min  = rtc_bcd2bin((rtc_tim & RTC_TIMR_MIN_MASK)   >> RTC_TIMR_MIN_SHIFT);
  t.tm_hour = rtc_bcd2bin((rtc_tim & RTC_TIMR_HOUR_MASK)  >> RTC_TIMR_HOUR_SHIFT);
  t.tm_mday = rtc_bcd2bin((rtc_cal & RTC_CALR_DATE_MASK)  >> RTC_CALR_DATE_SHIFT);
  t.tm_mon  = rtc_bcd2bin((rtc_cal & RTC_CALR_MONTH_MASK) >> RTC_CALR_MONTH_SHIFT);
  t.tm_year = (rtc_bcd2bin((rtc_cal & RTC_CALR_CENT_MASK) >> RTC_CALR_CENT_SHIFT) * 100)
             + rtc_bcd2bin((rtc_cal & RTC_CALR_YEAR_MASK) >> RTC_CALR_YEAR_SHIFT)
             - 1900;

  tp->tv_sec = mktime(&t);
  tp->tv_nsec = (((rtt_val-g_rtt_offset) & (CONFIG_RTC_FREQUENCY-1)) * 1000000000ULL) /
                CONFIG_RTC_FREQUENCY;

  return OK;
}
#endif

#endif /* CONFIG_RTC */
