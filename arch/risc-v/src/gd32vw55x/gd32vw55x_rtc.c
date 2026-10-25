/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gd32vw55x_rtc.c
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

#include <inttypes.h>
#include <stdint.h>
#include <time.h>
#include <errno.h>
#include <sys/param.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>

#include "riscv_internal.h"
#include "gd32vw55x_clockconfig.h"
#include "gd32vw55x_pm.h"
#include "hardware/gd32vw55x_exti.h"
#include "hardware/gd32vw55x_rcu.h"
#include "chip.h"

#ifdef CONFIG_RTC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RTC registers */

#define GD32VW55X_RTC_TIME      (GD32VW55X_RTC_BASE + 0x0000)
#define GD32VW55X_RTC_DATE      (GD32VW55X_RTC_BASE + 0x0004)
#define GD32VW55X_RTC_CTL       (GD32VW55X_RTC_BASE + 0x0008)
#define GD32VW55X_RTC_STAT      (GD32VW55X_RTC_BASE + 0x000c)
#define GD32VW55X_RTC_PSC       (GD32VW55X_RTC_BASE + 0x0010)
#define GD32VW55X_RTC_WUT       (GD32VW55X_RTC_BASE + 0x0014)
#define GD32VW55X_RTC_WPK       (GD32VW55X_RTC_BASE + 0x0024)
#define GD32VW55X_RTC_SS        (GD32VW55X_RTC_BASE + 0x0028)

/* CTL bits */

#define RTC_CTL_WTCS_MASK       (7 << 0)   /* Wake-up timer clock */
#define RTC_CTL_WTCS_DIV16      (0 << 0)   /* RTC clock / 16 */
#define RTC_CTL_BPSHAD          (1 << 5)   /* Bypass the shadow registers */
#define RTC_CTL_WTEN            (1 << 10)  /* Wake-up timer enable */
#define RTC_CTL_WTIE            (1 << 14)  /* Wake-up timer interrupt */

/* STAT bits */

#define RTC_STAT_WTWF           (1 << 2)   /* Wake-up timer writable */
#define RTC_STAT_RSYNF          (1 << 5)   /* Registers synchronized */
#define RTC_STAT_INITF          (1 << 6)   /* In initialization state */
#define RTC_STAT_INITM          (1 << 7)   /* Enter initialization mode */
#define RTC_STAT_WTF            (1 << 10)  /* Wake-up timer flag */

/* Write-protection key sequence */

#define RTC_UNLOCK_KEY1         0xca
#define RTC_UNLOCK_KEY2         0x53
#define RTC_LOCK_KEY            0xff

/* IRC32K (32 kHz): async 2, sync 16000 -> 1 Hz calendar clock.  The
 * small asynchronous prescaler gives sub-second units of about 60 us, which
 * measure the time spent in deep-sleep.
 */

#define RTC_PSC_FACTOR_A        0x01
#define RTC_PSC_FACTOR_S        0x3e7f
#define RTC_PSC_FACTOR_A_SHIFT  16
#define RTC_PSC_FACTOR_A_MASK   0x7f
#define RTC_PSC_FACTOR_S_MASK   0x7fff
#define RTC_SS_MASK             0xffff

/* The IRC32K is off by a few percent.  Its frequency is measured against
 * the core clock, which comes from the crystal: the CPU cycles are counted
 * over about 100 ms of sub-second ticks of the RTC (RTC_CALIB_CLOCKS RTC
 * clocks), each tick bounded by RTC_CALIB_TIMEOUT cycles (20 ms).  The
 * result is used if it is within 25 percent of the nominal frequency (the
 * datasheets give no tolerance).
 */

#define RTC_IRC32K_FREQUENCY    32000
#define RTC_CALIB_CLOCKS        3200
#define RTC_CALIB_TIMEOUT       (GD32VW55X_MTIME_FREQ / 50)

/* Low word of the SysTimer counter, which runs from the system clock */

#define GD32VW55X_RTC_MTIME    (GD32VW55X_SYSTIMER_BASE + 0x0000)

/* RCU_RSTSCK register (IRC32K control) */

#define GD32VW55X_RCU_RSTSCK    (GD32VW55X_RCU_BASE + 0x0074)
#define RCU_RSTSCK_IRC32KEN     (1 << 0)
#define RCU_RSTSCK_IRC32KSTB    (1 << 1)

/* RCU_BDCTL bits */

#define RCU_BDCTL_RTCSRC_SHIFT  8
#define RCU_BDCTL_RTCSRC_MASK   (3 << RCU_BDCTL_RTCSRC_SHIFT)
#define RCU_BDCTL_RTCSRC_IRC32K (2 << RCU_BDCTL_RTCSRC_SHIFT)
#define RCU_BDCTL_RTCEN         (1 << 15)

#define GD32VW55X_RCU_BDCTL     (GD32VW55X_RCU_BASE + 0x0070)

/* PMU: writes to the backup domain (RTC) require BKPWEN, otherwise the
 * hardware ignores them silently (symptom: INITM never sets, STAT=0x37).
 */

#define GD32VW55X_PMU_CTL0      (GD32VW55X_PMU_BASE + 0x0000)
#define PMU_CTL0_BKPWEN         (1 << 8)

/* The wake-up timer counts the RTC clock divided by 16, with a 16-bit
 * reload value.  Its interrupt goes through EXTI line 21.
 */

#define RTC_WUT_DIV             16
#define RTC_WUT_MAX             0xffff
#define RTC_WKUP_EXTI_LINE      (1 << 21)

/* BCD helpers */

#define BCD2DEC(x)              ((((x) >> 4) * 10) + ((x) & 0x0f))
#define DEC2BCD(x)              ((((x) / 10) << 4) | ((x) % 10))

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile bool g_rtc_enabled = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void rtc_unlock(void)
{
  putreg32(RTC_UNLOCK_KEY1, GD32VW55X_RTC_WPK);
  putreg32(RTC_UNLOCK_KEY2, GD32VW55X_RTC_WPK);
}

static void rtc_lock(void)
{
  putreg32(RTC_LOCK_KEY, GD32VW55X_RTC_WPK);
}

/* Wait until the calendar shadow registers are synchronized */

static int rtc_synchwait(void)
{
  int timeout;

  rtc_unlock();
  modifyreg32(GD32VW55X_RTC_STAT, RTC_STAT_RSYNF, 0);

  for (timeout = 1000000; timeout > 0; timeout--)
    {
      if ((getreg32(GD32VW55X_RTC_STAT) & RTC_STAT_RSYNF) != 0)
        {
          break;
        }
    }

  rtc_lock();
  return timeout > 0 ? OK : -ETIMEDOUT;
}

/****************************************************************************
 * Name: rtc_ss_edge
 *
 * Description:
 *   Wait for the next change of the sub-second counter, at most
 *   RTC_CALIB_TIMEOUT core cycles.
 *
 ****************************************************************************/

static int rtc_ss_edge(void)
{
  uint32_t ss    = getreg32(GD32VW55X_RTC_SS) & RTC_SS_MASK;
  uint32_t start = getreg32(GD32VW55X_RTC_MTIME);

  while ((getreg32(GD32VW55X_RTC_SS) & RTC_SS_MASK) == ss)
    {
      if (getreg32(GD32VW55X_RTC_MTIME) - start > RTC_CALIB_TIMEOUT)
        {
          return -ETIMEDOUT;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: rtc_sync_prescaler
 *
 * Description:
 *   Measure the IRC32K against the core clock and return the synchronous
 *   prescaler that gives a 1 Hz calendar with RTC_PSC_FACTOR_A, or
 *   RTC_PSC_FACTOR_S if the measure fails.
 *
 ****************************************************************************/

static uint32_t rtc_sync_prescaler(void)
{
  uint32_t diva;
  uint32_t ticks;
  uint32_t start;
  uint32_t cycles;
  uint32_t freq;
  uint32_t i;

  /* Read the sub-second counter only once the shadow registers hold the
   * calendar: reading it locks the TIME and DATE shadows until DATE is
   * read, and leaving the initialization mode loads the calendar from the
   * shadows.  After a system reset, they hold their reset values until the
   * first synchronization.
   */

  if (rtc_synchwait() < 0)
    {
      return RTC_PSC_FACTOR_S;
    }

  /* The RTC counts with the prescalers it has now */

  diva  = ((getreg32(GD32VW55X_RTC_PSC) >> RTC_PSC_FACTOR_A_SHIFT) &
           RTC_PSC_FACTOR_A_MASK) + 1;
  ticks = MAX(RTC_CALIB_CLOCKS / diva, 1);

  if (rtc_ss_edge() < 0)
    {
      getreg32(GD32VW55X_RTC_TIME);
      getreg32(GD32VW55X_RTC_DATE);
      return RTC_PSC_FACTOR_S;
    }

  start = getreg32(GD32VW55X_RTC_MTIME);

  for (i = 0; i < ticks; i++)
    {
      if (rtc_ss_edge() < 0)
        {
          break;
        }
    }

  cycles = getreg32(GD32VW55X_RTC_MTIME) - start;

  /* Release the TIME and DATE shadows */

  getreg32(GD32VW55X_RTC_TIME);
  getreg32(GD32VW55X_RTC_DATE);

  freq   = (uint32_t)(((uint64_t)ticks * diva * GD32VW55X_MTIME_FREQ) /
                       cycles);

  if (i < ticks ||
      freq < RTC_IRC32K_FREQUENCY - RTC_IRC32K_FREQUENCY / 4 ||
      freq > RTC_IRC32K_FREQUENCY + RTC_IRC32K_FREQUENCY / 4)
    {
      return RTC_PSC_FACTOR_S;
    }

  rtcinfo("IRC32K: %" PRIu32 " Hz\n", freq);

  return (freq + (RTC_PSC_FACTOR_A + 1) / 2) / (RTC_PSC_FACTOR_A + 1) - 1;
}

/****************************************************************************
 * Name: rtc_wakeup_interrupt
 *
 * Description:
 *   The RTC wake-up timer interrupt: its only job is to end deep-sleep.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static int rtc_wakeup_interrupt(int irq, void *context, void *arg)
{
  modifyreg32(GD32VW55X_RTC_STAT, RTC_STAT_WTF, 0);
  putreg32(RTC_WKUP_EXTI_LINE, GD32VW55X_EXTI_PD);
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
 *   Initialize the hardware RTC per the selected configuration.  Clock
 *   source is the internal 32 kHz RC oscillator (the START board has no
 *   LXTAL populated), divided to 1 Hz.
 *
 ****************************************************************************/

int up_rtc_initialize(void)
{
  uint32_t sync;
  int timeout;

  /* PMU clock + backup-domain write enable (without this every write to
   * the RTC is discarded by the hardware).
   */

  modifyreg32(GD32VW55X_RCU_APB1EN, 0, RCU_APB1EN_PMUEN);
  modifyreg32(GD32VW55X_PMU_CTL0, 0, PMU_CTL0_BKPWEN);

  /* Enable IRC32K and wait for it to stabilize */

  modifyreg32(GD32VW55X_RCU_RSTSCK, 0, RCU_RSTSCK_IRC32KEN);

  for (timeout = 1000000; timeout > 0; timeout--)
    {
      if ((getreg32(GD32VW55X_RCU_RSTSCK) & RCU_RSTSCK_IRC32KSTB) != 0)
        {
          break;
        }
    }

  if (timeout <= 0)
    {
      return -ETIMEDOUT;
    }

  /* RTC clock source = IRC32K, RTC clock on.  Preserve an already
   * configured source (e.g. set before a warm reset).
   */

  if ((getreg32(GD32VW55X_RCU_BDCTL) & RCU_BDCTL_RTCSRC_MASK) !=
      RCU_BDCTL_RTCSRC_IRC32K)
    {
      modifyreg32(GD32VW55X_RCU_BDCTL, RCU_BDCTL_RTCSRC_MASK,
                  RCU_BDCTL_RTCSRC_IRC32K);
    }

  modifyreg32(GD32VW55X_RCU_BDCTL, 0, RCU_BDCTL_RTCEN);

  /* Measure the IRC32K, then program the prescalers (init mode required;
   * the calendar keeps its value).  This is done at every boot, since the
   * IRC32K drifts with the temperature.
   */

  sync = rtc_sync_prescaler();
  rtc_synchwait();

  rtc_unlock();
  modifyreg32(GD32VW55X_RTC_STAT, 0, RTC_STAT_INITM);

  for (timeout = 1000000; timeout > 0; timeout--)
    {
      if ((getreg32(GD32VW55X_RTC_STAT) & RTC_STAT_INITF) != 0)
        {
          break;
        }
    }

  if (timeout > 0)
    {
      putreg32((RTC_PSC_FACTOR_A << RTC_PSC_FACTOR_A_SHIFT) | sync,
               GD32VW55X_RTC_PSC);
    }

  modifyreg32(GD32VW55X_RTC_STAT, RTC_STAT_INITM, 0);
  rtc_lock();

  rtc_synchwait();

  g_rtc_enabled = true;
  return OK;
}

/****************************************************************************
 * Name: up_rtc_getdatetime
 *
 * Description:
 *   Get the current date and time from the hardware RTC.
 *
 ****************************************************************************/

int up_rtc_getdatetime(struct tm *tp)
{
  uint32_t time;
  uint32_t date;

  /* Reading TIME locks DATE until DATE is read, so this pair is atomic */

  time = getreg32(GD32VW55X_RTC_TIME);
  date = getreg32(GD32VW55X_RTC_DATE);

  tp->tm_sec  = BCD2DEC(time & 0x7f);
  tp->tm_min  = BCD2DEC((time >> 8) & 0x7f);
  tp->tm_hour = BCD2DEC((time >> 16) & 0x3f);

  tp->tm_mday = BCD2DEC(date & 0x3f);
  tp->tm_mon  = BCD2DEC((date >> 8) & 0x1f) - 1;
  tp->tm_year = BCD2DEC((date >> 16) & 0xff) + 100;   /* 20xx */
  tp->tm_wday = (date >> 13) & 0x07;
  if (tp->tm_wday == 7)
    {
      tp->tm_wday = 0;
    }

  return OK;
}

/****************************************************************************
 * Name: up_rtc_settime
 *
 * Description:
 *   Set the hardware RTC to the provided time.
 *
 ****************************************************************************/

int up_rtc_settime(const struct timespec *tp)
{
  struct tm t;
  uint32_t regval;
  int timeout;
  int wday;

  gmtime_r(&tp->tv_sec, &t);

  rtc_unlock();
  modifyreg32(GD32VW55X_RTC_STAT, 0, RTC_STAT_INITM);

  for (timeout = 1000000; timeout > 0; timeout--)
    {
      if ((getreg32(GD32VW55X_RTC_STAT) & RTC_STAT_INITF) != 0)
        {
          break;
        }
    }

  if (timeout <= 0)
    {
      rtc_lock();
      rtcerr("ERROR: settime INITF timeout (STAT=%08lx)\n",
             (unsigned long)getreg32(GD32VW55X_RTC_STAT));
      return -ETIMEDOUT;
    }

  rtcinfo("settime %02d:%02d:%02d\n",
         t.tm_hour, t.tm_min, t.tm_sec);

  regval = (DEC2BCD(t.tm_sec)) |
           (DEC2BCD(t.tm_min) << 8) |
           (DEC2BCD(t.tm_hour) << 16);
  putreg32(regval, GD32VW55X_RTC_TIME);

  /* RTC day-of-week: 1..7 with 7 = Sunday */

  wday = (t.tm_wday == 0) ? 7 : t.tm_wday;

  regval = (DEC2BCD(t.tm_mday)) |
           (DEC2BCD(t.tm_mon + 1) << 8) |
           ((uint32_t)wday << 13) |
           (DEC2BCD(t.tm_year - 100) << 16);
  putreg32(regval, GD32VW55X_RTC_DATE);

  modifyreg32(GD32VW55X_RTC_STAT, RTC_STAT_INITM, 0);
  rtc_lock();

  return rtc_synchwait();
}

/****************************************************************************
 * Name: gd32vw55x_rtc_setwakeup
 *
 * Description:
 *   Start the RTC wake-up timer to raise its interrupt (EXTI line 21) in
 *   'usec' microseconds, at most about 30 s, or stop it if 'usec' is 0.
 *
 ****************************************************************************/

int gd32vw55x_rtc_setwakeup(uint32_t usec)
{
  uint32_t psc;
  uint32_t hz;
  uint64_t ticks;
  int timeout;
  int ret = OK;

#ifdef CONFIG_PM
  static bool attached;

  if (!attached)
    {
      irq_attach(GD32VW55X_IRQ_RTC_WKUP, rtc_wakeup_interrupt, NULL);
      up_enable_irq(GD32VW55X_IRQ_RTC_WKUP);
      attached = true;
    }
#endif

  rtc_unlock();

  /* Stop the timer, then wait until its reload value can be written */

  modifyreg32(GD32VW55X_RTC_CTL, RTC_CTL_WTEN | RTC_CTL_WTIE, 0);
  modifyreg32(GD32VW55X_RTC_STAT, RTC_STAT_WTF, 0);
  modifyreg32(GD32VW55X_EXTI_INTEN, RTC_WKUP_EXTI_LINE, 0);
  putreg32(RTC_WKUP_EXTI_LINE, GD32VW55X_EXTI_PD);

  if (usec > 0)
    {
      for (timeout = 1000000; timeout > 0; timeout--)
        {
          if ((getreg32(GD32VW55X_RTC_STAT) & RTC_STAT_WTWF) != 0)
            {
              break;
            }
        }

      if (timeout <= 0)
        {
          ret = -ETIMEDOUT;
        }
      else
        {
          /* The RTC clock is the calendar clock times the prescalers, the
           * timer counts the reload value plus one clock down to 0.
           */

          psc   = getreg32(GD32VW55X_RTC_PSC);
          hz    = (((psc >> RTC_PSC_FACTOR_A_SHIFT) &
                    RTC_PSC_FACTOR_A_MASK) + 1) *
                  ((psc & RTC_PSC_FACTOR_S_MASK) + 1);
          ticks = ((uint64_t)usec * (hz / RTC_WUT_DIV)) / USEC_PER_SEC;
          ticks = MAX(MIN(ticks, RTC_WUT_MAX), 1);

          putreg32((uint32_t)ticks - 1, GD32VW55X_RTC_WUT);
          modifyreg32(GD32VW55X_RTC_CTL, RTC_CTL_WTCS_MASK,
                      RTC_CTL_WTCS_DIV16);

          modifyreg32(GD32VW55X_EXTI_RTEN, 0, RTC_WKUP_EXTI_LINE);
          modifyreg32(GD32VW55X_EXTI_INTEN, 0, RTC_WKUP_EXTI_LINE);
          modifyreg32(GD32VW55X_RTC_CTL, 0, RTC_CTL_WTIE | RTC_CTL_WTEN);
        }
    }

  rtc_lock();
  return ret;
}

/****************************************************************************
 * Name: gd32vw55x_rtc_timeofday
 *
 * Description:
 *   Wait for the start of the next sub-second unit of the calendar, and
 *   return the time of day in these units, and their frequency in 'hz'.
 *   The time is right after deep-sleep, without waiting for the shadow
 *   registers.
 *
 ****************************************************************************/

uint32_t gd32vw55x_rtc_timeofday(uint32_t *hz)
{
  uint32_t sync = getreg32(GD32VW55X_RTC_PSC) & RTC_PSC_FACTOR_S_MASK;
  uint32_t start = getreg32(GD32VW55X_RTC_MTIME);
  uint32_t prevss;
  uint32_t prevtime;
  uint32_t ss;
  uint32_t time;
  int i;

  /* Read the calendar counters, not the shadow registers: these are only
   * updated every two RTC clocks, and not in deep-sleep.
   */

  rtc_unlock();
  modifyreg32(GD32VW55X_RTC_CTL, 0, RTC_CTL_BPSHAD);

  /* Wait for the next sub-second unit */

  prevss = getreg32(GD32VW55X_RTC_SS) & RTC_SS_MASK;
  while ((getreg32(GD32VW55X_RTC_SS) & RTC_SS_MASK) == prevss &&
         getreg32(GD32VW55X_RTC_MTIME) - start <= RTC_CALIB_TIMEOUT)
    {
    }

  /* The counters may change between two reads: read them until the same
   * values come twice.
   */

  ss   = getreg32(GD32VW55X_RTC_SS) & RTC_SS_MASK;
  time = getreg32(GD32VW55X_RTC_TIME);

  for (i = 0; i < 3; i++)
    {
      prevss   = ss;
      prevtime = time;
      ss       = getreg32(GD32VW55X_RTC_SS) & RTC_SS_MASK;
      time     = getreg32(GD32VW55X_RTC_TIME);

      if (ss == prevss && time == prevtime)
        {
          break;
        }
    }

  modifyreg32(GD32VW55X_RTC_CTL, RTC_CTL_BPSHAD, 0);
  rtc_lock();

  ss   = MIN(ss, sync);
  *hz  = sync + 1;

  return (BCD2DEC((time >> 16) & 0x3f) * SEC_PER_HOUR +
          BCD2DEC((time >> 8) & 0x7f) * SEC_PER_MIN +
          BCD2DEC(time & 0x7f)) * (sync + 1) + (sync - ss);
}

#endif /* CONFIG_RTC */
