/****************************************************************************
 * arch/x86_64/src/intel64/intel64_rtc.c
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
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/timers/rtc.h>
#include <nuttx/spinlock.h>
#include <arch/board/board.h>

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/param.h>
#include <time.h>

#include "x86_64_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* With CONFIG_RTC_HIRES, the RTC is a counter of the TSC.  Without it, the
 * RTC is the CMOS clock of the PC (MC146818).
 */

#ifdef CONFIG_RTC_HIRES
#  ifndef CONFIG_RTC_FREQUENCY
#    error "CONFIG_RTC_FREQUENCY is required for CONFIG_RTC_HIRES"
#  endif

/* The RTC divides TSC cycles by the TSC frequency (g_x86_64_timer_freq).
 * Only the TSC timers set it: with the HPET timer it is 0.
 */

#  ifndef CONFIG_ARCH_INTEL64_HAVE_TSC
#    error "CONFIG_RTC_HIRES needs a TSC system timer"
#  endif
#endif

#define NS_PER_USEC             1000UL
#define NS_PER_MSEC             1000000UL
#define NS_PER_SEC              1000000000UL

/* CMOS clock: I/O ports, registers and bits */

#define CMOS_ADDR               0x70
#define CMOS_DATA               0x71

#define CMOS_SECONDS            0x00
#define CMOS_MINUTES            0x02
#define CMOS_HOURS              0x04
#define CMOS_MDAY               0x07
#define CMOS_MONTH              0x08
#define CMOS_YEAR               0x09
#define CMOS_STATUS_A           0x0a
#define CMOS_STATUS_B           0x0b
#define CMOS_CENTURY            0x32

#define CMOS_A_UIP              0x80 /* Update in progress */
#define CMOS_B_SET              0x80 /* Updates stopped */
#define CMOS_B_DM               0x04 /* Binary, not BCD */
#define CMOS_B_24H              0x02 /* 24-hour mode */
#define CMOS_HOUR_PM            0x80 /* PM in 12-hour mode */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_RTC
static spinlock_t g_rtc_lock = SP_UNLOCKED;
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* drivers/timers/arch_rtc.c has it with CONFIG_RTC_ARCH */

#if defined(CONFIG_RTC) && !defined(CONFIG_RTC_ARCH)
volatile bool g_rtc_enabled = false;
#endif

#ifdef CONFIG_RTC_HIRES

/****************************************************************************
 * Private Functions
 ****************************************************************************/

extern unsigned long g_x86_64_timer_freq;
static unsigned long rtc_last;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static unsigned long rtc_read(void)
{
  uint64_t  tmr = rdtscp();
  irqstate_t flags;

  /* The 64-bit TSC does not wrap.  A smaller value comes from a CPU whose
   * TSC is behind the TSC of the CPU that read last: keep the last value.
   */

  flags = spin_lock_irqsave(&g_rtc_lock);
  if (tmr < rtc_last)
    {
      tmr = rtc_last;
    }

  rtc_last = tmr;
  spin_unlock_irqrestore(&g_rtc_lock, flags);

  return tmr / g_x86_64_timer_freq * NS_PER_SEC +
         tmr % g_x86_64_timer_freq * NS_PER_SEC / g_x86_64_timer_freq;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_rtc_initialize
 *
 * Description:
 *   Initialize the hardware RTC per the selected configuration.
 *   This function is called once during the OS initialization sequence.
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
  g_rtc_enabled = true;

  return OK;
}

/****************************************************************************
 * Name: up_rtc_gettime
 *
 * Description:
 *   Get the current time from the high resolution RTC clock/counter.
 *   This interface is only supported by the high-resolution RTC/counter
 *   hardware implementation.  It is used to replace the system timer.
 *
 * Input Parameters:
 *   tp - The location to return the high resolution time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_gettime(struct timespec *tp)
{
  uint64_t tmp;

  tmp = rtc_read();
  tp->tv_sec  = (tmp / NS_PER_SEC);
  tp->tv_nsec = (tmp - (tp->tv_sec) * NS_PER_SEC);

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

int up_rtc_settime(const struct timespec *tp)
{
  /* What so ever.. */

  return OK;
}

#elif defined(CONFIG_RTC)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint8_t cmos_read(uint8_t reg)
{
  outb(reg, CMOS_ADDR);
  return inb(CMOS_DATA);
}

static void cmos_write(uint8_t reg, uint8_t value)
{
  outb(reg, CMOS_ADDR);
  outb(value, CMOS_DATA);
}

static int cmos_frombcd(uint8_t value, bool binary)
{
  return binary ? value : (value >> 4) * 10 + (value & 0x0f);
}

static uint8_t cmos_tobcd(int value, bool binary)
{
  return binary ? value : ((value / 10) << 4) | (value % 10);
}

/****************************************************************************
 * Name: cmos_gettime
 *
 * Description:
 *   Read the date and the time (UTC) from the CMOS clock.  Read the
 *   registers when no update is in progress, and read them again until two
 *   reads are the same: an update can start during the reads.
 *
 ****************************************************************************/

static void cmos_gettime(struct tm *tp)
{
  static const uint8_t regidx[] =
  {
    CMOS_SECONDS, CMOS_MINUTES, CMOS_HOURS, CMOS_MDAY,
    CMOS_MONTH, CMOS_YEAR, CMOS_CENTURY
  };

  uint8_t regs[nitems(regidx)];
  uint8_t again[nitems(regidx)];
  irqstate_t flags;
  uint8_t statb;
  bool binary;
  int century;
  int hour;
  int i;

  flags = spin_lock_irqsave(&g_rtc_lock);
  statb = cmos_read(CMOS_STATUS_B);

  do
    {
      while ((cmos_read(CMOS_STATUS_A) & CMOS_A_UIP) != 0);

      for (i = 0; i < nitems(regidx); i++)
        {
          regs[i] = cmos_read(regidx[i]);
        }

      while ((cmos_read(CMOS_STATUS_A) & CMOS_A_UIP) != 0);

      for (i = 0; i < nitems(regidx); i++)
        {
          again[i] = cmos_read(regidx[i]);
        }
    }
  while (memcmp(regs, again, sizeof(regs)) != 0);

  spin_unlock_irqrestore(&g_rtc_lock, flags);

  binary = (statb & CMOS_B_DM) != 0;
  hour   = cmos_frombcd(regs[2] & ~CMOS_HOUR_PM, binary);
  if ((statb & CMOS_B_24H) == 0)
    {
      hour %= 12;
      if ((regs[2] & CMOS_HOUR_PM) != 0)
        {
          hour += 12;
        }
    }

  /* Not every PC has the century register: take 20 when it does not give
   * 19 or 20.
   */

  century = cmos_frombcd(regs[6], binary);
  if (century != 19 && century != 20)
    {
      century = 20;
    }

  memset(tp, 0, sizeof(*tp));
  tp->tm_sec  = cmos_frombcd(regs[0], binary);
  tp->tm_min  = cmos_frombcd(regs[1], binary);
  tp->tm_hour = hour;
  tp->tm_mday = cmos_frombcd(regs[3], binary);
  tp->tm_mon  = cmos_frombcd(regs[4], binary) - 1;
  tp->tm_year = century * 100 + cmos_frombcd(regs[5], binary) - 1900;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_rtc_initialize
 *
 * Description:
 *   Initialize the RTC.  The CMOS clock runs already.
 *
 * Returned Value:
 *   Zero (OK).
 *
 ****************************************************************************/

int up_rtc_initialize(void)
{
  g_rtc_enabled = true;
  return OK;
}

#ifdef CONFIG_RTC_DATETIME

/****************************************************************************
 * Name: up_rtc_getdatetime
 *
 * Description:
 *   Get the current date and time (UTC) from the CMOS clock.
 *
 * Input Parameters:
 *   tp - The location to return the date and time.
 *
 * Returned Value:
 *   Zero (OK).
 *
 ****************************************************************************/

int up_rtc_getdatetime(struct tm *tp)
{
  cmos_gettime(tp);
  return OK;
}

#else

/****************************************************************************
 * Name: up_rtc_time
 *
 * Description:
 *   Get the current time in seconds from the CMOS clock.
 *
 * Returned Value:
 *   The time in seconds since the epoch.
 *
 ****************************************************************************/

time_t up_rtc_time(void)
{
  struct tm tm;

  cmos_gettime(&tm);
  return timegm(&tm);
}

#endif

/****************************************************************************
 * Name: up_rtc_settime
 *
 * Description:
 *   Set the CMOS clock (UTC) to the provided time, in whole seconds.
 *
 * Input Parameters:
 *   tp - The time to use.
 *
 * Returned Value:
 *   Zero (OK).
 *
 ****************************************************************************/

int up_rtc_settime(const struct timespec *tp)
{
  irqstate_t flags;
  struct tm tm;
  uint8_t statb;
  uint8_t hour;
  bool binary;

  gmtime_r(&tp->tv_sec, &tm);

  flags  = spin_lock_irqsave(&g_rtc_lock);
  statb  = cmos_read(CMOS_STATUS_B);
  binary = (statb & CMOS_B_DM) != 0;

  if ((statb & CMOS_B_24H) != 0)
    {
      hour = cmos_tobcd(tm.tm_hour, binary);
    }
  else
    {
      hour = cmos_tobcd(tm.tm_hour % 12 == 0 ? 12 : tm.tm_hour % 12,
                        binary);
      if (tm.tm_hour >= 12)
        {
          hour |= CMOS_HOUR_PM;
        }
    }

  /* Stop the updates while the registers change */

  cmos_write(CMOS_STATUS_B, statb | CMOS_B_SET);
  cmos_write(CMOS_SECONDS, cmos_tobcd(tm.tm_sec, binary));
  cmos_write(CMOS_MINUTES, cmos_tobcd(tm.tm_min, binary));
  cmos_write(CMOS_HOURS, hour);
  cmos_write(CMOS_MDAY, cmos_tobcd(tm.tm_mday, binary));
  cmos_write(CMOS_MONTH, cmos_tobcd(tm.tm_mon + 1, binary));
  cmos_write(CMOS_YEAR, cmos_tobcd(tm.tm_year % 100, binary));
  cmos_write(CMOS_CENTURY, cmos_tobcd((tm.tm_year + 1900) / 100, binary));
  cmos_write(CMOS_STATUS_B, statb & ~CMOS_B_SET);

  spin_unlock_irqrestore(&g_rtc_lock, flags);
  return OK;
}
#endif
