/****************************************************************************
 * arch/x86_64/src/intel64/intel64_rtc.c
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
#include <arch/board/board.h>

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <errno.h>

#include "x86_64_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* This is a hacky implementation based on TSC, we only support Hi-RES mode */

#ifdef CONFIG_RTC_HIRES
#  ifndef CONFIG_RTC_FREQUENCY
#    error "CONFIG_RTC_FREQUENCY is required for CONFIG_RTC_HIRES"
#  endif
#endif

#define NS_PER_USEC             1000UL
#define NS_PER_MSEC             1000000UL
#define NS_PER_SEC              1000000000UL

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_RTC_HIRES
volatile bool g_rtc_enabled = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static unsigned long rtc_freq;
static unsigned long rtc_overflow;
static unsigned long rtc_last;
static unsigned long rtc_overflows;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static unsigned long rtc_read(void)
{
  uint64_t  tmr = rdtsc();

  if (tmr < rtc_last)
    {
      tmr += (0xffffffffffffffffull - rtc_last);
    }

  rtc_last = tmr;
  tmr = (tmr / (rtc_freq / 1000000ul)) * 1000l;
  return tmr;
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
  rtc_freq = comm_region->tsc_khz * 1000L;
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
#endif
