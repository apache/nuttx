/****************************************************************************
 * sched/clock_initialize.c
 *
 *   Copyright (C) 2007, 2009, 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <time.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/clock.h>
#include <nuttx/time.h>
#include <nuttx/rtc.h>

#include "clock_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

#ifdef CONFIG_RTC
#  ifndef CONFIG_SYSTEM_UTC
#  error "In order to support hardware RTC system must have set the CONFIG_SYSTEM_UTC=y"
#  endif
#endif

/* Standard time definitions (in units of seconds) */

#define SEC_PER_MIN  ((time_t)60)
#define SEC_PER_HOUR ((time_t)60 * SEC_PER_MIN)
#define SEC_PER_DAY  ((time_t)24 * SEC_PER_HOUR)

/* Macro to increment the system timer -- or not */

#ifndef CONFIG_SYSTEM_UTC
#  define incr_systimer() g_system_timer++
#else
#  define incr_systimer()
#endif

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/**************************************************************************
 * Public Constant Data
 **************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

#if CONFIG_SYSTEM_UTC
volatile time_t  g_system_utc;
#else
volatile clock_t g_system_timer;
struct timespec  g_basetime;
uint32_t         g_tickbias;
#endif

/**************************************************************************
 * Private Variables
 **************************************************************************/

/* This variable is used to count ticks and to increment the one-second
 * UTC variable.
 */

#if CONFIG_SYSTEM_UTC
#if TICK_PER_SEC > 32767
volatile uint32_t g_tickcount;
#elif TICK_PER_SEC > 255
volatile uint16_t g_tickcount;
#else
volatile uint8_t  g_tickcount;
#endif
#endif /* CONFIG_SYSTEM_UTC */

/**************************************************************************
 * Private Functions
 **************************************************************************/
/****************************************************************************
 * Function: incr_utc
 *
 * Description:
 *   This function must be called once every time the real
 *   time clock interrupt occurs.  The interval of this
 *   clock interrupt must be MSEC_PER_TICK
 *
 ****************************************************************************/

#if CONFIG_SYSTEM_UTC
static inline void incr_utc(void)
{
  g_tickcount++;
  
  if (g_tickcount >= TICK_PER_SEC)
    {
      g_system_utc++;
      g_tickcount -= TICK_PER_SEC;
    }
}
#else
#  define incr_utc()
#endif

/****************************************************************************
 * Function: clock_inittime
 *
 * Description:
 *   Get the initial time value from the best source available.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC
#ifdef CONFIG_RTC_HIRES

static inline void clock_inittime(FAR struct timespec *tp)
{

  /* Get the complete time from the hi-res RTC. */

  (void)up_rtc_gettime(tp);
}

#else

static inline void clock_inittime(FAR struct timespec *tp)
{
  /* Get the seconds (only) from the lo-res RTC */

  tp->tv_sec  = up_rtc_time();
  tp->tv_nsec = 0;
}

#endif /* CONFIG_RTC_HIRES */
#else /* CONFIG_RTC */

static inline void clock_inittime(FAR struct timespec *tp)
{
  time_t jdn = 0;

  /* Get the EPOCH-relative julian date from the calendar year,
   * month, and date
   */

  jdn = clock_calendar2utc(CONFIG_START_YEAR, CONFIG_START_MONTH,
                           CONFIG_START_DAY);

  /* Set the base time as seconds into this julian day. */

  tp->tv_sec  = jdn * SEC_PER_DAY;
  tp->tv_nsec = 0;
}

#endif /* CONFIG_RTC */


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: clock_initialize
 *
 * Description:
 *   Perform one-time initialization of the timing facilities.
 *
 ****************************************************************************/

void clock_initialize(void)
{
#ifdef CONFIG_SYSTEM_UTC
  struct timespec ts;
#endif

  /* Initialize the RTC hardware */

#ifdef CONFIG_RTC
  up_rtcinitialize();
#endif

  /* Initialize the time value */

#ifdef CONFIG_SYSTEM_UTC
  clock_inittime(&ts);
  g_system_utc = ts.tv_sec;
  g_tickcount  = ((ts.tv_nsec > 10) * CLOCKS_PER_SEC) / (1000000000 >> 10);
#else
  clock_inittime(&g_basetime);
  g_system_timer = 0;
  g_tickbias     = 0;
#endif
}

/****************************************************************************
 * Function: clock_timer
 *
 * Description:
 *   This function must be called once every time the real
 *   time clock interrupt occurs.  The interval of this
 *   clock interrupt must be MSEC_PER_TICK
 *
 ****************************************************************************/

void clock_timer(void)
{
  /* Increment the per-tick system counter */

  incr_systimer();

  /* Increment the per-second UTC counter */

  incr_utc();
}
