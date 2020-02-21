/************************************************************************************
 * arch/arm/src/stm32/stm32_rtcounter.c
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu>
 *
 * With extensions, modifications by:
 *
 *   Copyright (C) 2011-2013, 2015, 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

/* The STM32 RTC Driver offers standard precision of 1 Hz or High Resolution
 * operating at rate up to 16384 Hz. It provides UTC time and alarm interface
 * with external output pin (for wake-up).
 *
 * RTC is based on hardware RTC module which is located in a separate power
 * domain. The 32-bit counter is extended by 16-bit registers in BKP domain
 * STM32_BKP_DR1 to provide system equiv. function to the: time_t time(time_t *).
 *
 * Notation:
 *  - clock refers to 32-bit hardware counter
 *  - time is a combination of clock and upper bits stored in backuped domain
 *    with unit of 1 [s]
 *
 * TODO: Error Handling in case LSE fails during start-up or during operation.
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

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

#include "up_arch.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/
/* This is a hacky implementation based on TSC, we only support Hi-RES mode
 */

#ifdef CONFIG_RTC_HIRES
#  ifndef CONFIG_RTC_FREQUENCY
#    error "CONFIG_RTC_FREQUENCY is required for CONFIG_RTC_HIRES"
#  endif
#endif

#define NS_PER_USEC		1000UL
#define NS_PER_MSEC		1000000UL
#define NS_PER_SEC		1000000000UL

/************************************************************************************
 * Private Types
 ************************************************************************************/

/************************************************************************************
 * Private Data
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifdef CONFIG_RTC_HIRES
volatile bool g_rtc_enabled = false;

/************************************************************************************
 * Private Functions
 ************************************************************************************/

static unsigned long rtc_freq, rtc_overflow;
static unsigned long rtc_last;
static unsigned long rtc_overflows;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static unsigned long rtc_read(void)
{
	uint64_t  tmr = rdtsc();

	if (tmr < rtc_last)
        tmr += (0xffffffffffffffffULL - rtc_last);

	rtc_last = tmr;
    tmr = (tmr / (rtc_freq / 1000000UL)) * 1000L;
	return tmr;
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
  rtc_freq = comm_region->tsc_khz * 1000L;
  g_rtc_enabled = true;

  return OK;
}

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

int up_rtc_gettime(FAR struct timespec *tp)
{
  uint64_t tmp;

  tmp = rtc_read();
  tp->tv_sec  = (tmp / NS_PER_SEC);
  tp->tv_nsec = (tmp - (tp->tv_sec) * NS_PER_SEC);

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
  //What so ever..
  return OK;
}
#endif
