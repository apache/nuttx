/****************************************************************************
 * include/nuttx/rtc.h
 *
 *   Copyright(C) 2011 Uros Platise. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu>
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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_RTC_H
#define __INCLUDE_NUTTX_RTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/clock.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RTC_CLOCKS_PER_SEC      16384
#define RTC_CLOCKS_SHIFT        14

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/* Variable determines the state of the RTC module.
 *
 * After initialization value is set to 'true' if RTC starts successfully.
 * The value can be changed to false also during operation if RTC for
 * some reason fails.
 */

extern volatile bool g_rtc_enabled;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: up_rtcinitialize
 *
 * Description:
 *   Initialize the periodic timer interface. This function is called once
 *   from the clock_initialize() function.
 *
 * Returned Value:
 *   Returns OK if RTC has successfully started, otherwise ERROR.
 *
 ****************************************************************************/

EXTERN int up_rtcinitialize(void);
EXTERN int up_rtcinitialize(void);

EXTERN clock_t up_rtc_getclock(void);
EXTERN void up_rtc_setclock(clock_t clock);

EXTERN time_t up_rtc_gettime(void);
EXTERN void up_rtc_settime(time_t time);

EXTERN clock_t up_rtc_setalarm(clock_t atclock);

/* This callback is provided by the clock module and called by the RTC ISR */

EXTERN void clock_rtcalarmcb(clock_t clock);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_RTC_H */
