/****************************************************************************
 * arch/arm/src/kinetis/kinetis_alarm.h
 *
 *   Copyright (C) 2016-2017 Gregory Nutt. All rights reserved.
 *   Author:  Matias v01d <phreakuencies@gmail.com>
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

#ifndef __ARCH_ARM_SRC_KINETIS_ALARM_H
#define __ARCH_ARM_SRC_KINETIS_ALARM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#ifdef CONFIG_RTC_ALARM

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* The form of an alarm callback */

typedef CODE void (*alarmcb_t)(void);

/* These features are in KinetisK 1st generation
 * Time Alarm Interrupt
 * Time Overflow Interrupt
 * Time Seconds Interrupt
 *
 * For KinetisK 2nd Generation devices
 * 64bit Monotonic  register.
 */

enum alm_id_e
{
  /* Used for indexing - must be sequential */

  RTC_ALARMA = 0,     /* RTC ALARM A */
  RTC_ALARMM,         /* FUT: RTC Monotonic */
  RTC_ALARM_LAST
};

/* Structure used to pass parameters to set an alarm */

struct alm_setalarm_s
{
  int as_id;          /* enum alm_id_e */
  struct tm as_time;  /* Alarm expiration time */
  alarmcb_t as_cb;    /* Callback (if non-NULL) */
  FAR void *as_arg;   /* Argument for callback */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: kinetis_rtc_setalarm
 *
 * Description:
 *   Set up an alarm.
 *
 * Input Parameters:
 *   tp - the time to set the alarm
 *   callback - the function to call when the alarm expires.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int kinetis_rtc_setalarm(FAR const struct timespec *tp, alarmcb_t callback);

/************************************************************************************
 * Name: kinetis_rtc_rdalarm
 *
 * Description:
 *   Query an alarm configured in hardware.
 *
 * Input Parameters:
 *  tp - Location to return the timer match register.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

int kinetis_rtc_rdalarm(FAR struct timespec *tp);

/****************************************************************************
 * Name: kinetis_rtc_cancelalarm
 *
 * Description:
 *   Cancel a pending alarm alarm
 *
 * Input Parameters:
 *   none
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int kinetis_rtc_cancelalarm(void);

/****************************************************************************
 * Name: kinetis_rtc_lowerhalf
 *
 * Description:
 *   Instantiate the RTC lower half driver for the Kinetis.  General usage:
 *
 *     #include <nuttx/timers/rtc.h>
 *     #include "kinetis_rtc.h>
 *
 *     struct rtc_lowerhalf_s *lower;
 *     lower = kinetis_rtc_lowerhalf();
 *     rtc_initialize(0, lower);
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, a non-NULL RTC lower interface is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

FAR struct rtc_lowerhalf_s *kinetis_rtc_lowerhalf(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_RTC_ALARM */
#endif /* __ARCH_ARM_SRC_KINETIS_ALARM_H */
