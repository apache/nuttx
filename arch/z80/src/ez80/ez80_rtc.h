/****************************************************************************
 * arch/z80/src/ez80/ez80_rtc.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __ARCH_EZ80_SRC_EZ80_EZ80_RTC_H
#define __ARCH_EZ80_SRC_EZ80_EZ80_RTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <time.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RTC registers ************************************************************/

/* Provided in z80F91.h */

/* RTC register bit definitions *********************************************/

/* ACTRL register */

#define EZ80_RTC_ASECEN            (1 << 0)  /* Bit 0:  Enable seconds alarm */
#define EZ80_RTC_AMINEN            (1 << 1)  /* Bit 1:  Enable minutes alarm */
#define EZ80_RTC_AHRSEN            (1 << 2)  /* Bit 4:  Enable hours alarm */
#define EZ80_RTC_ADOWEN            (1 << 3)  /* Bit 3:  Enable day-of-week alarm */
#define EZ80_RTC_AALL              0x0f      /* All times */

/* CTRL register */

#define EZ80_RTC_UNLOCK            (1 << 0)  /* Bit 0:  RTC registers unlocked */
#define EZ80_RTC_SLPWAKE           (1 << 1)  /* Bit 1:  Alarm sleep recovery */
#define EZ80_RTC_FREQSEL           (1 << 3)  /* Bit 3:  Power line is 50Hz */
#define EZ80_RTC_CLKSEL            (1 << 4)  /* Bit 4:  Clock source is power line */
#define EZ80_RTC_BCDEN             (1 << 5)  /* Bit 5:  BCD encoded time */
#define EZ80_RTC_INTEN             (1 << 6)  /* Bit 6:  Alarm interrupt enabled */
#define EZ80_RTC_ALARM             (1 << 7)  /* Bit 7:  Alarm interrupt active */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

typedef CODE void (*alm_callback_t)(FAR void *arg);

/* Structure used to pass parameters to set an alarm */

struct alm_setalarm_s
{
  struct tm as_time;            /* Alarm expiration time */
  alm_callback_t as_cb;         /* Callback (if non-NULL) */
  FAR void *as_arg;             /* Argument for callback */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

struct tm;
int ez80_rtc_setdatetime(FAR const struct tm *tp);

/****************************************************************************
 * Name: ez80_rtc_lowerhalf
 *
 * Description:
 *   Instantiate the RTC lower half driver for the EZ80.  General usage:
 *
 *     #include <nuttx/timers/rtc.h>
 *     #include "ez80_rtc.h"
 *
 *     struct rtc_lowerhalf_s *lower;
 *     lower = ez80_rtc_lowerhalf();
 *     rtc_initialize(0, lower);
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, a non-NULL RTC lower interface is returned.  NULL is
 *   returned on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_DRIVER
struct rtc_lowerhalf_s;
FAR struct rtc_lowerhalf_s *ez80_rtc_lowerhalf(void);
#endif

#ifdef CONFIG_RTC_ALARM

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

int ez80_rtc_setalarm(FAR struct alm_setalarm_s *alminfo);

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

int ez80_rtc_rdalarm(FAR struct tm *almtime);

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

int ez80_rtc_cancelalarm(void);

#endif /* CONFIG_RTC_ALARM */
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_EZ80_SRC_EZ80_RTC_H */
