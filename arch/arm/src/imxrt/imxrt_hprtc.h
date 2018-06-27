/****************************************************************************
 * arch/arm/src/imxrt/imxrt_hprtc.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_HPRTC_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_HPRTC_H

#include <nuttx/config.h>

#include "chip.h"

#ifdef CONFIG_IMXRT_SNVS_HPRTC

/****************************************************************************
 * Preprocessor Definitions
 ****************************************************************************/

#ifdef CONFIG_RTC_DATETIME
#  error CONFIG_RTC_DATETIME should not be selected with this driver
#endif

#ifdef CONFIG_RTC_PERIODIC
#  error CONFIG_RTC_PERIODIC should not be selected with this driver
#endif

/* REVISIT: This is probably supportable.  The 47 bit timer does have
 * accuracy greater than 1 second.
 */

#ifdef CONFIG_RTC_HIRES
#  error CONFIG_RTC_PERIODIC should not be selected with this driver
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Callback type used by the HPRTC log to notify the RTC driver when the
 * alarm expires.
 */

typedef CODE void (*hprtc_alarm_callback_t)(void);

/****************************************************************************
 * Public Data
 ****************************************************************************/

#if !defined(CONFIG_IMXRT_SNVS_LPSRTC) && defined(CONFIG_RTC_DRIVER)
bool g_hprtc_timset;  /* True:  time has been set since power up */
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Functions used only for HPRTC
 ************************************************************************************/

/************************************************************************************
 * Logic Common to LPSRTC and HPRTC
 ************************************************************************************/

/****************************************************************************
 * Name: imxrt_rtc_lowerhalf
 *
 * Description:
 *   Instantiate the RTC lower half driver for the i.MXRT.  General usage:
 *
 *     #include <nuttx/timers/rtc.h>
 *     #include "imxrt_hprtc.h"
 *
 *     struct rtc_lowerhalf_s *lower;
 *     lower = imxrt_hprtc_lowerhalf();
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
FAR struct rtc_lowerhalf_s *imxrt_rtc_lowerhalf(void);
#endif

/************************************************************************************
 * Name: imxrt_hprtc_initialize
 *
 * Description:
 *   Initialize the LPSRTC per the selected configuration.  This function is called
 *   via up_rtc_initialize (see imxrt_hprtc.c).
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

int imxrt_hprtc_initialize(void);

/************************************************************************************
 * Name: imxrt_hprtc_synchronize
 *
 * Description:
 *   Synchronize the HPRTC to the LPSRTC and enable the HPRTC timer.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifdef CONFIG_IMXRT_SNVS_LPSRTC
void imxrt_hprtc_synchronize(void);
#endif

/************************************************************************************
 * Name: imxrt_hprtc_time
 *
 * Description:
 *   Get the current time in seconds.  This is the underlying implementation of the
 *   up_rtc_tim() function that is used by the RTOS during initialization to set up
 *   the system time.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current time in seconds
 *
 ************************************************************************************/

uint32_t imxrt_hprtc_time(void);

/************************************************************************************
 * Name: imxrt_hprtc_getalarm
 *
 * Description:
 *   Get the current alarm setting in seconds.  This is only used by the lower half
 *   RTC driver.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current alarm setting in seconds
 *
 ************************************************************************************/

#if defined(CONFIG_RTC_ALARM) && defined(CONFIG_RTC_DRIVER)
uint32_t imxrt_hprtc_getalarm(void);
#endif

/************************************************************************************
 * Name: imxrt_hprtc_setalarm
 *
 * Description:
 *   Set the alarm (in seconds) and enable alarm interrupts.  This is only used by
 *   the lower half RTC driver.
 *
 * Input Parameters:
 *   sec - The new alarm setting
 *
 * Returned Value:
 *   The current alarm setting in seconds
 *
 ************************************************************************************/

#if defined(CONFIG_RTC_ALARM) && defined(CONFIG_RTC_DRIVER)
int imxrt_hprtc_setalarm(FAR struct timespec *ts, hprtc_alarm_callback_t cb);
#endif

/************************************************************************************
 * Name: imxrt_hprtc_alarmdisable
 *
 * Description:
 *    Disable alarm interrupts.  Used internally after the receipt of the alarm
 *    interrupt.  Also called by the lower-half RTC driver in order to cancel an
 *    alarm.
 *
 * Input Parameters:
 *    None
 *
 * Returned Value:
 *    None
 *
 ************************************************************************************/

#if defined(CONFIG_RTC_ALARM) && defined(CONFIG_RTC_DRIVER)
void imxrt_hprtc_alarmdisable(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* CONFIG_IMXRT_SNVS_HPRTC */
#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT_HPRTC_H */
