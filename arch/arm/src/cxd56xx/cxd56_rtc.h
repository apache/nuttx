/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_rtc.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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

#ifndef __ARCH_ARM_SRC_CXD56XX_CXD56_RTC_H
#define __ARCH_ARM_SRC_CXD56XX_CXD56_RTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <time.h>
#include <nuttx/timers/rtc.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef CONFIG_RTC_ALARM

/* The form of an alarm callback */

typedef CODE void (*alm_callback_t)(FAR void *arg, unsigned int alarmid);

enum alm_id_e
{
  RTC_ALARM0 = 0,           /* RTC ALARM 0 */
  RTC_ALARM_LAST,
  RTC_ALARM1 = 1,           /* RTC ALARM 1 */
  RTC_ALARM2,               /* RTC ALARM 2 (relative) */
};

/* Structure used to pass parameters to set an alarm */

struct alm_setalarm_s
{
  int             as_id;    /* enum alm_id_e */
  struct timespec as_time;  /* Alarm expiration time */
  alm_callback_t  as_cb;    /* Callback (if non-NULL) */
  FAR void       *as_arg;   /* Argument for callback */
};

#endif /* CONFIG_RTC_ALARM */

/****************************************************************************
 * Public Data
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
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_rtc_count
 *
 * Description:
 *   Get RTC raw counter value
 *
 * Returned Value:
 *   64bit counter value running at 32kHz
 *
 ****************************************************************************/

uint64_t cxd56_rtc_count(void);

/****************************************************************************
 * Name: cxd56_rtc_almcount
 *
 * Description:
 *   Get RTC raw alarm counter value
 *
 * Returned Value:
 *   64bit alarm counter value running at 32kHz
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
uint64_t cxd56_rtc_almcount(void);
#endif /* CONFIG_RTC_ALARM */

/****************************************************************************
 * Name: cxd56_rtc_setalarm
 *
 * Description:
 *   Set up an alarm.
 *
 * Input Parameters:
 *   alminfo - Information about the alarm configuration.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
int cxd56_rtc_setalarm(FAR struct alm_setalarm_s *alminfo);
#endif /* CONFIG_RTC_ALARM */

/****************************************************************************
 * Name: cxd56_rtc_cancelalarm
 *
 * Description:
 *   Cancel an alaram.
 *
 * Input Parameters:
 *  alarmid - Identifies the alarm to be cancelled
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
int cxd56_rtc_cancelalarm(enum alm_id_e alarmid);
#endif /* CONFIG_RTC_ALARM */

/****************************************************************************
 * Name: cxd56_rtc_lowerhalf
 *
 * Description:
 *   Instantiate the RTC lower half driver for the STM32L4.  General usage:
 *
 *     #include <nuttx/timers/rtc.h>
 *     #include "stm32l4_rtc.h>
 *
 *     struct rtc_lowerhalf_s *lower;
 *     lower = stm32l4_rtc_lowerhalf();
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
FAR struct rtc_lowerhalf_s *cxd56_rtc_lowerhalf(void);
#endif /* CONFIG_RTC_DRIVER */

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_CXD56XX_CXD56_RTC_H */
