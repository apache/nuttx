/****************************************************************************
 * arch/arm/src/stm32l4/stm32l4_rtc.h
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

#ifndef __ARCH_ARM_SRC_STM32L4_STM32L4_RTC_H
#define __ARCH_ARM_SRC_STM32L4_STM32L4_RTC_H

#include <time.h>
#include <nuttx/config.h>

#include "chip.h"

#include "hardware/stm32l4_rtcc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STM32L4_RTC_PRESCALER_SECOND         32767  /* Default prescaler to get a second base */
#define STM32L4_RTC_PRESCALER_MIN             1     /* Maximum speed of 16384 Hz */

#if !defined(CONFIG_STM32L4_RTC_MAGIC)
# define CONFIG_STM32L4_RTC_MAGIC           (0xfacefeee)
#endif

#if !defined(CONFIG_STM32L4_RTC_MAGIC_TIME_SET)
#  define CONFIG_STM32L4_RTC_MAGIC_TIME_SET (0xf00dface)
#endif

#if !defined(CONFIG_STM32L4_RTC_MAGIC_REG)
# define CONFIG_STM32L4_RTC_MAGIC_REG       (0)
#endif

#define RTC_MAGIC                           CONFIG_STM32L4_RTC_MAGIC
#define RTC_MAGIC_TIME_SET                  CONFIG_STM32L4_RTC_MAGIC_TIME_SET
#define RTC_MAGIC_REG                       STM32L4_RTC_BKR(CONFIG_STM32L4_RTC_MAGIC_REG)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef CONFIG_RTC_ALARM

/* The form of an alarm callback */

typedef void (*alm_callback_t)(void *arg, unsigned int alarmid);

enum alm_id_e
{
  RTC_ALARMA = 0,               /* RTC ALARM A */
  RTC_ALARMB,                   /* RTC ALARM B */
  RTC_ALARM_LAST
};

/* Structure used to pass parameters to set an alarm */

struct alm_setalarm_s
{
  int as_id;                    /* enum alm_id_e */
  struct tm as_time;            /* Alarm expiration time */
  alm_callback_t as_cb;         /* Callback (if non-NULL) */
  void *as_arg;                 /* Argument for callback */
};

/* Structure used to pass parameters to query an alarm */

struct alm_rdalarm_s
{
  int ar_id;                    /* enum alm_id_e */
  struct rtc_time *ar_time;     /* Argument for storing ALARM RTC time */
};

#endif /* CONFIG_RTC_ALARM */

#ifdef CONFIG_RTC_PERIODIC
typedef int (*wakeupcb_t)(void);
#endif

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
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4_rtc_is_initialized
 *
 * Description:
 *    Returns 'true' if the RTC has been initialized
 *    Returns 'false' if the RTC has never been initialized since first time
 *    power up, and the counters are stopped until it is first initialized.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Returns true if RTC has been initialized.
 *
 ****************************************************************************/

bool stm32l4_rtc_is_initialized(void);

/****************************************************************************
 * Name: stm32l4_rtc_getdatetime_with_subseconds
 *
 * Description:
 *   Get the current date and time from the date/time RTC.  This interface
 *   is only supported by the date/time RTC hardware implementation.
 *   It is used to replace the system timer.  It is only used by the RTOS
 *   during initialization to set up the system time when CONFIG_RTC and
 *   CONFIG_RTC_DATETIME are selected (and CONFIG_RTC_HIRES is not).
 *
 *   NOTE: The sub-second accuracy is returned through 'nsec'.
 *
 * Input Parameters:
 *   tp - The location to return the high resolution time value.
 *   nsec - The location to return the subsecond time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_STM32L4_HAVE_RTC_SUBSECONDS
int stm32l4_rtc_getdatetime_with_subseconds(struct tm *tp,
                                            long *nsec);
#endif

/****************************************************************************
 * Name: stm32l4_rtc_setdatetime
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

#ifdef CONFIG_RTC_DATETIME
struct tm;
int stm32l4_rtc_setdatetime(const struct tm *tp);
#endif

/****************************************************************************
 * Name: stm32l4_rtc_havesettime
 *
 * Description:
 *   Check if RTC time has been set.
 *
 * Returned Value:
 *   Returns true if RTC date-time have been previously set.
 *
 ****************************************************************************/

bool stm32l4_rtc_havesettime(void);

#ifdef CONFIG_RTC_ALARM
/****************************************************************************
 * Name: stm32l4_rtc_setalarm
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

int stm32l4_rtc_setalarm(struct alm_setalarm_s *alminfo);

/****************************************************************************
 * Name: stm32l4_rtc_rdalarm
 *
 * Description:
 *   Query an alarm configured in hardware.
 *
 * Input Parameters:
 *  alminfo - Information about the alarm configuration.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int stm32l4_rtc_rdalarm(struct alm_rdalarm_s *alminfo);

/****************************************************************************
 * Name: stm32l4_rtc_cancelalarm
 *
 * Description:
 *   Cancel an alarm.
 *
 * Input Parameters:
 *  alarmid - Identifies the alarm to be cancelled
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int stm32l4_rtc_cancelalarm(enum alm_id_e alarmid);
#endif /* CONFIG_RTC_ALARM */

#ifdef CONFIG_RTC_PERIODIC

/****************************************************************************
 * Name: stm32l4_rtc_setperiodic
 *
 * Description:
 *   Set a periodic RTC wakeup
 *
 * Input Parameters:
 *  period   - Time to sleep between wakeups
 *  callback - Function to call when the period expires.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int stm32l4_rtc_setperiodic(const struct timespec *period,
                            wakeupcb_t callback);

/****************************************************************************
 * Name: stm32l4_rtc_cancelperiodic
 *
 * Description:
 *   Cancel a periodic wakeup
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int stm32l4_rtc_cancelperiodic(void);
#endif /* CONFIG_RTC_PERIODIC */

/****************************************************************************
 * Name: stm32l4_rtc_lowerhalf
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
struct rtc_lowerhalf_s;
struct rtc_lowerhalf_s *stm32l4_rtc_lowerhalf(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32L4_STM32L4_RTC_H */
