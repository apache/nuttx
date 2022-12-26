/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_rtc.h
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

#ifndef __ARCH_RENESAS_SRC_RX65N_RX65N_RTC_H
#define __ARCH_RENESAS_SRC_RX65N_RX65N_RTC_H

#include <nuttx/config.h>

#include "chip.h"
#include "rx65n_definitions.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef CONFIG_RTC_ALARM

/* The form of an alarm callback */

typedef void (*alm_callback_t)(void *arg, unsigned int alarmid);

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
typedef int (*periodiccb_t)(void *arg, unsigned int alarmid);
#endif

#ifdef CONFIG_RX65N_CARRY
typedef void (*carrycb_t)(void);
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
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rx65n_rtc_setdatetime
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
int rx65n_rtc_setdatetime(const struct tm *tp);
#endif

/****************************************************************************
 * Name: rx65n_rtc_havesettime
 *
 * Description:
 *   Check if RTC time has been set.
 *
 * Returned Value:
 *   Returns true if RTC date-time have been previously set.
 *
 ****************************************************************************/

bool rx65n_rtc_havesettime(void);

#ifdef CONFIG_RTC_ALARM
/****************************************************************************
 * Name: rx65n_rtc_setalarm
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

int rx65n_rtc_setalarm(struct alm_setalarm_s *alminfo);

/****************************************************************************
 * Name: rx65n_rtc_rdalarm
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

int rx65n_rtc_rdalarm(struct alm_rdalarm_s *alminfo);

/****************************************************************************
 * Name: rx65n_rtc_cancelalarm
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

int rx65n_rtc_cancelalarm(void);
#endif /* CONFIG_RTC_ALARM */

#ifdef CONFIG_RTC_PERIODIC

/****************************************************************************
 * Name: rx65n_rtc_setperiodic
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

int rx65n_rtc_setperiodic(const struct timespec *period,
                          periodiccb_t callback);

/****************************************************************************
 * Name: rx65n_rtc_cancelperiodic
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

int rx65n_rtc_cancelperiodic(void);
#endif /* CONFIG_RTC_PERIODIC */

/****************************************************************************
 * Name: rx65n_rtc_lowerhalf
 *
 * Description:
 *   Instantiate the RTC lower half driver for the rx65n.  General usage:
 *
 *     #include <nuttx/timers/rtc.h>
 *     #include "rx65n_rtc.h>
 *
 *     struct rtc_lowerhalf_s *lower;
 *     lower = rx65n_rtc_lowerhalf();
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
struct rtc_lowerhalf_s *rx65n_rtc_lowerhalf(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RENESAS_SRC_RX65N_RX65N_RTC_H */
