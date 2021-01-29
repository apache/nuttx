/****************************************************************************
 * include/nuttx/timers/ds3231.h
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

#ifndef __INCLUDE_NUTTX_TIMERS_DS3231_H
#define __INCLUDE_NUTTX_TIMERS_DS3231_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_RTC_DSXXXX

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: dsxxxx_rtc_initialize
 *
 * Description:
 *   Initialize the hardware RTC per the selected configuration.
 *   This function is called once during the OS initialization sequence by
 *   board-specific logic.
 *
 *   After dsxxxx_rtc_initialize() is called, the OS function
 *   clock_synchronize() should also be called to synchronize the system
 *   timer to the hardware RTC.  That operation is normally performed
 *   automatically by the system during clock initialization.
 *   However, when an external RTC is used, the board logic will need to
 *   explicitly re-synchronize the system timer to the RTC when the RTC
 *   becomes available.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

struct i2c_master_s; /* Forward reference */
int dsxxxx_rtc_initialize(FAR struct i2c_master_s *i2c);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_RTC_DSXXXX */
#endif /* __INCLUDE_NUTTX_TIMERS_DS3231_H */
