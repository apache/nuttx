/****************************************************************************
 * include/nuttx/timers/pcf8563.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __INCLUDE_NUTTX_TIMERS_PCF8563_H
#define __INCLUDE_NUTTX_TIMERS_PCF8563_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>

#ifdef CONFIG_RTC_PCF8563

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The address is fixed in the part and cannot be strapped. */

#define PCF8563_I2C_ADDRESS  0x51

/****************************************************************************
 * Public Function Prototypes
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
 * Name: pcf8563_rtc_initialize
 *
 * Description:
 *   Bind the driver to a bus and take over as the system's clock.  Called
 *   once from board logic during start up.
 *
 *   Call clock_synchronize() afterwards.  The system timer is started from
 *   whatever the architecture could tell it before this driver existed,
 *   which for a board whose only clock is on a bus is nothing at all, so
 *   the time does not become right until something asks for it to be
 *   copied across.
 *
 * Input Parameters:
 *   i2c - The bus the chip is on
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure.
 *
 ****************************************************************************/

int pcf8563_rtc_initialize(FAR struct i2c_master_s *i2c);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_RTC_PCF8563 */
#endif /* __INCLUDE_NUTTX_TIMERS_PCF8563_H */
