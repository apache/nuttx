/****************************************************************************
 * include/nuttx/timers/pcf85063.h
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

#ifndef __INCLUDE_NUTTX_TIMERS_PCF85063_H
#define __INCLUDE_NUTTX_TIMERS_PCF85063_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_RTC_PCF85063

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: pcf85063_rtc_initialize
 *
 * Description:
 *   Bind the PCF85063 to an I2C bus and make it the system RTC.  This
 *   function is called once from board-specific logic, after the I2C bus
 *   has been initialized.
 *
 *   The chip is put in 24-hour mode and its oscillator is started if it
 *   was stopped.  The load capacitance selection and the correction
 *   interrupt enable in Control_1 are left as they were found.
 *
 *   After pcf85063_rtc_initialize() is called, the OS function
 *   clock_synchronize() should also be called to synchronize the system
 *   timer to the RTC.  That operation is normally performed automatically
 *   by the system during clock initialization, but an external RTC is not
 *   available that early, so the board logic must do it explicitly.
 *
 * Input Parameters:
 *   i2c - The I2C bus the chip is attached to
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure (for example when the
 *   chip does not respond on the bus).
 *
 ****************************************************************************/

struct i2c_master_s; /* Forward reference */
int pcf85063_rtc_initialize(FAR struct i2c_master_s *i2c);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_RTC_PCF85063 */
#endif /* __INCLUDE_NUTTX_TIMERS_PCF85063_H */
