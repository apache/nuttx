/****************************************************************************
 * include/nuttx/timers/mcp794xx.h
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

#ifndef __INCLUDE_NUTTX_TIMERS_MCP794XX_H
#define __INCLUDE_NUTTX_TIMERS_MCP794XX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_RTC_MCP794XX

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
 * Name: mcp794xx_rtc_set_trim
 *
 * Description:
 *   Sets the digital trimming to correct for inaccuracies of clock source.
 *   Digital trimming consists of the MCP794XX periodically adding or
 *   subtracting clock cycles, resulting in small adjustments in the internal
 *   timing.
 *
 * Input Parameters:
 *   trim_val    - Calculated trimming value, refer to MCP794XX reference
 *                 manual.
 *   rtc_slow    - True indicates RTC is behind real clock, false otherwise.
 *                 This has to be set to ensure correct trimming direction.
 *   coarse_mode - MCP794XX allows coarse mode that trims every second
 *                 instead of every minute.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int mcp794xx_rtc_set_trim(uint8_t trim_val, bool rtc_slow, bool coarse_mode);

/****************************************************************************
 * Name: mcp794xx_rtc_initialize
 *
 * Description:
 *   Initialize the hardware RTC per the selected configuration.  This
 *   function is called once during the OS initialization sequence by board-
 *   specific logic.
 *
 *   After mcp794xx_rtc_initialize() is called, the OS function
 *   clock_synchronize() should also be called to synchronize the system
 *   timer to a hardware RTC.  That operation is normally performed
 *   automatically by the system during clock initialization.  However, when
 *   an external RTC is used, the board logic will need to explicitly re-
 *   synchronize the system timer to the RTC when the RTC becomes available.
 *
 * Input Parameters:
 *   i2c  - An instance of the I2C interface used to access the MCP794XX
 *          device
 *   addr - The (7-bit) I2C address of the MCP794XX device
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

struct i2c_master_s; /* Forward reference */
int mcp794xx_rtc_initialize(FAR struct i2c_master_s *i2c, uint8_t addr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_RTC_MCP794XX */
#endif /* __INCLUDE_NUTTX_TIMERS_MCP794XX_H */
