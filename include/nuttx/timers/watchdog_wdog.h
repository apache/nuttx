/****************************************************************************
 * include/nuttx/timers/watchdog_wdog.h
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

#ifndef __INCLUDE_NUTTX_TIMERS_WATCHDOG_WDOG_H
#define __INCLUDE_NUTTX_TIMERS_WATCHDOG_WDOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

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

#ifdef CONFIG_WATCHDOG_WDOG
/****************************************************************************
 * Name: watchdog_wdog_initialize
 *
 * Description:
 *   Register a watchdog timer implemented by a wdog timer, for the
 *   platforms that have no watchdog hardware (e.g. the simulator). Like any
 *   watchdog it expires if it is not fed within its timeout, but since
 *   there is no hardware to reset the system it does what
 *   CONFIG_WATCHDOG_WDOG_ACTION selects (panic, reboot or power off). It
 *   does not fire if the system tick stops.
 *
 * Input Parameters:
 *   path - The device path, e.g. CONFIG_WATCHDOG_DEVPATH. The initial
 *          timeout is CONFIG_WATCHDOG_WDOG_TIMEOUT.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int watchdog_wdog_initialize(FAR const char *path);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_TIMERS_WATCHDOG_WDOG_H */
