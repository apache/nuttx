/****************************************************************************
 * include/nuttx/timers/cmsdk_wdt.h
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

#ifndef __INCLUDE_NUTTX_TIMERS_CMSDK_WDT_H
#define __INCLUDE_NUTTX_TIMERS_CMSDK_WDT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: cmsdk_wdt_initialize
 *
 * Description:
 *   Initialize the ARM CMSDK APB Watchdog and register it as a NuttX
 *   watchdog character device (e.g. /dev/watchdog0).
 *
 *   Naming: "CMSDK APB Watchdog" is the actual ARM IP name.  It is a
 *   component of the ARM Cortex-M System Design Kit (CMSDK), documented in
 *   its own right as "APB Watchdog" in the CMSDK Technical Reference Manual
 *   (ARM DDI 0479), and is the watchdog used by ARM's reference subsystems
 *   (SSE-xxx) and boards (MPS2/MPS3, Musca).  We use this name because it is
 *   the IP's own name and is how QEMU models the device (type
 *   "cmsdk-apb-watchdog", hw/watchdog/cmsdk-apb-watchdog.c) -- our target
 *   platform.  Its programmer's model is close to the older PrimeCell SP805
 *   watchdog, but this is a distinct, separately documented CMSDK component,
 *   not an SP805.
 *
 *   The device implements the CMSDK two-stage timeout: the first expiry
 *   raises the watchdog interrupt (early-warning); if the counter is not
 *   reloaded ("kicked") before the second expiry, a system reset is
 *   asserted.  The reset stage is only armed when the caller has NOT
 *   installed a capture handler via WDIOC_CAPTURE.
 *
 * Input Parameters:
 *   devpath - The full path to the device (e.g. "/dev/watchdog0").
 *   base    - Physical base address of the watchdog register block.
 *   irq     - Watchdog (WDOGINT) interrupt number.
 *   freq    - WDOGCLK input clock frequency in Hz.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int cmsdk_wdt_initialize(FAR const char *devpath, uintptr_t base,
                         int irq, uint32_t freq);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_TIMERS_CMSDK_WDT_H */
