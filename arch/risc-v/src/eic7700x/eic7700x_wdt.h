/****************************************************************************
 * arch/risc-v/src/eic7700x/eic7700x_wdt.h
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

#ifndef __ARCH_RISCV_SRC_EIC7700X_EIC7700X_WDT_H
#define __ARCH_RISCV_SRC_EIC7700X_EIC7700X_WDT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__

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
 * Name: eic7700x_wdt_initialize
 *
 * Description:
 *   Bring up one of the four watchdogs and register it with the
 *   framework.  With the auto-monitor configured this also arms it: the
 *   framework starts every watchdog it is handed and feeds it from a
 *   kernel timer until an application claims it.
 *
 * Input Parameters:
 *   n       - Which instance, 0 to 3.
 *   devpath - The node to register, such as "/dev/watchdog0".
 *
 * Returned Value:
 *   Zero on success; a negated errno if the clock, interrupt or
 *   registration fails.
 *
 ****************************************************************************/

int eic7700x_wdt_initialize(int n, FAR const char *devpath);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_EIC7700X_EIC7700X_WDT_H */
