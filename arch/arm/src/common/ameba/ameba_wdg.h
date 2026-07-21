/****************************************************************************
 * arch/arm/src/common/ameba/ameba_wdg.h
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

#ifndef __ARCH_ARM_SRC_COMMON_AMEBA_AMEBA_WDG_H
#define __ARCH_ARM_SRC_COMMON_AMEBA_AMEBA_WDG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

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
 * Name: ameba_wdg_initialize
 *
 * Description:
 *   Instantiate the Ameba watchdog lower half and bind it to the NuttX
 *   watchdog character driver at CONFIG_WATCHDOG_DEVPATH ("/dev/watchdog0"
 *   by default).  General usage:
 *
 *     #include "ameba_wdg.h"
 *     ameba_wdg_initialize();
 *
 *   The watchdog is registered in the stopped state (nothing counts until
 *   the application issues WDIOC_SETTIMEOUT + WDIOC_START).
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ameba_wdg_initialize(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_COMMON_AMEBA_AMEBA_WDG_H */
