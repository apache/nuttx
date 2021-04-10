/****************************************************************************
 * arch/arm/src/imxrt/imxrt_wdog.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_WDOG_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_WDOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/imxrt_wdog.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(CONFIG_WATCHDOG) && defined(CONFIG_IMXRT_WDOG)

/****************************************************************************
 * Name: imxrt_wdog_initialize
 *
 * Description:
 *   Initialize the watchdog time.  The watchdog timer is initialized and
 *   registered at devpath.  The initial state of the watchdog time is
 *   disabled.
 *
 * Input Parameters:
 *   devpath - The full path to the watchdog.  This should be of the form
 *     /dev/watchdog0
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

void imxrt_wdog_initialize(void);

#endif /* CONFIG_WATCHDOG && CONFIG_IMXRT_WDOG */

/****************************************************************************
 * Name: imxrt_wdog_disable
 *
 * Description:
 *   Called at the very beginning of _start.  Disables all watchdogs
 *
 ****************************************************************************/

void imxrt_wdog_disable_all(void);

#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT_WDOG_H */
