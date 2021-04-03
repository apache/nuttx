/****************************************************************************
 * arch/arm/src/am335x/am335x_wdog.h
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

#ifndef __ARCH_ARM_SRC_AM335X_AM335X_WDOG_H
#define __ARCH_ARM_SRC_AM335X_AM335X_WDOG_H

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
#include "hardware/am335x_wdog.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: am335x_wdog_disable
 *
 * Description:
 *   Called at the very beginning of _start.  Disables all watchdogs
 *
 ****************************************************************************/

void am335x_wdog_disable_all(void);

#endif /* __ARCH_ARM_SRC_AM335X_AM335X_WDOG_H */
