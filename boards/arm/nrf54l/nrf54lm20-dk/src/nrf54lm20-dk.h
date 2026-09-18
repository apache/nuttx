/****************************************************************************
 * boards/arm/nrf54l/nrf54lm20-dk/src/nrf54lm20-dk.h
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

#ifndef __BOARDS_ARM_NRF54L_NRF54LM20_DK_SRC_NRF54LM20_DK_H
#define __BOARDS_ARM_NRF54L_NRF54LM20_DK_SRC_NRF54LM20_DK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include "nrf54l_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define NRF54L_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define NRF54L_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/* LED definitions **********************************************************/

/* Definitions to configure LED GPIO as outputs */

#define GPIO_LED1 (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT1 | GPIO_PIN(22))
#define GPIO_LED2 (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT1 | GPIO_PIN(25))
#define GPIO_LED3 (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT1 | GPIO_PIN(27))
#define GPIO_LED4 (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT1 | GPIO_PIN(28))

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_bringup
 *
 * Description:
 *   Perform board-specific initialization. Called from
 *   board_late_initialize() when CONFIG_BOARD_LATE_INITIALIZE is selected.
 *
 ****************************************************************************/

int nrf54l_bringup(void);

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_NRF54L_NRF54LM20_DK_SRC_NRF54LM20_DK_H */
