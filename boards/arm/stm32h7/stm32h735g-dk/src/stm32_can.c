/****************************************************************************
 * boards/arm/stm32h7/stm32h735g-dk/src/stm32_can.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdio.h>
#include <nuttx/debug.h>

#include <nuttx/can/can.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "stm32_fdcan.h"
#include "stm32h735g-dk.h"

#ifdef CONFIG_STM32_FDCAN_CHARDRIVER

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_can_setup
 *
 * Description:
 *  Initialize FDCAN port and register it as /dev/can<port - 1>
 *
 ****************************************************************************/

int stm32_can_setup(uint8_t port)
{
  struct can_dev_s *can;
  char devpath[12];

  /* Call stm32_fdcaninitialize() to get an instance of the CAN interface */

  can = stm32_fdcaninitialize(port);
  if (can == NULL)
    {
      canerr("ERROR:  Failed to get CAN interface\n");
      return -ENODEV;
    }

  snprintf(devpath, sizeof(devpath), "/dev/can%d", port - 1);
  return can_register(devpath, can);
}

#endif /* CONFIG_STM32_FDCAN_CHARDRIVER */
