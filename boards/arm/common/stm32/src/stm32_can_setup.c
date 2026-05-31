/****************************************************************************
 * boards/arm/common/stm32/src/stm32_can_setup.c
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

#include <stdbool.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/can/can.h>

#include "stm32_can.h"

#ifdef CONFIG_CAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_STM32_CAN1
#  define CAN_PORT 1
#else
#  define CAN_PORT 2
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_can_setup
 *
 * Description:
 *  Initialize CAN and register the CAN device
 *
 ****************************************************************************/

int stm32_can_setup(void)
{
  struct can_dev_s *can;
  int ret;

  /* Register the first enabled CAN interface at "/dev/can0" */

  can = stm32_caninitialize(CAN_PORT);
  if (can == NULL)
    {
      canerr("ERROR: Failed to get CAN interface\n");
      return -ENODEV;
    }

  ret = can_register("/dev/can0", can);
  if (ret < 0)
    {
      canerr("ERROR: can_register failed: %d\n", ret);
      return ret;
    }

#if defined(CONFIG_STM32_CAN1) && defined(CONFIG_STM32_CAN2)
  /* Both CAN1 and CAN2 are enabled: register CAN2 at "/dev/can1" */

  can = stm32_caninitialize(2);
  if (can == NULL)
    {
      canerr("ERROR: Failed to get CAN interface\n");
      return -ENODEV;
    }

  ret = can_register("/dev/can1", can);
  if (ret < 0)
    {
      canerr("ERROR: can_register failed: %d\n", ret);
      return ret;
    }
#endif

  return OK;
}

#endif /* CONFIG_CAN */
