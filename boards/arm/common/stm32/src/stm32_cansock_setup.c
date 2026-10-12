/****************************************************************************
 * boards/arm/common/stm32/src/stm32_cansock_setup.c
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

#include <nuttx/debug.h>

#include <arch/board/board.h>

#ifdef BOARD_CAN_GPIO_STBY
#  include "stm32_gpio.h"
#endif
#ifdef CONFIG_STM32_FDCAN_SOCKET
#  include "stm32_fdcan.h"
#else
#  include "stm32_can.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_STM32_FDCAN_SOCKET
#  define CANSOCK_INITIALIZE(port) stm32_fdcansockinitialize(port)
#else
#  define CANSOCK_INITIALIZE(port) stm32_cansockinitialize(port)
#endif

#if !defined(CONFIG_STM32_CAN1) && !defined(CONFIG_STM32_CAN2) && \
    !defined(CONFIG_STM32_CAN3) && !defined(CONFIG_STM32_FDCAN1) && \
    !defined(CONFIG_STM32_FDCAN2) && !defined(CONFIG_STM32_FDCAN3)
#  error "No CAN is enabled. Please enable at least one CAN device"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Enabled CAN instances */

static const uint8_t g_can_ports[] =
{
#if defined(CONFIG_STM32_CAN1) || defined(CONFIG_STM32_FDCAN1)
  1,
#endif
#if defined(CONFIG_STM32_CAN2) || defined(CONFIG_STM32_FDCAN2)
  2,
#endif
#if defined(CONFIG_STM32_CAN3) || defined(CONFIG_STM32_FDCAN3)
  3,
#endif
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_cansock_setup
 *
 * Description:
 *  Initialize CAN socket interface
 *
 ****************************************************************************/

int stm32_cansock_setup(void)
{
  int ret;
  int i;

#ifdef BOARD_CAN_GPIO_STBY
  /* Enable the CAN transceiver */

  stm32_configgpio(BOARD_CAN_GPIO_STBY);
  stm32_gpiowrite(BOARD_CAN_GPIO_STBY, false);
#endif

  for (i = 0; i < sizeof(g_can_ports); i++)
    {
      ret = CANSOCK_INITIALIZE(g_can_ports[i]);
      if (ret < 0)
        {
          canerr("ERROR: Failed to get CAN%d interface %d\n",
                 g_can_ports[i], ret);
          return ret;
        }
    }

  return OK;
}
