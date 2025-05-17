/****************************************************************************
 * boards/arm/stm32/b-g431b-esc1/src/stm32_cansock.c
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

#include <debug.h>

#include "stm32_fdcan.h"
#include "b-g431b-esc1.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#if !defined(CONFIG_STM32_FDCAN1)
#  error "No CAN is enable. Please enable at least one CAN device"
#endif

#ifdef CONFIG_BOARD_STM32_BG431BESC1_CANTERM
#  define BG431BESC1_CANTERM (true)
#else
#  define BG431BESC1_CANTERM (false)
#endif

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

  /* Call stm32_fdcaninitialize() to get an instance of the FDCAN interface */

  ret = stm32_fdcansockinitialize(1);
  if (ret < 0)
    {
      canerr("ERROR:  Failed to get FDCAN interface %d\n", ret);
      return ret;
    }

  /* Configure CAN_TERM pin for output */

  stm32_configgpio(GPIO_CANTERM);

  /* Set CAN_TERM pin high or low */

  stm32_gpiowrite(GPIO_CANTERM, BG431BESC1_CANTERM);

  return OK;
}
