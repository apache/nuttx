/****************************************************************************
 * boards/arm/stm32/b-g431b-esc1/src/stm32_can.c
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
#include <debug.h>

#include <nuttx/can/can.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "stm32.h"
#include "stm32_fdcan.h"
#include "b-g431b-esc1.h"

#ifdef CONFIG_CAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#if !defined(CONFIG_STM32_FDCAN1)
#  error "No CAN is enable. Please eneable at least one CAN device"
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

  /* Call stm32_fdcaninitialize() to get an instance of the CAN interface */

  can = stm32_fdcaninitialize(1);
  if (can == NULL)
    {
      canerr("ERROR:  Failed to get CAN interface\n");
      return -ENODEV;
    }

  /* Register the CAN driver at "/dev/can0" */

  ret = can_register("/dev/can0", can);
  if (ret < 0)
    {
      canerr("ERROR: can_register failed: %d\n", ret);
      return ret;
    }

  /* Configure CAN_TERM pin for output */

  stm32_configgpio(GPIO_CANTERM);

  /* Set CAN_TERM pin high or low */

  stm32_gpiowrite(GPIO_CANTERM, BG431BESC1_CANTERM);

  return OK;
}

#endif /* CONFIG_CAN */
