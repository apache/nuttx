/****************************************************************************
 * boards/arm/tiva/tm4c1294-launchpad/src/tm4c_can.c
 * Based heavily on tiva_can.c from the tm4c1293-launchpad board
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

#include "tiva_can.h"
#include "tm4c1294-launchpad.h"

#include "tiva_enableclks.h"
#include "tiva_gpio.h"
#include "hardware/tiva_pinmap.h"

#ifdef CONFIG_TIVA_CAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tm4c_can_setup
 *
 * Description:
 *  Initialize CAN and register the CAN device
 *
 ****************************************************************************/

int tm4c_can_setup(void)
{
  int ret = ERROR;

#  ifdef CONFIG_TIVA_CAN0
  tiva_can0_enableclk();

  ret = tiva_configgpio(GPIO_CAN0_RX);
  if (ret < 0)
    {
      goto configgpio_error;
    }

  ret = tiva_configgpio(GPIO_CAN0_TX);
  if (ret < 0)
    {
      goto configgpio_error;
    }

  /* Call tiva_can_initialize() to get an instance of CAN interface 0
   * and register it.
   */

  ret = tiva_can_initialize("/dev/can0", 0);
  if (ret < 0)
    {
      canerr("ERROR:  Failed to get/register CAN interface 0\n");
      return ret;
    }
#  endif /* CONFIG_TIVA_CAN0 */

#  ifdef CONFIG_TIVA_CAN1
  tiva_can1_enableclk();

  ret = tiva_configgpio(GPIO_CAN1_RX);
  if (ret < 0)
    {
      goto configgpio_error;
    }

  ret = tiva_configgpio(GPIO_CAN1_TX);
  if (ret < 0)
    {
      goto configgpio_error;
    }

  /* Call tiva_can_initialize() to get an instance of CAN interface 1
   * and register it.
   */

  ret = tiva_can_initialize("/dev/can1", 1);
  if (ret < 0)
    {
      canerr("ERROR:  Failed to get/register CAN interface 1\n");
      return ret;
    }
#  endif /* CONFIG_TIVA_CAN1 */

  return OK;

configgpio_error:
  canerr("ERROR: failed to configure CAN GPIO pin.\n");
  return ret;
}

#endif /* CONFIG_TIVA_CAN */
