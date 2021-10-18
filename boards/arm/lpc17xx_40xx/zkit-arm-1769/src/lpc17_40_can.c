/****************************************************************************
 * boards/arm/lpc17xx_40xx/zkit-arm-1769/src/lpc17_40_can.c
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
#include "arm_arch.h"

#include "lpc17_40_can.h"
#include "zkit-arm-1769.h"

#ifdef CONFIG_CAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define CAN_PORT1 1
#define CAN_PORT2 2

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: zkit_can_setup
 *
 * Description:
 *  Initialize CAN and register the CAN device
 *
 ****************************************************************************/

int zkit_can_setup(void)
{
#if defined(CONFIG_LPC17_40_CAN1) || defined(CONFIG_LPC17_40_CAN2)
  struct can_dev_s *can;
  int ret;

#ifdef CONFIG_LPC17_40_CAN1
  /* Call lpc17_40_caninitialize() to get an instance of the CAN1 interface */

  can = lpc17_40_caninitialize(CAN_PORT1);
  if (can == NULL)
    {
      canerr("ERROR:  Failed to get CAN1 interface\n");
      return -ENODEV;
    }

  /* Register the CAN1 driver at "/dev/can0" */

  ret = can_register("/dev/can0", can);
  if (ret < 0)
    {
      canerr("ERROR: CAN1 register failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_LPC17_40_CAN2
  /* Call lpc17_40_caninitialize() to get an instance of the CAN2 interface */

  can = lpc17_40_caninitialize(CAN_PORT2);
  if (can == NULL)
    {
      canerr("ERROR:  Failed to get CAN2 interface\n");
      return -ENODEV;
    }

  /* Register the CAN2 driver at "/dev/can1" */

  ret = can_register("/dev/can1", can);
  if (ret < 0)
    {
      canerr("ERROR: CAN2 register failed: %d\n", ret);
      return ret;
    }
#endif

  return OK;
#else
  return -ENODEV;
#endif
}

#endif /* CONFIG_CAN */
