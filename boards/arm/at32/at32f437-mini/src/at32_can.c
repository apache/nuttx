/****************************************************************************
 * boards/arm/at32/at32f437-mini/src/at32_can.c
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
#include "at32.h"
#include "at32_can.h"
#include "at32f437-mini.h"

#include <stdio.h>

#ifdef CONFIG_CAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#if !defined(CONFIG_AT32_CAN1) && !defined(CONFIG_AT32_CAN2)
#warning "Both CAN1 and CAN2 are not enabled."
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: at32_can_setup
 *
 * Description:
 *  Initialize CAN and register the CAN device
 *
 ****************************************************************************/

int at32_can_setup(int port)
{
#if defined(CONFIG_AT32_CAN1) || defined(CONFIG_AT32_CAN2)
  struct can_dev_s *can;
  int ret;
  char device_name[16];

  if ((port > 2) || (port < 1))
  return -ENODEV;

  /* Call at32_caninitialize() to get an instance of the CAN interface */

  can = at32_caninitialize(port);

  if (can == NULL)
    {
      canerr("ERROR:  Failed to get CAN interface\n");
      return -ENODEV;
    }

  /* Register the CAN driver at "/dev/can0" */

  sprintf(device_name, "/dev/can%d", port - 1);

  ret = can_register(device_name, can);
  if (ret < 0)
    {
      canerr("ERROR: can_register failed: %d\n", ret);
      return ret;
    }

  return OK;
#else
  return -ENODEV;
#endif
}

#endif /* CONFIG_CAN */
