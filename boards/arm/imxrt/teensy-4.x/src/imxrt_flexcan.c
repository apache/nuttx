/****************************************************************************
 * boards/arm/imxrt/teensy-4.x/src/imxrt_flexcan.c
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
#include <debug.h>

#include <nuttx/can/can.h>

#include "imxrt_flexcan.h"
#include "teensy-4.h"

#if defined(CONFIG_IMXRT_FLEXCAN) && defined(CONFIG_NETDEV_LATEINIT)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_can_setup
 *
 * Description:
 *  Initialize CAN and register the CAN device
 *
 ****************************************************************************/

int imxrt_can_setup(void)
{
  int ret;

#ifdef CONFIG_IMXRT_FLEXCAN3_AS_CAN0
# ifdef CONFIG_IMXRT_FLEXCAN3
  ret = imxrt_caninitialize(3);
  if (ret < 0)
    {
      canerr("ERROR: Failed to get CAN interface\n");
      return -ENODEV;
    }

# endif
#endif

#ifdef CONFIG_IMXRT_FLEXCAN1
  /* Call arm_caninitialize() to get an instance of the CAN interface */

  ret = imxrt_caninitialize(1);
  if (ret < 0)
    {
      canerr("ERROR: Failed to get CAN interface\n");
      return -ENODEV;
    }

#endif
#ifdef CONFIG_IMXRT_FLEXCAN2
  ret = imxrt_caninitialize(2);
  if (ret < 0)
    {
      canerr("ERROR: Failed to get CAN interface\n");
      return -ENODEV;
    }

#endif
#ifndef CONFIG_IMXRT_FLEXCAN3_AS_CAN0
# ifdef CONFIG_IMXRT_FLEXCAN3
  ret = imxrt_caninitialize(3);
  if (ret < 0)
    {
      canerr("ERROR: Failed to get CAN interface\n");
      return -ENODEV;
    }

# endif
#endif
  UNUSED(ret);
  return OK;
}

#endif /* CONFIG_IMXRT_FLEXCAN */
