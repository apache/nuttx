/****************************************************************************
 * boards/arm/rm57/rm57l843-launchxl2/src/rm57_can.c
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

/* DCAN bring-up for the LAUNCHXL2-RM57L.  Each enabled DCAN instance is
 * registered with the framework selected by its Kconfig choice: as a
 * character device (/dev/can<n>, numbered in instance-enable order) or as
 * a SocketCAN network interface (can<n>, numbered by netdev registration
 * order).
 *
 * This LaunchPad has no CAN transceiver on board: all four modules come
 * out on the BoosterPack headers at CMOS levels (DCAN1 A10/B10, DCAN2
 * H2/H1, DCAN3 M18/M19, DCAN4 F2/W10) and need an external transceiver
 * to reach a real bus.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <errno.h>
#include <debug.h>

#include "rm57_dcan.h"
#include "rm57l843-launchxl2.h"

#ifdef CONFIG_RM57_DCAN

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_CAN
static int rm57_can_chardev(int n, int *minor)
{
  char devpath[16];
  int ret;

  snprintf(devpath, sizeof(devpath), "/dev/can%d", *minor);
  ret = rm57_dcan_chardev_initialize(devpath, n);
  if (ret < 0)
    {
      canerr("ERROR: DCAN%d chardev init failed: %d\n", n + 1, ret);
      return ret;
    }

  (*minor)++;
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_can_setup
 *
 * Description:
 *   Register all enabled DCAN instances with their configured upper
 *   halves.  Called from rm57_bringup().
 *
 ****************************************************************************/

int rm57_can_setup(void)
{
  int minor = 0;
  int ret;

  UNUSED(minor);

#if defined(CONFIG_RM57_DCAN1_CHARDRIVER)
  ret = rm57_can_chardev(0, &minor);
  if (ret < 0)
    {
      return ret;
    }
#elif defined(CONFIG_RM57_DCAN1_SOCKET) && defined(CONFIG_NETDEV_LATEINIT)
  ret = rm57_dcan_sock_initialize(0);
  if (ret < 0)
    {
      canerr("ERROR: DCAN1 socket init failed: %d\n", ret);
      return ret;
    }
#endif

#if defined(CONFIG_RM57_DCAN2_CHARDRIVER)
  ret = rm57_can_chardev(1, &minor);
  if (ret < 0)
    {
      return ret;
    }
#elif defined(CONFIG_RM57_DCAN2_SOCKET) && defined(CONFIG_NETDEV_LATEINIT)
  ret = rm57_dcan_sock_initialize(1);
  if (ret < 0)
    {
      canerr("ERROR: DCAN2 socket init failed: %d\n", ret);
      return ret;
    }
#endif

#if defined(CONFIG_RM57_DCAN3_CHARDRIVER)
  ret = rm57_can_chardev(2, &minor);
  if (ret < 0)
    {
      return ret;
    }
#elif defined(CONFIG_RM57_DCAN3_SOCKET) && defined(CONFIG_NETDEV_LATEINIT)
  ret = rm57_dcan_sock_initialize(2);
  if (ret < 0)
    {
      canerr("ERROR: DCAN3 socket init failed: %d\n", ret);
      return ret;
    }
#endif

#if defined(CONFIG_RM57_DCAN4_CHARDRIVER)
  ret = rm57_can_chardev(3, &minor);
  if (ret < 0)
    {
      return ret;
    }
#elif defined(CONFIG_RM57_DCAN4_SOCKET) && defined(CONFIG_NETDEV_LATEINIT)
  ret = rm57_dcan_sock_initialize(3);
  if (ret < 0)
    {
      canerr("ERROR: DCAN4 socket init failed: %d\n", ret);
      return ret;
    }
#endif

  UNUSED(ret);
  return OK;
}

#endif /* CONFIG_RM57_DCAN */
