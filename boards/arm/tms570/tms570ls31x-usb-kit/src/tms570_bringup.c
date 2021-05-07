/****************************************************************************
 * boards/arm/tms570/tms570ls31x-usb-kit/src/tms570_bringup.c
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
#include <stdio.h>
#include <syslog.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>
#include "tms570ls31x_usb_kit.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Checking needed by MMC/SDCard */

#ifdef CONFIG_NSH_MMCSDMINOR
#  define MMCSD_MINOR CONFIG_NSH_MMCSDMINOR
#else
#  define MMCSD_MINOR 0
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tms570_bringup
 *
 * Description:
 *   Bring up simulated board features
 *
 ****************************************************************************/

int tms570_bringup(void)
{
  int ret = OK;

#ifdef CONFIG_MMCSD
  tms570_spidev_initialize();
  ret = tms570_mmcsd_initialize(MMCSD_MINOR);
  if (ret < 0)
    {
      serr("Failed to initialize SD slot %d: %d\n", ret);
      return ret;
    }
#endif

  UNUSED(ret);
  return OK;
}
