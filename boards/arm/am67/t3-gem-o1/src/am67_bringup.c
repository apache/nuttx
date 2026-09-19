/****************************************************************************
 * boards/arm/am67/t3-gem-o1/src/am67_bringup.c
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
#include <nuttx/fs/fs.h>
#include <debug.h>

#include "t3-gem-o1.h"

#ifdef CONFIG_AM67_MCSPI0
#include "am67_mcspi.h"
#include "am67_gpio.h"
#endif

#if defined(CONFIG_AM67_I2C0) || defined(CONFIG_AM67_WKUP_I2C0)
#include "am67_i2c.h"
#endif

#if defined(CONFIG_AM67_EPWM0) || defined(CONFIG_AM67_EPWM1)
#include "am67_pwm.h"
#endif

#if defined(CONFIG_AM67_ECAP0) || defined(CONFIG_AM67_ECAP1) || \
    defined(CONFIG_AM67_ECAP2)
#include "am67_ecap.h"
#endif

#ifdef CONFIG_RPTUN
#include "am67_rptun.h"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am67_bringup
 *
 * Description:
 *   Perform architecture-specific initialization.
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library.
 *
 ****************************************************************************/

int am67_bringup(void)
{
  int ret = OK;

#ifdef CONFIG_AM67_MCSPI0
  am67_sensors_power_enable(true);
  am67_spiinitialize();
  am67_spidev_initialize();
#endif

#if defined(CONFIG_AM67_I2C0) || defined(CONFIG_AM67_WKUP_I2C0)
  am67_i2cdev_initialize();
#endif

#if defined(CONFIG_AM67_EPWM0) || defined(CONFIG_AM67_EPWM1)
  am67_pwmdev_initialize();
#endif

#if defined(CONFIG_AM67_ECAP0) || defined(CONFIG_AM67_ECAP1) || \
    defined(CONFIG_AM67_ECAP2)
  am67_ecapdev_initialize();
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#ifdef CONFIG_RPTUN
  /* Initialize the rptun device.  This registers the resource table
   * (RPMsg + virtio-net) with NuttX's OpenAMP stack and starts the
   * rptun thread.  Linux remoteproc must be running and must have
   * already booted the R5F core (i.e. we arrive here after remoteproc
   * has written the vdev status/features into the resource table).
   *
   */

  ret = am67_rptun_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: am67_rptun_init failed: %d\n", ret);
    }
#endif

  return ret;
}
