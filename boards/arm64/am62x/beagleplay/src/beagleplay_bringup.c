/****************************************************************************
 * boards/arm64/am62x/beagleplay/src/beagleplay_bringup.c
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
#include <nuttx/kmalloc.h>
#include <sys/types.h>
#include <syslog.h>
#include <errno.h>
#include "beagleplay.h"

#ifdef CONFIG_FS_PROCFS
#  include <nuttx/fs/fs.h>
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: beagleplay_bringup
 *
 * Description:
 *   Bring up board features after the scheduler and device drivers are
 *   initialised.  This is the right place to:
 *     - Mount pseudo file systems (procfs, etc.)
 *     - Register I2C / SPI bus instances
 *     - Attach sensor drivers
 *
 *   Phase 1 only mounts procfs — additional drivers will be added as the
 *   port matures.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure.  Non-fatal errors
 *   are logged but do not abort the boot.
 *
 ****************************************************************************/

int beagleplay_bringup(void)
{
  int ret = OK;

#ifdef CONFIG_FS_PROCFS
  /* Mount the process filesystem at /proc */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

  return ret;
}
