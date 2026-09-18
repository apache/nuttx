/****************************************************************************
 * boards/arm/imxrt/frdm-imxrt1186/src/imxrt_bringup.c
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

#include <syslog.h>

#include <nuttx/board.h>
#include <nuttx/debug.h>
#include <nuttx/fs/fs.h>

#ifdef CONFIG_CDCACM
#  include <nuttx/usb/cdcacm.h>
#endif

#ifdef CONFIG_IMXRT_NETC
#  include "imxrt_netc.h"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_bringup
 *
 * Description:
 *   Bring up board features.
 *
 ****************************************************************************/

int imxrt_bringup(void)
{
  /* NSH_USBCONSOLE selects BOARDCTL_USBDEVCTRL, so usbnsh declares CDCACM
   * but does not call cdcacm_initialize() here.  Match ret to actual uses.
   */

#if defined(CONFIG_FS_PROCFS) || defined(CONFIG_IMXRT_NETC) || \
    (defined(CONFIG_CDCACM) && !defined(CONFIG_BOARDCTL_USBDEVCTRL) && \
     !defined(CONFIG_USBDEV_COMPOSITE) && !defined(CONFIG_SYSTEM_CDCACM))
  int ret;
#endif

#ifdef CONFIG_FS_PROCFS
  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#if !defined(CONFIG_BOARDCTL_USBDEVCTRL) && \
    !defined(CONFIG_USBDEV_COMPOSITE) && \
    !defined(CONFIG_SYSTEM_CDCACM)
#  ifdef CONFIG_CDCACM
  /* SYSTEM_CDCACM provides sercon/serdis for bring-up; only auto-bind when
   * that helper is absent.
   */

  ret = cdcacm_initialize(0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: cdcacm_initialize failed: %d\n", ret);
    }
#  endif
#endif

#ifdef CONFIG_IMXRT_NETC
  ret = imxrt_netc_initialize();
  if (ret < 0)
    {
      nerr("ERROR: NETC initialization failed: %d\n", ret);
    }
#endif

  return 0;
}
