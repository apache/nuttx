/****************************************************************************
 * boards/risc-v/mpfs/common/src/mpfs_usb.c
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

#include <debug.h>
#include <errno.h>

#include <nuttx/usb/cdcacm.h>
#include <nuttx/usb/usbmonitor.h>

#include "mpfs_usb.h"
#include "board_config.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_board_usb_init
 *
 * Description:
 *   Configure the USB driver.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int mpfs_board_usb_init(void)
{
  int ret = OK;

  mpfs_usbinitialize();

#if defined(CONFIG_CDCACM) && !defined(CONFIG_CDCACM_COMPOSITE)
  ret = cdcacm_initialize(0, NULL);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the CDC/ACM serial class: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_USBDEV_COMPOSITE
#ifndef CONFIG_BOARDCTL_USBDEVCTRL

  ret = board_composite_initialize(0);
  if (ret != OK)
    {
      syslog(LOG_ERR, "Failed to initialize composite: %d\n", ret);
      return ret;
    }

  if (board_composite_connect(0, 0) == NULL)
    {
      syslog(LOG_ERR, "Failed to connect composite: %d\n", ret);
      return ret;
    }

#endif /* !CONFIG_BOARDCTL_USBDEVCTRL */
#endif /* CONFIG_USBDEV_COMPOSITE */

#ifdef CONFIG_USBMONITOR
  /* Start the USB Monitor */

  syslog(LOG_INFO, "Starting USB Monitor\n");
  ret = usbmonitor_start();
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to start USB monitor: %d\n", ret);
      return ret;
    }
#endif

  return ret;
}
