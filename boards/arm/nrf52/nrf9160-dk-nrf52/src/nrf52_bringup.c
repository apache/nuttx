/****************************************************************************
 * boards/arm/nrf52/nrf9160-dk-nrf52/src/nrf52_bringup.c
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

#include <sys/types.h>
#include <syslog.h>

#include <nuttx/fs/fs.h>

#ifdef CONFIG_NRF52_RADIO_IEEE802154
#  include "nrf52_ieee802154.h"
#endif

#include "nrf9160-dk-nrf52.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int nrf52_bringup(void)
{
  int ret;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, NRF52_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to mount the PROC filesystem: %d\n",  ret);
    }
#endif /* CONFIG_FS_PROCFS */

#ifdef CONFIG_NRF52_RADIO_IEEE802154
  ret = nrf52_ieee802154_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize IEE802154 radio: %d\n",
             ret);
    }
#endif

  UNUSED(ret);
  return OK;
}
