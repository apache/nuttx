/****************************************************************************
 * boards/arm/stm32f0l0g0/stm32f051-discovery/src/stm32_bringup.c
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

#include <nuttx/fs/fs.h>

#include <nuttx/fs/nxffs.h>

#include <nuttx/mtd/mtd.h>

#include <syslog.h>

#include "phy6222.h"
#include "pplus_mtd_flash.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/
#define PPLUS_MTD_START_OFFSET  0x40000   //start from 256k offset
#define PPLUS_MTD_SIZE          0x40000   //mtd size is 256k bytes

int phy62xx_bringup(void)
{
  int ret;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
  {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
      return ret;
  }
#endif

#ifdef CONFIG_FS_NXFFS

      struct mtd_dev_s* mtd_temp = pplus_fls_initialize(PPLUS_MTD_START_OFFSET, PPLUS_MTD_SIZE);
      if (!mtd_temp)
        {
    	  syslog(LOG_ERR, "ERROR: pplus_initialize failed\n");
          return ret;
        }

      /* Initialize to provide NXFFS on the N25QXXX MTD interface */

      ret = nxffs_initialize(mtd_temp);
      if (ret < 0)
        {
    	  syslog(LOG_ERR, "ERROR: NXFFS initialization failed: %d\n", ret);
        }

      /* Mount the file system at /mnt/nxffs */

      ret = nx_mount(NULL, "/mnt/nxffs", "nxffs", 0, NULL);
      if (ret < 0)
        {
    	  syslog(LOG_ERR, "ERROR: Failed to mount the NXFFS volume: %d\n", ret);
          return ret;
        }

#endif
#ifdef CONFIG_FS_LITTLEFS

      struct mtd_dev_s* mtd = pplus_fls_initialize(PPLUS_MTD_START_OFFSET, PPLUS_MTD_SIZE);
      if (!mtd)
        {
    	  syslog(LOG_ERR, "ERROR: pplus_initialize failed\n");
          return ret;
        }


      /* Erase the RAM MTD */

      ret = mtd->ioctl(mtd, MTDIOC_BULKERASE, 0);

      ret = register_mtddriver("/dev/mtd", mtd, 0755, NULL);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register MTD driver: %d\n",
                 ret);
        }

      /* Mount the LittleFS file system */

      ret = nx_mount("/dev/mtd", "/mnt/lfs", "littlefs", 0,
                     "forceformat");
      if (ret < 0)
        {
          syslog(LOG_ERR,
                 "ERROR: Failed to mount LittleFS at /mnt/lfs: %d\n",
                 ret);
        }



#endif

#ifdef CONFIG_PHY6222_BLE

  ret = pplus_ble_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to init ble device\n",
             ret);
    }
#endif

  UNUSED(ret);
  return OK;
}
