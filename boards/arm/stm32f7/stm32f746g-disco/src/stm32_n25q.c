/****************************************************************************
 * boards/arm/stm32f7/stm32f746g-disco/src/stm32_n25q.c
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
#include <stdio.h>
#include <syslog.h>
#include <errno.h>
#include <debug.h>
#include <string.h>
#include <stdlib.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/fs/fs.h>

#include <arch/board/board.h>

#include <nuttx/mtd/mtd.h>
#include <nuttx/drivers/drivers.h>
#include <nuttx/drivers/ramdisk.h>

#ifdef CONFIG_FS_NXFFS
#include <nuttx/fs/nxffs.h>
#endif

#ifdef CONFIG_FS_SMARTFS
#include <nuttx/fs/smart.h>
#endif

#include "stm32f746g-disco.h"

#include "stm32_qspi.h"

#define HAVE_N25QXXX_NXFFS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_n25qxxx_setup
 *
 * Description:
 *   This function is called by board-bringup logic to configure the
 *   flash device.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int stm32_n25qxxx_setup(void)
{
  struct qspi_dev_s *qspi_dev;
  struct mtd_dev_s *mtd_dev;
  int ret = -1;

  qspi_dev = stm32f7_qspi_initialize(0);
  if (!qspi_dev)
    {
      _err("ERROR: Failed to initialize W25 minor %d: %d\n",
           0, ret);
      return -1;
    }

    mtd_dev = n25qxxx_initialize(qspi_dev, true);
  if (!mtd_dev)
    {
      _err("ERROR: n25qxxx_initialize() failed!\n");
      return -1;
    }

#ifdef HAVE_N25QXXX_NXFFS
  /* Initialize to provide NXFFS on the N25QXXX MTD interface */

  ret = nxffs_initialize(mtd_dev);
  if (ret < 0)
    {
      _err("ERROR: NXFFS initialization failed: %d\n", ret);
      return ret;
    }

  ret = nx_mount(NULL, "/mnt/nxffs", "nxffs", 0, NULL);
  if (ret < 0)
    {
      _err("ERROR: Failed to mount the NXFFS volume: %d\n", ret);
      return ret;
    }

#endif

  return 0;
}
