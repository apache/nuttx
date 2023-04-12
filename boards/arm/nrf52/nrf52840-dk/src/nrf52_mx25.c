/****************************************************************************
 * boards/arm/nrf52/nrf52840-dk/src/nrf52_mx25.c
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
#include <stdio.h>
#include <sys/types.h>

#include <nuttx/fs/fs.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/spi/qspi.h>

#include "nrf52_qspi.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_mx25_initialize
 *
 * Description:
 *   Initialize the MX25RXX QSPI memeory
 *
 ****************************************************************************/

int nrf52_mx25_initialize(void)
{
  struct qspi_dev_s *qspi_dev;
  struct mtd_dev_s  *mtd_dev;
  char               blockdev[32];
  int                ret = -1;

  /* Create an instance of the NRF52 QSPI device driver */

  qspi_dev = nrf52_qspi_initialize(0);
  if (!qspi_dev)
    {
      _err("nrf52_qspi_initialize() failed\n");
      return -1;
    }

  _info("nrf52_qspi_initialize() successful\n");

  /* Use the QSPI device instance to initialize the MX25 device */

  mtd_dev = mx25rxx_initialize(qspi_dev, true);
  if (!mtd_dev)
    {
      _err("mx25rxx_initialize() failed\n");
      return -1;
    }

  /* Configure the device with no partition support */

  snprintf(blockdev, sizeof(blockdev), "/dev/mtdqspi%d", 0);

  ret = register_mtddriver(blockdev, mtd_dev, 0755, NULL);
  if (ret != OK)
    {
      _err("register_mtddriver() failed: %d\n", ret);
      return -1;
    }

  _info("register_mtddriver() successful\n");

#ifdef CONFIG_FS_LITTLEFS
  ret = nx_mount(blockdev, "/mnt/qspi", "littlefs", 0, NULL);
  if (ret < 0)
    {
      ret = nx_mount(blockdev, "/mnt/qspi", "littlefs", 0,
                     "forceformat");
      if (ret < 0)
        {
          _err("nx_mount() failed: %d\n", ret);
        }
      else
        {
          _info("nx_mount() successful\n");
        }
    }
#endif

  return ret;
}
