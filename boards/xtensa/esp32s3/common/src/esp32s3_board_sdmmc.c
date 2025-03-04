/****************************************************************************
 * boards/xtensa/esp32s3/common/src/esp32s3_board_sdmmc.c
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
#include <debug.h>

#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>

extern struct sdio_dev_s *sdio_initialize(int slotno);
/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_sdmmc_initialize
 *
 * Description:
 *   Configure the sdmmc subsystem.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int board_sdmmc_initialize(void)
{
  struct sdio_dev_s *sdio;
  int rv;

  sdio = sdio_initialize(1);
  if (!sdio)
    {
      syslog(LOG_ERR, "Failed to initialize SDIO slot\n");
      return -ENODEV;
    }

  rv = mmcsd_slotinitialize(1, sdio);
  if (rv < 0)
    {
      syslog(LOG_ERR, "Failed to bind SPI port to SD slot\n");
      return rv;
    }

  return OK;
}
