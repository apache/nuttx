/****************************************************************************
 * boards/xtensa/esp32/common/src/esp32_board_sdmmc.c
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
#include <nuttx/mmcsd.h>
#include <nuttx/spi/spi.h>

#include "esp32_spi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_mmcsd_initialize
 *
 * Description:
 *   Configure a SPI subsystem peripheral to communicate with SD card.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int board_sdmmc_initialize(void)
{
  struct spi_dev_s *spi;
  int ret;

  syslog(LOG_INFO, "INFO: init MMC/SD slot %d on SPI%d: /dev/mmcsd%d\n",
         CONFIG_NSH_MMCSDSLOTNO, CONFIG_NSH_MMCSDSPIPORTNO,
         CONFIG_NSH_MMCSDMINOR);

  spi = esp32_spibus_initialize(CONFIG_NSH_MMCSDSPIPORTNO);

  if (spi == NULL)
    {
      syslog(LOG_ERR, "ERROR: failed to initialize SPI%d.\n",
             CONFIG_NSH_MMCSDSPIPORTNO);
      return -ENODEV;
    }

  /* Mounts to /dev/mmcsdN where N in the minor number */

  ret = mmcsd_spislotinitialize(CONFIG_NSH_MMCSDMINOR,
                                CONFIG_NSH_MMCSDSLOTNO, spi);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: failed to bind SPI%d to SD slot %d\n",
            CONFIG_NSH_MMCSDSPIPORTNO, CONFIG_NSH_MMCSDSLOTNO);
      return ret;
    }

  syslog(LOG_INFO, "INFO: MMCSD initialized\n");
  return OK;
}
