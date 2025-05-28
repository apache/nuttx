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

#ifdef CONFIG_MMCSD_SPI
#include <nuttx/spi/spi.h>
#include "esp32s3_spi.h"
#endif

extern struct sdio_dev_s *sdio_initialize(int slotno);
/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_sdmmc_initialize
 *
 * Description:
 *   Configure the SDMMC peripheral to communicate with SDIO or MMC card.
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

/****************************************************************************
 * Name: board_sdmmc_spi_initialize
 *
 * Description:
 *   Initialize SPI-based SD card.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_MMCSD_SPI
int board_sdmmc_spi_initialize(void)
{
  struct spi_dev_s *spi;
  int ret;

  syslog(LOG_INFO, "INFO: init MMC/SD slot %d on SPI%d: /dev/mmcsd%d\n",
         CONFIG_NSH_MMCSDSLOTNO, CONFIG_NSH_MMCSDSPIPORTNO,
         CONFIG_NSH_MMCSDMINOR);

  spi = esp32s3_spibus_initialize(CONFIG_NSH_MMCSDSPIPORTNO);

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
#endif /* CONFIG_MMCSD_SPI */
