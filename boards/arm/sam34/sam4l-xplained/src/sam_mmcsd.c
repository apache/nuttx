/****************************************************************************
 * boards/arm/sam34/sam4l-xplained/src/sam_mmcsd.c
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

#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/spi/spi.h>
#include <nuttx/mmcsd.h>

#include "sam_spi.h"

#include "sam4l-xplained.h"

#ifdef CONFIG_SAM4L_XPLAINED_IOMODULE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifdef CONFIG_DISABLE_MOUNTPOINT
#  error Mountpoints are disabled (CONFIG_DISABLE_MOUNTPOINT=y)
#endif

#ifndef CONFIG_SAM34_SPI0
#  error SPI support is required (CONFIG_SAM34_SPI0)
#endif

#ifndef CONFIG_MMCSD
#  error MMC/SD support is required (CONFIG_MMCSD)
#endif

#define SAM34_MMCSDSLOTNO    0 /* There is only one slot */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_sdinitialize
 *
 * Description:
 *   Initialize the SPI-based SD card.  Requires
 *   - CONFIG_SAM4L_XPLAINED_IOMODULE=y,
 *   - CONFIG_DISABLE_MOUNTPOINT=n,
 *   - CONFIG_MMCSD=y, and
 *   - CONFIG_SAM34_SPI0=y
 *
 ****************************************************************************/

int sam_sdinitialize(int minor)
{
  struct spi_dev_s *spi;
  int ret;

  /* Get the SPI driver instance for the SD chip select */

  finfo("Initializing SPI chip select %d\n", SD_CSNO);

  spi = sam_spibus_initialize(SD_CSNO);
  if (!spi)
    {
      ferr("ERROR: Failed to initialize SPI chip select %d\n", SD_CSNO);
      return -ENODEV;
    }

  finfo("Successfully initialized SPI chip select %d\n", SD_CSNO);

  /* Bind the SPI device for the chip select to the slot */

  finfo("Binding SPI chip select %d to MMC/SD slot %d\n",
          SD_CSNO, SAM34_MMCSDSLOTNO);

  ret = mmcsd_spislotinitialize(minor, SAM34_MMCSDSLOTNO, spi);
  if (ret < 0)
    {
      ferr(
          "ERROR: Failed to bind SPI chip select %d to MMC/SD slot %d: %d\n",
            SD_CSNO, SAM34_MMCSDSLOTNO, ret);
      return ret;
    }

  finfo("Successfully bound SPI chip select %d to MMC/SD slot %d\n",
        SD_CSNO, SAM34_MMCSDSLOTNO);

  return OK;
}

#endif /* CONFIG_SAM4L_XPLAINED_IOMODULE */
