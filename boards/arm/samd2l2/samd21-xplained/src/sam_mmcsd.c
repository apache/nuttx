/****************************************************************************
 * boards/arm/samd2l2/samd21-xplained/src/sam_mmcsd.c
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

#include "sam_config.h"
#include "sam_spi.h"

#include "samd21-xplained.h"

#ifdef CONFIG_SAMD21_XPLAINED_IOMODULE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifdef CONFIG_DISABLE_MOUNTPOINT
#  error Mountpoints are disabled (CONFIG_DISABLE_MOUNTPOINT=y)
#endif

#ifndef SAMD2L2_HAVE_SPI0
#  error SERCOM0 SPI support is required
#endif

#ifndef CONFIG_MMCSD
#  error MMC/SD support is required (CONFIG_MMCSD)
#endif

#define SAMD2L2_MMCSDSLOTNO  0 /* There is only one slot */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_sdinitialize
 *
 * Description:
 *   Initialize the SPI-based SD card.  Requires
 *   - CONFIG_SAMD21_XPLAINED_IOMODULE=y,
 *   - CONFIG_DISABLE_MOUNTPOINT=n,
 *   - CONFIG_MMCSD=y, and
 *   - SAMD2L2_HAVE_SPI0=y
 *     (CONFIG_SAMD2L2_SERCOM0 && CONFIG_SAMD2L2_SERCOM0_ISSPI)
 *
 ****************************************************************************/

int sam_sdinitialize(int port, int minor)
{
  struct spi_dev_s *spi;
  int ret;

  /* Get the SPI driver instance for the SD chip select */

  finfo("Initializing SERCOM SPI%d\n", port);

  spi = sam_spibus_initialize(port);
  if (!spi)
    {
      ferr("ERROR: Failed to initialize SPI%d\n", port);
      return -ENODEV;
    }

  finfo("Successfully initialized SPI%d\n", port);

  /* Bind the SPI device for the chip select to the slot */

  finfo("Binding SPI%d to MMC/SD slot %d\n", port, SAMD2L2_MMCSDSLOTNO);

  ret = mmcsd_spislotinitialize(minor, SAMD2L2_MMCSDSLOTNO, spi);
  if (ret < 0)
    {
      ferr("ERROR: Failed to bind SPI%d to MMC/SD slot %d: %d\n",
            port, SAMD2L2_MMCSDSLOTNO, ret);
      return ret;
    }

  finfo("Successfully bound SPI%d to MMC/SD slot %d\n",
        port, SAMD2L2_MMCSDSLOTNO);

  return OK;
}

#endif /* CONFIG_SAMD21_XPLAINED_IOMODULE */
