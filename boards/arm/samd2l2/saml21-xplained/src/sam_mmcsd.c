/****************************************************************************
 * boards/arm/samd2l2/saml21-xplained/src/sam_mmcsd.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

#include "saml21-xplained.h"

#ifdef CONFIG_SAML21_XPLAINED_IOMODULE

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
 *   - CONFIG_SAML21_XPLAINED_IOMODULE=y,
 *   - CONFIG_DISABLE_MOUNTPOINT=n,
 *   - CONFIG_MMCSD=y, and
 *   - SAMD2L2_HAVE_SPI0=y
 *    (CONFIG_SAMD2L2_SERCOM0 && CONFIG_SAMD2L2_SERCOM0_ISSPI)
 *
 ****************************************************************************/

int sam_sdinitialize(int port, int minor)
{
  FAR struct spi_dev_s *spi;
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

#endif /* CONFIG_SAML21_XPLAINED_IOMODULE */
