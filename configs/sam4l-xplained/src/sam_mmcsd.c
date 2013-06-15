/****************************************************************************
 * config/shenzhou/src/sam_mmcsd.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

#include <nuttx/spi.h>
#include <nuttx/mmcsd.h>

#ifdef CONFIG_SAM4L_XPLAINED_IOMODULE

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifdef CONFIG_DISABLE_MOUNTPOINT
#  error Mountpoints are disabled (CONFIG_DISABLE_MOUNTPOINT=y)
#endif

#ifndef CONFIG_SAM34_SPI
#  error SPI support is required (CONFIG_SAM34_SPI)
#endif

#ifndef CONFIG_MMCSD
#  error MMC/SD support is required (CONFIG_MMCSD)
#endif

#define SAM34_MMCSDSPIPORTNO 0 /* Port is SPI (there is only one port) */
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
 *   - CONFIG_SAM34_SPI=y
 *
 *****************************************************************************/

int sam_sdinitialize(int minor)
{
#ifdef HAVE_MMCSD
  FAR struct spi_dev_s *spi;
  int ret;

  /* Get the SPI port */

  fvdbg("Initializing SPI port %d\n", SAM34_MMCSDSPIPORTNO);

  spi = up_spiinitialize(SAM34_MMCSDSPIPORTNO);
  if (!spi)
    {
      fdbg("Failed to initialize SPI port %d\n", SAM34_MMCSDSPIPORTNO);
      return -ENODEV;
    }

  fvdbg("Successfully initialized SPI port %d\n", SAM34_MMCSDSPIPORTNO);

  /* Bind the SPI port to the slot */

  fvdbg("Binding SPI port %d to MMC/SD slot %d\n",
          SAM34_MMCSDSPIPORTNO, SAM34_MMCSDSLOTNO);

  ret = mmcsd_spislotinitialize(minor, SAM34_MMCSDSLOTNO, spi);
  if (ret < 0)
    {
      fdbg("Failed to bind SPI port %d to MMC/SD slot %d: %d\n",
            SAM34_MMCSDSPIPORTNO, SAM34_MMCSDSLOTNO, ret);
      return ret;
    }

  fvdbg("Successfuly bound SPI port %d to MMC/SD slot %d\n",
        SAM34_MMCSDSPIPORTNO, SAM34_MMCSDSLOTNO);
#endif

  return OK;
}

#endif /* CONFIG_SAM4L_XPLAINED_IOMODULE */
