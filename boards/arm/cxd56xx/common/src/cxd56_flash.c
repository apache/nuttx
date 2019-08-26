/****************************************************************************
 * boards/arm/cxd56xx/common/src/cxd56_flash.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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
#include <errno.h>
#include <debug.h>
#include <sys/mount.h>
#include <nuttx/board.h>
#include <arch/board/board.h>
#include <nuttx/mtd/mtd.h>

#ifdef CONFIG_FS_NXFFS
#  include <nuttx/fs/nxffs.h>
#endif

#include "cxd56_sfc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SFC_DEVNO
#  define CONFIG_SFC_DEVNO 0
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_flash_initialize
 *
 * Description:
 *   Initialize the SPI-Flash device and mount the file system.
 *
 ****************************************************************************/

int board_flash_initialize(void)
{
  int ret;
  FAR struct mtd_dev_s *mtd;

  mtd = cxd56_sfc_initialize();
  if (!mtd)
    {
      ferr("ERROR: Failed to initialize SFC. %d\n ", ret);
      return -ENODEV;
    }

  /* use the FTL layer to wrap the MTD driver as a block driver */

  ret = ftl_initialize(CONFIG_SFC_DEVNO, mtd);
  if (ret < 0)
    {
      ferr("ERROR: Initializing the FTL layer: %d\n", ret);
      return ret;
    }

#if defined(CONFIG_FS_SMARTFS)
  /* Initialize to provide SMARTFS on the MTD interface */

  ret = smart_initialize(CONFIG_SFC_DEVNO, mtd, NULL);
  if (ret < 0)
    {
      ferr("ERROR: SmartFS initialization failed: %d\n", ret);
      return ret;
    }

  ret = mount("/dev/smart0d1", "/mnt/spif", "smartfs", 0, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Failed to mount the SmartFS volume: %d\n", errno);
      return ret;
    }

#elif defined(CONFIG_FS_NXFFS)
  /* Initialize to provide NXFFS on the MTD interface */

  ret = nxffs_initialize(mtd);
  if (ret < 0)
    {
      ferr("ERROR: NXFFS initialization failed: %d\n", ret);
      return ret;
    }

  ret = mount(NULL, "/mnt/spif", "nxffs", 0, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Failed to mount the NXFFS volume: %d\n", errno);
      return ret;
    }
#endif

  return OK;
}
