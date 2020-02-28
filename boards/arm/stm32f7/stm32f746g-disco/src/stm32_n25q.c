/****************************************************************************
 * boards/arm/stm32f7/stm32f746g-disco/src/stm32_n25.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <sys/mount.h>
#include <stdio.h>
#include <syslog.h>
#include <errno.h>
#include <debug.h>
#include <string.h>
#include <stdlib.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>

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
  FAR struct qspi_dev_s *qspi_dev ;
  FAR struct mtd_dev_s *mtd_dev;
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

  ret = mount(NULL, "/mnt/nxffs", "nxffs", 0, NULL);
  if (ret < 0)
    {
      _err("ERROR: Failed to mount the NXFFS volume: %d\n", errno);
      return ret;
    }

#endif

  return 0;
}
