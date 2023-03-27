/****************************************************************************
 * boards/risc-v/litex/arty_a7/src/litex_ramdisk.c
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

#include <stdbool.h>
#include <stdio.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/board.h>

#include <arch/board/board_memorymap.h>
#include <nuttx/drivers/ramdisk.h>
#include <sys/boardctl.h>
#include <sys/mount.h>

#include "litex.h"
#include "arty_a7.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_BUILD_KERNEL
#error "Ramdisk usage is intended to be used with kernel build only"
#endif

#define SECTORSIZE   512
#define NSECTORS(b)  (((b) + SECTORSIZE - 1) / SECTORSIZE)
#define RAMDISK_DEVICE_MINOR 0

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: litex_mount_ramdisk
 *
 * Description:
 *  Mount a ramdisk defined in the ld-kernel.script to /dev/ramX.
 *  The ramdisk is intended to contain a romfs with applications which can
 *  be spawned at runtime.
 *
 * Returned Value:
 *   OK is returned on success.
 *   -ERRORNO is returned on failure.
 *
 ****************************************************************************/

int litex_mount_ramdisk(void)
{
  int ret;
  struct boardioc_romdisk_s desc;

  desc.minor    = RAMDISK_DEVICE_MINOR;
  desc.nsectors = NSECTORS((ssize_t)__ramdisk_size);
  desc.sectsize = SECTORSIZE;
  desc.image    = __ramdisk_start;

  ret = boardctl(BOARDIOC_ROMDISK, (uintptr_t)&desc);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Ramdisk register failed: %s\n", strerror(errno));
      syslog(LOG_ERR, "Ramdisk mountpoint /dev/ram%d\n",
                                          RAMDISK_DEVICE_MINOR);
      syslog(LOG_ERR, "Ramdisk length %u, origin %x\n",
                                          (ssize_t)__ramdisk_size,
                                          (uintptr_t)__ramdisk_start);
    }

  return ret;
}
