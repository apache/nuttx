/****************************************************************************
 * boards/arm/gd32f4/gd32f450zk-eval/src/gd32f4xx_romfs.c
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

#include <sys/mount.h>
#include <sys/types.h>
#include <stdint.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/drivers/ramdisk.h>
#include "gd32f4xx_romfs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_GD32F4_ROMFS
#  error "CONFIG_GD32F4_ROMFS must be defined"
#else

#ifndef CONFIG_GD32F4_ROMFS_IMAGEFILE
#  error "CONFIG_GD32F4_ROMFS_IMAGEFILE must be defined"
#endif

#ifndef CONFIG_GD32F4_ROMFS_DEV_MINOR
#  error "CONFIG_GD32F4_ROMFS_DEV_MINOR must be defined"
#endif

#ifndef CONFIG_GD32F4_ROMFS_MOUNTPOINT
#  error "CONFIG_GD32F4_ROMFS_MOUNTPOINT must be defined"
#endif

#define NSECTORS(size) (((size) + ROMFS_SECTOR_SIZE - 1)/ROMFS_SECTOR_SIZE)

#define STR2(m)  #m
#define STR(m) STR2(m)

#define MKMOUNT_DEVNAME(m) "/dev/ram" STR(m)
#define MOUNT_DEVNAME      MKMOUNT_DEVNAME(CONFIG_GD32F4_ROMFS_DEV_MINOR)

/****************************************************************************
 * Private Data
 ****************************************************************************/

__asm__ (
    "   .section .rodata                            \n"
    "   .balign  16                                 \n"
    "   .globl   romfs_data_begin                   \n"
    "romfs_data_begin:                              \n"
    "   .incbin " STR(CONFIG_GD32F4_ROMFS_IMAGEFILE)"\n"
    "   .balign " STR(ROMFS_SECTOR_SIZE)           "\n"
    "   .globl   romfs_data_end                     \n"
    "romfs_data_end:                                \n"
    );

extern const uint8_t romfs_data_begin[];
extern const uint8_t romfs_data_end[];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_romfs_initialize
 *
 * Description:
 *   Registers the aboveincluded binary file as block device.
 *   Then mounts the block device as ROMFS filesystems.
 *
 * Returned Value:
 *   Zero (OK) on success, a negated errno value on error.
 *
 * Assumptions/Limitations:
 *   Memory addresses [romfs_data_begin .. romfs_data_end) should contain
 *   ROMFS volume data, as included in the assembly snippet above.
 *
 ****************************************************************************/

int gd32_romfs_initialize(void)
{
  uintptr_t romfs_data_len;
  int  ret;

  /* Create a ROM disk for the /etc filesystem */

  romfs_data_len = romfs_data_end - romfs_data_begin;

  ret = romdisk_register(CONFIG_GD32F4_ROMFS_DEV_MINOR, romfs_data_begin,
                         NSECTORS(romfs_data_len), ROMFS_SECTOR_SIZE);
  if (ret < 0)
    {
      ferr("ERROR: romdisk_register failed: %d\n", -ret);
      return ret;
    }

  /* Mount the file system */

  finfo("Mounting ROMFS filesystem at target=%s with source=%s\n",
        CONFIG_GD32F4_ROMFS_MOUNTPOINT, MOUNT_DEVNAME);

  ret = nx_mount(MOUNT_DEVNAME, CONFIG_GD32F4_ROMFS_MOUNTPOINT,
                 "romfs", MS_RDONLY, NULL);
  if (ret < 0)
    {
      ferr("ERROR: nx_mount(%s,%s,romfs) failed: %d\n",
           MOUNT_DEVNAME, CONFIG_GD32F4_ROMFS_MOUNTPOINT, ret);
      return ret;
    }

  return OK;
}

#endif /* CONFIG_GD32F4_ROMFS */
