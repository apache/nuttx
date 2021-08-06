/****************************************************************************
 * boards/sim/sim/sim/src/sim_zoneinfo.c
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
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/drivers/ramdisk.h>
#include <nuttx/zoneinfo.h>

#ifdef CONFIG_LIBC_ZONEINFO_ROMFS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_LIBC_TZDIR
#  error CONFIG_LIBC_TZDIR is not defined
#endif

#ifdef CONFIG_DISABLE_MOUNTPOINT
#  error "Mountpoint support is disabled"
#endif

#ifndef CONFIG_FS_ROMFS
#  error "ROMFS support not enabled"
#endif

#define SECTORSIZE  64
#define NSECTORS(b) (((b)+SECTORSIZE-1)/SECTORSIZE)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_zoneinfo
 *
 * Description:
 *   Mount the TZ database.  The nuttx/zoneinfo directory contains
 *   logic to create a version of the TZ/Olson database.
 *   This database is required if localtime() support is selected via
 *   CONFIG_LIBC_LOCALTIME.  This logic in that directory does the following:
 *
 *   - It downloads the current TZ database from the IANA website
 *   - It downloads the current timezone tools from the same location
 *   - It builds the tools and constructs the binary TZ database
 *   - It will then, optionally, build a ROMFS filesystem image containing
 *     the data base.
 *
 *   The ROMFS filesystem image can that be mounted during the boot-up
 *   sequence so that it is available for the localtime logic.
 *   There are two steps to doing this:
 *
 *   - First, a ROM disk device must be created.  This is done by calling
 *     the function romdisk_register() as described in
 *     nuttx/include/nuttx/drivers/ramdisk.h.  This is an OS level operation
 *     and must be done in the board-level logic before your application
 *     starts.
 *
 *     romdisk_register() will create a block driver at /dev/ramN where N
 *     is the device minor number that was provided to romdisk_register.
 *
 *   - The second step is to mount the file system.  This step can be
 *     performed either in your board configuration logic or by your
 *     application using the mount() interface described in
 *     nuttx/include/sys/mount.h.
 *
 *     These steps, however, must be done very early in initialization,
 *     before there is any need for time-related services.
 *
 ****************************************************************************/

int sim_zoneinfo(int minor)
{
  char devname[32];
  int  ret;

  /* Create a RAM disk for the test */

  ret = romdisk_register(minor, romfs_zoneinfo_img,
                         NSECTORS(romfs_zoneinfo_img_len), SECTORSIZE);
  if (ret < 0)
    {
      printf("ERROR: Failed to create RAM disk\n");
      return ret;
    }

  /* Use the minor number to create a name for the ROM disk block device */

  snprintf(devname, 32, "/dev/ram%d", minor);

  /* Mount the ROMFS file system */

  printf("Mounting ROMFS filesystem at target=%s with source=%s\n",
         CONFIG_LIBC_TZDIR, devname);

  ret = nx_mount(devname, CONFIG_LIBC_TZDIR, "romfs", MS_RDONLY, NULL);
  if (ret < 0)
    {
      printf("ERROR: Mount failed: %d\n", ret);
      return ret;
    }

  printf("TZ database mounted at %s\n", CONFIG_LIBC_TZDIR);
  return OK;
}

#endif /* CONFIG_LIBC_ZONEINFO_ROMFS */
