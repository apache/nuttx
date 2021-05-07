/****************************************************************************
 * boards/arm/stm32/omnibusf4/src/stm32_romfs_initialize.c
 * This file provides contents of an optional ROMFS volume, mounted at boot.
 *
 *   Copyright (C) 2017 Tomasz Wozniak. All rights reserved.
 *   Author: Tomasz Wozniak <t.wozniak@samsung.com>
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

#include <sys/mount.h>
#include <sys/types.h>
#include <stdint.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/drivers/ramdisk.h>
#include "stm32_romfs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_STM32_ROMFS
#  error "CONFIG_STM32_ROMFS must be defined"
#else

#ifndef CONFIG_STM32_ROMFS_IMAGEFILE
#  error "CONFIG_STM32_ROMFS_IMAGEFILE must be defined"
#endif

#ifndef CONFIG_STM32_ROMFS_DEV_MINOR
#  error "CONFIG_STM32_ROMFS_DEV_MINOR must be defined"
#endif

#ifndef CONFIG_STM32_ROMFS_MOUNTPOINT
#  error "CONFIG_STM32_ROMFS_MOUNTPOINT must be defined"
#endif

#define NSECTORS(size) (((size) + ROMFS_SECTOR_SIZE - 1)/ROMFS_SECTOR_SIZE)

#define STR2(m)  #m
#define STR(m) STR2(m)

#define MKMOUNT_DEVNAME(m) "/dev/ram" STR(m)
#define MOUNT_DEVNAME      MKMOUNT_DEVNAME(CONFIG_STM32_ROMFS_DEV_MINOR)

/****************************************************************************
 * Private Data
 ****************************************************************************/

__asm__ (
    ".section .rodata\n"
    ".balign  16\n"
    ".globl   romfs_data_begin\n"
"romfs_data_begin:\n"
    ".incbin " STR(CONFIG_STM32_ROMFS_IMAGEFILE) "\n"\
    \
    ".balign " STR(ROMFS_SECTOR_SIZE) "\n"
    ".globl   romfs_data_end\n"
"romfs_data_end:\n"
    ".globl   romfs_data_size\n"
"romfs_data_size:\n"
    ".word romfs_data_end - romfs_data_begin\n");

extern const char romfs_data_begin;
extern const char romfs_data_end;
extern const int  romfs_data_size;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_romfs_initialize
 *
 * Description:
 *   Registers the aboveincluded binary file as block device.
 *   Then mounts the block device as ROMFS filesystems.
 *
 * Returned Value:
 *   Zero (OK) on success, a negated errno value on error.
 *
 * Assumptions/Limitations:
 *   Memory addresses [&romfs_data_begin .. &romfs_data_begin) should contain
 *   ROMFS volume data, as included in the assembly snippet above (l. 84).
 *
 ****************************************************************************/

int stm32_romfs_initialize(void)
{
  uintptr_t romfs_data_len
  int  ret;

  /* Create a ROM disk for the /etc filesystem */

  romfs_data_len = (uintptr_t)&romfs_data_end - (uintptr_t)&romfs_data_begin;

  ret = romdisk_register(CONFIG_STM32_ROMFS_DEV_MINOR, &romfs_data_begin,
                         NSECTORS(romfs_data_len), ROMFS_SECTOR_SIZE);
  if (ret < 0)
    {
      ferr("ERROR: romdisk_register failed: %d\n", -ret);
      return ret;
    }

  /* Mount the file system */

  finfo("Mounting ROMFS filesystem at target=%s with source=%s\n",
        CONFIG_STM32_ROMFS_MOUNTPOINT, MOUNT_DEVNAME);

  ret = nx_mount(MOUNT_DEVNAME, CONFIG_STM32_ROMFS_MOUNTPOINT,
                 "romfs", MS_RDONLY, NULL);
  if (ret < 0)
    {
      ferr("ERROR: nx_mount(%s,%s,romfs) failed: %d\n",
           MOUNT_DEVNAME, CONFIG_STM32_ROMFS_MOUNTPOINT, ret);
      return ret;
    }

  return OK;
}

#endif /* CONFIG_STM32_ROMFS */
