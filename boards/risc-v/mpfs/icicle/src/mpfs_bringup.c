/****************************************************************************
 * boards/risc-v/mpfs/icicle/src/mpfs_bringup.c
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
#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/drivers/ramdisk.h>

#ifdef CONFIG_MPFS_ROMFS_MOUNT
#  include <arch/board/boot_romfsimg.h>
#endif

#include "board_config.h"
#include "mpfs_corepwm.h"
#include "mpfs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_MPFS_ROMFS_MOUNT
#  define SECTORSIZE    512
#  define NSECTORS(b)   (((b) + SECTORSIZE - 1) / SECTORSIZE)
#endif /* CONFIG_MPFS_ROMFS_MOUNT */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_bringup
 ****************************************************************************/

int mpfs_bringup(void)
{
  int ret = OK;

#ifdef CONFIG_USBDEV
  /* Configure USB device driver */

  ret = mpfs_board_usb_init();

  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize USB driver: %d\n", ret);
    }
#endif

#if defined(CONFIG_I2C_DRIVER)
  /* Configure I2C peripheral interfaces */

  ret = mpfs_board_i2c_init();

  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize I2C driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      serr("ERROR: Failed to mount procfs at %s: %d\n", "/proc", ret);
    }
#endif

#ifdef CONFIG_MPFS_ROMFS_MOUNT
  /* Create a ROM disk for the /bin filesystem */

  ret = romdisk_register(0, romfs_img, NSECTORS(romfs_img_len), SECTORSIZE);
  if (ret < 0)
    {
      serr("ERROR: Failed to register romdisk: %d\n", -ret);
    }
  else
    {
      /* Mount the file system */

      ret = mount("/dev/ram0", "/bin", "romfs", MS_RDONLY, NULL);
      if (ret < 0)
        {
          serr("ERROR: Failed to mount romfs: %d\n", -ret);
        }
    }
#endif /* CONFIG_MPFS_ROMFS_MOUNT */

#if defined(CONFIG_MPFS_SPI0) || defined(CONFIG_MPFS_SPI1)
  /* Configure SPI peripheral interfaces */

  ret = mpfs_board_spi_init();

  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize SPI driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_MPFS_HAVE_COREPWM
  /* Configure PWM peripheral interfaces */

  ret = mpfs_pwm_setup();

  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize CorePWM driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_MPFS_EMMCSD
  ret = mpfs_board_emmcsd_init();

  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to init eMMCSD driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_MPFS_IHC_CLIENT
  ret = mpfs_board_ihc_init();

  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to init IHC driver: %d\n", ret);
    }
#endif

  return ret;
}
