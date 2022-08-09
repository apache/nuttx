/****************************************************************************
 * boards/arm/s32k1xx/rddrone-bms772/src/s32k1xx_bringup.c
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

/* Copyright 2022 NXP */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <syslog.h>

#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

#ifdef CONFIG_FS_PROCFS
#  include <nuttx/fs/fs.h>

#  ifdef CONFIG_S32K1XX_RESETCAUSE_PROCFS
#    include "s32k1xx_resetcause_procfs.h"
#  endif
#endif

#ifdef CONFIG_S32K1XX_PROGMEM
#  include <nuttx/mtd/mtd.h>
#endif

#ifdef CONFIG_S32K1XX_EEEPROM
#  include "s32k1xx_eeeprom.h"
#endif

#ifdef CONFIG_SMBUS_SBD
#  include <nuttx/i2c/i2c_slave.h>
#  include "s32k1xx_lpi2c_slave.h"
#  include <arch/board/smbus_sbd.h>
#endif

#ifdef CONFIG_S32K1XX_FLEXCAN
#  include "s32k1xx_flexcan.h"
#endif

#ifdef CONFIG_VIDEO_FB
#   include <nuttx/video/fb.h>
#endif

#include "rddrone-bms772.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int s32k1xx_bringup(void)
{
  int ret = OK;

#ifdef CONFIG_USERLED
  /* Register the LED driver */

  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_PROCFS
  /* Register procfs entries before mounting */

#  ifdef CONFIG_S32K1XX_RESETCAUSE_PROCFS
  ret = s32k1xx_resetcause_procfs_register();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register MCU Reset Cause PROCFS entry: %d\n",
             ret);
    }
#  endif /* CONFIG_S32K1XX_RESETCAUSE_PROCFS */

#  ifdef CONFIG_S32K1XX_NRSTCHECK_PROCFS
  ret = s32k1xx_nrstcheck_procfs_register();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register nRST Check PROCFS entry: %d\n", ret);
    }
#  endif /* CONFIG_S32K1XX_NRSTCHECK_PROCFS */

  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif /* CONFIG_FS_PROCFS */

#ifdef CONFIG_DEV_GPIO
  /* Initialize and register the GPIO driver */

  ret = s32k1xx_gpio_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize GPIO driver: %d\n", ret);
    }
#endif /* CONFIG_DEV_GPIO */

#ifdef CONFIG_S32K1XX_PROGMEM
  struct mtd_dev_s *mtd;

  mtd = progmem_initialize();
  if (mtd == NULL)
    {
      syslog(LOG_ERR, "ERROR: progmem_initialize() failed\n");
    }
#endif /* CONFIG_S32K1XX_PROGMEM */

#ifdef CONFIG_S32K1XX_EEEPROM
  /* Register EEEPROM block device */

  ret = s32k1xx_eeeprom_register(0, 4096);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: s32k1xx_eeeprom_register() failed\n");
    }
#endif /* CONFIG_S32K1XX_EEEPROM */

#ifdef CONFIG_S32K1XX_LPI2C
  /* Initialize I2C driver */

  ret = s32k1xx_i2cdev_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: s32k1xx_i2cdev_initialize() failed: %d\n",
             ret);
    }
#endif /* CONFIG_S32K1XX_LPI2C */

#ifdef CONFIG_VIDEO_FB
  /* Register the framebuffer device for the display */

  ret = fb_register(0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fb_register() failed: %d\n", ret);
    }
#endif /* CONFIG_VIDEO_FB */

#if defined(CONFIG_S32K1XX_LPI2C) && defined(CONFIG_I2C_SLAVE) && \
    defined(CONFIG_SMBUS_SBD)
  /* Initialize I2C slave device */

  struct i2c_slave_s *lpi2c0_slave = s32k1xx_i2cbus_slave_initialize(0);
  if (lpi2c0_slave == NULL)
    {
      syslog(LOG_ERR, "ERROR: s32k1xx_i2cbus_slave_initialize() failed\n");
    }

  /* Initialize a new SMBus Smart Battery Data slave device as
   * /dev/smbus-sbd0
   */

  ret = smbus_sbd_initialize(0, lpi2c0_slave);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: smbus_sbd_initialize() failed: %d\n", ret);
    }
#endif /* CONFIG_S32K1XX_LPI2C && CONFIG_I2C_SLAVE && CONFIG_SMBUS_SBD */

#ifdef CONFIG_S32K1XX_LPSPI
  /* Initialize SPI driver */

  ret = s32k1xx_spidev_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: s32k1xx_spidev_initialize() failed: %d\n",
             ret);
    }
#endif /* CONFIG_S32K1XX_LPSPI */

#if defined(CONFIG_S32K1XX_FLEXCAN0) && defined(CONFIG_NETDEV_LATEINIT)
  /* Initialize FlexCAN0 driver */

  ret = s32k1xx_caninitialize(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: s32k1xx_caninitialize(0) failed: %d\n", ret);
    }
#endif /* CONFIG_S32K1XX_FLEXCAN0 && CONFIG_NETDEV_LATEINIT */

  return ret;
}
