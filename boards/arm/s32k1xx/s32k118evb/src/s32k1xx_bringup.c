/****************************************************************************
 * boards/arm/s32k1xx/s32k118evb/src/s32k1xx_bringup.c
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

#include <sys/types.h>
#include <stdint.h>
#include <syslog.h>

#ifdef CONFIG_INPUT_BUTTONS
#  include <nuttx/input/buttons.h>
#endif

#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

#ifdef CONFIG_FS_PROCFS
#  include <nuttx/fs/fs.h>
#endif

#ifdef CONFIG_S32K1XX_PROGMEM
#  include <nuttx/mtd/mtd.h>
#endif

#ifdef CONFIG_S32K1XX_EEEPROM
#  include "s32k1xx_eeeprom.h"
#endif

#include "s32k118evb.h"

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

#ifdef CONFIG_INPUT_BUTTONS
  /* Register the BUTTON driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_USERLED
  /* Register the LED driver */

  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#ifdef CONFIG_S32K1XX_PROGMEM
  struct mtd_dev_s *mtd;

  mtd = progmem_initialize();
  if (mtd == NULL)
    {
      syslog(LOG_ERR, "ERROR: progmem_initialize() failed\n");
    }
#endif

#ifdef CONFIG_S32K1XX_EEEPROM
  /* Register EEEPROM block device */

  ret = s32k1xx_eeeprom_register(0, 4096);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: s32k1xx_eeeprom_register() failed\n");
    }
#endif

#ifdef CONFIG_S32K1XX_LPI2C
  /* Initialize I2C driver */

  ret = s32k1xx_i2cdev_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: s32k1xx_i2cdev_initialize() failed: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_S32K1XX_LPSPI
  /* Initialize SPI driver */

  ret = s32k1xx_spidev_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: s32k1xx_spidev_initialize() failed: %d\n",
             ret);
    }
#endif

  return ret;
}
