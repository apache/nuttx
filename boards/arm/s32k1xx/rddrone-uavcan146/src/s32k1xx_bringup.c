/****************************************************************************
 * boards/arm/s32k1xx/rddrone-uavcan146/src/s32k1xx_bringup.c
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
#include <arch/board/board.h>
#include <sys/types.h>
#include <syslog.h>

#include <nuttx/fs/fs.h>

#ifdef CONFIG_INPUT_BUTTONS
#  include <nuttx/input/buttons.h>
#endif

#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

#ifdef CONFIG_I2C_DRIVER
#  include "s32k1xx_pin.h"
#  include <nuttx/i2c/i2c_master.h>
#  include "s32k1xx_lpi2c.h"
#endif

#ifdef CONFIG_S32K1XX_EEEPROM
#  include "s32k1xx_eeeprom.h"
#endif

#include "rddrone-uavcan146.h"

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
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_LIB_BOARDCTL=y :
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

#ifdef CONFIG_S32K1XX_LPSPI
  /* Configure SPI chip selects if 1) SPI is not disabled, and 2) the weak
   * function s32k1xx_spidev_initialize() has been brought into the link.
   */

  s32k1xx_spidev_initialize();
#endif

#if defined(CONFIG_S32K1XX_LPI2C0)
#if defined(CONFIG_I2C_DRIVER)
  FAR struct i2c_master_s *i2c;
  i2c = s32k1xx_i2cbus_initialize(0);

  if (i2c == NULL)
    {
      serr("ERROR: Failed to get I2C interface\n");
    }
  else
    {
      ret = i2c_register(i2c, 0);
      if (ret < 0)
        {
          serr("ERROR: Failed to register I2C driver: %d\n", ret);
          s32k1xx_i2cbus_uninitialize(i2c);
        }
    }
#endif
#endif

#ifdef CONFIG_S32K1XX_PROGMEM
  FAR struct mtd_dev_s *mtd;
  int minor = 0;

  mtd = progmem_initialize();
  if (!mtd)
    {
      syslog(LOG_ERR, "ERROR: progmem_initialize failed\n");
    }
#endif

#ifdef CONFIG_S32K1XX_EEEPROM
  /* Register EEEPROM block device */

  s32k1xx_eeeprom_register(0, 4096);
#endif

#ifdef CONFIG_S32K1XX_FLEXCAN
  s32k1xx_pinconfig(BOARD_REVISION_DETECT_PIN);

  if (s32k1xx_gpioread(BOARD_REVISION_DETECT_PIN))
    {
      /* STB high -> active CAN phy */

      s32k1xx_pinconfig(PIN_CAN0_STB  | GPIO_OUTPUT_ONE);
    }
  else
    {
      /* STB low -> active CAN phy */

      s32k1xx_pinconfig(PIN_CAN0_STB  | GPIO_OUTPUT_ZERO);
    }

#endif

  return ret;
}
