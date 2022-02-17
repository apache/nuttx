/****************************************************************************
 * boards/arm/stm32/nucleo-l152re/src/stm32_appinitialize.c
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
#include <syslog.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/fs/fs.h>
#include <nuttx/leds/userled.h>

#include "stm32_i2c.h"

#include "nucleo-l152re.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef HAVE_LEDS
#undef HAVE_DAC

#if !defined(CONFIG_ARCH_LEDS) && defined(CONFIG_USERLED_LOWER)
#  define HAVE_LEDS 1
#endif

#if defined(CONFIG_DAC)
#  define HAVE_DAC 1
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initialization logic and the
 *         matching application logic.  The value could be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int board_app_initialize(uintptr_t arg)
{
  int ret;
#ifdef CONFIG_STM32_I2C1
  FAR struct i2c_master_s *i2c1;
#endif
#ifdef CONFIG_STM32_I2C2
  FAR struct i2c_master_s *i2c2;
#endif

#ifdef CONFIG_STM32_I2C1
  /* Get the I2C lower half instance */

  i2c1 = stm32_i2cbus_initialize(1);
  if (i2c1 == NULL)
    {
      i2cerr("ERROR: Initialize I2C1: %d\n", ret);
    }
  else
    {
      /* Register the I2C character driver */

      ret = i2c_register(i2c1, 1);
      if (ret < 0)
        {
          i2cerr("ERROR: Failed to register I2C1 device: %d\n", ret);
        }
    }
#endif

#ifdef CONFIG_STM32_I2C2
  /* Get the I2C lower half instance */

  i2c2 = stm32_i2cbus_initialize(2);
  if (i2c2 == NULL)
    {
      i2cerr("ERROR: Initialize I2C2: %d\n", ret);
    }
  else
    {
      /* Register the I2C character driver */

      ret = i2c_register(i2c2, 2);
      if (ret < 0)
        {
          i2cerr("ERROR: Failed to register I2C2 device: %d\n", ret);
        }
    }
#endif

#ifdef HAVE_LEDS
  /* Register the LED driver */

  ret = userled_lower_initialize(LED_DRIVER_PATH);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(0, STM32_PROCFS_MOUNTPOINT, "procfs", 0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at %s: %d\n",
             STM32_PROCFS_MOUNTPOINT, ret);
    }
#endif

#ifdef CONFIG_MMCSD_SPI

  /* Initialize the MMC/SD SPI driver (SPI1 is used) */

  ret = stm32_spisd_initialize(1, CONFIG_NSH_MMCSDMINOR);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize SD slot %d: %d\n",
             CONFIG_NSH_MMCSDMINOR, ret);
    }
#endif

  UNUSED(ret);
  return OK;
}
