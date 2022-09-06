/****************************************************************************
 * boards/arm/gd32f4/gd32f450zk-eval/src/gd32f4xx_appinit.c
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
#include <debug.h>
#include <syslog.h>

#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/nxffs.h>
#include <nuttx/leds/userled.h>
#include <nuttx/fs/nxffs.h>

#ifdef CONFIG_INPUT_BUTTONS
#  include <nuttx/input/buttons.h>
#endif

#ifdef CONFIG_GD32F4_ROMFS
#include "gd32f4xx_romfs.h"
#endif

#include "gd32f450z_eval.h"

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
  static bool initialized = false;

  /* Have we already initialized? */

  if (!initialized)
    {
#ifdef CONFIG_FS_PROCFS
      /* Mount the procfs file system */

      ret = nx_mount(NULL, GD32_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to mount procfs at %s: %d\n",
                GD32_PROCFS_MOUNTPOINT, ret);
        }
#endif

#ifdef CONFIG_GD32F4_ROMFS
      /* Mount the romfs partition */

      ret = gd32_romfs_initialize();
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to mount romfs at %s: %d\n",
                CONFIG_GD32F4_ROMFS_MOUNTPOINT, ret);
        }
#endif

#ifdef CONFIG_FS_NXFFS

#  ifdef CONFIG_GD32F4_PROGMEM

      /* Create an instance of the GD32F4 FLASH program memory
       * device driver
       */

      struct mtd_dev_s *mtd = progmem_initialize();
      if (!mtd)
        {
          syslog(LOG_ERR, "ERROR: progmem_initialize failed\n");
        }

      /* Initialize to provide NXFFS on the MTD interface */

      ret = nxffs_initialize(mtd);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: NXFFS initialization failed: %d\n",
                  ret);
        }

      /* Mount the file system */

      ret = nx_mount(NULL, CONFIG_GD32F4_NXFFS_MOUNTPT, "nxffs", 0, NULL);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to mount the NXFFS volume: %d\n",
                  ret);
        }

#  endif

#  ifdef CONFIG_MTD_GD25

      ret = gd32_gd25_automount(0);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to mount the NXFFS \
                 volume on spi flash: %d\n", ret);
        }

#  endif

#endif

#ifdef CONFIG_DEV_GPIO
      /* Register the GPIO driver */

      ret = gd32_gpio_initialize();
      if (ret < 0)
        {
          syslog(LOG_ERR, "Failed to initialize GPIO Driver: %d\n", ret);
          return ret;
        }
#endif

#ifdef CONFIG_INPUT_BUTTONS
#ifdef CONFIG_INPUT_BUTTONS_LOWER
  /* Register the BUTTON driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
      return ret;
    }
#else
  /* Enable BUTTON support for some other purpose */

  board_button_initialize();
#endif /* CONFIG_INPUT_BUTTONS_LOWER */
#endif /* CONFIG_INPUT_BUTTONS */

#if !defined(CONFIG_ARCH_LEDS) && defined(CONFIG_USERLED_LOWER)
      /* Register the LED driver */

      ret = userled_lower_initialize(LED_DRIVER_PATH);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n",
                 ret);
        }
#endif

      /* Now we are initialized */

      initialized = true;
    }

  UNUSED(ret);
  return OK;
}
