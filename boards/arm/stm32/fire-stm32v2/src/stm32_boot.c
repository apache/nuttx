/****************************************************************************
 * boards/arm/stm32/fire-stm32v2/src/stm32_boot.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/i2c/i2c_master.h>
#include <arch/board/board.h>

#include "stm32.h"
#include "stm32_i2c.h"
#include "arm_internal.h"
#include "fire-stm32v2.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Assume that we support everything until convinced otherwise */

#define HAVE_MMCSD    1
#define HAVE_USBDEV   1
#define HAVE_W25      1

/* Configuration ************************************************************/

/* SPI1 connects to the SD CARD (and to the SPI FLASH) */

#define STM32_MMCSDSPIPORTNO   1  /* SPI1 */
#define STM32_MMCSDSLOTNO      0  /* Only one slot */

/* Can't support MMC/SD features if the SDIO peripheral is disabled */

#ifndef CONFIG_STM32_SDIO
#  undef HAVE_MMCSD
#endif

/* Can't support MMC/SD features if mountpoints are disabled */

#ifdef CONFIG_DISABLE_MOUNTPOINT
#  undef HAVE_MMCSD
#endif

/* Default MMC/SD minor number */

#ifdef HAVE_MMCSD
#  ifndef CONFIG_NSH_MMCSDMINOR
#    define CONFIG_NSH_MMCSDMINOR 0
#  endif

/* Default MMC/SD SLOT number */

#  if defined(CONFIG_NSH_MMCSDSLOTNO) && CONFIG_NSH_MMCSDSLOTNO != STM32_MMCSDSLOTNO
#    error "Only one MMC/SD slot:  Slot 0"
#    undef  CONFIG_NSH_MMCSDSLOTNO
#    define CONFIG_NSH_MMCSDSLOTNO STM32_MMCSDSLOTNO
#  endif

#  ifndef CONFIG_NSH_MMCSDSLOTNO
#    define CONFIG_NSH_MMCSDSLOTNO STM32_MMCSDSLOTNO
#  endif
#endif

/* Can't support the W25 device if it SPI1 or W25 support is not enabled */

#if !defined(CONFIG_STM32_SPI1) || !defined(CONFIG_MTD_W25)
#  undef HAVE_W25
#endif

/* Can't support W25 features if mountpoints are disabled */

#if defined(CONFIG_DISABLE_MOUNTPOINT)
#  undef HAVE_W25
#endif

/* Default W25 minor number */

#if defined(HAVE_W25) && !defined(CONFIG_NSH_W25MINOR)
#  define CONFIG_NSH_W25MINOR 0
#endif

/* Can't support USB host or device features if the USB peripheral or the USB
 * device infrastructure is not enabled
 */

#if !defined(CONFIG_STM32_USB) || !defined(CONFIG_USBDEV)
#  undef HAVE_USBDEV
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_i2c_register
 *
 * Description:
 *   Register one I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#ifdef HAVE_I2CTOOL
static void stm32_i2c_register(int bus)
{
  struct i2c_master_s *i2c;
  int ret;

  i2c = stm32_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      _err("ERROR: Failed to get I2C%d interface\n", bus);
    }
  else
    {
      ret = i2c_register(i2c, bus);
      if (ret < 0)
        {
          _err("ERROR: Failed to register I2C%d driver: %d\n", bus, ret);
          stm32_i2cbus_uninitialize(i2c);
        }
    }
}
#endif

/****************************************************************************
 * Name: stm32_i2ctool
 *
 * Description:
 *   Register I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#ifdef HAVE_I2CTOOL
static void stm32_i2ctool(void)
{
#ifdef CONFIG_STM32_I2C1
  stm32_i2c_register(1);
#endif
#ifdef CONFIG_STM32_I2C2
  stm32_i2c_register(2);
#endif
#ifdef CONFIG_STM32_I2C3
  stm32_i2c_register(3);
#endif
}
#else
#  define stm32_i2ctool()
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void stm32_boardinitialize(void)
{
  /* Configure SPI chip selects if 1) SPI is not disabled, and 2) the weak
   * function stm32_spidev_initialize() has been brought into the link.
   */

#if defined(CONFIG_STM32_SPI1) || defined(CONFIG_STM32_SPI2)
  if (stm32_spidev_initialize)
    {
      stm32_spidev_initialize();
    }
#endif

  /* Initialize USB is 1) USBDEV is selected, 2) the USB controller is not
   * disabled, and 3) the weak function stm32_usbinitialize() has been
   * brought into the build.
   */

#if defined(CONFIG_USBDEV) && defined(CONFIG_STM32_USB)
  if (stm32_usbinitialize)
    {
      stm32_usbinitialize();
    }
#endif

  /* Configure on-board LEDs if LED support has been selected. */

#ifdef CONFIG_ARCH_LEDS
  board_autoled_initialize();
#endif
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize().  board_late_initialize() will
 *   be called immediately after up_initialize() is called and just before
 *   the initial application is started.  This additional initialization
 *   phase may be used, for example, to initialize board-specific device
 *   drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
#if defined(HAVE_MMCSD) || defined(HAVE_W25)
  int ret;
#endif

  /* Register I2C drivers on behalf of the I2C tool */

  stm32_i2ctool();

#ifdef HAVE_W25
  /* Initialize and register the W25 FLASH file system. */

  ret = stm32_w25initialize(CONFIG_NSH_W25MINOR);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize W25 minor %d: %d\n",
             CONFIG_NSH_W25MINOR, ret);
      return;
    }
#endif

#ifdef HAVE_MMCSD
  /* Initialize the SDIO-based MMC/SD slot */

  ret = stm32_sdinitialize(CONFIG_NSH_MMCSDMINOR);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize MMC/SD slot %d: %d\n",
             CONFIG_NSH_MMCSDSLOTNO, ret);
      return;
    }
#endif
}
#endif
