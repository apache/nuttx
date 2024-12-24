/****************************************************************************
 * boards/risc-v/bl808/ox64/src/bl808_appinit.c
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
#include <syslog.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/drivers/ramdisk.h>
#include <sys/mount.h>
#include <sys/boardctl.h>
#include <arch/board/board_memorymap.h>

#ifdef CONFIG_USERLED
#include <nuttx/leds/userled.h>
#endif
#if defined(CONFIG_BL808_I2C0) || defined(CONFIG_BL808_I2C1) \
  || defined(CONFIG_BL808_I2C2) || defined(CONFIG_BL808_I2C3)
#include "bl808_i2c.h"
#endif
#if defined(CONFIG_BL808_SPI0) || defined(CONFIG_BL808_SPI1)
#include "bl808_spi.h"
#endif
#ifdef CONFIG_BL808_TIMERS
#include "bl808_timer.h"
#endif
#ifdef CONFIG_BL808_WDT
#include "bl808_wdt.h"
#endif
#include "bl808_gpadc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Ramdisk Definition */

#define SECTORSIZE   512
#define NSECTORS(b)  (((b) + SECTORSIZE - 1) / SECTORSIZE)
#define RAMDISK_DEVICE_MINOR 0

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mount_ramdisk
 *
 * Description:
 *  Mount a ramdisk defined in the ld.script to /dev/ramX.  The ramdisk is
 *  intended to contain a romfs with applications which can be spawned at
 *  runtime.
 *
 * Returned Value:
 *   OK is returned on success.
 *   -ERRORNO is returned on failure.
 *
 ****************************************************************************/

static int mount_ramdisk(void)
{
  int ret;
  struct boardioc_romdisk_s desc;

  desc.minor    = RAMDISK_DEVICE_MINOR;
  desc.nsectors = NSECTORS((ssize_t)__ramdisk_size);
  desc.sectsize = SECTORSIZE;
  desc.image    = __ramdisk_start;

  ret = boardctl(BOARDIOC_ROMDISK, (uintptr_t)&desc);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Ramdisk register failed: %s\n", strerror(errno));
      syslog(LOG_ERR, "Ramdisk mountpoint /dev/ram%d\n",
             RAMDISK_DEVICE_MINOR);
      syslog(LOG_ERR, "Ramdisk length %lu, origin %lx\n",
             (ssize_t)__ramdisk_size, (uintptr_t)__ramdisk_start);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform architecture specific initialization
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
#ifdef CONFIG_BOARD_LATE_INITIALIZE
  /* Board initialization already performed by board_late_initialize() */

  return OK;
#else
  /* Perform board-specific initialization */

#ifdef CONFIG_NSH_ARCHINIT

  mount(NULL, "/proc", "procfs", 0, NULL);

#endif

  return OK;
#endif
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize().  board_late_initialize() will
 *   be called after up_initialize() and board_early_initialize() and just
 *   before the initial application is started.  This additional
 *   initialization phase may be used, for example, to initialize board-
 *   specific device drivers for which board_early_initialize() is not
 *   suitable.
 *
 *   Waiting for events, use of I2C, SPI, etc are permissible in the context
 *   of board_late_initialize().  That is because board_late_initialize()
 *   will run on a temporary, internal kernel thread.
 *
 ****************************************************************************/

void board_late_initialize(void)
{
  /* Mount the RAM Disk */

  mount_ramdisk();

  /* Perform board-specific initialization */

#ifdef CONFIG_BL808_GPADC
  bl808_gpadc_init();
#endif

#ifdef CONFIG_BL808_I2C0
  struct i2c_master_s *i2c0 = bl808_i2cbus_initialize(0);
  i2c_register(i2c0, 0);
#endif

#ifdef CONFIG_BL808_I2C1
  struct i2c_master_s *i2c1 = bl808_i2cbus_initialize(1);
  i2c_register(i2c1, 1);
#endif

#ifdef CONFIG_BL808_I2C2
  struct i2c_master_s *i2c2 = bl808_i2cbus_initialize(2);
  i2c_register(i2c2, 2);
#endif

#ifdef CONFIG_BL808_I2C3
  struct i2c_master_s *i2c3 = bl808_i2cbus_initialize(3);
  i2c_register(i2c3, 3);
#endif

#ifdef CONFIG_BL808_SPI0
  struct spi_dev_s *spi0 = bl808_spibus_initialize(0);
  spi_register(spi0, 0);
#endif

#ifdef CONFIG_BL808_SPI1
  struct spi_dev_s *spi1 = bl808_spibus_initialize(1);
  spi_register(spi1, 1);
#endif

#ifdef CONFIG_BL808_TIMERS
  bl808_timer_init();
#endif

#ifdef CONFIG_BL808_WDT
  bl808_wdt_init();
#endif

#ifdef CONFIG_NSH_ARCHINIT

  mount(NULL, "/proc", "procfs", 0, NULL);

#endif

#ifdef CONFIG_USERLED
  /* Register the LED driver */

  int ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#endif
}
