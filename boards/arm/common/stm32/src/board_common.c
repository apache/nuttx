/****************************************************************************
 * boards/arm/common/stm32/src/board_common.c
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

#include <sys/types.h>
#include <syslog.h>

#include <nuttx/fs/fs.h>

#include <arch/board/board.h>

/* The STM32H7 FDCAN driver has its own SocketCAN interface */

#if (defined(CONFIG_STM32_CAN_CHARDRIVER) || \
     defined(CONFIG_STM32_FDCAN_CHARDRIVER)) && \
    !defined(CONFIG_STM32_HAVE_FDCAN_H7)
#  define HAVE_CAN_SETUP
#endif

#if (defined(CONFIG_STM32_CAN_SOCKET) || defined(CONFIG_STM32_FDCAN_SOCKET)) && \
    !defined(CONFIG_STM32_HAVE_FDCAN_H7)
#  define HAVE_CANSOCK_SETUP
#endif

#if defined(CONFIG_I2C_DRIVER) && \
    (defined(CONFIG_STM32_I2C1) || defined(CONFIG_STM32_I2C2) || \
     defined(CONFIG_STM32_I2C3) || defined(CONFIG_STM32_I2C4) || \
     defined(CONFIG_STM32_I2C5) || defined(CONFIG_STM32_I2C6))
#  define HAVE_I2C_DRIVER
#endif

#if defined(CONFIG_STM32_IWDG) || defined(CONFIG_STM32_WWDG)
#  include "stm32_wdg.h"
#endif

/* Per-family I2C headers until all STM32 I2C drivers are unified */

#ifdef HAVE_I2C_DRIVER
#  include <nuttx/i2c/i2c_master.h>
#  if defined(CONFIG_ARCH_CHIP_STM32L4)
#    include "stm32l4_i2c.h"
#  elif defined(CONFIG_ARCH_CHIP_STM32WB)
#    include "stm32wb_i2c.h"
#  else
#    include "stm32_i2c.h"
#  endif
#endif

#ifdef HAVE_CAN_SETUP
#  include "stm32_can_setup.h"
#endif

#ifdef HAVE_CANSOCK_SETUP
#  include "stm32_cansock_setup.h"
#endif

#include "board_common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_DISABLE_MOUNTPOINT)
#  define HAVE_PROCFS
#endif

#if defined(CONFIG_FS_TMPFS) && !defined(CONFIG_DISABLE_MOUNTPOINT)
#  define HAVE_TMPFS
#endif

#ifdef CONFIG_NSH_PROC_MOUNTPOINT
#  define PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#else
#  define PROCFS_MOUNTPOINT "/proc"
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_i2c_register
 *
 * Description:
 *   Register the I2C character driver /dev/i2c[bus] for one I2C bus.
 *
 ****************************************************************************/

#ifdef HAVE_I2C_DRIVER
static void stm32_i2c_register(int bus)
{
  struct i2c_master_s *i2c;
  int ret;

  i2c = stm32_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get I2C%d interface\n", bus);
      return;
    }

  ret = i2c_register(i2c, bus);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register I2C%d driver: %d\n",
             bus, ret);
      stm32_i2cbus_uninitialize(i2c);
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_common_initialize
 *
 * Description:
 *   Initialize the board features that are common to all STM32 boards.
 *
 ****************************************************************************/

int stm32_common_initialize(void)
{
  int ret;

#ifdef HAVE_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at %s: %d\n",
             PROCFS_MOUNTPOINT, ret);
    }
#endif

#ifdef HAVE_TMPFS
  /* Mount the tmpfs file system */

  ret = nx_mount(NULL, CONFIG_LIBC_TMPDIR, "tmpfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount tmpfs at %s: %d\n",
             CONFIG_LIBC_TMPDIR, ret);
    }
#endif

#ifdef CONFIG_STM32_IWDG
  /* Initialize the independent watchdog */

  stm32_iwdginitialize("/dev/watchdog0", STM32_LSI_FREQUENCY);
#endif

#ifdef CONFIG_STM32_WWDG
  /* Initialize the window watchdog */

  stm32_wwdginitialize("/dev/watchdog1");
#endif

#ifdef HAVE_I2C_DRIVER
  /* Register the I2C character drivers */

#  ifdef CONFIG_STM32_I2C1
  stm32_i2c_register(1);
#  endif
#  ifdef CONFIG_STM32_I2C2
  stm32_i2c_register(2);
#  endif
#  ifdef CONFIG_STM32_I2C3
  stm32_i2c_register(3);
#  endif
#  ifdef CONFIG_STM32_I2C4
  stm32_i2c_register(4);
#  endif
#  ifdef CONFIG_STM32_I2C5
  stm32_i2c_register(5);
#  endif
#  ifdef CONFIG_STM32_I2C6
  stm32_i2c_register(6);
#  endif
#endif

#ifdef HAVE_CAN_SETUP
  /* Register the CAN character drivers */

  ret = stm32_can_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_can_setup() failed: %d\n", ret);
    }
#endif

#ifdef HAVE_CANSOCK_SETUP
  /* Initialize the SocketCAN interfaces */

  ret = stm32_cansock_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_cansock_setup() failed: %d\n", ret);
    }
#endif

  UNUSED(ret);
  return OK;
}
