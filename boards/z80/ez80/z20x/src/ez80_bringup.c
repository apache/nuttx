/****************************************************************************
 * boards/z80/ez80/z20x/src/ez80_bringup.c
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

#include <nuttx/fs/fs.h>

#include "z20x.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ez80_bringup
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

int ez80_bringup(void)
{
  int ret = OK;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Failed to mount procfs at /proc: %d\n", ret);
      return ret;
    }
#endif

#ifdef HAVE_SPIFLASH
  /* Initialize and register the W25 FLASH file system. */

  ret = ez80_w25_initialize(CONFIG_Z20X_W25_MINOR);
  if (ret < 0)
    {
      ferr("ERROR: Failed to initialize W25 minor %d: %d\n", 0, ret);
      return ret;
    }
#endif

#ifdef HAVE_MMCSD
  /* Initialize SPI-based SD card slot */

  ret = ez80_mmcsd_initialize();
  if (ret < 0)
    {
      mcerr("ERROR: Failed to initialize SD card: %d\n", ret);
      return ret;
    }
#endif

  UNUSED(ret);
  return OK;
}
