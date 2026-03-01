/****************************************************************************
 * boards/x86_64/qemu/qemu-intel64/src/qemu_boardinit.c
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
#include <nuttx/board.h>

#include <debug.h>

#include <nuttx/drivers/ramdisk.h>

#include "x86_64_internal.h"
#include "qemu_intel64.h"
#include "romfs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Additional checks for kernel build */

#ifdef CONFIG_BUILD_KERNEL
#  ifndef CONFIG_BOARD_LATE_INITIALIZE
#    error KERNEL build require CONFIG_BOARD_LATE_INITIALIZE
#  endif
#endif

#define SECTORSIZE   512
#define NSECTORS(b)  (((b) + SECTORSIZE - 1) / SECTORSIZE)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  int ret = OK;

  UNUSED(ret);

  /* Perform board-specific initialization */

  ret = qemu_bringup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: qemu_bringup() failed: %d\n", ret);
    }

  /* Create ROM disk for mount in nx_start_application */

  if (NSECTORS(romfs_img_len) > 1)
    {
      ret = romdisk_register(0, romfs_img, NSECTORS(romfs_img_len),
                             SECTORSIZE);
      if (ret < 0)
        {
          ferr("ERROR: Failed to register romfs: %d\n", -ret);
        }
    }
}
#endif
