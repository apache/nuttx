/****************************************************************************
 * boards/arm/n32h7/n32h762iil7/src/n32_progmem.c
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

#include <sys/mount.h>
#include <sys/param.h>

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/progmem.h>
#include <nuttx/drivers/drivers.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mtd/mtd.h>

#include "n32_flash.h"
#include "n32h762iil7.h"
#include <arch/board/board.h>

#ifdef HAVE_PROGMEM_CHARDEV

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct mtd_dev_s *g_progmem_mtd;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: n32_progmem_init
 *
 *   Initialize Progmem partition. Read partition information, and use
 *   these data for creating MTD.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

int n32_progmem_init(void)
{
  int ret = 0;

  g_progmem_mtd = progmem_initialize();
  if (g_progmem_mtd == NULL)
    {
      ferr("ERROR: Failed to get progmem flash MTD\n");
      ret = -EIO;
    }
  else
    {
      ret = register_mtddriver("/dev/mtd0", g_progmem_mtd, 0775, NULL);
      if (ret < 0)
        {
          ferr("ERROR: Failed to register progmem flash MTD driver\n");
        }
    }

#ifdef CONFIG_FS_LITTLEFS
  /* Mount the LittleFS file system */

  ret = nx_mount("/dev/mtd0", "/flash", "littlefs", 0, "autoformat");
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to mount LittleFS at /flash: %d\n", ret);
      return ret;
    }
#endif /* CONFIG_FS_LITTLEFS */

  return ret;
}

#endif /* HAVE_PROGMEM_CHARDEV */
