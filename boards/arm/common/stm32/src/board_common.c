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

#if defined(CONFIG_STM32_IWDG) || defined(CONFIG_STM32_WWDG)
#  include "stm32_wdg.h"
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

  UNUSED(ret);
  return OK;
}
