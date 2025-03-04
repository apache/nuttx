/****************************************************************************
 * boards/risc-v/k230/canmv230/src/canmv_init.c
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
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/drivers/ramdisk.h>
#include <sys/mount.h>
#include <sys/boardctl.h>
#include <arch/board/board_memorymap.h>

#ifdef CONFIG_RPMSG_UART
#  include <nuttx/serial/uart_rpmsg.h>
#endif

#if !defined(CONFIG_BUILD_KERNEL) || defined(CONFIG_NUTTSBI)
#  include "k230_hart.h"
#endif

#ifdef CONFIG_RPTUN
#  include "k230_rptun.h"
#endif

#ifdef CONFIG_BUILD_KERNEL
#include "romfs.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
#define SECTORSIZE    512
#define NSECTORS(b)   (((b) + SECTORSIZE - 1) / SECTORSIZE)
#endif /* CONFIG_BUILD_KERNEL */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void debug_dumps(void)
{
  /* Dumps to aid investigation */

#if !defined(CONFIG_BUILD_KERNEL) || defined(CONFIG_NUTTSBI)
  sinfo("is_big=%d\n", k230_hart_is_big());
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int board_reset(int status)
{
  _alert("status=%d, halt now.\n", status);
  while (1)
    {
      asm volatile("wfi");
    }

  return 0;
}

#ifdef CONFIG_RPMSG_UART
/****************************************************************************
 * Name: rpmsg_serialinit
 * Description: initialize /dev/ttyRpmsg device
 ****************************************************************************/

void rpmsg_serialinit(void)
{
#ifdef CONFIG_K230_RPTUN_MASTER
  uart_rpmsg_init("remote", "Rpmsg", 4096, false);
#else
  uart_rpmsg_init("master", "Rpmsg", 4096, true);
#endif
}
#endif

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
  debug_dumps();
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
  debug_dumps();

  /* Perform board-specific initialization */

#ifdef CONFIG_BUILD_KERNEL
  /* Create ROM disk for mount in nx_start_application */

  if (NSECTORS(romfs_img_len) > 1)
    {
      int ret = OK;
      ret = romdisk_register(0, romfs_img, NSECTORS(romfs_img_len),
        SECTORSIZE);
      if (ret < 0)
        {
          ferr("ERROR: Failed to register romfs: %d\n", -ret);
        }
    }
#endif /* CONFIG_BUILD_KERNEL */

#ifdef CONFIG_NSH_ARCHINIT

  mount(NULL, "/proc", "procfs", 0, NULL);

#endif

#ifdef CONFIG_RPTUN
#  ifdef CONFIG_K230_RPTUN_MASTER
  k230_rptun_init("remote");
#  else
  k230_rptun_init("master");
#  endif
#endif
}
