/****************************************************************************
 * boards/arm/stm32h7/nucleo-h745zi/src/stm32_bringup.c
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
#include <errno.h>

#include <arch/board/board.h>

#include <nuttx/fs/fs.h>

#ifdef CONFIG_RPTUN
#  include "stm32_rptun.h"
#endif
#ifdef CONFIG_RPMSG_UART
#  include <nuttx/serial/uart_rpmsg.h>
#endif

#include "nucleo-h745zi.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_RPMSG_UART
/****************************************************************************
 * Name: rpmsg_serialinit
 ****************************************************************************/

void rpmsg_serialinit(void)
{
#ifdef CONFIG_ARCH_CHIP_STM32H7_CORTEXM7
  uart_rpmsg_init("cm7", "proxy", 4096, false);
#endif

#ifdef CONFIG_ARCH_CHIP_STM32H7_CORTEXM4
#  ifdef CONFIG_RPMSG_UART_CONSOLE
  uart_rpmsg_init("cm4", "proxy", 4096, true);
#  else
  uart_rpmsg_init("cm4", "proxy", 4096, false);
#  endif
#endif
}
#endif

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y &&
 *   CONFIG_NSH_ARCHINIT:
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void)
{
  int ret = OK;

  UNUSED(ret);

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, STM32_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to mount the PROC filesystem: %d\n",  ret);
    }
#endif /* CONFIG_FS_PROCFS */

#ifdef CONFIG_RPTUN
#  ifdef CONFIG_ARCH_CHIP_STM32H7_CORTEXM7
  stm32_rptun_init("cm7-shmem", "cm7");
#  else
  stm32_rptun_init("cm4-shmem", "cm4");
#  endif
#endif

  return OK;
}
