/****************************************************************************
 * boards/arm/stm32wb/nucleo-wb55rg/src/stm32_boot.c
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
#include <sys/mount.h>
#include <stdio.h>
#include <syslog.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/sdio.h>

#include <arch/board/board.h>

#include "stm32wb_tim.h"

#ifdef CONFIG_RTC_DRIVER
#  include <nuttx/timers/rtc.h>
#  include "stm32wb_rtc.h"
#endif

#ifdef CONFIG_STM32WB_BLE
#  include "stm32wb_blehci.h"
#endif


#include "nucleo-wb55rg.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32wb_board_initialize
 *
 * Description:
 *   All STM32WB architectures must provide the following entry point.  This
 *   entry point is called early in the initialization -- after all memory
 *   has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void stm32wb_board_initialize(void)
{
  /* Configure on-board LEDs if LED support has been selected. */

#ifdef CONFIG_ARCH_LEDS
  board_autoled_initialize();
#endif
}

/****************************************************************************
 * Name: arm_netinitialize
 *
 * Description:
 *   Dummy function expected to start-up logic.
 *
 ****************************************************************************/

#if defined(CONFIG_NET) && !defined(CONFIG_NETDEV_LATEINIT)
void arm_netinitialize(void)
{
}
#endif

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
#ifdef CONFIG_RTC_DRIVER
  struct rtc_lowerhalf_s *rtclower;
#endif
  int ret = OK;

#ifdef HAVE_PROC
  /* Mount the proc filesystem */

  syslog(LOG_INFO, "Mounting procfs to /proc\n");

  ret = nx_mount(NULL, CONFIG_NSH_PROC_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to mount the PROC filesystem: %d (%d)\n",
             ret, errno);
      return;
    }
#endif

#ifdef CONFIG_RTC_DRIVER
  /* Instantiate the STM32WB lower-half RTC driver */

  rtclower = stm32wb_rtc_lowerhalf();
  if (!rtclower)
    {
      serr("ERROR: Failed to instantiate the RTC lower-half driver\n");
      return;
    }
  else
    {
      /* Bind the lower half driver and register the combined RTC driver
       * as /dev/rtc0
       */

      ret = rtc_initialize(0, rtclower);
      if (ret < 0)
        {
          serr("ERROR: Failed to bind/register the RTC driver: %d\n", ret);
          return;
        }
    }
#endif

#ifdef CONFIG_TIMER
  /* Initialize and register the timer driver */

  ret = stm32wb_timer_initialize("/dev/timer0", 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register the timer driver: %d\n",
             ret);
      return;
    }
#endif

#ifdef CONFIG_STM32WB_BLE
  /* Initialize and register BLE HCI driver */

  stm32wb_blehci_initialize();
#endif
  UNUSED(ret);
}
#endif

#ifdef CONFIG_BOARDCTL_IOCTL
int board_ioctl(unsigned int cmd, uintptr_t arg)
{
  return -ENOTTY;
}
#endif

#if defined(CONFIG_BOARDCTL_UNIQUEID)
int board_uniqueid(uint8_t *uniqueid)
{
  if (uniqueid == NULL)
    {
      return -EINVAL;
    }

  stm32wb_get_uniqueid(uniqueid);
  return OK;
}
#endif
