/****************************************************************************
 * boards/arm/stm32wl5/nucleo-wl55jc/src/stm32_boot.c
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
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/fs/fs.h>
#include <nuttx/leds/userled.h>
#include <nuttx/input/buttons.h>

#include <arch/board/board.h>

#ifdef CONFIG_VIDEO_FB
#include <nuttx/video/fb.h>
#endif

#include <stm32wl5.h>
#include <stm32wl5_uart.h>
#include <stm32wl5_pwr.h>

#include "arm_internal.h"
#include "nucleo-wl55jc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Define proc mountpoint in case procfs is used but nsh is not */

#ifndef CONFIG_NSH_PROC_MOUNTPOINT
#define CONFIG_NSH_PROC_MOUNTPOINT "/proc"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32wl5_board_initialize
 *
 * Description:
 *   All STM32WL5 architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void stm32wl5_board_initialize(void)
{
  /* Configure on-board LEDs, which are always enabled */

  board_leds_initialize();
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize(). board_late_initialize() will be
 *   called immediately after up_initialize() is called and just before the
 *   initial application is started.  This additional initialization phase
 *   may be used, for example, to initialize board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  int ret;

#if defined(CONFIG_STM32WL5_SPI1) || defined(CONFIG_STM32WL5_SPI2S2)
  stm32wl5_spidev_initialize();
#endif

#if defined(CONFIG_LCD_SSD1680) && !defined(CONFIG_VIDEO_FB)
  board_lcd_initialize();
#endif

#ifdef CONFIG_VIDEO_FB
  ret = fb_register(0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fb_register() failed: %d\n", ret);
    }
#endif

#ifdef HAVE_PROC
  /* Mount the proc filesystem */

  syslog(LOG_INFO, "Mounting procfs to /proc\n");

  ret = nx_mount(NULL, CONFIG_NSH_PROC_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount the PROC filesystem: %d\n",
             ret);
      return;
    }
#endif

#if defined(CONFIG_USERLED_LOWER)
  /* Register the LED driver */

  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_INPUT_BUTTONS_LOWER)
  /* Register the Button driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_ARCH_BOARD_FLASH_MOUNT)
  /* Register partition table for on-board FLASH memory */

  ret = stm32wl5_flash_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32wl5_flash_init() failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_ARCH_BOARD_IPCC)
  /* Register IPCC driver */

  ret = ipcc_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: ipcc_init() failed\n");
    }
#endif

#if defined(CONFIG_ARCH_BOARD_ENABLE_CPU2)
  /* Start second CPU */

  stm32wl5_pwr_boot_c2();
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
  if (uniqueid == 0)
    {
      return -EINVAL;
    }

  stm32wl5_get_uniqueid(uniqueid);
  return OK;
}
#endif
