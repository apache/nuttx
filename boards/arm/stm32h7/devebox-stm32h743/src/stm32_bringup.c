/****************************************************************************
 * boards/arm/stm32h7/devebox-stm32h743/src/stm32_bringup.c
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
#include <errno.h>

#include <arch/board/board.h>

#include <nuttx/fs/fs.h>
#include <nuttx/input/buttons.h>
#include <nuttx/usb/cdcacm.h>
#include <nuttx/mmcsd.h>

#include "devebox-stm32h743.h"

#include "stm32_gpio.h"
#include "stm32_gpio_helper.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

#if (defined(CONFIG_USART1_SERIAL_CONSOLE) && \
       (GPIO_OTGFS_ID & GPIO_PIN_MASK) == GPIO_PIN10 && \
       (GPIO_OTGFS_ID & GPIO_PORT_MASK) == GPIO_PORTA)
#  if (defined(CONFIG_STM32_OTGFS) && !defined(CONFIG_OTG_ID_GPIO_DISABLE))
#    error "CONFLICT: OTGFS_ID pin uses PA10, which is also USART1 RX. \
            Enable CONFIG_OTG_ID_GPIO_DISABLE to disable ID pin, or change console UART pins."
#  endif
#  if (defined(CONFIG_STM32_OTGHS) && !defined(CONFIG_OTG_HS_ID_DISABLE))
#    error "CONFLICT: OTGHS_ID pin uses PA10, which is also USART1 RX. \
            Enable CONFIG_OTG_HS_ID_DISABLE to disable ID pin, or change console UART pins."
#  endif
#endif

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

#if defined(CONFIG_FAT_DMAMEMORY)
  if (stm32_dma_alloc_init() < 0)
    {
      syslog(LOG_ERR, "DMA alloc FAILED");
    }
#endif

#ifdef HAVE_SDIO
  /* Initialize the SDIO block driver */

  ret = stm32_sdio_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize MMC/SD driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_MTD_W25QXXXJV
  ret = stm32_w25qxxx_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_w25qxxx_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_INPUT_BUTTONS
#ifdef CONFIG_INPUT_BUTTONS_LOWER
  ret = btn_lower_initialize("/dev/buttons");
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
      return ret;
    }
#else
  board_button_initialize();
#endif
#endif

#ifdef CONFIG_CDCACM
  ret = cdcacm_initialize(0, NULL);
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: cdcacm_initialize() failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_DAC
  ret = board_dac_initialize();
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: board_dac_initialize() failed: %d\n", ret);
      return ret;
    }
#endif

  return OK;
}
