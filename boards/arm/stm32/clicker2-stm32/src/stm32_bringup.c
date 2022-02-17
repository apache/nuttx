/****************************************************************************
 * boards/arm/stm32/clicker2-stm32/src/stm32_bringup.c
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

#include <nuttx/board.h>
#include <nuttx/fs/fs.h>
#include <nuttx/input/buttons.h>

#ifdef CONFIG_USBMONITOR
#  include <nuttx/usb/usbmonitor.h>
#endif

#ifdef CONFIG_RNDIS
#  include <nuttx/usb/rndis.h>
#  include <net/if.h>
#endif

#include "stm32.h"
#include "clicker2-stm32.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_RNDIS
#  ifndef CONFIG_CLICKER2_STM32_RNDIS_MACADDR
#    define CONFIG_CLICKER2_STM32_RNDIS_MACADDR 0xfadedeadbeef
#  endif
#endif

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
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void)
{
  int ret;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#ifdef CONFIG_STM32_CAN_CHARDRIVER
  /* Initialize CAN and register the CAN driver. */

  ret = stm32_can_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_can_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_ADC
  /* Initialize ADC and register the ADC driver. */

  ret = stm32_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_adc_setup failed: %d\n", ret);
    }
#endif

#ifdef HAVE_USBMONITOR
  /* Start the USB Monitor */

  ret = usbmonitor_start();
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to start USB monitor: %d\n", ret);
    }
#endif

#if defined(CONFIG_CLICKER2_STM32_MB1_BEE) || defined(CONFIG_CLICKER2_STM32_MB2_BEE)
  /* Configure MRF24J40 wireless */

  ret = stm32_mrf24j40_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_mrf24j40_initialize() failed: %d\n",
             ret);
    }
#endif

#if defined(CONFIG_CLICKER2_STM32_MB1_XBEE) || defined(CONFIG_CLICKER2_STM32_MB2_XBEE)
  /* Configure XBee wireless */

  ret = stm32_xbee_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_xbee_initialize() failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_CLICKER2_STM32_MB1_MMCSD_AUTOMOUNT) || \
    defined(CONFIG_CLICKER2_STM32_MB2_MMCSD_AUTOMOUNT)
  /* Configure uSD automounter */

  ret = stm32_automount_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_automount_initialize() failed: %d\n",
             ret);
    }
#endif

#if defined(CONFIG_CLICKER2_STM32_MB1_MMCSD) || defined(CONFIG_CLICKER2_STM32_MB2_MMCSD)
  /* Configure uSD card slot */

  ret = stm32_mmcsd_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_mmcsd_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_INPUT_BUTTONS
  /* Register the BUTTON driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_RNDIS
  uint8_t mac[IFHWADDRLEN];

  mac[0] = (CONFIG_CLICKER2_STM32_RNDIS_MACADDR >> (8 * 5)) & 0xff;
  mac[1] = (CONFIG_CLICKER2_STM32_RNDIS_MACADDR >> (8 * 4)) & 0xff;
  mac[2] = (CONFIG_CLICKER2_STM32_RNDIS_MACADDR >> (8 * 3)) & 0xff;
  mac[3] = (CONFIG_CLICKER2_STM32_RNDIS_MACADDR >> (8 * 2)) & 0xff;
  mac[4] = (CONFIG_CLICKER2_STM32_RNDIS_MACADDR >> (8 * 1)) & 0xff;
  mac[5] = (CONFIG_CLICKER2_STM32_RNDIS_MACADDR >> (8 * 0)) & 0xff;

  /* Register USB RNDIS Driver */

  ret = usbdev_rndis_initialize(mac);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: usbdev_rndis_initialize() failed %d\n", ret);
    }
#endif

  UNUSED(ret);
  return OK;
}
