/****************************************************************************
 * boards/arm/stm32/clicker2-stm32/src/stm32_bringup.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/mount.h>
#include <stdbool.h>
#include <stdio.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/board.h>
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
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void)
{
  int ret;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#ifdef CONFIG_CAN
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
      syslog(LOG_ERR, "ERROR: stm32_mrf24j40_initialize() failed: %d\n", ret);
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
      syslog(LOG_ERR, "ERROR: stm32_automount_initialize() failed: %d\n", ret);
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

#ifdef CONFIG_BUTTONS
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
