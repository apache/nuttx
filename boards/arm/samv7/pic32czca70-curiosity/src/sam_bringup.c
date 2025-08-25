/****************************************************************************
 * boards/arm/samv7/pic32czca70-curiosity/src/sam_bringup.c
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
#include <syslog.h>
#include <debug.h>
#include <nuttx/signal.h>

#include <nuttx/drivers/drivers.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/nxffs.h>

#include "sam_board.h"

#ifdef HAVE_LED_DRIVER
#  include <nuttx/leds/userled.h>
#endif

#ifdef CONFIG_CDCACM
#include <nuttx/usb/cdcacm.h>
#endif

#ifdef CONFIG_INPUT_BUTTONS
#  include <nuttx/input/buttons.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int sam_bringup(void)
{
  int ret;

/* Procfs mount *************************************************************
 */

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, SAMV71_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at %s: %d\n",
             SAMV71_PROCFS_MOUNTPOINT, ret);
    }
#endif

/* Embedded Progmem *********************************************************
 */

#ifdef HAVE_PROGMEM_CHARDEV
  /* Initialize the FLASH programming memory library */

  ret = sam_flash_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize embedded flash: %d\n",
             ret);
    }
#endif

/* SD Card support **********************************************************
 */

#ifdef HAVE_HSMCI
  ret = sam_sdcard_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: sam_sdcard_initialize() failed: %d\n", ret);
    }
#endif

/* LED support **************************************************************
 */

#ifdef HAVE_LED_DRIVER
  /* Register the LED driver */

  ret = userled_lower_initialize(LED_DRIVER_PATH);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n",
             ret);
    }
#endif

/* Button support ***********************************************************
 */

#ifdef CONFIG_INPUT_BUTTONS
#ifdef CONFIG_INPUT_BUTTONS_LOWER
  /* Register the BUTTON driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#else
  /* Enable BUTTON support for some other purpose */

  board_button_initialize();
#endif /* CONFIG_INPUT_BUTTONS_LOWER */
#endif /* CONFIG_INPUT_BUTTONS */

/* USB CDC/ACM **************************************************************
 */

#if !defined(CONFIG_BOARDCTL_USBDEVCTRL) && !defined(CONFIG_USBDEV_COMPOSITE)
# ifdef CONFIG_CDCACM
    cdcacm_initialize(0, NULL);
# endif
#endif

  /* If we got here then perhaps not all initialization was successful, but
   * at least enough succeeded to bring-up NSH with perhaps reduced
   * capabilities.
   */

  UNUSED(ret);
  return OK;
}
