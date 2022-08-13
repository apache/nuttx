/****************************************************************************
 * boards/arm/stm32/olimex-stm32-p407/src/stm32_bringup.c
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
#include <nuttx/mmcsd.h>
#include <nuttx/input/buttons.h>

#ifdef CONFIG_USBMONITOR
#  include <nuttx/usb/usbmonitor.h>
#endif

#ifdef CONFIG_MODULE
#  include "nuttx/symtab.h"
#  include "nuttx/lib/modlib.h"
#endif

#ifdef CONFIG_STM32_OTGFS
#  include "stm32_usbhost.h"
#endif

#include "stm32.h"
#include "olimex-stm32-p407.h"

#ifdef CONFIG_SENSORS_DHTXX
#include "stm32_dhtxx.h"
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef HAVE_MODSYMS
extern const struct symtab_s MODSYMS_SYMTAB_ARRAY[];
extern const int MODSYMS_NSYMBOLS_VAR;
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
#ifdef HAVE_MMCSD
  struct sdio_dev_s *sdio;
#endif
  int ret;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#ifdef HAVE_MODSYMS
  /* Install the module symbol table */

  modlib_setsymtab(MODSYMS_SYMTAB_ARRAY, MODSYMS_NSYMBOLS_VAR);
#endif

#ifdef HAVE_MMCSD
  /* Mount the SDIO-based MMC/SD block driver */

  /* First, get an instance of the SDIO interface */

  sdio = sdio_initialize(MMCSD_SLOTNO);
  if (!sdio)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize SDIO slot %d\n",
             MMCSD_SLOTNO);
      return -ENODEV;
    }

  /* Now bind the SDIO interface to the MMC/SD driver */

  ret = mmcsd_slotinitialize(MMCSD_MINOR, sdio);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to bind SDIO to the MMC/SD driver: %d\n",
             ret);
      return ret;
    }

  /* Then let's guess and say that there is a card in the slot.  The Olimex
   * STM32 P407 does not support a GPIO to detect if there is a card in
   * the slot so we are reduced to guessing.
   */

  sdio_mediachange(sdio, true);
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

#ifdef HAVE_USBHOST
  /* Initialize USB host operation.  stm32_usbhost_setup() starts a thread
   * will monitor for USB connection and disconnection events.
   */

  ret = stm32_usbhost_setup();
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize USB host: %d\n", ret);
      return ret;
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

#ifdef CONFIG_INPUT_BUTTONS
  /* Register the BUTTON driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_DHTXX
  ret = board_dhtxx_initialize(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_dhtxx_initialize() failed: %d\n", ret);
    }
#endif

#ifdef HAVE_CS4344
  /* Configure CS4344 audio */

  ret = stm32_cs4344_initialize(1);
  if (ret != OK)
    {
      syslog(LOG_ERR, "Failed to initialize CS4344 audio: %d\n", ret);
    }
#endif

#ifdef CONFIG_INPUT_DJOYSTICK
  ret = stm32_djoy_initialize();
  if (ret != OK)
    {
      syslog(LOG_ERR, "Failed to register djoystick driver: %d\n", ret);
    }
#endif

  UNUSED(ret);
  return OK;
}
