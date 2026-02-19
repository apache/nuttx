/****************************************************************************
 * boards/arm/sam34/sam4l-xplained/src/sam_boot.c
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

#include <debug.h>
#include <stdio.h>
#include <syslog.h>

#include <nuttx/board.h>

#include "sam4l-xplained.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SAM4L_XPLAINED_IOMODULE
/* Support for the SD card slot on the I/O1 module */

/* Verify NSH PORT and SLOT settings */

#  define SAM34_MMCSDSLOTNO    0 /* There is only one slot */

#  if defined(CONFIG_NSH_MMCSDSLOTNO) && CONFIG_NSH_MMCSDSLOTNO != SAM34_MMCSDSLOTNO
#    error Only one MMC/SD slot:  Slot 0 (CONFIG_NSH_MMCSDSLOTNO)
#  endif

#  if defined(CONFIG_NSH_MMCSDSPIPORTNO) && CONFIG_NSH_MMCSDSPIPORTNO != SD_CSNO
#    error CONFIG_NSH_MMCSDSPIPORTNO must have the same value as SD_CSNO
#  endif

/* Default MMC/SD minor number */

#  ifndef CONFIG_NSH_MMCSDMINOR
#    define CONFIG_NSH_MMCSDMINOR 0
#  endif
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_boardinitialize
 *
 * Description:
 *   All SAM3U architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void sam_boardinitialize(void)
{
  /* Configure SPI chip selects if 1) SPI is not disabled, and 2) the weak
   * function sam_spidev_initialize() has been brought into the link.
   */

#ifdef CONFIG_SAM34_SPI0
  if (sam_spidev_initialize)
    {
      sam_spidev_initialize();
    }
#endif

  /* Configure on-board LEDs if LED support has been selected. */

#ifdef CONFIG_ARCH_LEDS
  board_autoled_initialize();
#endif
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
#if defined(CONFIG_SAM34_LCDCA) && defined(CONFIG_SAM4L_XPLAINED_SLCD1MODULE)
  /* Initialize the SLCD and register the SLCD device as /dev/slcd0 */

  int ret = sam_slcd_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize the LCD: %d\n",
             ret);
      return;
    }
#endif
#if defined(CONFIG_SAM34_SPI0) && defined(CONFIG_SAM4L_XPLAINED_IOMODULE)

  /* Initialize the SPI-based MMC/SD slot */

  int ret = sam_sdinitialize(CONFIG_NSH_MMCSDMINOR);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize MMC/SD slot: %d\n",
             ret);
      return;
    }
#endif
}
#endif
