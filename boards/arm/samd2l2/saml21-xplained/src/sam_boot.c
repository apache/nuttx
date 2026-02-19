/****************************************************************************
 * boards/arm/samd2l2/saml21-xplained/src/sam_boot.c
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

#include "sam_config.h"
#include "saml21-xplained.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Some configuration checks */

#ifdef CONFIG_SAML21_XPLAINED_IOMODULE_EXT1
#  ifndef SAMD2L2_HAVE_SPI0
#    error I/O1 module on EXT1 requires SERCOM SPI0
#    undef CONFIG_SAML21_XPLAINED_IOMODULE
#  endif
#  define SPI_PORTNO 0
#endif

#ifdef CONFIG_SAML21_XPLAINED_IOMODULE_EXT2
#  ifndef SAMD2L2_HAVE_SPI1
#    error I/O1 module on EXT2 requires SERCOM SPI1
#    undef CONFIG_SAML21_XPLAINED_IOMODULE
#  endif
#  define SPI_PORTNO 1
#endif

#ifdef CONFIG_SAML21_XPLAINED_IOMODULE
/* Support for the SD card slot on the I/O1 module */

/* Verify NSH PORT and SLOT settings */

#  define SAMD2L2_MMCSDSLOTNO  0 /* There is only one slot */

#  if defined(CONFIG_NSH_MMCSDSLOTNO) && CONFIG_NSH_MMCSDSLOTNO != SAMD2L2_MMCSDSLOTNO
#    error Only one MMC/SD slot:  Slot 0 (CONFIG_NSH_MMCSDSLOTNO)
#  endif

#  if defined(CONFIG_NSH_MMCSDSPIPORTNO) && CONFIG_NSH_MMCSDSPIPORTNO != SPI_PORTNO
#    error CONFIG_NSH_MMCSDSPIPORTNO must have the same value as SPI_PORTNO
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
  /* Configure SPI chip selects if
   * 1) SPI is not disabled, and
   * 2) the weak function
   * sam_spidev_initialize() has been brought into the link.
   */

#ifdef SAMD2L2_HAVE_SPI
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
 *   called immediately after up_intitialize() is called and just before the
 *   initial application is started.  This additional initialization phase
 *   may be used, for example, to initialize board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
#if defined(SAMD2L2_HAVE_SPI0) && defined(CONFIG_SAML21_XPLAINED_IOMODULE)
  /* Initialize the SPI-based MMC/SD slot */

  int ret = sam_sdinitialize(SPI_PORTNO, CONFIG_NSH_MMCSDMINOR);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize MMC/SD slot: %d\n",
             ret);
    }
#endif
}
#endif /* CONFIG_BOARD_LATE_INITIALIZE */
