/****************************************************************************
 * config/samd20-xplained/src/sam_nsh.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#include <stdio.h>
#include <syslog.h>

#include <nuttx/board.h>

#include "sam_config.h"
#include "samd20-xplained.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Some configuration checks */

#ifdef CONFIG_SAMD20_XPLAINED_IOMODULE_EXT1
#  ifndef SAMDL_HAVE_SPI0
#    error I/O1 module on EXT1 requires SERCOM SPI0
#    undef CONFIG_SAMD20_XPLAINED_IOMODULE
#  endif
#  define SPI_PORTNO 0
#endif

#ifdef CONFIG_SAMD20_XPLAINED_IOMODULE_EXT2
#  ifndef SAMDL_HAVE_SPI1
#    error I/O1 module on EXT2 requires SERCOM SPI1
#    undef CONFIG_SAMD20_XPLAINED_IOMODULE
#  endif
#  define SPI_PORTNO 1
#endif

#ifdef CONFIG_SAMD20_XPLAINED_IOMODULE
/* Support for the SD card slot on the I/O1 module */
/* Verify NSH PORT and SLOT settings */

#  define SAMDL_MMCSDSLOTNO    0 /* There is only one slot */

#  if defined(CONFIG_NSH_MMCSDSLOTNO) && CONFIG_NSH_MMCSDSLOTNO != SAMDL_MMCSDSLOTNO
#    error Only one MMC/SD slot:  Slot 0 (CONFIG_NSH_MMCSDSLOTNO)
#    undef CONFIG_NSH_MMCSDSLOTNO
#    define CONFIG_NSH_MMCSDSLOTNO SAMDL_MMCSDSLOTNO
#  endif

#  if defined(CONFIG_NSH_MMCSDSPIPORTNO) && CONFIG_NSH_MMCSDSPIPORTNO != SPI_PORTNO
#    error CONFIG_NSH_MMCSDSPIPORTNO must have the same value as SPI_PORTNO
#    undef CONFIG_NSH_MMCSDSPIPORTNO
#    define CONFIG_NSH_MMCSDSPIPORTNO SPI_PORTNO
#  endif

/* Default MMC/SD minor number */

#  ifndef CONFIG_NSH_MMCSDMINOR
#    define CONFIG_NSH_MMCSDMINOR 0
#  endif
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 ****************************************************************************/

int board_app_initialize(void)
{
#if defined(SAMDL_HAVE_SPI0) && defined(CONFIG_SAMD20_XPLAINED_IOMODULE)
  /* Initialize the SPI-based MMC/SD slot */

  {
    int ret = sam_sdinitialize(SPI_PORTNO, CONFIG_NSH_MMCSDMINOR);
    if (ret < 0)
      {
        syslog(LOG_ERR, "ERROR: Failed to initialize MMC/SD slot: %d\n",
               ret);
       return ret;
      }
  }
#endif

  return OK;
}
