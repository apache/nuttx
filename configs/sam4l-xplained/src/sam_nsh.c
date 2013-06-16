/****************************************************************************
 * config/shenzhou/src/sam_nsh.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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
#include <debug.h>

#include "sam4l-xplained.h"

/****************************************************************************
 * Pre-Processor Definitions
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

/* Debug ********************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) lowsyslog(__VA_ARGS__)
#  else
#    define message(...) printf(__VA_ARGS__)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message lowsyslog
#  else
#    define message printf
#  endif
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 ****************************************************************************/

int nsh_archinitialize(void)
{
#ifdef CONFIG_SAM4L_XPLAINED_IOMODULE
  int ret;
#endif

  /* Initialize the SPI-based MMC/SD slot */

#ifdef CONFIG_SAM4L_XPLAINED_IOMODULE
  ret = sam_sdinitialize(CONFIG_NSH_MMCSDMINOR);
  if (ret < 0)
    {
      message("nsh_archinitialize: Failed to initialize MMC/SD slot: %d\n",
              ret);
      return ret;
    }
#endif

  return OK;
}
