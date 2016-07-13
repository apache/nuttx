/****************************************************************************
 * configs/freedom-k64f/src/k64_usbmsc.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Configure and register the Kinetis MMC/SD block driver.
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
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>

#include "kinetis.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SYSTEM_USBMSC_DEVMINOR1
#  define CONFIG_SYSTEM_USBMSC_DEVMINOR1 0
#endif

/* SLOT number(s) could depend on the board configuration */

#ifdef CONFIG_ARCH_BOARD_FREEDOM_K64F
#  undef K64_MMCSDSLOTNO
#  define K64_MMCSDSLOTNO 0
#else
   /* Add configuration for new Kinetis boards here */
#  error "Unrecognized Kinetis board"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_usbmsc_initialize
 *
 * Description:
 *   Perform architecture specific initialization as needed to establish
 *   the mass storage device that will be exported by the USB MSC device.
 *
 ****************************************************************************/

int board_usbmsc_initialize(int port)
{
  /* If system/usbmsc is built as an NSH command, then SD slot should
   * already have been initialized in board_app_initialize() (see k64_appinit.c).
   * In this case, there is nothing further to be done here.
   */

#ifndef CONFIG_NSH_BUILTIN_APPS
#  warning "Missing logic"
#endif /* CONFIG_NSH_BUILTIN_APPS */

   return OK;
}
