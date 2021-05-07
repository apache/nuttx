/****************************************************************************
 * boards/arm/kinetis/freedom-k64f/src/k64_usbmsc.c
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
   * already have been initialized in board_app_initialize()
   * (see k64_appinit.c).
   * In this case, there is nothing further to be done here.
   */

#ifndef CONFIG_NSH_BUILTIN_APPS
#  warning "Missing logic"
#endif /* CONFIG_NSH_BUILTIN_APPS */

  return OK;
}
