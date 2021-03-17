/****************************************************************************
 * boards/arm/lpc31xx/olimex-lpc-h3131/src/lpc31_boot.c
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

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "arm_arch.h"
#include "arm_internal.h"
#include "lpc31.h"
#include "lpc_h3131.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc31_boardinitialize
 *
 * Description:
 *   All LPC31XX architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void lpc31_boardinitialize(void)
{
  /* Initialize configured, external memory resources */

#ifdef CONFIG_LPC31_EXTDRAM
  lpc31_meminitialize();
#endif

  /* Configure SPI chip selects if 1) SPI is not disabled, and 2) the weak
   * function lpc31_spidev_initialize() has been brought into the link.
   */

#if defined(CONFIG_LPC31_SPI)
  if (lpc31_spidev_initialize)
    {
      lpc31_spidev_initialize();
    }
#endif

  /* Initialize USB is 1) USBDEV is selected, 2) the USB controller is not
   * disabled, and 3) the weak function lpc31_usbdev_initialize() has
   * been brought into the build.
   */

#ifdef HAVE_USBDEV
  if (lpc31_usbdev_initialize)
    {
      lpc31_usbdev_initialize();
    }
#endif

  /* Initialize USB if the 1) the HS host or device controller is in the
   * configuration and 2) the weak function lpc31_usbhost_bootinitialize()
   * has been brought into the build. Presumably either CONFIG_USBDEV or
   * CONFIG_USBHOST is also selected.
   */

#ifdef HAVE_USBHOST
  lpc31_usbhost_bootinitialize();
#endif

  /* Configure on-board LEDs in all cases */

  board_autoled_initialize();
}
