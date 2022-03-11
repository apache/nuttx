/****************************************************************************
 * boards/arm/lpc31xx/ea3152/src/lpc31_boot.c
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

#include "arm_internal.h"
#include "lpc31.h"
#include "ea3152.h"

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

  /* Initialize USB is 1) USBDEV is selected, 2) the USB controller is
   * not disabled, and 3) the weak function lpc31_usbinitialize() has
   * been brought into the build.
   */

#if defined(CONFIG_USBDEV) && defined(CONFIG_LPC31_USBOTG)
  if (lpc31_usbinitialize)
    {
      lpc31_usbinitialize();
    }
#endif

  /* Configure on-board LEDs if LED support has been selected. */

#ifdef CONFIG_ARCH_LEDS
  board_autoled_initialize();
#endif

  /* Set up mass storage device to support on demand paging */

#if defined(CONFIG_PAGING)
  if (lpc31_pginitialize)
    {
      lpc31_pginitialize();
    }
#endif
}
