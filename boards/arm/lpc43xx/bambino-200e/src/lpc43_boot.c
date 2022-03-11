/****************************************************************************
 * boards/arm/lpc43xx/bambino-200e/src/lpc43_boot.c
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
#include "bambino-200e.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum devid_e
{
  DEVID0 = 0,
  DEVID1
};

enum ssp_channel_e
{
  SSP0 = 0,
  SSP1
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_boardinitialize
 *
 * Description:
 *   All LPC43xx architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void lpc43_boardinitialize(void)
{
  /* Configure on-board LEDs if LED support has been selected. */

#ifdef CONFIG_ARCH_LEDS
  board_autoled_initialize();
#endif

  /* Configure SSP chip selects if 1) Any SSP channel is not disabled, and 2)
   * the weak function lpc43_sspdev_initialize() has been brought into the
   * link.
   */

#if defined(CONFIG_LPC43_SSP0) || defined(CONFIG_LPC43_SSP1)
  if (lpc43_sspdev_initialize)
    {
      lpc43_sspdev_initialize();
    }
#endif
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize().
 *   board_late_initialize() will be called immediately after up_initialize()
 *   is called and just before the initial application is started.
 *   This additional initialization phase may be used, for example, to
 *   initialize board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  /* Configure max31855 driver for SSP0 or SSP1 */

#if defined(CONFIG_SENSORS_MAX31855)
  int ret;

#if defined(CONFIG_LPC43_SSP0)
  ret = lpc43_max31855initialize("/dev/temp0", SSP0, DEVID0);
  if (ret < 0)
    {
      serr("ERROR:  lpc43_max31855initialize failed: %d\n", ret);
    }

  ret = lpc43_max31855initialize("/dev/temp1", SSP0, DEVID1);
  if (ret < 0)
    {
      serr("ERROR:  lpc43_max31855initialize failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_LPC43_SSP1)
  ret = lpc43_max31855initialize("/dev/temp2", SSP1, DEVID0);
  if (ret < 0)
    {
      serr("ERROR:  lpc43_max31855initialize failed: %d\n", ret);
    }

  ret = lpc43_max31855initialize("/dev/temp3", SSP1, DEVID1);
  if (ret < 0)
    {
      serr("ERROR:  lpc43_max31855initialize failed: %d\n", ret);
    }
#endif
#endif
}
#endif
