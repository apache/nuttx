/****************************************************************************
 * boards/arm/stm32l4/steval-stlcs01v1/src/stm32_boot.c
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

#include <nuttx/arch.h>
#include <nuttx/board.h>

#include <arch/board/board.h>

#include "stm32l4_pwr.h"

#include "steval-stlcs01v1.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4_board_initialize
 *
 * Description:
 *   All STM32L4 architectures must provide the following entry point.  This
 *   entry point is called early in the initialization -- after all memory
 *   has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void stm32l4_board_initialize(void)
{
  /* Initialize USB if the 1) OTG FS controller is in the configuration and
   * 2) disabled, and 3) the weak function stm32l4_usbinitialize() has been
   * brought into the build. Presumeably either CONFIG_USBDEV is also
   * selected.
   */

#ifdef CONFIG_STM32L4_OTGFS
  /* Enable Vddusb - mandatory to use the USB OTG FS peripheral */

  stm32l4_pwr_enableusv(true);

  if (stm32l4_usbinitialize)
    {
      stm32l4_usbinitialize();
    }
#endif

  /* Enable Vddio2 - mandatory to use the PG2 - PG15 I/Os. */

  stm32l4_pwr_vddio2_valid(true);

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
 *   function called board_late_initialize().  board_late_initialize() will
 *   be called immediately after up_initialize() is called and just before
 *   the initial application is started.  This additional initialization
 *   phase phase may be used, for example, to initialize board-specific
 *   device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  /* Perform board-specific initialization */

  stm32l4_bringup();
}
#endif
