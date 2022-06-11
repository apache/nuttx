/****************************************************************************
 * boards/arm/stm32/olimexino-stm32/src/stm32_boot.c
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
#include <arch/board/board.h>

#include <nuttx/board.h>
#include "olimexino-stm32.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vbus_handler
 ****************************************************************************/

#if defined(CONFIG_USBDEV)
static int vbus_handler(int irq, void *context, void *arg)
{
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void stm32_boardinitialize(void)
{
#ifdef CONFIG_ARCH_LEDS
  /* Configure on-board LEDs if LED support has been selected. */

  stm32_led_initialize();
#endif

#ifdef CONFIG_ARCH_BUTTONS
  /* Configure on-board buttons. */

  board_button_initialize();
#endif

#if defined(CONFIG_STM32_SPI1) || defined(CONFIG_STM32_SPI2) || \
    defined(CONFIG_STM32_SPI3)
  /* Configure SPI chip selects if 1) SP2 is not disabled, and 2) the weak
   * function stm32_spidev_initialize() has been brought into the link.
   */

  if (stm32_spidev_initialize)
    {
      stm32_spidev_initialize();
    }
#endif

#if defined(CONFIG_USBDEV) && defined(CONFIG_STM32_USB)
  /* Initialize USB is 1) USBDEV is selected, 2) the USB controller is not
   * disabled, and 3) the weak function stm32_usbinitialize() has been
   * brought into the build.
   */

  stm32_usbinitialize();
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
#if defined(CONFIG_NSH_LIBRARY) && !defined(CONFIG_BOARDCTL)
  /* Perform NSH initialization here instead of from the NSH.  This
   * alternative NSH initialization is necessary when NSH is ran in user-
   * space but the initialization function must run in kernel space.
   */

  board_app_initialize(0);
#endif

#if defined(CONFIG_USBDEV)
  stm32_usb_set_pwr_callback(vbus_handler);
#endif
}
#endif
