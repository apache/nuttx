/****************************************************************************
 * boards/arm/kl/teensy-lc/src/kl_boardinitialize.c
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
#include "chip.h"
#include "kl_gpio.h"
#include "hardware/kl_pinmux.h"
#include "arm_internal.h"
#include "teensy-lc.h"

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
 * Name: kl_boardinitialize
 *
 * Description:
 *   All K25Z architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void kl_boardinitialize(void)
{
#if defined(CONFIG_KL_UART0)
  /* Remap UART0 to the standard pins. */

  kl_configgpio(PIN_PORTA | PIN2);
  kl_configgpio(PIN_PORTA | PIN1);
  kl_configgpio(PIN_UART0_RX_3);
  kl_configgpio(PIN_UART0_TX_3);
#endif

  /* Configure SPI chip selects if 1) SPI is not disabled, and 2) the weak
   * function kl_spidev_initialize() has been brought into the link.
   */

#if defined(CONFIG_KL_SPI0) || defined(CONFIG_KL_SPI1)
  if (kl_spidev_initialize)
    {
      kl_spidev_initialize();
    }
#endif

  /* Initialize USB if the 1) USB device controller is in the configuration
   * and 2) disabled, and 3) the weak function kl_usbinitialize() has been
   * brought into the build. Presumably either CONFIG_USBHOST or
   * CONFIG_USBDEV is also selected.
   */

#ifdef CONFIG_KL_USBOTG
  if (kl_usbinitialize)
    {
      kl_usbinitialize();
    }
#endif

  /* Configure on-board LED if LED support has been selected. */

#ifdef CONFIG_ARCH_LEDS
  kl_led_initialize();
#endif
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize().  board_late_initialize() will
 *   be called immediately after up_intitialize() is called and just before
 *   the initial application is started.
 *   This additional initialization phase may be used, for example, to
 *   initialize board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  /* Perform NSH initialization here instead of from the NSH.
   * This alternative NSH initialization is necessary when NSH is ran in
   * user-space but the initialization function must run in kernel space.
   */

#if defined(CONFIG_NSH_LIBRARY) && !defined(CONFIG_BOARDCTL)
  board_app_initialize(0);
#endif
}
#endif
