/****************************************************************************
 * boards/arm/rzv2h/rzv2h-rdk/src/rzv2h_boot.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <stdint.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include <arch/rzv2h/rzv2h_gpio.h>
#include "rzv2h-rdk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPIO Port Security Control registers base - RZV2H specific */
#define RZV2H_GPIO_PMSAR_BASE       0x14030800  /* Port Mode Security Attribution */
#define RZV2H_GPIO_PSCU_BASE        0x14030C00  /* Port Security Control Unit */

/* Number of GPIO ports in RZV2H (ports 0x20-0x2B = 12 ports) */
#define RZV2H_GPIO_NPORTS           12

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rzv2h_gpio_security_init
 *
 * Description:
 *   Initialize GPIO port security attributes to secure mode (default).
 *   Must be called before configuring any GPIO pins.
 *   Based on FSP r_ioport security initialization.
 *
 ****************************************************************************/

static void rzv2h_gpio_security_init(void)
{
#ifdef CONFIG_ARM_TRUSTZONE
  volatile uint32_t *pmsar;
  volatile uint32_t *pscu;
  int port;

  /* Initialize PMSAR (Port Mode Security Attribution Register)
   * Set all pins to secure mode (0 = secure, 1 = non-secure)
   */

  pmsar = (volatile uint32_t *)RZV2H_GPIO_PMSAR_BASE;
  for (port = 0; port < RZV2H_GPIO_NPORTS; port++)
    {
      pmsar[port] = 0x00000000;  /* All pins secure */
    }

  /* Initialize PSCU (Port Security Control Unit)
   * Configure security control for each port
   */

  pscu = (volatile uint32_t *)RZV2H_GPIO_PSCU_BASE;
  for (port = 0; port < RZV2H_GPIO_NPORTS; port++)
    {
      pscu[port] = 0x00000000;  /* Default security settings */
    }

  ARM_DSB();
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rzv2h_board_initialize
 *
 * Description:
 *   All RZV architectures must provide the following entry point.  This
 *   entry point is called early in the initialization -- after clocks and
 *   memory have been configured but before any devices have been
 *   initialized.
 *
 *   This function is called from rzv_start.c during the boot sequence.
 *
 ****************************************************************************/

void rzv2h_board_initialize(void)
{
  /* Initialize GPIO security attributes (TrustZone) */

  rzv2h_gpio_security_init();

  /* Configure SCI/UART pins for serial communication */

#ifdef CONFIG_RZV2H_UART_SCI
  rzv2h_serial_setup();
#endif

  /* Configure on-board LEDs if LED support has been selected */

#ifdef CONFIG_ARCH_LEDS
  board_autoled_initialize();
#endif

  /* Initialize any board-specific GPIO configurations */

#ifdef CONFIG_BOARDCTL_IOCTL
  /* Additional board-specific initialization can be done via ioctl */
#endif
}

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  /* Perform board-specific initialization */

  rzv2h_bringup();
}

#endif
