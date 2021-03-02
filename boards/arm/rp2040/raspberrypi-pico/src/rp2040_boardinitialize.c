/****************************************************************************
 * boards/arm/rp2040/raspberrypi-pico/src/rp2040-boardinitialize.c
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

#include "rp2040_gpio.h"
#include "hardware/rp2040_pads_bank0.h"
#include "hardware/rp2040_io_bank0.h"
#include "hardware/rp2040_sio.h"

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
 * Name: rp2040_boardearlyinitialize
 *
 * Description:
 *
 ****************************************************************************/

void rp2040_boardearlyinitialize(void)
{
  /* Disable IE on GPIO 26-29 */

  clrbits_reg32(RP2040_PADS_BANK0_GPIO_IE, RP2040_PADS_BANK0_GPIO(26));
  clrbits_reg32(RP2040_PADS_BANK0_GPIO_IE, RP2040_PADS_BANK0_GPIO(27));
  clrbits_reg32(RP2040_PADS_BANK0_GPIO_IE, RP2040_PADS_BANK0_GPIO(28));
  clrbits_reg32(RP2040_PADS_BANK0_GPIO_IE, RP2040_PADS_BANK0_GPIO(29));

  /* Set board LED pin */

  rp2040_gpio_set_function(BOARD_GPIO_LED_PIN,
                           RP2040_IO_BANK0_GPIO_CTRL_FUNCSEL_SIO);
  putreg32(1 << BOARD_GPIO_LED_PIN, RP2040_SIO_GPIO_OE_SET);

  putreg32(1 << BOARD_GPIO_LED_PIN, RP2040_SIO_GPIO_OUT_SET);

  /* Set default UART pin */

#if defined(CONFIG_RP2040_UART0) && CONFIG_RP2040_UART0_GPIO >= 0
  rp2040_gpio_set_function(CONFIG_RP2040_UART0_GPIO,
                           RP2040_IO_BANK0_GPIO_CTRL_FUNCSEL_UART); /* TX */
  rp2040_gpio_set_function(CONFIG_RP2040_UART0_GPIO + 1,
                           RP2040_IO_BANK0_GPIO_CTRL_FUNCSEL_UART); /* RX */
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  rp2040_gpio_set_function(CONFIG_RP2040_UART0_GPIO + 2,
                           RP2040_IO_BANK0_GPIO_CTRL_FUNCSEL_UART); /* CTS */
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  rp2040_gpio_set_function(CONFIG_RP2040_UART0_GPIO + 3,
                           RP2040_IO_BANK0_GPIO_CTRL_FUNCSEL_UART); /* RTS */
#endif
#endif

#if defined(CONFIG_RP2040_UART1) && CONFIG_RP2040_UART1_GPIO >= 0
  rp2040_gpio_set_function(CONFIG_RP2040_UART1_GPIO,
                           RP2040_IO_BANK0_GPIO_CTRL_FUNCSEL_UART); /* TX */
  rp2040_gpio_set_function(CONFIG_RP2040_UART1_GPIO + 1,
                           RP2040_IO_BANK0_GPIO_CTRL_FUNCSEL_UART); /* RX */
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  rp2040_gpio_set_function(CONFIG_RP2040_UART1_GPIO + 2,
                           RP2040_IO_BANK0_GPIO_CTRL_FUNCSEL_UART); /* CTS */
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  rp2040_gpio_set_function(CONFIG_RP2040_UART1_GPIO + 3,
                           RP2040_IO_BANK0_GPIO_CTRL_FUNCSEL_UART); /* RTS */
#endif
#endif

  /* Set default I2C pin */

#if defined(CONFIG_RP2040_I2C0) && CONFIG_RP2040_I2C0_GPIO >= 0
  rp2040_gpio_set_function(CONFIG_RP2040_I2C0_GPIO,
                           RP2040_IO_BANK0_GPIO_CTRL_FUNCSEL_I2C);  /* SDA */
  rp2040_gpio_set_function(CONFIG_RP2040_I2C0_GPIO + 1,
                           RP2040_IO_BANK0_GPIO_CTRL_FUNCSEL_I2C);  /* SCL */

  rp2040_gpio_set_pulls(CONFIG_RP2040_I2C0_GPIO, false, true); /* Pull down */
  rp2040_gpio_set_pulls(CONFIG_RP2040_I2C0_GPIO + 1, false, true);
#endif

#if defined(CONFIG_RP2040_I2C1) &&  CONFIG_RP2040_I2C1_GPIO >= 0
  rp2040_gpio_set_function(CONFIG_RP2040_I2C1_GPIO,
                           RP2040_IO_BANK0_GPIO_CTRL_FUNCSEL_I2C);  /* SDA */
  rp2040_gpio_set_function(CONFIG_RP2040_I2C1_GPIO + 1,
                           RP2040_IO_BANK0_GPIO_CTRL_FUNCSEL_I2C);  /* SCL */

  rp2040_gpio_set_pulls(CONFIG_RP2040_I2C1_GPIO, false, true); /* Pull down */
  rp2040_gpio_set_pulls(CONFIG_RP2040_I2C1_GPIO + 1, false, true);
#endif
}

/****************************************************************************
 * Name: rp2040_boardinitialize
 *
 * Description:
 *
 ****************************************************************************/

void rp2040_boardinitialize(void)
{
}
