/****************************************************************************
 * boards/mips/pic32mz/ev49n51a/src/ev49n51a.h
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

#ifndef __BOARDS_MIPS_PIC32MZ_EV49N51A_SRC_EV49N51A_H
#define __BOARDS_MIPS_PIC32MZ_EV49N51A_SRC_EV49N51A_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "pic32mz_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LEDs: D201 red on RK1, D202 green on RK3, both active high (see
 * include/board.h).
 */

#define GPIO_LED_RED    (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORTK | GPIO_PIN1)
#define GPIO_LED_GREEN  (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORTK | GPIO_PIN3)

/* SST26VF032B serial flash (U202) on SPI1: chip select on RA1, active low,
 * initially deselected.
 */

#define GPIO_SST26_CS   (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORTA | GPIO_PIN1)

/* LAN8720A Ethernet PHY (U301) reset on RA14, active low.  R304 pulls it
 * high, so the PHY also runs when the pin is left as an input.
 */

#define GPIO_PHY_NRST   (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORTA | GPIO_PIN14)

/* LAN8720A interrupt output (nINT, open drain, active low) on RK6 through
 * R309 (not fitted on the EV49N51A).  The PHY holds it low until the
 * interrupt source register is read, so a falling edge marks a new event.
 */

#define GPIO_PHY_NINT   (GPIO_INPUT | GPIO_INTERRUPT | GPIO_PULLUP | \
                         GPIO_EDGE_DETECT | GPIO_EDGE_FALLING | \
                         GPIO_PORTK | GPIO_PIN6)

/* SST26 MTD partition is exported as /dev/mtdblock<SST26_MTD_MINOR> */

#define SST26_MTD_MINOR 0

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mz_bringup
 *
 * Description:
 *   Bring up board features.
 *
 ****************************************************************************/

int pic32mz_bringup(void);

/****************************************************************************
 * Name: pic32mz_led_initialize
 *
 * Description:
 *   Configure on-board LED GPIO for output (CONFIG_ARCH_LEDS only).
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void pic32mz_led_initialize(void);
#endif

/****************************************************************************
 * Name: pic32mz_spidev_initialize
 *
 * Description:
 *   Configure the SPI chip select GPIOs.
 *
 ****************************************************************************/

#ifdef CONFIG_PIC32MZ_SPI1
void pic32mz_spidev_initialize(void);
#endif

#endif /* __BOARDS_MIPS_PIC32MZ_EV49N51A_SRC_EV49N51A_H */
