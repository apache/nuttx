/****************************************************************************
 * boards/arm/rp23xx/pimoroni-pico-plus-2-w/src/rp23xx_pico.h
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

#ifndef __BOARDS_ARM_RP23XX_PIMORONI_PICO_PLUS_2_W_SRC_RP23XX_PICO_H
#define __BOARDS_ARM_RP23XX_PIMORONI_PICO_PLUS_2_W_SRC_RP23XX_PICO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* CYW43439 wireless chip
 *
 * The gSPI bus is half duplex -- a single data line carries both directions
 * and doubles as the chip's interrupt request line.
 */

#define GPIO_CYW43439_ON      23  /* Drive high to power the chip up   */
#define GPIO_CYW43439_DATA    24  /* Bidirectional data, also the IRQ  */
#define GPIO_CYW43439_CS      25  /* Drive low to select the chip      */
#define GPIO_CYW43439_CLOCK   29  /* gSPI clock                       */

/* LEDs
 *
 * The board's only LED hangs off GPIO 0 of the CYW43439, see
 * include/rp23xx_extra_gpio.h.  There is no LED on an RP2350 pin.
 */

/* Buttons */

/* Buttons GPIO pins definition */

/* The board's BOOT button doubles as a user button once NuttX is running.
 * It is active low.  RESET is not readable as a GPIO.
 */

#define GPIO_BTN_USER1     45

/* Buttons IRQ definitions */

#define MIN_IRQBUTTON     BUTTON_USER1
#define MAX_IRQBUTTON     BUTTON_USER1
#define NUM_IRQBUTTONS    (MAX_IRQBUTTON - MIN_IRQBUTTON + 1)

int rp23xx_bringup(void);

#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_ARCH_BOARD_COMMON)
int rp23xx_dev_gpio_init(void);
#endif

#endif /* __BOARDS_ARM_RP23XX_PIMORONI_PICO_PLUS_2_W_SRC_RP23XX_PICO_H */
