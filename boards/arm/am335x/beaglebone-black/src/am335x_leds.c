/****************************************************************************
 * boards/arm/am335x/beaglebone-black/src/am335x_leds.c
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

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "beaglebone-black.h"

/* The beaglebone black has four user LEDs; all four can be controlled from
 * software.  All are tied to ground and, hence, illuminated by driving the
 * output pins to a high value:
 *
 *   1. LED0 GPMC_A5   GPMC_A5/GMII2_TXD0/RGMII2_TD0/RMII2_TXD0/GPMC_A21/
 *                     PR1_MII1_RXD3/eQEP1B_IN/GPIO1_21
 *   2. LED1 GPMC_A6   GPMC_A6/GMII2_TXCLK/RGMII2_TCLK/MMC2_DAT4/GPMC_A22/
 *                     PR1_MII1_RXD2/eQEP1_INDEX/GPIO1_22
 *   3. LED2 GPMC_A7   GPMC_A7/GMII2_RXCLK/RGMII2_RCLK/MMC2_DAT5/GPMC_A23/
 *                     PR1_MII1_RXD1/eQEP1_STROBE/GPIO1_23
 *   4. LED3 GPMC_A8   GPMC_A8/GMII2_RXD3/RGMII2_RD3/MMC2_DAT6/GPMC_A24/
 *                     PR1_MII1_RXD0/MCASP0_ACLKX/GPIO1_24

 * These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/am335x_leds.c. The LEDs are used to encode OS-
 * related events as follows:
 *
 *   SYMBOL            Meaning                      LED state
 *                                              LED1 LED3 LED4
 *   ----------------- -----------------------  ---- ---- ---- ------------
 *   LED_STARTED       NuttX has been started   ON   OFF  OFF
 *   LED_HEAPALLOCATE  Heap has been allocated  OFF  ON   OFF
 *   LED_IRQSENABLED   Interrupts enabled       ON   ON   OFF
 *   LED_STACKCREATED  Idle stack created       ON   ON   OFF
 *   LED_INIRQ         In an interrupt          N/C  N/C  Soft glow
 *   LED_SIGNAL        In a signal handler      N/C  N/C  Soft glow
 *   LED_ASSERTION     An assertion failed      N/C  N/C  Soft glow
 *   LED_PANIC         The system has crashed   N/C  N/C  2Hz Flashing
 *   LED_IDLE          MCU is is sleep mode         Not used
 *
 * After booting, LED0 and 1 are not longer used by the system and can be
 * used for other purposes by the application (Of course, all LEDs are
 * available to the application if CONFIG_ARCH_LEDS is not defined.
 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am335x_led_initialize
 *
 * Description:
 *   Configure LEDs.  LEDs are left in the OFF state.
 *
 ****************************************************************************/

void am335x_led_initialize(void)
{
  am335x_gpio_config(GPIO_LED0);
  am335x_gpio_config(GPIO_LED1);
  am335x_gpio_config(GPIO_LED2);
  am335x_gpio_config(GPIO_LED3);
}

/****************************************************************************
 * Name: board_autoled_on
 *
 * Description:
 *   Select the "logical" ON state:
 *
 *   SYMBOL            Value Meaning                    LED state
 *                                                    LED1 LED3 LED4
 *   ----------------- ----- -----------------------  ---- ---- ------------
 *   LED_STARTED         0   NuttX has been started   ON   OFF  OFF
 *   LED_HEAPALLOCATE    1   Heap has been allocated  OFF  ON   OFF
 *   LED_IRQSENABLED     2   Interrupts enabled       ON   ON   OFF
 *   LED_STACKCREATED    2   Idle stack created       ON   ON   OFF
 *   LED_INIRQ           3   In an interrupt          N/C  N/C  Soft glow
 *   LED_SIGNAL          3   In a signal handler      N/C  N/C  Soft glow
 *   LED_ASSERTION       3   An assertion failed      N/C  N/C  Soft glow
 *   LED_PANIC           3   The system has crashed   N/C  N/C  2Hz Flashing
 *   LED_IDLE           ---  MCU is is sleep mode         Not used
 *
 *   LED1 is illuminated by driving the output pins to a high value
 *   LED3 and LED 4 are illuminated by taking the output to ground.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_autoled_on(int led)
{
  switch (led)
    {
    case 0:
      am335x_gpio_write(GPIO_LED0, true);
      am335x_gpio_write(GPIO_LED1, true);
      am335x_gpio_write(GPIO_LED3, true);
      break;

    case 1:
      am335x_gpio_write(GPIO_LED0, false);
      am335x_gpio_write(GPIO_LED1, false);
      am335x_gpio_write(GPIO_LED3, true);
      break;

    case 2:
      am335x_gpio_write(GPIO_LED0, false);
      am335x_gpio_write(GPIO_LED1, true);
      am335x_gpio_write(GPIO_LED3, true);
      break;

    case 3:
      am335x_gpio_write(GPIO_LED3, false);
      break;
    }
}
#endif

/****************************************************************************
 * Name: board_autoled_off
 *
 * Description:
 *   Select the "logical" OFF state:
 *
 *   SYMBOL            Value Meaning                    LED state
 *                                                    LED1 LED3 LED4
 *   ----------------- ----- -----------------------  ---- ---- ------------
 *   LED_STARTED         0   NuttX has been started   ON   OFF  OFF
 *   LED_HEAPALLOCATE    1   Heap has been allocated  OFF  ON   OFF
 *   LED_IRQSENABLED     2   Interrupts enabled       ON   ON   OFF
 *   LED_STACKCREATED    2   Idle stack created       ON   ON   OFF
 *   LED_INIRQ           3   In an interrupt          N/C  N/C  Soft glow
 *   LED_SIGNAL          3   In a signal handler      N/C  N/C  Soft glow
 *   LED_ASSERTION       3   An assertion failed      N/C  N/C  Soft glow
 *   LED_PANIC           3   The system has crashed   N/C  N/C  2Hz Flashing
 *   LED_IDLE           ---  MCU is is sleep mode         Not used
 *
 *   LED1 is illuminated by driving the output pins to a high value
 *   LED3 and LED 4 are illuminated by taking the output to ground.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_autoled_off(int led)
{
  switch (led)
    {
    case 0:
    case 1:
    case 2:
      break;

    case 3:
      am335x_gpio_write(GPIO_LED3, true);
      break;
    }
}
#endif

/****************************************************************************
 * Name:  board_userled_initialize, board_userled, and board_userled_all
 *
 * Description:
 *   These interfaces allow user control of the board LEDs.
 *
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control both on-board
 *   LEDs up until the completion of boot.
 *   Then it will continue to control LED1; LED0 is available for
 *   application use.
 *
 *   If CONFIG_ARCH_LEDS is not defined, then both LEDs are available for
 *   application use.
 *
 ****************************************************************************/

uint32_t board_userled_initialize(void)
{
  /* Initialization already performed in am335x_led_initialize */

  return BOARD_NLEDS;
}

void board_userled(int led, bool ledon)
{
  switch (led)
    {
    case USER_LED0:
      am335x_gpio_write(GPIO_LED0, ledon);
      break;

    case USER_LED1:
      am335x_gpio_write(GPIO_LED1, ledon);
      break;

#ifndef CONFIG_ARCH_LEDS
    case USER_LED3:
      am335x_gpio_write(GPIO_LED3, ledon);
      break;
#endif
    }
}

void board_userled_all(uint32_t ledset)
{
  board_userled(USER_LED0, (ledset & USER_LED0) != 0);
  board_userled(USER_LED1, (ledset & USER_LED1) != 0);
#ifndef CONFIG_ARCH_LEDS
  board_userled(USER_LED3, (ledset & USER_LED3) != 0);
#endif
}
