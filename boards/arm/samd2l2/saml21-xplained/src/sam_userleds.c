/****************************************************************************
 * boards/arm/samd2l2/saml21-xplained/src/sam_userleds.c
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

/* There are three LEDs on board the SAML21 Xplained Pro board:  The EDBG
 * controls two of the LEDs, a power LED and a status LED.  There is only
 * one user controllable LED, a yellow LED labeled STATUS near the SAML21 USB
 * connector.
 *
 * This LED is controlled by PB10 and the LED can be activated by driving
 * PB10 to GND.
 *
 * When CONFIG_ARCH_LEDS is defined in the NuttX configuration, NuttX will
 * control the LED.
 * Otherwise, the LED can be controlled from user applications using the
 * logic in this file.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <arch/board/board.h>

#include "sam_port.h"
#include "saml21-xplained.h"

#ifndef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_userled_initialize
 *
 * Description:
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board
 *   LEDs.  If CONFIG_ARCH_LEDS is not defined, then the
 *   board_userled_initialize() is available to initialize the LED from user
 *   application logic.
 *
 ****************************************************************************/

uint32_t board_userled_initialize(void)
{
  sam_configport(PORT_STATUS_LED);
  return BOARD_NLEDS;
}

/****************************************************************************
 * Name: board_userled
 *
 * Description:
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board
 *  LEDs.  If CONFIG_ARCH_LEDS is not defined, then the board_userled() is
 *  available to control the LED from user application logic.
 *
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  if (led == BOARD_STATUS_LED)
    {
      sam_portwrite(PORT_STATUS_LED, !ledon);
    }
}

/****************************************************************************
 * Name: board_userled_all
 *
 * Description:
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board
 *   LEDs.
 *   If CONFIG_ARCH_LEDS is not defined, then the board_userled_all() is
 *   available to control the LED from user application logic.  NOTE:  since
 *   there is only a single LED on-board, this is function isn't very useful.
 *
 ****************************************************************************/

void board_userled_all(uint32_t ledset)
{
  board_userled(BOARD_STATUS_LED, (ledset & BOARD_STATUS_LED_BIT) != 0);
}

#endif /* !CONFIG_ARCH_LEDS */
