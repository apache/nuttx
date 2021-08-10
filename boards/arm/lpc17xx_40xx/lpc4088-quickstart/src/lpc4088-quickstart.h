/****************************************************************************
 * boards/arm/lpc17xx_40xx/lpc4088-quickstart/src/lpc4088-quickstart.h
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

#ifndef __BOARDS_ARM_LPC17XX_40XX_LPC4088_QUICKSTART_SRC_LPC4088_QUICKSTART_H
#define __BOARDS_ARM_LPC17XX_40XX_LPC4088_QUICKSTART_SRC_LPC4088_QUICKSTART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LPC4088 QuickStart GPIO Pin Definitions **********************************/

/* If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
 * any way.  The following definitions are used to access individual LEDs.
 *
 * LED1 : Connected to P1[18]
 * LED2 : Connected to P0[13]
 * LED3 : Connected to P1[13]
 * LED4 : Connected to P2[19]
 *
 * These LEDs are driven through a PNP transistor so a low output value will
 * illuminate them.
 */

#define GPIO_LED1        (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT1 | GPIO_PIN18)
#define GPIO_LED2        (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT0 | GPIO_PIN13)

/* These LEDs are connected to ground so a high output value will illuminate
 * them.
 */

#define GPIO_LED3        (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT1 | GPIO_PIN13)
#define GPIO_LED4        (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT2 | GPIO_PIN19)

/* Button definitions *******************************************************/

/* The LPC4088 QuickStart supports a single button.
 * It must be pulled up by the MCU.
 * When closed, the pin will be pulled to ground.
 * So the button will read "1" when open and "0" when closed.
 * The button is capable of generating an interrupt.
 *
 * USER1           -- Connected to P2[10]
 * For the interrupting buttons, interrupts are generated on both edges
 * (press and release).
 */

#define GPIO_USER1       (GPIO_INPUT   | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN10)

/* IRQ numbers for the buttons that do support interrupts */

#define GPIO_USER2_IRQ   LPC17_40_IRQ_P2p10

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc4088_quickstart_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library via boardctl()
 *
 ****************************************************************************/

int lpc4088_quickstart_bringup(void);

/****************************************************************************
 * Name: lpc4088_quickstart_sdram_initialize
 *
 * Description:
 *   Initialize SDRAM
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_40_EMC
#ifdef CONFIG_LPC17_40_EXTDRAM
void lpc4088_quickstart_sdram_initialize(void);
#endif
#endif /* CONFIG_LPC17_40_EMC */

/****************************************************************************
 * Name: lpc4088_quickstart_flash_initialize
 *
 * Description:
 *   Initialize SPIFI FLASH
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_40_SPIFI
void lpc4088_quickstart_flash_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_LPC17XX_40XX_LPC4088_QUICKSTART_SRC_LPC4088_QUICKSTART_H */
