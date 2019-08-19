/****************************************************************************
 * boards/arm/lpc17xx_40xx/lpc4088-quickstart/src/lpc4088-quickstart.h
 * arch/arm/src/board/lpc4088-quickstart.h
 *
 *   Copyright (C) 2013, 2017-2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
 * Public data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions
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
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_LIB_BOARDCTL=y :
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
