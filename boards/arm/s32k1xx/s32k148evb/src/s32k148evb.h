/****************************************************************************
 * boards/arm/s32k1xx/s32k148evb/src/s32k148evb.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
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

#ifndef __BOARDS_ARM_S32K1XX_S32K148EVB_SRC_S32K148EVB_H
#define __BOARDS_ARM_S32K1XX_S32K148EVB_SRC_S32K148EVB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

#include "hardware/s32k1xx_pinmux.h"
#include "s32k1xx_periphclocks.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* S32K148EVB GPIOs *********************************************************/

/* LEDs.  The S32K148EVB has one RGB LED:
 *
 *   RedLED   PTE21
 *   GreenLED PTE22
 *   BlueLED  PTE23
 *
 * An output of '1' illuminates the LED.
 */

#define GPIO_LED_R     (PIN_PTE21 | GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO)
#define GPIO_LED_G     (PIN_PTE22 | GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO)
#define GPIO_LED_B     (PIN_PTE23 | GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO)

/* Buttons.  The S32K148EVB supports two buttons:
 *
 *   SW3  PTC12
 *   SW4  PTC13
 */

#define GPIO_SW3       (PIN_PTC12 | PIN_INT_BOTH)
#define GPIO_SW4       (PIN_PTC13 | PIN_INT_BOTH)

/* SPI chip selects */

/* Count of peripheral clock user configurations */

#define NUM_OF_PERIPHERAL_CLOCKS_0 18

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* User peripheral configuration structure 0 */

extern const struct peripheral_clock_config_s g_peripheral_clockconfig0[];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int s32k1xx_bringup(void);

/****************************************************************************
 * Name: s32k1xx_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the S32K148EVB
 *   board.
 *
 ****************************************************************************/

#ifdef CONFIG_S32K1XX_SPI
void s32k1xx_spidev_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_S32K1XX_S32K148EVB_SRC_S32K148EVB_H */
