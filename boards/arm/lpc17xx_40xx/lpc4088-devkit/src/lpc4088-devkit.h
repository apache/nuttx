/****************************************************************************
 * boards/arm/lpc17xx_40xx/lpc4088-devkit/src/lpc4088-devkit.h
 * arch/arm/src/board/lpc4088-devkit.h
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

#ifndef __BOARDS_ARM_LPC17XX_40XX_LPC4088_DEVKIT_SRC_LPC4088_DEVKIT_H
#define __BOARDS_ARM_LPC17XX_40XX_LPC4088_DEVKIT_SRC_LPC4088_DEVKIT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LPC4088 Developer's Kit GPIO Pin Definitions *****************************/

/* GPIO P2[21] connects to the Ready/Busy pin of the NAND part.  We need to
 * reconfigure this pin as normal GPIO input if NAND is used.
 */

#define GPIO_NAND_RB     (GPIO_INPUT | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN21)

/* If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
 * any way.  The following definitions are used to access individual LEDs.
 *
 * LED1 -- Connected to P1[14]
 * LED2 -- Connected to P0[16]
 * LED3 -- Connected to P1[13]
 * LED4 -- Connected to P4[27]
 *
 * These LEDs are connected to ground so a high output value will illuminate
 * them.
 */

#define GPIO_LED1        (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT1 | GPIO_PIN14)
#define GPIO_LED2        (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN16)
#define GPIO_LED3        (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT1 | GPIO_PIN13)
#define GPIO_LED4        (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT4 | GPIO_PIN27)

/* Button definitions *******************************************************/

/* The LPC4088 Developer's Kit supports several buttons.
 * All are pulled up externally.
 * When closed, the pins will be pulled to ground.
 * So the buttons will read "1" when open and "0" when closed.
 * All are capable of generating interrupts.
 *
 * USER1           -- Connected to P2[10]
 *
 * Joystick:
 *
 * JOY_A           -- Connected to P2[23]
 * JOY_B           -- Connected to P2[25]
 * JOY_C           -- Connected to P2[26]
 * JOY_D           -- Connected to P2[27]
 * JOY_CTR         -- Connected to P2[22]
 *
 * For the interrupting buttons, interrupts are generated on both edges
 * (press and release).
 */

#define GPIO_USER1       (GPIO_INPUT   | GPIO_PULLUP | GPIO_PORT4 | GPIO_PIN26)

#define GPIO_JOY_A       (GPIO_INTBOTH | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN25)
#define GPIO_JOY_B       (GPIO_INTBOTH | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN26)
#define GPIO_JOY_C       (GPIO_INTBOTH | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN23)
#define GPIO_JOY_D       (GPIO_INTBOTH | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN19)
#define GPIO_JOY_CTR     (GPIO_INTBOTH | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN14)

/* IRQ numbers for the buttons that do support interrupts */

#define GPIO_USER1_IRQ   LPC17_40_IRQ_P2p10
#define GPIO_JOY_A_IRQ   LPC17_40_IRQ_P2p23
#define GPIO_JOY_B_IRQ   LPC17_40_IRQ_P2p25
#define GPIO_JOY_C_IRQ   LPC17_40_IRQ_P2p26
#define GPIO_JOY_D_IRQ   LPC17_40_IRQ_P2p27
#define GPIO_JOY_CTR_IRQ LPC17_40_IRQ_P2p22

/* SD Card ******************************************************************/

/* The SD card detect (CD) signal is on bit 4 of the PCA9532 port expander
 * U8. Support for this is not currently set up.
 * The SD card's power is controlled through a P-channel MOSFET connected
 * to P1[5]. This pin must be driven LOW in order to enable the SD card.
 */

#define GPIO_SD_PWR      (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT1 | GPIO_PIN5)

/* More work is required to complete implementation of LCD support
 * on this board.
 */

#if 0
/* LCD **********************************************************************/

/* Backlight enable, P2[1].  Initial state is OFF (zero) */

#define GPIO_LCD_BL      (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT2 | GPIO_PIN1)

/* XPT2046 Touchscreen ******************************************************/

/* -------------- -------------------- ------------ -------------------------
 * XTPT2046       Module               Module       LPC4088 Developer's Kit LED
 *                Signal               Connector    Connector
 * -------------- -------------------- ------------ -------------------------
 * Pin 11 PENIRQ\ PENIRQ (pulled high) PORT3 Pin 1  P2.15 PENIRQ
 * Pin 12 DOUT    MISO                 PORT3 Pin 4  P1.18 MISO1  (Also USB HOST UP LED)
 * Pin 13 BUSY    BUSY (pulled high)   PORT3 Pin 9  P2.14 BUSY
 * Pin 14 DIN     MOSI                 PORT3 Pin 3  P0.13 MOSI1  (Also USB Device up LED and SD CD pin)
 * Pin 15 CS\     SSEL (pulled high)   PORT3 Pin 6  P1.8  GPIO   (Also RMII_CRS_DV)
 * Pin 16 DCLK    SCK                  PORT3 Pin 5  P1.19 SCK1
 * -------------- -------------------- ------------ -------------------------
 *
 * Pins should not need to be configured as pull-ups because, according to
 * the LCD schematic, the are pulled-up on board the LCD module.
 */

#define GPIO_TC_PENIRQ   (GPIO_INTBOTH | GPIO_FLOAT | GPIO_PORT2 | GPIO_PIN15)
#define GPIO_TC_BUSY     (GPIO_INPUT | GPIO_FLOAT | GPIO_PORT2 | GPIO_PIN14)
#define GPIO_TC_CS       (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT1 | GPIO_PIN8)

#define LPC17_40_IRQ_PENIRQ LPC17_40_IRQ_P2p15

#endif

/****************************************************************************
 * Public data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc4088_devkit_bringup
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

int lpc4088_devkit_bringup(void);

/****************************************************************************
 * Name: lpc4088_devkit_sspdev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the LPC4088 Developer's
 *   Kit board.
 *
 ****************************************************************************/

void weak_function lpc4088_devkit_sspdev_initialize(void);

/****************************************************************************
 * Name: lpc4088_devkit_sdram_initialize
 *
 * Description:
 *   Initialize SDRAM
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_40_EMC
#ifdef CONFIG_LPC17_40_EXTDRAM
void lpc4088_devkit_sdram_initialize(void);
#endif

/****************************************************************************
 * Name: lpc4088_devkit_nor_initialize
 *
 * Description:
 *   Initialize NOR FLASH
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_40_EXTNOR
void lpc4088_devkit_nor_initialize(void);
#endif

/****************************************************************************
 * Name: lpc4088_devkit_nand_initialize
 *
 * Description:
 *   Initialize NAND FLASH
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_40_EXTNAND
void lpc4088_devkit_nand_initialize(void);
#endif
#endif /* CONFIG_LPC17_40_EMC */

/****************************************************************************
 * Name: lpc4088_devkit_lcd_initialize
 *
 * Description:
 *   Initialize the LCD.  Setup backlight (initially off)
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_40_LCD
void lpc4088_devkit_lcd_initialize(void);
#endif

/****************************************************************************
 * Name: lpc4088_devkit_tsc_setup
 *
 * Description:
 *   This function is called by board-bringup logic to configure the
 *   touchscreen device.  This function will register the driver as
 *   /dev/inputN where N is the minor device number.
 *
 * Input Parameters:
 *   minor - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_INPUT_ADS7843E
int lpc4088_devkit_tsc_setup(int minor);
#endif

/****************************************************************************
 * Name: lpc17_40_djoy_initialization
 *
 * Description:
 *   Initialize and register the discrete joystick driver
 *
 ****************************************************************************/

#ifdef CONFIG_LPC4088_DEVKIT_DJOYSTICK
int lpc17_40_djoy_initialization(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_LPC17XX_40XX_LPC4088_DEVKIT_SRC_LPC4088_DEVKIT_H */
