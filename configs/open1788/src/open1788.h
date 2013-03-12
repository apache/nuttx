/************************************************************************************
 * configs/open1788/src/open1788.h
 * arch/arm/src/board/open1788.n
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef _CONFIGS_OPEN1788_SRC_OPEN1788_H
#define _CONFIGS_OPEN1788_SRC_OPEN1788_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Open1788 GPIO Pin Definitions ****************************************************/
/* GPIO P2[21] connects to the Ready/Busy pin of the NAND part.  We need to
 * reconfigure this pin as normal GPIO input if NAND is used.
 */

#define GPIO_NAND_RB (GPIO_INPUT | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN21)

/* If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
 * any way.  The following definitions are used to access individual LEDs.
 *
 * LED1 -- Connected to P1[14]
 * LED2 -- Connected to P0[16]
 * LED3 -- Connected to P1[13]
 * LED4 -- Connected to P4[27]
 *
 * These LEDs are connecte to ground so a high output value will illuminate them.
 */

#define GPIO_LED1    (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT1 | GPIO_PIN14)
#define GPIO_LED2    (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN16)
#define GPIO_LED3    (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT1 | GPIO_PIN13)
#define GPIO_LED4    (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT4 | GPIO_PIN27)

/* Button definitions ***************************************************************/
/* The Open1788 supports several buttons.  All will read "1" when open and "0"
 * when closed
 *
 * USER1           -- Connected to P4[26]
 * USER2           -- Connected to P2[22]
 * USER3           -- Connected to P0[10]
 *
 * And a Joystick
 *
 * JOY_A           -- Connected to P2[25]
 * JOY_B           -- Connected to P2[26]
 * JOY_C           -- Connected to P2[23]
 * JOY_D           -- Connected to P2[19]
 * JOY_CTR         -- Connected to P0[14]
 *
 * The switches are all connected to ground and should be pulled up and sensed
 * with a value of '0' when closed.
 */

#define GPIO_USER1   (GPIO_INTBOTH | GPIO_FLOAT | GPIO_PORT4 | GPIO_PIN26)
#define GPIO_USER2   (GPIO_INTBOTH | GPIO_FLOAT | GPIO_PORT2 | GPIO_PIN22)
#define GPIO_USER3   (GPIO_INTBOTH | GPIO_FLOAT | GPIO_PORT0 | GPIO_PIN10)

#define GPIO_JOY_A   (GPIO_INTBOTH | GPIO_FLOAT | GPIO_PORT2 | GPIO_PIN25)
#define GPIO_JOY_B   (GPIO_INTBOTH | GPIO_FLOAT | GPIO_PORT2 | GPIO_PIN26)
#define GPIO_JOY_C   (GPIO_INTBOTH | GPIO_FLOAT | GPIO_PORT2 | GPIO_PIN23)
#define GPIO_JOY_D   (GPIO_INTBOTH | GPIO_FLOAT | GPIO_PORT2 | GPIO_PIN19)
#define GPIO_JOY_CTR (GPIO_INTBOTH | GPIO_FLOAT | GPIO_PORT0 | GPIO_PIN14)

/* SD Card **************************************************************************/
/* The SD card detect (CD) signal is on P0[13].  This signal is shared.  It is also
 * used for MOSI1 and USB_UP_LED.  The CD pin may be disconnected.  There is a jumper
 * on board that enables the CD pin.
 */

#if 0 /* The CD pin may be interrupting */
#  define GPIO_SD_CD (GPIO_INTBOTH | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN13)
#else
#  define GPIO_SD_CD (GPIO_INPUT | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN13)
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public data
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: lpc17_sspinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the WaveShare Open1788 board.
 *
 ************************************************************************************/

void weak_function lpc17_sspinitialize(void);

/************************************************************************************
 * Name: lpc17_sdram_initialize
 *
 * Description:
 *   Initialize SDRAM
 *
 ************************************************************************************/

#ifdef CONFIG_LPC17_EMC
#ifdef CONFIG_LPC17_EMC_SDRAM
void lpc17_sdram_initialize(void);
#endif

/************************************************************************************
 * Name: lpc17_nor_initialize
 *
 * Description:
 *   Initialize NOR FLASH
 *
 ************************************************************************************/

#ifdef CONFIG_LPC17_EMC_NOR
void lpc17_nor_initialize(void);
#endif

/************************************************************************************
 * Name: lpc17_nand_initialize
 *
 * Description:
 *   Initialize NAND FLASH
 *
 ************************************************************************************/

#ifdef CONFIG_LPC17_EMC_NAND
void lpc17_nand_initialize(void);
#endif
#endif /* CONFIG_LPC17_EMC */

/****************************************************************************
 * Name: nsh_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization for NSH.
 *
 *   CONFIG_NSH_ARCHINIT=y :
 *     Called from the NSH library
 *
 *   CONFIG_BOARD_INITIALIZE=y, CONFIG_NSH_LIBRARY=y, &&
 *   CONFIG_NSH_ARCHINIT=n:
 *     Called from board_initialize().
 *
 ****************************************************************************/

#ifdef CONFIG_NSH_LIBRARY
int nsh_archinitialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* _CONFIGS_OPEN1788_SRC_OPEN1788_H */

