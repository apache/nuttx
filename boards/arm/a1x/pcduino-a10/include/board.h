/****************************************************************************
 * boards/arm/a1x/pcduino-a10/include/board.h
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
 ****************************************************************************/

#ifndef __BOARDS_ARM_A1X_PCDUINO_A10_INCLUDE_BOARD_H
#define __BOARDS_ARM_A1X_PCDUINO_A10_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/a1x_piocfg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* Since NuttX is booted from a loader on the A10, clocking should already
 * be setup when NuttX starts.
 */

/* LED definitions **********************************************************/

/* The pcDuino v1 has four green LEDs; three can be controlled from software.
 * Two are tied to ground and, hence, illuminated by driving the output pins
 * to a high value:
 *
 *  1. LED1 SPI0_CLK  SPI0_CLK/UART5_RX/EINT23/PI11
 *  2. LED5 IPSOUT    From the PMU (not controllable by software)
 *
 * And two are pull high and, hence, illuminated by grounding the output:
 *
 *   3. LED3 RX_LED    LCD1_D16/ATAD12/KP_IN6/SMC_DET/EINT16/CSI1_D16/PH16
 *   4. LED4 TX_LED    LCD1_D15/ATAD11/KP_IN5/SMC_VPPPP/EINT15/CSI1_D15/PH15
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1        0
#define BOARD_LED3        1
#define BOARD_LED4        2
#define BOARD_NLEDS       3

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED3_BIT    (1 << BOARD_LED3)
#define BOARD_LED4_BIT    (1 << BOARD_LED4)

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/a1x_leds.c. The LEDs are used to encode OS-related
 * events as follows:
 *
 *      SYMBOL            Value Meaning                    LED state
 *                                                    LED1 LED3 LED4
 *      ----------------- ----- -----------------------  ---- ---- --------
 */

#define LED_STARTED         0   /* NuttX has been started   ON   OFF  OFF */
#define LED_HEAPALLOCATE    1   /* Heap has been allocated  OFF  ON   OFF */
#define LED_IRQSENABLED     2   /* Interrupts enabled       ON   ON   OFF */
#define LED_STACKCREATED    2   /* Idle stack created       ON   ON   OFF */
#define LED_INIRQ           3   /* In an interrupt          N/C  N/C  Soft glow */
#define LED_SIGNAL          3   /* In a signal handler      N/C  N/C  Soft glow */
#define LED_ASSERTION       3   /* An assertion failed      N/C  N/C  Soft glow */
#define LED_PANIC           3   /* The system has crashed   N/C  N/C  2Hz Flashing */

/*      LED_IDLE           ---  /* MCU is is sleep mode         Not used
 *
 * After booting, LED1 and 3 are not longer used by the system and can be
 * used for other purposes by the application (Of course, all LEDs are
 * available to the application if CONFIG_ARCH_LEDS is not defined.
 */

/* Button definitions *******************************************************/

/* There are a total of five switches on-board.
 * All pulled high and, hence, will be sensed as low when closed.
 *
 *   SW1 Reset     (not available to software)
 *   SW2 UBOOT     UBOOT_SEL (?)
 *   SW3 Key_Back  LCD1_D17/ATAD13/KP_IN7/SMC_VCCEN/EINT17/CSI1_D17/PH17
 *   SW4 Key_Home  LCD1_D18/ATAD14/KP_OUT0/SMC_SLK/EINT18/CSI1_D18/PH18
 *   SW5 Key_Menu  LCD1_D19/ATAD15/KP_OUT1/SMC_SDA/EINT19/CSI1_D19/PH19
 */

#define BUTTON_KEY_BACK     0
#define BUTTON_KEY_HOME     1
#define BUTTON_KEY_MENU     2
#define NUM_BUTTONS         3

#define BUTTON_KEY_BACK_BIT (1 << BUTTON_KEY_BACK)
#define BUTTON_KEY_HOME_BIT (1 << BUTTON_KEY_HOME)
#define BUTTON_KEY_MENU_BIT (1 << BUTTON_KEY_MENU)

/* NAND *********************************************************************/

/* GPIO pin disambiguation **************************************************/

/* UARTs ********************************************************************/

/* Two UART connections are available:
 *
 * 1. UART0 is available on J5 Debug Port.
 *
 *    J15 Pin 1 Rx                UART0-RX  UART0_RX/IR1_RX/PB23
 *    J15 Pin 2 Tx                UART0-TX  UART0_TX/IR1_TX/PB22
 *
 * 2. UART2 is available on J11
 *
 *    J11 Pin1  UART-Rx / GPIO0   UART2_RX  EINT31/SPI1_MISO/UART2_RX/PI19
 *    J11 Pin2  UART-Tx / GPIO1   UART2_TX  EINT30/SPI1_MOSI/UART2_TX/PI18
 */

#define PIO_UART0_RX    PIO_UART0_RX_1
#define PIO_UART0_TX    PIO_UART0_TX_1

#define PIO_UART2_RX    PIO_UART2_RX_1
#define PIO_UART2_TX    PIO_UART2_TX_1

/****************************************************************************
 * Assembly Language Macros
 ****************************************************************************/

#ifdef __ASSEMBLY__
  .macro config_sdram
  .endm
#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_A1X_PCDUINO_A10_INCLUDE_BOARD_H */
