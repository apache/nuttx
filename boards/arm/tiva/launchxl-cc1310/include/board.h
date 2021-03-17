/****************************************************************************
 * boards/arm/tiva/launchxl-cc1310/include/board.h
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

#ifndef __BOARDS_ARM_TIVA_LAUNCH_CC1310_INCLUDE_BOARD_H
#define __BOARDS_ARM_TIVA_LAUNCH_CC1310_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* Frequency of the ARM core clock */

#define SYSCLK_FREQUENCY      48000000

/* Peripheral Clock (PCLK)
 *
 * Same frequency as the SYSCLK
 */

#define PCLK_FREQUENCY       SYSCLK_FREQUENCY

/* LED definitions **********************************************************/

/* The LaunchXL-cc1310 and two LEDs controlled by software: DIO7_GLED (CR1)
 * and DIO6_RLED (CR2).  A high output value illuminates an LED.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
 * any way.  The following definitions are used to access individual LEDs.
 */

/* LED index values for use with board_userled() */

#define BOARD_GLED        0
#define BOARD_RLED        1
#define BOARD_NLEDS       2

/* LED bits for use with board_userled_all() */

#define BOARD_GLED_BIT    (1 << BOARD_GLED)
#define BOARD_RLED_BIT    (1 << BOARD_RLED)

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/cc1310_autoleds.c.  The LEDs are used to
 * encode OS-related events as follows:
 *
 *   ------------------- ---------------------------- ---- ----
 *   SYMBOL                  Meaning                  GLED RLED
 *   ------------------- ---------------------------- ---- ----
 */

#define LED_STARTED      0 /* NuttX has been started  OFF  OFF   */
#define LED_HEAPALLOCATE 1 /* Heap has been allocated OFF  ON    */
#define LED_IRQSENABLED  1 /* Interrupts enabled      OFF  ON    */
#define LED_STACKCREATED 2 /* Idle stack created      ON   OFF   */
#define LED_INIRQ        3 /* In an interrupt         N/C  GLOW  */
#define LED_SIGNAL       3 /* In a signal handler     N/C  GLOW  */
#define LED_ASSERTION    3 /* An assertion failed     N/C  GLOW  */
#define LED_PANIC        4 /* The system has crashed  OFF  BLINK */
#undef  LED_IDLE           /* MCU is is sleep mode    -Not used- */

/* Thus iF GLED is statically on, NuttX has successfully  booted and is,
 * apparently, running normally.  A soft glow of the RLED means that the
 * board is taking interrupts.   If GLED is off and GLED is flashing at
 * approximately 2Hz, then a fatal error has been detected and the system
 * has halted.
 */

/* Button definitions *******************************************************/

/* The LaunchXL-CC1310 has two push-puttons:
 *
 *   DIO13_BTN1  SW1  Low input sensed when depressed
 *   DIO14_BTN2  SW2  Low input sensed when depressed
 */

#define BUTTON_SW1        0
#define BUTTON_SW2        1
#define NUM_BUTTONS       2

#define BUTTON_SW1_BIT    (1 << BUTTON_SW1)
#define BUTTON_SW2_BIT    (1 << BUTTON_SW2)

/* Pin configuration ********************************************************/

#ifdef CONFIG_TIVA_UART0
/* UART0:
 *
 * The on-board XDS110 Debugger provide a USB virtual serial console using
 * UART0 (PA0/U0RX and PA1/U0TX).
 */

#  define GPIO_UART0_RX &g_gpio_uart0_rx
#  define GPIO_UART0_TX &g_gpio_uart0_tx
#endif

/* DMA **********************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Pin configuration ********************************************************/

struct cc13xx_pinconfig_s; /* Forward reference */

#ifdef CONFIG_TIVA_UART0
extern const struct cc13xx_pinconfig_s g_gpio_uart0_rx;
extern const struct cc13xx_pinconfig_s g_gpio_uart0_tx;
#endif

#endif /* __BOARDS_ARM_TIVA_LAUNCH_CC1310_INCLUDE_BOARD_H */
