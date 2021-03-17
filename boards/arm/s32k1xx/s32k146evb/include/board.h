/****************************************************************************
 * boards/arm/s32k1xx/s32k146evb/include/board.h
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

#ifndef __BOARDS_ARM_S32K146EVB_INCLUDE_BOARD_H
#define __BOARDS_ARM_S32K146EVB_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The S32K146EVB is fitted with a 8MHz Crystal */

#define BOARD_XTAL_FREQUENCY 8000000

/* The S32K146 will run at 112MHz */

/* LED definitions **********************************************************/

/* The S32K146EVB has one RGB LED:
 *
 *   RedLED   PTD15 (FTM0 CH0)
 *   GreenLED PTD16 (FTM0 CH1)
 *   BlueLED  PTD0  (FTM0 CH2)
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
 * any way.  The following definitions are used to access individual RGB
 * components.
 *
 * The RGB components could, alternatively be controlled through PWM using
 * the common RGB LED driver.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED_R       0
#define BOARD_LED_G       1
#define BOARD_LED_B       2
#define BOARD_NLEDS       3

/* LED bits for use with board_userled_all() */

#define BOARD_LED_R_BIT   (1 << BOARD_LED_R)
#define BOARD_LED_G_BIT   (1 << BOARD_LED_G)
#define BOARD_LED_B_BIT   (1 << BOARD_LED_B)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the LEDs on board
 * the S32K146EVB.  The following definitions describe how NuttX controls the
 * LEDs:
 *
 *   SYMBOL                Meaning                      LED state
 *                                                      RED   GREEN  BLUE
 *   -------------------  ----------------------------  -----------------
 */

#define LED_STARTED       1 /* NuttX has been started    OFF   OFF    OFF */
#define LED_HEAPALLOCATE  2 /* Heap has been allocated   OFF   OFF    ON  */
#define LED_IRQSENABLED   0 /* Interrupts enabled        OFF   OFF    ON  */
#define LED_STACKCREATED  3 /* Idle stack created        OFF   ON     OFF */
#define LED_INIRQ         0 /* In an interrupt          (no change)       */
#define LED_SIGNAL        0 /* In a signal handler      (no change)       */
#define LED_ASSERTION     0 /* An assertion failed      (no change)       */
#define LED_PANIC         4 /* The system has crashed    FLASH OFF    OFF */
#undef  LED_IDLE            /* S32K146EVB in sleep mode (Not used)        */

/* Button definitions *******************************************************/

/* The S32K146EVB supports two buttons:
 *
 *   SW2  PTC12
 *   SW3  PTC13
 */

#define BUTTON_SW2         0
#define BUTTON_SW3         1
#define NUM_BUTTONS        2

#define BUTTON_SW2_BIT    (1 << BUTTON_SW2)
#define BUTTON_SW3_BIT    (1 << BUTTON_SW3)

/* Alternate function pin selections ****************************************/

/* By default, the serial console will be provided on the OpenSDA VCOM port:
 *
 *   OpenSDA UART TX  PTC7 (LPUART1_TX)
 *   OpenSDA UART RX  PTC6 (LPUART1_RX)
 */

#define PIN_LPUART0_RX    PIN_LPUART0_RX_1  /* PTB0 */
#define PIN_LPUART0_TX    PIN_LPUART0_TX_1  /* PTB1 */

#define PIN_LPUART1_RX    PIN_LPUART1_RX_1  /* PTC6 */
#define PIN_LPUART1_TX    PIN_LPUART1_TX_1  /* PTC7 */

#define PIN_LPUART2_RX    PIN_LPUART2_RX_1  /* PTA8 */
#define PIN_LPUART2_TX    PIN_LPUART2_TX_1  /* PTA9 */

/* SPI selections ***********************************************************/

#define PIN_LPSPI0_SCK   PIN_LPSPI0_SCK_2   /* PTB2 */
#define PIN_LPSPI0_MISO  PIN_LPSPI0_SIN_2   /* PTB3 */
#define PIN_LPSPI0_MOSI  PIN_LPSPI0_SOUT_3  /* PTB4 */
#define PIN_LPSPI0_PCS   PIN_LPSPI0_PCS0_1  /* PTB0 */

#define PIN_LPSPI1_SCK   PIN_LPSPI1_SCK_1   /* PTB14 */
#define PIN_LPSPI1_MISO  PIN_LPSPI1_SIN_1   /* PTB15 */
#define PIN_LPSPI1_MOSI  PIN_LPSPI1_SOUT_1  /* PTB16 */
#define PIN_LPSPI1_PCS   PIN_LPSPI1_PCS3    /* PTB17 */

#define PIN_LPSPI2_SCK   PIN_LPSPI2_SCK_2   /* PTE15 */
#define PIN_LPSPI2_MISO  PIN_LPSPI2_SIN_2   /* PTE16 */
#define PIN_LPSPI2_MOSI  PIN_LPSPI2_SOUT_1  /* PTA8  */
#define PIN_LPSPI2_PCS   PIN_LPSPI2_PCS0_2  /* PTA9  */

/* I2C selections ***********************************************************/

#define PIN_LPI2C0_SCL   PIN_LPI2C0_SCL_2   /* PTA3 */
#define PIN_LPI2C0_SDA   PIN_LPI2C0_SDA_2   /* PTA2 */

#endif  /* __BOARDS_ARM_S32K146EVB_INCLUDE_BOARD_H */
