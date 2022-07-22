/****************************************************************************
 * boards/arm/s32k3xx/s32k344evb/include/board.h
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

/* Copyright 2022 NXP */

#ifndef __BOARDS_ARM_S32K3XX_S32K344EVB_INCLUDE_BOARD_H
#define __BOARDS_ARM_S32K3XX_S32K344EVB_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The S32K344EVB is fitted with a 16 MHz crystal */

#define BOARD_XTAL_FREQUENCY  16000000

/* The S32K344 will run at 160 MHz */

/* LED definitions **********************************************************/

/* The S32K344EVB has two RGB LEDs:
 *
 *   RedLED0    PTA29  (EMIOS1 CH12 / EMIOS2 CH12)
 *   GreenLED0  PTA30  (EMIOS1 CH13 / EMIOS2 CH13)
 *   BlueLED0   PTA31  (EMIOS1 CH14 / FXIO D0)
 *
 *   RedLED1    PTB18  (EMIOS1 CH15 / EMIOS2 CH14 / FXIO D1)
 *   GreenLED1  PTB25  (EMIOS1 CH21 / EMIOS2 CH21 / FXIO D6)
 *   BlueLED1   PTE12  (EMIOS1 CH5  / FXIO D8)
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
 * any way.  The following definitions are used to access individual RGB
 * components.
 *
 * The RGB components could, alternatively be controlled through PWM using
 * the common RGB LED driver.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED0_R      0
#define BOARD_LED0_G      1
#define BOARD_LED0_B      2

#define BOARD_LED1_R      3
#define BOARD_LED1_G      4
#define BOARD_LED1_B      5

#define BOARD_NLEDS       6

/* LED bits for use with board_userled_all() */

#define BOARD_LED0_R_BIT  (1 << BOARD_LED0_R)
#define BOARD_LED0_G_BIT  (1 << BOARD_LED0_G)
#define BOARD_LED0_B_BIT  (1 << BOARD_LED0_B)

#define BOARD_LED1_R_BIT  (1 << BOARD_LED1_R)
#define BOARD_LED1_G_BIT  (1 << BOARD_LED1_G)
#define BOARD_LED1_B_BIT  (1 << BOARD_LED1_B)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the LEDs on board
 * the S32K344EVB.  The following definitions describe how NuttX controls the
 * LEDs:
 *
 *      SYMBOL            Meaning                         LED state
 *                                                        RED    GREEN  BLUE
 *      ----------------  -----------------------------  -------------------
 */

#define LED_STARTED       1 /* NuttX has been started     OFF    OFF    OFF */
#define LED_HEAPALLOCATE  2 /* Heap has been allocated    OFF    OFF    ON  */
#define LED_IRQSENABLED   0 /* Interrupts enabled         OFF    OFF    ON  */
#define LED_STACKCREATED  3 /* Idle stack created         OFF    ON     OFF */
#define LED_INIRQ         0 /* In an interrupt           (No change)        */
#define LED_SIGNAL        0 /* In a signal handler       (No change)        */
#define LED_ASSERTION     0 /* An assertion failed       (No change)        */
#define LED_PANIC         4 /* The system has crashed     FLASH  OFF    OFF */
#undef  LED_IDLE            /* S32K344 is in sleep mode  (Not used)         */

/* Button definitions *******************************************************/

/* The S32K344EVB supports two buttons:
 *
 *   SW0  PTB26  (EIRQ13 / WKPU41)
 *   SW1  PTB19  (WKPU38)
 */

#define BUTTON_SW0        0
#define BUTTON_SW1        1
#define NUM_BUTTONS       2

#define BUTTON_SW0_BIT    (1 << BUTTON_SW0)
#define BUTTON_SW1_BIT    (1 << BUTTON_SW1)

/* UART selections **********************************************************/

/* By default, the serial console will be provided on the OpenSDA VCOM port:
 *
 *   OpenSDA UART RX  PTA15  (LPUART6_RX)
 *   OpenSDA UART TX  PTA16  (LPUART6_TX)
 */

#define PIN_LPUART6_RX    PIN_LPUART6_RX_1   /* PTA15 */
#define PIN_LPUART6_TX    PIN_LPUART6_TX_1   /* PTA16 */

/* LPUART13  J58 USB-UART */

#define PIN_LPUART13_RX   PIN_LPUART13_RX_2  /* PTC27 */
#define PIN_LPUART13_TX   PIN_LPUART13_TX_2  /* PTC26 */

/* SPI selections ***********************************************************/

/* LPSPI0  FS26 Safety SBC */

#define PIN_LPSPI0_SCK    PIN_LPSPI0_SCK_1   /* PTC8 */
#define PIN_LPSPI0_MISO   PIN_LPSPI0_SIN_1   /* PTC9 */
#define PIN_LPSPI0_MOSI   PIN_LPSPI0_SOUT_2  /* PTB1 */

#define PIN_LPSPI0_PCS    (PIN_PTB0 | GPIO_LOWDRIVE | GPIO_OUTPUT_ONE)  /* PTB0 */

/* LPSPI1  J353 Arduino Header */

#define PIN_LPSPI1_SCK    PIN_LPSPI1_SCK_4   /* PTB14 */
#define PIN_LPSPI1_MISO   PIN_LPSPI1_SIN_4   /* PTB15 */
#define PIN_LPSPI1_MOSI   PIN_LPSPI1_SOUT_3  /* PTB16 */

#define PIN_LPSPI1_PCS    PIN_LPSPI1_PCS3_2  /* PTB17 */

/* I2C selections ***********************************************************/

/* LPI2C1  J353 Arduino Header / J63 ENET PHY / MMA8452Q Accelerometer
 *         / SGTL5000 Audio Codec
 */

#define PIN_LPI2C1_SCL    PIN_LPI2C1_SCL_1   /* PTC7 */
#define PIN_LPI2C1_SDA    PIN_LPI2C1_SDA_1   /* PTC6 */

/* FLEXCAN selections *******************************************************/

#define PIN_CAN0_TX       PIN_CAN0_TX_1      /* PTA7 */
#define PIN_CAN0_RX       PIN_CAN0_RX_1      /* PTA6 */

#endif  /* __BOARDS_ARM_S32K3XX_S32K344EVB_INCLUDE_BOARD_H */
