/****************************************************************************
 * boards/arm/s32k1xx/rddrone-bms772/include/board.h
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

#ifndef __BOARDS_ARM_S32K1XX_RDDRONE_BMS772_INCLUDE_BOARD_H
#define __BOARDS_ARM_S32K1XX_RDDRONE_BMS772_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The RDDRONE-BMS772 is fitted with a 32 MHz crystal */

#define BOARD_XTAL_FREQUENCY                  32000000

/* The S32K144 will run at 80 MHz in RUN mode */

#define RDDRONE_BMS772_RUN_SYSCLK_FREQUENCY   80000000

/* The S32K144 will run at 2 MHz in VLPR mode */

#define RDDRONE_BMS772_VLPR_SYSCLK_FREQUENCY   2000000

/* LED definitions **********************************************************/

/* The RDDRONE-BMS772 has one RGB LED:
 *
 *   RedLED    PTD16  (FTM0 CH1)
 *   GreenLED  PTB13  (FTM0 CH1)
 *   BlueLED   PTD15  (FTM0 CH0)
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
 * the RDDRONE-BMS772.  The following definitions describe how NuttX controls
 * the LEDs:
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
#undef  LED_IDLE            /* S32K144 is in sleep mode  (Not used)         */

/* UART selections **********************************************************/

/* By default, the serial console will be provided on the DCD-LZ UART
 * (available on the 7-pin DCD-LZ debug connector J19):
 *
 *   DCD-LZ UART RX  PTC6  (LPUART1_RX)
 *   DCD-LZ UART TX  PTC7  (LPUART1_TX)
 */

#define PIN_LPUART1_RX    PIN_LPUART1_RX_1   /* PTC6 */
#define PIN_LPUART1_TX    PIN_LPUART1_TX_1   /* PTC7 */

/* Header J21  (LPUART0) */

#define PIN_LPUART0_RX    PIN_LPUART0_RX_1   /* PTB0 */
#define PIN_LPUART0_TX    PIN_LPUART0_TX_1   /* PTB1 */

/* SPI selections ***********************************************************/

/* UJA1169TK/F SBC SPI  (LPSPI0) */

#define PIN_LPSPI0_SCK    PIN_LPSPI0_SCK_2   /* PTB2 */
#define PIN_LPSPI0_MISO   PIN_LPSPI0_SIN_2   /* PTB3 */
#define PIN_LPSPI0_MOSI   PIN_LPSPI0_SOUT_3  /* PTB4 */
#define PIN_LPSPI0_PCS    PIN_LPSPI0_PCS0_2  /* PTB5 */

/* MC33772B BCC SPI  (LPSPI1) */

#define PIN_LPSPI1_SCK    PIN_LPSPI1_SCK_2   /* PTD0 */
#define PIN_LPSPI1_MISO   PIN_LPSPI1_SIN_2   /* PTD1 */
#define PIN_LPSPI1_MOSI   PIN_LPSPI1_SOUT_2  /* PTD2 */
#define PIN_LPSPI1_PCS    PIN_LPSPI1_PCS0_1  /* PTD3 */

/* I2C selections ***********************************************************/

/* I2C Master Connector J23  (LPI2C0 Master) */

#define PIN_LPI2C0_SDA    PIN_LPI2C0_SDA_2   /* PTA2 */
#define PIN_LPI2C0_SCL    PIN_LPI2C0_SCL_2   /* PTA3 */

/* I2C/SMBus Connector J18  (LPI2C0 Slave) */

#define PIN_LPI2C0_SDAS   PIN_LPI2C0_SDAS_1  /* PTA1 */
#define PIN_LPI2C0_SCLS   PIN_LPI2C0_SCLS_1  /* PTA0 */

/* CAN selections ***********************************************************/

/* UJA1169TK/F SBC CAN  (CAN0) */

#define PIN_CAN0_RX       PIN_CAN0_RX_3      /* PTE4 */
#define PIN_CAN0_TX       PIN_CAN0_TX_3      /* PTE5 */

#endif  /* __BOARDS_ARM_S32K1XX_RDDRONE_BMS772_INCLUDE_BOARD_H */
