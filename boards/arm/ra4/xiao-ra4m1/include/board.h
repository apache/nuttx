/****************************************************************************
 * boards/arm/ra4/xiao-ra4m1/include/board.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __BOARDS_ARM_RA4_XIAO_RA4M1_INCLUDE_BOARD_H
#define __BOARDS_ARM_RA4_XIAO_RA4M1_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* This is the canonical configuration:
 *   System Clock source      : HOCO
 *   ICLK(Hz)                 : 64000000
 *   PCLKA(Hz)                : 64000000
 *   PCLKB(Hz)                : 64000000
 *   PCLKC(Hz)                : 64000000
 *   PCLKD(Hz)                : 64000000
 *   FCLK(Hz)                 : 64000000
 *   USBCLK(Hz)               : 0
 */

/* Clock selection HOCO, MOCO, LOCO, PLL */

#define RA_CKSEL  R_SYSTEM_SCKSCR_CKSEL_HOCO

#define RA_HOCOEN     !R_OFS1_HOCOEN /* Inverted logic to enable */
#define RA_HOCO_FREQUENCY  R_OFS1_HOCOFRQ1_64MHZ

#define RA_ICK_DIV       R_SYSTEM_SCKDIVCR_ICK_DIV_1
#define RA_ICLK_FREQUENCY  64000000
#define RA_FCK_DIV       R_SYSTEM_SCKDIVCR_FCK_DIV_1
#define RA_FCK_FREQUENCY  64000000
#define RA_PCKA_DIV      R_SYSTEM_SCKDIVCR_PCKA_DIV_1
#define RA_PCKA_FREQUENCY  64000000
#define RA_PCKB_DIV      R_SYSTEM_SCKDIVCR_PCKB_DIV_1
#define RA_PCKB_FREQUENCY  64000000
#define RA_PCKC_DIV      R_SYSTEM_SCKDIVCR_PCKC_DIV_1
#define RA_PCKC_FREQUENCY  64000000
#define RA_PCKD_DIV      R_SYSTEM_SCKDIVCR_PCKD_DIV_1
#define RA_PCKD_FREQUENCY  64000000

/* Alternate function pin selections */

#define GPIO_SCI2_RX   GPIO_RXD2_MISO2_SCL2_1  /* P301 */
#define GPIO_SCI2_TX   GPIO_TXD2_MOSI2_SDA2_1  /* P302 */

#define GPIO_SCI1_RX   GPIO_RXD1_MISO1_SCL1_3  /* P502 */
#define GPIO_SCI1_TX   GPIO_TXD1_MOSI1_SDA1_3  /* P501 */

#define GPIO_SCI9_RX   GPIO_RXD9_MISO9_SCL9_1  /* P110 */
#define GPIO_SCI9_TX   GPIO_TXD9_MOSI9_SDA9_1  /* P109 */

/* LED pin selections */

#define GPIO_LED    (gpio_pinset_t){ PORT0,PIN11, (GPIO_OUPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}   /* P011 */

#define LED_DRIVER_PATH "/dev/userleds"

/* LED index values for use with board_userled() */

#define BOARD_LED_L       0
#define BOARD_NLEDS       1

/* LED bits for use with board_userled_all() */

#define BOARD_LED_L_BIT   (1 << BOARD_LED_L)

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/ra4m1_leds.c. The LEDs are used to encode
 * OS-related events as follows:
 *
 *  SYMBOL                MEANING                         LED STATE
 *                                                         L   TX   RX
 *  -----------------------  --------------------------  ---- ---- ----
 */

 #define LED_STARTED       0  /* NuttX has been started   OFF OFF  OFF      */
 #define LED_HEAPALLOCATE  0  /* Heap has been allocated  OFF OFF  OFF      */
 #define LED_IRQSENABLED   0  /* Interrupts enabled       OFF OFF  OFF      */
 #define LED_STACKCREATED  1  /* Idle stack created       ON  OFF  OFF      */
 #define LED_INIRQ         2  /* In an interrupt          N/C GLOW OFF      */
 #define LED_SIGNAL        2  /* In a signal handler      N/C GLOW OFF      */
 #define LED_ASSERTION     2  /* An assertion failed      N/C GLOW OFF      */
 #define LED_PANIC         3  /* The system has crashed   N/C N/C  Blinking */
 #define LED_PANIC         3  /* MCU is is sleep mode    ---- Not used ---- */

/* GPIO definitions *********************************************************/

#define BOARD_NGPIOOUT          1
#define BOARD_NGPIOIN           1

#define GPIO_IN1    (gpio_pinset_t){ PORT0,PIN14, (GPIO_INPUT | GPIO_LOW_DRIVE)}                   /* P014 */
#define GPIO_OUT1   (gpio_pinset_t){ PORT0,PIN0, (GPIO_OUPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}  /* P000 */

/* ID_CODE */

#define IDCODE1   0xFFFFFFFF
#define IDCODE2   0xFFFFFFFF
#define IDCODE3   0xFFFFFFFF
#define IDCODE4   0xFFFFFFFF

#endif /* __BOARDS_ARM_RA4_XIAO_RA4M1_INCLUDE_BOARD_H */
