/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-xiao/include/board.h
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

#ifndef __BOARDS_XTENSA_ESP32S3_ESP32S3_XIAO_INCLUDE_BOARD_H
#define __BOARDS_XTENSA_ESP32S3_ESP32S3_XIAO_INCLUDE_BOARD_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The ESP32-S3 DevKit board is fitted with a 40MHz crystal */

#define BOARD_XTAL_FREQUENCY    40000000

#ifdef CONFIG_ESP32S3_DEFAULT_CPU_FREQ_MHZ
#  define BOARD_CLOCK_FREQUENCY (CONFIG_ESP32S3_DEFAULT_CPU_FREQ_MHZ * 1000000)
#else
#  define BOARD_CLOCK_FREQUENCY 80000000
#endif

/* LED definitions **********************************************************/

/* Define how many LEDs this board has (needed by userleds) */

#define BOARD_NLEDS       1
#define BOARD_LED_L       0

/* LED GPIO */

#define GPIO_LED1  21

/* LED bits for use with board_userled_all() */

#define BOARD_LED_L_BIT   (1 << BOARD_LED_L)

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/esp32s3_autoleds.c. The LEDs are used to encode
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

/* GPIO pins used by the GPIO Subsystem */

#define BOARD_NGPIOOUT    1 /* Amount of GPIO Output pins */
#define BOARD_NGPIOIN     1 /* Amount of GPIO Input without Interruption */
#define BOARD_NGPIOINT    1 /* Amount of GPIO Input w/ Interruption pins */

#endif /* __BOARDS_XTENSA_ESP32S3_ESP32S3_XIAO_INCLUDE_BOARD_H */
