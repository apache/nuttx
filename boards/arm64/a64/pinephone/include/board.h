/****************************************************************************
 * boards/arm64/a64/pinephone/include/board.h
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

#ifndef __BOARDS_ARM64_A64_PINEPHONE_INCLUDE_BOARD_H
#define __BOARDS_ARM64_A64_PINEPHONE_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LED definitions **********************************************************/

/* LED index values for use with board_userled() */

typedef enum
{
    BOARD_LED1 = 0,  /* Green LED */
    BOARD_LED2 = 1,  /* Red LED */
    BOARD_LED3 = 2,  /* Blue LED */
    BOARD_LEDS       /* Number of LEDs */
} led_typedef_enum;

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)
#define BOARD_LED3_BIT    (1 << BOARD_LED3)

/* If CONFIG_ARCH_LEDS is defined, the usage by the board port is defined in
 * include/board.h and src/pinephone_autoleds.c. The LEDs are used to encode
 * OS-related events as follows:
 *
 *   SYMBOL                     Meaning                      LED state
 *                                                        LED1  LED2  LED3
 *   ----------------------  --------------------------  ------ ------ ---
 */

#define LED_STARTED        0 /* NuttX has been started   OFF    OFF   OFF  */
#define LED_HEAPALLOCATE   1 /* Heap has been allocated  ON     OFF   OFF  */
#define LED_IRQSENABLED    2 /* Interrupts enabled       OFF    ON    OFF  */
#define LED_STACKCREATED   3 /* Idle stack created       OFF    OFF   ON   */
#define LED_INIRQ          4 /* In an interrupt          ON     ON    OFF  */
#define LED_SIGNAL         5 /* In a signal handler      ON     OFF   ON   */
#define LED_ASSERTION      6 /* An assertion failed      OFF    ON    ON   */
#define LED_PANIC          7 /* The system has crashed   FLASH  ON    ON   */
#define LED_IDLE           8 /* MCU is is sleep mode     OFF    FLASH OFF  */

#endif /* __BOARDS_ARM64_A64_PINEPHONE_INCLUDE_BOARD_H */
