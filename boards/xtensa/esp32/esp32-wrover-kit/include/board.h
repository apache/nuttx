/****************************************************************************
 * boards/xtensa/esp32/esp32-wrover-kit/include/board.h
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

#ifndef __BOARDS_XTENSA_ESP32_ESP32_WROVER_KIT_INCLUDE_BOARD_H
#define __BOARDS_XTENSA_ESP32_ESP32_WROVER_KIT_INCLUDE_BOARD_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The ESP32 Ethernet kit board V2 is fitted with either a 26 or a 40MHz
 * crystal
 */

#ifdef CONFIG_ESP32_XTAL_26MHz
#  define BOARD_XTAL_FREQUENCY  26000000
#else
#  define BOARD_XTAL_FREQUENCY  40000000
#endif

/* Clock reconfiguration is currently disabled, so the CPU will be running
 * at the XTAL frequency or at two times the XTAL frequency, depending upon
 * how we load the code:
 *
 * - If we load the code into FLASH at address 0x1000 where it is started by
 *   the second level bootloader, then the frequency is the crystal
 *   frequency.
 * - If we load the code into IRAM after the second level bootloader has run
 *   this frequency will be  twice the crystal frequency.
 *
 * Don't ask me for an explanation.
 */

/* Note: The bootloader (esp-idf bootloader.bin) configures:
 *
 * - CPU frequency to 80MHz
 * - The XTAL frequency according to the SDK config CONFIG_ESP32_XTAL_FREQ,
 *   which is 40MHz by default.
 *
 * Reference:
 *     https://github.com/espressif/esp-idf/blob
 *           /6fd855ab8d00d23bad4660216bc2122c2285d5be/components
 *           /bootloader_support/src/bootloader_clock.c#L38-L62
 */

#ifdef CONFIG_ESP32_RUN_IRAM
#  define BOARD_CLOCK_FREQUENCY (2 * BOARD_XTAL_FREQUENCY)
#else
#ifdef CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ
#  define BOARD_CLOCK_FREQUENCY (CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ * 1000000)
#else
#  define BOARD_CLOCK_FREQUENCY 80000000
#endif
#endif

/* GPIO definitions *********************************************************/

/* Display */

#define DISPLAY_SPI       2
#define DISPLAY_DC        21
#define DISPLAY_RST       18
#define DISPLAY_BCKL      5

/* LED definitions **********************************************************/

/* Define how many LEDs this board has (needed by userleds) */

#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_LED3        2
#define BOARD_NLEDS       3

#define BOARD_LED_RED     BOARD_LED1
#define BOARD_LED_GREEN   BOARD_LED2
#define BOARD_LED_BLUE    BOARD_LED3

/* LED bits for use with autoleds */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)
#define BOARD_LED3_BIT    (1 << BOARD_LED3)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the 3 LEDs on
 * board the ESP-WROVER-KIT.  The following definitions describe how
 * NuttX controls the LEDs:
 */

/* These values index an array that contains the bit definitions for the
 * correct LEDs (see esp32_autoleds.c).
 */

#define LED_STARTED       0  /* LED2 */
#define LED_HEAPALLOCATE  1  /* LED3 */
#define LED_IRQSENABLED   2  /* LED3 + LED2 */
#define LED_STACKCREATED  3  /* LED3 */
#define LED_INIRQ         4  /* LED1 + LED3 */
#define LED_SIGNAL        5  /* LED2 + LED3 */
#define LED_ASSERTION     6  /* LED1 + LED2 + LED3 */
#define LED_PANIC         7  /* LED1  + N/C  + N/C */

/* The values below are used only to distinguish between the CPUs.
 * The LEDs are actually mapped as:
 *    CPU0 -> GPIO_LED1 (Red LED)
 *    CPU1 -> GPIO_LED2 (Green LED)
 * Note that from the previous list only LED_HEAPALLOCATE will still be
 * valid.  This is to avoid collisions and to keep a way to show a successful
 * heap allocation.  The LED used is still LED3 (Blue LED).
 */

#ifdef CONFIG_ARCH_LEDS_CPU_ACTIVITY
#  define LED_CPU0        8
#  define LED_CPU1        9
#  define LED_CPU         (LED_CPU0 + up_cpu_index())
#endif

/* GPIO pins used by the GPIO Subsystem */

#define BOARD_NGPIOIN     1 /* Amount of GPIO Input pins */
#define BOARD_NGPIOOUT    1 /* Amount of GPIO Output pins */
#define BOARD_NGPIOINT    1 /* Amount of GPIO Input w/ Interruption pins */

#endif /* __BOARDS_XTENSA_ESP32_ESP32_WROVER_KIT_INCLUDE_BOARD_H */
