/****************************************************************************
 * boards/xtensa/esp32s2/esp32s2-saola-1/include/board.h
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

#ifndef __BOARDS_XTENSA_ESP32S2_ESP32S2_SAOLA_1_INCLUDE_BOARD_H
#define __BOARDS_XTENSA_ESP32S2_ESP32S2_SAOLA_1_INCLUDE_BOARD_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The ESP32S2 board V2 is fitted with a 40MHz crystal */

#define BOARD_XTAL_FREQUENCY  40000000

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
 * - The XTAL frequency according to the SDK config CONFIG_ESP32S2_XTAL_FREQ,
 *   which is 40MHz by default.
 *
 * Reference:
 *     https://github.com/espressif/esp-idf/blob
 *           /6fd855ab8d00d23bad4660216bc2122c2285d5be/components
 *           /bootloader_support/src/bootloader_clock.c#L38-L62
 */

#ifdef CONFIG_ESP32S2_RUN_IRAM
#  define BOARD_CLOCK_FREQUENCY (2 * BOARD_XTAL_FREQUENCY)
#else
#ifdef CONFIG_ESP32S2_DEFAULT_CPU_FREQ_MHZ
#  define BOARD_CLOCK_FREQUENCY (CONFIG_ESP32S2_DEFAULT_CPU_FREQ_MHZ * 1000000)
#else
#  define BOARD_CLOCK_FREQUENCY 80000000
#endif
#endif

/* LED definitions **********************************************************/

/* Define how many LEDs this board has (needed by userleds) */

#define BOARD_NLEDS       1

/* GPIO pins used by the GPIO Subsystem */

#define BOARD_NGPIOOUT    2 /* Amount of GPIO Output pins */
#define BOARD_NGPIOIN     1 /* Amount of GPIO Input without Interruption */
#define BOARD_NGPIOINT    1 /* Amount of GPIO Input w/ Interruption pins */

#endif /* __BOARDS_XTENSA_ESP32S2_ESP32S2_SAOLA_1_INCLUDE_BOARD_H */
