/****************************************************************************
 * boards/xtensa/esp32/esp32-core/include/board.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

#ifndef __BOARDS_XTENSA_ESP32_ESP32_CORE_INCLUDE_BOARD_H
#define __BOARDS_XTENSA_ESP32_ESP32_CORE_INCLUDE_BOARD_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The ESP32 Core board V2 is fitted with either a 26 a 40MHz crystal */

#ifdef CONFIG_ESP32CORE_XTAL_26MHz
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

#ifdef CONFIG_ESP32CORE_RUN_IRAM
#  define BOARD_CLOCK_FREQUENCY (2 * BOARD_XTAL_FREQUENCY)
#else
#ifdef CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ
#  define BOARD_CLOCK_FREQUENCY (CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ * 1000000)
#else
#  define BOARD_CLOCK_FREQUENCY 80000000
#endif
#endif

#endif /* __BOARDS_XTENSA_ESP32_ESP32_CORE_INCLUDE_BOARD_H */
