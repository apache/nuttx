/****************************************************************************
 * boards/xtensa/esp32/ttgo_lora_esp32/include/board.h
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

#ifndef __BOARDS_XTENSA_ESP32_TTGO_LORA_ESP32_INCLUDE_BOARD_H
#define __BOARDS_XTENSA_ESP32_TTGO_LORA_ESP32_INCLUDE_BOARD_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The TTGO-LoRa-SX1276-ESP32 is fitted with a 26MHz crystal */

#ifdef CONFIG_ESP32_XTAL_26MHz
#  define BOARD_XTAL_FREQUENCY  26000000
#else
#  define BOARD_XTAL_FREQUENCY  40000000
#endif

#ifdef CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ
#  define BOARD_CLOCK_FREQUENCY (CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ * 1000000)
#else
#  define BOARD_CLOCK_FREQUENCY 80000000
#endif

/* LED definitions **********************************************************/

/* Define how many LEDs this board has (needed by userleds) */

#define BOARD_NLEDS       1

/* GPIO pins used by the GPIO Subsystem */

#define BOARD_NGPIOOUT    1 /* Amount of GPIO Output pins */
#define BOARD_NGPIOIN     1 /* Amount of GPIO Input without Interruption */
#define BOARD_NGPIOINT    1 /* Amount of GPIO Input w/ Interruption pins */

#endif /* __BOARDS_XTENSA_ESP32_TTGO_LORA_ESP32_INCLUDE_BOARD_H */
