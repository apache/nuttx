/****************************************************************************
 * boards/risc-v/esp32c2/esp8684-devkitm/include/board.h
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

#ifndef __BOARDS_RISCV_ESP32C2_ESP8684_DEVKITM_INCLUDE_BOARD_H
#define __BOARDS_RISCV_ESP32C2_ESP8684_DEVKITM_INCLUDE_BOARD_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPIO pins used by the GPIO Subsystem */

#define BOARD_NGPIOOUT    3 /* Amount of GPIO Output pins (RGB LEDs) */
#define BOARD_NGPIOINT    1 /* Amount of GPIO Input w/ Interruption pins */

/* ESP8684-DevKitM GPIOs ****************************************************/

/* BOOT Button */

#define BUTTON_BOOT       9

/* On-board RGB LEDs */

#define LED_RED           0
#define LED_GREEN         1
#define LED_BLUE          8

#endif /* __BOARDS_RISCV_ESP32C2_ESP8684_DEVKITM_INCLUDE_BOARD_H */
