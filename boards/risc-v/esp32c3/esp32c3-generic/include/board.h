/****************************************************************************
 * boards/risc-v/esp32c3/esp32c3-generic/include/board.h
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

#ifndef __BOARDS_RISCV_ESP32C3_ESP32C3_GENERIC_INCLUDE_BOARD_H
#define __BOARDS_RISCV_ESP32C3_ESP32C3_GENERIC_INCLUDE_BOARD_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPIO pins used by the GPIO Subsystem */

#define BOARD_NGPIOOUT    2 /* Amount of GPIO Output pins */
#define BOARD_NGPIOINT    1 /* Amount of GPIO Input w/ Interruption pins */

/* ESP32C3-Generic GPIOs ****************************************************/

/* BOOT Button */

#define BUTTON_BOOT  9

#endif /* __BOARDS_RISCV_ESP32C3_ESP32C3_GENERIC_INCLUDE_BOARD_H */

