/****************************************************************************
 * arch/arm/src/rm57/rm57_iomm.h
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

#ifndef __ARCH_ARM_SRC_RM57_RM57_IOMM_H
#define __ARCH_ARM_SRC_RM57_RM57_IOMM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stddef.h>

#include "hardware/rm57_pinmux.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_iomm_config
 *
 * Description:
 *   Select the function of a single pin, input signal or special-function
 *   field.  The argument is normally one of the PINMUX_*_PIN initializers
 *   from the chip-specific pinmux header, for example:
 *
 *     static const struct rm57_pinmux_s sci3rx = PINMUX_W3_SCI3RX_PIN;
 *     rm57_iomm_config(&sci3rx);
 *
 *   The kicker registers are unlocked for the duration of the write only.
 *
 * Input Parameters:
 *   pin - The PINMMR index, shift and value describing the function
 *
 ****************************************************************************/

void rm57_iomm_config(const struct rm57_pinmux_s *pin);

/****************************************************************************
 * Name: rm57_iomm_configmultiple
 *
 * Description:
 *   Apply a table of pin function selections, unlocking the kicker
 *   registers once for the whole table.
 *
 * Input Parameters:
 *   table - Array of pin function selections, may be NULL if count is zero
 *   count - Number of entries in the table
 *
 ****************************************************************************/

void rm57_iomm_configmultiple(const struct rm57_pinmux_s *table,
                              size_t count);

/****************************************************************************
 * Name: rm57_iomm_initialize
 *
 * Description:
 *   Apply the board pin multiplexing described by the board-specific
 *   BOARD_PINMUX_INITIALIZER definition in board.h.  Called early in boot,
 *   after the clocks are configured but before any peripheral that depends
 *   on a multiplexed pin is brought up.  Boards that define no pin
 *   multiplexing leave every ball in its reset function.
 *
 ****************************************************************************/

void rm57_iomm_initialize(void);

#endif /* __ARCH_ARM_SRC_RM57_RM57_IOMM_H */
