/****************************************************************************
 * arch/arm/src/rm57/hardware/rm57_pinmux.h
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

#ifndef __ARCH_ARM_SRC_RM57_HARDWARE_RM57_PINMUX_H
#define __ARCH_ARM_SRC_RM57_HARDWARE_RM57_PINMUX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#if defined(CONFIG_ARCH_CHIP_RM57L843)
#  include "hardware/rm57l843_pinmux.h"
#else
#  error "Unrecognized RM57 chip"
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The chip-specific pinmux header above defines one initializer of this type
 * per selectable pin function, named PINMUX_<ball>_<function>_PIN.  A board
 * lists the ones it needs in BOARD_PINMUX_INITIALIZER; see
 * arch/arm/src/rm57/rm57_iomm.h.
 */

struct rm57_pinmux_s
{
  uint8_t mmrndx;  /* Index of the PINMMR register, 0-37, 80-99 or 160-179 */
  uint8_t shift;   /* Shift value to isolate the field in that register */
  uint8_t value;   /* The new value for the field */
};

#endif /* __ARCH_ARM_SRC_RM57_HARDWARE_RM57_PINMUX_H */
