/****************************************************************************
 * arch/arm/src/tms570/hardware/tms570_pinmux.h
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

#ifndef __ARCH_ARM_SRC_TMS570_HARDWARE_TMS570_PINMUX_H
#define __ARCH_ARM_SRC_TMS570_HARDWARE_TMS570_PINMUX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_ARCH_CHIP_TMS570LS0232PZ)
#  error No pin multiplexing for the TMS570LS0232PZ
#elif defined(CONFIG_ARCH_CHIP_TMS570LS0332PZ)
#  include "hardware/tms570ls04x03x_pinmux.h"
#elif defined(CONFIG_ARCH_CHIP_TMS570LS0432PZ)
#  include "hardware/tms570ls04x03x_pinmux.h"
#elif defined(CONFIG_ARCH_CHIP_TMS570LS0714PZ)
#  error No pin multiplexing for the TMS570LS0714PZ
#elif defined(CONFIG_ARCH_CHIP_TMS570LS0714PGE)
#  error No pin multiplexing for the TMS570LS0714PGE
#elif defined(CONFIG_ARCH_CHIP_TMS570LS0714ZWT)
#  error No pin multiplexing for the TMS570LS0714ZWT
#elif defined(CONFIG_ARCH_CHIP_TMS570LS1227ZWT)
#  error No pin multiplexing for the TMS570LS1227ZWT
#elif defined(CONFIG_ARCH_CHIP_TMS570LS3137ZWT)
#  include "hardware/tms570ls04x03x_pinmux.h"
#else
#  error "Unrecognized Hercules chip"
#endif

/****************************************************************************
 * Pulbic Type Definitions
 ****************************************************************************/

/* Each chip-specific pinmux header file defines initializers for a type
 * like:
 */

struct tms570_pinmux_s
{
  uint8_t mmrndx;  /* Index to the PINMMR register, 0-30 */
  uint8_t shift;   /* Shift value to isolate the pin field in the PINMMR register */
  uint8_t value;   /* The new value for the pin field in the PINMMR register */
};

#endif /* __ARCH_ARM_SRC_TMS570_HARDWARE_TMS570_PINMUX_H */
