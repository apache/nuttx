/****************************************************************************
 * arch/arm/src/imxrt/imxrt_iomuxc_ver2.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_IOMUXC_VER2_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_IOMUXC_VER2_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "hardware/imxrt_iomuxc_ver2.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 16-bit Encoding:
 *
 *   .... RRRR ODDD LSST
 */

/* Output Pull Up/Down:
 *
 *   .... RRRR .... ....
 */

#define IOMUX_PULL_SHIFT                  (8)       /* Bits 8-11:  Pull up/down selection */
#define IOMUX_PULL_MASK                   (0x0f << IOMUX_PULL_SHIFT)
#  define IOMUX_PULL_NONE                 (0x00 << IOMUX_PULL_SHIFT) /* Pull/keeper disabled */
#  define IOMUX_PULL_KEEP                 (0x01 << IOMUX_PULL_SHIFT) /* Output determined by keeper */
#  define IOMUX_PULL_UP                   (0x02 << IOMUX_PULL_SHIFT) /* Pull up */
#  define IOMUX_PULL_DOWN                 (0x03 << IOMUX_PULL_SHIFT) /* Pull down */

/* Open Drain Output:
 *
 *   .... .... O... ....
 */

#define IOMUX_CMOS_OUTPUT                 (0 << 7)  /* Bit 7:      0 = CMOS output */
#define IOMUX_OPENDRAIN                   (1 << 7)  /* Bit 7:      1 = Enable open-drain output */

/* Output Drive Strength:
 *
 *   .... .... .DDD ....
 */

#define IOMUX_DRIVE_SHIFT                 (4)       /* Bits 4-6:   Output Drive Strength */
#define IOMUX_DRIVE_MASK                  (0x07 << IOMUX_DRIVE_SHIFT)
#  define IOMUX_DRIVE_NORMALSTRENGTH      (0x00 << IOMUX_DRIVE_SHIFT) /* Normal drive strength */
#  define IOMUX_DRIVE_HIGHSTRENGTH        (0x01 << IOMUX_DRIVE_SHIFT) /* High drive strength */

/* Output Slew Rate:
 *
 *   .... .... .... L...
 */

#define IOMUX_SLEW_SLOW                   (0 << 3)  /* Bit 3:      0 = Slow Slew Rate */
#define IOMUX_SLEW_FAST                   (1 << 3)  /* Bit 3:      1 = Fast Slew Rate */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The smallest integer type that can hold the IOMUX encoding */

typedef uint16_t iomux_pinset_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_padmux_map
 *
 * Description:
 *   This function map a Pad Mux register index to the corresponding Pad
 *   Control register index.
 *
 ****************************************************************************/

unsigned int imxrt_padmux_map(unsigned int padmux);

/****************************************************************************
 * Name: imxrt_iomux_configure
 *
 * Description:
 *   This function writes the encoded pad configuration to the Pad Control
 *   register.
 *
 ****************************************************************************/

int imxrt_iomux_configure(uintptr_t padctl, iomux_pinset_t ioset);

#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT_IOMUXC_VER2_H */
