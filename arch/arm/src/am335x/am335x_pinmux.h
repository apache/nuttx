/****************************************************************************
 * arch/arm/src/am335x/am335x_pinmux.h
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

#ifndef __ARCH_ARM_SRC_AM335X_AM335X_PINMUX_H
#define __ARCH_ARM_SRC_AM335X_AM335X_PINMUX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/am335x_scm.h"
#include "hardware/am335x_pinmux.h"

#ifndef __ASSEMBLY__

#include <stdint.h>

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 8-bit Encoding:
 *
 *   .SRT PMMM
 */

/* Select Between Faster or Slower Slew Rate:
 *
 *   .S.. ....
 */

#define PINMUX_SLEW_FAST       (0)       /* Bit 6: 0=Fast Slew Rate */
#define PINMUX_SLEW_SLOW       (1 << 6)  /* Bit 6: 1=Slow Slew Rate */

/* Input Enable Value for the Pad. Set to 1 for Input or Output:
 *
 *   ..R. ....
 */

#define PINMUX_RX_DISABLE      (0)       /* Bit 5: 0=Receiver Disabled */
#define PINMUX_RX_ENABLE       (1 << 5)  /* Bit 5: 1=Receiver Enabled */

/* Pad Pull Up/Down Type Selection:
 *
 *   ...T ....
 */

#define PINMUX_PULL_TYPE_DOWN  (0)       /* Bit 4: 0=Pull Down Selected */
#define PINMUX_PULL_TYPE_UP    (1 << 4)  /* Bit 4: 1=Pull Up Selected */

/* Pad Pull Up/Down Enable:
 *
 *   .... P...
 */

#define PINMUX_PULL_UP_ENABLE  (0)       /* Bit 3: 0=Pull Up/Down Enabled */
#define PINMUX_PULL_UP_DISABLE (1 << 3)  /* Bit 3: 1=Pull Up/Down Disabled */

/* Pin Multiplexing Mode:
 *
 *   .... .MMM
 */

#define PINMUX_MODE_SHIFT      (0)      /* Bits 0-2: Pin Multiplexing Mode */
#define PINMUX_MODE_MASK       (7 << PINMUX_MODE_SHIFT)
#  define PINMUX_MODE0         (0 << PINMUX_MODE_SHIFT)  /* Mode 0 = Primary Mode */
#  define PINMUX_MODE1         (1 << PINMUX_MODE_SHIFT)  /* Mode 1 */
#  define PINMUX_MODE2         (2 << PINMUX_MODE_SHIFT)  /* Mode 2 */
#  define PINMUX_MODE3         (3 << PINMUX_MODE_SHIFT)  /* Mode 3 */
#  define PINMUX_MODE4         (4 << PINMUX_MODE_SHIFT)  /* Mode 4 */
#  define PINMUX_MODE5         (5 << PINMUX_MODE_SHIFT)  /* Mode 5 */
#  define PINMUX_MODE6         (6 << PINMUX_MODE_SHIFT)  /* Mode 6 */
#  define PINMUX_MODE7         (7 << PINMUX_MODE_SHIFT)  /* Mode 7 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* The smallest integer type that can hold the PINMUX encoding */

typedef uint8_t pinmux_pinset_t;

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: am335x_pinmux_configure
 *
 * Description:
 *   This function writes the encoded pad configuration to the Pad Control
 *   register.
 *
 ****************************************************************************/

int am335x_pinmux_configure(uintptr_t padctl, pinmux_pinset_t muxset);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_AM335X_AM335X_PINMUX_H */
