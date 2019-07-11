/****************************************************************************
 * arch/arm/src/am335x/am335x_pinmux.h
 *
 *   Copyright (C) 2018 Petro Karashchenko. All rights reserved.
 *   Author: Petro Karashchenko <petro.karashchenko@gmail.com>
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

#ifndef __ARCH_ARM_SRC_AM335X_AM335X_PINMUX_H
#define __ARCH_ARM_SRC_AM335X_AM335X_PINMUX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/am335x_scm.h"
#include "hardware/am335x_pinmux.h"

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

#include <stdint.h>

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
