/****************************************************************************
 * arch/arm/src/tiva/hardware/cc13x2_cc26x2/cc13x2_cc26x2_gpio.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *
 * Technical content derives from a TI header file that has a compatible
 * BSD license:
 *
 *   Copyright (c) 2015-2017, Texas Instruments Incorporated
 *   All rights reserved.
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

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_CC13X2_CC26X2_GPIO_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_CC13X2_CC26X2_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/tiva_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPIO Register Offsets ****************************************************/

#define TIVA_GPIO_DOUT_PIN_OFFSET(n)      ((n) & ~3)
#  define TIVA_GPIO_DOUT_PIN3_0_OFFSET    0x0000  /* Data Out 0 to 3 */
#  define TIVA_GPIO_DOUT_PIN7_4_OFFSET    0x0004  /* Data Out 4 to 7 */
#  define TIVA_GPIO_DOUT_PIN11_8_OFFSET   0x0008  /* Data Out 8 to 11 */
#  define TIVA_GPIO_DOUT_PIN15_12_OFFSET  0x000c  /* Data Out 12 to 15 */
#  define TIVA_GPIO_DOUT_PIN19_16_OFFSET  0x0010  /* Data Out 16 to 19 */
#  define TIVA_GPIO_DOUT_PIN23_20_OFFSET  0x0014  /* Data Out 20 to 23 */
#  define TIVA_GPIO_DOUT_PIN27_24_OFFSET  0x0018  /* Data Out 24 to 27 */
#  define TIVA_GPIO_DOUT_PIN31_28_OFFSET  0x001c  /* Data Out 28 to 31 */
#define TIVA_GPIO_DOUT_OFFSET             0x0080  /* Data Output for DIO 0 to 31 */
#define TIVA_GPIO_DOUTSET_OFFSET          0x0090  /* Data Out Set */
#define TIVA_GPIO_DOUTCLR_OFFSET          0x00a0  /* Data Out Clear */
#define TIVA_GPIO_DOUTTGL_OFFSET          0x00b0  /* Data Out Toggle */
#define TIVA_GPIO_DIN_OFFSET              0x00c0  /* Data Input from DIO 0 to 31 */
#define TIVA_GPIO_DOE_OFFSET              0x00d0  /* Data Output Enable for DIO 0 to 31 */
#define TIVA_GPIO_EVFLAGS_OFFSET          0x00e0  /* Event Register for DIO 0 to 31 */

/* GPIO Register Addresses **************************************************/

#define TIVA_GPIO_DOUT_PIN_BASE(n)        (TIVA_GPIO_BASE + TIVA_GPIO_DOUT_PIN_OFFSET(n))
#  define TIVA_GPIO_DOUT_PIN3_0           (TIVA_GPIO_BASE + TIVA_GPIO_DOUT_PIN3_0_OFFSET)
#  define TIVA_GPIO_DOUT_PIN7_4           (TIVA_GPIO_BASE + TIVA_GPIO_DOUT_PIN7_4_OFFSET)
#  define TIVA_GPIO_DOUT_PIN11_8          (TIVA_GPIO_BASE + TIVA_GPIO_DOUT_PIN11_8_OFFSET)
#  define TIVA_GPIO_DOUT_PIN15_12         (TIVA_GPIO_BASE + TIVA_GPIO_DOUT_PIN15_12_OFFSET)
#  define TIVA_GPIO_DOUT_PIN19_16         (TIVA_GPIO_BASE + TIVA_GPIO_DOUT_PIN19_16_OFFSET)
#  define TIVA_GPIO_DOUT_PIN23_20         (TIVA_GPIO_BASE + TIVA_GPIO_DOUT_PIN23_20_OFFSET)
#  define TIVA_GPIO_DOUT_PIN27_24         (TIVA_GPIO_BASE + TIVA_GPIO_DOUT_PIN27_24_OFFSET)
#  define TIVA_GPIO_DOUT_PIN31_28         (TIVA_GPIO_BASE + TIVA_GPIO_DOUT_PIN31_28_OFFSET)
#define TIVA_GPIO_DOUT                    (TIVA_GPIO_BASE + TIVA_GPIO_DOUT_OFFSET)
#define TIVA_GPIO_DOUTSET                 (TIVA_GPIO_BASE + TIVA_GPIO_DOUTSET_OFFSET)
#define TIVA_GPIO_DOUTCLR                 (TIVA_GPIO_BASE + TIVA_GPIO_DOUTCLR_OFFSET)
#define TIVA_GPIO_DOUTTGL                 (TIVA_GPIO_BASE + TIVA_GPIO_DOUTTGL_OFFSET)
#define TIVA_GPIO_DIN                     (TIVA_GPIO_BASE + TIVA_GPIO_DIN_OFFSET)
#define TIVA_GPIO_DOE                     (TIVA_GPIO_BASE + TIVA_GPIO_DOE_OFFSET)
#define TIVA_GPIO_EVFLAGS                 (TIVA_GPIO_BASE + TIVA_GPIO_EVFLAGS_OFFSET)

/* GPIO Register Bitfield Definitions ***************************************/

/* Data Out n to n + 3 */

#define GPIO_DOUT_SHIFT(n)                (((n) & 3) << 8)
#define GPIO_DOUT_VALUE(n)                (1 << GPIO_DOUT_SHIFT(n))

/* Data Output for DIO 0 to 31 */

#define GPIO_DOUT(n)                      (1 << (n))

/* Data Out Set */

#define GPIO_DOUTSET(n)                   (1 << (n))

/* Data Out Clear */

#define GPIO_DOUTCLR(n)                   (1 << (n))

/* Data Out Toggle */

#define GPIO_DOUTTGL(n)                   (1 << (n))

/* Data Input from DIO 0 to 31 */

#define GPIO_DIN(n)                       (1 << (n))

/* Data Output Enable for DIO 0 to 31 */

#define GPIO_DOE(n)                       (1 << (n))

/* Event Register for DIO 0 to 31 */

#define GPIO_EVFLAGS(n)                   (1 << (n))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_CC13X2_CC26X2_GPIO_H */
