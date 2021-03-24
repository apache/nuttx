/****************************************************************************
 * arch/arm/src/kinetis/hardware/kinetis_crc.h
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

#ifndef __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_CRC_H
#define __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_CRC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#if defined(KINETIS_NCRC) && KINETIS_NCRC > 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define KINETIS_CRC_DATA_OFFSET   0x0000 /* CRC Data Register */
#define KINETIS_CRC_GPOLY_OFFSET  0x0004 /* CRC Polynomial Register */
#define KINETIS_CRC_CTRL_OFFSET   0x0008 /* CRC Control Register */

/* Register Addresses *******************************************************/

#define KINETIS_CRC_DATA          (KINETIS_CRC_BASE+KINETIS_CRC_DATA_OFFSET)
#define KINETIS_CRC_GPOLY         (KINETIS_CRC_BASE+KINETIS_CRC_GPOLY_OFFSET)
#define KINETIS_CRC_CTRL          (KINETIS_CRC_BASE+KINETIS_CRC_CTRL_OFFSET)

/* Register Bit Definitions *************************************************/

/* CRC Data Register (32-bit) */

#define CRC_DATA_LL_SHIFT         (0)       /* Bits 0-7: CRC Low Lower Byte */
#define CRC_DATA_LL_MASK          (0xff << CRC_DATA_LL_SHIFT)
#define CRC_DATA_LU_SHIFT         (8)       /* Bits 8-15: CRC Low Upper Byte */
#define CRC_DATA_LU_MASK          (0xff << CRC_DATA_LU_SHIFT)
#define CRC_DATA_HL_SHIFT         (16)      /* Bits 16-23: CRC High Lower Byte */
#define CRC_DATA_HL_MASK          (0xff << CRC_DATA_HL_SHIFT)
#define CRC_DATA_HU_SHIFT         (24)      /* Bits 24-31: CRC High Upper Byte */
#define CRC_DATA_HU_MASK          (0xff << CRC_DATA_HU_SHIFT)

/* CRC Polynomial Register */

#define CRC_GPOLY_LOW_SHIFT       (0)       /* Bits 0-15: Low polynominal half-word */
#define CRC_GPOLY_LOW_MASK        (0xffff << CRC_GPOLY_LOW_SHIFT)
#define CRC_GPOLY_HIGH_SHIFT      (16)      /* Bits 16-31: High polynominal half-word */
#define CRC_GPOLY_HIGH_MASK       (0xffff << CRC_GPOLY_HIGH_SHIFT)

/* CRC Control Register */

                                            /* Bits 0-23: Reserved */
#define CRC_CTRL_TCRC             (1 << 24) /* Bit 24: Width of CRC protocol */
#define CRC_CTRL_WAS              (1 << 25) /* Bit 25: Write CRC data register as seed */
#define CRC_CTRL_FXOR             (1 << 26) /* Bit 26: Complement Read of CRC data register */
                                            /* Bit 27: Reserved */
#define CRC_CTRL_TOTR_SHIFT       (28)      /* Bits 28-29: Type of Transpose for Read */
#define CRC_CTRL_TOTR_MASK        (3 << CRC_CTRL_TOTR_SHIFT)
#  define CRC_CTRL_TOTR_NONE      (0 << CRC_CTRL_TOTR_SHIFT) /* No transposition */
#  define CRC_CTRL_TOTR_BITS      (1 << CRC_CTRL_TOTR_SHIFT) /* Bits transposed; bytes are not */
#  define CRC_CTRL_TOTR_BOTH      (2 << CRC_CTRL_TOTR_SHIFT) /* Both bits bytes and bytes transposed */
#  define CRC_CTRL_TOTR_BYTES     (3 << CRC_CTRL_TOTR_SHIFT) /* Bytes transposed; bits in byte are not */

#define CRC_CTRL_TOT_SHIFT        (30)      /* Bits 30-31: Type of Transpose for Writes */
#define CRC_CTRL_TOT_MASK         (3 << CRC_CTRL_TOT_SHIFT)
#  define CRC_CTRL_TOT_NONE       (0 << CRC_CTRL_TOT_SHIFT) /* No transposition */
#  define CRC_CTRL_TOT_BITS       (1 << CRC_CTRL_TOT_SHIFT) /* Bits transposed; bytes are not */
#  define CRC_CTRL_TOT_BOTH       (2 << CRC_CTRL_TOT_SHIFT) /* Both bits bytes and bytes transposed */
#  define CRC_CTRL_TOT_BYTES      (3 << CRC_CTRL_TOT_SHIFT) /* Bytes transposed; bits in byte are not */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* KINETIS_NCRC && KINETIS_NCRC > 0 */
#endif /* __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_CRC_H */
