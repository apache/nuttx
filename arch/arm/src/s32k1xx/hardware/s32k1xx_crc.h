/****************************************************************************
 * arch/arm/src/s32k1xx/hardware/s32k1xx_crc.h
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

#ifndef __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_CRC_H
#define __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_CRC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <hardware/s32k1xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CRC Register Offsets *****************************************************/

#define S32K1XX_CRC_DATA_OFFSET     0x0000  /* CRC Data register */
#define S32K1XX_CRC_GPOLY_OFFSET    0x0004  /* CRC Polynomial register */
#define S32K1XX_CRC_CTRL_OFFSET     0x0008  /* CRC Control register */

/* CRC Register Addresses ***************************************************/

#define S32K1XX_CRC_DATA            (S32K1XX_CRC_BASE + S32K1XX_CRC_DATA_OFFSET)
#define S32K1XX_CRC_GPOLY           (S32K1XX_CRC_BASE + S32K1XX_CRC_GPOLY_OFFSET)
#define S32K1XX_CRC_CTRL            (S32K1XX_CRC_BASE + S32K1XX_CRC_CTRL_OFFSET)

/* CRC Register Bitfield Definitions ****************************************/

/* CRC Data register */

#define CRC_DATA_LL_SHIFT           (0)     /* Bits 0-7: CRC Low Lower Byte */
#define CRC_DATA_LL_MASK            (0xff << CRC_DATA_LL_SHIFT)
#  define CRC_DATA_LL(n)            ((uint32_t)(n) << CRC_DATA_LL_SHIFT)
#define CRC_DATA_LU_SHIFT           (8)     /* Bits 8-15: CRC Low Upper Byte */
#define CRC_DATA_LU_MASK            (0xff << CRC_DATA_LU_SHIFT)
#  define CRC_DATA_LU(n)            ((uint32_t)(n) << CRC_DATA_LU_SHIFT)
#define CRC_DATA_HL_SHIFT           (16)    /* Bits 16-23: CRC High Lower Byte */
#define CRC_DATA_HL_MASK            (0xff << )CRC_DATA_HL_SHIFT
#  define CRC_DATA_HL(n)            ((uint32_t)(n) << )CRC_DATA_HL_SHIFT
#define CRC_DATA_HU_SHIFT           (24)    /* Bits 24-31: CRC High Upper Byte */
#define CRC_DATA_HU_MASK            (0xff << CRC_DATA_HU_SHIFT)
#  define CRC_DATA_HU(n)            ((uint32_t)(n) << CRC_DATA_HU_SHIFT)

/* CRC Polynomial register */

#define CRC_GPOLY_LOW_SHIFT         (0)     /* Bits 0-15: Low Polynominal Half-word */
#define CRC_GPOLY_LOW_MASK          (0xffff << CRC_GPOLY_LOW_SHIFT)
#  define CRC_GPOLY_LOW(n)          ((uint32_t)(n) << CRC_GPOLY_LOW_SHIFT)
#define CRC_GPOLY_HIGH_SHIFT        (16)    /* Bits 16-31: High Polynominal Half-word */
#define CRC_GPOLY_HIGH_MASK         (0xffff << CRC_GPOLY_HIGH_SHIFT)
#  define CRC_GPOLY_HIGH(n)         ((uint32_t)(n) << CRC_GPOLY_HIGH_SHIFT)

/* CRC Control register */

#define CRC_CTRL_TCRC               (1 << 24) /* Bit 24: Width of CRC protocol */
#  define CRC_CTRL_TCRC_16BIT       (0)       /*         16-bit CRC protocol */
#  define CRC_CTRL_TCRC_16BIT       (1 << 24) /*         32-bit CRC protocol */
#define CRC_CTRL_WAS                (1 << 25) /* Bit 25: Write CRC Data Register As Seed */
#define CRC_CTRL_FXOR               (1 << 26) /* Bit 26: Complement Read Of CRC Data Register */
#define CRC_CTRL_TOTR_SHIFT         (28)      /* Bits 28-29:  Type Of Transpose For Read */
#define CRC_CTRL_TOTR_MASK          (3 << CRC_CTRL_TOTR_SHIFT)
#  define CRC_CTRL_TOTR_NONE        (0 << CRC_CTRL_TOTR_SHIFT) /* No transposition */
#  define CRC_CTRL_TOTR_BITS        (1 << CRC_CTRL_TOTR_SHIFT) /* Bits in bytes are transposed */
#  define CRC_CTRL_TOTR_BOTH        (2 << CRC_CTRL_TOTR_SHIFT) /* Both bits in bytes and bytes transposed */
#  define CRC_CTRL_TOTR_BYTES       (3 << CRC_CTRL_TOTR_SHIFT) /* Bytes are transposed */

#define CRC_CTRL_TOT_SHIFT          (30)      /* Bits 30-31: Type Of Transpose For Writes */
#define CRC_CTRL_TOT_MASK           (3 << CRC_CTRL_TOT_SHIFT)
#  define CRC_CTRL_TOT_NONE         (0 << CRC_CTRL_TOT_SHIFT)  /* No transposition */
#  define CRC_CTRL_TOT_BITS         (1 << CRC_CTRL_TOT_SHIFT)  /* Bits in bytes are transposed */
#  define CRC_CTRL_TOT_BOTH         (2 << CRC_CTRL_TOT_SHIFT)  /* Both bits in bytes and bytes transposed */
#  define CRC_CTRL_TOT_BYTES        (3 << CRC_CTRL_TOT_SHIFT)  /* Bytes are transposed */

#endif /* __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_CRC_H */
