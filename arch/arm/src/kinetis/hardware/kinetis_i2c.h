/****************************************************************************
 * arch/arm/src/kinetis/hardware/kinetis_i2c.h
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

#ifndef __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_I2C_H
#define __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define KINETIS_I2C_A1_OFFSET    0x0000 /* I2C Address Register 1 */
#define KINETIS_I2C_F_OFFSET     0x0001 /* I2C Frequency Divider register */
#define KINETIS_I2C_C1_OFFSET    0x0002 /* I2C Control Register 1 */
#define KINETIS_I2C_S_OFFSET     0x0003 /* I2C Status Register */
#define KINETIS_I2C_D_OFFSET     0x0004 /* I2C Data I/O register */
#define KINETIS_I2C_C2_OFFSET    0x0005 /* I2C Control Register 2 */
#define KINETIS_I2C_FLT_OFFSET   0x0006 /* I2C Programmable Input Glitch Filter register */
#define KINETIS_I2C_RA_OFFSET    0x0007 /* I2C Range Address register */
#define KINETIS_I2C_SMB_OFFSET   0x0008 /* I2C SMBus Control and Status register */
#define KINETIS_I2C_A2_OFFSET    0x0009 /* I2C Address Register 2 */
#define KINETIS_I2C_SLTH_OFFSET  0x000a /* I2C SCL Low Timeout Register High */
#define KINETIS_I2C_SLTL_OFFSET  0x000b /* I2C SCL Low Timeout Register Low */

/* Register Addresses *******************************************************/

#define KINETIS_I2C0_A1          (KINETIS_I2C0_BASE+KINETIS_I2C_A1_OFFSET)
#define KINETIS_I2C0_F           (KINETIS_I2C0_BASE+KINETIS_I2C_F_OFFSET)
#define KINETIS_I2C0_C1          (KINETIS_I2C0_BASE+KINETIS_I2C_C1_OFFSET)
#define KINETIS_I2C0_S           (KINETIS_I2C0_BASE+KINETIS_I2C_S_OFFSET)
#define KINETIS_I2C0_D           (KINETIS_I2C0_BASE+KINETIS_I2C_D_OFFSET)
#define KINETIS_I2C0_C2          (KINETIS_I2C0_BASE+KINETIS_I2C_C2_OFFSET)
#define KINETIS_I2C0_FLT         (KINETIS_I2C0_BASE+KINETIS_I2C_FLT_OFFSET)
#define KINETIS_I2C0_RA          (KINETIS_I2C0_BASE+KINETIS_I2C_RA_OFFSET)
#define KINETIS_I2C0_SMB         (KINETIS_I2C0_BASE+KINETIS_I2C_SMB_OFFSET)
#define KINETIS_I2C0_A2          (KINETIS_I2C0_BASE+KINETIS_I2C_A2_OFFSET)
#define KINETIS_I2C0_SLTH        (KINETIS_I2C0_BASE+KINETIS_I2C_SLTH_OFFSET)
#define KINETIS_I2C0_SLTL        (KINETIS_I2C0_BASE+KINETIS_I2C_SLTL_OFFSET)

#ifdef CONFIG_KINETIS_HAVE_I2C1
#  define KINETIS_I2C1_A1        (KINETIS_I2C1_BASE+KINETIS_I2C_A1_OFFSET)
#  define KINETIS_I2C1_F         (KINETIS_I2C1_BASE+KINETIS_I2C_F_OFFSET)
#  define KINETIS_I2C1_C1        (KINETIS_I2C1_BASE+KINETIS_I2C_C1_OFFSET)
#  define KINETIS_I2C1_S         (KINETIS_I2C1_BASE+KINETIS_I2C_S_OFFSET)
#  define KINETIS_I2C1_D         (KINETIS_I2C1_BASE+KINETIS_I2C_D_OFFSET)
#  define KINETIS_I2C1_C2        (KINETIS_I2C1_BASE+KINETIS_I2C_C2_OFFSET)
#  define KINETIS_I2C1_FLT       (KINETIS_I2C1_BASE+KINETIS_I2C_FLT_OFFSET)
#  define KINETIS_I2C1_RA        (KINETIS_I2C1_BASE+KINETIS_I2C_RA_OFFSET)
#  define KINETIS_I2C1_SMB       (KINETIS_I2C1_BASE+KINETIS_I2C_SMB_OFFSET)
#  define KINETIS_I2C1_A2        (KINETIS_I2C1_BASE+KINETIS_I2C_A2_OFFSET)
#  define KINETIS_I2C1_SLTH      (KINETIS_I2C1_BASE+KINETIS_I2C_SLTH_OFFSET)
#  define KINETIS_I2C1_SLTL      (KINETIS_I2C1_BASE+KINETIS_I2C_SLTL_OFFSET)
#endif

#ifdef CONFIG_KINETIS_HAVE_I2C2
#  define KINETIS_I2C2_A1        (KINETIS_I2C2_BASE+KINETIS_I2C_A1_OFFSET)
#  define KINETIS_I2C2_F         (KINETIS_I2C2_BASE+KINETIS_I2C_F_OFFSET)
#  define KINETIS_I2C2_C1        (KINETIS_I2C2_BASE+KINETIS_I2C_C1_OFFSET)
#  define KINETIS_I2C2_S         (KINETIS_I2C2_BASE+KINETIS_I2C_S_OFFSET)
#  define KINETIS_I2C2_D         (KINETIS_I2C2_BASE+KINETIS_I2C_D_OFFSET)
#  define KINETIS_I2C2_C2        (KINETIS_I2C2_BASE+KINETIS_I2C_C2_OFFSET)
#  define KINETIS_I2C2_FLT       (KINETIS_I2C2_BASE+KINETIS_I2C_FLT_OFFSET)
#  define KINETIS_I2C2_RA        (KINETIS_I2C2_BASE+KINETIS_I2C_RA_OFFSET)
#  define KINETIS_I2C2_SMB       (KINETIS_I2C2_BASE+KINETIS_I2C_SMB_OFFSET)
#  define KINETIS_I2C2_A2        (KINETIS_I2C2_BASE+KINETIS_I2C_A2_OFFSET)
#  define KINETIS_I2C2_SLTH      (KINETIS_I2C2_BASE+KINETIS_I2C_SLTH_OFFSET)
#  define KINETIS_I2C2_SLTL      (KINETIS_I2C2_BASE+KINETIS_I2C_SLTL_OFFSET)
#endif

/* Register Bit Definitions *************************************************/

/* I2C Address Register 1 (8-bit) */

                                           /* Bit 0: Reserved */
#define I2C_A1_SHIFT             (1)       /* Bits 1-7: Address */
#define I2C_A1_MASK              (0x7f << I2C_A1_SHIFT)

/* I2C Frequency Divider register (8-bit) */

#define I2C_F_ICR_SHIFT          (0)       /* Bits 0-5: Clock rate */
#define I2C_F_ICR_MASK           (0x3f << I2C_F_ICR_SHIFT)
#  define I2C_F_ICR(n)           ((uint8_t)(n) << I2C_F_ICR_SHIFT)
#define I2C_F_MULT_SHIFT         (6)       /* Bits 6-7: Multiplier factor */
#define I2C_F_MULT_MASK          (3 << I2C_F_MULT_SHIFT)
#  define I2C_F_MULT_1           (0 << I2C_F_MULT_SHIFT)
#  define I2C_F_MULT_2           (1 << I2C_F_MULT_SHIFT)
#  define I2C_F_MULT_4           (2 << I2C_F_MULT_SHIFT)

/* From Table 51-54. I2C divider and hold values.
 *  Duplicate divider values differ in hold times.
 *  Refer to the Table 51-54. in the K64 Sub-Family Reference Manual.
 */

#define I2C_F_DIV20              ((uint8_t)0x00)
#define I2C_F_DIV22              ((uint8_t)0x01)
#define I2C_F_DIV24              ((uint8_t)0x02)
#define I2C_F_DIV26              ((uint8_t)0x03)
#define I2C_F_DIV28              ((uint8_t)0x04)
#define I2C_F_DIV30              ((uint8_t)0x05)
#define I2C_F_DIV34              ((uint8_t)0x06)
#define I2C_F_DIV36              ((uint8_t)0x0a)
#define I2C_F_DIV40_1            ((uint8_t)0x07)
#define I2C_F_DIV41              ((uint8_t)0x08)

#define I2C_F_DIV32              ((uint8_t)0x09)
#define I2C_F_DIV36              ((uint8_t)0x0a)
#define I2C_F_DIV40_2            ((uint8_t)0x0b)
#define I2C_F_DIV44              ((uint8_t)0x0c)
#define I2C_F_DIV48_1            ((uint8_t)0x0d)
#define I2C_F_DIV56_1            ((uint8_t)0x0e)
#define I2C_F_DIV68              ((uint8_t)0x0f)

#define I2C_F_DIV48_2            ((uint8_t)0x10)
#define I2C_F_DIV56_2            ((uint8_t)0x11)
#define I2C_F_DIV64              ((uint8_t)0x12)
#define I2C_F_DIV72              ((uint8_t)0x13)
#define I2C_F_DIV80_1            ((uint8_t)0x14)
#define I2C_F_DIV88              ((uint8_t)0x15)
#define I2C_F_DIV104             ((uint8_t)0x16)
#define I2C_F_DIV128_1           ((uint8_t)0x17)

#define I2C_F_DIV80_2            ((uint8_t)0x18)
#define I2C_F_DIV96              ((uint8_t)0x19)
#define I2C_F_DIV112             ((uint8_t)0x1a)
#define I2C_F_DIV128_2           ((uint8_t)0x1b)
#define I2C_F_DIV144             ((uint8_t)0x1c)
#define I2C_F_DIV160_1           ((uint8_t)0x1d)
#define I2C_F_DIV192_1           ((uint8_t)0x1e)
#define I2C_F_DIV240             ((uint8_t)0x1f)

#define I2C_F_DIV160_2           ((uint8_t)0x20)
#define I2C_F_DIV192_2           ((uint8_t)0x1e)
#define I2C_F_DIV224             ((uint8_t)0x22)
#define I2C_F_DIV256             ((uint8_t)0x23)
#define I2C_F_DIV288             ((uint8_t)0x24)
#define I2C_F_DIV320_1           ((uint8_t)0x25)
#define I2C_F_DIV384_1           ((uint8_t)0x26)
#define I2C_F_DIV480             ((uint8_t)0x27)

#define I2C_F_DIV320_2           ((uint8_t)0x28)
#define I2C_F_DIV384_2           ((uint8_t)0x29)
#define I2C_F_DIV448             ((uint8_t)0x2a)
#define I2C_F_DIV512             ((uint8_t)0x2b)
#define I2C_F_DIV576             ((uint8_t)0x2c)
#define I2C_F_DIV640_1           ((uint8_t)0x2d)
#define I2C_F_DIV768_1           ((uint8_t)0x2e)
#define I2C_F_DIV960             ((uint8_t)0x2f)

#define I2C_F_DIV640_2           ((uint8_t)0x30)
#define I2C_F_DIV768_3           ((uint8_t)0x31)
#define I2C_F_DIV896             ((uint8_t)0x32)
#define I2C_F_DIV1024            ((uint8_t)0x33)
#define I2C_F_DIV1152            ((uint8_t)0x34)
#define I2C_F_DIV1280_1          ((uint8_t)0x35)
#define I2C_F_DIV1536_1          ((uint8_t)0x36)
#define I2C_F_DIV1920            ((uint8_t)0x37)

#define I2C_F_DIV1280_2          ((uint8_t)0x38)
#define I2C_F_DIV1536_2          ((uint8_t)0x39)
#define I2C_F_DIV1792            ((uint8_t)0x3a)
#define I2C_F_DIV2048            ((uint8_t)0x3b)
#define I2C_F_DIV2304            ((uint8_t)0x3c)
#define I2C_F_DIV2560            ((uint8_t)0x3d)
#define I2C_F_DIV3072            ((uint8_t)0x3e)
#define I2C_F_DIV3840            ((uint8_t)0x3f)

/* I2C Control Register 1 (8-bit) */

#define I2C_C1_DMAEN             (1 << 0)  /* Bit 0:  DMA enable */
#define I2C_C1_WUEN              (1 << 1)  /* Bit 1:  Wakeup enable */
#define I2C_C1_RSTA              (1 << 2)  /* Bit 2:  Repeat START */
#define I2C_C1_TXAK              (1 << 3)  /* Bit 3:  Transmit acknowledge enable */
#define I2C_C1_TX                (1 << 4)  /* Bit 4:  Transmit mode select */
#define I2C_C1_MST               (1 << 5)  /* Bit 5:  Master mode select */
#define I2C_C1_IICIE             (1 << 6)  /* Bit 6:  I2C interrupt enable */
#define I2C_C1_IICEN             (1 << 7)  /* Bit 7:  I2C enable */

/* I2C Status Register (8-bit) */

#define I2C_S_RXAK               (1 << 0)  /* Bit 0:  Receive acknowledge */
#define I2C_S_IICIF              (1 << 1)  /* Bit 1:  Interrupt flag */
#define I2C_S_SRW                (1 << 2)  /* Bit 2:  Slave read/write */
#define I2C_S_RAM                (1 << 3)  /* Bit 3:  Range address match */
#define I2C_S_ARBL               (1 << 4)  /* Bit 4:  Arbitration lost */
#define I2C_S_BUSY               (1 << 5)  /* Bit 5:  Bus busy */
#define I2C_S_IAAS               (1 << 6)  /* Bit 6:  Addressed as a slave */
#define I2C_S_TCF                (1 << 7)  /* Bit 7:  Transfer complete flag */

/* I2C Data I/O register (8-bit data register) */

/* I2C Control Register 2 (8-bit) */

#define I2C_C2_AD_SHIFT          (0)       /* Bits 0-2: Slave address */
#define I2C_C2_AD_MASK           (7 << I2C_C2_AD_SHIFT)
#  define I2C_C2_AD(n)           ((uint8_t)(n) << I2C_C2_AD_SHIFT)
#define I2C_C2_RMEN              (1 << 3)  /* Bit 3:  Range address matching enable */
#define I2C_C2_SBRC              (1 << 4)  /* Bit 4:  Slave baud rate control */
#define I2C_C2_HDRS              (1 << 5)  /* Bit 5:  High drive select */
#define I2C_C2_ADEXT             (1 << 6)  /* Bit 6:  Address extension */
#define I2C_C2_GCAEN             (1 << 7)  /* Bit 7:  General call address enable */

/* I2C Programmable Input Glitch Filter register (8-bit) */

#if defined(KINETIS_K20) || defined(KINETIS_K40) || defined(KINETIS_K60)
#  define I2C_FLT_SHIFT          (0)       /* Bits 0-4: I2C programmable filter factor */
#  define I2C_FLT_MASK           (31 << I2C_FLT_SHIFT)
#    define I2C_FLT(n)           ((uint8_t)(n) << I2C_FLT_SHIFT)
                                           /* Bits 5-7: Reserved */
#endif

#if defined(KINETIS_K64) || defined(KINETIS_K66)
#  define I2C_FLT_SHIFT          (0)       /* Bits 0-3: I2C programmable filter factor */
#  define I2C_FLT_MASK           (15 << I2C_FLT_SHIFT)
#    define I2C_FLT(n)           ((uint8_t)(n) << I2C_FLT_SHIFT)
#  define I2C_FLT_STARTF         (1 << 4)  /* I2C bus start detect flag */
#  define I2C_FLT_SSIE           (1 << 5)  /* I2C bus stop or start interrupt enable */
#  define I2C_FLT_STOPF          (1 << 6)  /* I2C bus stop detect flag */
#  define I2C_FLT_SHEN           (1 << 7)  /* Stop hold enable */
#endif

/* I2C Range Address register (8-bit) */

                                           /* Bit 0: Reserved */
#define I2C_RA_SHIFT             (1)       /* Bits 1-7: Range slave address */
#define I2C_RA_MASK              (0x7f << I2C_RA_SHIFT)

/* I2C SMBus Control and Status register (8-bit) */

#define I2C_SMB_SHTF2IE          (1 << 0)  /* Bit 0:  SHTF2 interrupt enable */
#define I2C_SMB_SHTF2            (1 << 1)  /* Bit 1:  SCL high timeout flag 2 */
#define I2C_SMB_SHTF1            (1 << 2)  /* Bit 2:  SCL high timeout flag 1 */
#define I2C_SMB_SLTF             (1 << 3)  /* Bit 3:  SCL low timeout flag */
#define I2C_SMB_TCKSEL           (1 << 4)  /* Bit 4:  Timeout counter clock select */
#define I2C_SMB_SIICAEN          (1 << 5)  /* Bit 5:  Second I2C address enable */
#define I2C_SMB_ALERTEN          (1 << 6)  /* Bit 6:  SMBus alert response address enable */
#define I2C_SMB_FACK             (1 << 7)  /* Bit 7:  Fast NACK/ACK enable */

/* I2C Address Register 2 (8-bit) */

                                           /* Bit 0: Reserved */
#define I2C_A2_SHIFT             (1)       /* Bits 1-7: SMBus address */
#define I2C_A2_MASK              (0x7f << I2C_A2_SHIFT)

/* I2C SCL Low Timeout Register High/Low
 * (16-bit data in two 8-bit registers)
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_I2C_H */
