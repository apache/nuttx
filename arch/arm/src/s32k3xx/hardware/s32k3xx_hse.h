/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_hse.h
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

/* Copyright 2022 NXP */

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_HSE_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_HSE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/s32k3xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* HSE Register Offsets *****************************************************/

#define S32K3XX_HSE_CONFIG_REG0_OFFSET      (0x1c) /* General Purpose Configuration 0 (CONFIG_REG0) */
#define S32K3XX_HSE_CONFIG_REG6_OFFSET      (0x34) /* General Purpose Configuration 6 (CONFIG_REG6) */
#define S32K3XX_HSE_CONFIG_RAMPR_OFFSET     (0x38) /* Configuration RAM Protected Region (CONFIG_RAMPR) */
#define S32K3XX_HSE_CONFIG_CFPRL_OFFSET     (0x3c) /* Configuration Code Flash Memory Active Block (CONFIG_CFPRL) */
#define S32K3XX_HSE_CONFIG_CFPRH_OFFSET     (0x40) /* Configuration Code Flash Memory Passive Block (CONFIG_CFPRH) */
#define S32K3XX_HSE_CONFIG_DFPR_OFFSET      (0x44) /* Configuration Data Flash Memory Protected Region (CONFIG_DFPR) */
#define S32K3XX_HSE_CONFIG_PE_LOCK_OFFSET   (0x50) /* Configuration Program and Erase Lock (CONFIG_PE_LOCK) */
#define S32K3XX_HSE_CONFIG_RAMPR_ALT_OFFSET (0x54) /* Configuration RAM Protected Region Alternate (CONFIG_RAMPR_ALT) */
#define S32K3XX_HSE_CONFIG_CFPRL_ALT_OFFSET (0x58) /* Configuration Code Flash Memory Active Block Alternative (CONFIG_CFPRL_ALT) */
#define S32K3XX_HSE_CONFIG_CFPRH_ALT_OFFSET (0x5c) /* Configuration Code Flash Memory Passive Block Alternative (CONFIG_CFPRH_ALT) */
#define S32K3XX_HSE_CONFIG_DFPR_ALT_OFFSET  (0x60) /* Configuration Data Flash Memory Protected Region Alternative (CONFIG_DFPR_ALT) */
#define S32K3XX_HSE_CONFIG_REG_GPR_OFFSET   (0x64) /* Configuration REG_GPR (CONFIG_REG_GPR) */

/* HSE Register Addresses ***************************************************/

#define S32K3XX_HSE_CONFIG_REG0             (S32K3XX_HSE_BASE + S32K3XX_HSE_CONFIG_REG0_OFFSET)
#define S32K3XX_HSE_CONFIG_REG6             (S32K3XX_HSE_BASE + S32K3XX_HSE_CONFIG_REG6_OFFSET)
#define S32K3XX_HSE_CONFIG_RAMPR            (S32K3XX_HSE_BASE + S32K3XX_HSE_CONFIG_RAMPR_OFFSET)
#define S32K3XX_HSE_CONFIG_CFPRL            (S32K3XX_HSE_BASE + S32K3XX_HSE_CONFIG_CFPRL_OFFSET)
#define S32K3XX_HSE_CONFIG_CFPRH            (S32K3XX_HSE_BASE + S32K3XX_HSE_CONFIG_CFPRH_OFFSET)
#define S32K3XX_HSE_CONFIG_DFPR             (S32K3XX_HSE_BASE + S32K3XX_HSE_CONFIG_DFPR_OFFSET)
#define S32K3XX_HSE_CONFIG_PE_LOCK          (S32K3XX_HSE_BASE + S32K3XX_HSE_CONFIG_PE_LOCK_OFFSET)
#define S32K3XX_HSE_CONFIG_RAMPR_ALT        (S32K3XX_HSE_BASE + S32K3XX_HSE_CONFIG_RAMPR_ALT_OFFSET)
#define S32K3XX_HSE_CONFIG_CFPRL_ALT        (S32K3XX_HSE_BASE + S32K3XX_HSE_CONFIG_CFPRL_ALT_OFFSET)
#define S32K3XX_HSE_CONFIG_CFPRH_ALT        (S32K3XX_HSE_BASE + S32K3XX_HSE_CONFIG_CFPRH_ALT_OFFSET)
#define S32K3XX_HSE_CONFIG_DFPR_ALT         (S32K3XX_HSE_BASE + S32K3XX_HSE_CONFIG_DFPR_ALT_OFFSET)
#define S32K3XX_HSE_CONFIG_REG_GPR          (S32K3XX_HSE_BASE + S32K3XX_HSE_CONFIG_REG_GPR_OFFSET)

/* HSE Register Bitfield Definitions ****************************************/

/* General Purpose Configuration 0 (CONFIG_REG0) */

                                                    /* Bits 0-5: Reserved */
#define HSE_CONFIG_REG0_EDB               (1 << 6)  /* Bit 6: Hardware Debugger Attached (EDB) */
                                                    /* Bits 7-31: Reserved */

/* General Purpose Configuration 6 (CONFIG_REG6) */

#define HSE_CONFIG_REG6_QUADSPI_SDID_PCTL (1 << 0)  /* Bit 0: QuadSPI Clock Gating (QUADSPI_SDID_PCTL) */
                                                    /* Bit 1: Reserved */
#define HSE_CONFIG_REG6_EMAC_CLOCK_GATE   (1 << 2)  /* Bit 2: Ethernet Clock Gating (EMAC_CLOCK_GATE) */
                                                    /* Bit 3: Reserved */
#define HSE_CONFIG_REG6_FLEXIO_CLOCK_GATE (1 << 4)  /* Bit 4: FlexIO Clock Gating (FLEXIO_CLOCK_GATE) */
#define HSE_CONFIG_REG6_SAI_SDID_PCTL     (1 << 5)  /* Bit 5: SAI0 and SAI1 Clock Gating (SAI_SDID_PCTL) */
                                                    /* Bits 6-30: Reserved */
#define HSE_CONFIG_REG6_HL                (1 << 31) /* Bit 31: Hard Lock (HL) */

/* Configuration RAM Protected Region (CONFIG_RAMPR) */

                                                    /* Bits 0-4: Reserved */

#define HSE_CONFIG_RAMPR_SECURE_SIZE_SHIFT (5)      /* Bits 5-20: Secure Size (SECURE_SIZE) */
#define HSE_CONFIG_RAMPR_SECURE_SIZE_MASK  (0xffff << HSE_CONFIG_RAMPR_SECURE_SIZE_SHIFT)

                                                    /* Bits 21-29: Reserved */
#define HSE_CONFIG_RAMPR_SOFT_LOCK        (1 << 30) /* Bit 30: Soft Lock (SOFT_LOCK) */
#define HSE_CONFIG_RAMPR_HARD_LOCK        (1 << 31) /* Bit 31: Hard Lock (HARD_LOCK) */

/* Configuration Code Flash Memory Active Block (CONFIG_CFPRL) */

                                                    /* Bits 0-12: Reserved */

#define HSE_CONFIG_CFPRL_SECURE_SIZE_SHIFT (13)     /* Bits 13-20: Secure Size (SECURE_SIZE) */
#define HSE_CONFIG_CFPRL_SECURE_SIZE_MASK  (0xff << HSE_CONFIG_CFPRL_SECURE_SIZE_SHIFT)

                                                    /* Bits 21-29: Reserved */
#define HSE_CONFIG_CFPRL_SOFT_LOCK        (1 << 30) /* Bit 30: Soft Lock (SOFT_LOCK) */
#define HSE_CONFIG_CFPRL_HARD_LOCK        (1 << 31) /* Bit 31: Hard Lock (HARD_LOCK) */

/* Configuration Code Flash Memory Passive Block (CONFIG_CFPRH) */

                                                    /* Bits 0-12: Reserved */

#define HSE_CONFIG_CFPRH_SECURE_SIZE_SHIFT (13)     /* Bits 13-20: Secure Size (SECURE_SIZE) */
#define HSE_CONFIG_CFPRH_SECURE_SIZE_MASK  (0xff << HSE_CONFIG_CFPRH_SECURE_SIZE_SHIFT)

                                                    /* Bits 21-29: Reserved */
#define HSE_CONFIG_CFPRH_SOFT_LOCK        (1 << 30) /* Bit 30: Soft Lock (SOFT_LOCK) */
#define HSE_CONFIG_CFPRH_HARD_LOCK        (1 << 31) /* Bit 31: Hard Lock (HARD_LOCK) */

/* Configuration Data Flash Memory Protected Region (CONFIG_DFPR) */

                                                    /* Bits 0-12: Reserved */
#define HSE_CONFIG_DFPR_SECURE_SIZE_SHIFT (13)      /* Bits 13-20: Secure Size (SECURE_SIZE) */
#define HSE_CONFIG_DFPR_SECURE_SIZE_MASK  (0xff << HSE_CONFIG_DFPR_SECURE_SIZE_SHIFT)
                                                    /* Bits 21-29: Reserved */
#define HSE_CONFIG_DFPR_SOFT_LOCK         (1 << 30) /* Bit 30: Soft Lock (SOFT_LOCK) */
#define HSE_CONFIG_DFPR_HARD_LOCK         (1 << 31) /* Bit 31: Hard Lock (HARD_LOCK) */

/* Configuration Program and Erase Lock (CONFIG_PE_LOCK) */

                                                    /* Bits 0-11: Reserved */
#define HSE_CONFIG_PE_LOCK_BLOCK_0        (1 << 12) /* Bit 12: Program/Erase Lock for Block 0 (PE_LOCK_BLOCK_0) */
#define HSE_CONFIG_PE_LOCK_BLOCK_1        (1 << 13) /* Bit 13: Program/Erase Lock for Block 1 (PE_LOCK_BLOCK_1) */
#define HSE_CONFIG_PE_LOCK_BLOCK_2        (1 << 14) /* Bit 14: Program/Erase Lock for Block 2 (PE_LOCK_BLOCK_2) */
#define HSE_CONFIG_PE_LOCK_BLOCK_3        (1 << 15) /* Bit 15: Program/Erase Lock for Block 3 (PE_LOCK_BLOCK_3) */
#define HSE_CONFIG_PE_LOCK_BLOCK_4        (1 << 16) /* Bit 16: Program/Erase Lock for Block 4 (PE_LOCK_BLOCK_4) */
                                                    /* Bit 17: Reserved */
#define HSE_CONFIG_PE_LOCK_UTEST          (1 << 18) /* Bit 18: Program/Erase Lock for UTEST (PE_LOCK_UTEST) */
                                                    /* Bits 19-31: Reserved */

/* Configuration RAM Protected Region Alternate (CONFIG_RAMPR_ALT) */

#define HSE_CONFIG_RAMPR_ALT_INVERT_VALUE_RAMPR_SHIFT (0) /* Bits 0-31: Invert Value RAMPR (INVERT_VALUE_RAMPR) */
#define HSE_CONFIG_RAMPR_ALT_INVERT_VALUE_RAMPR_MASK  (0xffffffff)

/* Configuration Code Flash Memory Active Block Alternative
 * (CONFIG_CFPRL_ALT)
 */

#define HSE_CONFIG_CFPRL_ALT_INVERT_VALUE_CFPRL_SHIFT (0) /* Bits 0-31: Invert Value CFPRL (INVERT_VALUE_CFPRL) */
#define HSE_CONFIG_CFPRL_ALT_INVERT_VALUE_CFPRL_MASK  (0xffffffff)

/* Configuration Code Flash Memory Passive Block Alternative
 * (CONFIG_CFPRH_ALT)
 */

#define HSE_CONFIG_CFPRH_ALT_INVERT_VALUE_CFPRH_SHIFT (0) /* Bits 0-31: Invert Value CFPRH (INVERT_VALUE_CFPRH) */
#define HSE_CONFIG_CFPRH_ALT_INVERT_VALUE_CFPRH_MASK  (0xffffffff)

/* Configuration Data Flash Memory Protected Region Alternative
 * (CONFIG_DFPR_ALT)
 */

#define HSE_CONFIG_DFPR_ALT_INVERT_VALUE_DFPR_SHIFT (0) /* Bits 0-31: Invert Value DFPR (INVERT_VALUE_DFPR) */
#define HSE_CONFIG_DFPR_ALT_INVERT_VALUE_DFPR_MASK  (0xffffffff)

/* Configuration REG_GPR (CONFIG_REG_GPR) */

#define HSE_CONFIG_REG_GPR_FIRC_DIV_SEL_SHIFT (0)   /* Bits 0-1: FIRC Divider (FIRC_DIV_SEL) */
#define HSE_CONFIG_REG_GPR_FIRC_DIV_SEL_MASK  (0x03 << HSE_CONFIG_REG_GPR_FIRC_DIV_SEL_SHIFT)
#  define HSE_CONFIG_REG_GPR_FIRC_DIV_SEL2    (0x00 << HSE_CONFIG_REG_GPR_FIRC_DIV_SEL_SHIFT) /* Didivded by 2 */
#  define HSE_CONFIG_REG_GPR_FIRC_DIV_SEL16   (0x02 << HSE_CONFIG_REG_GPR_FIRC_DIV_SEL_SHIFT) /* Didivded by 16 */

                                                    /* Bits 2-28: Reserved */
#define HSE_CONFIG_REG_GPR_APP_CORE_ACC_SHIFT (29)  /* Bits 29-31: APP_CORE_ACC */
#define HSE_CONFIG_REG_GPR_APP_CORE_ACC_MASK  (0x07 << HSE_CONFIG_REG_GPR_APP_CORE_ACC_SHIFT)

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_HSE_H */
