/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_stm.h
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

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_STM_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_STM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/s32k3xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* STM Register Offsets *****************************************************/

#define S32K3XX_STM_CR_OFFSET   (0x00) /* Control Register (CR) */
#define S32K3XX_STM_CNT_OFFSET  (0x04) /* Count Register (CNT) */
#define S32K3XX_STM_CCR0_OFFSET (0x10) /* Channel Control Register 0 (CCR0) */
#define S32K3XX_STM_CIR0_OFFSET (0x14) /* Channel Interrupt Register 0 (CIR0) */
#define S32K3XX_STM_CMP0_OFFSET (0x18) /* Channel Compare Register 0 (CMP0) */
#define S32K3XX_STM_CCR1_OFFSET (0x20) /* Channel Control Register 1 (CCR1) */
#define S32K3XX_STM_CIR1_OFFSET (0x24) /* Channel Interrupt Register 1 (CIR1) */
#define S32K3XX_STM_CMP1_OFFSET (0x28) /* Channel Compare Register 1 (CMP1) */
#define S32K3XX_STM_CCR2_OFFSET (0x30) /* Channel Control Register 2 (CCR2) */
#define S32K3XX_STM_CIR2_OFFSET (0x34) /* Channel Interrupt Register 2 (CIR2) */
#define S32K3XX_STM_CMP2_OFFSET (0x38) /* Channel Compare Register 2 (CMP2) */
#define S32K3XX_STM_CCR3_OFFSET (0x40) /* Channel Control Register 3 (CCR3) */
#define S32K3XX_STM_CIR3_OFFSET (0x44) /* Channel Interrupt Register 3 (CIR3) */
#define S32K3XX_STM_CMP3_OFFSET (0x48) /* Channel Compare Register 3 (CMP3) */

/* STM Register Addresses ***************************************************/

#define S32K3XX_STM0_CR         (S32K3XX_STM0_BASE + S32K3XX_STM_CR_OFFSET)
#define S32K3XX_STM0_CNT        (S32K3XX_STM0_BASE + S32K3XX_STM_CNT_OFFSET)
#define S32K3XX_STM0_CCR0       (S32K3XX_STM0_BASE + S32K3XX_STM_CCR0_OFFSET)
#define S32K3XX_STM0_CIR0       (S32K3XX_STM0_BASE + S32K3XX_STM_CIR0_OFFSET)
#define S32K3XX_STM0_CMP0       (S32K3XX_STM0_BASE + S32K3XX_STM_CMP0_OFFSET)
#define S32K3XX_STM0_CCR1       (S32K3XX_STM0_BASE + S32K3XX_STM_CCR1_OFFSET)
#define S32K3XX_STM0_CIR1       (S32K3XX_STM0_BASE + S32K3XX_STM_CIR1_OFFSET)
#define S32K3XX_STM0_CMP1       (S32K3XX_STM0_BASE + S32K3XX_STM_CMP1_OFFSET)
#define S32K3XX_STM0_CCR2       (S32K3XX_STM0_BASE + S32K3XX_STM_CCR2_OFFSET)
#define S32K3XX_STM0_CIR2       (S32K3XX_STM0_BASE + S32K3XX_STM_CIR2_OFFSET)
#define S32K3XX_STM0_CMP2       (S32K3XX_STM0_BASE + S32K3XX_STM_CMP2_OFFSET)
#define S32K3XX_STM0_CCR3       (S32K3XX_STM0_BASE + S32K3XX_STM_CCR3_OFFSET)
#define S32K3XX_STM0_CIR3       (S32K3XX_STM0_BASE + S32K3XX_STM_CIR3_OFFSET)
#define S32K3XX_STM0_CMP3       (S32K3XX_STM0_BASE + S32K3XX_STM_CMP3_OFFSET)

#define S32K3XX_STM1_CR         (S32K3XX_STM1_BASE + S32K3XX_STM_CR_OFFSET)
#define S32K3XX_STM1_CNT        (S32K3XX_STM1_BASE + S32K3XX_STM_CNT_OFFSET)
#define S32K3XX_STM1_CCR0       (S32K3XX_STM1_BASE + S32K3XX_STM_CCR0_OFFSET)
#define S32K3XX_STM1_CIR0       (S32K3XX_STM1_BASE + S32K3XX_STM_CIR0_OFFSET)
#define S32K3XX_STM1_CMP0       (S32K3XX_STM1_BASE + S32K3XX_STM_CMP0_OFFSET)
#define S32K3XX_STM1_CCR1       (S32K3XX_STM1_BASE + S32K3XX_STM_CCR1_OFFSET)
#define S32K3XX_STM1_CIR1       (S32K3XX_STM1_BASE + S32K3XX_STM_CIR1_OFFSET)
#define S32K3XX_STM1_CMP1       (S32K3XX_STM1_BASE + S32K3XX_STM_CMP1_OFFSET)
#define S32K3XX_STM1_CCR2       (S32K3XX_STM1_BASE + S32K3XX_STM_CCR2_OFFSET)
#define S32K3XX_STM1_CIR2       (S32K3XX_STM1_BASE + S32K3XX_STM_CIR2_OFFSET)
#define S32K3XX_STM1_CMP2       (S32K3XX_STM1_BASE + S32K3XX_STM_CMP2_OFFSET)
#define S32K3XX_STM1_CCR3       (S32K3XX_STM1_BASE + S32K3XX_STM_CCR3_OFFSET)
#define S32K3XX_STM1_CIR3       (S32K3XX_STM1_BASE + S32K3XX_STM_CIR3_OFFSET)
#define S32K3XX_STM1_CMP3       (S32K3XX_STM1_BASE + S32K3XX_STM_CMP3_OFFSET)

/* STM Register Bitfield Definitions ****************************************/

/* Control Register (CR) */

#define STM_CR_TEN              (1 << 0)  /* Bit 0: Timer Enable (TEN) */
#define STM_CR_FRZ              (1 << 1)  /* Bit 1: Freeze (FRZ) */
                                          /* Bits 2-7: Reserved */
#define STM_CR_CPS_SHIFT        (8)       /* Bits 8-15: Counter Prescaler (CPS) */
#define STM_CR_CPS_MASK         (0xff << STM_CR_CPS_SHIFT)
#define STM_CR_CPS(n)           (((n) << STM_CR_CPS_SHIFT) & STM_CR_CPS_MASK)
                                          /* Bits 16-31: Reserved */

/* Count Register (CNT) */

#define STM_CNT_SHIFT           (0)       /* Bits 0-31: Timer Count (CNT) */
#define STM_CNT_MASK            (0xffffffff << STM_CNT_SHIFT)

/* Channel Control Register n (CCRn) */

#define STM_CCR_CEN             (1 << 0)  /* Bit 0: Channel Enable (CEN) */
                                          /* Bits 1-31: Reserved */

/* Channel Interrupt Register n (CIRn) */

#define STM_CIR_CIF             (1 << 0)  /* Bit 0: Channel Interrupt Flag (CIF) */
                                          /* Bits 1-31: Reserved */

/* Channel Compare Register n (CMPn) */

#define STM_CMP_SHIFT           (0)       /* Bits 0-31: Channel Compare (CMP) */
#define STM_CMP_MASK            (0xffffffff << STM_CMP_SHIFT)

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_STM_H */
