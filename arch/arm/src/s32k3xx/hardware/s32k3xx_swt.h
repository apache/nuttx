/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_swt.h
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

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_SWT_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_SWT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/s32k3xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SWT Register Offsets *****************************************************/

#define S32K3XX_SWT_CR_OFFSET  (0x00) /* Control Register (CR) */
#define S32K3XX_SWT_IR_OFFSET  (0x04) /* Interrupt Register (IR) */
#define S32K3XX_SWT_TO_OFFSET  (0x08) /* Timeout Register (TO) */
#define S32K3XX_SWT_WN_OFFSET  (0x0c) /* Window Register (WN) */
#define S32K3XX_SWT_SR_OFFSET  (0x10) /* Service Register (SR) */
#define S32K3XX_SWT_CO_OFFSET  (0x14) /* Counter Output Register (CO) */
#define S32K3XX_SWT_SK_OFFSET  (0x18) /* Service Key Register (SK) */
#define S32K3XX_SWT_RRR_OFFSET (0x1c) /* Event Request Register (RRR) */

/* SWT Register Addresses ***************************************************/

#define S32K3XX_SWT0_CR        (S32K3XX_SWT0_BASE + S32K3XX_SWT_CR_OFFSET)
#define S32K3XX_SWT0_IR        (S32K3XX_SWT0_BASE + S32K3XX_SWT_IR_OFFSET)
#define S32K3XX_SWT0_TO        (S32K3XX_SWT0_BASE + S32K3XX_SWT_TO_OFFSET)
#define S32K3XX_SWT0_WN        (S32K3XX_SWT0_BASE + S32K3XX_SWT_WN_OFFSET)
#define S32K3XX_SWT0_SR        (S32K3XX_SWT0_BASE + S32K3XX_SWT_SR_OFFSET)
#define S32K3XX_SWT0_CO        (S32K3XX_SWT0_BASE + S32K3XX_SWT_CO_OFFSET)
#define S32K3XX_SWT0_SK        (S32K3XX_SWT0_BASE + S32K3XX_SWT_SK_OFFSET)
#define S32K3XX_SWT0_RRR       (S32K3XX_SWT0_BASE + S32K3XX_SWT_RRR_OFFSET)

#define S32K3XX_SWT1_CR        (S32K3XX_SWT1_BASE + S32K3XX_SWT_CR_OFFSET)
#define S32K3XX_SWT1_IR        (S32K3XX_SWT1_BASE + S32K3XX_SWT_IR_OFFSET)
#define S32K3XX_SWT1_TO        (S32K3XX_SWT1_BASE + S32K3XX_SWT_TO_OFFSET)
#define S32K3XX_SWT1_WN        (S32K3XX_SWT1_BASE + S32K3XX_SWT_WN_OFFSET)
#define S32K3XX_SWT1_SR        (S32K3XX_SWT1_BASE + S32K3XX_SWT_SR_OFFSET)
#define S32K3XX_SWT1_CO        (S32K3XX_SWT1_BASE + S32K3XX_SWT_CO_OFFSET)
#define S32K3XX_SWT1_SK        (S32K3XX_SWT1_BASE + S32K3XX_SWT_SK_OFFSET)
#define S32K3XX_SWT1_RRR       (S32K3XX_SWT1_BASE + S32K3XX_SWT_RRR_OFFSET)

/* SWT Register Bitfield Definitions ****************************************/

/* Control Register (CR) */

#define SWT_CR_WEN             (1 << 0)  /* Bit 0: Watchdog Enable (WEN) */
#define SWT_CR_FRZ             (1 << 1)  /* Bit 1: Debug Mode Control (FRZ) */
#define SWT_CR_STP             (1 << 2)  /* Bit 2: Stop Mode Control (STP) */
                                         /* Bit 3: Reserved */
#define SWT_CR_SLK             (1 << 4)  /* Bit 4: Soft Lock (SLK) */
#define SWT_CR_HLK             (1 << 5)  /* Bit 5: Hard Lock (HLK) */
#define SWT_CR_ITR             (1 << 6)  /* Bit 6: Interrupt Then Reset Request (ITR) */
#define SWT_CR_WND             (1 << 7)  /* Bit 7: Window Mode (WND) */
#define SWT_CR_RIA             (1 << 8)  /* Bit 8: Reset on Invalid Access (RIA) */
#define SWT_CR_SMD_SHIFT       (9)       /* Bits 9-10: Service Mode (SMD) */
#define SWT_CR_SMD_MASK        (0x03 << SWT_CR_SMD_SHIFT)
#  define SWT_CR_SMD_FIXED     (0x00 << SWT_CR_SMD_SHIFT) /* Fixed Service Sequence */
#  define SWT_CR_SMD_KEYED     (0x01 << SWT_CR_SMD_SHIFT) /* Keyed Service Sequence */

                                         /* Bits 11-23: Reserved */
#define SWT_CR_MAP7            (1 << 24) /* Bit 24: Master Access Protection 7 (MAP7) */
#define SWT_CR_MAP6            (1 << 25) /* Bit 25: Master Access Protection 6 (MAP6) */
#define SWT_CR_MAP5            (1 << 26) /* Bit 26: Master Access Protection 5 (MAP5) */
#define SWT_CR_MAP4            (1 << 27) /* Bit 27: Master Access Protection 4 (MAP4) */
#define SWT_CR_MAP3            (1 << 28) /* Bit 28: Master Access Protection 3 (MAP3) */
#define SWT_CR_MAP2            (1 << 29) /* Bit 29: Master Access Protection 2 (MAP2) */
#define SWT_CR_MAP1            (1 << 30) /* Bit 30: Master Access Protection 1 (MAP1) */
#define SWT_CR_MAP0            (1 << 31) /* Bit 31: Master Access Protection 0 (MAP0) */

/* Interrupt Register (IR) */

#define SWT_IR_TIF             (1 << 0)  /* Bit 0: Timeout Interrupt Flag (TIF) */
                                         /* Bit 1-31: Reserved */

/* Timeout Register (TO) */

#define SWT_TO_WTO_SHIFT       (0)       /* Bits 0-31: Watchdog Timeout (WTO) */
#define SWT_TO_WTO_MASK        (0xffffffff << SWT_TO_WTO_SHIFT)

/* Window Register (WN) */

#define SWT_WN_WST_SHIFT       (0)       /* Bits 0-31: Window Start Value (WSV) */
#define SWT_WN_WST_MASK        (0xffffffff << SWT_WN_WST_SHIFT)

/* Service Register (SR) */

#define SWT_SR_WSC_SHIFT       (0)       /* Bits 0-15: Watchdog Service Code (WSC) */
#define SWT_SR_WSC_MASK        (0xffff << SWT_SR_WSC_SHIFT)
                                         /* Bits 16-31: Reserved */

/* Counter Output Register (CO) */

#define SWT_CO_CNT_SHIFT       (0)       /* Bits 0-31: Watchdog Count (CNT) */
#define SWT_CO_CNT_MASK        (0xffffffff << SWT_CO_CNT_SHIFT)

/* Service Key Register (SK) */

#define SWT_SK_SHIFT           (0)       /* Bits 0-15: Service Key (SK) */
#define SWT_SK_MASK            (0xffff << SWT_SK_SHIFT)
                                         /* Bits 16-31: Reserved */

/* Event Request Register (RRR) */

#define SWT_RRR_RRF            (1 << 0)  /* Bit 0: Reset Request Flag (RRF) */
                                         /* Bits 1-31: Reserved */

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_SWT_H */
