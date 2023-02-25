/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_wkpu.h
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

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_WKPU_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_WKPU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/s32k3xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* WKPU Register Offsets ****************************************************/

#define S32K3XX_WKPU_NSR_OFFSET       (0x00) /* WKPU NMI Status Flag Register (NSR) */
#define S32K3XX_WKPU_NCR_OFFSET       (0x08) /* WKPU NMI Configuration Register (NCR) */
#define S32K3XX_WKPU_WISR_OFFSET      (0x14) /* WKPU Wakeup/Interrupt Status Flag Register (WISR) */
#define S32K3XX_WKPU_IRER_OFFSET      (0x18) /* WKPU Interrupt Request Enable Register (IRER) */
#define S32K3XX_WKPU_WRER_OFFSET      (0x1c) /* WKPU Wakeup Request Enable Register (WRER) */
#define S32K3XX_WKPU_WIREER_OFFSET    (0x28) /* WKPU Wakeup/Interrupt Rising-Edge Event Enable Register (WIREER) */
#define S32K3XX_WKPU_WIFEER_OFFSET    (0x2c) /* WKPU Wakeup/Interrupt Falling-Edge Event Enable Register (WIFEER) */
#define S32K3XX_WKPU_WIFER_OFFSET     (0x30) /* WKPU Wakeup/Interrupt Filter Enable Register (WIFER) */
#define S32K3XX_WKPU_WISR_64_OFFSET   (0x54) /* WKPU Wakeup/Interrupt Status Flag Register (WISR_64) */
#define S32K3XX_WKPU_IRER_64_OFFSET   (0x58) /* WKPU Interrupt Request Enable Register (IRER_64) */
#define S32K3XX_WKPU_WRER_64_OFFSET   (0x5c) /* WKPU Wakeup Request Enable Register (WRER_64) */
#define S32K3XX_WKPU_WIREER_64_OFFSET (0x68) /* WKPU Wakeup/Interrupt Rising-Edge Event Enable Register (WIREER_64) */
#define S32K3XX_WKPU_WIFEER_64_OFFSET (0x6c) /* WKPU Wakeup/Interrupt Falling-Edge Event Enable Register (WIFEER_64) */
#define S32K3XX_WKPU_WIFER_64_OFFSET  (0x70) /* WKPU Wakeup/Interrupt Filter Enable Register (WIFER_64) */

/* WKPU Register Addresses **************************************************/

#define S32K3XX_WKPU_NSR              (S32K3XX_WKPU_BASE + S32K3XX_WKPU_NSR_OFFSET)
#define S32K3XX_WKPU_NCR              (S32K3XX_WKPU_BASE + S32K3XX_WKPU_NCR_OFFSET)
#define S32K3XX_WKPU_WISR             (S32K3XX_WKPU_BASE + S32K3XX_WKPU_WISR_OFFSET)
#define S32K3XX_WKPU_IRER             (S32K3XX_WKPU_BASE + S32K3XX_WKPU_IRER_OFFSET)
#define S32K3XX_WKPU_WRER             (S32K3XX_WKPU_BASE + S32K3XX_WKPU_WRER_OFFSET)
#define S32K3XX_WKPU_WIREER           (S32K3XX_WKPU_BASE + S32K3XX_WKPU_WIREER_OFFSET)
#define S32K3XX_WKPU_WIFEER           (S32K3XX_WKPU_BASE + S32K3XX_WKPU_WIFEER_OFFSET)
#define S32K3XX_WKPU_WIFER            (S32K3XX_WKPU_BASE + S32K3XX_WKPU_WIFER_OFFSET)
#define S32K3XX_WKPU_WISR_64          (S32K3XX_WKPU_BASE + S32K3XX_WKPU_WISR_64_OFFSET)
#define S32K3XX_WKPU_IRER_64          (S32K3XX_WKPU_BASE + S32K3XX_WKPU_IRER_64_OFFSET)
#define S32K3XX_WKPU_WRER_64          (S32K3XX_WKPU_BASE + S32K3XX_WKPU_WRER_64_OFFSET)
#define S32K3XX_WKPU_WIREER_64        (S32K3XX_WKPU_BASE + S32K3XX_WKPU_WIREER_64_OFFSET)
#define S32K3XX_WKPU_WIFEER_64        (S32K3XX_WKPU_BASE + S32K3XX_WKPU_WIFEER_64_OFFSET)
#define S32K3XX_WKPU_WIFER_64         (S32K3XX_WKPU_BASE + S32K3XX_WKPU_WIFER_64_OFFSET)

/* WKPU Register Bitfield Definitions ***************************************/

/* WKPU NMI Status Flag Register (NSR) */

                                                 /* Bits 0-21: Reserved */
#define WKPU_NSR_NOVF1                (1 << 22)  /* Bit 22: NMI Overrun Status Flag 1 (NOVF1) */
#define WKPU_NSR_NIF1                 (1 << 23)  /* Bit 23: NMI Status Flag 1 (NIF1) */
                                                 /* Bits 24-29: Reserved */
#define WKPU_NSR_NOVF0                (1 << 30)  /* Bit 30: NMI Overrun Status Flag 0 (NOVF0) */
#define WKPU_NSR_NIF0                 (1 << 31)  /* Bit 31: NMI Status Flag 0 (NIF0) */

/* WKPU NMI Configuration Register (NCR) */

                                                 /* Bits 0-15: Reserved */
#define WKPU_NCR_NFE1                 (1 << 16)  /* Bit 16: NMI Filter Enable 1 (NFE1) */
#define WKPU_NCR_NFEE1                (1 << 17)  /* Bit 17: NMI Falling-edge Events Enable 1 (NFEE1) */
#define WKPU_NCR_NREE1                (1 << 18)  /* Bit 18: NMI Rising-Edge Events Enable 1 (NREE1) */
                                                 /* Bit 19: Reserved */
#define WKPU_NCR_NWRE1                (1 << 20)  /* Bit 20: NMI Wakeup Request Enable 1 (NWRE1) */
#define WKPU_NCR_NDSS1_SHIFT          (21)       /* Bits 21-22: NMI Destination Source Select 1 (NDSS1) */
#define WKPU_NCR_NDSS1_MASK           (0x03 << WKPU_NCR_NDSS1_SHIFT)
#  define WKPU_NCR_NDSS1_NMI          (0x00 << WKPU_NCR_NDSS1_SHIFT) /* Non-maskable interrupt */

#define WKPU_NCR_NLOCK1               (1 << 23)  /* Bit 23: NMI Configuration Lock Register 1 (NLOCK1) */
#define WKPU_NCR_NFE0                 (1 << 24)  /* Bit 24: NMI Filter Enable 0 (NFE0) */
#define WKPU_NCR_NFEE0                (1 << 25)  /* Bit 25: NMI Falling-edge Events Enable 0 (NFEE0) */
#define WKPU_NCR_NREE0                (1 << 26)  /* Bit 26: NMI Rising-Edge Events Enable 0 (NREE0) */
                                                 /* Bit 27: Reserved */
#define WKPU_NCR_NWRE0                (1 << 28)  /* Bit 28: NMI Wakeup Request Enable 0 (NWRE0) */
#define WKPU_NCR_NDSS0_SHIFT          (29)       /* Bits 29-30: NMI Destination Source Select 0 (NDSS0) */
#define WKPU_NCR_NDSS0_MASK           (0x03 << WKPU_NCR_NDSS0_SHIFT)
#  define WKPU_NCR_NDSS0_NMI          (0x00 << WKPU_NCR_NDSS0_SHIFT) /* Non-maskable interrupt */

#define WKPU_NCR_NLOCK0               (1 << 31)  /* Bit 31: NMI Configuration Lock Register 0 (NLOCK0) */

/* WKPU Wakeup/Interrupt Status Flag Register (WISR, WISR_64) */

#define WKPU_WISR_EIF(n)              (1 << (n)) /* Bit n: External Wakeup/Interrupt Status Flag n (EIFn) */

/* WKPU Interrupt Request Enable Register (IRER, IRER_64) */

#define WKPU_IRER_EIRE(n)             (1 << (n)) /* Bit n: External Interrupt Request Enable n (EIREn) */

/* WKPU Wakeup Request Enable Register (WRER, WRER_64) */

#define WKPU_WRER_WRE(n)              (1 << (n)) /* Bit n: External Wakeup Request Enable n (WREn) */

/* WKPU Wakeup/Interrupt Rising-Edge Event Enable Register
 * (WIREER, WIREER_64)
 */

#define WKPU_WIREER_IREE(n)           (1 << (n)) /* Bit n: External Interrupt Rising-edge Events Enable n (IREEn) */

/* WKPU Wakeup/Interrupt Falling-Edge Event Enable Register
 * (WIFEER, WIFEER_64)
 */

#define WKPU_WIFEER_IFEE(n)           (1 << (n)) /* Bit n: External Interrupt Falling-edge Events Enable n (IFEEn) */

/* WKPU Wakeup/Interrupt Filter Enable Register (WIFER, WIFER_64) */

#define WKPU_WIFER_IFE(n)             (1 << (n)) /* Bit n: External Interrupt Filter Enable n (IFEn) */

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_WKPU_H */
