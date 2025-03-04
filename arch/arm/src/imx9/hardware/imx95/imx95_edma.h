/****************************************************************************
 * arch/arm/src/imx9/hardware/imx95/imx95_edma.h
 *
 * SPDX-License-Identifier: Apache-2.0
 * SPDX-FileCopyrightText: 2024 NXP
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

#ifndef __ARCH_ARM_SRC_IMX9_HARDWARE_IMX95_IMX95_EDMA_H
#define __ARCH_ARM_SRC_IMX9_HARDWARE_IMX95_IMX95_EDMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include "imx95_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* eDMA3 / eDMA5 Register Offsets */

#define IMX9_EDMA_CSR_OFFSET              (0x000000)              /* Management Page Control Register (CSR) */
#define IMX9_EDMA_ES_OFFSET               (0x000004)              /* Management Page Error Status Register (ES) */
#define IMX9_EDMA_CH_GRPRI_OFFSET(n)      (0x000100 + ((n) << 2)) /* Channel n Arbitration Group Register (CHn_GRPRI) */

/* eDMA3 only */

#define IMX9_EDMA_INT_OFFSET              (0x000008) /* Management Page Interrupt Request Status Register (INT) */
#define IMX9_EDMA_HRS_OFFSET              (0x00000c) /* Management Page Hardware Request Status Register (HRS) */

/* eDMA5 only */

#define IMX9_EDMA_INT_LOW_OFFSET          (0x000008) /* Management Page Interrupt Request Status Register (INT_LOW) */
#define IMX9_EDMA_INT_HIGH_OFFSET         (0x00000c) /* Management Page Interrupt Request Status Register (INT_HIGH) */
#define IMX9_EDMA_HRS_LOW_OFFSET          (0x000010) /* Management Page Hardware Request Status Register (HRS_LOW) */
#define IMX9_EDMA_HRS_HIGH_OFFSET         (0x000014) /* Management Page Hardware Request Status Register (HRS_HIGH) */

/* eDNA5 only */
#define IMX9_EDMA_MP_CH_MUX_OFFSET(n)     (0x000200 + ((n) << 2)) /* Channel Multiplexor Configuration (CH_MUX) */

/* eDMA3 / eDMA5 Register Addresses */

#define IMX9_EDMA_CSR(n)                  ((n) + IMX9_EDMA_CSR_OFFSET)
#define IMX9_EDMA_ES(n)                   ((n) + IMX9_EDMA_ES_OFFSET)
#define IMX9_EDMA_CH_GRPRI(n,c)           ((n) + IMX9_EDMA_CH_GRPRI_OFFSET(n))

/* eDMA3 only */

#define IMX9_EDMA_INT                     (IMX9_DMA3_BASE + IMX9_EDMA_INT_OFFSET)
#define IMX9_EDMA_HRS                     (IMX9_DMA3_BASE + IMX9_EDMA_HRS_OFFSET)

/* eDMA5 only */

#define IMX9_EDMA_INT_LOW                 (IMX9_EDMA5_2_BASE + IMX9_EDMA_INT_LOW_OFFSET)
#define IMX9_EDMA_INT_HIGH                (IMX9_EDMA5_2_BASE + IMX9_EDMA_INT_HIGH_OFFSET)
#define IMX9_EDMA_HRS_LOW                 (IMX9_EDMA5_2_BASE + IMX9_EDMA_HRS_LOW_OFFSET)
#define IMX9_EDMA_HRS_HIGH                (IMX9_EDMA5_2_BASE + IMX9_EDMA_HRS_HIGH_OFFSET)

/* eDMA Transfer Control Descriptor (TCD) Register Offsets */

#define IMX9_EDMA_CH_CSR_OFFSET           (0x000000) /* Channel Control and Status Register (CH0_CSR) */
#define IMX9_EDMA_CH_ES_OFFSET            (0x000004) /* Channel Error Status Register (CH0_ES) */
#define IMX9_EDMA_CH_INT_OFFSET           (0x000008) /* Channel Interrupt Status Register (CH0_INT) */
#define IMX9_EDMA_CH_SBR_OFFSET           (0x00000c) /* Channel System Bus Register (CH0_SBR) */
#define IMX9_EDMA_CH_PRI_OFFSET           (0x000010) /* Channel Priority Register (CH0_PRI) */
#define IMX9_EDMA_CH_MUX_OFFSET           (0x000014) /* Channel Multiplexor Configuration (CH0_MUX) (eDMA5 only) */
#define IMX9_EDMA_CH_MATTR_OFFSET         (0x000018) /* Memory Attributes Register (CH0_MATTR) (eDMA5 only) */
#define IMX9_EDMA_TCD_SADDR_OFFSET        (0x000020) /* TCD Source Address Register (TCD0_SADDR) */
#define IMX9_EDMA_TCD_SOFF_OFFSET         (0x000024) /* TCD Signed Source Address Offset Register (TCD0_SOFF) */
#define IMX9_EDMA_TCD_ATTR_OFFSET         (0x000026) /* TCD Transfer Attributes (TCD0_ATTR) */
#define IMX9_EDMA_TCD_NBYTES_OFFSET       (0x000028) /* TCD Transfer Size (TCD0_NBYTES) */
#define IMX9_EDMA_TCD_SLAST_SDA_OFFSET    (0x00002c) /* TCD Last Source Address Adjustment / Store DADDR Address Register (TCD0_SLAST_SDA) */
#define IMX9_EDMA_TCD_DADDR_OFFSET        (0x000030) /* TCD Destination Address Register (TCD0_DADDR) */
#define IMX9_EDMA_TCD_DOFF_OFFSET         (0x000034) /* TCD Signed Destination Address Offset Register (TCD0_DOFF) */
#define IMX9_EDMA_TCD_CITER_OFFSET        (0x000036) /* TCD Current Major Loop Count Register (TCD0_CITER) */
#define IMX9_EDMA_TCD_DLAST_SGA_OFFSET    (0x000038) /* TCD Last Destination Address Adjustment / Scatter Gather Address Register (TCD0_DLAST_SGA)*/
#define IMX9_EDMA_TCD_CSR_OFFSET          (0x00003c) /* TCD Control and Status Register (TCD0_CSR) */
#define IMX9_EDMA_TCD_BITER_OFFSET        (0x00003e) /* TCD Beginning Major Loop Count Register (TCD0_BITER) */

/* eDMA 3 and eDMA 5 have TCD instance offsets, but same base offset */

#define IMX9_EDMA_TCD_BASE_OFFSET         (0x10000) /* Offset to TCD for both eDMA3/4 */
#define IMX9_EDMA3_TCD_INST_OFFSET        (0x10000) /* Per instance TCD offset for eDMA3 */
#define IMX9_EDMA5_TCD_INST_OFFSET        (0x8000)  /* Per instance TCD offset for eDMA5 */
#define IMX9_EDMA_TCD_BASE(n)             ((n) + IMX9_EDMA_TCD_BASE_OFFSET)
#define IMX9_EDMA_TCD_INST_OFFSET(n)      ((n) == IMX9_DMA3_BASE ? IMX9_EDMA3_TCD_INST_OFFSET : IMX9_EDMA5_TCD_INST_OFFSET)
#define IMX9_EDMA_TCD(n,t)                (IMX9_EDMA_TCD_BASE(n) + (t) * IMX9_EDMA_TCD_INST_OFFSET(n))

/* eDMA Transfer Control Descriptor (TCD) Register Addresses ****************/

#define IMX9_EDMA_CH_CSR(n,t)             (IMX9_EDMA_TCD(n,t) + IMX9_EDMA_CH_CSR_OFFSET)
#define IMX9_EDMA_CH_ES(n,t)              (IMX9_EDMA_TCD(n,t) + IMX9_EDMA_CH_ES_OFFSET)
#define IMX9_EDMA_CH_INT(n,t)             (IMX9_EDMA_TCD(n,t) + IMX9_EDMA_CH_INT_OFFSET)
#define IMX9_EDMA_CH_SBR(n,t)             (IMX9_EDMA_TCD(n,t) + IMX9_EDMA_CH_SBR_OFFSET)
#define IMX9_EDMA_CH_PRI(n,t)             (IMX9_EDMA_TCD(n,t) + IMX9_EDMA_CH_PRI_OFFSET)
#define IMX9_EDMA_CH_MUX(n,t)             (IMX9_EDMA_TCD(n,t) + IMX9_EDMA_CH_MUX_OFFSET)
#define IMX9_EDMA_TCD_SADDR(n,t)          (IMX9_EDMA_TCD(n,t) + IMX9_EDMA_TCD_SADDR_OFFSET)
#define IMX9_EDMA_TCD_SOFF(n,t)           (IMX9_EDMA_TCD(n,t) + IMX9_EDMA_TCD_SOFF_OFFSET)
#define IMX9_EDMA_TCD_ATTR(n,t)           (IMX9_EDMA_TCD(n,t) + IMX9_EDMA_TCD_ATTR_OFFSET)
#define IMX9_EDMA_TCD_NBYTES(n,t)         (IMX9_EDMA_TCD(n,t) + IMX9_EDMA_TCD_NBYTES_OFFSET)
#define IMX9_EDMA_TCD_SLAST_SDA(n,t)      (IMX9_EDMA_TCD(n,t) + IMX9_EDMA_TCD_SLAST_SDA_OFFSET)
#define IMX9_EDMA_TCD_DADDR(n,t)          (IMX9_EDMA_TCD(n,t) + IMX9_EDMA_TCD_DADDR_OFFSET)
#define IMX9_EDMA_TCD_DOFF(n,t)           (IMX9_EDMA_TCD(n,t) + IMX9_EDMA_TCD_DOFF_OFFSET)
#define IMX9_EDMA_TCD_CITER(n,t)          (IMX9_EDMA_TCD(n,t) + IMX9_EDMA_TCD_CITER_OFFSET)
#define IMX9_EDMA_TCD_DLAST_SGA(n,t)      (IMX9_EDMA_TCD(n,t) + IMX9_EDMA_TCD_DLAST_SGA_OFFSET)
#define IMX9_EDMA_TCD_CSR(n,t)            (IMX9_EDMA_TCD(n,t) + IMX9_EDMA_TCD_CSR_OFFSET)
#define IMX9_EDMA_TCD_BITER(n,t)          (IMX9_EDMA_TCD(n,t) + IMX9_EDMA_TCD_BITER_OFFSET)

/* eDMA Register Bitfield Definitions ***************************************/

/* Management Page Control Register (CSR) */

                                                    /* Bit 0: Reserved */
#define EDMA_CSR_EDBG                     (1 << 1)  /* Bit 1: Enable Debug (EDBG) */
#define EDMA_CSR_ERCA                     (1 << 2)  /* Bit 2: Enable Round Robin Channel Arbitration (ERCA) */
                                                    /* Bit 3: Reserved */
#define EDMA_CSR_HAE                      (1 << 4)  /* Bit 4: Halt After Error (HAE) */
#define EDMA_CSR_HALT                     (1 << 5)  /* Bit 5: Halt DMA Operations (HALT) */
#define EDMA_CSR_GCLC                     (1 << 6)  /* Bit 6: Global Channel Linking Control (GCLC) */
#define EDMA_CSR_GMRC                     (1 << 7)  /* Bit 7: Global Master ID Replication Control (GMRC) */
#define EDMA_CSR_ECX                      (1 << 8)  /* Bit 8: Cancel Transfer With Error (ECX) */
#define EDMA_CSR_CX                       (1 << 9)  /* Bit 9: Cancel Transfer (CX) */
                                                    /* Bits 10-23: Reserved */
#define EDMA_CSR_ACTIVE_ID_SHIFT          (24)      /* Bits 24-28: Active Channel ID (ACTIVE_ID) */
#define EDMA_CSR_ACTIVE_ID_MASK           (0x1f << EDMA_CSR_ACTIVE_ID_SHIFT)
                                                    /* Bits 29-30: Reserved */
#define EDMA_CSR_ACTIVE                   (1 << 31) /* Bit 31: DMA Active Status (ACTIVE) */

/* Management Page Error Status Register (ES) */

#define EDMA_ES_DBE                       (1 << 0)  /* Bit 0: Destination Bus Error (DBE) */
#define EDMA_ES_SBE                       (1 << 1)  /* Bit 1: Source Bus Error (SBE) */
#define EDMA_ES_SGE                       (1 << 2)  /* Bit 2: Scatter/Gather Configuration Error (SGE) */
#define EDMA_ES_NCE                       (1 << 3)  /* Bit 3: NBYTES/CITER Configuration Error (NCE) */
#define EDMA_ES_DOE                       (1 << 4)  /* Bit 4: Destination Offset Error (DOE) */
#define EDMA_ES_DAE                       (1 << 5)  /* Bit 5: Destination Address Error (DAE) */
#define EDMA_ES_SOE                       (1 << 6)  /* Bit 6: Source Offset Error (SOE) */
#define EDMA_ES_SAE                       (1 << 7)  /* Bit 7: Source Address Error (SAE) */
#define EDMA_ES_ECX                       (1 << 8)  /* Bit 8: Transfer Canceled (ECX) */
                                                    /* Bits 9-23: Reserved */
#define EDMA_ES_ERRCHN_SHIFT              (24)      /* Bits 24-28: Error Channel Number or Canceled Channel Number (ERRCHN) */
#define EDMA_ES_ERRCHN_MASK               (0x1f << EDMA_ES_ERRCHN_SHIFT)
                                                    /* Bits 29-30: Reserved */
#define EDMA_ES_VLD                       (1 << 31) /* Bit 31: Logical OR of all ERR status fields (VALID) */

/* Management Page Interrupt Request Status Register (INT) */

#define EDMA_INT(n)                       (1 << (n)) /* Bit n: Interrupt Request Status (INT) */

/* Management Page Hardware Request Status Register (HRS) */

#define EDMA_HRS(n)                       (1 << (n)) /* Bit n: Hardware Request Status (HRS) */

/* Channel n Arbitration Group Register (CHn_GRPRI) */

#define EDMA_CH_GRPRI_SHIFT               (0)       /* Bits 0-4: Arbitration Group For Channel n (GRPRI) */
#define EDMA_CH_GRPRI_MASK                (0x1f << EDMA_CH_GRPRI_SHIFT)
                                                    /* Bits 5-31: Reserved */

/* eDMA Transfer Control Descriptor (TCD) Bitfield Definitions **************/

/* Channel n Control and Status Register (CHn_CSR) */

#define EDMA_CH_CSR_ERQ                   (1 << 0)  /* Bit 0: Enable DMA Request (ERQ) */
#define EDMA_CH_CSR_EARQ                  (1 << 1)  /* Bit 1: Enable Asynchronous DMA Request in Stop Mode for Channel (EARQ) */
#define EDMA_CH_CSR_EEI                   (1 << 2)  /* Bit 2: Enable Error Interrupt (EEI) */
#define EDMA_CH_CSR_EBW                   (1 << 3)  /* Bit 3: Enable Buffered Writes (EBW) */
                                                    /* Bit 4-29: Reserved */
#define EDMA_CH_CSR_DONE                  (1 << 30) /* Bit 30: Channel Done (DONE) */
#define EDMA_CH_CSR_ACTIVE                (1 << 31) /* Bit 31: CHannel Active (ACTIVE) */

/* Channel n Error Status Register (CHn_ES) */

#define EDMA_CH_ES_DBE                    (1 << 0)  /* Bit 0: Destination Bus Error (DBE) */
#define EDMA_CH_ES_SBE                    (1 << 1)  /* Bit 1: Source Bus Error (SBE) */
#define EDMA_CH_ES_SGE                    (1 << 2)  /* Bit 2: Scatter/Gather Configuration Error (SGE) */
#define EDMA_CH_ES_NCE                    (1 << 3)  /* Bit 3: NBYTES/CITER Configuration Error (NCE) */
#define EDMA_CH_ES_DOE                    (1 << 4)  /* Bit 4: Destination Offset Error (DOE) */
#define EDMA_CH_ES_DAE                    (1 << 5)  /* Bit 5: Destination Address Error (DAE) */
#define EDMA_CH_ES_SOE                    (1 << 6)  /* Bit 6: Source Offset Error (SOE) */
#define EDMA_CH_ES_SAE                    (1 << 7)  /* Bit 7: Source Address Error (SAE) */
                                                    /* Bit 8-30: Reserved */
#define EDMA_CH_ES_ERR                    (1 << 31) /* Bit 31: Error in this channel (ERR) */

/* Channel n Interrupt Status Register (CHn_INT) */

#define EDMA_CH_INT                       (1 << 0)  /* Bit 0: Interrupt Request (INT) */
                                                    /* Bits 1-31: Reserved */

/* Channel n System Bus Register (CHn_SBR) */

#define EDMA_CH_SBR_MID_SHIFT             (0)       /* Bits 0-3: Master ID (MID) */
#define EDMA_CH_SBR_MID_MASK              (0x0f << EDMA_CH_SBR_MID_SHIFT)
                                                    /* Bits 4-13: Reserved */
#define EDMA_CH_SBR_SEC                   (1 << 14) /* Bit 14: Security Level (SEC) */
#define EDMA_CH_SBR_PAL                   (1 << 15) /* Bit 15: Privileged Access Level (PAL) */
#define EDMA_CH_SBR_EMI                   (1 << 16) /* Bit 16: Enable Master ID Replication (EMI) */
#define EDMA_CH_SBR_ATTR_SHIFT            (17)      /* Bits 17-19: Attribute Output (ATTR) */
#define EDMA_CH_SBR_ATTR_MASK             (0x07 << EDMA_CH_SBR_ATTR_SHIFT)
                                                    /* Bits 20-31: Reserved */

/* Channel n Priority Register (CHn_PRI) */

#define EDMA_CH_PRI_APL_SHIFT             (0)       /* Bits 0-2: Arbitration Priority Level (APL) */
#define EDMA_CH_PRI_APL_MASK              (0x07 << EDMA_CH_PRI_APL_SHIFT)
                                                    /* Bits 3-29: Reserved */
#define EDMA_CH_PRI_DPA                   (1 << 30) /* Bit 30: Disable Preempt Ability (DPA) */
#define EDMA_CH_PRI_ECP                   (1 << 31) /* Bit 31: Enable Channel Preemption (ECP) */

/* Channel Multiplexor Configuration (CHn_MUX) */

#define EDMA_CH_SRC_SHIFT                 (0)       /* Bits 0-6: Service Request Source */
#define EDMA_CH_SRC_MASK                  (0x7f << EDMA_CH_SRC_SHIFT)

/* TCDn Source Address Register (TCDn_SADDR) */

#define EDMA_TCD_SADDR_SHIFT              (0)       /* Bits 0-31: Source Address (SADDR) */
#define EDMA_TCD_SADDR_MASK               (0xffffffff << EDMA_TCD_SADDR_SHIFT)

/* TCDn Signed Source Address Offset Register (TCDn_SOFF) */

#define EDMA_TCD_SOFF_SHIFT               (0)       /* Bits 0-31: Source Address Signed Offset (SOFF) */
#define EDMA_TCD_SOFF_MASK                (0xffffffff << EDMA_TCD_SOFF_SHIFT)

/* TCDn Transfer Attributes (TCDn_ATTR) */

#define EDMA_TCD_ATTR_DSIZE_SHIFT         (0)       /* Bits 0-2: Destination Data Transfer Size (DSIZE) */
#define EDMA_TCD_ATTR_DSIZE_MASK          (0x07 << EDMA_TCD_ATTR_DSIZE_SHIFT)
#define EDMA_TCD_ATTR_DSIZE(n)            (((n) << EDMA_TCD_ATTR_DSIZE_SHIFT) & EDMA_TCD_ATTR_DSIZE_MASK)
#define EDMA_TCD_ATTR_DMOD_SHIFT          (3)       /* Bits 3-7: Destination Address Modulo (DMOD) */
#define EDMA_TCD_ATTR_DMOD_MASK           (0x1f << EDMA_TCD_ATTR_DMOD_SHIFT)
#define EDMA_TCD_ATTR_DMOD(n)             (((n) << EDMA_TCD_ATTR_DMOD_SHIFT) & EDMA_TCD_ATTR_DMOD_MASK)
#define EDMA_TCD_ATTR_SSIZE_SHIFT         (8)       /* Bits 8-10: Source Data Transfer Size (SSIZE) */
#define EDMA_TCD_ATTR_SSIZE_MASK          (0x07 << EDMA_TCD_ATTR_SSIZE_SHIFT)
#define EDMA_TCD_ATTR_SSIZE(n)            (((n) << EDMA_TCD_ATTR_SSIZE_SHIFT) & EDMA_TCD_ATTR_SSIZE_MASK)
#  define EDMA_TCD_ATTR_SSIZE_8BIT        (0x00 << EDMA_TCD_ATTR_SSIZE_SHIFT) /* 8-bit */
#  define EDMA_TCD_ATTR_SSIZE_16BIT       (0x01 << EDMA_TCD_ATTR_SSIZE_SHIFT) /* 16-bit */
#  define EDMA_TCD_ATTR_SSIZE_32BIT       (0x02 << EDMA_TCD_ATTR_SSIZE_SHIFT) /* 32-bit */
#  define EDMA_TCD_ATTR_SSIZE_64BIT       (0x03 << EDMA_TCD_ATTR_SSIZE_SHIFT) /* 64-bit */
#  define EDMA_TCD_ATTR_SSIZE_16BYTE      (0x04 << EDMA_TCD_ATTR_SSIZE_SHIFT) /* 16-byte */
#  define EDMA_TCD_ATTR_SSIZE_32BYTE      (0x05 << EDMA_TCD_ATTR_SSIZE_SHIFT) /* 32-byte */
#  define EDMA_TCD_ATTR_SSIZE_64BYTE      (0x06 << EDMA_TCD_ATTR_SSIZE_SHIFT) /* 64-byte */

#define EDMA_TCD_ATTR_SMOD_SHIFT          (11)      /* Bits 11-15: Source Address Modulo (SMOD) */
#define EDMA_TCD_ATTR_SMOD_MASK           (0x1f << EDMA_TCD_ATTR_SMOD_SHIFT)
#define EDMA_TCD_ATTR_SMOD(n)             (((n) << EDMA_TCD_ATTR_SMOD_SHIFT) & EDMA_TCD_ATTR_SMOD_MASK)

/* TCDn Transfer Size (TCDn_NBYTES) */

#define EDMA_TCD_NBYTES_SHIFT             (0)       /* Bits 0-29: Number of Bytes to Transfer per Service Request (NBYTES) */
#define EDMA_TCD_NBYTES_MASK              (0x3fffffff << EDMA_TCD_NBYTES_SHIFT)
#define EDMA_TCD_NBYTES_MASK_MLOFF        (0x03ff << EDMA_TCD_NBYTES_SHIFT)
#define EDMA_TCD_NBYTES_MLOFF_SHIFT       (10)      /* Bits 10-29: Minor Loop Offset (MLOFF) */
#define EDMA_TCD_NBYTES_MLOFF_MASK        (0x0fffff << EDMA_TCD_NBYTES_MLOFF_SHIFT)
#define EDMA_TCD_NBYTES_DMLOE             (1 << 30) /* Bit 30: Destination Minor Loop Offset Enable (DMLOE) */
#define EDMA_TCD_NBYTES_SMLOE             (1 << 31) /* Bit 31: Source Minor Loop Offset Enable (SMLOE) */

/* TCDn Last Source Address Adjustment / Store DADDR Address Register
 * (TCDn_SLAST_SDA)
 */

#define EDMA_TCD_SLAST_SDA_SHIFT          (0)       /* Bits 0-31: Last Source Address Adjustment / Store DADDR Address (SLAST_SDA) */
#define EDMA_TCD_SLAST_SDA_MASK           (0xffffffff << EDMA_TCD_SLAST_SDA_SHIFT)

/* TCDn Destination Address Register (TCDn_DADDR) */

#define EDMA_TCD_DADDR_SHIFT              (0)       /* Bits 0-31: Destination Address (DADDR) */
#define EDMA_TCD_DADDR_MASK               (0xffffffff << EDMA_TCD_DADDR_SHIFT)

/* TCDn Signed Destination Address Offset Register (TCDn_DOFF) */

#define EDMA_TCD_DOFF_SHIFT               (0)       /* Bits 0-15: Destination Address Signed Offset (DOFF) */
#define EDMA_TCD_DOFF_MASK                (0xffff << EDMA_TCD_DOFF_SHIFT)

/* TCDn Current Major Loop Count Register (TCDn_CITER) */

#define EDMA_TCD_CITER_SHIFT              (0)       /* Bits 0-14: Current Major Iteration Count (CITER) */
#define EDMA_TCD_CITER_MASK               (0x7fff << EDMA_TCD_CITER_SHIFT)
#define EDMA_TCD_CITER_MASK_ELINK         (0x01ff << EDMA_TCD_CITER_SHIFT)
#define EDMA_TCD_CITER_LINKCH_SHIFT       (9)       /* Bits 9-13: Minor Loop Link Channel Number (LINKCH) */
#define EDMA_TCD_CITER_LINKCH_MASK        (0x1f << EDMA_TCD_CITER_LINKCH_SHIFT)
#define EDMA_TCD_CITER_LINKCH(n)          (((n) << EDMA_TCD_CITER_LINKCH_SHIFT) & EDMA_TCD_CITER_LINKCH_SHIFT)
#define EDMA_TCD_CITER_ELINK              (1 << 15) /* Bit 15: Enable Link (ELINK) */

/* TCDn Last Destination Address Adjustment / Scatter Gather Address Register
 * (TCDn_DLAST_SGA)
 */

#define EDMA_TCD_DLAST_SGA_SHIFT          (0)       /* Bits 0-31: Last Destination Address Adjustment / Scatter Gather Address (DLAST_SGA) */
#define EDMA_TCD_DLAST_SGA_MASK           (0xffffffff << EDMA_TCD_DLAST_SGA_SHIFT)

/* TCDn Control and Status Register (TCDn_CSR) */

#define EDMA_TCD_CSR_START                (1 << 0)  /* Bit 0: Channel Start (START) */
#define EDMA_TCD_CSR_INTMAJOR             (1 << 1)  /* Bit 1: Enable Interrupt if Major count complete (INTMAJOR) */
#define EDMA_TCD_CSR_INTHALF              (1 << 2)  /* Bit 2: Enable Interrupt if Major Count Half-complete (INTHALF) */
#define EDMA_TCD_CSR_DREQ                 (1 << 3)  /* Bit 3: Disable Request (DREQ) */
#define EDMA_TCD_CSR_ESG                  (1 << 4)  /* Bit 4: Enable Scatter/Gather Processing (ESG) */
#define EDMA_TCD_CSR_MAJORELINK           (1 << 5)  /* Bit 5: Enable Link When Major Loop Complete (MAJORELINK) */
#define EDMA_TCD_CSR_EEOP                 (1 << 6)  /* Bit 6: Enable End-Of-Packet Processing (EEOP) */
#define EDMA_TCD_CSR_ESDA                 (1 << 7)  /* Bit 7: Enable Store Destination Address (ESDA) */
#define EDMA_TCD_CSR_MAJORLINKCH_SHIFT    (8)       /* Bits 8-12: Major Loop Link Channel Number (MAJORLINKCH) */
#define EDMA_TCD_CSR_MAJORLINKCH_MASK     (0x1f << EDMA_TCD_CSR_MAJORLINKCH_SHIFT)
#define EDMA_TCD_CSR_MAJORLINKCH(n)       (((n) << EDMA_TCD_CSR_MAJORLINKCH_SHIFT) & EDMA_TCD_CSR_MAJORLINKCH_MASK)
                                                    /* Bit 13: Reserved */
#define EDMA_TCD_CSR_BWC_SHIFT            (14)      /* Bits 14-15: Bandwidth Control (BWC) */
#define EDMA_TCD_CSR_BWC_MASK             (0x03 << EDMA_TCD_CSR_BWC_SHIFT)
#  define EDMA_TCD_CSR_BWC_NOSTALL        (0x00 << EDMA_TCD_CSR_BWC_SHIFT) /* No eDMA engine stalls */
#  define EDMA_TCD_CSR_BWC_HPE            (0x01 << EDMA_TCD_CSR_BWC_SHIFT) /* Enable eDMA master high-priority elevation (HPE) mode */
#  define EDMA_TCD_CSR_BWC_4CYCLES        (0x02 << EDMA_TCD_CSR_BWC_SHIFT) /* eDMA engine stalls for 4 cycles after each R/W */
#  define EDMA_TCD_CSR_BWC_8CYCLES        (0x03 << EDMA_TCD_CSR_BWC_SHIFT) /* eDMA engine stalls for 8 cycles after each R/W */

/* TCDn Beginning Major Loop Count Register (TCDn_BITER) */

#define EDMA_TCD_BITER_SHIFT              (0)       /* Bits 0-14: Starting Major Iteration Count (BITER) */
#define EDMA_TCD_BITER_MASK               (0x7fff << EDMA_TCD_BITER_SHIFT)
#define EDMA_TCD_BITER_MASK_ELINK         (0x01ff << EDMA_TCD_BITER_SHIFT)
#define EDMA_TCD_BITER_LINKCH_SHIFT       (9)       /* Bits 9-13: Link Channel Number (LINKCH) */
#define EDMA_TCD_BITER_LINKCH_MASK        (0x1f << EDMA_TCD_BITER_LINKCH_SHIFT)
#define EDMA_TCD_BITER_LINKCH(n)          (((n) << EDMA_TCD_BITER_LINKCH_SHIFT) & EDMA_TCD_BITER_LINKCH_MASK)
#define EDMA_TCD_BITER_ELINK              (1 << 15) /* Bit 15: Enable Link (ELINK) */

/* Amount of channels */

#define DMA3_CHANNEL_COUNT                (31)
#define DMA4_CHANNEL_COUNT                (64)
#define IMX9_EDMA_NCHANNELS               (DMA3_CHANNEL_COUNT + DMA4_CHANNEL_COUNT)

/* Amount of interrupt sources */
#ifdef CONFIG_ARCH_CHIP_IMX95_M7
#define DMA3_IRQ_COUNT                    (31) /* Error interrupt not counted */
#else
#define DMA3_IRQ_COUNT                    (32) /* Error interrupt not counted */
#endif
#define DMA4_IRQ_COUNT                    (32) /* Error interrupt not counted */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* In-memory representation of the 32-byte Transfer Control Descriptor
 * (TCD)
 */

struct imx9_edmatcd_s
{
  uint32_t   saddr;         /* Offset: 0x0000  TCD Source Address */
  uint16_t   soff;          /* Offset: 0x0004  TCD Signed Source Address Offset */
  uint16_t   attr;          /* Offset: 0x0006  TCD Transfer Attributes */
  uint32_t   nbytes;        /* Offset: 0x0008  TCD Signed Minor Loop Offset / Byte Count */
  uint32_t   slast;         /* Offset: 0x000c  TCD Last Source Address Adjustment */
  uint32_t   daddr;         /* Offset: 0x0010  TCD Destination Address */
  uint16_t   doff;          /* Offset: 0x0014  TCD Signed Destination Address Offset */
  uint16_t   citer;         /* Offset: 0x0016  TCD Current Minor Loop Link, Major Loop Count */
  uint32_t   dlastsga;      /* Offset: 0x0018  TCD Last Destination Address Adjustment/Scatter Gather Address */
  uint16_t   csr;           /* Offset: 0x001c  TCD Control and Status */
  uint16_t   biter;         /* Offset: 0x001e  TCD Beginning Minor Loop Link, Major Loop Count */
};

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_edma_tcdhasmux
 *
 * Description:
 *   Check if DMA TCD has TCD.MUX register.
 *
 * Input Parameters:
 *   dmabase - The eDMA base.
 *
 * Returned Value:
 *   true if TCD.MUX exists; false if not.
 *
 ****************************************************************************/

static inline bool imx9_edma_tcdhasmux(uintptr_t dmabase)
{
  /* Only eDMA5 has TCD.MUX register */

  return dmabase == IMX9_EDMA5_2_BASE ? true : false;
}

/****************************************************************************
 * Name: imx9_edma_choffset
 *
 * Description:
 *   Channel offset in global channel list for dma base.
 *
 * Input Parameters:
 *   base - The eDMA base.
 *
 * Returned Value:
 *   Channel offset.
 *
 ****************************************************************************/

static inline uint32_t imx9_edma_choffset(uintptr_t base)
{
  return base == IMX9_DMA3_BASE ? 0 : DMA3_CHANNEL_COUNT;
}

/****************************************************************************
 * Name: imx9_edma_chmax
 *
 * Description:
 *   Max channel in global channel list for dma base.
 *
 * Input Parameters:
 *   base - The eDMA base.
 *
 * Returned Value:
 *   Channel max.
 *
 ****************************************************************************/

static inline uint32_t imx9_edma_chmax(uintptr_t base)
{
  return base == IMX9_DMA3_BASE ? DMA3_CHANNEL_COUNT : IMX9_EDMA_NCHANNELS;
}

#endif /* __ARCH_ARM_SRC_IMX9_HARDWARE_IMX95_IMX95_EDMA_H */
