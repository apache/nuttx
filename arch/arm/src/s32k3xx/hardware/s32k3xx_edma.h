/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_edma.h
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

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_EDMA_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_EDMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/s32k3xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define S32K3XX_EDMA_NCHANNELS              32

/* eDMA Register Offsets ****************************************************/

#define S32K3XX_EDMA_CSR_OFFSET             (0x000000) /* Management Page Control Register (CSR) */
#define S32K3XX_EDMA_ES_OFFSET              (0x000004) /* Management Page Error Status Register (ES) */
#define S32K3XX_EDMA_INT_OFFSET             (0x000008) /* Management Page Interrupt Request Status Register (INT) */
#define S32K3XX_EDMA_HRS_OFFSET             (0x00000c) /* Management Page Hardware Request Status Register (HRS) */

#define S32K3XX_EDMA_CH_GRPRI_OFFSET(n)     (0x000100 + ((n) << 2)) /* Channel n Arbitration Group Register (CHn_GRPRI) */

/* eDMA Register Addresses **************************************************/

#define S32K3XX_EDMA_CSR                    (S32K3XX_EDMA_BASE + S32K3XX_EDMA_CSR_OFFSET)
#define S32K3XX_EDMA_ES                     (S32K3XX_EDMA_BASE + S32K3XX_EDMA_ES_OFFSET)
#define S32K3XX_EDMA_INT                    (S32K3XX_EDMA_BASE + S32K3XX_EDMA_INT_OFFSET)
#define S32K3XX_EDMA_HRS                    (S32K3XX_EDMA_BASE + S32K3XX_EDMA_HRS_OFFSET)
#define S32K3XX_EDMA_CH_GRPRI(n)            (S32K3XX_EDMA_BASE + S32K3XX_EDMA_CH_GRPRI_OFFSET(n))

/* eDMA Transfer Control Descriptor (TCD) Register Offsets ******************/

#define S32K3XX_EDMA_CH_CSR_OFFSET         (0x000000)  /* Channel Control and Status Register (CH0_CSR) */
#define S32K3XX_EDMA_CH_ES_OFFSET          (0x000004)  /* Channel Error Status Register (CH0_ES) */
#define S32K3XX_EDMA_CH_INT_OFFSET         (0x000008)  /* Channel Interrupt Status Register (CH0_INT) */
#define S32K3XX_EDMA_CH_SBR_OFFSET         (0x00000c)  /* Channel System Bus Register (CH0_SBR) */
#define S32K3XX_EDMA_CH_PRI_OFFSET         (0x000010)  /* Channel Priority Register (CH0_PRI) */
#define S32K3XX_EDMA_TCD_SADDR_OFFSET       (0x000020) /* TCD Source Address Register (TCD0_SADDR) */
#define S32K3XX_EDMA_TCD_SOFF_OFFSET        (0x000024) /* TCD Signed Source Address Offset Register (TCD0_SOFF) */
#define S32K3XX_EDMA_TCD_ATTR_OFFSET        (0x000026) /* TCD Transfer Attributes (TCD0_ATTR) */
#define S32K3XX_EDMA_TCD_NBYTES_OFFSET      (0x000028) /* TCD Transfer Size (TCD0_NBYTES) */
#define S32K3XX_EDMA_TCD_SLAST_SDA_OFFSET   (0x00002c) /* TCD Last Source Address Adjustment / Store DADDR Address Register (TCD0_SLAST_SDA) */
#define S32K3XX_EDMA_TCD_DADDR_OFFSET       (0x000030) /* TCD Destination Address Register (TCD0_DADDR) */
#define S32K3XX_EDMA_TCD_DOFF_OFFSET        (0x000034) /* TCD Signed Destination Address Offset Register (TCD0_DOFF) */
#define S32K3XX_EDMA_TCD_CITER_OFFSET       (0x000036) /* TCD Current Major Loop Count Register (TCD0_CITER) */
#define S32K3XX_EDMA_TCD_DLAST_SGA_OFFSET   (0x000038) /* TCD Last Destination Address Adjustment / Scatter Gather Address Register (TCD0_DLAST_SGA)*/
#define S32K3XX_EDMA_TCD_CSR_OFFSET         (0x00003c) /* TCD Control and Status Register (TCD0_CSR) */
#define S32K3XX_EDMA_TCD_BITER_OFFSET       (0x00003e) /* TCD Beginning Major Loop Count Register (TCD0_BITER) */

#define S32K3XX_EDMA_CH0_CSR_OFFSET         (0x000000) /* Channel 0 Control and Status Register (CH0_CSR) */
#define S32K3XX_EDMA_CH0_ES_OFFSET          (0x000004) /* Channel 0 Error Status Register (CH0_ES) */
#define S32K3XX_EDMA_CH0_INT_OFFSET         (0x000008) /* Channel 0 Interrupt Status Register (CH0_INT) */
#define S32K3XX_EDMA_CH0_SBR_OFFSET         (0x00000c) /* Channel 0 System Bus Register (CH0_SBR) */
#define S32K3XX_EDMA_CH0_PRI_OFFSET         (0x000010) /* Channel 0 Priority Register (CH0_PRI) */
#define S32K3XX_EDMA_TCD0_SADDR_OFFSET      (0x000020) /* TCD0 Source Address Register (TCD0_SADDR) */
#define S32K3XX_EDMA_TCD0_SOFF_OFFSET       (0x000024) /* TCD0 Signed Source Address Offset Register (TCD0_SOFF) */
#define S32K3XX_EDMA_TCD0_ATTR_OFFSET       (0x000026) /* TCD0 Transfer Attributes (TCD0_ATTR) */
#define S32K3XX_EDMA_TCD0_NBYTES_OFFSET     (0x000028) /* TCD0 Transfer Size (TCD0_NBYTES) */
#define S32K3XX_EDMA_TCD0_SLAST_SDA_OFFSET  (0x00002c) /* TCD0 Last Source Address Adjustment / Store DADDR Address Register (TCD0_SLAST_SDA) */
#define S32K3XX_EDMA_TCD0_DADDR_OFFSET      (0x000030) /* TCD0 Destination Address Register (TCD0_DADDR) */
#define S32K3XX_EDMA_TCD0_DOFF_OFFSET       (0x000034) /* TCD0 Signed Destination Address Offset Register (TCD0_DOFF) */
#define S32K3XX_EDMA_TCD0_CITER_OFFSET      (0x000036) /* TCD0 Current Major Loop Count Register (TCD0_CITER) */
#define S32K3XX_EDMA_TCD0_DLAST_SGA_OFFSET  (0x000038) /* TCD0 Last Destination Address Adjustment / Scatter Gather Address Register (TCD0_DLAST_SGA)*/
#define S32K3XX_EDMA_TCD0_CSR_OFFSET        (0x00003c) /* TCD0 Control and Status Register (TCD0_CSR) */
#define S32K3XX_EDMA_TCD0_BITER_OFFSET      (0x00003e) /* TCD0 Beginning Major Loop Count Register (TCD0_BITER) */

#define S32K3XX_EDMA_CH1_CSR_OFFSET         (0x004000) /* Channel 1 Control and Status Register (CH1_CSR) */
#define S32K3XX_EDMA_CH1_ES_OFFSET          (0x004004) /* Channel 1 Error Status Register (CH1_ES) */
#define S32K3XX_EDMA_CH1_INT_OFFSET         (0x004008) /* Channel 1 Interrupt Status Register (CH1_INT) */
#define S32K3XX_EDMA_CH1_SBR_OFFSET         (0x00400c) /* Channel 1 System Bus Register (CH1_SBR) */
#define S32K3XX_EDMA_CH1_PRI_OFFSET         (0x004010) /* Channel 1 Priority Register (CH1_PRI) */
#define S32K3XX_EDMA_TCD1_SADDR_OFFSET      (0x004020) /* TCD1 Source Address Register (TCD1_SADDR) */
#define S32K3XX_EDMA_TCD1_SOFF_OFFSET       (0x004024) /* TCD1 Signed Source Address Offset Register (TCD1_SOFF) */
#define S32K3XX_EDMA_TCD1_ATTR_OFFSET       (0x004026) /* TCD1 Transfer Attributes (TCD1_ATTR) */
#define S32K3XX_EDMA_TCD1_NBYTES_OFFSET     (0x004028) /* TCD1 Transfer Size (TCD1_NBYTES) */
#define S32K3XX_EDMA_TCD1_SLAST_SDA_OFFSET  (0x00402c) /* TCD1 Last Source Address Adjustment / Store DADDR Address Register (TCD1_SLAST_SDA) */
#define S32K3XX_EDMA_TCD1_DADDR_OFFSET      (0x004030) /* TCD1 Destination Address Register (TCD1_DADDR) */
#define S32K3XX_EDMA_TCD1_DOFF_OFFSET       (0x004034) /* TCD1 Signed Destination Address Offset Register (TCD1_DOFF) */
#define S32K3XX_EDMA_TCD1_CITER_OFFSET      (0x004036) /* TCD1 Current Major Loop Count Register (TCD1_CITER) */
#define S32K3XX_EDMA_TCD1_DLAST_SGA_OFFSET  (0x004038) /* TCD1 Last Destination Address Adjustment / Scatter Gather Address Register (TCD1_DLAST_SGA)*/
#define S32K3XX_EDMA_TCD1_CSR_OFFSET        (0x00403c) /* TCD1 Control and Status Register (TCD1_CSR) */
#define S32K3XX_EDMA_TCD1_BITER_OFFSET      (0x00403e) /* TCD1 Beginning Major Loop Count Register (TCD1_BITER) */

#define S32K3XX_EDMA_CH2_CSR_OFFSET         (0x008000) /* Channel 2 Control and Status Register (CH2_CSR) */
#define S32K3XX_EDMA_CH2_ES_OFFSET          (0x008004) /* Channel 2 Error Status Register (CH2_ES) */
#define S32K3XX_EDMA_CH2_INT_OFFSET         (0x008008) /* Channel 2 Interrupt Status Register (CH2_INT) */
#define S32K3XX_EDMA_CH2_SBR_OFFSET         (0x00800c) /* Channel 2 System Bus Register (CH2_SBR) */
#define S32K3XX_EDMA_CH2_PRI_OFFSET         (0x008010) /* Channel 2 Priority Register (CH2_PRI) */
#define S32K3XX_EDMA_TCD2_SADDR_OFFSET      (0x008020) /* TCD2 Source Address Register (TCD2_SADDR) */
#define S32K3XX_EDMA_TCD2_SOFF_OFFSET       (0x008024) /* TCD2 Signed Source Address Offset Register (TCD2_SOFF) */
#define S32K3XX_EDMA_TCD2_ATTR_OFFSET       (0x008026) /* TCD2 Transfer Attributes (TCD2_ATTR) */
#define S32K3XX_EDMA_TCD2_NBYTES_OFFSET     (0x008028) /* TCD2 Transfer Size (TCD2_NBYTES) */
#define S32K3XX_EDMA_TCD2_SLAST_SDA_OFFSET  (0x00802c) /* TCD2 Last Source Address Adjustment / Store DADDR Address Register (TCD2_SLAST_SDA) */
#define S32K3XX_EDMA_TCD2_DADDR_OFFSET      (0x008030) /* TCD2 Destination Address Register (TCD2_DADDR) */
#define S32K3XX_EDMA_TCD2_DOFF_OFFSET       (0x008034) /* TCD2 Signed Destination Address Offset Register (TCD2_DOFF) */
#define S32K3XX_EDMA_TCD2_CITER_OFFSET      (0x008036) /* TCD2 Current Major Loop Count Register (TCD2_CITER) */
#define S32K3XX_EDMA_TCD2_DLAST_SGA_OFFSET  (0x008038) /* TCD2 Last Destination Address Adjustment / Scatter Gather Address Register (TCD2_DLAST_SGA)*/
#define S32K3XX_EDMA_TCD2_CSR_OFFSET        (0x00803c) /* TCD2 Control and Status Register (TCD2_CSR) */
#define S32K3XX_EDMA_TCD2_BITER_OFFSET      (0x00803e) /* TCD2 Beginning Major Loop Count Register (TCD2_BITER) */

#define S32K3XX_EDMA_CH3_CSR_OFFSET         (0x00c000) /* Channel 3 Control and Status Register (CH3_CSR) */
#define S32K3XX_EDMA_CH3_ES_OFFSET          (0x00c004) /* Channel 3 Error Status Register (CH3_ES) */
#define S32K3XX_EDMA_CH3_INT_OFFSET         (0x00c008) /* Channel 3 Interrupt Status Register (CH3_INT) */
#define S32K3XX_EDMA_CH3_SBR_OFFSET         (0x00c00c) /* Channel 3 System Bus Register (CH3_SBR) */
#define S32K3XX_EDMA_CH3_PRI_OFFSET         (0x00c010) /* Channel 3 Priority Register (CH3_PRI) */
#define S32K3XX_EDMA_TCD3_SADDR_OFFSET      (0x00c020) /* TCD3 Source Address Register (TCD3_SADDR) */
#define S32K3XX_EDMA_TCD3_SOFF_OFFSET       (0x00c024) /* TCD3 Signed Source Address Offset Register (TCD3_SOFF) */
#define S32K3XX_EDMA_TCD3_ATTR_OFFSET       (0x00c026) /* TCD3 Transfer Attributes (TCD3_ATTR) */
#define S32K3XX_EDMA_TCD3_NBYTES_OFFSET     (0x00c028) /* TCD3 Transfer Size (TCD3_NBYTES) */
#define S32K3XX_EDMA_TCD3_SLAST_SDA_OFFSET  (0x00c02c) /* TCD3 Last Source Address Adjustment / Store DADDR Address Register (TCD3_SLAST_SDA) */
#define S32K3XX_EDMA_TCD3_DADDR_OFFSET      (0x00c030) /* TCD3 Destination Address Register (TCD3_DADDR) */
#define S32K3XX_EDMA_TCD3_DOFF_OFFSET       (0x00c034) /* TCD3 Signed Destination Address Offset Register (TCD3_DOFF) */
#define S32K3XX_EDMA_TCD3_CITER_OFFSET      (0x00c036) /* TCD3 Current Major Loop Count Register (TCD3_CITER) */
#define S32K3XX_EDMA_TCD3_DLAST_SGA_OFFSET  (0x00c038) /* TCD3 Last Destination Address Adjustment / Scatter Gather Address Register (TCD3_DLAST_SGA)*/
#define S32K3XX_EDMA_TCD3_CSR_OFFSET        (0x00c03c) /* TCD3 Control and Status Register (TCD3_CSR) */
#define S32K3XX_EDMA_TCD3_BITER_OFFSET      (0x00c03e) /* TCD3 Beginning Major Loop Count Register (TCD3_BITER) */

#define S32K3XX_EDMA_CH4_CSR_OFFSET         (0x010000) /* Channel 4 Control and Status Register (CH4_CSR) */
#define S32K3XX_EDMA_CH4_ES_OFFSET          (0x010004) /* Channel 4 Error Status Register (CH4_ES) */
#define S32K3XX_EDMA_CH4_INT_OFFSET         (0x010008) /* Channel 4 Interrupt Status Register (CH4_INT) */
#define S32K3XX_EDMA_CH4_SBR_OFFSET         (0x01000c) /* Channel 4 System Bus Register (CH4_SBR) */
#define S32K3XX_EDMA_CH4_PRI_OFFSET         (0x010010) /* Channel 4 Priority Register (CH4_PRI) */
#define S32K3XX_EDMA_TCD4_SADDR_OFFSET      (0x010020) /* TCD4 Source Address Register (TCD4_SADDR) */
#define S32K3XX_EDMA_TCD4_SOFF_OFFSET       (0x010024) /* TCD4 Signed Source Address Offset Register (TCD4_SOFF) */
#define S32K3XX_EDMA_TCD4_ATTR_OFFSET       (0x010026) /* TCD4 Transfer Attributes (TCD4_ATTR) */
#define S32K3XX_EDMA_TCD4_NBYTES_OFFSET     (0x010028) /* TCD4 Transfer Size (TCD4_NBYTES) */
#define S32K3XX_EDMA_TCD4_SLAST_SDA_OFFSET  (0x01002c) /* TCD4 Last Source Address Adjustment / Store DADDR Address Register (TCD4_SLAST_SDA) */
#define S32K3XX_EDMA_TCD4_DADDR_OFFSET      (0x010030) /* TCD4 Destination Address Register (TCD4_DADDR) */
#define S32K3XX_EDMA_TCD4_DOFF_OFFSET       (0x010034) /* TCD4 Signed Destination Address Offset Register (TCD4_DOFF) */
#define S32K3XX_EDMA_TCD4_CITER_OFFSET      (0x010036) /* TCD4 Current Major Loop Count Register (TCD4_CITER) */
#define S32K3XX_EDMA_TCD4_DLAST_SGA_OFFSET  (0x010038) /* TCD4 Last Destination Address Adjustment / Scatter Gather Address Register (TCD4_DLAST_SGA)*/
#define S32K3XX_EDMA_TCD4_CSR_OFFSET        (0x01003c) /* TCD4 Control and Status Register (TCD4_CSR) */
#define S32K3XX_EDMA_TCD4_BITER_OFFSET      (0x01003e) /* TCD4 Beginning Major Loop Count Register (TCD4_BITER) */

#define S32K3XX_EDMA_CH5_CSR_OFFSET         (0x014000) /* Channel 5 Control and Status Register (CH5_CSR) */
#define S32K3XX_EDMA_CH5_ES_OFFSET          (0x014004) /* Channel 5 Error Status Register (CH5_ES) */
#define S32K3XX_EDMA_CH5_INT_OFFSET         (0x014008) /* Channel 5 Interrupt Status Register (CH5_INT) */
#define S32K3XX_EDMA_CH5_SBR_OFFSET         (0x01400c) /* Channel 5 System Bus Register (CH5_SBR) */
#define S32K3XX_EDMA_CH5_PRI_OFFSET         (0x014010) /* Channel 5 Priority Register (CH5_PRI) */
#define S32K3XX_EDMA_TCD5_SADDR_OFFSET      (0x014020) /* TCD5 Source Address Register (TCD5_SADDR) */
#define S32K3XX_EDMA_TCD5_SOFF_OFFSET       (0x014024) /* TCD5 Signed Source Address Offset Register (TCD5_SOFF) */
#define S32K3XX_EDMA_TCD5_ATTR_OFFSET       (0x014026) /* TCD5 Transfer Attributes (TCD5_ATTR) */
#define S32K3XX_EDMA_TCD5_NBYTES_OFFSET     (0x014028) /* TCD5 Transfer Size (TCD5_NBYTES) */
#define S32K3XX_EDMA_TCD5_SLAST_SDA_OFFSET  (0x01402c) /* TCD5 Last Source Address Adjustment / Store DADDR Address Register (TCD5_SLAST_SDA) */
#define S32K3XX_EDMA_TCD5_DADDR_OFFSET      (0x014030) /* TCD5 Destination Address Register (TCD5_DADDR) */
#define S32K3XX_EDMA_TCD5_DOFF_OFFSET       (0x014034) /* TCD5 Signed Destination Address Offset Register (TCD5_DOFF) */
#define S32K3XX_EDMA_TCD5_CITER_OFFSET      (0x014036) /* TCD5 Current Major Loop Count Register (TCD5_CITER) */
#define S32K3XX_EDMA_TCD5_DLAST_SGA_OFFSET  (0x014038) /* TCD5 Last Destination Address Adjustment / Scatter Gather Address Register (TCD5_DLAST_SGA)*/
#define S32K3XX_EDMA_TCD5_CSR_OFFSET        (0x01403c) /* TCD5 Control and Status Register (TCD5_CSR) */
#define S32K3XX_EDMA_TCD5_BITER_OFFSET      (0x01403e) /* TCD5 Beginning Major Loop Count Register (TCD5_BITER) */

#define S32K3XX_EDMA_CH6_CSR_OFFSET         (0x018000) /* Channel 6 Control and Status Register (CH6_CSR) */
#define S32K3XX_EDMA_CH6_ES_OFFSET          (0x018004) /* Channel 6 Error Status Register (CH6_ES) */
#define S32K3XX_EDMA_CH6_INT_OFFSET         (0x018008) /* Channel 6 Interrupt Status Register (CH6_INT) */
#define S32K3XX_EDMA_CH6_SBR_OFFSET         (0x01800c) /* Channel 6 System Bus Register (CH6_SBR) */
#define S32K3XX_EDMA_CH6_PRI_OFFSET         (0x018010) /* Channel 6 Priority Register (CH6_PRI) */
#define S32K3XX_EDMA_TCD6_SADDR_OFFSET      (0x018020) /* TCD6 Source Address Register (TCD6_SADDR) */
#define S32K3XX_EDMA_TCD6_SOFF_OFFSET       (0x018024) /* TCD6 Signed Source Address Offset Register (TCD6_SOFF) */
#define S32K3XX_EDMA_TCD6_ATTR_OFFSET       (0x018026) /* TCD6 Transfer Attributes (TCD6_ATTR) */
#define S32K3XX_EDMA_TCD6_NBYTES_OFFSET     (0x018028) /* TCD6 Transfer Size (TCD6_NBYTES) */
#define S32K3XX_EDMA_TCD6_SLAST_SDA_OFFSET  (0x01802c) /* TCD6 Last Source Address Adjustment / Store DADDR Address Register (TCD6_SLAST_SDA) */
#define S32K3XX_EDMA_TCD6_DADDR_OFFSET      (0x018030) /* TCD6 Destination Address Register (TCD6_DADDR) */
#define S32K3XX_EDMA_TCD6_DOFF_OFFSET       (0x018034) /* TCD6 Signed Destination Address Offset Register (TCD6_DOFF) */
#define S32K3XX_EDMA_TCD6_CITER_OFFSET      (0x018036) /* TCD6 Current Major Loop Count Register (TCD6_CITER) */
#define S32K3XX_EDMA_TCD6_DLAST_SGA_OFFSET  (0x018038) /* TCD6 Last Destination Address Adjustment / Scatter Gather Address Register (TCD6_DLAST_SGA)*/
#define S32K3XX_EDMA_TCD6_CSR_OFFSET        (0x01803c) /* TCD6 Control and Status Register (TCD6_CSR) */
#define S32K3XX_EDMA_TCD6_BITER_OFFSET      (0x01803e) /* TCD6 Beginning Major Loop Count Register (TCD6_BITER) */

#define S32K3XX_EDMA_CH7_CSR_OFFSET         (0x01c000) /* Channel 7 Control and Status Register (CH7_CSR) */
#define S32K3XX_EDMA_CH7_ES_OFFSET          (0x01c004) /* Channel 7 Error Status Register (CH7_ES) */
#define S32K3XX_EDMA_CH7_INT_OFFSET         (0x01c008) /* Channel 7 Interrupt Status Register (CH7_INT) */
#define S32K3XX_EDMA_CH7_SBR_OFFSET         (0x01c00c) /* Channel 7 System Bus Register (CH7_SBR) */
#define S32K3XX_EDMA_CH7_PRI_OFFSET         (0x01c010) /* Channel 7 Priority Register (CH7_PRI) */
#define S32K3XX_EDMA_TCD7_SADDR_OFFSET      (0x01c020) /* TCD7 Source Address Register (TCD7_SADDR) */
#define S32K3XX_EDMA_TCD7_SOFF_OFFSET       (0x01c024) /* TCD7 Signed Source Address Offset Register (TCD7_SOFF) */
#define S32K3XX_EDMA_TCD7_ATTR_OFFSET       (0x01c026) /* TCD7 Transfer Attributes (TCD7_ATTR) */
#define S32K3XX_EDMA_TCD7_NBYTES_OFFSET     (0x01c028) /* TCD7 Transfer Size (TCD7_NBYTES) */
#define S32K3XX_EDMA_TCD7_SLAST_SDA_OFFSET  (0x01c02c) /* TCD7 Last Source Address Adjustment / Store DADDR Address Register (TCD7_SLAST_SDA) */
#define S32K3XX_EDMA_TCD7_DADDR_OFFSET      (0x01c030) /* TCD7 Destination Address Register (TCD7_DADDR) */
#define S32K3XX_EDMA_TCD7_DOFF_OFFSET       (0x01c034) /* TCD7 Signed Destination Address Offset Register (TCD7_DOFF) */
#define S32K3XX_EDMA_TCD7_CITER_OFFSET      (0x01c036) /* TCD7 Current Major Loop Count Register (TCD7_CITER) */
#define S32K3XX_EDMA_TCD7_DLAST_SGA_OFFSET  (0x01c038) /* TCD7 Last Destination Address Adjustment / Scatter Gather Address Register (TCD7_DLAST_SGA)*/
#define S32K3XX_EDMA_TCD7_CSR_OFFSET        (0x01c03c) /* TCD7 Control and Status Register (TCD7_CSR) */
#define S32K3XX_EDMA_TCD7_BITER_OFFSET      (0x01c03e) /* TCD7 Beginning Major Loop Count Register (TCD7_BITER) */

#define S32K3XX_EDMA_CH8_CSR_OFFSET         (0x020000) /* Channel 8 Control and Status Register (CH8_CSR) */
#define S32K3XX_EDMA_CH8_ES_OFFSET          (0x020004) /* Channel 8 Error Status Register (CH8_ES) */
#define S32K3XX_EDMA_CH8_INT_OFFSET         (0x020008) /* Channel 8 Interrupt Status Register (CH8_INT) */
#define S32K3XX_EDMA_CH8_SBR_OFFSET         (0x02000c) /* Channel 8 System Bus Register (CH8_SBR) */
#define S32K3XX_EDMA_CH8_PRI_OFFSET         (0x020010) /* Channel 8 Priority Register (CH8_PRI) */
#define S32K3XX_EDMA_TCD8_SADDR_OFFSET      (0x020020) /* TCD8 Source Address Register (TCD8_SADDR) */
#define S32K3XX_EDMA_TCD8_SOFF_OFFSET       (0x020024) /* TCD8 Signed Source Address Offset Register (TCD8_SOFF) */
#define S32K3XX_EDMA_TCD8_ATTR_OFFSET       (0x020026) /* TCD8 Transfer Attributes (TCD8_ATTR) */
#define S32K3XX_EDMA_TCD8_NBYTES_OFFSET     (0x020028) /* TCD8 Transfer Size (TCD8_NBYTES) */
#define S32K3XX_EDMA_TCD8_SLAST_SDA_OFFSET  (0x02002c) /* TCD8 Last Source Address Adjustment / Store DADDR Address Register (TCD8_SLAST_SDA) */
#define S32K3XX_EDMA_TCD8_DADDR_OFFSET      (0x020030) /* TCD8 Destination Address Register (TCD8_DADDR) */
#define S32K3XX_EDMA_TCD8_DOFF_OFFSET       (0x020034) /* TCD8 Signed Destination Address Offset Register (TCD8_DOFF) */
#define S32K3XX_EDMA_TCD8_CITER_OFFSET      (0x020036) /* TCD8 Current Major Loop Count Register (TCD8_CITER) */
#define S32K3XX_EDMA_TCD8_DLAST_SGA_OFFSET  (0x020038) /* TCD8 Last Destination Address Adjustment / Scatter Gather Address Register (TCD8_DLAST_SGA)*/
#define S32K3XX_EDMA_TCD8_CSR_OFFSET        (0x02003c) /* TCD8 Control and Status Register (TCD8_CSR) */
#define S32K3XX_EDMA_TCD8_BITER_OFFSET      (0x02003e) /* TCD8 Beginning Major Loop Count Register (TCD8_BITER) */

#define S32K3XX_EDMA_CH9_CSR_OFFSET         (0x024000) /* Channel 9 Control and Status Register (CH9_CSR) */
#define S32K3XX_EDMA_CH9_ES_OFFSET          (0x024004) /* Channel 9 Error Status Register (CH9_ES) */
#define S32K3XX_EDMA_CH9_INT_OFFSET         (0x024008) /* Channel 9 Interrupt Status Register (CH9_INT) */
#define S32K3XX_EDMA_CH9_SBR_OFFSET         (0x02400c) /* Channel 9 System Bus Register (CH9_SBR) */
#define S32K3XX_EDMA_CH9_PRI_OFFSET         (0x024010) /* Channel 9 Priority Register (CH9_PRI) */
#define S32K3XX_EDMA_TCD9_SADDR_OFFSET      (0x024020) /* TCD9 Source Address Register (TCD9_SADDR) */
#define S32K3XX_EDMA_TCD9_SOFF_OFFSET       (0x024024) /* TCD9 Signed Source Address Offset Register (TCD9_SOFF) */
#define S32K3XX_EDMA_TCD9_ATTR_OFFSET       (0x024026) /* TCD9 Transfer Attributes (TCD9_ATTR) */
#define S32K3XX_EDMA_TCD9_NBYTES_OFFSET     (0x024028) /* TCD9 Transfer Size (TCD9_NBYTES) */
#define S32K3XX_EDMA_TCD9_SLAST_SDA_OFFSET  (0x02402c) /* TCD9 Last Source Address Adjustment / Store DADDR Address Register (TCD9_SLAST_SDA) */
#define S32K3XX_EDMA_TCD9_DADDR_OFFSET      (0x024030) /* TCD9 Destination Address Register (TCD9_DADDR) */
#define S32K3XX_EDMA_TCD9_DOFF_OFFSET       (0x024034) /* TCD9 Signed Destination Address Offset Register (TCD9_DOFF) */
#define S32K3XX_EDMA_TCD9_CITER_OFFSET      (0x024036) /* TCD9 Current Major Loop Count Register (TCD9_CITER) */
#define S32K3XX_EDMA_TCD9_DLAST_SGA_OFFSET  (0x024038) /* TCD9 Last Destination Address Adjustment / Scatter Gather Address Register (TCD9_DLAST_SGA)*/
#define S32K3XX_EDMA_TCD9_CSR_OFFSET        (0x02403c) /* TCD9 Control and Status Register (TCD9_CSR) */
#define S32K3XX_EDMA_TCD9_BITER_OFFSET      (0x02403e) /* TCD9 Beginning Major Loop Count Register (TCD9_BITER) */

#define S32K3XX_EDMA_CH10_CSR_OFFSET        (0x028000) /* Channel 10 Control and Status Register (CH10_CSR) */
#define S32K3XX_EDMA_CH10_ES_OFFSET         (0x028004) /* Channel 10 Error Status Register (CH10_ES) */
#define S32K3XX_EDMA_CH10_INT_OFFSET        (0x028008) /* Channel 10 Interrupt Status Register (CH10_INT) */
#define S32K3XX_EDMA_CH10_SBR_OFFSET        (0x02800c) /* Channel 10 System Bus Register (CH10_SBR) */
#define S32K3XX_EDMA_CH10_PRI_OFFSET        (0x028010) /* Channel 10 Priority Register (CH10_PRI) */
#define S32K3XX_EDMA_TCD10_SADDR_OFFSET     (0x028020) /* TCD10 Source Address Register (TCD10_SADDR) */
#define S32K3XX_EDMA_TCD10_SOFF_OFFSET      (0x028024) /* TCD10 Signed Source Address Offset Register (TCD10_SOFF) */
#define S32K3XX_EDMA_TCD10_ATTR_OFFSET      (0x028026) /* TCD10 Transfer Attributes (TCD10_ATTR) */
#define S32K3XX_EDMA_TCD10_NBYTES_OFFSET    (0x028028) /* TCD10 Transfer Size (TCD10_NBYTES) */
#define S32K3XX_EDMA_TCD10_SLAST_SDA_OFFSET (0x02802c) /* TCD10 Last Source Address Adjustment / Store DADDR Address Register (TCD10_SLAST_SDA) */
#define S32K3XX_EDMA_TCD10_DADDR_OFFSET     (0x028030) /* TCD10 Destination Address Register (TCD10_DADDR) */
#define S32K3XX_EDMA_TCD10_DOFF_OFFSET      (0x028034) /* TCD10 Signed Destination Address Offset Register (TCD10_DOFF) */
#define S32K3XX_EDMA_TCD10_CITER_OFFSET     (0x028036) /* TCD10 Current Major Loop Count Register (TCD10_CITER) */
#define S32K3XX_EDMA_TCD10_DLAST_SGA_OFFSET (0x028038) /* TCD10 Last Destination Address Adjustment / Scatter Gather Address Register (TCD10_DLAST_SGA)*/
#define S32K3XX_EDMA_TCD10_CSR_OFFSET       (0x02803c) /* TCD10 Control and Status Register (TCD10_CSR) */
#define S32K3XX_EDMA_TCD10_BITER_OFFSET     (0x02803e) /* TCD10 Beginning Major Loop Count Register (TCD10_BITER) */
#define S32K3XX_EDMA_CH11_CSR_OFFSET        (0x02c000) /* Channel 11 Control and Status Register (CH11_CSR) */
#define S32K3XX_EDMA_CH11_ES_OFFSET         (0x02c004) /* Channel 11 Error Status Register (CH11_ES) */
#define S32K3XX_EDMA_CH11_INT_OFFSET        (0x02c008) /* Channel 11 Interrupt Status Register (CH11_INT) */
#define S32K3XX_EDMA_CH11_SBR_OFFSET        (0x02c00c) /* Channel 11 System Bus Register (CH11_SBR) */
#define S32K3XX_EDMA_CH11_PRI_OFFSET        (0x02c010) /* Channel 11 Priority Register (CH11_PRI) */
#define S32K3XX_EDMA_TCD11_SADDR_OFFSET     (0x02c020) /* TCD11 Source Address Register (TCD11_SADDR) */
#define S32K3XX_EDMA_TCD11_SOFF_OFFSET      (0x02c024) /* TCD11 Signed Source Address Offset Register (TCD11_SOFF) */
#define S32K3XX_EDMA_TCD11_ATTR_OFFSET      (0x02c026) /* TCD11 Transfer Attributes (TCD11_ATTR) */
#define S32K3XX_EDMA_TCD11_NBYTES_OFFSET    (0x02c028) /* TCD11 Transfer Size (TCD11_NBYTES) */
#define S32K3XX_EDMA_TCD11_SLAST_SDA_OFFSET (0x02c02c) /* TCD11 Last Source Address Adjustment / Store DADDR Address Register (TCD11_SLAST_SDA) */
#define S32K3XX_EDMA_TCD11_DADDR_OFFSET     (0x02c030) /* TCD11 Destination Address Register (TCD11_DADDR) */
#define S32K3XX_EDMA_TCD11_DOFF_OFFSET      (0x02c034) /* TCD11 Signed Destination Address Offset Register (TCD11_DOFF) */
#define S32K3XX_EDMA_TCD11_CITER_OFFSET     (0x02c036) /* TCD11 Current Major Loop Count Register (TCD11_CITER) */
#define S32K3XX_EDMA_TCD11_DLAST_SGA_OFFSET (0x02c038) /* TCD11 Last Destination Address Adjustment / Scatter Gather Address Register (TCD11_DLAST_SGA)*/
#define S32K3XX_EDMA_TCD11_CSR_OFFSET       (0x02c03c) /* TCD11 Control and Status Register (TCD11_CSR) */
#define S32K3XX_EDMA_TCD11_BITER_OFFSET     (0x02c03e) /* TCD11 Beginning Major Loop Count Register (TCD11_BITER) */

#define S32K3XX_EDMA_CH12_CSR_OFFSET        (0x200000) /* Channel 12 Control and Status Register (CH12_CSR) */
#define S32K3XX_EDMA_CH12_ES_OFFSET         (0x200004) /* Channel 12 Error Status Register (CH12_ES) */
#define S32K3XX_EDMA_CH12_INT_OFFSET        (0x200008) /* Channel 12 Interrupt Status Register (CH12_INT) */
#define S32K3XX_EDMA_CH12_SBR_OFFSET        (0x20000c) /* Channel 12 System Bus Register (CH12_SBR) */
#define S32K3XX_EDMA_CH12_PRI_OFFSET        (0x200010) /* Channel 12 Priority Register (CH12_PRI) */
#define S32K3XX_EDMA_TCD12_SADDR_OFFSET     (0x200020) /* TCD12 Source Address Register (TCD12_SADDR) */
#define S32K3XX_EDMA_TCD12_SOFF_OFFSET      (0x200024) /* TCD12 Signed Source Address Offset Register (TCD12_SOFF) */
#define S32K3XX_EDMA_TCD12_ATTR_OFFSET      (0x200026) /* TCD12 Transfer Attributes (TCD12_ATTR) */
#define S32K3XX_EDMA_TCD12_NBYTES_OFFSET    (0x200028) /* TCD12 Transfer Size (TCD12_NBYTES) */
#define S32K3XX_EDMA_TCD12_SLAST_SDA_OFFSET (0x20002c) /* TCD12 Last Source Address Adjustment / Store DADDR Address Register (TCD12_SLAST_SDA) */
#define S32K3XX_EDMA_TCD12_DADDR_OFFSET     (0x200030) /* TCD12 Destination Address Register (TCD12_DADDR) */
#define S32K3XX_EDMA_TCD12_DOFF_OFFSET      (0x200034) /* TCD12 Signed Destination Address Offset Register (TCD12_DOFF) */
#define S32K3XX_EDMA_TCD12_CITER_OFFSET     (0x200036) /* TCD12 Current Major Loop Count Register (TCD12_CITER) */
#define S32K3XX_EDMA_TCD12_DLAST_SGA_OFFSET (0x200038) /* TCD12 Last Destination Address Adjustment / Scatter Gather Address Register (TCD12_DLAST_SGA)*/
#define S32K3XX_EDMA_TCD12_CSR_OFFSET       (0x20003c) /* TCD12 Control and Status Register (TCD12_CSR) */
#define S32K3XX_EDMA_TCD12_BITER_OFFSET     (0x20003e) /* TCD12 Beginning Major Loop Count Register (TCD12_BITER) */

#define S32K3XX_EDMA_CH13_CSR_OFFSET        (0x204000) /* Channel 13 Control and Status Register (CH13_CSR) */
#define S32K3XX_EDMA_CH13_ES_OFFSET         (0x204004) /* Channel 13 Error Status Register (CH13_ES) */
#define S32K3XX_EDMA_CH13_INT_OFFSET        (0x204008) /* Channel 13 Interrupt Status Register (CH13_INT) */
#define S32K3XX_EDMA_CH13_SBR_OFFSET        (0x20400c) /* Channel 13 System Bus Register (CH13_SBR) */
#define S32K3XX_EDMA_CH13_PRI_OFFSET        (0x204010) /* Channel 13 Priority Register (CH13_PRI) */
#define S32K3XX_EDMA_TCD13_SADDR_OFFSET     (0x204020) /* TCD13 Source Address Register (TCD13_SADDR) */
#define S32K3XX_EDMA_TCD13_SOFF_OFFSET      (0x204024) /* TCD13 Signed Source Address Offset Register (TCD13_SOFF) */
#define S32K3XX_EDMA_TCD13_ATTR_OFFSET      (0x204026) /* TCD13 Transfer Attributes (TCD13_ATTR) */
#define S32K3XX_EDMA_TCD13_NBYTES_OFFSET    (0x204028) /* TCD13 Transfer Size (TCD13_NBYTES) */
#define S32K3XX_EDMA_TCD13_SLAST_SDA_OFFSET (0x20402c) /* TCD13 Last Source Address Adjustment / Store DADDR Address Register (TCD13_SLAST_SDA) */
#define S32K3XX_EDMA_TCD13_DADDR_OFFSET     (0x204030) /* TCD13 Destination Address Register (TCD13_DADDR) */
#define S32K3XX_EDMA_TCD13_DOFF_OFFSET      (0x204034) /* TCD13 Signed Destination Address Offset Register (TCD13_DOFF) */
#define S32K3XX_EDMA_TCD13_CITER_OFFSET     (0x204036) /* TCD13 Current Major Loop Count Register (TCD13_CITER) */
#define S32K3XX_EDMA_TCD13_DLAST_SGA_OFFSET (0x204038) /* TCD13 Last Destination Address Adjustment / Scatter Gather Address Register (TCD13_DLAST_SGA)*/
#define S32K3XX_EDMA_TCD13_CSR_OFFSET       (0x20403c) /* TCD13 Control and Status Register (TCD13_CSR) */
#define S32K3XX_EDMA_TCD13_BITER_OFFSET     (0x20403e) /* TCD13 Beginning Major Loop Count Register (TCD13_BITER) */

#define S32K3XX_EDMA_CH14_CSR_OFFSET        (0x208000) /* Channel 14 Control and Status Register (CH14_CSR) */
#define S32K3XX_EDMA_CH14_ES_OFFSET         (0x208004) /* Channel 14 Error Status Register (CH14_ES) */
#define S32K3XX_EDMA_CH14_INT_OFFSET        (0x208008) /* Channel 14 Interrupt Status Register (CH14_INT) */
#define S32K3XX_EDMA_CH14_SBR_OFFSET        (0x20800c) /* Channel 14 System Bus Register (CH14_SBR) */
#define S32K3XX_EDMA_CH14_PRI_OFFSET        (0x208010) /* Channel 14 Priority Register (CH14_PRI) */
#define S32K3XX_EDMA_TCD14_SADDR_OFFSET     (0x208020) /* TCD14 Source Address Register (TCD14_SADDR) */
#define S32K3XX_EDMA_TCD14_SOFF_OFFSET      (0x208024) /* TCD14 Signed Source Address Offset Register (TCD14_SOFF) */
#define S32K3XX_EDMA_TCD14_ATTR_OFFSET      (0x208026) /* TCD14 Transfer Attributes (TCD14_ATTR) */
#define S32K3XX_EDMA_TCD14_NBYTES_OFFSET    (0x208028) /* TCD14 Transfer Size (TCD14_NBYTES) */
#define S32K3XX_EDMA_TCD14_SLAST_SDA_OFFSET (0x20802c) /* TCD14 Last Source Address Adjustment / Store DADDR Address Register (TCD14_SLAST_SDA) */
#define S32K3XX_EDMA_TCD14_DADDR_OFFSET     (0x208030) /* TCD14 Destination Address Register (TCD14_DADDR) */
#define S32K3XX_EDMA_TCD14_DOFF_OFFSET      (0x208034) /* TCD14 Signed Destination Address Offset Register (TCD14_DOFF) */
#define S32K3XX_EDMA_TCD14_CITER_OFFSET     (0x208036) /* TCD14 Current Major Loop Count Register (TCD14_CITER) */
#define S32K3XX_EDMA_TCD14_DLAST_SGA_OFFSET (0x208038) /* TCD14 Last Destination Address Adjustment / Scatter Gather Address Register (TCD14_DLAST_SGA)*/
#define S32K3XX_EDMA_TCD14_CSR_OFFSET       (0x20803c) /* TCD14 Control and Status Register (TCD14_CSR) */
#define S32K3XX_EDMA_TCD14_BITER_OFFSET     (0x20803e) /* TCD14 Beginning Major Loop Count Register (TCD14_BITER) */

#define S32K3XX_EDMA_CH15_CSR_OFFSET        (0x20c000) /* Channel 15 Control and Status Register (CH15_CSR) */
#define S32K3XX_EDMA_CH15_ES_OFFSET         (0x20c004) /* Channel 15 Error Status Register (CH15_ES) */
#define S32K3XX_EDMA_CH15_INT_OFFSET        (0x20c008) /* Channel 15 Interrupt Status Register (CH15_INT) */
#define S32K3XX_EDMA_CH15_SBR_OFFSET        (0x20c00c) /* Channel 15 System Bus Register (CH15_SBR) */
#define S32K3XX_EDMA_CH15_PRI_OFFSET        (0x20c010) /* Channel 15 Priority Register (CH15_PRI) */
#define S32K3XX_EDMA_TCD15_SADDR_OFFSET     (0x20c020) /* TCD15 Source Address Register (TCD15_SADDR) */
#define S32K3XX_EDMA_TCD15_SOFF_OFFSET      (0x20c024) /* TCD15 Signed Source Address Offset Register (TCD15_SOFF) */
#define S32K3XX_EDMA_TCD15_ATTR_OFFSET      (0x20c026) /* TCD15 Transfer Attributes (TCD15_ATTR) */
#define S32K3XX_EDMA_TCD15_NBYTES_OFFSET    (0x20c028) /* TCD15 Transfer Size (TCD15_NBYTES) */
#define S32K3XX_EDMA_TCD15_SLAST_SDA_OFFSET (0x20c02c) /* TCD15 Last Source Address Adjustment / Store DADDR Address Register (TCD15_SLAST_SDA) */
#define S32K3XX_EDMA_TCD15_DADDR_OFFSET     (0x20c030) /* TCD15 Destination Address Register (TCD15_DADDR) */
#define S32K3XX_EDMA_TCD15_DOFF_OFFSET      (0x20c034) /* TCD15 Signed Destination Address Offset Register (TCD15_DOFF) */
#define S32K3XX_EDMA_TCD15_CITER_OFFSET     (0x20c036) /* TCD15 Current Major Loop Count Register (TCD15_CITER) */
#define S32K3XX_EDMA_TCD15_DLAST_SGA_OFFSET (0x20c038) /* TCD15 Last Destination Address Adjustment / Scatter Gather Address Register (TCD15_DLAST_SGA)*/
#define S32K3XX_EDMA_TCD15_CSR_OFFSET       (0x20c03c) /* TCD15 Control and Status Register (TCD15_CSR) */
#define S32K3XX_EDMA_TCD15_BITER_OFFSET     (0x20c03e) /* TCD15 Beginning Major Loop Count Register (TCD15_BITER) */

#define S32K3XX_EDMA_CH16_CSR_OFFSET        (0x210000) /* Channel 16 Control and Status Register (CH16_CSR) */
#define S32K3XX_EDMA_CH16_ES_OFFSET         (0x210004) /* Channel 16 Error Status Register (CH16_ES) */
#define S32K3XX_EDMA_CH16_INT_OFFSET        (0x210008) /* Channel 16 Interrupt Status Register (CH16_INT) */
#define S32K3XX_EDMA_CH16_SBR_OFFSET        (0x21000c) /* Channel 16 System Bus Register (CH16_SBR) */
#define S32K3XX_EDMA_CH16_PRI_OFFSET        (0x210010) /* Channel 16 Priority Register (CH16_PRI) */
#define S32K3XX_EDMA_TCD16_SADDR_OFFSET     (0x210020) /* TCD16 Source Address Register (TCD16_SADDR) */
#define S32K3XX_EDMA_TCD16_SOFF_OFFSET      (0x210024) /* TCD16 Signed Source Address Offset Register (TCD16_SOFF) */
#define S32K3XX_EDMA_TCD16_ATTR_OFFSET      (0x210026) /* TCD16 Transfer Attributes (TCD16_ATTR) */
#define S32K3XX_EDMA_TCD16_NBYTES_OFFSET    (0x210028) /* TCD16 Transfer Size (TCD16_NBYTES) */
#define S32K3XX_EDMA_TCD16_SLAST_SDA_OFFSET (0x21002c) /* TCD16 Last Source Address Adjustment / Store DADDR Address Register (TCD16_SLAST_SDA) */
#define S32K3XX_EDMA_TCD16_DADDR_OFFSET     (0x210030) /* TCD16 Destination Address Register (TCD16_DADDR) */
#define S32K3XX_EDMA_TCD16_DOFF_OFFSET      (0x210034) /* TCD16 Signed Destination Address Offset Register (TCD16_DOFF) */
#define S32K3XX_EDMA_TCD16_CITER_OFFSET     (0x210036) /* TCD16 Current Major Loop Count Register (TCD16_CITER) */
#define S32K3XX_EDMA_TCD16_DLAST_SGA_OFFSET (0x210038) /* TCD16 Last Destination Address Adjustment / Scatter Gather Address Register (TCD16_DLAST_SGA)*/
#define S32K3XX_EDMA_TCD16_CSR_OFFSET       (0x21003c) /* TCD16 Control and Status Register (TCD16_CSR) */
#define S32K3XX_EDMA_TCD16_BITER_OFFSET     (0x21003e) /* TCD16 Beginning Major Loop Count Register (TCD16_BITER) */

#define S32K3XX_EDMA_CH17_CSR_OFFSET        (0x214000) /* Channel 17 Control and Status Register (CH17_CSR) */
#define S32K3XX_EDMA_CH17_ES_OFFSET         (0x214004) /* Channel 17 Error Status Register (CH17_ES) */
#define S32K3XX_EDMA_CH17_INT_OFFSET        (0x214008) /* Channel 17 Interrupt Status Register (CH17_INT) */
#define S32K3XX_EDMA_CH17_SBR_OFFSET        (0x21400c) /* Channel 17 System Bus Register (CH17_SBR) */
#define S32K3XX_EDMA_CH17_PRI_OFFSET        (0x214010) /* Channel 17 Priority Register (CH17_PRI) */
#define S32K3XX_EDMA_TCD17_SADDR_OFFSET     (0x214020) /* TCD17 Source Address Register (TCD17_SADDR) */
#define S32K3XX_EDMA_TCD17_SOFF_OFFSET      (0x214024) /* TCD17 Signed Source Address Offset Register (TCD17_SOFF) */
#define S32K3XX_EDMA_TCD17_ATTR_OFFSET      (0x214026) /* TCD17 Transfer Attributes (TCD17_ATTR) */
#define S32K3XX_EDMA_TCD17_NBYTES_OFFSET    (0x214028) /* TCD17 Transfer Size (TCD17_NBYTES) */
#define S32K3XX_EDMA_TCD17_SLAST_SDA_OFFSET (0x21402c) /* TCD17 Last Source Address Adjustment / Store DADDR Address Register (TCD17_SLAST_SDA) */
#define S32K3XX_EDMA_TCD17_DADDR_OFFSET     (0x214030) /* TCD17 Destination Address Register (TCD17_DADDR) */
#define S32K3XX_EDMA_TCD17_DOFF_OFFSET      (0x214034) /* TCD17 Signed Destination Address Offset Register (TCD17_DOFF) */
#define S32K3XX_EDMA_TCD17_CITER_OFFSET     (0x214036) /* TCD17 Current Major Loop Count Register (TCD17_CITER) */
#define S32K3XX_EDMA_TCD17_DLAST_SGA_OFFSET (0x214038) /* TCD17 Last Destination Address Adjustment / Scatter Gather Address Register (TCD17_DLAST_SGA)*/
#define S32K3XX_EDMA_TCD17_CSR_OFFSET       (0x21403c) /* TCD17 Control and Status Register (TCD17_CSR) */
#define S32K3XX_EDMA_TCD17_BITER_OFFSET     (0x21403e) /* TCD17 Beginning Major Loop Count Register (TCD17_BITER) */

#define S32K3XX_EDMA_CH18_CSR_OFFSET        (0x218000) /* Channel 18 Control and Status Register (CH18_CSR) */
#define S32K3XX_EDMA_CH18_ES_OFFSET         (0x218004) /* Channel 18 Error Status Register (CH18_ES) */
#define S32K3XX_EDMA_CH18_INT_OFFSET        (0x218008) /* Channel 18 Interrupt Status Register (CH18_INT) */
#define S32K3XX_EDMA_CH18_SBR_OFFSET        (0x21800c) /* Channel 18 System Bus Register (CH18_SBR) */
#define S32K3XX_EDMA_CH18_PRI_OFFSET        (0x218010) /* Channel 18 Priority Register (CH18_PRI) */
#define S32K3XX_EDMA_TCD18_SADDR_OFFSET     (0x218020) /* TCD18 Source Address Register (TCD18_SADDR) */
#define S32K3XX_EDMA_TCD18_SOFF_OFFSET      (0x218024) /* TCD18 Signed Source Address Offset Register (TCD18_SOFF) */
#define S32K3XX_EDMA_TCD18_ATTR_OFFSET      (0x218026) /* TCD18 Transfer Attributes (TCD18_ATTR) */
#define S32K3XX_EDMA_TCD18_NBYTES_OFFSET    (0x218028) /* TCD18 Transfer Size (TCD18_NBYTES) */
#define S32K3XX_EDMA_TCD18_SLAST_SDA_OFFSET (0x21802c) /* TCD18 Last Source Address Adjustment / Store DADDR Address Register (TCD18_SLAST_SDA) */
#define S32K3XX_EDMA_TCD18_DADDR_OFFSET     (0x218030) /* TCD18 Destination Address Register (TCD18_DADDR) */
#define S32K3XX_EDMA_TCD18_DOFF_OFFSET      (0x218034) /* TCD18 Signed Destination Address Offset Register (TCD18_DOFF) */
#define S32K3XX_EDMA_TCD18_CITER_OFFSET     (0x218036) /* TCD18 Current Major Loop Count Register (TCD18_CITER) */
#define S32K3XX_EDMA_TCD18_DLAST_SGA_OFFSET (0x218038) /* TCD18 Last Destination Address Adjustment / Scatter Gather Address Register (TCD18_DLAST_SGA)*/
#define S32K3XX_EDMA_TCD18_CSR_OFFSET       (0x21803c) /* TCD18 Control and Status Register (TCD18_CSR) */
#define S32K3XX_EDMA_TCD18_BITER_OFFSET     (0x21803e) /* TCD18 Beginning Major Loop Count Register (TCD18_BITER) */

#define S32K3XX_EDMA_CH19_CSR_OFFSET        (0x21c000) /* Channel 19 Control and Status Register (CH19_CSR) */
#define S32K3XX_EDMA_CH19_ES_OFFSET         (0x21c004) /* Channel 19 Error Status Register (CH19_ES) */
#define S32K3XX_EDMA_CH19_INT_OFFSET        (0x21c008) /* Channel 19 Interrupt Status Register (CH19_INT) */
#define S32K3XX_EDMA_CH19_SBR_OFFSET        (0x21c00c) /* Channel 19 System Bus Register (CH19_SBR) */
#define S32K3XX_EDMA_CH19_PRI_OFFSET        (0x21c010) /* Channel 19 Priority Register (CH19_PRI) */
#define S32K3XX_EDMA_TCD19_SADDR_OFFSET     (0x21c020) /* TCD19 Source Address Register (TCD19_SADDR) */
#define S32K3XX_EDMA_TCD19_SOFF_OFFSET      (0x21c024) /* TCD19 Signed Source Address Offset Register (TCD19_SOFF) */
#define S32K3XX_EDMA_TCD19_ATTR_OFFSET      (0x21c026) /* TCD19 Transfer Attributes (TCD19_ATTR) */
#define S32K3XX_EDMA_TCD19_NBYTES_OFFSET    (0x21c028) /* TCD19 Transfer Size (TCD19_NBYTES) */
#define S32K3XX_EDMA_TCD19_SLAST_SDA_OFFSET (0x21c02c) /* TCD19 Last Source Address Adjustment / Store DADDR Address Register (TCD19_SLAST_SDA) */
#define S32K3XX_EDMA_TCD19_DADDR_OFFSET     (0x21c030) /* TCD19 Destination Address Register (TCD19_DADDR) */
#define S32K3XX_EDMA_TCD19_DOFF_OFFSET      (0x21c034) /* TCD19 Signed Destination Address Offset Register (TCD19_DOFF) */
#define S32K3XX_EDMA_TCD19_CITER_OFFSET     (0x21c036) /* TCD19 Current Major Loop Count Register (TCD19_CITER) */
#define S32K3XX_EDMA_TCD19_DLAST_SGA_OFFSET (0x21c038) /* TCD19 Last Destination Address Adjustment / Scatter Gather Address Register (TCD19_DLAST_SGA)*/
#define S32K3XX_EDMA_TCD19_CSR_OFFSET       (0x21c03c) /* TCD19 Control and Status Register (TCD19_CSR) */
#define S32K3XX_EDMA_TCD19_BITER_OFFSET     (0x21c03e) /* TCD19 Beginning Major Loop Count Register (TCD19_BITER) */

#define S32K3XX_EDMA_CH20_CSR_OFFSET        (0x220000) /* Channel 20 Control and Status Register (CH20_CSR) */
#define S32K3XX_EDMA_CH20_ES_OFFSET         (0x220004) /* Channel 20 Error Status Register (CH20_ES) */
#define S32K3XX_EDMA_CH20_INT_OFFSET        (0x220008) /* Channel 20 Interrupt Status Register (CH20_INT) */
#define S32K3XX_EDMA_CH20_SBR_OFFSET        (0x22000c) /* Channel 20 System Bus Register (CH20_SBR) */
#define S32K3XX_EDMA_CH20_PRI_OFFSET        (0x220010) /* Channel 20 Priority Register (CH20_PRI) */
#define S32K3XX_EDMA_TCD20_SADDR_OFFSET     (0x220020) /* TCD20 Source Address Register (TCD20_SADDR) */
#define S32K3XX_EDMA_TCD20_SOFF_OFFSET      (0x220024) /* TCD20 Signed Source Address Offset Register (TCD20_SOFF) */
#define S32K3XX_EDMA_TCD20_ATTR_OFFSET      (0x220026) /* TCD20 Transfer Attributes (TCD20_ATTR) */
#define S32K3XX_EDMA_TCD20_NBYTES_OFFSET    (0x220028) /* TCD20 Transfer Size (TCD20_NBYTES) */
#define S32K3XX_EDMA_TCD20_SLAST_SDA_OFFSET (0x22002c) /* TCD20 Last Source Address Adjustment / Store DADDR Address Register (TCD20_SLAST_SDA) */
#define S32K3XX_EDMA_TCD20_DADDR_OFFSET     (0x220030) /* TCD20 Destination Address Register (TCD20_DADDR) */
#define S32K3XX_EDMA_TCD20_DOFF_OFFSET      (0x220034) /* TCD20 Signed Destination Address Offset Register (TCD20_DOFF) */
#define S32K3XX_EDMA_TCD20_CITER_OFFSET     (0x220036) /* TCD20 Current Major Loop Count Register (TCD20_CITER) */
#define S32K3XX_EDMA_TCD20_DLAST_SGA_OFFSET (0x220038) /* TCD20 Last Destination Address Adjustment / Scatter Gather Address Register (TCD20_DLAST_SGA)*/
#define S32K3XX_EDMA_TCD20_CSR_OFFSET       (0x22003c) /* TCD20 Control and Status Register (TCD20_CSR) */
#define S32K3XX_EDMA_TCD20_BITER_OFFSET     (0x22003e) /* TCD20 Beginning Major Loop Count Register (TCD20_BITER) */

#define S32K3XX_EDMA_CH21_CSR_OFFSET        (0x224000) /* Channel 21 Control and Status Register (CH21_CSR) */
#define S32K3XX_EDMA_CH21_ES_OFFSET         (0x224004) /* Channel 21 Error Status Register (CH21_ES) */
#define S32K3XX_EDMA_CH21_INT_OFFSET        (0x224008) /* Channel 21 Interrupt Status Register (CH21_INT) */
#define S32K3XX_EDMA_CH21_SBR_OFFSET        (0x22400c) /* Channel 21 System Bus Register (CH21_SBR) */
#define S32K3XX_EDMA_CH21_PRI_OFFSET        (0x224010) /* Channel 21 Priority Register (CH21_PRI) */
#define S32K3XX_EDMA_TCD21_SADDR_OFFSET     (0x224020) /* TCD21 Source Address Register (TCD21_SADDR) */
#define S32K3XX_EDMA_TCD21_SOFF_OFFSET      (0x224024) /* TCD21 Signed Source Address Offset Register (TCD21_SOFF) */
#define S32K3XX_EDMA_TCD21_ATTR_OFFSET      (0x224026) /* TCD21 Transfer Attributes (TCD21_ATTR) */
#define S32K3XX_EDMA_TCD21_NBYTES_OFFSET    (0x224028) /* TCD21 Transfer Size (TCD21_NBYTES) */
#define S32K3XX_EDMA_TCD21_SLAST_SDA_OFFSET (0x22402c) /* TCD21 Last Source Address Adjustment / Store DADDR Address Register (TCD21_SLAST_SDA) */
#define S32K3XX_EDMA_TCD21_DADDR_OFFSET     (0x224030) /* TCD21 Destination Address Register (TCD21_DADDR) */
#define S32K3XX_EDMA_TCD21_DOFF_OFFSET      (0x224034) /* TCD21 Signed Destination Address Offset Register (TCD21_DOFF) */
#define S32K3XX_EDMA_TCD21_CITER_OFFSET     (0x224036) /* TCD21 Current Major Loop Count Register (TCD21_CITER) */
#define S32K3XX_EDMA_TCD21_DLAST_SGA_OFFSET (0x224038) /* TCD21 Last Destination Address Adjustment / Scatter Gather Address Register (TCD21_DLAST_SGA)*/
#define S32K3XX_EDMA_TCD21_CSR_OFFSET       (0x22403c) /* TCD21 Control and Status Register (TCD21_CSR) */
#define S32K3XX_EDMA_TCD21_BITER_OFFSET     (0x22403e) /* TCD21 Beginning Major Loop Count Register (TCD21_BITER) */

#define S32K3XX_EDMA_CH22_CSR_OFFSET        (0x228000) /* Channel 22 Control and Status Register (CH22_CSR) */
#define S32K3XX_EDMA_CH22_ES_OFFSET         (0x228004) /* Channel 22 Error Status Register (CH22_ES) */
#define S32K3XX_EDMA_CH22_INT_OFFSET        (0x228008) /* Channel 22 Interrupt Status Register (CH22_INT) */
#define S32K3XX_EDMA_CH22_SBR_OFFSET        (0x22800c) /* Channel 22 System Bus Register (CH22_SBR) */
#define S32K3XX_EDMA_CH22_PRI_OFFSET        (0x228010) /* Channel 22 Priority Register (CH22_PRI) */
#define S32K3XX_EDMA_TCD22_SADDR_OFFSET     (0x228020) /* TCD22 Source Address Register (TCD22_SADDR) */
#define S32K3XX_EDMA_TCD22_SOFF_OFFSET      (0x228024) /* TCD22 Signed Source Address Offset Register (TCD22_SOFF) */
#define S32K3XX_EDMA_TCD22_ATTR_OFFSET      (0x228026) /* TCD22 Transfer Attributes (TCD22_ATTR) */
#define S32K3XX_EDMA_TCD22_NBYTES_OFFSET    (0x228028) /* TCD22 Transfer Size (TCD22_NBYTES) */
#define S32K3XX_EDMA_TCD22_SLAST_SDA_OFFSET (0x22802c) /* TCD22 Last Source Address Adjustment / Store DADDR Address Register (TCD22_SLAST_SDA) */
#define S32K3XX_EDMA_TCD22_DADDR_OFFSET     (0x228030) /* TCD22 Destination Address Register (TCD22_DADDR) */
#define S32K3XX_EDMA_TCD22_DOFF_OFFSET      (0x228034) /* TCD22 Signed Destination Address Offset Register (TCD22_DOFF) */
#define S32K3XX_EDMA_TCD22_CITER_OFFSET     (0x228036) /* TCD22 Current Major Loop Count Register (TCD22_CITER) */
#define S32K3XX_EDMA_TCD22_DLAST_SGA_OFFSET (0x228038) /* TCD22 Last Destination Address Adjustment / Scatter Gather Address Register (TCD22_DLAST_SGA)*/
#define S32K3XX_EDMA_TCD22_CSR_OFFSET       (0x22803c) /* TCD22 Control and Status Register (TCD22_CSR) */
#define S32K3XX_EDMA_TCD22_BITER_OFFSET     (0x22803e) /* TCD22 Beginning Major Loop Count Register (TCD22_BITER) */

#define S32K3XX_EDMA_CH23_CSR_OFFSET        (0x22c000) /* Channel 23 Control and Status Register (CH23_CSR) */
#define S32K3XX_EDMA_CH23_ES_OFFSET         (0x22c004) /* Channel 23 Error Status Register (CH23_ES) */
#define S32K3XX_EDMA_CH23_INT_OFFSET        (0x22c008) /* Channel 23 Interrupt Status Register (CH23_INT) */
#define S32K3XX_EDMA_CH23_SBR_OFFSET        (0x22c00c) /* Channel 23 System Bus Register (CH23_SBR) */
#define S32K3XX_EDMA_CH23_PRI_OFFSET        (0x22c010) /* Channel 23 Priority Register (CH23_PRI) */
#define S32K3XX_EDMA_TCD23_SADDR_OFFSET     (0x22c020) /* TCD23 Source Address Register (TCD23_SADDR) */
#define S32K3XX_EDMA_TCD23_SOFF_OFFSET      (0x22c024) /* TCD23 Signed Source Address Offset Register (TCD23_SOFF) */
#define S32K3XX_EDMA_TCD23_ATTR_OFFSET      (0x22c026) /* TCD23 Transfer Attributes (TCD23_ATTR) */
#define S32K3XX_EDMA_TCD23_NBYTES_OFFSET    (0x22c028) /* TCD23 Transfer Size (TCD23_NBYTES) */
#define S32K3XX_EDMA_TCD23_SLAST_SDA_OFFSET (0x22c02c) /* TCD23 Last Source Address Adjustment / Store DADDR Address Register (TCD23_SLAST_SDA) */
#define S32K3XX_EDMA_TCD23_DADDR_OFFSET     (0x22c030) /* TCD23 Destination Address Register (TCD23_DADDR) */
#define S32K3XX_EDMA_TCD23_DOFF_OFFSET      (0x22c034) /* TCD23 Signed Destination Address Offset Register (TCD23_DOFF) */
#define S32K3XX_EDMA_TCD23_CITER_OFFSET     (0x22c036) /* TCD23 Current Major Loop Count Register (TCD23_CITER) */
#define S32K3XX_EDMA_TCD23_DLAST_SGA_OFFSET (0x22c038) /* TCD23 Last Destination Address Adjustment / Scatter Gather Address Register (TCD23_DLAST_SGA)*/
#define S32K3XX_EDMA_TCD23_CSR_OFFSET       (0x22c03c) /* TCD23 Control and Status Register (TCD23_CSR) */
#define S32K3XX_EDMA_TCD23_BITER_OFFSET     (0x22c03e) /* TCD23 Beginning Major Loop Count Register (TCD23_BITER) */

#define S32K3XX_EDMA_CH24_CSR_OFFSET        (0x230000) /* Channel 24 Control and Status Register (CH24_CSR) */
#define S32K3XX_EDMA_CH24_ES_OFFSET         (0x230004) /* Channel 24 Error Status Register (CH24_ES) */
#define S32K3XX_EDMA_CH24_INT_OFFSET        (0x230008) /* Channel 24 Interrupt Status Register (CH24_INT) */
#define S32K3XX_EDMA_CH24_SBR_OFFSET        (0x23000c) /* Channel 24 System Bus Register (CH24_SBR) */
#define S32K3XX_EDMA_CH24_PRI_OFFSET        (0x230010) /* Channel 24 Priority Register (CH24_PRI) */
#define S32K3XX_EDMA_TCD24_SADDR_OFFSET     (0x230020) /* TCD24 Source Address Register (TCD24_SADDR) */
#define S32K3XX_EDMA_TCD24_SOFF_OFFSET      (0x230024) /* TCD24 Signed Source Address Offset Register (TCD24_SOFF) */
#define S32K3XX_EDMA_TCD24_ATTR_OFFSET      (0x230026) /* TCD24 Transfer Attributes (TCD24_ATTR) */
#define S32K3XX_EDMA_TCD24_NBYTES_OFFSET    (0x230028) /* TCD24 Transfer Size (TCD24_NBYTES) */
#define S32K3XX_EDMA_TCD24_SLAST_SDA_OFFSET (0x23002c) /* TCD24 Last Source Address Adjustment / Store DADDR Address Register (TCD24_SLAST_SDA) */
#define S32K3XX_EDMA_TCD24_DADDR_OFFSET     (0x230030) /* TCD24 Destination Address Register (TCD24_DADDR) */
#define S32K3XX_EDMA_TCD24_DOFF_OFFSET      (0x230034) /* TCD24 Signed Destination Address Offset Register (TCD24_DOFF) */
#define S32K3XX_EDMA_TCD24_CITER_OFFSET     (0x230036) /* TCD24 Current Major Loop Count Register (TCD24_CITER) */
#define S32K3XX_EDMA_TCD24_DLAST_SGA_OFFSET (0x230038) /* TCD24 Last Destination Address Adjustment / Scatter Gather Address Register (TCD24_DLAST_SGA)*/
#define S32K3XX_EDMA_TCD24_CSR_OFFSET       (0x23003c) /* TCD24 Control and Status Register (TCD24_CSR) */
#define S32K3XX_EDMA_TCD24_BITER_OFFSET     (0x23003e) /* TCD24 Beginning Major Loop Count Register (TCD24_BITER) */

#define S32K3XX_EDMA_CH25_CSR_OFFSET        (0x234000) /* Channel 25 Control and Status Register (CH25_CSR) */
#define S32K3XX_EDMA_CH25_ES_OFFSET         (0x234004) /* Channel 25 Error Status Register (CH25_ES) */
#define S32K3XX_EDMA_CH25_INT_OFFSET        (0x234008) /* Channel 25 Interrupt Status Register (CH25_INT) */
#define S32K3XX_EDMA_CH25_SBR_OFFSET        (0x23400c) /* Channel 25 System Bus Register (CH25_SBR) */
#define S32K3XX_EDMA_CH25_PRI_OFFSET        (0x234010) /* Channel 25 Priority Register (CH25_PRI) */
#define S32K3XX_EDMA_TCD25_SADDR_OFFSET     (0x234020) /* TCD25 Source Address Register (TCD25_SADDR) */
#define S32K3XX_EDMA_TCD25_SOFF_OFFSET      (0x234024) /* TCD25 Signed Source Address Offset Register (TCD25_SOFF) */
#define S32K3XX_EDMA_TCD25_ATTR_OFFSET      (0x234026) /* TCD25 Transfer Attributes (TCD25_ATTR) */
#define S32K3XX_EDMA_TCD25_NBYTES_OFFSET    (0x234028) /* TCD25 Transfer Size (TCD25_NBYTES) */
#define S32K3XX_EDMA_TCD25_SLAST_SDA_OFFSET (0x23402c) /* TCD25 Last Source Address Adjustment / Store DADDR Address Register (TCD25_SLAST_SDA) */
#define S32K3XX_EDMA_TCD25_DADDR_OFFSET     (0x234030) /* TCD25 Destination Address Register (TCD25_DADDR) */
#define S32K3XX_EDMA_TCD25_DOFF_OFFSET      (0x234034) /* TCD25 Signed Destination Address Offset Register (TCD25_DOFF) */
#define S32K3XX_EDMA_TCD25_CITER_OFFSET     (0x234036) /* TCD25 Current Major Loop Count Register (TCD25_CITER) */
#define S32K3XX_EDMA_TCD25_DLAST_SGA_OFFSET (0x234038) /* TCD25 Last Destination Address Adjustment / Scatter Gather Address Register (TCD25_DLAST_SGA)*/
#define S32K3XX_EDMA_TCD25_CSR_OFFSET       (0x23403c) /* TCD25 Control and Status Register (TCD25_CSR) */
#define S32K3XX_EDMA_TCD25_BITER_OFFSET     (0x23403e) /* TCD25 Beginning Major Loop Count Register (TCD25_BITER) */

#define S32K3XX_EDMA_CH26_CSR_OFFSET        (0x238000) /* Channel 26 Control and Status Register (CH26_CSR) */
#define S32K3XX_EDMA_CH26_ES_OFFSET         (0x238004) /* Channel 26 Error Status Register (CH26_ES) */
#define S32K3XX_EDMA_CH26_INT_OFFSET        (0x238008) /* Channel 26 Interrupt Status Register (CH26_INT) */
#define S32K3XX_EDMA_CH26_SBR_OFFSET        (0x23800c) /* Channel 26 System Bus Register (CH26_SBR) */
#define S32K3XX_EDMA_CH26_PRI_OFFSET        (0x238010) /* Channel 26 Priority Register (CH26_PRI) */
#define S32K3XX_EDMA_TCD26_SADDR_OFFSET     (0x238020) /* TCD26 Source Address Register (TCD26_SADDR) */
#define S32K3XX_EDMA_TCD26_SOFF_OFFSET      (0x238024) /* TCD26 Signed Source Address Offset Register (TCD26_SOFF) */
#define S32K3XX_EDMA_TCD26_ATTR_OFFSET      (0x238026) /* TCD26 Transfer Attributes (TCD26_ATTR) */
#define S32K3XX_EDMA_TCD26_NBYTES_OFFSET    (0x238028) /* TCD26 Transfer Size (TCD26_NBYTES) */
#define S32K3XX_EDMA_TCD26_SLAST_SDA_OFFSET (0x23802c) /* TCD26 Last Source Address Adjustment / Store DADDR Address Register (TCD26_SLAST_SDA) */
#define S32K3XX_EDMA_TCD26_DADDR_OFFSET     (0x238030) /* TCD26 Destination Address Register (TCD26_DADDR) */
#define S32K3XX_EDMA_TCD26_DOFF_OFFSET      (0x238034) /* TCD26 Signed Destination Address Offset Register (TCD26_DOFF) */
#define S32K3XX_EDMA_TCD26_CITER_OFFSET     (0x238036) /* TCD26 Current Major Loop Count Register (TCD26_CITER) */
#define S32K3XX_EDMA_TCD26_DLAST_SGA_OFFSET (0x238038) /* TCD26 Last Destination Address Adjustment / Scatter Gather Address Register (TCD26_DLAST_SGA)*/
#define S32K3XX_EDMA_TCD26_CSR_OFFSET       (0x23803c) /* TCD26 Control and Status Register (TCD26_CSR) */
#define S32K3XX_EDMA_TCD26_BITER_OFFSET     (0x23803e) /* TCD26 Beginning Major Loop Count Register (TCD26_BITER) */

#define S32K3XX_EDMA_CH27_CSR_OFFSET        (0x23c000) /* Channel 27 Control and Status Register (CH27_CSR) */
#define S32K3XX_EDMA_CH27_ES_OFFSET         (0x23c004) /* Channel 27 Error Status Register (CH27_ES) */
#define S32K3XX_EDMA_CH27_INT_OFFSET        (0x23c008) /* Channel 27 Interrupt Status Register (CH27_INT) */
#define S32K3XX_EDMA_CH27_SBR_OFFSET        (0x23c00c) /* Channel 27 System Bus Register (CH27_SBR) */
#define S32K3XX_EDMA_CH27_PRI_OFFSET        (0x23c010) /* Channel 27 Priority Register (CH27_PRI) */
#define S32K3XX_EDMA_TCD27_SADDR_OFFSET     (0x23c020) /* TCD27 Source Address Register (TCD27_SADDR) */
#define S32K3XX_EDMA_TCD27_SOFF_OFFSET      (0x23c024) /* TCD27 Signed Source Address Offset Register (TCD27_SOFF) */
#define S32K3XX_EDMA_TCD27_ATTR_OFFSET      (0x23c026) /* TCD27 Transfer Attributes (TCD27_ATTR) */
#define S32K3XX_EDMA_TCD27_NBYTES_OFFSET    (0x23c028) /* TCD27 Transfer Size (TCD27_NBYTES) */
#define S32K3XX_EDMA_TCD27_SLAST_SDA_OFFSET (0x23c02c) /* TCD27 Last Source Address Adjustment / Store DADDR Address Register (TCD27_SLAST_SDA) */
#define S32K3XX_EDMA_TCD27_DADDR_OFFSET     (0x23c030) /* TCD27 Destination Address Register (TCD27_DADDR) */
#define S32K3XX_EDMA_TCD27_DOFF_OFFSET      (0x23c034) /* TCD27 Signed Destination Address Offset Register (TCD27_DOFF) */
#define S32K3XX_EDMA_TCD27_CITER_OFFSET     (0x23c036) /* TCD27 Current Major Loop Count Register (TCD27_CITER) */
#define S32K3XX_EDMA_TCD27_DLAST_SGA_OFFSET (0x23c038) /* TCD27 Last Destination Address Adjustment / Scatter Gather Address Register (TCD27_DLAST_SGA)*/
#define S32K3XX_EDMA_TCD27_CSR_OFFSET       (0x23c03c) /* TCD27 Control and Status Register (TCD27_CSR) */
#define S32K3XX_EDMA_TCD27_BITER_OFFSET     (0x23c03e) /* TCD27 Beginning Major Loop Count Register (TCD27_BITER) */

#define S32K3XX_EDMA_CH28_CSR_OFFSET        (0x240000) /* Channel 28 Control and Status Register (CH28_CSR) */
#define S32K3XX_EDMA_CH28_ES_OFFSET         (0x240004) /* Channel 28 Error Status Register (CH28_ES) */
#define S32K3XX_EDMA_CH28_INT_OFFSET        (0x240008) /* Channel 28 Interrupt Status Register (CH28_INT) */
#define S32K3XX_EDMA_CH28_SBR_OFFSET        (0x24000c) /* Channel 28 System Bus Register (CH28_SBR) */
#define S32K3XX_EDMA_CH28_PRI_OFFSET        (0x240010) /* Channel 28 Priority Register (CH28_PRI) */
#define S32K3XX_EDMA_TCD28_SADDR_OFFSET     (0x240020) /* TCD28 Source Address Register (TCD28_SADDR) */
#define S32K3XX_EDMA_TCD28_SOFF_OFFSET      (0x240024) /* TCD28 Signed Source Address Offset Register (TCD28_SOFF) */
#define S32K3XX_EDMA_TCD28_ATTR_OFFSET      (0x240026) /* TCD28 Transfer Attributes (TCD28_ATTR) */
#define S32K3XX_EDMA_TCD28_NBYTES_OFFSET    (0x240028) /* TCD28 Transfer Size (TCD28_NBYTES) */
#define S32K3XX_EDMA_TCD28_SLAST_SDA_OFFSET (0x24002c) /* TCD28 Last Source Address Adjustment / Store DADDR Address Register (TCD28_SLAST_SDA) */
#define S32K3XX_EDMA_TCD28_DADDR_OFFSET     (0x240030) /* TCD28 Destination Address Register (TCD28_DADDR) */
#define S32K3XX_EDMA_TCD28_DOFF_OFFSET      (0x240034) /* TCD28 Signed Destination Address Offset Register (TCD28_DOFF) */
#define S32K3XX_EDMA_TCD28_CITER_OFFSET     (0x240036) /* TCD28 Current Major Loop Count Register (TCD28_CITER) */
#define S32K3XX_EDMA_TCD28_DLAST_SGA_OFFSET (0x240038) /* TCD28 Last Destination Address Adjustment / Scatter Gather Address Register (TCD28_DLAST_SGA)*/
#define S32K3XX_EDMA_TCD28_CSR_OFFSET       (0x24003c) /* TCD28 Control and Status Register (TCD28_CSR) */
#define S32K3XX_EDMA_TCD28_BITER_OFFSET     (0x24003e) /* TCD28 Beginning Major Loop Count Register (TCD28_BITER) */

#define S32K3XX_EDMA_CH29_CSR_OFFSET        (0x244000) /* Channel 29 Control and Status Register (CH29_CSR) */
#define S32K3XX_EDMA_CH29_ES_OFFSET         (0x244004) /* Channel 29 Error Status Register (CH29_ES) */
#define S32K3XX_EDMA_CH29_INT_OFFSET        (0x244008) /* Channel 29 Interrupt Status Register (CH29_INT) */
#define S32K3XX_EDMA_CH29_SBR_OFFSET        (0x24400c) /* Channel 29 System Bus Register (CH29_SBR) */
#define S32K3XX_EDMA_CH29_PRI_OFFSET        (0x244010) /* Channel 29 Priority Register (CH29_PRI) */
#define S32K3XX_EDMA_TCD29_SADDR_OFFSET     (0x244020) /* TCD29 Source Address Register (TCD29_SADDR) */
#define S32K3XX_EDMA_TCD29_SOFF_OFFSET      (0x244024) /* TCD29 Signed Source Address Offset Register (TCD29_SOFF) */
#define S32K3XX_EDMA_TCD29_ATTR_OFFSET      (0x244026) /* TCD29 Transfer Attributes (TCD29_ATTR) */
#define S32K3XX_EDMA_TCD29_NBYTES_OFFSET    (0x244028) /* TCD29 Transfer Size (TCD29_NBYTES) */
#define S32K3XX_EDMA_TCD29_SLAST_SDA_OFFSET (0x24402c) /* TCD29 Last Source Address Adjustment / Store DADDR Address Register (TCD29_SLAST_SDA) */
#define S32K3XX_EDMA_TCD29_DADDR_OFFSET     (0x244030) /* TCD29 Destination Address Register (TCD29_DADDR) */
#define S32K3XX_EDMA_TCD29_DOFF_OFFSET      (0x244034) /* TCD29 Signed Destination Address Offset Register (TCD29_DOFF) */
#define S32K3XX_EDMA_TCD29_CITER_OFFSET     (0x244036) /* TCD29 Current Major Loop Count Register (TCD29_CITER) */
#define S32K3XX_EDMA_TCD29_DLAST_SGA_OFFSET (0x244038) /* TCD29 Last Destination Address Adjustment / Scatter Gather Address Register (TCD29_DLAST_SGA)*/
#define S32K3XX_EDMA_TCD29_CSR_OFFSET       (0x24403c) /* TCD29 Control and Status Register (TCD29_CSR) */
#define S32K3XX_EDMA_TCD29_BITER_OFFSET     (0x24403e) /* TCD29 Beginning Major Loop Count Register (TCD29_BITER) */

#define S32K3XX_EDMA_CH30_CSR_OFFSET        (0x248000) /* Channel 30 Control and Status Register (CH30_CSR) */
#define S32K3XX_EDMA_CH30_ES_OFFSET         (0x248004) /* Channel 30 Error Status Register (CH30_ES) */
#define S32K3XX_EDMA_CH30_INT_OFFSET        (0x248008) /* Channel 30 Interrupt Status Register (CH30_INT) */
#define S32K3XX_EDMA_CH30_SBR_OFFSET        (0x24800c) /* Channel 30 System Bus Register (CH30_SBR) */
#define S32K3XX_EDMA_CH30_PRI_OFFSET        (0x248010) /* Channel 30 Priority Register (CH30_PRI) */
#define S32K3XX_EDMA_TCD30_SADDR_OFFSET     (0x248020) /* TCD30 Source Address Register (TCD30_SADDR) */
#define S32K3XX_EDMA_TCD30_SOFF_OFFSET      (0x248024) /* TCD30 Signed Source Address Offset Register (TCD30_SOFF) */
#define S32K3XX_EDMA_TCD30_ATTR_OFFSET      (0x248026) /* TCD30 Transfer Attributes (TCD30_ATTR) */
#define S32K3XX_EDMA_TCD30_NBYTES_OFFSET    (0x248028) /* TCD30 Transfer Size (TCD30_NBYTES) */
#define S32K3XX_EDMA_TCD30_SLAST_SDA_OFFSET (0x24802c) /* TCD30 Last Source Address Adjustment / Store DADDR Address Register (TCD30_SLAST_SDA) */
#define S32K3XX_EDMA_TCD30_DADDR_OFFSET     (0x248030) /* TCD30 Destination Address Register (TCD30_DADDR) */
#define S32K3XX_EDMA_TCD30_DOFF_OFFSET      (0x248034) /* TCD30 Signed Destination Address Offset Register (TCD30_DOFF) */
#define S32K3XX_EDMA_TCD30_CITER_OFFSET     (0x248036) /* TCD30 Current Major Loop Count Register (TCD30_CITER) */
#define S32K3XX_EDMA_TCD30_DLAST_SGA_OFFSET (0x248038) /* TCD30 Last Destination Address Adjustment / Scatter Gather Address Register (TCD30_DLAST_SGA)*/
#define S32K3XX_EDMA_TCD30_CSR_OFFSET       (0x24803c) /* TCD30 Control and Status Register (TCD30_CSR) */
#define S32K3XX_EDMA_TCD30_BITER_OFFSET     (0x24803e) /* TCD30 Beginning Major Loop Count Register (TCD30_BITER) */

#define S32K3XX_EDMA_CH31_CSR_OFFSET        (0x24c000) /* Channel 31 Control and Status Register (CH31_CSR) */
#define S32K3XX_EDMA_CH31_ES_OFFSET         (0x24c004) /* Channel 31 Error Status Register (CH31_ES) */
#define S32K3XX_EDMA_CH31_INT_OFFSET        (0x24c008) /* Channel 31 Interrupt Status Register (CH31_INT) */
#define S32K3XX_EDMA_CH31_SBR_OFFSET        (0x24c00c) /* Channel 31 System Bus Register (CH31_SBR) */
#define S32K3XX_EDMA_CH31_PRI_OFFSET        (0x24c010) /* Channel 31 Priority Register (CH31_PRI) */
#define S32K3XX_EDMA_TCD31_SADDR_OFFSET     (0x24c020) /* TCD31 Source Address Register (TCD31_SADDR) */
#define S32K3XX_EDMA_TCD31_SOFF_OFFSET      (0x24c024) /* TCD31 Signed Source Address Offset Register (TCD31_SOFF) */
#define S32K3XX_EDMA_TCD31_ATTR_OFFSET      (0x24c026) /* TCD31 Transfer Attributes (TCD31_ATTR) */
#define S32K3XX_EDMA_TCD31_NBYTES_OFFSET    (0x24c028) /* TCD31 Transfer Size (TCD31_NBYTES) */
#define S32K3XX_EDMA_TCD31_SLAST_SDA_OFFSET (0x24c02c) /* TCD31 Last Source Address Adjustment / Store DADDR Address Register (TCD31_SLAST_SDA) */
#define S32K3XX_EDMA_TCD31_DADDR_OFFSET     (0x24c030) /* TCD31 Destination Address Register (TCD31_DADDR) */
#define S32K3XX_EDMA_TCD31_DOFF_OFFSET      (0x24c034) /* TCD31 Signed Destination Address Offset Register (TCD31_DOFF) */
#define S32K3XX_EDMA_TCD31_CITER_OFFSET     (0x24c036) /* TCD31 Current Major Loop Count Register (TCD31_CITER) */
#define S32K3XX_EDMA_TCD31_DLAST_SGA_OFFSET (0x24c038) /* TCD31 Last Destination Address Adjustment / Scatter Gather Address Register (TCD31_DLAST_SGA)*/
#define S32K3XX_EDMA_TCD31_CSR_OFFSET       (0x24c03c) /* TCD31 Control and Status Register (TCD31_CSR) */
#define S32K3XX_EDMA_TCD31_BITER_OFFSET     (0x24c03e) /* TCD31 Beginning Major Loop Count Register (TCD31_BITER) */

/* eDMA Transfer Control Descriptor (TCD) Register Addresses ****************/

#define S32K3XX_EDMA_CH0_CSR                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH0_CSR_OFFSET)
#define S32K3XX_EDMA_CH0_ES                 (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH0_ES_OFFSET)
#define S32K3XX_EDMA_CH0_INT                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH0_INT_OFFSET)
#define S32K3XX_EDMA_CH0_SBR                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH0_SBR_OFFSET)
#define S32K3XX_EDMA_CH0_PRI                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH0_PRI_OFFSET)
#define S32K3XX_EDMA_TCD0_SADDR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD0_SADDR_OFFSET)
#define S32K3XX_EDMA_TCD0_SOFF              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD0_SOFF_OFFSET)
#define S32K3XX_EDMA_TCD0_ATTR              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD0_ATTR_OFFSET)
#define S32K3XX_EDMA_TCD0_NBYTES            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD0_NBYTES_OFFSET)
#define S32K3XX_EDMA_TCD0_SLAST_SDA         (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD0_SLAST_SDA_OFFSET)
#define S32K3XX_EDMA_TCD0_DADDR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD0_DADDR_OFFSET)
#define S32K3XX_EDMA_TCD0_DOFF              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD0_DOFF_OFFSET)
#define S32K3XX_EDMA_TCD0_CITER             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD0_CITER_OFFSET)
#define S32K3XX_EDMA_TCD0_DLAST_SGA         (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD0_DLAST_SGA_OFFSET)
#define S32K3XX_EDMA_TCD0_CSR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD0_CSR_OFFSET)
#define S32K3XX_EDMA_TCD0_BITER             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD0_BITER_OFFSET)

#define S32K3XX_EDMA_CH1_CSR                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH1_CSR_OFFSET)
#define S32K3XX_EDMA_CH1_ES                 (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH1_ES_OFFSET)
#define S32K3XX_EDMA_CH1_INT                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH1_INT_OFFSET)
#define S32K3XX_EDMA_CH1_SBR                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH1_SBR_OFFSET)
#define S32K3XX_EDMA_CH1_PRI                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH1_PRI_OFFSET)
#define S32K3XX_EDMA_TCD1_SADDR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD1_SADDR_OFFSET)
#define S32K3XX_EDMA_TCD1_SOFF              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD1_SOFF_OFFSET)
#define S32K3XX_EDMA_TCD1_ATTR              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD1_ATTR_OFFSET)
#define S32K3XX_EDMA_TCD1_NBYTES            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD1_NBYTES_OFFSET)
#define S32K3XX_EDMA_TCD1_SLAST_SDA         (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD1_SLAST_SDA_OFFSET)
#define S32K3XX_EDMA_TCD1_DADDR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD1_DADDR_OFFSET)
#define S32K3XX_EDMA_TCD1_DOFF              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD1_DOFF_OFFSET)
#define S32K3XX_EDMA_TCD1_CITER             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD1_CITER_OFFSET)
#define S32K3XX_EDMA_TCD1_DLAST_SGA         (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD1_DLAST_SGA_OFFSET)
#define S32K3XX_EDMA_TCD1_CSR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD1_CSR_OFFSET)
#define S32K3XX_EDMA_TCD1_BITER             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD1_BITER_OFFSET)

#define S32K3XX_EDMA_CH2_CSR                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH2_CSR_OFFSET)
#define S32K3XX_EDMA_CH2_ES                 (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH2_ES_OFFSET)
#define S32K3XX_EDMA_CH2_INT                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH2_INT_OFFSET)
#define S32K3XX_EDMA_CH2_SBR                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH2_SBR_OFFSET)
#define S32K3XX_EDMA_CH2_PRI                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH2_PRI_OFFSET)
#define S32K3XX_EDMA_TCD2_SADDR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD2_SADDR_OFFSET)
#define S32K3XX_EDMA_TCD2_SOFF              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD2_SOFF_OFFSET)
#define S32K3XX_EDMA_TCD2_ATTR              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD2_ATTR_OFFSET)
#define S32K3XX_EDMA_TCD2_NBYTES            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD2_NBYTES_OFFSET)
#define S32K3XX_EDMA_TCD2_SLAST_SDA         (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD2_SLAST_SDA_OFFSET)
#define S32K3XX_EDMA_TCD2_DADDR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD2_DADDR_OFFSET)
#define S32K3XX_EDMA_TCD2_DOFF              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD2_DOFF_OFFSET)
#define S32K3XX_EDMA_TCD2_CITER             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD2_CITER_OFFSET)
#define S32K3XX_EDMA_TCD2_DLAST_SGA         (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD2_DLAST_SGA_OFFSET)
#define S32K3XX_EDMA_TCD2_CSR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD2_CSR_OFFSET)
#define S32K3XX_EDMA_TCD2_BITER             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD2_BITER_OFFSET)

#define S32K3XX_EDMA_CH3_CSR                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH3_CSR_OFFSET)
#define S32K3XX_EDMA_CH3_ES                 (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH3_ES_OFFSET)
#define S32K3XX_EDMA_CH3_INT                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH3_INT_OFFSET)
#define S32K3XX_EDMA_CH3_SBR                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH3_SBR_OFFSET)
#define S32K3XX_EDMA_CH3_PRI                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH3_PRI_OFFSET)
#define S32K3XX_EDMA_TCD3_SADDR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD3_SADDR_OFFSET)
#define S32K3XX_EDMA_TCD3_SOFF              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD3_SOFF_OFFSET)
#define S32K3XX_EDMA_TCD3_ATTR              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD3_ATTR_OFFSET)
#define S32K3XX_EDMA_TCD3_NBYTES            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD3_NBYTES_OFFSET)
#define S32K3XX_EDMA_TCD3_SLAST_SDA         (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD3_SLAST_SDA_OFFSET)
#define S32K3XX_EDMA_TCD3_DADDR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD3_DADDR_OFFSET)
#define S32K3XX_EDMA_TCD3_DOFF              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD3_DOFF_OFFSET)
#define S32K3XX_EDMA_TCD3_CITER             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD3_CITER_OFFSET)
#define S32K3XX_EDMA_TCD3_DLAST_SGA         (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD3_DLAST_SGA_OFFSET)
#define S32K3XX_EDMA_TCD3_CSR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD3_CSR_OFFSET)
#define S32K3XX_EDMA_TCD3_BITER             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD3_BITER_OFFSET)

#define S32K3XX_EDMA_CH4_CSR                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH4_CSR_OFFSET)
#define S32K3XX_EDMA_CH4_ES                 (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH4_ES_OFFSET)
#define S32K3XX_EDMA_CH4_INT                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH4_INT_OFFSET)
#define S32K3XX_EDMA_CH4_SBR                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH4_SBR_OFFSET)
#define S32K3XX_EDMA_CH4_PRI                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH4_PRI_OFFSET)
#define S32K3XX_EDMA_TCD4_SADDR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD4_SADDR_OFFSET)
#define S32K3XX_EDMA_TCD4_SOFF              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD4_SOFF_OFFSET)
#define S32K3XX_EDMA_TCD4_ATTR              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD4_ATTR_OFFSET)
#define S32K3XX_EDMA_TCD4_NBYTES            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD4_NBYTES_OFFSET)
#define S32K3XX_EDMA_TCD4_SLAST_SDA         (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD4_SLAST_SDA_OFFSET)
#define S32K3XX_EDMA_TCD4_DADDR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD4_DADDR_OFFSET)
#define S32K3XX_EDMA_TCD4_DOFF              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD4_DOFF_OFFSET)
#define S32K3XX_EDMA_TCD4_CITER             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD4_CITER_OFFSET)
#define S32K3XX_EDMA_TCD4_DLAST_SGA         (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD4_DLAST_SGA_OFFSET)
#define S32K3XX_EDMA_TCD4_CSR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD4_CSR_OFFSET)
#define S32K3XX_EDMA_TCD4_BITER             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD4_BITER_OFFSET)

#define S32K3XX_EDMA_CH5_CSR                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH5_CSR_OFFSET)
#define S32K3XX_EDMA_CH5_ES                 (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH5_ES_OFFSET)
#define S32K3XX_EDMA_CH5_INT                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH5_INT_OFFSET)
#define S32K3XX_EDMA_CH5_SBR                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH5_SBR_OFFSET)
#define S32K3XX_EDMA_CH5_PRI                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH5_PRI_OFFSET)
#define S32K3XX_EDMA_TCD5_SADDR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD5_SADDR_OFFSET)
#define S32K3XX_EDMA_TCD5_SOFF              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD5_SOFF_OFFSET)
#define S32K3XX_EDMA_TCD5_ATTR              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD5_ATTR_OFFSET)
#define S32K3XX_EDMA_TCD5_NBYTES            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD5_NBYTES_OFFSET)
#define S32K3XX_EDMA_TCD5_SLAST_SDA         (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD5_SLAST_SDA_OFFSET)
#define S32K3XX_EDMA_TCD5_DADDR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD5_DADDR_OFFSET)
#define S32K3XX_EDMA_TCD5_DOFF              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD5_DOFF_OFFSET)
#define S32K3XX_EDMA_TCD5_CITER             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD5_CITER_OFFSET)
#define S32K3XX_EDMA_TCD5_DLAST_SGA         (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD5_DLAST_SGA_OFFSET)
#define S32K3XX_EDMA_TCD5_CSR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD5_CSR_OFFSET)
#define S32K3XX_EDMA_TCD5_BITER             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD5_BITER_OFFSET)

#define S32K3XX_EDMA_CH6_CSR                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH6_CSR_OFFSET)
#define S32K3XX_EDMA_CH6_ES                 (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH6_ES_OFFSET)
#define S32K3XX_EDMA_CH6_INT                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH6_INT_OFFSET)
#define S32K3XX_EDMA_CH6_SBR                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH6_SBR_OFFSET)
#define S32K3XX_EDMA_CH6_PRI                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH6_PRI_OFFSET)
#define S32K3XX_EDMA_TCD6_SADDR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD6_SADDR_OFFSET)
#define S32K3XX_EDMA_TCD6_SOFF              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD6_SOFF_OFFSET)
#define S32K3XX_EDMA_TCD6_ATTR              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD6_ATTR_OFFSET)
#define S32K3XX_EDMA_TCD6_NBYTES            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD6_NBYTES_OFFSET)
#define S32K3XX_EDMA_TCD6_SLAST_SDA         (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD6_SLAST_SDA_OFFSET)
#define S32K3XX_EDMA_TCD6_DADDR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD6_DADDR_OFFSET)
#define S32K3XX_EDMA_TCD6_DOFF              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD6_DOFF_OFFSET)
#define S32K3XX_EDMA_TCD6_CITER             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD6_CITER_OFFSET)
#define S32K3XX_EDMA_TCD6_DLAST_SGA         (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD6_DLAST_SGA_OFFSET)
#define S32K3XX_EDMA_TCD6_CSR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD6_CSR_OFFSET)
#define S32K3XX_EDMA_TCD6_BITER             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD6_BITER_OFFSET)

#define S32K3XX_EDMA_CH7_CSR                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH7_CSR_OFFSET)
#define S32K3XX_EDMA_CH7_ES                 (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH7_ES_OFFSET)
#define S32K3XX_EDMA_CH7_INT                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH7_INT_OFFSET)
#define S32K3XX_EDMA_CH7_SBR                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH7_SBR_OFFSET)
#define S32K3XX_EDMA_CH7_PRI                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH7_PRI_OFFSET)
#define S32K3XX_EDMA_TCD7_SADDR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD7_SADDR_OFFSET)
#define S32K3XX_EDMA_TCD7_SOFF              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD7_SOFF_OFFSET)
#define S32K3XX_EDMA_TCD7_ATTR              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD7_ATTR_OFFSET)
#define S32K3XX_EDMA_TCD7_NBYTES            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD7_NBYTES_OFFSET)
#define S32K3XX_EDMA_TCD7_SLAST_SDA         (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD7_SLAST_SDA_OFFSET)
#define S32K3XX_EDMA_TCD7_DADDR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD7_DADDR_OFFSET)
#define S32K3XX_EDMA_TCD7_DOFF              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD7_DOFF_OFFSET)
#define S32K3XX_EDMA_TCD7_CITER             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD7_CITER_OFFSET)
#define S32K3XX_EDMA_TCD7_DLAST_SGA         (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD7_DLAST_SGA_OFFSET)
#define S32K3XX_EDMA_TCD7_CSR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD7_CSR_OFFSET)
#define S32K3XX_EDMA_TCD7_BITER             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD7_BITER_OFFSET)

#define S32K3XX_EDMA_CH8_CSR                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH8_CSR_OFFSET)
#define S32K3XX_EDMA_CH8_ES                 (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH8_ES_OFFSET)
#define S32K3XX_EDMA_CH8_INT                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH8_INT_OFFSET)
#define S32K3XX_EDMA_CH8_SBR                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH8_SBR_OFFSET)
#define S32K3XX_EDMA_CH8_PRI                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH8_PRI_OFFSET)
#define S32K3XX_EDMA_TCD8_SADDR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD8_SADDR_OFFSET)
#define S32K3XX_EDMA_TCD8_SOFF              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD8_SOFF_OFFSET)
#define S32K3XX_EDMA_TCD8_ATTR              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD8_ATTR_OFFSET)
#define S32K3XX_EDMA_TCD8_NBYTES            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD8_NBYTES_OFFSET)
#define S32K3XX_EDMA_TCD8_SLAST_SDA         (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD8_SLAST_SDA_OFFSET)
#define S32K3XX_EDMA_TCD8_DADDR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD8_DADDR_OFFSET)
#define S32K3XX_EDMA_TCD8_DOFF              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD8_DOFF_OFFSET)
#define S32K3XX_EDMA_TCD8_CITER             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD8_CITER_OFFSET)
#define S32K3XX_EDMA_TCD8_DLAST_SGA         (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD8_DLAST_SGA_OFFSET)
#define S32K3XX_EDMA_TCD8_CSR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD8_CSR_OFFSET)
#define S32K3XX_EDMA_TCD8_BITER             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD8_BITER_OFFSET)

#define S32K3XX_EDMA_CH9_CSR                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH9_CSR_OFFSET)
#define S32K3XX_EDMA_CH9_ES                 (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH9_ES_OFFSET)
#define S32K3XX_EDMA_CH9_INT                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH9_INT_OFFSET)
#define S32K3XX_EDMA_CH9_SBR                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH9_SBR_OFFSET)
#define S32K3XX_EDMA_CH9_PRI                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH9_PRI_OFFSET)
#define S32K3XX_EDMA_TCD9_SADDR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD9_SADDR_OFFSET)
#define S32K3XX_EDMA_TCD9_SOFF              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD9_SOFF_OFFSET)
#define S32K3XX_EDMA_TCD9_ATTR              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD9_ATTR_OFFSET)
#define S32K3XX_EDMA_TCD9_NBYTES            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD9_NBYTES_OFFSET)
#define S32K3XX_EDMA_TCD9_SLAST_SDA         (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD9_SLAST_SDA_OFFSET)
#define S32K3XX_EDMA_TCD9_DADDR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD9_DADDR_OFFSET)
#define S32K3XX_EDMA_TCD9_DOFF              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD9_DOFF_OFFSET)
#define S32K3XX_EDMA_TCD9_CITER             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD9_CITER_OFFSET)
#define S32K3XX_EDMA_TCD9_DLAST_SGA         (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD9_DLAST_SGA_OFFSET)
#define S32K3XX_EDMA_TCD9_CSR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD9_CSR_OFFSET)
#define S32K3XX_EDMA_TCD9_BITER             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD9_BITER_OFFSET)

#define S32K3XX_EDMA_CH10_CSR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH10_CSR_OFFSET)
#define S32K3XX_EDMA_CH10_ES                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH10_ES_OFFSET)
#define S32K3XX_EDMA_CH10_INT               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH10_INT_OFFSET)
#define S32K3XX_EDMA_CH10_SBR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH10_SBR_OFFSET)
#define S32K3XX_EDMA_CH10_PRI               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH10_PRI_OFFSET)
#define S32K3XX_EDMA_TCD10_SADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD10_SADDR_OFFSET)
#define S32K3XX_EDMA_TCD10_SOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD10_SOFF_OFFSET)
#define S32K3XX_EDMA_TCD10_ATTR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD10_ATTR_OFFSET)
#define S32K3XX_EDMA_TCD10_NBYTES           (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD10_NBYTES_OFFSET)
#define S32K3XX_EDMA_TCD10_SLAST_SDA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD10_SLAST_SDA_OFFSET)
#define S32K3XX_EDMA_TCD10_DADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD10_DADDR_OFFSET)
#define S32K3XX_EDMA_TCD10_DOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD10_DOFF_OFFSET)
#define S32K3XX_EDMA_TCD10_CITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD10_CITER_OFFSET)
#define S32K3XX_EDMA_TCD10_DLAST_SGA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD10_DLAST_SGA_OFFSET)
#define S32K3XX_EDMA_TCD10_CSR              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD10_CSR_OFFSET)
#define S32K3XX_EDMA_TCD10_BITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD10_BITER_OFFSET)

#define S32K3XX_EDMA_CH11_CSR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH11_CSR_OFFSET)
#define S32K3XX_EDMA_CH11_ES                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH11_ES_OFFSET)
#define S32K3XX_EDMA_CH11_INT               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH11_INT_OFFSET)
#define S32K3XX_EDMA_CH11_SBR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH11_SBR_OFFSET)
#define S32K3XX_EDMA_CH11_PRI               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH11_PRI_OFFSET)
#define S32K3XX_EDMA_TCD11_SADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD11_SADDR_OFFSET)
#define S32K3XX_EDMA_TCD11_SOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD11_SOFF_OFFSET)
#define S32K3XX_EDMA_TCD11_ATTR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD11_ATTR_OFFSET)
#define S32K3XX_EDMA_TCD11_NBYTES           (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD11_NBYTES_OFFSET)
#define S32K3XX_EDMA_TCD11_SLAST_SDA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD11_SLAST_SDA_OFFSET)
#define S32K3XX_EDMA_TCD11_DADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD11_DADDR_OFFSET)
#define S32K3XX_EDMA_TCD11_DOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD11_DOFF_OFFSET)
#define S32K3XX_EDMA_TCD11_CITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD11_CITER_OFFSET)
#define S32K3XX_EDMA_TCD11_DLAST_SGA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD11_DLAST_SGA_OFFSET)
#define S32K3XX_EDMA_TCD11_CSR              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD11_CSR_OFFSET)
#define S32K3XX_EDMA_TCD11_BITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD11_BITER_OFFSET)

#define S32K3XX_EDMA_CH12_CSR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH12_CSR_OFFSET)
#define S32K3XX_EDMA_CH12_ES                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH12_ES_OFFSET)
#define S32K3XX_EDMA_CH12_INT               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH12_INT_OFFSET)
#define S32K3XX_EDMA_CH12_SBR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH12_SBR_OFFSET)
#define S32K3XX_EDMA_CH12_PRI               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH12_PRI_OFFSET)
#define S32K3XX_EDMA_TCD12_SADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD12_SADDR_OFFSET)
#define S32K3XX_EDMA_TCD12_SOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD12_SOFF_OFFSET)
#define S32K3XX_EDMA_TCD12_ATTR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD12_ATTR_OFFSET)
#define S32K3XX_EDMA_TCD12_NBYTES           (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD12_NBYTES_OFFSET)
#define S32K3XX_EDMA_TCD12_SLAST_SDA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD12_SLAST_SDA_OFFSET)
#define S32K3XX_EDMA_TCD12_DADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD12_DADDR_OFFSET)
#define S32K3XX_EDMA_TCD12_DOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD12_DOFF_OFFSET)
#define S32K3XX_EDMA_TCD12_CITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD12_CITER_OFFSET)
#define S32K3XX_EDMA_TCD12_DLAST_SGA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD12_DLAST_SGA_OFFSET)
#define S32K3XX_EDMA_TCD12_CSR              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD12_CSR_OFFSET)
#define S32K3XX_EDMA_TCD12_BITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD12_BITER_OFFSET)

#define S32K3XX_EDMA_CH13_CSR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH13_CSR_OFFSET)
#define S32K3XX_EDMA_CH13_ES                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH13_ES_OFFSET)
#define S32K3XX_EDMA_CH13_INT               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH13_INT_OFFSET)
#define S32K3XX_EDMA_CH13_SBR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH13_SBR_OFFSET)
#define S32K3XX_EDMA_CH13_PRI               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH13_PRI_OFFSET)
#define S32K3XX_EDMA_TCD13_SADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD13_SADDR_OFFSET)
#define S32K3XX_EDMA_TCD13_SOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD13_SOFF_OFFSET)
#define S32K3XX_EDMA_TCD13_ATTR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD13_ATTR_OFFSET)
#define S32K3XX_EDMA_TCD13_NBYTES           (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD13_NBYTES_OFFSET)
#define S32K3XX_EDMA_TCD13_SLAST_SDA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD13_SLAST_SDA_OFFSET)
#define S32K3XX_EDMA_TCD13_DADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD13_DADDR_OFFSET)
#define S32K3XX_EDMA_TCD13_DOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD13_DOFF_OFFSET)
#define S32K3XX_EDMA_TCD13_CITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD13_CITER_OFFSET)
#define S32K3XX_EDMA_TCD13_DLAST_SGA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD13_DLAST_SGA_OFFSET)
#define S32K3XX_EDMA_TCD13_CSR              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD13_CSR_OFFSET)
#define S32K3XX_EDMA_TCD13_BITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD13_BITER_OFFSET)

#define S32K3XX_EDMA_CH14_CSR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH14_CSR_OFFSET)
#define S32K3XX_EDMA_CH14_ES                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH14_ES_OFFSET)
#define S32K3XX_EDMA_CH14_INT               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH14_INT_OFFSET)
#define S32K3XX_EDMA_CH14_SBR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH14_SBR_OFFSET)
#define S32K3XX_EDMA_CH14_PRI               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH14_PRI_OFFSET)
#define S32K3XX_EDMA_TCD14_SADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD14_SADDR_OFFSET)
#define S32K3XX_EDMA_TCD14_SOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD14_SOFF_OFFSET)
#define S32K3XX_EDMA_TCD14_ATTR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD14_ATTR_OFFSET)
#define S32K3XX_EDMA_TCD14_NBYTES           (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD14_NBYTES_OFFSET)
#define S32K3XX_EDMA_TCD14_SLAST_SDA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD14_SLAST_SDA_OFFSET)
#define S32K3XX_EDMA_TCD14_DADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD14_DADDR_OFFSET)
#define S32K3XX_EDMA_TCD14_DOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD14_DOFF_OFFSET)
#define S32K3XX_EDMA_TCD14_CITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD14_CITER_OFFSET)
#define S32K3XX_EDMA_TCD14_DLAST_SGA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD14_DLAST_SGA_OFFSET)
#define S32K3XX_EDMA_TCD14_CSR              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD14_CSR_OFFSET)
#define S32K3XX_EDMA_TCD14_BITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD14_BITER_OFFSET)

#define S32K3XX_EDMA_CH15_CSR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH15_CSR_OFFSET)
#define S32K3XX_EDMA_CH15_ES                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH15_ES_OFFSET)
#define S32K3XX_EDMA_CH15_INT               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH15_INT_OFFSET)
#define S32K3XX_EDMA_CH15_SBR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH15_SBR_OFFSET)
#define S32K3XX_EDMA_CH15_PRI               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH15_PRI_OFFSET)
#define S32K3XX_EDMA_TCD15_SADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD15_SADDR_OFFSET)
#define S32K3XX_EDMA_TCD15_SOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD15_SOFF_OFFSET)
#define S32K3XX_EDMA_TCD15_ATTR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD15_ATTR_OFFSET)
#define S32K3XX_EDMA_TCD15_NBYTES           (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD15_NBYTES_OFFSET)
#define S32K3XX_EDMA_TCD15_SLAST_SDA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD15_SLAST_SDA_OFFSET)
#define S32K3XX_EDMA_TCD15_DADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD15_DADDR_OFFSET)
#define S32K3XX_EDMA_TCD15_DOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD15_DOFF_OFFSET)
#define S32K3XX_EDMA_TCD15_CITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD15_CITER_OFFSET)
#define S32K3XX_EDMA_TCD15_DLAST_SGA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD15_DLAST_SGA_OFFSET)
#define S32K3XX_EDMA_TCD15_CSR              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD15_CSR_OFFSET)
#define S32K3XX_EDMA_TCD15_BITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD15_BITER_OFFSET)

#define S32K3XX_EDMA_CH16_CSR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH16_CSR_OFFSET)
#define S32K3XX_EDMA_CH16_ES                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH16_ES_OFFSET)
#define S32K3XX_EDMA_CH16_INT               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH16_INT_OFFSET)
#define S32K3XX_EDMA_CH16_SBR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH16_SBR_OFFSET)
#define S32K3XX_EDMA_CH16_PRI               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH16_PRI_OFFSET)
#define S32K3XX_EDMA_TCD16_SADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD16_SADDR_OFFSET)
#define S32K3XX_EDMA_TCD16_SOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD16_SOFF_OFFSET)
#define S32K3XX_EDMA_TCD16_ATTR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD16_ATTR_OFFSET)
#define S32K3XX_EDMA_TCD16_NBYTES           (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD16_NBYTES_OFFSET)
#define S32K3XX_EDMA_TCD16_SLAST_SDA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD16_SLAST_SDA_OFFSET)
#define S32K3XX_EDMA_TCD16_DADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD16_DADDR_OFFSET)
#define S32K3XX_EDMA_TCD16_DOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD16_DOFF_OFFSET)
#define S32K3XX_EDMA_TCD16_CITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD16_CITER_OFFSET)
#define S32K3XX_EDMA_TCD16_DLAST_SGA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD16_DLAST_SGA_OFFSET)
#define S32K3XX_EDMA_TCD16_CSR              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD16_CSR_OFFSET)
#define S32K3XX_EDMA_TCD16_BITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD16_BITER_OFFSET)

#define S32K3XX_EDMA_CH17_CSR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH17_CSR_OFFSET)
#define S32K3XX_EDMA_CH17_ES                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH17_ES_OFFSET)
#define S32K3XX_EDMA_CH17_INT               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH17_INT_OFFSET)
#define S32K3XX_EDMA_CH17_SBR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH17_SBR_OFFSET)
#define S32K3XX_EDMA_CH17_PRI               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH17_PRI_OFFSET)
#define S32K3XX_EDMA_TCD17_SADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD17_SADDR_OFFSET)
#define S32K3XX_EDMA_TCD17_SOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD17_SOFF_OFFSET)
#define S32K3XX_EDMA_TCD17_ATTR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD17_ATTR_OFFSET)
#define S32K3XX_EDMA_TCD17_NBYTES           (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD17_NBYTES_OFFSET)
#define S32K3XX_EDMA_TCD17_SLAST_SDA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD17_SLAST_SDA_OFFSET)
#define S32K3XX_EDMA_TCD17_DADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD17_DADDR_OFFSET)
#define S32K3XX_EDMA_TCD17_DOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD17_DOFF_OFFSET)
#define S32K3XX_EDMA_TCD17_CITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD17_CITER_OFFSET)
#define S32K3XX_EDMA_TCD17_DLAST_SGA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD17_DLAST_SGA_OFFSET)
#define S32K3XX_EDMA_TCD17_CSR              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD17_CSR_OFFSET)
#define S32K3XX_EDMA_TCD17_BITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD17_BITER_OFFSET)

#define S32K3XX_EDMA_CH18_CSR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH18_CSR_OFFSET)
#define S32K3XX_EDMA_CH18_ES                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH18_ES_OFFSET)
#define S32K3XX_EDMA_CH18_INT               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH18_INT_OFFSET)
#define S32K3XX_EDMA_CH18_SBR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH18_SBR_OFFSET)
#define S32K3XX_EDMA_CH18_PRI               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH18_PRI_OFFSET)
#define S32K3XX_EDMA_TCD18_SADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD18_SADDR_OFFSET)
#define S32K3XX_EDMA_TCD18_SOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD18_SOFF_OFFSET)
#define S32K3XX_EDMA_TCD18_ATTR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD18_ATTR_OFFSET)
#define S32K3XX_EDMA_TCD18_NBYTES           (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD18_NBYTES_OFFSET)
#define S32K3XX_EDMA_TCD18_SLAST_SDA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD18_SLAST_SDA_OFFSET)
#define S32K3XX_EDMA_TCD18_DADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD18_DADDR_OFFSET)
#define S32K3XX_EDMA_TCD18_DOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD18_DOFF_OFFSET)
#define S32K3XX_EDMA_TCD18_CITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD18_CITER_OFFSET)
#define S32K3XX_EDMA_TCD18_DLAST_SGA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD18_DLAST_SGA_OFFSET)
#define S32K3XX_EDMA_TCD18_CSR              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD18_CSR_OFFSET)
#define S32K3XX_EDMA_TCD18_BITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD18_BITER_OFFSET)

#define S32K3XX_EDMA_CH19_CSR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH19_CSR_OFFSET)
#define S32K3XX_EDMA_CH19_ES                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH19_ES_OFFSET)
#define S32K3XX_EDMA_CH19_INT               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH19_INT_OFFSET)
#define S32K3XX_EDMA_CH19_SBR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH19_SBR_OFFSET)
#define S32K3XX_EDMA_CH19_PRI               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH19_PRI_OFFSET)
#define S32K3XX_EDMA_TCD19_SADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD19_SADDR_OFFSET)
#define S32K3XX_EDMA_TCD19_SOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD19_SOFF_OFFSET)
#define S32K3XX_EDMA_TCD19_ATTR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD19_ATTR_OFFSET)
#define S32K3XX_EDMA_TCD19_NBYTES           (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD19_NBYTES_OFFSET)
#define S32K3XX_EDMA_TCD19_SLAST_SDA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD19_SLAST_SDA_OFFSET)
#define S32K3XX_EDMA_TCD19_DADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD19_DADDR_OFFSET)
#define S32K3XX_EDMA_TCD19_DOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD19_DOFF_OFFSET)
#define S32K3XX_EDMA_TCD19_CITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD19_CITER_OFFSET)
#define S32K3XX_EDMA_TCD19_DLAST_SGA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD19_DLAST_SGA_OFFSET)
#define S32K3XX_EDMA_TCD19_CSR              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD19_CSR_OFFSET)
#define S32K3XX_EDMA_TCD19_BITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD19_BITER_OFFSET)

#define S32K3XX_EDMA_CH20_CSR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH20_CSR_OFFSET)
#define S32K3XX_EDMA_CH20_ES                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH20_ES_OFFSET)
#define S32K3XX_EDMA_CH20_INT               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH20_INT_OFFSET)
#define S32K3XX_EDMA_CH20_SBR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH20_SBR_OFFSET)
#define S32K3XX_EDMA_CH20_PRI               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH20_PRI_OFFSET)
#define S32K3XX_EDMA_TCD20_SADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD20_SADDR_OFFSET)
#define S32K3XX_EDMA_TCD20_SOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD20_SOFF_OFFSET)
#define S32K3XX_EDMA_TCD20_ATTR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD20_ATTR_OFFSET)
#define S32K3XX_EDMA_TCD20_NBYTES           (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD20_NBYTES_OFFSET)
#define S32K3XX_EDMA_TCD20_SLAST_SDA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD20_SLAST_SDA_OFFSET)
#define S32K3XX_EDMA_TCD20_DADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD20_DADDR_OFFSET)
#define S32K3XX_EDMA_TCD20_DOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD20_DOFF_OFFSET)
#define S32K3XX_EDMA_TCD20_CITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD20_CITER_OFFSET)
#define S32K3XX_EDMA_TCD20_DLAST_SGA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD20_DLAST_SGA_OFFSET)
#define S32K3XX_EDMA_TCD20_CSR              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD20_CSR_OFFSET)
#define S32K3XX_EDMA_TCD20_BITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD20_BITER_OFFSET)

#define S32K3XX_EDMA_CH21_CSR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH21_CSR_OFFSET)
#define S32K3XX_EDMA_CH21_ES                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH21_ES_OFFSET)
#define S32K3XX_EDMA_CH21_INT               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH21_INT_OFFSET)
#define S32K3XX_EDMA_CH21_SBR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH21_SBR_OFFSET)
#define S32K3XX_EDMA_CH21_PRI               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH21_PRI_OFFSET)
#define S32K3XX_EDMA_TCD21_SADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD21_SADDR_OFFSET)
#define S32K3XX_EDMA_TCD21_SOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD21_SOFF_OFFSET)
#define S32K3XX_EDMA_TCD21_ATTR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD21_ATTR_OFFSET)
#define S32K3XX_EDMA_TCD21_NBYTES           (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD21_NBYTES_OFFSET)
#define S32K3XX_EDMA_TCD21_SLAST_SDA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD21_SLAST_SDA_OFFSET)
#define S32K3XX_EDMA_TCD21_DADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD21_DADDR_OFFSET)
#define S32K3XX_EDMA_TCD21_DOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD21_DOFF_OFFSET)
#define S32K3XX_EDMA_TCD21_CITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD21_CITER_OFFSET)
#define S32K3XX_EDMA_TCD21_DLAST_SGA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD21_DLAST_SGA_OFFSET)
#define S32K3XX_EDMA_TCD21_CSR              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD21_CSR_OFFSET)
#define S32K3XX_EDMA_TCD21_BITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD21_BITER_OFFSET)

#define S32K3XX_EDMA_CH22_CSR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH22_CSR_OFFSET)
#define S32K3XX_EDMA_CH22_ES                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH22_ES_OFFSET)
#define S32K3XX_EDMA_CH22_INT               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH22_INT_OFFSET)
#define S32K3XX_EDMA_CH22_SBR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH22_SBR_OFFSET)
#define S32K3XX_EDMA_CH22_PRI               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH22_PRI_OFFSET)
#define S32K3XX_EDMA_TCD22_SADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD22_SADDR_OFFSET)
#define S32K3XX_EDMA_TCD22_SOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD22_SOFF_OFFSET)
#define S32K3XX_EDMA_TCD22_ATTR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD22_ATTR_OFFSET)
#define S32K3XX_EDMA_TCD22_NBYTES           (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD22_NBYTES_OFFSET)
#define S32K3XX_EDMA_TCD22_SLAST_SDA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD22_SLAST_SDA_OFFSET)
#define S32K3XX_EDMA_TCD22_DADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD22_DADDR_OFFSET)
#define S32K3XX_EDMA_TCD22_DOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD22_DOFF_OFFSET)
#define S32K3XX_EDMA_TCD22_CITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD22_CITER_OFFSET)
#define S32K3XX_EDMA_TCD22_DLAST_SGA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD22_DLAST_SGA_OFFSET)
#define S32K3XX_EDMA_TCD22_CSR              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD22_CSR_OFFSET)
#define S32K3XX_EDMA_TCD22_BITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD22_BITER_OFFSET)

#define S32K3XX_EDMA_CH23_CSR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH23_CSR_OFFSET)
#define S32K3XX_EDMA_CH23_ES                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH23_ES_OFFSET)
#define S32K3XX_EDMA_CH23_INT               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH23_INT_OFFSET)
#define S32K3XX_EDMA_CH23_SBR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH23_SBR_OFFSET)
#define S32K3XX_EDMA_CH23_PRI               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH23_PRI_OFFSET)
#define S32K3XX_EDMA_TCD23_SADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD23_SADDR_OFFSET)
#define S32K3XX_EDMA_TCD23_SOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD23_SOFF_OFFSET)
#define S32K3XX_EDMA_TCD23_ATTR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD23_ATTR_OFFSET)
#define S32K3XX_EDMA_TCD23_NBYTES           (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD23_NBYTES_OFFSET)
#define S32K3XX_EDMA_TCD23_SLAST_SDA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD23_SLAST_SDA_OFFSET)
#define S32K3XX_EDMA_TCD23_DADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD23_DADDR_OFFSET)
#define S32K3XX_EDMA_TCD23_DOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD23_DOFF_OFFSET)
#define S32K3XX_EDMA_TCD23_CITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD23_CITER_OFFSET)
#define S32K3XX_EDMA_TCD23_DLAST_SGA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD23_DLAST_SGA_OFFSET)
#define S32K3XX_EDMA_TCD23_CSR              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD23_CSR_OFFSET)
#define S32K3XX_EDMA_TCD23_BITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD23_BITER_OFFSET)

#define S32K3XX_EDMA_CH24_CSR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH24_CSR_OFFSET)
#define S32K3XX_EDMA_CH24_ES                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH24_ES_OFFSET)
#define S32K3XX_EDMA_CH24_INT               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH24_INT_OFFSET)
#define S32K3XX_EDMA_CH24_SBR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH24_SBR_OFFSET)
#define S32K3XX_EDMA_CH24_PRI               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH24_PRI_OFFSET)
#define S32K3XX_EDMA_TCD24_SADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD24_SADDR_OFFSET)
#define S32K3XX_EDMA_TCD24_SOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD24_SOFF_OFFSET)
#define S32K3XX_EDMA_TCD24_ATTR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD24_ATTR_OFFSET)
#define S32K3XX_EDMA_TCD24_NBYTES           (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD24_NBYTES_OFFSET)
#define S32K3XX_EDMA_TCD24_SLAST_SDA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD24_SLAST_SDA_OFFSET)
#define S32K3XX_EDMA_TCD24_DADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD24_DADDR_OFFSET)
#define S32K3XX_EDMA_TCD24_DOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD24_DOFF_OFFSET)
#define S32K3XX_EDMA_TCD24_CITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD24_CITER_OFFSET)
#define S32K3XX_EDMA_TCD24_DLAST_SGA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD24_DLAST_SGA_OFFSET)
#define S32K3XX_EDMA_TCD24_CSR              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD24_CSR_OFFSET)
#define S32K3XX_EDMA_TCD24_BITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD24_BITER_OFFSET)

#define S32K3XX_EDMA_CH25_CSR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH25_CSR_OFFSET)
#define S32K3XX_EDMA_CH25_ES                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH25_ES_OFFSET)
#define S32K3XX_EDMA_CH25_INT               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH25_INT_OFFSET)
#define S32K3XX_EDMA_CH25_SBR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH25_SBR_OFFSET)
#define S32K3XX_EDMA_CH25_PRI               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH25_PRI_OFFSET)
#define S32K3XX_EDMA_TCD25_SADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD25_SADDR_OFFSET)
#define S32K3XX_EDMA_TCD25_SOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD25_SOFF_OFFSET)
#define S32K3XX_EDMA_TCD25_ATTR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD25_ATTR_OFFSET)
#define S32K3XX_EDMA_TCD25_NBYTES           (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD25_NBYTES_OFFSET)
#define S32K3XX_EDMA_TCD25_SLAST_SDA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD25_SLAST_SDA_OFFSET)
#define S32K3XX_EDMA_TCD25_DADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD25_DADDR_OFFSET)
#define S32K3XX_EDMA_TCD25_DOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD25_DOFF_OFFSET)
#define S32K3XX_EDMA_TCD25_CITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD25_CITER_OFFSET)
#define S32K3XX_EDMA_TCD25_DLAST_SGA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD25_DLAST_SGA_OFFSET)
#define S32K3XX_EDMA_TCD25_CSR              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD25_CSR_OFFSET)
#define S32K3XX_EDMA_TCD25_BITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD25_BITER_OFFSET)

#define S32K3XX_EDMA_CH26_CSR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH26_CSR_OFFSET)
#define S32K3XX_EDMA_CH26_ES                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH26_ES_OFFSET)
#define S32K3XX_EDMA_CH26_INT               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH26_INT_OFFSET)
#define S32K3XX_EDMA_CH26_SBR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH26_SBR_OFFSET)
#define S32K3XX_EDMA_CH26_PRI               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH26_PRI_OFFSET)
#define S32K3XX_EDMA_TCD26_SADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD26_SADDR_OFFSET)
#define S32K3XX_EDMA_TCD26_SOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD26_SOFF_OFFSET)
#define S32K3XX_EDMA_TCD26_ATTR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD26_ATTR_OFFSET)
#define S32K3XX_EDMA_TCD26_NBYTES           (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD26_NBYTES_OFFSET)
#define S32K3XX_EDMA_TCD26_SLAST_SDA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD26_SLAST_SDA_OFFSET)
#define S32K3XX_EDMA_TCD26_DADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD26_DADDR_OFFSET)
#define S32K3XX_EDMA_TCD26_DOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD26_DOFF_OFFSET)
#define S32K3XX_EDMA_TCD26_CITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD26_CITER_OFFSET)
#define S32K3XX_EDMA_TCD26_DLAST_SGA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD26_DLAST_SGA_OFFSET)
#define S32K3XX_EDMA_TCD26_CSR              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD26_CSR_OFFSET)
#define S32K3XX_EDMA_TCD26_BITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD26_BITER_OFFSET)

#define S32K3XX_EDMA_CH27_CSR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH27_CSR_OFFSET)
#define S32K3XX_EDMA_CH27_ES                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH27_ES_OFFSET)
#define S32K3XX_EDMA_CH27_INT               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH27_INT_OFFSET)
#define S32K3XX_EDMA_CH27_SBR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH27_SBR_OFFSET)
#define S32K3XX_EDMA_CH27_PRI               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH27_PRI_OFFSET)
#define S32K3XX_EDMA_TCD27_SADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD27_SADDR_OFFSET)
#define S32K3XX_EDMA_TCD27_SOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD27_SOFF_OFFSET)
#define S32K3XX_EDMA_TCD27_ATTR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD27_ATTR_OFFSET)
#define S32K3XX_EDMA_TCD27_NBYTES           (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD27_NBYTES_OFFSET)
#define S32K3XX_EDMA_TCD27_SLAST_SDA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD27_SLAST_SDA_OFFSET)
#define S32K3XX_EDMA_TCD27_DADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD27_DADDR_OFFSET)
#define S32K3XX_EDMA_TCD27_DOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD27_DOFF_OFFSET)
#define S32K3XX_EDMA_TCD27_CITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD27_CITER_OFFSET)
#define S32K3XX_EDMA_TCD27_DLAST_SGA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD27_DLAST_SGA_OFFSET)
#define S32K3XX_EDMA_TCD27_CSR              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD27_CSR_OFFSET)
#define S32K3XX_EDMA_TCD27_BITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD27_BITER_OFFSET)

#define S32K3XX_EDMA_CH28_CSR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH28_CSR_OFFSET)
#define S32K3XX_EDMA_CH28_ES                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH28_ES_OFFSET)
#define S32K3XX_EDMA_CH28_INT               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH28_INT_OFFSET)
#define S32K3XX_EDMA_CH28_SBR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH28_SBR_OFFSET)
#define S32K3XX_EDMA_CH28_PRI               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH28_PRI_OFFSET)
#define S32K3XX_EDMA_TCD28_SADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD28_SADDR_OFFSET)
#define S32K3XX_EDMA_TCD28_SOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD28_SOFF_OFFSET)
#define S32K3XX_EDMA_TCD28_ATTR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD28_ATTR_OFFSET)
#define S32K3XX_EDMA_TCD28_NBYTES           (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD28_NBYTES_OFFSET)
#define S32K3XX_EDMA_TCD28_SLAST_SDA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD28_SLAST_SDA_OFFSET)
#define S32K3XX_EDMA_TCD28_DADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD28_DADDR_OFFSET)
#define S32K3XX_EDMA_TCD28_DOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD28_DOFF_OFFSET)
#define S32K3XX_EDMA_TCD28_CITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD28_CITER_OFFSET)
#define S32K3XX_EDMA_TCD28_DLAST_SGA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD28_DLAST_SGA_OFFSET)
#define S32K3XX_EDMA_TCD28_CSR              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD28_CSR_OFFSET)
#define S32K3XX_EDMA_TCD28_BITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD28_BITER_OFFSET)

#define S32K3XX_EDMA_CH29_CSR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH29_CSR_OFFSET)
#define S32K3XX_EDMA_CH29_ES                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH29_ES_OFFSET)
#define S32K3XX_EDMA_CH29_INT               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH29_INT_OFFSET)
#define S32K3XX_EDMA_CH29_SBR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH29_SBR_OFFSET)
#define S32K3XX_EDMA_CH29_PRI               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH29_PRI_OFFSET)
#define S32K3XX_EDMA_TCD29_SADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD29_SADDR_OFFSET)
#define S32K3XX_EDMA_TCD29_SOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD29_SOFF_OFFSET)
#define S32K3XX_EDMA_TCD29_ATTR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD29_ATTR_OFFSET)
#define S32K3XX_EDMA_TCD29_NBYTES           (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD29_NBYTES_OFFSET)
#define S32K3XX_EDMA_TCD29_SLAST_SDA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD29_SLAST_SDA_OFFSET)
#define S32K3XX_EDMA_TCD29_DADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD29_DADDR_OFFSET)
#define S32K3XX_EDMA_TCD29_DOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD29_DOFF_OFFSET)
#define S32K3XX_EDMA_TCD29_CITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD29_CITER_OFFSET)
#define S32K3XX_EDMA_TCD29_DLAST_SGA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD29_DLAST_SGA_OFFSET)
#define S32K3XX_EDMA_TCD29_CSR              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD29_CSR_OFFSET)
#define S32K3XX_EDMA_TCD29_BITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD29_BITER_OFFSET)

#define S32K3XX_EDMA_CH30_CSR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH30_CSR_OFFSET)
#define S32K3XX_EDMA_CH30_ES                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH30_ES_OFFSET)
#define S32K3XX_EDMA_CH30_INT               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH30_INT_OFFSET)
#define S32K3XX_EDMA_CH30_SBR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH30_SBR_OFFSET)
#define S32K3XX_EDMA_CH30_PRI               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH30_PRI_OFFSET)
#define S32K3XX_EDMA_TCD30_SADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD30_SADDR_OFFSET)
#define S32K3XX_EDMA_TCD30_SOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD30_SOFF_OFFSET)
#define S32K3XX_EDMA_TCD30_ATTR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD30_ATTR_OFFSET)
#define S32K3XX_EDMA_TCD30_NBYTES           (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD30_NBYTES_OFFSET)
#define S32K3XX_EDMA_TCD30_SLAST_SDA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD30_SLAST_SDA_OFFSET)
#define S32K3XX_EDMA_TCD30_DADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD30_DADDR_OFFSET)
#define S32K3XX_EDMA_TCD30_DOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD30_DOFF_OFFSET)
#define S32K3XX_EDMA_TCD30_CITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD30_CITER_OFFSET)
#define S32K3XX_EDMA_TCD30_DLAST_SGA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD30_DLAST_SGA_OFFSET)
#define S32K3XX_EDMA_TCD30_CSR              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD30_CSR_OFFSET)
#define S32K3XX_EDMA_TCD30_BITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD30_BITER_OFFSET)

#define S32K3XX_EDMA_CH31_CSR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH31_CSR_OFFSET)
#define S32K3XX_EDMA_CH31_ES                (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH31_ES_OFFSET)
#define S32K3XX_EDMA_CH31_INT               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH31_INT_OFFSET)
#define S32K3XX_EDMA_CH31_SBR               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH31_SBR_OFFSET)
#define S32K3XX_EDMA_CH31_PRI               (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_CH31_PRI_OFFSET)
#define S32K3XX_EDMA_TCD31_SADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD31_SADDR_OFFSET)
#define S32K3XX_EDMA_TCD31_SOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD31_SOFF_OFFSET)
#define S32K3XX_EDMA_TCD31_ATTR             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD31_ATTR_OFFSET)
#define S32K3XX_EDMA_TCD31_NBYTES           (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD31_NBYTES_OFFSET)
#define S32K3XX_EDMA_TCD31_SLAST_SDA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD31_SLAST_SDA_OFFSET)
#define S32K3XX_EDMA_TCD31_DADDR            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD31_DADDR_OFFSET)
#define S32K3XX_EDMA_TCD31_DOFF             (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD31_DOFF_OFFSET)
#define S32K3XX_EDMA_TCD31_CITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD31_CITER_OFFSET)
#define S32K3XX_EDMA_TCD31_DLAST_SGA        (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD31_DLAST_SGA_OFFSET)
#define S32K3XX_EDMA_TCD31_CSR              (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD31_CSR_OFFSET)
#define S32K3XX_EDMA_TCD31_BITER            (S32K3XX_EDMA_TCD_BASE + S32K3XX_EDMA_TCD31_BITER_OFFSET)

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
#define EDMA_ES_UCE                       (1 << 9)  /* Bit 9: Uncorrectable TCD Error During Channel Execution (UCE) */
                                                    /* Bits 10-23: Reserved */
#define EDMA_ES_ERRCHN_SHIFT              (8)       /* Bits 24-28: Error Channel Number or Canceled Channel Number (ERRCHN) */
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
                                                    /* Bits 4-14: Reserved */
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

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* In-memory representation of the 32-byte Transfer Control Descriptor
 * (TCD)
 */

struct s32k3xx_edmatcd_s
{
  sq_entry_t node;
  uint8_t    flags;         /* See EDMA_CONFIG_* definitions */
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

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_EDMA_H */
