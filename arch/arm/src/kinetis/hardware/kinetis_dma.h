/****************************************************************************
 * arch/arm/src/kinetis/hardware/kinetis_dma.h
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

#ifndef __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_DMA_H
#define __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define KINETIS_DMA_CR_OFFSET             0x0000 /* Control Register */
#define KINETIS_DMA_ES_OFFSET             0x0004 /* Error Status Register */
#define KINETIS_DMA_ERQ_OFFSET            0x000c /* Enable Request Register */
#define KINETIS_DMA_EEI_OFFSET            0x0014 /* Enable Error Interrupt Register */
#define KINETIS_DMA_CEEI_OFFSET           0x0018 /* Clear Enable Error Interrupt Register */
#define KINETIS_DMA_SEEI_OFFSET           0x0019 /* Set Enable Error Interrupt Register */
#define KINETIS_DMA_CERQ_OFFSET           0x001a /* Clear Enable Request Register */
#define KINETIS_DMA_SERQ_OFFSET           0x001b /* Set Enable Request Register */
#define KINETIS_DMA_CDNE_OFFSET           0x001c /* Clear DONE Status Bit Register */
#define KINETIS_DMA_SSRT_OFFSET           0x001d /* Set START Bit Register */
#define KINETIS_DMA_CERR_OFFSET           0x001e /* Clear Error Register */
#define KINETIS_DMA_CINT_OFFSET           0x001f /* Clear Interrupt Request Register */
#define KINETIS_DMA_INT_OFFSET            0x0024 /* Interrupt Request Register */
#define KINETIS_DMA_ERR_OFFSET            0x002c /* Error Register */
#define KINETIS_DMA_HRS_OFFSET            0x0034 /* Hardware Request Status Register */

#define KINETIS_DMA_DCHPRI3_OFFSET        0x0100 /* Channel 3 Priority Register */
#define KINETIS_DMA_DCHPRI2_OFFSET        0x0101 /* Channel 2 Priority Register */
#define KINETIS_DMA_DCHPRI1_OFFSET        0x0102 /* Channel 1 Priority Register */
#define KINETIS_DMA_DCHPRI0_OFFSET        0x0103 /* Channel 0 Priority Register */
#define KINETIS_DMA_DCHPRI7_OFFSET        0x0104 /* Channel 7 Priority Register */
#define KINETIS_DMA_DCHPRI6_OFFSET        0x0105 /* Channel 6 Priority Register */
#define KINETIS_DMA_DCHPRI5_OFFSET        0x0106 /* Channel 5 Priority Register */
#define KINETIS_DMA_DCHPRI4_OFFSET        0x0107 /* Channel 4 Priority Register */
#define KINETIS_DMA_DCHPRI11_OFFSET       0x0108 /* Channel 11 Priority Register */
#define KINETIS_DMA_DCHPRI10_OFFSET       0x0109 /* Channel 10 Priority Register */
#define KINETIS_DMA_DCHPRI9_OFFSET        0x010a /* Channel 9 Priority Register */
#define KINETIS_DMA_DCHPRI8_OFFSET        0x010b /* Channel 8 Priority Register */
#define KINETIS_DMA_DCHPRI15_OFFSET       0x010c /* Channel 15 Priority Register */
#define KINETIS_DMA_DCHPRI14_OFFSET       0x010d /* Channel 14 Priority Register */
#define KINETIS_DMA_DCHPRI13_OFFSET       0x010e /* Channel 13 Priority Register */
#define KINETIS_DMA_DCHPRI12_OFFSET       0x010f /* Channel 12 Priority Register */

#define KINETIS_DMA_DCHPRI_OFFSET(n)      0x0100 + (n - (n % 4)) + (3 - (n % 4)) /* Channel n Priority Register */

#define KINETIS_DMA_TCD_OFFSET(n)         (0x0000 + ((n) << 5))
#define KINETIS_DMA_TCD_SADDR_OFFSET      0x0000 /* TCD Source Address */
#define KINETIS_DMA_TCD_SOFF_OFFSET       0x0004 /* TCD Signed Source Address Offset */
#define KINETIS_DMA_TCD_ATTR_OFFSET       0x0006 /* TCD Transfer Attributes */
#define KINETIS_DMA_TCD_NBYTES_OFFSET     0x0008 /* TCD Minor Byte Count */
#define KINETIS_DMA_TCD_SLAST_OFFSET      0x000c /* TCD Last Source Address Adjustment */
#define KINETIS_DMA_TCD_DADDR_OFFSET      0x0010 /* TCD Destination Address */
#define KINETIS_DMA_TCD_DOFF_OFFSET       0x0014 /* TCD Signed Destination Address Offset */
#define KINETIS_DMA_TCD_CITER_OFFSET      0x0016 /* TCD Current Minor Loop Link, Major Loop Count */
#define KINETIS_DMA_TCD_DLASTSGA_OFFSET   0x0018 /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define KINETIS_DMA_TCD_CSR_OFFSET        0x001c /* TCD Control and Status */
#define KINETIS_DMA_TCD_BITER_OFFSET      0x001e /* TCD Beginning Minor Loop Link, Major Loop Count */

#define KINETIS_DMA_TCD0_SADDR_OFFSET     0x0000 /* TCD Source Address */
#define KINETIS_DMA_TCD0_SOFF_OFFSET      0x0004 /* TCD Signed Source Address Offset */
#define KINETIS_DMA_TCD0_ATTR_OFFSET      0x0006 /* TCD Transfer Attributes */
#define KINETIS_DMA_TCD0_NBYTES_OFFSET    0x0008 /* TCD Minor Byte Count */
#define KINETIS_DMA_TCD0_SLAST_OFFSET     0x000c /* TCD Last Source Address Adjustment */
#define KINETIS_DMA_TCD0_DADDR_OFFSET     0x0010 /* TCD Destination Address */
#define KINETIS_DMA_TCD0_DOFF_OFFSET      0x0014 /* TCD Signed Destination Address Offset */
#define KINETIS_DMA_TCD0_CITER_OFFSET     0x0016 /* TCD Current Minor Loop Link, Major Loop Count */
#define KINETIS_DMA_TCD0_DLASTSGA_OFFSET  0x0018 /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define KINETIS_DMA_TCD0_CSR_OFFSET       0x001c /* TCD Control and Status */
#define KINETIS_DMA_TCD0_BITER_OFFSET     0x001e /* TCD Beginning Minor Loop Link, Major Loop Count */

#define KINETIS_DMA_TCD1_SADDR_OFFSET     0x0020 /* TCD Source Address */
#define KINETIS_DMA_TCD1_SOFF_OFFSET      0x0024 /* TCD Signed Source Address Offset */
#define KINETIS_DMA_TCD1_ATTR_OFFSET      0x0026 /* TCD Transfer Attributes */
#define KINETIS_DMA_TCD1_NBYTES_OFFSET    0x0028 /* TCD Minor Byte Count */
#define KINETIS_DMA_TCD1_SLAST_OFFSET     0x002c /* TCD Last Source Address Adjustment */
#define KINETIS_DMA_TCD1_DADDR_OFFSET     0x0030 /* TCD Destination Address */
#define KINETIS_DMA_TCD1_DOFF_OFFSET      0x0034 /* TCD Signed Destination Address Offset */
#define KINETIS_DMA_TCD1_CITER_OFFSET     0x0036 /* TCD Current Minor Loop Link, Major Loop Count */
#define KINETIS_DMA_TCD1_DLASTSGA_OFFSET  0x0038 /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define KINETIS_DMA_TCD1_CSR_OFFSET       0x003c /* TCD Control and Status */
#define KINETIS_DMA_TCD1_BITER_OFFSET     0x003e /* TCD Beginning Minor Loop Link, Major Loop Count */

#define KINETIS_DMA_TCD2_SADDR_OFFSET     0x0040 /* TCD Source Address */
#define KINETIS_DMA_TCD2_SOFF_OFFSET      0x0044 /* TCD Signed Source Address Offset */
#define KINETIS_DMA_TCD2_ATTR_OFFSET      0x0046 /* TCD Transfer Attributes */
#define KINETIS_DMA_TCD2_NBYTES_OFFSET    0x0048 /* TCD Minor Byte Count */
#define KINETIS_DMA_TCD2_SLAST_OFFSET     0x004c /* TCD Last Source Address Adjustment */
#define KINETIS_DMA_TCD2_DADDR_OFFSET     0x0050 /* TCD Destination Address */
#define KINETIS_DMA_TCD2_DOFF_OFFSET      0x0054 /* TCD Signed Destination Address Offset */
#define KINETIS_DMA_TCD2_CITER_OFFSET     0x0056 /* TCD Current Minor Loop Link, Major Loop Count */
#define KINETIS_DMA_TCD2_DLASTSGA_OFFSET  0x0058 /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define KINETIS_DMA_TCD2_CSR_OFFSET       0x005c /* TCD Control and Status */
#define KINETIS_DMA_TCD2_BITER_OFFSET     0x005e /* TCD Beginning Minor Loop Link, Major Loop Count */

#define KINETIS_DMA_TCD3_SADDR_OFFSET     0x0060 /* TCD Source Address */
#define KINETIS_DMA_TCD3_SOFF_OFFSET      0x0064 /* TCD Signed Source Address Offset */
#define KINETIS_DMA_TCD3_ATTR_OFFSET      0x0066 /* TCD Transfer Attributes */
#define KINETIS_DMA_TCD3_NBYTES_OFFSET    0x0068 /* TCD Minor Byte Count */
#define KINETIS_DMA_TCD3_SLAST_OFFSET     0x006c /* TCD Last Source Address Adjustment */
#define KINETIS_DMA_TCD3_DADDR_OFFSET     0x0070 /* TCD Destination Address */
#define KINETIS_DMA_TCD3_DOFF_OFFSET      0x0074 /* TCD Signed Destination Address Offset */
#define KINETIS_DMA_TCD3_CITER_OFFSET     0x0076 /* TCD Current Minor Loop Link, Major Loop Count */
#define KINETIS_DMA_TCD3_DLASTSGA_OFFSET  0x0078 /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define KINETIS_DMA_TCD3_CSR_OFFSET       0x007c /* TCD Control and Status */
#define KINETIS_DMA_TCD3_BITER_OFFSET     0x007e /* TCD Beginning Minor Loop Link, Major Loop Count */

#define KINETIS_DMA_TCD4_SADDR_OFFSET     0x0080 /* TCD Source Address */
#define KINETIS_DMA_TCD4_SOFF_OFFSET      0x0084 /* TCD Signed Source Address Offset */
#define KINETIS_DMA_TCD4_ATTR_OFFSET      0x0086 /* TCD Transfer Attributes */
#define KINETIS_DMA_TCD4_NBYTES_OFFSET    0x0088 /* TCD Minor Byte Count */
#define KINETIS_DMA_TCD4_SLAST_OFFSET     0x008c /* TCD Last Source Address Adjustment */
#define KINETIS_DMA_TCD4_DADDR_OFFSET     0x0090 /* TCD Destination Address */
#define KINETIS_DMA_TCD4_DOFF_OFFSET      0x0094 /* TCD Signed Destination Address Offset */
#define KINETIS_DMA_TCD4_CITER_OFFSET     0x0096 /* TCD Current Minor Loop Link, Major Loop Count */
#define KINETIS_DMA_TCD4_DLASTSGA_OFFSET  0x0098 /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define KINETIS_DMA_TCD4_CSR_OFFSET       0x009c /* TCD Control and Status */
#define KINETIS_DMA_TCD4_BITER_OFFSET     0x009e /* TCD Beginning Minor Loop Link, Major Loop Count */

#define KINETIS_DMA_TCD5_SADDR_OFFSET     0x00a0 /* TCD Source Address */
#define KINETIS_DMA_TCD5_SOFF_OFFSET      0x00a4 /* TCD Signed Source Address Offset */
#define KINETIS_DMA_TCD5_ATTR_OFFSET      0x00a6 /* TCD Transfer Attributes */
#define KINETIS_DMA_TCD5_NBYTES_OFFSET    0x00a8 /* TCD Minor Byte Count */
#define KINETIS_DMA_TCD5_SLAST_OFFSET     0x00ac /* TCD Last Source Address Adjustment */
#define KINETIS_DMA_TCD5_DADDR_OFFSET     0x00b0 /* TCD Destination Address */
#define KINETIS_DMA_TCD5_DOFF_OFFSET      0x00b4 /* TCD Signed Destination Address Offset */
#define KINETIS_DMA_TCD5_CITER_OFFSET     0x00b6 /* TCD Current Minor Loop Link, Major Loop Count */
#define KINETIS_DMA_TCD5_DLASTSGA_OFFSET  0x00b8 /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define KINETIS_DMA_TCD5_CSR_OFFSET       0x00bc /* TCD Control and Status */
#define KINETIS_DMA_TCD5_BITER_OFFSET     0x00be /* TCD Beginning Minor Loop Link, Major Loop Count */

#define KINETIS_DMA_TCD6_SADDR_OFFSET     0x00c0 /* TCD Source Address */
#define KINETIS_DMA_TCD6_SOFF_OFFSET      0x00c4 /* TCD Signed Source Address Offset */
#define KINETIS_DMA_TCD6_ATTR_OFFSET      0x00c6 /* TCD Transfer Attributes */
#define KINETIS_DMA_TCD6_NBYTES_OFFSET    0x00c8 /* TCD Minor Byte Count */
#define KINETIS_DMA_TCD6_SLAST_OFFSET     0x00cc /* TCD Last Source Address Adjustment */
#define KINETIS_DMA_TCD6_DADDR_OFFSET     0x00d0 /* TCD Destination Address */
#define KINETIS_DMA_TCD6_DOFF_OFFSET      0x00d4 /* TCD Signed Destination Address Offset */
#define KINETIS_DMA_TCD6_CITER_OFFSET     0x00d6 /* TCD Current Minor Loop Link, Major Loop Count */
#define KINETIS_DMA_TCD6_DLASTSGA_OFFSET  0x00d8 /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define KINETIS_DMA_TCD6_CSR_OFFSET       0x00dc /* TCD Control and Status */
#define KINETIS_DMA_TCD6_BITER_OFFSET     0x00de /* TCD Beginning Minor Loop Link, Major Loop Count */

#define KINETIS_DMA_TCD7_SADDR_OFFSET     0x00e0 /* TCD Source Address */
#define KINETIS_DMA_TCD7_SOFF_OFFSET      0x00e4 /* TCD Signed Source Address Offset */
#define KINETIS_DMA_TCD7_ATTR_OFFSET      0x00e6 /* TCD Transfer Attributes */
#define KINETIS_DMA_TCD7_NBYTES_OFFSET    0x00e8 /* TCD Minor Byte Count */
#define KINETIS_DMA_TCD7_SLAST_OFFSET     0x00ec /* TCD Last Source Address Adjustment */
#define KINETIS_DMA_TCD7_DADDR_OFFSET     0x00f0 /* TCD Destination Address */
#define KINETIS_DMA_TCD7_DOFF_OFFSET      0x00f4 /* TCD Signed Destination Address Offset */
#define KINETIS_DMA_TCD7_CITER_OFFSET     0x00f6 /* TCD Current Minor Loop Link, Major Loop Count */
#define KINETIS_DMA_TCD7_DLASTSGA_OFFSET  0x00f8 /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define KINETIS_DMA_TCD7_CSR_OFFSET       0x00fc /* TCD Control and Status */
#define KINETIS_DMA_TCD7_BITER_OFFSET     0x00fe /* TCD Beginning Minor Loop Link, Major Loop Count */

#define KINETIS_DMA_TCD8_SADDR_OFFSET     0x0100 /* TCD Source Address */
#define KINETIS_DMA_TCD8_SOFF_OFFSET      0x0104 /* TCD Signed Source Address Offset */
#define KINETIS_DMA_TCD8_ATTR_OFFSET      0x0106 /* TCD Transfer Attributes */
#define KINETIS_DMA_TCD8_NBYTES_OFFSET    0x0108 /* TCD Minor Byte Count */
#define KINETIS_DMA_TCD8_SLAST_OFFSET     0x010c /* TCD Last Source Address Adjustment */
#define KINETIS_DMA_TCD8_DADDR_OFFSET     0x0110 /* TCD Destination Address */
#define KINETIS_DMA_TCD8_DOFF_OFFSET      0x0114 /* TCD Signed Destination Address Offset */
#define KINETIS_DMA_TCD8_CITER_OFFSET     0x0116 /* TCD Current Minor Loop Link, Major Loop Count */
#define KINETIS_DMA_TCD8_DLASTSGA_OFFSET  0x0118 /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define KINETIS_DMA_TCD8_CSR_OFFSET       0x011c /* TCD Control and Status */
#define KINETIS_DMA_TCD8_BITER_OFFSET     0x011e /* TCD Beginning Minor Loop Link, Major Loop Count */

#define KINETIS_DMA_TCD9_SADDR_OFFSET     0x0120 /* TCD Source Address */
#define KINETIS_DMA_TCD9_SOFF_OFFSET      0x0124 /* TCD Signed Source Address Offset */
#define KINETIS_DMA_TCD9_ATTR_OFFSET      0x0126 /* TCD Transfer Attributes */
#define KINETIS_DMA_TCD9_NBYTES_OFFSET    0x0128 /* TCD Minor Byte Count */
#define KINETIS_DMA_TCD9_SLAST_OFFSET     0x012c /* TCD Last Source Address Adjustment */
#define KINETIS_DMA_TCD9_DADDR_OFFSET     0x0130 /* TCD Destination Address */
#define KINETIS_DMA_TCD9_DOFF_OFFSET      0x0134 /* TCD Signed Destination Address Offset */
#define KINETIS_DMA_TCD9_CITER_OFFSET     0x0136 /* TCD Current Minor Loop Link, Major Loop Count */
#define KINETIS_DMA_TCD9_DLASTSGA_OFFSET  0x0138 /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define KINETIS_DMA_TCD9_CSR_OFFSET       0x013c /* TCD Control and Status */
#define KINETIS_DMA_TCD9_BITER_OFFSET     0x013e /* TCD Beginning Minor Loop Link, Major Loop Count */

#define KINETIS_DMA_TCD10_SADDR_OFFSET    0x0140 /* TCD Source Address */
#define KINETIS_DMA_TCD10_SOFF_OFFSET     0x0144 /* TCD Signed Source Address Offset */
#define KINETIS_DMA_TCD10_ATTR_OFFSET     0x0146 /* TCD Transfer Attributes */
#define KINETIS_DMA_TCD10_NBYTES_OFFSET   0x0148 /* TCD Minor Byte Count */
#define KINETIS_DMA_TCD10_SLAST_OFFSET    0x014c /* TCD Last Source Address Adjustment */
#define KINETIS_DMA_TCD10_DADDR_OFFSET    0x0150 /* TCD Destination Address */
#define KINETIS_DMA_TCD10_DOFF_OFFSET     0x0154 /* TCD Signed Destination Address Offset */
#define KINETIS_DMA_TCD10_CITER_OFFSET    0x0156 /* TCD Current Minor Loop Link, Major Loop Count */
#define KINETIS_DMA_TCD10_DLASTSGA_OFFSET 0x0158 /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define KINETIS_DMA_TCD10_CSR_OFFSET      0x015c /* TCD Control and Status */
#define KINETIS_DMA_TCD10_BITER_OFFSET    0x015e /* TCD Beginning Minor Loop Link, Major Loop Count */

#define KINETIS_DMA_TCD11_SADDR_OFFSET    0x0160 /* TCD Source Address */
#define KINETIS_DMA_TCD11_SOFF_OFFSET     0x0164 /* TCD Signed Source Address Offset */
#define KINETIS_DMA_TCD11_ATTR_OFFSET     0x0166 /* TCD Transfer Attributes */
#define KINETIS_DMA_TCD11_NBYTES_OFFSET   0x0168 /* TCD Minor Byte Count */
#define KINETIS_DMA_TCD11_SLAST_OFFSET    0x016c /* TCD Last Source Address Adjustment */
#define KINETIS_DMA_TCD11_DADDR_OFFSET    0x0170 /* TCD Destination Address */
#define KINETIS_DMA_TCD11_DOFF_OFFSET     0x0174 /* TCD Signed Destination Address Offset */
#define KINETIS_DMA_TCD11_CITER_OFFSET    0x0176 /* TCD Current Minor Loop Link, Major Loop Count */
#define KINETIS_DMA_TCD11_DLASTSGA_OFFSET 0x0178 /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define KINETIS_DMA_TCD11_CSR_OFFSET      0x017c /* TCD Control and Status */
#define KINETIS_DMA_TCD11_BITER_OFFSET    0x017e /* TCD Beginning Minor Loop Link, Major Loop Count */

#define KINETIS_DMA_TCD12_SADDR_OFFSET    0x0180 /* TCD Source Address */
#define KINETIS_DMA_TCD12_SOFF_OFFSET     0x0184 /* TCD Signed Source Address Offset */
#define KINETIS_DMA_TCD12_ATTR_OFFSET     0x0186 /* TCD Transfer Attributes */
#define KINETIS_DMA_TCD12_NBYTES_OFFSET   0x0188 /* TCD Minor Byte Count */
#define KINETIS_DMA_TCD12_SLAST_OFFSET    0x018c /* TCD Last Source Address Adjustment */
#define KINETIS_DMA_TCD12_DADDR_OFFSET    0x0190 /* TCD Destination Address */
#define KINETIS_DMA_TCD12_DOFF_OFFSET     0x0194 /* TCD Signed Destination Address Offset */
#define KINETIS_DMA_TCD12_CITER_OFFSET    0x0196 /* TCD Current Minor Loop Link, Major Loop Count */
#define KINETIS_DMA_TCD12_DLASTSGA_OFFSET 0x0198 /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define KINETIS_DMA_TCD12_CSR_OFFSET      0x019c /* TCD Control and Status */
#define KINETIS_DMA_TCD12_BITER_OFFSET    0x019e /* TCD Beginning Minor Loop Link, Major Loop Count */

#define KINETIS_DMA_TCD13_SADDR_OFFSET    0x01a0 /* TCD Source Address */
#define KINETIS_DMA_TCD13_SOFF_OFFSET     0x01a4 /* TCD Signed Source Address Offset */
#define KINETIS_DMA_TCD13_ATTR_OFFSET     0x01a6 /* TCD Transfer Attributes */
#define KINETIS_DMA_TCD13_NBYTES_OFFSET   0x01a8 /* TCD Minor Byte Count */
#define KINETIS_DMA_TCD13_SLAST_OFFSET    0x01ac /* TCD Last Source Address Adjustment */
#define KINETIS_DMA_TCD13_DADDR_OFFSET    0x01b0 /* TCD Destination Address */
#define KINETIS_DMA_TCD13_DOFF_OFFSET     0x01b4 /* TCD Signed Destination Address Offset */
#define KINETIS_DMA_TCD13_CITER_OFFSET    0x01b6 /* TCD Current Minor Loop Link, Major Loop Count */
#define KINETIS_DMA_TCD13_DLASTSGA_OFFSET 0x01b8 /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define KINETIS_DMA_TCD13_CSR_OFFSET      0x01bc /* TCD Control and Status */
#define KINETIS_DMA_TCD13_BITER_OFFSET    0x01be /* TCD Beginning Minor Loop Link, Major Loop Count */

#define KINETIS_DMA_TCD14_SADDR_OFFSET    0x01c0 /* TCD Source Address */
#define KINETIS_DMA_TCD14_SOFF_OFFSET     0x01c4 /* TCD Signed Source Address Offset */
#define KINETIS_DMA_TCD14_ATTR_OFFSET     0x01c6 /* TCD Transfer Attributes */
#define KINETIS_DMA_TCD14_NBYTES_OFFSET   0x01c8 /* TCD Minor Byte Count */
#define KINETIS_DMA_TCD14_SLAST_OFFSET    0x01cc /* TCD Last Source Address Adjustment */
#define KINETIS_DMA_TCD14_DADDR_OFFSET    0x01d0 /* TCD Destination Address */
#define KINETIS_DMA_TCD14_DOFF_OFFSET     0x01d4 /* TCD Signed Destination Address Offset */
#define KINETIS_DMA_TCD14_CITER_OFFSET    0x01d6 /* TCD Current Minor Loop Link, Major Loop Count */
#define KINETIS_DMA_TCD14_DLASTSGA_OFFSET 0x01d8 /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define KINETIS_DMA_TCD14_CSR_OFFSET      0x01dc /* TCD Control and Status */
#define KINETIS_DMA_TCD14_BITER_OFFSET    0x01de /* TCD Beginning Minor Loop Link, Major Loop Count */

#define KINETIS_DMA_TCD15_SADDR_OFFSET    0x01e0 /* TCD Source Address */
#define KINETIS_DMA_TCD15_SOFF_OFFSET     0x01e4 /* TCD Signed Source Address Offset */
#define KINETIS_DMA_TCD15_ATTR_OFFSET     0x01e6 /* TCD Transfer Attributes */
#define KINETIS_DMA_TCD15_NBYTES_OFFSET   0x01e8 /* TCD Minor Byte Count */
#define KINETIS_DMA_TCD15_SLAST_OFFSET    0x01ec /* TCD Last Source Address Adjustment */
#define KINETIS_DMA_TCD15_DADDR_OFFSET    0x01f0 /* TCD Destination Address */
#define KINETIS_DMA_TCD15_DOFF_OFFSET     0x01f4 /* TCD Signed Destination Address Offset */
#define KINETIS_DMA_TCD15_CITER_OFFSET    0x01f6 /* TCD Current Minor Loop Link, Major Loop Count */
#define KINETIS_DMA_TCD15_DLASTSGA_OFFSET 0x01f8 /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define KINETIS_DMA_TCD15_CSR_OFFSET      0x01fc /* TCD Control and Status */
#define KINETIS_DMA_TCD15_BITER_OFFSET    0x01fe /* TCD Beginning Minor Loop Link, Major Loop Count */

/* Register Addresses *******************************************************/

#define KINETIS_DMA_CR                    (KINETIS_DMAC_BASE + KINETIS_DMA_CR_OFFSET)
#define KINETIS_DMA_ES                    (KINETIS_DMAC_BASE + KINETIS_DMA_ES_OFFSET)
#define KINETIS_DMA_ERQ                   (KINETIS_DMAC_BASE + KINETIS_DMA_ERQ_OFFSET)
#define KINETIS_DMA_EEI                   (KINETIS_DMAC_BASE + KINETIS_DMA_EEI_OFFSET)
#define KINETIS_DMA_CEEI                  (KINETIS_DMAC_BASE + KINETIS_DMA_CEEI_OFFSET)
#define KINETIS_DMA_SEEI                  (KINETIS_DMAC_BASE + KINETIS_DMA_SEEI_OFFSET)
#define KINETIS_DMA_CERQ                  (KINETIS_DMAC_BASE + KINETIS_DMA_CERQ_OFFSET)
#define KINETIS_DMA_SERQ                  (KINETIS_DMAC_BASE + KINETIS_DMA_SERQ_OFFSET)
#define KINETIS_DMA_CDNE                  (KINETIS_DMAC_BASE + KINETIS_DMA_CDNE_OFFSET)
#define KINETIS_DMA_SSRT                  (KINETIS_DMAC_BASE + KINETIS_DMA_SSRT_OFFSET)
#define KINETIS_DMA_CERR                  (KINETIS_DMAC_BASE + KINETIS_DMA_CERR_OFFSET)
#define KINETIS_DMA_CINT                  (KINETIS_DMAC_BASE + KINETIS_DMA_CINT_OFFSET)
#define KINETIS_DMA_INT                   (KINETIS_DMAC_BASE + KINETIS_DMA_INT_OFFSET)
#define KINETIS_DMA_ERR                   (KINETIS_DMAC_BASE + KINETIS_DMA_ERR_OFFSET)
#define KINETIS_DMA_HRS                   (KINETIS_DMAC_BASE + KINETIS_DMA_HRS_OFFSET)

#define KINETIS_DMA_DCHPRI(n)             (KINETIS_DMAC_BASE + KINETIS_DMA_DCHPRI_OFFSET(n))

#define KINETIS_DMA_DCHPRI3               (KINETIS_DMAC_BASE + KINETIS_DMA_DCHPRI3_OFFSET)
#define KINETIS_DMA_DCHPRI2               (KINETIS_DMAC_BASE + KINETIS_DMA_DCHPRI2_OFFSET)
#define KINETIS_DMA_DCHPRI1               (KINETIS_DMAC_BASE + KINETIS_DMA_DCHPRI1_OFFSET)
#define KINETIS_DMA_DCHPRI0               (KINETIS_DMAC_BASE + KINETIS_DMA_DCHPRI0_OFFSET)
#define KINETIS_DMA_DCHPRI7               (KINETIS_DMAC_BASE + KINETIS_DMA_DCHPRI7_OFFSET)
#define KINETIS_DMA_DCHPRI6               (KINETIS_DMAC_BASE + KINETIS_DMA_DCHPRI6_OFFSET)
#define KINETIS_DMA_DCHPRI5               (KINETIS_DMAC_BASE + KINETIS_DMA_DCHPRI5_OFFSET)
#define KINETIS_DMA_DCHPRI4               (KINETIS_DMAC_BASE + KINETIS_DMA_DCHPRI4_OFFSET)
#define KINETIS_DMA_DCHPRI11              (KINETIS_DMAC_BASE + KINETIS_DMA_DCHPRI11_OFFSET)
#define KINETIS_DMA_DCHPRI10              (KINETIS_DMAC_BASE + KINETIS_DMA_DCHPRI10_OFFSET)
#define KINETIS_DMA_DCHPRI9               (KINETIS_DMAC_BASE + KINETIS_DMA_DCHPRI9_OFFSET)
#define KINETIS_DMA_DCHPRI8               (KINETIS_DMAC_BASE + KINETIS_DMA_DCHPRI8_OFFSET)
#define KINETIS_DMA_DCHPRI15              (KINETIS_DMAC_BASE + KINETIS_DMA_DCHPRI15_OFFSET)
#define KINETIS_DMA_DCHPRI14              (KINETIS_DMAC_BASE + KINETIS_DMA_DCHPRI14_OFFSET)
#define KINETIS_DMA_DCHPRI13              (KINETIS_DMAC_BASE + KINETIS_DMA_DCHPRI13_OFFSET)
#define KINETIS_DMA_DCHPRI12              (KINETIS_DMAC_BASE + KINETIS_DMA_DCHPRI12_OFFSET)

#define KINETIS_DMA_TCD_BASE(n)           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD_OFFSET(n))

#define KINETIS_DMA_TCD_SADDR(n)          (KINETIS_DMA_TCD_BASE(n)+KINETIS_DMA_TCD_SADDR_OFFSET)
#define KINETIS_DMA_TCD_SOFF(n)           (KINETIS_DMA_TCD_BASE(n)+KINETIS_DMA_TCD_SOFF_OFFSET)
#define KINETIS_DMA_TCD_ATTR(n)           (KINETIS_DMA_TCD_BASE(n)+KINETIS_DMA_TCD_ATTR_OFFSET)
#define KINETIS_DMA_TCD_NBYTES(n)         (KINETIS_DMA_TCD_BASE(n)+KINETIS_DMA_TCD_NBYTES_OFFSET)
#define KINETIS_DMA_TCD_SLAST(n)          (KINETIS_DMA_TCD_BASE(n)+KINETIS_DMA_TCD_SLAST_OFFSET)
#define KINETIS_DMA_TCD_DADDR(n)          (KINETIS_DMA_TCD_BASE(n)+KINETIS_DMA_TCD_DADDR_OFFSET)
#define KINETIS_DMA_TCD_DOFF(n)           (KINETIS_DMA_TCD_BASE(n)+KINETIS_DMA_TCD_DOFF_OFFSET)
#define KINETIS_DMA_TCD_CITER(n)          (KINETIS_DMA_TCD_BASE(n)+KINETIS_DMA_TCD_CITER_OFFSET)
#define KINETIS_DMA_TCD_DLASTSGA(n)       (KINETIS_DMA_TCD_BASE(n)+KINETIS_DMA_TCD_DLASTSGA_OFFSET)
#define KINETIS_DMA_TCD_CSR(n)            (KINETIS_DMA_TCD_BASE(n)+KINETIS_DMA_TCD_CSR_OFFSET)
#define KINETIS_DMA_TCD_BITER(n)          (KINETIS_DMA_TCD_BASE(n)+KINETIS_DMA_TCD_BITER_OFFSET)

#define KINETIS_DMA_TCD0_SADDR            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD0_SADDR_OFFSET)
#define KINETIS_DMA_TCD0_SOFF             (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD0_SOFF_OFFSET)
#define KINETIS_DMA_TCD0_ATTR             (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD0_ATTR_OFFSET)
#define KINETIS_DMA_TCD0_NBYTES           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD0_NBYTES_OFFSET)
#define KINETIS_DMA_TCD0_SLAST            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD0_SLAST_OFFSET)
#define KINETIS_DMA_TCD0_DADDR            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD0_DADDR_OFFSET)
#define KINETIS_DMA_TCD0_DOFF             (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD0_DOFF_OFFSET)
#define KINETIS_DMA_TCD0_CITER            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD0_CITER_OFFSET)
#define KINETIS_DMA_TCD0_DLASTSGA         (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD0_DLASTSGA_OFFSET)
#define KINETIS_DMA_TCD0_CSR              (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD0_CSR_OFFSET)
#define KINETIS_DMA_TCD0_BITER            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD0_BITER_OFFSET)

#define KINETIS_DMA_TCD1_SADDR            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD1_SADDR_OFFSET)
#define KINETIS_DMA_TCD1_SOFF             (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD1_SOFF_OFFSET)
#define KINETIS_DMA_TCD1_ATTR             (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD1_ATTR_OFFSET)
#define KINETIS_DMA_TCD1_NBYTES           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD1_NBYTES_OFFSET)
#define KINETIS_DMA_TCD1_SLAST            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD1_SLAST_OFFSET)
#define KINETIS_DMA_TCD1_DADDR            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD1_DADDR_OFFSET)
#define KINETIS_DMA_TCD1_DOFF             (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD1_DOFF_OFFSET)
#define KINETIS_DMA_TCD1_CITER            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD1_CITER_OFFSET)
#define KINETIS_DMA_TCD1_DLASTSGA         (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD1_DLASTSGA_OFFSET)
#define KINETIS_DMA_TCD1_CSR              (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD1_CSR_OFFSET)
#define KINETIS_DMA_TCD1_BITER            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD1_BITER_OFFSET)

#define KINETIS_DMA_TCD2_SADDR            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD2_SADDR_OFFSET)
#define KINETIS_DMA_TCD2_SOFF             (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD2_SOFF_OFFSET)
#define KINETIS_DMA_TCD2_ATTR             (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD2_ATTR_OFFSET)
#define KINETIS_DMA_TCD2_NBYTES           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD2_NBYTES_OFFSET)
#define KINETIS_DMA_TCD2_SLAST            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD2_SLAST_OFFSET)
#define KINETIS_DMA_TCD2_DADDR            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD2_DADDR_OFFSET)
#define KINETIS_DMA_TCD2_DOFF             (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD2_DOFF_OFFSET)
#define KINETIS_DMA_TCD2_CITER            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD2_CITER_OFFSET)
#define KINETIS_DMA_TCD2_DLASTSGA         (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD2_DLASTSGA_OFFSET)
#define KINETIS_DMA_TCD2_CSR_             (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD2_CSR_OFFSET)
#define KINETIS_DMA_TCD2_BITER            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD2_BITER_OFFSET)

#define KINETIS_DMA_TCD3_SADDR            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD3_SADDR_OFFSET)
#define KINETIS_DMA_TCD3_SOFF             (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD3_SOFF_OFFSET)
#define KINETIS_DMA_TCD3_ATTR             (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD3_ATTR_OFFSET)
#define KINETIS_DMA_TCD3_NBYTES           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD3_NBYTES_OFFSET)
#define KINETIS_DMA_TCD3_SLAST            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD3_SLAST_OFFSET)
#define KINETIS_DMA_TCD3_DADDR            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD3_DADDR_OFFSET)
#define KINETIS_DMA_TCD3_DOFF             (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD3_DOFF_OFFSET)
#define KINETIS_DMA_TCD3_CITER            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD3_CITER_OFFSET)
#define KINETIS_DMA_TCD3_DLASTSGA         (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD3_DLASTSGA_OFFSET
#define KINETIS_DMA_TCD3_CSR              (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD3_CSR_OFFSET)
#define KINETIS_DMA_TCD3_BITER            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD3_BITER_OFFSET)

#define KINETIS_DMA_TCD4_SADDR            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD4_SADDR_OFFSET)
#define KINETIS_DMA_TCD4_SOFF             (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD4_SOFF_OFFSET)
#define KINETIS_DMA_TCD4_ATTR             (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD4_ATTR_OFFSET0)
#define KINETIS_DMA_TCD4_NBYTES           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD4_NBYTES_OFFSET)
#define KINETIS_DMA_TCD4_SLAST            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD4_SLAST_OFFSET)
#define KINETIS_DMA_TCD4_DADDR            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD4_DADDR_OFFSET)
#define KINETIS_DMA_TCD4_DOFF             (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD4_DOFF_OFFSET)
#define KINETIS_DMA_TCD4_CITER            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD4_CITER_OFFSET)
#define KINETIS_DMA_TCD4_DLASTSGA         (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD4_DLASTSGA_OFFSET)
#define KINETIS_DMA_TCD4_CSR              (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD4_CSR_OFFSET)
#define KINETIS_DMA_TCD4_BITER            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD4_BITER_OFFSET)

#define KINETIS_DMA_TCD5_SADDR            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD5_SADDR_OFFSET)
#define KINETIS_DMA_TCD5_SOFF             (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD5_SOFF_OFFSET)
#define KINETIS_DMA_TCD5_ATTR             (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD5_ATTR_OFFSET)
#define KINETIS_DMA_TCD5_NBYTES           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD5_NBYTES_OFFSET)
#define KINETIS_DMA_TCD5_SLAST            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD5_SLAST_OFFSET)
#define KINETIS_DMA_TCD5_DADDR            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD5_DADDR_OFFSET)
#define KINETIS_DMA_TCD5_DOFF             (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD5_DOFF_OFFSET)
#define KINETIS_DMA_TCD5_CITER            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD5_CITER_OFFSET)
#define KINETIS_DMA_TCD5_DLASTSGA         (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD5_DLASTSGA_OFFSET)
#define KINETIS_DMA_TCD5_CSR              (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD5_CSR_OFFSET)
#define KINETIS_DMA_TCD5_BITER            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD5_BITER_OFFSET)

#define KINETIS_DMA_TCD6_SADDR            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD6_SADDR_OFFSET)
#define KINETIS_DMA_TCD6_SOFF             (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD6_SOFF_OFFSET)
#define KINETIS_DMA_TCD6_ATTR             (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD6_ATTR_OFFSET)
#define KINETIS_DMA_TCD6_NBYTES           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD6_NBYTES_OFFSET)
#define KINETIS_DMA_TCD6_SLAST            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD6_SLAST_OFFSET)
#define KINETIS_DMA_TCD6_DADDR            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD6_DADDR_OFFSET)
#define KINETIS_DMA_TCD6_DOFF             (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD6_DOFF_OFFSET)
#define KINETIS_DMA_TCD6_CITER            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD6_CITER_OFFSET)
#define KINETIS_DMA_TCD6_DLASTSGA         (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD6_DLASTSGA_OFFSET)
#define KINETIS_DMA_TCD6_CSR              (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD6_CSR_OFFSET)
#define KINETIS_DMA_TCD6_BITER            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD6_BITER_OFFSET)

#define KINETIS_DMA_TCD7_SADDR            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD7_SADDR_OFFSET)
#define KINETIS_DMA_TCD7_SOFF             (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD7_SOFF_OFFSET)
#define KINETIS_DMA_TCD7_ATTR             (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD7_ATTR_OFFSET)
#define KINETIS_DMA_TCD7_NBYTES           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD7_NBYTES_OFFSET)
#define KINETIS_DMA_TCD7_SLAST            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD7_SLAST_OFFSET)
#define KINETIS_DMA_TCD7_DADDR            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD7_DADDR_OFFSET)
#define KINETIS_DMA_TCD7_DOFF             (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD7_DOFF_OFFSET)
#define KINETIS_DMA_TCD7_CITER            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD7_CITER_OFFSET)
#define KINETIS_DMA_TCD7_DLASTSGA_        (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD7_DLASTSGA_OFFSET)
#define KINETIS_DMA_TCD7_CSR              (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD7_CSR_OFFSET)
#define KINETIS_DMA_TCD7_BITER            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD7_BITER_OFFSET)

#define KINETIS_DMA_TCD8_SADDR            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD8_SADDR_OFFSET)
#define KINETIS_DMA_TCD8_SOFF             (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD8_SOFF_OFFSET)
#define KINETIS_DMA_TCD8_ATTR             (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD8_ATTR_OFFSET)
#define KINETIS_DMA_TCD8_NBYTES           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD8_NBYTES_OFFSET)
#define KINETIS_DMA_TCD8_SLAST            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD8_SLAST_OFFSET)
#define KINETIS_DMA_TCD8_DADDR            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD8_DADDR_OFFSET)
#define KINETIS_DMA_TCD8_DOFF             (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD8_DOFF_OFFSET)
#define KINETIS_DMA_TCD8_CITER            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD8_CITER_OFFSET)
#define KINETIS_DMA_TCD8_DLASTSGA         (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD8_DLASTSGA_OFFSET)
#define KINETIS_DMA_TCD8_CSR              (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD8_CSR_OFFSET)
#define KINETIS_DMA_TCD8_BITER            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD8_BITER_OFFSET)

#define KINETIS_DMA_TCD9_SADDR            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD9_SADDR_OFFSET)
#define KINETIS_DMA_TCD9_SOFF             (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD9_SOFF_OFFSET)
#define KINETIS_DMA_TCD9_ATTR             (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD9_ATTR_OFFSET)
#define KINETIS_DMA_TCD9_NBYTES           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD9_NBYTES_OFFSET)
#define KINETIS_DMA_TCD9_SLAST            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD9_SLAST_OFFSET)
#define KINETIS_DMA_TCD9_DADDR            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD9_DADDR_OFFSET)
#define KINETIS_DMA_TCD9_DOFF             (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD9_DOFF_OFFSET)
#define KINETIS_DMA_TCD9_CITER            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD9_CITER_OFFSET)
#define KINETIS_DMA_TCD9_DLASTSGA         (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD9_DLASTSGA_OFFSET)
#define KINETIS_DMA_TCD9_CSR              (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD9_CSR_OFFSET)
#define KINETIS_DMA_TCD9_BITER            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD9_BITER_OFFSET)

#define KINETIS_DMA_TCD10_SADDR           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD10_SADDR_OFFSET)
#define KINETIS_DMA_TCD10_SOFF            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD10_SOFF_OFFSET)
#define KINETIS_DMA_TCD10_ATTR            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD10_ATTR_OFFSET)
#define KINETIS_DMA_TCD10_NBYTES          (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD10_NBYTES_OFFSET)
#define KINETIS_DMA_TCD10_SLAST           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD10_SLAST_OFFSET)
#define KINETIS_DMA_TCD10_DADDR           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD10_DADDR_OFFSET)
#define KINETIS_DMA_TCD10_DOFF            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD10_DOFF_OFFSET)
#define KINETIS_DMA_TCD10_CITER           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD10_CITER_OFFSET)
#define KINETIS_DMA_TCD10_DLASTSGA        (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD10_DLASTSGA_OFFSET)
#define KINETIS_DMA_TCD10_CSR             (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD10_CSR_OFFSET)
#define KINETIS_DMA_TCD10_BITER           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD10_BITER_OFFSET)

#define KINETIS_DMA_TCD11_SADDR           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD11_SADDR_OFFSET)
#define KINETIS_DMA_TCD11_SOFF            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD11_SOFF_OFFSET)
#define KINETIS_DMA_TCD11_ATTR            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD11_ATTR_OFFSET)
#define KINETIS_DMA_TCD11_NBYTES          (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD11_NBYTES_OFFSET)
#define KINETIS_DMA_TCD11_SLAST           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD11_SLAST_OFFSET)
#define KINETIS_DMA_TCD11_DADDR           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD11_DADDR_OFFSET)
#define KINETIS_DMA_TCD11_DOFF            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD11_DOFF_OFFSET)
#define KINETIS_DMA_TCD11_CITER           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD11_CITER_OFFSET)
#define KINETIS_DMA_TCD11_DLASTSGA        (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD11_DLASTSGA_OFFSET)
#define KINETIS_DMA_TCD11_CSR             (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD11_CSR_OFFSET)
#define KINETIS_DMA_TCD11_BITER           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD11_BITER_OFFSET)

#define KINETIS_DMA_TCD12_SADDR           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD12_SADDR_OFFSET)
#define KINETIS_DMA_TCD12_SOFF            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD12_SOFF_OFFSET)
#define KINETIS_DMA_TCD12_ATTR            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD12_ATTR_OFFSET)
#define KINETIS_DMA_TCD12_NBYTES          (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD12_NBYTES_OFFSET)
#define KINETIS_DMA_TCD12_SLAST           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD12_SLAST_OFFSET)
#define KINETIS_DMA_TCD12_DADDR           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD12_DADDR_OFFSET)
#define KINETIS_DMA_TCD12_DOFF            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD12_DOFF_OFFSET)
#define KINETIS_DMA_TCD12_CITER           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD12_CITER_OFFSET)
#define KINETIS_DMA_TCD12_DLASTSGA        (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD12_DLASTSGA_OFFSET)
#define KINETIS_DMA_TCD12_CSR             (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD12_CSR_OFFSET)
#define KINETIS_DMA_TCD12_BITER           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD12_BITER_OFFSET)

#define KINETIS_DMA_TCD13_SADDR           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD13_SADDR_OFFSET)
#define KINETIS_DMA_TCD13_SOFF            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD13_SOFF_OFFSET)
#define KINETIS_DMA_TCD13_ATTR            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD13_ATTR_OFFSET)
#define KINETIS_DMA_TCD13_NBYTES          (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD13_NBYTES_OFFSET)
#define KINETIS_DMA_TCD13_SLAST           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD13_SLAST_OFFSET)
#define KINETIS_DMA_TCD13_DADDR           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD13_DADDR_OFFSET)
#define KINETIS_DMA_TCD13_DOFF            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD13_DOFF_OFFSET)
#define KINETIS_DMA_TCD13_CITER           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD13_CITER_OFFSET)
#define KINETIS_DMA_TCD13_DLASTSGA        (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD13_DLASTSGA_OFFSET)
#define KINETIS_DMA_TCD13_CSR             (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD13_CSR_OFFSET)
#define KINETIS_DMA_TCD13_BITER           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD13_BITER_OFFSET)

#define KINETIS_DMA_TCD14_SADDR           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD14_SADDR_OFFSET)
#define KINETIS_DMA_TCD14_SOFF            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD14_SOFF_OFFSET)
#define KINETIS_DMA_TCD14_ATTR            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD14_ATTR_OFFSET)
#define KINETIS_DMA_TCD14_NBYTES          (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD14_NBYTES_OFFSET)
#define KINETIS_DMA_TCD14_SLAST           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD14_SLAST_OFFSET)
#define KINETIS_DMA_TCD14_DADDR           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD14_DADDR_OFFSET)
#define KINETIS_DMA_TCD14_DOFF            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD14_DOFF_OFFSET)
#define KINETIS_DMA_TCD14_CITER           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD14_CITER_OFFSET)
#define KINETIS_DMA_TCD14_DLASTSGA        (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD14_DLASTSGA_OFFSET)
#define KINETIS_DMA_TCD14_CSR             (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD14_CSR_OFFSET)
#define KINETIS_DMA_TCD14_BITER           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD14_BITER_OFFSET)

#define KINETIS_DMA_TCD15_SADDR           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD15_SADDR_OFFSET)
#define KINETIS_DMA_TCD15_SOFF            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD15_SOFF_OFFSET)
#define KINETIS_DMA_TCD15_ATTR            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD15_ATTR_OFFSET)
#define KINETIS_DMA_TCD15_NBYTES          (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD15_NBYTES_OFFSET)
#define KINETIS_DMA_TCD15_SLAST           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD15_SLAST_OFFSET)
#define KINETIS_DMA_TCD15_DADDR           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD15_DADDR_OFFSET)
#define KINETIS_DMA_TCD15_DOFF            (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD15_DOFF_OFFSET)
#define KINETIS_DMA_TCD15_CITER           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD15_CITER_OFFSET)
#define KINETIS_DMA_TCD15_DLASTSGA        (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD15_DLASTSGA_OFFSET)
#define KINETIS_DMA_TCD15_CSR             (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD15_CSR_OFFSET)
#define KINETIS_DMA_TCD15_BITER           (KINETIS_DMADESC_BASE + KINETIS_DMA_TCD15_BITER_OFFSET)

/* Register Bit Definitions *************************************************/

/* Control Register (32-bit) */

                                                    /* Bit 0:  Reserved */
#define DMA_CR_EDBG                       (1 << 1)  /* Bit 1:  Enable debug */
#define DMA_CR_ERCA                       (1 << 2)  /* Bit 2:  Enable round robin channel arbitration */
#if defined  KINETIS_DMA_HAS_CR_ERGA
#  define DMA_CR_ERGA                     (1 << 3)  /* Bit 3:  Enable round robin group arbitration */
#endif
#define DMA_CR_HOE                        (1 << 4)  /* Bit 4:  Halt on error */
#define DMA_CR_HALT                       (1 << 5)  /* Bit 5:  Halt DMA operations */
#define DMA_CR_CLM                        (1 << 6)  /* Bit 6:  Continuous link mode */
#define DMA_CR_EMLM                       (1 << 7)  /* Bit 7:  Enable minor loop mapping */
#ifdef KINETIS_DMA_HAS_CR_GRP0PRI
#   define DMA_CR_GRP0PRI                 (1 << 8)  /* Bit 8:  Channel Group 0 Priority */
#endif
                                                    /* Bit 9: Reserved */
#ifdef KINETIS_DMA_HAS_CR_GRP1PRI
#  define DMA_CR_GRP1PRI                  (1 << 10) /* Bit 10: Channel Group 1 Priority */
#endif
                                                    /* Bits 11-15: Reserved */
#define DMA_CR_ECX                        (1 << 16) /* Bit 16: Error cancel transfer */
#define DMA_CR_CX                         (1 << 17) /* Bit 17: Cancel transfer */
                                                    /* Bits 18-31: Reserved */

/* Error Status Register */

#define DMA_ES_DBE                        (1 << 0)  /* Bit 0:  Destination bus error */
#define DMA_ES_SBE                        (1 << 1)  /* Bit 1:  Source bus error */
#define DMA_ES_SGE                        (1 << 2)  /* Bit 2:  Scatter/gather configuration error */
#define DMA_ES_NCE                        (1 << 3)  /* Bit 3:  NBYTES/CITER configuration error */
#define DMA_ES_DOE                        (1 << 4)  /* Bit 4:  Destination offset error */
#define DMA_ES_DAE                        (1 << 5)  /* Bit 5:  Destination address error */
#define DMA_ES_SOE                        (1 << 6)  /* Bit 6:  Source offset error */
#define DMA_ES_SAE                        (1 << 7)  /* Bit 7:  Source address error */
#define DMA_ES_ERRCHN_SHIFT               (8)       /* Bits 8-11/12: Error channel number or cancelled channel number */
#define DMA_ES_ERRCHN_MASK                (((1 << KINETIS_DMA_HAS_ES_ERRCHN_BITS) - 1) << DMA_ES_ERRCHN_SHIFT)
                                                    /* Bits 13: Reserved */
#define DMA_ES_CPE                        (1 << 14) /* Bit 14:  Channel priority error */
#ifdef KINETIS_DMA_HAS_ES_GPE
#  define DMA_ES_GPE                      (1 << 15) /* Bit 15:  Group priority error */
#endif
#define DMA_ES_ECX                        (1 << 16) /* Bit 16:  Transfer cancelled */
                                                    /* Bits 17-30: Reserved */
#define DMA_ES_VLD                        (1 << 31) /* Bit 31:  Logical OR of all ERR status bits */

/* Enable Request Register (ERQ), Enable Error Interrupt Register (EEI),
 * Interrupt Request Register (INT), Error Register (ERR),
 * Hardware Request Status Register (HRS) common bit definitions
 */

#define DMA_REQ(n)                        (1 << (n)) /* Bit n: DMA Request n, n=0..<KINETIS_NDMACH */
                                                     /* Bits KINETIS_NDMACH-31: Reserved */

/* Clear Enable Error Interrupt Register (8-bit) */

#define DMA_CEEI_SHIFT                    (0)       /* Bits 0-3/4: Clear enable error interrupt */
#define DMA_CEEI_MASK                     (((1 << KINETIS_DMA_HAS_CEEI_CEEI_BITS) - 1) << DMA_CEEI_SHIFT)
                                                    /* Bits 5: Reserved */
#define DMA_CEEI_CAEE                     (1 << 6)  /* Bit 6:  Clear all enable error interrupts */
#define DMA_CEEI_NOP                      (1 << 7)  /* Bit 7:  No operation */

/* Set Enable Error Interrupt Register (8-bit) */

#define DMA_SEEI_SHIFT                    (0)       /* Bits 0-3/4: Set enable error interrupt */
#define DMA_SEEI_MASK                     (((1 << KINETIS_DMA_HAS_SEEI_SEEI_BITS) - 1) << DMA_SEEI_SHIFT)
                                                    /* Bits 5: Reserved */
#define DMA_SEEI_SAEE                     (1 << 6)  /* Bit 6:  Set all enable error interrupts */
#define DMA_SEEI_NOP                      (1 << 7)  /* Bit 7:  No operation */

/* Clear Enable Request Register (8-bit) */

#define DMA_CERQ_SHIFT                    (0)       /* Bits 0-3: Clear enable request */
#define DMA_CERQ_MASK                     (((1 << KINETIS_DMA_HAS_CERQ_CERQ_BITS) - 1) << DMA_CERQ_SHIFT)
                                                    /* Bits 4-5: Reserved */
#define DMA_CERQ_CAER                     (1 << 6)  /* Bit 6:  Clear all enable requests */
#define DMA_CERQ_NOP                      (1 << 7)  /* Bit 7:  No operation */

/* Set Enable Request Register (8-bit) */

#define DMA_SERQ_SHIFT                    (0)       /* Bits 0-3: Set enable request */
#define DMA_SERQ_MASK                     (((1 << KINETIS_DMA_HAS_SERQ_SERQ_BITS) - 1) << DMA_SERQ_SHIFT)
                                                    /* Bits 4-5: Reserved */
#define DMA_SERQ_SAER                     (1 << 6)  /* Bit 6:  Set all enable requests */
#define DMA_SERQ_NOP                      (1 << 7)  /* Bit 7:  No operation */

/* Clear DONE Status Bit Register (8-bit) */
#define DMA_CDNE_SHIFT                    (0)       /* Bits 0-3: Clear DONE bit */
#define DMA_CDNE_MASK                     (((1 << KINETIS_DMA_HAS_CDNE_CDNE_BITS) - 1) << DMA_CDNE_SHIFT)
                                                    /* Bits 4-5: Reserved */
#define DMA_CDNE_CADN                     (1 << 6)  /* Bit 6:  Clears all DONE bits */
#define DMA_CDNE_NOP                      (1 << 7)  /* Bit 7:  No operation */

/* Set START Bit Register (8-bit) */

#define DMA_SSRT_SHIFT                    (0)       /* Bits 0-3: Set START bit */
#define DMA_SSRT_MASK                     (((1 << KINETIS_DMA_HAS_SSRT_SSRT_BITS) - 1) << DMA_SSRT_SHIFT)
                                                    /* Bits 4-5: Reserved */
#define DMA_SSRT_SAST                     (1 << 6)  /* Bit 6:  Set all START bits (activates all channels) */
#define DMA_SSRT_NOP                      (1 << 7)  /* Bit 7:  No operation */

/* Clear Error Register (8-bit) */

#define DMA_CERR_SHIFT                    (0)       /* Bits 0-3: Clear error indicator */
#define DMA_CERR_MASK                     (((1 << KINETIS_DMA_HAS_CERR_CERR_BITS) - 1) << DMA_CERR_SHIFT)
                                                    /* Bits 4-5: Reserved */
#define DMA_CERR_CAEI                     (1 << 6)  /* Bit 6:  Clear all error indicators */
#define DMA_CERR_NOP                      (1 << 7)  /* Bit 7:  No operation */

/* Clear Interrupt Request Register (8-bit) */

#define DMA_CINT_SHIFT                    (0)       /* Bits 0-3: Clear interrupt request */
#define DMA_CINT_MASK                     (((1 << KINETIS_DMA_HAS_CINT_CINT_BITS) - 1) << DMA_CINT_SHIFT)
                                                    /* Bits 4-5: Reserved */
#define DMA_CINT_CAIR                     (1 << 6)  /* Bit 6:  Clear all interrupt requests */
#define DMA_CINT_NOP                      (1 << 7)  /* Bit 7:  No operation */

/* Channel n Priority Register (8-bit) */

#define DMA_DCHPR_SHIFT                   (0)       /* Bits 0-3: Channel n arbitration priority */
#define DMA_DCHPR_MASK                    (((1 << KINETIS_DMA_HAS_DCHPRI_CHPRI_BITS) - 1) << DMA_DCHPR_SHIFT)
#ifdef KINETIS_DMA_HAS_DCHPRI_GRPPRI
#  define DMA_DCHPR_GRPPRI                (1 << 4)  /* Bits 4-5: Channel n Current Group Priority */
#endif
#define DMA_DCHPR_DPA                     (1 << 6)  /* Bit 6:  Disable preempt ability */
#define DMA_DCHPR_ECP                     (1 << 7)  /* Bit 7:  Enable channel preemption */

/* Enable Asynchronous Request in Stop Register (32-bit) */

#ifdef KINETIS_DMA_HAS_EARS
#  define DMA_EARS(n)                      (1 << (n)) /* Bit n: DMA EARS n, n=0..<KINETIS_NDMACH */
#endif

/* TCD Source Address.  32-bit address value. */

/* TCD Signed Source Address Offset.  32-bit offset value. */

/* TCD Transfer Attributes (16-bit) */

#define DMA_TCD_ATTR_DSIZE_SHIFT          (0)       /* Bits 0-2: Destination data transfer size */
#define DMA_TCD_ATTR_DSIZE_MASK           (7 << DMA_TCD_ATTR_DSIZE_SHIFT)
#  define DMA_TCD_ATTR_DSIZE_8BIT         (0 << DMA_TCD_ATTR_DSIZE_SHIFT) /* 8-bit */
#  define DMA_TCD_ATTR_DSIZE_16BIT        (1 << DMA_TCD_ATTR_DSIZE_SHIFT) /* 16-bit */
#  define DMA_TCD_ATTR_DSIZE_32BIT        (2 << DMA_TCD_ATTR_DSIZE_SHIFT) /* 32-bit */
#  define DMA_TCD_ATTR_DSIZE_16BYTE       (4 << DMA_TCD_ATTR_DSIZE_SHIFT) /* 16-byte */

#define DMA_TCD_ATTR_DMOD_SHIFT           (3)       /* Bits 3-7: Destination address modulo */
#define DMA_TCD_ATTR_DMOD_MASK            (31 << DMA_TCD_ATTR_DMOD_SHIFT)
#define DMA_TCD_ATTR_SSIZE_SHIFT          (8)       /* Bits 8-10: Source data transfer size */
#define DMA_TCD_ATTR_SSIZE_MASK           (7 << DMA_TCD_ATTR_SSIZE_SHIFT)
#  define DMA_TCD_ATTR_SSIZE_8BIT         (0 << DMA_TCD_ATTR_SSIZE_SHIFT) /* 8-bit */
#  define DMA_TCD_ATTR_SSIZE_16BIT        (1 << DMA_TCD_ATTR_SSIZE_SHIFT) /* 16-bit */
#  define DMA_TCD_ATTR_SSIZE_32BIT        (2 << DMA_TCD_ATTR_SSIZE_SHIFT) /* 32-bit */
#  define DMA_TCD_ATTR_SSIZE_16BYTE       (4 << DMA_TCD_ATTR_SSIZE_SHIFT) /* 16-byte */

#define DMA_TCD_ATTR_SMOD_SHIFT           (11)      /* Bits 11-15: Source address modulo */
#define DMA_TCD_ATTR_SMOD_MASK            (31 << DMA_TCD_ATTR_SMOD_SHIFT)

/* TCD Minor Byte Count.
 * Case 1: Minor Loop Disabled.
 *         In this case, the register holds a simple 32-bit count value.
 * Case 2: Minor Loop Enabled and Offset Disabled:
 */

#define DMA_TCD_NBYTES2_SHIFT             (0)       /* Bits 0-29: Minor byte transfer count */
#define DMA_TCD_NBYTES2_MASK              (0x3fffffff)
#define DMA_TCD_NBYTES_DMLOE              (1 << 30) /* Bit 30: Destination minor loop offset enable (Case 2&3) */
#define DMA_TCD_NBYTES_SMLOE              (1 << 31) /* Bit 31: Source minor loop offset enable (Case 2&3) */

/* Case 3: (Minor Loop and Offset Enabled): */

#define DMA_TCD_NBYTES3_SHIFT             (0)       /* Bits 0-9: Minor byte transfer count */
#define DMA_TCD_NBYTES3_MASK              (0x3ff << DMA_TCD_NBYTES3_SHIFT)
#define DMA_TCD_NBYTES_MLOFF_SHIFT        (10)      /* Bits 10-29: Sign-extended address offset */
#define DMA_TCD_NBYTES_MLOFF_MASK         (0xfffff << DMA_TCD_NBYTES_MLOFF_SHIFT)
                                                    /* Bit 30: Same as Case 2 */
                                                    /* Bit 31: Same as Case 2 */

/* TCD Last Source Address Adjustment. 32-bit address value. */

/* TCD Destination Address. 32-bit address value. */

/* TCD Signed Destination Address Offset. 32-bit offset value. */

/* TCD Current Minor Loop Link, Major Loop Count. 16-bit.
 * Case 1:  Channel Linking Enabled:
 */

#define DMA_TCD_CITER1_SHIFT              (0)       /* Bits 0-8: Current major iteration count */
#define DMA_TCD_CITER1_MASK               (0x1ff << DMA_TCD_CITER1_SHIFT)
#define DMA_TCD_CITER1_LINKCH_SHIFT       (9)       /* Bits 9-12/13: Link channel number */
#define DMA_TCD_CITER1_LINKCH_MASK        (((1 << KINETIS_DMA_HAS_TCD_CITER1_LINKCH_BITS) - 1) << DMA_TCD_CITER1_LINKCH_SHIFT)
                                                    /* Bits 14: Reserved */
#define DMA_TCD_CITER_ELINK               (1 << 15) /* Bit 15: Enable channel-to-channel linking on minor-loop complete (Case 1&2) */

/* Case 2:  Channel Linking Disabled: */

#define DMA_TCD_CITER2_SHIFT              (0)       /* Bits 0-14: Current major iteration count */
#define DMA_TCD_CITER2_MASK               (0x7fff << DMA_TCD_CITER2_SHIFT)
                                                    /* Bits 15: Same as Case 1 */

/* TCD Last Destination Address Adjustment/Scatter Gather Address.
 * 32-bit address value.
 */

/* TCD Control and Status (16-bit) */

#define DMA_TCD_CSR_START                 (1 << 0)  /* Bit 0:  Channel start */
#define DMA_TCD_CSR_INTMAJOR              (1 << 1)  /* Bit 1:  Enable an interrupt when major iteration count completes */
#define DMA_TCD_CSR_INTHALF               (1 << 2)  /* Bit 2:  Enable an interrupt when major counter is half complete */
#define DMA_TCD_CSR_DREQ                  (1 << 3)  /* Bit 3:  Disable request */
#define DMA_TCD_CSR_ESG                   (1 << 4)  /* Bit 4:  Enable scatter/gather processing */
#define DMA_TCD_CSR_MAJORELINK            (1 << 5)  /* Bit 5:  Enable channel-to-channel linking on major loop complete */
#define DMA_TCD_CSR_ACTIVE                (1 << 6)  /* Bit 6:  Channel active */
#define DMA_TCD_CSR_DONE                  (1 << 7)  /* Bit 7:  Channel done */
#define DMA_TCD_CSR_MAJORLINKCH_SHIFT     (8)       /* Bits 8-11/12: Link channel number */
#define DMA_TCD_CSR_MAJORLINKCH_MASK      (((1 << KINETIS_DMA_HAS_TCD_CSR_MAJORLINKCH_BITS) - 1) << DMA_TCD_CSR_MAJORLINKCH_SHIFT)
                                                    /* Bits 13: Reserved */
#define DMA_TCD_CSR_BWC_SHIFT             (14)      /* Bits 14-15: Bandwidth control */
#define DMA_TCD_CSR_BWC_MASK              (3 << DMA_TCD_CSR_BWC_SHIFT)
#  define DMA_TCD_CSR_BWC_NOSTALLS        (0 << DMA_TCD_CSR_BWC_SHIFT) /* No eDMA engine stalls */
#  define DMA_TCD_CSR_BWC_4CYCLES         (2 << DMA_TCD_CSR_BWC_SHIFT) /* eDMA engine stalls 4 cycles after each R/W */
#  define DMA_TCD_CSR_BWC_8CYCLES         (3 << DMA_TCD_CSR_BWC_SHIFT) /* eDMA engine stalls 8 cycles after each R/W */

/* TCD Beginning Minor Loop Link, Major Loop Count (16-bit).
 *
 * Case 1: Channel Linking Enabled:
 */

#define DMA_TCD_BITER1_SHIFT              (0)       /* Bits 0-8: Starting major iteration count */
#define DMA_TCD_BITER1_MASK               (0x1ff << DMA_TCD_BITER1_SHIFT)
#define DMA_TCD_BITER1_LINKCH_SHIFT       (9)       /* Bits 9-12: Link channel number */
#define DMA_TCD_BITER1_LINKCH_MASK        (((1 << KINETIS_DMA_HAS_TCD_BITER1_LINKCH_BITS) - 1) << DMA_TCD_BITER1_LINKCH_SHIFT)
                                                    /* Bits 13-14: Reserved */
#define DMA_TCD_BITER_ELINK               (1 << 15) /* Bit 15: Enable channel-to-channel linking on minor-loop complete (Case 1&2) */

/* Case 2: Channel Linking Disabled: */

#define DMA_TCD_BITER2_SHIFT              (0)       /* Bits 0-14: Starting major iteration count */
#define DMA_TCD_BITER2_MASK               (0x7fff << DMA_TCD_CITER2_SHIFT)
                                                    /* Bits 15: Same as Case 1 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_DMA_H */
