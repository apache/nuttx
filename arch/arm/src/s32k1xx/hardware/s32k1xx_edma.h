/****************************************************************************
 * arch/arm/src/s32k1xx/hardware/s32k1xx_edma.h
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

#ifndef __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_EDMA_H
#define __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_EDMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/s32k1xx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_ARCH_CHIP_S32K11X)
#  define S32K1XX_EDMA_NCHANNELS              4
#elif defined(CONFIG_ARCH_CHIP_S32K14x)
#  define S32K1XX_EDMA_NCHANNELS              16
#else
#  error "Unknown number of DMA channels for this S32K1xx part"
#endif

/* eDMA Register Offsets ****************************************************/

#define S32K1XX_EDMA_CR_OFFSET                0x0000  /* Control */
#define S32K1XX_EDMA_ES_OFFSET                0x0004  /* Error Status */
#define S32K1XX_EDMA_ERQ_OFFSET               0x000c  /* Enable Request */
#define S32K1XX_EDMA_EEI_OFFSET               0x0014  /* Enable Error Interrupt */
#define S32K1XX_EDMA_CEEI_OFFSET              0x0018  /* Clear Enable Error Interrupt */
#define S32K1XX_EDMA_SEEI_OFFSET              0x0019  /* Set Enable Error Interrupt */
#define S32K1XX_EDMA_CERQ_OFFSET              0x001a  /* Clear Enable Request */
#define S32K1XX_EDMA_SERQ_OFFSET              0x001b  /* Set Enable Request */
#define S32K1XX_EDMA_CDNE_OFFSET              0x001c  /* Clear DONE Status Bit */
#define S32K1XX_EDMA_SSRT_OFFSET              0x001d  /* Set START Bit */
#define S32K1XX_EDMA_CERR_OFFSET              0x001e  /* Clear Error */
#define S32K1XX_EDMA_CINT_OFFSET              0x001f  /* Clear Interrupt Request */
#define S32K1XX_EDMA_INT_OFFSET               0x0024  /* Interrupt Request */
#define S32K1XX_EDMA_ERR_OFFSET               0x002c  /* Error */
#define S32K1XX_EDMA_HRS_OFFSET               0x0034  /* Hardware Request Status */
#define S32K1XX_EDMA_EARS_OFFSET              0x0044  /* Enable Asynchronous Request in Stop */

#define S32K1XX_EDMA_DCHPRI_OFFSET(n)         (0x0100 + ((n) & ~3) + (3 - ((n) & 3)))

#define S32K1XX_EDMA_DCHPRI3_OFFSET           0x0100  /* Channel 3 Priority */
#define S32K1XX_EDMA_DCHPRI2_OFFSET           0x0101  /* Channel 2 Priority */
#define S32K1XX_EDMA_DCHPRI1_OFFSET           0x0102  /* Channel 1 Priority */
#define S32K1XX_EDMA_DCHPRI0_OFFSET           0x0103  /* Channel 0 Priority */
#define S32K1XX_EDMA_DCHPRI7_OFFSET           0x0104  /* Channel 7 Priority */
#define S32K1XX_EDMA_DCHPRI6_OFFSET           0x0105  /* Channel 6 Priority */
#define S32K1XX_EDMA_DCHPRI5_OFFSET           0x0106  /* Channel 5 Priority */
#define S32K1XX_EDMA_DCHPRI4_OFFSET           0x0107  /* Channel 4 Priority */
#define S32K1XX_EDMA_DCHPRI11_OFFSET          0x0108  /* Channel 11 Priority */
#define S32K1XX_EDMA_DCHPRI10_OFFSET          0x0109  /* Channel 10 Priority */
#define S32K1XX_EDMA_DCHPRI9_OFFSET           0x010a  /* Channel 9 Priority */
#define S32K1XX_EDMA_DCHPRI8_OFFSET           0x010b  /* Channel 8 Priority */
#define S32K1XX_EDMA_DCHPRI15_OFFSET          0x010c  /* Channel 15 Priority */
#define S32K1XX_EDMA_DCHPRI14_OFFSET          0x010d  /* Channel 14 Priority */
#define S32K1XX_EDMA_DCHPRI13_OFFSET          0x010e  /* Channel 13 Priority */
#define S32K1XX_EDMA_DCHPRI12_OFFSET          0x010f  /* Channel 12 Priority */

/* Transfer Control Descriptor (TCD) */

#define S32K1XX_EDMA_TCD_OFFSET(n)            (0x1000 + ((n) << 5))
#define S32K1XX_EDMA_TCD_SADDR_OFFSET         0x0000  /* TCD Source Address */
#define S32K1XX_EDMA_TCD_SOFF_OFFSET          0x0004  /* TCD Signed Source Address Offset */
#define S32K1XX_EDMA_TCD_ATTR_OFFSET          0x0006  /* TCD Transfer Attributes */
#define S32K1XX_EDMA_TCD_NBYTES_ML_OFFSET     0x0008  /* TCD Signed Minor Loop Offset / Byte Count */
#define S32K1XX_EDMA_TCD_SLAST_OFFSET         0x000c  /* TCD Last Source Address Adjustment */
#define S32K1XX_EDMA_TCD_DADDR_OFFSET         0x0010  /* TCD Destination Address */
#define S32K1XX_EDMA_TCD_DOFF_OFFSET          0x0014  /* TCD Signed Destination Address Offset */
#define S32K1XX_EDMA_TCD_CITER_ELINK_OFFSET   0x0016  /* TCD Current Minor Loop Link, Major Loop Count */
#define S32K1XX_EDMA_TCD_DLASTSGA_OFFSET      0x0018  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define S32K1XX_EDMA_TCD_CSR_OFFSET           0x001c  /* TCD Control and Status */
#define S32K1XX_EDMA_TCD_BITER_ELINK_OFFSET   0x001e  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define S32K1XX_EDMA_TCD0_SADDR_OFFSET        0x1000  /* TCD Source Address */
#define S32K1XX_EDMA_TCD0_SOFF_OFFSET         0x1004  /* TCD Signed Source Address Offset */
#define S32K1XX_EDMA_TCD0_ATTR_OFFSET         0x1006  /* TCD Transfer Attributes */
#define S32K1XX_EDMA_TCD0_NBYTES_ML_OFFSET    0x1008  /* TCD Signed Minor Loop Offset / Byte Count */
#define S32K1XX_EDMA_TCD0_SLAST_OFFSET        0x100c  /* TCD Last Source Address Adjustment */
#define S32K1XX_EDMA_TCD0_DADDR_OFFSET        0x1010  /* TCD Destination Address */
#define S32K1XX_EDMA_TCD0_DOFF_OFFSET         0x1014  /* TCD Signed Destination Address Offset */
#define S32K1XX_EDMA_TCD0_CITER_ELINK_OFFSET  0x1016  /* TCD Current Minor Loop Link, Major Loop Count */
#define S32K1XX_EDMA_TCD0_DLASTSGA_OFFSET     0x1018  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define S32K1XX_EDMA_TCD0_CSR_OFFSET          0x101c  /* TCD Control and Status */
#define S32K1XX_EDMA_TCD0_BITER_ELINK_OFFSET  0x101e  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define S32K1XX_EDMA_TCD1_SADDR_OFFSET        0x1020  /* TCD Source Address */
#define S32K1XX_EDMA_TCD1_SOFF_OFFSET         0x1024  /* TCD Signed Source Address Offset */
#define S32K1XX_EDMA_TCD1_ATTR_OFFSET         0x1026  /* TCD Transfer Attributes */
#define S32K1XX_EDMA_TCD1_NBYTES_ML_OFFSET    0x1028  /* TCD Signed Minor Loop Offset / Byte Count */
#define S32K1XX_EDMA_TCD1_SLAST_OFFSET        0x102c  /* TCD Last Source Address Adjustment */
#define S32K1XX_EDMA_TCD1_DADDR_OFFSET        0x1030  /* TCD Destination Address */
#define S32K1XX_EDMA_TCD1_DOFF_OFFSET         0x1034  /* TCD Signed Destination Address Offset */
#define S32K1XX_EDMA_TCD1_CITER_ELINK_OFFSET  0x1036  /* TCD Current Minor Loop Link, Major Loop Count */
#define S32K1XX_EDMA_TCD1_DLASTSGA_OFFSET     0x1038  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define S32K1XX_EDMA_TCD1_CSR_OFFSET          0x103c  /* TCD Control and Status */
#define S32K1XX_EDMA_TCD1_BITER_ELINK_OFFSET  0x103e  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define S32K1XX_EDMA_TCD1_SADDR_OFFSET        0x1020  /* TCD Source Address */
#define S32K1XX_EDMA_TCD1_SOFF_OFFSET         0x1024  /* TCD Signed Source Address Offset */
#define S32K1XX_EDMA_TCD1_ATTR_OFFSET         0x1026  /* TCD Transfer Attributes */
#define S32K1XX_EDMA_TCD1_NBYTES_ML_OFFSET    0x1028  /* TCD Signed Minor Loop Offset / Byte Count */
#define S32K1XX_EDMA_TCD1_SLAST_OFFSET        0x102c  /* TCD Last Source Address Adjustment */
#define S32K1XX_EDMA_TCD1_DADDR_OFFSET        0x1030  /* TCD Destination Address */
#define S32K1XX_EDMA_TCD1_DOFF_OFFSET         0x1034  /* TCD Signed Destination Address Offset */
#define S32K1XX_EDMA_TCD1_CITER_ELINK_OFFSET  0x1036  /* TCD Current Minor Loop Link, Major Loop Count */
#define S32K1XX_EDMA_TCD1_DLASTSGA_OFFSET     0x1038  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define S32K1XX_EDMA_TCD1_CSR_OFFSET          0x103c  /* TCD Control and Status */
#define S32K1XX_EDMA_TCD1_BITER_ELINK_OFFSET  0x103e  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define S32K1XX_EDMA_TCD2_SADDR_OFFSET        0x1040  /* TCD Source Address */
#define S32K1XX_EDMA_TCD2_SOFF_OFFSET         0x1044  /* TCD Signed Source Address Offset */
#define S32K1XX_EDMA_TCD2_ATTR_OFFSET         0x1046  /* TCD Transfer Attributes */
#define S32K1XX_EDMA_TCD2_NBYTES_ML_OFFSET    0x1048  /* TCD Signed Minor Loop Offset / Byte Count */
#define S32K1XX_EDMA_TCD2_SLAST_OFFSET        0x104c  /* TCD Last Source Address Adjustment */
#define S32K1XX_EDMA_TCD2_DADDR_OFFSET        0x1050  /* TCD Destination Address */
#define S32K1XX_EDMA_TCD2_DOFF_OFFSET         0x1054  /* TCD Signed Destination Address Offset */
#define S32K1XX_EDMA_TCD2_CITER_ELINK_OFFSET  0x1056  /* TCD Current Minor Loop Link, Major Loop Count */
#define S32K1XX_EDMA_TCD2_DLASTSGA_OFFSET     0x1058  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define S32K1XX_EDMA_TCD2_CSR_OFFSET          0x105c  /* TCD Control and Status */
#define S32K1XX_EDMA_TCD2_BITER_ELINK_OFFSET  0x105e  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define S32K1XX_EDMA_TCD3_SADDR_OFFSET        0x1060  /* TCD Source Address */
#define S32K1XX_EDMA_TCD3_SOFF_OFFSET         0x1064  /* TCD Signed Source Address Offset */
#define S32K1XX_EDMA_TCD3_ATTR_OFFSET         0x1066  /* TCD Transfer Attributes */
#define S32K1XX_EDMA_TCD3_NBYTES_ML_OFFSET    0x1068  /* TCD Signed Minor Loop Offset / Byte Count */
#define S32K1XX_EDMA_TCD3_SLAST_OFFSET        0x106c  /* TCD Last Source Address Adjustment */
#define S32K1XX_EDMA_TCD3_DADDR_OFFSET        0x1070  /* TCD Destination Address */
#define S32K1XX_EDMA_TCD3_DOFF_OFFSET         0x1074  /* TCD Signed Destination Address Offset */
#define S32K1XX_EDMA_TCD3_CITER_ELINK_OFFSET  0x1076  /* TCD Current Minor Loop Link, Major Loop Count */
#define S32K1XX_EDMA_TCD3_DLASTSGA_OFFSET     0x1078  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define S32K1XX_EDMA_TCD3_CSR_OFFSET          0x107c  /* TCD Control and Status */
#define S32K1XX_EDMA_TCD3_BITER_ELINK_OFFSET  0x107e  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define S32K1XX_EDMA_TCD4_SADDR_OFFSET        0x1080  /* TCD Source Address */
#define S32K1XX_EDMA_TCD4_SOFF_OFFSET         0x1084  /* TCD Signed Source Address Offset */
#define S32K1XX_EDMA_TCD4_ATTR_OFFSET         0x1086  /* TCD Transfer Attributes */
#define S32K1XX_EDMA_TCD4_NBYTES_ML_OFFSET    0x1088  /* TCD Signed Minor Loop Offset / Byte Count */
#define S32K1XX_EDMA_TCD4_SLAST_OFFSET        0x108c  /* TCD Last Source Address Adjustment */
#define S32K1XX_EDMA_TCD4_DADDR_OFFSET        0x1090  /* TCD Destination Address */
#define S32K1XX_EDMA_TCD4_DOFF_OFFSET         0x1094  /* TCD Signed Destination Address Offset */
#define S32K1XX_EDMA_TCD4_CITER_ELINK_OFFSET  0x1096  /* TCD Current Minor Loop Link, Major Loop Count */
#define S32K1XX_EDMA_TCD4_DLASTSGA_OFFSET     0x1098  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define S32K1XX_EDMA_TCD4_CSR_OFFSET          0x109c  /* TCD Control and Status */
#define S32K1XX_EDMA_TCD4_BITER_ELINK_OFFSET  0x109e  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define S32K1XX_EDMA_TCD5_SADDR_OFFSET        0x10a0  /* TCD Source Address */
#define S32K1XX_EDMA_TCD5_SOFF_OFFSET         0x10a4  /* TCD Signed Source Address Offset */
#define S32K1XX_EDMA_TCD5_ATTR_OFFSET         0x10a6  /* TCD Transfer Attributes */
#define S32K1XX_EDMA_TCD5_NBYTES_ML_OFFSET    0x10a8  /* TCD Signed Minor Loop Offset / Byte Count */
#define S32K1XX_EDMA_TCD5_SLAST_OFFSET        0x10ac  /* TCD Last Source Address Adjustment */
#define S32K1XX_EDMA_TCD5_DADDR_OFFSET        0x10b0  /* TCD Destination Address */
#define S32K1XX_EDMA_TCD5_DOFF_OFFSET         0x10b4  /* TCD Signed Destination Address Offset */
#define S32K1XX_EDMA_TCD5_CITER_ELINK_OFFSET  0x10b6  /* TCD Current Minor Loop Link, Major Loop Count */
#define S32K1XX_EDMA_TCD5_DLASTSGA_OFFSET     0x10b8  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define S32K1XX_EDMA_TCD5_CSR_OFFSET          0x10bc  /* TCD Control and Status */
#define S32K1XX_EDMA_TCD5_BITER_ELINK_OFFSET  0x10be  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define S32K1XX_EDMA_TCD6_SADDR_OFFSET        0x10c0  /* TCD Source Address */
#define S32K1XX_EDMA_TCD6_SOFF_OFFSET         0x10c4  /* TCD Signed Source Address Offset */
#define S32K1XX_EDMA_TCD6_ATTR_OFFSET         0x10c6  /* TCD Transfer Attributes */
#define S32K1XX_EDMA_TCD6_NBYTES_ML_OFFSET    0x10c8  /* TCD Signed Minor Loop Offset / Byte Count */
#define S32K1XX_EDMA_TCD6_SLAST_OFFSET        0x10cc  /* TCD Last Source Address Adjustment */
#define S32K1XX_EDMA_TCD6_DADDR_OFFSET        0x10d0  /* TCD Destination Address */
#define S32K1XX_EDMA_TCD6_DOFF_OFFSET         0x10d4  /* TCD Signed Destination Address Offset */
#define S32K1XX_EDMA_TCD6_CITER_ELINK_OFFSET  0x10d6  /* TCD Current Minor Loop Link, Major Loop Count */
#define S32K1XX_EDMA_TCD6_DLASTSGA_OFFSET     0x10d8  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define S32K1XX_EDMA_TCD6_CSR_OFFSET          0x10dc  /* TCD Control and Status */
#define S32K1XX_EDMA_TCD6_BITER_ELINK_OFFSET  0x10de  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define S32K1XX_EDMA_TCD7_SADDR_OFFSET        0x10e0  /* TCD Source Address */
#define S32K1XX_EDMA_TCD7_SOFF_OFFSET         0x10e4  /* TCD Signed Source Address Offset */
#define S32K1XX_EDMA_TCD7_ATTR_OFFSET         0x10e6  /* TCD Transfer Attributes */
#define S32K1XX_EDMA_TCD7_NBYTES_ML_OFFSET    0x10e8  /* TCD Signed Minor Loop Offset / Byte Count */
#define S32K1XX_EDMA_TCD7_SLAST_OFFSET        0x10ec  /* TCD Last Source Address Adjustment */
#define S32K1XX_EDMA_TCD7_DADDR_OFFSET        0x10f0  /* TCD Destination Address */
#define S32K1XX_EDMA_TCD7_DOFF_OFFSET         0x10f4  /* TCD Signed Destination Address Offset */
#define S32K1XX_EDMA_TCD7_CITER_ELINK_OFFSET  0x10f6  /* TCD Current Minor Loop Link, Major Loop Count */
#define S32K1XX_EDMA_TCD7_DLASTSGA_OFFSET     0x10f8  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define S32K1XX_EDMA_TCD7_CSR_OFFSET          0x10fc  /* TCD Control and Status */
#define S32K1XX_EDMA_TCD7_BITER_ELINK_OFFSET  0x10fe  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define S32K1XX_EDMA_TCD8_SADDR_OFFSET        0x1100  /* TCD Source Address */
#define S32K1XX_EDMA_TCD8_SOFF_OFFSET         0x1104  /* TCD Signed Source Address Offset */
#define S32K1XX_EDMA_TCD8_ATTR_OFFSET         0x1106  /* TCD Transfer Attributes */
#define S32K1XX_EDMA_TCD8_NBYTES_ML_OFFSET    0x1108  /* TCD Signed Minor Loop Offset / Byte Count */
#define S32K1XX_EDMA_TCD8_SLAST_OFFSET        0x110c  /* TCD Last Source Address Adjustment */
#define S32K1XX_EDMA_TCD8_DADDR_OFFSET        0x1110  /* TCD Destination Address */
#define S32K1XX_EDMA_TCD8_DOFF_OFFSET         0x1114  /* TCD Signed Destination Address Offset */
#define S32K1XX_EDMA_TCD8_CITER_ELINK_OFFSET  0x1116  /* TCD Current Minor Loop Link, Major Loop Count */
#define S32K1XX_EDMA_TCD8_DLASTSGA_OFFSET     0x1118  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define S32K1XX_EDMA_TCD8_CSR_OFFSET          0x111c  /* TCD Control and Status */
#define S32K1XX_EDMA_TCD8_BITER_ELINK_OFFSET  0x111e  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define S32K1XX_EDMA_TCD9_SADDR_OFFSET        0x1120  /* TCD Source Address */
#define S32K1XX_EDMA_TCD9_SOFF_OFFSET         0x1124  /* TCD Signed Source Address Offset */
#define S32K1XX_EDMA_TCD9_ATTR_OFFSET         0x1126  /* TCD Transfer Attributes */
#define S32K1XX_EDMA_TCD9_NBYTES_ML_OFFSET    0x1128  /* TCD Signed Minor Loop Offset / Byte Count */
#define S32K1XX_EDMA_TCD9_SLAST_OFFSET        0x112c  /* TCD Last Source Address Adjustment */
#define S32K1XX_EDMA_TCD9_DADDR_OFFSET        0x1130  /* TCD Destination Address */
#define S32K1XX_EDMA_TCD9_DOFF_OFFSET         0x1134  /* TCD Signed Destination Address Offset */
#define S32K1XX_EDMA_TCD9_CITER_ELINK_OFFSET  0x1136  /* TCD Current Minor Loop Link, Major Loop Count */
#define S32K1XX_EDMA_TCD9_DLASTSGA_OFFSET     0x1138  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define S32K1XX_EDMA_TCD9_CSR_OFFSET          0x113c  /* TCD Control and Status */
#define S32K1XX_EDMA_TCD9_BITER_ELINK_OFFSET  0x113e  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define S32K1XX_EDMA_TCD10_SADDR_OFFSET       0x1140  /* TCD Source Address */
#define S32K1XX_EDMA_TCD10_SOFF_OFFSET        0x1144  /* TCD Signed Source Address Offset */
#define S32K1XX_EDMA_TCD10_ATTR_OFFSET        0x1146  /* TCD Transfer Attributes */
#define S32K1XX_EDMA_TCD10_NBYTES_ML_OFFSET   0x1148  /* TCD Signed Minor Loop Offset / Byte Count */
#define S32K1XX_EDMA_TCD10_SLAST_OFFSET       0x114c  /* TCD Last Source Address Adjustment */
#define S32K1XX_EDMA_TCD10_DADDR_OFFSET       0x1150  /* TCD Destination Address */
#define S32K1XX_EDMA_TCD10_DOFF_OFFSET        0x1154  /* TCD Signed Destination Address Offset */
#define S32K1XX_EDMA_TCD10_CITER_ELINK_OFFSET 0x1156  /* TCD Current Minor Loop Link, Major Loop Count */
#define S32K1XX_EDMA_TCD10_DLASTSGA_OFFSET    0x1158  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define S32K1XX_EDMA_TCD10_CSR_OFFSET         0x115c  /* TCD Control and Status */
#define S32K1XX_EDMA_TCD10_BITER_ELINK_OFFSET 0x115e  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define S32K1XX_EDMA_TCD11_SADDR_OFFSET       0x1160  /* TCD Source Address */
#define S32K1XX_EDMA_TCD11_SOFF_OFFSET        0x1164  /* TCD Signed Source Address Offset */
#define S32K1XX_EDMA_TCD11_ATTR_OFFSET        0x1166  /* TCD Transfer Attributes */
#define S32K1XX_EDMA_TCD11_NBYTES_ML_OFFSET   0x1168  /* TCD Signed Minor Loop Offset / Byte Count */
#define S32K1XX_EDMA_TCD11_SLAST_OFFSET       0x116c  /* TCD Last Source Address Adjustment */
#define S32K1XX_EDMA_TCD11_DADDR_OFFSET       0x1170  /* TCD Destination Address */
#define S32K1XX_EDMA_TCD11_DOFF_OFFSET        0x1174  /* TCD Signed Destination Address Offset */
#define S32K1XX_EDMA_TCD11_CITER_ELINK_OFFSET 0x1176  /* TCD Current Minor Loop Link, Major Loop Count */
#define S32K1XX_EDMA_TCD11_DLASTSGA_OFFSET    0x1178  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define S32K1XX_EDMA_TCD11_CSR_OFFSET         0x117c  /* TCD Control and Status */
#define S32K1XX_EDMA_TCD11_BITER_ELINK_OFFSET 0x117e  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define S32K1XX_EDMA_TCD12_SADDR_OFFSET       0x1180  /* TCD Source Address */
#define S32K1XX_EDMA_TCD12_SOFF_OFFSET        0x1184  /* TCD Signed Source Address Offset */
#define S32K1XX_EDMA_TCD12_ATTR_OFFSET        0x1186  /* TCD Transfer Attributes */
#define S32K1XX_EDMA_TCD12_NBYTES_ML_OFFSET   0x1188  /* TCD Signed Minor Loop Offset / Byte Count */
#define S32K1XX_EDMA_TCD12_SLAST_OFFSET       0x118c  /* TCD Last Source Address Adjustment */
#define S32K1XX_EDMA_TCD12_DADDR_OFFSET       0x1190  /* TCD Destination Address */
#define S32K1XX_EDMA_TCD12_DOFF_OFFSET        0x1194  /* TCD Signed Destination Address Offset */
#define S32K1XX_EDMA_TCD12_CITER_ELINK_OFFSET 0x1196  /* TCD Current Minor Loop Link, Major Loop Count */
#define S32K1XX_EDMA_TCD12_DLASTSGA_OFFSET    0x1198  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define S32K1XX_EDMA_TCD12_CSR_OFFSET         0x119c  /* TCD Control and Status */
#define S32K1XX_EDMA_TCD12_BITER_ELINK_OFFSET 0x119e  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define S32K1XX_EDMA_TCD13_SADDR_OFFSET       0x11a0  /* TCD Source Address */
#define S32K1XX_EDMA_TCD13_SOFF_OFFSET        0x11a4  /* TCD Signed Source Address Offset */
#define S32K1XX_EDMA_TCD13_ATTR_OFFSET        0x11a6  /* TCD Transfer Attributes */
#define S32K1XX_EDMA_TCD13_NBYTES_ML_OFFSET   0x11a8  /* TCD Signed Minor Loop Offset / Byte Count */
#define S32K1XX_EDMA_TCD13_SLAST_OFFSET       0x11ac  /* TCD Last Source Address Adjustment */
#define S32K1XX_EDMA_TCD13_DADDR_OFFSET       0x11b0  /* TCD Destination Address */
#define S32K1XX_EDMA_TCD13_DOFF_OFFSET        0x11b4  /* TCD Signed Destination Address Offset */
#define S32K1XX_EDMA_TCD13_CITER_ELINK_OFFSET 0x11b6  /* TCD Current Minor Loop Link, Major Loop Count */
#define S32K1XX_EDMA_TCD13_DLASTSGA_OFFSET    0x11b8  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define S32K1XX_EDMA_TCD13_CSR_OFFSET         0x11bc  /* TCD Control and Status */
#define S32K1XX_EDMA_TCD13_BITER_ELINK_OFFSET 0x11be  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define S32K1XX_EDMA_TCD14_SADDR_OFFSET       0x11c0  /* TCD Source Address */
#define S32K1XX_EDMA_TCD14_SOFF_OFFSET        0x11c4  /* TCD Signed Source Address Offset */
#define S32K1XX_EDMA_TCD14_ATTR_OFFSET        0x11c6  /* TCD Transfer Attributes */
#define S32K1XX_EDMA_TCD14_NBYTES_ML_OFFSET   0x11c8  /* TCD Signed Minor Loop Offset / Byte Count */
#define S32K1XX_EDMA_TCD14_SLAST_OFFSET       0x11cc  /* TCD Last Source Address Adjustment */
#define S32K1XX_EDMA_TCD14_DADDR_OFFSET       0x11d0  /* TCD Destination Address */
#define S32K1XX_EDMA_TCD14_DOFF_OFFSET        0x11d4  /* TCD Signed Destination Address Offset */
#define S32K1XX_EDMA_TCD14_CITER_ELINK_OFFSET 0x11d6  /* TCD Current Minor Loop Link, Major Loop Count */
#define S32K1XX_EDMA_TCD14_DLASTSGA_OFFSET    0x11d8  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define S32K1XX_EDMA_TCD14_CSR_OFFSET         0x11dc  /* TCD Control and Status */
#define S32K1XX_EDMA_TCD14_BITER_ELINK_OFFSET 0x11de  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define S32K1XX_EDMA_TCD15_SADDR_OFFSET       0x11e0  /* TCD Source Address */
#define S32K1XX_EDMA_TCD15_SOFF_OFFSET        0x11e4  /* TCD Signed Source Address Offset */
#define S32K1XX_EDMA_TCD15_ATTR_OFFSET        0x11e6  /* TCD Transfer Attributes */
#define S32K1XX_EDMA_TCD15_NBYTES_ML_OFFSET   0x11e8  /* TCD Signed Minor Loop Offset / Byte Count */
#define S32K1XX_EDMA_TCD15_SLAST_OFFSET       0x11ec  /* TCD Last Source Address Adjustment */
#define S32K1XX_EDMA_TCD15_DADDR_OFFSET       0x11f0  /* TCD Destination Address */
#define S32K1XX_EDMA_TCD15_DOFF_OFFSET        0x11f4  /* TCD Signed Destination Address Offset */
#define S32K1XX_EDMA_TCD15_CITER_ELINK_OFFSET 0x11f6  /* TCD Current Minor Loop Link, Major Loop Count */
#define S32K1XX_EDMA_TCD15_DLASTSGA_OFFSET    0x11f8  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define S32K1XX_EDMA_TCD15_CSR_OFFSET         0x11fc  /* TCD Control and Status */
#define S32K1XX_EDMA_TCD15_BITER_ELINK_OFFSET 0x11fe  /* TCD Beginning Minor Loop Link, Major Loop Count */

/* eDMA Register Addresses **************************************************/

#define S32K1XX_EDMA_CR                       (S32K1XX_EDMA_BASE + S32K1XX_EDMA_CR_OFFSET)
#define S32K1XX_EDMA_ES                       (S32K1XX_EDMA_BASE + S32K1XX_EDMA_ES_OFFSET)
#define S32K1XX_EDMA_ERQ                      (S32K1XX_EDMA_BASE + S32K1XX_EDMA_ERQ_OFFSET)
#define S32K1XX_EDMA_EEI                      (S32K1XX_EDMA_BASE + S32K1XX_EDMA_EEI_OFFSET)
#define S32K1XX_EDMA_CEEI                     (S32K1XX_EDMA_BASE + S32K1XX_EDMA_CEEI_OFFSET)
#define S32K1XX_EDMA_SEEI                     (S32K1XX_EDMA_BASE + S32K1XX_EDMA_SEEI_OFFSET)
#define S32K1XX_EDMA_CERQ                     (S32K1XX_EDMA_BASE + S32K1XX_EDMA_CERQ_OFFSET)
#define S32K1XX_EDMA_SERQ                     (S32K1XX_EDMA_BASE + S32K1XX_EDMA_SERQ_OFFSET)
#define S32K1XX_EDMA_CDNE                     (S32K1XX_EDMA_BASE + S32K1XX_EDMA_CDNE_OFFSET)
#define S32K1XX_EDMA_SSRT                     (S32K1XX_EDMA_BASE + S32K1XX_EDMA_SSRT_OFFSET)
#define S32K1XX_EDMA_CERR                     (S32K1XX_EDMA_BASE + S32K1XX_EDMA_CERR_OFFSET)
#define S32K1XX_EDMA_CINT                     (S32K1XX_EDMA_BASE + S32K1XX_EDMA_CINT_OFFSET)
#define S32K1XX_EDMA_INT                      (S32K1XX_EDMA_BASE + S32K1XX_EDMA_INT_OFFSET)
#define S32K1XX_EDMA_ERR                      (S32K1XX_EDMA_BASE + S32K1XX_EDMA_ERR_OFFSET)
#define S32K1XX_EDMA_HRS                      (S32K1XX_EDMA_BASE + S32K1XX_EDMA_HRS_OFFSET)
#define S32K1XX_EDMA_EARS                     (S32K1XX_EDMA_BASE + S32K1XX_EDMA_EARS_OFFSET)

#define S32K1XX_EDMA_DCHPRI(n)                (S32K1XX_EDMA_BASE + S32K1XX_EDMA_DCHPRI_OFFSET(n))

#define S32K1XX_EDMA_DCHPRI0                  (S32K1XX_EDMA_BASE + S32K1XX_EDMA_DCHPRI0_OFFSET)
#define S32K1XX_EDMA_DCHPRI1                  (S32K1XX_EDMA_BASE + S32K1XX_EDMA_DCHPRI1_OFFSET)
#define S32K1XX_EDMA_DCHPRI2                  (S32K1XX_EDMA_BASE + S32K1XX_EDMA_DCHPRI2_OFFSET)
#define S32K1XX_EDMA_DCHPRI3                  (S32K1XX_EDMA_BASE + S32K1XX_EDMA_DCHPRI3_OFFSET)
#define S32K1XX_EDMA_DCHPRI4                  (S32K1XX_EDMA_BASE + S32K1XX_EDMA_DCHPRI4_OFFSET)
#define S32K1XX_EDMA_DCHPRI5                  (S32K1XX_EDMA_BASE + S32K1XX_EDMA_DCHPRI5_OFFSET)
#define S32K1XX_EDMA_DCHPRI6                  (S32K1XX_EDMA_BASE + S32K1XX_EDMA_DCHPRI6_OFFSET)
#define S32K1XX_EDMA_DCHPRI7                  (S32K1XX_EDMA_BASE + S32K1XX_EDMA_DCHPRI7_OFFSET)
#define S32K1XX_EDMA_DCHPRI8                  (S32K1XX_EDMA_BASE + S32K1XX_EDMA_DCHPRI8_OFFSET)
#define S32K1XX_EDMA_DCHPRI9                  (S32K1XX_EDMA_BASE + S32K1XX_EDMA_DCHPRI9_OFFSET)
#define S32K1XX_EDMA_DCHPRI10                 (S32K1XX_EDMA_BASE + S32K1XX_EDMA_DCHPRI10_OFFSET)
#define S32K1XX_EDMA_DCHPRI11                 (S32K1XX_EDMA_BASE + S32K1XX_EDMA_DCHPRI11_OFFSET)
#define S32K1XX_EDMA_DCHPRI12                 (S32K1XX_EDMA_BASE + S32K1XX_EDMA_DCHPRI12_OFFSET)
#define S32K1XX_EDMA_DCHPRI13                 (S32K1XX_EDMA_BASE + S32K1XX_EDMA_DCHPRI13_OFFSET)
#define S32K1XX_EDMA_DCHPRI14                 (S32K1XX_EDMA_BASE + S32K1XX_EDMA_DCHPRI14_OFFSET)
#define S32K1XX_EDMA_DCHPRI15                 (S32K1XX_EDMA_BASE + S32K1XX_EDMA_DCHPRI15_OFFSET)

/* Transfer Control Descriptor (TCD) */

#define S32K1XX_EDMA_TCD_BASE(n)              (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD_OFFSET(n))
#define S32K1XX_EDMA_TCD_SADDR(n)             (S32K1XX_EDMA_TCD_BASE(n) + S32K1XX_EDMA_TCD_SADDR_OFFSET)
#define S32K1XX_EDMA_TCD_SOFF(n)              (S32K1XX_EDMA_TCD_BASE(n) + S32K1XX_EDMA_TCD_SOFF_OFFSET)
#define S32K1XX_EDMA_TCD_ATTR(n)              (S32K1XX_EDMA_TCD_BASE(n) + S32K1XX_EDMA_TCD_ATTR_OFFSET)
#define S32K1XX_EDMA_TCD_NBYTES_ML(n)         (S32K1XX_EDMA_TCD_BASE(n) + S32K1XX_EDMA_TCD_NBYTES_ML_OFFSET)
#define S32K1XX_EDMA_TCD_SLAST(n)             (S32K1XX_EDMA_TCD_BASE(n) + S32K1XX_EDMA_TCD_SLAST_OFFSET)
#define S32K1XX_EDMA_TCD_DADDR(n)             (S32K1XX_EDMA_TCD_BASE(n) + S32K1XX_EDMA_TCD_DADDR_OFFSET)
#define S32K1XX_EDMA_TCD_DOFF(n)              (S32K1XX_EDMA_TCD_BASE(n) + S32K1XX_EDMA_TCD_DOFF_OFFSET)
#define S32K1XX_EDMA_TCD_CITER_ELINK(n)       (S32K1XX_EDMA_TCD_BASE(n) + S32K1XX_EDMA_TCD_CITER_ELINK_OFFSET)
#define S32K1XX_EDMA_TCD_DLASTSGA(n)          (S32K1XX_EDMA_TCD_BASE(n) + S32K1XX_EDMA_TCD_DLASTSGA_OFFSET)
#define S32K1XX_EDMA_TCD_CSR(n)               (S32K1XX_EDMA_TCD_BASE(n) + S32K1XX_EDMA_TCD_CSR_OFFSET)
#define S32K1XX_EDMA_TCD_BITER_ELINK(n)       (S32K1XX_EDMA_TCD_BASE(n) + S32K1XX_EDMA_TCD_BITER_ELINK_OFFSET)

#define S32K1XX_EDMA_TCD0_SADDR               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD0_SADDR_OFFSET)
#define S32K1XX_EDMA_TCD0_SOFF                (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD0_SOFF_OFFSET)
#define S32K1XX_EDMA_TCD0_ATTR                (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD0_ATTR_OFFSET)
#define S32K1XX_EDMA_TCD0_NBYTES_ML           (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD0_NBYTES_ML_OFFSET)
#define S32K1XX_EDMA_TCD0_SLAST               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD0_SLAST_OFFSET)
#define S32K1XX_EDMA_TCD0_DADDR               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD0_DADDR_OFFSET)
#define S32K1XX_EDMA_TCD0_DOFF                (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD0_DOFF_OFFSET)
#define S32K1XX_EDMA_TCD0_CITER_ELINK         (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD0_CITER_ELINK_OFFSET)
#define S32K1XX_EDMA_TCD0_DLASTSGA            (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD0_DLASTSGA_OFFSET)
#define S32K1XX_EDMA_TCD0_CSR                 (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD0_CSR_OFFSET)
#define S32K1XX_EDMA_TCD0_BITER_ELINK         (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD0_BITER_ELINK_OFFSET)

#define S32K1XX_EDMA_TCD1_SADDR               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD1_SADDR_OFFSET)
#define S32K1XX_EDMA_TCD1_SOFF                (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD1_SOFF_OFFSET)
#define S32K1XX_EDMA_TCD1_ATTR                (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD1_ATTR_OFFSET)
#define S32K1XX_EDMA_TCD1_NBYTES_ML           (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD1_NBYTES_ML_OFFSET)
#define S32K1XX_EDMA_TCD1_SLAST               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD1_SLAST_OFFSET)
#define S32K1XX_EDMA_TCD1_DADDR               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD1_DADDR_OFFSET)
#define S32K1XX_EDMA_TCD1_DOFF                (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD1_DOFF_OFFSET)
#define S32K1XX_EDMA_TCD1_CITER_ELINK         (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD1_CITER_ELINK_OFFSET)
#define S32K1XX_EDMA_TCD1_DLASTSGA            (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD1_DLASTSGA_OFFSET)
#define S32K1XX_EDMA_TCD1_CSR                 (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD1_CSR_OFFSET)
#define S32K1XX_EDMA_TCD1_BITER_ELINK         (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD1_BITER_ELINK_OFFSET)

#define S32K1XX_EDMA_TCD2_SADDR               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD2_SADDR_OFFSET)
#define S32K1XX_EDMA_TCD2_SOFF                (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD2_SOFF_OFFSET)
#define S32K1XX_EDMA_TCD2_ATTR                (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD2_ATTR_OFFSET)
#define S32K1XX_EDMA_TCD2_NBYTES_ML           (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD2_NBYTES_ML_OFFSET)
#define S32K1XX_EDMA_TCD2_SLAST               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD2_SLAST_OFFSET)
#define S32K1XX_EDMA_TCD2_DADDR               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD2_DADDR_OFFSET)
#define S32K1XX_EDMA_TCD2_DOFF                (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD2_DOFF_OFFSET)
#define S32K1XX_EDMA_TCD2_CITER_ELINK         (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD2_CITER_ELINK_OFFSET)
#define S32K1XX_EDMA_TCD2_DLASTSGA            (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD2_DLASTSGA_OFFSET)
#define S32K1XX_EDMA_TCD2_CSR                 (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD2_CSR_OFFSET)
#define S32K1XX_EDMA_TCD2_BITER_ELINK         (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD2_BITER_ELINK_OFFSET)

#define S32K1XX_EDMA_TCD3_SADDR               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD3_SADDR_OFFSET)
#define S32K1XX_EDMA_TCD3_SOFF                (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD3_SOFF_OFFSET)
#define S32K1XX_EDMA_TCD3_ATTR                (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD3_ATTR_OFFSET)
#define S32K1XX_EDMA_TCD3_NBYTES_ML           (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD3_NBYTES_ML_OFFSET)
#define S32K1XX_EDMA_TCD3_SLAST               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD3_SLAST_OFFSET)
#define S32K1XX_EDMA_TCD3_DADDR               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD3_DADDR_OFFSET)
#define S32K1XX_EDMA_TCD3_DOFF                (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD3_DOFF_OFFSET)
#define S32K1XX_EDMA_TCD3_CITER_ELINK         (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD3_CITER_ELINK_OFFSET)
#define S32K1XX_EDMA_TCD3_DLASTSGA            (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD3_DLASTSGA_OFFSET)
#define S32K1XX_EDMA_TCD3_CSR                 (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD3_CSR_OFFSET)
#define S32K1XX_EDMA_TCD3_BITER_ELINK         (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD3_BITER_ELINK_OFFSET)

#define S32K1XX_EDMA_TCD4_SADDR               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD4_SADDR_OFFSET)
#define S32K1XX_EDMA_TCD4_SOFF                (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD4_SOFF_OFFSET)
#define S32K1XX_EDMA_TCD4_ATTR                (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD4_ATTR_OFFSET)
#define S32K1XX_EDMA_TCD4_NBYTES_ML           (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD4_NBYTES_ML_OFFSET)
#define S32K1XX_EDMA_TCD4_SLAST               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD4_SLAST_OFFSET)
#define S32K1XX_EDMA_TCD4_DADDR               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD4_DADDR_OFFSET)
#define S32K1XX_EDMA_TCD4_DOFF                (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD4_DOFF_OFFSET)
#define S32K1XX_EDMA_TCD4_CITER_ELINK         (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD4_CITER_ELINK_OFFSET)
#define S32K1XX_EDMA_TCD4_DLASTSGA            (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD4_DLASTSGA_OFFSET)
#define S32K1XX_EDMA_TCD4_CSR                 (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD4_CSR_OFFSET)
#define S32K1XX_EDMA_TCD4_BITER_ELINK         (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD4_BITER_ELINK_OFFSET)

#define S32K1XX_EDMA_TCD5_SADDR               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD5_SADDR_OFFSET)
#define S32K1XX_EDMA_TCD5_SOFF                (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD5_SOFF_OFFSET)
#define S32K1XX_EDMA_TCD5_ATTR                (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD5_ATTR_OFFSET)
#define S32K1XX_EDMA_TCD5_NBYTES_ML           (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD5_NBYTES_ML_OFFSET)
#define S32K1XX_EDMA_TCD5_SLAST               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD5_SLAST_OFFSET)
#define S32K1XX_EDMA_TCD5_DADDR               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD5_DADDR_OFFSET)
#define S32K1XX_EDMA_TCD5_DOFF                (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD5_DOFF_OFFSET)
#define S32K1XX_EDMA_TCD5_CITER_ELINK         (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD5_CITER_ELINK_OFFSET)
#define S32K1XX_EDMA_TCD5_DLASTSGA            (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD5_DLASTSGA_OFFSET)
#define S32K1XX_EDMA_TCD5_CSR                 (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD5_CSR_OFFSET)
#define S32K1XX_EDMA_TCD5_BITER_ELINK         (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD5_BITER_ELINK_OFFSET)

#define S32K1XX_EDMA_TCD6_SADDR               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD6_SADDR_OFFSET)
#define S32K1XX_EDMA_TCD6_SOFF                (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD6_SOFF_OFFSET)
#define S32K1XX_EDMA_TCD6_ATTR                (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD6_ATTR_OFFSET)
#define S32K1XX_EDMA_TCD6_NBYTES_ML           (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD6_NBYTES_ML_OFFSET)
#define S32K1XX_EDMA_TCD6_SLAST               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD6_SLAST_OFFSET)
#define S32K1XX_EDMA_TCD6_DADDR               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD6_DADDR_OFFSET)
#define S32K1XX_EDMA_TCD6_DOFF                (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD6_DOFF_OFFSET)
#define S32K1XX_EDMA_TCD6_CITER_ELINK         (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD6_CITER_ELINK_OFFSET)
#define S32K1XX_EDMA_TCD6_DLASTSGA            (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD6_DLASTSGA_OFFSET)
#define S32K1XX_EDMA_TCD6_CSR                 (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD6_CSR_OFFSET)
#define S32K1XX_EDMA_TCD6_BITER_ELINK         (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD6_BITER_ELINK_OFFSET)

#define S32K1XX_EDMA_TCD7_SADDR               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD7_SADDR_OFFSET)
#define S32K1XX_EDMA_TCD7_SOFF                (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD7_SOFF_OFFSET)
#define S32K1XX_EDMA_TCD7_ATTR                (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD7_ATTR_OFFSET)
#define S32K1XX_EDMA_TCD7_NBYTES_ML           (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD7_NBYTES_ML_OFFSET)
#define S32K1XX_EDMA_TCD7_SLAST               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD7_SLAST_OFFSET)
#define S32K1XX_EDMA_TCD7_DADDR               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD7_DADDR_OFFSET)
#define S32K1XX_EDMA_TCD7_DOFF                (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD7_DOFF_OFFSET)
#define S32K1XX_EDMA_TCD7_CITER_ELINK         (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD7_CITER_ELINK_OFFSET)
#define S32K1XX_EDMA_TCD7_DLASTSGA            (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD7_DLASTSGA_OFFSET)
#define S32K1XX_EDMA_TCD7_CSR                 (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD7_CSR_OFFSET)
#define S32K1XX_EDMA_TCD7_BITER_ELINK         (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD7_BITER_ELINK_OFFSET)

#define S32K1XX_EDMA_TCD8_SADDR               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD8_SADDR_OFFSET)
#define S32K1XX_EDMA_TCD8_SOFF                (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD8_SOFF_OFFSET)
#define S32K1XX_EDMA_TCD8_ATTR                (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD8_ATTR_OFFSET)
#define S32K1XX_EDMA_TCD8_NBYTES_ML           (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD8_NBYTES_ML_OFFSET)
#define S32K1XX_EDMA_TCD8_SLAST               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD8_SLAST_OFFSET)
#define S32K1XX_EDMA_TCD8_DADDR               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD8_DADDR_OFFSET)
#define S32K1XX_EDMA_TCD8_DOFF                (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD8_DOFF_OFFSET)
#define S32K1XX_EDMA_TCD8_CITER_ELINK         (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD8_CITER_ELINK_OFFSET)
#define S32K1XX_EDMA_TCD8_DLASTSGA            (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD8_DLASTSGA_OFFSET)
#define S32K1XX_EDMA_TCD8_CSR                 (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD8_CSR_OFFSET)
#define S32K1XX_EDMA_TCD8_BITER_ELINK         (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD8_BITER_ELINK_OFFSET)

#define S32K1XX_EDMA_TCD9_SADDR               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD9_SADDR_OFFSET)
#define S32K1XX_EDMA_TCD9_SOFF                (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD9_SOFF_OFFSET)
#define S32K1XX_EDMA_TCD9_ATTR                (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD9_ATTR_OFFSET)
#define S32K1XX_EDMA_TCD9_NBYTES_ML           (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD9_NBYTES_ML_OFFSET)
#define S32K1XX_EDMA_TCD9_SLAST               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD9_SLAST_OFFSET)
#define S32K1XX_EDMA_TCD9_DADDR               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD9_DADDR_OFFSET)
#define S32K1XX_EDMA_TCD9_DOFF                (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD9_DOFF_OFFSET)
#define S32K1XX_EDMA_TCD9_CITER_ELINK         (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD9_CITER_ELINK_OFFSET)
#define S32K1XX_EDMA_TCD9_DLASTSGA            (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD9_DLASTSGA_OFFSET)
#define S32K1XX_EDMA_TCD9_CSR                 (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD9_CSR_OFFSET)
#define S32K1XX_EDMA_TCD9_BITER_ELINK         (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD9_BITER_ELINK_OFFSET)

#define S32K1XX_EDMA_TCD10_SADDR              (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD10_SADDR_OFFSET)
#define S32K1XX_EDMA_TCD10_SOFF               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD10_SOFF_OFFSET)
#define S32K1XX_EDMA_TCD10_ATTR               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD10_ATTR_OFFSET)
#define S32K1XX_EDMA_TCD10_NBYTES_ML          (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD10_NBYTES_ML_OFFSET)
#define S32K1XX_EDMA_TCD10_SLAST              (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD10_SLAST_OFFSET)
#define S32K1XX_EDMA_TCD10_DADDR              (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD10_DADDR_OFFSET)
#define S32K1XX_EDMA_TCD10_DOFF               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD10_DOFF_OFFSET)
#define S32K1XX_EDMA_TCD10_CITER_ELINK        (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD10_CITER_ELINK_OFFSET)
#define S32K1XX_EDMA_TCD10_DLASTSGA           (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD10_DLASTSGA_OFFSET)
#define S32K1XX_EDMA_TCD10_CSR                (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD10_CSR_OFFSET)
#define S32K1XX_EDMA_TCD10_BITER_ELINK        (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD10_BITER_ELINK_OFFSET)

#define S32K1XX_EDMA_TCD11_SADDR              (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD11_SADDR_OFFSET)
#define S32K1XX_EDMA_TCD11_SOFF               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD11_SOFF_OFFSET)
#define S32K1XX_EDMA_TCD11_ATTR               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD11_ATTR_OFFSET)
#define S32K1XX_EDMA_TCD11_NBYTES_ML          (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD11_NBYTES_ML_OFFSET)
#define S32K1XX_EDMA_TCD11_SLAST              (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD11_SLAST_OFFSET)
#define S32K1XX_EDMA_TCD11_DADDR              (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD11_DADDR_OFFSET)
#define S32K1XX_EDMA_TCD11_DOFF               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD11_DOFF_OFFSET)
#define S32K1XX_EDMA_TCD11_CITER_ELINK        (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD11_CITER_ELINK_OFFSET)
#define S32K1XX_EDMA_TCD11_DLASTSGA           (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD11_DLASTSGA_OFFSET)
#define S32K1XX_EDMA_TCD11_CSR                (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD11_CSR_OFFSET)
#define S32K1XX_EDMA_TCD11_BITER_ELINK        (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD11_BITER_ELINK_OFFSET)

#define S32K1XX_EDMA_TCD12_SADDR              (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD12_SADDR_OFFSET)
#define S32K1XX_EDMA_TCD12_SOFF               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD12_SOFF_OFFSET)
#define S32K1XX_EDMA_TCD12_ATTR               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD12_ATTR_OFFSET)
#define S32K1XX_EDMA_TCD12_NBYTES_ML          (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD12_NBYTES_ML_OFFSET)
#define S32K1XX_EDMA_TCD12_SLAST              (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD12_SLAST_OFFSET)
#define S32K1XX_EDMA_TCD12_DADDR              (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD12_DADDR_OFFSET)
#define S32K1XX_EDMA_TCD12_DOFF               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD12_DOFF_OFFSET)
#define S32K1XX_EDMA_TCD12_CITER_ELINK        (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD12_CITER_ELINK_OFFSET)
#define S32K1XX_EDMA_TCD12_DLASTSGA           (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD12_DLASTSGA_OFFSET)
#define S32K1XX_EDMA_TCD12_CSR                (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD12_CSR_OFFSET)
#define S32K1XX_EDMA_TCD12_BITER_ELINK        (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD12_BITER_ELINK_OFFSET)

#define S32K1XX_EDMA_TCD13_SADDR              (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD13_SADDR_OFFSET)
#define S32K1XX_EDMA_TCD13_SOFF               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD13_SOFF_OFFSET)
#define S32K1XX_EDMA_TCD13_ATTR               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD13_ATTR_OFFSET)
#define S32K1XX_EDMA_TCD13_NBYTES_ML          (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD13_NBYTES_ML_OFFSET)
#define S32K1XX_EDMA_TCD13_SLAST              (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD13_SLAST_OFFSET)
#define S32K1XX_EDMA_TCD13_DADDR              (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD13_DADDR_OFFSET)
#define S32K1XX_EDMA_TCD13_DOFF               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD13_DOFF_OFFSET)
#define S32K1XX_EDMA_TCD13_CITER_ELINK        (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD13_CITER_ELINK_OFFSET)
#define S32K1XX_EDMA_TCD13_DLASTSGA           (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD13_DLASTSGA_OFFSET)
#define S32K1XX_EDMA_TCD13_CSR                (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD13_CSR_OFFSET)
#define S32K1XX_EDMA_TCD13_BITER_ELINK        (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD13_BITER_ELINK_OFFSET)

#define S32K1XX_EDMA_TCD14_SADDR              (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD14_SADDR_OFFSET)
#define S32K1XX_EDMA_TCD14_SOFF               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD14_SOFF_OFFSET)
#define S32K1XX_EDMA_TCD14_ATTR               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD14_ATTR_OFFSET)
#define S32K1XX_EDMA_TCD14_NBYTES_ML          (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD14_NBYTES_ML_OFFSET)
#define S32K1XX_EDMA_TCD14_SLAST              (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD14_SLAST_OFFSET)
#define S32K1XX_EDMA_TCD14_DADDR              (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD14_DADDR_OFFSET)
#define S32K1XX_EDMA_TCD14_DOFF               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD14_DOFF_OFFSET)
#define S32K1XX_EDMA_TCD14_CITER_ELINK        (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD14_CITER_ELINK_OFFSET)
#define S32K1XX_EDMA_TCD14_DLASTSGA           (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD14_DLASTSGA_OFFSET)
#define S32K1XX_EDMA_TCD14_CSR                (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD14_CSR_OFFSET)
#define S32K1XX_EDMA_TCD14_BITER_ELINK        (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD14_BITER_ELINK_OFFSET)

#define S32K1XX_EDMA_TCD15_SADDR              (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD15_SADDR_OFFSET)
#define S32K1XX_EDMA_TCD15_SOFF               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD15_SOFF_OFFSET)
#define S32K1XX_EDMA_TCD15_ATTR               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD15_ATTR_OFFSET)
#define S32K1XX_EDMA_TCD15_NBYTES_ML          (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD15_NBYTES_ML_OFFSET)
#define S32K1XX_EDMA_TCD15_SLAST              (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD15_SLAST_OFFSET)
#define S32K1XX_EDMA_TCD15_DADDR              (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD15_DADDR_OFFSET)
#define S32K1XX_EDMA_TCD15_DOFF               (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD15_DOFF_OFFSET)
#define S32K1XX_EDMA_TCD15_CITER_ELINK        (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD15_CITER_ELINK_OFFSET)
#define S32K1XX_EDMA_TCD15_DLASTSGA           (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD15_DLASTSGA_OFFSET)
#define S32K1XX_EDMA_TCD15_CSR                (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD15_CSR_OFFSET)
#define S32K1XX_EDMA_TCD15_BITER_ELINK        (S32K1XX_EDMA_BASE + S32K1XX_EDMA_TCD15_BITER_ELINK_OFFSET)

/* eDMA Bit-Field Definitions ***********************************************/

/* Control */

                                                      /* Bit 0:  Reserved */
#define EDMA_CR_EDBG                        (1 << 1)  /* Bit 1:  Enable Debug */
#define EDMA_CR_ERCA                        (1 << 2)  /* Bit 2:  Enable Round Robin Channel Arbitration */
                                                      /* Bit 3:  Reserved */
#define EDMA_CR_HOE                         (1 << 4)  /* Bit 4:  Halt On Error */
#define EDMA_CR_HALT                        (1 << 5)  /* Bit 5:  Halt DMA Operations */
#define EDMA_CR_CLM                         (1 << 6)  /* Bit 6:  Continuous Link Mode */
#define EDMA_CR_EMLM                        (1 << 7)  /* Bit 7:  Enable Minor Loop Mapping */
                                                      /* Bit 8-15:  Reserved */
#define EDMA_CR_ECX                         (1 << 16) /* Bit 16: Error Cancel Transfer */
#define EDMA_CR_CX                          (1 << 17) /* Bit 17: Cancel Transfer */
                                                      /* Bits 18-23: Reserved */
                                                      /* Bits 24-30: eDMA version number (reserved) */
#define EDMA_CR_ACTIVE                      (1 << 31) /* Bit 31: DMA Active Status */

/* Error Status */

#define EDMA_ES_DBE                         (1 << 0)  /* Bit 0:  Destination Bus Error */
#define EDMA_ES_SBE                         (1 << 1)  /* Bit 1:  Source Bus Error */
#define EDMA_ES_SGE                         (1 << 2)  /* Bit 2:  Scatter/Gather Configuration Error */
#define EDMA_ES_NCE                         (1 << 3)  /* Bit 3:  NBYTES/CITER Configuration Error */
#define EDMA_ES_DOE                         (1 << 4)  /* Bit 4:  Destination Offset Error */
#define EDMA_ES_DAE                         (1 << 5)  /* Bit 5:  Destination Address Error */
#define EDMA_ES_SOE                         (1 << 6)  /* Bit 6:  Source Offset Error */
#define EDMA_ES_SAE                         (1 << 7)  /* Bit 7:  Source Address Error */
#define EDMA_ES_ERRCHN_SHIFT                (8)       /* Bits 8-11: Error Channel Number or
                                                       *         Canceled Channel Number */
#define EDMA_ES_ERRCHN_MASK                 (15 << EDMA_ES_ERRCHN_SHIFT)
                                                      /* Bits 23-13: Reserved */
#define EDMA_ES_CPE                         (1 << 14) /* Bit 14: Channel Priority Error */
                                                      /* Bit 15:  Reserved */
#define EDMA_ES_ECX                         (1 << 16) /* Bit 16: Transfer Canceled */
                                                      /* Bits 17-30: Reserved */
#define EDMA_ES_VLD                         (1 << 31) /* Bit 31: Logical OR of all ERR status bits */

/* Enable Request */

#define EDMA_ERQ(n)                         ((uint32_t)1 << (n)) /* Bit n:  Enable DMA request n */

/* Enable Error Interrupt */

#define EDMA_EEI(n)                         ((uint32_t)1 << (n)) /* Bit n:  Enable error interrupt n */

/* Clear Enable Error Interrupt */

#define EDMA_CEEI_SHIFT                     (0)       /* Bits 0-3: Clear Enable Error Interrupt */
#define EDMA_CEEI_MASK                      (15 << EDMA_CEEI_SHIFT)
#  define EDMA_CEEI(n)                      ((uint32_t)(n) << EDMA_CEEI_SHIFT)
                                                      /* Bit 54-:  Reserved */
#define EDMA_CEEI_CAEE                      (1 << 6)  /* Bit 6:  Clear All Enable Error Interrupts */
#define EDMA_CEEI_NOP                       (1 << 7)  /* Bit 7:  No Op enable */

/* Set Enable Error Interrupt */

#define EDMA_SEEI_SHIFT                     (0)       /* Bits 0-3: Set Enable Error Interrupt */
#define EDMA_SEEI_MASK                      (15 << EDMA_SEEI_SHIFT)
#  define EDMA_SEEI(n)                      ((uint32_t)(n) << EDMA_SEEI_SHIFT)
                                                      /* Bit 54-:  Reserved */
#define EDMA_SEEI_SAEE                      (1 << 6)  /* Bit 6:  Set All Enable Error Interrupts */
#define EDMA_SEEI_NOP                       (1 << 7)  /* Bit 7:  No Op enable */

/* Clear Enable Request */

#define EDMA_CERQ_SHIFT                     (0)       /* Bits 0-3: Clear Enable Request */
#define EDMA_CERQ_MASK                      (15 << EDMA_CERQ_SHIFT)
#  define EDMA_CERQ(n)                      ((uint32_t)(n) << EDMA_CERQ_SHIFT)
                                                      /* Bit 4-5:  Reserved */
#define EDMA_CERQ_CAER                      (1 << 6)  /* Bit 6:  Clear All Enable Requests */
#define EDMA_CERQ_NOP                       (1 << 7)  /* Bit 7:  No Op enable */

/* Set Enable Request */

#define EDMA_SERQ_SHIFT                     (0)       /* Bits 0-3: Set Enable Request */
#define EDMA_SERQ_MASK                      (15 << EDMA_SERQ_SHIFT)
#  define EDMA_SERQ(n)                      ((uint32_t)(n) << EDMA_SERQ_SHIFT)
                                                      /* Bit 4-5:  Reserved */
#define EDMA_SERQ_SAER                      (1 << 6)  /* Bit 6:  Set All Enable Requests */
#define EDMA_SERQ_NOP                       (1 << 7)  /* Bit 7:  No Op enable */

/* Clear DONE Status Bit */

#define EDMA_CDNE_SHIFT                     (0)       /* Bits 0-3: Clear DONE Bit */
#define EDMA_CDNE_MASK                      (15 << EDMA_CDNE_SHIFT)
#  define EDMA_CDNE(n)                      ((uint32_t)(n) << EDMA_CDNE_SHIFT)
                                                      /* Bit 4-5:  Reserved */
#define EDMA_CDNE_CADN                      (1 << 6)  /* Bit 6:  Clears All DONE Bits */
#define EDMA_CDNE_NOP                       (1 << 7)  /* Bit 7:  No Op enable */

/* Set START Bit */

#define EDMA_SSRT_SHIFT                     (0)       /* Bits 0-3: Set START Bit */
#define EDMA_SSRT_MASK                      (15 << EDMA_SSRT_SHIFT)
#  define EDMA_SSRT(n)                      ((uint32_t)(n) << EDMA_SSRT_SHIFT)
                                                      /* Bit 4-5:  Reserved */
#define EDMA_SSRT_SAST                      (1 << 6)  /* Bit 6:  Set All START Bits (activates all channels) */
#define EDMA_SSRT_NOP                       (1 << 7)  /* Bit 7:  No Op enable */

/* Clear Error */

#define EDMA_CERR_SHIFT                     (0)       /* Bits 0-3: Clear Error Indicator */
#define EDMA_CERR_MASK                      (15 << EDMA_CERR_SHIFT)
#  define EDMA_CERR(n)                      ((uint32_t)(n) << EDMA_CERR_SHIFT)
                                                      /* Bit 4-5:  Reserved */
#define EDMA_CERR_CAEI                      (1 << 6)  /* Bit 6:  Clear All Error Indicators */
#define EDMA_CERR_NOP                       (1 << 7)  /* Bit 7:  No Op enable */

/* Clear Interrupt Request */

#define EDMA_CINT_SHIFT                     (0)       /* Bits 0-3: Clear Interrupt Request */
#define EDMA_CINT_MASK                      (15 << EDMA_CINT_SHIFT)
#  define EDMA_CINT(n)                      ((uint32_t)(n) << EDMA_CINT_SHIFT)
                                                      /* Bit 4-5:  Reserved */
#define EDMA_CINT_CAIR                      (1 << 6)  /* Bit 6:  Clear All Interrupt Requests */
#define EDMA_CINT_NOP                       (1 << 7)  /* Bit 7:  No Op enable */

/* Interrupt Request */

#define EDMA_INT(n)                         ((uint32_t)1 << (n)) /* Bit n:  Interrupt Request n */

/* Error */

#define EDMA_ERR(n)                         ((uint32_t)1 << (n)) /* Bit n:  Error In Channel n */

/* Hardware Request Status */

#define EDMA_HRS(n)                         ((uint32_t)1 << (n)) /* Bit n:  Hardware Request Status
                                                                  * Channel n */

/* Enable Asynchronous Request in Stop */

#define EDMA_EARS(n)                        ((uint32_t)1 << (n)) /* Bit n:  Enable asynchronous DMA
                                                                  * request in stop mode for channel n */

/* Channel n Priority */

#define EDMA_DCHPRI_CHPRI_SHIFT             (0)       /* Bits 0-3: Channel n Arbitration Priority */
#define EDMA_DCHPRI_CHPRI_MASK              (15 << EDMA_DCHPRI_CHPRI_SHIFT)
#  define EDMA_DCHPRI_CHPRI(n)              ((uint32_t)(n) << EDMA_DCHPRI_CHPRI_SHIFT)
                                                      /* Bit 4-5:  Reserved */
#define EDMA_DCHPRI_DPA                     (1 << 6)  /* Bit 6:  Disable Preempt Ability */
#define EDMA_DCHPRI_ECP                     (1 << 7)  /* Bit 7:  Enable Channel Preemption */

/* TCD Source Address (32-bit address) */

/* TCD Signed Source Address Offset (16-bit offset) */

/* TCD Transfer Attributes */

#define TCD_ATTR_SIZE_8BIT                  (0)       /* 8-bit */
#define TCD_ATTR_SIZE_16BIT                 (1)       /* 16-bit */
#define TCD_ATTR_SIZE_32BIT                 (2)       /* 32-bit */
#define TCD_ATTR_SIZE_64BIT                 (3)       /* 64-bit */
#define TCD_ATTR_SIZE_256BIT                (5)       /* 32-byte burst (4 beats of 64 bits) */

#define EDMA_TCD_ATTR_DSIZE_SHIFT           (0)       /* Bits 0-2: Destination data transfer size */
#define EDMA_TCD_ATTR_DSIZE_MASK            (7 << EDMA_TCD_ATTR_DSIZE_SHIFT)
#  define EDMA_TCD_ATTR_DSIZE(n)            ((uint32_t)(n)        << EDMA_TCD_ATTR_DSIZE_SHIFT) /* 8-bit */
#  define EDMA_TCD_ATTR_DSIZE_8BIT          (TCD_ATTR_SIZE_8BIT   << EDMA_TCD_ATTR_DSIZE_SHIFT) /* 8-bit */
#  define EDMA_TCD_ATTR_DSIZE_16BIT         (TCD_ATTR_SIZE_16BIT  << EDMA_TCD_ATTR_DSIZE_SHIFT) /* 16-bit */
#  define EDMA_TCD_ATTR_DSIZE_32BIT         (TCD_ATTR_SIZE_32BIT  << EDMA_TCD_ATTR_DSIZE_SHIFT) /* 32-bit */
#  define EDMA_TCD_ATTR_DSIZE_64BIT         (TCD_ATTR_SIZE_64BIT  << EDMA_TCD_ATTR_DSIZE_SHIFT) /* 64-bit */
#  define EDMA_TCD_ATTR_DSIZE_256BIT        (TCD_ATTR_SIZE_256BIT << EDMA_TCD_ATTR_DSIZE_SHIFT) /* 32-byte burst */

#define EDMA_TCD_ATTR_DMOD_SHIFT            (3)       /* Bits 3-7: Destination Address Modulo */
#define EDMA_TCD_ATTR_DMOD_MASK             (31 << EDMA_TCD_ATTR_DMOD_SHIFT)
#  define EDMA_TCD_ATTR_DMOD(n)             ((uint32_t)(n) << EDMA_TCD_ATTR_DMOD_SHIFT)
#define EDMA_TCD_ATTR_SSIZE_SHIFT           (8)       /* Bits 8-10: Source data transfer size */
#define EDMA_TCD_ATTR_SSIZE_MASK            (7 << EDMA_TCD_ATTR_SSIZE_SHIFT)
#  define EDMA_TCD_ATTR_SSIZE(n)            ((uint32_t)(n)        << EDMA_TCD_ATTR_SSIZE_SHIFT) /* 8-bit */
#  define EDMA_TCD_ATTR_SSIZE_8BIT          (TCD_ATTR_SIZE_8BIT   << EDMA_TCD_ATTR_SSIZE_SHIFT) /* 8-bit */
#  define EDMA_TCD_ATTR_SSIZE_16BIT         (TCD_ATTR_SIZE_16BIT  << EDMA_TCD_ATTR_SSIZE_SHIFT) /* 16-bit */
#  define EDMA_TCD_ATTR_SSIZE_32BIT         (TCD_ATTR_SIZE_32BIT  << EDMA_TCD_ATTR_SSIZE_SHIFT) /* 32-bit */
#  define EDMA_TCD_ATTR_SSIZE_64BIT         (TCD_ATTR_SIZE_64BIT  << EDMA_TCD_ATTR_SSIZE_SHIFT) /* 64-bit */
#  define EDMA_TCD_ATTR_SSIZE_256BIT        (TCD_ATTR_SIZE_256BIT << EDMA_TCD_ATTR_SSIZE_SHIFT) /* 32-byte burst */

#define EDMA_TCD_ATTR_SMOD_SHIFT            (11)      /* Bits 11-15: Source Address Modulo */
#define EDMA_TCD_ATTR_SMOD_MASK             (31 << EDMA_TCD_ATTR_SMOD_SHIFT)
#  define EDMA_TCD_ATTR_SMOD(n)             ((uint32_t)(n) << EDMA_TCD_ATTR_SMOD_SHIFT)

/* TCD Signed Minor Loop Offset / Byte Count */

/* Minor Byte Count (Minor Loop Mapping Disabled -- 32-bit byte count) */

/* TCD Signed Minor Loop Offset / Byte Count */

/* Minor Byte Count (Minor Loop Mapping Enabled, offset disabled) */

#define EDMA_TCD_NBYTES_ML_NBYTES_SHIFT     (0)       /* Bits 0-29: Minor Byte Transfer Count */
#define EDMA_TCD_NBYTES_ML_NBYTES_MASK      (0x3fffffff << EDMA_TCD_NBYTES_ML_NBYTES_SHIFT)
#  define EDMA_TCD_NBYTES_ML_NBYTES(n)      ((uint32_t)(n) << EDMA_TCD_NBYTES_ML_NBYTES_SHIFT)
#define EDMA_TCD_NBYTES_ML_DMLOE            (1 << 30) /* Bit 30: Destination Minor Loop Offset enable */
#define EDMA_TCD_NBYTES_ML_SMLOE            (1 << 31) /* Bit 31: Source Minor Loop Offset Enable */

/* TCD Signed Minor Loop Offset / Byte Count */

/* Minor Byte Count (Minor Loop Mapping Enabled, offset enabled) */

#define EDMA_TCD_NBYTES_MLOFF_NBYTES_SHIFT  (0)      /* Bits 0-9: Minor Byte Transfer Count */
#define EDMA_TCD_NBYTES_MLOFF_NBYTES_MASK   (0x3ff << EDMA_TCD_NBYTES_MLOFF_NBYTES_SHIFT)
#  define EDMA_TCD_NBYTES_MLOFF_NBYTES(n)   ((uint32_t)(n) << EDMA_TCD_NBYTES_MLOFF_NBYTES_SHIFT)
#define EDMA_TCD_NBYTES_MLOFF_MLOFF_SHIFT   (10)     /* Bits 10-29: Minor Byte Transfer Count */
#define EDMA_TCD_NBYTES_MLOFF_MLOFF_MASK    (0xfffff << EDMA_TCD_NBYTES_MLOFF_MLOFF_SHIFT)
#  define EDMA_TCD_NBYTES_MLOFF_MLOFF(n)    ((uint32_t)(n) << EDMA_TCD_NBYTES_MLOFF_MLOFF_SHIFT)
#define EDMA_TCD_NBYTES_MLOFF_DMLOE         (1 << 30) /* Bit 30: Destination Minor Loop Offset enable */
#define EDMA_TCD_NBYTES_MLOFF_SMLOE         (1 << 31) /* Bit 31: Source Minor Loop Offset Enable */

/* TCD Last Source Address Adjustment (32-bit address adjustment) */

/* TCD Destination Address (32-bit address) */

/* TCD Signed Destination Address Offset (16-bit signed address offset) */

/* TCD Current Minor Loop Link, Major Loop Count (Channel linking disabled) */

#define EDMA_TCD_CITER_CITER_SHIFT          (0)       /* Bit 0-14: Starting Major Iteration Count */
#define EDMA_TCD_CITER_CITER_MASK           (0x7fff << EDMA_TCD_CITER_CITER_SHIFT)
#  define EDMA_TCD_CITER_CITER(n)           ((uint32_t)(n) << EDMA_TCD_CITER_CITER_SHIFT)
#define EDMA_TCD_CITER_ELINK                (1 << 15) /* Bit 15: Enable channel-to-channel linking
                                                       * on minor-loop complete */

/* TCD Current Minor Loop Link, Major Loop Count
 * (Channel linking enabled)
 */

#define EDMA_TCD_CITER_ELINK_CITER_SHIFT    (0)       /* Bit 0-8: Current major iteration count */
#define EDMA_TCD_CITER_ELINK_CITER_MASK     (0x1ff << EDMA_TCD_CITER_ELINK_CITER_SHIFT)
#  define EDMA_TCD_CITER_ELINK_CITER(n)     ((uint32_t)(n) << EDMA_TCD_CITER_ELINK_CITER_SHIFT)
#define EDMA_TCD_CITER_ELINK_LINKCH_SHIFT   (9)       /* Bit 9-12: Minor Loop Link Channel Number */
#define EDMA_TCD_CITER_ELINK_LINKCH_MASK    (15 << EDMA_TCD_CITER_ELINK_LINKCH_SHIFT)
#  define EDMA_TCD_CITER_ELINK_LINKCH(n)    ((uint32_t)(n) << EDMA_TCD_CITER_ELINK_LINKCH_SHIFT)
                                                      /* Bit 13-14: Reserved */
#define EDMA_TCD_CITER_ELINK_ELINK          (1 << 15) /* Bit 15: Enable channel-to-channel linking
                                                       * on minor-loop complete */

/* TCD Last Destination Address Adjustment/Scatter Gather Address
 * (32-bit address)
 */

/* TCD Control and Status */

#define EDMA_TCD_CSR_START                  (1 << 0)  /* Bit 0:  Channel Start */
#define EDMA_TCD_CSR_INTMAJOR               (1 << 1)  /* Bit 1:  Enable an interrupt when major
                                                       *         iteration count completes */
#define EDMA_TCD_CSR_INTHALF                (1 << 2)  /* Bit 2:  Enable an interrupt when major
                                                       *         counter is half complete */
#define EDMA_TCD_CSR_DREQ                   (1 << 3)  /* Bit 3:  Disable Request */
#define EDMA_TCD_CSR_ESG                    (1 << 4)  /* Bit 4:  Enable Scatter/Gather Processing */
#define EDMA_TCD_CSR_MAJORELINK             (1 << 5)  /* Bit 5:  Enable channel-to-channel linking
                                                       *         on major loop complete */
#define EDMA_TCD_CSR_ACTIVE                 (1 << 6)  /* Bit 6:  Channel Active */
#define EDMA_TCD_CSR_DONE                   (1 << 7)  /* Bit 7:  Channel Done */
#define EDMA_TCD_CSR_MAJORLINKCH_SHIFT      (8)       /* Bits 8-11: Major Loop Link Channel Number */
#define EDMA_TCD_CSR_MAJORLINKCH_MASK       (15 << EDMA_TCD_CSR_MAJORLINKCH_SHIFT)
#  define EDMA_TCD_CSR_MAJORLINKCH(n)       ((uint32_t)(n) << EDMA_TCD_CSR_MAJORLINKCH_SHIFT)
                                                      /* Bit 112-3: Reserved */
#define EDMA_TCD_CSR_BWC_SHIFT              (14)      /* Bits 14-15: Bandwidth Control */
#define EDMA_TCD_CSR_BWC_MASK               (3 << EDMA_TCD_CSR_BWC_SHIFT)
#  define EDMA_TCD_CSR_BWC_NONE             (0 << EDMA_TCD_CSR_BWC_SHIFT) /* No eDMA engine stalls */
#  define EDMA_TCD_CSR_BWC_4CYCLES          (2 << EDMA_TCD_CSR_BWC_SHIFT) /* eDMA engine stalls for 4
                                                                           * cycles after each R/W */
#  define EDMA_TCD_CSR_BWC_8CYCLES          (3 << EDMA_TCD_CSR_BWC_SHIFT) /* eDMA engine stalls for 8
                                                                           * cycles after each R/W */

/* TCD Beginning Minor Loop Link, Major Loop Count
 * (Channel linking disabled)
 */

#define EDMA_TCD_BITER_BITER_SHIFT          (0)       /* Bit 0-14: Starting Major Iteration Count */
#define EDMA_TCD_BITER_BITER_MASK           (0x7fff << EDMA_TCD_BITER_BITER_SHIFT)
#  define EDMA_TCD_BITER_BITER(n)           ((uint32_t)(n) << EDMA_TCD_BITER_BITER_SHIFT)
#define EDMA_TCD_BITER_ELINK                (1 << 15) /* Bit 15: Enable channel-to-channel linking
                                                       * on minor-loop complete */

/* TCD Beginning Minor Loop Link, Major Loop Count
 * (Channel linking enabled)
 */

#define EDMA_TCD_BITER_ELINK_BITER_SHIFT    (0)       /* Bit 0-8: Current major iteration count */
#define EDMA_TCD_BITER_ELINK_BITER_MASK     (0x1ff << EDMA_TCD_BITER_ELINK_BITER_SHIFT)
#  define EDMA_TCD_BITER_ELINK_BITER(n)     ((uint32_t)(n) << EDMA_TCD_BITER_ELINK_BITER_SHIFT)
#define EDMA_TCD_BITER_ELINK_LINKCH_SHIFT   (9)       /* Bit 9-12: Link Channel Number */
#define EDMA_TCD_BITER_ELINK_LINKCH_MASK    (15 << EDMA_TCD_BITER_ELINK_LINKCH_SHIFT)
#  define EDMA_TCD_BITER_ELINK_LINKCH(n)    ((uint32_t)(n) << EDMA_TCD_BITER_ELINK_LINKCH_SHIFT)
                                                      /* Bit 13-4: Reserved */
#define EDMA_TCD_BITER_ELINK_ELINK          (1 << 15) /* Bit 15: Enable channel-to-channel linking
                                                       * on minor-loop complete */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* In-memory representation of the 32-byte Transfer Control Descriptor
 * (TCD)
 */

struct s32k1xx_edmatcd_s
{
  uint32_t saddr;         /* Offset: 0x0000  TCD Source Address */
  uint16_t soff;          /* Offset: 0x0004  TCD Signed Source Address Offset */
  uint16_t attr;          /* Offset: 0x0006  TCD Transfer Attributes */
  uint32_t nbytes;        /* Offset: 0x0008  TCD Signed Minor Loop Offset / Byte Count */
  uint32_t slast;         /* Offset: 0x000c  TCD Last Source Address Adjustment */
  uint32_t daddr;         /* Offset: 0x0010  TCD Destination Address */
  uint16_t doff;          /* Offset: 0x0014  TCD Signed Destination Address Offset */
  uint16_t citer;         /* Offset: 0x0016  TCD Current Minor Loop Link, Major Loop Count */
  uint32_t dlastsga;      /* Offset: 0x0018  TCD Last Destination Address Adjustment/Scatter Gather Address */
  uint16_t csr;           /* Offset: 0x001c  TCD Control and Status */
  uint16_t biter;         /* Offset: 0x001e  TCD Beginning Minor Loop Link, Major Loop Count */
};

#endif /* __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_EDMA_H */
