/****************************************************************************************************
 * arch/arm/src/imxrt/chip/imxrt_edma.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_IMXRT_CHIP_IMXRT_EDMA_H
#define __ARCH_ARM_SRC_IMXRT_CHIP_IMXRT_EDMA_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include "chip/imxrt_memorymap.h"

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

#define IMXRT_EDMA_NCHANNELS                32

/* eDMA Register Offsets ****************************************************************************/

#define IMXRT_EDMA_CR_OFFSET                0x0000  /* Control */
#define IMXRT_EDMA_ES_OFFSET                0x0004  /* Error Status */
#define IMXRT_EDMA_ERQ_OFFSET               0x000c  /* Enable Request */
#define IMXRT_EDMA_EEI_OFFSET               0x0014  /* Enable Error Interrupt */
#define IMXRT_EDMA_CEEI_OFFSET              0x0018  /* Clear Enable Error Interrupt */
#define IMXRT_EDMA_SEEI_OFFSET              0x0019  /* Set Enable Error Interrupt */
#define IMXRT_EDMA_CERQ_OFFSET              0x001a  /* Clear Enable Request */
#define IMXRT_EDMA_SERQ_OFFSET              0x001b  /* Set Enable Request */
#define IMXRT_EDMA_CDNE_OFFSET              0x001c  /* Clear DONE Status Bit */
#define IMXRT_EDMA_SSRT_OFFSET              0x001d  /* Set START Bit */
#define IMXRT_EDMA_CERR_OFFSET              0x001e  /* Clear Error */
#define IMXRT_EDMA_CINT_OFFSET              0x001f  /* Clear Interrupt Request */
#define IMXRT_EDMA_INT_OFFSET               0x0024  /* Interrupt Request */
#define IMXRT_EDMA_ERR_OFFSET               0x002c  /* Error */
#define IMXRT_EDMA_HRS_OFFSET               0x0034  /* Hardware Request Status */
#define IMXRT_EDMA_EARS_OFFSET              0x0044  /* Enable Asynchronous Request in Stop */

#define IMXRT_EDMA_DCHPRI_OFFSET(n)         (0x0100 + ((n) & ~3) + (3 - ((n) & 3)))

#define IMXRT_EDMA_DCHPRI3_OFFSET           0x0100  /* Channel 3 Priority */
#define IMXRT_EDMA_DCHPRI2_OFFSET           0x0101  /* Channel 2 Priority */
#define IMXRT_EDMA_DCHPRI1_OFFSET           0x0102  /* Channel 1 Priority */
#define IMXRT_EDMA_DCHPRI0_OFFSET           0x0103  /* Channel 0 Priority */
#define IMXRT_EDMA_DCHPRI7_OFFSET           0x0104  /* Channel 7 Priority */
#define IMXRT_EDMA_DCHPRI6_OFFSET           0x0105  /* Channel 6 Priority */
#define IMXRT_EDMA_DCHPRI5_OFFSET           0x0106  /* Channel 5 Priority */
#define IMXRT_EDMA_DCHPRI4_OFFSET           0x0107  /* Channel 4 Priority */
#define IMXRT_EDMA_DCHPRI11_OFFSET          0x0108  /* Channel 11 Priority */
#define IMXRT_EDMA_DCHPRI10_OFFSET          0x0109  /* Channel 10 Priority */
#define IMXRT_EDMA_DCHPRI9_OFFSET           0x010a  /* Channel 9 Priority */
#define IMXRT_EDMA_DCHPRI8_OFFSET           0x010b  /* Channel 8 Priority */
#define IMXRT_EDMA_DCHPRI15_OFFSET          0x010c  /* Channel 15 Priority */
#define IMXRT_EDMA_DCHPRI14_OFFSET          0x010d  /* Channel 14 Priority */
#define IMXRT_EDMA_DCHPRI13_OFFSET          0x010e  /* Channel 13 Priority */
#define IMXRT_EDMA_DCHPRI12_OFFSET          0x010f  /* Channel 12 Priority */
#define IMXRT_EDMA_DCHPRI19_OFFSET          0x0110  /* Channel 19 Priority */
#define IMXRT_EDMA_DCHPRI18_OFFSET          0x0111  /* Channel 18 Priority */
#define IMXRT_EDMA_DCHPRI17_OFFSET          0x0112  /* Channel 17 Priority */
#define IMXRT_EDMA_DCHPRI16_OFFSET          0x0113  /* Channel 16 Priority */
#define IMXRT_EDMA_DCHPRI23_OFFSET          0x0114  /* Channel 23 Priority */
#define IMXRT_EDMA_DCHPRI22_OFFSET          0x0115  /* Channel 22 Priority */
#define IMXRT_EDMA_DCHPRI21_OFFSET          0x0116  /* Channel 21 Priority */
#define IMXRT_EDMA_DCHPRI20_OFFSET          0x0117  /* Channel 20 Priority */
#define IMXRT_EDMA_DCHPRI27_OFFSET          0x0118  /* Channel 27 Priority */
#define IMXRT_EDMA_DCHPRI26_OFFSET          0x0119  /* Channel 26 Priority */
#define IMXRT_EDMA_DCHPRI25_OFFSET          0x011a  /* Channel 25 Priority */
#define IMXRT_EDMA_DCHPRI24_OFFSET          0x011b  /* Channel 24 Priority */
#define IMXRT_EDMA_DCHPRI31_OFFSET          0x011c  /* Channel 31 Priority */
#define IMXRT_EDMA_DCHPRI30_OFFSET          0x011d  /* Channel 30 Priority */
#define IMXRT_EDMA_DCHPRI29_OFFSET          0x011e  /* Channel 29 Priority */
#define IMXRT_EDMA_DCHPRI28_OFFSET          0x011f  /* Channel 28 Priority */

/* Transfer Control Descriptor (TCD) */

#define IMXRT_EDMA_TCD_OFFSET(n)            (0x1000 + ((n) << 5))
#define IMXRT_EDMA_TCD_SADDR_OFFSET         0x0000  /* TCD Source Address */
#define IMXRT_EDMA_TCD_SOFF_OFFSET          0x0004  /* TCD Signed Source Address Offset */
#define IMXRT_EDMA_TCD_ATTR_OFFSET          0x0006  /* TCD Transfer Attributes */
#define IMXRT_EDMA_TCD_NBYTES_ML_OFFSET     0x0008  /* TCD Signed Minor Loop Offset / Byte Count */
#define IMXRT_EDMA_TCD_SLAST_OFFSET         0x000c  /* TCD Last Source Address Adjustment */
#define IMXRT_EDMA_TCD_DADDR_OFFSET         0x0010  /* TCD Destination Address */
#define IMXRT_EDMA_TCD_DOFF_OFFSET          0x0014  /* TCD Signed Destination Address Offset */
#define IMXRT_EDMA_TCD_CITER_ELINK_OFFSET   0x0016  /* TCD Current Minor Loop Link, Major Loop Count */
#define IMXRT_EDMA_TCD_DLASTSGA_OFFSET      0x0018  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define IMXRT_EDMA_TCD_CSR_OFFSET           0x001c  /* TCD Control and Status */
#define IMXRT_EDMA_TCD_BITER_ELINK_OFFSET   0x001e  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define IMXRT_EDMA_TCD0_SADDR_OFFSET        0x1000  /* TCD Source Address */
#define IMXRT_EDMA_TCD0_SOFF_OFFSET         0x1004  /* TCD Signed Source Address Offset */
#define IMXRT_EDMA_TCD0_ATTR_OFFSET         0x1006  /* TCD Transfer Attributes */
#define IMXRT_EDMA_TCD0_NBYTES_ML_OFFSET    0x1008  /* TCD Signed Minor Loop Offset / Byte Count */
#define IMXRT_EDMA_TCD0_SLAST_OFFSET        0x100c  /* TCD Last Source Address Adjustment */
#define IMXRT_EDMA_TCD0_DADDR_OFFSET        0x1010  /* TCD Destination Address */
#define IMXRT_EDMA_TCD0_DOFF_OFFSET         0x1014  /* TCD Signed Destination Address Offset */
#define IMXRT_EDMA_TCD0_CITER_ELINK_OFFSET  0x1016  /* TCD Current Minor Loop Link, Major Loop Count */
#define IMXRT_EDMA_TCD0_DLASTSGA_OFFSET     0x1018  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define IMXRT_EDMA_TCD0_CSR_OFFSET          0x101c  /* TCD Control and Status */
#define IMXRT_EDMA_TCD0_BITER_ELINK_OFFSET  0x101e  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define IMXRT_EDMA_TCD1_SADDR_OFFSET        0x1020  /* TCD Source Address */
#define IMXRT_EDMA_TCD1_SOFF_OFFSET         0x1024  /* TCD Signed Source Address Offset */
#define IMXRT_EDMA_TCD1_ATTR_OFFSET         0x1026  /* TCD Transfer Attributes */
#define IMXRT_EDMA_TCD1_NBYTES_ML_OFFSET    0x1028  /* TCD Signed Minor Loop Offset / Byte Count */
#define IMXRT_EDMA_TCD1_SLAST_OFFSET        0x102c  /* TCD Last Source Address Adjustment */
#define IMXRT_EDMA_TCD1_DADDR_OFFSET        0x1030  /* TCD Destination Address */
#define IMXRT_EDMA_TCD1_DOFF_OFFSET         0x1034  /* TCD Signed Destination Address Offset */
#define IMXRT_EDMA_TCD1_CITER_ELINK_OFFSET  0x1036  /* TCD Current Minor Loop Link, Major Loop Count */
#define IMXRT_EDMA_TCD1_DLASTSGA_OFFSET     0x1038  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define IMXRT_EDMA_TCD1_CSR_OFFSET          0x103c  /* TCD Control and Status */
#define IMXRT_EDMA_TCD1_BITER_ELINK_OFFSET  0x103e  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define IMXRT_EDMA_TCD1_SADDR_OFFSET        0x1020  /* TCD Source Address */
#define IMXRT_EDMA_TCD1_SOFF_OFFSET         0x1024  /* TCD Signed Source Address Offset */
#define IMXRT_EDMA_TCD1_ATTR_OFFSET         0x1026  /* TCD Transfer Attributes */
#define IMXRT_EDMA_TCD1_NBYTES_ML_OFFSET    0x1028  /* TCD Signed Minor Loop Offset / Byte Count */
#define IMXRT_EDMA_TCD1_SLAST_OFFSET        0x102c  /* TCD Last Source Address Adjustment */
#define IMXRT_EDMA_TCD1_DADDR_OFFSET        0x1030  /* TCD Destination Address */
#define IMXRT_EDMA_TCD1_DOFF_OFFSET         0x1034  /* TCD Signed Destination Address Offset */
#define IMXRT_EDMA_TCD1_CITER_ELINK_OFFSET  0x1036  /* TCD Current Minor Loop Link, Major Loop Count */
#define IMXRT_EDMA_TCD1_DLASTSGA_OFFSET     0x1038  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define IMXRT_EDMA_TCD1_CSR_OFFSET          0x103c  /* TCD Control and Status */
#define IMXRT_EDMA_TCD1_BITER_ELINK_OFFSET  0x103e  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define IMXRT_EDMA_TCD2_SADDR_OFFSET        0x1040  /* TCD Source Address */
#define IMXRT_EDMA_TCD2_SOFF_OFFSET         0x1044  /* TCD Signed Source Address Offset */
#define IMXRT_EDMA_TCD2_ATTR_OFFSET         0x1046  /* TCD Transfer Attributes */
#define IMXRT_EDMA_TCD2_NBYTES_ML_OFFSET    0x1048  /* TCD Signed Minor Loop Offset / Byte Count */
#define IMXRT_EDMA_TCD2_SLAST_OFFSET        0x104c  /* TCD Last Source Address Adjustment */
#define IMXRT_EDMA_TCD2_DADDR_OFFSET        0x1050  /* TCD Destination Address */
#define IMXRT_EDMA_TCD2_DOFF_OFFSET         0x1054  /* TCD Signed Destination Address Offset */
#define IMXRT_EDMA_TCD2_CITER_ELINK_OFFSET  0x1056  /* TCD Current Minor Loop Link, Major Loop Count */
#define IMXRT_EDMA_TCD2_DLASTSGA_OFFSET     0x1058  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define IMXRT_EDMA_TCD2_CSR_OFFSET          0x105c  /* TCD Control and Status */
#define IMXRT_EDMA_TCD2_BITER_ELINK_OFFSET  0x105e  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define IMXRT_EDMA_TCD3_SADDR_OFFSET        0x1060  /* TCD Source Address */
#define IMXRT_EDMA_TCD3_SOFF_OFFSET         0x1064  /* TCD Signed Source Address Offset */
#define IMXRT_EDMA_TCD3_ATTR_OFFSET         0x1066  /* TCD Transfer Attributes */
#define IMXRT_EDMA_TCD3_NBYTES_ML_OFFSET    0x1068  /* TCD Signed Minor Loop Offset / Byte Count */
#define IMXRT_EDMA_TCD3_SLAST_OFFSET        0x106c  /* TCD Last Source Address Adjustment */
#define IMXRT_EDMA_TCD3_DADDR_OFFSET        0x1070  /* TCD Destination Address */
#define IMXRT_EDMA_TCD3_DOFF_OFFSET         0x1074  /* TCD Signed Destination Address Offset */
#define IMXRT_EDMA_TCD3_CITER_ELINK_OFFSET  0x1076  /* TCD Current Minor Loop Link, Major Loop Count */
#define IMXRT_EDMA_TCD3_DLASTSGA_OFFSET     0x1078  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define IMXRT_EDMA_TCD3_CSR_OFFSET          0x107c  /* TCD Control and Status */
#define IMXRT_EDMA_TCD3_BITER_ELINK_OFFSET  0x107e  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define IMXRT_EDMA_TCD4_SADDR_OFFSET        0x1080  /* TCD Source Address */
#define IMXRT_EDMA_TCD4_SOFF_OFFSET         0x1084  /* TCD Signed Source Address Offset */
#define IMXRT_EDMA_TCD4_ATTR_OFFSET         0x1086  /* TCD Transfer Attributes */
#define IMXRT_EDMA_TCD4_NBYTES_ML_OFFSET    0x1088  /* TCD Signed Minor Loop Offset / Byte Count */
#define IMXRT_EDMA_TCD4_SLAST_OFFSET        0x108c  /* TCD Last Source Address Adjustment */
#define IMXRT_EDMA_TCD4_DADDR_OFFSET        0x1090  /* TCD Destination Address */
#define IMXRT_EDMA_TCD4_DOFF_OFFSET         0x1094  /* TCD Signed Destination Address Offset */
#define IMXRT_EDMA_TCD4_CITER_ELINK_OFFSET  0x1096  /* TCD Current Minor Loop Link, Major Loop Count */
#define IMXRT_EDMA_TCD4_DLASTSGA_OFFSET     0x1098  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define IMXRT_EDMA_TCD4_CSR_OFFSET          0x109c  /* TCD Control and Status */
#define IMXRT_EDMA_TCD4_BITER_ELINK_OFFSET  0x109e  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define IMXRT_EDMA_TCD5_SADDR_OFFSET        0x10a0  /* TCD Source Address */
#define IMXRT_EDMA_TCD5_SOFF_OFFSET         0x10a4  /* TCD Signed Source Address Offset */
#define IMXRT_EDMA_TCD5_ATTR_OFFSET         0x10a6  /* TCD Transfer Attributes */
#define IMXRT_EDMA_TCD5_NBYTES_ML_OFFSET    0x10a8  /* TCD Signed Minor Loop Offset / Byte Count */
#define IMXRT_EDMA_TCD5_SLAST_OFFSET        0x10ac  /* TCD Last Source Address Adjustment */
#define IMXRT_EDMA_TCD5_DADDR_OFFSET        0x10b0  /* TCD Destination Address */
#define IMXRT_EDMA_TCD5_DOFF_OFFSET         0x10b4  /* TCD Signed Destination Address Offset */
#define IMXRT_EDMA_TCD5_CITER_ELINK_OFFSET  0x10b6  /* TCD Current Minor Loop Link, Major Loop Count */
#define IMXRT_EDMA_TCD5_DLASTSGA_OFFSET     0x10b8  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define IMXRT_EDMA_TCD5_CSR_OFFSET          0x10bc  /* TCD Control and Status */
#define IMXRT_EDMA_TCD5_BITER_ELINK_OFFSET  0x10be  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define IMXRT_EDMA_TCD6_SADDR_OFFSET        0x10c0  /* TCD Source Address */
#define IMXRT_EDMA_TCD6_SOFF_OFFSET         0x10c4  /* TCD Signed Source Address Offset */
#define IMXRT_EDMA_TCD6_ATTR_OFFSET         0x10c6  /* TCD Transfer Attributes */
#define IMXRT_EDMA_TCD6_NBYTES_ML_OFFSET    0x10c8  /* TCD Signed Minor Loop Offset / Byte Count */
#define IMXRT_EDMA_TCD6_SLAST_OFFSET        0x10cc  /* TCD Last Source Address Adjustment */
#define IMXRT_EDMA_TCD6_DADDR_OFFSET        0x10d0  /* TCD Destination Address */
#define IMXRT_EDMA_TCD6_DOFF_OFFSET         0x10d4  /* TCD Signed Destination Address Offset */
#define IMXRT_EDMA_TCD6_CITER_ELINK_OFFSET  0x10d6  /* TCD Current Minor Loop Link, Major Loop Count */
#define IMXRT_EDMA_TCD6_DLASTSGA_OFFSET     0x10d8  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define IMXRT_EDMA_TCD6_CSR_OFFSET          0x10dc  /* TCD Control and Status */
#define IMXRT_EDMA_TCD6_BITER_ELINK_OFFSET  0x10de  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define IMXRT_EDMA_TCD7_SADDR_OFFSET        0x10e0  /* TCD Source Address */
#define IMXRT_EDMA_TCD7_SOFF_OFFSET         0x10e4  /* TCD Signed Source Address Offset */
#define IMXRT_EDMA_TCD7_ATTR_OFFSET         0x10e6  /* TCD Transfer Attributes */
#define IMXRT_EDMA_TCD7_NBYTES_ML_OFFSET    0x10e8  /* TCD Signed Minor Loop Offset / Byte Count */
#define IMXRT_EDMA_TCD7_SLAST_OFFSET        0x10ec  /* TCD Last Source Address Adjustment */
#define IMXRT_EDMA_TCD7_DADDR_OFFSET        0x10f0  /* TCD Destination Address */
#define IMXRT_EDMA_TCD7_DOFF_OFFSET         0x10f4  /* TCD Signed Destination Address Offset */
#define IMXRT_EDMA_TCD7_CITER_ELINK_OFFSET  0x10f6  /* TCD Current Minor Loop Link, Major Loop Count */
#define IMXRT_EDMA_TCD7_DLASTSGA_OFFSET     0x10f8  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define IMXRT_EDMA_TCD7_CSR_OFFSET          0x10fc  /* TCD Control and Status */
#define IMXRT_EDMA_TCD7_BITER_ELINK_OFFSET  0x10fe  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define IMXRT_EDMA_TCD8_SADDR_OFFSET        0x1100  /* TCD Source Address */
#define IMXRT_EDMA_TCD8_SOFF_OFFSET         0x1104  /* TCD Signed Source Address Offset */
#define IMXRT_EDMA_TCD8_ATTR_OFFSET         0x1106  /* TCD Transfer Attributes */
#define IMXRT_EDMA_TCD8_NBYTES_ML_OFFSET    0x1108  /* TCD Signed Minor Loop Offset / Byte Count */
#define IMXRT_EDMA_TCD8_SLAST_OFFSET        0x110c  /* TCD Last Source Address Adjustment */
#define IMXRT_EDMA_TCD8_DADDR_OFFSET        0x1110  /* TCD Destination Address */
#define IMXRT_EDMA_TCD8_DOFF_OFFSET         0x1114  /* TCD Signed Destination Address Offset */
#define IMXRT_EDMA_TCD8_CITER_ELINK_OFFSET  0x1116  /* TCD Current Minor Loop Link, Major Loop Count */
#define IMXRT_EDMA_TCD8_DLASTSGA_OFFSET     0x1118  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define IMXRT_EDMA_TCD8_CSR_OFFSET          0x111c  /* TCD Control and Status */
#define IMXRT_EDMA_TCD8_BITER_ELINK_OFFSET  0x111e  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define IMXRT_EDMA_TCD9_SADDR_OFFSET        0x1120  /* TCD Source Address */
#define IMXRT_EDMA_TCD9_SOFF_OFFSET         0x1124  /* TCD Signed Source Address Offset */
#define IMXRT_EDMA_TCD9_ATTR_OFFSET         0x1126  /* TCD Transfer Attributes */
#define IMXRT_EDMA_TCD9_NBYTES_ML_OFFSET    0x1128  /* TCD Signed Minor Loop Offset / Byte Count */
#define IMXRT_EDMA_TCD9_SLAST_OFFSET        0x112c  /* TCD Last Source Address Adjustment */
#define IMXRT_EDMA_TCD9_DADDR_OFFSET        0x1130  /* TCD Destination Address */
#define IMXRT_EDMA_TCD9_DOFF_OFFSET         0x1134  /* TCD Signed Destination Address Offset */
#define IMXRT_EDMA_TCD9_CITER_ELINK_OFFSET  0x1136  /* TCD Current Minor Loop Link, Major Loop Count */
#define IMXRT_EDMA_TCD9_DLASTSGA_OFFSET     0x1138  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define IMXRT_EDMA_TCD9_CSR_OFFSET          0x113c  /* TCD Control and Status */
#define IMXRT_EDMA_TCD9_BITER_ELINK_OFFSET  0x113e  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define IMXRT_EDMA_TCD10_SADDR_OFFSET       0x1140  /* TCD Source Address */
#define IMXRT_EDMA_TCD10_SOFF_OFFSET        0x1144  /* TCD Signed Source Address Offset */
#define IMXRT_EDMA_TCD10_ATTR_OFFSET        0x1146  /* TCD Transfer Attributes */
#define IMXRT_EDMA_TCD10_NBYTES_ML_OFFSET   0x1148  /* TCD Signed Minor Loop Offset / Byte Count */
#define IMXRT_EDMA_TCD10_SLAST_OFFSET       0x114c  /* TCD Last Source Address Adjustment */
#define IMXRT_EDMA_TCD10_DADDR_OFFSET       0x1150  /* TCD Destination Address */
#define IMXRT_EDMA_TCD10_DOFF_OFFSET        0x1154  /* TCD Signed Destination Address Offset */
#define IMXRT_EDMA_TCD10_CITER_ELINK_OFFSET 0x1156  /* TCD Current Minor Loop Link, Major Loop Count */
#define IMXRT_EDMA_TCD10_DLASTSGA_OFFSET    0x1158  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define IMXRT_EDMA_TCD10_CSR_OFFSET         0x115c  /* TCD Control and Status */
#define IMXRT_EDMA_TCD10_BITER_ELINK_OFFSET 0x115e  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define IMXRT_EDMA_TCD11_SADDR_OFFSET       0x1160  /* TCD Source Address */
#define IMXRT_EDMA_TCD11_SOFF_OFFSET        0x1164  /* TCD Signed Source Address Offset */
#define IMXRT_EDMA_TCD11_ATTR_OFFSET        0x1166  /* TCD Transfer Attributes */
#define IMXRT_EDMA_TCD11_NBYTES_ML_OFFSET   0x1168  /* TCD Signed Minor Loop Offset / Byte Count */
#define IMXRT_EDMA_TCD11_SLAST_OFFSET       0x116c  /* TCD Last Source Address Adjustment */
#define IMXRT_EDMA_TCD11_DADDR_OFFSET       0x1170  /* TCD Destination Address */
#define IMXRT_EDMA_TCD11_DOFF_OFFSET        0x1174  /* TCD Signed Destination Address Offset */
#define IMXRT_EDMA_TCD11_CITER_ELINK_OFFSET 0x1176  /* TCD Current Minor Loop Link, Major Loop Count */
#define IMXRT_EDMA_TCD11_DLASTSGA_OFFSET    0x1178  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define IMXRT_EDMA_TCD11_CSR_OFFSET         0x117c  /* TCD Control and Status */
#define IMXRT_EDMA_TCD11_BITER_ELINK_OFFSET 0x117e  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define IMXRT_EDMA_TCD12_SADDR_OFFSET       0x1180  /* TCD Source Address */
#define IMXRT_EDMA_TCD12_SOFF_OFFSET        0x1184  /* TCD Signed Source Address Offset */
#define IMXRT_EDMA_TCD12_ATTR_OFFSET        0x1186  /* TCD Transfer Attributes */
#define IMXRT_EDMA_TCD12_NBYTES_ML_OFFSET   0x1188  /* TCD Signed Minor Loop Offset / Byte Count */
#define IMXRT_EDMA_TCD12_SLAST_OFFSET       0x118c  /* TCD Last Source Address Adjustment */
#define IMXRT_EDMA_TCD12_DADDR_OFFSET       0x1190  /* TCD Destination Address */
#define IMXRT_EDMA_TCD12_DOFF_OFFSET        0x1194  /* TCD Signed Destination Address Offset */
#define IMXRT_EDMA_TCD12_CITER_ELINK_OFFSET 0x1196  /* TCD Current Minor Loop Link, Major Loop Count */
#define IMXRT_EDMA_TCD12_DLASTSGA_OFFSET    0x1198  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define IMXRT_EDMA_TCD12_CSR_OFFSET         0x119c  /* TCD Control and Status */
#define IMXRT_EDMA_TCD12_BITER_ELINK_OFFSET 0x119e  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define IMXRT_EDMA_TCD13_SADDR_OFFSET       0x11a0  /* TCD Source Address */
#define IMXRT_EDMA_TCD13_SOFF_OFFSET        0x11a4  /* TCD Signed Source Address Offset */
#define IMXRT_EDMA_TCD13_ATTR_OFFSET        0x11a6  /* TCD Transfer Attributes */
#define IMXRT_EDMA_TCD13_NBYTES_ML_OFFSET   0x11a8  /* TCD Signed Minor Loop Offset / Byte Count */
#define IMXRT_EDMA_TCD13_SLAST_OFFSET       0x11ac  /* TCD Last Source Address Adjustment */
#define IMXRT_EDMA_TCD13_DADDR_OFFSET       0x11b0  /* TCD Destination Address */
#define IMXRT_EDMA_TCD13_DOFF_OFFSET        0x11b4  /* TCD Signed Destination Address Offset */
#define IMXRT_EDMA_TCD13_CITER_ELINK_OFFSET 0x11b6  /* TCD Current Minor Loop Link, Major Loop Count */
#define IMXRT_EDMA_TCD13_DLASTSGA_OFFSET    0x11b8  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define IMXRT_EDMA_TCD13_CSR_OFFSET         0x11bc  /* TCD Control and Status */
#define IMXRT_EDMA_TCD13_BITER_ELINK_OFFSET 0x11be  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define IMXRT_EDMA_TCD14_SADDR_OFFSET       0x11c0  /* TCD Source Address */
#define IMXRT_EDMA_TCD14_SOFF_OFFSET        0x11c4  /* TCD Signed Source Address Offset */
#define IMXRT_EDMA_TCD14_ATTR_OFFSET        0x11c6  /* TCD Transfer Attributes */
#define IMXRT_EDMA_TCD14_NBYTES_ML_OFFSET   0x11c8  /* TCD Signed Minor Loop Offset / Byte Count */
#define IMXRT_EDMA_TCD14_SLAST_OFFSET       0x11cc  /* TCD Last Source Address Adjustment */
#define IMXRT_EDMA_TCD14_DADDR_OFFSET       0x11d0  /* TCD Destination Address */
#define IMXRT_EDMA_TCD14_DOFF_OFFSET        0x11d4  /* TCD Signed Destination Address Offset */
#define IMXRT_EDMA_TCD14_CITER_ELINK_OFFSET 0x11d6  /* TCD Current Minor Loop Link, Major Loop Count */
#define IMXRT_EDMA_TCD14_DLASTSGA_OFFSET    0x11d8  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define IMXRT_EDMA_TCD14_CSR_OFFSET         0x11dc  /* TCD Control and Status */
#define IMXRT_EDMA_TCD14_BITER_ELINK_OFFSET 0x11de  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define IMXRT_EDMA_TCD15_SADDR_OFFSET       0x11e0  /* TCD Source Address */
#define IMXRT_EDMA_TCD15_SOFF_OFFSET        0x11e4  /* TCD Signed Source Address Offset */
#define IMXRT_EDMA_TCD15_ATTR_OFFSET        0x11e6  /* TCD Transfer Attributes */
#define IMXRT_EDMA_TCD15_NBYTES_ML_OFFSET   0x11e8  /* TCD Signed Minor Loop Offset / Byte Count */
#define IMXRT_EDMA_TCD15_SLAST_OFFSET       0x11ec  /* TCD Last Source Address Adjustment */
#define IMXRT_EDMA_TCD15_DADDR_OFFSET       0x11f0  /* TCD Destination Address */
#define IMXRT_EDMA_TCD15_DOFF_OFFSET        0x11f4  /* TCD Signed Destination Address Offset */
#define IMXRT_EDMA_TCD15_CITER_ELINK_OFFSET 0x11f6  /* TCD Current Minor Loop Link, Major Loop Count */
#define IMXRT_EDMA_TCD15_DLASTSGA_OFFSET    0x11f8  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define IMXRT_EDMA_TCD15_CSR_OFFSET         0x11fc  /* TCD Control and Status */
#define IMXRT_EDMA_TCD15_BITER_ELINK_OFFSET 0x11fe  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define IMXRT_EDMA_TCD16_SADDR_OFFSET       0x1200  /* TCD Source Address */
#define IMXRT_EDMA_TCD16_SOFF_OFFSET        0x1204  /* TCD Signed Source Address Offset */
#define IMXRT_EDMA_TCD16_ATTR_OFFSET        0x1206  /* TCD Transfer Attributes */
#define IMXRT_EDMA_TCD16_NBYTES_ML_OFFSET   0x1208  /* TCD Signed Minor Loop Offset / Byte Count */
#define IMXRT_EDMA_TCD16_SLAST_OFFSET       0x120c  /* TCD Last Source Address Adjustment */
#define IMXRT_EDMA_TCD16_DADDR_OFFSET       0x1210  /* TCD Destination Address */
#define IMXRT_EDMA_TCD16_DOFF_OFFSET        0x1214  /* TCD Signed Destination Address Offset */
#define IMXRT_EDMA_TCD16_CITER_ELINK_OFFSET 0x1216  /* TCD Current Minor Loop Link, Major Loop Count */
#define IMXRT_EDMA_TCD16_DLASTSGA_OFFSET    0x1218  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define IMXRT_EDMA_TCD16_CSR_OFFSET         0x121c  /* TCD Control and Status */
#define IMXRT_EDMA_TCD16_BITER_ELINK_OFFSET 0x121e  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define IMXRT_EDMA_TCD17_SADDR_OFFSET       0x1220  /* TCD Source Address */
#define IMXRT_EDMA_TCD17_SOFF_OFFSET        0x1224  /* TCD Signed Source Address Offset */
#define IMXRT_EDMA_TCD17_ATTR_OFFSET        0x1226  /* TCD Transfer Attributes */
#define IMXRT_EDMA_TCD17_NBYTES_ML_OFFSET   0x1228  /* TCD Signed Minor Loop Offset / Byte Count */
#define IMXRT_EDMA_TCD17_SLAST_OFFSET       0x122c  /* TCD Last Source Address Adjustment */
#define IMXRT_EDMA_TCD17_DADDR_OFFSET       0x1230  /* TCD Destination Address */
#define IMXRT_EDMA_TCD17_DOFF_OFFSET        0x1234  /* TCD Signed Destination Address Offset */
#define IMXRT_EDMA_TCD17_CITER_ELINK_OFFSET 0x1236  /* TCD Current Minor Loop Link, Major Loop Count */
#define IMXRT_EDMA_TCD17_DLASTSGA_OFFSET    0x1238  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define IMXRT_EDMA_TCD17_CSR_OFFSET         0x123c  /* TCD Control and Status */
#define IMXRT_EDMA_TCD17_BITER_ELINK_OFFSET 0x123e  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define IMXRT_EDMA_TCD18_SADDR_OFFSET       0x1240  /* TCD Source Address */
#define IMXRT_EDMA_TCD18_SOFF_OFFSET        0x1244  /* TCD Signed Source Address Offset */
#define IMXRT_EDMA_TCD18_ATTR_OFFSET        0x1246  /* TCD Transfer Attributes */
#define IMXRT_EDMA_TCD18_NBYTES_ML_OFFSET   0x1248  /* TCD Signed Minor Loop Offset / Byte Count */
#define IMXRT_EDMA_TCD18_SLAST_OFFSET       0x124c  /* TCD Last Source Address Adjustment */
#define IMXRT_EDMA_TCD18_DADDR_OFFSET       0x1250  /* TCD Destination Address */
#define IMXRT_EDMA_TCD18_DOFF_OFFSET        0x1254  /* TCD Signed Destination Address Offset */
#define IMXRT_EDMA_TCD18_CITER_ELINK_OFFSET 0x1256  /* TCD Current Minor Loop Link, Major Loop Count */
#define IMXRT_EDMA_TCD18_DLASTSGA_OFFSET    0x1258  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define IMXRT_EDMA_TCD18_CSR_OFFSET         0x125c  /* TCD Control and Status */
#define IMXRT_EDMA_TCD18_BITER_ELINK_OFFSET 0x125e  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define IMXRT_EDMA_TCD19_SADDR_OFFSET       0x1260  /* TCD Source Address */
#define IMXRT_EDMA_TCD19_SOFF_OFFSET        0x1264  /* TCD Signed Source Address Offset */
#define IMXRT_EDMA_TCD19_ATTR_OFFSET        0x1266  /* TCD Transfer Attributes */
#define IMXRT_EDMA_TCD19_NBYTES_ML_OFFSET   0x1268  /* TCD Signed Minor Loop Offset / Byte Count */
#define IMXRT_EDMA_TCD19_SLAST_OFFSET       0x126c  /* TCD Last Source Address Adjustment */
#define IMXRT_EDMA_TCD19_DADDR_OFFSET       0x1270  /* TCD Destination Address */
#define IMXRT_EDMA_TCD19_DOFF_OFFSET        0x1274  /* TCD Signed Destination Address Offset */
#define IMXRT_EDMA_TCD19_CITER_ELINK_OFFSET 0x1276  /* TCD Current Minor Loop Link, Major Loop Count */
#define IMXRT_EDMA_TCD19_DLASTSGA_OFFSET    0x1278  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define IMXRT_EDMA_TCD19_CSR_OFFSET         0x127c  /* TCD Control and Status */
#define IMXRT_EDMA_TCD19_BITER_ELINK_OFFSET 0x127e  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define IMXRT_EDMA_TCD20_SADDR_OFFSET       0x1280  /* TCD Source Address */
#define IMXRT_EDMA_TCD20_SOFF_OFFSET        0x1284  /* TCD Signed Source Address Offset */
#define IMXRT_EDMA_TCD20_ATTR_OFFSET        0x1286  /* TCD Transfer Attributes */
#define IMXRT_EDMA_TCD20_NBYTES_ML_OFFSET   0x1288  /* TCD Signed Minor Loop Offset / Byte Count */
#define IMXRT_EDMA_TCD20_SLAST_OFFSET       0x128c  /* TCD Last Source Address Adjustment */
#define IMXRT_EDMA_TCD20_DADDR_OFFSET       0x1290  /* TCD Destination Address */
#define IMXRT_EDMA_TCD20_DOFF_OFFSET        0x1294  /* TCD Signed Destination Address Offset */
#define IMXRT_EDMA_TCD20_CITER_ELINK_OFFSET 0x1296  /* TCD Current Minor Loop Link, Major Loop Count */
#define IMXRT_EDMA_TCD20_DLASTSGA_OFFSET    0x1298  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define IMXRT_EDMA_TCD20_CSR_OFFSET         0x129c  /* TCD Control and Status */
#define IMXRT_EDMA_TCD20_BITER_ELINK_OFFSET 0x129e  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define IMXRT_EDMA_TCD21_SADDR_OFFSET       0x12a0  /* TCD Source Address */
#define IMXRT_EDMA_TCD21_SOFF_OFFSET        0x12a4  /* TCD Signed Source Address Offset */
#define IMXRT_EDMA_TCD21_ATTR_OFFSET        0x12a6  /* TCD Transfer Attributes */
#define IMXRT_EDMA_TCD21_NBYTES_ML_OFFSET   0x12a8  /* TCD Signed Minor Loop Offset / Byte Count */
#define IMXRT_EDMA_TCD21_SLAST_OFFSET       0x12ac  /* TCD Last Source Address Adjustment */
#define IMXRT_EDMA_TCD21_DADDR_OFFSET       0x12b0  /* TCD Destination Address */
#define IMXRT_EDMA_TCD21_DOFF_OFFSET        0x12b4  /* TCD Signed Destination Address Offset */
#define IMXRT_EDMA_TCD21_CITER_ELINK_OFFSET 0x12b6  /* TCD Current Minor Loop Link, Major Loop Count */
#define IMXRT_EDMA_TCD21_DLASTSGA_OFFSET    0x12b8  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define IMXRT_EDMA_TCD21_CSR_OFFSET         0x12bc  /* TCD Control and Status */
#define IMXRT_EDMA_TCD21_BITER_ELINK_OFFSET 0x12be  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define IMXRT_EDMA_TCD22_SADDR_OFFSET       0x12c0  /* TCD Source Address */
#define IMXRT_EDMA_TCD22_SOFF_OFFSET        0x12c4  /* TCD Signed Source Address Offset */
#define IMXRT_EDMA_TCD22_ATTR_OFFSET        0x12c6  /* TCD Transfer Attributes */
#define IMXRT_EDMA_TCD22_NBYTES_ML_OFFSET   0x12c8  /* TCD Signed Minor Loop Offset / Byte Count */
#define IMXRT_EDMA_TCD22_SLAST_OFFSET       0x12cc  /* TCD Last Source Address Adjustment */
#define IMXRT_EDMA_TCD22_DADDR_OFFSET       0x12d0  /* TCD Destination Address */
#define IMXRT_EDMA_TCD22_DOFF_OFFSET        0x12d4  /* TCD Signed Destination Address Offset */
#define IMXRT_EDMA_TCD22_CITER_ELINK_OFFSET 0x12d6  /* TCD Current Minor Loop Link, Major Loop Count */
#define IMXRT_EDMA_TCD22_DLASTSGA_OFFSET    0x12d8  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define IMXRT_EDMA_TCD22_CSR_OFFSET         0x12dc  /* TCD Control and Status */
#define IMXRT_EDMA_TCD22_BITER_ELINK_OFFSET 0x12de  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define IMXRT_EDMA_TCD23_SADDR_OFFSET       0x12e0  /* TCD Source Address */
#define IMXRT_EDMA_TCD23_SOFF_OFFSET        0x12e4  /* TCD Signed Source Address Offset */
#define IMXRT_EDMA_TCD23_ATTR_OFFSET        0x12e6  /* TCD Transfer Attributes */
#define IMXRT_EDMA_TCD23_NBYTES_ML_OFFSET   0x12e8  /* TCD Signed Minor Loop Offset / Byte Count */
#define IMXRT_EDMA_TCD23_SLAST_OFFSET       0x12ec  /* TCD Last Source Address Adjustment */
#define IMXRT_EDMA_TCD23_DADDR_OFFSET       0x12f0  /* TCD Destination Address */
#define IMXRT_EDMA_TCD23_DOFF_OFFSET        0x12f4  /* TCD Signed Destination Address Offset */
#define IMXRT_EDMA_TCD23_CITER_ELINK_OFFSET 0x12f6  /* TCD Current Minor Loop Link, Major Loop Count */
#define IMXRT_EDMA_TCD23_DLASTSGA_OFFSET    0x12f8  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define IMXRT_EDMA_TCD23_CSR_OFFSET         0x12fc  /* TCD Control and Status */
#define IMXRT_EDMA_TCD23_BITER_ELINK_OFFSET 0x12fe  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define IMXRT_EDMA_TCD24_SADDR_OFFSET       0x1300  /* TCD Source Address */
#define IMXRT_EDMA_TCD24_SOFF_OFFSET        0x1304  /* TCD Signed Source Address Offset */
#define IMXRT_EDMA_TCD24_ATTR_OFFSET        0x1306  /* TCD Transfer Attributes */
#define IMXRT_EDMA_TCD24_NBYTES_ML_OFFSET   0x1308  /* TCD Signed Minor Loop Offset / Byte Count */
#define IMXRT_EDMA_TCD24_SLAST_OFFSET       0x130c  /* TCD Last Source Address Adjustment */
#define IMXRT_EDMA_TCD24_DADDR_OFFSET       0x1310  /* TCD Destination Address */
#define IMXRT_EDMA_TCD24_DOFF_OFFSET        0x1314  /* TCD Signed Destination Address Offset */
#define IMXRT_EDMA_TCD24_CITER_ELINK_OFFSET 0x1316  /* TCD Current Minor Loop Link, Major Loop Count */
#define IMXRT_EDMA_TCD24_DLASTSGA_OFFSET    0x1318  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define IMXRT_EDMA_TCD24_CSR_OFFSET         0x131c  /* TCD Control and Status */
#define IMXRT_EDMA_TCD24_BITER_ELINK_OFFSET 0x131e  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define IMXRT_EDMA_TCD25_SADDR_OFFSET       0x1320  /* TCD Source Address */
#define IMXRT_EDMA_TCD25_SOFF_OFFSET        0x1324  /* TCD Signed Source Address Offset */
#define IMXRT_EDMA_TCD25_ATTR_OFFSET        0x1326  /* TCD Transfer Attributes */
#define IMXRT_EDMA_TCD25_NBYTES_ML_OFFSET   0x1328  /* TCD Signed Minor Loop Offset / Byte Count */
#define IMXRT_EDMA_TCD25_SLAST_OFFSET       0x132c  /* TCD Last Source Address Adjustment */
#define IMXRT_EDMA_TCD25_DADDR_OFFSET       0x1330  /* TCD Destination Address */
#define IMXRT_EDMA_TCD25_DOFF_OFFSET        0x1334  /* TCD Signed Destination Address Offset */
#define IMXRT_EDMA_TCD25_CITER_ELINK_OFFSET 0x1336  /* TCD Current Minor Loop Link, Major Loop Count */
#define IMXRT_EDMA_TCD25_DLASTSGA_OFFSET    0x1338  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define IMXRT_EDMA_TCD25_CSR_OFFSET         0x133c  /* TCD Control and Status */
#define IMXRT_EDMA_TCD25_BITER_ELINK_OFFSET 0x133e  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define IMXRT_EDMA_TCD26_SADDR_OFFSET       0x1340  /* TCD Source Address */
#define IMXRT_EDMA_TCD26_SOFF_OFFSET        0x1344  /* TCD Signed Source Address Offset */
#define IMXRT_EDMA_TCD26_ATTR_OFFSET        0x1346  /* TCD Transfer Attributes */
#define IMXRT_EDMA_TCD26_NBYTES_ML_OFFSET   0x1348  /* TCD Signed Minor Loop Offset / Byte Count */
#define IMXRT_EDMA_TCD26_SLAST_OFFSET       0x134c  /* TCD Last Source Address Adjustment */
#define IMXRT_EDMA_TCD26_DADDR_OFFSET       0x1350  /* TCD Destination Address */
#define IMXRT_EDMA_TCD26_DOFF_OFFSET        0x1354  /* TCD Signed Destination Address Offset */
#define IMXRT_EDMA_TCD26_CITER_ELINK_OFFSET 0x1356  /* TCD Current Minor Loop Link, Major Loop Count */
#define IMXRT_EDMA_TCD26_DLASTSGA_OFFSET    0x1358  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define IMXRT_EDMA_TCD26_CSR_OFFSET         0x135c  /* TCD Control and Status */
#define IMXRT_EDMA_TCD26_BITER_ELINK_OFFSET 0x135e  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define IMXRT_EDMA_TCD27_SADDR_OFFSET       0x1360  /* TCD Source Address */
#define IMXRT_EDMA_TCD27_SOFF_OFFSET        0x1364  /* TCD Signed Source Address Offset */
#define IMXRT_EDMA_TCD27_ATTR_OFFSET        0x1366  /* TCD Transfer Attributes */
#define IMXRT_EDMA_TCD27_NBYTES_ML_OFFSET   0x1368  /* TCD Signed Minor Loop Offset / Byte Count */
#define IMXRT_EDMA_TCD27_SLAST_OFFSET       0x136c  /* TCD Last Source Address Adjustment */
#define IMXRT_EDMA_TCD27_DADDR_OFFSET       0x1370  /* TCD Destination Address */
#define IMXRT_EDMA_TCD27_DOFF_OFFSET        0x1374  /* TCD Signed Destination Address Offset */
#define IMXRT_EDMA_TCD27_CITER_ELINK_OFFSET 0x1376  /* TCD Current Minor Loop Link, Major Loop Count */
#define IMXRT_EDMA_TCD27_DLASTSGA_OFFSET    0x1378  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define IMXRT_EDMA_TCD27_CSR_OFFSET         0x137c  /* TCD Control and Status */
#define IMXRT_EDMA_TCD27_BITER_ELINK_OFFSET 0x137e  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define IMXRT_EDMA_TCD28_SADDR_OFFSET       0x1380  /* TCD Source Address */
#define IMXRT_EDMA_TCD28_SOFF_OFFSET        0x1384  /* TCD Signed Source Address Offset */
#define IMXRT_EDMA_TCD28_ATTR_OFFSET        0x1386  /* TCD Transfer Attributes */
#define IMXRT_EDMA_TCD28_NBYTES_ML_OFFSET   0x1388  /* TCD Signed Minor Loop Offset / Byte Count */
#define IMXRT_EDMA_TCD28_SLAST_OFFSET       0x138c  /* TCD Last Source Address Adjustment */
#define IMXRT_EDMA_TCD28_DADDR_OFFSET       0x1390  /* TCD Destination Address */
#define IMXRT_EDMA_TCD28_DOFF_OFFSET        0x1394  /* TCD Signed Destination Address Offset */
#define IMXRT_EDMA_TCD28_CITER_ELINK_OFFSET 0x1396  /* TCD Current Minor Loop Link, Major Loop Count */
#define IMXRT_EDMA_TCD28_DLASTSGA_OFFSET    0x1398  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define IMXRT_EDMA_TCD28_CSR_OFFSET         0x139c  /* TCD Control and Status */
#define IMXRT_EDMA_TCD28_BITER_ELINK_OFFSET 0x139e  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define IMXRT_EDMA_TCD29_SADDR_OFFSET       0x13a0  /* TCD Source Address */
#define IMXRT_EDMA_TCD29_SOFF_OFFSET        0x13a4  /* TCD Signed Source Address Offset */
#define IMXRT_EDMA_TCD29_ATTR_OFFSET        0x13a6  /* TCD Transfer Attributes */
#define IMXRT_EDMA_TCD29_NBYTES_ML_OFFSET   0x13a8  /* TCD Signed Minor Loop Offset / Byte Count */
#define IMXRT_EDMA_TCD29_SLAST_OFFSET       0x13ac  /* TCD Last Source Address Adjustment */
#define IMXRT_EDMA_TCD29_DADDR_OFFSET       0x13b0  /* TCD Destination Address */
#define IMXRT_EDMA_TCD29_DOFF_OFFSET        0x13b4  /* TCD Signed Destination Address Offset */
#define IMXRT_EDMA_TCD29_CITER_ELINK_OFFSET 0x13b6  /* TCD Current Minor Loop Link, Major Loop Count */
#define IMXRT_EDMA_TCD29_DLASTSGA_OFFSET    0x13b8  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define IMXRT_EDMA_TCD29_CSR_OFFSET         0x13bc  /* TCD Control and Status */
#define IMXRT_EDMA_TCD29_BITER_ELINK_OFFSET 0x13be  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define IMXRT_EDMA_TCD30_SADDR_OFFSET       0x13c0  /* TCD Source Address */
#define IMXRT_EDMA_TCD30_SOFF_OFFSET        0x13c4  /* TCD Signed Source Address Offset */
#define IMXRT_EDMA_TCD30_ATTR_OFFSET        0x13c6  /* TCD Transfer Attributes */
#define IMXRT_EDMA_TCD30_NBYTES_ML_OFFSET   0x13c8  /* TCD Signed Minor Loop Offset / Byte Count */
#define IMXRT_EDMA_TCD30_SLAST_OFFSET       0x13cc  /* TCD Last Source Address Adjustment */
#define IMXRT_EDMA_TCD30_DADDR_OFFSET       0x13d0  /* TCD Destination Address */
#define IMXRT_EDMA_TCD30_DOFF_OFFSET        0x13d4  /* TCD Signed Destination Address Offset */
#define IMXRT_EDMA_TCD30_CITER_ELINK_OFFSET 0x13d6  /* TCD Current Minor Loop Link, Major Loop Count */
#define IMXRT_EDMA_TCD30_DLASTSGA_OFFSET    0x13d8  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define IMXRT_EDMA_TCD30_CSR_OFFSET         0x13dc  /* TCD Control and Status */
#define IMXRT_EDMA_TCD30_BITER_ELINK_OFFSET 0x13de  /* TCD Beginning Minor Loop Link, Major Loop Count */

#define IMXRT_EDMA_TCD31_SADDR_OFFSET       0x13e0  /* TCD Source Address */
#define IMXRT_EDMA_TCD31_SOFF_OFFSET        0x13e4  /* TCD Signed Source Address Offset */
#define IMXRT_EDMA_TCD31_ATTR_OFFSET        0x13e6  /* TCD Transfer Attributes */
#define IMXRT_EDMA_TCD31_NBYTES_ML_OFFSET   0x13e8  /* TCD Signed Minor Loop Offset / Byte Count */
#define IMXRT_EDMA_TCD31_SLAST_OFFSET       0x13ec  /* TCD Last Source Address Adjustment */
#define IMXRT_EDMA_TCD31_DADDR_OFFSET       0x13f0  /* TCD Destination Address */
#define IMXRT_EDMA_TCD31_DOFF_OFFSET        0x13f4  /* TCD Signed Destination Address Offset */
#define IMXRT_EDMA_TCD31_CITER_ELINK_OFFSET 0x13f6  /* TCD Current Minor Loop Link, Major Loop Count */
#define IMXRT_EDMA_TCD31_DLASTSGA_OFFSET    0x13f8  /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define IMXRT_EDMA_TCD31_CSR_OFFSET         0x13fc  /* TCD Control and Status */
#define IMXRT_EDMA_TCD31_BITER_ELINK_OFFSET 0x13fe  /* TCD Beginning Minor Loop Link, Major Loop Count */

/* eDMA Register Addresses **************************************************************************/

#define IMXRT_EDMA_CR                       (IMXRT_EDMA_BASE + IMXRT_EDMA_CR_OFFSET)
#define IMXRT_EDMA_ES                       (IMXRT_EDMA_BASE + IMXRT_EDMA_ES_OFFSET)
#define IMXRT_EDMA_ERQ                      (IMXRT_EDMA_BASE + IMXRT_EDMA_ERQ_OFFSET)
#define IMXRT_EDMA_EEI                      (IMXRT_EDMA_BASE + IMXRT_EDMA_EEI_OFFSET)
#define IMXRT_EDMA_CEEI                     (IMXRT_EDMA_BASE + IMXRT_EDMA_CEEI_OFFSET)
#define IMXRT_EDMA_SEEI                     (IMXRT_EDMA_BASE + IMXRT_EDMA_SEEI_OFFSET)
#define IMXRT_EDMA_CERQ                     (IMXRT_EDMA_BASE + IMXRT_EDMA_CERQ_OFFSET)
#define IMXRT_EDMA_SERQ                     (IMXRT_EDMA_BASE + IMXRT_EDMA_SERQ_OFFSET)
#define IMXRT_EDMA_CDNE                     (IMXRT_EDMA_BASE + IMXRT_EDMA_CDNE_OFFSET)
#define IMXRT_EDMA_SSRT                     (IMXRT_EDMA_BASE + IMXRT_EDMA_SSRT_OFFSET)
#define IMXRT_EDMA_CERR                     (IMXRT_EDMA_BASE + IMXRT_EDMA_CERR_OFFSET)
#define IMXRT_EDMA_CINT                     (IMXRT_EDMA_BASE + IMXRT_EDMA_CINT_OFFSET)
#define IMXRT_EDMA_INT                      (IMXRT_EDMA_BASE + IMXRT_EDMA_INT_OFFSET)
#define IMXRT_EDMA_ERR                      (IMXRT_EDMA_BASE + IMXRT_EDMA_ERR_OFFSET)
#define IMXRT_EDMA_HRS                      (IMXRT_EDMA_BASE + IMXRT_EDMA_HRS_OFFSET)
#define IMXRT_EDMA_EARS                     (IMXRT_EDMA_BASE + IMXRT_EDMA_EARS_OFFSET)

#define IMXRT_EDMA_DCHPRI(n)                (IMXRT_EDMA_BASE + IMXRT_EDMA_DCHPRI_OFFSET(n))

#define IMXRT_EDMA_DCHPRI0                  (IMXRT_EDMA_BASE + IMXRT_EDMA_DCHPRI0_OFFSET)
#define IMXRT_EDMA_DCHPRI1                  (IMXRT_EDMA_BASE + IMXRT_EDMA_DCHPRI1_OFFSET)
#define IMXRT_EDMA_DCHPRI2                  (IMXRT_EDMA_BASE + IMXRT_EDMA_DCHPRI2_OFFSET)
#define IMXRT_EDMA_DCHPRI3                  (IMXRT_EDMA_BASE + IMXRT_EDMA_DCHPRI3_OFFSET)
#define IMXRT_EDMA_DCHPRI4                  (IMXRT_EDMA_BASE + IMXRT_EDMA_DCHPRI4_OFFSET)
#define IMXRT_EDMA_DCHPRI5                  (IMXRT_EDMA_BASE + IMXRT_EDMA_DCHPRI5_OFFSET)
#define IMXRT_EDMA_DCHPRI6                  (IMXRT_EDMA_BASE + IMXRT_EDMA_DCHPRI6_OFFSET)
#define IMXRT_EDMA_DCHPRI7                  (IMXRT_EDMA_BASE + IMXRT_EDMA_DCHPRI7_OFFSET)
#define IMXRT_EDMA_DCHPRI8                  (IMXRT_EDMA_BASE + IMXRT_EDMA_DCHPRI8_OFFSET)
#define IMXRT_EDMA_DCHPRI9                  (IMXRT_EDMA_BASE + IMXRT_EDMA_DCHPRI9_OFFSET)
#define IMXRT_EDMA_DCHPRI10                 (IMXRT_EDMA_BASE + IMXRT_EDMA_DCHPRI10_OFFSET)
#define IMXRT_EDMA_DCHPRI11                 (IMXRT_EDMA_BASE + IMXRT_EDMA_DCHPRI11_OFFSET)
#define IMXRT_EDMA_DCHPRI12                 (IMXRT_EDMA_BASE + IMXRT_EDMA_DCHPRI12_OFFSET)
#define IMXRT_EDMA_DCHPRI13                 (IMXRT_EDMA_BASE + IMXRT_EDMA_DCHPRI13_OFFSET)
#define IMXRT_EDMA_DCHPRI14                 (IMXRT_EDMA_BASE + IMXRT_EDMA_DCHPRI14_OFFSET)
#define IMXRT_EDMA_DCHPRI15                 (IMXRT_EDMA_BASE + IMXRT_EDMA_DCHPRI15_OFFSET)
#define IMXRT_EDMA_DCHPRI16                 (IMXRT_EDMA_BASE + IMXRT_EDMA_DCHPRI16_OFFSET)
#define IMXRT_EDMA_DCHPRI17                 (IMXRT_EDMA_BASE + IMXRT_EDMA_DCHPRI17_OFFSET)
#define IMXRT_EDMA_DCHPRI18                 (IMXRT_EDMA_BASE + IMXRT_EDMA_DCHPRI18_OFFSET)
#define IMXRT_EDMA_DCHPRI19                 (IMXRT_EDMA_BASE + IMXRT_EDMA_DCHPRI19_OFFSET)
#define IMXRT_EDMA_DCHPRI20                 (IMXRT_EDMA_BASE + IMXRT_EDMA_DCHPRI20_OFFSET)
#define IMXRT_EDMA_DCHPRI21                 (IMXRT_EDMA_BASE + IMXRT_EDMA_DCHPRI21_OFFSET)
#define IMXRT_EDMA_DCHPRI22                 (IMXRT_EDMA_BASE + IMXRT_EDMA_DCHPRI22_OFFSET)
#define IMXRT_EDMA_DCHPRI23                 (IMXRT_EDMA_BASE + IMXRT_EDMA_DCHPRI23_OFFSET)
#define IMXRT_EDMA_DCHPRI24                 (IMXRT_EDMA_BASE + IMXRT_EDMA_DCHPRI24_OFFSET)
#define IMXRT_EDMA_DCHPRI25                 (IMXRT_EDMA_BASE + IMXRT_EDMA_DCHPRI25_OFFSET)
#define IMXRT_EDMA_DCHPRI26                 (IMXRT_EDMA_BASE + IMXRT_EDMA_DCHPRI26_OFFSET)
#define IMXRT_EDMA_DCHPRI27                 (IMXRT_EDMA_BASE + IMXRT_EDMA_DCHPRI27_OFFSET)
#define IMXRT_EDMA_DCHPRI28                 (IMXRT_EDMA_BASE + IMXRT_EDMA_DCHPRI28_OFFSET)
#define IMXRT_EDMA_DCHPRI29                 (IMXRT_EDMA_BASE + IMXRT_EDMA_DCHPRI29_OFFSET)
#define IMXRT_EDMA_DCHPRI30                 (IMXRT_EDMA_BASE + IMXRT_EDMA_DCHPRI30_OFFSET)
#define IMXRT_EDMA_DCHPRI31                 (IMXRT_EDMA_BASE + IMXRT_EDMA_DCHPRI31_OFFSET)

#define IMXRT_EDMA_DCHPRI3_OFFSET           0x0100  /* Channel 3 Priority */
#define IMXRT_EDMA_DCHPRI2_OFFSET           0x0101  /* Channel 2 Priority */
#define IMXRT_EDMA_DCHPRI1_OFFSET           0x0102  /* Channel 1 Priority */
#define IMXRT_EDMA_DCHPRI0_OFFSET           0x0103  /* Channel 0 Priority */
#define IMXRT_EDMA_DCHPRI7_OFFSET           0x0104  /* Channel 7 Priority */
#define IMXRT_EDMA_DCHPRI6_OFFSET           0x0105  /* Channel 6 Priority */
#define IMXRT_EDMA_DCHPRI5_OFFSET           0x0106  /* Channel 5 Priority */
#define IMXRT_EDMA_DCHPRI4_OFFSET           0x0107  /* Channel 4 Priority */
#define IMXRT_EDMA_DCHPRI11_OFFSET          0x0108  /* Channel 11 Priority */
#define IMXRT_EDMA_DCHPRI10_OFFSET          0x0109  /* Channel 10 Priority */
#define IMXRT_EDMA_DCHPRI9_OFFSET           0x010a  /* Channel 9 Priority */
#define IMXRT_EDMA_DCHPRI8_OFFSET           0x010b  /* Channel 8 Priority */
#define IMXRT_EDMA_DCHPRI15_OFFSET          0x010c  /* Channel 15 Priority */
#define IMXRT_EDMA_DCHPRI14_OFFSET          0x010d  /* Channel 14 Priority */
#define IMXRT_EDMA_DCHPRI13_OFFSET          0x010e  /* Channel 13 Priority */
#define IMXRT_EDMA_DCHPRI12_OFFSET          0x010f  /* Channel 12 Priority */
#define IMXRT_EDMA_DCHPRI19_OFFSET          0x0110  /* Channel 19 Priority */
#define IMXRT_EDMA_DCHPRI18_OFFSET          0x0111  /* Channel 18 Priority */
#define IMXRT_EDMA_DCHPRI17_OFFSET          0x0112  /* Channel 17 Priority */
#define IMXRT_EDMA_DCHPRI16_OFFSET          0x0113  /* Channel 16 Priority */
#define IMXRT_EDMA_DCHPRI23_OFFSET          0x0114  /* Channel 23 Priority */
#define IMXRT_EDMA_DCHPRI22_OFFSET          0x0115  /* Channel 22 Priority */
#define IMXRT_EDMA_DCHPRI21_OFFSET          0x0116  /* Channel 21 Priority */
#define IMXRT_EDMA_DCHPRI20_OFFSET          0x0117  /* Channel 20 Priority */
#define IMXRT_EDMA_DCHPRI27_OFFSET          0x0118  /* Channel 27 Priority */
#define IMXRT_EDMA_DCHPRI26_OFFSET          0x0119  /* Channel 26 Priority */
#define IMXRT_EDMA_DCHPRI25_OFFSET          0x011a  /* Channel 25 Priority */
#define IMXRT_EDMA_DCHPRI24_OFFSET          0x011b  /* Channel 24 Priority */
#define IMXRT_EDMA_DCHPRI31_OFFSET          0x011c  /* Channel 31 Priority */
#define IMXRT_EDMA_DCHPRI30_OFFSET          0x011d  /* Channel 30 Priority */
#define IMXRT_EDMA_DCHPRI29_OFFSET          0x011e  /* Channel 29 Priority */
#define IMXRT_EDMA_DCHPRI28_OFFSET          0x011f  /* Channel 28 Priority */

/* Transfer Control Descriptor (TCD) */

#define IMXRT_EDMA_TCD_BASE(n)              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD_OFFSET(n))
#define IMXRT_EDMA_TCD_SADDR(n)             (IMXRT_EDMA_TCD_BASE(n) + IMXRT_EDMA_TCD_SADDR_OFFSET)
#define IMXRT_EDMA_TCD_SOFF(n)              (IMXRT_EDMA_TCD_BASE(n) + IMXRT_EDMA_TCD_SOFF_OFFSET)
#define IMXRT_EDMA_TCD_ATTR(n)              (IMXRT_EDMA_TCD_BASE(n) + IMXRT_EDMA_TCD_ATTR_OFFSET)
#define IMXRT_EDMA_TCD_NBYTES_ML(n)         (IMXRT_EDMA_TCD_BASE(n) + IMXRT_EDMA_TCD_NBYTES_ML_OFFSET)
#define IMXRT_EDMA_TCD_SLAST(n)             (IMXRT_EDMA_TCD_BASE(n) + IMXRT_EDMA_TCD_SLAST_OFFSET)
#define IMXRT_EDMA_TCD_DADDR(n)             (IMXRT_EDMA_TCD_BASE(n) + IMXRT_EDMA_TCD_DADDR_OFFSET)
#define IMXRT_EDMA_TCD_DOFF(n)              (IMXRT_EDMA_TCD_BASE(n) + IMXRT_EDMA_TCD_DOFF_OFFSET)
#define IMXRT_EDMA_TCD_CITER_ELINK(n)       (IMXRT_EDMA_TCD_BASE(n) + IMXRT_EDMA_TCD_CITER_ELINK_OFFSET)
#define IMXRT_EDMA_TCD_DLASTSGA(n)          (IMXRT_EDMA_TCD_BASE(n) + IMXRT_EDMA_TCD_DLASTSGA_OFFSET)
#define IMXRT_EDMA_TCD_CSR(n)               (IMXRT_EDMA_TCD_BASE(n) + IMXRT_EDMA_TCD_CSR_OFFSET)
#define IMXRT_EDMA_TCD_BITER_ELINK(n)       (IMXRT_EDMA_TCD_BASE(n) + IMXRT_EDMA_TCD_BITER_ELINK_OFFSET)

#define IMXRT_EDMA_TCD0_SADDR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD0_SADDR_OFFSET)
#define IMXRT_EDMA_TCD0_SOFF                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD0_SOFF_OFFSET)
#define IMXRT_EDMA_TCD0_ATTR                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD0_ATTR_OFFSET)
#define IMXRT_EDMA_TCD0_NBYTES_ML           (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD0_NBYTES_ML_OFFSET)
#define IMXRT_EDMA_TCD0_SLAST               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD0_SLAST_OFFSET)
#define IMXRT_EDMA_TCD0_DADDR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD0_DADDR_OFFSET)
#define IMXRT_EDMA_TCD0_DOFF                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD0_DOFF_OFFSET)
#define IMXRT_EDMA_TCD0_CITER_ELINK         (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD0_CITER_ELINK_OFFSET)
#define IMXRT_EDMA_TCD0_DLASTSGA            (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD0_DLASTSGA_OFFSET)
#define IMXRT_EDMA_TCD0_CSR                 (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD0_CSR_OFFSET)
#define IMXRT_EDMA_TCD0_BITER_ELINK         (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD0_BITER_ELINK_OFFSET)

#define IMXRT_EDMA_TCD1_SADDR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD1_SADDR_OFFSET)
#define IMXRT_EDMA_TCD1_SOFF                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD1_SOFF_OFFSET)
#define IMXRT_EDMA_TCD1_ATTR                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD1_ATTR_OFFSET)
#define IMXRT_EDMA_TCD1_NBYTES_ML           (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD1_NBYTES_ML_OFFSET)
#define IMXRT_EDMA_TCD1_SLAST               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD1_SLAST_OFFSET)
#define IMXRT_EDMA_TCD1_DADDR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD1_DADDR_OFFSET)
#define IMXRT_EDMA_TCD1_DOFF                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD1_DOFF_OFFSET)
#define IMXRT_EDMA_TCD1_CITER_ELINK         (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD1_CITER_ELINK_OFFSET)
#define IMXRT_EDMA_TCD1_DLASTSGA            (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD1_DLASTSGA_OFFSET)
#define IMXRT_EDMA_TCD1_CSR                 (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD1_CSR_OFFSET)
#define IMXRT_EDMA_TCD1_BITER_ELINK         (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD1_BITER_ELINK_OFFSET)

#define IMXRT_EDMA_TCD2_SADDR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD2_SADDR_OFFSET)
#define IMXRT_EDMA_TCD2_SOFF                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD2_SOFF_OFFSET)
#define IMXRT_EDMA_TCD2_ATTR                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD2_ATTR_OFFSET)
#define IMXRT_EDMA_TCD2_NBYTES_ML           (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD2_NBYTES_ML_OFFSET)
#define IMXRT_EDMA_TCD2_SLAST               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD2_SLAST_OFFSET)
#define IMXRT_EDMA_TCD2_DADDR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD2_DADDR_OFFSET)
#define IMXRT_EDMA_TCD2_DOFF                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD2_DOFF_OFFSET)
#define IMXRT_EDMA_TCD2_CITER_ELINK         (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD2_CITER_ELINK_OFFSET)
#define IMXRT_EDMA_TCD2_DLASTSGA            (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD2_DLASTSGA_OFFSET)
#define IMXRT_EDMA_TCD2_CSR                 (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD2_CSR_OFFSET)
#define IMXRT_EDMA_TCD2_BITER_ELINK         (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD2_BITER_ELINK_OFFSET)

#define IMXRT_EDMA_TCD3_SADDR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD3_SADDR_OFFSET)
#define IMXRT_EDMA_TCD3_SOFF                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD3_SOFF_OFFSET)
#define IMXRT_EDMA_TCD3_ATTR                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD3_ATTR_OFFSET)
#define IMXRT_EDMA_TCD3_NBYTES_ML           (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD3_NBYTES_ML_OFFSET)
#define IMXRT_EDMA_TCD3_SLAST               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD3_SLAST_OFFSET)
#define IMXRT_EDMA_TCD3_DADDR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD3_DADDR_OFFSET)
#define IMXRT_EDMA_TCD3_DOFF                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD3_DOFF_OFFSET)
#define IMXRT_EDMA_TCD3_CITER_ELINK         (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD3_CITER_ELINK_OFFSET)
#define IMXRT_EDMA_TCD3_DLASTSGA            (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD3_DLASTSGA_OFFSET)
#define IMXRT_EDMA_TCD3_CSR                 (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD3_CSR_OFFSET)
#define IMXRT_EDMA_TCD3_BITER_ELINK         (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD3_BITER_ELINK_OFFSET)

#define IMXRT_EDMA_TCD4_SADDR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD4_SADDR_OFFSET)
#define IMXRT_EDMA_TCD4_SOFF                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD4_SOFF_OFFSET)
#define IMXRT_EDMA_TCD4_ATTR                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD4_ATTR_OFFSET)
#define IMXRT_EDMA_TCD4_NBYTES_ML           (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD4_NBYTES_ML_OFFSET)
#define IMXRT_EDMA_TCD4_SLAST               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD4_SLAST_OFFSET)
#define IMXRT_EDMA_TCD4_DADDR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD4_DADDR_OFFSET)
#define IMXRT_EDMA_TCD4_DOFF                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD4_DOFF_OFFSET)
#define IMXRT_EDMA_TCD4_CITER_ELINK         (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD4_CITER_ELINK_OFFSET)
#define IMXRT_EDMA_TCD4_DLASTSGA            (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD4_DLASTSGA_OFFSET)
#define IMXRT_EDMA_TCD4_CSR                 (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD4_CSR_OFFSET)
#define IMXRT_EDMA_TCD4_BITER_ELINK         (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD4_BITER_ELINK_OFFSET)

#define IMXRT_EDMA_TCD5_SADDR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD5_SADDR_OFFSET)
#define IMXRT_EDMA_TCD5_SOFF                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD5_SOFF_OFFSET)
#define IMXRT_EDMA_TCD5_ATTR                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD5_ATTR_OFFSET)
#define IMXRT_EDMA_TCD5_NBYTES_ML           (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD5_NBYTES_ML_OFFSET)
#define IMXRT_EDMA_TCD5_SLAST               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD5_SLAST_OFFSET)
#define IMXRT_EDMA_TCD5_DADDR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD5_DADDR_OFFSET)
#define IMXRT_EDMA_TCD5_DOFF                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD5_DOFF_OFFSET)
#define IMXRT_EDMA_TCD5_CITER_ELINK         (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD5_CITER_ELINK_OFFSET)
#define IMXRT_EDMA_TCD5_DLASTSGA            (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD5_DLASTSGA_OFFSET)
#define IMXRT_EDMA_TCD5_CSR                 (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD5_CSR_OFFSET)
#define IMXRT_EDMA_TCD5_BITER_ELINK         (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD5_BITER_ELINK_OFFSET)

#define IMXRT_EDMA_TCD6_SADDR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD6_SADDR_OFFSET)
#define IMXRT_EDMA_TCD6_SOFF                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD6_SOFF_OFFSET)
#define IMXRT_EDMA_TCD6_ATTR                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD6_ATTR_OFFSET)
#define IMXRT_EDMA_TCD6_NBYTES_ML           (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD6_NBYTES_ML_OFFSET)
#define IMXRT_EDMA_TCD6_SLAST               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD6_SLAST_OFFSET)
#define IMXRT_EDMA_TCD6_DADDR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD6_DADDR_OFFSET)
#define IMXRT_EDMA_TCD6_DOFF                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD6_DOFF_OFFSET)
#define IMXRT_EDMA_TCD6_CITER_ELINK         (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD6_CITER_ELINK_OFFSET)
#define IMXRT_EDMA_TCD6_DLASTSGA            (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD6_DLASTSGA_OFFSET)
#define IMXRT_EDMA_TCD6_CSR                 (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD6_CSR_OFFSET)
#define IMXRT_EDMA_TCD6_BITER_ELINK         (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD6_BITER_ELINK_OFFSET)

#define IMXRT_EDMA_TCD7_SADDR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD7_SADDR_OFFSET)
#define IMXRT_EDMA_TCD7_SOFF                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD7_SOFF_OFFSET)
#define IMXRT_EDMA_TCD7_ATTR                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD7_ATTR_OFFSET)
#define IMXRT_EDMA_TCD7_NBYTES_ML           (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD7_NBYTES_ML_OFFSET)
#define IMXRT_EDMA_TCD7_SLAST               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD7_SLAST_OFFSET)
#define IMXRT_EDMA_TCD7_DADDR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD7_DADDR_OFFSET)
#define IMXRT_EDMA_TCD7_DOFF                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD7_DOFF_OFFSET)
#define IMXRT_EDMA_TCD7_CITER_ELINK         (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD7_CITER_ELINK_OFFSET)
#define IMXRT_EDMA_TCD7_DLASTSGA            (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD7_DLASTSGA_OFFSET)
#define IMXRT_EDMA_TCD7_CSR                 (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD7_CSR_OFFSET)
#define IMXRT_EDMA_TCD7_BITER_ELINK         (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD7_BITER_ELINK_OFFSET)

#define IMXRT_EDMA_TCD8_SADDR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD8_SADDR_OFFSET)
#define IMXRT_EDMA_TCD8_SOFF                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD8_SOFF_OFFSET)
#define IMXRT_EDMA_TCD8_ATTR                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD8_ATTR_OFFSET)
#define IMXRT_EDMA_TCD8_NBYTES_ML           (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD8_NBYTES_ML_OFFSET)
#define IMXRT_EDMA_TCD8_SLAST               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD8_SLAST_OFFSET)
#define IMXRT_EDMA_TCD8_DADDR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD8_DADDR_OFFSET)
#define IMXRT_EDMA_TCD8_DOFF                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD8_DOFF_OFFSET)
#define IMXRT_EDMA_TCD8_CITER_ELINK         (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD8_CITER_ELINK_OFFSET)
#define IMXRT_EDMA_TCD8_DLASTSGA            (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD8_DLASTSGA_OFFSET)
#define IMXRT_EDMA_TCD8_CSR                 (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD8_CSR_OFFSET)
#define IMXRT_EDMA_TCD8_BITER_ELINK         (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD8_BITER_ELINK_OFFSET)

#define IMXRT_EDMA_TCD9_SADDR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD9_SADDR_OFFSET)
#define IMXRT_EDMA_TCD9_SOFF                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD9_SOFF_OFFSET)
#define IMXRT_EDMA_TCD9_ATTR                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD9_ATTR_OFFSET)
#define IMXRT_EDMA_TCD9_NBYTES_ML           (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD9_NBYTES_ML_OFFSET)
#define IMXRT_EDMA_TCD9_SLAST               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD9_SLAST_OFFSET)
#define IMXRT_EDMA_TCD9_DADDR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD9_DADDR_OFFSET)
#define IMXRT_EDMA_TCD9_DOFF                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD9_DOFF_OFFSET)
#define IMXRT_EDMA_TCD9_CITER_ELINK         (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD9_CITER_ELINK_OFFSET)
#define IMXRT_EDMA_TCD9_DLASTSGA            (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD9_DLASTSGA_OFFSET)
#define IMXRT_EDMA_TCD9_CSR                 (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD9_CSR_OFFSET)
#define IMXRT_EDMA_TCD9_BITER_ELINK         (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD9_BITER_ELINK_OFFSET)

#define IMXRT_EDMA_TCD10_SADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD10_SADDR_OFFSET)
#define IMXRT_EDMA_TCD10_SOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD10_SOFF_OFFSET)
#define IMXRT_EDMA_TCD10_ATTR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD10_ATTR_OFFSET)
#define IMXRT_EDMA_TCD10_NBYTES_ML          (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD10_NBYTES_ML_OFFSET)
#define IMXRT_EDMA_TCD10_SLAST              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD10_SLAST_OFFSET)
#define IMXRT_EDMA_TCD10_DADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD10_DADDR_OFFSET)
#define IMXRT_EDMA_TCD10_DOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD10_DOFF_OFFSET)
#define IMXRT_EDMA_TCD10_CITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD10_CITER_ELINK_OFFSET)
#define IMXRT_EDMA_TCD10_DLASTSGA           (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD10_DLASTSGA_OFFSET)
#define IMXRT_EDMA_TCD10_CSR                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD10_CSR_OFFSET)
#define IMXRT_EDMA_TCD10_BITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD10_BITER_ELINK_OFFSET)

#define IMXRT_EDMA_TCD11_SADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD11_SADDR_OFFSET)
#define IMXRT_EDMA_TCD11_SOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD11_SOFF_OFFSET)
#define IMXRT_EDMA_TCD11_ATTR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD11_ATTR_OFFSET)
#define IMXRT_EDMA_TCD11_NBYTES_ML          (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD11_NBYTES_ML_OFFSET)
#define IMXRT_EDMA_TCD11_SLAST              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD11_SLAST_OFFSET)
#define IMXRT_EDMA_TCD11_DADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD11_DADDR_OFFSET)
#define IMXRT_EDMA_TCD11_DOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD11_DOFF_OFFSET)
#define IMXRT_EDMA_TCD11_CITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD11_CITER_ELINK_OFFSET)
#define IMXRT_EDMA_TCD11_DLASTSGA           (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD11_DLASTSGA_OFFSET)
#define IMXRT_EDMA_TCD11_CSR                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD11_CSR_OFFSET)
#define IMXRT_EDMA_TCD11_BITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD11_BITER_ELINK_OFFSET)

#define IMXRT_EDMA_TCD12_SADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD12_SADDR_OFFSET)
#define IMXRT_EDMA_TCD12_SOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD12_SOFF_OFFSET)
#define IMXRT_EDMA_TCD12_ATTR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD12_ATTR_OFFSET)
#define IMXRT_EDMA_TCD12_NBYTES_ML          (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD12_NBYTES_ML_OFFSET)
#define IMXRT_EDMA_TCD12_SLAST              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD12_SLAST_OFFSET)
#define IMXRT_EDMA_TCD12_DADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD12_DADDR_OFFSET)
#define IMXRT_EDMA_TCD12_DOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD12_DOFF_OFFSET)
#define IMXRT_EDMA_TCD12_CITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD12_CITER_ELINK_OFFSET)
#define IMXRT_EDMA_TCD12_DLASTSGA           (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD12_DLASTSGA_OFFSET)
#define IMXRT_EDMA_TCD12_CSR                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD12_CSR_OFFSET)
#define IMXRT_EDMA_TCD12_BITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD12_BITER_ELINK_OFFSET)

#define IMXRT_EDMA_TCD13_SADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD13_SADDR_OFFSET)
#define IMXRT_EDMA_TCD13_SOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD13_SOFF_OFFSET)
#define IMXRT_EDMA_TCD13_ATTR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD13_ATTR_OFFSET)
#define IMXRT_EDMA_TCD13_NBYTES_ML          (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD13_NBYTES_ML_OFFSET)
#define IMXRT_EDMA_TCD13_SLAST              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD13_SLAST_OFFSET)
#define IMXRT_EDMA_TCD13_DADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD13_DADDR_OFFSET)
#define IMXRT_EDMA_TCD13_DOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD13_DOFF_OFFSET)
#define IMXRT_EDMA_TCD13_CITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD13_CITER_ELINK_OFFSET)
#define IMXRT_EDMA_TCD13_DLASTSGA           (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD13_DLASTSGA_OFFSET)
#define IMXRT_EDMA_TCD13_CSR                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD13_CSR_OFFSET)
#define IMXRT_EDMA_TCD13_BITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD13_BITER_ELINK_OFFSET)

#define IMXRT_EDMA_TCD14_SADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD14_SADDR_OFFSET)
#define IMXRT_EDMA_TCD14_SOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD14_SOFF_OFFSET)
#define IMXRT_EDMA_TCD14_ATTR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD14_ATTR_OFFSET)
#define IMXRT_EDMA_TCD14_NBYTES_ML          (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD14_NBYTES_ML_OFFSET)
#define IMXRT_EDMA_TCD14_SLAST              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD14_SLAST_OFFSET)
#define IMXRT_EDMA_TCD14_DADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD14_DADDR_OFFSET)
#define IMXRT_EDMA_TCD14_DOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD14_DOFF_OFFSET)
#define IMXRT_EDMA_TCD14_CITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD14_CITER_ELINK_OFFSET)
#define IMXRT_EDMA_TCD14_DLASTSGA           (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD14_DLASTSGA_OFFSET)
#define IMXRT_EDMA_TCD14_CSR                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD14_CSR_OFFSET)
#define IMXRT_EDMA_TCD14_BITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD14_BITER_ELINK_OFFSET)

#define IMXRT_EDMA_TCD15_SADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD15_SADDR_OFFSET)
#define IMXRT_EDMA_TCD15_SOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD15_SOFF_OFFSET)
#define IMXRT_EDMA_TCD15_ATTR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD15_ATTR_OFFSET)
#define IMXRT_EDMA_TCD15_NBYTES_ML          (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD15_NBYTES_ML_OFFSET)
#define IMXRT_EDMA_TCD15_SLAST              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD15_SLAST_OFFSET)
#define IMXRT_EDMA_TCD15_DADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD15_DADDR_OFFSET)
#define IMXRT_EDMA_TCD15_DOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD15_DOFF_OFFSET)
#define IMXRT_EDMA_TCD15_CITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD15_CITER_ELINK_OFFSET)
#define IMXRT_EDMA_TCD15_DLASTSGA           (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD15_DLASTSGA_OFFSET)
#define IMXRT_EDMA_TCD15_CSR                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD15_CSR_OFFSET)
#define IMXRT_EDMA_TCD15_BITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD15_BITER_ELINK_OFFSET)

#define IMXRT_EDMA_TCD16_SADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD16_SADDR_OFFSET)
#define IMXRT_EDMA_TCD16_SOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD16_SOFF_OFFSET)
#define IMXRT_EDMA_TCD16_ATTR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD16_ATTR_OFFSET)
#define IMXRT_EDMA_TCD16_NBYTES_ML          (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD16_NBYTES_ML_OFFSET)
#define IMXRT_EDMA_TCD16_SLAST              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD16_SLAST_OFFSET)
#define IMXRT_EDMA_TCD16_DADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD16_DADDR_OFFSET)
#define IMXRT_EDMA_TCD16_DOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD16_DOFF_OFFSET)
#define IMXRT_EDMA_TCD16_CITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD16_CITER_ELINK_OFFSET)
#define IMXRT_EDMA_TCD16_DLASTSGA           (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD16_DLASTSGA_OFFSET)
#define IMXRT_EDMA_TCD16_CSR                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD16_CSR_OFFSET)
#define IMXRT_EDMA_TCD16_BITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD16_BITER_ELINK_OFFSET)

#define IMXRT_EDMA_TCD17_SADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD17_SADDR_OFFSET)
#define IMXRT_EDMA_TCD17_SOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD17_SOFF_OFFSET)
#define IMXRT_EDMA_TCD17_ATTR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD17_ATTR_OFFSET)
#define IMXRT_EDMA_TCD17_NBYTES_ML          (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD17_NBYTES_ML_OFFSET)
#define IMXRT_EDMA_TCD17_SLAST              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD17_SLAST_OFFSET)
#define IMXRT_EDMA_TCD17_DADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD17_DADDR_OFFSET)
#define IMXRT_EDMA_TCD17_DOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD17_DOFF_OFFSET)
#define IMXRT_EDMA_TCD17_CITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD17_CITER_ELINK_OFFSET)
#define IMXRT_EDMA_TCD17_DLASTSGA           (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD17_DLASTSGA_OFFSET)
#define IMXRT_EDMA_TCD17_CSR                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD17_CSR_OFFSET)
#define IMXRT_EDMA_TCD17_BITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD17_BITER_ELINK_OFFSET)

#define IMXRT_EDMA_TCD18_SADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD18_SADDR_OFFSET)
#define IMXRT_EDMA_TCD18_SOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD18_SOFF_OFFSET)
#define IMXRT_EDMA_TCD18_ATTR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD18_ATTR_OFFSET)
#define IMXRT_EDMA_TCD18_NBYTES_ML          (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD18_NBYTES_ML_OFFSET)
#define IMXRT_EDMA_TCD18_SLAST              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD18_SLAST_OFFSET)
#define IMXRT_EDMA_TCD18_DADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD18_DADDR_OFFSET)
#define IMXRT_EDMA_TCD18_DOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD18_DOFF_OFFSET)
#define IMXRT_EDMA_TCD18_CITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD18_CITER_ELINK_OFFSET)
#define IMXRT_EDMA_TCD18_DLASTSGA           (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD18_DLASTSGA_OFFSET)
#define IMXRT_EDMA_TCD18_CSR                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD18_CSR_OFFSET)
#define IMXRT_EDMA_TCD18_BITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD18_BITER_ELINK_OFFSET)

#define IMXRT_EDMA_TCD19_SADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD19_SADDR_OFFSET)
#define IMXRT_EDMA_TCD19_SOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD19_SOFF_OFFSET)
#define IMXRT_EDMA_TCD19_ATTR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD19_ATTR_OFFSET)
#define IMXRT_EDMA_TCD19_NBYTES_ML          (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD19_NBYTES_ML_OFFSET)
#define IMXRT_EDMA_TCD19_SLAST              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD19_SLAST_OFFSET)
#define IMXRT_EDMA_TCD19_DADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD19_DADDR_OFFSET)
#define IMXRT_EDMA_TCD19_DOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD19_DOFF_OFFSET)
#define IMXRT_EDMA_TCD19_CITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD19_CITER_ELINK_OFFSET)
#define IMXRT_EDMA_TCD19_DLASTSGA           (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD19_DLASTSGA_OFFSET)
#define IMXRT_EDMA_TCD19_CSR                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD19_CSR_OFFSET)
#define IMXRT_EDMA_TCD19_BITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD19_BITER_ELINK_OFFSET)

#define IMXRT_EDMA_TCD20_SADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD20_SADDR_OFFSET)
#define IMXRT_EDMA_TCD20_SOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD20_SOFF_OFFSET)
#define IMXRT_EDMA_TCD20_ATTR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD20_ATTR_OFFSET)
#define IMXRT_EDMA_TCD20_NBYTES_ML          (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD20_NBYTES_ML_OFFSET)
#define IMXRT_EDMA_TCD20_SLAST              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD20_SLAST_OFFSET)
#define IMXRT_EDMA_TCD20_DADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD20_DADDR_OFFSET)
#define IMXRT_EDMA_TCD20_DOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD20_DOFF_OFFSET)
#define IMXRT_EDMA_TCD20_CITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD20_CITER_ELINK_OFFSET)
#define IMXRT_EDMA_TCD20_DLASTSGA           (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD20_DLASTSGA_OFFSET)
#define IMXRT_EDMA_TCD20_CSR                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD20_CSR_OFFSET)
#define IMXRT_EDMA_TCD20_BITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD20_BITER_ELINK_OFFSET)

#define IMXRT_EDMA_TCD21_SADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD21_SADDR_OFFSET)
#define IMXRT_EDMA_TCD21_SOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD21_SOFF_OFFSET)
#define IMXRT_EDMA_TCD21_ATTR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD21_ATTR_OFFSET)
#define IMXRT_EDMA_TCD21_NBYTES_ML          (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD21_NBYTES_ML_OFFSET)
#define IMXRT_EDMA_TCD21_SLAST              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD21_SLAST_OFFSET)
#define IMXRT_EDMA_TCD21_DADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD21_DADDR_OFFSET)
#define IMXRT_EDMA_TCD21_DOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD21_DOFF_OFFSET)
#define IMXRT_EDMA_TCD21_CITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD21_CITER_ELINK_OFFSET)
#define IMXRT_EDMA_TCD21_DLASTSGA           (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD21_DLASTSGA_OFFSET)
#define IMXRT_EDMA_TCD21_CSR                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD21_CSR_OFFSET)
#define IMXRT_EDMA_TCD21_BITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD21_BITER_ELINK_OFFSET)

#define IMXRT_EDMA_TCD22_SADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD22_SADDR_OFFSET)
#define IMXRT_EDMA_TCD22_SOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD22_SOFF_OFFSET)
#define IMXRT_EDMA_TCD22_ATTR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD22_ATTR_OFFSET)
#define IMXRT_EDMA_TCD22_NBYTES_ML          (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD22_NBYTES_ML_OFFSET)
#define IMXRT_EDMA_TCD22_SLAST              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD22_SLAST_OFFSET)
#define IMXRT_EDMA_TCD22_DADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD22_DADDR_OFFSET)
#define IMXRT_EDMA_TCD22_DOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD22_DOFF_OFFSET)
#define IMXRT_EDMA_TCD22_CITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD22_CITER_ELINK_OFFSET)
#define IMXRT_EDMA_TCD22_DLASTSGA           (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD22_DLASTSGA_OFFSET)
#define IMXRT_EDMA_TCD22_CSR                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD22_CSR_OFFSET)
#define IMXRT_EDMA_TCD22_BITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD22_BITER_ELINK_OFFSET)

#define IMXRT_EDMA_TCD23_SADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD23_SADDR_OFFSET)
#define IMXRT_EDMA_TCD23_SOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD23_SOFF_OFFSET)
#define IMXRT_EDMA_TCD23_ATTR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD23_ATTR_OFFSET)
#define IMXRT_EDMA_TCD23_NBYTES_ML          (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD23_NBYTES_ML_OFFSET)
#define IMXRT_EDMA_TCD23_SLAST              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD23_SLAST_OFFSET)
#define IMXRT_EDMA_TCD23_DADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD23_DADDR_OFFSET)
#define IMXRT_EDMA_TCD23_DOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD23_DOFF_OFFSET)
#define IMXRT_EDMA_TCD23_CITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD23_CITER_ELINK_OFFSET)
#define IMXRT_EDMA_TCD23_DLASTSGA           (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD23_DLASTSGA_OFFSET)
#define IMXRT_EDMA_TCD23_CSR                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD23_CSR_OFFSET)
#define IMXRT_EDMA_TCD23_BITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD23_BITER_ELINK_OFFSET)

#define IMXRT_EDMA_TCD24_SADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD24_SADDR_OFFSET)
#define IMXRT_EDMA_TCD24_SOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD24_SOFF_OFFSET)
#define IMXRT_EDMA_TCD24_ATTR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD24_ATTR_OFFSET)
#define IMXRT_EDMA_TCD24_NBYTES_ML          (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD24_NBYTES_ML_OFFSET)
#define IMXRT_EDMA_TCD24_SLAST              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD24_SLAST_OFFSET)
#define IMXRT_EDMA_TCD24_DADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD24_DADDR_OFFSET)
#define IMXRT_EDMA_TCD24_DOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD24_DOFF_OFFSET)
#define IMXRT_EDMA_TCD24_CITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD24_CITER_ELINK_OFFSET)
#define IMXRT_EDMA_TCD24_DLASTSGA           (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD24_DLASTSGA_OFFSET)
#define IMXRT_EDMA_TCD24_CSR                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD24_CSR_OFFSET)
#define IMXRT_EDMA_TCD24_BITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD24_BITER_ELINK_OFFSET)

#define IMXRT_EDMA_TCD25_SADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD25_SADDR_OFFSET)
#define IMXRT_EDMA_TCD25_SOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD25_SOFF_OFFSET)
#define IMXRT_EDMA_TCD25_ATTR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD25_ATTR_OFFSET)
#define IMXRT_EDMA_TCD25_NBYTES_ML          (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD25_NBYTES_ML_OFFSET)
#define IMXRT_EDMA_TCD25_SLAST              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD25_SLAST_OFFSET)
#define IMXRT_EDMA_TCD25_DADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD25_DADDR_OFFSET)
#define IMXRT_EDMA_TCD25_DOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD25_DOFF_OFFSET)
#define IMXRT_EDMA_TCD25_CITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD25_CITER_ELINK_OFFSET)
#define IMXRT_EDMA_TCD25_DLASTSGA           (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD25_DLASTSGA_OFFSET)
#define IMXRT_EDMA_TCD25_CSR                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD25_CSR_OFFSET)
#define IMXRT_EDMA_TCD25_BITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD25_BITER_ELINK_OFFSET)

#define IMXRT_EDMA_TCD26_SADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD26_SADDR_OFFSET)
#define IMXRT_EDMA_TCD26_SOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD26_SOFF_OFFSET)
#define IMXRT_EDMA_TCD26_ATTR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD26_ATTR_OFFSET)
#define IMXRT_EDMA_TCD26_NBYTES_ML          (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD26_NBYTES_ML_OFFSET)
#define IMXRT_EDMA_TCD26_SLAST              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD26_SLAST_OFFSET)
#define IMXRT_EDMA_TCD26_DADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD26_DADDR_OFFSET)
#define IMXRT_EDMA_TCD26_DOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD26_DOFF_OFFSET)
#define IMXRT_EDMA_TCD26_CITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD26_CITER_ELINK_OFFSET)
#define IMXRT_EDMA_TCD26_DLASTSGA           (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD26_DLASTSGA_OFFSET)
#define IMXRT_EDMA_TCD26_CSR                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD26_CSR_OFFSET)
#define IMXRT_EDMA_TCD26_BITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD26_BITER_ELINK_OFFSET)

#define IMXRT_EDMA_TCD27_SADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD27_SADDR_OFFSET)
#define IMXRT_EDMA_TCD27_SOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD27_SOFF_OFFSET)
#define IMXRT_EDMA_TCD27_ATTR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD27_ATTR_OFFSET)
#define IMXRT_EDMA_TCD27_NBYTES_ML          (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD27_NBYTES_ML_OFFSET)
#define IMXRT_EDMA_TCD27_SLAST              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD27_SLAST_OFFSET)
#define IMXRT_EDMA_TCD27_DADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD27_DADDR_OFFSET)
#define IMXRT_EDMA_TCD27_DOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD27_DOFF_OFFSET)
#define IMXRT_EDMA_TCD27_CITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD27_CITER_ELINK_OFFSET)
#define IMXRT_EDMA_TCD27_DLASTSGA           (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD27_DLASTSGA_OFFSET)
#define IMXRT_EDMA_TCD27_CSR                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD27_CSR_OFFSET)
#define IMXRT_EDMA_TCD27_BITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD27_BITER_ELINK_OFFSET)

#define IMXRT_EDMA_TCD28_SADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD28_SADDR_OFFSET)
#define IMXRT_EDMA_TCD28_SOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD28_SOFF_OFFSET)
#define IMXRT_EDMA_TCD28_ATTR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD28_ATTR_OFFSET)
#define IMXRT_EDMA_TCD28_NBYTES_ML          (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD28_NBYTES_ML_OFFSET)
#define IMXRT_EDMA_TCD28_SLAST              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD28_SLAST_OFFSET)
#define IMXRT_EDMA_TCD28_DADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD28_DADDR_OFFSET)
#define IMXRT_EDMA_TCD28_DOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD28_DOFF_OFFSET)
#define IMXRT_EDMA_TCD28_CITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD28_CITER_ELINK_OFFSET)
#define IMXRT_EDMA_TCD28_DLASTSGA           (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD28_DLASTSGA_OFFSET)
#define IMXRT_EDMA_TCD28_CSR                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD28_CSR_OFFSET)
#define IMXRT_EDMA_TCD28_BITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD28_BITER_ELINK_OFFSET)

#define IMXRT_EDMA_TCD29_SADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD29_SADDR_OFFSET)
#define IMXRT_EDMA_TCD29_SOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD29_SOFF_OFFSET)
#define IMXRT_EDMA_TCD29_ATTR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD29_ATTR_OFFSET)
#define IMXRT_EDMA_TCD29_NBYTES_ML          (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD29_NBYTES_ML_OFFSET)
#define IMXRT_EDMA_TCD29_SLAST              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD29_SLAST_OFFSET)
#define IMXRT_EDMA_TCD29_DADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD29_DADDR_OFFSET)
#define IMXRT_EDMA_TCD29_DOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD29_DOFF_OFFSET)
#define IMXRT_EDMA_TCD29_CITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD29_CITER_ELINK_OFFSET)
#define IMXRT_EDMA_TCD29_DLASTSGA           (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD29_DLASTSGA_OFFSET)
#define IMXRT_EDMA_TCD29_CSR                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD29_CSR_OFFSET)
#define IMXRT_EDMA_TCD29_BITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD29_BITER_ELINK_OFFSET)

#define IMXRT_EDMA_TCD30_SADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD30_SADDR_OFFSET)
#define IMXRT_EDMA_TCD30_SOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD30_SOFF_OFFSET)
#define IMXRT_EDMA_TCD30_ATTR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD30_ATTR_OFFSET)
#define IMXRT_EDMA_TCD30_NBYTES_ML          (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD30_NBYTES_ML_OFFSET)
#define IMXRT_EDMA_TCD30_SLAST              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD30_SLAST_OFFSET)
#define IMXRT_EDMA_TCD30_DADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD30_DADDR_OFFSET)
#define IMXRT_EDMA_TCD30_DOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD30_DOFF_OFFSET)
#define IMXRT_EDMA_TCD30_CITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD30_CITER_ELINK_OFFSET)
#define IMXRT_EDMA_TCD30_DLASTSGA           (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD30_DLASTSGA_OFFSET)
#define IMXRT_EDMA_TCD30_CSR                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD30_CSR_OFFSET)
#define IMXRT_EDMA_TCD30_BITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD30_BITER_ELINK_OFFSET)

#define IMXRT_EDMA_TCD31_SADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD31_SADDR_OFFSET)
#define IMXRT_EDMA_TCD31_SOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD31_SOFF_OFFSET)
#define IMXRT_EDMA_TCD31_ATTR               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD31_ATTR_OFFSET)
#define IMXRT_EDMA_TCD31_NBYTES_ML          (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD31_NBYTES_ML_OFFSET)
#define IMXRT_EDMA_TCD31_SLAST              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD31_SLAST_OFFSET)
#define IMXRT_EDMA_TCD31_DADDR              (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD31_DADDR_OFFSET)
#define IMXRT_EDMA_TCD31_DOFF               (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD31_DOFF_OFFSET)
#define IMXRT_EDMA_TCD31_CITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD31_CITER_ELINK_OFFSET)
#define IMXRT_EDMA_TCD31_DLASTSGA           (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD31_DLASTSGA_OFFSET)
#define IMXRT_EDMA_TCD31_CSR                (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD31_CSR_OFFSET)
#define IMXRT_EDMA_TCD31_BITER_ELINK        (IMXRT_EDMA_BASE + IMXRT_EDMA_TCD31_BITER_ELINK_OFFSET)

/* eDMA Bit-Field Definitions ***********************************************************************/

/* Control */
                                                      /* Bit 0:  Reserved */
#define EDMA_CR_EDBG                        (1 << 1)  /* Bit 1:  Enable Debug */
#define EDMA_CR_ERCA                        (1 << 2)  /* Bit 2:  Enable Round Robin Channel Arbitration */
#define EDMA_CR_ERGA                        (1 << 3)  /* Bit 3:  Enable Round Robin Group Arbitration */
#define EDMA_CR_HOE                         (1 << 4)  /* Bit 4:  Halt On Error */
#define EDMA_CR_HALT                        (1 << 5)  /* Bit 5:  Halt DMA Operations */
#define EDMA_CR_CLM                         (1 << 6)  /* Bit 6:  Continuous Link Mode */
#define EDMA_CR_EMLM                        (1 << 7)  /* Bit 7:  Enable Minor Loop Mapping */
#define EDMA_CR_GRP0PRI                     (1 << 8)  /* Bit 8:  Channel Group 0 Priority */
                                                      /* Bit 9:  Reserved */
#define EDMA_CR_GRP1PRI                     (1 << 10) /* Bit 10: Channel Group 1 Priority */
                                                      /* Bits 11-15: Reserved */
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
#define EDMA_ES_ERRCHN_SHIFT                (8)       /* Bits 8-12: Error Channel Number or
                                                       *         Canceled Channel Number */
#define EDMA_ES_ERRCHN_MASK                 (0x1f << EDMA_ES_ERRCHN_SHIFT)
                                                      /* Bit 13: Reserved */
#define EDMA_ES_CPE                         (1 << 14) /* Bit 14: Channel Priority Error */
#define EDMA_ES_GPE                         (1 << 15) /* Bit 15: Group Priority Error */
#define EDMA_ES_ECX                         (1 << 16) /* Bit 16: Transfer Canceled */
                                                      /* Bits 17-30: Reserved */
#define EDMA_ES_VLD                         (1 << 31) /* Bit 31: Logical OR of all ERR status bits */

/* Enable Request */

#define EDMA_ERQ(n)                         ((uint32_t)1 << (n)) /* Bit n:  Enable DMA request n */

/* Enable Error Interrupt */

#define EDMA_EEI(n)                         ((uint32_t)1 << (n)) /* Bit n:  Enable error interrupt n */

/* Clear Enable Error Interrupt */

#define EDMA_CEEI_SHIFT                     (0)       /* Bits 0-4: Clear Enable Error Interrupt */
#define EDMA_CEEI_MASK                      (0x1f << EDMA_CEEI_SHIFT)
#  define EDMA_CEEI(n)                      ((uint32_t)(n) << EDMA_CEEI_SHIFT)
                                                      /* Bit 5:  Reserved */
#define EDMA_CEEI_CAEE                      (1 << 6)  /* Bit 6:  Clear All Enable Error Interrupts */
#define EDMA_CEEI_NOP                       (1 << 7)  /* Bit 7:  No Op enable */

/* Set Enable Error Interrupt */

#define EDMA_SEEI_SHIFT                     (0)       /* Bits 0-4: Set Enable Error Interrupt */
#define EDMA_SEEI_MASK                      (0x1f << EDMA_SEEI_SHIFT)
#  define EDMA_SEEI(n)                      ((uint32_t)(n) << EDMA_SEEI_SHIFT)
                                                      /* Bit 5:  Reserved */
#define EDMA_SEEI_SAEE                      (1 << 6)  /* Bit 6:  Set All Enable Error Interrupts */
#define EDMA_SEEI_NOP                       (1 << 7)  /* Bit 7:  No Op enable */

/* Clear Enable Request */

#define EDMA_CERQ_SHIFT                     (0)       /* Bits 0-4: Clear Enable Request */
#define EDMA_CERQ_MASK                      (0x1f << EDMA_CERQ_SHIFT)
#  define EDMA_CERQ(n)                      ((uint32_t)(n) << EDMA_CERQ_SHIFT)
                                                      /* Bit 5:  Reserved */
#define EDMA_CERQ_CAER                      (1 << 6)  /* Bit 6:  Clear All Enable Requests */
#define EDMA_CERQ_NOP                       (1 << 7)  /* Bit 7:  No Op enable */

/* Set Enable Request */

#define EDMA_SERQ_SHIFT                     (0)       /* Bits 0-4: Set Enable Request */
#define EDMA_SERQ_MASK                      (0x1f << EDMA_SERQ_SHIFT)
#  define EDMA_SERQ(n)                      ((uint32_t)(n) << EDMA_SERQ_SHIFT)
                                                      /* Bit 5:  Reserved */
#define EDMA_SERQ_SAER                      (1 << 6)  /* Bit 6:  Set All Enable Requests */
#define EDMA_SERQ_NOP                       (1 << 7)  /* Bit 7:  No Op enable */

/* Clear DONE Status Bit */

#define EDMA_CDNE_SHIFT                     (0)       /* Bits 0-4: Clear DONE Bit */
#define EDMA_CDNE_MASK                      (0x1f << EDMA_CDNE_SHIFT)
#  define EDMA_CDNE(n)                      ((uint32_t)(n) << EDMA_CDNE_SHIFT)
                                                      /* Bit 5:  Reserved */
#define EDMA_CDNE_CADN                      (1 << 6)  /* Bit 6:  Clears All DONE Bits */
#define EDMA_CDNE_NOP                       (1 << 7)  /* Bit 7:  No Op enable */

/* Set START Bit */

#define EDMA_SSRT_SHIFT                     (0)       /* Bits 0-4: Set START Bit */
#define EDMA_SSRT_MASK                      (0x1f << EDMA_SSRT_SHIFT)
#  define EDMA_SSRT(n)                      ((uint32_t)(n) << EDMA_SSRT_SHIFT)
                                                      /* Bit 5:  Reserved */
#define EDMA_SSRT_SAST                      (1 << 6)  /* Bit 6:  Set All START Bits (activates all channels) */
#define EDMA_SSRT_NOP                       (1 << 7)  /* Bit 7:  No Op enable */

/* Clear Error */

#define EDMA_CERR_SHIFT                     (0)       /* Bits 0-4: Clear Error Indicator */
#define EDMA_CERR_MASK                      (0x1f << EDMA_CERR_SHIFT)
#  define EDMA_CERR(n)                      ((uint32_t)(n) << EDMA_CERR_SHIFT)
                                                      /* Bit 5:  Reserved */
#define EDMA_CERR_CAEI                      (1 << 6)  /* Bit 6:  Clear All Error Indicators */
#define EDMA_CERR_NOP                       (1 << 7)  /* Bit 7:  No Op enable */

/* Clear Interrupt Request */
#define EDMA_CINT_

#define EDMA_CINT_SHIFT                     (0)       /* Bits 0-4: Clear Interrupt Request */
#define EDMA_CINT_MASK                      (0x1f << EDMA_CINT_SHIFT)
#  define EDMA_CINT(n)                      ((uint32_t)(n) << EDMA_CINT_SHIFT)
                                                      /* Bit 5:  Reserved */
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
#define EDMA_DCHPRI_GRPPRI_SHIFT            (4)       /* Bits 4-5: Channel n Current Group Priority */
#define EDMA_DCHPRI_GRPPRI_MASK             (3 << EDMA_DCHPRI_GRPPRI_SHIFT)
#  define EDMA_DCHPRI_GRPPRI(n)             ((uint32_t)(n) << EDMA_DCHPRI_GRPPRI_SHIFT)
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
/* TCD Signed Destination Address Offset (32-bit signed address offset) */

/* TCD Current Minor Loop Link, Major Loop Count (Channel linking disabled) */

#define EDMA_TCD_CITER_CITER_SHIFT          (0)       /* Bit 0-14: Starting Major Iteration Count */
#define EDMA_TCD_CITER_CITER_MASK           (0x7fff << EDMA_TCD_CITER_CITER_SHIFT)
#  define EDMA_TCD_CITER_CITER(n)           ((uint32_t)(n) << EDMA_TCD_CITER_CITER_SHIFT)
#define EDMA_TCD_CITER_ELINK                (1 << 15) /* Bit 15: Enable channel-to-channel linking
                                                       * on minor-loop complete */

/* TCD Current Minor Loop Link, Major Loop Count (Channel linking enabled) */

#define EDMA_TCD_CITER_ELINK_CITER_SHIFT    (0)       /* Bit 0-8: Current major iteration count */
#define EDMA_TCD_CITER_ELINK_CITER_MASK     (0x1ff << EDMA_TCD_CITER_ELINK_CITER_SHIFT)
#  define EDMA_TCD_CITER_ELINK_CITER(n)     ((uint32_t)(n) << EDMA_TCD_CITER_ELINK_CITER_SHIFT)
#define EDMA_TCD_CITER_ELINK_LINKCH_SHIFT   (9)       /* Bit 9-13: Minor Loop Link Channel Number */
#define EDMA_TCD_CITER_ELINK_LINKCH_MASK    (31 << EDMA_TCD_CITER_ELINK_LINKCH_SHIFT)
#  define EDMA_TCD_CITER_ELINK_LINKCH(n)    ((uint32_t)(n) << EDMA_TCD_CITER_ELINK_LINKCH_SHIFT)
                                                      /* Bit 14: Reserved */
#define EDMA_TCD_CITER_ELINK_ELINK          (1 << 15) /* Bit 15: Enable channel-to-channel linking
                                                       * on minor-loop complete */

/* TCD Last Destination Address Adjustment/Scatter Gather Address (32-bit address) */

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
#define EDMA_TCD_CSR_MAJORLINKCH_SHIFT      (8)       /* Bits 8-12: Major Loop Link Channel Number */
#define EDMA_TCD_CSR_MAJORLINKCH_MASK       (31 << EDMA_TCD_CSR_MAJORLINKCH_SHIFT)
#  define EDMA_TCD_CSR_MAJORLINKCH(n)       ((uint32_t)(n) << EDMA_TCD_CSR_MAJORLINKCH_SHIFT)
                                                      /* Bit 13: Reserved */
#define EDMA_TCD_CSR_BWC_SHIFT              (14)      /* Bits 14-15: Bandwidth Control */
#define EDMA_TCD_CSR_BWC_MASK               (3 << EDMA_TCD_CSR_BWC_SHIFT)
#  define EDMA_TCD_CSR_BWC_NONE             (0 << EDMA_TCD_CSR_BWC_SHIFT) /* No eDMA engine stalls */
#  define EDMA_TCD_CSR_BWC_4CYCLES          (2 << EDMA_TCD_CSR_BWC_SHIFT) /* eDMA engine stalls for 4
                                                                           * cycles after each R/W */
#  define EDMA_TCD_CSR_BWC_8CYCLES          (3 << EDMA_TCD_CSR_BWC_SHIFT) /* eDMA engine stalls for 8
                                                                           * cycles after each R/W */

/* TCD Beginning Minor Loop Link, Major Loop Count (Channel linking disabled) */

#define EDMA_TCD_BITER_BITER_SHIFT          (0)       /* Bit 0-14: Starting Major Iteration Count */
#define EDMA_TCD_BITER_BITER_MASK           (0x7fff << EDMA_TCD_BITER_BITER_SHIFT)
#  define EDMA_TCD_BITER_BITER(n)           ((uint32_t)(n) << EDMA_TCD_BITER_BITER_SHIFT)
#define EDMA_TCD_BITER_ELINK                (1 << 15) /* Bit 15: Enable channel-to-channel linking
                                                       * on minor-loop complete */

/* TCD Beginning Minor Loop Link, Major Loop Count (Channel linking enabled) */

#define EDMA_TCD_BITER_ELINK_BITER_SHIFT    (0)       /* Bit 0-8: Current major iteration count */
#define EDMA_TCD_BITER_ELINK_BITER_MASK     (0x1ff << EDMA_TCD_BITER_ELINK_BITER_SHIFT)
#  define EDMA_TCD_BITER_ELINK_BITER(n)     ((uint32_t)(n) << EDMA_TCD_BITER_ELINK_BITER_SHIFT)
#define EDMA_TCD_BITER_ELINK_LINKCH_SHIFT   (9)       /* Bit 9-13: Link Channel Number */
#define EDMA_TCD_BITER_ELINK_LINKCH_MASK    (31 << EDMA_TCD_BITER_ELINK_LINKCH_SHIFT)
#  define EDMA_TCD_BITER_ELINK_LINKCH(n)    ((uint32_t)(n) << EDMA_TCD_BITER_ELINK_LINKCH_SHIFT)
                                                      /* Bit 14: Reserved */
#define EDMA_TCD_BITER_ELINK_ELINK          (1 << 15) /* Bit 15: Enable channel-to-channel linking
                                                       * on minor-loop complete */

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/* In-memory representation of the 32-byte Transfer Control Descriptor (TCD) */

struct imxrt_edmatcd_s
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

#endif /* __ARCH_ARM_SRC_IMXRT_CHIP_IMXRT_EDMA_H */
