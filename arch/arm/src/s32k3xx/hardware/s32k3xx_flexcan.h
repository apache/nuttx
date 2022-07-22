/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_flexcan.h
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

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_CAN_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_CAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/s32k3xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* FlexCAN Register Offsets *************************************************/

#define S32K3XX_CAN_MCR_OFFSET               (0x0000) /* Module Configuration Register (MCR) */
#define S32K3XX_CAN_CTRL1_OFFSET             (0x0004) /* Control 1 Register (CTRL1) */
#define S32K3XX_CAN_TIMER_OFFSET             (0x0008) /* Free Running Timer (TIMER) */
#define S32K3XX_CAN_RXMGMASK_OFFSET          (0x0010) /* Rx Mailboxes Global Mask Register (RXMGMASK) */
#define S32K3XX_CAN_RX14MASK_OFFSET          (0x0014) /* Rx 14 Mask Register (RX14MASK) */
#define S32K3XX_CAN_RX15MASK_OFFSET          (0x0018) /* Rx 15 Mask Register (RX15MASK) */
#define S32K3XX_CAN_ECR_OFFSET               (0x001c) /* Error Counter (ECR) */
#define S32K3XX_CAN_ESR1_OFFSET              (0x0020) /* Error and Status 1 Register (ESR1) */
#define S32K3XX_CAN_IMASK2_OFFSET            (0x0024) /* Interrupt Masks 2 Register (IMASK2) */
#define S32K3XX_CAN_IMASK1_OFFSET            (0x0028) /* Interrupt Masks 1 Register (IMASK1) */
#define S32K3XX_CAN_IFLAG2_OFFSET            (0x002c) /* Interrupt Flags 2 Register (IFLAG2) */
#define S32K3XX_CAN_IFLAG1_OFFSET            (0x0030) /* Interrupt Flags 1 Register (IFLAG1) */
#define S32K3XX_CAN_CTRL2_OFFSET             (0x0034) /* Control 2 Register (CTRL2) */
#define S32K3XX_CAN_ESR2_OFFSET              (0x0038) /* Error and Status 2 Register (ESR2) */
#define S32K3XX_CAN_CRCR_OFFSET              (0x0044) /* CRC Register (CRCR) */
#define S32K3XX_CAN_RXFGMASK_OFFSET          (0x0048) /* Legacy Rx FIFO Global Mask Register (RXFGMASK) */
#define S32K3XX_CAN_RXFIR_OFFSET             (0x004c) /* Legacy Rx FIFO Information Register (RXFIR) */
#define S32K3XX_CAN_CBT_OFFSET               (0x0050) /* CAN Bit Timing register (CBT) */
#define S32K3XX_CAN_IMASK3_OFFSET            (0x006c) /* Interrupt Masks 3 Register (IMASK3) */
#define S32K3XX_CAN_IFLAG3_OFFSET            (0x0074) /* Interrupt Flags 3 Register (IFLAG3) */

#define S32K3XX_CAN_MB_OFFSET                (0x0080) /* CAN MB RAM */

#define S32K3XX_CAN_RXIMR_OFFSET(n)          (0x0880 + ((n) << 2))
#  define S32K3XX_CAN_RXIMR0_OFFSET          (0x0880) /* Rx Individual Mask Register 0 (RXIMR0) */
#  define S32K3XX_CAN_RXIMR1_OFFSET          (0x0884) /* Rx Individual Mask Register 1 (RXIMR1) */
#  define S32K3XX_CAN_RXIMR2_OFFSET          (0x0888) /* Rx Individual Mask Register 2 (RXIMR2) */
#  define S32K3XX_CAN_RXIMR3_OFFSET          (0x088c) /* Rx Individual Mask Register 3 (RXIMR3) */
#  define S32K3XX_CAN_RXIMR4_OFFSET          (0x0890) /* Rx Individual Mask Register 4 (RXIMR4) */
#  define S32K3XX_CAN_RXIMR5_OFFSET          (0x0894) /* Rx Individual Mask Register 5 (RXIMR5) */
#  define S32K3XX_CAN_RXIMR6_OFFSET          (0x0898) /* Rx Individual Mask Register 6 (RXIMR6) */
#  define S32K3XX_CAN_RXIMR7_OFFSET          (0x089c) /* Rx Individual Mask Register 7 (RXIMR7) */
#  define S32K3XX_CAN_RXIMR8_OFFSET          (0x08a0) /* Rx Individual Mask Register 8 (RXIMR8) */
#  define S32K3XX_CAN_RXIMR9_OFFSET          (0x08a4) /* Rx Individual Mask Register 9 (RXIMR9) */
#  define S32K3XX_CAN_RXIMR10_OFFSET         (0x08a8) /* Rx Individual Mask Register 10 (RXIMR10) */
#  define S32K3XX_CAN_RXIMR11_OFFSET         (0x08ac) /* Rx Individual Mask Register 11 (RXIMR11) */
#  define S32K3XX_CAN_RXIMR12_OFFSET         (0x08b0) /* Rx Individual Mask Register 12 (RXIMR12) */
#  define S32K3XX_CAN_RXIMR13_OFFSET         (0x08b4) /* Rx Individual Mask Register 13 (RXIMR13) */
#  define S32K3XX_CAN_RXIMR14_OFFSET         (0x08b8) /* Rx Individual Mask Register 14 (RXIMR14) */
#  define S32K3XX_CAN_RXIMR15_OFFSET         (0x08bc) /* Rx Individual Mask Register 15 (RXIMR15) */
#  define S32K3XX_CAN_RXIMR16_OFFSET         (0x08c0) /* Rx Individual Mask Register 16 (RXIMR16) */
#  define S32K3XX_CAN_RXIMR17_OFFSET         (0x08c4) /* Rx Individual Mask Register 17 (RXIMR17) */
#  define S32K3XX_CAN_RXIMR18_OFFSET         (0x08c8) /* Rx Individual Mask Register 18 (RXIMR18) */
#  define S32K3XX_CAN_RXIMR19_OFFSET         (0x08cc) /* Rx Individual Mask Register 19 (RXIMR19) */
#  define S32K3XX_CAN_RXIMR20_OFFSET         (0x08d0) /* Rx Individual Mask Register 20 (RXIMR20) */
#  define S32K3XX_CAN_RXIMR21_OFFSET         (0x08d4) /* Rx Individual Mask Register 21 (RXIMR21) */
#  define S32K3XX_CAN_RXIMR22_OFFSET         (0x08d8) /* Rx Individual Mask Register 22 (RXIMR22) */
#  define S32K3XX_CAN_RXIMR23_OFFSET         (0x08dc) /* Rx Individual Mask Register 23 (RXIMR23) */
#  define S32K3XX_CAN_RXIMR24_OFFSET         (0x08e0) /* Rx Individual Mask Register 24 (RXIMR24) */
#  define S32K3XX_CAN_RXIMR25_OFFSET         (0x08e4) /* Rx Individual Mask Register 25 (RXIMR25) */
#  define S32K3XX_CAN_RXIMR26_OFFSET         (0x08e8) /* Rx Individual Mask Register 26 (RXIMR26) */
#  define S32K3XX_CAN_RXIMR27_OFFSET         (0x08ec) /* Rx Individual Mask Register 27 (RXIMR27) */
#  define S32K3XX_CAN_RXIMR28_OFFSET         (0x08f0) /* Rx Individual Mask Register 28 (RXIMR28) */
#  define S32K3XX_CAN_RXIMR29_OFFSET         (0x08f4) /* Rx Individual Mask Register 29 (RXIMR29) */
#  define S32K3XX_CAN_RXIMR30_OFFSET         (0x08f8) /* Rx Individual Mask Register 30 (RXIMR30) */
#  define S32K3XX_CAN_RXIMR31_OFFSET         (0x08fc) /* Rx Individual Mask Register 31 (RXIMR31) */
#  define S32K3XX_CAN_RXIMR32_OFFSET         (0x0900) /* Rx Individual Mask Register 32 (RXIMR32) */
#  define S32K3XX_CAN_RXIMR33_OFFSET         (0x0904) /* Rx Individual Mask Register 33 (RXIMR33) */
#  define S32K3XX_CAN_RXIMR34_OFFSET         (0x0908) /* Rx Individual Mask Register 34 (RXIMR34) */
#  define S32K3XX_CAN_RXIMR35_OFFSET         (0x090c) /* Rx Individual Mask Register 35 (RXIMR35) */
#  define S32K3XX_CAN_RXIMR36_OFFSET         (0x0910) /* Rx Individual Mask Register 36 (RXIMR36) */
#  define S32K3XX_CAN_RXIMR37_OFFSET         (0x0914) /* Rx Individual Mask Register 37 (RXIMR37) */
#  define S32K3XX_CAN_RXIMR38_OFFSET         (0x0918) /* Rx Individual Mask Register 38 (RXIMR38) */
#  define S32K3XX_CAN_RXIMR39_OFFSET         (0x091c) /* Rx Individual Mask Register 39 (RXIMR39) */
#  define S32K3XX_CAN_RXIMR40_OFFSET         (0x0920) /* Rx Individual Mask Register 40 (RXIMR40) */
#  define S32K3XX_CAN_RXIMR41_OFFSET         (0x0924) /* Rx Individual Mask Register 41 (RXIMR41) */
#  define S32K3XX_CAN_RXIMR42_OFFSET         (0x0928) /* Rx Individual Mask Register 42 (RXIMR42) */
#  define S32K3XX_CAN_RXIMR43_OFFSET         (0x092c) /* Rx Individual Mask Register 43 (RXIMR43) */
#  define S32K3XX_CAN_RXIMR44_OFFSET         (0x0930) /* Rx Individual Mask Register 44 (RXIMR44) */
#  define S32K3XX_CAN_RXIMR45_OFFSET         (0x0934) /* Rx Individual Mask Register 45 (RXIMR45) */
#  define S32K3XX_CAN_RXIMR46_OFFSET         (0x0938) /* Rx Individual Mask Register 46 (RXIMR46) */
#  define S32K3XX_CAN_RXIMR47_OFFSET         (0x093c) /* Rx Individual Mask Register 47 (RXIMR47) */
#  define S32K3XX_CAN_RXIMR48_OFFSET         (0x0940) /* Rx Individual Mask Register 48 (RXIMR48) */
#  define S32K3XX_CAN_RXIMR49_OFFSET         (0x0944) /* Rx Individual Mask Register 49 (RXIMR49) */
#  define S32K3XX_CAN_RXIMR50_OFFSET         (0x0948) /* Rx Individual Mask Register 50 (RXIMR50) */
#  define S32K3XX_CAN_RXIMR51_OFFSET         (0x094c) /* Rx Individual Mask Register 51 (RXIMR51) */
#  define S32K3XX_CAN_RXIMR52_OFFSET         (0x0950) /* Rx Individual Mask Register 52 (RXIMR52) */
#  define S32K3XX_CAN_RXIMR53_OFFSET         (0x0954) /* Rx Individual Mask Register 53 (RXIMR53) */
#  define S32K3XX_CAN_RXIMR54_OFFSET         (0x0958) /* Rx Individual Mask Register 54 (RXIMR54) */
#  define S32K3XX_CAN_RXIMR55_OFFSET         (0x095c) /* Rx Individual Mask Register 55 (RXIMR55) */
#  define S32K3XX_CAN_RXIMR56_OFFSET         (0x0960) /* Rx Individual Mask Register 56 (RXIMR56) */
#  define S32K3XX_CAN_RXIMR57_OFFSET         (0x0964) /* Rx Individual Mask Register 57 (RXIMR57) */
#  define S32K3XX_CAN_RXIMR58_OFFSET         (0x0968) /* Rx Individual Mask Register 58 (RXIMR58) */
#  define S32K3XX_CAN_RXIMR59_OFFSET         (0x096c) /* Rx Individual Mask Register 59 (RXIMR59) */
#  define S32K3XX_CAN_RXIMR60_OFFSET         (0x0970) /* Rx Individual Mask Register 60 (RXIMR60) */
#  define S32K3XX_CAN_RXIMR61_OFFSET         (0x0974) /* Rx Individual Mask Register 61 (RXIMR61) */
#  define S32K3XX_CAN_RXIMR62_OFFSET         (0x0978) /* Rx Individual Mask Register 62 (RXIMR62) */
#  define S32K3XX_CAN_RXIMR63_OFFSET         (0x097c) /* Rx Individual Mask Register 63 (RXIMR63) */
#  define S32K3XX_CAN_RXIMR64_OFFSET         (0x0980) /* Rx Individual Mask Register 64 (RXIMR64) */
#  define S32K3XX_CAN_RXIMR65_OFFSET         (0x0984) /* Rx Individual Mask Register 65 (RXIMR65) */
#  define S32K3XX_CAN_RXIMR66_OFFSET         (0x0988) /* Rx Individual Mask Register 66 (RXIMR66) */
#  define S32K3XX_CAN_RXIMR67_OFFSET         (0x098c) /* Rx Individual Mask Register 67 (RXIMR67) */
#  define S32K3XX_CAN_RXIMR68_OFFSET         (0x0990) /* Rx Individual Mask Register 68 (RXIMR68) */
#  define S32K3XX_CAN_RXIMR69_OFFSET         (0x0994) /* Rx Individual Mask Register 69 (RXIMR69) */
#  define S32K3XX_CAN_RXIMR70_OFFSET         (0x0998) /* Rx Individual Mask Register 70 (RXIMR70) */
#  define S32K3XX_CAN_RXIMR71_OFFSET         (0x099c) /* Rx Individual Mask Register 71 (RXIMR71) */
#  define S32K3XX_CAN_RXIMR72_OFFSET         (0x09a0) /* Rx Individual Mask Register 72 (RXIMR72) */
#  define S32K3XX_CAN_RXIMR73_OFFSET         (0x09a4) /* Rx Individual Mask Register 73 (RXIMR73) */
#  define S32K3XX_CAN_RXIMR74_OFFSET         (0x09a8) /* Rx Individual Mask Register 74 (RXIMR74) */
#  define S32K3XX_CAN_RXIMR75_OFFSET         (0x09ac) /* Rx Individual Mask Register 75 (RXIMR75) */
#  define S32K3XX_CAN_RXIMR76_OFFSET         (0x09b0) /* Rx Individual Mask Register 76 (RXIMR76) */
#  define S32K3XX_CAN_RXIMR77_OFFSET         (0x09b4) /* Rx Individual Mask Register 77 (RXIMR77) */
#  define S32K3XX_CAN_RXIMR78_OFFSET         (0x09b8) /* Rx Individual Mask Register 78 (RXIMR78) */
#  define S32K3XX_CAN_RXIMR79_OFFSET         (0x09bc) /* Rx Individual Mask Register 79 (RXIMR79) */
#  define S32K3XX_CAN_RXIMR80_OFFSET         (0x09c0) /* Rx Individual Mask Register 80 (RXIMR80) */
#  define S32K3XX_CAN_RXIMR81_OFFSET         (0x09c4) /* Rx Individual Mask Register 81 (RXIMR81) */
#  define S32K3XX_CAN_RXIMR82_OFFSET         (0x09c8) /* Rx Individual Mask Register 82 (RXIMR82) */
#  define S32K3XX_CAN_RXIMR83_OFFSET         (0x09cc) /* Rx Individual Mask Register 83 (RXIMR83) */
#  define S32K3XX_CAN_RXIMR84_OFFSET         (0x09d0) /* Rx Individual Mask Register 84 (RXIMR84) */
#  define S32K3XX_CAN_RXIMR85_OFFSET         (0x09d4) /* Rx Individual Mask Register 85 (RXIMR85) */
#  define S32K3XX_CAN_RXIMR86_OFFSET         (0x09d8) /* Rx Individual Mask Register 86 (RXIMR86) */
#  define S32K3XX_CAN_RXIMR87_OFFSET         (0x09dc) /* Rx Individual Mask Register 87 (RXIMR87) */
#  define S32K3XX_CAN_RXIMR88_OFFSET         (0x09e0) /* Rx Individual Mask Register 88 (RXIMR88) */
#  define S32K3XX_CAN_RXIMR89_OFFSET         (0x09e4) /* Rx Individual Mask Register 89 (RXIMR89) */
#  define S32K3XX_CAN_RXIMR90_OFFSET         (0x09e8) /* Rx Individual Mask Register 90 (RXIMR90) */
#  define S32K3XX_CAN_RXIMR91_OFFSET         (0x09ec) /* Rx Individual Mask Register 91 (RXIMR91) */
#  define S32K3XX_CAN_RXIMR92_OFFSET         (0x09f0) /* Rx Individual Mask Register 92 (RXIMR92) */
#  define S32K3XX_CAN_RXIMR93_OFFSET         (0x09f4) /* Rx Individual Mask Register 93 (RXIMR93) */
#  define S32K3XX_CAN_RXIMR94_OFFSET         (0x09f8) /* Rx Individual Mask Register 94 (RXIMR94) */
#  define S32K3XX_CAN_RXIMR95_OFFSET         (0x09fc) /* Rx Individual Mask Register 95 (RXIMR95) */
#define S32K3XX_CAN_RXIMR_COUNT              (96)     /* Individual Mask Register Count */

#define S32K3XX_CAN_MECR_OFFSET              (0x0ae0) /* Memory Error Control Register (MECR) */
#define S32K3XX_CAN_ERRIAR_OFFSET            (0x0ae4) /* Error Injection Address Register (ERRIAR) */
#define S32K3XX_CAN_ERRIDPR_OFFSET           (0x0ae8) /* Error Injection Data Pattern Register (ERRIDPR) */
#define S32K3XX_CAN_ERRIPPR_OFFSET           (0x0aec) /* Error Injection Parity Pattern Register (ERRIPPR) */
#define S32K3XX_CAN_RERRAR_OFFSET            (0x0af0) /* Error Report Address Register (RERRAR) */
#define S32K3XX_CAN_RERRDR_OFFSET            (0x0af4) /* Error Report Data Register (RERRDR) */
#define S32K3XX_CAN_RERRSYNR_OFFSET          (0x0af8) /* Error Report Syndrome Register (RERRSYNR) */
#define S32K3XX_CAN_ERRSR_OFFSET             (0x0afc) /* Error Status Register (ERRSR) */
#define S32K3XX_CAN_EPRS_OFFSET              (0x0bf0) /* Enhanced CAN Bit Timing Prescalers (EPRS) */
#define S32K3XX_CAN_ENCBT_OFFSET             (0x0bf4) /* Enhanced Nominal CAN Bit Timing (ENCBT) */
#define S32K3XX_CAN_EDCBT_OFFSET             (0x0bf8) /* Enhanced Data Phase CAN Bit Timing (EDCBT) */
#define S32K3XX_CAN_ETDC_OFFSET              (0x0bfc) /* Enhanced Transceiver Delay Compensation (ETDC) */
#define S32K3XX_CAN_FDCTRL_OFFSET            (0x0c00) /* CAN FD Control Register (FDCTRL) */
#define S32K3XX_CAN_FDCBT_OFFSET             (0x0c04) /* CAN FD Bit Timing Register (FDCBT) */
#define S32K3XX_CAN_FDCRC_OFFSET             (0x0c08) /* CAN FD CRC Register (FDCRC) */
#define S32K3XX_CAN_ERFCR_OFFSET             (0x0c0c) /* Enhanced Rx FIFO Control Register (ERFCR) */
#define S32K3XX_CAN_ERFIER_OFFSET            (0x0c10) /* Enhanced Rx FIFO Interrupt Enable Register (ERFIER) */
#define S32K3XX_CAN_ERFSR_OFFSET             (0x0c14) /* Enhanced Rx FIFO Status Register (ERFSR) */

#define S32K3XX_CAN_HR_TIME_STAMP_OFFSET(n)  (0x0c30 + ((n) << 2))
#  define S32K3XX_CAN_HR_TIME_STAMP0_OFFSET  (0x0c30) /* High Resolution Time Stamp 0 (HR_TIME_STAMP0) */
#  define S32K3XX_CAN_HR_TIME_STAMP1_OFFSET  (0x0c34) /* High Resolution Time Stamp 1 (HR_TIME_STAMP1) */
#  define S32K3XX_CAN_HR_TIME_STAMP2_OFFSET  (0x0c38) /* High Resolution Time Stamp 2 (HR_TIME_STAMP2) */
#  define S32K3XX_CAN_HR_TIME_STAMP3_OFFSET  (0x0c3c) /* High Resolution Time Stamp 3 (HR_TIME_STAMP3) */
#  define S32K3XX_CAN_HR_TIME_STAMP4_OFFSET  (0x0c40) /* High Resolution Time Stamp 4 (HR_TIME_STAMP4) */
#  define S32K3XX_CAN_HR_TIME_STAMP5_OFFSET  (0x0c44) /* High Resolution Time Stamp 5 (HR_TIME_STAMP5) */
#  define S32K3XX_CAN_HR_TIME_STAMP6_OFFSET  (0x0c48) /* High Resolution Time Stamp 6 (HR_TIME_STAMP6) */
#  define S32K3XX_CAN_HR_TIME_STAMP7_OFFSET  (0x0c4c) /* High Resolution Time Stamp 7 (HR_TIME_STAMP7) */
#  define S32K3XX_CAN_HR_TIME_STAMP8_OFFSET  (0x0c50) /* High Resolution Time Stamp 8 (HR_TIME_STAMP8) */
#  define S32K3XX_CAN_HR_TIME_STAMP9_OFFSET  (0x0c54) /* High Resolution Time Stamp 9 (HR_TIME_STAMP9) */
#  define S32K3XX_CAN_HR_TIME_STAMP10_OFFSET (0x0c58) /* High Resolution Time Stamp 10 (HR_TIME_STAMP10) */
#  define S32K3XX_CAN_HR_TIME_STAMP11_OFFSET (0x0c5c) /* High Resolution Time Stamp 11 (HR_TIME_STAMP11) */
#  define S32K3XX_CAN_HR_TIME_STAMP12_OFFSET (0x0c60) /* High Resolution Time Stamp 12 (HR_TIME_STAMP12) */
#  define S32K3XX_CAN_HR_TIME_STAMP13_OFFSET (0x0c64) /* High Resolution Time Stamp 13 (HR_TIME_STAMP13) */
#  define S32K3XX_CAN_HR_TIME_STAMP14_OFFSET (0x0c68) /* High Resolution Time Stamp 14 (HR_TIME_STAMP14) */
#  define S32K3XX_CAN_HR_TIME_STAMP15_OFFSET (0x0c6c) /* High Resolution Time Stamp 15 (HR_TIME_STAMP15) */
#  define S32K3XX_CAN_HR_TIME_STAMP16_OFFSET (0x0c70) /* High Resolution Time Stamp 16 (HR_TIME_STAMP16) */
#  define S32K3XX_CAN_HR_TIME_STAMP17_OFFSET (0x0c74) /* High Resolution Time Stamp 17 (HR_TIME_STAMP17) */
#  define S32K3XX_CAN_HR_TIME_STAMP18_OFFSET (0x0c78) /* High Resolution Time Stamp 18 (HR_TIME_STAMP18) */
#  define S32K3XX_CAN_HR_TIME_STAMP19_OFFSET (0x0c7c) /* High Resolution Time Stamp 19 (HR_TIME_STAMP19) */
#  define S32K3XX_CAN_HR_TIME_STAMP20_OFFSET (0x0c80) /* High Resolution Time Stamp 20 (HR_TIME_STAMP20) */
#  define S32K3XX_CAN_HR_TIME_STAMP21_OFFSET (0x0c84) /* High Resolution Time Stamp 21 (HR_TIME_STAMP21) */
#  define S32K3XX_CAN_HR_TIME_STAMP22_OFFSET (0x0c88) /* High Resolution Time Stamp 22 (HR_TIME_STAMP22) */
#  define S32K3XX_CAN_HR_TIME_STAMP23_OFFSET (0x0c8c) /* High Resolution Time Stamp 23 (HR_TIME_STAMP23) */
#  define S32K3XX_CAN_HR_TIME_STAMP24_OFFSET (0x0c90) /* High Resolution Time Stamp 24 (HR_TIME_STAMP24) */
#  define S32K3XX_CAN_HR_TIME_STAMP25_OFFSET (0x0c94) /* High Resolution Time Stamp 25 (HR_TIME_STAMP25) */
#  define S32K3XX_CAN_HR_TIME_STAMP26_OFFSET (0x0c98) /* High Resolution Time Stamp 26 (HR_TIME_STAMP26) */
#  define S32K3XX_CAN_HR_TIME_STAMP27_OFFSET (0x0c9c) /* High Resolution Time Stamp 27 (HR_TIME_STAMP27) */
#  define S32K3XX_CAN_HR_TIME_STAMP28_OFFSET (0x0ca0) /* High Resolution Time Stamp 28 (HR_TIME_STAMP28) */
#  define S32K3XX_CAN_HR_TIME_STAMP29_OFFSET (0x0ca4) /* High Resolution Time Stamp 29 (HR_TIME_STAMP29) */
#  define S32K3XX_CAN_HR_TIME_STAMP30_OFFSET (0x0ca8) /* High Resolution Time Stamp 30 (HR_TIME_STAMP30) */
#  define S32K3XX_CAN_HR_TIME_STAMP31_OFFSET (0x0cac) /* High Resolution Time Stamp 31 (HR_TIME_STAMP31) */
#  define S32K3XX_CAN_HR_TIME_STAMP32_OFFSET (0x0cb0) /* High Resolution Time Stamp 32 (HR_TIME_STAMP32) */
#  define S32K3XX_CAN_HR_TIME_STAMP33_OFFSET (0x0cb4) /* High Resolution Time Stamp 33 (HR_TIME_STAMP33) */
#  define S32K3XX_CAN_HR_TIME_STAMP34_OFFSET (0x0cb8) /* High Resolution Time Stamp 34 (HR_TIME_STAMP34) */
#  define S32K3XX_CAN_HR_TIME_STAMP35_OFFSET (0x0cbc) /* High Resolution Time Stamp 35 (HR_TIME_STAMP35) */
#  define S32K3XX_CAN_HR_TIME_STAMP36_OFFSET (0x0cc0) /* High Resolution Time Stamp 36 (HR_TIME_STAMP36) */
#  define S32K3XX_CAN_HR_TIME_STAMP37_OFFSET (0x0cc4) /* High Resolution Time Stamp 37 (HR_TIME_STAMP37) */
#  define S32K3XX_CAN_HR_TIME_STAMP38_OFFSET (0x0cc8) /* High Resolution Time Stamp 38 (HR_TIME_STAMP38) */
#  define S32K3XX_CAN_HR_TIME_STAMP39_OFFSET (0x0ccc) /* High Resolution Time Stamp 39 (HR_TIME_STAMP39) */
#  define S32K3XX_CAN_HR_TIME_STAMP40_OFFSET (0x0cd0) /* High Resolution Time Stamp 40 (HR_TIME_STAMP40) */
#  define S32K3XX_CAN_HR_TIME_STAMP41_OFFSET (0x0cd4) /* High Resolution Time Stamp 41 (HR_TIME_STAMP41) */
#  define S32K3XX_CAN_HR_TIME_STAMP42_OFFSET (0x0cd8) /* High Resolution Time Stamp 42 (HR_TIME_STAMP42) */
#  define S32K3XX_CAN_HR_TIME_STAMP43_OFFSET (0x0cdc) /* High Resolution Time Stamp 43 (HR_TIME_STAMP43) */
#  define S32K3XX_CAN_HR_TIME_STAMP44_OFFSET (0x0ce0) /* High Resolution Time Stamp 44 (HR_TIME_STAMP44) */
#  define S32K3XX_CAN_HR_TIME_STAMP45_OFFSET (0x0ce4) /* High Resolution Time Stamp 45 (HR_TIME_STAMP45) */
#  define S32K3XX_CAN_HR_TIME_STAMP46_OFFSET (0x0ce8) /* High Resolution Time Stamp 46 (HR_TIME_STAMP46) */
#  define S32K3XX_CAN_HR_TIME_STAMP47_OFFSET (0x0cec) /* High Resolution Time Stamp 47 (HR_TIME_STAMP47) */
#  define S32K3XX_CAN_HR_TIME_STAMP48_OFFSET (0x0cf0) /* High Resolution Time Stamp 48 (HR_TIME_STAMP48) */
#  define S32K3XX_CAN_HR_TIME_STAMP49_OFFSET (0x0cf4) /* High Resolution Time Stamp 49 (HR_TIME_STAMP49) */
#  define S32K3XX_CAN_HR_TIME_STAMP50_OFFSET (0x0cf8) /* High Resolution Time Stamp 50 (HR_TIME_STAMP50) */
#  define S32K3XX_CAN_HR_TIME_STAMP51_OFFSET (0x0cfc) /* High Resolution Time Stamp 51 (HR_TIME_STAMP51) */
#  define S32K3XX_CAN_HR_TIME_STAMP52_OFFSET (0x0d00) /* High Resolution Time Stamp 52 (HR_TIME_STAMP52) */
#  define S32K3XX_CAN_HR_TIME_STAMP53_OFFSET (0x0d04) /* High Resolution Time Stamp 53 (HR_TIME_STAMP53) */
#  define S32K3XX_CAN_HR_TIME_STAMP54_OFFSET (0x0d08) /* High Resolution Time Stamp 54 (HR_TIME_STAMP54) */
#  define S32K3XX_CAN_HR_TIME_STAMP55_OFFSET (0x0d0c) /* High Resolution Time Stamp 55 (HR_TIME_STAMP55) */
#  define S32K3XX_CAN_HR_TIME_STAMP56_OFFSET (0x0d10) /* High Resolution Time Stamp 56 (HR_TIME_STAMP56) */
#  define S32K3XX_CAN_HR_TIME_STAMP57_OFFSET (0x0d14) /* High Resolution Time Stamp 57 (HR_TIME_STAMP57) */
#  define S32K3XX_CAN_HR_TIME_STAMP58_OFFSET (0x0d18) /* High Resolution Time Stamp 58 (HR_TIME_STAMP58) */
#  define S32K3XX_CAN_HR_TIME_STAMP59_OFFSET (0x0d1c) /* High Resolution Time Stamp 59 (HR_TIME_STAMP59) */
#  define S32K3XX_CAN_HR_TIME_STAMP60_OFFSET (0x0d20) /* High Resolution Time Stamp 60 (HR_TIME_STAMP60) */
#  define S32K3XX_CAN_HR_TIME_STAMP61_OFFSET (0x0d24) /* High Resolution Time Stamp 61 (HR_TIME_STAMP61) */
#  define S32K3XX_CAN_HR_TIME_STAMP62_OFFSET (0x0d28) /* High Resolution Time Stamp 62 (HR_TIME_STAMP62) */
#  define S32K3XX_CAN_HR_TIME_STAMP63_OFFSET (0x0d2c) /* High Resolution Time Stamp 63 (HR_TIME_STAMP63) */
#  define S32K3XX_CAN_HR_TIME_STAMP64_OFFSET (0x0d30) /* High Resolution Time Stamp 64 (HR_TIME_STAMP64) */
#  define S32K3XX_CAN_HR_TIME_STAMP65_OFFSET (0x0d34) /* High Resolution Time Stamp 65 (HR_TIME_STAMP65) */
#  define S32K3XX_CAN_HR_TIME_STAMP66_OFFSET (0x0d38) /* High Resolution Time Stamp 66 (HR_TIME_STAMP66) */
#  define S32K3XX_CAN_HR_TIME_STAMP67_OFFSET (0x0d3c) /* High Resolution Time Stamp 67 (HR_TIME_STAMP67) */
#  define S32K3XX_CAN_HR_TIME_STAMP68_OFFSET (0x0d40) /* High Resolution Time Stamp 68 (HR_TIME_STAMP68) */
#  define S32K3XX_CAN_HR_TIME_STAMP69_OFFSET (0x0d44) /* High Resolution Time Stamp 69 (HR_TIME_STAMP69) */
#  define S32K3XX_CAN_HR_TIME_STAMP70_OFFSET (0x0d48) /* High Resolution Time Stamp 70 (HR_TIME_STAMP70) */
#  define S32K3XX_CAN_HR_TIME_STAMP71_OFFSET (0x0d4c) /* High Resolution Time Stamp 71 (HR_TIME_STAMP71) */
#  define S32K3XX_CAN_HR_TIME_STAMP72_OFFSET (0x0d50) /* High Resolution Time Stamp 72 (HR_TIME_STAMP72) */
#  define S32K3XX_CAN_HR_TIME_STAMP73_OFFSET (0x0d54) /* High Resolution Time Stamp 73 (HR_TIME_STAMP73) */
#  define S32K3XX_CAN_HR_TIME_STAMP74_OFFSET (0x0d58) /* High Resolution Time Stamp 74 (HR_TIME_STAMP74) */
#  define S32K3XX_CAN_HR_TIME_STAMP75_OFFSET (0x0d5c) /* High Resolution Time Stamp 75 (HR_TIME_STAMP75) */
#  define S32K3XX_CAN_HR_TIME_STAMP76_OFFSET (0x0d60) /* High Resolution Time Stamp 76 (HR_TIME_STAMP76) */
#  define S32K3XX_CAN_HR_TIME_STAMP77_OFFSET (0x0d64) /* High Resolution Time Stamp 77 (HR_TIME_STAMP77) */
#  define S32K3XX_CAN_HR_TIME_STAMP78_OFFSET (0x0d68) /* High Resolution Time Stamp 78 (HR_TIME_STAMP78) */
#  define S32K3XX_CAN_HR_TIME_STAMP79_OFFSET (0x0d6c) /* High Resolution Time Stamp 79 (HR_TIME_STAMP79) */
#  define S32K3XX_CAN_HR_TIME_STAMP80_OFFSET (0x0d70) /* High Resolution Time Stamp 80 (HR_TIME_STAMP80) */
#  define S32K3XX_CAN_HR_TIME_STAMP81_OFFSET (0x0d74) /* High Resolution Time Stamp 81 (HR_TIME_STAMP81) */
#  define S32K3XX_CAN_HR_TIME_STAMP82_OFFSET (0x0d78) /* High Resolution Time Stamp 82 (HR_TIME_STAMP82) */
#  define S32K3XX_CAN_HR_TIME_STAMP83_OFFSET (0x0d7c) /* High Resolution Time Stamp 83 (HR_TIME_STAMP83) */
#  define S32K3XX_CAN_HR_TIME_STAMP84_OFFSET (0x0d80) /* High Resolution Time Stamp 84 (HR_TIME_STAMP84) */
#  define S32K3XX_CAN_HR_TIME_STAMP85_OFFSET (0x0d84) /* High Resolution Time Stamp 85 (HR_TIME_STAMP85) */
#  define S32K3XX_CAN_HR_TIME_STAMP86_OFFSET (0x0d88) /* High Resolution Time Stamp 86 (HR_TIME_STAMP86) */
#  define S32K3XX_CAN_HR_TIME_STAMP87_OFFSET (0x0d8c) /* High Resolution Time Stamp 87 (HR_TIME_STAMP87) */
#  define S32K3XX_CAN_HR_TIME_STAMP88_OFFSET (0x0d90) /* High Resolution Time Stamp 88 (HR_TIME_STAMP88) */
#  define S32K3XX_CAN_HR_TIME_STAMP89_OFFSET (0x0d94) /* High Resolution Time Stamp 89 (HR_TIME_STAMP89) */
#  define S32K3XX_CAN_HR_TIME_STAMP90_OFFSET (0x0d98) /* High Resolution Time Stamp 90 (HR_TIME_STAMP90) */
#  define S32K3XX_CAN_HR_TIME_STAMP91_OFFSET (0x0d9c) /* High Resolution Time Stamp 91 (HR_TIME_STAMP91) */
#  define S32K3XX_CAN_HR_TIME_STAMP92_OFFSET (0x0da0) /* High Resolution Time Stamp 92 (HR_TIME_STAMP92) */
#  define S32K3XX_CAN_HR_TIME_STAMP93_OFFSET (0x0da4) /* High Resolution Time Stamp 93 (HR_TIME_STAMP93) */
#  define S32K3XX_CAN_HR_TIME_STAMP94_OFFSET (0x0da8) /* High Resolution Time Stamp 94 (HR_TIME_STAMP94) */
#  define S32K3XX_CAN_HR_TIME_STAMP95_OFFSET (0x0dac) /* High Resolution Time Stamp 95 (HR_TIME_STAMP95) */
#define S32K3XX_CAN_HR_TIME_STAMP_COUNT      (96)     /* High Resolution Time Stamp Count */

#define S32K3XX_CAN_ERFFEL_OFFSET(n)         (0x3000 + ((n) << 2))
#  define S32K3XX_CAN_ERFFEL0_OFFSET         (0x3000) /* Enhanced Rx FIFO Filter Element 0 (ERFFEL0) */
#  define S32K3XX_CAN_ERFFEL1_OFFSET         (0x3004) /* Enhanced Rx FIFO Filter Element 1 (ERFFEL1) */
#  define S32K3XX_CAN_ERFFEL2_OFFSET         (0x3008) /* Enhanced Rx FIFO Filter Element 2 (ERFFEL2) */
#  define S32K3XX_CAN_ERFFEL3_OFFSET         (0x300c) /* Enhanced Rx FIFO Filter Element 3 (ERFFEL3) */
#  define S32K3XX_CAN_ERFFEL4_OFFSET         (0x3010) /* Enhanced Rx FIFO Filter Element 4 (ERFFEL4) */
#  define S32K3XX_CAN_ERFFEL5_OFFSET         (0x3014) /* Enhanced Rx FIFO Filter Element 5 (ERFFEL5) */
#  define S32K3XX_CAN_ERFFEL6_OFFSET         (0x3018) /* Enhanced Rx FIFO Filter Element 6 (ERFFEL6) */
#  define S32K3XX_CAN_ERFFEL7_OFFSET         (0x301c) /* Enhanced Rx FIFO Filter Element 7 (ERFFEL7) */
#  define S32K3XX_CAN_ERFFEL8_OFFSET         (0x3020) /* Enhanced Rx FIFO Filter Element 8 (ERFFEL8) */
#  define S32K3XX_CAN_ERFFEL9_OFFSET         (0x3024) /* Enhanced Rx FIFO Filter Element 9 (ERFFEL9) */
#  define S32K3XX_CAN_ERFFEL10_OFFSET        (0x3028) /* Enhanced Rx FIFO Filter Element 10 (ERFFEL10) */
#  define S32K3XX_CAN_ERFFEL11_OFFSET        (0x302c) /* Enhanced Rx FIFO Filter Element 11 (ERFFEL11) */
#  define S32K3XX_CAN_ERFFEL12_OFFSET        (0x3030) /* Enhanced Rx FIFO Filter Element 12 (ERFFEL12) */
#  define S32K3XX_CAN_ERFFEL13_OFFSET        (0x3034) /* Enhanced Rx FIFO Filter Element 13 (ERFFEL13) */
#  define S32K3XX_CAN_ERFFEL14_OFFSET        (0x3038) /* Enhanced Rx FIFO Filter Element 14 (ERFFEL14) */
#  define S32K3XX_CAN_ERFFEL15_OFFSET        (0x303c) /* Enhanced Rx FIFO Filter Element 15 (ERFFEL15) */
#  define S32K3XX_CAN_ERFFEL16_OFFSET        (0x3040) /* Enhanced Rx FIFO Filter Element 16 (ERFFEL16) */
#  define S32K3XX_CAN_ERFFEL17_OFFSET        (0x3044) /* Enhanced Rx FIFO Filter Element 17 (ERFFEL17) */
#  define S32K3XX_CAN_ERFFEL18_OFFSET        (0x3048) /* Enhanced Rx FIFO Filter Element 18 (ERFFEL18) */
#  define S32K3XX_CAN_ERFFEL19_OFFSET        (0x304c) /* Enhanced Rx FIFO Filter Element 19 (ERFFEL19) */
#  define S32K3XX_CAN_ERFFEL20_OFFSET        (0x3050) /* Enhanced Rx FIFO Filter Element 20 (ERFFEL20) */
#  define S32K3XX_CAN_ERFFEL21_OFFSET        (0x3054) /* Enhanced Rx FIFO Filter Element 21 (ERFFEL21) */
#  define S32K3XX_CAN_ERFFEL22_OFFSET        (0x3058) /* Enhanced Rx FIFO Filter Element 22 (ERFFEL22) */
#  define S32K3XX_CAN_ERFFEL23_OFFSET        (0x305c) /* Enhanced Rx FIFO Filter Element 23 (ERFFEL23) */
#  define S32K3XX_CAN_ERFFEL24_OFFSET        (0x3060) /* Enhanced Rx FIFO Filter Element 24 (ERFFEL24) */
#  define S32K3XX_CAN_ERFFEL25_OFFSET        (0x3064) /* Enhanced Rx FIFO Filter Element 25 (ERFFEL25) */
#  define S32K3XX_CAN_ERFFEL26_OFFSET        (0x3068) /* Enhanced Rx FIFO Filter Element 26 (ERFFEL26) */
#  define S32K3XX_CAN_ERFFEL27_OFFSET        (0x306c) /* Enhanced Rx FIFO Filter Element 27 (ERFFEL27) */
#  define S32K3XX_CAN_ERFFEL28_OFFSET        (0x3070) /* Enhanced Rx FIFO Filter Element 28 (ERFFEL28) */
#  define S32K3XX_CAN_ERFFEL29_OFFSET        (0x3074) /* Enhanced Rx FIFO Filter Element 29 (ERFFEL29) */
#  define S32K3XX_CAN_ERFFEL30_OFFSET        (0x3078) /* Enhanced Rx FIFO Filter Element 30 (ERFFEL30) */
#  define S32K3XX_CAN_ERFFEL31_OFFSET        (0x307c) /* Enhanced Rx FIFO Filter Element 31 (ERFFEL31) */
#  define S32K3XX_CAN_ERFFEL32_OFFSET        (0x3080) /* Enhanced Rx FIFO Filter Element 32 (ERFFEL32) */
#  define S32K3XX_CAN_ERFFEL33_OFFSET        (0x3084) /* Enhanced Rx FIFO Filter Element 33 (ERFFEL33) */
#  define S32K3XX_CAN_ERFFEL34_OFFSET        (0x3088) /* Enhanced Rx FIFO Filter Element 34 (ERFFEL34) */
#  define S32K3XX_CAN_ERFFEL35_OFFSET        (0x308c) /* Enhanced Rx FIFO Filter Element 35 (ERFFEL35) */
#  define S32K3XX_CAN_ERFFEL36_OFFSET        (0x3090) /* Enhanced Rx FIFO Filter Element 36 (ERFFEL36) */
#  define S32K3XX_CAN_ERFFEL37_OFFSET        (0x3094) /* Enhanced Rx FIFO Filter Element 37 (ERFFEL37) */
#  define S32K3XX_CAN_ERFFEL38_OFFSET        (0x3098) /* Enhanced Rx FIFO Filter Element 38 (ERFFEL38) */
#  define S32K3XX_CAN_ERFFEL39_OFFSET        (0x309c) /* Enhanced Rx FIFO Filter Element 39 (ERFFEL39) */
#  define S32K3XX_CAN_ERFFEL40_OFFSET        (0x30a0) /* Enhanced Rx FIFO Filter Element 40 (ERFFEL40) */
#  define S32K3XX_CAN_ERFFEL41_OFFSET        (0x30a4) /* Enhanced Rx FIFO Filter Element 41 (ERFFEL41) */
#  define S32K3XX_CAN_ERFFEL42_OFFSET        (0x30a8) /* Enhanced Rx FIFO Filter Element 42 (ERFFEL42) */
#  define S32K3XX_CAN_ERFFEL43_OFFSET        (0x30ac) /* Enhanced Rx FIFO Filter Element 43 (ERFFEL43) */
#  define S32K3XX_CAN_ERFFEL44_OFFSET        (0x30b0) /* Enhanced Rx FIFO Filter Element 44 (ERFFEL44) */
#  define S32K3XX_CAN_ERFFEL45_OFFSET        (0x30b4) /* Enhanced Rx FIFO Filter Element 45 (ERFFEL45) */
#  define S32K3XX_CAN_ERFFEL46_OFFSET        (0x30b8) /* Enhanced Rx FIFO Filter Element 46 (ERFFEL46) */
#  define S32K3XX_CAN_ERFFEL47_OFFSET        (0x30bc) /* Enhanced Rx FIFO Filter Element 47 (ERFFEL47) */
#  define S32K3XX_CAN_ERFFEL48_OFFSET        (0x30c0) /* Enhanced Rx FIFO Filter Element 48 (ERFFEL48) */
#  define S32K3XX_CAN_ERFFEL49_OFFSET        (0x30c4) /* Enhanced Rx FIFO Filter Element 49 (ERFFEL49) */
#  define S32K3XX_CAN_ERFFEL50_OFFSET        (0x30c8) /* Enhanced Rx FIFO Filter Element 50 (ERFFEL50) */
#  define S32K3XX_CAN_ERFFEL51_OFFSET        (0x30cc) /* Enhanced Rx FIFO Filter Element 51 (ERFFEL51) */
#  define S32K3XX_CAN_ERFFEL52_OFFSET        (0x30d0) /* Enhanced Rx FIFO Filter Element 52 (ERFFEL52) */
#  define S32K3XX_CAN_ERFFEL53_OFFSET        (0x30d4) /* Enhanced Rx FIFO Filter Element 53 (ERFFEL53) */
#  define S32K3XX_CAN_ERFFEL54_OFFSET        (0x30d8) /* Enhanced Rx FIFO Filter Element 54 (ERFFEL54) */
#  define S32K3XX_CAN_ERFFEL55_OFFSET        (0x30dc) /* Enhanced Rx FIFO Filter Element 55 (ERFFEL55) */
#  define S32K3XX_CAN_ERFFEL56_OFFSET        (0x30e0) /* Enhanced Rx FIFO Filter Element 56 (ERFFEL56) */
#  define S32K3XX_CAN_ERFFEL57_OFFSET        (0x30e4) /* Enhanced Rx FIFO Filter Element 57 (ERFFEL57) */
#  define S32K3XX_CAN_ERFFEL58_OFFSET        (0x30e8) /* Enhanced Rx FIFO Filter Element 58 (ERFFEL58) */
#  define S32K3XX_CAN_ERFFEL59_OFFSET        (0x30ec) /* Enhanced Rx FIFO Filter Element 59 (ERFFEL59) */
#  define S32K3XX_CAN_ERFFEL60_OFFSET        (0x30f0) /* Enhanced Rx FIFO Filter Element 60 (ERFFEL60) */
#  define S32K3XX_CAN_ERFFEL61_OFFSET        (0x30f4) /* Enhanced Rx FIFO Filter Element 61 (ERFFEL61) */
#  define S32K3XX_CAN_ERFFEL62_OFFSET        (0x30f8) /* Enhanced Rx FIFO Filter Element 62 (ERFFEL62) */
#  define S32K3XX_CAN_ERFFEL63_OFFSET        (0x30fc) /* Enhanced Rx FIFO Filter Element 63 (ERFFEL63) */
#  define S32K3XX_CAN_ERFFEL64_OFFSET        (0x3100) /* Enhanced Rx FIFO Filter Element 64 (ERFFEL64) */
#  define S32K3XX_CAN_ERFFEL65_OFFSET        (0x3104) /* Enhanced Rx FIFO Filter Element 65 (ERFFEL65) */
#  define S32K3XX_CAN_ERFFEL66_OFFSET        (0x3108) /* Enhanced Rx FIFO Filter Element 66 (ERFFEL66) */
#  define S32K3XX_CAN_ERFFEL67_OFFSET        (0x310c) /* Enhanced Rx FIFO Filter Element 67 (ERFFEL67) */
#  define S32K3XX_CAN_ERFFEL68_OFFSET        (0x3110) /* Enhanced Rx FIFO Filter Element 68 (ERFFEL68) */
#  define S32K3XX_CAN_ERFFEL69_OFFSET        (0x3114) /* Enhanced Rx FIFO Filter Element 69 (ERFFEL69) */
#  define S32K3XX_CAN_ERFFEL70_OFFSET        (0x3118) /* Enhanced Rx FIFO Filter Element 70 (ERFFEL70) */
#  define S32K3XX_CAN_ERFFEL71_OFFSET        (0x311c) /* Enhanced Rx FIFO Filter Element 71 (ERFFEL71) */
#  define S32K3XX_CAN_ERFFEL72_OFFSET        (0x3120) /* Enhanced Rx FIFO Filter Element 72 (ERFFEL72) */
#  define S32K3XX_CAN_ERFFEL73_OFFSET        (0x3124) /* Enhanced Rx FIFO Filter Element 73 (ERFFEL73) */
#  define S32K3XX_CAN_ERFFEL74_OFFSET        (0x3128) /* Enhanced Rx FIFO Filter Element 74 (ERFFEL74) */
#  define S32K3XX_CAN_ERFFEL75_OFFSET        (0x312c) /* Enhanced Rx FIFO Filter Element 75 (ERFFEL75) */
#  define S32K3XX_CAN_ERFFEL76_OFFSET        (0x3130) /* Enhanced Rx FIFO Filter Element 76 (ERFFEL76) */
#  define S32K3XX_CAN_ERFFEL77_OFFSET        (0x3134) /* Enhanced Rx FIFO Filter Element 77 (ERFFEL77) */
#  define S32K3XX_CAN_ERFFEL78_OFFSET        (0x3138) /* Enhanced Rx FIFO Filter Element 78 (ERFFEL78) */
#  define S32K3XX_CAN_ERFFEL79_OFFSET        (0x313c) /* Enhanced Rx FIFO Filter Element 79 (ERFFEL79) */
#  define S32K3XX_CAN_ERFFEL80_OFFSET        (0x3140) /* Enhanced Rx FIFO Filter Element 80 (ERFFEL80) */
#  define S32K3XX_CAN_ERFFEL81_OFFSET        (0x3144) /* Enhanced Rx FIFO Filter Element 81 (ERFFEL81) */
#  define S32K3XX_CAN_ERFFEL82_OFFSET        (0x3148) /* Enhanced Rx FIFO Filter Element 82 (ERFFEL82) */
#  define S32K3XX_CAN_ERFFEL83_OFFSET        (0x314c) /* Enhanced Rx FIFO Filter Element 83 (ERFFEL83) */
#  define S32K3XX_CAN_ERFFEL84_OFFSET        (0x3150) /* Enhanced Rx FIFO Filter Element 84 (ERFFEL84) */
#  define S32K3XX_CAN_ERFFEL85_OFFSET        (0x3154) /* Enhanced Rx FIFO Filter Element 85 (ERFFEL85) */
#  define S32K3XX_CAN_ERFFEL86_OFFSET        (0x3158) /* Enhanced Rx FIFO Filter Element 86 (ERFFEL86) */
#  define S32K3XX_CAN_ERFFEL87_OFFSET        (0x315c) /* Enhanced Rx FIFO Filter Element 87 (ERFFEL87) */
#  define S32K3XX_CAN_ERFFEL88_OFFSET        (0x3160) /* Enhanced Rx FIFO Filter Element 88 (ERFFEL88) */
#  define S32K3XX_CAN_ERFFEL89_OFFSET        (0x3164) /* Enhanced Rx FIFO Filter Element 89 (ERFFEL89) */
#  define S32K3XX_CAN_ERFFEL90_OFFSET        (0x3168) /* Enhanced Rx FIFO Filter Element 90 (ERFFEL90) */
#  define S32K3XX_CAN_ERFFEL91_OFFSET        (0x316c) /* Enhanced Rx FIFO Filter Element 91 (ERFFEL91) */
#  define S32K3XX_CAN_ERFFEL92_OFFSET        (0x3170) /* Enhanced Rx FIFO Filter Element 92 (ERFFEL92) */
#  define S32K3XX_CAN_ERFFEL93_OFFSET        (0x3174) /* Enhanced Rx FIFO Filter Element 93 (ERFFEL93) */
#  define S32K3XX_CAN_ERFFEL94_OFFSET        (0x3178) /* Enhanced Rx FIFO Filter Element 94 (ERFFEL94) */
#  define S32K3XX_CAN_ERFFEL95_OFFSET        (0x317c) /* Enhanced Rx FIFO Filter Element 95 (ERFFEL95) */
#  define S32K3XX_CAN_ERFFEL96_OFFSET        (0x3180) /* Enhanced Rx FIFO Filter Element 96 (ERFFEL96) */
#  define S32K3XX_CAN_ERFFEL97_OFFSET        (0x3184) /* Enhanced Rx FIFO Filter Element 97 (ERFFEL97) */
#  define S32K3XX_CAN_ERFFEL98_OFFSET        (0x3188) /* Enhanced Rx FIFO Filter Element 98 (ERFFEL98) */
#  define S32K3XX_CAN_ERFFEL99_OFFSET        (0x318c) /* Enhanced Rx FIFO Filter Element 99 (ERFFEL99) */
#  define S32K3XX_CAN_ERFFEL100_OFFSET       (0x3190) /* Enhanced Rx FIFO Filter Element 100 (ERFFEL100) */
#  define S32K3XX_CAN_ERFFEL101_OFFSET       (0x3194) /* Enhanced Rx FIFO Filter Element 101 (ERFFEL101) */
#  define S32K3XX_CAN_ERFFEL102_OFFSET       (0x3198) /* Enhanced Rx FIFO Filter Element 102 (ERFFEL102) */
#  define S32K3XX_CAN_ERFFEL103_OFFSET       (0x319c) /* Enhanced Rx FIFO Filter Element 103 (ERFFEL103) */
#  define S32K3XX_CAN_ERFFEL104_OFFSET       (0x31a0) /* Enhanced Rx FIFO Filter Element 104 (ERFFEL104) */
#  define S32K3XX_CAN_ERFFEL105_OFFSET       (0x31a4) /* Enhanced Rx FIFO Filter Element 105 (ERFFEL105) */
#  define S32K3XX_CAN_ERFFEL106_OFFSET       (0x31a8) /* Enhanced Rx FIFO Filter Element 106 (ERFFEL106) */
#  define S32K3XX_CAN_ERFFEL107_OFFSET       (0x31ac) /* Enhanced Rx FIFO Filter Element 107 (ERFFEL107) */
#  define S32K3XX_CAN_ERFFEL108_OFFSET       (0x31b0) /* Enhanced Rx FIFO Filter Element 108 (ERFFEL108) */
#  define S32K3XX_CAN_ERFFEL109_OFFSET       (0x31b4) /* Enhanced Rx FIFO Filter Element 109 (ERFFEL109) */
#  define S32K3XX_CAN_ERFFEL110_OFFSET       (0x31b8) /* Enhanced Rx FIFO Filter Element 110 (ERFFEL110) */
#  define S32K3XX_CAN_ERFFEL111_OFFSET       (0x31bc) /* Enhanced Rx FIFO Filter Element 111 (ERFFEL111) */
#  define S32K3XX_CAN_ERFFEL112_OFFSET       (0x31c0) /* Enhanced Rx FIFO Filter Element 112 (ERFFEL112) */
#  define S32K3XX_CAN_ERFFEL113_OFFSET       (0x31c4) /* Enhanced Rx FIFO Filter Element 113 (ERFFEL113) */
#  define S32K3XX_CAN_ERFFEL114_OFFSET       (0x31c8) /* Enhanced Rx FIFO Filter Element 114 (ERFFEL114) */
#  define S32K3XX_CAN_ERFFEL115_OFFSET       (0x31cc) /* Enhanced Rx FIFO Filter Element 115 (ERFFEL115) */
#  define S32K3XX_CAN_ERFFEL116_OFFSET       (0x31d0) /* Enhanced Rx FIFO Filter Element 116 (ERFFEL116) */
#  define S32K3XX_CAN_ERFFEL117_OFFSET       (0x31d4) /* Enhanced Rx FIFO Filter Element 117 (ERFFEL117) */
#  define S32K3XX_CAN_ERFFEL118_OFFSET       (0x31d8) /* Enhanced Rx FIFO Filter Element 118 (ERFFEL118) */
#  define S32K3XX_CAN_ERFFEL119_OFFSET       (0x31dc) /* Enhanced Rx FIFO Filter Element 119 (ERFFEL119) */
#  define S32K3XX_CAN_ERFFEL120_OFFSET       (0x31e0) /* Enhanced Rx FIFO Filter Element 120 (ERFFEL120) */
#  define S32K3XX_CAN_ERFFEL121_OFFSET       (0x31e4) /* Enhanced Rx FIFO Filter Element 121 (ERFFEL121) */
#  define S32K3XX_CAN_ERFFEL122_OFFSET       (0x31e8) /* Enhanced Rx FIFO Filter Element 122 (ERFFEL122) */
#  define S32K3XX_CAN_ERFFEL123_OFFSET       (0x31ec) /* Enhanced Rx FIFO Filter Element 123 (ERFFEL123) */
#  define S32K3XX_CAN_ERFFEL124_OFFSET       (0x31f0) /* Enhanced Rx FIFO Filter Element 124 (ERFFEL124) */
#  define S32K3XX_CAN_ERFFEL125_OFFSET       (0x31f4) /* Enhanced Rx FIFO Filter Element 125 (ERFFEL125) */
#  define S32K3XX_CAN_ERFFEL126_OFFSET       (0x31f8) /* Enhanced Rx FIFO Filter Element 126 (ERFFEL126) */
#  define S32K3XX_CAN_ERFFEL127_OFFSET       (0x31fc) /* Enhanced Rx FIFO Filter Element 127 (ERFFEL127) */
#define S32K3XX_CAN_ERFFEL_COUNT             (128)    /* Enhanced Rx FIFO Filter Element Count */

/* FlexCAN Register Addresses ***********************************************/

/* FlexCAN0 */

#define S32K3XX_CAN0_MCR                     (S32K3XX_CAN0_BASE + S32K3XX_CAN_MCR_OFFSET)
#define S32K3XX_CAN0_CTRL1                   (S32K3XX_CAN0_BASE + S32K3XX_CAN_CTRL1_OFFSET)
#define S32K3XX_CAN0_TIMER                   (S32K3XX_CAN0_BASE + S32K3XX_CAN_TIMER_OFFSET)
#define S32K3XX_CAN0_RXMGMASK                (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXMGMASK_OFFSET)
#define S32K3XX_CAN0_RX14MASK                (S32K3XX_CAN0_BASE + S32K3XX_CAN_RX14MASK_OFFSET)
#define S32K3XX_CAN0_RX15MASK                (S32K3XX_CAN0_BASE + S32K3XX_CAN_RX15MASK_OFFSET)
#define S32K3XX_CAN0_ECR                     (S32K3XX_CAN0_BASE + S32K3XX_CAN_ECR_OFFSET)
#define S32K3XX_CAN0_ESR1                    (S32K3XX_CAN0_BASE + S32K3XX_CAN_ESR1_OFFSET)
#define S32K3XX_CAN0_IMASK2                  (S32K3XX_CAN0_BASE + S32K3XX_CAN_IMASK2_OFFSET)
#define S32K3XX_CAN0_IMASK1                  (S32K3XX_CAN0_BASE + S32K3XX_CAN_IMASK1_OFFSET)
#define S32K3XX_CAN0_IFLAG2                  (S32K3XX_CAN0_BASE + S32K3XX_CAN_IFLAG2_OFFSET)
#define S32K3XX_CAN0_IFLAG1                  (S32K3XX_CAN0_BASE + S32K3XX_CAN_IFLAG1_OFFSET)
#define S32K3XX_CAN0_CTRL2                   (S32K3XX_CAN0_BASE + S32K3XX_CAN_CTRL2_OFFSET)
#define S32K3XX_CAN0_ESR2                    (S32K3XX_CAN0_BASE + S32K3XX_CAN_ESR2_OFFSET)
#define S32K3XX_CAN0_CRCR                    (S32K3XX_CAN0_BASE + S32K3XX_CAN_CRCR_OFFSET)
#define S32K3XX_CAN0_RXFGMASK                (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXFGMASK_OFFSET)
#define S32K3XX_CAN0_RXFIR                   (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXFIR_OFFSET)
#define S32K3XX_CAN0_CBT                     (S32K3XX_CAN0_BASE + S32K3XX_CAN_CBT_OFFSET)
#define S32K3XX_CAN0_IMASK3                  (S32K3XX_CAN0_BASE + S32K3XX_CAN_IMASK3_OFFSET)
#define S32K3XX_CAN0_IFLAG3                  (S32K3XX_CAN0_BASE + S32K3XX_CAN_IFLAG3_OFFSET)

#define S32K3XX_CAN0_RXIMR(n)                (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR_OFFSET(n))
#  define S32K3XX_CAN0_RXIMR0                (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR0_OFFSET)
#  define S32K3XX_CAN0_RXIMR1                (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR1_OFFSET)
#  define S32K3XX_CAN0_RXIMR2                (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR2_OFFSET)
#  define S32K3XX_CAN0_RXIMR3                (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR3_OFFSET)
#  define S32K3XX_CAN0_RXIMR4                (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR4_OFFSET)
#  define S32K3XX_CAN0_RXIMR5                (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR5_OFFSET)
#  define S32K3XX_CAN0_RXIMR6                (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR6_OFFSET)
#  define S32K3XX_CAN0_RXIMR7                (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR7_OFFSET)
#  define S32K3XX_CAN0_RXIMR8                (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR8_OFFSET)
#  define S32K3XX_CAN0_RXIMR9                (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR9_OFFSET)
#  define S32K3XX_CAN0_RXIMR10               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR10_OFFSET)
#  define S32K3XX_CAN0_RXIMR11               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR11_OFFSET)
#  define S32K3XX_CAN0_RXIMR12               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR12_OFFSET)
#  define S32K3XX_CAN0_RXIMR13               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR13_OFFSET)
#  define S32K3XX_CAN0_RXIMR14               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR14_OFFSET)
#  define S32K3XX_CAN0_RXIMR15               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR15_OFFSET)
#  define S32K3XX_CAN0_RXIMR16               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR16_OFFSET)
#  define S32K3XX_CAN0_RXIMR17               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR17_OFFSET)
#  define S32K3XX_CAN0_RXIMR18               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR18_OFFSET)
#  define S32K3XX_CAN0_RXIMR19               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR19_OFFSET)
#  define S32K3XX_CAN0_RXIMR20               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR20_OFFSET)
#  define S32K3XX_CAN0_RXIMR21               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR21_OFFSET)
#  define S32K3XX_CAN0_RXIMR22               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR22_OFFSET)
#  define S32K3XX_CAN0_RXIMR23               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR23_OFFSET)
#  define S32K3XX_CAN0_RXIMR24               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR24_OFFSET)
#  define S32K3XX_CAN0_RXIMR25               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR25_OFFSET)
#  define S32K3XX_CAN0_RXIMR26               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR26_OFFSET)
#  define S32K3XX_CAN0_RXIMR27               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR27_OFFSET)
#  define S32K3XX_CAN0_RXIMR28               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR28_OFFSET)
#  define S32K3XX_CAN0_RXIMR29               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR29_OFFSET)
#  define S32K3XX_CAN0_RXIMR30               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR30_OFFSET)
#  define S32K3XX_CAN0_RXIMR31               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR31_OFFSET)
#  define S32K3XX_CAN0_RXIMR32               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR32_OFFSET)
#  define S32K3XX_CAN0_RXIMR33               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR33_OFFSET)
#  define S32K3XX_CAN0_RXIMR34               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR34_OFFSET)
#  define S32K3XX_CAN0_RXIMR35               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR35_OFFSET)
#  define S32K3XX_CAN0_RXIMR36               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR36_OFFSET)
#  define S32K3XX_CAN0_RXIMR37               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR37_OFFSET)
#  define S32K3XX_CAN0_RXIMR38               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR38_OFFSET)
#  define S32K3XX_CAN0_RXIMR39               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR39_OFFSET)
#  define S32K3XX_CAN0_RXIMR40               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR40_OFFSET)
#  define S32K3XX_CAN0_RXIMR41               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR41_OFFSET)
#  define S32K3XX_CAN0_RXIMR42               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR42_OFFSET)
#  define S32K3XX_CAN0_RXIMR43               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR43_OFFSET)
#  define S32K3XX_CAN0_RXIMR44               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR44_OFFSET)
#  define S32K3XX_CAN0_RXIMR45               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR45_OFFSET)
#  define S32K3XX_CAN0_RXIMR46               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR46_OFFSET)
#  define S32K3XX_CAN0_RXIMR47               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR47_OFFSET)
#  define S32K3XX_CAN0_RXIMR48               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR48_OFFSET)
#  define S32K3XX_CAN0_RXIMR49               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR49_OFFSET)
#  define S32K3XX_CAN0_RXIMR50               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR50_OFFSET)
#  define S32K3XX_CAN0_RXIMR51               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR51_OFFSET)
#  define S32K3XX_CAN0_RXIMR52               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR52_OFFSET)
#  define S32K3XX_CAN0_RXIMR53               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR53_OFFSET)
#  define S32K3XX_CAN0_RXIMR54               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR54_OFFSET)
#  define S32K3XX_CAN0_RXIMR55               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR55_OFFSET)
#  define S32K3XX_CAN0_RXIMR56               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR56_OFFSET)
#  define S32K3XX_CAN0_RXIMR57               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR57_OFFSET)
#  define S32K3XX_CAN0_RXIMR58               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR58_OFFSET)
#  define S32K3XX_CAN0_RXIMR59               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR59_OFFSET)
#  define S32K3XX_CAN0_RXIMR60               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR60_OFFSET)
#  define S32K3XX_CAN0_RXIMR61               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR61_OFFSET)
#  define S32K3XX_CAN0_RXIMR62               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR62_OFFSET)
#  define S32K3XX_CAN0_RXIMR63               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR63_OFFSET)
#  define S32K3XX_CAN0_RXIMR64               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR64_OFFSET)
#  define S32K3XX_CAN0_RXIMR65               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR65_OFFSET)
#  define S32K3XX_CAN0_RXIMR66               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR66_OFFSET)
#  define S32K3XX_CAN0_RXIMR67               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR67_OFFSET)
#  define S32K3XX_CAN0_RXIMR68               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR68_OFFSET)
#  define S32K3XX_CAN0_RXIMR69               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR69_OFFSET)
#  define S32K3XX_CAN0_RXIMR70               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR70_OFFSET)
#  define S32K3XX_CAN0_RXIMR71               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR71_OFFSET)
#  define S32K3XX_CAN0_RXIMR72               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR72_OFFSET)
#  define S32K3XX_CAN0_RXIMR73               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR73_OFFSET)
#  define S32K3XX_CAN0_RXIMR74               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR74_OFFSET)
#  define S32K3XX_CAN0_RXIMR75               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR75_OFFSET)
#  define S32K3XX_CAN0_RXIMR76               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR76_OFFSET)
#  define S32K3XX_CAN0_RXIMR77               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR77_OFFSET)
#  define S32K3XX_CAN0_RXIMR78               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR78_OFFSET)
#  define S32K3XX_CAN0_RXIMR79               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR79_OFFSET)
#  define S32K3XX_CAN0_RXIMR80               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR80_OFFSET)
#  define S32K3XX_CAN0_RXIMR81               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR81_OFFSET)
#  define S32K3XX_CAN0_RXIMR82               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR82_OFFSET)
#  define S32K3XX_CAN0_RXIMR83               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR83_OFFSET)
#  define S32K3XX_CAN0_RXIMR84               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR84_OFFSET)
#  define S32K3XX_CAN0_RXIMR85               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR85_OFFSET)
#  define S32K3XX_CAN0_RXIMR86               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR86_OFFSET)
#  define S32K3XX_CAN0_RXIMR87               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR87_OFFSET)
#  define S32K3XX_CAN0_RXIMR88               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR88_OFFSET)
#  define S32K3XX_CAN0_RXIMR89               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR89_OFFSET)
#  define S32K3XX_CAN0_RXIMR90               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR90_OFFSET)
#  define S32K3XX_CAN0_RXIMR91               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR91_OFFSET)
#  define S32K3XX_CAN0_RXIMR92               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR92_OFFSET)
#  define S32K3XX_CAN0_RXIMR93               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR93_OFFSET)
#  define S32K3XX_CAN0_RXIMR94               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR94_OFFSET)
#  define S32K3XX_CAN0_RXIMR95               (S32K3XX_CAN0_BASE + S32K3XX_CAN_RXIMR95_OFFSET)

#define S32K3XX_CAN0_MECR                    (S32K3XX_CAN0_BASE + S32K3XX_CAN_MECR_OFFSET)
#define S32K3XX_CAN0_ERRIAR                  (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERRIAR_OFFSET)
#define S32K3XX_CAN0_ERRIDPR                 (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERRIDPR_OFFSET)
#define S32K3XX_CAN0_ERRIPPR                 (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERRIPPR_OFFSET)
#define S32K3XX_CAN0_RERRAR                  (S32K3XX_CAN0_BASE + S32K3XX_CAN_RERRAR_OFFSET)
#define S32K3XX_CAN0_RERRDR                  (S32K3XX_CAN0_BASE + S32K3XX_CAN_RERRDR_OFFSET)
#define S32K3XX_CAN0_RERRSYNR                (S32K3XX_CAN0_BASE + S32K3XX_CAN_RERRSYNR_OFFSET)
#define S32K3XX_CAN0_ERRSR                   (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERRSR_OFFSET)
#define S32K3XX_CAN0_EPRS                    (S32K3XX_CAN0_BASE + S32K3XX_CAN_EPRS_OFFSET)
#define S32K3XX_CAN0_ENCBT                   (S32K3XX_CAN0_BASE + S32K3XX_CAN_ENCBT_OFFSET)
#define S32K3XX_CAN0_EDCBT                   (S32K3XX_CAN0_BASE + S32K3XX_CAN_EDCBT_OFFSET)
#define S32K3XX_CAN0_ETDC                    (S32K3XX_CAN0_BASE + S32K3XX_CAN_ETDC_OFFSET)
#define S32K3XX_CAN0_FDCTRL                  (S32K3XX_CAN0_BASE + S32K3XX_CAN_FDCTRL_OFFSET)
#define S32K3XX_CAN0_FDCBT                   (S32K3XX_CAN0_BASE + S32K3XX_CAN_FDCBT_OFFSET)
#define S32K3XX_CAN0_FDCRC                   (S32K3XX_CAN0_BASE + S32K3XX_CAN_FDCRC_OFFSET)
#define S32K3XX_CAN0_ERFCR                   (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFCR_OFFSET)
#define S32K3XX_CAN0_ERFIER                  (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFIER_OFFSET)
#define S32K3XX_CAN0_ERFSR                   (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFSR_OFFSET)

#define S32K3XX_CAN0_HR_TIME_STAMP(n)        (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP_OFFSET(n))  
#  define S32K3XX_CAN0_HR_TIME_STAMP0        (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP0_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP1        (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP1_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP2        (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP2_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP3        (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP3_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP4        (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP4_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP5        (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP5_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP6        (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP6_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP7        (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP7_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP8        (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP8_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP9        (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP9_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP10       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP10_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP11       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP11_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP12       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP12_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP13       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP13_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP14       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP14_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP15       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP15_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP16       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP16_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP17       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP17_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP18       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP18_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP19       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP19_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP20       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP20_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP21       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP21_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP22       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP22_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP23       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP23_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP24       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP24_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP25       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP25_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP26       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP26_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP27       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP27_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP28       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP28_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP29       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP29_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP30       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP30_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP31       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP31_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP32       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP32_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP33       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP33_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP34       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP34_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP35       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP35_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP36       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP36_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP37       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP37_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP38       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP38_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP39       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP39_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP40       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP40_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP41       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP41_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP42       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP42_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP43       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP43_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP44       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP44_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP45       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP45_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP46       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP46_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP47       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP47_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP48       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP48_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP49       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP49_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP50       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP50_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP51       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP51_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP52       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP52_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP53       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP53_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP54       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP54_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP55       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP55_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP56       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP56_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP57       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP57_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP58       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP58_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP59       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP59_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP60       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP60_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP61       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP61_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP62       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP62_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP63       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP63_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP64       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP64_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP65       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP65_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP66       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP66_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP67       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP67_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP68       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP68_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP69       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP69_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP70       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP70_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP71       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP71_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP72       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP72_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP73       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP73_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP74       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP74_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP75       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP75_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP76       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP76_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP77       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP77_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP78       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP78_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP79       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP79_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP80       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP80_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP81       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP81_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP82       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP82_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP83       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP83_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP84       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP84_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP85       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP85_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP86       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP86_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP87       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP87_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP88       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP88_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP89       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP89_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP90       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP90_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP91       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP91_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP92       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP92_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP93       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP93_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP94       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP94_OFFSET)
#  define S32K3XX_CAN0_HR_TIME_STAMP95       (S32K3XX_CAN0_BASE + S32K3XX_CAN_HR_TIME_STAMP95_OFFSET)

#define S32K3XX_CAN0_ERFFEL(n)               (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL_OFFSET(n))
#  define S32K3XX_CAN0_ERFFEL0               (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL0_OFFSET)
#  define S32K3XX_CAN0_ERFFEL1               (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL1_OFFSET)
#  define S32K3XX_CAN0_ERFFEL2               (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL2_OFFSET)
#  define S32K3XX_CAN0_ERFFEL3               (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL3_OFFSET)
#  define S32K3XX_CAN0_ERFFEL4               (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL4_OFFSET)
#  define S32K3XX_CAN0_ERFFEL5               (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL5_OFFSET)
#  define S32K3XX_CAN0_ERFFEL6               (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL6_OFFSET)
#  define S32K3XX_CAN0_ERFFEL7               (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL7_OFFSET)
#  define S32K3XX_CAN0_ERFFEL8               (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL8_OFFSET)
#  define S32K3XX_CAN0_ERFFEL9               (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL9_OFFSET)
#  define S32K3XX_CAN0_ERFFEL10              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL10_OFFSET)
#  define S32K3XX_CAN0_ERFFEL11              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL11_OFFSET)
#  define S32K3XX_CAN0_ERFFEL12              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL12_OFFSET)
#  define S32K3XX_CAN0_ERFFEL13              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL13_OFFSET)
#  define S32K3XX_CAN0_ERFFEL14              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL14_OFFSET)
#  define S32K3XX_CAN0_ERFFEL15              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL15_OFFSET)
#  define S32K3XX_CAN0_ERFFEL16              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL16_OFFSET)
#  define S32K3XX_CAN0_ERFFEL17              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL17_OFFSET)
#  define S32K3XX_CAN0_ERFFEL18              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL18_OFFSET)
#  define S32K3XX_CAN0_ERFFEL19              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL19_OFFSET)
#  define S32K3XX_CAN0_ERFFEL20              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL20_OFFSET)
#  define S32K3XX_CAN0_ERFFEL21              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL21_OFFSET)
#  define S32K3XX_CAN0_ERFFEL22              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL22_OFFSET)
#  define S32K3XX_CAN0_ERFFEL23              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL23_OFFSET)
#  define S32K3XX_CAN0_ERFFEL24              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL24_OFFSET)
#  define S32K3XX_CAN0_ERFFEL25              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL25_OFFSET)
#  define S32K3XX_CAN0_ERFFEL26              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL26_OFFSET)
#  define S32K3XX_CAN0_ERFFEL27              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL27_OFFSET)
#  define S32K3XX_CAN0_ERFFEL28              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL28_OFFSET)
#  define S32K3XX_CAN0_ERFFEL29              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL29_OFFSET)
#  define S32K3XX_CAN0_ERFFEL30              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL30_OFFSET)
#  define S32K3XX_CAN0_ERFFEL31              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL31_OFFSET)
#  define S32K3XX_CAN0_ERFFEL32              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL32_OFFSET)
#  define S32K3XX_CAN0_ERFFEL33              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL33_OFFSET)
#  define S32K3XX_CAN0_ERFFEL34              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL34_OFFSET)
#  define S32K3XX_CAN0_ERFFEL35              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL35_OFFSET)
#  define S32K3XX_CAN0_ERFFEL36              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL36_OFFSET)
#  define S32K3XX_CAN0_ERFFEL37              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL37_OFFSET)
#  define S32K3XX_CAN0_ERFFEL38              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL38_OFFSET)
#  define S32K3XX_CAN0_ERFFEL39              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL39_OFFSET)
#  define S32K3XX_CAN0_ERFFEL40              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL40_OFFSET)
#  define S32K3XX_CAN0_ERFFEL41              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL41_OFFSET)
#  define S32K3XX_CAN0_ERFFEL42              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL42_OFFSET)
#  define S32K3XX_CAN0_ERFFEL43              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL43_OFFSET)
#  define S32K3XX_CAN0_ERFFEL44              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL44_OFFSET)
#  define S32K3XX_CAN0_ERFFEL45              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL45_OFFSET)
#  define S32K3XX_CAN0_ERFFEL46              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL46_OFFSET)
#  define S32K3XX_CAN0_ERFFEL47              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL47_OFFSET)
#  define S32K3XX_CAN0_ERFFEL48              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL48_OFFSET)
#  define S32K3XX_CAN0_ERFFEL49              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL49_OFFSET)
#  define S32K3XX_CAN0_ERFFEL50              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL50_OFFSET)
#  define S32K3XX_CAN0_ERFFEL51              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL51_OFFSET)
#  define S32K3XX_CAN0_ERFFEL52              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL52_OFFSET)
#  define S32K3XX_CAN0_ERFFEL53              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL53_OFFSET)
#  define S32K3XX_CAN0_ERFFEL54              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL54_OFFSET)
#  define S32K3XX_CAN0_ERFFEL55              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL55_OFFSET)
#  define S32K3XX_CAN0_ERFFEL56              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL56_OFFSET)
#  define S32K3XX_CAN0_ERFFEL57              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL57_OFFSET)
#  define S32K3XX_CAN0_ERFFEL58              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL58_OFFSET)
#  define S32K3XX_CAN0_ERFFEL59              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL59_OFFSET)
#  define S32K3XX_CAN0_ERFFEL60              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL60_OFFSET)
#  define S32K3XX_CAN0_ERFFEL61              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL61_OFFSET)
#  define S32K3XX_CAN0_ERFFEL62              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL62_OFFSET)
#  define S32K3XX_CAN0_ERFFEL63              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL63_OFFSET)
#  define S32K3XX_CAN0_ERFFEL64              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL64_OFFSET)
#  define S32K3XX_CAN0_ERFFEL65              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL65_OFFSET)
#  define S32K3XX_CAN0_ERFFEL66              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL66_OFFSET)
#  define S32K3XX_CAN0_ERFFEL67              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL67_OFFSET)
#  define S32K3XX_CAN0_ERFFEL68              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL68_OFFSET)
#  define S32K3XX_CAN0_ERFFEL69              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL69_OFFSET)
#  define S32K3XX_CAN0_ERFFEL70              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL70_OFFSET)
#  define S32K3XX_CAN0_ERFFEL71              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL71_OFFSET)
#  define S32K3XX_CAN0_ERFFEL72              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL72_OFFSET)
#  define S32K3XX_CAN0_ERFFEL73              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL73_OFFSET)
#  define S32K3XX_CAN0_ERFFEL74              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL74_OFFSET)
#  define S32K3XX_CAN0_ERFFEL75              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL75_OFFSET)
#  define S32K3XX_CAN0_ERFFEL76              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL76_OFFSET)
#  define S32K3XX_CAN0_ERFFEL77              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL77_OFFSET)
#  define S32K3XX_CAN0_ERFFEL78              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL78_OFFSET)
#  define S32K3XX_CAN0_ERFFEL79              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL79_OFFSET)
#  define S32K3XX_CAN0_ERFFEL80              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL80_OFFSET)
#  define S32K3XX_CAN0_ERFFEL81              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL81_OFFSET)
#  define S32K3XX_CAN0_ERFFEL82              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL82_OFFSET)
#  define S32K3XX_CAN0_ERFFEL83              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL83_OFFSET)
#  define S32K3XX_CAN0_ERFFEL84              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL84_OFFSET)
#  define S32K3XX_CAN0_ERFFEL85              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL85_OFFSET)
#  define S32K3XX_CAN0_ERFFEL86              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL86_OFFSET)
#  define S32K3XX_CAN0_ERFFEL87              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL87_OFFSET)
#  define S32K3XX_CAN0_ERFFEL88              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL88_OFFSET)
#  define S32K3XX_CAN0_ERFFEL89              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL89_OFFSET)
#  define S32K3XX_CAN0_ERFFEL90              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL90_OFFSET)
#  define S32K3XX_CAN0_ERFFEL91              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL91_OFFSET)
#  define S32K3XX_CAN0_ERFFEL92              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL92_OFFSET)
#  define S32K3XX_CAN0_ERFFEL93              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL93_OFFSET)
#  define S32K3XX_CAN0_ERFFEL94              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL94_OFFSET)
#  define S32K3XX_CAN0_ERFFEL95              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL95_OFFSET)
#  define S32K3XX_CAN0_ERFFEL96              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL96_OFFSET)
#  define S32K3XX_CAN0_ERFFEL97              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL97_OFFSET)
#  define S32K3XX_CAN0_ERFFEL98              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL98_OFFSET)
#  define S32K3XX_CAN0_ERFFEL99              (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL99_OFFSET)
#  define S32K3XX_CAN0_ERFFEL100             (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL100_OFFSET)
#  define S32K3XX_CAN0_ERFFEL101             (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL101_OFFSET)
#  define S32K3XX_CAN0_ERFFEL102             (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL102_OFFSET)
#  define S32K3XX_CAN0_ERFFEL103             (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL103_OFFSET)
#  define S32K3XX_CAN0_ERFFEL104             (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL104_OFFSET)
#  define S32K3XX_CAN0_ERFFEL105             (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL105_OFFSET)
#  define S32K3XX_CAN0_ERFFEL106             (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL106_OFFSET)
#  define S32K3XX_CAN0_ERFFEL107             (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL107_OFFSET)
#  define S32K3XX_CAN0_ERFFEL108             (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL108_OFFSET)
#  define S32K3XX_CAN0_ERFFEL109             (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL109_OFFSET)
#  define S32K3XX_CAN0_ERFFEL110             (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL110_OFFSET)
#  define S32K3XX_CAN0_ERFFEL111             (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL111_OFFSET)
#  define S32K3XX_CAN0_ERFFEL112             (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL112_OFFSET)
#  define S32K3XX_CAN0_ERFFEL113             (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL113_OFFSET)
#  define S32K3XX_CAN0_ERFFEL114             (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL114_OFFSET)
#  define S32K3XX_CAN0_ERFFEL115             (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL115_OFFSET)
#  define S32K3XX_CAN0_ERFFEL116             (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL116_OFFSET)
#  define S32K3XX_CAN0_ERFFEL117             (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL117_OFFSET)
#  define S32K3XX_CAN0_ERFFEL118             (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL118_OFFSET)
#  define S32K3XX_CAN0_ERFFEL119             (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL119_OFFSET)
#  define S32K3XX_CAN0_ERFFEL120             (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL120_OFFSET)
#  define S32K3XX_CAN0_ERFFEL121             (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL121_OFFSET)
#  define S32K3XX_CAN0_ERFFEL122             (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL122_OFFSET)
#  define S32K3XX_CAN0_ERFFEL123             (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL123_OFFSET)
#  define S32K3XX_CAN0_ERFFEL124             (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL124_OFFSET)
#  define S32K3XX_CAN0_ERFFEL125             (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL125_OFFSET)
#  define S32K3XX_CAN0_ERFFEL126             (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL126_OFFSET)
#  define S32K3XX_CAN0_ERFFEL127             (S32K3XX_CAN0_BASE + S32K3XX_CAN_ERFFEL127_OFFSET)

/* FlexCAN1 */

#define S32K3XX_CAN1_MCR                     (S32K3XX_CAN1_BASE + S32K3XX_CAN_MCR_OFFSET)
#define S32K3XX_CAN1_CTRL1                   (S32K3XX_CAN1_BASE + S32K3XX_CAN_CTRL1_OFFSET)
#define S32K3XX_CAN1_TIMER                   (S32K3XX_CAN1_BASE + S32K3XX_CAN_TIMER_OFFSET)
#define S32K3XX_CAN1_RXMGMASK                (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXMGMASK_OFFSET)
#define S32K3XX_CAN1_RX14MASK                (S32K3XX_CAN1_BASE + S32K3XX_CAN_RX14MASK_OFFSET)
#define S32K3XX_CAN1_RX15MASK                (S32K3XX_CAN1_BASE + S32K3XX_CAN_RX15MASK_OFFSET)
#define S32K3XX_CAN1_ECR                     (S32K3XX_CAN1_BASE + S32K3XX_CAN_ECR_OFFSET)
#define S32K3XX_CAN1_ESR1                    (S32K3XX_CAN1_BASE + S32K3XX_CAN_ESR1_OFFSET)
#define S32K3XX_CAN1_IMASK2                  (S32K3XX_CAN1_BASE + S32K3XX_CAN_IMASK2_OFFSET)
#define S32K3XX_CAN1_IMASK1                  (S32K3XX_CAN1_BASE + S32K3XX_CAN_IMASK1_OFFSET)
#define S32K3XX_CAN1_IFLAG2                  (S32K3XX_CAN1_BASE + S32K3XX_CAN_IFLAG2_OFFSET)
#define S32K3XX_CAN1_IFLAG1                  (S32K3XX_CAN1_BASE + S32K3XX_CAN_IFLAG1_OFFSET)
#define S32K3XX_CAN1_CTRL2                   (S32K3XX_CAN1_BASE + S32K3XX_CAN_CTRL2_OFFSET)
#define S32K3XX_CAN1_ESR2                    (S32K3XX_CAN1_BASE + S32K3XX_CAN_ESR2_OFFSET)
#define S32K3XX_CAN1_CRCR                    (S32K3XX_CAN1_BASE + S32K3XX_CAN_CRCR_OFFSET)
#define S32K3XX_CAN1_RXFGMASK                (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXFGMASK_OFFSET)
#define S32K3XX_CAN1_RXFIR                   (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXFIR_OFFSET)
#define S32K3XX_CAN1_CBT                     (S32K3XX_CAN1_BASE + S32K3XX_CAN_CBT_OFFSET)
#define S32K3XX_CAN1_IMASK3                  (S32K3XX_CAN1_BASE + S32K3XX_CAN_IMASK3_OFFSET)
#define S32K3XX_CAN1_IFLAG3                  (S32K3XX_CAN1_BASE + S32K3XX_CAN_IFLAG3_OFFSET)

#define S32K3XX_CAN1_RXIMR(n)                (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR_OFFSET(n))
#  define S32K3XX_CAN1_RXIMR0                (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR0_OFFSET)
#  define S32K3XX_CAN1_RXIMR1                (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR1_OFFSET)
#  define S32K3XX_CAN1_RXIMR2                (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR2_OFFSET)
#  define S32K3XX_CAN1_RXIMR3                (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR3_OFFSET)
#  define S32K3XX_CAN1_RXIMR4                (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR4_OFFSET)
#  define S32K3XX_CAN1_RXIMR5                (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR5_OFFSET)
#  define S32K3XX_CAN1_RXIMR6                (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR6_OFFSET)
#  define S32K3XX_CAN1_RXIMR7                (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR7_OFFSET)
#  define S32K3XX_CAN1_RXIMR8                (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR8_OFFSET)
#  define S32K3XX_CAN1_RXIMR9                (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR9_OFFSET)
#  define S32K3XX_CAN1_RXIMR10               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR10_OFFSET)
#  define S32K3XX_CAN1_RXIMR11               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR11_OFFSET)
#  define S32K3XX_CAN1_RXIMR12               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR12_OFFSET)
#  define S32K3XX_CAN1_RXIMR13               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR13_OFFSET)
#  define S32K3XX_CAN1_RXIMR14               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR14_OFFSET)
#  define S32K3XX_CAN1_RXIMR15               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR15_OFFSET)
#  define S32K3XX_CAN1_RXIMR16               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR16_OFFSET)
#  define S32K3XX_CAN1_RXIMR17               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR17_OFFSET)
#  define S32K3XX_CAN1_RXIMR18               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR18_OFFSET)
#  define S32K3XX_CAN1_RXIMR19               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR19_OFFSET)
#  define S32K3XX_CAN1_RXIMR20               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR20_OFFSET)
#  define S32K3XX_CAN1_RXIMR21               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR21_OFFSET)
#  define S32K3XX_CAN1_RXIMR22               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR22_OFFSET)
#  define S32K3XX_CAN1_RXIMR23               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR23_OFFSET)
#  define S32K3XX_CAN1_RXIMR24               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR24_OFFSET)
#  define S32K3XX_CAN1_RXIMR25               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR25_OFFSET)
#  define S32K3XX_CAN1_RXIMR26               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR26_OFFSET)
#  define S32K3XX_CAN1_RXIMR27               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR27_OFFSET)
#  define S32K3XX_CAN1_RXIMR28               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR28_OFFSET)
#  define S32K3XX_CAN1_RXIMR29               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR29_OFFSET)
#  define S32K3XX_CAN1_RXIMR30               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR30_OFFSET)
#  define S32K3XX_CAN1_RXIMR31               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR31_OFFSET)
#  define S32K3XX_CAN1_RXIMR32               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR32_OFFSET)
#  define S32K3XX_CAN1_RXIMR33               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR33_OFFSET)
#  define S32K3XX_CAN1_RXIMR34               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR34_OFFSET)
#  define S32K3XX_CAN1_RXIMR35               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR35_OFFSET)
#  define S32K3XX_CAN1_RXIMR36               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR36_OFFSET)
#  define S32K3XX_CAN1_RXIMR37               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR37_OFFSET)
#  define S32K3XX_CAN1_RXIMR38               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR38_OFFSET)
#  define S32K3XX_CAN1_RXIMR39               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR39_OFFSET)
#  define S32K3XX_CAN1_RXIMR40               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR40_OFFSET)
#  define S32K3XX_CAN1_RXIMR41               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR41_OFFSET)
#  define S32K3XX_CAN1_RXIMR42               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR42_OFFSET)
#  define S32K3XX_CAN1_RXIMR43               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR43_OFFSET)
#  define S32K3XX_CAN1_RXIMR44               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR44_OFFSET)
#  define S32K3XX_CAN1_RXIMR45               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR45_OFFSET)
#  define S32K3XX_CAN1_RXIMR46               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR46_OFFSET)
#  define S32K3XX_CAN1_RXIMR47               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR47_OFFSET)
#  define S32K3XX_CAN1_RXIMR48               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR48_OFFSET)
#  define S32K3XX_CAN1_RXIMR49               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR49_OFFSET)
#  define S32K3XX_CAN1_RXIMR50               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR50_OFFSET)
#  define S32K3XX_CAN1_RXIMR51               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR51_OFFSET)
#  define S32K3XX_CAN1_RXIMR52               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR52_OFFSET)
#  define S32K3XX_CAN1_RXIMR53               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR53_OFFSET)
#  define S32K3XX_CAN1_RXIMR54               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR54_OFFSET)
#  define S32K3XX_CAN1_RXIMR55               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR55_OFFSET)
#  define S32K3XX_CAN1_RXIMR56               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR56_OFFSET)
#  define S32K3XX_CAN1_RXIMR57               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR57_OFFSET)
#  define S32K3XX_CAN1_RXIMR58               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR58_OFFSET)
#  define S32K3XX_CAN1_RXIMR59               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR59_OFFSET)
#  define S32K3XX_CAN1_RXIMR60               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR60_OFFSET)
#  define S32K3XX_CAN1_RXIMR61               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR61_OFFSET)
#  define S32K3XX_CAN1_RXIMR62               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR62_OFFSET)
#  define S32K3XX_CAN1_RXIMR63               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR63_OFFSET)
#  define S32K3XX_CAN1_RXIMR64               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR64_OFFSET)
#  define S32K3XX_CAN1_RXIMR65               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR65_OFFSET)
#  define S32K3XX_CAN1_RXIMR66               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR66_OFFSET)
#  define S32K3XX_CAN1_RXIMR67               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR67_OFFSET)
#  define S32K3XX_CAN1_RXIMR68               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR68_OFFSET)
#  define S32K3XX_CAN1_RXIMR69               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR69_OFFSET)
#  define S32K3XX_CAN1_RXIMR70               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR70_OFFSET)
#  define S32K3XX_CAN1_RXIMR71               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR71_OFFSET)
#  define S32K3XX_CAN1_RXIMR72               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR72_OFFSET)
#  define S32K3XX_CAN1_RXIMR73               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR73_OFFSET)
#  define S32K3XX_CAN1_RXIMR74               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR74_OFFSET)
#  define S32K3XX_CAN1_RXIMR75               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR75_OFFSET)
#  define S32K3XX_CAN1_RXIMR76               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR76_OFFSET)
#  define S32K3XX_CAN1_RXIMR77               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR77_OFFSET)
#  define S32K3XX_CAN1_RXIMR78               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR78_OFFSET)
#  define S32K3XX_CAN1_RXIMR79               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR79_OFFSET)
#  define S32K3XX_CAN1_RXIMR80               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR80_OFFSET)
#  define S32K3XX_CAN1_RXIMR81               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR81_OFFSET)
#  define S32K3XX_CAN1_RXIMR82               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR82_OFFSET)
#  define S32K3XX_CAN1_RXIMR83               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR83_OFFSET)
#  define S32K3XX_CAN1_RXIMR84               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR84_OFFSET)
#  define S32K3XX_CAN1_RXIMR85               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR85_OFFSET)
#  define S32K3XX_CAN1_RXIMR86               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR86_OFFSET)
#  define S32K3XX_CAN1_RXIMR87               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR87_OFFSET)
#  define S32K3XX_CAN1_RXIMR88               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR88_OFFSET)
#  define S32K3XX_CAN1_RXIMR89               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR89_OFFSET)
#  define S32K3XX_CAN1_RXIMR90               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR90_OFFSET)
#  define S32K3XX_CAN1_RXIMR91               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR91_OFFSET)
#  define S32K3XX_CAN1_RXIMR92               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR92_OFFSET)
#  define S32K3XX_CAN1_RXIMR93               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR93_OFFSET)
#  define S32K3XX_CAN1_RXIMR94               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR94_OFFSET)
#  define S32K3XX_CAN1_RXIMR95               (S32K3XX_CAN1_BASE + S32K3XX_CAN_RXIMR95_OFFSET)

#define S32K3XX_CAN1_MECR                    (S32K3XX_CAN1_BASE + S32K3XX_CAN_MECR_OFFSET)
#define S32K3XX_CAN1_ERRIAR                  (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERRIAR_OFFSET)
#define S32K3XX_CAN1_ERRIDPR                 (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERRIDPR_OFFSET)
#define S32K3XX_CAN1_ERRIPPR                 (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERRIPPR_OFFSET)
#define S32K3XX_CAN1_RERRAR                  (S32K3XX_CAN1_BASE + S32K3XX_CAN_RERRAR_OFFSET)
#define S32K3XX_CAN1_RERRDR                  (S32K3XX_CAN1_BASE + S32K3XX_CAN_RERRDR_OFFSET)
#define S32K3XX_CAN1_RERRSYNR                (S32K3XX_CAN1_BASE + S32K3XX_CAN_RERRSYNR_OFFSET)
#define S32K3XX_CAN1_ERRSR                   (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERRSR_OFFSET)
#define S32K3XX_CAN1_EPRS                    (S32K3XX_CAN1_BASE + S32K3XX_CAN_EPRS_OFFSET)
#define S32K3XX_CAN1_ENCBT                   (S32K3XX_CAN1_BASE + S32K3XX_CAN_ENCBT_OFFSET)
#define S32K3XX_CAN1_EDCBT                   (S32K3XX_CAN1_BASE + S32K3XX_CAN_EDCBT_OFFSET)
#define S32K3XX_CAN1_ETDC                    (S32K3XX_CAN1_BASE + S32K3XX_CAN_ETDC_OFFSET)
#define S32K3XX_CAN1_FDCTRL                  (S32K3XX_CAN1_BASE + S32K3XX_CAN_FDCTRL_OFFSET)
#define S32K3XX_CAN1_FDCBT                   (S32K3XX_CAN1_BASE + S32K3XX_CAN_FDCBT_OFFSET)
#define S32K3XX_CAN1_FDCRC                   (S32K3XX_CAN1_BASE + S32K3XX_CAN_FDCRC_OFFSET)
#define S32K3XX_CAN1_ERFCR                   (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFCR_OFFSET)
#define S32K3XX_CAN1_ERFIER                  (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFIER_OFFSET)
#define S32K3XX_CAN1_ERFSR                   (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFSR_OFFSET)

#define S32K3XX_CAN1_HR_TIME_STAMP(n)        (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP_OFFSET(n))  
#  define S32K3XX_CAN1_HR_TIME_STAMP0        (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP0_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP1        (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP1_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP2        (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP2_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP3        (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP3_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP4        (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP4_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP5        (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP5_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP6        (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP6_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP7        (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP7_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP8        (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP8_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP9        (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP9_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP10       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP10_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP11       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP11_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP12       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP12_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP13       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP13_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP14       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP14_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP15       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP15_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP16       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP16_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP17       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP17_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP18       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP18_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP19       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP19_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP20       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP20_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP21       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP21_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP22       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP22_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP23       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP23_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP24       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP24_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP25       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP25_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP26       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP26_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP27       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP27_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP28       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP28_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP29       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP29_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP30       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP30_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP31       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP31_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP32       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP32_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP33       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP33_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP34       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP34_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP35       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP35_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP36       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP36_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP37       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP37_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP38       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP38_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP39       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP39_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP40       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP40_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP41       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP41_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP42       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP42_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP43       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP43_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP44       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP44_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP45       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP45_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP46       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP46_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP47       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP47_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP48       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP48_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP49       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP49_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP50       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP50_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP51       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP51_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP52       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP52_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP53       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP53_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP54       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP54_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP55       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP55_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP56       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP56_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP57       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP57_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP58       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP58_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP59       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP59_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP60       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP60_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP61       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP61_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP62       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP62_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP63       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP63_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP64       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP64_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP65       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP65_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP66       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP66_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP67       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP67_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP68       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP68_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP69       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP69_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP70       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP70_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP71       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP71_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP72       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP72_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP73       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP73_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP74       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP74_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP75       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP75_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP76       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP76_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP77       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP77_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP78       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP78_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP79       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP79_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP80       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP80_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP81       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP81_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP82       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP82_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP83       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP83_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP84       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP84_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP85       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP85_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP86       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP86_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP87       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP87_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP88       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP88_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP89       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP89_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP90       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP90_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP91       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP91_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP92       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP92_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP93       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP93_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP94       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP94_OFFSET)
#  define S32K3XX_CAN1_HR_TIME_STAMP95       (S32K3XX_CAN1_BASE + S32K3XX_CAN_HR_TIME_STAMP95_OFFSET)

#define S32K3XX_CAN1_ERFFEL(n)               (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL_OFFSET(n))
#  define S32K3XX_CAN1_ERFFEL0               (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL0_OFFSET)
#  define S32K3XX_CAN1_ERFFEL1               (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL1_OFFSET)
#  define S32K3XX_CAN1_ERFFEL2               (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL2_OFFSET)
#  define S32K3XX_CAN1_ERFFEL3               (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL3_OFFSET)
#  define S32K3XX_CAN1_ERFFEL4               (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL4_OFFSET)
#  define S32K3XX_CAN1_ERFFEL5               (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL5_OFFSET)
#  define S32K3XX_CAN1_ERFFEL6               (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL6_OFFSET)
#  define S32K3XX_CAN1_ERFFEL7               (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL7_OFFSET)
#  define S32K3XX_CAN1_ERFFEL8               (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL8_OFFSET)
#  define S32K3XX_CAN1_ERFFEL9               (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL9_OFFSET)
#  define S32K3XX_CAN1_ERFFEL10              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL10_OFFSET)
#  define S32K3XX_CAN1_ERFFEL11              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL11_OFFSET)
#  define S32K3XX_CAN1_ERFFEL12              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL12_OFFSET)
#  define S32K3XX_CAN1_ERFFEL13              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL13_OFFSET)
#  define S32K3XX_CAN1_ERFFEL14              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL14_OFFSET)
#  define S32K3XX_CAN1_ERFFEL15              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL15_OFFSET)
#  define S32K3XX_CAN1_ERFFEL16              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL16_OFFSET)
#  define S32K3XX_CAN1_ERFFEL17              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL17_OFFSET)
#  define S32K3XX_CAN1_ERFFEL18              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL18_OFFSET)
#  define S32K3XX_CAN1_ERFFEL19              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL19_OFFSET)
#  define S32K3XX_CAN1_ERFFEL20              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL20_OFFSET)
#  define S32K3XX_CAN1_ERFFEL21              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL21_OFFSET)
#  define S32K3XX_CAN1_ERFFEL22              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL22_OFFSET)
#  define S32K3XX_CAN1_ERFFEL23              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL23_OFFSET)
#  define S32K3XX_CAN1_ERFFEL24              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL24_OFFSET)
#  define S32K3XX_CAN1_ERFFEL25              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL25_OFFSET)
#  define S32K3XX_CAN1_ERFFEL26              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL26_OFFSET)
#  define S32K3XX_CAN1_ERFFEL27              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL27_OFFSET)
#  define S32K3XX_CAN1_ERFFEL28              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL28_OFFSET)
#  define S32K3XX_CAN1_ERFFEL29              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL29_OFFSET)
#  define S32K3XX_CAN1_ERFFEL30              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL30_OFFSET)
#  define S32K3XX_CAN1_ERFFEL31              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL31_OFFSET)
#  define S32K3XX_CAN1_ERFFEL32              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL32_OFFSET)
#  define S32K3XX_CAN1_ERFFEL33              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL33_OFFSET)
#  define S32K3XX_CAN1_ERFFEL34              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL34_OFFSET)
#  define S32K3XX_CAN1_ERFFEL35              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL35_OFFSET)
#  define S32K3XX_CAN1_ERFFEL36              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL36_OFFSET)
#  define S32K3XX_CAN1_ERFFEL37              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL37_OFFSET)
#  define S32K3XX_CAN1_ERFFEL38              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL38_OFFSET)
#  define S32K3XX_CAN1_ERFFEL39              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL39_OFFSET)
#  define S32K3XX_CAN1_ERFFEL40              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL40_OFFSET)
#  define S32K3XX_CAN1_ERFFEL41              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL41_OFFSET)
#  define S32K3XX_CAN1_ERFFEL42              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL42_OFFSET)
#  define S32K3XX_CAN1_ERFFEL43              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL43_OFFSET)
#  define S32K3XX_CAN1_ERFFEL44              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL44_OFFSET)
#  define S32K3XX_CAN1_ERFFEL45              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL45_OFFSET)
#  define S32K3XX_CAN1_ERFFEL46              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL46_OFFSET)
#  define S32K3XX_CAN1_ERFFEL47              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL47_OFFSET)
#  define S32K3XX_CAN1_ERFFEL48              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL48_OFFSET)
#  define S32K3XX_CAN1_ERFFEL49              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL49_OFFSET)
#  define S32K3XX_CAN1_ERFFEL50              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL50_OFFSET)
#  define S32K3XX_CAN1_ERFFEL51              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL51_OFFSET)
#  define S32K3XX_CAN1_ERFFEL52              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL52_OFFSET)
#  define S32K3XX_CAN1_ERFFEL53              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL53_OFFSET)
#  define S32K3XX_CAN1_ERFFEL54              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL54_OFFSET)
#  define S32K3XX_CAN1_ERFFEL55              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL55_OFFSET)
#  define S32K3XX_CAN1_ERFFEL56              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL56_OFFSET)
#  define S32K3XX_CAN1_ERFFEL57              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL57_OFFSET)
#  define S32K3XX_CAN1_ERFFEL58              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL58_OFFSET)
#  define S32K3XX_CAN1_ERFFEL59              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL59_OFFSET)
#  define S32K3XX_CAN1_ERFFEL60              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL60_OFFSET)
#  define S32K3XX_CAN1_ERFFEL61              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL61_OFFSET)
#  define S32K3XX_CAN1_ERFFEL62              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL62_OFFSET)
#  define S32K3XX_CAN1_ERFFEL63              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL63_OFFSET)
#  define S32K3XX_CAN1_ERFFEL64              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL64_OFFSET)
#  define S32K3XX_CAN1_ERFFEL65              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL65_OFFSET)
#  define S32K3XX_CAN1_ERFFEL66              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL66_OFFSET)
#  define S32K3XX_CAN1_ERFFEL67              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL67_OFFSET)
#  define S32K3XX_CAN1_ERFFEL68              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL68_OFFSET)
#  define S32K3XX_CAN1_ERFFEL69              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL69_OFFSET)
#  define S32K3XX_CAN1_ERFFEL70              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL70_OFFSET)
#  define S32K3XX_CAN1_ERFFEL71              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL71_OFFSET)
#  define S32K3XX_CAN1_ERFFEL72              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL72_OFFSET)
#  define S32K3XX_CAN1_ERFFEL73              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL73_OFFSET)
#  define S32K3XX_CAN1_ERFFEL74              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL74_OFFSET)
#  define S32K3XX_CAN1_ERFFEL75              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL75_OFFSET)
#  define S32K3XX_CAN1_ERFFEL76              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL76_OFFSET)
#  define S32K3XX_CAN1_ERFFEL77              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL77_OFFSET)
#  define S32K3XX_CAN1_ERFFEL78              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL78_OFFSET)
#  define S32K3XX_CAN1_ERFFEL79              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL79_OFFSET)
#  define S32K3XX_CAN1_ERFFEL80              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL80_OFFSET)
#  define S32K3XX_CAN1_ERFFEL81              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL81_OFFSET)
#  define S32K3XX_CAN1_ERFFEL82              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL82_OFFSET)
#  define S32K3XX_CAN1_ERFFEL83              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL83_OFFSET)
#  define S32K3XX_CAN1_ERFFEL84              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL84_OFFSET)
#  define S32K3XX_CAN1_ERFFEL85              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL85_OFFSET)
#  define S32K3XX_CAN1_ERFFEL86              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL86_OFFSET)
#  define S32K3XX_CAN1_ERFFEL87              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL87_OFFSET)
#  define S32K3XX_CAN1_ERFFEL88              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL88_OFFSET)
#  define S32K3XX_CAN1_ERFFEL89              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL89_OFFSET)
#  define S32K3XX_CAN1_ERFFEL90              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL90_OFFSET)
#  define S32K3XX_CAN1_ERFFEL91              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL91_OFFSET)
#  define S32K3XX_CAN1_ERFFEL92              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL92_OFFSET)
#  define S32K3XX_CAN1_ERFFEL93              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL93_OFFSET)
#  define S32K3XX_CAN1_ERFFEL94              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL94_OFFSET)
#  define S32K3XX_CAN1_ERFFEL95              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL95_OFFSET)
#  define S32K3XX_CAN1_ERFFEL96              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL96_OFFSET)
#  define S32K3XX_CAN1_ERFFEL97              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL97_OFFSET)
#  define S32K3XX_CAN1_ERFFEL98              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL98_OFFSET)
#  define S32K3XX_CAN1_ERFFEL99              (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL99_OFFSET)
#  define S32K3XX_CAN1_ERFFEL100             (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL100_OFFSET)
#  define S32K3XX_CAN1_ERFFEL101             (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL101_OFFSET)
#  define S32K3XX_CAN1_ERFFEL102             (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL102_OFFSET)
#  define S32K3XX_CAN1_ERFFEL103             (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL103_OFFSET)
#  define S32K3XX_CAN1_ERFFEL104             (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL104_OFFSET)
#  define S32K3XX_CAN1_ERFFEL105             (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL105_OFFSET)
#  define S32K3XX_CAN1_ERFFEL106             (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL106_OFFSET)
#  define S32K3XX_CAN1_ERFFEL107             (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL107_OFFSET)
#  define S32K3XX_CAN1_ERFFEL108             (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL108_OFFSET)
#  define S32K3XX_CAN1_ERFFEL109             (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL109_OFFSET)
#  define S32K3XX_CAN1_ERFFEL110             (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL110_OFFSET)
#  define S32K3XX_CAN1_ERFFEL111             (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL111_OFFSET)
#  define S32K3XX_CAN1_ERFFEL112             (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL112_OFFSET)
#  define S32K3XX_CAN1_ERFFEL113             (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL113_OFFSET)
#  define S32K3XX_CAN1_ERFFEL114             (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL114_OFFSET)
#  define S32K3XX_CAN1_ERFFEL115             (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL115_OFFSET)
#  define S32K3XX_CAN1_ERFFEL116             (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL116_OFFSET)
#  define S32K3XX_CAN1_ERFFEL117             (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL117_OFFSET)
#  define S32K3XX_CAN1_ERFFEL118             (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL118_OFFSET)
#  define S32K3XX_CAN1_ERFFEL119             (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL119_OFFSET)
#  define S32K3XX_CAN1_ERFFEL120             (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL120_OFFSET)
#  define S32K3XX_CAN1_ERFFEL121             (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL121_OFFSET)
#  define S32K3XX_CAN1_ERFFEL122             (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL122_OFFSET)
#  define S32K3XX_CAN1_ERFFEL123             (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL123_OFFSET)
#  define S32K3XX_CAN1_ERFFEL124             (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL124_OFFSET)
#  define S32K3XX_CAN1_ERFFEL125             (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL125_OFFSET)
#  define S32K3XX_CAN1_ERFFEL126             (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL126_OFFSET)
#  define S32K3XX_CAN1_ERFFEL127             (S32K3XX_CAN1_BASE + S32K3XX_CAN_ERFFEL127_OFFSET)

/* FlexCAN2 */

#define S32K3XX_CAN2_MCR                     (S32K3XX_CAN2_BASE + S32K3XX_CAN_MCR_OFFSET)
#define S32K3XX_CAN2_CTRL1                   (S32K3XX_CAN2_BASE + S32K3XX_CAN_CTRL1_OFFSET)
#define S32K3XX_CAN2_TIMER                   (S32K3XX_CAN2_BASE + S32K3XX_CAN_TIMER_OFFSET)
#define S32K3XX_CAN2_RXMGMASK                (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXMGMASK_OFFSET)
#define S32K3XX_CAN2_RX14MASK                (S32K3XX_CAN2_BASE + S32K3XX_CAN_RX14MASK_OFFSET)
#define S32K3XX_CAN2_RX15MASK                (S32K3XX_CAN2_BASE + S32K3XX_CAN_RX15MASK_OFFSET)
#define S32K3XX_CAN2_ECR                     (S32K3XX_CAN2_BASE + S32K3XX_CAN_ECR_OFFSET)
#define S32K3XX_CAN2_ESR1                    (S32K3XX_CAN2_BASE + S32K3XX_CAN_ESR1_OFFSET)
#define S32K3XX_CAN2_IMASK2                  (S32K3XX_CAN2_BASE + S32K3XX_CAN_IMASK2_OFFSET)
#define S32K3XX_CAN2_IMASK1                  (S32K3XX_CAN2_BASE + S32K3XX_CAN_IMASK1_OFFSET)
#define S32K3XX_CAN2_IFLAG2                  (S32K3XX_CAN2_BASE + S32K3XX_CAN_IFLAG2_OFFSET)
#define S32K3XX_CAN2_IFLAG1                  (S32K3XX_CAN2_BASE + S32K3XX_CAN_IFLAG1_OFFSET)
#define S32K3XX_CAN2_CTRL2                   (S32K3XX_CAN2_BASE + S32K3XX_CAN_CTRL2_OFFSET)
#define S32K3XX_CAN2_ESR2                    (S32K3XX_CAN2_BASE + S32K3XX_CAN_ESR2_OFFSET)
#define S32K3XX_CAN2_CRCR                    (S32K3XX_CAN2_BASE + S32K3XX_CAN_CRCR_OFFSET)
#define S32K3XX_CAN2_RXFGMASK                (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXFGMASK_OFFSET)
#define S32K3XX_CAN2_RXFIR                   (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXFIR_OFFSET)
#define S32K3XX_CAN2_CBT                     (S32K3XX_CAN2_BASE + S32K3XX_CAN_CBT_OFFSET)
#define S32K3XX_CAN2_IMASK3                  (S32K3XX_CAN2_BASE + S32K3XX_CAN_IMASK3_OFFSET)
#define S32K3XX_CAN2_IFLAG3                  (S32K3XX_CAN2_BASE + S32K3XX_CAN_IFLAG3_OFFSET)

#define S32K3XX_CAN2_RXIMR(n)                (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR_OFFSET(n))
#  define S32K3XX_CAN2_RXIMR0                (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR0_OFFSET)
#  define S32K3XX_CAN2_RXIMR1                (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR1_OFFSET)
#  define S32K3XX_CAN2_RXIMR2                (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR2_OFFSET)
#  define S32K3XX_CAN2_RXIMR3                (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR3_OFFSET)
#  define S32K3XX_CAN2_RXIMR4                (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR4_OFFSET)
#  define S32K3XX_CAN2_RXIMR5                (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR5_OFFSET)
#  define S32K3XX_CAN2_RXIMR6                (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR6_OFFSET)
#  define S32K3XX_CAN2_RXIMR7                (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR7_OFFSET)
#  define S32K3XX_CAN2_RXIMR8                (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR8_OFFSET)
#  define S32K3XX_CAN2_RXIMR9                (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR9_OFFSET)
#  define S32K3XX_CAN2_RXIMR10               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR10_OFFSET)
#  define S32K3XX_CAN2_RXIMR11               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR11_OFFSET)
#  define S32K3XX_CAN2_RXIMR12               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR12_OFFSET)
#  define S32K3XX_CAN2_RXIMR13               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR13_OFFSET)
#  define S32K3XX_CAN2_RXIMR14               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR14_OFFSET)
#  define S32K3XX_CAN2_RXIMR15               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR15_OFFSET)
#  define S32K3XX_CAN2_RXIMR16               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR16_OFFSET)
#  define S32K3XX_CAN2_RXIMR17               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR17_OFFSET)
#  define S32K3XX_CAN2_RXIMR18               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR18_OFFSET)
#  define S32K3XX_CAN2_RXIMR19               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR19_OFFSET)
#  define S32K3XX_CAN2_RXIMR20               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR20_OFFSET)
#  define S32K3XX_CAN2_RXIMR21               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR21_OFFSET)
#  define S32K3XX_CAN2_RXIMR22               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR22_OFFSET)
#  define S32K3XX_CAN2_RXIMR23               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR23_OFFSET)
#  define S32K3XX_CAN2_RXIMR24               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR24_OFFSET)
#  define S32K3XX_CAN2_RXIMR25               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR25_OFFSET)
#  define S32K3XX_CAN2_RXIMR26               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR26_OFFSET)
#  define S32K3XX_CAN2_RXIMR27               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR27_OFFSET)
#  define S32K3XX_CAN2_RXIMR28               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR28_OFFSET)
#  define S32K3XX_CAN2_RXIMR29               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR29_OFFSET)
#  define S32K3XX_CAN2_RXIMR30               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR30_OFFSET)
#  define S32K3XX_CAN2_RXIMR31               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR31_OFFSET)
#  define S32K3XX_CAN2_RXIMR32               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR32_OFFSET)
#  define S32K3XX_CAN2_RXIMR33               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR33_OFFSET)
#  define S32K3XX_CAN2_RXIMR34               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR34_OFFSET)
#  define S32K3XX_CAN2_RXIMR35               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR35_OFFSET)
#  define S32K3XX_CAN2_RXIMR36               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR36_OFFSET)
#  define S32K3XX_CAN2_RXIMR37               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR37_OFFSET)
#  define S32K3XX_CAN2_RXIMR38               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR38_OFFSET)
#  define S32K3XX_CAN2_RXIMR39               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR39_OFFSET)
#  define S32K3XX_CAN2_RXIMR40               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR40_OFFSET)
#  define S32K3XX_CAN2_RXIMR41               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR41_OFFSET)
#  define S32K3XX_CAN2_RXIMR42               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR42_OFFSET)
#  define S32K3XX_CAN2_RXIMR43               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR43_OFFSET)
#  define S32K3XX_CAN2_RXIMR44               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR44_OFFSET)
#  define S32K3XX_CAN2_RXIMR45               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR45_OFFSET)
#  define S32K3XX_CAN2_RXIMR46               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR46_OFFSET)
#  define S32K3XX_CAN2_RXIMR47               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR47_OFFSET)
#  define S32K3XX_CAN2_RXIMR48               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR48_OFFSET)
#  define S32K3XX_CAN2_RXIMR49               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR49_OFFSET)
#  define S32K3XX_CAN2_RXIMR50               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR50_OFFSET)
#  define S32K3XX_CAN2_RXIMR51               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR51_OFFSET)
#  define S32K3XX_CAN2_RXIMR52               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR52_OFFSET)
#  define S32K3XX_CAN2_RXIMR53               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR53_OFFSET)
#  define S32K3XX_CAN2_RXIMR54               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR54_OFFSET)
#  define S32K3XX_CAN2_RXIMR55               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR55_OFFSET)
#  define S32K3XX_CAN2_RXIMR56               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR56_OFFSET)
#  define S32K3XX_CAN2_RXIMR57               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR57_OFFSET)
#  define S32K3XX_CAN2_RXIMR58               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR58_OFFSET)
#  define S32K3XX_CAN2_RXIMR59               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR59_OFFSET)
#  define S32K3XX_CAN2_RXIMR60               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR60_OFFSET)
#  define S32K3XX_CAN2_RXIMR61               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR61_OFFSET)
#  define S32K3XX_CAN2_RXIMR62               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR62_OFFSET)
#  define S32K3XX_CAN2_RXIMR63               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR63_OFFSET)
#  define S32K3XX_CAN2_RXIMR64               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR64_OFFSET)
#  define S32K3XX_CAN2_RXIMR65               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR65_OFFSET)
#  define S32K3XX_CAN2_RXIMR66               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR66_OFFSET)
#  define S32K3XX_CAN2_RXIMR67               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR67_OFFSET)
#  define S32K3XX_CAN2_RXIMR68               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR68_OFFSET)
#  define S32K3XX_CAN2_RXIMR69               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR69_OFFSET)
#  define S32K3XX_CAN2_RXIMR70               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR70_OFFSET)
#  define S32K3XX_CAN2_RXIMR71               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR71_OFFSET)
#  define S32K3XX_CAN2_RXIMR72               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR72_OFFSET)
#  define S32K3XX_CAN2_RXIMR73               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR73_OFFSET)
#  define S32K3XX_CAN2_RXIMR74               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR74_OFFSET)
#  define S32K3XX_CAN2_RXIMR75               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR75_OFFSET)
#  define S32K3XX_CAN2_RXIMR76               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR76_OFFSET)
#  define S32K3XX_CAN2_RXIMR77               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR77_OFFSET)
#  define S32K3XX_CAN2_RXIMR78               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR78_OFFSET)
#  define S32K3XX_CAN2_RXIMR79               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR79_OFFSET)
#  define S32K3XX_CAN2_RXIMR80               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR80_OFFSET)
#  define S32K3XX_CAN2_RXIMR81               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR81_OFFSET)
#  define S32K3XX_CAN2_RXIMR82               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR82_OFFSET)
#  define S32K3XX_CAN2_RXIMR83               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR83_OFFSET)
#  define S32K3XX_CAN2_RXIMR84               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR84_OFFSET)
#  define S32K3XX_CAN2_RXIMR85               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR85_OFFSET)
#  define S32K3XX_CAN2_RXIMR86               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR86_OFFSET)
#  define S32K3XX_CAN2_RXIMR87               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR87_OFFSET)
#  define S32K3XX_CAN2_RXIMR88               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR88_OFFSET)
#  define S32K3XX_CAN2_RXIMR89               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR89_OFFSET)
#  define S32K3XX_CAN2_RXIMR90               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR90_OFFSET)
#  define S32K3XX_CAN2_RXIMR91               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR91_OFFSET)
#  define S32K3XX_CAN2_RXIMR92               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR92_OFFSET)
#  define S32K3XX_CAN2_RXIMR93               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR93_OFFSET)
#  define S32K3XX_CAN2_RXIMR94               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR94_OFFSET)
#  define S32K3XX_CAN2_RXIMR95               (S32K3XX_CAN2_BASE + S32K3XX_CAN_RXIMR95_OFFSET)

#define S32K3XX_CAN2_MECR                    (S32K3XX_CAN2_BASE + S32K3XX_CAN_MECR_OFFSET)
#define S32K3XX_CAN2_ERRIAR                  (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERRIAR_OFFSET)
#define S32K3XX_CAN2_ERRIDPR                 (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERRIDPR_OFFSET)
#define S32K3XX_CAN2_ERRIPPR                 (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERRIPPR_OFFSET)
#define S32K3XX_CAN2_RERRAR                  (S32K3XX_CAN2_BASE + S32K3XX_CAN_RERRAR_OFFSET)
#define S32K3XX_CAN2_RERRDR                  (S32K3XX_CAN2_BASE + S32K3XX_CAN_RERRDR_OFFSET)
#define S32K3XX_CAN2_RERRSYNR                (S32K3XX_CAN2_BASE + S32K3XX_CAN_RERRSYNR_OFFSET)
#define S32K3XX_CAN2_ERRSR                   (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERRSR_OFFSET)
#define S32K3XX_CAN2_EPRS                    (S32K3XX_CAN2_BASE + S32K3XX_CAN_EPRS_OFFSET)
#define S32K3XX_CAN2_ENCBT                   (S32K3XX_CAN2_BASE + S32K3XX_CAN_ENCBT_OFFSET)
#define S32K3XX_CAN2_EDCBT                   (S32K3XX_CAN2_BASE + S32K3XX_CAN_EDCBT_OFFSET)
#define S32K3XX_CAN2_ETDC                    (S32K3XX_CAN2_BASE + S32K3XX_CAN_ETDC_OFFSET)
#define S32K3XX_CAN2_FDCTRL                  (S32K3XX_CAN2_BASE + S32K3XX_CAN_FDCTRL_OFFSET)
#define S32K3XX_CAN2_FDCBT                   (S32K3XX_CAN2_BASE + S32K3XX_CAN_FDCBT_OFFSET)
#define S32K3XX_CAN2_FDCRC                   (S32K3XX_CAN2_BASE + S32K3XX_CAN_FDCRC_OFFSET)
#define S32K3XX_CAN2_ERFCR                   (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFCR_OFFSET)
#define S32K3XX_CAN2_ERFIER                  (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFIER_OFFSET)
#define S32K3XX_CAN2_ERFSR                   (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFSR_OFFSET)

#define S32K3XX_CAN2_HR_TIME_STAMP(n)        (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP_OFFSET(n))  
#  define S32K3XX_CAN2_HR_TIME_STAMP0        (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP0_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP1        (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP1_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP2        (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP2_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP3        (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP3_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP4        (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP4_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP5        (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP5_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP6        (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP6_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP7        (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP7_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP8        (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP8_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP9        (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP9_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP10       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP10_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP11       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP11_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP12       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP12_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP13       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP13_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP14       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP14_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP15       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP15_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP16       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP16_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP17       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP17_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP18       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP18_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP19       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP19_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP20       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP20_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP21       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP21_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP22       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP22_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP23       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP23_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP24       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP24_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP25       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP25_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP26       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP26_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP27       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP27_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP28       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP28_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP29       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP29_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP30       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP30_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP31       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP31_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP32       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP32_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP33       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP33_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP34       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP34_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP35       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP35_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP36       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP36_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP37       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP37_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP38       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP38_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP39       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP39_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP40       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP40_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP41       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP41_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP42       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP42_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP43       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP43_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP44       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP44_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP45       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP45_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP46       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP46_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP47       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP47_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP48       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP48_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP49       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP49_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP50       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP50_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP51       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP51_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP52       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP52_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP53       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP53_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP54       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP54_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP55       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP55_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP56       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP56_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP57       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP57_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP58       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP58_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP59       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP59_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP60       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP60_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP61       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP61_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP62       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP62_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP63       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP63_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP64       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP64_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP65       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP65_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP66       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP66_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP67       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP67_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP68       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP68_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP69       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP69_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP70       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP70_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP71       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP71_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP72       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP72_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP73       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP73_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP74       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP74_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP75       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP75_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP76       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP76_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP77       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP77_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP78       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP78_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP79       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP79_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP80       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP80_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP81       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP81_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP82       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP82_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP83       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP83_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP84       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP84_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP85       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP85_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP86       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP86_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP87       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP87_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP88       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP88_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP89       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP89_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP90       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP90_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP91       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP91_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP92       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP92_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP93       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP93_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP94       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP94_OFFSET)
#  define S32K3XX_CAN2_HR_TIME_STAMP95       (S32K3XX_CAN2_BASE + S32K3XX_CAN_HR_TIME_STAMP95_OFFSET)

#define S32K3XX_CAN2_ERFFEL(n)               (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL_OFFSET(n))
#  define S32K3XX_CAN2_ERFFEL0               (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL0_OFFSET)
#  define S32K3XX_CAN2_ERFFEL1               (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL1_OFFSET)
#  define S32K3XX_CAN2_ERFFEL2               (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL2_OFFSET)
#  define S32K3XX_CAN2_ERFFEL3               (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL3_OFFSET)
#  define S32K3XX_CAN2_ERFFEL4               (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL4_OFFSET)
#  define S32K3XX_CAN2_ERFFEL5               (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL5_OFFSET)
#  define S32K3XX_CAN2_ERFFEL6               (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL6_OFFSET)
#  define S32K3XX_CAN2_ERFFEL7               (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL7_OFFSET)
#  define S32K3XX_CAN2_ERFFEL8               (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL8_OFFSET)
#  define S32K3XX_CAN2_ERFFEL9               (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL9_OFFSET)
#  define S32K3XX_CAN2_ERFFEL10              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL10_OFFSET)
#  define S32K3XX_CAN2_ERFFEL11              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL11_OFFSET)
#  define S32K3XX_CAN2_ERFFEL12              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL12_OFFSET)
#  define S32K3XX_CAN2_ERFFEL13              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL13_OFFSET)
#  define S32K3XX_CAN2_ERFFEL14              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL14_OFFSET)
#  define S32K3XX_CAN2_ERFFEL15              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL15_OFFSET)
#  define S32K3XX_CAN2_ERFFEL16              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL16_OFFSET)
#  define S32K3XX_CAN2_ERFFEL17              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL17_OFFSET)
#  define S32K3XX_CAN2_ERFFEL18              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL18_OFFSET)
#  define S32K3XX_CAN2_ERFFEL19              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL19_OFFSET)
#  define S32K3XX_CAN2_ERFFEL20              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL20_OFFSET)
#  define S32K3XX_CAN2_ERFFEL21              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL21_OFFSET)
#  define S32K3XX_CAN2_ERFFEL22              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL22_OFFSET)
#  define S32K3XX_CAN2_ERFFEL23              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL23_OFFSET)
#  define S32K3XX_CAN2_ERFFEL24              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL24_OFFSET)
#  define S32K3XX_CAN2_ERFFEL25              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL25_OFFSET)
#  define S32K3XX_CAN2_ERFFEL26              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL26_OFFSET)
#  define S32K3XX_CAN2_ERFFEL27              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL27_OFFSET)
#  define S32K3XX_CAN2_ERFFEL28              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL28_OFFSET)
#  define S32K3XX_CAN2_ERFFEL29              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL29_OFFSET)
#  define S32K3XX_CAN2_ERFFEL30              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL30_OFFSET)
#  define S32K3XX_CAN2_ERFFEL31              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL31_OFFSET)
#  define S32K3XX_CAN2_ERFFEL32              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL32_OFFSET)
#  define S32K3XX_CAN2_ERFFEL33              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL33_OFFSET)
#  define S32K3XX_CAN2_ERFFEL34              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL34_OFFSET)
#  define S32K3XX_CAN2_ERFFEL35              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL35_OFFSET)
#  define S32K3XX_CAN2_ERFFEL36              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL36_OFFSET)
#  define S32K3XX_CAN2_ERFFEL37              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL37_OFFSET)
#  define S32K3XX_CAN2_ERFFEL38              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL38_OFFSET)
#  define S32K3XX_CAN2_ERFFEL39              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL39_OFFSET)
#  define S32K3XX_CAN2_ERFFEL40              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL40_OFFSET)
#  define S32K3XX_CAN2_ERFFEL41              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL41_OFFSET)
#  define S32K3XX_CAN2_ERFFEL42              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL42_OFFSET)
#  define S32K3XX_CAN2_ERFFEL43              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL43_OFFSET)
#  define S32K3XX_CAN2_ERFFEL44              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL44_OFFSET)
#  define S32K3XX_CAN2_ERFFEL45              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL45_OFFSET)
#  define S32K3XX_CAN2_ERFFEL46              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL46_OFFSET)
#  define S32K3XX_CAN2_ERFFEL47              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL47_OFFSET)
#  define S32K3XX_CAN2_ERFFEL48              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL48_OFFSET)
#  define S32K3XX_CAN2_ERFFEL49              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL49_OFFSET)
#  define S32K3XX_CAN2_ERFFEL50              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL50_OFFSET)
#  define S32K3XX_CAN2_ERFFEL51              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL51_OFFSET)
#  define S32K3XX_CAN2_ERFFEL52              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL52_OFFSET)
#  define S32K3XX_CAN2_ERFFEL53              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL53_OFFSET)
#  define S32K3XX_CAN2_ERFFEL54              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL54_OFFSET)
#  define S32K3XX_CAN2_ERFFEL55              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL55_OFFSET)
#  define S32K3XX_CAN2_ERFFEL56              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL56_OFFSET)
#  define S32K3XX_CAN2_ERFFEL57              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL57_OFFSET)
#  define S32K3XX_CAN2_ERFFEL58              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL58_OFFSET)
#  define S32K3XX_CAN2_ERFFEL59              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL59_OFFSET)
#  define S32K3XX_CAN2_ERFFEL60              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL60_OFFSET)
#  define S32K3XX_CAN2_ERFFEL61              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL61_OFFSET)
#  define S32K3XX_CAN2_ERFFEL62              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL62_OFFSET)
#  define S32K3XX_CAN2_ERFFEL63              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL63_OFFSET)
#  define S32K3XX_CAN2_ERFFEL64              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL64_OFFSET)
#  define S32K3XX_CAN2_ERFFEL65              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL65_OFFSET)
#  define S32K3XX_CAN2_ERFFEL66              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL66_OFFSET)
#  define S32K3XX_CAN2_ERFFEL67              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL67_OFFSET)
#  define S32K3XX_CAN2_ERFFEL68              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL68_OFFSET)
#  define S32K3XX_CAN2_ERFFEL69              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL69_OFFSET)
#  define S32K3XX_CAN2_ERFFEL70              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL70_OFFSET)
#  define S32K3XX_CAN2_ERFFEL71              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL71_OFFSET)
#  define S32K3XX_CAN2_ERFFEL72              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL72_OFFSET)
#  define S32K3XX_CAN2_ERFFEL73              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL73_OFFSET)
#  define S32K3XX_CAN2_ERFFEL74              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL74_OFFSET)
#  define S32K3XX_CAN2_ERFFEL75              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL75_OFFSET)
#  define S32K3XX_CAN2_ERFFEL76              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL76_OFFSET)
#  define S32K3XX_CAN2_ERFFEL77              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL77_OFFSET)
#  define S32K3XX_CAN2_ERFFEL78              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL78_OFFSET)
#  define S32K3XX_CAN2_ERFFEL79              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL79_OFFSET)
#  define S32K3XX_CAN2_ERFFEL80              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL80_OFFSET)
#  define S32K3XX_CAN2_ERFFEL81              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL81_OFFSET)
#  define S32K3XX_CAN2_ERFFEL82              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL82_OFFSET)
#  define S32K3XX_CAN2_ERFFEL83              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL83_OFFSET)
#  define S32K3XX_CAN2_ERFFEL84              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL84_OFFSET)
#  define S32K3XX_CAN2_ERFFEL85              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL85_OFFSET)
#  define S32K3XX_CAN2_ERFFEL86              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL86_OFFSET)
#  define S32K3XX_CAN2_ERFFEL87              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL87_OFFSET)
#  define S32K3XX_CAN2_ERFFEL88              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL88_OFFSET)
#  define S32K3XX_CAN2_ERFFEL89              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL89_OFFSET)
#  define S32K3XX_CAN2_ERFFEL90              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL90_OFFSET)
#  define S32K3XX_CAN2_ERFFEL91              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL91_OFFSET)
#  define S32K3XX_CAN2_ERFFEL92              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL92_OFFSET)
#  define S32K3XX_CAN2_ERFFEL93              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL93_OFFSET)
#  define S32K3XX_CAN2_ERFFEL94              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL94_OFFSET)
#  define S32K3XX_CAN2_ERFFEL95              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL95_OFFSET)
#  define S32K3XX_CAN2_ERFFEL96              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL96_OFFSET)
#  define S32K3XX_CAN2_ERFFEL97              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL97_OFFSET)
#  define S32K3XX_CAN2_ERFFEL98              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL98_OFFSET)
#  define S32K3XX_CAN2_ERFFEL99              (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL99_OFFSET)
#  define S32K3XX_CAN2_ERFFEL100             (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL100_OFFSET)
#  define S32K3XX_CAN2_ERFFEL101             (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL101_OFFSET)
#  define S32K3XX_CAN2_ERFFEL102             (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL102_OFFSET)
#  define S32K3XX_CAN2_ERFFEL103             (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL103_OFFSET)
#  define S32K3XX_CAN2_ERFFEL104             (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL104_OFFSET)
#  define S32K3XX_CAN2_ERFFEL105             (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL105_OFFSET)
#  define S32K3XX_CAN2_ERFFEL106             (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL106_OFFSET)
#  define S32K3XX_CAN2_ERFFEL107             (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL107_OFFSET)
#  define S32K3XX_CAN2_ERFFEL108             (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL108_OFFSET)
#  define S32K3XX_CAN2_ERFFEL109             (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL109_OFFSET)
#  define S32K3XX_CAN2_ERFFEL110             (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL110_OFFSET)
#  define S32K3XX_CAN2_ERFFEL111             (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL111_OFFSET)
#  define S32K3XX_CAN2_ERFFEL112             (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL112_OFFSET)
#  define S32K3XX_CAN2_ERFFEL113             (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL113_OFFSET)
#  define S32K3XX_CAN2_ERFFEL114             (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL114_OFFSET)
#  define S32K3XX_CAN2_ERFFEL115             (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL115_OFFSET)
#  define S32K3XX_CAN2_ERFFEL116             (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL116_OFFSET)
#  define S32K3XX_CAN2_ERFFEL117             (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL117_OFFSET)
#  define S32K3XX_CAN2_ERFFEL118             (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL118_OFFSET)
#  define S32K3XX_CAN2_ERFFEL119             (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL119_OFFSET)
#  define S32K3XX_CAN2_ERFFEL120             (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL120_OFFSET)
#  define S32K3XX_CAN2_ERFFEL121             (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL121_OFFSET)
#  define S32K3XX_CAN2_ERFFEL122             (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL122_OFFSET)
#  define S32K3XX_CAN2_ERFFEL123             (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL123_OFFSET)
#  define S32K3XX_CAN2_ERFFEL124             (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL124_OFFSET)
#  define S32K3XX_CAN2_ERFFEL125             (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL125_OFFSET)
#  define S32K3XX_CAN2_ERFFEL126             (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL126_OFFSET)
#  define S32K3XX_CAN2_ERFFEL127             (S32K3XX_CAN2_BASE + S32K3XX_CAN_ERFFEL127_OFFSET)

/* FlexCAN3 */

#define S32K3XX_CAN3_MCR                     (S32K3XX_CAN3_BASE + S32K3XX_CAN_MCR_OFFSET)
#define S32K3XX_CAN3_CTRL1                   (S32K3XX_CAN3_BASE + S32K3XX_CAN_CTRL1_OFFSET)
#define S32K3XX_CAN3_TIMER                   (S32K3XX_CAN3_BASE + S32K3XX_CAN_TIMER_OFFSET)
#define S32K3XX_CAN3_RXMGMASK                (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXMGMASK_OFFSET)
#define S32K3XX_CAN3_RX14MASK                (S32K3XX_CAN3_BASE + S32K3XX_CAN_RX14MASK_OFFSET)
#define S32K3XX_CAN3_RX15MASK                (S32K3XX_CAN3_BASE + S32K3XX_CAN_RX15MASK_OFFSET)
#define S32K3XX_CAN3_ECR                     (S32K3XX_CAN3_BASE + S32K3XX_CAN_ECR_OFFSET)
#define S32K3XX_CAN3_ESR1                    (S32K3XX_CAN3_BASE + S32K3XX_CAN_ESR1_OFFSET)
#define S32K3XX_CAN3_IMASK2                  (S32K3XX_CAN3_BASE + S32K3XX_CAN_IMASK2_OFFSET)
#define S32K3XX_CAN3_IMASK1                  (S32K3XX_CAN3_BASE + S32K3XX_CAN_IMASK1_OFFSET)
#define S32K3XX_CAN3_IFLAG2                  (S32K3XX_CAN3_BASE + S32K3XX_CAN_IFLAG2_OFFSET)
#define S32K3XX_CAN3_IFLAG1                  (S32K3XX_CAN3_BASE + S32K3XX_CAN_IFLAG1_OFFSET)
#define S32K3XX_CAN3_CTRL2                   (S32K3XX_CAN3_BASE + S32K3XX_CAN_CTRL2_OFFSET)
#define S32K3XX_CAN3_ESR2                    (S32K3XX_CAN3_BASE + S32K3XX_CAN_ESR2_OFFSET)
#define S32K3XX_CAN3_CRCR                    (S32K3XX_CAN3_BASE + S32K3XX_CAN_CRCR_OFFSET)
#define S32K3XX_CAN3_RXFGMASK                (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXFGMASK_OFFSET)
#define S32K3XX_CAN3_RXFIR                   (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXFIR_OFFSET)
#define S32K3XX_CAN3_CBT                     (S32K3XX_CAN3_BASE + S32K3XX_CAN_CBT_OFFSET)
#define S32K3XX_CAN3_IMASK3                  (S32K3XX_CAN3_BASE + S32K3XX_CAN_IMASK3_OFFSET)
#define S32K3XX_CAN3_IFLAG3                  (S32K3XX_CAN3_BASE + S32K3XX_CAN_IFLAG3_OFFSET)

#define S32K3XX_CAN3_RXIMR(n)                (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR_OFFSET(n))
#  define S32K3XX_CAN3_RXIMR0                (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR0_OFFSET)
#  define S32K3XX_CAN3_RXIMR1                (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR1_OFFSET)
#  define S32K3XX_CAN3_RXIMR2                (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR2_OFFSET)
#  define S32K3XX_CAN3_RXIMR3                (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR3_OFFSET)
#  define S32K3XX_CAN3_RXIMR4                (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR4_OFFSET)
#  define S32K3XX_CAN3_RXIMR5                (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR5_OFFSET)
#  define S32K3XX_CAN3_RXIMR6                (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR6_OFFSET)
#  define S32K3XX_CAN3_RXIMR7                (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR7_OFFSET)
#  define S32K3XX_CAN3_RXIMR8                (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR8_OFFSET)
#  define S32K3XX_CAN3_RXIMR9                (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR9_OFFSET)
#  define S32K3XX_CAN3_RXIMR10               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR10_OFFSET)
#  define S32K3XX_CAN3_RXIMR11               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR11_OFFSET)
#  define S32K3XX_CAN3_RXIMR12               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR12_OFFSET)
#  define S32K3XX_CAN3_RXIMR13               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR13_OFFSET)
#  define S32K3XX_CAN3_RXIMR14               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR14_OFFSET)
#  define S32K3XX_CAN3_RXIMR15               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR15_OFFSET)
#  define S32K3XX_CAN3_RXIMR16               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR16_OFFSET)
#  define S32K3XX_CAN3_RXIMR17               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR17_OFFSET)
#  define S32K3XX_CAN3_RXIMR18               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR18_OFFSET)
#  define S32K3XX_CAN3_RXIMR19               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR19_OFFSET)
#  define S32K3XX_CAN3_RXIMR20               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR20_OFFSET)
#  define S32K3XX_CAN3_RXIMR21               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR21_OFFSET)
#  define S32K3XX_CAN3_RXIMR22               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR22_OFFSET)
#  define S32K3XX_CAN3_RXIMR23               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR23_OFFSET)
#  define S32K3XX_CAN3_RXIMR24               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR24_OFFSET)
#  define S32K3XX_CAN3_RXIMR25               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR25_OFFSET)
#  define S32K3XX_CAN3_RXIMR26               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR26_OFFSET)
#  define S32K3XX_CAN3_RXIMR27               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR27_OFFSET)
#  define S32K3XX_CAN3_RXIMR28               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR28_OFFSET)
#  define S32K3XX_CAN3_RXIMR29               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR29_OFFSET)
#  define S32K3XX_CAN3_RXIMR30               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR30_OFFSET)
#  define S32K3XX_CAN3_RXIMR31               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR31_OFFSET)
#  define S32K3XX_CAN3_RXIMR32               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR32_OFFSET)
#  define S32K3XX_CAN3_RXIMR33               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR33_OFFSET)
#  define S32K3XX_CAN3_RXIMR34               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR34_OFFSET)
#  define S32K3XX_CAN3_RXIMR35               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR35_OFFSET)
#  define S32K3XX_CAN3_RXIMR36               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR36_OFFSET)
#  define S32K3XX_CAN3_RXIMR37               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR37_OFFSET)
#  define S32K3XX_CAN3_RXIMR38               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR38_OFFSET)
#  define S32K3XX_CAN3_RXIMR39               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR39_OFFSET)
#  define S32K3XX_CAN3_RXIMR40               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR40_OFFSET)
#  define S32K3XX_CAN3_RXIMR41               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR41_OFFSET)
#  define S32K3XX_CAN3_RXIMR42               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR42_OFFSET)
#  define S32K3XX_CAN3_RXIMR43               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR43_OFFSET)
#  define S32K3XX_CAN3_RXIMR44               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR44_OFFSET)
#  define S32K3XX_CAN3_RXIMR45               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR45_OFFSET)
#  define S32K3XX_CAN3_RXIMR46               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR46_OFFSET)
#  define S32K3XX_CAN3_RXIMR47               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR47_OFFSET)
#  define S32K3XX_CAN3_RXIMR48               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR48_OFFSET)
#  define S32K3XX_CAN3_RXIMR49               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR49_OFFSET)
#  define S32K3XX_CAN3_RXIMR50               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR50_OFFSET)
#  define S32K3XX_CAN3_RXIMR51               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR51_OFFSET)
#  define S32K3XX_CAN3_RXIMR52               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR52_OFFSET)
#  define S32K3XX_CAN3_RXIMR53               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR53_OFFSET)
#  define S32K3XX_CAN3_RXIMR54               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR54_OFFSET)
#  define S32K3XX_CAN3_RXIMR55               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR55_OFFSET)
#  define S32K3XX_CAN3_RXIMR56               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR56_OFFSET)
#  define S32K3XX_CAN3_RXIMR57               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR57_OFFSET)
#  define S32K3XX_CAN3_RXIMR58               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR58_OFFSET)
#  define S32K3XX_CAN3_RXIMR59               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR59_OFFSET)
#  define S32K3XX_CAN3_RXIMR60               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR60_OFFSET)
#  define S32K3XX_CAN3_RXIMR61               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR61_OFFSET)
#  define S32K3XX_CAN3_RXIMR62               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR62_OFFSET)
#  define S32K3XX_CAN3_RXIMR63               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR63_OFFSET)
#  define S32K3XX_CAN3_RXIMR64               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR64_OFFSET)
#  define S32K3XX_CAN3_RXIMR65               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR65_OFFSET)
#  define S32K3XX_CAN3_RXIMR66               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR66_OFFSET)
#  define S32K3XX_CAN3_RXIMR67               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR67_OFFSET)
#  define S32K3XX_CAN3_RXIMR68               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR68_OFFSET)
#  define S32K3XX_CAN3_RXIMR69               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR69_OFFSET)
#  define S32K3XX_CAN3_RXIMR70               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR70_OFFSET)
#  define S32K3XX_CAN3_RXIMR71               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR71_OFFSET)
#  define S32K3XX_CAN3_RXIMR72               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR72_OFFSET)
#  define S32K3XX_CAN3_RXIMR73               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR73_OFFSET)
#  define S32K3XX_CAN3_RXIMR74               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR74_OFFSET)
#  define S32K3XX_CAN3_RXIMR75               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR75_OFFSET)
#  define S32K3XX_CAN3_RXIMR76               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR76_OFFSET)
#  define S32K3XX_CAN3_RXIMR77               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR77_OFFSET)
#  define S32K3XX_CAN3_RXIMR78               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR78_OFFSET)
#  define S32K3XX_CAN3_RXIMR79               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR79_OFFSET)
#  define S32K3XX_CAN3_RXIMR80               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR80_OFFSET)
#  define S32K3XX_CAN3_RXIMR81               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR81_OFFSET)
#  define S32K3XX_CAN3_RXIMR82               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR82_OFFSET)
#  define S32K3XX_CAN3_RXIMR83               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR83_OFFSET)
#  define S32K3XX_CAN3_RXIMR84               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR84_OFFSET)
#  define S32K3XX_CAN3_RXIMR85               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR85_OFFSET)
#  define S32K3XX_CAN3_RXIMR86               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR86_OFFSET)
#  define S32K3XX_CAN3_RXIMR87               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR87_OFFSET)
#  define S32K3XX_CAN3_RXIMR88               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR88_OFFSET)
#  define S32K3XX_CAN3_RXIMR89               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR89_OFFSET)
#  define S32K3XX_CAN3_RXIMR90               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR90_OFFSET)
#  define S32K3XX_CAN3_RXIMR91               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR91_OFFSET)
#  define S32K3XX_CAN3_RXIMR92               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR92_OFFSET)
#  define S32K3XX_CAN3_RXIMR93               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR93_OFFSET)
#  define S32K3XX_CAN3_RXIMR94               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR94_OFFSET)
#  define S32K3XX_CAN3_RXIMR95               (S32K3XX_CAN3_BASE + S32K3XX_CAN_RXIMR95_OFFSET)

#define S32K3XX_CAN3_MECR                    (S32K3XX_CAN3_BASE + S32K3XX_CAN_MECR_OFFSET)
#define S32K3XX_CAN3_ERRIAR                  (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERRIAR_OFFSET)
#define S32K3XX_CAN3_ERRIDPR                 (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERRIDPR_OFFSET)
#define S32K3XX_CAN3_ERRIPPR                 (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERRIPPR_OFFSET)
#define S32K3XX_CAN3_RERRAR                  (S32K3XX_CAN3_BASE + S32K3XX_CAN_RERRAR_OFFSET)
#define S32K3XX_CAN3_RERRDR                  (S32K3XX_CAN3_BASE + S32K3XX_CAN_RERRDR_OFFSET)
#define S32K3XX_CAN3_RERRSYNR                (S32K3XX_CAN3_BASE + S32K3XX_CAN_RERRSYNR_OFFSET)
#define S32K3XX_CAN3_ERRSR                   (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERRSR_OFFSET)
#define S32K3XX_CAN3_EPRS                    (S32K3XX_CAN3_BASE + S32K3XX_CAN_EPRS_OFFSET)
#define S32K3XX_CAN3_ENCBT                   (S32K3XX_CAN3_BASE + S32K3XX_CAN_ENCBT_OFFSET)
#define S32K3XX_CAN3_EDCBT                   (S32K3XX_CAN3_BASE + S32K3XX_CAN_EDCBT_OFFSET)
#define S32K3XX_CAN3_ETDC                    (S32K3XX_CAN3_BASE + S32K3XX_CAN_ETDC_OFFSET)
#define S32K3XX_CAN3_FDCTRL                  (S32K3XX_CAN3_BASE + S32K3XX_CAN_FDCTRL_OFFSET)
#define S32K3XX_CAN3_FDCBT                   (S32K3XX_CAN3_BASE + S32K3XX_CAN_FDCBT_OFFSET)
#define S32K3XX_CAN3_FDCRC                   (S32K3XX_CAN3_BASE + S32K3XX_CAN_FDCRC_OFFSET)
#define S32K3XX_CAN3_ERFCR                   (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFCR_OFFSET)
#define S32K3XX_CAN3_ERFIER                  (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFIER_OFFSET)
#define S32K3XX_CAN3_ERFSR                   (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFSR_OFFSET)

#define S32K3XX_CAN3_HR_TIME_STAMP(n)        (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP_OFFSET(n))  
#  define S32K3XX_CAN3_HR_TIME_STAMP0        (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP0_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP1        (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP1_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP2        (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP2_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP3        (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP3_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP4        (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP4_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP5        (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP5_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP6        (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP6_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP7        (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP7_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP8        (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP8_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP9        (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP9_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP10       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP10_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP11       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP11_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP12       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP12_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP13       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP13_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP14       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP14_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP15       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP15_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP16       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP16_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP17       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP17_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP18       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP18_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP19       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP19_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP20       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP20_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP21       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP21_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP22       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP22_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP23       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP23_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP24       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP24_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP25       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP25_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP26       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP26_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP27       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP27_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP28       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP28_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP29       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP29_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP30       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP30_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP31       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP31_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP32       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP32_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP33       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP33_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP34       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP34_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP35       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP35_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP36       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP36_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP37       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP37_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP38       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP38_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP39       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP39_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP40       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP40_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP41       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP41_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP42       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP42_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP43       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP43_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP44       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP44_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP45       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP45_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP46       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP46_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP47       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP47_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP48       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP48_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP49       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP49_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP50       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP50_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP51       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP51_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP52       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP52_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP53       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP53_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP54       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP54_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP55       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP55_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP56       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP56_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP57       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP57_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP58       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP58_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP59       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP59_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP60       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP60_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP61       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP61_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP62       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP62_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP63       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP63_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP64       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP64_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP65       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP65_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP66       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP66_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP67       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP67_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP68       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP68_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP69       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP69_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP70       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP70_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP71       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP71_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP72       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP72_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP73       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP73_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP74       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP74_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP75       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP75_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP76       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP76_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP77       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP77_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP78       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP78_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP79       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP79_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP80       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP80_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP81       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP81_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP82       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP82_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP83       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP83_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP84       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP84_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP85       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP85_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP86       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP86_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP87       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP87_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP88       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP88_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP89       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP89_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP90       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP90_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP91       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP91_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP92       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP92_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP93       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP93_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP94       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP94_OFFSET)
#  define S32K3XX_CAN3_HR_TIME_STAMP95       (S32K3XX_CAN3_BASE + S32K3XX_CAN_HR_TIME_STAMP95_OFFSET)

#define S32K3XX_CAN3_ERFFEL(n)               (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL_OFFSET(n))
#  define S32K3XX_CAN3_ERFFEL0               (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL0_OFFSET)
#  define S32K3XX_CAN3_ERFFEL1               (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL1_OFFSET)
#  define S32K3XX_CAN3_ERFFEL2               (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL2_OFFSET)
#  define S32K3XX_CAN3_ERFFEL3               (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL3_OFFSET)
#  define S32K3XX_CAN3_ERFFEL4               (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL4_OFFSET)
#  define S32K3XX_CAN3_ERFFEL5               (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL5_OFFSET)
#  define S32K3XX_CAN3_ERFFEL6               (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL6_OFFSET)
#  define S32K3XX_CAN3_ERFFEL7               (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL7_OFFSET)
#  define S32K3XX_CAN3_ERFFEL8               (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL8_OFFSET)
#  define S32K3XX_CAN3_ERFFEL9               (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL9_OFFSET)
#  define S32K3XX_CAN3_ERFFEL10              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL10_OFFSET)
#  define S32K3XX_CAN3_ERFFEL11              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL11_OFFSET)
#  define S32K3XX_CAN3_ERFFEL12              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL12_OFFSET)
#  define S32K3XX_CAN3_ERFFEL13              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL13_OFFSET)
#  define S32K3XX_CAN3_ERFFEL14              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL14_OFFSET)
#  define S32K3XX_CAN3_ERFFEL15              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL15_OFFSET)
#  define S32K3XX_CAN3_ERFFEL16              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL16_OFFSET)
#  define S32K3XX_CAN3_ERFFEL17              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL17_OFFSET)
#  define S32K3XX_CAN3_ERFFEL18              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL18_OFFSET)
#  define S32K3XX_CAN3_ERFFEL19              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL19_OFFSET)
#  define S32K3XX_CAN3_ERFFEL20              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL20_OFFSET)
#  define S32K3XX_CAN3_ERFFEL21              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL21_OFFSET)
#  define S32K3XX_CAN3_ERFFEL22              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL22_OFFSET)
#  define S32K3XX_CAN3_ERFFEL23              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL23_OFFSET)
#  define S32K3XX_CAN3_ERFFEL24              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL24_OFFSET)
#  define S32K3XX_CAN3_ERFFEL25              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL25_OFFSET)
#  define S32K3XX_CAN3_ERFFEL26              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL26_OFFSET)
#  define S32K3XX_CAN3_ERFFEL27              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL27_OFFSET)
#  define S32K3XX_CAN3_ERFFEL28              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL28_OFFSET)
#  define S32K3XX_CAN3_ERFFEL29              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL29_OFFSET)
#  define S32K3XX_CAN3_ERFFEL30              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL30_OFFSET)
#  define S32K3XX_CAN3_ERFFEL31              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL31_OFFSET)
#  define S32K3XX_CAN3_ERFFEL32              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL32_OFFSET)
#  define S32K3XX_CAN3_ERFFEL33              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL33_OFFSET)
#  define S32K3XX_CAN3_ERFFEL34              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL34_OFFSET)
#  define S32K3XX_CAN3_ERFFEL35              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL35_OFFSET)
#  define S32K3XX_CAN3_ERFFEL36              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL36_OFFSET)
#  define S32K3XX_CAN3_ERFFEL37              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL37_OFFSET)
#  define S32K3XX_CAN3_ERFFEL38              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL38_OFFSET)
#  define S32K3XX_CAN3_ERFFEL39              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL39_OFFSET)
#  define S32K3XX_CAN3_ERFFEL40              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL40_OFFSET)
#  define S32K3XX_CAN3_ERFFEL41              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL41_OFFSET)
#  define S32K3XX_CAN3_ERFFEL42              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL42_OFFSET)
#  define S32K3XX_CAN3_ERFFEL43              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL43_OFFSET)
#  define S32K3XX_CAN3_ERFFEL44              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL44_OFFSET)
#  define S32K3XX_CAN3_ERFFEL45              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL45_OFFSET)
#  define S32K3XX_CAN3_ERFFEL46              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL46_OFFSET)
#  define S32K3XX_CAN3_ERFFEL47              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL47_OFFSET)
#  define S32K3XX_CAN3_ERFFEL48              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL48_OFFSET)
#  define S32K3XX_CAN3_ERFFEL49              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL49_OFFSET)
#  define S32K3XX_CAN3_ERFFEL50              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL50_OFFSET)
#  define S32K3XX_CAN3_ERFFEL51              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL51_OFFSET)
#  define S32K3XX_CAN3_ERFFEL52              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL52_OFFSET)
#  define S32K3XX_CAN3_ERFFEL53              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL53_OFFSET)
#  define S32K3XX_CAN3_ERFFEL54              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL54_OFFSET)
#  define S32K3XX_CAN3_ERFFEL55              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL55_OFFSET)
#  define S32K3XX_CAN3_ERFFEL56              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL56_OFFSET)
#  define S32K3XX_CAN3_ERFFEL57              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL57_OFFSET)
#  define S32K3XX_CAN3_ERFFEL58              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL58_OFFSET)
#  define S32K3XX_CAN3_ERFFEL59              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL59_OFFSET)
#  define S32K3XX_CAN3_ERFFEL60              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL60_OFFSET)
#  define S32K3XX_CAN3_ERFFEL61              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL61_OFFSET)
#  define S32K3XX_CAN3_ERFFEL62              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL62_OFFSET)
#  define S32K3XX_CAN3_ERFFEL63              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL63_OFFSET)
#  define S32K3XX_CAN3_ERFFEL64              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL64_OFFSET)
#  define S32K3XX_CAN3_ERFFEL65              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL65_OFFSET)
#  define S32K3XX_CAN3_ERFFEL66              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL66_OFFSET)
#  define S32K3XX_CAN3_ERFFEL67              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL67_OFFSET)
#  define S32K3XX_CAN3_ERFFEL68              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL68_OFFSET)
#  define S32K3XX_CAN3_ERFFEL69              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL69_OFFSET)
#  define S32K3XX_CAN3_ERFFEL70              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL70_OFFSET)
#  define S32K3XX_CAN3_ERFFEL71              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL71_OFFSET)
#  define S32K3XX_CAN3_ERFFEL72              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL72_OFFSET)
#  define S32K3XX_CAN3_ERFFEL73              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL73_OFFSET)
#  define S32K3XX_CAN3_ERFFEL74              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL74_OFFSET)
#  define S32K3XX_CAN3_ERFFEL75              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL75_OFFSET)
#  define S32K3XX_CAN3_ERFFEL76              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL76_OFFSET)
#  define S32K3XX_CAN3_ERFFEL77              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL77_OFFSET)
#  define S32K3XX_CAN3_ERFFEL78              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL78_OFFSET)
#  define S32K3XX_CAN3_ERFFEL79              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL79_OFFSET)
#  define S32K3XX_CAN3_ERFFEL80              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL80_OFFSET)
#  define S32K3XX_CAN3_ERFFEL81              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL81_OFFSET)
#  define S32K3XX_CAN3_ERFFEL82              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL82_OFFSET)
#  define S32K3XX_CAN3_ERFFEL83              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL83_OFFSET)
#  define S32K3XX_CAN3_ERFFEL84              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL84_OFFSET)
#  define S32K3XX_CAN3_ERFFEL85              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL85_OFFSET)
#  define S32K3XX_CAN3_ERFFEL86              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL86_OFFSET)
#  define S32K3XX_CAN3_ERFFEL87              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL87_OFFSET)
#  define S32K3XX_CAN3_ERFFEL88              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL88_OFFSET)
#  define S32K3XX_CAN3_ERFFEL89              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL89_OFFSET)
#  define S32K3XX_CAN3_ERFFEL90              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL90_OFFSET)
#  define S32K3XX_CAN3_ERFFEL91              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL91_OFFSET)
#  define S32K3XX_CAN3_ERFFEL92              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL92_OFFSET)
#  define S32K3XX_CAN3_ERFFEL93              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL93_OFFSET)
#  define S32K3XX_CAN3_ERFFEL94              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL94_OFFSET)
#  define S32K3XX_CAN3_ERFFEL95              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL95_OFFSET)
#  define S32K3XX_CAN3_ERFFEL96              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL96_OFFSET)
#  define S32K3XX_CAN3_ERFFEL97              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL97_OFFSET)
#  define S32K3XX_CAN3_ERFFEL98              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL98_OFFSET)
#  define S32K3XX_CAN3_ERFFEL99              (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL99_OFFSET)
#  define S32K3XX_CAN3_ERFFEL100             (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL100_OFFSET)
#  define S32K3XX_CAN3_ERFFEL101             (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL101_OFFSET)
#  define S32K3XX_CAN3_ERFFEL102             (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL102_OFFSET)
#  define S32K3XX_CAN3_ERFFEL103             (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL103_OFFSET)
#  define S32K3XX_CAN3_ERFFEL104             (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL104_OFFSET)
#  define S32K3XX_CAN3_ERFFEL105             (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL105_OFFSET)
#  define S32K3XX_CAN3_ERFFEL106             (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL106_OFFSET)
#  define S32K3XX_CAN3_ERFFEL107             (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL107_OFFSET)
#  define S32K3XX_CAN3_ERFFEL108             (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL108_OFFSET)
#  define S32K3XX_CAN3_ERFFEL109             (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL109_OFFSET)
#  define S32K3XX_CAN3_ERFFEL110             (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL110_OFFSET)
#  define S32K3XX_CAN3_ERFFEL111             (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL111_OFFSET)
#  define S32K3XX_CAN3_ERFFEL112             (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL112_OFFSET)
#  define S32K3XX_CAN3_ERFFEL113             (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL113_OFFSET)
#  define S32K3XX_CAN3_ERFFEL114             (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL114_OFFSET)
#  define S32K3XX_CAN3_ERFFEL115             (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL115_OFFSET)
#  define S32K3XX_CAN3_ERFFEL116             (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL116_OFFSET)
#  define S32K3XX_CAN3_ERFFEL117             (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL117_OFFSET)
#  define S32K3XX_CAN3_ERFFEL118             (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL118_OFFSET)
#  define S32K3XX_CAN3_ERFFEL119             (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL119_OFFSET)
#  define S32K3XX_CAN3_ERFFEL120             (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL120_OFFSET)
#  define S32K3XX_CAN3_ERFFEL121             (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL121_OFFSET)
#  define S32K3XX_CAN3_ERFFEL122             (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL122_OFFSET)
#  define S32K3XX_CAN3_ERFFEL123             (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL123_OFFSET)
#  define S32K3XX_CAN3_ERFFEL124             (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL124_OFFSET)
#  define S32K3XX_CAN3_ERFFEL125             (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL125_OFFSET)
#  define S32K3XX_CAN3_ERFFEL126             (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL126_OFFSET)
#  define S32K3XX_CAN3_ERFFEL127             (S32K3XX_CAN3_BASE + S32K3XX_CAN_ERFFEL127_OFFSET)

/* FlexCAN4 */

#define S32K3XX_CAN4_MCR                     (S32K3XX_CAN4_BASE + S32K3XX_CAN_MCR_OFFSET)
#define S32K3XX_CAN4_CTRL1                   (S32K3XX_CAN4_BASE + S32K3XX_CAN_CTRL1_OFFSET)
#define S32K3XX_CAN4_TIMER                   (S32K3XX_CAN4_BASE + S32K3XX_CAN_TIMER_OFFSET)
#define S32K3XX_CAN4_RXMGMASK                (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXMGMASK_OFFSET)
#define S32K3XX_CAN4_RX14MASK                (S32K3XX_CAN4_BASE + S32K3XX_CAN_RX14MASK_OFFSET)
#define S32K3XX_CAN4_RX15MASK                (S32K3XX_CAN4_BASE + S32K3XX_CAN_RX15MASK_OFFSET)
#define S32K3XX_CAN4_ECR                     (S32K3XX_CAN4_BASE + S32K3XX_CAN_ECR_OFFSET)
#define S32K3XX_CAN4_ESR1                    (S32K3XX_CAN4_BASE + S32K3XX_CAN_ESR1_OFFSET)
#define S32K3XX_CAN4_IMASK2                  (S32K3XX_CAN4_BASE + S32K3XX_CAN_IMASK2_OFFSET)
#define S32K3XX_CAN4_IMASK1                  (S32K3XX_CAN4_BASE + S32K3XX_CAN_IMASK1_OFFSET)
#define S32K3XX_CAN4_IFLAG2                  (S32K3XX_CAN4_BASE + S32K3XX_CAN_IFLAG2_OFFSET)
#define S32K3XX_CAN4_IFLAG1                  (S32K3XX_CAN4_BASE + S32K3XX_CAN_IFLAG1_OFFSET)
#define S32K3XX_CAN4_CTRL2                   (S32K3XX_CAN4_BASE + S32K3XX_CAN_CTRL2_OFFSET)
#define S32K3XX_CAN4_ESR2                    (S32K3XX_CAN4_BASE + S32K3XX_CAN_ESR2_OFFSET)
#define S32K3XX_CAN4_CRCR                    (S32K3XX_CAN4_BASE + S32K3XX_CAN_CRCR_OFFSET)
#define S32K3XX_CAN4_RXFGMASK                (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXFGMASK_OFFSET)
#define S32K3XX_CAN4_RXFIR                   (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXFIR_OFFSET)
#define S32K3XX_CAN4_CBT                     (S32K3XX_CAN4_BASE + S32K3XX_CAN_CBT_OFFSET)
#define S32K3XX_CAN4_IMASK3                  (S32K3XX_CAN4_BASE + S32K3XX_CAN_IMASK3_OFFSET)
#define S32K3XX_CAN4_IFLAG3                  (S32K3XX_CAN4_BASE + S32K3XX_CAN_IFLAG3_OFFSET)

#define S32K3XX_CAN4_RXIMR(n)                (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR_OFFSET(n))
#  define S32K3XX_CAN4_RXIMR0                (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR0_OFFSET)
#  define S32K3XX_CAN4_RXIMR1                (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR1_OFFSET)
#  define S32K3XX_CAN4_RXIMR2                (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR2_OFFSET)
#  define S32K3XX_CAN4_RXIMR3                (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR3_OFFSET)
#  define S32K3XX_CAN4_RXIMR4                (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR4_OFFSET)
#  define S32K3XX_CAN4_RXIMR5                (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR5_OFFSET)
#  define S32K3XX_CAN4_RXIMR6                (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR6_OFFSET)
#  define S32K3XX_CAN4_RXIMR7                (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR7_OFFSET)
#  define S32K3XX_CAN4_RXIMR8                (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR8_OFFSET)
#  define S32K3XX_CAN4_RXIMR9                (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR9_OFFSET)
#  define S32K3XX_CAN4_RXIMR10               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR10_OFFSET)
#  define S32K3XX_CAN4_RXIMR11               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR11_OFFSET)
#  define S32K3XX_CAN4_RXIMR12               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR12_OFFSET)
#  define S32K3XX_CAN4_RXIMR13               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR13_OFFSET)
#  define S32K3XX_CAN4_RXIMR14               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR14_OFFSET)
#  define S32K3XX_CAN4_RXIMR15               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR15_OFFSET)
#  define S32K3XX_CAN4_RXIMR16               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR16_OFFSET)
#  define S32K3XX_CAN4_RXIMR17               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR17_OFFSET)
#  define S32K3XX_CAN4_RXIMR18               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR18_OFFSET)
#  define S32K3XX_CAN4_RXIMR19               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR19_OFFSET)
#  define S32K3XX_CAN4_RXIMR20               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR20_OFFSET)
#  define S32K3XX_CAN4_RXIMR21               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR21_OFFSET)
#  define S32K3XX_CAN4_RXIMR22               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR22_OFFSET)
#  define S32K3XX_CAN4_RXIMR23               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR23_OFFSET)
#  define S32K3XX_CAN4_RXIMR24               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR24_OFFSET)
#  define S32K3XX_CAN4_RXIMR25               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR25_OFFSET)
#  define S32K3XX_CAN4_RXIMR26               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR26_OFFSET)
#  define S32K3XX_CAN4_RXIMR27               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR27_OFFSET)
#  define S32K3XX_CAN4_RXIMR28               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR28_OFFSET)
#  define S32K3XX_CAN4_RXIMR29               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR29_OFFSET)
#  define S32K3XX_CAN4_RXIMR30               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR30_OFFSET)
#  define S32K3XX_CAN4_RXIMR31               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR31_OFFSET)
#  define S32K3XX_CAN4_RXIMR32               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR32_OFFSET)
#  define S32K3XX_CAN4_RXIMR33               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR33_OFFSET)
#  define S32K3XX_CAN4_RXIMR34               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR34_OFFSET)
#  define S32K3XX_CAN4_RXIMR35               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR35_OFFSET)
#  define S32K3XX_CAN4_RXIMR36               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR36_OFFSET)
#  define S32K3XX_CAN4_RXIMR37               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR37_OFFSET)
#  define S32K3XX_CAN4_RXIMR38               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR38_OFFSET)
#  define S32K3XX_CAN4_RXIMR39               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR39_OFFSET)
#  define S32K3XX_CAN4_RXIMR40               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR40_OFFSET)
#  define S32K3XX_CAN4_RXIMR41               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR41_OFFSET)
#  define S32K3XX_CAN4_RXIMR42               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR42_OFFSET)
#  define S32K3XX_CAN4_RXIMR43               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR43_OFFSET)
#  define S32K3XX_CAN4_RXIMR44               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR44_OFFSET)
#  define S32K3XX_CAN4_RXIMR45               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR45_OFFSET)
#  define S32K3XX_CAN4_RXIMR46               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR46_OFFSET)
#  define S32K3XX_CAN4_RXIMR47               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR47_OFFSET)
#  define S32K3XX_CAN4_RXIMR48               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR48_OFFSET)
#  define S32K3XX_CAN4_RXIMR49               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR49_OFFSET)
#  define S32K3XX_CAN4_RXIMR50               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR50_OFFSET)
#  define S32K3XX_CAN4_RXIMR51               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR51_OFFSET)
#  define S32K3XX_CAN4_RXIMR52               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR52_OFFSET)
#  define S32K3XX_CAN4_RXIMR53               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR53_OFFSET)
#  define S32K3XX_CAN4_RXIMR54               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR54_OFFSET)
#  define S32K3XX_CAN4_RXIMR55               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR55_OFFSET)
#  define S32K3XX_CAN4_RXIMR56               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR56_OFFSET)
#  define S32K3XX_CAN4_RXIMR57               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR57_OFFSET)
#  define S32K3XX_CAN4_RXIMR58               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR58_OFFSET)
#  define S32K3XX_CAN4_RXIMR59               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR59_OFFSET)
#  define S32K3XX_CAN4_RXIMR60               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR60_OFFSET)
#  define S32K3XX_CAN4_RXIMR61               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR61_OFFSET)
#  define S32K3XX_CAN4_RXIMR62               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR62_OFFSET)
#  define S32K3XX_CAN4_RXIMR63               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR63_OFFSET)
#  define S32K3XX_CAN4_RXIMR64               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR64_OFFSET)
#  define S32K3XX_CAN4_RXIMR65               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR65_OFFSET)
#  define S32K3XX_CAN4_RXIMR66               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR66_OFFSET)
#  define S32K3XX_CAN4_RXIMR67               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR67_OFFSET)
#  define S32K3XX_CAN4_RXIMR68               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR68_OFFSET)
#  define S32K3XX_CAN4_RXIMR69               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR69_OFFSET)
#  define S32K3XX_CAN4_RXIMR70               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR70_OFFSET)
#  define S32K3XX_CAN4_RXIMR71               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR71_OFFSET)
#  define S32K3XX_CAN4_RXIMR72               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR72_OFFSET)
#  define S32K3XX_CAN4_RXIMR73               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR73_OFFSET)
#  define S32K3XX_CAN4_RXIMR74               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR74_OFFSET)
#  define S32K3XX_CAN4_RXIMR75               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR75_OFFSET)
#  define S32K3XX_CAN4_RXIMR76               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR76_OFFSET)
#  define S32K3XX_CAN4_RXIMR77               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR77_OFFSET)
#  define S32K3XX_CAN4_RXIMR78               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR78_OFFSET)
#  define S32K3XX_CAN4_RXIMR79               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR79_OFFSET)
#  define S32K3XX_CAN4_RXIMR80               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR80_OFFSET)
#  define S32K3XX_CAN4_RXIMR81               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR81_OFFSET)
#  define S32K3XX_CAN4_RXIMR82               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR82_OFFSET)
#  define S32K3XX_CAN4_RXIMR83               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR83_OFFSET)
#  define S32K3XX_CAN4_RXIMR84               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR84_OFFSET)
#  define S32K3XX_CAN4_RXIMR85               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR85_OFFSET)
#  define S32K3XX_CAN4_RXIMR86               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR86_OFFSET)
#  define S32K3XX_CAN4_RXIMR87               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR87_OFFSET)
#  define S32K3XX_CAN4_RXIMR88               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR88_OFFSET)
#  define S32K3XX_CAN4_RXIMR89               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR89_OFFSET)
#  define S32K3XX_CAN4_RXIMR90               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR90_OFFSET)
#  define S32K3XX_CAN4_RXIMR91               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR91_OFFSET)
#  define S32K3XX_CAN4_RXIMR92               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR92_OFFSET)
#  define S32K3XX_CAN4_RXIMR93               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR93_OFFSET)
#  define S32K3XX_CAN4_RXIMR94               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR94_OFFSET)
#  define S32K3XX_CAN4_RXIMR95               (S32K3XX_CAN4_BASE + S32K3XX_CAN_RXIMR95_OFFSET)

#define S32K3XX_CAN4_MECR                    (S32K3XX_CAN4_BASE + S32K3XX_CAN_MECR_OFFSET)
#define S32K3XX_CAN4_ERRIAR                  (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERRIAR_OFFSET)
#define S32K3XX_CAN4_ERRIDPR                 (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERRIDPR_OFFSET)
#define S32K3XX_CAN4_ERRIPPR                 (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERRIPPR_OFFSET)
#define S32K3XX_CAN4_RERRAR                  (S32K3XX_CAN4_BASE + S32K3XX_CAN_RERRAR_OFFSET)
#define S32K3XX_CAN4_RERRDR                  (S32K3XX_CAN4_BASE + S32K3XX_CAN_RERRDR_OFFSET)
#define S32K3XX_CAN4_RERRSYNR                (S32K3XX_CAN4_BASE + S32K3XX_CAN_RERRSYNR_OFFSET)
#define S32K3XX_CAN4_ERRSR                   (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERRSR_OFFSET)
#define S32K3XX_CAN4_EPRS                    (S32K3XX_CAN4_BASE + S32K3XX_CAN_EPRS_OFFSET)
#define S32K3XX_CAN4_ENCBT                   (S32K3XX_CAN4_BASE + S32K3XX_CAN_ENCBT_OFFSET)
#define S32K3XX_CAN4_EDCBT                   (S32K3XX_CAN4_BASE + S32K3XX_CAN_EDCBT_OFFSET)
#define S32K3XX_CAN4_ETDC                    (S32K3XX_CAN4_BASE + S32K3XX_CAN_ETDC_OFFSET)
#define S32K3XX_CAN4_FDCTRL                  (S32K3XX_CAN4_BASE + S32K3XX_CAN_FDCTRL_OFFSET)
#define S32K3XX_CAN4_FDCBT                   (S32K3XX_CAN4_BASE + S32K3XX_CAN_FDCBT_OFFSET)
#define S32K3XX_CAN4_FDCRC                   (S32K3XX_CAN4_BASE + S32K3XX_CAN_FDCRC_OFFSET)
#define S32K3XX_CAN4_ERFCR                   (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFCR_OFFSET)
#define S32K3XX_CAN4_ERFIER                  (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFIER_OFFSET)
#define S32K3XX_CAN4_ERFSR                   (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFSR_OFFSET)

#define S32K3XX_CAN4_HR_TIME_STAMP(n)        (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP_OFFSET(n))  
#  define S32K3XX_CAN4_HR_TIME_STAMP0        (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP0_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP1        (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP1_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP2        (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP2_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP3        (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP3_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP4        (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP4_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP5        (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP5_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP6        (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP6_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP7        (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP7_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP8        (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP8_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP9        (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP9_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP10       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP10_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP11       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP11_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP12       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP12_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP13       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP13_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP14       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP14_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP15       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP15_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP16       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP16_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP17       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP17_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP18       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP18_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP19       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP19_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP20       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP20_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP21       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP21_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP22       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP22_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP23       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP23_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP24       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP24_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP25       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP25_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP26       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP26_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP27       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP27_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP28       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP28_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP29       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP29_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP30       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP30_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP31       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP31_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP32       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP32_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP33       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP33_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP34       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP34_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP35       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP35_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP36       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP36_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP37       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP37_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP38       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP38_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP39       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP39_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP40       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP40_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP41       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP41_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP42       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP42_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP43       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP43_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP44       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP44_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP45       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP45_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP46       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP46_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP47       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP47_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP48       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP48_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP49       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP49_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP50       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP50_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP51       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP51_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP52       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP52_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP53       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP53_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP54       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP54_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP55       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP55_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP56       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP56_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP57       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP57_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP58       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP58_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP59       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP59_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP60       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP60_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP61       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP61_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP62       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP62_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP63       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP63_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP64       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP64_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP65       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP65_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP66       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP66_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP67       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP67_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP68       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP68_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP69       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP69_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP70       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP70_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP71       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP71_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP72       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP72_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP73       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP73_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP74       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP74_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP75       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP75_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP76       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP76_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP77       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP77_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP78       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP78_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP79       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP79_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP80       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP80_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP81       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP81_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP82       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP82_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP83       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP83_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP84       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP84_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP85       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP85_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP86       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP86_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP87       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP87_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP88       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP88_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP89       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP89_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP90       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP90_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP91       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP91_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP92       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP92_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP93       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP93_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP94       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP94_OFFSET)
#  define S32K3XX_CAN4_HR_TIME_STAMP95       (S32K3XX_CAN4_BASE + S32K3XX_CAN_HR_TIME_STAMP95_OFFSET)

#define S32K3XX_CAN4_ERFFEL(n)               (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL_OFFSET(n))
#  define S32K3XX_CAN4_ERFFEL0               (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL0_OFFSET)
#  define S32K3XX_CAN4_ERFFEL1               (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL1_OFFSET)
#  define S32K3XX_CAN4_ERFFEL2               (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL2_OFFSET)
#  define S32K3XX_CAN4_ERFFEL3               (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL3_OFFSET)
#  define S32K3XX_CAN4_ERFFEL4               (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL4_OFFSET)
#  define S32K3XX_CAN4_ERFFEL5               (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL5_OFFSET)
#  define S32K3XX_CAN4_ERFFEL6               (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL6_OFFSET)
#  define S32K3XX_CAN4_ERFFEL7               (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL7_OFFSET)
#  define S32K3XX_CAN4_ERFFEL8               (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL8_OFFSET)
#  define S32K3XX_CAN4_ERFFEL9               (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL9_OFFSET)
#  define S32K3XX_CAN4_ERFFEL10              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL10_OFFSET)
#  define S32K3XX_CAN4_ERFFEL11              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL11_OFFSET)
#  define S32K3XX_CAN4_ERFFEL12              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL12_OFFSET)
#  define S32K3XX_CAN4_ERFFEL13              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL13_OFFSET)
#  define S32K3XX_CAN4_ERFFEL14              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL14_OFFSET)
#  define S32K3XX_CAN4_ERFFEL15              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL15_OFFSET)
#  define S32K3XX_CAN4_ERFFEL16              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL16_OFFSET)
#  define S32K3XX_CAN4_ERFFEL17              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL17_OFFSET)
#  define S32K3XX_CAN4_ERFFEL18              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL18_OFFSET)
#  define S32K3XX_CAN4_ERFFEL19              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL19_OFFSET)
#  define S32K3XX_CAN4_ERFFEL20              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL20_OFFSET)
#  define S32K3XX_CAN4_ERFFEL21              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL21_OFFSET)
#  define S32K3XX_CAN4_ERFFEL22              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL22_OFFSET)
#  define S32K3XX_CAN4_ERFFEL23              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL23_OFFSET)
#  define S32K3XX_CAN4_ERFFEL24              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL24_OFFSET)
#  define S32K3XX_CAN4_ERFFEL25              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL25_OFFSET)
#  define S32K3XX_CAN4_ERFFEL26              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL26_OFFSET)
#  define S32K3XX_CAN4_ERFFEL27              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL27_OFFSET)
#  define S32K3XX_CAN4_ERFFEL28              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL28_OFFSET)
#  define S32K3XX_CAN4_ERFFEL29              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL29_OFFSET)
#  define S32K3XX_CAN4_ERFFEL30              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL30_OFFSET)
#  define S32K3XX_CAN4_ERFFEL31              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL31_OFFSET)
#  define S32K3XX_CAN4_ERFFEL32              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL32_OFFSET)
#  define S32K3XX_CAN4_ERFFEL33              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL33_OFFSET)
#  define S32K3XX_CAN4_ERFFEL34              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL34_OFFSET)
#  define S32K3XX_CAN4_ERFFEL35              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL35_OFFSET)
#  define S32K3XX_CAN4_ERFFEL36              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL36_OFFSET)
#  define S32K3XX_CAN4_ERFFEL37              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL37_OFFSET)
#  define S32K3XX_CAN4_ERFFEL38              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL38_OFFSET)
#  define S32K3XX_CAN4_ERFFEL39              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL39_OFFSET)
#  define S32K3XX_CAN4_ERFFEL40              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL40_OFFSET)
#  define S32K3XX_CAN4_ERFFEL41              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL41_OFFSET)
#  define S32K3XX_CAN4_ERFFEL42              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL42_OFFSET)
#  define S32K3XX_CAN4_ERFFEL43              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL43_OFFSET)
#  define S32K3XX_CAN4_ERFFEL44              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL44_OFFSET)
#  define S32K3XX_CAN4_ERFFEL45              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL45_OFFSET)
#  define S32K3XX_CAN4_ERFFEL46              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL46_OFFSET)
#  define S32K3XX_CAN4_ERFFEL47              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL47_OFFSET)
#  define S32K3XX_CAN4_ERFFEL48              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL48_OFFSET)
#  define S32K3XX_CAN4_ERFFEL49              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL49_OFFSET)
#  define S32K3XX_CAN4_ERFFEL50              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL50_OFFSET)
#  define S32K3XX_CAN4_ERFFEL51              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL51_OFFSET)
#  define S32K3XX_CAN4_ERFFEL52              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL52_OFFSET)
#  define S32K3XX_CAN4_ERFFEL53              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL53_OFFSET)
#  define S32K3XX_CAN4_ERFFEL54              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL54_OFFSET)
#  define S32K3XX_CAN4_ERFFEL55              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL55_OFFSET)
#  define S32K3XX_CAN4_ERFFEL56              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL56_OFFSET)
#  define S32K3XX_CAN4_ERFFEL57              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL57_OFFSET)
#  define S32K3XX_CAN4_ERFFEL58              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL58_OFFSET)
#  define S32K3XX_CAN4_ERFFEL59              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL59_OFFSET)
#  define S32K3XX_CAN4_ERFFEL60              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL60_OFFSET)
#  define S32K3XX_CAN4_ERFFEL61              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL61_OFFSET)
#  define S32K3XX_CAN4_ERFFEL62              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL62_OFFSET)
#  define S32K3XX_CAN4_ERFFEL63              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL63_OFFSET)
#  define S32K3XX_CAN4_ERFFEL64              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL64_OFFSET)
#  define S32K3XX_CAN4_ERFFEL65              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL65_OFFSET)
#  define S32K3XX_CAN4_ERFFEL66              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL66_OFFSET)
#  define S32K3XX_CAN4_ERFFEL67              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL67_OFFSET)
#  define S32K3XX_CAN4_ERFFEL68              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL68_OFFSET)
#  define S32K3XX_CAN4_ERFFEL69              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL69_OFFSET)
#  define S32K3XX_CAN4_ERFFEL70              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL70_OFFSET)
#  define S32K3XX_CAN4_ERFFEL71              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL71_OFFSET)
#  define S32K3XX_CAN4_ERFFEL72              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL72_OFFSET)
#  define S32K3XX_CAN4_ERFFEL73              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL73_OFFSET)
#  define S32K3XX_CAN4_ERFFEL74              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL74_OFFSET)
#  define S32K3XX_CAN4_ERFFEL75              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL75_OFFSET)
#  define S32K3XX_CAN4_ERFFEL76              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL76_OFFSET)
#  define S32K3XX_CAN4_ERFFEL77              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL77_OFFSET)
#  define S32K3XX_CAN4_ERFFEL78              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL78_OFFSET)
#  define S32K3XX_CAN4_ERFFEL79              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL79_OFFSET)
#  define S32K3XX_CAN4_ERFFEL80              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL80_OFFSET)
#  define S32K3XX_CAN4_ERFFEL81              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL81_OFFSET)
#  define S32K3XX_CAN4_ERFFEL82              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL82_OFFSET)
#  define S32K3XX_CAN4_ERFFEL83              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL83_OFFSET)
#  define S32K3XX_CAN4_ERFFEL84              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL84_OFFSET)
#  define S32K3XX_CAN4_ERFFEL85              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL85_OFFSET)
#  define S32K3XX_CAN4_ERFFEL86              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL86_OFFSET)
#  define S32K3XX_CAN4_ERFFEL87              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL87_OFFSET)
#  define S32K3XX_CAN4_ERFFEL88              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL88_OFFSET)
#  define S32K3XX_CAN4_ERFFEL89              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL89_OFFSET)
#  define S32K3XX_CAN4_ERFFEL90              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL90_OFFSET)
#  define S32K3XX_CAN4_ERFFEL91              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL91_OFFSET)
#  define S32K3XX_CAN4_ERFFEL92              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL92_OFFSET)
#  define S32K3XX_CAN4_ERFFEL93              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL93_OFFSET)
#  define S32K3XX_CAN4_ERFFEL94              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL94_OFFSET)
#  define S32K3XX_CAN4_ERFFEL95              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL95_OFFSET)
#  define S32K3XX_CAN4_ERFFEL96              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL96_OFFSET)
#  define S32K3XX_CAN4_ERFFEL97              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL97_OFFSET)
#  define S32K3XX_CAN4_ERFFEL98              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL98_OFFSET)
#  define S32K3XX_CAN4_ERFFEL99              (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL99_OFFSET)
#  define S32K3XX_CAN4_ERFFEL100             (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL100_OFFSET)
#  define S32K3XX_CAN4_ERFFEL101             (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL101_OFFSET)
#  define S32K3XX_CAN4_ERFFEL102             (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL102_OFFSET)
#  define S32K3XX_CAN4_ERFFEL103             (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL103_OFFSET)
#  define S32K3XX_CAN4_ERFFEL104             (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL104_OFFSET)
#  define S32K3XX_CAN4_ERFFEL105             (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL105_OFFSET)
#  define S32K3XX_CAN4_ERFFEL106             (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL106_OFFSET)
#  define S32K3XX_CAN4_ERFFEL107             (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL107_OFFSET)
#  define S32K3XX_CAN4_ERFFEL108             (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL108_OFFSET)
#  define S32K3XX_CAN4_ERFFEL109             (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL109_OFFSET)
#  define S32K3XX_CAN4_ERFFEL110             (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL110_OFFSET)
#  define S32K3XX_CAN4_ERFFEL111             (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL111_OFFSET)
#  define S32K3XX_CAN4_ERFFEL112             (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL112_OFFSET)
#  define S32K3XX_CAN4_ERFFEL113             (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL113_OFFSET)
#  define S32K3XX_CAN4_ERFFEL114             (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL114_OFFSET)
#  define S32K3XX_CAN4_ERFFEL115             (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL115_OFFSET)
#  define S32K3XX_CAN4_ERFFEL116             (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL116_OFFSET)
#  define S32K3XX_CAN4_ERFFEL117             (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL117_OFFSET)
#  define S32K3XX_CAN4_ERFFEL118             (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL118_OFFSET)
#  define S32K3XX_CAN4_ERFFEL119             (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL119_OFFSET)
#  define S32K3XX_CAN4_ERFFEL120             (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL120_OFFSET)
#  define S32K3XX_CAN4_ERFFEL121             (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL121_OFFSET)
#  define S32K3XX_CAN4_ERFFEL122             (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL122_OFFSET)
#  define S32K3XX_CAN4_ERFFEL123             (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL123_OFFSET)
#  define S32K3XX_CAN4_ERFFEL124             (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL124_OFFSET)
#  define S32K3XX_CAN4_ERFFEL125             (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL125_OFFSET)
#  define S32K3XX_CAN4_ERFFEL126             (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL126_OFFSET)
#  define S32K3XX_CAN4_ERFFEL127             (S32K3XX_CAN4_BASE + S32K3XX_CAN_ERFFEL127_OFFSET)

/* FlexCAN5 */

#define S32K3XX_CAN5_MCR                     (S32K3XX_CAN5_BASE + S32K3XX_CAN_MCR_OFFSET)
#define S32K3XX_CAN5_CTRL1                   (S32K3XX_CAN5_BASE + S32K3XX_CAN_CTRL1_OFFSET)
#define S32K3XX_CAN5_TIMER                   (S32K3XX_CAN5_BASE + S32K3XX_CAN_TIMER_OFFSET)
#define S32K3XX_CAN5_RXMGMASK                (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXMGMASK_OFFSET)
#define S32K3XX_CAN5_RX14MASK                (S32K3XX_CAN5_BASE + S32K3XX_CAN_RX14MASK_OFFSET)
#define S32K3XX_CAN5_RX15MASK                (S32K3XX_CAN5_BASE + S32K3XX_CAN_RX15MASK_OFFSET)
#define S32K3XX_CAN5_ECR                     (S32K3XX_CAN5_BASE + S32K3XX_CAN_ECR_OFFSET)
#define S32K3XX_CAN5_ESR1                    (S32K3XX_CAN5_BASE + S32K3XX_CAN_ESR1_OFFSET)
#define S32K3XX_CAN5_IMASK2                  (S32K3XX_CAN5_BASE + S32K3XX_CAN_IMASK2_OFFSET)
#define S32K3XX_CAN5_IMASK1                  (S32K3XX_CAN5_BASE + S32K3XX_CAN_IMASK1_OFFSET)
#define S32K3XX_CAN5_IFLAG2                  (S32K3XX_CAN5_BASE + S32K3XX_CAN_IFLAG2_OFFSET)
#define S32K3XX_CAN5_IFLAG1                  (S32K3XX_CAN5_BASE + S32K3XX_CAN_IFLAG1_OFFSET)
#define S32K3XX_CAN5_CTRL2                   (S32K3XX_CAN5_BASE + S32K3XX_CAN_CTRL2_OFFSET)
#define S32K3XX_CAN5_ESR2                    (S32K3XX_CAN5_BASE + S32K3XX_CAN_ESR2_OFFSET)
#define S32K3XX_CAN5_CRCR                    (S32K3XX_CAN5_BASE + S32K3XX_CAN_CRCR_OFFSET)
#define S32K3XX_CAN5_RXFGMASK                (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXFGMASK_OFFSET)
#define S32K3XX_CAN5_RXFIR                   (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXFIR_OFFSET)
#define S32K3XX_CAN5_CBT                     (S32K3XX_CAN5_BASE + S32K3XX_CAN_CBT_OFFSET)
#define S32K3XX_CAN5_IMASK3                  (S32K3XX_CAN5_BASE + S32K3XX_CAN_IMASK3_OFFSET)
#define S32K3XX_CAN5_IFLAG3                  (S32K3XX_CAN5_BASE + S32K3XX_CAN_IFLAG3_OFFSET)

#define S32K3XX_CAN5_RXIMR(n)                (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR_OFFSET(n))
#  define S32K3XX_CAN5_RXIMR0                (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR0_OFFSET)
#  define S32K3XX_CAN5_RXIMR1                (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR1_OFFSET)
#  define S32K3XX_CAN5_RXIMR2                (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR2_OFFSET)
#  define S32K3XX_CAN5_RXIMR3                (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR3_OFFSET)
#  define S32K3XX_CAN5_RXIMR4                (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR4_OFFSET)
#  define S32K3XX_CAN5_RXIMR5                (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR5_OFFSET)
#  define S32K3XX_CAN5_RXIMR6                (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR6_OFFSET)
#  define S32K3XX_CAN5_RXIMR7                (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR7_OFFSET)
#  define S32K3XX_CAN5_RXIMR8                (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR8_OFFSET)
#  define S32K3XX_CAN5_RXIMR9                (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR9_OFFSET)
#  define S32K3XX_CAN5_RXIMR10               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR10_OFFSET)
#  define S32K3XX_CAN5_RXIMR11               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR11_OFFSET)
#  define S32K3XX_CAN5_RXIMR12               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR12_OFFSET)
#  define S32K3XX_CAN5_RXIMR13               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR13_OFFSET)
#  define S32K3XX_CAN5_RXIMR14               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR14_OFFSET)
#  define S32K3XX_CAN5_RXIMR15               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR15_OFFSET)
#  define S32K3XX_CAN5_RXIMR16               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR16_OFFSET)
#  define S32K3XX_CAN5_RXIMR17               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR17_OFFSET)
#  define S32K3XX_CAN5_RXIMR18               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR18_OFFSET)
#  define S32K3XX_CAN5_RXIMR19               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR19_OFFSET)
#  define S32K3XX_CAN5_RXIMR20               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR20_OFFSET)
#  define S32K3XX_CAN5_RXIMR21               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR21_OFFSET)
#  define S32K3XX_CAN5_RXIMR22               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR22_OFFSET)
#  define S32K3XX_CAN5_RXIMR23               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR23_OFFSET)
#  define S32K3XX_CAN5_RXIMR24               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR24_OFFSET)
#  define S32K3XX_CAN5_RXIMR25               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR25_OFFSET)
#  define S32K3XX_CAN5_RXIMR26               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR26_OFFSET)
#  define S32K3XX_CAN5_RXIMR27               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR27_OFFSET)
#  define S32K3XX_CAN5_RXIMR28               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR28_OFFSET)
#  define S32K3XX_CAN5_RXIMR29               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR29_OFFSET)
#  define S32K3XX_CAN5_RXIMR30               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR30_OFFSET)
#  define S32K3XX_CAN5_RXIMR31               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR31_OFFSET)
#  define S32K3XX_CAN5_RXIMR32               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR32_OFFSET)
#  define S32K3XX_CAN5_RXIMR33               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR33_OFFSET)
#  define S32K3XX_CAN5_RXIMR34               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR34_OFFSET)
#  define S32K3XX_CAN5_RXIMR35               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR35_OFFSET)
#  define S32K3XX_CAN5_RXIMR36               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR36_OFFSET)
#  define S32K3XX_CAN5_RXIMR37               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR37_OFFSET)
#  define S32K3XX_CAN5_RXIMR38               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR38_OFFSET)
#  define S32K3XX_CAN5_RXIMR39               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR39_OFFSET)
#  define S32K3XX_CAN5_RXIMR40               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR40_OFFSET)
#  define S32K3XX_CAN5_RXIMR41               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR41_OFFSET)
#  define S32K3XX_CAN5_RXIMR42               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR42_OFFSET)
#  define S32K3XX_CAN5_RXIMR43               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR43_OFFSET)
#  define S32K3XX_CAN5_RXIMR44               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR44_OFFSET)
#  define S32K3XX_CAN5_RXIMR45               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR45_OFFSET)
#  define S32K3XX_CAN5_RXIMR46               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR46_OFFSET)
#  define S32K3XX_CAN5_RXIMR47               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR47_OFFSET)
#  define S32K3XX_CAN5_RXIMR48               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR48_OFFSET)
#  define S32K3XX_CAN5_RXIMR49               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR49_OFFSET)
#  define S32K3XX_CAN5_RXIMR50               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR50_OFFSET)
#  define S32K3XX_CAN5_RXIMR51               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR51_OFFSET)
#  define S32K3XX_CAN5_RXIMR52               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR52_OFFSET)
#  define S32K3XX_CAN5_RXIMR53               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR53_OFFSET)
#  define S32K3XX_CAN5_RXIMR54               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR54_OFFSET)
#  define S32K3XX_CAN5_RXIMR55               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR55_OFFSET)
#  define S32K3XX_CAN5_RXIMR56               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR56_OFFSET)
#  define S32K3XX_CAN5_RXIMR57               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR57_OFFSET)
#  define S32K3XX_CAN5_RXIMR58               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR58_OFFSET)
#  define S32K3XX_CAN5_RXIMR59               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR59_OFFSET)
#  define S32K3XX_CAN5_RXIMR60               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR60_OFFSET)
#  define S32K3XX_CAN5_RXIMR61               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR61_OFFSET)
#  define S32K3XX_CAN5_RXIMR62               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR62_OFFSET)
#  define S32K3XX_CAN5_RXIMR63               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR63_OFFSET)
#  define S32K3XX_CAN5_RXIMR64               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR64_OFFSET)
#  define S32K3XX_CAN5_RXIMR65               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR65_OFFSET)
#  define S32K3XX_CAN5_RXIMR66               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR66_OFFSET)
#  define S32K3XX_CAN5_RXIMR67               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR67_OFFSET)
#  define S32K3XX_CAN5_RXIMR68               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR68_OFFSET)
#  define S32K3XX_CAN5_RXIMR69               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR69_OFFSET)
#  define S32K3XX_CAN5_RXIMR70               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR70_OFFSET)
#  define S32K3XX_CAN5_RXIMR71               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR71_OFFSET)
#  define S32K3XX_CAN5_RXIMR72               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR72_OFFSET)
#  define S32K3XX_CAN5_RXIMR73               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR73_OFFSET)
#  define S32K3XX_CAN5_RXIMR74               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR74_OFFSET)
#  define S32K3XX_CAN5_RXIMR75               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR75_OFFSET)
#  define S32K3XX_CAN5_RXIMR76               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR76_OFFSET)
#  define S32K3XX_CAN5_RXIMR77               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR77_OFFSET)
#  define S32K3XX_CAN5_RXIMR78               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR78_OFFSET)
#  define S32K3XX_CAN5_RXIMR79               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR79_OFFSET)
#  define S32K3XX_CAN5_RXIMR80               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR80_OFFSET)
#  define S32K3XX_CAN5_RXIMR81               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR81_OFFSET)
#  define S32K3XX_CAN5_RXIMR82               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR82_OFFSET)
#  define S32K3XX_CAN5_RXIMR83               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR83_OFFSET)
#  define S32K3XX_CAN5_RXIMR84               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR84_OFFSET)
#  define S32K3XX_CAN5_RXIMR85               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR85_OFFSET)
#  define S32K3XX_CAN5_RXIMR86               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR86_OFFSET)
#  define S32K3XX_CAN5_RXIMR87               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR87_OFFSET)
#  define S32K3XX_CAN5_RXIMR88               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR88_OFFSET)
#  define S32K3XX_CAN5_RXIMR89               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR89_OFFSET)
#  define S32K3XX_CAN5_RXIMR90               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR90_OFFSET)
#  define S32K3XX_CAN5_RXIMR91               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR91_OFFSET)
#  define S32K3XX_CAN5_RXIMR92               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR92_OFFSET)
#  define S32K3XX_CAN5_RXIMR93               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR93_OFFSET)
#  define S32K3XX_CAN5_RXIMR94               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR94_OFFSET)
#  define S32K3XX_CAN5_RXIMR95               (S32K3XX_CAN5_BASE + S32K3XX_CAN_RXIMR95_OFFSET)

#define S32K3XX_CAN5_MECR                    (S32K3XX_CAN5_BASE + S32K3XX_CAN_MECR_OFFSET)
#define S32K3XX_CAN5_ERRIAR                  (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERRIAR_OFFSET)
#define S32K3XX_CAN5_ERRIDPR                 (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERRIDPR_OFFSET)
#define S32K3XX_CAN5_ERRIPPR                 (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERRIPPR_OFFSET)
#define S32K3XX_CAN5_RERRAR                  (S32K3XX_CAN5_BASE + S32K3XX_CAN_RERRAR_OFFSET)
#define S32K3XX_CAN5_RERRDR                  (S32K3XX_CAN5_BASE + S32K3XX_CAN_RERRDR_OFFSET)
#define S32K3XX_CAN5_RERRSYNR                (S32K3XX_CAN5_BASE + S32K3XX_CAN_RERRSYNR_OFFSET)
#define S32K3XX_CAN5_ERRSR                   (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERRSR_OFFSET)
#define S32K3XX_CAN5_EPRS                    (S32K3XX_CAN5_BASE + S32K3XX_CAN_EPRS_OFFSET)
#define S32K3XX_CAN5_ENCBT                   (S32K3XX_CAN5_BASE + S32K3XX_CAN_ENCBT_OFFSET)
#define S32K3XX_CAN5_EDCBT                   (S32K3XX_CAN5_BASE + S32K3XX_CAN_EDCBT_OFFSET)
#define S32K3XX_CAN5_ETDC                    (S32K3XX_CAN5_BASE + S32K3XX_CAN_ETDC_OFFSET)
#define S32K3XX_CAN5_FDCTRL                  (S32K3XX_CAN5_BASE + S32K3XX_CAN_FDCTRL_OFFSET)
#define S32K3XX_CAN5_FDCBT                   (S32K3XX_CAN5_BASE + S32K3XX_CAN_FDCBT_OFFSET)
#define S32K3XX_CAN5_FDCRC                   (S32K3XX_CAN5_BASE + S32K3XX_CAN_FDCRC_OFFSET)
#define S32K3XX_CAN5_ERFCR                   (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFCR_OFFSET)
#define S32K3XX_CAN5_ERFIER                  (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFIER_OFFSET)
#define S32K3XX_CAN5_ERFSR                   (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFSR_OFFSET)

#define S32K3XX_CAN5_HR_TIME_STAMP(n)        (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP_OFFSET(n))  
#  define S32K3XX_CAN5_HR_TIME_STAMP0        (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP0_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP1        (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP1_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP2        (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP2_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP3        (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP3_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP4        (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP4_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP5        (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP5_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP6        (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP6_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP7        (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP7_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP8        (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP8_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP9        (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP9_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP10       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP10_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP11       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP11_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP12       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP12_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP13       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP13_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP14       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP14_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP15       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP15_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP16       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP16_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP17       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP17_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP18       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP18_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP19       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP19_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP20       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP20_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP21       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP21_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP22       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP22_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP23       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP23_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP24       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP24_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP25       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP25_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP26       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP26_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP27       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP27_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP28       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP28_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP29       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP29_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP30       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP30_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP31       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP31_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP32       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP32_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP33       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP33_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP34       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP34_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP35       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP35_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP36       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP36_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP37       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP37_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP38       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP38_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP39       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP39_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP40       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP40_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP41       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP41_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP42       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP42_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP43       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP43_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP44       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP44_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP45       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP45_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP46       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP46_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP47       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP47_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP48       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP48_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP49       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP49_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP50       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP50_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP51       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP51_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP52       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP52_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP53       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP53_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP54       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP54_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP55       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP55_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP56       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP56_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP57       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP57_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP58       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP58_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP59       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP59_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP60       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP60_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP61       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP61_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP62       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP62_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP63       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP63_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP64       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP64_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP65       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP65_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP66       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP66_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP67       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP67_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP68       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP68_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP69       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP69_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP70       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP70_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP71       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP71_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP72       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP72_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP73       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP73_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP74       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP74_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP75       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP75_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP76       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP76_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP77       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP77_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP78       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP78_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP79       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP79_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP80       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP80_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP81       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP81_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP82       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP82_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP83       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP83_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP84       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP84_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP85       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP85_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP86       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP86_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP87       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP87_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP88       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP88_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP89       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP89_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP90       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP90_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP91       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP91_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP92       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP92_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP93       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP93_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP94       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP94_OFFSET)
#  define S32K3XX_CAN5_HR_TIME_STAMP95       (S32K3XX_CAN5_BASE + S32K3XX_CAN_HR_TIME_STAMP95_OFFSET)

#define S32K3XX_CAN5_ERFFEL(n)               (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL_OFFSET(n))
#  define S32K3XX_CAN5_ERFFEL0               (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL0_OFFSET)
#  define S32K3XX_CAN5_ERFFEL1               (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL1_OFFSET)
#  define S32K3XX_CAN5_ERFFEL2               (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL2_OFFSET)
#  define S32K3XX_CAN5_ERFFEL3               (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL3_OFFSET)
#  define S32K3XX_CAN5_ERFFEL4               (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL4_OFFSET)
#  define S32K3XX_CAN5_ERFFEL5               (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL5_OFFSET)
#  define S32K3XX_CAN5_ERFFEL6               (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL6_OFFSET)
#  define S32K3XX_CAN5_ERFFEL7               (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL7_OFFSET)
#  define S32K3XX_CAN5_ERFFEL8               (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL8_OFFSET)
#  define S32K3XX_CAN5_ERFFEL9               (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL9_OFFSET)
#  define S32K3XX_CAN5_ERFFEL10              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL10_OFFSET)
#  define S32K3XX_CAN5_ERFFEL11              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL11_OFFSET)
#  define S32K3XX_CAN5_ERFFEL12              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL12_OFFSET)
#  define S32K3XX_CAN5_ERFFEL13              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL13_OFFSET)
#  define S32K3XX_CAN5_ERFFEL14              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL14_OFFSET)
#  define S32K3XX_CAN5_ERFFEL15              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL15_OFFSET)
#  define S32K3XX_CAN5_ERFFEL16              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL16_OFFSET)
#  define S32K3XX_CAN5_ERFFEL17              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL17_OFFSET)
#  define S32K3XX_CAN5_ERFFEL18              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL18_OFFSET)
#  define S32K3XX_CAN5_ERFFEL19              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL19_OFFSET)
#  define S32K3XX_CAN5_ERFFEL20              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL20_OFFSET)
#  define S32K3XX_CAN5_ERFFEL21              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL21_OFFSET)
#  define S32K3XX_CAN5_ERFFEL22              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL22_OFFSET)
#  define S32K3XX_CAN5_ERFFEL23              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL23_OFFSET)
#  define S32K3XX_CAN5_ERFFEL24              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL24_OFFSET)
#  define S32K3XX_CAN5_ERFFEL25              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL25_OFFSET)
#  define S32K3XX_CAN5_ERFFEL26              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL26_OFFSET)
#  define S32K3XX_CAN5_ERFFEL27              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL27_OFFSET)
#  define S32K3XX_CAN5_ERFFEL28              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL28_OFFSET)
#  define S32K3XX_CAN5_ERFFEL29              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL29_OFFSET)
#  define S32K3XX_CAN5_ERFFEL30              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL30_OFFSET)
#  define S32K3XX_CAN5_ERFFEL31              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL31_OFFSET)
#  define S32K3XX_CAN5_ERFFEL32              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL32_OFFSET)
#  define S32K3XX_CAN5_ERFFEL33              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL33_OFFSET)
#  define S32K3XX_CAN5_ERFFEL34              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL34_OFFSET)
#  define S32K3XX_CAN5_ERFFEL35              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL35_OFFSET)
#  define S32K3XX_CAN5_ERFFEL36              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL36_OFFSET)
#  define S32K3XX_CAN5_ERFFEL37              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL37_OFFSET)
#  define S32K3XX_CAN5_ERFFEL38              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL38_OFFSET)
#  define S32K3XX_CAN5_ERFFEL39              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL39_OFFSET)
#  define S32K3XX_CAN5_ERFFEL40              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL40_OFFSET)
#  define S32K3XX_CAN5_ERFFEL41              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL41_OFFSET)
#  define S32K3XX_CAN5_ERFFEL42              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL42_OFFSET)
#  define S32K3XX_CAN5_ERFFEL43              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL43_OFFSET)
#  define S32K3XX_CAN5_ERFFEL44              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL44_OFFSET)
#  define S32K3XX_CAN5_ERFFEL45              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL45_OFFSET)
#  define S32K3XX_CAN5_ERFFEL46              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL46_OFFSET)
#  define S32K3XX_CAN5_ERFFEL47              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL47_OFFSET)
#  define S32K3XX_CAN5_ERFFEL48              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL48_OFFSET)
#  define S32K3XX_CAN5_ERFFEL49              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL49_OFFSET)
#  define S32K3XX_CAN5_ERFFEL50              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL50_OFFSET)
#  define S32K3XX_CAN5_ERFFEL51              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL51_OFFSET)
#  define S32K3XX_CAN5_ERFFEL52              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL52_OFFSET)
#  define S32K3XX_CAN5_ERFFEL53              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL53_OFFSET)
#  define S32K3XX_CAN5_ERFFEL54              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL54_OFFSET)
#  define S32K3XX_CAN5_ERFFEL55              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL55_OFFSET)
#  define S32K3XX_CAN5_ERFFEL56              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL56_OFFSET)
#  define S32K3XX_CAN5_ERFFEL57              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL57_OFFSET)
#  define S32K3XX_CAN5_ERFFEL58              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL58_OFFSET)
#  define S32K3XX_CAN5_ERFFEL59              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL59_OFFSET)
#  define S32K3XX_CAN5_ERFFEL60              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL60_OFFSET)
#  define S32K3XX_CAN5_ERFFEL61              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL61_OFFSET)
#  define S32K3XX_CAN5_ERFFEL62              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL62_OFFSET)
#  define S32K3XX_CAN5_ERFFEL63              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL63_OFFSET)
#  define S32K3XX_CAN5_ERFFEL64              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL64_OFFSET)
#  define S32K3XX_CAN5_ERFFEL65              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL65_OFFSET)
#  define S32K3XX_CAN5_ERFFEL66              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL66_OFFSET)
#  define S32K3XX_CAN5_ERFFEL67              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL67_OFFSET)
#  define S32K3XX_CAN5_ERFFEL68              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL68_OFFSET)
#  define S32K3XX_CAN5_ERFFEL69              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL69_OFFSET)
#  define S32K3XX_CAN5_ERFFEL70              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL70_OFFSET)
#  define S32K3XX_CAN5_ERFFEL71              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL71_OFFSET)
#  define S32K3XX_CAN5_ERFFEL72              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL72_OFFSET)
#  define S32K3XX_CAN5_ERFFEL73              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL73_OFFSET)
#  define S32K3XX_CAN5_ERFFEL74              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL74_OFFSET)
#  define S32K3XX_CAN5_ERFFEL75              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL75_OFFSET)
#  define S32K3XX_CAN5_ERFFEL76              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL76_OFFSET)
#  define S32K3XX_CAN5_ERFFEL77              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL77_OFFSET)
#  define S32K3XX_CAN5_ERFFEL78              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL78_OFFSET)
#  define S32K3XX_CAN5_ERFFEL79              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL79_OFFSET)
#  define S32K3XX_CAN5_ERFFEL80              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL80_OFFSET)
#  define S32K3XX_CAN5_ERFFEL81              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL81_OFFSET)
#  define S32K3XX_CAN5_ERFFEL82              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL82_OFFSET)
#  define S32K3XX_CAN5_ERFFEL83              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL83_OFFSET)
#  define S32K3XX_CAN5_ERFFEL84              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL84_OFFSET)
#  define S32K3XX_CAN5_ERFFEL85              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL85_OFFSET)
#  define S32K3XX_CAN5_ERFFEL86              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL86_OFFSET)
#  define S32K3XX_CAN5_ERFFEL87              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL87_OFFSET)
#  define S32K3XX_CAN5_ERFFEL88              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL88_OFFSET)
#  define S32K3XX_CAN5_ERFFEL89              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL89_OFFSET)
#  define S32K3XX_CAN5_ERFFEL90              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL90_OFFSET)
#  define S32K3XX_CAN5_ERFFEL91              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL91_OFFSET)
#  define S32K3XX_CAN5_ERFFEL92              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL92_OFFSET)
#  define S32K3XX_CAN5_ERFFEL93              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL93_OFFSET)
#  define S32K3XX_CAN5_ERFFEL94              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL94_OFFSET)
#  define S32K3XX_CAN5_ERFFEL95              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL95_OFFSET)
#  define S32K3XX_CAN5_ERFFEL96              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL96_OFFSET)
#  define S32K3XX_CAN5_ERFFEL97              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL97_OFFSET)
#  define S32K3XX_CAN5_ERFFEL98              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL98_OFFSET)
#  define S32K3XX_CAN5_ERFFEL99              (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL99_OFFSET)
#  define S32K3XX_CAN5_ERFFEL100             (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL100_OFFSET)
#  define S32K3XX_CAN5_ERFFEL101             (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL101_OFFSET)
#  define S32K3XX_CAN5_ERFFEL102             (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL102_OFFSET)
#  define S32K3XX_CAN5_ERFFEL103             (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL103_OFFSET)
#  define S32K3XX_CAN5_ERFFEL104             (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL104_OFFSET)
#  define S32K3XX_CAN5_ERFFEL105             (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL105_OFFSET)
#  define S32K3XX_CAN5_ERFFEL106             (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL106_OFFSET)
#  define S32K3XX_CAN5_ERFFEL107             (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL107_OFFSET)
#  define S32K3XX_CAN5_ERFFEL108             (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL108_OFFSET)
#  define S32K3XX_CAN5_ERFFEL109             (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL109_OFFSET)
#  define S32K3XX_CAN5_ERFFEL110             (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL110_OFFSET)
#  define S32K3XX_CAN5_ERFFEL111             (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL111_OFFSET)
#  define S32K3XX_CAN5_ERFFEL112             (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL112_OFFSET)
#  define S32K3XX_CAN5_ERFFEL113             (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL113_OFFSET)
#  define S32K3XX_CAN5_ERFFEL114             (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL114_OFFSET)
#  define S32K3XX_CAN5_ERFFEL115             (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL115_OFFSET)
#  define S32K3XX_CAN5_ERFFEL116             (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL116_OFFSET)
#  define S32K3XX_CAN5_ERFFEL117             (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL117_OFFSET)
#  define S32K3XX_CAN5_ERFFEL118             (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL118_OFFSET)
#  define S32K3XX_CAN5_ERFFEL119             (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL119_OFFSET)
#  define S32K3XX_CAN5_ERFFEL120             (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL120_OFFSET)
#  define S32K3XX_CAN5_ERFFEL121             (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL121_OFFSET)
#  define S32K3XX_CAN5_ERFFEL122             (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL122_OFFSET)
#  define S32K3XX_CAN5_ERFFEL123             (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL123_OFFSET)
#  define S32K3XX_CAN5_ERFFEL124             (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL124_OFFSET)
#  define S32K3XX_CAN5_ERFFEL125             (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL125_OFFSET)
#  define S32K3XX_CAN5_ERFFEL126             (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL126_OFFSET)
#  define S32K3XX_CAN5_ERFFEL127             (S32K3XX_CAN5_BASE + S32K3XX_CAN_ERFFEL127_OFFSET)

/* FlexCAN Register Bitfield Definitions ************************************/

/* Module Configuration Register (MCR) */

#define CAN_MCR_MAXMB_SHIFT           (0)       /* Bits 0-6: Number of the Last Message Buffer (MAXMB) */
#define CAN_MCR_MAXMB_MASK            (0x7f << CAN_MCR_MAXMB_SHIFT)
                                                /* Bit 7: Reserved */
#define CAN_MCR_IDAM_SHIFT            (8)       /* Bits 8-9: ID Acceptance Mode (IDAM) */
#define CAN_MCR_IDAM_MASK             (0x03 << CAN_MCR_IDAM_SHIFT)
#  define CAN_MCR_IDAM_FMTA           (0x00 << CAN_MCR_IDAM_SHIFT) /* Format A: One full ID  */
#  define CAN_MCR_IDAM_FMTB           (0x01 << CAN_MCR_IDAM_SHIFT) /* Format B: Two full (or partial) IDs */
#  define CAN_MCR_IDAM_FMTC           (0x02 << CAN_MCR_IDAM_SHIFT) /* Format C: Four partial IDs */
#  define CAN_MCR_IDAM_FMTD           (0x03 << CAN_MCR_IDAM_SHIFT) /* Format D: All frames rejected */

                                                /* Bit 10: Reserved */
#define CAN_MCR_FDEN                  (1 << 11) /* Bit 11: CAN FD operation enable (FDEN) */
#define CAN_MCR_AEN                   (1 << 12) /* Bit 12: Abort Enable (AEN) */
#define CAN_MCR_LPRIOEN               (1 << 13) /* Bit 13: Local Priority Enable (LPRIOEN) */
                                                /* Bit 14: Reserved */
#define CAN_MCR_DMA                   (1 << 15) /* Bit 15: DMA Enable (DMA) */
#define CAN_MCR_IRMQ                  (1 << 16) /* Bit 16: Individual Rx Masking and Queue Enable (IRMQ) */
#define CAN_MCR_SRXDIS                (1 << 17) /* Bit 17: Self Reception Disable (SRXDIS) */
                                                /* Bits 18-19: Reserved */
#define CAN_MCR_LPMACK                (1 << 20) /* Bit 20: Low-Power Mode Acknowledge (LPMACK) */
#define CAN_MCR_WRNEN                 (1 << 21) /* Bit 21: Warning Interrupt Enable (WRNEN) */
                                                /* Bit 22: Reserved */
#define CAN_MCR_SUPV                  (1 << 23) /* Bit 23: Supervisor Mode (SUPV) */
#define CAN_MCR_FRZACK                (1 << 24) /* Bit 24: Freeze Mode Acknowledge (FRZACK) */
#define CAN_MCR_SOFTRST               (1 << 25) /* Bit 25: Soft Reset (SOFTRST) */
                                                /* Bit 26: Reserved */
#define CAN_MCR_NOTRDY                (1 << 27) /* Bit 27: FlexCAN Not Ready (NOTRDY) */
#define CAN_MCR_HALT                  (1 << 28) /* Bit 28: Halt FlexCAN (HALT) */
#define CAN_MCR_RFEN                  (1 << 29) /* Bit 29: Legacy Rx FIFO Enable (RFEN) */
#define CAN_MCR_FRZ                   (1 << 30) /* Bit 30: Freeze Enable (FRZ) */
#define CAN_MCR_MDIS                  (1 << 31) /* Bit 31: Module Disable (MDIS) */

/* Control 1 Register (CTRL1) */

#define CAN_CTRL1_PROPSEG_SHIFT       (0)       /* Bits 0-2: Propagation Segment (PROPSEG) */
#define CAN_CTRL1_PROPSEG_MASK        (0x07 << CAN_CTRL1_PROPSEG_SHIFT)
#define CAN_CTRL1_PROPSEG(x)          (((x) << CAN_CTRL1_PROPSEG_SHIFT) & CAN_CTRL1_PROPSEG_MASK)
#define CAN_CTRL1_LOM                 (1 << 3)  /* Bit 3: Listen-Only Mode (LOM) */
#define CAN_CTRL1_LBUF                (1 << 4)  /* Bit 4: Lowest Buffer Transmitted First (LBUF) */
#define CAN_CTRL1_TSYN                (1 << 5)  /* Bit 5: Timer Sync (TSYN) */
#define CAN_CTRL1_BOFFREC             (1 << 6)  /* Bit 6: Bus Off Recovery (BOFFREC) */
#define CAN_CTRL1_SMP                 (1 << 7)  /* Bit 7: CAN Bit Sampling (SMP) */
                                                /* Bits 8-9: Reserved */
#define CAN_CTRL1_RWRNMSK             (1 << 10) /* Bit 10: Rx Warning Interrupt Mask (RWRNMSK) */
#define CAN_CTRL1_TWRNMSK             (1 << 11) /* Bit 11: Tx Warning Interrupt Mask (TWRNMSK) */
#define CAN_CTRL1_LPB                 (1 << 12) /* Bit 12: Loop Back Mode (LPB) */
                                                /* Bit 13: Reserved */
#define CAN_CTRL1_ERRMSK              (1 << 14) /* Bit 14: Error Interrupt Mask (ERRMSK) */
#define CAN_CTRL1_BOFFMSK             (1 << 15) /* Bit 15: Bus Off Interrupt Mask (BOFFMSK) */
#define CAN_CTRL1_TIMINGMSK           (0xffff << 16)
#define CAN_CTRL1_PSEG2_SHIFT         (16)      /* Bits 16-18: Phase Segment 2 (PSEG2) */
#define CAN_CTRL1_PSEG2_MASK          (0x07 << CAN_CTRL1_PSEG2_SHIFT)
#define CAN_CTRL1_PSEG2(x)            (((x) << CAN_CTRL1_PSEG2_SHIFT) & CAN_CTRL1_PSEG2_MASK)
#define CAN_CTRL1_PSEG1_SHIFT         (19)      /* Bits 19-21: Phase Segment 1 (PSEG1) */
#define CAN_CTRL1_PSEG1_MASK          (0x07 << CAN_CTRL1_PSEG1_SHIFT)
#define CAN_CTRL1_PSEG1(x)            (((x) << CAN_CTRL1_PSEG1_SHIFT) & CAN_CTRL1_PSEG1_MASK)
#define CAN_CTRL1_RJW_SHIFT           (22)      /* Bits 22-23: Resync Jump Width (RJW) */
#define CAN_CTRL1_RJW_MASK            (0x03 << CAN_CTRL1_RJW_SHIFT)
#define CAN_CTRL1_RJW(x)              (((x) << CAN_CTRL1_RJW_SHIFT) & CAN_CTRL1_RJW_MASK)
#define CAN_CTRL1_PRESDIV_SHIFT       (24)      /* Bits 24-31: Prescaler Division Factor (PRESDIV) */
#define CAN_CTRL1_PRESDIV_MASK        (0xff << CAN_CTRL1_PRESDIV_SHIFT)
#define CAN_CTRL1_PRESDIV(x)          (((x) << CAN_CTRL1_PRESDIV_SHIFT) & CAN_CTRL1_PRESDIV_MASK)

/* Free Running Timer (TIMER) */

#define CAN_TIMER_SHIFT               (0)       /* Bits 0-15: Timer value (TIMER) */
#define CAN_TIMER_MASK                (0xffff << CAN_TIMER_SHIFT)
                                                    /* Bits 16-31: Reserved */

/* Rx Mailboxes Global Mask Register (RXMGMASK) */

#define CAN_RXMGMASK(n)               (1 << (n)) /* Bit n: Rx Mailbox n Global Mask Bit (MGn) */

/* Rx 14 Mask Register (RX14MASK) */

#define CAN_RX14MASK(n)               (1 << (n)) /* Bit n: Rx Buffer 14 Mask Bit n (RX14M) */

/* Rx 15 Mask Register (RX15MASK) */

#define CAN_RX15MASK(n)               (1 << (n)) /* Bit n: Rx Buffer 15 Mask Bit n (RX15M) */

/* Error Counter (ECR) */

#define CAN_ECR_TXERRCNT_SHIFT        (0)       /* Bits 0-7: Transmit Error Counter (TXERRCNT) */
#define CAN_ECR_TXERRCNT_MASK         (0xff << CAN_ECR_TXERRCNT_SHIFT)
#define CAN_ECR_RXERRCNT_SHIFT        (8)       /* Bits 8-15: Receive Error Counter (RXERRCNT) */
#define CAN_ECR_RXERRCNT_MASK         (0xff << CAN_ECR_RXERRCNT_SHIFT)
#define CAN_ECR_TXERRCNT_FAST_SHIFT   (16)       /* Bits 16-23: Transmit Error Counter for fast bits (TXERRCNT_FAST) */
#define CAN_ECR_TXERRCNT_FAST_MASK    (0xff << CAN_ECR_TXERRCNT_FAST_SHIFT)
#define CAN_ECR_RXERRCNT_FAST_SHIFT   (24)       /* Bits 24-31: Receive Error Counter for fast bits (RXERRCNT_FAST) */
#define CAN_ECR_RXERRCNT_FAST_MASK    (0xff << CAN_ECR_RXERRCNT_FAST_SHIFT)

/* Error and Status 1 Register (ESR1) */

                                                    /* Bit 0: Reserved */
#define CAN_ESR1_ERRINT               (1 << 1)  /* Bit 1: Error Interrupt (ERRINT) */
#define CAN_ESR1_BOFFINT              (1 << 2)  /* Bit 2: Bus Off Interrupt (BOFFINT) */
#define CAN_ESR1_RX                   (1 << 3)  /* Bit 3: FlexCAN in Reception (RX) */
#define CAN_ESR1_FLTCONF_SHIFT        (4)       /* Bits 4-5: Fault Confinement State (FLTCONF) */
#define CAN_ESR1_FLTCONF_MASK         (0x03 << CAN_ESR1_FLTCONF_SHIFT)
#  define CAN_ESR1_FLTCONF_ACTV       (0x00 << CAN_ESR1_FLTCONF_SHIFT) /* Error Active */
#  define CAN_ESR1_FLTCONF_PASV       (0x01 << CAN_ESR1_FLTCONF_SHIFT) /* Error Passive */
#  define CAN_ESR1_FLTCONF_OFF        (0x02 << CAN_ESR1_FLTCONF_SHIFT) /* Bus Off */

#define CAN_ESR1_TX                   (1 << 6)  /* Bit 6: FlexCAN in Transmission (TX) */
#define CAN_ESR1_IDLE                 (1 << 7)  /* Bit 7: CAN bus is in IDLE state (IDLE) */
#define CAN_ESR1_RXWRN                (1 << 8)  /* Bit 8: Rx Error Warning (RXWRN) */
#define CAN_ESR1_TXWRN                (1 << 9)  /* Bit 9: TX Error Warning (TXWRN) */
#define CAN_ESR1_STFERR               (1 << 10) /* Bit 10: Stuffing Error (STFERR) */
#define CAN_ESR1_FRMERR               (1 << 11) /* Bit 11: Form Error (FRMERR) */
#define CAN_ESR1_CRCERR               (1 << 12) /* Bit 12: Cyclic Redundancy Check Error (CRCERR) */
#define CAN_ESR1_ACKERR               (1 << 13) /* Bit 13: Acknowledge Error (ACKERR) */
#define CAN_ESR1_BIT0ERR              (1 << 14) /* Bit 14: Bit0 Error (BIT0ERR) */
#define CAN_ESR1_BIT1ERR              (1 << 15) /* Bit 15: Bit1 Error (BIT1ERR) */
#define CAN_ESR1_RWRNINT              (1 << 16) /* Bit 16: Rx Warning Interrupt Flag (RWRNINT) */
#define CAN_ESR1_TWRNINT              (1 << 17) /* Bit 17: Tx Warning Interrupt Flag (TWRNINT) */
#define CAN_ESR1_SYNCH                (1 << 18) /* Bit 18: CAN Synchronization Status (SYNCH) */
#define CAN_ESR1_BOFFDONEINT          (1 << 19) /* Bit 19: Bus Off Done Interrupt (BOFFDONEINT) */
#define CAN_ESR1_ERRINT_FAST          (1 << 20) /* Bit 20: Error interrupt for errors detected in Data Phase of CAN FD frames with BRS bit set (ERRINT_FAST) */
#define CAN_ESR1_ERROVR               (1 << 21) /* Bit 21: Error Overrun (ERROVR) */
                                                /* Bits 22-25: Reserved */
#define CAN_ESR1_STFERR_FAST          (1 << 26) /* Bit 26: Stuffing Error in the Data Phase of CAN FD frames with the BRS bit set (STFERR_FAST) */
#define CAN_ESR1_FRMERR_FAST          (1 << 27) /* Bit 27: Form Error in the Data Phase of CAN FD frames with the BRS bit set (FRMERR_FAST) */
#define CAN_ESR1_CRCERR_FAST          (1 << 28) /* Bit 28: Cyclic Redundancy Check Error in the CRC field of CAN FD frames with the BRS bit set (CRCERR_FAST) */
                                                /* Bit 29: Reserved */
#define CAN_ESR1_BIT0ERR_FAST         (1 << 30) /* Bit 30: Bit0 Error in the Data Phase of CAN FD frames with the BRS bit set (BIT0ERR_FAST) */
#define CAN_ESR1_BIT1ERR_FAST         (1 << 31) /* Bit 31: Bit1 Error in the Data Phase of CAN FD frames with the BRS bit set (BIT1ERR_FAST) */

/* Interrupt Masks 2 Register (IMASK2) */

#define CAN_IMASK2(n)                 (1 << (n)) /* Bit n: Buffer MBn Mask (BUF63TO32M) */

/* Interrupt Masks 1 Register (IMASK1) */

#define CAN_IMASK1(n)                 (1 << (n)) /* Bit n: Buffer MBn Mask (BUF31TO0M) */

/* Interrupt Flags 2 Register (IFLAG2) */

#define CAN_IFLAG2(n)                 (1 << (n)) /* Bit n: Buffer MBn Interrupt (BUF63TO32I) */

/* Interrupt Flags 1 Register (IFLAG1) */

#define CAN_IFLAG1(n)                 (1 << (n)) /* Bit n: Buffer MBn Interrupt (BUF31TO0I) */

/* Control 2 Register (CTRL2) */

                                                /* Bits 0-5: Reserved */
#define CAN_CTRL2_TSTAMPCAP_SHIFT     (6)       /* Bits 6-7: Time Stamp Capture Point (TSTAMPCAP) */
#define CAN_CTRL2_TSTAMPCAP_MASK      (0x03 << CAN_CTRL2_TSTAMPCAP_SHIFT)
#  define CAN_CTRL2_TSTAMPCAP_DIS     (0x00 << CAN_CTRL2_TSTAMPCAP_SHIFT) /* Disabled */
#  define CAN_CTRL2_TSTAMPCAP_END     (0x01 << CAN_CTRL2_TSTAMPCAP_SHIFT) /* Captured in the end of the CAN frame */
#  define CAN_CTRL2_TSTAMPCAP_START   (0x02 << CAN_CTRL2_TSTAMPCAP_SHIFT) /* Captured in the start of the CAN frame */
#  define CAN_CTRL2_TSTAMPCAP_RES     (0x03 << CAN_CTRL2_TSTAMPCAP_SHIFT) /* Captured in the start of frame for classical CAN frames and in res bit for CAN FD frames */

#define CAN_CTRL2_MBTSBASE_SHIFT      (8)       /* Bits 8-9: Message Buffer Time Stamp Base (MBTSBASE) */
#define CAN_CTRL2_MBTSBASE_MASK       (0x03 << CAN_CTRL2_MBTSBASE_SHIFT)
#  define CAN_CTRL2_MBTSBASE_TIMER    (0x00 << CAN_CTRL2_MBTSBASE_SHIFT) /* Message buffer time stamp base is TIMER */
#  define CAN_CTRL2_MBTSBASE_LOWER    (0x01 << CAN_CTRL2_MBTSBASE_SHIFT) /* Message buffer time stamp base is lower 16 bits of higher resolution timer */
#  define CAN_CTRL2_MBTSBASE_UPPER    (0x02 << CAN_CTRL2_MBTSBASE_SHIFT) /* Message buffer time stamp base is upper 16 bits of higher resolution timer */

                                                /* Bit 10: Reserved */
#define CAN_CTRL2_EDFLTDIS            (1 << 11) /* Bit 11: Edge Filter Disable (EDFLTDIS) */
#define CAN_CTRL2_ISOCANFDEN          (1 << 12) /* Bit 12: ISO CAN FD Enable (ISOCANFDEN) */
#define CAN_CTRL2_BTE                 (1 << 13) /* Bit 13: Bit Timing Expansion enable (BTE) */
#define CAN_CTRL2_PREXCEN             (1 << 14) /* Bit 14: Protocol Exception Enable (PREXCEN) */
#define CAN_CTRL2_TIMER_SRC           (1 << 15) /* Bit 15: Timer Source (TIMER_SRC) */
#define CAN_CTRL2_EACEN               (1 << 16) /* Bit 16: Entire Frame Arbitration Field Comparison Enable For Rx Mailboxes (EACEN) */
#define CAN_CTRL2_RRS                 (1 << 17) /* Bit 17: Remote Request Storing (RRS) */
#define CAN_CTRL2_MRP                 (1 << 18) /* Bit 18: Mailboxes Reception Priority (MRP) */
#define CAN_CTRL2_TASD_SHIFT          (19)      /* Bits 19-23: Tx Arbitration Start Delay (TASD) */
#define CAN_CTRL2_TASD_MASK           (0x1f << CAN_CTRL2_TASD_SHIFT)
#define CAN_CTRL2_RFFN_SHIFT          (24)      /* Bits 24-27: Number of Legacy Rx FIFO Filters (RFFN) */
#define CAN_CTRL2_RFFN_MASK           (0x0f << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_8MB          (0x00 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_16MB         (0x01 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_24MB         (0x02 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_32MB         (0x03 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_40MB         (0x04 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_48MB         (0x05 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_56MB         (0x06 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_64MB         (0x07 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_72MB         (0x08 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_80MB         (0x09 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_88MB         (0x0a << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_96MB         (0x0b << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_104MB        (0x0c << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_112MB        (0x0d << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_120MB        (0x0e << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_128MB        (0x0f << CAN_CTRL2_RFFN_SHIFT)
#define CAN_CTRL2_WRMFRZ              (1 << 28) /* Bit 28: Write-Access to Memory in Freeze mode (WRMFRZ) */
#define CAN_CTRL2_ECRWRE              (1 << 29) /* Bit 29: Error-correction Configuration Register Write Enable (ECRWRE) */
#define CAN_CTRL2_BOFFDONEMSK         (1 << 30) /* Bit 30: Bus Off Done Interrupt Mask (BOFFDONEMSK) */
#define CAN_CTRL2_ERRMSK_FAST         (1 << 31) /* Bit 31: Error Interrupt Mask for errors detected in the data phase of fast CAN FD frames (ERRMSK_FAST) */

/* Error and Status 2 Register (ESR2) */

                                                /* Bits 0-12: Reserved */
#define CAN_ESR2_IMB                  (1 << 13) /* Bit 13: Inactive Mailbox (IMB) */
#define CAN_ESR2_VPS                  (1 << 14) /* Bit 14: Valid Priority Status (VPS) */
                                                /* Bit 15: Reserved */
#define CAN_ESR2_LPTM_SHIFT           (16)      /* Bits 16-22: Lowest Priority Tx Mailbox (LPTM) */
#define CAN_ESR2_LPTM_MASK            (0x7f << CAN_ESR2_LPTM_SHIFT)
                                                /* Bits 23-31: Reserved */

/* CRC Register (CRCR) */

#define CAN_CRCR_TXCRC_SHIFT          (0)       /* Bits 0-14: Transmitted CRC value (TXCRC) */
#define CAN_CRCR_TXCRC_MASK           (0x7fff << CAN_CRCR_TXCRC_SHIFT)
                                                /* Bit 15: Reserved */
#define CAN_CRCR_MBCRC_SHIFT          (16)      /* Bits 16-22: CRC Mailbox (MBCRC) */
#define CAN_CRCR_MBCRC_MASK           (0x7f << CAN_CRCR_MBCRC_SHIFT)
                                                /* Bits 23-31: Reserved */

/* Legacy Rx FIFO Global Mask Register (RXFGMASK) */

#define CAN_RXFGMASK_FGM_SHIFT        (0)       /* Bits 0-31: Legacy Rx FIFO GLobal Mask Bits (FGM) */
#define CAN_RXFGMASK_FGM_MASK         (0xffffffff << CAN_RXFGMASK_FGM_SHIFT)

/* Legacy Rx FIFO Information Register (RXFIR) */

#define CAN_RXFIR_IDHIT_SHIFT         (0)       /* Bits 0-8: Identifier Acceptance Filter Hit Indicator (IDHIT) */
#define CAN_RXFIR_IDHIT_MASK          (0x1ff << CAN_RXFIR_IDHIT_SHIFT)
                                                /* Bits 9-31: Reserved */

/* CAN Bit Timing register (CBT) */

#define CAN_CBT_EPSEG2_SHIFT          (0)       /* Bits 0-4: Extended Phase Segment 2 (EPSEG2) */
#define CAN_CBT_EPSEG2_MASK           (0x1f << CAN_CBT_EPSEG2_SHIFT)
#define CAN_CBT_EPSEG2(x)             (((x) << CAN_CBT_EPSEG2_SHIFT) & CAN_CBT_EPSEG2_MASK)
#define CAN_CBT_EPSEG1_SHIFT          (5)       /* Bits 5-9: Extended Phase Segment 1 (EPSEG1) */
#define CAN_CBT_EPSEG1_MASK           (0x1f << CAN_CBT_EPSEG1_SHIFT)
#define CAN_CBT_EPSEG1(x)             (((x) << CAN_CBT_EPSEG1_SHIFT) & CAN_CBT_EPSEG1_MASK)
#define CAN_CBT_EPROPSEG_SHIFT        (10)      /* Bits 10-15: Extended Propagation Segment (EPROPSEG) */
#define CAN_CBT_EPROPSEG_MASK         (0x3f << CAN_CBT_EPROPSEG_SHIFT)
#define CAN_CBT_EPROPSEG(x)           (((x) << CAN_CBT_EPROPSEG_SHIFT) & CAN_CBT_EPROPSEG_MASK)
#define CAN_CBT_ERJW_SHIFT            (16)      /* Bits 16-20: Extended Resync Jump Width (ERJW) */
#define CAN_CBT_ERJW_MASK             (0x1f << CAN_CBT_ERJW_SHIFT)
#define CAN_CBT_ERJW(x)               (((x) << CAN_CBT_ERJW_SHIFT) & CAN_CBT_ERJW_MASK)
#define CAN_CBT_EPRESDIV_SHIFT        (21)      /* Bits 21-30: Extended Prescaler Division Factor (EPRESDIV) */
#define CAN_CBT_EPRESDIV_MASK         (0x03ff << CAN_CBT_EPRESDIV_SHIFT)
#define CAN_CBT_EPRESDIV(x)           (((x) << CAN_CBT_EPRESDIV_SHIFT) & CAN_CBT_EPRESDIV_MASK)
#define CAN_CBT_BTF                   (1 << 31) /* Bit 31: Bit Timing Format Enable (BTF) */

/* Interrupt Masks 3 Register (IMASK3) */

#define CAN_IMASK3(n)                 (1 << (n)) /* Bit n: Buffer MBn Mask (BUF95TO64M) */

/* Interrupt Flags 3 Register (IFLAG3) */

#define CAN_IFLAG3(n)                 (1 << (n)) /* Bit n: Buffer MBn Interrupt (BUF95TO64I) */

/* Rx Individual Mask Registers (RXIMRn) */

#define CAN_RXIMR(n)                  (1 << (n)) /* Bit n: Individual Mask Bits (MI) */

/* Memory Error Control Register (MECR) */

                                                /* Bit 0-6: Reserved */
#define CAN_MECR_NCEFAFRZ             (1 << 7)  /* Bit 7: Non-Correctable Errors in FlexCAN Access Put Device in Freeze Mode (NCEFAFRZ) */
#define CAN_MECR_ECCDIS               (1 << 8)  /* Bit 8: Error Correction Disable (ECCDIS) */
#define CAN_MECR_RERRDIS              (1 << 9)  /* Bit 9: Error Report Disable (RERRDIS) */
                                                /* Bits 10-12: Reserved */
#define CAN_MECR_EXTERRIE             (1 << 13) /* Bit 13: Extended Error Injection Enable (EXTERRIE) */
#define CAN_MECR_FAERRIE              (1 << 14) /* Bit 14: FlexCAN Access Error Injection Enable (FAERRIE) */
#define CAN_MECR_HAERRIE              (1 << 15) /* Bit 15: Host Access Error Injection Enable (HAERRIE) */
#define CAN_MECR_CEI_MSK              (1 << 16) /* Bit 16: Correctable Errors Interrupt Mask (CEI_MSK) */
                                                /* Bit 17: Reserved */
#define CAN_MECR_FANCEI_MSK           (1 << 18) /* Bit 18: FlexCAN Access With Non-Correctable Errors Interrupt Mask (FANCEI_MSK) */
#define CAN_MECR_HANCEI_MSK           (1 << 19) /* Bit 19: Host Access With Non-Correctable Errors Interrupt Mask (HANCEI_MSK) */
                                                /* Bit 20-30: Reserved */
#define CAN_MECR_ECRWRDIS             (1 << 31) /* Bit 31: Error Configuration Register Write Disable (ECRWRDIS) */

/* Error Injection Address Register (ERRIAR) */

#define CAN_ERRIAR_INJADDR_L_SHIFT    (0)       /* Bits 0-1: Error Injection Address Low (INJADDR_L) */
#define CAN_ERRIAR_INJADDR_L_MASK     (0x03 << CAN_ERRIAR_INJADDR_L_SHIFT)
#define CAN_ERRIAR_INJADDR_H_SHIFT    (2)       /* Bits 2-13: Error Injection Address High (INJADDR_H) */
                                                /* Bits 14-31: Reserved */

/* Error Injection Data Pattern Register (ERRIDPR) */

#define CAN_ERRIDPR_DFLIP_SHIFT       (0)       /* Bits 0-31: Data flip pattern (DFLIP) */
#define CAN_ERRIDPR_DFLIP_MASK        (0xffffffff << CAN_ERRIDPR_DFLIP_SHIFT)

/* Error Injection Parity Pattern Register (ERRIPPR) */

#define CAN_ERRIPPR_PFLIP0_SHIFT      (0)       /* Bits 0-4: Parity Flip Pattern For Byte 0 (Least Significant) (PFLIP0) */
#define CAN_ERRIPPR_PFLIP0_MASK       (0x1f << CAN_ERRIPPR_PFLIP0_SHIFT)
                                                /* Bits 5-7: Reserved */
#define CAN_ERRIPPR_PFLIP1_SHIFT      (8)       /* Bits 8-12: Parity Flip Pattern For Byte 1 (PFLIP1) */
#define CAN_ERRIPPR_PFLIP1_MASK       (0x1f << CAN_ERRIPPR_PFLIP1_SHIFT)
                                                /* Bits 13-15: Reserved */
#define CAN_ERRIPPR_PFLIP2_SHIFT      (16)      /* Bits 16-20: Parity Flip Pattern For Byte 2 (PFLIP2) */
#define CAN_ERRIPPR_PFLIP2_MASK       (0x1f << CAN_ERRIPPR_PFLIP2_SHIFT)
                                                /* Bits 21-23: Reserved */
#define CAN_ERRIPPR_PFLIP3_SHIFT      (24)      /* Bits 24-28: Parity Flip Pattern For Byte 3 (Most Significant) (PFLIP3) */
#define CAN_ERRIPPR_PFLIP3_MASK       (0x1f << CAN_ERRIPPR_PFLIP3_SHIFT)
                                                /* Bits 29-31: Reserved */

/* Error Report Address Register (RERRAR) */

#define CAN_RERRAR_ERRADDR_SHIFT      (0)       /* Bits 0-13: Address Where Error Detected (ERADDR) */
#define CAN_RERRAR_ERRADDR_MASK       (0x3fff << CAN_RERRAR_ERRADDR_SHIFT)
                                                /* Bits 14-15: Reserved */
#define CAN_RERRAR_SAID_SHIFT         (16)      /* Bits 16-18: SAID */
#define CAN_RERRAR_SAID_MASK          (0x07 << CAN_RERRAR_SAID_SHIFT)
                                                /* Bits 19-23: Reserved */
#define CAN_RERRAR_NCE                (1 << 24) /* Bit 24: Non-Correctable Error (NCE) */
                                                /* Bits 25-31: Reserved */

/* Error Report Data Register (RERRDR) */

#define CAN_RERRDR_RDATA_SHIFT        (0)       /* Bits 0-31: Raw data word read from memory with error (RDATA) */
#define CAN_RERRDR_RDATA_MASK         (0xffffffff << CAN_RERRDR_RDATA_SHIFT)

/* Error Report Syndrome Register (RERRSYNR) */

#define CAN_RERRSYNR_SYND0_SHIFT      (0)       /* Bits 0-4: Error Syndrome For Byte 0 (Least Significant) (SYND0) */
#define CAN_RERRSYNR_SYND0_MASK       (0x1f << CAN_RERRSYNR_SYND0_SHIFT)
                                                /* Bits 5-6: Reserved */
#define CAN_RERRSYNR_BE0              (1 << 7)  /* Bit 7: Byte Enabled For Byte 0 (Least Significant) (BE0) */
#define CAN_RERRSYNR_SYND1_SHIFT      (8)       /* Bits 8-12: Error Syndrome For Byte 1 (SYND1) */
#define CAN_RERRSYNR_SYND1_MASK       (0x1f << CAN_RERRSYNR_SYND1_SHIFT)
                                                /* Bits 13-14: Reserved */
#define CAN_RERRSYNR_BE1              (1 << 15) /* Bit 15: Byte Enabled For Byte 1 (BE1) */
#define CAN_RERRSYNR_SYND2_SHIFT      (16)      /* Bits 16-20: Error Syndrome For Byte 2 (SYND2) */
#define CAN_RERRSYNR_SYND2_MASK       (0x1f << CAN_RERRSYNR_SYND2_SHIFT)
                                                /* Bits 21-22: Reserved */
#define CAN_RERRSYNR_BE2              (1 << 23) /* Bit 23: Byte Enabled For Byte 2 (BE2) */
#define CAN_RERRSYNR_SYND3_SHIFT      (24)      /* Bits 24-28: Error Syndrome For Byte 3 (Most Significant) (SYND3) */
#define CAN_RERRSYNR_SYND3_MASK       (0x1f << CAN_RERRSYNR_SYND3_SHIFT)
                                                /* Bits 29-30: Reserved */
#define CAN_RERRSYNR_BE3              (1 << 31) /* Bit 31: Byte Enabled For Byte 3 (Most Significant) (BE3) */

/* Error Status Register (ERRSR) */

#define CAN_ERRSR_CEIOF               (1 << 0)  /* Bit 0: Correctable Error Interrupt Overrun Flag (CEIOF) */
                                                /* Bit 1: Reserved */
#define CAN_ESSR_FANCEIOF             (1 << 2)  /* Bit 2: FlexCAN Access With Non-Correctable Error Interrupt Overrun Flag (FANCEIOF) */
#define CAN_ESSR_HANCEIOF             (1 << 3)  /* Bit 3: Host Access With Non-Correctable Error Interrupt Overrun Flag (HANCEIOF) */
                                                /* Bits 4-15: Reserved */
#define CAN_ESSR_CEIF                 (1 << 16) /* Bit 16: Correctable Error Interrupt Flag (CEIF) */
                                                /* Bit 17: Reserved */
#define CAN_ESSR_FANCEIF              (1 << 18) /* Bit 18: FlexCAN Access With Non-Correctable Error Interrupt Flag (FANCEIF) */
#define CAN_ESSR_HANCEIF              (1 << 19) /* Bit 19: Host Access With Non-Correctable Error Interrupt Flag (HANCEIF) */
                                                /* Bits 20-31: Reserved */

/* Enhanced CAN Bit Timing Prescalers (EPRS) */

#define CAN_EPRS_ENPRESDIV_SHIFT      (0)       /* Bits 0-9: Extended Nominal Prescaler Division Factor (ENPRESDIV) */
#define CAN_EPRS_ENPRESDIV_MASK       (0x03ff << CAN_EPRS_ENPRESDIV_SHIFT)
                                                /* Bits 10-15: Reserved */
#define CAN_EPRS_EDPRESDIV_SHIFT      (16)      /* Bits 16-25: Extended Data Phase Prescaler Division Factor (EDPRESDIV) */
                                                /* Bits 26-31: Reserved */

/* Enhanced Nominal CAN Bit Timing (ENCBT) */

#define CAN_ENCBT_NTSEG1_SHIFT        (0)       /* Bits 0-7: Nominal Time Segment 1 (NTSEG1) */
#define CAN_ENCBT_NTSEG1_MASK         (0xff << CAN_ENCBT_NTSEG1_SHIFT)
                                                /* Bits 8-11: Reserved */
#define CAN_ENCBT_NTSEG2_SHIFT        (12)      /* Bits 12-18: Nominal Time Segment 2 (NTSEG2) */
#define CAN_ENCBT_NTSEG2_MASK         (0x7f << CAN_ENCBT_NTSEG2_SHIFT)
                                                /* Bits 19-21: Reserved */
#define CAN_ENCBT_NRJW_SHIFT          (22)      /* Bits 22-28: Nominal Resynchronization Jump Width (NRJW) */
#define CAN_ENCBT_NRJW_MASK           (0x7f << CAN_ENCBT_NRJW_SHIFT)
                                                /* Bits 29-31: Reserved */

/* Enhanced Data Phase CAN Bit Timing (EDCBT) */

#define CAN_EDCBT_DTSEG1_SHIFT        (0)       /* Bits 0-4: Data Phase Time Segment 1 (DTSEG1) */
#define CAN_EDCBT_DTSEG1_MASK         (0x1f << CAN_EDCBT_DTSEG1_SHIFT)
                                                /* Bits 5-11: Reserved */
#define CAN_EDCBT_DTSEG2_SHIFT        (12)      /* Bits 12-15: Data Phase Time Segment 2 (DTSEG2) */
#define CAN_EDCBT_DTSEG2_MASK         (0x0f << CAN_EDCBT_DTSEG2_SHIFT)
                                                /* Bits 16-21: Reserved */
#define CAN_EDCBT_DRJW_SHIFT          (22)      /* Bits 22-25: Data Phase Resynchronization Jump Width (DRJW) */
#define CAN_EDCBT_DRJW_MASK           (0x0f << CAN_EDCBT_DRJW_SHIFT)
                                                /* Bits 26-31: Reserved */

/* Enhanced Transceiver Delay Compensation (ETDC) */

#define CAN_ETDC_ETDCVAL_SHIFT        (0)       /* Bits 0-7: Enhanced Transceiver Delay Compensation Value (ETDCVAL) */
#define CAN_ETDC_ETDCVAL_MASK         (0xff << CAN_ETDC_ETDCVAL_SHIFT)
                                                /* Bits 8-14: Reserved */
#define CAN_ETDC_ETDCFAIL             (1 << 15) /* Bit 15: Transceiver Delay Compensation Fail (ETDCFAIL) */
#define CAN_ETDC_ETDCOFF_SHIFT        (16)      /* Bits 16-22: Enhanced Transceiver Delay Compensation Offset (ETDCOFF) */
#define CAN_ETDC_ETDCOFF_MASK         (0x7f << CAN_ETDC_ETDCOFF_SHIFT)
                                                /* Bits 23-29: Reserved */
#define CAN_ETDC_TDMDIS               (1 << 30) /* Bit 30: Transceiver Delay Measurement Disable (TDMDIS) */
#define CAN_ETDC_ETDCEN               (1 << 31) /* Bit 31: Transceiver Delay Compensation Enable (ETDCEN) */

/* CAN FD Control register (FDCTRL) */

#define CAN_FDCTRL_TDCVAL_SHIFT       (0)       /* Bits 0-5: Transceiver Delay Compensation Value (TDCVAL) */
#define CAN_FDCTRL_TDCVAL_MASK        (0x3f << CAN_FDCTRL_TDCVAL_SHIFT)
#define CAN_FDCTRL_TDCVAL(x)          (((x) << CAN_FDCTRL_TDCVAL_SHIFT) & CAN_FDCTRL_TDCVAL_MASK)
                                                /* Bits 6-7: Reserved */
#define CAN_FDCTRL_TDCOFF_SHIFT       (8)       /* Bits 8-12: Transceiver Delay Compensation Offset (TDCOFF) */
#define CAN_FDCTRL_TDCOFF_MASK        (0x1f << CAN_FDCTRL_TDCOFF_SHIFT)
#define CAN_FDCTRL_TDCOFF(x)          (((x) << CAN_FDCTRL_TDCOFF_SHIFT) & CAN_FDCTRL_TDCOFF_MASK)
                                                /* Bit 13: Reserved */
#define CAN_FDCTRL_TDCFAIL            (1 << 14) /* Bit 14: Transceiver Delay Compensation Fail (TDCFAIL) */
#define CAN_FDCTRL_TDCEN              (1 << 15) /* Bit 15: Transceiver Delay Compensation Enable (TDCEN) */
#define CAN_FDCTRL_MBDSR0_SHIFT       (16)      /* Bits 16-17: Message Buffer Data Size for Region 0 (MBDSR0) */
#define CAN_FDCTRL_MBDSR0_MASK        (0x03 << CAN_FDCTRL_MBDSR0_SHIFT)
#define CAN_FDCTRL_MBDSR0(x)          (((x) << CAN_FDCTRL_MBDSR0_SHIFT) & CAN_FDCTRL_MBDSR0_MASK)
                                                /* Bit 18: Reserved */
#define CAN_FDCTRL_MBDSR1_SHIFT       (19)      /* Bits 19-20: Message Buffer Data Size for Region 1 (MBDSR1) */
#define CAN_FDCTRL_MBDSR1_MASK        (0x03 << CAN_FDCTRL_MBDSR1_SHIFT)
#define CAN_FDCTRL_MBDSR1(x)          (((x) << CAN_FDCTRL_MBDSR1_SHIFT) & CAN_FDCTRL_MBDSR1_MASK)
                                                /* Bit 21: Reserved */
#define CAN_FDCTRL_MBDSR2_SHIFT       (22)      /* Bits 22-23: Message Buffer Data Size for Region 2 (MBDSR2) */
#define CAN_FDCTRL_MBDSR2_MASK        (0x03 << CAN_FDCTRL_MBDSR2_SHIFT)
#define CAN_FDCTRL_MBDSR2(x)          (((x) << CAN_FDCTRL_MBDSR2_SHIFT) & CAN_FDCTRL_MBDSR2_MASK)
                                                /* Bits 24-30: Reserved */
#define CAN_FDCTRL_FDRATE             (1 << 31) /* Bit 31: Bit Rate Switch Enable (FDRATE) */

/* CAN FD Bit Timing Register (FDCBT) */

#define CAN_FDCBT_FPSEG2_SHIFT        (0)       /* Bits 0-2: Fast Phase Segment 2 (FPSEG2) */
#define CAN_FDCBT_FPSEG2_MASK         (0x07 << CAN_FDCBT_FPSEG2_SHIFT)
#define CAN_FDCBT_FPSEG2(x)           (((x) << CAN_FDCBT_FPSEG2_SHIFT) & CAN_FDCBT_FPSEG2_MASK)
                                                /* Bits 3-4: Reserved */
#define CAN_FDCBT_FPSEG1_SHIFT        (5)       /* Bits 5-7: Fast Phase Segment 1 (FPSEG1) */
#define CAN_FDCBT_FPSEG1_MASK         (0x07 << CAN_FDCBT_FPSEG1_SHIFT)
#define CAN_FDCBT_FPSEG1(x)           (((x) << CAN_FDCBT_FPSEG1_SHIFT) & CAN_FDCBT_FPSEG1_MASK)
                                                /* Bits 8-9: Reserved */
#define CAN_FDCBT_FPROPSEG_SHIFT      (10)      /* Bits 10-14: Fast Propagation Segment (FPROPSEG) */
#define CAN_FDCBT_FPROPSEG_MASK       (0x1f << CAN_FDCBT_FPROPSEG_SHIFT)
#define CAN_FDCBT_FPROPSEG(x)         (((x) << CAN_FDCBT_FPROPSEG_SHIFT) & CAN_FDCBT_FPROPSEG_MASK)
                                                /* Bit 15: Reserved */
#define CAN_FDCBT_FRJW_SHIFT          (16)      /* Bits 16-18: Fast Resync Jump Width (FRJW) */
#define CAN_FDCBT_FRJW_MASK           (0x07 << CAN_FDCBT_FRJW_SHIFT)
#define CAN_FDCBT_FRJW(x)             (((x) << CAN_FDCBT_FRJW_SHIFT) & CAN_FDCBT_FRJW_MASK)
                                                /* Bit 19: Reserved */
#define CAN_FDCBT_FPRESDIV_SHIFT      (20)      /* Bits 20-29: Fast Prescaler Division Factor (FPRESDIV) */
#define CAN_FDCBT_FPRESDIV_MASK       (0x03ff << CAN_FDCBT_FPRESDIV_SHIFT)
#define CAN_FDCBT_FPRESDIV(x)         (((x) << CAN_FDCBT_FPRESDIV_SHIFT) & CAN_FDCBT_FPRESDIV_MASK)
                                                /* Bits 30-31: Reserved */

/* CAN FD CRC Register (FDCRC) */

#define CAN_FDCRC_FD_TXCRC_SHIFT      (0)       /* Bits 0-20: Extended Transmitted CRC value (FD_TXCRC) */
#define CAN_FDCRC_FD_TXCRC_MASK       (0x1fffff << CAN_FDCRC_FD_TXCRC_SHIFT)
#define CAN_FDCRC_FD_TXCRC(x)         (((x) << CAN_FDCRC_FD_TXCRC_SHIFT) & CAN_FDCRC_FD_TXCRC_MASK)
                                                /* Bits 21-23: Reserved */
#define CAN_FDCRC_FD_MBCRC_SHIFT      (24)      /* Bits 24-30: CRC Mailbox Number for FD_TXCRC (FD_MBCRC) */
#define CAN_FDCRC_FD_MBCRC_MASK       (0x7f << CAN_FDCRC_FD_MBCRC_SHIFT)
#define CAN_FDCRC_FD_MBCRC(x)         (((x) << CAN_FDCRC_FD_MBCRC_SHIFT) & CAN_FDCRC_FD_MBCRC_MASK)
                                                /* Bit 31: Reserved */

/* Enhanced Rx FIFO Control Register (ERFCR) */

#define CAN_ERFCR_ERFWM_SHIFT         (0)       /* Bits 0-4: Enhanced Rx FIFO Watermark (ERFWM) */
#define CAN_ERFCR_ERFWM_MASK          (0x1f << CAN_ERFCR_ERFWM_SHIFT)
                                                /* Bits 5-7: Reserved */
#define CAN_ERFCR_NFE_SHIFT           (8)       /* Bits 8-13: Number of Enhanced Rx FIFO Filter Elements (NFE) */
#define CAN_ERFCR_NFE_MASK            (0x3f << CAN_ERFCR_NFE_SHIFT)
                                                /* Bits 14-15: Reserved */
#define CAN_ERFCR_NEXIF_SHIFT         (16)      /* Bits 16-22: Number of Extended ID Filter Elements (NEXIF) */
#define CAN_ERFCR_NEXIF_MASK          (0x7f << CAN_ERFCR_NEXIF_SHIFT)
                                                /* Bits 23-25: Reserved */
#define CAN_ERFCR_DMALW_SHIFT         (26)      /* Bits 26-30: DMA Last Word (DMALW) */
#define CAN_ERFCR_DMALW_MASK          (0x1f << CAN_ERFCR_DMALW_SHIFT)
                                                /* Bit 31: Reserved */

/* Enhanced Rx FIFO Interrupt Enable Register (ERFIER) */

                                                    /* Bits 0-27: Reserved */
#define CAN_ERFIER_ERFDAIE            (1 << 28) /* Bit 28: Enhanced Rx FIFO Data Available Interrupt Enable (ERFDAIE) */
#define CAN_ERFIER_ERFWMIIE           (1 << 29) /* Bit 29: Enhanced Rx FIFO Watermark Indication Interrupt Enable (ERFWMIIE) */
#define CAN_ERFIER_ERFOVFIE           (1 << 30) /* Bit 30: Enhanced Rx FIFO Overflow Interrupt Enable (ERFOVFIE) */
#define CAN_ERFIER_ERFUFWIE           (1 << 31) /* Bit 31: Enhanced Rx FIFO Underflow Interrupt Enable (ERFUFWIE) */

/* Enhanced Rx FIFO Status Register (ERFSR) */

#define CAN_ERFSR_ERFEL_SHIFT         (0)       /* Bits 0-5: Enhanced Rx FIFO Elements (ERFEL) */
#define CAN_ERFSR_ERFEL_MASK          (0x3f << CAN_ERFSR_ERFEL_SHIFT)
                                                /* Bits 6-15: Reserved */
#define CAN_ERFSR_ERFF                (1 << 16) /* Bit 16: Enhanced Rx FIFO Full (ERFF) */
#define CAN_ERFSR_ERFE                (1 << 17) /* Bit 17: Enhanced Rx FIFO Empty (ERFE) */
                                                /* Bits 18-26: Reserved */
#define CAN_ERFSR_ERFCLR              (1 << 27) /* Bit 27: Enhanced Rx FIFO Clear (ERFCLR) */
#define CAN_ERFSR_ERFDA               (1 << 28) /* Bit 28: Enhanced Rx FIFO Data Available (ERFDA) */
#define CAN_ERFSR_ERFWMI              (1 << 29) /* Bit 29: Enhanced Rx FIFO Watermark Indication (ERFWMI) */
#define CAN_ERFSR_ERFOVF              (1 << 30) /* Bit 30: Enhanced Rx FIFO Overflow (ERFOVF) */
#define CAN_ERFSR_ERFUFW              (1 << 31) /* Bit 31: Enhanced Rx FIFO Underflow (ERFUFW) */

/* High Resolution Time Stamp n (HR_TIME_STAMPn) */

#define CAN_HR_TIME_STAMP_TS_SHIFT    (0)       /* Bits 0-31: High Resolution Time Stamp (TS) */
#define CAN_HR_TIME_STAMP_TS_MASK     (0xffffffff << CAN_HR_TIME_STAMP_TS_SHIFT)

/* Enhanced Rx FIFO Filter Element n (ERFFELn) */

#define CAN_ERFFEL_FEL_SHIFT          (0)       /* Bits 0-31: Filter Element Bits (FEL) */
#define CAN_ERFFEL_FEL_MASK           (0xffffffff << CAN_ERFFEL_FEL_SHIFT)

/* CAN MB TX codes */
#define CAN_TXMB_INACTIVE             0x8       /* MB is not active. */
#define CAN_TXMB_ABORT                0x9       /* MB is aborted. */
#define CAN_TXMB_DATAORREMOTE         0xc       /* MB is a TX Data Frame(when MB RTR = 0) or */
                                                /* MB is a TX Remote Request Frame (when MB RTR = 1). */
#define CAN_TXMB_TANSWER              0xe       /* MB is a TX Response Request Frame from */
                                                /* an incoming Remote Request Frame. */
#define CAN_TXMB_NOTUSED              0xf       /* Not used.*/

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_CAN_H */
