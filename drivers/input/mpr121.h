/****************************************************************************
 * drivers/input/mpr121.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __DRIVERS_INPUT_MPR121_H
#define __DRIVERS_INPUT_MPR121_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MPR121 Address */

#define MPR121_ELE0_7_TS         (0x00)    /* Bits 0-7: E(n) Touch Status */
#define MPR121_ELE8_11_TS        (0x01)    /* Bits 0-7: E(n)TS, OVCF */
#define MPR121_ELE0_7_OOR        (0x02)    /* Bits 0-7: E(n)OOR status */
#define MPR121_ELE8_11_OOR       (0x03)    /* Bits 0-7: E(n)OOR, ACFF, ARFF */
#define MPR121_ELE0_FDL          (0x04)    /* Bits 0-7: ELE0 Filtered Data LSB */
#define MPR121_ELE0_FDM          (0x05)    /* Bits 0-1: ELE0 Filtered Data MSB */
#define MPR121_ELE1_FDL          (0x06)    /* Bits 0-7: ELE1 Filtered Data LSB */
#define MPR121_ELE1_FDM          (0x07)    /* Bits 0-1: ELE1 Filtered Data MSB */
#define MPR121_ELE2_FDL          (0x08)    /* Bits 0-7: ELE2 Filtered Data LSB */
#define MPR121_ELE2_FDM          (0x09)    /* Bits 0-1: ELE2 Filtered Data MSB */
#define MPR121_ELE3_FDL          (0x0a)    /* Bits 0-7: ELE3 Filtered Data LSB */
#define MPR121_ELE3_FDM          (0x0b)    /* Bits 0-1: ELE3 Filtered Data MSB */
#define MPR121_ELE4_FDL          (0x0c)    /* Bits 0-7: ELE4 Filtered Data LSB */
#define MPR121_ELE4_FDM          (0x0d)    /* Bits 0-1: ELE4 Filtered Data MSB */
#define MPR121_ELE5_FDL          (0x0e)    /* Bits 0-7: ELE5 Filtered Data LSB */
#define MPR121_ELE5_FDM          (0x0f)    /* Bits 0-1: ELE5 Filtered Data MSB */
#define MPR121_ELE6_FDL          (0x10)    /* Bits 0-7: ELE6 Filtered Data LSB */
#define MPR121_ELE6_FDM          (0x11)    /* Bits 0-1: ELE6 Filtered Data MSB */
#define MPR121_ELE7_FDL          (0x12)    /* Bits 0-7: ELE7 Filtered Data LSB */
#define MPR121_ELE7_FDM          (0x13)    /* Bits 0-1: ELE7 Filtered Data MSB */
#define MPR121_ELE8_FDL          (0x14)    /* Bits 0-7: ELE8 Filtered Data LSB */
#define MPR121_ELE8_FDM          (0x15)    /* Bits 0-1: ELE8 Filtered Data MSB */
#define MPR121_ELE9_FDL          (0x16)    /* Bits 0-7: ELE9 Filtered Data LSB */
#define MPR121_ELE9_FDM          (0x17)    /* Bits 0-1: ELE9 Filtered Data MSB */
#define MPR121_ELE10_FDL         (0x18)    /* Bits 0-7: ELE10 Filtered Data LSB */
#define MPR121_ELE10_FDM         (0x19)    /* Bits 0-1: ELE10 Filtered Data MSB */
#define MPR121_ELE11_FDL         (0x1a)    /* Bits 0-7: ELE11 Filtered Data LSB */
#define MPR121_ELE11_FDM         (0x1b)    /* Bits 0-1: ELE11 Filtered Data MSB */
#define MPR121_ELE12_FDL         (0x1c)    /* Bits 0-7: ELEPROX Filtered Data LSB */
#define MPR121_ELE12_FDM         (0x1d)    /* Bits 0-1: ELEPROX Filtered Data MSB */
#define MPR121_ELE0_BV           (0x1e)    /* Bits 0-7: ELE0 Baseline Value */
#define MPR121_ELE1_BV           (0x1f)    /* Bits 0-7: ELE1 Baseline Value */
#define MPR121_ELE2_BV           (0x20)    /* Bits 0-7: ELE2 Baseline Value */
#define MPR121_ELE3_BV           (0x21)    /* Bits 0-7: ELE3 Baseline Value */
#define MPR121_ELE4_BV           (0x22)    /* Bits 0-7: ELE4 Baseline Value */
#define MPR121_ELE5_BV           (0x23)    /* Bits 0-7: ELE5 Baseline Value */
#define MPR121_ELE6_BV           (0x24)    /* Bits 0-7: ELE6 Baseline Value */
#define MPR121_ELE7_BV           (0x25)    /* Bits 0-7: ELE7 Baseline Value */
#define MPR121_ELE8_BV           (0x26)    /* Bits 0-7: ELE8 Baseline Value */
#define MPR121_ELE9_BV           (0x27)    /* Bits 0-7: ELE9 Baseline Value */
#define MPR121_ELE10_BV          (0x28)    /* Bits 0-7: ELE10 Baseline Value */
#define MPR121_ELE11_BV          (0x29)    /* Bits 0-7: ELE11 Baseline Value */
#define MPR121_ELE12_BV          (0x2a)    /* Bits 0-7: ELEPROX Baseline Value */
#define MPR121_MHDR              (0x2b)    /* Bits 0-5: MHD Rising */
#define MPR121_NHDR              (0x2c)    /* Bits 0-5: NHD Rising */
#define MPR121_NCLR              (0x2d)    /* Bits 0-7: NCL Rising */
#define MPR121_FDLR              (0x2e)    /* Bits 0-7: FDL Rising */
#define MPR121_MHDF              (0x2f)    /* Bits 0-5: MHD Falling */
#define MPR121_NHDF              (0x30)    /* Bits 0-5: NHD Falling */
#define MPR121_NCLF              (0x31)    /* Bits 0-7: NCL Falling */
#define MPR121_FDLF              (0x32)    /* Bits 0-7: FDL Falling */
#define MPR121_NHDT              (0x33)    /* Bits 0-5: NHD Touched */
#define MPR121_NCLT              (0x34)    /* Bits 0-7: NCL Touched */
#define MPR121_FDLT              (0x35)    /* Bits 0-7: FDL Touched */
#define MPR121_MHDPROXR          (0x36)    /* Bits 0-5: ELEPROX MHD Rising */
#define MPR121_NHDPROXR          (0x37)    /* Bits 0-5: ELEPROX NHD Rising */
#define MPR121_NCLPROXR          (0x38)    /* Bits 0-7: ELEPROX NCL Rising */
#define MPR121_FDLPROXR          (0x39)    /* Bits 0-7: ELEPROX FDL Rising */
#define MPR121_MHDPROXF          (0x3a)    /* Bits 0-5: ELEPROX MHD Falling */
#define MPR121_NHDPROXF          (0x3b)    /* Bits 0-5: ELEPROX NHD Falling */
#define MPR121_NCLPROXF          (0x3c)    /* Bits 0-7: ELEPROX NCL Falling */
#define MPR121_FDLPROXF          (0x3d)    /* Bits 0-7: ELEPROX FDL Falling */
#define MPR121_NHDPROXT          (0x3e)    /* Bits 0-5: ELEPROX NHD Touched */
#define MPR121_NCLPROXT          (0x3f)    /* Bits 0-7: ELEPROX NCL Touched */
#define MPR121_FDLPROXT          (0x40)    /* Bits 0-7: ELEPROX FDL Touched */
#define MPR121_E0TTH             (0x41)    /* Bits 0-7: ELE0 Touch Threshold */
#define MPR121_E0RTH             (0x42)    /* Bits 0-7: ELE0 Release Threshold */
#define MPR121_E1TTH             (0x43)    /* Bits 0-7: ELE1 Touch Threshold */
#define MPR121_E1RTH             (0x44)    /* Bits 0-7: ELE1 Release Threshold */
#define MPR121_E2TTH             (0x45)    /* Bits 0-7: ELE2 Touch Threshold */
#define MPR121_E2RTH             (0x46)    /* Bits 0-7: ELE2 Release Threshold */
#define MPR121_E3TTH             (0x47)    /* Bits 0-7: ELE3 Touch Threshold */
#define MPR121_E3RTH             (0x48)    /* Bits 0-7: ELE3 Release Threshold */
#define MPR121_E4TTH             (0x49)    /* Bits 0-7: ELE4 Touch Threshold */
#define MPR121_E4RTH             (0x4a)    /* Bits 0-7: ELE4 Release Threshold */
#define MPR121_E5TTH             (0x4b)    /* Bits 0-7: ELE5 Touch Threshold */
#define MPR121_E5RTH             (0x4c)    /* Bits 0-7: ELE5 Release Threshold */
#define MPR121_E6TTH             (0x4d)    /* Bits 0-7: ELE6 Touch Threshold */
#define MPR121_E6RTH             (0x4e)    /* Bits 0-7: ELE6 Release Threshold */
#define MPR121_E7TTH             (0x4f)    /* Bits 0-7: ELE7 Touch Threshold */
#define MPR121_E7RTH             (0x50)    /* Bits 0-7: ELE7 Release Threshold */
#define MPR121_E8TTH             (0x51)    /* Bits 0-7: ELE8 Touch Threshold */
#define MPR121_E8RTH             (0x52)    /* Bits 0-7: ELE8 Release Threshold */
#define MPR121_E9TTH             (0x53)    /* Bits 0-7: ELE9 Touch Threshold */
#define MPR121_E9RTH             (0x54)    /* Bits 0-7: ELE9 Release Threshold */
#define MPR121_E10TTH            (0x55)    /* Bits 0-7: ELE10 Touch Threshold */
#define MPR121_E10RTH            (0x56)    /* Bits 0-7: ELE10 Release Threshold */
#define MPR121_E11TTH            (0x57)    /* Bits 0-7: ELE11 Touch Threshold */
#define MPR121_E11RTH            (0x58)    /* Bits 0-7: ELE11 Release Threshold */
#define MPR121_E12TTH            (0x59)    /* Bits 0-7: ELE12 Touch Threshold */
#define MPR121_E12RTH            (0x5a)    /* Bits 0-7: ELE12 Release Threshold */
#define MPR121_DT_DR             (0x5b)    /* Bits 0-2,4-6: Debounce Touch & Release */
#define MPR121_AFE1              (0x5c)    /* Bits 0-5,6-7: CDC and FFI */
#define MPR121_AFE2              (0x5d)    /* Bits 0-2,3-4,5-7: ESI, SFI, CDT */
#define MPR121_ECR               (0x5e)    /* Bits 0-4,4-5,6-7: ELE, ELEPROX, CL */
#define MPR121_CDC0              (0x5f)    /* Bits 0-5: ELE0 Electrode Current */
#define MPR121_CDC1              (0x60)    /* Bits 0-5: ELE1 Electrode Current */
#define MPR121_CDC2              (0x61)    /* Bits 0-5: ELE2 Electrode Current */
#define MPR121_CDC3              (0x62)    /* Bits 0-5: ELE3 Electrode Current */
#define MPR121_CDC4              (0x63)    /* Bits 0-5: ELE4 Electrode Current */
#define MPR121_CDC5              (0x64)    /* Bits 0-5: ELE5 Electrode Current */
#define MPR121_CDC6              (0x65)    /* Bits 0-5: ELE6 Electrode Current */
#define MPR121_CDC7              (0x66)    /* Bits 0-5: ELE7 Electrode Current */
#define MPR121_CDC8              (0x67)    /* Bits 0-5: ELE8 Electrode Current */
#define MPR121_CDC9              (0x68)    /* Bits 0-5: ELE9 Electrode Current */
#define MPR121_CDC10             (0x69)    /* Bits 0-5: ELE10 Electrode Current */
#define MPR121_CDC11             (0x6a)    /* Bits 0-5: ELE11 Electrode Current */
#define MPR121_CDC12             (0x6b)    /* Bits 0-5: ELE12 Electrode Current */
#define MPR121_CDT0_CDT1         (0x6c)    /* Bits 0-2,4-6: E0 & E1 Charge time */
#define MPR121_CDT2_CDT3         (0x6d)    /* Bits 0-2,4-6: E2 & E3 Charge time */
#define MPR121_CDT4_CDT5         (0x6e)    /* Bits 0-2,4-6: E4 & E5 Charge time */
#define MPR121_CDT6_CDT7         (0x6f)    /* Bits 0-2,4-6: E6 & E7 Charge time */
#define MPR121_CDT8_CDT9         (0x70)    /* Bits 0-2,4-6: E8 & E9 Charge time */
#define MPR121_CDT10_CDT11       (0x71)    /* Bits 0-2,4-6: E10 & E11 Charge time */
#define MPR121_CDT12             (0x72)    /* Bits 0-2: E12 Charge time */
#define MPR121_CTL0              (0x73)    /* Bits 0-7: GPIO Control Register 0 */
#define MPR121_CTL1              (0x74)    /* Bits 0-7: GPIO Control Register 0 */
#define MPR121_DAT               (0x75)    /* Bits 0-7: GPIO Data Register */
#define MPR121_DIR               (0x76)    /* Bits 0-7: GPIO Direction Control Register */
#define MPR121_EN                (0x77)    /* Bits 0-7: GPIO Enable Register */
#define MPR121_SET               (0x78)    /* Bits 0-7: GPIO Data Set Register */
#define MPR121_CLR               (0x79)    /* Bits 0-7: GPIO Data Clear Register */
#define MPR121_TOG               (0x7a)    /* Bits 0-7: GPIO Data Toggle Register */
#define MPR121_AUTOCONF0         (0x7b)    /* Bits 0,1,2-3,4-5,6-7: ACE, ARE, BVA, RETRY, AFES */
#define MPR121_AUTOCONF1         (0x7c)    /* Bits 0,1,3,7: ACFIE, ARFIE, OORIE, SCTS */
#define MPR121_UPLIMIT           (0x7d)    /* Bits 0-7: Auto Config USL Register */
#define MPR121_LOWLIMIT          (0x7e)    /* Bits 0-7: Auto Config LSL Register */
#define MPR121_TARGETLIMIT       (0x7f)    /* Bits 0-7: Auto Config Target Level Register */
#define MPR121_SRST              (0x80)    /* Bits 0-7: Soft Reset Register */

/* Factory Reserved Region (0x81~0xff) */

/* Tuning defaults for a standard 3.3 V keypad */

#define MPR121_TOUCH_THRESHOLD   12   /* Lower means more sensitive   */
#define MPR121_RELEASE_THRESHOLD 6    /* Must be < touch threshold    */

/* Touch / Release thresholds (0x41..0x64, 2 bytes each) */

#define MPR121_REG_TOUCH_THRESH(n) (0x41 + (n) * 2)
#define MPR121_REG_REL_THRESH(n)   (0x42 + (n) * 2)

/* AFE1: FFI=6 samples (00), CDC=16uA charge (10000) => 0x10 */

#define MPR121_AFE1_VALUE         0x10

/* AFE2: CDT=0.5 us (001), SFI=4 samples (00), ESI=1 ms (001) => 0x24 */

#define MPR121_AFE2_VALUE         0x24

/* Auto-config: limit registers (Vdd = 3.3 V, target ~70 % of Vdd) */

#define MPR121_UPLIMIT_VALUE      200   /* (Vdd - 0.7) / Vdd * 256  */
#define MPR121_LOWLIMIT_VALUE     130   /* UPLIMIT * 0.65           */
#define MPR121_TARGETLIMIT_VALUE  180   /* UPLIMIT * 0.9            */

/* ECR: CL=10 (use 5 MSB of baseline), ELEPROX_EN=0, ELE_EN=1100 (ELE0-11) */

#define MPR121_ECR_VALUE          0xCC   /* Run mode, all 12 electrodes */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __DRIVERS_INPUT_MPR121_H */
