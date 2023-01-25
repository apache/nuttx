/****************************************************************************
 * arch/arm/src/imxrt/hardware/rt117x/imxrt117x_iomuxc.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_RT117X_IMXRT117X_IOMUXC_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_RT117X_IMXRT117X_IOMUXC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/imxrt_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* General Purpose Register Offsets *****************************************/

/* IOMUXC GPR Register Offsets (from IMXRT_IOMUXCGPR_BASE) */

#define IMXRT_IOMUXC_GPR_GPR0_OFFSET             (0x0000)  /* GPR0 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR1_OFFSET             (0x0004)  /* GPR1 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR2_OFFSET             (0x0008)  /* GPR2 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR3_OFFSET             (0x000c)  /* GPR3 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR4_OFFSET             (0x0010)  /* GPR4 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR5_OFFSET             (0x0014)  /* GPR5 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR6_OFFSET             (0x0018)  /* GPR6 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR7_OFFSET             (0x001c)  /* GPR7 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR8_OFFSET             (0x0020)  /* GPR8 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR9_OFFSET             (0x0024)  /* GPR9 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR10_OFFSET            (0x0028)  /* GPR10 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR11_OFFSET            (0x002c)  /* GPR11 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR12_OFFSET            (0x0030)  /* GPR12 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR13_OFFSET            (0x0034)  /* GPR13 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR14_OFFSET            (0x0038)  /* GPR14 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR15_OFFSET            (0x003c)  /* GPR15 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR16_OFFSET            (0x0040)  /* GPR16 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR17_OFFSET            (0x0044)  /* GPR17 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR18_OFFSET            (0x0048)  /* GPR18 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR20_OFFSET            (0x0050)  /* GPR20 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR21_OFFSET            (0x0054)  /* GPR21 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR22_OFFSET            (0x0058)  /* GPR22 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR23_OFFSET            (0x005c)  /* GPR23 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR24_OFFSET            (0x0060)  /* GPR24 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR25_OFFSET            (0x0064)  /* GPR25 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR26_OFFSET            (0x0068)  /* GPR26 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR27_OFFSET            (0x006c)  /* GPR27 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR28_OFFSET            (0x0070)  /* GPR28 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR29_OFFSET            (0x0074)  /* GPR29 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR30_OFFSET            (0x0078)  /* GPR30 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR31_OFFSET            (0x007c)  /* GPR31 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR32_OFFSET            (0x0080)  /* GPR32 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR33_OFFSET            (0x0084)  /* GPR33 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR34_OFFSET            (0x0088)  /* GPR34 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR35_OFFSET            (0x008c)  /* GPR35 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR36_OFFSET            (0x0090)  /* GPR36 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR37_OFFSET            (0x0094)  /* GPR37 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR38_OFFSET            (0x0098)  /* GPR38 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR39_OFFSET            (0x009c)  /* GPR39 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR40_OFFSET            (0x00a0)  /* GPR40 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR41_OFFSET            (0x00a4)  /* GPR41 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR42_OFFSET            (0x00a8)  /* GPR42 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR43_OFFSET            (0x00ac)  /* GPR43 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR44_OFFSET            (0x00b0)  /* GPR44 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR45_OFFSET            (0x00b4)  /* GPR45 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR46_OFFSET            (0x00b8)  /* GPR46 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR47_OFFSET            (0x00bc)  /* GPR47 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR48_OFFSET            (0x00c0)  /* GPR48 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR49_OFFSET            (0x00c4)  /* GPR49 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR50_OFFSET            (0x00c8)  /* GPR50 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR51_OFFSET            (0x00cc)  /* GPR51 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR52_OFFSET            (0x00d0)  /* GPR52 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR53_OFFSET            (0x00d4)  /* GPR53 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR54_OFFSET            (0x00d8)  /* GPR54 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR55_OFFSET            (0x00dc)  /* GPR55 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR59_OFFSET            (0x00ec)  /* GPR59 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR62_OFFSET            (0x00f8)  /* GPR62 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR63_OFFSET            (0x00fc)  /* GPR63 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR64_OFFSET            (0x0100)  /* GPR64 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR65_OFFSET            (0x0104)  /* GPR65 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR66_OFFSET            (0x0108)  /* GPR66 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR67_OFFSET            (0x010c)  /* GPR67 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR68_OFFSET            (0x0110)  /* GPR68 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR69_OFFSET            (0x0114)  /* GPR69 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR70_OFFSET            (0x0118)  /* GPR70 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR71_OFFSET            (0x011c)  /* GPR71 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR72_OFFSET            (0x0120)  /* GPR72 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR73_OFFSET            (0x0124)  /* GPR73 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR74_OFFSET            (0x0128)  /* GPR74 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR75_OFFSET            (0x012c)  /* GPR75 General Purpose Register */
#define IMXRT_IOMUXC_GPR_GPR76_OFFSET            (0x0130)  /* GPR76 General Purpose Register */

/* IOMUXC LPSR GPR Register Offsets (from IMXRT_IOMUXCLPSRGPR_BASE) */

#define IMXRT_IOMUXC_LPSR_GPR_GPR26_OFFSET       (0x0068)  /* GPR26 General Purpose Register */
#define IMXRT_IOMUXC_LPSR_GPR_GPR33_OFFSET       (0x0084)  /* GPR33 General Purpose Register */
#define IMXRT_IOMUXC_LPSR_GPR_GPR34_OFFSET       (0x0088)  /* GPR34 General Purpose Register */
#define IMXRT_IOMUXC_LPSR_GPR_GPR35_OFFSET       (0x008c)  /* GPR35 General Purpose Register */
#define IMXRT_IOMUXC_LPSR_GPR_GPR36_OFFSET       (0x0090)  /* GPR36 General Purpose Register */
#define IMXRT_IOMUXC_LPSR_GPR_GPR37_OFFSET       (0x0094)  /* GPR37 General Purpose Register */
#define IMXRT_IOMUXC_LPSR_GPR_GPR38_OFFSET       (0x0098)  /* GPR38 General Purpose Register */
#define IMXRT_IOMUXC_LPSR_GPR_GPR39_OFFSET       (0x009c)  /* GPR39 General Purpose Register */
#define IMXRT_IOMUXC_LPSR_GPR_GPR40_OFFSET       (0x00a0)  /* GPR40 General Purpose Register */
#define IMXRT_IOMUXC_LPSR_GPR_GPR41_OFFSET       (0x00a4)  /* GPR41 General Purpose Register */

/* Pad Mux Register Indices & Offsets****************************************/

/* Pad Mux Register Indices (used by software for table lookups) */

#define IMXRT_PADMUX_GPIO_EMC_B1_00_INDEX        (  0)
#define IMXRT_PADMUX_GPIO_EMC_B1_01_INDEX        (  1)
#define IMXRT_PADMUX_GPIO_EMC_B1_02_INDEX        (  2)
#define IMXRT_PADMUX_GPIO_EMC_B1_03_INDEX        (  3)
#define IMXRT_PADMUX_GPIO_EMC_B1_04_INDEX        (  4)
#define IMXRT_PADMUX_GPIO_EMC_B1_05_INDEX        (  5)
#define IMXRT_PADMUX_GPIO_EMC_B1_06_INDEX        (  6)
#define IMXRT_PADMUX_GPIO_EMC_B1_07_INDEX        (  7)
#define IMXRT_PADMUX_GPIO_EMC_B1_08_INDEX        (  8)
#define IMXRT_PADMUX_GPIO_EMC_B1_09_INDEX        (  9)
#define IMXRT_PADMUX_GPIO_EMC_B1_10_INDEX        ( 10)
#define IMXRT_PADMUX_GPIO_EMC_B1_11_INDEX        ( 11)
#define IMXRT_PADMUX_GPIO_EMC_B1_12_INDEX        ( 12)
#define IMXRT_PADMUX_GPIO_EMC_B1_13_INDEX        ( 13)
#define IMXRT_PADMUX_GPIO_EMC_B1_14_INDEX        ( 14)
#define IMXRT_PADMUX_GPIO_EMC_B1_15_INDEX        ( 15)
#define IMXRT_PADMUX_GPIO_EMC_B1_16_INDEX        ( 16)
#define IMXRT_PADMUX_GPIO_EMC_B1_17_INDEX        ( 17)
#define IMXRT_PADMUX_GPIO_EMC_B1_18_INDEX        ( 18)
#define IMXRT_PADMUX_GPIO_EMC_B1_19_INDEX        ( 19)
#define IMXRT_PADMUX_GPIO_EMC_B1_20_INDEX        ( 20)
#define IMXRT_PADMUX_GPIO_EMC_B1_21_INDEX        ( 21)
#define IMXRT_PADMUX_GPIO_EMC_B1_22_INDEX        ( 22)
#define IMXRT_PADMUX_GPIO_EMC_B1_23_INDEX        ( 23)
#define IMXRT_PADMUX_GPIO_EMC_B1_24_INDEX        ( 24)
#define IMXRT_PADMUX_GPIO_EMC_B1_25_INDEX        ( 25)
#define IMXRT_PADMUX_GPIO_EMC_B1_26_INDEX        ( 26)
#define IMXRT_PADMUX_GPIO_EMC_B1_27_INDEX        ( 27)
#define IMXRT_PADMUX_GPIO_EMC_B1_28_INDEX        ( 28)
#define IMXRT_PADMUX_GPIO_EMC_B1_29_INDEX        ( 29)
#define IMXRT_PADMUX_GPIO_EMC_B1_30_INDEX        ( 30)
#define IMXRT_PADMUX_GPIO_EMC_B1_31_INDEX        ( 31)
#define IMXRT_PADMUX_GPIO_EMC_B1_32_INDEX        ( 32)
#define IMXRT_PADMUX_GPIO_EMC_B1_33_INDEX        ( 33)
#define IMXRT_PADMUX_GPIO_EMC_B1_34_INDEX        ( 34)
#define IMXRT_PADMUX_GPIO_EMC_B1_35_INDEX        ( 35)
#define IMXRT_PADMUX_GPIO_EMC_B1_36_INDEX        ( 36)
#define IMXRT_PADMUX_GPIO_EMC_B1_37_INDEX        ( 37)
#define IMXRT_PADMUX_GPIO_EMC_B1_38_INDEX        ( 38)
#define IMXRT_PADMUX_GPIO_EMC_B1_39_INDEX        ( 39)
#define IMXRT_PADMUX_GPIO_EMC_B1_40_INDEX        ( 40)
#define IMXRT_PADMUX_GPIO_EMC_B1_41_INDEX        ( 41)
#define IMXRT_PADMUX_GPIO_EMC_B2_00_INDEX        ( 42)
#define IMXRT_PADMUX_GPIO_EMC_B2_01_INDEX        ( 43)
#define IMXRT_PADMUX_GPIO_EMC_B2_02_INDEX        ( 44)
#define IMXRT_PADMUX_GPIO_EMC_B2_03_INDEX        ( 45)
#define IMXRT_PADMUX_GPIO_EMC_B2_04_INDEX        ( 46)
#define IMXRT_PADMUX_GPIO_EMC_B2_05_INDEX        ( 47)
#define IMXRT_PADMUX_GPIO_EMC_B2_06_INDEX        ( 48)
#define IMXRT_PADMUX_GPIO_EMC_B2_07_INDEX        ( 49)
#define IMXRT_PADMUX_GPIO_EMC_B2_08_INDEX        ( 50)
#define IMXRT_PADMUX_GPIO_EMC_B2_09_INDEX        ( 51)
#define IMXRT_PADMUX_GPIO_EMC_B2_10_INDEX        ( 52)
#define IMXRT_PADMUX_GPIO_EMC_B2_11_INDEX        ( 53)
#define IMXRT_PADMUX_GPIO_EMC_B2_12_INDEX        ( 54)
#define IMXRT_PADMUX_GPIO_EMC_B2_13_INDEX        ( 55)
#define IMXRT_PADMUX_GPIO_EMC_B2_14_INDEX        ( 56)
#define IMXRT_PADMUX_GPIO_EMC_B2_15_INDEX        ( 57)
#define IMXRT_PADMUX_GPIO_EMC_B2_16_INDEX        ( 58)
#define IMXRT_PADMUX_GPIO_EMC_B2_17_INDEX        ( 59)
#define IMXRT_PADMUX_GPIO_EMC_B2_18_INDEX        ( 60)
#define IMXRT_PADMUX_GPIO_EMC_B2_19_INDEX        ( 61)
#define IMXRT_PADMUX_GPIO_EMC_B2_20_INDEX        ( 62)
#define IMXRT_PADMUX_GPIO_AD_00_INDEX            ( 63)
#define IMXRT_PADMUX_GPIO_AD_01_INDEX            ( 64)
#define IMXRT_PADMUX_GPIO_AD_02_INDEX            ( 65)
#define IMXRT_PADMUX_GPIO_AD_03_INDEX            ( 66)
#define IMXRT_PADMUX_GPIO_AD_04_INDEX            ( 67)
#define IMXRT_PADMUX_GPIO_AD_05_INDEX            ( 68)
#define IMXRT_PADMUX_GPIO_AD_06_INDEX            ( 69)
#define IMXRT_PADMUX_GPIO_AD_07_INDEX            ( 70)
#define IMXRT_PADMUX_GPIO_AD_08_INDEX            ( 71)
#define IMXRT_PADMUX_GPIO_AD_09_INDEX            ( 72)
#define IMXRT_PADMUX_GPIO_AD_10_INDEX            ( 73)
#define IMXRT_PADMUX_GPIO_AD_11_INDEX            ( 74)
#define IMXRT_PADMUX_GPIO_AD_12_INDEX            ( 75)
#define IMXRT_PADMUX_GPIO_AD_13_INDEX            ( 76)
#define IMXRT_PADMUX_GPIO_AD_14_INDEX            ( 77)
#define IMXRT_PADMUX_GPIO_AD_15_INDEX            ( 78)
#define IMXRT_PADMUX_GPIO_AD_16_INDEX            ( 79)
#define IMXRT_PADMUX_GPIO_AD_17_INDEX            ( 80)
#define IMXRT_PADMUX_GPIO_AD_18_INDEX            ( 81)
#define IMXRT_PADMUX_GPIO_AD_19_INDEX            ( 82)
#define IMXRT_PADMUX_GPIO_AD_20_INDEX            ( 83)
#define IMXRT_PADMUX_GPIO_AD_21_INDEX            ( 84)
#define IMXRT_PADMUX_GPIO_AD_22_INDEX            ( 85)
#define IMXRT_PADMUX_GPIO_AD_23_INDEX            ( 86)
#define IMXRT_PADMUX_GPIO_AD_24_INDEX            ( 87)
#define IMXRT_PADMUX_GPIO_AD_25_INDEX            ( 88)
#define IMXRT_PADMUX_GPIO_AD_26_INDEX            ( 89)
#define IMXRT_PADMUX_GPIO_AD_27_INDEX            ( 90)
#define IMXRT_PADMUX_GPIO_AD_28_INDEX            ( 91)
#define IMXRT_PADMUX_GPIO_AD_29_INDEX            ( 92)
#define IMXRT_PADMUX_GPIO_AD_30_INDEX            ( 93)
#define IMXRT_PADMUX_GPIO_AD_31_INDEX            ( 94)
#define IMXRT_PADMUX_GPIO_AD_32_INDEX            ( 95)
#define IMXRT_PADMUX_GPIO_AD_33_INDEX            ( 96)
#define IMXRT_PADMUX_GPIO_AD_34_INDEX            ( 97)
#define IMXRT_PADMUX_GPIO_AD_35_INDEX            ( 98)
#define IMXRT_PADMUX_GPIO_SD_B1_00_INDEX         ( 99)
#define IMXRT_PADMUX_GPIO_SD_B1_01_INDEX         (100)
#define IMXRT_PADMUX_GPIO_SD_B1_02_INDEX         (101)
#define IMXRT_PADMUX_GPIO_SD_B1_03_INDEX         (102)
#define IMXRT_PADMUX_GPIO_SD_B1_04_INDEX         (103)
#define IMXRT_PADMUX_GPIO_SD_B1_05_INDEX         (104)
#define IMXRT_PADMUX_GPIO_SD_B2_00_INDEX         (105)
#define IMXRT_PADMUX_GPIO_SD_B2_01_INDEX         (106)
#define IMXRT_PADMUX_GPIO_SD_B2_02_INDEX         (107)
#define IMXRT_PADMUX_GPIO_SD_B2_03_INDEX         (108)
#define IMXRT_PADMUX_GPIO_SD_B2_04_INDEX         (109)
#define IMXRT_PADMUX_GPIO_SD_B2_05_INDEX         (110)
#define IMXRT_PADMUX_GPIO_SD_B2_06_INDEX         (111)
#define IMXRT_PADMUX_GPIO_SD_B2_07_INDEX         (112)
#define IMXRT_PADMUX_GPIO_SD_B2_08_INDEX         (113)
#define IMXRT_PADMUX_GPIO_SD_B2_09_INDEX         (114)
#define IMXRT_PADMUX_GPIO_SD_B2_10_INDEX         (115)
#define IMXRT_PADMUX_GPIO_SD_B2_11_INDEX         (116)
#define IMXRT_PADMUX_GPIO_DISP_B1_00_INDEX       (117)
#define IMXRT_PADMUX_GPIO_DISP_B1_01_INDEX       (118)
#define IMXRT_PADMUX_GPIO_DISP_B1_02_INDEX       (119)
#define IMXRT_PADMUX_GPIO_DISP_B1_03_INDEX       (120)
#define IMXRT_PADMUX_GPIO_DISP_B1_04_INDEX       (121)
#define IMXRT_PADMUX_GPIO_DISP_B1_05_INDEX       (122)
#define IMXRT_PADMUX_GPIO_DISP_B1_06_INDEX       (123)
#define IMXRT_PADMUX_GPIO_DISP_B1_07_INDEX       (124)
#define IMXRT_PADMUX_GPIO_DISP_B1_08_INDEX       (125)
#define IMXRT_PADMUX_GPIO_DISP_B1_09_INDEX       (126)
#define IMXRT_PADMUX_GPIO_DISP_B1_10_INDEX       (127)
#define IMXRT_PADMUX_GPIO_DISP_B1_11_INDEX       (128)
#define IMXRT_PADMUX_GPIO_DISP_B2_00_INDEX       (129)
#define IMXRT_PADMUX_GPIO_DISP_B2_01_INDEX       (130)
#define IMXRT_PADMUX_GPIO_DISP_B2_02_INDEX       (131)
#define IMXRT_PADMUX_GPIO_DISP_B2_03_INDEX       (132)
#define IMXRT_PADMUX_GPIO_DISP_B2_04_INDEX       (133)
#define IMXRT_PADMUX_GPIO_DISP_B2_05_INDEX       (134)
#define IMXRT_PADMUX_GPIO_DISP_B2_06_INDEX       (135)
#define IMXRT_PADMUX_GPIO_DISP_B2_07_INDEX       (136)
#define IMXRT_PADMUX_GPIO_DISP_B2_08_INDEX       (137)
#define IMXRT_PADMUX_GPIO_DISP_B2_09_INDEX       (138)
#define IMXRT_PADMUX_GPIO_DISP_B2_10_INDEX       (139)
#define IMXRT_PADMUX_GPIO_DISP_B2_11_INDEX       (140)
#define IMXRT_PADMUX_GPIO_DISP_B2_12_INDEX       (141)
#define IMXRT_PADMUX_GPIO_DISP_B2_13_INDEX       (142)
#define IMXRT_PADMUX_GPIO_DISP_B2_14_INDEX       (143)
#define IMXRT_PADMUX_GPIO_DISP_B2_15_INDEX       (144)

#define IMXRT_PADMUX_WAKEUP_INDEX                (145)
#define IMXRT_PADMUX_PMIC_ON_REQ_INDEX           (146)
#define IMXRT_PADMUX_PMIC_STBY_REQ_INDEX         (147)
#define IMXRT_PADMUX_GPIO_SNVS_00_INDEX          (148)
#define IMXRT_PADMUX_GPIO_SNVS_01_INDEX          (149)
#define IMXRT_PADMUX_GPIO_SNVS_02_INDEX          (150)
#define IMXRT_PADMUX_GPIO_SNVS_03_INDEX          (151)
#define IMXRT_PADMUX_GPIO_SNVS_04_INDEX          (152)
#define IMXRT_PADMUX_GPIO_SNVS_05_INDEX          (153)
#define IMXRT_PADMUX_GPIO_SNVS_06_INDEX          (154)
#define IMXRT_PADMUX_GPIO_SNVS_07_INDEX          (155)
#define IMXRT_PADMUX_GPIO_SNVS_08_INDEX          (156)
#define IMXRT_PADMUX_GPIO_SNVS_09_INDEX          (157)

#define IMXRT_PADMUX_GPIO_LPSR_00_INDEX          (158)
#define IMXRT_PADMUX_GPIO_LPSR_01_INDEX          (159)
#define IMXRT_PADMUX_GPIO_LPSR_02_INDEX          (160)
#define IMXRT_PADMUX_GPIO_LPSR_03_INDEX          (161)
#define IMXRT_PADMUX_GPIO_LPSR_04_INDEX          (162)
#define IMXRT_PADMUX_GPIO_LPSR_05_INDEX          (163)
#define IMXRT_PADMUX_GPIO_LPSR_06_INDEX          (164)
#define IMXRT_PADMUX_GPIO_LPSR_07_INDEX          (165)
#define IMXRT_PADMUX_GPIO_LPSR_08_INDEX          (166)
#define IMXRT_PADMUX_GPIO_LPSR_09_INDEX          (167)
#define IMXRT_PADMUX_GPIO_LPSR_10_INDEX          (168)
#define IMXRT_PADMUX_GPIO_LPSR_11_INDEX          (169)
#define IMXRT_PADMUX_GPIO_LPSR_12_INDEX          (170)
#define IMXRT_PADMUX_GPIO_LPSR_13_INDEX          (171)
#define IMXRT_PADMUX_GPIO_LPSR_14_INDEX          (172)
#define IMXRT_PADMUX_GPIO_LPSR_15_INDEX          (173)

#define IMXRT_PADMUX_NREGISTERS                  (174)

/* IOMUXC Pad Mux Register Offsets (from IMXRT_IOMUXC_BASE) */

#define IMXRT_PADMUX_OFFSET(n)                   (0x0010 + ((unsigned int)(n) << 2))

#define IMXRT_PADMUX_GPIO_EMC_B1_00_OFFSET       (0x0010)
#define IMXRT_PADMUX_GPIO_EMC_B1_01_OFFSET       (0x0014)
#define IMXRT_PADMUX_GPIO_EMC_B1_02_OFFSET       (0x0018)
#define IMXRT_PADMUX_GPIO_EMC_B1_03_OFFSET       (0x001c)
#define IMXRT_PADMUX_GPIO_EMC_B1_04_OFFSET       (0x0020)
#define IMXRT_PADMUX_GPIO_EMC_B1_05_OFFSET       (0x0024)
#define IMXRT_PADMUX_GPIO_EMC_B1_06_OFFSET       (0x0028)
#define IMXRT_PADMUX_GPIO_EMC_B1_07_OFFSET       (0x002c)
#define IMXRT_PADMUX_GPIO_EMC_B1_08_OFFSET       (0x0030)
#define IMXRT_PADMUX_GPIO_EMC_B1_09_OFFSET       (0x0034)
#define IMXRT_PADMUX_GPIO_EMC_B1_10_OFFSET       (0x0038)
#define IMXRT_PADMUX_GPIO_EMC_B1_11_OFFSET       (0x003c)
#define IMXRT_PADMUX_GPIO_EMC_B1_12_OFFSET       (0x0040)
#define IMXRT_PADMUX_GPIO_EMC_B1_13_OFFSET       (0x0044)
#define IMXRT_PADMUX_GPIO_EMC_B1_14_OFFSET       (0x0048)
#define IMXRT_PADMUX_GPIO_EMC_B1_15_OFFSET       (0x004c)
#define IMXRT_PADMUX_GPIO_EMC_B1_16_OFFSET       (0x0050)
#define IMXRT_PADMUX_GPIO_EMC_B1_17_OFFSET       (0x0054)
#define IMXRT_PADMUX_GPIO_EMC_B1_18_OFFSET       (0x0058)
#define IMXRT_PADMUX_GPIO_EMC_B1_19_OFFSET       (0x005c)
#define IMXRT_PADMUX_GPIO_EMC_B1_20_OFFSET       (0x0060)
#define IMXRT_PADMUX_GPIO_EMC_B1_21_OFFSET       (0x0064)
#define IMXRT_PADMUX_GPIO_EMC_B1_22_OFFSET       (0x0068)
#define IMXRT_PADMUX_GPIO_EMC_B1_23_OFFSET       (0x006c)
#define IMXRT_PADMUX_GPIO_EMC_B1_24_OFFSET       (0x0070)
#define IMXRT_PADMUX_GPIO_EMC_B1_25_OFFSET       (0x0074)
#define IMXRT_PADMUX_GPIO_EMC_B1_26_OFFSET       (0x0078)
#define IMXRT_PADMUX_GPIO_EMC_B1_27_OFFSET       (0x007c)
#define IMXRT_PADMUX_GPIO_EMC_B1_28_OFFSET       (0x0080)
#define IMXRT_PADMUX_GPIO_EMC_B1_29_OFFSET       (0x0084)
#define IMXRT_PADMUX_GPIO_EMC_B1_30_OFFSET       (0x0088)
#define IMXRT_PADMUX_GPIO_EMC_B1_31_OFFSET       (0x008c)
#define IMXRT_PADMUX_GPIO_EMC_B1_32_OFFSET       (0x0090)
#define IMXRT_PADMUX_GPIO_EMC_B1_33_OFFSET       (0x0094)
#define IMXRT_PADMUX_GPIO_EMC_B1_34_OFFSET       (0x0098)
#define IMXRT_PADMUX_GPIO_EMC_B1_35_OFFSET       (0x009c)
#define IMXRT_PADMUX_GPIO_EMC_B1_36_OFFSET       (0x00a0)
#define IMXRT_PADMUX_GPIO_EMC_B1_37_OFFSET       (0x00a4)
#define IMXRT_PADMUX_GPIO_EMC_B1_38_OFFSET       (0x00a8)
#define IMXRT_PADMUX_GPIO_EMC_B1_39_OFFSET       (0x00ac)
#define IMXRT_PADMUX_GPIO_EMC_B1_40_OFFSET       (0x00b0)
#define IMXRT_PADMUX_GPIO_EMC_B1_41_OFFSET       (0x00b4)
#define IMXRT_PADMUX_GPIO_EMC_B2_00_OFFSET       (0x00b8)
#define IMXRT_PADMUX_GPIO_EMC_B2_01_OFFSET       (0x00bc)
#define IMXRT_PADMUX_GPIO_EMC_B2_02_OFFSET       (0x00c0)
#define IMXRT_PADMUX_GPIO_EMC_B2_03_OFFSET       (0x00c4)
#define IMXRT_PADMUX_GPIO_EMC_B2_04_OFFSET       (0x00c8)
#define IMXRT_PADMUX_GPIO_EMC_B2_05_OFFSET       (0x00cc)
#define IMXRT_PADMUX_GPIO_EMC_B2_06_OFFSET       (0x00d0)
#define IMXRT_PADMUX_GPIO_EMC_B2_07_OFFSET       (0x00d4)
#define IMXRT_PADMUX_GPIO_EMC_B2_08_OFFSET       (0x00d8)
#define IMXRT_PADMUX_GPIO_EMC_B2_09_OFFSET       (0x00dc)
#define IMXRT_PADMUX_GPIO_EMC_B2_10_OFFSET       (0x00e0)
#define IMXRT_PADMUX_GPIO_EMC_B2_11_OFFSET       (0x00e4)
#define IMXRT_PADMUX_GPIO_EMC_B2_12_OFFSET       (0x00e8)
#define IMXRT_PADMUX_GPIO_EMC_B2_13_OFFSET       (0x00ec)
#define IMXRT_PADMUX_GPIO_EMC_B2_14_OFFSET       (0x00f0)
#define IMXRT_PADMUX_GPIO_EMC_B2_15_OFFSET       (0x00f4)
#define IMXRT_PADMUX_GPIO_EMC_B2_16_OFFSET       (0x00f8)
#define IMXRT_PADMUX_GPIO_EMC_B2_17_OFFSET       (0x00fc)
#define IMXRT_PADMUX_GPIO_EMC_B2_18_OFFSET       (0x0100)
#define IMXRT_PADMUX_GPIO_EMC_B2_19_OFFSET       (0x0104)
#define IMXRT_PADMUX_GPIO_EMC_B2_20_OFFSET       (0x0108)
#define IMXRT_PADMUX_GPIO_AD_00_OFFSET           (0x010c)
#define IMXRT_PADMUX_GPIO_AD_01_OFFSET           (0x0110)
#define IMXRT_PADMUX_GPIO_AD_02_OFFSET           (0x0114)
#define IMXRT_PADMUX_GPIO_AD_03_OFFSET           (0x0118)
#define IMXRT_PADMUX_GPIO_AD_04_OFFSET           (0x011c)
#define IMXRT_PADMUX_GPIO_AD_05_OFFSET           (0x0120)
#define IMXRT_PADMUX_GPIO_AD_06_OFFSET           (0x0124)
#define IMXRT_PADMUX_GPIO_AD_07_OFFSET           (0x0128)
#define IMXRT_PADMUX_GPIO_AD_08_OFFSET           (0x012c)
#define IMXRT_PADMUX_GPIO_AD_09_OFFSET           (0x0130)
#define IMXRT_PADMUX_GPIO_AD_10_OFFSET           (0x0134)
#define IMXRT_PADMUX_GPIO_AD_11_OFFSET           (0x0138)
#define IMXRT_PADMUX_GPIO_AD_12_OFFSET           (0x013c)
#define IMXRT_PADMUX_GPIO_AD_13_OFFSET           (0x0140)
#define IMXRT_PADMUX_GPIO_AD_14_OFFSET           (0x0144)
#define IMXRT_PADMUX_GPIO_AD_15_OFFSET           (0x0148)
#define IMXRT_PADMUX_GPIO_AD_16_OFFSET           (0x014c)
#define IMXRT_PADMUX_GPIO_AD_17_OFFSET           (0x0150)
#define IMXRT_PADMUX_GPIO_AD_18_OFFSET           (0x0154)
#define IMXRT_PADMUX_GPIO_AD_19_OFFSET           (0x0158)
#define IMXRT_PADMUX_GPIO_AD_20_OFFSET           (0x015c)
#define IMXRT_PADMUX_GPIO_AD_21_OFFSET           (0x0160)
#define IMXRT_PADMUX_GPIO_AD_22_OFFSET           (0x0164)
#define IMXRT_PADMUX_GPIO_AD_23_OFFSET           (0x0168)
#define IMXRT_PADMUX_GPIO_AD_24_OFFSET           (0x016c)
#define IMXRT_PADMUX_GPIO_AD_25_OFFSET           (0x0170)
#define IMXRT_PADMUX_GPIO_AD_26_OFFSET           (0x0174)
#define IMXRT_PADMUX_GPIO_AD_27_OFFSET           (0x0178)
#define IMXRT_PADMUX_GPIO_AD_28_OFFSET           (0x017c)
#define IMXRT_PADMUX_GPIO_AD_29_OFFSET           (0x0180)
#define IMXRT_PADMUX_GPIO_AD_30_OFFSET           (0x0184)
#define IMXRT_PADMUX_GPIO_AD_31_OFFSET           (0x0188)
#define IMXRT_PADMUX_GPIO_AD_32_OFFSET           (0x018c)
#define IMXRT_PADMUX_GPIO_AD_33_OFFSET           (0x0190)
#define IMXRT_PADMUX_GPIO_AD_34_OFFSET           (0x0194)
#define IMXRT_PADMUX_GPIO_AD_35_OFFSET           (0x0198)
#define IMXRT_PADMUX_GPIO_SD_B1_00_OFFSET        (0x019c)
#define IMXRT_PADMUX_GPIO_SD_B1_01_OFFSET        (0x01a0)
#define IMXRT_PADMUX_GPIO_SD_B1_02_OFFSET        (0x01a4)
#define IMXRT_PADMUX_GPIO_SD_B1_03_OFFSET        (0x01a8)
#define IMXRT_PADMUX_GPIO_SD_B1_04_OFFSET        (0x01ac)
#define IMXRT_PADMUX_GPIO_SD_B1_05_OFFSET        (0x01b0)
#define IMXRT_PADMUX_GPIO_SD_B2_00_OFFSET        (0x01b4)
#define IMXRT_PADMUX_GPIO_SD_B2_01_OFFSET        (0x01b8)
#define IMXRT_PADMUX_GPIO_SD_B2_02_OFFSET        (0x01bc)
#define IMXRT_PADMUX_GPIO_SD_B2_03_OFFSET        (0x01c0)
#define IMXRT_PADMUX_GPIO_SD_B2_04_OFFSET        (0x01c4)
#define IMXRT_PADMUX_GPIO_SD_B2_05_OFFSET        (0x01c8)
#define IMXRT_PADMUX_GPIO_SD_B2_06_OFFSET        (0x01cc)
#define IMXRT_PADMUX_GPIO_SD_B2_07_OFFSET        (0x01d0)
#define IMXRT_PADMUX_GPIO_SD_B2_08_OFFSET        (0x01d4)
#define IMXRT_PADMUX_GPIO_SD_B2_09_OFFSET        (0x01d8)
#define IMXRT_PADMUX_GPIO_SD_B2_10_OFFSET        (0x01dc)
#define IMXRT_PADMUX_GPIO_SD_B2_11_OFFSET        (0x01e0)
#define IMXRT_PADMUX_GPIO_DISP_B1_00_OFFSET      (0x01e4)
#define IMXRT_PADMUX_GPIO_DISP_B1_01_OFFSET      (0x01e8)
#define IMXRT_PADMUX_GPIO_DISP_B1_02_OFFSET      (0x01ec)
#define IMXRT_PADMUX_GPIO_DISP_B1_03_OFFSET      (0x01f0)
#define IMXRT_PADMUX_GPIO_DISP_B1_04_OFFSET      (0x01f4)
#define IMXRT_PADMUX_GPIO_DISP_B1_05_OFFSET      (0x01f8)
#define IMXRT_PADMUX_GPIO_DISP_B1_06_OFFSET      (0x01fc)
#define IMXRT_PADMUX_GPIO_DISP_B1_07_OFFSET      (0x0200)
#define IMXRT_PADMUX_GPIO_DISP_B1_08_OFFSET      (0x0204)
#define IMXRT_PADMUX_GPIO_DISP_B1_09_OFFSET      (0x0208)
#define IMXRT_PADMUX_GPIO_DISP_B1_10_OFFSET      (0x020c)
#define IMXRT_PADMUX_GPIO_DISP_B1_11_OFFSET      (0x0210)
#define IMXRT_PADMUX_GPIO_DISP_B2_00_OFFSET      (0x0214)
#define IMXRT_PADMUX_GPIO_DISP_B2_01_OFFSET      (0x0218)
#define IMXRT_PADMUX_GPIO_DISP_B2_02_OFFSET      (0x021c)
#define IMXRT_PADMUX_GPIO_DISP_B2_03_OFFSET      (0x0220)
#define IMXRT_PADMUX_GPIO_DISP_B2_04_OFFSET      (0x0224)
#define IMXRT_PADMUX_GPIO_DISP_B2_05_OFFSET      (0x0228)
#define IMXRT_PADMUX_GPIO_DISP_B2_06_OFFSET      (0x022c)
#define IMXRT_PADMUX_GPIO_DISP_B2_07_OFFSET      (0x0230)
#define IMXRT_PADMUX_GPIO_DISP_B2_08_OFFSET      (0x0234)
#define IMXRT_PADMUX_GPIO_DISP_B2_09_OFFSET      (0x0238)
#define IMXRT_PADMUX_GPIO_DISP_B2_10_OFFSET      (0x023c)
#define IMXRT_PADMUX_GPIO_DISP_B2_11_OFFSET      (0x0240)
#define IMXRT_PADMUX_GPIO_DISP_B2_12_OFFSET      (0x0244)
#define IMXRT_PADMUX_GPIO_DISP_B2_13_OFFSET      (0x0248)
#define IMXRT_PADMUX_GPIO_DISP_B2_14_OFFSET      (0x024c)
#define IMXRT_PADMUX_GPIO_DISP_B2_15_OFFSET      (0x0250)

/* IOMUXC_SNVS Pad Mux Register Offsets (from IMXRT_IOMUXCSNVS_BASE) */

#define IMXRT_PADMUX_OFFSET_SNVS(n)              ((unsigned int)(n) << 2)

#define IMXRT_PADMUX_WAKEUP_OFFSET               (0x0000)
#define IMXRT_PADMUX_PMIC_ON_REQ_OFFSET          (0x0004)
#define IMXRT_PADMUX_PMIC_STBY_REQ_OFFSET        (0x0008)
#define IMXRT_PADMUX_GPIO_SNVS_00_OFFSET         (0x000c)
#define IMXRT_PADMUX_GPIO_SNVS_01_OFFSET         (0x0010)
#define IMXRT_PADMUX_GPIO_SNVS_02_OFFSET         (0x0014)
#define IMXRT_PADMUX_GPIO_SNVS_03_OFFSET         (0x0018)
#define IMXRT_PADMUX_GPIO_SNVS_04_OFFSET         (0x001c)
#define IMXRT_PADMUX_GPIO_SNVS_05_OFFSET         (0x0020)
#define IMXRT_PADMUX_GPIO_SNVS_06_OFFSET         (0x0024)
#define IMXRT_PADMUX_GPIO_SNVS_07_OFFSET         (0x0028)
#define IMXRT_PADMUX_GPIO_SNVS_08_OFFSET         (0x002c)
#define IMXRT_PADMUX_GPIO_SNVS_09_OFFSET         (0x0030)

/* IOMUXC_LPSR Pad Mux Register Offsets (from IMXRT_IOMUXCLPSR_BASE) */

#define IMXRT_PADMUX_OFFSET_LPSR(n)              ((unsigned int)(n) << 2)

#define IMXRT_PADMUX_GPIO_LPSR_00_OFFSET         (0x0000)
#define IMXRT_PADMUX_GPIO_LPSR_01_OFFSET         (0x0004)
#define IMXRT_PADMUX_GPIO_LPSR_02_OFFSET         (0x0008)
#define IMXRT_PADMUX_GPIO_LPSR_03_OFFSET         (0x000c)
#define IMXRT_PADMUX_GPIO_LPSR_04_OFFSET         (0x0010)
#define IMXRT_PADMUX_GPIO_LPSR_05_OFFSET         (0x0014)
#define IMXRT_PADMUX_GPIO_LPSR_06_OFFSET         (0x0018)
#define IMXRT_PADMUX_GPIO_LPSR_07_OFFSET         (0x001c)
#define IMXRT_PADMUX_GPIO_LPSR_08_OFFSET         (0x0020)
#define IMXRT_PADMUX_GPIO_LPSR_09_OFFSET         (0x0024)
#define IMXRT_PADMUX_GPIO_LPSR_10_OFFSET         (0x0028)
#define IMXRT_PADMUX_GPIO_LPSR_11_OFFSET         (0x002c)
#define IMXRT_PADMUX_GPIO_LPSR_12_OFFSET         (0x0030)
#define IMXRT_PADMUX_GPIO_LPSR_13_OFFSET         (0x0034)
#define IMXRT_PADMUX_GPIO_LPSR_14_OFFSET         (0x0038)
#define IMXRT_PADMUX_GPIO_LPSR_15_OFFSET         (0x003c)

/* Pad Control Register Indices & Offsets ***********************************/

/* Pad Control Register Indices (used by software for table lookups) */

#define IMXRT_PADCTL_GPIO_EMC_B1_00_INDEX        (  0)
#define IMXRT_PADCTL_GPIO_EMC_B1_01_INDEX        (  1)
#define IMXRT_PADCTL_GPIO_EMC_B1_02_INDEX        (  2)
#define IMXRT_PADCTL_GPIO_EMC_B1_03_INDEX        (  3)
#define IMXRT_PADCTL_GPIO_EMC_B1_04_INDEX        (  4)
#define IMXRT_PADCTL_GPIO_EMC_B1_05_INDEX        (  5)
#define IMXRT_PADCTL_GPIO_EMC_B1_06_INDEX        (  6)
#define IMXRT_PADCTL_GPIO_EMC_B1_07_INDEX        (  7)
#define IMXRT_PADCTL_GPIO_EMC_B1_08_INDEX        (  8)
#define IMXRT_PADCTL_GPIO_EMC_B1_09_INDEX        (  9)
#define IMXRT_PADCTL_GPIO_EMC_B1_10_INDEX        ( 10)
#define IMXRT_PADCTL_GPIO_EMC_B1_11_INDEX        ( 11)
#define IMXRT_PADCTL_GPIO_EMC_B1_12_INDEX        ( 12)
#define IMXRT_PADCTL_GPIO_EMC_B1_13_INDEX        ( 13)
#define IMXRT_PADCTL_GPIO_EMC_B1_14_INDEX        ( 14)
#define IMXRT_PADCTL_GPIO_EMC_B1_15_INDEX        ( 15)
#define IMXRT_PADCTL_GPIO_EMC_B1_16_INDEX        ( 16)
#define IMXRT_PADCTL_GPIO_EMC_B1_17_INDEX        ( 17)
#define IMXRT_PADCTL_GPIO_EMC_B1_18_INDEX        ( 18)
#define IMXRT_PADCTL_GPIO_EMC_B1_19_INDEX        ( 19)
#define IMXRT_PADCTL_GPIO_EMC_B1_20_INDEX        ( 20)
#define IMXRT_PADCTL_GPIO_EMC_B1_21_INDEX        ( 21)
#define IMXRT_PADCTL_GPIO_EMC_B1_22_INDEX        ( 22)
#define IMXRT_PADCTL_GPIO_EMC_B1_23_INDEX        ( 23)
#define IMXRT_PADCTL_GPIO_EMC_B1_24_INDEX        ( 24)
#define IMXRT_PADCTL_GPIO_EMC_B1_25_INDEX        ( 25)
#define IMXRT_PADCTL_GPIO_EMC_B1_26_INDEX        ( 26)
#define IMXRT_PADCTL_GPIO_EMC_B1_27_INDEX        ( 27)
#define IMXRT_PADCTL_GPIO_EMC_B1_28_INDEX        ( 28)
#define IMXRT_PADCTL_GPIO_EMC_B1_29_INDEX        ( 29)
#define IMXRT_PADCTL_GPIO_EMC_B1_30_INDEX        ( 30)
#define IMXRT_PADCTL_GPIO_EMC_B1_31_INDEX        ( 31)
#define IMXRT_PADCTL_GPIO_EMC_B1_32_INDEX        ( 32)
#define IMXRT_PADCTL_GPIO_EMC_B1_33_INDEX        ( 33)
#define IMXRT_PADCTL_GPIO_EMC_B1_34_INDEX        ( 34)
#define IMXRT_PADCTL_GPIO_EMC_B1_35_INDEX        ( 35)
#define IMXRT_PADCTL_GPIO_EMC_B1_36_INDEX        ( 36)
#define IMXRT_PADCTL_GPIO_EMC_B1_37_INDEX        ( 37)
#define IMXRT_PADCTL_GPIO_EMC_B1_38_INDEX        ( 38)
#define IMXRT_PADCTL_GPIO_EMC_B1_39_INDEX        ( 39)
#define IMXRT_PADCTL_GPIO_EMC_B1_40_INDEX        ( 40)
#define IMXRT_PADCTL_GPIO_EMC_B1_41_INDEX        ( 41)
#define IMXRT_PADCTL_GPIO_EMC_B2_00_INDEX        ( 42)
#define IMXRT_PADCTL_GPIO_EMC_B2_01_INDEX        ( 43)
#define IMXRT_PADCTL_GPIO_EMC_B2_02_INDEX        ( 44)
#define IMXRT_PADCTL_GPIO_EMC_B2_03_INDEX        ( 45)
#define IMXRT_PADCTL_GPIO_EMC_B2_04_INDEX        ( 46)
#define IMXRT_PADCTL_GPIO_EMC_B2_05_INDEX        ( 47)
#define IMXRT_PADCTL_GPIO_EMC_B2_06_INDEX        ( 48)
#define IMXRT_PADCTL_GPIO_EMC_B2_07_INDEX        ( 49)
#define IMXRT_PADCTL_GPIO_EMC_B2_08_INDEX        ( 50)
#define IMXRT_PADCTL_GPIO_EMC_B2_09_INDEX        ( 51)
#define IMXRT_PADCTL_GPIO_EMC_B2_10_INDEX        ( 52)
#define IMXRT_PADCTL_GPIO_EMC_B2_11_INDEX        ( 53)
#define IMXRT_PADCTL_GPIO_EMC_B2_12_INDEX        ( 54)
#define IMXRT_PADCTL_GPIO_EMC_B2_13_INDEX        ( 55)
#define IMXRT_PADCTL_GPIO_EMC_B2_14_INDEX        ( 56)
#define IMXRT_PADCTL_GPIO_EMC_B2_15_INDEX        ( 57)
#define IMXRT_PADCTL_GPIO_EMC_B2_16_INDEX        ( 58)
#define IMXRT_PADCTL_GPIO_EMC_B2_17_INDEX        ( 59)
#define IMXRT_PADCTL_GPIO_EMC_B2_18_INDEX        ( 60)
#define IMXRT_PADCTL_GPIO_EMC_B2_19_INDEX        ( 61)
#define IMXRT_PADCTL_GPIO_EMC_B2_20_INDEX        ( 62)
#define IMXRT_PADCTL_GPIO_AD_00_INDEX            ( 63)
#define IMXRT_PADCTL_GPIO_AD_01_INDEX            ( 64)
#define IMXRT_PADCTL_GPIO_AD_02_INDEX            ( 65)
#define IMXRT_PADCTL_GPIO_AD_03_INDEX            ( 66)
#define IMXRT_PADCTL_GPIO_AD_04_INDEX            ( 67)
#define IMXRT_PADCTL_GPIO_AD_05_INDEX            ( 68)
#define IMXRT_PADCTL_GPIO_AD_06_INDEX            ( 69)
#define IMXRT_PADCTL_GPIO_AD_07_INDEX            ( 70)
#define IMXRT_PADCTL_GPIO_AD_08_INDEX            ( 71)
#define IMXRT_PADCTL_GPIO_AD_09_INDEX            ( 72)
#define IMXRT_PADCTL_GPIO_AD_10_INDEX            ( 73)
#define IMXRT_PADCTL_GPIO_AD_11_INDEX            ( 74)
#define IMXRT_PADCTL_GPIO_AD_12_INDEX            ( 75)
#define IMXRT_PADCTL_GPIO_AD_13_INDEX            ( 76)
#define IMXRT_PADCTL_GPIO_AD_14_INDEX            ( 77)
#define IMXRT_PADCTL_GPIO_AD_15_INDEX            ( 78)
#define IMXRT_PADCTL_GPIO_AD_16_INDEX            ( 79)
#define IMXRT_PADCTL_GPIO_AD_17_INDEX            ( 80)
#define IMXRT_PADCTL_GPIO_AD_18_INDEX            ( 81)
#define IMXRT_PADCTL_GPIO_AD_19_INDEX            ( 82)
#define IMXRT_PADCTL_GPIO_AD_20_INDEX            ( 83)
#define IMXRT_PADCTL_GPIO_AD_21_INDEX            ( 84)
#define IMXRT_PADCTL_GPIO_AD_22_INDEX            ( 85)
#define IMXRT_PADCTL_GPIO_AD_23_INDEX            ( 86)
#define IMXRT_PADCTL_GPIO_AD_24_INDEX            ( 87)
#define IMXRT_PADCTL_GPIO_AD_25_INDEX            ( 88)
#define IMXRT_PADCTL_GPIO_AD_26_INDEX            ( 89)
#define IMXRT_PADCTL_GPIO_AD_27_INDEX            ( 90)
#define IMXRT_PADCTL_GPIO_AD_28_INDEX            ( 91)
#define IMXRT_PADCTL_GPIO_AD_29_INDEX            ( 92)
#define IMXRT_PADCTL_GPIO_AD_30_INDEX            ( 93)
#define IMXRT_PADCTL_GPIO_AD_31_INDEX            ( 94)
#define IMXRT_PADCTL_GPIO_AD_32_INDEX            ( 95)
#define IMXRT_PADCTL_GPIO_AD_33_INDEX            ( 96)
#define IMXRT_PADCTL_GPIO_AD_34_INDEX            ( 97)
#define IMXRT_PADCTL_GPIO_AD_35_INDEX            ( 98)
#define IMXRT_PADCTL_GPIO_SD_B1_00_INDEX         ( 99)
#define IMXRT_PADCTL_GPIO_SD_B1_01_INDEX         (100)
#define IMXRT_PADCTL_GPIO_SD_B1_02_INDEX         (101)
#define IMXRT_PADCTL_GPIO_SD_B1_03_INDEX         (102)
#define IMXRT_PADCTL_GPIO_SD_B1_04_INDEX         (103)
#define IMXRT_PADCTL_GPIO_SD_B1_05_INDEX         (104)
#define IMXRT_PADCTL_GPIO_SD_B2_00_INDEX         (105)
#define IMXRT_PADCTL_GPIO_SD_B2_01_INDEX         (106)
#define IMXRT_PADCTL_GPIO_SD_B2_02_INDEX         (107)
#define IMXRT_PADCTL_GPIO_SD_B2_03_INDEX         (108)
#define IMXRT_PADCTL_GPIO_SD_B2_04_INDEX         (109)
#define IMXRT_PADCTL_GPIO_SD_B2_05_INDEX         (110)
#define IMXRT_PADCTL_GPIO_SD_B2_06_INDEX         (111)
#define IMXRT_PADCTL_GPIO_SD_B2_07_INDEX         (112)
#define IMXRT_PADCTL_GPIO_SD_B2_08_INDEX         (113)
#define IMXRT_PADCTL_GPIO_SD_B2_09_INDEX         (114)
#define IMXRT_PADCTL_GPIO_SD_B2_10_INDEX         (115)
#define IMXRT_PADCTL_GPIO_SD_B2_11_INDEX         (116)
#define IMXRT_PADCTL_GPIO_DISP_B1_00_INDEX       (117)
#define IMXRT_PADCTL_GPIO_DISP_B1_01_INDEX       (118)
#define IMXRT_PADCTL_GPIO_DISP_B1_02_INDEX       (119)
#define IMXRT_PADCTL_GPIO_DISP_B1_03_INDEX       (120)
#define IMXRT_PADCTL_GPIO_DISP_B1_04_INDEX       (121)
#define IMXRT_PADCTL_GPIO_DISP_B1_05_INDEX       (122)
#define IMXRT_PADCTL_GPIO_DISP_B1_06_INDEX       (123)
#define IMXRT_PADCTL_GPIO_DISP_B1_07_INDEX       (124)
#define IMXRT_PADCTL_GPIO_DISP_B1_08_INDEX       (125)
#define IMXRT_PADCTL_GPIO_DISP_B1_09_INDEX       (126)
#define IMXRT_PADCTL_GPIO_DISP_B1_10_INDEX       (127)
#define IMXRT_PADCTL_GPIO_DISP_B1_11_INDEX       (128)
#define IMXRT_PADCTL_GPIO_DISP_B2_00_INDEX       (129)
#define IMXRT_PADCTL_GPIO_DISP_B2_01_INDEX       (130)
#define IMXRT_PADCTL_GPIO_DISP_B2_02_INDEX       (131)
#define IMXRT_PADCTL_GPIO_DISP_B2_03_INDEX       (132)
#define IMXRT_PADCTL_GPIO_DISP_B2_04_INDEX       (133)
#define IMXRT_PADCTL_GPIO_DISP_B2_05_INDEX       (134)
#define IMXRT_PADCTL_GPIO_DISP_B2_06_INDEX       (135)
#define IMXRT_PADCTL_GPIO_DISP_B2_07_INDEX       (136)
#define IMXRT_PADCTL_GPIO_DISP_B2_08_INDEX       (137)
#define IMXRT_PADCTL_GPIO_DISP_B2_09_INDEX       (138)
#define IMXRT_PADCTL_GPIO_DISP_B2_10_INDEX       (139)
#define IMXRT_PADCTL_GPIO_DISP_B2_11_INDEX       (140)
#define IMXRT_PADCTL_GPIO_DISP_B2_12_INDEX       (141)
#define IMXRT_PADCTL_GPIO_DISP_B2_13_INDEX       (142)
#define IMXRT_PADCTL_GPIO_DISP_B2_14_INDEX       (143)
#define IMXRT_PADCTL_GPIO_DISP_B2_15_INDEX       (144)

#define IMXRT_PADCTL_TEST_MODE_INDEX             (145)
#define IMXRT_PADCTL_POR_B_INDEX                 (146)
#define IMXRT_PADCTL_ONOFF_INDEX                 (147)
#define IMXRT_PADCTL_WAKEUP_INDEX                (148)
#define IMXRT_PADCTL_PMIC_ON_REQ_INDEX           (149)
#define IMXRT_PADCTL_PMIC_STBY_REQ_INDEX         (150)
#define IMXRT_PADCTL_GPIO_SNVS_00_INDEX          (151)
#define IMXRT_PADCTL_GPIO_SNVS_01_INDEX          (152)
#define IMXRT_PADCTL_GPIO_SNVS_02_INDEX          (153)
#define IMXRT_PADCTL_GPIO_SNVS_03_INDEX          (154)
#define IMXRT_PADCTL_GPIO_SNVS_04_INDEX          (155)
#define IMXRT_PADCTL_GPIO_SNVS_05_INDEX          (156)
#define IMXRT_PADCTL_GPIO_SNVS_06_INDEX          (157)
#define IMXRT_PADCTL_GPIO_SNVS_07_INDEX          (158)
#define IMXRT_PADCTL_GPIO_SNVS_08_INDEX          (159)
#define IMXRT_PADCTL_GPIO_SNVS_09_INDEX          (160)

#define IMXRT_PADCTL_GPIO_LPSR_00_INDEX          (161)
#define IMXRT_PADCTL_GPIO_LPSR_01_INDEX          (162)
#define IMXRT_PADCTL_GPIO_LPSR_02_INDEX          (163)
#define IMXRT_PADCTL_GPIO_LPSR_03_INDEX          (164)
#define IMXRT_PADCTL_GPIO_LPSR_04_INDEX          (165)
#define IMXRT_PADCTL_GPIO_LPSR_05_INDEX          (166)
#define IMXRT_PADCTL_GPIO_LPSR_06_INDEX          (167)
#define IMXRT_PADCTL_GPIO_LPSR_07_INDEX          (168)
#define IMXRT_PADCTL_GPIO_LPSR_08_INDEX          (169)
#define IMXRT_PADCTL_GPIO_LPSR_09_INDEX          (170)
#define IMXRT_PADCTL_GPIO_LPSR_10_INDEX          (171)
#define IMXRT_PADCTL_GPIO_LPSR_11_INDEX          (172)
#define IMXRT_PADCTL_GPIO_LPSR_12_INDEX          (173)
#define IMXRT_PADCTL_GPIO_LPSR_13_INDEX          (174)
#define IMXRT_PADCTL_GPIO_LPSR_14_INDEX          (175)
#define IMXRT_PADCTL_GPIO_LPSR_15_INDEX          (176)

#define IMXRT_PADCTL_NREGISTERS                  (177)

/* IOMUXC Pad Control Register Offsets (from IMXRT_IOMUXC_BASE) */

#define IMXRT_PADCTL_OFFSET(n)                   (0x0254 + ((unsigned int)(n) << 2))

#define IMXRT_PADCTL_GPIO_EMC_B1_00_OFFSET       (0x0254)
#define IMXRT_PADCTL_GPIO_EMC_B1_01_OFFSET       (0x0258)
#define IMXRT_PADCTL_GPIO_EMC_B1_02_OFFSET       (0x025c)
#define IMXRT_PADCTL_GPIO_EMC_B1_03_OFFSET       (0x0260)
#define IMXRT_PADCTL_GPIO_EMC_B1_04_OFFSET       (0x0264)
#define IMXRT_PADCTL_GPIO_EMC_B1_05_OFFSET       (0x0268)
#define IMXRT_PADCTL_GPIO_EMC_B1_06_OFFSET       (0x026c)
#define IMXRT_PADCTL_GPIO_EMC_B1_07_OFFSET       (0x0270)
#define IMXRT_PADCTL_GPIO_EMC_B1_08_OFFSET       (0x0274)
#define IMXRT_PADCTL_GPIO_EMC_B1_09_OFFSET       (0x0278)
#define IMXRT_PADCTL_GPIO_EMC_B1_10_OFFSET       (0x027c)
#define IMXRT_PADCTL_GPIO_EMC_B1_11_OFFSET       (0x0280)
#define IMXRT_PADCTL_GPIO_EMC_B1_12_OFFSET       (0x0284)
#define IMXRT_PADCTL_GPIO_EMC_B1_13_OFFSET       (0x0288)
#define IMXRT_PADCTL_GPIO_EMC_B1_14_OFFSET       (0x028c)
#define IMXRT_PADCTL_GPIO_EMC_B1_15_OFFSET       (0x0290)
#define IMXRT_PADCTL_GPIO_EMC_B1_16_OFFSET       (0x0294)
#define IMXRT_PADCTL_GPIO_EMC_B1_17_OFFSET       (0x0298)
#define IMXRT_PADCTL_GPIO_EMC_B1_18_OFFSET       (0x029c)
#define IMXRT_PADCTL_GPIO_EMC_B1_19_OFFSET       (0x02a0)
#define IMXRT_PADCTL_GPIO_EMC_B1_20_OFFSET       (0x02a4)
#define IMXRT_PADCTL_GPIO_EMC_B1_21_OFFSET       (0x02a8)
#define IMXRT_PADCTL_GPIO_EMC_B1_22_OFFSET       (0x02ac)
#define IMXRT_PADCTL_GPIO_EMC_B1_23_OFFSET       (0x02b0)
#define IMXRT_PADCTL_GPIO_EMC_B1_24_OFFSET       (0x02b4)
#define IMXRT_PADCTL_GPIO_EMC_B1_25_OFFSET       (0x02b8)
#define IMXRT_PADCTL_GPIO_EMC_B1_26_OFFSET       (0x02bc)
#define IMXRT_PADCTL_GPIO_EMC_B1_27_OFFSET       (0x02c0)
#define IMXRT_PADCTL_GPIO_EMC_B1_28_OFFSET       (0x02c4)
#define IMXRT_PADCTL_GPIO_EMC_B1_29_OFFSET       (0x02c8)
#define IMXRT_PADCTL_GPIO_EMC_B1_30_OFFSET       (0x02cc)
#define IMXRT_PADCTL_GPIO_EMC_B1_31_OFFSET       (0x02d0)
#define IMXRT_PADCTL_GPIO_EMC_B1_32_OFFSET       (0x02d4)
#define IMXRT_PADCTL_GPIO_EMC_B1_33_OFFSET       (0x02d8)
#define IMXRT_PADCTL_GPIO_EMC_B1_34_OFFSET       (0x02dc)
#define IMXRT_PADCTL_GPIO_EMC_B1_35_OFFSET       (0x02e0)
#define IMXRT_PADCTL_GPIO_EMC_B1_36_OFFSET       (0x02e4)
#define IMXRT_PADCTL_GPIO_EMC_B1_37_OFFSET       (0x02e8)
#define IMXRT_PADCTL_GPIO_EMC_B1_38_OFFSET       (0x02ec)
#define IMXRT_PADCTL_GPIO_EMC_B1_39_OFFSET       (0x02f0)
#define IMXRT_PADCTL_GPIO_EMC_B1_40_OFFSET       (0x02f4)
#define IMXRT_PADCTL_GPIO_EMC_B1_41_OFFSET       (0x02f8)
#define IMXRT_PADCTL_GPIO_EMC_B2_00_OFFSET       (0x02fc)
#define IMXRT_PADCTL_GPIO_EMC_B2_01_OFFSET       (0x0300)
#define IMXRT_PADCTL_GPIO_EMC_B2_02_OFFSET       (0x0304)
#define IMXRT_PADCTL_GPIO_EMC_B2_03_OFFSET       (0x0308)
#define IMXRT_PADCTL_GPIO_EMC_B2_04_OFFSET       (0x030c)
#define IMXRT_PADCTL_GPIO_EMC_B2_05_OFFSET       (0x0310)
#define IMXRT_PADCTL_GPIO_EMC_B2_06_OFFSET       (0x0314)
#define IMXRT_PADCTL_GPIO_EMC_B2_07_OFFSET       (0x0318)
#define IMXRT_PADCTL_GPIO_EMC_B2_08_OFFSET       (0x031c)
#define IMXRT_PADCTL_GPIO_EMC_B2_09_OFFSET       (0x0320)
#define IMXRT_PADCTL_GPIO_EMC_B2_10_OFFSET       (0x0324)
#define IMXRT_PADCTL_GPIO_EMC_B2_11_OFFSET       (0x0328)
#define IMXRT_PADCTL_GPIO_EMC_B2_12_OFFSET       (0x032c)
#define IMXRT_PADCTL_GPIO_EMC_B2_13_OFFSET       (0x0330)
#define IMXRT_PADCTL_GPIO_EMC_B2_14_OFFSET       (0x0334)
#define IMXRT_PADCTL_GPIO_EMC_B2_15_OFFSET       (0x0338)
#define IMXRT_PADCTL_GPIO_EMC_B2_16_OFFSET       (0x033c)
#define IMXRT_PADCTL_GPIO_EMC_B2_17_OFFSET       (0x0340)
#define IMXRT_PADCTL_GPIO_EMC_B2_18_OFFSET       (0x0344)
#define IMXRT_PADCTL_GPIO_EMC_B2_19_OFFSET       (0x0348)
#define IMXRT_PADCTL_GPIO_EMC_B2_20_OFFSET       (0x034c)
#define IMXRT_PADCTL_GPIO_AD_00_OFFSET           (0x0350)
#define IMXRT_PADCTL_GPIO_AD_01_OFFSET           (0x0354)
#define IMXRT_PADCTL_GPIO_AD_02_OFFSET           (0x0358)
#define IMXRT_PADCTL_GPIO_AD_03_OFFSET           (0x035c)
#define IMXRT_PADCTL_GPIO_AD_04_OFFSET           (0x0360)
#define IMXRT_PADCTL_GPIO_AD_05_OFFSET           (0x0364)
#define IMXRT_PADCTL_GPIO_AD_06_OFFSET           (0x0368)
#define IMXRT_PADCTL_GPIO_AD_07_OFFSET           (0x036c)
#define IMXRT_PADCTL_GPIO_AD_08_OFFSET           (0x0370)
#define IMXRT_PADCTL_GPIO_AD_09_OFFSET           (0x0374)
#define IMXRT_PADCTL_GPIO_AD_10_OFFSET           (0x0378)
#define IMXRT_PADCTL_GPIO_AD_11_OFFSET           (0x037c)
#define IMXRT_PADCTL_GPIO_AD_12_OFFSET           (0x0380)
#define IMXRT_PADCTL_GPIO_AD_13_OFFSET           (0x0384)
#define IMXRT_PADCTL_GPIO_AD_14_OFFSET           (0x0388)
#define IMXRT_PADCTL_GPIO_AD_15_OFFSET           (0x038c)
#define IMXRT_PADCTL_GPIO_AD_16_OFFSET           (0x0390)
#define IMXRT_PADCTL_GPIO_AD_17_OFFSET           (0x0394)
#define IMXRT_PADCTL_GPIO_AD_18_OFFSET           (0x0398)
#define IMXRT_PADCTL_GPIO_AD_19_OFFSET           (0x039c)
#define IMXRT_PADCTL_GPIO_AD_20_OFFSET           (0x03a0)
#define IMXRT_PADCTL_GPIO_AD_21_OFFSET           (0x03a4)
#define IMXRT_PADCTL_GPIO_AD_22_OFFSET           (0x03a8)
#define IMXRT_PADCTL_GPIO_AD_23_OFFSET           (0x03ac)
#define IMXRT_PADCTL_GPIO_AD_24_OFFSET           (0x03b0)
#define IMXRT_PADCTL_GPIO_AD_25_OFFSET           (0x03b4)
#define IMXRT_PADCTL_GPIO_AD_26_OFFSET           (0x03b8)
#define IMXRT_PADCTL_GPIO_AD_27_OFFSET           (0x03bc)
#define IMXRT_PADCTL_GPIO_AD_28_OFFSET           (0x03c0)
#define IMXRT_PADCTL_GPIO_AD_29_OFFSET           (0x03c4)
#define IMXRT_PADCTL_GPIO_AD_30_OFFSET           (0x03c8)
#define IMXRT_PADCTL_GPIO_AD_31_OFFSET           (0x03cc)
#define IMXRT_PADCTL_GPIO_AD_32_OFFSET           (0x03d0)
#define IMXRT_PADCTL_GPIO_AD_33_OFFSET           (0x03d4)
#define IMXRT_PADCTL_GPIO_AD_34_OFFSET           (0x03d8)
#define IMXRT_PADCTL_GPIO_AD_35_OFFSET           (0x03dc)
#define IMXRT_PADCTL_GPIO_SD_B1_00_OFFSET        (0x03e0)
#define IMXRT_PADCTL_GPIO_SD_B1_01_OFFSET        (0x03e4)
#define IMXRT_PADCTL_GPIO_SD_B1_02_OFFSET        (0x03e8)
#define IMXRT_PADCTL_GPIO_SD_B1_03_OFFSET        (0x03ec)
#define IMXRT_PADCTL_GPIO_SD_B1_04_OFFSET        (0x03f0)
#define IMXRT_PADCTL_GPIO_SD_B1_05_OFFSET        (0x03f4)
#define IMXRT_PADCTL_GPIO_SD_B2_00_OFFSET        (0x03f8)
#define IMXRT_PADCTL_GPIO_SD_B2_01_OFFSET        (0x03fc)
#define IMXRT_PADCTL_GPIO_SD_B2_02_OFFSET        (0x0400)
#define IMXRT_PADCTL_GPIO_SD_B2_03_OFFSET        (0x0404)
#define IMXRT_PADCTL_GPIO_SD_B2_04_OFFSET        (0x0408)
#define IMXRT_PADCTL_GPIO_SD_B2_05_OFFSET        (0x040c)
#define IMXRT_PADCTL_GPIO_SD_B2_06_OFFSET        (0x0410)
#define IMXRT_PADCTL_GPIO_SD_B2_07_OFFSET        (0x0414)
#define IMXRT_PADCTL_GPIO_SD_B2_08_OFFSET        (0x0418)
#define IMXRT_PADCTL_GPIO_SD_B2_09_OFFSET        (0x041c)
#define IMXRT_PADCTL_GPIO_SD_B2_10_OFFSET        (0x0420)
#define IMXRT_PADCTL_GPIO_SD_B2_11_OFFSET        (0x0424)
#define IMXRT_PADCTL_GPIO_DISP_B1_00_OFFSET      (0x0428)
#define IMXRT_PADCTL_GPIO_DISP_B1_01_OFFSET      (0x042c)
#define IMXRT_PADCTL_GPIO_DISP_B1_02_OFFSET      (0x0430)
#define IMXRT_PADCTL_GPIO_DISP_B1_03_OFFSET      (0x0434)
#define IMXRT_PADCTL_GPIO_DISP_B1_04_OFFSET      (0x0438)
#define IMXRT_PADCTL_GPIO_DISP_B1_05_OFFSET      (0x043c)
#define IMXRT_PADCTL_GPIO_DISP_B1_06_OFFSET      (0x0440)
#define IMXRT_PADCTL_GPIO_DISP_B1_07_OFFSET      (0x0444)
#define IMXRT_PADCTL_GPIO_DISP_B1_08_OFFSET      (0x0448)
#define IMXRT_PADCTL_GPIO_DISP_B1_09_OFFSET      (0x044c)
#define IMXRT_PADCTL_GPIO_DISP_B1_10_OFFSET      (0x0450)
#define IMXRT_PADCTL_GPIO_DISP_B1_11_OFFSET      (0x0454)
#define IMXRT_PADCTL_GPIO_DISP_B2_00_OFFSET      (0x0458)
#define IMXRT_PADCTL_GPIO_DISP_B2_01_OFFSET      (0x045c)
#define IMXRT_PADCTL_GPIO_DISP_B2_02_OFFSET      (0x0460)
#define IMXRT_PADCTL_GPIO_DISP_B2_03_OFFSET      (0x0464)
#define IMXRT_PADCTL_GPIO_DISP_B2_04_OFFSET      (0x0468)
#define IMXRT_PADCTL_GPIO_DISP_B2_05_OFFSET      (0x046c)
#define IMXRT_PADCTL_GPIO_DISP_B2_06_OFFSET      (0x0470)
#define IMXRT_PADCTL_GPIO_DISP_B2_07_OFFSET      (0x0474)
#define IMXRT_PADCTL_GPIO_DISP_B2_08_OFFSET      (0x0478)
#define IMXRT_PADCTL_GPIO_DISP_B2_09_OFFSET      (0x047c)
#define IMXRT_PADCTL_GPIO_DISP_B2_10_OFFSET      (0x0480)
#define IMXRT_PADCTL_GPIO_DISP_B2_11_OFFSET      (0x0484)
#define IMXRT_PADCTL_GPIO_DISP_B2_12_OFFSET      (0x0488)
#define IMXRT_PADCTL_GPIO_DISP_B2_13_OFFSET      (0x048c)
#define IMXRT_PADCTL_GPIO_DISP_B2_14_OFFSET      (0x0490)
#define IMXRT_PADCTL_GPIO_DISP_B2_15_OFFSET      (0x0494)

/* IOMUXC_SNVS Pad Control Register Offsets (from IMXRT_IOMUXCSNVS_BASE) */

#define IMXRT_PADCTL_OFFSET_SNVS(n)              (0x0034 + ((unsigned int)(n) << 2))

#define IMXRT_PADCTL_TEST_MODE_OFFSET            (0x0034)
#define IMXRT_PADCTL_POR_B_OFFSET                (0x0038)
#define IMXRT_PADCTL_ONOFF_OFFSET                (0x003c)
#define IMXRT_PADCTL_WAKEUP_OFFSET               (0x0040)
#define IMXRT_PADCTL_PMIC_ON_REQ_OFFSET          (0x0044)
#define IMXRT_PADCTL_PMIC_STBY_REQ_OFFSET        (0x0048)
#define IMXRT_PADCTL_GPIO_SNVS_00_OFFSET         (0x004c)
#define IMXRT_PADCTL_GPIO_SNVS_01_OFFSET         (0x0050)
#define IMXRT_PADCTL_GPIO_SNVS_02_OFFSET         (0x0054)
#define IMXRT_PADCTL_GPIO_SNVS_03_OFFSET         (0x0058)
#define IMXRT_PADCTL_GPIO_SNVS_04_OFFSET         (0x005c)
#define IMXRT_PADCTL_GPIO_SNVS_05_OFFSET         (0x0060)
#define IMXRT_PADCTL_GPIO_SNVS_06_OFFSET         (0x0064)
#define IMXRT_PADCTL_GPIO_SNVS_07_OFFSET         (0x0068)
#define IMXRT_PADCTL_GPIO_SNVS_08_OFFSET         (0x006c)
#define IMXRT_PADCTL_GPIO_SNVS_09_OFFSET         (0x0070)

/* IOMUXC_LPSR Pad Control Register Offsets (from IMXRT_IOMUXCLPSR_BASE) */

#define IMXRT_PADCTL_OFFSET_LPSR(n)              (0x0040 + ((unsigned int)(n) << 2))

#define IMXRT_PADCTL_GPIO_LPSR_00_OFFSET         (0x0040)
#define IMXRT_PADCTL_GPIO_LPSR_01_OFFSET         (0x0044)
#define IMXRT_PADCTL_GPIO_LPSR_02_OFFSET         (0x0048)
#define IMXRT_PADCTL_GPIO_LPSR_03_OFFSET         (0x004c)
#define IMXRT_PADCTL_GPIO_LPSR_04_OFFSET         (0x0050)
#define IMXRT_PADCTL_GPIO_LPSR_05_OFFSET         (0x0054)
#define IMXRT_PADCTL_GPIO_LPSR_06_OFFSET         (0x0058)
#define IMXRT_PADCTL_GPIO_LPSR_07_OFFSET         (0x005c)
#define IMXRT_PADCTL_GPIO_LPSR_08_OFFSET         (0x0060)
#define IMXRT_PADCTL_GPIO_LPSR_09_OFFSET         (0x0064)
#define IMXRT_PADCTL_GPIO_LPSR_10_OFFSET         (0x0068)
#define IMXRT_PADCTL_GPIO_LPSR_11_OFFSET         (0x006c)
#define IMXRT_PADCTL_GPIO_LPSR_12_OFFSET         (0x0070)
#define IMXRT_PADCTL_GPIO_LPSR_13_OFFSET         (0x0074)
#define IMXRT_PADCTL_GPIO_LPSR_14_OFFSET         (0x0078)
#define IMXRT_PADCTL_GPIO_LPSR_15_OFFSET         (0x007c)

/* IOMUXC Select Input Daisy Chain Register Offsets
 * (from IMXRT_IOMUXC_BASE)
 */

#define IMXRT_INPUT_INDEX2OFFSET(n)              (0x0498 + ((unsigned int)(n) << 2))
#define IMXRT_INPUT_OFFSET2INDEX(o)              (((unsigned int)(o) - 0x0498) >> 2)

#define IMXRT_INPUT_FLEXCAN1_RX_OFFSET           (0x0498)
#define IMXRT_INPUT_FLEXCAN2_RX_OFFSET           (0x049c)
#define IMXRT_INPUT_CCM_ENET_QOS_REFCLK_OFFSET   (0x04a0)
#define IMXRT_INPUT_CCM_ENET_QOS_TXCLK_OFFSET    (0x04a4)
#define IMXRT_INPUT_ENET_IPG_CLK_RMII_OFFSET     (0x04a8)
#define IMXRT_INPUT_ENET_MDIO_OFFSET             (0x04ac)
#define IMXRT_INPUT_ENET_RXDATA0_OFFSET          (0x04b0)
#define IMXRT_INPUT_ENET_RXDATA1_OFFSET          (0x04b4)
#define IMXRT_INPUT_ENET_RXEN_OFFSET             (0x04b8)
#define IMXRT_INPUT_ENET_RXERR_OFFSET            (0x04bc)
#define IMXRT_INPUT_ENET_TXCLK_OFFSET            (0x04c0)
#define IMXRT_INPUT_ENET_1G_IPG_CLK_RMII_OFFSET  (0x04c4)
#define IMXRT_INPUT_ENET_1G_MDIO_OFFSET          (0x04c8)
#define IMXRT_INPUT_ENET_1G_RXCLK_OFFSET         (0x04cc)
#define IMXRT_INPUT_ENET_1G_RXDATA0_OFFSET       (0x04d0)
#define IMXRT_INPUT_ENET_1G_RXDATA1_OFFSET       (0x04d4)
#define IMXRT_INPUT_ENET_1G_RXDATA2_OFFSET       (0x04d8)
#define IMXRT_INPUT_ENET_1G_RXDATA3_OFFSET       (0x04dc)
#define IMXRT_INPUT_ENET_1G_RXEN_OFFSET          (0x04e0)
#define IMXRT_INPUT_ENET_1G_RXERR_OFFSET         (0x04e4)
#define IMXRT_INPUT_ENET_1G_TXCLK_OFFSET         (0x04e8)
#define IMXRT_INPUT_ENET_QOS_MDIO_OFFSET         (0x04ec)
#define IMXRT_INPUT_ENET_QOS_RXDATA0_OFFSET      (0x04f0)
#define IMXRT_INPUT_ENET_QOS_RXDATA1_OFFSET      (0x04f4)
#define IMXRT_INPUT_ENET_QOS_RXDV_OFFSET         (0x04f8)
#define IMXRT_INPUT_ENET_QOS_RXERR_OFFSET        (0x04fc)
#define IMXRT_INPUT_FLEXPWM1_PWMA0_OFFSET        (0x0500)
#define IMXRT_INPUT_FLEXPWM1_PWMA1_OFFSET        (0x0504)
#define IMXRT_INPUT_FLEXPWM1_PWMA2_OFFSET        (0x0508)
#define IMXRT_INPUT_FLEXPWM1_PWMB0_OFFSET        (0x050c)
#define IMXRT_INPUT_FLEXPWM1_PWMB1_OFFSET        (0x0510)
#define IMXRT_INPUT_FLEXPWM1_PWMB2_OFFSET        (0x0514)
#define IMXRT_INPUT_FLEXPWM2_PWMA0_OFFSET        (0x0518)
#define IMXRT_INPUT_FLEXPWM2_PWMA1_OFFSET        (0x051c)
#define IMXRT_INPUT_FLEXPWM2_PWMA2_OFFSET        (0x0520)
#define IMXRT_INPUT_FLEXPWM2_PWMB0_OFFSET        (0x0524)
#define IMXRT_INPUT_FLEXPWM2_PWMB1_OFFSET        (0x0528)
#define IMXRT_INPUT_FLEXPWM2_PWMB2_OFFSET        (0x052c)
#define IMXRT_INPUT_FLEXPWM3_PWMA0_OFFSET        (0x0530)
#define IMXRT_INPUT_FLEXPWM3_PWMA1_OFFSET        (0x0534)
#define IMXRT_INPUT_FLEXPWM3_PWMA2_OFFSET        (0x0538)
#define IMXRT_INPUT_FLEXPWM3_PWMA3_OFFSET        (0x053c)
#define IMXRT_INPUT_FLEXPWM3_PWMB0_OFFSET        (0x0540)
#define IMXRT_INPUT_FLEXPWM3_PWMB1_OFFSET        (0x0544)
#define IMXRT_INPUT_FLEXPWM3_PWMB2_OFFSET        (0x0548)
#define IMXRT_INPUT_FLEXPWM3_PWMB3_OFFSET        (0x054c)
#define IMXRT_INPUT_FLEXSPI1A_DQS_OFFSET         (0x0550)
#define IMXRT_INPUT_FLEXSPI1A_DATA0_OFFSET       (0x0554)
#define IMXRT_INPUT_FLEXSPI1A_DATA1_OFFSET       (0x0558)
#define IMXRT_INPUT_FLEXSPI1A_DATA2_OFFSET       (0x055c)
#define IMXRT_INPUT_FLEXSPI1A_DATA3_OFFSET       (0x0560)
#define IMXRT_INPUT_FLEXSPI1B_DATA0_OFFSET       (0x0564)
#define IMXRT_INPUT_FLEXSPI1B_DATA1_OFFSET       (0x0568)
#define IMXRT_INPUT_FLEXSPI1B_DATA2_OFFSET       (0x056c)
#define IMXRT_INPUT_FLEXSPI1B_DATA3_OFFSET       (0x0570)
#define IMXRT_INPUT_FLEXSPI1A_SCK_OFFSET         (0x0574)
#define IMXRT_INPUT_FLEXSPI1B_SCK_OFFSET         (0x0578)
#define IMXRT_INPUT_FLEXSPI2A_DATA0_OFFSET       (0x057c)
#define IMXRT_INPUT_FLEXSPI2A_DATA1_OFFSET       (0x0580)
#define IMXRT_INPUT_FLEXSPI2A_DATA2_OFFSET       (0x0584)
#define IMXRT_INPUT_FLEXSPI2A_DATA3_OFFSET       (0x0588)
#define IMXRT_INPUT_FLEXSPI2A_SCK_OFFSET         (0x058c)
#define IMXRT_INPUT_GPT3_CAPIN1_OFFSET           (0x0590)
#define IMXRT_INPUT_GPT3_CAPIN2_OFFSET           (0x0594)
#define IMXRT_INPUT_GPT3_CLKIN_OFFSET            (0x0598)
#define IMXRT_INPUT_KPP_COL6_OFFSET              (0x059c)
#define IMXRT_INPUT_KPP_COL7_OFFSET              (0x05a0)
#define IMXRT_INPUT_KPP_ROW6_OFFSET              (0x05a4)
#define IMXRT_INPUT_KPP_ROW7_OFFSET              (0x05a8)
#define IMXRT_INPUT_LPI2C1_SCL_OFFSET            (0x05ac)
#define IMXRT_INPUT_LPI2C1_SDA_OFFSET            (0x05b0)
#define IMXRT_INPUT_LPI2C2_SCL_OFFSET            (0x05b4)
#define IMXRT_INPUT_LPI2C2_SDA_OFFSET            (0x05b8)
#define IMXRT_INPUT_LPI2C3_SCL_OFFSET            (0x05bc)
#define IMXRT_INPUT_LPI2C3_SDA_OFFSET            (0x05c0)
#define IMXRT_INPUT_LPI2C4_SCL_OFFSET            (0x05c4)
#define IMXRT_INPUT_LPI2C4_SDA_OFFSET            (0x05c8)
#define IMXRT_INPUT_LPSPI1_PCS0_OFFSET           (0x05cc)
#define IMXRT_INPUT_LPSPI1_SCK_OFFSET            (0x05d0)
#define IMXRT_INPUT_LPSPI1_SDI_OFFSET            (0x05d4)
#define IMXRT_INPUT_LPSPI1_SDO_OFFSET            (0x05d8)
#define IMXRT_INPUT_LPSPI2_PCS0_OFFSET           (0x05dc)
#define IMXRT_INPUT_LPSPI2_PCS1_OFFSET           (0x05e0)
#define IMXRT_INPUT_LPSPI2_SCK_OFFSET            (0x05e4)
#define IMXRT_INPUT_LPSPI2_SDI_OFFSET            (0x05e8)
#define IMXRT_INPUT_LPSPI2_SDO_OFFSET            (0x05ec)
#define IMXRT_INPUT_LPSPI3_PCS0_OFFSET           (0x05f0)
#define IMXRT_INPUT_LPSPI3_PCS1_OFFSET           (0x05f4)
#define IMXRT_INPUT_LPSPI3_PCS2_OFFSET           (0x05f8)
#define IMXRT_INPUT_LPSPI3_PCS3_OFFSET           (0x05fc)
#define IMXRT_INPUT_LPSPI3_SCK_OFFSET            (0x0600)
#define IMXRT_INPUT_LPSPI3_SDI_OFFSET            (0x0604)
#define IMXRT_INPUT_LPSPI3_SDO_OFFSET            (0x0608)
#define IMXRT_INPUT_LPSPI4_PCS0_OFFSET           (0x060c)
#define IMXRT_INPUT_LPSPI4_SCK_OFFSET            (0x0610)
#define IMXRT_INPUT_LPSPI4_SDI_OFFSET            (0x0614)
#define IMXRT_INPUT_LPSPI4_SDO_OFFSET            (0x0618)
#define IMXRT_INPUT_LPUART1_RX_OFFSET            (0x061c)
#define IMXRT_INPUT_LPUART1_TX_OFFSET            (0x0620)
#define IMXRT_INPUT_LPUART10_RX_OFFSET           (0x0624)
#define IMXRT_INPUT_LPUART10_TX_OFFSET           (0x0628)
#define IMXRT_INPUT_LPUART7_RX_OFFSET            (0x062c)
#define IMXRT_INPUT_LPUART7_TX_OFFSET            (0x0630)
#define IMXRT_INPUT_LPUART8_RX_OFFSET            (0x0634)
#define IMXRT_INPUT_LPUART8_TX_OFFSET            (0x0638)
#define IMXRT_INPUT_QTIMER1_TIMER0_OFFSET        (0x063c)
#define IMXRT_INPUT_QTIMER1_TIMER1_OFFSET        (0x0640)
#define IMXRT_INPUT_QTIMER1_TIMER2_OFFSET        (0x0644)
#define IMXRT_INPUT_QTIMER2_TIMER0_OFFSET        (0x0648)
#define IMXRT_INPUT_QTIMER2_TIMER1_OFFSET        (0x064c)
#define IMXRT_INPUT_QTIMER2_TIMER2_OFFSET        (0x0650)
#define IMXRT_INPUT_QTIMER3_TIMER0_OFFSET        (0x0654)
#define IMXRT_INPUT_QTIMER3_TIMER1_OFFSET        (0x0658)
#define IMXRT_INPUT_QTIMER3_TIMER2_OFFSET        (0x065c)
#define IMXRT_INPUT_QTIMER4_TIMER0_OFFSET        (0x0660)
#define IMXRT_INPUT_QTIMER4_TIMER1_OFFSET        (0x0664)
#define IMXRT_INPUT_QTIMER4_TIMER2_OFFSET        (0x0668)
#define IMXRT_INPUT_SAI1_MCLK_OFFSET             (0x066c)
#define IMXRT_INPUT_SAI1_RX_BCLK_OFFSET          (0x0670)
#define IMXRT_INPUT_SAI1_RX_DATA0_OFFSET         (0x0674)
#define IMXRT_INPUT_SAI1_RX_SYNC_OFFSET          (0x0678)
#define IMXRT_INPUT_SAI1_TX_BCLK_OFFSET          (0x067c)
#define IMXRT_INPUT_SAI1_TX_SYNC_OFFSET          (0x0680)
#define IMXRT_INPUT_EMVSIM1_SIO_OFFSET           (0x069c)
#define IMXRT_INPUT_EMVSIM1_SIMPD_OFFSET         (0x06a0)
#define IMXRT_INPUT_EMVSIM1_POWER_FAIL_OFFSET    (0x06a4)
#define IMXRT_INPUT_EMVSIM2_SIO_OFFSET           (0x06a8)
#define IMXRT_INPUT_EMVSIM2_SIMPD_OFFSET         (0x06ac)
#define IMXRT_INPUT_EMVSIM2_POWER_FAIL_OFFSET    (0x06b0)
#define IMXRT_INPUT_SPDIF_IN_OFFSET              (0x06b4)
#define IMXRT_INPUT_USB_OTG2_OC_OFFSET           (0x06b8)
#define IMXRT_INPUT_USB_OTG1_OC_OFFSET           (0x06bc)
#define IMXRT_INPUT_USBPHY1_USB_ID_OFFSET        (0x06c0)
#define IMXRT_INPUT_USBPHY2_USB_ID_OFFSET        (0x06c4)
#define IMXRT_INPUT_USDHC1_CD_B_OFFSET           (0x06c8)
#define IMXRT_INPUT_USDHC1_WP_OFFSET             (0x06cc)
#define IMXRT_INPUT_USDHC2_CD_B_OFFSET           (0x06d0)
#define IMXRT_INPUT_USDHC2_WP_OFFSET             (0x06d4)
#define IMXRT_INPUT_XBAR1_IN20_OFFSET            (0x06d8)
#define IMXRT_INPUT_XBAR1_IN21_OFFSET            (0x06dc)
#define IMXRT_INPUT_XBAR1_IN22_OFFSET            (0x06e0)
#define IMXRT_INPUT_XBAR1_IN23_OFFSET            (0x06e4)
#define IMXRT_INPUT_XBAR1_IN24_OFFSET            (0x06e8)
#define IMXRT_INPUT_XBAR1_IN25_OFFSET            (0x06ec)
#define IMXRT_INPUT_XBAR1_IN26_OFFSET            (0x06f0)
#define IMXRT_INPUT_XBAR1_IN27_OFFSET            (0x06f4)
#define IMXRT_INPUT_XBAR1_IN28_OFFSET            (0x06f8)
#define IMXRT_INPUT_XBAR1_IN29_OFFSET            (0x06fc)
#define IMXRT_INPUT_XBAR1_IN30_OFFSET            (0x0700)
#define IMXRT_INPUT_XBAR1_IN31_OFFSET            (0x0704)
#define IMXRT_INPUT_XBAR1_IN32_OFFSET            (0x0708)
#define IMXRT_INPUT_XBAR1_IN33_OFFSET            (0x070c)
#define IMXRT_INPUT_XBAR1_IN34_OFFSET            (0x0710)
#define IMXRT_INPUT_XBAR1_IN35_OFFSET            (0x0714)

/* IOMUXC_LPSR Select Input Daisy Chain Register Offsets
 * (from IMXRT_IOMUXCLPSR_BASE)
 */

#define IMXRT_INPUT_LPSR_INDEX2OFFSET(n)         (0x0080 + ((unsigned int)(n) << 2))
#define IMXRT_INPUT_LPSR_OFFSET2INDEX(o)         (((unsigned int)(o) - 0x0080) >> 2)

#define IMXRT_INPUT_LPSR_FLEXCAN3_RX_OFFSET      (0x0080)
#define IMXRT_INPUT_LPSR_LPI2C5_SCL_OFFSET       (0x0084)
#define IMXRT_INPUT_LPSR_LPI2C5_SDA_OFFSET       (0x0088)
#define IMXRT_INPUT_LPSR_LPI2C6_SCL_OFFSET       (0x008c)
#define IMXRT_INPUT_LPSR_LPI2C6_SDA_OFFSET       (0x0090)
#define IMXRT_INPUT_LPSR_LPSPI5_PCS0_OFFSET      (0x0094)
#define IMXRT_INPUT_LPSR_LPSPI5_SCK_OFFSET       (0x0098)
#define IMXRT_INPUT_LPSR_LPSPI5_SDI_OFFSET       (0x009c)
#define IMXRT_INPUT_LPSR_LPSPI5_SDO_OFFSET       (0x00a0)
#define IMXRT_INPUT_LPSR_LPUART11_RX_OFFSET      (0x00a4)
#define IMXRT_INPUT_LPSR_LPUART11_TX_OFFSET      (0x00a8)
#define IMXRT_INPUT_LPSR_LPUART12_RX_OFFSET      (0x00ac)
#define IMXRT_INPUT_LPSR_LPUART12_TX_OFFSET      (0x00b0)
#define IMXRT_INPUT_LPSR_PDM_BITSTREAM0_OFFSET   (0x00b4)
#define IMXRT_INPUT_LPSR_PDM_BITSTREAM1_OFFSET   (0x00b8)
#define IMXRT_INPUT_LPSR_PDM_BITSTREAM2_OFFSET   (0x00bc)
#define IMXRT_INPUT_LPSR_PDM_BITSTREAM3_OFFSET   (0x00c0)
#define IMXRT_INPUT_LPSR_NMI_GLUE_NMI_OFFSET     (0x00c4)
#define IMXRT_INPUT_LPSR_SAI4_MCLK_OFFSET        (0x00c8)
#define IMXRT_INPUT_LPSR_SAI4_RX_BCLK_OFFSET     (0x00cc)
#define IMXRT_INPUT_LPSR_SAI4_RX_DATA0_OFFSET    (0x00d0)
#define IMXRT_INPUT_LPSR_SAI4_RX_SYNC_OFFSET     (0x00d4)
#define IMXRT_INPUT_LPSR_SAI4_TX_BCLK_OFFSET     (0x00d8)
#define IMXRT_INPUT_LPSR_SAI4_TX_SYNC_OFFSET     (0x00dc)

/* Register Addresses *******************************************************/

/* IOMUXC GPR Register Addresses */

#define IMXRT_IOMUXC_GPR_GPR0                    (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR0_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR1                    (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR1_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR2                    (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR2_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR3                    (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR3_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR4                    (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR4_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR5                    (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR5_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR6                    (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR6_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR7                    (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR7_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR8                    (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR8_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR9                    (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR9_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR10                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR10_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR11                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR11_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR12                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR12_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR13                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR13_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR14                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR14_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR15                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR15_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR16                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR16_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR17                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR17_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR18                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR18_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR20                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR20_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR21                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR21_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR22                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR22_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR23                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR23_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR24                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR24_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR25                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR25_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR26                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR26_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR27                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR27_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR28                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR28_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR29                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR29_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR30                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR30_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR31                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR31_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR32                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR32_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR33                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR33_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR34                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR34_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR35                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR35_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR36                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR36_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR37                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR37_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR38                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR38_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR39                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR39_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR40                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR40_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR41                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR41_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR42                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR42_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR43                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR43_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR44                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR44_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR45                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR45_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR46                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR46_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR47                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR47_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR48                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR48_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR49                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR49_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR50                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR50_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR51                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR51_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR52                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR52_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR53                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR53_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR54                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR54_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR55                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR55_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR59                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR59_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR62                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR62_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR63                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR63_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR64                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR64_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR65                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR65_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR66                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR66_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR67                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR67_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR68                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR68_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR69                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR69_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR70                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR70_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR71                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR71_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR72                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR72_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR73                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR73_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR74                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR74_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR75                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR75_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR76                   (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR76_OFFSET)

/* IOMUXC LPSR GPR Register Addresses */

#define IMXRT_IOMUXC_LPSR_GPR_GPR26              (IMXRT_IOMUXCLPSRGPR_BASE + IMXRT_IOMUXC_LPSR_GPR_GPR26_OFFSET)
#define IMXRT_IOMUXC_LPSR_GPR_GPR33              (IMXRT_IOMUXCLPSRGPR_BASE + IMXRT_IOMUXC_LPSR_GPR_GPR33_OFFSET)
#define IMXRT_IOMUXC_LPSR_GPR_GPR34              (IMXRT_IOMUXCLPSRGPR_BASE + IMXRT_IOMUXC_LPSR_GPR_GPR34_OFFSET)
#define IMXRT_IOMUXC_LPSR_GPR_GPR35              (IMXRT_IOMUXCLPSRGPR_BASE + IMXRT_IOMUXC_LPSR_GPR_GPR35_OFFSET)
#define IMXRT_IOMUXC_LPSR_GPR_GPR36              (IMXRT_IOMUXCLPSRGPR_BASE + IMXRT_IOMUXC_LPSR_GPR_GPR36_OFFSET)
#define IMXRT_IOMUXC_LPSR_GPR_GPR37              (IMXRT_IOMUXCLPSRGPR_BASE + IMXRT_IOMUXC_LPSR_GPR_GPR37_OFFSET)
#define IMXRT_IOMUXC_LPSR_GPR_GPR38              (IMXRT_IOMUXCLPSRGPR_BASE + IMXRT_IOMUXC_LPSR_GPR_GPR38_OFFSET)
#define IMXRT_IOMUXC_LPSR_GPR_GPR39              (IMXRT_IOMUXCLPSRGPR_BASE + IMXRT_IOMUXC_LPSR_GPR_GPR39_OFFSET)
#define IMXRT_IOMUXC_LPSR_GPR_GPR40              (IMXRT_IOMUXCLPSRGPR_BASE + IMXRT_IOMUXC_LPSR_GPR_GPR40_OFFSET)
#define IMXRT_IOMUXC_LPSR_GPR_GPR41              (IMXRT_IOMUXCLPSRGPR_BASE + IMXRT_IOMUXC_LPSR_GPR_GPR41_OFFSET)

/* IOMUXC Pad Mux Register Addresses */

#define IMXRT_PADMUX_ADDRESS(n)                  (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_OFFSET(n))

#define IMXRT_PADMUX_GPIO_EMC_B1_00              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_00_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_01              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_01_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_02              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_02_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_03              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_03_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_04              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_04_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_05              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_05_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_06              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_06_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_07              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_07_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_08              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_08_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_09              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_09_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_10              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_10_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_11              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_11_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_12              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_12_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_13              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_13_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_14              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_14_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_15              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_15_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_16              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_16_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_17              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_17_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_18              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_18_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_19              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_19_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_20              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_20_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_21              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_21_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_22              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_22_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_23              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_23_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_24              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_24_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_25              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_25_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_26              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_26_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_27              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_27_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_28              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_28_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_29              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_29_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_30              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_30_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_31              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_31_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_32              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_32_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_33              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_33_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_34              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_34_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_35              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_35_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_36              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_36_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_37              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_37_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_38              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_38_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_39              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_39_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_40              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_40_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B1_41              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B1_41_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B2_00              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B2_00_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B2_01              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B2_01_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B2_02              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B2_02_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B2_03              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B2_03_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B2_04              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B2_04_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B2_05              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B2_05_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B2_06              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B2_06_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B2_07              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B2_07_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B2_08              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B2_08_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B2_09              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B2_09_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B2_10              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B2_10_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B2_11              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B2_11_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B2_12              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B2_12_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B2_13              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B2_13_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B2_14              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B2_14_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B2_15              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B2_15_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B2_16              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B2_16_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B2_17              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B2_17_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B2_18              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B2_18_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B2_19              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B2_19_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_B2_20              (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_B2_20_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_00                  (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_00_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_01                  (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_01_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_02                  (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_02_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_03                  (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_03_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_04                  (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_04_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_05                  (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_05_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_06                  (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_06_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_07                  (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_07_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_08                  (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_08_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_09                  (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_09_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_10                  (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_10_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_11                  (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_11_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_12                  (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_12_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_13                  (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_13_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_14                  (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_14_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_15                  (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_15_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_16                  (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_16_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_17                  (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_17_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_18                  (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_18_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_19                  (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_19_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_20                  (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_20_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_21                  (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_21_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_22                  (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_22_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_23                  (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_23_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_24                  (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_24_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_25                  (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_25_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_26                  (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_26_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_27                  (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_27_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_28                  (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_28_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_29                  (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_29_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_30                  (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_30_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_31                  (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_31_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_32                  (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_32_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_33                  (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_33_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_34                  (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_34_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_35                  (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_35_OFFSET)
#define IMXRT_PADMUX_GPIO_SD_B1_00               (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_SD_B1_00_OFFSET)
#define IMXRT_PADMUX_GPIO_SD_B1_01               (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_SD_B1_01_OFFSET)
#define IMXRT_PADMUX_GPIO_SD_B1_02               (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_SD_B1_02_OFFSET)
#define IMXRT_PADMUX_GPIO_SD_B1_03               (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_SD_B1_03_OFFSET)
#define IMXRT_PADMUX_GPIO_SD_B1_04               (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_SD_B1_04_OFFSET)
#define IMXRT_PADMUX_GPIO_SD_B1_05               (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_SD_B1_05_OFFSET)
#define IMXRT_PADMUX_GPIO_SD_B2_00               (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_SD_B2_00_OFFSET)
#define IMXRT_PADMUX_GPIO_SD_B2_01               (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_SD_B2_01_OFFSET)
#define IMXRT_PADMUX_GPIO_SD_B2_02               (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_SD_B2_02_OFFSET)
#define IMXRT_PADMUX_GPIO_SD_B2_03               (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_SD_B2_03_OFFSET)
#define IMXRT_PADMUX_GPIO_SD_B2_04               (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_SD_B2_04_OFFSET)
#define IMXRT_PADMUX_GPIO_SD_B2_05               (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_SD_B2_05_OFFSET)
#define IMXRT_PADMUX_GPIO_SD_B2_06               (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_SD_B2_06_OFFSET)
#define IMXRT_PADMUX_GPIO_SD_B2_07               (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_SD_B2_07_OFFSET)
#define IMXRT_PADMUX_GPIO_SD_B2_08               (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_SD_B2_08_OFFSET)
#define IMXRT_PADMUX_GPIO_SD_B2_09               (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_SD_B2_09_OFFSET)
#define IMXRT_PADMUX_GPIO_SD_B2_10               (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_SD_B2_10_OFFSET)
#define IMXRT_PADMUX_GPIO_SD_B2_11               (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_SD_B2_11_OFFSET)
#define IMXRT_PADMUX_GPIO_DISP_B1_00             (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_DISP_B1_00_OFFSET)
#define IMXRT_PADMUX_GPIO_DISP_B1_01             (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_DISP_B1_01_OFFSET)
#define IMXRT_PADMUX_GPIO_DISP_B1_02             (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_DISP_B1_02_OFFSET)
#define IMXRT_PADMUX_GPIO_DISP_B1_03             (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_DISP_B1_03_OFFSET)
#define IMXRT_PADMUX_GPIO_DISP_B1_04             (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_DISP_B1_04_OFFSET)
#define IMXRT_PADMUX_GPIO_DISP_B1_05             (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_DISP_B1_05_OFFSET)
#define IMXRT_PADMUX_GPIO_DISP_B1_06             (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_DISP_B1_06_OFFSET)
#define IMXRT_PADMUX_GPIO_DISP_B1_07             (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_DISP_B1_07_OFFSET)
#define IMXRT_PADMUX_GPIO_DISP_B1_08             (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_DISP_B1_08_OFFSET)
#define IMXRT_PADMUX_GPIO_DISP_B1_09             (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_DISP_B1_09_OFFSET)
#define IMXRT_PADMUX_GPIO_DISP_B1_10             (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_DISP_B1_10_OFFSET)
#define IMXRT_PADMUX_GPIO_DISP_B1_11             (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_DISP_B1_11_OFFSET)
#define IMXRT_PADMUX_GPIO_DISP_B2_00             (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_DISP_B2_00_OFFSET)
#define IMXRT_PADMUX_GPIO_DISP_B2_01             (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_DISP_B2_01_OFFSET)
#define IMXRT_PADMUX_GPIO_DISP_B2_02             (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_DISP_B2_02_OFFSET)
#define IMXRT_PADMUX_GPIO_DISP_B2_03             (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_DISP_B2_03_OFFSET)
#define IMXRT_PADMUX_GPIO_DISP_B2_04             (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_DISP_B2_04_OFFSET)
#define IMXRT_PADMUX_GPIO_DISP_B2_05             (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_DISP_B2_05_OFFSET)
#define IMXRT_PADMUX_GPIO_DISP_B2_06             (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_DISP_B2_06_OFFSET)
#define IMXRT_PADMUX_GPIO_DISP_B2_07             (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_DISP_B2_07_OFFSET)
#define IMXRT_PADMUX_GPIO_DISP_B2_08             (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_DISP_B2_08_OFFSET)
#define IMXRT_PADMUX_GPIO_DISP_B2_09             (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_DISP_B2_09_OFFSET)
#define IMXRT_PADMUX_GPIO_DISP_B2_10             (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_DISP_B2_10_OFFSET)
#define IMXRT_PADMUX_GPIO_DISP_B2_11             (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_DISP_B2_11_OFFSET)
#define IMXRT_PADMUX_GPIO_DISP_B2_12             (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_DISP_B2_12_OFFSET)
#define IMXRT_PADMUX_GPIO_DISP_B2_13             (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_DISP_B2_13_OFFSET)
#define IMXRT_PADMUX_GPIO_DISP_B2_14             (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_DISP_B2_14_OFFSET)
#define IMXRT_PADMUX_GPIO_DISP_B2_15             (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_DISP_B2_15_OFFSET)

/* IOMUXC_SNVS Pad Mux Register Addresses */

#define IMXRT_PADMUX_ADDRESS_SNVS(n)             (IMXRT_IOMUXCSNVS_BASE + IMXRT_PADMUX_OFFSET_SNVS(n))

#define IMXRT_PADMUX_WAKEUP                      (IMXRT_IOMUXCSNVS_BASE + IMXRT_PADMUX_WAKEUP_OFFSET)
#define IMXRT_PADMUX_PMIC_ON_REQ                 (IMXRT_IOMUXCSNVS_BASE + IMXRT_PADMUX_PMIC_ON_REQ_OFFSET)
#define IMXRT_PADMUX_PMIC_STBY_REQ               (IMXRT_IOMUXCSNVS_BASE + IMXRT_PADMUX_PMIC_STBY_REQ_OFFSET)
#define IMXRT_PADMUX_GPIO_SNVS_00                (IMXRT_IOMUXCSNVS_BASE + IMXRT_PADMUX_GPIO_SNVS_00_OFFSET)
#define IMXRT_PADMUX_GPIO_SNVS_01                (IMXRT_IOMUXCSNVS_BASE + IMXRT_PADMUX_GPIO_SNVS_01_OFFSET)
#define IMXRT_PADMUX_GPIO_SNVS_02                (IMXRT_IOMUXCSNVS_BASE + IMXRT_PADMUX_GPIO_SNVS_02_OFFSET)
#define IMXRT_PADMUX_GPIO_SNVS_03                (IMXRT_IOMUXCSNVS_BASE + IMXRT_PADMUX_GPIO_SNVS_03_OFFSET)
#define IMXRT_PADMUX_GPIO_SNVS_04                (IMXRT_IOMUXCSNVS_BASE + IMXRT_PADMUX_GPIO_SNVS_04_OFFSET)
#define IMXRT_PADMUX_GPIO_SNVS_05                (IMXRT_IOMUXCSNVS_BASE + IMXRT_PADMUX_GPIO_SNVS_05_OFFSET)
#define IMXRT_PADMUX_GPIO_SNVS_06                (IMXRT_IOMUXCSNVS_BASE + IMXRT_PADMUX_GPIO_SNVS_06_OFFSET)
#define IMXRT_PADMUX_GPIO_SNVS_07                (IMXRT_IOMUXCSNVS_BASE + IMXRT_PADMUX_GPIO_SNVS_07_OFFSET)
#define IMXRT_PADMUX_GPIO_SNVS_08                (IMXRT_IOMUXCSNVS_BASE + IMXRT_PADMUX_GPIO_SNVS_08_OFFSET)
#define IMXRT_PADMUX_GPIO_SNVS_09                (IMXRT_IOMUXCSNVS_BASE + IMXRT_PADMUX_GPIO_SNVS_09_OFFSET)

/* IOMUXC_LPSR Pad Mux Register Addresses */

#define IMXRT_PADMUX_ADDRESS_LPSR(n)             (IMXRT_IOMUXCLPSR_BASE + IMXRT_PADMUX_OFFSET_LPSR(n))

#define IMXRT_PADMUX_GPIO_LPSR_00                (IMXRT_IOMUXCLPSR_BASE + IMXRT_PADMUX_GPIO_LPSR_00_OFFSET)
#define IMXRT_PADMUX_GPIO_LPSR_01                (IMXRT_IOMUXCLPSR_BASE + IMXRT_PADMUX_GPIO_LPSR_01_OFFSET)
#define IMXRT_PADMUX_GPIO_LPSR_02                (IMXRT_IOMUXCLPSR_BASE + IMXRT_PADMUX_GPIO_LPSR_02_OFFSET)
#define IMXRT_PADMUX_GPIO_LPSR_03                (IMXRT_IOMUXCLPSR_BASE + IMXRT_PADMUX_GPIO_LPSR_03_OFFSET)
#define IMXRT_PADMUX_GPIO_LPSR_04                (IMXRT_IOMUXCLPSR_BASE + IMXRT_PADMUX_GPIO_LPSR_04_OFFSET)
#define IMXRT_PADMUX_GPIO_LPSR_05                (IMXRT_IOMUXCLPSR_BASE + IMXRT_PADMUX_GPIO_LPSR_05_OFFSET)
#define IMXRT_PADMUX_GPIO_LPSR_06                (IMXRT_IOMUXCLPSR_BASE + IMXRT_PADMUX_GPIO_LPSR_06_OFFSET)
#define IMXRT_PADMUX_GPIO_LPSR_07                (IMXRT_IOMUXCLPSR_BASE + IMXRT_PADMUX_GPIO_LPSR_07_OFFSET)
#define IMXRT_PADMUX_GPIO_LPSR_08                (IMXRT_IOMUXCLPSR_BASE + IMXRT_PADMUX_GPIO_LPSR_08_OFFSET)
#define IMXRT_PADMUX_GPIO_LPSR_09                (IMXRT_IOMUXCLPSR_BASE + IMXRT_PADMUX_GPIO_LPSR_09_OFFSET)
#define IMXRT_PADMUX_GPIO_LPSR_10                (IMXRT_IOMUXCLPSR_BASE + IMXRT_PADMUX_GPIO_LPSR_10_OFFSET)
#define IMXRT_PADMUX_GPIO_LPSR_11                (IMXRT_IOMUXCLPSR_BASE + IMXRT_PADMUX_GPIO_LPSR_11_OFFSET)
#define IMXRT_PADMUX_GPIO_LPSR_12                (IMXRT_IOMUXCLPSR_BASE + IMXRT_PADMUX_GPIO_LPSR_12_OFFSET)
#define IMXRT_PADMUX_GPIO_LPSR_13                (IMXRT_IOMUXCLPSR_BASE + IMXRT_PADMUX_GPIO_LPSR_13_OFFSET)
#define IMXRT_PADMUX_GPIO_LPSR_14                (IMXRT_IOMUXCLPSR_BASE + IMXRT_PADMUX_GPIO_LPSR_14_OFFSET)
#define IMXRT_PADMUX_GPIO_LPSR_15                (IMXRT_IOMUXCLPSR_BASE + IMXRT_PADMUX_GPIO_LPSR_15_OFFSET)

/* IOMUXC Pad Control Register Addresses */

#define IMXRT_PADCTL_ADDRESS(n)                  (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_OFFSET(n))

#define IMXRT_PADCTL_GPIO_EMC_B1_00              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_00_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_01              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_01_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_02              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_02_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_03              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_03_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_04              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_04_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_05              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_05_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_06              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_06_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_07              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_07_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_08              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_08_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_09              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_09_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_10              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_10_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_11              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_11_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_12              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_12_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_13              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_13_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_14              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_14_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_15              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_15_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_16              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_16_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_17              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_17_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_18              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_18_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_19              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_19_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_20              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_20_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_21              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_21_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_22              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_22_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_23              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_23_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_24              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_24_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_25              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_25_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_26              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_26_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_27              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_27_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_28              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_28_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_29              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_29_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_30              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_30_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_31              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_31_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_32              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_32_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_33              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_33_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_34              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_34_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_35              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_35_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_36              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_36_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_37              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_37_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_38              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_38_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_39              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_39_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_40              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_40_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B1_41              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B1_41_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B2_00              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B2_00_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B2_01              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B2_01_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B2_02              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B2_02_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B2_03              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B2_03_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B2_04              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B2_04_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B2_05              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B2_05_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B2_06              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B2_06_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B2_07              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B2_07_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B2_08              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B2_08_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B2_09              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B2_09_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B2_10              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B2_10_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B2_11              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B2_11_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B2_12              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B2_12_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B2_13              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B2_13_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B2_14              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B2_14_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B2_15              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B2_15_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B2_16              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B2_16_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B2_17              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B2_17_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B2_18              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B2_18_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B2_19              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B2_19_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_B2_20              (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_B2_20_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_00                  (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_00_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_01                  (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_01_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_02                  (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_02_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_03                  (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_03_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_04                  (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_04_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_05                  (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_05_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_06                  (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_06_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_07                  (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_07_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_08                  (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_08_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_09                  (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_09_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_10                  (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_10_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_11                  (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_11_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_12                  (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_12_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_13                  (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_13_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_14                  (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_14_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_15                  (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_15_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_16                  (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_16_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_17                  (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_17_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_18                  (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_18_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_19                  (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_19_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_20                  (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_20_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_21                  (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_21_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_22                  (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_22_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_23                  (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_23_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_24                  (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_24_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_25                  (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_25_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_26                  (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_26_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_27                  (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_27_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_28                  (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_28_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_29                  (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_29_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_30                  (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_30_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_31                  (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_31_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_32                  (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_32_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_33                  (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_33_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_34                  (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_34_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_35                  (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_35_OFFSET)
#define IMXRT_PADCTL_GPIO_SD_B1_00               (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_SD_B1_00_OFFSET)
#define IMXRT_PADCTL_GPIO_SD_B1_01               (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_SD_B1_01_OFFSET)
#define IMXRT_PADCTL_GPIO_SD_B1_02               (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_SD_B1_02_OFFSET)
#define IMXRT_PADCTL_GPIO_SD_B1_03               (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_SD_B1_03_OFFSET)
#define IMXRT_PADCTL_GPIO_SD_B1_04               (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_SD_B1_04_OFFSET)
#define IMXRT_PADCTL_GPIO_SD_B1_05               (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_SD_B1_05_OFFSET)
#define IMXRT_PADCTL_GPIO_SD_B2_00               (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_SD_B2_00_OFFSET)
#define IMXRT_PADCTL_GPIO_SD_B2_01               (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_SD_B2_01_OFFSET)
#define IMXRT_PADCTL_GPIO_SD_B2_02               (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_SD_B2_02_OFFSET)
#define IMXRT_PADCTL_GPIO_SD_B2_03               (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_SD_B2_03_OFFSET)
#define IMXRT_PADCTL_GPIO_SD_B2_04               (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_SD_B2_04_OFFSET)
#define IMXRT_PADCTL_GPIO_SD_B2_05               (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_SD_B2_05_OFFSET)
#define IMXRT_PADCTL_GPIO_SD_B2_06               (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_SD_B2_06_OFFSET)
#define IMXRT_PADCTL_GPIO_SD_B2_07               (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_SD_B2_07_OFFSET)
#define IMXRT_PADCTL_GPIO_SD_B2_08               (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_SD_B2_08_OFFSET)
#define IMXRT_PADCTL_GPIO_SD_B2_09               (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_SD_B2_09_OFFSET)
#define IMXRT_PADCTL_GPIO_SD_B2_10               (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_SD_B2_10_OFFSET)
#define IMXRT_PADCTL_GPIO_SD_B2_11               (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_SD_B2_11_OFFSET)
#define IMXRT_PADCTL_GPIO_DISP_B1_00             (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_DISP_B1_00_OFFSET)
#define IMXRT_PADCTL_GPIO_DISP_B1_01             (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_DISP_B1_01_OFFSET)
#define IMXRT_PADCTL_GPIO_DISP_B1_02             (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_DISP_B1_02_OFFSET)
#define IMXRT_PADCTL_GPIO_DISP_B1_03             (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_DISP_B1_03_OFFSET)
#define IMXRT_PADCTL_GPIO_DISP_B1_04             (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_DISP_B1_04_OFFSET)
#define IMXRT_PADCTL_GPIO_DISP_B1_05             (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_DISP_B1_05_OFFSET)
#define IMXRT_PADCTL_GPIO_DISP_B1_06             (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_DISP_B1_06_OFFSET)
#define IMXRT_PADCTL_GPIO_DISP_B1_07             (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_DISP_B1_07_OFFSET)
#define IMXRT_PADCTL_GPIO_DISP_B1_08             (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_DISP_B1_08_OFFSET)
#define IMXRT_PADCTL_GPIO_DISP_B1_09             (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_DISP_B1_09_OFFSET)
#define IMXRT_PADCTL_GPIO_DISP_B1_10             (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_DISP_B1_10_OFFSET)
#define IMXRT_PADCTL_GPIO_DISP_B1_11             (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_DISP_B1_11_OFFSET)
#define IMXRT_PADCTL_GPIO_DISP_B2_00             (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_DISP_B2_00_OFFSET)
#define IMXRT_PADCTL_GPIO_DISP_B2_01             (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_DISP_B2_01_OFFSET)
#define IMXRT_PADCTL_GPIO_DISP_B2_02             (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_DISP_B2_02_OFFSET)
#define IMXRT_PADCTL_GPIO_DISP_B2_03             (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_DISP_B2_03_OFFSET)
#define IMXRT_PADCTL_GPIO_DISP_B2_04             (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_DISP_B2_04_OFFSET)
#define IMXRT_PADCTL_GPIO_DISP_B2_05             (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_DISP_B2_05_OFFSET)
#define IMXRT_PADCTL_GPIO_DISP_B2_06             (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_DISP_B2_06_OFFSET)
#define IMXRT_PADCTL_GPIO_DISP_B2_07             (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_DISP_B2_07_OFFSET)
#define IMXRT_PADCTL_GPIO_DISP_B2_08             (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_DISP_B2_08_OFFSET)
#define IMXRT_PADCTL_GPIO_DISP_B2_09             (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_DISP_B2_09_OFFSET)
#define IMXRT_PADCTL_GPIO_DISP_B2_10             (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_DISP_B2_10_OFFSET)
#define IMXRT_PADCTL_GPIO_DISP_B2_11             (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_DISP_B2_11_OFFSET)
#define IMXRT_PADCTL_GPIO_DISP_B2_12             (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_DISP_B2_12_OFFSET)
#define IMXRT_PADCTL_GPIO_DISP_B2_13             (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_DISP_B2_13_OFFSET)
#define IMXRT_PADCTL_GPIO_DISP_B2_14             (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_DISP_B2_14_OFFSET)
#define IMXRT_PADCTL_GPIO_DISP_B2_15             (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_DISP_B2_15_OFFSET)

/* IOMUXC_SNVS Pad Control Register Addresses */

#define IMXRT_PADCTL_ADDRESS_SNVS(n)             (IMXRT_IOMUXCSNVS_BASE + IMXRT_PADCTL_OFFSET_SNVS(n))

#define IMXRT_PADCTL_TEST_MODE                   (IMXRT_IOMUXCSNVS_BASE + IMXRT_PADCTL_TEST_MODE_OFFSET)
#define IMXRT_PADCTL_POR_B                       (IMXRT_IOMUXCSNVS_BASE + IMXRT_PADCTL_POR_B_OFFSET)
#define IMXRT_PADCTL_ONOFF                       (IMXRT_IOMUXCSNVS_BASE + IMXRT_PADCTL_ONOFF_OFFSET)
#define IMXRT_PADCTL_WAKEUP                      (IMXRT_IOMUXCSNVS_BASE + IMXRT_PADCTL_WAKEUP_OFFSET)
#define IMXRT_PADCTL_PMIC_ON_REQ                 (IMXRT_IOMUXCSNVS_BASE + IMXRT_PADCTL_PMIC_ON_REQ_OFFSET)
#define IMXRT_PADCTL_PMIC_STBY_REQ               (IMXRT_IOMUXCSNVS_BASE + IMXRT_PADCTL_PMIC_STBY_REQ_OFFSET)
#define IMXRT_PADCTL_GPIO_SNVS_00                (IMXRT_IOMUXCSNVS_BASE + IMXRT_PADCTL_GPIO_SNVS_00_OFFSET)
#define IMXRT_PADCTL_GPIO_SNVS_01                (IMXRT_IOMUXCSNVS_BASE + IMXRT_PADCTL_GPIO_SNVS_01_OFFSET)
#define IMXRT_PADCTL_GPIO_SNVS_02                (IMXRT_IOMUXCSNVS_BASE + IMXRT_PADCTL_GPIO_SNVS_02_OFFSET)
#define IMXRT_PADCTL_GPIO_SNVS_03                (IMXRT_IOMUXCSNVS_BASE + IMXRT_PADCTL_GPIO_SNVS_03_OFFSET)
#define IMXRT_PADCTL_GPIO_SNVS_04                (IMXRT_IOMUXCSNVS_BASE + IMXRT_PADCTL_GPIO_SNVS_04_OFFSET)
#define IMXRT_PADCTL_GPIO_SNVS_05                (IMXRT_IOMUXCSNVS_BASE + IMXRT_PADCTL_GPIO_SNVS_05_OFFSET)
#define IMXRT_PADCTL_GPIO_SNVS_06                (IMXRT_IOMUXCSNVS_BASE + IMXRT_PADCTL_GPIO_SNVS_06_OFFSET)
#define IMXRT_PADCTL_GPIO_SNVS_07                (IMXRT_IOMUXCSNVS_BASE + IMXRT_PADCTL_GPIO_SNVS_07_OFFSET)
#define IMXRT_PADCTL_GPIO_SNVS_08                (IMXRT_IOMUXCSNVS_BASE + IMXRT_PADCTL_GPIO_SNVS_08_OFFSET)
#define IMXRT_PADCTL_GPIO_SNVS_09                (IMXRT_IOMUXCSNVS_BASE + IMXRT_PADCTL_GPIO_SNVS_09_OFFSET)

/* IOMUXC_LPSR Pad Control Register Addresses */

#define IMXRT_PADCTL_ADDRESS_LPSR(n)             (IMXRT_IOMUXCLPSR_BASE + IMXRT_PADCTL_OFFSET_LPSR(n))

#define IMXRT_PADCTL_GPIO_LPSR_00                (IMXRT_IOMUXCLPSR_BASE + IMXRT_PADCTL_GPIO_LPSR_00_OFFSET)
#define IMXRT_PADCTL_GPIO_LPSR_01                (IMXRT_IOMUXCLPSR_BASE + IMXRT_PADCTL_GPIO_LPSR_01_OFFSET)
#define IMXRT_PADCTL_GPIO_LPSR_02                (IMXRT_IOMUXCLPSR_BASE + IMXRT_PADCTL_GPIO_LPSR_02_OFFSET)
#define IMXRT_PADCTL_GPIO_LPSR_03                (IMXRT_IOMUXCLPSR_BASE + IMXRT_PADCTL_GPIO_LPSR_03_OFFSET)
#define IMXRT_PADCTL_GPIO_LPSR_04                (IMXRT_IOMUXCLPSR_BASE + IMXRT_PADCTL_GPIO_LPSR_04_OFFSET)
#define IMXRT_PADCTL_GPIO_LPSR_05                (IMXRT_IOMUXCLPSR_BASE + IMXRT_PADCTL_GPIO_LPSR_05_OFFSET)
#define IMXRT_PADCTL_GPIO_LPSR_06                (IMXRT_IOMUXCLPSR_BASE + IMXRT_PADCTL_GPIO_LPSR_06_OFFSET)
#define IMXRT_PADCTL_GPIO_LPSR_07                (IMXRT_IOMUXCLPSR_BASE + IMXRT_PADCTL_GPIO_LPSR_07_OFFSET)
#define IMXRT_PADCTL_GPIO_LPSR_08                (IMXRT_IOMUXCLPSR_BASE + IMXRT_PADCTL_GPIO_LPSR_08_OFFSET)
#define IMXRT_PADCTL_GPIO_LPSR_09                (IMXRT_IOMUXCLPSR_BASE + IMXRT_PADCTL_GPIO_LPSR_09_OFFSET)
#define IMXRT_PADCTL_GPIO_LPSR_10                (IMXRT_IOMUXCLPSR_BASE + IMXRT_PADCTL_GPIO_LPSR_10_OFFSET)
#define IMXRT_PADCTL_GPIO_LPSR_11                (IMXRT_IOMUXCLPSR_BASE + IMXRT_PADCTL_GPIO_LPSR_11_OFFSET)
#define IMXRT_PADCTL_GPIO_LPSR_12                (IMXRT_IOMUXCLPSR_BASE + IMXRT_PADCTL_GPIO_LPSR_12_OFFSET)
#define IMXRT_PADCTL_GPIO_LPSR_13                (IMXRT_IOMUXCLPSR_BASE + IMXRT_PADCTL_GPIO_LPSR_13_OFFSET)
#define IMXRT_PADCTL_GPIO_LPSR_14                (IMXRT_IOMUXCLPSR_BASE + IMXRT_PADCTL_GPIO_LPSR_14_OFFSET)
#define IMXRT_PADCTL_GPIO_LPSR_15                (IMXRT_IOMUXCLPSR_BASE + IMXRT_PADCTL_GPIO_LPSR_15_OFFSET)

/* IOMUXC Select Input Daisy Chain Registers */

#define IMXRT_INPUT_FLEXCAN1_RX                  (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXCAN1_RX_OFFSET)
#define IMXRT_INPUT_FLEXCAN2_RX                  (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXCAN2_RX_OFFSET)
#define IMXRT_INPUT_CCM_ENET_QOS_REFCLK          (IMXRT_IOMUXC_BASE + IMXRT_INPUT_CCM_ENET_QOS_REFCLK_OFFSET)
#define IMXRT_INPUT_CCM_ENET_QOS_TXCLK           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_CCM_ENET_QOS_TXCLK_OFFSET)
#define IMXRT_INPUT_ENET_IPG_CLK_RMII            (IMXRT_IOMUXC_BASE + IMXRT_INPUT_ENET_IPG_CLK_RMII_OFFSET)
#define IMXRT_INPUT_ENET_MDIO                    (IMXRT_IOMUXC_BASE + IMXRT_INPUT_ENET_MDIO_OFFSET)
#define IMXRT_INPUT_ENET_RXDATA0                 (IMXRT_IOMUXC_BASE + IMXRT_INPUT_ENET_RXDATA0_OFFSET)
#define IMXRT_INPUT_ENET_RXDATA1                 (IMXRT_IOMUXC_BASE + IMXRT_INPUT_ENET_RXDATA1_OFFSET)
#define IMXRT_INPUT_ENET_RXEN                    (IMXRT_IOMUXC_BASE + IMXRT_INPUT_ENET_RXEN_OFFSET)
#define IMXRT_INPUT_ENET_RXERR                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_ENET_RXERR_OFFSET)
#define IMXRT_INPUT_ENET_TXCLK                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_ENET_TXCLK_OFFSET)
#define IMXRT_INPUT_ENET_1G_IPG_CLK_RMII         (IMXRT_IOMUXC_BASE + IMXRT_INPUT_ENET_1G_IPG_CLK_RMII_OFFSET)
#define IMXRT_INPUT_ENET_1G_MDIO                 (IMXRT_IOMUXC_BASE + IMXRT_INPUT_ENET_1G_MDIO_OFFSET)
#define IMXRT_INPUT_ENET_1G_RXCLK                (IMXRT_IOMUXC_BASE + IMXRT_INPUT_ENET_1G_RXCLK_OFFSET)
#define IMXRT_INPUT_ENET_1G_RXDATA0              (IMXRT_IOMUXC_BASE + IMXRT_INPUT_ENET_1G_RXDATA0_OFFSET)
#define IMXRT_INPUT_ENET_1G_RXDATA1              (IMXRT_IOMUXC_BASE + IMXRT_INPUT_ENET_1G_RXDATA1_OFFSET)
#define IMXRT_INPUT_ENET_1G_RXDATA2              (IMXRT_IOMUXC_BASE + IMXRT_INPUT_ENET_1G_RXDATA2_OFFSET)
#define IMXRT_INPUT_ENET_1G_RXDATA3              (IMXRT_IOMUXC_BASE + IMXRT_INPUT_ENET_1G_RXDATA3_OFFSET)
#define IMXRT_INPUT_ENET_1G_RXEN                 (IMXRT_IOMUXC_BASE + IMXRT_INPUT_ENET_1G_RXEN_OFFSET)
#define IMXRT_INPUT_ENET_1G_RXERR                (IMXRT_IOMUXC_BASE + IMXRT_INPUT_ENET_1G_RXERR_OFFSET)
#define IMXRT_INPUT_ENET_1G_TXCLK                (IMXRT_IOMUXC_BASE + IMXRT_INPUT_ENET_1G_TXCLK_OFFSET)
#define IMXRT_INPUT_ENET_QOS_MDIO                (IMXRT_IOMUXC_BASE + IMXRT_INPUT_ENET_QOS_MDIO_OFFSET)
#define IMXRT_INPUT_ENET_QOS_RXDATA0             (IMXRT_IOMUXC_BASE + IMXRT_INPUT_ENET_QOS_RXDATA0_OFFSET)
#define IMXRT_INPUT_ENET_QOS_RXDATA1             (IMXRT_IOMUXC_BASE + IMXRT_INPUT_ENET_QOS_RXDATA1_OFFSET)
#define IMXRT_INPUT_ENET_QOS_RXDV                (IMXRT_IOMUXC_BASE + IMXRT_INPUT_ENET_QOS_RXDV_OFFSET)
#define IMXRT_INPUT_ENET_QOS_RXERR               (IMXRT_IOMUXC_BASE + IMXRT_INPUT_ENET_QOS_RXERR_OFFSET)
#define IMXRT_INPUT_FLEXPWM1_PWMA0               (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXPWM1_PWMA0_OFFSET)
#define IMXRT_INPUT_FLEXPWM1_PWMA1               (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXPWM1_PWMA1_OFFSET)
#define IMXRT_INPUT_FLEXPWM1_PWMA2               (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXPWM1_PWMA2_OFFSET)
#define IMXRT_INPUT_FLEXPWM1_PWMB0               (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXPWM1_PWMB0_OFFSET)
#define IMXRT_INPUT_FLEXPWM1_PWMB1               (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXPWM1_PWMB1_OFFSET)
#define IMXRT_INPUT_FLEXPWM1_PWMB2               (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXPWM1_PWMB2_OFFSET)
#define IMXRT_INPUT_FLEXPWM2_PWMA0               (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXPWM2_PWMA0_OFFSET)
#define IMXRT_INPUT_FLEXPWM2_PWMA1               (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXPWM2_PWMA1_OFFSET)
#define IMXRT_INPUT_FLEXPWM2_PWMA2               (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXPWM2_PWMA2_OFFSET)
#define IMXRT_INPUT_FLEXPWM2_PWMB0               (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXPWM2_PWMB0_OFFSET)
#define IMXRT_INPUT_FLEXPWM2_PWMB1               (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXPWM2_PWMB1_OFFSET)
#define IMXRT_INPUT_FLEXPWM2_PWMB2               (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXPWM2_PWMB2_OFFSET)
#define IMXRT_INPUT_FLEXPWM3_PWMA0               (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXPWM3_PWMA0_OFFSET)
#define IMXRT_INPUT_FLEXPWM3_PWMA1               (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXPWM3_PWMA1_OFFSET)
#define IMXRT_INPUT_FLEXPWM3_PWMA2               (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXPWM3_PWMA2_OFFSET)
#define IMXRT_INPUT_FLEXPWM3_PWMA3               (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXPWM3_PWMA3_OFFSET)
#define IMXRT_INPUT_FLEXPWM3_PWMB0               (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXPWM3_PWMB0_OFFSET)
#define IMXRT_INPUT_FLEXPWM3_PWMB1               (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXPWM3_PWMB1_OFFSET)
#define IMXRT_INPUT_FLEXPWM3_PWMB2               (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXPWM3_PWMB2_OFFSET)
#define IMXRT_INPUT_FLEXPWM3_PWMB3               (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXPWM3_PWMB3_OFFSET)
#define IMXRT_INPUT_FLEXSPI1A_DQS                (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXSPI1A_DQS_OFFSET)
#define IMXRT_INPUT_FLEXSPI1A_DATA0              (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXSPI1A_DATA0_OFFSET)
#define IMXRT_INPUT_FLEXSPI1A_DATA1              (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXSPI1A_DATA1_OFFSET)
#define IMXRT_INPUT_FLEXSPI1A_DATA2              (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXSPI1A_DATA2_OFFSET)
#define IMXRT_INPUT_FLEXSPI1A_DATA3              (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXSPI1A_DATA3_OFFSET)
#define IMXRT_INPUT_FLEXSPI1B_DATA0              (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXSPI1B_DATA0_OFFSET)
#define IMXRT_INPUT_FLEXSPI1B_DATA1              (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXSPI1B_DATA1_OFFSET)
#define IMXRT_INPUT_FLEXSPI1B_DATA2              (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXSPI1B_DATA2_OFFSET)
#define IMXRT_INPUT_FLEXSPI1B_DATA3              (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXSPI1B_DATA3_OFFSET)
#define IMXRT_INPUT_FLEXSPI1A_SCK                (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXSPI1A_SCK_OFFSET)
#define IMXRT_INPUT_FLEXSPI1B_SCK                (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXSPI1B_SCK_OFFSET)
#define IMXRT_INPUT_FLEXSPI2A_DATA0              (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXSPI2A_DATA0_OFFSET)
#define IMXRT_INPUT_FLEXSPI2A_DATA1              (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXSPI2A_DATA1_OFFSET)
#define IMXRT_INPUT_FLEXSPI2A_DATA2              (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXSPI2A_DATA2_OFFSET)
#define IMXRT_INPUT_FLEXSPI2A_DATA3              (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXSPI2A_DATA3_OFFSET)
#define IMXRT_INPUT_FLEXSPI2A_SCK                (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXSPI2A_SCK_OFFSET)
#define IMXRT_INPUT_GPT3_CAPIN1                  (IMXRT_IOMUXC_BASE + IMXRT_INPUT_GPT3_CAPIN1_OFFSET)
#define IMXRT_INPUT_GPT3_CAPIN2                  (IMXRT_IOMUXC_BASE + IMXRT_INPUT_GPT3_CAPIN2_OFFSET)
#define IMXRT_INPUT_GPT3_CLKIN                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_GPT3_CLKIN_OFFSET)
#define IMXRT_INPUT_KPP_COL6                     (IMXRT_IOMUXC_BASE + IMXRT_INPUT_KPP_COL6_OFFSET)
#define IMXRT_INPUT_KPP_COL7                     (IMXRT_IOMUXC_BASE + IMXRT_INPUT_KPP_COL7_OFFSET)
#define IMXRT_INPUT_KPP_ROW6                     (IMXRT_IOMUXC_BASE + IMXRT_INPUT_KPP_ROW6_OFFSET)
#define IMXRT_INPUT_KPP_ROW7                     (IMXRT_IOMUXC_BASE + IMXRT_INPUT_KPP_ROW7_OFFSET)
#define IMXRT_INPUT_LPI2C1_SCL                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPI2C1_SCL_OFFSET)
#define IMXRT_INPUT_LPI2C1_SDA                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPI2C1_SDA_OFFSET)
#define IMXRT_INPUT_LPI2C2_SCL                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPI2C2_SCL_OFFSET)
#define IMXRT_INPUT_LPI2C2_SDA                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPI2C2_SDA_OFFSET)
#define IMXRT_INPUT_LPI2C3_SCL                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPI2C3_SCL_OFFSET)
#define IMXRT_INPUT_LPI2C3_SDA                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPI2C3_SDA_OFFSET)
#define IMXRT_INPUT_LPI2C4_SCL                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPI2C4_SCL_OFFSET)
#define IMXRT_INPUT_LPI2C4_SDA                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPI2C4_SDA_OFFSET)
#define IMXRT_INPUT_LPSPI1_PCS0                  (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPSPI1_PCS0_OFFSET)
#define IMXRT_INPUT_LPSPI1_SCK                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPSPI1_SCK_OFFSET)
#define IMXRT_INPUT_LPSPI1_SDI                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPSPI1_SDI_OFFSET)
#define IMXRT_INPUT_LPSPI1_SDO                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPSPI1_SDO_OFFSET)
#define IMXRT_INPUT_LPSPI2_PCS0                  (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPSPI2_PCS0_OFFSET)
#define IMXRT_INPUT_LPSPI2_PCS1                  (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPSPI2_PCS1_OFFSET)
#define IMXRT_INPUT_LPSPI2_SCK                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPSPI2_SCK_OFFSET)
#define IMXRT_INPUT_LPSPI2_SDI                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPSPI2_SDI_OFFSET)
#define IMXRT_INPUT_LPSPI2_SDO                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPSPI2_SDO_OFFSET)
#define IMXRT_INPUT_LPSPI3_PCS0                  (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPSPI3_PCS0_OFFSET)
#define IMXRT_INPUT_LPSPI3_PCS1                  (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPSPI3_PCS1_OFFSET)
#define IMXRT_INPUT_LPSPI3_PCS2                  (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPSPI3_PCS2_OFFSET)
#define IMXRT_INPUT_LPSPI3_PCS3                  (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPSPI3_PCS3_OFFSET)
#define IMXRT_INPUT_LPSPI3_SCK                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPSPI3_SCK_OFFSET)
#define IMXRT_INPUT_LPSPI3_SDI                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPSPI3_SDI_OFFSET)
#define IMXRT_INPUT_LPSPI3_SDO                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPSPI3_SDO_OFFSET)
#define IMXRT_INPUT_LPSPI4_PCS0                  (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPSPI4_PCS0_OFFSET)
#define IMXRT_INPUT_LPSPI4_SCK                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPSPI4_SCK_OFFSET)
#define IMXRT_INPUT_LPSPI4_SDI                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPSPI4_SDI_OFFSET)
#define IMXRT_INPUT_LPSPI4_SDO                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPSPI4_SDO_OFFSET)
#define IMXRT_INPUT_LPUART1_RX                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPUART1_RX_OFFSET)
#define IMXRT_INPUT_LPUART1_TX                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPUART1_TX_OFFSET)
#define IMXRT_INPUT_LPUART10_RX                  (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPUART10_RX_OFFSET)
#define IMXRT_INPUT_LPUART10_TX                  (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPUART10_TX_OFFSET)
#define IMXRT_INPUT_LPUART7_RX                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPUART7_RX_OFFSET)
#define IMXRT_INPUT_LPUART7_TX                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPUART7_TX_OFFSET)
#define IMXRT_INPUT_LPUART8_RX                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPUART8_RX_OFFSET)
#define IMXRT_INPUT_LPUART8_TX                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPUART8_TX_OFFSET)
#define IMXRT_INPUT_QTIMER1_TIMER0               (IMXRT_IOMUXC_BASE + IMXRT_INPUT_QTIMER1_TIMER0_OFFSET)
#define IMXRT_INPUT_QTIMER1_TIMER1               (IMXRT_IOMUXC_BASE + IMXRT_INPUT_QTIMER1_TIMER1_OFFSET)
#define IMXRT_INPUT_QTIMER1_TIMER2               (IMXRT_IOMUXC_BASE + IMXRT_INPUT_QTIMER1_TIMER2_OFFSET)
#define IMXRT_INPUT_QTIMER2_TIMER0               (IMXRT_IOMUXC_BASE + IMXRT_INPUT_QTIMER2_TIMER0_OFFSET)
#define IMXRT_INPUT_QTIMER2_TIMER1               (IMXRT_IOMUXC_BASE + IMXRT_INPUT_QTIMER2_TIMER1_OFFSET)
#define IMXRT_INPUT_QTIMER2_TIMER2               (IMXRT_IOMUXC_BASE + IMXRT_INPUT_QTIMER2_TIMER2_OFFSET)
#define IMXRT_INPUT_QTIMER3_TIMER0               (IMXRT_IOMUXC_BASE + IMXRT_INPUT_QTIMER3_TIMER0_OFFSET)
#define IMXRT_INPUT_QTIMER3_TIMER1               (IMXRT_IOMUXC_BASE + IMXRT_INPUT_QTIMER3_TIMER1_OFFSET)
#define IMXRT_INPUT_QTIMER3_TIMER2               (IMXRT_IOMUXC_BASE + IMXRT_INPUT_QTIMER3_TIMER2_OFFSET)
#define IMXRT_INPUT_QTIMER4_TIMER0               (IMXRT_IOMUXC_BASE + IMXRT_INPUT_QTIMER4_TIMER0_OFFSET)
#define IMXRT_INPUT_QTIMER4_TIMER1               (IMXRT_IOMUXC_BASE + IMXRT_INPUT_QTIMER4_TIMER1_OFFSET)
#define IMXRT_INPUT_QTIMER4_TIMER2               (IMXRT_IOMUXC_BASE + IMXRT_INPUT_QTIMER4_TIMER2_OFFSET)
#define IMXRT_INPUT_SAI1_MCLK                    (IMXRT_IOMUXC_BASE + IMXRT_INPUT_SAI1_MCLK_OFFSET)
#define IMXRT_INPUT_SAI1_RX_BCLK                 (IMXRT_IOMUXC_BASE + IMXRT_INPUT_SAI1_RX_BCLK_OFFSET)
#define IMXRT_INPUT_SAI1_RX_DATA0                (IMXRT_IOMUXC_BASE + IMXRT_INPUT_SAI1_RX_DATA0_OFFSET)
#define IMXRT_INPUT_SAI1_RX_SYNC                 (IMXRT_IOMUXC_BASE + IMXRT_INPUT_SAI1_RX_SYNC_OFFSET)
#define IMXRT_INPUT_SAI1_TX_BCLK                 (IMXRT_IOMUXC_BASE + IMXRT_INPUT_SAI1_TX_BCLK_OFFSET)
#define IMXRT_INPUT_SAI1_TX_SYNC                 (IMXRT_IOMUXC_BASE + IMXRT_INPUT_SAI1_TX_SYNC_OFFSET)
#define IMXRT_INPUT_EMVSIM1_SIO                  (IMXRT_IOMUXC_BASE + IMXRT_INPUT_EMVSIM1_SIO_OFFSET)
#define IMXRT_INPUT_EMVSIM1_SIMPD                (IMXRT_IOMUXC_BASE + IMXRT_INPUT_EMVSIM1_SIMPD_OFFSET)
#define IMXRT_INPUT_EMVSIM1_POWER_FAIL           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_EMVSIM1_POWER_FAIL_OFFSET)
#define IMXRT_INPUT_EMVSIM2_SIO                  (IMXRT_IOMUXC_BASE + IMXRT_INPUT_EMVSIM2_SIO_OFFSET)
#define IMXRT_INPUT_EMVSIM2_SIMPD                (IMXRT_IOMUXC_BASE + IMXRT_INPUT_EMVSIM2_SIMPD_OFFSET)
#define IMXRT_INPUT_EMVSIM2_POWER_FAIL           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_EMVSIM2_POWER_FAIL_OFFSET)
#define IMXRT_INPUT_SPDIF_IN                     (IMXRT_IOMUXC_BASE + IMXRT_INPUT_SPDIF_IN_OFFSET)
#define IMXRT_INPUT_USB_OTG2_OC                  (IMXRT_IOMUXC_BASE + IMXRT_INPUT_USB_OTG2_OC_OFFSET)
#define IMXRT_INPUT_USB_OTG1_OC                  (IMXRT_IOMUXC_BASE + IMXRT_INPUT_USB_OTG1_OC_OFFSET)
#define IMXRT_INPUT_USBPHY1_USB_ID               (IMXRT_IOMUXC_BASE + IMXRT_INPUT_USBPHY1_USB_ID_OFFSET)
#define IMXRT_INPUT_USBPHY2_USB_ID               (IMXRT_IOMUXC_BASE + IMXRT_INPUT_USBPHY2_USB_ID_OFFSET)
#define IMXRT_INPUT_USDHC1_CD_B                  (IMXRT_IOMUXC_BASE + IMXRT_INPUT_USDHC1_CD_B_OFFSET)
#define IMXRT_INPUT_USDHC1_WP                    (IMXRT_IOMUXC_BASE + IMXRT_INPUT_USDHC1_WP_OFFSET)
#define IMXRT_INPUT_USDHC2_CD_B                  (IMXRT_IOMUXC_BASE + IMXRT_INPUT_USDHC2_CD_B_OFFSET)
#define IMXRT_INPUT_USDHC2_WP                    (IMXRT_IOMUXC_BASE + IMXRT_INPUT_USDHC2_WP_OFFSET)
#define IMXRT_INPUT_XBAR1_IN20                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_XBAR1_IN20_OFFSET)
#define IMXRT_INPUT_XBAR1_IN21                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_XBAR1_IN21_OFFSET)
#define IMXRT_INPUT_XBAR1_IN22                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_XBAR1_IN22_OFFSET)
#define IMXRT_INPUT_XBAR1_IN23                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_XBAR1_IN23_OFFSET)
#define IMXRT_INPUT_XBAR1_IN24                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_XBAR1_IN24_OFFSET)
#define IMXRT_INPUT_XBAR1_IN25                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_XBAR1_IN25_OFFSET)
#define IMXRT_INPUT_XBAR1_IN26                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_XBAR1_IN26_OFFSET)
#define IMXRT_INPUT_XBAR1_IN27                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_XBAR1_IN27_OFFSET)
#define IMXRT_INPUT_XBAR1_IN28                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_XBAR1_IN28_OFFSET)
#define IMXRT_INPUT_XBAR1_IN29                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_XBAR1_IN29_OFFSET)
#define IMXRT_INPUT_XBAR1_IN30                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_XBAR1_IN30_OFFSET)
#define IMXRT_INPUT_XBAR1_IN31                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_XBAR1_IN31_OFFSET)
#define IMXRT_INPUT_XBAR1_IN32                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_XBAR1_IN32_OFFSET)
#define IMXRT_INPUT_XBAR1_IN33                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_XBAR1_IN33_OFFSET)
#define IMXRT_INPUT_XBAR1_IN34                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_XBAR1_IN34_OFFSET)
#define IMXRT_INPUT_XBAR1_IN35                   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_XBAR1_IN35_OFFSET)

/* IOMUXC_LPSR Select Input Daisy Chain Registers */

#define IMXRT_INPUT_LPSR_FLEXCAN3_RX             (IMXRT_IOMUXCLPSR_BASE + IMXRT_INPUT_LPSR_FLEXCAN3_RX_OFFSET)
#define IMXRT_INPUT_LPSR_LPI2C5_SCL              (IMXRT_IOMUXCLPSR_BASE + IMXRT_INPUT_LPSR_LPI2C5_SCL_OFFSET)
#define IMXRT_INPUT_LPSR_LPI2C5_SDA              (IMXRT_IOMUXCLPSR_BASE + IMXRT_INPUT_LPSR_LPI2C5_SDA_OFFSET)
#define IMXRT_INPUT_LPSR_LPI2C6_SCL              (IMXRT_IOMUXCLPSR_BASE + IMXRT_INPUT_LPSR_LPI2C6_SCL_OFFSET)
#define IMXRT_INPUT_LPSR_LPI2C6_SDA              (IMXRT_IOMUXCLPSR_BASE + IMXRT_INPUT_LPSR_LPI2C6_SDA_OFFSET)
#define IMXRT_INPUT_LPSR_LPSPI5_PCS0             (IMXRT_IOMUXCLPSR_BASE + IMXRT_INPUT_LPSR_LPSPI5_PCS0_OFFSET)
#define IMXRT_INPUT_LPSR_LPSPI5_SCK              (IMXRT_IOMUXCLPSR_BASE + IMXRT_INPUT_LPSR_LPSPI5_SCK_OFFSET)
#define IMXRT_INPUT_LPSR_LPSPI5_SDI              (IMXRT_IOMUXCLPSR_BASE + IMXRT_INPUT_LPSR_LPSPI5_SDI_OFFSET)
#define IMXRT_INPUT_LPSR_LPSPI5_SDO              (IMXRT_IOMUXCLPSR_BASE + IMXRT_INPUT_LPSR_LPSPI5_SDO_OFFSET)
#define IMXRT_INPUT_LPSR_LPUART11_RX             (IMXRT_IOMUXCLPSR_BASE + IMXRT_INPUT_LPSR_LPUART11_RX_OFFSET)
#define IMXRT_INPUT_LPSR_LPUART11_TX             (IMXRT_IOMUXCLPSR_BASE + IMXRT_INPUT_LPSR_LPUART11_TX_OFFSET)
#define IMXRT_INPUT_LPSR_LPUART12_RX             (IMXRT_IOMUXCLPSR_BASE + IMXRT_INPUT_LPSR_LPUART12_RX_OFFSET)
#define IMXRT_INPUT_LPSR_LPUART12_TX             (IMXRT_IOMUXCLPSR_BASE + IMXRT_INPUT_LPSR_LPUART12_TX_OFFSET)
#define IMXRT_INPUT_LPSR_PDM_BITSTREAM0          (IMXRT_IOMUXCLPSR_BASE + IMXRT_INPUT_LPSR_PDM_BITSTREAM0_OFFSET)
#define IMXRT_INPUT_LPSR_PDM_BITSTREAM1          (IMXRT_IOMUXCLPSR_BASE + IMXRT_INPUT_LPSR_PDM_BITSTREAM1_OFFSET)
#define IMXRT_INPUT_LPSR_PDM_BITSTREAM2          (IMXRT_IOMUXCLPSR_BASE + IMXRT_INPUT_LPSR_PDM_BITSTREAM2_OFFSET)
#define IMXRT_INPUT_LPSR_PDM_BITSTREAM3          (IMXRT_IOMUXCLPSR_BASE + IMXRT_INPUT_LPSR_PDM_BITSTREAM3_OFFSET)
#define IMXRT_INPUT_LPSR_NMI_GLUE_NMI            (IMXRT_IOMUXCLPSR_BASE + IMXRT_INPUT_LPSR_NMI_GLUE_NMI_OFFSET)
#define IMXRT_INPUT_LPSR_SAI4_MCLK               (IMXRT_IOMUXCLPSR_BASE + IMXRT_INPUT_LPSR_SAI4_MCLK_OFFSET)
#define IMXRT_INPUT_LPSR_SAI4_RX_BCLK            (IMXRT_IOMUXCLPSR_BASE + IMXRT_INPUT_LPSR_SAI4_RX_BCLK_OFFSET)
#define IMXRT_INPUT_LPSR_SAI4_RX_DATA0           (IMXRT_IOMUXCLPSR_BASE + IMXRT_INPUT_LPSR_SAI4_RX_DATA0_OFFSET)
#define IMXRT_INPUT_LPSR_SAI4_RX_SYNC            (IMXRT_IOMUXCLPSR_BASE + IMXRT_INPUT_LPSR_SAI4_RX_SYNC_OFFSET)
#define IMXRT_INPUT_LPSR_SAI4_TX_BCLK            (IMXRT_IOMUXCLPSR_BASE + IMXRT_INPUT_LPSR_SAI4_TX_BCLK_OFFSET)
#define IMXRT_INPUT_LPSR_SAI4_TX_SYNC            (IMXRT_IOMUXCLPSR_BASE + IMXRT_INPUT_LPSR_SAI4_TX_SYNC_OFFSET)

/* Register Bit Definitions *************************************************/

/* Pad MUX Registers */

#define PADMUX_MUXMODE_SHIFT              (0)       /* Bit 0-3:    MUX Mode Select Field (MUX_MODE) */
#define PADMUX_MUXMODE_MASK               (0x0f << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT0             (0 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT1             (1 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT2             (2 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT3             (3 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT4             (4 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT5             (5 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT6             (6 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT7             (7 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT8             (8 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT9             (9 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT10            (10 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT11            (11 << PADMUX_MUXMODE_SHIFT)
#define PADMUX_SION                       (1 << 4)  /* Bit 4:      Software Input On Field (SION) */
                                                    /* Bits 5-31:  Reserved */

/* GPIO_EMC_B1 Pad CTL Registers */

                                                    /* Bit 0:      Reserved */
#define PADCTL_EMC_B1_PDRV                (1 << 1)  /* Bit 1:      PDRV Field (PDRV) */
#define PADCTL_EMC_B1_PULL_SHIFT          (2)       /* Bits 2-3:   Pull Down / Pull Up Field (PULL) */
#define PADCTL_EMC_B1_PULL_MASK           (0x03 << PADCTL_EMC_B1_PULL_SHIFT)
#  define PADCTL_EMC_B1_PULL_UP           (0x01 << PADCTL_EMC_B1_PULL_SHIFT) /* Internal pullup resistor enabled */
#  define PADCTL_EMC_B1_PULL_DOWN         (0x02 << PADCTL_EMC_B1_PULL_SHIFT) /* Internal pulldown resistor enabled */
#  define PADCTL_EMC_B1_PULL_NONE         (0x03 << PADCTL_EMC_B1_PULL_SHIFT) /* No pull */

#define PADCTL_EMC_B1_ODE                 (1 << 4)  /* Bit 4:      Open Drain Field (ODE) */
                                                    /* Bits 5-27:  Reserved */
#define PADCTL_EMC_B1_DWP_SHIFT           (28)      /* Bits 28-29: Domain write protection (DWP) */
#define PADCTL_EMC_B1_DWP_MASK            (0x03 << PADCTL_EMC_B1_DWP_SHIFT)
#  define PADCTL_EMC_B1_DWP_ALLOWBOTH     (0x00 << PADCTL_EMC_B1_DWP_SHIFT) /* Both cores are allowed */
#  define PADCTL_EMC_B1_DWP_DENYCM7       (0x01 << PADCTL_EMC_B1_DWP_SHIFT) /* CM7 is forbidden */
#  define PADCTL_EMC_B1_DWP_DENYCM4       (0x02 << PADCTL_EMC_B1_DWP_SHIFT) /* CM4 is forbidden */
#  define PADCTL_EMC_B1_DWP_DENYBOTH      (0x03 << PADCTL_EMC_B1_DWP_SHIFT) /* Both cores are forbidden */

#define PADCTL_EMC_B1_DWP_LOCK_SHIFT      (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define PADCTL_EMC_B1_DWP_LOCK_MASK       (0x03 << PADCTL_EMC_B1_DWP_LOCK_SHIFT)
#  define PADCTL_EMC_B1_DWP_LOCK_NOLCK    (0x00 << PADCTL_EMC_B1_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define PADCTL_EMC_B1_DWP_LOCK_LCKLO    (0x01 << PADCTL_EMC_B1_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define PADCTL_EMC_B1_DWP_LOCK_LCKHI    (0x02 << PADCTL_EMC_B1_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define PADCTL_EMC_B1_DWP_LOCK_LCKBOTH  (0x03 << PADCTL_EMC_B1_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* GPIO_EMC_B2 Pad CTL Registers */

                                                    /* Bit 0:      Reserved */
#define PADCTL_EMC_B2_PDRV                (1 << 1)  /* Bit 1:      PDRV Field (PDRV) */
#define PADCTL_EMC_B2_PULL_SHIFT          (2)       /* Bits 2-3:   Pull Down / Pull Up Field (PULL) */
#define PADCTL_EMC_B2_PULL_MASK           (0x03 << PADCTL_EMC_B2_PULL_SHIFT)
#  define PADCTL_EMC_B2_PULL_UP           (0x01 << PADCTL_EMC_B2_PULL_SHIFT) /* Internal pullup resistor enabled */
#  define PADCTL_EMC_B2_PULL_DOWN         (0x02 << PADCTL_EMC_B2_PULL_SHIFT) /* Internal pulldown resistor enabled */
#  define PADCTL_EMC_B2_PULL_NONE         (0x03 << PADCTL_EMC_B2_PULL_SHIFT) /* No pull */

#define PADCTL_EMC_B2_ODE                 (1 << 4)  /* Bit 4:      Open Drain Field (ODE) */
                                                    /* Bits 5-27:  Reserved */
#define PADCTL_EMC_B2_DWP_SHIFT           (28)      /* Bits 28-29: Domain write protection (DWP) */
#define PADCTL_EMC_B2_DWP_MASK            (0x03 << PADCTL_EMC_B2_DWP_SHIFT)
#  define PADCTL_EMC_B2_DWP_ALLOWBOTH     (0x00 << PADCTL_EMC_B2_DWP_SHIFT) /* Both cores are allowed */
#  define PADCTL_EMC_B2_DWP_DENYCM7       (0x01 << PADCTL_EMC_B2_DWP_SHIFT) /* CM7 is forbidden */
#  define PADCTL_EMC_B2_DWP_DENYCM4       (0x02 << PADCTL_EMC_B2_DWP_SHIFT) /* CM4 is forbidden */
#  define PADCTL_EMC_B2_DWP_DENYBOTH      (0x03 << PADCTL_EMC_B2_DWP_SHIFT) /* Both cores are forbidden */

#define PADCTL_EMC_B2_DWP_LOCK_SHIFT      (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define PADCTL_EMC_B2_DWP_LOCK_MASK       (0x03 << PADCTL_EMC_B2_DWP_LOCK_SHIFT)
#  define PADCTL_EMC_B2_DWP_LOCK_NOLCK    (0x00 << PADCTL_EMC_B2_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define PADCTL_EMC_B2_DWP_LOCK_LCKLO    (0x01 << PADCTL_EMC_B2_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define PADCTL_EMC_B2_DWP_LOCK_LCKHI    (0x02 << PADCTL_EMC_B2_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define PADCTL_EMC_B2_DWP_LOCK_LCKBOTH  (0x03 << PADCTL_EMC_B2_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* GPIO_AD Pad CTL Registers */

#define PADCTL_AD_SRE                     (1 << 0)  /* Bit 0:      Slew Rate Field (SRE) */
#define PADCTL_AD_DSE                     (1 << 1)  /* Bit 1:      Drive Strength Field (DSE) */
#define PADCTL_AD_PUE                     (1 << 2)  /* Bit 2:      Pull / Keep Select Field (PUE) */
#define PADCTL_AD_PUS                     (1 << 3)  /* Bit 3:      Pull Up / Down Config. Field (PUS) */
#  define PADCTL_AD_PULL_DOWN             (0 << 3)  /*             Weak pull down */
#  define PADCTL_AD_PULL_UP               (1 << 3)  /*             Weak pull up */
#define PADCTL_AD_ODE                     (1 << 4)  /* Bit 4:      Open Drain Field (ODE) */
                                                    /* Bits 5-27:  Reserved */
#define PADCTL_AD_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define PADCTL_AD_DWP_MASK                (0x03 << PADCTL_AD_DWP_SHIFT)
#  define PADCTL_AD_DWP_ALLOWBOTH         (0x00 << PADCTL_AD_DWP_SHIFT) /* Both cores are allowed */
#  define PADCTL_AD_DWP_DENYCM7           (0x01 << PADCTL_AD_DWP_SHIFT) /* CM7 is forbidden */
#  define PADCTL_AD_DWP_DENYCM4           (0x02 << PADCTL_AD_DWP_SHIFT) /* CM4 is forbidden */
#  define PADCTL_AD_DWP_DENYBOTH          (0x03 << PADCTL_AD_DWP_SHIFT) /* Both cores are forbidden */

#define PADCTL_AD_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define PADCTL_AD_DWP_LOCK_MASK           (0x03 << PADCTL_AD_DWP_LOCK_SHIFT)
#  define PADCTL_AD_DWP_LOCK_NOLCK        (0x00 << PADCTL_AD_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define PADCTL_AD_DWP_LOCK_LCKLO        (0x01 << PADCTL_AD_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define PADCTL_AD_DWP_LOCK_LCKHI        (0x02 << PADCTL_AD_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define PADCTL_AD_DWP_LOCK_LCKBOTH      (0x03 << PADCTL_AD_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* GPIO_SD_B1 Pad CTL Registers GPIO_DISP_B1 */

                                                    /* Bit 0:      Reserved */
#define PADCTL_SD_DISP_B1_PDRV                 (1 << 1)  /* Bit 1:      PDRV Field (PDRV) */
#define PADCTL_SD_DISP_B1_PULL_SHIFT           (2)       /* Bits 2-3:   Pull Down / Pull Up Field (PULL) */
#define PADCTL_SD_DISP_B1_PULL_MASK            (0x03 << PADCTL_SD_DISP_B1_PULL_SHIFT)
#  define PADCTL_SD_DISP_B1_PULL_UP            (0x01 << PADCTL_SD_DISP_B1_PULL_SHIFT) /* Internal pullup resistor enabled */
#  define PADCTL_SD_DISP_B1_PULL_DOWN          (0x02 << PADCTL_SD_DISP_B1_PULL_SHIFT) /* Internal pulldown resistor enabled */
#  define PADCTL_SD_DISP_B1_PULL_NONE          (0x03 << PADCTL_SD_DISP_B1_PULL_SHIFT) /* No pull */

#define PADCTL_SD_DISP_B1_ODE                  (1 << 4)  /* Bit 4:      Open Drain Field (ODE) */
                                                         /* Bits 5-27:  Reserved */
#define PADCTL_SD_DISP_B1_DWP_SHIFT            (28)      /* Bits 28-29: Domain write protection (DWP) */
#define PADCTL_SD_DISP_B1_DWP_MASK             (0x03 << PADCTL_SD_DISP_B1_DWP_SHIFT)
#  define PADCTL_SD_DISP_B1_DWP_ALLOWBOTH      (0x00 << PADCTL_SD_DISP_B1_DWP_SHIFT) /* Both cores are allowed */
#  define PADCTL_SD_DISP_B1_DWP_DENYCM7        (0x01 << PADCTL_SD_DISP_B1_DWP_SHIFT) /* CM7 is forbidden */
#  define PADCTL_SD_DISP_B1_DWP_DENYCM4        (0x02 << PADCTL_SD_DISP_B1_DWP_SHIFT) /* CM4 is forbidden */
#  define PADCTL_SD_DISP_B1_DWP_DENYBOTH       (0x03 << PADCTL_SD_DISP_B1_DWP_SHIFT) /* Both cores are forbidden */

#define PADCTL_SD_DISP_B1_DWP_LOCK_SHIFT       (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define PADCTL_SD_DISP_B1_DWP_LOCK_MASK        (0x03 << PADCTL_SD_DISP_B1_DWP_LOCK_SHIFT)
#  define PADCTL_SD_DISP_B1_DWP_LOCK_NOLCK     (0x00 << PADCTL_SD_DISP_B1_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define PADCTL_SD_DISP_B1_DWP_LOCK_LCKLO     (0x01 << PADCTL_SD_DISP_B1_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define PADCTL_SD_DISP_B1_DWP_LOCK_LCKHI     (0x02 << PADCTL_SD_DISP_B1_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define PADCTL_SD_DISP_B1_DWP_LOCK_LCKBOTH   (0x03 << PADCTL_SD_DISP_B1_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* GPIO_SD_B2 Pad CTL Registers */

                                                    /* Bit 0:      Reserved */
#define PADCTL_SD_B2_PDRV                 (1 << 1)  /* Bit 1:      PDRV Field (PDRV) */
#define PADCTL_SD_B2_PULL_SHIFT           (2)       /* Bits 2-3:   Pull Down / Pull Up Field (PULL) */
#define PADCTL_SD_B2_PULL_MASK            (0x03 << PADCTL_SD_B2_PULL_SHIFT)
#  define PADCTL_SD_B2_PULL_UP            (0x01 << PADCTL_SD_B2_PULL_SHIFT) /* Internal pullup resistor enabled */
#  define PADCTL_SD_B2_PULL_DOWN          (0x02 << PADCTL_SD_B2_PULL_SHIFT) /* Internal pulldown resistor enabled */
#  define PADCTL_SD_B2_PULL_NONE          (0x03 << PADCTL_SD_B2_PULL_SHIFT) /* No pull */

#define PADCTL_SD_B2_ODE                  (1 << 4)  /* Bit 4:      Open Drain Field (ODE) */
                                                    /* Bits 5-27:  Reserved */
#define PADCTL_SD_B2_DWP_SHIFT            (28)      /* Bits 28-29: Domain write protection (DWP) */
#define PADCTL_SD_B2_DWP_MASK             (0x03 << PADCTL_SD_B2_DWP_SHIFT)
#  define PADCTL_SD_B2_DWP_ALLOWBOTH      (0x00 << PADCTL_SD_B2_DWP_SHIFT) /* Both cores are allowed */
#  define PADCTL_SD_B2_DWP_DENYCM7        (0x01 << PADCTL_SD_B2_DWP_SHIFT) /* CM7 is forbidden */
#  define PADCTL_SD_B2_DWP_DENYCM4        (0x02 << PADCTL_SD_B2_DWP_SHIFT) /* CM4 is forbidden */
#  define PADCTL_SD_B2_DWP_DENYBOTH       (0x03 << PADCTL_SD_B2_DWP_SHIFT) /* Both cores are forbidden */

#define PADCTL_SD_B2_DWP_LOCK_SHIFT       (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define PADCTL_SD_B2_DWP_LOCK_MASK        (0x03 << PADCTL_SD_B2_DWP_LOCK_SHIFT)
#  define PADCTL_SD_B2_DWP_LOCK_NOLCK     (0x00 << PADCTL_SD_B2_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define PADCTL_SD_B2_DWP_LOCK_LCKLO     (0x01 << PADCTL_SD_B2_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define PADCTL_SD_B2_DWP_LOCK_LCKHI     (0x02 << PADCTL_SD_B2_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define PADCTL_SD_B2_DWP_LOCK_LCKBOTH   (0x03 << PADCTL_SD_B2_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* GPIO_DISP_B2 Pad CTL Registers */

#define PADCTL_DISP_B2_SRE                (1 << 0)  /* Bit 0:      Slew Rate Field (SRE) */
#define PADCTL_DISP_B2_DSE                (1 << 1)  /* Bit 1:      Drive Strength Field (DSE) */
#define PADCTL_DISP_B2_PUE                (1 << 2)  /* Bit 2:      Pull / Keep Select Field (PUE) */
#define PADCTL_DISP_B2_PUS                (1 << 3)  /* Bit 3:      Pull Up / Down Config. Field (PUS) */
#  define PADCTL_DISP_B2_PULL_DOWN        (0 << 3)  /*             Weak pull down */
#  define PADCTL_DISP_B2_PULL_UP          (1 << 3)  /*             Weak pull up */
#define PADCTL_DISP_B2_ODE                (1 << 4)  /* Bit 4:      Open Drain Field (ODE) */
                                                    /* Bits 5-27:  Reserved */
#define PADCTL_DISP_B2_DWP_SHIFT          (28)      /* Bits 28-29: Domain write protection (DWP) */
#define PADCTL_DISP_B2_DWP_MASK           (0x03 << PADCTL_DISP_B2_DWP_SHIFT)
#  define PADCTL_DISP_B2_DWP_ALLOWBOTH    (0x00 << PADCTL_DISP_B2_DWP_SHIFT) /* Both cores are allowed */
#  define PADCTL_DISP_B2_DWP_DENYCM7      (0x01 << PADCTL_DISP_B2_DWP_SHIFT) /* CM7 is forbidden */
#  define PADCTL_DISP_B2_DWP_DENYCM4      (0x02 << PADCTL_DISP_B2_DWP_SHIFT) /* CM4 is forbidden */
#  define PADCTL_DISP_B2_DWP_DENYBOTH     (0x03 << PADCTL_DISP_B2_DWP_SHIFT) /* Both cores are forbidden */

#define PADCTL_DISP_B2_DWP_LOCK_SHIFT     (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define PADCTL_DISP_B2_DWP_LOCK_MASK      (0x03 << PADCTL_DISP_B2_DWP_LOCK_SHIFT)
#  define PADCTL_DISP_B2_DWP_LOCK_NOLCK   (0x00 << PADCTL_DISP_B2_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define PADCTL_DISP_B2_DWP_LOCK_LCKLO   (0x01 << PADCTL_DISP_B2_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define PADCTL_DISP_B2_DWP_LOCK_LCKHI   (0x02 << PADCTL_DISP_B2_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define PADCTL_DISP_B2_DWP_LOCK_LCKBOTH (0x03 << PADCTL_DISP_B2_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* GPIO_SNVS Pad CTL Registers */

#define PADCTL_SNVS_PUE                   (1 << 2)  /* Bit 2:      Pull / Keep Select Field (PUE) */
#define PADCTL_SNVS_PUS                   (1 << 3)  /* Bit 3:      Pull Up / Down Config. Field (PUS) */
#  define PADCTL_SNVS_PULL_DOWN           (0 << 3)  /*             Weak pull down */
#  define PADCTL_SNVS_PULL_UP             (1 << 3)  /*             Weak pull up */
                                                    /* Bits 4-5:   Reserved */
#define PADCTL_SNVS_ODE                   (1 << 6)  /* Bit 6:      Open Drain Field (ODE) */
                                                    /* Bits 7-27:  Reserved */
#define PADCTL_SNVS_DWP_SHIFT             (28)      /* Bits 28-29: Domain write protection (DWP) */
#define PADCTL_SNVS_DWP_MASK              (0x03 << PADCTL_SNVS_DWP_SHIFT)
#  define PADCTL_SNVS_DWP_ALLOWBOTH       (0x00 << PADCTL_SNVS_DWP_SHIFT) /* Both cores are allowed */
#  define PADCTL_SNVS_DWP_DENYCM7         (0x01 << PADCTL_SNVS_DWP_SHIFT) /* CM7 is forbidden */
#  define PADCTL_SNVS_DWP_DENYCM4         (0x02 << PADCTL_SNVS_DWP_SHIFT) /* CM4 is forbidden */
#  define PADCTL_SNVS_DWP_DENYBOTH        (0x03 << PADCTL_SNVS_DWP_SHIFT) /* Both cores are forbidden */

#define PADCTL_SNVS_DWP_LOCK_SHIFT        (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define PADCTL_SNVS_DWP_LOCK_MASK         (0x03 << PADCTL_SNVS_DWP_LOCK_SHIFT)
#  define PADCTL_SNVS_DWP_LOCK_NOLCK      (0x00 << PADCTL_SNVS_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define PADCTL_SNVS_DWP_LOCK_LCKLO      (0x01 << PADCTL_SNVS_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define PADCTL_SNVS_DWP_LOCK_LCKHI      (0x02 << PADCTL_SNVS_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define PADCTL_SNVS_DWP_LOCK_LCKBOTH    (0x03 << PADCTL_SNVS_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* GPIO_LPSR Pad CTL Registers */

#define PADCTL_LPSR_SRE                   (1 << 0)  /* Bit 0:      Slew Rate Field (SRE) */
#define PADCTL_LPSR_DSE                   (1 << 1)  /* Bit 1:      Drive Strength Field (DSE) */
#define PADCTL_LPSR_PUE                   (1 << 2)  /* Bit 2:      Pull / Keep Select Field (PUE) */
#define PADCTL_LPSR_PUS                   (1 << 3)  /* Bit 3:      Pull Up / Down Config. Field (PUS) */
#  define PADCTL_LPSR_PULL_DOWN           (0 << 3)  /*             Weak pull down */
#  define PADCTL_LPSR_PULL_UP             (1 << 3)  /*             Weak pull up */
                                                    /* Bit 4:      Reserved */
#define PADCTL_LPSR_ODE                   (1 << 5)  /* Bit 5:      Open Drain Field (ODE) */
                                                    /* Bits 6-27:  Reserved */
#define PADCTL_LPSR_DWP_SHIFT             (28)      /* Bits 28-29: Domain write protection (DWP) */
#define PADCTL_LPSR_DWP_MASK              (0x03 << PADCTL_LPSR_DWP_SHIFT)
#  define PADCTL_LPSR_DWP_ALLOWBOTH       (0x00 << PADCTL_LPSR_DWP_SHIFT) /* Both cores are allowed */
#  define PADCTL_LPSR_DWP_DENYCM7         (0x01 << PADCTL_LPSR_DWP_SHIFT) /* CM7 is forbidden */
#  define PADCTL_LPSR_DWP_DENYCM4         (0x02 << PADCTL_LPSR_DWP_SHIFT) /* CM4 is forbidden */
#  define PADCTL_LPSR_DWP_DENYBOTH        (0x03 << PADCTL_LPSR_DWP_SHIFT) /* Both cores are forbidden */

#define PADCTL_LPSR_DWP_LOCK_SHIFT        (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define PADCTL_LPSR_DWP_LOCK_MASK         (0x03 << PADCTL_LPSR_DWP_LOCK_SHIFT)
#  define PADCTL_LPSR_DWP_LOCK_NOLCK      (0x00 << PADCTL_LPSR_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define PADCTL_LPSR_DWP_LOCK_LCKLO      (0x01 << PADCTL_LPSR_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define PADCTL_LPSR_DWP_LOCK_LCKHI      (0x02 << PADCTL_LPSR_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define PADCTL_LPSR_DWP_LOCK_LCKBOTH    (0x03 << PADCTL_LPSR_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 0 (GPR0) */

#define GPR_GPR0_SAI1_MCLK1_SEL_SHIFT                    (0) /* Bits 0-2: SAI1 MCLK1 source select (SAI1_MCLK1_SEL) */
#define GPR_GPR0_SAI1_MCLK1_SEL_MASK                     (0x07 << GPR_GPR0_SAI1_MCLK1_SEL_SHIFT)
#  define GPR_GPR0_SAI1_MCLK1_SEL_SAI1_CLK_ROOT          (0x00 << GPR_GPR0_SAI1_MCLK1_SEL_SHIFT) /* SAI1_CLK_ROOT */
#  define GPR_GPR0_SAI1_MCLK1_SEL_SAI2_CLK_ROOT          (0x01 << GPR_GPR0_SAI1_MCLK1_SEL_SHIFT) /* SAI2_CLK_ROOT */
#  define GPR_GPR0_SAI1_MCLK1_SEL_SAI3_CLK_ROOT          (0x02 << GPR_GPR0_SAI1_MCLK1_SEL_SHIFT) /* SAI3_CLK_ROOT */
#  define GPR_GPR0_SAI1_MCLK1_SEL_SAI1_IPG_CLK_SAI_MCLK  (0x03 << GPR_GPR0_SAI1_MCLK1_SEL_SHIFT) /* SAI1_IPG_CLK_SAI_MCLK */
#  define GPR_GPR0_SAI1_MCLK1_SEL_SAI2_IPG_CLK_SAI_MCLK  (0x04 << GPR_GPR0_SAI1_MCLK1_SEL_SHIFT) /* SAI2_IPG_CLK_SAI_MCLK */
#  define GPR_GPR0_SAI1_MCLK1_SEL_SAI3_IPG_CLK_SAI_MCLK  (0x05 << GPR_GPR0_SAI1_MCLK1_SEL_SHIFT) /* SAI3_IPG_CLK_SAI_MCLK */

#define GPR_GPR0_SAI1_MCLK2_SEL_SHIFT                    (3) /* Bits 3-5: SAI1 MCLK2 source select (SAI1_MCLK2_SEL) */
#define GPR_GPR0_SAI1_MCLK2_SEL_MASK                     (0x07 << GPR_GPR0_SAI1_MCLK2_SEL_SHIFT)
#  define GPR_GPR0_SAI1_MCLK2_SEL_SAI1_CLK_ROOT          (0x00 << GPR_GPR0_SAI1_MCLK2_SEL_SHIFT) /* SAI1_CLK_ROOT */
#  define GPR_GPR0_SAI1_MCLK2_SEL_SAI2_CLK_ROOT          (0x01 << GPR_GPR0_SAI1_MCLK2_SEL_SHIFT) /* SAI2_CLK_ROOT */
#  define GPR_GPR0_SAI1_MCLK2_SEL_SAI3_CLK_ROOT          (0x02 << GPR_GPR0_SAI1_MCLK2_SEL_SHIFT) /* SAI3_CLK_ROOT */
#  define GPR_GPR0_SAI1_MCLK2_SEL_SAI1_IPG_CLK_SAI_MCLK  (0x03 << GPR_GPR0_SAI1_MCLK2_SEL_SHIFT) /* SAI1_IPG_CLK_SAI_MCLK */
#  define GPR_GPR0_SAI1_MCLK2_SEL_SAI2_IPG_CLK_SAI_MCLK  (0x04 << GPR_GPR0_SAI1_MCLK2_SEL_SHIFT) /* SAI2_IPG_CLK_SAI_MCLK */
#  define GPR_GPR0_SAI1_MCLK2_SEL_SAI3_IPG_CLK_SAI_MCLK  (0x05 << GPR_GPR0_SAI1_MCLK2_SEL_SHIFT) /* SAI3_IPG_CLK_SAI_MCLK */

#define GPR_GPR0_SAI1_MCLK3_SEL_SHIFT             (6) /* Bits 6-7: SAI1 MCLK3 source select (SAI1_MCLK3_SEL) */
#define GPR_GPR0_SAI1_MCLK3_SEL_MASK              (0x03 << GPR_GPR0_SAI1_MCLK3_SEL_SHIFT)
#  define GPR_GPR0_SAI1_MCLK3_SEL_SPDIF_CLK_ROOT  (0x00 << GPR_GPR0_SAI1_MCLK3_SEL_SHIFT) /* SPDIF_CLK_ROOT */
#  define GPR_GPR0_SAI1_MCLK3_SEL_SPDIF_TX_CLK2   (0x01 << GPR_GPR0_SAI1_MCLK3_SEL_SHIFT) /* SPDIF_TX_CLK2 */
#  define GPR_GPR0_SAI1_MCLK3_SEL_SPDIF_SRCLK     (0x02 << GPR_GPR0_SAI1_MCLK3_SEL_SHIFT) /* SPDIF_SRCLK */
#  define GPR_GPR0_SAI1_MCLK3_SEL_SPDIF_OUTCLOCK  (0x03 << GPR_GPR0_SAI1_MCLK3_SEL_SHIFT) /* SPDIF_OUTCLOCK */

#define GPR_GPR0_SAI1_MCLK_DIR            (1 << 8)  /* Bit 8:      SAI1_MCLK signal direction control (SAI1_MCLK_DIR) */
#  define GPR_GPR0_SAI1_MCLK_DIR_IN       (0 << 8)  /*             SAI1_MCLK is input signal */
#  define GPR_GPR0_SAI1_MCLK_DIR_OUT      (1 << 8)  /*             SAI1_MCLK is output signal */
                                                    /* Bits 9-15:  Reserved */
                                                    /* Bits 16-27: Reserved */
#define GPR_GPR0_DWP_SHIFT                (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR0_DWP_MASK                 (0x03 << GPR_GPR0_DWP_SHIFT)
#  define GPR_GPR0_DWP_ALLOWBOTH          (0x00 << GPR_GPR0_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR0_DWP_DENYCM7            (0x01 << GPR_GPR0_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR0_DWP_DENYCM4            (0x02 << GPR_GPR0_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR0_DWP_DENYBOTH           (0x03 << GPR_GPR0_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR0_DWP_LOCK_SHIFT           (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR0_DWP_LOCK_MASK            (0x03 << GPR_GPR0_DWP_LOCK_SHIFT)
#  define GPR_GPR0_DWP_LOCK_NOLCK         (0x00 << GPR_GPR0_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR0_DWP_LOCK_LCKLO         (0x01 << GPR_GPR0_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR0_DWP_LOCK_LCKHI         (0x02 << GPR_GPR0_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR0_DWP_LOCK_LCKBOTH       (0x03 << GPR_GPR0_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 1 (GPR1) */

#define GPR_GPR1_SAI2_MCLK3_SEL_SHIFT             (0) /* Bits 0-1: SAI2 MCLK3 source select (SAI2_MCLK3_SEL) */
#define GPR_GPR1_SAI2_MCLK3_SEL_MASK              (0x03 << GPR_GPR1_SAI2_MCLK3_SEL_SHIFT)
#  define GPR_GPR1_SAI2_MCLK3_SEL_SPDIF_CLK_ROOT  (0x00 << GPR_GPR1_SAI2_MCLK3_SEL_SHIFT) /* SPDIF_CLK_ROOT */
#  define GPR_GPR1_SAI2_MCLK3_SEL_SPDIF_TX_CLK2   (0x01 << GPR_GPR1_SAI2_MCLK3_SEL_SHIFT) /* SPDIF_TX_CLK2 */
#  define GPR_GPR1_SAI2_MCLK3_SEL_SPDIF_SRCLK     (0x02 << GPR_GPR1_SAI2_MCLK3_SEL_SHIFT) /* SPDIF_SRCLK */
#  define GPR_GPR1_SAI2_MCLK3_SEL_SPDIF_OUTCLOCK  (0x03 << GPR_GPR1_SAI2_MCLK3_SEL_SHIFT) /* SPDIF_OUTCLOCK */

                                                    /* Bits 2-7:   Reserved */
#define GPR_GPR1_SAI2_MCLK_DIR            (1 << 8)  /* Bit 8:      SAI2_MCLK signal direction control (SAI2_MCLK_DIR) */
#  define GPR_GPR1_SAI2_MCLK_DIR_IN       (0 << 8)  /*             SAI2_MCLK is input signal */
#  define GPR_GPR1_SAI2_MCLK_DIR_OUT      (1 << 8)  /*             SAI2_MCLK is output signal */
                                                    /* Bits 9-15:  Reserved */
                                                    /* Bits 16-27: Reserved */
#define GPR_GPR1_DWP_SHIFT                (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR1_DWP_MASK                 (0x03 << GPR_GPR1_DWP_SHIFT)
#  define GPR_GPR1_DWP_ALLOWBOTH          (0x00 << GPR_GPR1_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR1_DWP_DENYCM7            (0x01 << GPR_GPR1_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR1_DWP_DENYCM4            (0x02 << GPR_GPR1_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR1_DWP_DENYBOTH           (0x03 << GPR_GPR1_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR1_DWP_LOCK_SHIFT           (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR1_DWP_LOCK_MASK            (0x03 << GPR_GPR1_DWP_LOCK_SHIFT)
#  define GPR_GPR1_DWP_LOCK_NOLCK         (0x00 << GPR_GPR1_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR1_DWP_LOCK_LCKLO         (0x01 << GPR_GPR1_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR1_DWP_LOCK_LCKHI         (0x02 << GPR_GPR1_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR1_DWP_LOCK_LCKBOTH       (0x03 << GPR_GPR1_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 2 (GPR2) */

#define GPR_GPR2_SAI3_MCLK3_SEL_SHIFT             (0) /* Bits 0-1: SAI3 MCLK3 source select (SAI3_MCLK3_SEL) */
#define GPR_GPR2_SAI3_MCLK3_SEL_MASK              (0x03 << GPR_GPR2_SAI3_MCLK3_SEL_SHIFT)
#  define GPR_GPR2_SAI3_MCLK3_SEL_SPDIF_CLK_ROOT  (0x00 << GPR_GPR2_SAI3_MCLK3_SEL_SHIFT) /* SPDIF_CLK_ROOT */
#  define GPR_GPR2_SAI3_MCLK3_SEL_SPDIF_TX_CLK2   (0x01 << GPR_GPR2_SAI3_MCLK3_SEL_SHIFT) /* SPDIF_TX_CLK2 */
#  define GPR_GPR2_SAI3_MCLK3_SEL_SPDIF_SRCLK     (0x02 << GPR_GPR2_SAI3_MCLK3_SEL_SHIFT) /* SPDIF_SRCLK */
#  define GPR_GPR2_SAI3_MCLK3_SEL_SPDIF_OUTCLOCK  (0x03 << GPR_GPR2_SAI3_MCLK3_SEL_SHIFT) /* SPDIF_OUTCLOCK */

                                                    /* Bits 2-7:   Reserved */
#define GPR_GPR2_SAI3_MCLK_DIR            (1 << 8)  /* Bit 8:      SAI3_MCLK signal direction control (SAI3_MCLK_DIR) */
#  define GPR_GPR2_SAI3_MCLK_DIR_IN       (0 << 8)  /*             SAI3_MCLK is input signal */
#  define GPR_GPR2_SAI3_MCLK_DIR_OUT      (1 << 8)  /*             SAI3_MCLK is output signal */
#define GPR_GPR2_SAI4_MCLK_DIR            (1 << 9)  /* Bit 9:      SAI4_MCLK signal direction control (SAI4_MCLK_DIR) */
#  define GPR_GPR2_SAI4_MCLK_DIR_IN       (0 << 9)  /*             SAI4_MCLK is input signal */
#  define GPR_GPR2_SAI4_MCLK_DIR_OUT      (1 << 9)  /*             SAI4_MCLK is output signal */
                                                    /* Bits 10-15: Reserved */
                                                    /* Bits 16-27: Reserved */
#define GPR_GPR2_DWP_SHIFT                (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR2_DWP_MASK                 (0x03 << GPR_GPR2_DWP_SHIFT)
#  define GPR_GPR2_DWP_ALLOWBOTH          (0x00 << GPR_GPR2_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR2_DWP_DENYCM7            (0x01 << GPR_GPR2_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR2_DWP_DENYCM4            (0x02 << GPR_GPR2_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR2_DWP_DENYBOTH           (0x03 << GPR_GPR2_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR2_DWP_LOCK_SHIFT           (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR2_DWP_LOCK_MASK            (0x03 << GPR_GPR2_DWP_LOCK_SHIFT)
#  define GPR_GPR2_DWP_LOCK_NOLCK         (0x00 << GPR_GPR2_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR2_DWP_LOCK_LCKLO         (0x01 << GPR_GPR2_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR2_DWP_LOCK_LCKHI         (0x02 << GPR_GPR2_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR2_DWP_LOCK_LCKBOTH       (0x03 << GPR_GPR2_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 3 (GPR3) */

#define GPR_GPR3_MQS_CLK_DIV_SHIFT        (16)      /* Bits 0-7:   Divider ratio control for MCLK from HMCLK (MQS_CLK_DIV) */
#define GPR_GPR3_MQS_CLK_DIV_MASK         (0xff << GPR_GPR3_MQS_CLK_DIV_SHIFT)
#  define GPR_GPR3_MQS_CLK_DIV(n)         (((n)-1) << GPR_GPR3_MQS_CLK_DIV_SHIFT) /* f_MCLK = f_HMCLK / n */

#define GPR_GPR3_MQS_SW_RST               (1 << 8)  /* Bit 8:      MQS software reset (MQS_SW_RST) */
#define GPR_GPR3_MQS_EN                   (1 << 9)  /* Bit 9:      MQS enable (MQS_EN) */
#define GPR_GPR3_MQS_OVERSAMPLE           (1 << 10) /* Bit 10:     MQS oversample (MQS_OVERSAMPLE) */
#define GPR_GPR3_MQS_OVERSAMPLE_32        (0 << 10) /*             32x oversampling rate */
#define GPR_GPR3_MQS_OVERSAMPLE_64        (1 << 10) /*             64x oversampling rate */
                                                    /* Bits 11-15: Reserved */
                                                    /* Bits 16-27: Reserved */
#define GPR_GPR3_DWP_SHIFT                (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR3_DWP_MASK                 (0x03 << GPR_GPR3_DWP_SHIFT)
#  define GPR_GPR3_DWP_ALLOWBOTH          (0x00 << GPR_GPR3_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR3_DWP_DENYCM7            (0x01 << GPR_GPR3_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR3_DWP_DENYCM4            (0x02 << GPR_GPR3_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR3_DWP_DENYBOTH           (0x03 << GPR_GPR3_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR3_DWP_LOCK_SHIFT           (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR3_DWP_LOCK_MASK            (0x03 << GPR_GPR3_DWP_LOCK_SHIFT)
#  define GPR_GPR3_DWP_LOCK_NOLCK         (0x00 << GPR_GPR3_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR3_DWP_LOCK_LCKLO         (0x01 << GPR_GPR3_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR3_DWP_LOCK_LCKHI         (0x02 << GPR_GPR3_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR3_DWP_LOCK_LCKBOTH       (0x03 << GPR_GPR3_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 4 (GPR4) */

#define GPR_GPR4_ENET_TX_CLK_SEL          (1 << 0)  /* Bit 0:      ENET TX_CLK select (ENET_TX_CLK_SEL) */
#  define GPR_GPR4_ENET_TX_CLK_SEL_NS     (0 << 0)  /*             Not supported */
#  define GPR_GPR4_ENET_TX_CLK_SEL_PAD    (1 << 0)  /*             ENET TX_CLK is from pad */
#define GPR_GPR4_ENET_REF_CLK_DIR         (1 << 1)  /* Bit 1:      ENET_REF_CLK direction control (ENET_REF_CLK_DIR) */
#  define GPR_GPR4_ENET_REF_CLK_DIR_IN    (0 << 1)  /*             ENET_REF_CLK is input */
#  define GPR_GPR4_ENET_REF_CLK_DIR_OUT   (1 << 1)  /*             ENET_REF_CLK is output driven by ENET1_CLK_ROOT */
#define GPR_GPR4_ENET_TIME_SEL            (1 << 2)  /* Bit 2:      ENET master timer source select (ENET_TIME_SEL) */
#  define GPR_GPR4_ENET_TIME_SEL_1G       (0 << 2)  /*             Master counter value is from ENET_1G module */
#  define GPR_GPR4_ENET_TIME_SEL_QOS      (1 << 2)  /*             Master counter value is from ENET_QOS module */
#define GPR_GPR4_ENET_EVENT0IN_SEL        (1 << 3)  /* Bit 3:      ENET_1588_EVENT0_IN source select (ENET_EVENT0IN_SEL) */
#  define GPR_GPR4_ENET_EVENT0IN_SEL_PAD  (0 << 3)  /*             ENET_1588_EVENT0_IN input is from pad */
#  define GPR_GPR4_ENET_EVENT0IN_SEL_GPT2 (1 << 3)  /*             ENET_1588_EVENT0_IN input is from GPT2 COMPARE1 output */
                                                    /* Bits 4-15:  Reserved */
                                                    /* Bits 16-27: Reserved */
#define GPR_GPR4_DWP_SHIFT                (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR4_DWP_MASK                 (0x03 << GPR_GPR4_DWP_SHIFT)
#  define GPR_GPR4_DWP_ALLOWBOTH          (0x00 << GPR_GPR4_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR4_DWP_DENYCM7            (0x01 << GPR_GPR4_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR4_DWP_DENYCM4            (0x02 << GPR_GPR4_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR4_DWP_DENYBOTH           (0x03 << GPR_GPR4_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR4_DWP_LOCK_SHIFT           (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR4_DWP_LOCK_MASK            (0x03 << GPR_GPR4_DWP_LOCK_SHIFT)
#  define GPR_GPR4_DWP_LOCK_NOLCK         (0x00 << GPR_GPR4_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR4_DWP_LOCK_LCKLO         (0x01 << GPR_GPR4_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR4_DWP_LOCK_LCKHI         (0x02 << GPR_GPR4_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR4_DWP_LOCK_LCKBOTH       (0x03 << GPR_GPR4_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 5 (GPR5) */

#define GPR_GPR5_ENET1G_TX_CLK_SEL        (1 << 0)  /* Bit 0:      ENET1G TX_CLK select (ENET1G_TX_CLK_SEL) */
#  define GPR_GPR5_ENET1G_TX_CLK_SEL_CLK  (0 << 0)  /*             ENET1G TX_CLK is driven by ENET2_CLK_ROOT */
#  define GPR_GPR5_ENET1G_TX_CLK_SEL_PAD  (1 << 0)  /*             ENET1G TX_CLK is from pad */
#define GPR_GPR5_ENET1G_REF_CLK_DIR       (1 << 1)  /* Bit 1:      ENET1G_REF_CLK direction control (ENET1G_REF_CLK_DIR) */
#  define GPR_GPR5_ENET1G_REF_CLK_DIR_IN  (0 << 1)  /*             ENET1G_REF_CLK is input */
#  define GPR_GPR5_ENET1G_REF_CLK_DIR_OUT (1 << 1)  /*             ENET1G_REF_CLK is output driven by ENET2_CLK_ROOT */
#define GPR_GPR5_ENET1G_RGMII_EN          (1 << 2)  /* Bit 2:      ENET1G RGMII TX clock output enable (ENET1G_RGMII_EN) */
#define GPR_GPR5_ENET1G_TIME_SEL          (1 << 3)  /* Bit 3:      ENET1G master timer source select (ENET1G_TIME_SEL) */
#  define GPR_GPR5_ENET1G_TIME_SEL_ENET   (0 << 3)  /*             Master counter value is from ENET module */
#  define GPR_GPR5_ENET1G_TIME_SEL_QOS    (1 << 3)  /*             Master counter value is from ENET_QOS module */

#define GPR_GPR5_ENET1G_EVENT0IN_SEL        (1 << 4) /* Bit 4:     ENET_1588_EVENT0_IN source select (ENET1G_EVENT0IN_SEL) */
#  define GPR_GPR5_ENET1G_EVENT0IN_SEL_PAD  (0 << 4) /*            ENET_1588_EVENT0_IN input is from pad */
#  define GPR_GPR5_ENET1G_EVENT0IN_SEL_GPT2 (1 << 4) /*            ENET_1588_EVENT0_IN input is from GPT2 COMPARE1 output */

                                                    /* Bits 5-15:  Reserved */

                                                    /* Bits 16-27: Reserved */

#define GPR_GPR5_DWP_SHIFT                (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR5_DWP_MASK                 (0x03 << GPR_GPR5_DWP_SHIFT)
#  define GPR_GPR5_DWP_ALLOWBOTH          (0x00 << GPR_GPR5_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR5_DWP_DENYCM7            (0x01 << GPR_GPR5_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR5_DWP_DENYCM4            (0x02 << GPR_GPR5_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR5_DWP_DENYBOTH           (0x03 << GPR_GPR5_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR5_DWP_LOCK_SHIFT           (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR5_DWP_LOCK_MASK            (0x03 << GPR_GPR5_DWP_LOCK_SHIFT)
#  define GPR_GPR5_DWP_LOCK_NOLCK         (0x00 << GPR_GPR5_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR5_DWP_LOCK_LCKLO         (0x01 << GPR_GPR5_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR5_DWP_LOCK_LCKHI         (0x02 << GPR_GPR5_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR5_DWP_LOCK_LCKBOTH       (0x03 << GPR_GPR5_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 6 (GPR6) */

#define GPR_GPR6_ENET_QOS_REF_CLK_DIR       (1 << 0) /* Bit 0:     ENET_QOS_REF_CLK direction control (ENET_QOS_REF_CLK_DIR) */
#  define GPR_GPR6_ENET_QOS_REF_CLK_DIR_IN  (0 << 0) /*            ENET_QOS_REF_CLK is input */
#  define GPR_GPR6_ENET_QOS_REF_CLK_DIR_OUT (1 << 0) /*            ENET_QOS_REF_CLK is output driven by ENET_QOS_CLK_ROOT */

#define GPR_GPR6_ENET_QOS_RGMII_EN        (1 << 1)  /* Bit 1:      ENET_QOS RGMII TX clock output enable (ENET_QOS_RGMII_EN) */
#define GPR_GPR6_ENET_QOS_TIME_SEL        (1 << 2)  /* Bit 2:      ENET_QOS master timer source select (ENET_QOS_TIME_SEL) */
#  define GPR_GPR6_ENET_QOS_TIME_SEL_ENET (0 << 2)  /*             Master counter value is from ENET module */
#  define GPR_GPR6_ENET_QOS_TIME_SEL_1G   (1 << 2)  /*             Master counter value is from ENET_1G module */

#define GPR_GPR6_ENET_QOS_INTF_SEL_SHIFT   (3)      /* Bits 3-5:   ENET_QOS PHY interface select (ENET_QOS_INTF_SEL) */
#define GPR_GPR6_ENET_QOS_INTF_SEL_MASK    (0x07 << GPR_GPR6_ENET_QOS_INTF_SEL_SHIFT)
#  define GPR_GPR6_ENET_QOS_INTF_SEL_MII   (0x00 << GPR_GPR6_ENET_QOS_INTF_SEL_SHIFT) /* MII */
#  define GPR_GPR6_ENET_QOS_INTF_SEL_RGMII (0x01 << GPR_GPR6_ENET_QOS_INTF_SEL_SHIFT) /* RGMII */
#  define GPR_GPR6_ENET_QOS_INTF_SEL_RMII  (0x04 << GPR_GPR6_ENET_QOS_INTF_SEL_SHIFT) /* RMII */

#define GPR_GPR6_ENET_QOS_CLKGEN_EN       (1 << 6)  /* Bit 6:      ENET_QOS clock generator enable (ENET_QOS_CLKGEN_EN) */

#define GPR_GPR6_ENET_QOS_EVENT0IN_SEL        (1 << 7) /* Bit 7:   ENET_1588_EVENT0_IN source select (ENET_QOS_EVENT0IN_SEL) */
#  define GPR_GPR6_ENET_QOS_EVENT0IN_SEL_PAD  (0 << 7) /*          ENET_1588_EVENT0_IN input is from pad */
#  define GPR_GPR6_ENET_QOS_EVENT0IN_SEL_GPT3 (1 << 7) /*          ENET_1588_EVENT0_IN input is from GPT3 COMPARE1 output */

                                                    /* Bits 8-15:  Reserved */

                                                    /* Bits 16-27: Reserved */

#define GPR_GPR6_DWP_SHIFT                (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR6_DWP_MASK                 (0x03 << GPR_GPR6_DWP_SHIFT)
#  define GPR_GPR6_DWP_ALLOWBOTH          (0x00 << GPR_GPR6_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR6_DWP_DENYCM7            (0x01 << GPR_GPR6_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR6_DWP_DENYCM4            (0x02 << GPR_GPR6_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR6_DWP_DENYBOTH           (0x03 << GPR_GPR6_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR6_DWP_LOCK_SHIFT           (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR6_DWP_LOCK_MASK            (0x03 << GPR_GPR6_DWP_LOCK_SHIFT)
#  define GPR_GPR6_DWP_LOCK_NOLCK         (0x00 << GPR_GPR6_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR6_DWP_LOCK_LCKLO         (0x01 << GPR_GPR6_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR6_DWP_LOCK_LCKHI         (0x02 << GPR_GPR6_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR6_DWP_LOCK_LCKBOTH       (0x03 << GPR_GPR6_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 7 (GPR7) */

#define GPR_GPR7_GINT                     (1 << 0)  /* Bit 0:      Global interrupt (GINT) */
                                                    /* Bits 1-15:  Reserved */
                                                    /* Bits 16-27: Reserved */
#define GPR_GPR7_DWP_SHIFT                (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR7_DWP_MASK                 (0x03 << GPR_GPR7_DWP_SHIFT)
#  define GPR_GPR7_DWP_ALLOWBOTH          (0x00 << GPR_GPR7_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR7_DWP_DENYCM7            (0x01 << GPR_GPR7_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR7_DWP_DENYCM4            (0x02 << GPR_GPR7_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR7_DWP_DENYBOTH           (0x03 << GPR_GPR7_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR7_DWP_LOCK_SHIFT           (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR7_DWP_LOCK_MASK            (0x03 << GPR_GPR7_DWP_LOCK_SHIFT)
#  define GPR_GPR7_DWP_LOCK_NOLCK         (0x00 << GPR_GPR7_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR7_DWP_LOCK_LCKLO         (0x01 << GPR_GPR7_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR7_DWP_LOCK_LCKHI         (0x02 << GPR_GPR7_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR7_DWP_LOCK_LCKBOTH       (0x03 << GPR_GPR7_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 8 (GPR8) */

#define GPR_GPR8_WDOG1_MASK               (1 << 0)  /* Bit 0:      WDOG1 timeout mask for WDOG_ANY (WDOG1_MASK) */
                                                    /* Bits 1-15:  Reserved */
                                                    /* Bits 16-27: Reserved */
#define GPR_GPR8_DWP_SHIFT                (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR8_DWP_MASK                 (0x03 << GPR_GPR8_DWP_SHIFT)
#  define GPR_GPR8_DWP_ALLOWBOTH          (0x00 << GPR_GPR8_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR8_DWP_DENYCM7            (0x01 << GPR_GPR8_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR8_DWP_DENYCM4            (0x02 << GPR_GPR8_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR8_DWP_DENYBOTH           (0x03 << GPR_GPR8_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR8_DWP_LOCK_SHIFT           (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR8_DWP_LOCK_MASK            (0x03 << GPR_GPR8_DWP_LOCK_SHIFT)
#  define GPR_GPR8_DWP_LOCK_NOLCK         (0x00 << GPR_GPR8_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR8_DWP_LOCK_LCKLO         (0x01 << GPR_GPR8_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR8_DWP_LOCK_LCKHI         (0x02 << GPR_GPR8_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR8_DWP_LOCK_LCKBOTH       (0x03 << GPR_GPR8_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 9 (GPR9) */

#define GPR_GPR9_WDOG2_MASK               (1 << 0)  /* Bit 0:      WDOG2 timeout mask for WDOG_ANY (WDOG2_MASK) */
                                                    /* Bits 1-15:  Reserved */
                                                    /* Bits 16-27: Reserved */
#define GPR_GPR9_DWP_SHIFT                (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR9_DWP_MASK                 (0x03 << GPR_GPR9_DWP_SHIFT)
#  define GPR_GPR9_DWP_ALLOWBOTH          (0x00 << GPR_GPR9_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR9_DWP_DENYCM7            (0x01 << GPR_GPR9_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR9_DWP_DENYCM4            (0x02 << GPR_GPR9_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR9_DWP_DENYBOTH           (0x03 << GPR_GPR9_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR9_DWP_LOCK_SHIFT           (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR9_DWP_LOCK_MASK            (0x03 << GPR_GPR9_DWP_LOCK_SHIFT)
#  define GPR_GPR9_DWP_LOCK_NOLCK         (0x00 << GPR_GPR9_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR9_DWP_LOCK_LCKLO         (0x01 << GPR_GPR9_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR9_DWP_LOCK_LCKHI         (0x02 << GPR_GPR9_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR9_DWP_LOCK_LCKBOTH       (0x03 << GPR_GPR9_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 10 (GPR10) - Reserved */

/* General Purpose Register 11 (GPR11) - Reserved */

/* General Purpose Register 12 (GPR12) */

#define GPR_GPR12_QTIMER1_TMR_CNTS_FREEZE (1 << 0)  /* Bit 0:      QTIMER1 timer counter freeze (QTIMER1_TMR_CNTS_FREEZE) */
                                                    /* Bits 1-7:   Reserved */

#define GPR_GPR12_QTIMER1_TMR0_INPUT_SEL         (1 << 8)  /* Bit 8:  QTIMER1 TMR0 input select */
#  define GPR_GPR12_QTIMER1_TMR0_INPUT_SEL_IOMUX (0 << 8)  /*         input from IOMUX */
#  define GPR_GPR12_QTIMER1_TMR0_INPUT_SEL_XBAR  (1 << 8)  /*         input from XBAR */
#define GPR_GPR12_QTIMER1_TMR1_INPUT_SEL         (1 << 9)  /* Bit 9:  QTIMER1 TMR1 input select */
#  define GPR_GPR12_QTIMER1_TMR1_INPUT_SEL_IOMUX (0 << 9)  /*         input from IOMUX */
#  define GPR_GPR12_QTIMER1_TMR1_INPUT_SEL_XBAR  (1 << 9)  /*         input from XBAR */
#define GPR_GPR12_QTIMER1_TMR2_INPUT_SEL         (1 << 10) /* Bit 10: QTIMER1 TMR2 input select */
#  define GPR_GPR12_QTIMER1_TMR2_INPUT_SEL_IOMUX (0 << 10) /*         input from IOMUX */
#  define GPR_GPR12_QTIMER1_TMR2_INPUT_SEL_XBAR  (1 << 10) /*         input from XBAR */
#define GPR_GPR12_QTIMER1_TMR3_INPUT_SEL         (1 << 11) /* Bit 11: QTIMER1 TMR3 input select */
#  define GPR_GPR12_QTIMER1_TMR3_INPUT_SEL_IOMUX (0 << 11) /*         input from IOMUX */
#  define GPR_GPR12_QTIMER1_TMR3_INPUT_SEL_XBAR  (1 << 11) /*         input from XBAR */

                                                    /* Bits 12-15: Reserved */

                                                    /* Bits 16-27: Reserved */

#define GPR_GPR12_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR12_DWP_MASK                (0x03 << GPR_GPR12_DWP_SHIFT)
#  define GPR_GPR12_DWP_ALLOWBOTH         (0x00 << GPR_GPR12_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR12_DWP_DENYCM7           (0x01 << GPR_GPR12_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR12_DWP_DENYCM4           (0x02 << GPR_GPR12_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR12_DWP_DENYBOTH          (0x03 << GPR_GPR12_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR12_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR12_DWP_LOCK_MASK           (0x03 << GPR_GPR12_DWP_LOCK_SHIFT)
#  define GPR_GPR12_DWP_LOCK_NOLCK        (0x00 << GPR_GPR12_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR12_DWP_LOCK_LCKLO        (0x01 << GPR_GPR12_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR12_DWP_LOCK_LCKHI        (0x02 << GPR_GPR12_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR12_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR12_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 13 (GPR13) */

#define GPR_GPR13_QTIMER2_TMR_CNTS_FREEZE (1 << 0)  /* Bit 0:      QTIMER2 timer counter freeze (QTIMER2_TMR_CNTS_FREEZE) */
                                                    /* Bits 1-7:   Reserved */

#define GPR_GPR13_QTIMER2_TMR0_INPUT_SEL         (1 << 8)  /* Bit 8:  QTIMER2 TMR0 input select */
#  define GPR_GPR13_QTIMER2_TMR0_INPUT_SEL_IOMUX (0 << 8)  /*         input from IOMUX */
#  define GPR_GPR13_QTIMER2_TMR0_INPUT_SEL_XBAR  (1 << 8)  /*         input from XBAR */
#define GPR_GPR13_QTIMER2_TMR1_INPUT_SEL         (1 << 9)  /* Bit 9:  QTIMER2 TMR1 input select */
#  define GPR_GPR13_QTIMER2_TMR1_INPUT_SEL_IOMUX (0 << 9)  /*         input from IOMUX */
#  define GPR_GPR13_QTIMER2_TMR1_INPUT_SEL_XBAR  (1 << 9)  /*         input from XBAR */
#define GPR_GPR13_QTIMER2_TMR2_INPUT_SEL         (1 << 10) /* Bit 10: QTIMER2 TMR2 input select */
#  define GPR_GPR13_QTIMER2_TMR2_INPUT_SEL_IOMUX (0 << 10) /*         input from IOMUX */
#  define GPR_GPR13_QTIMER2_TMR2_INPUT_SEL_XBAR  (1 << 10) /*         input from XBAR */
#define GPR_GPR13_QTIMER2_TMR3_INPUT_SEL         (1 << 11) /* Bit 11: QTIMER2 TMR3 input select */
#  define GPR_GPR13_QTIMER2_TMR3_INPUT_SEL_IOMUX (0 << 11) /*         input from IOMUX */
#  define GPR_GPR13_QTIMER2_TMR3_INPUT_SEL_XBAR  (1 << 11) /*         input from XBAR */

                                                    /* Bits 12-15: Reserved */

                                                    /* Bits 16-27: Reserved */

#define GPR_GPR13_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR13_DWP_MASK                (0x03 << GPR_GPR13_DWP_SHIFT)
#  define GPR_GPR13_DWP_ALLOWBOTH         (0x00 << GPR_GPR13_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR13_DWP_DENYCM7           (0x01 << GPR_GPR13_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR13_DWP_DENYCM4           (0x02 << GPR_GPR13_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR13_DWP_DENYBOTH          (0x03 << GPR_GPR13_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR13_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR13_DWP_LOCK_MASK           (0x03 << GPR_GPR13_DWP_LOCK_SHIFT)
#  define GPR_GPR13_DWP_LOCK_NOLCK        (0x00 << GPR_GPR13_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR13_DWP_LOCK_LCKLO        (0x01 << GPR_GPR13_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR13_DWP_LOCK_LCKHI        (0x02 << GPR_GPR13_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR13_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR13_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 14 (GPR14) */

#define GPR_GPR14_QTIMER3_TMR_CNTS_FREEZE (1 << 0)  /* Bit 0:      QTIMER3 timer counter freeze (QTIMER3_TMR_CNTS_FREEZE) */
                                                    /* Bits 1-7:   Reserved */

#define GPR_GPR14_QTIMER3_TMR0_INPUT_SEL         (1 << 8)  /* Bit 8:  QTIMER3 TMR0 input select */
#  define GPR_GPR14_QTIMER3_TMR0_INPUT_SEL_IOMUX (0 << 8)  /*         input from IOMUX */
#  define GPR_GPR14_QTIMER3_TMR0_INPUT_SEL_XBAR  (1 << 8)  /*         input from XBAR */
#define GPR_GPR14_QTIMER3_TMR1_INPUT_SEL         (1 << 9)  /* Bit 9:  QTIMER3 TMR1 input select */
#  define GPR_GPR14_QTIMER3_TMR1_INPUT_SEL_IOMUX (0 << 9)  /*         input from IOMUX */
#  define GPR_GPR14_QTIMER3_TMR1_INPUT_SEL_XBAR  (1 << 9)  /*         input from XBAR */
#define GPR_GPR14_QTIMER3_TMR2_INPUT_SEL         (1 << 10) /* Bit 10: QTIMER3 TMR2 input select */
#  define GPR_GPR14_QTIMER3_TMR2_INPUT_SEL_IOMUX (0 << 10) /*         input from IOMUX */
#  define GPR_GPR14_QTIMER3_TMR2_INPUT_SEL_XBAR  (1 << 10) /*         input from XBAR */
#define GPR_GPR14_QTIMER3_TMR3_INPUT_SEL         (1 << 11) /* Bit 11: QTIMER3 TMR3 input select */
#  define GPR_GPR14_QTIMER3_TMR3_INPUT_SEL_IOMUX (0 << 11) /*         input from IOMUX */
#  define GPR_GPR14_QTIMER3_TMR3_INPUT_SEL_XBAR  (1 << 11) /*         input from XBAR */

                                                    /* Bits 12-15: Reserved */

                                                    /* Bits 16-27: Reserved */

#define GPR_GPR14_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR14_DWP_MASK                (0x03 << GPR_GPR14_DWP_SHIFT)
#  define GPR_GPR14_DWP_ALLOWBOTH         (0x00 << GPR_GPR14_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR14_DWP_DENYCM7           (0x01 << GPR_GPR14_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR14_DWP_DENYCM4           (0x02 << GPR_GPR14_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR14_DWP_DENYBOTH          (0x03 << GPR_GPR14_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR14_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR14_DWP_LOCK_MASK           (0x03 << GPR_GPR14_DWP_LOCK_SHIFT)
#  define GPR_GPR14_DWP_LOCK_NOLCK        (0x00 << GPR_GPR14_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR14_DWP_LOCK_LCKLO        (0x01 << GPR_GPR14_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR14_DWP_LOCK_LCKHI        (0x02 << GPR_GPR14_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR14_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR14_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 15 (GPR15) */

#define GPR_GPR15_QTIMER4_TMR_CNTS_FREEZE (1 << 0)  /* Bit 0:      QTIMER4 timer counter freeze (QTIMER4_TMR_CNTS_FREEZE) */
                                                    /* Bits 1-7:   Reserved */

#define GPR_GPR15_QTIMER4_TMR0_INPUT_SEL         (1 << 8)  /* Bit 8:  QTIMER4 TMR0 input select */
#  define GPR_GPR15_QTIMER4_TMR0_INPUT_SEL_IOMUX (0 << 8)  /*         input from IOMUX */
#  define GPR_GPR15_QTIMER4_TMR0_INPUT_SEL_XBAR  (1 << 8)  /*         input from XBAR */
#define GPR_GPR15_QTIMER4_TMR1_INPUT_SEL         (1 << 9)  /* Bit 9:  QTIMER4 TMR1 input select */
#  define GPR_GPR15_QTIMER4_TMR1_INPUT_SEL_IOMUX (0 << 9)  /*         input from IOMUX */
#  define GPR_GPR15_QTIMER4_TMR1_INPUT_SEL_XBAR  (1 << 9)  /*         input from XBAR */
#define GPR_GPR15_QTIMER4_TMR2_INPUT_SEL         (1 << 10) /* Bit 10: QTIMER4 TMR2 input select */
#  define GPR_GPR15_QTIMER4_TMR2_INPUT_SEL_IOMUX (0 << 10) /*         input from IOMUX */
#  define GPR_GPR15_QTIMER4_TMR2_INPUT_SEL_XBAR  (1 << 10) /*         input from XBAR */
#define GPR_GPR15_QTIMER4_TMR3_INPUT_SEL         (1 << 11) /* Bit 11: QTIMER4 TMR3 input select */
#  define GPR_GPR15_QTIMER4_TMR3_INPUT_SEL_IOMUX (0 << 11) /*         input from IOMUX */
#  define GPR_GPR15_QTIMER4_TMR3_INPUT_SEL_XBAR  (1 << 11) /*         input from XBAR */

                                                    /* Bits 12-15: Reserved */

                                                    /* Bits 16-27: Reserved */

#define GPR_GPR15_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR15_DWP_MASK                (0x03 << GPR_GPR15_DWP_SHIFT)
#  define GPR_GPR15_DWP_ALLOWBOTH         (0x00 << GPR_GPR15_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR15_DWP_DENYCM7           (0x01 << GPR_GPR15_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR15_DWP_DENYCM4           (0x02 << GPR_GPR15_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR15_DWP_DENYBOTH          (0x03 << GPR_GPR15_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR15_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR15_DWP_LOCK_MASK           (0x03 << GPR_GPR15_DWP_LOCK_SHIFT)
#  define GPR_GPR15_DWP_LOCK_NOLCK        (0x00 << GPR_GPR15_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR15_DWP_LOCK_LCKLO        (0x01 << GPR_GPR15_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR15_DWP_LOCK_LCKHI        (0x02 << GPR_GPR15_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR15_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR15_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 16 (GPR16) */

                                                    /* Bits 0-1:   Reserved */

#define GPR_GPR16_FLEXRAM_BANK_CFG_SEL        (1 << 2) /* Bit 2:   FlexRAM bank config source select (FLEXRAM_BANK_CFG_SEL) */
#  define GPR_GPR16_FLEXRAM_BANK_CFG_SEL_FUSE (0 << 2) /*          Use fuse value to configure */
#  define GPR_GPR16_FLEXRAM_BANK_CFG_SEL_REG  (1 << 2) /*          Use FLEXRAM_BANK_CFG to configure */

                                                    /* Bit 4:      Reserved */

#define GPR_GPR16_M7_GPC_SLEEP_SEL             (1 << 5) /* Bit 5:  CM7 sleep request selection (M7_GPC_SLEEP_SEL) */
#  define GPR_GPR16_M7_GPC_SLEEP_SEL_SLEEPDEEP (0 << 5) /*         CM7 SLEEPDEEP is sent to GPC */
#  define GPR_GPR16_M7_GPC_SLEEP_SEL_SLEEPING  (1 << 5) /*         CM7 SLEEPING is sent to GPC */

                                                    /* Bits 6-7:   Reserved */

                                                    /* Bits 8-15:  Reserved */

                                                    /* Bits 16-27: Reserved */

#define GPR_GPR16_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR16_DWP_MASK                (0x03 << GPR_GPR16_DWP_SHIFT)
#  define GPR_GPR16_DWP_ALLOWBOTH         (0x00 << GPR_GPR16_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR16_DWP_DENYCM7           (0x01 << GPR_GPR16_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR16_DWP_DENYCM4           (0x02 << GPR_GPR16_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR16_DWP_DENYBOTH          (0x03 << GPR_GPR16_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR16_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR16_DWP_LOCK_MASK           (0x03 << GPR_GPR16_DWP_LOCK_SHIFT)
#  define GPR_GPR16_DWP_LOCK_NOLCK        (0x00 << GPR_GPR16_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR16_DWP_LOCK_LCKLO        (0x01 << GPR_GPR16_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR16_DWP_LOCK_LCKHI        (0x02 << GPR_GPR16_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR16_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR16_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 17 (GPR17) */

#define GPR_GPR17_FLEXRAM_BANK_CFG_LOW_SHIFT  (0)   /* Bits 0-15:  FlexRAM bank config value (FLEXRAM_BANK_CFG_LOW) */
#define GPR_GPR17_FLEXRAM_BANK_CFG_LOW_MASK   (0xffffffff << GPR_GPR17_FLEXRAM_BANK_CFG_LOW_SHIFT)
#define GPR_GPR17_FLEXRAM_BANK_CFG_LOW(n)     (((uint32_t)(n)) << GPR_GPR17_FLEXRAM_BANK_CFG_LOW_SHIFT)
#define GPR_GPR17_FLEXRAM_BANK0_CFG_SHIFT     (0)   /* Bits 0-1:   FlexRAM bank 0 config value */
#define GPR_GPR17_FLEXRAM_BANK0_CFG_MASK      (0x03 << GPR_GPR17_FLEXRAM_BANK0_CFG_SHIFT)
#  define GPR_GPR17_FLEXRAM_BANK0_CFG_NOTUSED (0x00 << GPR_GPR17_FLEXRAM_BANK0_CFG_SHIFT) /* RAM bank 0 is not used */
#  define GPR_GPR17_FLEXRAM_BANK0_CFG_OCRAM   (0x01 << GPR_GPR17_FLEXRAM_BANK0_CFG_SHIFT) /* RAM bank 0 is OCRAM */
#  define GPR_GPR17_FLEXRAM_BANK0_CFG_DTCM    (0x02 << GPR_GPR17_FLEXRAM_BANK0_CFG_SHIFT) /* RAM bank 0 is DTCM */
#  define GPR_GPR17_FLEXRAM_BANK0_CFG_ITCM    (0x03 << GPR_GPR17_FLEXRAM_BANK0_CFG_SHIFT) /* RAM bank 0 is ITCM */

#define GPR_GPR17_FLEXRAM_BANK1_CFG_SHIFT     (2)   /* Bits 2-3:   FlexRAM bank 1 config value */
#define GPR_GPR17_FLEXRAM_BANK1_CFG_MASK      (0x03 << GPR_GPR17_FLEXRAM_BANK1_CFG_SHIFT)
#  define GPR_GPR17_FLEXRAM_BANK1_CFG_NOTUSED (0x00 << GPR_GPR17_FLEXRAM_BANK1_CFG_SHIFT) /* RAM bank 1 is not used */
#  define GPR_GPR17_FLEXRAM_BANK1_CFG_OCRAM   (0x01 << GPR_GPR17_FLEXRAM_BANK1_CFG_SHIFT) /* RAM bank 1 is OCRAM */
#  define GPR_GPR17_FLEXRAM_BANK1_CFG_DTCM    (0x02 << GPR_GPR17_FLEXRAM_BANK1_CFG_SHIFT) /* RAM bank 1 is DTCM */
#  define GPR_GPR17_FLEXRAM_BANK1_CFG_ITCM    (0x03 << GPR_GPR17_FLEXRAM_BANK1_CFG_SHIFT) /* RAM bank 1 is ITCM */

#define GPR_GPR17_FLEXRAM_BANK2_CFG_SHIFT     (4)   /* Bits 4-5:   FlexRAM bank 2 config value */
#define GPR_GPR17_FLEXRAM_BANK2_CFG_MASK      (0x03 << GPR_GPR17_FLEXRAM_BANK2_CFG_SHIFT)
#  define GPR_GPR17_FLEXRAM_BANK2_CFG_NOTUSED (0x00 << GPR_GPR17_FLEXRAM_BANK2_CFG_SHIFT) /* RAM bank 2 is not used */
#  define GPR_GPR17_FLEXRAM_BANK2_CFG_OCRAM   (0x01 << GPR_GPR17_FLEXRAM_BANK2_CFG_SHIFT) /* RAM bank 2 is OCRAM */
#  define GPR_GPR17_FLEXRAM_BANK2_CFG_DTCM    (0x02 << GPR_GPR17_FLEXRAM_BANK2_CFG_SHIFT) /* RAM bank 2 is DTCM */
#  define GPR_GPR17_FLEXRAM_BANK2_CFG_ITCM    (0x03 << GPR_GPR17_FLEXRAM_BANK2_CFG_SHIFT) /* RAM bank 2 is ITCM */

#define GPR_GPR17_FLEXRAM_BANK3_CFG_SHIFT     (6)   /* Bits 6-7:   FlexRAM bank 3 config value */
#define GPR_GPR17_FLEXRAM_BANK3_CFG_MASK      (0x03 << GPR_GPR17_FLEXRAM_BANK3_CFG_SHIFT)
#  define GPR_GPR17_FLEXRAM_BANK3_CFG_NOTUSED (0x00 << GPR_GPR17_FLEXRAM_BANK3_CFG_SHIFT) /* RAM bank 3 is not used */
#  define GPR_GPR17_FLEXRAM_BANK3_CFG_OCRAM   (0x01 << GPR_GPR17_FLEXRAM_BANK3_CFG_SHIFT) /* RAM bank 3 is OCRAM */
#  define GPR_GPR17_FLEXRAM_BANK3_CFG_DTCM    (0x02 << GPR_GPR17_FLEXRAM_BANK3_CFG_SHIFT) /* RAM bank 3 is DTCM */
#  define GPR_GPR17_FLEXRAM_BANK3_CFG_ITCM    (0x03 << GPR_GPR17_FLEXRAM_BANK3_CFG_SHIFT) /* RAM bank 3 is ITCM */

#define GPR_GPR17_FLEXRAM_BANK4_CFG_SHIFT     (8)   /* Bits 8-9:   FlexRAM bank 4 config value */
#define GPR_GPR17_FLEXRAM_BANK4_CFG_MASK      (0x03 << GPR_GPR17_FLEXRAM_BANK4_CFG_SHIFT)
#  define GPR_GPR17_FLEXRAM_BANK4_CFG_NOTUSED (0x00 << GPR_GPR17_FLEXRAM_BANK4_CFG_SHIFT) /* RAM bank 4 is not used */
#  define GPR_GPR17_FLEXRAM_BANK4_CFG_OCRAM   (0x01 << GPR_GPR17_FLEXRAM_BANK4_CFG_SHIFT) /* RAM bank 4 is OCRAM */
#  define GPR_GPR17_FLEXRAM_BANK4_CFG_DTCM    (0x02 << GPR_GPR17_FLEXRAM_BANK4_CFG_SHIFT) /* RAM bank 4 is DTCM */
#  define GPR_GPR17_FLEXRAM_BANK4_CFG_ITCM    (0x03 << GPR_GPR17_FLEXRAM_BANK4_CFG_SHIFT) /* RAM bank 4 is ITCM */

#define GPR_GPR17_FLEXRAM_BANK5_CFG_SHIFT     (10)  /* Bits 10-11: FlexRAM bank 5 config value */
#define GPR_GPR17_FLEXRAM_BANK5_CFG_MASK      (0x03 << GPR_GPR17_FLEXRAM_BANK5_CFG_SHIFT)
#  define GPR_GPR17_FLEXRAM_BANK5_CFG_NOTUSED (0x00 << GPR_GPR17_FLEXRAM_BANK5_CFG_SHIFT) /* RAM bank 5 is not used */
#  define GPR_GPR17_FLEXRAM_BANK5_CFG_OCRAM   (0x01 << GPR_GPR17_FLEXRAM_BANK5_CFG_SHIFT) /* RAM bank 5 is OCRAM */
#  define GPR_GPR17_FLEXRAM_BANK5_CFG_DTCM    (0x02 << GPR_GPR17_FLEXRAM_BANK5_CFG_SHIFT) /* RAM bank 5 is DTCM */
#  define GPR_GPR17_FLEXRAM_BANK5_CFG_ITCM    (0x03 << GPR_GPR17_FLEXRAM_BANK5_CFG_SHIFT) /* RAM bank 5 is ITCM */

#define GPR_GPR17_FLEXRAM_BANK6_CFG_SHIFT     (12)  /* Bits 12-13: FlexRAM bank 6 config value */
#define GPR_GPR17_FLEXRAM_BANK6_CFG_MASK      (0x03 << GPR_GPR17_FLEXRAM_BANK6_CFG_SHIFT)
#  define GPR_GPR17_FLEXRAM_BANK6_CFG_NOTUSED (0x00 << GPR_GPR17_FLEXRAM_BANK6_CFG_SHIFT) /* RAM bank 6 is not used */
#  define GPR_GPR17_FLEXRAM_BANK6_CFG_OCRAM   (0x01 << GPR_GPR17_FLEXRAM_BANK6_CFG_SHIFT) /* RAM bank 6 is OCRAM */
#  define GPR_GPR17_FLEXRAM_BANK6_CFG_DTCM    (0x02 << GPR_GPR17_FLEXRAM_BANK6_CFG_SHIFT) /* RAM bank 6 is DTCM */
#  define GPR_GPR17_FLEXRAM_BANK6_CFG_ITCM    (0x03 << GPR_GPR17_FLEXRAM_BANK6_CFG_SHIFT) /* RAM bank 6 is ITCM */

#define GPR_GPR17_FLEXRAM_BANK7_CFG_SHIFT     (14)  /* Bits 14-15: FlexRAM bank 7 config value */
#define GPR_GPR17_FLEXRAM_BANK7_CFG_MASK      (0x03 << GPR_GPR17_FLEXRAM_BANK7_CFG_SHIFT)
#  define GPR_GPR17_FLEXRAM_BANK7_CFG_NOTUSED (0x00 << GPR_GPR17_FLEXRAM_BANK7_CFG_SHIFT) /* RAM bank 7 is not used */
#  define GPR_GPR17_FLEXRAM_BANK7_CFG_OCRAM   (0x01 << GPR_GPR17_FLEXRAM_BANK7_CFG_SHIFT) /* RAM bank 7 is OCRAM */
#  define GPR_GPR17_FLEXRAM_BANK7_CFG_DTCM    (0x02 << GPR_GPR17_FLEXRAM_BANK7_CFG_SHIFT) /* RAM bank 7 is DTCM */
#  define GPR_GPR17_FLEXRAM_BANK7_CFG_ITCM    (0x03 << GPR_GPR17_FLEXRAM_BANK7_CFG_SHIFT) /* RAM bank 7 is ITCM */

                                                    /* Bits 16-27: Reserved */

#define GPR_GPR17_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR17_DWP_MASK                (0x03 << GPR_GPR17_DWP_SHIFT)
#  define GPR_GPR17_DWP_ALLOWBOTH         (0x00 << GPR_GPR17_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR17_DWP_DENYCM7           (0x01 << GPR_GPR17_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR17_DWP_DENYCM4           (0x02 << GPR_GPR17_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR17_DWP_DENYBOTH          (0x03 << GPR_GPR17_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR17_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR17_DWP_LOCK_MASK           (0x03 << GPR_GPR17_DWP_LOCK_SHIFT)
#  define GPR_GPR17_DWP_LOCK_NOLCK        (0x00 << GPR_GPR17_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR17_DWP_LOCK_LCKLO        (0x01 << GPR_GPR17_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR17_DWP_LOCK_LCKHI        (0x02 << GPR_GPR17_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR17_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR17_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 18 (GPR18) */

#define GPR_GPR18_FLEXRAM_BANK_CFG_HIGH_SHIFT  (0)   /* Bits 0-15:  FlexRAM bank config value (FLEXRAM_BANK_CFG_HIGH) */
#define GPR_GPR18_FLEXRAM_BANK_CFG_HIGH_MASK   (0xffff << GPR_GPR18_FLEXRAM_BANK_CFG_HIGH_SHIFT)
#define GPR_GPR18_FLEXRAM_BANK_CFG_HIGH(n)     (((uint32_t)(n)) << GPR_GPR18_FLEXRAM_BANK_CFG_HIGH_SHIFT)
#define GPR_GPR18_FLEXRAM_BANK8_CFG_SHIFT      (0)   /* Bits 0-1:   FlexRAM bank 8 config value */
#define GPR_GPR18_FLEXRAM_BANK8_CFG_MASK       (0x03 << GPR_GPR18_FLEXRAM_BANK0_CFG_SHIFT)
#  define GPR_GPR18_FLEXRAM_BANK8_CFG_NOTUSED  (0x00 << GPR_GPR18_FLEXRAM_BANK8_CFG_SHIFT) /* RAM bank 8 is not used */
#  define GPR_GPR18_FLEXRAM_BANK8_CFG_OCRAM    (0x01 << GPR_GPR18_FLEXRAM_BANK8_CFG_SHIFT) /* RAM bank 8 is OCRAM */
#  define GPR_GPR18_FLEXRAM_BANK8_CFG_DTCM     (0x02 << GPR_GPR18_FLEXRAM_BANK8_CFG_SHIFT) /* RAM bank 8 is DTCM */
#  define GPR_GPR18_FLEXRAM_BANK8_CFG_ITCM     (0x03 << GPR_GPR18_FLEXRAM_BANK8_CFG_SHIFT) /* RAM bank 8 is ITCM */

#define GPR_GPR18_FLEXRAM_BANK9_CFG_SHIFT      (2)   /* Bits 2-3:   FlexRAM bank 9 config value */
#define GPR_GPR18_FLEXRAM_BANK9_CFG_MASK       (0x03 << GPR_GPR18_FLEXRAM_BANK1_CFG_SHIFT)
#  define GPR_GPR18_FLEXRAM_BANK9_CFG_NOTUSED  (0x00 << GPR_GPR18_FLEXRAM_BANK9_CFG_SHIFT) /* RAM bank 9 is not used */
#  define GPR_GPR18_FLEXRAM_BANK9_CFG_OCRAM    (0x01 << GPR_GPR18_FLEXRAM_BANK9_CFG_SHIFT) /* RAM bank 9 is OCRAM */
#  define GPR_GPR18_FLEXRAM_BANK9_CFG_DTCM     (0x02 << GPR_GPR18_FLEXRAM_BANK9_CFG_SHIFT) /* RAM bank 9 is DTCM */
#  define GPR_GPR18_FLEXRAM_BANK9_CFG_ITCM     (0x03 << GPR_GPR18_FLEXRAM_BANK9_CFG_SHIFT) /* RAM bank 9 is ITCM */

#define GPR_GPR18_FLEXRAM_BANK10_CFG_SHIFT     (4)   /* Bits 4-5:   FlexRAM bank 10 config value */
#define GPR_GPR18_FLEXRAM_BANK10_CFG_MASK      (0x03 << GPR_GPR18_FLEXRAM_BANK10_CFG_SHIFT)
#  define GPR_GPR18_FLEXRAM_BANK10_CFG_NOTUSED (0x00 << GPR_GPR18_FLEXRAM_BANK10_CFG_SHIFT) /* RAM bank 10 is not used */
#  define GPR_GPR18_FLEXRAM_BANK10_CFG_OCRAM   (0x01 << GPR_GPR18_FLEXRAM_BANK10_CFG_SHIFT) /* RAM bank 10 is OCRAM */
#  define GPR_GPR18_FLEXRAM_BANK10_CFG_DTCM    (0x02 << GPR_GPR18_FLEXRAM_BANK10_CFG_SHIFT) /* RAM bank 10 is DTCM */
#  define GPR_GPR18_FLEXRAM_BANK10_CFG_ITCM    (0x03 << GPR_GPR18_FLEXRAM_BANK10_CFG_SHIFT) /* RAM bank 10 is ITCM */

#define GPR_GPR18_FLEXRAM_BANK11_CFG_SHIFT     (6)   /* Bits 6-7:   FlexRAM bank 11 config value */
#define GPR_GPR18_FLEXRAM_BANK11_CFG_MASK      (0x03 << GPR_GPR18_FLEXRAM_BANK11_CFG_SHIFT)
#  define GPR_GPR18_FLEXRAM_BANK11_CFG_NOTUSED (0x00 << GPR_GPR18_FLEXRAM_BANK11_CFG_SHIFT) /* RAM bank 11 is not used */
#  define GPR_GPR18_FLEXRAM_BANK11_CFG_OCRAM   (0x01 << GPR_GPR18_FLEXRAM_BANK11_CFG_SHIFT) /* RAM bank 11 is OCRAM */
#  define GPR_GPR18_FLEXRAM_BANK11_CFG_DTCM    (0x02 << GPR_GPR18_FLEXRAM_BANK11_CFG_SHIFT) /* RAM bank 11 is DTCM */
#  define GPR_GPR18_FLEXRAM_BANK11_CFG_ITCM    (0x03 << GPR_GPR18_FLEXRAM_BANK11_CFG_SHIFT) /* RAM bank 11 is ITCM */

#define GPR_GPR18_FLEXRAM_BANK12_CFG_SHIFT     (8)   /* Bits 8-9:   FlexRAM bank 12 config value */
#define GPR_GPR18_FLEXRAM_BANK12_CFG_MASK      (0x03 << GPR_GPR18_FLEXRAM_BANK12_CFG_SHIFT)
#  define GPR_GPR18_FLEXRAM_BANK12_CFG_NOTUSED (0x00 << GPR_GPR18_FLEXRAM_BANK12_CFG_SHIFT) /* RAM bank 12 is not used */
#  define GPR_GPR18_FLEXRAM_BANK12_CFG_OCRAM   (0x01 << GPR_GPR18_FLEXRAM_BANK12_CFG_SHIFT) /* RAM bank 12 is OCRAM */
#  define GPR_GPR18_FLEXRAM_BANK12_CFG_DTCM    (0x02 << GPR_GPR18_FLEXRAM_BANK12_CFG_SHIFT) /* RAM bank 12 is DTCM */
#  define GPR_GPR18_FLEXRAM_BANK12_CFG_ITCM    (0x03 << GPR_GPR18_FLEXRAM_BANK12_CFG_SHIFT) /* RAM bank 12 is ITCM */

#define GPR_GPR18_FLEXRAM_BANK13_CFG_SHIFT     (10)  /* Bits 10-11: FlexRAM bank 13 config value */
#define GPR_GPR18_FLEXRAM_BANK13_CFG_MASK      (0x03 << GPR_GPR18_FLEXRAM_BANK13_CFG_SHIFT)
#  define GPR_GPR18_FLEXRAM_BANK13_CFG_NOTUSED (0x00 << GPR_GPR18_FLEXRAM_BANK13_CFG_SHIFT) /* RAM bank 13 is not used */
#  define GPR_GPR18_FLEXRAM_BANK13_CFG_OCRAM   (0x01 << GPR_GPR18_FLEXRAM_BANK13_CFG_SHIFT) /* RAM bank 13 is OCRAM */
#  define GPR_GPR18_FLEXRAM_BANK13_CFG_DTCM    (0x02 << GPR_GPR18_FLEXRAM_BANK13_CFG_SHIFT) /* RAM bank 13 is DTCM */
#  define GPR_GPR18_FLEXRAM_BANK13_CFG_ITCM    (0x03 << GPR_GPR18_FLEXRAM_BANK13_CFG_SHIFT) /* RAM bank 13 is ITCM */

#define GPR_GPR18_FLEXRAM_BANK14_CFG_SHIFT     (12)  /* Bits 12-13: FlexRAM bank 14 config value */
#define GPR_GPR18_FLEXRAM_BANK14_CFG_MASK      (0x03 << GPR_GPR18_FLEXRAM_BANK14_CFG_SHIFT)
#  define GPR_GPR18_FLEXRAM_BANK14_CFG_NOTUSED (0x00 << GPR_GPR18_FLEXRAM_BANK14_CFG_SHIFT) /* RAM bank 14 is not used */
#  define GPR_GPR18_FLEXRAM_BANK14_CFG_OCRAM   (0x01 << GPR_GPR18_FLEXRAM_BANK14_CFG_SHIFT) /* RAM bank 14 is OCRAM */
#  define GPR_GPR18_FLEXRAM_BANK14_CFG_DTCM    (0x02 << GPR_GPR18_FLEXRAM_BANK14_CFG_SHIFT) /* RAM bank 14 is DTCM */
#  define GPR_GPR18_FLEXRAM_BANK14_CFG_ITCM    (0x03 << GPR_GPR18_FLEXRAM_BANK14_CFG_SHIFT) /* RAM bank 14 is ITCM */

#define GPR_GPR18_FLEXRAM_BANK15_CFG_SHIFT     (14)  /* Bits 14-15: FlexRAM bank 15 config value */
#define GPR_GPR18_FLEXRAM_BANK15_CFG_MASK      (0x03 << GPR_GPR18_FLEXRAM_BANK15_CFG_SHIFT)
#  define GPR_GPR18_FLEXRAM_BANK15_CFG_NOTUSED (0x00 << GPR_GPR18_FLEXRAM_BANK15_CFG_SHIFT) /* RAM bank 15 is not used */
#  define GPR_GPR18_FLEXRAM_BANK15_CFG_OCRAM   (0x01 << GPR_GPR18_FLEXRAM_BANK15_CFG_SHIFT) /* RAM bank 15 is OCRAM */
#  define GPR_GPR18_FLEXRAM_BANK15_CFG_DTCM    (0x02 << GPR_GPR18_FLEXRAM_BANK15_CFG_SHIFT) /* RAM bank 15 is DTCM */
#  define GPR_GPR18_FLEXRAM_BANK15_CFG_ITCM    (0x03 << GPR_GPR18_FLEXRAM_BANK15_CFG_SHIFT) /* RAM bank 15 is ITCM */

                                                    /* Bits 16-27: Reserved */

#define GPR_GPR18_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR18_DWP_MASK                (0x03 << GPR_GPR18_DWP_SHIFT)
#  define GPR_GPR18_DWP_ALLOWBOTH         (0x00 << GPR_GPR18_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR18_DWP_DENYCM7           (0x01 << GPR_GPR18_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR18_DWP_DENYCM4           (0x02 << GPR_GPR18_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR18_DWP_DENYBOTH          (0x03 << GPR_GPR18_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR18_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR18_DWP_LOCK_MASK           (0x03 << GPR_GPR18_DWP_LOCK_SHIFT)
#  define GPR_GPR18_DWP_LOCK_NOLCK        (0x00 << GPR_GPR18_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR18_DWP_LOCK_LCKLO        (0x01 << GPR_GPR18_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR18_DWP_LOCK_LCKHI        (0x02 << GPR_GPR18_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR18_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR18_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 20 (GPR20) */

#define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_4        (1 << 0)  /* Bit 0:  IOMUXC XBAR_INOUT4 function direction select (IOMUXC_XBAR_DIR_SEL_4) */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_4_IN   (0 << 0)  /*         XBAR_INOUT as input */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_4_OUT  (1 << 0)  /*         XBAR_INOUT as output */
#define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_5        (1 << 1)  /* Bit 1:  IOMUXC XBAR_INOUT5 function direction select (IOMUXC_XBAR_DIR_SEL_5) */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_5_IN   (0 << 1)  /*         XBAR_INOUT as input */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_5_OUT  (1 << 1)  /*         XBAR_INOUT as output */
#define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_6        (1 << 2)  /* Bit 2:  IOMUXC XBAR_INOUT6 function direction select (IOMUXC_XBAR_DIR_SEL_6) */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_6_IN   (0 << 2)  /*         XBAR_INOUT as input */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_6_OUT  (1 << 2)  /*         XBAR_INOUT as output */
#define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_7        (1 << 3)  /* Bit 3:  IOMUXC XBAR_INOUT7 function direction select (IOMUXC_XBAR_DIR_SEL_7) */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_7_IN   (0 << 3)  /*         XBAR_INOUT as input */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_7_OUT  (1 << 3)  /*         XBAR_INOUT as output */
#define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_8        (1 << 4)  /* Bit 4:  IOMUXC XBAR_INOUT8 function direction select (IOMUXC_XBAR_DIR_SEL_8) */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_8_IN   (0 << 4)  /*         XBAR_INOUT as input */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_8_OUT  (1 << 4)  /*         XBAR_INOUT as output */
#define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_9        (1 << 5)  /* Bit 5:  IOMUXC XBAR_INOUT9 function direction select (IOMUXC_XBAR_DIR_SEL_9) */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_9_IN   (0 << 5)  /*         XBAR_INOUT as input */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_9_OUT  (1 << 5)  /*         XBAR_INOUT as output */
#define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_10       (1 << 6)  /* Bit 6:  IOMUXC XBAR_INOUT10 function direction select (IOMUXC_XBAR_DIR_SEL_10) */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_10_IN  (0 << 6)  /*         XBAR_INOUT as input */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_10_OUT (1 << 6)  /*         XBAR_INOUT as output */
#define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_11       (1 << 7)  /* Bit 7:  IOMUXC XBAR_INOUT11 function direction select (IOMUXC_XBAR_DIR_SEL_11) */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_11_IN  (0 << 7)  /*         XBAR_INOUT as input */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_11_OUT (1 << 7)  /*         XBAR_INOUT as output */
#define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_12       (1 << 8)  /* Bit 8:  IOMUXC XBAR_INOUT12 function direction select (IOMUXC_XBAR_DIR_SEL_12) */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_12_IN  (0 << 8)  /*         XBAR_INOUT as input */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_12_OUT (1 << 8)  /*         XBAR_INOUT as output */
#define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_13       (1 << 9)  /* Bit 9:  IOMUXC XBAR_INOUT13 function direction select (IOMUXC_XBAR_DIR_SEL_13) */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_13_IN  (0 << 9)  /*         XBAR_INOUT as input */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_13_OUT (1 << 9)  /*         XBAR_INOUT as output */
#define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_14       (1 << 10) /* Bit 10: IOMUXC XBAR_INOUT14 function direction select (IOMUXC_XBAR_DIR_SEL_14) */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_14_IN  (0 << 10) /*         XBAR_INOUT as input */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_14_OUT (1 << 10) /*         XBAR_INOUT as output */
#define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_15       (1 << 11) /* Bit 11: IOMUXC XBAR_INOUT15 function direction select (IOMUXC_XBAR_DIR_SEL_15) */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_15_IN  (0 << 11) /*         XBAR_INOUT as input */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_15_OUT (1 << 11) /*         XBAR_INOUT as output */
#define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_16       (1 << 12) /* Bit 12: IOMUXC XBAR_INOUT16 function direction select (IOMUXC_XBAR_DIR_SEL_16) */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_16_IN  (0 << 12) /*         XBAR_INOUT as input */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_16_OUT (1 << 12) /*         XBAR_INOUT as output */
#define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_17       (1 << 13) /* Bit 13: IOMUXC XBAR_INOUT17 function direction select (IOMUXC_XBAR_DIR_SEL_17) */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_17_IN  (0 << 13) /*         XBAR_INOUT as input */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_17_OUT (1 << 13) /*         XBAR_INOUT as output */
#define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_18       (1 << 14) /* Bit 14: IOMUXC XBAR_INOUT18 function direction select (IOMUXC_XBAR_DIR_SEL_18) */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_18_IN  (0 << 14) /*         XBAR_INOUT as input */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_18_OUT (1 << 14) /*         XBAR_INOUT as output */
#define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_19       (1 << 15) /* Bit 15: IOMUXC XBAR_INOUT19 function direction select (IOMUXC_XBAR_DIR_SEL_19) */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_19_IN  (0 << 15) /*         XBAR_INOUT as input */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_19_OUT (1 << 15) /*         XBAR_INOUT as output */
#define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_20       (1 << 16) /* Bit 16: IOMUXC XBAR_INOUT20 function direction select (IOMUXC_XBAR_DIR_SEL_20) */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_20_IN  (0 << 16) /*         XBAR_INOUT as input */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_20_OUT (1 << 16) /*         XBAR_INOUT as output */
#define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_21       (1 << 17) /* Bit 17: IOMUXC XBAR_INOUT21 function direction select (IOMUXC_XBAR_DIR_SEL_21) */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_21_IN  (0 << 17) /*         XBAR_INOUT as input */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_21_OUT (1 << 17) /*         XBAR_INOUT as output */
#define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_22       (1 << 18) /* Bit 18: IOMUXC XBAR_INOUT22 function direction select (IOMUXC_XBAR_DIR_SEL_22) */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_22_IN  (0 << 18) /*         XBAR_INOUT as input */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_22_OUT (1 << 18) /*         XBAR_INOUT as output */
#define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_23       (1 << 19) /* Bit 19: IOMUXC XBAR_INOUT23 function direction select (IOMUXC_XBAR_DIR_SEL_23) */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_23_IN  (0 << 19) /*         XBAR_INOUT as input */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_23_OUT (1 << 19) /*         XBAR_INOUT as output */
#define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_24       (1 << 20) /* Bit 20: IOMUXC XBAR_INOUT24 function direction select (IOMUXC_XBAR_DIR_SEL_24) */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_24_IN  (0 << 20) /*         XBAR_INOUT as input */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_24_OUT (1 << 20) /*         XBAR_INOUT as output */
#define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_25       (1 << 21) /* Bit 21: IOMUXC XBAR_INOUT25 function direction select (IOMUXC_XBAR_DIR_SEL_25) */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_25_IN  (0 << 21) /*         XBAR_INOUT as input */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_25_OUT (1 << 21) /*         XBAR_INOUT as output */
#define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_26       (1 << 22) /* Bit 22: IOMUXC XBAR_INOUT26 function direction select (IOMUXC_XBAR_DIR_SEL_26) */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_26_IN  (0 << 22) /*         XBAR_INOUT as input */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_26_OUT (1 << 22) /*         XBAR_INOUT as output */
#define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_27       (1 << 23) /* Bit 23: IOMUXC XBAR_INOUT27 function direction select (IOMUXC_XBAR_DIR_SEL_27) */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_27_IN  (0 << 23) /*         XBAR_INOUT as input */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_27_OUT (1 << 23) /*         XBAR_INOUT as output */
#define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_28       (1 << 24) /* Bit 24: IOMUXC XBAR_INOUT28 function direction select (IOMUXC_XBAR_DIR_SEL_28) */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_28_IN  (0 << 24) /*         XBAR_INOUT as input */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_28_OUT (1 << 24) /*         XBAR_INOUT as output */
#define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_29       (1 << 25) /* Bit 25: IOMUXC XBAR_INOUT29 function direction select (IOMUXC_XBAR_DIR_SEL_29) */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_29_IN  (0 << 25) /*         XBAR_INOUT as input */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_29_OUT (1 << 25) /*         XBAR_INOUT as output */
#define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_30       (1 << 26) /* Bit 26: IOMUXC XBAR_INOUT30 function direction select (IOMUXC_XBAR_DIR_SEL_30) */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_30_IN  (0 << 26) /*         XBAR_INOUT as input */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_30_OUT (1 << 26) /*         XBAR_INOUT as output */
#define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_31       (1 << 27) /* Bit 27: IOMUXC XBAR_INOUT31 function direction select (IOMUXC_XBAR_DIR_SEL_31) */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_31_IN  (0 << 27) /*         XBAR_INOUT as input */
#  define GPR_GPR20_IOMUXC_XBAR_DIR_SEL_31_OUT (1 << 27) /*         XBAR_INOUT as output */

#define GPR_GPR20_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR20_DWP_MASK                (0x03 << GPR_GPR20_DWP_SHIFT)
#  define GPR_GPR20_DWP_ALLOWBOTH         (0x00 << GPR_GPR20_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR20_DWP_DENYCM7           (0x01 << GPR_GPR20_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR20_DWP_DENYCM4           (0x02 << GPR_GPR20_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR20_DWP_DENYBOTH          (0x03 << GPR_GPR20_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR20_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR20_DWP_LOCK_MASK           (0x03 << GPR_GPR20_DWP_LOCK_SHIFT)
#  define GPR_GPR20_DWP_LOCK_NOLCK        (0x00 << GPR_GPR20_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR20_DWP_LOCK_LCKLO        (0x01 << GPR_GPR20_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR20_DWP_LOCK_LCKHI        (0x02 << GPR_GPR20_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR20_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR20_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 21 (GPR21) */

#define GPR_GPR21_IOMUXC_XBAR_DIR_SEL_32       (1 << 0)  /* Bit 0:  IOMUXC XBAR_INOUT32 function direction select (IOMUXC_XBAR_DIR_SEL_32) */
#  define GPR_GPR21_IOMUXC_XBAR_DIR_SEL_32_IN  (0 << 0)  /*         XBAR_INOUT as input */
#  define GPR_GPR21_IOMUXC_XBAR_DIR_SEL_32_OUT (1 << 0)  /*         XBAR_INOUT as output */
#define GPR_GPR21_IOMUXC_XBAR_DIR_SEL_33       (1 << 1)  /* Bit 1:  IOMUXC XBAR_INOUT33 function direction select (IOMUXC_XBAR_DIR_SEL_33) */
#  define GPR_GPR21_IOMUXC_XBAR_DIR_SEL_33_IN  (0 << 1)  /*         XBAR_INOUT as input */
#  define GPR_GPR21_IOMUXC_XBAR_DIR_SEL_33_OUT (1 << 1)  /*         XBAR_INOUT as output */
#define GPR_GPR21_IOMUXC_XBAR_DIR_SEL_34       (1 << 2)  /* Bit 2:  IOMUXC XBAR_INOUT34 function direction select (IOMUXC_XBAR_DIR_SEL_34) */
#  define GPR_GPR21_IOMUXC_XBAR_DIR_SEL_34_IN  (0 << 2)  /*         XBAR_INOUT as input */
#  define GPR_GPR21_IOMUXC_XBAR_DIR_SEL_34_OUT (1 << 2)  /*         XBAR_INOUT as output */
#define GPR_GPR21_IOMUXC_XBAR_DIR_SEL_35       (1 << 3)  /* Bit 3:  IOMUXC XBAR_INOUT35 function direction select (IOMUXC_XBAR_DIR_SEL_35) */
#  define GPR_GPR21_IOMUXC_XBAR_DIR_SEL_35_IN  (0 << 3)  /*         XBAR_INOUT as input */
#  define GPR_GPR21_IOMUXC_XBAR_DIR_SEL_35_OUT (1 << 3)  /*         XBAR_INOUT as output */
#define GPR_GPR21_IOMUXC_XBAR_DIR_SEL_36       (1 << 4)  /* Bit 4:  IOMUXC XBAR_INOUT36 function direction select (IOMUXC_XBAR_DIR_SEL_36) */
#  define GPR_GPR21_IOMUXC_XBAR_DIR_SEL_36_IN  (0 << 4)  /*         XBAR_INOUT as input */
#  define GPR_GPR21_IOMUXC_XBAR_DIR_SEL_36_OUT (1 << 4)  /*         XBAR_INOUT as output */
#define GPR_GPR21_IOMUXC_XBAR_DIR_SEL_37       (1 << 5)  /* Bit 5:  IOMUXC XBAR_INOUT37 function direction select (IOMUXC_XBAR_DIR_SEL_37) */
#  define GPR_GPR21_IOMUXC_XBAR_DIR_SEL_37_IN  (0 << 5)  /*         XBAR_INOUT as input */
#  define GPR_GPR21_IOMUXC_XBAR_DIR_SEL_37_OUT (1 << 5)  /*         XBAR_INOUT as output */
#define GPR_GPR21_IOMUXC_XBAR_DIR_SEL_38       (1 << 6)  /* Bit 6:  IOMUXC XBAR_INOUT38 function direction select (IOMUXC_XBAR_DIR_SEL_38) */
#  define GPR_GPR21_IOMUXC_XBAR_DIR_SEL_38_IN  (0 << 6)  /*         XBAR_INOUT as input */
#  define GPR_GPR21_IOMUXC_XBAR_DIR_SEL_38_OUT (1 << 6)  /*         XBAR_INOUT as output */
#define GPR_GPR21_IOMUXC_XBAR_DIR_SEL_39       (1 << 7)  /* Bit 7:  IOMUXC XBAR_INOUT39 function direction select (IOMUXC_XBAR_DIR_SEL_39) */
#  define GPR_GPR21_IOMUXC_XBAR_DIR_SEL_39_IN  (0 << 7)  /*         XBAR_INOUT as input */
#  define GPR_GPR21_IOMUXC_XBAR_DIR_SEL_39_OUT (1 << 7)  /*         XBAR_INOUT as output */
#define GPR_GPR21_IOMUXC_XBAR_DIR_SEL_40       (1 << 8)  /* Bit 8:  IOMUXC XBAR_INOUT40 function direction select (IOMUXC_XBAR_DIR_SEL_40) */
#  define GPR_GPR21_IOMUXC_XBAR_DIR_SEL_40_IN  (0 << 8)  /*         XBAR_INOUT as input */
#  define GPR_GPR21_IOMUXC_XBAR_DIR_SEL_40_OUT (1 << 8)  /*         XBAR_INOUT as output */
#define GPR_GPR21_IOMUXC_XBAR_DIR_SEL_41       (1 << 9)  /* Bit 9:  IOMUXC XBAR_INOUT41 function direction select (IOMUXC_XBAR_DIR_SEL_41) */
#  define GPR_GPR21_IOMUXC_XBAR_DIR_SEL_41_IN  (0 << 9)  /*         XBAR_INOUT as input */
#  define GPR_GPR21_IOMUXC_XBAR_DIR_SEL_41_OUT (1 << 9)  /*         XBAR_INOUT as output */
#define GPR_GPR21_IOMUXC_XBAR_DIR_SEL_42       (1 << 10) /* Bit 10: IOMUXC XBAR_INOUT42 function direction select (IOMUXC_XBAR_DIR_SEL_42) */
#  define GPR_GPR21_IOMUXC_XBAR_DIR_SEL_42_IN  (0 << 10) /*         XBAR_INOUT as input */
#  define GPR_GPR21_IOMUXC_XBAR_DIR_SEL_42_OUT (1 << 10) /*         XBAR_INOUT as output */

                                                    /* Bits 11-15: Reserved */

                                                    /* Bits 16-27: Reserved */

#define GPR_GPR21_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR21_DWP_MASK                (0x03 << GPR_GPR21_DWP_SHIFT)
#  define GPR_GPR21_DWP_ALLOWBOTH         (0x00 << GPR_GPR21_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR21_DWP_DENYCM7           (0x01 << GPR_GPR21_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR21_DWP_DENYCM4           (0x02 << GPR_GPR21_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR21_DWP_DENYBOTH          (0x03 << GPR_GPR21_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR21_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR21_DWP_LOCK_MASK           (0x03 << GPR_GPR21_DWP_LOCK_SHIFT)
#  define GPR_GPR21_DWP_LOCK_NOLCK        (0x00 << GPR_GPR21_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR21_DWP_LOCK_LCKLO        (0x01 << GPR_GPR21_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR21_DWP_LOCK_LCKHI        (0x02 << GPR_GPR21_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR21_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR21_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 22 (GPR22) */

#define GPR_GPR22_REF_1M_CLK_GPT1         (1 << 0)  /* Bit 0:      GPT1 1 MHz clock source select (REF_1M_CLK_GPT1) */
#define GPR_GPR22_REF_1M_CLK_GPT1_CLKROOT (0 << 0)  /*             GPT1 IPG_CLK_HIGHFREQ driven by GPT1_CLK_ROOT */
#define GPR_GPR22_REF_1M_CLK_GPT1_RCOSC   (1 << 0)  /*             GPT1 IPG_CLK_HIGHFREQ driven by 1 MHz clock derived from RCOSC_400M */
                                                    /* Bits 1-15:  Reserved */
                                                    /* Bits 16-27: Reserved */
#define GPR_GPR22_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR22_DWP_MASK                (0x03 << GPR_GPR22_DWP_SHIFT)
#  define GPR_GPR22_DWP_ALLOWBOTH         (0x00 << GPR_GPR22_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR22_DWP_DENYCM7           (0x01 << GPR_GPR22_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR22_DWP_DENYCM4           (0x02 << GPR_GPR22_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR22_DWP_DENYBOTH          (0x03 << GPR_GPR22_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR22_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR22_DWP_LOCK_MASK           (0x03 << GPR_GPR22_DWP_LOCK_SHIFT)
#  define GPR_GPR22_DWP_LOCK_NOLCK        (0x00 << GPR_GPR22_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR22_DWP_LOCK_LCKLO        (0x01 << GPR_GPR22_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR22_DWP_LOCK_LCKHI        (0x02 << GPR_GPR22_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR22_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR22_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 23 (GPR23) */

#define GPR_GPR23_REF_1M_CLK_GPT2         (1 << 0)  /* Bit 0:      GPT2 1 MHz clock source select (REF_1M_CLK_GPT2) */
#define GPR_GPR23_REF_1M_CLK_GPT2_CLKROOT (0 << 0)  /*             GPT2 IPG_CLK_HIGHFREQ driven by GPT2_CLK_ROOT */
#define GPR_GPR23_REF_1M_CLK_GPT2_RCOSC   (1 << 0)  /*             GPT2 IPG_CLK_HIGHFREQ driven by 1 MHz clock derived from RCOSC_400M */
#define GPR_GPR23_GPT2_CAPIN1_SEL         (1 << 1)  /* Bit 1:      GPT2 input capture channel 1 source select (GPT2_CAPIN1_SEL) */
#define GPR_GPR23_GPT2_CAPIN1_SEL_PAD     (0 << 1)  /*             Source from pad */
#define GPR_GPR23_GPT2_CAPIN1_SEL_ENET    (1 << 1)  /*             Source from ENET_1588_EVENT1_OUT */
#define GPR_GPR23_GPT2_CAPIN2_SEL         (1 << 2)  /* Bit 2:      GPT2 input capture channel 2 source select (GPT2_CAPIN2_SEL) */
#define GPR_GPR23_GPT2_CAPIN2_SEL_PAD     (0 << 2)  /*             Source from pad */
#define GPR_GPR23_GPT2_CAPIN2_SEL_ENET    (1 << 2)  /*             Source from ENET1G_1588_EVENT1_OUT */
                                                    /* Bits 3-15:  Reserved */
                                                    /* Bits 16-27: Reserved */
#define GPR_GPR23_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR23_DWP_MASK                (0x03 << GPR_GPR23_DWP_SHIFT)
#  define GPR_GPR23_DWP_ALLOWBOTH         (0x00 << GPR_GPR23_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR23_DWP_DENYCM7           (0x01 << GPR_GPR23_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR23_DWP_DENYCM4           (0x02 << GPR_GPR23_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR23_DWP_DENYBOTH          (0x03 << GPR_GPR23_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR23_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR23_DWP_LOCK_MASK           (0x03 << GPR_GPR23_DWP_LOCK_SHIFT)
#  define GPR_GPR23_DWP_LOCK_NOLCK        (0x00 << GPR_GPR23_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR23_DWP_LOCK_LCKLO        (0x01 << GPR_GPR23_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR23_DWP_LOCK_LCKHI        (0x02 << GPR_GPR23_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR23_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR23_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 24 (GPR24) */

#define GPR_GPR24_REF_1M_CLK_GPT3         (1 << 0)  /* Bit 0:      GPT3 1 MHz clock source select (REF_1M_CLK_GPT3) */
#define GPR_GPR24_REF_1M_CLK_GPT3_CLKROOT (0 << 0)  /*             GPT3 IPG_CLK_HIGHFREQ driven by GPT3_CLK_ROOT */
#define GPR_GPR24_REF_1M_CLK_GPT3_RCOSC   (1 << 0)  /*             GPT3 IPG_CLK_HIGHFREQ driven by 1 MHz clock derived from RCOSC_400M */
#define GPR_GPR24_GPT3_CAPIN1_SEL         (1 << 1)  /* Bit 1:      GPT3 input capture channel 1 source select (GPT3_CAPIN1_SEL) */
#define GPR_GPR24_GPT3_CAPIN1_SEL_PAD     (0 << 1)  /*             Source from pad */
#define GPR_GPR24_GPT3_CAPIN1_SEL_ENET    (1 << 1)  /*             Source from ENET_QOS PTP_PPS_O[1] */
                                                    /* Bits 2-15:  Reserved */
                                                    /* Bits 16-27: Reserved */
#define GPR_GPR24_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR24_DWP_MASK                (0x03 << GPR_GPR24_DWP_SHIFT)
#  define GPR_GPR24_DWP_ALLOWBOTH         (0x00 << GPR_GPR24_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR24_DWP_DENYCM7           (0x01 << GPR_GPR24_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR24_DWP_DENYCM4           (0x02 << GPR_GPR24_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR24_DWP_DENYBOTH          (0x03 << GPR_GPR24_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR24_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR24_DWP_LOCK_MASK           (0x03 << GPR_GPR24_DWP_LOCK_SHIFT)
#  define GPR_GPR24_DWP_LOCK_NOLCK        (0x00 << GPR_GPR24_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR24_DWP_LOCK_LCKLO        (0x01 << GPR_GPR24_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR24_DWP_LOCK_LCKHI        (0x02 << GPR_GPR24_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR24_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR24_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 25 (GPR25) */

#define GPR_GPR25_REF_1M_CLK_GPT4         (1 << 0)  /* Bit 0:      GPT4 1 MHz clock source select (REF_1M_CLK_GPT4) */
#define GPR_GPR25_REF_1M_CLK_GPT4_CLKROOT (0 << 0)  /*             GPT4 IPG_CLK_HIGHFREQ driven by GPT4_CLK_ROOT */
#define GPR_GPR25_REF_1M_CLK_GPT4_RCOSC   (1 << 0)  /*             GPT4 IPG_CLK_HIGHFREQ driven by 1 MHz clock derived from RCOSC_400M */
                                                    /* Bits 1-15:  Reserved */
                                                    /* Bits 16-27: Reserved */
#define GPR_GPR25_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR25_DWP_MASK                (0x03 << GPR_GPR25_DWP_SHIFT)
#  define GPR_GPR25_DWP_ALLOWBOTH         (0x00 << GPR_GPR25_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR25_DWP_DENYCM7           (0x01 << GPR_GPR25_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR25_DWP_DENYCM4           (0x02 << GPR_GPR25_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR25_DWP_DENYBOTH          (0x03 << GPR_GPR25_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR25_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR25_DWP_LOCK_MASK           (0x03 << GPR_GPR25_DWP_LOCK_SHIFT)
#  define GPR_GPR25_DWP_LOCK_NOLCK        (0x00 << GPR_GPR25_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR25_DWP_LOCK_LCKLO        (0x01 << GPR_GPR25_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR25_DWP_LOCK_LCKHI        (0x02 << GPR_GPR25_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR25_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR25_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 26 (GPR26) */

#define GPR_GPR26_REF_1M_CLK_GPT5         (1 << 0)  /* Bit 0:      GPT5 1 MHz clock source select (REF_1M_CLK_GPT5) */
#define GPR_GPR26_REF_1M_CLK_GPT5_CLKROOT (0 << 0)  /*             GPT5 IPG_CLK_HIGHFREQ driven by GPT5_CLK_ROOT */
#define GPR_GPR26_REF_1M_CLK_GPT5_RCOSC   (1 << 0)  /*             GPT5 IPG_CLK_HIGHFREQ driven by 1 MHz clock derived from RCOSC_400M */
                                                    /* Bits 1-15:  Reserved */
                                                    /* Bits 16-27: Reserved */
#define GPR_GPR26_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR26_DWP_MASK                (0x03 << GPR_GPR26_DWP_SHIFT)
#  define GPR_GPR26_DWP_ALLOWBOTH         (0x00 << GPR_GPR26_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR26_DWP_DENYCM7           (0x01 << GPR_GPR26_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR26_DWP_DENYCM4           (0x02 << GPR_GPR26_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR26_DWP_DENYBOTH          (0x03 << GPR_GPR26_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR26_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR26_DWP_LOCK_MASK           (0x03 << GPR_GPR26_DWP_LOCK_SHIFT)
#  define GPR_GPR26_DWP_LOCK_NOLCK        (0x00 << GPR_GPR26_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR26_DWP_LOCK_LCKLO        (0x01 << GPR_GPR26_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR26_DWP_LOCK_LCKHI        (0x02 << GPR_GPR26_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR26_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR26_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 27 (GPR27) */

#define GPR_GPR27_REF_1M_CLK_GPT6         (1 << 0)  /* Bit 0:      GPT6 1 MHz clock source select (REF_1M_CLK_GPT6) */
#define GPR_GPR27_REF_1M_CLK_GPT6_CLKROOT (0 << 0)  /*             GPT6 IPG_CLK_HIGHFREQ driven by GPT6_CLK_ROOT */
#define GPR_GPR27_REF_1M_CLK_GPT6_RCOSC   (1 << 0)  /*             GPT6 IPG_CLK_HIGHFREQ driven by 1 MHz clock derived from RCOSC_400M */
                                                    /* Bits 1-15:  Reserved */
                                                    /* Bits 16-27: Reserved */
#define GPR_GPR27_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR27_DWP_MASK                (0x03 << GPR_GPR27_DWP_SHIFT)
#  define GPR_GPR27_DWP_ALLOWBOTH         (0x00 << GPR_GPR27_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR27_DWP_DENYCM7           (0x01 << GPR_GPR27_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR27_DWP_DENYCM4           (0x02 << GPR_GPR27_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR27_DWP_DENYBOTH          (0x03 << GPR_GPR27_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR27_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR27_DWP_LOCK_MASK           (0x03 << GPR_GPR27_DWP_LOCK_SHIFT)
#  define GPR_GPR27_DWP_LOCK_NOLCK        (0x00 << GPR_GPR27_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR27_DWP_LOCK_LCKLO        (0x01 << GPR_GPR27_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR27_DWP_LOCK_LCKHI        (0x02 << GPR_GPR27_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR27_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR27_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 28 (GPR28) */

#define GPR_GPR28_ARCACHE_USDHC           (1 << 0)  /* Bit 0:      uSDHC block cacheable attribute value of AXI read transactions (ARCACHE_USDHC) */
#define GPR_GPR28_AWCACHE_USDHC           (1 << 1)  /* Bit 1:      uSDHC block cacheable attribute value of AXI write transactions (AWCACHE_USDHC) */
                                                    /* Bits 2-4:   Reserved */
#define GPR_GPR28_CACHE_ENET1G            (1 << 5)  /* Bit 5:      ENET1G block cacheable attribute value of AXI transactions (CACHE_ENET1G) */
                                                    /* Bit 6:      Reserved */
#define GPR_GPR28_CACHE_ENET              (1 << 7)  /* Bit 7:      ENET block cacheable attribute value of AXI transactions (CACHE_ENET) */
                                                    /* Bits 8-12:  Reserved */
#define GPR_GPR28_CACHE_USB               (1 << 13) /* Bit 13:     USB block cacheable attribute value of AXI transactions (CACHE_USB) */
                                                    /* Bits 14-15: Reserved */
                                                    /* Bits 16-27: Reserved */
#define GPR_GPR28_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR28_DWP_MASK                (0x03 << GPR_GPR28_DWP_SHIFT)
#  define GPR_GPR28_DWP_ALLOWBOTH         (0x00 << GPR_GPR28_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR28_DWP_DENYCM7           (0x01 << GPR_GPR28_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR28_DWP_DENYCM4           (0x02 << GPR_GPR28_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR28_DWP_DENYBOTH          (0x03 << GPR_GPR28_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR28_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR28_DWP_LOCK_MASK           (0x03 << GPR_GPR28_DWP_LOCK_SHIFT)
#  define GPR_GPR28_DWP_LOCK_NOLCK        (0x00 << GPR_GPR28_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR28_DWP_LOCK_LCKLO        (0x01 << GPR_GPR28_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR28_DWP_LOCK_LCKHI        (0x02 << GPR_GPR28_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR28_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR28_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 29 (GPR29) */

#define GPR_GPR29_USBPHY1_IPG_CLK_ACTIVE  (1 << 0)  /* Bit 0:      USBPHY1 register access clock enable (USBPHY1_IPG_CLK_ACTIVE) */
                                                    /* Bits 1-15:  Reserved */
                                                    /* Bits 16-27: Reserved */
#define GPR_GPR29_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR29_DWP_MASK                (0x03 << GPR_GPR29_DWP_SHIFT)
#  define GPR_GPR29_DWP_ALLOWBOTH         (0x00 << GPR_GPR29_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR29_DWP_DENYCM7           (0x01 << GPR_GPR29_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR29_DWP_DENYCM4           (0x02 << GPR_GPR29_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR29_DWP_DENYBOTH          (0x03 << GPR_GPR29_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR29_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR29_DWP_LOCK_MASK           (0x03 << GPR_GPR29_DWP_LOCK_SHIFT)
#  define GPR_GPR29_DWP_LOCK_NOLCK        (0x00 << GPR_GPR29_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR29_DWP_LOCK_LCKLO        (0x01 << GPR_GPR29_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR29_DWP_LOCK_LCKHI        (0x02 << GPR_GPR29_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR29_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR29_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 30 (GPR30) */

#define GPR_GPR30_USBPHY2_IPG_CLK_ACTIVE  (1 << 0)  /* Bit 0:      USBPHY2 register access clock enable (USBPHY2_IPG_CLK_ACTIVE) */
                                                    /* Bits 1-15:  Reserved */
                                                    /* Bits 16-27: Reserved */
#define GPR_GPR30_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR30_DWP_MASK                (0x03 << GPR_GPR30_DWP_SHIFT)
#  define GPR_GPR30_DWP_ALLOWBOTH         (0x00 << GPR_GPR30_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR30_DWP_DENYCM7           (0x01 << GPR_GPR30_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR30_DWP_DENYCM4           (0x02 << GPR_GPR30_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR30_DWP_DENYBOTH          (0x03 << GPR_GPR30_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR30_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR30_DWP_LOCK_MASK           (0x03 << GPR_GPR30_DWP_LOCK_SHIFT)
#  define GPR_GPR30_DWP_LOCK_NOLCK        (0x00 << GPR_GPR30_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR30_DWP_LOCK_LCKLO        (0x01 << GPR_GPR30_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR30_DWP_LOCK_LCKHI        (0x02 << GPR_GPR30_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR30_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR30_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 31 (GPR31) */

#define GPR_GPR31_RMW2_WAIT_BVALID_CPL    (1 << 0)  /* Bit 0:      OCRAM M7 RMW wait enable (RMW2_WAIT_BVALID_CPL) */
                                                    /* Bit 1:      Reserved */
#define GPR_GPR31_OCRAM_M7_CLK_GATING     (1 << 2)  /* Bit 2:      OCRAM M7 clock gating enable (OCRAM_M7_CLK_GATING) */
                                                    /* Bits 3-15:  Reserved */
                                                    /* Bits 16-27: Reserved */
#define GPR_GPR31_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR31_DWP_MASK                (0x03 << GPR_GPR31_DWP_SHIFT)
#  define GPR_GPR31_DWP_ALLOWBOTH         (0x00 << GPR_GPR31_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR31_DWP_DENYCM7           (0x01 << GPR_GPR31_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR31_DWP_DENYCM4           (0x02 << GPR_GPR31_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR31_DWP_DENYBOTH          (0x03 << GPR_GPR31_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR31_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR31_DWP_LOCK_MASK           (0x03 << GPR_GPR31_DWP_LOCK_SHIFT)
#  define GPR_GPR31_DWP_LOCK_NOLCK        (0x00 << GPR_GPR31_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR31_DWP_LOCK_LCKLO        (0x01 << GPR_GPR31_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR31_DWP_LOCK_LCKHI        (0x02 << GPR_GPR31_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR31_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR31_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 32 (GPR32) */

#define GPR_GPR32_RMW1_WAIT_BVALID_CPL    (1 << 0)  /* Bit 0:      OCRAM1 RMW wait enable (RMW1_WAIT_BVALID_CPL) */
                                                    /* Bits 1-15:  Reserved */
                                                    /* Bits 16-27: Reserved */
#define GPR_GPR32_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR32_DWP_MASK                (0x03 << GPR_GPR32_DWP_SHIFT)
#  define GPR_GPR32_DWP_ALLOWBOTH         (0x00 << GPR_GPR32_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR32_DWP_DENYCM7           (0x01 << GPR_GPR32_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR32_DWP_DENYCM4           (0x02 << GPR_GPR32_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR32_DWP_DENYBOTH          (0x03 << GPR_GPR32_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR32_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR32_DWP_LOCK_MASK           (0x03 << GPR_GPR32_DWP_LOCK_SHIFT)
#  define GPR_GPR32_DWP_LOCK_NOLCK        (0x00 << GPR_GPR32_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR32_DWP_LOCK_LCKLO        (0x01 << GPR_GPR32_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR32_DWP_LOCK_LCKHI        (0x02 << GPR_GPR32_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR32_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR32_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 33 (GPR33) */

#define GPR_GPR33_RMW2_WAIT_BVALID_CPL    (1 << 0)  /* Bit 0:      OCRAM2 RMW wait enable (RMW2_WAIT_BVALID_CPL) */
                                                    /* Bits 1-15:  Reserved */
                                                    /* Bits 16-27: Reserved */
#define GPR_GPR33_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR33_DWP_MASK                (0x03 << GPR_GPR33_DWP_SHIFT)
#  define GPR_GPR33_DWP_ALLOWBOTH         (0x00 << GPR_GPR33_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR33_DWP_DENYCM7           (0x01 << GPR_GPR33_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR33_DWP_DENYCM4           (0x02 << GPR_GPR33_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR33_DWP_DENYBOTH          (0x03 << GPR_GPR33_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR33_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR33_DWP_LOCK_MASK           (0x03 << GPR_GPR33_DWP_LOCK_SHIFT)
#  define GPR_GPR33_DWP_LOCK_NOLCK        (0x00 << GPR_GPR33_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR33_DWP_LOCK_LCKLO        (0x01 << GPR_GPR33_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR33_DWP_LOCK_LCKHI        (0x02 << GPR_GPR33_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR33_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR33_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 34 (GPR34) */

#define GPR_GPR34_XECC_FLEXSPI1_WAIT_BVALID_CPL (1 << 0) /* Bit 0: XECC_FLEXSPI1 RMW wait enable (XECC_FLEXSPI1_WAIT_BVALID_CPL) */

#define GPR_GPR34_FLEXSPI1_OTFAD_EN       (1 << 1)  /* Bit 1: FlexSPI1 OTFAD enable (FLEXSPI1_OTFAD_EN) */
                                                    /* Bits 2-15:  Reserved */
                                                    /* Bits 16-27: Reserved */
#define GPR_GPR34_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR34_DWP_MASK                (0x03 << GPR_GPR34_DWP_SHIFT)
#  define GPR_GPR34_DWP_ALLOWBOTH         (0x00 << GPR_GPR34_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR34_DWP_DENYCM7           (0x01 << GPR_GPR34_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR34_DWP_DENYCM4           (0x02 << GPR_GPR34_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR34_DWP_DENYBOTH          (0x03 << GPR_GPR34_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR34_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR34_DWP_LOCK_MASK           (0x03 << GPR_GPR34_DWP_LOCK_SHIFT)
#  define GPR_GPR34_DWP_LOCK_NOLCK        (0x00 << GPR_GPR34_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR34_DWP_LOCK_LCKLO        (0x01 << GPR_GPR34_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR34_DWP_LOCK_LCKHI        (0x02 << GPR_GPR34_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR34_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR34_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 35 (GPR35) */

#define GPR_GPR35_XECC_FLEXSPI2_WAIT_BVALID_CPL (1 << 0) /* Bit 0: XECC_FLEXSPI2 RMW wait enable (XECC_FLEXSPI2_WAIT_BVALID_CPL) */

#define GPR_GPR35_FLEXSPI2_OTFAD_EN       (1 << 1)  /* Bit 1: FlexSPI2 OTFAD enable (FLEXSPI2_OTFAD_EN) */
                                                    /* Bits 2-15:  Reserved */
                                                    /* Bits 16-27: Reserved */
#define GPR_GPR35_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR35_DWP_MASK                (0x03 << GPR_GPR35_DWP_SHIFT)
#  define GPR_GPR35_DWP_ALLOWBOTH         (0x00 << GPR_GPR35_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR35_DWP_DENYCM7           (0x01 << GPR_GPR35_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR35_DWP_DENYCM4           (0x02 << GPR_GPR35_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR35_DWP_DENYBOTH          (0x03 << GPR_GPR35_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR35_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR35_DWP_LOCK_MASK           (0x03 << GPR_GPR35_DWP_LOCK_SHIFT)
#  define GPR_GPR35_DWP_LOCK_NOLCK        (0x00 << GPR_GPR35_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR35_DWP_LOCK_LCKLO        (0x01 << GPR_GPR35_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR35_DWP_LOCK_LCKHI        (0x02 << GPR_GPR35_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR35_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR35_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 36 (GPR36) */

#define GPR_GPR36_XECC_SEMC_WAIT_BVALID_CPL (1 << 0) /* Bit 0: XECC_SEMC RMW wait enable (XECC_SEMC_WAIT_BVALID_CPL) */

                                                    /* Bits 1-15:  Reserved */

                                                    /* Bits 16-27: Reserved */

#define GPR_GPR36_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR36_DWP_MASK                (0x03 << GPR_GPR36_DWP_SHIFT)
#  define GPR_GPR36_DWP_ALLOWBOTH         (0x00 << GPR_GPR36_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR36_DWP_DENYCM7           (0x01 << GPR_GPR36_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR36_DWP_DENYCM4           (0x02 << GPR_GPR36_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR36_DWP_DENYBOTH          (0x03 << GPR_GPR36_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR36_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR36_DWP_LOCK_MASK           (0x03 << GPR_GPR36_DWP_LOCK_SHIFT)
#  define GPR_GPR36_DWP_LOCK_NOLCK        (0x00 << GPR_GPR36_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR36_DWP_LOCK_LCKLO        (0x01 << GPR_GPR36_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR36_DWP_LOCK_LCKHI        (0x02 << GPR_GPR36_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR36_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR36_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 37 (GPR37) */

#define GPR_GPR37_NIDEN                   (1 << 0)  /* Bit 0:      ARM non-secure (non-invasive) debug enable (NIDEN) */
#define GPR_GPR37_DBG_EN                  (1 << 1)  /* Bit 1:      ARM invasive debug enable (DBG_EN) */
                                                    /* Bit 2:      Reserved */
#define GPR_GPR37_EXC_MON                           /* Bit 3:      Exclusive monitor response select of illegal command (EXC_MON) */
                                                    /* Bit 4:      Reserved */
#define GPR_GPR37_M7_DBG_ACK_MASK                   /* Bit 5:      CM7 debug halt mask (M7_DBG_ACK_MASK) */
#define GPR_GPR37_M4_DBG_ACK_MASK                   /* Bit 6:      CM4 debug halt mask (M4_DBG_ACK_MASK) */
                                                    /* Bits 7-15:  Reserved */
                                                    /* Bits 16-27: Reserved */
#define GPR_GPR37_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR37_DWP_MASK                (0x03 << GPR_GPR37_DWP_SHIFT)
#  define GPR_GPR37_DWP_ALLOWBOTH         (0x00 << GPR_GPR37_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR37_DWP_DENYCM7           (0x01 << GPR_GPR37_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR37_DWP_DENYCM4           (0x02 << GPR_GPR37_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR37_DWP_DENYBOTH          (0x03 << GPR_GPR37_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR37_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR37_DWP_LOCK_MASK           (0x03 << GPR_GPR37_DWP_LOCK_SHIFT)
#  define GPR_GPR37_DWP_LOCK_NOLCK        (0x00 << GPR_GPR37_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR37_DWP_LOCK_LCKLO        (0x01 << GPR_GPR37_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR37_DWP_LOCK_LCKHI        (0x02 << GPR_GPR37_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR37_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR37_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 38 (GPR38) - Reserved */

/* General Purpose Register 39 (GPR39) - Reserved */

/* General Purpose Register 40 (GPR40) */

#define GPR_GPR40_GPIO_MUX2_GPIO_SEL_LOW_SHIFT      (0) /* Bits 0-15: GPIO2 and CM7_GPIO2 share same IO MUX function, GPIO_MUX2 selects one GPIO function (GPIO_MUX2_GPIO_SEL_LOW) */
#define GPR_GPR40_GPIO_MUX2_GPIO_SEL_LOW_MASK       (0xffff << GPR_GPR40_GPIO_MUX2_GPIO_SEL_LOW_SHIFT)
#  define GPR_GPR40_GPIO_MUX2_GPIO0_SEL_GPIO2       (0 << 0)   /* Select GPIO2_0 */
#  define GPR_GPR40_GPIO_MUX2_GPIO0_SEL_CM7_GPIO2   (1 << 0)   /* Select CM7_GPIO2_0 */
#  define GPR_GPR40_GPIO_MUX2_GPIO1_SEL_GPIO2       (0 << 1)   /* Select GPIO2_1 */
#  define GPR_GPR40_GPIO_MUX2_GPIO1_SEL_CM7_GPIO2   (1 << 1)   /* Select CM7_GPIO2_1 */
#  define GPR_GPR40_GPIO_MUX2_GPIO2_SEL_GPIO2       (0 << 2)   /* Select GPIO2_2 */
#  define GPR_GPR40_GPIO_MUX2_GPIO2_SEL_CM7_GPIO2   (1 << 2)   /* Select CM7_GPIO2_2 */
#  define GPR_GPR40_GPIO_MUX2_GPIO3_SEL_GPIO2       (0 << 3)   /* Select GPIO2_3 */
#  define GPR_GPR40_GPIO_MUX2_GPIO3_SEL_CM7_GPIO2   (1 << 3)   /* Select CM7_GPIO2_3 */
#  define GPR_GPR40_GPIO_MUX2_GPIO4_SEL_GPIO2       (0 << 4)   /* Select GPIO2_4 */
#  define GPR_GPR40_GPIO_MUX2_GPIO4_SEL_CM7_GPIO2   (1 << 4)   /* Select CM7_GPIO2_4 */
#  define GPR_GPR40_GPIO_MUX2_GPIO5_SEL_GPIO2       (0 << 5)   /* Select GPIO2_5 */
#  define GPR_GPR40_GPIO_MUX2_GPIO5_SEL_CM7_GPIO2   (1 << 5)   /* Select CM7_GPIO2_5 */
#  define GPR_GPR40_GPIO_MUX2_GPIO6_SEL_GPIO2       (0 << 6)   /* Select GPIO2_6 */
#  define GPR_GPR40_GPIO_MUX2_GPIO6_SEL_CM7_GPIO2   (1 << 6)   /* Select CM7_GPIO2_6 */
#  define GPR_GPR40_GPIO_MUX2_GPIO7_SEL_GPIO2       (0 << 7)   /* Select GPIO2_7 */
#  define GPR_GPR40_GPIO_MUX2_GPIO7_SEL_CM7_GPIO2   (1 << 7)   /* Select CM7_GPIO2_7 */
#  define GPR_GPR40_GPIO_MUX2_GPIO8_SEL_GPIO2       (0 << 8)   /* Select GPIO2_8 */
#  define GPR_GPR40_GPIO_MUX2_GPIO8_SEL_CM7_GPIO2   (1 << 8)   /* Select CM7_GPIO2_8 */
#  define GPR_GPR40_GPIO_MUX2_GPIO9_SEL_GPIO2       (0 << 9)   /* Select GPIO2_9 */
#  define GPR_GPR40_GPIO_MUX2_GPIO9_SEL_CM7_GPIO2   (1 << 9)   /* Select CM7_GPIO2_9 */
#  define GPR_GPR40_GPIO_MUX2_GPIO10_SEL_GPIO2      (0 << 10)  /* Select GPIO2_10 */
#  define GPR_GPR40_GPIO_MUX2_GPIO10_SEL_CM7_GPIO2  (1 << 10)  /* Select CM7_GPIO2_10 */
#  define GPR_GPR40_GPIO_MUX2_GPIO11_SEL_GPIO2      (0 << 11)  /* Select GPIO2_11 */
#  define GPR_GPR40_GPIO_MUX2_GPIO11_SEL_CM7_GPIO2  (1 << 11)  /* Select CM7_GPIO2_11 */
#  define GPR_GPR40_GPIO_MUX2_GPIO12_SEL_GPIO2      (0 << 12)  /* Select GPIO2_12 */
#  define GPR_GPR40_GPIO_MUX2_GPIO12_SEL_CM7_GPIO2  (1 << 12)  /* Select CM7_GPIO2_12 */
#  define GPR_GPR40_GPIO_MUX2_GPIO13_SEL_GPIO2      (0 << 13)  /* Select GPIO2_13 */
#  define GPR_GPR40_GPIO_MUX2_GPIO13_SEL_CM7_GPIO2  (1 << 13)  /* Select CM7_GPIO2_13 */
#  define GPR_GPR40_GPIO_MUX2_GPIO14_SEL_GPIO2      (0 << 14)  /* Select GPIO2_14 */
#  define GPR_GPR40_GPIO_MUX2_GPIO14_SEL_CM7_GPIO2  (1 << 14)  /* Select CM7_GPIO2_14 */
#  define GPR_GPR40_GPIO_MUX2_GPIO15_SEL_GPIO2      (0 << 15)  /* Select GPIO2_15 */
#  define GPR_GPR40_GPIO_MUX2_GPIO15_SEL_CM7_GPIO2  (1 << 15)  /* Select CM7_GPIO2_15 */
#  define GPR_GPR40_GPIO_MUX2_GPIO_SEL_GPIO2(n)     (0 << (n)) /* Select GPIO2_n */
#  define GPR_GPR40_GPIO_MUX2_GPIO_SEL_CM7_GPIO2(n) (1 << (n)) /* Select CM7_GPIO2_n */

                                                    /* Bits 16-27: Reserved */
#define GPR_GPR40_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR40_DWP_MASK                (0x03 << GPR_GPR40_DWP_SHIFT)
#  define GPR_GPR40_DWP_ALLOWBOTH         (0x00 << GPR_GPR40_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR40_DWP_DENYCM7           (0x01 << GPR_GPR40_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR40_DWP_DENYCM4           (0x02 << GPR_GPR40_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR40_DWP_DENYBOTH          (0x03 << GPR_GPR40_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR40_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR40_DWP_LOCK_MASK           (0x03 << GPR_GPR40_DWP_LOCK_SHIFT)
#  define GPR_GPR40_DWP_LOCK_NOLCK        (0x00 << GPR_GPR40_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR40_DWP_LOCK_LCKLO        (0x01 << GPR_GPR40_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR40_DWP_LOCK_LCKHI        (0x02 << GPR_GPR40_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR40_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR40_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 41 (GPR41) */

#define GPR_GPR41_GPIO_MUX2_GPIO_SEL_HIGH_SHIFT     (0) /* Bits 0-15: GPIO2 and CM7_GPIO2 share same IO MUX function, GPIO_MUX2 selects one GPIO function (GPIO_MUX2_GPIO_SEL_HIGH) */
#define GPR_GPR41_GPIO_MUX2_GPIO_SEL_HIGH_MASK      (0xffff << GPR_GPR41_GPIO_MUX2_GPIO_SEL_HIGH_SHIFT)
#  define GPR_GPR41_GPIO_MUX2_GPIO16_SEL_GPIO2      (0 << 0)       /* Select GPIO2_16 */
#  define GPR_GPR41_GPIO_MUX2_GPIO16_SEL_CM7_GPIO2  (1 << 0)       /* Select CM7_GPIO2_16 */
#  define GPR_GPR41_GPIO_MUX2_GPIO17_SEL_GPIO2      (0 << 1)       /* Select GPIO2_17 */
#  define GPR_GPR41_GPIO_MUX2_GPIO17_SEL_CM7_GPIO2  (1 << 1)       /* Select CM7_GPIO2_17 */
#  define GPR_GPR41_GPIO_MUX2_GPIO18_SEL_GPIO2      (0 << 2)       /* Select GPIO2_18 */
#  define GPR_GPR41_GPIO_MUX2_GPIO18_SEL_CM7_GPIO2  (1 << 2)       /* Select CM7_GPIO2_18 */
#  define GPR_GPR41_GPIO_MUX2_GPIO19_SEL_GPIO2      (0 << 3)       /* Select GPIO2_19 */
#  define GPR_GPR41_GPIO_MUX2_GPIO19_SEL_CM7_GPIO2  (1 << 3)       /* Select CM7_GPIO2_19 */
#  define GPR_GPR41_GPIO_MUX2_GPIO20_SEL_GPIO2      (0 << 4)       /* Select GPIO2_20 */
#  define GPR_GPR41_GPIO_MUX2_GPIO20_SEL_CM7_GPIO2  (1 << 4)       /* Select CM7_GPIO2_20 */
#  define GPR_GPR41_GPIO_MUX2_GPIO21_SEL_GPIO2      (0 << 5)       /* Select GPIO2_21 */
#  define GPR_GPR41_GPIO_MUX2_GPIO21_SEL_CM7_GPIO2  (1 << 5)       /* Select CM7_GPIO2_21 */
#  define GPR_GPR41_GPIO_MUX2_GPIO22_SEL_GPIO2      (0 << 6)       /* Select GPIO2_22 */
#  define GPR_GPR41_GPIO_MUX2_GPIO22_SEL_CM7_GPIO2  (1 << 6)       /* Select CM7_GPIO2_22 */
#  define GPR_GPR41_GPIO_MUX2_GPIO23_SEL_GPIO2      (0 << 7)       /* Select GPIO2_23 */
#  define GPR_GPR41_GPIO_MUX2_GPIO23_SEL_CM7_GPIO2  (1 << 7)       /* Select CM7_GPIO2_23 */
#  define GPR_GPR41_GPIO_MUX2_GPIO24_SEL_GPIO2      (0 << 8)       /* Select GPIO2_24 */
#  define GPR_GPR41_GPIO_MUX2_GPIO24_SEL_CM7_GPIO2  (1 << 8)       /* Select CM7_GPIO2_24 */
#  define GPR_GPR41_GPIO_MUX2_GPIO25_SEL_GPIO2      (0 << 9)       /* Select GPIO2_25 */
#  define GPR_GPR41_GPIO_MUX2_GPIO25_SEL_CM7_GPIO2  (1 << 9)       /* Select CM7_GPIO2_25 */
#  define GPR_GPR41_GPIO_MUX2_GPIO26_SEL_GPIO2      (0 << 10)      /* Select GPIO2_26 */
#  define GPR_GPR41_GPIO_MUX2_GPIO26_SEL_CM7_GPIO2  (1 << 10)      /* Select CM7_GPIO2_26 */
#  define GPR_GPR41_GPIO_MUX2_GPIO27_SEL_GPIO2      (0 << 11)      /* Select GPIO2_27 */
#  define GPR_GPR41_GPIO_MUX2_GPIO27_SEL_CM7_GPIO2  (1 << 11)      /* Select CM7_GPIO2_27 */
#  define GPR_GPR41_GPIO_MUX2_GPIO28_SEL_GPIO2      (0 << 12)      /* Select GPIO2_28 */
#  define GPR_GPR41_GPIO_MUX2_GPIO28_SEL_CM7_GPIO2  (1 << 12)      /* Select CM7_GPIO2_28 */
#  define GPR_GPR41_GPIO_MUX2_GPIO29_SEL_GPIO2      (0 << 13)      /* Select GPIO2_29 */
#  define GPR_GPR41_GPIO_MUX2_GPIO29_SEL_CM7_GPIO2  (1 << 13)      /* Select CM7_GPIO2_29 */
#  define GPR_GPR41_GPIO_MUX2_GPIO30_SEL_GPIO2      (0 << 14)      /* Select GPIO2_30 */
#  define GPR_GPR41_GPIO_MUX2_GPIO30_SEL_CM7_GPIO2  (1 << 14)      /* Select CM7_GPIO2_30 */
#  define GPR_GPR41_GPIO_MUX2_GPIO31_SEL_GPIO2      (0 << 15)      /* Select GPIO2_31 */
#  define GPR_GPR41_GPIO_MUX2_GPIO31_SEL_CM7_GPIO2  (1 << 15)      /* Select CM7_GPIO2_31 */
#  define GPR_GPR41_GPIO_MUX2_GPIO_SEL_GPIO2(n)     (0 << ((n)-16) /* Select GPIO2_n */
#  define GPR_GPR41_GPIO_MUX2_GPIO_SEL_CM7_GPIO2(n) (1 << ((n)-16) /* Select CM7_GPIO2_n */

                                                    /* Bits 16-27: Reserved */
#define GPR_GPR41_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR41_DWP_MASK                (0x03 << GPR_GPR41_DWP_SHIFT)
#  define GPR_GPR41_DWP_ALLOWBOTH         (0x00 << GPR_GPR41_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR41_DWP_DENYCM7           (0x01 << GPR_GPR41_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR41_DWP_DENYCM4           (0x02 << GPR_GPR41_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR41_DWP_DENYBOTH          (0x03 << GPR_GPR41_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR41_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR41_DWP_LOCK_MASK           (0x03 << GPR_GPR41_DWP_LOCK_SHIFT)
#  define GPR_GPR41_DWP_LOCK_NOLCK        (0x00 << GPR_GPR41_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR41_DWP_LOCK_LCKLO        (0x01 << GPR_GPR41_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR41_DWP_LOCK_LCKHI        (0x02 << GPR_GPR41_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR41_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR41_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 42 (GPR42) */

#define GPR_GPR42_GPIO_MUX3_GPIO_SEL_LOW_SHIFT      (0) /* Bits 0-15: GPIO3 and CM7_GPIO3 share same IO MUX function, GPIO_MUX3 selects one GPIO function (GPIO_MUX3_GPIO_SEL_LOW) */
#define GPR_GPR42_GPIO_MUX3_GPIO_SEL_LOW_MASK       (0xffff << GPR_GPR42_GPIO_MUX3_GPIO_SEL_LOW_SHIFT)
#  define GPR_GPR42_GPIO_MUX3_GPIO0_SEL_GPIO3       (0 << 0)   /* Select GPIO3_0 */
#  define GPR_GPR42_GPIO_MUX3_GPIO0_SEL_CM7_GPIO3   (1 << 0)   /* Select CM7_GPIO3_0 */
#  define GPR_GPR42_GPIO_MUX3_GPIO1_SEL_GPIO3       (0 << 1)   /* Select GPIO3_1 */
#  define GPR_GPR42_GPIO_MUX3_GPIO1_SEL_CM7_GPIO3   (1 << 1)   /* Select CM7_GPIO3_1 */
#  define GPR_GPR42_GPIO_MUX3_GPIO2_SEL_GPIO3       (0 << 2)   /* Select GPIO3_2 */
#  define GPR_GPR42_GPIO_MUX3_GPIO2_SEL_CM7_GPIO3   (1 << 2)   /* Select CM7_GPIO3_2 */
#  define GPR_GPR42_GPIO_MUX3_GPIO3_SEL_GPIO3       (0 << 3)   /* Select GPIO3_3 */
#  define GPR_GPR42_GPIO_MUX3_GPIO3_SEL_CM7_GPIO3   (1 << 3)   /* Select CM7_GPIO3_3 */
#  define GPR_GPR42_GPIO_MUX3_GPIO4_SEL_GPIO3       (0 << 4)   /* Select GPIO3_4 */
#  define GPR_GPR42_GPIO_MUX3_GPIO4_SEL_CM7_GPIO3   (1 << 4)   /* Select CM7_GPIO3_4 */
#  define GPR_GPR42_GPIO_MUX3_GPIO5_SEL_GPIO3       (0 << 5)   /* Select GPIO3_5 */
#  define GPR_GPR42_GPIO_MUX3_GPIO5_SEL_CM7_GPIO3   (1 << 5)   /* Select CM7_GPIO3_5 */
#  define GPR_GPR42_GPIO_MUX3_GPIO6_SEL_GPIO3       (0 << 6)   /* Select GPIO3_6 */
#  define GPR_GPR42_GPIO_MUX3_GPIO6_SEL_CM7_GPIO3   (1 << 6)   /* Select CM7_GPIO3_6 */
#  define GPR_GPR42_GPIO_MUX3_GPIO7_SEL_GPIO3       (0 << 7)   /* Select GPIO3_7 */
#  define GPR_GPR42_GPIO_MUX3_GPIO7_SEL_CM7_GPIO3   (1 << 7)   /* Select CM7_GPIO3_7 */
#  define GPR_GPR42_GPIO_MUX3_GPIO8_SEL_GPIO3       (0 << 8)   /* Select GPIO3_8 */
#  define GPR_GPR42_GPIO_MUX3_GPIO8_SEL_CM7_GPIO3   (1 << 8)   /* Select CM7_GPIO3_8 */
#  define GPR_GPR42_GPIO_MUX3_GPIO9_SEL_GPIO3       (0 << 9)   /* Select GPIO3_9 */
#  define GPR_GPR42_GPIO_MUX3_GPIO9_SEL_CM7_GPIO3   (1 << 9)   /* Select CM7_GPIO3_9 */
#  define GPR_GPR42_GPIO_MUX3_GPIO10_SEL_GPIO3      (0 << 10)  /* Select GPIO3_10 */
#  define GPR_GPR42_GPIO_MUX3_GPIO10_SEL_CM7_GPIO3  (1 << 10)  /* Select CM7_GPIO3_10 */
#  define GPR_GPR42_GPIO_MUX3_GPIO11_SEL_GPIO3      (0 << 11)  /* Select GPIO3_11 */
#  define GPR_GPR42_GPIO_MUX3_GPIO11_SEL_CM7_GPIO3  (1 << 11)  /* Select CM7_GPIO3_11 */
#  define GPR_GPR42_GPIO_MUX3_GPIO12_SEL_GPIO3      (0 << 12)  /* Select GPIO3_12 */
#  define GPR_GPR42_GPIO_MUX3_GPIO12_SEL_CM7_GPIO3  (1 << 12)  /* Select CM7_GPIO3_12 */
#  define GPR_GPR42_GPIO_MUX3_GPIO13_SEL_GPIO3      (0 << 13)  /* Select GPIO3_13 */
#  define GPR_GPR42_GPIO_MUX3_GPIO13_SEL_CM7_GPIO3  (1 << 13)  /* Select CM7_GPIO3_13 */
#  define GPR_GPR42_GPIO_MUX3_GPIO14_SEL_GPIO3      (0 << 14)  /* Select GPIO3_14 */
#  define GPR_GPR42_GPIO_MUX3_GPIO14_SEL_CM7_GPIO3  (1 << 14)  /* Select CM7_GPIO3_14 */
#  define GPR_GPR42_GPIO_MUX3_GPIO15_SEL_GPIO3      (0 << 15)  /* Select GPIO3_15 */
#  define GPR_GPR42_GPIO_MUX3_GPIO15_SEL_CM7_GPIO3  (1 << 15)  /* Select CM7_GPIO3_15 */
#  define GPR_GPR42_GPIO_MUX3_GPIO_SEL_GPIO3(n)     (0 << (n)) /* Select GPIO3_n */
#  define GPR_GPR42_GPIO_MUX3_GPIO_SEL_CM7_GPIO3(n) (1 << (n)) /* Select CM7_GPIO3_n */

                                                    /* Bits 16-27: Reserved */
#define GPR_GPR42_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR42_DWP_MASK                (0x03 << GPR_GPR42_DWP_SHIFT)
#  define GPR_GPR42_DWP_ALLOWBOTH         (0x00 << GPR_GPR42_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR42_DWP_DENYCM7           (0x01 << GPR_GPR42_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR42_DWP_DENYCM4           (0x02 << GPR_GPR42_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR42_DWP_DENYBOTH          (0x03 << GPR_GPR42_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR42_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR42_DWP_LOCK_MASK           (0x03 << GPR_GPR42_DWP_LOCK_SHIFT)
#  define GPR_GPR42_DWP_LOCK_NOLCK        (0x00 << GPR_GPR42_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR42_DWP_LOCK_LCKLO        (0x01 << GPR_GPR42_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR42_DWP_LOCK_LCKHI        (0x02 << GPR_GPR42_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR42_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR42_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 43 (GPR43) */

#define GPR_GPR43_GPIO_MUX3_GPIO_SEL_HIGH_SHIFT     (0) /* Bits 0-15: GPIO3 and CM7_GPIO3 share same IO MUX function, GPIO_MUX3 selects one GPIO function (GPIO_MUX3_GPIO_SEL_HIGH) */
#define GPR_GPR43_GPIO_MUX3_GPIO_SEL_HIGH_MASK      (0xffff << GPR_GPR43_GPIO_MUX3_GPIO_SEL_HIGH_SHIFT)
#  define GPR_GPR43_GPIO_MUX3_GPIO16_SEL_GPIO3      (0 << 0)       /* Select GPIO3_16 */
#  define GPR_GPR43_GPIO_MUX3_GPIO16_SEL_CM7_GPIO3  (1 << 0)       /* Select CM7_GPIO3_16 */
#  define GPR_GPR43_GPIO_MUX3_GPIO17_SEL_GPIO3      (0 << 1)       /* Select GPIO3_17 */
#  define GPR_GPR43_GPIO_MUX3_GPIO17_SEL_CM7_GPIO3  (1 << 1)       /* Select CM7_GPIO3_17 */
#  define GPR_GPR43_GPIO_MUX3_GPIO18_SEL_GPIO3      (0 << 2)       /* Select GPIO3_18 */
#  define GPR_GPR43_GPIO_MUX3_GPIO18_SEL_CM7_GPIO3  (1 << 2)       /* Select CM7_GPIO3_18 */
#  define GPR_GPR43_GPIO_MUX3_GPIO19_SEL_GPIO3      (0 << 3)       /* Select GPIO3_19 */
#  define GPR_GPR43_GPIO_MUX3_GPIO19_SEL_CM7_GPIO3  (1 << 3)       /* Select CM7_GPIO3_19 */
#  define GPR_GPR43_GPIO_MUX3_GPIO20_SEL_GPIO3      (0 << 4)       /* Select GPIO3_20 */
#  define GPR_GPR43_GPIO_MUX3_GPIO20_SEL_CM7_GPIO3  (1 << 4)       /* Select CM7_GPIO3_20 */
#  define GPR_GPR43_GPIO_MUX3_GPIO21_SEL_GPIO3      (0 << 5)       /* Select GPIO3_21 */
#  define GPR_GPR43_GPIO_MUX3_GPIO21_SEL_CM7_GPIO3  (1 << 5)       /* Select CM7_GPIO3_21 */
#  define GPR_GPR43_GPIO_MUX3_GPIO22_SEL_GPIO3      (0 << 6)       /* Select GPIO3_22 */
#  define GPR_GPR43_GPIO_MUX3_GPIO22_SEL_CM7_GPIO3  (1 << 6)       /* Select CM7_GPIO3_22 */
#  define GPR_GPR43_GPIO_MUX3_GPIO23_SEL_GPIO3      (0 << 7)       /* Select GPIO3_23 */
#  define GPR_GPR43_GPIO_MUX3_GPIO23_SEL_CM7_GPIO3  (1 << 7)       /* Select CM7_GPIO3_23 */
#  define GPR_GPR43_GPIO_MUX3_GPIO24_SEL_GPIO3      (0 << 8)       /* Select GPIO3_24 */
#  define GPR_GPR43_GPIO_MUX3_GPIO24_SEL_CM7_GPIO3  (1 << 8)       /* Select CM7_GPIO3_24 */
#  define GPR_GPR43_GPIO_MUX3_GPIO25_SEL_GPIO3      (0 << 9)       /* Select GPIO3_25 */
#  define GPR_GPR43_GPIO_MUX3_GPIO25_SEL_CM7_GPIO3  (1 << 9)       /* Select CM7_GPIO3_25 */
#  define GPR_GPR43_GPIO_MUX3_GPIO26_SEL_GPIO3      (0 << 10)      /* Select GPIO3_26 */
#  define GPR_GPR43_GPIO_MUX3_GPIO26_SEL_CM7_GPIO3  (1 << 10)      /* Select CM7_GPIO3_26 */
#  define GPR_GPR43_GPIO_MUX3_GPIO27_SEL_GPIO3      (0 << 11)      /* Select GPIO3_27 */
#  define GPR_GPR43_GPIO_MUX3_GPIO27_SEL_CM7_GPIO3  (1 << 11)      /* Select CM7_GPIO3_27 */
#  define GPR_GPR43_GPIO_MUX3_GPIO28_SEL_GPIO3      (0 << 12)      /* Select GPIO3_28 */
#  define GPR_GPR43_GPIO_MUX3_GPIO28_SEL_CM7_GPIO3  (1 << 12)      /* Select CM7_GPIO3_28 */
#  define GPR_GPR43_GPIO_MUX3_GPIO29_SEL_GPIO3      (0 << 13)      /* Select GPIO3_29 */
#  define GPR_GPR43_GPIO_MUX3_GPIO29_SEL_CM7_GPIO3  (1 << 13)      /* Select CM7_GPIO3_29 */
#  define GPR_GPR43_GPIO_MUX3_GPIO30_SEL_GPIO3      (0 << 14)      /* Select GPIO3_40 */
#  define GPR_GPR43_GPIO_MUX3_GPIO30_SEL_CM7_GPIO3  (1 << 14)      /* Select CM7_GPIO3_30 */
#  define GPR_GPR43_GPIO_MUX3_GPIO31_SEL_GPIO3      (0 << 15)      /* Select GPIO3_31 */
#  define GPR_GPR43_GPIO_MUX3_GPIO31_SEL_CM7_GPIO3  (1 << 15)      /* Select CM7_GPIO3_31 */
#  define GPR_GPR43_GPIO_MUX3_GPIO_SEL_GPIO3(n)     (0 << ((n)-16) /* Select GPIO3_n */
#  define GPR_GPR43_GPIO_MUX3_GPIO_SEL_CM7_GPIO3(n) (1 << ((n)-16) /* Select CM7_GPIO3_n */

                                                    /* Bits 16-27: Reserved */
#define GPR_GPR43_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR43_DWP_MASK                (0x03 << GPR_GPR43_DWP_SHIFT)
#  define GPR_GPR43_DWP_ALLOWBOTH         (0x00 << GPR_GPR43_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR43_DWP_DENYCM7           (0x01 << GPR_GPR43_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR43_DWP_DENYCM4           (0x02 << GPR_GPR43_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR43_DWP_DENYBOTH          (0x03 << GPR_GPR43_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR43_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR43_DWP_LOCK_MASK           (0x03 << GPR_GPR43_DWP_LOCK_SHIFT)
#  define GPR_GPR43_DWP_LOCK_NOLCK        (0x00 << GPR_GPR43_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR43_DWP_LOCK_LCKLO        (0x01 << GPR_GPR43_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR43_DWP_LOCK_LCKHI        (0x02 << GPR_GPR43_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR43_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR43_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 44 (GPR44) - Reserved */

/* General Purpose Register 45 (GPR45) - Reserved */

/* General Purpose Register 46 (GPR46) - Reserved */

/* General Purpose Register 47 (GPR47) - Reserved */

/* General Purpose Register 48 (GPR48) - Reserved */

/* General Purpose Register 49 (GPR49) - Reserved */

/* General Purpose Register 50 (GPR50) */

#define GPR_GPR50_CAAM_IPS_MGR_SHIFT      (0)       /* Bits 0-4:   CAAM manager processor identifier (CAAM_IPS_MGR) */
#define GPR_GPR50_CAAM_IPS_MGR_MASK       (0x1f << GPR_GPR50_CAAM_IPS_MGR_SHIFT)
                                                    /* Bits 5-19:  Reserved */
                                                    /* Bits 20-27: Reserved */
#define GPR_GPR50_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR50_DWP_MASK                (0x03 << GPR_GPR50_DWP_SHIFT)
#  define GPR_GPR50_DWP_ALLOWBOTH         (0x00 << GPR_GPR50_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR50_DWP_DENYCM7           (0x01 << GPR_GPR50_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR50_DWP_DENYCM4           (0x02 << GPR_GPR50_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR50_DWP_DENYBOTH          (0x03 << GPR_GPR50_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR50_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR50_DWP_LOCK_MASK           (0x03 << GPR_GPR50_DWP_LOCK_SHIFT)
#  define GPR_GPR50_DWP_LOCK_NOLCK        (0x00 << GPR_GPR50_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR50_DWP_LOCK_LCKLO        (0x01 << GPR_GPR50_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR50_DWP_LOCK_LCKHI        (0x02 << GPR_GPR50_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR50_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR50_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 51 (GPR51) */

#define GPR_GPR51_M7_NMI_CLEAR            (1 << 0)  /* Bit 0:      Clear CM7 NMI holding register (M7_NMI_CLEAR) */
                                                    /* Bits 1-19:  Reserved */
                                                    /* Bits 20-27: Reserved */
#define GPR_GPR51_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR51_DWP_MASK                (0x03 << GPR_GPR51_DWP_SHIFT)
#  define GPR_GPR51_DWP_ALLOWBOTH         (0x00 << GPR_GPR51_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR51_DWP_DENYCM7           (0x01 << GPR_GPR51_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR51_DWP_DENYCM4           (0x02 << GPR_GPR51_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR51_DWP_DENYBOTH          (0x03 << GPR_GPR51_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR51_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR51_DWP_LOCK_MASK           (0x03 << GPR_GPR51_DWP_LOCK_SHIFT)
#  define GPR_GPR51_DWP_LOCK_NOLCK        (0x00 << GPR_GPR51_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR51_DWP_LOCK_LCKLO        (0x01 << GPR_GPR51_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR51_DWP_LOCK_LCKHI        (0x02 << GPR_GPR51_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR51_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR51_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 52 (GPR52) - Reserved */

/* General Purpose Register 53 (GPR53) - Reserved */

/* General Purpose Register 54 (GPR54) - Reserved */

/* General Purpose Register 55 (GPR55) - Reserved */

/* General Purpose Register 59 (GPR59) */

#define GPR_GPR59_MIPI_CSI_AUTO_PD_EN     (1 << 0)  /* Bit 0:      Powers down inactive lanes reported by CSI2X_CFG_NUM_LANES (MIPI_CSI_AUTO_PD_EN) */
#define GPR_GPR59_MIPI_CSI_SOFT_RST_N     (1 << 1)  /* Bit 1:      MIPI CSI APB clock domain and user interface clock domain software reset bit (MIPI_CSI_SOFT_RST_N) */
#define GPR_GPR59_MIPI_CSI_CONT_CLK_MODE  (1 << 2)  /* Bit 2:      Enables the slave clock lane feature to maintain HS reception state during continuous clock mode operation (MIPI_CSI_CONT_CLK_MODE) */
#define GPR_GPR59_MIPI_CSI_DDR_CLK_EN     (1 << 3)  /* Bit 3:      When high, enables received DDR clock on CLK_DRXHS (MIPI_CSI_DDR_CLK_EN) */
#define GPR_GPR59_MIPI_CSI_PD_RX          (1 << 4)  /* Bit 4:      Power down input for MIPI CSI PHY (MIPI_CSI_PD_RX) */
#define GPR_GPR59_MIPI_CSI_RX_ENABLE      (1 << 5)  /* Bit 5:      Assert to enable MIPI CSI receive enable (MIPI_CSI_RX_ENABLE) */
#define GPR_GPR59_MIPI_CSI_RX_RCAL_SHIFT  (6)       /* Bits 6-7:   MIPI CSI PHY on-chip termination control bits (MIPI_CSI_RX_RCAL) */
#define GPR_GPR59_MIPI_CSI_RX_RCAL_MASK   (0x03 << GPR_GPR59_MIPI_CSI_RX_RCAL_SHIFT)
#  define GPR_GPR59_MIPI_CSI_RX_RCAL_20PH (0x00 << GPR_GPR59_MIPI_CSI_RX_RCAL_SHIFT) /* 20% higher than mid range */
#  define GPR_GPR59_MIPI_CSI_RX_RCAL_MID  (0x01 << GPR_GPR59_MIPI_CSI_RX_RCAL_SHIFT) /* Mid range impedance setting */
#  define GPR_GPR59_MIPI_CSI_RX_RCAL_15PL (0x02 << GPR_GPR59_MIPI_CSI_RX_RCAL_SHIFT) /* 15% lower than mid range */
#  define GPR_GPR59_MIPI_CSI_RX_RCAL_25PL (0x03 << GPR_GPR59_MIPI_CSI_RX_RCAL_SHIFT) /* 25% lower than mid range */

#define GPR_GPR59_MIPI_CSI_RXCDRP_SHIFT   (8)       /* Bits 8-9:   Programming bits that adjust the treshold voltage of LP-CD (MIPI_CSI_RXCDRP) */
#define GPR_GPR59_MIPI_CSI_RXCDRP_MASK    (0x03 << GPR_GPR59_MIPI_CSI_RXCDRP_SHIFT)
#  define GPR_GPR59_MIPI_CSI_RXCDRP_344MV (0x00 << GPR_GPR59_MIPI_CSI_RXCDRP_SHIFT) /* 344 mV */
#  define GPR_GPR59_MIPI_CSI_RXCDRP_325MV (0x01 << GPR_GPR59_MIPI_CSI_RXCDRP_SHIFT) /* 325 mV */
#  define GPR_GPR59_MIPI_CSI_RXCDRP_307MV (0x02 << GPR_GPR59_MIPI_CSI_RXCDRP_SHIFT) /* 307 mV */
#  define GPR_GPR59_MIPI_CSI_RXCDRP_INV   (0x03 << GPR_GPR59_MIPI_CSI_RXCDRP_SHIFT) /* Invalid */

#define GPR_GPR59_MIPI_CSI_RXLPRP_SHIFT   (10)      /* Bits 10-11:   Programming bits that adjust the threshold voltage of LP-RX (MIPI_CSI_RXLPRP) */
#define GPR_GPR59_MIPI_CSI_RXLPRP_MASK    (0x03 << GPR_GPR59_MIPI_CSI_RXLPRP_SHIFT)
#  define GPR_GPR59_MIPI_CSI_RXLPRP_B00   (0x00 << GPR_GPR59_MIPI_CSI_RXLPRP_SHIFT) /* High treshold: 782 mV, low treshold: 730 mV */
#  define GPR_GPR59_MIPI_CSI_RXLPRP_B01   (0x01 << GPR_GPR59_MIPI_CSI_RXLPRP_SHIFT) /* High treshold: 745 mV, low treshold: 692 mV */
#  define GPR_GPR59_MIPI_CSI_RXLPRP_B10   (0x02 << GPR_GPR59_MIPI_CSI_RXLPRP_SHIFT) /* High treshold: 708 mV, low treshold: 655 mV */
#  define GPR_GPR59_MIPI_CSI_RXLPRP_B11   (0x03 << GPR_GPR59_MIPI_CSI_RXLPRP_SHIFT) /* High treshold: invalid, low treshold: invalid */

#define GPR_GPR59_MIPI_CSI_S_PRG_RXHS_SETTLE_SHIFT (12) /* Bits 12-17: Bits used to program T_HS_SETTLE (MIPI_CSI_S_PRG_RXHS_SETTLE) */
#define GPR_GPR59_MIPI_CSI_S_PRG_RXHS_SETTLE_MASK  (0x3f << GPR_GPR59_MIPI_CSI_S_PRG_RXHS_SETTLE_SHIFT)

                                                    /* Bits 18-27: Reserved */
#define GPR_GPR59_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR59_DWP_MASK                (0x03 << GPR_GPR59_DWP_SHIFT)
#  define GPR_GPR59_DWP_ALLOWBOTH         (0x00 << GPR_GPR59_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR59_DWP_DENYCM7           (0x01 << GPR_GPR59_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR59_DWP_DENYCM4           (0x02 << GPR_GPR59_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR59_DWP_DENYBOTH          (0x03 << GPR_GPR59_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR59_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR59_DWP_LOCK_MASK           (0x03 << GPR_GPR59_DWP_LOCK_SHIFT)
#  define GPR_GPR59_DWP_LOCK_NOLCK        (0x00 << GPR_GPR59_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR59_DWP_LOCK_LCKLO        (0x01 << GPR_GPR59_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR59_DWP_LOCK_LCKHI        (0x02 << GPR_GPR59_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR59_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR59_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 62 (GPR62) */

#define GPR_GPR60_MIPI_DSI_CLK_TM_SHIFT   (0)       /* Bits 0-2:   MIPI DSI clock lane trimming bits (MIPI_DSI_CLK_TM) */
#define GPR_GPR60_MIPI_DSI_CLK_TM_MASK    (0x07 << GPR_GPR60_MIPI_DSI_CLK_TM_SHIFT)

#define GPR_GPR60_MIPI_DSI_D0_TM_SHIFT    (3)       /* Bits 3-5:   MIPI DSI data lane 0 trimming bits (MIPI_DSI_D0_TM) */
#define GPR_GPR60_MIPI_DSI_D0_TM_MASK     (0x07 << GPR_GPR60_MIPI_DSI_D0_TM_SHIFT)

#define GPR_GPR60_MIPI_DSI_D1_TM_SHIFT    (6)       /* Bits 6-8:   MIPI DSI data lane 1 trimming bits (MIPI_DSI_D1_TM) */
#define GPR_GPR60_MIPI_DSI_D1_TM_MASK     (0x07 << GPR_GPR60_MIPI_DSI_D1_TM_SHIFT)

#define GPR_GPR60_MIPI_DSI_TX_RCAL_SHIFT  (9)       /* Bits 9-10:  MIPI DSI PHY on-chip termination control bits (MIPI_DSI_TX_RCAL) */
#define GPR_GPR60_MIPI_DSI_TX_RCAL_MASK   (0x03 << GPR_GPR60_MIPI_DSI_TX_RCAL_SHIFT)
#  define GPR_GPR60_MIPI_DSI_TX_RCAL_20PH (0x00 << GPR_GPR60_MIPI_DSI_TX_RCAL_SHIFT) /* 20% higher than mid range */
#  define GPR_GPR60_MIPI_DSI_TX_RCAL_MID  (0x01 << GPR_GPR60_MIPI_DSI_TX_RCAL_SHIFT) /* Mid range impedance setting */
#  define GPR_GPR60_MIPI_DSI_TX_RCAL_15PL (0x02 << GPR_GPR60_MIPI_DSI_TX_RCAL_SHIFT) /* 15% lower than mid range */
#  define GPR_GPR60_MIPI_DSI_TX_RCAL_25PL (0x03 << GPR_GPR60_MIPI_DSI_TX_RCAL_SHIFT) /* 25% lower than mid range */

#define GPR_GPR60_MIPI_DSI_TX_ULPS_ENABLE_SHIFT (11) /* Bits 11-13: DSI transmit ULPS mode enable (MIPI_DSI_TX_ULPS_ENABLE) */
#define GPR_GPR60_MIPI_DSI_TX_ULPS_ENABLE_MASK  (0x07 << GPR_GPR60_MIPI_DSI_TX_ULPS_ENABLE_SHIFT)
#  define GPR_GPR60_MIPI_DSI_TX_ULPS_ENABLE_CLK (0x01 << GPR_GPR60_MIPI_DSI_TX_ULPS_ENABLE_SHIFT) /* Clock lane */
#  define GPR_GPR60_MIPI_DSI_TX_ULPS_ENABLE_D0  (0x02 << GPR_GPR60_MIPI_DSI_TX_ULPS_ENABLE_SHIFT) /* Data lane 0 */
#  define GPR_GPR60_MIPI_DSI_TX_ULPS_ENABLE_D1  (0x04 << GPR_GPR60_MIPI_DSI_TX_ULPS_ENABLE_SHIFT) /* Data lane 1 */

                                                    /* Bits 14-15: Reserved */

#define GPR_GPR60_MIPI_DSI_PCLK_SOFT_RESET_N (1 << 16) /* Bit 16:  MIPI DSI APB clock domain software reset bit (MIPI_DSI_PCLK_SOFT_RESET_N) */
#define GPR_GPR60_MIPI_DSI_BYTE_SOFT_RESET_N (1 << 17) /* Bit 17:  MIPI DSI byte clock domain software reset bit (MIPI_DSI_BYTE_SOFT_RESET_N) */
#define GPR_GPR60_MIPI_DSI_DPI_SOFT_RESET_N  (1 << 18) /* Bit 18:  MIPI DSI pixel clock domain software reset bit (MIPI_DSI_DPI_SOFT_RESET_N) */
#define GPR_GPR60_MIPI_DSI_ESC_SOFT_RESET_N  (1 << 19) /* Bit 19:  MIPI DSI escape clock domain software reset bit (MIPI_DSI_ESC_SOFT_RESET_N) */

                                                    /* Bits 20-27: Reserved */
#define GPR_GPR62_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR62_DWP_MASK                (0x03 << GPR_GPR62_DWP_SHIFT)
#  define GPR_GPR62_DWP_ALLOWBOTH         (0x00 << GPR_GPR62_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR62_DWP_DENYCM7           (0x01 << GPR_GPR62_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR62_DWP_DENYCM4           (0x02 << GPR_GPR62_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR62_DWP_DENYBOTH          (0x03 << GPR_GPR62_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR62_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR62_DWP_LOCK_MASK           (0x03 << GPR_GPR62_DWP_LOCK_SHIFT)
#  define GPR_GPR62_DWP_LOCK_NOLCK        (0x00 << GPR_GPR62_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR62_DWP_LOCK_LCKLO        (0x01 << GPR_GPR62_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR62_DWP_LOCK_LCKHI        (0x02 << GPR_GPR62_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR62_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR62_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 63 (GPR63) */

#define GPR_GPR63_MIPI_DSI_TX_ULPS_ACTIVE_SHIFT (0) /* Bits 0-2:   DSI transmit ULPS mode enable (MIPI_DSI_TX_ULPS_ENABLE) */
#define GPR_GPR63_MIPI_DSI_TX_ULPS_ACTIVE_MASK  (0x07 << GPR_GPR63_MIPI_DSI_TX_ULPS_ACTIVE_SHIFT)
#  define GPR_GPR63_MIPI_DSI_TX_ULPS_ACTIVE_CLK (0x01 << GPR_GPR63_MIPI_DSI_TX_ULPS_ACTIVE_SHIFT) /* Clock lane */
#  define GPR_GPR63_MIPI_DSI_TX_ULPS_ACTIVE_D0  (0x02 << GPR_GPR63_MIPI_DSI_TX_ULPS_ACTIVE_SHIFT) /* Data lane 0 */
#  define GPR_GPR63_MIPI_DSI_TX_ULPS_ACTIVE_D1  (0x04 << GPR_GPR63_MIPI_DSI_TX_ULPS_ACTIVE_SHIFT) /* Data lane 1 */

                                                    /* Bits 3-31:  Reserved */

/* General Purpose Register 64 (GPR64) */

#define GPR_GPR64_GPIO_DISP1_FREEZE       (1 << 0)  /* Bit 0:      Compensation code freeze (GPIO_DISP1_FREEZE) */
#define GPR_GPR64_GPIO_DISP1_COMPTQ       (1 << 1)  /* Bit 1:      COMPEN and COMPTQ control the operating modes of the compensation cell (GPIO_DISP1_COMPTQ) */
#define GPR_GPR64_GPIO_DISP1_COMPEN       (1 << 2)  /* Bit 2:      COMPEN and COMPTQ control the operating modes of the compensation cell (GPIO_DISP1_COMPEN) */
#define GPR_GPR64_GPIO_DISP1_FASTFRZ_EN   (1 << 3)  /* Bit 3:      Compensation code fast freeze (GPIO_DISP1_FASTFRZ_EN) */
#define GPR_GPR64_GPIO_DISP1_RASRCP_SHIFT (4)       /* Bits 4-7:   GPIO_DISP_B1 IO bank's 4-bit PMOS compensation codes from core (GPIO_DISP1_RASRCP) */
#define GPR_GPR64_GPIO_DISP1_RASRCP_MASK  (0x0f << GPR_GPR64_GPIO_DISP1_RASRCP_SHIFT)
#define GPR_GPR64_GPIO_DISP1_RASRCN_SHIFT (8)       /* Bits 8-11:  GPIO_DISP_B1 IO bank's 4-bit NMOS compensation codes from core (GPIO_DISP1_RASRCN) */
#define GPR_GPR64_GPIO_DISP1_RASRCN_MASK  (0x0f << GPR_GPR64_GPIO_DISP1_RASRCN_SHIFT)
#define GPR_GPR64_GPIO_DISP1_SELECT_NASRC (1 << 12) /* Bit 12:     GPIO_DISP1_NASRC selection (GPIO_DISP1_SELECT_NASRC) */
#define GPR_GPR64_GPIO_DISP1_REFGEN_SLEEP (1 << 13) /* Bit 13:     GPIO_DISP_B1 IO bank reference voltage generator cell sleep enable (GPIO_DISP1_REFGEN_SLEEP) */

#define GPR_GPR64_GPIO_DISP1_SUPLYDET_LATCH (1 << 14) /* Bit 14:   GPIO_DISP_B1 IO bank power supply mode latch enable (GPIO_DISP1_SUPLYDET_LATCH) */

                                                    /* Bits 15-19: Reserved */

#define GPR_GPR64_GPIO_DISP1_COMPOK       (1 << 20) /* Bit 20:     GPIO_DISP_B1 IO bank compensation OK flag (GPIO_DISP1_COMPOK) */
#define GPR_GPR64_GPIO_DISP1_NASRC_SHIFT  (21)      /* Bits 21-24: GPIO_DISP_B1 IO bank compensation codes (GPIO_DISP1_NASRC) */
#define GPR_GPR64_GPIO_DISP1_NASRC_MASK   (0x0f << GPR_GPR64_GPIO_DISP1_NASRC_SHIFT)

                                                    /* Bits 25-27: Reserved */
#define GPR_GPR64_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR64_DWP_MASK                (0x03 << GPR_GPR64_DWP_SHIFT)
#  define GPR_GPR64_DWP_ALLOWBOTH         (0x00 << GPR_GPR64_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR64_DWP_DENYCM7           (0x01 << GPR_GPR64_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR64_DWP_DENYCM4           (0x02 << GPR_GPR64_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR64_DWP_DENYBOTH          (0x03 << GPR_GPR64_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR64_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR64_DWP_LOCK_MASK           (0x03 << GPR_GPR64_DWP_LOCK_SHIFT)
#  define GPR_GPR64_DWP_LOCK_NOLCK        (0x00 << GPR_GPR64_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR64_DWP_LOCK_LCKLO        (0x01 << GPR_GPR64_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR64_DWP_LOCK_LCKHI        (0x02 << GPR_GPR64_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR64_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR64_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 65 (GPR65) */

#define GPR_GPR65_GPIO_EMC1_FREEZE        (1 << 0)  /* Bit 0:      Compensation code freeze (GPIO_EMC1_FREEZE) */
#define GPR_GPR65_GPIO_EMC1_COMPTQ        (1 << 1)  /* Bit 1:      COMPEN and COMPTQ control the operating modes of the compensation cell (GPIO_EMC1_COMPTQ) */
#define GPR_GPR65_GPIO_EMC1_COMPEN        (1 << 2)  /* Bit 2:      COMPEN and COMPTQ control the operating modes of the compensation cell (GPIO_EMC1_COMPEN) */
#define GPR_GPR65_GPIO_EMC1_FASTFRZ_EN    (1 << 3)  /* Bit 3:      Compensation code fast freeze (GPIO_EMC1_FASTFRZ_EN) */
#define GPR_GPR65_GPIO_EMC1_RASRCP_SHIFT  (4)       /* Bits 4-7:   GPIO_EMC_B1 IO bank's 4-bit PMOS compensation codes from core (GPIO_EMC1_RASRCP) */
#define GPR_GPR65_GPIO_EMC1_RASRCP_MASK   (0x0f << GPR_GPR65_GPIO_EMC1_RASRCP_SHIFT)
#define GPR_GPR65_GPIO_EMC1_RASRCN_SHIFT  (8)       /* Bits 8-11:  GPIO_EMC_B1 IO bank's 4-bit NMOS compensation codes from core (GPIO_EMC1_RASRCN) */
#define GPR_GPR65_GPIO_EMC1_RASRCN_MASK   (0x0f << GPR_GPR65_GPIO_EMC1_RASRCN_SHIFT)
#define GPR_GPR65_GPIO_EMC1_SELECT_NASRC  (1 << 12) /* Bit 12:     GPIO_EMC1_NASRC selection (GPIO_EMC1_SELECT_NASRC) */
#define GPR_GPR65_GPIO_EMC1_REFGEN_SLEEP  (1 << 13) /* Bit 13:     GPIO_EMC_B1 IO bank reference voltage generator cell sleep enable (GPIO_EMC1_REFGEN_SLEEP) */

#define GPR_GPR65_GPIO_EMC1_SUPLYDET_LATCH (1 << 14) /* Bit 14:    GPIO_EMC_B1 IO bank power supply mode latch enable (GPIO_EMC1_SUPLYDET_LATCH) */

                                                    /* Bits 15-19: Reserved */

#define GPR_GPR65_GPIO_EMC1_COMPOK        (1 << 20) /* Bit 20:     GPIO_EMC_B1 IO bank compensation OK flag (GPIO_EMC1_COMPOK) */
#define GPR_GPR65_GPIO_EMC1_NASRC_SHIFT   (21)      /* Bits 21-24: GPIO_EMC_B1 IO bank compensation codes (GPIO_EMC1_NASRC) */
#define GPR_GPR65_GPIO_EMC1_NASRC_MASK    (0x0f << GPR_GPR65_GPIO_EMC1_NASRC_SHIFT)

                                                    /* Bits 25-27: Reserved */
#define GPR_GPR65_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR65_DWP_MASK                (0x03 << GPR_GPR65_DWP_SHIFT)
#  define GPR_GPR65_DWP_ALLOWBOTH         (0x00 << GPR_GPR65_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR65_DWP_DENYCM7           (0x01 << GPR_GPR65_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR65_DWP_DENYCM4           (0x02 << GPR_GPR65_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR65_DWP_DENYBOTH          (0x03 << GPR_GPR65_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR65_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR65_DWP_LOCK_MASK           (0x03 << GPR_GPR65_DWP_LOCK_SHIFT)
#  define GPR_GPR65_DWP_LOCK_NOLCK        (0x00 << GPR_GPR65_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR65_DWP_LOCK_LCKLO        (0x01 << GPR_GPR65_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR65_DWP_LOCK_LCKHI        (0x02 << GPR_GPR65_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR65_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR65_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 66 (GPR66) */

#define GPR_GPR66_GPIO_EMC2_FREEZE        (1 << 0)  /* Bit 0:      Compensation code freeze (GPIO_EMC2_FREEZE) */
#define GPR_GPR66_GPIO_EMC2_COMPTQ        (1 << 1)  /* Bit 1:      COMPEN and COMPTQ control the operating modes of the compensation cell (GPIO_EMC2_COMPTQ) */
#define GPR_GPR66_GPIO_EMC2_COMPEN        (1 << 2)  /* Bit 2:      COMPEN and COMPTQ control the operating modes of the compensation cell (GPIO_EMC2_COMPEN) */
#define GPR_GPR66_GPIO_EMC2_FASTFRZ_EN    (1 << 3)  /* Bit 3:      Compensation code fast freeze (GPIO_EMC2_FASTFRZ_EN) */
#define GPR_GPR66_GPIO_EMC2_RASRCP_SHIFT  (4)       /* Bits 4-7:   GPIO_EMC_B2 IO bank's 4-bit PMOS compensation codes from core (GPIO_EMC2_RASRCP) */
#define GPR_GPR66_GPIO_EMC2_RASRCP_MASK   (0x0f << GPR_GPR66_GPIO_EMC2_RASRCP_SHIFT)
#define GPR_GPR66_GPIO_EMC2_RASRCN_SHIFT  (8)       /* Bits 8-11:  GPIO_EMC_B2 IO bank's 4-bit NMOS compensation codes from core (GPIO_EMC2_RASRCN) */
#define GPR_GPR66_GPIO_EMC2_RASRCN_MASK   (0x0f << GPR_GPR66_GPIO_EMC2_RASRCN_SHIFT)
#define GPR_GPR66_GPIO_EMC2_SELECT_NASRC  (1 << 12) /* Bit 12:     GPIO_EMC2_NASRC selection (GPIO_EMC2_SELECT_NASRC) */
#define GPR_GPR66_GPIO_EMC2_REFGEN_SLEEP  (1 << 13) /* Bit 13:     GPIO_EMC_B2 IO bank reference voltage generator cell sleep enable (GPIO_EMC2_REFGEN_SLEEP) */

#define GPR_GPR66_GPIO_EMC2_SUPLYDET_LATCH (1 << 14) /* Bit 14:    GPIO_EMC_B2 IO bank power supply mode latch enable (GPIO_EMC2_SUPLYDET_LATCH) */

                                                    /* Bits 15-19: Reserved */

#define GPR_GPR66_GPIO_EMC2_COMPOK        (1 << 20) /* Bit 20:     GPIO_EMC_B2 IO bank compensation OK flag (GPIO_EMC2_COMPOK) */
#define GPR_GPR66_GPIO_EMC2_NASRC_SHIFT   (21)      /* Bits 21-24: GPIO_EMC_B2 IO bank compensation codes (GPIO_EMC2_NASRC) */
#define GPR_GPR66_GPIO_EMC2_NASRC_MASK    (0x0f << GPR_GPR66_GPIO_EMC2_NASRC_SHIFT)

                                                    /* Bits 25-27: Reserved */
#define GPR_GPR66_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR66_DWP_MASK                (0x03 << GPR_GPR66_DWP_SHIFT)
#  define GPR_GPR66_DWP_ALLOWBOTH         (0x00 << GPR_GPR66_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR66_DWP_DENYCM7           (0x01 << GPR_GPR66_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR66_DWP_DENYCM4           (0x02 << GPR_GPR66_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR66_DWP_DENYBOTH          (0x03 << GPR_GPR66_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR66_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR66_DWP_LOCK_MASK           (0x03 << GPR_GPR66_DWP_LOCK_SHIFT)
#  define GPR_GPR66_DWP_LOCK_NOLCK        (0x00 << GPR_GPR66_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR66_DWP_LOCK_LCKLO        (0x01 << GPR_GPR66_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR66_DWP_LOCK_LCKHI        (0x02 << GPR_GPR66_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR66_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR66_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 67 (GPR67) */

#define GPR_GPR67_GPIO_SD1_FREEZE         (1 << 0)  /* Bit 0:      Compensation code freeze (GPIO_SD1_FREEZE) */
#define GPR_GPR67_GPIO_SD1_COMPTQ         (1 << 1)  /* Bit 1:      COMPEN and COMPTQ control the operating modes of the compensation cell (GPIO_SD1_COMPTQ) */
#define GPR_GPR67_GPIO_SD1_COMPEN         (1 << 2)  /* Bit 2:      COMPEN and COMPTQ control the operating modes of the compensation cell (GPIO_SD1_COMPEN) */
#define GPR_GPR67_GPIO_SD1_FASTFRZ_EN     (1 << 3)  /* Bit 3:      Compensation code fast freeze (GPIO_SD1_FASTFRZ_EN) */
#define GPR_GPR67_GPIO_SD1_RASRCP_SHIFT   (4)       /* Bits 4-7:   GPIO_SD_B1 IO bank's 4-bit PMOS compensation codes from core (GPIO_SD1_RASRCP) */
#define GPR_GPR67_GPIO_SD1_RASRCP_MASK    (0x0f << GPR_GPR67_GPIO_SD1_RASRCP_SHIFT)
#define GPR_GPR67_GPIO_SD1_RASRCN_SHIFT   (8)       /* Bits 8-11:  GPIO_SD_B1 IO bank's 4-bit NMOS compensation codes from core (GPIO_SD1_RASRCN) */
#define GPR_GPR67_GPIO_SD1_RASRCN_MASK    (0x0f << GPR_GPR67_GPIO_SD1_RASRCN_SHIFT)
#define GPR_GPR67_GPIO_SD1_SELECT_NASRC   (1 << 12) /* Bit 12:     GPIO_SD1_NASRC selection (GPIO_SD1_SELECT_NASRC) */
#define GPR_GPR67_GPIO_SD1_REFGEN_SLEEP   (1 << 13) /* Bit 13:     GPIO_SD_B1 IO bank reference voltage generator cell sleep enable (GPIO_SD1_REFGEN_SLEEP) */

#define GPR_GPR67_GPIO_SD1_SUPLYDET_LATCH (1 << 14) /* Bit 14:    GPIO_SD_B1 IO bank power supply mode latch enable (GPIO_SD1_SUPLYDET_LATCH) */

                                                    /* Bits 15-19: Reserved */

#define GPR_GPR67_GPIO_SD1_COMPOK         (1 << 20) /* Bit 20:     GPIO_SD_B1 IO bank compensation OK flag (GPIO_SD1_COMPOK) */
#define GPR_GPR67_GPIO_SD1_NASRC_SHIFT    (21)      /* Bits 21-24: GPIO_SD_B1 IO bank compensation codes (GPIO_SD1_NASRC) */
#define GPR_GPR67_GPIO_SD1_NASRC_MASK     (0x0f << GPR_GPR67_GPIO_SD1_NASRC_SHIFT)

                                                    /* Bits 25-27: Reserved */
#define GPR_GPR67_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR67_DWP_MASK                (0x03 << GPR_GPR67_DWP_SHIFT)
#  define GPR_GPR67_DWP_ALLOWBOTH         (0x00 << GPR_GPR67_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR67_DWP_DENYCM7           (0x01 << GPR_GPR67_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR67_DWP_DENYCM4           (0x02 << GPR_GPR67_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR67_DWP_DENYBOTH          (0x03 << GPR_GPR67_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR67_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR67_DWP_LOCK_MASK           (0x03 << GPR_GPR67_DWP_LOCK_SHIFT)
#  define GPR_GPR67_DWP_LOCK_NOLCK        (0x00 << GPR_GPR67_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR67_DWP_LOCK_LCKLO        (0x01 << GPR_GPR67_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR67_DWP_LOCK_LCKHI        (0x02 << GPR_GPR67_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR67_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR67_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 68 (GPR68) */

#define GPR_GPR68_GPIO_SD2_FREEZE         (1 << 0)  /* Bit 0:      Compensation code freeze (GPIO_SD2_FREEZE) */
#define GPR_GPR68_GPIO_SD2_COMPTQ         (1 << 1)  /* Bit 1:      COMPEN and COMPTQ control the operating modes of the compensation cell (GPIO_SD2_COMPTQ) */
#define GPR_GPR68_GPIO_SD2_COMPEN         (1 << 2)  /* Bit 2:      COMPEN and COMPTQ control the operating modes of the compensation cell (GPIO_SD2_COMPEN) */
#define GPR_GPR68_GPIO_SD2_FASTFRZ_EN     (1 << 3)  /* Bit 3:      Compensation code fast freeze (GPIO_SD2_FASTFRZ_EN) */
#define GPR_GPR68_GPIO_SD2_RASRCP_SHIFT   (4)       /* Bits 4-7:   GPIO_SD_B2 IO bank's 4-bit PMOS compensation codes from core (GPIO_SD2_RASRCP) */
#define GPR_GPR68_GPIO_SD2_RASRCP_MASK    (0x0f << GPR_GPR68_GPIO_SD2_RASRCP_SHIFT)
#define GPR_GPR68_GPIO_SD2_RASRCN_SHIFT   (8)       /* Bits 8-11:  GPIO_SD_B2 IO bank's 4-bit NMOS compensation codes from core (GPIO_SD2_RASRCN) */
#define GPR_GPR68_GPIO_SD2_RASRCN_MASK    (0x0f << GPR_GPR68_GPIO_SD2_RASRCN_SHIFT)
#define GPR_GPR68_GPIO_SD2_SELECT_NASRC   (1 << 12) /* Bit 12:     GPIO_SD2_NASRC selection (GPIO_SD2_SELECT_NASRC) */
#define GPR_GPR68_GPIO_SD2_REFGEN_SLEEP   (1 << 13) /* Bit 13:     GPIO_SD_B2 IO bank reference voltage generator cell sleep enable (GPIO_SD2_REFGEN_SLEEP) */

#define GPR_GPR68_GPIO_SD2_SUPLYDET_LATCH (1 << 14) /* Bit 14:    GPIO_SD_B2 IO bank power supply mode latch enable (GPIO_SD2_SUPLYDET_LATCH) */

                                                    /* Bits 15-19: Reserved */

#define GPR_GPR68_GPIO_SD2_COMPOK         (1 << 20) /* Bit 20:     GPIO_SD_B2 IO bank compensation OK flag (GPIO_SD2_COMPOK) */
#define GPR_GPR68_GPIO_SD2_NASRC_SHIFT    (21)      /* Bits 21-24: GPIO_SD_B2 IO bank compensation codes (GPIO_SD2_NASRC) */
#define GPR_GPR68_GPIO_SD2_NASRC_MASK     (0x0f << GPR_GPR68_GPIO_SD2_NASRC_SHIFT)

                                                    /* Bits 25-27: Reserved */
#define GPR_GPR68_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR68_DWP_MASK                (0x03 << GPR_GPR68_DWP_SHIFT)
#  define GPR_GPR68_DWP_ALLOWBOTH         (0x00 << GPR_GPR68_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR68_DWP_DENYCM7           (0x01 << GPR_GPR68_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR68_DWP_DENYCM4           (0x02 << GPR_GPR68_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR68_DWP_DENYBOTH          (0x03 << GPR_GPR68_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR68_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR68_DWP_LOCK_MASK           (0x03 << GPR_GPR68_DWP_LOCK_SHIFT)
#  define GPR_GPR68_DWP_LOCK_NOLCK        (0x00 << GPR_GPR68_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR68_DWP_LOCK_LCKLO        (0x01 << GPR_GPR68_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR68_DWP_LOCK_LCKHI        (0x02 << GPR_GPR68_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR68_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR68_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 69 (GPR69) */

                                                    /* Bit 0:      Reserved */
#define GPR_GPR69_GPIO_DISP2_HIGH_RANGE   (1 << 1)  /* Bit 1:      GPIO_DISP_B2 IO bank supply voltage range selection (GPIO_DISP2_HIGH_RANGE) */
#define GPR_GPR69_GPIO_DISP2_LOW_RANGE    (1 << 2)  /* Bit 2:      GPIO_DISP_B2 IO bank supply voltage range selection (GPIO_DISP2_LOW_RANGE) */
                                                    /* Bit 3:      Reserved */
#define GPR_GPR69_GPIO_AD0_HIGH_RANGE     (1 << 4)  /* Bit 4:      GPIO_AD IO bank supply voltage range selection for GPIO_AD_00 to GPIO_AD_17 (GPIO_AD0_HIGH_RANGE) */
#define GPR_GPR69_GPIO_AD0_LOW_RANGE      (1 << 5)  /* Bit 5:      GPIO_AD IO bank supply voltage range selection for GPIO_AD_00 to GPIO_AD_17 (GPIO_AD0_LOW_RANGE) */
                                                    /* Bit 6:      Reserved */
#define GPR_GPR69_GPIO_AD1_HIGH_RANGE     (1 << 7)  /* Bit 7:      GPIO_LPSR IO bank supply voltage range selection for GPIO_AD_18 to GPIO_AD_35 (GPIO_AD1_HIGH_RANGE) */
#define GPR_GPR69_GPIO_AD1_LOW_RANGE      (1 << 8)  /* Bit 8:      GPIO_LPSR IO bank supply voltage range selection for GPIO_AD_18 to GPIO_AD_35 (GPIO_AD1_LOW_RANGE) */
#define GPR_GPR69_SUPLYDET_DISP1_SLEEP    (1 << 9)  /* Bit 9:      GPIO_DISP_B1 IO bank supply voltage detector sleep mode enable (SUPLYDET_DISP1_SLEEP) */
#define GPR_GPR69_SUPLYDET_EMC1_SLEEP     (1 << 10) /* Bit 10:     GPIO_EMC_B1 IO bank supply voltage detector sleep mode enable (SUPLYDET_EMC1_SLEEP) */
#define GPR_GPR69_SUPLYDET_EMC2_SLEEP     (1 << 11) /* Bit 11:     GPIO_EMC_B2 IO bank supply voltage detector sleep mode enable (SUPLYDET_EMC2_SLEEP) */
#define GPR_GPR69_SUPLYDET_SD1_SLEEP      (1 << 12) /* Bit 12:     GPIO_SD_B1 IO bank supply voltage detector sleep mode enable (SUPLYDET_SD1_SLEEP) */
#define GPR_GPR69_SUPLYDET_SD2_SLEEP      (1 << 13) /* Bit 13:     GPIO_SD_B2 IO bank supply voltage detector sleep mode enable (SUPLYDET_SD2_SLEEP) */
                                                    /* Bits 14-15: Reserved */
                                                    /* Bits 16-27: Reserved */
#define GPR_GPR69_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR69_DWP_MASK                (0x03 << GPR_GPR69_DWP_SHIFT)
#  define GPR_GPR69_DWP_ALLOWBOTH         (0x00 << GPR_GPR69_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR69_DWP_DENYCM7           (0x01 << GPR_GPR69_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR69_DWP_DENYCM4           (0x02 << GPR_GPR69_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR69_DWP_DENYBOTH          (0x03 << GPR_GPR69_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR69_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR69_DWP_LOCK_MASK           (0x03 << GPR_GPR69_DWP_LOCK_SHIFT)
#  define GPR_GPR69_DWP_LOCK_NOLCK        (0x00 << GPR_GPR69_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR69_DWP_LOCK_LCKLO        (0x01 << GPR_GPR69_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR69_DWP_LOCK_LCKHI        (0x02 << GPR_GPR69_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR69_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR69_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 70 (GPR70) */

#define GPR_GPR70_ADC1_IPG_DOZE           (1 << 0)  /* Bit 0:      ADC1 doze mode (ADC1_IPG_DOZE) */
#define GPR_GPR70_ADC1_STOP_REQ           (1 << 1)  /* Bit 1:      ADC1 stop request (ADC1_STOP_REQ) */
#define GPR_GPR70_ADC1_IPG_STOP_MODE      (1 << 2)  /* Bit 2:      ADC1 stop mode selection (ADC1_IPG_STOP_MODE) */
#define GPR_GPR70_ADC2_IPG_DOZE           (1 << 3)  /* Bit 3:      ADC2 doze mode (ADC2_IPG_DOZE) */
#define GPR_GPR70_ADC2_STOP_REQ           (1 << 4)  /* Bit 4:      ADC2 stop request (ADC2_STOP_REQ) */
#define GPR_GPR70_ADC2_IPG_STOP_MODE      (1 << 5)  /* Bit 5:      ADC2 stop mode selection (ADC2_IPG_STOP_MODE) */
#define GPR_GPR70_CAAM_IPG_DOZE           (1 << 6)  /* Bit 6:      CAAM doze mode (CAAM_IPG_DOZE) */
#define GPR_GPR70_CAAM_STOP_REQ           (1 << 7)  /* Bit 7:      CAAM stop request (CAAM_STOP_REQ) */
#define GPR_GPR70_CAN1_IPG_DOZE           (1 << 8)  /* Bit 8:      CAN1 doze mode (CAN1_IPG_DOZE) */
#define GPR_GPR70_CAN1_STOP_REQ           (1 << 9)  /* Bit 9:      CAN1 stop request (CAN1_STOP_REQ) */
#define GPR_GPR70_CAN2_IPG_DOZE           (1 << 10) /* Bit 10:     CAN2 doze mode (CAN2_IPG_DOZE) */
#define GPR_GPR70_CAN2_STOP_REQ           (1 << 11) /* Bit 11:     CAN2 stop request (CAN2_STOP_REQ) */
#define GPR_GPR70_CAN3_IPG_DOZE           (1 << 12) /* Bit 12:     CAN3 doze mode (CAN3_IPG_DOZE) */
#define GPR_GPR70_CAN3_STOP_REQ           (1 << 13) /* Bit 13:     CAN3 stop request (CAN3_STOP_REQ) */
                                                    /* Bit 14:     Reserved */
#define GPR_GPR70_EDMA_STOP_REQ           (1 << 15) /* Bit 15:     EDMA stop request (EDMA_STOP_REQ) */
#define GPR_GPR70_EDMA_LPSR_STOP_REQ      (1 << 16) /* Bit 16:     EDMA_LPSR stop request (EDMA_LPSR_STOP_REQ) */
#define GPR_GPR70_ENET_IPG_DOZE           (1 << 17) /* Bit 17:     ENET doze mode (ENET_IPG_DOZE) */
#define GPR_GPR70_ENET_STOP_REQ           (1 << 18) /* Bit 18:     ENET stop request (ENET_STOP_REQ) */
#define GPR_GPR70_ENET1G_IPG_DOZE         (1 << 19) /* Bit 19:     ENET1G doze mode (ENET1G_IPG_DOZE) */
#define GPR_GPR70_ENET1G_STOP_REQ         (1 << 20) /* Bit 20:     ENET1G stop request (ENET1G_STOP_REQ) */
#define GPR_GPR70_FLEXIO1_IPG_DOZE        (1 << 21) /* Bit 21:     FLEXIO1 doze mode (FLEXIO1_IPG_DOZE) */
#define GPR_GPR70_FLEXIO2_IPG_DOZE        (1 << 22) /* Bit 22:     FLEXIO2 doze mode (FLEXIO2_IPG_DOZE) */
#define GPR_GPR70_FLEXSPI1_IPG_DOZE       (1 << 23) /* Bit 23:     FLEXSPI1 doze mode (FLEXSPI1_IPG_DOZE) */
#define GPR_GPR70_FLEXSPI1_STOP_REQ       (1 << 24) /* Bit 24:     FLEXSPI1 stop request (FLEXSPI1_STOP_REQ) */
#define GPR_GPR70_FLEXSPI2_IPG_DOZE       (1 << 25) /* Bit 25:     FLEXSPI2 doze mode (FLEXSPI2_IPG_DOZE) */
#define GPR_GPR70_FLEXSPI2_STOP_REQ       (1 << 26) /* Bit 26:     FLEXSPI2 stop request (FLEXSPI2_STOP_REQ) */
                                                    /* Bit 27:     Reserved */
#define GPR_GPR70_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR70_DWP_MASK                (0x03 << GPR_GPR70_DWP_SHIFT)
#  define GPR_GPR70_DWP_ALLOWBOTH         (0x00 << GPR_GPR70_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR70_DWP_DENYCM7           (0x01 << GPR_GPR70_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR70_DWP_DENYCM4           (0x02 << GPR_GPR70_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR70_DWP_DENYBOTH          (0x03 << GPR_GPR70_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR70_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR70_DWP_LOCK_MASK           (0x03 << GPR_GPR70_DWP_LOCK_SHIFT)
#  define GPR_GPR70_DWP_LOCK_NOLCK        (0x00 << GPR_GPR70_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR70_DWP_LOCK_LCKLO        (0x01 << GPR_GPR70_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR70_DWP_LOCK_LCKHI        (0x02 << GPR_GPR70_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR70_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR70_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 71 (GPR71) */

#define GPR_GPR71_GPT1_IPG_DOZE           (1 << 0)  /* Bit 0:      GPT1 doze mode (GPT1_IPG_DOZE) */
#define GPR_GPR71_GPT2_IPG_DOZE           (1 << 1)  /* Bit 1:      GPT2 doze mode (GPT2_IPG_DOZE) */
#define GPR_GPR71_GPT3_IPG_DOZE           (1 << 2)  /* Bit 2:      GPT3 doze mode (GPT3_IPG_DOZE) */
#define GPR_GPR71_GPT4_IPG_DOZE           (1 << 3)  /* Bit 3:      GPT4 doze mode (GPT4_IPG_DOZE) */
#define GPR_GPR71_GPT5_IPG_DOZE           (1 << 4)  /* Bit 4:      GPT5 doze mode (GPT5_IPG_DOZE) */
#define GPR_GPR71_GPT6_IPG_DOZE           (1 << 5)  /* Bit 5:      GPT6 doze mode (GPT6_IPG_DOZE) */
#define GPR_GPR71_LPI2C1_IPG_DOZE         (1 << 6)  /* Bit 6:      LPI2C1 doze mode (LPI2C1_IPG_DOZE) */
#define GPR_GPR71_LPI2C1_STOP_REQ         (1 << 7)  /* Bit 7:      LPI2C1 stop request (LPI2C1_STOP_REQ) */
#define GPR_GPR71_LPI2C1_IPG_STOP_MODE    (1 << 8)  /* Bit 8:      LPI2C1 stop mode selection (LPI2C1_IPG_STOP_MODE) */
#define GPR_GPR71_LPI2C2_IPG_DOZE         (1 << 9)  /* Bit 9:      LPI2C2 doze mode (LPI2C2_IPG_DOZE) */
#define GPR_GPR71_LPI2C2_STOP_REQ         (1 << 10) /* Bit 10:     LPI2C2 stop request (LPI2C2_STOP_REQ) */
#define GPR_GPR71_LPI2C2_IPG_STOP_MODE    (1 << 11) /* Bit 11:     LPI2C2 stop mode selection (LPI2C2_IPG_STOP_MODE) */
#define GPR_GPR71_LPI2C3_IPG_DOZE         (1 << 12) /* Bit 12:     LPI2C3 doze mode (LPI2C3_IPG_DOZE) */
#define GPR_GPR71_LPI2C3_STOP_REQ         (1 << 13) /* Bit 13:     LPI2C3 stop request (LPI2C3_STOP_REQ) */
#define GPR_GPR71_LPI2C3_IPG_STOP_MODE    (1 << 14) /* Bit 14:     LPI2C3 stop mode selection (LPI2C3_IPG_STOP_MODE) */
#define GPR_GPR71_LPI2C4_IPG_DOZE         (1 << 15) /* Bit 15:     LPI2C4 doze mode (LPI2C4_IPG_DOZE) */
#define GPR_GPR71_LPI2C4_STOP_REQ         (1 << 16) /* Bit 16:     LPI2C4 stop request (LPI2C4_STOP_REQ) */
#define GPR_GPR71_LPI2C4_IPG_STOP_MODE    (1 << 17) /* Bit 17:     LPI2C4 stop mode selection (LPI2C4_IPG_STOP_MODE) */
#define GPR_GPR71_LPI2C5_IPG_DOZE         (1 << 18) /* Bit 18:     LPI2C5 doze mode (LPI2C5_IPG_DOZE) */
#define GPR_GPR71_LPI2C5_STOP_REQ         (1 << 19) /* Bit 19:     LPI2C5 stop request (LPI2C5_STOP_REQ) */
#define GPR_GPR71_LPI2C5_IPG_STOP_MODE    (1 << 20) /* Bit 20:     LPI2C5 stop mode selection (LPI2C5_IPG_STOP_MODE) */
#define GPR_GPR71_LPI2C6_IPG_DOZE         (1 << 21) /* Bit 21:     LPI2C6 doze mode (LPI2C6_IPG_DOZE) */
#define GPR_GPR71_LPI2C6_STOP_REQ         (1 << 22) /* Bit 22:     LPI2C6 stop request (LPI2C6_STOP_REQ) */
#define GPR_GPR71_LPI2C6_IPG_STOP_MODE    (1 << 23) /* Bit 23:     LPI2C6 stop mode selection (LPI2C6_IPG_STOP_MODE) */
#define GPR_GPR71_LPSPI1_IPG_DOZE         (1 << 24) /* Bit 24:     LPSPI1 doze mode (LPSPI1_IPG_DOZE) */
#define GPR_GPR71_LPSPI1_STOP_REQ         (1 << 25) /* Bit 25:     LPSPI1 stop request (LPSPI1_STOP_REQ) */
#define GPR_GPR71_LPSPI1_IPG_STOP_MODE    (1 << 26) /* Bit 26:     LPSPI1 stop mode selection (LPSPI1_IPG_STOP_MODE) */
                                                    /* Bit 27:     Reserved */
#define GPR_GPR71_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR71_DWP_MASK                (0x03 << GPR_GPR71_DWP_SHIFT)
#  define GPR_GPR71_DWP_ALLOWBOTH         (0x00 << GPR_GPR71_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR71_DWP_DENYCM7           (0x01 << GPR_GPR71_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR71_DWP_DENYCM4           (0x02 << GPR_GPR71_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR71_DWP_DENYBOTH          (0x03 << GPR_GPR71_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR71_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR71_DWP_LOCK_MASK           (0x03 << GPR_GPR71_DWP_LOCK_SHIFT)
#  define GPR_GPR71_DWP_LOCK_NOLCK        (0x00 << GPR_GPR71_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR71_DWP_LOCK_LCKLO        (0x01 << GPR_GPR71_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR71_DWP_LOCK_LCKHI        (0x02 << GPR_GPR71_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR71_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR71_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 72 (GPR72) */

#define GPR_GPR72_LPSPI2_IPG_DOZE         (1 << 0)  /* Bit 0:      LPSPI2 doze mode (LPSPI2_IPG_DOZE) */
#define GPR_GPR72_LPSPI2_STOP_REQ         (1 << 1)  /* Bit 1:      LPSPI2 stop request (LPSPI2_STOP_REQ) */
#define GPR_GPR72_LPSPI2_IPG_STOP_MODE    (1 << 2)  /* Bit 2:      LPSPI2 stop mode selection (LPSPI2_IPG_STOP_MODE) */
#define GPR_GPR72_LPSPI3_IPG_DOZE         (1 << 3)  /* Bit 3:      LPSPI3 doze mode (LPSPI3_IPG_DOZE) */
#define GPR_GPR72_LPSPI3_STOP_REQ         (1 << 4)  /* Bit 4:      LPSPI3 stop request (LPSPI3_STOP_REQ) */
#define GPR_GPR72_LPSPI3_IPG_STOP_MODE    (1 << 5)  /* Bit 5:      LPSPI3 stop mode selection (LPSPI3_IPG_STOP_MODE) */
#define GPR_GPR72_LPSPI4_IPG_DOZE         (1 << 6)  /* Bit 6:      LPSPI4 doze mode (LPSPI4_IPG_DOZE) */
#define GPR_GPR72_LPSPI4_STOP_REQ         (1 << 7)  /* Bit 7:      LPSPI4 stop request (LPSPI4_STOP_REQ) */
#define GPR_GPR72_LPSPI4_IPG_STOP_MODE    (1 << 8)  /* Bit 8:      LPSPI4 stop mode selection (LPSPI4_IPG_STOP_MODE) */
#define GPR_GPR72_LPSPI5_IPG_DOZE         (1 << 9)  /* Bit 9:      LPSPI5 doze mode (LPSPI5_IPG_DOZE) */
#define GPR_GPR72_LPSPI5_STOP_REQ         (1 << 10) /* Bit 10:     LPSPI5 stop request (LPSPI5_STOP_REQ) */
#define GPR_GPR72_LPSPI5_IPG_STOP_MODE    (1 << 11) /* Bit 11:     LPSPI5 stop mode selection (LPSPI5_IPG_STOP_MODE) */
#define GPR_GPR72_LPSPI6_IPG_DOZE         (1 << 12) /* Bit 12:     LPSPI6 doze mode (LPSPI6_IPG_DOZE) */
#define GPR_GPR72_LPSPI6_STOP_REQ         (1 << 13) /* Bit 13:     LPSPI6 stop request (LPSPI6_STOP_REQ) */
#define GPR_GPR72_LPSPI6_IPG_STOP_MODE    (1 << 14) /* Bit 14:     LPSPI6 stop mode selection (LPSPI6_IPG_STOP_MODE) */
#define GPR_GPR72_LPUART1_IPG_DOZE        (1 << 15) /* Bit 15:     LPUART1 doze mode (LPUART1_IPG_DOZE) */
#define GPR_GPR72_LPUART1_STOP_REQ        (1 << 16) /* Bit 16:     LPUART1 stop request (LPUART1_STOP_REQ) */
#define GPR_GPR72_LPUART1_IPG_STOP_MODE   (1 << 17) /* Bit 17:     LPUART1 stop mode selection (LPUART1_IPG_STOP_MODE) */
#define GPR_GPR72_LPUART2_IPG_DOZE        (1 << 18) /* Bit 18:     LPUART2 doze mode (LPUART2_IPG_DOZE) */
#define GPR_GPR72_LPUART2_STOP_REQ        (1 << 19) /* Bit 19:     LPUART2 stop request (LPUART2_STOP_REQ) */
#define GPR_GPR72_LPUART2_IPG_STOP_MODE   (1 << 20) /* Bit 20:     LPUART2 stop mode selection (LPUART2_IPG_STOP_MODE) */
#define GPR_GPR72_LPUART3_IPG_DOZE        (1 << 21) /* Bit 21:     LPUART3 doze mode (LPUART3_IPG_DOZE) */
#define GPR_GPR72_LPUART3_STOP_REQ        (1 << 22) /* Bit 22:     LPUART3 stop request (LPUART3_STOP_REQ) */
#define GPR_GPR72_LPUART3_IPG_STOP_MODE   (1 << 23) /* Bit 23:     LPUART3 stop mode selection (LPUART3_IPG_STOP_MODE) */
#define GPR_GPR72_LPUART4_IPG_DOZE        (1 << 24) /* Bit 24:     LPUART4 doze mode (LPUART4_IPG_DOZE) */
#define GPR_GPR72_LPUART4_STOP_REQ        (1 << 25) /* Bit 25:     LPUART4 stop request (LPUART4_STOP_REQ) */
#define GPR_GPR72_LPUART4_IPG_STOP_MODE   (1 << 26) /* Bit 26:     LPUART4 stop mode selection (LPUART4_IPG_STOP_MODE) */
                                                    /* Bit 27:     Reserved */
#define GPR_GPR72_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR72_DWP_MASK                (0x03 << GPR_GPR72_DWP_SHIFT)
#  define GPR_GPR72_DWP_ALLOWBOTH         (0x00 << GPR_GPR72_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR72_DWP_DENYCM7           (0x01 << GPR_GPR72_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR72_DWP_DENYCM4           (0x02 << GPR_GPR72_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR72_DWP_DENYBOTH          (0x03 << GPR_GPR72_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR72_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR72_DWP_LOCK_MASK           (0x03 << GPR_GPR72_DWP_LOCK_SHIFT)
#  define GPR_GPR72_DWP_LOCK_NOLCK        (0x00 << GPR_GPR72_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR72_DWP_LOCK_LCKLO        (0x01 << GPR_GPR72_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR72_DWP_LOCK_LCKHI        (0x02 << GPR_GPR72_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR72_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR72_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 73 (GPR73) */

#define GPR_GPR73_LPUART5_IPG_DOZE        (1 << 0)  /* Bit 0:      LPUART5 doze mode (LPUART5_IPG_DOZE) */
#define GPR_GPR73_LPUART5_STOP_REQ        (1 << 1)  /* Bit 1:      LPUART5 stop request (LPUART5_STOP_REQ) */
#define GPR_GPR73_LPUART5_IPG_STOP_MODE   (1 << 2)  /* Bit 2:      LPUART5 stop mode selection (LPUART5_IPG_STOP_MODE) */
#define GPR_GPR73_LPUART6_IPG_DOZE        (1 << 3)  /* Bit 3:      LPUART6 doze mode (LPUART6_IPG_DOZE) */
#define GPR_GPR73_LPUART6_STOP_REQ        (1 << 4)  /* Bit 4:      LPUART6 stop request (LPUART6_STOP_REQ) */
#define GPR_GPR73_LPUART6_IPG_STOP_MODE   (1 << 5)  /* Bit 5:      LPUART6 stop mode selection (LPUART6_IPG_STOP_MODE) */
#define GPR_GPR73_LPUART7_IPG_DOZE        (1 << 6)  /* Bit 6:      LPUART7 doze mode (LPUART7_IPG_DOZE) */
#define GPR_GPR73_LPUART7_STOP_REQ        (1 << 7)  /* Bit 7:      LPUART7 stop request (LPUART7_STOP_REQ) */
#define GPR_GPR73_LPUART7_IPG_STOP_MODE   (1 << 8)  /* Bit 8:      LPUART7 stop mode selection (LPUART7_IPG_STOP_MODE) */
#define GPR_GPR73_LPUART8_IPG_DOZE        (1 << 9)  /* Bit 9:      LPUART8 doze mode (LPUART8_IPG_DOZE) */
#define GPR_GPR73_LPUART8_STOP_REQ        (1 << 10) /* Bit 10:     LPUART8 stop request (LPUART8_STOP_REQ) */
#define GPR_GPR73_LPUART8_IPG_STOP_MODE   (1 << 11) /* Bit 11:     LPUART8 stop mode selection (LPUART8_IPG_STOP_MODE) */
#define GPR_GPR73_LPUART9_IPG_DOZE        (1 << 12) /* Bit 12:     LPUART9 doze mode (LPUART9_IPG_DOZE) */
#define GPR_GPR73_LPUART9_STOP_REQ        (1 << 13) /* Bit 13:     LPUART9 stop request (LPUART9_STOP_REQ) */
#define GPR_GPR73_LPUART9_IPG_STOP_MODE   (1 << 14) /* Bit 14:     LPUART9 stop mode selection (LPUART9_IPG_STOP_MODE) */
#define GPR_GPR73_LPUART10_IPG_DOZE       (1 << 15) /* Bit 15:     LPUART10 doze mode (LPUART10_IPG_DOZE) */
#define GPR_GPR73_LPUART10_STOP_REQ       (1 << 16) /* Bit 16:     LPUART10 stop request (LPUART10_STOP_REQ) */
#define GPR_GPR73_LPUART10_IPG_STOP_MODE  (1 << 17) /* Bit 17:     LPUART10 stop mode selection (LPUART10_IPG_STOP_MODE) */
#define GPR_GPR73_LPUART11_IPG_DOZE       (1 << 18) /* Bit 18:     LPUART11 doze mode (LPUART11_IPG_DOZE) */
#define GPR_GPR73_LPUART11_STOP_REQ       (1 << 19) /* Bit 19:     LPUART11 stop request (LPUART11_STOP_REQ) */
#define GPR_GPR73_LPUART11_IPG_STOP_MODE  (1 << 20) /* Bit 20:     LPUART11 stop mode selection (LPUART11_IPG_STOP_MODE) */
#define GPR_GPR73_LPUART12_IPG_DOZE       (1 << 21) /* Bit 21:     LPUART12 doze mode (LPUART12_IPG_DOZE) */
#define GPR_GPR73_LPUART12_STOP_REQ       (1 << 22) /* Bit 22:     LPUART12 stop request (LPUART12_STOP_REQ) */
#define GPR_GPR73_LPUART12_IPG_STOP_MODE  (1 << 23) /* Bit 23:     LPUART12 stop mode selection (LPUART12_IPG_STOP_MODE) */
#define GPR_GPR73_MIC_IPG_DOZE            (1 << 24) /* Bit 24:     MIC doze mode (MIC_IPG_DOZE) */
#define GPR_GPR73_MIC_STOP_REQ            (1 << 25) /* Bit 25:     MIC stop request (MIC_STOP_REQ) */
#define GPR_GPR73_MIC_IPG_STOP_MODE       (1 << 26) /* Bit 26:     MIC stop mode selection (MIC_IPG_STOP_MODE) */
                                                    /* Bit 27:     Reserved */
#define GPR_GPR73_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR73_DWP_MASK                (0x03 << GPR_GPR73_DWP_SHIFT)
#  define GPR_GPR73_DWP_ALLOWBOTH         (0x00 << GPR_GPR73_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR73_DWP_DENYCM7           (0x01 << GPR_GPR73_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR73_DWP_DENYCM4           (0x02 << GPR_GPR73_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR73_DWP_DENYBOTH          (0x03 << GPR_GPR73_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR73_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR73_DWP_LOCK_MASK           (0x03 << GPR_GPR73_DWP_LOCK_SHIFT)
#  define GPR_GPR73_DWP_LOCK_NOLCK        (0x00 << GPR_GPR73_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR73_DWP_LOCK_LCKLO        (0x01 << GPR_GPR73_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR73_DWP_LOCK_LCKHI        (0x02 << GPR_GPR73_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR73_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR73_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 74 (GPR74) */

                                                    /* Bit 0:      Reserved */
#define GPR_GPR74_PIT1_STOP_REQ           (1 << 1)  /* Bit 1:      PIT1 stop request (PIT1_STOP_REQ) */
#define GPR_GPR74_PIT2_STOP_REQ           (1 << 2)  /* Bit 2:      PIT2 stop request (PIT2_STOP_REQ) */
#define GPR_GPR74_SEMC_STOP_REQ           (1 << 3)  /* Bit 3:      SEMC stop request (SEMC_STOP_REQ) */
#define GPR_GPR74_SIM1_IPG_DOZE           (1 << 4)  /* Bit 4:      SIM1 doze mode (SIM1_IPG_DOZE) */
#define GPR_GPR74_SIM2_IPG_DOZE           (1 << 5)  /* Bit 5:      SIM2 doze mode (SIM2_IPG_DOZE) */
#define GPR_GPR74_SNVS_HP_IPG_DOZE        (1 << 6)  /* Bit 6:      SNVS_HP doze mode (SNVS_HP_IPG_DOZE) */
#define GPR_GPR74_SNVS_HP_STOP_REQ        (1 << 7)  /* Bit 7:      SNVS_HP stop request (SNVS_HP_STOP_REQ) */
#define GPR_GPR74_WDOG1_IPG_DOZE          (1 << 8)  /* Bit 8:      WDOG1 doze mode (WDOG1_IPG_DOZE) */
#define GPR_GPR74_WDOG2_IPG_DOZE          (1 << 9)  /* Bit 9:      WDOG2 doze mode (WDOG2_IPG_DOZE) */
#define GPR_GPR74_SAI1_STOP_REQ           (1 << 10) /* Bit 10:     SAI1 stop request (SAI1_STOP_REQ) */
#define GPR_GPR74_SAI2_STOP_REQ           (1 << 11) /* Bit 11:     SAI2 stop request (SAI2_STOP_REQ) */
#define GPR_GPR74_SAI3_STOP_REQ           (1 << 12) /* Bit 12:     SAI3 stop request (SAI3_STOP_REQ) */
#define GPR_GPR74_SAI4_STOP_REQ           (1 << 13) /* Bit 13:     SAI4 stop request (SAI4_STOP_REQ) */
#define GPR_GPR74_FLEXIO1_STOP_REQ_BUS    (1 << 14) /* Bit 14:     FLEXIO1 bus clock domain stop request (FLEXIO1_STOP_REQ_BUS) */
#define GPR_GPR74_FLEXIO1_STOP_REQ_PER    (1 << 15) /* Bit 15:     FLEXIO1 peripheral clock domain stop request (FLEXIO1_STOP_REQ_PER) */
#define GPR_GPR74_FLEXIO2_STOP_REQ_BUS    (1 << 16) /* Bit 16:     FLEXIO2 bus clock domain stop request (FLEXIO2_STOP_REQ_BUS) */
#define GPR_GPR74_FLEXIO2_STOP_REQ_PER    (1 << 17) /* Bit 17:     FLEXIO2 peripheral clock domain stop request (FLEXIO2_STOP_REQ_PER) */
                                                    /* Bits 18-27: Reserved */
#define GPR_GPR74_DWP_SHIFT               (28)      /* Bits 28-29: Domain write protection (DWP) */
#define GPR_GPR74_DWP_MASK                (0x03 << GPR_GPR74_DWP_SHIFT)
#  define GPR_GPR74_DWP_ALLOWBOTH         (0x00 << GPR_GPR74_DWP_SHIFT) /* Both cores are allowed */
#  define GPR_GPR74_DWP_DENYCM7           (0x01 << GPR_GPR74_DWP_SHIFT) /* CM7 is forbidden */
#  define GPR_GPR74_DWP_DENYCM4           (0x02 << GPR_GPR74_DWP_SHIFT) /* CM4 is forbidden */
#  define GPR_GPR74_DWP_DENYBOTH          (0x03 << GPR_GPR74_DWP_SHIFT) /* Both cores are forbidden */

#define GPR_GPR74_DWP_LOCK_SHIFT          (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define GPR_GPR74_DWP_LOCK_MASK           (0x03 << GPR_GPR74_DWP_LOCK_SHIFT)
#  define GPR_GPR74_DWP_LOCK_NOLCK        (0x00 << GPR_GPR74_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define GPR_GPR74_DWP_LOCK_LCKLO        (0x01 << GPR_GPR74_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define GPR_GPR74_DWP_LOCK_LCKHI        (0x02 << GPR_GPR74_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define GPR_GPR74_DWP_LOCK_LCKBOTH      (0x03 << GPR_GPR74_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* General Purpose Register 75 (GPR75) */

#define GPR_GPR75_ADC1_STOP_ACK           (1 << 0)  /* Bit 0:      ADC1 stop acknowledge (ADC1_STOP_ACK) */
#define GPR_GPR75_ADC2_STOP_ACK           (1 << 1)  /* Bit 1:      ADC2 stop acknowledge (ADC2_STOP_ACK) */
#define GPR_GPR75_CAAM_STOP_ACK           (1 << 2)  /* Bit 2:      CAAM stop acknowledge (CAAM_STOP_ACK) */
#define GPR_GPR75_CAN1_STOP_ACK           (1 << 3)  /* Bit 3:      CAN1 stop acknowledge (CAN1_STOP_ACK) */
#define GPR_GPR75_CAN2_STOP_ACK           (1 << 4)  /* Bit 4:      CAN2 stop acknowledge (CAN2_STOP_ACK) */
#define GPR_GPR75_CAN3_STOP_ACK           (1 << 5)  /* Bit 5:      CAN3 stop acknowledge (CAN3_STOP_ACK) */
#define GPR_GPR75_EDMA_STOP_ACK           (1 << 6)  /* Bit 6:      EDMA stop acknowledge (EDMA_STOP_ACK) */
#define GPR_GPR75_EDMA_LPSR_STOP_ACK      (1 << 7)  /* Bit 7:      EDMA_LPSR stop acknowledge (EDMA_LPSR_STOP_ACK) */
#define GPR_GPR75_ENET_STOP_ACK           (1 << 8)  /* Bit 8:      ENET stop acknowledge (ENET_STOP_ACK) */
#define GPR_GPR75_ENET1G_STOP_ACK         (1 << 9)  /* Bit 9:      ENET1G stop acknowledge (ENET1G_STOP_ACK) */
#define GPR_GPR75_FLEXSPI1_STOP_ACK       (1 << 10) /* Bit 10:     FLEXSPI1 stop acknowledge (FLEXSPI1_STOP_ACK) */
#define GPR_GPR75_FLEXSPI2_STOP_ACK       (1 << 11) /* Bit 11:     FLEXSPI2 stop acknowledge (FLEXSPI2_STOP_ACK) */
#define GPR_GPR75_LPI2C1_STOP_ACK         (1 << 12) /* Bit 12:     LPI2C1 stop acknowledge (LPI2C1_STOP_ACK) */
#define GPR_GPR75_LPI2C2_STOP_ACK         (1 << 13) /* Bit 13:     LPI2C2 stop acknowledge (LPI2C2_STOP_ACK) */
#define GPR_GPR75_LPI2C3_STOP_ACK         (1 << 14) /* Bit 14:     LPI2C3 stop acknowledge (LPI2C3_STOP_ACK) */
#define GPR_GPR75_LPI2C4_STOP_ACK         (1 << 15) /* Bit 15:     LPI2C4 stop acknowledge (LPI2C4_STOP_ACK) */
#define GPR_GPR75_LPI2C5_STOP_ACK         (1 << 16) /* Bit 16:     LPI2C5 stop acknowledge (LPI2C5_STOP_ACK) */
#define GPR_GPR75_LPI2C6_STOP_ACK         (1 << 17) /* Bit 17:     LPI2C6 stop acknowledge (LPI2C6_STOP_ACK) */
#define GPR_GPR75_LPSPI1_STOP_ACK         (1 << 18) /* Bit 18:     LPSPI1 stop acknowledge (LPSPI1_STOP_ACK) */
#define GPR_GPR75_LPSPI2_STOP_ACK         (1 << 19) /* Bit 19:     LPSPI2 stop acknowledge (LPSPI2_STOP_ACK) */
#define GPR_GPR75_LPSPI3_STOP_ACK         (1 << 20) /* Bit 20:     LPSPI3 stop acknowledge (LPSPI3_STOP_ACK) */
#define GPR_GPR75_LPSPI4_STOP_ACK         (1 << 21) /* Bit 21:     LPSPI4 stop acknowledge (LPSPI4_STOP_ACK) */
#define GPR_GPR75_LPSPI5_STOP_ACK         (1 << 22) /* Bit 22:     LPSPI5 stop acknowledge (LPSPI5_STOP_ACK) */
#define GPR_GPR75_LPSPI6_STOP_ACK         (1 << 23) /* Bit 23:     LPSPI6 stop acknowledge (LPSPI6_STOP_ACK) */
#define GPR_GPR75_LPUART1_STOP_ACK        (1 << 24) /* Bit 24:     LPUART1 stop acknowledge (LPUART1_STOP_ACK) */
#define GPR_GPR75_LPUART2_STOP_ACK        (1 << 25) /* Bit 25:     LPUART2 stop acknowledge (LPUART2_STOP_ACK) */
#define GPR_GPR75_LPUART3_STOP_ACK        (1 << 26) /* Bit 26:     LPUART3 stop acknowledge (LPUART3_STOP_ACK) */
#define GPR_GPR75_LPUART4_STOP_ACK        (1 << 27) /* Bit 27:     LPUART4 stop acknowledge (LPUART4_STOP_ACK) */
#define GPR_GPR75_LPUART5_STOP_ACK        (1 << 28) /* Bit 28:     LPUART5 stop acknowledge (LPUART5_STOP_ACK) */
#define GPR_GPR75_LPUART6_STOP_ACK        (1 << 29) /* Bit 29:     LPUART6 stop acknowledge (LPUART6_STOP_ACK) */
#define GPR_GPR75_LPUART7_STOP_ACK        (1 << 30) /* Bit 30:     LPUART7 stop acknowledge (LPUART7_STOP_ACK) */
#define GPR_GPR75_LPUART8_STOP_ACK        (1 << 31) /* Bit 31:     LPUART8 stop acknowledge (LPUART8_STOP_ACK) */

/* General Purpose Register 76 (GPR76) */

#define GPR_GPR76_LPUART9_STOP_ACK        (1 << 0)  /* Bit 0:      LPUART9 stop acknowledge (LPUART9_STOP_ACK) */
#define GPR_GPR76_LPUART10_STOP_ACK       (1 << 1)  /* Bit 1:      LPUART10 stop acknowledge (LPUART10_STOP_ACK) */
#define GPR_GPR76_LPUART11_STOP_ACK       (1 << 2)  /* Bit 2:      LPUART11 stop acknowledge (LPUART11_STOP_ACK) */
#define GPR_GPR76_LPUART12_STOP_ACK       (1 << 3)  /* Bit 3:      LPUART12 stop acknowledge (LPUART12_STOP_ACK) */
#define GPR_GPR76_MIC_STOP_ACK            (1 << 4)  /* Bit 4:      MIC stop acknowledge (MIC_STOP_ACK) */
#define GPR_GPR76_PIT1_STOP_ACK           (1 << 5)  /* Bit 5:      PIT1 stop acknowledge (PIT1_STOP_ACK) */
#define GPR_GPR76_PIT2_STOP_ACK           (1 << 6)  /* Bit 6:      PIT2 stop acknowledge (PIT2_STOP_ACK) */
#define GPR_GPR76_SEMC_STOP_ACK           (1 << 7)  /* Bit 7:      SEMC stop acknowledge (SEMC_STOP_ACK) */
#define GPR_GPR76_SNVS_HP_STOP_ACK        (1 << 8)  /* Bit 8:      SNVS_HP stop acknowledge (SNVS_HP_STOP_ACK) */
#define GPR_GPR76_SAI1_STOP_ACK           (1 << 9)  /* Bit 9:      SAI1 stop acknowledge (SAI1_STOP_ACK) */
#define GPR_GPR76_SAI2_STOP_ACK           (1 << 10) /* Bit 10:     SAI2 stop acknowledge (SAI2_STOP_ACK) */
#define GPR_GPR76_SAI3_STOP_ACK           (1 << 11) /* Bit 11:     SAI3 stop acknowledge (SAI3_STOP_ACK) */
#define GPR_GPR76_SAI4_STOP_ACK           (1 << 12) /* Bit 12:     SAI4 stop acknowledge (SAI4_STOP_ACK) */
#define GPR_GPR76_FLEXIO1_STOP_ACK_BUS    (1 << 13) /* Bit 13:     FLEXIO1 stop acknowledge of bus clock domain (FLEXIO1_STOP_ACK_BUS) */
#define GPR_GPR76_FLEXIO1_STOP_ACK_PER    (1 << 14) /* Bit 14:     FLEXIO1 stop acknowledge of peripheral clock domain (FLEXIO1_STOP_ACK_PER) */
#define GPR_GPR76_FLEXIO2_STOP_ACK_BUS    (1 << 15) /* Bit 15:     FLEXIO2 stop acknowledge of bus clock domain (FLEXIO2_STOP_ACK_BUS) */
#define GPR_GPR76_FLEXIO2_STOP_ACK_PER    (1 << 16) /* Bit 16:     FLEXIO2 stop acknowledge of peripheral clock domain (FLEXIO2_STOP_ACK_PER) */
                                                    /* Bits 17-31: Reserved */

/* LPSR General Purpose Register 26 (LPSR_GPR26) */

#define LPSR_GPR_GPR26_CM7_INIT_VTOR_SHIFT (0)      /* Bits 0-24:  Vector table offset register out of reset (CM7_INIT_VTOR) */
#define LPSR_GPR_GPR26_CM7_INIT_VTOR_MASK  (0xffffff << LPSR_GPR_GPR26_CM7_INIT_VTOR_SHIFT)

#define LPSR_GPR_GPR26_FIELD_0_SHIFT      (25)      /* Bits 25-27: General purpose bits (FIELD_0) */
#define LPSR_GPR_GPR26_FIELD_0_MASK       (0x07 << LPSR_GPR_GPR26_FIELD_0_SHIFT)
#define LPSR_GPR_GPR26_DWP_SHIFT          (28)      /* Bits 28-29: Domain write protection (DWP) */
#define LPSR_GPR_GPR26_DWP_MASK           (0x03 << LPSR_GPR_GPR26_DWP_SHIFT)
#  define LPSR_GPR_GPR26_DWP_ALLOWBOTH    (0x00 << LPSR_GPR_GPR26_DWP_SHIFT) /* Both cores are allowed */
#  define LPSR_GPR_GPR26_DWP_DENYCM7      (0x01 << LPSR_GPR_GPR26_DWP_SHIFT) /* CM7 is forbidden */
#  define LPSR_GPR_GPR26_DWP_DENYCM4      (0x02 << LPSR_GPR_GPR26_DWP_SHIFT) /* CM4 is forbidden */
#  define LPSR_GPR_GPR26_DWP_DENYBOTH     (0x03 << LPSR_GPR_GPR26_DWP_SHIFT) /* Both cores are forbidden */

#define LPSR_GPR_GPR26_DWP_LOCK_SHIFT     (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define LPSR_GPR_GPR26_DWP_LOCK_MASK      (0x03 << LPSR_GPR_GPR26_DWP_LOCK_SHIFT)
#  define LPSR_GPR_GPR26_DWP_LOCK_NOLCK   (0x00 << LPSR_GPR_GPR26_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define LPSR_GPR_GPR26_DWP_LOCK_LCKLO   (0x01 << LPSR_GPR_GPR26_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define LPSR_GPR_GPR26_DWP_LOCK_LCKHI   (0x02 << LPSR_GPR_GPR26_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define LPSR_GPR_GPR26_DWP_LOCK_LCKBOTH (0x03 << LPSR_GPR_GPR26_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* LPSR General Purpose Register 33 (LPSR_GPR33) */

#define LPSR_GPR_GPR33_M4_NMI_CLEAR       (1 << 0)  /* Bit 0:      Clear CM4 NMI holding register (M4_NMI_CLEAR) */
                                                    /* Bits 1-7:   Reserved */

#define LPSR_GPR_GPR33_USBPHY1_WAKEUP_IRQ_CLEAR (1 << 8) /* Bit 8: Clear USBPHY1 wakeup interrupt holding register (USBPHY1_WAKEUP_IRQ_CLEAR) */
#define LPSR_GPR_GPR33_USBPHY2_WAKEUP_IRQ_CLEAR (1 << 9) /* Bit 9: Clear USBPHY2 wakeup interrupt holding register (USBPHY2_WAKEUP_IRQ_CLEAR) */

                                                    /* Bits 10-15: Reserved */

                                                    /* Bits 16-27: Reserved */

#define LPSR_GPR_GPR33_DWP_SHIFT          (28)      /* Bits 28-29: Domain write protection (DWP) */
#define LPSR_GPR_GPR33_DWP_MASK           (0x03 << LPSR_GPR_GPR33_DWP_SHIFT)
#  define LPSR_GPR_GPR33_DWP_ALLOWBOTH    (0x00 << LPSR_GPR_GPR33_DWP_SHIFT) /* Both cores are allowed */
#  define LPSR_GPR_GPR33_DWP_DENYCM7      (0x01 << LPSR_GPR_GPR33_DWP_SHIFT) /* CM7 is forbidden */
#  define LPSR_GPR_GPR33_DWP_DENYCM4      (0x02 << LPSR_GPR_GPR33_DWP_SHIFT) /* CM4 is forbidden */
#  define LPSR_GPR_GPR33_DWP_DENYBOTH     (0x03 << LPSR_GPR_GPR33_DWP_SHIFT) /* Both cores are forbidden */

#define LPSR_GPR_GPR33_DWP_LOCK_SHIFT     (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define LPSR_GPR_GPR33_DWP_LOCK_MASK      (0x03 << LPSR_GPR_GPR33_DWP_LOCK_SHIFT)
#  define LPSR_GPR_GPR33_DWP_LOCK_NOLCK   (0x00 << LPSR_GPR_GPR33_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define LPSR_GPR_GPR33_DWP_LOCK_LCKLO   (0x01 << LPSR_GPR_GPR33_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define LPSR_GPR_GPR33_DWP_LOCK_LCKHI   (0x02 << LPSR_GPR_GPR33_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define LPSR_GPR_GPR33_DWP_LOCK_LCKBOTH (0x03 << LPSR_GPR_GPR33_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* LPSR General Purpose Register 34 (LPSR_GPR34) */

                                                    /* Bit 0:      Reserved */

#define LPSR_GPR_GPR34_GPIO_LPSR_HIGH_RANGE (1 << 1) /* Bit 1:     GPIO_LPSR IO bank supply voltage range selection (GPIO_LPSR_HIGH_RANGE) */
#define LPSR_GPR_GPR34_GPIO_LPSR_LOW_RANGE  (1 << 2) /* Bit 2:     GPIO_LPSR IO bank supply voltage range selection (GPIO_LPSR_LOW_RANGE) */

#define LPSR_GPR_GPR34_M7_NMI_MASK        (1 << 3)  /* Bit 3:      Mask CM7 NMI pin input (M7_NMI_MASK) */
#define LPSR_GPR_GPR34_M4_NMI_MASK        (1 << 4)  /* Bit 4:      Mask CM4 NMI pin input (M4_NMI_MASK) */
#define LPSR_GPR_GPR34_M4_GPC_SLEEP_SEL   (1 << 5)  /* Bit 5:      CM4 sleep request selection (M4_GPC_SLEEP_SEL) */

                                                    /* Bits 6-8:   Reserved */

                                                    /* Bits 9-10:  Reserved */

#define LPSR_GPR_GPR34_SEC_ERR_RESP       (1 << 3)  /* Bit 11:     Security error response enable (SEC_ERR_RESP) */
                                                    /* Bit 12-15:  Reserved */
                                                    /* Bit 16-27:  Reserved */
#define LPSR_GPR_GPR34_DWP_SHIFT          (28)      /* Bits 28-29: Domain write protection (DWP) */
#define LPSR_GPR_GPR34_DWP_MASK           (0x03 << LPSR_GPR_GPR34_DWP_SHIFT)
#  define LPSR_GPR_GPR34_DWP_ALLOWBOTH    (0x00 << LPSR_GPR_GPR34_DWP_SHIFT) /* Both cores are allowed */
#  define LPSR_GPR_GPR34_DWP_DENYCM7      (0x01 << LPSR_GPR_GPR34_DWP_SHIFT) /* CM7 is forbidden */
#  define LPSR_GPR_GPR34_DWP_DENYCM4      (0x02 << LPSR_GPR_GPR34_DWP_SHIFT) /* CM4 is forbidden */
#  define LPSR_GPR_GPR34_DWP_DENYBOTH     (0x03 << LPSR_GPR_GPR34_DWP_SHIFT) /* Both cores are forbidden */

#define LPSR_GPR_GPR34_DWP_LOCK_SHIFT     (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define LPSR_GPR_GPR34_DWP_LOCK_MASK      (0x03 << LPSR_GPR_GPR34_DWP_LOCK_SHIFT)
#  define LPSR_GPR_GPR34_DWP_LOCK_NOLCK   (0x00 << LPSR_GPR_GPR34_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define LPSR_GPR_GPR34_DWP_LOCK_LCKLO   (0x01 << LPSR_GPR_GPR34_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define LPSR_GPR_GPR34_DWP_LOCK_LCKHI   (0x02 << LPSR_GPR_GPR34_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define LPSR_GPR_GPR34_DWP_LOCK_LCKBOTH (0x03 << LPSR_GPR_GPR34_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* LPSR General Purpose Register 35 (LPSR_GPR35) */

#define LPSR_GPR_GPR35_ADC1_IPG_DOZE      (1 << 0)  /* Bit 0:      ADC1 doze mode (ADC1_IPG_DOZE) */
#define LPSR_GPR_GPR35_ADC1_STOP_REQ      (1 << 1)  /* Bit 1:      ADC1 stop request (ADC1_STOP_REQ) */
#define LPSR_GPR_GPR35_ADC1_IPG_STOP_MODE (1 << 2)  /* Bit 2:      ADC1 stop mode selection (ADC1_IPG_STOP_MODE) */
#define LPSR_GPR_GPR35_ADC2_IPG_DOZE      (1 << 3)  /* Bit 3:      ADC2 doze mode (ADC2_IPG_DOZE) */
#define LPSR_GPR_GPR35_ADC2_STOP_REQ      (1 << 4)  /* Bit 4:      ADC2 stop request (ADC2_STOP_REQ) */
#define LPSR_GPR_GPR35_ADC2_IPG_STOP_MODE (1 << 5)  /* Bit 5:      ADC2 stop mode selection (ADC2_IPG_STOP_MODE) */
#define LPSR_GPR_GPR35_CAAM_IPG_DOZE      (1 << 6)  /* Bit 6:      CAAM doze mode (CAAM_IPG_DOZE) */
#define LPSR_GPR_GPR35_CAAM_STOP_REQ      (1 << 7)  /* Bit 7:      CAAM stop request (CAAM_STOP_REQ) */
#define LPSR_GPR_GPR35_CAN1_IPG_DOZE      (1 << 8)  /* Bit 8:      CAN1 doze mode (CAN1_IPG_DOZE) */
#define LPSR_GPR_GPR35_CAN1_STOP_REQ      (1 << 9)  /* Bit 9:      CAN1 stop request (CAN1_STOP_REQ) */
#define LPSR_GPR_GPR35_CAN2_IPG_DOZE      (1 << 10) /* Bit 10:     CAN2 doze mode (CAN2_IPG_DOZE) */
#define LPSR_GPR_GPR35_CAN2_STOP_REQ      (1 << 11) /* Bit 11:     CAN2 stop request (CAN2_STOP_REQ) */
#define LPSR_GPR_GPR35_CAN3_IPG_DOZE      (1 << 12) /* Bit 12:     CAN3 doze mode (CAN3_IPG_DOZE) */
#define LPSR_GPR_GPR35_CAN3_STOP_REQ      (1 << 13) /* Bit 13:     CAN3 stop request (CAN3_STOP_REQ) */
                                                    /* Bit 14:     Reserved */
#define LPSR_GPR_GPR35_EDMA_STOP_REQ      (1 << 15) /* Bit 15:     EDMA stop request (EDMA_STOP_REQ) */
#define LPSR_GPR_GPR35_EDMA_LPSR_STOP_REQ (1 << 16) /* Bit 16:     EDMA_LPSR stop request (EDMA_LPSR_STOP_REQ) */
#define LPSR_GPR_GPR35_ENET_IPG_DOZE      (1 << 17) /* Bit 17:     ENET doze mode (ENET_IPG_DOZE) */
#define LPSR_GPR_GPR35_ENET_STOP_REQ      (1 << 18) /* Bit 18:     ENET stop request (ENET_STOP_REQ) */
#define LPSR_GPR_GPR35_ENET1G_IPG_DOZE    (1 << 19) /* Bit 19:     ENET1G doze mode (ENET1G_IPG_DOZE) */
#define LPSR_GPR_GPR35_ENET1G_STOP_REQ    (1 << 20) /* Bit 20:     ENET1G stop request (ENET1G_STOP_REQ) */
#define LPSR_GPR_GPR35_FLEXIO1_IPG_DOZE   (1 << 21) /* Bit 21:     FLEXIO1 doze mode (FLEXIO1_IPG_DOZE) */
#define LPSR_GPR_GPR35_FLEXIO2_IPG_DOZE   (1 << 22) /* Bit 22:     FLEXIO2 doze mode (FLEXIO2_IPG_DOZE) */
#define LPSR_GPR_GPR35_FLEXSPI1_IPG_DOZE  (1 << 23) /* Bit 23:     FLEXSPI1 doze mode (FLEXSPI1_IPG_DOZE) */
#define LPSR_GPR_GPR35_FLEXSPI1_STOP_REQ  (1 << 24) /* Bit 24:     FLEXSPI1 stop request (FLEXSPI1_STOP_REQ) */
#define LPSR_GPR_GPR35_FLEXSPI2_IPG_DOZE  (1 << 25) /* Bit 25:     FLEXSPI2 doze mode (FLEXSPI2_IPG_DOZE) */
#define LPSR_GPR_GPR35_FLEXSPI2_STOP_REQ  (1 << 26) /* Bit 26:     FLEXSPI2 stop request (FLEXSPI2_STOP_REQ) */
                                                    /* Bit 27:     Reserved */
#define LPSR_GPR_GPR35_DWP_SHIFT          (28)      /* Bits 28-29: Domain write protection (DWP) */
#define LPSR_GPR_GPR35_DWP_MASK           (0x03 << LPSR_GPR_GPR35_DWP_SHIFT)
#  define LPSR_GPR_GPR35_DWP_ALLOWBOTH    (0x00 << LPSR_GPR_GPR35_DWP_SHIFT) /* Both cores are allowed */
#  define LPSR_GPR_GPR35_DWP_DENYCM7      (0x01 << LPSR_GPR_GPR35_DWP_SHIFT) /* CM7 is forbidden */
#  define LPSR_GPR_GPR35_DWP_DENYCM4      (0x02 << LPSR_GPR_GPR35_DWP_SHIFT) /* CM4 is forbidden */
#  define LPSR_GPR_GPR35_DWP_DENYBOTH     (0x03 << LPSR_GPR_GPR35_DWP_SHIFT) /* Both cores are forbidden */

#define LPSR_GPR_GPR35_DWP_LOCK_SHIFT     (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define LPSR_GPR_GPR35_DWP_LOCK_MASK      (0x03 << LPSR_GPR_GPR35_DWP_LOCK_SHIFT)
#  define LPSR_GPR_GPR35_DWP_LOCK_NOLCK   (0x00 << LPSR_GPR_GPR35_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define LPSR_GPR_GPR35_DWP_LOCK_LCKLO   (0x01 << LPSR_GPR_GPR35_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define LPSR_GPR_GPR35_DWP_LOCK_LCKHI   (0x02 << LPSR_GPR_GPR35_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define LPSR_GPR_GPR35_DWP_LOCK_LCKBOTH (0x03 << LPSR_GPR_GPR35_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* LPSR General Purpose Register 36 (LPSR_GPR36) */

#define LPSR_GPR_GPR36_GPT1_IPG_DOZE      (1 << 0)  /* Bit 0:      GPT1 doze mode (GPT1_IPG_DOZE) */
#define LPSR_GPR_GPR36_GPT2_IPG_DOZE      (1 << 1)  /* Bit 1:      GPT2 doze mode (GPT2_IPG_DOZE) */
#define LPSR_GPR_GPR36_GPT3_IPG_DOZE      (1 << 2)  /* Bit 2:      GPT3 doze mode (GPT3_IPG_DOZE) */
#define LPSR_GPR_GPR36_GPT4_IPG_DOZE      (1 << 3)  /* Bit 3:      GPT4 doze mode (GPT4_IPG_DOZE) */
#define LPSR_GPR_GPR36_GPT5_IPG_DOZE      (1 << 4)  /* Bit 4:      GPT5 doze mode (GPT5_IPG_DOZE) */
#define LPSR_GPR_GPR36_GPT6_IPG_DOZE      (1 << 5)  /* Bit 5:      GPT6 doze mode (GPT6_IPG_DOZE) */
#define LPSR_GPR_GPR36_LPI2C1_IPG_DOZE    (1 << 6)  /* Bit 6:      LPI2C1 doze mode (LPI2C1_IPG_DOZE) */
#define LPSR_GPR_GPR36_LPI2C1_STOP_REQ    (1 << 7)  /* Bit 7:      LPI2C1 stop request (LPI2C1_STOP_REQ) */

#define LPSR_GPR_GPR36_LPI2C1_IPG_STOP_MODE (1 << 8)  /* Bit 8:    LPI2C1 stop mode selection (LPI2C1_IPG_STOP_MODE) */

#define LPSR_GPR_GPR36_LPI2C2_IPG_DOZE    (1 << 9)  /* Bit 9:      LPI2C2 doze mode (LPI2C2_IPG_DOZE) */
#define LPSR_GPR_GPR36_LPI2C2_STOP_REQ    (1 << 10) /* Bit 10:     LPI2C2 stop request (LPI2C2_STOP_REQ) */

#define LPSR_GPR_GPR36_LPI2C2_IPG_STOP_MODE (1 << 11) /* Bit 11:   LPI2C2 stop mode selection (LPI2C2_IPG_STOP_MODE) */

#define LPSR_GPR_GPR36_LPI2C3_IPG_DOZE    (1 << 12) /* Bit 12:     LPI2C3 doze mode (LPI2C3_IPG_DOZE) */
#define LPSR_GPR_GPR36_LPI2C3_STOP_REQ    (1 << 13) /* Bit 13:     LPI2C3 stop request (LPI2C3_STOP_REQ) */

#define LPSR_GPR_GPR36_LPI2C3_IPG_STOP_MODE (1 << 14) /* Bit 14:   LPI2C3 stop mode selection (LPI2C3_IPG_STOP_MODE) */

#define LPSR_GPR_GPR36_LPI2C4_IPG_DOZE    (1 << 15) /* Bit 15:     LPI2C4 doze mode (LPI2C4_IPG_DOZE) */
#define LPSR_GPR_GPR36_LPI2C4_STOP_REQ    (1 << 16) /* Bit 16:     LPI2C4 stop request (LPI2C4_STOP_REQ) */

#define LPSR_GPR_GPR36_LPI2C4_IPG_STOP_MODE (1 << 17) /* Bit 17:   LPI2C4 stop mode selection (LPI2C4_IPG_STOP_MODE) */

#define LPSR_GPR_GPR36_LPI2C5_IPG_DOZE    (1 << 18) /* Bit 18:     LPI2C5 doze mode (LPI2C5_IPG_DOZE) */
#define LPSR_GPR_GPR36_LPI2C5_STOP_REQ    (1 << 19) /* Bit 19:     LPI2C5 stop request (LPI2C5_STOP_REQ) */

#define LPSR_GPR_GPR36_LPI2C5_IPG_STOP_MODE (1 << 20) /* Bit 20:   LPI2C5 stop mode selection (LPI2C5_IPG_STOP_MODE) */

#define LPSR_GPR_GPR36_LPI2C6_IPG_DOZE    (1 << 21) /* Bit 21:     LPI2C6 doze mode (LPI2C6_IPG_DOZE) */
#define LPSR_GPR_GPR36_LPI2C6_STOP_REQ    (1 << 22) /* Bit 22:     LPI2C6 stop request (LPI2C6_STOP_REQ) */

#define LPSR_GPR_GPR36_LPI2C6_IPG_STOP_MODE (1 << 23) /* Bit 23:   LPI2C6 stop mode selection (LPI2C6_IPG_STOP_MODE) */

#define LPSR_GPR_GPR36_LPSPI1_IPG_DOZE    (1 << 24) /* Bit 24:     LPSPI1 doze mode (LPSPI1_IPG_DOZE) */
#define LPSR_GPR_GPR36_LPSPI1_STOP_REQ    (1 << 25) /* Bit 25:     LPSPI1 stop request (LPSPI1_STOP_REQ) */

#define LPSR_GPR_GPR36_LPSPI1_IPG_STOP_MODE (1 << 26) /* Bit 26:   LPSPI1 stop mode selection (LPSPI1_IPG_STOP_MODE) */

                                                    /* Bit 27:     Reserved */
#define LPSR_GPR_GPR36_DWP_SHIFT          (28)      /* Bits 28-29: Domain write protection (DWP) */
#define LPSR_GPR_GPR36_DWP_MASK           (0x03 << LPSR_GPR_GPR36_DWP_SHIFT)
#  define LPSR_GPR_GPR36_DWP_ALLOWBOTH    (0x00 << LPSR_GPR_GPR36_DWP_SHIFT) /* Both cores are allowed */
#  define LPSR_GPR_GPR36_DWP_DENYCM7      (0x01 << LPSR_GPR_GPR36_DWP_SHIFT) /* CM7 is forbidden */
#  define LPSR_GPR_GPR36_DWP_DENYCM4      (0x02 << LPSR_GPR_GPR36_DWP_SHIFT) /* CM4 is forbidden */
#  define LPSR_GPR_GPR36_DWP_DENYBOTH     (0x03 << LPSR_GPR_GPR36_DWP_SHIFT) /* Both cores are forbidden */

#define LPSR_GPR_GPR36_DWP_LOCK_SHIFT     (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define LPSR_GPR_GPR36_DWP_LOCK_MASK      (0x03 << LPSR_GPR_GPR36_DWP_LOCK_SHIFT)
#  define LPSR_GPR_GPR36_DWP_LOCK_NOLCK   (0x00 << LPSR_GPR_GPR36_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define LPSR_GPR_GPR36_DWP_LOCK_LCKLO   (0x01 << LPSR_GPR_GPR36_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define LPSR_GPR_GPR36_DWP_LOCK_LCKHI   (0x02 << LPSR_GPR_GPR36_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define LPSR_GPR_GPR36_DWP_LOCK_LCKBOTH (0x03 << LPSR_GPR_GPR36_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* LPSR General Purpose Register 37 (LPSR_GPR37) */

#define LPSR_GPR_GPR37_LPSPI2_IPG_DOZE    (1 << 0)  /* Bit 0:      LPSPI2 doze mode (LPSPI2_IPG_DOZE) */
#define LPSR_GPR_GPR37_LPSPI2_STOP_REQ    (1 << 1)  /* Bit 1:      LPSPI2 stop request (LPSPI2_STOP_REQ) */

#define LPSR_GPR_GPR37_LPSPI2_IPG_STOP_MODE (1 << 2)  /* Bit 2:    LPSPI2 stop mode selection (LPSPI2_IPG_STOP_MODE) */

#define LPSR_GPR_GPR37_LPSPI3_IPG_DOZE    (1 << 3)  /* Bit 3:      LPSPI3 doze mode (LPSPI3_IPG_DOZE) */
#define LPSR_GPR_GPR37_LPSPI3_STOP_REQ    (1 << 4)  /* Bit 4:      LPSPI3 stop request (LPSPI3_STOP_REQ) */

#define LPSR_GPR_GPR37_LPSPI3_IPG_STOP_MODE (1 << 5)  /* Bit 5:    LPSPI3 stop mode selection (LPSPI3_IPG_STOP_MODE) */

#define LPSR_GPR_GPR37_LPSPI4_IPG_DOZE    (1 << 6)  /* Bit 6:      LPSPI4 doze mode (LPSPI4_IPG_DOZE) */
#define LPSR_GPR_GPR37_LPSPI4_STOP_REQ    (1 << 7)  /* Bit 7:      LPSPI4 stop request (LPSPI4_STOP_REQ) */

#define LPSR_GPR_GPR37_LPSPI4_IPG_STOP_MODE (1 << 8)  /* Bit 8:    LPSPI4 stop mode selection (LPSPI4_IPG_STOP_MODE) */

#define LPSR_GPR_GPR37_LPSPI5_IPG_DOZE    (1 << 9)  /* Bit 9:      LPSPI5 doze mode (LPSPI5_IPG_DOZE) */
#define LPSR_GPR_GPR37_LPSPI5_STOP_REQ    (1 << 10) /* Bit 10:     LPSPI5 stop request (LPSPI5_STOP_REQ) */

#define LPSR_GPR_GPR37_LPSPI5_IPG_STOP_MODE (1 << 11) /* Bit 11:   LPSPI5 stop mode selection (LPSPI5_IPG_STOP_MODE) */

#define LPSR_GPR_GPR37_LPSPI6_IPG_DOZE    (1 << 12) /* Bit 12:     LPSPI6 doze mode (LPSPI6_IPG_DOZE) */
#define LPSR_GPR_GPR37_LPSPI6_STOP_REQ    (1 << 13) /* Bit 13:     LPSPI6 stop request (LPSPI6_STOP_REQ) */

#define LPSR_GPR_GPR37_LPSPI6_IPG_STOP_MODE (1 << 14) /* Bit 14:   LPSPI6 stop mode selection (LPSPI6_IPG_STOP_MODE) */

#define LPSR_GPR_GPR37_LPUART1_IPG_DOZE   (1 << 15) /* Bit 15:     LPUART1 doze mode (LPUART1_IPG_DOZE) */
#define LPSR_GPR_GPR37_LPUART1_STOP_REQ   (1 << 16) /* Bit 16:     LPUART1 stop request (LPUART1_STOP_REQ) */

#define LPSR_GPR_GPR37_LPUART1_IPG_STOP_MODE (1 << 17) /* Bit 17:  LPUART1 stop mode selection (LPUART1_IPG_STOP_MODE) */

#define LPSR_GPR_GPR37_LPUART2_IPG_DOZE   (1 << 18) /* Bit 18:     LPUART2 doze mode (LPUART2_IPG_DOZE) */
#define LPSR_GPR_GPR37_LPUART2_STOP_REQ   (1 << 19) /* Bit 19:     LPUART2 stop request (LPUART2_STOP_REQ) */

#define LPSR_GPR_GPR37_LPUART2_IPG_STOP_MODE (1 << 20) /* Bit 20:  LPUART2 stop mode selection (LPUART2_IPG_STOP_MODE) */

#define LPSR_GPR_GPR37_LPUART3_IPG_DOZE   (1 << 21) /* Bit 21:     LPUART3 doze mode (LPUART3_IPG_DOZE) */
#define LPSR_GPR_GPR37_LPUART3_STOP_REQ   (1 << 22) /* Bit 22:     LPUART3 stop request (LPUART3_STOP_REQ) */

#define LPSR_GPR_GPR37_LPUART3_IPG_STOP_MODE (1 << 23) /* Bit 23:  LPUART3 stop mode selection (LPUART3_IPG_STOP_MODE) */

#define LPSR_GPR_GPR37_LPUART4_IPG_DOZE   (1 << 24) /* Bit 24:     LPUART4 doze mode (LPUART4_IPG_DOZE) */
#define LPSR_GPR_GPR37_LPUART4_STOP_REQ   (1 << 25) /* Bit 25:     LPUART4 stop request (LPUART4_STOP_REQ) */

#define LPSR_GPR_GPR37_LPUART4_IPG_STOP_MODE (1 << 26) /* Bit 26:  LPUART4 stop mode selection (LPUART4_IPG_STOP_MODE) */

                                                    /* Bit 27:     Reserved */
#define LPSR_GPR_GPR37_DWP_SHIFT          (28)      /* Bits 28-29: Domain write protection (DWP) */
#define LPSR_GPR_GPR37_DWP_MASK           (0x03 << LPSR_GPR_GPR37_DWP_SHIFT)
#  define LPSR_GPR_GPR37_DWP_ALLOWBOTH    (0x00 << LPSR_GPR_GPR37_DWP_SHIFT) /* Both cores are allowed */
#  define LPSR_GPR_GPR37_DWP_DENYCM7      (0x01 << LPSR_GPR_GPR37_DWP_SHIFT) /* CM7 is forbidden */
#  define LPSR_GPR_GPR37_DWP_DENYCM4      (0x02 << LPSR_GPR_GPR37_DWP_SHIFT) /* CM4 is forbidden */
#  define LPSR_GPR_GPR37_DWP_DENYBOTH     (0x03 << LPSR_GPR_GPR37_DWP_SHIFT) /* Both cores are forbidden */

#define LPSR_GPR_GPR37_DWP_LOCK_SHIFT     (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define LPSR_GPR_GPR37_DWP_LOCK_MASK      (0x03 << LPSR_GPR_GPR37_DWP_LOCK_SHIFT)
#  define LPSR_GPR_GPR37_DWP_LOCK_NOLCK   (0x00 << LPSR_GPR_GPR37_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define LPSR_GPR_GPR37_DWP_LOCK_LCKLO   (0x01 << LPSR_GPR_GPR37_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define LPSR_GPR_GPR37_DWP_LOCK_LCKHI   (0x02 << LPSR_GPR_GPR37_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define LPSR_GPR_GPR37_DWP_LOCK_LCKBOTH (0x03 << LPSR_GPR_GPR37_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* LPSR General Purpose Register 38 (LPSR_GPR38) */

#define LPSR_GPR_GPR38_LPUART5_IPG_DOZE   (1 << 0)  /* Bit 0:      LPUART5 doze mode (LPUART5_IPG_DOZE) */
#define LPSR_GPR_GPR38_LPUART5_STOP_REQ   (1 << 1)  /* Bit 1:      LPUART5 stop request (LPUART5_STOP_REQ) */

#define LPSR_GPR_GPR38_LPUART5_IPG_STOP_MODE  (1 << 2)  /* Bit 2:  LPUART5 stop mode selection (LPUART5_IPG_STOP_MODE) */

#define LPSR_GPR_GPR38_LPUART6_IPG_DOZE   (1 << 3)  /* Bit 3:      LPUART6 doze mode (LPUART6_IPG_DOZE) */
#define LPSR_GPR_GPR38_LPUART6_STOP_REQ   (1 << 4)  /* Bit 4:      LPUART6 stop request (LPUART6_STOP_REQ) */

#define LPSR_GPR_GPR38_LPUART6_IPG_STOP_MODE  (1 << 5)  /* Bit 5:  LPUART6 stop mode selection (LPUART6_IPG_STOP_MODE) */

#define LPSR_GPR_GPR38_LPUART7_IPG_DOZE   (1 << 6)  /* Bit 6:      LPUART7 doze mode (LPUART7_IPG_DOZE) */
#define LPSR_GPR_GPR38_LPUART7_STOP_REQ   (1 << 7)  /* Bit 7:      LPUART7 stop request (LPUART7_STOP_REQ) */

#define LPSR_GPR_GPR38_LPUART7_IPG_STOP_MODE  (1 << 8)  /* Bit 8:  LPUART7 stop mode selection (LPUART7_IPG_STOP_MODE) */

#define LPSR_GPR_GPR38_LPUART8_IPG_DOZE   (1 << 9)  /* Bit 9:      LPUART8 doze mode (LPUART8_IPG_DOZE) */
#define LPSR_GPR_GPR38_LPUART8_STOP_REQ   (1 << 10) /* Bit 10:     LPUART8 stop request (LPUART8_STOP_REQ) */

#define LPSR_GPR_GPR38_LPUART8_IPG_STOP_MODE  (1 << 11) /* Bit 11: LPUART8 stop mode selection (LPUART8_IPG_STOP_MODE) */

#define LPSR_GPR_GPR38_LPUART9_IPG_DOZE   (1 << 12) /* Bit 12:     LPUART9 doze mode (LPUART9_IPG_DOZE) */
#define LPSR_GPR_GPR38_LPUART9_STOP_REQ   (1 << 13) /* Bit 13:     LPUART9 stop request (LPUART9_STOP_REQ) */

#define LPSR_GPR_GPR38_LPUART9_IPG_STOP_MODE  (1 << 14) /* Bit 14: LPUART9 stop mode selection (LPUART9_IPG_STOP_MODE) */

#define LPSR_GPR_GPR38_LPUART10_IPG_DOZE  (1 << 15) /* Bit 15:     LPUART10 doze mode (LPUART10_IPG_DOZE) */
#define LPSR_GPR_GPR38_LPUART10_STOP_REQ  (1 << 16) /* Bit 16:     LPUART10 stop request (LPUART10_STOP_REQ) */

#define LPSR_GPR_GPR38_LPUART10_IPG_STOP_MODE (1 << 17) /* Bit 17: LPUART10 stop mode selection (LPUART10_IPG_STOP_MODE) */

#define LPSR_GPR_GPR38_LPUART11_IPG_DOZE  (1 << 18) /* Bit 18:     LPUART11 doze mode (LPUART11_IPG_DOZE) */
#define LPSR_GPR_GPR38_LPUART11_STOP_REQ  (1 << 19) /* Bit 19:     LPUART11 stop request (LPUART11_STOP_REQ) */

#define LPSR_GPR_GPR38_LPUART11_IPG_STOP_MODE (1 << 20) /* Bit 20: LPUART11 stop mode selection (LPUART11_IPG_STOP_MODE) */

#define LPSR_GPR_GPR38_LPUART12_IPG_DOZE  (1 << 21) /* Bit 21:     LPUART12 doze mode (LPUART12_IPG_DOZE) */
#define LPSR_GPR_GPR38_LPUART12_STOP_REQ  (1 << 22) /* Bit 22:     LPUART12 stop request (LPUART12_STOP_REQ) */

#define LPSR_GPR_GPR38_LPUART12_IPG_STOP_MODE (1 << 23) /* Bit 23: LPUART12 stop mode selection (LPUART12_IPG_STOP_MODE) */

#define LPSR_GPR_GPR38_MIC_IPG_DOZE       (1 << 24) /* Bit 24:     MIC doze mode (MIC_IPG_DOZE) */
#define LPSR_GPR_GPR38_MIC_STOP_REQ       (1 << 25) /* Bit 25:     MIC stop request (MIC_STOP_REQ) */
#define LPSR_GPR_GPR38_MIC_IPG_STOP_MODE  (1 << 26) /* Bit 26:     MIC stop mode selection (MIC_IPG_STOP_MODE) */
                                                    /* Bit 27:     Reserved */
#define LPSR_GPR_GPR38_DWP_SHIFT          (28)      /* Bits 28-29: Domain write protection (DWP) */
#define LPSR_GPR_GPR38_DWP_MASK           (0x03 << LPSR_GPR_GPR38_DWP_SHIFT)
#  define LPSR_GPR_GPR38_DWP_ALLOWBOTH    (0x00 << LPSR_GPR_GPR38_DWP_SHIFT) /* Both cores are allowed */
#  define LPSR_GPR_GPR38_DWP_DENYCM7      (0x01 << LPSR_GPR_GPR38_DWP_SHIFT) /* CM7 is forbidden */
#  define LPSR_GPR_GPR38_DWP_DENYCM4      (0x02 << LPSR_GPR_GPR38_DWP_SHIFT) /* CM4 is forbidden */
#  define LPSR_GPR_GPR38_DWP_DENYBOTH     (0x03 << LPSR_GPR_GPR38_DWP_SHIFT) /* Both cores are forbidden */

#define LPSR_GPR_GPR38_DWP_LOCK_SHIFT     (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define LPSR_GPR_GPR38_DWP_LOCK_MASK      (0x03 << LPSR_GPR_GPR38_DWP_LOCK_SHIFT)
#  define LPSR_GPR_GPR38_DWP_LOCK_NOLCK   (0x00 << LPSR_GPR_GPR38_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define LPSR_GPR_GPR38_DWP_LOCK_LCKLO   (0x01 << LPSR_GPR_GPR38_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define LPSR_GPR_GPR38_DWP_LOCK_LCKHI   (0x02 << LPSR_GPR_GPR38_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define LPSR_GPR_GPR38_DWP_LOCK_LCKBOTH (0x03 << LPSR_GPR_GPR38_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* LPSR General Purpose Register 39 (LPSR_GPR39) */

                                                    /* Bit 0:      Reserved */
#define LPSR_GPR_GPR39_PIT1_STOP_REQ      (1 << 1)  /* Bit 1:      PIT1 stop request (PIT1_STOP_REQ) */
#define LPSR_GPR_GPR39_PIT2_STOP_REQ      (1 << 2)  /* Bit 2:      PIT2 stop request (PIT2_STOP_REQ) */
#define LPSR_GPR_GPR39_SEMC_STOP_REQ      (1 << 3)  /* Bit 3:      SEMC stop request (SEMC_STOP_REQ) */
#define LPSR_GPR_GPR39_SIM1_IPG_DOZE      (1 << 4)  /* Bit 4:      SIM1 doze mode (SIM1_IPG_DOZE) */
#define LPSR_GPR_GPR39_SIM2_IPG_DOZE      (1 << 5)  /* Bit 5:      SIM2 doze mode (SIM2_IPG_DOZE) */
#define LPSR_GPR_GPR39_SNVS_HP_IPG_DOZE   (1 << 6)  /* Bit 6:      SNVS_HP doze mode (SNVS_HP_IPG_DOZE) */
#define LPSR_GPR_GPR39_SNVS_HP_STOP_REQ   (1 << 7)  /* Bit 7:      SNVS_HP stop request (SNVS_HP_STOP_REQ) */
#define LPSR_GPR_GPR39_WDOG1_IPG_DOZE     (1 << 8)  /* Bit 8:      WDOG1 doze mode (WDOG1_IPG_DOZE) */
#define LPSR_GPR_GPR39_WDOG2_IPG_DOZE     (1 << 9)  /* Bit 9:      WDOG2 doze mode (WDOG2_IPG_DOZE) */
#define LPSR_GPR_GPR39_SAI1_STOP_REQ      (1 << 10) /* Bit 10:     SAI1 stop request (SAI1_STOP_REQ) */
#define LPSR_GPR_GPR39_SAI2_STOP_REQ      (1 << 11) /* Bit 11:     SAI2 stop request (SAI2_STOP_REQ) */
#define LPSR_GPR_GPR39_SAI3_STOP_REQ      (1 << 12) /* Bit 12:     SAI3 stop request (SAI3_STOP_REQ) */
#define LPSR_GPR_GPR39_SAI4_STOP_REQ      (1 << 13) /* Bit 13:     SAI4 stop request (SAI4_STOP_REQ) */

#define LPSR_GPR_GPR39_FLEXIO1_STOP_REQ_BUS (1 << 14) /* Bit 14:   FLEXIO1 bus clock domain stop request (FLEXIO1_STOP_REQ_BUS) */
#define LPSR_GPR_GPR39_FLEXIO1_STOP_REQ_PER (1 << 15) /* Bit 15:   FLEXIO1 peripheral clock domain stop request (FLEXIO1_STOP_REQ_PER) */
#define LPSR_GPR_GPR39_FLEXIO2_STOP_REQ_BUS (1 << 16) /* Bit 16:   FLEXIO2 bus clock domain stop request (FLEXIO2_STOP_REQ_BUS) */
#define LPSR_GPR_GPR39_FLEXIO2_STOP_REQ_PER (1 << 17) /* Bit 17:   FLEXIO2 peripheral clock domain stop request (FLEXIO2_STOP_REQ_PER) */

                                                    /* Bits 18-27: Reserved */
#define LPSR_GPR_GPR39_DWP_SHIFT          (28)      /* Bits 28-29: Domain write protection (DWP) */
#define LPSR_GPR_GPR39_DWP_MASK           (0x03 << LPSR_GPR_GPR39_DWP_SHIFT)
#  define LPSR_GPR_GPR39_DWP_ALLOWBOTH    (0x00 << LPSR_GPR_GPR39_DWP_SHIFT) /* Both cores are allowed */
#  define LPSR_GPR_GPR39_DWP_DENYCM7      (0x01 << LPSR_GPR_GPR39_DWP_SHIFT) /* CM7 is forbidden */
#  define LPSR_GPR_GPR39_DWP_DENYCM4      (0x02 << LPSR_GPR_GPR39_DWP_SHIFT) /* CM4 is forbidden */
#  define LPSR_GPR_GPR39_DWP_DENYBOTH     (0x03 << LPSR_GPR_GPR39_DWP_SHIFT) /* Both cores are forbidden */

#define LPSR_GPR_GPR39_DWP_LOCK_SHIFT     (30)      /* Bits 30-31: Domain write protection lock (DWP_LOCK) */
#define LPSR_GPR_GPR39_DWP_LOCK_MASK      (0x03 << LPSR_GPR_GPR39_DWP_LOCK_SHIFT)
#  define LPSR_GPR_GPR39_DWP_LOCK_NOLCK   (0x00 << LPSR_GPR_GPR39_DWP_LOCK_SHIFT) /* Neither of DWP bits is locked */
#  define LPSR_GPR_GPR39_DWP_LOCK_LCKLO   (0x01 << LPSR_GPR_GPR39_DWP_LOCK_SHIFT) /* Lower DWP bit is locked */
#  define LPSR_GPR_GPR39_DWP_LOCK_LCKHI   (0x02 << LPSR_GPR_GPR39_DWP_LOCK_SHIFT) /* Higher DWP bit is locked */
#  define LPSR_GPR_GPR39_DWP_LOCK_LCKBOTH (0x03 << LPSR_GPR_GPR39_DWP_LOCK_SHIFT) /* Both DWP bits are locked */

/* LPSR General Purpose Register 40 (LPSR_GPR40) */

#define LPSR_GPR_GPR40_ADC1_STOP_ACK      (1 << 0)  /* Bit 0:      ADC1 stop acknowledge (ADC1_STOP_ACK) */
#define LPSR_GPR_GPR40_ADC2_STOP_ACK      (1 << 1)  /* Bit 1:      ADC2 stop acknowledge (ADC2_STOP_ACK) */
#define LPSR_GPR_GPR40_CAAM_STOP_ACK      (1 << 2)  /* Bit 2:      CAAM stop acknowledge (CAAM_STOP_ACK) */
#define LPSR_GPR_GPR40_CAN1_STOP_ACK      (1 << 3)  /* Bit 3:      CAN1 stop acknowledge (CAN1_STOP_ACK) */
#define LPSR_GPR_GPR40_CAN2_STOP_ACK      (1 << 4)  /* Bit 4:      CAN2 stop acknowledge (CAN2_STOP_ACK) */
#define LPSR_GPR_GPR40_CAN3_STOP_ACK      (1 << 5)  /* Bit 5:      CAN3 stop acknowledge (CAN3_STOP_ACK) */
#define LPSR_GPR_GPR40_EDMA_STOP_ACK      (1 << 6)  /* Bit 6:      EDMA stop acknowledge (EDMA_STOP_ACK) */
#define LPSR_GPR_GPR40_EDMA_LPSR_STOP_ACK (1 << 7)  /* Bit 7:      EDMA_LPSR stop acknowledge (EDMA_LPSR_STOP_ACK) */
#define LPSR_GPR_GPR40_ENET_STOP_ACK      (1 << 8)  /* Bit 8:      ENET stop acknowledge (ENET_STOP_ACK) */
#define LPSR_GPR_GPR40_ENET1G_STOP_ACK    (1 << 9)  /* Bit 9:      ENET1G stop acknowledge (ENET1G_STOP_ACK) */
#define LPSR_GPR_GPR40_FLEXSPI1_STOP_ACK  (1 << 10) /* Bit 10:     FLEXSPI1 stop acknowledge (FLEXSPI1_STOP_ACK) */
#define LPSR_GPR_GPR40_FLEXSPI2_STOP_ACK  (1 << 11) /* Bit 11:     FLEXSPI2 stop acknowledge (FLEXSPI2_STOP_ACK) */
#define LPSR_GPR_GPR40_LPI2C1_STOP_ACK    (1 << 12) /* Bit 12:     LPI2C1 stop acknowledge (LPI2C1_STOP_ACK) */
#define LPSR_GPR_GPR40_LPI2C2_STOP_ACK    (1 << 13) /* Bit 13:     LPI2C2 stop acknowledge (LPI2C2_STOP_ACK) */
#define LPSR_GPR_GPR40_LPI2C3_STOP_ACK    (1 << 14) /* Bit 14:     LPI2C3 stop acknowledge (LPI2C3_STOP_ACK) */
#define LPSR_GPR_GPR40_LPI2C4_STOP_ACK    (1 << 15) /* Bit 15:     LPI2C4 stop acknowledge (LPI2C4_STOP_ACK) */
#define LPSR_GPR_GPR40_LPI2C5_STOP_ACK    (1 << 16) /* Bit 16:     LPI2C5 stop acknowledge (LPI2C5_STOP_ACK) */
#define LPSR_GPR_GPR40_LPI2C6_STOP_ACK    (1 << 17) /* Bit 17:     LPI2C6 stop acknowledge (LPI2C6_STOP_ACK) */
#define LPSR_GPR_GPR40_LPSPI1_STOP_ACK    (1 << 18) /* Bit 18:     LPSPI1 stop acknowledge (LPSPI1_STOP_ACK) */
#define LPSR_GPR_GPR40_LPSPI2_STOP_ACK    (1 << 19) /* Bit 19:     LPSPI2 stop acknowledge (LPSPI2_STOP_ACK) */
#define LPSR_GPR_GPR40_LPSPI3_STOP_ACK    (1 << 20) /* Bit 20:     LPSPI3 stop acknowledge (LPSPI3_STOP_ACK) */
#define LPSR_GPR_GPR40_LPSPI4_STOP_ACK    (1 << 21) /* Bit 21:     LPSPI4 stop acknowledge (LPSPI4_STOP_ACK) */
#define LPSR_GPR_GPR40_LPSPI5_STOP_ACK    (1 << 22) /* Bit 22:     LPSPI5 stop acknowledge (LPSPI5_STOP_ACK) */
#define LPSR_GPR_GPR40_LPSPI6_STOP_ACK    (1 << 23) /* Bit 23:     LPSPI6 stop acknowledge (LPSPI6_STOP_ACK) */
#define LPSR_GPR_GPR40_LPUART1_STOP_ACK   (1 << 24) /* Bit 24:     LPUART1 stop acknowledge (LPUART1_STOP_ACK) */
#define LPSR_GPR_GPR40_LPUART2_STOP_ACK   (1 << 25) /* Bit 25:     LPUART2 stop acknowledge (LPUART2_STOP_ACK) */
#define LPSR_GPR_GPR40_LPUART3_STOP_ACK   (1 << 26) /* Bit 26:     LPUART3 stop acknowledge (LPUART3_STOP_ACK) */
#define LPSR_GPR_GPR40_LPUART4_STOP_ACK   (1 << 27) /* Bit 27:     LPUART4 stop acknowledge (LPUART4_STOP_ACK) */
#define LPSR_GPR_GPR40_LPUART5_STOP_ACK   (1 << 28) /* Bit 28:     LPUART5 stop acknowledge (LPUART5_STOP_ACK) */
#define LPSR_GPR_GPR40_LPUART6_STOP_ACK   (1 << 29) /* Bit 29:     LPUART6 stop acknowledge (LPUART6_STOP_ACK) */
#define LPSR_GPR_GPR40_LPUART7_STOP_ACK   (1 << 30) /* Bit 30:     LPUART7 stop acknowledge (LPUART7_STOP_ACK) */
#define LPSR_GPR_GPR40_LPUART8_STOP_ACK   (1 << 31) /* Bit 31:     LPUART8 stop acknowledge (LPUART8_STOP_ACK) */

/* LPSR General Purpose Register 41 (LPSR_GPR41) */

#define LPSR_GPR_GPR41_LPUART9_STOP_ACK   (1 << 0)  /* Bit 0:      LPUART9 stop acknowledge (LPUART9_STOP_ACK) */
#define LPSR_GPR_GPR41_LPUART10_STOP_ACK  (1 << 1)  /* Bit 1:      LPUART10 stop acknowledge (LPUART10_STOP_ACK) */
#define LPSR_GPR_GPR41_LPUART11_STOP_ACK  (1 << 2)  /* Bit 2:      LPUART11 stop acknowledge (LPUART11_STOP_ACK) */
#define LPSR_GPR_GPR41_LPUART12_STOP_ACK  (1 << 3)  /* Bit 3:      LPUART12 stop acknowledge (LPUART12_STOP_ACK) */
#define LPSR_GPR_GPR41_MIC_STOP_ACK       (1 << 4)  /* Bit 4:      MIC stop acknowledge (MIC_STOP_ACK) */
#define LPSR_GPR_GPR41_PIT1_STOP_ACK      (1 << 5)  /* Bit 5:      PIT1 stop acknowledge (PIT1_STOP_ACK) */
#define LPSR_GPR_GPR41_PIT2_STOP_ACK      (1 << 6)  /* Bit 6:      PIT2 stop acknowledge (PIT2_STOP_ACK) */
#define LPSR_GPR_GPR41_SEMC_STOP_ACK      (1 << 7)  /* Bit 7:      SEMC stop acknowledge (SEMC_STOP_ACK) */
#define LPSR_GPR_GPR41_SNVS_HP_STOP_ACK   (1 << 8)  /* Bit 8:      SNVS_HP stop acknowledge (SNVS_HP_STOP_ACK) */
#define LPSR_GPR_GPR41_SAI1_STOP_ACK      (1 << 9)  /* Bit 9:      SAI1 stop acknowledge (SAI1_STOP_ACK) */
#define LPSR_GPR_GPR41_SAI2_STOP_ACK      (1 << 10) /* Bit 10:     SAI2 stop acknowledge (SAI2_STOP_ACK) */
#define LPSR_GPR_GPR41_SAI3_STOP_ACK      (1 << 11) /* Bit 11:     SAI3 stop acknowledge (SAI3_STOP_ACK) */
#define LPSR_GPR_GPR41_SAI4_STOP_ACK      (1 << 12) /* Bit 12:     SAI4 stop acknowledge (SAI4_STOP_ACK) */

#define LPSR_GPR_GPR41_FLEXIO1_STOP_ACK_BUS (1 << 13) /* Bit 13:   FLEXIO1 stop acknowledge of bus clock domain (FLEXIO1_STOP_ACK_BUS) */
#define LPSR_GPR_GPR41_FLEXIO1_STOP_ACK_PER (1 << 14) /* Bit 14:   FLEXIO1 stop acknowledge of peripheral clock domain (FLEXIO1_STOP_ACK_PER) */
#define LPSR_GPR_GPR41_FLEXIO2_STOP_ACK_BUS (1 << 15) /* Bit 15:   FLEXIO2 stop acknowledge of bus clock domain (FLEXIO2_STOP_ACK_BUS) */
#define LPSR_GPR_GPR41_FLEXIO2_STOP_ACK_PER (1 << 16) /* Bit 16:   FLEXIO2 stop acknowledge of peripheral clock domain (FLEXIO2_STOP_ACK_PER) */

                                                    /* Bits 17-31: Reserved */

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_RT117X_IMXRT117X_IOMUXC_H */
