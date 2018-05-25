/************************************************************************************
 * arch/arm/src/imxrt/imxrt_iomuxc.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_IMXRT_CHIP_IMXRT_IOMUXC_H
#define __ARCH_ARM_SRC_IMXRT_CHIP_IMXRT_IOMUXC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip/imxrt_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define IMXRT_IOMUXC_GPR_GPR0_OFFSET                    0x0000 /* GPR0 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR1_OFFSET                    0x0004 /* GPR1 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR2_OFFSET                    0x0008 /* GPR2 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR3_OFFSET                    0x000c /* GPR3 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR4_OFFSET                    0x0010 /* GPR4 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR5_OFFSET                    0x0014 /* GPR5 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR6_OFFSET                    0x0018 /* GPR6 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR7_OFFSET                    0x001c /* GPR7 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR8_OFFSET                    0x0020 /* GPR8 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR9_OFFSET                    0x0024 /* GPR9 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR10_OFFSET                   0x0028 /* GPR10 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR11_OFFSET                   0x002c /* GPR11 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR12_OFFSET                   0x0030 /* GPR12 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR13_OFFSET                   0x0034 /* GPR13 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR14_OFFSET                   0x0038 /* GPR14 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR15_OFFSET                   0x003c /* GPR15 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR16_OFFSET                   0x0040 /* GPR16 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR17_OFFSET                   0x0044 /* GPR17 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR18_OFFSET                   0x0048 /* GPR18 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR19_OFFSET                   0x004c /* GPR19 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR20_OFFSET                   0x0050 /* GPR20 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR21_OFFSET                   0x0054 /* GPR21 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR22_OFFSET                   0x0058 /* GPR22 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR23_OFFSET                   0x005c /* GPR23 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR24_OFFSET                   0x0060 /* GPR24 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR25_OFFSET                   0x0064 /* GPR25 General Purpose Register*/

#define IMXRT_IOMUXC_SNVS_SW_MUX_CTL_PAD_WAKEUP_OFFSET         0x0000 /* SW_MUX_CTL_PAD_WAKEUP SW MUX Control Register */
#define IMXRT_IOMUXC_SNVS_SW_MUX_CTL_PAD_PMIC_ON_REQ_OFFSET    0x0004 /* SW_MUX_CTL_PAD_PMIC_ON_REQ SW MUX Control Register */
#define IMXRT_IOMUXC_SNVS_SW_MUX_CTL_PAD_PMIC_STBY_REQ_OFFSET  0x0008 /* SW_MUX_CTL_PAD_PMIC_STBY_REQ SW MUX Control Register */
#define IMXRT_IOMUXC_SNVS_SW_PAD_CTL_PAD_TEST_MODE_OFFSET      0x000c /* SW_PAD_CTL_PAD_TEST_MODE SW PAD Control Register */
#define IMXRT_IOMUXC_SNVS_SW_PAD_CTL_PAD_POR_B_OFFSET          0x0010 /* SW_PAD_CTL_PAD_POR_B SW PAD Control Register */
#define IMXRT_IOMUXC_SNVS_SW_PAD_CTL_PAD_ONOFF_OFFSET          0x0014 /* SW_PAD_CTL_PAD_ONOFF SW PAD Control Register */
#define IMXRT_IOMUXC_SNVS_SW_PAD_CTL_PAD_WAKEUP_OFFSET         0x0018 /* SW_PAD_CTL_PAD_WAKEUP SW PAD Control Register */
#define IMXRT_IOMUXC_SNVS_SW_PAD_CTL_PAD_PMIC_ON_REQ_OFFSET    0x001c /* SW_PAD_CTL_PAD_PMIC_ON_REQ SW PAD Control Register */
#define IMXRT_IOMUXC_SNVS_SW_PAD_CTL_PAD_PMIC_STBY_REQ_OFFSET  0x0020 /* SW_PAD_CTL_PAD_PMIC_STBY_REQ SW PAD Control Register */

#define IMXRT_IOMUXC_SNVS_GPR_GPR0_OFFSET                      0x0000 /* SNVC GPR0 General Purpose Register */
#define IMXRT_IOMUXC_SNVS_GPR_GPR1_OFFSET                      0x0004 /* SNVC GPR1 General Purpose Register */
#define IMXRT_IOMUXC_SNVS_GPR_GPR2_OFFSET                      0x0008 /* SNVC GPR2 General Purpose Register */
#define IMXRT_IOMUXC_SNVS_GPR_GPR3_OFFSET                      0x000c /* SNVC GPR3 General Purpose Register */

/* Pad Mux Registers */
/* Pad Mux Register Indices (used by software for table lookups) */

#define IMXRT_PADMUX_GPIO_EMC_00_INDEX              0
#define IMXRT_PADMUX_GPIO_EMC_01_INDEX              1
#define IMXRT_PADMUX_GPIO_EMC_02_INDEX              2
#define IMXRT_PADMUX_GPIO_EMC_03_INDEX              3
#define IMXRT_PADMUX_GPIO_EMC_04_INDEX              4
#define IMXRT_PADMUX_GPIO_EMC_05_INDEX              5
#define IMXRT_PADMUX_GPIO_EMC_06_INDEX              6
#define IMXRT_PADMUX_GPIO_EMC_07_INDEX              7
#define IMXRT_PADMUX_GPIO_EMC_08_INDEX              8
#define IMXRT_PADMUX_GPIO_EMC_09_INDEX              9
#define IMXRT_PADMUX_GPIO_EMC_10_INDEX              10
#define IMXRT_PADMUX_GPIO_EMC_11_INDEX              11
#define IMXRT_PADMUX_GPIO_EMC_12_INDEX              12
#define IMXRT_PADMUX_GPIO_EMC_13_INDEX              13
#define IMXRT_PADMUX_GPIO_EMC_14_INDEX              14
#define IMXRT_PADMUX_GPIO_EMC_15_INDEX              15
#define IMXRT_PADMUX_GPIO_EMC_16_INDEX              16
#define IMXRT_PADMUX_GPIO_EMC_17_INDEX              17
#define IMXRT_PADMUX_GPIO_EMC_18_INDEX              18
#define IMXRT_PADMUX_GPIO_EMC_19_INDEX              19
#define IMXRT_PADMUX_GPIO_EMC_20_INDEX              20
#define IMXRT_PADMUX_GPIO_EMC_21_INDEX              21
#define IMXRT_PADMUX_GPIO_EMC_22_INDEX              22
#define IMXRT_PADMUX_GPIO_EMC_23_INDEX              23
#define IMXRT_PADMUX_GPIO_EMC_24_INDEX              24
#define IMXRT_PADMUX_GPIO_EMC_25_INDEX              25
#define IMXRT_PADMUX_GPIO_EMC_26_INDEX              26
#define IMXRT_PADMUX_GPIO_EMC_27_INDEX              27
#define IMXRT_PADMUX_GPIO_EMC_28_INDEX              28
#define IMXRT_PADMUX_GPIO_EMC_29_INDEX              29
#define IMXRT_PADMUX_GPIO_EMC_30_INDEX              30
#define IMXRT_PADMUX_GPIO_EMC_31_INDEX              31
#define IMXRT_PADMUX_GPIO_EMC_32_INDEX              32
#define IMXRT_PADMUX_GPIO_EMC_33_INDEX              33
#define IMXRT_PADMUX_GPIO_EMC_34_INDEX              34
#define IMXRT_PADMUX_GPIO_EMC_35_INDEX              35
#define IMXRT_PADMUX_GPIO_EMC_36_INDEX              36
#define IMXRT_PADMUX_GPIO_EMC_37_INDEX              37
#define IMXRT_PADMUX_GPIO_EMC_38_INDEX              38
#define IMXRT_PADMUX_GPIO_EMC_39_INDEX              39
#define IMXRT_PADMUX_GPIO_EMC_40_INDEX              40
#define IMXRT_PADMUX_GPIO_EMC_41_INDEX              41
#define IMXRT_PADMUX_GPIO_AD_B0_00_INDEX            42
#define IMXRT_PADMUX_GPIO_AD_B0_01_INDEX            43
#define IMXRT_PADMUX_GPIO_AD_B0_02_INDEX            44
#define IMXRT_PADMUX_GPIO_AD_B0_03_INDEX            45
#define IMXRT_PADMUX_GPIO_AD_B0_04_INDEX            46
#define IMXRT_PADMUX_GPIO_AD_B0_05_INDEX            47
#define IMXRT_PADMUX_GPIO_AD_B0_06_INDEX            48
#define IMXRT_PADMUX_GPIO_AD_B0_07_INDEX            49
#define IMXRT_PADMUX_GPIO_AD_B0_08_INDEX            50
#define IMXRT_PADMUX_GPIO_AD_B0_09_INDEX            51
#define IMXRT_PADMUX_GPIO_AD_B0_10_INDEX            52
#define IMXRT_PADMUX_GPIO_AD_B0_11_INDEX            53
#define IMXRT_PADMUX_GPIO_AD_B0_12_INDEX            54
#define IMXRT_PADMUX_GPIO_AD_B0_13_INDEX            55
#define IMXRT_PADMUX_GPIO_AD_B0_14_INDEX            56
#define IMXRT_PADMUX_GPIO_AD_B0_15_INDEX            57
#define IMXRT_PADMUX_GPIO_AD_B1_00_INDEX            58
#define IMXRT_PADMUX_GPIO_AD_B1_01_INDEX            59
#define IMXRT_PADMUX_GPIO_AD_B1_02_INDEX            60
#define IMXRT_PADMUX_GPIO_AD_B1_03_INDEX            61
#define IMXRT_PADMUX_GPIO_AD_B1_04_INDEX            62
#define IMXRT_PADMUX_GPIO_AD_B1_05_INDEX            63
#define IMXRT_PADMUX_GPIO_AD_B1_06_INDEX            64
#define IMXRT_PADMUX_GPIO_AD_B1_07_INDEX            65
#define IMXRT_PADMUX_GPIO_AD_B1_08_INDEX            66
#define IMXRT_PADMUX_GPIO_AD_B1_09_INDEX            67
#define IMXRT_PADMUX_GPIO_AD_B1_10_INDEX            68
#define IMXRT_PADMUX_GPIO_AD_B1_11_INDEX            69
#define IMXRT_PADMUX_GPIO_AD_B1_12_INDEX            70
#define IMXRT_PADMUX_GPIO_AD_B1_13_INDEX            71
#define IMXRT_PADMUX_GPIO_AD_B1_14_INDEX            72
#define IMXRT_PADMUX_GPIO_AD_B1_15_INDEX            73
#define IMXRT_PADMUX_GPIO_B0_00_INDEX               74
#define IMXRT_PADMUX_GPIO_B0_01_INDEX               75
#define IMXRT_PADMUX_GPIO_B0_02_INDEX               76
#define IMXRT_PADMUX_GPIO_B0_03_INDEX               77
#define IMXRT_PADMUX_GPIO_B0_04_INDEX               78
#define IMXRT_PADMUX_GPIO_B0_05_INDEX               79
#define IMXRT_PADMUX_GPIO_B0_06_INDEX               80
#define IMXRT_PADMUX_GPIO_B0_07_INDEX               81
#define IMXRT_PADMUX_GPIO_B0_08_INDEX               82
#define IMXRT_PADMUX_GPIO_B0_09_INDEX               83
#define IMXRT_PADMUX_GPIO_B0_10_INDEX               84
#define IMXRT_PADMUX_GPIO_B0_11_INDEX               85
#define IMXRT_PADMUX_GPIO_B0_12_INDEX               86
#define IMXRT_PADMUX_GPIO_B0_13_INDEX               87
#define IMXRT_PADMUX_GPIO_B0_14_INDEX               88
#define IMXRT_PADMUX_GPIO_B0_15_INDEX               89
#define IMXRT_PADMUX_GPIO_B1_00_INDEX               90
#define IMXRT_PADMUX_GPIO_B1_01_INDEX               91
#define IMXRT_PADMUX_GPIO_B1_02_INDEX               92
#define IMXRT_PADMUX_GPIO_B1_03_INDEX               93
#define IMXRT_PADMUX_GPIO_B1_04_INDEX               94
#define IMXRT_PADMUX_GPIO_B1_05_INDEX               95
#define IMXRT_PADMUX_GPIO_B1_06_INDEX               96
#define IMXRT_PADMUX_GPIO_B1_07_INDEX               97
#define IMXRT_PADMUX_GPIO_B1_08_INDEX               98
#define IMXRT_PADMUX_GPIO_B1_09_INDEX               99
#define IMXRT_PADMUX_GPIO_B1_10_INDEX               100
#define IMXRT_PADMUX_GPIO_B1_11_INDEX               101
#define IMXRT_PADMUX_GPIO_B1_12_INDEX               102
#define IMXRT_PADMUX_GPIO_B1_13_INDEX               103
#define IMXRT_PADMUX_GPIO_B1_14_INDEX               104
#define IMXRT_PADMUX_GPIO_B1_15_INDEX               105
#define IMXRT_PADMUX_GPIO_SD_B0_00_INDEX            106
#define IMXRT_PADMUX_GPIO_SD_B0_01_INDEX            107
#define IMXRT_PADMUX_GPIO_SD_B0_02_INDEX            108
#define IMXRT_PADMUX_GPIO_SD_B0_03_INDEX            109
#define IMXRT_PADMUX_GPIO_SD_B0_04_INDEX            110
#define IMXRT_PADMUX_GPIO_SD_B0_05_INDEX            111
#define IMXRT_PADMUX_GPIO_SD_B1_00_INDEX            112
#define IMXRT_PADMUX_GPIO_SD_B1_01_INDEX            113
#define IMXRT_PADMUX_GPIO_SD_B1_02_INDEX            114
#define IMXRT_PADMUX_GPIO_SD_B1_03_INDEX            115
#define IMXRT_PADMUX_GPIO_SD_B1_04_INDEX            116
#define IMXRT_PADMUX_GPIO_SD_B1_05_INDEX            117
#define IMXRT_PADMUX_GPIO_SD_B1_06_INDEX            118
#define IMXRT_PADMUX_GPIO_SD_B1_07_INDEX            119
#define IMXRT_PADMUX_GPIO_SD_B1_08_INDEX            120
#define IMXRT_PADMUX_GPIO_SD_B1_09_INDEX            121
#define IMXRT_PADMUX_GPIO_SD_B1_10_INDEX            122
#define IMXRT_PADMUX_GPIO_SD_B1_11_INDEX            123

#define IMXRT_PADMUX_WAKEUP_INDEX                   124
#define IMXRT_PADMUX_PMIC_ON_REQ_INDEX              125
#define IMXRT_PADMUX_PMIC_STBY_REQ_INDEX            126

#define IMXRT_PADMUX_NREGISTERS                     127

/* Pad Mux Register Offsets */

#define IMXRT_PADMUX_OFFSET(n)                      (0x0014 + ((unsigned int)(n) << 2))
#define IMXRT_PADMUX_OFFSET_SNVS(n)                 ((unsigned int)(n) << 2)

#define IMXRT_PADMUX_GPIO_EMC_00_OFFSET             0x0014
#define IMXRT_PADMUX_GPIO_EMC_01_OFFSET             0x0018
#define IMXRT_PADMUX_GPIO_EMC_02_OFFSET             0x001c
#define IMXRT_PADMUX_GPIO_EMC_03_OFFSET             0x0020
#define IMXRT_PADMUX_GPIO_EMC_04_OFFSET             0x0024
#define IMXRT_PADMUX_GPIO_EMC_05_OFFSET             0x0028
#define IMXRT_PADMUX_GPIO_EMC_06_OFFSET             0x002c
#define IMXRT_PADMUX_GPIO_EMC_07_OFFSET             0x0030
#define IMXRT_PADMUX_GPIO_EMC_08_OFFSET             0x0034
#define IMXRT_PADMUX_GPIO_EMC_09_OFFSET             0x0038
#define IMXRT_PADMUX_GPIO_EMC_10_OFFSET             0x003c
#define IMXRT_PADMUX_GPIO_EMC_11_OFFSET             0x0040
#define IMXRT_PADMUX_GPIO_EMC_12_OFFSET             0x0044
#define IMXRT_PADMUX_GPIO_EMC_13_OFFSET             0x0048
#define IMXRT_PADMUX_GPIO_EMC_14_OFFSET             0x004c
#define IMXRT_PADMUX_GPIO_EMC_15_OFFSET             0x0050
#define IMXRT_PADMUX_GPIO_EMC_16_OFFSET             0x0054
#define IMXRT_PADMUX_GPIO_EMC_17_OFFSET             0x0058
#define IMXRT_PADMUX_GPIO_EMC_18_OFFSET             0x005c
#define IMXRT_PADMUX_GPIO_EMC_19_OFFSET             0x0060
#define IMXRT_PADMUX_GPIO_EMC_20_OFFSET             0x0064
#define IMXRT_PADMUX_GPIO_EMC_21_OFFSET             0x0068
#define IMXRT_PADMUX_GPIO_EMC_22_OFFSET             0x006c
#define IMXRT_PADMUX_GPIO_EMC_23_OFFSET             0x0070
#define IMXRT_PADMUX_GPIO_EMC_24_OFFSET             0x0074
#define IMXRT_PADMUX_GPIO_EMC_25_OFFSET             0x0078
#define IMXRT_PADMUX_GPIO_EMC_26_OFFSET             0x007c
#define IMXRT_PADMUX_GPIO_EMC_27_OFFSET             0x0080
#define IMXRT_PADMUX_GPIO_EMC_28_OFFSET             0x0084
#define IMXRT_PADMUX_GPIO_EMC_29_OFFSET             0x0088
#define IMXRT_PADMUX_GPIO_EMC_30_OFFSET             0x008c
#define IMXRT_PADMUX_GPIO_EMC_31_OFFSET             0x0090
#define IMXRT_PADMUX_GPIO_EMC_32_OFFSET             0x0094
#define IMXRT_PADMUX_GPIO_EMC_33_OFFSET             0x0098
#define IMXRT_PADMUX_GPIO_EMC_34_OFFSET             0x009c
#define IMXRT_PADMUX_GPIO_EMC_35_OFFSET             0x00a0
#define IMXRT_PADMUX_GPIO_EMC_36_OFFSET             0x00a4
#define IMXRT_PADMUX_GPIO_EMC_37_OFFSET             0x00a8
#define IMXRT_PADMUX_GPIO_EMC_38_OFFSET             0x00ac
#define IMXRT_PADMUX_GPIO_EMC_39_OFFSET             0x00b0
#define IMXRT_PADMUX_GPIO_EMC_40_OFFSET             0x00b4
#define IMXRT_PADMUX_GPIO_EMC_41_OFFSET             0x00b8
#define IMXRT_PADMUX_GPIO_AD_B0_00_OFFSET           0x00bc
#define IMXRT_PADMUX_GPIO_AD_B0_01_OFFSET           0x00c0
#define IMXRT_PADMUX_GPIO_AD_B0_02_OFFSET           0x00c4
#define IMXRT_PADMUX_GPIO_AD_B0_03_OFFSET           0x00c8
#define IMXRT_PADMUX_GPIO_AD_B0_04_OFFSET           0x00cc
#define IMXRT_PADMUX_GPIO_AD_B0_05_OFFSET           0x00d0
#define IMXRT_PADMUX_GPIO_AD_B0_06_OFFSET           0x00d4
#define IMXRT_PADMUX_GPIO_AD_B0_07_OFFSET           0x00d8
#define IMXRT_PADMUX_GPIO_AD_B0_08_OFFSET           0x00dc
#define IMXRT_PADMUX_GPIO_AD_B0_09_OFFSET           0x00e0
#define IMXRT_PADMUX_GPIO_AD_B0_10_OFFSET           0x00e4
#define IMXRT_PADMUX_GPIO_AD_B0_11_OFFSET           0x00e8
#define IMXRT_PADMUX_GPIO_AD_B0_12_OFFSET           0x00ec
#define IMXRT_PADMUX_GPIO_AD_B0_13_OFFSET           0x00f0
#define IMXRT_PADMUX_GPIO_AD_B0_14_OFFSET           0x00f4
#define IMXRT_PADMUX_GPIO_AD_B0_15_OFFSET           0x00f8
#define IMXRT_PADMUX_GPIO_AD_B1_00_OFFSET           0x00fc
#define IMXRT_PADMUX_GPIO_AD_B1_01_OFFSET           0x0100
#define IMXRT_PADMUX_GPIO_AD_B1_02_OFFSET           0x0104
#define IMXRT_PADMUX_GPIO_AD_B1_03_OFFSET           0x0108
#define IMXRT_PADMUX_GPIO_AD_B1_04_OFFSET           0x010c
#define IMXRT_PADMUX_GPIO_AD_B1_05_OFFSET           0x0110
#define IMXRT_PADMUX_GPIO_AD_B1_06_OFFSET           0x0114
#define IMXRT_PADMUX_GPIO_AD_B1_07_OFFSET           0x0118
#define IMXRT_PADMUX_GPIO_AD_B1_08_OFFSET           0x011c
#define IMXRT_PADMUX_GPIO_AD_B1_09_OFFSET           0x0120
#define IMXRT_PADMUX_GPIO_AD_B1_10_OFFSET           0x0124
#define IMXRT_PADMUX_GPIO_AD_B1_11_OFFSET           0x0128
#define IMXRT_PADMUX_GPIO_AD_B1_12_OFFSET           0x012c
#define IMXRT_PADMUX_GPIO_AD_B1_13_OFFSET           0x0130
#define IMXRT_PADMUX_GPIO_AD_B1_14_OFFSET           0x0134
#define IMXRT_PADMUX_GPIO_AD_B1_15_OFFSET           0x0138
#define IMXRT_PADMUX_GPIO_B0_00_OFFSET              0x013c
#define IMXRT_PADMUX_GPIO_B0_01_OFFSET              0x0140
#define IMXRT_PADMUX_GPIO_B0_02_OFFSET              0x0144
#define IMXRT_PADMUX_GPIO_B0_03_OFFSET              0x0148
#define IMXRT_PADMUX_GPIO_B0_04_OFFSET              0x014c
#define IMXRT_PADMUX_GPIO_B0_05_OFFSET              0x0150
#define IMXRT_PADMUX_GPIO_B0_06_OFFSET              0x0154
#define IMXRT_PADMUX_GPIO_B0_07_OFFSET              0x0158
#define IMXRT_PADMUX_GPIO_B0_08_OFFSET              0x015c
#define IMXRT_PADMUX_GPIO_B0_09_OFFSET              0x0160
#define IMXRT_PADMUX_GPIO_B0_10_OFFSET              0x0164
#define IMXRT_PADMUX_GPIO_B0_11_OFFSET              0x0168
#define IMXRT_PADMUX_GPIO_B0_12_OFFSET              0x016c
#define IMXRT_PADMUX_GPIO_B0_13_OFFSET              0x0170
#define IMXRT_PADMUX_GPIO_B0_14_OFFSET              0x0174
#define IMXRT_PADMUX_GPIO_B0_15_OFFSET              0x0178
#define IMXRT_PADMUX_GPIO_B1_00_OFFSET              0x017c
#define IMXRT_PADMUX_GPIO_B1_01_OFFSET              0x0180
#define IMXRT_PADMUX_GPIO_B1_02_OFFSET              0x0184
#define IMXRT_PADMUX_GPIO_B1_03_OFFSET              0x0188
#define IMXRT_PADMUX_GPIO_B1_04_OFFSET              0x018c
#define IMXRT_PADMUX_GPIO_B1_05_OFFSET              0x0190
#define IMXRT_PADMUX_GPIO_B1_06_OFFSET              0x0194
#define IMXRT_PADMUX_GPIO_B1_07_OFFSET              0x0198
#define IMXRT_PADMUX_GPIO_B1_08_OFFSET              0x019c
#define IMXRT_PADMUX_GPIO_B1_09_OFFSET              0x01a0
#define IMXRT_PADMUX_GPIO_B1_10_OFFSET              0x01a4
#define IMXRT_PADMUX_GPIO_B1_11_OFFSET              0x01a8
#define IMXRT_PADMUX_GPIO_B1_12_OFFSET              0x01ac
#define IMXRT_PADMUX_GPIO_B1_13_OFFSET              0x01b0
#define IMXRT_PADMUX_GPIO_B1_14_OFFSET              0x01b4
#define IMXRT_PADMUX_GPIO_B1_15_OFFSET              0x01b8
#define IMXRT_PADMUX_GPIO_SD_B0_00_OFFSET           0x01bc
#define IMXRT_PADMUX_GPIO_SD_B0_01_OFFSET           0x01c0
#define IMXRT_PADMUX_GPIO_SD_B0_02_OFFSET           0x01c4
#define IMXRT_PADMUX_GPIO_SD_B0_03_OFFSET           0x01c8
#define IMXRT_PADMUX_GPIO_SD_B0_04_OFFSET           0x01cc
#define IMXRT_PADMUX_GPIO_SD_B0_05_OFFSET           0x01d0
#define IMXRT_PADMUX_GPIO_SD_B1_00_OFFSET           0x01d4
#define IMXRT_PADMUX_GPIO_SD_B1_01_OFFSET           0x01d8
#define IMXRT_PADMUX_GPIO_SD_B1_02_OFFSET           0x01dc
#define IMXRT_PADMUX_GPIO_SD_B1_03_OFFSET           0x01e0
#define IMXRT_PADMUX_GPIO_SD_B1_04_OFFSET           0x01e4
#define IMXRT_PADMUX_GPIO_SD_B1_05_OFFSET           0x01e8
#define IMXRT_PADMUX_GPIO_SD_B1_06_OFFSET           0x01ec
#define IMXRT_PADMUX_GPIO_SD_B1_07_OFFSET           0x01f0
#define IMXRT_PADMUX_GPIO_SD_B1_08_OFFSET           0x01f4
#define IMXRT_PADMUX_GPIO_SD_B1_09_OFFSET           0x01f8
#define IMXRT_PADMUX_GPIO_SD_B1_10_OFFSET           0x01fc
#define IMXRT_PADMUX_GPIO_SD_B1_11_OFFSET           0x0200


/* Pad Control Registers */
/* Pad Mux Register Indices (used by software for table lookups) */

#define IMXRT_PADCTL_GPIO_EMC_00_INDEX              0
#define IMXRT_PADCTL_GPIO_EMC_01_INDEX              1
#define IMXRT_PADCTL_GPIO_EMC_02_INDEX              2
#define IMXRT_PADCTL_GPIO_EMC_03_INDEX              3
#define IMXRT_PADCTL_GPIO_EMC_04_INDEX              4
#define IMXRT_PADCTL_GPIO_EMC_05_INDEX              5
#define IMXRT_PADCTL_GPIO_EMC_06_INDEX              6
#define IMXRT_PADCTL_GPIO_EMC_07_INDEX              7
#define IMXRT_PADCTL_GPIO_EMC_08_INDEX              8
#define IMXRT_PADCTL_GPIO_EMC_09_INDEX              9
#define IMXRT_PADCTL_GPIO_EMC_10_INDEX              10
#define IMXRT_PADCTL_GPIO_EMC_11_INDEX              11
#define IMXRT_PADCTL_GPIO_EMC_12_INDEX              12
#define IMXRT_PADCTL_GPIO_EMC_13_INDEX              13
#define IMXRT_PADCTL_GPIO_EMC_14_INDEX              14
#define IMXRT_PADCTL_GPIO_EMC_15_INDEX              15
#define IMXRT_PADCTL_GPIO_EMC_16_INDEX              16
#define IMXRT_PADCTL_GPIO_EMC_17_INDEX              17
#define IMXRT_PADCTL_GPIO_EMC_18_INDEX              18
#define IMXRT_PADCTL_GPIO_EMC_19_INDEX              19
#define IMXRT_PADCTL_GPIO_EMC_20_INDEX              20
#define IMXRT_PADCTL_GPIO_EMC_21_INDEX              21
#define IMXRT_PADCTL_GPIO_EMC_22_INDEX              22
#define IMXRT_PADCTL_GPIO_EMC_23_INDEX              23
#define IMXRT_PADCTL_GPIO_EMC_24_INDEX              24
#define IMXRT_PADCTL_GPIO_EMC_25_INDEX              25
#define IMXRT_PADCTL_GPIO_EMC_26_INDEX              26
#define IMXRT_PADCTL_GPIO_EMC_27_INDEX              27
#define IMXRT_PADCTL_GPIO_EMC_28_INDEX              28
#define IMXRT_PADCTL_GPIO_EMC_29_INDEX              29
#define IMXRT_PADCTL_GPIO_EMC_30_INDEX              30
#define IMXRT_PADCTL_GPIO_EMC_31_INDEX              31
#define IMXRT_PADCTL_GPIO_EMC_32_INDEX              32
#define IMXRT_PADCTL_GPIO_EMC_33_INDEX              33
#define IMXRT_PADCTL_GPIO_EMC_34_INDEX              34
#define IMXRT_PADCTL_GPIO_EMC_35_INDEX              35
#define IMXRT_PADCTL_GPIO_EMC_36_INDEX              36
#define IMXRT_PADCTL_GPIO_EMC_37_INDEX              37
#define IMXRT_PADCTL_GPIO_EMC_38_INDEX              38
#define IMXRT_PADCTL_GPIO_EMC_39_INDEX              39
#define IMXRT_PADCTL_GPIO_EMC_40_INDEX              40
#define IMXRT_PADCTL_GPIO_EMC_41_INDEX              41
#define IMXRT_PADCTL_GPIO_AD_B0_00_INDEX            42
#define IMXRT_PADCTL_GPIO_AD_B0_01_INDEX            43
#define IMXRT_PADCTL_GPIO_AD_B0_02_INDEX            44
#define IMXRT_PADCTL_GPIO_AD_B0_03_INDEX            45
#define IMXRT_PADCTL_GPIO_AD_B0_04_INDEX            46
#define IMXRT_PADCTL_GPIO_AD_B0_05_INDEX            47
#define IMXRT_PADCTL_GPIO_AD_B0_06_INDEX            48
#define IMXRT_PADCTL_GPIO_AD_B0_07_INDEX            49
#define IMXRT_PADCTL_GPIO_AD_B0_08_INDEX            50
#define IMXRT_PADCTL_GPIO_AD_B0_09_INDEX            51
#define IMXRT_PADCTL_GPIO_AD_B0_10_INDEX            52
#define IMXRT_PADCTL_GPIO_AD_B0_11_INDEX            53
#define IMXRT_PADCTL_GPIO_AD_B0_12_INDEX            54
#define IMXRT_PADCTL_GPIO_AD_B0_13_INDEX            55
#define IMXRT_PADCTL_GPIO_AD_B0_14_INDEX            56
#define IMXRT_PADCTL_GPIO_AD_B0_15_INDEX            57
#define IMXRT_PADCTL_GPIO_AD_B1_00_INDEX            58
#define IMXRT_PADCTL_GPIO_AD_B1_01_INDEX            59
#define IMXRT_PADCTL_GPIO_AD_B1_02_INDEX            60
#define IMXRT_PADCTL_GPIO_AD_B1_03_INDEX            61
#define IMXRT_PADCTL_GPIO_AD_B1_04_INDEX            62
#define IMXRT_PADCTL_GPIO_AD_B1_05_INDEX            63
#define IMXRT_PADCTL_GPIO_AD_B1_06_INDEX            64
#define IMXRT_PADCTL_GPIO_AD_B1_07_INDEX            65
#define IMXRT_PADCTL_GPIO_AD_B1_08_INDEX            66
#define IMXRT_PADCTL_GPIO_AD_B1_09_INDEX            67
#define IMXRT_PADCTL_GPIO_AD_B1_10_INDEX            68
#define IMXRT_PADCTL_GPIO_AD_B1_11_INDEX            69
#define IMXRT_PADCTL_GPIO_AD_B1_12_INDEX            70
#define IMXRT_PADCTL_GPIO_AD_B1_13_INDEX            71
#define IMXRT_PADCTL_GPIO_AD_B1_14_INDEX            72
#define IMXRT_PADCTL_GPIO_AD_B1_15_INDEX            73
#define IMXRT_PADCTL_GPIO_B0_00_INDEX               74
#define IMXRT_PADCTL_GPIO_B0_01_INDEX               75
#define IMXRT_PADCTL_GPIO_B0_02_INDEX               76
#define IMXRT_PADCTL_GPIO_B0_03_INDEX               77
#define IMXRT_PADCTL_GPIO_B0_04_INDEX               78
#define IMXRT_PADCTL_GPIO_B0_05_INDEX               79
#define IMXRT_PADCTL_GPIO_B0_06_INDEX               80
#define IMXRT_PADCTL_GPIO_B0_07_INDEX               81
#define IMXRT_PADCTL_GPIO_B0_08_INDEX               82
#define IMXRT_PADCTL_GPIO_B0_09_INDEX               83
#define IMXRT_PADCTL_GPIO_B0_10_INDEX               84
#define IMXRT_PADCTL_GPIO_B0_11_INDEX               85
#define IMXRT_PADCTL_GPIO_B0_12_INDEX               86
#define IMXRT_PADCTL_GPIO_B0_13_INDEX               87
#define IMXRT_PADCTL_GPIO_B0_14_INDEX               88
#define IMXRT_PADCTL_GPIO_B0_15_INDEX               89
#define IMXRT_PADCTL_GPIO_B1_00_INDEX               90
#define IMXRT_PADCTL_GPIO_B1_01_INDEX               91
#define IMXRT_PADCTL_GPIO_B1_02_INDEX               92
#define IMXRT_PADCTL_GPIO_B1_03_INDEX               93
#define IMXRT_PADCTL_GPIO_B1_04_INDEX               94
#define IMXRT_PADCTL_GPIO_B1_05_INDEX               95
#define IMXRT_PADCTL_GPIO_B1_06_INDEX               96
#define IMXRT_PADCTL_GPIO_B1_07_INDEX               97
#define IMXRT_PADCTL_GPIO_B1_08_INDEX               98
#define IMXRT_PADCTL_GPIO_B1_09_INDEX               99
#define IMXRT_PADCTL_GPIO_B1_10_INDEX               100
#define IMXRT_PADCTL_GPIO_B1_11_INDEX               101
#define IMXRT_PADCTL_GPIO_B1_12_INDEX               102
#define IMXRT_PADCTL_GPIO_B1_13_INDEX               103
#define IMXRT_PADCTL_GPIO_B1_14_INDEX               104
#define IMXRT_PADCTL_GPIO_B1_15_INDEX               105
#define IMXRT_PADCTL_GPIO_SD_B0_00_INDEX            106
#define IMXRT_PADCTL_GPIO_SD_B0_01_INDEX            107
#define IMXRT_PADCTL_GPIO_SD_B0_02_INDEX            108
#define IMXRT_PADCTL_GPIO_SD_B0_03_INDEX            109
#define IMXRT_PADCTL_GPIO_SD_B0_04_INDEX            110
#define IMXRT_PADCTL_GPIO_SD_B0_05_INDEX            111
#define IMXRT_PADCTL_GPIO_SD_B1_00_INDEX            112
#define IMXRT_PADCTL_GPIO_SD_B1_01_INDEX            113
#define IMXRT_PADCTL_GPIO_SD_B1_02_INDEX            114
#define IMXRT_PADCTL_GPIO_SD_B1_03_INDEX            115
#define IMXRT_PADCTL_GPIO_SD_B1_04_INDEX            116
#define IMXRT_PADCTL_GPIO_SD_B1_05_INDEX            117
#define IMXRT_PADCTL_GPIO_SD_B1_06_INDEX            118
#define IMXRT_PADCTL_GPIO_SD_B1_07_INDEX            119
#define IMXRT_PADCTL_GPIO_SD_B1_08_INDEX            120
#define IMXRT_PADCTL_GPIO_SD_B1_09_INDEX            121
#define IMXRT_PADCTL_GPIO_SD_B1_10_INDEX            122
#define IMXRT_PADCTL_GPIO_SD_B1_11_INDEX            123

#define IMXRT_PADCTL_WAKEUP_INDEX                   124
#define IMXRT_PADCTL_PMIC_ON_REQ_INDEX              125
#define IMXRT_PADCTL_PMIC_STBY_REQ_INDEX            126

#define IMXRT_PADCTL_NREGISTERS                     127

/* Pad Mux Register Offsets */

#define IMXRT_PADCTL_OFFSET(n)                      (0x0204 + ((unsigned int)(n) << 2))
#define IMXRT_PADCTL_OFFSET_SNVS(n)                 (0x18 + ((unsigned int)(n) << 2))

#define IMXRT_PADCTL_GPIO_EMC_00_OFFSET             0x0204
#define IMXRT_PADCTL_GPIO_EMC_01_OFFSET             0x0208
#define IMXRT_PADCTL_GPIO_EMC_02_OFFSET             0x020c
#define IMXRT_PADCTL_GPIO_EMC_03_OFFSET             0x0210
#define IMXRT_PADCTL_GPIO_EMC_04_OFFSET             0x0214
#define IMXRT_PADCTL_GPIO_EMC_05_OFFSET             0x0218
#define IMXRT_PADCTL_GPIO_EMC_06_OFFSET             0x021c
#define IMXRT_PADCTL_GPIO_EMC_07_OFFSET             0x0220
#define IMXRT_PADCTL_GPIO_EMC_08_OFFSET             0x0224
#define IMXRT_PADCTL_GPIO_EMC_09_OFFSET             0x0228
#define IMXRT_PADCTL_GPIO_EMC_10_OFFSET             0x022c
#define IMXRT_PADCTL_GPIO_EMC_11_OFFSET             0x0230
#define IMXRT_PADCTL_GPIO_EMC_12_OFFSET             0x0234
#define IMXRT_PADCTL_GPIO_EMC_13_OFFSET             0x0238
#define IMXRT_PADCTL_GPIO_EMC_14_OFFSET             0x023c
#define IMXRT_PADCTL_GPIO_EMC_15_OFFSET             0x0240
#define IMXRT_PADCTL_GPIO_EMC_16_OFFSET             0x0244
#define IMXRT_PADCTL_GPIO_EMC_17_OFFSET             0x0248
#define IMXRT_PADCTL_GPIO_EMC_18_OFFSET             0x024c
#define IMXRT_PADCTL_GPIO_EMC_19_OFFSET             0x0250
#define IMXRT_PADCTL_GPIO_EMC_20_OFFSET             0x0254
#define IMXRT_PADCTL_GPIO_EMC_21_OFFSET             0x0258
#define IMXRT_PADCTL_GPIO_EMC_22_OFFSET             0x025c
#define IMXRT_PADCTL_GPIO_EMC_23_OFFSET             0x0260
#define IMXRT_PADCTL_GPIO_EMC_24_OFFSET             0x0264
#define IMXRT_PADCTL_GPIO_EMC_25_OFFSET             0x0268
#define IMXRT_PADCTL_GPIO_EMC_26_OFFSET             0x026c
#define IMXRT_PADCTL_GPIO_EMC_27_OFFSET             0x0270
#define IMXRT_PADCTL_GPIO_EMC_28_OFFSET             0x0274
#define IMXRT_PADCTL_GPIO_EMC_29_OFFSET             0x0278
#define IMXRT_PADCTL_GPIO_EMC_30_OFFSET             0x027c
#define IMXRT_PADCTL_GPIO_EMC_31_OFFSET             0x0280
#define IMXRT_PADCTL_GPIO_EMC_32_OFFSET             0x0284
#define IMXRT_PADCTL_GPIO_EMC_33_OFFSET             0x0288
#define IMXRT_PADCTL_GPIO_EMC_34_OFFSET             0x028c
#define IMXRT_PADCTL_GPIO_EMC_35_OFFSET             0x0290
#define IMXRT_PADCTL_GPIO_EMC_36_OFFSET             0x0294
#define IMXRT_PADCTL_GPIO_EMC_37_OFFSET             0x0298
#define IMXRT_PADCTL_GPIO_EMC_38_OFFSET             0x029c
#define IMXRT_PADCTL_GPIO_EMC_39_OFFSET             0x02a0
#define IMXRT_PADCTL_GPIO_EMC_40_OFFSET             0x02a4
#define IMXRT_PADCTL_GPIO_EMC_41_OFFSET             0x02a8
#define IMXRT_PADCTL_GPIO_AD_B0_00_OFFSET           0x02ac
#define IMXRT_PADCTL_GPIO_AD_B0_01_OFFSET           0x02b0
#define IMXRT_PADCTL_GPIO_AD_B0_02_OFFSET           0x02b4
#define IMXRT_PADCTL_GPIO_AD_B0_03_OFFSET           0x02b8
#define IMXRT_PADCTL_GPIO_AD_B0_04_OFFSET           0x02bc
#define IMXRT_PADCTL_GPIO_AD_B0_05_OFFSET           0x02c0
#define IMXRT_PADCTL_GPIO_AD_B0_06_OFFSET           0x02c4
#define IMXRT_PADCTL_GPIO_AD_B0_07_OFFSET           0x02c8
#define IMXRT_PADCTL_GPIO_AD_B0_08_OFFSET           0x02cc
#define IMXRT_PADCTL_GPIO_AD_B0_09_OFFSET           0x02d0
#define IMXRT_PADCTL_GPIO_AD_B0_10_OFFSET           0x02d4
#define IMXRT_PADCTL_GPIO_AD_B0_11_OFFSET           0x02d8
#define IMXRT_PADCTL_GPIO_AD_B0_12_OFFSET           0x02dc
#define IMXRT_PADCTL_GPIO_AD_B0_13_OFFSET           0x02e0
#define IMXRT_PADCTL_GPIO_AD_B0_14_OFFSET           0x02e4
#define IMXRT_PADCTL_GPIO_AD_B0_15_OFFSET           0x02e8
#define IMXRT_PADCTL_GPIO_AD_B1_00_OFFSET           0x02ec
#define IMXRT_PADCTL_GPIO_AD_B1_01_OFFSET           0x02f0
#define IMXRT_PADCTL_GPIO_AD_B1_02_OFFSET           0x02f4
#define IMXRT_PADCTL_GPIO_AD_B1_03_OFFSET           0x02f8
#define IMXRT_PADCTL_GPIO_AD_B1_04_OFFSET           0x02fc
#define IMXRT_PADCTL_GPIO_AD_B1_05_OFFSET           0x0300
#define IMXRT_PADCTL_GPIO_AD_B1_06_OFFSET           0x0304
#define IMXRT_PADCTL_GPIO_AD_B1_07_OFFSET           0x0308
#define IMXRT_PADCTL_GPIO_AD_B1_08_OFFSET           0x030c
#define IMXRT_PADCTL_GPIO_AD_B1_09_OFFSET           0x0310
#define IMXRT_PADCTL_GPIO_AD_B1_10_OFFSET           0x0314
#define IMXRT_PADCTL_GPIO_AD_B1_11_OFFSET           0x0318
#define IMXRT_PADCTL_GPIO_AD_B1_12_OFFSET           0x031c
#define IMXRT_PADCTL_GPIO_AD_B1_13_OFFSET           0x0320
#define IMXRT_PADCTL_GPIO_AD_B1_14_OFFSET           0x0324
#define IMXRT_PADCTL_GPIO_AD_B1_15_OFFSET           0x0328
#define IMXRT_PADCTL_GPIO_B0_00_OFFSET              0x032c
#define IMXRT_PADCTL_GPIO_B0_01_OFFSET              0x0330
#define IMXRT_PADCTL_GPIO_B0_02_OFFSET              0x0334
#define IMXRT_PADCTL_GPIO_B0_03_OFFSET              0x0338
#define IMXRT_PADCTL_GPIO_B0_04_OFFSET              0x033c
#define IMXRT_PADCTL_GPIO_B0_05_OFFSET              0x0340
#define IMXRT_PADCTL_GPIO_B0_06_OFFSET              0x0344
#define IMXRT_PADCTL_GPIO_B0_07_OFFSET              0x0348
#define IMXRT_PADCTL_GPIO_B0_08_OFFSET              0x034c
#define IMXRT_PADCTL_GPIO_B0_09_OFFSET              0x0350
#define IMXRT_PADCTL_GPIO_B0_10_OFFSET              0x0354
#define IMXRT_PADCTL_GPIO_B0_11_OFFSET              0x0358
#define IMXRT_PADCTL_GPIO_B0_12_OFFSET              0x035c
#define IMXRT_PADCTL_GPIO_B0_13_OFFSET              0x0360
#define IMXRT_PADCTL_GPIO_B0_14_OFFSET              0x0364
#define IMXRT_PADCTL_GPIO_B0_15_OFFSET              0x0368
#define IMXRT_PADCTL_GPIO_B1_00_OFFSET              0x036c
#define IMXRT_PADCTL_GPIO_B1_01_OFFSET              0x0370
#define IMXRT_PADCTL_GPIO_B1_02_OFFSET              0x0374
#define IMXRT_PADCTL_GPIO_B1_03_OFFSET              0x0378
#define IMXRT_PADCTL_GPIO_B1_04_OFFSET              0x037c
#define IMXRT_PADCTL_GPIO_B1_05_OFFSET              0x0380
#define IMXRT_PADCTL_GPIO_B1_06_OFFSET              0x0384
#define IMXRT_PADCTL_GPIO_B1_07_OFFSET              0x0388
#define IMXRT_PADCTL_GPIO_B1_08_OFFSET              0x038c
#define IMXRT_PADCTL_GPIO_B1_09_OFFSET              0x0390
#define IMXRT_PADCTL_GPIO_B1_10_OFFSET              0x0394
#define IMXRT_PADCTL_GPIO_B1_11_OFFSET              0x0398
#define IMXRT_PADCTL_GPIO_B1_12_OFFSET              0x039c
#define IMXRT_PADCTL_GPIO_B1_13_OFFSET              0x03a0
#define IMXRT_PADCTL_GPIO_B1_14_OFFSET              0x03a4
#define IMXRT_PADCTL_GPIO_B1_15_OFFSET              0x03a8
#define IMXRT_PADCTL_GPIO_SD_B0_00_OFFSET           0x03ac
#define IMXRT_PADCTL_GPIO_SD_B0_01_OFFSET           0x03b0
#define IMXRT_PADCTL_GPIO_SD_B0_02_OFFSET           0x03b4
#define IMXRT_PADCTL_GPIO_SD_B0_03_OFFSET           0x03b8
#define IMXRT_PADCTL_GPIO_SD_B0_04_OFFSET           0x03bc
#define IMXRT_PADCTL_GPIO_SD_B0_05_OFFSET           0x03c0
#define IMXRT_PADCTL_GPIO_SD_B1_00_OFFSET           0x03c4
#define IMXRT_PADCTL_GPIO_SD_B1_01_OFFSET           0x03c8
#define IMXRT_PADCTL_GPIO_SD_B1_02_OFFSET           0x03cc
#define IMXRT_PADCTL_GPIO_SD_B1_03_OFFSET           0x03d0
#define IMXRT_PADCTL_GPIO_SD_B1_04_OFFSET           0x03d4
#define IMXRT_PADCTL_GPIO_SD_B1_05_OFFSET           0x03d8
#define IMXRT_PADCTL_GPIO_SD_B1_06_OFFSET           0x03dc
#define IMXRT_PADCTL_GPIO_SD_B1_07_OFFSET           0x03e0
#define IMXRT_PADCTL_GPIO_SD_B1_08_OFFSET           0x03e4
#define IMXRT_PADCTL_GPIO_SD_B1_09_OFFSET           0x03e8
#define IMXRT_PADCTL_GPIO_SD_B1_10_OFFSET           0x03ec
#define IMXRT_PADCTL_GPIO_SD_B1_11_OFFSET           0x03f0

/* Select Input Register Offsets */

#define IMXRT_INPUT_ANATOP_USB_OTG1_ID_OFFSET       0x03f4
#define IMXRT_INPUT_ANATOP_USB_OTG2_ID_OFFSET       0x03f8
#define IMXRT_INPUT_CCM_PMIC_READY_OFFSET           0x03fc
#define IMXRT_INPUT_CSI_DATA02_OFFSET               0x0400
#define IMXRT_INPUT_CSI_DATA03_OFFSET               0x0404
#define IMXRT_INPUT_CSI_DATA04_OFFSET               0x0408
#define IMXRT_INPUT_CSI_DATA05_OFFSET               0x040c
#define IMXRT_INPUT_CSI_DATA06_OFFSET               0x0410
#define IMXRT_INPUT_CSI_DATA07_OFFSET               0x0414
#define IMXRT_INPUT_CSI_DATA08_OFFSET               0x0418
#define IMXRT_INPUT_CSI_DATA09_OFFSET               0x041c
#define IMXRT_INPUT_CSI_HSYNC_OFFSET                0x0420
#define IMXRT_INPUT_CSI_PIXCLK_OFFSET               0x0424
#define IMXRT_INPUT_CSI_VSYNC_OFFSET                0x0428
#define IMXRT_INPUT_ENET_IPG_CLK_RMII_OFFSET        0x042c
#define IMXRT_INPUT_ENET_MDIO_OFFSET                0x0430
#define IMXRT_INPUT_ENET0_RXDATA_OFFSET             0x0434
#define IMXRT_INPUT_ENET1_RXDATA_OFFSET             0x0438
#define IMXRT_INPUT_ENET_RXEN_OFFSET                0x043c
#define IMXRT_INPUT_ENET_RXERR_OFFSET               0x0440
#define IMXRT_INPUT_ENET0_TIMER_OFFSET              0x0444
#define IMXRT_INPUT_ENET_TXCLK_OFFSET               0x0448
#define IMXRT_INPUT_FLEXCAN1_RX_OFFSET              0x044c
#define IMXRT_INPUT_FLEXCAN2_RX_OFFSET              0x0450
#define IMXRT_INPUT_FLEXPWM1_PWMA3_OFFSET           0x0454
#define IMXRT_INPUT_FLEXPWM1_PWMA0_OFFSET           0x0458
#define IMXRT_INPUT_FLEXPWM1_PWMA1_OFFSET           0x045c
#define IMXRT_INPUT_FLEXPWM1_PWMA2_OFFSET           0x0460
#define IMXRT_INPUT_FLEXPWM1_PWMB3_OFFSET           0x0464
#define IMXRT_INPUT_FLEXPWM1_PWMB0_OFFSET           0x0468
#define IMXRT_INPUT_FLEXPWM1_PWMB1_OFFSET           0x046c
#define IMXRT_INPUT_FLEXPWM1_PWMB2_OFFSET           0x0470
#define IMXRT_INPUT_FLEXPWM2_PWMA3_OFFSET           0x0474
#define IMXRT_INPUT_FLEXPWM2_PWMA0_OFFSET           0x0478
#define IMXRT_INPUT_FLEXPWM2_PWMA1_OFFSET           0x047c
#define IMXRT_INPUT_FLEXPWM2_PWMA2_OFFSET           0x0480
#define IMXRT_INPUT_FLEXPWM2_PWMB3_OFFSET           0x0484
#define IMXRT_INPUT_FLEXPWM2_PWMB0_OFFSET           0x0488
#define IMXRT_INPUT_FLEXPWM2_PWMB1_OFFSET           0x048c
#define IMXRT_INPUT_FLEXPWM2_PWMB2_OFFSET           0x0490
#define IMXRT_INPUT_FLEXPWM4_PWMA0_OFFSET           0x0494
#define IMXRT_INPUT_FLEXPWM4_PWMA1_OFFSET           0x0498
#define IMXRT_INPUT_FLEXPWM4_PWMA2_OFFSET           0x049c
#define IMXRT_INPUT_FLEXPWM4_PWMA3_OFFSET           0x04a0
#define IMXRT_INPUT_FLEXSPIA_DQS_OFFSET             0x04a4
#define IMXRT_INPUT_FLEXSPIA_DATA0_OFFSET           0x04a8
#define IMXRT_INPUT_FLEXSPIA_DATA1_OFFSET           0x04ac
#define IMXRT_INPUT_FLEXSPIA_DATA2_OFFSET           0x04b0
#define IMXRT_INPUT_FLEXSPIA_DATA3_OFFSET           0x04b4
#define IMXRT_INPUT_FLEXSPIB_DATA0_OFFSET           0x04b8
#define IMXRT_INPUT_FLEXSPIB_DATA1_OFFSET           0x04bc
#define IMXRT_INPUT_FLEXSPIB_DATA2_OFFSET           0x04c0
#define IMXRT_INPUT_FLEXSPIB_DATA3_OFFSET           0x04c4
#define IMXRT_INPUT_FLEXSPIA_SCK_OFFSET             0x04c8
#define IMXRT_INPUT_LPI2C1_SCL_OFFSET               0x04cc
#define IMXRT_INPUT_LPI2C1_SDA_OFFSET               0x04d0
#define IMXRT_INPUT_LPI2C2_SCL_OFFSET               0x04d4
#define IMXRT_INPUT_LPI2C2_SDA_OFFSET               0x04d8
#define IMXRT_INPUT_LPI2C3_SCL_OFFSET               0x04dc
#define IMXRT_INPUT_LPI2C3_SDA_OFFSET               0x04e0
#define IMXRT_INPUT_LPI2C4_SCL_OFFSET               0x04e4
#define IMXRT_INPUT_LPI2C4_SDA_OFFSET               0x04e8
#define IMXRT_INPUT_LPSPI1_PCS0_OFFSET              0x04ec
#define IMXRT_INPUT_LPSPI1_SCK_OFFSET               0x04f0
#define IMXRT_INPUT_LPSPI1_SDI_OFFSET               0x04f4
#define IMXRT_INPUT_LPSPI1_SDO_OFFSET               0x04f8
#define IMXRT_INPUT_LPSPI2_PCS0_OFFSET              0x04fc
#define IMXRT_INPUT_LPSPI2_SCK_OFFSET               0x0500
#define IMXRT_INPUT_LPSPI2_SDI_OFFSET               0x0504
#define IMXRT_INPUT_LPSPI2_SDO_OFFSET               0x0508
#define IMXRT_INPUT_LPSPI3_PCS0_OFFSET              0x050c
#define IMXRT_INPUT_LPSPI3_SCK_OFFSET               0x0510
#define IMXRT_INPUT_LPSPI3_SDI_OFFSET               0x0514
#define IMXRT_INPUT_LPSPI3_SDO_OFFSET               0x0518
#define IMXRT_INPUT_LPSPI4_PCS0_OFFSET              0x051c
#define IMXRT_INPUT_LPSPI4_SCK_OFFSET               0x0520
#define IMXRT_INPUT_LPSPI4_SDI_OFFSET               0x0524
#define IMXRT_INPUT_LPSPI4_SDO_OFFSET               0x0528
#define IMXRT_INPUT_LPUART2_RX_OFFSET               0x052c
#define IMXRT_INPUT_LPUART2_TX_OFFSET               0x0530
#define IMXRT_INPUT_LPUART3_CTS_B_OFFSET            0x0534
#define IMXRT_INPUT_LPUART3_RX_OFFSET               0x0538
#define IMXRT_INPUT_LPUART3_TX_OFFSET               0x053c
#define IMXRT_INPUT_LPUART4_RX_OFFSET               0x0540
#define IMXRT_INPUT_LPUART4_TX_OFFSET               0x0544
#define IMXRT_INPUT_LPUART5_RX_OFFSET               0x0548
#define IMXRT_INPUT_LPUART5_TX_OFFSET               0x054c
#define IMXRT_INPUT_LPUART6_RX_OFFSET               0x0550
#define IMXRT_INPUT_LPUART6_TX_OFFSET               0x0554
#define IMXRT_INPUT_LPUART7_RX_OFFSET               0x0558
#define IMXRT_INPUT_LPUART7_TX_OFFSET               0x055c
#define IMXRT_INPUT_LPUART8_RX_OFFSET               0x0560
#define IMXRT_INPUT_LPUART8_TX_OFFSET               0x0564
#define IMXRT_INPUT_NMI_GLUE_NMI_OFFSET             0x0568
#define IMXRT_INPUT_QTIMER2_TIMER0_OFFSET           0x056c
#define IMXRT_INPUT_QTIMER2_TIMER1_OFFSET           0x0570
#define IMXRT_INPUT_QTIMER2_TIMER2_OFFSET           0x0574
#define IMXRT_INPUT_QTIMER2_TIMER3_OFFSET           0x0578
#define IMXRT_INPUT_QTIMER3_TIMER0_OFFSET           0x057c
#define IMXRT_INPUT_QTIMER3_TIMER1_OFFSET           0x0580
#define IMXRT_INPUT_QTIMER3_TIMER2_OFFSET           0x0584
#define IMXRT_INPUT_QTIMER3_TIMER3_OFFSET           0x0588
#define IMXRT_INPUT_SAI1_MCLK2_OFFSET               0x058c
#define IMXRT_INPUT_SAI1_RX_BCLK_OFFSET             0x0590
#define IMXRT_INPUT_SAI1_RX_DATA0_OFFSET            0x0594
#define IMXRT_INPUT_SAI1_RX_DATA1_OFFSET            0x0598
#define IMXRT_INPUT_SAI1_RX_DATA2_OFFSET            0x059c
#define IMXRT_INPUT_SAI1_RX_DATA3_OFFSET            0x05a0
#define IMXRT_INPUT_SAI1_RX_SYNC_OFFSET             0x05a4
#define IMXRT_INPUT_SAI1_TX_BCLK_OFFSET             0x05a8
#define IMXRT_INPUT_SAI1_TX_SYNC_OFFSET             0x05ac
#define IMXRT_INPUT_SAI2_MCLK2_OFFSET               0x05b0
#define IMXRT_INPUT_SAI2_RX_BCLK_OFFSET             0x05b4
#define IMXRT_INPUT_SAI2_RX_DATA0_OFFSET            0x05b8
#define IMXRT_INPUT_SAI2_RX_SYNC_OFFSET             0x05bc
#define IMXRT_INPUT_SAI2_TX_BCLK_OFFSET             0x05c0
#define IMXRT_INPUT_SAI2_TX_SYNC_OFFSET             0x05c4
#define IMXRT_INPUT_SPDIF_IN_OFFSET                 0x05c8
#define IMXRT_INPUT_USB_OTG2_OC_OFFSET              0x05cc
#define IMXRT_INPUT_USB_OTG1_OC_OFFSET              0x05d0
#define IMXRT_INPUT_USDHC1_CD_B_OFFSET              0x05d4
#define IMXRT_INPUT_USDHC1_WP_OFFSET                0x05d8
#define IMXRT_INPUT_USDHC2_CLK_OFFSET               0x05dc
#define IMXRT_INPUT_USDHC2_CD_B_OFFSET              0x05e0
#define IMXRT_INPUT_USDHC2_CMD_OFFSET               0x05e4
#define IMXRT_INPUT_USDHC2_DATA0_OFFSET             0x05e8
#define IMXRT_INPUT_USDHC2_DATA1_OFFSET             0x05ec
#define IMXRT_INPUT_USDHC2_DATA2_OFFSET             0x05f0
#define IMXRT_INPUT_USDHC2_DATA3_OFFSET             0x05f4
#define IMXRT_INPUT_USDHC2_DATA4_OFFSET             0x05f8
#define IMXRT_INPUT_USDHC2_DATA5_OFFSET             0x05fc
#define IMXRT_INPUT_USDHC2_DATA6_OFFSET             0x0600
#define IMXRT_INPUT_USDHC2_DATA7_OFFSET             0x0604
#define IMXRT_INPUT_USDHC2_WP_OFFSET                0x0608
#define IMXRT_INPUT_XBAR1_IN02_OFFSET               0x060c
#define IMXRT_INPUT_XBAR1_IN03_OFFSET               0x0610
#define IMXRT_INPUT_XBAR1_IN04_OFFSET               0x0614
#define IMXRT_INPUT_XBAR1_IN05_OFFSET               0x0618
#define IMXRT_INPUT_XBAR1_IN06_OFFSET               0x061c
#define IMXRT_INPUT_XBAR1_IN07_OFFSET               0x0620
#define IMXRT_INPUT_XBAR1_IN08_OFFSET               0x0624
#define IMXRT_INPUT_XBAR1_IN09_OFFSET               0x0628
#define IMXRT_INPUT_XBAR1_IN17_OFFSET               0x062c
#define IMXRT_INPUT_XBAR1_IN18_OFFSET               0x0630
#define IMXRT_INPUT_XBAR1_IN20_OFFSET               0x0634
#define IMXRT_INPUT_XBAR1_IN22_OFFSET               0x0638
#define IMXRT_INPUT_XBAR1_IN23_OFFSET               0x063c
#define IMXRT_INPUT_XBAR1_IN24_OFFSET               0x0640
#define IMXRT_INPUT_XBAR1_IN14_OFFSET               0x0644
#define IMXRT_INPUT_XBAR1_IN15_OFFSET               0x0648
#define IMXRT_INPUT_XBAR1_IN16_OFFSET               0x064c
#define IMXRT_INPUT_XBAR1_IN25_OFFSET               0x0650
#define IMXRT_INPUT_XBAR1_IN19_OFFSET               0x0654
#define IMXRT_INPUT_XBAR1_IN21_OFFSET               0x0658

/* Register addresses ***************************************************************/

#define IMXRT_IOMUXC_GPR_GPR0            (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR0_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR1            (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR1_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR2            (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR2_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR3            (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR3_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR4            (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR4_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR5            (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR5_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR6            (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR6_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR7            (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR7_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR8            (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR8_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR9            (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR9_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR10           (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR10_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR11           (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR11_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR12           (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR12_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR13           (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR13_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR14           (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR14_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR15           (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR15_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR16           (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR16_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR17           (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR17_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR18           (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR18_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR19           (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR19_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR20           (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR20_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR21           (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR21_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR22           (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR22_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR23           (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR23_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR24           (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR24_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR25           (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR25_OFFSET)

#define IMXRT_IOMUXC_SNVS_SW_MUX_CTL_PAD_WAKEUP         (IMXRT_IOMUXCSNVS_BASE + IMXRT_IOMUXC_SNVS_SW_MUX_CTL_PAD_WAKEUP_OFFSET)
#define IMXRT_IOMUXC_SNVS_SW_MUX_CTL_PAD_PMIC_ON_REQ   (IMXRT_IOMUXCSNVS_BASE + IMXRT_IOMUXC_SNVS_SW_MUX_CTL_PAD_PMIC_ON_REQ_OFFSET)
#define IMXRT_IOMUXC_SNVS_SW_MUX_CTL_PAD_PMIC_STBY_REQ (IMXRT_IOMUXCSNVS_BASE + IMXRT_IOMUXC_SNVS_SW_MUX_CTL_PAD_PMIC_STBY_REQ_OFFSET)
#define IMXRT_IOMUXC_SNVS_SW_PAD_CTL_PAD_TEST_MODE     (IMXRT_IOMUXCSNVS_BASE + IMXRT_IOMUXC_SNVS_SW_PAD_CTL_PAD_TEST_MODE_OFFSET)
#define IMXRT_IOMUXC_SNVS_SW_PAD_CTL_PAD_POR_B         (IMXRT_IOMUXCSNVS_BASE + IMXRT_IOMUXC_SNVS_SW_PAD_CTL_PAD_POR_B_OFFSET)
#define IMXRT_IOMUXC_SNVS_SW_PAD_CTL_PAD_ONOFF         (IMXRT_IOMUXCSNVS_BASE + IMXRT_IOMUXC_SNVS_SW_PAD_CTL_PAD_ONOFF_OFFSET)
#define IMXRT_IOMUXC_SNVS_SW_PAD_CTL_PAD_WAKEUP        (IMXRT_IOMUXCSNVS_BASE + IMXRT_IOMUXC_SNVS_SW_PAD_CTL_PAD_WAKEUP_OFFSET)
#define IMXRT_IOMUXC_SNVS_SW_PAD_CTL_PAD_PMIC_ON_REQ   (IMXRT_IOMUXCSNVS_BASE + IMXRT_IOMUXC_SNVS_SW_PAD_CTL_PAD_PMIC_ON_REQ_OFFSET)
#define IMXRT_IOMUXC_SNVS_SW_PAD_CTL_PAD_PMIC_STBY_REQ (IMXRT_IOMUXCSNVS_BASE + IMXRT_IOMUXC_SNVS_SW_PAD_CTL_PAD_PMIC_STBY_REQ_OFFSET)

#define IMXRT_IOMUXC_SNVS_GPR_GPR0       (IMXRT_IOMUXCSNVSGPR_BASE + IMXRT_IOMUXC_SNVS_GPR_GPR0_OFFSET)
#define IMXRT_IOMUXC_SNVS_GPR_GPR1       (IMXRT_IOMUXCSNVSGPR_BASE + IMXRT_IOMUXC_SNVS_GPR_GPR1_OFFSET)
#define IMXRT_IOMUXC_SNVS_GPR_GPR2       (IMXRT_IOMUXCSNVSGPR_BASE + IMXRT_IOMUXC_SNVS_GPR_GPR2_OFFSET)
#define IMXRT_IOMUXC_SNVS_GPR_GPR3       (IMXRT_IOMUXCSNVSGPR_BASE + IMXRT_IOMUXC_SNVS_GPR_GPR3_OFFSET)

/* Pad Mux Registers */

#define IMXRT_PADMUX_ADDRESS(n)          (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_OFFSET(n))
#define IMXRT_PADMUX_ADDRESS_SNVS(n)     (IMXRT_IOMUXCSNVS_BASE + IMXRT_PADMUX_OFFSET_SNVS(n))

#define IMXRT_PADMUX_GPIO_EMC_00         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_00_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_01         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_01_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_02         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_02_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_03         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_03_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_04         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_04_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_05         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_05_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_06         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_06_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_07         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_07_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_08         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_08_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_09         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_09_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_10         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_10_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_11         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_11_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_12         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_12_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_13         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_13_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_14         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_14_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_15         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_15_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_16         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_16_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_17         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_17_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_18         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_18_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_19         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_19_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_20         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_20_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_21         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_21_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_22         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_22_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_23         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_23_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_24         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_24_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_25         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_25_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_26         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_26_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_27         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_27_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_28         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_28_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_29         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_29_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_30         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_30_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_31         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_31_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_32         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_32_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_33         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_33_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_34         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_34_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_35         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_35_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_36         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_36_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_37         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_37_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_38         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_38_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_39         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_39_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_40         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_40_OFFSET)
#define IMXRT_PADMUX_GPIO_EMC_41         (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_EMC_41_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_B0_00       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_B0_00_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_B0_01       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_B0_01_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_B0_02       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_B0_02_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_B0_03       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_B0_03_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_B0_04       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_B0_04_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_B0_05       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_B0_05_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_B0_06       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_B0_06_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_B0_07       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_B0_07_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_B0_08       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_B0_08_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_B0_09       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_B0_09_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_B0_10       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_B0_10_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_B0_11       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_B0_11_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_B0_12       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_B0_12_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_B0_13       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_B0_13_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_B0_14       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_B0_14_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_B0_15       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_B0_15_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_B1_00       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_B1_00_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_B1_01       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_B1_01_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_B1_02       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_B1_02_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_B1_03       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_B1_03_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_B1_04       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_B1_04_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_B1_05       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_B1_05_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_B1_06       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_B1_06_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_B1_07       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_B1_07_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_B1_08       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_B1_08_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_B1_09       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_B1_09_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_B1_10       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_B1_10_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_B1_11       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_B1_11_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_B1_12       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_B1_12_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_B1_13       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_B1_13_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_B1_14       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_B1_14_OFFSET)
#define IMXRT_PADMUX_GPIO_AD_B1_15       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_AD_B1_15_OFFSET)
#define IMXRT_PADMUX_GPIO_B0_00          (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_B0_00_OFFSET)
#define IMXRT_PADMUX_GPIO_B0_01          (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_B0_01_OFFSET)
#define IMXRT_PADMUX_GPIO_B0_02          (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_B0_02_OFFSET)
#define IMXRT_PADMUX_GPIO_B0_03          (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_B0_03_OFFSET)
#define IMXRT_PADMUX_GPIO_B0_04          (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_B0_04_OFFSET)
#define IMXRT_PADMUX_GPIO_B0_05          (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_B0_05_OFFSET)
#define IMXRT_PADMUX_GPIO_B0_06          (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_B0_06_OFFSET)
#define IMXRT_PADMUX_GPIO_B0_07          (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_B0_07_OFFSET)
#define IMXRT_PADMUX_GPIO_B0_08          (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_B0_08_OFFSET)
#define IMXRT_PADMUX_GPIO_B0_09          (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_B0_09_OFFSET)
#define IMXRT_PADMUX_GPIO_B0_10          (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_B0_10_OFFSET)
#define IMXRT_PADMUX_GPIO_B0_11          (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_B0_11_OFFSET)
#define IMXRT_PADMUX_GPIO_B0_12          (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_B0_12_OFFSET)
#define IMXRT_PADMUX_GPIO_B0_13          (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_B0_13_OFFSET)
#define IMXRT_PADMUX_GPIO_B0_14          (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_B0_14_OFFSET)
#define IMXRT_PADMUX_GPIO_B0_15          (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_B0_15_OFFSET)
#define IMXRT_PADMUX_GPIO_B1_00          (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_B1_00_OFFSET)
#define IMXRT_PADMUX_GPIO_B1_01          (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_B1_01_OFFSET)
#define IMXRT_PADMUX_GPIO_B1_02          (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_B1_02_OFFSET)
#define IMXRT_PADMUX_GPIO_B1_03          (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_B1_03_OFFSET)
#define IMXRT_PADMUX_GPIO_B1_04          (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_B1_04_OFFSET)
#define IMXRT_PADMUX_GPIO_B1_05          (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_B1_05_OFFSET)
#define IMXRT_PADMUX_GPIO_B1_06          (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_B1_06_OFFSET)
#define IMXRT_PADMUX_GPIO_B1_07          (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_B1_07_OFFSET)
#define IMXRT_PADMUX_GPIO_B1_08          (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_B1_08_OFFSET)
#define IMXRT_PADMUX_GPIO_B1_09          (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_B1_09_OFFSET)
#define IMXRT_PADMUX_GPIO_B1_10          (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_B1_10_OFFSET)
#define IMXRT_PADMUX_GPIO_B1_11          (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_B1_11_OFFSET)
#define IMXRT_PADMUX_GPIO_B1_12          (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_B1_12_OFFSET)
#define IMXRT_PADMUX_GPIO_B1_13          (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_B1_13_OFFSET)
#define IMXRT_PADMUX_GPIO_B1_14          (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_B1_14_OFFSET)
#define IMXRT_PADMUX_GPIO_B1_15          (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_B1_15_OFFSET)
#define IMXRT_PADMUX_GPIO_SD_B0_00       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_SD_B0_00_OFFSET)
#define IMXRT_PADMUX_GPIO_SD_B0_01       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_SD_B0_01_OFFSET)
#define IMXRT_PADMUX_GPIO_SD_B0_02       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_SD_B0_02_OFFSET)
#define IMXRT_PADMUX_GPIO_SD_B0_03       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_SD_B0_03_OFFSET)
#define IMXRT_PADMUX_GPIO_SD_B0_04       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_SD_B0_04_OFFSET)
#define IMXRT_PADMUX_GPIO_SD_B0_05       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_SD_B0_05_OFFSET)
#define IMXRT_PADMUX_GPIO_SD_B1_00       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_SD_B1_00_OFFSET)
#define IMXRT_PADMUX_GPIO_SD_B1_01       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_SD_B1_01_OFFSET)
#define IMXRT_PADMUX_GPIO_SD_B1_02       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_SD_B1_02_OFFSET)
#define IMXRT_PADMUX_GPIO_SD_B1_03       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_SD_B1_03_OFFSET)
#define IMXRT_PADMUX_GPIO_SD_B1_04       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_SD_B1_04_OFFSET)
#define IMXRT_PADMUX_GPIO_SD_B1_05       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_SD_B1_05_OFFSET)
#define IMXRT_PADMUX_GPIO_SD_B1_06       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_SD_B1_06_OFFSET)
#define IMXRT_PADMUX_GPIO_SD_B1_07       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_SD_B1_07_OFFSET)
#define IMXRT_PADMUX_GPIO_SD_B1_08       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_SD_B1_08_OFFSET)
#define IMXRT_PADMUX_GPIO_SD_B1_09       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_SD_B1_09_OFFSET)
#define IMXRT_PADMUX_GPIO_SD_B1_10       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_SD_B1_10_OFFSET)
#define IMXRT_PADMUX_GPIO_SD_B1_11       (IMXRT_IOMUXC_BASE + IMXRT_PADMUX_GPIO_SD_B1_11_OFFSET)

/* Pad Control Registers */

#define IMXRT_PADCTL_ADDRESS(n)          (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_OFFSET(n))
#define IMXRT_PADCTL_ADDRESS_SNVS(n)     (IMXRT_IOMUXCSNVS_BASE + IMXRT_PADCTL_OFFSET_SNVS(n))

#define IMXRT_PADCTL_GPIO_EMC_00         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_00_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_01         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_01_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_02         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_02_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_03         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_03_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_04         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_04_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_05         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_05_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_06         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_06_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_07         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_07_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_08         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_08_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_09         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_09_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_10         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_10_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_11         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_11_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_12         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_12_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_13         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_13_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_14         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_14_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_15         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_15_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_16         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_16_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_17         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_17_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_18         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_18_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_19         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_19_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_20         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_20_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_21         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_21_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_22         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_22_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_23         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_23_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_24         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_24_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_25         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_25_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_26         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_26_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_27         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_27_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_28         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_28_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_29         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_29_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_30         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_30_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_31         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_31_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_32         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_32_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_33         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_33_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_34         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_34_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_35         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_35_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_36         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_36_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_37         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_37_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_38         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_38_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_39         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_39_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_40         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_40_OFFSET)
#define IMXRT_PADCTL_GPIO_EMC_41         (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_EMC_41_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_B0_00       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_B0_00_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_B0_01       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_B0_01_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_B0_02       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_B0_02_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_B0_03       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_B0_03_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_B0_04       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_B0_04_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_B0_05       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_B0_05_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_B0_06       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_B0_06_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_B0_07       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_B0_07_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_B0_08       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_B0_08_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_B0_09       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_B0_09_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_B0_10       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_B0_10_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_B0_11       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_B0_11_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_B0_12       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_B0_12_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_B0_13       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_B0_13_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_B0_14       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_B0_14_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_B0_15       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_B0_15_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_B1_00       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_B1_00_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_B1_01       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_B1_01_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_B1_02       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_B1_02_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_B1_03       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_B1_03_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_B1_04       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_B1_04_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_B1_05       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_B1_05_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_B1_06       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_B1_06_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_B1_07       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_B1_07_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_B1_08       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_B1_08_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_B1_09       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_B1_09_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_B1_10       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_B1_10_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_B1_11       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_B1_11_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_B1_12       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_B1_12_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_B1_13       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_B1_13_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_B1_14       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_B1_14_OFFSET)
#define IMXRT_PADCTL_GPIO_AD_B1_15       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_AD_B1_15_OFFSET)
#define IMXRT_PADCTL_GPIO_B0_00          (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_B0_00_OFFSET)
#define IMXRT_PADCTL_GPIO_B0_01          (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_B0_01_OFFSET)
#define IMXRT_PADCTL_GPIO_B0_02          (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_B0_02_OFFSET)
#define IMXRT_PADCTL_GPIO_B0_03          (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_B0_03_OFFSET)
#define IMXRT_PADCTL_GPIO_B0_04          (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_B0_04_OFFSET)
#define IMXRT_PADCTL_GPIO_B0_05          (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_B0_05_OFFSET)
#define IMXRT_PADCTL_GPIO_B0_06          (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_B0_06_OFFSET)
#define IMXRT_PADCTL_GPIO_B0_07          (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_B0_07_OFFSET)
#define IMXRT_PADCTL_GPIO_B0_08          (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_B0_08_OFFSET)
#define IMXRT_PADCTL_GPIO_B0_09          (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_B0_09_OFFSET)
#define IMXRT_PADCTL_GPIO_B0_10          (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_B0_10_OFFSET)
#define IMXRT_PADCTL_GPIO_B0_11          (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_B0_11_OFFSET)
#define IMXRT_PADCTL_GPIO_B0_12          (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_B0_12_OFFSET)
#define IMXRT_PADCTL_GPIO_B0_13          (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_B0_13_OFFSET)
#define IMXRT_PADCTL_GPIO_B0_14          (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_B0_14_OFFSET)
#define IMXRT_PADCTL_GPIO_B0_15          (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_B0_15_OFFSET)
#define IMXRT_PADCTL_GPIO_B1_00          (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_B1_00_OFFSET)
#define IMXRT_PADCTL_GPIO_B1_01          (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_B1_01_OFFSET)
#define IMXRT_PADCTL_GPIO_B1_02          (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_B1_02_OFFSET)
#define IMXRT_PADCTL_GPIO_B1_03          (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_B1_03_OFFSET)
#define IMXRT_PADCTL_GPIO_B1_04          (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_B1_04_OFFSET)
#define IMXRT_PADCTL_GPIO_B1_05          (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_B1_05_OFFSET)
#define IMXRT_PADCTL_GPIO_B1_06          (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_B1_06_OFFSET)
#define IMXRT_PADCTL_GPIO_B1_07          (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_B1_07_OFFSET)
#define IMXRT_PADCTL_GPIO_B1_08          (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_B1_08_OFFSET)
#define IMXRT_PADCTL_GPIO_B1_09          (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_B1_09_OFFSET)
#define IMXRT_PADCTL_GPIO_B1_10          (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_B1_10_OFFSET)
#define IMXRT_PADCTL_GPIO_B1_11          (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_B1_11_OFFSET)
#define IMXRT_PADCTL_GPIO_B1_12          (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_B1_12_OFFSET)
#define IMXRT_PADCTL_GPIO_B1_13          (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_B1_13_OFFSET)
#define IMXRT_PADCTL_GPIO_B1_14          (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_B1_14_OFFSET)
#define IMXRT_PADCTL_GPIO_B1_15          (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_B1_15_OFFSET)
#define IMXRT_PADCTL_GPIO_SD_B0_00       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_SD_B0_00_OFFSET)
#define IMXRT_PADCTL_GPIO_SD_B0_01       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_SD_B0_01_OFFSET)
#define IMXRT_PADCTL_GPIO_SD_B0_02       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_SD_B0_02_OFFSET)
#define IMXRT_PADCTL_GPIO_SD_B0_03       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_SD_B0_03_OFFSET)
#define IMXRT_PADCTL_GPIO_SD_B0_04       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_SD_B0_04_OFFSET)
#define IMXRT_PADCTL_GPIO_SD_B0_05       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_SD_B0_05_OFFSET)
#define IMXRT_PADCTL_GPIO_SD_B1_00       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_SD_B1_00_OFFSET)
#define IMXRT_PADCTL_GPIO_SD_B1_01       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_SD_B1_01_OFFSET)
#define IMXRT_PADCTL_GPIO_SD_B1_02       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_SD_B1_02_OFFSET)
#define IMXRT_PADCTL_GPIO_SD_B1_03       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_SD_B1_03_OFFSET)
#define IMXRT_PADCTL_GPIO_SD_B1_04       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_SD_B1_04_OFFSET)
#define IMXRT_PADCTL_GPIO_SD_B1_05       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_SD_B1_05_OFFSET)
#define IMXRT_PADCTL_GPIO_SD_B1_06       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_SD_B1_06_OFFSET)
#define IMXRT_PADCTL_GPIO_SD_B1_07       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_SD_B1_07_OFFSET)
#define IMXRT_PADCTL_GPIO_SD_B1_08       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_SD_B1_08_OFFSET)
#define IMXRT_PADCTL_GPIO_SD_B1_09       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_SD_B1_09_OFFSET)
#define IMXRT_PADCTL_GPIO_SD_B1_10       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_SD_B1_10_OFFSET)
#define IMXRT_PADCTL_GPIO_SD_B1_11       (IMXRT_IOMUXC_BASE + IMXRT_PADCTL_GPIO_SD_B1_11_OFFSET)

/* Select Input Registers */

#define IMXRT_INPUT_ANATOP_USB_OTG1_ID   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_ANATOP_USB_OTG1_ID_OFFSET)
#define IMXRT_INPUT_ANATOP_USB_OTG2_ID   (IMXRT_IOMUXC_BASE + IMXRT_INPUT_ANATOP_USB_OTG2_ID_OFFSET)
#define IMXRT_INPUT_CCM_PMIC_READY       (IMXRT_IOMUXC_BASE + IMXRT_INPUT_CCM_PMIC_READY_OFFSET)
#define IMXRT_INPUT_CSI_DATA02           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_CSI_DATA02_OFFSET)
#define IMXRT_INPUT_CSI_DATA03           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_CSI_DATA03_OFFSET)
#define IMXRT_INPUT_CSI_DATA04           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_CSI_DATA04_OFFSET)
#define IMXRT_INPUT_CSI_DATA05           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_CSI_DATA05_OFFSET)
#define IMXRT_INPUT_CSI_DATA06           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_CSI_DATA06_OFFSET)
#define IMXRT_INPUT_CSI_DATA07           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_CSI_DATA07_OFFSET)
#define IMXRT_INPUT_CSI_DATA08           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_CSI_DATA08_OFFSET)
#define IMXRT_INPUT_CSI_DATA09           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_CSI_DATA09_OFFSET)
#define IMXRT_INPUT_CSI_HSYNC            (IMXRT_IOMUXC_BASE + IMXRT_INPUT_CSI_HSYNC_OFFSET)
#define IMXRT_INPUT_CSI_PIXCLK           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_CSI_PIXCLK_OFFSET)
#define IMXRT_INPUT_CSI_VSYNC            (IMXRT_IOMUXC_BASE + IMXRT_INPUT_CSI_VSYNC_OFFSET)
#define IMXRT_INPUT_ENET_IPG_CLK_RMII    (IMXRT_IOMUXC_BASE + IMXRT_INPUT_ENET_IPG_CLK_RMII_OFFSET)
#define IMXRT_INPUT_ENET_MDIO            (IMXRT_IOMUXC_BASE + IMXRT_INPUT_ENET_MDIO_OFFSET)
#define IMXRT_INPUT_ENET0_RXDATA         (IMXRT_IOMUXC_BASE + IMXRT_INPUT_ENET0_RXDATA_OFFSET)
#define IMXRT_INPUT_ENET1_RXDATA         (IMXRT_IOMUXC_BASE + IMXRT_INPUT_ENET1_RXDATA_OFFSET)
#define IMXRT_INPUT_ENET_RXEN            (IMXRT_IOMUXC_BASE + IMXRT_INPUT_ENET_RXEN_OFFSET)
#define IMXRT_INPUT_ENET_RXERR           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_ENET_RXERR_OFFSET)
#define IMXRT_INPUT_ENET0_TIMER          (IMXRT_IOMUXC_BASE + IMXRT_INPUT_ENET0_TIMER_OFFSET)
#define IMXRT_INPUT_ENET_TXCLK           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_ENET_TXCLK_OFFSET)
#define IMXRT_INPUT_FLEXCAN1_RX          (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXCAN1_RX_OFFSET)
#define IMXRT_INPUT_FLEXCAN2_RX          (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXCAN2_RX_OFFSET)
#define IMXRT_INPUT_FLEXPWM1_PWMA3       (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXPWM1_PWMA3_OFFSET)
#define IMXRT_INPUT_FLEXPWM1_PWMA0       (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXPWM1_PWMA0_OFFSET)
#define IMXRT_INPUT_FLEXPWM1_PWMA1       (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXPWM1_PWMA1_OFFSET)
#define IMXRT_INPUT_FLEXPWM1_PWMA2       (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXPWM1_PWMA2_OFFSET)
#define IMXRT_INPUT_FLEXPWM1_PWMB3       (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXPWM1_PWMB3_OFFSET)
#define IMXRT_INPUT_FLEXPWM1_PWMB0       (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXPWM1_PWMB0_OFFSET)
#define IMXRT_INPUT_FLEXPWM1_PWMB1       (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXPWM1_PWMB1_OFFSET)
#define IMXRT_INPUT_FLEXPWM1_PWMB2       (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXPWM1_PWMB2_OFFSET)
#define IMXRT_INPUT_FLEXPWM2_PWMA3       (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXPWM2_PWMA3_OFFSET)
#define IMXRT_INPUT_FLEXPWM2_PWMA0       (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXPWM2_PWMA0_OFFSET)
#define IMXRT_INPUT_FLEXPWM2_PWMA1       (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXPWM2_PWMA1_OFFSET)
#define IMXRT_INPUT_FLEXPWM2_PWMA2       (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXPWM2_PWMA2_OFFSET)
#define IMXRT_INPUT_FLEXPWM2_PWMB3       (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXPWM2_PWMB3_OFFSET)
#define IMXRT_INPUT_FLEXPWM2_PWMB0       (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXPWM2_PWMB0_OFFSET)
#define IMXRT_INPUT_FLEXPWM2_PWMB1       (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXPWM2_PWMB1_OFFSET)
#define IMXRT_INPUT_FLEXPWM2_PWMB2       (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXPWM2_PWMB2_OFFSET)
#define IMXRT_INPUT_FLEXPWM4_PWMA0       (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXPWM4_PWMA0_OFFSET)
#define IMXRT_INPUT_FLEXPWM4_PWMA1       (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXPWM4_PWMA1_OFFSET)
#define IMXRT_INPUT_FLEXPWM4_PWMA2       (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXPWM4_PWMA2_OFFSET)
#define IMXRT_INPUT_FLEXPWM4_PWMA3       (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXPWM4_PWMA3_OFFSET)
#define IMXRT_INPUT_FLEXSPIA_DQS         (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXSPIA_DQS_OFFSET)
#define IMXRT_INPUT_FLEXSPIA_DATA0       (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXSPIA_DATA0_OFFSET)
#define IMXRT_INPUT_FLEXSPIA_DATA1       (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXSPIA_DATA1_OFFSET)
#define IMXRT_INPUT_FLEXSPIA_DATA2       (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXSPIA_DATA2_OFFSET)
#define IMXRT_INPUT_FLEXSPIA_DATA3       (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXSPIA_DATA3_OFFSET)
#define IMXRT_INPUT_FLEXSPIB_DATA0       (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXSPIB_DATA0_OFFSET)
#define IMXRT_INPUT_FLEXSPIB_DATA1       (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXSPIB_DATA1_OFFSET)
#define IMXRT_INPUT_FLEXSPIB_DATA2       (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXSPIB_DATA2_OFFSET)
#define IMXRT_INPUT_FLEXSPIB_DATA3       (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXSPIB_DATA3_OFFSET)
#define IMXRT_INPUT_FLEXSPIA_SCK         (IMXRT_IOMUXC_BASE + IMXRT_INPUT_FLEXSPIA_SCK_OFFSET)
#define IMXRT_INPUT_LPI2C1_SCL           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPI2C1_SCL_OFFSET)
#define IMXRT_INPUT_LPI2C1_SDA           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPI2C1_SDA_OFFSET)
#define IMXRT_INPUT_LPI2C2_SCL           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPI2C2_SCL_OFFSET)
#define IMXRT_INPUT_LPI2C2_SDA           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPI2C2_SDA_OFFSET)
#define IMXRT_INPUT_LPI2C3_SCL           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPI2C3_SCL_OFFSET)
#define IMXRT_INPUT_LPI2C3_SDA           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPI2C3_SDA_OFFSET)
#define IMXRT_INPUT_LPI2C4_SCL           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPI2C4_SCL_OFFSET)
#define IMXRT_INPUT_LPI2C4_SDA           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPI2C4_SDA_OFFSET)
#define IMXRT_INPUT_LPSPI1_PCS0          (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPSPI1_PCS0_OFFSET)
#define IMXRT_INPUT_LPSPI1_SCK           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPSPI1_SCK_OFFSET)
#define IMXRT_INPUT_LPSPI1_SDI           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPSPI1_SDI_OFFSET)
#define IMXRT_INPUT_LPSPI1_SDO           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPSPI1_SDO_OFFSET)
#define IMXRT_INPUT_LPSPI2_PCS0          (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPSPI2_PCS0_OFFSET)
#define IMXRT_INPUT_LPSPI2_SCK           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPSPI2_SCK_OFFSET)
#define IMXRT_INPUT_LPSPI2_SDI           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPSPI2_SDI_OFFSET)
#define IMXRT_INPUT_LPSPI2_SDO           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPSPI2_SDO_OFFSET)
#define IMXRT_INPUT_LPSPI3_PCS0          (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPSPI3_PCS0_OFFSET)
#define IMXRT_INPUT_LPSPI3_SCK           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPSPI3_SCK_OFFSET)
#define IMXRT_INPUT_LPSPI3_SDI           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPSPI3_SDI_OFFSET)
#define IMXRT_INPUT_LPSPI3_SDO           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPSPI3_SDO_OFFSET)
#define IMXRT_INPUT_LPSPI4_PCS0          (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPSPI4_PCS0_OFFSET)
#define IMXRT_INPUT_LPSPI4_SCK           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPSPI4_SCK_OFFSET)
#define IMXRT_INPUT_LPSPI4_SDI           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPSPI4_SDI_OFFSET)
#define IMXRT_INPUT_LPSPI4_SDO           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPSPI4_SDO_OFFSET)
#define IMXRT_INPUT_LPUART2_RX           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPUART2_RX_OFFSET)
#define IMXRT_INPUT_LPUART2_TX           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPUART2_TX_OFFSET)
#define IMXRT_INPUT_LPUART3_CTS_B        (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPUART3_CTS_B_OFFSET)
#define IMXRT_INPUT_LPUART3_RX           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPUART3_RX_OFFSET)
#define IMXRT_INPUT_LPUART3_TX           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPUART3_TX_OFFSET)
#define IMXRT_INPUT_LPUART4_RX           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPUART4_RX_OFFSET)
#define IMXRT_INPUT_LPUART4_TX           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPUART4_TX_OFFSET)
#define IMXRT_INPUT_LPUART5_RX           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPUART5_RX_OFFSET)
#define IMXRT_INPUT_LPUART5_TX           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPUART5_TX_OFFSET)
#define IMXRT_INPUT_LPUART6_RX           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPUART6_RX_OFFSET)
#define IMXRT_INPUT_LPUART6_TX           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPUART6_TX_OFFSET)
#define IMXRT_INPUT_LPUART7_RX           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPUART7_RX_OFFSET)
#define IMXRT_INPUT_LPUART7_TX           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPUART7_TX_OFFSET)
#define IMXRT_INPUT_LPUART8_RX           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPUART8_RX_OFFSET)
#define IMXRT_INPUT_LPUART8_TX           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_LPUART8_TX_OFFSET)
#define IMXRT_INPUT_NMI_GLUE_NMI         (IMXRT_IOMUXC_BASE + IMXRT_INPUT_NMI_GLUE_NMI_OFFSET)
#define IMXRT_INPUT_QTIMER2_TIMER0       (IMXRT_IOMUXC_BASE + IMXRT_INPUT_QTIMER2_TIMER0_OFFSET)
#define IMXRT_INPUT_QTIMER2_TIMER1       (IMXRT_IOMUXC_BASE + IMXRT_INPUT_QTIMER2_TIMER1_OFFSET)
#define IMXRT_INPUT_QTIMER2_TIMER2       (IMXRT_IOMUXC_BASE + IMXRT_INPUT_QTIMER2_TIMER2_OFFSET)
#define IMXRT_INPUT_QTIMER2_TIMER3       (IMXRT_IOMUXC_BASE + IMXRT_INPUT_QTIMER2_TIMER3_OFFSET)
#define IMXRT_INPUT_QTIMER3_TIMER0       (IMXRT_IOMUXC_BASE + IMXRT_INPUT_QTIMER3_TIMER0_OFFSET)
#define IMXRT_INPUT_QTIMER3_TIMER1       (IMXRT_IOMUXC_BASE + IMXRT_INPUT_QTIMER3_TIMER1_OFFSET)
#define IMXRT_INPUT_QTIMER3_TIMER2       (IMXRT_IOMUXC_BASE + IMXRT_INPUT_QTIMER3_TIMER2_OFFSET)
#define IMXRT_INPUT_QTIMER3_TIMER3       (IMXRT_IOMUXC_BASE + IMXRT_INPUT_QTIMER3_TIMER3_OFFSET)
#define IMXRT_INPUT_SAI1_MCLK2           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_SAI1_MCLK2_OFFSET)
#define IMXRT_INPUT_SAI1_RX_BCLK         (IMXRT_IOMUXC_BASE + IMXRT_INPUT_SAI1_RX_BCLK_OFFSET)
#define IMXRT_INPUT_SAI1_RX_DATA0        (IMXRT_IOMUXC_BASE + IMXRT_INPUT_SAI1_RX_DATA0_OFFSET)
#define IMXRT_INPUT_SAI1_RX_DATA1        (IMXRT_IOMUXC_BASE + IMXRT_INPUT_SAI1_RX_DATA1_OFFSET)
#define IMXRT_INPUT_SAI1_RX_DATA2        (IMXRT_IOMUXC_BASE + IMXRT_INPUT_SAI1_RX_DATA2_OFFSET)
#define IMXRT_INPUT_SAI1_RX_DATA3        (IMXRT_IOMUXC_BASE + IMXRT_INPUT_SAI1_RX_DATA3_OFFSET)
#define IMXRT_INPUT_SAI1_RX_SYNC         (IMXRT_IOMUXC_BASE + IMXRT_INPUT_SAI1_RX_SYNC_OFFSET)
#define IMXRT_INPUT_SAI1_TX_BCLK         (IMXRT_IOMUXC_BASE + IMXRT_INPUT_SAI1_TX_BCLK_OFFSET)
#define IMXRT_INPUT_SAI1_TX_SYNC         (IMXRT_IOMUXC_BASE + IMXRT_INPUT_SAI1_TX_SYNC_OFFSET)
#define IMXRT_INPUT_SAI2_MCLK2           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_SAI2_MCLK2_OFFSET)
#define IMXRT_INPUT_SAI2_RX_BCLK         (IMXRT_IOMUXC_BASE + IMXRT_INPUT_SAI2_RX_BCLK_OFFSET)
#define IMXRT_INPUT_SAI2_RX_DATA0        (IMXRT_IOMUXC_BASE + IMXRT_INPUT_SAI2_RX_DATA0_OFFSET)
#define IMXRT_INPUT_SAI2_RX_SYNC         (IMXRT_IOMUXC_BASE + IMXRT_INPUT_SAI2_RX_SYNC_OFFSET)
#define IMXRT_INPUT_SAI2_TX_BCLK         (IMXRT_IOMUXC_BASE + IMXRT_INPUT_SAI2_TX_BCLK_OFFSET)
#define IMXRT_INPUT_SAI2_TX_SYNC         (IMXRT_IOMUXC_BASE + IMXRT_INPUT_SAI2_TX_SYNC_OFFSET)
#define IMXRT_INPUT_SPDIF_IN             (IMXRT_IOMUXC_BASE + IMXRT_INPUT_SPDIF_IN_OFFSET)
#define IMXRT_INPUT_USB_OTG2_OC          (IMXRT_IOMUXC_BASE + IMXRT_INPUT_USB_OTG2_OC_OFFSET)
#define IMXRT_INPUT_USB_OTG1_OC          (IMXRT_IOMUXC_BASE + IMXRT_INPUT_USB_OTG1_OC_OFFSET)
#define IMXRT_INPUT_USDHC1_CD_B          (IMXRT_IOMUXC_BASE + IMXRT_INPUT_USDHC1_CD_B_OFFSET)
#define IMXRT_INPUT_USDHC1_WP            (IMXRT_IOMUXC_BASE + IMXRT_INPUT_USDHC1_WP_OFFSET)
#define IMXRT_INPUT_USDHC2_CLK           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_USDHC2_CLK_OFFSET)
#define IMXRT_INPUT_USDHC2_CD_B          (IMXRT_IOMUXC_BASE + IMXRT_INPUT_USDHC2_CD_B_OFFSET)
#define IMXRT_INPUT_USDHC2_CMD           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_USDHC2_CMD_OFFSET)
#define IMXRT_INPUT_USDHC2_DATA0         (IMXRT_IOMUXC_BASE + IMXRT_INPUT_USDHC2_DATA0_OFFSET)
#define IMXRT_INPUT_USDHC2_DATA1         (IMXRT_IOMUXC_BASE + IMXRT_INPUT_USDHC2_DATA1_OFFSET)
#define IMXRT_INPUT_USDHC2_DATA2         (IMXRT_IOMUXC_BASE + IMXRT_INPUT_USDHC2_DATA2_OFFSET)
#define IMXRT_INPUT_USDHC2_DATA3         (IMXRT_IOMUXC_BASE + IMXRT_INPUT_USDHC2_DATA3_OFFSET)
#define IMXRT_INPUT_USDHC2_DATA4         (IMXRT_IOMUXC_BASE + IMXRT_INPUT_USDHC2_DATA4_OFFSET)
#define IMXRT_INPUT_USDHC2_DATA5         (IMXRT_IOMUXC_BASE + IMXRT_INPUT_USDHC2_DATA5_OFFSET)
#define IMXRT_INPUT_USDHC2_DATA6         (IMXRT_IOMUXC_BASE + IMXRT_INPUT_USDHC2_DATA6_OFFSET)
#define IMXRT_INPUT_USDHC2_DATA7         (IMXRT_IOMUXC_BASE + IMXRT_INPUT_USDHC2_DATA7_OFFSET)
#define IMXRT_INPUT_USDHC2_WP            (IMXRT_IOMUXC_BASE + IMXRT_INPUT_USDHC2_WP_OFFSET)
#define IMXRT_INPUT_XBAR1_IN02           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_XBAR1_IN02_OFFSET)
#define IMXRT_INPUT_XBAR1_IN03           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_XBAR1_IN03_OFFSET)
#define IMXRT_INPUT_XBAR1_IN04           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_XBAR1_IN04_OFFSET)
#define IMXRT_INPUT_XBAR1_IN05           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_XBAR1_IN05_OFFSET)
#define IMXRT_INPUT_XBAR1_IN06           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_XBAR1_IN05_OFFSET)
#define IMXRT_INPUT_XBAR1_IN07           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_XBAR1_IN07_OFFSET)
#define IMXRT_INPUT_XBAR1_IN08           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_XBAR1_IN08_OFFSET)
#define IMXRT_INPUT_XBAR1_IN09           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_XBAR1_IN09_OFFSET)
#define IMXRT_INPUT_XBAR1_IN17           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_XBAR1_IN17_OFFSET)
#define IMXRT_INPUT_XBAR1_IN18           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_XBAR1_IN18_OFFSET)
#define IMXRT_INPUT_XBAR1_IN20           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_XBAR1_IN20_OFFSET)
#define IMXRT_INPUT_XBAR1_IN22           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_XBAR1_IN22_OFFSET)
#define IMXRT_INPUT_XBAR1_IN23           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_XBAR1_IN23_OFFSET)
#define IMXRT_INPUT_XBAR1_IN24           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_XBAR1_IN24_OFFSET)
#define IMXRT_INPUT_XBAR1_IN14           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_XBAR1_IN14_OFFSET)
#define IMXRT_INPUT_XBAR1_IN15           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_XBAR1_IN15_OFFSET)
#define IMXRT_INPUT_XBAR1_IN16           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_XBAR1_IN16_OFFSET)
#define IMXRT_INPUT_XBAR1_IN25           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_XBAR1_IN25_OFFSET)
#define IMXRT_INPUT_XBAR1_IN19           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_XBAR1_IN19_OFFSET)
#define IMXRT_INPUT_XBAR1_IN21           (IMXRT_IOMUXC_BASE + IMXRT_INPUT_XBAR1_IN21_OFFSET)

/* Register bit definitions *********************************************************/

/* General Purpose Register 1 (GPR1) */

#define GPR_GPR1_SAI1_MCLK1_SEL_SHIFT                       (0)
#define GPR_GPR1_SAI1_MCLK1_SEL_MASK                        (7 << GPR_GPR1_SAI1_MCLK1_SEL_SHIFT)
#   define   GPR_GPR1_SAI1_MCLK1_SEL_CCM_SSI1_CLK_ROOT             (0 << GPR_GPR1_SAI1_MCLK1_SEL_SHIFT)
#   define   GPR_GPR1_SAI1_MCLK1_SEL_CCM_SSI2_CLK_ROOT             (1 << GPR_GPR1_SAI1_MCLK1_SEL_SHIFT)
#   define   GPR_GPR1_SAI1_MCLK1_SEL_CCM_SSI3_CLK_ROOT             (2 << GPR_GPR1_SAI1_MCLK1_SEL_SHIFT)
#   define   GPR_GPR1_SAI1_MCLK1_SEL_IOMUX_SAI1_IPG_CLK_SAI_MCLK2  (3 << GPR_GPR1_SAI1_MCLK1_SEL_SHIFT)
#   define   GPR_GPR1_SAI1_MCLK1_SEL_IOMUX_SAI2_IPG_CLK_SAI_MCLK2  (4 << GPR_GPR1_SAI1_MCLK1_SEL_SHIFT)
#   define   GPR_GPR1_SAI1_MCLK1_SEL_IOMUX_SAI3_IPG_CLK_SAI_MCLK2  (5 << GPR_GPR1_SAI1_MCLK1_SEL_SHIFT)
#define GPR_GPR1_SAI1_MCLK2_SEL_SHIFT                       (3)
#define GPR_GPR1_SAI1_MCLK2_SEL_MASK                        (7 << GPR_GPR1_SAI1_MCLK2_SEL_SHIFT)
#   define   GPR_GPR1_SAI1_MCLK2_SEL_CCM_SSI1_CLK_ROOT             (0 << GPR_GPR1_SAI1_MCLK2_SEL_SHIFT)
#   define   GPR_GPR1_SAI1_MCLK2_SEL_CCM_SSI2_CLK_ROOT             (1 << GPR_GPR1_SAI1_MCLK2_SEL_SHIFT)
#   define   GPR_GPR1_SAI1_MCLK2_SEL_CCM_SSI3_CLK_ROOT             (2 << GPR_GPR1_SAI1_MCLK2_SEL_SHIFT)
#   define   GPR_GPR1_SAI1_MCLK2_SEL_IOMUX_SAI1_IPG_CLK_SAI_MCLK2  (3 << GPR_GPR1_SAI1_MCLK2_SEL_SHIFT)
#   define   GPR_GPR1_SAI1_MCLK2_SEL_IOMUX_SAI2_IPG_CLK_SAI_MCLK2  (4 << GPR_GPR1_SAI1_MCLK2_SEL_SHIFT)
#   define   GPR_GPR1_SAI1_MCLK2_SEL_IOMUX_SAI3_IPG_CLK_SAI_MCLK2  (5 << GPR_GPR1_SAI1_MCLK2_SEL_SHIFT)
#define GPR_GPR1_SAI1_MCLK3_SEL_SHIFT                       (6)
#define GPR_GPR1_SAI1_MCLK3_SEL_MASK                        (3 << GPR_GPR1_SAI1_MCLK3_SEL_SHIFT)
#   define   GPR_GPR1_SAI1_MCLK3_SEL_CCM_SPDIF0_CLK_ROOT           (0 << GPR_GPR1_SAI1_MCLK3_SEL_SHIFT)
#   define   GPR_GPR1_SAI1_MCLK3_SEL_IOMUX_SPDIF_TX_CLK2           (1 << GPR_GPR1_SAI1_MCLK3_SEL_SHIFT)
#   define   GPR_GPR1_SAI1_MCLK3_SEL_SPDIF_SPDIF_SRCLK             (2 << GPR_GPR1_SAI1_MCLK3_SEL_SHIFT)
#   define   GPR_GPR1_SAI1_MCLK3_SEL_SPDIF_SPDIF_OUTCLOCK          (3 << GPR_GPR1_SAI1_MCLK3_SEL_SHIFT)
#define GPR_GPR1_SAI2_MCLK3_SEL_SHIFT                       (8)
#define GPR_GPR1_SAI2_MCLK3_SEL_MASK                        (3 << GPR_GPR1_SAI2_MCLK3_SEL_SHIFT)
#   define   GPR_GPR1_SAI2_MCLK3_SEL_CCM_SPDIF0_CLK_ROOT           (0 << GPR_GPR1_SAI2_MCLK3_SEL_SHIFT)
#   define   GPR_GPR1_SAI2_MCLK3_SEL_IOMUX_SPDIF_TX_CLK2           (1 << GPR_GPR1_SAI2_MCLK3_SEL_SHIFT)
#   define   GPR_GPR1_SAI2_MCLK3_SEL_SPDIF_SPDIF_SRCLK             (2 << GPR_GPR1_SAI2_MCLK3_SEL_SHIFT)
#   define   GPR_GPR1_SAI2_MCLK3_SEL_SPDIF_SPDIF_OUTCLOCK          (3 << GPR_GPR1_SAI2_MCLK3_SEL_SHIFT)
#define GPR_GPR1_SAI3_MCLK3_SEL_SHIFT                       (10)
#define GPR_GPR1_SAI3_MCLK3_SEL_MASK                        (3 << GPR_GPR1_SAI3_MCLK3_SEL_SHIFT)
#   define   GPR_GPR1_SAI3_MCLK3_SEL_CCM_SPDIF0_CLK_ROOT           (0 << GPR_GPR1_SAI3_MCLK3_SEL_SHIFT)
#   define   GPR_GPR1_SAI3_MCLK3_SEL_IOMUX_SPDIF_TX_CLK2           (1 << GPR_GPR1_SAI3_MCLK3_SEL_SHIFT)
#   define   GPR_GPR1_SAI3_MCLK3_SEL_SPDIF_SPDIF_SRCLK             (2 << GPR_GPR1_SAI3_MCLK3_SEL_SHIFT)
#   define   GPR_GPR1_SAI3_MCLK3_SEL_SPDIF_SPDIF_OUTCLOCK          (3 << GPR_GPR1_SAI3_MCLK3_SEL_SHIFT)
#define GPR_GPR1_GINT                                       (1 << 12)
#define GPR_GPR1_ENET1_CLK_SEL                              (1 << 13)
#define GPR_GPR1_USB_EXP_MODE_EN                            (1 << 15)
#define GPR_GPR1_ENET1_TX_CLK_OUT_EN                        (1 << 17)
#define GPR_GPR1_SAI1_MCLK_DIR_IN                           (0 << 19)
#define GPR_GPR1_SAI1_MCLK_DIR_OUT                          (1 << 19)
#define GPR_GPR1_SAI2_MCLK_DIR_IN                           (0 << 20)
#define GPR_GPR1_SAI2_MCLK_DIR_OUT                          (1 << 20)
#define GPR_GPR1_SAI3_MCLK_DIR_IN                           (0 << 21)
#define GPR_GPR1_SAI3_MCLK_DIR_OUT                          (1 << 21)
#define GPR_GPR1_EXC_MON_OKAY                               (0 << 22)
#define GPR_GPR1_EXC_MON_SLVERR                             (1 << 22)
#define GPR_GPR1_ENET_IMG_CLS_S_EN                          (1 << 23)
#define GPR_GPR1_CM7_FORCE_HCLK_EN                          (1 << 31)

/* General Purpose Register 2 (GPR2) */

#define GPR_GPR2_L2_MEM_POWERSAVE_EN                 (1 << 12)
#define GPR_GPR2_L2_MEM_FORCE_DEEPSLEEP              (1 << 14)
#define GPR_GPR2_MQS_CLK_DIV_SHIFT                   (16)
#define GPR_GPR2_MQS_CLK_DIV_MASK                    (255 << GPR_GPR2_MQS_CLK_DIV_SHIFT)
#   define GPR_GPR2_MQS_CLK_DIV(n)                   ((n - 1) << GPR_GPR2_MQS_CLK_DIV_SHIFT)
#define GPR_GPR2_MQS_SW_RST_EN                       (1 << 24)
#define GPR_GPR2_MQS_EN                              (1 << 25)
#define GPR_GPR2_MQS_OVERSAMPLE32                          (0 << 26)
#define GPR_GPR2_MQS_OVERSAMPLE64                    (1 << 26)
#define GPR_GPR2_QTIM1_TMR_RESET                     (1 << 28)
#define GPR_GPR2_QTIM2_TMR_RESET                     (1 << 29)
#define GPR_GPR2_QTIM3_TMR_RESET                     (1 << 30)
#define GPR_GPR2_QTIM4_TMR_RESET                     (1 << 31)

/* General Purpose Register 3 (GPR3) */

#define GPR_GPR3_OCRAM_CTL_SHIFT                     (0)
#define GPR_GPR3_OCRAM_CTL_MASK                        (15 << GPR_GPR3_OCRAM_CTL_SHIFT)
#   define GPR_GPR3_OCRAM_CTL_READ_DATA_PIPELINE_EN       (1 << GPR_GPR3_OCRAM_CTL_SHIFT)
#   define GPR_GPR3_OCRAM_CTL_READ_ADDR_PIPELINE_EN       (2 << GPR_GPR3_OCRAM_CTL_SHIFT)
#   define GPR_GPR3_OCRAM_CTL_WRITE_DATA_PIPELINE_EN      (4 << GPR_GPR3_OCRAM_CTL_SHIFT)
#   define GPR_GPR3_OCRAM_CTL_WRITE_ADDR_PIPELINE_EN      (8 << GPR_GPR3_OCRAM_CTL_SHIFT)
#define GPR_GPR3_DCP_KEY_SEL_128                     (0 << 4)
#define GPR_GPR3_DCP_KEY_SEL_256                     (1 << 4)

/* General Purpose Register 4 (GPR4) */
#define GPR_GRP4_EDMA_STOP_REQ                       (1 << 0)
#define GPR_GPR4_CAN1_STOP_REQ                       (1 << 1)
#define GPR_GPR4_CAN2_STOP_REQ                       (1 << 2)
#define GPR_GPR4_TRNG_STOP_REQ                       (1 << 3)
#define GPR_GPR4_ENET_STOP_REQ                       (1 << 4)
#define GPR_GPR4_SAI1_STOP_REQ                       (1 << 5)
#define GPR_GPR4_SAI2_STOP_REQ                       (1 << 6)
#define GPR_GPR4_SAI3_STOP_REQ                       (1 << 7)
#define GPR_GPR4_SEMC_STOP_REQ                       (1 << 9)
#define GPR_GPR4_PIT_STOP_REQ                        (1 << 10)
#define GPR_GPR4_FLEXSPI_STOP_REQ                    (1 << 11)
#define GPR_GPR4_FLEXIO1_STOP_REQ                    (1 << 12)
#define GPR_GPR4_FLEXIO2_STOP_REQ                    (1 << 13)

/* General Purpose Register 5 (GPR5) */

#define GPR_GPR5_WDOG1_MASK                          (1 << 6)
#define GPR_GPR5_WDOG2_MASK                          (1 << 7)
#define GPR_GPR5_GPT2_CAPIN1_SEL_PAD                 (0 << 23)
#define GPR_GPR5_GPT2_CAPIN1_SEL_ENET1               (1 << 23)
#define GPR_GPR5_GPT2_CAPIN2_SEL_PAD                 (0 << 24)
#define GPR_GPR5_GPT2_CAPIN2_SEL_ENET2               (1 << 24)
#define GPR_GPR5_ENET_EVENT3IN_SEL_PAD               (0 << 25)
#define GPR_GPR5_ENET_EVENT3IN_SEL_ENET2             (1 << 25)
#define GPR_GPR5_VREF_1M_CLK_GPT1_IPG_PERCLK         (0 << 28)
#define GPR_GPR5_VREF_1M_CLK_GPT1_ANATOP             (1 << 28)
#define GPR_GPR5_VREF_1M_CLK_GPT2_IPG_PERCLK         (0 << 29)
#define GPR_GPR5_VREF_1M_CLK_GPT2_ANATOP             (1 << 29)

/* Pad Mux Registers */

#define PADMUX_MUXMODE_SHIFT                  (0)       /* Bit 0-2: Software Input On Field */
#define PADMUX_MUXMODE_MASK                   (7 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT0                 (0 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT1                 (1 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT2                 (2 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT3                 (3 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT4                 (4 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT5                 (5 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT7                 (7 << PADMUX_MUXMODE_SHIFT)
#define PADMUX_SION_SHIFT                     (4)       /* Bit 4: Software Input On Field */
#  define PADMUX_SION                         (1 << PADMUX_SION_SHIFT)

/* Pad Control Registers */

#define DRIVE_HIZ                             (0)       /* HI-Z */
#define DRIVE_260OHM                          (1)       /* 150 Ohm @3.3V, 260 Ohm @1.8V */
#define DRIVE_130OHM                          (2)       /* 75 Ohm @3.3V, 130 Ohm @1.8V */
#define DRIVE_90OHM                           (3)       /* 50 Ohm @3.3V, 90 Ohm @1.8V */
#define DRIVE_60OHM                           (4)       /* 37 Ohm @3.3V, 60 Ohm @1.8V */
#define DRIVE_50OHM                           (5)       /* 30 Ohm @3.3V, 50 Ohm @1.8V */
#define DRIVE_40OHM                           (6)       /* 25 Ohm @3.3V, 40 Ohm @1.8V */
#define DRIVE_33OHM                           (7)       /* 20 Ohm @3.3V, 33 Ohm @1.8V */

#define SPEED_LOW                             (0)       /* Low frequency (50 MHz) */
#define SPEED_MEDIUM                          (2)       /* Medium frequency (100, MHz) */
#define SPEED_MAX                             (3)       /* Maximum frequency (200 MHz) */

#define PULL_DOWN_100K                        (0)       /* 100K Ohm Pull Down */
#define PULL_UP_47K                           (1)       /*  47K Ohm Pull Up */
#define PULL_UP_100K                          (2)       /* 100K Ohm Pull Up */
#define PULL_UP_22K                           (3)       /*  22K Ohm Pull Up */

#define PADCTL_SRE                            (1 << 0)  /* Bit 0: Slew Rate Field */
#define PADCTL_DSE_SHIFT                      (3)       /* Bits 3-5: Drive Strength Field */
#define PADCTL_DSE_MASK                       (7 << PADCTL_DSE_SHIFT)
#  define PADCTL_DSE(n)                       ((uint32_t)(n) << PADCTL_DSE_SHIFT) /* n=DRIVE_* */
#  define PADCTL_DSE_HIZ                      (0 << PADCTL_DSE_SHIFT) /* HI-Z */
#  define PADCTL_DSE_260OHM                   (1 << PADCTL_DSE_SHIFT) /* 150 Ohm @3.3V, 260 Ohm @1.8V */
#  define PADCTL_DSE_130OHM                   (2 << PADCTL_DSE_SHIFT) /* 75 Ohm @3.3V, 130 Ohm @1.8V */
#  define PADCTL_DSE_90OHM                    (3 << PADCTL_DSE_SHIFT) /* 50 Ohm @3.3V, 90 Ohm @1.8V */
#  define PADCTL_DSE_60OHM                    (4 << PADCTL_DSE_SHIFT) /* 37 Ohm @3.3V, 60 Ohm @1.8V */
#  define PADCTL_DSE_50OHM                    (5 << PADCTL_DSE_SHIFT) /* 30 Ohm @3.3V, 50 Ohm @1.8V */
#  define PADCTL_DSE_40OHM                    (6 << PADCTL_DSE_SHIFT) /* 25 Ohm @3.3V, 40 Ohm @1.8V */
#  define PADCTL_DSE_33OHM                    (7 << PADCTL_DSE_SHIFT) /* 20 Ohm @3.3V, 33 Ohm @1.8V */
#define PADCTL_SPEED_SHIFT                    (6)       /* Bits 6-7: Speed Field */
#define PADCTL_SPEED_MASK                     (3 << PADCTL_SPEED_SHIFT)
#  define PADCTL_SPEED(n)                     ((uint32_t)(n) << PADCTL_SPEED_SHIFT) /* n=SPEED_* */
#  define PADCTL_SPEED_LOW                    (0 << PADCTL_SPEED_SHIFT) /* Low frequency (50 MHz) */
#  define PADCTL_SPEED_MEDIUM                 (1 << PADCTL_SPEED_SHIFT) /* Medium frequency (100, 150 MHz) */
#  define PADCTL_SPEED_MAX                    (3 << PADCTL_SPEED_SHIFT) /* Maximum frequency (100, 150, 200 MHz) */
#define PADCTL_ODE                            (1 << 11) /* Bit 11: Open Drain Enable Field */
#define PADCTL_PKE                            (1 << 12) /* Bit 12: Pull / Keep Enable Field */
#define PADCTL_PUE                            (1 << 13) /* Bit 13: Pull / Keep Select Field */
#define PADCTL_PUS_SHIFT                      (14)      /* Bits 14-15: Pull Up / Down Config. Field */
#define PADCTL_PUS_MASK                       (3 << PADCTL_PUS_SHIFT)
#  define PADCTL_PUS(n)                       ((uint32_t)(n) << PADCTL_PUS_SHIFT) /* n=PULL_* */
#  define PADCTL_PUS_DOWN_100K                (0 << PADCTL_PUS_SHIFT) /* 100K Ohm Pull Down */
#  define PADCTL_PUS_UP_47K                   (1 << PADCTL_PUS_SHIFT) /* 47K Ohm Pull Up */
#  define PADCTL_PUS_UP_100K                  (2 << PADCTL_PUS_SHIFT) /* 100K Ohm Pull Up */
#  define PADCTL_PUS_UP_22K                   (3 << PADCTL_PUS_SHIFT) /*  22K Ohm Pull Up */
#define PADCTL_HYS                            (1 << 16) /* Bit 16: Hysteresis Enable Field */

#endif /* __ARCH_ARM_SRC_IMXRT_CHIP_IMXRT_IOMUXC_H */
