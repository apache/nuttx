/************************************************************************************
 * arch/arm/src/imx6/hardware/imx_iomuxc.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Reference:
 *   "i.MX 6Dual/6Quad ApplicationsProcessor Reference Manual," Document Number
 *   IMX6DQRM, Rev. 3, 07/2015, FreeScale.
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

#ifndef __ARCH_ARM_SRC_IMX6_HARDWARE_IMX_IOMUXC_H
#define __ARCH_ARM_SRC_IMX6_HARDWARE_IMX_IOMUXC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "hardware/imx_memorymap.h"

/* These definitions derive from specifications for the i.MX 6Quad/6Dual and require
 * review and modification in order to support other family members.
 */

#if defined(CONFIG_ARCH_CHIP_IMX6_6QUAD) || defined(CONFIG_ARCH_CHIP_IMX6_6DUAL)

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* IOMUXC Register Offsets **********************************************************/
/* General Purpose Registers */

#define IMX_IOMUXC_GPR0_OFFSET                0x0000
#define IMX_IOMUXC_GPR1_OFFSET                0x0004
#define IMX_IOMUXC_GPR2_OFFSET                0x0008
#define IMX_IOMUXC_GPR3_OFFSET                0x000c
#define IMX_IOMUXC_GPR4_OFFSET                0x0010
#define IMX_IOMUXC_GPR5_OFFSET                0x0014
#define IMX_IOMUXC_GPR6_OFFSET                0x0018
#define IMX_IOMUXC_GPR7_OFFSET                0x001c
#define IMX_IOMUXC_GPR8_OFFSET                0x0020
#define IMX_IOMUXC_GPR9_OFFSET                0x0024
#define IMX_IOMUXC_GPR10_OFFSET               0x0028
#define IMX_IOMUXC_GPR11_OFFSET               0x002c
#define IMX_IOMUXC_GPR12_OFFSET               0x0030
#define IMX_IOMUXC_GPR13_OFFSET               0x0034

/* Pad Mux Registers */
/* Pad Mux Register Indices (used by software for table lookups) */

#define IMX_PADMUX_SD2_DATA1_INDEX              0
#define IMX_PADMUX_SD2_DATA2_INDEX              1
#define IMX_PADMUX_SD2_DATA0_INDEX              2
#define IMX_PADMUX_RGMII_TXC_INDEX              3
#define IMX_PADMUX_RGMII_TD0_INDEX              4
#define IMX_PADMUX_RGMII_TD1_INDEX              5
#define IMX_PADMUX_RGMII_TD2_INDEX              6
#define IMX_PADMUX_RGMII_TD3_INDEX              7
#define IMX_PADMUX_RGMII_RX_CTL_INDEX           8
#define IMX_PADMUX_RGMII_RD0_INDEX              9
#define IMX_PADMUX_RGMII_TX_CTL_INDEX          10
#define IMX_PADMUX_RGMII_RD1_INDEX             11
#define IMX_PADMUX_RGMII_RD2_INDEX             12
#define IMX_PADMUX_RGMII_RD3_INDEX             13
#define IMX_PADMUX_RGMII_RXC_INDEX             14
#define IMX_PADMUX_EIM_ADDR25_INDEX            15
#define IMX_PADMUX_EIM_EB2_INDEX               16
#define IMX_PADMUX_EIM_DATA16_INDEX            17
#define IMX_PADMUX_EIM_DATA17_INDEX            18
#define IMX_PADMUX_EIM_DATA18_INDEX            19
#define IMX_PADMUX_EIM_DATA19_INDEX            20
#define IMX_PADMUX_EIM_DATA20_INDEX            21
#define IMX_PADMUX_EIM_DATA21_INDEX            22
#define IMX_PADMUX_EIM_DATA22_INDEX            23
#define IMX_PADMUX_EIM_DATA23_INDEX            24
#define IMX_PADMUX_EIM_EB3_INDEX               25
#define IMX_PADMUX_EIM_DATA24_INDEX            26
#define IMX_PADMUX_EIM_DATA25_INDEX            27
#define IMX_PADMUX_EIM_DATA26_INDEX            28
#define IMX_PADMUX_EIM_DATA27_INDEX            29
#define IMX_PADMUX_EIM_DATA28_INDEX            30
#define IMX_PADMUX_EIM_DATA29_INDEX            31
#define IMX_PADMUX_EIM_DATA30_INDEX            32
#define IMX_PADMUX_EIM_DATA31_INDEX            33
#define IMX_PADMUX_EIM_ADDR24_INDEX            34
#define IMX_PADMUX_EIM_ADDR23_INDEX            35
#define IMX_PADMUX_EIM_ADDR22_INDEX            36
#define IMX_PADMUX_EIM_ADDR21_INDEX            37
#define IMX_PADMUX_EIM_ADDR20_INDEX            38
#define IMX_PADMUX_EIM_ADDR19_INDEX            49
#define IMX_PADMUX_EIM_ADDR18_INDEX            40
#define IMX_PADMUX_EIM_ADDR17_INDEX            41
#define IMX_PADMUX_EIM_ADDR16_INDEX            42
#define IMX_PADMUX_EIM_CS0_INDEX               43
#define IMX_PADMUX_EIM_CS1_INDEX               44
#define IMX_PADMUX_EIM_OE_INDEX                45
#define IMX_PADMUX_EIM_RW_INDEX                46
#define IMX_PADMUX_EIM_LBA_INDEX               47
#define IMX_PADMUX_EIM_EB0_INDEX               58
#define IMX_PADMUX_EIM_EB1_INDEX               59
#define IMX_PADMUX_EIM_AD00_INDEX              50
#define IMX_PADMUX_EIM_AD01_INDEX              51
#define IMX_PADMUX_EIM_AD02_INDEX              52
#define IMX_PADMUX_EIM_AD03_INDEX              53
#define IMX_PADMUX_EIM_AD04_INDEX              54
#define IMX_PADMUX_EIM_AD05_INDEX              55
#define IMX_PADMUX_EIM_AD06_INDEX              56
#define IMX_PADMUX_EIM_AD07_INDEX              67
#define IMX_PADMUX_EIM_AD08_INDEX              68
#define IMX_PADMUX_EIM_AD09_INDEX              69
#define IMX_PADMUX_EIM_AD10_INDEX              60
#define IMX_PADMUX_EIM_AD11_INDEX              61
#define IMX_PADMUX_EIM_AD12_INDEX              62
#define IMX_PADMUX_EIM_AD13_INDEX              63
#define IMX_PADMUX_EIM_AD14_INDEX              64
#define IMX_PADMUX_EIM_AD15_INDEX              65
#define IMX_PADMUX_EIM_WAIT_INDEX              66
#define IMX_PADMUX_EIM_BCLK_INDEX              67
#define IMX_PADMUX_DI0_DISP_CLK_INDEX          68
#define IMX_PADMUX_DI0_PIN15_INDEX             69
#define IMX_PADMUX_DI0_PIN02_INDEX             70
#define IMX_PADMUX_DI0_PIN03_INDEX             71
#define IMX_PADMUX_DI0_PIN04_INDEX             72
#define IMX_PADMUX_DISP0_DATA00_INDEX          73
#define IMX_PADMUX_DISP0_DATA01_INDEX          74
#define IMX_PADMUX_DISP0_DATA02_INDEX          75
#define IMX_PADMUX_DISP0_DATA03_INDEX          76
#define IMX_PADMUX_DISP0_DATA04_INDEX          77
#define IMX_PADMUX_DISP0_DATA05_INDEX          78
#define IMX_PADMUX_DISP0_DATA06_INDEX          79
#define IMX_PADMUX_DISP0_DATA07_INDEX          80
#define IMX_PADMUX_DISP0_DATA08_INDEX          81
#define IMX_PADMUX_DISP0_DATA09_INDEX          82
#define IMX_PADMUX_DISP0_DATA10_INDEX          83
#define IMX_PADMUX_DISP0_DATA11_INDEX          84
#define IMX_PADMUX_DISP0_DATA12_INDEX          85
#define IMX_PADMUX_DISP0_DATA13_INDEX          86
#define IMX_PADMUX_DISP0_DATA14_INDEX          87
#define IMX_PADMUX_DISP0_DATA15_INDEX          88
#define IMX_PADMUX_DISP0_DATA16_INDEX          89
#define IMX_PADMUX_DISP0_DATA17_INDEX          90
#define IMX_PADMUX_DISP0_DATA18_INDEX          91
#define IMX_PADMUX_DISP0_DATA19_INDEX          92
#define IMX_PADMUX_DISP0_DATA20_INDEX          93
#define IMX_PADMUX_DISP0_DATA21_INDEX          94
#define IMX_PADMUX_DISP0_DATA22_INDEX          95
#define IMX_PADMUX_DISP0_DATA23_INDEX          96
#define IMX_PADMUX_ENET_MDIO_INDEX             97
#define IMX_PADMUX_ENET_REF_CLK_INDEX          98
#define IMX_PADMUX_ENET_RX_ER_INDEX            99
#define IMX_PADMUX_ENET_CRS_DV_INDEX          100
#define IMX_PADMUX_ENET_RX_DATA1_INDEX        101
#define IMX_PADMUX_ENET_RX_DATA0_INDEX        102
#define IMX_PADMUX_ENET_TX_EN_INDEX           103
#define IMX_PADMUX_ENET_TX_DATA1_INDEX        104
#define IMX_PADMUX_ENET_TX_DATA0_INDEX        105
#define IMX_PADMUX_ENET_MDC_INDEX             106
#define IMX_PADMUX_KEY_COL0_INDEX             107
#define IMX_PADMUX_KEY_ROW0_INDEX             108
#define IMX_PADMUX_KEY_COL1_INDEX             109
#define IMX_PADMUX_KEY_ROW1_INDEX             110
#define IMX_PADMUX_KEY_COL2_INDEX             111
#define IMX_PADMUX_KEY_ROW2_INDEX             112
#define IMX_PADMUX_KEY_COL3_INDEX             113
#define IMX_PADMUX_KEY_ROW3_INDEX             114
#define IMX_PADMUX_KEY_COL4_INDEX             115
#define IMX_PADMUX_KEY_ROW4_INDEX             116
#define IMX_PADMUX_GPIO00_INDEX               117
#define IMX_PADMUX_GPIO01_INDEX               118
#define IMX_PADMUX_GPIO09_INDEX               119
#define IMX_PADMUX_GPIO03_INDEX               120
#define IMX_PADMUX_GPIO06_INDEX               121
#define IMX_PADMUX_GPIO02_INDEX               122
#define IMX_PADMUX_GPIO04_INDEX               123
#define IMX_PADMUX_GPIO05_INDEX               124
#define IMX_PADMUX_GPIO07_INDEX               125
#define IMX_PADMUX_GPIO08_INDEX               126
#define IMX_PADMUX_GPIO16_INDEX               127
#define IMX_PADMUX_GPIO17_INDEX               128
#define IMX_PADMUX_GPIO18_INDEX               129
#define IMX_PADMUX_GPIO19_INDEX               130
#define IMX_PADMUX_CSI0_PIXCLK_INDEX          131
#define IMX_PADMUX_CSI0_HSYNC_INDEX           132
#define IMX_PADMUX_CSI0_DATA_EN_INDEX         133
#define IMX_PADMUX_CSI0_VSYNC_INDEX           134
#define IMX_PADMUX_CSI0_DATA04_INDEX          135
#define IMX_PADMUX_CSI0_DATA05_INDEX          136
#define IMX_PADMUX_CSI0_DATA06_INDEX          137
#define IMX_PADMUX_CSI0_DATA07_INDEX          138
#define IMX_PADMUX_CSI0_DATA08_INDEX          139
#define IMX_PADMUX_CSI0_DATA09_INDEX          140
#define IMX_PADMUX_CSI0_DATA10_INDEX          141
#define IMX_PADMUX_CSI0_DATA11_INDEX          142
#define IMX_PADMUX_CSI0_DATA12_INDEX          143
#define IMX_PADMUX_CSI0_DATA13_INDEX          144
#define IMX_PADMUX_CSI0_DATA14_INDEX          145
#define IMX_PADMUX_CSI0_DATA15_INDEX          146
#define IMX_PADMUX_CSI0_DATA16_INDEX          147
#define IMX_PADMUX_CSI0_DATA17_INDEX          148
#define IMX_PADMUX_CSI0_DATA18_INDEX          149
#define IMX_PADMUX_CSI0_DATA19_INDEX          150
#define IMX_PADMUX_SD3_DATA7_INDEX            151
#define IMX_PADMUX_SD3_DATA6_INDEX            152
#define IMX_PADMUX_SD3_DATA5_INDEX            153
#define IMX_PADMUX_SD3_DATA4_INDEX            154
#define IMX_PADMUX_SD3_CMD_INDEX              155
#define IMX_PADMUX_SD3_CLK_INDEX              156
#define IMX_PADMUX_SD3_DATA0_INDEX            157
#define IMX_PADMUX_SD3_DATA1_INDEX            158
#define IMX_PADMUX_SD3_DATA2_INDEX            159
#define IMX_PADMUX_SD3_DATA3_INDEX            160
#define IMX_PADMUX_SD3_RESET_INDEX            161
#define IMX_PADMUX_NAND_CLE_INDEX             162
#define IMX_PADMUX_NAND_ALE_INDEX             163
#define IMX_PADMUX_NAND_WP_INDEX              164
#define IMX_PADMUX_NAND_READY_INDEX           165
#define IMX_PADMUX_NAND_CS0_INDEX             166
#define IMX_PADMUX_NAND_CS1_INDEX             167
#define IMX_PADMUX_NAND_CS2_INDEX             168
#define IMX_PADMUX_NAND_CS3_INDEX             169
#define IMX_PADMUX_SD4_CMD_INDEX              170
#define IMX_PADMUX_SD4_CLK_INDEX              171
#define IMX_PADMUX_NAND_DATA00_INDEX          172
#define IMX_PADMUX_NAND_DATA01_INDEX          173
#define IMX_PADMUX_NAND_DATA02_INDEX          174
#define IMX_PADMUX_NAND_DATA03_INDEX          175
#define IMX_PADMUX_NAND_DATA04_INDEX          176
#define IMX_PADMUX_NAND_DATA05_INDEX          177
#define IMX_PADMUX_NAND_DATA06_INDEX          178
#define IMX_PADMUX_NAND_DATA07_INDEX          189
#define IMX_PADMUX_SD4_DATA0_INDEX            180
#define IMX_PADMUX_SD4_DATA1_INDEX            181
#define IMX_PADMUX_SD4_DATA2_INDEX            182
#define IMX_PADMUX_SD4_DATA3_INDEX            183
#define IMX_PADMUX_SD4_DATA4_INDEX            184
#define IMX_PADMUX_SD4_DATA5_INDEX            185
#define IMX_PADMUX_SD4_DATA6_INDEX            186
#define IMX_PADMUX_SD4_DATA7_INDEX            187
#define IMX_PADMUX_SD1_DATA1_INDEX            188
#define IMX_PADMUX_SD1_DATA0_INDEX            189
#define IMX_PADMUX_SD1_DATA3_INDEX            190
#define IMX_PADMUX_SD1_CMD_INDEX              191
#define IMX_PADMUX_SD1_DATA2_INDEX            192
#define IMX_PADMUX_SD1_CLK_INDEX              193
#define IMX_PADMUX_SD2_CLK_INDEX              194
#define IMX_PADMUX_SD2_CMD_INDEX              195
#define IMX_PADMUX_SD2_DATA3_INDEX            196

#define IMX_PADMUX_NREGISTERS                 197

/* Pad Mux Register Offsets */

#define IMX_PADMUX_OFFSET(n)                  (0x004c + ((unsigned int)(n) << 2))

#define IMX_PADMUX_SD2_DATA1_OFFSET           0x004c
#define IMX_PADMUX_SD2_DATA2_OFFSET           0x0050
#define IMX_PADMUX_SD2_DATA0_OFFSET           0x0054
#define IMX_PADMUX_RGMII_TXC_OFFSET           0x0058
#define IMX_PADMUX_RGMII_TD0_OFFSET           0x005c
#define IMX_PADMUX_RGMII_TD1_OFFSET           0x0060
#define IMX_PADMUX_RGMII_TD2_OFFSET           0x0064
#define IMX_PADMUX_RGMII_TD3_OFFSET           0x0068
#define IMX_PADMUX_RGMII_RX_CTL_OFFSET        0x006c
#define IMX_PADMUX_RGMII_RD0_OFFSET           0x0070
#define IMX_PADMUX_RGMII_TX_CTL_OFFSET        0x0074
#define IMX_PADMUX_RGMII_RD1_OFFSET           0x0078
#define IMX_PADMUX_RGMII_RD2_OFFSET           0x007c
#define IMX_PADMUX_RGMII_RD3_OFFSET           0x0080
#define IMX_PADMUX_RGMII_RXC_OFFSET           0x0084
#define IMX_PADMUX_EIM_ADDR25_OFFSET          0x0088
#define IMX_PADMUX_EIM_EB2_OFFSET             0x008c
#define IMX_PADMUX_EIM_DATA16_OFFSET          0x0090
#define IMX_PADMUX_EIM_DATA17_OFFSET          0x0094
#define IMX_PADMUX_EIM_DATA18_OFFSET          0x0098
#define IMX_PADMUX_EIM_DATA19_OFFSET          0x009c
#define IMX_PADMUX_EIM_DATA20_OFFSET          0x00a0
#define IMX_PADMUX_EIM_DATA21_OFFSET          0x00a4
#define IMX_PADMUX_EIM_DATA22_OFFSET          0x00a8
#define IMX_PADMUX_EIM_DATA23_OFFSET          0x00ac
#define IMX_PADMUX_EIM_EB3_OFFSET             0x00b0
#define IMX_PADMUX_EIM_DATA24_OFFSET          0x00b4
#define IMX_PADMUX_EIM_DATA25_OFFSET          0x00b8
#define IMX_PADMUX_EIM_DATA26_OFFSET          0x00bc
#define IMX_PADMUX_EIM_DATA27_OFFSET          0x00c0
#define IMX_PADMUX_EIM_DATA28_OFFSET          0x00c4
#define IMX_PADMUX_EIM_DATA29_OFFSET          0x00c8
#define IMX_PADMUX_EIM_DATA30_OFFSET          0x00cc
#define IMX_PADMUX_EIM_DATA31_OFFSET          0x00d0
#define IMX_PADMUX_EIM_ADDR24_OFFSET          0x00d4
#define IMX_PADMUX_EIM_ADDR23_OFFSET          0x00d8
#define IMX_PADMUX_EIM_ADDR22_OFFSET          0x00dc
#define IMX_PADMUX_EIM_ADDR21_OFFSET          0x00e0
#define IMX_PADMUX_EIM_ADDR20_OFFSET          0x00e4
#define IMX_PADMUX_EIM_ADDR19_OFFSET          0x00e8
#define IMX_PADMUX_EIM_ADDR18_OFFSET          0x00ec
#define IMX_PADMUX_EIM_ADDR17_OFFSET          0x00f0
#define IMX_PADMUX_EIM_ADDR16_OFFSET          0x00f4
#define IMX_PADMUX_EIM_CS0_OFFSET             0x00f8
#define IMX_PADMUX_EIM_CS1_OFFSET             0x00fc
#define IMX_PADMUX_EIM_OE_OFFSET              0x0100
#define IMX_PADMUX_EIM_RW_OFFSET              0x0104
#define IMX_PADMUX_EIM_LBA_OFFSET             0x0108
#define IMX_PADMUX_EIM_EB0_OFFSET             0x010c
#define IMX_PADMUX_EIM_EB1_OFFSET             0x0110
#define IMX_PADMUX_EIM_AD00_OFFSET            0x0114
#define IMX_PADMUX_EIM_AD01_OFFSET            0x0118
#define IMX_PADMUX_EIM_AD02_OFFSET            0x011c
#define IMX_PADMUX_EIM_AD03_OFFSET            0x0120
#define IMX_PADMUX_EIM_AD04_OFFSET            0x0124
#define IMX_PADMUX_EIM_AD05_OFFSET            0x0128
#define IMX_PADMUX_EIM_AD06_OFFSET            0x012c
#define IMX_PADMUX_EIM_AD07_OFFSET            0x0130
#define IMX_PADMUX_EIM_AD08_OFFSET            0x0134
#define IMX_PADMUX_EIM_AD09_OFFSET            0x0138
#define IMX_PADMUX_EIM_AD10_OFFSET            0x013c
#define IMX_PADMUX_EIM_AD11_OFFSET            0x0140
#define IMX_PADMUX_EIM_AD12_OFFSET            0x0144
#define IMX_PADMUX_EIM_AD13_OFFSET            0x0148
#define IMX_PADMUX_EIM_AD14_OFFSET            0x014c
#define IMX_PADMUX_EIM_AD15_OFFSET            0x0150
#define IMX_PADMUX_EIM_WAIT_OFFSET            0x0154
#define IMX_PADMUX_EIM_BCLK_OFFSET            0x0158
#define IMX_PADMUX_DI0_DISP_CLK_OFFSET        0x015c
#define IMX_PADMUX_DI0_PIN15_OFFSET           0x0160
#define IMX_PADMUX_DI0_PIN02_OFFSET           0x0164
#define IMX_PADMUX_DI0_PIN03_OFFSET           0x0168
#define IMX_PADMUX_DI0_PIN04_OFFSET           0x016c
#define IMX_PADMUX_DISP0_DATA00_OFFSET        0x0170
#define IMX_PADMUX_DISP0_DATA01_OFFSET        0x0174
#define IMX_PADMUX_DISP0_DATA02_OFFSET        0x0178
#define IMX_PADMUX_DISP0_DATA03_OFFSET        0x017c
#define IMX_PADMUX_DISP0_DATA04_OFFSET        0x0180
#define IMX_PADMUX_DISP0_DATA05_OFFSET        0x0184
#define IMX_PADMUX_DISP0_DATA06_OFFSET        0x0188
#define IMX_PADMUX_DISP0_DATA07_OFFSET        0x018c
#define IMX_PADMUX_DISP0_DATA08_OFFSET        0x0190
#define IMX_PADMUX_DISP0_DATA09_OFFSET        0x0194
#define IMX_PADMUX_DISP0_DATA10_OFFSET        0x0198
#define IMX_PADMUX_DISP0_DATA11_OFFSET        0x019c
#define IMX_PADMUX_DISP0_DATA12_OFFSET        0x01a0
#define IMX_PADMUX_DISP0_DATA13_OFFSET        0x01a4
#define IMX_PADMUX_DISP0_DATA14_OFFSET        0x01a8
#define IMX_PADMUX_DISP0_DATA15_OFFSET        0x01ac
#define IMX_PADMUX_DISP0_DATA16_OFFSET        0x01b0
#define IMX_PADMUX_DISP0_DATA17_OFFSET        0x01b4
#define IMX_PADMUX_DISP0_DATA18_OFFSET        0x01b8
#define IMX_PADMUX_DISP0_DATA19_OFFSET        0x01bc
#define IMX_PADMUX_DISP0_DATA20_OFFSET        0x01c0
#define IMX_PADMUX_DISP0_DATA21_OFFSET        0x01c4
#define IMX_PADMUX_DISP0_DATA22_OFFSET        0x01c8
#define IMX_PADMUX_DISP0_DATA23_OFFSET        0x01cc
#define IMX_PADMUX_ENET_MDIO_OFFSET           0x01d0
#define IMX_PADMUX_ENET_REF_CLK_OFFSET        0x01d4
#define IMX_PADMUX_ENET_RX_ER_OFFSET          0x01d8
#define IMX_PADMUX_ENET_CRS_DV_OFFSET         0x01dc
#define IMX_PADMUX_ENET_RX_DATA1_OFFSET       0x01e0
#define IMX_PADMUX_ENET_RX_DATA0_OFFSET       0x01e4
#define IMX_PADMUX_ENET_TX_EN_OFFSET          0x01e8
#define IMX_PADMUX_ENET_TX_DATA1_OFFSET       0x01ec
#define IMX_PADMUX_ENET_TX_DATA0_OFFSET       0x01f0
#define IMX_PADMUX_ENET_MDC_OFFSET            0x01f4
#define IMX_PADMUX_KEY_COL0_OFFSET            0x01f8
#define IMX_PADMUX_KEY_ROW0_OFFSET            0x01fc
#define IMX_PADMUX_KEY_COL1_OFFSET            0x0200
#define IMX_PADMUX_KEY_ROW1_OFFSET            0x0204
#define IMX_PADMUX_KEY_COL2_OFFSET            0x0208
#define IMX_PADMUX_KEY_ROW2_OFFSET            0x020c
#define IMX_PADMUX_KEY_COL3_OFFSET            0x0210
#define IMX_PADMUX_KEY_ROW3_OFFSET            0x0214
#define IMX_PADMUX_KEY_COL4_OFFSET            0x0218
#define IMX_PADMUX_KEY_ROW4_OFFSET            0x021c
#define IMX_PADMUX_GPIO00_OFFSET              0x0220
#define IMX_PADMUX_GPIO01_OFFSET              0x0224
#define IMX_PADMUX_GPIO09_OFFSET              0x0228
#define IMX_PADMUX_GPIO03_OFFSET              0x022c
#define IMX_PADMUX_GPIO06_OFFSET              0x0230
#define IMX_PADMUX_GPIO02_OFFSET              0x0234
#define IMX_PADMUX_GPIO04_OFFSET              0x0238
#define IMX_PADMUX_GPIO05_OFFSET              0x023c
#define IMX_PADMUX_GPIO07_OFFSET              0x0240
#define IMX_PADMUX_GPIO08_OFFSET              0x0244
#define IMX_PADMUX_GPIO16_OFFSET              0x0248
#define IMX_PADMUX_GPIO17_OFFSET              0x024c
#define IMX_PADMUX_GPIO18_OFFSET              0x0250
#define IMX_PADMUX_GPIO19_OFFSET              0x0254
#define IMX_PADMUX_CSI0_PIXCLK_OFFSET         0x0258
#define IMX_PADMUX_CSI0_HSYNC_OFFSET          0x025c
#define IMX_PADMUX_CSI0_DATA_EN_OFFSET        0x0260
#define IMX_PADMUX_CSI0_VSYNC_OFFSET          0x0264
#define IMX_PADMUX_CSI0_DATA04_OFFSET         0x0268
#define IMX_PADMUX_CSI0_DATA05_OFFSET         0x026c
#define IMX_PADMUX_CSI0_DATA06_OFFSET         0x0270
#define IMX_PADMUX_CSI0_DATA07_OFFSET         0x0274
#define IMX_PADMUX_CSI0_DATA08_OFFSET         0x0278
#define IMX_PADMUX_CSI0_DATA09_OFFSET         0x027c
#define IMX_PADMUX_CSI0_DATA10_OFFSET         0x0280
#define IMX_PADMUX_CSI0_DATA11_OFFSET         0x0284
#define IMX_PADMUX_CSI0_DATA12_OFFSET         0x0288
#define IMX_PADMUX_CSI0_DATA13_OFFSET         0x028c
#define IMX_PADMUX_CSI0_DATA14_OFFSET         0x0290
#define IMX_PADMUX_CSI0_DATA15_OFFSET         0x0294
#define IMX_PADMUX_CSI0_DATA16_OFFSET         0x0298
#define IMX_PADMUX_CSI0_DATA17_OFFSET         0x029c
#define IMX_PADMUX_CSI0_DATA18_OFFSET         0x02a0
#define IMX_PADMUX_CSI0_DATA19_OFFSET         0x02a4
#define IMX_PADMUX_SD3_DATA7_OFFSET           0x02a8
#define IMX_PADMUX_SD3_DATA6_OFFSET           0x02ac
#define IMX_PADMUX_SD3_DATA5_OFFSET           0x02b0
#define IMX_PADMUX_SD3_DATA4_OFFSET           0x02b4
#define IMX_PADMUX_SD3_CMD_OFFSET             0x02b8
#define IMX_PADMUX_SD3_CLK_OFFSET             0x02bc
#define IMX_PADMUX_SD3_DATA0_OFFSET           0x02c0
#define IMX_PADMUX_SD3_DATA1_OFFSET           0x02c4
#define IMX_PADMUX_SD3_DATA2_OFFSET           0x02c8
#define IMX_PADMUX_SD3_DATA3_OFFSET           0x02cc
#define IMX_PADMUX_SD3_RESET_OFFSET           0x02d0
#define IMX_PADMUX_NAND_CLE_OFFSET            0x02d4
#define IMX_PADMUX_NAND_ALE_OFFSET            0x02d8
#define IMX_PADMUX_NAND_WP_OFFSET             0x02dc
#define IMX_PADMUX_NAND_READY_OFFSET          0x02e0
#define IMX_PADMUX_NAND_CS0_OFFSET            0x02e4
#define IMX_PADMUX_NAND_CS1_OFFSET            0x02e8
#define IMX_PADMUX_NAND_CS2_OFFSET            0x02ec
#define IMX_PADMUX_NAND_CS3_OFFSET            0x02f0
#define IMX_PADMUX_SD4_CMD_OFFSET             0x02f4
#define IMX_PADMUX_SD4_CLK_OFFSET             0x02f8
#define IMX_PADMUX_NAND_DATA00_OFFSET         0x02fc
#define IMX_PADMUX_NAND_DATA01_OFFSET         0x0300
#define IMX_PADMUX_NAND_DATA02_OFFSET         0x0304
#define IMX_PADMUX_NAND_DATA03_OFFSET         0x0308
#define IMX_PADMUX_NAND_DATA04_OFFSET         0x030c
#define IMX_PADMUX_NAND_DATA05_OFFSET         0x0310
#define IMX_PADMUX_NAND_DATA06_OFFSET         0x0314
#define IMX_PADMUX_NAND_DATA07_OFFSET         0x0318
#define IMX_PADMUX_SD4_DATA0_OFFSET           0x031c
#define IMX_PADMUX_SD4_DATA1_OFFSET           0x0320
#define IMX_PADMUX_SD4_DATA2_OFFSET           0x0324
#define IMX_PADMUX_SD4_DATA3_OFFSET           0x0328
#define IMX_PADMUX_SD4_DATA4_OFFSET           0x032c
#define IMX_PADMUX_SD4_DATA5_OFFSET           0x0330
#define IMX_PADMUX_SD4_DATA6_OFFSET           0x0334
#define IMX_PADMUX_SD4_DATA7_OFFSET           0x0338
#define IMX_PADMUX_SD1_DATA1_OFFSET           0x033c
#define IMX_PADMUX_SD1_DATA0_OFFSET           0x0340
#define IMX_PADMUX_SD1_DATA3_OFFSET           0x0344
#define IMX_PADMUX_SD1_CMD_OFFSET             0x0348
#define IMX_PADMUX_SD1_DATA2_OFFSET           0x034c
#define IMX_PADMUX_SD1_CLK_OFFSET             0x0350
#define IMX_PADMUX_SD2_CLK_OFFSET             0x0354
#define IMX_PADMUX_SD2_CMD_OFFSET             0x0358
#define IMX_PADMUX_SD2_DATA3_OFFSET           0x035c

/* Pad Control Registers */
/* Pad Mux Register Indices (used by software for table lookups) */

#define IMX_PADCTL_SD2_DATA1_INDEX              0
#define IMX_PADCTL_SD2_DATA2_INDEX              1
#define IMX_PADCTL_SD2_DATA0_INDEX              2
#define IMX_PADCTL_RGMII_TXC_INDEX              3
#define IMX_PADCTL_RGMII_TD0_INDEX              4
#define IMX_PADCTL_RGMII_TD1_INDEX              5
#define IMX_PADCTL_RGMII_TD2_INDEX              6
#define IMX_PADCTL_RGMII_TD3_INDEX              7
#define IMX_PADCTL_RGMII_RX_CTL_INDEX           8
#define IMX_PADCTL_RGMII_RD0_INDEX              9
#define IMX_PADCTL_RGMII_TX_CTL_INDEX          10
#define IMX_PADCTL_RGMII_RD1_INDEX             11
#define IMX_PADCTL_RGMII_RD2_INDEX             12
#define IMX_PADCTL_RGMII_RD3_INDEX             13
#define IMX_PADCTL_RGMII_RXC_INDEX             14
#define IMX_PADCTL_EIM_ADDR25_INDEX            15
#define IMX_PADCTL_EIM_EB2_INDEX               16
#define IMX_PADCTL_EIM_DATA16_INDEX            17
#define IMX_PADCTL_EIM_DATA17_INDEX            18
#define IMX_PADCTL_EIM_DATA18_INDEX            19
#define IMX_PADCTL_EIM_DATA19_INDEX            20
#define IMX_PADCTL_EIM_DATA20_INDEX            21
#define IMX_PADCTL_EIM_DATA21_INDEX            22
#define IMX_PADCTL_EIM_DATA22_INDEX            23
#define IMX_PADCTL_EIM_DATA23_INDEX            24
#define IMX_PADCTL_EIM_EB3_INDEX               25
#define IMX_PADCTL_EIM_DATA24_INDEX            26
#define IMX_PADCTL_EIM_DATA25_INDEX            27
#define IMX_PADCTL_EIM_DATA26_INDEX            28
#define IMX_PADCTL_EIM_DATA27_INDEX            29
#define IMX_PADCTL_EIM_DATA28_INDEX            30
#define IMX_PADCTL_EIM_DATA29_INDEX            31
#define IMX_PADCTL_EIM_DATA30_INDEX            32
#define IMX_PADCTL_EIM_DATA31_INDEX            33
#define IMX_PADCTL_EIM_ADDR24_INDEX            34
#define IMX_PADCTL_EIM_ADDR23_INDEX            35
#define IMX_PADCTL_EIM_ADDR22_INDEX            36
#define IMX_PADCTL_EIM_ADDR21_INDEX            37
#define IMX_PADCTL_EIM_ADDR20_INDEX            38
#define IMX_PADCTL_EIM_ADDR19_INDEX            39
#define IMX_PADCTL_EIM_ADDR18_INDEX            40
#define IMX_PADCTL_EIM_ADDR17_INDEX            41
#define IMX_PADCTL_EIM_ADDR16_INDEX            42
#define IMX_PADCTL_EIM_CS0_INDEX               43
#define IMX_PADCTL_EIM_CS1_INDEX               44
#define IMX_PADCTL_EIM_OE_INDEX                45
#define IMX_PADCTL_EIM_RW_INDEX                46
#define IMX_PADCTL_EIM_LBA_INDEX               47
#define IMX_PADCTL_EIM_EB0_INDEX               48
#define IMX_PADCTL_EIM_EB1_INDEX               49
#define IMX_PADCTL_EIM_AD00_INDEX              50
#define IMX_PADCTL_EIM_AD01_INDEX              51
#define IMX_PADCTL_EIM_AD02_INDEX              52
#define IMX_PADCTL_EIM_AD03_INDEX              53
#define IMX_PADCTL_EIM_AD04_INDEX              54
#define IMX_PADCTL_EIM_AD05_INDEX              55
#define IMX_PADCTL_EIM_AD06_INDEX              56
#define IMX_PADCTL_EIM_AD07_INDEX              57
#define IMX_PADCTL_EIM_AD08_INDEX              58
#define IMX_PADCTL_EIM_AD09_INDEX              59
#define IMX_PADCTL_EIM_AD10_INDEX              60
#define IMX_PADCTL_EIM_AD11_INDEX              61
#define IMX_PADCTL_EIM_AD12_INDEX              62
#define IMX_PADCTL_EIM_AD13_INDEX              63
#define IMX_PADCTL_EIM_AD14_INDEX              64
#define IMX_PADCTL_EIM_AD15_INDEX              65
#define IMX_PADCTL_EIM_WAIT_INDEX              66
#define IMX_PADCTL_EIM_BCLK_INDEX              67
#define IMX_PADCTL_DI0_DISP_CLK_INDEX          68
#define IMX_PADCTL_DI0_PIN15_INDEX             69
#define IMX_PADCTL_DI0_PIN02_INDEX             70
#define IMX_PADCTL_DI0_PIN03_INDEX             71
#define IMX_PADCTL_DI0_PIN04_INDEX             72
#define IMX_PADCTL_DISP0_DATA00_INDEX          73
#define IMX_PADCTL_DISP0_DATA01_INDEX          74
#define IMX_PADCTL_DISP0_DATA02_INDEX          75
#define IMX_PADCTL_DISP0_DATA03_INDEX          76
#define IMX_PADCTL_DISP0_DATA04_INDEX          77
#define IMX_PADCTL_DISP0_DATA05_INDEX          78
#define IMX_PADCTL_DISP0_DATA06_INDEX          79
#define IMX_PADCTL_DISP0_DATA07_INDEX          80
#define IMX_PADCTL_DISP0_DATA08_INDEX          81
#define IMX_PADCTL_DISP0_DATA09_INDEX          82
#define IMX_PADCTL_DISP0_DATA10_INDEX          83
#define IMX_PADCTL_DISP0_DATA11_INDEX          84
#define IMX_PADCTL_DISP0_DATA12_INDEX          85
#define IMX_PADCTL_DISP0_DATA13_INDEX          86
#define IMX_PADCTL_DISP0_DATA14_INDEX          87
#define IMX_PADCTL_DISP0_DATA15_INDEX          88
#define IMX_PADCTL_DISP0_DATA16_INDEX          89
#define IMX_PADCTL_DISP0_DATA17_INDEX          90
#define IMX_PADCTL_DISP0_DATA18_INDEX          91
#define IMX_PADCTL_DISP0_DATA19_INDEX          92
#define IMX_PADCTL_DISP0_DATA20_INDEX          93
#define IMX_PADCTL_DISP0_DATA21_INDEX          94
#define IMX_PADCTL_DISP0_DATA22_INDEX          95
#define IMX_PADCTL_DISP0_DATA23_INDEX          96
#define IMX_PADCTL_ENET_MDIO_INDEX             97
#define IMX_PADCTL_ENET_REF_CLK_INDEX          98
#define IMX_PADCTL_ENET_RX_ER_INDEX            99
#define IMX_PADCTL_ENET_CRS_DV_INDEX          100
#define IMX_PADCTL_ENET_RX_DATA1_INDEX        101
#define IMX_PADCTL_ENET_RX_DATA0_INDEX        102
#define IMX_PADCTL_ENET_TX_EN_INDEX           103
#define IMX_PADCTL_ENET_TX_DATA1_INDEX        104
#define IMX_PADCTL_ENET_TX_DATA0_INDEX        105
#define IMX_PADCTL_ENET_MDC_INDEX             106
#define IMX_PADCTL_DRAM_SDQS5_P_INDEX         107
#define IMX_PADCTL_DRAM_DQM5_INDEX            108
#define IMX_PADCTL_DRAM_DQM4_INDEX            109
#define IMX_PADCTL_DRAM_SDQS4_P_INDEX         110
#define IMX_PADCTL_DRAM_SDQS3_P_INDEX         111
#define IMX_PADCTL_DRAM_DQM3_INDEX            112
#define IMX_PADCTL_DRAM_SDQS2_P_INDEX         113
#define IMX_PADCTL_DRAM_DQM2_INDEX            114
#define IMX_PADCTL_DRAM_ADDR00_INDEX          115
#define IMX_PADCTL_DRAM_ADDR01_INDEX          116
#define IMX_PADCTL_DRAM_ADDR02_INDEX          117
#define IMX_PADCTL_DRAM_ADDR03_INDEX          118
#define IMX_PADCTL_DRAM_ADDR04_INDEX          119
#define IMX_PADCTL_DRAM_ADDR05_INDEX          120
#define IMX_PADCTL_DRAM_ADDR06_INDEX          121
#define IMX_PADCTL_DRAM_ADDR07_INDEX          122
#define IMX_PADCTL_DRAM_ADDR08_INDEX          123
#define IMX_PADCTL_DRAM_ADDR09_INDEX          124
#define IMX_PADCTL_DRAM_ADDR10_INDEX          125
#define IMX_PADCTL_DRAM_ADDR11_INDEX          126
#define IMX_PADCTL_DRAM_ADDR12_INDEX          127
#define IMX_PADCTL_DRAM_ADDR13_INDEX          128
#define IMX_PADCTL_DRAM_ADDR14_INDEX          129
#define IMX_PADCTL_DRAM_ADDR15_INDEX          130
#define IMX_PADCTL_DRAM_CAS_INDEX             131
#define IMX_PADCTL_DRAM_CS0_INDEX             132
#define IMX_PADCTL_DRAM_CS1_INDEX             133
#define IMX_PADCTL_DRAM_RAS_INDEX             134
#define IMX_PADCTL_DRAM_RESET_INDEX           135
#define IMX_PADCTL_DRAM_SDBA0_INDEX           136
#define IMX_PADCTL_DRAM_SDBA1_INDEX           137
#define IMX_PADCTL_DRAM_SDCLK0_P_INDEX        138
#define IMX_PADCTL_DRAM_SDBA2_INDEX           149
#define IMX_PADCTL_DRAM_SDCKE0_INDEX          140
#define IMX_PADCTL_DRAM_SDCLK1_P_INDEX        141
#define IMX_PADCTL_DRAM_SDCKE1_INDEX          142
#define IMX_PADCTL_DRAM_ODT0_INDEX            143
#define IMX_PADCTL_DRAM_ODT1_INDEX            144
#define IMX_PADCTL_DRAM_SDWE_B_INDEX          145
#define IMX_PADCTL_DRAM_SDQS0_P_INDEX         146
#define IMX_PADCTL_DRAM_DQM0_INDEX            147
#define IMX_PADCTL_DRAM_SDQS1_P_INDEX         148
#define IMX_PADCTL_DRAM_DQM1_INDEX            149
#define IMX_PADCTL_DRAM_SDQS6_P_INDEX         150
#define IMX_PADCTL_DRAM_DQM6_INDEX            151
#define IMX_PADCTL_DRAM_SDQS7_P_INDEX         152
#define IMX_PADCTL_DRAM_DQM7_INDEX            153
#define IMX_PADCTL_KEY_COL0_INDEX             154
#define IMX_PADCTL_KEY_ROW0_INDEX             155
#define IMX_PADCTL_KEY_COL1_INDEX             156
#define IMX_PADCTL_KEY_ROW1_INDEX             157
#define IMX_PADCTL_KEY_COL2_INDEX             158
#define IMX_PADCTL_KEY_ROW2_INDEX             159
#define IMX_PADCTL_KEY_COL3_INDEX             160
#define IMX_PADCTL_KEY_ROW3_INDEX             161
#define IMX_PADCTL_KEY_COL4_INDEX             162
#define IMX_PADCTL_KEY_ROW4_INDEX             163
#define IMX_PADCTL_GPIO00_INDEX               164
#define IMX_PADCTL_GPIO01_INDEX               165
#define IMX_PADCTL_GPIO09_INDEX               166
#define IMX_PADCTL_GPIO03_INDEX               167
#define IMX_PADCTL_GPIO06_INDEX               168
#define IMX_PADCTL_GPIO02_INDEX               169
#define IMX_PADCTL_GPIO04_INDEX               170
#define IMX_PADCTL_GPIO05_INDEX               171
#define IMX_PADCTL_GPIO07_INDEX               172
#define IMX_PADCTL_GPIO08_INDEX               173
#define IMX_PADCTL_GPIO16_INDEX               174
#define IMX_PADCTL_GPIO17_INDEX               175
#define IMX_PADCTL_GPIO18_INDEX               176
#define IMX_PADCTL_GPIO19_INDEX               177
#define IMX_PADCTL_CSI0_PIXCLK_INDEX          178
#define IMX_PADCTL_CSI0_HSYNC_INDEX           179
#define IMX_PADCTL_CSI0_DATA_EN_INDEX         180
#define IMX_PADCTL_CSI0_VSYNC_INDEX           181
#define IMX_PADCTL_CSI0_DATA04_INDEX          182
#define IMX_PADCTL_CSI0_DATA05_INDEX          183
#define IMX_PADCTL_CSI0_DATA06_INDEX          184
#define IMX_PADCTL_CSI0_DATA07_INDEX          185
#define IMX_PADCTL_CSI0_DATA08_INDEX          186
#define IMX_PADCTL_CSI0_DATA09_INDEX          187
#define IMX_PADCTL_CSI0_DATA10_INDEX          188
#define IMX_PADCTL_CSI0_DATA11_INDEX          189
#define IMX_PADCTL_CSI0_DATA12_INDEX          190
#define IMX_PADCTL_CSI0_DATA13_INDEX          191
#define IMX_PADCTL_CSI0_DATA14_INDEX          192
#define IMX_PADCTL_CSI0_DATA15_INDEX          193
#define IMX_PADCTL_CSI0_DATA16_INDEX          194
#define IMX_PADCTL_CSI0_DATA17_INDEX          195
#define IMX_PADCTL_CSI0_DATA18_INDEX          196
#define IMX_PADCTL_CSI0_DATA19_INDEX          197
#define IMX_PADCTL_JTAG_TMS_INDEX             198
#define IMX_PADCTL_JTAG_MOD_INDEX             199
#define IMX_PADCTL_JTAG_TRSTB_INDEX           200
#define IMX_PADCTL_JTAG_TDI_INDEX             201
#define IMX_PADCTL_JTAG_TCK_INDEX             202
#define IMX_PADCTL_JTAG_TDO_INDEX             203
#define IMX_PADCTL_SD3_DATA7_INDEX            204
#define IMX_PADCTL_SD3_DATA6_INDEX            205
#define IMX_PADCTL_SD3_DATA5_INDEX            206
#define IMX_PADCTL_SD3_DATA4_INDEX            207
#define IMX_PADCTL_SD3_CMD_INDEX              208
#define IMX_PADCTL_SD3_CLK_INDEX              209
#define IMX_PADCTL_SD3_DATA0_INDEX            210
#define IMX_PADCTL_SD3_DATA1_INDEX            211
#define IMX_PADCTL_SD3_DATA2_INDEX            212
#define IMX_PADCTL_SD3_DATA3_INDEX            213
#define IMX_PADCTL_SD3_RESET_INDEX            214
#define IMX_PADCTL_NAND_CLE_INDEX             215
#define IMX_PADCTL_NAND_ALE_INDEX             216
#define IMX_PADCTL_NAND_WP_INDEX              217
#define IMX_PADCTL_NAND_READY_INDEX           218
#define IMX_PADCTL_NAND_CS0_INDEX             219
#define IMX_PADCTL_NAND_CS1_INDEX             220
#define IMX_PADCTL_NAND_CS2_INDEX             221
#define IMX_PADCTL_NAND_CS3_INDEX             222
#define IMX_PADCTL_SD4_CMD_INDEX              223
#define IMX_PADCTL_SD4_CLK_INDEX              224
#define IMX_PADCTL_NAND_DATA00_INDEX          225
#define IMX_PADCTL_NAND_DATA01_INDEX          226
#define IMX_PADCTL_NAND_DATA02_INDEX          227
#define IMX_PADCTL_NAND_DATA03_INDEX          228
#define IMX_PADCTL_NAND_DATA04_INDEX          229
#define IMX_PADCTL_NAND_DATA05_INDEX          230
#define IMX_PADCTL_NAND_DATA06_INDEX          231
#define IMX_PADCTL_NAND_DATA07_INDEX          232
#define IMX_PADCTL_SD4_DATA0_INDEX            233
#define IMX_PADCTL_SD4_DATA1_INDEX            234
#define IMX_PADCTL_SD4_DATA2_INDEX            235
#define IMX_PADCTL_SD4_DATA3_INDEX            236
#define IMX_PADCTL_SD4_DATA4_INDEX            237
#define IMX_PADCTL_SD4_DATA5_INDEX            238
#define IMX_PADCTL_SD4_DATA6_INDEX            239
#define IMX_PADCTL_SD4_DATA7_INDEX            240
#define IMX_PADCTL_SD1_DATA1_INDEX            241
#define IMX_PADCTL_SD1_DATA0_INDEX            242
#define IMX_PADCTL_SD1_DATA3_INDEX            243
#define IMX_PADCTL_SD1_CMD_INDEX              244
#define IMX_PADCTL_SD1_DATA2_INDEX            245
#define IMX_PADCTL_SD1_CLK_INDEX              246
#define IMX_PADCTL_SD2_CLK_INDEX              247
#define IMX_PADCTL_SD2_CMD_INDEX              248
#define IMX_PADCTL_SD2_DATA3_INDEX            249

#define IMX_PADCTL_NREGISTERS                 250

/* Pad Control Register Offsets */

#define IMX_PADCTL_OFFSET(n)                  (0x0360 + ((unsigned int)(n) << 2))

#define IMX_PADCTL_SD2_DATA1_OFFSET           0x0360
#define IMX_PADCTL_SD2_DATA2_OFFSET           0x0364
#define IMX_PADCTL_SD2_DATA0_OFFSET           0x0368
#define IMX_PADCTL_RGMII_TXC_OFFSET           0x036c
#define IMX_PADCTL_RGMII_TD0_OFFSET           0x0370
#define IMX_PADCTL_RGMII_TD1_OFFSET           0x0374
#define IMX_PADCTL_RGMII_TD2_OFFSET           0x0378
#define IMX_PADCTL_RGMII_TD3_OFFSET           0x037c
#define IMX_PADCTL_RGMII_RX_CTL_OFFSET        0x0380
#define IMX_PADCTL_RGMII_RD0_OFFSET           0x0384
#define IMX_PADCTL_RGMII_TX_CTL_OFFSET        0x0388
#define IMX_PADCTL_RGMII_RD1_OFFSET           0x038c
#define IMX_PADCTL_RGMII_RD2_OFFSET           0x0390
#define IMX_PADCTL_RGMII_RD3_OFFSET           0x0394
#define IMX_PADCTL_RGMII_RXC_OFFSET           0x0398
#define IMX_PADCTL_EIM_ADDR25_OFFSET          0x039c
#define IMX_PADCTL_EIM_EB2_OFFSET             0x03a0
#define IMX_PADCTL_EIM_DATA16_OFFSET          0x03a4
#define IMX_PADCTL_EIM_DATA17_OFFSET          0x03a8
#define IMX_PADCTL_EIM_DATA18_OFFSET          0x03ac
#define IMX_PADCTL_EIM_DATA19_OFFSET          0x03b0
#define IMX_PADCTL_EIM_DATA20_OFFSET          0x03b4
#define IMX_PADCTL_EIM_DATA21_OFFSET          0x03b8
#define IMX_PADCTL_EIM_DATA22_OFFSET          0x03bc
#define IMX_PADCTL_EIM_DATA23_OFFSET          0x03c0
#define IMX_PADCTL_EIM_EB3_OFFSET             0x03c4
#define IMX_PADCTL_EIM_DATA24_OFFSET          0x03c8
#define IMX_PADCTL_EIM_DATA25_OFFSET          0x03cc
#define IMX_PADCTL_EIM_DATA26_OFFSET          0x03d0
#define IMX_PADCTL_EIM_DATA27_OFFSET          0x03d4
#define IMX_PADCTL_EIM_DATA28_OFFSET          0x03d8
#define IMX_PADCTL_EIM_DATA29_OFFSET          0x03dc
#define IMX_PADCTL_EIM_DATA30_OFFSET          0x03e0
#define IMX_PADCTL_EIM_DATA31_OFFSET          0x03e4
#define IMX_PADCTL_EIM_ADDR24_OFFSET          0x03e8
#define IMX_PADCTL_EIM_ADDR23_OFFSET          0x03ec
#define IMX_PADCTL_EIM_ADDR22_OFFSET          0x03f0
#define IMX_PADCTL_EIM_ADDR21_OFFSET          0x03f4
#define IMX_PADCTL_EIM_ADDR20_OFFSET          0x03f8
#define IMX_PADCTL_EIM_ADDR19_OFFSET          0x03fc
#define IMX_PADCTL_EIM_ADDR18_OFFSET          0x0400
#define IMX_PADCTL_EIM_ADDR17_OFFSET          0x0404
#define IMX_PADCTL_EIM_ADDR16_OFFSET          0x0408
#define IMX_PADCTL_EIM_CS0_OFFSET             0x040c
#define IMX_PADCTL_EIM_CS1_OFFSET             0x0410
#define IMX_PADCTL_EIM_OE_OFFSET              0x0414
#define IMX_PADCTL_EIM_RW_OFFSET              0x0418
#define IMX_PADCTL_EIM_LBA_OFFSET             0x041c
#define IMX_PADCTL_EIM_EB0_OFFSET             0x0420
#define IMX_PADCTL_EIM_EB1_OFFSET             0x0424
#define IMX_PADCTL_EIM_AD00_OFFSET            0x0428
#define IMX_PADCTL_EIM_AD01_OFFSET            0x042c
#define IMX_PADCTL_EIM_AD02_OFFSET            0x0430
#define IMX_PADCTL_EIM_AD03_OFFSET            0x0434
#define IMX_PADCTL_EIM_AD04_OFFSET            0x0438
#define IMX_PADCTL_EIM_AD05_OFFSET            0x043c
#define IMX_PADCTL_EIM_AD06_OFFSET            0x0440
#define IMX_PADCTL_EIM_AD07_OFFSET            0x0444
#define IMX_PADCTL_EIM_AD08_OFFSET            0x0448
#define IMX_PADCTL_EIM_AD09_OFFSET            0x044c
#define IMX_PADCTL_EIM_AD10_OFFSET            0x0450
#define IMX_PADCTL_EIM_AD11_OFFSET            0x0454
#define IMX_PADCTL_EIM_AD12_OFFSET            0x0458
#define IMX_PADCTL_EIM_AD13_OFFSET            0x045c
#define IMX_PADCTL_EIM_AD14_OFFSET            0x0460
#define IMX_PADCTL_EIM_AD15_OFFSET            0x0464
#define IMX_PADCTL_EIM_WAIT_OFFSET            0x0468
#define IMX_PADCTL_EIM_BCLK_OFFSET            0x046c
#define IMX_PADCTL_DI0_DISP_CLK_OFFSET        0x0470
#define IMX_PADCTL_DI0_PIN15_OFFSET           0x0474
#define IMX_PADCTL_DI0_PIN02_OFFSET           0x0478
#define IMX_PADCTL_DI0_PIN03_OFFSET           0x047c
#define IMX_PADCTL_DI0_PIN04_OFFSET           0x0480
#define IMX_PADCTL_DISP0_DATA00_OFFSET        0x0484
#define IMX_PADCTL_DISP0_DATA01_OFFSET        0x0488
#define IMX_PADCTL_DISP0_DATA02_OFFSET        0x048c
#define IMX_PADCTL_DISP0_DATA03_OFFSET        0x0490
#define IMX_PADCTL_DISP0_DATA04_OFFSET        0x0494
#define IMX_PADCTL_DISP0_DATA05_OFFSET        0x0498
#define IMX_PADCTL_DISP0_DATA06_OFFSET        0x049c
#define IMX_PADCTL_DISP0_DATA07_OFFSET        0x04a0
#define IMX_PADCTL_DISP0_DATA08_OFFSET        0x04a4
#define IMX_PADCTL_DISP0_DATA09_OFFSET        0x04a8
#define IMX_PADCTL_DISP0_DATA10_OFFSET        0x04ac
#define IMX_PADCTL_DISP0_DATA11_OFFSET        0x04b0
#define IMX_PADCTL_DISP0_DATA12_OFFSET        0x04b4
#define IMX_PADCTL_DISP0_DATA13_OFFSET        0x04b8
#define IMX_PADCTL_DISP0_DATA14_OFFSET        0x04bc
#define IMX_PADCTL_DISP0_DATA15_OFFSET        0x04c0
#define IMX_PADCTL_DISP0_DATA16_OFFSET        0x04c4
#define IMX_PADCTL_DISP0_DATA17_OFFSET        0x04c8
#define IMX_PADCTL_DISP0_DATA18_OFFSET        0x04cc
#define IMX_PADCTL_DISP0_DATA19_OFFSET        0x04d0
#define IMX_PADCTL_DISP0_DATA20_OFFSET        0x04d4
#define IMX_PADCTL_DISP0_DATA21_OFFSET        0x04d8
#define IMX_PADCTL_DISP0_DATA22_OFFSET        0x04dc
#define IMX_PADCTL_DISP0_DATA23_OFFSET        0x04e0
#define IMX_PADCTL_ENET_MDIO_OFFSET           0x04e4
#define IMX_PADCTL_ENET_REF_CLK_OFFSET        0x04e8
#define IMX_PADCTL_ENET_RX_ER_OFFSET          0x04ec
#define IMX_PADCTL_ENET_CRS_DV_OFFSET         0x04f0
#define IMX_PADCTL_ENET_RX_DATA1_OFFSET       0x04f4
#define IMX_PADCTL_ENET_RX_DATA0_OFFSET       0x04f8
#define IMX_PADCTL_ENET_TX_EN_OFFSET          0x04fc
#define IMX_PADCTL_ENET_TX_DATA1_OFFSET       0x0500
#define IMX_PADCTL_ENET_TX_DATA0_OFFSET       0x0504
#define IMX_PADCTL_ENET_MDC_OFFSET            0x0508
#define IMX_PADCTL_DRAM_SDQS5_P_OFFSET        0x050c
#define IMX_PADCTL_DRAM_DQM5_OFFSET           0x0510
#define IMX_PADCTL_DRAM_DQM4_OFFSET           0x0514
#define IMX_PADCTL_DRAM_SDQS4_P_OFFSET        0x0518
#define IMX_PADCTL_DRAM_SDQS3_P_OFFSET        0x051c
#define IMX_PADCTL_DRAM_DQM3_OFFSET           0x0520
#define IMX_PADCTL_DRAM_SDQS2_P_OFFSET        0x0524
#define IMX_PADCTL_DRAM_DQM2_OFFSET           0x0528
#define IMX_PADCTL_DRAM_ADDR00_OFFSET         0x052c
#define IMX_PADCTL_DRAM_ADDR01_OFFSET         0x0530
#define IMX_PADCTL_DRAM_ADDR02_OFFSET         0x0534
#define IMX_PADCTL_DRAM_ADDR03_OFFSET         0x0538
#define IMX_PADCTL_DRAM_ADDR04_OFFSET         0x053c
#define IMX_PADCTL_DRAM_ADDR05_OFFSET         0x0540
#define IMX_PADCTL_DRAM_ADDR06_OFFSET         0x0544
#define IMX_PADCTL_DRAM_ADDR07_OFFSET         0x0548
#define IMX_PADCTL_DRAM_ADDR08_OFFSET         0x054c
#define IMX_PADCTL_DRAM_ADDR09_OFFSET         0x0550
#define IMX_PADCTL_DRAM_ADDR10_OFFSET         0x0554
#define IMX_PADCTL_DRAM_ADDR11_OFFSET         0x0558
#define IMX_PADCTL_DRAM_ADDR12_OFFSET         0x055c
#define IMX_PADCTL_DRAM_ADDR13_OFFSET         0x0560
#define IMX_PADCTL_DRAM_ADDR14_OFFSET         0x0564
#define IMX_PADCTL_DRAM_ADDR15_OFFSET         0x0568
#define IMX_PADCTL_DRAM_CAS_OFFSET            0x056c
#define IMX_PADCTL_DRAM_CS0_OFFSET            0x0570
#define IMX_PADCTL_DRAM_CS1_OFFSET            0x0574
#define IMX_PADCTL_DRAM_RAS_OFFSET            0x0578
#define IMX_PADCTL_DRAM_RESET_OFFSET          0x057c
#define IMX_PADCTL_DRAM_SDBA0_OFFSET          0x0580
#define IMX_PADCTL_DRAM_SDBA1_OFFSET          0x0584
#define IMX_PADCTL_DRAM_SDCLK0_P_OFFSET       0x0588
#define IMX_PADCTL_DRAM_SDBA2_OFFSET          0x058c
#define IMX_PADCTL_DRAM_SDCKE0_OFFSET         0x0590
#define IMX_PADCTL_DRAM_SDCLK1_P_OFFSET       0x0594
#define IMX_PADCTL_DRAM_SDCKE1_OFFSET         0x0598
#define IMX_PADCTL_DRAM_ODT0_OFFSET           0x059c
#define IMX_PADCTL_DRAM_ODT1_OFFSET           0x05a0
#define IMX_PADCTL_DRAM_SDWE_B_OFFSET         0x05a4
#define IMX_PADCTL_DRAM_SDQS0_P_OFFSET        0x05a8
#define IMX_PADCTL_DRAM_DQM0_OFFSET           0x05ac
#define IMX_PADCTL_DRAM_SDQS1_P_OFFSET        0x05b0
#define IMX_PADCTL_DRAM_DQM1_OFFSET           0x05b4
#define IMX_PADCTL_DRAM_SDQS6_P_OFFSET        0x05b8
#define IMX_PADCTL_DRAM_DQM6_OFFSET           0x05bc
#define IMX_PADCTL_DRAM_SDQS7_P_OFFSET        0x05c0
#define IMX_PADCTL_DRAM_DQM7_OFFSET           0x05c4
#define IMX_PADCTL_KEY_COL0_OFFSET            0x05c8
#define IMX_PADCTL_KEY_ROW0_OFFSET            0x05cc
#define IMX_PADCTL_KEY_COL1_OFFSET            0x05d0
#define IMX_PADCTL_KEY_ROW1_OFFSET            0x05d4
#define IMX_PADCTL_KEY_COL2_OFFSET            0x05d8
#define IMX_PADCTL_KEY_ROW2_OFFSET            0x05dc
#define IMX_PADCTL_KEY_COL3_OFFSET            0x05e0
#define IMX_PADCTL_KEY_ROW3_OFFSET            0x05e4
#define IMX_PADCTL_KEY_COL4_OFFSET            0x05e8
#define IMX_PADCTL_KEY_ROW4_OFFSET            0x05ec
#define IMX_PADCTL_GPIO00_OFFSET              0x05f0
#define IMX_PADCTL_GPIO01_OFFSET              0x05f4
#define IMX_PADCTL_GPIO09_OFFSET              0x05f8
#define IMX_PADCTL_GPIO03_OFFSET              0x05fc
#define IMX_PADCTL_GPIO06_OFFSET              0x0600
#define IMX_PADCTL_GPIO02_OFFSET              0x0604
#define IMX_PADCTL_GPIO04_OFFSET              0x0608
#define IMX_PADCTL_GPIO05_OFFSET              0x060c
#define IMX_PADCTL_GPIO07_OFFSET              0x0610
#define IMX_PADCTL_GPIO08_OFFSET              0x0614
#define IMX_PADCTL_GPIO16_OFFSET              0x0618
#define IMX_PADCTL_GPIO17_OFFSET              0x061c
#define IMX_PADCTL_GPIO18_OFFSET              0x0620
#define IMX_PADCTL_GPIO19_OFFSET              0x0624
#define IMX_PADCTL_CSI0_PIXCLK_OFFSET         0x0628
#define IMX_PADCTL_CSI0_HSYNC_OFFSET          0x062c
#define IMX_PADCTL_CSI0_DATA_EN_OFFSET        0x0630
#define IMX_PADCTL_CSI0_VSYNC_OFFSET          0x0634
#define IMX_PADCTL_CSI0_DATA04_OFFSET         0x0638
#define IMX_PADCTL_CSI0_DATA05_OFFSET         0x063c
#define IMX_PADCTL_CSI0_DATA06_OFFSET         0x0640
#define IMX_PADCTL_CSI0_DATA07_OFFSET         0x0644
#define IMX_PADCTL_CSI0_DATA08_OFFSET         0x0648
#define IMX_PADCTL_CSI0_DATA09_OFFSET         0x064c
#define IMX_PADCTL_CSI0_DATA10_OFFSET         0x0650
#define IMX_PADCTL_CSI0_DATA11_OFFSET         0x0654
#define IMX_PADCTL_CSI0_DATA12_OFFSET         0x0658
#define IMX_PADCTL_CSI0_DATA13_OFFSET         0x065c
#define IMX_PADCTL_CSI0_DATA14_OFFSET         0x0660
#define IMX_PADCTL_CSI0_DATA15_OFFSET         0x0664
#define IMX_PADCTL_CSI0_DATA16_OFFSET         0x0668
#define IMX_PADCTL_CSI0_DATA17_OFFSET         0x066c
#define IMX_PADCTL_CSI0_DATA18_OFFSET         0x0670
#define IMX_PADCTL_CSI0_DATA19_OFFSET         0x0674
#define IMX_PADCTL_JTAG_TMS_OFFSET            0x0678
#define IMX_PADCTL_JTAG_MOD_OFFSET            0x067c
#define IMX_PADCTL_JTAG_TRSTB_OFFSET          0x0680
#define IMX_PADCTL_JTAG_TDI_OFFSET            0x0684
#define IMX_PADCTL_JTAG_TCK_OFFSET            0x0688
#define IMX_PADCTL_JTAG_TDO_OFFSET            0x068c
#define IMX_PADCTL_SD3_DATA7_OFFSET           0x0690
#define IMX_PADCTL_SD3_DATA6_OFFSET           0x0694
#define IMX_PADCTL_SD3_DATA5_OFFSET           0x0698
#define IMX_PADCTL_SD3_DATA4_OFFSET           0x069c
#define IMX_PADCTL_SD3_CMD_OFFSET             0x06a0
#define IMX_PADCTL_SD3_CLK_OFFSET             0x06a4
#define IMX_PADCTL_SD3_DATA0_OFFSET           0x06a8
#define IMX_PADCTL_SD3_DATA1_OFFSET           0x06ac
#define IMX_PADCTL_SD3_DATA2_OFFSET           0x06b0
#define IMX_PADCTL_SD3_DATA3_OFFSET           0x06b4
#define IMX_PADCTL_SD3_RESET_OFFSET           0x06b8
#define IMX_PADCTL_NAND_CLE_OFFSET            0x06bc
#define IMX_PADCTL_NAND_ALE_OFFSET            0x06c0
#define IMX_PADCTL_NAND_WP_OFFSET             0x06c4
#define IMX_PADCTL_NAND_READY_OFFSET          0x06c8
#define IMX_PADCTL_NAND_CS0_OFFSET            0x06cc
#define IMX_PADCTL_NAND_CS1_OFFSET            0x06d0
#define IMX_PADCTL_NAND_CS2_OFFSET            0x06d4
#define IMX_PADCTL_NAND_CS3_OFFSET            0x06d8
#define IMX_PADCTL_SD4_CMD_OFFSET             0x06dc
#define IMX_PADCTL_SD4_CLK_OFFSET             0x06e0
#define IMX_PADCTL_NAND_DATA00_OFFSET         0x06e4
#define IMX_PADCTL_NAND_DATA01_OFFSET         0x06e8
#define IMX_PADCTL_NAND_DATA02_OFFSET         0x06ec
#define IMX_PADCTL_NAND_DATA03_OFFSET         0x06f0
#define IMX_PADCTL_NAND_DATA04_OFFSET         0x06f4
#define IMX_PADCTL_NAND_DATA05_OFFSET         0x06f8
#define IMX_PADCTL_NAND_DATA06_OFFSET         0x06fc
#define IMX_PADCTL_NAND_DATA07_OFFSET         0x0700
#define IMX_PADCTL_SD4_DATA0_OFFSET           0x0704
#define IMX_PADCTL_SD4_DATA1_OFFSET           0x0708
#define IMX_PADCTL_SD4_DATA2_OFFSET           0x070c
#define IMX_PADCTL_SD4_DATA3_OFFSET           0x0710
#define IMX_PADCTL_SD4_DATA4_OFFSET           0x0714
#define IMX_PADCTL_SD4_DATA5_OFFSET           0x0718
#define IMX_PADCTL_SD4_DATA6_OFFSET           0x071c
#define IMX_PADCTL_SD4_DATA7_OFFSET           0x0720
#define IMX_PADCTL_SD1_DATA1_OFFSET           0x0724
#define IMX_PADCTL_SD1_DATA0_OFFSET           0x0728
#define IMX_PADCTL_SD1_DATA3_OFFSET           0x072c
#define IMX_PADCTL_SD1_CMD_OFFSET             0x0730
#define IMX_PADCTL_SD1_DATA2_OFFSET           0x0734
#define IMX_PADCTL_SD1_CLK_OFFSET             0x0738
#define IMX_PADCTL_SD2_CLK_OFFSET             0x073c
#define IMX_PADCTL_SD2_CMD_OFFSET             0x0740
#define IMX_PADCTL_SD2_DATA3_OFFSET           0x0744

/* Pad Group Control Registers */

#define IMX_PADGROUP_B7DS_OFFSET              0x0748
#define IMX_PADGROUP_ADDDS_OFFSET             0x074c
#define IMX_PADGROUP_DDRMODE_CTL_OFFSET       0x0750
#define IMX_PADGROUP_TERM_CTL0_OFFSET         0x0754
#define IMX_PADGROUP_DDRPKE_OFFSET            0x0758
#define IMX_PADGROUP_TERM_CTL1_OFFSET         0x075c
#define IMX_PADGROUP_TERM_CTL2_OFFSET         0x0760
#define IMX_PADGROUP_TERM_CTL3_OFFSET         0x0764
#define IMX_PADGROUP_DDRPK_OFFSET             0x0768
#define IMX_PADGROUP_TERM_CTL4_OFFSET         0x076c
#define IMX_PADGROUP_DDRHYS_OFFSET            0x0770
#define IMX_PADGROUP_DDRMODE_OFFSET           0x0774
#define IMX_PADGROUP_TERM_CTL5_OFFSET         0x0778
#define IMX_PADGROUP_TERM_CTL6_OFFSET         0x077c
#define IMX_PADGROUP_TERM_CTL7_OFFSET         0x0780
#define IMX_PADGROUP_B0DS_OFFSET              0x0784
#define IMX_PADGROUP_B1DS_OFFSET              0x0788
#define IMX_PADGROUP_CTLDS_OFFSET             0x078c
#define IMX_PADGROUP_DDR_TYPE_RGMII_OFFSET    0x0790
#define IMX_PADGROUP_B2DS_OFFSET              0x0794
#define IMX_PADGROUP_DDR_TYPE_OFFSET          0x0798
#define IMX_PADGROUP_B3DS_OFFSET              0x079c
#define IMX_PADGROUP_B4DS_OFFSET              0x07a0
#define IMX_PADGROUP_B5DS_OFFSET              0x07a4
#define IMX_PADGROUP_B6DS_OFFSET              0x07a8
#define IMX_PADGROUP_RGMII_TERM_OFFSET        0x07ac

/* Select Input Registers */

#define IMX_INPUT_ASRC_ASRCK_CLOCK_6_OFFSET   0x07b0
#define IMX_INPUT_AUD4_INPUT_DA_AMX_OFFSET    0x07b4
#define IMX_INPUT_AUD4_INPUT_DB_AMX_OFFSET    0x07b8
#define IMX_INPUT_AUD4_INPUT_RXCLK_AMX_OFFSET 0x07bc
#define IMX_INPUT_AUD4_INPUT_RXFS_AMX_OFFSET  0x07c0
#define IMX_INPUT_AUD4_INPUT_TXCLK_AMX_OFFSET 0x07c4
#define IMX_INPUT_AUD4_INPUT_TXFS_AMX_OFFSET  0x07c8
#define IMX_INPUT_AUD5_INPUT_DA_AMX_OFFSET    0x07cc
#define IMX_INPUT_AUD5_INPUT_DB_AMX_OFFSET    0x07d0
#define IMX_INPUT_AUD5_INPUT_RXCLK_AMX_OFFSET 0x07d4
#define IMX_INPUT_AUD5_INPUT_RXFS_AMX_OFFSET  0x07d8
#define IMX_INPUT_AUD5_INPUT_TXCLK_AMX_OFFSET 0x07dc
#define IMX_INPUT_AUD5_INPUT_TXFS_AMX_OFFSET  0x07e0
#define IMX_INPUT_FLEXCAN1_RX_OFFSET          0x07e4
#define IMX_INPUT_FLEXCAN2_RX_OFFSET          0x07e8
#define IMX_INPUT_CCM_PMIC_READY_OFFSET       0x07e0
#define IMX_INPUT_ECSPI1_CSPI_CLK_IN_OFFSET   0x07f4
#define IMX_INPUT_ECSPI1_MISO_OFFSET          0x07f8
#define IMX_INPUT_ECSPI1_MOSI_OFFSET          0x07fc
#define IMX_INPUT_ECSPI1_SS0_OFFSET           0x0800
#define IMX_INPUT_ECSPI1_SS1_OFFSET           0x0804
#define IMX_INPUT_ECSPI1_SS2_OFFSET           0x0808
#define IMX_INPUT_ECSPI1_SS3_OFFSET           0x080c
#define IMX_INPUT_ECSPI2_CSPI_CLK_IN_OFFSET   0x0810
#define IMX_INPUT_ECSPI2_MISO_OFFSET          0x0814
#define IMX_INPUT_ECSPI2_MOSI_OFFSET          0x0818
#define IMX_INPUT_ECSPI2_SS0_OFFSET           0x081c
#define IMX_INPUT_ECSPI2_SS1_OFFSET           0x0820
#define IMX_INPUT_ECSPI4_SS0_OFFSET           0x0824
#define IMX_INPUT_ECSPI5_CSPI_CLK_IN_OFFSET   0x0828
#define IMX_INPUT_ECSPI5_MISO_OFFSET          0x082c
#define IMX_INPUT_ECSPI5_MOSI_OFFSET          0x0830
#define IMX_INPUT_ECSPI5_SS0_OFFSET           0x0834
#define IMX_INPUT_ECSPI5_SS1_OFFSET           0x0838
#define IMX_INPUT_ENET_REF_CLK_OFFSET         0x083c
#define IMX_INPUT_ENET_MAC0_MDIO_OFFSET       0x0840
#define IMX_INPUT_ENET_MAC0_RX_CLK_OFFSET     0x0844
#define IMX_INPUT_ENET_MAC0_RX_DATA0_OFFSET   0x0848
#define IMX_INPUT_ENET_MAC0_RX_DATA1_OFFSET   0x084c
#define IMX_INPUT_ENET_MAC0_RX_DATA2_OFFSET   0x0850
#define IMX_INPUT_ENET_MAC0_RX_DATA3_OFFSET   0x0854
#define IMX_INPUT_ENET_MAC0_RX_EN_OFFSET      0x0858
#define IMX_INPUT_ESAI_RX_FS_OFFSET           0x085c
#define IMX_INPUT_ESAI_TX_FS_OFFSET           0x0860
#define IMX_INPUT_ESAI_RX_HF_CLK_OFFSET       0x0864
#define IMX_INPUT_ESAI_TX_HF_CLK_OFFSET       0x0868
#define IMX_INPUT_ESAI_RX_CLK_OFFSET          0x086c
#define IMX_INPUT_ESAI_TX_CLK_OFFSET          0x0870
#define IMX_INPUT_ESAI_SDO0_OFFSET            0x0874
#define IMX_INPUT_ESAI_SDO1_OFFSET            0x0878
#define IMX_INPUT_ESAI_SDO2_SDI3_OFFSET       0x087c
#define IMX_INPUT_ESAI_SDO3_SDI2_OFFSET       0x0880
#define IMX_INPUT_ESAI_SDO4_SDI1_OFFSET       0x0884
#define IMX_INPUT_ESAI_SDO5_SDI0_OFFSET       0x0888
#define IMX_INPUT_HDMI_ICECIN_OFFSET          0x088c
#define IMX_INPUT_HDMI_II2C_CLKIN_OFFSET      0x0890
#define IMX_INPUT_HDMI_II2C_DATAIN_OFFSET     0x0894
#define IMX_INPUT_I2C1_SCL_IN_OFFSET          0x0898
#define IMX_INPUT_I2C1_SDA_IN_OFFSET          0x089c
#define IMX_INPUT_I2C2_SCL_IN_OFFSET          0x08a0
#define IMX_INPUT_I2C2_SDA_IN_OFFSET          0x08a4
#define IMX_INPUT_I2C3_SCL_IN_OFFSET          0x08a8
#define IMX_INPUT_I2C3_SDA_IN_OFFSET          0x08ac
#define IMX_INPUT_IPU2_SENS1_DATA10_OFFSET    0x08b0
#define IMX_INPUT_IPU2_SENS1_DATA11_OFFSET    0x08b4
#define IMX_INPUT_IPU2_SENS1_DATA12_OFFSET    0x08b8
#define IMX_INPUT_IPU2_SENS1_DATA13_OFFSET    0x08bc
#define IMX_INPUT_IPU2_SENS1_DATA14_OFFSET    0x08c0
#define IMX_INPUT_IPU2_SENS1_DATA15_OFFSET    0x08c4
#define IMX_INPUT_IPU2_SENS1_DATA16_OFFSET    0x08c8
#define IMX_INPUT_IPU2_SENS1_DATA17_OFFSET    0x08cc
#define IMX_INPUT_IPU2_SENS1_DATA18_OFFSET    0x08d0
#define IMX_INPUT_IPU2_SENS1_DATA19_OFFSET    0x08d4
#define IMX_INPUT_IPU2_SENS1_DATA_EN_OFFSET   0x08d8
#define IMX_INPUT_IPU2_SENS1_HSYNC_OFFSET     0x08dc
#define IMX_INPUT_IPU2_SENS1_PIX_CLK_OFFSET   0x08e0
#define IMX_INPUT_IPU2_SENS1_VSYNC_OFFSET     0x08e4
#define IMX_INPUT_KEY_COL5_OFFSET             0x08e8
#define IMX_INPUT_KEY_COL6_OFFSET             0x08ec
#define IMX_INPUT_KEY_COL7_OFFSET             0x08f0
#define IMX_INPUT_KEY_ROW5_OFFSET             0x08f4
#define IMX_INPUT_KEY_ROW6_OFFSET             0x08f8
#define IMX_INPUT_KEY_ROW7_OFFSET             0x08fc
#define IMX_INPUT_MLB_MLB_CLK_IN_OFFSET       0x0900
#define IMX_INPUT_MLB_MLB_DATA_IN_OFFSET      0x0904
#define IMX_INPUT_MLB_MLB_SIG_IN_OFFSET       0x0908
#define IMX_INPUT_SDMA_EVENTS14_OFFSET        0x090c
#define IMX_INPUT_SDMA_EVENTS47_OFFSET        0x0910
#define IMX_INPUT_SPDIF_SPDIF_IN1_OFFSET      0x0914
#define IMX_INPUT_SPDIF_TX_CLK2_OFFSET        0x0918
#define IMX_INPUT_UART1_UART_RTS_B_OFFSET     0x091c
#define IMX_INPUT_UART1_UART_RX_DATA_OFFSET   0x0920
#define IMX_INPUT_UART2_UART_RTS_B_OFFSET     0x0924
#define IMX_INPUT_UART2_UART_RX_DATA_OFFSET   0x0928
#define IMX_INPUT_UART3_UART_RTS_B_OFFSET     0x092c
#define IMX_INPUT_UART3_UART_RX_DATA_OFFSET   0x0930
#define IMX_INPUT_UART4_UART_RTS_B_OFFSET     0x0934
#define IMX_INPUT_UART4_UART_RX_DATA_OFFSET   0x0938
#define IMX_INPUT_UART5_UART_RTS_B_OFFSET     0x093c
#define IMX_INPUT_UART5_UART_RX_DATA_OFFSET   0x0940
#define IMX_INPUT_USB_OTG_OC_OFFSET           0x0944
#define IMX_INPUT_USB_H1_OC_OFFSET            0x0948
#define IMX_INPUT_USDHC1_WP_ON_OFFSET         0x094c

/* IOMUXC Register Addresses ********************************************************/
/* General Purpose Registers */

#define IMX_IOMUXC_GPR0                       (IMX_IOMUXC_VBASE+IMX_IOMUXC_GPR0_OFFSET)
#define IMX_IOMUXC_GPR1                       (IMX_IOMUXC_VBASE+IMX_IOMUXC_GPR1_OFFSET)
#define IMX_IOMUXC_GPR2                       (IMX_IOMUXC_VBASE+IMX_IOMUXC_GPR2_OFFSET)
#define IMX_IOMUXC_GPR3                       (IMX_IOMUXC_VBASE+IMX_IOMUXC_GPR3_OFFSET)
#define IMX_IOMUXC_GPR4                       (IMX_IOMUXC_VBASE+IMX_IOMUXC_GPR4_OFFSET)
#define IMX_IOMUXC_GPR5                       (IMX_IOMUXC_VBASE+IMX_IOMUXC_GPR5_OFFSET)
#define IMX_IOMUXC_GPR6                       (IMX_IOMUXC_VBASE+IMX_IOMUXC_GPR6_OFFSET)
#define IMX_IOMUXC_GPR7                       (IMX_IOMUXC_VBASE+IMX_IOMUXC_GPR7_OFFSET)
#define IMX_IOMUXC_GPR8                       (IMX_IOMUXC_VBASE+IMX_IOMUXC_GPR8_OFFSET)
#define IMX_IOMUXC_GPR9                       (IMX_IOMUXC_VBASE+IMX_IOMUXC_GPR9_OFFSET)
#define IMX_IOMUXC_GPR10                      (IMX_IOMUXC_VBASE+IMX_IOMUXC_GPR10_OFFSET)
#define IMX_IOMUXC_GPR11                      (IMX_IOMUXC_VBASE+IMX_IOMUXC_GPR11_OFFSET)
#define IMX_IOMUXC_GPR12                      (IMX_IOMUXC_VBASE+IMX_IOMUXC_GPR12_OFFSET)
#define IMX_IOMUXC_GPR13                      (IMX_IOMUXC_VBASE+IMX_IOMUXC_GPR13_OFFSET)

/* Pad Mux Registers */

#define IMX_PADMUX_ADDRESS(n)                 (IMX_IOMUXC_VBASE+IMX_PADMUX_OFFSET(n))

#define IMX_PADMUX_SD2_DATA1                  (IMX_IOMUXC_VBASE+IMX_PADMUX_SD2_DATA1_OFFSET)
#define IMX_PADMUX_SD2_DATA2                  (IMX_IOMUXC_VBASE+IMX_PADMUX_SD2_DATA2_OFFSET)
#define IMX_PADMUX_SD2_DATA0                  (IMX_IOMUXC_VBASE+IMX_PADMUX_SD2_DATA0_OFFSET)
#define IMX_PADMUX_RGMII_TXC                  (IMX_IOMUXC_VBASE+IMX_PADMUX_RGMII_TXC_OFFSET)
#define IMX_PADMUX_RGMII_TD0                  (IMX_IOMUXC_VBASE+IMX_PADMUX_RGMII_TD0_OFFSET)
#define IMX_PADMUX_RGMII_TD1                  (IMX_IOMUXC_VBASE+IMX_PADMUX_RGMII_TD1_OFFSET)
#define IMX_PADMUX_RGMII_TD2                  (IMX_IOMUXC_VBASE+IMX_PADMUX_RGMII_TD2_OFFSET)
#define IMX_PADMUX_RGMII_TD3                  (IMX_IOMUXC_VBASE+IMX_PADMUX_RGMII_TD3_OFFSET)
#define IMX_PADMUX_RGMII_RX_CTL               (IMX_IOMUXC_VBASE+IMX_PADMUX_RGMII_RX_CTL_OFFSET)
#define IMX_PADMUX_RGMII_RD0                  (IMX_IOMUXC_VBASE+IMX_PADMUX_RGMII_RD0_OFFSET)
#define IMX_PADMUX_RGMII_TX_CTL               (IMX_IOMUXC_VBASE+IMX_PADMUX_RGMII_TX_CTL_OFFSET)
#define IMX_PADMUX_RGMII_RD1                  (IMX_IOMUXC_VBASE+IMX_PADMUX_RGMII_RD1_OFFSET)
#define IMX_PADMUX_RGMII_RD2                  (IMX_IOMUXC_VBASE+IMX_PADMUX_RGMII_RD2_OFFSET)
#define IMX_PADMUX_RGMII_RD3                  (IMX_IOMUXC_VBASE+IMX_PADMUX_RGMII_RD3_OFFSET)
#define IMX_PADMUX_RGMII_RXC                  (IMX_IOMUXC_VBASE+IMX_PADMUX_RGMII_RXC_OFFSET)
#define IMX_PADMUX_EIM_ADDR25                 (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_ADDR25_OFFSET)
#define IMX_PADMUX_EIM_EB2                    (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_EB2_OFFSET)
#define IMX_PADMUX_EIM_DATA16                 (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_DATA16_OFFSET)
#define IMX_PADMUX_EIM_DATA17                 (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_DATA17_OFFSET)
#define IMX_PADMUX_EIM_DATA18                 (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_DATA18_OFFSET)
#define IMX_PADMUX_EIM_DATA19                 (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_DATA19_OFFSET)
#define IMX_PADMUX_EIM_DATA20                 (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_DATA20_OFFSET)
#define IMX_PADMUX_EIM_DATA21                 (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_DATA21_OFFSET)
#define IMX_PADMUX_EIM_DATA22                 (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_DATA22_OFFSET)
#define IMX_PADMUX_EIM_DATA23                 (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_DATA23_OFFSET)
#define IMX_PADMUX_EIM_EB3                    (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_EB3_OFFSET)
#define IMX_PADMUX_EIM_DATA24                 (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_DATA24_OFFSET)
#define IMX_PADMUX_EIM_DATA25                 (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_DATA25_OFFSET)
#define IMX_PADMUX_EIM_DATA26                 (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_DATA26_OFFSET)
#define IMX_PADMUX_EIM_DATA27                 (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_DATA27_OFFSET)
#define IMX_PADMUX_EIM_DATA28                 (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_DATA28_OFFSET)
#define IMX_PADMUX_EIM_DATA29                 (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_DATA29_OFFSET)
#define IMX_PADMUX_EIM_DATA30                 (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_DATA30_OFFSET)
#define IMX_PADMUX_EIM_DATA31                 (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_DATA31_OFFSET)
#define IMX_PADMUX_EIM_ADDR24                 (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_ADDR24_OFFSET)
#define IMX_PADMUX_EIM_ADDR23                 (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_ADDR23_OFFSET)
#define IMX_PADMUX_EIM_ADDR22                 (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_ADDR22_OFFSET)
#define IMX_PADMUX_EIM_ADDR21                 (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_ADDR21_OFFSET)
#define IMX_PADMUX_EIM_ADDR20                 (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_ADDR20_OFFSET)
#define IMX_PADMUX_EIM_ADDR19                 (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_ADDR19_OFFSET)
#define IMX_PADMUX_EIM_ADDR18                 (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_ADDR18_OFFSET)
#define IMX_PADMUX_EIM_ADDR17                 (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_ADDR17_OFFSET)
#define IMX_PADMUX_EIM_ADDR16                 (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_ADDR16_OFFSET)
#define IMX_PADMUX_EIM_CS0                    (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_CS0_OFFSET)
#define IMX_PADMUX_EIM_CS1                    (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_CS1_OFFSET)
#define IMX_PADMUX_EIM_OE                     (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_OE_OFFSET)
#define IMX_PADMUX_EIM_RW                     (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_RW_OFFSET)
#define IMX_PADMUX_EIM_LBA                    (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_LBA_OFFSET)
#define IMX_PADMUX_EIM_EB0                    (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_EB0_OFFSET)
#define IMX_PADMUX_EIM_EB1                    (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_EB1_OFFSET)
#define IMX_PADMUX_EIM_AD00                   (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_AD00_OFFSET)
#define IMX_PADMUX_EIM_AD01                   (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_AD01_OFFSET)
#define IMX_PADMUX_EIM_AD02                   (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_AD02_OFFSET)
#define IMX_PADMUX_EIM_AD03                   (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_AD03_OFFSET)
#define IMX_PADMUX_EIM_AD04                   (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_AD04_OFFSET)
#define IMX_PADMUX_EIM_AD05                   (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_AD05_OFFSET)
#define IMX_PADMUX_EIM_AD06                   (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_AD06_OFFSET)
#define IMX_PADMUX_EIM_AD07                   (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_AD07_OFFSET)
#define IMX_PADMUX_EIM_AD08                   (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_AD08_OFFSET)
#define IMX_PADMUX_EIM_AD09                   (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_AD09_OFFSET)
#define IMX_PADMUX_EIM_AD10                   (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_AD10_OFFSET)
#define IMX_PADMUX_EIM_AD11                   (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_AD11_OFFSET)
#define IMX_PADMUX_EIM_AD12                   (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_AD12_OFFSET)
#define IMX_PADMUX_EIM_AD13                   (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_AD13_OFFSET)
#define IMX_PADMUX_EIM_AD14                   (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_AD14_OFFSET)
#define IMX_PADMUX_EIM_AD15                   (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_AD15_OFFSET)
#define IMX_PADMUX_EIM_WAIT                   (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_WAIT_OFFSET)
#define IMX_PADMUX_EIM_BCLK                   (IMX_IOMUXC_VBASE+IMX_PADMUX_EIM_BCLK_OFFSET)
#define IMX_PADMUX_DI0_DISP_CLK               (IMX_IOMUXC_VBASE+IMX_PADMUX_DI0_DISP_CLK_OFFSET)
#define IMX_PADMUX_DI0_PIN15                  (IMX_IOMUXC_VBASE+IMX_PADMUX_DI0_PIN15_OFFSET)
#define IMX_PADMUX_DI0_PIN02                  (IMX_IOMUXC_VBASE+IMX_PADMUX_DI0_PIN02_OFFSET)
#define IMX_PADMUX_DI0_PIN03                  (IMX_IOMUXC_VBASE+IMX_PADMUX_DI0_PIN03_OFFSET)
#define IMX_PADMUX_DI0_PIN04                  (IMX_IOMUXC_VBASE+IMX_PADMUX_DI0_PIN04_OFFSET)
#define IMX_PADMUX_DISP0_DATA00               (IMX_IOMUXC_VBASE+IMX_PADMUX_DISP0_DATA00_OFFSET)
#define IMX_PADMUX_DISP0_DATA01               (IMX_IOMUXC_VBASE+IMX_PADMUX_DISP0_DATA01_OFFSET)
#define IMX_PADMUX_DISP0_DATA02               (IMX_IOMUXC_VBASE+IMX_PADMUX_DISP0_DATA02_OFFSET)
#define IMX_PADMUX_DISP0_DATA03               (IMX_IOMUXC_VBASE+IMX_PADMUX_DISP0_DATA03_OFFSET)
#define IMX_PADMUX_DISP0_DATA04               (IMX_IOMUXC_VBASE+IMX_PADMUX_DISP0_DATA04_OFFSET)
#define IMX_PADMUX_DISP0_DATA05               (IMX_IOMUXC_VBASE+IMX_PADMUX_DISP0_DATA05_OFFSET)
#define IMX_PADMUX_DISP0_DATA06               (IMX_IOMUXC_VBASE+IMX_PADMUX_DISP0_DATA06_OFFSET)
#define IMX_PADMUX_DISP0_DATA07               (IMX_IOMUXC_VBASE+IMX_PADMUX_DISP0_DATA07_OFFSET)
#define IMX_PADMUX_DISP0_DATA08               (IMX_IOMUXC_VBASE+IMX_PADMUX_DISP0_DATA08_OFFSET)
#define IMX_PADMUX_DISP0_DATA09               (IMX_IOMUXC_VBASE+IMX_PADMUX_DISP0_DATA09_OFFSET)
#define IMX_PADMUX_DISP0_DATA10               (IMX_IOMUXC_VBASE+IMX_PADMUX_DISP0_DATA10_OFFSET)
#define IMX_PADMUX_DISP0_DATA11               (IMX_IOMUXC_VBASE+IMX_PADMUX_DISP0_DATA11_OFFSET)
#define IMX_PADMUX_DISP0_DATA12               (IMX_IOMUXC_VBASE+IMX_PADMUX_DISP0_DATA12_OFFSET)
#define IMX_PADMUX_DISP0_DATA13               (IMX_IOMUXC_VBASE+IMX_PADMUX_DISP0_DATA13_OFFSET)
#define IMX_PADMUX_DISP0_DATA14               (IMX_IOMUXC_VBASE+IMX_PADMUX_DISP0_DATA14_OFFSET)
#define IMX_PADMUX_DISP0_DATA15               (IMX_IOMUXC_VBASE+IMX_PADMUX_DISP0_DATA15_OFFSET)
#define IMX_PADMUX_DISP0_DATA16               (IMX_IOMUXC_VBASE+IMX_PADMUX_DISP0_DATA16_OFFSET)
#define IMX_PADMUX_DISP0_DATA17               (IMX_IOMUXC_VBASE+IMX_PADMUX_DISP0_DATA17_OFFSET)
#define IMX_PADMUX_DISP0_DATA18               (IMX_IOMUXC_VBASE+IMX_PADMUX_DISP0_DATA18_OFFSET)
#define IMX_PADMUX_DISP0_DATA19               (IMX_IOMUXC_VBASE+IMX_PADMUX_DISP0_DATA19_OFFSET)
#define IMX_PADMUX_DISP0_DATA20               (IMX_IOMUXC_VBASE+IMX_PADMUX_DISP0_DATA20_OFFSET)
#define IMX_PADMUX_DISP0_DATA21               (IMX_IOMUXC_VBASE+IMX_PADMUX_DISP0_DATA21_OFFSET)
#define IMX_PADMUX_DISP0_DATA22               (IMX_IOMUXC_VBASE+IMX_PADMUX_DISP0_DATA22_OFFSET)
#define IMX_PADMUX_DISP0_DATA23               (IMX_IOMUXC_VBASE+IMX_PADMUX_DISP0_DATA23_OFFSET)
#define IMX_PADMUX_ENET_MDIO                  (IMX_IOMUXC_VBASE+IMX_PADMUX_ENET_MDIO_OFFSET)
#define IMX_PADMUX_ENET_REF_CLK               (IMX_IOMUXC_VBASE+IMX_PADMUX_ENET_REF_CLK_OFFSET)
#define IMX_PADMUX_ENET_RX_ER                 (IMX_IOMUXC_VBASE+IMX_PADMUX_ENET_RX_ER_OFFSET)
#define IMX_PADMUX_ENET_CRS_DV                (IMX_IOMUXC_VBASE+IMX_PADMUX_ENET_CRS_DV_OFFSET)
#define IMX_PADMUX_ENET_RX_DATA1              (IMX_IOMUXC_VBASE+IMX_PADMUX_ENET_RX_DATA1_OFFSET)
#define IMX_PADMUX_ENET_RX_DATA0              (IMX_IOMUXC_VBASE+IMX_PADMUX_ENET_RX_DATA0_OFFSET)
#define IMX_PADMUX_ENET_TX_EN                 (IMX_IOMUXC_VBASE+IMX_PADMUX_ENET_TX_EN_OFFSET)
#define IMX_PADMUX_ENET_TX_DATA1              (IMX_IOMUXC_VBASE+IMX_PADMUX_ENET_TX_DATA1_OFFSET)
#define IMX_PADMUX_ENET_TX_DATA0              (IMX_IOMUXC_VBASE+IMX_PADMUX_ENET_TX_DATA0_OFFSET)
#define IMX_PADMUX_ENET_MDC                   (IMX_IOMUXC_VBASE+IMX_PADMUX_ENET_MDC_OFFSET)
#define IMX_PADMUX_KEY_COL0                   (IMX_IOMUXC_VBASE+IMX_PADMUX_KEY_COL0_OFFSET)
#define IMX_PADMUX_KEY_ROW0                   (IMX_IOMUXC_VBASE+IMX_PADMUX_KEY_ROW0_OFFSET)
#define IMX_PADMUX_KEY_COL1                   (IMX_IOMUXC_VBASE+IMX_PADMUX_KEY_COL1_OFFSET)
#define IMX_PADMUX_KEY_ROW1                   (IMX_IOMUXC_VBASE+IMX_PADMUX_KEY_ROW1_OFFSET)
#define IMX_PADMUX_KEY_COL2                   (IMX_IOMUXC_VBASE+IMX_PADMUX_KEY_COL2_OFFSET)
#define IMX_PADMUX_KEY_ROW2                   (IMX_IOMUXC_VBASE+IMX_PADMUX_KEY_ROW2_OFFSET)
#define IMX_PADMUX_KEY_COL3                   (IMX_IOMUXC_VBASE+IMX_PADMUX_KEY_COL3_OFFSET)
#define IMX_PADMUX_KEY_ROW3                   (IMX_IOMUXC_VBASE+IMX_PADMUX_KEY_ROW3_OFFSET)
#define IMX_PADMUX_KEY_COL4                   (IMX_IOMUXC_VBASE+IMX_PADMUX_KEY_COL4_OFFSET)
#define IMX_PADMUX_KEY_ROW4                   (IMX_IOMUXC_VBASE+IMX_PADMUX_KEY_ROW4_OFFSET)
#define IMX_PADMUX_GPIO00                     (IMX_IOMUXC_VBASE+IMX_PADMUX_GPIO00_OFFSET)
#define IMX_PADMUX_GPIO01                     (IMX_IOMUXC_VBASE+IMX_PADMUX_GPIO01_OFFSET)
#define IMX_PADMUX_GPIO09                     (IMX_IOMUXC_VBASE+IMX_PADMUX_GPIO09_OFFSET)
#define IMX_PADMUX_GPIO03                     (IMX_IOMUXC_VBASE+IMX_PADMUX_GPIO03_OFFSET)
#define IMX_PADMUX_GPIO06                     (IMX_IOMUXC_VBASE+IMX_PADMUX_GPIO06_OFFSET)
#define IMX_PADMUX_GPIO02                     (IMX_IOMUXC_VBASE+IMX_PADMUX_GPIO02_OFFSET)
#define IMX_PADMUX_GPIO04                     (IMX_IOMUXC_VBASE+IMX_PADMUX_GPIO04_OFFSET)
#define IMX_PADMUX_GPIO05                     (IMX_IOMUXC_VBASE+IMX_PADMUX_GPIO05_OFFSET)
#define IMX_PADMUX_GPIO07                     (IMX_IOMUXC_VBASE+IMX_PADMUX_GPIO07_OFFSET)
#define IMX_PADMUX_GPIO08                     (IMX_IOMUXC_VBASE+IMX_PADMUX_GPIO08_OFFSET)
#define IMX_PADMUX_GPIO16                     (IMX_IOMUXC_VBASE+IMX_PADMUX_GPIO16_OFFSET)
#define IMX_PADMUX_GPIO17                     (IMX_IOMUXC_VBASE+IMX_PADMUX_GPIO17_OFFSET)
#define IMX_PADMUX_GPIO18                     (IMX_IOMUXC_VBASE+IMX_PADMUX_GPIO18_OFFSET)
#define IMX_PADMUX_GPIO19                     (IMX_IOMUXC_VBASE+IMX_PADMUX_GPIO19_OFFSET)
#define IMX_PADMUX_CSI0_PIXCLK                (IMX_IOMUXC_VBASE+IMX_PADMUX_CSI0_PIXCLK_OFFSET)
#define IMX_PADMUX_CSI0_HSYNC                 (IMX_IOMUXC_VBASE+IMX_PADMUX_CSI0_HSYNC_OFFSET)
#define IMX_PADMUX_CSI0_DATA_EN               (IMX_IOMUXC_VBASE+IMX_PADMUX_CSI0_DATA_EN_OFFSET)
#define IMX_PADMUX_CSI0_VSYNC                 (IMX_IOMUXC_VBASE+IMX_PADMUX_CSI0_VSYNC_OFFSET)
#define IMX_PADMUX_CSI0_DATA04                (IMX_IOMUXC_VBASE+IMX_PADMUX_CSI0_DATA04_OFFSET)
#define IMX_PADMUX_CSI0_DATA05                (IMX_IOMUXC_VBASE+IMX_PADMUX_CSI0_DATA05_OFFSET)
#define IMX_PADMUX_CSI0_DATA06                (IMX_IOMUXC_VBASE+IMX_PADMUX_CSI0_DATA06_OFFSET)
#define IMX_PADMUX_CSI0_DATA07                (IMX_IOMUXC_VBASE+IMX_PADMUX_CSI0_DATA07_OFFSET)
#define IMX_PADMUX_CSI0_DATA08                (IMX_IOMUXC_VBASE+IMX_PADMUX_CSI0_DATA08_OFFSET)
#define IMX_PADMUX_CSI0_DATA09                (IMX_IOMUXC_VBASE+IMX_PADMUX_CSI0_DATA09_OFFSET)
#define IMX_PADMUX_CSI0_DATA10                (IMX_IOMUXC_VBASE+IMX_PADMUX_CSI0_DATA10_OFFSET)
#define IMX_PADMUX_CSI0_DATA11                (IMX_IOMUXC_VBASE+IMX_PADMUX_CSI0_DATA11_OFFSET)
#define IMX_PADMUX_CSI0_DATA12                (IMX_IOMUXC_VBASE+IMX_PADMUX_CSI0_DATA12_OFFSET)
#define IMX_PADMUX_CSI0_DATA13                (IMX_IOMUXC_VBASE+IMX_PADMUX_CSI0_DATA13_OFFSET)
#define IMX_PADMUX_CSI0_DATA14                (IMX_IOMUXC_VBASE+IMX_PADMUX_CSI0_DATA14_OFFSET)
#define IMX_PADMUX_CSI0_DATA15                (IMX_IOMUXC_VBASE+IMX_PADMUX_CSI0_DATA15_OFFSET)
#define IMX_PADMUX_CSI0_DATA16                (IMX_IOMUXC_VBASE+IMX_PADMUX_CSI0_DATA16_OFFSET)
#define IMX_PADMUX_CSI0_DATA17                (IMX_IOMUXC_VBASE+IMX_PADMUX_CSI0_DATA17_OFFSET)
#define IMX_PADMUX_CSI0_DATA18                (IMX_IOMUXC_VBASE+IMX_PADMUX_CSI0_DATA18_OFFSET)
#define IMX_PADMUX_CSI0_DATA19                (IMX_IOMUXC_VBASE+IMX_PADMUX_CSI0_DATA19_OFFSET)
#define IMX_PADMUX_SD3_DATA7                  (IMX_IOMUXC_VBASE+IMX_PADMUX_SD3_DATA7_OFFSET)
#define IMX_PADMUX_SD3_DATA6                  (IMX_IOMUXC_VBASE+IMX_PADMUX_SD3_DATA6_OFFSET)
#define IMX_PADMUX_SD3_DATA5                  (IMX_IOMUXC_VBASE+IMX_PADMUX_SD3_DATA5_OFFSET)
#define IMX_PADMUX_SD3_DATA4                  (IMX_IOMUXC_VBASE+IMX_PADMUX_SD3_DATA4_OFFSET)
#define IMX_PADMUX_SD3_CMD                    (IMX_IOMUXC_VBASE+IMX_PADMUX_SD3_CMD_OFFSET)
#define IMX_PADMUX_SD3_CLK                    (IMX_IOMUXC_VBASE+IMX_PADMUX_SD3_CLK_OFFSET)
#define IMX_PADMUX_SD3_DATA0                  (IMX_IOMUXC_VBASE+IMX_PADMUX_SD3_DATA0_OFFSET)
#define IMX_PADMUX_SD3_DATA1                  (IMX_IOMUXC_VBASE+IMX_PADMUX_SD3_DATA1_OFFSET)
#define IMX_PADMUX_SD3_DATA2                  (IMX_IOMUXC_VBASE+IMX_PADMUX_SD3_DATA2_OFFSET)
#define IMX_PADMUX_SD3_DATA3                  (IMX_IOMUXC_VBASE+IMX_PADMUX_SD3_DATA3_OFFSET)
#define IMX_PADMUX_SD3_RESET                  (IMX_IOMUXC_VBASE+IMX_PADMUX_SD3_RESET_OFFSET)
#define IMX_PADMUX_NAND_CLE                   (IMX_IOMUXC_VBASE+IMX_PADMUX_NAND_CLE_OFFSET)
#define IMX_PADMUX_NAND_ALE                   (IMX_IOMUXC_VBASE+IMX_PADMUX_NAND_ALE_OFFSET)
#define IMX_PADMUX_NAND_WP                    (IMX_IOMUXC_VBASE+IMX_PADMUX_NAND_WP_OFFSET)
#define IMX_PADMUX_NAND_READY                 (IMX_IOMUXC_VBASE+IMX_PADMUX_NAND_READY_OFFSET)
#define IMX_PADMUX_NAND_CS0                   (IMX_IOMUXC_VBASE+IMX_PADMUX_NAND_CS0_OFFSET)
#define IMX_PADMUX_NAND_CS1                   (IMX_IOMUXC_VBASE+IMX_PADMUX_NAND_CS1_OFFSET)
#define IMX_PADMUX_NAND_CS2                   (IMX_IOMUXC_VBASE+IMX_PADMUX_NAND_CS2_OFFSET)
#define IMX_PADMUX_NAND_CS3                   (IMX_IOMUXC_VBASE+IMX_PADMUX_NAND_CS3_OFFSET)
#define IMX_PADMUX_SD4_CMD                    (IMX_IOMUXC_VBASE+IMX_PADMUX_SD4_CMD_OFFSET)
#define IMX_PADMUX_SD4_CLK                    (IMX_IOMUXC_VBASE+IMX_PADMUX_SD4_CLK_OFFSET)
#define IMX_PADMUX_NAND_DATA00                (IMX_IOMUXC_VBASE+IMX_PADMUX_NAND_DATA00_OFFSET)
#define IMX_PADMUX_NAND_DATA01                (IMX_IOMUXC_VBASE+IMX_PADMUX_NAND_DATA01_OFFSET)
#define IMX_PADMUX_NAND_DATA02                (IMX_IOMUXC_VBASE+IMX_PADMUX_NAND_DATA02_OFFSET)
#define IMX_PADMUX_NAND_DATA03                (IMX_IOMUXC_VBASE+IMX_PADMUX_NAND_DATA03_OFFSET)
#define IMX_PADMUX_NAND_DATA04                (IMX_IOMUXC_VBASE+IMX_PADMUX_NAND_DATA04_OFFSET)
#define IMX_PADMUX_NAND_DATA05                (IMX_IOMUXC_VBASE+IMX_PADMUX_NAND_DATA05_OFFSET)
#define IMX_PADMUX_NAND_DATA06                (IMX_IOMUXC_VBASE+IMX_PADMUX_NAND_DATA06_OFFSET)
#define IMX_PADMUX_NAND_DATA07                (IMX_IOMUXC_VBASE+IMX_PADMUX_NAND_DATA07_OFFSET)
#define IMX_PADMUX_SD4_DATA0                  (IMX_IOMUXC_VBASE+IMX_PADMUX_SD4_DATA0_OFFSET)
#define IMX_PADMUX_SD4_DATA1                  (IMX_IOMUXC_VBASE+IMX_PADMUX_SD4_DATA1_OFFSET)
#define IMX_PADMUX_SD4_DATA2                  (IMX_IOMUXC_VBASE+IMX_PADMUX_SD4_DATA2_OFFSET)
#define IMX_PADMUX_SD4_DATA3                  (IMX_IOMUXC_VBASE+IMX_PADMUX_SD4_DATA3_OFFSET)
#define IMX_PADMUX_SD4_DATA4                  (IMX_IOMUXC_VBASE+IMX_PADMUX_SD4_DATA4_OFFSET)
#define IMX_PADMUX_SD4_DATA5                  (IMX_IOMUXC_VBASE+IMX_PADMUX_SD4_DATA5_OFFSET)
#define IMX_PADMUX_SD4_DATA6                  (IMX_IOMUXC_VBASE+IMX_PADMUX_SD4_DATA6_OFFSET)
#define IMX_PADMUX_SD4_DATA7                  (IMX_IOMUXC_VBASE+IMX_PADMUX_SD4_DATA7_OFFSET)
#define IMX_PADMUX_SD1_DATA1                  (IMX_IOMUXC_VBASE+IMX_PADMUX_SD1_DATA1_OFFSET)
#define IMX_PADMUX_SD1_DATA0                  (IMX_IOMUXC_VBASE+IMX_PADMUX_SD1_DATA0_OFFSET)
#define IMX_PADMUX_SD1_DATA3                  (IMX_IOMUXC_VBASE+IMX_PADMUX_SD1_DATA3_OFFSET)
#define IMX_PADMUX_SD1_CMD                    (IMX_IOMUXC_VBASE+IMX_PADMUX_SD1_CMD_OFFSET)
#define IMX_PADMUX_SD1_DATA2                  (IMX_IOMUXC_VBASE+IMX_PADMUX_SD1_DATA2_OFFSET)
#define IMX_PADMUX_SD1_CLK                    (IMX_IOMUXC_VBASE+IMX_PADMUX_SD1_CLK_OFFSET)
#define IMX_PADMUX_SD2_CLK                    (IMX_IOMUXC_VBASE+IMX_PADMUX_SD2_CLK_OFFSET)
#define IMX_PADMUX_SD2_CMD                    (IMX_IOMUXC_VBASE+IMX_PADMUX_SD2_CMD_OFFSET)
#define IMX_PADMUX_SD2_DATA3                  (IMX_IOMUXC_VBASE+IMX_PADMUX_SD2_DATA3_OFFSET)

/* Pad Control Registers */

#define IMX_PADCTL_ADDRESS(n)                 (IMX_IOMUXC_VBASE+IMX_PADCTL_OFFSET(n))

#define IMX_PADCTL_SD2_DATA1                  (IMX_IOMUXC_VBASE+IMX_PADCTL_SD2_DATA1_OFFSET)
#define IMX_PADCTL_SD2_DATA2                  (IMX_IOMUXC_VBASE+IMX_PADCTL_SD2_DATA2_OFFSET)
#define IMX_PADCTL_SD2_DATA0                  (IMX_IOMUXC_VBASE+IMX_PADCTL_SD2_DATA0_OFFSET)
#define IMX_PADCTL_RGMII_TXC                  (IMX_IOMUXC_VBASE+IMX_PADCTL_RGMII_TXC_OFFSET)
#define IMX_PADCTL_RGMII_TD0                  (IMX_IOMUXC_VBASE+IMX_PADCTL_RGMII_TD0_OFFSET)
#define IMX_PADCTL_RGMII_TD1                  (IMX_IOMUXC_VBASE+IMX_PADCTL_RGMII_TD1_OFFSET)
#define IMX_PADCTL_RGMII_TD2                  (IMX_IOMUXC_VBASE+IMX_PADCTL_RGMII_TD2_OFFSET)
#define IMX_PADCTL_RGMII_TD3                  (IMX_IOMUXC_VBASE+IMX_PADCTL_RGMII_TD3_OFFSET)
#define IMX_PADCTL_RGMII_RX_CTL               (IMX_IOMUXC_VBASE+IMX_PADCTL_RGMII_RX_CTL_OFFSET)
#define IMX_PADCTL_RGMII_RD0                  (IMX_IOMUXC_VBASE+IMX_PADCTL_RGMII_RD0_OFFSET)
#define IMX_PADCTL_RGMII_TX_CTL               (IMX_IOMUXC_VBASE+IMX_PADCTL_RGMII_TX_CTL_OFFSET)
#define IMX_PADCTL_RGMII_RD1                  (IMX_IOMUXC_VBASE+IMX_PADCTL_RGMII_RD1_OFFSET)
#define IMX_PADCTL_RGMII_RD2                  (IMX_IOMUXC_VBASE+IMX_PADCTL_RGMII_RD2_OFFSET)
#define IMX_PADCTL_RGMII_RD3                  (IMX_IOMUXC_VBASE+IMX_PADCTL_RGMII_RD3_OFFSET)
#define IMX_PADCTL_RGMII_RXC                  (IMX_IOMUXC_VBASE+IMX_PADCTL_RGMII_RXC_OFFSET)
#define IMX_PADCTL_EIM_ADDR25                 (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_ADDR25_OFFSET)
#define IMX_PADCTL_EIM_EB2                    (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_EB2_OFFSET)
#define IMX_PADCTL_EIM_DATA16                 (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_DATA16_OFFSET)
#define IMX_PADCTL_EIM_DATA17                 (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_DATA17_OFFSET)
#define IMX_PADCTL_EIM_DATA18                 (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_DATA18_OFFSET)
#define IMX_PADCTL_EIM_DATA19                 (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_DATA19_OFFSET)
#define IMX_PADCTL_EIM_DATA20                 (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_DATA20_OFFSET)
#define IMX_PADCTL_EIM_DATA21                 (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_DATA21_OFFSET)
#define IMX_PADCTL_EIM_DATA22                 (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_DATA22_OFFSET)
#define IMX_PADCTL_EIM_DATA23                 (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_DATA23_OFFSET)
#define IMX_PADCTL_EIM_EB3                    (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_EB3_OFFSET)
#define IMX_PADCTL_EIM_DATA24                 (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_DATA24_OFFSET)
#define IMX_PADCTL_EIM_DATA25                 (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_DATA25_OFFSET)
#define IMX_PADCTL_EIM_DATA26                 (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_DATA26_OFFSET)
#define IMX_PADCTL_EIM_DATA27                 (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_DATA27_OFFSET)
#define IMX_PADCTL_EIM_DATA28                 (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_DATA28_OFFSET)
#define IMX_PADCTL_EIM_DATA29                 (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_DATA29_OFFSET)
#define IMX_PADCTL_EIM_DATA30                 (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_DATA30_OFFSET)
#define IMX_PADCTL_EIM_DATA31                 (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_DATA31_OFFSET)
#define IMX_PADCTL_EIM_ADDR24                 (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_ADDR24_OFFSET)
#define IMX_PADCTL_EIM_ADDR23                 (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_ADDR23_OFFSET)
#define IMX_PADCTL_EIM_ADDR22                 (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_ADDR22_OFFSET)
#define IMX_PADCTL_EIM_ADDR21                 (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_ADDR21_OFFSET)
#define IMX_PADCTL_EIM_ADDR20                 (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_ADDR20_OFFSET)
#define IMX_PADCTL_EIM_ADDR19                 (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_ADDR19_OFFSET)
#define IMX_PADCTL_EIM_ADDR18                 (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_ADDR18_OFFSET)
#define IMX_PADCTL_EIM_ADDR17                 (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_ADDR17_OFFSET)
#define IMX_PADCTL_EIM_ADDR16                 (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_ADDR16_OFFSET)
#define IMX_PADCTL_EIM_CS0                    (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_CS0_OFFSET)
#define IMX_PADCTL_EIM_CS1                    (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_CS1_OFFSET)
#define IMX_PADCTL_EIM_OE                     (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_OE_OFFSET)
#define IMX_PADCTL_EIM_RW                     (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_RW_OFFSET)
#define IMX_PADCTL_EIM_LBA                    (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_LBA_OFFSET)
#define IMX_PADCTL_EIM_EB0                    (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_EB0_OFFSET)
#define IMX_PADCTL_EIM_EB1                    (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_EB1_OFFSET)
#define IMX_PADCTL_EIM_AD00                   (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_AD00_OFFSET)
#define IMX_PADCTL_EIM_AD01                   (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_AD01_OFFSET)
#define IMX_PADCTL_EIM_AD02                   (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_AD02_OFFSET)
#define IMX_PADCTL_EIM_AD03                   (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_AD03_OFFSET)
#define IMX_PADCTL_EIM_AD04                   (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_AD04_OFFSET)
#define IMX_PADCTL_EIM_AD05                   (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_AD05_OFFSET)
#define IMX_PADCTL_EIM_AD06                   (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_AD06_OFFSET)
#define IMX_PADCTL_EIM_AD07                   (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_AD07_OFFSET)
#define IMX_PADCTL_EIM_AD08                   (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_AD08_OFFSET)
#define IMX_PADCTL_EIM_AD09                   (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_AD09_OFFSET)
#define IMX_PADCTL_EIM_AD10                   (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_AD10_OFFSET)
#define IMX_PADCTL_EIM_AD11                   (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_AD11_OFFSET)
#define IMX_PADCTL_EIM_AD12                   (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_AD12_OFFSET)
#define IMX_PADCTL_EIM_AD13                   (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_AD13_OFFSET)
#define IMX_PADCTL_EIM_AD14                   (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_AD14_OFFSET)
#define IMX_PADCTL_EIM_AD15                   (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_AD15_OFFSET)
#define IMX_PADCTL_EIM_WAIT                   (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_WAIT_OFFSET)
#define IMX_PADCTL_EIM_BCLK                   (IMX_IOMUXC_VBASE+IMX_PADCTL_EIM_BCLK_OFFSET)
#define IMX_PADCTL_DI0_DISP_CLK               (IMX_IOMUXC_VBASE+IMX_PADCTL_DI0_DISP_CLK_OFFSET)
#define IMX_PADCTL_DI0_PIN15                  (IMX_IOMUXC_VBASE+IMX_PADCTL_DI0_PIN15_OFFSET)
#define IMX_PADCTL_DI0_PIN02                  (IMX_IOMUXC_VBASE+IMX_PADCTL_DI0_PIN02_OFFSET)
#define IMX_PADCTL_DI0_PIN03                  (IMX_IOMUXC_VBASE+IMX_PADCTL_DI0_PIN03_OFFSET)
#define IMX_PADCTL_DI0_PIN04                  (IMX_IOMUXC_VBASE+IMX_PADCTL_DI0_PIN04_OFFSET)
#define IMX_PADCTL_DISP0_DATA00               (IMX_IOMUXC_VBASE+IMX_PADCTL_DISP0_DATA00_OFFSET)
#define IMX_PADCTL_DISP0_DATA01               (IMX_IOMUXC_VBASE+IMX_PADCTL_DISP0_DATA01_OFFSET)
#define IMX_PADCTL_DISP0_DATA02               (IMX_IOMUXC_VBASE+IMX_PADCTL_DISP0_DATA02_OFFSET)
#define IMX_PADCTL_DISP0_DATA03               (IMX_IOMUXC_VBASE+IMX_PADCTL_DISP0_DATA03_OFFSET)
#define IMX_PADCTL_DISP0_DATA04               (IMX_IOMUXC_VBASE+IMX_PADCTL_DISP0_DATA04_OFFSET)
#define IMX_PADCTL_DISP0_DATA05               (IMX_IOMUXC_VBASE+IMX_PADCTL_DISP0_DATA05_OFFSET)
#define IMX_PADCTL_DISP0_DATA06               (IMX_IOMUXC_VBASE+IMX_PADCTL_DISP0_DATA06_OFFSET)
#define IMX_PADCTL_DISP0_DATA07               (IMX_IOMUXC_VBASE+IMX_PADCTL_DISP0_DATA07_OFFSET)
#define IMX_PADCTL_DISP0_DATA08               (IMX_IOMUXC_VBASE+IMX_PADCTL_DISP0_DATA08_OFFSET)
#define IMX_PADCTL_DISP0_DATA09               (IMX_IOMUXC_VBASE+IMX_PADCTL_DISP0_DATA09_OFFSET)
#define IMX_PADCTL_DISP0_DATA10               (IMX_IOMUXC_VBASE+IMX_PADCTL_DISP0_DATA10_OFFSET)
#define IMX_PADCTL_DISP0_DATA11               (IMX_IOMUXC_VBASE+IMX_PADCTL_DISP0_DATA11_OFFSET)
#define IMX_PADCTL_DISP0_DATA12               (IMX_IOMUXC_VBASE+IMX_PADCTL_DISP0_DATA12_OFFSET)
#define IMX_PADCTL_DISP0_DATA13               (IMX_IOMUXC_VBASE+IMX_PADCTL_DISP0_DATA13_OFFSET)
#define IMX_PADCTL_DISP0_DATA14               (IMX_IOMUXC_VBASE+IMX_PADCTL_DISP0_DATA14_OFFSET)
#define IMX_PADCTL_DISP0_DATA15               (IMX_IOMUXC_VBASE+IMX_PADCTL_DISP0_DATA15_OFFSET)
#define IMX_PADCTL_DISP0_DATA16               (IMX_IOMUXC_VBASE+IMX_PADCTL_DISP0_DATA16_OFFSET)
#define IMX_PADCTL_DISP0_DATA17               (IMX_IOMUXC_VBASE+IMX_PADCTL_DISP0_DATA17_OFFSET)
#define IMX_PADCTL_DISP0_DATA18               (IMX_IOMUXC_VBASE+IMX_PADCTL_DISP0_DATA18_OFFSET)
#define IMX_PADCTL_DISP0_DATA19               (IMX_IOMUXC_VBASE+IMX_PADCTL_DISP0_DATA19_OFFSET)
#define IMX_PADCTL_DISP0_DATA20               (IMX_IOMUXC_VBASE+IMX_PADCTL_DISP0_DATA20_OFFSET)
#define IMX_PADCTL_DISP0_DATA21               (IMX_IOMUXC_VBASE+IMX_PADCTL_DISP0_DATA21_OFFSET)
#define IMX_PADCTL_DISP0_DATA22               (IMX_IOMUXC_VBASE+IMX_PADCTL_DISP0_DATA22_OFFSET)
#define IMX_PADCTL_DISP0_DATA23               (IMX_IOMUXC_VBASE+IMX_PADCTL_DISP0_DATA23_OFFSET)
#define IMX_PADCTL_ENET_MDIO                  (IMX_IOMUXC_VBASE+IMX_PADCTL_ENET_MDIO_OFFSET)
#define IMX_PADCTL_ENET_REF_CLK               (IMX_IOMUXC_VBASE+IMX_PADCTL_ENET_REF_CLK_OFFSET)
#define IMX_PADCTL_ENET_RX_ER                 (IMX_IOMUXC_VBASE+IMX_PADCTL_ENET_RX_ER_OFFSET)
#define IMX_PADCTL_ENET_CRS_DV                (IMX_IOMUXC_VBASE+IMX_PADCTL_ENET_CRS_DV_OFFSET)
#define IMX_PADCTL_ENET_RX_DATA1              (IMX_IOMUXC_VBASE+IMX_PADCTL_ENET_RX_DATA1_OFFSET)
#define IMX_PADCTL_ENET_RX_DATA0              (IMX_IOMUXC_VBASE+IMX_PADCTL_ENET_RX_DATA0_OFFSET)
#define IMX_PADCTL_ENET_TX_EN                 (IMX_IOMUXC_VBASE+IMX_PADCTL_ENET_TX_EN_OFFSET)
#define IMX_PADCTL_ENET_TX_DATA1              (IMX_IOMUXC_VBASE+IMX_PADCTL_ENET_TX_DATA1_OFFSET)
#define IMX_PADCTL_ENET_TX_DATA0              (IMX_IOMUXC_VBASE+IMX_PADCTL_ENET_TX_DATA0_OFFSET)
#define IMX_PADCTL_ENET_MDC                   (IMX_IOMUXC_VBASE+IMX_PADCTL_ENET_MDC_OFFSET)
#define IMX_PADCTL_DRAM_SDQS5_P               (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_SDQS5_P_OFFSET)
#define IMX_PADCTL_DRAM_DQM5                  (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_DQM5_OFFSET)
#define IMX_PADCTL_DRAM_DQM4                  (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_DQM4_OFFSET)
#define IMX_PADCTL_DRAM_SDQS4_P               (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_SDQS4_P_OFFSET)
#define IMX_PADCTL_DRAM_SDQS3_P               (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_SDQS3_P_OFFSET)
#define IMX_PADCTL_DRAM_DQM3                  (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_DQM3_OFFSET)
#define IMX_PADCTL_DRAM_SDQS2_P               (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_SDQS2_P_OFFSET)
#define IMX_PADCTL_DRAM_DQM2                  (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_DQM2_OFFSET)
#define IMX_PADCTL_DRAM_ADDR00                (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_ADDR00_OFFSET)
#define IMX_PADCTL_DRAM_ADDR01                (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_ADDR01_OFFSET)
#define IMX_PADCTL_DRAM_ADDR02                (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_ADDR02_OFFSET)
#define IMX_PADCTL_DRAM_ADDR03                (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_ADDR03_OFFSET)
#define IMX_PADCTL_DRAM_ADDR04                (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_ADDR04_OFFSET)
#define IMX_PADCTL_DRAM_ADDR05                (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_ADDR05_OFFSET)
#define IMX_PADCTL_DRAM_ADDR06                (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_ADDR06_OFFSET)
#define IMX_PADCTL_DRAM_ADDR07                (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_ADDR07_OFFSET)
#define IMX_PADCTL_DRAM_ADDR08                (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_ADDR08_OFFSET)
#define IMX_PADCTL_DRAM_ADDR09                (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_ADDR09_OFFSET)
#define IMX_PADCTL_DRAM_ADDR10                (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_ADDR10_OFFSET)
#define IMX_PADCTL_DRAM_ADDR11                (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_ADDR11_OFFSET)
#define IMX_PADCTL_DRAM_ADDR12                (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_ADDR12_OFFSET)
#define IMX_PADCTL_DRAM_ADDR13                (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_ADDR13_OFFSET)
#define IMX_PADCTL_DRAM_ADDR14                (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_ADDR14_OFFSET)
#define IMX_PADCTL_DRAM_ADDR15                (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_ADDR15_OFFSET)
#define IMX_PADCTL_DRAM_CAS                   (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_CAS_OFFSET)
#define IMX_PADCTL_DRAM_CS0                   (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_CS0_OFFSET)
#define IMX_PADCTL_DRAM_CS1                   (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_CS1_OFFSET)
#define IMX_PADCTL_DRAM_RAS                   (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_RAS_OFFSET)
#define IMX_PADCTL_DRAM_RESET                 (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_RESET_OFFSET)
#define IMX_PADCTL_DRAM_SDBA0                 (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_SDBA0_OFFSET)
#define IMX_PADCTL_DRAM_SDBA1                 (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_SDBA1_OFFSET)
#define IMX_PADCTL_DRAM_SDCLK0_P              (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_SDCLK0_P_OFFSET)
#define IMX_PADCTL_DRAM_SDBA2                 (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_SDBA2_OFFSET)
#define IMX_PADCTL_DRAM_SDCKE0                (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_SDCKE0_OFFSET)
#define IMX_PADCTL_DRAM_SDCLK1_P              (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_SDCLK1_P_OFFSET)
#define IMX_PADCTL_DRAM_SDCKE1                (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_SDCKE1_OFFSET)
#define IMX_PADCTL_DRAM_ODT0                  (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_ODT0_OFFSET)
#define IMX_PADCTL_DRAM_ODT1                  (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_ODT1_OFFSET)
#define IMX_PADCTL_DRAM_SDWE_B                (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_SDWE_B_OFFSET)
#define IMX_PADCTL_DRAM_SDQS0_P               (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_SDQS0_P_OFFSET)
#define IMX_PADCTL_DRAM_DQM0                  (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_DQM0_OFFSET)
#define IMX_PADCTL_DRAM_SDQS1_P               (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_SDQS1_P_OFFSET)
#define IMX_PADCTL_DRAM_DQM1                  (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_DQM1_OFFSET)
#define IMX_PADCTL_DRAM_SDQS6_P               (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_SDQS6_P_OFFSET)
#define IMX_PADCTL_DRAM_DQM6                  (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_DQM6_OFFSET)
#define IMX_PADCTL_DRAM_SDQS7_P               (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_SDQS7_P_OFFSET)
#define IMX_PADCTL_DRAM_DQM7                  (IMX_IOMUXC_VBASE+IMX_PADCTL_DRAM_DQM7_OFFSET)
#define IMX_PADCTL_KEY_COL0                   (IMX_IOMUXC_VBASE+IMX_PADCTL_KEY_COL0_OFFSET)
#define IMX_PADCTL_KEY_ROW0                   (IMX_IOMUXC_VBASE+IMX_PADCTL_KEY_ROW0_OFFSET)
#define IMX_PADCTL_KEY_COL1                   (IMX_IOMUXC_VBASE+IMX_PADCTL_KEY_COL1_OFFSET)
#define IMX_PADCTL_KEY_ROW1                   (IMX_IOMUXC_VBASE+IMX_PADCTL_KEY_ROW1_OFFSET)
#define IMX_PADCTL_KEY_COL2                   (IMX_IOMUXC_VBASE+IMX_PADCTL_KEY_COL2_OFFSET)
#define IMX_PADCTL_KEY_ROW2                   (IMX_IOMUXC_VBASE+IMX_PADCTL_KEY_ROW2_OFFSET)
#define IMX_PADCTL_KEY_COL3                   (IMX_IOMUXC_VBASE+IMX_PADCTL_KEY_COL3_OFFSET)
#define IMX_PADCTL_KEY_ROW3                   (IMX_IOMUXC_VBASE+IMX_PADCTL_KEY_ROW3_OFFSET)
#define IMX_PADCTL_KEY_COL4                   (IMX_IOMUXC_VBASE+IMX_PADCTL_KEY_COL4_OFFSET)
#define IMX_PADCTL_KEY_ROW4                   (IMX_IOMUXC_VBASE+IMX_PADCTL_KEY_ROW4_OFFSET)
#define IMX_PADCTL_GPIO00                     (IMX_IOMUXC_VBASE+IMX_PADCTL_GPIO00_OFFSET)
#define IMX_PADCTL_GPIO01                     (IMX_IOMUXC_VBASE+IMX_PADCTL_GPIO01_OFFSET)
#define IMX_PADCTL_GPIO09                     (IMX_IOMUXC_VBASE+IMX_PADCTL_GPIO09_OFFSET)
#define IMX_PADCTL_GPIO03                     (IMX_IOMUXC_VBASE+IMX_PADCTL_GPIO03_OFFSET)
#define IMX_PADCTL_GPIO06                     (IMX_IOMUXC_VBASE+IMX_PADCTL_GPIO06_OFFSET)
#define IMX_PADCTL_GPIO02                     (IMX_IOMUXC_VBASE+IMX_PADCTL_GPIO02_OFFSET)
#define IMX_PADCTL_GPIO04                     (IMX_IOMUXC_VBASE+IMX_PADCTL_GPIO04_OFFSET)
#define IMX_PADCTL_GPIO05                     (IMX_IOMUXC_VBASE+IMX_PADCTL_GPIO05_OFFSET)
#define IMX_PADCTL_GPIO07                     (IMX_IOMUXC_VBASE+IMX_PADCTL_GPIO07_OFFSET)
#define IMX_PADCTL_GPIO08                     (IMX_IOMUXC_VBASE+IMX_PADCTL_GPIO08_OFFSET)
#define IMX_PADCTL_GPIO16                     (IMX_IOMUXC_VBASE+IMX_PADCTL_GPIO16_OFFSET)
#define IMX_PADCTL_GPIO17                     (IMX_IOMUXC_VBASE+IMX_PADCTL_GPIO17_OFFSET)
#define IMX_PADCTL_GPIO18                     (IMX_IOMUXC_VBASE+IMX_PADCTL_GPIO18_OFFSET)
#define IMX_PADCTL_GPIO19                     (IMX_IOMUXC_VBASE+IMX_PADCTL_GPIO19_OFFSET)
#define IMX_PADCTL_CSI0_PIXCLK                (IMX_IOMUXC_VBASE+IMX_PADCTL_CSI0_PIXCLK_OFFSET)
#define IMX_PADCTL_CSI0_HSYNC                 (IMX_IOMUXC_VBASE+IMX_PADCTL_CSI0_HSYNC_OFFSET)
#define IMX_PADCTL_CSI0_DATA_EN               (IMX_IOMUXC_VBASE+IMX_PADCTL_CSI0_DATA_EN_OFFSET)
#define IMX_PADCTL_CSI0_VSYNC                 (IMX_IOMUXC_VBASE+IMX_PADCTL_CSI0_VSYNC_OFFSET)
#define IMX_PADCTL_CSI0_DATA04                (IMX_IOMUXC_VBASE+IMX_PADCTL_CSI0_DATA04_OFFSET)
#define IMX_PADCTL_CSI0_DATA05                (IMX_IOMUXC_VBASE+IMX_PADCTL_CSI0_DATA05_OFFSET)
#define IMX_PADCTL_CSI0_DATA06                (IMX_IOMUXC_VBASE+IMX_PADCTL_CSI0_DATA06_OFFSET)
#define IMX_PADCTL_CSI0_DATA07                (IMX_IOMUXC_VBASE+IMX_PADCTL_CSI0_DATA07_OFFSET)
#define IMX_PADCTL_CSI0_DATA08                (IMX_IOMUXC_VBASE+IMX_PADCTL_CSI0_DATA08_OFFSET)
#define IMX_PADCTL_CSI0_DATA09                (IMX_IOMUXC_VBASE+IMX_PADCTL_CSI0_DATA09_OFFSET)
#define IMX_PADCTL_CSI0_DATA10                (IMX_IOMUXC_VBASE+IMX_PADCTL_CSI0_DATA10_OFFSET)
#define IMX_PADCTL_CSI0_DATA11                (IMX_IOMUXC_VBASE+IMX_PADCTL_CSI0_DATA11_OFFSET)
#define IMX_PADCTL_CSI0_DATA12                (IMX_IOMUXC_VBASE+IMX_PADCTL_CSI0_DATA12_OFFSET)
#define IMX_PADCTL_CSI0_DATA13                (IMX_IOMUXC_VBASE+IMX_PADCTL_CSI0_DATA13_OFFSET)
#define IMX_PADCTL_CSI0_DATA14                (IMX_IOMUXC_VBASE+IMX_PADCTL_CSI0_DATA14_OFFSET)
#define IMX_PADCTL_CSI0_DATA15                (IMX_IOMUXC_VBASE+IMX_PADCTL_CSI0_DATA15_OFFSET)
#define IMX_PADCTL_CSI0_DATA16                (IMX_IOMUXC_VBASE+IMX_PADCTL_CSI0_DATA16_OFFSET)
#define IMX_PADCTL_CSI0_DATA17                (IMX_IOMUXC_VBASE+IMX_PADCTL_CSI0_DATA17_OFFSET)
#define IMX_PADCTL_CSI0_DATA18                (IMX_IOMUXC_VBASE+IMX_PADCTL_CSI0_DATA18_OFFSET)
#define IMX_PADCTL_CSI0_DATA19                (IMX_IOMUXC_VBASE+IMX_PADCTL_CSI0_DATA19_OFFSET)
#define IMX_PADCTL_JTAG_TMS                   (IMX_IOMUXC_VBASE+IMX_PADCTL_JTAG_TMS_OFFSET)
#define IMX_PADCTL_JTAG_MOD                   (IMX_IOMUXC_VBASE+IMX_PADCTL_JTAG_MOD_OFFSET)
#define IMX_PADCTL_JTAG_TRSTB                 (IMX_IOMUXC_VBASE+IMX_PADCTL_JTAG_TRSTB_OFFSET)
#define IMX_PADCTL_JTAG_TDI                   (IMX_IOMUXC_VBASE+IMX_PADCTL_JTAG_TDI_OFFSET)
#define IMX_PADCTL_JTAG_TCK                   (IMX_IOMUXC_VBASE+IMX_PADCTL_JTAG_TCK_OFFSET)
#define IMX_PADCTL_JTAG_TDO                   (IMX_IOMUXC_VBASE+IMX_PADCTL_JTAG_TDO_OFFSET)
#define IMX_PADCTL_SD3_DATA7                  (IMX_IOMUXC_VBASE+IMX_PADCTL_SD3_DATA7_OFFSET)
#define IMX_PADCTL_SD3_DATA6                  (IMX_IOMUXC_VBASE+IMX_PADCTL_SD3_DATA6_OFFSET)
#define IMX_PADCTL_SD3_DATA5                  (IMX_IOMUXC_VBASE+IMX_PADCTL_SD3_DATA5_OFFSET)
#define IMX_PADCTL_SD3_DATA4                  (IMX_IOMUXC_VBASE+IMX_PADCTL_SD3_DATA4_OFFSET)
#define IMX_PADCTL_SD3_CMD                    (IMX_IOMUXC_VBASE+IMX_PADCTL_SD3_CMD_OFFSET)
#define IMX_PADCTL_SD3_CLK                    (IMX_IOMUXC_VBASE+IMX_PADCTL_SD3_CLK_OFFSET)
#define IMX_PADCTL_SD3_DATA0                  (IMX_IOMUXC_VBASE+IMX_PADCTL_SD3_DATA0_OFFSET)
#define IMX_PADCTL_SD3_DATA1                  (IMX_IOMUXC_VBASE+IMX_PADCTL_SD3_DATA1_OFFSET)
#define IMX_PADCTL_SD3_DATA2                  (IMX_IOMUXC_VBASE+IMX_PADCTL_SD3_DATA2_OFFSET)
#define IMX_PADCTL_SD3_DATA3                  (IMX_IOMUXC_VBASE+IMX_PADCTL_SD3_DATA3_OFFSET)
#define IMX_PADCTL_SD3_RESET                  (IMX_IOMUXC_VBASE+IMX_PADCTL_SD3_RESET_OFFSET)
#define IMX_PADCTL_NAND_CLE                   (IMX_IOMUXC_VBASE+IMX_PADCTL_NAND_CLE_OFFSET)
#define IMX_PADCTL_NAND_ALE                   (IMX_IOMUXC_VBASE+IMX_PADCTL_NAND_ALE_OFFSET)
#define IMX_PADCTL_NAND_WP                    (IMX_IOMUXC_VBASE+IMX_PADCTL_NAND_WP_OFFSET)
#define IMX_PADCTL_NAND_READY                 (IMX_IOMUXC_VBASE+IMX_PADCTL_NAND_READY_OFFSET)
#define IMX_PADCTL_NAND_CS0                   (IMX_IOMUXC_VBASE+IMX_PADCTL_NAND_CS0_OFFSET)
#define IMX_PADCTL_NAND_CS1                   (IMX_IOMUXC_VBASE+IMX_PADCTL_NAND_CS1_OFFSET)
#define IMX_PADCTL_NAND_CS2                   (IMX_IOMUXC_VBASE+IMX_PADCTL_NAND_CS2_OFFSET)
#define IMX_PADCTL_NAND_CS3                   (IMX_IOMUXC_VBASE+IMX_PADCTL_NAND_CS3_OFFSET)
#define IMX_PADCTL_SD4_CMD                    (IMX_IOMUXC_VBASE+IMX_PADCTL_SD4_CMD_OFFSET)
#define IMX_PADCTL_SD4_CLK                    (IMX_IOMUXC_VBASE+IMX_PADCTL_SD4_CLK_OFFSET)
#define IMX_PADCTL_NAND_DATA00                (IMX_IOMUXC_VBASE+IMX_PADCTL_NAND_DATA00_OFFSET)
#define IMX_PADCTL_NAND_DATA01                (IMX_IOMUXC_VBASE+IMX_PADCTL_NAND_DATA01_OFFSET)
#define IMX_PADCTL_NAND_DATA02                (IMX_IOMUXC_VBASE+IMX_PADCTL_NAND_DATA02_OFFSET)
#define IMX_PADCTL_NAND_DATA03                (IMX_IOMUXC_VBASE+IMX_PADCTL_NAND_DATA03_OFFSET)
#define IMX_PADCTL_NAND_DATA04                (IMX_IOMUXC_VBASE+IMX_PADCTL_NAND_DATA04_OFFSET)
#define IMX_PADCTL_NAND_DATA05                (IMX_IOMUXC_VBASE+IMX_PADCTL_NAND_DATA05_OFFSET)
#define IMX_PADCTL_NAND_DATA06                (IMX_IOMUXC_VBASE+IMX_PADCTL_NAND_DATA06_OFFSET)
#define IMX_PADCTL_NAND_DATA07                (IMX_IOMUXC_VBASE+IMX_PADCTL_NAND_DATA07_OFFSET)
#define IMX_PADCTL_SD4_DATA0                  (IMX_IOMUXC_VBASE+IMX_PADCTL_SD4_DATA0_OFFSET)
#define IMX_PADCTL_SD4_DATA1                  (IMX_IOMUXC_VBASE+IMX_PADCTL_SD4_DATA1_OFFSET)
#define IMX_PADCTL_SD4_DATA2                  (IMX_IOMUXC_VBASE+IMX_PADCTL_SD4_DATA2_OFFSET)
#define IMX_PADCTL_SD4_DATA3                  (IMX_IOMUXC_VBASE+IMX_PADCTL_SD4_DATA3_OFFSET)
#define IMX_PADCTL_SD4_DATA4                  (IMX_IOMUXC_VBASE+IMX_PADCTL_SD4_DATA4_OFFSET)
#define IMX_PADCTL_SD4_DATA5                  (IMX_IOMUXC_VBASE+IMX_PADCTL_SD4_DATA5_OFFSET)
#define IMX_PADCTL_SD4_DATA6                  (IMX_IOMUXC_VBASE+IMX_PADCTL_SD4_DATA6_OFFSET)
#define IMX_PADCTL_SD4_DATA7                  (IMX_IOMUXC_VBASE+IMX_PADCTL_SD4_DATA7_OFFSET)
#define IMX_PADCTL_SD1_DATA1                  (IMX_IOMUXC_VBASE+IMX_PADCTL_SD1_DATA1_OFFSET)
#define IMX_PADCTL_SD1_DATA0                  (IMX_IOMUXC_VBASE+IMX_PADCTL_SD1_DATA0_OFFSET)
#define IMX_PADCTL_SD1_DATA3                  (IMX_IOMUXC_VBASE+IMX_PADCTL_SD1_DATA3_OFFSET)
#define IMX_PADCTL_SD1_CMD                    (IMX_IOMUXC_VBASE+IMX_PADCTL_SD1_CMD_OFFSET)
#define IMX_PADCTL_SD1_DATA2                  (IMX_IOMUXC_VBASE+IMX_PADCTL_SD1_DATA2_OFFSET)
#define IMX_PADCTL_SD1_CLK                    (IMX_IOMUXC_VBASE+IMX_PADCTL_SD1_CLK_OFFSET)
#define IMX_PADCTL_SD2_CLK                    (IMX_IOMUXC_VBASE+IMX_PADCTL_SD2_CLK_OFFSET)
#define IMX_PADCTL_SD2_CMD                    (IMX_IOMUXC_VBASE+IMX_PADCTL_SD2_CMD_OFFSET)
#define IMX_PADCTL_SD2_DATA3                  (IMX_IOMUXC_VBASE+IMX_PADCTL_SD2_DATA3_OFFSET)

/* Pad Group Control Registers */

#define IMX_PADGROUP_B7DS                     (IMX_IOMUXC_VBASE+IMX_PADGROUP_B7DS_OFFSET)
#define IMX_PADGROUP_ADDDS                    (IMX_IOMUXC_VBASE+IMX_PADGROUP_ADDDS_OFFSET)
#define IMX_PADGROUP_DDRMODE_CTL              (IMX_IOMUXC_VBASE+IMX_PADGROUP_DDRMODE_CTL_OFFSET)
#define IMX_PADGROUP_TERM_CTL0                (IMX_IOMUXC_VBASE+IMX_PADGROUP_TERM_CTL0_OFFSET)
#define IMX_PADGROUP_DDRPKE                   (IMX_IOMUXC_VBASE+IMX_PADGROUP_DDRPKE_OFFSET)
#define IMX_PADGROUP_TERM_CTL1                (IMX_IOMUXC_VBASE+IMX_PADGROUP_TERM_CTL1_OFFSET)
#define IMX_PADGROUP_TERM_CTL2                (IMX_IOMUXC_VBASE+IMX_PADGROUP_TERM_CTL2_OFFSET)
#define IMX_PADGROUP_TERM_CTL3                (IMX_IOMUXC_VBASE+IMX_PADGROUP_TERM_CTL3_OFFSET)
#define IMX_PADGROUP_DDRPK                    (IMX_IOMUXC_VBASE+IMX_PADGROUP_DDRPK_OFFSET)
#define IMX_PADGROUP_TERM_CTL4                (IMX_IOMUXC_VBASE+IMX_PADGROUP_TERM_CTL4_OFFSET)
#define IMX_PADGROUP_DDRHYS                   (IMX_IOMUXC_VBASE+IMX_PADGROUP_DDRHYS_OFFSET)
#define IMX_PADGROUP_DDRMODE                  (IMX_IOMUXC_VBASE+IMX_PADGROUP_DDRMODE_OFFSET)
#define IMX_PADGROUP_TERM_CTL5                (IMX_IOMUXC_VBASE+IMX_PADGROUP_TERM_CTL5_OFFSET)
#define IMX_PADGROUP_TERM_CTL6                (IMX_IOMUXC_VBASE+IMX_PADGROUP_TERM_CTL6_OFFSET)
#define IMX_PADGROUP_TERM_CTL7                (IMX_IOMUXC_VBASE+IMX_PADGROUP_TERM_CTL7_OFFSET)
#define IMX_PADGROUP_B0DS                     (IMX_IOMUXC_VBASE+IMX_PADGROUP_B0DS_OFFSET)
#define IMX_PADGROUP_B1DS                     (IMX_IOMUXC_VBASE+IMX_PADGROUP_B1DS_OFFSET)
#define IMX_PADGROUP_CTLDS                    (IMX_IOMUXC_VBASE+IMX_PADGROUP_CTLDS_OFFSET)
#define IMX_PADGROUP_DDR_TYPE_RGMII           (IMX_IOMUXC_VBASE+IMX_PADGROUP_DDR_TYPE_RGMII_OFFSET)
#define IMX_PADGROUP_B2DS                     (IMX_IOMUXC_VBASE+IMX_PADGROUP_B2DS_OFFSET)
#define IMX_PADGROUP_DDR_TYPE                 (IMX_IOMUXC_VBASE+IMX_PADGROUP_DDR_TYPE_OFFSET)
#define IMX_PADGROUP_B3DS                     (IMX_IOMUXC_VBASE+IMX_PADGROUP_B3DS_OFFSET)
#define IMX_PADGROUP_B4DS                     (IMX_IOMUXC_VBASE+IMX_PADGROUP_B4DS_OFFSET)
#define IMX_PADGROUP_B5DS                     (IMX_IOMUXC_VBASE+IMX_PADGROUP_B5DS_OFFSET)
#define IMX_PADGROUP_B6DS                     (IMX_IOMUXC_VBASE+IMX_PADGROUP_B6DS_OFFSET)
#define IMX_PADGROUP_RGMII_TERM               (IMX_IOMUXC_VBASE+IMX_PADGROUP_RGMII_TERM_OFFSET)

/* Select Input Registers */

#define IMX_INPUT_ASRC_ASRCK_CLOCK_6          (IMX_IOMUXC_VBASE+IMX_INPUT_ASRC_ASRCK_CLOCK_6_OFFSET)
#define IMX_INPUT_AUD4_INPUT_DA_AMX           (IMX_IOMUXC_VBASE+IMX_INPUT_AUD4_INPUT_DA_AMX_OFFSET)
#define IMX_INPUT_AUD4_INPUT_DB_AMX           (IMX_IOMUXC_VBASE+IMX_INPUT_AUD4_INPUT_DB_AMX_OFFSET)
#define IMX_INPUT_AUD4_INPUT_RXCLK_AMX        (IMX_IOMUXC_VBASE+IMX_INPUT_AUD4_INPUT_RXCLK_AMX_OFFSET)
#define IMX_INPUT_AUD4_INPUT_RXFS_AMX         (IMX_IOMUXC_VBASE+IMX_INPUT_AUD4_INPUT_RXFS_AMX_OFFSET)
#define IMX_INPUT_AUD4_INPUT_TXCLK_AMX        (IMX_IOMUXC_VBASE+IMX_INPUT_AUD4_INPUT_TXCLK_AMX_OFFSET)
#define IMX_INPUT_AUD4_INPUT_TXFS_AMX         (IMX_IOMUXC_VBASE+IMX_INPUT_AUD4_INPUT_TXFS_AMX_OFFSET)
#define IMX_INPUT_AUD5_INPUT_DA_AMX           (IMX_IOMUXC_VBASE+IMX_INPUT_AUD5_INPUT_DA_AMX_OFFSET)
#define IMX_INPUT_AUD5_INPUT_DB_AMX           (IMX_IOMUXC_VBASE+IMX_INPUT_AUD5_INPUT_DB_AMX_OFFSET)
#define IMX_INPUT_AUD5_INPUT_RXCLK_AMX        (IMX_IOMUXC_VBASE+IMX_INPUT_AUD5_INPUT_RXCLK_AMX_OFFSET)
#define IMX_INPUT_AUD5_INPUT_RXFS_AMX         (IMX_IOMUXC_VBASE+IMX_INPUT_AUD5_INPUT_RXFS_AMX_OFFSET)
#define IMX_INPUT_AUD5_INPUT_TXCLK_AMX        (IMX_IOMUXC_VBASE+IMX_INPUT_AUD5_INPUT_TXCLK_AMX_OFFSET)
#define IMX_INPUT_AUD5_INPUT_TXFS_AMX         (IMX_IOMUXC_VBASE+IMX_INPUT_AUD5_INPUT_TXFS_AMX_OFFSET)
#define IMX_INPUT_FLEXCAN1_RX                 (IMX_IOMUXC_VBASE+IMX_INPUT_FLEXCAN1_RX_OFFSET)
#define IMX_INPUT_FLEXCAN2_RX                 (IMX_IOMUXC_VBASE+IMX_INPUT_FLEXCAN2_RX_OFFSET)
#define IMX_INPUT_CCM_PMIC_READY              (IMX_IOMUXC_VBASE+IMX_INPUT_CCM_PMIC_READY_OFFSET)
#define IMX_INPUT_ECSPI1_CSPI_CLK_IN          (IMX_IOMUXC_VBASE+IMX_INPUT_ECSPI1_CSPI_CLK_IN_OFFSET)
#define IMX_INPUT_ECSPI1_MISO                 (IMX_IOMUXC_VBASE+IMX_INPUT_ECSPI1_MISO_OFFSET)
#define IMX_INPUT_ECSPI1_MOSI                 (IMX_IOMUXC_VBASE+IMX_INPUT_ECSPI1_MOSI_OFFSET)
#define IMX_INPUT_ECSPI1_SS0                  (IMX_IOMUXC_VBASE+IMX_INPUT_ECSPI1_SS0_OFFSET)
#define IMX_INPUT_ECSPI1_SS1                  (IMX_IOMUXC_VBASE+IMX_INPUT_ECSPI1_SS1_OFFSET)
#define IMX_INPUT_ECSPI1_SS2                  (IMX_IOMUXC_VBASE+IMX_INPUT_ECSPI1_SS2_OFFSET)
#define IMX_INPUT_ECSPI1_SS3                  (IMX_IOMUXC_VBASE+IMX_INPUT_ECSPI1_SS3_OFFSET)
#define IMX_INPUT_ECSPI2_CSPI_CLK_IN          (IMX_IOMUXC_VBASE+IMX_INPUT_ECSPI2_CSPI_CLK_IN_OFFSET)
#define IMX_INPUT_ECSPI2_MISO                 (IMX_IOMUXC_VBASE+IMX_INPUT_ECSPI2_MISO_OFFSET)
#define IMX_INPUT_ECSPI2_MOSI                 (IMX_IOMUXC_VBASE+IMX_INPUT_ECSPI2_MOSI_OFFSET)
#define IMX_INPUT_ECSPI2_SS0                  (IMX_IOMUXC_VBASE+IMX_INPUT_ECSPI2_SS0_OFFSET)
#define IMX_INPUT_ECSPI2_SS1                  (IMX_IOMUXC_VBASE+IMX_INPUT_ECSPI2_SS1_OFFSET)
#define IMX_INPUT_ECSPI4_SS0                  (IMX_IOMUXC_VBASE+IMX_INPUT_ECSPI4_SS0_OFFSET)
#define IMX_INPUT_ECSPI5_CSPI_CLK_IN          (IMX_IOMUXC_VBASE+IMX_INPUT_ECSPI5_CSPI_CLK_IN_OFFSET)
#define IMX_INPUT_ECSPI5_MISO                 (IMX_IOMUXC_VBASE+IMX_INPUT_ECSPI5_MISO_OFFSET)
#define IMX_INPUT_ECSPI5_MOSI                 (IMX_IOMUXC_VBASE+IMX_INPUT_ECSPI5_MOSI_OFFSET)
#define IMX_INPUT_ECSPI5_SS0                  (IMX_IOMUXC_VBASE+IMX_INPUT_ECSPI5_SS0_OFFSET)
#define IMX_INPUT_ECSPI5_SS1                  (IMX_IOMUXC_VBASE+IMX_INPUT_ECSPI5_SS1_OFFSET)
#define IMX_INPUT_ENET_REF_CLK                (IMX_IOMUXC_VBASE+IMX_INPUT_ENET_REF_CLK_OFFSET)
#define IMX_INPUT_ENET_MAC0_MDIO              (IMX_IOMUXC_VBASE+IMX_INPUT_ENET_MAC0_MDIO_OFFSET)
#define IMX_INPUT_ENET_MAC0_RX_CLK            (IMX_IOMUXC_VBASE+IMX_INPUT_ENET_MAC0_RX_CLK_OFFSET)
#define IMX_INPUT_ENET_MAC0_RX_DATA0          (IMX_IOMUXC_VBASE+IMX_INPUT_ENET_MAC0_RX_DATA0_OFFSET)
#define IMX_INPUT_ENET_MAC0_RX_DATA1          (IMX_IOMUXC_VBASE+IMX_INPUT_ENET_MAC0_RX_DATA1_OFFSET)
#define IMX_INPUT_ENET_MAC0_RX_DATA2          (IMX_IOMUXC_VBASE+IMX_INPUT_ENET_MAC0_RX_DATA2_OFFSET)
#define IMX_INPUT_ENET_MAC0_RX_DATA3          (IMX_IOMUXC_VBASE+IMX_INPUT_ENET_MAC0_RX_DATA3_OFFSET)
#define IMX_INPUT_ENET_MAC0_RX_EN             (IMX_IOMUXC_VBASE+IMX_INPUT_ENET_MAC0_RX_EN_OFFSET)
#define IMX_INPUT_ESAI_RX_FS                  (IMX_IOMUXC_VBASE+IMX_INPUT_ESAI_RX_FS_OFFSET)
#define IMX_INPUT_ESAI_TX_FS                  (IMX_IOMUXC_VBASE+IMX_INPUT_ESAI_TX_FS_OFFSET)
#define IMX_INPUT_ESAI_RX_HF_CLK              (IMX_IOMUXC_VBASE+IMX_INPUT_ESAI_RX_HF_CLK_OFFSET)
#define IMX_INPUT_ESAI_TX_HF_CLK              (IMX_IOMUXC_VBASE+IMX_INPUT_ESAI_TX_HF_CLK_OFFSET)
#define IMX_INPUT_ESAI_RX_CLK                 (IMX_IOMUXC_VBASE+IMX_INPUT_ESAI_RX_CLK_OFFSET)
#define IMX_INPUT_ESAI_TX_CLK                 (IMX_IOMUXC_VBASE+IMX_INPUT_ESAI_TX_CLK_OFFSET)
#define IMX_INPUT_ESAI_SDO0                   (IMX_IOMUXC_VBASE+IMX_INPUT_ESAI_SDO0_OFFSET)
#define IMX_INPUT_ESAI_SDO1                   (IMX_IOMUXC_VBASE+IMX_INPUT_ESAI_SDO1_OFFSET)
#define IMX_INPUT_ESAI_SDO2_SDI3              (IMX_IOMUXC_VBASE+IMX_INPUT_ESAI_SDO2_SDI3_OFFSET)
#define IMX_INPUT_ESAI_SDO3_SDI2              (IMX_IOMUXC_VBASE+IMX_INPUT_ESAI_SDO3_SDI2_OFFSET)
#define IMX_INPUT_ESAI_SDO4_SDI1              (IMX_IOMUXC_VBASE+IMX_INPUT_ESAI_SDO4_SDI1_OFFSET)
#define IMX_INPUT_ESAI_SDO5_SDI0              (IMX_IOMUXC_VBASE+IMX_INPUT_ESAI_SDO5_SDI0_OFFSET)
#define IMX_INPUT_HDMI_ICECIN                 (IMX_IOMUXC_VBASE+IMX_INPUT_HDMI_ICECIN_OFFSET)
#define IMX_INPUT_HDMI_II2C_CLKIN             (IMX_IOMUXC_VBASE+IMX_INPUT_HDMI_II2C_CLKIN_OFFSET)
#define IMX_INPUT_HDMI_II2C_DATAIN            (IMX_IOMUXC_VBASE+IMX_INPUT_HDMI_II2C_DATAIN_OFFSET)
#define IMX_INPUT_I2C1_SCL_IN                 (IMX_IOMUXC_VBASE+IMX_INPUT_I2C1_SCL_IN_OFFSET)
#define IMX_INPUT_I2C1_SDA_IN                 (IMX_IOMUXC_VBASE+IMX_INPUT_I2C1_SDA_IN_OFFSET)
#define IMX_INPUT_I2C2_SCL_IN                 (IMX_IOMUXC_VBASE+IMX_INPUT_I2C2_SCL_IN_OFFSET)
#define IMX_INPUT_I2C2_SDA_IN                 (IMX_IOMUXC_VBASE+IMX_INPUT_I2C2_SDA_IN_OFFSET)
#define IMX_INPUT_I2C3_SCL_IN                 (IMX_IOMUXC_VBASE+IMX_INPUT_I2C3_SCL_IN_OFFSET)
#define IMX_INPUT_I2C3_SDA_IN                 (IMX_IOMUXC_VBASE+IMX_INPUT_I2C3_SDA_IN_OFFSET)
#define IMX_INPUT_IPU2_SENS1_DATA10           (IMX_IOMUXC_VBASE+IMX_INPUT_IPU2_SENS1_DATA10_OFFSET)
#define IMX_INPUT_IPU2_SENS1_DATA11           (IMX_IOMUXC_VBASE+IMX_INPUT_IPU2_SENS1_DATA11_OFFSET)
#define IMX_INPUT_IPU2_SENS1_DATA12           (IMX_IOMUXC_VBASE+IMX_INPUT_IPU2_SENS1_DATA12_OFFSET)
#define IMX_INPUT_IPU2_SENS1_DATA13           (IMX_IOMUXC_VBASE+IMX_INPUT_IPU2_SENS1_DATA13_OFFSET)
#define IMX_INPUT_IPU2_SENS1_DATA14           (IMX_IOMUXC_VBASE+IMX_INPUT_IPU2_SENS1_DATA14_OFFSET)
#define IMX_INPUT_IPU2_SENS1_DATA15           (IMX_IOMUXC_VBASE+IMX_INPUT_IPU2_SENS1_DATA15_OFFSET)
#define IMX_INPUT_IPU2_SENS1_DATA16           (IMX_IOMUXC_VBASE+IMX_INPUT_IPU2_SENS1_DATA16_OFFSET)
#define IMX_INPUT_IPU2_SENS1_DATA17           (IMX_IOMUXC_VBASE+IMX_INPUT_IPU2_SENS1_DATA17_OFFSET)
#define IMX_INPUT_IPU2_SENS1_DATA18           (IMX_IOMUXC_VBASE+IMX_INPUT_IPU2_SENS1_DATA18_OFFSET)
#define IMX_INPUT_IPU2_SENS1_DATA19           (IMX_IOMUXC_VBASE+IMX_INPUT_IPU2_SENS1_DATA19_OFFSET)
#define IMX_INPUT_IPU2_SENS1_DATA_EN          (IMX_IOMUXC_VBASE+IMX_INPUT_IPU2_SENS1_DATA_EN_OFFSET)
#define IMX_INPUT_IPU2_SENS1_HSYNC            (IMX_IOMUXC_VBASE+IMX_INPUT_IPU2_SENS1_HSYNC_OFFSET)
#define IMX_INPUT_IPU2_SENS1_PIX_CLK          (IMX_IOMUXC_VBASE+IMX_INPUT_IPU2_SENS1_PIX_CLK_OFFSET)
#define IMX_INPUT_IPU2_SENS1_VSYNC            (IMX_IOMUXC_VBASE+IMX_INPUT_IPU2_SENS1_VSYNC_OFFSET)
#define IMX_INPUT_KEY_COL5                    (IMX_IOMUXC_VBASE+IMX_INPUT_KEY_COL5_OFFSET)
#define IMX_INPUT_KEY_COL6                    (IMX_IOMUXC_VBASE+IMX_INPUT_KEY_COL6_OFFSET)
#define IMX_INPUT_KEY_COL7                    (IMX_IOMUXC_VBASE+IMX_INPUT_KEY_COL7_OFFSET)
#define IMX_INPUT_KEY_ROW5                    (IMX_IOMUXC_VBASE+IMX_INPUT_KEY_ROW5_OFFSET)
#define IMX_INPUT_KEY_ROW6                    (IMX_IOMUXC_VBASE+IMX_INPUT_KEY_ROW6_OFFSET)
#define IMX_INPUT_KEY_ROW7                    (IMX_IOMUXC_VBASE+IMX_INPUT_KEY_ROW7_OFFSET)
#define IMX_INPUT_MLB_MLB_CLK_IN              (IMX_IOMUXC_VBASE+IMX_INPUT_MLB_MLB_CLK_IN_OFFSET)
#define IMX_INPUT_MLB_MLB_DATA_IN             (IMX_IOMUXC_VBASE+IMX_INPUT_MLB_MLB_DATA_IN_OFFSET)
#define IMX_INPUT_MLB_MLB_SIG_IN              (IMX_IOMUXC_VBASE+IMX_INPUT_MLB_MLB_SIG_IN_OFFSET)
#define IMX_INPUT_SDMA_EVENTS14               (IMX_IOMUXC_VBASE+IMX_INPUT_SDMA_EVENTS14_OFFSET)
#define IMX_INPUT_SDMA_EVENTS47               (IMX_IOMUXC_VBASE+IMX_INPUT_SDMA_EVENTS47_OFFSET)
#define IMX_INPUT_SPDIF_SPDIF_IN1             (IMX_IOMUXC_VBASE+IMX_INPUT_SPDIF_SPDIF_IN1_OFFSET)
#define IMX_INPUT_SPDIF_TX_CLK2               (IMX_IOMUXC_VBASE+IMX_INPUT_SPDIF_TX_CLK2_OFFSET)
#define IMX_INPUT_UART1_UART_RTS_B            (IMX_IOMUXC_VBASE+IMX_INPUT_UART1_UART_RTS_B_OFFSET)
#define IMX_INPUT_UART1_UART_RX_DATA          (IMX_IOMUXC_VBASE+IMX_INPUT_UART1_UART_RX_DATA_OFFSET)
#define IMX_INPUT_UART2_UART_RTS_B            (IMX_IOMUXC_VBASE+IMX_INPUT_UART2_UART_RTS_B_OFFSET)
#define IMX_INPUT_UART2_UART_RX_DATA          (IMX_IOMUXC_VBASE+IMX_INPUT_UART2_UART_RX_DATA_OFFSET)
#define IMX_INPUT_UART3_UART_RTS_B            (IMX_IOMUXC_VBASE+IMX_INPUT_UART3_UART_RTS_B_OFFSET)
#define IMX_INPUT_UART3_UART_RX_DATA          (IMX_IOMUXC_VBASE+IMX_INPUT_UART3_UART_RX_DATA_OFFSET)
#define IMX_INPUT_UART4_UART_RTS_B            (IMX_IOMUXC_VBASE+IMX_INPUT_UART4_UART_RTS_B_OFFSET)
#define IMX_INPUT_UART4_UART_RX_DATA          (IMX_IOMUXC_VBASE+IMX_INPUT_UART4_UART_RX_DATA_OFFSET)
#define IMX_INPUT_UART5_UART_RTS_B            (IMX_IOMUXC_VBASE+IMX_INPUT_UART5_UART_RTS_B_OFFSET)
#define IMX_INPUT_UART5_UART_RX_DATA          (IMX_IOMUXC_VBASE+IMX_INPUT_UART5_UART_RX_DATA_OFFSET)
#define IMX_INPUT_USB_OTG_OC                  (IMX_IOMUXC_VBASE+IMX_INPUT_USB_OTG_OC_OFFSET)
#define IMX_INPUT_USB_H1_OC                   (IMX_IOMUXC_VBASE+IMX_INPUT_USB_H1_OC_OFFSET)
#define IMX_INPUT_USDHC1_WP_ON                (IMX_IOMUXC_VBASE+IMX_INPUT_USDHC1_WP_ON_OFFSET)

/* IOMUXC Register Bit Definitions **************************************************/

/* General Purpose Register 0 (GPR0) */

#define GPR0_DMAREQ_MUX_SEL0                            (1 << 0)
#define GPR0_DMAREQ_MUX_SEL1                            (1 << 1)
#define GPR0_DMAREQ_MUX_SEL2                            (1 << 2)
#define GPR0_DMAREQ_MUX_SEL3                            (1 << 3)
#define GPR0_DMAREQ_MUX_SEL4                            (1 << 4)
#define GPR0_DMAREQ_MUX_SEL5                            (1 << 5)
#define GPR0_DMAREQ_MUX_SEL6                            (1 << 6)
#define GPR0_DMAREQ_MUX_SEL7                            (1 << 7)
#define GPR0_AUDIO_VIDEO_MUXING_SHIFT                   (8)
#define GPR0_AUDIO_VIDEO_MUXING_MASK                    (0x3f << GPR0_TX_CLK2_MUX_SEL_SHIFT)
#define GPR0_TX_CLK2_MUX_SEL_SHIFT                      (14)
#define GPR0_TX_CLK2_MUX_SEL_MASK                       (3 << GPR0_TX_CLK2_MUX_SEL_SHIFT)
#  define GPR0_TX_CLK2_MUX_SEL_ASRCK_CLK1               (0 << GPR0_TX_CLK2_MUX_SEL_SHIFT)
#  define GPR0_TX_CLK2_MUX_SEL_ASRCK_CLK2               (1 << GPR0_TX_CLK2_MUX_SEL_SHIFT)
#  define GPR0_TX_CLK2_MUX_SEL_ASRCK_CLK3               (2 << GPR0_TX_CLK2_MUX_SEL_SHIFT)
#define GPR0_CLOCK_1_MUX_SEL_SHIFT                      (16)
#define GPR0_CLOCK_1_MUX_SEL_MASK                       (3 << GPR0_CLOCK_1_MUX_SEL_SHIFT)
#  define GPR0_CLOCK_1_MUX_SEL_AUDMUX_RXCLK_P1_MUXED    (0 << GPR0_CLOCK_1_MUX_SEL_SHIFT)
#  define GPR0_CLOCK_1_MUX_SEL_AUDMUX_RXCLK_P1          (1 << GPR0_CLOCK_1_MUX_SEL_SHIFT)
#  define GPR0_CLOCK_1_MUX_SEL_SSI1_SSI_SRCK            (2 << GPR0_CLOCK_1_MUX_SEL_SHIFT)
#  define GPR0_CLOCK_1_MUX_SEL_SSI1_SSI_RX_BIT_CLK      (3 << GPR0_CLOCK_1_MUX_SEL_SHIFT)
#define GPR0_CLOCK_9_MUX_SEL_SHIFT                      (3 << 18)
#define GPR0_CLOCK_9_MUX_SEL_MASK                       (3 << GPR0_CLOCK_9_MUX_SEL_SHIFT)
#  define GPR0_CLOCK_9_MUX_SEL_AUDMUX_TXCLK_P1_MUXED    (0 << GPR0_CLOCK_9_MUX_SEL_SHIFT)
#  define GPR0_CLOCK_9_MUX_SEL_AUDMUX_TXCLK_P1          (1 << GPR0_CLOCK_9_MUX_SEL_SHIFT)
#  define GPR0_CLOCK_9_MUX_SEL_SSI1_SSI_STCK            (2 << GPR0_CLOCK_9_MUX_SEL_SHIFT)
#  define GPR0_CLOCK_9_MUX_SEL_SSI1_SSI_TX_BIT_CLK      (3 << GPR0_CLOCK_9_MUX_SEL_SHIFT)
#define GPR0_CLOCK_2_MUX_SEL_SHIFT                      (20)
#define GPR0_CLOCK_2_MUX_SEL_MASK                       (3 << GPR0_CLOCK_2_MUX_SEL_SHIFT)
#  define GPR0_CLOCK_2_MUX_SEL_AUDMUX_RXCLK_P2_MUXED    (0 << GPR0_CLOCK_2_MUX_SEL_SHIFT)
#  define GPR0_CLOCK_2_MUX_SEL_AUDMUX_RXCLK_P2          (1 << GPR0_CLOCK_2_MUX_SEL_SHIFT)
#  define GPR0_CLOCK_2_MUX_SEL_SSI2_SSI_SRCK            (2 << GPR0_CLOCK_2_MUX_SEL_SHIFT)
#  define GPR0_CLOCK_2_MUX_SEL_SSI2_RX_BIT_CLK          (3 << GPR0_CLOCK_2_MUX_SEL_SHIFT)
#define GPR0_CLOCK_A_MUX_SEL_SHIFT                      (22)
#define GPR0_CLOCK_A_MUX_SEL_MASK                       (3 << GPR0_CLOCK_A_MUX_SEL_SHIFT)
#  define GPR0_CLOCK_A_MUX_SEL_AUDMUX_TXCLK_P2_MUXED    (0 << GPR0_CLOCK_A_MUX_SEL_SHIFT)
#  define GPR0_CLOCK_A_MUX_SEL_AUDMUX_TXCLK_P2          (1 << GPR0_CLOCK_A_MUX_SEL_SHIFT)
#  define GPR0_CLOCK_A_MUX_SEL_SSI2_SSI_STCK            (2 << GPR0_CLOCK_A_MUX_SEL_SHIFT)
#  define GPR0_CLOCK_A_MUX_SEL_SSI2_TX_BIT_CLK          (3 << GPR0_CLOCK_A_MUX_SEL_SHIFT)
#define GPR0_CLOCK_3_MUX_SEL_SHIFT                      (24)
#define GPR0_CLOCK_3_MUX_SEL_MASK                       (3 << GPR0_CLOCK_3_MUX_SEL_SHIFT)
#  define GPR0_CLOCK_3_MUX_SEL_AUDMUX_RXCLK_P7_MUXED    (3 << GPR0_CLOCK_3_MUX_SEL_SHIFT)
#  define GPR0_CLOCK_3_MUX_SEL_AUDMUX_RXCLK_P7          (3 << GPR0_CLOCK_3_MUX_SEL_SHIFT)
#  define GPR0_CLOCK_3_MUX_SEL_SSI3_SSI_SRCK            (3 << GPR0_CLOCK_3_MUX_SEL_SHIFT)
#  define GPR0_CLOCK_3_MUX_SEL_SSI3_RX_BIT_CLK          (3 << GPR0_CLOCK_3_MUX_SEL_SHIFT)
#define GPR0_CLOCK_B_MUX_SEL_SHIFT                      (26)
#define GPR0_CLOCK_B_MUX_SEL_MASK                       (3 << GPR0_CLOCK_B_MUX_SEL_SHIFT)
#  define GPR0_CLOCK_B_MUX_SEL_AUDMUX_TXCLK_P7_MUXED    (0 << GPR0_CLOCK_B_MUX_SEL_SHIFT)
#  define GPR0_CLOCK_B_MUX_SEL_AUDMUX_TXCLK_P7          (1 << GPR0_CLOCK_B_MUX_SEL_SHIFT)
#  define GPR0_CLOCK_B_MUX_SEL_SSI3_SSI_STCK            (2 << GPR0_CLOCK_B_MUX_SEL_SHIFT)
#  define GPR0_CLOCK_B_MUX_SEL_SSI3_TX_BIT_CLK          (3 << GPR0_CLOCK_B_MUX_SEL_SHIFT)
#define GPR0_CLOCK_0_MUX_SEL_SHIFT                      (28)
#define GPR0_CLOCK_0_MUX_SEL_MASK                       (3 << GPR0_CLOCK_0_MUX_SEL_SHIFT)
#  define GPR0_CLOCK_0_MUX_SEL_ESAI1_IPP_IND_SCKR_MUXED (0 << GPR0_CLOCK_0_MUX_SEL_SHIFT)
#  define GPR0_CLOCK_0_MUX_SEL_ESAI1_IPP_IND_SCKR       (1 << GPR0_CLOCK_0_MUX_SEL_SHIFT)
#  define GPR0_CLOCK_0_MUX_SEL_ESAI1_IPP_DO_SCKR        (2 << GPR0_CLOCK_0_MUX_SEL_SHIFT)
#define GPR0_CLOCK_8_MUX_SEL_SHIFT                      (30)
#define GPR0_CLOCK_8_MUX_SEL_MASK                       (3 << GPR0_CLOCK_8_MUX_SEL_SHIFT)
#  define GPR0_CLOCK_8_MUX_SEL_AUDMUX_RXCLK_P7_MUXED    (0 << GPR0_CLOCK_8_MUX_SEL_SHIFT)
#  define GPR0_CLOCK_8_MUX_SEL_AUDMUX_RXCLK_P7          (1 << GPR0_CLOCK_8_MUX_SEL_SHIFT)
#  define GPR0_CLOCK_8_MUX_SEL_SSI3_SSI_SRCK            (2 << GPR0_CLOCK_8_MUX_SEL_SHIFT)
#  define GPR0_CLOCK_8_MUX_SEL_SSI3_RX_BIT_CLK          (3 << GPR0_CLOCK_8_MUX_SEL_SHIFT)
#define GPR1_REF_SSP_EN                                 (1 << 16)

/* General Purpose Register 1 (GPR1) */

#define GPR1_ACT_CS0                          (1 << 0)
#define GPR1_ADDRS0_SHIFT                     (1)
#define GPR1_ADDRS0_MASK                      (3 << GPR1_ADDRS0_SHIFT)
#  define GPR1_ADDRS0(n)                      ((uint32_t)(n) << GPR1_ADDRS0_SHIFT)
#define GPR1_ACT_CS1                          (1 << 3)
#define GPR1_ADDRS1_SHIFT                     (4)
#define GPR1_ADDRS1_MASK                      (3 << GPR1_ADDRS1_SHIFT)
#  define GPR1_ADDRS1(n)                      ((uint32_t)(n) << GPR1_ADDRS1_SHIFT)
#define GPR1_ACT_CS2                          (1 << 6)
#define GPR1_ADDRS2_SHFIT                     (7)
#define GPR1_ADDRS2_MASK                      (3 << GPR1_ADDRS2_SHFIT)
#  define GPR1_ADDRS2(n)                      ((uint32_t)(n) << GPR1_ADDRS2_SHFIT)
#define GPR1_ACT_CS3                          (1 << 9)
#define GPR1_ADDRS3_SHIFT                     (10)
#define GPR1_ADDRS3_MASK                      (3 << GPR1_ADDRS3_SHIFT)
#  define GPR1_ADDRS3_32MB                    (0 << GPR1_ADDRS3_SHIFT)
#  define GPR1_ADDRS3_64MB                    (1 << GPR1_ADDRS3_SHIFT)
#  define GPR1_ADDRS3_128MB                   (2 << GPR1_ADDRS3_SHIFT)
#define GPR1_GINT                             (1 << 12)
#define GPR1_USB_OTG_ID_SEL                   (1 << 13)
#define GPR1_SYS_INT                          (1 << 14)
#define GPR1_USB_EXP_MODE                     (1 << 15)
#define GPR1_IPU_VPU_MUX                      (1 << 17)
#define GPR1_TEST_POWERDOWN                   (1 << 18)
#define GPR1_MIPI_IPU1_MUX                    (1 << 19)
#define GPR1_MIPI_IPU2_MUX                    (1 << 20)
#define GPR1_ENET_CLK_SEL                     (1 << 21)
#define GPR1_EXC_MON                          (1 << 22)
#define GPR1_MIPI_DPI_OFF                     (1 << 24)
#define GPR1_MIPI_COLOR_SW                    (1 << 25)
#define GPR1_APP_REQ_ENTR_L1                  (1 << 26)
#define GPR1_APP_READY_ENTR_L23               (1 << 27)
#define GPR1_APP_REQ_EXIT_L1                  (1 << 28)
#define GPR1_APP_CLK_REQ_N                    (1 << 30)
#define GPR1_CFG_L1_CLK_REMOVAL_EN            (1 << 31)

/* General Purpose Register 2 (GPR2) */

#define GPR2_CH0_MODE_SHIFT                   (0)
#define GPR2_CH0_MODE_MASK                    (3 << GPR2_CH0_MODE_SHIFT)
#  define GPR2_CH0_MODE_DISABLED              (0 << GPR2_CH0_MODE_SHIFT)
#  define GPR2_CH0_MODE_ROUTED_DI0            (1 << GPR2_CH0_MODE_SHIFT)
#  define GPR2_CH0_MODE_ROUTED_DI1            (3 << GPR2_CH0_MODE_SHIFT)
#define GPR2_CH1_MODE_SHIFT                   (2)
#define GPR2_CH1_MODE_MASK                    (3 << GPR2_CH1_MODE_SHIFT)
#  define GPR2_CH1_MODE_DISABLED              (0 << GPR2_CH1_MODE_SHIFT)
#  define GPR2_CH1_MODE_ROUTED_DI0            (1 << GPR2_CH1_MODE_SHIFT)
#  define GPR2_CH1_MODE_ROUTED_DI1            (3 << GPR2_CH1_MODE_SHIFT)
#define GPR2_SPLIT_MODE_EN                    (1 << 4)
#define GPR2_DATA_WIDTH_CH0                   (1 << 5)
#define GPR2_BIT_MAPPING_CH0                  (1 << 6)
#define GPR2_DATA_WIDTH_CH1                   (1 << 7)
#define GPR2_BIT_MAPPING_CH1                  (1 << 8)
#define GPR2_DI0_VS_POLARITY                  (1 << 9)
#define GPR2_DI1_VS_POLARITY                  (1 << 10)
#define GPR2_LVDS_CLK_SHIFT_SHIFT             (16)
#define GPR2_LVDS_CLK_SHIFT_MASK              (7 << GPR2_LVDS_CLK_SHIFT_SHIFT)
#  define GPR2_LVDS_CLK_SHIFT0                (0 << GPR2_LVDS_CLK_SHIFT_SHIFT)
#  define GPR2_LVDS_CLK_SHIFT1                (1 << GPR2_LVDS_CLK_SHIFT_SHIFT)
#  define GPR2_LVDS_CLK_SHIFT2                (2 << GPR2_LVDS_CLK_SHIFT_SHIFT)
#  define GPR2_LVDS_CLK_SHIFT3                (3 << GPR2_LVDS_CLK_SHIFT_SHIFT)
#  define GPR2_LVDS_CLK_SHIFT4                (4 << GPR2_LVDS_CLK_SHIFT_SHIFT)
#  define GPR2_LVDS_CLK_SHIFT5                (5 << GPR2_LVDS_CLK_SHIFT_SHIFT)
#  define GPR2_LVDS_CLK_SHIFT6                (6 << GPR2_LVDS_CLK_SHIFT_SHIFT)
#  define GPR2_LVDS_CLK_SHIFT7                (7 << GPR2_LVDS_CLK_SHIFT_SHIFT)
#define GPR2_COUNTER_RESET_VAL_SHIFT          (20)
#define GPR2_COUNTER_RESET_VAL_MASK           (3 << GPR2_COUNTER_RESET_VAL_SHIFT)
#  define GPR2_COUNTER_RESET_VAL5             (0 << GPR2_COUNTER_RESET_VAL_SHIFT)
#  define GPR2_COUNTER_RESET_VAL3             (1 << GPR2_COUNTER_RESET_VAL_SHIFT)
#  define GPR2_COUNTER_RESET_VAL4             (2 << GPR2_COUNTER_RESET_VAL_SHIFT)
#  define GPR2_COUNTER_RESET_VAL6             (3 << GPR2_COUNTER_RESET_VAL_SHIFT)

/* General Purpose Register 3 (GPR3) */

#define GPR3_HDMI_MUX_CTL_SHIFT               (2)
#define GPR3_HDMI_MUX_CTL_MASK                (3 << GPR3_HDMI_MUX_CTL_SHIFT)
#  define GPR3_HDMI_MUX_CTL_IPU1_DI0          (0 << GPR3_HDMI_MUX_CTL_SHIFT)
#  define GPR3_HDMI_MUX_CTL_IPU1_DI1          (1 << GPR3_HDMI_MUX_CTL_SHIFT)
#  define GPR3_HDMI_MUX_CTL_IPU2_DI0          (2 << GPR3_HDMI_MUX_CTL_SHIFT)
#  define GPR3_HDMI_MUX_CTL_IPU2_DI1          (3 << GPR3_HDMI_MUX_CTL_SHIFT)
#define GPR3_MIPI_MUX_CTL_SHIFT               (4)
#define GPR3_MIPI_MUX_CTL_MASK                (3 << GPR3_MIPI_MUX_CTL_SHIFT)
#  define GPR3_MIPI_MUX_CTL_IPU1_DI0          (0 << GPR3_MIPI_MUX_CTL_SHIFT)
#  define GPR3_MIPI_MUX_CTL_IPU1_DI1          (1 << GPR3_MIPI_MUX_CTL_SHIFT)
#  define GPR3_MIPI_MUX_CTL_IPU2_DI0          (2 << GPR3_MIPI_MUX_CTL_SHIFT)
#  define GPR3_MIPI_MUX_CTL_IPU2_DI1          (3 << GPR3_MIPI_MUX_CTL_SHIFT)
#define GPR3_LVDS0_MUX_CTL_SHIFT              (6)
#define GPR3_LVDS0_MUX_CTL_MASK               (3 << GPR3_LVDS0_MUX_CTL_SHIFT)
#  define GPR3_LVDS0_MUX_CTL_IPU1_DI0         (0 << GPR3_LVDS0_MUX_CTL_SHIFT)
#  define GPR3_LVDS0_MUX_CTL_IPU1_DI1         (1 << GPR3_LVDS0_MUX_CTL_SHIFT)
#  define GPR3_LVDS0_MUX_CTL_IPU2_DI0         (2 << GPR3_LVDS0_MUX_CTL_SHIFT)
#  define GPR3_LVDS0_MUX_CTL_IPU2_DI1         (3 << GPR3_LVDS0_MUX_CTL_SHIFT)
#define GPR3_LVDS1_MUX_CTL_SHIFT              (8)
#define GPR3_LVDS1_MUX_CTL_MASK               (3 << GPR3_LVDS1_MUX_CTL_SHIFT)
#  define GPR3_LVDS1_MUX_CTL_IPU1_DI0         (0 << GPR3_LVDS1_MUX_CTL_SHIFT)
#  define GPR3_LVDS1_MUX_CTL_IPU1_DI1         (1 << GPR3_LVDS1_MUX_CTL_SHIFT)
#  define GPR3_LVDS1_MUX_CTL_IPU2_DI0         (2 << GPR3_LVDS1_MUX_CTL_SHIFT)
#  define GPR3_LVDS1_MUX_CTL_IPU2_DI1         (3 << GPR3_LVDS1_MUX_CTL_SHIFT)
#define GPR3_IPU_DIAG                         (1 << 10)
#define GPR3_TZASC1_BOOT_LOCK                 (1 << 11)
#define GPR3_TZASC2_BOOT_LOCK                 (1 << 12)
#define GPR3_CORE0_DBG_ACK_EN                 (1 << 13)
#define GPR3_CORE1_DBG_ACK_EN                 (1 << 14)
#define GPR3_CORE2_DBG_ACK_EN                 (1 << 15)
#define GPR3_CORE3_DBG_ACK_EN                 (1 << 16)
#define GPR3_OCRAM_STATUS_SHIFT               (17)
#define GPR3_OCRAM_STATUS_MASK                (15 << GPR3_OCRAM_STATUS_SHIFT)
#  define GPR3_OCRAM_STATUS(n)                ((uint32_t)(n) << GPR3_OCRAM_STATUS_SHIFT)
#define GPR3_OCRAM_CTL_SHIFT                  (21)
#define GPR3_OCRAM_CTL_MASK                   (15 << GPR3_OCRAM_CTL_SHIFT)
#  define GPR3_OCRAM_CTL(n)                   ((uint32_t)(n) << GPR3_OCRAM_CTL_SHIFT)
#define GPR3_USDHCX_WR_CACHE_CTL              (1 << 26)
#define GPR3_USDHCX_RD_CACHE_CTL              (1 << 25)
#define GPR3_BCH_RD_CACHE_CTL                 (1 << 27)
#define GPR3_BCH_WR_CACHE_CTL                 (1 << 28)
#define GPR3_GPU_DBG_SHIFT                    (29)
#define GPR3_GPU_DBG_MASK                     (3 << GPR3_GPU_DBG_SHIFT)
#  define GPR3_GPU_DBG_GPU3D                  (0 << GPR3_GPU_DBG_SHIFT)
#  define GPR3_GPU_DBG_GPU2D                  (1 << GPR3_GPU_DBG_SHIFT)
#  define GPR3_GPU_DBG_OPENVG                 (2 << GPR3_GPU_DBG_SHIFT)

/* General Purpose Register 4 (GPR4) */

#define GPR4_IPU_RD_CACHE_CTL                 (1 << 0)
#define GPR4_IPU_WR_CACHE_CTL                 (1 << 1)
#define GPR4_VPU_P_RD_CACHE_VAL               (1 << 2)
#define GPR4_VPU_P_WR_CACHE_VAL               (1 << 3)
#define GPR4_VPU_RD_CACHE_SEL                 (1 << 6)
#define GPR4_VPU_WR_CACHE_SEL                 (1 << 7)
#define GPR4_SOC_VERSION_SHIFT                (8)
#define GPR4_SOC_VERSION_MASK                 (0xff << GPR4_SOC_VERSION_SHIFT)
#define GPR4_ENET_STOP_ACK                    (1 << 16)
#define GPR4_CAN1_STOP_ACK                    (1 << 17)
#define GPR4_CAN2_STOP_ACK                    (1 << 18)
#define GPR4_SDMA_STOP_ACK                    (1 << 19)
#define GPR4_PCIE_RD_CACHE_VAL                (1 << 24)
#define GPR4_PCIE_WR_CACHE_VAL                (1 << 25)
#define GPR4_PCIE_RD_CACHE_SEL                (1 << 26)
#define GPR4_PCIE_WR_CACHE_SEL                (1 << 27)
#define GPR4_VDOA_RD_CACHE_VAL                (1 << 28)
#define GPR4_VDOA_WR_CACHE_VAL                (1 << 29)
#define GPR4_VDOA_RD_CACHE_SEL                (1 << 30)
#define GPR4_VDOA_WR_CACHE_SEL                (1 << 31)

/* General Purpose Register 5 (GPR5) */

#define GPR5_ARM_WFI_SHIFT                    (0)
#define GPR5_ARM_WFI_MASK                     (15 << GPR5_ARM_WFI_SHIFT)
#define GPR5_ARM_WFE_SHIFT                    (4)
#define GPR5_ARM_WFE_MASK                     (15 << GPR5_ARM_WFE_SHIFT)
#define GPR5_L2_CLK_STOP                      (1 << 8)

/* General Purpose Register 6 (GPR6) */

#define GPR6_IPU1_ID00_WR_QOS_SHIFT           (0)
#define GPR6_IPU1_ID00_WR_QOS_MASK            (15 << GPR6_IPU1_ID00_WR_QOS_SHIFT)
#  define GPR6_IPU1_ID00_WR_QOS(n)            ((uint32_t)(n) << GPR6_IPU1_ID00_WR_QOS_SHIFT)
#define GPR6_IPU1_ID01_WR_QOS_SHIFT           (4)
#define GPR6_IPU1_ID01_WR_QOS_MASK            (15 << GPR6_IPU1_ID01_WR_QOS_SHIFT)
#  define GPR6_IPU1_ID01_WR_QOS(n)            ((uint32_t)(n) << GPR6_IPU1_ID01_WR_QOS_SHIFT)
#define GPR6_IPU1_ID10_WR_QOS_SHIFT           (8)
#define GPR6_IPU1_ID10_WR_QOS_MASK            (15 << GPR6_IPU1_ID10_WR_QOS_SHIFT)
#  define GPR6_IPU1_ID10_WR_QOS(n)            ((uint32_t)(n) << GPR6_IPU1_ID10_WR_QOS_SHIFT)
#define GPR6_IPU1_ID11_WR_QOS_SHIFT           (12)
#define GPR6_IPU1_ID11_WR_QOS_MASK            (15 << GPR6_IPU1_ID11_WR_QOS_SHIFT)
#  define GPR6_IPU1_ID11_WR_QOS(n)            ((uint32_t)(n) << GPR6_IPU1_ID11_WR_QOS_SHIFT)
#define GPR6_IPU1_ID00_RD_QOS_SHIFT           (16)
#define GPR6_IPU1_ID00_RD_QOS_MASK            (15 << GPR6_IPU1_ID00_RD_QOS_SHIFT)
#  define GPR6_IPU1_ID00_RD_QOS(n)            ((uint32_t)(n) << GPR6_IPU1_ID00_RD_QOS_SHIFT)
#define GPR6_IPU1_ID01_RD_QOS_SHIFT           (20)
#define GPR6_IPU1_ID01_RD_QOS_MASK            (15 << GPR6_IPU1_ID01_RD_QOS_SHIFT)
#  define GPR6_IPU1_ID01_RD_QOS(n)            ((uint32_t)(n) << GPR6_IPU1_ID01_RD_QOS_SHIFT)
#define GPR6_IPU1_ID10_RD_QOS_SHIFT           (24)
#define GPR6_IPU1_ID10_RD_QOS_MASK            (15 << GPR6_IPU1_ID10_RD_QOS_SHIFT)
#  define GPR6_IPU1_ID10_RD_QOS(n)            ((uint32_t)(n) << GPR6_IPU1_ID10_RD_QOS_SHIFT)
#define GPR6_IPU1_ID11_RD_QOS_SHIFT           (28)
#define GPR6_IPU1_ID11_RD_QOS_MASK            (15 << GPR6_IPU1_ID11_RD_QOS_SHIFT)
#  define GPR6_IPU1_ID11_RD_QOS(n)            ((uint32_t)(n) << GPR6_IPU1_ID11_RD_QOS_SHIFT)

/* General Purpose Register 7 (GPR7) */

#define GPR7_IPU2_ID00_WR_QOS_SHIFT           (0)
#define GPR7_IPU2_ID00_WR_QOS_MASK            (15 << GPR7_IPU2_ID00_WR_QOS_SHIFT)
#  define GPR7_IPU2_ID00_WR_QOS(n)            ((uint32_t)(n) << GPR7_IPU2_ID00_WR_QOS_SHIFT)
#define GPR7_IPU2_ID01_WR_QOS_SHIFT           (4)
#define GPR7_IPU2_ID01_WR_QOS_MASK            (15 << GPR7_IPU2_ID01_WR_QOS_SHIFT)
#  define GPR7_IPU2_ID01_WR_QOS(n)            ((uint32_t)(n) << GPR7_IPU2_ID01_WR_QOS_SHIFT)
#define GPR7_IPU2_ID10_WR_QOS_SHIFT           (8)
#define GPR7_IPU2_ID10_WR_QOS_MASK            (15 << GPR7_IPU2_ID10_WR_QOS_SHIFT)
#  define GPR7_IPU2_ID10_WR_QOS(n)            ((uint32_t)(n) << GPR7_IPU2_ID10_WR_QOS_SHIFT)
#define GPR7_IPU2_ID11_WR_QOS_SHIFT           (12)
#define GPR7_IPU2_ID11_WR_QOS_MASK            (15 << GPR7_IPU2_ID11_WR_QOS_SHIFT)
#  define GPR7_IPU2_ID11_WR_QOS(n)            ((uint32_t)(n) << GPR7_IPU2_ID11_WR_QOS_SHIFT)
#define GPR7_IPU2_ID00_RD_QOS_SHIFT           (16)
#define GPR7_IPU2_ID00_RD_QOS_MASK            (15 << GPR7_IPU2_ID00_RD_QOS_SHIFT)
#  define GPR7_IPU2_ID00_RD_QOS(n)            ((uint32_t)(n) << GPR7_IPU2_ID00_RD_QOS_SHIFT)
#define GPR7_IPU2_ID01_RD_QOS_SHIFT           (20)
#define GPR7_IPU2_ID01_RD_QOS_MASK            (15 << GPR7_IPU2_ID01_RD_QOS_SHIFT)
#  define GPR7_IPU2_ID01_RD_QOS(n)            ((uint32_t)(n) << GPR7_IPU2_ID01_RD_QOS_SHIFT)
#define GPR7_IPU2_ID10_RD_QOS_SHIFT           (24)
#define GPR7_IPU2_ID10_RD_QOS_MASK            (15 << GPR7_IPU2_ID10_RD_QOS_SHIFT)
#  define GPR7_IPU2_ID10_RD_QOS(n)            ((uint32_t)(n) << GPR7_IPU2_ID10_RD_QOS_SHIFT)
#define GPR7_IPU2_ID11_RD_QOS_SHIFT           (28)
#define GPR7_IPU2_ID11_RD_QOS_MASK            (15 << GPR7_IPU2_ID11_RD_QOS_SHIFT)
#  define GPR7_IPU2_ID11_RD_QOS(n)            ((uint32_t)(n) << GPR7_IPU2_ID11_RD_QOS_SHIFT)

/* General Purpose Register 8 (GPR8) */

#define GPR8_PCS_TX_DEEMPH_GEN1_SHIFT         (0)
#define GPR8_PCS_TX_DEEMPH_GEN1_MASK          (0x3f << GPR8_PCS_TX_DEEMPH_GEN1_SHIFT)
#  define GPR8_PCS_TX_DEEMPH_GEN1(n)          ((uint32_t)(n) << GPR8_PCS_TX_DEEMPH_GEN1_SHIFT)
#define GPR8_PCS_TX_DEEMPH_GEN2_3P5DB_SHIFT   (6)
#define GPR8_PCS_TX_DEEMPH_GEN2_3P5DB_MASK    (0x3f << GPR8_PCS_TX_DEEMPH_GEN2_3P5DB_SHIFT)
#  define GPR8_PCS_TX_DEEMPH_GEN2_3P5DB(n)    ((uint32_t)(n) << GPR8_PCS_TX_DEEMPH_GEN2_3P5DB_SHIFT)
#define GPR8_PCS_TX_DEEMPH_GEN2_6DB_SHIFT     (12)
#define GPR8_PCS_TX_DEEMPH_GEN2_6DB_MASK      (0x3f << GPR8_PCS_TX_DEEMPH_GEN2_6DB_SHIFT)
#  define GPR8_PCS_TX_DEEMPH_GEN2_6DB(n)      ((uint32_t)(n) << GPR8_PCS_TX_DEEMPH_GEN2_6DB_SHIFT)
#define GPR8_PCS_TX_SWING_FULL_SHIFT          (18)
#define GPR8_PCS_TX_SWING_FULL_MASK           (0x7f << GPR8_PCS_TX_SWING_FULL_SHIFT)
#  define GPR8_PCS_TX_SWING_FULL(n)           ((uint32_t)(n) << GPR8_PCS_TX_SWING_FULL_SHIFT)
#define GPR8_PCS_TX_SWING_LOW_SHIFT           (25)
#define GPR8_PCS_TX_SWING_LOW_MASK            (0x7f << GPR8_PCS_TX_SWING_LOW_SHIFT)
#  define GPR8_PCS_TX_SWING_LOW(n)            ((uint32_t)(n) << GPR8_PCS_TX_SWING_LOW_SHIFT)

/* General Purpose Register 9 (GPR9) */

#define GPR9_TZASC1_BYP                       (1 << 0)
#define GPR9_TZASC2_BYP                       (1 << 1)

/* General Purpose Register 10 (GPR10) */

#define GPR10_DCIC1_MUX_CTL_SHIFT             (0)
#define GPR10_DCIC1_MUX_CTL_MASK              (3 << GPR10_DCIC1_MUX_CTL_SHIFT)
#  define GPR10_DCIC1_MUX_CTL_IPU1_DI0        (0 << GPR10_DCIC1_MUX_CTL_SHIFT)
#  define GPR10_DCIC1_MUX_CTL_IPU1_DI1        (1 << GPR10_DCIC1_MUX_CTL_SHIFT)
#  define GPR10_DCIC1_MUX_CTL_IPU2_DI0        (2 << GPR10_DCIC1_MUX_CTL_SHIFT)
#  define GPR10_DCIC1_MUX_CTL_IPU2_DI1        (3 << GPR10_DCIC1_MUX_CTL_SHIFT)
#define GPR10_DCIC2_MUX_CTL_SHIFT             (2)
#define GPR10_DCIC2_MUX_CTL_MASK              (3 << GPR10_DCIC2_MUX_CTL_SHIFT)
#  define GPR10_DCIC2_MUX_CTL_IPU1_DI0        (0 << GPR10_DCIC2_MUX_CTL_SHIFT)
#  define GPR10_DCIC2_MUX_CTL_IPU1_DI1        (1 << GPR10_DCIC2_MUX_CTL_SHIFT)
#  define GPR10_DCIC2_MUX_CTL_IPU2_DI0        (2 << GPR10_DCIC2_MUX_CTL_SHIFT)
#  define GPR10_DCIC2_MUX_CTL_IPU2_DI1        (3 << GPR10_DCIC2_MUX_CTL_SHIFT)
#define GPR10_OCRAM_TZ_EN                     (1 << 4)
#define GPR10_OCRAM_TZ_ADDR_SHIFT             (5)
#define GPR10_OCRAM_TZ_ADDR_MASK              (0x3f << GPR10_OCRAM_TZ_ADDR_SHIFT)
#  define GPR10_OCRAM_TZ_ADDR(n)              ((uint32_t)(n) << GPR10_OCRAM_TZ_ADDR_SHIFT)
#define GPR10_SEC_ERR_RESP                    (1 << 11)
#define GPR10_DBG_CLK_EN                      (1 << 12)
#define GPR10_DBG_EN                          (1 << 13)
#define GPR10_LOCK_DCIC1_MUX_SHIFT            (16)
#define GPR10_LOCK_DCIC1_MUX_MASK             (3 << GPR10_LOCK_DCIC1_MUX_SHIFT)
#  define GPR10_LOCK_DCIC1_MUX_IPU1           (0 << GPR10_LOCK_DCIC1_MUX_SHIFT) /* DCIC-1 source is IPU1 or IPU2 DI0 port */
#  define GPR10_LOCK_DCIC1_MUX_LVDS0          (1 << GPR10_LOCK_DCIC1_MUX_SHIFT) /* DCIC-1 source is LVDS0 */
#  define GPR10_LOCK_DCIC1_MUX_LVDS1          (2 << GPR10_LOCK_DCIC1_MUX_SHIFT) /* DCIC-1 source is LVDS1 */
#  define GPR10_LOCK_DCIC1_MUX_MIPI           (3 << GPR10_LOCK_DCIC1_MUX_SHIFT) /* DCIC-1 source is MIPI DPI */
#define GPR10_LOCK_DCIC2_MUX_SHIFT            (18)
#define GPR10_LOCK_DCIC2_MUX_MASK             (3 << GPR10_LOCK_DCIC2_MUX_SHIFT)
#  define GPR10_LOCK_DCIC2_MUX_IPU1           (0 << GPR10_LOCK_DCIC2_MUX_SHIFT) /* DCIC-2 source is IPU1 DI1 port */
#  define GPR10_LOCK_DCIC2_MUX_LVDS0          (1 << GPR10_LOCK_DCIC2_MUX_SHIFT) /* DCIC-2 source is LVDS0 */
#  define GPR10_LOCK_DCIC2_MUX_LVDS1          (2 << GPR10_LOCK_DCIC2_MUX_SHIFT) /* DCIC-2 source is LVDS1 */
#  define GPR10_LOCK_DCIC2_MUX_MIPI           (3 << GPR10_LOCK_DCIC2_MUX_SHIFT) /* DCIC-2 source is MIPI DPI */
#define GPR10_LOCK_OCRAM_TZ_EN                (1 << 20)
#define GPR10_LOCK_OCRAM_TZ_ADDR_SHIFT        (21)
#define GPR10_LOCK_OCRAM_TZ_ADDR_MASK         (0x3f << GPR10_LOCK_OCRAM_TZ_ADDR_SHIFT)
#  define GPR10_LOCK_OCRAM_TZ_ADDR(n)         ((uint32_t)(n) << GPR10_LOCK_OCRAM_TZ_ADDR_SHIFT)
#define GPR10_LOCK_SEC_ERR_RESP               (1 << 27)
#define GPR10_LOCK_DBG_CLK_EN                 (1 << 28)
#define GPR10_LOCK_DBG_EN                     (1 << 29)

/* General Purpose Register 11 (GPR11) -- Contains no fields of interest. */
/* General Purpose Register 12 (GPR12) */

#define GPR12_USDHC_DBG_MUX_SHIFT             (2)
#define GPR12_USDHC_DBG_MUX_MASK              (3 << GPR12_USDHC_DBG_MUX_SHIFT)
#  define GPR12_USDHC_DBG_MUX_USDHC1          (0 << GPR12_USDHC_DBG_MUX_SHIFT) /* uSDHC1 debug */
#  define GPR12_USDHC_DBG_MUX_USDHC2          (1 << GPR12_USDHC_DBG_MUX_SHIFT) /* uSDHC2 debug */
#  define GPR12_USDHC_DBG_MUX_USDHC3          (2 << GPR12_USDHC_DBG_MUX_SHIFT) /* uSDHC3 debug */
#  define GPR12_USDHC_DBG_MUX_USDHC4          (3 << GPR12_USDHC_DBG_MUX_SHIFT) /* uSDHC4 debug */
#define GPR12_LOS_LEVEL_SHIFT                 (4)
#define GPR12_LOS_LEVEL_MASK                  (0x1f << GPR12_LOS_LEVEL_SHIFT)
#  define GPR12_LOS_LEVEL(n)                  ((uint32_t)(n) << GPR12_LOS_LEVEL_SHIFT)
#define GPR12_APPS_PM_XMT_PME                 (1 << 9)
#define GPR12_APP_LTSSM_ENABLE                (1 << 10)
#define GPR12_APP_INIT_RST                    (1 << 11)
#define GPR12_DEVICE_TYPE_SHIFT               (12)
#define GPR12_DEVICE_TYPE_MASK                (15 << GPR12_DEVICE_SHIFT)
#  define GPR12_DEVICE_TYPE_PCIE_EP           (0 << GPR12_DEVICE_SHIFT) /* EP mode */
#  define GPR12_DEVICE_TYPE_PCIE_RC           (2 << GPR12_DEVICE_SHIFT) /* RC mode */
#define GPR12_APPS_PM_XMT_TURNOFF             (1 << 16)
#define GPR12_DIA_STATUS_BUS_SELECT_SHIFT     (17)
#define GPR12_DIA_STATUS_BUS_SELECT_MASK      (15 << GPR12_DIA_STATUS_BUS_SELECT_SHIFT)
#  define GPR12_DIA_STATUS_BUS_SELECT(n)      ((uint32_t)(n) << GPR12_DIA_STATUS_BUS_SELECT_SHIFT)
#define GPR12_PCIE_CTL_7_SHIFT                (21)
#define GPR12_PCIE_CTL_7_MASK                 (7 << GPR12_PCIE_CTL_7_SHIFT)
  #define GPR12_PCIE_CTL_7(n)                 ((uint32_t)(n) << GPR12_PCIE_CTL_7_SHIFT)
#define GPR12_ARMP_APB_CLK_EN                 (1 << 24)
#define GPR12_ARMP_ATB_CLK_EN                 (1 << 25)
#define GPR12_ARMP_AHB_CLK_EN                 (1 << 26)
#define GPR12_ARMP_IPG_CLK_EN                 (1 << 27)

/* General Purpose Register 13 (GPR13) */

#define GPR13_SATA_PHY_0                      (1 << 0)
#define GPR13_SATA_PHY_1                      (1 << 1)
#define GPR13_SATA_PHY_2_SHIFT                (2)
#define GPR13_SATA_PHY_2_MASK                 (0x1f << GPR13_SATA_PHY_2_SHIFT)
#  define GPR13_SATA_PHY_2_0p937_V            (0x00 << GPR13_SATA_PHY_2_SHIFT)
#  define GPR13_SATA_PHY_2_0p947_V            (0x01 << GPR13_SATA_PHY_2_SHIFT)
#  define GPR13_SATA_PHY_2_0p957_V            (0x02 << GPR13_SATA_PHY_2_SHIFT)
#  define GPR13_SATA_PHY_2_0p966_V            (0x03 << GPR13_SATA_PHY_2_SHIFT)
#  define GPR13_SATA_PHY_2_0p976_V            (0x04 << GPR13_SATA_PHY_2_SHIFT)
#  define GPR13_SATA_PHY_2_0p986_V            (0x05 << GPR13_SATA_PHY_2_SHIFT)
#  define GPR13_SATA_PHY_2_0p996_V            (0x06 << GPR13_SATA_PHY_2_SHIFT)
#  define GPR13_SATA_PHY_2_1p005_V            (0x07 << GPR13_SATA_PHY_2_SHIFT)
#  define GPR13_SATA_PHY_2_1p015_V            (0x08 << GPR13_SATA_PHY_2_SHIFT)
#  define GPR13_SATA_PHY_2_1p025_V            (0x09 << GPR13_SATA_PHY_2_SHIFT)
#  define GPR13_SATA_PHY_2_1p035_V            (0x0a << GPR13_SATA_PHY_2_SHIFT)
#  define GPR13_SATA_PHY_2_1p045_V            (0x0b << GPR13_SATA_PHY_2_SHIFT)
#  define GPR13_SATA_PHY_2_1p054_V            (0x0c << GPR13_SATA_PHY_2_SHIFT)
#  define GPR13_SATA_PHY_2_1p064_V            (0x0d << GPR13_SATA_PHY_2_SHIFT)
#  define GPR13_SATA_PHY_2_1p074_V            (0x0e << GPR13_SATA_PHY_2_SHIFT)
#  define GPR13_SATA_PHY_2_1p084_V            (0x0f << GPR13_SATA_PHY_2_SHIFT)
#  define GPR13_SATA_PHY_2_1p094_V            (0x10 << GPR13_SATA_PHY_2_SHIFT)
#  define GPR13_SATA_PHY_2_1p104_V            (0x11 << GPR13_SATA_PHY_2_SHIFT)
#  define GPR13_SATA_PHY_2_1p113_V            (0x12 << GPR13_SATA_PHY_2_SHIFT)
#  define GPR13_SATA_PHY_2_1p123_V            (0x13 << GPR13_SATA_PHY_2_SHIFT)
#  define GPR13_SATA_PHY_2_1p133_V            (0x14 << GPR13_SATA_PHY_2_SHIFT)
#  define GPR13_SATA_PHY_2_1p143_V            (0x15 << GPR13_SATA_PHY_2_SHIFT)
#  define GPR13_SATA_PHY_2_1p152_V            (0x16 << GPR13_SATA_PHY_2_SHIFT)
#  define GPR13_SATA_PHY_2_1p162_V            (0x17 << GPR13_SATA_PHY_2_SHIFT)
#  define GPR13_SATA_PHY_2_1p172_V            (0x18 << GPR13_SATA_PHY_2_SHIFT)
#  define GPR13_SATA_PHY_2_1p182_V            (0x19 << GPR13_SATA_PHY_2_SHIFT)
#  define GPR13_SATA_PHY_2_1p191_V            (0x1a << GPR13_SATA_PHY_2_SHIFT)
#  define GPR13_SATA_PHY_2_1p201_V            (0x1b << GPR13_SATA_PHY_2_SHIFT)
#  define GPR13_SATA_PHY_2_1p211_V            (0x1c << GPR13_SATA_PHY_2_SHIFT)
#  define GPR13_SATA_PHY_2_1p221_V            (0x1d << GPR13_SATA_PHY_2_SHIFT)
#  define GPR13_SATA_PHY_2_1p230_V            (0x1e << GPR13_SATA_PHY_2_SHIFT)
#  define GPR13_SATA_PHY_2_1p240_V            (0x1f << GPR13_SATA_PHY_2_SHIFT)
#define GPR13_SATA_PHY_3_SHIFT                (7)
#define GPR13_SATA_PHY_3_MASK                 (15 << GPR13_SATA_PHY_3_SHIFT)
#  define GPR13_SATA_PHY_3_0p00_DB            (0  << GPR13_SATA_PHY_3_SHIFT)
#  define GPR13_SATA_PHY_3_0p37_DB            (1  << GPR13_SATA_PHY_3_SHIFT)
#  define GPR13_SATA_PHY_3_0p74_DB            (2  << GPR13_SATA_PHY_3_SHIFT)
#  define GPR13_SATA_PHY_3_1p11_DB            (3  << GPR13_SATA_PHY_3_SHIFT)
#  define GPR13_SATA_PHY_3_1p48_DB            (4  << GPR13_SATA_PHY_3_SHIFT)
#  define GPR13_SATA_PHY_3_1p85_DB            (5  << GPR13_SATA_PHY_3_SHIFT)
#  define GPR13_SATA_PHY_3_2p22_DB            (6  << GPR13_SATA_PHY_3_SHIFT)
#  define GPR13_SATA_PHY_3_2p59_DB            (7  << GPR13_SATA_PHY_3_SHIFT)
#  define GPR13_SATA_PHY_3_2p96_DB            (8  << GPR13_SATA_PHY_3_SHIFT)
#  define GPR13_SATA_PHY_3_3p33_DB            (9  << GPR13_SATA_PHY_3_SHIFT)
#  define GPR13_SATA_PHY_3_3p70_DB            (10 << GPR13_SATA_PHY_3_SHIFT)
#  define GPR13_SATA_PHY_3_4p07_DB            (11 << GPR13_SATA_PHY_3_SHIFT)
#  define GPR13_SATA_PHY_3_4p44_DB            (12 << GPR13_SATA_PHY_3_SHIFT)
#  define GPR13_SATA_PHY_3_4p81_DB            (13 << GPR13_SATA_PHY_3_SHIFT)
#  define GPR13_SATA_PHY_3_5p28_DB            (14 << GPR13_SATA_PHY_3_SHIFT)
#  define GPR13_SATA_PHY_3_5p75_DB            (15 << GPR13_SATA_PHY_3_SHIFT)
#define GPR13SATA_PHY_4_SHIFT                 (11)
#define GPR13SATA_PHY_4_MASK                  (7 << GPR13SATA_PHY_4_SHIFT)
#  define GPR13SATA_PHY_4_16_16               (0 << GPR13SATA_PHY_4_SHIFT) /* 16/16 */
#  define GPR13SATA_PHY_4_14_16               (1 << GPR13SATA_PHY_4_SHIFT) /* 14/16 */
#  define GPR13SATA_PHY_4_12_16               (2 << GPR13SATA_PHY_4_SHIFT) /* 12/16 */
#  define GPR13SATA_PHY_4_10_16               (3 << GPR13SATA_PHY_4_SHIFT) /* 10/16 */
#  define GPR13SATA_PHY_4_9_16                (4 << GPR13SATA_PHY_4_SHIFT) /* 9/16 (default) */
#  define GPR13SATA_PHY_4_8_16                (5 << GPR13SATA_PHY_4_SHIFT) /* 8/16 */
#define GPR13_SATA_PHY_5                      (1 << 14)
#  define GPR13_SATA_PHY_5_SSEN               (1 << 14)
#define GPR13_SATA_SPEED                      (1 << 15)
#define GPR13_SSATA_PHY_6_SHIFT               (16)
#define GPR13_SSATA_PHY_6_MASK                (7 << GPR13_SSATA_PHY_6_SHIFT)
#  define GPR13_SSATA_PHY_6_1P_1F             (0 << GPR13_SSATA_PHY_6_SHIFT)
#  define GPR13_SSATA_PHY_6_2P_2F             (1 << GPR13_SSATA_PHY_6_SHIFT)
#  define GPR13_SSATA_PHY_6_1P_4F             (2 << GPR13_SSATA_PHY_6_SHIFT)
#  define GPR13_SSATA_PHY_6_2P_4F             (3 << GPR13_SSATA_PHY_6_SHIFT)
#define GPR13_SATA_PHY_7_SHIFT                (19)
#define GPR13_SATA_PHY_7_MASK                 (0x1f << GPR13_SATA_PHY_7_SHIFT)
#  define GPR13_SATA_PHY_7_SATA1I             (0x10 << GPR13_SATA_PHY_7_SHIFT)
#  define GPR13_SATA_PHY_7_SATA1M             (0x10 << GPR13_SATA_PHY_7_SHIFT)
#  define GPR13_SATA_PHY_7_SATA1X             (0x1a << GPR13_SATA_PHY_7_SHIFT)
#  define GPR13_SATA_PHY_7_SATA2I             (0x12 << GPR13_SATA_PHY_7_SHIFT)
#  define GPR13_SATA_PHY_7_SATA2M             (0x12 << GPR13_SATA_PHY_7_SHIFT)
#  define GPR13_SATA_PHY_7_SATA2X             (0x1a << GPR13_SATA_PHY_7_SHIFT)
#define GPR13_SATA_PHY_8_SHIFT                (24)
#define GPR13_SATA_PHY_8_MASK                 (7 << GPR13_SATA_PHY_8_SHIFT)
#  define GPR13_SATA_PHY_8_0p5_DB             (0 << GPR13_SATA_PHY_8_SHIFT)
#  define GPR13_SATA_PHY_8_1p0_DB             (1 << GPR13_SATA_PHY_8_SHIFT)
#  define GPR13_SATA_PHY_8_1p5_DB             (2 << GPR13_SATA_PHY_8_SHIFT)
#  define GPR13_SATA_PHY_8_2p0_DB             (3 << GPR13_SATA_PHY_8_SHIFT)
#  define GPR13_SATA_PHY_8_2p5_DB             (4 << GPR13_SATA_PHY_8_SHIFT)
#  define GPR13_SATA_PHY_8_3p0_DB             (5 << GPR13_SATA_PHY_8_SHIFT)
#  define GPR13_SATA_PHY_8_3p5_DB             (6 << GPR13_SATA_PHY_8_SHIFT)
#  define GPR13_SATA_PHY_8_4p0_DB             (7 << GPR13_SATA_PHY_8_SHIFT)
#define GPR13_ENET_STOP_REQ                   (1 << 27)
#define GPR13_CAN1_STOP_REQ                   (1 << 28)
#define GPR13_CAN2_STOP_REQ                   (1 << 29)
#define GPR13_SDMA_STOP_REQ                   (1 << 30)

/* Pad Mux Registers */

#define PADMUX_MUXMODE_SHIFT                  (0)       /* Bit 0-2: Software Input On Field */
#define PADMUX_MUXMODE_MASK                   (7 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT0                 (0 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT1                 (1 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT2                 (2 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT3                 (3 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT4                 (4 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT5                 (5 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT6                 (6 << PADMUX_MUXMODE_SHIFT)
#  define PADMUX_MUXMODE_ALT7                 (7 << PADMUX_MUXMODE_SHIFT)
#define PADMUX_SION                           (1 << 4)  /* Bit 4: Software Input On Field */

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
#define SPEED_MEDIUM                          (2)       /* Medium frequency (100, 150 MHz) */
#define SPEED_MAX                             (3)       /* Maximum frequency (100, 150, 200 MHz) */

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

/* Pad Group Control Registers */
/* Select Input Registers */

#endif /* CONFIG_ARCH_CHIP_IMX6_6QUAD || CONFIG_ARCH_CHIP_IMX6_6DUAL */
#endif /* __ARCH_ARM_SRC_IMX6_HARDWARE_IMX_IOMUXC_H */
