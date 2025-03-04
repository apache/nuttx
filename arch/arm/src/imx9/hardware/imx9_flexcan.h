/****************************************************************************
 * arch/arm/src/imx9/hardware/imx9_flexcan.h
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

#ifndef __ARCH_ARM_SRC_IMX9_HARDWARE_IMX9_FLEXCAN_H
#define __ARCH_ARM_SRC_IMX9_HARDWARE_IMX9_FLEXCAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CAN Register Offsets *****************************************************/
#define IMX9_CAN_MCR_OFFSET              (0x0000)
#define IMX9_CAN_CTRL1_OFFSET            (0x0004)
#define IMX9_CAN_TIMER_OFFSET            (0x0008)
#define IMX9_CAN_RXMGMASK_OFFSET         (0x0010)
#define IMX9_CAN_RX14MASK_OFFSET         (0x0014)
#define IMX9_CAN_RX15MASK_OFFSET         (0x0018)
#define IMX9_CAN_ECR_OFFSET              (0x001c)
#define IMX9_CAN_ESR1_OFFSET             (0x0020)
#define IMX9_CAN_IMASK2_OFFSET           (0x0024)
#define IMX9_CAN_IMASK1_OFFSET           (0x0028)
#define IMX9_CAN_IFLAG2_OFFSET           (0x002c)
#define IMX9_CAN_IFLAG1_OFFSET           (0x0030)
#define IMX9_CAN_CTRL2_OFFSET            (0x0034)
#define IMX9_CAN_ESR2_OFFSET             (0x0038)
#define IMX9_CAN_CRCR_OFFSET             (0x0044)
#define IMX9_CAN_RXFGMASK_OFFSET         (0x0048)
#define IMX9_CAN_RXFIR_OFFSET            (0x004c)
#define IMX9_CAN_CBT_OFFSET              (0x0050)
#define IMX9_CAN_IMASK3_OFFSET           (0x006c)
#define IMX9_CAN_IFLAG3_OFFSET           (0x0074)
#define IMX9_CAN_CS0_OFFSET              (0x0080)
#define IMX9_CAN_WORD00_OFFSET           (0x0088)
#define IMX9_CAN_WORD10_OFFSET           (0x008c)
#define IMX9_CAN_CS1_OFFSET              (0x0090)
#define IMX9_CAN_WORD01_OFFSET           (0x0098)
#define IMX9_CAN_WORD11_OFFSET           (0x009c)
#define IMX9_CAN_CS2_OFFSET              (0x00a0)
#define IMX9_CAN_WORD02_OFFSET           (0x00a8)
#define IMX9_CAN_WORD12_OFFSET           (0x00ac)
#define IMX9_CAN_CS3_OFFSET              (0x00b0)
#define IMX9_CAN_WORD03_OFFSET           (0x00b8)
#define IMX9_CAN_WORD13_OFFSET           (0x00bc)
#define IMX9_CAN_CS4_OFFSET              (0x00c0)
#define IMX9_CAN_WORD04_OFFSET           (0x00c8)
#define IMX9_CAN_WORD14_OFFSET           (0x00cc)
#define IMX9_CAN_CS5_OFFSET              (0x00d0)
#define IMX9_CAN_WORD05_OFFSET           (0x00d8)
#define IMX9_CAN_WORD15_OFFSET           (0x00dc)
#define IMX9_CAN_CS6_OFFSET              (0x00e0)
#define IMX9_CAN_WORD06_OFFSET           (0x00e8)
#define IMX9_CAN_WORD16_OFFSET           (0x00ec)
#define IMX9_CAN_CS7_OFFSET              (0x00f0)
#define IMX9_CAN_WORD07_OFFSET           (0x00f8)
#define IMX9_CAN_WORD17_OFFSET           (0x00fc)
#define IMX9_CAN_CS8_OFFSET              (0x0100)
#define IMX9_CAN_WORD08_OFFSET           (0x0108)
#define IMX9_CAN_WORD18_OFFSET           (0x010c)
#define IMX9_CAN_CS9_OFFSET              (0x0110)
#define IMX9_CAN_WORD09_OFFSET           (0x0118)
#define IMX9_CAN_WORD19_OFFSET           (0x011c)
#define IMX9_CAN_CS10_OFFSET             (0x0120)
#define IMX9_CAN_WORD010_OFFSET          (0x0128)
#define IMX9_CAN_WORD110_OFFSET          (0x012c)
#define IMX9_CAN_CS11_OFFSET             (0x0130)
#define IMX9_CAN_WORD011_OFFSET          (0x0138)
#define IMX9_CAN_WORD111_OFFSET          (0x013c)
#define IMX9_CAN_CS12_OFFSET             (0x0140)
#define IMX9_CAN_WORD012_OFFSET          (0x0148)
#define IMX9_CAN_WORD112_OFFSET          (0x014c)
#define IMX9_CAN_CS13_OFFSET             (0x0150)
#define IMX9_CAN_WORD013_OFFSET          (0x0158)
#define IMX9_CAN_WORD113_OFFSET          (0x015c)
#define IMX9_CAN_CS14_OFFSET             (0x0160)
#define IMX9_CAN_WORD014_OFFSET          (0x0168)
#define IMX9_CAN_WORD114_OFFSET          (0x016c)
#define IMX9_CAN_CS15_OFFSET             (0x0170)
#define IMX9_CAN_WORD015_OFFSET          (0x0178)
#define IMX9_CAN_WORD115_OFFSET          (0x017c)
#define IMX9_CAN_CS16_OFFSET             (0x0180)
#define IMX9_CAN_WORD016_OFFSET          (0x0188)
#define IMX9_CAN_WORD116_OFFSET          (0x018c)
#define IMX9_CAN_CS17_OFFSET             (0x0190)
#define IMX9_CAN_WORD017_OFFSET          (0x0198)
#define IMX9_CAN_WORD117_OFFSET          (0x019c)
#define IMX9_CAN_CS18_OFFSET             (0x01a0)
#define IMX9_CAN_WORD018_OFFSET          (0x01a8)
#define IMX9_CAN_WORD118_OFFSET          (0x01ac)
#define IMX9_CAN_CS19_OFFSET             (0x01b0)
#define IMX9_CAN_WORD019_OFFSET          (0x01b8)
#define IMX9_CAN_WORD119_OFFSET          (0x01bc)
#define IMX9_CAN_CS20_OFFSET             (0x01c0)
#define IMX9_CAN_WORD020_OFFSET          (0x01c8)
#define IMX9_CAN_WORD120_OFFSET          (0x01cc)
#define IMX9_CAN_CS21_OFFSET             (0x01d0)
#define IMX9_CAN_WORD021_OFFSET          (0x01d8)
#define IMX9_CAN_WORD121_OFFSET          (0x01dc)
#define IMX9_CAN_CS22_OFFSET             (0x01e0)
#define IMX9_CAN_WORD022_OFFSET          (0x01e8)
#define IMX9_CAN_WORD122_OFFSET          (0x01ec)
#define IMX9_CAN_CS23_OFFSET             (0x01f0)
#define IMX9_CAN_WORD023_OFFSET          (0x01f8)
#define IMX9_CAN_WORD123_OFFSET          (0x01fc)
#define IMX9_CAN_CS24_OFFSET             (0x0200)
#define IMX9_CAN_WORD024_OFFSET          (0x0208)
#define IMX9_CAN_WORD124_OFFSET          (0x020c)
#define IMX9_CAN_CS25_OFFSET             (0x0210)
#define IMX9_CAN_WORD025_OFFSET          (0x0218)
#define IMX9_CAN_WORD125_OFFSET          (0x021c)
#define IMX9_CAN_CS26_OFFSET             (0x0220)
#define IMX9_CAN_WORD026_OFFSET          (0x0228)
#define IMX9_CAN_WORD126_OFFSET          (0x022c)
#define IMX9_CAN_CS27_OFFSET             (0x0230)
#define IMX9_CAN_WORD027_OFFSET          (0x0238)
#define IMX9_CAN_WORD127_OFFSET          (0x023c)
#define IMX9_CAN_CS28_OFFSET             (0x0240)
#define IMX9_CAN_WORD028_OFFSET          (0x0248)
#define IMX9_CAN_WORD128_OFFSET          (0x024c)
#define IMX9_CAN_CS29_OFFSET             (0x0250)
#define IMX9_CAN_WORD029_OFFSET          (0x0258)
#define IMX9_CAN_WORD129_OFFSET          (0x025c)
#define IMX9_CAN_CS30_OFFSET             (0x0260)
#define IMX9_CAN_WORD030_OFFSET          (0x0268)
#define IMX9_CAN_WORD130_OFFSET          (0x026c)
#define IMX9_CAN_CS31_OFFSET             (0x0270)
#define IMX9_CAN_WORD031_OFFSET          (0x0278)
#define IMX9_CAN_WORD131_OFFSET          (0x027c)
#define IMX9_CAN_CS32_OFFSET             (0x0280)
#define IMX9_CAN_WORD032_OFFSET          (0x0288)
#define IMX9_CAN_WORD132_OFFSET          (0x028c)
#define IMX9_CAN_CS33_OFFSET             (0x0290)
#define IMX9_CAN_WORD033_OFFSET          (0x0298)
#define IMX9_CAN_WORD133_OFFSET          (0x029c)
#define IMX9_CAN_CS34_OFFSET             (0x02a0)
#define IMX9_CAN_WORD034_OFFSET          (0x02a8)
#define IMX9_CAN_WORD134_OFFSET          (0x02ac)
#define IMX9_CAN_CS35_OFFSET             (0x02b0)
#define IMX9_CAN_WORD035_OFFSET          (0x02b8)
#define IMX9_CAN_WORD135_OFFSET          (0x02bc)
#define IMX9_CAN_CS36_OFFSET             (0x02c0)
#define IMX9_CAN_WORD036_OFFSET          (0x02c8)
#define IMX9_CAN_WORD136_OFFSET          (0x02cc)
#define IMX9_CAN_CS37_OFFSET             (0x02d0)
#define IMX9_CAN_WORD037_OFFSET          (0x02d8)
#define IMX9_CAN_WORD137_OFFSET          (0x02dc)
#define IMX9_CAN_CS38_OFFSET             (0x02e0)
#define IMX9_CAN_WORD038_OFFSET          (0x02e8)
#define IMX9_CAN_WORD138_OFFSET          (0x02ec)
#define IMX9_CAN_CS39_OFFSET             (0x02f0)
#define IMX9_CAN_WORD039_OFFSET          (0x02f8)
#define IMX9_CAN_WORD139_OFFSET          (0x02fc)
#define IMX9_CAN_CS40_OFFSET             (0x0300)
#define IMX9_CAN_WORD040_OFFSET          (0x0308)
#define IMX9_CAN_WORD140_OFFSET          (0x030c)
#define IMX9_CAN_CS41_OFFSET             (0x0310)
#define IMX9_CAN_WORD041_OFFSET          (0x0318)
#define IMX9_CAN_WORD141_OFFSET          (0x031c)
#define IMX9_CAN_CS42_OFFSET             (0x0320)
#define IMX9_CAN_WORD042_OFFSET          (0x0328)
#define IMX9_CAN_WORD142_OFFSET          (0x032c)
#define IMX9_CAN_CS43_OFFSET             (0x0330)
#define IMX9_CAN_WORD043_OFFSET          (0x0338)
#define IMX9_CAN_WORD143_OFFSET          (0x033c)
#define IMX9_CAN_CS44_OFFSET             (0x0340)
#define IMX9_CAN_WORD044_OFFSET          (0x0348)
#define IMX9_CAN_WORD144_OFFSET          (0x034c)
#define IMX9_CAN_CS45_OFFSET             (0x0350)
#define IMX9_CAN_WORD045_OFFSET          (0x0358)
#define IMX9_CAN_WORD145_OFFSET          (0x035c)
#define IMX9_CAN_CS46_OFFSET             (0x0360)
#define IMX9_CAN_WORD046_OFFSET          (0x0368)
#define IMX9_CAN_WORD146_OFFSET          (0x036c)
#define IMX9_CAN_CS47_OFFSET             (0x0370)
#define IMX9_CAN_WORD047_OFFSET          (0x0378)
#define IMX9_CAN_WORD147_OFFSET          (0x037c)
#define IMX9_CAN_CS48_OFFSET             (0x0380)
#define IMX9_CAN_WORD048_OFFSET          (0x0388)
#define IMX9_CAN_WORD148_OFFSET          (0x038c)
#define IMX9_CAN_CS49_OFFSET             (0x0390)
#define IMX9_CAN_WORD049_OFFSET          (0x0398)
#define IMX9_CAN_WORD149_OFFSET          (0x039c)
#define IMX9_CAN_CS50_OFFSET             (0x03a0)
#define IMX9_CAN_WORD050_OFFSET          (0x03a8)
#define IMX9_CAN_WORD150_OFFSET          (0x03ac)
#define IMX9_CAN_CS51_OFFSET             (0x03b0)
#define IMX9_CAN_WORD051_OFFSET          (0x03b8)
#define IMX9_CAN_WORD151_OFFSET          (0x03bc)
#define IMX9_CAN_CS52_OFFSET             (0x03c0)
#define IMX9_CAN_WORD052_OFFSET          (0x03c8)
#define IMX9_CAN_WORD152_OFFSET          (0x03cc)
#define IMX9_CAN_CS53_OFFSET             (0x03d0)
#define IMX9_CAN_WORD053_OFFSET          (0x03d8)
#define IMX9_CAN_WORD153_OFFSET          (0x03dc)
#define IMX9_CAN_CS54_OFFSET             (0x03e0)
#define IMX9_CAN_WORD054_OFFSET          (0x03e8)
#define IMX9_CAN_WORD154_OFFSET          (0x03ec)
#define IMX9_CAN_CS55_OFFSET             (0x03f0)
#define IMX9_CAN_WORD055_OFFSET          (0x03f8)
#define IMX9_CAN_WORD155_OFFSET          (0x03fc)
#define IMX9_CAN_CS56_OFFSET             (0x0400)
#define IMX9_CAN_WORD056_OFFSET          (0x0408)
#define IMX9_CAN_WORD156_OFFSET          (0x040c)
#define IMX9_CAN_CS57_OFFSET             (0x0410)
#define IMX9_CAN_WORD057_OFFSET          (0x0418)
#define IMX9_CAN_WORD157_OFFSET          (0x041c)
#define IMX9_CAN_CS58_OFFSET             (0x0420)
#define IMX9_CAN_WORD058_OFFSET          (0x0428)
#define IMX9_CAN_WORD158_OFFSET          (0x042c)
#define IMX9_CAN_CS59_OFFSET             (0x0430)
#define IMX9_CAN_WORD059_OFFSET          (0x0438)
#define IMX9_CAN_WORD159_OFFSET          (0x043c)
#define IMX9_CAN_CS60_OFFSET             (0x0440)
#define IMX9_CAN_WORD060_OFFSET          (0x0448)
#define IMX9_CAN_WORD160_OFFSET          (0x044c)
#define IMX9_CAN_CS61_OFFSET             (0x0450)
#define IMX9_CAN_WORD061_OFFSET          (0x0458)
#define IMX9_CAN_WORD161_OFFSET          (0x045c)
#define IMX9_CAN_CS62_OFFSET             (0x0460)
#define IMX9_CAN_WORD062_OFFSET          (0x0468)
#define IMX9_CAN_WORD162_OFFSET          (0x046c)
#define IMX9_CAN_CS63_OFFSET             (0x0470)
#define IMX9_CAN_WORD063_OFFSET          (0x0478)
#define IMX9_CAN_WORD163_OFFSET          (0x047c)
#define IMX9_CAN_CS64_OFFSET             (0x0480)
#define IMX9_CAN_WORD064_OFFSET          (0x0488)
#define IMX9_CAN_WORD164_OFFSET          (0x048c)
#define IMX9_CAN_CS65_OFFSET             (0x0490)
#define IMX9_CAN_WORD065_OFFSET          (0x0498)
#define IMX9_CAN_WORD165_OFFSET          (0x049c)
#define IMX9_CAN_CS66_OFFSET             (0x04a0)
#define IMX9_CAN_WORD066_OFFSET          (0x04a8)
#define IMX9_CAN_WORD166_OFFSET          (0x04ac)
#define IMX9_CAN_CS67_OFFSET             (0x04b0)
#define IMX9_CAN_WORD067_OFFSET          (0x04b8)
#define IMX9_CAN_WORD167_OFFSET          (0x04bc)
#define IMX9_CAN_CS68_OFFSET             (0x04c0)
#define IMX9_CAN_WORD068_OFFSET          (0x04c8)
#define IMX9_CAN_WORD168_OFFSET          (0x04cc)
#define IMX9_CAN_CS69_OFFSET             (0x04d0)
#define IMX9_CAN_WORD069_OFFSET          (0x04d8)
#define IMX9_CAN_WORD169_OFFSET          (0x04dc)
#define IMX9_CAN_CS70_OFFSET             (0x04e0)
#define IMX9_CAN_WORD070_OFFSET          (0x04e8)
#define IMX9_CAN_WORD170_OFFSET          (0x04ec)
#define IMX9_CAN_CS71_OFFSET             (0x04f0)
#define IMX9_CAN_WORD071_OFFSET          (0x04f8)
#define IMX9_CAN_WORD171_OFFSET          (0x04fc)
#define IMX9_CAN_CS72_OFFSET             (0x0500)
#define IMX9_CAN_WORD072_OFFSET          (0x0508)
#define IMX9_CAN_WORD172_OFFSET          (0x050c)
#define IMX9_CAN_CS73_OFFSET             (0x0510)
#define IMX9_CAN_WORD073_OFFSET          (0x0518)
#define IMX9_CAN_WORD173_OFFSET          (0x051c)
#define IMX9_CAN_CS74_OFFSET             (0x0520)
#define IMX9_CAN_WORD074_OFFSET          (0x0528)
#define IMX9_CAN_WORD174_OFFSET          (0x052c)
#define IMX9_CAN_CS75_OFFSET             (0x0530)
#define IMX9_CAN_WORD075_OFFSET          (0x0538)
#define IMX9_CAN_WORD175_OFFSET          (0x053c)
#define IMX9_CAN_CS76_OFFSET             (0x0540)
#define IMX9_CAN_WORD076_OFFSET          (0x0548)
#define IMX9_CAN_WORD176_OFFSET          (0x054c)
#define IMX9_CAN_CS77_OFFSET             (0x0550)
#define IMX9_CAN_WORD077_OFFSET          (0x0558)
#define IMX9_CAN_WORD177_OFFSET          (0x055c)
#define IMX9_CAN_CS78_OFFSET             (0x0560)
#define IMX9_CAN_WORD078_OFFSET          (0x0568)
#define IMX9_CAN_WORD178_OFFSET          (0x056c)
#define IMX9_CAN_CS79_OFFSET             (0x0570)
#define IMX9_CAN_WORD079_OFFSET          (0x0578)
#define IMX9_CAN_WORD179_OFFSET          (0x057c)
#define IMX9_CAN_CS80_OFFSET             (0x0580)
#define IMX9_CAN_WORD080_OFFSET          (0x0588)
#define IMX9_CAN_WORD180_OFFSET          (0x058c)
#define IMX9_CAN_CS81_OFFSET             (0x0590)
#define IMX9_CAN_WORD081_OFFSET          (0x0598)
#define IMX9_CAN_WORD181_OFFSET          (0x059c)
#define IMX9_CAN_CS82_OFFSET             (0x05a0)
#define IMX9_CAN_WORD082_OFFSET          (0x05a8)
#define IMX9_CAN_WORD182_OFFSET          (0x05ac)
#define IMX9_CAN_CS83_OFFSET             (0x05b0)
#define IMX9_CAN_WORD083_OFFSET          (0x05b8)
#define IMX9_CAN_WORD183_OFFSET          (0x05bc)
#define IMX9_CAN_CS84_OFFSET             (0x05c0)
#define IMX9_CAN_WORD084_OFFSET          (0x05c8)
#define IMX9_CAN_WORD184_OFFSET          (0x05cc)
#define IMX9_CAN_CS85_OFFSET             (0x05d0)
#define IMX9_CAN_WORD085_OFFSET          (0x05d8)
#define IMX9_CAN_WORD185_OFFSET          (0x05dc)
#define IMX9_CAN_CS86_OFFSET             (0x05e0)
#define IMX9_CAN_WORD086_OFFSET          (0x05e8)
#define IMX9_CAN_WORD186_OFFSET          (0x05ec)
#define IMX9_CAN_CS87_OFFSET             (0x05f0)
#define IMX9_CAN_WORD087_OFFSET          (0x05f8)
#define IMX9_CAN_WORD187_OFFSET          (0x05fc)
#define IMX9_CAN_CS88_OFFSET             (0x0600)
#define IMX9_CAN_WORD088_OFFSET          (0x0608)
#define IMX9_CAN_WORD188_OFFSET          (0x060c)
#define IMX9_CAN_CS89_OFFSET             (0x0610)
#define IMX9_CAN_WORD089_OFFSET          (0x0618)
#define IMX9_CAN_WORD189_OFFSET          (0x061c)
#define IMX9_CAN_CS90_OFFSET             (0x0620)
#define IMX9_CAN_WORD090_OFFSET          (0x0628)
#define IMX9_CAN_WORD190_OFFSET          (0x062c)
#define IMX9_CAN_CS91_OFFSET             (0x0630)
#define IMX9_CAN_WORD091_OFFSET          (0x0638)
#define IMX9_CAN_WORD191_OFFSET          (0x063c)
#define IMX9_CAN_CS92_OFFSET             (0x0640)
#define IMX9_CAN_WORD092_OFFSET          (0x0648)
#define IMX9_CAN_WORD192_OFFSET          (0x064c)
#define IMX9_CAN_CS93_OFFSET             (0x0650)
#define IMX9_CAN_WORD093_OFFSET          (0x0658)
#define IMX9_CAN_WORD193_OFFSET          (0x065c)
#define IMX9_CAN_CS94_OFFSET             (0x0660)
#define IMX9_CAN_WORD094_OFFSET          (0x0668)
#define IMX9_CAN_WORD194_OFFSET          (0x066c)
#define IMX9_CAN_CS95_OFFSET             (0x0670)
#define IMX9_CAN_WORD095_OFFSET          (0x0678)
#define IMX9_CAN_WORD195_OFFSET          (0x067c)
#define IMX9_CAN_MECR_OFFSET             (0x0ae0)
#define IMX9_CAN_ERRIAR_OFFSET           (0x0ae4)
#define IMX9_CAN_ERRIDPR_OFFSET          (0x0ae8)
#define IMX9_CAN_ERRIPPR_OFFSET          (0x0aec)
#define IMX9_CAN_RERRAR_OFFSET           (0x0af0)
#define IMX9_CAN_RERRDR_OFFSET           (0x0af4)
#define IMX9_CAN_RERRSYNR_OFFSET         (0x0af8)
#define IMX9_CAN_ERRSR_OFFSET            (0x0afc)
#define IMX9_CAN_EPRS_OFFSET             (0x0bf0)
#define IMX9_CAN_ENCBT_OFFSET            (0x0bf4)
#define IMX9_CAN_EDCBT_OFFSET            (0x0bf8)
#define IMX9_CAN_ETDC_OFFSET             (0x0bfc)
#define IMX9_CAN_FDCTRL_OFFSET           (0x0c00)
#define IMX9_CAN_FDCBT_OFFSET            (0x0c04)
#define IMX9_CAN_FDCRC_OFFSET            (0x0c08)
#define IMX9_CAN_ERFCR_OFFSET            (0x0c0c)
#define IMX9_CAN_ERFIER_OFFSET           (0x0c10)
#define IMX9_CAN_ERFSR_OFFSET            (0x0c14)
#define IMX9_CAN_RXIMR_OFFSET(n)         (0x0880+((n)<<2)) /* Rn Individual Mask Registers */
#define IMX9_CAN_RXIMR0_OFFSET           (0x0880)
#define IMX9_CAN_RXIMR1_OFFSET           (0x0884)
#define IMX9_CAN_RXIMR2_OFFSET           (0x0888)
#define IMX9_CAN_RXIMR3_OFFSET           (0x088c)
#define IMX9_CAN_RXIMR4_OFFSET           (0x0890)
#define IMX9_CAN_RXIMR5_OFFSET           (0x0894)
#define IMX9_CAN_RXIMR6_OFFSET           (0x0898)
#define IMX9_CAN_RXIMR7_OFFSET           (0x089c)
#define IMX9_CAN_RXIMR8_OFFSET           (0x08a0)
#define IMX9_CAN_RXIMR9_OFFSET           (0x08a4)
#define IMX9_CAN_RXIMR10_OFFSET          (0x08a8)
#define IMX9_CAN_RXIMR11_OFFSET          (0x08ac)
#define IMX9_CAN_RXIMR12_OFFSET          (0x08b0)
#define IMX9_CAN_RXIMR13_OFFSET          (0x08b4)
#define IMX9_CAN_RXIMR14_OFFSET          (0x08b8)
#define IMX9_CAN_RXIMR15_OFFSET          (0x08bc)
#define IMX9_CAN_RXIMR16_OFFSET          (0x08c0)
#define IMX9_CAN_RXIMR17_OFFSET          (0x08c4)
#define IMX9_CAN_RXIMR18_OFFSET          (0x08c8)
#define IMX9_CAN_RXIMR19_OFFSET          (0x08cc)
#define IMX9_CAN_RXIMR20_OFFSET          (0x08d0)
#define IMX9_CAN_RXIMR21_OFFSET          (0x08d4)
#define IMX9_CAN_RXIMR22_OFFSET          (0x08d8)
#define IMX9_CAN_RXIMR23_OFFSET          (0x08dc)
#define IMX9_CAN_RXIMR24_OFFSET          (0x08e0)
#define IMX9_CAN_RXIMR25_OFFSET          (0x08e4)
#define IMX9_CAN_RXIMR26_OFFSET          (0x08e8)
#define IMX9_CAN_RXIMR27_OFFSET          (0x08ec)
#define IMX9_CAN_RXIMR28_OFFSET          (0x08f0)
#define IMX9_CAN_RXIMR29_OFFSET          (0x08f4)
#define IMX9_CAN_RXIMR30_OFFSET          (0x08f8)
#define IMX9_CAN_RXIMR31_OFFSET          (0x08fc)
#define IMX9_CAN_RXIMR32_OFFSET          (0x0900)
#define IMX9_CAN_RXIMR33_OFFSET          (0x0904)
#define IMX9_CAN_RXIMR34_OFFSET          (0x0908)
#define IMX9_CAN_RXIMR35_OFFSET          (0x090c)
#define IMX9_CAN_RXIMR36_OFFSET          (0x0910)
#define IMX9_CAN_RXIMR37_OFFSET          (0x0914)
#define IMX9_CAN_RXIMR38_OFFSET          (0x0918)
#define IMX9_CAN_RXIMR39_OFFSET          (0x091c)
#define IMX9_CAN_RXIMR40_OFFSET          (0x0920)
#define IMX9_CAN_RXIMR41_OFFSET          (0x0924)
#define IMX9_CAN_RXIMR42_OFFSET          (0x0928)
#define IMX9_CAN_RXIMR43_OFFSET          (0x092c)
#define IMX9_CAN_RXIMR44_OFFSET          (0x0930)
#define IMX9_CAN_RXIMR45_OFFSET          (0x0934)
#define IMX9_CAN_RXIMR46_OFFSET          (0x0938)
#define IMX9_CAN_RXIMR47_OFFSET          (0x093c)
#define IMX9_CAN_RXIMR48_OFFSET          (0x0940)
#define IMX9_CAN_RXIMR49_OFFSET          (0x0944)
#define IMX9_CAN_RXIMR50_OFFSET          (0x0948)
#define IMX9_CAN_RXIMR51_OFFSET          (0x094c)
#define IMX9_CAN_RXIMR52_OFFSET          (0x0950)
#define IMX9_CAN_RXIMR53_OFFSET          (0x0954)
#define IMX9_CAN_RXIMR54_OFFSET          (0x0958)
#define IMX9_CAN_RXIMR55_OFFSET          (0x095c)
#define IMX9_CAN_RXIMR56_OFFSET          (0x0960)
#define IMX9_CAN_RXIMR57_OFFSET          (0x0964)
#define IMX9_CAN_RXIMR58_OFFSET          (0x0968)
#define IMX9_CAN_RXIMR59_OFFSET          (0x096c)
#define IMX9_CAN_RXIMR60_OFFSET          (0x0970)
#define IMX9_CAN_RXIMR61_OFFSET          (0x0974)
#define IMX9_CAN_RXIMR62_OFFSET          (0x0978)
#define IMX9_CAN_RXIMR63_OFFSET          (0x097c)
#define IMX9_CAN_RXIMR64_OFFSET          (0x0980)
#define IMX9_CAN_RXIMR65_OFFSET          (0x0984)
#define IMX9_CAN_RXIMR66_OFFSET          (0x0988)
#define IMX9_CAN_RXIMR67_OFFSET          (0x098c)
#define IMX9_CAN_RXIMR68_OFFSET          (0x0990)
#define IMX9_CAN_RXIMR69_OFFSET          (0x0994)
#define IMX9_CAN_RXIMR70_OFFSET          (0x0998)
#define IMX9_CAN_RXIMR71_OFFSET          (0x099c)
#define IMX9_CAN_RXIMR72_OFFSET          (0x09a0)
#define IMX9_CAN_RXIMR73_OFFSET          (0x09a4)
#define IMX9_CAN_RXIMR74_OFFSET          (0x09a8)
#define IMX9_CAN_RXIMR75_OFFSET          (0x09ac)
#define IMX9_CAN_RXIMR76_OFFSET          (0x09b0)
#define IMX9_CAN_RXIMR77_OFFSET          (0x09b4)
#define IMX9_CAN_RXIMR78_OFFSET          (0x09b8)
#define IMX9_CAN_RXIMR79_OFFSET          (0x09bc)
#define IMX9_CAN_RXIMR80_OFFSET          (0x09c0)
#define IMX9_CAN_RXIMR81_OFFSET          (0x09c4)
#define IMX9_CAN_RXIMR82_OFFSET          (0x09c8)
#define IMX9_CAN_RXIMR83_OFFSET          (0x09cc)
#define IMX9_CAN_RXIMR84_OFFSET          (0x09d0)
#define IMX9_CAN_RXIMR85_OFFSET          (0x09d4)
#define IMX9_CAN_RXIMR86_OFFSET          (0x09d8)
#define IMX9_CAN_RXIMR87_OFFSET          (0x09dc)
#define IMX9_CAN_RXIMR88_OFFSET          (0x09e0)
#define IMX9_CAN_RXIMR89_OFFSET          (0x09e4)
#define IMX9_CAN_RXIMR90_OFFSET          (0x09e8)
#define IMX9_CAN_RXIMR91_OFFSET          (0x09ec)
#define IMX9_CAN_RXIMR92_OFFSET          (0x09f0)
#define IMX9_CAN_RXIMR93_OFFSET          (0x09f4)
#define IMX9_CAN_RXIMR94_OFFSET          (0x09f8)
#define IMX9_CAN_RXIMR95_OFFSET          (0x09fc)
#define IMX9_CAN_HR_TIME_STAMP0_OFFSET   (0x0c30)
#define IMX9_CAN_HR_TIME_STAMP1_OFFSET   (0x0c34)
#define IMX9_CAN_HR_TIME_STAMP2_OFFSET   (0x0c38)
#define IMX9_CAN_HR_TIME_STAMP3_OFFSET   (0x0c3c)
#define IMX9_CAN_HR_TIME_STAMP4_OFFSET   (0x0c40)
#define IMX9_CAN_HR_TIME_STAMP5_OFFSET   (0x0c44)
#define IMX9_CAN_HR_TIME_STAMP6_OFFSET   (0x0c48)
#define IMX9_CAN_HR_TIME_STAMP7_OFFSET   (0x0c4c)
#define IMX9_CAN_HR_TIME_STAMP8_OFFSET   (0x0c50)
#define IMX9_CAN_HR_TIME_STAMP9_OFFSET   (0x0c54)
#define IMX9_CAN_HR_TIME_STAMP10_OFFSET  (0x0c58)
#define IMX9_CAN_HR_TIME_STAMP11_OFFSET  (0x0c5c)
#define IMX9_CAN_HR_TIME_STAMP12_OFFSET  (0x0c60)
#define IMX9_CAN_HR_TIME_STAMP13_OFFSET  (0x0c64)
#define IMX9_CAN_HR_TIME_STAMP14_OFFSET  (0x0c68)
#define IMX9_CAN_HR_TIME_STAMP15_OFFSET  (0x0c6c)
#define IMX9_CAN_HR_TIME_STAMP16_OFFSET  (0x0c70)
#define IMX9_CAN_HR_TIME_STAMP17_OFFSET  (0x0c74)
#define IMX9_CAN_HR_TIME_STAMP18_OFFSET  (0x0c78)
#define IMX9_CAN_HR_TIME_STAMP19_OFFSET  (0x0c7c)
#define IMX9_CAN_HR_TIME_STAMP20_OFFSET  (0x0c80)
#define IMX9_CAN_HR_TIME_STAMP21_OFFSET  (0x0c84)
#define IMX9_CAN_HR_TIME_STAMP22_OFFSET  (0x0c88)
#define IMX9_CAN_HR_TIME_STAMP23_OFFSET  (0x0c8c)
#define IMX9_CAN_HR_TIME_STAMP24_OFFSET  (0x0c90)
#define IMX9_CAN_HR_TIME_STAMP25_OFFSET  (0x0c94)
#define IMX9_CAN_HR_TIME_STAMP26_OFFSET  (0x0c98)
#define IMX9_CAN_HR_TIME_STAMP27_OFFSET  (0x0c9c)
#define IMX9_CAN_HR_TIME_STAMP28_OFFSET  (0x0ca0)
#define IMX9_CAN_HR_TIME_STAMP29_OFFSET  (0x0ca4)
#define IMX9_CAN_HR_TIME_STAMP30_OFFSET  (0x0ca8)
#define IMX9_CAN_HR_TIME_STAMP31_OFFSET  (0x0cac)
#define IMX9_CAN_HR_TIME_STAMP32_OFFSET  (0x0cb0)
#define IMX9_CAN_HR_TIME_STAMP33_OFFSET  (0x0cb4)
#define IMX9_CAN_HR_TIME_STAMP34_OFFSET  (0x0cb8)
#define IMX9_CAN_HR_TIME_STAMP35_OFFSET  (0x0cbc)
#define IMX9_CAN_HR_TIME_STAMP36_OFFSET  (0x0cc0)
#define IMX9_CAN_HR_TIME_STAMP37_OFFSET  (0x0cc4)
#define IMX9_CAN_HR_TIME_STAMP38_OFFSET  (0x0cc8)
#define IMX9_CAN_HR_TIME_STAMP39_OFFSET  (0x0ccc)
#define IMX9_CAN_HR_TIME_STAMP40_OFFSET  (0x0cd0)
#define IMX9_CAN_HR_TIME_STAMP41_OFFSET  (0x0cd4)
#define IMX9_CAN_HR_TIME_STAMP42_OFFSET  (0x0cd8)
#define IMX9_CAN_HR_TIME_STAMP43_OFFSET  (0x0cdc)
#define IMX9_CAN_HR_TIME_STAMP44_OFFSET  (0x0ce0)
#define IMX9_CAN_HR_TIME_STAMP45_OFFSET  (0x0ce4)
#define IMX9_CAN_HR_TIME_STAMP46_OFFSET  (0x0ce8)
#define IMX9_CAN_HR_TIME_STAMP47_OFFSET  (0x0cec)
#define IMX9_CAN_HR_TIME_STAMP48_OFFSET  (0x0cf0)
#define IMX9_CAN_HR_TIME_STAMP49_OFFSET  (0x0cf4)
#define IMX9_CAN_HR_TIME_STAMP50_OFFSET  (0x0cf8)
#define IMX9_CAN_HR_TIME_STAMP51_OFFSET  (0x0cfc)
#define IMX9_CAN_HR_TIME_STAMP52_OFFSET  (0x0d00)
#define IMX9_CAN_HR_TIME_STAMP53_OFFSET  (0x0d04)
#define IMX9_CAN_HR_TIME_STAMP54_OFFSET  (0x0d08)
#define IMX9_CAN_HR_TIME_STAMP55_OFFSET  (0x0d0c)
#define IMX9_CAN_HR_TIME_STAMP56_OFFSET  (0x0d10)
#define IMX9_CAN_HR_TIME_STAMP57_OFFSET  (0x0d14)
#define IMX9_CAN_HR_TIME_STAMP58_OFFSET  (0x0d18)
#define IMX9_CAN_HR_TIME_STAMP59_OFFSET  (0x0d1c)
#define IMX9_CAN_HR_TIME_STAMP60_OFFSET  (0x0d20)
#define IMX9_CAN_HR_TIME_STAMP61_OFFSET  (0x0d24)
#define IMX9_CAN_HR_TIME_STAMP62_OFFSET  (0x0d28)
#define IMX9_CAN_HR_TIME_STAMP63_OFFSET  (0x0d2c)
#define IMX9_CAN_HR_TIME_STAMP64_OFFSET  (0x0d30)
#define IMX9_CAN_HR_TIME_STAMP65_OFFSET  (0x0d34)
#define IMX9_CAN_HR_TIME_STAMP66_OFFSET  (0x0d38)
#define IMX9_CAN_HR_TIME_STAMP67_OFFSET  (0x0d3c)
#define IMX9_CAN_HR_TIME_STAMP68_OFFSET  (0x0d40)
#define IMX9_CAN_HR_TIME_STAMP69_OFFSET  (0x0d44)
#define IMX9_CAN_HR_TIME_STAMP70_OFFSET  (0x0d48)
#define IMX9_CAN_HR_TIME_STAMP71_OFFSET  (0x0d4c)
#define IMX9_CAN_HR_TIME_STAMP72_OFFSET  (0x0d50)
#define IMX9_CAN_HR_TIME_STAMP73_OFFSET  (0x0d54)
#define IMX9_CAN_HR_TIME_STAMP74_OFFSET  (0x0d58)
#define IMX9_CAN_HR_TIME_STAMP75_OFFSET  (0x0d5c)
#define IMX9_CAN_HR_TIME_STAMP76_OFFSET  (0x0d60)
#define IMX9_CAN_HR_TIME_STAMP77_OFFSET  (0x0d64)
#define IMX9_CAN_HR_TIME_STAMP78_OFFSET  (0x0d68)
#define IMX9_CAN_HR_TIME_STAMP79_OFFSET  (0x0d6c)
#define IMX9_CAN_HR_TIME_STAMP80_OFFSET  (0x0d70)
#define IMX9_CAN_HR_TIME_STAMP81_OFFSET  (0x0d74)
#define IMX9_CAN_HR_TIME_STAMP82_OFFSET  (0x0d78)
#define IMX9_CAN_HR_TIME_STAMP83_OFFSET  (0x0d7c)
#define IMX9_CAN_HR_TIME_STAMP84_OFFSET  (0x0d80)
#define IMX9_CAN_HR_TIME_STAMP85_OFFSET  (0x0d84)
#define IMX9_CAN_HR_TIME_STAMP86_OFFSET  (0x0d88)
#define IMX9_CAN_HR_TIME_STAMP87_OFFSET  (0x0d8c)
#define IMX9_CAN_HR_TIME_STAMP88_OFFSET  (0x0d90)
#define IMX9_CAN_HR_TIME_STAMP89_OFFSET  (0x0d94)
#define IMX9_CAN_HR_TIME_STAMP90_OFFSET  (0x0d98)
#define IMX9_CAN_HR_TIME_STAMP91_OFFSET  (0x0d9c)
#define IMX9_CAN_HR_TIME_STAMP92_OFFSET  (0x0da0)
#define IMX9_CAN_HR_TIME_STAMP93_OFFSET  (0x0da4)
#define IMX9_CAN_HR_TIME_STAMP94_OFFSET  (0x0da8)
#define IMX9_CAN_HR_TIME_STAMP95_OFFSET  (0x0dac)
#define IMX9_CAN_ERFFEL0_OFFSET          (0x3000)
#define IMX9_CAN_ERFFEL1_OFFSET          (0x3004)
#define IMX9_CAN_ERFFEL2_OFFSET          (0x3008)
#define IMX9_CAN_ERFFEL3_OFFSET          (0x300c)
#define IMX9_CAN_ERFFEL4_OFFSET          (0x3010)
#define IMX9_CAN_ERFFEL5_OFFSET          (0x3014)
#define IMX9_CAN_ERFFEL6_OFFSET          (0x3018)
#define IMX9_CAN_ERFFEL7_OFFSET          (0x301c)
#define IMX9_CAN_ERFFEL8_OFFSET          (0x3020)
#define IMX9_CAN_ERFFEL9_OFFSET          (0x3024)
#define IMX9_CAN_ERFFEL10_OFFSET         (0x3028)
#define IMX9_CAN_ERFFEL11_OFFSET         (0x302c)
#define IMX9_CAN_ERFFEL12_OFFSET         (0x3030)
#define IMX9_CAN_ERFFEL13_OFFSET         (0x3034)
#define IMX9_CAN_ERFFEL14_OFFSET         (0x3038)
#define IMX9_CAN_ERFFEL15_OFFSET         (0x303c)
#define IMX9_CAN_ERFFEL16_OFFSET         (0x3040)
#define IMX9_CAN_ERFFEL17_OFFSET         (0x3044)
#define IMX9_CAN_ERFFEL18_OFFSET         (0x3048)
#define IMX9_CAN_ERFFEL19_OFFSET         (0x304c)
#define IMX9_CAN_ERFFEL20_OFFSET         (0x3050)
#define IMX9_CAN_ERFFEL21_OFFSET         (0x3054)
#define IMX9_CAN_ERFFEL22_OFFSET         (0x3058)
#define IMX9_CAN_ERFFEL23_OFFSET         (0x305c)
#define IMX9_CAN_ERFFEL24_OFFSET         (0x3060)
#define IMX9_CAN_ERFFEL25_OFFSET         (0x3064)
#define IMX9_CAN_ERFFEL26_OFFSET         (0x3068)
#define IMX9_CAN_ERFFEL27_OFFSET         (0x306c)
#define IMX9_CAN_ERFFEL28_OFFSET         (0x3070)
#define IMX9_CAN_ERFFEL29_OFFSET         (0x3074)
#define IMX9_CAN_ERFFEL30_OFFSET         (0x3078)
#define IMX9_CAN_ERFFEL31_OFFSET         (0x307c)
#define IMX9_CAN_ERFFEL32_OFFSET         (0x3080)
#define IMX9_CAN_ERFFEL33_OFFSET         (0x3084)
#define IMX9_CAN_ERFFEL34_OFFSET         (0x3088)
#define IMX9_CAN_ERFFEL35_OFFSET         (0x308c)
#define IMX9_CAN_ERFFEL36_OFFSET         (0x3090)
#define IMX9_CAN_ERFFEL37_OFFSET         (0x3094)
#define IMX9_CAN_ERFFEL38_OFFSET         (0x3098)
#define IMX9_CAN_ERFFEL39_OFFSET         (0x309c)
#define IMX9_CAN_ERFFEL40_OFFSET         (0x30a0)
#define IMX9_CAN_ERFFEL41_OFFSET         (0x30a4)
#define IMX9_CAN_ERFFEL42_OFFSET         (0x30a8)
#define IMX9_CAN_ERFFEL43_OFFSET         (0x30ac)
#define IMX9_CAN_ERFFEL44_OFFSET         (0x30b0)
#define IMX9_CAN_ERFFEL45_OFFSET         (0x30b4)
#define IMX9_CAN_ERFFEL46_OFFSET         (0x30b8)
#define IMX9_CAN_ERFFEL47_OFFSET         (0x30bc)
#define IMX9_CAN_ERFFEL48_OFFSET         (0x30c0)
#define IMX9_CAN_ERFFEL49_OFFSET         (0x30c4)
#define IMX9_CAN_ERFFEL50_OFFSET         (0x30c8)
#define IMX9_CAN_ERFFEL51_OFFSET         (0x30cc)
#define IMX9_CAN_ERFFEL52_OFFSET         (0x30d0)
#define IMX9_CAN_ERFFEL53_OFFSET         (0x30d4)
#define IMX9_CAN_ERFFEL54_OFFSET         (0x30d8)
#define IMX9_CAN_ERFFEL55_OFFSET         (0x30dc)
#define IMX9_CAN_ERFFEL56_OFFSET         (0x30e0)
#define IMX9_CAN_ERFFEL57_OFFSET         (0x30e4)
#define IMX9_CAN_ERFFEL58_OFFSET         (0x30e8)
#define IMX9_CAN_ERFFEL59_OFFSET         (0x30ec)
#define IMX9_CAN_ERFFEL60_OFFSET         (0x30f0)
#define IMX9_CAN_ERFFEL61_OFFSET         (0x30f4)
#define IMX9_CAN_ERFFEL62_OFFSET         (0x30f8)
#define IMX9_CAN_ERFFEL63_OFFSET         (0x30fc)
#define IMX9_CAN_ERFFEL64_OFFSET         (0x3100)
#define IMX9_CAN_ERFFEL65_OFFSET         (0x3104)
#define IMX9_CAN_ERFFEL66_OFFSET         (0x3108)
#define IMX9_CAN_ERFFEL67_OFFSET         (0x310c)
#define IMX9_CAN_ERFFEL68_OFFSET         (0x3110)
#define IMX9_CAN_ERFFEL69_OFFSET         (0x3114)
#define IMX9_CAN_ERFFEL70_OFFSET         (0x3118)
#define IMX9_CAN_ERFFEL71_OFFSET         (0x311c)
#define IMX9_CAN_ERFFEL72_OFFSET         (0x3120)
#define IMX9_CAN_ERFFEL73_OFFSET         (0x3124)
#define IMX9_CAN_ERFFEL74_OFFSET         (0x3128)
#define IMX9_CAN_ERFFEL75_OFFSET         (0x312c)
#define IMX9_CAN_ERFFEL76_OFFSET         (0x3130)
#define IMX9_CAN_ERFFEL77_OFFSET         (0x3134)
#define IMX9_CAN_ERFFEL78_OFFSET         (0x3138)
#define IMX9_CAN_ERFFEL79_OFFSET         (0x313c)
#define IMX9_CAN_ERFFEL80_OFFSET         (0x3140)
#define IMX9_CAN_ERFFEL81_OFFSET         (0x3144)
#define IMX9_CAN_ERFFEL82_OFFSET         (0x3148)
#define IMX9_CAN_ERFFEL83_OFFSET         (0x314c)
#define IMX9_CAN_ERFFEL84_OFFSET         (0x3150)
#define IMX9_CAN_ERFFEL85_OFFSET         (0x3154)
#define IMX9_CAN_ERFFEL86_OFFSET         (0x3158)
#define IMX9_CAN_ERFFEL87_OFFSET         (0x315c)
#define IMX9_CAN_ERFFEL88_OFFSET         (0x3160)
#define IMX9_CAN_ERFFEL89_OFFSET         (0x3164)
#define IMX9_CAN_ERFFEL90_OFFSET         (0x3168)
#define IMX9_CAN_ERFFEL91_OFFSET         (0x316c)
#define IMX9_CAN_ERFFEL92_OFFSET         (0x3170)
#define IMX9_CAN_ERFFEL93_OFFSET         (0x3174)
#define IMX9_CAN_ERFFEL94_OFFSET         (0x3178)
#define IMX9_CAN_ERFFEL95_OFFSET         (0x317c)
#define IMX9_CAN_ERFFEL96_OFFSET         (0x3180)
#define IMX9_CAN_ERFFEL97_OFFSET         (0x3184)
#define IMX9_CAN_ERFFEL98_OFFSET         (0x3188)
#define IMX9_CAN_ERFFEL99_OFFSET         (0x318c)
#define IMX9_CAN_ERFFEL100_OFFSET        (0x3190)
#define IMX9_CAN_ERFFEL101_OFFSET        (0x3194)
#define IMX9_CAN_ERFFEL102_OFFSET        (0x3198)
#define IMX9_CAN_ERFFEL103_OFFSET        (0x319c)
#define IMX9_CAN_ERFFEL104_OFFSET        (0x31a0)
#define IMX9_CAN_ERFFEL105_OFFSET        (0x31a4)
#define IMX9_CAN_ERFFEL106_OFFSET        (0x31a8)
#define IMX9_CAN_ERFFEL107_OFFSET        (0x31ac)
#define IMX9_CAN_ERFFEL108_OFFSET        (0x31b0)
#define IMX9_CAN_ERFFEL109_OFFSET        (0x31b4)
#define IMX9_CAN_ERFFEL110_OFFSET        (0x31b8)
#define IMX9_CAN_ERFFEL111_OFFSET        (0x31bc)
#define IMX9_CAN_ERFFEL112_OFFSET        (0x31c0)
#define IMX9_CAN_ERFFEL113_OFFSET        (0x31c4)
#define IMX9_CAN_ERFFEL114_OFFSET        (0x31c8)
#define IMX9_CAN_ERFFEL115_OFFSET        (0x31cc)
#define IMX9_CAN_ERFFEL116_OFFSET        (0x31d0)
#define IMX9_CAN_ERFFEL117_OFFSET        (0x31d4)
#define IMX9_CAN_ERFFEL118_OFFSET        (0x31d8)
#define IMX9_CAN_ERFFEL119_OFFSET        (0x31dc)
#define IMX9_CAN_ERFFEL120_OFFSET        (0x31e0)
#define IMX9_CAN_ERFFEL121_OFFSET        (0x31e4)
#define IMX9_CAN_ERFFEL122_OFFSET        (0x31e8)
#define IMX9_CAN_ERFFEL123_OFFSET        (0x31ec)
#define IMX9_CAN_ERFFEL124_OFFSET        (0x31f0)
#define IMX9_CAN_ERFFEL125_OFFSET        (0x31f4)
#define IMX9_CAN_ERFFEL126_OFFSET        (0x31f8)
#define IMX9_CAN_ERFFEL127_OFFSET        (0x31fc)

#define IMX9_CAN_MB_OFFSET       IMX9_CAN_CS0_OFFSET
#define IMX9_CAN_MB_END          0x0ADF

#define IMX9_CAN_MB2_OFFSET      0x0C20 /* CAN MB2 register */
#define IMX9_CAN_MB2_END         0x31FF

/* CAN Register Addresses ***************************************************/
#define IMX9_CAN_MCR              (IMX9_CAN_BASE + IMX9_CAN_MCR_OFFSET)
#define IMX9_CAN_CTRL1            (IMX9_CAN_BASE + IMX9_CAN_CTRL1_OFFSET)
#define IMX9_CAN_TIMER            (IMX9_CAN_BASE + IMX9_CAN_TIMER_OFFSET)
#define IMX9_CAN_RXMGMASK         (IMX9_CAN_BASE + IMX9_CAN_RXMGMASK_OFFSET)
#define IMX9_CAN_RX14MASK         (IMX9_CAN_BASE + IMX9_CAN_RX14MASK_OFFSET)
#define IMX9_CAN_RX15MASK         (IMX9_CAN_BASE + IMX9_CAN_RX15MASK_OFFSET)
#define IMX9_CAN_ECR              (IMX9_CAN_BASE + IMX9_CAN_ECR_OFFSET)
#define IMX9_CAN_ESR1             (IMX9_CAN_BASE + IMX9_CAN_ESR1_OFFSET)
#define IMX9_CAN_IMASK2           (IMX9_CAN_BASE + IMX9_CAN_IMASK2_OFFSET)
#define IMX9_CAN_IMASK1           (IMX9_CAN_BASE + IMX9_CAN_IMASK1_OFFSET)
#define IMX9_CAN_IFLAG2           (IMX9_CAN_BASE + IMX9_CAN_IFLAG2_OFFSET)
#define IMX9_CAN_IFLAG1           (IMX9_CAN_BASE + IMX9_CAN_IFLAG1_OFFSET)
#define IMX9_CAN_CTRL2            (IMX9_CAN_BASE + IMX9_CAN_CTRL2_OFFSET)
#define IMX9_CAN_ESR2             (IMX9_CAN_BASE + IMX9_CAN_ESR2_OFFSET)
#define IMX9_CAN_CRCR             (IMX9_CAN_BASE + IMX9_CAN_CRCR_OFFSET)
#define IMX9_CAN_RXFGMASK         (IMX9_CAN_BASE + IMX9_CAN_RXFGMASK_OFFSET)
#define IMX9_CAN_RXFIR            (IMX9_CAN_BASE + IMX9_CAN_RXFIR_OFFSET)
#define IMX9_CAN_CBT              (IMX9_CAN_BASE + IMX9_CAN_CBT_OFFSET)
#define IMX9_CAN_IMASK3           (IMX9_CAN_BASE + IMX9_CAN_IMASK3_OFFSET)
#define IMX9_CAN_IFLAG3           (IMX9_CAN_BASE + IMX9_CAN_IFLAG3_OFFSET)
#define IMX9_CAN_CS0              (IMX9_CAN_BASE + IMX9_CAN_CS0_OFFSET)
#define IMX9_CAN_WORD00           (IMX9_CAN_BASE + IMX9_CAN_WORD00_OFFSET)
#define IMX9_CAN_WORD10           (IMX9_CAN_BASE + IMX9_CAN_WORD10_OFFSET)
#define IMX9_CAN_CS1              (IMX9_CAN_BASE + IMX9_CAN_CS1_OFFSET)
#define IMX9_CAN_WORD01           (IMX9_CAN_BASE + IMX9_CAN_WORD01_OFFSET)
#define IMX9_CAN_WORD11           (IMX9_CAN_BASE + IMX9_CAN_WORD11_OFFSET)
#define IMX9_CAN_CS2              (IMX9_CAN_BASE + IMX9_CAN_CS2_OFFSET)
#define IMX9_CAN_WORD02           (IMX9_CAN_BASE + IMX9_CAN_WORD02_OFFSET)
#define IMX9_CAN_WORD12           (IMX9_CAN_BASE + IMX9_CAN_WORD12_OFFSET)
#define IMX9_CAN_CS3              (IMX9_CAN_BASE + IMX9_CAN_CS3_OFFSET)
#define IMX9_CAN_WORD03           (IMX9_CAN_BASE + IMX9_CAN_WORD03_OFFSET)
#define IMX9_CAN_WORD13           (IMX9_CAN_BASE + IMX9_CAN_WORD13_OFFSET)
#define IMX9_CAN_CS4              (IMX9_CAN_BASE + IMX9_CAN_CS4_OFFSET)
#define IMX9_CAN_WORD04           (IMX9_CAN_BASE + IMX9_CAN_WORD04_OFFSET)
#define IMX9_CAN_WORD14           (IMX9_CAN_BASE + IMX9_CAN_WORD14_OFFSET)
#define IMX9_CAN_CS5              (IMX9_CAN_BASE + IMX9_CAN_CS5_OFFSET)
#define IMX9_CAN_WORD05           (IMX9_CAN_BASE + IMX9_CAN_WORD05_OFFSET)
#define IMX9_CAN_WORD15           (IMX9_CAN_BASE + IMX9_CAN_WORD15_OFFSET)
#define IMX9_CAN_CS6              (IMX9_CAN_BASE + IMX9_CAN_CS6_OFFSET)
#define IMX9_CAN_WORD06           (IMX9_CAN_BASE + IMX9_CAN_WORD06_OFFSET)
#define IMX9_CAN_WORD16           (IMX9_CAN_BASE + IMX9_CAN_WORD16_OFFSET)
#define IMX9_CAN_CS7              (IMX9_CAN_BASE + IMX9_CAN_CS7_OFFSET)
#define IMX9_CAN_WORD07           (IMX9_CAN_BASE + IMX9_CAN_WORD07_OFFSET)
#define IMX9_CAN_WORD17           (IMX9_CAN_BASE + IMX9_CAN_WORD17_OFFSET)
#define IMX9_CAN_CS8              (IMX9_CAN_BASE + IMX9_CAN_CS8_OFFSET)
#define IMX9_CAN_WORD08           (IMX9_CAN_BASE + IMX9_CAN_WORD08_OFFSET)
#define IMX9_CAN_WORD18           (IMX9_CAN_BASE + IMX9_CAN_WORD18_OFFSET)
#define IMX9_CAN_CS9              (IMX9_CAN_BASE + IMX9_CAN_CS9_OFFSET)
#define IMX9_CAN_WORD09           (IMX9_CAN_BASE + IMX9_CAN_WORD09_OFFSET)
#define IMX9_CAN_WORD19           (IMX9_CAN_BASE + IMX9_CAN_WORD19_OFFSET)
#define IMX9_CAN_CS10             (IMX9_CAN_BASE + IMX9_CAN_CS10_OFFSET)
#define IMX9_CAN_WORD010          (IMX9_CAN_BASE + IMX9_CAN_WORD010_OFFSET)
#define IMX9_CAN_WORD110          (IMX9_CAN_BASE + IMX9_CAN_WORD110_OFFSET)
#define IMX9_CAN_CS11             (IMX9_CAN_BASE + IMX9_CAN_CS11_OFFSET)
#define IMX9_CAN_WORD011          (IMX9_CAN_BASE + IMX9_CAN_WORD011_OFFSET)
#define IMX9_CAN_WORD111          (IMX9_CAN_BASE + IMX9_CAN_WORD111_OFFSET)
#define IMX9_CAN_CS12             (IMX9_CAN_BASE + IMX9_CAN_CS12_OFFSET)
#define IMX9_CAN_WORD012          (IMX9_CAN_BASE + IMX9_CAN_WORD012_OFFSET)
#define IMX9_CAN_WORD112          (IMX9_CAN_BASE + IMX9_CAN_WORD112_OFFSET)
#define IMX9_CAN_CS13             (IMX9_CAN_BASE + IMX9_CAN_CS13_OFFSET)
#define IMX9_CAN_WORD013          (IMX9_CAN_BASE + IMX9_CAN_WORD013_OFFSET)
#define IMX9_CAN_WORD113          (IMX9_CAN_BASE + IMX9_CAN_WORD113_OFFSET)
#define IMX9_CAN_CS14             (IMX9_CAN_BASE + IMX9_CAN_CS14_OFFSET)
#define IMX9_CAN_WORD014          (IMX9_CAN_BASE + IMX9_CAN_WORD014_OFFSET)
#define IMX9_CAN_WORD114          (IMX9_CAN_BASE + IMX9_CAN_WORD114_OFFSET)
#define IMX9_CAN_CS15             (IMX9_CAN_BASE + IMX9_CAN_CS15_OFFSET)
#define IMX9_CAN_WORD015          (IMX9_CAN_BASE + IMX9_CAN_WORD015_OFFSET)
#define IMX9_CAN_WORD115          (IMX9_CAN_BASE + IMX9_CAN_WORD115_OFFSET)
#define IMX9_CAN_CS16             (IMX9_CAN_BASE + IMX9_CAN_CS16_OFFSET)
#define IMX9_CAN_WORD016          (IMX9_CAN_BASE + IMX9_CAN_WORD016_OFFSET)
#define IMX9_CAN_WORD116          (IMX9_CAN_BASE + IMX9_CAN_WORD116_OFFSET)
#define IMX9_CAN_CS17             (IMX9_CAN_BASE + IMX9_CAN_CS17_OFFSET)
#define IMX9_CAN_WORD017          (IMX9_CAN_BASE + IMX9_CAN_WORD017_OFFSET)
#define IMX9_CAN_WORD117          (IMX9_CAN_BASE + IMX9_CAN_WORD117_OFFSET)
#define IMX9_CAN_CS18             (IMX9_CAN_BASE + IMX9_CAN_CS18_OFFSET)
#define IMX9_CAN_WORD018          (IMX9_CAN_BASE + IMX9_CAN_WORD018_OFFSET)
#define IMX9_CAN_WORD118          (IMX9_CAN_BASE + IMX9_CAN_WORD118_OFFSET)
#define IMX9_CAN_CS19             (IMX9_CAN_BASE + IMX9_CAN_CS19_OFFSET)
#define IMX9_CAN_WORD019          (IMX9_CAN_BASE + IMX9_CAN_WORD019_OFFSET)
#define IMX9_CAN_WORD119          (IMX9_CAN_BASE + IMX9_CAN_WORD119_OFFSET)
#define IMX9_CAN_CS20             (IMX9_CAN_BASE + IMX9_CAN_CS20_OFFSET)
#define IMX9_CAN_WORD020          (IMX9_CAN_BASE + IMX9_CAN_WORD020_OFFSET)
#define IMX9_CAN_WORD120          (IMX9_CAN_BASE + IMX9_CAN_WORD120_OFFSET)
#define IMX9_CAN_CS21             (IMX9_CAN_BASE + IMX9_CAN_CS21_OFFSET)
#define IMX9_CAN_WORD021          (IMX9_CAN_BASE + IMX9_CAN_WORD021_OFFSET)
#define IMX9_CAN_WORD121          (IMX9_CAN_BASE + IMX9_CAN_WORD121_OFFSET)
#define IMX9_CAN_CS22             (IMX9_CAN_BASE + IMX9_CAN_CS22_OFFSET)
#define IMX9_CAN_WORD022          (IMX9_CAN_BASE + IMX9_CAN_WORD022_OFFSET)
#define IMX9_CAN_WORD122          (IMX9_CAN_BASE + IMX9_CAN_WORD122_OFFSET)
#define IMX9_CAN_CS23             (IMX9_CAN_BASE + IMX9_CAN_CS23_OFFSET)
#define IMX9_CAN_WORD023          (IMX9_CAN_BASE + IMX9_CAN_WORD023_OFFSET)
#define IMX9_CAN_WORD123          (IMX9_CAN_BASE + IMX9_CAN_WORD123_OFFSET)
#define IMX9_CAN_CS24             (IMX9_CAN_BASE + IMX9_CAN_CS24_OFFSET)
#define IMX9_CAN_WORD024          (IMX9_CAN_BASE + IMX9_CAN_WORD024_OFFSET)
#define IMX9_CAN_WORD124          (IMX9_CAN_BASE + IMX9_CAN_WORD124_OFFSET)
#define IMX9_CAN_CS25             (IMX9_CAN_BASE + IMX9_CAN_CS25_OFFSET)
#define IMX9_CAN_WORD025          (IMX9_CAN_BASE + IMX9_CAN_WORD025_OFFSET)
#define IMX9_CAN_WORD125          (IMX9_CAN_BASE + IMX9_CAN_WORD125_OFFSET)
#define IMX9_CAN_CS26             (IMX9_CAN_BASE + IMX9_CAN_CS26_OFFSET)
#define IMX9_CAN_WORD026          (IMX9_CAN_BASE + IMX9_CAN_WORD026_OFFSET)
#define IMX9_CAN_WORD126          (IMX9_CAN_BASE + IMX9_CAN_WORD126_OFFSET)
#define IMX9_CAN_CS27             (IMX9_CAN_BASE + IMX9_CAN_CS27_OFFSET)
#define IMX9_CAN_WORD027          (IMX9_CAN_BASE + IMX9_CAN_WORD027_OFFSET)
#define IMX9_CAN_WORD127          (IMX9_CAN_BASE + IMX9_CAN_WORD127_OFFSET)
#define IMX9_CAN_CS28             (IMX9_CAN_BASE + IMX9_CAN_CS28_OFFSET)
#define IMX9_CAN_WORD028          (IMX9_CAN_BASE + IMX9_CAN_WORD028_OFFSET)
#define IMX9_CAN_WORD128          (IMX9_CAN_BASE + IMX9_CAN_WORD128_OFFSET)
#define IMX9_CAN_CS29             (IMX9_CAN_BASE + IMX9_CAN_CS29_OFFSET)
#define IMX9_CAN_WORD029          (IMX9_CAN_BASE + IMX9_CAN_WORD029_OFFSET)
#define IMX9_CAN_WORD129          (IMX9_CAN_BASE + IMX9_CAN_WORD129_OFFSET)
#define IMX9_CAN_CS30             (IMX9_CAN_BASE + IMX9_CAN_CS30_OFFSET)
#define IMX9_CAN_WORD030          (IMX9_CAN_BASE + IMX9_CAN_WORD030_OFFSET)
#define IMX9_CAN_WORD130          (IMX9_CAN_BASE + IMX9_CAN_WORD130_OFFSET)
#define IMX9_CAN_CS31             (IMX9_CAN_BASE + IMX9_CAN_CS31_OFFSET)
#define IMX9_CAN_WORD031          (IMX9_CAN_BASE + IMX9_CAN_WORD031_OFFSET)
#define IMX9_CAN_WORD131          (IMX9_CAN_BASE + IMX9_CAN_WORD131_OFFSET)
#define IMX9_CAN_CS32             (IMX9_CAN_BASE + IMX9_CAN_CS32_OFFSET)
#define IMX9_CAN_WORD032          (IMX9_CAN_BASE + IMX9_CAN_WORD032_OFFSET)
#define IMX9_CAN_WORD132          (IMX9_CAN_BASE + IMX9_CAN_WORD132_OFFSET)
#define IMX9_CAN_CS33             (IMX9_CAN_BASE + IMX9_CAN_CS33_OFFSET)
#define IMX9_CAN_WORD033          (IMX9_CAN_BASE + IMX9_CAN_WORD033_OFFSET)
#define IMX9_CAN_WORD133          (IMX9_CAN_BASE + IMX9_CAN_WORD133_OFFSET)
#define IMX9_CAN_CS34             (IMX9_CAN_BASE + IMX9_CAN_CS34_OFFSET)
#define IMX9_CAN_WORD034          (IMX9_CAN_BASE + IMX9_CAN_WORD034_OFFSET)
#define IMX9_CAN_WORD134          (IMX9_CAN_BASE + IMX9_CAN_WORD134_OFFSET)
#define IMX9_CAN_CS35             (IMX9_CAN_BASE + IMX9_CAN_CS35_OFFSET)
#define IMX9_CAN_WORD035          (IMX9_CAN_BASE + IMX9_CAN_WORD035_OFFSET)
#define IMX9_CAN_WORD135          (IMX9_CAN_BASE + IMX9_CAN_WORD135_OFFSET)
#define IMX9_CAN_CS36             (IMX9_CAN_BASE + IMX9_CAN_CS36_OFFSET)
#define IMX9_CAN_WORD036          (IMX9_CAN_BASE + IMX9_CAN_WORD036_OFFSET)
#define IMX9_CAN_WORD136          (IMX9_CAN_BASE + IMX9_CAN_WORD136_OFFSET)
#define IMX9_CAN_CS37             (IMX9_CAN_BASE + IMX9_CAN_CS37_OFFSET)
#define IMX9_CAN_WORD037          (IMX9_CAN_BASE + IMX9_CAN_WORD037_OFFSET)
#define IMX9_CAN_WORD137          (IMX9_CAN_BASE + IMX9_CAN_WORD137_OFFSET)
#define IMX9_CAN_CS38             (IMX9_CAN_BASE + IMX9_CAN_CS38_OFFSET)
#define IMX9_CAN_WORD038          (IMX9_CAN_BASE + IMX9_CAN_WORD038_OFFSET)
#define IMX9_CAN_WORD138          (IMX9_CAN_BASE + IMX9_CAN_WORD138_OFFSET)
#define IMX9_CAN_CS39             (IMX9_CAN_BASE + IMX9_CAN_CS39_OFFSET)
#define IMX9_CAN_WORD039          (IMX9_CAN_BASE + IMX9_CAN_WORD039_OFFSET)
#define IMX9_CAN_WORD139          (IMX9_CAN_BASE + IMX9_CAN_WORD139_OFFSET)
#define IMX9_CAN_CS40             (IMX9_CAN_BASE + IMX9_CAN_CS40_OFFSET)
#define IMX9_CAN_WORD040          (IMX9_CAN_BASE + IMX9_CAN_WORD040_OFFSET)
#define IMX9_CAN_WORD140          (IMX9_CAN_BASE + IMX9_CAN_WORD140_OFFSET)
#define IMX9_CAN_CS41             (IMX9_CAN_BASE + IMX9_CAN_CS41_OFFSET)
#define IMX9_CAN_WORD041          (IMX9_CAN_BASE + IMX9_CAN_WORD041_OFFSET)
#define IMX9_CAN_WORD141          (IMX9_CAN_BASE + IMX9_CAN_WORD141_OFFSET)
#define IMX9_CAN_CS42             (IMX9_CAN_BASE + IMX9_CAN_CS42_OFFSET)
#define IMX9_CAN_WORD042          (IMX9_CAN_BASE + IMX9_CAN_WORD042_OFFSET)
#define IMX9_CAN_WORD142          (IMX9_CAN_BASE + IMX9_CAN_WORD142_OFFSET)
#define IMX9_CAN_CS43             (IMX9_CAN_BASE + IMX9_CAN_CS43_OFFSET)
#define IMX9_CAN_WORD043          (IMX9_CAN_BASE + IMX9_CAN_WORD043_OFFSET)
#define IMX9_CAN_WORD143          (IMX9_CAN_BASE + IMX9_CAN_WORD143_OFFSET)
#define IMX9_CAN_CS44             (IMX9_CAN_BASE + IMX9_CAN_CS44_OFFSET)
#define IMX9_CAN_WORD044          (IMX9_CAN_BASE + IMX9_CAN_WORD044_OFFSET)
#define IMX9_CAN_WORD144          (IMX9_CAN_BASE + IMX9_CAN_WORD144_OFFSET)
#define IMX9_CAN_CS45             (IMX9_CAN_BASE + IMX9_CAN_CS45_OFFSET)
#define IMX9_CAN_WORD045          (IMX9_CAN_BASE + IMX9_CAN_WORD045_OFFSET)
#define IMX9_CAN_WORD145          (IMX9_CAN_BASE + IMX9_CAN_WORD145_OFFSET)
#define IMX9_CAN_CS46             (IMX9_CAN_BASE + IMX9_CAN_CS46_OFFSET)
#define IMX9_CAN_WORD046          (IMX9_CAN_BASE + IMX9_CAN_WORD046_OFFSET)
#define IMX9_CAN_WORD146          (IMX9_CAN_BASE + IMX9_CAN_WORD146_OFFSET)
#define IMX9_CAN_CS47             (IMX9_CAN_BASE + IMX9_CAN_CS47_OFFSET)
#define IMX9_CAN_WORD047          (IMX9_CAN_BASE + IMX9_CAN_WORD047_OFFSET)
#define IMX9_CAN_WORD147          (IMX9_CAN_BASE + IMX9_CAN_WORD147_OFFSET)
#define IMX9_CAN_CS48             (IMX9_CAN_BASE + IMX9_CAN_CS48_OFFSET)
#define IMX9_CAN_WORD048          (IMX9_CAN_BASE + IMX9_CAN_WORD048_OFFSET)
#define IMX9_CAN_WORD148          (IMX9_CAN_BASE + IMX9_CAN_WORD148_OFFSET)
#define IMX9_CAN_CS49             (IMX9_CAN_BASE + IMX9_CAN_CS49_OFFSET)
#define IMX9_CAN_WORD049          (IMX9_CAN_BASE + IMX9_CAN_WORD049_OFFSET)
#define IMX9_CAN_WORD149          (IMX9_CAN_BASE + IMX9_CAN_WORD149_OFFSET)
#define IMX9_CAN_CS50             (IMX9_CAN_BASE + IMX9_CAN_CS50_OFFSET)
#define IMX9_CAN_WORD050          (IMX9_CAN_BASE + IMX9_CAN_WORD050_OFFSET)
#define IMX9_CAN_WORD150          (IMX9_CAN_BASE + IMX9_CAN_WORD150_OFFSET)
#define IMX9_CAN_CS51             (IMX9_CAN_BASE + IMX9_CAN_CS51_OFFSET)
#define IMX9_CAN_WORD051          (IMX9_CAN_BASE + IMX9_CAN_WORD051_OFFSET)
#define IMX9_CAN_WORD151          (IMX9_CAN_BASE + IMX9_CAN_WORD151_OFFSET)
#define IMX9_CAN_CS52             (IMX9_CAN_BASE + IMX9_CAN_CS52_OFFSET)
#define IMX9_CAN_WORD052          (IMX9_CAN_BASE + IMX9_CAN_WORD052_OFFSET)
#define IMX9_CAN_WORD152          (IMX9_CAN_BASE + IMX9_CAN_WORD152_OFFSET)
#define IMX9_CAN_CS53             (IMX9_CAN_BASE + IMX9_CAN_CS53_OFFSET)
#define IMX9_CAN_WORD053          (IMX9_CAN_BASE + IMX9_CAN_WORD053_OFFSET)
#define IMX9_CAN_WORD153          (IMX9_CAN_BASE + IMX9_CAN_WORD153_OFFSET)
#define IMX9_CAN_CS54             (IMX9_CAN_BASE + IMX9_CAN_CS54_OFFSET)
#define IMX9_CAN_WORD054          (IMX9_CAN_BASE + IMX9_CAN_WORD054_OFFSET)
#define IMX9_CAN_WORD154          (IMX9_CAN_BASE + IMX9_CAN_WORD154_OFFSET)
#define IMX9_CAN_CS55             (IMX9_CAN_BASE + IMX9_CAN_CS55_OFFSET)
#define IMX9_CAN_WORD055          (IMX9_CAN_BASE + IMX9_CAN_WORD055_OFFSET)
#define IMX9_CAN_WORD155          (IMX9_CAN_BASE + IMX9_CAN_WORD155_OFFSET)
#define IMX9_CAN_CS56             (IMX9_CAN_BASE + IMX9_CAN_CS56_OFFSET)
#define IMX9_CAN_WORD056          (IMX9_CAN_BASE + IMX9_CAN_WORD056_OFFSET)
#define IMX9_CAN_WORD156          (IMX9_CAN_BASE + IMX9_CAN_WORD156_OFFSET)
#define IMX9_CAN_CS57             (IMX9_CAN_BASE + IMX9_CAN_CS57_OFFSET)
#define IMX9_CAN_WORD057          (IMX9_CAN_BASE + IMX9_CAN_WORD057_OFFSET)
#define IMX9_CAN_WORD157          (IMX9_CAN_BASE + IMX9_CAN_WORD157_OFFSET)
#define IMX9_CAN_CS58             (IMX9_CAN_BASE + IMX9_CAN_CS58_OFFSET)
#define IMX9_CAN_WORD058          (IMX9_CAN_BASE + IMX9_CAN_WORD058_OFFSET)
#define IMX9_CAN_WORD158          (IMX9_CAN_BASE + IMX9_CAN_WORD158_OFFSET)
#define IMX9_CAN_CS59             (IMX9_CAN_BASE + IMX9_CAN_CS59_OFFSET)
#define IMX9_CAN_WORD059          (IMX9_CAN_BASE + IMX9_CAN_WORD059_OFFSET)
#define IMX9_CAN_WORD159          (IMX9_CAN_BASE + IMX9_CAN_WORD159_OFFSET)
#define IMX9_CAN_CS60             (IMX9_CAN_BASE + IMX9_CAN_CS60_OFFSET)
#define IMX9_CAN_WORD060          (IMX9_CAN_BASE + IMX9_CAN_WORD060_OFFSET)
#define IMX9_CAN_WORD160          (IMX9_CAN_BASE + IMX9_CAN_WORD160_OFFSET)
#define IMX9_CAN_CS61             (IMX9_CAN_BASE + IMX9_CAN_CS61_OFFSET)
#define IMX9_CAN_WORD061          (IMX9_CAN_BASE + IMX9_CAN_WORD061_OFFSET)
#define IMX9_CAN_WORD161          (IMX9_CAN_BASE + IMX9_CAN_WORD161_OFFSET)
#define IMX9_CAN_CS62             (IMX9_CAN_BASE + IMX9_CAN_CS62_OFFSET)
#define IMX9_CAN_WORD062          (IMX9_CAN_BASE + IMX9_CAN_WORD062_OFFSET)
#define IMX9_CAN_WORD162          (IMX9_CAN_BASE + IMX9_CAN_WORD162_OFFSET)
#define IMX9_CAN_CS63             (IMX9_CAN_BASE + IMX9_CAN_CS63_OFFSET)
#define IMX9_CAN_WORD063          (IMX9_CAN_BASE + IMX9_CAN_WORD063_OFFSET)
#define IMX9_CAN_WORD163          (IMX9_CAN_BASE + IMX9_CAN_WORD163_OFFSET)
#define IMX9_CAN_CS64             (IMX9_CAN_BASE + IMX9_CAN_CS64_OFFSET)
#define IMX9_CAN_WORD064          (IMX9_CAN_BASE + IMX9_CAN_WORD064_OFFSET)
#define IMX9_CAN_WORD164          (IMX9_CAN_BASE + IMX9_CAN_WORD164_OFFSET)
#define IMX9_CAN_CS65             (IMX9_CAN_BASE + IMX9_CAN_CS65_OFFSET)
#define IMX9_CAN_WORD065          (IMX9_CAN_BASE + IMX9_CAN_WORD065_OFFSET)
#define IMX9_CAN_WORD165          (IMX9_CAN_BASE + IMX9_CAN_WORD165_OFFSET)
#define IMX9_CAN_CS66             (IMX9_CAN_BASE + IMX9_CAN_CS66_OFFSET)
#define IMX9_CAN_WORD066          (IMX9_CAN_BASE + IMX9_CAN_WORD066_OFFSET)
#define IMX9_CAN_WORD166          (IMX9_CAN_BASE + IMX9_CAN_WORD166_OFFSET)
#define IMX9_CAN_CS67             (IMX9_CAN_BASE + IMX9_CAN_CS67_OFFSET)
#define IMX9_CAN_WORD067          (IMX9_CAN_BASE + IMX9_CAN_WORD067_OFFSET)
#define IMX9_CAN_WORD167          (IMX9_CAN_BASE + IMX9_CAN_WORD167_OFFSET)
#define IMX9_CAN_CS68             (IMX9_CAN_BASE + IMX9_CAN_CS68_OFFSET)
#define IMX9_CAN_WORD068          (IMX9_CAN_BASE + IMX9_CAN_WORD068_OFFSET)
#define IMX9_CAN_WORD168          (IMX9_CAN_BASE + IMX9_CAN_WORD168_OFFSET)
#define IMX9_CAN_CS69             (IMX9_CAN_BASE + IMX9_CAN_CS69_OFFSET)
#define IMX9_CAN_WORD069          (IMX9_CAN_BASE + IMX9_CAN_WORD069_OFFSET)
#define IMX9_CAN_WORD169          (IMX9_CAN_BASE + IMX9_CAN_WORD169_OFFSET)
#define IMX9_CAN_CS70             (IMX9_CAN_BASE + IMX9_CAN_CS70_OFFSET)
#define IMX9_CAN_WORD070          (IMX9_CAN_BASE + IMX9_CAN_WORD070_OFFSET)
#define IMX9_CAN_WORD170          (IMX9_CAN_BASE + IMX9_CAN_WORD170_OFFSET)
#define IMX9_CAN_CS71             (IMX9_CAN_BASE + IMX9_CAN_CS71_OFFSET)
#define IMX9_CAN_WORD071          (IMX9_CAN_BASE + IMX9_CAN_WORD071_OFFSET)
#define IMX9_CAN_WORD171          (IMX9_CAN_BASE + IMX9_CAN_WORD171_OFFSET)
#define IMX9_CAN_CS72             (IMX9_CAN_BASE + IMX9_CAN_CS72_OFFSET)
#define IMX9_CAN_WORD072          (IMX9_CAN_BASE + IMX9_CAN_WORD072_OFFSET)
#define IMX9_CAN_WORD172          (IMX9_CAN_BASE + IMX9_CAN_WORD172_OFFSET)
#define IMX9_CAN_CS73             (IMX9_CAN_BASE + IMX9_CAN_CS73_OFFSET)
#define IMX9_CAN_WORD073          (IMX9_CAN_BASE + IMX9_CAN_WORD073_OFFSET)
#define IMX9_CAN_WORD173          (IMX9_CAN_BASE + IMX9_CAN_WORD173_OFFSET)
#define IMX9_CAN_CS74             (IMX9_CAN_BASE + IMX9_CAN_CS74_OFFSET)
#define IMX9_CAN_WORD074          (IMX9_CAN_BASE + IMX9_CAN_WORD074_OFFSET)
#define IMX9_CAN_WORD174          (IMX9_CAN_BASE + IMX9_CAN_WORD174_OFFSET)
#define IMX9_CAN_CS75             (IMX9_CAN_BASE + IMX9_CAN_CS75_OFFSET)
#define IMX9_CAN_WORD075          (IMX9_CAN_BASE + IMX9_CAN_WORD075_OFFSET)
#define IMX9_CAN_WORD175          (IMX9_CAN_BASE + IMX9_CAN_WORD175_OFFSET)
#define IMX9_CAN_CS76             (IMX9_CAN_BASE + IMX9_CAN_CS76_OFFSET)
#define IMX9_CAN_WORD076          (IMX9_CAN_BASE + IMX9_CAN_WORD076_OFFSET)
#define IMX9_CAN_WORD176          (IMX9_CAN_BASE + IMX9_CAN_WORD176_OFFSET)
#define IMX9_CAN_CS77             (IMX9_CAN_BASE + IMX9_CAN_CS77_OFFSET)
#define IMX9_CAN_WORD077          (IMX9_CAN_BASE + IMX9_CAN_WORD077_OFFSET)
#define IMX9_CAN_WORD177          (IMX9_CAN_BASE + IMX9_CAN_WORD177_OFFSET)
#define IMX9_CAN_CS78             (IMX9_CAN_BASE + IMX9_CAN_CS78_OFFSET)
#define IMX9_CAN_WORD078          (IMX9_CAN_BASE + IMX9_CAN_WORD078_OFFSET)
#define IMX9_CAN_WORD178          (IMX9_CAN_BASE + IMX9_CAN_WORD178_OFFSET)
#define IMX9_CAN_CS79             (IMX9_CAN_BASE + IMX9_CAN_CS79_OFFSET)
#define IMX9_CAN_WORD079          (IMX9_CAN_BASE + IMX9_CAN_WORD079_OFFSET)
#define IMX9_CAN_WORD179          (IMX9_CAN_BASE + IMX9_CAN_WORD179_OFFSET)
#define IMX9_CAN_CS80             (IMX9_CAN_BASE + IMX9_CAN_CS80_OFFSET)
#define IMX9_CAN_WORD080          (IMX9_CAN_BASE + IMX9_CAN_WORD080_OFFSET)
#define IMX9_CAN_WORD180          (IMX9_CAN_BASE + IMX9_CAN_WORD180_OFFSET)
#define IMX9_CAN_CS81             (IMX9_CAN_BASE + IMX9_CAN_CS81_OFFSET)
#define IMX9_CAN_WORD081          (IMX9_CAN_BASE + IMX9_CAN_WORD081_OFFSET)
#define IMX9_CAN_WORD181          (IMX9_CAN_BASE + IMX9_CAN_WORD181_OFFSET)
#define IMX9_CAN_CS82             (IMX9_CAN_BASE + IMX9_CAN_CS82_OFFSET)
#define IMX9_CAN_WORD082          (IMX9_CAN_BASE + IMX9_CAN_WORD082_OFFSET)
#define IMX9_CAN_WORD182          (IMX9_CAN_BASE + IMX9_CAN_WORD182_OFFSET)
#define IMX9_CAN_CS83             (IMX9_CAN_BASE + IMX9_CAN_CS83_OFFSET)
#define IMX9_CAN_WORD083          (IMX9_CAN_BASE + IMX9_CAN_WORD083_OFFSET)
#define IMX9_CAN_WORD183          (IMX9_CAN_BASE + IMX9_CAN_WORD183_OFFSET)
#define IMX9_CAN_CS84             (IMX9_CAN_BASE + IMX9_CAN_CS84_OFFSET)
#define IMX9_CAN_WORD084          (IMX9_CAN_BASE + IMX9_CAN_WORD084_OFFSET)
#define IMX9_CAN_WORD184          (IMX9_CAN_BASE + IMX9_CAN_WORD184_OFFSET)
#define IMX9_CAN_CS85             (IMX9_CAN_BASE + IMX9_CAN_CS85_OFFSET)
#define IMX9_CAN_WORD085          (IMX9_CAN_BASE + IMX9_CAN_WORD085_OFFSET)
#define IMX9_CAN_WORD185          (IMX9_CAN_BASE + IMX9_CAN_WORD185_OFFSET)
#define IMX9_CAN_CS86             (IMX9_CAN_BASE + IMX9_CAN_CS86_OFFSET)
#define IMX9_CAN_WORD086          (IMX9_CAN_BASE + IMX9_CAN_WORD086_OFFSET)
#define IMX9_CAN_WORD186          (IMX9_CAN_BASE + IMX9_CAN_WORD186_OFFSET)
#define IMX9_CAN_CS87             (IMX9_CAN_BASE + IMX9_CAN_CS87_OFFSET)
#define IMX9_CAN_WORD087          (IMX9_CAN_BASE + IMX9_CAN_WORD087_OFFSET)
#define IMX9_CAN_WORD187          (IMX9_CAN_BASE + IMX9_CAN_WORD187_OFFSET)
#define IMX9_CAN_CS88             (IMX9_CAN_BASE + IMX9_CAN_CS88_OFFSET)
#define IMX9_CAN_WORD088          (IMX9_CAN_BASE + IMX9_CAN_WORD088_OFFSET)
#define IMX9_CAN_WORD188          (IMX9_CAN_BASE + IMX9_CAN_WORD188_OFFSET)
#define IMX9_CAN_CS89             (IMX9_CAN_BASE + IMX9_CAN_CS89_OFFSET)
#define IMX9_CAN_WORD089          (IMX9_CAN_BASE + IMX9_CAN_WORD089_OFFSET)
#define IMX9_CAN_WORD189          (IMX9_CAN_BASE + IMX9_CAN_WORD189_OFFSET)
#define IMX9_CAN_CS90             (IMX9_CAN_BASE + IMX9_CAN_CS90_OFFSET)
#define IMX9_CAN_WORD090          (IMX9_CAN_BASE + IMX9_CAN_WORD090_OFFSET)
#define IMX9_CAN_WORD190          (IMX9_CAN_BASE + IMX9_CAN_WORD190_OFFSET)
#define IMX9_CAN_CS91             (IMX9_CAN_BASE + IMX9_CAN_CS91_OFFSET)
#define IMX9_CAN_WORD091          (IMX9_CAN_BASE + IMX9_CAN_WORD091_OFFSET)
#define IMX9_CAN_WORD191          (IMX9_CAN_BASE + IMX9_CAN_WORD191_OFFSET)
#define IMX9_CAN_CS92             (IMX9_CAN_BASE + IMX9_CAN_CS92_OFFSET)
#define IMX9_CAN_WORD092          (IMX9_CAN_BASE + IMX9_CAN_WORD092_OFFSET)
#define IMX9_CAN_WORD192          (IMX9_CAN_BASE + IMX9_CAN_WORD192_OFFSET)
#define IMX9_CAN_CS93             (IMX9_CAN_BASE + IMX9_CAN_CS93_OFFSET)
#define IMX9_CAN_WORD093          (IMX9_CAN_BASE + IMX9_CAN_WORD093_OFFSET)
#define IMX9_CAN_WORD193          (IMX9_CAN_BASE + IMX9_CAN_WORD193_OFFSET)
#define IMX9_CAN_CS94             (IMX9_CAN_BASE + IMX9_CAN_CS94_OFFSET)
#define IMX9_CAN_WORD094          (IMX9_CAN_BASE + IMX9_CAN_WORD094_OFFSET)
#define IMX9_CAN_WORD194          (IMX9_CAN_BASE + IMX9_CAN_WORD194_OFFSET)
#define IMX9_CAN_CS95             (IMX9_CAN_BASE + IMX9_CAN_CS95_OFFSET)
#define IMX9_CAN_WORD095          (IMX9_CAN_BASE + IMX9_CAN_WORD095_OFFSET)
#define IMX9_CAN_WORD195          (IMX9_CAN_BASE + IMX9_CAN_WORD195_OFFSET)
#define IMX9_CAN_MECR             (IMX9_CAN_BASE + IMX9_CAN_MECR_OFFSET)
#define IMX9_CAN_ERRIAR           (IMX9_CAN_BASE + IMX9_CAN_ERRIAR_OFFSET)
#define IMX9_CAN_ERRIDPR          (IMX9_CAN_BASE + IMX9_CAN_ERRIDPR_OFFSET)
#define IMX9_CAN_ERRIPPR          (IMX9_CAN_BASE + IMX9_CAN_ERRIPPR_OFFSET)
#define IMX9_CAN_RERRAR           (IMX9_CAN_BASE + IMX9_CAN_RERRAR_OFFSET)
#define IMX9_CAN_RERRDR           (IMX9_CAN_BASE + IMX9_CAN_RERRDR_OFFSET)
#define IMX9_CAN_RERRSYNR         (IMX9_CAN_BASE + IMX9_CAN_RERRSYNR_OFFSET)
#define IMX9_CAN_ERRSR            (IMX9_CAN_BASE + IMX9_CAN_ERRSR_OFFSET)
#define IMX9_CAN_EPRS             (IMX9_CAN_BASE + IMX9_CAN_EPRS_OFFSET)
#define IMX9_CAN_ENCBT            (IMX9_CAN_BASE + IMX9_CAN_ENCBT_OFFSET)
#define IMX9_CAN_EDCBT            (IMX9_CAN_BASE + IMX9_CAN_EDCBT_OFFSET)
#define IMX9_CAN_ETDC             (IMX9_CAN_BASE + IMX9_CAN_ETDC_OFFSET)
#define IMX9_CAN_FDCTRL           (IMX9_CAN_BASE + IMX9_CAN_FDCTRL_OFFSET)
#define IMX9_CAN_FDCBT            (IMX9_CAN_BASE + IMX9_CAN_FDCBT_OFFSET)
#define IMX9_CAN_FDCRC            (IMX9_CAN_BASE + IMX9_CAN_FDCRC_OFFSET)
#define IMX9_CAN_ERFCR            (IMX9_CAN_BASE + IMX9_CAN_ERFCR_OFFSET)
#define IMX9_CAN_ERFIER           (IMX9_CAN_BASE + IMX9_CAN_ERFIER_OFFSET)
#define IMX9_CAN_ERFSR            (IMX9_CAN_BASE + IMX9_CAN_ERFSR_OFFSET)
#define IMX9_CAN_RXIMR0           (IMX9_CAN_BASE + IMX9_CAN_RXIMR0_OFFSET)
#define IMX9_CAN_RXIMR1           (IMX9_CAN_BASE + IMX9_CAN_RXIMR1_OFFSET)
#define IMX9_CAN_RXIMR2           (IMX9_CAN_BASE + IMX9_CAN_RXIMR2_OFFSET)
#define IMX9_CAN_RXIMR3           (IMX9_CAN_BASE + IMX9_CAN_RXIMR3_OFFSET)
#define IMX9_CAN_RXIMR4           (IMX9_CAN_BASE + IMX9_CAN_RXIMR4_OFFSET)
#define IMX9_CAN_RXIMR5           (IMX9_CAN_BASE + IMX9_CAN_RXIMR5_OFFSET)
#define IMX9_CAN_RXIMR6           (IMX9_CAN_BASE + IMX9_CAN_RXIMR6_OFFSET)
#define IMX9_CAN_RXIMR7           (IMX9_CAN_BASE + IMX9_CAN_RXIMR7_OFFSET)
#define IMX9_CAN_RXIMR8           (IMX9_CAN_BASE + IMX9_CAN_RXIMR8_OFFSET)
#define IMX9_CAN_RXIMR9           (IMX9_CAN_BASE + IMX9_CAN_RXIMR9_OFFSET)
#define IMX9_CAN_RXIMR10          (IMX9_CAN_BASE + IMX9_CAN_RXIMR10_OFFSET)
#define IMX9_CAN_RXIMR11          (IMX9_CAN_BASE + IMX9_CAN_RXIMR11_OFFSET)
#define IMX9_CAN_RXIMR12          (IMX9_CAN_BASE + IMX9_CAN_RXIMR12_OFFSET)
#define IMX9_CAN_RXIMR13          (IMX9_CAN_BASE + IMX9_CAN_RXIMR13_OFFSET)
#define IMX9_CAN_RXIMR14          (IMX9_CAN_BASE + IMX9_CAN_RXIMR14_OFFSET)
#define IMX9_CAN_RXIMR15          (IMX9_CAN_BASE + IMX9_CAN_RXIMR15_OFFSET)
#define IMX9_CAN_RXIMR16          (IMX9_CAN_BASE + IMX9_CAN_RXIMR16_OFFSET)
#define IMX9_CAN_RXIMR17          (IMX9_CAN_BASE + IMX9_CAN_RXIMR17_OFFSET)
#define IMX9_CAN_RXIMR18          (IMX9_CAN_BASE + IMX9_CAN_RXIMR18_OFFSET)
#define IMX9_CAN_RXIMR19          (IMX9_CAN_BASE + IMX9_CAN_RXIMR19_OFFSET)
#define IMX9_CAN_RXIMR20          (IMX9_CAN_BASE + IMX9_CAN_RXIMR20_OFFSET)
#define IMX9_CAN_RXIMR21          (IMX9_CAN_BASE + IMX9_CAN_RXIMR21_OFFSET)
#define IMX9_CAN_RXIMR22          (IMX9_CAN_BASE + IMX9_CAN_RXIMR22_OFFSET)
#define IMX9_CAN_RXIMR23          (IMX9_CAN_BASE + IMX9_CAN_RXIMR23_OFFSET)
#define IMX9_CAN_RXIMR24          (IMX9_CAN_BASE + IMX9_CAN_RXIMR24_OFFSET)
#define IMX9_CAN_RXIMR25          (IMX9_CAN_BASE + IMX9_CAN_RXIMR25_OFFSET)
#define IMX9_CAN_RXIMR26          (IMX9_CAN_BASE + IMX9_CAN_RXIMR26_OFFSET)
#define IMX9_CAN_RXIMR27          (IMX9_CAN_BASE + IMX9_CAN_RXIMR27_OFFSET)
#define IMX9_CAN_RXIMR28          (IMX9_CAN_BASE + IMX9_CAN_RXIMR28_OFFSET)
#define IMX9_CAN_RXIMR29          (IMX9_CAN_BASE + IMX9_CAN_RXIMR29_OFFSET)
#define IMX9_CAN_RXIMR30          (IMX9_CAN_BASE + IMX9_CAN_RXIMR30_OFFSET)
#define IMX9_CAN_RXIMR31          (IMX9_CAN_BASE + IMX9_CAN_RXIMR31_OFFSET)
#define IMX9_CAN_RXIMR32          (IMX9_CAN_BASE + IMX9_CAN_RXIMR32_OFFSET)
#define IMX9_CAN_RXIMR33          (IMX9_CAN_BASE + IMX9_CAN_RXIMR33_OFFSET)
#define IMX9_CAN_RXIMR34          (IMX9_CAN_BASE + IMX9_CAN_RXIMR34_OFFSET)
#define IMX9_CAN_RXIMR35          (IMX9_CAN_BASE + IMX9_CAN_RXIMR35_OFFSET)
#define IMX9_CAN_RXIMR36          (IMX9_CAN_BASE + IMX9_CAN_RXIMR36_OFFSET)
#define IMX9_CAN_RXIMR37          (IMX9_CAN_BASE + IMX9_CAN_RXIMR37_OFFSET)
#define IMX9_CAN_RXIMR38          (IMX9_CAN_BASE + IMX9_CAN_RXIMR38_OFFSET)
#define IMX9_CAN_RXIMR39          (IMX9_CAN_BASE + IMX9_CAN_RXIMR39_OFFSET)
#define IMX9_CAN_RXIMR40          (IMX9_CAN_BASE + IMX9_CAN_RXIMR40_OFFSET)
#define IMX9_CAN_RXIMR41          (IMX9_CAN_BASE + IMX9_CAN_RXIMR41_OFFSET)
#define IMX9_CAN_RXIMR42          (IMX9_CAN_BASE + IMX9_CAN_RXIMR42_OFFSET)
#define IMX9_CAN_RXIMR43          (IMX9_CAN_BASE + IMX9_CAN_RXIMR43_OFFSET)
#define IMX9_CAN_RXIMR44          (IMX9_CAN_BASE + IMX9_CAN_RXIMR44_OFFSET)
#define IMX9_CAN_RXIMR45          (IMX9_CAN_BASE + IMX9_CAN_RXIMR45_OFFSET)
#define IMX9_CAN_RXIMR46          (IMX9_CAN_BASE + IMX9_CAN_RXIMR46_OFFSET)
#define IMX9_CAN_RXIMR47          (IMX9_CAN_BASE + IMX9_CAN_RXIMR47_OFFSET)
#define IMX9_CAN_RXIMR48          (IMX9_CAN_BASE + IMX9_CAN_RXIMR48_OFFSET)
#define IMX9_CAN_RXIMR49          (IMX9_CAN_BASE + IMX9_CAN_RXIMR49_OFFSET)
#define IMX9_CAN_RXIMR50          (IMX9_CAN_BASE + IMX9_CAN_RXIMR50_OFFSET)
#define IMX9_CAN_RXIMR51          (IMX9_CAN_BASE + IMX9_CAN_RXIMR51_OFFSET)
#define IMX9_CAN_RXIMR52          (IMX9_CAN_BASE + IMX9_CAN_RXIMR52_OFFSET)
#define IMX9_CAN_RXIMR53          (IMX9_CAN_BASE + IMX9_CAN_RXIMR53_OFFSET)
#define IMX9_CAN_RXIMR54          (IMX9_CAN_BASE + IMX9_CAN_RXIMR54_OFFSET)
#define IMX9_CAN_RXIMR55          (IMX9_CAN_BASE + IMX9_CAN_RXIMR55_OFFSET)
#define IMX9_CAN_RXIMR56          (IMX9_CAN_BASE + IMX9_CAN_RXIMR56_OFFSET)
#define IMX9_CAN_RXIMR57          (IMX9_CAN_BASE + IMX9_CAN_RXIMR57_OFFSET)
#define IMX9_CAN_RXIMR58          (IMX9_CAN_BASE + IMX9_CAN_RXIMR58_OFFSET)
#define IMX9_CAN_RXIMR59          (IMX9_CAN_BASE + IMX9_CAN_RXIMR59_OFFSET)
#define IMX9_CAN_RXIMR60          (IMX9_CAN_BASE + IMX9_CAN_RXIMR60_OFFSET)
#define IMX9_CAN_RXIMR61          (IMX9_CAN_BASE + IMX9_CAN_RXIMR61_OFFSET)
#define IMX9_CAN_RXIMR62          (IMX9_CAN_BASE + IMX9_CAN_RXIMR62_OFFSET)
#define IMX9_CAN_RXIMR63          (IMX9_CAN_BASE + IMX9_CAN_RXIMR63_OFFSET)
#define IMX9_CAN_RXIMR64          (IMX9_CAN_BASE + IMX9_CAN_RXIMR64_OFFSET)
#define IMX9_CAN_RXIMR65          (IMX9_CAN_BASE + IMX9_CAN_RXIMR65_OFFSET)
#define IMX9_CAN_RXIMR66          (IMX9_CAN_BASE + IMX9_CAN_RXIMR66_OFFSET)
#define IMX9_CAN_RXIMR67          (IMX9_CAN_BASE + IMX9_CAN_RXIMR67_OFFSET)
#define IMX9_CAN_RXIMR68          (IMX9_CAN_BASE + IMX9_CAN_RXIMR68_OFFSET)
#define IMX9_CAN_RXIMR69          (IMX9_CAN_BASE + IMX9_CAN_RXIMR69_OFFSET)
#define IMX9_CAN_RXIMR70          (IMX9_CAN_BASE + IMX9_CAN_RXIMR70_OFFSET)
#define IMX9_CAN_RXIMR71          (IMX9_CAN_BASE + IMX9_CAN_RXIMR71_OFFSET)
#define IMX9_CAN_RXIMR72          (IMX9_CAN_BASE + IMX9_CAN_RXIMR72_OFFSET)
#define IMX9_CAN_RXIMR73          (IMX9_CAN_BASE + IMX9_CAN_RXIMR73_OFFSET)
#define IMX9_CAN_RXIMR74          (IMX9_CAN_BASE + IMX9_CAN_RXIMR74_OFFSET)
#define IMX9_CAN_RXIMR75          (IMX9_CAN_BASE + IMX9_CAN_RXIMR75_OFFSET)
#define IMX9_CAN_RXIMR76          (IMX9_CAN_BASE + IMX9_CAN_RXIMR76_OFFSET)
#define IMX9_CAN_RXIMR77          (IMX9_CAN_BASE + IMX9_CAN_RXIMR77_OFFSET)
#define IMX9_CAN_RXIMR78          (IMX9_CAN_BASE + IMX9_CAN_RXIMR78_OFFSET)
#define IMX9_CAN_RXIMR79          (IMX9_CAN_BASE + IMX9_CAN_RXIMR79_OFFSET)
#define IMX9_CAN_RXIMR80          (IMX9_CAN_BASE + IMX9_CAN_RXIMR80_OFFSET)
#define IMX9_CAN_RXIMR81          (IMX9_CAN_BASE + IMX9_CAN_RXIMR81_OFFSET)
#define IMX9_CAN_RXIMR82          (IMX9_CAN_BASE + IMX9_CAN_RXIMR82_OFFSET)
#define IMX9_CAN_RXIMR83          (IMX9_CAN_BASE + IMX9_CAN_RXIMR83_OFFSET)
#define IMX9_CAN_RXIMR84          (IMX9_CAN_BASE + IMX9_CAN_RXIMR84_OFFSET)
#define IMX9_CAN_RXIMR85          (IMX9_CAN_BASE + IMX9_CAN_RXIMR85_OFFSET)
#define IMX9_CAN_RXIMR86          (IMX9_CAN_BASE + IMX9_CAN_RXIMR86_OFFSET)
#define IMX9_CAN_RXIMR87          (IMX9_CAN_BASE + IMX9_CAN_RXIMR87_OFFSET)
#define IMX9_CAN_RXIMR88          (IMX9_CAN_BASE + IMX9_CAN_RXIMR88_OFFSET)
#define IMX9_CAN_RXIMR89          (IMX9_CAN_BASE + IMX9_CAN_RXIMR89_OFFSET)
#define IMX9_CAN_RXIMR90          (IMX9_CAN_BASE + IMX9_CAN_RXIMR90_OFFSET)
#define IMX9_CAN_RXIMR91          (IMX9_CAN_BASE + IMX9_CAN_RXIMR91_OFFSET)
#define IMX9_CAN_RXIMR92          (IMX9_CAN_BASE + IMX9_CAN_RXIMR92_OFFSET)
#define IMX9_CAN_RXIMR93          (IMX9_CAN_BASE + IMX9_CAN_RXIMR93_OFFSET)
#define IMX9_CAN_RXIMR94          (IMX9_CAN_BASE + IMX9_CAN_RXIMR94_OFFSET)
#define IMX9_CAN_RXIMR95          (IMX9_CAN_BASE + IMX9_CAN_RXIMR95_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP0   (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP0_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP1   (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP1_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP2   (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP2_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP3   (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP3_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP4   (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP4_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP5   (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP5_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP6   (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP6_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP7   (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP7_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP8   (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP8_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP9   (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP9_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP10  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP10_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP11  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP11_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP12  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP12_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP13  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP13_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP14  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP14_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP15  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP15_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP16  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP16_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP17  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP17_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP18  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP18_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP19  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP19_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP20  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP20_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP21  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP21_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP22  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP22_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP23  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP23_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP24  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP24_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP25  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP25_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP26  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP26_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP27  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP27_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP28  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP28_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP29  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP29_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP30  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP30_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP31  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP31_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP32  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP32_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP33  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP33_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP34  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP34_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP35  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP35_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP36  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP36_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP37  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP37_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP38  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP38_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP39  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP39_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP40  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP40_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP41  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP41_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP42  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP42_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP43  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP43_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP44  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP44_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP45  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP45_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP46  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP46_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP47  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP47_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP48  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP48_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP49  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP49_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP50  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP50_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP51  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP51_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP52  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP52_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP53  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP53_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP54  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP54_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP55  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP55_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP56  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP56_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP57  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP57_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP58  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP58_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP59  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP59_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP60  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP60_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP61  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP61_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP62  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP62_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP63  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP63_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP64  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP64_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP65  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP65_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP66  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP66_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP67  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP67_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP68  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP68_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP69  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP69_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP70  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP70_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP71  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP71_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP72  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP72_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP73  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP73_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP74  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP74_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP75  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP75_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP76  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP76_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP77  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP77_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP78  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP78_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP79  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP79_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP80  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP80_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP81  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP81_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP82  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP82_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP83  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP83_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP84  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP84_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP85  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP85_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP86  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP86_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP87  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP87_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP88  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP88_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP89  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP89_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP90  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP90_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP91  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP91_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP92  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP92_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP93  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP93_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP94  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP94_OFFSET)
#define IMX9_CAN_HR_TIME_STAMP95  (IMX9_CAN_BASE + IMX9_CAN_HR_TIME_STAMP95_OFFSET)
#define IMX9_CAN_ERFFEL0          (IMX9_CAN_BASE + IMX9_CAN_ERFFEL0_OFFSET)
#define IMX9_CAN_ERFFEL1          (IMX9_CAN_BASE + IMX9_CAN_ERFFEL1_OFFSET)
#define IMX9_CAN_ERFFEL2          (IMX9_CAN_BASE + IMX9_CAN_ERFFEL2_OFFSET)
#define IMX9_CAN_ERFFEL3          (IMX9_CAN_BASE + IMX9_CAN_ERFFEL3_OFFSET)
#define IMX9_CAN_ERFFEL4          (IMX9_CAN_BASE + IMX9_CAN_ERFFEL4_OFFSET)
#define IMX9_CAN_ERFFEL5          (IMX9_CAN_BASE + IMX9_CAN_ERFFEL5_OFFSET)
#define IMX9_CAN_ERFFEL6          (IMX9_CAN_BASE + IMX9_CAN_ERFFEL6_OFFSET)
#define IMX9_CAN_ERFFEL7          (IMX9_CAN_BASE + IMX9_CAN_ERFFEL7_OFFSET)
#define IMX9_CAN_ERFFEL8          (IMX9_CAN_BASE + IMX9_CAN_ERFFEL8_OFFSET)
#define IMX9_CAN_ERFFEL9          (IMX9_CAN_BASE + IMX9_CAN_ERFFEL9_OFFSET)
#define IMX9_CAN_ERFFEL10         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL10_OFFSET)
#define IMX9_CAN_ERFFEL11         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL11_OFFSET)
#define IMX9_CAN_ERFFEL12         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL12_OFFSET)
#define IMX9_CAN_ERFFEL13         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL13_OFFSET)
#define IMX9_CAN_ERFFEL14         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL14_OFFSET)
#define IMX9_CAN_ERFFEL15         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL15_OFFSET)
#define IMX9_CAN_ERFFEL16         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL16_OFFSET)
#define IMX9_CAN_ERFFEL17         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL17_OFFSET)
#define IMX9_CAN_ERFFEL18         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL18_OFFSET)
#define IMX9_CAN_ERFFEL19         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL19_OFFSET)
#define IMX9_CAN_ERFFEL20         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL20_OFFSET)
#define IMX9_CAN_ERFFEL21         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL21_OFFSET)
#define IMX9_CAN_ERFFEL22         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL22_OFFSET)
#define IMX9_CAN_ERFFEL23         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL23_OFFSET)
#define IMX9_CAN_ERFFEL24         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL24_OFFSET)
#define IMX9_CAN_ERFFEL25         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL25_OFFSET)
#define IMX9_CAN_ERFFEL26         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL26_OFFSET)
#define IMX9_CAN_ERFFEL27         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL27_OFFSET)
#define IMX9_CAN_ERFFEL28         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL28_OFFSET)
#define IMX9_CAN_ERFFEL29         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL29_OFFSET)
#define IMX9_CAN_ERFFEL30         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL30_OFFSET)
#define IMX9_CAN_ERFFEL31         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL31_OFFSET)
#define IMX9_CAN_ERFFEL32         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL32_OFFSET)
#define IMX9_CAN_ERFFEL33         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL33_OFFSET)
#define IMX9_CAN_ERFFEL34         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL34_OFFSET)
#define IMX9_CAN_ERFFEL35         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL35_OFFSET)
#define IMX9_CAN_ERFFEL36         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL36_OFFSET)
#define IMX9_CAN_ERFFEL37         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL37_OFFSET)
#define IMX9_CAN_ERFFEL38         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL38_OFFSET)
#define IMX9_CAN_ERFFEL39         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL39_OFFSET)
#define IMX9_CAN_ERFFEL40         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL40_OFFSET)
#define IMX9_CAN_ERFFEL41         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL41_OFFSET)
#define IMX9_CAN_ERFFEL42         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL42_OFFSET)
#define IMX9_CAN_ERFFEL43         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL43_OFFSET)
#define IMX9_CAN_ERFFEL44         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL44_OFFSET)
#define IMX9_CAN_ERFFEL45         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL45_OFFSET)
#define IMX9_CAN_ERFFEL46         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL46_OFFSET)
#define IMX9_CAN_ERFFEL47         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL47_OFFSET)
#define IMX9_CAN_ERFFEL48         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL48_OFFSET)
#define IMX9_CAN_ERFFEL49         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL49_OFFSET)
#define IMX9_CAN_ERFFEL50         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL50_OFFSET)
#define IMX9_CAN_ERFFEL51         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL51_OFFSET)
#define IMX9_CAN_ERFFEL52         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL52_OFFSET)
#define IMX9_CAN_ERFFEL53         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL53_OFFSET)
#define IMX9_CAN_ERFFEL54         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL54_OFFSET)
#define IMX9_CAN_ERFFEL55         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL55_OFFSET)
#define IMX9_CAN_ERFFEL56         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL56_OFFSET)
#define IMX9_CAN_ERFFEL57         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL57_OFFSET)
#define IMX9_CAN_ERFFEL58         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL58_OFFSET)
#define IMX9_CAN_ERFFEL59         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL59_OFFSET)
#define IMX9_CAN_ERFFEL60         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL60_OFFSET)
#define IMX9_CAN_ERFFEL61         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL61_OFFSET)
#define IMX9_CAN_ERFFEL62         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL62_OFFSET)
#define IMX9_CAN_ERFFEL63         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL63_OFFSET)
#define IMX9_CAN_ERFFEL64         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL64_OFFSET)
#define IMX9_CAN_ERFFEL65         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL65_OFFSET)
#define IMX9_CAN_ERFFEL66         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL66_OFFSET)
#define IMX9_CAN_ERFFEL67         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL67_OFFSET)
#define IMX9_CAN_ERFFEL68         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL68_OFFSET)
#define IMX9_CAN_ERFFEL69         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL69_OFFSET)
#define IMX9_CAN_ERFFEL70         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL70_OFFSET)
#define IMX9_CAN_ERFFEL71         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL71_OFFSET)
#define IMX9_CAN_ERFFEL72         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL72_OFFSET)
#define IMX9_CAN_ERFFEL73         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL73_OFFSET)
#define IMX9_CAN_ERFFEL74         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL74_OFFSET)
#define IMX9_CAN_ERFFEL75         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL75_OFFSET)
#define IMX9_CAN_ERFFEL76         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL76_OFFSET)
#define IMX9_CAN_ERFFEL77         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL77_OFFSET)
#define IMX9_CAN_ERFFEL78         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL78_OFFSET)
#define IMX9_CAN_ERFFEL79         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL79_OFFSET)
#define IMX9_CAN_ERFFEL80         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL80_OFFSET)
#define IMX9_CAN_ERFFEL81         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL81_OFFSET)
#define IMX9_CAN_ERFFEL82         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL82_OFFSET)
#define IMX9_CAN_ERFFEL83         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL83_OFFSET)
#define IMX9_CAN_ERFFEL84         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL84_OFFSET)
#define IMX9_CAN_ERFFEL85         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL85_OFFSET)
#define IMX9_CAN_ERFFEL86         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL86_OFFSET)
#define IMX9_CAN_ERFFEL87         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL87_OFFSET)
#define IMX9_CAN_ERFFEL88         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL88_OFFSET)
#define IMX9_CAN_ERFFEL89         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL89_OFFSET)
#define IMX9_CAN_ERFFEL90         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL90_OFFSET)
#define IMX9_CAN_ERFFEL91         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL91_OFFSET)
#define IMX9_CAN_ERFFEL92         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL92_OFFSET)
#define IMX9_CAN_ERFFEL93         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL93_OFFSET)
#define IMX9_CAN_ERFFEL94         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL94_OFFSET)
#define IMX9_CAN_ERFFEL95         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL95_OFFSET)
#define IMX9_CAN_ERFFEL96         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL96_OFFSET)
#define IMX9_CAN_ERFFEL97         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL97_OFFSET)
#define IMX9_CAN_ERFFEL98         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL98_OFFSET)
#define IMX9_CAN_ERFFEL99         (IMX9_CAN_BASE + IMX9_CAN_ERFFEL99_OFFSET)
#define IMX9_CAN_ERFFEL100        (IMX9_CAN_BASE + IMX9_CAN_ERFFEL100_OFFSET)
#define IMX9_CAN_ERFFEL101        (IMX9_CAN_BASE + IMX9_CAN_ERFFEL101_OFFSET)
#define IMX9_CAN_ERFFEL102        (IMX9_CAN_BASE + IMX9_CAN_ERFFEL102_OFFSET)
#define IMX9_CAN_ERFFEL103        (IMX9_CAN_BASE + IMX9_CAN_ERFFEL103_OFFSET)
#define IMX9_CAN_ERFFEL104        (IMX9_CAN_BASE + IMX9_CAN_ERFFEL104_OFFSET)
#define IMX9_CAN_ERFFEL105        (IMX9_CAN_BASE + IMX9_CAN_ERFFEL105_OFFSET)
#define IMX9_CAN_ERFFEL106        (IMX9_CAN_BASE + IMX9_CAN_ERFFEL106_OFFSET)
#define IMX9_CAN_ERFFEL107        (IMX9_CAN_BASE + IMX9_CAN_ERFFEL107_OFFSET)
#define IMX9_CAN_ERFFEL108        (IMX9_CAN_BASE + IMX9_CAN_ERFFEL108_OFFSET)
#define IMX9_CAN_ERFFEL109        (IMX9_CAN_BASE + IMX9_CAN_ERFFEL109_OFFSET)
#define IMX9_CAN_ERFFEL110        (IMX9_CAN_BASE + IMX9_CAN_ERFFEL110_OFFSET)
#define IMX9_CAN_ERFFEL111        (IMX9_CAN_BASE + IMX9_CAN_ERFFEL111_OFFSET)
#define IMX9_CAN_ERFFEL112        (IMX9_CAN_BASE + IMX9_CAN_ERFFEL112_OFFSET)
#define IMX9_CAN_ERFFEL113        (IMX9_CAN_BASE + IMX9_CAN_ERFFEL113_OFFSET)
#define IMX9_CAN_ERFFEL114        (IMX9_CAN_BASE + IMX9_CAN_ERFFEL114_OFFSET)
#define IMX9_CAN_ERFFEL115        (IMX9_CAN_BASE + IMX9_CAN_ERFFEL115_OFFSET)
#define IMX9_CAN_ERFFEL116        (IMX9_CAN_BASE + IMX9_CAN_ERFFEL116_OFFSET)
#define IMX9_CAN_ERFFEL117        (IMX9_CAN_BASE + IMX9_CAN_ERFFEL117_OFFSET)
#define IMX9_CAN_ERFFEL118        (IMX9_CAN_BASE + IMX9_CAN_ERFFEL118_OFFSET)
#define IMX9_CAN_ERFFEL119        (IMX9_CAN_BASE + IMX9_CAN_ERFFEL119_OFFSET)
#define IMX9_CAN_ERFFEL120        (IMX9_CAN_BASE + IMX9_CAN_ERFFEL120_OFFSET)
#define IMX9_CAN_ERFFEL121        (IMX9_CAN_BASE + IMX9_CAN_ERFFEL121_OFFSET)
#define IMX9_CAN_ERFFEL122        (IMX9_CAN_BASE + IMX9_CAN_ERFFEL122_OFFSET)
#define IMX9_CAN_ERFFEL123        (IMX9_CAN_BASE + IMX9_CAN_ERFFEL123_OFFSET)
#define IMX9_CAN_ERFFEL124        (IMX9_CAN_BASE + IMX9_CAN_ERFFEL124_OFFSET)
#define IMX9_CAN_ERFFEL125        (IMX9_CAN_BASE + IMX9_CAN_ERFFEL125_OFFSET)
#define IMX9_CAN_ERFFEL126        (IMX9_CAN_BASE + IMX9_CAN_ERFFEL126_OFFSET)
#define IMX9_CAN_ERFFEL127        (IMX9_CAN_BASE + IMX9_CAN_ERFFEL127_OFFSET)

/* Module Configuration (MCR) */
#define CAN_MCR_MAXMB_SHIFT  (0)        /* Bits 0-7: Number of the Last Message Buffer */
#define CAN_MCR_MAXMB_MASK   (0x7F << CAN_MCR_MAXMB_SHIFT)
#define CAN_MCR_MAXMB(n)     (((n) << CAN_MCR_MAXMB_SHIFT) & CAN_MCR_MAXMB_MASK)
#define CAN_MCR_IDAM_SHIFT   (8)        /* Bits 8-10: ID Acceptance Mode */
#define CAN_MCR_IDAM_MASK    (0x3 << CAN_MCR_IDAM_SHIFT)
#define CAN_MCR_IDAM(n)      (((n) << CAN_MCR_IDAM_SHIFT) & CAN_MCR_IDAM_MASK)
#define CAN_MCR_FDEN         (1 << 11)  /* Bit 11: CAN FD Operation Enable */
#define CAN_MCR_AEN          (1 << 12)  /* Bit 12: Abort Enable */
#define CAN_MCR_LPRIOEN      (1 << 13)  /* Bit 13: Local Priority Enable */
#define CAN_MCR_DMA          (1 << 15)  /* Bit 15: DMA Enable */
#define CAN_MCR_IRMQ         (1 << 16)  /* Bit 16: Individual RX Masking and Queue Enable */
#define CAN_MCR_SRXDIS       (1 << 17)  /* Bit 17: Self-Reception Disable */
#define CAN_MCR_DOZE         (1 << 18)  /* Bit 18: Doze Mode Enable */
#define CAN_MCR_WAKSRC       (1 << 19)  /* Bit 19: Wake-Up Source */
#define CAN_MCR_LPMACK       (1 << 20)  /* Bit 20: Low-Power Mode Acknowledge */
#define CAN_MCR_WRNEN        (1 << 21)  /* Bit 21: Warning Interrupt Enable */
#define CAN_MCR_SLFWAK       (1 << 22)  /* Bit 22: Self Wake-up */
#define CAN_MCR_SUPV         (1 << 23)  /* Bit 23: Supervisor Mode */
#define CAN_MCR_FRZACK       (1 << 24)  /* Bit 24: Freeze Mode Acknowledge */
#define CAN_MCR_SOFTRST      (1 << 25)  /* Bit 25: Soft Reset */
#define CAN_MCR_WAKMSK       (1 << 26)  /* Bit 26: Wake-up Interrupt Mask */
#define CAN_MCR_NOTRDY       (1 << 27)  /* Bit 27: FlexCAN Not Ready */
#define CAN_MCR_HALT         (1 << 28)  /* Bit 28: Halt FlexCAN */
#define CAN_MCR_RFEN         (1 << 29)  /* Bit 29: Legacy RX FIFO Enable */
#define CAN_MCR_FRZ          (1 << 30)  /* Bit 30: Freeze Enable */
#define CAN_MCR_MDIS         (1 << 31)  /* Bit 31: Module Disable */

/* Control 1 (CTRL1) */
#define CAN_CTRL1_PROPSEG_SHIFT  (0)        /* Bits 0-3: Propagation Segment */
#define CAN_CTRL1_PROPSEG_MASK   (0x7 << CAN_CTRL1_PROPSEG_SHIFT)
#define CAN_CTRL1_PROPSEG(n)     (((n) << CAN_CTRL1_PROPSEG_SHIFT) & CAN_CTRL1_PROPSEG_MASK)
#define CAN_CTRL1_LOM            (1 << 3)   /* Bit 3: Listen-Only Mode */
#define CAN_CTRL1_LBUF           (1 << 4)   /* Bit 4: Lowest Buffer Transmitted First */
#define CAN_CTRL1_TSYN           (1 << 5)   /* Bit 5: Timer Sync */
#define CAN_CTRL1_BOFFREC        (1 << 6)   /* Bit 6: Bus Off Recovery */
#define CAN_CTRL1_SMP            (1 << 7)   /* Bit 7: CAN Bit Sampling */
#define CAN_CTRL1_RWRNMSK        (1 << 10)  /* Bit 10: RX Warning Interrupt Mask */
#define CAN_CTRL1_TWRNMSK        (1 << 11)  /* Bit 11: TX Warning Interrupt Mask */
#define CAN_CTRL1_LPB            (1 << 12)  /* Bit 12: Loopback Mode */
#define CAN_CTRL1_CLKSRC         (1 << 13)  /* Bit 13: CAN Engine Clock Source */
#define CAN_CTRL1_ERRMSK         (1 << 14)  /* Bit 14: Error Interrupt Mask */
#define CAN_CTRL1_BOFFMSK        (1 << 15)  /* Bit 15: Bus Off Interrupt Mask */
#define CAN_CTRL1_PSEG2_SHIFT    (16)       /* Bits 16-19: Phase Segment 2 */
#define CAN_CTRL1_PSEG2_MASK     (0x7 << CAN_CTRL1_PSEG2_SHIFT)
#define CAN_CTRL1_PSEG2(n)       (((n) << CAN_CTRL1_PSEG2_SHIFT) & CAN_CTRL1_PSEG2_MASK)
#define CAN_CTRL1_PSEG1_SHIFT    (19)       /* Bits 19-22: Phase Segment 1 */
#define CAN_CTRL1_PSEG1_MASK     (0x7 << CAN_CTRL1_PSEG1_SHIFT)
#define CAN_CTRL1_PSEG1(n)       (((n) << CAN_CTRL1_PSEG1_SHIFT) & CAN_CTRL1_PSEG1_MASK)
#define CAN_CTRL1_RJW_SHIFT      (22)       /* Bits 22-24: Resync Jump Width */
#define CAN_CTRL1_RJW_MASK       (0x3 << CAN_CTRL1_RJW_SHIFT)
#define CAN_CTRL1_RJW(n)         (((n) << CAN_CTRL1_RJW_SHIFT) & CAN_CTRL1_RJW_MASK)
#define CAN_CTRL1_PRESDIV_SHIFT  (24)       /* Bits 24-32: Prescaler Division Factor */
#define CAN_CTRL1_PRESDIV_MASK   (0xFF << CAN_CTRL1_PRESDIV_SHIFT)
#define CAN_CTRL1_PRESDIV(n)     (((n) << CAN_CTRL1_PRESDIV_SHIFT) & CAN_CTRL1_PRESDIV_MASK)

/* Free-Running Timer (TIMER) */
#define CAN_TIMER_TIMER_SHIFT  (0)  /* Bits 0-16: Timer Value */
#define CAN_TIMER_TIMER_MASK   (0xFFFF << CAN_TIMER_TIMER_SHIFT)
#define CAN_TIMER_TIMER(n)     (((n) << CAN_TIMER_TIMER_SHIFT) & CAN_TIMER_TIMER_MASK)

/* RX Message Buffers Global Mask (RXMGMASK) */
#define CAN_RXMGMASK_MG_SHIFT  (0)  /* Bits 0-32: Global Mask for RX Message Buffers */
#define CAN_RXMGMASK_MG_MASK   (0xFFFFFFFF << CAN_RXMGMASK_MG_SHIFT)
#define CAN_RXMGMASK_MG(n)     (((n) << CAN_RXMGMASK_MG_SHIFT) & CAN_RXMGMASK_MG_MASK)

/* Receive 14 Mask (RX14MASK) */
#define CAN_RX14MASK_RX14M_SHIFT  (0)  /* Bits 0-32: RX Buffer 14 Mask Bits */
#define CAN_RX14MASK_RX14M_MASK   (0xFFFFFFFF << CAN_RX14MASK_RX14M_SHIFT)
#define CAN_RX14MASK_RX14M(n)     (((n) << CAN_RX14MASK_RX14M_SHIFT) & CAN_RX14MASK_RX14M_MASK)

/* Receive 15 Mask (RX15MASK) */
#define CAN_RX15MASK_RX15M_SHIFT  (0)  /* Bits 0-32: RX Buffer 15 Mask Bits */
#define CAN_RX15MASK_RX15M_MASK   (0xFFFFFFFF << CAN_RX15MASK_RX15M_SHIFT)
#define CAN_RX15MASK_RX15M(n)     (((n) << CAN_RX15MASK_RX15M_SHIFT) & CAN_RX15MASK_RX15M_MASK)

/* Error Counter (ECR) */
#define CAN_ECR_TXERRCNT_SHIFT       (0)   /* Bits 0-8: Transmit Error Counter */
#define CAN_ECR_TXERRCNT_MASK        (0xFF << CAN_ECR_TXERRCNT_SHIFT)
#define CAN_ECR_TXERRCNT(n)          (((n) << CAN_ECR_TXERRCNT_SHIFT) & CAN_ECR_TXERRCNT_MASK)
#define CAN_ECR_RXERRCNT_SHIFT       (8)   /* Bits 8-16: Receive Error Counter */
#define CAN_ECR_RXERRCNT_MASK        (0xFF << CAN_ECR_RXERRCNT_SHIFT)
#define CAN_ECR_RXERRCNT(n)          (((n) << CAN_ECR_RXERRCNT_SHIFT) & CAN_ECR_RXERRCNT_MASK)
#define CAN_ECR_TXERRCNT_FAST_SHIFT  (16)  /* Bits 16-24: Transmit Error Counter for Fast Bits */
#define CAN_ECR_TXERRCNT_FAST_MASK   (0xFF << CAN_ECR_TXERRCNT_FAST_SHIFT)
#define CAN_ECR_TXERRCNT_FAST(n)     (((n) << CAN_ECR_TXERRCNT_FAST_SHIFT) & CAN_ECR_TXERRCNT_FAST_MASK)
#define CAN_ECR_RXERRCNT_FAST_SHIFT  (24)  /* Bits 24-32: Receive Error Counter for Fast Bits */
#define CAN_ECR_RXERRCNT_FAST_MASK   (0xFF << CAN_ECR_RXERRCNT_FAST_SHIFT)
#define CAN_ECR_RXERRCNT_FAST(n)     (((n) << CAN_ECR_RXERRCNT_FAST_SHIFT) & CAN_ECR_RXERRCNT_FAST_MASK)

/* Error and Status 1 (ESR1) */
#define CAN_ESR1_WAKINT         (1 << 0)   /* Bit 0: Wake-up Interrupt Flag */
#define CAN_ESR1_ERRINT         (1 << 1)   /* Bit 1: Error Interrupt Flag */
#define CAN_ESR1_BOFFINT        (1 << 2)   /* Bit 2: Bus Off Interrupt Flag */
#define CAN_ESR1_RX             (1 << 3)   /* Bit 3: FlexCAN in Reception Flag */
#define CAN_ESR1_FLTCONF_SHIFT  (4)        /* Bits 4-6: Fault Confinement State */
#define CAN_ESR1_FLTCONF_MASK   (0x3 << CAN_ESR1_FLTCONF_SHIFT)
#define CAN_ESR1_FLTCONF(n)     (((n) << CAN_ESR1_FLTCONF_SHIFT) & CAN_ESR1_FLTCONF_MASK)
#define CAN_ESR1_TX             (1 << 6)   /* Bit 6: FlexCAN In Transmission */
#define CAN_ESR1_IDLE           (1 << 7)   /* Bit 7: Idle */
#define CAN_ESR1_RXWRN          (1 << 8)   /* Bit 8: RX Error Warning Flag */
#define CAN_ESR1_TXWRN          (1 << 9)   /* Bit 9: TX Error Warning Flag */
#define CAN_ESR1_STFERR         (1 << 10)  /* Bit 10: Stuffing Error Flag */
#define CAN_ESR1_FRMERR         (1 << 11)  /* Bit 11: Form Error Flag */
#define CAN_ESR1_CRCERR         (1 << 12)  /* Bit 12: Cyclic Redundancy Check Error Flag */
#define CAN_ESR1_ACKERR         (1 << 13)  /* Bit 13: Acknowledge Error Flag */
#define CAN_ESR1_BIT0ERR        (1 << 14)  /* Bit 14: Bit0 Error Flag */
#define CAN_ESR1_BIT1ERR        (1 << 15)  /* Bit 15: Bit1 Error Flag */
#define CAN_ESR1_RWRNINT        (1 << 16)  /* Bit 16: RX Warning Interrupt Flag */
#define CAN_ESR1_TWRNINT        (1 << 17)  /* Bit 17: TX Warning Interrupt Flag */
#define CAN_ESR1_SYNCH          (1 << 18)  /* Bit 18: CAN Synchronization Status Flag */
#define CAN_ESR1_BOFFDONEINT    (1 << 19)  /* Bit 19: Bus Off Done Interrupt Flag */
#define CAN_ESR1_ERRINT_FAST    (1 << 20)  /* Bit 20: Fast Error Interrupt Flag */
#define CAN_ESR1_ERROVR         (1 << 21)  /* Bit 21: Error Overrun Flag */
#define CAN_ESR1_STFERR_FAST    (1 << 26)  /* Bit 26: Fast Stuffing Error Flag */
#define CAN_ESR1_FRMERR_FAST    (1 << 27)  /* Bit 27: Fast Form Error Flag */
#define CAN_ESR1_CRCERR_FAST    (1 << 28)  /* Bit 28: Fast Cyclic Redundancy Check Error Flag */
#define CAN_ESR1_BIT0ERR_FAST   (1 << 30)  /* Bit 30: Fast Bit0 Error Flag */
#define CAN_ESR1_BIT1ERR_FAST   (1 << 31)  /* Bit 31: Fast Bit1 Error Flag */

/* Interrupt Masks 2 (IMASK2) */
#define CAN_IMASK2_BUF63TO32M_SHIFT  (0)  /* Bits 0-32: Buffer MBi Mask */
#define CAN_IMASK2_BUF63TO32M_MASK   (0xFFFFFFFF << CAN_IMASK2_BUF63TO32M_SHIFT)
#define CAN_IMASK2_BUF63TO32M(n)     (((n) << CAN_IMASK2_BUF63TO32M_SHIFT) & CAN_IMASK2_BUF63TO32M_MASK)

/* Interrupt Masks 1 (IMASK1) */
#define CAN_IMASK1_BUF31TO0M_SHIFT  (0)  /* Bits 0-32: Buffer MBi Mask */
#define CAN_IMASK1_BUF31TO0M_MASK   (0xFFFFFFFF << CAN_IMASK1_BUF31TO0M_SHIFT)
#define CAN_IMASK1_BUF31TO0M(n)     (((n) << CAN_IMASK1_BUF31TO0M_SHIFT) & CAN_IMASK1_BUF31TO0M_MASK)

/* Interrupt Flags 2 (IFLAG2) */
#define CAN_IFLAG2_BUF63TO32I_SHIFT  (0)  /* Bits 0-32: Buffer MBi Interrupt */
#define CAN_IFLAG2_BUF63TO32I_MASK   (0xFFFFFFFF << CAN_IFLAG2_BUF63TO32I_SHIFT)
#define CAN_IFLAG2_BUF63TO32I(n)     (((n) << CAN_IFLAG2_BUF63TO32I_SHIFT) & CAN_IFLAG2_BUF63TO32I_MASK)

/* Interrupt Flags 1 (IFLAG1) */
#define CAN_IFLAG1_BUF0I            (1 << 0)  /* Bit 0: Buffer MB0 Interrupt or Clear Legacy FIFO bit */
#define CAN_IFLAG1_BUF4TO1I_SHIFT   (1)       /* Bits 1-5: Buffer MBi Interrupt or Reserved */
#define CAN_IFLAG1_BUF4TO1I_MASK    (0xF << CAN_IFLAG1_BUF4TO1I_SHIFT)
#define CAN_IFLAG1_BUF4TO1I(n)      (((n) << CAN_IFLAG1_BUF4TO1I_SHIFT) & CAN_IFLAG1_BUF4TO1I_MASK)
#define CAN_IFLAG1_BUF5I            (1 << 5)  /* Bit 5: Buffer MB5 Interrupt or Frames available in Legacy RX FIFO */
#define CAN_IFLAG1_BUF6I            (1 << 6)  /* Bit 6: Buffer MB6 Interrupt or Legacy RX FIFO Warning */
#define CAN_IFLAG1_BUF7I            (1 << 7)  /* Bit 7: Buffer MB7 Interrupt or Legacy RX FIFO Overflow */
#define CAN_IFLAG1_BUF31TO8I_SHIFT  (8)       /* Bits 8-32: Buffer MBi Interrupt */
#define CAN_IFLAG1_BUF31TO8I_MASK   (0xFFFFFF << CAN_IFLAG1_BUF31TO8I_SHIFT)
#define CAN_IFLAG1_BUF31TO8I(n)     (((n) << CAN_IFLAG1_BUF31TO8I_SHIFT) & CAN_IFLAG1_BUF31TO8I_MASK)

/* Control 2 (CTRL2) */
#define CAN_CTRL2_TSTAMPCAP_SHIFT  (6)        /* Bits 6-8: Timestamp Capture Point */
#define CAN_CTRL2_TSTAMPCAP_MASK   (0x3 << CAN_CTRL2_TSTAMPCAP_SHIFT)
#define CAN_CTRL2_TSTAMPCAP(n)     (((n) << CAN_CTRL2_TSTAMPCAP_SHIFT) & CAN_CTRL2_TSTAMPCAP_MASK)
#define CAN_CTRL2_MBTSBASE_SHIFT   (8)        /* Bits 8-10: Message Buffer Timestamp Base */
#define CAN_CTRL2_MBTSBASE_MASK    (0x3 << CAN_CTRL2_MBTSBASE_SHIFT)
#define CAN_CTRL2_MBTSBASE(n)      (((n) << CAN_CTRL2_MBTSBASE_SHIFT) & CAN_CTRL2_MBTSBASE_MASK)
#define CAN_CTRL2_EDFLTDIS         (1 << 11)  /* Bit 11: Edge Filter Disable */
#define CAN_CTRL2_ISOCANFDEN       (1 << 12)  /* Bit 12: ISO CAN FD Enable */
#define CAN_CTRL2_BTE              (1 << 13)  /* Bit 13: Bit Timing Expansion Enable */
#define CAN_CTRL2_PREXCEN          (1 << 14)  /* Bit 14: Protocol Exception Enable */
#define CAN_CTRL2_TIMER_SRC        (1 << 15)  /* Bit 15: Timer Source */
#define CAN_CTRL2_EACEN            (1 << 16)  /* Bit 16: Entire Frame Arbitration Field Comparison Enable for RX Message Buffers */
#define CAN_CTRL2_RRS              (1 << 17)  /* Bit 17: Remote Request Storing */
#define CAN_CTRL2_MRP              (1 << 18)  /* Bit 18: Message Buffers Reception Priority */
#define CAN_CTRL2_TASD_SHIFT       (19)       /* Bits 19-24: Transmission Arbitration Start Delay */
#define CAN_CTRL2_TASD_MASK        (0x1F << CAN_CTRL2_TASD_SHIFT)
#define CAN_CTRL2_TASD(n)          (((n) << CAN_CTRL2_TASD_SHIFT) & CAN_CTRL2_TASD_MASK)
#define CAN_CTRL2_RFFN_SHIFT       (24)       /* Bits 24-28: Number of Legacy Receive FIFO Filters */
#define CAN_CTRL2_RFFN_MASK        (0xF << CAN_CTRL2_RFFN_SHIFT)
#define CAN_CTRL2_RFFN(n)          (((n) << CAN_CTRL2_RFFN_SHIFT) & CAN_CTRL2_RFFN_MASK)
#define CAN_CTRL2_WRMFRZ           (1 << 28)  /* Bit 28: Write Access to Memory in Freeze Mode */
#define CAN_CTRL2_ECRWRE           (1 << 29)  /* Bit 29: Error Correction Configuration Register Write Enable */
#define CAN_CTRL2_BOFFDONEMSK      (1 << 30)  /* Bit 30: Bus Off Done Interrupt Mask */
#define CAN_CTRL2_ERRMSK_FAST      (1 << 31)  /* Bit 31: Error Interrupt Mask for Errors Detected in the Data Phase of Fast CAN FD Frames */

/* Error and Status 2 (ESR2) */
#define CAN_ESR2_IMB         (1 << 13)  /* Bit 13: Inactive Message Buffer */
#define CAN_ESR2_VPS         (1 << 14)  /* Bit 14: Valid Priority Status */
#define CAN_ESR2_LPTM_SHIFT  (16)       /* Bits 16-23: Lowest Priority TX Message Buffer */
#define CAN_ESR2_LPTM_MASK   (0x7F << CAN_ESR2_LPTM_SHIFT)
#define CAN_ESR2_LPTM(n)     (((n) << CAN_ESR2_LPTM_SHIFT) & CAN_ESR2_LPTM_MASK)

/* Cyclic Redundancy Check (CRCR) */
#define CAN_CRCR_TXCRC_SHIFT  (0)   /* Bits 0-15: Transmitted CRC value */
#define CAN_CRCR_TXCRC_MASK   (0x7FFF << CAN_CRCR_TXCRC_SHIFT)
#define CAN_CRCR_TXCRC(n)     (((n) << CAN_CRCR_TXCRC_SHIFT) & CAN_CRCR_TXCRC_MASK)
#define CAN_CRCR_MBCRC_SHIFT  (16)  /* Bits 16-23: CRC Message Buffer */
#define CAN_CRCR_MBCRC_MASK   (0x7F << CAN_CRCR_MBCRC_SHIFT)
#define CAN_CRCR_MBCRC(n)     (((n) << CAN_CRCR_MBCRC_SHIFT) & CAN_CRCR_MBCRC_MASK)

/* Legacy RX FIFO Global Mask (RXFGMASK) */
#define CAN_RXFGMASK_FGM_SHIFT  (0)  /* Bits 0-32: Legacy RX FIFO Global Mask Bits */
#define CAN_RXFGMASK_FGM_MASK   (0xFFFFFFFF << CAN_RXFGMASK_FGM_SHIFT)
#define CAN_RXFGMASK_FGM(n)     (((n) << CAN_RXFGMASK_FGM_SHIFT) & CAN_RXFGMASK_FGM_MASK)

/* Legacy RX FIFO Information (RXFIR) */
#define CAN_RXFIR_IDHIT_SHIFT  (0)  /* Bits 0-9: Identifier Acceptance Filter Hit Indicator */
#define CAN_RXFIR_IDHIT_MASK   (0x1FF << CAN_RXFIR_IDHIT_SHIFT)
#define CAN_RXFIR_IDHIT(n)     (((n) << CAN_RXFIR_IDHIT_SHIFT) & CAN_RXFIR_IDHIT_MASK)

/* CAN Bit Timing (CBT) */
#define CAN_CBT_EPSEG2_SHIFT    (0)        /* Bits 0-5: Extended Phase Segment 2 */
#define CAN_CBT_EPSEG2_MASK     (0x1F << CAN_CBT_EPSEG2_SHIFT)
#define CAN_CBT_EPSEG2(n)       (((n) << CAN_CBT_EPSEG2_SHIFT) & CAN_CBT_EPSEG2_MASK)
#define CAN_CBT_EPSEG1_SHIFT    (5)        /* Bits 5-10: Extended Phase Segment 1 */
#define CAN_CBT_EPSEG1_MASK     (0x1F << CAN_CBT_EPSEG1_SHIFT)
#define CAN_CBT_EPSEG1(n)       (((n) << CAN_CBT_EPSEG1_SHIFT) & CAN_CBT_EPSEG1_MASK)
#define CAN_CBT_EPROPSEG_SHIFT  (10)       /* Bits 10-16: Extended Propagation Segment */
#define CAN_CBT_EPROPSEG_MASK   (0x3F << CAN_CBT_EPROPSEG_SHIFT)
#define CAN_CBT_EPROPSEG(n)     (((n) << CAN_CBT_EPROPSEG_SHIFT) & CAN_CBT_EPROPSEG_MASK)
#define CAN_CBT_ERJW_SHIFT      (16)       /* Bits 16-21: Extended Resync Jump Width */
#define CAN_CBT_ERJW_MASK       (0x1F << CAN_CBT_ERJW_SHIFT)
#define CAN_CBT_ERJW(n)         (((n) << CAN_CBT_ERJW_SHIFT) & CAN_CBT_ERJW_MASK)
#define CAN_CBT_EPRESDIV_SHIFT  (21)       /* Bits 21-31: Extended Prescaler Division Factor */
#define CAN_CBT_EPRESDIV_MASK   (0x3FF << CAN_CBT_EPRESDIV_SHIFT)
#define CAN_CBT_EPRESDIV(n)     (((n) << CAN_CBT_EPRESDIV_SHIFT) & CAN_CBT_EPRESDIV_MASK)
#define CAN_CBT_BTF             (1 << 31)  /* Bit 31: Bit Timing Format Enable */

/* Interrupt Masks 3 (IMASK3) */
#define CAN_IMASK3_BUF95TO64M_SHIFT  (0)  /* Bits 0-32: Buffer MBi Mask */
#define CAN_IMASK3_BUF95TO64M_MASK   (0xFFFFFFFF << CAN_IMASK3_BUF95TO64M_SHIFT)
#define CAN_IMASK3_BUF95TO64M(n)     (((n) << CAN_IMASK3_BUF95TO64M_SHIFT) & CAN_IMASK3_BUF95TO64M_MASK)

/* Interrupt Flags 3 (IFLAG3) */
#define CAN_IFLAG3_BUF95TO64_SHIFT  (0)  /* Bits 0-32: Buffer MBi Interrupt */
#define CAN_IFLAG3_BUF95TO64_MASK   (0xFFFFFFFF << CAN_IFLAG3_BUF95TO64_SHIFT)
#define CAN_IFLAG3_BUF95TO64(n)     (((n) << CAN_IFLAG3_BUF95TO64_SHIFT) & CAN_IFLAG3_BUF95TO64_MASK)

/* Message Buffer 0 CS Register (CS0) */
#define CAN_CS0_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS0_TIME_STAMP_MASK   (0xFFFF << CAN_CS0_TIME_STAMP_SHIFT)
#define CAN_CS0_TIME_STAMP(n)     (((n) << CAN_CS0_TIME_STAMP_SHIFT) & CAN_CS0_TIME_STAMP_MASK)
#define CAN_CS0_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS0_DLC_MASK          (0xF << CAN_CS0_DLC_SHIFT)
#define CAN_CS0_DLC(n)            (((n) << CAN_CS0_DLC_SHIFT) & CAN_CS0_DLC_MASK)
#define CAN_CS0_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS0_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS0_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS0_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS0_CODE_MASK         (0xF << CAN_CS0_CODE_SHIFT)
#define CAN_CS0_CODE(n)           (((n) << CAN_CS0_CODE_SHIFT) & CAN_CS0_CODE_MASK)
#define CAN_CS0_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS0_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS0_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 0 WORD0 Register (WORD00) */
#define CAN_WORD00_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD00_DATA_BYTE_3_MASK   (0xFF << CAN_WORD00_DATA_BYTE_3_SHIFT)
#define CAN_WORD00_DATA_BYTE_3(n)     (((n) << CAN_WORD00_DATA_BYTE_3_SHIFT) & CAN_WORD00_DATA_BYTE_3_MASK)
#define CAN_WORD00_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD00_DATA_BYTE_2_MASK   (0xFF << CAN_WORD00_DATA_BYTE_2_SHIFT)
#define CAN_WORD00_DATA_BYTE_2(n)     (((n) << CAN_WORD00_DATA_BYTE_2_SHIFT) & CAN_WORD00_DATA_BYTE_2_MASK)
#define CAN_WORD00_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD00_DATA_BYTE_1_MASK   (0xFF << CAN_WORD00_DATA_BYTE_1_SHIFT)
#define CAN_WORD00_DATA_BYTE_1(n)     (((n) << CAN_WORD00_DATA_BYTE_1_SHIFT) & CAN_WORD00_DATA_BYTE_1_MASK)
#define CAN_WORD00_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD00_DATA_BYTE_0_MASK   (0xFF << CAN_WORD00_DATA_BYTE_0_SHIFT)
#define CAN_WORD00_DATA_BYTE_0(n)     (((n) << CAN_WORD00_DATA_BYTE_0_SHIFT) & CAN_WORD00_DATA_BYTE_0_MASK)

/* Message Buffer 0 WORD1 Register (WORD10) */
#define CAN_WORD10_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD10_DATA_BYTE_7_MASK   (0xFF << CAN_WORD10_DATA_BYTE_7_SHIFT)
#define CAN_WORD10_DATA_BYTE_7(n)     (((n) << CAN_WORD10_DATA_BYTE_7_SHIFT) & CAN_WORD10_DATA_BYTE_7_MASK)
#define CAN_WORD10_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD10_DATA_BYTE_6_MASK   (0xFF << CAN_WORD10_DATA_BYTE_6_SHIFT)
#define CAN_WORD10_DATA_BYTE_6(n)     (((n) << CAN_WORD10_DATA_BYTE_6_SHIFT) & CAN_WORD10_DATA_BYTE_6_MASK)
#define CAN_WORD10_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD10_DATA_BYTE_5_MASK   (0xFF << CAN_WORD10_DATA_BYTE_5_SHIFT)
#define CAN_WORD10_DATA_BYTE_5(n)     (((n) << CAN_WORD10_DATA_BYTE_5_SHIFT) & CAN_WORD10_DATA_BYTE_5_MASK)
#define CAN_WORD10_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD10_DATA_BYTE_4_MASK   (0xFF << CAN_WORD10_DATA_BYTE_4_SHIFT)
#define CAN_WORD10_DATA_BYTE_4(n)     (((n) << CAN_WORD10_DATA_BYTE_4_SHIFT) & CAN_WORD10_DATA_BYTE_4_MASK)

/* Message Buffer 1 CS Register (CS1) */
#define CAN_CS1_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS1_TIME_STAMP_MASK   (0xFFFF << CAN_CS1_TIME_STAMP_SHIFT)
#define CAN_CS1_TIME_STAMP(n)     (((n) << CAN_CS1_TIME_STAMP_SHIFT) & CAN_CS1_TIME_STAMP_MASK)
#define CAN_CS1_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS1_DLC_MASK          (0xF << CAN_CS1_DLC_SHIFT)
#define CAN_CS1_DLC(n)            (((n) << CAN_CS1_DLC_SHIFT) & CAN_CS1_DLC_MASK)
#define CAN_CS1_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS1_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS1_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS1_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS1_CODE_MASK         (0xF << CAN_CS1_CODE_SHIFT)
#define CAN_CS1_CODE(n)           (((n) << CAN_CS1_CODE_SHIFT) & CAN_CS1_CODE_MASK)
#define CAN_CS1_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS1_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS1_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 1 WORD0 Register (WORD01) */
#define CAN_WORD01_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD01_DATA_BYTE_3_MASK   (0xFF << CAN_WORD01_DATA_BYTE_3_SHIFT)
#define CAN_WORD01_DATA_BYTE_3(n)     (((n) << CAN_WORD01_DATA_BYTE_3_SHIFT) & CAN_WORD01_DATA_BYTE_3_MASK)
#define CAN_WORD01_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD01_DATA_BYTE_2_MASK   (0xFF << CAN_WORD01_DATA_BYTE_2_SHIFT)
#define CAN_WORD01_DATA_BYTE_2(n)     (((n) << CAN_WORD01_DATA_BYTE_2_SHIFT) & CAN_WORD01_DATA_BYTE_2_MASK)
#define CAN_WORD01_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD01_DATA_BYTE_1_MASK   (0xFF << CAN_WORD01_DATA_BYTE_1_SHIFT)
#define CAN_WORD01_DATA_BYTE_1(n)     (((n) << CAN_WORD01_DATA_BYTE_1_SHIFT) & CAN_WORD01_DATA_BYTE_1_MASK)
#define CAN_WORD01_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD01_DATA_BYTE_0_MASK   (0xFF << CAN_WORD01_DATA_BYTE_0_SHIFT)
#define CAN_WORD01_DATA_BYTE_0(n)     (((n) << CAN_WORD01_DATA_BYTE_0_SHIFT) & CAN_WORD01_DATA_BYTE_0_MASK)

/* Message Buffer 1 WORD1 Register (WORD11) */
#define CAN_WORD11_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD11_DATA_BYTE_7_MASK   (0xFF << CAN_WORD11_DATA_BYTE_7_SHIFT)
#define CAN_WORD11_DATA_BYTE_7(n)     (((n) << CAN_WORD11_DATA_BYTE_7_SHIFT) & CAN_WORD11_DATA_BYTE_7_MASK)
#define CAN_WORD11_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD11_DATA_BYTE_6_MASK   (0xFF << CAN_WORD11_DATA_BYTE_6_SHIFT)
#define CAN_WORD11_DATA_BYTE_6(n)     (((n) << CAN_WORD11_DATA_BYTE_6_SHIFT) & CAN_WORD11_DATA_BYTE_6_MASK)
#define CAN_WORD11_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD11_DATA_BYTE_5_MASK   (0xFF << CAN_WORD11_DATA_BYTE_5_SHIFT)
#define CAN_WORD11_DATA_BYTE_5(n)     (((n) << CAN_WORD11_DATA_BYTE_5_SHIFT) & CAN_WORD11_DATA_BYTE_5_MASK)
#define CAN_WORD11_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD11_DATA_BYTE_4_MASK   (0xFF << CAN_WORD11_DATA_BYTE_4_SHIFT)
#define CAN_WORD11_DATA_BYTE_4(n)     (((n) << CAN_WORD11_DATA_BYTE_4_SHIFT) & CAN_WORD11_DATA_BYTE_4_MASK)

/* Message Buffer 2 CS Register (CS2) */
#define CAN_CS2_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS2_TIME_STAMP_MASK   (0xFFFF << CAN_CS2_TIME_STAMP_SHIFT)
#define CAN_CS2_TIME_STAMP(n)     (((n) << CAN_CS2_TIME_STAMP_SHIFT) & CAN_CS2_TIME_STAMP_MASK)
#define CAN_CS2_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS2_DLC_MASK          (0xF << CAN_CS2_DLC_SHIFT)
#define CAN_CS2_DLC(n)            (((n) << CAN_CS2_DLC_SHIFT) & CAN_CS2_DLC_MASK)
#define CAN_CS2_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS2_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS2_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS2_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS2_CODE_MASK         (0xF << CAN_CS2_CODE_SHIFT)
#define CAN_CS2_CODE(n)           (((n) << CAN_CS2_CODE_SHIFT) & CAN_CS2_CODE_MASK)
#define CAN_CS2_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS2_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS2_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 2 WORD0 Register (WORD02) */
#define CAN_WORD02_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD02_DATA_BYTE_3_MASK   (0xFF << CAN_WORD02_DATA_BYTE_3_SHIFT)
#define CAN_WORD02_DATA_BYTE_3(n)     (((n) << CAN_WORD02_DATA_BYTE_3_SHIFT) & CAN_WORD02_DATA_BYTE_3_MASK)
#define CAN_WORD02_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD02_DATA_BYTE_2_MASK   (0xFF << CAN_WORD02_DATA_BYTE_2_SHIFT)
#define CAN_WORD02_DATA_BYTE_2(n)     (((n) << CAN_WORD02_DATA_BYTE_2_SHIFT) & CAN_WORD02_DATA_BYTE_2_MASK)
#define CAN_WORD02_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD02_DATA_BYTE_1_MASK   (0xFF << CAN_WORD02_DATA_BYTE_1_SHIFT)
#define CAN_WORD02_DATA_BYTE_1(n)     (((n) << CAN_WORD02_DATA_BYTE_1_SHIFT) & CAN_WORD02_DATA_BYTE_1_MASK)
#define CAN_WORD02_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD02_DATA_BYTE_0_MASK   (0xFF << CAN_WORD02_DATA_BYTE_0_SHIFT)
#define CAN_WORD02_DATA_BYTE_0(n)     (((n) << CAN_WORD02_DATA_BYTE_0_SHIFT) & CAN_WORD02_DATA_BYTE_0_MASK)

/* Message Buffer 2 WORD1 Register (WORD12) */
#define CAN_WORD12_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD12_DATA_BYTE_7_MASK   (0xFF << CAN_WORD12_DATA_BYTE_7_SHIFT)
#define CAN_WORD12_DATA_BYTE_7(n)     (((n) << CAN_WORD12_DATA_BYTE_7_SHIFT) & CAN_WORD12_DATA_BYTE_7_MASK)
#define CAN_WORD12_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD12_DATA_BYTE_6_MASK   (0xFF << CAN_WORD12_DATA_BYTE_6_SHIFT)
#define CAN_WORD12_DATA_BYTE_6(n)     (((n) << CAN_WORD12_DATA_BYTE_6_SHIFT) & CAN_WORD12_DATA_BYTE_6_MASK)
#define CAN_WORD12_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD12_DATA_BYTE_5_MASK   (0xFF << CAN_WORD12_DATA_BYTE_5_SHIFT)
#define CAN_WORD12_DATA_BYTE_5(n)     (((n) << CAN_WORD12_DATA_BYTE_5_SHIFT) & CAN_WORD12_DATA_BYTE_5_MASK)
#define CAN_WORD12_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD12_DATA_BYTE_4_MASK   (0xFF << CAN_WORD12_DATA_BYTE_4_SHIFT)
#define CAN_WORD12_DATA_BYTE_4(n)     (((n) << CAN_WORD12_DATA_BYTE_4_SHIFT) & CAN_WORD12_DATA_BYTE_4_MASK)

/* Message Buffer 3 CS Register (CS3) */
#define CAN_CS3_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS3_TIME_STAMP_MASK   (0xFFFF << CAN_CS3_TIME_STAMP_SHIFT)
#define CAN_CS3_TIME_STAMP(n)     (((n) << CAN_CS3_TIME_STAMP_SHIFT) & CAN_CS3_TIME_STAMP_MASK)
#define CAN_CS3_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS3_DLC_MASK          (0xF << CAN_CS3_DLC_SHIFT)
#define CAN_CS3_DLC(n)            (((n) << CAN_CS3_DLC_SHIFT) & CAN_CS3_DLC_MASK)
#define CAN_CS3_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS3_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS3_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS3_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS3_CODE_MASK         (0xF << CAN_CS3_CODE_SHIFT)
#define CAN_CS3_CODE(n)           (((n) << CAN_CS3_CODE_SHIFT) & CAN_CS3_CODE_MASK)
#define CAN_CS3_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS3_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS3_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 3 WORD0 Register (WORD03) */
#define CAN_WORD03_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD03_DATA_BYTE_3_MASK   (0xFF << CAN_WORD03_DATA_BYTE_3_SHIFT)
#define CAN_WORD03_DATA_BYTE_3(n)     (((n) << CAN_WORD03_DATA_BYTE_3_SHIFT) & CAN_WORD03_DATA_BYTE_3_MASK)
#define CAN_WORD03_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD03_DATA_BYTE_2_MASK   (0xFF << CAN_WORD03_DATA_BYTE_2_SHIFT)
#define CAN_WORD03_DATA_BYTE_2(n)     (((n) << CAN_WORD03_DATA_BYTE_2_SHIFT) & CAN_WORD03_DATA_BYTE_2_MASK)
#define CAN_WORD03_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD03_DATA_BYTE_1_MASK   (0xFF << CAN_WORD03_DATA_BYTE_1_SHIFT)
#define CAN_WORD03_DATA_BYTE_1(n)     (((n) << CAN_WORD03_DATA_BYTE_1_SHIFT) & CAN_WORD03_DATA_BYTE_1_MASK)
#define CAN_WORD03_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD03_DATA_BYTE_0_MASK   (0xFF << CAN_WORD03_DATA_BYTE_0_SHIFT)
#define CAN_WORD03_DATA_BYTE_0(n)     (((n) << CAN_WORD03_DATA_BYTE_0_SHIFT) & CAN_WORD03_DATA_BYTE_0_MASK)

/* Message Buffer 3 WORD1 Register (WORD13) */
#define CAN_WORD13_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD13_DATA_BYTE_7_MASK   (0xFF << CAN_WORD13_DATA_BYTE_7_SHIFT)
#define CAN_WORD13_DATA_BYTE_7(n)     (((n) << CAN_WORD13_DATA_BYTE_7_SHIFT) & CAN_WORD13_DATA_BYTE_7_MASK)
#define CAN_WORD13_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD13_DATA_BYTE_6_MASK   (0xFF << CAN_WORD13_DATA_BYTE_6_SHIFT)
#define CAN_WORD13_DATA_BYTE_6(n)     (((n) << CAN_WORD13_DATA_BYTE_6_SHIFT) & CAN_WORD13_DATA_BYTE_6_MASK)
#define CAN_WORD13_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD13_DATA_BYTE_5_MASK   (0xFF << CAN_WORD13_DATA_BYTE_5_SHIFT)
#define CAN_WORD13_DATA_BYTE_5(n)     (((n) << CAN_WORD13_DATA_BYTE_5_SHIFT) & CAN_WORD13_DATA_BYTE_5_MASK)
#define CAN_WORD13_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD13_DATA_BYTE_4_MASK   (0xFF << CAN_WORD13_DATA_BYTE_4_SHIFT)
#define CAN_WORD13_DATA_BYTE_4(n)     (((n) << CAN_WORD13_DATA_BYTE_4_SHIFT) & CAN_WORD13_DATA_BYTE_4_MASK)

/* Message Buffer 4 CS Register (CS4) */
#define CAN_CS4_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS4_TIME_STAMP_MASK   (0xFFFF << CAN_CS4_TIME_STAMP_SHIFT)
#define CAN_CS4_TIME_STAMP(n)     (((n) << CAN_CS4_TIME_STAMP_SHIFT) & CAN_CS4_TIME_STAMP_MASK)
#define CAN_CS4_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS4_DLC_MASK          (0xF << CAN_CS4_DLC_SHIFT)
#define CAN_CS4_DLC(n)            (((n) << CAN_CS4_DLC_SHIFT) & CAN_CS4_DLC_MASK)
#define CAN_CS4_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS4_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS4_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS4_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS4_CODE_MASK         (0xF << CAN_CS4_CODE_SHIFT)
#define CAN_CS4_CODE(n)           (((n) << CAN_CS4_CODE_SHIFT) & CAN_CS4_CODE_MASK)
#define CAN_CS4_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS4_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS4_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 4 WORD0 Register (WORD04) */
#define CAN_WORD04_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD04_DATA_BYTE_3_MASK   (0xFF << CAN_WORD04_DATA_BYTE_3_SHIFT)
#define CAN_WORD04_DATA_BYTE_3(n)     (((n) << CAN_WORD04_DATA_BYTE_3_SHIFT) & CAN_WORD04_DATA_BYTE_3_MASK)
#define CAN_WORD04_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD04_DATA_BYTE_2_MASK   (0xFF << CAN_WORD04_DATA_BYTE_2_SHIFT)
#define CAN_WORD04_DATA_BYTE_2(n)     (((n) << CAN_WORD04_DATA_BYTE_2_SHIFT) & CAN_WORD04_DATA_BYTE_2_MASK)
#define CAN_WORD04_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD04_DATA_BYTE_1_MASK   (0xFF << CAN_WORD04_DATA_BYTE_1_SHIFT)
#define CAN_WORD04_DATA_BYTE_1(n)     (((n) << CAN_WORD04_DATA_BYTE_1_SHIFT) & CAN_WORD04_DATA_BYTE_1_MASK)
#define CAN_WORD04_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD04_DATA_BYTE_0_MASK   (0xFF << CAN_WORD04_DATA_BYTE_0_SHIFT)
#define CAN_WORD04_DATA_BYTE_0(n)     (((n) << CAN_WORD04_DATA_BYTE_0_SHIFT) & CAN_WORD04_DATA_BYTE_0_MASK)

/* Message Buffer 4 WORD1 Register (WORD14) */
#define CAN_WORD14_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD14_DATA_BYTE_7_MASK   (0xFF << CAN_WORD14_DATA_BYTE_7_SHIFT)
#define CAN_WORD14_DATA_BYTE_7(n)     (((n) << CAN_WORD14_DATA_BYTE_7_SHIFT) & CAN_WORD14_DATA_BYTE_7_MASK)
#define CAN_WORD14_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD14_DATA_BYTE_6_MASK   (0xFF << CAN_WORD14_DATA_BYTE_6_SHIFT)
#define CAN_WORD14_DATA_BYTE_6(n)     (((n) << CAN_WORD14_DATA_BYTE_6_SHIFT) & CAN_WORD14_DATA_BYTE_6_MASK)
#define CAN_WORD14_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD14_DATA_BYTE_5_MASK   (0xFF << CAN_WORD14_DATA_BYTE_5_SHIFT)
#define CAN_WORD14_DATA_BYTE_5(n)     (((n) << CAN_WORD14_DATA_BYTE_5_SHIFT) & CAN_WORD14_DATA_BYTE_5_MASK)
#define CAN_WORD14_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD14_DATA_BYTE_4_MASK   (0xFF << CAN_WORD14_DATA_BYTE_4_SHIFT)
#define CAN_WORD14_DATA_BYTE_4(n)     (((n) << CAN_WORD14_DATA_BYTE_4_SHIFT) & CAN_WORD14_DATA_BYTE_4_MASK)

/* Message Buffer 5 CS Register (CS5) */
#define CAN_CS5_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS5_TIME_STAMP_MASK   (0xFFFF << CAN_CS5_TIME_STAMP_SHIFT)
#define CAN_CS5_TIME_STAMP(n)     (((n) << CAN_CS5_TIME_STAMP_SHIFT) & CAN_CS5_TIME_STAMP_MASK)
#define CAN_CS5_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS5_DLC_MASK          (0xF << CAN_CS5_DLC_SHIFT)
#define CAN_CS5_DLC(n)            (((n) << CAN_CS5_DLC_SHIFT) & CAN_CS5_DLC_MASK)
#define CAN_CS5_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS5_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS5_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS5_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS5_CODE_MASK         (0xF << CAN_CS5_CODE_SHIFT)
#define CAN_CS5_CODE(n)           (((n) << CAN_CS5_CODE_SHIFT) & CAN_CS5_CODE_MASK)
#define CAN_CS5_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS5_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS5_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 5 WORD0 Register (WORD05) */
#define CAN_WORD05_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD05_DATA_BYTE_3_MASK   (0xFF << CAN_WORD05_DATA_BYTE_3_SHIFT)
#define CAN_WORD05_DATA_BYTE_3(n)     (((n) << CAN_WORD05_DATA_BYTE_3_SHIFT) & CAN_WORD05_DATA_BYTE_3_MASK)
#define CAN_WORD05_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD05_DATA_BYTE_2_MASK   (0xFF << CAN_WORD05_DATA_BYTE_2_SHIFT)
#define CAN_WORD05_DATA_BYTE_2(n)     (((n) << CAN_WORD05_DATA_BYTE_2_SHIFT) & CAN_WORD05_DATA_BYTE_2_MASK)
#define CAN_WORD05_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD05_DATA_BYTE_1_MASK   (0xFF << CAN_WORD05_DATA_BYTE_1_SHIFT)
#define CAN_WORD05_DATA_BYTE_1(n)     (((n) << CAN_WORD05_DATA_BYTE_1_SHIFT) & CAN_WORD05_DATA_BYTE_1_MASK)
#define CAN_WORD05_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD05_DATA_BYTE_0_MASK   (0xFF << CAN_WORD05_DATA_BYTE_0_SHIFT)
#define CAN_WORD05_DATA_BYTE_0(n)     (((n) << CAN_WORD05_DATA_BYTE_0_SHIFT) & CAN_WORD05_DATA_BYTE_0_MASK)

/* Message Buffer 5 WORD1 Register (WORD15) */
#define CAN_WORD15_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD15_DATA_BYTE_7_MASK   (0xFF << CAN_WORD15_DATA_BYTE_7_SHIFT)
#define CAN_WORD15_DATA_BYTE_7(n)     (((n) << CAN_WORD15_DATA_BYTE_7_SHIFT) & CAN_WORD15_DATA_BYTE_7_MASK)
#define CAN_WORD15_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD15_DATA_BYTE_6_MASK   (0xFF << CAN_WORD15_DATA_BYTE_6_SHIFT)
#define CAN_WORD15_DATA_BYTE_6(n)     (((n) << CAN_WORD15_DATA_BYTE_6_SHIFT) & CAN_WORD15_DATA_BYTE_6_MASK)
#define CAN_WORD15_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD15_DATA_BYTE_5_MASK   (0xFF << CAN_WORD15_DATA_BYTE_5_SHIFT)
#define CAN_WORD15_DATA_BYTE_5(n)     (((n) << CAN_WORD15_DATA_BYTE_5_SHIFT) & CAN_WORD15_DATA_BYTE_5_MASK)
#define CAN_WORD15_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD15_DATA_BYTE_4_MASK   (0xFF << CAN_WORD15_DATA_BYTE_4_SHIFT)
#define CAN_WORD15_DATA_BYTE_4(n)     (((n) << CAN_WORD15_DATA_BYTE_4_SHIFT) & CAN_WORD15_DATA_BYTE_4_MASK)

/* Message Buffer 6 CS Register (CS6) */
#define CAN_CS6_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS6_TIME_STAMP_MASK   (0xFFFF << CAN_CS6_TIME_STAMP_SHIFT)
#define CAN_CS6_TIME_STAMP(n)     (((n) << CAN_CS6_TIME_STAMP_SHIFT) & CAN_CS6_TIME_STAMP_MASK)
#define CAN_CS6_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS6_DLC_MASK          (0xF << CAN_CS6_DLC_SHIFT)
#define CAN_CS6_DLC(n)            (((n) << CAN_CS6_DLC_SHIFT) & CAN_CS6_DLC_MASK)
#define CAN_CS6_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS6_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS6_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS6_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS6_CODE_MASK         (0xF << CAN_CS6_CODE_SHIFT)
#define CAN_CS6_CODE(n)           (((n) << CAN_CS6_CODE_SHIFT) & CAN_CS6_CODE_MASK)
#define CAN_CS6_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS6_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS6_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 6 WORD0 Register (WORD06) */
#define CAN_WORD06_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD06_DATA_BYTE_3_MASK   (0xFF << CAN_WORD06_DATA_BYTE_3_SHIFT)
#define CAN_WORD06_DATA_BYTE_3(n)     (((n) << CAN_WORD06_DATA_BYTE_3_SHIFT) & CAN_WORD06_DATA_BYTE_3_MASK)
#define CAN_WORD06_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD06_DATA_BYTE_2_MASK   (0xFF << CAN_WORD06_DATA_BYTE_2_SHIFT)
#define CAN_WORD06_DATA_BYTE_2(n)     (((n) << CAN_WORD06_DATA_BYTE_2_SHIFT) & CAN_WORD06_DATA_BYTE_2_MASK)
#define CAN_WORD06_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD06_DATA_BYTE_1_MASK   (0xFF << CAN_WORD06_DATA_BYTE_1_SHIFT)
#define CAN_WORD06_DATA_BYTE_1(n)     (((n) << CAN_WORD06_DATA_BYTE_1_SHIFT) & CAN_WORD06_DATA_BYTE_1_MASK)
#define CAN_WORD06_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD06_DATA_BYTE_0_MASK   (0xFF << CAN_WORD06_DATA_BYTE_0_SHIFT)
#define CAN_WORD06_DATA_BYTE_0(n)     (((n) << CAN_WORD06_DATA_BYTE_0_SHIFT) & CAN_WORD06_DATA_BYTE_0_MASK)

/* Message Buffer 6 WORD1 Register (WORD16) */
#define CAN_WORD16_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD16_DATA_BYTE_7_MASK   (0xFF << CAN_WORD16_DATA_BYTE_7_SHIFT)
#define CAN_WORD16_DATA_BYTE_7(n)     (((n) << CAN_WORD16_DATA_BYTE_7_SHIFT) & CAN_WORD16_DATA_BYTE_7_MASK)
#define CAN_WORD16_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD16_DATA_BYTE_6_MASK   (0xFF << CAN_WORD16_DATA_BYTE_6_SHIFT)
#define CAN_WORD16_DATA_BYTE_6(n)     (((n) << CAN_WORD16_DATA_BYTE_6_SHIFT) & CAN_WORD16_DATA_BYTE_6_MASK)
#define CAN_WORD16_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD16_DATA_BYTE_5_MASK   (0xFF << CAN_WORD16_DATA_BYTE_5_SHIFT)
#define CAN_WORD16_DATA_BYTE_5(n)     (((n) << CAN_WORD16_DATA_BYTE_5_SHIFT) & CAN_WORD16_DATA_BYTE_5_MASK)
#define CAN_WORD16_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD16_DATA_BYTE_4_MASK   (0xFF << CAN_WORD16_DATA_BYTE_4_SHIFT)
#define CAN_WORD16_DATA_BYTE_4(n)     (((n) << CAN_WORD16_DATA_BYTE_4_SHIFT) & CAN_WORD16_DATA_BYTE_4_MASK)

/* Message Buffer 7 CS Register (CS7) */
#define CAN_CS7_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS7_TIME_STAMP_MASK   (0xFFFF << CAN_CS7_TIME_STAMP_SHIFT)
#define CAN_CS7_TIME_STAMP(n)     (((n) << CAN_CS7_TIME_STAMP_SHIFT) & CAN_CS7_TIME_STAMP_MASK)
#define CAN_CS7_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS7_DLC_MASK          (0xF << CAN_CS7_DLC_SHIFT)
#define CAN_CS7_DLC(n)            (((n) << CAN_CS7_DLC_SHIFT) & CAN_CS7_DLC_MASK)
#define CAN_CS7_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS7_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS7_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS7_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS7_CODE_MASK         (0xF << CAN_CS7_CODE_SHIFT)
#define CAN_CS7_CODE(n)           (((n) << CAN_CS7_CODE_SHIFT) & CAN_CS7_CODE_MASK)
#define CAN_CS7_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS7_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS7_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 7 WORD0 Register (WORD07) */
#define CAN_WORD07_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD07_DATA_BYTE_3_MASK   (0xFF << CAN_WORD07_DATA_BYTE_3_SHIFT)
#define CAN_WORD07_DATA_BYTE_3(n)     (((n) << CAN_WORD07_DATA_BYTE_3_SHIFT) & CAN_WORD07_DATA_BYTE_3_MASK)
#define CAN_WORD07_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD07_DATA_BYTE_2_MASK   (0xFF << CAN_WORD07_DATA_BYTE_2_SHIFT)
#define CAN_WORD07_DATA_BYTE_2(n)     (((n) << CAN_WORD07_DATA_BYTE_2_SHIFT) & CAN_WORD07_DATA_BYTE_2_MASK)
#define CAN_WORD07_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD07_DATA_BYTE_1_MASK   (0xFF << CAN_WORD07_DATA_BYTE_1_SHIFT)
#define CAN_WORD07_DATA_BYTE_1(n)     (((n) << CAN_WORD07_DATA_BYTE_1_SHIFT) & CAN_WORD07_DATA_BYTE_1_MASK)
#define CAN_WORD07_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD07_DATA_BYTE_0_MASK   (0xFF << CAN_WORD07_DATA_BYTE_0_SHIFT)
#define CAN_WORD07_DATA_BYTE_0(n)     (((n) << CAN_WORD07_DATA_BYTE_0_SHIFT) & CAN_WORD07_DATA_BYTE_0_MASK)

/* Message Buffer 7 WORD1 Register (WORD17) */
#define CAN_WORD17_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD17_DATA_BYTE_7_MASK   (0xFF << CAN_WORD17_DATA_BYTE_7_SHIFT)
#define CAN_WORD17_DATA_BYTE_7(n)     (((n) << CAN_WORD17_DATA_BYTE_7_SHIFT) & CAN_WORD17_DATA_BYTE_7_MASK)
#define CAN_WORD17_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD17_DATA_BYTE_6_MASK   (0xFF << CAN_WORD17_DATA_BYTE_6_SHIFT)
#define CAN_WORD17_DATA_BYTE_6(n)     (((n) << CAN_WORD17_DATA_BYTE_6_SHIFT) & CAN_WORD17_DATA_BYTE_6_MASK)
#define CAN_WORD17_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD17_DATA_BYTE_5_MASK   (0xFF << CAN_WORD17_DATA_BYTE_5_SHIFT)
#define CAN_WORD17_DATA_BYTE_5(n)     (((n) << CAN_WORD17_DATA_BYTE_5_SHIFT) & CAN_WORD17_DATA_BYTE_5_MASK)
#define CAN_WORD17_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD17_DATA_BYTE_4_MASK   (0xFF << CAN_WORD17_DATA_BYTE_4_SHIFT)
#define CAN_WORD17_DATA_BYTE_4(n)     (((n) << CAN_WORD17_DATA_BYTE_4_SHIFT) & CAN_WORD17_DATA_BYTE_4_MASK)

/* Message Buffer 8 CS Register (CS8) */
#define CAN_CS8_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS8_TIME_STAMP_MASK   (0xFFFF << CAN_CS8_TIME_STAMP_SHIFT)
#define CAN_CS8_TIME_STAMP(n)     (((n) << CAN_CS8_TIME_STAMP_SHIFT) & CAN_CS8_TIME_STAMP_MASK)
#define CAN_CS8_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS8_DLC_MASK          (0xF << CAN_CS8_DLC_SHIFT)
#define CAN_CS8_DLC(n)            (((n) << CAN_CS8_DLC_SHIFT) & CAN_CS8_DLC_MASK)
#define CAN_CS8_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS8_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS8_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS8_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS8_CODE_MASK         (0xF << CAN_CS8_CODE_SHIFT)
#define CAN_CS8_CODE(n)           (((n) << CAN_CS8_CODE_SHIFT) & CAN_CS8_CODE_MASK)
#define CAN_CS8_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS8_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS8_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 8 WORD0 Register (WORD08) */
#define CAN_WORD08_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD08_DATA_BYTE_3_MASK   (0xFF << CAN_WORD08_DATA_BYTE_3_SHIFT)
#define CAN_WORD08_DATA_BYTE_3(n)     (((n) << CAN_WORD08_DATA_BYTE_3_SHIFT) & CAN_WORD08_DATA_BYTE_3_MASK)
#define CAN_WORD08_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD08_DATA_BYTE_2_MASK   (0xFF << CAN_WORD08_DATA_BYTE_2_SHIFT)
#define CAN_WORD08_DATA_BYTE_2(n)     (((n) << CAN_WORD08_DATA_BYTE_2_SHIFT) & CAN_WORD08_DATA_BYTE_2_MASK)
#define CAN_WORD08_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD08_DATA_BYTE_1_MASK   (0xFF << CAN_WORD08_DATA_BYTE_1_SHIFT)
#define CAN_WORD08_DATA_BYTE_1(n)     (((n) << CAN_WORD08_DATA_BYTE_1_SHIFT) & CAN_WORD08_DATA_BYTE_1_MASK)
#define CAN_WORD08_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD08_DATA_BYTE_0_MASK   (0xFF << CAN_WORD08_DATA_BYTE_0_SHIFT)
#define CAN_WORD08_DATA_BYTE_0(n)     (((n) << CAN_WORD08_DATA_BYTE_0_SHIFT) & CAN_WORD08_DATA_BYTE_0_MASK)

/* Message Buffer 8 WORD1 Register (WORD18) */
#define CAN_WORD18_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD18_DATA_BYTE_7_MASK   (0xFF << CAN_WORD18_DATA_BYTE_7_SHIFT)
#define CAN_WORD18_DATA_BYTE_7(n)     (((n) << CAN_WORD18_DATA_BYTE_7_SHIFT) & CAN_WORD18_DATA_BYTE_7_MASK)
#define CAN_WORD18_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD18_DATA_BYTE_6_MASK   (0xFF << CAN_WORD18_DATA_BYTE_6_SHIFT)
#define CAN_WORD18_DATA_BYTE_6(n)     (((n) << CAN_WORD18_DATA_BYTE_6_SHIFT) & CAN_WORD18_DATA_BYTE_6_MASK)
#define CAN_WORD18_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD18_DATA_BYTE_5_MASK   (0xFF << CAN_WORD18_DATA_BYTE_5_SHIFT)
#define CAN_WORD18_DATA_BYTE_5(n)     (((n) << CAN_WORD18_DATA_BYTE_5_SHIFT) & CAN_WORD18_DATA_BYTE_5_MASK)
#define CAN_WORD18_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD18_DATA_BYTE_4_MASK   (0xFF << CAN_WORD18_DATA_BYTE_4_SHIFT)
#define CAN_WORD18_DATA_BYTE_4(n)     (((n) << CAN_WORD18_DATA_BYTE_4_SHIFT) & CAN_WORD18_DATA_BYTE_4_MASK)

/* Message Buffer 9 CS Register (CS9) */
#define CAN_CS9_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS9_TIME_STAMP_MASK   (0xFFFF << CAN_CS9_TIME_STAMP_SHIFT)
#define CAN_CS9_TIME_STAMP(n)     (((n) << CAN_CS9_TIME_STAMP_SHIFT) & CAN_CS9_TIME_STAMP_MASK)
#define CAN_CS9_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS9_DLC_MASK          (0xF << CAN_CS9_DLC_SHIFT)
#define CAN_CS9_DLC(n)            (((n) << CAN_CS9_DLC_SHIFT) & CAN_CS9_DLC_MASK)
#define CAN_CS9_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS9_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS9_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS9_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS9_CODE_MASK         (0xF << CAN_CS9_CODE_SHIFT)
#define CAN_CS9_CODE(n)           (((n) << CAN_CS9_CODE_SHIFT) & CAN_CS9_CODE_MASK)
#define CAN_CS9_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS9_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS9_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 9 WORD0 Register (WORD09) */
#define CAN_WORD09_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD09_DATA_BYTE_3_MASK   (0xFF << CAN_WORD09_DATA_BYTE_3_SHIFT)
#define CAN_WORD09_DATA_BYTE_3(n)     (((n) << CAN_WORD09_DATA_BYTE_3_SHIFT) & CAN_WORD09_DATA_BYTE_3_MASK)
#define CAN_WORD09_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD09_DATA_BYTE_2_MASK   (0xFF << CAN_WORD09_DATA_BYTE_2_SHIFT)
#define CAN_WORD09_DATA_BYTE_2(n)     (((n) << CAN_WORD09_DATA_BYTE_2_SHIFT) & CAN_WORD09_DATA_BYTE_2_MASK)
#define CAN_WORD09_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD09_DATA_BYTE_1_MASK   (0xFF << CAN_WORD09_DATA_BYTE_1_SHIFT)
#define CAN_WORD09_DATA_BYTE_1(n)     (((n) << CAN_WORD09_DATA_BYTE_1_SHIFT) & CAN_WORD09_DATA_BYTE_1_MASK)
#define CAN_WORD09_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD09_DATA_BYTE_0_MASK   (0xFF << CAN_WORD09_DATA_BYTE_0_SHIFT)
#define CAN_WORD09_DATA_BYTE_0(n)     (((n) << CAN_WORD09_DATA_BYTE_0_SHIFT) & CAN_WORD09_DATA_BYTE_0_MASK)

/* Message Buffer 9 WORD1 Register (WORD19) */
#define CAN_WORD19_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD19_DATA_BYTE_7_MASK   (0xFF << CAN_WORD19_DATA_BYTE_7_SHIFT)
#define CAN_WORD19_DATA_BYTE_7(n)     (((n) << CAN_WORD19_DATA_BYTE_7_SHIFT) & CAN_WORD19_DATA_BYTE_7_MASK)
#define CAN_WORD19_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD19_DATA_BYTE_6_MASK   (0xFF << CAN_WORD19_DATA_BYTE_6_SHIFT)
#define CAN_WORD19_DATA_BYTE_6(n)     (((n) << CAN_WORD19_DATA_BYTE_6_SHIFT) & CAN_WORD19_DATA_BYTE_6_MASK)
#define CAN_WORD19_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD19_DATA_BYTE_5_MASK   (0xFF << CAN_WORD19_DATA_BYTE_5_SHIFT)
#define CAN_WORD19_DATA_BYTE_5(n)     (((n) << CAN_WORD19_DATA_BYTE_5_SHIFT) & CAN_WORD19_DATA_BYTE_5_MASK)
#define CAN_WORD19_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD19_DATA_BYTE_4_MASK   (0xFF << CAN_WORD19_DATA_BYTE_4_SHIFT)
#define CAN_WORD19_DATA_BYTE_4(n)     (((n) << CAN_WORD19_DATA_BYTE_4_SHIFT) & CAN_WORD19_DATA_BYTE_4_MASK)

/* Message Buffer 10 CS Register (CS10) */
#define CAN_CS10_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS10_TIME_STAMP_MASK   (0xFFFF << CAN_CS10_TIME_STAMP_SHIFT)
#define CAN_CS10_TIME_STAMP(n)     (((n) << CAN_CS10_TIME_STAMP_SHIFT) & CAN_CS10_TIME_STAMP_MASK)
#define CAN_CS10_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS10_DLC_MASK          (0xF << CAN_CS10_DLC_SHIFT)
#define CAN_CS10_DLC(n)            (((n) << CAN_CS10_DLC_SHIFT) & CAN_CS10_DLC_MASK)
#define CAN_CS10_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS10_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS10_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS10_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS10_CODE_MASK         (0xF << CAN_CS10_CODE_SHIFT)
#define CAN_CS10_CODE(n)           (((n) << CAN_CS10_CODE_SHIFT) & CAN_CS10_CODE_MASK)
#define CAN_CS10_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS10_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS10_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 10 WORD0 Register (WORD010) */
#define CAN_WORD010_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD010_DATA_BYTE_3_MASK   (0xFF << CAN_WORD010_DATA_BYTE_3_SHIFT)
#define CAN_WORD010_DATA_BYTE_3(n)     (((n) << CAN_WORD010_DATA_BYTE_3_SHIFT) & CAN_WORD010_DATA_BYTE_3_MASK)
#define CAN_WORD010_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD010_DATA_BYTE_2_MASK   (0xFF << CAN_WORD010_DATA_BYTE_2_SHIFT)
#define CAN_WORD010_DATA_BYTE_2(n)     (((n) << CAN_WORD010_DATA_BYTE_2_SHIFT) & CAN_WORD010_DATA_BYTE_2_MASK)
#define CAN_WORD010_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD010_DATA_BYTE_1_MASK   (0xFF << CAN_WORD010_DATA_BYTE_1_SHIFT)
#define CAN_WORD010_DATA_BYTE_1(n)     (((n) << CAN_WORD010_DATA_BYTE_1_SHIFT) & CAN_WORD010_DATA_BYTE_1_MASK)
#define CAN_WORD010_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD010_DATA_BYTE_0_MASK   (0xFF << CAN_WORD010_DATA_BYTE_0_SHIFT)
#define CAN_WORD010_DATA_BYTE_0(n)     (((n) << CAN_WORD010_DATA_BYTE_0_SHIFT) & CAN_WORD010_DATA_BYTE_0_MASK)

/* Message Buffer 10 WORD1 Register (WORD110) */
#define CAN_WORD110_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD110_DATA_BYTE_7_MASK   (0xFF << CAN_WORD110_DATA_BYTE_7_SHIFT)
#define CAN_WORD110_DATA_BYTE_7(n)     (((n) << CAN_WORD110_DATA_BYTE_7_SHIFT) & CAN_WORD110_DATA_BYTE_7_MASK)
#define CAN_WORD110_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD110_DATA_BYTE_6_MASK   (0xFF << CAN_WORD110_DATA_BYTE_6_SHIFT)
#define CAN_WORD110_DATA_BYTE_6(n)     (((n) << CAN_WORD110_DATA_BYTE_6_SHIFT) & CAN_WORD110_DATA_BYTE_6_MASK)
#define CAN_WORD110_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD110_DATA_BYTE_5_MASK   (0xFF << CAN_WORD110_DATA_BYTE_5_SHIFT)
#define CAN_WORD110_DATA_BYTE_5(n)     (((n) << CAN_WORD110_DATA_BYTE_5_SHIFT) & CAN_WORD110_DATA_BYTE_5_MASK)
#define CAN_WORD110_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD110_DATA_BYTE_4_MASK   (0xFF << CAN_WORD110_DATA_BYTE_4_SHIFT)
#define CAN_WORD110_DATA_BYTE_4(n)     (((n) << CAN_WORD110_DATA_BYTE_4_SHIFT) & CAN_WORD110_DATA_BYTE_4_MASK)

/* Message Buffer 11 CS Register (CS11) */
#define CAN_CS11_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS11_TIME_STAMP_MASK   (0xFFFF << CAN_CS11_TIME_STAMP_SHIFT)
#define CAN_CS11_TIME_STAMP(n)     (((n) << CAN_CS11_TIME_STAMP_SHIFT) & CAN_CS11_TIME_STAMP_MASK)
#define CAN_CS11_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS11_DLC_MASK          (0xF << CAN_CS11_DLC_SHIFT)
#define CAN_CS11_DLC(n)            (((n) << CAN_CS11_DLC_SHIFT) & CAN_CS11_DLC_MASK)
#define CAN_CS11_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS11_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS11_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS11_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS11_CODE_MASK         (0xF << CAN_CS11_CODE_SHIFT)
#define CAN_CS11_CODE(n)           (((n) << CAN_CS11_CODE_SHIFT) & CAN_CS11_CODE_MASK)
#define CAN_CS11_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS11_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS11_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 11 WORD0 Register (WORD011) */
#define CAN_WORD011_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD011_DATA_BYTE_3_MASK   (0xFF << CAN_WORD011_DATA_BYTE_3_SHIFT)
#define CAN_WORD011_DATA_BYTE_3(n)     (((n) << CAN_WORD011_DATA_BYTE_3_SHIFT) & CAN_WORD011_DATA_BYTE_3_MASK)
#define CAN_WORD011_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD011_DATA_BYTE_2_MASK   (0xFF << CAN_WORD011_DATA_BYTE_2_SHIFT)
#define CAN_WORD011_DATA_BYTE_2(n)     (((n) << CAN_WORD011_DATA_BYTE_2_SHIFT) & CAN_WORD011_DATA_BYTE_2_MASK)
#define CAN_WORD011_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD011_DATA_BYTE_1_MASK   (0xFF << CAN_WORD011_DATA_BYTE_1_SHIFT)
#define CAN_WORD011_DATA_BYTE_1(n)     (((n) << CAN_WORD011_DATA_BYTE_1_SHIFT) & CAN_WORD011_DATA_BYTE_1_MASK)
#define CAN_WORD011_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD011_DATA_BYTE_0_MASK   (0xFF << CAN_WORD011_DATA_BYTE_0_SHIFT)
#define CAN_WORD011_DATA_BYTE_0(n)     (((n) << CAN_WORD011_DATA_BYTE_0_SHIFT) & CAN_WORD011_DATA_BYTE_0_MASK)

/* Message Buffer 11 WORD1 Register (WORD111) */
#define CAN_WORD111_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD111_DATA_BYTE_7_MASK   (0xFF << CAN_WORD111_DATA_BYTE_7_SHIFT)
#define CAN_WORD111_DATA_BYTE_7(n)     (((n) << CAN_WORD111_DATA_BYTE_7_SHIFT) & CAN_WORD111_DATA_BYTE_7_MASK)
#define CAN_WORD111_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD111_DATA_BYTE_6_MASK   (0xFF << CAN_WORD111_DATA_BYTE_6_SHIFT)
#define CAN_WORD111_DATA_BYTE_6(n)     (((n) << CAN_WORD111_DATA_BYTE_6_SHIFT) & CAN_WORD111_DATA_BYTE_6_MASK)
#define CAN_WORD111_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD111_DATA_BYTE_5_MASK   (0xFF << CAN_WORD111_DATA_BYTE_5_SHIFT)
#define CAN_WORD111_DATA_BYTE_5(n)     (((n) << CAN_WORD111_DATA_BYTE_5_SHIFT) & CAN_WORD111_DATA_BYTE_5_MASK)
#define CAN_WORD111_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD111_DATA_BYTE_4_MASK   (0xFF << CAN_WORD111_DATA_BYTE_4_SHIFT)
#define CAN_WORD111_DATA_BYTE_4(n)     (((n) << CAN_WORD111_DATA_BYTE_4_SHIFT) & CAN_WORD111_DATA_BYTE_4_MASK)

/* Message Buffer 12 CS Register (CS12) */
#define CAN_CS12_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS12_TIME_STAMP_MASK   (0xFFFF << CAN_CS12_TIME_STAMP_SHIFT)
#define CAN_CS12_TIME_STAMP(n)     (((n) << CAN_CS12_TIME_STAMP_SHIFT) & CAN_CS12_TIME_STAMP_MASK)
#define CAN_CS12_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS12_DLC_MASK          (0xF << CAN_CS12_DLC_SHIFT)
#define CAN_CS12_DLC(n)            (((n) << CAN_CS12_DLC_SHIFT) & CAN_CS12_DLC_MASK)
#define CAN_CS12_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS12_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS12_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS12_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS12_CODE_MASK         (0xF << CAN_CS12_CODE_SHIFT)
#define CAN_CS12_CODE(n)           (((n) << CAN_CS12_CODE_SHIFT) & CAN_CS12_CODE_MASK)
#define CAN_CS12_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS12_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS12_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 12 WORD0 Register (WORD012) */
#define CAN_WORD012_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD012_DATA_BYTE_3_MASK   (0xFF << CAN_WORD012_DATA_BYTE_3_SHIFT)
#define CAN_WORD012_DATA_BYTE_3(n)     (((n) << CAN_WORD012_DATA_BYTE_3_SHIFT) & CAN_WORD012_DATA_BYTE_3_MASK)
#define CAN_WORD012_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD012_DATA_BYTE_2_MASK   (0xFF << CAN_WORD012_DATA_BYTE_2_SHIFT)
#define CAN_WORD012_DATA_BYTE_2(n)     (((n) << CAN_WORD012_DATA_BYTE_2_SHIFT) & CAN_WORD012_DATA_BYTE_2_MASK)
#define CAN_WORD012_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD012_DATA_BYTE_1_MASK   (0xFF << CAN_WORD012_DATA_BYTE_1_SHIFT)
#define CAN_WORD012_DATA_BYTE_1(n)     (((n) << CAN_WORD012_DATA_BYTE_1_SHIFT) & CAN_WORD012_DATA_BYTE_1_MASK)
#define CAN_WORD012_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD012_DATA_BYTE_0_MASK   (0xFF << CAN_WORD012_DATA_BYTE_0_SHIFT)
#define CAN_WORD012_DATA_BYTE_0(n)     (((n) << CAN_WORD012_DATA_BYTE_0_SHIFT) & CAN_WORD012_DATA_BYTE_0_MASK)

/* Message Buffer 12 WORD1 Register (WORD112) */
#define CAN_WORD112_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD112_DATA_BYTE_7_MASK   (0xFF << CAN_WORD112_DATA_BYTE_7_SHIFT)
#define CAN_WORD112_DATA_BYTE_7(n)     (((n) << CAN_WORD112_DATA_BYTE_7_SHIFT) & CAN_WORD112_DATA_BYTE_7_MASK)
#define CAN_WORD112_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD112_DATA_BYTE_6_MASK   (0xFF << CAN_WORD112_DATA_BYTE_6_SHIFT)
#define CAN_WORD112_DATA_BYTE_6(n)     (((n) << CAN_WORD112_DATA_BYTE_6_SHIFT) & CAN_WORD112_DATA_BYTE_6_MASK)
#define CAN_WORD112_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD112_DATA_BYTE_5_MASK   (0xFF << CAN_WORD112_DATA_BYTE_5_SHIFT)
#define CAN_WORD112_DATA_BYTE_5(n)     (((n) << CAN_WORD112_DATA_BYTE_5_SHIFT) & CAN_WORD112_DATA_BYTE_5_MASK)
#define CAN_WORD112_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD112_DATA_BYTE_4_MASK   (0xFF << CAN_WORD112_DATA_BYTE_4_SHIFT)
#define CAN_WORD112_DATA_BYTE_4(n)     (((n) << CAN_WORD112_DATA_BYTE_4_SHIFT) & CAN_WORD112_DATA_BYTE_4_MASK)

/* Message Buffer 13 CS Register (CS13) */
#define CAN_CS13_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS13_TIME_STAMP_MASK   (0xFFFF << CAN_CS13_TIME_STAMP_SHIFT)
#define CAN_CS13_TIME_STAMP(n)     (((n) << CAN_CS13_TIME_STAMP_SHIFT) & CAN_CS13_TIME_STAMP_MASK)
#define CAN_CS13_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS13_DLC_MASK          (0xF << CAN_CS13_DLC_SHIFT)
#define CAN_CS13_DLC(n)            (((n) << CAN_CS13_DLC_SHIFT) & CAN_CS13_DLC_MASK)
#define CAN_CS13_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS13_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS13_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS13_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS13_CODE_MASK         (0xF << CAN_CS13_CODE_SHIFT)
#define CAN_CS13_CODE(n)           (((n) << CAN_CS13_CODE_SHIFT) & CAN_CS13_CODE_MASK)
#define CAN_CS13_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS13_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS13_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 13 WORD0 Register (WORD013) */
#define CAN_WORD013_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD013_DATA_BYTE_3_MASK   (0xFF << CAN_WORD013_DATA_BYTE_3_SHIFT)
#define CAN_WORD013_DATA_BYTE_3(n)     (((n) << CAN_WORD013_DATA_BYTE_3_SHIFT) & CAN_WORD013_DATA_BYTE_3_MASK)
#define CAN_WORD013_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD013_DATA_BYTE_2_MASK   (0xFF << CAN_WORD013_DATA_BYTE_2_SHIFT)
#define CAN_WORD013_DATA_BYTE_2(n)     (((n) << CAN_WORD013_DATA_BYTE_2_SHIFT) & CAN_WORD013_DATA_BYTE_2_MASK)
#define CAN_WORD013_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD013_DATA_BYTE_1_MASK   (0xFF << CAN_WORD013_DATA_BYTE_1_SHIFT)
#define CAN_WORD013_DATA_BYTE_1(n)     (((n) << CAN_WORD013_DATA_BYTE_1_SHIFT) & CAN_WORD013_DATA_BYTE_1_MASK)
#define CAN_WORD013_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD013_DATA_BYTE_0_MASK   (0xFF << CAN_WORD013_DATA_BYTE_0_SHIFT)
#define CAN_WORD013_DATA_BYTE_0(n)     (((n) << CAN_WORD013_DATA_BYTE_0_SHIFT) & CAN_WORD013_DATA_BYTE_0_MASK)

/* Message Buffer 13 WORD1 Register (WORD113) */
#define CAN_WORD113_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD113_DATA_BYTE_7_MASK   (0xFF << CAN_WORD113_DATA_BYTE_7_SHIFT)
#define CAN_WORD113_DATA_BYTE_7(n)     (((n) << CAN_WORD113_DATA_BYTE_7_SHIFT) & CAN_WORD113_DATA_BYTE_7_MASK)
#define CAN_WORD113_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD113_DATA_BYTE_6_MASK   (0xFF << CAN_WORD113_DATA_BYTE_6_SHIFT)
#define CAN_WORD113_DATA_BYTE_6(n)     (((n) << CAN_WORD113_DATA_BYTE_6_SHIFT) & CAN_WORD113_DATA_BYTE_6_MASK)
#define CAN_WORD113_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD113_DATA_BYTE_5_MASK   (0xFF << CAN_WORD113_DATA_BYTE_5_SHIFT)
#define CAN_WORD113_DATA_BYTE_5(n)     (((n) << CAN_WORD113_DATA_BYTE_5_SHIFT) & CAN_WORD113_DATA_BYTE_5_MASK)
#define CAN_WORD113_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD113_DATA_BYTE_4_MASK   (0xFF << CAN_WORD113_DATA_BYTE_4_SHIFT)
#define CAN_WORD113_DATA_BYTE_4(n)     (((n) << CAN_WORD113_DATA_BYTE_4_SHIFT) & CAN_WORD113_DATA_BYTE_4_MASK)

/* Message Buffer 14 CS Register (CS14) */
#define CAN_CS14_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS14_TIME_STAMP_MASK   (0xFFFF << CAN_CS14_TIME_STAMP_SHIFT)
#define CAN_CS14_TIME_STAMP(n)     (((n) << CAN_CS14_TIME_STAMP_SHIFT) & CAN_CS14_TIME_STAMP_MASK)
#define CAN_CS14_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS14_DLC_MASK          (0xF << CAN_CS14_DLC_SHIFT)
#define CAN_CS14_DLC(n)            (((n) << CAN_CS14_DLC_SHIFT) & CAN_CS14_DLC_MASK)
#define CAN_CS14_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS14_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS14_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS14_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS14_CODE_MASK         (0xF << CAN_CS14_CODE_SHIFT)
#define CAN_CS14_CODE(n)           (((n) << CAN_CS14_CODE_SHIFT) & CAN_CS14_CODE_MASK)
#define CAN_CS14_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS14_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS14_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 14 WORD0 Register (WORD014) */
#define CAN_WORD014_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD014_DATA_BYTE_3_MASK   (0xFF << CAN_WORD014_DATA_BYTE_3_SHIFT)
#define CAN_WORD014_DATA_BYTE_3(n)     (((n) << CAN_WORD014_DATA_BYTE_3_SHIFT) & CAN_WORD014_DATA_BYTE_3_MASK)
#define CAN_WORD014_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD014_DATA_BYTE_2_MASK   (0xFF << CAN_WORD014_DATA_BYTE_2_SHIFT)
#define CAN_WORD014_DATA_BYTE_2(n)     (((n) << CAN_WORD014_DATA_BYTE_2_SHIFT) & CAN_WORD014_DATA_BYTE_2_MASK)
#define CAN_WORD014_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD014_DATA_BYTE_1_MASK   (0xFF << CAN_WORD014_DATA_BYTE_1_SHIFT)
#define CAN_WORD014_DATA_BYTE_1(n)     (((n) << CAN_WORD014_DATA_BYTE_1_SHIFT) & CAN_WORD014_DATA_BYTE_1_MASK)
#define CAN_WORD014_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD014_DATA_BYTE_0_MASK   (0xFF << CAN_WORD014_DATA_BYTE_0_SHIFT)
#define CAN_WORD014_DATA_BYTE_0(n)     (((n) << CAN_WORD014_DATA_BYTE_0_SHIFT) & CAN_WORD014_DATA_BYTE_0_MASK)

/* Message Buffer 14 WORD1 Register (WORD114) */
#define CAN_WORD114_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD114_DATA_BYTE_7_MASK   (0xFF << CAN_WORD114_DATA_BYTE_7_SHIFT)
#define CAN_WORD114_DATA_BYTE_7(n)     (((n) << CAN_WORD114_DATA_BYTE_7_SHIFT) & CAN_WORD114_DATA_BYTE_7_MASK)
#define CAN_WORD114_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD114_DATA_BYTE_6_MASK   (0xFF << CAN_WORD114_DATA_BYTE_6_SHIFT)
#define CAN_WORD114_DATA_BYTE_6(n)     (((n) << CAN_WORD114_DATA_BYTE_6_SHIFT) & CAN_WORD114_DATA_BYTE_6_MASK)
#define CAN_WORD114_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD114_DATA_BYTE_5_MASK   (0xFF << CAN_WORD114_DATA_BYTE_5_SHIFT)
#define CAN_WORD114_DATA_BYTE_5(n)     (((n) << CAN_WORD114_DATA_BYTE_5_SHIFT) & CAN_WORD114_DATA_BYTE_5_MASK)
#define CAN_WORD114_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD114_DATA_BYTE_4_MASK   (0xFF << CAN_WORD114_DATA_BYTE_4_SHIFT)
#define CAN_WORD114_DATA_BYTE_4(n)     (((n) << CAN_WORD114_DATA_BYTE_4_SHIFT) & CAN_WORD114_DATA_BYTE_4_MASK)

/* Message Buffer 15 CS Register (CS15) */
#define CAN_CS15_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS15_TIME_STAMP_MASK   (0xFFFF << CAN_CS15_TIME_STAMP_SHIFT)
#define CAN_CS15_TIME_STAMP(n)     (((n) << CAN_CS15_TIME_STAMP_SHIFT) & CAN_CS15_TIME_STAMP_MASK)
#define CAN_CS15_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS15_DLC_MASK          (0xF << CAN_CS15_DLC_SHIFT)
#define CAN_CS15_DLC(n)            (((n) << CAN_CS15_DLC_SHIFT) & CAN_CS15_DLC_MASK)
#define CAN_CS15_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS15_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS15_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS15_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS15_CODE_MASK         (0xF << CAN_CS15_CODE_SHIFT)
#define CAN_CS15_CODE(n)           (((n) << CAN_CS15_CODE_SHIFT) & CAN_CS15_CODE_MASK)
#define CAN_CS15_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS15_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS15_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 15 WORD0 Register (WORD015) */
#define CAN_WORD015_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD015_DATA_BYTE_3_MASK   (0xFF << CAN_WORD015_DATA_BYTE_3_SHIFT)
#define CAN_WORD015_DATA_BYTE_3(n)     (((n) << CAN_WORD015_DATA_BYTE_3_SHIFT) & CAN_WORD015_DATA_BYTE_3_MASK)
#define CAN_WORD015_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD015_DATA_BYTE_2_MASK   (0xFF << CAN_WORD015_DATA_BYTE_2_SHIFT)
#define CAN_WORD015_DATA_BYTE_2(n)     (((n) << CAN_WORD015_DATA_BYTE_2_SHIFT) & CAN_WORD015_DATA_BYTE_2_MASK)
#define CAN_WORD015_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD015_DATA_BYTE_1_MASK   (0xFF << CAN_WORD015_DATA_BYTE_1_SHIFT)
#define CAN_WORD015_DATA_BYTE_1(n)     (((n) << CAN_WORD015_DATA_BYTE_1_SHIFT) & CAN_WORD015_DATA_BYTE_1_MASK)
#define CAN_WORD015_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD015_DATA_BYTE_0_MASK   (0xFF << CAN_WORD015_DATA_BYTE_0_SHIFT)
#define CAN_WORD015_DATA_BYTE_0(n)     (((n) << CAN_WORD015_DATA_BYTE_0_SHIFT) & CAN_WORD015_DATA_BYTE_0_MASK)

/* Message Buffer 15 WORD1 Register (WORD115) */
#define CAN_WORD115_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD115_DATA_BYTE_7_MASK   (0xFF << CAN_WORD115_DATA_BYTE_7_SHIFT)
#define CAN_WORD115_DATA_BYTE_7(n)     (((n) << CAN_WORD115_DATA_BYTE_7_SHIFT) & CAN_WORD115_DATA_BYTE_7_MASK)
#define CAN_WORD115_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD115_DATA_BYTE_6_MASK   (0xFF << CAN_WORD115_DATA_BYTE_6_SHIFT)
#define CAN_WORD115_DATA_BYTE_6(n)     (((n) << CAN_WORD115_DATA_BYTE_6_SHIFT) & CAN_WORD115_DATA_BYTE_6_MASK)
#define CAN_WORD115_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD115_DATA_BYTE_5_MASK   (0xFF << CAN_WORD115_DATA_BYTE_5_SHIFT)
#define CAN_WORD115_DATA_BYTE_5(n)     (((n) << CAN_WORD115_DATA_BYTE_5_SHIFT) & CAN_WORD115_DATA_BYTE_5_MASK)
#define CAN_WORD115_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD115_DATA_BYTE_4_MASK   (0xFF << CAN_WORD115_DATA_BYTE_4_SHIFT)
#define CAN_WORD115_DATA_BYTE_4(n)     (((n) << CAN_WORD115_DATA_BYTE_4_SHIFT) & CAN_WORD115_DATA_BYTE_4_MASK)

/* Message Buffer 16 CS Register (CS16) */
#define CAN_CS16_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS16_TIME_STAMP_MASK   (0xFFFF << CAN_CS16_TIME_STAMP_SHIFT)
#define CAN_CS16_TIME_STAMP(n)     (((n) << CAN_CS16_TIME_STAMP_SHIFT) & CAN_CS16_TIME_STAMP_MASK)
#define CAN_CS16_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS16_DLC_MASK          (0xF << CAN_CS16_DLC_SHIFT)
#define CAN_CS16_DLC(n)            (((n) << CAN_CS16_DLC_SHIFT) & CAN_CS16_DLC_MASK)
#define CAN_CS16_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS16_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS16_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS16_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS16_CODE_MASK         (0xF << CAN_CS16_CODE_SHIFT)
#define CAN_CS16_CODE(n)           (((n) << CAN_CS16_CODE_SHIFT) & CAN_CS16_CODE_MASK)
#define CAN_CS16_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS16_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS16_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 16 WORD0 Register (WORD016) */
#define CAN_WORD016_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD016_DATA_BYTE_3_MASK   (0xFF << CAN_WORD016_DATA_BYTE_3_SHIFT)
#define CAN_WORD016_DATA_BYTE_3(n)     (((n) << CAN_WORD016_DATA_BYTE_3_SHIFT) & CAN_WORD016_DATA_BYTE_3_MASK)
#define CAN_WORD016_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD016_DATA_BYTE_2_MASK   (0xFF << CAN_WORD016_DATA_BYTE_2_SHIFT)
#define CAN_WORD016_DATA_BYTE_2(n)     (((n) << CAN_WORD016_DATA_BYTE_2_SHIFT) & CAN_WORD016_DATA_BYTE_2_MASK)
#define CAN_WORD016_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD016_DATA_BYTE_1_MASK   (0xFF << CAN_WORD016_DATA_BYTE_1_SHIFT)
#define CAN_WORD016_DATA_BYTE_1(n)     (((n) << CAN_WORD016_DATA_BYTE_1_SHIFT) & CAN_WORD016_DATA_BYTE_1_MASK)
#define CAN_WORD016_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD016_DATA_BYTE_0_MASK   (0xFF << CAN_WORD016_DATA_BYTE_0_SHIFT)
#define CAN_WORD016_DATA_BYTE_0(n)     (((n) << CAN_WORD016_DATA_BYTE_0_SHIFT) & CAN_WORD016_DATA_BYTE_0_MASK)

/* Message Buffer 16 WORD1 Register (WORD116) */
#define CAN_WORD116_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD116_DATA_BYTE_7_MASK   (0xFF << CAN_WORD116_DATA_BYTE_7_SHIFT)
#define CAN_WORD116_DATA_BYTE_7(n)     (((n) << CAN_WORD116_DATA_BYTE_7_SHIFT) & CAN_WORD116_DATA_BYTE_7_MASK)
#define CAN_WORD116_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD116_DATA_BYTE_6_MASK   (0xFF << CAN_WORD116_DATA_BYTE_6_SHIFT)
#define CAN_WORD116_DATA_BYTE_6(n)     (((n) << CAN_WORD116_DATA_BYTE_6_SHIFT) & CAN_WORD116_DATA_BYTE_6_MASK)
#define CAN_WORD116_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD116_DATA_BYTE_5_MASK   (0xFF << CAN_WORD116_DATA_BYTE_5_SHIFT)
#define CAN_WORD116_DATA_BYTE_5(n)     (((n) << CAN_WORD116_DATA_BYTE_5_SHIFT) & CAN_WORD116_DATA_BYTE_5_MASK)
#define CAN_WORD116_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD116_DATA_BYTE_4_MASK   (0xFF << CAN_WORD116_DATA_BYTE_4_SHIFT)
#define CAN_WORD116_DATA_BYTE_4(n)     (((n) << CAN_WORD116_DATA_BYTE_4_SHIFT) & CAN_WORD116_DATA_BYTE_4_MASK)

/* Message Buffer 17 CS Register (CS17) */
#define CAN_CS17_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS17_TIME_STAMP_MASK   (0xFFFF << CAN_CS17_TIME_STAMP_SHIFT)
#define CAN_CS17_TIME_STAMP(n)     (((n) << CAN_CS17_TIME_STAMP_SHIFT) & CAN_CS17_TIME_STAMP_MASK)
#define CAN_CS17_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS17_DLC_MASK          (0xF << CAN_CS17_DLC_SHIFT)
#define CAN_CS17_DLC(n)            (((n) << CAN_CS17_DLC_SHIFT) & CAN_CS17_DLC_MASK)
#define CAN_CS17_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS17_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS17_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS17_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS17_CODE_MASK         (0xF << CAN_CS17_CODE_SHIFT)
#define CAN_CS17_CODE(n)           (((n) << CAN_CS17_CODE_SHIFT) & CAN_CS17_CODE_MASK)
#define CAN_CS17_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS17_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS17_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 17 WORD0 Register (WORD017) */
#define CAN_WORD017_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD017_DATA_BYTE_3_MASK   (0xFF << CAN_WORD017_DATA_BYTE_3_SHIFT)
#define CAN_WORD017_DATA_BYTE_3(n)     (((n) << CAN_WORD017_DATA_BYTE_3_SHIFT) & CAN_WORD017_DATA_BYTE_3_MASK)
#define CAN_WORD017_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD017_DATA_BYTE_2_MASK   (0xFF << CAN_WORD017_DATA_BYTE_2_SHIFT)
#define CAN_WORD017_DATA_BYTE_2(n)     (((n) << CAN_WORD017_DATA_BYTE_2_SHIFT) & CAN_WORD017_DATA_BYTE_2_MASK)
#define CAN_WORD017_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD017_DATA_BYTE_1_MASK   (0xFF << CAN_WORD017_DATA_BYTE_1_SHIFT)
#define CAN_WORD017_DATA_BYTE_1(n)     (((n) << CAN_WORD017_DATA_BYTE_1_SHIFT) & CAN_WORD017_DATA_BYTE_1_MASK)
#define CAN_WORD017_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD017_DATA_BYTE_0_MASK   (0xFF << CAN_WORD017_DATA_BYTE_0_SHIFT)
#define CAN_WORD017_DATA_BYTE_0(n)     (((n) << CAN_WORD017_DATA_BYTE_0_SHIFT) & CAN_WORD017_DATA_BYTE_0_MASK)

/* Message Buffer 17 WORD1 Register (WORD117) */
#define CAN_WORD117_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD117_DATA_BYTE_7_MASK   (0xFF << CAN_WORD117_DATA_BYTE_7_SHIFT)
#define CAN_WORD117_DATA_BYTE_7(n)     (((n) << CAN_WORD117_DATA_BYTE_7_SHIFT) & CAN_WORD117_DATA_BYTE_7_MASK)
#define CAN_WORD117_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD117_DATA_BYTE_6_MASK   (0xFF << CAN_WORD117_DATA_BYTE_6_SHIFT)
#define CAN_WORD117_DATA_BYTE_6(n)     (((n) << CAN_WORD117_DATA_BYTE_6_SHIFT) & CAN_WORD117_DATA_BYTE_6_MASK)
#define CAN_WORD117_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD117_DATA_BYTE_5_MASK   (0xFF << CAN_WORD117_DATA_BYTE_5_SHIFT)
#define CAN_WORD117_DATA_BYTE_5(n)     (((n) << CAN_WORD117_DATA_BYTE_5_SHIFT) & CAN_WORD117_DATA_BYTE_5_MASK)
#define CAN_WORD117_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD117_DATA_BYTE_4_MASK   (0xFF << CAN_WORD117_DATA_BYTE_4_SHIFT)
#define CAN_WORD117_DATA_BYTE_4(n)     (((n) << CAN_WORD117_DATA_BYTE_4_SHIFT) & CAN_WORD117_DATA_BYTE_4_MASK)

/* Message Buffer 18 CS Register (CS18) */
#define CAN_CS18_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS18_TIME_STAMP_MASK   (0xFFFF << CAN_CS18_TIME_STAMP_SHIFT)
#define CAN_CS18_TIME_STAMP(n)     (((n) << CAN_CS18_TIME_STAMP_SHIFT) & CAN_CS18_TIME_STAMP_MASK)
#define CAN_CS18_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS18_DLC_MASK          (0xF << CAN_CS18_DLC_SHIFT)
#define CAN_CS18_DLC(n)            (((n) << CAN_CS18_DLC_SHIFT) & CAN_CS18_DLC_MASK)
#define CAN_CS18_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS18_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS18_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS18_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS18_CODE_MASK         (0xF << CAN_CS18_CODE_SHIFT)
#define CAN_CS18_CODE(n)           (((n) << CAN_CS18_CODE_SHIFT) & CAN_CS18_CODE_MASK)
#define CAN_CS18_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS18_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS18_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 18 WORD0 Register (WORD018) */
#define CAN_WORD018_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD018_DATA_BYTE_3_MASK   (0xFF << CAN_WORD018_DATA_BYTE_3_SHIFT)
#define CAN_WORD018_DATA_BYTE_3(n)     (((n) << CAN_WORD018_DATA_BYTE_3_SHIFT) & CAN_WORD018_DATA_BYTE_3_MASK)
#define CAN_WORD018_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD018_DATA_BYTE_2_MASK   (0xFF << CAN_WORD018_DATA_BYTE_2_SHIFT)
#define CAN_WORD018_DATA_BYTE_2(n)     (((n) << CAN_WORD018_DATA_BYTE_2_SHIFT) & CAN_WORD018_DATA_BYTE_2_MASK)
#define CAN_WORD018_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD018_DATA_BYTE_1_MASK   (0xFF << CAN_WORD018_DATA_BYTE_1_SHIFT)
#define CAN_WORD018_DATA_BYTE_1(n)     (((n) << CAN_WORD018_DATA_BYTE_1_SHIFT) & CAN_WORD018_DATA_BYTE_1_MASK)
#define CAN_WORD018_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD018_DATA_BYTE_0_MASK   (0xFF << CAN_WORD018_DATA_BYTE_0_SHIFT)
#define CAN_WORD018_DATA_BYTE_0(n)     (((n) << CAN_WORD018_DATA_BYTE_0_SHIFT) & CAN_WORD018_DATA_BYTE_0_MASK)

/* Message Buffer 18 WORD1 Register (WORD118) */
#define CAN_WORD118_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD118_DATA_BYTE_7_MASK   (0xFF << CAN_WORD118_DATA_BYTE_7_SHIFT)
#define CAN_WORD118_DATA_BYTE_7(n)     (((n) << CAN_WORD118_DATA_BYTE_7_SHIFT) & CAN_WORD118_DATA_BYTE_7_MASK)
#define CAN_WORD118_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD118_DATA_BYTE_6_MASK   (0xFF << CAN_WORD118_DATA_BYTE_6_SHIFT)
#define CAN_WORD118_DATA_BYTE_6(n)     (((n) << CAN_WORD118_DATA_BYTE_6_SHIFT) & CAN_WORD118_DATA_BYTE_6_MASK)
#define CAN_WORD118_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD118_DATA_BYTE_5_MASK   (0xFF << CAN_WORD118_DATA_BYTE_5_SHIFT)
#define CAN_WORD118_DATA_BYTE_5(n)     (((n) << CAN_WORD118_DATA_BYTE_5_SHIFT) & CAN_WORD118_DATA_BYTE_5_MASK)
#define CAN_WORD118_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD118_DATA_BYTE_4_MASK   (0xFF << CAN_WORD118_DATA_BYTE_4_SHIFT)
#define CAN_WORD118_DATA_BYTE_4(n)     (((n) << CAN_WORD118_DATA_BYTE_4_SHIFT) & CAN_WORD118_DATA_BYTE_4_MASK)

/* Message Buffer 19 CS Register (CS19) */
#define CAN_CS19_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS19_TIME_STAMP_MASK   (0xFFFF << CAN_CS19_TIME_STAMP_SHIFT)
#define CAN_CS19_TIME_STAMP(n)     (((n) << CAN_CS19_TIME_STAMP_SHIFT) & CAN_CS19_TIME_STAMP_MASK)
#define CAN_CS19_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS19_DLC_MASK          (0xF << CAN_CS19_DLC_SHIFT)
#define CAN_CS19_DLC(n)            (((n) << CAN_CS19_DLC_SHIFT) & CAN_CS19_DLC_MASK)
#define CAN_CS19_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS19_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS19_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS19_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS19_CODE_MASK         (0xF << CAN_CS19_CODE_SHIFT)
#define CAN_CS19_CODE(n)           (((n) << CAN_CS19_CODE_SHIFT) & CAN_CS19_CODE_MASK)
#define CAN_CS19_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS19_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS19_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 19 WORD0 Register (WORD019) */
#define CAN_WORD019_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD019_DATA_BYTE_3_MASK   (0xFF << CAN_WORD019_DATA_BYTE_3_SHIFT)
#define CAN_WORD019_DATA_BYTE_3(n)     (((n) << CAN_WORD019_DATA_BYTE_3_SHIFT) & CAN_WORD019_DATA_BYTE_3_MASK)
#define CAN_WORD019_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD019_DATA_BYTE_2_MASK   (0xFF << CAN_WORD019_DATA_BYTE_2_SHIFT)
#define CAN_WORD019_DATA_BYTE_2(n)     (((n) << CAN_WORD019_DATA_BYTE_2_SHIFT) & CAN_WORD019_DATA_BYTE_2_MASK)
#define CAN_WORD019_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD019_DATA_BYTE_1_MASK   (0xFF << CAN_WORD019_DATA_BYTE_1_SHIFT)
#define CAN_WORD019_DATA_BYTE_1(n)     (((n) << CAN_WORD019_DATA_BYTE_1_SHIFT) & CAN_WORD019_DATA_BYTE_1_MASK)
#define CAN_WORD019_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD019_DATA_BYTE_0_MASK   (0xFF << CAN_WORD019_DATA_BYTE_0_SHIFT)
#define CAN_WORD019_DATA_BYTE_0(n)     (((n) << CAN_WORD019_DATA_BYTE_0_SHIFT) & CAN_WORD019_DATA_BYTE_0_MASK)

/* Message Buffer 19 WORD1 Register (WORD119) */
#define CAN_WORD119_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD119_DATA_BYTE_7_MASK   (0xFF << CAN_WORD119_DATA_BYTE_7_SHIFT)
#define CAN_WORD119_DATA_BYTE_7(n)     (((n) << CAN_WORD119_DATA_BYTE_7_SHIFT) & CAN_WORD119_DATA_BYTE_7_MASK)
#define CAN_WORD119_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD119_DATA_BYTE_6_MASK   (0xFF << CAN_WORD119_DATA_BYTE_6_SHIFT)
#define CAN_WORD119_DATA_BYTE_6(n)     (((n) << CAN_WORD119_DATA_BYTE_6_SHIFT) & CAN_WORD119_DATA_BYTE_6_MASK)
#define CAN_WORD119_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD119_DATA_BYTE_5_MASK   (0xFF << CAN_WORD119_DATA_BYTE_5_SHIFT)
#define CAN_WORD119_DATA_BYTE_5(n)     (((n) << CAN_WORD119_DATA_BYTE_5_SHIFT) & CAN_WORD119_DATA_BYTE_5_MASK)
#define CAN_WORD119_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD119_DATA_BYTE_4_MASK   (0xFF << CAN_WORD119_DATA_BYTE_4_SHIFT)
#define CAN_WORD119_DATA_BYTE_4(n)     (((n) << CAN_WORD119_DATA_BYTE_4_SHIFT) & CAN_WORD119_DATA_BYTE_4_MASK)

/* Message Buffer 20 CS Register (CS20) */
#define CAN_CS20_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS20_TIME_STAMP_MASK   (0xFFFF << CAN_CS20_TIME_STAMP_SHIFT)
#define CAN_CS20_TIME_STAMP(n)     (((n) << CAN_CS20_TIME_STAMP_SHIFT) & CAN_CS20_TIME_STAMP_MASK)
#define CAN_CS20_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS20_DLC_MASK          (0xF << CAN_CS20_DLC_SHIFT)
#define CAN_CS20_DLC(n)            (((n) << CAN_CS20_DLC_SHIFT) & CAN_CS20_DLC_MASK)
#define CAN_CS20_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS20_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS20_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS20_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS20_CODE_MASK         (0xF << CAN_CS20_CODE_SHIFT)
#define CAN_CS20_CODE(n)           (((n) << CAN_CS20_CODE_SHIFT) & CAN_CS20_CODE_MASK)
#define CAN_CS20_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS20_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS20_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 20 WORD0 Register (WORD020) */
#define CAN_WORD020_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD020_DATA_BYTE_3_MASK   (0xFF << CAN_WORD020_DATA_BYTE_3_SHIFT)
#define CAN_WORD020_DATA_BYTE_3(n)     (((n) << CAN_WORD020_DATA_BYTE_3_SHIFT) & CAN_WORD020_DATA_BYTE_3_MASK)
#define CAN_WORD020_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD020_DATA_BYTE_2_MASK   (0xFF << CAN_WORD020_DATA_BYTE_2_SHIFT)
#define CAN_WORD020_DATA_BYTE_2(n)     (((n) << CAN_WORD020_DATA_BYTE_2_SHIFT) & CAN_WORD020_DATA_BYTE_2_MASK)
#define CAN_WORD020_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD020_DATA_BYTE_1_MASK   (0xFF << CAN_WORD020_DATA_BYTE_1_SHIFT)
#define CAN_WORD020_DATA_BYTE_1(n)     (((n) << CAN_WORD020_DATA_BYTE_1_SHIFT) & CAN_WORD020_DATA_BYTE_1_MASK)
#define CAN_WORD020_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD020_DATA_BYTE_0_MASK   (0xFF << CAN_WORD020_DATA_BYTE_0_SHIFT)
#define CAN_WORD020_DATA_BYTE_0(n)     (((n) << CAN_WORD020_DATA_BYTE_0_SHIFT) & CAN_WORD020_DATA_BYTE_0_MASK)

/* Message Buffer 20 WORD1 Register (WORD120) */
#define CAN_WORD120_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD120_DATA_BYTE_7_MASK   (0xFF << CAN_WORD120_DATA_BYTE_7_SHIFT)
#define CAN_WORD120_DATA_BYTE_7(n)     (((n) << CAN_WORD120_DATA_BYTE_7_SHIFT) & CAN_WORD120_DATA_BYTE_7_MASK)
#define CAN_WORD120_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD120_DATA_BYTE_6_MASK   (0xFF << CAN_WORD120_DATA_BYTE_6_SHIFT)
#define CAN_WORD120_DATA_BYTE_6(n)     (((n) << CAN_WORD120_DATA_BYTE_6_SHIFT) & CAN_WORD120_DATA_BYTE_6_MASK)
#define CAN_WORD120_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD120_DATA_BYTE_5_MASK   (0xFF << CAN_WORD120_DATA_BYTE_5_SHIFT)
#define CAN_WORD120_DATA_BYTE_5(n)     (((n) << CAN_WORD120_DATA_BYTE_5_SHIFT) & CAN_WORD120_DATA_BYTE_5_MASK)
#define CAN_WORD120_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD120_DATA_BYTE_4_MASK   (0xFF << CAN_WORD120_DATA_BYTE_4_SHIFT)
#define CAN_WORD120_DATA_BYTE_4(n)     (((n) << CAN_WORD120_DATA_BYTE_4_SHIFT) & CAN_WORD120_DATA_BYTE_4_MASK)

/* Message Buffer 21 CS Register (CS21) */
#define CAN_CS21_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS21_TIME_STAMP_MASK   (0xFFFF << CAN_CS21_TIME_STAMP_SHIFT)
#define CAN_CS21_TIME_STAMP(n)     (((n) << CAN_CS21_TIME_STAMP_SHIFT) & CAN_CS21_TIME_STAMP_MASK)
#define CAN_CS21_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS21_DLC_MASK          (0xF << CAN_CS21_DLC_SHIFT)
#define CAN_CS21_DLC(n)            (((n) << CAN_CS21_DLC_SHIFT) & CAN_CS21_DLC_MASK)
#define CAN_CS21_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS21_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS21_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS21_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS21_CODE_MASK         (0xF << CAN_CS21_CODE_SHIFT)
#define CAN_CS21_CODE(n)           (((n) << CAN_CS21_CODE_SHIFT) & CAN_CS21_CODE_MASK)
#define CAN_CS21_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS21_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS21_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 21 WORD0 Register (WORD021) */
#define CAN_WORD021_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD021_DATA_BYTE_3_MASK   (0xFF << CAN_WORD021_DATA_BYTE_3_SHIFT)
#define CAN_WORD021_DATA_BYTE_3(n)     (((n) << CAN_WORD021_DATA_BYTE_3_SHIFT) & CAN_WORD021_DATA_BYTE_3_MASK)
#define CAN_WORD021_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD021_DATA_BYTE_2_MASK   (0xFF << CAN_WORD021_DATA_BYTE_2_SHIFT)
#define CAN_WORD021_DATA_BYTE_2(n)     (((n) << CAN_WORD021_DATA_BYTE_2_SHIFT) & CAN_WORD021_DATA_BYTE_2_MASK)
#define CAN_WORD021_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD021_DATA_BYTE_1_MASK   (0xFF << CAN_WORD021_DATA_BYTE_1_SHIFT)
#define CAN_WORD021_DATA_BYTE_1(n)     (((n) << CAN_WORD021_DATA_BYTE_1_SHIFT) & CAN_WORD021_DATA_BYTE_1_MASK)
#define CAN_WORD021_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD021_DATA_BYTE_0_MASK   (0xFF << CAN_WORD021_DATA_BYTE_0_SHIFT)
#define CAN_WORD021_DATA_BYTE_0(n)     (((n) << CAN_WORD021_DATA_BYTE_0_SHIFT) & CAN_WORD021_DATA_BYTE_0_MASK)

/* Message Buffer 21 WORD1 Register (WORD121) */
#define CAN_WORD121_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD121_DATA_BYTE_7_MASK   (0xFF << CAN_WORD121_DATA_BYTE_7_SHIFT)
#define CAN_WORD121_DATA_BYTE_7(n)     (((n) << CAN_WORD121_DATA_BYTE_7_SHIFT) & CAN_WORD121_DATA_BYTE_7_MASK)
#define CAN_WORD121_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD121_DATA_BYTE_6_MASK   (0xFF << CAN_WORD121_DATA_BYTE_6_SHIFT)
#define CAN_WORD121_DATA_BYTE_6(n)     (((n) << CAN_WORD121_DATA_BYTE_6_SHIFT) & CAN_WORD121_DATA_BYTE_6_MASK)
#define CAN_WORD121_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD121_DATA_BYTE_5_MASK   (0xFF << CAN_WORD121_DATA_BYTE_5_SHIFT)
#define CAN_WORD121_DATA_BYTE_5(n)     (((n) << CAN_WORD121_DATA_BYTE_5_SHIFT) & CAN_WORD121_DATA_BYTE_5_MASK)
#define CAN_WORD121_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD121_DATA_BYTE_4_MASK   (0xFF << CAN_WORD121_DATA_BYTE_4_SHIFT)
#define CAN_WORD121_DATA_BYTE_4(n)     (((n) << CAN_WORD121_DATA_BYTE_4_SHIFT) & CAN_WORD121_DATA_BYTE_4_MASK)

/* Message Buffer 22 CS Register (CS22) */
#define CAN_CS22_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS22_TIME_STAMP_MASK   (0xFFFF << CAN_CS22_TIME_STAMP_SHIFT)
#define CAN_CS22_TIME_STAMP(n)     (((n) << CAN_CS22_TIME_STAMP_SHIFT) & CAN_CS22_TIME_STAMP_MASK)
#define CAN_CS22_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS22_DLC_MASK          (0xF << CAN_CS22_DLC_SHIFT)
#define CAN_CS22_DLC(n)            (((n) << CAN_CS22_DLC_SHIFT) & CAN_CS22_DLC_MASK)
#define CAN_CS22_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS22_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS22_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS22_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS22_CODE_MASK         (0xF << CAN_CS22_CODE_SHIFT)
#define CAN_CS22_CODE(n)           (((n) << CAN_CS22_CODE_SHIFT) & CAN_CS22_CODE_MASK)
#define CAN_CS22_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS22_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS22_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 22 WORD0 Register (WORD022) */
#define CAN_WORD022_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD022_DATA_BYTE_3_MASK   (0xFF << CAN_WORD022_DATA_BYTE_3_SHIFT)
#define CAN_WORD022_DATA_BYTE_3(n)     (((n) << CAN_WORD022_DATA_BYTE_3_SHIFT) & CAN_WORD022_DATA_BYTE_3_MASK)
#define CAN_WORD022_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD022_DATA_BYTE_2_MASK   (0xFF << CAN_WORD022_DATA_BYTE_2_SHIFT)
#define CAN_WORD022_DATA_BYTE_2(n)     (((n) << CAN_WORD022_DATA_BYTE_2_SHIFT) & CAN_WORD022_DATA_BYTE_2_MASK)
#define CAN_WORD022_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD022_DATA_BYTE_1_MASK   (0xFF << CAN_WORD022_DATA_BYTE_1_SHIFT)
#define CAN_WORD022_DATA_BYTE_1(n)     (((n) << CAN_WORD022_DATA_BYTE_1_SHIFT) & CAN_WORD022_DATA_BYTE_1_MASK)
#define CAN_WORD022_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD022_DATA_BYTE_0_MASK   (0xFF << CAN_WORD022_DATA_BYTE_0_SHIFT)
#define CAN_WORD022_DATA_BYTE_0(n)     (((n) << CAN_WORD022_DATA_BYTE_0_SHIFT) & CAN_WORD022_DATA_BYTE_0_MASK)

/* Message Buffer 22 WORD1 Register (WORD122) */
#define CAN_WORD122_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD122_DATA_BYTE_7_MASK   (0xFF << CAN_WORD122_DATA_BYTE_7_SHIFT)
#define CAN_WORD122_DATA_BYTE_7(n)     (((n) << CAN_WORD122_DATA_BYTE_7_SHIFT) & CAN_WORD122_DATA_BYTE_7_MASK)
#define CAN_WORD122_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD122_DATA_BYTE_6_MASK   (0xFF << CAN_WORD122_DATA_BYTE_6_SHIFT)
#define CAN_WORD122_DATA_BYTE_6(n)     (((n) << CAN_WORD122_DATA_BYTE_6_SHIFT) & CAN_WORD122_DATA_BYTE_6_MASK)
#define CAN_WORD122_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD122_DATA_BYTE_5_MASK   (0xFF << CAN_WORD122_DATA_BYTE_5_SHIFT)
#define CAN_WORD122_DATA_BYTE_5(n)     (((n) << CAN_WORD122_DATA_BYTE_5_SHIFT) & CAN_WORD122_DATA_BYTE_5_MASK)
#define CAN_WORD122_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD122_DATA_BYTE_4_MASK   (0xFF << CAN_WORD122_DATA_BYTE_4_SHIFT)
#define CAN_WORD122_DATA_BYTE_4(n)     (((n) << CAN_WORD122_DATA_BYTE_4_SHIFT) & CAN_WORD122_DATA_BYTE_4_MASK)

/* Message Buffer 23 CS Register (CS23) */
#define CAN_CS23_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS23_TIME_STAMP_MASK   (0xFFFF << CAN_CS23_TIME_STAMP_SHIFT)
#define CAN_CS23_TIME_STAMP(n)     (((n) << CAN_CS23_TIME_STAMP_SHIFT) & CAN_CS23_TIME_STAMP_MASK)
#define CAN_CS23_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS23_DLC_MASK          (0xF << CAN_CS23_DLC_SHIFT)
#define CAN_CS23_DLC(n)            (((n) << CAN_CS23_DLC_SHIFT) & CAN_CS23_DLC_MASK)
#define CAN_CS23_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS23_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS23_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS23_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS23_CODE_MASK         (0xF << CAN_CS23_CODE_SHIFT)
#define CAN_CS23_CODE(n)           (((n) << CAN_CS23_CODE_SHIFT) & CAN_CS23_CODE_MASK)
#define CAN_CS23_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS23_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS23_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 23 WORD0 Register (WORD023) */
#define CAN_WORD023_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD023_DATA_BYTE_3_MASK   (0xFF << CAN_WORD023_DATA_BYTE_3_SHIFT)
#define CAN_WORD023_DATA_BYTE_3(n)     (((n) << CAN_WORD023_DATA_BYTE_3_SHIFT) & CAN_WORD023_DATA_BYTE_3_MASK)
#define CAN_WORD023_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD023_DATA_BYTE_2_MASK   (0xFF << CAN_WORD023_DATA_BYTE_2_SHIFT)
#define CAN_WORD023_DATA_BYTE_2(n)     (((n) << CAN_WORD023_DATA_BYTE_2_SHIFT) & CAN_WORD023_DATA_BYTE_2_MASK)
#define CAN_WORD023_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD023_DATA_BYTE_1_MASK   (0xFF << CAN_WORD023_DATA_BYTE_1_SHIFT)
#define CAN_WORD023_DATA_BYTE_1(n)     (((n) << CAN_WORD023_DATA_BYTE_1_SHIFT) & CAN_WORD023_DATA_BYTE_1_MASK)
#define CAN_WORD023_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD023_DATA_BYTE_0_MASK   (0xFF << CAN_WORD023_DATA_BYTE_0_SHIFT)
#define CAN_WORD023_DATA_BYTE_0(n)     (((n) << CAN_WORD023_DATA_BYTE_0_SHIFT) & CAN_WORD023_DATA_BYTE_0_MASK)

/* Message Buffer 23 WORD1 Register (WORD123) */
#define CAN_WORD123_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD123_DATA_BYTE_7_MASK   (0xFF << CAN_WORD123_DATA_BYTE_7_SHIFT)
#define CAN_WORD123_DATA_BYTE_7(n)     (((n) << CAN_WORD123_DATA_BYTE_7_SHIFT) & CAN_WORD123_DATA_BYTE_7_MASK)
#define CAN_WORD123_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD123_DATA_BYTE_6_MASK   (0xFF << CAN_WORD123_DATA_BYTE_6_SHIFT)
#define CAN_WORD123_DATA_BYTE_6(n)     (((n) << CAN_WORD123_DATA_BYTE_6_SHIFT) & CAN_WORD123_DATA_BYTE_6_MASK)
#define CAN_WORD123_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD123_DATA_BYTE_5_MASK   (0xFF << CAN_WORD123_DATA_BYTE_5_SHIFT)
#define CAN_WORD123_DATA_BYTE_5(n)     (((n) << CAN_WORD123_DATA_BYTE_5_SHIFT) & CAN_WORD123_DATA_BYTE_5_MASK)
#define CAN_WORD123_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD123_DATA_BYTE_4_MASK   (0xFF << CAN_WORD123_DATA_BYTE_4_SHIFT)
#define CAN_WORD123_DATA_BYTE_4(n)     (((n) << CAN_WORD123_DATA_BYTE_4_SHIFT) & CAN_WORD123_DATA_BYTE_4_MASK)

/* Message Buffer 24 CS Register (CS24) */
#define CAN_CS24_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS24_TIME_STAMP_MASK   (0xFFFF << CAN_CS24_TIME_STAMP_SHIFT)
#define CAN_CS24_TIME_STAMP(n)     (((n) << CAN_CS24_TIME_STAMP_SHIFT) & CAN_CS24_TIME_STAMP_MASK)
#define CAN_CS24_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS24_DLC_MASK          (0xF << CAN_CS24_DLC_SHIFT)
#define CAN_CS24_DLC(n)            (((n) << CAN_CS24_DLC_SHIFT) & CAN_CS24_DLC_MASK)
#define CAN_CS24_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS24_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS24_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS24_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS24_CODE_MASK         (0xF << CAN_CS24_CODE_SHIFT)
#define CAN_CS24_CODE(n)           (((n) << CAN_CS24_CODE_SHIFT) & CAN_CS24_CODE_MASK)
#define CAN_CS24_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS24_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS24_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 24 WORD0 Register (WORD024) */
#define CAN_WORD024_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD024_DATA_BYTE_3_MASK   (0xFF << CAN_WORD024_DATA_BYTE_3_SHIFT)
#define CAN_WORD024_DATA_BYTE_3(n)     (((n) << CAN_WORD024_DATA_BYTE_3_SHIFT) & CAN_WORD024_DATA_BYTE_3_MASK)
#define CAN_WORD024_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD024_DATA_BYTE_2_MASK   (0xFF << CAN_WORD024_DATA_BYTE_2_SHIFT)
#define CAN_WORD024_DATA_BYTE_2(n)     (((n) << CAN_WORD024_DATA_BYTE_2_SHIFT) & CAN_WORD024_DATA_BYTE_2_MASK)
#define CAN_WORD024_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD024_DATA_BYTE_1_MASK   (0xFF << CAN_WORD024_DATA_BYTE_1_SHIFT)
#define CAN_WORD024_DATA_BYTE_1(n)     (((n) << CAN_WORD024_DATA_BYTE_1_SHIFT) & CAN_WORD024_DATA_BYTE_1_MASK)
#define CAN_WORD024_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD024_DATA_BYTE_0_MASK   (0xFF << CAN_WORD024_DATA_BYTE_0_SHIFT)
#define CAN_WORD024_DATA_BYTE_0(n)     (((n) << CAN_WORD024_DATA_BYTE_0_SHIFT) & CAN_WORD024_DATA_BYTE_0_MASK)

/* Message Buffer 24 WORD1 Register (WORD124) */
#define CAN_WORD124_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD124_DATA_BYTE_7_MASK   (0xFF << CAN_WORD124_DATA_BYTE_7_SHIFT)
#define CAN_WORD124_DATA_BYTE_7(n)     (((n) << CAN_WORD124_DATA_BYTE_7_SHIFT) & CAN_WORD124_DATA_BYTE_7_MASK)
#define CAN_WORD124_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD124_DATA_BYTE_6_MASK   (0xFF << CAN_WORD124_DATA_BYTE_6_SHIFT)
#define CAN_WORD124_DATA_BYTE_6(n)     (((n) << CAN_WORD124_DATA_BYTE_6_SHIFT) & CAN_WORD124_DATA_BYTE_6_MASK)
#define CAN_WORD124_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD124_DATA_BYTE_5_MASK   (0xFF << CAN_WORD124_DATA_BYTE_5_SHIFT)
#define CAN_WORD124_DATA_BYTE_5(n)     (((n) << CAN_WORD124_DATA_BYTE_5_SHIFT) & CAN_WORD124_DATA_BYTE_5_MASK)
#define CAN_WORD124_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD124_DATA_BYTE_4_MASK   (0xFF << CAN_WORD124_DATA_BYTE_4_SHIFT)
#define CAN_WORD124_DATA_BYTE_4(n)     (((n) << CAN_WORD124_DATA_BYTE_4_SHIFT) & CAN_WORD124_DATA_BYTE_4_MASK)

/* Message Buffer 25 CS Register (CS25) */
#define CAN_CS25_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS25_TIME_STAMP_MASK   (0xFFFF << CAN_CS25_TIME_STAMP_SHIFT)
#define CAN_CS25_TIME_STAMP(n)     (((n) << CAN_CS25_TIME_STAMP_SHIFT) & CAN_CS25_TIME_STAMP_MASK)
#define CAN_CS25_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS25_DLC_MASK          (0xF << CAN_CS25_DLC_SHIFT)
#define CAN_CS25_DLC(n)            (((n) << CAN_CS25_DLC_SHIFT) & CAN_CS25_DLC_MASK)
#define CAN_CS25_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS25_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS25_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS25_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS25_CODE_MASK         (0xF << CAN_CS25_CODE_SHIFT)
#define CAN_CS25_CODE(n)           (((n) << CAN_CS25_CODE_SHIFT) & CAN_CS25_CODE_MASK)
#define CAN_CS25_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS25_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS25_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 25 WORD0 Register (WORD025) */
#define CAN_WORD025_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD025_DATA_BYTE_3_MASK   (0xFF << CAN_WORD025_DATA_BYTE_3_SHIFT)
#define CAN_WORD025_DATA_BYTE_3(n)     (((n) << CAN_WORD025_DATA_BYTE_3_SHIFT) & CAN_WORD025_DATA_BYTE_3_MASK)
#define CAN_WORD025_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD025_DATA_BYTE_2_MASK   (0xFF << CAN_WORD025_DATA_BYTE_2_SHIFT)
#define CAN_WORD025_DATA_BYTE_2(n)     (((n) << CAN_WORD025_DATA_BYTE_2_SHIFT) & CAN_WORD025_DATA_BYTE_2_MASK)
#define CAN_WORD025_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD025_DATA_BYTE_1_MASK   (0xFF << CAN_WORD025_DATA_BYTE_1_SHIFT)
#define CAN_WORD025_DATA_BYTE_1(n)     (((n) << CAN_WORD025_DATA_BYTE_1_SHIFT) & CAN_WORD025_DATA_BYTE_1_MASK)
#define CAN_WORD025_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD025_DATA_BYTE_0_MASK   (0xFF << CAN_WORD025_DATA_BYTE_0_SHIFT)
#define CAN_WORD025_DATA_BYTE_0(n)     (((n) << CAN_WORD025_DATA_BYTE_0_SHIFT) & CAN_WORD025_DATA_BYTE_0_MASK)

/* Message Buffer 25 WORD1 Register (WORD125) */
#define CAN_WORD125_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD125_DATA_BYTE_7_MASK   (0xFF << CAN_WORD125_DATA_BYTE_7_SHIFT)
#define CAN_WORD125_DATA_BYTE_7(n)     (((n) << CAN_WORD125_DATA_BYTE_7_SHIFT) & CAN_WORD125_DATA_BYTE_7_MASK)
#define CAN_WORD125_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD125_DATA_BYTE_6_MASK   (0xFF << CAN_WORD125_DATA_BYTE_6_SHIFT)
#define CAN_WORD125_DATA_BYTE_6(n)     (((n) << CAN_WORD125_DATA_BYTE_6_SHIFT) & CAN_WORD125_DATA_BYTE_6_MASK)
#define CAN_WORD125_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD125_DATA_BYTE_5_MASK   (0xFF << CAN_WORD125_DATA_BYTE_5_SHIFT)
#define CAN_WORD125_DATA_BYTE_5(n)     (((n) << CAN_WORD125_DATA_BYTE_5_SHIFT) & CAN_WORD125_DATA_BYTE_5_MASK)
#define CAN_WORD125_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD125_DATA_BYTE_4_MASK   (0xFF << CAN_WORD125_DATA_BYTE_4_SHIFT)
#define CAN_WORD125_DATA_BYTE_4(n)     (((n) << CAN_WORD125_DATA_BYTE_4_SHIFT) & CAN_WORD125_DATA_BYTE_4_MASK)

/* Message Buffer 26 CS Register (CS26) */
#define CAN_CS26_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS26_TIME_STAMP_MASK   (0xFFFF << CAN_CS26_TIME_STAMP_SHIFT)
#define CAN_CS26_TIME_STAMP(n)     (((n) << CAN_CS26_TIME_STAMP_SHIFT) & CAN_CS26_TIME_STAMP_MASK)
#define CAN_CS26_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS26_DLC_MASK          (0xF << CAN_CS26_DLC_SHIFT)
#define CAN_CS26_DLC(n)            (((n) << CAN_CS26_DLC_SHIFT) & CAN_CS26_DLC_MASK)
#define CAN_CS26_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS26_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS26_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS26_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS26_CODE_MASK         (0xF << CAN_CS26_CODE_SHIFT)
#define CAN_CS26_CODE(n)           (((n) << CAN_CS26_CODE_SHIFT) & CAN_CS26_CODE_MASK)
#define CAN_CS26_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS26_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS26_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 26 WORD0 Register (WORD026) */
#define CAN_WORD026_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD026_DATA_BYTE_3_MASK   (0xFF << CAN_WORD026_DATA_BYTE_3_SHIFT)
#define CAN_WORD026_DATA_BYTE_3(n)     (((n) << CAN_WORD026_DATA_BYTE_3_SHIFT) & CAN_WORD026_DATA_BYTE_3_MASK)
#define CAN_WORD026_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD026_DATA_BYTE_2_MASK   (0xFF << CAN_WORD026_DATA_BYTE_2_SHIFT)
#define CAN_WORD026_DATA_BYTE_2(n)     (((n) << CAN_WORD026_DATA_BYTE_2_SHIFT) & CAN_WORD026_DATA_BYTE_2_MASK)
#define CAN_WORD026_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD026_DATA_BYTE_1_MASK   (0xFF << CAN_WORD026_DATA_BYTE_1_SHIFT)
#define CAN_WORD026_DATA_BYTE_1(n)     (((n) << CAN_WORD026_DATA_BYTE_1_SHIFT) & CAN_WORD026_DATA_BYTE_1_MASK)
#define CAN_WORD026_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD026_DATA_BYTE_0_MASK   (0xFF << CAN_WORD026_DATA_BYTE_0_SHIFT)
#define CAN_WORD026_DATA_BYTE_0(n)     (((n) << CAN_WORD026_DATA_BYTE_0_SHIFT) & CAN_WORD026_DATA_BYTE_0_MASK)

/* Message Buffer 26 WORD1 Register (WORD126) */
#define CAN_WORD126_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD126_DATA_BYTE_7_MASK   (0xFF << CAN_WORD126_DATA_BYTE_7_SHIFT)
#define CAN_WORD126_DATA_BYTE_7(n)     (((n) << CAN_WORD126_DATA_BYTE_7_SHIFT) & CAN_WORD126_DATA_BYTE_7_MASK)
#define CAN_WORD126_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD126_DATA_BYTE_6_MASK   (0xFF << CAN_WORD126_DATA_BYTE_6_SHIFT)
#define CAN_WORD126_DATA_BYTE_6(n)     (((n) << CAN_WORD126_DATA_BYTE_6_SHIFT) & CAN_WORD126_DATA_BYTE_6_MASK)
#define CAN_WORD126_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD126_DATA_BYTE_5_MASK   (0xFF << CAN_WORD126_DATA_BYTE_5_SHIFT)
#define CAN_WORD126_DATA_BYTE_5(n)     (((n) << CAN_WORD126_DATA_BYTE_5_SHIFT) & CAN_WORD126_DATA_BYTE_5_MASK)
#define CAN_WORD126_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD126_DATA_BYTE_4_MASK   (0xFF << CAN_WORD126_DATA_BYTE_4_SHIFT)
#define CAN_WORD126_DATA_BYTE_4(n)     (((n) << CAN_WORD126_DATA_BYTE_4_SHIFT) & CAN_WORD126_DATA_BYTE_4_MASK)

/* Message Buffer 27 CS Register (CS27) */
#define CAN_CS27_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS27_TIME_STAMP_MASK   (0xFFFF << CAN_CS27_TIME_STAMP_SHIFT)
#define CAN_CS27_TIME_STAMP(n)     (((n) << CAN_CS27_TIME_STAMP_SHIFT) & CAN_CS27_TIME_STAMP_MASK)
#define CAN_CS27_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS27_DLC_MASK          (0xF << CAN_CS27_DLC_SHIFT)
#define CAN_CS27_DLC(n)            (((n) << CAN_CS27_DLC_SHIFT) & CAN_CS27_DLC_MASK)
#define CAN_CS27_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS27_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS27_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS27_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS27_CODE_MASK         (0xF << CAN_CS27_CODE_SHIFT)
#define CAN_CS27_CODE(n)           (((n) << CAN_CS27_CODE_SHIFT) & CAN_CS27_CODE_MASK)
#define CAN_CS27_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS27_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS27_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 27 WORD0 Register (WORD027) */
#define CAN_WORD027_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD027_DATA_BYTE_3_MASK   (0xFF << CAN_WORD027_DATA_BYTE_3_SHIFT)
#define CAN_WORD027_DATA_BYTE_3(n)     (((n) << CAN_WORD027_DATA_BYTE_3_SHIFT) & CAN_WORD027_DATA_BYTE_3_MASK)
#define CAN_WORD027_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD027_DATA_BYTE_2_MASK   (0xFF << CAN_WORD027_DATA_BYTE_2_SHIFT)
#define CAN_WORD027_DATA_BYTE_2(n)     (((n) << CAN_WORD027_DATA_BYTE_2_SHIFT) & CAN_WORD027_DATA_BYTE_2_MASK)
#define CAN_WORD027_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD027_DATA_BYTE_1_MASK   (0xFF << CAN_WORD027_DATA_BYTE_1_SHIFT)
#define CAN_WORD027_DATA_BYTE_1(n)     (((n) << CAN_WORD027_DATA_BYTE_1_SHIFT) & CAN_WORD027_DATA_BYTE_1_MASK)
#define CAN_WORD027_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD027_DATA_BYTE_0_MASK   (0xFF << CAN_WORD027_DATA_BYTE_0_SHIFT)
#define CAN_WORD027_DATA_BYTE_0(n)     (((n) << CAN_WORD027_DATA_BYTE_0_SHIFT) & CAN_WORD027_DATA_BYTE_0_MASK)

/* Message Buffer 27 WORD1 Register (WORD127) */
#define CAN_WORD127_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD127_DATA_BYTE_7_MASK   (0xFF << CAN_WORD127_DATA_BYTE_7_SHIFT)
#define CAN_WORD127_DATA_BYTE_7(n)     (((n) << CAN_WORD127_DATA_BYTE_7_SHIFT) & CAN_WORD127_DATA_BYTE_7_MASK)
#define CAN_WORD127_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD127_DATA_BYTE_6_MASK   (0xFF << CAN_WORD127_DATA_BYTE_6_SHIFT)
#define CAN_WORD127_DATA_BYTE_6(n)     (((n) << CAN_WORD127_DATA_BYTE_6_SHIFT) & CAN_WORD127_DATA_BYTE_6_MASK)
#define CAN_WORD127_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD127_DATA_BYTE_5_MASK   (0xFF << CAN_WORD127_DATA_BYTE_5_SHIFT)
#define CAN_WORD127_DATA_BYTE_5(n)     (((n) << CAN_WORD127_DATA_BYTE_5_SHIFT) & CAN_WORD127_DATA_BYTE_5_MASK)
#define CAN_WORD127_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD127_DATA_BYTE_4_MASK   (0xFF << CAN_WORD127_DATA_BYTE_4_SHIFT)
#define CAN_WORD127_DATA_BYTE_4(n)     (((n) << CAN_WORD127_DATA_BYTE_4_SHIFT) & CAN_WORD127_DATA_BYTE_4_MASK)

/* Message Buffer 28 CS Register (CS28) */
#define CAN_CS28_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS28_TIME_STAMP_MASK   (0xFFFF << CAN_CS28_TIME_STAMP_SHIFT)
#define CAN_CS28_TIME_STAMP(n)     (((n) << CAN_CS28_TIME_STAMP_SHIFT) & CAN_CS28_TIME_STAMP_MASK)
#define CAN_CS28_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS28_DLC_MASK          (0xF << CAN_CS28_DLC_SHIFT)
#define CAN_CS28_DLC(n)            (((n) << CAN_CS28_DLC_SHIFT) & CAN_CS28_DLC_MASK)
#define CAN_CS28_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS28_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS28_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS28_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS28_CODE_MASK         (0xF << CAN_CS28_CODE_SHIFT)
#define CAN_CS28_CODE(n)           (((n) << CAN_CS28_CODE_SHIFT) & CAN_CS28_CODE_MASK)
#define CAN_CS28_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS28_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS28_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 28 WORD0 Register (WORD028) */
#define CAN_WORD028_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD028_DATA_BYTE_3_MASK   (0xFF << CAN_WORD028_DATA_BYTE_3_SHIFT)
#define CAN_WORD028_DATA_BYTE_3(n)     (((n) << CAN_WORD028_DATA_BYTE_3_SHIFT) & CAN_WORD028_DATA_BYTE_3_MASK)
#define CAN_WORD028_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD028_DATA_BYTE_2_MASK   (0xFF << CAN_WORD028_DATA_BYTE_2_SHIFT)
#define CAN_WORD028_DATA_BYTE_2(n)     (((n) << CAN_WORD028_DATA_BYTE_2_SHIFT) & CAN_WORD028_DATA_BYTE_2_MASK)
#define CAN_WORD028_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD028_DATA_BYTE_1_MASK   (0xFF << CAN_WORD028_DATA_BYTE_1_SHIFT)
#define CAN_WORD028_DATA_BYTE_1(n)     (((n) << CAN_WORD028_DATA_BYTE_1_SHIFT) & CAN_WORD028_DATA_BYTE_1_MASK)
#define CAN_WORD028_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD028_DATA_BYTE_0_MASK   (0xFF << CAN_WORD028_DATA_BYTE_0_SHIFT)
#define CAN_WORD028_DATA_BYTE_0(n)     (((n) << CAN_WORD028_DATA_BYTE_0_SHIFT) & CAN_WORD028_DATA_BYTE_0_MASK)

/* Message Buffer 28 WORD1 Register (WORD128) */
#define CAN_WORD128_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD128_DATA_BYTE_7_MASK   (0xFF << CAN_WORD128_DATA_BYTE_7_SHIFT)
#define CAN_WORD128_DATA_BYTE_7(n)     (((n) << CAN_WORD128_DATA_BYTE_7_SHIFT) & CAN_WORD128_DATA_BYTE_7_MASK)
#define CAN_WORD128_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD128_DATA_BYTE_6_MASK   (0xFF << CAN_WORD128_DATA_BYTE_6_SHIFT)
#define CAN_WORD128_DATA_BYTE_6(n)     (((n) << CAN_WORD128_DATA_BYTE_6_SHIFT) & CAN_WORD128_DATA_BYTE_6_MASK)
#define CAN_WORD128_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD128_DATA_BYTE_5_MASK   (0xFF << CAN_WORD128_DATA_BYTE_5_SHIFT)
#define CAN_WORD128_DATA_BYTE_5(n)     (((n) << CAN_WORD128_DATA_BYTE_5_SHIFT) & CAN_WORD128_DATA_BYTE_5_MASK)
#define CAN_WORD128_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD128_DATA_BYTE_4_MASK   (0xFF << CAN_WORD128_DATA_BYTE_4_SHIFT)
#define CAN_WORD128_DATA_BYTE_4(n)     (((n) << CAN_WORD128_DATA_BYTE_4_SHIFT) & CAN_WORD128_DATA_BYTE_4_MASK)

/* Message Buffer 29 CS Register (CS29) */
#define CAN_CS29_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS29_TIME_STAMP_MASK   (0xFFFF << CAN_CS29_TIME_STAMP_SHIFT)
#define CAN_CS29_TIME_STAMP(n)     (((n) << CAN_CS29_TIME_STAMP_SHIFT) & CAN_CS29_TIME_STAMP_MASK)
#define CAN_CS29_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS29_DLC_MASK          (0xF << CAN_CS29_DLC_SHIFT)
#define CAN_CS29_DLC(n)            (((n) << CAN_CS29_DLC_SHIFT) & CAN_CS29_DLC_MASK)
#define CAN_CS29_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS29_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS29_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS29_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS29_CODE_MASK         (0xF << CAN_CS29_CODE_SHIFT)
#define CAN_CS29_CODE(n)           (((n) << CAN_CS29_CODE_SHIFT) & CAN_CS29_CODE_MASK)
#define CAN_CS29_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS29_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS29_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 29 WORD0 Register (WORD029) */
#define CAN_WORD029_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD029_DATA_BYTE_3_MASK   (0xFF << CAN_WORD029_DATA_BYTE_3_SHIFT)
#define CAN_WORD029_DATA_BYTE_3(n)     (((n) << CAN_WORD029_DATA_BYTE_3_SHIFT) & CAN_WORD029_DATA_BYTE_3_MASK)
#define CAN_WORD029_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD029_DATA_BYTE_2_MASK   (0xFF << CAN_WORD029_DATA_BYTE_2_SHIFT)
#define CAN_WORD029_DATA_BYTE_2(n)     (((n) << CAN_WORD029_DATA_BYTE_2_SHIFT) & CAN_WORD029_DATA_BYTE_2_MASK)
#define CAN_WORD029_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD029_DATA_BYTE_1_MASK   (0xFF << CAN_WORD029_DATA_BYTE_1_SHIFT)
#define CAN_WORD029_DATA_BYTE_1(n)     (((n) << CAN_WORD029_DATA_BYTE_1_SHIFT) & CAN_WORD029_DATA_BYTE_1_MASK)
#define CAN_WORD029_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD029_DATA_BYTE_0_MASK   (0xFF << CAN_WORD029_DATA_BYTE_0_SHIFT)
#define CAN_WORD029_DATA_BYTE_0(n)     (((n) << CAN_WORD029_DATA_BYTE_0_SHIFT) & CAN_WORD029_DATA_BYTE_0_MASK)

/* Message Buffer 29 WORD1 Register (WORD129) */
#define CAN_WORD129_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD129_DATA_BYTE_7_MASK   (0xFF << CAN_WORD129_DATA_BYTE_7_SHIFT)
#define CAN_WORD129_DATA_BYTE_7(n)     (((n) << CAN_WORD129_DATA_BYTE_7_SHIFT) & CAN_WORD129_DATA_BYTE_7_MASK)
#define CAN_WORD129_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD129_DATA_BYTE_6_MASK   (0xFF << CAN_WORD129_DATA_BYTE_6_SHIFT)
#define CAN_WORD129_DATA_BYTE_6(n)     (((n) << CAN_WORD129_DATA_BYTE_6_SHIFT) & CAN_WORD129_DATA_BYTE_6_MASK)
#define CAN_WORD129_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD129_DATA_BYTE_5_MASK   (0xFF << CAN_WORD129_DATA_BYTE_5_SHIFT)
#define CAN_WORD129_DATA_BYTE_5(n)     (((n) << CAN_WORD129_DATA_BYTE_5_SHIFT) & CAN_WORD129_DATA_BYTE_5_MASK)
#define CAN_WORD129_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD129_DATA_BYTE_4_MASK   (0xFF << CAN_WORD129_DATA_BYTE_4_SHIFT)
#define CAN_WORD129_DATA_BYTE_4(n)     (((n) << CAN_WORD129_DATA_BYTE_4_SHIFT) & CAN_WORD129_DATA_BYTE_4_MASK)

/* Message Buffer 30 CS Register (CS30) */
#define CAN_CS30_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS30_TIME_STAMP_MASK   (0xFFFF << CAN_CS30_TIME_STAMP_SHIFT)
#define CAN_CS30_TIME_STAMP(n)     (((n) << CAN_CS30_TIME_STAMP_SHIFT) & CAN_CS30_TIME_STAMP_MASK)
#define CAN_CS30_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS30_DLC_MASK          (0xF << CAN_CS30_DLC_SHIFT)
#define CAN_CS30_DLC(n)            (((n) << CAN_CS30_DLC_SHIFT) & CAN_CS30_DLC_MASK)
#define CAN_CS30_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS30_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS30_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS30_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS30_CODE_MASK         (0xF << CAN_CS30_CODE_SHIFT)
#define CAN_CS30_CODE(n)           (((n) << CAN_CS30_CODE_SHIFT) & CAN_CS30_CODE_MASK)
#define CAN_CS30_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS30_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS30_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 30 WORD0 Register (WORD030) */
#define CAN_WORD030_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD030_DATA_BYTE_3_MASK   (0xFF << CAN_WORD030_DATA_BYTE_3_SHIFT)
#define CAN_WORD030_DATA_BYTE_3(n)     (((n) << CAN_WORD030_DATA_BYTE_3_SHIFT) & CAN_WORD030_DATA_BYTE_3_MASK)
#define CAN_WORD030_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD030_DATA_BYTE_2_MASK   (0xFF << CAN_WORD030_DATA_BYTE_2_SHIFT)
#define CAN_WORD030_DATA_BYTE_2(n)     (((n) << CAN_WORD030_DATA_BYTE_2_SHIFT) & CAN_WORD030_DATA_BYTE_2_MASK)
#define CAN_WORD030_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD030_DATA_BYTE_1_MASK   (0xFF << CAN_WORD030_DATA_BYTE_1_SHIFT)
#define CAN_WORD030_DATA_BYTE_1(n)     (((n) << CAN_WORD030_DATA_BYTE_1_SHIFT) & CAN_WORD030_DATA_BYTE_1_MASK)
#define CAN_WORD030_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD030_DATA_BYTE_0_MASK   (0xFF << CAN_WORD030_DATA_BYTE_0_SHIFT)
#define CAN_WORD030_DATA_BYTE_0(n)     (((n) << CAN_WORD030_DATA_BYTE_0_SHIFT) & CAN_WORD030_DATA_BYTE_0_MASK)

/* Message Buffer 30 WORD1 Register (WORD130) */
#define CAN_WORD130_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD130_DATA_BYTE_7_MASK   (0xFF << CAN_WORD130_DATA_BYTE_7_SHIFT)
#define CAN_WORD130_DATA_BYTE_7(n)     (((n) << CAN_WORD130_DATA_BYTE_7_SHIFT) & CAN_WORD130_DATA_BYTE_7_MASK)
#define CAN_WORD130_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD130_DATA_BYTE_6_MASK   (0xFF << CAN_WORD130_DATA_BYTE_6_SHIFT)
#define CAN_WORD130_DATA_BYTE_6(n)     (((n) << CAN_WORD130_DATA_BYTE_6_SHIFT) & CAN_WORD130_DATA_BYTE_6_MASK)
#define CAN_WORD130_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD130_DATA_BYTE_5_MASK   (0xFF << CAN_WORD130_DATA_BYTE_5_SHIFT)
#define CAN_WORD130_DATA_BYTE_5(n)     (((n) << CAN_WORD130_DATA_BYTE_5_SHIFT) & CAN_WORD130_DATA_BYTE_5_MASK)
#define CAN_WORD130_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD130_DATA_BYTE_4_MASK   (0xFF << CAN_WORD130_DATA_BYTE_4_SHIFT)
#define CAN_WORD130_DATA_BYTE_4(n)     (((n) << CAN_WORD130_DATA_BYTE_4_SHIFT) & CAN_WORD130_DATA_BYTE_4_MASK)

/* Message Buffer 31 CS Register (CS31) */
#define CAN_CS31_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS31_TIME_STAMP_MASK   (0xFFFF << CAN_CS31_TIME_STAMP_SHIFT)
#define CAN_CS31_TIME_STAMP(n)     (((n) << CAN_CS31_TIME_STAMP_SHIFT) & CAN_CS31_TIME_STAMP_MASK)
#define CAN_CS31_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS31_DLC_MASK          (0xF << CAN_CS31_DLC_SHIFT)
#define CAN_CS31_DLC(n)            (((n) << CAN_CS31_DLC_SHIFT) & CAN_CS31_DLC_MASK)
#define CAN_CS31_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS31_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS31_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS31_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS31_CODE_MASK         (0xF << CAN_CS31_CODE_SHIFT)
#define CAN_CS31_CODE(n)           (((n) << CAN_CS31_CODE_SHIFT) & CAN_CS31_CODE_MASK)
#define CAN_CS31_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS31_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS31_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 31 WORD0 Register (WORD031) */
#define CAN_WORD031_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD031_DATA_BYTE_3_MASK   (0xFF << CAN_WORD031_DATA_BYTE_3_SHIFT)
#define CAN_WORD031_DATA_BYTE_3(n)     (((n) << CAN_WORD031_DATA_BYTE_3_SHIFT) & CAN_WORD031_DATA_BYTE_3_MASK)
#define CAN_WORD031_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD031_DATA_BYTE_2_MASK   (0xFF << CAN_WORD031_DATA_BYTE_2_SHIFT)
#define CAN_WORD031_DATA_BYTE_2(n)     (((n) << CAN_WORD031_DATA_BYTE_2_SHIFT) & CAN_WORD031_DATA_BYTE_2_MASK)
#define CAN_WORD031_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD031_DATA_BYTE_1_MASK   (0xFF << CAN_WORD031_DATA_BYTE_1_SHIFT)
#define CAN_WORD031_DATA_BYTE_1(n)     (((n) << CAN_WORD031_DATA_BYTE_1_SHIFT) & CAN_WORD031_DATA_BYTE_1_MASK)
#define CAN_WORD031_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD031_DATA_BYTE_0_MASK   (0xFF << CAN_WORD031_DATA_BYTE_0_SHIFT)
#define CAN_WORD031_DATA_BYTE_0(n)     (((n) << CAN_WORD031_DATA_BYTE_0_SHIFT) & CAN_WORD031_DATA_BYTE_0_MASK)

/* Message Buffer 31 WORD1 Register (WORD131) */
#define CAN_WORD131_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD131_DATA_BYTE_7_MASK   (0xFF << CAN_WORD131_DATA_BYTE_7_SHIFT)
#define CAN_WORD131_DATA_BYTE_7(n)     (((n) << CAN_WORD131_DATA_BYTE_7_SHIFT) & CAN_WORD131_DATA_BYTE_7_MASK)
#define CAN_WORD131_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD131_DATA_BYTE_6_MASK   (0xFF << CAN_WORD131_DATA_BYTE_6_SHIFT)
#define CAN_WORD131_DATA_BYTE_6(n)     (((n) << CAN_WORD131_DATA_BYTE_6_SHIFT) & CAN_WORD131_DATA_BYTE_6_MASK)
#define CAN_WORD131_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD131_DATA_BYTE_5_MASK   (0xFF << CAN_WORD131_DATA_BYTE_5_SHIFT)
#define CAN_WORD131_DATA_BYTE_5(n)     (((n) << CAN_WORD131_DATA_BYTE_5_SHIFT) & CAN_WORD131_DATA_BYTE_5_MASK)
#define CAN_WORD131_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD131_DATA_BYTE_4_MASK   (0xFF << CAN_WORD131_DATA_BYTE_4_SHIFT)
#define CAN_WORD131_DATA_BYTE_4(n)     (((n) << CAN_WORD131_DATA_BYTE_4_SHIFT) & CAN_WORD131_DATA_BYTE_4_MASK)

/* Message Buffer 32 CS Register (CS32) */
#define CAN_CS32_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS32_TIME_STAMP_MASK   (0xFFFF << CAN_CS32_TIME_STAMP_SHIFT)
#define CAN_CS32_TIME_STAMP(n)     (((n) << CAN_CS32_TIME_STAMP_SHIFT) & CAN_CS32_TIME_STAMP_MASK)
#define CAN_CS32_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS32_DLC_MASK          (0xF << CAN_CS32_DLC_SHIFT)
#define CAN_CS32_DLC(n)            (((n) << CAN_CS32_DLC_SHIFT) & CAN_CS32_DLC_MASK)
#define CAN_CS32_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS32_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS32_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS32_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS32_CODE_MASK         (0xF << CAN_CS32_CODE_SHIFT)
#define CAN_CS32_CODE(n)           (((n) << CAN_CS32_CODE_SHIFT) & CAN_CS32_CODE_MASK)
#define CAN_CS32_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS32_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS32_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 32 WORD0 Register (WORD032) */
#define CAN_WORD032_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD032_DATA_BYTE_3_MASK   (0xFF << CAN_WORD032_DATA_BYTE_3_SHIFT)
#define CAN_WORD032_DATA_BYTE_3(n)     (((n) << CAN_WORD032_DATA_BYTE_3_SHIFT) & CAN_WORD032_DATA_BYTE_3_MASK)
#define CAN_WORD032_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD032_DATA_BYTE_2_MASK   (0xFF << CAN_WORD032_DATA_BYTE_2_SHIFT)
#define CAN_WORD032_DATA_BYTE_2(n)     (((n) << CAN_WORD032_DATA_BYTE_2_SHIFT) & CAN_WORD032_DATA_BYTE_2_MASK)
#define CAN_WORD032_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD032_DATA_BYTE_1_MASK   (0xFF << CAN_WORD032_DATA_BYTE_1_SHIFT)
#define CAN_WORD032_DATA_BYTE_1(n)     (((n) << CAN_WORD032_DATA_BYTE_1_SHIFT) & CAN_WORD032_DATA_BYTE_1_MASK)
#define CAN_WORD032_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD032_DATA_BYTE_0_MASK   (0xFF << CAN_WORD032_DATA_BYTE_0_SHIFT)
#define CAN_WORD032_DATA_BYTE_0(n)     (((n) << CAN_WORD032_DATA_BYTE_0_SHIFT) & CAN_WORD032_DATA_BYTE_0_MASK)

/* Message Buffer 32 WORD1 Register (WORD132) */
#define CAN_WORD132_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD132_DATA_BYTE_7_MASK   (0xFF << CAN_WORD132_DATA_BYTE_7_SHIFT)
#define CAN_WORD132_DATA_BYTE_7(n)     (((n) << CAN_WORD132_DATA_BYTE_7_SHIFT) & CAN_WORD132_DATA_BYTE_7_MASK)
#define CAN_WORD132_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD132_DATA_BYTE_6_MASK   (0xFF << CAN_WORD132_DATA_BYTE_6_SHIFT)
#define CAN_WORD132_DATA_BYTE_6(n)     (((n) << CAN_WORD132_DATA_BYTE_6_SHIFT) & CAN_WORD132_DATA_BYTE_6_MASK)
#define CAN_WORD132_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD132_DATA_BYTE_5_MASK   (0xFF << CAN_WORD132_DATA_BYTE_5_SHIFT)
#define CAN_WORD132_DATA_BYTE_5(n)     (((n) << CAN_WORD132_DATA_BYTE_5_SHIFT) & CAN_WORD132_DATA_BYTE_5_MASK)
#define CAN_WORD132_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD132_DATA_BYTE_4_MASK   (0xFF << CAN_WORD132_DATA_BYTE_4_SHIFT)
#define CAN_WORD132_DATA_BYTE_4(n)     (((n) << CAN_WORD132_DATA_BYTE_4_SHIFT) & CAN_WORD132_DATA_BYTE_4_MASK)

/* Message Buffer 33 CS Register (CS33) */
#define CAN_CS33_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS33_TIME_STAMP_MASK   (0xFFFF << CAN_CS33_TIME_STAMP_SHIFT)
#define CAN_CS33_TIME_STAMP(n)     (((n) << CAN_CS33_TIME_STAMP_SHIFT) & CAN_CS33_TIME_STAMP_MASK)
#define CAN_CS33_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS33_DLC_MASK          (0xF << CAN_CS33_DLC_SHIFT)
#define CAN_CS33_DLC(n)            (((n) << CAN_CS33_DLC_SHIFT) & CAN_CS33_DLC_MASK)
#define CAN_CS33_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS33_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS33_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS33_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS33_CODE_MASK         (0xF << CAN_CS33_CODE_SHIFT)
#define CAN_CS33_CODE(n)           (((n) << CAN_CS33_CODE_SHIFT) & CAN_CS33_CODE_MASK)
#define CAN_CS33_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS33_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS33_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 33 WORD0 Register (WORD033) */
#define CAN_WORD033_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD033_DATA_BYTE_3_MASK   (0xFF << CAN_WORD033_DATA_BYTE_3_SHIFT)
#define CAN_WORD033_DATA_BYTE_3(n)     (((n) << CAN_WORD033_DATA_BYTE_3_SHIFT) & CAN_WORD033_DATA_BYTE_3_MASK)
#define CAN_WORD033_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD033_DATA_BYTE_2_MASK   (0xFF << CAN_WORD033_DATA_BYTE_2_SHIFT)
#define CAN_WORD033_DATA_BYTE_2(n)     (((n) << CAN_WORD033_DATA_BYTE_2_SHIFT) & CAN_WORD033_DATA_BYTE_2_MASK)
#define CAN_WORD033_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD033_DATA_BYTE_1_MASK   (0xFF << CAN_WORD033_DATA_BYTE_1_SHIFT)
#define CAN_WORD033_DATA_BYTE_1(n)     (((n) << CAN_WORD033_DATA_BYTE_1_SHIFT) & CAN_WORD033_DATA_BYTE_1_MASK)
#define CAN_WORD033_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD033_DATA_BYTE_0_MASK   (0xFF << CAN_WORD033_DATA_BYTE_0_SHIFT)
#define CAN_WORD033_DATA_BYTE_0(n)     (((n) << CAN_WORD033_DATA_BYTE_0_SHIFT) & CAN_WORD033_DATA_BYTE_0_MASK)

/* Message Buffer 33 WORD1 Register (WORD133) */
#define CAN_WORD133_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD133_DATA_BYTE_7_MASK   (0xFF << CAN_WORD133_DATA_BYTE_7_SHIFT)
#define CAN_WORD133_DATA_BYTE_7(n)     (((n) << CAN_WORD133_DATA_BYTE_7_SHIFT) & CAN_WORD133_DATA_BYTE_7_MASK)
#define CAN_WORD133_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD133_DATA_BYTE_6_MASK   (0xFF << CAN_WORD133_DATA_BYTE_6_SHIFT)
#define CAN_WORD133_DATA_BYTE_6(n)     (((n) << CAN_WORD133_DATA_BYTE_6_SHIFT) & CAN_WORD133_DATA_BYTE_6_MASK)
#define CAN_WORD133_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD133_DATA_BYTE_5_MASK   (0xFF << CAN_WORD133_DATA_BYTE_5_SHIFT)
#define CAN_WORD133_DATA_BYTE_5(n)     (((n) << CAN_WORD133_DATA_BYTE_5_SHIFT) & CAN_WORD133_DATA_BYTE_5_MASK)
#define CAN_WORD133_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD133_DATA_BYTE_4_MASK   (0xFF << CAN_WORD133_DATA_BYTE_4_SHIFT)
#define CAN_WORD133_DATA_BYTE_4(n)     (((n) << CAN_WORD133_DATA_BYTE_4_SHIFT) & CAN_WORD133_DATA_BYTE_4_MASK)

/* Message Buffer 34 CS Register (CS34) */
#define CAN_CS34_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS34_TIME_STAMP_MASK   (0xFFFF << CAN_CS34_TIME_STAMP_SHIFT)
#define CAN_CS34_TIME_STAMP(n)     (((n) << CAN_CS34_TIME_STAMP_SHIFT) & CAN_CS34_TIME_STAMP_MASK)
#define CAN_CS34_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS34_DLC_MASK          (0xF << CAN_CS34_DLC_SHIFT)
#define CAN_CS34_DLC(n)            (((n) << CAN_CS34_DLC_SHIFT) & CAN_CS34_DLC_MASK)
#define CAN_CS34_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS34_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS34_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS34_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS34_CODE_MASK         (0xF << CAN_CS34_CODE_SHIFT)
#define CAN_CS34_CODE(n)           (((n) << CAN_CS34_CODE_SHIFT) & CAN_CS34_CODE_MASK)
#define CAN_CS34_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS34_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS34_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 34 WORD0 Register (WORD034) */
#define CAN_WORD034_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD034_DATA_BYTE_3_MASK   (0xFF << CAN_WORD034_DATA_BYTE_3_SHIFT)
#define CAN_WORD034_DATA_BYTE_3(n)     (((n) << CAN_WORD034_DATA_BYTE_3_SHIFT) & CAN_WORD034_DATA_BYTE_3_MASK)
#define CAN_WORD034_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD034_DATA_BYTE_2_MASK   (0xFF << CAN_WORD034_DATA_BYTE_2_SHIFT)
#define CAN_WORD034_DATA_BYTE_2(n)     (((n) << CAN_WORD034_DATA_BYTE_2_SHIFT) & CAN_WORD034_DATA_BYTE_2_MASK)
#define CAN_WORD034_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD034_DATA_BYTE_1_MASK   (0xFF << CAN_WORD034_DATA_BYTE_1_SHIFT)
#define CAN_WORD034_DATA_BYTE_1(n)     (((n) << CAN_WORD034_DATA_BYTE_1_SHIFT) & CAN_WORD034_DATA_BYTE_1_MASK)
#define CAN_WORD034_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD034_DATA_BYTE_0_MASK   (0xFF << CAN_WORD034_DATA_BYTE_0_SHIFT)
#define CAN_WORD034_DATA_BYTE_0(n)     (((n) << CAN_WORD034_DATA_BYTE_0_SHIFT) & CAN_WORD034_DATA_BYTE_0_MASK)

/* Message Buffer 34 WORD1 Register (WORD134) */
#define CAN_WORD134_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD134_DATA_BYTE_7_MASK   (0xFF << CAN_WORD134_DATA_BYTE_7_SHIFT)
#define CAN_WORD134_DATA_BYTE_7(n)     (((n) << CAN_WORD134_DATA_BYTE_7_SHIFT) & CAN_WORD134_DATA_BYTE_7_MASK)
#define CAN_WORD134_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD134_DATA_BYTE_6_MASK   (0xFF << CAN_WORD134_DATA_BYTE_6_SHIFT)
#define CAN_WORD134_DATA_BYTE_6(n)     (((n) << CAN_WORD134_DATA_BYTE_6_SHIFT) & CAN_WORD134_DATA_BYTE_6_MASK)
#define CAN_WORD134_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD134_DATA_BYTE_5_MASK   (0xFF << CAN_WORD134_DATA_BYTE_5_SHIFT)
#define CAN_WORD134_DATA_BYTE_5(n)     (((n) << CAN_WORD134_DATA_BYTE_5_SHIFT) & CAN_WORD134_DATA_BYTE_5_MASK)
#define CAN_WORD134_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD134_DATA_BYTE_4_MASK   (0xFF << CAN_WORD134_DATA_BYTE_4_SHIFT)
#define CAN_WORD134_DATA_BYTE_4(n)     (((n) << CAN_WORD134_DATA_BYTE_4_SHIFT) & CAN_WORD134_DATA_BYTE_4_MASK)

/* Message Buffer 35 CS Register (CS35) */
#define CAN_CS35_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS35_TIME_STAMP_MASK   (0xFFFF << CAN_CS35_TIME_STAMP_SHIFT)
#define CAN_CS35_TIME_STAMP(n)     (((n) << CAN_CS35_TIME_STAMP_SHIFT) & CAN_CS35_TIME_STAMP_MASK)
#define CAN_CS35_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS35_DLC_MASK          (0xF << CAN_CS35_DLC_SHIFT)
#define CAN_CS35_DLC(n)            (((n) << CAN_CS35_DLC_SHIFT) & CAN_CS35_DLC_MASK)
#define CAN_CS35_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS35_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS35_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS35_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS35_CODE_MASK         (0xF << CAN_CS35_CODE_SHIFT)
#define CAN_CS35_CODE(n)           (((n) << CAN_CS35_CODE_SHIFT) & CAN_CS35_CODE_MASK)
#define CAN_CS35_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS35_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS35_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 35 WORD0 Register (WORD035) */
#define CAN_WORD035_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD035_DATA_BYTE_3_MASK   (0xFF << CAN_WORD035_DATA_BYTE_3_SHIFT)
#define CAN_WORD035_DATA_BYTE_3(n)     (((n) << CAN_WORD035_DATA_BYTE_3_SHIFT) & CAN_WORD035_DATA_BYTE_3_MASK)
#define CAN_WORD035_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD035_DATA_BYTE_2_MASK   (0xFF << CAN_WORD035_DATA_BYTE_2_SHIFT)
#define CAN_WORD035_DATA_BYTE_2(n)     (((n) << CAN_WORD035_DATA_BYTE_2_SHIFT) & CAN_WORD035_DATA_BYTE_2_MASK)
#define CAN_WORD035_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD035_DATA_BYTE_1_MASK   (0xFF << CAN_WORD035_DATA_BYTE_1_SHIFT)
#define CAN_WORD035_DATA_BYTE_1(n)     (((n) << CAN_WORD035_DATA_BYTE_1_SHIFT) & CAN_WORD035_DATA_BYTE_1_MASK)
#define CAN_WORD035_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD035_DATA_BYTE_0_MASK   (0xFF << CAN_WORD035_DATA_BYTE_0_SHIFT)
#define CAN_WORD035_DATA_BYTE_0(n)     (((n) << CAN_WORD035_DATA_BYTE_0_SHIFT) & CAN_WORD035_DATA_BYTE_0_MASK)

/* Message Buffer 35 WORD1 Register (WORD135) */
#define CAN_WORD135_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD135_DATA_BYTE_7_MASK   (0xFF << CAN_WORD135_DATA_BYTE_7_SHIFT)
#define CAN_WORD135_DATA_BYTE_7(n)     (((n) << CAN_WORD135_DATA_BYTE_7_SHIFT) & CAN_WORD135_DATA_BYTE_7_MASK)
#define CAN_WORD135_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD135_DATA_BYTE_6_MASK   (0xFF << CAN_WORD135_DATA_BYTE_6_SHIFT)
#define CAN_WORD135_DATA_BYTE_6(n)     (((n) << CAN_WORD135_DATA_BYTE_6_SHIFT) & CAN_WORD135_DATA_BYTE_6_MASK)
#define CAN_WORD135_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD135_DATA_BYTE_5_MASK   (0xFF << CAN_WORD135_DATA_BYTE_5_SHIFT)
#define CAN_WORD135_DATA_BYTE_5(n)     (((n) << CAN_WORD135_DATA_BYTE_5_SHIFT) & CAN_WORD135_DATA_BYTE_5_MASK)
#define CAN_WORD135_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD135_DATA_BYTE_4_MASK   (0xFF << CAN_WORD135_DATA_BYTE_4_SHIFT)
#define CAN_WORD135_DATA_BYTE_4(n)     (((n) << CAN_WORD135_DATA_BYTE_4_SHIFT) & CAN_WORD135_DATA_BYTE_4_MASK)

/* Message Buffer 36 CS Register (CS36) */
#define CAN_CS36_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS36_TIME_STAMP_MASK   (0xFFFF << CAN_CS36_TIME_STAMP_SHIFT)
#define CAN_CS36_TIME_STAMP(n)     (((n) << CAN_CS36_TIME_STAMP_SHIFT) & CAN_CS36_TIME_STAMP_MASK)
#define CAN_CS36_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS36_DLC_MASK          (0xF << CAN_CS36_DLC_SHIFT)
#define CAN_CS36_DLC(n)            (((n) << CAN_CS36_DLC_SHIFT) & CAN_CS36_DLC_MASK)
#define CAN_CS36_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS36_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS36_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS36_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS36_CODE_MASK         (0xF << CAN_CS36_CODE_SHIFT)
#define CAN_CS36_CODE(n)           (((n) << CAN_CS36_CODE_SHIFT) & CAN_CS36_CODE_MASK)
#define CAN_CS36_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS36_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS36_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 36 WORD0 Register (WORD036) */
#define CAN_WORD036_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD036_DATA_BYTE_3_MASK   (0xFF << CAN_WORD036_DATA_BYTE_3_SHIFT)
#define CAN_WORD036_DATA_BYTE_3(n)     (((n) << CAN_WORD036_DATA_BYTE_3_SHIFT) & CAN_WORD036_DATA_BYTE_3_MASK)
#define CAN_WORD036_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD036_DATA_BYTE_2_MASK   (0xFF << CAN_WORD036_DATA_BYTE_2_SHIFT)
#define CAN_WORD036_DATA_BYTE_2(n)     (((n) << CAN_WORD036_DATA_BYTE_2_SHIFT) & CAN_WORD036_DATA_BYTE_2_MASK)
#define CAN_WORD036_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD036_DATA_BYTE_1_MASK   (0xFF << CAN_WORD036_DATA_BYTE_1_SHIFT)
#define CAN_WORD036_DATA_BYTE_1(n)     (((n) << CAN_WORD036_DATA_BYTE_1_SHIFT) & CAN_WORD036_DATA_BYTE_1_MASK)
#define CAN_WORD036_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD036_DATA_BYTE_0_MASK   (0xFF << CAN_WORD036_DATA_BYTE_0_SHIFT)
#define CAN_WORD036_DATA_BYTE_0(n)     (((n) << CAN_WORD036_DATA_BYTE_0_SHIFT) & CAN_WORD036_DATA_BYTE_0_MASK)

/* Message Buffer 36 WORD1 Register (WORD136) */
#define CAN_WORD136_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD136_DATA_BYTE_7_MASK   (0xFF << CAN_WORD136_DATA_BYTE_7_SHIFT)
#define CAN_WORD136_DATA_BYTE_7(n)     (((n) << CAN_WORD136_DATA_BYTE_7_SHIFT) & CAN_WORD136_DATA_BYTE_7_MASK)
#define CAN_WORD136_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD136_DATA_BYTE_6_MASK   (0xFF << CAN_WORD136_DATA_BYTE_6_SHIFT)
#define CAN_WORD136_DATA_BYTE_6(n)     (((n) << CAN_WORD136_DATA_BYTE_6_SHIFT) & CAN_WORD136_DATA_BYTE_6_MASK)
#define CAN_WORD136_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD136_DATA_BYTE_5_MASK   (0xFF << CAN_WORD136_DATA_BYTE_5_SHIFT)
#define CAN_WORD136_DATA_BYTE_5(n)     (((n) << CAN_WORD136_DATA_BYTE_5_SHIFT) & CAN_WORD136_DATA_BYTE_5_MASK)
#define CAN_WORD136_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD136_DATA_BYTE_4_MASK   (0xFF << CAN_WORD136_DATA_BYTE_4_SHIFT)
#define CAN_WORD136_DATA_BYTE_4(n)     (((n) << CAN_WORD136_DATA_BYTE_4_SHIFT) & CAN_WORD136_DATA_BYTE_4_MASK)

/* Message Buffer 37 CS Register (CS37) */
#define CAN_CS37_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS37_TIME_STAMP_MASK   (0xFFFF << CAN_CS37_TIME_STAMP_SHIFT)
#define CAN_CS37_TIME_STAMP(n)     (((n) << CAN_CS37_TIME_STAMP_SHIFT) & CAN_CS37_TIME_STAMP_MASK)
#define CAN_CS37_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS37_DLC_MASK          (0xF << CAN_CS37_DLC_SHIFT)
#define CAN_CS37_DLC(n)            (((n) << CAN_CS37_DLC_SHIFT) & CAN_CS37_DLC_MASK)
#define CAN_CS37_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS37_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS37_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS37_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS37_CODE_MASK         (0xF << CAN_CS37_CODE_SHIFT)
#define CAN_CS37_CODE(n)           (((n) << CAN_CS37_CODE_SHIFT) & CAN_CS37_CODE_MASK)
#define CAN_CS37_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS37_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS37_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 37 WORD0 Register (WORD037) */
#define CAN_WORD037_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD037_DATA_BYTE_3_MASK   (0xFF << CAN_WORD037_DATA_BYTE_3_SHIFT)
#define CAN_WORD037_DATA_BYTE_3(n)     (((n) << CAN_WORD037_DATA_BYTE_3_SHIFT) & CAN_WORD037_DATA_BYTE_3_MASK)
#define CAN_WORD037_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD037_DATA_BYTE_2_MASK   (0xFF << CAN_WORD037_DATA_BYTE_2_SHIFT)
#define CAN_WORD037_DATA_BYTE_2(n)     (((n) << CAN_WORD037_DATA_BYTE_2_SHIFT) & CAN_WORD037_DATA_BYTE_2_MASK)
#define CAN_WORD037_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD037_DATA_BYTE_1_MASK   (0xFF << CAN_WORD037_DATA_BYTE_1_SHIFT)
#define CAN_WORD037_DATA_BYTE_1(n)     (((n) << CAN_WORD037_DATA_BYTE_1_SHIFT) & CAN_WORD037_DATA_BYTE_1_MASK)
#define CAN_WORD037_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD037_DATA_BYTE_0_MASK   (0xFF << CAN_WORD037_DATA_BYTE_0_SHIFT)
#define CAN_WORD037_DATA_BYTE_0(n)     (((n) << CAN_WORD037_DATA_BYTE_0_SHIFT) & CAN_WORD037_DATA_BYTE_0_MASK)

/* Message Buffer 37 WORD1 Register (WORD137) */
#define CAN_WORD137_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD137_DATA_BYTE_7_MASK   (0xFF << CAN_WORD137_DATA_BYTE_7_SHIFT)
#define CAN_WORD137_DATA_BYTE_7(n)     (((n) << CAN_WORD137_DATA_BYTE_7_SHIFT) & CAN_WORD137_DATA_BYTE_7_MASK)
#define CAN_WORD137_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD137_DATA_BYTE_6_MASK   (0xFF << CAN_WORD137_DATA_BYTE_6_SHIFT)
#define CAN_WORD137_DATA_BYTE_6(n)     (((n) << CAN_WORD137_DATA_BYTE_6_SHIFT) & CAN_WORD137_DATA_BYTE_6_MASK)
#define CAN_WORD137_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD137_DATA_BYTE_5_MASK   (0xFF << CAN_WORD137_DATA_BYTE_5_SHIFT)
#define CAN_WORD137_DATA_BYTE_5(n)     (((n) << CAN_WORD137_DATA_BYTE_5_SHIFT) & CAN_WORD137_DATA_BYTE_5_MASK)
#define CAN_WORD137_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD137_DATA_BYTE_4_MASK   (0xFF << CAN_WORD137_DATA_BYTE_4_SHIFT)
#define CAN_WORD137_DATA_BYTE_4(n)     (((n) << CAN_WORD137_DATA_BYTE_4_SHIFT) & CAN_WORD137_DATA_BYTE_4_MASK)

/* Message Buffer 38 CS Register (CS38) */
#define CAN_CS38_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS38_TIME_STAMP_MASK   (0xFFFF << CAN_CS38_TIME_STAMP_SHIFT)
#define CAN_CS38_TIME_STAMP(n)     (((n) << CAN_CS38_TIME_STAMP_SHIFT) & CAN_CS38_TIME_STAMP_MASK)
#define CAN_CS38_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS38_DLC_MASK          (0xF << CAN_CS38_DLC_SHIFT)
#define CAN_CS38_DLC(n)            (((n) << CAN_CS38_DLC_SHIFT) & CAN_CS38_DLC_MASK)
#define CAN_CS38_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS38_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS38_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS38_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS38_CODE_MASK         (0xF << CAN_CS38_CODE_SHIFT)
#define CAN_CS38_CODE(n)           (((n) << CAN_CS38_CODE_SHIFT) & CAN_CS38_CODE_MASK)
#define CAN_CS38_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS38_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS38_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 38 WORD0 Register (WORD038) */
#define CAN_WORD038_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD038_DATA_BYTE_3_MASK   (0xFF << CAN_WORD038_DATA_BYTE_3_SHIFT)
#define CAN_WORD038_DATA_BYTE_3(n)     (((n) << CAN_WORD038_DATA_BYTE_3_SHIFT) & CAN_WORD038_DATA_BYTE_3_MASK)
#define CAN_WORD038_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD038_DATA_BYTE_2_MASK   (0xFF << CAN_WORD038_DATA_BYTE_2_SHIFT)
#define CAN_WORD038_DATA_BYTE_2(n)     (((n) << CAN_WORD038_DATA_BYTE_2_SHIFT) & CAN_WORD038_DATA_BYTE_2_MASK)
#define CAN_WORD038_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD038_DATA_BYTE_1_MASK   (0xFF << CAN_WORD038_DATA_BYTE_1_SHIFT)
#define CAN_WORD038_DATA_BYTE_1(n)     (((n) << CAN_WORD038_DATA_BYTE_1_SHIFT) & CAN_WORD038_DATA_BYTE_1_MASK)
#define CAN_WORD038_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD038_DATA_BYTE_0_MASK   (0xFF << CAN_WORD038_DATA_BYTE_0_SHIFT)
#define CAN_WORD038_DATA_BYTE_0(n)     (((n) << CAN_WORD038_DATA_BYTE_0_SHIFT) & CAN_WORD038_DATA_BYTE_0_MASK)

/* Message Buffer 38 WORD1 Register (WORD138) */
#define CAN_WORD138_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD138_DATA_BYTE_7_MASK   (0xFF << CAN_WORD138_DATA_BYTE_7_SHIFT)
#define CAN_WORD138_DATA_BYTE_7(n)     (((n) << CAN_WORD138_DATA_BYTE_7_SHIFT) & CAN_WORD138_DATA_BYTE_7_MASK)
#define CAN_WORD138_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD138_DATA_BYTE_6_MASK   (0xFF << CAN_WORD138_DATA_BYTE_6_SHIFT)
#define CAN_WORD138_DATA_BYTE_6(n)     (((n) << CAN_WORD138_DATA_BYTE_6_SHIFT) & CAN_WORD138_DATA_BYTE_6_MASK)
#define CAN_WORD138_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD138_DATA_BYTE_5_MASK   (0xFF << CAN_WORD138_DATA_BYTE_5_SHIFT)
#define CAN_WORD138_DATA_BYTE_5(n)     (((n) << CAN_WORD138_DATA_BYTE_5_SHIFT) & CAN_WORD138_DATA_BYTE_5_MASK)
#define CAN_WORD138_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD138_DATA_BYTE_4_MASK   (0xFF << CAN_WORD138_DATA_BYTE_4_SHIFT)
#define CAN_WORD138_DATA_BYTE_4(n)     (((n) << CAN_WORD138_DATA_BYTE_4_SHIFT) & CAN_WORD138_DATA_BYTE_4_MASK)

/* Message Buffer 39 CS Register (CS39) */
#define CAN_CS39_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS39_TIME_STAMP_MASK   (0xFFFF << CAN_CS39_TIME_STAMP_SHIFT)
#define CAN_CS39_TIME_STAMP(n)     (((n) << CAN_CS39_TIME_STAMP_SHIFT) & CAN_CS39_TIME_STAMP_MASK)
#define CAN_CS39_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS39_DLC_MASK          (0xF << CAN_CS39_DLC_SHIFT)
#define CAN_CS39_DLC(n)            (((n) << CAN_CS39_DLC_SHIFT) & CAN_CS39_DLC_MASK)
#define CAN_CS39_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS39_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS39_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS39_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS39_CODE_MASK         (0xF << CAN_CS39_CODE_SHIFT)
#define CAN_CS39_CODE(n)           (((n) << CAN_CS39_CODE_SHIFT) & CAN_CS39_CODE_MASK)
#define CAN_CS39_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS39_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS39_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 39 WORD0 Register (WORD039) */
#define CAN_WORD039_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD039_DATA_BYTE_3_MASK   (0xFF << CAN_WORD039_DATA_BYTE_3_SHIFT)
#define CAN_WORD039_DATA_BYTE_3(n)     (((n) << CAN_WORD039_DATA_BYTE_3_SHIFT) & CAN_WORD039_DATA_BYTE_3_MASK)
#define CAN_WORD039_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD039_DATA_BYTE_2_MASK   (0xFF << CAN_WORD039_DATA_BYTE_2_SHIFT)
#define CAN_WORD039_DATA_BYTE_2(n)     (((n) << CAN_WORD039_DATA_BYTE_2_SHIFT) & CAN_WORD039_DATA_BYTE_2_MASK)
#define CAN_WORD039_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD039_DATA_BYTE_1_MASK   (0xFF << CAN_WORD039_DATA_BYTE_1_SHIFT)
#define CAN_WORD039_DATA_BYTE_1(n)     (((n) << CAN_WORD039_DATA_BYTE_1_SHIFT) & CAN_WORD039_DATA_BYTE_1_MASK)
#define CAN_WORD039_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD039_DATA_BYTE_0_MASK   (0xFF << CAN_WORD039_DATA_BYTE_0_SHIFT)
#define CAN_WORD039_DATA_BYTE_0(n)     (((n) << CAN_WORD039_DATA_BYTE_0_SHIFT) & CAN_WORD039_DATA_BYTE_0_MASK)

/* Message Buffer 39 WORD1 Register (WORD139) */
#define CAN_WORD139_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD139_DATA_BYTE_7_MASK   (0xFF << CAN_WORD139_DATA_BYTE_7_SHIFT)
#define CAN_WORD139_DATA_BYTE_7(n)     (((n) << CAN_WORD139_DATA_BYTE_7_SHIFT) & CAN_WORD139_DATA_BYTE_7_MASK)
#define CAN_WORD139_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD139_DATA_BYTE_6_MASK   (0xFF << CAN_WORD139_DATA_BYTE_6_SHIFT)
#define CAN_WORD139_DATA_BYTE_6(n)     (((n) << CAN_WORD139_DATA_BYTE_6_SHIFT) & CAN_WORD139_DATA_BYTE_6_MASK)
#define CAN_WORD139_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD139_DATA_BYTE_5_MASK   (0xFF << CAN_WORD139_DATA_BYTE_5_SHIFT)
#define CAN_WORD139_DATA_BYTE_5(n)     (((n) << CAN_WORD139_DATA_BYTE_5_SHIFT) & CAN_WORD139_DATA_BYTE_5_MASK)
#define CAN_WORD139_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD139_DATA_BYTE_4_MASK   (0xFF << CAN_WORD139_DATA_BYTE_4_SHIFT)
#define CAN_WORD139_DATA_BYTE_4(n)     (((n) << CAN_WORD139_DATA_BYTE_4_SHIFT) & CAN_WORD139_DATA_BYTE_4_MASK)

/* Message Buffer 40 CS Register (CS40) */
#define CAN_CS40_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS40_TIME_STAMP_MASK   (0xFFFF << CAN_CS40_TIME_STAMP_SHIFT)
#define CAN_CS40_TIME_STAMP(n)     (((n) << CAN_CS40_TIME_STAMP_SHIFT) & CAN_CS40_TIME_STAMP_MASK)
#define CAN_CS40_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS40_DLC_MASK          (0xF << CAN_CS40_DLC_SHIFT)
#define CAN_CS40_DLC(n)            (((n) << CAN_CS40_DLC_SHIFT) & CAN_CS40_DLC_MASK)
#define CAN_CS40_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS40_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS40_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS40_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS40_CODE_MASK         (0xF << CAN_CS40_CODE_SHIFT)
#define CAN_CS40_CODE(n)           (((n) << CAN_CS40_CODE_SHIFT) & CAN_CS40_CODE_MASK)
#define CAN_CS40_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS40_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS40_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 40 WORD0 Register (WORD040) */
#define CAN_WORD040_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD040_DATA_BYTE_3_MASK   (0xFF << CAN_WORD040_DATA_BYTE_3_SHIFT)
#define CAN_WORD040_DATA_BYTE_3(n)     (((n) << CAN_WORD040_DATA_BYTE_3_SHIFT) & CAN_WORD040_DATA_BYTE_3_MASK)
#define CAN_WORD040_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD040_DATA_BYTE_2_MASK   (0xFF << CAN_WORD040_DATA_BYTE_2_SHIFT)
#define CAN_WORD040_DATA_BYTE_2(n)     (((n) << CAN_WORD040_DATA_BYTE_2_SHIFT) & CAN_WORD040_DATA_BYTE_2_MASK)
#define CAN_WORD040_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD040_DATA_BYTE_1_MASK   (0xFF << CAN_WORD040_DATA_BYTE_1_SHIFT)
#define CAN_WORD040_DATA_BYTE_1(n)     (((n) << CAN_WORD040_DATA_BYTE_1_SHIFT) & CAN_WORD040_DATA_BYTE_1_MASK)
#define CAN_WORD040_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD040_DATA_BYTE_0_MASK   (0xFF << CAN_WORD040_DATA_BYTE_0_SHIFT)
#define CAN_WORD040_DATA_BYTE_0(n)     (((n) << CAN_WORD040_DATA_BYTE_0_SHIFT) & CAN_WORD040_DATA_BYTE_0_MASK)

/* Message Buffer 40 WORD1 Register (WORD140) */
#define CAN_WORD140_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD140_DATA_BYTE_7_MASK   (0xFF << CAN_WORD140_DATA_BYTE_7_SHIFT)
#define CAN_WORD140_DATA_BYTE_7(n)     (((n) << CAN_WORD140_DATA_BYTE_7_SHIFT) & CAN_WORD140_DATA_BYTE_7_MASK)
#define CAN_WORD140_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD140_DATA_BYTE_6_MASK   (0xFF << CAN_WORD140_DATA_BYTE_6_SHIFT)
#define CAN_WORD140_DATA_BYTE_6(n)     (((n) << CAN_WORD140_DATA_BYTE_6_SHIFT) & CAN_WORD140_DATA_BYTE_6_MASK)
#define CAN_WORD140_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD140_DATA_BYTE_5_MASK   (0xFF << CAN_WORD140_DATA_BYTE_5_SHIFT)
#define CAN_WORD140_DATA_BYTE_5(n)     (((n) << CAN_WORD140_DATA_BYTE_5_SHIFT) & CAN_WORD140_DATA_BYTE_5_MASK)
#define CAN_WORD140_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD140_DATA_BYTE_4_MASK   (0xFF << CAN_WORD140_DATA_BYTE_4_SHIFT)
#define CAN_WORD140_DATA_BYTE_4(n)     (((n) << CAN_WORD140_DATA_BYTE_4_SHIFT) & CAN_WORD140_DATA_BYTE_4_MASK)

/* Message Buffer 41 CS Register (CS41) */
#define CAN_CS41_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS41_TIME_STAMP_MASK   (0xFFFF << CAN_CS41_TIME_STAMP_SHIFT)
#define CAN_CS41_TIME_STAMP(n)     (((n) << CAN_CS41_TIME_STAMP_SHIFT) & CAN_CS41_TIME_STAMP_MASK)
#define CAN_CS41_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS41_DLC_MASK          (0xF << CAN_CS41_DLC_SHIFT)
#define CAN_CS41_DLC(n)            (((n) << CAN_CS41_DLC_SHIFT) & CAN_CS41_DLC_MASK)
#define CAN_CS41_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS41_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS41_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS41_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS41_CODE_MASK         (0xF << CAN_CS41_CODE_SHIFT)
#define CAN_CS41_CODE(n)           (((n) << CAN_CS41_CODE_SHIFT) & CAN_CS41_CODE_MASK)
#define CAN_CS41_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS41_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS41_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 41 WORD0 Register (WORD041) */
#define CAN_WORD041_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD041_DATA_BYTE_3_MASK   (0xFF << CAN_WORD041_DATA_BYTE_3_SHIFT)
#define CAN_WORD041_DATA_BYTE_3(n)     (((n) << CAN_WORD041_DATA_BYTE_3_SHIFT) & CAN_WORD041_DATA_BYTE_3_MASK)
#define CAN_WORD041_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD041_DATA_BYTE_2_MASK   (0xFF << CAN_WORD041_DATA_BYTE_2_SHIFT)
#define CAN_WORD041_DATA_BYTE_2(n)     (((n) << CAN_WORD041_DATA_BYTE_2_SHIFT) & CAN_WORD041_DATA_BYTE_2_MASK)
#define CAN_WORD041_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD041_DATA_BYTE_1_MASK   (0xFF << CAN_WORD041_DATA_BYTE_1_SHIFT)
#define CAN_WORD041_DATA_BYTE_1(n)     (((n) << CAN_WORD041_DATA_BYTE_1_SHIFT) & CAN_WORD041_DATA_BYTE_1_MASK)
#define CAN_WORD041_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD041_DATA_BYTE_0_MASK   (0xFF << CAN_WORD041_DATA_BYTE_0_SHIFT)
#define CAN_WORD041_DATA_BYTE_0(n)     (((n) << CAN_WORD041_DATA_BYTE_0_SHIFT) & CAN_WORD041_DATA_BYTE_0_MASK)

/* Message Buffer 41 WORD1 Register (WORD141) */
#define CAN_WORD141_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD141_DATA_BYTE_7_MASK   (0xFF << CAN_WORD141_DATA_BYTE_7_SHIFT)
#define CAN_WORD141_DATA_BYTE_7(n)     (((n) << CAN_WORD141_DATA_BYTE_7_SHIFT) & CAN_WORD141_DATA_BYTE_7_MASK)
#define CAN_WORD141_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD141_DATA_BYTE_6_MASK   (0xFF << CAN_WORD141_DATA_BYTE_6_SHIFT)
#define CAN_WORD141_DATA_BYTE_6(n)     (((n) << CAN_WORD141_DATA_BYTE_6_SHIFT) & CAN_WORD141_DATA_BYTE_6_MASK)
#define CAN_WORD141_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD141_DATA_BYTE_5_MASK   (0xFF << CAN_WORD141_DATA_BYTE_5_SHIFT)
#define CAN_WORD141_DATA_BYTE_5(n)     (((n) << CAN_WORD141_DATA_BYTE_5_SHIFT) & CAN_WORD141_DATA_BYTE_5_MASK)
#define CAN_WORD141_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD141_DATA_BYTE_4_MASK   (0xFF << CAN_WORD141_DATA_BYTE_4_SHIFT)
#define CAN_WORD141_DATA_BYTE_4(n)     (((n) << CAN_WORD141_DATA_BYTE_4_SHIFT) & CAN_WORD141_DATA_BYTE_4_MASK)

/* Message Buffer 42 CS Register (CS42) */
#define CAN_CS42_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS42_TIME_STAMP_MASK   (0xFFFF << CAN_CS42_TIME_STAMP_SHIFT)
#define CAN_CS42_TIME_STAMP(n)     (((n) << CAN_CS42_TIME_STAMP_SHIFT) & CAN_CS42_TIME_STAMP_MASK)
#define CAN_CS42_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS42_DLC_MASK          (0xF << CAN_CS42_DLC_SHIFT)
#define CAN_CS42_DLC(n)            (((n) << CAN_CS42_DLC_SHIFT) & CAN_CS42_DLC_MASK)
#define CAN_CS42_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS42_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS42_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS42_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS42_CODE_MASK         (0xF << CAN_CS42_CODE_SHIFT)
#define CAN_CS42_CODE(n)           (((n) << CAN_CS42_CODE_SHIFT) & CAN_CS42_CODE_MASK)
#define CAN_CS42_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS42_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS42_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 42 WORD0 Register (WORD042) */
#define CAN_WORD042_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD042_DATA_BYTE_3_MASK   (0xFF << CAN_WORD042_DATA_BYTE_3_SHIFT)
#define CAN_WORD042_DATA_BYTE_3(n)     (((n) << CAN_WORD042_DATA_BYTE_3_SHIFT) & CAN_WORD042_DATA_BYTE_3_MASK)
#define CAN_WORD042_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD042_DATA_BYTE_2_MASK   (0xFF << CAN_WORD042_DATA_BYTE_2_SHIFT)
#define CAN_WORD042_DATA_BYTE_2(n)     (((n) << CAN_WORD042_DATA_BYTE_2_SHIFT) & CAN_WORD042_DATA_BYTE_2_MASK)
#define CAN_WORD042_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD042_DATA_BYTE_1_MASK   (0xFF << CAN_WORD042_DATA_BYTE_1_SHIFT)
#define CAN_WORD042_DATA_BYTE_1(n)     (((n) << CAN_WORD042_DATA_BYTE_1_SHIFT) & CAN_WORD042_DATA_BYTE_1_MASK)
#define CAN_WORD042_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD042_DATA_BYTE_0_MASK   (0xFF << CAN_WORD042_DATA_BYTE_0_SHIFT)
#define CAN_WORD042_DATA_BYTE_0(n)     (((n) << CAN_WORD042_DATA_BYTE_0_SHIFT) & CAN_WORD042_DATA_BYTE_0_MASK)

/* Message Buffer 42 WORD1 Register (WORD142) */
#define CAN_WORD142_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD142_DATA_BYTE_7_MASK   (0xFF << CAN_WORD142_DATA_BYTE_7_SHIFT)
#define CAN_WORD142_DATA_BYTE_7(n)     (((n) << CAN_WORD142_DATA_BYTE_7_SHIFT) & CAN_WORD142_DATA_BYTE_7_MASK)
#define CAN_WORD142_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD142_DATA_BYTE_6_MASK   (0xFF << CAN_WORD142_DATA_BYTE_6_SHIFT)
#define CAN_WORD142_DATA_BYTE_6(n)     (((n) << CAN_WORD142_DATA_BYTE_6_SHIFT) & CAN_WORD142_DATA_BYTE_6_MASK)
#define CAN_WORD142_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD142_DATA_BYTE_5_MASK   (0xFF << CAN_WORD142_DATA_BYTE_5_SHIFT)
#define CAN_WORD142_DATA_BYTE_5(n)     (((n) << CAN_WORD142_DATA_BYTE_5_SHIFT) & CAN_WORD142_DATA_BYTE_5_MASK)
#define CAN_WORD142_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD142_DATA_BYTE_4_MASK   (0xFF << CAN_WORD142_DATA_BYTE_4_SHIFT)
#define CAN_WORD142_DATA_BYTE_4(n)     (((n) << CAN_WORD142_DATA_BYTE_4_SHIFT) & CAN_WORD142_DATA_BYTE_4_MASK)

/* Message Buffer 43 CS Register (CS43) */
#define CAN_CS43_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS43_TIME_STAMP_MASK   (0xFFFF << CAN_CS43_TIME_STAMP_SHIFT)
#define CAN_CS43_TIME_STAMP(n)     (((n) << CAN_CS43_TIME_STAMP_SHIFT) & CAN_CS43_TIME_STAMP_MASK)
#define CAN_CS43_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS43_DLC_MASK          (0xF << CAN_CS43_DLC_SHIFT)
#define CAN_CS43_DLC(n)            (((n) << CAN_CS43_DLC_SHIFT) & CAN_CS43_DLC_MASK)
#define CAN_CS43_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS43_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS43_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS43_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS43_CODE_MASK         (0xF << CAN_CS43_CODE_SHIFT)
#define CAN_CS43_CODE(n)           (((n) << CAN_CS43_CODE_SHIFT) & CAN_CS43_CODE_MASK)
#define CAN_CS43_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS43_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS43_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 43 WORD0 Register (WORD043) */
#define CAN_WORD043_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD043_DATA_BYTE_3_MASK   (0xFF << CAN_WORD043_DATA_BYTE_3_SHIFT)
#define CAN_WORD043_DATA_BYTE_3(n)     (((n) << CAN_WORD043_DATA_BYTE_3_SHIFT) & CAN_WORD043_DATA_BYTE_3_MASK)
#define CAN_WORD043_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD043_DATA_BYTE_2_MASK   (0xFF << CAN_WORD043_DATA_BYTE_2_SHIFT)
#define CAN_WORD043_DATA_BYTE_2(n)     (((n) << CAN_WORD043_DATA_BYTE_2_SHIFT) & CAN_WORD043_DATA_BYTE_2_MASK)
#define CAN_WORD043_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD043_DATA_BYTE_1_MASK   (0xFF << CAN_WORD043_DATA_BYTE_1_SHIFT)
#define CAN_WORD043_DATA_BYTE_1(n)     (((n) << CAN_WORD043_DATA_BYTE_1_SHIFT) & CAN_WORD043_DATA_BYTE_1_MASK)
#define CAN_WORD043_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD043_DATA_BYTE_0_MASK   (0xFF << CAN_WORD043_DATA_BYTE_0_SHIFT)
#define CAN_WORD043_DATA_BYTE_0(n)     (((n) << CAN_WORD043_DATA_BYTE_0_SHIFT) & CAN_WORD043_DATA_BYTE_0_MASK)

/* Message Buffer 43 WORD1 Register (WORD143) */
#define CAN_WORD143_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD143_DATA_BYTE_7_MASK   (0xFF << CAN_WORD143_DATA_BYTE_7_SHIFT)
#define CAN_WORD143_DATA_BYTE_7(n)     (((n) << CAN_WORD143_DATA_BYTE_7_SHIFT) & CAN_WORD143_DATA_BYTE_7_MASK)
#define CAN_WORD143_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD143_DATA_BYTE_6_MASK   (0xFF << CAN_WORD143_DATA_BYTE_6_SHIFT)
#define CAN_WORD143_DATA_BYTE_6(n)     (((n) << CAN_WORD143_DATA_BYTE_6_SHIFT) & CAN_WORD143_DATA_BYTE_6_MASK)
#define CAN_WORD143_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD143_DATA_BYTE_5_MASK   (0xFF << CAN_WORD143_DATA_BYTE_5_SHIFT)
#define CAN_WORD143_DATA_BYTE_5(n)     (((n) << CAN_WORD143_DATA_BYTE_5_SHIFT) & CAN_WORD143_DATA_BYTE_5_MASK)
#define CAN_WORD143_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD143_DATA_BYTE_4_MASK   (0xFF << CAN_WORD143_DATA_BYTE_4_SHIFT)
#define CAN_WORD143_DATA_BYTE_4(n)     (((n) << CAN_WORD143_DATA_BYTE_4_SHIFT) & CAN_WORD143_DATA_BYTE_4_MASK)

/* Message Buffer 44 CS Register (CS44) */
#define CAN_CS44_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS44_TIME_STAMP_MASK   (0xFFFF << CAN_CS44_TIME_STAMP_SHIFT)
#define CAN_CS44_TIME_STAMP(n)     (((n) << CAN_CS44_TIME_STAMP_SHIFT) & CAN_CS44_TIME_STAMP_MASK)
#define CAN_CS44_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS44_DLC_MASK          (0xF << CAN_CS44_DLC_SHIFT)
#define CAN_CS44_DLC(n)            (((n) << CAN_CS44_DLC_SHIFT) & CAN_CS44_DLC_MASK)
#define CAN_CS44_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS44_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS44_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS44_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS44_CODE_MASK         (0xF << CAN_CS44_CODE_SHIFT)
#define CAN_CS44_CODE(n)           (((n) << CAN_CS44_CODE_SHIFT) & CAN_CS44_CODE_MASK)
#define CAN_CS44_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS44_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS44_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 44 WORD0 Register (WORD044) */
#define CAN_WORD044_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD044_DATA_BYTE_3_MASK   (0xFF << CAN_WORD044_DATA_BYTE_3_SHIFT)
#define CAN_WORD044_DATA_BYTE_3(n)     (((n) << CAN_WORD044_DATA_BYTE_3_SHIFT) & CAN_WORD044_DATA_BYTE_3_MASK)
#define CAN_WORD044_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD044_DATA_BYTE_2_MASK   (0xFF << CAN_WORD044_DATA_BYTE_2_SHIFT)
#define CAN_WORD044_DATA_BYTE_2(n)     (((n) << CAN_WORD044_DATA_BYTE_2_SHIFT) & CAN_WORD044_DATA_BYTE_2_MASK)
#define CAN_WORD044_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD044_DATA_BYTE_1_MASK   (0xFF << CAN_WORD044_DATA_BYTE_1_SHIFT)
#define CAN_WORD044_DATA_BYTE_1(n)     (((n) << CAN_WORD044_DATA_BYTE_1_SHIFT) & CAN_WORD044_DATA_BYTE_1_MASK)
#define CAN_WORD044_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD044_DATA_BYTE_0_MASK   (0xFF << CAN_WORD044_DATA_BYTE_0_SHIFT)
#define CAN_WORD044_DATA_BYTE_0(n)     (((n) << CAN_WORD044_DATA_BYTE_0_SHIFT) & CAN_WORD044_DATA_BYTE_0_MASK)

/* Message Buffer 44 WORD1 Register (WORD144) */
#define CAN_WORD144_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD144_DATA_BYTE_7_MASK   (0xFF << CAN_WORD144_DATA_BYTE_7_SHIFT)
#define CAN_WORD144_DATA_BYTE_7(n)     (((n) << CAN_WORD144_DATA_BYTE_7_SHIFT) & CAN_WORD144_DATA_BYTE_7_MASK)
#define CAN_WORD144_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD144_DATA_BYTE_6_MASK   (0xFF << CAN_WORD144_DATA_BYTE_6_SHIFT)
#define CAN_WORD144_DATA_BYTE_6(n)     (((n) << CAN_WORD144_DATA_BYTE_6_SHIFT) & CAN_WORD144_DATA_BYTE_6_MASK)
#define CAN_WORD144_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD144_DATA_BYTE_5_MASK   (0xFF << CAN_WORD144_DATA_BYTE_5_SHIFT)
#define CAN_WORD144_DATA_BYTE_5(n)     (((n) << CAN_WORD144_DATA_BYTE_5_SHIFT) & CAN_WORD144_DATA_BYTE_5_MASK)
#define CAN_WORD144_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD144_DATA_BYTE_4_MASK   (0xFF << CAN_WORD144_DATA_BYTE_4_SHIFT)
#define CAN_WORD144_DATA_BYTE_4(n)     (((n) << CAN_WORD144_DATA_BYTE_4_SHIFT) & CAN_WORD144_DATA_BYTE_4_MASK)

/* Message Buffer 45 CS Register (CS45) */
#define CAN_CS45_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS45_TIME_STAMP_MASK   (0xFFFF << CAN_CS45_TIME_STAMP_SHIFT)
#define CAN_CS45_TIME_STAMP(n)     (((n) << CAN_CS45_TIME_STAMP_SHIFT) & CAN_CS45_TIME_STAMP_MASK)
#define CAN_CS45_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS45_DLC_MASK          (0xF << CAN_CS45_DLC_SHIFT)
#define CAN_CS45_DLC(n)            (((n) << CAN_CS45_DLC_SHIFT) & CAN_CS45_DLC_MASK)
#define CAN_CS45_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS45_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS45_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS45_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS45_CODE_MASK         (0xF << CAN_CS45_CODE_SHIFT)
#define CAN_CS45_CODE(n)           (((n) << CAN_CS45_CODE_SHIFT) & CAN_CS45_CODE_MASK)
#define CAN_CS45_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS45_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS45_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 45 WORD0 Register (WORD045) */
#define CAN_WORD045_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD045_DATA_BYTE_3_MASK   (0xFF << CAN_WORD045_DATA_BYTE_3_SHIFT)
#define CAN_WORD045_DATA_BYTE_3(n)     (((n) << CAN_WORD045_DATA_BYTE_3_SHIFT) & CAN_WORD045_DATA_BYTE_3_MASK)
#define CAN_WORD045_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD045_DATA_BYTE_2_MASK   (0xFF << CAN_WORD045_DATA_BYTE_2_SHIFT)
#define CAN_WORD045_DATA_BYTE_2(n)     (((n) << CAN_WORD045_DATA_BYTE_2_SHIFT) & CAN_WORD045_DATA_BYTE_2_MASK)
#define CAN_WORD045_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD045_DATA_BYTE_1_MASK   (0xFF << CAN_WORD045_DATA_BYTE_1_SHIFT)
#define CAN_WORD045_DATA_BYTE_1(n)     (((n) << CAN_WORD045_DATA_BYTE_1_SHIFT) & CAN_WORD045_DATA_BYTE_1_MASK)
#define CAN_WORD045_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD045_DATA_BYTE_0_MASK   (0xFF << CAN_WORD045_DATA_BYTE_0_SHIFT)
#define CAN_WORD045_DATA_BYTE_0(n)     (((n) << CAN_WORD045_DATA_BYTE_0_SHIFT) & CAN_WORD045_DATA_BYTE_0_MASK)

/* Message Buffer 45 WORD1 Register (WORD145) */
#define CAN_WORD145_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD145_DATA_BYTE_7_MASK   (0xFF << CAN_WORD145_DATA_BYTE_7_SHIFT)
#define CAN_WORD145_DATA_BYTE_7(n)     (((n) << CAN_WORD145_DATA_BYTE_7_SHIFT) & CAN_WORD145_DATA_BYTE_7_MASK)
#define CAN_WORD145_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD145_DATA_BYTE_6_MASK   (0xFF << CAN_WORD145_DATA_BYTE_6_SHIFT)
#define CAN_WORD145_DATA_BYTE_6(n)     (((n) << CAN_WORD145_DATA_BYTE_6_SHIFT) & CAN_WORD145_DATA_BYTE_6_MASK)
#define CAN_WORD145_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD145_DATA_BYTE_5_MASK   (0xFF << CAN_WORD145_DATA_BYTE_5_SHIFT)
#define CAN_WORD145_DATA_BYTE_5(n)     (((n) << CAN_WORD145_DATA_BYTE_5_SHIFT) & CAN_WORD145_DATA_BYTE_5_MASK)
#define CAN_WORD145_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD145_DATA_BYTE_4_MASK   (0xFF << CAN_WORD145_DATA_BYTE_4_SHIFT)
#define CAN_WORD145_DATA_BYTE_4(n)     (((n) << CAN_WORD145_DATA_BYTE_4_SHIFT) & CAN_WORD145_DATA_BYTE_4_MASK)

/* Message Buffer 46 CS Register (CS46) */
#define CAN_CS46_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS46_TIME_STAMP_MASK   (0xFFFF << CAN_CS46_TIME_STAMP_SHIFT)
#define CAN_CS46_TIME_STAMP(n)     (((n) << CAN_CS46_TIME_STAMP_SHIFT) & CAN_CS46_TIME_STAMP_MASK)
#define CAN_CS46_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS46_DLC_MASK          (0xF << CAN_CS46_DLC_SHIFT)
#define CAN_CS46_DLC(n)            (((n) << CAN_CS46_DLC_SHIFT) & CAN_CS46_DLC_MASK)
#define CAN_CS46_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS46_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS46_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS46_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS46_CODE_MASK         (0xF << CAN_CS46_CODE_SHIFT)
#define CAN_CS46_CODE(n)           (((n) << CAN_CS46_CODE_SHIFT) & CAN_CS46_CODE_MASK)
#define CAN_CS46_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS46_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS46_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 46 WORD0 Register (WORD046) */
#define CAN_WORD046_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD046_DATA_BYTE_3_MASK   (0xFF << CAN_WORD046_DATA_BYTE_3_SHIFT)
#define CAN_WORD046_DATA_BYTE_3(n)     (((n) << CAN_WORD046_DATA_BYTE_3_SHIFT) & CAN_WORD046_DATA_BYTE_3_MASK)
#define CAN_WORD046_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD046_DATA_BYTE_2_MASK   (0xFF << CAN_WORD046_DATA_BYTE_2_SHIFT)
#define CAN_WORD046_DATA_BYTE_2(n)     (((n) << CAN_WORD046_DATA_BYTE_2_SHIFT) & CAN_WORD046_DATA_BYTE_2_MASK)
#define CAN_WORD046_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD046_DATA_BYTE_1_MASK   (0xFF << CAN_WORD046_DATA_BYTE_1_SHIFT)
#define CAN_WORD046_DATA_BYTE_1(n)     (((n) << CAN_WORD046_DATA_BYTE_1_SHIFT) & CAN_WORD046_DATA_BYTE_1_MASK)
#define CAN_WORD046_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD046_DATA_BYTE_0_MASK   (0xFF << CAN_WORD046_DATA_BYTE_0_SHIFT)
#define CAN_WORD046_DATA_BYTE_0(n)     (((n) << CAN_WORD046_DATA_BYTE_0_SHIFT) & CAN_WORD046_DATA_BYTE_0_MASK)

/* Message Buffer 46 WORD1 Register (WORD146) */
#define CAN_WORD146_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD146_DATA_BYTE_7_MASK   (0xFF << CAN_WORD146_DATA_BYTE_7_SHIFT)
#define CAN_WORD146_DATA_BYTE_7(n)     (((n) << CAN_WORD146_DATA_BYTE_7_SHIFT) & CAN_WORD146_DATA_BYTE_7_MASK)
#define CAN_WORD146_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD146_DATA_BYTE_6_MASK   (0xFF << CAN_WORD146_DATA_BYTE_6_SHIFT)
#define CAN_WORD146_DATA_BYTE_6(n)     (((n) << CAN_WORD146_DATA_BYTE_6_SHIFT) & CAN_WORD146_DATA_BYTE_6_MASK)
#define CAN_WORD146_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD146_DATA_BYTE_5_MASK   (0xFF << CAN_WORD146_DATA_BYTE_5_SHIFT)
#define CAN_WORD146_DATA_BYTE_5(n)     (((n) << CAN_WORD146_DATA_BYTE_5_SHIFT) & CAN_WORD146_DATA_BYTE_5_MASK)
#define CAN_WORD146_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD146_DATA_BYTE_4_MASK   (0xFF << CAN_WORD146_DATA_BYTE_4_SHIFT)
#define CAN_WORD146_DATA_BYTE_4(n)     (((n) << CAN_WORD146_DATA_BYTE_4_SHIFT) & CAN_WORD146_DATA_BYTE_4_MASK)

/* Message Buffer 47 CS Register (CS47) */
#define CAN_CS47_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS47_TIME_STAMP_MASK   (0xFFFF << CAN_CS47_TIME_STAMP_SHIFT)
#define CAN_CS47_TIME_STAMP(n)     (((n) << CAN_CS47_TIME_STAMP_SHIFT) & CAN_CS47_TIME_STAMP_MASK)
#define CAN_CS47_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS47_DLC_MASK          (0xF << CAN_CS47_DLC_SHIFT)
#define CAN_CS47_DLC(n)            (((n) << CAN_CS47_DLC_SHIFT) & CAN_CS47_DLC_MASK)
#define CAN_CS47_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS47_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS47_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS47_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS47_CODE_MASK         (0xF << CAN_CS47_CODE_SHIFT)
#define CAN_CS47_CODE(n)           (((n) << CAN_CS47_CODE_SHIFT) & CAN_CS47_CODE_MASK)
#define CAN_CS47_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS47_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS47_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 47 WORD0 Register (WORD047) */
#define CAN_WORD047_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD047_DATA_BYTE_3_MASK   (0xFF << CAN_WORD047_DATA_BYTE_3_SHIFT)
#define CAN_WORD047_DATA_BYTE_3(n)     (((n) << CAN_WORD047_DATA_BYTE_3_SHIFT) & CAN_WORD047_DATA_BYTE_3_MASK)
#define CAN_WORD047_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD047_DATA_BYTE_2_MASK   (0xFF << CAN_WORD047_DATA_BYTE_2_SHIFT)
#define CAN_WORD047_DATA_BYTE_2(n)     (((n) << CAN_WORD047_DATA_BYTE_2_SHIFT) & CAN_WORD047_DATA_BYTE_2_MASK)
#define CAN_WORD047_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD047_DATA_BYTE_1_MASK   (0xFF << CAN_WORD047_DATA_BYTE_1_SHIFT)
#define CAN_WORD047_DATA_BYTE_1(n)     (((n) << CAN_WORD047_DATA_BYTE_1_SHIFT) & CAN_WORD047_DATA_BYTE_1_MASK)
#define CAN_WORD047_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD047_DATA_BYTE_0_MASK   (0xFF << CAN_WORD047_DATA_BYTE_0_SHIFT)
#define CAN_WORD047_DATA_BYTE_0(n)     (((n) << CAN_WORD047_DATA_BYTE_0_SHIFT) & CAN_WORD047_DATA_BYTE_0_MASK)

/* Message Buffer 47 WORD1 Register (WORD147) */
#define CAN_WORD147_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD147_DATA_BYTE_7_MASK   (0xFF << CAN_WORD147_DATA_BYTE_7_SHIFT)
#define CAN_WORD147_DATA_BYTE_7(n)     (((n) << CAN_WORD147_DATA_BYTE_7_SHIFT) & CAN_WORD147_DATA_BYTE_7_MASK)
#define CAN_WORD147_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD147_DATA_BYTE_6_MASK   (0xFF << CAN_WORD147_DATA_BYTE_6_SHIFT)
#define CAN_WORD147_DATA_BYTE_6(n)     (((n) << CAN_WORD147_DATA_BYTE_6_SHIFT) & CAN_WORD147_DATA_BYTE_6_MASK)
#define CAN_WORD147_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD147_DATA_BYTE_5_MASK   (0xFF << CAN_WORD147_DATA_BYTE_5_SHIFT)
#define CAN_WORD147_DATA_BYTE_5(n)     (((n) << CAN_WORD147_DATA_BYTE_5_SHIFT) & CAN_WORD147_DATA_BYTE_5_MASK)
#define CAN_WORD147_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD147_DATA_BYTE_4_MASK   (0xFF << CAN_WORD147_DATA_BYTE_4_SHIFT)
#define CAN_WORD147_DATA_BYTE_4(n)     (((n) << CAN_WORD147_DATA_BYTE_4_SHIFT) & CAN_WORD147_DATA_BYTE_4_MASK)

/* Message Buffer 48 CS Register (CS48) */
#define CAN_CS48_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS48_TIME_STAMP_MASK   (0xFFFF << CAN_CS48_TIME_STAMP_SHIFT)
#define CAN_CS48_TIME_STAMP(n)     (((n) << CAN_CS48_TIME_STAMP_SHIFT) & CAN_CS48_TIME_STAMP_MASK)
#define CAN_CS48_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS48_DLC_MASK          (0xF << CAN_CS48_DLC_SHIFT)
#define CAN_CS48_DLC(n)            (((n) << CAN_CS48_DLC_SHIFT) & CAN_CS48_DLC_MASK)
#define CAN_CS48_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS48_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS48_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS48_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS48_CODE_MASK         (0xF << CAN_CS48_CODE_SHIFT)
#define CAN_CS48_CODE(n)           (((n) << CAN_CS48_CODE_SHIFT) & CAN_CS48_CODE_MASK)
#define CAN_CS48_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS48_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS48_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 48 WORD0 Register (WORD048) */
#define CAN_WORD048_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD048_DATA_BYTE_3_MASK   (0xFF << CAN_WORD048_DATA_BYTE_3_SHIFT)
#define CAN_WORD048_DATA_BYTE_3(n)     (((n) << CAN_WORD048_DATA_BYTE_3_SHIFT) & CAN_WORD048_DATA_BYTE_3_MASK)
#define CAN_WORD048_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD048_DATA_BYTE_2_MASK   (0xFF << CAN_WORD048_DATA_BYTE_2_SHIFT)
#define CAN_WORD048_DATA_BYTE_2(n)     (((n) << CAN_WORD048_DATA_BYTE_2_SHIFT) & CAN_WORD048_DATA_BYTE_2_MASK)
#define CAN_WORD048_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD048_DATA_BYTE_1_MASK   (0xFF << CAN_WORD048_DATA_BYTE_1_SHIFT)
#define CAN_WORD048_DATA_BYTE_1(n)     (((n) << CAN_WORD048_DATA_BYTE_1_SHIFT) & CAN_WORD048_DATA_BYTE_1_MASK)
#define CAN_WORD048_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD048_DATA_BYTE_0_MASK   (0xFF << CAN_WORD048_DATA_BYTE_0_SHIFT)
#define CAN_WORD048_DATA_BYTE_0(n)     (((n) << CAN_WORD048_DATA_BYTE_0_SHIFT) & CAN_WORD048_DATA_BYTE_0_MASK)

/* Message Buffer 48 WORD1 Register (WORD148) */
#define CAN_WORD148_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD148_DATA_BYTE_7_MASK   (0xFF << CAN_WORD148_DATA_BYTE_7_SHIFT)
#define CAN_WORD148_DATA_BYTE_7(n)     (((n) << CAN_WORD148_DATA_BYTE_7_SHIFT) & CAN_WORD148_DATA_BYTE_7_MASK)
#define CAN_WORD148_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD148_DATA_BYTE_6_MASK   (0xFF << CAN_WORD148_DATA_BYTE_6_SHIFT)
#define CAN_WORD148_DATA_BYTE_6(n)     (((n) << CAN_WORD148_DATA_BYTE_6_SHIFT) & CAN_WORD148_DATA_BYTE_6_MASK)
#define CAN_WORD148_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD148_DATA_BYTE_5_MASK   (0xFF << CAN_WORD148_DATA_BYTE_5_SHIFT)
#define CAN_WORD148_DATA_BYTE_5(n)     (((n) << CAN_WORD148_DATA_BYTE_5_SHIFT) & CAN_WORD148_DATA_BYTE_5_MASK)
#define CAN_WORD148_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD148_DATA_BYTE_4_MASK   (0xFF << CAN_WORD148_DATA_BYTE_4_SHIFT)
#define CAN_WORD148_DATA_BYTE_4(n)     (((n) << CAN_WORD148_DATA_BYTE_4_SHIFT) & CAN_WORD148_DATA_BYTE_4_MASK)

/* Message Buffer 49 CS Register (CS49) */
#define CAN_CS49_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS49_TIME_STAMP_MASK   (0xFFFF << CAN_CS49_TIME_STAMP_SHIFT)
#define CAN_CS49_TIME_STAMP(n)     (((n) << CAN_CS49_TIME_STAMP_SHIFT) & CAN_CS49_TIME_STAMP_MASK)
#define CAN_CS49_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS49_DLC_MASK          (0xF << CAN_CS49_DLC_SHIFT)
#define CAN_CS49_DLC(n)            (((n) << CAN_CS49_DLC_SHIFT) & CAN_CS49_DLC_MASK)
#define CAN_CS49_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS49_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS49_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS49_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS49_CODE_MASK         (0xF << CAN_CS49_CODE_SHIFT)
#define CAN_CS49_CODE(n)           (((n) << CAN_CS49_CODE_SHIFT) & CAN_CS49_CODE_MASK)
#define CAN_CS49_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS49_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS49_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 49 WORD0 Register (WORD049) */
#define CAN_WORD049_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD049_DATA_BYTE_3_MASK   (0xFF << CAN_WORD049_DATA_BYTE_3_SHIFT)
#define CAN_WORD049_DATA_BYTE_3(n)     (((n) << CAN_WORD049_DATA_BYTE_3_SHIFT) & CAN_WORD049_DATA_BYTE_3_MASK)
#define CAN_WORD049_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD049_DATA_BYTE_2_MASK   (0xFF << CAN_WORD049_DATA_BYTE_2_SHIFT)
#define CAN_WORD049_DATA_BYTE_2(n)     (((n) << CAN_WORD049_DATA_BYTE_2_SHIFT) & CAN_WORD049_DATA_BYTE_2_MASK)
#define CAN_WORD049_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD049_DATA_BYTE_1_MASK   (0xFF << CAN_WORD049_DATA_BYTE_1_SHIFT)
#define CAN_WORD049_DATA_BYTE_1(n)     (((n) << CAN_WORD049_DATA_BYTE_1_SHIFT) & CAN_WORD049_DATA_BYTE_1_MASK)
#define CAN_WORD049_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD049_DATA_BYTE_0_MASK   (0xFF << CAN_WORD049_DATA_BYTE_0_SHIFT)
#define CAN_WORD049_DATA_BYTE_0(n)     (((n) << CAN_WORD049_DATA_BYTE_0_SHIFT) & CAN_WORD049_DATA_BYTE_0_MASK)

/* Message Buffer 49 WORD1 Register (WORD149) */
#define CAN_WORD149_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD149_DATA_BYTE_7_MASK   (0xFF << CAN_WORD149_DATA_BYTE_7_SHIFT)
#define CAN_WORD149_DATA_BYTE_7(n)     (((n) << CAN_WORD149_DATA_BYTE_7_SHIFT) & CAN_WORD149_DATA_BYTE_7_MASK)
#define CAN_WORD149_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD149_DATA_BYTE_6_MASK   (0xFF << CAN_WORD149_DATA_BYTE_6_SHIFT)
#define CAN_WORD149_DATA_BYTE_6(n)     (((n) << CAN_WORD149_DATA_BYTE_6_SHIFT) & CAN_WORD149_DATA_BYTE_6_MASK)
#define CAN_WORD149_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD149_DATA_BYTE_5_MASK   (0xFF << CAN_WORD149_DATA_BYTE_5_SHIFT)
#define CAN_WORD149_DATA_BYTE_5(n)     (((n) << CAN_WORD149_DATA_BYTE_5_SHIFT) & CAN_WORD149_DATA_BYTE_5_MASK)
#define CAN_WORD149_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD149_DATA_BYTE_4_MASK   (0xFF << CAN_WORD149_DATA_BYTE_4_SHIFT)
#define CAN_WORD149_DATA_BYTE_4(n)     (((n) << CAN_WORD149_DATA_BYTE_4_SHIFT) & CAN_WORD149_DATA_BYTE_4_MASK)

/* Message Buffer 50 CS Register (CS50) */
#define CAN_CS50_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS50_TIME_STAMP_MASK   (0xFFFF << CAN_CS50_TIME_STAMP_SHIFT)
#define CAN_CS50_TIME_STAMP(n)     (((n) << CAN_CS50_TIME_STAMP_SHIFT) & CAN_CS50_TIME_STAMP_MASK)
#define CAN_CS50_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS50_DLC_MASK          (0xF << CAN_CS50_DLC_SHIFT)
#define CAN_CS50_DLC(n)            (((n) << CAN_CS50_DLC_SHIFT) & CAN_CS50_DLC_MASK)
#define CAN_CS50_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS50_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS50_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS50_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS50_CODE_MASK         (0xF << CAN_CS50_CODE_SHIFT)
#define CAN_CS50_CODE(n)           (((n) << CAN_CS50_CODE_SHIFT) & CAN_CS50_CODE_MASK)
#define CAN_CS50_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS50_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS50_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 50 WORD0 Register (WORD050) */
#define CAN_WORD050_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD050_DATA_BYTE_3_MASK   (0xFF << CAN_WORD050_DATA_BYTE_3_SHIFT)
#define CAN_WORD050_DATA_BYTE_3(n)     (((n) << CAN_WORD050_DATA_BYTE_3_SHIFT) & CAN_WORD050_DATA_BYTE_3_MASK)
#define CAN_WORD050_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD050_DATA_BYTE_2_MASK   (0xFF << CAN_WORD050_DATA_BYTE_2_SHIFT)
#define CAN_WORD050_DATA_BYTE_2(n)     (((n) << CAN_WORD050_DATA_BYTE_2_SHIFT) & CAN_WORD050_DATA_BYTE_2_MASK)
#define CAN_WORD050_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD050_DATA_BYTE_1_MASK   (0xFF << CAN_WORD050_DATA_BYTE_1_SHIFT)
#define CAN_WORD050_DATA_BYTE_1(n)     (((n) << CAN_WORD050_DATA_BYTE_1_SHIFT) & CAN_WORD050_DATA_BYTE_1_MASK)
#define CAN_WORD050_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD050_DATA_BYTE_0_MASK   (0xFF << CAN_WORD050_DATA_BYTE_0_SHIFT)
#define CAN_WORD050_DATA_BYTE_0(n)     (((n) << CAN_WORD050_DATA_BYTE_0_SHIFT) & CAN_WORD050_DATA_BYTE_0_MASK)

/* Message Buffer 50 WORD1 Register (WORD150) */
#define CAN_WORD150_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD150_DATA_BYTE_7_MASK   (0xFF << CAN_WORD150_DATA_BYTE_7_SHIFT)
#define CAN_WORD150_DATA_BYTE_7(n)     (((n) << CAN_WORD150_DATA_BYTE_7_SHIFT) & CAN_WORD150_DATA_BYTE_7_MASK)
#define CAN_WORD150_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD150_DATA_BYTE_6_MASK   (0xFF << CAN_WORD150_DATA_BYTE_6_SHIFT)
#define CAN_WORD150_DATA_BYTE_6(n)     (((n) << CAN_WORD150_DATA_BYTE_6_SHIFT) & CAN_WORD150_DATA_BYTE_6_MASK)
#define CAN_WORD150_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD150_DATA_BYTE_5_MASK   (0xFF << CAN_WORD150_DATA_BYTE_5_SHIFT)
#define CAN_WORD150_DATA_BYTE_5(n)     (((n) << CAN_WORD150_DATA_BYTE_5_SHIFT) & CAN_WORD150_DATA_BYTE_5_MASK)
#define CAN_WORD150_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD150_DATA_BYTE_4_MASK   (0xFF << CAN_WORD150_DATA_BYTE_4_SHIFT)
#define CAN_WORD150_DATA_BYTE_4(n)     (((n) << CAN_WORD150_DATA_BYTE_4_SHIFT) & CAN_WORD150_DATA_BYTE_4_MASK)

/* Message Buffer 51 CS Register (CS51) */
#define CAN_CS51_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS51_TIME_STAMP_MASK   (0xFFFF << CAN_CS51_TIME_STAMP_SHIFT)
#define CAN_CS51_TIME_STAMP(n)     (((n) << CAN_CS51_TIME_STAMP_SHIFT) & CAN_CS51_TIME_STAMP_MASK)
#define CAN_CS51_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS51_DLC_MASK          (0xF << CAN_CS51_DLC_SHIFT)
#define CAN_CS51_DLC(n)            (((n) << CAN_CS51_DLC_SHIFT) & CAN_CS51_DLC_MASK)
#define CAN_CS51_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS51_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS51_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS51_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS51_CODE_MASK         (0xF << CAN_CS51_CODE_SHIFT)
#define CAN_CS51_CODE(n)           (((n) << CAN_CS51_CODE_SHIFT) & CAN_CS51_CODE_MASK)
#define CAN_CS51_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS51_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS51_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 51 WORD0 Register (WORD051) */
#define CAN_WORD051_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD051_DATA_BYTE_3_MASK   (0xFF << CAN_WORD051_DATA_BYTE_3_SHIFT)
#define CAN_WORD051_DATA_BYTE_3(n)     (((n) << CAN_WORD051_DATA_BYTE_3_SHIFT) & CAN_WORD051_DATA_BYTE_3_MASK)
#define CAN_WORD051_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD051_DATA_BYTE_2_MASK   (0xFF << CAN_WORD051_DATA_BYTE_2_SHIFT)
#define CAN_WORD051_DATA_BYTE_2(n)     (((n) << CAN_WORD051_DATA_BYTE_2_SHIFT) & CAN_WORD051_DATA_BYTE_2_MASK)
#define CAN_WORD051_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD051_DATA_BYTE_1_MASK   (0xFF << CAN_WORD051_DATA_BYTE_1_SHIFT)
#define CAN_WORD051_DATA_BYTE_1(n)     (((n) << CAN_WORD051_DATA_BYTE_1_SHIFT) & CAN_WORD051_DATA_BYTE_1_MASK)
#define CAN_WORD051_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD051_DATA_BYTE_0_MASK   (0xFF << CAN_WORD051_DATA_BYTE_0_SHIFT)
#define CAN_WORD051_DATA_BYTE_0(n)     (((n) << CAN_WORD051_DATA_BYTE_0_SHIFT) & CAN_WORD051_DATA_BYTE_0_MASK)

/* Message Buffer 51 WORD1 Register (WORD151) */
#define CAN_WORD151_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD151_DATA_BYTE_7_MASK   (0xFF << CAN_WORD151_DATA_BYTE_7_SHIFT)
#define CAN_WORD151_DATA_BYTE_7(n)     (((n) << CAN_WORD151_DATA_BYTE_7_SHIFT) & CAN_WORD151_DATA_BYTE_7_MASK)
#define CAN_WORD151_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD151_DATA_BYTE_6_MASK   (0xFF << CAN_WORD151_DATA_BYTE_6_SHIFT)
#define CAN_WORD151_DATA_BYTE_6(n)     (((n) << CAN_WORD151_DATA_BYTE_6_SHIFT) & CAN_WORD151_DATA_BYTE_6_MASK)
#define CAN_WORD151_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD151_DATA_BYTE_5_MASK   (0xFF << CAN_WORD151_DATA_BYTE_5_SHIFT)
#define CAN_WORD151_DATA_BYTE_5(n)     (((n) << CAN_WORD151_DATA_BYTE_5_SHIFT) & CAN_WORD151_DATA_BYTE_5_MASK)
#define CAN_WORD151_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD151_DATA_BYTE_4_MASK   (0xFF << CAN_WORD151_DATA_BYTE_4_SHIFT)
#define CAN_WORD151_DATA_BYTE_4(n)     (((n) << CAN_WORD151_DATA_BYTE_4_SHIFT) & CAN_WORD151_DATA_BYTE_4_MASK)

/* Message Buffer 52 CS Register (CS52) */
#define CAN_CS52_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS52_TIME_STAMP_MASK   (0xFFFF << CAN_CS52_TIME_STAMP_SHIFT)
#define CAN_CS52_TIME_STAMP(n)     (((n) << CAN_CS52_TIME_STAMP_SHIFT) & CAN_CS52_TIME_STAMP_MASK)
#define CAN_CS52_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS52_DLC_MASK          (0xF << CAN_CS52_DLC_SHIFT)
#define CAN_CS52_DLC(n)            (((n) << CAN_CS52_DLC_SHIFT) & CAN_CS52_DLC_MASK)
#define CAN_CS52_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS52_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS52_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS52_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS52_CODE_MASK         (0xF << CAN_CS52_CODE_SHIFT)
#define CAN_CS52_CODE(n)           (((n) << CAN_CS52_CODE_SHIFT) & CAN_CS52_CODE_MASK)
#define CAN_CS52_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS52_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS52_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 52 WORD0 Register (WORD052) */
#define CAN_WORD052_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD052_DATA_BYTE_3_MASK   (0xFF << CAN_WORD052_DATA_BYTE_3_SHIFT)
#define CAN_WORD052_DATA_BYTE_3(n)     (((n) << CAN_WORD052_DATA_BYTE_3_SHIFT) & CAN_WORD052_DATA_BYTE_3_MASK)
#define CAN_WORD052_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD052_DATA_BYTE_2_MASK   (0xFF << CAN_WORD052_DATA_BYTE_2_SHIFT)
#define CAN_WORD052_DATA_BYTE_2(n)     (((n) << CAN_WORD052_DATA_BYTE_2_SHIFT) & CAN_WORD052_DATA_BYTE_2_MASK)
#define CAN_WORD052_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD052_DATA_BYTE_1_MASK   (0xFF << CAN_WORD052_DATA_BYTE_1_SHIFT)
#define CAN_WORD052_DATA_BYTE_1(n)     (((n) << CAN_WORD052_DATA_BYTE_1_SHIFT) & CAN_WORD052_DATA_BYTE_1_MASK)
#define CAN_WORD052_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD052_DATA_BYTE_0_MASK   (0xFF << CAN_WORD052_DATA_BYTE_0_SHIFT)
#define CAN_WORD052_DATA_BYTE_0(n)     (((n) << CAN_WORD052_DATA_BYTE_0_SHIFT) & CAN_WORD052_DATA_BYTE_0_MASK)

/* Message Buffer 52 WORD1 Register (WORD152) */
#define CAN_WORD152_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD152_DATA_BYTE_7_MASK   (0xFF << CAN_WORD152_DATA_BYTE_7_SHIFT)
#define CAN_WORD152_DATA_BYTE_7(n)     (((n) << CAN_WORD152_DATA_BYTE_7_SHIFT) & CAN_WORD152_DATA_BYTE_7_MASK)
#define CAN_WORD152_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD152_DATA_BYTE_6_MASK   (0xFF << CAN_WORD152_DATA_BYTE_6_SHIFT)
#define CAN_WORD152_DATA_BYTE_6(n)     (((n) << CAN_WORD152_DATA_BYTE_6_SHIFT) & CAN_WORD152_DATA_BYTE_6_MASK)
#define CAN_WORD152_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD152_DATA_BYTE_5_MASK   (0xFF << CAN_WORD152_DATA_BYTE_5_SHIFT)
#define CAN_WORD152_DATA_BYTE_5(n)     (((n) << CAN_WORD152_DATA_BYTE_5_SHIFT) & CAN_WORD152_DATA_BYTE_5_MASK)
#define CAN_WORD152_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD152_DATA_BYTE_4_MASK   (0xFF << CAN_WORD152_DATA_BYTE_4_SHIFT)
#define CAN_WORD152_DATA_BYTE_4(n)     (((n) << CAN_WORD152_DATA_BYTE_4_SHIFT) & CAN_WORD152_DATA_BYTE_4_MASK)

/* Message Buffer 53 CS Register (CS53) */
#define CAN_CS53_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS53_TIME_STAMP_MASK   (0xFFFF << CAN_CS53_TIME_STAMP_SHIFT)
#define CAN_CS53_TIME_STAMP(n)     (((n) << CAN_CS53_TIME_STAMP_SHIFT) & CAN_CS53_TIME_STAMP_MASK)
#define CAN_CS53_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS53_DLC_MASK          (0xF << CAN_CS53_DLC_SHIFT)
#define CAN_CS53_DLC(n)            (((n) << CAN_CS53_DLC_SHIFT) & CAN_CS53_DLC_MASK)
#define CAN_CS53_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS53_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS53_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS53_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS53_CODE_MASK         (0xF << CAN_CS53_CODE_SHIFT)
#define CAN_CS53_CODE(n)           (((n) << CAN_CS53_CODE_SHIFT) & CAN_CS53_CODE_MASK)
#define CAN_CS53_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS53_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS53_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 53 WORD0 Register (WORD053) */
#define CAN_WORD053_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD053_DATA_BYTE_3_MASK   (0xFF << CAN_WORD053_DATA_BYTE_3_SHIFT)
#define CAN_WORD053_DATA_BYTE_3(n)     (((n) << CAN_WORD053_DATA_BYTE_3_SHIFT) & CAN_WORD053_DATA_BYTE_3_MASK)
#define CAN_WORD053_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD053_DATA_BYTE_2_MASK   (0xFF << CAN_WORD053_DATA_BYTE_2_SHIFT)
#define CAN_WORD053_DATA_BYTE_2(n)     (((n) << CAN_WORD053_DATA_BYTE_2_SHIFT) & CAN_WORD053_DATA_BYTE_2_MASK)
#define CAN_WORD053_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD053_DATA_BYTE_1_MASK   (0xFF << CAN_WORD053_DATA_BYTE_1_SHIFT)
#define CAN_WORD053_DATA_BYTE_1(n)     (((n) << CAN_WORD053_DATA_BYTE_1_SHIFT) & CAN_WORD053_DATA_BYTE_1_MASK)
#define CAN_WORD053_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD053_DATA_BYTE_0_MASK   (0xFF << CAN_WORD053_DATA_BYTE_0_SHIFT)
#define CAN_WORD053_DATA_BYTE_0(n)     (((n) << CAN_WORD053_DATA_BYTE_0_SHIFT) & CAN_WORD053_DATA_BYTE_0_MASK)

/* Message Buffer 53 WORD1 Register (WORD153) */
#define CAN_WORD153_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD153_DATA_BYTE_7_MASK   (0xFF << CAN_WORD153_DATA_BYTE_7_SHIFT)
#define CAN_WORD153_DATA_BYTE_7(n)     (((n) << CAN_WORD153_DATA_BYTE_7_SHIFT) & CAN_WORD153_DATA_BYTE_7_MASK)
#define CAN_WORD153_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD153_DATA_BYTE_6_MASK   (0xFF << CAN_WORD153_DATA_BYTE_6_SHIFT)
#define CAN_WORD153_DATA_BYTE_6(n)     (((n) << CAN_WORD153_DATA_BYTE_6_SHIFT) & CAN_WORD153_DATA_BYTE_6_MASK)
#define CAN_WORD153_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD153_DATA_BYTE_5_MASK   (0xFF << CAN_WORD153_DATA_BYTE_5_SHIFT)
#define CAN_WORD153_DATA_BYTE_5(n)     (((n) << CAN_WORD153_DATA_BYTE_5_SHIFT) & CAN_WORD153_DATA_BYTE_5_MASK)
#define CAN_WORD153_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD153_DATA_BYTE_4_MASK   (0xFF << CAN_WORD153_DATA_BYTE_4_SHIFT)
#define CAN_WORD153_DATA_BYTE_4(n)     (((n) << CAN_WORD153_DATA_BYTE_4_SHIFT) & CAN_WORD153_DATA_BYTE_4_MASK)

/* Message Buffer 54 CS Register (CS54) */
#define CAN_CS54_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS54_TIME_STAMP_MASK   (0xFFFF << CAN_CS54_TIME_STAMP_SHIFT)
#define CAN_CS54_TIME_STAMP(n)     (((n) << CAN_CS54_TIME_STAMP_SHIFT) & CAN_CS54_TIME_STAMP_MASK)
#define CAN_CS54_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS54_DLC_MASK          (0xF << CAN_CS54_DLC_SHIFT)
#define CAN_CS54_DLC(n)            (((n) << CAN_CS54_DLC_SHIFT) & CAN_CS54_DLC_MASK)
#define CAN_CS54_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS54_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS54_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS54_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS54_CODE_MASK         (0xF << CAN_CS54_CODE_SHIFT)
#define CAN_CS54_CODE(n)           (((n) << CAN_CS54_CODE_SHIFT) & CAN_CS54_CODE_MASK)
#define CAN_CS54_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS54_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS54_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 54 WORD0 Register (WORD054) */
#define CAN_WORD054_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD054_DATA_BYTE_3_MASK   (0xFF << CAN_WORD054_DATA_BYTE_3_SHIFT)
#define CAN_WORD054_DATA_BYTE_3(n)     (((n) << CAN_WORD054_DATA_BYTE_3_SHIFT) & CAN_WORD054_DATA_BYTE_3_MASK)
#define CAN_WORD054_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD054_DATA_BYTE_2_MASK   (0xFF << CAN_WORD054_DATA_BYTE_2_SHIFT)
#define CAN_WORD054_DATA_BYTE_2(n)     (((n) << CAN_WORD054_DATA_BYTE_2_SHIFT) & CAN_WORD054_DATA_BYTE_2_MASK)
#define CAN_WORD054_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD054_DATA_BYTE_1_MASK   (0xFF << CAN_WORD054_DATA_BYTE_1_SHIFT)
#define CAN_WORD054_DATA_BYTE_1(n)     (((n) << CAN_WORD054_DATA_BYTE_1_SHIFT) & CAN_WORD054_DATA_BYTE_1_MASK)
#define CAN_WORD054_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD054_DATA_BYTE_0_MASK   (0xFF << CAN_WORD054_DATA_BYTE_0_SHIFT)
#define CAN_WORD054_DATA_BYTE_0(n)     (((n) << CAN_WORD054_DATA_BYTE_0_SHIFT) & CAN_WORD054_DATA_BYTE_0_MASK)

/* Message Buffer 54 WORD1 Register (WORD154) */
#define CAN_WORD154_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD154_DATA_BYTE_7_MASK   (0xFF << CAN_WORD154_DATA_BYTE_7_SHIFT)
#define CAN_WORD154_DATA_BYTE_7(n)     (((n) << CAN_WORD154_DATA_BYTE_7_SHIFT) & CAN_WORD154_DATA_BYTE_7_MASK)
#define CAN_WORD154_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD154_DATA_BYTE_6_MASK   (0xFF << CAN_WORD154_DATA_BYTE_6_SHIFT)
#define CAN_WORD154_DATA_BYTE_6(n)     (((n) << CAN_WORD154_DATA_BYTE_6_SHIFT) & CAN_WORD154_DATA_BYTE_6_MASK)
#define CAN_WORD154_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD154_DATA_BYTE_5_MASK   (0xFF << CAN_WORD154_DATA_BYTE_5_SHIFT)
#define CAN_WORD154_DATA_BYTE_5(n)     (((n) << CAN_WORD154_DATA_BYTE_5_SHIFT) & CAN_WORD154_DATA_BYTE_5_MASK)
#define CAN_WORD154_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD154_DATA_BYTE_4_MASK   (0xFF << CAN_WORD154_DATA_BYTE_4_SHIFT)
#define CAN_WORD154_DATA_BYTE_4(n)     (((n) << CAN_WORD154_DATA_BYTE_4_SHIFT) & CAN_WORD154_DATA_BYTE_4_MASK)

/* Message Buffer 55 CS Register (CS55) */
#define CAN_CS55_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS55_TIME_STAMP_MASK   (0xFFFF << CAN_CS55_TIME_STAMP_SHIFT)
#define CAN_CS55_TIME_STAMP(n)     (((n) << CAN_CS55_TIME_STAMP_SHIFT) & CAN_CS55_TIME_STAMP_MASK)
#define CAN_CS55_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS55_DLC_MASK          (0xF << CAN_CS55_DLC_SHIFT)
#define CAN_CS55_DLC(n)            (((n) << CAN_CS55_DLC_SHIFT) & CAN_CS55_DLC_MASK)
#define CAN_CS55_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS55_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS55_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS55_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS55_CODE_MASK         (0xF << CAN_CS55_CODE_SHIFT)
#define CAN_CS55_CODE(n)           (((n) << CAN_CS55_CODE_SHIFT) & CAN_CS55_CODE_MASK)
#define CAN_CS55_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS55_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS55_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 55 WORD0 Register (WORD055) */
#define CAN_WORD055_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD055_DATA_BYTE_3_MASK   (0xFF << CAN_WORD055_DATA_BYTE_3_SHIFT)
#define CAN_WORD055_DATA_BYTE_3(n)     (((n) << CAN_WORD055_DATA_BYTE_3_SHIFT) & CAN_WORD055_DATA_BYTE_3_MASK)
#define CAN_WORD055_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD055_DATA_BYTE_2_MASK   (0xFF << CAN_WORD055_DATA_BYTE_2_SHIFT)
#define CAN_WORD055_DATA_BYTE_2(n)     (((n) << CAN_WORD055_DATA_BYTE_2_SHIFT) & CAN_WORD055_DATA_BYTE_2_MASK)
#define CAN_WORD055_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD055_DATA_BYTE_1_MASK   (0xFF << CAN_WORD055_DATA_BYTE_1_SHIFT)
#define CAN_WORD055_DATA_BYTE_1(n)     (((n) << CAN_WORD055_DATA_BYTE_1_SHIFT) & CAN_WORD055_DATA_BYTE_1_MASK)
#define CAN_WORD055_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD055_DATA_BYTE_0_MASK   (0xFF << CAN_WORD055_DATA_BYTE_0_SHIFT)
#define CAN_WORD055_DATA_BYTE_0(n)     (((n) << CAN_WORD055_DATA_BYTE_0_SHIFT) & CAN_WORD055_DATA_BYTE_0_MASK)

/* Message Buffer 55 WORD1 Register (WORD155) */
#define CAN_WORD155_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD155_DATA_BYTE_7_MASK   (0xFF << CAN_WORD155_DATA_BYTE_7_SHIFT)
#define CAN_WORD155_DATA_BYTE_7(n)     (((n) << CAN_WORD155_DATA_BYTE_7_SHIFT) & CAN_WORD155_DATA_BYTE_7_MASK)
#define CAN_WORD155_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD155_DATA_BYTE_6_MASK   (0xFF << CAN_WORD155_DATA_BYTE_6_SHIFT)
#define CAN_WORD155_DATA_BYTE_6(n)     (((n) << CAN_WORD155_DATA_BYTE_6_SHIFT) & CAN_WORD155_DATA_BYTE_6_MASK)
#define CAN_WORD155_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD155_DATA_BYTE_5_MASK   (0xFF << CAN_WORD155_DATA_BYTE_5_SHIFT)
#define CAN_WORD155_DATA_BYTE_5(n)     (((n) << CAN_WORD155_DATA_BYTE_5_SHIFT) & CAN_WORD155_DATA_BYTE_5_MASK)
#define CAN_WORD155_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD155_DATA_BYTE_4_MASK   (0xFF << CAN_WORD155_DATA_BYTE_4_SHIFT)
#define CAN_WORD155_DATA_BYTE_4(n)     (((n) << CAN_WORD155_DATA_BYTE_4_SHIFT) & CAN_WORD155_DATA_BYTE_4_MASK)

/* Message Buffer 56 CS Register (CS56) */
#define CAN_CS56_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS56_TIME_STAMP_MASK   (0xFFFF << CAN_CS56_TIME_STAMP_SHIFT)
#define CAN_CS56_TIME_STAMP(n)     (((n) << CAN_CS56_TIME_STAMP_SHIFT) & CAN_CS56_TIME_STAMP_MASK)
#define CAN_CS56_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS56_DLC_MASK          (0xF << CAN_CS56_DLC_SHIFT)
#define CAN_CS56_DLC(n)            (((n) << CAN_CS56_DLC_SHIFT) & CAN_CS56_DLC_MASK)
#define CAN_CS56_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS56_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS56_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS56_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS56_CODE_MASK         (0xF << CAN_CS56_CODE_SHIFT)
#define CAN_CS56_CODE(n)           (((n) << CAN_CS56_CODE_SHIFT) & CAN_CS56_CODE_MASK)
#define CAN_CS56_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS56_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS56_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 56 WORD0 Register (WORD056) */
#define CAN_WORD056_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD056_DATA_BYTE_3_MASK   (0xFF << CAN_WORD056_DATA_BYTE_3_SHIFT)
#define CAN_WORD056_DATA_BYTE_3(n)     (((n) << CAN_WORD056_DATA_BYTE_3_SHIFT) & CAN_WORD056_DATA_BYTE_3_MASK)
#define CAN_WORD056_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD056_DATA_BYTE_2_MASK   (0xFF << CAN_WORD056_DATA_BYTE_2_SHIFT)
#define CAN_WORD056_DATA_BYTE_2(n)     (((n) << CAN_WORD056_DATA_BYTE_2_SHIFT) & CAN_WORD056_DATA_BYTE_2_MASK)
#define CAN_WORD056_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD056_DATA_BYTE_1_MASK   (0xFF << CAN_WORD056_DATA_BYTE_1_SHIFT)
#define CAN_WORD056_DATA_BYTE_1(n)     (((n) << CAN_WORD056_DATA_BYTE_1_SHIFT) & CAN_WORD056_DATA_BYTE_1_MASK)
#define CAN_WORD056_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD056_DATA_BYTE_0_MASK   (0xFF << CAN_WORD056_DATA_BYTE_0_SHIFT)
#define CAN_WORD056_DATA_BYTE_0(n)     (((n) << CAN_WORD056_DATA_BYTE_0_SHIFT) & CAN_WORD056_DATA_BYTE_0_MASK)

/* Message Buffer 56 WORD1 Register (WORD156) */
#define CAN_WORD156_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD156_DATA_BYTE_7_MASK   (0xFF << CAN_WORD156_DATA_BYTE_7_SHIFT)
#define CAN_WORD156_DATA_BYTE_7(n)     (((n) << CAN_WORD156_DATA_BYTE_7_SHIFT) & CAN_WORD156_DATA_BYTE_7_MASK)
#define CAN_WORD156_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD156_DATA_BYTE_6_MASK   (0xFF << CAN_WORD156_DATA_BYTE_6_SHIFT)
#define CAN_WORD156_DATA_BYTE_6(n)     (((n) << CAN_WORD156_DATA_BYTE_6_SHIFT) & CAN_WORD156_DATA_BYTE_6_MASK)
#define CAN_WORD156_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD156_DATA_BYTE_5_MASK   (0xFF << CAN_WORD156_DATA_BYTE_5_SHIFT)
#define CAN_WORD156_DATA_BYTE_5(n)     (((n) << CAN_WORD156_DATA_BYTE_5_SHIFT) & CAN_WORD156_DATA_BYTE_5_MASK)
#define CAN_WORD156_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD156_DATA_BYTE_4_MASK   (0xFF << CAN_WORD156_DATA_BYTE_4_SHIFT)
#define CAN_WORD156_DATA_BYTE_4(n)     (((n) << CAN_WORD156_DATA_BYTE_4_SHIFT) & CAN_WORD156_DATA_BYTE_4_MASK)

/* Message Buffer 57 CS Register (CS57) */
#define CAN_CS57_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS57_TIME_STAMP_MASK   (0xFFFF << CAN_CS57_TIME_STAMP_SHIFT)
#define CAN_CS57_TIME_STAMP(n)     (((n) << CAN_CS57_TIME_STAMP_SHIFT) & CAN_CS57_TIME_STAMP_MASK)
#define CAN_CS57_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS57_DLC_MASK          (0xF << CAN_CS57_DLC_SHIFT)
#define CAN_CS57_DLC(n)            (((n) << CAN_CS57_DLC_SHIFT) & CAN_CS57_DLC_MASK)
#define CAN_CS57_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS57_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS57_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS57_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS57_CODE_MASK         (0xF << CAN_CS57_CODE_SHIFT)
#define CAN_CS57_CODE(n)           (((n) << CAN_CS57_CODE_SHIFT) & CAN_CS57_CODE_MASK)
#define CAN_CS57_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS57_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS57_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 57 WORD0 Register (WORD057) */
#define CAN_WORD057_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD057_DATA_BYTE_3_MASK   (0xFF << CAN_WORD057_DATA_BYTE_3_SHIFT)
#define CAN_WORD057_DATA_BYTE_3(n)     (((n) << CAN_WORD057_DATA_BYTE_3_SHIFT) & CAN_WORD057_DATA_BYTE_3_MASK)
#define CAN_WORD057_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD057_DATA_BYTE_2_MASK   (0xFF << CAN_WORD057_DATA_BYTE_2_SHIFT)
#define CAN_WORD057_DATA_BYTE_2(n)     (((n) << CAN_WORD057_DATA_BYTE_2_SHIFT) & CAN_WORD057_DATA_BYTE_2_MASK)
#define CAN_WORD057_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD057_DATA_BYTE_1_MASK   (0xFF << CAN_WORD057_DATA_BYTE_1_SHIFT)
#define CAN_WORD057_DATA_BYTE_1(n)     (((n) << CAN_WORD057_DATA_BYTE_1_SHIFT) & CAN_WORD057_DATA_BYTE_1_MASK)
#define CAN_WORD057_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD057_DATA_BYTE_0_MASK   (0xFF << CAN_WORD057_DATA_BYTE_0_SHIFT)
#define CAN_WORD057_DATA_BYTE_0(n)     (((n) << CAN_WORD057_DATA_BYTE_0_SHIFT) & CAN_WORD057_DATA_BYTE_0_MASK)

/* Message Buffer 57 WORD1 Register (WORD157) */
#define CAN_WORD157_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD157_DATA_BYTE_7_MASK   (0xFF << CAN_WORD157_DATA_BYTE_7_SHIFT)
#define CAN_WORD157_DATA_BYTE_7(n)     (((n) << CAN_WORD157_DATA_BYTE_7_SHIFT) & CAN_WORD157_DATA_BYTE_7_MASK)
#define CAN_WORD157_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD157_DATA_BYTE_6_MASK   (0xFF << CAN_WORD157_DATA_BYTE_6_SHIFT)
#define CAN_WORD157_DATA_BYTE_6(n)     (((n) << CAN_WORD157_DATA_BYTE_6_SHIFT) & CAN_WORD157_DATA_BYTE_6_MASK)
#define CAN_WORD157_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD157_DATA_BYTE_5_MASK   (0xFF << CAN_WORD157_DATA_BYTE_5_SHIFT)
#define CAN_WORD157_DATA_BYTE_5(n)     (((n) << CAN_WORD157_DATA_BYTE_5_SHIFT) & CAN_WORD157_DATA_BYTE_5_MASK)
#define CAN_WORD157_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD157_DATA_BYTE_4_MASK   (0xFF << CAN_WORD157_DATA_BYTE_4_SHIFT)
#define CAN_WORD157_DATA_BYTE_4(n)     (((n) << CAN_WORD157_DATA_BYTE_4_SHIFT) & CAN_WORD157_DATA_BYTE_4_MASK)

/* Message Buffer 58 CS Register (CS58) */
#define CAN_CS58_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS58_TIME_STAMP_MASK   (0xFFFF << CAN_CS58_TIME_STAMP_SHIFT)
#define CAN_CS58_TIME_STAMP(n)     (((n) << CAN_CS58_TIME_STAMP_SHIFT) & CAN_CS58_TIME_STAMP_MASK)
#define CAN_CS58_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS58_DLC_MASK          (0xF << CAN_CS58_DLC_SHIFT)
#define CAN_CS58_DLC(n)            (((n) << CAN_CS58_DLC_SHIFT) & CAN_CS58_DLC_MASK)
#define CAN_CS58_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS58_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS58_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS58_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS58_CODE_MASK         (0xF << CAN_CS58_CODE_SHIFT)
#define CAN_CS58_CODE(n)           (((n) << CAN_CS58_CODE_SHIFT) & CAN_CS58_CODE_MASK)
#define CAN_CS58_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS58_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS58_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 58 WORD0 Register (WORD058) */
#define CAN_WORD058_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD058_DATA_BYTE_3_MASK   (0xFF << CAN_WORD058_DATA_BYTE_3_SHIFT)
#define CAN_WORD058_DATA_BYTE_3(n)     (((n) << CAN_WORD058_DATA_BYTE_3_SHIFT) & CAN_WORD058_DATA_BYTE_3_MASK)
#define CAN_WORD058_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD058_DATA_BYTE_2_MASK   (0xFF << CAN_WORD058_DATA_BYTE_2_SHIFT)
#define CAN_WORD058_DATA_BYTE_2(n)     (((n) << CAN_WORD058_DATA_BYTE_2_SHIFT) & CAN_WORD058_DATA_BYTE_2_MASK)
#define CAN_WORD058_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD058_DATA_BYTE_1_MASK   (0xFF << CAN_WORD058_DATA_BYTE_1_SHIFT)
#define CAN_WORD058_DATA_BYTE_1(n)     (((n) << CAN_WORD058_DATA_BYTE_1_SHIFT) & CAN_WORD058_DATA_BYTE_1_MASK)
#define CAN_WORD058_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD058_DATA_BYTE_0_MASK   (0xFF << CAN_WORD058_DATA_BYTE_0_SHIFT)
#define CAN_WORD058_DATA_BYTE_0(n)     (((n) << CAN_WORD058_DATA_BYTE_0_SHIFT) & CAN_WORD058_DATA_BYTE_0_MASK)

/* Message Buffer 58 WORD1 Register (WORD158) */
#define CAN_WORD158_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD158_DATA_BYTE_7_MASK   (0xFF << CAN_WORD158_DATA_BYTE_7_SHIFT)
#define CAN_WORD158_DATA_BYTE_7(n)     (((n) << CAN_WORD158_DATA_BYTE_7_SHIFT) & CAN_WORD158_DATA_BYTE_7_MASK)
#define CAN_WORD158_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD158_DATA_BYTE_6_MASK   (0xFF << CAN_WORD158_DATA_BYTE_6_SHIFT)
#define CAN_WORD158_DATA_BYTE_6(n)     (((n) << CAN_WORD158_DATA_BYTE_6_SHIFT) & CAN_WORD158_DATA_BYTE_6_MASK)
#define CAN_WORD158_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD158_DATA_BYTE_5_MASK   (0xFF << CAN_WORD158_DATA_BYTE_5_SHIFT)
#define CAN_WORD158_DATA_BYTE_5(n)     (((n) << CAN_WORD158_DATA_BYTE_5_SHIFT) & CAN_WORD158_DATA_BYTE_5_MASK)
#define CAN_WORD158_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD158_DATA_BYTE_4_MASK   (0xFF << CAN_WORD158_DATA_BYTE_4_SHIFT)
#define CAN_WORD158_DATA_BYTE_4(n)     (((n) << CAN_WORD158_DATA_BYTE_4_SHIFT) & CAN_WORD158_DATA_BYTE_4_MASK)

/* Message Buffer 59 CS Register (CS59) */
#define CAN_CS59_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS59_TIME_STAMP_MASK   (0xFFFF << CAN_CS59_TIME_STAMP_SHIFT)
#define CAN_CS59_TIME_STAMP(n)     (((n) << CAN_CS59_TIME_STAMP_SHIFT) & CAN_CS59_TIME_STAMP_MASK)
#define CAN_CS59_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS59_DLC_MASK          (0xF << CAN_CS59_DLC_SHIFT)
#define CAN_CS59_DLC(n)            (((n) << CAN_CS59_DLC_SHIFT) & CAN_CS59_DLC_MASK)
#define CAN_CS59_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS59_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS59_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS59_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS59_CODE_MASK         (0xF << CAN_CS59_CODE_SHIFT)
#define CAN_CS59_CODE(n)           (((n) << CAN_CS59_CODE_SHIFT) & CAN_CS59_CODE_MASK)
#define CAN_CS59_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS59_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS59_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 59 WORD0 Register (WORD059) */
#define CAN_WORD059_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD059_DATA_BYTE_3_MASK   (0xFF << CAN_WORD059_DATA_BYTE_3_SHIFT)
#define CAN_WORD059_DATA_BYTE_3(n)     (((n) << CAN_WORD059_DATA_BYTE_3_SHIFT) & CAN_WORD059_DATA_BYTE_3_MASK)
#define CAN_WORD059_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD059_DATA_BYTE_2_MASK   (0xFF << CAN_WORD059_DATA_BYTE_2_SHIFT)
#define CAN_WORD059_DATA_BYTE_2(n)     (((n) << CAN_WORD059_DATA_BYTE_2_SHIFT) & CAN_WORD059_DATA_BYTE_2_MASK)
#define CAN_WORD059_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD059_DATA_BYTE_1_MASK   (0xFF << CAN_WORD059_DATA_BYTE_1_SHIFT)
#define CAN_WORD059_DATA_BYTE_1(n)     (((n) << CAN_WORD059_DATA_BYTE_1_SHIFT) & CAN_WORD059_DATA_BYTE_1_MASK)
#define CAN_WORD059_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD059_DATA_BYTE_0_MASK   (0xFF << CAN_WORD059_DATA_BYTE_0_SHIFT)
#define CAN_WORD059_DATA_BYTE_0(n)     (((n) << CAN_WORD059_DATA_BYTE_0_SHIFT) & CAN_WORD059_DATA_BYTE_0_MASK)

/* Message Buffer 59 WORD1 Register (WORD159) */
#define CAN_WORD159_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD159_DATA_BYTE_7_MASK   (0xFF << CAN_WORD159_DATA_BYTE_7_SHIFT)
#define CAN_WORD159_DATA_BYTE_7(n)     (((n) << CAN_WORD159_DATA_BYTE_7_SHIFT) & CAN_WORD159_DATA_BYTE_7_MASK)
#define CAN_WORD159_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD159_DATA_BYTE_6_MASK   (0xFF << CAN_WORD159_DATA_BYTE_6_SHIFT)
#define CAN_WORD159_DATA_BYTE_6(n)     (((n) << CAN_WORD159_DATA_BYTE_6_SHIFT) & CAN_WORD159_DATA_BYTE_6_MASK)
#define CAN_WORD159_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD159_DATA_BYTE_5_MASK   (0xFF << CAN_WORD159_DATA_BYTE_5_SHIFT)
#define CAN_WORD159_DATA_BYTE_5(n)     (((n) << CAN_WORD159_DATA_BYTE_5_SHIFT) & CAN_WORD159_DATA_BYTE_5_MASK)
#define CAN_WORD159_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD159_DATA_BYTE_4_MASK   (0xFF << CAN_WORD159_DATA_BYTE_4_SHIFT)
#define CAN_WORD159_DATA_BYTE_4(n)     (((n) << CAN_WORD159_DATA_BYTE_4_SHIFT) & CAN_WORD159_DATA_BYTE_4_MASK)

/* Message Buffer 60 CS Register (CS60) */
#define CAN_CS60_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS60_TIME_STAMP_MASK   (0xFFFF << CAN_CS60_TIME_STAMP_SHIFT)
#define CAN_CS60_TIME_STAMP(n)     (((n) << CAN_CS60_TIME_STAMP_SHIFT) & CAN_CS60_TIME_STAMP_MASK)
#define CAN_CS60_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS60_DLC_MASK          (0xF << CAN_CS60_DLC_SHIFT)
#define CAN_CS60_DLC(n)            (((n) << CAN_CS60_DLC_SHIFT) & CAN_CS60_DLC_MASK)
#define CAN_CS60_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS60_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS60_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS60_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS60_CODE_MASK         (0xF << CAN_CS60_CODE_SHIFT)
#define CAN_CS60_CODE(n)           (((n) << CAN_CS60_CODE_SHIFT) & CAN_CS60_CODE_MASK)
#define CAN_CS60_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS60_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS60_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 60 WORD0 Register (WORD060) */
#define CAN_WORD060_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD060_DATA_BYTE_3_MASK   (0xFF << CAN_WORD060_DATA_BYTE_3_SHIFT)
#define CAN_WORD060_DATA_BYTE_3(n)     (((n) << CAN_WORD060_DATA_BYTE_3_SHIFT) & CAN_WORD060_DATA_BYTE_3_MASK)
#define CAN_WORD060_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD060_DATA_BYTE_2_MASK   (0xFF << CAN_WORD060_DATA_BYTE_2_SHIFT)
#define CAN_WORD060_DATA_BYTE_2(n)     (((n) << CAN_WORD060_DATA_BYTE_2_SHIFT) & CAN_WORD060_DATA_BYTE_2_MASK)
#define CAN_WORD060_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD060_DATA_BYTE_1_MASK   (0xFF << CAN_WORD060_DATA_BYTE_1_SHIFT)
#define CAN_WORD060_DATA_BYTE_1(n)     (((n) << CAN_WORD060_DATA_BYTE_1_SHIFT) & CAN_WORD060_DATA_BYTE_1_MASK)
#define CAN_WORD060_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD060_DATA_BYTE_0_MASK   (0xFF << CAN_WORD060_DATA_BYTE_0_SHIFT)
#define CAN_WORD060_DATA_BYTE_0(n)     (((n) << CAN_WORD060_DATA_BYTE_0_SHIFT) & CAN_WORD060_DATA_BYTE_0_MASK)

/* Message Buffer 60 WORD1 Register (WORD160) */
#define CAN_WORD160_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD160_DATA_BYTE_7_MASK   (0xFF << CAN_WORD160_DATA_BYTE_7_SHIFT)
#define CAN_WORD160_DATA_BYTE_7(n)     (((n) << CAN_WORD160_DATA_BYTE_7_SHIFT) & CAN_WORD160_DATA_BYTE_7_MASK)
#define CAN_WORD160_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD160_DATA_BYTE_6_MASK   (0xFF << CAN_WORD160_DATA_BYTE_6_SHIFT)
#define CAN_WORD160_DATA_BYTE_6(n)     (((n) << CAN_WORD160_DATA_BYTE_6_SHIFT) & CAN_WORD160_DATA_BYTE_6_MASK)
#define CAN_WORD160_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD160_DATA_BYTE_5_MASK   (0xFF << CAN_WORD160_DATA_BYTE_5_SHIFT)
#define CAN_WORD160_DATA_BYTE_5(n)     (((n) << CAN_WORD160_DATA_BYTE_5_SHIFT) & CAN_WORD160_DATA_BYTE_5_MASK)
#define CAN_WORD160_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD160_DATA_BYTE_4_MASK   (0xFF << CAN_WORD160_DATA_BYTE_4_SHIFT)
#define CAN_WORD160_DATA_BYTE_4(n)     (((n) << CAN_WORD160_DATA_BYTE_4_SHIFT) & CAN_WORD160_DATA_BYTE_4_MASK)

/* Message Buffer 61 CS Register (CS61) */
#define CAN_CS61_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS61_TIME_STAMP_MASK   (0xFFFF << CAN_CS61_TIME_STAMP_SHIFT)
#define CAN_CS61_TIME_STAMP(n)     (((n) << CAN_CS61_TIME_STAMP_SHIFT) & CAN_CS61_TIME_STAMP_MASK)
#define CAN_CS61_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS61_DLC_MASK          (0xF << CAN_CS61_DLC_SHIFT)
#define CAN_CS61_DLC(n)            (((n) << CAN_CS61_DLC_SHIFT) & CAN_CS61_DLC_MASK)
#define CAN_CS61_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS61_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS61_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS61_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS61_CODE_MASK         (0xF << CAN_CS61_CODE_SHIFT)
#define CAN_CS61_CODE(n)           (((n) << CAN_CS61_CODE_SHIFT) & CAN_CS61_CODE_MASK)
#define CAN_CS61_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS61_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS61_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 61 WORD0 Register (WORD061) */
#define CAN_WORD061_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD061_DATA_BYTE_3_MASK   (0xFF << CAN_WORD061_DATA_BYTE_3_SHIFT)
#define CAN_WORD061_DATA_BYTE_3(n)     (((n) << CAN_WORD061_DATA_BYTE_3_SHIFT) & CAN_WORD061_DATA_BYTE_3_MASK)
#define CAN_WORD061_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD061_DATA_BYTE_2_MASK   (0xFF << CAN_WORD061_DATA_BYTE_2_SHIFT)
#define CAN_WORD061_DATA_BYTE_2(n)     (((n) << CAN_WORD061_DATA_BYTE_2_SHIFT) & CAN_WORD061_DATA_BYTE_2_MASK)
#define CAN_WORD061_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD061_DATA_BYTE_1_MASK   (0xFF << CAN_WORD061_DATA_BYTE_1_SHIFT)
#define CAN_WORD061_DATA_BYTE_1(n)     (((n) << CAN_WORD061_DATA_BYTE_1_SHIFT) & CAN_WORD061_DATA_BYTE_1_MASK)
#define CAN_WORD061_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD061_DATA_BYTE_0_MASK   (0xFF << CAN_WORD061_DATA_BYTE_0_SHIFT)
#define CAN_WORD061_DATA_BYTE_0(n)     (((n) << CAN_WORD061_DATA_BYTE_0_SHIFT) & CAN_WORD061_DATA_BYTE_0_MASK)

/* Message Buffer 61 WORD1 Register (WORD161) */
#define CAN_WORD161_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD161_DATA_BYTE_7_MASK   (0xFF << CAN_WORD161_DATA_BYTE_7_SHIFT)
#define CAN_WORD161_DATA_BYTE_7(n)     (((n) << CAN_WORD161_DATA_BYTE_7_SHIFT) & CAN_WORD161_DATA_BYTE_7_MASK)
#define CAN_WORD161_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD161_DATA_BYTE_6_MASK   (0xFF << CAN_WORD161_DATA_BYTE_6_SHIFT)
#define CAN_WORD161_DATA_BYTE_6(n)     (((n) << CAN_WORD161_DATA_BYTE_6_SHIFT) & CAN_WORD161_DATA_BYTE_6_MASK)
#define CAN_WORD161_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD161_DATA_BYTE_5_MASK   (0xFF << CAN_WORD161_DATA_BYTE_5_SHIFT)
#define CAN_WORD161_DATA_BYTE_5(n)     (((n) << CAN_WORD161_DATA_BYTE_5_SHIFT) & CAN_WORD161_DATA_BYTE_5_MASK)
#define CAN_WORD161_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD161_DATA_BYTE_4_MASK   (0xFF << CAN_WORD161_DATA_BYTE_4_SHIFT)
#define CAN_WORD161_DATA_BYTE_4(n)     (((n) << CAN_WORD161_DATA_BYTE_4_SHIFT) & CAN_WORD161_DATA_BYTE_4_MASK)

/* Message Buffer 62 CS Register (CS62) */
#define CAN_CS62_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS62_TIME_STAMP_MASK   (0xFFFF << CAN_CS62_TIME_STAMP_SHIFT)
#define CAN_CS62_TIME_STAMP(n)     (((n) << CAN_CS62_TIME_STAMP_SHIFT) & CAN_CS62_TIME_STAMP_MASK)
#define CAN_CS62_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS62_DLC_MASK          (0xF << CAN_CS62_DLC_SHIFT)
#define CAN_CS62_DLC(n)            (((n) << CAN_CS62_DLC_SHIFT) & CAN_CS62_DLC_MASK)
#define CAN_CS62_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS62_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS62_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS62_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS62_CODE_MASK         (0xF << CAN_CS62_CODE_SHIFT)
#define CAN_CS62_CODE(n)           (((n) << CAN_CS62_CODE_SHIFT) & CAN_CS62_CODE_MASK)
#define CAN_CS62_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS62_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS62_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 62 WORD0 Register (WORD062) */
#define CAN_WORD062_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD062_DATA_BYTE_3_MASK   (0xFF << CAN_WORD062_DATA_BYTE_3_SHIFT)
#define CAN_WORD062_DATA_BYTE_3(n)     (((n) << CAN_WORD062_DATA_BYTE_3_SHIFT) & CAN_WORD062_DATA_BYTE_3_MASK)
#define CAN_WORD062_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD062_DATA_BYTE_2_MASK   (0xFF << CAN_WORD062_DATA_BYTE_2_SHIFT)
#define CAN_WORD062_DATA_BYTE_2(n)     (((n) << CAN_WORD062_DATA_BYTE_2_SHIFT) & CAN_WORD062_DATA_BYTE_2_MASK)
#define CAN_WORD062_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD062_DATA_BYTE_1_MASK   (0xFF << CAN_WORD062_DATA_BYTE_1_SHIFT)
#define CAN_WORD062_DATA_BYTE_1(n)     (((n) << CAN_WORD062_DATA_BYTE_1_SHIFT) & CAN_WORD062_DATA_BYTE_1_MASK)
#define CAN_WORD062_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD062_DATA_BYTE_0_MASK   (0xFF << CAN_WORD062_DATA_BYTE_0_SHIFT)
#define CAN_WORD062_DATA_BYTE_0(n)     (((n) << CAN_WORD062_DATA_BYTE_0_SHIFT) & CAN_WORD062_DATA_BYTE_0_MASK)

/* Message Buffer 62 WORD1 Register (WORD162) */
#define CAN_WORD162_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD162_DATA_BYTE_7_MASK   (0xFF << CAN_WORD162_DATA_BYTE_7_SHIFT)
#define CAN_WORD162_DATA_BYTE_7(n)     (((n) << CAN_WORD162_DATA_BYTE_7_SHIFT) & CAN_WORD162_DATA_BYTE_7_MASK)
#define CAN_WORD162_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD162_DATA_BYTE_6_MASK   (0xFF << CAN_WORD162_DATA_BYTE_6_SHIFT)
#define CAN_WORD162_DATA_BYTE_6(n)     (((n) << CAN_WORD162_DATA_BYTE_6_SHIFT) & CAN_WORD162_DATA_BYTE_6_MASK)
#define CAN_WORD162_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD162_DATA_BYTE_5_MASK   (0xFF << CAN_WORD162_DATA_BYTE_5_SHIFT)
#define CAN_WORD162_DATA_BYTE_5(n)     (((n) << CAN_WORD162_DATA_BYTE_5_SHIFT) & CAN_WORD162_DATA_BYTE_5_MASK)
#define CAN_WORD162_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD162_DATA_BYTE_4_MASK   (0xFF << CAN_WORD162_DATA_BYTE_4_SHIFT)
#define CAN_WORD162_DATA_BYTE_4(n)     (((n) << CAN_WORD162_DATA_BYTE_4_SHIFT) & CAN_WORD162_DATA_BYTE_4_MASK)

/* Message Buffer 63 CS Register (CS63) */
#define CAN_CS63_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS63_TIME_STAMP_MASK   (0xFFFF << CAN_CS63_TIME_STAMP_SHIFT)
#define CAN_CS63_TIME_STAMP(n)     (((n) << CAN_CS63_TIME_STAMP_SHIFT) & CAN_CS63_TIME_STAMP_MASK)
#define CAN_CS63_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS63_DLC_MASK          (0xF << CAN_CS63_DLC_SHIFT)
#define CAN_CS63_DLC(n)            (((n) << CAN_CS63_DLC_SHIFT) & CAN_CS63_DLC_MASK)
#define CAN_CS63_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS63_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS63_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS63_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS63_CODE_MASK         (0xF << CAN_CS63_CODE_SHIFT)
#define CAN_CS63_CODE(n)           (((n) << CAN_CS63_CODE_SHIFT) & CAN_CS63_CODE_MASK)
#define CAN_CS63_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS63_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS63_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 63 WORD0 Register (WORD063) */
#define CAN_WORD063_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD063_DATA_BYTE_3_MASK   (0xFF << CAN_WORD063_DATA_BYTE_3_SHIFT)
#define CAN_WORD063_DATA_BYTE_3(n)     (((n) << CAN_WORD063_DATA_BYTE_3_SHIFT) & CAN_WORD063_DATA_BYTE_3_MASK)
#define CAN_WORD063_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD063_DATA_BYTE_2_MASK   (0xFF << CAN_WORD063_DATA_BYTE_2_SHIFT)
#define CAN_WORD063_DATA_BYTE_2(n)     (((n) << CAN_WORD063_DATA_BYTE_2_SHIFT) & CAN_WORD063_DATA_BYTE_2_MASK)
#define CAN_WORD063_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD063_DATA_BYTE_1_MASK   (0xFF << CAN_WORD063_DATA_BYTE_1_SHIFT)
#define CAN_WORD063_DATA_BYTE_1(n)     (((n) << CAN_WORD063_DATA_BYTE_1_SHIFT) & CAN_WORD063_DATA_BYTE_1_MASK)
#define CAN_WORD063_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD063_DATA_BYTE_0_MASK   (0xFF << CAN_WORD063_DATA_BYTE_0_SHIFT)
#define CAN_WORD063_DATA_BYTE_0(n)     (((n) << CAN_WORD063_DATA_BYTE_0_SHIFT) & CAN_WORD063_DATA_BYTE_0_MASK)

/* Message Buffer 63 WORD1 Register (WORD163) */
#define CAN_WORD163_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD163_DATA_BYTE_7_MASK   (0xFF << CAN_WORD163_DATA_BYTE_7_SHIFT)
#define CAN_WORD163_DATA_BYTE_7(n)     (((n) << CAN_WORD163_DATA_BYTE_7_SHIFT) & CAN_WORD163_DATA_BYTE_7_MASK)
#define CAN_WORD163_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD163_DATA_BYTE_6_MASK   (0xFF << CAN_WORD163_DATA_BYTE_6_SHIFT)
#define CAN_WORD163_DATA_BYTE_6(n)     (((n) << CAN_WORD163_DATA_BYTE_6_SHIFT) & CAN_WORD163_DATA_BYTE_6_MASK)
#define CAN_WORD163_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD163_DATA_BYTE_5_MASK   (0xFF << CAN_WORD163_DATA_BYTE_5_SHIFT)
#define CAN_WORD163_DATA_BYTE_5(n)     (((n) << CAN_WORD163_DATA_BYTE_5_SHIFT) & CAN_WORD163_DATA_BYTE_5_MASK)
#define CAN_WORD163_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD163_DATA_BYTE_4_MASK   (0xFF << CAN_WORD163_DATA_BYTE_4_SHIFT)
#define CAN_WORD163_DATA_BYTE_4(n)     (((n) << CAN_WORD163_DATA_BYTE_4_SHIFT) & CAN_WORD163_DATA_BYTE_4_MASK)

/* Message Buffer 64 CS Register (CS64) */
#define CAN_CS64_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS64_TIME_STAMP_MASK   (0xFFFF << CAN_CS64_TIME_STAMP_SHIFT)
#define CAN_CS64_TIME_STAMP(n)     (((n) << CAN_CS64_TIME_STAMP_SHIFT) & CAN_CS64_TIME_STAMP_MASK)
#define CAN_CS64_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS64_DLC_MASK          (0xF << CAN_CS64_DLC_SHIFT)
#define CAN_CS64_DLC(n)            (((n) << CAN_CS64_DLC_SHIFT) & CAN_CS64_DLC_MASK)
#define CAN_CS64_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS64_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS64_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS64_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS64_CODE_MASK         (0xF << CAN_CS64_CODE_SHIFT)
#define CAN_CS64_CODE(n)           (((n) << CAN_CS64_CODE_SHIFT) & CAN_CS64_CODE_MASK)
#define CAN_CS64_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS64_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS64_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 64 WORD0 Register (WORD064) */
#define CAN_WORD064_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD064_DATA_BYTE_3_MASK   (0xFF << CAN_WORD064_DATA_BYTE_3_SHIFT)
#define CAN_WORD064_DATA_BYTE_3(n)     (((n) << CAN_WORD064_DATA_BYTE_3_SHIFT) & CAN_WORD064_DATA_BYTE_3_MASK)
#define CAN_WORD064_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD064_DATA_BYTE_2_MASK   (0xFF << CAN_WORD064_DATA_BYTE_2_SHIFT)
#define CAN_WORD064_DATA_BYTE_2(n)     (((n) << CAN_WORD064_DATA_BYTE_2_SHIFT) & CAN_WORD064_DATA_BYTE_2_MASK)
#define CAN_WORD064_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD064_DATA_BYTE_1_MASK   (0xFF << CAN_WORD064_DATA_BYTE_1_SHIFT)
#define CAN_WORD064_DATA_BYTE_1(n)     (((n) << CAN_WORD064_DATA_BYTE_1_SHIFT) & CAN_WORD064_DATA_BYTE_1_MASK)
#define CAN_WORD064_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD064_DATA_BYTE_0_MASK   (0xFF << CAN_WORD064_DATA_BYTE_0_SHIFT)
#define CAN_WORD064_DATA_BYTE_0(n)     (((n) << CAN_WORD064_DATA_BYTE_0_SHIFT) & CAN_WORD064_DATA_BYTE_0_MASK)

/* Message Buffer 64 WORD1 Register (WORD164) */
#define CAN_WORD164_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD164_DATA_BYTE_7_MASK   (0xFF << CAN_WORD164_DATA_BYTE_7_SHIFT)
#define CAN_WORD164_DATA_BYTE_7(n)     (((n) << CAN_WORD164_DATA_BYTE_7_SHIFT) & CAN_WORD164_DATA_BYTE_7_MASK)
#define CAN_WORD164_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD164_DATA_BYTE_6_MASK   (0xFF << CAN_WORD164_DATA_BYTE_6_SHIFT)
#define CAN_WORD164_DATA_BYTE_6(n)     (((n) << CAN_WORD164_DATA_BYTE_6_SHIFT) & CAN_WORD164_DATA_BYTE_6_MASK)
#define CAN_WORD164_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD164_DATA_BYTE_5_MASK   (0xFF << CAN_WORD164_DATA_BYTE_5_SHIFT)
#define CAN_WORD164_DATA_BYTE_5(n)     (((n) << CAN_WORD164_DATA_BYTE_5_SHIFT) & CAN_WORD164_DATA_BYTE_5_MASK)
#define CAN_WORD164_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD164_DATA_BYTE_4_MASK   (0xFF << CAN_WORD164_DATA_BYTE_4_SHIFT)
#define CAN_WORD164_DATA_BYTE_4(n)     (((n) << CAN_WORD164_DATA_BYTE_4_SHIFT) & CAN_WORD164_DATA_BYTE_4_MASK)

/* Message Buffer 65 CS Register (CS65) */
#define CAN_CS65_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS65_TIME_STAMP_MASK   (0xFFFF << CAN_CS65_TIME_STAMP_SHIFT)
#define CAN_CS65_TIME_STAMP(n)     (((n) << CAN_CS65_TIME_STAMP_SHIFT) & CAN_CS65_TIME_STAMP_MASK)
#define CAN_CS65_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS65_DLC_MASK          (0xF << CAN_CS65_DLC_SHIFT)
#define CAN_CS65_DLC(n)            (((n) << CAN_CS65_DLC_SHIFT) & CAN_CS65_DLC_MASK)
#define CAN_CS65_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS65_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS65_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS65_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS65_CODE_MASK         (0xF << CAN_CS65_CODE_SHIFT)
#define CAN_CS65_CODE(n)           (((n) << CAN_CS65_CODE_SHIFT) & CAN_CS65_CODE_MASK)
#define CAN_CS65_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS65_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS65_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 65 WORD0 Register (WORD065) */
#define CAN_WORD065_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD065_DATA_BYTE_3_MASK   (0xFF << CAN_WORD065_DATA_BYTE_3_SHIFT)
#define CAN_WORD065_DATA_BYTE_3(n)     (((n) << CAN_WORD065_DATA_BYTE_3_SHIFT) & CAN_WORD065_DATA_BYTE_3_MASK)
#define CAN_WORD065_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD065_DATA_BYTE_2_MASK   (0xFF << CAN_WORD065_DATA_BYTE_2_SHIFT)
#define CAN_WORD065_DATA_BYTE_2(n)     (((n) << CAN_WORD065_DATA_BYTE_2_SHIFT) & CAN_WORD065_DATA_BYTE_2_MASK)
#define CAN_WORD065_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD065_DATA_BYTE_1_MASK   (0xFF << CAN_WORD065_DATA_BYTE_1_SHIFT)
#define CAN_WORD065_DATA_BYTE_1(n)     (((n) << CAN_WORD065_DATA_BYTE_1_SHIFT) & CAN_WORD065_DATA_BYTE_1_MASK)
#define CAN_WORD065_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD065_DATA_BYTE_0_MASK   (0xFF << CAN_WORD065_DATA_BYTE_0_SHIFT)
#define CAN_WORD065_DATA_BYTE_0(n)     (((n) << CAN_WORD065_DATA_BYTE_0_SHIFT) & CAN_WORD065_DATA_BYTE_0_MASK)

/* Message Buffer 65 WORD1 Register (WORD165) */
#define CAN_WORD165_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD165_DATA_BYTE_7_MASK   (0xFF << CAN_WORD165_DATA_BYTE_7_SHIFT)
#define CAN_WORD165_DATA_BYTE_7(n)     (((n) << CAN_WORD165_DATA_BYTE_7_SHIFT) & CAN_WORD165_DATA_BYTE_7_MASK)
#define CAN_WORD165_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD165_DATA_BYTE_6_MASK   (0xFF << CAN_WORD165_DATA_BYTE_6_SHIFT)
#define CAN_WORD165_DATA_BYTE_6(n)     (((n) << CAN_WORD165_DATA_BYTE_6_SHIFT) & CAN_WORD165_DATA_BYTE_6_MASK)
#define CAN_WORD165_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD165_DATA_BYTE_5_MASK   (0xFF << CAN_WORD165_DATA_BYTE_5_SHIFT)
#define CAN_WORD165_DATA_BYTE_5(n)     (((n) << CAN_WORD165_DATA_BYTE_5_SHIFT) & CAN_WORD165_DATA_BYTE_5_MASK)
#define CAN_WORD165_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD165_DATA_BYTE_4_MASK   (0xFF << CAN_WORD165_DATA_BYTE_4_SHIFT)
#define CAN_WORD165_DATA_BYTE_4(n)     (((n) << CAN_WORD165_DATA_BYTE_4_SHIFT) & CAN_WORD165_DATA_BYTE_4_MASK)

/* Message Buffer 66 CS Register (CS66) */
#define CAN_CS66_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS66_TIME_STAMP_MASK   (0xFFFF << CAN_CS66_TIME_STAMP_SHIFT)
#define CAN_CS66_TIME_STAMP(n)     (((n) << CAN_CS66_TIME_STAMP_SHIFT) & CAN_CS66_TIME_STAMP_MASK)
#define CAN_CS66_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS66_DLC_MASK          (0xF << CAN_CS66_DLC_SHIFT)
#define CAN_CS66_DLC(n)            (((n) << CAN_CS66_DLC_SHIFT) & CAN_CS66_DLC_MASK)
#define CAN_CS66_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS66_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS66_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS66_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS66_CODE_MASK         (0xF << CAN_CS66_CODE_SHIFT)
#define CAN_CS66_CODE(n)           (((n) << CAN_CS66_CODE_SHIFT) & CAN_CS66_CODE_MASK)
#define CAN_CS66_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS66_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS66_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 66 WORD0 Register (WORD066) */
#define CAN_WORD066_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD066_DATA_BYTE_3_MASK   (0xFF << CAN_WORD066_DATA_BYTE_3_SHIFT)
#define CAN_WORD066_DATA_BYTE_3(n)     (((n) << CAN_WORD066_DATA_BYTE_3_SHIFT) & CAN_WORD066_DATA_BYTE_3_MASK)
#define CAN_WORD066_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD066_DATA_BYTE_2_MASK   (0xFF << CAN_WORD066_DATA_BYTE_2_SHIFT)
#define CAN_WORD066_DATA_BYTE_2(n)     (((n) << CAN_WORD066_DATA_BYTE_2_SHIFT) & CAN_WORD066_DATA_BYTE_2_MASK)
#define CAN_WORD066_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD066_DATA_BYTE_1_MASK   (0xFF << CAN_WORD066_DATA_BYTE_1_SHIFT)
#define CAN_WORD066_DATA_BYTE_1(n)     (((n) << CAN_WORD066_DATA_BYTE_1_SHIFT) & CAN_WORD066_DATA_BYTE_1_MASK)
#define CAN_WORD066_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD066_DATA_BYTE_0_MASK   (0xFF << CAN_WORD066_DATA_BYTE_0_SHIFT)
#define CAN_WORD066_DATA_BYTE_0(n)     (((n) << CAN_WORD066_DATA_BYTE_0_SHIFT) & CAN_WORD066_DATA_BYTE_0_MASK)

/* Message Buffer 66 WORD1 Register (WORD166) */
#define CAN_WORD166_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD166_DATA_BYTE_7_MASK   (0xFF << CAN_WORD166_DATA_BYTE_7_SHIFT)
#define CAN_WORD166_DATA_BYTE_7(n)     (((n) << CAN_WORD166_DATA_BYTE_7_SHIFT) & CAN_WORD166_DATA_BYTE_7_MASK)
#define CAN_WORD166_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD166_DATA_BYTE_6_MASK   (0xFF << CAN_WORD166_DATA_BYTE_6_SHIFT)
#define CAN_WORD166_DATA_BYTE_6(n)     (((n) << CAN_WORD166_DATA_BYTE_6_SHIFT) & CAN_WORD166_DATA_BYTE_6_MASK)
#define CAN_WORD166_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD166_DATA_BYTE_5_MASK   (0xFF << CAN_WORD166_DATA_BYTE_5_SHIFT)
#define CAN_WORD166_DATA_BYTE_5(n)     (((n) << CAN_WORD166_DATA_BYTE_5_SHIFT) & CAN_WORD166_DATA_BYTE_5_MASK)
#define CAN_WORD166_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD166_DATA_BYTE_4_MASK   (0xFF << CAN_WORD166_DATA_BYTE_4_SHIFT)
#define CAN_WORD166_DATA_BYTE_4(n)     (((n) << CAN_WORD166_DATA_BYTE_4_SHIFT) & CAN_WORD166_DATA_BYTE_4_MASK)

/* Message Buffer 67 CS Register (CS67) */
#define CAN_CS67_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS67_TIME_STAMP_MASK   (0xFFFF << CAN_CS67_TIME_STAMP_SHIFT)
#define CAN_CS67_TIME_STAMP(n)     (((n) << CAN_CS67_TIME_STAMP_SHIFT) & CAN_CS67_TIME_STAMP_MASK)
#define CAN_CS67_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS67_DLC_MASK          (0xF << CAN_CS67_DLC_SHIFT)
#define CAN_CS67_DLC(n)            (((n) << CAN_CS67_DLC_SHIFT) & CAN_CS67_DLC_MASK)
#define CAN_CS67_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS67_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS67_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS67_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS67_CODE_MASK         (0xF << CAN_CS67_CODE_SHIFT)
#define CAN_CS67_CODE(n)           (((n) << CAN_CS67_CODE_SHIFT) & CAN_CS67_CODE_MASK)
#define CAN_CS67_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS67_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS67_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 67 WORD0 Register (WORD067) */
#define CAN_WORD067_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD067_DATA_BYTE_3_MASK   (0xFF << CAN_WORD067_DATA_BYTE_3_SHIFT)
#define CAN_WORD067_DATA_BYTE_3(n)     (((n) << CAN_WORD067_DATA_BYTE_3_SHIFT) & CAN_WORD067_DATA_BYTE_3_MASK)
#define CAN_WORD067_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD067_DATA_BYTE_2_MASK   (0xFF << CAN_WORD067_DATA_BYTE_2_SHIFT)
#define CAN_WORD067_DATA_BYTE_2(n)     (((n) << CAN_WORD067_DATA_BYTE_2_SHIFT) & CAN_WORD067_DATA_BYTE_2_MASK)
#define CAN_WORD067_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD067_DATA_BYTE_1_MASK   (0xFF << CAN_WORD067_DATA_BYTE_1_SHIFT)
#define CAN_WORD067_DATA_BYTE_1(n)     (((n) << CAN_WORD067_DATA_BYTE_1_SHIFT) & CAN_WORD067_DATA_BYTE_1_MASK)
#define CAN_WORD067_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD067_DATA_BYTE_0_MASK   (0xFF << CAN_WORD067_DATA_BYTE_0_SHIFT)
#define CAN_WORD067_DATA_BYTE_0(n)     (((n) << CAN_WORD067_DATA_BYTE_0_SHIFT) & CAN_WORD067_DATA_BYTE_0_MASK)

/* Message Buffer 67 WORD1 Register (WORD167) */
#define CAN_WORD167_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD167_DATA_BYTE_7_MASK   (0xFF << CAN_WORD167_DATA_BYTE_7_SHIFT)
#define CAN_WORD167_DATA_BYTE_7(n)     (((n) << CAN_WORD167_DATA_BYTE_7_SHIFT) & CAN_WORD167_DATA_BYTE_7_MASK)
#define CAN_WORD167_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD167_DATA_BYTE_6_MASK   (0xFF << CAN_WORD167_DATA_BYTE_6_SHIFT)
#define CAN_WORD167_DATA_BYTE_6(n)     (((n) << CAN_WORD167_DATA_BYTE_6_SHIFT) & CAN_WORD167_DATA_BYTE_6_MASK)
#define CAN_WORD167_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD167_DATA_BYTE_5_MASK   (0xFF << CAN_WORD167_DATA_BYTE_5_SHIFT)
#define CAN_WORD167_DATA_BYTE_5(n)     (((n) << CAN_WORD167_DATA_BYTE_5_SHIFT) & CAN_WORD167_DATA_BYTE_5_MASK)
#define CAN_WORD167_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD167_DATA_BYTE_4_MASK   (0xFF << CAN_WORD167_DATA_BYTE_4_SHIFT)
#define CAN_WORD167_DATA_BYTE_4(n)     (((n) << CAN_WORD167_DATA_BYTE_4_SHIFT) & CAN_WORD167_DATA_BYTE_4_MASK)

/* Message Buffer 68 CS Register (CS68) */
#define CAN_CS68_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS68_TIME_STAMP_MASK   (0xFFFF << CAN_CS68_TIME_STAMP_SHIFT)
#define CAN_CS68_TIME_STAMP(n)     (((n) << CAN_CS68_TIME_STAMP_SHIFT) & CAN_CS68_TIME_STAMP_MASK)
#define CAN_CS68_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS68_DLC_MASK          (0xF << CAN_CS68_DLC_SHIFT)
#define CAN_CS68_DLC(n)            (((n) << CAN_CS68_DLC_SHIFT) & CAN_CS68_DLC_MASK)
#define CAN_CS68_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS68_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS68_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS68_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS68_CODE_MASK         (0xF << CAN_CS68_CODE_SHIFT)
#define CAN_CS68_CODE(n)           (((n) << CAN_CS68_CODE_SHIFT) & CAN_CS68_CODE_MASK)
#define CAN_CS68_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS68_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS68_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 68 WORD0 Register (WORD068) */
#define CAN_WORD068_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD068_DATA_BYTE_3_MASK   (0xFF << CAN_WORD068_DATA_BYTE_3_SHIFT)
#define CAN_WORD068_DATA_BYTE_3(n)     (((n) << CAN_WORD068_DATA_BYTE_3_SHIFT) & CAN_WORD068_DATA_BYTE_3_MASK)
#define CAN_WORD068_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD068_DATA_BYTE_2_MASK   (0xFF << CAN_WORD068_DATA_BYTE_2_SHIFT)
#define CAN_WORD068_DATA_BYTE_2(n)     (((n) << CAN_WORD068_DATA_BYTE_2_SHIFT) & CAN_WORD068_DATA_BYTE_2_MASK)
#define CAN_WORD068_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD068_DATA_BYTE_1_MASK   (0xFF << CAN_WORD068_DATA_BYTE_1_SHIFT)
#define CAN_WORD068_DATA_BYTE_1(n)     (((n) << CAN_WORD068_DATA_BYTE_1_SHIFT) & CAN_WORD068_DATA_BYTE_1_MASK)
#define CAN_WORD068_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD068_DATA_BYTE_0_MASK   (0xFF << CAN_WORD068_DATA_BYTE_0_SHIFT)
#define CAN_WORD068_DATA_BYTE_0(n)     (((n) << CAN_WORD068_DATA_BYTE_0_SHIFT) & CAN_WORD068_DATA_BYTE_0_MASK)

/* Message Buffer 68 WORD1 Register (WORD168) */
#define CAN_WORD168_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD168_DATA_BYTE_7_MASK   (0xFF << CAN_WORD168_DATA_BYTE_7_SHIFT)
#define CAN_WORD168_DATA_BYTE_7(n)     (((n) << CAN_WORD168_DATA_BYTE_7_SHIFT) & CAN_WORD168_DATA_BYTE_7_MASK)
#define CAN_WORD168_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD168_DATA_BYTE_6_MASK   (0xFF << CAN_WORD168_DATA_BYTE_6_SHIFT)
#define CAN_WORD168_DATA_BYTE_6(n)     (((n) << CAN_WORD168_DATA_BYTE_6_SHIFT) & CAN_WORD168_DATA_BYTE_6_MASK)
#define CAN_WORD168_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD168_DATA_BYTE_5_MASK   (0xFF << CAN_WORD168_DATA_BYTE_5_SHIFT)
#define CAN_WORD168_DATA_BYTE_5(n)     (((n) << CAN_WORD168_DATA_BYTE_5_SHIFT) & CAN_WORD168_DATA_BYTE_5_MASK)
#define CAN_WORD168_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD168_DATA_BYTE_4_MASK   (0xFF << CAN_WORD168_DATA_BYTE_4_SHIFT)
#define CAN_WORD168_DATA_BYTE_4(n)     (((n) << CAN_WORD168_DATA_BYTE_4_SHIFT) & CAN_WORD168_DATA_BYTE_4_MASK)

/* Message Buffer 69 CS Register (CS69) */
#define CAN_CS69_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS69_TIME_STAMP_MASK   (0xFFFF << CAN_CS69_TIME_STAMP_SHIFT)
#define CAN_CS69_TIME_STAMP(n)     (((n) << CAN_CS69_TIME_STAMP_SHIFT) & CAN_CS69_TIME_STAMP_MASK)
#define CAN_CS69_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS69_DLC_MASK          (0xF << CAN_CS69_DLC_SHIFT)
#define CAN_CS69_DLC(n)            (((n) << CAN_CS69_DLC_SHIFT) & CAN_CS69_DLC_MASK)
#define CAN_CS69_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS69_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS69_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS69_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS69_CODE_MASK         (0xF << CAN_CS69_CODE_SHIFT)
#define CAN_CS69_CODE(n)           (((n) << CAN_CS69_CODE_SHIFT) & CAN_CS69_CODE_MASK)
#define CAN_CS69_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS69_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS69_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 69 WORD0 Register (WORD069) */
#define CAN_WORD069_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD069_DATA_BYTE_3_MASK   (0xFF << CAN_WORD069_DATA_BYTE_3_SHIFT)
#define CAN_WORD069_DATA_BYTE_3(n)     (((n) << CAN_WORD069_DATA_BYTE_3_SHIFT) & CAN_WORD069_DATA_BYTE_3_MASK)
#define CAN_WORD069_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD069_DATA_BYTE_2_MASK   (0xFF << CAN_WORD069_DATA_BYTE_2_SHIFT)
#define CAN_WORD069_DATA_BYTE_2(n)     (((n) << CAN_WORD069_DATA_BYTE_2_SHIFT) & CAN_WORD069_DATA_BYTE_2_MASK)
#define CAN_WORD069_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD069_DATA_BYTE_1_MASK   (0xFF << CAN_WORD069_DATA_BYTE_1_SHIFT)
#define CAN_WORD069_DATA_BYTE_1(n)     (((n) << CAN_WORD069_DATA_BYTE_1_SHIFT) & CAN_WORD069_DATA_BYTE_1_MASK)
#define CAN_WORD069_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD069_DATA_BYTE_0_MASK   (0xFF << CAN_WORD069_DATA_BYTE_0_SHIFT)
#define CAN_WORD069_DATA_BYTE_0(n)     (((n) << CAN_WORD069_DATA_BYTE_0_SHIFT) & CAN_WORD069_DATA_BYTE_0_MASK)

/* Message Buffer 69 WORD1 Register (WORD169) */
#define CAN_WORD169_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD169_DATA_BYTE_7_MASK   (0xFF << CAN_WORD169_DATA_BYTE_7_SHIFT)
#define CAN_WORD169_DATA_BYTE_7(n)     (((n) << CAN_WORD169_DATA_BYTE_7_SHIFT) & CAN_WORD169_DATA_BYTE_7_MASK)
#define CAN_WORD169_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD169_DATA_BYTE_6_MASK   (0xFF << CAN_WORD169_DATA_BYTE_6_SHIFT)
#define CAN_WORD169_DATA_BYTE_6(n)     (((n) << CAN_WORD169_DATA_BYTE_6_SHIFT) & CAN_WORD169_DATA_BYTE_6_MASK)
#define CAN_WORD169_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD169_DATA_BYTE_5_MASK   (0xFF << CAN_WORD169_DATA_BYTE_5_SHIFT)
#define CAN_WORD169_DATA_BYTE_5(n)     (((n) << CAN_WORD169_DATA_BYTE_5_SHIFT) & CAN_WORD169_DATA_BYTE_5_MASK)
#define CAN_WORD169_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD169_DATA_BYTE_4_MASK   (0xFF << CAN_WORD169_DATA_BYTE_4_SHIFT)
#define CAN_WORD169_DATA_BYTE_4(n)     (((n) << CAN_WORD169_DATA_BYTE_4_SHIFT) & CAN_WORD169_DATA_BYTE_4_MASK)

/* Message Buffer 70 CS Register (CS70) */
#define CAN_CS70_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS70_TIME_STAMP_MASK   (0xFFFF << CAN_CS70_TIME_STAMP_SHIFT)
#define CAN_CS70_TIME_STAMP(n)     (((n) << CAN_CS70_TIME_STAMP_SHIFT) & CAN_CS70_TIME_STAMP_MASK)
#define CAN_CS70_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS70_DLC_MASK          (0xF << CAN_CS70_DLC_SHIFT)
#define CAN_CS70_DLC(n)            (((n) << CAN_CS70_DLC_SHIFT) & CAN_CS70_DLC_MASK)
#define CAN_CS70_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS70_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS70_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS70_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS70_CODE_MASK         (0xF << CAN_CS70_CODE_SHIFT)
#define CAN_CS70_CODE(n)           (((n) << CAN_CS70_CODE_SHIFT) & CAN_CS70_CODE_MASK)
#define CAN_CS70_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS70_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS70_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 70 WORD0 Register (WORD070) */
#define CAN_WORD070_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD070_DATA_BYTE_3_MASK   (0xFF << CAN_WORD070_DATA_BYTE_3_SHIFT)
#define CAN_WORD070_DATA_BYTE_3(n)     (((n) << CAN_WORD070_DATA_BYTE_3_SHIFT) & CAN_WORD070_DATA_BYTE_3_MASK)
#define CAN_WORD070_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD070_DATA_BYTE_2_MASK   (0xFF << CAN_WORD070_DATA_BYTE_2_SHIFT)
#define CAN_WORD070_DATA_BYTE_2(n)     (((n) << CAN_WORD070_DATA_BYTE_2_SHIFT) & CAN_WORD070_DATA_BYTE_2_MASK)
#define CAN_WORD070_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD070_DATA_BYTE_1_MASK   (0xFF << CAN_WORD070_DATA_BYTE_1_SHIFT)
#define CAN_WORD070_DATA_BYTE_1(n)     (((n) << CAN_WORD070_DATA_BYTE_1_SHIFT) & CAN_WORD070_DATA_BYTE_1_MASK)
#define CAN_WORD070_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD070_DATA_BYTE_0_MASK   (0xFF << CAN_WORD070_DATA_BYTE_0_SHIFT)
#define CAN_WORD070_DATA_BYTE_0(n)     (((n) << CAN_WORD070_DATA_BYTE_0_SHIFT) & CAN_WORD070_DATA_BYTE_0_MASK)

/* Message Buffer 70 WORD1 Register (WORD170) */
#define CAN_WORD170_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD170_DATA_BYTE_7_MASK   (0xFF << CAN_WORD170_DATA_BYTE_7_SHIFT)
#define CAN_WORD170_DATA_BYTE_7(n)     (((n) << CAN_WORD170_DATA_BYTE_7_SHIFT) & CAN_WORD170_DATA_BYTE_7_MASK)
#define CAN_WORD170_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD170_DATA_BYTE_6_MASK   (0xFF << CAN_WORD170_DATA_BYTE_6_SHIFT)
#define CAN_WORD170_DATA_BYTE_6(n)     (((n) << CAN_WORD170_DATA_BYTE_6_SHIFT) & CAN_WORD170_DATA_BYTE_6_MASK)
#define CAN_WORD170_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD170_DATA_BYTE_5_MASK   (0xFF << CAN_WORD170_DATA_BYTE_5_SHIFT)
#define CAN_WORD170_DATA_BYTE_5(n)     (((n) << CAN_WORD170_DATA_BYTE_5_SHIFT) & CAN_WORD170_DATA_BYTE_5_MASK)
#define CAN_WORD170_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD170_DATA_BYTE_4_MASK   (0xFF << CAN_WORD170_DATA_BYTE_4_SHIFT)
#define CAN_WORD170_DATA_BYTE_4(n)     (((n) << CAN_WORD170_DATA_BYTE_4_SHIFT) & CAN_WORD170_DATA_BYTE_4_MASK)

/* Message Buffer 71 CS Register (CS71) */
#define CAN_CS71_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS71_TIME_STAMP_MASK   (0xFFFF << CAN_CS71_TIME_STAMP_SHIFT)
#define CAN_CS71_TIME_STAMP(n)     (((n) << CAN_CS71_TIME_STAMP_SHIFT) & CAN_CS71_TIME_STAMP_MASK)
#define CAN_CS71_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS71_DLC_MASK          (0xF << CAN_CS71_DLC_SHIFT)
#define CAN_CS71_DLC(n)            (((n) << CAN_CS71_DLC_SHIFT) & CAN_CS71_DLC_MASK)
#define CAN_CS71_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS71_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS71_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS71_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS71_CODE_MASK         (0xF << CAN_CS71_CODE_SHIFT)
#define CAN_CS71_CODE(n)           (((n) << CAN_CS71_CODE_SHIFT) & CAN_CS71_CODE_MASK)
#define CAN_CS71_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS71_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS71_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 71 WORD0 Register (WORD071) */
#define CAN_WORD071_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD071_DATA_BYTE_3_MASK   (0xFF << CAN_WORD071_DATA_BYTE_3_SHIFT)
#define CAN_WORD071_DATA_BYTE_3(n)     (((n) << CAN_WORD071_DATA_BYTE_3_SHIFT) & CAN_WORD071_DATA_BYTE_3_MASK)
#define CAN_WORD071_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD071_DATA_BYTE_2_MASK   (0xFF << CAN_WORD071_DATA_BYTE_2_SHIFT)
#define CAN_WORD071_DATA_BYTE_2(n)     (((n) << CAN_WORD071_DATA_BYTE_2_SHIFT) & CAN_WORD071_DATA_BYTE_2_MASK)
#define CAN_WORD071_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD071_DATA_BYTE_1_MASK   (0xFF << CAN_WORD071_DATA_BYTE_1_SHIFT)
#define CAN_WORD071_DATA_BYTE_1(n)     (((n) << CAN_WORD071_DATA_BYTE_1_SHIFT) & CAN_WORD071_DATA_BYTE_1_MASK)
#define CAN_WORD071_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD071_DATA_BYTE_0_MASK   (0xFF << CAN_WORD071_DATA_BYTE_0_SHIFT)
#define CAN_WORD071_DATA_BYTE_0(n)     (((n) << CAN_WORD071_DATA_BYTE_0_SHIFT) & CAN_WORD071_DATA_BYTE_0_MASK)

/* Message Buffer 71 WORD1 Register (WORD171) */
#define CAN_WORD171_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD171_DATA_BYTE_7_MASK   (0xFF << CAN_WORD171_DATA_BYTE_7_SHIFT)
#define CAN_WORD171_DATA_BYTE_7(n)     (((n) << CAN_WORD171_DATA_BYTE_7_SHIFT) & CAN_WORD171_DATA_BYTE_7_MASK)
#define CAN_WORD171_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD171_DATA_BYTE_6_MASK   (0xFF << CAN_WORD171_DATA_BYTE_6_SHIFT)
#define CAN_WORD171_DATA_BYTE_6(n)     (((n) << CAN_WORD171_DATA_BYTE_6_SHIFT) & CAN_WORD171_DATA_BYTE_6_MASK)
#define CAN_WORD171_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD171_DATA_BYTE_5_MASK   (0xFF << CAN_WORD171_DATA_BYTE_5_SHIFT)
#define CAN_WORD171_DATA_BYTE_5(n)     (((n) << CAN_WORD171_DATA_BYTE_5_SHIFT) & CAN_WORD171_DATA_BYTE_5_MASK)
#define CAN_WORD171_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD171_DATA_BYTE_4_MASK   (0xFF << CAN_WORD171_DATA_BYTE_4_SHIFT)
#define CAN_WORD171_DATA_BYTE_4(n)     (((n) << CAN_WORD171_DATA_BYTE_4_SHIFT) & CAN_WORD171_DATA_BYTE_4_MASK)

/* Message Buffer 72 CS Register (CS72) */
#define CAN_CS72_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS72_TIME_STAMP_MASK   (0xFFFF << CAN_CS72_TIME_STAMP_SHIFT)
#define CAN_CS72_TIME_STAMP(n)     (((n) << CAN_CS72_TIME_STAMP_SHIFT) & CAN_CS72_TIME_STAMP_MASK)
#define CAN_CS72_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS72_DLC_MASK          (0xF << CAN_CS72_DLC_SHIFT)
#define CAN_CS72_DLC(n)            (((n) << CAN_CS72_DLC_SHIFT) & CAN_CS72_DLC_MASK)
#define CAN_CS72_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS72_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS72_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS72_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS72_CODE_MASK         (0xF << CAN_CS72_CODE_SHIFT)
#define CAN_CS72_CODE(n)           (((n) << CAN_CS72_CODE_SHIFT) & CAN_CS72_CODE_MASK)
#define CAN_CS72_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS72_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS72_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 72 WORD0 Register (WORD072) */
#define CAN_WORD072_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD072_DATA_BYTE_3_MASK   (0xFF << CAN_WORD072_DATA_BYTE_3_SHIFT)
#define CAN_WORD072_DATA_BYTE_3(n)     (((n) << CAN_WORD072_DATA_BYTE_3_SHIFT) & CAN_WORD072_DATA_BYTE_3_MASK)
#define CAN_WORD072_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD072_DATA_BYTE_2_MASK   (0xFF << CAN_WORD072_DATA_BYTE_2_SHIFT)
#define CAN_WORD072_DATA_BYTE_2(n)     (((n) << CAN_WORD072_DATA_BYTE_2_SHIFT) & CAN_WORD072_DATA_BYTE_2_MASK)
#define CAN_WORD072_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD072_DATA_BYTE_1_MASK   (0xFF << CAN_WORD072_DATA_BYTE_1_SHIFT)
#define CAN_WORD072_DATA_BYTE_1(n)     (((n) << CAN_WORD072_DATA_BYTE_1_SHIFT) & CAN_WORD072_DATA_BYTE_1_MASK)
#define CAN_WORD072_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD072_DATA_BYTE_0_MASK   (0xFF << CAN_WORD072_DATA_BYTE_0_SHIFT)
#define CAN_WORD072_DATA_BYTE_0(n)     (((n) << CAN_WORD072_DATA_BYTE_0_SHIFT) & CAN_WORD072_DATA_BYTE_0_MASK)

/* Message Buffer 72 WORD1 Register (WORD172) */
#define CAN_WORD172_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD172_DATA_BYTE_7_MASK   (0xFF << CAN_WORD172_DATA_BYTE_7_SHIFT)
#define CAN_WORD172_DATA_BYTE_7(n)     (((n) << CAN_WORD172_DATA_BYTE_7_SHIFT) & CAN_WORD172_DATA_BYTE_7_MASK)
#define CAN_WORD172_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD172_DATA_BYTE_6_MASK   (0xFF << CAN_WORD172_DATA_BYTE_6_SHIFT)
#define CAN_WORD172_DATA_BYTE_6(n)     (((n) << CAN_WORD172_DATA_BYTE_6_SHIFT) & CAN_WORD172_DATA_BYTE_6_MASK)
#define CAN_WORD172_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD172_DATA_BYTE_5_MASK   (0xFF << CAN_WORD172_DATA_BYTE_5_SHIFT)
#define CAN_WORD172_DATA_BYTE_5(n)     (((n) << CAN_WORD172_DATA_BYTE_5_SHIFT) & CAN_WORD172_DATA_BYTE_5_MASK)
#define CAN_WORD172_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD172_DATA_BYTE_4_MASK   (0xFF << CAN_WORD172_DATA_BYTE_4_SHIFT)
#define CAN_WORD172_DATA_BYTE_4(n)     (((n) << CAN_WORD172_DATA_BYTE_4_SHIFT) & CAN_WORD172_DATA_BYTE_4_MASK)

/* Message Buffer 73 CS Register (CS73) */
#define CAN_CS73_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS73_TIME_STAMP_MASK   (0xFFFF << CAN_CS73_TIME_STAMP_SHIFT)
#define CAN_CS73_TIME_STAMP(n)     (((n) << CAN_CS73_TIME_STAMP_SHIFT) & CAN_CS73_TIME_STAMP_MASK)
#define CAN_CS73_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS73_DLC_MASK          (0xF << CAN_CS73_DLC_SHIFT)
#define CAN_CS73_DLC(n)            (((n) << CAN_CS73_DLC_SHIFT) & CAN_CS73_DLC_MASK)
#define CAN_CS73_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS73_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS73_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS73_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS73_CODE_MASK         (0xF << CAN_CS73_CODE_SHIFT)
#define CAN_CS73_CODE(n)           (((n) << CAN_CS73_CODE_SHIFT) & CAN_CS73_CODE_MASK)
#define CAN_CS73_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS73_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS73_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 73 WORD0 Register (WORD073) */
#define CAN_WORD073_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD073_DATA_BYTE_3_MASK   (0xFF << CAN_WORD073_DATA_BYTE_3_SHIFT)
#define CAN_WORD073_DATA_BYTE_3(n)     (((n) << CAN_WORD073_DATA_BYTE_3_SHIFT) & CAN_WORD073_DATA_BYTE_3_MASK)
#define CAN_WORD073_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD073_DATA_BYTE_2_MASK   (0xFF << CAN_WORD073_DATA_BYTE_2_SHIFT)
#define CAN_WORD073_DATA_BYTE_2(n)     (((n) << CAN_WORD073_DATA_BYTE_2_SHIFT) & CAN_WORD073_DATA_BYTE_2_MASK)
#define CAN_WORD073_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD073_DATA_BYTE_1_MASK   (0xFF << CAN_WORD073_DATA_BYTE_1_SHIFT)
#define CAN_WORD073_DATA_BYTE_1(n)     (((n) << CAN_WORD073_DATA_BYTE_1_SHIFT) & CAN_WORD073_DATA_BYTE_1_MASK)
#define CAN_WORD073_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD073_DATA_BYTE_0_MASK   (0xFF << CAN_WORD073_DATA_BYTE_0_SHIFT)
#define CAN_WORD073_DATA_BYTE_0(n)     (((n) << CAN_WORD073_DATA_BYTE_0_SHIFT) & CAN_WORD073_DATA_BYTE_0_MASK)

/* Message Buffer 73 WORD1 Register (WORD173) */
#define CAN_WORD173_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD173_DATA_BYTE_7_MASK   (0xFF << CAN_WORD173_DATA_BYTE_7_SHIFT)
#define CAN_WORD173_DATA_BYTE_7(n)     (((n) << CAN_WORD173_DATA_BYTE_7_SHIFT) & CAN_WORD173_DATA_BYTE_7_MASK)
#define CAN_WORD173_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD173_DATA_BYTE_6_MASK   (0xFF << CAN_WORD173_DATA_BYTE_6_SHIFT)
#define CAN_WORD173_DATA_BYTE_6(n)     (((n) << CAN_WORD173_DATA_BYTE_6_SHIFT) & CAN_WORD173_DATA_BYTE_6_MASK)
#define CAN_WORD173_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD173_DATA_BYTE_5_MASK   (0xFF << CAN_WORD173_DATA_BYTE_5_SHIFT)
#define CAN_WORD173_DATA_BYTE_5(n)     (((n) << CAN_WORD173_DATA_BYTE_5_SHIFT) & CAN_WORD173_DATA_BYTE_5_MASK)
#define CAN_WORD173_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD173_DATA_BYTE_4_MASK   (0xFF << CAN_WORD173_DATA_BYTE_4_SHIFT)
#define CAN_WORD173_DATA_BYTE_4(n)     (((n) << CAN_WORD173_DATA_BYTE_4_SHIFT) & CAN_WORD173_DATA_BYTE_4_MASK)

/* Message Buffer 74 CS Register (CS74) */
#define CAN_CS74_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS74_TIME_STAMP_MASK   (0xFFFF << CAN_CS74_TIME_STAMP_SHIFT)
#define CAN_CS74_TIME_STAMP(n)     (((n) << CAN_CS74_TIME_STAMP_SHIFT) & CAN_CS74_TIME_STAMP_MASK)
#define CAN_CS74_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS74_DLC_MASK          (0xF << CAN_CS74_DLC_SHIFT)
#define CAN_CS74_DLC(n)            (((n) << CAN_CS74_DLC_SHIFT) & CAN_CS74_DLC_MASK)
#define CAN_CS74_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS74_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS74_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS74_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS74_CODE_MASK         (0xF << CAN_CS74_CODE_SHIFT)
#define CAN_CS74_CODE(n)           (((n) << CAN_CS74_CODE_SHIFT) & CAN_CS74_CODE_MASK)
#define CAN_CS74_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS74_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS74_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 74 WORD0 Register (WORD074) */
#define CAN_WORD074_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD074_DATA_BYTE_3_MASK   (0xFF << CAN_WORD074_DATA_BYTE_3_SHIFT)
#define CAN_WORD074_DATA_BYTE_3(n)     (((n) << CAN_WORD074_DATA_BYTE_3_SHIFT) & CAN_WORD074_DATA_BYTE_3_MASK)
#define CAN_WORD074_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD074_DATA_BYTE_2_MASK   (0xFF << CAN_WORD074_DATA_BYTE_2_SHIFT)
#define CAN_WORD074_DATA_BYTE_2(n)     (((n) << CAN_WORD074_DATA_BYTE_2_SHIFT) & CAN_WORD074_DATA_BYTE_2_MASK)
#define CAN_WORD074_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD074_DATA_BYTE_1_MASK   (0xFF << CAN_WORD074_DATA_BYTE_1_SHIFT)
#define CAN_WORD074_DATA_BYTE_1(n)     (((n) << CAN_WORD074_DATA_BYTE_1_SHIFT) & CAN_WORD074_DATA_BYTE_1_MASK)
#define CAN_WORD074_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD074_DATA_BYTE_0_MASK   (0xFF << CAN_WORD074_DATA_BYTE_0_SHIFT)
#define CAN_WORD074_DATA_BYTE_0(n)     (((n) << CAN_WORD074_DATA_BYTE_0_SHIFT) & CAN_WORD074_DATA_BYTE_0_MASK)

/* Message Buffer 74 WORD1 Register (WORD174) */
#define CAN_WORD174_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD174_DATA_BYTE_7_MASK   (0xFF << CAN_WORD174_DATA_BYTE_7_SHIFT)
#define CAN_WORD174_DATA_BYTE_7(n)     (((n) << CAN_WORD174_DATA_BYTE_7_SHIFT) & CAN_WORD174_DATA_BYTE_7_MASK)
#define CAN_WORD174_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD174_DATA_BYTE_6_MASK   (0xFF << CAN_WORD174_DATA_BYTE_6_SHIFT)
#define CAN_WORD174_DATA_BYTE_6(n)     (((n) << CAN_WORD174_DATA_BYTE_6_SHIFT) & CAN_WORD174_DATA_BYTE_6_MASK)
#define CAN_WORD174_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD174_DATA_BYTE_5_MASK   (0xFF << CAN_WORD174_DATA_BYTE_5_SHIFT)
#define CAN_WORD174_DATA_BYTE_5(n)     (((n) << CAN_WORD174_DATA_BYTE_5_SHIFT) & CAN_WORD174_DATA_BYTE_5_MASK)
#define CAN_WORD174_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD174_DATA_BYTE_4_MASK   (0xFF << CAN_WORD174_DATA_BYTE_4_SHIFT)
#define CAN_WORD174_DATA_BYTE_4(n)     (((n) << CAN_WORD174_DATA_BYTE_4_SHIFT) & CAN_WORD174_DATA_BYTE_4_MASK)

/* Message Buffer 75 CS Register (CS75) */
#define CAN_CS75_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS75_TIME_STAMP_MASK   (0xFFFF << CAN_CS75_TIME_STAMP_SHIFT)
#define CAN_CS75_TIME_STAMP(n)     (((n) << CAN_CS75_TIME_STAMP_SHIFT) & CAN_CS75_TIME_STAMP_MASK)
#define CAN_CS75_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS75_DLC_MASK          (0xF << CAN_CS75_DLC_SHIFT)
#define CAN_CS75_DLC(n)            (((n) << CAN_CS75_DLC_SHIFT) & CAN_CS75_DLC_MASK)
#define CAN_CS75_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS75_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS75_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS75_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS75_CODE_MASK         (0xF << CAN_CS75_CODE_SHIFT)
#define CAN_CS75_CODE(n)           (((n) << CAN_CS75_CODE_SHIFT) & CAN_CS75_CODE_MASK)
#define CAN_CS75_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS75_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS75_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 75 WORD0 Register (WORD075) */
#define CAN_WORD075_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD075_DATA_BYTE_3_MASK   (0xFF << CAN_WORD075_DATA_BYTE_3_SHIFT)
#define CAN_WORD075_DATA_BYTE_3(n)     (((n) << CAN_WORD075_DATA_BYTE_3_SHIFT) & CAN_WORD075_DATA_BYTE_3_MASK)
#define CAN_WORD075_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD075_DATA_BYTE_2_MASK   (0xFF << CAN_WORD075_DATA_BYTE_2_SHIFT)
#define CAN_WORD075_DATA_BYTE_2(n)     (((n) << CAN_WORD075_DATA_BYTE_2_SHIFT) & CAN_WORD075_DATA_BYTE_2_MASK)
#define CAN_WORD075_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD075_DATA_BYTE_1_MASK   (0xFF << CAN_WORD075_DATA_BYTE_1_SHIFT)
#define CAN_WORD075_DATA_BYTE_1(n)     (((n) << CAN_WORD075_DATA_BYTE_1_SHIFT) & CAN_WORD075_DATA_BYTE_1_MASK)
#define CAN_WORD075_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD075_DATA_BYTE_0_MASK   (0xFF << CAN_WORD075_DATA_BYTE_0_SHIFT)
#define CAN_WORD075_DATA_BYTE_0(n)     (((n) << CAN_WORD075_DATA_BYTE_0_SHIFT) & CAN_WORD075_DATA_BYTE_0_MASK)

/* Message Buffer 75 WORD1 Register (WORD175) */
#define CAN_WORD175_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD175_DATA_BYTE_7_MASK   (0xFF << CAN_WORD175_DATA_BYTE_7_SHIFT)
#define CAN_WORD175_DATA_BYTE_7(n)     (((n) << CAN_WORD175_DATA_BYTE_7_SHIFT) & CAN_WORD175_DATA_BYTE_7_MASK)
#define CAN_WORD175_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD175_DATA_BYTE_6_MASK   (0xFF << CAN_WORD175_DATA_BYTE_6_SHIFT)
#define CAN_WORD175_DATA_BYTE_6(n)     (((n) << CAN_WORD175_DATA_BYTE_6_SHIFT) & CAN_WORD175_DATA_BYTE_6_MASK)
#define CAN_WORD175_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD175_DATA_BYTE_5_MASK   (0xFF << CAN_WORD175_DATA_BYTE_5_SHIFT)
#define CAN_WORD175_DATA_BYTE_5(n)     (((n) << CAN_WORD175_DATA_BYTE_5_SHIFT) & CAN_WORD175_DATA_BYTE_5_MASK)
#define CAN_WORD175_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD175_DATA_BYTE_4_MASK   (0xFF << CAN_WORD175_DATA_BYTE_4_SHIFT)
#define CAN_WORD175_DATA_BYTE_4(n)     (((n) << CAN_WORD175_DATA_BYTE_4_SHIFT) & CAN_WORD175_DATA_BYTE_4_MASK)

/* Message Buffer 76 CS Register (CS76) */
#define CAN_CS76_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS76_TIME_STAMP_MASK   (0xFFFF << CAN_CS76_TIME_STAMP_SHIFT)
#define CAN_CS76_TIME_STAMP(n)     (((n) << CAN_CS76_TIME_STAMP_SHIFT) & CAN_CS76_TIME_STAMP_MASK)
#define CAN_CS76_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS76_DLC_MASK          (0xF << CAN_CS76_DLC_SHIFT)
#define CAN_CS76_DLC(n)            (((n) << CAN_CS76_DLC_SHIFT) & CAN_CS76_DLC_MASK)
#define CAN_CS76_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS76_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS76_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS76_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS76_CODE_MASK         (0xF << CAN_CS76_CODE_SHIFT)
#define CAN_CS76_CODE(n)           (((n) << CAN_CS76_CODE_SHIFT) & CAN_CS76_CODE_MASK)
#define CAN_CS76_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS76_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS76_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 76 WORD0 Register (WORD076) */
#define CAN_WORD076_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD076_DATA_BYTE_3_MASK   (0xFF << CAN_WORD076_DATA_BYTE_3_SHIFT)
#define CAN_WORD076_DATA_BYTE_3(n)     (((n) << CAN_WORD076_DATA_BYTE_3_SHIFT) & CAN_WORD076_DATA_BYTE_3_MASK)
#define CAN_WORD076_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD076_DATA_BYTE_2_MASK   (0xFF << CAN_WORD076_DATA_BYTE_2_SHIFT)
#define CAN_WORD076_DATA_BYTE_2(n)     (((n) << CAN_WORD076_DATA_BYTE_2_SHIFT) & CAN_WORD076_DATA_BYTE_2_MASK)
#define CAN_WORD076_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD076_DATA_BYTE_1_MASK   (0xFF << CAN_WORD076_DATA_BYTE_1_SHIFT)
#define CAN_WORD076_DATA_BYTE_1(n)     (((n) << CAN_WORD076_DATA_BYTE_1_SHIFT) & CAN_WORD076_DATA_BYTE_1_MASK)
#define CAN_WORD076_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD076_DATA_BYTE_0_MASK   (0xFF << CAN_WORD076_DATA_BYTE_0_SHIFT)
#define CAN_WORD076_DATA_BYTE_0(n)     (((n) << CAN_WORD076_DATA_BYTE_0_SHIFT) & CAN_WORD076_DATA_BYTE_0_MASK)

/* Message Buffer 76 WORD1 Register (WORD176) */
#define CAN_WORD176_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD176_DATA_BYTE_7_MASK   (0xFF << CAN_WORD176_DATA_BYTE_7_SHIFT)
#define CAN_WORD176_DATA_BYTE_7(n)     (((n) << CAN_WORD176_DATA_BYTE_7_SHIFT) & CAN_WORD176_DATA_BYTE_7_MASK)
#define CAN_WORD176_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD176_DATA_BYTE_6_MASK   (0xFF << CAN_WORD176_DATA_BYTE_6_SHIFT)
#define CAN_WORD176_DATA_BYTE_6(n)     (((n) << CAN_WORD176_DATA_BYTE_6_SHIFT) & CAN_WORD176_DATA_BYTE_6_MASK)
#define CAN_WORD176_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD176_DATA_BYTE_5_MASK   (0xFF << CAN_WORD176_DATA_BYTE_5_SHIFT)
#define CAN_WORD176_DATA_BYTE_5(n)     (((n) << CAN_WORD176_DATA_BYTE_5_SHIFT) & CAN_WORD176_DATA_BYTE_5_MASK)
#define CAN_WORD176_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD176_DATA_BYTE_4_MASK   (0xFF << CAN_WORD176_DATA_BYTE_4_SHIFT)
#define CAN_WORD176_DATA_BYTE_4(n)     (((n) << CAN_WORD176_DATA_BYTE_4_SHIFT) & CAN_WORD176_DATA_BYTE_4_MASK)

/* Message Buffer 77 CS Register (CS77) */
#define CAN_CS77_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS77_TIME_STAMP_MASK   (0xFFFF << CAN_CS77_TIME_STAMP_SHIFT)
#define CAN_CS77_TIME_STAMP(n)     (((n) << CAN_CS77_TIME_STAMP_SHIFT) & CAN_CS77_TIME_STAMP_MASK)
#define CAN_CS77_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS77_DLC_MASK          (0xF << CAN_CS77_DLC_SHIFT)
#define CAN_CS77_DLC(n)            (((n) << CAN_CS77_DLC_SHIFT) & CAN_CS77_DLC_MASK)
#define CAN_CS77_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS77_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS77_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS77_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS77_CODE_MASK         (0xF << CAN_CS77_CODE_SHIFT)
#define CAN_CS77_CODE(n)           (((n) << CAN_CS77_CODE_SHIFT) & CAN_CS77_CODE_MASK)
#define CAN_CS77_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS77_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS77_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 77 WORD0 Register (WORD077) */
#define CAN_WORD077_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD077_DATA_BYTE_3_MASK   (0xFF << CAN_WORD077_DATA_BYTE_3_SHIFT)
#define CAN_WORD077_DATA_BYTE_3(n)     (((n) << CAN_WORD077_DATA_BYTE_3_SHIFT) & CAN_WORD077_DATA_BYTE_3_MASK)
#define CAN_WORD077_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD077_DATA_BYTE_2_MASK   (0xFF << CAN_WORD077_DATA_BYTE_2_SHIFT)
#define CAN_WORD077_DATA_BYTE_2(n)     (((n) << CAN_WORD077_DATA_BYTE_2_SHIFT) & CAN_WORD077_DATA_BYTE_2_MASK)
#define CAN_WORD077_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD077_DATA_BYTE_1_MASK   (0xFF << CAN_WORD077_DATA_BYTE_1_SHIFT)
#define CAN_WORD077_DATA_BYTE_1(n)     (((n) << CAN_WORD077_DATA_BYTE_1_SHIFT) & CAN_WORD077_DATA_BYTE_1_MASK)
#define CAN_WORD077_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD077_DATA_BYTE_0_MASK   (0xFF << CAN_WORD077_DATA_BYTE_0_SHIFT)
#define CAN_WORD077_DATA_BYTE_0(n)     (((n) << CAN_WORD077_DATA_BYTE_0_SHIFT) & CAN_WORD077_DATA_BYTE_0_MASK)

/* Message Buffer 77 WORD1 Register (WORD177) */
#define CAN_WORD177_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD177_DATA_BYTE_7_MASK   (0xFF << CAN_WORD177_DATA_BYTE_7_SHIFT)
#define CAN_WORD177_DATA_BYTE_7(n)     (((n) << CAN_WORD177_DATA_BYTE_7_SHIFT) & CAN_WORD177_DATA_BYTE_7_MASK)
#define CAN_WORD177_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD177_DATA_BYTE_6_MASK   (0xFF << CAN_WORD177_DATA_BYTE_6_SHIFT)
#define CAN_WORD177_DATA_BYTE_6(n)     (((n) << CAN_WORD177_DATA_BYTE_6_SHIFT) & CAN_WORD177_DATA_BYTE_6_MASK)
#define CAN_WORD177_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD177_DATA_BYTE_5_MASK   (0xFF << CAN_WORD177_DATA_BYTE_5_SHIFT)
#define CAN_WORD177_DATA_BYTE_5(n)     (((n) << CAN_WORD177_DATA_BYTE_5_SHIFT) & CAN_WORD177_DATA_BYTE_5_MASK)
#define CAN_WORD177_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD177_DATA_BYTE_4_MASK   (0xFF << CAN_WORD177_DATA_BYTE_4_SHIFT)
#define CAN_WORD177_DATA_BYTE_4(n)     (((n) << CAN_WORD177_DATA_BYTE_4_SHIFT) & CAN_WORD177_DATA_BYTE_4_MASK)

/* Message Buffer 78 CS Register (CS78) */
#define CAN_CS78_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS78_TIME_STAMP_MASK   (0xFFFF << CAN_CS78_TIME_STAMP_SHIFT)
#define CAN_CS78_TIME_STAMP(n)     (((n) << CAN_CS78_TIME_STAMP_SHIFT) & CAN_CS78_TIME_STAMP_MASK)
#define CAN_CS78_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS78_DLC_MASK          (0xF << CAN_CS78_DLC_SHIFT)
#define CAN_CS78_DLC(n)            (((n) << CAN_CS78_DLC_SHIFT) & CAN_CS78_DLC_MASK)
#define CAN_CS78_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS78_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS78_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS78_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS78_CODE_MASK         (0xF << CAN_CS78_CODE_SHIFT)
#define CAN_CS78_CODE(n)           (((n) << CAN_CS78_CODE_SHIFT) & CAN_CS78_CODE_MASK)
#define CAN_CS78_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS78_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS78_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 78 WORD0 Register (WORD078) */
#define CAN_WORD078_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD078_DATA_BYTE_3_MASK   (0xFF << CAN_WORD078_DATA_BYTE_3_SHIFT)
#define CAN_WORD078_DATA_BYTE_3(n)     (((n) << CAN_WORD078_DATA_BYTE_3_SHIFT) & CAN_WORD078_DATA_BYTE_3_MASK)
#define CAN_WORD078_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD078_DATA_BYTE_2_MASK   (0xFF << CAN_WORD078_DATA_BYTE_2_SHIFT)
#define CAN_WORD078_DATA_BYTE_2(n)     (((n) << CAN_WORD078_DATA_BYTE_2_SHIFT) & CAN_WORD078_DATA_BYTE_2_MASK)
#define CAN_WORD078_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD078_DATA_BYTE_1_MASK   (0xFF << CAN_WORD078_DATA_BYTE_1_SHIFT)
#define CAN_WORD078_DATA_BYTE_1(n)     (((n) << CAN_WORD078_DATA_BYTE_1_SHIFT) & CAN_WORD078_DATA_BYTE_1_MASK)
#define CAN_WORD078_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD078_DATA_BYTE_0_MASK   (0xFF << CAN_WORD078_DATA_BYTE_0_SHIFT)
#define CAN_WORD078_DATA_BYTE_0(n)     (((n) << CAN_WORD078_DATA_BYTE_0_SHIFT) & CAN_WORD078_DATA_BYTE_0_MASK)

/* Message Buffer 78 WORD1 Register (WORD178) */
#define CAN_WORD178_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD178_DATA_BYTE_7_MASK   (0xFF << CAN_WORD178_DATA_BYTE_7_SHIFT)
#define CAN_WORD178_DATA_BYTE_7(n)     (((n) << CAN_WORD178_DATA_BYTE_7_SHIFT) & CAN_WORD178_DATA_BYTE_7_MASK)
#define CAN_WORD178_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD178_DATA_BYTE_6_MASK   (0xFF << CAN_WORD178_DATA_BYTE_6_SHIFT)
#define CAN_WORD178_DATA_BYTE_6(n)     (((n) << CAN_WORD178_DATA_BYTE_6_SHIFT) & CAN_WORD178_DATA_BYTE_6_MASK)
#define CAN_WORD178_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD178_DATA_BYTE_5_MASK   (0xFF << CAN_WORD178_DATA_BYTE_5_SHIFT)
#define CAN_WORD178_DATA_BYTE_5(n)     (((n) << CAN_WORD178_DATA_BYTE_5_SHIFT) & CAN_WORD178_DATA_BYTE_5_MASK)
#define CAN_WORD178_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD178_DATA_BYTE_4_MASK   (0xFF << CAN_WORD178_DATA_BYTE_4_SHIFT)
#define CAN_WORD178_DATA_BYTE_4(n)     (((n) << CAN_WORD178_DATA_BYTE_4_SHIFT) & CAN_WORD178_DATA_BYTE_4_MASK)

/* Message Buffer 79 CS Register (CS79) */
#define CAN_CS79_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS79_TIME_STAMP_MASK   (0xFFFF << CAN_CS79_TIME_STAMP_SHIFT)
#define CAN_CS79_TIME_STAMP(n)     (((n) << CAN_CS79_TIME_STAMP_SHIFT) & CAN_CS79_TIME_STAMP_MASK)
#define CAN_CS79_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS79_DLC_MASK          (0xF << CAN_CS79_DLC_SHIFT)
#define CAN_CS79_DLC(n)            (((n) << CAN_CS79_DLC_SHIFT) & CAN_CS79_DLC_MASK)
#define CAN_CS79_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS79_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS79_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS79_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS79_CODE_MASK         (0xF << CAN_CS79_CODE_SHIFT)
#define CAN_CS79_CODE(n)           (((n) << CAN_CS79_CODE_SHIFT) & CAN_CS79_CODE_MASK)
#define CAN_CS79_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS79_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS79_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 79 WORD0 Register (WORD079) */
#define CAN_WORD079_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD079_DATA_BYTE_3_MASK   (0xFF << CAN_WORD079_DATA_BYTE_3_SHIFT)
#define CAN_WORD079_DATA_BYTE_3(n)     (((n) << CAN_WORD079_DATA_BYTE_3_SHIFT) & CAN_WORD079_DATA_BYTE_3_MASK)
#define CAN_WORD079_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD079_DATA_BYTE_2_MASK   (0xFF << CAN_WORD079_DATA_BYTE_2_SHIFT)
#define CAN_WORD079_DATA_BYTE_2(n)     (((n) << CAN_WORD079_DATA_BYTE_2_SHIFT) & CAN_WORD079_DATA_BYTE_2_MASK)
#define CAN_WORD079_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD079_DATA_BYTE_1_MASK   (0xFF << CAN_WORD079_DATA_BYTE_1_SHIFT)
#define CAN_WORD079_DATA_BYTE_1(n)     (((n) << CAN_WORD079_DATA_BYTE_1_SHIFT) & CAN_WORD079_DATA_BYTE_1_MASK)
#define CAN_WORD079_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD079_DATA_BYTE_0_MASK   (0xFF << CAN_WORD079_DATA_BYTE_0_SHIFT)
#define CAN_WORD079_DATA_BYTE_0(n)     (((n) << CAN_WORD079_DATA_BYTE_0_SHIFT) & CAN_WORD079_DATA_BYTE_0_MASK)

/* Message Buffer 79 WORD1 Register (WORD179) */
#define CAN_WORD179_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD179_DATA_BYTE_7_MASK   (0xFF << CAN_WORD179_DATA_BYTE_7_SHIFT)
#define CAN_WORD179_DATA_BYTE_7(n)     (((n) << CAN_WORD179_DATA_BYTE_7_SHIFT) & CAN_WORD179_DATA_BYTE_7_MASK)
#define CAN_WORD179_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD179_DATA_BYTE_6_MASK   (0xFF << CAN_WORD179_DATA_BYTE_6_SHIFT)
#define CAN_WORD179_DATA_BYTE_6(n)     (((n) << CAN_WORD179_DATA_BYTE_6_SHIFT) & CAN_WORD179_DATA_BYTE_6_MASK)
#define CAN_WORD179_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD179_DATA_BYTE_5_MASK   (0xFF << CAN_WORD179_DATA_BYTE_5_SHIFT)
#define CAN_WORD179_DATA_BYTE_5(n)     (((n) << CAN_WORD179_DATA_BYTE_5_SHIFT) & CAN_WORD179_DATA_BYTE_5_MASK)
#define CAN_WORD179_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD179_DATA_BYTE_4_MASK   (0xFF << CAN_WORD179_DATA_BYTE_4_SHIFT)
#define CAN_WORD179_DATA_BYTE_4(n)     (((n) << CAN_WORD179_DATA_BYTE_4_SHIFT) & CAN_WORD179_DATA_BYTE_4_MASK)

/* Message Buffer 80 CS Register (CS80) */
#define CAN_CS80_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS80_TIME_STAMP_MASK   (0xFFFF << CAN_CS80_TIME_STAMP_SHIFT)
#define CAN_CS80_TIME_STAMP(n)     (((n) << CAN_CS80_TIME_STAMP_SHIFT) & CAN_CS80_TIME_STAMP_MASK)
#define CAN_CS80_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS80_DLC_MASK          (0xF << CAN_CS80_DLC_SHIFT)
#define CAN_CS80_DLC(n)            (((n) << CAN_CS80_DLC_SHIFT) & CAN_CS80_DLC_MASK)
#define CAN_CS80_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS80_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS80_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS80_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS80_CODE_MASK         (0xF << CAN_CS80_CODE_SHIFT)
#define CAN_CS80_CODE(n)           (((n) << CAN_CS80_CODE_SHIFT) & CAN_CS80_CODE_MASK)
#define CAN_CS80_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS80_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS80_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 80 WORD0 Register (WORD080) */
#define CAN_WORD080_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD080_DATA_BYTE_3_MASK   (0xFF << CAN_WORD080_DATA_BYTE_3_SHIFT)
#define CAN_WORD080_DATA_BYTE_3(n)     (((n) << CAN_WORD080_DATA_BYTE_3_SHIFT) & CAN_WORD080_DATA_BYTE_3_MASK)
#define CAN_WORD080_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD080_DATA_BYTE_2_MASK   (0xFF << CAN_WORD080_DATA_BYTE_2_SHIFT)
#define CAN_WORD080_DATA_BYTE_2(n)     (((n) << CAN_WORD080_DATA_BYTE_2_SHIFT) & CAN_WORD080_DATA_BYTE_2_MASK)
#define CAN_WORD080_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD080_DATA_BYTE_1_MASK   (0xFF << CAN_WORD080_DATA_BYTE_1_SHIFT)
#define CAN_WORD080_DATA_BYTE_1(n)     (((n) << CAN_WORD080_DATA_BYTE_1_SHIFT) & CAN_WORD080_DATA_BYTE_1_MASK)
#define CAN_WORD080_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD080_DATA_BYTE_0_MASK   (0xFF << CAN_WORD080_DATA_BYTE_0_SHIFT)
#define CAN_WORD080_DATA_BYTE_0(n)     (((n) << CAN_WORD080_DATA_BYTE_0_SHIFT) & CAN_WORD080_DATA_BYTE_0_MASK)

/* Message Buffer 80 WORD1 Register (WORD180) */
#define CAN_WORD180_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD180_DATA_BYTE_7_MASK   (0xFF << CAN_WORD180_DATA_BYTE_7_SHIFT)
#define CAN_WORD180_DATA_BYTE_7(n)     (((n) << CAN_WORD180_DATA_BYTE_7_SHIFT) & CAN_WORD180_DATA_BYTE_7_MASK)
#define CAN_WORD180_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD180_DATA_BYTE_6_MASK   (0xFF << CAN_WORD180_DATA_BYTE_6_SHIFT)
#define CAN_WORD180_DATA_BYTE_6(n)     (((n) << CAN_WORD180_DATA_BYTE_6_SHIFT) & CAN_WORD180_DATA_BYTE_6_MASK)
#define CAN_WORD180_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD180_DATA_BYTE_5_MASK   (0xFF << CAN_WORD180_DATA_BYTE_5_SHIFT)
#define CAN_WORD180_DATA_BYTE_5(n)     (((n) << CAN_WORD180_DATA_BYTE_5_SHIFT) & CAN_WORD180_DATA_BYTE_5_MASK)
#define CAN_WORD180_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD180_DATA_BYTE_4_MASK   (0xFF << CAN_WORD180_DATA_BYTE_4_SHIFT)
#define CAN_WORD180_DATA_BYTE_4(n)     (((n) << CAN_WORD180_DATA_BYTE_4_SHIFT) & CAN_WORD180_DATA_BYTE_4_MASK)

/* Message Buffer 81 CS Register (CS81) */
#define CAN_CS81_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS81_TIME_STAMP_MASK   (0xFFFF << CAN_CS81_TIME_STAMP_SHIFT)
#define CAN_CS81_TIME_STAMP(n)     (((n) << CAN_CS81_TIME_STAMP_SHIFT) & CAN_CS81_TIME_STAMP_MASK)
#define CAN_CS81_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS81_DLC_MASK          (0xF << CAN_CS81_DLC_SHIFT)
#define CAN_CS81_DLC(n)            (((n) << CAN_CS81_DLC_SHIFT) & CAN_CS81_DLC_MASK)
#define CAN_CS81_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS81_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS81_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS81_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS81_CODE_MASK         (0xF << CAN_CS81_CODE_SHIFT)
#define CAN_CS81_CODE(n)           (((n) << CAN_CS81_CODE_SHIFT) & CAN_CS81_CODE_MASK)
#define CAN_CS81_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS81_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS81_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 81 WORD0 Register (WORD081) */
#define CAN_WORD081_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD081_DATA_BYTE_3_MASK   (0xFF << CAN_WORD081_DATA_BYTE_3_SHIFT)
#define CAN_WORD081_DATA_BYTE_3(n)     (((n) << CAN_WORD081_DATA_BYTE_3_SHIFT) & CAN_WORD081_DATA_BYTE_3_MASK)
#define CAN_WORD081_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD081_DATA_BYTE_2_MASK   (0xFF << CAN_WORD081_DATA_BYTE_2_SHIFT)
#define CAN_WORD081_DATA_BYTE_2(n)     (((n) << CAN_WORD081_DATA_BYTE_2_SHIFT) & CAN_WORD081_DATA_BYTE_2_MASK)
#define CAN_WORD081_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD081_DATA_BYTE_1_MASK   (0xFF << CAN_WORD081_DATA_BYTE_1_SHIFT)
#define CAN_WORD081_DATA_BYTE_1(n)     (((n) << CAN_WORD081_DATA_BYTE_1_SHIFT) & CAN_WORD081_DATA_BYTE_1_MASK)
#define CAN_WORD081_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD081_DATA_BYTE_0_MASK   (0xFF << CAN_WORD081_DATA_BYTE_0_SHIFT)
#define CAN_WORD081_DATA_BYTE_0(n)     (((n) << CAN_WORD081_DATA_BYTE_0_SHIFT) & CAN_WORD081_DATA_BYTE_0_MASK)

/* Message Buffer 81 WORD1 Register (WORD181) */
#define CAN_WORD181_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD181_DATA_BYTE_7_MASK   (0xFF << CAN_WORD181_DATA_BYTE_7_SHIFT)
#define CAN_WORD181_DATA_BYTE_7(n)     (((n) << CAN_WORD181_DATA_BYTE_7_SHIFT) & CAN_WORD181_DATA_BYTE_7_MASK)
#define CAN_WORD181_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD181_DATA_BYTE_6_MASK   (0xFF << CAN_WORD181_DATA_BYTE_6_SHIFT)
#define CAN_WORD181_DATA_BYTE_6(n)     (((n) << CAN_WORD181_DATA_BYTE_6_SHIFT) & CAN_WORD181_DATA_BYTE_6_MASK)
#define CAN_WORD181_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD181_DATA_BYTE_5_MASK   (0xFF << CAN_WORD181_DATA_BYTE_5_SHIFT)
#define CAN_WORD181_DATA_BYTE_5(n)     (((n) << CAN_WORD181_DATA_BYTE_5_SHIFT) & CAN_WORD181_DATA_BYTE_5_MASK)
#define CAN_WORD181_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD181_DATA_BYTE_4_MASK   (0xFF << CAN_WORD181_DATA_BYTE_4_SHIFT)
#define CAN_WORD181_DATA_BYTE_4(n)     (((n) << CAN_WORD181_DATA_BYTE_4_SHIFT) & CAN_WORD181_DATA_BYTE_4_MASK)

/* Message Buffer 82 CS Register (CS82) */
#define CAN_CS82_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS82_TIME_STAMP_MASK   (0xFFFF << CAN_CS82_TIME_STAMP_SHIFT)
#define CAN_CS82_TIME_STAMP(n)     (((n) << CAN_CS82_TIME_STAMP_SHIFT) & CAN_CS82_TIME_STAMP_MASK)
#define CAN_CS82_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS82_DLC_MASK          (0xF << CAN_CS82_DLC_SHIFT)
#define CAN_CS82_DLC(n)            (((n) << CAN_CS82_DLC_SHIFT) & CAN_CS82_DLC_MASK)
#define CAN_CS82_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS82_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS82_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS82_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS82_CODE_MASK         (0xF << CAN_CS82_CODE_SHIFT)
#define CAN_CS82_CODE(n)           (((n) << CAN_CS82_CODE_SHIFT) & CAN_CS82_CODE_MASK)
#define CAN_CS82_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS82_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS82_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 82 WORD0 Register (WORD082) */
#define CAN_WORD082_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD082_DATA_BYTE_3_MASK   (0xFF << CAN_WORD082_DATA_BYTE_3_SHIFT)
#define CAN_WORD082_DATA_BYTE_3(n)     (((n) << CAN_WORD082_DATA_BYTE_3_SHIFT) & CAN_WORD082_DATA_BYTE_3_MASK)
#define CAN_WORD082_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD082_DATA_BYTE_2_MASK   (0xFF << CAN_WORD082_DATA_BYTE_2_SHIFT)
#define CAN_WORD082_DATA_BYTE_2(n)     (((n) << CAN_WORD082_DATA_BYTE_2_SHIFT) & CAN_WORD082_DATA_BYTE_2_MASK)
#define CAN_WORD082_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD082_DATA_BYTE_1_MASK   (0xFF << CAN_WORD082_DATA_BYTE_1_SHIFT)
#define CAN_WORD082_DATA_BYTE_1(n)     (((n) << CAN_WORD082_DATA_BYTE_1_SHIFT) & CAN_WORD082_DATA_BYTE_1_MASK)
#define CAN_WORD082_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD082_DATA_BYTE_0_MASK   (0xFF << CAN_WORD082_DATA_BYTE_0_SHIFT)
#define CAN_WORD082_DATA_BYTE_0(n)     (((n) << CAN_WORD082_DATA_BYTE_0_SHIFT) & CAN_WORD082_DATA_BYTE_0_MASK)

/* Message Buffer 82 WORD1 Register (WORD182) */
#define CAN_WORD182_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD182_DATA_BYTE_7_MASK   (0xFF << CAN_WORD182_DATA_BYTE_7_SHIFT)
#define CAN_WORD182_DATA_BYTE_7(n)     (((n) << CAN_WORD182_DATA_BYTE_7_SHIFT) & CAN_WORD182_DATA_BYTE_7_MASK)
#define CAN_WORD182_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD182_DATA_BYTE_6_MASK   (0xFF << CAN_WORD182_DATA_BYTE_6_SHIFT)
#define CAN_WORD182_DATA_BYTE_6(n)     (((n) << CAN_WORD182_DATA_BYTE_6_SHIFT) & CAN_WORD182_DATA_BYTE_6_MASK)
#define CAN_WORD182_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD182_DATA_BYTE_5_MASK   (0xFF << CAN_WORD182_DATA_BYTE_5_SHIFT)
#define CAN_WORD182_DATA_BYTE_5(n)     (((n) << CAN_WORD182_DATA_BYTE_5_SHIFT) & CAN_WORD182_DATA_BYTE_5_MASK)
#define CAN_WORD182_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD182_DATA_BYTE_4_MASK   (0xFF << CAN_WORD182_DATA_BYTE_4_SHIFT)
#define CAN_WORD182_DATA_BYTE_4(n)     (((n) << CAN_WORD182_DATA_BYTE_4_SHIFT) & CAN_WORD182_DATA_BYTE_4_MASK)

/* Message Buffer 83 CS Register (CS83) */
#define CAN_CS83_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS83_TIME_STAMP_MASK   (0xFFFF << CAN_CS83_TIME_STAMP_SHIFT)
#define CAN_CS83_TIME_STAMP(n)     (((n) << CAN_CS83_TIME_STAMP_SHIFT) & CAN_CS83_TIME_STAMP_MASK)
#define CAN_CS83_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS83_DLC_MASK          (0xF << CAN_CS83_DLC_SHIFT)
#define CAN_CS83_DLC(n)            (((n) << CAN_CS83_DLC_SHIFT) & CAN_CS83_DLC_MASK)
#define CAN_CS83_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS83_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS83_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS83_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS83_CODE_MASK         (0xF << CAN_CS83_CODE_SHIFT)
#define CAN_CS83_CODE(n)           (((n) << CAN_CS83_CODE_SHIFT) & CAN_CS83_CODE_MASK)
#define CAN_CS83_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS83_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS83_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 83 WORD0 Register (WORD083) */
#define CAN_WORD083_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD083_DATA_BYTE_3_MASK   (0xFF << CAN_WORD083_DATA_BYTE_3_SHIFT)
#define CAN_WORD083_DATA_BYTE_3(n)     (((n) << CAN_WORD083_DATA_BYTE_3_SHIFT) & CAN_WORD083_DATA_BYTE_3_MASK)
#define CAN_WORD083_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD083_DATA_BYTE_2_MASK   (0xFF << CAN_WORD083_DATA_BYTE_2_SHIFT)
#define CAN_WORD083_DATA_BYTE_2(n)     (((n) << CAN_WORD083_DATA_BYTE_2_SHIFT) & CAN_WORD083_DATA_BYTE_2_MASK)
#define CAN_WORD083_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD083_DATA_BYTE_1_MASK   (0xFF << CAN_WORD083_DATA_BYTE_1_SHIFT)
#define CAN_WORD083_DATA_BYTE_1(n)     (((n) << CAN_WORD083_DATA_BYTE_1_SHIFT) & CAN_WORD083_DATA_BYTE_1_MASK)
#define CAN_WORD083_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD083_DATA_BYTE_0_MASK   (0xFF << CAN_WORD083_DATA_BYTE_0_SHIFT)
#define CAN_WORD083_DATA_BYTE_0(n)     (((n) << CAN_WORD083_DATA_BYTE_0_SHIFT) & CAN_WORD083_DATA_BYTE_0_MASK)

/* Message Buffer 83 WORD1 Register (WORD183) */
#define CAN_WORD183_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD183_DATA_BYTE_7_MASK   (0xFF << CAN_WORD183_DATA_BYTE_7_SHIFT)
#define CAN_WORD183_DATA_BYTE_7(n)     (((n) << CAN_WORD183_DATA_BYTE_7_SHIFT) & CAN_WORD183_DATA_BYTE_7_MASK)
#define CAN_WORD183_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD183_DATA_BYTE_6_MASK   (0xFF << CAN_WORD183_DATA_BYTE_6_SHIFT)
#define CAN_WORD183_DATA_BYTE_6(n)     (((n) << CAN_WORD183_DATA_BYTE_6_SHIFT) & CAN_WORD183_DATA_BYTE_6_MASK)
#define CAN_WORD183_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD183_DATA_BYTE_5_MASK   (0xFF << CAN_WORD183_DATA_BYTE_5_SHIFT)
#define CAN_WORD183_DATA_BYTE_5(n)     (((n) << CAN_WORD183_DATA_BYTE_5_SHIFT) & CAN_WORD183_DATA_BYTE_5_MASK)
#define CAN_WORD183_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD183_DATA_BYTE_4_MASK   (0xFF << CAN_WORD183_DATA_BYTE_4_SHIFT)
#define CAN_WORD183_DATA_BYTE_4(n)     (((n) << CAN_WORD183_DATA_BYTE_4_SHIFT) & CAN_WORD183_DATA_BYTE_4_MASK)

/* Message Buffer 84 CS Register (CS84) */
#define CAN_CS84_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS84_TIME_STAMP_MASK   (0xFFFF << CAN_CS84_TIME_STAMP_SHIFT)
#define CAN_CS84_TIME_STAMP(n)     (((n) << CAN_CS84_TIME_STAMP_SHIFT) & CAN_CS84_TIME_STAMP_MASK)
#define CAN_CS84_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS84_DLC_MASK          (0xF << CAN_CS84_DLC_SHIFT)
#define CAN_CS84_DLC(n)            (((n) << CAN_CS84_DLC_SHIFT) & CAN_CS84_DLC_MASK)
#define CAN_CS84_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS84_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS84_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS84_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS84_CODE_MASK         (0xF << CAN_CS84_CODE_SHIFT)
#define CAN_CS84_CODE(n)           (((n) << CAN_CS84_CODE_SHIFT) & CAN_CS84_CODE_MASK)
#define CAN_CS84_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS84_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS84_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 84 WORD0 Register (WORD084) */
#define CAN_WORD084_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD084_DATA_BYTE_3_MASK   (0xFF << CAN_WORD084_DATA_BYTE_3_SHIFT)
#define CAN_WORD084_DATA_BYTE_3(n)     (((n) << CAN_WORD084_DATA_BYTE_3_SHIFT) & CAN_WORD084_DATA_BYTE_3_MASK)
#define CAN_WORD084_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD084_DATA_BYTE_2_MASK   (0xFF << CAN_WORD084_DATA_BYTE_2_SHIFT)
#define CAN_WORD084_DATA_BYTE_2(n)     (((n) << CAN_WORD084_DATA_BYTE_2_SHIFT) & CAN_WORD084_DATA_BYTE_2_MASK)
#define CAN_WORD084_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD084_DATA_BYTE_1_MASK   (0xFF << CAN_WORD084_DATA_BYTE_1_SHIFT)
#define CAN_WORD084_DATA_BYTE_1(n)     (((n) << CAN_WORD084_DATA_BYTE_1_SHIFT) & CAN_WORD084_DATA_BYTE_1_MASK)
#define CAN_WORD084_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD084_DATA_BYTE_0_MASK   (0xFF << CAN_WORD084_DATA_BYTE_0_SHIFT)
#define CAN_WORD084_DATA_BYTE_0(n)     (((n) << CAN_WORD084_DATA_BYTE_0_SHIFT) & CAN_WORD084_DATA_BYTE_0_MASK)

/* Message Buffer 84 WORD1 Register (WORD184) */
#define CAN_WORD184_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD184_DATA_BYTE_7_MASK   (0xFF << CAN_WORD184_DATA_BYTE_7_SHIFT)
#define CAN_WORD184_DATA_BYTE_7(n)     (((n) << CAN_WORD184_DATA_BYTE_7_SHIFT) & CAN_WORD184_DATA_BYTE_7_MASK)
#define CAN_WORD184_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD184_DATA_BYTE_6_MASK   (0xFF << CAN_WORD184_DATA_BYTE_6_SHIFT)
#define CAN_WORD184_DATA_BYTE_6(n)     (((n) << CAN_WORD184_DATA_BYTE_6_SHIFT) & CAN_WORD184_DATA_BYTE_6_MASK)
#define CAN_WORD184_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD184_DATA_BYTE_5_MASK   (0xFF << CAN_WORD184_DATA_BYTE_5_SHIFT)
#define CAN_WORD184_DATA_BYTE_5(n)     (((n) << CAN_WORD184_DATA_BYTE_5_SHIFT) & CAN_WORD184_DATA_BYTE_5_MASK)
#define CAN_WORD184_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD184_DATA_BYTE_4_MASK   (0xFF << CAN_WORD184_DATA_BYTE_4_SHIFT)
#define CAN_WORD184_DATA_BYTE_4(n)     (((n) << CAN_WORD184_DATA_BYTE_4_SHIFT) & CAN_WORD184_DATA_BYTE_4_MASK)

/* Message Buffer 85 CS Register (CS85) */
#define CAN_CS85_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS85_TIME_STAMP_MASK   (0xFFFF << CAN_CS85_TIME_STAMP_SHIFT)
#define CAN_CS85_TIME_STAMP(n)     (((n) << CAN_CS85_TIME_STAMP_SHIFT) & CAN_CS85_TIME_STAMP_MASK)
#define CAN_CS85_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS85_DLC_MASK          (0xF << CAN_CS85_DLC_SHIFT)
#define CAN_CS85_DLC(n)            (((n) << CAN_CS85_DLC_SHIFT) & CAN_CS85_DLC_MASK)
#define CAN_CS85_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS85_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS85_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS85_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS85_CODE_MASK         (0xF << CAN_CS85_CODE_SHIFT)
#define CAN_CS85_CODE(n)           (((n) << CAN_CS85_CODE_SHIFT) & CAN_CS85_CODE_MASK)
#define CAN_CS85_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS85_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS85_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 85 WORD0 Register (WORD085) */
#define CAN_WORD085_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD085_DATA_BYTE_3_MASK   (0xFF << CAN_WORD085_DATA_BYTE_3_SHIFT)
#define CAN_WORD085_DATA_BYTE_3(n)     (((n) << CAN_WORD085_DATA_BYTE_3_SHIFT) & CAN_WORD085_DATA_BYTE_3_MASK)
#define CAN_WORD085_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD085_DATA_BYTE_2_MASK   (0xFF << CAN_WORD085_DATA_BYTE_2_SHIFT)
#define CAN_WORD085_DATA_BYTE_2(n)     (((n) << CAN_WORD085_DATA_BYTE_2_SHIFT) & CAN_WORD085_DATA_BYTE_2_MASK)
#define CAN_WORD085_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD085_DATA_BYTE_1_MASK   (0xFF << CAN_WORD085_DATA_BYTE_1_SHIFT)
#define CAN_WORD085_DATA_BYTE_1(n)     (((n) << CAN_WORD085_DATA_BYTE_1_SHIFT) & CAN_WORD085_DATA_BYTE_1_MASK)
#define CAN_WORD085_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD085_DATA_BYTE_0_MASK   (0xFF << CAN_WORD085_DATA_BYTE_0_SHIFT)
#define CAN_WORD085_DATA_BYTE_0(n)     (((n) << CAN_WORD085_DATA_BYTE_0_SHIFT) & CAN_WORD085_DATA_BYTE_0_MASK)

/* Message Buffer 85 WORD1 Register (WORD185) */
#define CAN_WORD185_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD185_DATA_BYTE_7_MASK   (0xFF << CAN_WORD185_DATA_BYTE_7_SHIFT)
#define CAN_WORD185_DATA_BYTE_7(n)     (((n) << CAN_WORD185_DATA_BYTE_7_SHIFT) & CAN_WORD185_DATA_BYTE_7_MASK)
#define CAN_WORD185_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD185_DATA_BYTE_6_MASK   (0xFF << CAN_WORD185_DATA_BYTE_6_SHIFT)
#define CAN_WORD185_DATA_BYTE_6(n)     (((n) << CAN_WORD185_DATA_BYTE_6_SHIFT) & CAN_WORD185_DATA_BYTE_6_MASK)
#define CAN_WORD185_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD185_DATA_BYTE_5_MASK   (0xFF << CAN_WORD185_DATA_BYTE_5_SHIFT)
#define CAN_WORD185_DATA_BYTE_5(n)     (((n) << CAN_WORD185_DATA_BYTE_5_SHIFT) & CAN_WORD185_DATA_BYTE_5_MASK)
#define CAN_WORD185_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD185_DATA_BYTE_4_MASK   (0xFF << CAN_WORD185_DATA_BYTE_4_SHIFT)
#define CAN_WORD185_DATA_BYTE_4(n)     (((n) << CAN_WORD185_DATA_BYTE_4_SHIFT) & CAN_WORD185_DATA_BYTE_4_MASK)

/* Message Buffer 86 CS Register (CS86) */
#define CAN_CS86_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS86_TIME_STAMP_MASK   (0xFFFF << CAN_CS86_TIME_STAMP_SHIFT)
#define CAN_CS86_TIME_STAMP(n)     (((n) << CAN_CS86_TIME_STAMP_SHIFT) & CAN_CS86_TIME_STAMP_MASK)
#define CAN_CS86_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS86_DLC_MASK          (0xF << CAN_CS86_DLC_SHIFT)
#define CAN_CS86_DLC(n)            (((n) << CAN_CS86_DLC_SHIFT) & CAN_CS86_DLC_MASK)
#define CAN_CS86_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS86_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS86_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS86_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS86_CODE_MASK         (0xF << CAN_CS86_CODE_SHIFT)
#define CAN_CS86_CODE(n)           (((n) << CAN_CS86_CODE_SHIFT) & CAN_CS86_CODE_MASK)
#define CAN_CS86_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS86_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS86_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 86 WORD0 Register (WORD086) */
#define CAN_WORD086_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD086_DATA_BYTE_3_MASK   (0xFF << CAN_WORD086_DATA_BYTE_3_SHIFT)
#define CAN_WORD086_DATA_BYTE_3(n)     (((n) << CAN_WORD086_DATA_BYTE_3_SHIFT) & CAN_WORD086_DATA_BYTE_3_MASK)
#define CAN_WORD086_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD086_DATA_BYTE_2_MASK   (0xFF << CAN_WORD086_DATA_BYTE_2_SHIFT)
#define CAN_WORD086_DATA_BYTE_2(n)     (((n) << CAN_WORD086_DATA_BYTE_2_SHIFT) & CAN_WORD086_DATA_BYTE_2_MASK)
#define CAN_WORD086_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD086_DATA_BYTE_1_MASK   (0xFF << CAN_WORD086_DATA_BYTE_1_SHIFT)
#define CAN_WORD086_DATA_BYTE_1(n)     (((n) << CAN_WORD086_DATA_BYTE_1_SHIFT) & CAN_WORD086_DATA_BYTE_1_MASK)
#define CAN_WORD086_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD086_DATA_BYTE_0_MASK   (0xFF << CAN_WORD086_DATA_BYTE_0_SHIFT)
#define CAN_WORD086_DATA_BYTE_0(n)     (((n) << CAN_WORD086_DATA_BYTE_0_SHIFT) & CAN_WORD086_DATA_BYTE_0_MASK)

/* Message Buffer 86 WORD1 Register (WORD186) */
#define CAN_WORD186_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD186_DATA_BYTE_7_MASK   (0xFF << CAN_WORD186_DATA_BYTE_7_SHIFT)
#define CAN_WORD186_DATA_BYTE_7(n)     (((n) << CAN_WORD186_DATA_BYTE_7_SHIFT) & CAN_WORD186_DATA_BYTE_7_MASK)
#define CAN_WORD186_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD186_DATA_BYTE_6_MASK   (0xFF << CAN_WORD186_DATA_BYTE_6_SHIFT)
#define CAN_WORD186_DATA_BYTE_6(n)     (((n) << CAN_WORD186_DATA_BYTE_6_SHIFT) & CAN_WORD186_DATA_BYTE_6_MASK)
#define CAN_WORD186_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD186_DATA_BYTE_5_MASK   (0xFF << CAN_WORD186_DATA_BYTE_5_SHIFT)
#define CAN_WORD186_DATA_BYTE_5(n)     (((n) << CAN_WORD186_DATA_BYTE_5_SHIFT) & CAN_WORD186_DATA_BYTE_5_MASK)
#define CAN_WORD186_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD186_DATA_BYTE_4_MASK   (0xFF << CAN_WORD186_DATA_BYTE_4_SHIFT)
#define CAN_WORD186_DATA_BYTE_4(n)     (((n) << CAN_WORD186_DATA_BYTE_4_SHIFT) & CAN_WORD186_DATA_BYTE_4_MASK)

/* Message Buffer 87 CS Register (CS87) */
#define CAN_CS87_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS87_TIME_STAMP_MASK   (0xFFFF << CAN_CS87_TIME_STAMP_SHIFT)
#define CAN_CS87_TIME_STAMP(n)     (((n) << CAN_CS87_TIME_STAMP_SHIFT) & CAN_CS87_TIME_STAMP_MASK)
#define CAN_CS87_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS87_DLC_MASK          (0xF << CAN_CS87_DLC_SHIFT)
#define CAN_CS87_DLC(n)            (((n) << CAN_CS87_DLC_SHIFT) & CAN_CS87_DLC_MASK)
#define CAN_CS87_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS87_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS87_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS87_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS87_CODE_MASK         (0xF << CAN_CS87_CODE_SHIFT)
#define CAN_CS87_CODE(n)           (((n) << CAN_CS87_CODE_SHIFT) & CAN_CS87_CODE_MASK)
#define CAN_CS87_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS87_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS87_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 87 WORD0 Register (WORD087) */
#define CAN_WORD087_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD087_DATA_BYTE_3_MASK   (0xFF << CAN_WORD087_DATA_BYTE_3_SHIFT)
#define CAN_WORD087_DATA_BYTE_3(n)     (((n) << CAN_WORD087_DATA_BYTE_3_SHIFT) & CAN_WORD087_DATA_BYTE_3_MASK)
#define CAN_WORD087_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD087_DATA_BYTE_2_MASK   (0xFF << CAN_WORD087_DATA_BYTE_2_SHIFT)
#define CAN_WORD087_DATA_BYTE_2(n)     (((n) << CAN_WORD087_DATA_BYTE_2_SHIFT) & CAN_WORD087_DATA_BYTE_2_MASK)
#define CAN_WORD087_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD087_DATA_BYTE_1_MASK   (0xFF << CAN_WORD087_DATA_BYTE_1_SHIFT)
#define CAN_WORD087_DATA_BYTE_1(n)     (((n) << CAN_WORD087_DATA_BYTE_1_SHIFT) & CAN_WORD087_DATA_BYTE_1_MASK)
#define CAN_WORD087_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD087_DATA_BYTE_0_MASK   (0xFF << CAN_WORD087_DATA_BYTE_0_SHIFT)
#define CAN_WORD087_DATA_BYTE_0(n)     (((n) << CAN_WORD087_DATA_BYTE_0_SHIFT) & CAN_WORD087_DATA_BYTE_0_MASK)

/* Message Buffer 87 WORD1 Register (WORD187) */
#define CAN_WORD187_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD187_DATA_BYTE_7_MASK   (0xFF << CAN_WORD187_DATA_BYTE_7_SHIFT)
#define CAN_WORD187_DATA_BYTE_7(n)     (((n) << CAN_WORD187_DATA_BYTE_7_SHIFT) & CAN_WORD187_DATA_BYTE_7_MASK)
#define CAN_WORD187_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD187_DATA_BYTE_6_MASK   (0xFF << CAN_WORD187_DATA_BYTE_6_SHIFT)
#define CAN_WORD187_DATA_BYTE_6(n)     (((n) << CAN_WORD187_DATA_BYTE_6_SHIFT) & CAN_WORD187_DATA_BYTE_6_MASK)
#define CAN_WORD187_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD187_DATA_BYTE_5_MASK   (0xFF << CAN_WORD187_DATA_BYTE_5_SHIFT)
#define CAN_WORD187_DATA_BYTE_5(n)     (((n) << CAN_WORD187_DATA_BYTE_5_SHIFT) & CAN_WORD187_DATA_BYTE_5_MASK)
#define CAN_WORD187_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD187_DATA_BYTE_4_MASK   (0xFF << CAN_WORD187_DATA_BYTE_4_SHIFT)
#define CAN_WORD187_DATA_BYTE_4(n)     (((n) << CAN_WORD187_DATA_BYTE_4_SHIFT) & CAN_WORD187_DATA_BYTE_4_MASK)

/* Message Buffer 88 CS Register (CS88) */
#define CAN_CS88_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS88_TIME_STAMP_MASK   (0xFFFF << CAN_CS88_TIME_STAMP_SHIFT)
#define CAN_CS88_TIME_STAMP(n)     (((n) << CAN_CS88_TIME_STAMP_SHIFT) & CAN_CS88_TIME_STAMP_MASK)
#define CAN_CS88_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS88_DLC_MASK          (0xF << CAN_CS88_DLC_SHIFT)
#define CAN_CS88_DLC(n)            (((n) << CAN_CS88_DLC_SHIFT) & CAN_CS88_DLC_MASK)
#define CAN_CS88_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS88_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS88_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS88_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS88_CODE_MASK         (0xF << CAN_CS88_CODE_SHIFT)
#define CAN_CS88_CODE(n)           (((n) << CAN_CS88_CODE_SHIFT) & CAN_CS88_CODE_MASK)
#define CAN_CS88_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS88_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS88_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 88 WORD0 Register (WORD088) */
#define CAN_WORD088_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD088_DATA_BYTE_3_MASK   (0xFF << CAN_WORD088_DATA_BYTE_3_SHIFT)
#define CAN_WORD088_DATA_BYTE_3(n)     (((n) << CAN_WORD088_DATA_BYTE_3_SHIFT) & CAN_WORD088_DATA_BYTE_3_MASK)
#define CAN_WORD088_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD088_DATA_BYTE_2_MASK   (0xFF << CAN_WORD088_DATA_BYTE_2_SHIFT)
#define CAN_WORD088_DATA_BYTE_2(n)     (((n) << CAN_WORD088_DATA_BYTE_2_SHIFT) & CAN_WORD088_DATA_BYTE_2_MASK)
#define CAN_WORD088_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD088_DATA_BYTE_1_MASK   (0xFF << CAN_WORD088_DATA_BYTE_1_SHIFT)
#define CAN_WORD088_DATA_BYTE_1(n)     (((n) << CAN_WORD088_DATA_BYTE_1_SHIFT) & CAN_WORD088_DATA_BYTE_1_MASK)
#define CAN_WORD088_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD088_DATA_BYTE_0_MASK   (0xFF << CAN_WORD088_DATA_BYTE_0_SHIFT)
#define CAN_WORD088_DATA_BYTE_0(n)     (((n) << CAN_WORD088_DATA_BYTE_0_SHIFT) & CAN_WORD088_DATA_BYTE_0_MASK)

/* Message Buffer 88 WORD1 Register (WORD188) */
#define CAN_WORD188_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD188_DATA_BYTE_7_MASK   (0xFF << CAN_WORD188_DATA_BYTE_7_SHIFT)
#define CAN_WORD188_DATA_BYTE_7(n)     (((n) << CAN_WORD188_DATA_BYTE_7_SHIFT) & CAN_WORD188_DATA_BYTE_7_MASK)
#define CAN_WORD188_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD188_DATA_BYTE_6_MASK   (0xFF << CAN_WORD188_DATA_BYTE_6_SHIFT)
#define CAN_WORD188_DATA_BYTE_6(n)     (((n) << CAN_WORD188_DATA_BYTE_6_SHIFT) & CAN_WORD188_DATA_BYTE_6_MASK)
#define CAN_WORD188_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD188_DATA_BYTE_5_MASK   (0xFF << CAN_WORD188_DATA_BYTE_5_SHIFT)
#define CAN_WORD188_DATA_BYTE_5(n)     (((n) << CAN_WORD188_DATA_BYTE_5_SHIFT) & CAN_WORD188_DATA_BYTE_5_MASK)
#define CAN_WORD188_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD188_DATA_BYTE_4_MASK   (0xFF << CAN_WORD188_DATA_BYTE_4_SHIFT)
#define CAN_WORD188_DATA_BYTE_4(n)     (((n) << CAN_WORD188_DATA_BYTE_4_SHIFT) & CAN_WORD188_DATA_BYTE_4_MASK)

/* Message Buffer 89 CS Register (CS89) */
#define CAN_CS89_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS89_TIME_STAMP_MASK   (0xFFFF << CAN_CS89_TIME_STAMP_SHIFT)
#define CAN_CS89_TIME_STAMP(n)     (((n) << CAN_CS89_TIME_STAMP_SHIFT) & CAN_CS89_TIME_STAMP_MASK)
#define CAN_CS89_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS89_DLC_MASK          (0xF << CAN_CS89_DLC_SHIFT)
#define CAN_CS89_DLC(n)            (((n) << CAN_CS89_DLC_SHIFT) & CAN_CS89_DLC_MASK)
#define CAN_CS89_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS89_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS89_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS89_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS89_CODE_MASK         (0xF << CAN_CS89_CODE_SHIFT)
#define CAN_CS89_CODE(n)           (((n) << CAN_CS89_CODE_SHIFT) & CAN_CS89_CODE_MASK)
#define CAN_CS89_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS89_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS89_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 89 WORD0 Register (WORD089) */
#define CAN_WORD089_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD089_DATA_BYTE_3_MASK   (0xFF << CAN_WORD089_DATA_BYTE_3_SHIFT)
#define CAN_WORD089_DATA_BYTE_3(n)     (((n) << CAN_WORD089_DATA_BYTE_3_SHIFT) & CAN_WORD089_DATA_BYTE_3_MASK)
#define CAN_WORD089_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD089_DATA_BYTE_2_MASK   (0xFF << CAN_WORD089_DATA_BYTE_2_SHIFT)
#define CAN_WORD089_DATA_BYTE_2(n)     (((n) << CAN_WORD089_DATA_BYTE_2_SHIFT) & CAN_WORD089_DATA_BYTE_2_MASK)
#define CAN_WORD089_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD089_DATA_BYTE_1_MASK   (0xFF << CAN_WORD089_DATA_BYTE_1_SHIFT)
#define CAN_WORD089_DATA_BYTE_1(n)     (((n) << CAN_WORD089_DATA_BYTE_1_SHIFT) & CAN_WORD089_DATA_BYTE_1_MASK)
#define CAN_WORD089_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD089_DATA_BYTE_0_MASK   (0xFF << CAN_WORD089_DATA_BYTE_0_SHIFT)
#define CAN_WORD089_DATA_BYTE_0(n)     (((n) << CAN_WORD089_DATA_BYTE_0_SHIFT) & CAN_WORD089_DATA_BYTE_0_MASK)

/* Message Buffer 89 WORD1 Register (WORD189) */
#define CAN_WORD189_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD189_DATA_BYTE_7_MASK   (0xFF << CAN_WORD189_DATA_BYTE_7_SHIFT)
#define CAN_WORD189_DATA_BYTE_7(n)     (((n) << CAN_WORD189_DATA_BYTE_7_SHIFT) & CAN_WORD189_DATA_BYTE_7_MASK)
#define CAN_WORD189_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD189_DATA_BYTE_6_MASK   (0xFF << CAN_WORD189_DATA_BYTE_6_SHIFT)
#define CAN_WORD189_DATA_BYTE_6(n)     (((n) << CAN_WORD189_DATA_BYTE_6_SHIFT) & CAN_WORD189_DATA_BYTE_6_MASK)
#define CAN_WORD189_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD189_DATA_BYTE_5_MASK   (0xFF << CAN_WORD189_DATA_BYTE_5_SHIFT)
#define CAN_WORD189_DATA_BYTE_5(n)     (((n) << CAN_WORD189_DATA_BYTE_5_SHIFT) & CAN_WORD189_DATA_BYTE_5_MASK)
#define CAN_WORD189_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD189_DATA_BYTE_4_MASK   (0xFF << CAN_WORD189_DATA_BYTE_4_SHIFT)
#define CAN_WORD189_DATA_BYTE_4(n)     (((n) << CAN_WORD189_DATA_BYTE_4_SHIFT) & CAN_WORD189_DATA_BYTE_4_MASK)

/* Message Buffer 90 CS Register (CS90) */
#define CAN_CS90_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS90_TIME_STAMP_MASK   (0xFFFF << CAN_CS90_TIME_STAMP_SHIFT)
#define CAN_CS90_TIME_STAMP(n)     (((n) << CAN_CS90_TIME_STAMP_SHIFT) & CAN_CS90_TIME_STAMP_MASK)
#define CAN_CS90_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS90_DLC_MASK          (0xF << CAN_CS90_DLC_SHIFT)
#define CAN_CS90_DLC(n)            (((n) << CAN_CS90_DLC_SHIFT) & CAN_CS90_DLC_MASK)
#define CAN_CS90_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS90_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS90_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS90_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS90_CODE_MASK         (0xF << CAN_CS90_CODE_SHIFT)
#define CAN_CS90_CODE(n)           (((n) << CAN_CS90_CODE_SHIFT) & CAN_CS90_CODE_MASK)
#define CAN_CS90_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS90_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS90_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 90 WORD0 Register (WORD090) */
#define CAN_WORD090_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD090_DATA_BYTE_3_MASK   (0xFF << CAN_WORD090_DATA_BYTE_3_SHIFT)
#define CAN_WORD090_DATA_BYTE_3(n)     (((n) << CAN_WORD090_DATA_BYTE_3_SHIFT) & CAN_WORD090_DATA_BYTE_3_MASK)
#define CAN_WORD090_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD090_DATA_BYTE_2_MASK   (0xFF << CAN_WORD090_DATA_BYTE_2_SHIFT)
#define CAN_WORD090_DATA_BYTE_2(n)     (((n) << CAN_WORD090_DATA_BYTE_2_SHIFT) & CAN_WORD090_DATA_BYTE_2_MASK)
#define CAN_WORD090_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD090_DATA_BYTE_1_MASK   (0xFF << CAN_WORD090_DATA_BYTE_1_SHIFT)
#define CAN_WORD090_DATA_BYTE_1(n)     (((n) << CAN_WORD090_DATA_BYTE_1_SHIFT) & CAN_WORD090_DATA_BYTE_1_MASK)
#define CAN_WORD090_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD090_DATA_BYTE_0_MASK   (0xFF << CAN_WORD090_DATA_BYTE_0_SHIFT)
#define CAN_WORD090_DATA_BYTE_0(n)     (((n) << CAN_WORD090_DATA_BYTE_0_SHIFT) & CAN_WORD090_DATA_BYTE_0_MASK)

/* Message Buffer 90 WORD1 Register (WORD190) */
#define CAN_WORD190_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD190_DATA_BYTE_7_MASK   (0xFF << CAN_WORD190_DATA_BYTE_7_SHIFT)
#define CAN_WORD190_DATA_BYTE_7(n)     (((n) << CAN_WORD190_DATA_BYTE_7_SHIFT) & CAN_WORD190_DATA_BYTE_7_MASK)
#define CAN_WORD190_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD190_DATA_BYTE_6_MASK   (0xFF << CAN_WORD190_DATA_BYTE_6_SHIFT)
#define CAN_WORD190_DATA_BYTE_6(n)     (((n) << CAN_WORD190_DATA_BYTE_6_SHIFT) & CAN_WORD190_DATA_BYTE_6_MASK)
#define CAN_WORD190_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD190_DATA_BYTE_5_MASK   (0xFF << CAN_WORD190_DATA_BYTE_5_SHIFT)
#define CAN_WORD190_DATA_BYTE_5(n)     (((n) << CAN_WORD190_DATA_BYTE_5_SHIFT) & CAN_WORD190_DATA_BYTE_5_MASK)
#define CAN_WORD190_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD190_DATA_BYTE_4_MASK   (0xFF << CAN_WORD190_DATA_BYTE_4_SHIFT)
#define CAN_WORD190_DATA_BYTE_4(n)     (((n) << CAN_WORD190_DATA_BYTE_4_SHIFT) & CAN_WORD190_DATA_BYTE_4_MASK)

/* Message Buffer 91 CS Register (CS91) */
#define CAN_CS91_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS91_TIME_STAMP_MASK   (0xFFFF << CAN_CS91_TIME_STAMP_SHIFT)
#define CAN_CS91_TIME_STAMP(n)     (((n) << CAN_CS91_TIME_STAMP_SHIFT) & CAN_CS91_TIME_STAMP_MASK)
#define CAN_CS91_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS91_DLC_MASK          (0xF << CAN_CS91_DLC_SHIFT)
#define CAN_CS91_DLC(n)            (((n) << CAN_CS91_DLC_SHIFT) & CAN_CS91_DLC_MASK)
#define CAN_CS91_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS91_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS91_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS91_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS91_CODE_MASK         (0xF << CAN_CS91_CODE_SHIFT)
#define CAN_CS91_CODE(n)           (((n) << CAN_CS91_CODE_SHIFT) & CAN_CS91_CODE_MASK)
#define CAN_CS91_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS91_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS91_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 91 WORD0 Register (WORD091) */
#define CAN_WORD091_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD091_DATA_BYTE_3_MASK   (0xFF << CAN_WORD091_DATA_BYTE_3_SHIFT)
#define CAN_WORD091_DATA_BYTE_3(n)     (((n) << CAN_WORD091_DATA_BYTE_3_SHIFT) & CAN_WORD091_DATA_BYTE_3_MASK)
#define CAN_WORD091_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD091_DATA_BYTE_2_MASK   (0xFF << CAN_WORD091_DATA_BYTE_2_SHIFT)
#define CAN_WORD091_DATA_BYTE_2(n)     (((n) << CAN_WORD091_DATA_BYTE_2_SHIFT) & CAN_WORD091_DATA_BYTE_2_MASK)
#define CAN_WORD091_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD091_DATA_BYTE_1_MASK   (0xFF << CAN_WORD091_DATA_BYTE_1_SHIFT)
#define CAN_WORD091_DATA_BYTE_1(n)     (((n) << CAN_WORD091_DATA_BYTE_1_SHIFT) & CAN_WORD091_DATA_BYTE_1_MASK)
#define CAN_WORD091_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD091_DATA_BYTE_0_MASK   (0xFF << CAN_WORD091_DATA_BYTE_0_SHIFT)
#define CAN_WORD091_DATA_BYTE_0(n)     (((n) << CAN_WORD091_DATA_BYTE_0_SHIFT) & CAN_WORD091_DATA_BYTE_0_MASK)

/* Message Buffer 91 WORD1 Register (WORD191) */
#define CAN_WORD191_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD191_DATA_BYTE_7_MASK   (0xFF << CAN_WORD191_DATA_BYTE_7_SHIFT)
#define CAN_WORD191_DATA_BYTE_7(n)     (((n) << CAN_WORD191_DATA_BYTE_7_SHIFT) & CAN_WORD191_DATA_BYTE_7_MASK)
#define CAN_WORD191_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD191_DATA_BYTE_6_MASK   (0xFF << CAN_WORD191_DATA_BYTE_6_SHIFT)
#define CAN_WORD191_DATA_BYTE_6(n)     (((n) << CAN_WORD191_DATA_BYTE_6_SHIFT) & CAN_WORD191_DATA_BYTE_6_MASK)
#define CAN_WORD191_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD191_DATA_BYTE_5_MASK   (0xFF << CAN_WORD191_DATA_BYTE_5_SHIFT)
#define CAN_WORD191_DATA_BYTE_5(n)     (((n) << CAN_WORD191_DATA_BYTE_5_SHIFT) & CAN_WORD191_DATA_BYTE_5_MASK)
#define CAN_WORD191_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD191_DATA_BYTE_4_MASK   (0xFF << CAN_WORD191_DATA_BYTE_4_SHIFT)
#define CAN_WORD191_DATA_BYTE_4(n)     (((n) << CAN_WORD191_DATA_BYTE_4_SHIFT) & CAN_WORD191_DATA_BYTE_4_MASK)

/* Message Buffer 92 CS Register (CS92) */
#define CAN_CS92_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS92_TIME_STAMP_MASK   (0xFFFF << CAN_CS92_TIME_STAMP_SHIFT)
#define CAN_CS92_TIME_STAMP(n)     (((n) << CAN_CS92_TIME_STAMP_SHIFT) & CAN_CS92_TIME_STAMP_MASK)
#define CAN_CS92_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS92_DLC_MASK          (0xF << CAN_CS92_DLC_SHIFT)
#define CAN_CS92_DLC(n)            (((n) << CAN_CS92_DLC_SHIFT) & CAN_CS92_DLC_MASK)
#define CAN_CS92_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS92_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS92_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS92_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS92_CODE_MASK         (0xF << CAN_CS92_CODE_SHIFT)
#define CAN_CS92_CODE(n)           (((n) << CAN_CS92_CODE_SHIFT) & CAN_CS92_CODE_MASK)
#define CAN_CS92_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS92_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS92_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 92 WORD0 Register (WORD092) */
#define CAN_WORD092_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD092_DATA_BYTE_3_MASK   (0xFF << CAN_WORD092_DATA_BYTE_3_SHIFT)
#define CAN_WORD092_DATA_BYTE_3(n)     (((n) << CAN_WORD092_DATA_BYTE_3_SHIFT) & CAN_WORD092_DATA_BYTE_3_MASK)
#define CAN_WORD092_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD092_DATA_BYTE_2_MASK   (0xFF << CAN_WORD092_DATA_BYTE_2_SHIFT)
#define CAN_WORD092_DATA_BYTE_2(n)     (((n) << CAN_WORD092_DATA_BYTE_2_SHIFT) & CAN_WORD092_DATA_BYTE_2_MASK)
#define CAN_WORD092_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD092_DATA_BYTE_1_MASK   (0xFF << CAN_WORD092_DATA_BYTE_1_SHIFT)
#define CAN_WORD092_DATA_BYTE_1(n)     (((n) << CAN_WORD092_DATA_BYTE_1_SHIFT) & CAN_WORD092_DATA_BYTE_1_MASK)
#define CAN_WORD092_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD092_DATA_BYTE_0_MASK   (0xFF << CAN_WORD092_DATA_BYTE_0_SHIFT)
#define CAN_WORD092_DATA_BYTE_0(n)     (((n) << CAN_WORD092_DATA_BYTE_0_SHIFT) & CAN_WORD092_DATA_BYTE_0_MASK)

/* Message Buffer 92 WORD1 Register (WORD192) */
#define CAN_WORD192_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD192_DATA_BYTE_7_MASK   (0xFF << CAN_WORD192_DATA_BYTE_7_SHIFT)
#define CAN_WORD192_DATA_BYTE_7(n)     (((n) << CAN_WORD192_DATA_BYTE_7_SHIFT) & CAN_WORD192_DATA_BYTE_7_MASK)
#define CAN_WORD192_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD192_DATA_BYTE_6_MASK   (0xFF << CAN_WORD192_DATA_BYTE_6_SHIFT)
#define CAN_WORD192_DATA_BYTE_6(n)     (((n) << CAN_WORD192_DATA_BYTE_6_SHIFT) & CAN_WORD192_DATA_BYTE_6_MASK)
#define CAN_WORD192_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD192_DATA_BYTE_5_MASK   (0xFF << CAN_WORD192_DATA_BYTE_5_SHIFT)
#define CAN_WORD192_DATA_BYTE_5(n)     (((n) << CAN_WORD192_DATA_BYTE_5_SHIFT) & CAN_WORD192_DATA_BYTE_5_MASK)
#define CAN_WORD192_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD192_DATA_BYTE_4_MASK   (0xFF << CAN_WORD192_DATA_BYTE_4_SHIFT)
#define CAN_WORD192_DATA_BYTE_4(n)     (((n) << CAN_WORD192_DATA_BYTE_4_SHIFT) & CAN_WORD192_DATA_BYTE_4_MASK)

/* Message Buffer 93 CS Register (CS93) */
#define CAN_CS93_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS93_TIME_STAMP_MASK   (0xFFFF << CAN_CS93_TIME_STAMP_SHIFT)
#define CAN_CS93_TIME_STAMP(n)     (((n) << CAN_CS93_TIME_STAMP_SHIFT) & CAN_CS93_TIME_STAMP_MASK)
#define CAN_CS93_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS93_DLC_MASK          (0xF << CAN_CS93_DLC_SHIFT)
#define CAN_CS93_DLC(n)            (((n) << CAN_CS93_DLC_SHIFT) & CAN_CS93_DLC_MASK)
#define CAN_CS93_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS93_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS93_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS93_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS93_CODE_MASK         (0xF << CAN_CS93_CODE_SHIFT)
#define CAN_CS93_CODE(n)           (((n) << CAN_CS93_CODE_SHIFT) & CAN_CS93_CODE_MASK)
#define CAN_CS93_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS93_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS93_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 93 WORD0 Register (WORD093) */
#define CAN_WORD093_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD093_DATA_BYTE_3_MASK   (0xFF << CAN_WORD093_DATA_BYTE_3_SHIFT)
#define CAN_WORD093_DATA_BYTE_3(n)     (((n) << CAN_WORD093_DATA_BYTE_3_SHIFT) & CAN_WORD093_DATA_BYTE_3_MASK)
#define CAN_WORD093_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD093_DATA_BYTE_2_MASK   (0xFF << CAN_WORD093_DATA_BYTE_2_SHIFT)
#define CAN_WORD093_DATA_BYTE_2(n)     (((n) << CAN_WORD093_DATA_BYTE_2_SHIFT) & CAN_WORD093_DATA_BYTE_2_MASK)
#define CAN_WORD093_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD093_DATA_BYTE_1_MASK   (0xFF << CAN_WORD093_DATA_BYTE_1_SHIFT)
#define CAN_WORD093_DATA_BYTE_1(n)     (((n) << CAN_WORD093_DATA_BYTE_1_SHIFT) & CAN_WORD093_DATA_BYTE_1_MASK)
#define CAN_WORD093_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD093_DATA_BYTE_0_MASK   (0xFF << CAN_WORD093_DATA_BYTE_0_SHIFT)
#define CAN_WORD093_DATA_BYTE_0(n)     (((n) << CAN_WORD093_DATA_BYTE_0_SHIFT) & CAN_WORD093_DATA_BYTE_0_MASK)

/* Message Buffer 93 WORD1 Register (WORD193) */
#define CAN_WORD193_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD193_DATA_BYTE_7_MASK   (0xFF << CAN_WORD193_DATA_BYTE_7_SHIFT)
#define CAN_WORD193_DATA_BYTE_7(n)     (((n) << CAN_WORD193_DATA_BYTE_7_SHIFT) & CAN_WORD193_DATA_BYTE_7_MASK)
#define CAN_WORD193_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD193_DATA_BYTE_6_MASK   (0xFF << CAN_WORD193_DATA_BYTE_6_SHIFT)
#define CAN_WORD193_DATA_BYTE_6(n)     (((n) << CAN_WORD193_DATA_BYTE_6_SHIFT) & CAN_WORD193_DATA_BYTE_6_MASK)
#define CAN_WORD193_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD193_DATA_BYTE_5_MASK   (0xFF << CAN_WORD193_DATA_BYTE_5_SHIFT)
#define CAN_WORD193_DATA_BYTE_5(n)     (((n) << CAN_WORD193_DATA_BYTE_5_SHIFT) & CAN_WORD193_DATA_BYTE_5_MASK)
#define CAN_WORD193_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD193_DATA_BYTE_4_MASK   (0xFF << CAN_WORD193_DATA_BYTE_4_SHIFT)
#define CAN_WORD193_DATA_BYTE_4(n)     (((n) << CAN_WORD193_DATA_BYTE_4_SHIFT) & CAN_WORD193_DATA_BYTE_4_MASK)

/* Message Buffer 94 CS Register (CS94) */
#define CAN_CS94_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS94_TIME_STAMP_MASK   (0xFFFF << CAN_CS94_TIME_STAMP_SHIFT)
#define CAN_CS94_TIME_STAMP(n)     (((n) << CAN_CS94_TIME_STAMP_SHIFT) & CAN_CS94_TIME_STAMP_MASK)
#define CAN_CS94_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS94_DLC_MASK          (0xF << CAN_CS94_DLC_SHIFT)
#define CAN_CS94_DLC(n)            (((n) << CAN_CS94_DLC_SHIFT) & CAN_CS94_DLC_MASK)
#define CAN_CS94_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS94_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS94_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS94_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS94_CODE_MASK         (0xF << CAN_CS94_CODE_SHIFT)
#define CAN_CS94_CODE(n)           (((n) << CAN_CS94_CODE_SHIFT) & CAN_CS94_CODE_MASK)
#define CAN_CS94_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS94_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS94_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 94 WORD0 Register (WORD094) */
#define CAN_WORD094_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD094_DATA_BYTE_3_MASK   (0xFF << CAN_WORD094_DATA_BYTE_3_SHIFT)
#define CAN_WORD094_DATA_BYTE_3(n)     (((n) << CAN_WORD094_DATA_BYTE_3_SHIFT) & CAN_WORD094_DATA_BYTE_3_MASK)
#define CAN_WORD094_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD094_DATA_BYTE_2_MASK   (0xFF << CAN_WORD094_DATA_BYTE_2_SHIFT)
#define CAN_WORD094_DATA_BYTE_2(n)     (((n) << CAN_WORD094_DATA_BYTE_2_SHIFT) & CAN_WORD094_DATA_BYTE_2_MASK)
#define CAN_WORD094_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD094_DATA_BYTE_1_MASK   (0xFF << CAN_WORD094_DATA_BYTE_1_SHIFT)
#define CAN_WORD094_DATA_BYTE_1(n)     (((n) << CAN_WORD094_DATA_BYTE_1_SHIFT) & CAN_WORD094_DATA_BYTE_1_MASK)
#define CAN_WORD094_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD094_DATA_BYTE_0_MASK   (0xFF << CAN_WORD094_DATA_BYTE_0_SHIFT)
#define CAN_WORD094_DATA_BYTE_0(n)     (((n) << CAN_WORD094_DATA_BYTE_0_SHIFT) & CAN_WORD094_DATA_BYTE_0_MASK)

/* Message Buffer 94 WORD1 Register (WORD194) */
#define CAN_WORD194_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD194_DATA_BYTE_7_MASK   (0xFF << CAN_WORD194_DATA_BYTE_7_SHIFT)
#define CAN_WORD194_DATA_BYTE_7(n)     (((n) << CAN_WORD194_DATA_BYTE_7_SHIFT) & CAN_WORD194_DATA_BYTE_7_MASK)
#define CAN_WORD194_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD194_DATA_BYTE_6_MASK   (0xFF << CAN_WORD194_DATA_BYTE_6_SHIFT)
#define CAN_WORD194_DATA_BYTE_6(n)     (((n) << CAN_WORD194_DATA_BYTE_6_SHIFT) & CAN_WORD194_DATA_BYTE_6_MASK)
#define CAN_WORD194_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD194_DATA_BYTE_5_MASK   (0xFF << CAN_WORD194_DATA_BYTE_5_SHIFT)
#define CAN_WORD194_DATA_BYTE_5(n)     (((n) << CAN_WORD194_DATA_BYTE_5_SHIFT) & CAN_WORD194_DATA_BYTE_5_MASK)
#define CAN_WORD194_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD194_DATA_BYTE_4_MASK   (0xFF << CAN_WORD194_DATA_BYTE_4_SHIFT)
#define CAN_WORD194_DATA_BYTE_4(n)     (((n) << CAN_WORD194_DATA_BYTE_4_SHIFT) & CAN_WORD194_DATA_BYTE_4_MASK)

/* Message Buffer 95 CS Register (CS95) */
#define CAN_CS95_TIME_STAMP_SHIFT  (0)        /* Bits 0-16: Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field appears on the CAN bus. */
#define CAN_CS95_TIME_STAMP_MASK   (0xFFFF << CAN_CS95_TIME_STAMP_SHIFT)
#define CAN_CS95_TIME_STAMP(n)     (((n) << CAN_CS95_TIME_STAMP_SHIFT) & CAN_CS95_TIME_STAMP_MASK)
#define CAN_CS95_DLC_SHIFT         (16)       /* Bits 16-20: Length of the data to be stored/transmitted. */
#define CAN_CS95_DLC_MASK          (0xF << CAN_CS95_DLC_SHIFT)
#define CAN_CS95_DLC(n)            (((n) << CAN_CS95_DLC_SHIFT) & CAN_CS95_DLC_MASK)
#define CAN_CS95_RTR               (1 << 20)  /* Bit 20: Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS95_IDE               (1 << 21)  /* Bit 21: ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS95_SRR               (1 << 22)  /* Bit 22: Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS95_CODE_SHIFT        (24)       /* Bits 24-28: Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by the FlexCAN module itself, as part of the message buffer matching and arbitration process. */
#define CAN_CS95_CODE_MASK         (0xF << CAN_CS95_CODE_SHIFT)
#define CAN_CS95_CODE(n)           (((n) << CAN_CS95_CODE_SHIFT) & CAN_CS95_CODE_MASK)
#define CAN_CS95_ESI               (1 << 29)  /* Bit 29: Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS95_BRS               (1 << 30)  /* Bit 30: Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS95_EDL               (1 << 31)  /* Bit 31: Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames. The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */

/* Message Buffer 95 WORD0 Register (WORD095) */
#define CAN_WORD095_DATA_BYTE_3_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD095_DATA_BYTE_3_MASK   (0xFF << CAN_WORD095_DATA_BYTE_3_SHIFT)
#define CAN_WORD095_DATA_BYTE_3(n)     (((n) << CAN_WORD095_DATA_BYTE_3_SHIFT) & CAN_WORD095_DATA_BYTE_3_MASK)
#define CAN_WORD095_DATA_BYTE_2_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD095_DATA_BYTE_2_MASK   (0xFF << CAN_WORD095_DATA_BYTE_2_SHIFT)
#define CAN_WORD095_DATA_BYTE_2(n)     (((n) << CAN_WORD095_DATA_BYTE_2_SHIFT) & CAN_WORD095_DATA_BYTE_2_MASK)
#define CAN_WORD095_DATA_BYTE_1_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD095_DATA_BYTE_1_MASK   (0xFF << CAN_WORD095_DATA_BYTE_1_SHIFT)
#define CAN_WORD095_DATA_BYTE_1(n)     (((n) << CAN_WORD095_DATA_BYTE_1_SHIFT) & CAN_WORD095_DATA_BYTE_1_MASK)
#define CAN_WORD095_DATA_BYTE_0_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD095_DATA_BYTE_0_MASK   (0xFF << CAN_WORD095_DATA_BYTE_0_SHIFT)
#define CAN_WORD095_DATA_BYTE_0(n)     (((n) << CAN_WORD095_DATA_BYTE_0_SHIFT) & CAN_WORD095_DATA_BYTE_0_MASK)

/* Message Buffer 95 WORD1 Register (WORD195) */
#define CAN_WORD195_DATA_BYTE_7_SHIFT  (0)   /* Bits 0-8: Data byte 0 of Rx/Tx frame. */
#define CAN_WORD195_DATA_BYTE_7_MASK   (0xFF << CAN_WORD195_DATA_BYTE_7_SHIFT)
#define CAN_WORD195_DATA_BYTE_7(n)     (((n) << CAN_WORD195_DATA_BYTE_7_SHIFT) & CAN_WORD195_DATA_BYTE_7_MASK)
#define CAN_WORD195_DATA_BYTE_6_SHIFT  (8)   /* Bits 8-16: Data byte 1 of Rx/Tx frame. */
#define CAN_WORD195_DATA_BYTE_6_MASK   (0xFF << CAN_WORD195_DATA_BYTE_6_SHIFT)
#define CAN_WORD195_DATA_BYTE_6(n)     (((n) << CAN_WORD195_DATA_BYTE_6_SHIFT) & CAN_WORD195_DATA_BYTE_6_MASK)
#define CAN_WORD195_DATA_BYTE_5_SHIFT  (16)  /* Bits 16-24: Data byte 2 of Rx/Tx frame. */
#define CAN_WORD195_DATA_BYTE_5_MASK   (0xFF << CAN_WORD195_DATA_BYTE_5_SHIFT)
#define CAN_WORD195_DATA_BYTE_5(n)     (((n) << CAN_WORD195_DATA_BYTE_5_SHIFT) & CAN_WORD195_DATA_BYTE_5_MASK)
#define CAN_WORD195_DATA_BYTE_4_SHIFT  (24)  /* Bits 24-32: Data byte 3 of Rx/Tx frame. */
#define CAN_WORD195_DATA_BYTE_4_MASK   (0xFF << CAN_WORD195_DATA_BYTE_4_SHIFT)
#define CAN_WORD195_DATA_BYTE_4(n)     (((n) << CAN_WORD195_DATA_BYTE_4_SHIFT) & CAN_WORD195_DATA_BYTE_4_MASK)

/* Memory Error Control (MECR) */
#define CAN_MECR_NCEFAFRZ    (1 << 7)   /* Bit 7: Noncorrectable Errors in FlexCAN Access Put Chip in Freeze Mode */
#define CAN_MECR_ECCDIS      (1 << 8)   /* Bit 8: Error Correction Disable */
#define CAN_MECR_RERRDIS     (1 << 9)   /* Bit 9: Error Report Disable */
#define CAN_MECR_EXTERRIE    (1 << 13)  /* Bit 13: Extended Error Injection Enable */
#define CAN_MECR_FAERRIE     (1 << 14)  /* Bit 14: FlexCAN Access Error Injection Enable */
#define CAN_MECR_HAERRIE     (1 << 15)  /* Bit 15: Host Access Error Injection Enable */
#define CAN_MECR_CEI_MSK     (1 << 16)  /* Bit 16: Correctable Errors Interrupt Mask */
#define CAN_MECR_FANCEI_MSK  (1 << 18)  /* Bit 18: FlexCAN Access with Noncorrectable Errors Interrupt Mask */
#define CAN_MECR_HANCEI_MSK  (1 << 19)  /* Bit 19: Host Access with Noncorrectable Errors Interrupt Mask */
#define CAN_MECR_ECRWRDIS    (1 << 31)  /* Bit 31: Error Configuration Register Write Disable */

/* Error Injection Address (ERRIAR) */
#define CAN_ERRIAR_INJADDR_L_SHIFT  (0)  /* Bits 0-2: Error Injection Address Low */
#define CAN_ERRIAR_INJADDR_L_MASK   (0x3 << CAN_ERRIAR_INJADDR_L_SHIFT)
#define CAN_ERRIAR_INJADDR_L(n)     (((n) << CAN_ERRIAR_INJADDR_L_SHIFT) & CAN_ERRIAR_INJADDR_L_MASK)
#define CAN_ERRIAR_INJADDR_H_SHIFT  (2)  /* Bits 2-14: Error Injection Address High */
#define CAN_ERRIAR_INJADDR_H_MASK   (0xFFF << CAN_ERRIAR_INJADDR_H_SHIFT)
#define CAN_ERRIAR_INJADDR_H(n)     (((n) << CAN_ERRIAR_INJADDR_H_SHIFT) & CAN_ERRIAR_INJADDR_H_MASK)

/* Error Injection Data Pattern (ERRIDPR) */
#define CAN_ERRIDPR_DFLIP_SHIFT  (0)  /* Bits 0-32: Data Flip Pattern */
#define CAN_ERRIDPR_DFLIP_MASK   (0xFFFFFFFF << CAN_ERRIDPR_DFLIP_SHIFT)
#define CAN_ERRIDPR_DFLIP(n)     (((n) << CAN_ERRIDPR_DFLIP_SHIFT) & CAN_ERRIDPR_DFLIP_MASK)

/* Error Injection Parity Pattern (ERRIPPR) */
#define CAN_ERRIPPR_PFLIP0_SHIFT  (0)   /* Bits 0-5: Parity Flip Pattern for Byte 0 (Least Significant) */
#define CAN_ERRIPPR_PFLIP0_MASK   (0x1F << CAN_ERRIPPR_PFLIP0_SHIFT)
#define CAN_ERRIPPR_PFLIP0(n)     (((n) << CAN_ERRIPPR_PFLIP0_SHIFT) & CAN_ERRIPPR_PFLIP0_MASK)
#define CAN_ERRIPPR_PFLIP1_SHIFT  (8)   /* Bits 8-13: Parity Flip Pattern for Byte 1 */
#define CAN_ERRIPPR_PFLIP1_MASK   (0x1F << CAN_ERRIPPR_PFLIP1_SHIFT)
#define CAN_ERRIPPR_PFLIP1(n)     (((n) << CAN_ERRIPPR_PFLIP1_SHIFT) & CAN_ERRIPPR_PFLIP1_MASK)
#define CAN_ERRIPPR_PFLIP2_SHIFT  (16)  /* Bits 16-21: Parity Flip Pattern for Byte 2 */
#define CAN_ERRIPPR_PFLIP2_MASK   (0x1F << CAN_ERRIPPR_PFLIP2_SHIFT)
#define CAN_ERRIPPR_PFLIP2(n)     (((n) << CAN_ERRIPPR_PFLIP2_SHIFT) & CAN_ERRIPPR_PFLIP2_MASK)
#define CAN_ERRIPPR_PFLIP3_SHIFT  (24)  /* Bits 24-29: Parity Flip Pattern for Byte 3 (Most Significant) */
#define CAN_ERRIPPR_PFLIP3_MASK   (0x1F << CAN_ERRIPPR_PFLIP3_SHIFT)
#define CAN_ERRIPPR_PFLIP3(n)     (((n) << CAN_ERRIPPR_PFLIP3_SHIFT) & CAN_ERRIPPR_PFLIP3_MASK)

/* Error Report Address (RERRAR) */
#define CAN_RERRAR_ERRADDR_SHIFT  (0)        /* Bits 0-14: Address Where Error Detected */
#define CAN_RERRAR_ERRADDR_MASK   (0x3FFF << CAN_RERRAR_ERRADDR_SHIFT)
#define CAN_RERRAR_ERRADDR(n)     (((n) << CAN_RERRAR_ERRADDR_SHIFT) & CAN_RERRAR_ERRADDR_MASK)
#define CAN_RERRAR_SAID_SHIFT     (16)       /* Bits 16-19: SAID */
#define CAN_RERRAR_SAID_MASK      (0x7 << CAN_RERRAR_SAID_SHIFT)
#define CAN_RERRAR_SAID(n)        (((n) << CAN_RERRAR_SAID_SHIFT) & CAN_RERRAR_SAID_MASK)
#define CAN_RERRAR_NCE            (1 << 24)  /* Bit 24: Noncorrectable Error */

/* Error Report Data (RERRDR) */
#define CAN_RERRDR_RDATA_SHIFT  (0)  /* Bits 0-32: Raw Data Word Read from Memory with Error */
#define CAN_RERRDR_RDATA_MASK   (0xFFFFFFFF << CAN_RERRDR_RDATA_SHIFT)
#define CAN_RERRDR_RDATA(n)     (((n) << CAN_RERRDR_RDATA_SHIFT) & CAN_RERRDR_RDATA_MASK)

/* Error Report Syndrome (RERRSYNR) */
#define CAN_RERRSYNR_SYND0_SHIFT  (0)        /* Bits 0-5: Error Syndrome for Byte 0 (Least Significant) */
#define CAN_RERRSYNR_SYND0_MASK   (0x1F << CAN_RERRSYNR_SYND0_SHIFT)
#define CAN_RERRSYNR_SYND0(n)     (((n) << CAN_RERRSYNR_SYND0_SHIFT) & CAN_RERRSYNR_SYND0_MASK)
#define CAN_RERRSYNR_BE0          (1 << 7)   /* Bit 7: Byte Enabled for Byte 0 (Least Significant) */
#define CAN_RERRSYNR_SYND1_SHIFT  (8)        /* Bits 8-13: Error Syndrome for Byte 1 */
#define CAN_RERRSYNR_SYND1_MASK   (0x1F << CAN_RERRSYNR_SYND1_SHIFT)
#define CAN_RERRSYNR_SYND1(n)     (((n) << CAN_RERRSYNR_SYND1_SHIFT) & CAN_RERRSYNR_SYND1_MASK)
#define CAN_RERRSYNR_BE1          (1 << 15)  /* Bit 15: Byte Enabled for Byte 1 */
#define CAN_RERRSYNR_SYND2_SHIFT  (16)       /* Bits 16-21: Error Syndrome for Byte 2 */
#define CAN_RERRSYNR_SYND2_MASK   (0x1F << CAN_RERRSYNR_SYND2_SHIFT)
#define CAN_RERRSYNR_SYND2(n)     (((n) << CAN_RERRSYNR_SYND2_SHIFT) & CAN_RERRSYNR_SYND2_MASK)
#define CAN_RERRSYNR_BE2          (1 << 23)  /* Bit 23: Byte Enabled for Byte 2 */
#define CAN_RERRSYNR_SYND3_SHIFT  (24)       /* Bits 24-29: Error Syndrome for Byte 3 (Most Significant) */
#define CAN_RERRSYNR_SYND3_MASK   (0x1F << CAN_RERRSYNR_SYND3_SHIFT)
#define CAN_RERRSYNR_SYND3(n)     (((n) << CAN_RERRSYNR_SYND3_SHIFT) & CAN_RERRSYNR_SYND3_MASK)
#define CAN_RERRSYNR_BE3          (1 << 31)  /* Bit 31: Byte Enabled for Byte 3 (Most Significant) */

/* Error Status (ERRSR) */
#define CAN_ERRSR_CEIOF     (1 << 0)   /* Bit 0: Correctable Error Interrupt Overrun Flag */
#define CAN_ERRSR_FANCEIOF  (1 << 2)   /* Bit 2: FlexCAN Access with Noncorrectable Error Interrupt Overrun Flag */
#define CAN_ERRSR_HANCEIOF  (1 << 3)   /* Bit 3: Host Access With Noncorrectable Error Interrupt Overrun Flag */
#define CAN_ERRSR_CEIF      (1 << 16)  /* Bit 16: Correctable Error Interrupt Flag */
#define CAN_ERRSR_FANCEIF   (1 << 18)  /* Bit 18: FlexCAN Access with Noncorrectable Error Interrupt Flag */
#define CAN_ERRSR_HANCEIF   (1 << 19)  /* Bit 19: Host Access with Noncorrectable Error Interrupt Flag */

/* Enhanced CAN Bit Timing Prescalers (EPRS) */
#define CAN_EPRS_ENPRESDIV_SHIFT  (0)   /* Bits 0-10: Extended Nominal Prescaler Division Factor */
#define CAN_EPRS_ENPRESDIV_MASK   (0x3FF << CAN_EPRS_ENPRESDIV_SHIFT)
#define CAN_EPRS_ENPRESDIV(n)     (((n) << CAN_EPRS_ENPRESDIV_SHIFT) & CAN_EPRS_ENPRESDIV_MASK)
#define CAN_EPRS_EDPRESDIV_SHIFT  (16)  /* Bits 16-26: Extended Data Phase Prescaler Division Factor */
#define CAN_EPRS_EDPRESDIV_MASK   (0x3FF << CAN_EPRS_EDPRESDIV_SHIFT)
#define CAN_EPRS_EDPRESDIV(n)     (((n) << CAN_EPRS_EDPRESDIV_SHIFT) & CAN_EPRS_EDPRESDIV_MASK)

/* Enhanced Nominal CAN Bit Timing (ENCBT) */
#define CAN_ENCBT_NTSEG1_SHIFT  (0)   /* Bits 0-8: Nominal Time Segment 1 */
#define CAN_ENCBT_NTSEG1_MASK   (0xFF << CAN_ENCBT_NTSEG1_SHIFT)
#define CAN_ENCBT_NTSEG1(n)     (((n) << CAN_ENCBT_NTSEG1_SHIFT) & CAN_ENCBT_NTSEG1_MASK)
#define CAN_ENCBT_NTSEG2_SHIFT  (12)  /* Bits 12-19: Nominal Time Segment 2 */
#define CAN_ENCBT_NTSEG2_MASK   (0x7F << CAN_ENCBT_NTSEG2_SHIFT)
#define CAN_ENCBT_NTSEG2(n)     (((n) << CAN_ENCBT_NTSEG2_SHIFT) & CAN_ENCBT_NTSEG2_MASK)
#define CAN_ENCBT_NRJW_SHIFT    (22)  /* Bits 22-29: Nominal Resynchronization Jump Width */
#define CAN_ENCBT_NRJW_MASK     (0x7F << CAN_ENCBT_NRJW_SHIFT)
#define CAN_ENCBT_NRJW(n)       (((n) << CAN_ENCBT_NRJW_SHIFT) & CAN_ENCBT_NRJW_MASK)

/* Enhanced Data Phase CAN Bit Timing (EDCBT) */
#define CAN_EDCBT_DTSEG1_SHIFT  (0)   /* Bits 0-5: Data Phase Segment 1 */
#define CAN_EDCBT_DTSEG1_MASK   (0x1F << CAN_EDCBT_DTSEG1_SHIFT)
#define CAN_EDCBT_DTSEG1(n)     (((n) << CAN_EDCBT_DTSEG1_SHIFT) & CAN_EDCBT_DTSEG1_MASK)
#define CAN_EDCBT_DTSEG2_SHIFT  (12)  /* Bits 12-16: Data Phase Time Segment 2 */
#define CAN_EDCBT_DTSEG2_MASK   (0xF << CAN_EDCBT_DTSEG2_SHIFT)
#define CAN_EDCBT_DTSEG2(n)     (((n) << CAN_EDCBT_DTSEG2_SHIFT) & CAN_EDCBT_DTSEG2_MASK)
#define CAN_EDCBT_DRJW_SHIFT    (22)  /* Bits 22-26: Data Phase Resynchronization Jump Width */
#define CAN_EDCBT_DRJW_MASK     (0xF << CAN_EDCBT_DRJW_SHIFT)
#define CAN_EDCBT_DRJW(n)       (((n) << CAN_EDCBT_DRJW_SHIFT) & CAN_EDCBT_DRJW_MASK)

/* Enhanced Transceiver Delay Compensation (ETDC) */
#define CAN_ETDC_ETDCVAL_SHIFT  (0)        /* Bits 0-8: Enhanced Transceiver Delay Compensation Value */
#define CAN_ETDC_ETDCVAL_MASK   (0xFF << CAN_ETDC_ETDCVAL_SHIFT)
#define CAN_ETDC_ETDCVAL(n)     (((n) << CAN_ETDC_ETDCVAL_SHIFT) & CAN_ETDC_ETDCVAL_MASK)
#define CAN_ETDC_ETDCFAIL       (1 << 15)  /* Bit 15: Transceiver Delay Compensation Fail */
#define CAN_ETDC_ETDCOFF_SHIFT  (16)       /* Bits 16-23: Enhanced Transceiver Delay Compensation Offset */
#define CAN_ETDC_ETDCOFF_MASK   (0x7F << CAN_ETDC_ETDCOFF_SHIFT)
#define CAN_ETDC_ETDCOFF(n)     (((n) << CAN_ETDC_ETDCOFF_SHIFT) & CAN_ETDC_ETDCOFF_MASK)
#define CAN_ETDC_TDMDIS         (1 << 30)  /* Bit 30: Transceiver Delay Measurement Disable */
#define CAN_ETDC_ETDCEN         (1 << 31)  /* Bit 31: Transceiver Delay Compensation Enable */

/* CAN FD Control (FDCTRL) */
#define CAN_FDCTRL_TDCVAL_SHIFT  (0)        /* Bits 0-6: Transceiver Delay Compensation Value */
#define CAN_FDCTRL_TDCVAL_MASK   (0x3F << CAN_FDCTRL_TDCVAL_SHIFT)
#define CAN_FDCTRL_TDCVAL(n)     (((n) << CAN_FDCTRL_TDCVAL_SHIFT) & CAN_FDCTRL_TDCVAL_MASK)
#define CAN_FDCTRL_TDCOFF_SHIFT  (8)        /* Bits 8-13: Transceiver Delay Compensation Offset */
#define CAN_FDCTRL_TDCOFF_MASK   (0x1F << CAN_FDCTRL_TDCOFF_SHIFT)
#define CAN_FDCTRL_TDCOFF(n)     (((n) << CAN_FDCTRL_TDCOFF_SHIFT) & CAN_FDCTRL_TDCOFF_MASK)
#define CAN_FDCTRL_TDCFAIL       (1 << 14)  /* Bit 14: Transceiver Delay Compensation Fail */
#define CAN_FDCTRL_TDCEN         (1 << 15)  /* Bit 15: Transceiver Delay Compensation Enable */
#define CAN_FDCTRL_MBDSR0_SHIFT  (16)       /* Bits 16-18: Message Buffer Data Size for Region 0 */
#define CAN_FDCTRL_MBDSR0_MASK   (0x3 << CAN_FDCTRL_MBDSR0_SHIFT)
#define CAN_FDCTRL_MBDSR0(n)     (((n) << CAN_FDCTRL_MBDSR0_SHIFT) & CAN_FDCTRL_MBDSR0_MASK)
#define CAN_FDCTRL_MBDSR1_SHIFT  (19)       /* Bits 19-21: Message Buffer Data Size for Region 1 */
#define CAN_FDCTRL_MBDSR1_MASK   (0x3 << CAN_FDCTRL_MBDSR1_SHIFT)
#define CAN_FDCTRL_MBDSR1(n)     (((n) << CAN_FDCTRL_MBDSR1_SHIFT) & CAN_FDCTRL_MBDSR1_MASK)
#define CAN_FDCTRL_MBDSR2_SHIFT  (22)       /* Bits 22-24: Message Buffer Data Size for Region 2 */
#define CAN_FDCTRL_MBDSR2_MASK   (0x3 << CAN_FDCTRL_MBDSR2_SHIFT)
#define CAN_FDCTRL_MBDSR2(n)     (((n) << CAN_FDCTRL_MBDSR2_SHIFT) & CAN_FDCTRL_MBDSR2_MASK)
#define CAN_FDCTRL_FDRATE        (1 << 31)  /* Bit 31: Bit Rate Switch Enable */

/* CAN FD Bit Timing (FDCBT) */
#define CAN_FDCBT_FPSEG2_SHIFT    (0)   /* Bits 0-3: Fast Phase Segment 2 */
#define CAN_FDCBT_FPSEG2_MASK     (0x7 << CAN_FDCBT_FPSEG2_SHIFT)
#define CAN_FDCBT_FPSEG2(n)       (((n) << CAN_FDCBT_FPSEG2_SHIFT) & CAN_FDCBT_FPSEG2_MASK)
#define CAN_FDCBT_FPSEG1_SHIFT    (5)   /* Bits 5-8: Fast Phase Segment 1 */
#define CAN_FDCBT_FPSEG1_MASK     (0x7 << CAN_FDCBT_FPSEG1_SHIFT)
#define CAN_FDCBT_FPSEG1(n)       (((n) << CAN_FDCBT_FPSEG1_SHIFT) & CAN_FDCBT_FPSEG1_MASK)
#define CAN_FDCBT_FPROPSEG_SHIFT  (10)  /* Bits 10-15: Fast Propagation Segment */
#define CAN_FDCBT_FPROPSEG_MASK   (0x1F << CAN_FDCBT_FPROPSEG_SHIFT)
#define CAN_FDCBT_FPROPSEG(n)     (((n) << CAN_FDCBT_FPROPSEG_SHIFT) & CAN_FDCBT_FPROPSEG_MASK)
#define CAN_FDCBT_FRJW_SHIFT      (16)  /* Bits 16-19: Fast Resync Jump Width */
#define CAN_FDCBT_FRJW_MASK       (0x7 << CAN_FDCBT_FRJW_SHIFT)
#define CAN_FDCBT_FRJW(n)         (((n) << CAN_FDCBT_FRJW_SHIFT) & CAN_FDCBT_FRJW_MASK)
#define CAN_FDCBT_FPRESDIV_SHIFT  (20)  /* Bits 20-30: Fast Prescaler Division Factor */
#define CAN_FDCBT_FPRESDIV_MASK   (0x3FF << CAN_FDCBT_FPRESDIV_SHIFT)
#define CAN_FDCBT_FPRESDIV(n)     (((n) << CAN_FDCBT_FPRESDIV_SHIFT) & CAN_FDCBT_FPRESDIV_MASK)

/* CAN FD CRC (FDCRC) */
#define CAN_FDCRC_FD_TXCRC_SHIFT  (0)   /* Bits 0-21: Extended Transmitted CRC value */
#define CAN_FDCRC_FD_TXCRC_MASK   (0x1FFFFF << CAN_FDCRC_FD_TXCRC_SHIFT)
#define CAN_FDCRC_FD_TXCRC(n)     (((n) << CAN_FDCRC_FD_TXCRC_SHIFT) & CAN_FDCRC_FD_TXCRC_MASK)
#define CAN_FDCRC_FD_MBCRC_SHIFT  (24)  /* Bits 24-31: CRC Message Buffer Number for FD_TXCRC */
#define CAN_FDCRC_FD_MBCRC_MASK   (0x7F << CAN_FDCRC_FD_MBCRC_SHIFT)
#define CAN_FDCRC_FD_MBCRC(n)     (((n) << CAN_FDCRC_FD_MBCRC_SHIFT) & CAN_FDCRC_FD_MBCRC_MASK)

/* Enhanced RX FIFO Control (ERFCR) */
#define CAN_ERFCR_ERFWM_SHIFT  (0)        /* Bits 0-5: Enhanced RX FIFO Watermark */
#define CAN_ERFCR_ERFWM_MASK   (0x1F << CAN_ERFCR_ERFWM_SHIFT)
#define CAN_ERFCR_ERFWM(n)     (((n) << CAN_ERFCR_ERFWM_SHIFT) & CAN_ERFCR_ERFWM_MASK)
#define CAN_ERFCR_NFE_SHIFT    (8)        /* Bits 8-14: Number of Enhanced RX FIFO Filter Elements */
#define CAN_ERFCR_NFE_MASK     (0x3F << CAN_ERFCR_NFE_SHIFT)
#define CAN_ERFCR_NFE(n)       (((n) << CAN_ERFCR_NFE_SHIFT) & CAN_ERFCR_NFE_MASK)
#define CAN_ERFCR_NEXIF_SHIFT  (16)       /* Bits 16-23: Number of Extended ID Filter Elements */
#define CAN_ERFCR_NEXIF_MASK   (0x7F << CAN_ERFCR_NEXIF_SHIFT)
#define CAN_ERFCR_NEXIF(n)     (((n) << CAN_ERFCR_NEXIF_SHIFT) & CAN_ERFCR_NEXIF_MASK)
#define CAN_ERFCR_DMALW_SHIFT  (26)       /* Bits 26-31: DMA Last Word */
#define CAN_ERFCR_DMALW_MASK   (0x1F << CAN_ERFCR_DMALW_SHIFT)
#define CAN_ERFCR_DMALW(n)     (((n) << CAN_ERFCR_DMALW_SHIFT) & CAN_ERFCR_DMALW_MASK)
#define CAN_ERFCR_ERFEN        (1 << 31)  /* Bit 31: Enhanced RX FIFO enable */

/* Enhanced RX FIFO Interrupt Enable (ERFIER) */
#define CAN_ERFIER_ERFDAIE   (1 << 28)  /* Bit 28: Enhanced RX FIFO Data Available Interrupt Enable */
#define CAN_ERFIER_ERFWMIIE  (1 << 29)  /* Bit 29: Enhanced RX FIFO Watermark Indication Interrupt Enable */
#define CAN_ERFIER_ERFOVFIE  (1 << 30)  /* Bit 30: Enhanced RX FIFO Overflow Interrupt Enable */
#define CAN_ERFIER_ERFUFWIE  (1 << 31)  /* Bit 31: Enhanced RX FIFO Underflow Interrupt Enable */

/* Enhanced RX FIFO Status (ERFSR) */
#define CAN_ERFSR_ERFEL_SHIFT  (0)        /* Bits 0-6: Enhanced RX FIFO Elements */
#define CAN_ERFSR_ERFEL_MASK   (0x3F << CAN_ERFSR_ERFEL_SHIFT)
#define CAN_ERFSR_ERFEL(n)     (((n) << CAN_ERFSR_ERFEL_SHIFT) & CAN_ERFSR_ERFEL_MASK)
#define CAN_ERFSR_ERFF         (1 << 16)  /* Bit 16: Enhanced RX FIFO Full Flag */
#define CAN_ERFSR_ERFE         (1 << 17)  /* Bit 17: Enhanced RX FIFO Empty Flag */
#define CAN_ERFSR_ERFCLR       (1 << 27)  /* Bit 27: Enhanced RX FIFO Clear */
#define CAN_ERFSR_ERFDA        (1 << 28)  /* Bit 28: Enhanced RX FIFO Data Available Flag */
#define CAN_ERFSR_ERFWMI       (1 << 29)  /* Bit 29: Enhanced RX FIFO Watermark Indication Flag */
#define CAN_ERFSR_ERFOVF       (1 << 30)  /* Bit 30: Enhanced RX FIFO Overflow Flag */
#define CAN_ERFSR_ERFUFW       (1 << 31)  /* Bit 31: Enhanced RX FIFO Underflow Flag */

/* Receive Individual Mask (RXIMR0) */
#define CAN_RXIMR0_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR0_MI_MASK   (0xFFFFFFFF << CAN_RXIMR0_MI_SHIFT)
#define CAN_RXIMR0_MI(n)     (((n) << CAN_RXIMR0_MI_SHIFT) & CAN_RXIMR0_MI_MASK)

/* Receive Individual Mask (RXIMR1) */
#define CAN_RXIMR1_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR1_MI_MASK   (0xFFFFFFFF << CAN_RXIMR1_MI_SHIFT)
#define CAN_RXIMR1_MI(n)     (((n) << CAN_RXIMR1_MI_SHIFT) & CAN_RXIMR1_MI_MASK)

/* Receive Individual Mask (RXIMR2) */
#define CAN_RXIMR2_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR2_MI_MASK   (0xFFFFFFFF << CAN_RXIMR2_MI_SHIFT)
#define CAN_RXIMR2_MI(n)     (((n) << CAN_RXIMR2_MI_SHIFT) & CAN_RXIMR2_MI_MASK)

/* Receive Individual Mask (RXIMR3) */
#define CAN_RXIMR3_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR3_MI_MASK   (0xFFFFFFFF << CAN_RXIMR3_MI_SHIFT)
#define CAN_RXIMR3_MI(n)     (((n) << CAN_RXIMR3_MI_SHIFT) & CAN_RXIMR3_MI_MASK)

/* Receive Individual Mask (RXIMR4) */
#define CAN_RXIMR4_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR4_MI_MASK   (0xFFFFFFFF << CAN_RXIMR4_MI_SHIFT)
#define CAN_RXIMR4_MI(n)     (((n) << CAN_RXIMR4_MI_SHIFT) & CAN_RXIMR4_MI_MASK)

/* Receive Individual Mask (RXIMR5) */
#define CAN_RXIMR5_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR5_MI_MASK   (0xFFFFFFFF << CAN_RXIMR5_MI_SHIFT)
#define CAN_RXIMR5_MI(n)     (((n) << CAN_RXIMR5_MI_SHIFT) & CAN_RXIMR5_MI_MASK)

/* Receive Individual Mask (RXIMR6) */
#define CAN_RXIMR6_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR6_MI_MASK   (0xFFFFFFFF << CAN_RXIMR6_MI_SHIFT)
#define CAN_RXIMR6_MI(n)     (((n) << CAN_RXIMR6_MI_SHIFT) & CAN_RXIMR6_MI_MASK)

/* Receive Individual Mask (RXIMR7) */
#define CAN_RXIMR7_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR7_MI_MASK   (0xFFFFFFFF << CAN_RXIMR7_MI_SHIFT)
#define CAN_RXIMR7_MI(n)     (((n) << CAN_RXIMR7_MI_SHIFT) & CAN_RXIMR7_MI_MASK)

/* Receive Individual Mask (RXIMR8) */
#define CAN_RXIMR8_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR8_MI_MASK   (0xFFFFFFFF << CAN_RXIMR8_MI_SHIFT)
#define CAN_RXIMR8_MI(n)     (((n) << CAN_RXIMR8_MI_SHIFT) & CAN_RXIMR8_MI_MASK)

/* Receive Individual Mask (RXIMR9) */
#define CAN_RXIMR9_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR9_MI_MASK   (0xFFFFFFFF << CAN_RXIMR9_MI_SHIFT)
#define CAN_RXIMR9_MI(n)     (((n) << CAN_RXIMR9_MI_SHIFT) & CAN_RXIMR9_MI_MASK)

/* Receive Individual Mask (RXIMR10) */
#define CAN_RXIMR10_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR10_MI_MASK   (0xFFFFFFFF << CAN_RXIMR10_MI_SHIFT)
#define CAN_RXIMR10_MI(n)     (((n) << CAN_RXIMR10_MI_SHIFT) & CAN_RXIMR10_MI_MASK)

/* Receive Individual Mask (RXIMR11) */
#define CAN_RXIMR11_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR11_MI_MASK   (0xFFFFFFFF << CAN_RXIMR11_MI_SHIFT)
#define CAN_RXIMR11_MI(n)     (((n) << CAN_RXIMR11_MI_SHIFT) & CAN_RXIMR11_MI_MASK)

/* Receive Individual Mask (RXIMR12) */
#define CAN_RXIMR12_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR12_MI_MASK   (0xFFFFFFFF << CAN_RXIMR12_MI_SHIFT)
#define CAN_RXIMR12_MI(n)     (((n) << CAN_RXIMR12_MI_SHIFT) & CAN_RXIMR12_MI_MASK)

/* Receive Individual Mask (RXIMR13) */
#define CAN_RXIMR13_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR13_MI_MASK   (0xFFFFFFFF << CAN_RXIMR13_MI_SHIFT)
#define CAN_RXIMR13_MI(n)     (((n) << CAN_RXIMR13_MI_SHIFT) & CAN_RXIMR13_MI_MASK)

/* Receive Individual Mask (RXIMR14) */
#define CAN_RXIMR14_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR14_MI_MASK   (0xFFFFFFFF << CAN_RXIMR14_MI_SHIFT)
#define CAN_RXIMR14_MI(n)     (((n) << CAN_RXIMR14_MI_SHIFT) & CAN_RXIMR14_MI_MASK)

/* Receive Individual Mask (RXIMR15) */
#define CAN_RXIMR15_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR15_MI_MASK   (0xFFFFFFFF << CAN_RXIMR15_MI_SHIFT)
#define CAN_RXIMR15_MI(n)     (((n) << CAN_RXIMR15_MI_SHIFT) & CAN_RXIMR15_MI_MASK)

/* Receive Individual Mask (RXIMR16) */
#define CAN_RXIMR16_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR16_MI_MASK   (0xFFFFFFFF << CAN_RXIMR16_MI_SHIFT)
#define CAN_RXIMR16_MI(n)     (((n) << CAN_RXIMR16_MI_SHIFT) & CAN_RXIMR16_MI_MASK)

/* Receive Individual Mask (RXIMR17) */
#define CAN_RXIMR17_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR17_MI_MASK   (0xFFFFFFFF << CAN_RXIMR17_MI_SHIFT)
#define CAN_RXIMR17_MI(n)     (((n) << CAN_RXIMR17_MI_SHIFT) & CAN_RXIMR17_MI_MASK)

/* Receive Individual Mask (RXIMR18) */
#define CAN_RXIMR18_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR18_MI_MASK   (0xFFFFFFFF << CAN_RXIMR18_MI_SHIFT)
#define CAN_RXIMR18_MI(n)     (((n) << CAN_RXIMR18_MI_SHIFT) & CAN_RXIMR18_MI_MASK)

/* Receive Individual Mask (RXIMR19) */
#define CAN_RXIMR19_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR19_MI_MASK   (0xFFFFFFFF << CAN_RXIMR19_MI_SHIFT)
#define CAN_RXIMR19_MI(n)     (((n) << CAN_RXIMR19_MI_SHIFT) & CAN_RXIMR19_MI_MASK)

/* Receive Individual Mask (RXIMR20) */
#define CAN_RXIMR20_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR20_MI_MASK   (0xFFFFFFFF << CAN_RXIMR20_MI_SHIFT)
#define CAN_RXIMR20_MI(n)     (((n) << CAN_RXIMR20_MI_SHIFT) & CAN_RXIMR20_MI_MASK)

/* Receive Individual Mask (RXIMR21) */
#define CAN_RXIMR21_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR21_MI_MASK   (0xFFFFFFFF << CAN_RXIMR21_MI_SHIFT)
#define CAN_RXIMR21_MI(n)     (((n) << CAN_RXIMR21_MI_SHIFT) & CAN_RXIMR21_MI_MASK)

/* Receive Individual Mask (RXIMR22) */
#define CAN_RXIMR22_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR22_MI_MASK   (0xFFFFFFFF << CAN_RXIMR22_MI_SHIFT)
#define CAN_RXIMR22_MI(n)     (((n) << CAN_RXIMR22_MI_SHIFT) & CAN_RXIMR22_MI_MASK)

/* Receive Individual Mask (RXIMR23) */
#define CAN_RXIMR23_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR23_MI_MASK   (0xFFFFFFFF << CAN_RXIMR23_MI_SHIFT)
#define CAN_RXIMR23_MI(n)     (((n) << CAN_RXIMR23_MI_SHIFT) & CAN_RXIMR23_MI_MASK)

/* Receive Individual Mask (RXIMR24) */
#define CAN_RXIMR24_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR24_MI_MASK   (0xFFFFFFFF << CAN_RXIMR24_MI_SHIFT)
#define CAN_RXIMR24_MI(n)     (((n) << CAN_RXIMR24_MI_SHIFT) & CAN_RXIMR24_MI_MASK)

/* Receive Individual Mask (RXIMR25) */
#define CAN_RXIMR25_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR25_MI_MASK   (0xFFFFFFFF << CAN_RXIMR25_MI_SHIFT)
#define CAN_RXIMR25_MI(n)     (((n) << CAN_RXIMR25_MI_SHIFT) & CAN_RXIMR25_MI_MASK)

/* Receive Individual Mask (RXIMR26) */
#define CAN_RXIMR26_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR26_MI_MASK   (0xFFFFFFFF << CAN_RXIMR26_MI_SHIFT)
#define CAN_RXIMR26_MI(n)     (((n) << CAN_RXIMR26_MI_SHIFT) & CAN_RXIMR26_MI_MASK)

/* Receive Individual Mask (RXIMR27) */
#define CAN_RXIMR27_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR27_MI_MASK   (0xFFFFFFFF << CAN_RXIMR27_MI_SHIFT)
#define CAN_RXIMR27_MI(n)     (((n) << CAN_RXIMR27_MI_SHIFT) & CAN_RXIMR27_MI_MASK)

/* Receive Individual Mask (RXIMR28) */
#define CAN_RXIMR28_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR28_MI_MASK   (0xFFFFFFFF << CAN_RXIMR28_MI_SHIFT)
#define CAN_RXIMR28_MI(n)     (((n) << CAN_RXIMR28_MI_SHIFT) & CAN_RXIMR28_MI_MASK)

/* Receive Individual Mask (RXIMR29) */
#define CAN_RXIMR29_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR29_MI_MASK   (0xFFFFFFFF << CAN_RXIMR29_MI_SHIFT)
#define CAN_RXIMR29_MI(n)     (((n) << CAN_RXIMR29_MI_SHIFT) & CAN_RXIMR29_MI_MASK)

/* Receive Individual Mask (RXIMR30) */
#define CAN_RXIMR30_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR30_MI_MASK   (0xFFFFFFFF << CAN_RXIMR30_MI_SHIFT)
#define CAN_RXIMR30_MI(n)     (((n) << CAN_RXIMR30_MI_SHIFT) & CAN_RXIMR30_MI_MASK)

/* Receive Individual Mask (RXIMR31) */
#define CAN_RXIMR31_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR31_MI_MASK   (0xFFFFFFFF << CAN_RXIMR31_MI_SHIFT)
#define CAN_RXIMR31_MI(n)     (((n) << CAN_RXIMR31_MI_SHIFT) & CAN_RXIMR31_MI_MASK)

/* Receive Individual Mask (RXIMR32) */
#define CAN_RXIMR32_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR32_MI_MASK   (0xFFFFFFFF << CAN_RXIMR32_MI_SHIFT)
#define CAN_RXIMR32_MI(n)     (((n) << CAN_RXIMR32_MI_SHIFT) & CAN_RXIMR32_MI_MASK)

/* Receive Individual Mask (RXIMR33) */
#define CAN_RXIMR33_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR33_MI_MASK   (0xFFFFFFFF << CAN_RXIMR33_MI_SHIFT)
#define CAN_RXIMR33_MI(n)     (((n) << CAN_RXIMR33_MI_SHIFT) & CAN_RXIMR33_MI_MASK)

/* Receive Individual Mask (RXIMR34) */
#define CAN_RXIMR34_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR34_MI_MASK   (0xFFFFFFFF << CAN_RXIMR34_MI_SHIFT)
#define CAN_RXIMR34_MI(n)     (((n) << CAN_RXIMR34_MI_SHIFT) & CAN_RXIMR34_MI_MASK)

/* Receive Individual Mask (RXIMR35) */
#define CAN_RXIMR35_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR35_MI_MASK   (0xFFFFFFFF << CAN_RXIMR35_MI_SHIFT)
#define CAN_RXIMR35_MI(n)     (((n) << CAN_RXIMR35_MI_SHIFT) & CAN_RXIMR35_MI_MASK)

/* Receive Individual Mask (RXIMR36) */
#define CAN_RXIMR36_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR36_MI_MASK   (0xFFFFFFFF << CAN_RXIMR36_MI_SHIFT)
#define CAN_RXIMR36_MI(n)     (((n) << CAN_RXIMR36_MI_SHIFT) & CAN_RXIMR36_MI_MASK)

/* Receive Individual Mask (RXIMR37) */
#define CAN_RXIMR37_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR37_MI_MASK   (0xFFFFFFFF << CAN_RXIMR37_MI_SHIFT)
#define CAN_RXIMR37_MI(n)     (((n) << CAN_RXIMR37_MI_SHIFT) & CAN_RXIMR37_MI_MASK)

/* Receive Individual Mask (RXIMR38) */
#define CAN_RXIMR38_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR38_MI_MASK   (0xFFFFFFFF << CAN_RXIMR38_MI_SHIFT)
#define CAN_RXIMR38_MI(n)     (((n) << CAN_RXIMR38_MI_SHIFT) & CAN_RXIMR38_MI_MASK)

/* Receive Individual Mask (RXIMR39) */
#define CAN_RXIMR39_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR39_MI_MASK   (0xFFFFFFFF << CAN_RXIMR39_MI_SHIFT)
#define CAN_RXIMR39_MI(n)     (((n) << CAN_RXIMR39_MI_SHIFT) & CAN_RXIMR39_MI_MASK)

/* Receive Individual Mask (RXIMR40) */
#define CAN_RXIMR40_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR40_MI_MASK   (0xFFFFFFFF << CAN_RXIMR40_MI_SHIFT)
#define CAN_RXIMR40_MI(n)     (((n) << CAN_RXIMR40_MI_SHIFT) & CAN_RXIMR40_MI_MASK)

/* Receive Individual Mask (RXIMR41) */
#define CAN_RXIMR41_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR41_MI_MASK   (0xFFFFFFFF << CAN_RXIMR41_MI_SHIFT)
#define CAN_RXIMR41_MI(n)     (((n) << CAN_RXIMR41_MI_SHIFT) & CAN_RXIMR41_MI_MASK)

/* Receive Individual Mask (RXIMR42) */
#define CAN_RXIMR42_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR42_MI_MASK   (0xFFFFFFFF << CAN_RXIMR42_MI_SHIFT)
#define CAN_RXIMR42_MI(n)     (((n) << CAN_RXIMR42_MI_SHIFT) & CAN_RXIMR42_MI_MASK)

/* Receive Individual Mask (RXIMR43) */
#define CAN_RXIMR43_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR43_MI_MASK   (0xFFFFFFFF << CAN_RXIMR43_MI_SHIFT)
#define CAN_RXIMR43_MI(n)     (((n) << CAN_RXIMR43_MI_SHIFT) & CAN_RXIMR43_MI_MASK)

/* Receive Individual Mask (RXIMR44) */
#define CAN_RXIMR44_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR44_MI_MASK   (0xFFFFFFFF << CAN_RXIMR44_MI_SHIFT)
#define CAN_RXIMR44_MI(n)     (((n) << CAN_RXIMR44_MI_SHIFT) & CAN_RXIMR44_MI_MASK)

/* Receive Individual Mask (RXIMR45) */
#define CAN_RXIMR45_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR45_MI_MASK   (0xFFFFFFFF << CAN_RXIMR45_MI_SHIFT)
#define CAN_RXIMR45_MI(n)     (((n) << CAN_RXIMR45_MI_SHIFT) & CAN_RXIMR45_MI_MASK)

/* Receive Individual Mask (RXIMR46) */
#define CAN_RXIMR46_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR46_MI_MASK   (0xFFFFFFFF << CAN_RXIMR46_MI_SHIFT)
#define CAN_RXIMR46_MI(n)     (((n) << CAN_RXIMR46_MI_SHIFT) & CAN_RXIMR46_MI_MASK)

/* Receive Individual Mask (RXIMR47) */
#define CAN_RXIMR47_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR47_MI_MASK   (0xFFFFFFFF << CAN_RXIMR47_MI_SHIFT)
#define CAN_RXIMR47_MI(n)     (((n) << CAN_RXIMR47_MI_SHIFT) & CAN_RXIMR47_MI_MASK)

/* Receive Individual Mask (RXIMR48) */
#define CAN_RXIMR48_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR48_MI_MASK   (0xFFFFFFFF << CAN_RXIMR48_MI_SHIFT)
#define CAN_RXIMR48_MI(n)     (((n) << CAN_RXIMR48_MI_SHIFT) & CAN_RXIMR48_MI_MASK)

/* Receive Individual Mask (RXIMR49) */
#define CAN_RXIMR49_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR49_MI_MASK   (0xFFFFFFFF << CAN_RXIMR49_MI_SHIFT)
#define CAN_RXIMR49_MI(n)     (((n) << CAN_RXIMR49_MI_SHIFT) & CAN_RXIMR49_MI_MASK)

/* Receive Individual Mask (RXIMR50) */
#define CAN_RXIMR50_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR50_MI_MASK   (0xFFFFFFFF << CAN_RXIMR50_MI_SHIFT)
#define CAN_RXIMR50_MI(n)     (((n) << CAN_RXIMR50_MI_SHIFT) & CAN_RXIMR50_MI_MASK)

/* Receive Individual Mask (RXIMR51) */
#define CAN_RXIMR51_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR51_MI_MASK   (0xFFFFFFFF << CAN_RXIMR51_MI_SHIFT)
#define CAN_RXIMR51_MI(n)     (((n) << CAN_RXIMR51_MI_SHIFT) & CAN_RXIMR51_MI_MASK)

/* Receive Individual Mask (RXIMR52) */
#define CAN_RXIMR52_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR52_MI_MASK   (0xFFFFFFFF << CAN_RXIMR52_MI_SHIFT)
#define CAN_RXIMR52_MI(n)     (((n) << CAN_RXIMR52_MI_SHIFT) & CAN_RXIMR52_MI_MASK)

/* Receive Individual Mask (RXIMR53) */
#define CAN_RXIMR53_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR53_MI_MASK   (0xFFFFFFFF << CAN_RXIMR53_MI_SHIFT)
#define CAN_RXIMR53_MI(n)     (((n) << CAN_RXIMR53_MI_SHIFT) & CAN_RXIMR53_MI_MASK)

/* Receive Individual Mask (RXIMR54) */
#define CAN_RXIMR54_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR54_MI_MASK   (0xFFFFFFFF << CAN_RXIMR54_MI_SHIFT)
#define CAN_RXIMR54_MI(n)     (((n) << CAN_RXIMR54_MI_SHIFT) & CAN_RXIMR54_MI_MASK)

/* Receive Individual Mask (RXIMR55) */
#define CAN_RXIMR55_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR55_MI_MASK   (0xFFFFFFFF << CAN_RXIMR55_MI_SHIFT)
#define CAN_RXIMR55_MI(n)     (((n) << CAN_RXIMR55_MI_SHIFT) & CAN_RXIMR55_MI_MASK)

/* Receive Individual Mask (RXIMR56) */
#define CAN_RXIMR56_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR56_MI_MASK   (0xFFFFFFFF << CAN_RXIMR56_MI_SHIFT)
#define CAN_RXIMR56_MI(n)     (((n) << CAN_RXIMR56_MI_SHIFT) & CAN_RXIMR56_MI_MASK)

/* Receive Individual Mask (RXIMR57) */
#define CAN_RXIMR57_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR57_MI_MASK   (0xFFFFFFFF << CAN_RXIMR57_MI_SHIFT)
#define CAN_RXIMR57_MI(n)     (((n) << CAN_RXIMR57_MI_SHIFT) & CAN_RXIMR57_MI_MASK)

/* Receive Individual Mask (RXIMR58) */
#define CAN_RXIMR58_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR58_MI_MASK   (0xFFFFFFFF << CAN_RXIMR58_MI_SHIFT)
#define CAN_RXIMR58_MI(n)     (((n) << CAN_RXIMR58_MI_SHIFT) & CAN_RXIMR58_MI_MASK)

/* Receive Individual Mask (RXIMR59) */
#define CAN_RXIMR59_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR59_MI_MASK   (0xFFFFFFFF << CAN_RXIMR59_MI_SHIFT)
#define CAN_RXIMR59_MI(n)     (((n) << CAN_RXIMR59_MI_SHIFT) & CAN_RXIMR59_MI_MASK)

/* Receive Individual Mask (RXIMR60) */
#define CAN_RXIMR60_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR60_MI_MASK   (0xFFFFFFFF << CAN_RXIMR60_MI_SHIFT)
#define CAN_RXIMR60_MI(n)     (((n) << CAN_RXIMR60_MI_SHIFT) & CAN_RXIMR60_MI_MASK)

/* Receive Individual Mask (RXIMR61) */
#define CAN_RXIMR61_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR61_MI_MASK   (0xFFFFFFFF << CAN_RXIMR61_MI_SHIFT)
#define CAN_RXIMR61_MI(n)     (((n) << CAN_RXIMR61_MI_SHIFT) & CAN_RXIMR61_MI_MASK)

/* Receive Individual Mask (RXIMR62) */
#define CAN_RXIMR62_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR62_MI_MASK   (0xFFFFFFFF << CAN_RXIMR62_MI_SHIFT)
#define CAN_RXIMR62_MI(n)     (((n) << CAN_RXIMR62_MI_SHIFT) & CAN_RXIMR62_MI_MASK)

/* Receive Individual Mask (RXIMR63) */
#define CAN_RXIMR63_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR63_MI_MASK   (0xFFFFFFFF << CAN_RXIMR63_MI_SHIFT)
#define CAN_RXIMR63_MI(n)     (((n) << CAN_RXIMR63_MI_SHIFT) & CAN_RXIMR63_MI_MASK)

/* Receive Individual Mask (RXIMR64) */
#define CAN_RXIMR64_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR64_MI_MASK   (0xFFFFFFFF << CAN_RXIMR64_MI_SHIFT)
#define CAN_RXIMR64_MI(n)     (((n) << CAN_RXIMR64_MI_SHIFT) & CAN_RXIMR64_MI_MASK)

/* Receive Individual Mask (RXIMR65) */
#define CAN_RXIMR65_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR65_MI_MASK   (0xFFFFFFFF << CAN_RXIMR65_MI_SHIFT)
#define CAN_RXIMR65_MI(n)     (((n) << CAN_RXIMR65_MI_SHIFT) & CAN_RXIMR65_MI_MASK)

/* Receive Individual Mask (RXIMR66) */
#define CAN_RXIMR66_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR66_MI_MASK   (0xFFFFFFFF << CAN_RXIMR66_MI_SHIFT)
#define CAN_RXIMR66_MI(n)     (((n) << CAN_RXIMR66_MI_SHIFT) & CAN_RXIMR66_MI_MASK)

/* Receive Individual Mask (RXIMR67) */
#define CAN_RXIMR67_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR67_MI_MASK   (0xFFFFFFFF << CAN_RXIMR67_MI_SHIFT)
#define CAN_RXIMR67_MI(n)     (((n) << CAN_RXIMR67_MI_SHIFT) & CAN_RXIMR67_MI_MASK)

/* Receive Individual Mask (RXIMR68) */
#define CAN_RXIMR68_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR68_MI_MASK   (0xFFFFFFFF << CAN_RXIMR68_MI_SHIFT)
#define CAN_RXIMR68_MI(n)     (((n) << CAN_RXIMR68_MI_SHIFT) & CAN_RXIMR68_MI_MASK)

/* Receive Individual Mask (RXIMR69) */
#define CAN_RXIMR69_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR69_MI_MASK   (0xFFFFFFFF << CAN_RXIMR69_MI_SHIFT)
#define CAN_RXIMR69_MI(n)     (((n) << CAN_RXIMR69_MI_SHIFT) & CAN_RXIMR69_MI_MASK)

/* Receive Individual Mask (RXIMR70) */
#define CAN_RXIMR70_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR70_MI_MASK   (0xFFFFFFFF << CAN_RXIMR70_MI_SHIFT)
#define CAN_RXIMR70_MI(n)     (((n) << CAN_RXIMR70_MI_SHIFT) & CAN_RXIMR70_MI_MASK)

/* Receive Individual Mask (RXIMR71) */
#define CAN_RXIMR71_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR71_MI_MASK   (0xFFFFFFFF << CAN_RXIMR71_MI_SHIFT)
#define CAN_RXIMR71_MI(n)     (((n) << CAN_RXIMR71_MI_SHIFT) & CAN_RXIMR71_MI_MASK)

/* Receive Individual Mask (RXIMR72) */
#define CAN_RXIMR72_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR72_MI_MASK   (0xFFFFFFFF << CAN_RXIMR72_MI_SHIFT)
#define CAN_RXIMR72_MI(n)     (((n) << CAN_RXIMR72_MI_SHIFT) & CAN_RXIMR72_MI_MASK)

/* Receive Individual Mask (RXIMR73) */
#define CAN_RXIMR73_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR73_MI_MASK   (0xFFFFFFFF << CAN_RXIMR73_MI_SHIFT)
#define CAN_RXIMR73_MI(n)     (((n) << CAN_RXIMR73_MI_SHIFT) & CAN_RXIMR73_MI_MASK)

/* Receive Individual Mask (RXIMR74) */
#define CAN_RXIMR74_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR74_MI_MASK   (0xFFFFFFFF << CAN_RXIMR74_MI_SHIFT)
#define CAN_RXIMR74_MI(n)     (((n) << CAN_RXIMR74_MI_SHIFT) & CAN_RXIMR74_MI_MASK)

/* Receive Individual Mask (RXIMR75) */
#define CAN_RXIMR75_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR75_MI_MASK   (0xFFFFFFFF << CAN_RXIMR75_MI_SHIFT)
#define CAN_RXIMR75_MI(n)     (((n) << CAN_RXIMR75_MI_SHIFT) & CAN_RXIMR75_MI_MASK)

/* Receive Individual Mask (RXIMR76) */
#define CAN_RXIMR76_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR76_MI_MASK   (0xFFFFFFFF << CAN_RXIMR76_MI_SHIFT)
#define CAN_RXIMR76_MI(n)     (((n) << CAN_RXIMR76_MI_SHIFT) & CAN_RXIMR76_MI_MASK)

/* Receive Individual Mask (RXIMR77) */
#define CAN_RXIMR77_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR77_MI_MASK   (0xFFFFFFFF << CAN_RXIMR77_MI_SHIFT)
#define CAN_RXIMR77_MI(n)     (((n) << CAN_RXIMR77_MI_SHIFT) & CAN_RXIMR77_MI_MASK)

/* Receive Individual Mask (RXIMR78) */
#define CAN_RXIMR78_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR78_MI_MASK   (0xFFFFFFFF << CAN_RXIMR78_MI_SHIFT)
#define CAN_RXIMR78_MI(n)     (((n) << CAN_RXIMR78_MI_SHIFT) & CAN_RXIMR78_MI_MASK)

/* Receive Individual Mask (RXIMR79) */
#define CAN_RXIMR79_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR79_MI_MASK   (0xFFFFFFFF << CAN_RXIMR79_MI_SHIFT)
#define CAN_RXIMR79_MI(n)     (((n) << CAN_RXIMR79_MI_SHIFT) & CAN_RXIMR79_MI_MASK)

/* Receive Individual Mask (RXIMR80) */
#define CAN_RXIMR80_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR80_MI_MASK   (0xFFFFFFFF << CAN_RXIMR80_MI_SHIFT)
#define CAN_RXIMR80_MI(n)     (((n) << CAN_RXIMR80_MI_SHIFT) & CAN_RXIMR80_MI_MASK)

/* Receive Individual Mask (RXIMR81) */
#define CAN_RXIMR81_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR81_MI_MASK   (0xFFFFFFFF << CAN_RXIMR81_MI_SHIFT)
#define CAN_RXIMR81_MI(n)     (((n) << CAN_RXIMR81_MI_SHIFT) & CAN_RXIMR81_MI_MASK)

/* Receive Individual Mask (RXIMR82) */
#define CAN_RXIMR82_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR82_MI_MASK   (0xFFFFFFFF << CAN_RXIMR82_MI_SHIFT)
#define CAN_RXIMR82_MI(n)     (((n) << CAN_RXIMR82_MI_SHIFT) & CAN_RXIMR82_MI_MASK)

/* Receive Individual Mask (RXIMR83) */
#define CAN_RXIMR83_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR83_MI_MASK   (0xFFFFFFFF << CAN_RXIMR83_MI_SHIFT)
#define CAN_RXIMR83_MI(n)     (((n) << CAN_RXIMR83_MI_SHIFT) & CAN_RXIMR83_MI_MASK)

/* Receive Individual Mask (RXIMR84) */
#define CAN_RXIMR84_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR84_MI_MASK   (0xFFFFFFFF << CAN_RXIMR84_MI_SHIFT)
#define CAN_RXIMR84_MI(n)     (((n) << CAN_RXIMR84_MI_SHIFT) & CAN_RXIMR84_MI_MASK)

/* Receive Individual Mask (RXIMR85) */
#define CAN_RXIMR85_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR85_MI_MASK   (0xFFFFFFFF << CAN_RXIMR85_MI_SHIFT)
#define CAN_RXIMR85_MI(n)     (((n) << CAN_RXIMR85_MI_SHIFT) & CAN_RXIMR85_MI_MASK)

/* Receive Individual Mask (RXIMR86) */
#define CAN_RXIMR86_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR86_MI_MASK   (0xFFFFFFFF << CAN_RXIMR86_MI_SHIFT)
#define CAN_RXIMR86_MI(n)     (((n) << CAN_RXIMR86_MI_SHIFT) & CAN_RXIMR86_MI_MASK)

/* Receive Individual Mask (RXIMR87) */
#define CAN_RXIMR87_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR87_MI_MASK   (0xFFFFFFFF << CAN_RXIMR87_MI_SHIFT)
#define CAN_RXIMR87_MI(n)     (((n) << CAN_RXIMR87_MI_SHIFT) & CAN_RXIMR87_MI_MASK)

/* Receive Individual Mask (RXIMR88) */
#define CAN_RXIMR88_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR88_MI_MASK   (0xFFFFFFFF << CAN_RXIMR88_MI_SHIFT)
#define CAN_RXIMR88_MI(n)     (((n) << CAN_RXIMR88_MI_SHIFT) & CAN_RXIMR88_MI_MASK)

/* Receive Individual Mask (RXIMR89) */
#define CAN_RXIMR89_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR89_MI_MASK   (0xFFFFFFFF << CAN_RXIMR89_MI_SHIFT)
#define CAN_RXIMR89_MI(n)     (((n) << CAN_RXIMR89_MI_SHIFT) & CAN_RXIMR89_MI_MASK)

/* Receive Individual Mask (RXIMR90) */
#define CAN_RXIMR90_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR90_MI_MASK   (0xFFFFFFFF << CAN_RXIMR90_MI_SHIFT)
#define CAN_RXIMR90_MI(n)     (((n) << CAN_RXIMR90_MI_SHIFT) & CAN_RXIMR90_MI_MASK)

/* Receive Individual Mask (RXIMR91) */
#define CAN_RXIMR91_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR91_MI_MASK   (0xFFFFFFFF << CAN_RXIMR91_MI_SHIFT)
#define CAN_RXIMR91_MI(n)     (((n) << CAN_RXIMR91_MI_SHIFT) & CAN_RXIMR91_MI_MASK)

/* Receive Individual Mask (RXIMR92) */
#define CAN_RXIMR92_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR92_MI_MASK   (0xFFFFFFFF << CAN_RXIMR92_MI_SHIFT)
#define CAN_RXIMR92_MI(n)     (((n) << CAN_RXIMR92_MI_SHIFT) & CAN_RXIMR92_MI_MASK)

/* Receive Individual Mask (RXIMR93) */
#define CAN_RXIMR93_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR93_MI_MASK   (0xFFFFFFFF << CAN_RXIMR93_MI_SHIFT)
#define CAN_RXIMR93_MI(n)     (((n) << CAN_RXIMR93_MI_SHIFT) & CAN_RXIMR93_MI_MASK)

/* Receive Individual Mask (RXIMR94) */
#define CAN_RXIMR94_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR94_MI_MASK   (0xFFFFFFFF << CAN_RXIMR94_MI_SHIFT)
#define CAN_RXIMR94_MI(n)     (((n) << CAN_RXIMR94_MI_SHIFT) & CAN_RXIMR94_MI_MASK)

/* Receive Individual Mask (RXIMR95) */
#define CAN_RXIMR95_MI_SHIFT  (0)  /* Bits 0-32: Individual Mask Bits */
#define CAN_RXIMR95_MI_MASK   (0xFFFFFFFF << CAN_RXIMR95_MI_SHIFT)
#define CAN_RXIMR95_MI(n)     (((n) << CAN_RXIMR95_MI_SHIFT) & CAN_RXIMR95_MI_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP0) */
#define CAN_HR_TIME_STAMP0_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP0_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP0_TS_SHIFT)
#define CAN_HR_TIME_STAMP0_TS(n)     (((n) << CAN_HR_TIME_STAMP0_TS_SHIFT) & CAN_HR_TIME_STAMP0_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP1) */
#define CAN_HR_TIME_STAMP1_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP1_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP1_TS_SHIFT)
#define CAN_HR_TIME_STAMP1_TS(n)     (((n) << CAN_HR_TIME_STAMP1_TS_SHIFT) & CAN_HR_TIME_STAMP1_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP2) */
#define CAN_HR_TIME_STAMP2_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP2_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP2_TS_SHIFT)
#define CAN_HR_TIME_STAMP2_TS(n)     (((n) << CAN_HR_TIME_STAMP2_TS_SHIFT) & CAN_HR_TIME_STAMP2_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP3) */
#define CAN_HR_TIME_STAMP3_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP3_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP3_TS_SHIFT)
#define CAN_HR_TIME_STAMP3_TS(n)     (((n) << CAN_HR_TIME_STAMP3_TS_SHIFT) & CAN_HR_TIME_STAMP3_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP4) */
#define CAN_HR_TIME_STAMP4_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP4_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP4_TS_SHIFT)
#define CAN_HR_TIME_STAMP4_TS(n)     (((n) << CAN_HR_TIME_STAMP4_TS_SHIFT) & CAN_HR_TIME_STAMP4_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP5) */
#define CAN_HR_TIME_STAMP5_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP5_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP5_TS_SHIFT)
#define CAN_HR_TIME_STAMP5_TS(n)     (((n) << CAN_HR_TIME_STAMP5_TS_SHIFT) & CAN_HR_TIME_STAMP5_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP6) */
#define CAN_HR_TIME_STAMP6_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP6_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP6_TS_SHIFT)
#define CAN_HR_TIME_STAMP6_TS(n)     (((n) << CAN_HR_TIME_STAMP6_TS_SHIFT) & CAN_HR_TIME_STAMP6_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP7) */
#define CAN_HR_TIME_STAMP7_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP7_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP7_TS_SHIFT)
#define CAN_HR_TIME_STAMP7_TS(n)     (((n) << CAN_HR_TIME_STAMP7_TS_SHIFT) & CAN_HR_TIME_STAMP7_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP8) */
#define CAN_HR_TIME_STAMP8_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP8_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP8_TS_SHIFT)
#define CAN_HR_TIME_STAMP8_TS(n)     (((n) << CAN_HR_TIME_STAMP8_TS_SHIFT) & CAN_HR_TIME_STAMP8_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP9) */
#define CAN_HR_TIME_STAMP9_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP9_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP9_TS_SHIFT)
#define CAN_HR_TIME_STAMP9_TS(n)     (((n) << CAN_HR_TIME_STAMP9_TS_SHIFT) & CAN_HR_TIME_STAMP9_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP10) */
#define CAN_HR_TIME_STAMP10_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP10_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP10_TS_SHIFT)
#define CAN_HR_TIME_STAMP10_TS(n)     (((n) << CAN_HR_TIME_STAMP10_TS_SHIFT) & CAN_HR_TIME_STAMP10_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP11) */
#define CAN_HR_TIME_STAMP11_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP11_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP11_TS_SHIFT)
#define CAN_HR_TIME_STAMP11_TS(n)     (((n) << CAN_HR_TIME_STAMP11_TS_SHIFT) & CAN_HR_TIME_STAMP11_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP12) */
#define CAN_HR_TIME_STAMP12_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP12_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP12_TS_SHIFT)
#define CAN_HR_TIME_STAMP12_TS(n)     (((n) << CAN_HR_TIME_STAMP12_TS_SHIFT) & CAN_HR_TIME_STAMP12_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP13) */
#define CAN_HR_TIME_STAMP13_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP13_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP13_TS_SHIFT)
#define CAN_HR_TIME_STAMP13_TS(n)     (((n) << CAN_HR_TIME_STAMP13_TS_SHIFT) & CAN_HR_TIME_STAMP13_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP14) */
#define CAN_HR_TIME_STAMP14_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP14_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP14_TS_SHIFT)
#define CAN_HR_TIME_STAMP14_TS(n)     (((n) << CAN_HR_TIME_STAMP14_TS_SHIFT) & CAN_HR_TIME_STAMP14_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP15) */
#define CAN_HR_TIME_STAMP15_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP15_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP15_TS_SHIFT)
#define CAN_HR_TIME_STAMP15_TS(n)     (((n) << CAN_HR_TIME_STAMP15_TS_SHIFT) & CAN_HR_TIME_STAMP15_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP16) */
#define CAN_HR_TIME_STAMP16_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP16_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP16_TS_SHIFT)
#define CAN_HR_TIME_STAMP16_TS(n)     (((n) << CAN_HR_TIME_STAMP16_TS_SHIFT) & CAN_HR_TIME_STAMP16_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP17) */
#define CAN_HR_TIME_STAMP17_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP17_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP17_TS_SHIFT)
#define CAN_HR_TIME_STAMP17_TS(n)     (((n) << CAN_HR_TIME_STAMP17_TS_SHIFT) & CAN_HR_TIME_STAMP17_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP18) */
#define CAN_HR_TIME_STAMP18_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP18_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP18_TS_SHIFT)
#define CAN_HR_TIME_STAMP18_TS(n)     (((n) << CAN_HR_TIME_STAMP18_TS_SHIFT) & CAN_HR_TIME_STAMP18_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP19) */
#define CAN_HR_TIME_STAMP19_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP19_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP19_TS_SHIFT)
#define CAN_HR_TIME_STAMP19_TS(n)     (((n) << CAN_HR_TIME_STAMP19_TS_SHIFT) & CAN_HR_TIME_STAMP19_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP20) */
#define CAN_HR_TIME_STAMP20_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP20_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP20_TS_SHIFT)
#define CAN_HR_TIME_STAMP20_TS(n)     (((n) << CAN_HR_TIME_STAMP20_TS_SHIFT) & CAN_HR_TIME_STAMP20_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP21) */
#define CAN_HR_TIME_STAMP21_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP21_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP21_TS_SHIFT)
#define CAN_HR_TIME_STAMP21_TS(n)     (((n) << CAN_HR_TIME_STAMP21_TS_SHIFT) & CAN_HR_TIME_STAMP21_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP22) */
#define CAN_HR_TIME_STAMP22_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP22_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP22_TS_SHIFT)
#define CAN_HR_TIME_STAMP22_TS(n)     (((n) << CAN_HR_TIME_STAMP22_TS_SHIFT) & CAN_HR_TIME_STAMP22_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP23) */
#define CAN_HR_TIME_STAMP23_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP23_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP23_TS_SHIFT)
#define CAN_HR_TIME_STAMP23_TS(n)     (((n) << CAN_HR_TIME_STAMP23_TS_SHIFT) & CAN_HR_TIME_STAMP23_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP24) */
#define CAN_HR_TIME_STAMP24_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP24_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP24_TS_SHIFT)
#define CAN_HR_TIME_STAMP24_TS(n)     (((n) << CAN_HR_TIME_STAMP24_TS_SHIFT) & CAN_HR_TIME_STAMP24_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP25) */
#define CAN_HR_TIME_STAMP25_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP25_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP25_TS_SHIFT)
#define CAN_HR_TIME_STAMP25_TS(n)     (((n) << CAN_HR_TIME_STAMP25_TS_SHIFT) & CAN_HR_TIME_STAMP25_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP26) */
#define CAN_HR_TIME_STAMP26_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP26_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP26_TS_SHIFT)
#define CAN_HR_TIME_STAMP26_TS(n)     (((n) << CAN_HR_TIME_STAMP26_TS_SHIFT) & CAN_HR_TIME_STAMP26_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP27) */
#define CAN_HR_TIME_STAMP27_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP27_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP27_TS_SHIFT)
#define CAN_HR_TIME_STAMP27_TS(n)     (((n) << CAN_HR_TIME_STAMP27_TS_SHIFT) & CAN_HR_TIME_STAMP27_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP28) */
#define CAN_HR_TIME_STAMP28_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP28_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP28_TS_SHIFT)
#define CAN_HR_TIME_STAMP28_TS(n)     (((n) << CAN_HR_TIME_STAMP28_TS_SHIFT) & CAN_HR_TIME_STAMP28_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP29) */
#define CAN_HR_TIME_STAMP29_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP29_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP29_TS_SHIFT)
#define CAN_HR_TIME_STAMP29_TS(n)     (((n) << CAN_HR_TIME_STAMP29_TS_SHIFT) & CAN_HR_TIME_STAMP29_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP30) */
#define CAN_HR_TIME_STAMP30_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP30_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP30_TS_SHIFT)
#define CAN_HR_TIME_STAMP30_TS(n)     (((n) << CAN_HR_TIME_STAMP30_TS_SHIFT) & CAN_HR_TIME_STAMP30_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP31) */
#define CAN_HR_TIME_STAMP31_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP31_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP31_TS_SHIFT)
#define CAN_HR_TIME_STAMP31_TS(n)     (((n) << CAN_HR_TIME_STAMP31_TS_SHIFT) & CAN_HR_TIME_STAMP31_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP32) */
#define CAN_HR_TIME_STAMP32_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP32_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP32_TS_SHIFT)
#define CAN_HR_TIME_STAMP32_TS(n)     (((n) << CAN_HR_TIME_STAMP32_TS_SHIFT) & CAN_HR_TIME_STAMP32_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP33) */
#define CAN_HR_TIME_STAMP33_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP33_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP33_TS_SHIFT)
#define CAN_HR_TIME_STAMP33_TS(n)     (((n) << CAN_HR_TIME_STAMP33_TS_SHIFT) & CAN_HR_TIME_STAMP33_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP34) */
#define CAN_HR_TIME_STAMP34_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP34_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP34_TS_SHIFT)
#define CAN_HR_TIME_STAMP34_TS(n)     (((n) << CAN_HR_TIME_STAMP34_TS_SHIFT) & CAN_HR_TIME_STAMP34_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP35) */
#define CAN_HR_TIME_STAMP35_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP35_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP35_TS_SHIFT)
#define CAN_HR_TIME_STAMP35_TS(n)     (((n) << CAN_HR_TIME_STAMP35_TS_SHIFT) & CAN_HR_TIME_STAMP35_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP36) */
#define CAN_HR_TIME_STAMP36_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP36_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP36_TS_SHIFT)
#define CAN_HR_TIME_STAMP36_TS(n)     (((n) << CAN_HR_TIME_STAMP36_TS_SHIFT) & CAN_HR_TIME_STAMP36_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP37) */
#define CAN_HR_TIME_STAMP37_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP37_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP37_TS_SHIFT)
#define CAN_HR_TIME_STAMP37_TS(n)     (((n) << CAN_HR_TIME_STAMP37_TS_SHIFT) & CAN_HR_TIME_STAMP37_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP38) */
#define CAN_HR_TIME_STAMP38_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP38_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP38_TS_SHIFT)
#define CAN_HR_TIME_STAMP38_TS(n)     (((n) << CAN_HR_TIME_STAMP38_TS_SHIFT) & CAN_HR_TIME_STAMP38_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP39) */
#define CAN_HR_TIME_STAMP39_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP39_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP39_TS_SHIFT)
#define CAN_HR_TIME_STAMP39_TS(n)     (((n) << CAN_HR_TIME_STAMP39_TS_SHIFT) & CAN_HR_TIME_STAMP39_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP40) */
#define CAN_HR_TIME_STAMP40_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP40_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP40_TS_SHIFT)
#define CAN_HR_TIME_STAMP40_TS(n)     (((n) << CAN_HR_TIME_STAMP40_TS_SHIFT) & CAN_HR_TIME_STAMP40_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP41) */
#define CAN_HR_TIME_STAMP41_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP41_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP41_TS_SHIFT)
#define CAN_HR_TIME_STAMP41_TS(n)     (((n) << CAN_HR_TIME_STAMP41_TS_SHIFT) & CAN_HR_TIME_STAMP41_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP42) */
#define CAN_HR_TIME_STAMP42_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP42_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP42_TS_SHIFT)
#define CAN_HR_TIME_STAMP42_TS(n)     (((n) << CAN_HR_TIME_STAMP42_TS_SHIFT) & CAN_HR_TIME_STAMP42_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP43) */
#define CAN_HR_TIME_STAMP43_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP43_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP43_TS_SHIFT)
#define CAN_HR_TIME_STAMP43_TS(n)     (((n) << CAN_HR_TIME_STAMP43_TS_SHIFT) & CAN_HR_TIME_STAMP43_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP44) */
#define CAN_HR_TIME_STAMP44_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP44_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP44_TS_SHIFT)
#define CAN_HR_TIME_STAMP44_TS(n)     (((n) << CAN_HR_TIME_STAMP44_TS_SHIFT) & CAN_HR_TIME_STAMP44_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP45) */
#define CAN_HR_TIME_STAMP45_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP45_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP45_TS_SHIFT)
#define CAN_HR_TIME_STAMP45_TS(n)     (((n) << CAN_HR_TIME_STAMP45_TS_SHIFT) & CAN_HR_TIME_STAMP45_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP46) */
#define CAN_HR_TIME_STAMP46_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP46_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP46_TS_SHIFT)
#define CAN_HR_TIME_STAMP46_TS(n)     (((n) << CAN_HR_TIME_STAMP46_TS_SHIFT) & CAN_HR_TIME_STAMP46_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP47) */
#define CAN_HR_TIME_STAMP47_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP47_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP47_TS_SHIFT)
#define CAN_HR_TIME_STAMP47_TS(n)     (((n) << CAN_HR_TIME_STAMP47_TS_SHIFT) & CAN_HR_TIME_STAMP47_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP48) */
#define CAN_HR_TIME_STAMP48_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP48_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP48_TS_SHIFT)
#define CAN_HR_TIME_STAMP48_TS(n)     (((n) << CAN_HR_TIME_STAMP48_TS_SHIFT) & CAN_HR_TIME_STAMP48_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP49) */
#define CAN_HR_TIME_STAMP49_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP49_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP49_TS_SHIFT)
#define CAN_HR_TIME_STAMP49_TS(n)     (((n) << CAN_HR_TIME_STAMP49_TS_SHIFT) & CAN_HR_TIME_STAMP49_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP50) */
#define CAN_HR_TIME_STAMP50_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP50_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP50_TS_SHIFT)
#define CAN_HR_TIME_STAMP50_TS(n)     (((n) << CAN_HR_TIME_STAMP50_TS_SHIFT) & CAN_HR_TIME_STAMP50_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP51) */
#define CAN_HR_TIME_STAMP51_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP51_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP51_TS_SHIFT)
#define CAN_HR_TIME_STAMP51_TS(n)     (((n) << CAN_HR_TIME_STAMP51_TS_SHIFT) & CAN_HR_TIME_STAMP51_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP52) */
#define CAN_HR_TIME_STAMP52_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP52_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP52_TS_SHIFT)
#define CAN_HR_TIME_STAMP52_TS(n)     (((n) << CAN_HR_TIME_STAMP52_TS_SHIFT) & CAN_HR_TIME_STAMP52_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP53) */
#define CAN_HR_TIME_STAMP53_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP53_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP53_TS_SHIFT)
#define CAN_HR_TIME_STAMP53_TS(n)     (((n) << CAN_HR_TIME_STAMP53_TS_SHIFT) & CAN_HR_TIME_STAMP53_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP54) */
#define CAN_HR_TIME_STAMP54_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP54_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP54_TS_SHIFT)
#define CAN_HR_TIME_STAMP54_TS(n)     (((n) << CAN_HR_TIME_STAMP54_TS_SHIFT) & CAN_HR_TIME_STAMP54_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP55) */
#define CAN_HR_TIME_STAMP55_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP55_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP55_TS_SHIFT)
#define CAN_HR_TIME_STAMP55_TS(n)     (((n) << CAN_HR_TIME_STAMP55_TS_SHIFT) & CAN_HR_TIME_STAMP55_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP56) */
#define CAN_HR_TIME_STAMP56_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP56_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP56_TS_SHIFT)
#define CAN_HR_TIME_STAMP56_TS(n)     (((n) << CAN_HR_TIME_STAMP56_TS_SHIFT) & CAN_HR_TIME_STAMP56_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP57) */
#define CAN_HR_TIME_STAMP57_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP57_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP57_TS_SHIFT)
#define CAN_HR_TIME_STAMP57_TS(n)     (((n) << CAN_HR_TIME_STAMP57_TS_SHIFT) & CAN_HR_TIME_STAMP57_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP58) */
#define CAN_HR_TIME_STAMP58_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP58_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP58_TS_SHIFT)
#define CAN_HR_TIME_STAMP58_TS(n)     (((n) << CAN_HR_TIME_STAMP58_TS_SHIFT) & CAN_HR_TIME_STAMP58_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP59) */
#define CAN_HR_TIME_STAMP59_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP59_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP59_TS_SHIFT)
#define CAN_HR_TIME_STAMP59_TS(n)     (((n) << CAN_HR_TIME_STAMP59_TS_SHIFT) & CAN_HR_TIME_STAMP59_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP60) */
#define CAN_HR_TIME_STAMP60_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP60_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP60_TS_SHIFT)
#define CAN_HR_TIME_STAMP60_TS(n)     (((n) << CAN_HR_TIME_STAMP60_TS_SHIFT) & CAN_HR_TIME_STAMP60_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP61) */
#define CAN_HR_TIME_STAMP61_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP61_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP61_TS_SHIFT)
#define CAN_HR_TIME_STAMP61_TS(n)     (((n) << CAN_HR_TIME_STAMP61_TS_SHIFT) & CAN_HR_TIME_STAMP61_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP62) */
#define CAN_HR_TIME_STAMP62_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP62_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP62_TS_SHIFT)
#define CAN_HR_TIME_STAMP62_TS(n)     (((n) << CAN_HR_TIME_STAMP62_TS_SHIFT) & CAN_HR_TIME_STAMP62_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP63) */
#define CAN_HR_TIME_STAMP63_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP63_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP63_TS_SHIFT)
#define CAN_HR_TIME_STAMP63_TS(n)     (((n) << CAN_HR_TIME_STAMP63_TS_SHIFT) & CAN_HR_TIME_STAMP63_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP64) */
#define CAN_HR_TIME_STAMP64_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP64_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP64_TS_SHIFT)
#define CAN_HR_TIME_STAMP64_TS(n)     (((n) << CAN_HR_TIME_STAMP64_TS_SHIFT) & CAN_HR_TIME_STAMP64_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP65) */
#define CAN_HR_TIME_STAMP65_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP65_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP65_TS_SHIFT)
#define CAN_HR_TIME_STAMP65_TS(n)     (((n) << CAN_HR_TIME_STAMP65_TS_SHIFT) & CAN_HR_TIME_STAMP65_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP66) */
#define CAN_HR_TIME_STAMP66_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP66_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP66_TS_SHIFT)
#define CAN_HR_TIME_STAMP66_TS(n)     (((n) << CAN_HR_TIME_STAMP66_TS_SHIFT) & CAN_HR_TIME_STAMP66_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP67) */
#define CAN_HR_TIME_STAMP67_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP67_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP67_TS_SHIFT)
#define CAN_HR_TIME_STAMP67_TS(n)     (((n) << CAN_HR_TIME_STAMP67_TS_SHIFT) & CAN_HR_TIME_STAMP67_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP68) */
#define CAN_HR_TIME_STAMP68_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP68_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP68_TS_SHIFT)
#define CAN_HR_TIME_STAMP68_TS(n)     (((n) << CAN_HR_TIME_STAMP68_TS_SHIFT) & CAN_HR_TIME_STAMP68_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP69) */
#define CAN_HR_TIME_STAMP69_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP69_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP69_TS_SHIFT)
#define CAN_HR_TIME_STAMP69_TS(n)     (((n) << CAN_HR_TIME_STAMP69_TS_SHIFT) & CAN_HR_TIME_STAMP69_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP70) */
#define CAN_HR_TIME_STAMP70_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP70_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP70_TS_SHIFT)
#define CAN_HR_TIME_STAMP70_TS(n)     (((n) << CAN_HR_TIME_STAMP70_TS_SHIFT) & CAN_HR_TIME_STAMP70_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP71) */
#define CAN_HR_TIME_STAMP71_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP71_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP71_TS_SHIFT)
#define CAN_HR_TIME_STAMP71_TS(n)     (((n) << CAN_HR_TIME_STAMP71_TS_SHIFT) & CAN_HR_TIME_STAMP71_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP72) */
#define CAN_HR_TIME_STAMP72_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP72_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP72_TS_SHIFT)
#define CAN_HR_TIME_STAMP72_TS(n)     (((n) << CAN_HR_TIME_STAMP72_TS_SHIFT) & CAN_HR_TIME_STAMP72_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP73) */
#define CAN_HR_TIME_STAMP73_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP73_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP73_TS_SHIFT)
#define CAN_HR_TIME_STAMP73_TS(n)     (((n) << CAN_HR_TIME_STAMP73_TS_SHIFT) & CAN_HR_TIME_STAMP73_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP74) */
#define CAN_HR_TIME_STAMP74_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP74_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP74_TS_SHIFT)
#define CAN_HR_TIME_STAMP74_TS(n)     (((n) << CAN_HR_TIME_STAMP74_TS_SHIFT) & CAN_HR_TIME_STAMP74_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP75) */
#define CAN_HR_TIME_STAMP75_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP75_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP75_TS_SHIFT)
#define CAN_HR_TIME_STAMP75_TS(n)     (((n) << CAN_HR_TIME_STAMP75_TS_SHIFT) & CAN_HR_TIME_STAMP75_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP76) */
#define CAN_HR_TIME_STAMP76_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP76_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP76_TS_SHIFT)
#define CAN_HR_TIME_STAMP76_TS(n)     (((n) << CAN_HR_TIME_STAMP76_TS_SHIFT) & CAN_HR_TIME_STAMP76_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP77) */
#define CAN_HR_TIME_STAMP77_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP77_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP77_TS_SHIFT)
#define CAN_HR_TIME_STAMP77_TS(n)     (((n) << CAN_HR_TIME_STAMP77_TS_SHIFT) & CAN_HR_TIME_STAMP77_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP78) */
#define CAN_HR_TIME_STAMP78_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP78_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP78_TS_SHIFT)
#define CAN_HR_TIME_STAMP78_TS(n)     (((n) << CAN_HR_TIME_STAMP78_TS_SHIFT) & CAN_HR_TIME_STAMP78_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP79) */
#define CAN_HR_TIME_STAMP79_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP79_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP79_TS_SHIFT)
#define CAN_HR_TIME_STAMP79_TS(n)     (((n) << CAN_HR_TIME_STAMP79_TS_SHIFT) & CAN_HR_TIME_STAMP79_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP80) */
#define CAN_HR_TIME_STAMP80_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP80_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP80_TS_SHIFT)
#define CAN_HR_TIME_STAMP80_TS(n)     (((n) << CAN_HR_TIME_STAMP80_TS_SHIFT) & CAN_HR_TIME_STAMP80_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP81) */
#define CAN_HR_TIME_STAMP81_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP81_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP81_TS_SHIFT)
#define CAN_HR_TIME_STAMP81_TS(n)     (((n) << CAN_HR_TIME_STAMP81_TS_SHIFT) & CAN_HR_TIME_STAMP81_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP82) */
#define CAN_HR_TIME_STAMP82_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP82_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP82_TS_SHIFT)
#define CAN_HR_TIME_STAMP82_TS(n)     (((n) << CAN_HR_TIME_STAMP82_TS_SHIFT) & CAN_HR_TIME_STAMP82_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP83) */
#define CAN_HR_TIME_STAMP83_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP83_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP83_TS_SHIFT)
#define CAN_HR_TIME_STAMP83_TS(n)     (((n) << CAN_HR_TIME_STAMP83_TS_SHIFT) & CAN_HR_TIME_STAMP83_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP84) */
#define CAN_HR_TIME_STAMP84_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP84_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP84_TS_SHIFT)
#define CAN_HR_TIME_STAMP84_TS(n)     (((n) << CAN_HR_TIME_STAMP84_TS_SHIFT) & CAN_HR_TIME_STAMP84_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP85) */
#define CAN_HR_TIME_STAMP85_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP85_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP85_TS_SHIFT)
#define CAN_HR_TIME_STAMP85_TS(n)     (((n) << CAN_HR_TIME_STAMP85_TS_SHIFT) & CAN_HR_TIME_STAMP85_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP86) */
#define CAN_HR_TIME_STAMP86_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP86_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP86_TS_SHIFT)
#define CAN_HR_TIME_STAMP86_TS(n)     (((n) << CAN_HR_TIME_STAMP86_TS_SHIFT) & CAN_HR_TIME_STAMP86_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP87) */
#define CAN_HR_TIME_STAMP87_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP87_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP87_TS_SHIFT)
#define CAN_HR_TIME_STAMP87_TS(n)     (((n) << CAN_HR_TIME_STAMP87_TS_SHIFT) & CAN_HR_TIME_STAMP87_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP88) */
#define CAN_HR_TIME_STAMP88_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP88_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP88_TS_SHIFT)
#define CAN_HR_TIME_STAMP88_TS(n)     (((n) << CAN_HR_TIME_STAMP88_TS_SHIFT) & CAN_HR_TIME_STAMP88_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP89) */
#define CAN_HR_TIME_STAMP89_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP89_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP89_TS_SHIFT)
#define CAN_HR_TIME_STAMP89_TS(n)     (((n) << CAN_HR_TIME_STAMP89_TS_SHIFT) & CAN_HR_TIME_STAMP89_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP90) */
#define CAN_HR_TIME_STAMP90_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP90_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP90_TS_SHIFT)
#define CAN_HR_TIME_STAMP90_TS(n)     (((n) << CAN_HR_TIME_STAMP90_TS_SHIFT) & CAN_HR_TIME_STAMP90_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP91) */
#define CAN_HR_TIME_STAMP91_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP91_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP91_TS_SHIFT)
#define CAN_HR_TIME_STAMP91_TS(n)     (((n) << CAN_HR_TIME_STAMP91_TS_SHIFT) & CAN_HR_TIME_STAMP91_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP92) */
#define CAN_HR_TIME_STAMP92_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP92_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP92_TS_SHIFT)
#define CAN_HR_TIME_STAMP92_TS(n)     (((n) << CAN_HR_TIME_STAMP92_TS_SHIFT) & CAN_HR_TIME_STAMP92_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP93) */
#define CAN_HR_TIME_STAMP93_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP93_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP93_TS_SHIFT)
#define CAN_HR_TIME_STAMP93_TS(n)     (((n) << CAN_HR_TIME_STAMP93_TS_SHIFT) & CAN_HR_TIME_STAMP93_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP94) */
#define CAN_HR_TIME_STAMP94_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP94_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP94_TS_SHIFT)
#define CAN_HR_TIME_STAMP94_TS(n)     (((n) << CAN_HR_TIME_STAMP94_TS_SHIFT) & CAN_HR_TIME_STAMP94_TS_MASK)

/* High-Resolution Timestamp (HR_TIME_STAMP95) */
#define CAN_HR_TIME_STAMP95_TS_SHIFT  (0)  /* Bits 0-32: High-Resolution Timestamp */
#define CAN_HR_TIME_STAMP95_TS_MASK   (0xFFFFFFFF << CAN_HR_TIME_STAMP95_TS_SHIFT)
#define CAN_HR_TIME_STAMP95_TS(n)     (((n) << CAN_HR_TIME_STAMP95_TS_SHIFT) & CAN_HR_TIME_STAMP95_TS_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL0) */
#define CAN_ERFFEL0_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL0_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL0_FEL_SHIFT)
#define CAN_ERFFEL0_FEL(n)     (((n) << CAN_ERFFEL0_FEL_SHIFT) & CAN_ERFFEL0_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL1) */
#define CAN_ERFFEL1_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL1_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL1_FEL_SHIFT)
#define CAN_ERFFEL1_FEL(n)     (((n) << CAN_ERFFEL1_FEL_SHIFT) & CAN_ERFFEL1_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL2) */
#define CAN_ERFFEL2_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL2_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL2_FEL_SHIFT)
#define CAN_ERFFEL2_FEL(n)     (((n) << CAN_ERFFEL2_FEL_SHIFT) & CAN_ERFFEL2_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL3) */
#define CAN_ERFFEL3_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL3_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL3_FEL_SHIFT)
#define CAN_ERFFEL3_FEL(n)     (((n) << CAN_ERFFEL3_FEL_SHIFT) & CAN_ERFFEL3_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL4) */
#define CAN_ERFFEL4_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL4_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL4_FEL_SHIFT)
#define CAN_ERFFEL4_FEL(n)     (((n) << CAN_ERFFEL4_FEL_SHIFT) & CAN_ERFFEL4_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL5) */
#define CAN_ERFFEL5_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL5_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL5_FEL_SHIFT)
#define CAN_ERFFEL5_FEL(n)     (((n) << CAN_ERFFEL5_FEL_SHIFT) & CAN_ERFFEL5_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL6) */
#define CAN_ERFFEL6_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL6_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL6_FEL_SHIFT)
#define CAN_ERFFEL6_FEL(n)     (((n) << CAN_ERFFEL6_FEL_SHIFT) & CAN_ERFFEL6_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL7) */
#define CAN_ERFFEL7_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL7_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL7_FEL_SHIFT)
#define CAN_ERFFEL7_FEL(n)     (((n) << CAN_ERFFEL7_FEL_SHIFT) & CAN_ERFFEL7_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL8) */
#define CAN_ERFFEL8_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL8_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL8_FEL_SHIFT)
#define CAN_ERFFEL8_FEL(n)     (((n) << CAN_ERFFEL8_FEL_SHIFT) & CAN_ERFFEL8_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL9) */
#define CAN_ERFFEL9_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL9_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL9_FEL_SHIFT)
#define CAN_ERFFEL9_FEL(n)     (((n) << CAN_ERFFEL9_FEL_SHIFT) & CAN_ERFFEL9_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL10) */
#define CAN_ERFFEL10_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL10_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL10_FEL_SHIFT)
#define CAN_ERFFEL10_FEL(n)     (((n) << CAN_ERFFEL10_FEL_SHIFT) & CAN_ERFFEL10_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL11) */
#define CAN_ERFFEL11_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL11_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL11_FEL_SHIFT)
#define CAN_ERFFEL11_FEL(n)     (((n) << CAN_ERFFEL11_FEL_SHIFT) & CAN_ERFFEL11_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL12) */
#define CAN_ERFFEL12_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL12_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL12_FEL_SHIFT)
#define CAN_ERFFEL12_FEL(n)     (((n) << CAN_ERFFEL12_FEL_SHIFT) & CAN_ERFFEL12_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL13) */
#define CAN_ERFFEL13_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL13_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL13_FEL_SHIFT)
#define CAN_ERFFEL13_FEL(n)     (((n) << CAN_ERFFEL13_FEL_SHIFT) & CAN_ERFFEL13_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL14) */
#define CAN_ERFFEL14_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL14_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL14_FEL_SHIFT)
#define CAN_ERFFEL14_FEL(n)     (((n) << CAN_ERFFEL14_FEL_SHIFT) & CAN_ERFFEL14_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL15) */
#define CAN_ERFFEL15_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL15_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL15_FEL_SHIFT)
#define CAN_ERFFEL15_FEL(n)     (((n) << CAN_ERFFEL15_FEL_SHIFT) & CAN_ERFFEL15_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL16) */
#define CAN_ERFFEL16_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL16_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL16_FEL_SHIFT)
#define CAN_ERFFEL16_FEL(n)     (((n) << CAN_ERFFEL16_FEL_SHIFT) & CAN_ERFFEL16_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL17) */
#define CAN_ERFFEL17_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL17_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL17_FEL_SHIFT)
#define CAN_ERFFEL17_FEL(n)     (((n) << CAN_ERFFEL17_FEL_SHIFT) & CAN_ERFFEL17_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL18) */
#define CAN_ERFFEL18_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL18_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL18_FEL_SHIFT)
#define CAN_ERFFEL18_FEL(n)     (((n) << CAN_ERFFEL18_FEL_SHIFT) & CAN_ERFFEL18_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL19) */
#define CAN_ERFFEL19_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL19_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL19_FEL_SHIFT)
#define CAN_ERFFEL19_FEL(n)     (((n) << CAN_ERFFEL19_FEL_SHIFT) & CAN_ERFFEL19_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL20) */
#define CAN_ERFFEL20_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL20_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL20_FEL_SHIFT)
#define CAN_ERFFEL20_FEL(n)     (((n) << CAN_ERFFEL20_FEL_SHIFT) & CAN_ERFFEL20_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL21) */
#define CAN_ERFFEL21_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL21_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL21_FEL_SHIFT)
#define CAN_ERFFEL21_FEL(n)     (((n) << CAN_ERFFEL21_FEL_SHIFT) & CAN_ERFFEL21_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL22) */
#define CAN_ERFFEL22_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL22_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL22_FEL_SHIFT)
#define CAN_ERFFEL22_FEL(n)     (((n) << CAN_ERFFEL22_FEL_SHIFT) & CAN_ERFFEL22_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL23) */
#define CAN_ERFFEL23_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL23_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL23_FEL_SHIFT)
#define CAN_ERFFEL23_FEL(n)     (((n) << CAN_ERFFEL23_FEL_SHIFT) & CAN_ERFFEL23_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL24) */
#define CAN_ERFFEL24_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL24_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL24_FEL_SHIFT)
#define CAN_ERFFEL24_FEL(n)     (((n) << CAN_ERFFEL24_FEL_SHIFT) & CAN_ERFFEL24_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL25) */
#define CAN_ERFFEL25_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL25_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL25_FEL_SHIFT)
#define CAN_ERFFEL25_FEL(n)     (((n) << CAN_ERFFEL25_FEL_SHIFT) & CAN_ERFFEL25_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL26) */
#define CAN_ERFFEL26_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL26_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL26_FEL_SHIFT)
#define CAN_ERFFEL26_FEL(n)     (((n) << CAN_ERFFEL26_FEL_SHIFT) & CAN_ERFFEL26_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL27) */
#define CAN_ERFFEL27_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL27_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL27_FEL_SHIFT)
#define CAN_ERFFEL27_FEL(n)     (((n) << CAN_ERFFEL27_FEL_SHIFT) & CAN_ERFFEL27_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL28) */
#define CAN_ERFFEL28_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL28_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL28_FEL_SHIFT)
#define CAN_ERFFEL28_FEL(n)     (((n) << CAN_ERFFEL28_FEL_SHIFT) & CAN_ERFFEL28_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL29) */
#define CAN_ERFFEL29_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL29_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL29_FEL_SHIFT)
#define CAN_ERFFEL29_FEL(n)     (((n) << CAN_ERFFEL29_FEL_SHIFT) & CAN_ERFFEL29_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL30) */
#define CAN_ERFFEL30_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL30_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL30_FEL_SHIFT)
#define CAN_ERFFEL30_FEL(n)     (((n) << CAN_ERFFEL30_FEL_SHIFT) & CAN_ERFFEL30_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL31) */
#define CAN_ERFFEL31_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL31_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL31_FEL_SHIFT)
#define CAN_ERFFEL31_FEL(n)     (((n) << CAN_ERFFEL31_FEL_SHIFT) & CAN_ERFFEL31_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL32) */
#define CAN_ERFFEL32_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL32_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL32_FEL_SHIFT)
#define CAN_ERFFEL32_FEL(n)     (((n) << CAN_ERFFEL32_FEL_SHIFT) & CAN_ERFFEL32_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL33) */
#define CAN_ERFFEL33_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL33_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL33_FEL_SHIFT)
#define CAN_ERFFEL33_FEL(n)     (((n) << CAN_ERFFEL33_FEL_SHIFT) & CAN_ERFFEL33_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL34) */
#define CAN_ERFFEL34_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL34_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL34_FEL_SHIFT)
#define CAN_ERFFEL34_FEL(n)     (((n) << CAN_ERFFEL34_FEL_SHIFT) & CAN_ERFFEL34_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL35) */
#define CAN_ERFFEL35_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL35_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL35_FEL_SHIFT)
#define CAN_ERFFEL35_FEL(n)     (((n) << CAN_ERFFEL35_FEL_SHIFT) & CAN_ERFFEL35_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL36) */
#define CAN_ERFFEL36_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL36_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL36_FEL_SHIFT)
#define CAN_ERFFEL36_FEL(n)     (((n) << CAN_ERFFEL36_FEL_SHIFT) & CAN_ERFFEL36_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL37) */
#define CAN_ERFFEL37_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL37_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL37_FEL_SHIFT)
#define CAN_ERFFEL37_FEL(n)     (((n) << CAN_ERFFEL37_FEL_SHIFT) & CAN_ERFFEL37_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL38) */
#define CAN_ERFFEL38_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL38_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL38_FEL_SHIFT)
#define CAN_ERFFEL38_FEL(n)     (((n) << CAN_ERFFEL38_FEL_SHIFT) & CAN_ERFFEL38_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL39) */
#define CAN_ERFFEL39_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL39_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL39_FEL_SHIFT)
#define CAN_ERFFEL39_FEL(n)     (((n) << CAN_ERFFEL39_FEL_SHIFT) & CAN_ERFFEL39_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL40) */
#define CAN_ERFFEL40_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL40_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL40_FEL_SHIFT)
#define CAN_ERFFEL40_FEL(n)     (((n) << CAN_ERFFEL40_FEL_SHIFT) & CAN_ERFFEL40_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL41) */
#define CAN_ERFFEL41_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL41_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL41_FEL_SHIFT)
#define CAN_ERFFEL41_FEL(n)     (((n) << CAN_ERFFEL41_FEL_SHIFT) & CAN_ERFFEL41_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL42) */
#define CAN_ERFFEL42_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL42_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL42_FEL_SHIFT)
#define CAN_ERFFEL42_FEL(n)     (((n) << CAN_ERFFEL42_FEL_SHIFT) & CAN_ERFFEL42_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL43) */
#define CAN_ERFFEL43_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL43_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL43_FEL_SHIFT)
#define CAN_ERFFEL43_FEL(n)     (((n) << CAN_ERFFEL43_FEL_SHIFT) & CAN_ERFFEL43_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL44) */
#define CAN_ERFFEL44_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL44_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL44_FEL_SHIFT)
#define CAN_ERFFEL44_FEL(n)     (((n) << CAN_ERFFEL44_FEL_SHIFT) & CAN_ERFFEL44_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL45) */
#define CAN_ERFFEL45_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL45_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL45_FEL_SHIFT)
#define CAN_ERFFEL45_FEL(n)     (((n) << CAN_ERFFEL45_FEL_SHIFT) & CAN_ERFFEL45_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL46) */
#define CAN_ERFFEL46_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL46_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL46_FEL_SHIFT)
#define CAN_ERFFEL46_FEL(n)     (((n) << CAN_ERFFEL46_FEL_SHIFT) & CAN_ERFFEL46_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL47) */
#define CAN_ERFFEL47_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL47_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL47_FEL_SHIFT)
#define CAN_ERFFEL47_FEL(n)     (((n) << CAN_ERFFEL47_FEL_SHIFT) & CAN_ERFFEL47_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL48) */
#define CAN_ERFFEL48_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL48_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL48_FEL_SHIFT)
#define CAN_ERFFEL48_FEL(n)     (((n) << CAN_ERFFEL48_FEL_SHIFT) & CAN_ERFFEL48_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL49) */
#define CAN_ERFFEL49_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL49_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL49_FEL_SHIFT)
#define CAN_ERFFEL49_FEL(n)     (((n) << CAN_ERFFEL49_FEL_SHIFT) & CAN_ERFFEL49_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL50) */
#define CAN_ERFFEL50_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL50_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL50_FEL_SHIFT)
#define CAN_ERFFEL50_FEL(n)     (((n) << CAN_ERFFEL50_FEL_SHIFT) & CAN_ERFFEL50_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL51) */
#define CAN_ERFFEL51_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL51_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL51_FEL_SHIFT)
#define CAN_ERFFEL51_FEL(n)     (((n) << CAN_ERFFEL51_FEL_SHIFT) & CAN_ERFFEL51_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL52) */
#define CAN_ERFFEL52_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL52_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL52_FEL_SHIFT)
#define CAN_ERFFEL52_FEL(n)     (((n) << CAN_ERFFEL52_FEL_SHIFT) & CAN_ERFFEL52_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL53) */
#define CAN_ERFFEL53_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL53_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL53_FEL_SHIFT)
#define CAN_ERFFEL53_FEL(n)     (((n) << CAN_ERFFEL53_FEL_SHIFT) & CAN_ERFFEL53_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL54) */
#define CAN_ERFFEL54_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL54_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL54_FEL_SHIFT)
#define CAN_ERFFEL54_FEL(n)     (((n) << CAN_ERFFEL54_FEL_SHIFT) & CAN_ERFFEL54_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL55) */
#define CAN_ERFFEL55_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL55_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL55_FEL_SHIFT)
#define CAN_ERFFEL55_FEL(n)     (((n) << CAN_ERFFEL55_FEL_SHIFT) & CAN_ERFFEL55_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL56) */
#define CAN_ERFFEL56_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL56_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL56_FEL_SHIFT)
#define CAN_ERFFEL56_FEL(n)     (((n) << CAN_ERFFEL56_FEL_SHIFT) & CAN_ERFFEL56_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL57) */
#define CAN_ERFFEL57_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL57_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL57_FEL_SHIFT)
#define CAN_ERFFEL57_FEL(n)     (((n) << CAN_ERFFEL57_FEL_SHIFT) & CAN_ERFFEL57_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL58) */
#define CAN_ERFFEL58_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL58_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL58_FEL_SHIFT)
#define CAN_ERFFEL58_FEL(n)     (((n) << CAN_ERFFEL58_FEL_SHIFT) & CAN_ERFFEL58_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL59) */
#define CAN_ERFFEL59_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL59_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL59_FEL_SHIFT)
#define CAN_ERFFEL59_FEL(n)     (((n) << CAN_ERFFEL59_FEL_SHIFT) & CAN_ERFFEL59_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL60) */
#define CAN_ERFFEL60_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL60_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL60_FEL_SHIFT)
#define CAN_ERFFEL60_FEL(n)     (((n) << CAN_ERFFEL60_FEL_SHIFT) & CAN_ERFFEL60_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL61) */
#define CAN_ERFFEL61_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL61_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL61_FEL_SHIFT)
#define CAN_ERFFEL61_FEL(n)     (((n) << CAN_ERFFEL61_FEL_SHIFT) & CAN_ERFFEL61_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL62) */
#define CAN_ERFFEL62_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL62_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL62_FEL_SHIFT)
#define CAN_ERFFEL62_FEL(n)     (((n) << CAN_ERFFEL62_FEL_SHIFT) & CAN_ERFFEL62_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL63) */
#define CAN_ERFFEL63_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL63_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL63_FEL_SHIFT)
#define CAN_ERFFEL63_FEL(n)     (((n) << CAN_ERFFEL63_FEL_SHIFT) & CAN_ERFFEL63_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL64) */
#define CAN_ERFFEL64_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL64_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL64_FEL_SHIFT)
#define CAN_ERFFEL64_FEL(n)     (((n) << CAN_ERFFEL64_FEL_SHIFT) & CAN_ERFFEL64_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL65) */
#define CAN_ERFFEL65_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL65_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL65_FEL_SHIFT)
#define CAN_ERFFEL65_FEL(n)     (((n) << CAN_ERFFEL65_FEL_SHIFT) & CAN_ERFFEL65_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL66) */
#define CAN_ERFFEL66_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL66_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL66_FEL_SHIFT)
#define CAN_ERFFEL66_FEL(n)     (((n) << CAN_ERFFEL66_FEL_SHIFT) & CAN_ERFFEL66_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL67) */
#define CAN_ERFFEL67_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL67_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL67_FEL_SHIFT)
#define CAN_ERFFEL67_FEL(n)     (((n) << CAN_ERFFEL67_FEL_SHIFT) & CAN_ERFFEL67_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL68) */
#define CAN_ERFFEL68_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL68_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL68_FEL_SHIFT)
#define CAN_ERFFEL68_FEL(n)     (((n) << CAN_ERFFEL68_FEL_SHIFT) & CAN_ERFFEL68_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL69) */
#define CAN_ERFFEL69_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL69_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL69_FEL_SHIFT)
#define CAN_ERFFEL69_FEL(n)     (((n) << CAN_ERFFEL69_FEL_SHIFT) & CAN_ERFFEL69_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL70) */
#define CAN_ERFFEL70_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL70_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL70_FEL_SHIFT)
#define CAN_ERFFEL70_FEL(n)     (((n) << CAN_ERFFEL70_FEL_SHIFT) & CAN_ERFFEL70_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL71) */
#define CAN_ERFFEL71_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL71_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL71_FEL_SHIFT)
#define CAN_ERFFEL71_FEL(n)     (((n) << CAN_ERFFEL71_FEL_SHIFT) & CAN_ERFFEL71_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL72) */
#define CAN_ERFFEL72_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL72_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL72_FEL_SHIFT)
#define CAN_ERFFEL72_FEL(n)     (((n) << CAN_ERFFEL72_FEL_SHIFT) & CAN_ERFFEL72_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL73) */
#define CAN_ERFFEL73_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL73_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL73_FEL_SHIFT)
#define CAN_ERFFEL73_FEL(n)     (((n) << CAN_ERFFEL73_FEL_SHIFT) & CAN_ERFFEL73_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL74) */
#define CAN_ERFFEL74_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL74_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL74_FEL_SHIFT)
#define CAN_ERFFEL74_FEL(n)     (((n) << CAN_ERFFEL74_FEL_SHIFT) & CAN_ERFFEL74_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL75) */
#define CAN_ERFFEL75_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL75_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL75_FEL_SHIFT)
#define CAN_ERFFEL75_FEL(n)     (((n) << CAN_ERFFEL75_FEL_SHIFT) & CAN_ERFFEL75_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL76) */
#define CAN_ERFFEL76_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL76_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL76_FEL_SHIFT)
#define CAN_ERFFEL76_FEL(n)     (((n) << CAN_ERFFEL76_FEL_SHIFT) & CAN_ERFFEL76_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL77) */
#define CAN_ERFFEL77_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL77_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL77_FEL_SHIFT)
#define CAN_ERFFEL77_FEL(n)     (((n) << CAN_ERFFEL77_FEL_SHIFT) & CAN_ERFFEL77_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL78) */
#define CAN_ERFFEL78_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL78_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL78_FEL_SHIFT)
#define CAN_ERFFEL78_FEL(n)     (((n) << CAN_ERFFEL78_FEL_SHIFT) & CAN_ERFFEL78_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL79) */
#define CAN_ERFFEL79_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL79_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL79_FEL_SHIFT)
#define CAN_ERFFEL79_FEL(n)     (((n) << CAN_ERFFEL79_FEL_SHIFT) & CAN_ERFFEL79_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL80) */
#define CAN_ERFFEL80_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL80_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL80_FEL_SHIFT)
#define CAN_ERFFEL80_FEL(n)     (((n) << CAN_ERFFEL80_FEL_SHIFT) & CAN_ERFFEL80_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL81) */
#define CAN_ERFFEL81_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL81_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL81_FEL_SHIFT)
#define CAN_ERFFEL81_FEL(n)     (((n) << CAN_ERFFEL81_FEL_SHIFT) & CAN_ERFFEL81_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL82) */
#define CAN_ERFFEL82_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL82_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL82_FEL_SHIFT)
#define CAN_ERFFEL82_FEL(n)     (((n) << CAN_ERFFEL82_FEL_SHIFT) & CAN_ERFFEL82_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL83) */
#define CAN_ERFFEL83_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL83_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL83_FEL_SHIFT)
#define CAN_ERFFEL83_FEL(n)     (((n) << CAN_ERFFEL83_FEL_SHIFT) & CAN_ERFFEL83_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL84) */
#define CAN_ERFFEL84_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL84_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL84_FEL_SHIFT)
#define CAN_ERFFEL84_FEL(n)     (((n) << CAN_ERFFEL84_FEL_SHIFT) & CAN_ERFFEL84_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL85) */
#define CAN_ERFFEL85_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL85_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL85_FEL_SHIFT)
#define CAN_ERFFEL85_FEL(n)     (((n) << CAN_ERFFEL85_FEL_SHIFT) & CAN_ERFFEL85_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL86) */
#define CAN_ERFFEL86_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL86_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL86_FEL_SHIFT)
#define CAN_ERFFEL86_FEL(n)     (((n) << CAN_ERFFEL86_FEL_SHIFT) & CAN_ERFFEL86_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL87) */
#define CAN_ERFFEL87_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL87_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL87_FEL_SHIFT)
#define CAN_ERFFEL87_FEL(n)     (((n) << CAN_ERFFEL87_FEL_SHIFT) & CAN_ERFFEL87_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL88) */
#define CAN_ERFFEL88_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL88_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL88_FEL_SHIFT)
#define CAN_ERFFEL88_FEL(n)     (((n) << CAN_ERFFEL88_FEL_SHIFT) & CAN_ERFFEL88_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL89) */
#define CAN_ERFFEL89_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL89_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL89_FEL_SHIFT)
#define CAN_ERFFEL89_FEL(n)     (((n) << CAN_ERFFEL89_FEL_SHIFT) & CAN_ERFFEL89_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL90) */
#define CAN_ERFFEL90_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL90_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL90_FEL_SHIFT)
#define CAN_ERFFEL90_FEL(n)     (((n) << CAN_ERFFEL90_FEL_SHIFT) & CAN_ERFFEL90_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL91) */
#define CAN_ERFFEL91_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL91_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL91_FEL_SHIFT)
#define CAN_ERFFEL91_FEL(n)     (((n) << CAN_ERFFEL91_FEL_SHIFT) & CAN_ERFFEL91_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL92) */
#define CAN_ERFFEL92_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL92_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL92_FEL_SHIFT)
#define CAN_ERFFEL92_FEL(n)     (((n) << CAN_ERFFEL92_FEL_SHIFT) & CAN_ERFFEL92_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL93) */
#define CAN_ERFFEL93_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL93_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL93_FEL_SHIFT)
#define CAN_ERFFEL93_FEL(n)     (((n) << CAN_ERFFEL93_FEL_SHIFT) & CAN_ERFFEL93_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL94) */
#define CAN_ERFFEL94_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL94_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL94_FEL_SHIFT)
#define CAN_ERFFEL94_FEL(n)     (((n) << CAN_ERFFEL94_FEL_SHIFT) & CAN_ERFFEL94_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL95) */
#define CAN_ERFFEL95_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL95_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL95_FEL_SHIFT)
#define CAN_ERFFEL95_FEL(n)     (((n) << CAN_ERFFEL95_FEL_SHIFT) & CAN_ERFFEL95_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL96) */
#define CAN_ERFFEL96_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL96_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL96_FEL_SHIFT)
#define CAN_ERFFEL96_FEL(n)     (((n) << CAN_ERFFEL96_FEL_SHIFT) & CAN_ERFFEL96_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL97) */
#define CAN_ERFFEL97_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL97_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL97_FEL_SHIFT)
#define CAN_ERFFEL97_FEL(n)     (((n) << CAN_ERFFEL97_FEL_SHIFT) & CAN_ERFFEL97_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL98) */
#define CAN_ERFFEL98_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL98_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL98_FEL_SHIFT)
#define CAN_ERFFEL98_FEL(n)     (((n) << CAN_ERFFEL98_FEL_SHIFT) & CAN_ERFFEL98_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL99) */
#define CAN_ERFFEL99_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL99_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL99_FEL_SHIFT)
#define CAN_ERFFEL99_FEL(n)     (((n) << CAN_ERFFEL99_FEL_SHIFT) & CAN_ERFFEL99_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL100) */
#define CAN_ERFFEL100_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL100_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL100_FEL_SHIFT)
#define CAN_ERFFEL100_FEL(n)     (((n) << CAN_ERFFEL100_FEL_SHIFT) & CAN_ERFFEL100_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL101) */
#define CAN_ERFFEL101_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL101_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL101_FEL_SHIFT)
#define CAN_ERFFEL101_FEL(n)     (((n) << CAN_ERFFEL101_FEL_SHIFT) & CAN_ERFFEL101_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL102) */
#define CAN_ERFFEL102_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL102_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL102_FEL_SHIFT)
#define CAN_ERFFEL102_FEL(n)     (((n) << CAN_ERFFEL102_FEL_SHIFT) & CAN_ERFFEL102_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL103) */
#define CAN_ERFFEL103_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL103_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL103_FEL_SHIFT)
#define CAN_ERFFEL103_FEL(n)     (((n) << CAN_ERFFEL103_FEL_SHIFT) & CAN_ERFFEL103_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL104) */
#define CAN_ERFFEL104_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL104_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL104_FEL_SHIFT)
#define CAN_ERFFEL104_FEL(n)     (((n) << CAN_ERFFEL104_FEL_SHIFT) & CAN_ERFFEL104_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL105) */
#define CAN_ERFFEL105_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL105_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL105_FEL_SHIFT)
#define CAN_ERFFEL105_FEL(n)     (((n) << CAN_ERFFEL105_FEL_SHIFT) & CAN_ERFFEL105_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL106) */
#define CAN_ERFFEL106_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL106_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL106_FEL_SHIFT)
#define CAN_ERFFEL106_FEL(n)     (((n) << CAN_ERFFEL106_FEL_SHIFT) & CAN_ERFFEL106_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL107) */
#define CAN_ERFFEL107_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL107_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL107_FEL_SHIFT)
#define CAN_ERFFEL107_FEL(n)     (((n) << CAN_ERFFEL107_FEL_SHIFT) & CAN_ERFFEL107_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL108) */
#define CAN_ERFFEL108_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL108_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL108_FEL_SHIFT)
#define CAN_ERFFEL108_FEL(n)     (((n) << CAN_ERFFEL108_FEL_SHIFT) & CAN_ERFFEL108_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL109) */
#define CAN_ERFFEL109_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL109_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL109_FEL_SHIFT)
#define CAN_ERFFEL109_FEL(n)     (((n) << CAN_ERFFEL109_FEL_SHIFT) & CAN_ERFFEL109_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL110) */
#define CAN_ERFFEL110_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL110_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL110_FEL_SHIFT)
#define CAN_ERFFEL110_FEL(n)     (((n) << CAN_ERFFEL110_FEL_SHIFT) & CAN_ERFFEL110_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL111) */
#define CAN_ERFFEL111_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL111_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL111_FEL_SHIFT)
#define CAN_ERFFEL111_FEL(n)     (((n) << CAN_ERFFEL111_FEL_SHIFT) & CAN_ERFFEL111_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL112) */
#define CAN_ERFFEL112_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL112_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL112_FEL_SHIFT)
#define CAN_ERFFEL112_FEL(n)     (((n) << CAN_ERFFEL112_FEL_SHIFT) & CAN_ERFFEL112_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL113) */
#define CAN_ERFFEL113_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL113_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL113_FEL_SHIFT)
#define CAN_ERFFEL113_FEL(n)     (((n) << CAN_ERFFEL113_FEL_SHIFT) & CAN_ERFFEL113_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL114) */
#define CAN_ERFFEL114_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL114_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL114_FEL_SHIFT)
#define CAN_ERFFEL114_FEL(n)     (((n) << CAN_ERFFEL114_FEL_SHIFT) & CAN_ERFFEL114_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL115) */
#define CAN_ERFFEL115_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL115_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL115_FEL_SHIFT)
#define CAN_ERFFEL115_FEL(n)     (((n) << CAN_ERFFEL115_FEL_SHIFT) & CAN_ERFFEL115_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL116) */
#define CAN_ERFFEL116_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL116_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL116_FEL_SHIFT)
#define CAN_ERFFEL116_FEL(n)     (((n) << CAN_ERFFEL116_FEL_SHIFT) & CAN_ERFFEL116_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL117) */
#define CAN_ERFFEL117_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL117_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL117_FEL_SHIFT)
#define CAN_ERFFEL117_FEL(n)     (((n) << CAN_ERFFEL117_FEL_SHIFT) & CAN_ERFFEL117_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL118) */
#define CAN_ERFFEL118_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL118_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL118_FEL_SHIFT)
#define CAN_ERFFEL118_FEL(n)     (((n) << CAN_ERFFEL118_FEL_SHIFT) & CAN_ERFFEL118_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL119) */
#define CAN_ERFFEL119_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL119_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL119_FEL_SHIFT)
#define CAN_ERFFEL119_FEL(n)     (((n) << CAN_ERFFEL119_FEL_SHIFT) & CAN_ERFFEL119_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL120) */
#define CAN_ERFFEL120_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL120_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL120_FEL_SHIFT)
#define CAN_ERFFEL120_FEL(n)     (((n) << CAN_ERFFEL120_FEL_SHIFT) & CAN_ERFFEL120_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL121) */
#define CAN_ERFFEL121_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL121_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL121_FEL_SHIFT)
#define CAN_ERFFEL121_FEL(n)     (((n) << CAN_ERFFEL121_FEL_SHIFT) & CAN_ERFFEL121_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL122) */
#define CAN_ERFFEL122_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL122_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL122_FEL_SHIFT)
#define CAN_ERFFEL122_FEL(n)     (((n) << CAN_ERFFEL122_FEL_SHIFT) & CAN_ERFFEL122_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL123) */
#define CAN_ERFFEL123_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL123_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL123_FEL_SHIFT)
#define CAN_ERFFEL123_FEL(n)     (((n) << CAN_ERFFEL123_FEL_SHIFT) & CAN_ERFFEL123_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL124) */
#define CAN_ERFFEL124_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL124_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL124_FEL_SHIFT)
#define CAN_ERFFEL124_FEL(n)     (((n) << CAN_ERFFEL124_FEL_SHIFT) & CAN_ERFFEL124_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL125) */
#define CAN_ERFFEL125_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL125_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL125_FEL_SHIFT)
#define CAN_ERFFEL125_FEL(n)     (((n) << CAN_ERFFEL125_FEL_SHIFT) & CAN_ERFFEL125_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL126) */
#define CAN_ERFFEL126_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL126_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL126_FEL_SHIFT)
#define CAN_ERFFEL126_FEL(n)     (((n) << CAN_ERFFEL126_FEL_SHIFT) & CAN_ERFFEL126_FEL_MASK)

/* Enhanced RX FIFO Filter Element (ERFFEL127) */
#define CAN_ERFFEL127_FEL_SHIFT  (0)  /* Bits 0-32: Filter Element Bits */
#define CAN_ERFFEL127_FEL_MASK   (0xFFFFFFFF << CAN_ERFFEL127_FEL_SHIFT)
#define CAN_ERFFEL127_FEL(n)     (((n) << CAN_ERFFEL127_FEL_SHIFT) & CAN_ERFFEL127_FEL_MASK)

/* CAN MB TX codes */
#define CAN_TXMB_INACTIVE          0x8        /* MB is not active. */
#define CAN_TXMB_ABORT             0x9        /* MB is aborted. */
#define CAN_TXMB_DATAORREMOTE      0xC        /* MB is a TX Data Frame(when MB RTR = 0) or */
                                              /* MB is a TX Remote Request Frame (when MB RTR = 1). */
#define CAN_TXMB_TANSWER           0xE        /* MB is a TX Response Request Frame from */
                                              /* an incoming Remote Request Frame. */
#define CAN_TXMB_NOTUSED           0xF        /* Not used.*/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_IMX9_HARDWARE_IMX9_FLEXCAN_H */
