/****************************************************************************
 * arch/arm/src/lpc214x/lpc214x_pinsel.h
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

#ifndef __ARCH_ARM_SRC_LPC214X_LPC214X_PINSEL_H
#define __ARCH_ARM_SRC_LPC214X_LPC214X_PINSEL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register address definitions *********************************************/

#define LPC214X_PINSEL0               (LPC214X_PINSEL_BASE + LPC214X_PINSEL0_OFFSET)
#define LPC214X_PINSEL1               (LPC214X_PINSEL_BASE + LPC214X_PINSEL1_OFFSET)
#define LPC214X_PINSEL2               (LPC214X_PINSEL_BASE + LPC214X_PINSEL2_OFFSET)

/* Register bit definitions *************************************************/

#define LPC214X_PINSEL0_P00_GPIO      (0x00000000)
#define LPC214X_PINSEL0_P00_TXD0      (0x00000001)
#define LPC214X_PINSEL0_P00_PWM1      (0x00000002)
#define LPC214X_PINSEL0_P00_RSVD3     (0x00000003)
#define LPC214X_PINSEL0_P00_MASK      (0x00000003)

#define LPC214X_PINSEL0_P01_GPIO      (0x00000000)
#define LPC214X_PINSEL0_P01_RXD0      (0x00000004)
#define LPC214X_PINSEL0_P01_PWM3      (0x00000008)
#define LPC214X_PINSEL0_P01_EINT0     (0x0000000c)
#define LPC214X_PINSEL0_P01_MASK      (0x0000000c)

#define LPC214X_PINSEL0_P02_GPIO      (0x00000000)
#define LPC214X_PINSEL0_P02_SCL0      (0x00000010)
#define LPC214X_PINSEL0_P02_CAP00     (0x00000020)
#define LPC214X_PINSEL0_P02_RSVD3     (0x00000030)
#define LPC214X_PINSEL0_P02_MASK      (0x00000030)

#define LPC214X_PINSEL0_P03_GPIO      (0x00000000)
#define LPC214X_PINSEL0_P03_SDA0      (0x00000040)
#define LPC214X_PINSEL0_P03_MAT00     (0x00000080)
#define LPC214X_PINSEL0_P03_EINT1     (0x000000c0)
#define LPC214X_PINSEL0_P03_MASK      (0x000000c0)

#define LPC214X_PINSEL0_P04_GPIO      (0x00000000)
#define LPC214X_PINSEL0_P04_SCK0      (0x00000100)
#define LPC214X_PINSEL0_P04_CAP01     (0x00000200)
#define LPC214X_PINSEL0_P04_RSVD3     (0x00000300)
#define LPC214X_PINSEL0_P04_MASK      (0x00000300)

#define LPC214X_PINSEL0_P05_GPIO      (0x00000000)
#define LPC214X_PINSEL0_P05_MISO0     (0x00000400)
#define LPC214X_PINSEL0_P05_MAT01     (0x00000800)
#define LPC214X_PINSEL0_P05_AD06      (0x00000c00)
#define LPC214X_PINSEL0_P05_MASK      (0x00000c00)

#define LPC214X_PINSEL0_P06_GPIO      (0x00000000)
#define LPC214X_PINSEL0_P06_MOSI0     (0x00001000)
#define LPC214X_PINSEL0_P06_CAP02     (0x00002000)
#define LPC214X_PINSEL0_P06_AD10      (0x00003000)
#define LPC214X_PINSEL0_P06_MASK      (0x00003000)

#define LPC214X_PINSEL0_P07_GPIO      (0x00000000)
#define LPC214X_PINSEL0_P07_SSEL0     (0x00004000)
#define LPC214X_PINSEL0_P07_PWM2      (0x00008000)
#define LPC214X_PINSEL0_P07_EINT2     (0x0000c000)
#define LPC214X_PINSEL0_P07_MASK      (0x0000c000)

#define LPC214X_PINSEL0_P08_GPIO      (0x00000000)
#define LPC214X_PINSEL0_P08_TXD1      (0x00010000)
#define LPC214X_PINSEL0_P08_PWM4      (0x00020000)
#define LPC214X_PINSEL0_P08_AD11      (0x00030000)
#define LPC214X_PINSEL0_P08_MASK      (0x00030000)

#define LPC214X_PINSEL0_P09_GPIO      (0x00000000)
#define LPC214X_PINSEL0_P09_RXD1      (0x00040000)
#define LPC214X_PINSEL0_P09_PWM6      (0x00080000)
#define LPC214X_PINSEL0_P09_EINT3     (0x000c0000)
#define LPC214X_PINSEL0_P09_MASK      (0x000c0000)

#define LPC214X_PINSEL0_P010_GPIO     (0x00000000)
#define LPC214X_PINSEL0_P010_RTS1     (0x00100000)
#define LPC214X_PINSEL0_P010_CAP10    (0x00200000)
#define LPC214X_PINSEL0_P010_AD12     (0x00300000)
#define LPC214X_PINSEL0_P010_MASK     (0x00300000)

#define LPC214X_PINSEL0_P011_GPIO     (0x00000000)
#define LPC214X_PINSEL0_P011_CTS1     (0x00400000)
#define LPC214X_PINSEL0_P011_CAP11    (0x00800000)
#define LPC214X_PINSEL0_P011_SCL1     (0x00c00000)
#define LPC214X_PINSEL0_P011_MASK     (0x00c00000)

#define LPC214X_PINSEL0_P012_GPIO     (0x00000000)
#define LPC214X_PINSEL0_P012_DSR1     (0x01000000)
#define LPC214X_PINSEL0_P012_MAT10    (0x02000000)
#define LPC214X_PINSEL0_P012_AD13     (0x03000000)
#define LPC214X_PINSEL0_P012_MASK     (0x03000000)

#define LPC214X_PINSEL0_P013_GPIO     (0x00000000)
#define LPC214X_PINSEL0_P013_DTR1     (0x04000000)
#define LPC214X_PINSEL0_P013_MAT11    (0x08000000)
#define LPC214X_PINSEL0_P013_AD14     (0x0c000000)
#define LPC214X_PINSEL0_P013_MASK     (0x0c000000)

#define LPC214X_PINSEL0_P014_GPIO     (0x00000000)
#define LPC214X_PINSEL0_P014_DCD1     (0x10000000)
#define LPC214X_PINSEL0_P014_EINT1    (0x20000000)
#define LPC214X_PINSEL0_P014_SDA1     (0x30000000)
#define LPC214X_PINSEL0_P014_MASK     (0x30000000)

#define LPC214X_PINSEL0_P015_GPIO     (0x00000000)
#define LPC214X_PINSEL0_P015_RI1      (0x40000000)
#define LPC214X_PINSEL0_P015_EINT2    (0x80000000)
#define LPC214X_PINSEL0_P015_AD15     (0xc0000000)
#define LPC214X_PINSEL0_P015_MASK     (0xc0000000)

#define LPC214X_PINSEL1_P016_GPIO     (0x00000000)
#define LPC214X_PINSEL1_P016_EINT0    (0x00000001)
#define LPC214X_PINSEL1_P016_MAT02    (0x00000002)
#define LPC214X_PINSEL1_P016_CAP02    (0x00000003)
#define LPC214X_PINSEL1_P016_MASK     (0x00000003)

#define LPC214X_PINSEL1_P017_GPIO     (0x00000000)
#define LPC214X_PINSEL1_P017_CAP12    (0x00000004)
#define LPC214X_PINSEL1_P017_SCK1     (0x00000008)
#define LPC214X_PINSEL1_P017_MAT12    (0x0000000c)
#define LPC214X_PINSEL1_P017_MASK     (0x0000000c)

#define LPC214X_PINSEL1_P018_GPIO     (0x00000000)
#define LPC214X_PINSEL1_P018_CAP13    (0x00000010)
#define LPC214X_PINSEL1_P018_MISO1    (0x00000020)
#define LPC214X_PINSEL1_P018_MAT13    (0x00000030)
#define LPC214X_PINSEL1_P018_MASK     (0x00000030)

#define LPC214X_PINSEL1_P019_GPIO     (0x00000000)
#define LPC214X_PINSEL1_P019_MAT12    (0x00000040)
#define LPC214X_PINSEL1_P019_MOSI1    (0x00000080)
#define LPC214X_PINSEL1_P019_CAP12    (0x000000c0)
#define LPC214X_PINSEL1_P019_MASK     (0x000000c0)

#define LPC214X_PINSEL1_P020_GPIO     (0x00000000)
#define LPC214X_PINSEL1_P020_MAT13    (0x00000100)
#define LPC214X_PINSEL1_P020_SSEL1    (0x00000200)
#define LPC214X_PINSEL1_P020_EINT3    (0x00000300)
#define LPC214X_PINSEL1_P020_MASK     (0x00000300)

#define LPC214X_PINSEL1_P021_GPIO     (0x00000000)
#define LPC214X_PINSEL1_P021_PWM5     (0x00000400)
#define LPC214X_PINSEL1_P021_AD16     (0x00000800)
#define LPC214X_PINSEL1_P021_CAP13    (0x00000c00)
#define LPC214X_PINSEL1_P021_MASK     (0x00000c00)

#define LPC214X_PINSEL1_P022_GPIO     (0x00000000)
#define LPC214X_PINSEL1_P022_AD17     (0x00001000)
#define LPC214X_PINSEL1_P022_CAP00    (0x00002000)
#define LPC214X_PINSEL1_P022_MAT00    (0x00003000)
#define LPC214X_PINSEL1_P022_MASK     (0x00003000)

#define LPC214X_PINSEL1_P023_GPIO     (0x00000000)
#define LPC214X_PINSEL1_P023_VBUS     (0x00004000)
#define LPC214X_PINSEL1_P023_RSVD2    (0x00008000)
#define LPC214X_PINSEL1_P023_RSVD3    (0x0000c000)
#define LPC214X_PINSEL1_P023_MASK     (0x0000c000)

#define LPC214X_PINSEL1_P024_RSVD0    (0x00000000)
#define LPC214X_PINSEL1_P024_RSVD1    (0x00010000)
#define LPC214X_PINSEL1_P024_RSVD2    (0x00020000)
#define LPC214X_PINSEL1_P024_RSVD3    (0x00030000)
#define LPC214X_PINSEL1_P024_MASK     (0x00030000)

#define LPC214X_PINSEL1_P025_GPIO     (0x00000000)
#define LPC214X_PINSEL1_P025_AD04     (0x00040000)
#define LPC214X_PINSEL1_P025_AOUT     (0x00080000)
#define LPC214X_PINSEL1_P025_RSVD3    (0x000c0000)
#define LPC214X_PINSEL1_P025_MASK     (0x000c0000)

#define LPC214X_PINSEL1_P026_RSVD0    (0x00000000)
#define LPC214X_PINSEL1_P026_RSVD1    (0x00100000)
#define LPC214X_PINSEL1_P026_RSVD2    (0x00200000)
#define LPC214X_PINSEL1_P026_RSVD3    (0x00300000)
#define LPC214X_PINSEL1_P026_MASK     (0x00300000)

#define LPC214X_PINSEL1_P027_RSVD0    (0x00000000)
#define LPC214X_PINSEL1_P027_RSVD1    (0x00400000)
#define LPC214X_PINSEL1_P027_RSVD2    (0x00800000)
#define LPC214X_PINSEL1_P027_RSVD3    (0x00c00000)
#define LPC214X_PINSEL1_P027_MASK     (0x00c00000)

#define LPC214X_PINSEL1_P028_GPIO     (0x00000000)
#define LPC214X_PINSEL1_P028_AD01     (0x01000000)
#define LPC214X_PINSEL1_P028_CAP02    (0x02000000)
#define LPC214X_PINSEL1_P028_MAT02    (0x03000000)
#define LPC214X_PINSEL1_P028_MASK     (0x03000000)

#define LPC214X_PINSEL1_P029_GPIO     (0x00000000)
#define LPC214X_PINSEL1_P029_AD02     (0x04000000)
#define LPC214X_PINSEL1_P029_CAP03    (0x08000000)
#define LPC214X_PINSEL1_P029_MAT03    (0x0c000000)
#define LPC214X_PINSEL1_P029_MASK     (0x0c000000)

#define LPC214X_PINSEL1_P030_GPIO     (0x00000000)
#define LPC214X_PINSEL1_P030_AD03     (0x10000000)
#define LPC214X_PINSEL1_P030_EINT3    (0x20000000)
#define LPC214X_PINSEL1_P030_CAP00    (0x30000000)
#define LPC214X_PINSEL1_P030_MASK     (0x30000000)

#define LPC214X_PINSEL1_P031_GPIO     (0x00000000)
#define LPC214X_PINSEL1_P031_UPLED    (0x40000000)
#define LPC214X_PINSEL1_P031_CONNECT  (0x80000000)
#define LPC214X_PINSEL1_P031_RSVD3    (0xc0000000)
#define LPC214X_PINSEL1_P031_MASK     (0xc0000000)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC214X_LPC214X_PINSEL_H */
