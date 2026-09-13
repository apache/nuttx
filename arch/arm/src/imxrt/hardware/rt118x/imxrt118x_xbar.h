/****************************************************************************
 * arch/arm/src/imxrt/hardware/rt118x/imxrt118x_xbar.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_XBAR_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_XBAR_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Inter-Peripheral Crossbar Switch (XBAR) for i.MX RT1180 (IMXRT1180RM
 * Ch. 78).  Three instances are present:
 *
 *   XBAR1  base 0x4275_0000  216 inputs, 221 outputs (SEL0..SEL110)
 *   XBAR2  base 0x4276_0000  178 inputs,  32 outputs (SEL0..SEL15)
 *   XBAR3  base 0x4277_0000  178 inputs,  32 outputs (SEL0..SEL15)
 *
 * All three instances use full 8-bit select fields.
 */

#define IMXRT_XBAR_SEL_MASKS  {0xff, 0xff, 0xff}

/* IMXRT_XBARn_BASE constants live in the RT1180 memory map
 * (hardware/rt118x/imxrt118x_memorymap.h).
 */

/****************************************************************************
 * XBAR Input Assignments (IMXRT1180RM Table 14)
 ****************************************************************************/

#define IMXRT_XBAR1_IN_GND                                                      IMXRT_XBAR1(XBAR_INPUT, 0)      /* XBAR1_IN0 <- -  GND */
#define IMXRT_XBAR1_IN_1B1                                                      IMXRT_XBAR1(XBAR_INPUT, 1)      /* XBAR1_IN1 <- -  1'B1 */
#define IMXRT_XBAR1_IN_GND_2                                                    IMXRT_XBAR1(XBAR_INPUT, 2)      /* XBAR1_IN2 <- -  GND */
#define IMXRT_XBAR1_IN_1B1_2                                                    IMXRT_XBAR1(XBAR_INPUT, 3)      /* XBAR1_IN3 <- -  1'B1 */
#define IMXRT_XBAR1_IN_IOMUX_XBAR_INOUT04                                       IMXRT_XBAR1(XBAR_INPUT, 4)      /* XBAR1_IN4 <- -  SW_MUX_CTL_PAD_GPIO_EMC_B1_ 00[MUX_MODE] = 0x1 */
#define IMXRT_XBAR1_IN_IOMUX_XBAR_INOUT05                                       IMXRT_XBAR1(XBAR_INPUT, 5)      /* XBAR1_IN5 <- -  SW_MUX_CTL_PAD_GPIO_EMC_B1_ 01[MUX_MODE] = 0x1 */
#define IMXRT_XBAR1_IN_IOMUX_XBAR_INOUT06                                       IMXRT_XBAR1(XBAR_INPUT, 6)      /* XBAR1_IN6 <- -  SW_MUX_CTL_PAD_GPIO_EMC_B1_ 02[MUX_MODE] = 0x1 */
#define IMXRT_XBAR1_IN_IOMUX_XBAR_INOUT07                                       IMXRT_XBAR1(XBAR_INPUT, 7)      /* XBAR1_IN7 <- -  SW_MUX_CTL_PAD_GPIO_EMC_B1_ 03[MUX_MODE] = 0x1 */
#define IMXRT_XBAR1_IN_IOMUX_XBAR_INOUT08                                       IMXRT_XBAR1(XBAR_INPUT, 8)      /* XBAR1_IN8 <- -  SW_MUX_CTL_PAD_GPIO_EMC_B1_ 04[MUX_MODE] = 0x1 */
#define IMXRT_XBAR1_IN_IOMUX_XBAR_INOUT09                                       IMXRT_XBAR1(XBAR_INPUT, 9)      /* XBAR1_IN9 <- -  SW_MUX_CTL_PAD_GPIO_EMC_B1_ 05[MUX_MODE] = 0x1 */
#define IMXRT_XBAR1_IN_IOMUX_XBAR_INOUT10                                       IMXRT_XBAR1(XBAR_INPUT, 10)     /* XBAR1_IN10 <- -  SW_MUX_CTL_PAD_GPIO_EMC_B1_ 26[MUX_MODE] = 0x2 */
#define IMXRT_XBAR1_IN_IOMUX_XBAR_INOUT11                                       IMXRT_XBAR1(XBAR_INPUT, 11)     /* XBAR1_IN11 <- -  SW_MUX_CTL_PAD_GPIO_EMC_B1_ 27[MUX_MODE] = 0x2 */
#define IMXRT_XBAR1_IN_IOMUX_XBAR_INOUT12                                       IMXRT_XBAR1(XBAR_INPUT, 12)     /* XBAR1_IN12 <- -  SW_MUX_CTL_PAD_GPIO_EMC_B1_ 28[MUX_MODE] = 0x2 */
#define IMXRT_XBAR1_IN_IOMUX_XBAR_INOUT13                                       IMXRT_XBAR1(XBAR_INPUT, 13)     /* XBAR1_IN13 <- -  SW_MUX_CTL_PAD_GPIO_EMC_B1_ 29[MUX_MODE] = 0x2 */
#define IMXRT_XBAR1_IN_IOMUX_XBAR_INOUT14                                       IMXRT_XBAR1(XBAR_INPUT, 14)     /* XBAR1_IN14 <- -  SW_MUX_CTL_PAD_GPIO_EMC_B2_ 16[MUX_MODE] = 0x6 */
#define IMXRT_XBAR1_IN_IOMUX_XBAR_INOUT15                                       IMXRT_XBAR1(XBAR_INPUT, 15)     /* XBAR1_IN15 <- -  SW_MUX_CTL_PAD_GPIO_EMC_B2_ 17[MUX_MODE] = 0x6 */
#define IMXRT_XBAR1_IN_IOMUX_XBAR_INOUT16                                       IMXRT_XBAR1(XBAR_INPUT, 16)     /* XBAR1_IN16 <- -  SW_MUX_CTL_PAD_GPIO_EMC_B2_ 18[MUX_MODE] = 0x6 */
#define IMXRT_XBAR1_IN_IOMUX_XBAR_INOUT17                                       IMXRT_XBAR1(XBAR_INPUT, 17)     /* XBAR1_IN17 <- -  SW_MUX_CTL_PAD_GPIO_AD_33[MU See IOMUX Controller X_MODE] = */
#define IMXRT_XBAR1_IN_IOMUX_XBAR_INOUT18                                       IMXRT_XBAR1(XBAR_INPUT, 18)     /* XBAR1_IN18 <- -  SW_MUX_CTL_PAD_GPIO_AD_12[MU See IOMUX Controller X_MODE] = */
#define IMXRT_XBAR1_IN_IOMUX_XBAR_INOUT19                                       IMXRT_XBAR1(XBAR_INPUT, 19)     /* XBAR1_IN19 <- -  SW_MUX_CTL_PAD_GPIO_AD_19[MU See IOMUX Controller X_MODE] = */
#define IMXRT_XBAR1_IN_IOMUX_XBAR_INOUT20                                       IMXRT_XBAR1(XBAR_INPUT, 20)     /* XBAR1_IN20 <- -  SW_MUX_CTL_PAD_GPIO_EMC_B2_ 00[MUX_MODE] = 0x6 */
#define IMXRT_XBAR1_IN_IOMUX_XBAR_INOUT21                                       IMXRT_XBAR1(XBAR_INPUT, 21)     /* XBAR1_IN21 <- -  SW_MUX_CTL_PAD_GPIO_EMC_B2_ 01[MUX_MODE] = 0x6 */
#define IMXRT_XBAR1_IN_IOMUX_XBAR_INOUT22                                       IMXRT_XBAR1(XBAR_INPUT, 22)     /* XBAR1_IN22 <- -  SW_MUX_CTL_PAD_GPIO_EMC_B2_ 02[MUX_MODE] = 0x6 */
#define IMXRT_XBAR1_IN_IOMUX_XBAR_INOUT23                                       IMXRT_XBAR1(XBAR_INPUT, 23)     /* XBAR1_IN23 <- -  SW_MUX_CTL_PAD_GPIO_EMC_B2_ 03[MUX_MODE] = 0x6 */
#define IMXRT_XBAR1_IN_IOMUX_XBAR_INOUT24                                       IMXRT_XBAR1(XBAR_INPUT, 24)     /* XBAR1_IN24 <- -  SW_MUX_CTL_PAD_GPIO_EMC_B2_ 04[MUX_MODE] = 0x6 */
#define IMXRT_XBAR1_IN_IOMUX_XBAR_INOUT25                                       IMXRT_XBAR1(XBAR_INPUT, 25)     /* XBAR1_IN25 <- -  SW_MUX_CTL_PAD_GPIO_EMC_B2_ 05[MUX_MODE] = 0x6 */
#define IMXRT_XBAR1_IN_IOMUX_XBAR_INOUT26                                       IMXRT_XBAR1(XBAR_INPUT, 26)     /* XBAR1_IN26 <- -  SW_MUX_CTL_PAD_GPIO_EMC_B2_ 06[MUX_MODE] = 0x6 */
#define IMXRT_XBAR1_IN_IOMUX_XBAR_INOUT27                                       IMXRT_XBAR1(XBAR_INPUT, 27)     /* XBAR1_IN27 <- -  SW_MUX_CTL_PAD_GPIO_EMC_B2_ 07[MUX_MODE] = 0x6 */
#define IMXRT_XBAR1_IN_IOMUX_XBAR_INOUT28                                       IMXRT_XBAR1(XBAR_INPUT, 28)     /* XBAR1_IN28 <- -  SW_MUX_CTL_PAD_GPIO_EMC_B2_ 08[MUX_MODE] = 0x6 */
#define IMXRT_XBAR1_IN_IOMUX_XBAR_INOUT29                                       IMXRT_XBAR1(XBAR_INPUT, 29)     /* XBAR1_IN29 <- -  SW_MUX_CTL_PAD_GPIO_EMC_B2_ 09[MUX_MODE] = 0x6 */
#define IMXRT_XBAR1_IN_IOMUX_XBAR_INOUT30                                       IMXRT_XBAR1(XBAR_INPUT, 30)     /* XBAR1_IN30 <- -  SW_MUX_CTL_PAD_GPIO_EMC_B2_ 10[MUX_MODE] = 0x6 */
#define IMXRT_XBAR1_IN_IOMUX_XBAR_INOUT31                                       IMXRT_XBAR1(XBAR_INPUT, 31)     /* XBAR1_IN31 <- -  SW_MUX_CTL_PAD_GPIO_EMC_B2_ 11[MUX_MODE] = 0x6 */
#define IMXRT_XBAR1_IN_IOMUX_XBAR_INOUT32                                       IMXRT_XBAR1(XBAR_INPUT, 32)     /* XBAR1_IN32 <- -  SW_MUX_CTL_PAD_GPIO_EMC_B2_ 12[MUX_MODE] = 0x6 */
#define IMXRT_XBAR1_IN_IOMUX_XBAR_INOUT33                                       IMXRT_XBAR1(XBAR_INPUT, 33)     /* XBAR1_IN33 <- -  SW_MUX_CTL_PAD_GPIO_EMC_B2_ 13[MUX_MODE] = 0x6 */
#define IMXRT_XBAR1_IN_IOMUX_XBAR_INOUT34                                       IMXRT_XBAR1(XBAR_INPUT, 34)     /* XBAR1_IN34 <- -  SW_MUX_CTL_PAD_GPIO_EMC_B2_ 14[MUX_MODE] = 0x6 */
#define IMXRT_XBAR1_IN_IOMUX_XBAR_INOUT35                                       IMXRT_XBAR1(XBAR_INPUT, 35)     /* XBAR1_IN35 <- -  SW_MUX_CTL_PAD_GPIO_EMC_B2_ 15[MUX_MODE] = 0x6 */
#define IMXRT_XBAR1_IN_IOMUX_XBAR_INOUT36                                       IMXRT_XBAR1(XBAR_INPUT, 36)     /* XBAR1_IN36 <- -  SW_MUX_CTL_PAD_GPIO_EMC_B2_ 19[MUX_MODE] = 0x6 */
#define IMXRT_XBAR1_IN_IOMUX_XBAR_INOUT37                                       IMXRT_XBAR1(XBAR_INPUT, 37)     /* XBAR1_IN37 <- -  SW_MUX_CTL_PAD_GPIO_EMC_B2_ 20[MUX_MODE] = 0x6 */
#define IMXRT_XBAR1_IN_CMP1_COUT                                                IMXRT_XBAR1(XBAR_INPUT, 38)     /* XBAR1_IN38 <- CMP1  COUT */
#define IMXRT_XBAR1_IN_CMP2_COUT                                                IMXRT_XBAR1(XBAR_INPUT, 39)     /* XBAR1_IN39 <- CMP2  COUT */
#define IMXRT_XBAR1_IN_CMP3_COUT                                                IMXRT_XBAR1(XBAR_INPUT, 40)     /* XBAR1_IN40 <- CMP3  COUT */
#define IMXRT_XBAR1_IN_CMP4_COUT                                                IMXRT_XBAR1(XBAR_INPUT, 41)     /* XBAR1_IN41 <- CMP4  COUT */
#define IMXRT_XBAR1_IN_TMR1_TMR0_OUTPUT                                         IMXRT_XBAR1(XBAR_INPUT, 42)     /* XBAR1_IN42 <- TMR1  TMR0_OUTPUT */
#define IMXRT_XBAR1_IN_TMR1_TMR1_OUTPUT                                         IMXRT_XBAR1(XBAR_INPUT, 43)     /* XBAR1_IN43 <- TMR1  TMR1_OUTPUT */
#define IMXRT_XBAR1_IN_TMR1_TMR2_OUTPUT                                         IMXRT_XBAR1(XBAR_INPUT, 44)     /* XBAR1_IN44 <- TMR1  TMR2_OUTPUT */
#define IMXRT_XBAR1_IN_TMR1_TMR3_OUTPUT                                         IMXRT_XBAR1(XBAR_INPUT, 45)     /* XBAR1_IN45 <- TMR1  TMR3_OUTPUT */
#define IMXRT_XBAR1_IN_TMR2_TMR0_OUTPUT                                         IMXRT_XBAR1(XBAR_INPUT, 46)     /* XBAR1_IN46 <- TMR2  TMR0_OUTPUT */
#define IMXRT_XBAR1_IN_TMR2_TMR1_OUTPUT                                         IMXRT_XBAR1(XBAR_INPUT, 47)     /* XBAR1_IN47 <- TMR2  TMR1_OUTPUT */
#define IMXRT_XBAR1_IN_TMR2_TMR2_OUTPUT                                         IMXRT_XBAR1(XBAR_INPUT, 48)     /* XBAR1_IN48 <- TMR2  TMR2_OUTPUT */
#define IMXRT_XBAR1_IN_TMR2_TMR3_OUTPUT                                         IMXRT_XBAR1(XBAR_INPUT, 49)     /* XBAR1_IN49 <- TMR2  TMR3_OUTPUT */
#define IMXRT_XBAR1_IN_TMR3_TMR0_OUTPUT                                         IMXRT_XBAR1(XBAR_INPUT, 50)     /* XBAR1_IN50 <- TMR3  TMR0_OUTPUT */
#define IMXRT_XBAR1_IN_TMR3_TMR1_OUTPUT                                         IMXRT_XBAR1(XBAR_INPUT, 51)     /* XBAR1_IN51 <- TMR3  TMR1_OUTPUT */
#define IMXRT_XBAR1_IN_TMR3_TMR2_OUTPUT                                         IMXRT_XBAR1(XBAR_INPUT, 52)     /* XBAR1_IN52 <- TMR3  TMR2_OUTPUT */
#define IMXRT_XBAR1_IN_TMR3_TMR3_OUTPUT                                         IMXRT_XBAR1(XBAR_INPUT, 53)     /* XBAR1_IN53 <- TMR3  TMR3_OUTPUT */
#define IMXRT_XBAR1_IN_TMR4_TMR0_OUTPUT                                         IMXRT_XBAR1(XBAR_INPUT, 54)     /* XBAR1_IN54 <- TMR4  TMR0_OUTPUT */
#define IMXRT_XBAR1_IN_TMR4_TMR1_OUTPUT                                         IMXRT_XBAR1(XBAR_INPUT, 55)     /* XBAR1_IN55 <- TMR4  TMR1_OUTPUT */
#define IMXRT_XBAR1_IN_TMR4_TMR2_OUTPUT                                         IMXRT_XBAR1(XBAR_INPUT, 56)     /* XBAR1_IN56 <- TMR4  TMR2_OUTPUT */
#define IMXRT_XBAR1_IN_TMR4_TMR3_OUTPUT                                         IMXRT_XBAR1(XBAR_INPUT, 57)     /* XBAR1_IN57 <- TMR4  TMR3_OUTPUT */
#define IMXRT_XBAR1_IN_TMR5_TMR0_OUTPUT                                         IMXRT_XBAR1(XBAR_INPUT, 58)     /* XBAR1_IN58 <- TMR5  TMR0_OUTPUT */
#define IMXRT_XBAR1_IN_TMR5_TMR1_OUTPUT                                         IMXRT_XBAR1(XBAR_INPUT, 59)     /* XBAR1_IN59 <- TMR5  TMR1_OUTPUT */
#define IMXRT_XBAR1_IN_TMR5_TMR2_OUTPUT                                         IMXRT_XBAR1(XBAR_INPUT, 60)     /* XBAR1_IN60 <- TMR5  TMR2_OUTPUT */
#define IMXRT_XBAR1_IN_TMR5_TMR3_OUTPUT                                         IMXRT_XBAR1(XBAR_INPUT, 61)     /* XBAR1_IN61 <- TMR5  TMR3_OUTPUT */
#define IMXRT_XBAR1_IN_TMR6_TMR0_OUTPUT                                         IMXRT_XBAR1(XBAR_INPUT, 62)     /* XBAR1_IN62 <- TMR6  TMR0_OUTPUT */
#define IMXRT_XBAR1_IN_TMR6_TMR1_OUTPUT                                         IMXRT_XBAR1(XBAR_INPUT, 63)     /* XBAR1_IN63 <- TMR6  TMR1_OUTPUT */
#define IMXRT_XBAR1_IN_TMR6_TMR2_OUTPUT                                         IMXRT_XBAR1(XBAR_INPUT, 64)     /* XBAR1_IN64 <- TMR6  TMR2_OUTPUT */
#define IMXRT_XBAR1_IN_TMR6_TMR3_OUTPUT                                         IMXRT_XBAR1(XBAR_INPUT, 65)     /* XBAR1_IN65 <- TMR6  TMR3_OUTPUT */
#define IMXRT_XBAR1_IN_TMR7_TMR0_OUTPUT                                         IMXRT_XBAR1(XBAR_INPUT, 66)     /* XBAR1_IN66 <- TMR7  TMR0_OUTPUT */
#define IMXRT_XBAR1_IN_TMR7_TMR1_OUTPUT                                         IMXRT_XBAR1(XBAR_INPUT, 67)     /* XBAR1_IN67 <- TMR7  TMR1_OUTPUT */
#define IMXRT_XBAR1_IN_TMR7_TMR2_OUTPUT                                         IMXRT_XBAR1(XBAR_INPUT, 68)     /* XBAR1_IN68 <- TMR7  TMR2_OUTPUT */
#define IMXRT_XBAR1_IN_TMR7_TMR3_OUTPUT                                         IMXRT_XBAR1(XBAR_INPUT, 69)     /* XBAR1_IN69 <- TMR7  TMR3_OUTPUT */
#define IMXRT_XBAR1_IN_TMR8_TMR0_OUTPUT                                         IMXRT_XBAR1(XBAR_INPUT, 70)     /* XBAR1_IN70 <- TMR8  TMR0_OUTPUT */
#define IMXRT_XBAR1_IN_TMR8_TMR1_OUTPUT                                         IMXRT_XBAR1(XBAR_INPUT, 71)     /* XBAR1_IN71 <- TMR8  TMR1_OUTPUT */
#define IMXRT_XBAR1_IN_TMR8_TMR2_OUTPUT                                         IMXRT_XBAR1(XBAR_INPUT, 72)     /* XBAR1_IN72 <- TMR8  TMR2_OUTPUT */
#define IMXRT_XBAR1_IN_TMR8_TMR3_OUTPUT                                         IMXRT_XBAR1(XBAR_INPUT, 73)     /* XBAR1_IN73 <- TMR8  TMR3_OUTPUT */
#define IMXRT_XBAR1_IN_FLEXPWM1_PWM0_MUX_TRIGGER0                               IMXRT_XBAR1(XBAR_INPUT, 74)     /* XBAR1_IN74 <- FLEXPWM1  PWM0_MUX_TRIGGER0 */
#define IMXRT_XBAR1_IN_FLEXPWM1_PWM0_MUX_TRIGGER1                               IMXRT_XBAR1(XBAR_INPUT, 75)     /* XBAR1_IN75 <- FLEXPWM1  PWM0_MUX_TRIGGER1 */
#define IMXRT_XBAR1_IN_FLEXPWM1_PWM1_MUX_TRIGGER0                               IMXRT_XBAR1(XBAR_INPUT, 76)     /* XBAR1_IN76 <- FLEXPWM1  PWM1_MUX_TRIGGER0 */
#define IMXRT_XBAR1_IN_FLEXPWM1_PWM1_MUX_TRIGGER1                               IMXRT_XBAR1(XBAR_INPUT, 77)     /* XBAR1_IN77 <- FLEXPWM1  PWM1_MUX_TRIGGER1 */
#define IMXRT_XBAR1_IN_FLEXPWM1_PWM2_MUX_TRIGGER0                               IMXRT_XBAR1(XBAR_INPUT, 78)     /* XBAR1_IN78 <- FLEXPWM1  PWM2_MUX_TRIGGER0 */
#define IMXRT_XBAR1_IN_FLEXPWM1_PWM2_MUX_TRIGGER1                               IMXRT_XBAR1(XBAR_INPUT, 79)     /* XBAR1_IN79 <- FLEXPWM1  PWM2_MUX_TRIGGER1 */
#define IMXRT_XBAR1_IN_FLEXPWM1_PWM3_MUX_TRIGGER0                               IMXRT_XBAR1(XBAR_INPUT, 80)     /* XBAR1_IN80 <- FLEXPWM1  PWM3_MUX_TRIGGER0 */
#define IMXRT_XBAR1_IN_FLEXPWM1_PWM3_MUX_TRIGGER1                               IMXRT_XBAR1(XBAR_INPUT, 81)     /* XBAR1_IN81 <- FLEXPWM1  PWM3_MUX_TRIGGER1 */
#define IMXRT_XBAR1_IN_FLEXPWM2_PWM0_MUX_TRIGGER0                               IMXRT_XBAR1(XBAR_INPUT, 82)     /* XBAR1_IN82 <- FLEXPWM2  PWM0_MUX_TRIGGER0 */
#define IMXRT_XBAR1_IN_FLEXPWM2_PWM1_MUX_TRIGGER0                               IMXRT_XBAR1(XBAR_INPUT, 83)     /* XBAR1_IN83 <- FLEXPWM2  PWM1_MUX_TRIGGER0 */
#define IMXRT_XBAR1_IN_FLEXPWM2_PWM2_MUX_TRIGGER0                               IMXRT_XBAR1(XBAR_INPUT, 84)     /* XBAR1_IN84 <- FLEXPWM2  PWM2_MUX_TRIGGER0 */
#define IMXRT_XBAR1_IN_FLEXPWM2_PWM3_MUX_TRIGGER0                               IMXRT_XBAR1(XBAR_INPUT, 85)     /* XBAR1_IN85 <- FLEXPWM2  PWM3_MUX_TRIGGER0 */
#define IMXRT_XBAR1_IN_FLEXPWM3_PWM0_MUX_TRIGGER0                               IMXRT_XBAR1(XBAR_INPUT, 86)     /* XBAR1_IN86 <- FLEXPWM3  PWM0_MUX_TRIGGER0 */
#define IMXRT_XBAR1_IN_FLEXPWM3_PWM1_MUX_TRIGGER0                               IMXRT_XBAR1(XBAR_INPUT, 87)     /* XBAR1_IN87 <- FLEXPWM3  PWM1_MUX_TRIGGER0 */
#define IMXRT_XBAR1_IN_FLEXPWM3_PWM2_MUX_TRIGGER0                               IMXRT_XBAR1(XBAR_INPUT, 88)     /* XBAR1_IN88 <- FLEXPWM3  PWM2_MUX_TRIGGER0 */
#define IMXRT_XBAR1_IN_FLEXPWM3_PWM3_MUX_TRIGGER0                               IMXRT_XBAR1(XBAR_INPUT, 89)     /* XBAR1_IN89 <- FLEXPWM3  PWM3_MUX_TRIGGER0 */
#define IMXRT_XBAR1_IN_FLEXPWM4_PWM0_MUX_TRIGGER0                               IMXRT_XBAR1(XBAR_INPUT, 90)     /* XBAR1_IN90 <- FLEXPWM4  PWM0_MUX_TRIGGER0 */
#define IMXRT_XBAR1_IN_FLEXPWM4_PWM1_MUX_TRIGGER0                               IMXRT_XBAR1(XBAR_INPUT, 91)     /* XBAR1_IN91 <- FLEXPWM4  PWM1_MUX_TRIGGER0 */
#define IMXRT_XBAR1_IN_FLEXPWM4_PWM2_MUX_TRIGGER0                               IMXRT_XBAR1(XBAR_INPUT, 92)     /* XBAR1_IN92 <- FLEXPWM4  PWM2_MUX_TRIGGER0 */
#define IMXRT_XBAR1_IN_FLEXPWM4_PWM3_MUX_TRIGGER0                               IMXRT_XBAR1(XBAR_INPUT, 93)     /* XBAR1_IN93 <- FLEXPWM4  PWM3_MUX_TRIGGER0 */
#define IMXRT_XBAR1_IN_LPIT1_LPIT_TRIG_OUT0                                     IMXRT_XBAR1(XBAR_INPUT, 94)     /* XBAR1_IN94 <- LPIT1  LPIT_TRIG_OUT0 */
#define IMXRT_XBAR1_IN_LPIT1_LPIT_TRIG_OUT1                                     IMXRT_XBAR1(XBAR_INPUT, 95)     /* XBAR1_IN95 <- LPIT1  LPIT_TRIG_OUT1 */
#define IMXRT_XBAR1_IN_LPIT1_LPIT_TRIG_OUT2                                     IMXRT_XBAR1(XBAR_INPUT, 96)     /* XBAR1_IN96 <- LPIT1  LPIT_TRIG_OUT2 */
#define IMXRT_XBAR1_IN_LPIT1_LPIT_TRIG_OUT3                                     IMXRT_XBAR1(XBAR_INPUT, 97)     /* XBAR1_IN97 <- LPIT1  LPIT_TRIG_OUT3 */
#define IMXRT_XBAR1_IN_LPIT2_LPIT_TRIG_OUT0                                     IMXRT_XBAR1(XBAR_INPUT, 98)     /* XBAR1_IN98 <- LPIT2  LPIT_TRIG_OUT0 */
#define IMXRT_XBAR1_IN_LPIT2_LPIT_TRIG_OUT1                                     IMXRT_XBAR1(XBAR_INPUT, 99)     /* XBAR1_IN99 <- LPIT2  LPIT_TRIG_OUT1 */
#define IMXRT_XBAR1_IN_LPIT2_LPIT_TRIG_OUT2                                     IMXRT_XBAR1(XBAR_INPUT, 100)    /* XBAR1_IN100 <- LPIT2  LPIT_TRIG_OUT2 */
#define IMXRT_XBAR1_IN_LPIT2_LPIT_TRIG_OUT3                                     IMXRT_XBAR1(XBAR_INPUT, 101)    /* XBAR1_IN101 <- LPIT2  LPIT_TRIG_OUT3 */
#define IMXRT_XBAR1_IN_LPIT3_LPIT_TRIG_OUT0                                     IMXRT_XBAR1(XBAR_INPUT, 102)    /* XBAR1_IN102 <- LPIT3  LPIT_TRIG_OUT0 */
#define IMXRT_XBAR1_IN_LPIT3_LPIT_TRIG_OUT1                                     IMXRT_XBAR1(XBAR_INPUT, 103)    /* XBAR1_IN103 <- LPIT3  LPIT_TRIG_OUT1 */
#define IMXRT_XBAR1_IN_LPIT3_LPIT_TRIG_OUT2                                     IMXRT_XBAR1(XBAR_INPUT, 104)    /* XBAR1_IN104 <- LPIT3  LPIT_TRIG_OUT2 */
#define IMXRT_XBAR1_IN_LPIT3_LPIT_TRIG_OUT3                                     IMXRT_XBAR1(XBAR_INPUT, 105)    /* XBAR1_IN105 <- LPIT3  LPIT_TRIG_OUT3 */
#define IMXRT_XBAR1_IN_TRIGGER_SY_TRIGGER_SYNC_OUT0_NC                          IMXRT_XBAR1(XBAR_INPUT, 106)    /* XBAR1_IN106 <- TRIGGER_SY TRIGGER_SYNC_OUT0  - NC */
#define IMXRT_XBAR1_IN_TRIGGER_SY_TRIGGER_SYNC_OUT1_NC                          IMXRT_XBAR1(XBAR_INPUT, 107)    /* XBAR1_IN107 <- TRIGGER_SY TRIGGER_SYNC_OUT1  - NC */
#define IMXRT_XBAR1_IN_TRIGGER_SY_TRIGGER_SYNC_OUT2_NC                          IMXRT_XBAR1(XBAR_INPUT, 108)    /* XBAR1_IN108 <- TRIGGER_SY TRIGGER_SYNC_OUT2  - NC */
#define IMXRT_XBAR1_IN_TRIGGER_SY_TRIGGER_SYNC_OUT3_NC                          IMXRT_XBAR1(XBAR_INPUT, 109)    /* XBAR1_IN109 <- TRIGGER_SY TRIGGER_SYNC_OUT3  - NC */
#define IMXRT_XBAR1_IN_EDMA4_DMA_TRIGGER_OUT0                                   IMXRT_XBAR1(XBAR_INPUT, 110)    /* XBAR1_IN110 <- eDMA4  DMA_TRIGGER_OUT0 */
#define IMXRT_XBAR1_IN_EDMA4_DMA_TRIGGER_OUT1                                   IMXRT_XBAR1(XBAR_INPUT, 111)    /* XBAR1_IN111 <- eDMA4  DMA_TRIGGER_OUT1 */
#define IMXRT_XBAR1_IN_EDMA4_DMA_TRIGGER_OUT2                                   IMXRT_XBAR1(XBAR_INPUT, 112)    /* XBAR1_IN112 <- eDMA4  DMA_TRIGGER_OUT2 */
#define IMXRT_XBAR1_IN_EDMA4_DMA_TRIGGER_OUT3                                   IMXRT_XBAR1(XBAR_INPUT, 113)    /* XBAR1_IN113 <- eDMA4  DMA_TRIGGER_OUT3 */
#define IMXRT_XBAR1_IN_EDMA4_DMA_TRIGGER_OUT4                                   IMXRT_XBAR1(XBAR_INPUT, 114)    /* XBAR1_IN114 <- eDMA4  DMA_TRIGGER_OUT4 */
#define IMXRT_XBAR1_IN_EDMA4_DMA_TRIGGER_OUT5                                   IMXRT_XBAR1(XBAR_INPUT, 115)    /* XBAR1_IN115 <- eDMA4  DMA_TRIGGER_OUT5 */
#define IMXRT_XBAR1_IN_EDMA4_DMA_TRIGGER_OUT6                                   IMXRT_XBAR1(XBAR_INPUT, 116)    /* XBAR1_IN116 <- eDMA4  DMA_TRIGGER_OUT6 */
#define IMXRT_XBAR1_IN_EDMA4_DMA_TRIGGER_OUT7                                   IMXRT_XBAR1(XBAR_INPUT, 117)    /* XBAR1_IN117 <- eDMA4  DMA_TRIGGER_OUT7 */
#define IMXRT_XBAR1_IN_EDMA3_DMA_TRIGGER_OUT0                                   IMXRT_XBAR1(XBAR_INPUT, 118)    /* XBAR1_IN118 <- eDMA3  DMA_TRIGGER_OUT0 */
#define IMXRT_XBAR1_IN_EDMA3_DMA_TRIGGER_OUT1                                   IMXRT_XBAR1(XBAR_INPUT, 119)    /* XBAR1_IN119 <- eDMA3  DMA_TRIGGER_OUT1 */
#define IMXRT_XBAR1_IN_EDMA3_DMA_TRIGGER_OUT2                                   IMXRT_XBAR1(XBAR_INPUT, 120)    /* XBAR1_IN120 <- eDMA3  DMA_TRIGGER_OUT2 */
#define IMXRT_XBAR1_IN_EDMA3_DMA_TRIGGER_OUT3                                   IMXRT_XBAR1(XBAR_INPUT, 121)    /* XBAR1_IN121 <- eDMA3  DMA_TRIGGER_OUT3 */
#define IMXRT_XBAR1_IN_EDMA3_DMA_TRIGGER_OUT4                                   IMXRT_XBAR1(XBAR_INPUT, 122)    /* XBAR1_IN122 <- eDMA3  DMA_TRIGGER_OUT4 */
#define IMXRT_XBAR1_IN_EDMA3_DMA_TRIGGER_OUT5                                   IMXRT_XBAR1(XBAR_INPUT, 123)    /* XBAR1_IN123 <- eDMA3  DMA_TRIGGER_OUT5 */
#define IMXRT_XBAR1_IN_EDMA3_DMA_TRIGGER_OUT6                                   IMXRT_XBAR1(XBAR_INPUT, 124)    /* XBAR1_IN124 <- eDMA3  DMA_TRIGGER_OUT6 */
#define IMXRT_XBAR1_IN_EDMA3_DMA_TRIGGER_OUT7                                   IMXRT_XBAR1(XBAR_INPUT, 125)    /* XBAR1_IN125 <- eDMA3  DMA_TRIGGER_OUT7 */
#define IMXRT_XBAR1_IN_ADC1_ADC_TCOMP_PULSE0                                    IMXRT_XBAR1(XBAR_INPUT, 126)    /* XBAR1_IN126 <- ADC1  ADC_TCOMP_PULSE0 */
#define IMXRT_XBAR1_IN_ADC1_ADC_TCOMP_PULSE1                                    IMXRT_XBAR1(XBAR_INPUT, 127)    /* XBAR1_IN127 <- ADC1  ADC_TCOMP_PULSE1 */
#define IMXRT_XBAR1_IN_ADC1_ADC_TCOMP_PULSE2                                    IMXRT_XBAR1(XBAR_INPUT, 128)    /* XBAR1_IN128 <- ADC1  ADC_TCOMP_PULSE2 */
#define IMXRT_XBAR1_IN_ADC1_ADC_TCOMP_PULSE3                                    IMXRT_XBAR1(XBAR_INPUT, 129)    /* XBAR1_IN129 <- ADC1  ADC_TCOMP_PULSE3 */
#define IMXRT_XBAR1_IN_ADC1_ADC_TCOMP_PULSE4                                    IMXRT_XBAR1(XBAR_INPUT, 130)    /* XBAR1_IN130 <- ADC1  ADC_TCOMP_PULSE4 */
#define IMXRT_XBAR1_IN_ADC1_ADC_TCOMP_PULSE5                                    IMXRT_XBAR1(XBAR_INPUT, 131)    /* XBAR1_IN131 <- ADC1  ADC_TCOMP_PULSE5 */
#define IMXRT_XBAR1_IN_ADC1_ADC_TCOMP_PULSE6                                    IMXRT_XBAR1(XBAR_INPUT, 132)    /* XBAR1_IN132 <- ADC1  ADC_TCOMP_PULSE6 */
#define IMXRT_XBAR1_IN_ADC1_ADC_TCOMP_PULSE7                                    IMXRT_XBAR1(XBAR_INPUT, 133)    /* XBAR1_IN133 <- ADC1  ADC_TCOMP_PULSE7 */
#define IMXRT_XBAR1_IN_ADC2_ADC_TCOMP_PULSE0                                    IMXRT_XBAR1(XBAR_INPUT, 134)    /* XBAR1_IN134 <- ADC2  ADC_TCOMP_PULSE0 */
#define IMXRT_XBAR1_IN_ADC2_ADC_TCOMP_PULSE1                                    IMXRT_XBAR1(XBAR_INPUT, 135)    /* XBAR1_IN135 <- ADC2  ADC_TCOMP_PULSE1 */
#define IMXRT_XBAR1_IN_ADC2_ADC_TCOMP_PULSE2                                    IMXRT_XBAR1(XBAR_INPUT, 136)    /* XBAR1_IN136 <- ADC2  ADC_TCOMP_PULSE2 */
#define IMXRT_XBAR1_IN_ADC2_ADC_TCOMP_PULSE3                                    IMXRT_XBAR1(XBAR_INPUT, 137)    /* XBAR1_IN137 <- ADC2  ADC_TCOMP_PULSE3 */
#define IMXRT_XBAR1_IN_ADC2_ADC_TCOMP_PULSE4                                    IMXRT_XBAR1(XBAR_INPUT, 138)    /* XBAR1_IN138 <- ADC2  ADC_TCOMP_PULSE4 */
#define IMXRT_XBAR1_IN_ADC2_ADC_TCOMP_PULSE5                                    IMXRT_XBAR1(XBAR_INPUT, 139)    /* XBAR1_IN139 <- ADC2  ADC_TCOMP_PULSE5 */
#define IMXRT_XBAR1_IN_ADC2_ADC_TCOMP_PULSE6                                    IMXRT_XBAR1(XBAR_INPUT, 140)    /* XBAR1_IN140 <- ADC2  ADC_TCOMP_PULSE6 */
#define IMXRT_XBAR1_IN_ADC2_ADC_TCOMP_PULSE7                                    IMXRT_XBAR1(XBAR_INPUT, 141)    /* XBAR1_IN141 <- ADC2  ADC_TCOMP_PULSE7 */
#define IMXRT_XBAR1_IN_TPM1_TPM_CH_TRIGGER0                                     IMXRT_XBAR1(XBAR_INPUT, 142)    /* XBAR1_IN142 <- TPM1  TPM_CH_TRIGGER0 */
#define IMXRT_XBAR1_IN_TPM1_TPM_CH_TRIGGER1                                     IMXRT_XBAR1(XBAR_INPUT, 143)    /* XBAR1_IN143 <- TPM1  TPM_CH_TRIGGER1 */
#define IMXRT_XBAR1_IN_TPM1_TPM_CH_TRIGGER2                                     IMXRT_XBAR1(XBAR_INPUT, 144)    /* XBAR1_IN144 <- TPM1  TPM_CH_TRIGGER2 */
#define IMXRT_XBAR1_IN_TPM1_TPM_CH_TRIGGER3                                     IMXRT_XBAR1(XBAR_INPUT, 145)    /* XBAR1_IN145 <- TPM1  TPM_CH_TRIGGER3 */
#define IMXRT_XBAR1_IN_TPM1_TPM_TRIGGER                                         IMXRT_XBAR1(XBAR_INPUT, 146)    /* XBAR1_IN146 <- TPM1  TPM_TRIGGER */
#define IMXRT_XBAR1_IN_TPM2_TPM_CH_TRIGGER0                                     IMXRT_XBAR1(XBAR_INPUT, 147)    /* XBAR1_IN147 <- TPM2  TPM_CH_TRIGGER0 */
#define IMXRT_XBAR1_IN_TPM2_TPM_CH_TRIGGER1                                     IMXRT_XBAR1(XBAR_INPUT, 148)    /* XBAR1_IN148 <- TPM2  TPM_CH_TRIGGER1 */
#define IMXRT_XBAR1_IN_TPM2_TPM_CH_TRIGGER2                                     IMXRT_XBAR1(XBAR_INPUT, 149)    /* XBAR1_IN149 <- TPM2  TPM_CH_TRIGGER2 */
#define IMXRT_XBAR1_IN_TPM2_TPM_CH_TRIGGER3                                     IMXRT_XBAR1(XBAR_INPUT, 150)    /* XBAR1_IN150 <- TPM2  TPM_CH_TRIGGER3 */
#define IMXRT_XBAR1_IN_TPM2_TPM_TRIGGER                                         IMXRT_XBAR1(XBAR_INPUT, 151)    /* XBAR1_IN151 <- TPM2  TPM_TRIGGER */
#define IMXRT_XBAR1_IN_TPM3_TPM_CH_TRIGGER0                                     IMXRT_XBAR1(XBAR_INPUT, 152)    /* XBAR1_IN152 <- TPM3  TPM_CH_TRIGGER0 */
#define IMXRT_XBAR1_IN_TPM3_TPM_CH_TRIGGER1                                     IMXRT_XBAR1(XBAR_INPUT, 153)    /* XBAR1_IN153 <- TPM3  TPM_CH_TRIGGER1 */
#define IMXRT_XBAR1_IN_TPM3_TPM_CH_TRIGGER2                                     IMXRT_XBAR1(XBAR_INPUT, 154)    /* XBAR1_IN154 <- TPM3  TPM_CH_TRIGGER2 */
#define IMXRT_XBAR1_IN_TPM3_TPM_CH_TRIGGER3                                     IMXRT_XBAR1(XBAR_INPUT, 155)    /* XBAR1_IN155 <- TPM3  TPM_CH_TRIGGER3 */
#define IMXRT_XBAR1_IN_TPM3_TPM_TRIGGER                                         IMXRT_XBAR1(XBAR_INPUT, 156)    /* XBAR1_IN156 <- TPM3  TPM_TRIGGER */
#define IMXRT_XBAR1_IN_TPM4_TPM_CH_TRIGGER0                                     IMXRT_XBAR1(XBAR_INPUT, 157)    /* XBAR1_IN157 <- TPM4  TPM_CH_TRIGGER0 */
#define IMXRT_XBAR1_IN_TPM4_TPM_CH_TRIGGER1                                     IMXRT_XBAR1(XBAR_INPUT, 158)    /* XBAR1_IN158 <- TPM4  TPM_CH_TRIGGER1 */
#define IMXRT_XBAR1_IN_TPM4_TPM_CH_TRIGGER2                                     IMXRT_XBAR1(XBAR_INPUT, 159)    /* XBAR1_IN159 <- TPM4  TPM_CH_TRIGGER2 */
#define IMXRT_XBAR1_IN_TPM4_TPM_CH_TRIGGER3                                     IMXRT_XBAR1(XBAR_INPUT, 160)    /* XBAR1_IN160 <- TPM4  TPM_CH_TRIGGER3 */
#define IMXRT_XBAR1_IN_TPM4_TPM_TRIGGER                                         IMXRT_XBAR1(XBAR_INPUT, 161)    /* XBAR1_IN161 <- TPM4  TPM_TRIGGER */
#define IMXRT_XBAR1_IN_TPM5_TPM_CH_TRIGGER0                                     IMXRT_XBAR1(XBAR_INPUT, 162)    /* XBAR1_IN162 <- TPM5  TPM_CH_TRIGGER0 */
#define IMXRT_XBAR1_IN_TPM5_TPM_CH_TRIGGER1                                     IMXRT_XBAR1(XBAR_INPUT, 163)    /* XBAR1_IN163 <- TPM5  TPM_CH_TRIGGER1 */
#define IMXRT_XBAR1_IN_TPM5_TPM_CH_TRIGGER2                                     IMXRT_XBAR1(XBAR_INPUT, 164)    /* XBAR1_IN164 <- TPM5  TPM_CH_TRIGGER2 */
#define IMXRT_XBAR1_IN_TPM5_TPM_CH_TRIGGER3                                     IMXRT_XBAR1(XBAR_INPUT, 165)    /* XBAR1_IN165 <- TPM5  TPM_CH_TRIGGER3 */
#define IMXRT_XBAR1_IN_TPM5_TPM_TRIGGER                                         IMXRT_XBAR1(XBAR_INPUT, 166)    /* XBAR1_IN166 <- TPM5  TPM_TRIGGER */
#define IMXRT_XBAR1_IN_TPM6_TPM_CH_TRIGGER0                                     IMXRT_XBAR1(XBAR_INPUT, 167)    /* XBAR1_IN167 <- TPM6  TPM_CH_TRIGGER0 */
#define IMXRT_XBAR1_IN_TPM6_TPM_CH_TRIGGER1                                     IMXRT_XBAR1(XBAR_INPUT, 168)    /* XBAR1_IN168 <- TPM6  TPM_CH_TRIGGER1 */
#define IMXRT_XBAR1_IN_TPM6_TPM_CH_TRIGGER2                                     IMXRT_XBAR1(XBAR_INPUT, 169)    /* XBAR1_IN169 <- TPM6  TPM_CH_TRIGGER2 */
#define IMXRT_XBAR1_IN_TPM6_TPM_CH_TRIGGER3                                     IMXRT_XBAR1(XBAR_INPUT, 170)    /* XBAR1_IN170 <- TPM6  TPM_CH_TRIGGER3 */
#define IMXRT_XBAR1_IN_TPM6_TPM_TRIGGER                                         IMXRT_XBAR1(XBAR_INPUT, 171)    /* XBAR1_IN171 <- TPM6  TPM_TRIGGER */
#define IMXRT_XBAR1_IN_LPTMR1_LPTIMER_TRIGGER_DELAY                             IMXRT_XBAR1(XBAR_INPUT, 172)    /* XBAR1_IN172 <- LPTMR1  LPTIMER_TRIGGER_DELAY */
#define IMXRT_XBAR1_IN_LPTMR2_LPTIMER_TRIGGER_DELAY                             IMXRT_XBAR1(XBAR_INPUT, 173)    /* XBAR1_IN173 <- LPTMR2  LPTIMER_TRIGGER_DELAY */
#define IMXRT_XBAR1_IN_LPTMR3_LPTIMER_TRIGGER_DELAY                             IMXRT_XBAR1(XBAR_INPUT, 174)    /* XBAR1_IN174 <- LPTMR3  LPTIMER_TRIGGER_DELAY */
#define IMXRT_XBAR1_IN_NETC_TMR_1588_PP1                                        IMXRT_XBAR1(XBAR_INPUT, 175)    /* XBAR1_IN175 <- NETC  TMR_1588_PP1 */
#define IMXRT_XBAR1_IN_NETC_TMR_1588_PP2                                        IMXRT_XBAR1(XBAR_INPUT, 176)    /* XBAR1_IN176 <- NETC  TMR_1588_PP2 */
#define IMXRT_XBAR1_IN_NETC_TMR_1588_PP3                                        IMXRT_XBAR1(XBAR_INPUT, 177)    /* XBAR1_IN177 <- NETC  TMR_1588_PP3 */
#define IMXRT_XBAR1_IN_SINC1_CHANNEL_0_BREAK_DUE_TO_CLOCK_ABSENCE_DETECTED      IMXRT_XBAR1(XBAR_INPUT, 178)    /* XBAR1_IN178 <- SINC1  Channel 0 break due to clock absence detected */
#define IMXRT_XBAR1_IN_SINC1_CHANNEL_1_BREAK_DUE_TO_CLOCK_ABSENCE_DETECTED      IMXRT_XBAR1(XBAR_INPUT, 179)    /* XBAR1_IN179 <- SINC1  Channel 1 break due to clock absence detected */
#define IMXRT_XBAR1_IN_SINC1_CHANNEL_2_BREAK_DUE_TO_CLOCK_ABSENCE_DETECTED      IMXRT_XBAR1(XBAR_INPUT, 180)    /* XBAR1_IN180 <- SINC1  Channel 2 break due to clock absence detected */
#define IMXRT_XBAR1_IN_SINC1_CHANNEL_3_BREAK_DUE_TO_CLOCK_ABSENCE_DETECTED      IMXRT_XBAR1(XBAR_INPUT, 181)    /* XBAR1_IN181 <- SINC1  Channel 3 break due to clock absence detected */
#define IMXRT_XBAR1_IN_SINC2_CHANNEL_0_BREAK_DUE_TO_CLOCK_ABSENCE_DETECTED      IMXRT_XBAR1(XBAR_INPUT, 182)    /* XBAR1_IN182 <- SINC2  Channel 0 break due to clock absence detected */
#define IMXRT_XBAR1_IN_SINC2_CHANNEL_1_BREAK_DUE_TO_CLOCK_ABSENCE_DETECTED      IMXRT_XBAR1(XBAR_INPUT, 183)    /* XBAR1_IN183 <- SINC2  Channel 1 break due to clock absence detected */
#define IMXRT_XBAR1_IN_SINC2_CHANNEL_2_BREAK_DUE_TO_CLOCK_ABSENCE_DETECTED      IMXRT_XBAR1(XBAR_INPUT, 184)    /* XBAR1_IN184 <- SINC2  Channel 2 break due to clock absence detected */
#define IMXRT_XBAR1_IN_SINC2_CHANNEL_3_BREAK_DUE_TO_CLOCK_ABSENCE_DETECTED      IMXRT_XBAR1(XBAR_INPUT, 185)    /* XBAR1_IN185 <- SINC2  Channel 3 break due to clock absence detected */
#define IMXRT_XBAR1_IN_SINC3_CHANNEL_0_BREAK_DUE_TO_CLOCK_ABSENCE_DETECTED      IMXRT_XBAR1(XBAR_INPUT, 186)    /* XBAR1_IN186 <- SINC3  Channel 0 break due to clock absence detected */
#define IMXRT_XBAR1_IN_SINC3_CHANNEL_1_BREAK_DUE_TO_CLOCK_ABSENCE_DETECTED      IMXRT_XBAR1(XBAR_INPUT, 187)    /* XBAR1_IN187 <- SINC3  Channel 1 break due to clock absence detected */
#define IMXRT_XBAR1_IN_SINC3_CHANNEL_2_BREAK_DUE_TO_CLOCK_ABSENCE_DETECTED      IMXRT_XBAR1(XBAR_INPUT, 188)    /* XBAR1_IN188 <- SINC3  Channel 2 break due to clock absence detected */
#define IMXRT_XBAR1_IN_SINC3_CHANNEL_3_BREAK_DUE_TO_CLOCK_ABSENCE_DETECTED      IMXRT_XBAR1(XBAR_INPUT, 189)    /* XBAR1_IN189 <- SINC3  Channel 3 break due to clock absence detected */
#define IMXRT_XBAR1_IN_AOI1_AOI_OUT0                                            IMXRT_XBAR1(XBAR_INPUT, 190)    /* XBAR1_IN190 <- AOI1  AOI_OUT0 */
#define IMXRT_XBAR1_IN_AOI1_AOI_OUT1                                            IMXRT_XBAR1(XBAR_INPUT, 191)    /* XBAR1_IN191 <- AOI1  AOI_OUT1 */
#define IMXRT_XBAR1_IN_AOI1_AOI_OUT2                                            IMXRT_XBAR1(XBAR_INPUT, 192)    /* XBAR1_IN192 <- AOI1  AOI_OUT2 */
#define IMXRT_XBAR1_IN_AOI1_AOI_OUT3                                            IMXRT_XBAR1(XBAR_INPUT, 193)    /* XBAR1_IN193 <- AOI1  AOI_OUT3 */
#define IMXRT_XBAR1_IN_AOI2_AOI_OUT0                                            IMXRT_XBAR1(XBAR_INPUT, 194)    /* XBAR1_IN194 <- AOI2  AOI_OUT0 */
#define IMXRT_XBAR1_IN_AOI2_AOI_OUT1                                            IMXRT_XBAR1(XBAR_INPUT, 195)    /* XBAR1_IN195 <- AOI2  AOI_OUT1 */
#define IMXRT_XBAR1_IN_AOI2_AOI_OUT2                                            IMXRT_XBAR1(XBAR_INPUT, 196)    /* XBAR1_IN196 <- AOI2  AOI_OUT2 */
#define IMXRT_XBAR1_IN_AOI2_AOI_OUT3                                            IMXRT_XBAR1(XBAR_INPUT, 197)    /* XBAR1_IN197 <- AOI2  AOI_OUT3 */
#define IMXRT_XBAR1_IN_TRIGGER_SY_TRIGGER_SYNC_OUT4_NC                          IMXRT_XBAR1(XBAR_INPUT, 198)    /* XBAR1_IN198 <- TRIGGER_SY TRIGGER_SYNC_OUT4  - NC */
#define IMXRT_XBAR1_IN_TRIGGER_SY_TRIGGER_SYNC_OUT5_NC                          IMXRT_XBAR1(XBAR_INPUT, 199)    /* XBAR1_IN199 <- TRIGGER_SY TRIGGER_SYNC_OUT5  - NC */
#define IMXRT_XBAR1_IN_TRIGGER_SY_TRIGGER_SYNC_OUT6_NC                          IMXRT_XBAR1(XBAR_INPUT, 200)    /* XBAR1_IN200 <- TRIGGER_SY TRIGGER_SYNC_OUT6  - NC */
#define IMXRT_XBAR1_IN_TRIGGER_SY_TRIGGER_SYNC_OUT7_NC                          IMXRT_XBAR1(XBAR_INPUT, 201)    /* XBAR1_IN201 <- TRIGGER_SY TRIGGER_SYNC_OUT7  - NC */
#define IMXRT_XBAR1_IN_RESERVED                                                 IMXRT_XBAR1(XBAR_INPUT, 202)    /* XBAR1_IN202 <- -  Reserved */
#define IMXRT_XBAR1_IN_RESERVED_2                                               IMXRT_XBAR1(XBAR_INPUT, 203)    /* XBAR1_IN203 <- -  Reserved */
#define IMXRT_XBAR1_IN_RESERVED_3                                               IMXRT_XBAR1(XBAR_INPUT, 204)    /* XBAR1_IN204 <- -  Reserved */
#define IMXRT_XBAR1_IN_RESERVED_4                                               IMXRT_XBAR1(XBAR_INPUT, 205)    /* XBAR1_IN205 <- -  Reserved */
#define IMXRT_XBAR1_IN_AOI3_AOI_OUT0                                            IMXRT_XBAR1(XBAR_INPUT, 206)    /* XBAR1_IN206 <- AOI3  AOI_OUT0 */
#define IMXRT_XBAR1_IN_AOI3_AOI_OUT1                                            IMXRT_XBAR1(XBAR_INPUT, 207)    /* XBAR1_IN207 <- AOI3  AOI_OUT1 */
#define IMXRT_XBAR1_IN_AOI3_AOI_OUT2                                            IMXRT_XBAR1(XBAR_INPUT, 208)    /* XBAR1_IN208 <- AOI3  AOI_OUT2 */
#define IMXRT_XBAR1_IN_AOI3_AOI_OUT3                                            IMXRT_XBAR1(XBAR_INPUT, 209)    /* XBAR1_IN209 <- AOI3  AOI_OUT3 */
#define IMXRT_XBAR1_IN_AOI4_AOI_OUT0                                            IMXRT_XBAR1(XBAR_INPUT, 210)    /* XBAR1_IN210 <- AOI4  AOI_OUT0 */
#define IMXRT_XBAR1_IN_AOI4_AOI_OUT1                                            IMXRT_XBAR1(XBAR_INPUT, 211)    /* XBAR1_IN211 <- AOI4  AOI_OUT1 */
#define IMXRT_XBAR1_IN_AOI4_AOI_OUT2                                            IMXRT_XBAR1(XBAR_INPUT, 212)    /* XBAR1_IN212 <- AOI4  AOI_OUT2 */
#define IMXRT_XBAR1_IN_AOI4_AOI_OUT3                                            IMXRT_XBAR1(XBAR_INPUT, 213)    /* XBAR1_IN213 <- AOI4  AOI_OUT3 */
#define IMXRT_XBAR1_IN_ECAT_SYNC_OUT0                                           IMXRT_XBAR1(XBAR_INPUT, 214)    /* XBAR1_IN214 <- ECAT  SYNC_OUT0 */
#define IMXRT_XBAR1_IN_ECAT_SYNC_OUT1                                           IMXRT_XBAR1(XBAR_INPUT, 215)    /* XBAR1_IN215 <- ECAT  SYNC_OUT1 */

#define IMXRT_XBAR2_IN_GND                                                      IMXRT_XBAR2(XBAR_INPUT, 0)      /* XBAR2_IN0 <- -  GND */
#define IMXRT_XBAR2_IN_1B1                                                      IMXRT_XBAR2(XBAR_INPUT, 1)      /* XBAR2_IN1 <- -  1'B1 */
#define IMXRT_XBAR2_IN_GND_2                                                    IMXRT_XBAR2(XBAR_INPUT, 2)      /* XBAR2_IN2 <- -  GND */
#define IMXRT_XBAR2_IN_1B1_2                                                    IMXRT_XBAR2(XBAR_INPUT, 3)      /* XBAR2_IN3 <- -  1'B1 */
#define IMXRT_XBAR2_IN_CMP1_COUT                                                IMXRT_XBAR2(XBAR_INPUT, 4)      /* XBAR2_IN4 <- CMP1  COUT */
#define IMXRT_XBAR2_IN_CMP2_COUT                                                IMXRT_XBAR2(XBAR_INPUT, 5)      /* XBAR2_IN5 <- CMP2  COUT */
#define IMXRT_XBAR2_IN_CMP3_COUT                                                IMXRT_XBAR2(XBAR_INPUT, 6)      /* XBAR2_IN6 <- CMP3  COUT */
#define IMXRT_XBAR2_IN_CMP4_COUT                                                IMXRT_XBAR2(XBAR_INPUT, 7)      /* XBAR2_IN7 <- CMP4  COUT */
#define IMXRT_XBAR2_IN_TMR1_TMR0_OUTPUT                                         IMXRT_XBAR2(XBAR_INPUT, 8)      /* XBAR2_IN8 <- TMR1  TMR0_OUTPUT */
#define IMXRT_XBAR2_IN_TMR1_TMR1_OUTPUT                                         IMXRT_XBAR2(XBAR_INPUT, 9)      /* XBAR2_IN9 <- TMR1  TMR1_OUTPUT */
#define IMXRT_XBAR2_IN_TMR1_TMR2_OUTPUT                                         IMXRT_XBAR2(XBAR_INPUT, 10)     /* XBAR2_IN10 <- TMR1  TMR2_OUTPUT */
#define IMXRT_XBAR2_IN_TMR1_TMR3_OUTPUT                                         IMXRT_XBAR2(XBAR_INPUT, 11)     /* XBAR2_IN11 <- TMR1  TMR3_OUTPUT */
#define IMXRT_XBAR2_IN_TMR2_TMR0_OUTPUT                                         IMXRT_XBAR2(XBAR_INPUT, 12)     /* XBAR2_IN12 <- TMR2  TMR0_OUTPUT */
#define IMXRT_XBAR2_IN_TMR2_TMR1_OUTPUT                                         IMXRT_XBAR2(XBAR_INPUT, 13)     /* XBAR2_IN13 <- TMR2  TMR1_OUTPUT */
#define IMXRT_XBAR2_IN_TMR2_TMR2_OUTPUT                                         IMXRT_XBAR2(XBAR_INPUT, 14)     /* XBAR2_IN14 <- TMR2  TMR2_OUTPUT */
#define IMXRT_XBAR2_IN_TMR2_TMR3_OUTPUT                                         IMXRT_XBAR2(XBAR_INPUT, 15)     /* XBAR2_IN15 <- TMR2  TMR3_OUTPUT */
#define IMXRT_XBAR2_IN_TMR3_TMR0_OUTPUT                                         IMXRT_XBAR2(XBAR_INPUT, 16)     /* XBAR2_IN16 <- TMR3  TMR0_OUTPUT */
#define IMXRT_XBAR2_IN_TMR3_TMR1_OUTPUT                                         IMXRT_XBAR2(XBAR_INPUT, 17)     /* XBAR2_IN17 <- TMR3  TMR1_OUTPUT */
#define IMXRT_XBAR2_IN_TMR3_TMR2_OUTPUT                                         IMXRT_XBAR2(XBAR_INPUT, 18)     /* XBAR2_IN18 <- TMR3  TMR2_OUTPUT */
#define IMXRT_XBAR2_IN_TMR3_TMR3_OUTPUT                                         IMXRT_XBAR2(XBAR_INPUT, 19)     /* XBAR2_IN19 <- TMR3  TMR3_OUTPUT */
#define IMXRT_XBAR2_IN_TMR4_TMR0_OUTPUT                                         IMXRT_XBAR2(XBAR_INPUT, 20)     /* XBAR2_IN20 <- TMR4  TMR0_OUTPUT */
#define IMXRT_XBAR2_IN_TMR4_TMR1_OUTPUT                                         IMXRT_XBAR2(XBAR_INPUT, 21)     /* XBAR2_IN21 <- TMR4  TMR1_OUTPUT */
#define IMXRT_XBAR2_IN_TMR4_TMR2_OUTPUT                                         IMXRT_XBAR2(XBAR_INPUT, 22)     /* XBAR2_IN22 <- TMR4  TMR2_OUTPUT */
#define IMXRT_XBAR2_IN_TMR4_TMR3_OUTPUT                                         IMXRT_XBAR2(XBAR_INPUT, 23)     /* XBAR2_IN23 <- TMR4  TMR3_OUTPUT */
#define IMXRT_XBAR2_IN_TMR5_TMR0_OUTPUT                                         IMXRT_XBAR2(XBAR_INPUT, 24)     /* XBAR2_IN24 <- TMR5  TMR0_OUTPUT */
#define IMXRT_XBAR2_IN_TMR5_TMR1_OUTPUT                                         IMXRT_XBAR2(XBAR_INPUT, 25)     /* XBAR2_IN25 <- TMR5  TMR1_OUTPUT */
#define IMXRT_XBAR2_IN_TMR5_TMR2_OUTPUT                                         IMXRT_XBAR2(XBAR_INPUT, 26)     /* XBAR2_IN26 <- TMR5  TMR2_OUTPUT */
#define IMXRT_XBAR2_IN_TMR5_TMR3_OUTPUT                                         IMXRT_XBAR2(XBAR_INPUT, 27)     /* XBAR2_IN27 <- TMR5  TMR3_OUTPUT */
#define IMXRT_XBAR2_IN_TMR6_TMR0_OUTPUT                                         IMXRT_XBAR2(XBAR_INPUT, 28)     /* XBAR2_IN28 <- TMR6  TMR0_OUTPUT */
#define IMXRT_XBAR2_IN_TMR6_TMR1_OUTPUT                                         IMXRT_XBAR2(XBAR_INPUT, 29)     /* XBAR2_IN29 <- TMR6  TMR1_OUTPUT */
#define IMXRT_XBAR2_IN_TMR6_TMR2_OUTPUT                                         IMXRT_XBAR2(XBAR_INPUT, 30)     /* XBAR2_IN30 <- TMR6  TMR2_OUTPUT */
#define IMXRT_XBAR2_IN_TMR6_TMR3_OUTPUT                                         IMXRT_XBAR2(XBAR_INPUT, 31)     /* XBAR2_IN31 <- TMR6  TMR3_OUTPUT */
#define IMXRT_XBAR2_IN_TMR7_TMR0_OUTPUT                                         IMXRT_XBAR2(XBAR_INPUT, 32)     /* XBAR2_IN32 <- TMR7  TMR0_OUTPUT */
#define IMXRT_XBAR2_IN_TMR7_TMR1_OUTPUT                                         IMXRT_XBAR2(XBAR_INPUT, 33)     /* XBAR2_IN33 <- TMR7  TMR1_OUTPUT */
#define IMXRT_XBAR2_IN_TMR7_TMR2_OUTPUT                                         IMXRT_XBAR2(XBAR_INPUT, 34)     /* XBAR2_IN34 <- TMR7  TMR2_OUTPUT */
#define IMXRT_XBAR2_IN_TMR7_TMR3_OUTPUT                                         IMXRT_XBAR2(XBAR_INPUT, 35)     /* XBAR2_IN35 <- TMR7  TMR3_OUTPUT */
#define IMXRT_XBAR2_IN_TMR8_TMR0_OUTPUT                                         IMXRT_XBAR2(XBAR_INPUT, 36)     /* XBAR2_IN36 <- TMR8  TMR0_OUTPUT */
#define IMXRT_XBAR2_IN_TMR8_TMR1_OUTPUT                                         IMXRT_XBAR2(XBAR_INPUT, 37)     /* XBAR2_IN37 <- TMR8  TMR1_OUTPUT */
#define IMXRT_XBAR2_IN_TMR8_TMR2_OUTPUT                                         IMXRT_XBAR2(XBAR_INPUT, 38)     /* XBAR2_IN38 <- TMR8  TMR2_OUTPUT */
#define IMXRT_XBAR2_IN_TMR8_TMR3_OUTPUT                                         IMXRT_XBAR2(XBAR_INPUT, 39)     /* XBAR2_IN39 <- TMR8  TMR3_OUTPUT */
#define IMXRT_XBAR2_IN_FLEXPWM1_PWM0_MUX_TRIGGER0                               IMXRT_XBAR2(XBAR_INPUT, 40)     /* XBAR2_IN40 <- FLEXPWM1  PWM0_MUX_TRIGGER0 */
#define IMXRT_XBAR2_IN_FLEXPWM1_PWM1_MUX_TRIGGER0                               IMXRT_XBAR2(XBAR_INPUT, 41)     /* XBAR2_IN41 <- FLEXPWM1  PWM1_MUX_TRIGGER0 */
#define IMXRT_XBAR2_IN_FLEXPWM1_PWM2_MUX_TRIGGER0                               IMXRT_XBAR2(XBAR_INPUT, 42)     /* XBAR2_IN42 <- FLEXPWM1  PWM2_MUX_TRIGGER0 */
#define IMXRT_XBAR2_IN_FLEXPWM1_PWM3_MUX_TRIGGER0                               IMXRT_XBAR2(XBAR_INPUT, 43)     /* XBAR2_IN43 <- FLEXPWM1  PWM3_MUX_TRIGGER0 */
#define IMXRT_XBAR2_IN_FLEXPWM2_PWM0_MUX_TRIGGER0                               IMXRT_XBAR2(XBAR_INPUT, 44)     /* XBAR2_IN44 <- FLEXPWM2  PWM0_MUX_TRIGGER0 */
#define IMXRT_XBAR2_IN_FLEXPWM2_PWM1_MUX_TRIGGER0                               IMXRT_XBAR2(XBAR_INPUT, 45)     /* XBAR2_IN45 <- FLEXPWM2  PWM1_MUX_TRIGGER0 */
#define IMXRT_XBAR2_IN_FLEXPWM2_PWM2_MUX_TRIGGER0                               IMXRT_XBAR2(XBAR_INPUT, 46)     /* XBAR2_IN46 <- FLEXPWM2  PWM2_MUX_TRIGGER0 */
#define IMXRT_XBAR2_IN_FLEXPWM2_PWM3_MUX_TRIGGER0                               IMXRT_XBAR2(XBAR_INPUT, 47)     /* XBAR2_IN47 <- FLEXPWM2  PWM3_MUX_TRIGGER0 */
#define IMXRT_XBAR2_IN_FLEXPWM3_PWM0_MUX_TRIGGER0                               IMXRT_XBAR2(XBAR_INPUT, 48)     /* XBAR2_IN48 <- FLEXPWM3  PWM0_MUX_TRIGGER0 */
#define IMXRT_XBAR2_IN_FLEXPWM3_PWM1_MUX_TRIGGER0                               IMXRT_XBAR2(XBAR_INPUT, 49)     /* XBAR2_IN49 <- FLEXPWM3  PWM1_MUX_TRIGGER0 */
#define IMXRT_XBAR2_IN_FLEXPWM3_PWM2_MUX_TRIGGER0                               IMXRT_XBAR2(XBAR_INPUT, 50)     /* XBAR2_IN50 <- FLEXPWM3  PWM2_MUX_TRIGGER0 */
#define IMXRT_XBAR2_IN_FLEXPWM3_PWM3_MUX_TRIGGER0                               IMXRT_XBAR2(XBAR_INPUT, 51)     /* XBAR2_IN51 <- FLEXPWM3  PWM3_MUX_TRIGGER0 */
#define IMXRT_XBAR2_IN_FLEXPWM4_PWM0_MUX_TRIGGER0                               IMXRT_XBAR2(XBAR_INPUT, 52)     /* XBAR2_IN52 <- FLEXPWM4  PWM0_MUX_TRIGGER0 */
#define IMXRT_XBAR2_IN_FLEXPWM4_PWM1_MUX_TRIGGER0                               IMXRT_XBAR2(XBAR_INPUT, 53)     /* XBAR2_IN53 <- FLEXPWM4  PWM1_MUX_TRIGGER0 */
#define IMXRT_XBAR2_IN_FLEXPWM4_PWM2_MUX_TRIGGER0                               IMXRT_XBAR2(XBAR_INPUT, 54)     /* XBAR2_IN54 <- FLEXPWM4  PWM2_MUX_TRIGGER0 */
#define IMXRT_XBAR2_IN_FLEXPWM4_PWM3_MUX_TRIGGER0                               IMXRT_XBAR2(XBAR_INPUT, 55)     /* XBAR2_IN55 <- FLEXPWM4  PWM3_MUX_TRIGGER0 */
#define IMXRT_XBAR2_IN_LPIT1_LPIT_TRIG_OUT0                                     IMXRT_XBAR2(XBAR_INPUT, 56)     /* XBAR2_IN56 <- LPIT1  LPIT_TRIG_OUT0 */
#define IMXRT_XBAR2_IN_LPIT1_LPIT_TRIG_OUT1                                     IMXRT_XBAR2(XBAR_INPUT, 57)     /* XBAR2_IN57 <- LPIT1  LPIT_TRIG_OUT1 */
#define IMXRT_XBAR2_IN_LPIT1_LPIT_TRIG_OUT2                                     IMXRT_XBAR2(XBAR_INPUT, 58)     /* XBAR2_IN58 <- LPIT1  LPIT_TRIG_OUT2 */
#define IMXRT_XBAR2_IN_LPIT1_LPIT_TRIG_OUT3                                     IMXRT_XBAR2(XBAR_INPUT, 59)     /* XBAR2_IN59 <- LPIT1  LPIT_TRIG_OUT3 */
#define IMXRT_XBAR2_IN_LPIT2_LPIT_TRIG_OUT0                                     IMXRT_XBAR2(XBAR_INPUT, 60)     /* XBAR2_IN60 <- LPIT2  LPIT_TRIG_OUT0 */
#define IMXRT_XBAR2_IN_LPIT2_LPIT_TRIG_OUT1                                     IMXRT_XBAR2(XBAR_INPUT, 61)     /* XBAR2_IN61 <- LPIT2  LPIT_TRIG_OUT1 */
#define IMXRT_XBAR2_IN_LPIT2_LPIT_TRIG_OUT2                                     IMXRT_XBAR2(XBAR_INPUT, 62)     /* XBAR2_IN62 <- LPIT2  LPIT_TRIG_OUT2 */
#define IMXRT_XBAR2_IN_LPIT2_LPIT_TRIG_OUT3                                     IMXRT_XBAR2(XBAR_INPUT, 63)     /* XBAR2_IN63 <- LPIT2  LPIT_TRIG_OUT3 */
#define IMXRT_XBAR2_IN_LPIT3_LPIT_TRIG_OUT0                                     IMXRT_XBAR2(XBAR_INPUT, 64)     /* XBAR2_IN64 <- LPIT3  LPIT_TRIG_OUT0 */
#define IMXRT_XBAR2_IN_LPIT3_LPIT_TRIG_OUT1                                     IMXRT_XBAR2(XBAR_INPUT, 65)     /* XBAR2_IN65 <- LPIT3  LPIT_TRIG_OUT1 */
#define IMXRT_XBAR2_IN_LPIT3_LPIT_TRIG_OUT2                                     IMXRT_XBAR2(XBAR_INPUT, 66)     /* XBAR2_IN66 <- LPIT3  LPIT_TRIG_OUT2 */
#define IMXRT_XBAR2_IN_LPIT3_LPIT_TRIG_OUT3                                     IMXRT_XBAR2(XBAR_INPUT, 67)     /* XBAR2_IN67 <- LPIT3  LPIT_TRIG_OUT3 */
#define IMXRT_XBAR2_IN_SINC1_PULSE_TRG0                                         IMXRT_XBAR2(XBAR_INPUT, 68)     /* XBAR2_IN68 <- SINC1  PULSE_TRG0 */
#define IMXRT_XBAR2_IN_SINC1_PULSE_TRG1                                         IMXRT_XBAR2(XBAR_INPUT, 69)     /* XBAR2_IN69 <- SINC1  PULSE_TRG1 */
#define IMXRT_XBAR2_IN_SINC1_PULSE_TRG2                                         IMXRT_XBAR2(XBAR_INPUT, 70)     /* XBAR2_IN70 <- SINC1  PULSE_TRG2 */
#define IMXRT_XBAR2_IN_SINC1_PULSE_TRG3                                         IMXRT_XBAR2(XBAR_INPUT, 71)     /* XBAR2_IN71 <- SINC1  PULSE_TRG3 */
#define IMXRT_XBAR2_IN_EDMA4_DMA_TRIGGER_OUT0                                   IMXRT_XBAR2(XBAR_INPUT, 72)     /* XBAR2_IN72 <- eDMA4  DMA_TRIGGER_OUT0 */
#define IMXRT_XBAR2_IN_EDMA4_DMA_TRIGGER_OUT1                                   IMXRT_XBAR2(XBAR_INPUT, 73)     /* XBAR2_IN73 <- eDMA4  DMA_TRIGGER_OUT1 */
#define IMXRT_XBAR2_IN_EDMA4_DMA_TRIGGER_OUT2                                   IMXRT_XBAR2(XBAR_INPUT, 74)     /* XBAR2_IN74 <- eDMA4  DMA_TRIGGER_OUT2 */
#define IMXRT_XBAR2_IN_EDMA4_DMA_TRIGGER_OUT3                                   IMXRT_XBAR2(XBAR_INPUT, 75)     /* XBAR2_IN75 <- eDMA4  DMA_TRIGGER_OUT3 */
#define IMXRT_XBAR2_IN_EDMA4_DMA_TRIGGER_OUT4                                   IMXRT_XBAR2(XBAR_INPUT, 76)     /* XBAR2_IN76 <- eDMA4  DMA_TRIGGER_OUT4 */
#define IMXRT_XBAR2_IN_EDMA4_DMA_TRIGGER_OUT5                                   IMXRT_XBAR2(XBAR_INPUT, 77)     /* XBAR2_IN77 <- eDMA4  DMA_TRIGGER_OUT5 */
#define IMXRT_XBAR2_IN_EDMA4_DMA_TRIGGER_OUT6                                   IMXRT_XBAR2(XBAR_INPUT, 78)     /* XBAR2_IN78 <- eDMA4  DMA_TRIGGER_OUT6 */
#define IMXRT_XBAR2_IN_EDMA4_DMA_TRIGGER_OUT7                                   IMXRT_XBAR2(XBAR_INPUT, 79)     /* XBAR2_IN79 <- eDMA4  DMA_TRIGGER_OUT7 */
#define IMXRT_XBAR2_IN_EDMA3_DMA_TRIGGER_OUT0                                   IMXRT_XBAR2(XBAR_INPUT, 80)     /* XBAR2_IN80 <- eDMA3  DMA_TRIGGER_OUT0 */
#define IMXRT_XBAR2_IN_EDMA3_DMA_TRIGGER_OUT1                                   IMXRT_XBAR2(XBAR_INPUT, 81)     /* XBAR2_IN81 <- eDMA3  DMA_TRIGGER_OUT1 */
#define IMXRT_XBAR2_IN_EDMA3_DMA_TRIGGER_OUT2                                   IMXRT_XBAR2(XBAR_INPUT, 82)     /* XBAR2_IN82 <- eDMA3  DMA_TRIGGER_OUT2 */
#define IMXRT_XBAR2_IN_EDMA3_DMA_TRIGGER_OUT3                                   IMXRT_XBAR2(XBAR_INPUT, 83)     /* XBAR2_IN83 <- eDMA3  DMA_TRIGGER_OUT3 */
#define IMXRT_XBAR2_IN_EDMA3_DMA_TRIGGER_OUT4                                   IMXRT_XBAR2(XBAR_INPUT, 84)     /* XBAR2_IN84 <- eDMA3  DMA_TRIGGER_OUT4 */
#define IMXRT_XBAR2_IN_EDMA3_DMA_TRIGGER_OUT5                                   IMXRT_XBAR2(XBAR_INPUT, 85)     /* XBAR2_IN85 <- eDMA3  DMA_TRIGGER_OUT5 */
#define IMXRT_XBAR2_IN_EDMA3_DMA_TRIGGER_OUT6                                   IMXRT_XBAR2(XBAR_INPUT, 86)     /* XBAR2_IN86 <- eDMA3  DMA_TRIGGER_OUT6 */
#define IMXRT_XBAR2_IN_EDMA3_DMA_TRIGGER_OUT7                                   IMXRT_XBAR2(XBAR_INPUT, 87)     /* XBAR2_IN87 <- eDMA3  DMA_TRIGGER_OUT7 */
#define IMXRT_XBAR2_IN_ADC1_ADC_TCOMP_PULSE0                                    IMXRT_XBAR2(XBAR_INPUT, 88)     /* XBAR2_IN88 <- ADC1  ADC_TCOMP_PULSE0 */
#define IMXRT_XBAR2_IN_ADC1_ADC_TCOMP_PULSE1                                    IMXRT_XBAR2(XBAR_INPUT, 89)     /* XBAR2_IN89 <- ADC1  ADC_TCOMP_PULSE1 */
#define IMXRT_XBAR2_IN_ADC1_ADC_TCOMP_PULSE2                                    IMXRT_XBAR2(XBAR_INPUT, 90)     /* XBAR2_IN90 <- ADC1  ADC_TCOMP_PULSE2 */
#define IMXRT_XBAR2_IN_ADC1_ADC_TCOMP_PULSE3                                    IMXRT_XBAR2(XBAR_INPUT, 91)     /* XBAR2_IN91 <- ADC1  ADC_TCOMP_PULSE3 */
#define IMXRT_XBAR2_IN_ADC1_ADC_TCOMP_PULSE4                                    IMXRT_XBAR2(XBAR_INPUT, 92)     /* XBAR2_IN92 <- ADC1  ADC_TCOMP_PULSE4 */
#define IMXRT_XBAR2_IN_ADC1_ADC_TCOMP_PULSE5                                    IMXRT_XBAR2(XBAR_INPUT, 93)     /* XBAR2_IN93 <- ADC1  ADC_TCOMP_PULSE5 */
#define IMXRT_XBAR2_IN_ADC1_ADC_TCOMP_PULSE6                                    IMXRT_XBAR2(XBAR_INPUT, 94)     /* XBAR2_IN94 <- ADC1  ADC_TCOMP_PULSE6 */
#define IMXRT_XBAR2_IN_ADC1_ADC_TCOMP_PULSE7                                    IMXRT_XBAR2(XBAR_INPUT, 95)     /* XBAR2_IN95 <- ADC1  ADC_TCOMP_PULSE7 */
#define IMXRT_XBAR2_IN_ADC2_ADC_TCOMP_PULSE0                                    IMXRT_XBAR2(XBAR_INPUT, 96)     /* XBAR2_IN96 <- ADC2  ADC_TCOMP_PULSE0 */
#define IMXRT_XBAR2_IN_ADC2_ADC_TCOMP_PULSE1                                    IMXRT_XBAR2(XBAR_INPUT, 97)     /* XBAR2_IN97 <- ADC2  ADC_TCOMP_PULSE1 */
#define IMXRT_XBAR2_IN_ADC2_ADC_TCOMP_PULSE2                                    IMXRT_XBAR2(XBAR_INPUT, 98)     /* XBAR2_IN98 <- ADC2  ADC_TCOMP_PULSE2 */
#define IMXRT_XBAR2_IN_ADC2_ADC_TCOMP_PULSE3                                    IMXRT_XBAR2(XBAR_INPUT, 99)     /* XBAR2_IN99 <- ADC2  ADC_TCOMP_PULSE3 */
#define IMXRT_XBAR2_IN_ADC2_ADC_TCOMP_PULSE4                                    IMXRT_XBAR2(XBAR_INPUT, 100)    /* XBAR2_IN100 <- ADC2  ADC_TCOMP_PULSE4 */
#define IMXRT_XBAR2_IN_ADC2_ADC_TCOMP_PULSE5                                    IMXRT_XBAR2(XBAR_INPUT, 101)    /* XBAR2_IN101 <- ADC2  ADC_TCOMP_PULSE5 */
#define IMXRT_XBAR2_IN_ADC2_ADC_TCOMP_PULSE6                                    IMXRT_XBAR2(XBAR_INPUT, 102)    /* XBAR2_IN102 <- ADC2  ADC_TCOMP_PULSE6 */
#define IMXRT_XBAR2_IN_ADC2_ADC_TCOMP_PULSE7                                    IMXRT_XBAR2(XBAR_INPUT, 103)    /* XBAR2_IN103 <- ADC2  ADC_TCOMP_PULSE7 */
#define IMXRT_XBAR2_IN_TPM1_TPM_CH_TRIGGER0                                     IMXRT_XBAR2(XBAR_INPUT, 104)    /* XBAR2_IN104 <- TPM1  TPM_CH_TRIGGER0 */
#define IMXRT_XBAR2_IN_TPM1_TPM_CH_TRIGGER1                                     IMXRT_XBAR2(XBAR_INPUT, 105)    /* XBAR2_IN105 <- TPM1  TPM_CH_TRIGGER1 */
#define IMXRT_XBAR2_IN_TPM1_TPM_CH_TRIGGER2                                     IMXRT_XBAR2(XBAR_INPUT, 106)    /* XBAR2_IN106 <- TPM1  TPM_CH_TRIGGER2 */
#define IMXRT_XBAR2_IN_TPM1_TPM_CH_TRIGGER3                                     IMXRT_XBAR2(XBAR_INPUT, 107)    /* XBAR2_IN107 <- TPM1  TPM_CH_TRIGGER3 */
#define IMXRT_XBAR2_IN_TPM1_TPM_TRIGGER                                         IMXRT_XBAR2(XBAR_INPUT, 108)    /* XBAR2_IN108 <- TPM1  TPM_TRIGGER */
#define IMXRT_XBAR2_IN_TPM2_TPM_CH_TRIGGER0                                     IMXRT_XBAR2(XBAR_INPUT, 109)    /* XBAR2_IN109 <- TPM2  TPM_CH_TRIGGER0 */
#define IMXRT_XBAR2_IN_TPM2_TPM_CH_TRIGGER1                                     IMXRT_XBAR2(XBAR_INPUT, 110)    /* XBAR2_IN110 <- TPM2  TPM_CH_TRIGGER1 */
#define IMXRT_XBAR2_IN_TPM2_TPM_CH_TRIGGER2                                     IMXRT_XBAR2(XBAR_INPUT, 111)    /* XBAR2_IN111 <- TPM2  TPM_CH_TRIGGER2 */
#define IMXRT_XBAR2_IN_TPM2_TPM_CH_TRIGGER3                                     IMXRT_XBAR2(XBAR_INPUT, 112)    /* XBAR2_IN112 <- TPM2  TPM_CH_TRIGGER3 */
#define IMXRT_XBAR2_IN_TPM2_TPM_TRIGGER                                         IMXRT_XBAR2(XBAR_INPUT, 113)    /* XBAR2_IN113 <- TPM2  TPM_TRIGGER */
#define IMXRT_XBAR2_IN_TPM3_TPM_CH_TRIGGER0                                     IMXRT_XBAR2(XBAR_INPUT, 114)    /* XBAR2_IN114 <- TPM3  TPM_CH_TRIGGER0 */
#define IMXRT_XBAR2_IN_TPM3_TPM_CH_TRIGGER1                                     IMXRT_XBAR2(XBAR_INPUT, 115)    /* XBAR2_IN115 <- TPM3  TPM_CH_TRIGGER1 */
#define IMXRT_XBAR2_IN_TPM3_TPM_CH_TRIGGER2                                     IMXRT_XBAR2(XBAR_INPUT, 116)    /* XBAR2_IN116 <- TPM3  TPM_CH_TRIGGER2 */
#define IMXRT_XBAR2_IN_TPM3_TPM_CH_TRIGGER3                                     IMXRT_XBAR2(XBAR_INPUT, 117)    /* XBAR2_IN117 <- TPM3  TPM_CH_TRIGGER3 */
#define IMXRT_XBAR2_IN_TPM3_TPM_TRIGGER                                         IMXRT_XBAR2(XBAR_INPUT, 118)    /* XBAR2_IN118 <- TPM3  TPM_TRIGGER */
#define IMXRT_XBAR2_IN_TPM4_TPM_CH_TRIGGER0                                     IMXRT_XBAR2(XBAR_INPUT, 119)    /* XBAR2_IN119 <- TPM4  TPM_CH_TRIGGER0 */
#define IMXRT_XBAR2_IN_TPM4_TPM_CH_TRIGGER1                                     IMXRT_XBAR2(XBAR_INPUT, 120)    /* XBAR2_IN120 <- TPM4  TPM_CH_TRIGGER1 */
#define IMXRT_XBAR2_IN_TPM4_TPM_CH_TRIGGER2                                     IMXRT_XBAR2(XBAR_INPUT, 121)    /* XBAR2_IN121 <- TPM4  TPM_CH_TRIGGER2 */
#define IMXRT_XBAR2_IN_TPM4_TPM_CH_TRIGGER3                                     IMXRT_XBAR2(XBAR_INPUT, 122)    /* XBAR2_IN122 <- TPM4  TPM_CH_TRIGGER3 */
#define IMXRT_XBAR2_IN_TPM4_TPM_TRIGGER                                         IMXRT_XBAR2(XBAR_INPUT, 123)    /* XBAR2_IN123 <- TPM4  TPM_TRIGGER */
#define IMXRT_XBAR2_IN_TPM5_TPM_CH_TRIGGER0                                     IMXRT_XBAR2(XBAR_INPUT, 124)    /* XBAR2_IN124 <- TPM5  TPM_CH_TRIGGER0 */
#define IMXRT_XBAR2_IN_TPM5_TPM_CH_TRIGGER1                                     IMXRT_XBAR2(XBAR_INPUT, 125)    /* XBAR2_IN125 <- TPM5  TPM_CH_TRIGGER1 */
#define IMXRT_XBAR2_IN_TPM5_TPM_CH_TRIGGER2                                     IMXRT_XBAR2(XBAR_INPUT, 126)    /* XBAR2_IN126 <- TPM5  TPM_CH_TRIGGER2 */
#define IMXRT_XBAR2_IN_TPM5_TPM_CH_TRIGGER3                                     IMXRT_XBAR2(XBAR_INPUT, 127)    /* XBAR2_IN127 <- TPM5  TPM_CH_TRIGGER3 */
#define IMXRT_XBAR2_IN_TPM5_TPM_TRIGGER                                         IMXRT_XBAR2(XBAR_INPUT, 128)    /* XBAR2_IN128 <- TPM5  TPM_TRIGGER */
#define IMXRT_XBAR2_IN_TPM6_TPM_CH_TRIGGER0                                     IMXRT_XBAR2(XBAR_INPUT, 129)    /* XBAR2_IN129 <- TPM6  TPM_CH_TRIGGER0 */
#define IMXRT_XBAR2_IN_TPM6_TPM_CH_TRIGGER1                                     IMXRT_XBAR2(XBAR_INPUT, 130)    /* XBAR2_IN130 <- TPM6  TPM_CH_TRIGGER1 */
#define IMXRT_XBAR2_IN_TPM6_TPM_CH_TRIGGER2                                     IMXRT_XBAR2(XBAR_INPUT, 131)    /* XBAR2_IN131 <- TPM6  TPM_CH_TRIGGER2 */
#define IMXRT_XBAR2_IN_TPM6_TPM_CH_TRIGGER3                                     IMXRT_XBAR2(XBAR_INPUT, 132)    /* XBAR2_IN132 <- TPM6  TPM_CH_TRIGGER3 */
#define IMXRT_XBAR2_IN_TPM6_TPM_TRIGGER                                         IMXRT_XBAR2(XBAR_INPUT, 133)    /* XBAR2_IN133 <- TPM6  TPM_TRIGGER */
#define IMXRT_XBAR2_IN_LPTMR1_LPTIMER_TRIGGER_DELAY                             IMXRT_XBAR2(XBAR_INPUT, 134)    /* XBAR2_IN134 <- LPTMR1  LPTIMER_TRIGGER_DELAY */
#define IMXRT_XBAR2_IN_LPTMR2_LPTIMER_TRIGGER_DELAY                             IMXRT_XBAR2(XBAR_INPUT, 135)    /* XBAR2_IN135 <- LPTMR2  LPTIMER_TRIGGER_DELAY */
#define IMXRT_XBAR2_IN_LPTMR3_LPTIMER_TRIGGER_DELAY                             IMXRT_XBAR2(XBAR_INPUT, 136)    /* XBAR2_IN136 <- LPTMR3  LPTIMER_TRIGGER_DELAY */
#define IMXRT_XBAR2_IN_NETC_TMR_1588_PP1                                        IMXRT_XBAR2(XBAR_INPUT, 137)    /* XBAR2_IN137 <- NETC  TMR_1588_PP1 */
#define IMXRT_XBAR2_IN_NETC_TMR_1588_PP2                                        IMXRT_XBAR2(XBAR_INPUT, 138)    /* XBAR2_IN138 <- NETC  TMR_1588_PP2 */
#define IMXRT_XBAR2_IN_NETC_TMR_1588_PP3                                        IMXRT_XBAR2(XBAR_INPUT, 139)    /* XBAR2_IN139 <- NETC  TMR_1588_PP3 */
#define IMXRT_XBAR2_IN_SINC1_CHANNEL_0_BREAK_DUE_TO_CLOCK_ABSENCE_DETECTED      IMXRT_XBAR2(XBAR_INPUT, 140)    /* XBAR2_IN140 <- SINC1  Channel 0 break due to clock absence detected */
#define IMXRT_XBAR2_IN_SINC1_CHANNEL_1_BREAK_DUE_TO_CLOCK_ABSENCE_DETECTED      IMXRT_XBAR2(XBAR_INPUT, 141)    /* XBAR2_IN141 <- SINC1  Channel 1 break due to clock absence detected */
#define IMXRT_XBAR2_IN_SINC1_CHANNEL_2_BREAK_DUE_TO_CLOCK_ABSENCE_DETECTED      IMXRT_XBAR2(XBAR_INPUT, 142)    /* XBAR2_IN142 <- SINC1  Channel 2 break due to clock absence detected */
#define IMXRT_XBAR2_IN_SINC1_CHANNEL_3_BREAK_DUE_TO_CLOCK_ABSENCE_DETECTED      IMXRT_XBAR2(XBAR_INPUT, 143)    /* XBAR2_IN143 <- SINC1  Channel 3 break due to clock absence detected */
#define IMXRT_XBAR2_IN_SINC2_CHANNEL_0_BREAK_DUE_TO_CLOCK_ABSENCE_DETECTED      IMXRT_XBAR2(XBAR_INPUT, 144)    /* XBAR2_IN144 <- SINC2  Channel 0 break due to clock absence detected */
#define IMXRT_XBAR2_IN_SINC2_CHANNEL_1_BREAK_DUE_TO_CLOCK_ABSENCE_DETECTED      IMXRT_XBAR2(XBAR_INPUT, 145)    /* XBAR2_IN145 <- SINC2  Channel 1 break due to clock absence detected */
#define IMXRT_XBAR2_IN_SINC2_CHANNEL_2_BREAK_DUE_TO_CLOCK_ABSENCE_DETECTED      IMXRT_XBAR2(XBAR_INPUT, 146)    /* XBAR2_IN146 <- SINC2  Channel 2 break due to clock absence detected */
#define IMXRT_XBAR2_IN_SINC2_CHANNEL_3_BREAK_DUE_TO_CLOCK_ABSENCE_DETECTED      IMXRT_XBAR2(XBAR_INPUT, 147)    /* XBAR2_IN147 <- SINC2  Channel 3 break due to clock absence detected */
#define IMXRT_XBAR2_IN_SINC3_CHANNEL_0_BREAK_DUE_TO_CLOCK_ABSENCE_DETECTED      IMXRT_XBAR2(XBAR_INPUT, 148)    /* XBAR2_IN148 <- SINC3  Channel 0 break due to clock absence detected */
#define IMXRT_XBAR2_IN_SINC3_CHANNEL_1_BREAK_DUE_TO_CLOCK_ABSENCE_DETECTED      IMXRT_XBAR2(XBAR_INPUT, 149)    /* XBAR2_IN149 <- SINC3  Channel 1 break due to clock absence detected */
#define IMXRT_XBAR2_IN_SINC3_CHANNEL_2_BREAK_DUE_TO_CLOCK_ABSENCE_DETECTED      IMXRT_XBAR2(XBAR_INPUT, 150)    /* XBAR2_IN150 <- SINC3  Channel 2 break due to clock absence detected */
#define IMXRT_XBAR2_IN_SINC3_CHANNEL_3_BREAK_DUE_TO_CLOCK_ABSENCE_DETECTED      IMXRT_XBAR2(XBAR_INPUT, 151)    /* XBAR2_IN151 <- SINC3  Channel 3 break due to clock absence detected */
#define IMXRT_XBAR2_IN_RESERVED                                                 IMXRT_XBAR2(XBAR_INPUT, 152)    /* XBAR2_IN152 <- -  Reserved */
#define IMXRT_XBAR2_IN_RESERVED_2                                               IMXRT_XBAR2(XBAR_INPUT, 153)    /* XBAR2_IN153 <- -  Reserved */
#define IMXRT_XBAR2_IN_RESERVED_3                                               IMXRT_XBAR2(XBAR_INPUT, 154)    /* XBAR2_IN154 <- -  Reserved */
#define IMXRT_XBAR2_IN_RESERVED_4                                               IMXRT_XBAR2(XBAR_INPUT, 155)    /* XBAR2_IN155 <- -  Reserved */
#define IMXRT_XBAR2_IN_EQDC1_POS_MATCH0                                         IMXRT_XBAR2(XBAR_INPUT, 156)    /* XBAR2_IN156 <- eQDC1  POS_MATCH0 */
#define IMXRT_XBAR2_IN_EQDC1_POS_MATCH1                                         IMXRT_XBAR2(XBAR_INPUT, 157)    /* XBAR2_IN157 <- eQDC1  POS_MATCH1 */
#define IMXRT_XBAR2_IN_EQDC1_POS_MATCH2                                         IMXRT_XBAR2(XBAR_INPUT, 158)    /* XBAR2_IN158 <- eQDC1  POS_MATCH2 */
#define IMXRT_XBAR2_IN_EQDC1_POS_MATCH3                                         IMXRT_XBAR2(XBAR_INPUT, 159)    /* XBAR2_IN159 <- eQDC1  POS_MATCH3 */
#define IMXRT_XBAR2_IN_EQDC1_COMP_FLG0                                          IMXRT_XBAR2(XBAR_INPUT, 160)    /* XBAR2_IN160 <- eQDC1  COMP_FLG0 */
#define IMXRT_XBAR2_IN_EQDC1_COMP_FLG1                                          IMXRT_XBAR2(XBAR_INPUT, 161)    /* XBAR2_IN161 <- eQDC1  COMP_FLG1 */
#define IMXRT_XBAR2_IN_EQDC1_COMP_FLG2                                          IMXRT_XBAR2(XBAR_INPUT, 162)    /* XBAR2_IN162 <- eQDC1  COMP_FLG2 */
#define IMXRT_XBAR2_IN_EQDC1_COMP_FLG3                                          IMXRT_XBAR2(XBAR_INPUT, 163)    /* XBAR2_IN163 <- eQDC1  COMP_FLG3 */
#define IMXRT_XBAR2_IN_EQDC1_CNT_DN                                             IMXRT_XBAR2(XBAR_INPUT, 164)    /* XBAR2_IN164 <- eQDC1  CNT_DN */
#define IMXRT_XBAR2_IN_EQDC1_CNT_UP                                             IMXRT_XBAR2(XBAR_INPUT, 165)    /* XBAR2_IN165 <- eQDC1  CNT_UP */
#define IMXRT_XBAR2_IN_EQDC1_DIR                                                IMXRT_XBAR2(XBAR_INPUT, 166)    /* XBAR2_IN166 <- eQDC1  DIR */
#define IMXRT_XBAR2_IN_EQDC3_POS_MATCH0                                         IMXRT_XBAR2(XBAR_INPUT, 167)    /* XBAR2_IN167 <- eQDC3  POS_MATCH0 */
#define IMXRT_XBAR2_IN_EQDC3_POS_MATCH1                                         IMXRT_XBAR2(XBAR_INPUT, 168)    /* XBAR2_IN168 <- eQDC3  POS_MATCH1 */
#define IMXRT_XBAR2_IN_EQDC3_POS_MATCH2                                         IMXRT_XBAR2(XBAR_INPUT, 169)    /* XBAR2_IN169 <- eQDC3  POS_MATCH2 */
#define IMXRT_XBAR2_IN_EQDC3_POS_MATCH3                                         IMXRT_XBAR2(XBAR_INPUT, 170)    /* XBAR2_IN170 <- eQDC3  POS_MATCH3 */
#define IMXRT_XBAR2_IN_EQDC3_COMP_FLG0                                          IMXRT_XBAR2(XBAR_INPUT, 171)    /* XBAR2_IN171 <- eQDC3  COMP_FLG0 */
#define IMXRT_XBAR2_IN_EQDC3_COMP_FLG1                                          IMXRT_XBAR2(XBAR_INPUT, 172)    /* XBAR2_IN172 <- eQDC3  COMP_FLG1 */
#define IMXRT_XBAR2_IN_EQDC3_COMP_FLG2                                          IMXRT_XBAR2(XBAR_INPUT, 173)    /* XBAR2_IN173 <- eQDC3  COMP_FLG2 */
#define IMXRT_XBAR2_IN_EQDC3_COMP_FLG3                                          IMXRT_XBAR2(XBAR_INPUT, 174)    /* XBAR2_IN174 <- eQDC3  COMP_FLG3 */
#define IMXRT_XBAR2_IN_EQDC3_CNT_DN                                             IMXRT_XBAR2(XBAR_INPUT, 175)    /* XBAR2_IN175 <- eQDC3  CNT_DN */
#define IMXRT_XBAR2_IN_EQDC3_CNT_UP                                             IMXRT_XBAR2(XBAR_INPUT, 176)    /* XBAR2_IN176 <- eQDC3  CNT_UP */
#define IMXRT_XBAR2_IN_EQDC3_DIR                                                IMXRT_XBAR2(XBAR_INPUT, 177)    /* XBAR2_IN177 <- eQDC3  DIR */

#define IMXRT_XBAR3_IN_GND                                                      IMXRT_XBAR3(XBAR_INPUT, 0)      /* XBAR3_IN0 <- -  GND */
#define IMXRT_XBAR3_IN_1B1                                                      IMXRT_XBAR3(XBAR_INPUT, 1)      /* XBAR3_IN1 <- -  1'B1 */
#define IMXRT_XBAR3_IN_GND_2                                                    IMXRT_XBAR3(XBAR_INPUT, 2)      /* XBAR3_IN2 <- -  GND */
#define IMXRT_XBAR3_IN_1B1_2                                                    IMXRT_XBAR3(XBAR_INPUT, 3)      /* XBAR3_IN3 <- -  1'B1 */
#define IMXRT_XBAR3_IN_CMP1_COUT                                                IMXRT_XBAR3(XBAR_INPUT, 4)      /* XBAR3_IN4 <- CMP1  COUT */
#define IMXRT_XBAR3_IN_CMP2_COUT                                                IMXRT_XBAR3(XBAR_INPUT, 5)      /* XBAR3_IN5 <- CMP2  COUT */
#define IMXRT_XBAR3_IN_CMP3_COUT                                                IMXRT_XBAR3(XBAR_INPUT, 6)      /* XBAR3_IN6 <- CMP3  COUT */
#define IMXRT_XBAR3_IN_CMP4_COUT                                                IMXRT_XBAR3(XBAR_INPUT, 7)      /* XBAR3_IN7 <- CMP4  COUT */
#define IMXRT_XBAR3_IN_TMR1_TMR0_OUTPUT                                         IMXRT_XBAR3(XBAR_INPUT, 8)      /* XBAR3_IN8 <- TMR1  TMR0_OUTPUT */
#define IMXRT_XBAR3_IN_TMR1_TMR1_OUTPUT                                         IMXRT_XBAR3(XBAR_INPUT, 9)      /* XBAR3_IN9 <- TMR1  TMR1_OUTPUT */
#define IMXRT_XBAR3_IN_TMR1_TMR2_OUTPUT                                         IMXRT_XBAR3(XBAR_INPUT, 10)     /* XBAR3_IN10 <- TMR1  TMR2_OUTPUT */
#define IMXRT_XBAR3_IN_TMR1_TMR3_OUTPUT                                         IMXRT_XBAR3(XBAR_INPUT, 11)     /* XBAR3_IN11 <- TMR1  TMR3_OUTPUT */
#define IMXRT_XBAR3_IN_TMR2_TMR0_OUTPUT                                         IMXRT_XBAR3(XBAR_INPUT, 12)     /* XBAR3_IN12 <- TMR2  TMR0_OUTPUT */
#define IMXRT_XBAR3_IN_TMR2_TMR1_OUTPUT                                         IMXRT_XBAR3(XBAR_INPUT, 13)     /* XBAR3_IN13 <- TMR2  TMR1_OUTPUT */
#define IMXRT_XBAR3_IN_TMR2_TMR2_OUTPUT                                         IMXRT_XBAR3(XBAR_INPUT, 14)     /* XBAR3_IN14 <- TMR2  TMR2_OUTPUT */
#define IMXRT_XBAR3_IN_TMR2_TMR3_OUTPUT                                         IMXRT_XBAR3(XBAR_INPUT, 15)     /* XBAR3_IN15 <- TMR2  TMR3_OUTPUT */
#define IMXRT_XBAR3_IN_TMR3_TMR0_OUTPUT                                         IMXRT_XBAR3(XBAR_INPUT, 16)     /* XBAR3_IN16 <- TMR3  TMR0_OUTPUT */
#define IMXRT_XBAR3_IN_TMR3_TMR1_OUTPUT                                         IMXRT_XBAR3(XBAR_INPUT, 17)     /* XBAR3_IN17 <- TMR3  TMR1_OUTPUT */
#define IMXRT_XBAR3_IN_TMR3_TMR2_OUTPUT                                         IMXRT_XBAR3(XBAR_INPUT, 18)     /* XBAR3_IN18 <- TMR3  TMR2_OUTPUT */
#define IMXRT_XBAR3_IN_TMR3_TMR3_OUTPUT                                         IMXRT_XBAR3(XBAR_INPUT, 19)     /* XBAR3_IN19 <- TMR3  TMR3_OUTPUT */
#define IMXRT_XBAR3_IN_TMR4_TMR0_OUTPUT                                         IMXRT_XBAR3(XBAR_INPUT, 20)     /* XBAR3_IN20 <- TMR4  TMR0_OUTPUT */
#define IMXRT_XBAR3_IN_TMR4_TMR1_OUTPUT                                         IMXRT_XBAR3(XBAR_INPUT, 21)     /* XBAR3_IN21 <- TMR4  TMR1_OUTPUT */
#define IMXRT_XBAR3_IN_TMR4_TMR2_OUTPUT                                         IMXRT_XBAR3(XBAR_INPUT, 22)     /* XBAR3_IN22 <- TMR4  TMR2_OUTPUT */
#define IMXRT_XBAR3_IN_TMR4_TMR3_OUTPUT                                         IMXRT_XBAR3(XBAR_INPUT, 23)     /* XBAR3_IN23 <- TMR4  TMR3_OUTPUT */
#define IMXRT_XBAR3_IN_TMR5_TMR0_OUTPUT                                         IMXRT_XBAR3(XBAR_INPUT, 24)     /* XBAR3_IN24 <- TMR5  TMR0_OUTPUT */
#define IMXRT_XBAR3_IN_TMR5_TMR1_OUTPUT                                         IMXRT_XBAR3(XBAR_INPUT, 25)     /* XBAR3_IN25 <- TMR5  TMR1_OUTPUT */
#define IMXRT_XBAR3_IN_TMR5_TMR2_OUTPUT                                         IMXRT_XBAR3(XBAR_INPUT, 26)     /* XBAR3_IN26 <- TMR5  TMR2_OUTPUT */
#define IMXRT_XBAR3_IN_TMR5_TMR3_OUTPUT                                         IMXRT_XBAR3(XBAR_INPUT, 27)     /* XBAR3_IN27 <- TMR5  TMR3_OUTPUT */
#define IMXRT_XBAR3_IN_TMR6_TMR0_OUTPUT                                         IMXRT_XBAR3(XBAR_INPUT, 28)     /* XBAR3_IN28 <- TMR6  TMR0_OUTPUT */
#define IMXRT_XBAR3_IN_TMR6_TMR1_OUTPUT                                         IMXRT_XBAR3(XBAR_INPUT, 29)     /* XBAR3_IN29 <- TMR6  TMR1_OUTPUT */
#define IMXRT_XBAR3_IN_TMR6_TMR2_OUTPUT                                         IMXRT_XBAR3(XBAR_INPUT, 30)     /* XBAR3_IN30 <- TMR6  TMR2_OUTPUT */
#define IMXRT_XBAR3_IN_TMR6_TMR3_OUTPUT                                         IMXRT_XBAR3(XBAR_INPUT, 31)     /* XBAR3_IN31 <- TMR6  TMR3_OUTPUT */
#define IMXRT_XBAR3_IN_TMR7_TMR0_OUTPUT                                         IMXRT_XBAR3(XBAR_INPUT, 32)     /* XBAR3_IN32 <- TMR7  TMR0_OUTPUT */
#define IMXRT_XBAR3_IN_TMR7_TMR1_OUTPUT                                         IMXRT_XBAR3(XBAR_INPUT, 33)     /* XBAR3_IN33 <- TMR7  TMR1_OUTPUT */
#define IMXRT_XBAR3_IN_TMR7_TMR2_OUTPUT                                         IMXRT_XBAR3(XBAR_INPUT, 34)     /* XBAR3_IN34 <- TMR7  TMR2_OUTPUT */
#define IMXRT_XBAR3_IN_TMR7_TMR3_OUTPUT                                         IMXRT_XBAR3(XBAR_INPUT, 35)     /* XBAR3_IN35 <- TMR7  TMR3_OUTPUT */
#define IMXRT_XBAR3_IN_TMR8_TMR0_OUTPUT                                         IMXRT_XBAR3(XBAR_INPUT, 36)     /* XBAR3_IN36 <- TMR8  TMR0_OUTPUT */
#define IMXRT_XBAR3_IN_TMR8_TMR1_OUTPUT                                         IMXRT_XBAR3(XBAR_INPUT, 37)     /* XBAR3_IN37 <- TMR8  TMR1_OUTPUT */
#define IMXRT_XBAR3_IN_TMR8_TMR2_OUTPUT                                         IMXRT_XBAR3(XBAR_INPUT, 38)     /* XBAR3_IN38 <- TMR8  TMR2_OUTPUT */
#define IMXRT_XBAR3_IN_TMR8_TMR3_OUTPUT                                         IMXRT_XBAR3(XBAR_INPUT, 39)     /* XBAR3_IN39 <- TMR8  TMR3_OUTPUT */
#define IMXRT_XBAR3_IN_FLEXPWM1_PWM0_MUX_TRIGGER0                               IMXRT_XBAR3(XBAR_INPUT, 40)     /* XBAR3_IN40 <- FLEXPWM1  PWM0_MUX_TRIGGER0 */
#define IMXRT_XBAR3_IN_FLEXPWM1_PWM1_MUX_TRIGGER0                               IMXRT_XBAR3(XBAR_INPUT, 41)     /* XBAR3_IN41 <- FLEXPWM1  PWM1_MUX_TRIGGER0 */
#define IMXRT_XBAR3_IN_FLEXPWM1_PWM2_MUX_TRIGGER0                               IMXRT_XBAR3(XBAR_INPUT, 42)     /* XBAR3_IN42 <- FLEXPWM1  PWM2_MUX_TRIGGER0 */
#define IMXRT_XBAR3_IN_FLEXPWM1_PWM3_MUX_TRIGGER0                               IMXRT_XBAR3(XBAR_INPUT, 43)     /* XBAR3_IN43 <- FLEXPWM1  PWM3_MUX_TRIGGER0 */
#define IMXRT_XBAR3_IN_FLEXPWM2_PWM0_MUX_TRIGGER0                               IMXRT_XBAR3(XBAR_INPUT, 44)     /* XBAR3_IN44 <- FLEXPWM2  PWM0_MUX_TRIGGER0 */
#define IMXRT_XBAR3_IN_FLEXPWM2_PWM1_MUX_TRIGGER0                               IMXRT_XBAR3(XBAR_INPUT, 45)     /* XBAR3_IN45 <- FLEXPWM2  PWM1_MUX_TRIGGER0 */
#define IMXRT_XBAR3_IN_FLEXPWM2_PWM2_MUX_TRIGGER0                               IMXRT_XBAR3(XBAR_INPUT, 46)     /* XBAR3_IN46 <- FLEXPWM2  PWM2_MUX_TRIGGER0 */
#define IMXRT_XBAR3_IN_FLEXPWM2_PWM3_MUX_TRIGGER0                               IMXRT_XBAR3(XBAR_INPUT, 47)     /* XBAR3_IN47 <- FLEXPWM2  PWM3_MUX_TRIGGER0 */
#define IMXRT_XBAR3_IN_FLEXPWM3_PWM0_MUX_TRIGGER0                               IMXRT_XBAR3(XBAR_INPUT, 48)     /* XBAR3_IN48 <- FLEXPWM3  PWM0_MUX_TRIGGER0 */
#define IMXRT_XBAR3_IN_FLEXPWM3_PWM1_MUX_TRIGGER0                               IMXRT_XBAR3(XBAR_INPUT, 49)     /* XBAR3_IN49 <- FLEXPWM3  PWM1_MUX_TRIGGER0 */
#define IMXRT_XBAR3_IN_FLEXPWM3_PWM2_MUX_TRIGGER0                               IMXRT_XBAR3(XBAR_INPUT, 50)     /* XBAR3_IN50 <- FLEXPWM3  PWM2_MUX_TRIGGER0 */
#define IMXRT_XBAR3_IN_FLEXPWM3_PWM3_MUX_TRIGGER0                               IMXRT_XBAR3(XBAR_INPUT, 51)     /* XBAR3_IN51 <- FLEXPWM3  PWM3_MUX_TRIGGER0 */
#define IMXRT_XBAR3_IN_FLEXPWM4_PWM0_MUX_TRIGGER0                               IMXRT_XBAR3(XBAR_INPUT, 52)     /* XBAR3_IN52 <- FLEXPWM4  PWM0_MUX_TRIGGER0 */
#define IMXRT_XBAR3_IN_FLEXPWM4_PWM1_MUX_TRIGGER0                               IMXRT_XBAR3(XBAR_INPUT, 53)     /* XBAR3_IN53 <- FLEXPWM4  PWM1_MUX_TRIGGER0 */
#define IMXRT_XBAR3_IN_FLEXPWM4_PWM2_MUX_TRIGGER0                               IMXRT_XBAR3(XBAR_INPUT, 54)     /* XBAR3_IN54 <- FLEXPWM4  PWM2_MUX_TRIGGER0 */
#define IMXRT_XBAR3_IN_FLEXPWM4_PWM3_MUX_TRIGGER0                               IMXRT_XBAR3(XBAR_INPUT, 55)     /* XBAR3_IN55 <- FLEXPWM4  PWM3_MUX_TRIGGER0 */
#define IMXRT_XBAR3_IN_LPIT1_LPIT_TRIG_OUT0                                     IMXRT_XBAR3(XBAR_INPUT, 56)     /* XBAR3_IN56 <- LPIT1  LPIT_TRIG_OUT0 */
#define IMXRT_XBAR3_IN_LPIT1_LPIT_TRIG_OUT1                                     IMXRT_XBAR3(XBAR_INPUT, 57)     /* XBAR3_IN57 <- LPIT1  LPIT_TRIG_OUT1 */
#define IMXRT_XBAR3_IN_LPIT1_LPIT_TRIG_OUT2                                     IMXRT_XBAR3(XBAR_INPUT, 58)     /* XBAR3_IN58 <- LPIT1  LPIT_TRIG_OUT2 */
#define IMXRT_XBAR3_IN_LPIT1_LPIT_TRIG_OUT3                                     IMXRT_XBAR3(XBAR_INPUT, 59)     /* XBAR3_IN59 <- LPIT1  LPIT_TRIG_OUT3 */
#define IMXRT_XBAR3_IN_LPIT2_LPIT_TRIG_OUT0                                     IMXRT_XBAR3(XBAR_INPUT, 60)     /* XBAR3_IN60 <- LPIT2  LPIT_TRIG_OUT0 */
#define IMXRT_XBAR3_IN_LPIT2_LPIT_TRIG_OUT1                                     IMXRT_XBAR3(XBAR_INPUT, 61)     /* XBAR3_IN61 <- LPIT2  LPIT_TRIG_OUT1 */
#define IMXRT_XBAR3_IN_LPIT2_LPIT_TRIG_OUT2                                     IMXRT_XBAR3(XBAR_INPUT, 62)     /* XBAR3_IN62 <- LPIT2  LPIT_TRIG_OUT2 */
#define IMXRT_XBAR3_IN_LPIT2_LPIT_TRIG_OUT3                                     IMXRT_XBAR3(XBAR_INPUT, 63)     /* XBAR3_IN63 <- LPIT2  LPIT_TRIG_OUT3 */
#define IMXRT_XBAR3_IN_LPIT3_LPIT_TRIG_OUT0                                     IMXRT_XBAR3(XBAR_INPUT, 64)     /* XBAR3_IN64 <- LPIT3  LPIT_TRIG_OUT0 */
#define IMXRT_XBAR3_IN_LPIT3_LPIT_TRIG_OUT1                                     IMXRT_XBAR3(XBAR_INPUT, 65)     /* XBAR3_IN65 <- LPIT3  LPIT_TRIG_OUT1 */
#define IMXRT_XBAR3_IN_LPIT3_LPIT_TRIG_OUT2                                     IMXRT_XBAR3(XBAR_INPUT, 66)     /* XBAR3_IN66 <- LPIT3  LPIT_TRIG_OUT2 */
#define IMXRT_XBAR3_IN_LPIT3_LPIT_TRIG_OUT3                                     IMXRT_XBAR3(XBAR_INPUT, 67)     /* XBAR3_IN67 <- LPIT3  LPIT_TRIG_OUT3 */
#define IMXRT_XBAR3_IN_SINC2_PULSE_TRG0                                         IMXRT_XBAR3(XBAR_INPUT, 68)     /* XBAR3_IN68 <- SINC2  PULSE_TRG0 */
#define IMXRT_XBAR3_IN_SINC2_PULSE_TRG1                                         IMXRT_XBAR3(XBAR_INPUT, 69)     /* XBAR3_IN69 <- SINC2  PULSE_TRG1 */
#define IMXRT_XBAR3_IN_SINC2_PULSE_TRG2                                         IMXRT_XBAR3(XBAR_INPUT, 70)     /* XBAR3_IN70 <- SINC2  PULSE_TRG2 */
#define IMXRT_XBAR3_IN_SINC2_PULSE_TRG3                                         IMXRT_XBAR3(XBAR_INPUT, 71)     /* XBAR3_IN71 <- SINC2  PULSE_TRG3 */
#define IMXRT_XBAR3_IN_EDMA4_DMA_TRIGGER_OUT0                                   IMXRT_XBAR3(XBAR_INPUT, 72)     /* XBAR3_IN72 <- eDMA4  DMA_TRIGGER_OUT0 */
#define IMXRT_XBAR3_IN_EDMA4_DMA_TRIGGER_OUT1                                   IMXRT_XBAR3(XBAR_INPUT, 73)     /* XBAR3_IN73 <- eDMA4  DMA_TRIGGER_OUT1 */
#define IMXRT_XBAR3_IN_EDMA4_DMA_TRIGGER_OUT2                                   IMXRT_XBAR3(XBAR_INPUT, 74)     /* XBAR3_IN74 <- eDMA4  DMA_TRIGGER_OUT2 */
#define IMXRT_XBAR3_IN_EDMA4_DMA_TRIGGER_OUT3                                   IMXRT_XBAR3(XBAR_INPUT, 75)     /* XBAR3_IN75 <- eDMA4  DMA_TRIGGER_OUT3 */
#define IMXRT_XBAR3_IN_EDMA4_DMA_TRIGGER_OUT4                                   IMXRT_XBAR3(XBAR_INPUT, 76)     /* XBAR3_IN76 <- eDMA4  DMA_TRIGGER_OUT4 */
#define IMXRT_XBAR3_IN_EDMA4_DMA_TRIGGER_OUT5                                   IMXRT_XBAR3(XBAR_INPUT, 77)     /* XBAR3_IN77 <- eDMA4  DMA_TRIGGER_OUT5 */
#define IMXRT_XBAR3_IN_EDMA4_DMA_TRIGGER_OUT6                                   IMXRT_XBAR3(XBAR_INPUT, 78)     /* XBAR3_IN78 <- eDMA4  DMA_TRIGGER_OUT6 */
#define IMXRT_XBAR3_IN_EDMA4_DMA_TRIGGER_OUT7                                   IMXRT_XBAR3(XBAR_INPUT, 79)     /* XBAR3_IN79 <- eDMA4  DMA_TRIGGER_OUT7 */
#define IMXRT_XBAR3_IN_EDMA3_DMA_TRIGGER_OUT0                                   IMXRT_XBAR3(XBAR_INPUT, 80)     /* XBAR3_IN80 <- eDMA3  DMA_TRIGGER_OUT0 */
#define IMXRT_XBAR3_IN_EDMA3_DMA_TRIGGER_OUT1                                   IMXRT_XBAR3(XBAR_INPUT, 81)     /* XBAR3_IN81 <- eDMA3  DMA_TRIGGER_OUT1 */
#define IMXRT_XBAR3_IN_EDMA3_DMA_TRIGGER_OUT2                                   IMXRT_XBAR3(XBAR_INPUT, 82)     /* XBAR3_IN82 <- eDMA3  DMA_TRIGGER_OUT2 */
#define IMXRT_XBAR3_IN_EDMA3_DMA_TRIGGER_OUT3                                   IMXRT_XBAR3(XBAR_INPUT, 83)     /* XBAR3_IN83 <- eDMA3  DMA_TRIGGER_OUT3 */
#define IMXRT_XBAR3_IN_EDMA3_DMA_TRIGGER_OUT4                                   IMXRT_XBAR3(XBAR_INPUT, 84)     /* XBAR3_IN84 <- eDMA3  DMA_TRIGGER_OUT4 */
#define IMXRT_XBAR3_IN_EDMA3_DMA_TRIGGER_OUT5                                   IMXRT_XBAR3(XBAR_INPUT, 85)     /* XBAR3_IN85 <- eDMA3  DMA_TRIGGER_OUT5 */
#define IMXRT_XBAR3_IN_EDMA3_DMA_TRIGGER_OUT6                                   IMXRT_XBAR3(XBAR_INPUT, 86)     /* XBAR3_IN86 <- eDMA3  DMA_TRIGGER_OUT6 */
#define IMXRT_XBAR3_IN_EDMA3_DMA_TRIGGER_OUT7                                   IMXRT_XBAR3(XBAR_INPUT, 87)     /* XBAR3_IN87 <- eDMA3  DMA_TRIGGER_OUT7 */
#define IMXRT_XBAR3_IN_ADC1_ADC_TCOMP_PULSE0                                    IMXRT_XBAR3(XBAR_INPUT, 88)     /* XBAR3_IN88 <- ADC1  ADC_TCOMP_PULSE0 */
#define IMXRT_XBAR3_IN_ADC1_ADC_TCOMP_PULSE1                                    IMXRT_XBAR3(XBAR_INPUT, 89)     /* XBAR3_IN89 <- ADC1  ADC_TCOMP_PULSE1 */
#define IMXRT_XBAR3_IN_ADC1_ADC_TCOMP_PULSE2                                    IMXRT_XBAR3(XBAR_INPUT, 90)     /* XBAR3_IN90 <- ADC1  ADC_TCOMP_PULSE2 */
#define IMXRT_XBAR3_IN_ADC1_ADC_TCOMP_PULSE3                                    IMXRT_XBAR3(XBAR_INPUT, 91)     /* XBAR3_IN91 <- ADC1  ADC_TCOMP_PULSE3 */
#define IMXRT_XBAR3_IN_ADC1_ADC_TCOMP_PULSE4                                    IMXRT_XBAR3(XBAR_INPUT, 92)     /* XBAR3_IN92 <- ADC1  ADC_TCOMP_PULSE4 */
#define IMXRT_XBAR3_IN_ADC1_ADC_TCOMP_PULSE5                                    IMXRT_XBAR3(XBAR_INPUT, 93)     /* XBAR3_IN93 <- ADC1  ADC_TCOMP_PULSE5 */
#define IMXRT_XBAR3_IN_ADC1_ADC_TCOMP_PULSE6                                    IMXRT_XBAR3(XBAR_INPUT, 94)     /* XBAR3_IN94 <- ADC1  ADC_TCOMP_PULSE6 */
#define IMXRT_XBAR3_IN_ADC1_ADC_TCOMP_PULSE7                                    IMXRT_XBAR3(XBAR_INPUT, 95)     /* XBAR3_IN95 <- ADC1  ADC_TCOMP_PULSE7 */
#define IMXRT_XBAR3_IN_ADC2_ADC_TCOMP_PULSE0                                    IMXRT_XBAR3(XBAR_INPUT, 96)     /* XBAR3_IN96 <- ADC2  ADC_TCOMP_PULSE0 */
#define IMXRT_XBAR3_IN_ADC2_ADC_TCOMP_PULSE1                                    IMXRT_XBAR3(XBAR_INPUT, 97)     /* XBAR3_IN97 <- ADC2  ADC_TCOMP_PULSE1 */
#define IMXRT_XBAR3_IN_ADC2_ADC_TCOMP_PULSE2                                    IMXRT_XBAR3(XBAR_INPUT, 98)     /* XBAR3_IN98 <- ADC2  ADC_TCOMP_PULSE2 */
#define IMXRT_XBAR3_IN_ADC2_ADC_TCOMP_PULSE3                                    IMXRT_XBAR3(XBAR_INPUT, 99)     /* XBAR3_IN99 <- ADC2  ADC_TCOMP_PULSE3 */
#define IMXRT_XBAR3_IN_ADC2_ADC_TCOMP_PULSE4                                    IMXRT_XBAR3(XBAR_INPUT, 100)    /* XBAR3_IN100 <- ADC2  ADC_TCOMP_PULSE4 */
#define IMXRT_XBAR3_IN_ADC2_ADC_TCOMP_PULSE5                                    IMXRT_XBAR3(XBAR_INPUT, 101)    /* XBAR3_IN101 <- ADC2  ADC_TCOMP_PULSE5 */
#define IMXRT_XBAR3_IN_ADC2_ADC_TCOMP_PULSE6                                    IMXRT_XBAR3(XBAR_INPUT, 102)    /* XBAR3_IN102 <- ADC2  ADC_TCOMP_PULSE6 */
#define IMXRT_XBAR3_IN_ADC2_ADC_TCOMP_PULSE7                                    IMXRT_XBAR3(XBAR_INPUT, 103)    /* XBAR3_IN103 <- ADC2  ADC_TCOMP_PULSE7 */
#define IMXRT_XBAR3_IN_TPM1_TPM_CH_TRIGGER0                                     IMXRT_XBAR3(XBAR_INPUT, 104)    /* XBAR3_IN104 <- TPM1  TPM_CH_TRIGGER0 */
#define IMXRT_XBAR3_IN_TPM1_TPM_CH_TRIGGER1                                     IMXRT_XBAR3(XBAR_INPUT, 105)    /* XBAR3_IN105 <- TPM1  TPM_CH_TRIGGER1 */
#define IMXRT_XBAR3_IN_TPM1_TPM_CH_TRIGGER2                                     IMXRT_XBAR3(XBAR_INPUT, 106)    /* XBAR3_IN106 <- TPM1  TPM_CH_TRIGGER2 */
#define IMXRT_XBAR3_IN_TPM1_TPM_CH_TRIGGER3                                     IMXRT_XBAR3(XBAR_INPUT, 107)    /* XBAR3_IN107 <- TPM1  TPM_CH_TRIGGER3 */
#define IMXRT_XBAR3_IN_TPM1_TPM_TRIGGER                                         IMXRT_XBAR3(XBAR_INPUT, 108)    /* XBAR3_IN108 <- TPM1  TPM_TRIGGER */
#define IMXRT_XBAR3_IN_TPM2_TPM_CH_TRIGGER0                                     IMXRT_XBAR3(XBAR_INPUT, 109)    /* XBAR3_IN109 <- TPM2  TPM_CH_TRIGGER0 */
#define IMXRT_XBAR3_IN_TPM2_TPM_CH_TRIGGER1                                     IMXRT_XBAR3(XBAR_INPUT, 110)    /* XBAR3_IN110 <- TPM2  TPM_CH_TRIGGER1 */
#define IMXRT_XBAR3_IN_TPM2_TPM_CH_TRIGGER2                                     IMXRT_XBAR3(XBAR_INPUT, 111)    /* XBAR3_IN111 <- TPM2  TPM_CH_TRIGGER2 */
#define IMXRT_XBAR3_IN_TPM2_TPM_CH_TRIGGER3                                     IMXRT_XBAR3(XBAR_INPUT, 112)    /* XBAR3_IN112 <- TPM2  TPM_CH_TRIGGER3 */
#define IMXRT_XBAR3_IN_TPM2_TPM_TRIGGER                                         IMXRT_XBAR3(XBAR_INPUT, 113)    /* XBAR3_IN113 <- TPM2  TPM_TRIGGER */
#define IMXRT_XBAR3_IN_TPM3_TPM_CH_TRIGGER0                                     IMXRT_XBAR3(XBAR_INPUT, 114)    /* XBAR3_IN114 <- TPM3  TPM_CH_TRIGGER0 */
#define IMXRT_XBAR3_IN_TPM3_TPM_CH_TRIGGER1                                     IMXRT_XBAR3(XBAR_INPUT, 115)    /* XBAR3_IN115 <- TPM3  TPM_CH_TRIGGER1 */
#define IMXRT_XBAR3_IN_TPM3_TPM_CH_TRIGGER2                                     IMXRT_XBAR3(XBAR_INPUT, 116)    /* XBAR3_IN116 <- TPM3  TPM_CH_TRIGGER2 */
#define IMXRT_XBAR3_IN_TPM3_TPM_CH_TRIGGER3                                     IMXRT_XBAR3(XBAR_INPUT, 117)    /* XBAR3_IN117 <- TPM3  TPM_CH_TRIGGER3 */
#define IMXRT_XBAR3_IN_TPM3_TPM_TRIGGER                                         IMXRT_XBAR3(XBAR_INPUT, 118)    /* XBAR3_IN118 <- TPM3  TPM_TRIGGER */
#define IMXRT_XBAR3_IN_TPM4_TPM_CH_TRIGGER0                                     IMXRT_XBAR3(XBAR_INPUT, 119)    /* XBAR3_IN119 <- TPM4  TPM_CH_TRIGGER0 */
#define IMXRT_XBAR3_IN_TPM4_TPM_CH_TRIGGER1                                     IMXRT_XBAR3(XBAR_INPUT, 120)    /* XBAR3_IN120 <- TPM4  TPM_CH_TRIGGER1 */
#define IMXRT_XBAR3_IN_TPM4_TPM_CH_TRIGGER2                                     IMXRT_XBAR3(XBAR_INPUT, 121)    /* XBAR3_IN121 <- TPM4  TPM_CH_TRIGGER2 */
#define IMXRT_XBAR3_IN_TPM4_TPM_CH_TRIGGER3                                     IMXRT_XBAR3(XBAR_INPUT, 122)    /* XBAR3_IN122 <- TPM4  TPM_CH_TRIGGER3 */
#define IMXRT_XBAR3_IN_TPM4_TPM_TRIGGER                                         IMXRT_XBAR3(XBAR_INPUT, 123)    /* XBAR3_IN123 <- TPM4  TPM_TRIGGER */
#define IMXRT_XBAR3_IN_TPM5_TPM_CH_TRIGGER0                                     IMXRT_XBAR3(XBAR_INPUT, 124)    /* XBAR3_IN124 <- TPM5  TPM_CH_TRIGGER0 */
#define IMXRT_XBAR3_IN_TPM5_TPM_CH_TRIGGER1                                     IMXRT_XBAR3(XBAR_INPUT, 125)    /* XBAR3_IN125 <- TPM5  TPM_CH_TRIGGER1 */
#define IMXRT_XBAR3_IN_TPM5_TPM_CH_TRIGGER2                                     IMXRT_XBAR3(XBAR_INPUT, 126)    /* XBAR3_IN126 <- TPM5  TPM_CH_TRIGGER2 */
#define IMXRT_XBAR3_IN_TPM5_TPM_CH_TRIGGER3                                     IMXRT_XBAR3(XBAR_INPUT, 127)    /* XBAR3_IN127 <- TPM5  TPM_CH_TRIGGER3 */
#define IMXRT_XBAR3_IN_TPM5_TPM_TRIGGER                                         IMXRT_XBAR3(XBAR_INPUT, 128)    /* XBAR3_IN128 <- TPM5  TPM_TRIGGER */
#define IMXRT_XBAR3_IN_TPM6_TPM_CH_TRIGGER0                                     IMXRT_XBAR3(XBAR_INPUT, 129)    /* XBAR3_IN129 <- TPM6  TPM_CH_TRIGGER0 */
#define IMXRT_XBAR3_IN_TPM6_TPM_CH_TRIGGER1                                     IMXRT_XBAR3(XBAR_INPUT, 130)    /* XBAR3_IN130 <- TPM6  TPM_CH_TRIGGER1 */
#define IMXRT_XBAR3_IN_TPM6_TPM_CH_TRIGGER2                                     IMXRT_XBAR3(XBAR_INPUT, 131)    /* XBAR3_IN131 <- TPM6  TPM_CH_TRIGGER2 */
#define IMXRT_XBAR3_IN_TPM6_TPM_CH_TRIGGER3                                     IMXRT_XBAR3(XBAR_INPUT, 132)    /* XBAR3_IN132 <- TPM6  TPM_CH_TRIGGER3 */
#define IMXRT_XBAR3_IN_TPM6_TPM_TRIGGER                                         IMXRT_XBAR3(XBAR_INPUT, 133)    /* XBAR3_IN133 <- TPM6  TPM_TRIGGER */
#define IMXRT_XBAR3_IN_LPTMR1_LPTIMER_TRIGGER_DELAY                             IMXRT_XBAR3(XBAR_INPUT, 134)    /* XBAR3_IN134 <- LPTMR1  LPTIMER_TRIGGER_DELAY */
#define IMXRT_XBAR3_IN_LPTMR2_LPTIMER_TRIGGER_DELAY                             IMXRT_XBAR3(XBAR_INPUT, 135)    /* XBAR3_IN135 <- LPTMR2  LPTIMER_TRIGGER_DELAY */
#define IMXRT_XBAR3_IN_LPTMR3_LPTIMER_TRIGGER_DELAY                             IMXRT_XBAR3(XBAR_INPUT, 136)    /* XBAR3_IN136 <- LPTMR3  LPTIMER_TRIGGER_DELAY */
#define IMXRT_XBAR3_IN_NETC_TMR_1588_PP1                                        IMXRT_XBAR3(XBAR_INPUT, 137)    /* XBAR3_IN137 <- NETC  TMR_1588_PP1 */
#define IMXRT_XBAR3_IN_NETC_TMR_1588_PP2                                        IMXRT_XBAR3(XBAR_INPUT, 138)    /* XBAR3_IN138 <- NETC  TMR_1588_PP2 */
#define IMXRT_XBAR3_IN_NETC_TMR_1588_PP3                                        IMXRT_XBAR3(XBAR_INPUT, 139)    /* XBAR3_IN139 <- NETC  TMR_1588_PP3 */
#define IMXRT_XBAR3_IN_SINC1_CHANNEL_0_BREAK_DUE_TO_CLOCK_ABSENCE_DETECTED      IMXRT_XBAR3(XBAR_INPUT, 140)    /* XBAR3_IN140 <- SINC1  Channel 0 break due to clock absence detected */
#define IMXRT_XBAR3_IN_SINC1_CHANNEL_1_BREAK_DUE_TO_CLOCK_ABSENCE_DETECTED      IMXRT_XBAR3(XBAR_INPUT, 141)    /* XBAR3_IN141 <- SINC1  Channel 1 break due to clock absence detected */
#define IMXRT_XBAR3_IN_SINC1_CHANNEL_2_BREAK_DUE_TO_CLOCK_ABSENCE_DETECTED      IMXRT_XBAR3(XBAR_INPUT, 142)    /* XBAR3_IN142 <- SINC1  Channel 2 break due to clock absence detected */
#define IMXRT_XBAR3_IN_SINC1_CHANNEL_3_BREAK_DUE_TO_CLOCK_ABSENCE_DETECTED      IMXRT_XBAR3(XBAR_INPUT, 143)    /* XBAR3_IN143 <- SINC1  Channel 3 break due to clock absence detected */
#define IMXRT_XBAR3_IN_SINC2_CHANNEL_0_BREAK_DUE_TO_CLOCK_ABSENCE_DETECTED      IMXRT_XBAR3(XBAR_INPUT, 144)    /* XBAR3_IN144 <- SINC2  Channel 0 break due to clock absence detected */
#define IMXRT_XBAR3_IN_SINC2_CHANNEL_1_BREAK_DUE_TO_CLOCK_ABSENCE_DETECTED      IMXRT_XBAR3(XBAR_INPUT, 145)    /* XBAR3_IN145 <- SINC2  Channel 1 break due to clock absence detected */
#define IMXRT_XBAR3_IN_SINC2_CHANNEL_2_BREAK_DUE_TO_CLOCK_ABSENCE_DETECTED      IMXRT_XBAR3(XBAR_INPUT, 146)    /* XBAR3_IN146 <- SINC2  Channel 2 break due to clock absence detected */
#define IMXRT_XBAR3_IN_SINC2_CHANNEL_3_BREAK_DUE_TO_CLOCK_ABSENCE_DETECTED      IMXRT_XBAR3(XBAR_INPUT, 147)    /* XBAR3_IN147 <- SINC2  Channel 3 break due to clock absence detected */
#define IMXRT_XBAR3_IN_SINC3_CHANNEL_0_BREAK_DUE_TO_CLOCK_ABSENCE_DETECTED      IMXRT_XBAR3(XBAR_INPUT, 148)    /* XBAR3_IN148 <- SINC3  Channel 0 break due to clock absence detected */
#define IMXRT_XBAR3_IN_SINC3_CHANNEL_1_BREAK_DUE_TO_CLOCK_ABSENCE_DETECTED      IMXRT_XBAR3(XBAR_INPUT, 149)    /* XBAR3_IN149 <- SINC3  Channel 1 break due to clock absence detected */
#define IMXRT_XBAR3_IN_SINC3_CHANNEL_2_BREAK_DUE_TO_CLOCK_ABSENCE_DETECTED      IMXRT_XBAR3(XBAR_INPUT, 150)    /* XBAR3_IN150 <- SINC3  Channel 2 break due to clock absence detected */
#define IMXRT_XBAR3_IN_SINC3_CHANNEL_3_BREAK_DUE_TO_CLOCK_ABSENCE_DETECTED      IMXRT_XBAR3(XBAR_INPUT, 151)    /* XBAR3_IN151 <- SINC3  Channel 3 break due to clock absence detected */
#define IMXRT_XBAR3_IN_RESERVED                                                 IMXRT_XBAR3(XBAR_INPUT, 152)    /* XBAR3_IN152 <- -  Reserved */
#define IMXRT_XBAR3_IN_RESERVED_2                                               IMXRT_XBAR3(XBAR_INPUT, 153)    /* XBAR3_IN153 <- -  Reserved */
#define IMXRT_XBAR3_IN_RESERVED_3                                               IMXRT_XBAR3(XBAR_INPUT, 154)    /* XBAR3_IN154 <- -  Reserved */
#define IMXRT_XBAR3_IN_RESERVED_4                                               IMXRT_XBAR3(XBAR_INPUT, 155)    /* XBAR3_IN155 <- -  Reserved */
#define IMXRT_XBAR3_IN_EQDC2_POS_MATCH0                                         IMXRT_XBAR3(XBAR_INPUT, 156)    /* XBAR3_IN156 <- eQDC2  POS_MATCH0 */
#define IMXRT_XBAR3_IN_EQDC2_POS_MATCH1                                         IMXRT_XBAR3(XBAR_INPUT, 157)    /* XBAR3_IN157 <- eQDC2  POS_MATCH1 */
#define IMXRT_XBAR3_IN_EQDC2_POS_MATCH2                                         IMXRT_XBAR3(XBAR_INPUT, 158)    /* XBAR3_IN158 <- eQDC2  POS_MATCH2 */
#define IMXRT_XBAR3_IN_EQDC2_POS_MATCH3                                         IMXRT_XBAR3(XBAR_INPUT, 159)    /* XBAR3_IN159 <- eQDC2  POS_MATCH3 */
#define IMXRT_XBAR3_IN_EQDC2_COMP_FLG0                                          IMXRT_XBAR3(XBAR_INPUT, 160)    /* XBAR3_IN160 <- eQDC2  COMP_FLG0 */
#define IMXRT_XBAR3_IN_EQDC2_COMP_FLG1                                          IMXRT_XBAR3(XBAR_INPUT, 161)    /* XBAR3_IN161 <- eQDC2  COMP_FLG1 */
#define IMXRT_XBAR3_IN_EQDC2_COMP_FLG2                                          IMXRT_XBAR3(XBAR_INPUT, 162)    /* XBAR3_IN162 <- eQDC2  COMP_FLG2 */
#define IMXRT_XBAR3_IN_EQDC2_COMP_FLG3                                          IMXRT_XBAR3(XBAR_INPUT, 163)    /* XBAR3_IN163 <- eQDC2  COMP_FLG3 */
#define IMXRT_XBAR3_IN_EQDC2_CNT_DN                                             IMXRT_XBAR3(XBAR_INPUT, 164)    /* XBAR3_IN164 <- eQDC2  CNT_DN */
#define IMXRT_XBAR3_IN_EQDC2_CNT_UP                                             IMXRT_XBAR3(XBAR_INPUT, 165)    /* XBAR3_IN165 <- eQDC2  CNT_UP */
#define IMXRT_XBAR3_IN_EQDC2_DIR                                                IMXRT_XBAR3(XBAR_INPUT, 166)    /* XBAR3_IN166 <- eQDC2  DIR */
#define IMXRT_XBAR3_IN_EQDC4_POS_MATCH0                                         IMXRT_XBAR3(XBAR_INPUT, 167)    /* XBAR3_IN167 <- eQDC4  POS_MATCH0 */
#define IMXRT_XBAR3_IN_EQDC4_POS_MATCH1                                         IMXRT_XBAR3(XBAR_INPUT, 168)    /* XBAR3_IN168 <- eQDC4  POS_MATCH1 */
#define IMXRT_XBAR3_IN_EQDC4_POS_MATCH2                                         IMXRT_XBAR3(XBAR_INPUT, 169)    /* XBAR3_IN169 <- eQDC4  POS_MATCH2 */
#define IMXRT_XBAR3_IN_EQDC4_POS_MATCH3                                         IMXRT_XBAR3(XBAR_INPUT, 170)    /* XBAR3_IN170 <- eQDC4  POS_MATCH3 */
#define IMXRT_XBAR3_IN_EQDC4_COMP_FLG0                                          IMXRT_XBAR3(XBAR_INPUT, 171)    /* XBAR3_IN171 <- eQDC4  COMP_FLG0 */
#define IMXRT_XBAR3_IN_EQDC4_COMP_FLG1                                          IMXRT_XBAR3(XBAR_INPUT, 172)    /* XBAR3_IN172 <- eQDC4  COMP_FLG1 */
#define IMXRT_XBAR3_IN_EQDC4_COMP_FLG2                                          IMXRT_XBAR3(XBAR_INPUT, 173)    /* XBAR3_IN173 <- eQDC4  COMP_FLG2 */
#define IMXRT_XBAR3_IN_EQDC4_COMP_FLG3                                          IMXRT_XBAR3(XBAR_INPUT, 174)    /* XBAR3_IN174 <- eQDC4  COMP_FLG3 */
#define IMXRT_XBAR3_IN_EQDC4_CNT_DN                                             IMXRT_XBAR3(XBAR_INPUT, 175)    /* XBAR3_IN175 <- eQDC4  CNT_DN */
#define IMXRT_XBAR3_IN_EQDC4_CNT_UP                                             IMXRT_XBAR3(XBAR_INPUT, 176)    /* XBAR3_IN176 <- eQDC4  CNT_UP */
#define IMXRT_XBAR3_IN_EQDC4_DIR                                                IMXRT_XBAR3(XBAR_INPUT, 177)    /* XBAR3_IN177 <- eQDC4  DIR */

/****************************************************************************
 * XBAR Output Assignments (IMXRT1180RM Table 15)
 ****************************************************************************/

#define IMXRT_XBAR1_OUT_EDMA4_DMA_MUX_REQ154            IMXRT_XBAR1(XBAR_OUTPUT, 0)     /* XBAR1_OUT0 -> eDMA4  DMA_MUX_REQ154 */
#define IMXRT_XBAR1_OUT_EDMA4_DMA_MUX_REQ155            IMXRT_XBAR1(XBAR_OUTPUT, 1)     /* XBAR1_OUT1 -> eDMA4  DMA_MUX_REQ155 */
#define IMXRT_XBAR1_OUT_EDMA4_DMA_MUX_REQ156            IMXRT_XBAR1(XBAR_OUTPUT, 2)     /* XBAR1_OUT2 -> eDMA4  DMA_MUX_REQ156 */
#define IMXRT_XBAR1_OUT_EDMA4_DMA_MUX_REQ157            IMXRT_XBAR1(XBAR_OUTPUT, 3)     /* XBAR1_OUT3 -> eDMA4  DMA_MUX_REQ157 */
#define IMXRT_XBAR1_OUT_IOMUX_XBAR_INOUT04              IMXRT_XBAR1(XBAR_OUTPUT, 4)     /* XBAR1_OUT4 -> -  SW_MUX_CTL_PAD_GPIO_EMC_B1_ 00[MUX_MODE] = 0x1 */
#define IMXRT_XBAR1_OUT_IOMUX_XBAR_INOUT05              IMXRT_XBAR1(XBAR_OUTPUT, 5)     /* XBAR1_OUT5 -> -  SW_MUX_CTL_PAD_GPIO_EMC_B1_ 01[MUX_MODE] = 0x1 */
#define IMXRT_XBAR1_OUT_IOMUX_XBAR_INOUT06              IMXRT_XBAR1(XBAR_OUTPUT, 6)     /* XBAR1_OUT6 -> -  SW_MUX_CTL_PAD_GPIO_EMC_B1_ 02[MUX_MODE] = 0x1 */
#define IMXRT_XBAR1_OUT_IOMUX_XBAR_INOUT07              IMXRT_XBAR1(XBAR_OUTPUT, 7)     /* XBAR1_OUT7 -> -  SW_MUX_CTL_PAD_GPIO_EMC_B1_ 03[MUX_MODE] = 0x1 */
#define IMXRT_XBAR1_OUT_IOMUX_XBAR_INOUT08              IMXRT_XBAR1(XBAR_OUTPUT, 8)     /* XBAR1_OUT8 -> -  SW_MUX_CTL_PAD_GPIO_EMC_B1_ 04[MUX_MODE] = 0x1 */
#define IMXRT_XBAR1_OUT_IOMUX_XBAR_INOUT09              IMXRT_XBAR1(XBAR_OUTPUT, 9)     /* XBAR1_OUT9 -> -  SW_MUX_CTL_PAD_GPIO_EMC_B1_ 05[MUX_MODE] = 0x1 */
#define IMXRT_XBAR1_OUT_IOMUX_XBAR_INOUT10              IMXRT_XBAR1(XBAR_OUTPUT, 10)    /* XBAR1_OUT10 -> -  SW_MUX_CTL_PAD_GPIO_EMC_B1_ 26[MUX_MODE] = 0x2 */
#define IMXRT_XBAR1_OUT_IOMUX_XBAR_INOUT11              IMXRT_XBAR1(XBAR_OUTPUT, 11)    /* XBAR1_OUT11 -> -  SW_MUX_CTL_PAD_GPIO_EMC_B1_ 27[MUX_MODE] = 0x2 */
#define IMXRT_XBAR1_OUT_IOMUX_XBAR_INOUT12              IMXRT_XBAR1(XBAR_OUTPUT, 12)    /* XBAR1_OUT12 -> -  SW_MUX_CTL_PAD_GPIO_EMC_B1_ 28[MUX_MODE] = 0x2 */
#define IMXRT_XBAR1_OUT_IOMUX_XBAR_INOUT13              IMXRT_XBAR1(XBAR_OUTPUT, 13)    /* XBAR1_OUT13 -> -  SW_MUX_CTL_PAD_GPIO_EMC_B1_ 29[MUX_MODE] = 0x2 */
#define IMXRT_XBAR1_OUT_IOMUX_XBAR_INOUT14              IMXRT_XBAR1(XBAR_OUTPUT, 14)    /* XBAR1_OUT14 -> -  SW_MUX_CTL_PAD_GPIO_EMC_B1_ 30[MUX_MODE] = 0x2 */
#define IMXRT_XBAR1_OUT_IOMUX_XBAR_INOUT15              IMXRT_XBAR1(XBAR_OUTPUT, 15)    /* XBAR1_OUT15 -> -  SW_MUX_CTL_PAD_GPIO_EMC_B1_ 39[MUX_MODE] = 0x2 */
#define IMXRT_XBAR1_OUT_IOMUX_XBAR_INOUT16              IMXRT_XBAR1(XBAR_OUTPUT, 16)    /* XBAR1_OUT16 -> -  SW_MUX_CTL_PAD_GPIO_EMC_B2_ 18[MUX_MODE] = 0x6 */
#define IMXRT_XBAR1_OUT_IOMUX_XBAR_INOUT17              IMXRT_XBAR1(XBAR_OUTPUT, 17)    /* XBAR1_OUT17 -> -  SW_MUX_CTL_PAD_GPIO_AD_33[MU See IOMUX Controller X_MODE] = */
#define IMXRT_XBAR1_OUT_IOMUX_XBAR_INOUT18              IMXRT_XBAR1(XBAR_OUTPUT, 18)    /* XBAR1_OUT18 -> -  SW_MUX_CTL_PAD_GPIO_AD_12[MU See IOMUX Controller X_MODE] = */
#define IMXRT_XBAR1_OUT_IOMUX_XBAR_INOUT19              IMXRT_XBAR1(XBAR_OUTPUT, 19)    /* XBAR1_OUT19 -> -  SW_MUX_CTL_PAD_GPIO_AD_19[MU See IOMUX Controller X_MODE] = */
#define IMXRT_XBAR1_OUT_IOMUX_XBAR_INOUT20              IMXRT_XBAR1(XBAR_OUTPUT, 20)    /* XBAR1_OUT20 -> -  SW_MUX_CTL_PAD_GPIO_EMC_B2_ 00[MUX_MODE] = 0x6 */
#define IMXRT_XBAR1_OUT_IOMUX_XBAR_INOUT21              IMXRT_XBAR1(XBAR_OUTPUT, 21)    /* XBAR1_OUT21 -> -  SW_MUX_CTL_PAD_GPIO_EMC_B2_ 01[MUX_MODE] = 0x6 */
#define IMXRT_XBAR1_OUT_IOMUX_XBAR_INOUT22              IMXRT_XBAR1(XBAR_OUTPUT, 22)    /* XBAR1_OUT22 -> -  SW_MUX_CTL_PAD_GPIO_EMC_B2_ 02[MUX_MODE] = 0x6 */
#define IMXRT_XBAR1_OUT_IOMUX_XBAR_INOUT23              IMXRT_XBAR1(XBAR_OUTPUT, 23)    /* XBAR1_OUT23 -> -  SW_MUX_CTL_PAD_GPIO_EMC_B2_ 03[MUX_MODE] = 0x6 */
#define IMXRT_XBAR1_OUT_IOMUX_XBAR_INOUT24              IMXRT_XBAR1(XBAR_OUTPUT, 24)    /* XBAR1_OUT24 -> -  SW_MUX_CTL_PAD_GPIO_EMC_B2_ 04[MUX_MODE] = 0x6 */
#define IMXRT_XBAR1_OUT_IOMUX_XBAR_INOUT25              IMXRT_XBAR1(XBAR_OUTPUT, 25)    /* XBAR1_OUT25 -> -  SW_MUX_CTL_PAD_GPIO_EMC_B2_ 05[MUX_MODE] = 0x6 */
#define IMXRT_XBAR1_OUT_IOMUX_XBAR_INOUT26              IMXRT_XBAR1(XBAR_OUTPUT, 26)    /* XBAR1_OUT26 -> -  SW_MUX_CTL_PAD_GPIO_EMC_B2_ 06[MUX_MODE] = 0x6 */
#define IMXRT_XBAR1_OUT_IOMUX_XBAR_INOUT27              IMXRT_XBAR1(XBAR_OUTPUT, 27)    /* XBAR1_OUT27 -> -  SW_MUX_CTL_PAD_GPIO_EMC_B2_ 07[MUX_MODE] = 0x6 */
#define IMXRT_XBAR1_OUT_IOMUX_XBAR_INOUT28              IMXRT_XBAR1(XBAR_OUTPUT, 28)    /* XBAR1_OUT28 -> -  SW_MUX_CTL_PAD_GPIO_EMC_B2_ 08[MUX_MODE] = 0x6 */
#define IMXRT_XBAR1_OUT_IOMUX_XBAR_INOUT29              IMXRT_XBAR1(XBAR_OUTPUT, 29)    /* XBAR1_OUT29 -> -  SW_MUX_CTL_PAD_GPIO_EMC_B2_ 09[MUX_MODE] = 0x6 */
#define IMXRT_XBAR1_OUT_IOMUX_XBAR_INOUT30              IMXRT_XBAR1(XBAR_OUTPUT, 30)    /* XBAR1_OUT30 -> -  SW_MUX_CTL_PAD_GPIO_EMC_B2_ 10[MUX_MODE] = 0x6 */
#define IMXRT_XBAR1_OUT_IOMUX_XBAR_INOUT31              IMXRT_XBAR1(XBAR_OUTPUT, 31)    /* XBAR1_OUT31 -> -  SW_MUX_CTL_PAD_GPIO_EMC_B2_ 11[MUX_MODE] = 0x6 */
#define IMXRT_XBAR1_OUT_IOMUX_XBAR_INOUT32              IMXRT_XBAR1(XBAR_OUTPUT, 32)    /* XBAR1_OUT32 -> -  SW_MUX_CTL_PAD_GPIO_EMC_B2_ 12[MUX_MODE] = 0x6 */
#define IMXRT_XBAR1_OUT_IOMUX_XBAR_INOUT33              IMXRT_XBAR1(XBAR_OUTPUT, 33)    /* XBAR1_OUT33 -> -  SW_MUX_CTL_PAD_GPIO_EMC_B2_ 13[MUX_MODE] = 0x6 */
#define IMXRT_XBAR1_OUT_IOMUX_XBAR_INOUT34              IMXRT_XBAR1(XBAR_OUTPUT, 34)    /* XBAR1_OUT34 -> -  SW_MUX_CTL_PAD_GPIO_EMC_B2_ 14[MUX_MODE] = 0x6 */
#define IMXRT_XBAR1_OUT_IOMUX_XBAR_INOUT35              IMXRT_XBAR1(XBAR_OUTPUT, 35)    /* XBAR1_OUT35 -> -  SW_MUX_CTL_PAD_GPIO_EMC_B2_ 15[MUX_MODE] = 0x6 */
#define IMXRT_XBAR1_OUT_IOMUX_XBAR_INOUT36              IMXRT_XBAR1(XBAR_OUTPUT, 36)    /* XBAR1_OUT36 -> -  SW_MUX_CTL_PAD_GPIO_EMC_B2_ 19[MUX_MODE] = 0x6 */
#define IMXRT_XBAR1_OUT_IOMUX_XBAR_INOUT37              IMXRT_XBAR1(XBAR_OUTPUT, 37)    /* XBAR1_OUT37 -> -  SW_MUX_CTL_PAD_GPIO_EMC_B2_ 20[MUX_MODE] = 0x6 */
#define IMXRT_XBAR1_OUT_CMP1_SAMPLE                     IMXRT_XBAR1(XBAR_OUTPUT, 38)    /* XBAR1_OUT38 -> CMP1  SAMPLE */
#define IMXRT_XBAR1_OUT_CMP2_SAMPLE                     IMXRT_XBAR1(XBAR_OUTPUT, 39)    /* XBAR1_OUT39 -> CMP2  SAMPLE */
#define IMXRT_XBAR1_OUT_CMP3_SAMPLE                     IMXRT_XBAR1(XBAR_OUTPUT, 40)    /* XBAR1_OUT40 -> CMP3  SAMPLE */
#define IMXRT_XBAR1_OUT_CMP4_SAMPLE                     IMXRT_XBAR1(XBAR_OUTPUT, 41)    /* XBAR1_OUT41 -> CMP4  SAMPLE */
#define IMXRT_XBAR1_OUT_FLEXPWM1_EXTA0                  IMXRT_XBAR1(XBAR_OUTPUT, 42)    /* XBAR1_OUT42 -> FLEXPWM1  EXTA0 */
#define IMXRT_XBAR1_OUT_FLEXPWM1_EXTA1                  IMXRT_XBAR1(XBAR_OUTPUT, 43)    /* XBAR1_OUT43 -> FLEXPWM1  EXTA1 */
#define IMXRT_XBAR1_OUT_FLEXPWM1_EXTA2                  IMXRT_XBAR1(XBAR_OUTPUT, 44)    /* XBAR1_OUT44 -> FLEXPWM1  EXTA2 */
#define IMXRT_XBAR1_OUT_FLEXPWM1_EXTA3                  IMXRT_XBAR1(XBAR_OUTPUT, 45)    /* XBAR1_OUT45 -> FLEXPWM1  EXTA3 */
#define IMXRT_XBAR1_OUT_FLEXPWM1_EXT_SYNC0              IMXRT_XBAR1(XBAR_OUTPUT, 46)    /* XBAR1_OUT46 -> FLEXPWM1  EXT_SYNC0 */
#define IMXRT_XBAR1_OUT_FLEXPWM1_EXT_SYNC1              IMXRT_XBAR1(XBAR_OUTPUT, 47)    /* XBAR1_OUT47 -> FLEXPWM1  EXT_SYNC1 */
#define IMXRT_XBAR1_OUT_FLEXPWM1_EXT_SYNC2              IMXRT_XBAR1(XBAR_OUTPUT, 48)    /* XBAR1_OUT48 -> FLEXPWM1  EXT_SYNC2 */
#define IMXRT_XBAR1_OUT_FLEXPWM1_EXT_SYNC3              IMXRT_XBAR1(XBAR_OUTPUT, 49)    /* XBAR1_OUT49 -> FLEXPWM1  EXT_SYNC3 */
#define IMXRT_XBAR1_OUT_FLEXPWM1_EXT_CLK                IMXRT_XBAR1(XBAR_OUTPUT, 50)    /* XBAR1_OUT50 -> FLEXPWM1  EXT_CLK */
#define IMXRT_XBAR1_OUT_FLEXPWM1_FAULT0                 IMXRT_XBAR1(XBAR_OUTPUT, 51)    /* XBAR1_OUT51 -> FLEXPWM1  FAULT0 */
#define IMXRT_XBAR1_OUT_FLEXPWM1_FAULT1                 IMXRT_XBAR1(XBAR_OUTPUT, 52)    /* XBAR1_OUT52 -> FLEXPWM1  FAULT1 */
#define IMXRT_XBAR1_OUT_FLEXPWM1_FAULT2                 IMXRT_XBAR1(XBAR_OUTPUT, 53)    /* XBAR1_OUT53 -> FLEXPWM1  FAULT2 */
#define IMXRT_XBAR1_OUT_FLEXPWM1_FAULT3                 IMXRT_XBAR1(XBAR_OUTPUT, 54)    /* XBAR1_OUT54 -> FLEXPWM1  FAULT3 */
#define IMXRT_XBAR1_OUT_FLEXPWM1_EXT_FORCE              IMXRT_XBAR1(XBAR_OUTPUT, 55)    /* XBAR1_OUT55 -> FLEXPWM1  EXT_FORCE */
#define IMXRT_XBAR1_OUT_FLEXPWM2_EXTA0                  IMXRT_XBAR1(XBAR_OUTPUT, 56)    /* XBAR1_OUT56 -> FLEXPWM2  EXTA0 */
#define IMXRT_XBAR1_OUT_FLEXPWM2_EXTA1                  IMXRT_XBAR1(XBAR_OUTPUT, 57)    /* XBAR1_OUT57 -> FLEXPWM2  EXTA1 */
#define IMXRT_XBAR1_OUT_FLEXPWM2_EXTA2                  IMXRT_XBAR1(XBAR_OUTPUT, 58)    /* XBAR1_OUT58 -> FLEXPWM2  EXTA2 */
#define IMXRT_XBAR1_OUT_FLEXPWM2_EXTA3                  IMXRT_XBAR1(XBAR_OUTPUT, 59)    /* XBAR1_OUT59 -> FLEXPWM2  EXTA3 */
#define IMXRT_XBAR1_OUT_FLEXPWM2_EXT_SYNC0              IMXRT_XBAR1(XBAR_OUTPUT, 60)    /* XBAR1_OUT60 -> FLEXPWM2  EXT_SYNC0 */
#define IMXRT_XBAR1_OUT_FLEXPWM2_EXT_SYNC1              IMXRT_XBAR1(XBAR_OUTPUT, 61)    /* XBAR1_OUT61 -> FLEXPWM2  EXT_SYNC1 */
#define IMXRT_XBAR1_OUT_FLEXPWM2_EXT_SYNC2              IMXRT_XBAR1(XBAR_OUTPUT, 62)    /* XBAR1_OUT62 -> FLEXPWM2  EXT_SYNC2 */
#define IMXRT_XBAR1_OUT_FLEXPWM2_EXT_SYNC3              IMXRT_XBAR1(XBAR_OUTPUT, 63)    /* XBAR1_OUT63 -> FLEXPWM2  EXT_SYNC3 */
#define IMXRT_XBAR1_OUT_FLEXPWM2_EXT_CLK                IMXRT_XBAR1(XBAR_OUTPUT, 64)    /* XBAR1_OUT64 -> FLEXPWM2  EXT_CLK */
#define IMXRT_XBAR1_OUT_FLEXPWM2_FAULT0                 IMXRT_XBAR1(XBAR_OUTPUT, 65)    /* XBAR1_OUT65 -> FLEXPWM2  FAULT0 */
#define IMXRT_XBAR1_OUT_FLEXPWM2_FAULT1                 IMXRT_XBAR1(XBAR_OUTPUT, 66)    /* XBAR1_OUT66 -> FLEXPWM2  FAULT1 */
#define IMXRT_XBAR1_OUT_FLEXPWM2_EXT_FORCE              IMXRT_XBAR1(XBAR_OUTPUT, 67)    /* XBAR1_OUT67 -> FLEXPWM2  EXT_FORCE */
#define IMXRT_XBAR1_OUT_FLEXPWM3_EXTA0                  IMXRT_XBAR1(XBAR_OUTPUT, 68)    /* XBAR1_OUT68 -> FLEXPWM3  EXTA0 */
#define IMXRT_XBAR1_OUT_FLEXPWM3_EXTA1                  IMXRT_XBAR1(XBAR_OUTPUT, 69)    /* XBAR1_OUT69 -> FLEXPWM3  EXTA1 */
#define IMXRT_XBAR1_OUT_FLEXPWM3_EXTA2                  IMXRT_XBAR1(XBAR_OUTPUT, 70)    /* XBAR1_OUT70 -> FLEXPWM3  EXTA2 */
#define IMXRT_XBAR1_OUT_FLEXPWM3_EXTA3                  IMXRT_XBAR1(XBAR_OUTPUT, 71)    /* XBAR1_OUT71 -> FLEXPWM3  EXTA3 */
#define IMXRT_XBAR1_OUT_FLEXPWM3_EXT_CLK                IMXRT_XBAR1(XBAR_OUTPUT, 72)    /* XBAR1_OUT72 -> FLEXPWM3  EXT_CLK */
#define IMXRT_XBAR1_OUT_FLEXPWM3_EXT_SYNC0              IMXRT_XBAR1(XBAR_OUTPUT, 73)    /* XBAR1_OUT73 -> FLEXPWM3  EXT_SYNC0 */
#define IMXRT_XBAR1_OUT_FLEXPWM3_EXT_SYNC1              IMXRT_XBAR1(XBAR_OUTPUT, 74)    /* XBAR1_OUT74 -> FLEXPWM3  EXT_SYNC1 */
#define IMXRT_XBAR1_OUT_FLEXPWM3_EXT_SYNC2              IMXRT_XBAR1(XBAR_OUTPUT, 75)    /* XBAR1_OUT75 -> FLEXPWM3  EXT_SYNC2 */
#define IMXRT_XBAR1_OUT_FLEXPWM3_EXT_SYNC3              IMXRT_XBAR1(XBAR_OUTPUT, 76)    /* XBAR1_OUT76 -> FLEXPWM3  EXT_SYNC3 */
#define IMXRT_XBAR1_OUT_FLEXPWM3_FAULT0                 IMXRT_XBAR1(XBAR_OUTPUT, 77)    /* XBAR1_OUT77 -> FLEXPWM3  FAULT0 */
#define IMXRT_XBAR1_OUT_FLEXPWM3_FAULT1                 IMXRT_XBAR1(XBAR_OUTPUT, 78)    /* XBAR1_OUT78 -> FLEXPWM3  FAULT1 */
#define IMXRT_XBAR1_OUT_FLEXPWM3_EXT_FORCE              IMXRT_XBAR1(XBAR_OUTPUT, 79)    /* XBAR1_OUT79 -> FLEXPWM3  EXT_FORCE */
#define IMXRT_XBAR1_OUT_FLEXPWM4_EXT_SYNC0              IMXRT_XBAR1(XBAR_OUTPUT, 80)    /* XBAR1_OUT80 -> FLEXPWM4  EXT_SYNC0 */
#define IMXRT_XBAR1_OUT_FLEXPWM4_EXT_SYNC1              IMXRT_XBAR1(XBAR_OUTPUT, 81)    /* XBAR1_OUT81 -> FLEXPWM4  EXT_SYNC1 */
#define IMXRT_XBAR1_OUT_FLEXPWM4_EXT_SYNC2              IMXRT_XBAR1(XBAR_OUTPUT, 82)    /* XBAR1_OUT82 -> FLEXPWM4  EXT_SYNC2 */
#define IMXRT_XBAR1_OUT_FLEXPWM4_EXT_SYNC3              IMXRT_XBAR1(XBAR_OUTPUT, 83)    /* XBAR1_OUT83 -> FLEXPWM4  EXT_SYNC3 */
#define IMXRT_XBAR1_OUT_FLEXPWM4_FAULT0                 IMXRT_XBAR1(XBAR_OUTPUT, 84)    /* XBAR1_OUT84 -> FLEXPWM4  FAULT0 */
#define IMXRT_XBAR1_OUT_FLEXPWM4_FAULT1                 IMXRT_XBAR1(XBAR_OUTPUT, 85)    /* XBAR1_OUT85 -> FLEXPWM4  FAULT1 */
#define IMXRT_XBAR1_OUT_FLEXPWM4_EXT_FORCE              IMXRT_XBAR1(XBAR_OUTPUT, 86)    /* XBAR1_OUT86 -> FLEXPWM4  EXT_FORCE */
#define IMXRT_XBAR1_OUT_EQDC1_PHASE_A_INPUT             IMXRT_XBAR1(XBAR_OUTPUT, 87)    /* XBAR1_OUT87 -> eQDC1  PHASE_A_INPUT */
#define IMXRT_XBAR1_OUT_EQDC1_PHASE_B_INPUT             IMXRT_XBAR1(XBAR_OUTPUT, 88)    /* XBAR1_OUT88 -> eQDC1  PHASE_B_INPUT */
#define IMXRT_XBAR1_OUT_EQDC1_INDEX                     IMXRT_XBAR1(XBAR_OUTPUT, 89)    /* XBAR1_OUT89 -> eQDC1  INDEX */
#define IMXRT_XBAR1_OUT_EQDC1_HOME                      IMXRT_XBAR1(XBAR_OUTPUT, 90)    /* XBAR1_OUT90 -> eQDC1  HOME */
#define IMXRT_XBAR1_OUT_EQDC1_TRIGGER                   IMXRT_XBAR1(XBAR_OUTPUT, 91)    /* XBAR1_OUT91 -> eQDC1  TRIGGER */
#define IMXRT_XBAR1_OUT_EQDC2_PHASE_A_INPUT             IMXRT_XBAR1(XBAR_OUTPUT, 92)    /* XBAR1_OUT92 -> eQDC2  PHASE_A_INPUT */
#define IMXRT_XBAR1_OUT_EQDC2_PHASE_B_INPUT             IMXRT_XBAR1(XBAR_OUTPUT, 93)    /* XBAR1_OUT93 -> eQDC2  PHASE_B_INPUT */
#define IMXRT_XBAR1_OUT_EQDC2_INDEX                     IMXRT_XBAR1(XBAR_OUTPUT, 94)    /* XBAR1_OUT94 -> eQDC2  INDEX */
#define IMXRT_XBAR1_OUT_EQDC2_HOME                      IMXRT_XBAR1(XBAR_OUTPUT, 95)    /* XBAR1_OUT95 -> eQDC2  HOME */
#define IMXRT_XBAR1_OUT_EQDC2_TRIGGER                   IMXRT_XBAR1(XBAR_OUTPUT, 96)    /* XBAR1_OUT96 -> eQDC2  TRIGGER */
#define IMXRT_XBAR1_OUT_EQDC3_PHASE_A_INPUT             IMXRT_XBAR1(XBAR_OUTPUT, 97)    /* XBAR1_OUT97 -> eQDC3  PHASE_A_INPUT */
#define IMXRT_XBAR1_OUT_EQDC3_PHASE_B_INPUT             IMXRT_XBAR1(XBAR_OUTPUT, 98)    /* XBAR1_OUT98 -> eQDC3  PHASE_B_INPUT */
#define IMXRT_XBAR1_OUT_EQDC3_INDEX                     IMXRT_XBAR1(XBAR_OUTPUT, 99)    /* XBAR1_OUT99 -> eQDC3  INDEX */
#define IMXRT_XBAR1_OUT_EQDC3_HOME                      IMXRT_XBAR1(XBAR_OUTPUT, 100)   /* XBAR1_OUT100 -> eQDC3  HOME */
#define IMXRT_XBAR1_OUT_EQDC3_TRIGGER                   IMXRT_XBAR1(XBAR_OUTPUT, 101)   /* XBAR1_OUT101 -> eQDC3  TRIGGER */
#define IMXRT_XBAR1_OUT_EQDC4_PHASE_A_INPUT             IMXRT_XBAR1(XBAR_OUTPUT, 102)   /* XBAR1_OUT102 -> eQDC4  PHASE_A_INPUT */
#define IMXRT_XBAR1_OUT_EQDC4_PHASE_B_INPUT             IMXRT_XBAR1(XBAR_OUTPUT, 103)   /* XBAR1_OUT103 -> eQDC4  PHASE_B_INPUT */
#define IMXRT_XBAR1_OUT_EQDC4_INDEX                     IMXRT_XBAR1(XBAR_OUTPUT, 104)   /* XBAR1_OUT104 -> eQDC4  INDEX */
#define IMXRT_XBAR1_OUT_EQDC4_HOME                      IMXRT_XBAR1(XBAR_OUTPUT, 105)   /* XBAR1_OUT105 -> eQDC4  HOME */
#define IMXRT_XBAR1_OUT_EQDC4_TRIGGER                   IMXRT_XBAR1(XBAR_OUTPUT, 106)   /* XBAR1_OUT106 -> eQDC4  TRIGGER */
#define IMXRT_XBAR1_OUT_TMR1_TMR0_INPUT_QTIMER_CTRL1    IMXRT_XBAR1(XBAR_OUTPUT, 107)   /* XBAR1_OUT107 -> TMR1  TMR0_INPUT QTIMER_CTRL1[QTIMER1_TM R0_INPUT_SEL] = 1 */
#define IMXRT_XBAR1_OUT_TMR1_TMR1_INPUT_QTIMER_CTRL1    IMXRT_XBAR1(XBAR_OUTPUT, 108)   /* XBAR1_OUT108 -> TMR1  TMR1_INPUT QTIMER_CTRL1[QTIMER1_TM R1_INPUT_SEL] = 1 */
#define IMXRT_XBAR1_OUT_TMR1_TMR2_INPUT_QTIMER_CTRL1    IMXRT_XBAR1(XBAR_OUTPUT, 109)   /* XBAR1_OUT109 -> TMR1  TMR2_INPUT QTIMER_CTRL1[QTIMER1_TM R2_INPUT_SEL] = 1 */
#define IMXRT_XBAR1_OUT_TMR1_TMR3_INPUT_QTIMER_CTRL1    IMXRT_XBAR1(XBAR_OUTPUT, 110)   /* XBAR1_OUT110 -> TMR1  TMR3_INPUT QTIMER_CTRL1[QTIMER1_TM R3_INPUT_SEL] = 1 */
#define IMXRT_XBAR1_OUT_TMR2_TMR0_INPUT_QTIMER_CTRL1    IMXRT_XBAR1(XBAR_OUTPUT, 111)   /* XBAR1_OUT111 -> TMR2  TMR0_INPUT QTIMER_CTRL1[QTIMER2_TM R0_INPUT_SEL] = 1 */
#define IMXRT_XBAR1_OUT_TMR2_TMR1_INPUT_QTIMER_CTRL1    IMXRT_XBAR1(XBAR_OUTPUT, 112)   /* XBAR1_OUT112 -> TMR2  TMR1_INPUT QTIMER_CTRL1[QTIMER2_TM R1_INPUT_SEL] = 1 */
#define IMXRT_XBAR1_OUT_TMR2_TMR2_INPUT_QTIMER_CTRL1    IMXRT_XBAR1(XBAR_OUTPUT, 113)   /* XBAR1_OUT113 -> TMR2  TMR2_INPUT QTIMER_CTRL1[QTIMER2_TM R2_INPUT_SEL] = 1 */
#define IMXRT_XBAR1_OUT_TMR2_TMR3_INPUT_QTIMER_CTRL1    IMXRT_XBAR1(XBAR_OUTPUT, 114)   /* XBAR1_OUT114 -> TMR2  TMR3_INPUT QTIMER_CTRL1[QTIMER2_TM R3_INPUT_SEL] = 1 */
#define IMXRT_XBAR1_OUT_TMR3_TMR0_INPUT_QTIMER_CTRL1    IMXRT_XBAR1(XBAR_OUTPUT, 115)   /* XBAR1_OUT115 -> TMR3  TMR0_INPUT QTIMER_CTRL1[QTIMER3_TM R0_INPUT_SEL] = 1 */
#define IMXRT_XBAR1_OUT_TMR3_TMR1_INPUT_QTIMER_CTRL1    IMXRT_XBAR1(XBAR_OUTPUT, 116)   /* XBAR1_OUT116 -> TMR3  TMR1_INPUT QTIMER_CTRL1[QTIMER3_TM R1_INPUT_SEL] = 1 */
#define IMXRT_XBAR1_OUT_TMR3_TMR2_INPUT_QTIMER_CTRL1    IMXRT_XBAR1(XBAR_OUTPUT, 117)   /* XBAR1_OUT117 -> TMR3  TMR2_INPUT QTIMER_CTRL1[QTIMER3_TM R2_INPUT_SEL] = 1 */
#define IMXRT_XBAR1_OUT_TMR3_TMR3_INPUT_QTIMER_CTRL1    IMXRT_XBAR1(XBAR_OUTPUT, 118)   /* XBAR1_OUT118 -> TMR3  TMR3_INPUT QTIMER_CTRL1[QTIMER3_TM R3_INPUT_SEL] = 1 */
#define IMXRT_XBAR1_OUT_TMR4_TMR0_INPUT_QTIMER_CTRL1    IMXRT_XBAR1(XBAR_OUTPUT, 119)   /* XBAR1_OUT119 -> TMR4  TMR0_INPUT QTIMER_CTRL1[QTIMER4_TM R0_INPUT_SEL] = 1 */
#define IMXRT_XBAR1_OUT_TMR4_TMR1_INPUT_QTIMER_CTRL1    IMXRT_XBAR1(XBAR_OUTPUT, 120)   /* XBAR1_OUT120 -> TMR4  TMR1_INPUT QTIMER_CTRL1[QTIMER4_TM R1_INPUT_SEL] = 1 */
#define IMXRT_XBAR1_OUT_TMR4_TMR2_INPUT_QTIMER_CTRL1    IMXRT_XBAR1(XBAR_OUTPUT, 121)   /* XBAR1_OUT121 -> TMR4  TMR2_INPUT QTIMER_CTRL1[QTIMER4_TM R2_INPUT_SEL] = 1 */
#define IMXRT_XBAR1_OUT_TMR4_TMR3_INPUT_QTIMER_CTRL1    IMXRT_XBAR1(XBAR_OUTPUT, 122)   /* XBAR1_OUT122 -> TMR4  TMR3_INPUT QTIMER_CTRL1[QTIMER4_TM R3_INPUT_SEL] = 1 */
#define IMXRT_XBAR1_OUT_TMR5_TMR0_INPUT_QTIMER_CTRL2    IMXRT_XBAR1(XBAR_OUTPUT, 123)   /* XBAR1_OUT123 -> TMR5  TMR0_INPUT QTIMER_CTRL2[QTIMER5_TM R0_INPUT_SEL] = 1 */
#define IMXRT_XBAR1_OUT_TMR5_TMR1_INPUT_QTIMER_CTRL2    IMXRT_XBAR1(XBAR_OUTPUT, 124)   /* XBAR1_OUT124 -> TMR5  TMR1_INPUT QTIMER_CTRL2[QTIMER5_TM R1_INPUT_SEL] = 1 */
#define IMXRT_XBAR1_OUT_TMR5_TMR2_INPUT_QTIMER_CTRL2    IMXRT_XBAR1(XBAR_OUTPUT, 125)   /* XBAR1_OUT125 -> TMR5  TMR2_INPUT QTIMER_CTRL2[QTIMER5_TM R2_INPUT_SEL] = 1 */
#define IMXRT_XBAR1_OUT_TMR5_TMR3_INPUT_QTIMER_CTRL2    IMXRT_XBAR1(XBAR_OUTPUT, 126)   /* XBAR1_OUT126 -> TMR5  TMR3_INPUT QTIMER_CTRL2[QTIMER5_TM R3_INPUT_SEL] = 1 */
#define IMXRT_XBAR1_OUT_TMR6_TMR0_INPUT_QTIMER_CTRL2    IMXRT_XBAR1(XBAR_OUTPUT, 127)   /* XBAR1_OUT127 -> TMR6  TMR0_INPUT QTIMER_CTRL2[QTIMER6_TM R0_INPUT_SEL] = 1 */
#define IMXRT_XBAR1_OUT_TMR6_TMR1_INPUT_QTIMER_CTRL2    IMXRT_XBAR1(XBAR_OUTPUT, 128)   /* XBAR1_OUT128 -> TMR6  TMR1_INPUT QTIMER_CTRL2[QTIMER6_TM R1_INPUT_SEL] = 1 */
#define IMXRT_XBAR1_OUT_TMR6_TMR2_INPUT_QTIMER_CTRL2    IMXRT_XBAR1(XBAR_OUTPUT, 129)   /* XBAR1_OUT129 -> TMR6  TMR2_INPUT QTIMER_CTRL2[QTIMER6_TM R2_INPUT_SEL] = 1 */
#define IMXRT_XBAR1_OUT_TMR6_TMR3_INPUT_QTIMER_CTRL2    IMXRT_XBAR1(XBAR_OUTPUT, 130)   /* XBAR1_OUT130 -> TMR6  TMR3_INPUT QTIMER_CTRL2[QTIMER6_TM R3_INPUT_SEL] = 1 */
#define IMXRT_XBAR1_OUT_TMR7_TMR0_INPUT_QTIMER_CTRL2    IMXRT_XBAR1(XBAR_OUTPUT, 131)   /* XBAR1_OUT131 -> TMR7  TMR0_INPUT QTIMER_CTRL2[QTIMER7_TM R0_INPUT_SEL] = 1 */
#define IMXRT_XBAR1_OUT_TMR7_TMR1_INPUT_QTIMER_CTRL2    IMXRT_XBAR1(XBAR_OUTPUT, 132)   /* XBAR1_OUT132 -> TMR7  TMR1_INPUT QTIMER_CTRL2[QTIMER7_TM R1_INPUT_SEL] = 1 */
#define IMXRT_XBAR1_OUT_TMR7_TMR2_INPUT_QTIMER_CTRL2    IMXRT_XBAR1(XBAR_OUTPUT, 133)   /* XBAR1_OUT133 -> TMR7  TMR2_INPUT QTIMER_CTRL2[QTIMER7_TM R2_INPUT_SEL] = 1 */
#define IMXRT_XBAR1_OUT_TMR7_TMR3_INPUT_QTIMER_CTRL2    IMXRT_XBAR1(XBAR_OUTPUT, 134)   /* XBAR1_OUT134 -> TMR7  TMR3_INPUT QTIMER_CTRL2[QTIMER7_TM R3_INPUT_SEL] = 1 */
#define IMXRT_XBAR1_OUT_TMR8_TMR0_INPUT_QTIMER_CTRL2    IMXRT_XBAR1(XBAR_OUTPUT, 135)   /* XBAR1_OUT135 -> TMR8  TMR0_INPUT QTIMER_CTRL2[QTIMER8_TM R0_INPUT_SEL] = 1 */
#define IMXRT_XBAR1_OUT_TMR8_TMR1_INPUT_QTIMER_CTRL2    IMXRT_XBAR1(XBAR_OUTPUT, 136)   /* XBAR1_OUT136 -> TMR8  TMR1_INPUT QTIMER_CTRL2[QTIMER8_TM R1_INPUT_SEL] = 1 */
#define IMXRT_XBAR1_OUT_TMR8_TMR2_INPUT_QTIMER_CTRL2    IMXRT_XBAR1(XBAR_OUTPUT, 137)   /* XBAR1_OUT137 -> TMR8  TMR2_INPUT QTIMER_CTRL2[QTIMER8_TM R2_INPUT_SEL] = 1 */
#define IMXRT_XBAR1_OUT_TMR8_TMR3_INPUT_QTIMER_CTRL2    IMXRT_XBAR1(XBAR_OUTPUT, 138)   /* XBAR1_OUT138 -> TMR8  TMR3_INPUT QTIMER_CTRL2[QTIMER8_TM R3_INPUT_SEL] = 1 */
#define IMXRT_XBAR1_OUT_EWM_EWM_IN                      IMXRT_XBAR1(XBAR_OUTPUT, 139)   /* XBAR1_OUT139 -> EWM  EWM_IN */
#define IMXRT_XBAR1_OUT_ADC1_HW_TRG0                    IMXRT_XBAR1(XBAR_OUTPUT, 140)   /* XBAR1_OUT140 -> ADC1  HW_TRG0 */
#define IMXRT_XBAR1_OUT_ADC1_HW_TRG1                    IMXRT_XBAR1(XBAR_OUTPUT, 141)   /* XBAR1_OUT141 -> ADC1  HW_TRG1 */
#define IMXRT_XBAR1_OUT_ADC1_HW_TRG2                    IMXRT_XBAR1(XBAR_OUTPUT, 142)   /* XBAR1_OUT142 -> ADC1  HW_TRG2 */
#define IMXRT_XBAR1_OUT_ADC1_HW_TRG3                    IMXRT_XBAR1(XBAR_OUTPUT, 143)   /* XBAR1_OUT143 -> ADC1  HW_TRG3 */
#define IMXRT_XBAR1_OUT_ADC1_HW_TRG4                    IMXRT_XBAR1(XBAR_OUTPUT, 144)   /* XBAR1_OUT144 -> ADC1  HW_TRG4 */
#define IMXRT_XBAR1_OUT_ADC1_HW_TRG5                    IMXRT_XBAR1(XBAR_OUTPUT, 145)   /* XBAR1_OUT145 -> ADC1  HW_TRG5 */
#define IMXRT_XBAR1_OUT_ADC1_HW_TRG6                    IMXRT_XBAR1(XBAR_OUTPUT, 146)   /* XBAR1_OUT146 -> ADC1  HW_TRG6 */
#define IMXRT_XBAR1_OUT_ADC1_HW_TRG7                    IMXRT_XBAR1(XBAR_OUTPUT, 147)   /* XBAR1_OUT147 -> ADC1  HW_TRG7 */
#define IMXRT_XBAR1_OUT_SINC1_EXT_TRIGGER0              IMXRT_XBAR1(XBAR_OUTPUT, 148)   /* XBAR1_OUT148 -> SINC1  EXT_TRIGGER0 */
#define IMXRT_XBAR1_OUT_SINC1_EXT_TRIGGER1              IMXRT_XBAR1(XBAR_OUTPUT, 149)   /* XBAR1_OUT149 -> SINC1  EXT_TRIGGER1 */
#define IMXRT_XBAR1_OUT_SINC1_EXT_TRIGGER2              IMXRT_XBAR1(XBAR_OUTPUT, 150)   /* XBAR1_OUT150 -> SINC1  EXT_TRIGGER2 */
#define IMXRT_XBAR1_OUT_SINC1_EXT_TRIGGER3              IMXRT_XBAR1(XBAR_OUTPUT, 151)   /* XBAR1_OUT151 -> SINC1  EXT_TRIGGER3 */
#define IMXRT_XBAR1_OUT_FLEXIO1_FLEXIO_TRIGGER_IN0      IMXRT_XBAR1(XBAR_OUTPUT, 152)   /* XBAR1_OUT152 -> FLEXIO1  FLEXIO_TRIGGER_IN0 */
#define IMXRT_XBAR1_OUT_FLEXIO1_FLEXIO_TRIGGER_IN1      IMXRT_XBAR1(XBAR_OUTPUT, 153)   /* XBAR1_OUT153 -> FLEXIO1  FLEXIO_TRIGGER_IN1 */
#define IMXRT_XBAR1_OUT_FLEXIO2_FLEXIO_TRIGGER_IN0      IMXRT_XBAR1(XBAR_OUTPUT, 154)   /* XBAR1_OUT154 -> FLEXIO2  FLEXIO_TRIGGER_IN0 */
#define IMXRT_XBAR1_OUT_FLEXIO2_FLEXIO_TRIGGER_IN1      IMXRT_XBAR1(XBAR_OUTPUT, 155)   /* XBAR1_OUT155 -> FLEXIO2  FLEXIO_TRIGGER_IN1 */
#define IMXRT_XBAR1_OUT_LPI2C1_LPI2C_TRG_INPUT          IMXRT_XBAR1(XBAR_OUTPUT, 156)   /* XBAR1_OUT156 -> LPI2C1  LPI2C_TRG_INPUT */
#define IMXRT_XBAR1_OUT_LPI2C2_LPI2C_TRG_INPUT          IMXRT_XBAR1(XBAR_OUTPUT, 157)   /* XBAR1_OUT157 -> LPI2C2  LPI2C_TRG_INPUT */
#define IMXRT_XBAR1_OUT_LPI2C3_LPI2C_TRG_INPUT          IMXRT_XBAR1(XBAR_OUTPUT, 158)   /* XBAR1_OUT158 -> LPI2C3  LPI2C_TRG_INPUT */
#define IMXRT_XBAR1_OUT_LPI2C4_LPI2C_TRG_INPUT          IMXRT_XBAR1(XBAR_OUTPUT, 159)   /* XBAR1_OUT159 -> LPI2C4  LPI2C_TRG_INPUT */
#define IMXRT_XBAR1_OUT_LPI2C5_LPI2C_TRG_INPUT          IMXRT_XBAR1(XBAR_OUTPUT, 160)   /* XBAR1_OUT160 -> LPI2C5  LPI2C_TRG_INPUT */
#define IMXRT_XBAR1_OUT_LPI2C6_LPI2C_TRG_INPUT          IMXRT_XBAR1(XBAR_OUTPUT, 161)   /* XBAR1_OUT161 -> LPI2C6  LPI2C_TRG_INPUT */
#define IMXRT_XBAR1_OUT_LPSPI1_LPSPI_TRG_INPUT          IMXRT_XBAR1(XBAR_OUTPUT, 162)   /* XBAR1_OUT162 -> LPSPI1  LPSPI_TRG_INPUT */
#define IMXRT_XBAR1_OUT_LPSPI2_LPSPI_TRG_INPUT          IMXRT_XBAR1(XBAR_OUTPUT, 163)   /* XBAR1_OUT163 -> LPSPI2  LPSPI_TRG_INPUT */
#define IMXRT_XBAR1_OUT_LPSPI3_LPSPI_TRG_INPUT          IMXRT_XBAR1(XBAR_OUTPUT, 164)   /* XBAR1_OUT164 -> LPSPI3  LPSPI_TRG_INPUT */
#define IMXRT_XBAR1_OUT_LPSPI4_LPSPI_TRG_INPUT          IMXRT_XBAR1(XBAR_OUTPUT, 165)   /* XBAR1_OUT165 -> LPSPI4  LPSPI_TRG_INPUT */
#define IMXRT_XBAR1_OUT_LPSPI5_LPSPI_TRG_INPUT          IMXRT_XBAR1(XBAR_OUTPUT, 166)   /* XBAR1_OUT166 -> LPSPI5  LPSPI_TRG_INPUT */
#define IMXRT_XBAR1_OUT_LPSPI6_LPSPI_TRG_INPUT          IMXRT_XBAR1(XBAR_OUTPUT, 167)   /* XBAR1_OUT167 -> LPSPI6  LPSPI_TRG_INPUT */
#define IMXRT_XBAR1_OUT_LPUART1_LPUART_TRG_INPUT        IMXRT_XBAR1(XBAR_OUTPUT, 168)   /* XBAR1_OUT168 -> LPUART1  LPUART_TRG_INPUT */
#define IMXRT_XBAR1_OUT_LPUART2_LPUART_TRG_INPUT        IMXRT_XBAR1(XBAR_OUTPUT, 169)   /* XBAR1_OUT169 -> LPUART2  LPUART_TRG_INPUT */
#define IMXRT_XBAR1_OUT_LPUART3_LPUART_TRG_INPUT        IMXRT_XBAR1(XBAR_OUTPUT, 170)   /* XBAR1_OUT170 -> LPUART3  LPUART_TRG_INPUT */
#define IMXRT_XBAR1_OUT_LPUART4_LPUART_TRG_INPUT        IMXRT_XBAR1(XBAR_OUTPUT, 171)   /* XBAR1_OUT171 -> LPUART4  LPUART_TRG_INPUT */
#define IMXRT_XBAR1_OUT_LPUART5_LPUART_TRG_INPUT        IMXRT_XBAR1(XBAR_OUTPUT, 172)   /* XBAR1_OUT172 -> LPUART5  LPUART_TRG_INPUT */
#define IMXRT_XBAR1_OUT_LPUART6_LPUART_TRG_INPUT        IMXRT_XBAR1(XBAR_OUTPUT, 173)   /* XBAR1_OUT173 -> LPUART6  LPUART_TRG_INPUT */
#define IMXRT_XBAR1_OUT_LPUART7_LPUART_TRG_INPUT        IMXRT_XBAR1(XBAR_OUTPUT, 174)   /* XBAR1_OUT174 -> LPUART7  LPUART_TRG_INPUT */
#define IMXRT_XBAR1_OUT_LPUART8_LPUART_TRG_INPUT        IMXRT_XBAR1(XBAR_OUTPUT, 175)   /* XBAR1_OUT175 -> LPUART8  LPUART_TRG_INPUT */
#define IMXRT_XBAR1_OUT_LPUART9_LPUART_TRG_INPUT        IMXRT_XBAR1(XBAR_OUTPUT, 176)   /* XBAR1_OUT176 -> LPUART9  LPUART_TRG_INPUT */
#define IMXRT_XBAR1_OUT_LPUART10_LPUART_TRG_INPUT       IMXRT_XBAR1(XBAR_OUTPUT, 177)   /* XBAR1_OUT177 -> LPUART10  LPUART_TRG_INPUT */
#define IMXRT_XBAR1_OUT_LPUART11_LPUART_TRG_INPUT       IMXRT_XBAR1(XBAR_OUTPUT, 178)   /* XBAR1_OUT178 -> LPUART11  LPUART_TRG_INPUT */
#define IMXRT_XBAR1_OUT_LPUART12_LPUART_TRG_INPUT       IMXRT_XBAR1(XBAR_OUTPUT, 179)   /* XBAR1_OUT179 -> LPUART12  LPUART_TRG_INPUT */
#define IMXRT_XBAR1_OUT_LPIT1_LPIT_EXT_TRIG_IN0         IMXRT_XBAR1(XBAR_OUTPUT, 180)   /* XBAR1_OUT180 -> LPIT1  LPIT_EXT_TRIG_IN0 */
#define IMXRT_XBAR1_OUT_LPIT1_LPIT_EXT_TRIG_IN1         IMXRT_XBAR1(XBAR_OUTPUT, 181)   /* XBAR1_OUT181 -> LPIT1  LPIT_EXT_TRIG_IN1 */
#define IMXRT_XBAR1_OUT_LPIT1_LPIT_EXT_TRIG_IN2         IMXRT_XBAR1(XBAR_OUTPUT, 182)   /* XBAR1_OUT182 -> LPIT1  LPIT_EXT_TRIG_IN2 */
#define IMXRT_XBAR1_OUT_LPIT1_LPIT_EXT_TRIG_IN3         IMXRT_XBAR1(XBAR_OUTPUT, 183)   /* XBAR1_OUT183 -> LPIT1  LPIT_EXT_TRIG_IN3 */
#define IMXRT_XBAR1_OUT_TPM1_TPM_TRIGGER_IN0            IMXRT_XBAR1(XBAR_OUTPUT, 184)   /* XBAR1_OUT184 -> TPM1  TPM_TRIGGER_IN0 */
#define IMXRT_XBAR1_OUT_TPM1_TPM_TRIGGER_IN1            IMXRT_XBAR1(XBAR_OUTPUT, 185)   /* XBAR1_OUT185 -> TPM1  TPM_TRIGGER_IN1 */
#define IMXRT_XBAR1_OUT_TPM2_TPM_TRIGGER_IN1            IMXRT_XBAR1(XBAR_OUTPUT, 186)   /* XBAR1_OUT186 -> TPM2  TPM_TRIGGER_IN1 */
#define IMXRT_XBAR1_OUT_TPM3_TPM_TRIGGER_IN1            IMXRT_XBAR1(XBAR_OUTPUT, 187)   /* XBAR1_OUT187 -> TPM3  TPM_TRIGGER_IN1 */
#define IMXRT_XBAR1_OUT_TPM1_TPM_TRIGGER_IN2            IMXRT_XBAR1(XBAR_OUTPUT, 188)   /* XBAR1_OUT188 -> TPM1  TPM_TRIGGER_IN2 */
#define IMXRT_XBAR1_OUT_TPM1_TPM_TRIGGER_IN3            IMXRT_XBAR1(XBAR_OUTPUT, 189)   /* XBAR1_OUT189 -> TPM1  TPM_TRIGGER_IN3 */
#define IMXRT_XBAR1_OUT_TPM2_TPM_TRIGGER_IN3            IMXRT_XBAR1(XBAR_OUTPUT, 190)   /* XBAR1_OUT190 -> TPM2  TPM_TRIGGER_IN3 */
#define IMXRT_XBAR1_OUT_TPM3_TPM_TRIGGER_IN3            IMXRT_XBAR1(XBAR_OUTPUT, 191)   /* XBAR1_OUT191 -> TPM3  TPM_TRIGGER_IN3 */
#define IMXRT_XBAR1_OUT_TPM4_TPM_TRIGGER_IN0            IMXRT_XBAR1(XBAR_OUTPUT, 192)   /* XBAR1_OUT192 -> TPM4  TPM_TRIGGER_IN0 */
#define IMXRT_XBAR1_OUT_TPM4_TPM_TRIGGER_IN1            IMXRT_XBAR1(XBAR_OUTPUT, 193)   /* XBAR1_OUT193 -> TPM4  TPM_TRIGGER_IN1 */
#define IMXRT_XBAR1_OUT_TPM5_TPM_TRIGGER_IN1            IMXRT_XBAR1(XBAR_OUTPUT, 194)   /* XBAR1_OUT194 -> TPM5  TPM_TRIGGER_IN1 */
#define IMXRT_XBAR1_OUT_TPM6_TPM_TRIGGER_IN1            IMXRT_XBAR1(XBAR_OUTPUT, 195)   /* XBAR1_OUT195 -> TPM6  TPM_TRIGGER_IN1 */
#define IMXRT_XBAR1_OUT_TPM4_TPM_TRIGGER_IN2            IMXRT_XBAR1(XBAR_OUTPUT, 196)   /* XBAR1_OUT196 -> TPM4  TPM_TRIGGER_IN2 */
#define IMXRT_XBAR1_OUT_TPM4_TPM_TRIGGER_IN3            IMXRT_XBAR1(XBAR_OUTPUT, 197)   /* XBAR1_OUT197 -> TPM4  TPM_TRIGGER_IN3 */
#define IMXRT_XBAR1_OUT_TPM5_TPM_TRIGGER_IN3            IMXRT_XBAR1(XBAR_OUTPUT, 198)   /* XBAR1_OUT198 -> TPM5  TPM_TRIGGER_IN3 */
#define IMXRT_XBAR1_OUT_TPM6_TPM_TRIGGER_IN3            IMXRT_XBAR1(XBAR_OUTPUT, 199)   /* XBAR1_OUT199 -> TPM6  TPM_TRIGGER_IN3 */
#define IMXRT_XBAR1_OUT_NETC_TMR_1588_TRIG1             IMXRT_XBAR1(XBAR_OUTPUT, 200)   /* XBAR1_OUT200 -> NETC  TMR_1588_TRIG1 */
#define IMXRT_XBAR1_OUT_NETC_TMR_1588_TRIG2             IMXRT_XBAR1(XBAR_OUTPUT, 201)   /* XBAR1_OUT201 -> NETC  TMR_1588_TRIG2 */
#define IMXRT_XBAR1_OUT_RESERVED                        IMXRT_XBAR1(XBAR_OUTPUT, 202)   /* XBAR1_OUT202 -> -  Reserved */
#define IMXRT_XBAR1_OUT_RESERVED_2                      IMXRT_XBAR1(XBAR_OUTPUT, 203)   /* XBAR1_OUT203 -> -  Reserved */
#define IMXRT_XBAR1_OUT_RESERVED_3                      IMXRT_XBAR1(XBAR_OUTPUT, 204)   /* XBAR1_OUT204 -> -  Reserved */
#define IMXRT_XBAR1_OUT_RESERVED_4                      IMXRT_XBAR1(XBAR_OUTPUT, 205)   /* XBAR1_OUT205 -> -  Reserved */
#define IMXRT_XBAR1_OUT_EQDC1_ICAP1                     IMXRT_XBAR1(XBAR_OUTPUT, 206)   /* XBAR1_OUT206 -> eQDC1  ICAP1 */
#define IMXRT_XBAR1_OUT_EQDC1_ICAP2                     IMXRT_XBAR1(XBAR_OUTPUT, 207)   /* XBAR1_OUT207 -> eQDC1  ICAP2 */
#define IMXRT_XBAR1_OUT_EQDC1_ICAP3                     IMXRT_XBAR1(XBAR_OUTPUT, 208)   /* XBAR1_OUT208 -> eQDC1  ICAP3 */
#define IMXRT_XBAR1_OUT_EQDC2_ICAP1                     IMXRT_XBAR1(XBAR_OUTPUT, 209)   /* XBAR1_OUT209 -> eQDC2  ICAP1 */
#define IMXRT_XBAR1_OUT_EQDC2_ICAP2                     IMXRT_XBAR1(XBAR_OUTPUT, 210)   /* XBAR1_OUT210 -> eQDC2  ICAP2 */
#define IMXRT_XBAR1_OUT_EQDC2_ICAP3                     IMXRT_XBAR1(XBAR_OUTPUT, 211)   /* XBAR1_OUT211 -> eQDC2  ICAP3 */
#define IMXRT_XBAR1_OUT_EQDC3_ICAP1                     IMXRT_XBAR1(XBAR_OUTPUT, 212)   /* XBAR1_OUT212 -> eQDC3  ICAP1 */
#define IMXRT_XBAR1_OUT_EQDC3_ICAP2                     IMXRT_XBAR1(XBAR_OUTPUT, 213)   /* XBAR1_OUT213 -> eQDC3  ICAP2 */
#define IMXRT_XBAR1_OUT_EQDC3_ICAP3                     IMXRT_XBAR1(XBAR_OUTPUT, 214)   /* XBAR1_OUT214 -> eQDC3  ICAP3 */
#define IMXRT_XBAR1_OUT_EQDC4_ICAP1                     IMXRT_XBAR1(XBAR_OUTPUT, 215)   /* XBAR1_OUT215 -> eQDC4  ICAP1 */
#define IMXRT_XBAR1_OUT_EQDC4_ICAP2                     IMXRT_XBAR1(XBAR_OUTPUT, 216)   /* XBAR1_OUT216 -> eQDC4  ICAP2 */
#define IMXRT_XBAR1_OUT_EQDC4_ICAP3                     IMXRT_XBAR1(XBAR_OUTPUT, 217)   /* XBAR1_OUT217 -> eQDC4  ICAP3 */
#define IMXRT_XBAR1_OUT_ECAT_LATCH_IN0                  IMXRT_XBAR1(XBAR_OUTPUT, 218)   /* XBAR1_OUT218 -> ECAT  LATCH_IN0 */
#define IMXRT_XBAR1_OUT_ECAT_LATCH_IN1                  IMXRT_XBAR1(XBAR_OUTPUT, 219)   /* XBAR1_OUT219 -> ECAT  LATCH_IN1 */
#define IMXRT_XBAR1_OUT_SAFETY_CLK_DUT_CLK_MON          IMXRT_XBAR1(XBAR_OUTPUT, 220)   /* XBAR1_OUT220 -> SAFETY_CLK DUT_CLK  - _MON */

#define IMXRT_XBAR2_OUT_AOI1_AOI_IN0                    IMXRT_XBAR2(XBAR_OUTPUT, 0)     /* XBAR2_OUT0 -> AOI1  AOI_IN0 */
#define IMXRT_XBAR2_OUT_AOI1_AOI_IN1                    IMXRT_XBAR2(XBAR_OUTPUT, 1)     /* XBAR2_OUT1 -> AOI1  AOI_IN1 */
#define IMXRT_XBAR2_OUT_AOI1_AOI_IN2                    IMXRT_XBAR2(XBAR_OUTPUT, 2)     /* XBAR2_OUT2 -> AOI1  AOI_IN2 */
#define IMXRT_XBAR2_OUT_AOI1_AOI_IN3                    IMXRT_XBAR2(XBAR_OUTPUT, 3)     /* XBAR2_OUT3 -> AOI1  AOI_IN3 */
#define IMXRT_XBAR2_OUT_AOI1_AOI_IN4                    IMXRT_XBAR2(XBAR_OUTPUT, 4)     /* XBAR2_OUT4 -> AOI1  AOI_IN4 */
#define IMXRT_XBAR2_OUT_AOI1_AOI_IN5                    IMXRT_XBAR2(XBAR_OUTPUT, 5)     /* XBAR2_OUT5 -> AOI1  AOI_IN5 */
#define IMXRT_XBAR2_OUT_AOI1_AOI_IN6                    IMXRT_XBAR2(XBAR_OUTPUT, 6)     /* XBAR2_OUT6 -> AOI1  AOI_IN6 */
#define IMXRT_XBAR2_OUT_AOI1_AOI_IN7                    IMXRT_XBAR2(XBAR_OUTPUT, 7)     /* XBAR2_OUT7 -> AOI1  AOI_IN7 */
#define IMXRT_XBAR2_OUT_AOI1_AOI_IN8                    IMXRT_XBAR2(XBAR_OUTPUT, 8)     /* XBAR2_OUT8 -> AOI1  AOI_IN8 */
#define IMXRT_XBAR2_OUT_AOI1_AOI_IN9                    IMXRT_XBAR2(XBAR_OUTPUT, 9)     /* XBAR2_OUT9 -> AOI1  AOI_IN9 */
#define IMXRT_XBAR2_OUT_AOI1_AOI_IN10                   IMXRT_XBAR2(XBAR_OUTPUT, 10)    /* XBAR2_OUT10 -> AOI1  AOI_IN10 */
#define IMXRT_XBAR2_OUT_AOI1_AOI_IN11                   IMXRT_XBAR2(XBAR_OUTPUT, 11)    /* XBAR2_OUT11 -> AOI1  AOI_IN11 */
#define IMXRT_XBAR2_OUT_AOI1_AOI_IN12                   IMXRT_XBAR2(XBAR_OUTPUT, 12)    /* XBAR2_OUT12 -> AOI1  AOI_IN12 */
#define IMXRT_XBAR2_OUT_AOI1_AOI_IN13                   IMXRT_XBAR2(XBAR_OUTPUT, 13)    /* XBAR2_OUT13 -> AOI1  AOI_IN13 */
#define IMXRT_XBAR2_OUT_AOI1_AOI_IN14                   IMXRT_XBAR2(XBAR_OUTPUT, 14)    /* XBAR2_OUT14 -> AOI1  AOI_IN14 */
#define IMXRT_XBAR2_OUT_AOI1_AOI_IN15                   IMXRT_XBAR2(XBAR_OUTPUT, 15)    /* XBAR2_OUT15 -> AOI1  AOI_IN15 */
#define IMXRT_XBAR2_OUT_AOI3_AOI_IN0                    IMXRT_XBAR2(XBAR_OUTPUT, 16)    /* XBAR2_OUT16 -> AOI3  AOI_IN0 */
#define IMXRT_XBAR2_OUT_AOI3_AOI_IN1                    IMXRT_XBAR2(XBAR_OUTPUT, 17)    /* XBAR2_OUT17 -> AOI3  AOI_IN1 */
#define IMXRT_XBAR2_OUT_AOI3_AOI_IN2                    IMXRT_XBAR2(XBAR_OUTPUT, 18)    /* XBAR2_OUT18 -> AOI3  AOI_IN2 */
#define IMXRT_XBAR2_OUT_AOI3_AOI_IN3                    IMXRT_XBAR2(XBAR_OUTPUT, 19)    /* XBAR2_OUT19 -> AOI3  AOI_IN3 */
#define IMXRT_XBAR2_OUT_AOI3_AOI_IN4                    IMXRT_XBAR2(XBAR_OUTPUT, 20)    /* XBAR2_OUT20 -> AOI3  AOI_IN4 */
#define IMXRT_XBAR2_OUT_AOI3_AOI_IN5                    IMXRT_XBAR2(XBAR_OUTPUT, 21)    /* XBAR2_OUT21 -> AOI3  AOI_IN5 */
#define IMXRT_XBAR2_OUT_AOI3_AOI_IN6                    IMXRT_XBAR2(XBAR_OUTPUT, 22)    /* XBAR2_OUT22 -> AOI3  AOI_IN6 */
#define IMXRT_XBAR2_OUT_AOI3_AOI_IN7                    IMXRT_XBAR2(XBAR_OUTPUT, 23)    /* XBAR2_OUT23 -> AOI3  AOI_IN7 */
#define IMXRT_XBAR2_OUT_AOI3_AOI_IN8                    IMXRT_XBAR2(XBAR_OUTPUT, 24)    /* XBAR2_OUT24 -> AOI3  AOI_IN8 */
#define IMXRT_XBAR2_OUT_AOI3_AOI_IN9                    IMXRT_XBAR2(XBAR_OUTPUT, 25)    /* XBAR2_OUT25 -> AOI3  AOI_IN9 */
#define IMXRT_XBAR2_OUT_AOI3_AOI_IN10                   IMXRT_XBAR2(XBAR_OUTPUT, 26)    /* XBAR2_OUT26 -> AOI3  AOI_IN10 */
#define IMXRT_XBAR2_OUT_AOI3_AOI_IN11                   IMXRT_XBAR2(XBAR_OUTPUT, 27)    /* XBAR2_OUT27 -> AOI3  AOI_IN11 */
#define IMXRT_XBAR2_OUT_AOI3_AOI_IN12                   IMXRT_XBAR2(XBAR_OUTPUT, 28)    /* XBAR2_OUT28 -> AOI3  AOI_IN12 */
#define IMXRT_XBAR2_OUT_AOI3_AOI_IN13                   IMXRT_XBAR2(XBAR_OUTPUT, 29)    /* XBAR2_OUT29 -> AOI3  AOI_IN13 */
#define IMXRT_XBAR2_OUT_AOI3_AOI_IN14                   IMXRT_XBAR2(XBAR_OUTPUT, 30)    /* XBAR2_OUT30 -> AOI3  AOI_IN14 */
#define IMXRT_XBAR2_OUT_AOI3_AOI_IN15                   IMXRT_XBAR2(XBAR_OUTPUT, 31)    /* XBAR2_OUT31 -> AOI3  AOI_IN15 */

#define IMXRT_XBAR3_OUT_AOI2_AOI_IN0                    IMXRT_XBAR3(XBAR_OUTPUT, 0)     /* XBAR3_OUT0 -> AOI2  AOI_IN0 */
#define IMXRT_XBAR3_OUT_AOI2_AOI_IN1                    IMXRT_XBAR3(XBAR_OUTPUT, 1)     /* XBAR3_OUT1 -> AOI2  AOI_IN1 */
#define IMXRT_XBAR3_OUT_AOI2_AOI_IN2                    IMXRT_XBAR3(XBAR_OUTPUT, 2)     /* XBAR3_OUT2 -> AOI2  AOI_IN2 */
#define IMXRT_XBAR3_OUT_AOI2_AOI_IN3                    IMXRT_XBAR3(XBAR_OUTPUT, 3)     /* XBAR3_OUT3 -> AOI2  AOI_IN3 */
#define IMXRT_XBAR3_OUT_AOI2_AOI_IN4                    IMXRT_XBAR3(XBAR_OUTPUT, 4)     /* XBAR3_OUT4 -> AOI2  AOI_IN4 */
#define IMXRT_XBAR3_OUT_AOI2_AOI_IN5                    IMXRT_XBAR3(XBAR_OUTPUT, 5)     /* XBAR3_OUT5 -> AOI2  AOI_IN5 */
#define IMXRT_XBAR3_OUT_AOI2_AOI_IN6                    IMXRT_XBAR3(XBAR_OUTPUT, 6)     /* XBAR3_OUT6 -> AOI2  AOI_IN6 */
#define IMXRT_XBAR3_OUT_AOI2_AOI_IN7                    IMXRT_XBAR3(XBAR_OUTPUT, 7)     /* XBAR3_OUT7 -> AOI2  AOI_IN7 */
#define IMXRT_XBAR3_OUT_AOI2_AOI_IN8                    IMXRT_XBAR3(XBAR_OUTPUT, 8)     /* XBAR3_OUT8 -> AOI2  AOI_IN8 */
#define IMXRT_XBAR3_OUT_AOI2_AOI_IN9                    IMXRT_XBAR3(XBAR_OUTPUT, 9)     /* XBAR3_OUT9 -> AOI2  AOI_IN9 */
#define IMXRT_XBAR3_OUT_AOI2_AOI_IN10                   IMXRT_XBAR3(XBAR_OUTPUT, 10)    /* XBAR3_OUT10 -> AOI2  AOI_IN10 */
#define IMXRT_XBAR3_OUT_AOI2_AOI_IN11                   IMXRT_XBAR3(XBAR_OUTPUT, 11)    /* XBAR3_OUT11 -> AOI2  AOI_IN11 */
#define IMXRT_XBAR3_OUT_AOI2_AOI_IN12                   IMXRT_XBAR3(XBAR_OUTPUT, 12)    /* XBAR3_OUT12 -> AOI2  AOI_IN12 */
#define IMXRT_XBAR3_OUT_AOI2_AOI_IN13                   IMXRT_XBAR3(XBAR_OUTPUT, 13)    /* XBAR3_OUT13 -> AOI2  AOI_IN13 */
#define IMXRT_XBAR3_OUT_AOI2_AOI_IN14                   IMXRT_XBAR3(XBAR_OUTPUT, 14)    /* XBAR3_OUT14 -> AOI2  AOI_IN14 */
#define IMXRT_XBAR3_OUT_AOI2_AOI_IN15                   IMXRT_XBAR3(XBAR_OUTPUT, 15)    /* XBAR3_OUT15 -> AOI2  AOI_IN15 */
#define IMXRT_XBAR3_OUT_AOI4_AOI_IN0                    IMXRT_XBAR3(XBAR_OUTPUT, 16)    /* XBAR3_OUT16 -> AOI4  AOI_IN0 */
#define IMXRT_XBAR3_OUT_AOI4_AOI_IN1                    IMXRT_XBAR3(XBAR_OUTPUT, 17)    /* XBAR3_OUT17 -> AOI4  AOI_IN1 */
#define IMXRT_XBAR3_OUT_AOI4_AOI_IN2                    IMXRT_XBAR3(XBAR_OUTPUT, 18)    /* XBAR3_OUT18 -> AOI4  AOI_IN2 */
#define IMXRT_XBAR3_OUT_AOI4_AOI_IN3                    IMXRT_XBAR3(XBAR_OUTPUT, 19)    /* XBAR3_OUT19 -> AOI4  AOI_IN3 */
#define IMXRT_XBAR3_OUT_AOI4_AOI_IN4                    IMXRT_XBAR3(XBAR_OUTPUT, 20)    /* XBAR3_OUT20 -> AOI4  AOI_IN4 */
#define IMXRT_XBAR3_OUT_AOI4_AOI_IN5                    IMXRT_XBAR3(XBAR_OUTPUT, 21)    /* XBAR3_OUT21 -> AOI4  AOI_IN5 */
#define IMXRT_XBAR3_OUT_AOI4_AOI_IN6                    IMXRT_XBAR3(XBAR_OUTPUT, 22)    /* XBAR3_OUT22 -> AOI4  AOI_IN6 */
#define IMXRT_XBAR3_OUT_AOI4_AOI_IN7                    IMXRT_XBAR3(XBAR_OUTPUT, 23)    /* XBAR3_OUT23 -> AOI4  AOI_IN7 */
#define IMXRT_XBAR3_OUT_AOI4_AOI_IN8                    IMXRT_XBAR3(XBAR_OUTPUT, 24)    /* XBAR3_OUT24 -> AOI4  AOI_IN8 */
#define IMXRT_XBAR3_OUT_AOI4_AOI_IN9                    IMXRT_XBAR3(XBAR_OUTPUT, 25)    /* XBAR3_OUT25 -> AOI4  AOI_IN9 */
#define IMXRT_XBAR3_OUT_AOI4_AOI_IN10                   IMXRT_XBAR3(XBAR_OUTPUT, 26)    /* XBAR3_OUT26 -> AOI4  AOI_IN10 */
#define IMXRT_XBAR3_OUT_AOI4_AOI_IN11                   IMXRT_XBAR3(XBAR_OUTPUT, 27)    /* XBAR3_OUT27 -> AOI4  AOI_IN11 */
#define IMXRT_XBAR3_OUT_AOI4_AOI_IN12                   IMXRT_XBAR3(XBAR_OUTPUT, 28)    /* XBAR3_OUT28 -> AOI4  AOI_IN12 */
#define IMXRT_XBAR3_OUT_AOI4_AOI_IN13                   IMXRT_XBAR3(XBAR_OUTPUT, 29)    /* XBAR3_OUT29 -> AOI4  AOI_IN13 */
#define IMXRT_XBAR3_OUT_AOI4_AOI_IN14                   IMXRT_XBAR3(XBAR_OUTPUT, 30)    /* XBAR3_OUT30 -> AOI4  AOI_IN14 */
#define IMXRT_XBAR3_OUT_AOI4_AOI_IN15                   IMXRT_XBAR3(XBAR_OUTPUT, 31)    /* XBAR3_OUT31 -> AOI4  AOI_IN15 */

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_XBAR_H */
