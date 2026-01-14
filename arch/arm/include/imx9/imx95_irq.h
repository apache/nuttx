/****************************************************************************
 * arch/arm/include/imx9/imx95_irq.h
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

#ifndef __ARCH_ARM_INCLUDE_IMX9_IMX95_IRQ_H
#define __ARCH_ARM_INCLUDE_IMX9_IMX95_IRQ_H

#define IMX9_IRQ_RESERVED16      (IMX9_IRQ_EXTINT + 0)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED17      (IMX9_IRQ_EXTINT + 1)   /* DAP interrupt */
#define IMX9_IRQ_RESERVED18      (IMX9_IRQ_EXTINT + 2)   /* CTI trigger outputs from CM7 platform */
#define IMX9_IRQ_RESERVED19      (IMX9_IRQ_EXTINT + 3)   /* CTI trigger outputs from CM33 platform */
#define IMX9_IRQ_RESERVED20      (IMX9_IRQ_EXTINT + 4)   /* CTI trigger outputs from CA55 platform */
#define IMX9_IRQ_RESERVED21      (IMX9_IRQ_EXTINT + 5)   /* Performance Unit Interrupts from CA55 platform */
#define IMX9_IRQ_RESERVED22      (IMX9_IRQ_EXTINT + 6)   /* ECC error from CA55 platform cache */
#define IMX9_IRQ_RESERVED23      (IMX9_IRQ_EXTINT + 7)   /* 1-bit or 2-bit ECC or Parity error from CA55 platform cache */
#define IMX9_IRQ_CAN1            (IMX9_IRQ_EXTINT + 8)   /* CAN1 interrupt */
#define IMX9_IRQ_CAN1_ERROR      (IMX9_IRQ_EXTINT + 9)   /* CAN1 error interrupt */
#define IMX9_IRQ_GPIO1_0         (IMX9_IRQ_EXTINT + 10)  /* General Purpose Input/Output 1 interrupt 0 */
#define IMX9_IRQ_GPIO1_1         (IMX9_IRQ_EXTINT + 11)  /* General Purpose Input/Output 1 interrupt 1 */
#define IMX9_IRQ_I3C1            (IMX9_IRQ_EXTINT + 12)  /* Improved Inter-Integrated Circuit 1 interrupt */
#define IMX9_IRQ_LPI2C1          (IMX9_IRQ_EXTINT + 13)  /* Low Power Inter-Integrated Circuit module 1 */
#define IMX9_IRQ_LPI2C2          (IMX9_IRQ_EXTINT + 14)  /* Low Power Inter-Integrated Circuit module 2 */
#define IMX9_IRQ_LPIT1           (IMX9_IRQ_EXTINT + 15)  /* Low Power Periodic Interrupt Timer 1 */
#define IMX9_IRQ_LPSPI1          (IMX9_IRQ_EXTINT + 16)  /* Low Power Serial Peripheral Interface 1 */
#define IMX9_IRQ_LPSPI2          (IMX9_IRQ_EXTINT + 17)  /* Low Power Serial Peripheral Interface 2 */
#define IMX9_IRQ_LPTMR1          (IMX9_IRQ_EXTINT + 18)  /* Low Power Timer 1 */
#define IMX9_IRQ_LPUART1         (IMX9_IRQ_EXTINT + 19)  /* Low Power UART 1 */
#define IMX9_IRQ_LPUART2         (IMX9_IRQ_EXTINT + 20)  /* Low Power UART 2 */
#define IMX9_IRQ_RESERVED37      (IMX9_IRQ_EXTINT + 21)  /* AONMIX Sentinel MU0 SideA interrupt */
#define IMX9_IRQ_RESERVED38      (IMX9_IRQ_EXTINT + 22)  /* AONMIX Sentinel MU1 SideA interrupt */
#define IMX9_IRQ_RESERVED39      (IMX9_IRQ_EXTINT + 23)  /* AONMIX Sentinel MU2 SideA interrupt */
#define IMX9_IRQ_RESERVED40      (IMX9_IRQ_EXTINT + 24)  /* AONMIX Sentinel MU3 SideA interrupt */
#define IMX9_IRQ_RESERVED41      (IMX9_IRQ_EXTINT + 25)  /* AONMIX Sentinel MU4 SideA interrupt */
#define IMX9_IRQ_RESERVED42      (IMX9_IRQ_EXTINT + 26)  /* AONMIX Sentinel MU5 SideA interrupt */
#define IMX9_IRQ_V2X_FH_APCH0    (IMX9_IRQ_EXTINT + 27)  /* V2X-FH MU APCH0 (APP0) interrupt */
#define IMX9_IRQ_V2X_FH_APHSM1   (IMX9_IRQ_EXTINT + 28)  /* V2X-FH MU APHSM1 (HSM1) interrupt */
#define IMX9_IRQ_TPM1            (IMX9_IRQ_EXTINT + 29)  /* Timer PWM module 1 */
#define IMX9_IRQ_TPM2            (IMX9_IRQ_EXTINT + 30)  /* Timer PWM module 2 */
#define IMX9_IRQ_WDOG1           (IMX9_IRQ_EXTINT + 31)  /* Watchdog 1 Interrupt */
#define IMX9_IRQ_WDOG2           (IMX9_IRQ_EXTINT + 32)  /* Watchdog 2 Interrupt */
#define IMX9_IRQ_TRDC_MGR_A      (IMX9_IRQ_EXTINT + 33)  /* AONMIX TRDC transfer error interrupt */
#define IMX9_IRQ_SAI1            (IMX9_IRQ_EXTINT + 34)  /* Serial Audio Interface 1 */
#define IMX9_IRQ_RESERVED51      (IMX9_IRQ_EXTINT + 35)  /* AONMIX M33 PS Error */
#define IMX9_IRQ_RESERVED52      (IMX9_IRQ_EXTINT + 36)  /* AONMIX M33 TCM Error interrupt */
#define IMX9_IRQ_RESERVED53      (IMX9_IRQ_EXTINT + 37)  /* M7MIX ECC Multi-bit error */
#define IMX9_IRQ_CAN2            (IMX9_IRQ_EXTINT + 38)  /* CAN2 interrupt */
#define IMX9_IRQ_CAN2_ERROR      (IMX9_IRQ_EXTINT + 39)  /* CAN2 error interrupt */
#define IMX9_IRQ_CAN3            (IMX9_IRQ_EXTINT + 40)  /* CAN3 interrupt */
#define IMX9_IRQ_CAN3_ERROR      (IMX9_IRQ_EXTINT + 41)  /* CAN3 error interrupt */
#define IMX9_IRQ_CAN4            (IMX9_IRQ_EXTINT + 42)  /* CAN4 interrupt */
#define IMX9_IRQ_CAN4_ERROR      (IMX9_IRQ_EXTINT + 43)  /* CAN4 error interrupt */
#define IMX9_IRQ_CAN5            (IMX9_IRQ_EXTINT + 44)  /* CAN5 interrupt */
#define IMX9_IRQ_CAN5_ERROR      (IMX9_IRQ_EXTINT + 45)  /* CAN5 error interrupt */
#define IMX9_IRQ_FLEXIO1         (IMX9_IRQ_EXTINT + 46)  /* Flexible IO 1 interrupt */
#define IMX9_IRQ_FLEXIO2         (IMX9_IRQ_EXTINT + 47)  /* Flexible IO 2 interrupt */
#define IMX9_IRQ_FlexSPI1        (IMX9_IRQ_EXTINT + 48)  /* FlexSPI controller interface interrupt 1 */
#define IMX9_IRQ_GPIO2_0         (IMX9_IRQ_EXTINT + 49)  /* General Purpose Input/Output 2 interrupt 0 */
#define IMX9_IRQ_GPIO2_1         (IMX9_IRQ_EXTINT + 50)  /* General Purpose Input/Output 2 interrupt 1 */
#define IMX9_IRQ_GPIO3_0         (IMX9_IRQ_EXTINT + 51)  /* General Purpose Input/Output 3 interrupt 0 */
#define IMX9_IRQ_GPIO3_1         (IMX9_IRQ_EXTINT + 52)  /* General Purpose Input/Output 3 interrupt 1 */
#define IMX9_IRQ_GPIO4_0         (IMX9_IRQ_EXTINT + 53)  /* General Purpose Input/Output 4 interrupt 0 */
#define IMX9_IRQ_GPIO4_1         (IMX9_IRQ_EXTINT + 54)  /* General Purpose Input/Output 4 interrupt 1 */
#define IMX9_IRQ_GPIO5_0         (IMX9_IRQ_EXTINT + 55)  /* General Purpose Input/Output 5 interrupt 0 */
#define IMX9_IRQ_GPIO5_1         (IMX9_IRQ_EXTINT + 56)  /* General Purpose Input/Output 5 interrupt 1 */
#define IMX9_IRQ_I3C2            (IMX9_IRQ_EXTINT + 57)  /* Improved Inter-Integrated Circuit 2 interrupt */
#define IMX9_IRQ_LPI2C3          (IMX9_IRQ_EXTINT + 58)  /* Low Power Inter-Integrated Circuit module 3 */
#define IMX9_IRQ_LPI2C4          (IMX9_IRQ_EXTINT + 59)  /* Low Power Inter-Integrated Circuit module 4 */
#define IMX9_IRQ_LPIT2           (IMX9_IRQ_EXTINT + 60)  /* Low Power Periodic Interrupt Timer 2 */
#define IMX9_IRQ_LPSPI3          (IMX9_IRQ_EXTINT + 61)  /* Low Power Serial Peripheral Interface 3 */
#define IMX9_IRQ_LPSPI4          (IMX9_IRQ_EXTINT + 62)  /* Low Power Serial Peripheral Interface 4 */
#define IMX9_IRQ_LPTMR2          (IMX9_IRQ_EXTINT + 63)  /* Low Power Timer 2 */
#define IMX9_IRQ_LPUART3         (IMX9_IRQ_EXTINT + 64)  /* Low Power UART 3 */
#define IMX9_IRQ_LPUART4         (IMX9_IRQ_EXTINT + 65)  /* Low Power UART 4 */
#define IMX9_IRQ_LPUART5         (IMX9_IRQ_EXTINT + 66)  /* Low Power UART 5 */
#define IMX9_IRQ_LPUART6         (IMX9_IRQ_EXTINT + 67)  /* Low Power UART 6 */
#define IMX9_IRQ_LPUART7         (IMX9_IRQ_EXTINT + 68)  /* Low Power UART 7 */
#define IMX9_IRQ_LPUART8         (IMX9_IRQ_EXTINT + 69)  /* Low Power UART 8 */
#define IMX9_IRQ_RESERVED86      (IMX9_IRQ_EXTINT + 70)  /* MTR Master error interrupt */
#define IMX9_IRQ_RESERVED87      (IMX9_IRQ_EXTINT + 71)  /* BBNSM Non-Secure interrupt */
#define IMX9_IRQ_RESERVED88      (IMX9_IRQ_EXTINT + 72)  /* System Counter compare interrupt */
#define IMX9_IRQ_TPM3            (IMX9_IRQ_EXTINT + 73)  /* Timer PWM module 3 */
#define IMX9_IRQ_TPM4            (IMX9_IRQ_EXTINT + 74)  /* Timer PWM module 4 */
#define IMX9_IRQ_TPM5            (IMX9_IRQ_EXTINT + 75)  /* Timer PWM module 5 */
#define IMX9_IRQ_TPM6            (IMX9_IRQ_EXTINT + 76)  /* Timer PWM module 6 */
#define IMX9_IRQ_WDOG3           (IMX9_IRQ_EXTINT + 77)  /* Watchdog 3 Interrupt */
#define IMX9_IRQ_WDOG4           (IMX9_IRQ_EXTINT + 78)  /* Watchdog 4 Interrupt */
#define IMX9_IRQ_WDOG5           (IMX9_IRQ_EXTINT + 79)  /* Watchdog 5 Interrupt */
#define IMX9_IRQ_TMPSNS1_THR1    (IMX9_IRQ_EXTINT + 80)  /* ANAMIX TempSensor non-secure interrupt from Threshold 1 */
#define IMX9_IRQ_TMPSNS1_THR2    (IMX9_IRQ_EXTINT + 81)  /* ANAMIX TempSensor non-secure interrupt from Threshold 2 */
#define IMX9_IRQ_TMPSNS1_DRDY    (IMX9_IRQ_EXTINT + 82)  /* ANAMIX TempSensor non-secure data ready interrupt */
#define IMX9_IRQ_TMPSNS2_THR1    (IMX9_IRQ_EXTINT + 83)  /* CORTEXAMIX TempSensor non-secure interrupt from Threshold 1 */
#define IMX9_IRQ_TMPSNS2_THR2    (IMX9_IRQ_EXTINT + 84)  /* CORTEXAMIX TempSensor non-secure interrupt from Threshold 2 */
#define IMX9_IRQ_TMPSNS2_DRDY    (IMX9_IRQ_EXTINT + 85)  /* CORTEXAMIX TempSensor non-secure data ready interrupt */
#define IMX9_IRQ_uSDHC1          (IMX9_IRQ_EXTINT + 86)  /* ultra Secure Digital Host Controller interrupt 1 */
#define IMX9_IRQ_uSDHC2          (IMX9_IRQ_EXTINT + 87)  /* ultra Secure Digital Host Controller interrupt 2 */
#define IMX9_IRQ_RESERVED104     (IMX9_IRQ_EXTINT + 88)  /* MEGAMIX TRDC transfer error interrupt */
#define IMX9_IRQ_RESERVED105     (IMX9_IRQ_EXTINT + 89)  /* NIC_WRAPPER TRDC transfer error interrupt */
#define IMX9_IRQ_RESERVED106     (IMX9_IRQ_EXTINT + 90)  /* NOCMIX TRDC transfer error interrupt */
#define IMX9_IRQ_RESERVED107     (IMX9_IRQ_EXTINT + 91)  /* DRAM controller Performance Monitor Interrupt */
#define IMX9_IRQ_RESERVED108     (IMX9_IRQ_EXTINT + 92)  /* DRAM controller Critical Interrupt */
#define IMX9_IRQ_RESERVED109     (IMX9_IRQ_EXTINT + 93)  /* DRAM Phy Critical Interrupt */
#define IMX9_IRQ_RESERVED110     (IMX9_IRQ_EXTINT + 94)  /* Reserved */
#define IMX9_IRQ_DMA3_ERROR      (IMX9_IRQ_EXTINT + 95)  /* eDMA1 error interrupt */
#define IMX9_IRQ_DMA3_0          (IMX9_IRQ_EXTINT + 96)  /* eDMA1 channel 0 interrupt */
#define IMX9_IRQ_DMA3_1          (IMX9_IRQ_EXTINT + 97)  /* eDMA1 channel 1 interrupt */
#define IMX9_IRQ_DMA3_2          (IMX9_IRQ_EXTINT + 98)  /* eDMA1 channel 2 interrupt */
#define IMX9_IRQ_DMA3_3          (IMX9_IRQ_EXTINT + 99)  /* eDMA1 channel 3 interrupt */
#define IMX9_IRQ_DMA3_4          (IMX9_IRQ_EXTINT + 100) /* eDMA1 channel 4 interrupt */
#define IMX9_IRQ_DMA3_5          (IMX9_IRQ_EXTINT + 101) /* eDMA1 channel 5 interrupt */
#define IMX9_IRQ_DMA3_6          (IMX9_IRQ_EXTINT + 102) /* eDMA1 channel 6 interrupt */
#define IMX9_IRQ_DMA3_7          (IMX9_IRQ_EXTINT + 103) /* eDMA1 channel 7 interrupt */
#define IMX9_IRQ_DMA3_8          (IMX9_IRQ_EXTINT + 104) /* eDMA1 channel 8 interrupt */
#define IMX9_IRQ_DMA3_9          (IMX9_IRQ_EXTINT + 105) /* eDMA1 channel 9 interrupt */
#define IMX9_IRQ_DMA3_10         (IMX9_IRQ_EXTINT + 106) /* eDMA1 channel 10 interrupt */
#define IMX9_IRQ_DMA3_11         (IMX9_IRQ_EXTINT + 107) /* eDMA1 channel 11 interrupt */
#define IMX9_IRQ_DMA3_12         (IMX9_IRQ_EXTINT + 108) /* eDMA1 channel 12 interrupt */
#define IMX9_IRQ_DMA3_13         (IMX9_IRQ_EXTINT + 109) /* eDMA1 channel 13 interrupt */
#define IMX9_IRQ_DMA3_14         (IMX9_IRQ_EXTINT + 110) /* eDMA1 channel 14 interrupt */
#define IMX9_IRQ_DMA3_15         (IMX9_IRQ_EXTINT + 111) /* eDMA1 channel 15 interrupt */
#define IMX9_IRQ_DMA3_16         (IMX9_IRQ_EXTINT + 112) /* eDMA1 channel 16 interrupt */
#define IMX9_IRQ_DMA3_17         (IMX9_IRQ_EXTINT + 113) /* eDMA1 channel 17 interrupt */
#define IMX9_IRQ_DMA3_18         (IMX9_IRQ_EXTINT + 114) /* eDMA1 channel 18 interrupt */
#define IMX9_IRQ_DMA3_19         (IMX9_IRQ_EXTINT + 115) /* eDMA1 channel 19 interrupt */
#define IMX9_IRQ_DMA3_20         (IMX9_IRQ_EXTINT + 116) /* eDMA1 channel 20 interrupt */
#define IMX9_IRQ_DMA3_21         (IMX9_IRQ_EXTINT + 117) /* eDMA1 channel 21 interrupt */
#define IMX9_IRQ_DMA3_22         (IMX9_IRQ_EXTINT + 118) /* eDMA1 channel 22 interrupt */
#define IMX9_IRQ_DMA3_23         (IMX9_IRQ_EXTINT + 119) /* eDMA1 channel 23 interrupt */
#define IMX9_IRQ_DMA3_24         (IMX9_IRQ_EXTINT + 120) /* eDMA1 channel 24 interrupt */
#define IMX9_IRQ_DMA3_25         (IMX9_IRQ_EXTINT + 121) /* eDMA1 channel 25 interrupt */
#define IMX9_IRQ_DMA3_26         (IMX9_IRQ_EXTINT + 122) /* eDMA1 channel 26 interrupt */
#define IMX9_IRQ_DMA3_27         (IMX9_IRQ_EXTINT + 123) /* eDMA1 channel 27 interrupt */
#define IMX9_IRQ_DMA3_28         (IMX9_IRQ_EXTINT + 124) /* eDMA1 channel 28 interrupt */
#define IMX9_IRQ_DMA3_29         (IMX9_IRQ_EXTINT + 125) /* eDMA1 channel 29 interrupt */
#define IMX9_IRQ_DMA3_30         (IMX9_IRQ_EXTINT + 126) /* eDMA1 channel 30 interrupt */
#define IMX9_IRQ_DMA5_2_ERROR    (IMX9_IRQ_EXTINT + 127) /* eDMA2 error interrupt */
#define IMX9_IRQ_DMA5_2_0_1      (IMX9_IRQ_EXTINT + 128) /* eDMA2 channel 0/1 interrupt */
#define IMX9_IRQ_DMA5_2_2_3      (IMX9_IRQ_EXTINT + 129) /* eDMA2 channel 2/3 interrupt */
#define IMX9_IRQ_DMA5_2_4_5      (IMX9_IRQ_EXTINT + 130) /* eDMA2 channel 4/5 interrupt */
#define IMX9_IRQ_DMA5_2_6_7      (IMX9_IRQ_EXTINT + 131) /* eDMA2 channel 6/7 interrupt */
#define IMX9_IRQ_DMA5_2_8_9      (IMX9_IRQ_EXTINT + 132) /* eDMA2 channel 8/9 interrupt */
#define IMX9_IRQ_DMA5_2_10_11    (IMX9_IRQ_EXTINT + 133) /* eDMA2 channel 10/11 interrupt */
#define IMX9_IRQ_DMA5_2_12_13    (IMX9_IRQ_EXTINT + 134) /* eDMA2 channel 12/13 interrupt */
#define IMX9_IRQ_DMA5_2_14_15    (IMX9_IRQ_EXTINT + 135) /* eDMA2 channel 14/15 interrupt */
#define IMX9_IRQ_DMA5_2_16_17    (IMX9_IRQ_EXTINT + 136) /* eDMA2 channel 16/17 interrupt */
#define IMX9_IRQ_DMA5_2_18_19    (IMX9_IRQ_EXTINT + 137) /* eDMA2 channel 18/19 interrupt */
#define IMX9_IRQ_DMA5_2_20_21    (IMX9_IRQ_EXTINT + 138) /* eDMA2 channel 20/21 interrupt */
#define IMX9_IRQ_DMA5_2_22_23    (IMX9_IRQ_EXTINT + 139) /* eDMA2 channel 22/23 interrupt */
#define IMX9_IRQ_DMA5_2_24_25    (IMX9_IRQ_EXTINT + 140) /* eDMA2 channel 24/25 interrupt */
#define IMX9_IRQ_DMA5_2_26_27    (IMX9_IRQ_EXTINT + 141) /* eDMA2 channel 26/27 interrupt */
#define IMX9_IRQ_DMA5_2_28_29    (IMX9_IRQ_EXTINT + 142) /* eDMA2 channel 28/29 interrupt */
#define IMX9_IRQ_DMA5_2_30_31    (IMX9_IRQ_EXTINT + 143) /* eDMA2 channel 30/31 interrupt */
#define IMX9_IRQ_DMA5_2_32_33    (IMX9_IRQ_EXTINT + 144) /* eDMA2 channel 32/33 interrupt */
#define IMX9_IRQ_DMA5_2_34_35    (IMX9_IRQ_EXTINT + 145) /* eDMA2 channel 34/35 interrupt */
#define IMX9_IRQ_DMA5_2_36_37    (IMX9_IRQ_EXTINT + 146) /* eDMA2 channel 36/37 interrupt */
#define IMX9_IRQ_DMA5_2_38_39    (IMX9_IRQ_EXTINT + 147) /* eDMA2 channel 38/39 interrupt */
#define IMX9_IRQ_DMA5_2_40_41    (IMX9_IRQ_EXTINT + 148) /* eDMA2 channel 40/41 interrupt */
#define IMX9_IRQ_DMA5_2_42_43    (IMX9_IRQ_EXTINT + 149) /* eDMA2 channel 42/43 interrupt */
#define IMX9_IRQ_DMA5_2_44_45    (IMX9_IRQ_EXTINT + 150) /* eDMA2 channel 44/45 interrupt */
#define IMX9_IRQ_DMA5_2_46_47    (IMX9_IRQ_EXTINT + 151) /* eDMA2 channel 46/47 interrupt */
#define IMX9_IRQ_DMA5_2_48_49    (IMX9_IRQ_EXTINT + 152) /* eDMA2 channel 48/49 interrupt */
#define IMX9_IRQ_DMA5_2_50_51    (IMX9_IRQ_EXTINT + 153) /* eDMA2 channel 50/51 interrupt */
#define IMX9_IRQ_DMA5_2_52_53    (IMX9_IRQ_EXTINT + 154) /* eDMA2 channel 52/53 interrupt */
#define IMX9_IRQ_DMA5_2_54_55    (IMX9_IRQ_EXTINT + 155) /* eDMA2 channel 54/55 interrupt */
#define IMX9_IRQ_DMA5_2_56_57    (IMX9_IRQ_EXTINT + 156) /* eDMA2 channel 56/57 interrupt */
#define IMX9_IRQ_DMA5_2_58_59    (IMX9_IRQ_EXTINT + 157) /* eDMA2 channel 58/59 interrupt */
#define IMX9_IRQ_DMA5_2_60_61    (IMX9_IRQ_EXTINT + 158) /* eDMA2 channel 60/61 interrupt */
#define IMX9_IRQ_DMA5_2_62_63    (IMX9_IRQ_EXTINT + 159) /* eDMA2 channel 62/63 interrupt */
#define IMX9_IRQ_RESERVED176     (IMX9_IRQ_EXTINT + 160) /* Sentinel Group 1 reset source if no s500 reference clock is detected. Output synchronized to 32khz clk. */
#define IMX9_IRQ_RESERVED177     (IMX9_IRQ_EXTINT + 161) /* Sentinel Group 2 reset source s500 reference clock is not detected or too slow. Output synchronized to ref1_clk. */
#define IMX9_IRQ_RESERVED178     (IMX9_IRQ_EXTINT + 162) /* Sentinel Group 2 reset source s500 reference clock is not detected or too slow. Output synchronized to ref1_clk. */
#define IMX9_IRQ_RESERVED179     (IMX9_IRQ_EXTINT + 163) /* JTAGSW DAP MDM-AP SRC reset source */
#define IMX9_IRQ_RESERVED180     (IMX9_IRQ_EXTINT + 164) /* JTAGC SRC reset source */
#define IMX9_IRQ_RESERVED181     (IMX9_IRQ_EXTINT + 165) /* CM33 SYSREQRST SRC reset source */
#define IMX9_IRQ_RESERVED182     (IMX9_IRQ_EXTINT + 166) /* CM33 LOCKUP SRC reset source */
#define IMX9_IRQ_RESERVED183     (IMX9_IRQ_EXTINT + 167) /* CM7 SYSREQRST SRC reset source */
#define IMX9_IRQ_RESERVED184     (IMX9_IRQ_EXTINT + 168) /* CM7 LOCKUP SRC reset source */
#define IMX9_IRQ_SAI2            (IMX9_IRQ_EXTINT + 169) /* Serial Audio Interface 2 */
#define IMX9_IRQ_SAI3            (IMX9_IRQ_EXTINT + 170) /* Serial Audio Interface 3 */
#define IMX9_IRQ_SAI4            (IMX9_IRQ_EXTINT + 171) /* Serial Audio Interface 4 */
#define IMX9_IRQ_SAI5            (IMX9_IRQ_EXTINT + 172) /* Serial Audio Interface 5 */
#define IMX9_IRQ_RESERVED189     (IMX9_IRQ_EXTINT + 173) /* USB-1 Wake-up Interrupt */
#define IMX9_IRQ_RESERVED190     (IMX9_IRQ_EXTINT + 174) /* USB-2 Wake-up Interrupt */
#define IMX9_IRQ_USB1            (IMX9_IRQ_EXTINT + 175) /* USB-1 Interrupt */
#define IMX9_IRQ_USB2            (IMX9_IRQ_EXTINT + 176) /* USB-2 Interrupt */
#define IMX9_IRQ_LPSPI5          (IMX9_IRQ_EXTINT + 177) /* Low Power Serial Peripheral Interface 5 */
#define IMX9_IRQ_LPSPI6          (IMX9_IRQ_EXTINT + 178) /* Low Power Serial Peripheral Interface 6 */
#define IMX9_IRQ_LPSPI7          (IMX9_IRQ_EXTINT + 179) /* Low Power Serial Peripheral Interface 7 */
#define IMX9_IRQ_LPSPI8          (IMX9_IRQ_EXTINT + 180) /* Low Power Serial Peripheral Interface 8 */
#define IMX9_IRQ_LPI2C5          (IMX9_IRQ_EXTINT + 181) /* Low Power Inter-Integrated Circuit module 5 */
#define IMX9_IRQ_LPI2C6          (IMX9_IRQ_EXTINT + 182) /* Low Power Inter-Integrated Circuit module 6 */
#define IMX9_IRQ_LPI2C7          (IMX9_IRQ_EXTINT + 183) /* Low Power Inter-Integrated Circuit module 7 */
#define IMX9_IRQ_LPI2C8          (IMX9_IRQ_EXTINT + 184) /* Low Power Inter-Integrated Circuit module 8 */
#define IMX9_IRQ_PDM_HWVAD_ERROR (IMX9_IRQ_EXTINT + 185) /* PDM interrupt */
#define IMX9_IRQ_PDM_HWVAD_EVENT (IMX9_IRQ_EXTINT + 186) /* PDM interrupt */
#define IMX9_IRQ_PDM_ERROR       (IMX9_IRQ_EXTINT + 187) /* PDM interrupt */
#define IMX9_IRQ_PDM_EVENT       (IMX9_IRQ_EXTINT + 188) /* PDM interrupt */
#define IMX9_IRQ_RESERVED205     (IMX9_IRQ_EXTINT + 189) /* AUDIO XCVR interrupt */
#define IMX9_IRQ_RESERVED206     (IMX9_IRQ_EXTINT + 190) /* AUDIO XCVR interrupt */
#define IMX9_IRQ_uSDHC3          (IMX9_IRQ_EXTINT + 191) /* ultra Secure Digital Host Controller interrupt 3 */
#define IMX9_IRQ_RESERVED208     (IMX9_IRQ_EXTINT + 192) /* OCRAM MECC interrupt */
#define IMX9_IRQ_RESERVED209     (IMX9_IRQ_EXTINT + 193) /* OCRAM MECC interrupt */
#define IMX9_IRQ_RESERVED210     (IMX9_IRQ_EXTINT + 194) /* CM33 MCM interrupt */
#define IMX9_IRQ_RESERVED211     (IMX9_IRQ_EXTINT + 195) /* ANAMIX SFA interrupt */
#define IMX9_IRQ_RESERVED212     (IMX9_IRQ_EXTINT + 196) /* GIC700 Fault */
#define IMX9_IRQ_RESERVED213     (IMX9_IRQ_EXTINT + 197) /* GIC700 Error */
#define IMX9_IRQ_RESERVED214     (IMX9_IRQ_EXTINT + 198) /* GIC700 PMU Counter Overflow */
#define IMX9_IRQ_ADC_ER          (IMX9_IRQ_EXTINT + 199) /* ADC interrupt */
#define IMX9_IRQ_ADC_WD          (IMX9_IRQ_EXTINT + 200) /* ADC interrupt */
#define IMX9_IRQ_ADC_EOC         (IMX9_IRQ_EXTINT + 201) /* ADC interrupt */
#define IMX9_IRQ_RESERVED218     (IMX9_IRQ_EXTINT + 202) /* s500 glue logic IRQ */
#define IMX9_IRQ_RESERVED219     (IMX9_IRQ_EXTINT + 203) /* I3C1 wakeup irq after double sync */
#define IMX9_IRQ_RESERVED220     (IMX9_IRQ_EXTINT + 204) /* I3C2 wakeup irq after double sync */
#define IMX9_IRQ_MU5_A           (IMX9_IRQ_EXTINT + 205) /* AONMIX MU Ored of all (tx,rx,gp,core,murip) interrupt to MUA */
#define IMX9_IRQ_MU6_A           (IMX9_IRQ_EXTINT + 206) /* AONMIX MU Ored of all (tx,rx,gp,core,murip) interrupt to MUA */
#define IMX9_IRQ_MU7_B           (IMX9_IRQ_EXTINT + 207) /* WAKEUPMIX MU Ored of all (tx,rx,gp,core,murip) interrupt to MUB */
#define IMX9_IRQ_MU8_B           (IMX9_IRQ_EXTINT + 208) /* WAKEUPMIX MU Ored of all (tx,rx,gp,core,murip) interrupt to MUB */
#define IMX9_IRQ_RESERVED225     (IMX9_IRQ_EXTINT + 209) /* WAKEUPMIX XSPI Responder */
#define IMX9_IRQ_RESERVED226     (IMX9_IRQ_EXTINT + 210) /* AONMIX FCCU Interrupt Reaction 0 */
#define IMX9_IRQ_RESERVED227     (IMX9_IRQ_EXTINT + 211) /* AONMIX FCCU Interrupt Reaction 1 */
#define IMX9_IRQ_RESERVED228     (IMX9_IRQ_EXTINT + 212) /* AONMIX FCCU Interrupt Reaction 2 */
#define IMX9_IRQ_RESERVED229     (IMX9_IRQ_EXTINT + 213) /* AONMIX STCU Selftest end Interrupt */
#define IMX9_IRQ_DISP_IRQSTEER0  (IMX9_IRQ_EXTINT + 214) /* DISPLAYMIX IRQSTEER 0 */
#define IMX9_IRQ_DISP_IRQSTEER1  (IMX9_IRQ_EXTINT + 215) /* DISPLAYMIX IRQSTEER 1 */
#define IMX9_IRQ_DISP_IRQSTEER2  (IMX9_IRQ_EXTINT + 216) /* DISPLAYMIX IRQSTEER 2 */
#define IMX9_IRQ_DISP_IRQSTEER3  (IMX9_IRQ_EXTINT + 217) /* DISPLAYMIX IRQSTEER 3 */
#define IMX9_IRQ_DISP_IRQSTEER4  (IMX9_IRQ_EXTINT + 218) /* DISPLAYMIX IRQSTEER 4 */
#define IMX9_IRQ_DISP_IRQSTEER7  (IMX9_IRQ_EXTINT + 219) /* DISPLAYMIX IRQSTEER 7 */
#define IMX9_IRQ_RESERVED236     (IMX9_IRQ_EXTINT + 220) /* CAMERAMIX MU Ored of all (tx,rx,gp,core,murip) interrupt to MUA */
#define IMX9_IRQ_ISI             (IMX9_IRQ_EXTINT + 221) /* CAMERAMIX ISI interrupt Channel 0 */
#define IMX9_IRQ_RESERVED238     (IMX9_IRQ_EXTINT + 222) /* ISP Processing Interrupt - Context 0 */
#define IMX9_IRQ_RESERVED239     (IMX9_IRQ_EXTINT + 223) /* M7MIX MCM interrupt */
#define IMX9_IRQ_IRQSTEER_0      (IMX9_IRQ_EXTINT + 224) /* IRQSTEER0 interrupt */
#define IMX9_IRQ_IRQSTEER_1      (IMX9_IRQ_EXTINT + 225) /* IRQSTEER1 interrupt */
#define IMX9_IRQ_IRQSTEER_2      (IMX9_IRQ_EXTINT + 226) /* IRQSTEER2 interrupt */
#define IMX9_IRQ_IRQSTEER_3      (IMX9_IRQ_EXTINT + 227) /* IRQSTEER3 interrupt */
#define IMX9_IRQ_IRQSTEER_4      (IMX9_IRQ_EXTINT + 228) /* IRQSTEER4 interrupt */
#define IMX9_IRQ_IRQSTEER_5      (IMX9_IRQ_EXTINT + 229) /* IRQSTEER5 interrupt */
#define IMX9_IRQ_IRQSTEER_6      (IMX9_IRQ_EXTINT + 230) /* IRQSTEER6 interrupt */
#define IMX9_IRQ_IRQSTEER_7      (IMX9_IRQ_EXTINT + 231) /* IRQSTEER7 interrupt */
#define IMX9_IRQ_IRQSTEER_8      (IMX9_IRQ_EXTINT + 232) /* IRQSTEER8 interrupt */
#define IMX9_IRQ_IRQSTEER_9      (IMX9_IRQ_EXTINT + 233) /* IRQSTEER9 interrupt */
#define IMX9_IRQ_MU1_A           (IMX9_IRQ_EXTINT + 234) /* AONMIX MU Ored of all (tx,rx,gp,core,murip) interrupt to MUA */
#define IMX9_IRQ_MU1_B           (IMX9_IRQ_EXTINT + 235) /* AONMIX MU Ored of all (tx,rx,gp,core,murip) interrupt to MUB */
#define IMX9_IRQ_MU2_A           (IMX9_IRQ_EXTINT + 236) /* AONMIX MU Ored of all (tx,rx,gp,core,murip) interrupt to MUA */
#define IMX9_IRQ_MU2_B           (IMX9_IRQ_EXTINT + 237) /* AONMIX MU Ored of all (tx,rx,gp,core,murip) interrupt to MUB */
#define IMX9_IRQ_MU3_A           (IMX9_IRQ_EXTINT + 238) /* AONMIX MU Ored of all (tx,rx,gp,core,murip) interrupt to MUA */
#define IMX9_IRQ_MU3_B           (IMX9_IRQ_EXTINT + 239) /* AONMIX MU Ored of all (tx,rx,gp,core,murip) interrupt to MUB */
#define IMX9_IRQ_MU4_A           (IMX9_IRQ_EXTINT + 240) /* AONMIX MU Ored of all (tx,rx,gp,core,murip) interrupt to MUA */
#define IMX9_IRQ_MU4_B           (IMX9_IRQ_EXTINT + 241) /* AONMIX MU Ored of all (tx,rx,gp,core,murip) interrupt to MUB */
#define IMX9_IRQ_MU5_B           (IMX9_IRQ_EXTINT + 242) /* AONMIX MU Ored of all (tx,rx,gp,core,murip) interrupt to MUB */
#define IMX9_IRQ_MU6_B           (IMX9_IRQ_EXTINT + 243) /* AONMIX MU Ored of all (tx,rx,gp,core,murip) interrupt to MUB */
#define IMX9_IRQ_MU7_A           (IMX9_IRQ_EXTINT + 244) /* WAKEUPMIX MU Ored of all (tx,rx,gp,core,murip) interrupt to MUA */
#define IMX9_IRQ_MU8_A           (IMX9_IRQ_EXTINT + 245) /* WAKEUPMIX MU Ored of all (tx,rx,gp,core,murip) interrupt to MUA */
#define IMX9_IRQ_MSGINTR1        (IMX9_IRQ_EXTINT + 246) /* MSGINTR Instance 1, Interrupt */
#define IMX9_IRQ_MSGINTR2        (IMX9_IRQ_EXTINT + 247) /* MSGINTR Instance 2, Interrupts */
#define IMX9_IRQ_RESERVED264     (IMX9_IRQ_EXTINT + 248) /* V2X-FH MU APCH1 (APP1) interrupt */
#define IMX9_IRQ_RESERVED265     (IMX9_IRQ_EXTINT + 249) /* V2X-FH MU APHSM2 (HSM2) interrupt */
#define IMX9_IRQ_RESERVED266     (IMX9_IRQ_EXTINT + 250) /* CAMERAMIX TRDC transfer error interrupt */
#define IMX9_IRQ_RESERVED267     (IMX9_IRQ_EXTINT + 251) /* DISPLAYMIX TRDC transfer error interrupt */
#define IMX9_IRQ_RESERVED268     (IMX9_IRQ_EXTINT + 252) /* NETCMIX TRDC transfer error interrupt */
#define IMX9_IRQ_RESERVED269     (IMX9_IRQ_EXTINT + 253) /* GPUMIX TRDC transfer error interrupt */
#define IMX9_IRQ_RESERVED270     (IMX9_IRQ_EXTINT + 254) /* HSIOMIX TRDC transfer error interrupt */
#define IMX9_IRQ_RESERVED271     (IMX9_IRQ_EXTINT + 255) /* VPUMIX TRDC transfer error interrupt */
#define IMX9_IRQ_RESERVED272     (IMX9_IRQ_EXTINT + 256) /* AONMIX ERM Single bit corrected ECC Error */
#define IMX9_IRQ_RESERVED273     (IMX9_IRQ_EXTINT + 257) /* M7MIX ERM Single bit corrected ECC Error */
#define IMX9_IRQ_RESERVED274     (IMX9_IRQ_EXTINT + 258) /* WAKEUPMIX ERM Single bit corrected ECC Error */
#define IMX9_IRQ_RESERVED275     (IMX9_IRQ_EXTINT + 259) /* NPUMIX ERM Single bit corrected ECC Error */
#define IMX9_IRQ_RESERVED276     (IMX9_IRQ_EXTINT + 260) /* WAKEUPMIX ACP EDMA error interrupt */
#define IMX9_IRQ_RESERVED277     (IMX9_IRQ_EXTINT + 261) /* OCRAM_C ECC multiple bit or address error */
#define IMX9_IRQ_RESERVED278     (IMX9_IRQ_EXTINT + 262) /* CAMERAMIX Cortex-M0+ Cache write-buffer error */
#define IMX9_IRQ_RESERVED279     (IMX9_IRQ_EXTINT + 263) /* CAMERAMIX Cortex-M0+ Cache data parity error */
#define IMX9_IRQ_RESERVED280     (IMX9_IRQ_EXTINT + 264) /* V2X-FH MU APSHE (SHE) interrupt */
#define IMX9_IRQ_RESERVED281     (IMX9_IRQ_EXTINT + 265) /* V2X-FH MU SCU/APDEBUG (DEBUG) interrupt */
#define IMX9_IRQ_DMA5_3_0_1      (IMX9_IRQ_EXTINT + 266) /* eDMA3 channel 0/1 interrupt */
#define IMX9_IRQ_DMA5_3_2_3      (IMX9_IRQ_EXTINT + 267) /* eDMA3 channel 2/3 interrupt */
#define IMX9_IRQ_DMA5_3_4_5      (IMX9_IRQ_EXTINT + 268) /* eDMA3 channel 4/5 interrupt */
#define IMX9_IRQ_DMA5_3_6_7      (IMX9_IRQ_EXTINT + 269) /* eDMA3 channel 6/7 interrupt */
#define IMX9_IRQ_DMA5_3_8_9      (IMX9_IRQ_EXTINT + 270) /* eDMA3 channel 8/9 interrupt */
#define IMX9_IRQ_DMA5_3_10_11    (IMX9_IRQ_EXTINT + 271) /* eDMA3 channel 10/11 interrupt */
#define IMX9_IRQ_DMA5_3_12_13    (IMX9_IRQ_EXTINT + 272) /* eDMA3 channel 12/13 interrupt */
#define IMX9_IRQ_DMA5_3_14_15    (IMX9_IRQ_EXTINT + 273) /* eDMA3 channel 14/15 interrupt */
#define IMX9_IRQ_DMA5_3_16_17    (IMX9_IRQ_EXTINT + 274) /* eDMA3 channel 16/17 interrupt */
#define IMX9_IRQ_DMA5_3_18_19    (IMX9_IRQ_EXTINT + 275) /* eDMA3 channel 18/19 interrupt */
#define IMX9_IRQ_DMA5_3_20_21    (IMX9_IRQ_EXTINT + 276) /* eDMA3 channel 20/21 interrupt */
#define IMX9_IRQ_DMA5_3_22_23    (IMX9_IRQ_EXTINT + 277) /* eDMA3 channel 22/23 interrupt */
#define IMX9_IRQ_DMA5_3_24_25    (IMX9_IRQ_EXTINT + 278) /* eDMA3 channel 24/25 interrupt */
#define IMX9_IRQ_DMA5_3_26_27    (IMX9_IRQ_EXTINT + 279) /* eDMA3 channel 26/27 interrupt */
#define IMX9_IRQ_DMA5_3_28_29    (IMX9_IRQ_EXTINT + 280) /* eDMA3 channel 29/29 interrupt */
#define IMX9_IRQ_DMA5_3_30_31    (IMX9_IRQ_EXTINT + 281) /* eDMA3 channel 30/31 interrupt */
#define IMX9_IRQ_DMA5_3_32_33    (IMX9_IRQ_EXTINT + 282) /* eDMA3 channel 32/33 interrupt */
#define IMX9_IRQ_DMA5_3_34_35    (IMX9_IRQ_EXTINT + 283) /* eDMA3 channel 34/35 interrupt */
#define IMX9_IRQ_DMA5_3_36_37    (IMX9_IRQ_EXTINT + 284) /* eDMA3 channel 36/37 interrupt */
#define IMX9_IRQ_DMA5_3_38_39    (IMX9_IRQ_EXTINT + 285) /* eDMA3 channel 38/39 interrupt */
#define IMX9_IRQ_DMA5_3_40_41    (IMX9_IRQ_EXTINT + 286) /* eDMA3 channel 40/41 interrupt */
#define IMX9_IRQ_DMA5_3_42_43    (IMX9_IRQ_EXTINT + 287) /* eDMA3 channel 42/43 interrupt */
#define IMX9_IRQ_DMA5_3_44_45    (IMX9_IRQ_EXTINT + 288) /* eDMA3 channel 44/45 interrupt */
#define IMX9_IRQ_DMA5_3_46_47    (IMX9_IRQ_EXTINT + 289) /* eDMA3 channel 46/47 interrupt */
#define IMX9_IRQ_DMA5_3_48_49    (IMX9_IRQ_EXTINT + 290) /* eDMA3 channel 48/49 interrupt */
#define IMX9_IRQ_DMA5_3_50_51    (IMX9_IRQ_EXTINT + 291) /* eDMA3 channel 50/51 interrupt */
#define IMX9_IRQ_DMA5_3_52_53    (IMX9_IRQ_EXTINT + 292) /* eDMA3 channel 52/53 interrupt */
#define IMX9_IRQ_DMA5_3_54_55    (IMX9_IRQ_EXTINT + 293) /* eDMA3 channel 54/55 interrupt */
#define IMX9_IRQ_DMA5_3_56_57    (IMX9_IRQ_EXTINT + 294) /* eDMA3 channel 56/57 interrupt */
#define IMX9_IRQ_DMA5_3_58_59    (IMX9_IRQ_EXTINT + 295) /* eDMA3 channel 58/59 interrupt */
#define IMX9_IRQ_DMA5_3_60_61    (IMX9_IRQ_EXTINT + 296) /* eDMA3 channel 60/61 interrupt */
#define IMX9_IRQ_DMA5_3_62_63    (IMX9_IRQ_EXTINT + 297) /* eDMA3 channel 62/63 interrupt */
#define IMX9_IRQ_RESERVED314     (IMX9_IRQ_EXTINT + 298) /* GPUMIX GPU Interrupt */
#define IMX9_IRQ_RESERVED315     (IMX9_IRQ_EXTINT + 299) /* GPUMIX Job Interrupt */
#define IMX9_IRQ_RESERVED316     (IMX9_IRQ_EXTINT + 300) /* GPUMIX MMU Interrupt */
#define IMX9_IRQ_RESERVED317     (IMX9_IRQ_EXTINT + 301) /* Reserved INTERRUPT */
#define IMX9_IRQ_RESERVED318     (IMX9_IRQ_EXTINT + 302) /* Reserved interrupt */
#define IMX9_IRQ_RESERVED319     (IMX9_IRQ_EXTINT + 303) /* Reserved interrupt */
#define IMX9_IRQ_RESERVED320     (IMX9_IRQ_EXTINT + 304) /* Reserved interrupt */
#define IMX9_IRQ_RESERVED321     (IMX9_IRQ_EXTINT + 305) /* Reserved interrupt */
#define IMX9_IRQ_RESERVED322     (IMX9_IRQ_EXTINT + 306) /* Reserved interrupt */
#define IMX9_IRQ_RESERVED323     (IMX9_IRQ_EXTINT + 307) /* Reserved interrupt */
#define IMX9_IRQ_RESERVED324     (IMX9_IRQ_EXTINT + 308) /* Reserved interrupt */
#define IMX9_IRQ_RESERVED325     (IMX9_IRQ_EXTINT + 309) /* Reserved interrupt */
#define IMX9_IRQ_RESERVED326     (IMX9_IRQ_EXTINT + 310) /* Reserved interrupt */
#define IMX9_IRQ_RESERVED327     (IMX9_IRQ_EXTINT + 311) /* Reserved interrupt */
#define IMX9_IRQ_RESERVED328     (IMX9_IRQ_EXTINT + 312) /* Reserved interrupt */
#define IMX9_IRQ_RESERVED329     (IMX9_IRQ_EXTINT + 313) /* Reserved interrupt */
#define IMX9_IRQ_RESERVED330     (IMX9_IRQ_EXTINT + 314) /* NETC iEPRC PCI INT */
#define IMX9_IRQ_RESERVED331     (IMX9_IRQ_EXTINT + 315) /* NETC iEPRC PCI INT */
#define IMX9_IRQ_RESERVED332     (IMX9_IRQ_EXTINT + 316) /* PCIe Controller 1 INTA */
#define IMX9_IRQ_RESERVED333     (IMX9_IRQ_EXTINT + 317) /* PCIe Controller 1 INTB */
#define IMX9_IRQ_RESERVED334     (IMX9_IRQ_EXTINT + 318) /* PCIe Controller 1 INTC */
#define IMX9_IRQ_RESERVED335     (IMX9_IRQ_EXTINT + 319) /* PCIe Controller 1 INTD */
#define IMX9_IRQ_RESERVED336     (IMX9_IRQ_EXTINT + 320) /* PCIe interrupts */
#define IMX9_IRQ_RESERVED337     (IMX9_IRQ_EXTINT + 321) /* PCIe Controller EDMA channel interrupt */
#define IMX9_IRQ_RESERVED338     (IMX9_IRQ_EXTINT + 322) /* PCIe Controller 1 INTA */
#define IMX9_IRQ_RESERVED339     (IMX9_IRQ_EXTINT + 323) /* PCIe Controller 1 INTB */
#define IMX9_IRQ_RESERVED340     (IMX9_IRQ_EXTINT + 324) /* PCIe Controller 1 INTC */
#define IMX9_IRQ_RESERVED341     (IMX9_IRQ_EXTINT + 325) /* PCIe Controller 1 INTD */
#define IMX9_IRQ_RESERVED342     (IMX9_IRQ_EXTINT + 326) /* PCIe miscellaneous interrupts */
#define IMX9_IRQ_RESERVED343     (IMX9_IRQ_EXTINT + 327) /* PCIe Controller EDMA channel interrupt */
#define IMX9_IRQ_RESERVED344     (IMX9_IRQ_EXTINT + 328) /* Wakeup interrupt from CLKREQ#, WAKEUP#, BEACON_DET */
#define IMX9_IRQ_RESERVED345     (IMX9_IRQ_EXTINT + 329) /* NPUMIX Functional interrupt */
#define IMX9_IRQ_RESERVED346     (IMX9_IRQ_EXTINT + 330) /* DISPLAYMIX Real-time traffic TBU: Fault Handling RAS Interrupt for a contained error */
#define IMX9_IRQ_RESERVED347     (IMX9_IRQ_EXTINT + 331) /* DISPLAYMIX Real-time traffic TBU: Error Handling RAS Interrupt for an uncontained error */
#define IMX9_IRQ_RESERVED348     (IMX9_IRQ_EXTINT + 332) /* DISPLAYMIX Real-time traffic TBU: Critical Error Interrupt for an uncontainable error */
#define IMX9_IRQ_RESERVED349     (IMX9_IRQ_EXTINT + 333) /* DISPLAYMIX Real-time traffic TBU: PMU Interrupt */
#define IMX9_IRQ_RESERVED350     (IMX9_IRQ_EXTINT + 334) /* TCU Event queue, secure interrupt */
#define IMX9_IRQ_RESERVED351     (IMX9_IRQ_EXTINT + 335) /* TCU Event queue, non-secure interrupt */
#define IMX9_IRQ_RESERVED352     (IMX9_IRQ_EXTINT + 336) /* TCU SYNC complete, non-secure interrupt */
#define IMX9_IRQ_RESERVED353     (IMX9_IRQ_EXTINT + 337) /* TCU SYNC complete, secure interrupt */
#define IMX9_IRQ_RESERVED354     (IMX9_IRQ_EXTINT + 338) /* TCU global non-secure interrupt */
#define IMX9_IRQ_RESERVED355     (IMX9_IRQ_EXTINT + 339) /* TCU global secure interrupt */
#define IMX9_IRQ_RESERVED356     (IMX9_IRQ_EXTINT + 340) /* TCU fault handling RAS interrupt for a contained error */
#define IMX9_IRQ_RESERVED357     (IMX9_IRQ_EXTINT + 341) /* TCU error recovery RAS interrupt for an uncontained error */
#define IMX9_IRQ_RESERVED358     (IMX9_IRQ_EXTINT + 342) /* TCU critical error interrupt, for an uncontainable uncorrected error */
#define IMX9_IRQ_RESERVED359     (IMX9_IRQ_EXTINT + 343) /* TCU PMU interrupt */
#define IMX9_IRQ_RESERVED360     (IMX9_IRQ_EXTINT + 344) /* TCU Page Request Interface */
#define IMX9_IRQ_RESERVED361     (IMX9_IRQ_EXTINT + 345) /* SRC GPC Low Power Handshake Gasket interrupt request for system management */
#define IMX9_IRQ_RESERVED362     (IMX9_IRQ_EXTINT + 346) /* CAMERAMIX MU Ored of all */
#define IMX9_IRQ_RESERVED363     (IMX9_IRQ_EXTINT + 347) /* CAMERAMIX MU Ored of all */
#define IMX9_IRQ_RESERVED364     (IMX9_IRQ_EXTINT + 348) /* CAMERAMIX MU Ored of all */
#define IMX9_IRQ_RESERVED365     (IMX9_IRQ_EXTINT + 349) /* CAMERAMIX MU Ored of all */
#define IMX9_IRQ_RESERVED366     (IMX9_IRQ_EXTINT + 350) /* CAMERAMIX MU Ored of all */
#define IMX9_IRQ_RESERVED367     (IMX9_IRQ_EXTINT + 351) /* CAMERAMIX MU Ored of all */
#define IMX9_IRQ_RESERVED368     (IMX9_IRQ_EXTINT + 352) /* CAMERAMIX MU Ored of all */
#define IMX9_IRQ_RESERVED369     (IMX9_IRQ_EXTINT + 353) /* CAMERAMIX MU Ored of all */
#define IMX9_IRQ_RESERVED370     (IMX9_IRQ_EXTINT + 354) /* CAMERAMIX ISI interrupt Channel 1 */
#define IMX9_IRQ_RESERVED371     (IMX9_IRQ_EXTINT + 355) /* CAMERAMIX ISI interrupt Channel 2 */
#define IMX9_IRQ_RESERVED372     (IMX9_IRQ_EXTINT + 356) /* CAMERAMIX ISI interrupt Channel 3 */
#define IMX9_IRQ_RESERVED373     (IMX9_IRQ_EXTINT + 357) /* CAMERAMIX ISI interrupt Channel 4 */
#define IMX9_IRQ_RESERVED374     (IMX9_IRQ_EXTINT + 358) /* CAMERAMIX ISI interrupt Channel 5 */
#define IMX9_IRQ_RESERVED375     (IMX9_IRQ_EXTINT + 359) /* CAMERAMIX ISI interrupt Channel 6 */
#define IMX9_IRQ_RESERVED376     (IMX9_IRQ_EXTINT + 360) /* CAMERAMIX ISI interrupt Channel 7 */
#define IMX9_IRQ_DMA5_4_ERROR    (IMX9_IRQ_EXTINT + 361) /* CAMERAMIX EDMA error interrupt */
#define IMX9_IRQ_DMA5_4_0_1      (IMX9_IRQ_EXTINT + 362) /* CAMERAMIX EDMA channel 0 interrupt */
#define IMX9_IRQ_DMA5_4_2_3      (IMX9_IRQ_EXTINT + 363) /* CAMERAMIX EDMA channel 2 interrupt */
#define IMX9_IRQ_DMA5_4_4_5      (IMX9_IRQ_EXTINT + 364) /* CAMERAMIX EDMA channel 4 interrupt */
#define IMX9_IRQ_DMA5_4_6_7      (IMX9_IRQ_EXTINT + 365) /* CAMERAMIX EDMA channel 6 interrupt */
#define IMX9_IRQ_DMA5_4_8_9      (IMX9_IRQ_EXTINT + 366) /* CAMERAMIX EDMA channel 8 interrupt */
#define IMX9_IRQ_DMA5_4_10_11    (IMX9_IRQ_EXTINT + 367) /* CAMERAMIX EDMA channel 10 interrupt */
#define IMX9_IRQ_DMA5_4_12_13    (IMX9_IRQ_EXTINT + 368) /* CAMERAMIX EDMA channel 12 interrupt */
#define IMX9_IRQ_DMA5_4_14_15    (IMX9_IRQ_EXTINT + 369) /* CAMERAMIX EDMA channel 14 interrupt */
#define IMX9_IRQ_DMA5_4_16_17    (IMX9_IRQ_EXTINT + 370) /* CAMERAMIX EDMA channel 16 interrupt */
#define IMX9_IRQ_DMA5_4_18_19    (IMX9_IRQ_EXTINT + 371) /* CAMERAMIX EDMA channel 18 interrupt */
#define IMX9_IRQ_DMA5_4_20_21    (IMX9_IRQ_EXTINT + 372) /* CAMERAMIX EDMA channel 20 interrupt */
#define IMX9_IRQ_DMA5_4_22_23    (IMX9_IRQ_EXTINT + 373) /* CAMERAMIX EDMA channel 22 interrupt */
#define IMX9_IRQ_DMA5_4_24_25    (IMX9_IRQ_EXTINT + 374) /* CAMERAMIX EDMA channel 24 interrupt */
#define IMX9_IRQ_DMA5_4_26_27    (IMX9_IRQ_EXTINT + 375) /* CAMERAMIX EDMA channel 26 interrupt */
#define IMX9_IRQ_DMA5_4_28_29    (IMX9_IRQ_EXTINT + 376) /* CAMERAMIX EDMA channel 28 interrupt */
#define IMX9_IRQ_DMA5_4_30_31    (IMX9_IRQ_EXTINT + 377) /* CAMERAMIX EDMA channel 30 interrupt */
#define IMX9_IRQ_RESERVED394     (IMX9_IRQ_EXTINT + 378) /* CAMERAMIX CSI Formatting Unit 1: Buffer overflow */
#define IMX9_IRQ_RESERVED395     (IMX9_IRQ_EXTINT + 379) /* CAMERAMIX CSI Formatting Unit 1: Interlaced Error */
#define IMX9_IRQ_RESERVED396     (IMX9_IRQ_EXTINT + 380) /* CAMERAMIX CSI Formatting Unit 1: Pixel Data Type Error */
#define IMX9_IRQ_RESERVED397     (IMX9_IRQ_EXTINT + 381) /* CAMERAMIX CSI Formatting Unit 2: Buffer overflow */
#define IMX9_IRQ_RESERVED398     (IMX9_IRQ_EXTINT + 382) /* CAMERAMIX CSI Formatting Unit 2: Interlaced Error */
#define IMX9_IRQ_RESERVED399     (IMX9_IRQ_EXTINT + 383) /* CAMERAMIX CSI Formatting Unit 2: Pixel Data Type Error */
#define IMX9_IRQ_RESERVED400     (IMX9_IRQ_EXTINT + 384) /* CAMERAMIX CSI1 */
#define IMX9_IRQ_RESERVED401     (IMX9_IRQ_EXTINT + 385) /* CAMERAMIX CSI2 */

#define IMX9_IRQ_NEXTINT        (218)

/* Total amount of entries in system vector table */

#define NR_IRQS                  (IMX9_IRQ_EXTINT + IMX9_IRQ_NEXTINT)

#endif /* __ARCH_ARM_INCLUDE_IMX9_IMX95_IRQ_H */
