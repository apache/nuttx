/****************************************************************************
 * arch/arm64/include/imx9/imx95_irq.h
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

#ifndef __ARCH_ARM64_INCLUDE_IMX9_IMX95_IRQ_H
#define __ARCH_ARM64_INCLUDE_IMX9_IMX95_IRQ_H

#define IMX9_IRQ_RESERVED16      (IMX9_IRQ_EXT + 0)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED17      (IMX9_IRQ_EXT + 1)   /* DAP interrupt */
#define IMX9_IRQ_RESERVED18      (IMX9_IRQ_EXT + 2)   /* CTI trigger outputs from CM7 platform */
#define IMX9_IRQ_RESERVED19      (IMX9_IRQ_EXT + 3)   /* CTI trigger outputs from CM33 platform */
#define IMX9_IRQ_RESERVED20      (IMX9_IRQ_EXT + 4)   /* CTI trigger outputs from CA55 platform */
#define IMX9_IRQ_RESERVED21      (IMX9_IRQ_EXT + 5)   /* Performance Unit Interrupts from CA55 platform */
#define IMX9_IRQ_RESERVED22      (IMX9_IRQ_EXT + 6)   /* ECC error from CA55 platform cache */
#define IMX9_IRQ_RESERVED23      (IMX9_IRQ_EXT + 7)   /* 1-bit or 2-bit ECC or Parity error from CA55 platform cache */
#define IMX9_IRQ_CAN1            (IMX9_IRQ_EXT + 8)   /* CAN1 interrupt */
#define IMX9_IRQ_CAN1_ERROR      (IMX9_IRQ_EXT + 9)   /* CAN1 error interrupt */
#define IMX9_IRQ_GPIO1_0         (IMX9_IRQ_EXT + 10)  /* General Purpose Input/Output 1 interrupt 0 */
#define IMX9_IRQ_GPIO1_1         (IMX9_IRQ_EXT + 11)  /* General Purpose Input/Output 1 interrupt 1 */
#define IMX9_IRQ_I3C1            (IMX9_IRQ_EXT + 12)  /* Improved Inter-Integrated Circuit 1 interrupt */
#define IMX9_IRQ_LPI2C1          (IMX9_IRQ_EXT + 13)  /* Low Power Inter-Integrated Circuit module 1 */
#define IMX9_IRQ_LPI2C2          (IMX9_IRQ_EXT + 14)  /* Low Power Inter-Integrated Circuit module 2 */
#define IMX9_IRQ_LPIT1           (IMX9_IRQ_EXT + 15)  /* Low Power Periodic Interrupt Timer 1 */
#define IMX9_IRQ_LPSPI1          (IMX9_IRQ_EXT + 16)  /* Low Power Serial Peripheral Interface 1 */
#define IMX9_IRQ_LPSPI2          (IMX9_IRQ_EXT + 17)  /* Low Power Serial Peripheral Interface 2 */
#define IMX9_IRQ_LPTMR1          (IMX9_IRQ_EXT + 18)  /* Low Power Timer 1 */
#define IMX9_IRQ_LPUART1         (IMX9_IRQ_EXT + 19)  /* Low Power UART 1 */
#define IMX9_IRQ_LPUART2         (IMX9_IRQ_EXT + 20)  /* Low Power UART 2 */
#define IMX9_IRQ_RESERVED37      (IMX9_IRQ_EXT + 21)  /* AONMIX Sentinel MU0 SideA interrupt */
#define IMX9_IRQ_RESERVED38      (IMX9_IRQ_EXT + 22)  /* AONMIX Sentinel MU1 SideA interrupt */
#define IMX9_IRQ_RESERVED39      (IMX9_IRQ_EXT + 23)  /* AONMIX Sentinel MU2 SideA interrupt */
#define IMX9_IRQ_RESERVED40      (IMX9_IRQ_EXT + 24)  /* AONMIX Sentinel MU3 SideA interrupt */
#define IMX9_IRQ_RESERVED41      (IMX9_IRQ_EXT + 25)  /* AONMIX Sentinel MU4 SideA interrupt */
#define IMX9_IRQ_RESERVED42      (IMX9_IRQ_EXT + 26)  /* AONMIX Sentinel MU5 SideA interrupt */
#define IMX9_IRQ_V2X_FH_APCH0    (IMX9_IRQ_EXT + 27)  /* V2X-FH MU APCH0 (APP0) interrupt */
#define IMX9_IRQ_V2X_FH_APHSM1   (IMX9_IRQ_EXT + 28)  /* V2X-FH MU APHSM1 (HSM1) interrupt */
#define IMX9_IRQ_TPM1            (IMX9_IRQ_EXT + 29)  /* Timer PWM module 1 */
#define IMX9_IRQ_TPM2            (IMX9_IRQ_EXT + 30)  /* Timer PWM module 2 */
#define IMX9_IRQ_WDOG1           (IMX9_IRQ_EXT + 31)  /* Watchdog 1 Interrupt */
#define IMX9_IRQ_WDOG2           (IMX9_IRQ_EXT + 32)  /* Watchdog 2 Interrupt */
#define IMX9_IRQ_TRDC_MGR_A      (IMX9_IRQ_EXT + 33)  /* AONMIX TRDC transfer error interrupt */
#define IMX9_IRQ_SAI1            (IMX9_IRQ_EXT + 34)  /* Serial Audio Interface 1 */
#define IMX9_IRQ_RESERVED51      (IMX9_IRQ_EXT + 35)  /* AONMIX M33 PS Error */
#define IMX9_IRQ_RESERVED52      (IMX9_IRQ_EXT + 36)  /* AONMIX M33 TCM Error interrupt */
#define IMX9_IRQ_RESERVED53      (IMX9_IRQ_EXT + 37)  /* M7MIX ECC Multi-bit error */
#define IMX9_IRQ_CAN2            (IMX9_IRQ_EXT + 38)  /* CAN2 interrupt */
#define IMX9_IRQ_CAN2_ERROR      (IMX9_IRQ_EXT + 39)  /* CAN2 error interrupt */
#define IMX9_IRQ_CAN3            (IMX9_IRQ_EXT + 40)  /* CAN3 interrupt */
#define IMX9_IRQ_CAN3_ERROR      (IMX9_IRQ_EXT + 41)  /* CAN3 error interrupt */
#define IMX9_IRQ_CAN4            (IMX9_IRQ_EXT + 42)  /* CAN4 interrupt */
#define IMX9_IRQ_CAN4_ERROR      (IMX9_IRQ_EXT + 43)  /* CAN4 error interrupt */
#define IMX9_IRQ_CAN5            (IMX9_IRQ_EXT + 44)  /* CAN5 interrupt */
#define IMX9_IRQ_CAN5_ERROR      (IMX9_IRQ_EXT + 45)  /* CAN5 error interrupt */
#define IMX9_IRQ_FLEXIO1         (IMX9_IRQ_EXT + 46)  /* Flexible IO 1 interrupt */
#define IMX9_IRQ_FLEXIO2         (IMX9_IRQ_EXT + 47)  /* Flexible IO 2 interrupt */
#define IMX9_IRQ_FlEXSPI1        (IMX9_IRQ_EXT + 48)  /* FlexSPI controller interface interrupt 1 */
#define IMX9_IRQ_GPIO2_0         (IMX9_IRQ_EXT + 49)  /* General Purpose Input/Output 2 interrupt 0 */
#define IMX9_IRQ_GPIO2_1         (IMX9_IRQ_EXT + 50)  /* General Purpose Input/Output 2 interrupt 1 */
#define IMX9_IRQ_GPIO3_0         (IMX9_IRQ_EXT + 51)  /* General Purpose Input/Output 3 interrupt 0 */
#define IMX9_IRQ_GPIO3_1         (IMX9_IRQ_EXT + 52)  /* General Purpose Input/Output 3 interrupt 1 */
#define IMX9_IRQ_GPIO4_0         (IMX9_IRQ_EXT + 53)  /* General Purpose Input/Output 4 interrupt 0 */
#define IMX9_IRQ_GPIO4_1         (IMX9_IRQ_EXT + 54)  /* General Purpose Input/Output 4 interrupt 1 */
#define IMX9_IRQ_GPIO5_0         (IMX9_IRQ_EXT + 55)  /* General Purpose Input/Output 5 interrupt 0 */
#define IMX9_IRQ_GPIO5_1         (IMX9_IRQ_EXT + 56)  /* General Purpose Input/Output 5 interrupt 1 */
#define IMX9_IRQ_I3C2            (IMX9_IRQ_EXT + 57)  /* Improved Inter-Integrated Circuit 2 interrupt */
#define IMX9_IRQ_LPI2C3          (IMX9_IRQ_EXT + 58)  /* Low Power Inter-Integrated Circuit module 3 */
#define IMX9_IRQ_LPI2C4          (IMX9_IRQ_EXT + 59)  /* Low Power Inter-Integrated Circuit module 4 */
#define IMX9_IRQ_LPIT2           (IMX9_IRQ_EXT + 60)  /* Low Power Periodic Interrupt Timer 2 */
#define IMX9_IRQ_LPSPI3          (IMX9_IRQ_EXT + 61)  /* Low Power Serial Peripheral Interface 3 */
#define IMX9_IRQ_LPSPI4          (IMX9_IRQ_EXT + 62)  /* Low Power Serial Peripheral Interface 4 */
#define IMX9_IRQ_LPTMR2          (IMX9_IRQ_EXT + 63)  /* Low Power Timer 2 */
#define IMX9_IRQ_LPUART3         (IMX9_IRQ_EXT + 64)  /* Low Power UART 3 */
#define IMX9_IRQ_LPUART4         (IMX9_IRQ_EXT + 65)  /* Low Power UART 4 */
#define IMX9_IRQ_LPUART5         (IMX9_IRQ_EXT + 66)  /* Low Power UART 5 */
#define IMX9_IRQ_LPUART6         (IMX9_IRQ_EXT + 67)  /* Low Power UART 6 */
#define IMX9_IRQ_LPUART7         (IMX9_IRQ_EXT + 68)  /* Low Power UART 7 */
#define IMX9_IRQ_LPUART8         (IMX9_IRQ_EXT + 69)  /* Low Power UART 8 */
#define IMX9_IRQ_RESERVED86      (IMX9_IRQ_EXT + 70)  /* MTR Master error interrupt */
#define IMX9_IRQ_RESERVED87      (IMX9_IRQ_EXT + 71)  /* BBNSM Non-Secure interrupt */
#define IMX9_IRQ_RESERVED88      (IMX9_IRQ_EXT + 72)  /* System Counter compare interrupt */
#define IMX9_IRQ_TPM3            (IMX9_IRQ_EXT + 73)  /* Timer PWM module 3 */
#define IMX9_IRQ_TPM4            (IMX9_IRQ_EXT + 74)  /* Timer PWM module 4 */
#define IMX9_IRQ_TPM5            (IMX9_IRQ_EXT + 75)  /* Timer PWM module 5 */
#define IMX9_IRQ_TPM6            (IMX9_IRQ_EXT + 76)  /* Timer PWM module 6 */
#define IMX9_IRQ_WDOG3           (IMX9_IRQ_EXT + 77)  /* Watchdog 3 Interrupt */
#define IMX9_IRQ_WDOG4           (IMX9_IRQ_EXT + 78)  /* Watchdog 4 Interrupt */
#define IMX9_IRQ_WDOG5           (IMX9_IRQ_EXT + 79)  /* Watchdog 5 Interrupt */
#define IMX9_IRQ_TMPSNS1_THR1    (IMX9_IRQ_EXT + 80)  /* ANAMIX TempSensor non-secure interrupt from Threshold 1 */
#define IMX9_IRQ_TMPSNS1_THR2    (IMX9_IRQ_EXT + 81)  /* ANAMIX TempSensor non-secure interrupt from Threshold 2 */
#define IMX9_IRQ_TMPSNS1_DRDY    (IMX9_IRQ_EXT + 82)  /* ANAMIX TempSensor non-secure data ready interrupt */
#define IMX9_IRQ_TMPSNS2_THR1    (IMX9_IRQ_EXT + 83)  /* CORTEXAMIX TempSensor non-secure interrupt from Threshold 1 */
#define IMX9_IRQ_TMPSNS2_THR2    (IMX9_IRQ_EXT + 84)  /* CORTEXAMIX TempSensor non-secure interrupt from Threshold 2 */
#define IMX9_IRQ_TMPSNS2_DRDY    (IMX9_IRQ_EXT + 85)  /* CORTEXAMIX TempSensor non-secure data ready interrupt */
#define IMX9_IRQ_uSDHC1          (IMX9_IRQ_EXT + 86)  /* ultra Secure Digital Host Controller interrupt 1 */
#define IMX9_IRQ_uSDHC2          (IMX9_IRQ_EXT + 87)  /* ultra Secure Digital Host Controller interrupt 2 */
#define IMX9_IRQ_RESERVED104     (IMX9_IRQ_EXT + 88)  /* MEGAMIX TRDC transfer error interrupt */
#define IMX9_IRQ_RESERVED105     (IMX9_IRQ_EXT + 89)  /* NIC_WRAPPER TRDC transfer error interrupt */
#define IMX9_IRQ_RESERVED106     (IMX9_IRQ_EXT + 90)  /* NOCMIX TRDC transfer error interrupt */
#define IMX9_IRQ_RESERVED107     (IMX9_IRQ_EXT + 91)  /* DRAM controller Performance Monitor Interrupt */
#define IMX9_IRQ_RESERVED108     (IMX9_IRQ_EXT + 92)  /* DRAM controller Critical Interrupt */
#define IMX9_IRQ_RESERVED109     (IMX9_IRQ_EXT + 93)  /* DRAM Phy Critical Interrupt */
#define IMX9_IRQ_RESERVED110     (IMX9_IRQ_EXT + 94)  /* Reserved */
#define IMX9_IRQ_DMA3_ERROR      (IMX9_IRQ_EXT + 95)  /* eDMA1 error interrupt */
#define IMX9_IRQ_DMA3_0          (IMX9_IRQ_EXT + 96)  /* eDMA1 channel 0 interrupt */
#define IMX9_IRQ_DMA3_1          (IMX9_IRQ_EXT + 97)  /* eDMA1 channel 1 interrupt */
#define IMX9_IRQ_DMA3_2          (IMX9_IRQ_EXT + 98)  /* eDMA1 channel 2 interrupt */
#define IMX9_IRQ_DMA3_3          (IMX9_IRQ_EXT + 99)  /* eDMA1 channel 3 interrupt */
#define IMX9_IRQ_DMA3_4          (IMX9_IRQ_EXT + 100) /* eDMA1 channel 4 interrupt */
#define IMX9_IRQ_DMA3_5          (IMX9_IRQ_EXT + 101) /* eDMA1 channel 5 interrupt */
#define IMX9_IRQ_DMA3_6          (IMX9_IRQ_EXT + 102) /* eDMA1 channel 6 interrupt */
#define IMX9_IRQ_DMA3_7          (IMX9_IRQ_EXT + 103) /* eDMA1 channel 7 interrupt */
#define IMX9_IRQ_DMA3_8          (IMX9_IRQ_EXT + 104) /* eDMA1 channel 8 interrupt */
#define IMX9_IRQ_DMA3_9          (IMX9_IRQ_EXT + 105) /* eDMA1 channel 9 interrupt */
#define IMX9_IRQ_DMA3_10         (IMX9_IRQ_EXT + 106) /* eDMA1 channel 10 interrupt */
#define IMX9_IRQ_DMA3_11         (IMX9_IRQ_EXT + 107) /* eDMA1 channel 11 interrupt */
#define IMX9_IRQ_DMA3_12         (IMX9_IRQ_EXT + 108) /* eDMA1 channel 12 interrupt */
#define IMX9_IRQ_DMA3_13         (IMX9_IRQ_EXT + 109) /* eDMA1 channel 13 interrupt */
#define IMX9_IRQ_DMA3_14         (IMX9_IRQ_EXT + 110) /* eDMA1 channel 14 interrupt */
#define IMX9_IRQ_DMA3_15         (IMX9_IRQ_EXT + 111) /* eDMA1 channel 15 interrupt */
#define IMX9_IRQ_DMA3_16         (IMX9_IRQ_EXT + 112) /* eDMA1 channel 16 interrupt */
#define IMX9_IRQ_DMA3_17         (IMX9_IRQ_EXT + 113) /* eDMA1 channel 17 interrupt */
#define IMX9_IRQ_DMA3_18         (IMX9_IRQ_EXT + 114) /* eDMA1 channel 18 interrupt */
#define IMX9_IRQ_DMA3_19         (IMX9_IRQ_EXT + 115) /* eDMA1 channel 19 interrupt */
#define IMX9_IRQ_DMA3_20         (IMX9_IRQ_EXT + 116) /* eDMA1 channel 20 interrupt */
#define IMX9_IRQ_DMA3_21         (IMX9_IRQ_EXT + 117) /* eDMA1 channel 21 interrupt */
#define IMX9_IRQ_DMA3_22         (IMX9_IRQ_EXT + 118) /* eDMA1 channel 22 interrupt */
#define IMX9_IRQ_DMA3_23         (IMX9_IRQ_EXT + 119) /* eDMA1 channel 23 interrupt */
#define IMX9_IRQ_DMA3_24         (IMX9_IRQ_EXT + 120) /* eDMA1 channel 24 interrupt */
#define IMX9_IRQ_DMA3_25         (IMX9_IRQ_EXT + 121) /* eDMA1 channel 25 interrupt */
#define IMX9_IRQ_DMA3_26         (IMX9_IRQ_EXT + 122) /* eDMA1 channel 26 interrupt */
#define IMX9_IRQ_DMA3_27         (IMX9_IRQ_EXT + 123) /* eDMA1 channel 27 interrupt */
#define IMX9_IRQ_DMA3_28         (IMX9_IRQ_EXT + 124) /* eDMA1 channel 28 interrupt */
#define IMX9_IRQ_DMA3_29         (IMX9_IRQ_EXT + 125) /* eDMA1 channel 29 interrupt */
#define IMX9_IRQ_DMA3_30         (IMX9_IRQ_EXT + 126) /* eDMA1 channel 30 interrupt */
#define IMX9_IRQ_DMA5_2_ERROR    (IMX9_IRQ_EXT + 127) /* eDMA2 error interrupt */
#define IMX9_IRQ_DMA5_2_0_1      (IMX9_IRQ_EXT + 128) /* eDMA2 channel 0/1 interrupt */
#define IMX9_IRQ_DMA5_2_2_3      (IMX9_IRQ_EXT + 129) /* eDMA2 channel 2/3 interrupt */
#define IMX9_IRQ_DMA5_2_4_5      (IMX9_IRQ_EXT + 130) /* eDMA2 channel 4/5 interrupt */
#define IMX9_IRQ_DMA5_2_6_7      (IMX9_IRQ_EXT + 131) /* eDMA2 channel 6/7 interrupt */
#define IMX9_IRQ_DMA5_2_8_9      (IMX9_IRQ_EXT + 132) /* eDMA2 channel 8/9 interrupt */
#define IMX9_IRQ_DMA5_2_10_11    (IMX9_IRQ_EXT + 133) /* eDMA2 channel 10/11 interrupt */
#define IMX9_IRQ_DMA5_2_12_13    (IMX9_IRQ_EXT + 134) /* eDMA2 channel 12/13 interrupt */
#define IMX9_IRQ_DMA5_2_14_15    (IMX9_IRQ_EXT + 135) /* eDMA2 channel 14/15 interrupt */
#define IMX9_IRQ_DMA5_2_16_17    (IMX9_IRQ_EXT + 136) /* eDMA2 channel 16/17 interrupt */
#define IMX9_IRQ_DMA5_2_18_19    (IMX9_IRQ_EXT + 137) /* eDMA2 channel 18/19 interrupt */
#define IMX9_IRQ_DMA5_2_20_21    (IMX9_IRQ_EXT + 138) /* eDMA2 channel 20/21 interrupt */
#define IMX9_IRQ_DMA5_2_22_23    (IMX9_IRQ_EXT + 139) /* eDMA2 channel 22/23 interrupt */
#define IMX9_IRQ_DMA5_2_24_25    (IMX9_IRQ_EXT + 140) /* eDMA2 channel 24/25 interrupt */
#define IMX9_IRQ_DMA5_2_26_27    (IMX9_IRQ_EXT + 141) /* eDMA2 channel 26/27 interrupt */
#define IMX9_IRQ_DMA5_2_28_29    (IMX9_IRQ_EXT + 142) /* eDMA2 channel 28/29 interrupt */
#define IMX9_IRQ_DMA5_2_30_31    (IMX9_IRQ_EXT + 143) /* eDMA2 channel 30/31 interrupt */
#define IMX9_IRQ_DMA5_2_32_33    (IMX9_IRQ_EXT + 144) /* eDMA2 channel 32/33 interrupt */
#define IMX9_IRQ_DMA5_2_34_35    (IMX9_IRQ_EXT + 145) /* eDMA2 channel 34/35 interrupt */
#define IMX9_IRQ_DMA5_2_36_37    (IMX9_IRQ_EXT + 146) /* eDMA2 channel 36/37 interrupt */
#define IMX9_IRQ_DMA5_2_38_39    (IMX9_IRQ_EXT + 147) /* eDMA2 channel 38/39 interrupt */
#define IMX9_IRQ_DMA5_2_40_41    (IMX9_IRQ_EXT + 148) /* eDMA2 channel 40/41 interrupt */
#define IMX9_IRQ_DMA5_2_42_43    (IMX9_IRQ_EXT + 149) /* eDMA2 channel 42/43 interrupt */
#define IMX9_IRQ_DMA5_2_44_45    (IMX9_IRQ_EXT + 150) /* eDMA2 channel 44/45 interrupt */
#define IMX9_IRQ_DMA5_2_46_47    (IMX9_IRQ_EXT + 151) /* eDMA2 channel 46/47 interrupt */
#define IMX9_IRQ_DMA5_2_48_49    (IMX9_IRQ_EXT + 152) /* eDMA2 channel 48/49 interrupt */
#define IMX9_IRQ_DMA5_2_50_51    (IMX9_IRQ_EXT + 153) /* eDMA2 channel 50/51 interrupt */
#define IMX9_IRQ_DMA5_2_52_53    (IMX9_IRQ_EXT + 154) /* eDMA2 channel 52/53 interrupt */
#define IMX9_IRQ_DMA5_2_54_55    (IMX9_IRQ_EXT + 155) /* eDMA2 channel 54/55 interrupt */
#define IMX9_IRQ_DMA5_2_56_57    (IMX9_IRQ_EXT + 156) /* eDMA2 channel 56/57 interrupt */
#define IMX9_IRQ_DMA5_2_58_59    (IMX9_IRQ_EXT + 157) /* eDMA2 channel 58/59 interrupt */
#define IMX9_IRQ_DMA5_2_60_61    (IMX9_IRQ_EXT + 158) /* eDMA2 channel 60/61 interrupt */
#define IMX9_IRQ_DMA5_2_62_63    (IMX9_IRQ_EXT + 159) /* eDMA2 channel 62/63 interrupt */
#define IMX9_IRQ_RESERVED176     (IMX9_IRQ_EXT + 160) /* Sentinel Group 1 reset source if no s500 reference clock is detected. Output synchronized to 32khz clk. */
#define IMX9_IRQ_RESERVED177     (IMX9_IRQ_EXT + 161) /* Sentinel Group 2 reset source s500 reference clock is not detected or too slow. Output synchronized to ref1_clk. */
#define IMX9_IRQ_RESERVED178     (IMX9_IRQ_EXT + 162) /* Sentinel Group 2 reset source s500 reference clock is not detected or too slow. Output synchronized to ref1_clk. */
#define IMX9_IRQ_RESERVED179     (IMX9_IRQ_EXT + 163) /* JTAGSW DAP MDM-AP SRC reset source */
#define IMX9_IRQ_RESERVED180     (IMX9_IRQ_EXT + 164) /* JTAGC SRC reset source */
#define IMX9_IRQ_RESERVED181     (IMX9_IRQ_EXT + 165) /* CM33 SYSREQRST SRC reset source */
#define IMX9_IRQ_RESERVED182     (IMX9_IRQ_EXT + 166) /* CM33 LOCKUP SRC reset source */
#define IMX9_IRQ_RESERVED183     (IMX9_IRQ_EXT + 167) /* CM7 SYSREQRST SRC reset source */
#define IMX9_IRQ_RESERVED184     (IMX9_IRQ_EXT + 168) /* CM7 LOCKUP SRC reset source */
#define IMX9_IRQ_SAI2            (IMX9_IRQ_EXT + 169) /* Serial Audio Interface 2 */
#define IMX9_IRQ_SAI3            (IMX9_IRQ_EXT + 170) /* Serial Audio Interface 3 */
#define IMX9_IRQ_SAI4            (IMX9_IRQ_EXT + 171) /* Serial Audio Interface 4 */
#define IMX9_IRQ_SAI5            (IMX9_IRQ_EXT + 172) /* Serial Audio Interface 5 */
#define IMX9_IRQ_RESERVED189     (IMX9_IRQ_EXT + 173) /* USB-1 Wake-up Interrupt */
#define IMX9_IRQ_RESERVED190     (IMX9_IRQ_EXT + 174) /* USB-2 Wake-up Interrupt */
#define IMX9_IRQ_USB1            (IMX9_IRQ_EXT + 175) /* USB-1 Interrupt */
#define IMX9_IRQ_USB2            (IMX9_IRQ_EXT + 176) /* USB-2 Interrupt */
#define IMX9_IRQ_LPSPI5          (IMX9_IRQ_EXT + 177) /* Low Power Serial Peripheral Interface 5 */
#define IMX9_IRQ_LPSPI6          (IMX9_IRQ_EXT + 178) /* Low Power Serial Peripheral Interface 6 */
#define IMX9_IRQ_LPSPI7          (IMX9_IRQ_EXT + 179) /* Low Power Serial Peripheral Interface 7 */
#define IMX9_IRQ_LPSPI8          (IMX9_IRQ_EXT + 180) /* Low Power Serial Peripheral Interface 8 */
#define IMX9_IRQ_LPI2C5          (IMX9_IRQ_EXT + 181) /* Low Power Inter-Integrated Circuit module 5 */
#define IMX9_IRQ_LPI2C6          (IMX9_IRQ_EXT + 182) /* Low Power Inter-Integrated Circuit module 6 */
#define IMX9_IRQ_LPI2C7          (IMX9_IRQ_EXT + 183) /* Low Power Inter-Integrated Circuit module 7 */
#define IMX9_IRQ_LPI2C8          (IMX9_IRQ_EXT + 184) /* Low Power Inter-Integrated Circuit module 8 */
#define IMX9_IRQ_PDM_HWVAD_ERROR (IMX9_IRQ_EXT + 185) /* PDM interrupt */
#define IMX9_IRQ_PDM_HWVAD_EVENT (IMX9_IRQ_EXT + 186) /* PDM interrupt */
#define IMX9_IRQ_PDM_ERROR       (IMX9_IRQ_EXT + 187) /* PDM interrupt */
#define IMX9_IRQ_PDM_EVENT       (IMX9_IRQ_EXT + 188) /* PDM interrupt */
#define IMX9_IRQ_RESERVED205     (IMX9_IRQ_EXT + 189) /* AUDIO XCVR interrupt */
#define IMX9_IRQ_RESERVED206     (IMX9_IRQ_EXT + 190) /* AUDIO XCVR interrupt */
#define IMX9_IRQ_uSDHC3          (IMX9_IRQ_EXT + 191) /* ultra Secure Digital Host Controller interrupt 3 */
#define IMX9_IRQ_RESERVED208     (IMX9_IRQ_EXT + 192) /* OCRAM MECC interrupt */
#define IMX9_IRQ_RESERVED209     (IMX9_IRQ_EXT + 193) /* OCRAM MECC interrupt */
#define IMX9_IRQ_RESERVED210     (IMX9_IRQ_EXT + 194) /* CM33 MCM interrupt */
#define IMX9_IRQ_RESERVED211     (IMX9_IRQ_EXT + 195) /* ANAMIX SFA interrupt */
#define IMX9_IRQ_RESERVED212     (IMX9_IRQ_EXT + 196) /* GIC700 Fault */
#define IMX9_IRQ_RESERVED213     (IMX9_IRQ_EXT + 197) /* GIC700 Error */
#define IMX9_IRQ_RESERVED214     (IMX9_IRQ_EXT + 198) /* GIC700 PMU Counter Overflow */
#define IMX9_IRQ_ADC_ER          (IMX9_IRQ_EXT + 199) /* ADC interrupt */
#define IMX9_IRQ_ADC_WD          (IMX9_IRQ_EXT + 200) /* ADC interrupt */
#define IMX9_IRQ_ADC_EOC         (IMX9_IRQ_EXT + 201) /* ADC interrupt */
#define IMX9_IRQ_RESERVED218     (IMX9_IRQ_EXT + 202) /* s500 glue logic IRQ */
#define IMX9_IRQ_RESERVED219     (IMX9_IRQ_EXT + 203) /* I3C1 wakeup irq after double sync */
#define IMX9_IRQ_RESERVED220     (IMX9_IRQ_EXT + 204) /* I3C2 wakeup irq after double sync */
#define IMX9_IRQ_MU5_A           (IMX9_IRQ_EXT + 205) /* AONMIX MU Ored of all (tx,rx,gp,core,murip) interrupt to MUA */
#define IMX9_IRQ_MU6_A           (IMX9_IRQ_EXT + 206) /* AONMIX MU Ored of all (tx,rx,gp,core,murip) interrupt to MUA */
#define IMX9_IRQ_MU7_B           (IMX9_IRQ_EXT + 207) /* WAKEUPMIX MU Ored of all (tx,rx,gp,core,murip) interrupt to MUB */
#define IMX9_IRQ_MU8_B           (IMX9_IRQ_EXT + 208) /* WAKEUPMIX MU Ored of all (tx,rx,gp,core,murip) interrupt to MUB */
#define IMX9_IRQ_RESERVED225     (IMX9_IRQ_EXT + 209) /* WAKEUPMIX XSPI Responder */
#define IMX9_IRQ_RESERVED226     (IMX9_IRQ_EXT + 210) /* AONMIX FCCU Interrupt Reaction 0 */
#define IMX9_IRQ_RESERVED227     (IMX9_IRQ_EXT + 211) /* AONMIX FCCU Interrupt Reaction 1 */
#define IMX9_IRQ_RESERVED228     (IMX9_IRQ_EXT + 212) /* AONMIX FCCU Interrupt Reaction 2 */
#define IMX9_IRQ_RESERVED229     (IMX9_IRQ_EXT + 213) /* AONMIX STCU Selftest end Interrupt */
#define IMX9_IRQ_DISP_IRQSTEER0  (IMX9_IRQ_EXT + 214) /* DISPLAYMIX IRQSTEER 0 */
#define IMX9_IRQ_DISP_IRQSTEER1  (IMX9_IRQ_EXT + 215) /* DISPLAYMIX IRQSTEER 1 */
#define IMX9_IRQ_DISP_IRQSTEER2  (IMX9_IRQ_EXT + 216) /* DISPLAYMIX IRQSTEER 2 */
#define IMX9_IRQ_DISP_IRQSTEER3  (IMX9_IRQ_EXT + 217) /* DISPLAYMIX IRQSTEER 3 */
#define IMX9_IRQ_DISP_IRQSTEER4  (IMX9_IRQ_EXT + 218) /* DISPLAYMIX IRQSTEER 4 */
#define IMX9_IRQ_DISP_IRQSTEER7  (IMX9_IRQ_EXT + 219) /* DISPLAYMIX IRQSTEER 7 */
#define IMX9_IRQ_RESERVED236     (IMX9_IRQ_EXT + 220) /* CAMERAMIX MU Ored of all (tx,rx,gp,core,murip) interrupt to MUA */
#define IMX9_IRQ_ISI             (IMX9_IRQ_EXT + 221) /* CAMERAMIX ISI interrupt Channel 0 */
#define IMX9_IRQ_RESERVED238     (IMX9_IRQ_EXT + 222) /* ISP Processing Interrupt - Context 0 */
#define IMX9_IRQ_RESERVED239     (IMX9_IRQ_EXT + 223) /* M7MIX MCM interrupt */
#define IMX9_IRQ_MU1_A           (IMX9_IRQ_EXT + 224) /* AONMIX MU Ored of all (tx,rx,gp,core,murip) interrupt to MUA */
#define IMX9_IRQ_MU1_B           (IMX9_IRQ_EXT + 225) /* AONMIX MU Ored of all (tx,rx,gp,core,murip) interrupt to MUB */
#define IMX9_IRQ_MU2_A           (IMX9_IRQ_EXT + 226) /* AONMIX MU Ored of all (tx,rx,gp,core,murip) interrupt to MUA */
#define IMX9_IRQ_MU2_B           (IMX9_IRQ_EXT + 227) /* AONMIX MU Ored of all (tx,rx,gp,core,murip) interrupt to MUB */
#define IMX9_IRQ_MU3_A           (IMX9_IRQ_EXT + 228) /* AONMIX MU Ored of all (tx,rx,gp,core,murip) interrupt to MUA */
#define IMX9_IRQ_MU3_B           (IMX9_IRQ_EXT + 229) /* AONMIX MU Ored of all (tx,rx,gp,core,murip) interrupt to MUB */
#define IMX9_IRQ_MU4_A           (IMX9_IRQ_EXT + 230) /* AONMIX MU Ored of all (tx,rx,gp,core,murip) interrupt to MUA */
#define IMX9_IRQ_MU4_B           (IMX9_IRQ_EXT + 231) /* AONMIX MU Ored of all (tx,rx,gp,core,murip) interrupt to MUB */
#define IMX9_IRQ_MU5_B           (IMX9_IRQ_EXT + 232) /* AONMIX MU Ored of all (tx,rx,gp,core,murip) interrupt to MUB */
#define IMX9_IRQ_MU6_B           (IMX9_IRQ_EXT + 233) /* AONMIX MU Ored of all (tx,rx,gp,core,murip) interrupt to MUB */
#define IMX9_IRQ_MU7_A           (IMX9_IRQ_EXT + 234) /* WAKEUPMIX MU Ored of all (tx,rx,gp,core,murip) interrupt to MUA */
#define IMX9_IRQ_MU8_A           (IMX9_IRQ_EXT + 235) /* WAKEUPMIX MU Ored of all (tx,rx,gp,core,murip) interrupt to MUA */
#define IMX9_IRQ_MSGINTR1        (IMX9_IRQ_EXT + 236) /* MSGINTR Instance 1, Interrupt */
#define IMX9_IRQ_MSGINTR2        (IMX9_IRQ_EXT + 237) /* MSGINTR Instance 2, Interrupts */
#define IMX9_IRQ_RESERVED238     (IMX9_IRQ_EXT + 238) /* RESERVED */
#define IMX9_IRQ_RESERVED239     (IMX9_IRQ_EXT + 239) /* RESERVED */
#define IMX9_IRQ_RESERVED266     (IMX9_IRQ_EXT + 240) /* CAMERAMIX TRDC transfer error interrupt */
#define IMX9_IRQ_RESERVED267     (IMX9_IRQ_EXT + 241) /* DISPLAYMIX TRDC transfer error interrupt */
#define IMX9_IRQ_RESERVED268     (IMX9_IRQ_EXT + 242) /* NETCMIX TRDC transfer error interrupt */
#define IMX9_IRQ_RESERVED269     (IMX9_IRQ_EXT + 243) /* GPUMIX TRDC transfer error interrupt */
#define IMX9_IRQ_RESERVED270     (IMX9_IRQ_EXT + 244) /* HSIOMIX TRDC transfer error interrupt */
#define IMX9_IRQ_RESERVED271     (IMX9_IRQ_EXT + 245) /* VPUMIX TRDC transfer error interrupt */
#define IMX9_IRQ_RESERVED272     (IMX9_IRQ_EXT + 246) /* AONMIX ERM Single bit corrected ECC Error */
#define IMX9_IRQ_RESERVED273     (IMX9_IRQ_EXT + 247) /* M7MIX ERM Single bit corrected ECC Error */
#define IMX9_IRQ_RESERVED274     (IMX9_IRQ_EXT + 248) /* WAKEUPMIX ERM Single bit corrected ECC Error */
#define IMX9_IRQ_RESERVED275     (IMX9_IRQ_EXT + 249) /* NPUMIX ERM Single bit corrected ECC Error */
#define IMX9_IRQ_RESERVED276     (IMX9_IRQ_EXT + 250) /* WAKEUPMIX ACP EDMA error interrupt */
#define IMX9_IRQ_RESERVED277     (IMX9_IRQ_EXT + 251) /* OCRAM_C ECC multiple bit or address error */
#define IMX9_IRQ_RESERVED278     (IMX9_IRQ_EXT + 252) /* CAMERAMIX Cortex-M0+ Cache write-buffer error */
#define IMX9_IRQ_RESERVED279     (IMX9_IRQ_EXT + 253) /* CAMERAMIX Cortex-M0+ Cache data parity error */
#define IMX9_IRQ_RESERVED280     (IMX9_IRQ_EXT + 254) /* V2X-FH MU APSHE (SHE) interrupt */
#define IMX9_IRQ_RESERVED281     (IMX9_IRQ_EXT + 255) /* V2X-FH MU SCU/APDEBUG (DEBUG) interrupt */
#define IMX9_IRQ_DMA5_3_0_1      (IMX9_IRQ_EXT + 256) /* eDMA3 channel 0/1 interrupt */
#define IMX9_IRQ_DMA5_3_2_3      (IMX9_IRQ_EXT + 257) /* eDMA3 channel 2/3 interrupt */
#define IMX9_IRQ_DMA5_3_4_5      (IMX9_IRQ_EXT + 258) /* eDMA3 channel 4/5 interrupt */
#define IMX9_IRQ_DMA5_3_6_7      (IMX9_IRQ_EXT + 259) /* eDMA3 channel 6/7 interrupt */
#define IMX9_IRQ_DMA5_3_8_9      (IMX9_IRQ_EXT + 260) /* eDMA3 channel 8/9 interrupt */
#define IMX9_IRQ_DMA5_3_10_11    (IMX9_IRQ_EXT + 261) /* eDMA3 channel 10/11 interrupt */
#define IMX9_IRQ_DMA5_3_12_13    (IMX9_IRQ_EXT + 262) /* eDMA3 channel 12/13 interrupt */
#define IMX9_IRQ_DMA5_3_14_15    (IMX9_IRQ_EXT + 263) /* eDMA3 channel 14/15 interrupt */
#define IMX9_IRQ_DMA5_3_16_17    (IMX9_IRQ_EXT + 264) /* eDMA3 channel 16/17 interrupt */
#define IMX9_IRQ_DMA5_3_18_19    (IMX9_IRQ_EXT + 265) /* eDMA3 channel 18/19 interrupt */
#define IMX9_IRQ_DMA5_3_20_21    (IMX9_IRQ_EXT + 266) /* eDMA3 channel 20/21 interrupt */
#define IMX9_IRQ_DMA5_3_22_23    (IMX9_IRQ_EXT + 267) /* eDMA3 channel 22/23 interrupt */
#define IMX9_IRQ_DMA5_3_24_25    (IMX9_IRQ_EXT + 268) /* eDMA3 channel 24/25 interrupt */
#define IMX9_IRQ_DMA5_3_26_27    (IMX9_IRQ_EXT + 269) /* eDMA3 channel 26/27 interrupt */
#define IMX9_IRQ_DMA5_3_28_29    (IMX9_IRQ_EXT + 270) /* eDMA3 channel 29/29 interrupt */
#define IMX9_IRQ_DMA5_3_30_31    (IMX9_IRQ_EXT + 271) /* eDMA3 channel 30/31 interrupt */
#define IMX9_IRQ_DMA5_3_32_33    (IMX9_IRQ_EXT + 272) /* eDMA3 channel 32/33 interrupt */
#define IMX9_IRQ_DMA5_3_34_35    (IMX9_IRQ_EXT + 273) /* eDMA3 channel 34/35 interrupt */
#define IMX9_IRQ_DMA5_3_36_37    (IMX9_IRQ_EXT + 274) /* eDMA3 channel 36/37 interrupt */
#define IMX9_IRQ_DMA5_3_38_39    (IMX9_IRQ_EXT + 275) /* eDMA3 channel 38/39 interrupt */
#define IMX9_IRQ_DMA5_3_40_41    (IMX9_IRQ_EXT + 276) /* eDMA3 channel 40/41 interrupt */
#define IMX9_IRQ_DMA5_3_42_43    (IMX9_IRQ_EXT + 277) /* eDMA3 channel 42/43 interrupt */
#define IMX9_IRQ_DMA5_3_44_45    (IMX9_IRQ_EXT + 278) /* eDMA3 channel 44/45 interrupt */
#define IMX9_IRQ_DMA5_3_46_47    (IMX9_IRQ_EXT + 279) /* eDMA3 channel 46/47 interrupt */
#define IMX9_IRQ_DMA5_3_48_49    (IMX9_IRQ_EXT + 280) /* eDMA3 channel 48/49 interrupt */
#define IMX9_IRQ_DMA5_3_50_51    (IMX9_IRQ_EXT + 281) /* eDMA3 channel 50/51 interrupt */
#define IMX9_IRQ_DMA5_3_52_53    (IMX9_IRQ_EXT + 282) /* eDMA3 channel 52/53 interrupt */
#define IMX9_IRQ_DMA5_3_54_55    (IMX9_IRQ_EXT + 283) /* eDMA3 channel 54/55 interrupt */
#define IMX9_IRQ_DMA5_3_56_57    (IMX9_IRQ_EXT + 284) /* eDMA3 channel 56/57 interrupt */
#define IMX9_IRQ_DMA5_3_58_59    (IMX9_IRQ_EXT + 285) /* eDMA3 channel 58/59 interrupt */
#define IMX9_IRQ_DMA5_3_60_61    (IMX9_IRQ_EXT + 286) /* eDMA3 channel 60/61 interrupt */
#define IMX9_IRQ_DMA5_3_62_63    (IMX9_IRQ_EXT + 287) /* eDMA3 channel 62/63 interrupt */
#define IMX9_IRQ_RESERVED314     (IMX9_IRQ_EXT + 288) /* GPUMIX GPU Interrupt */
#define IMX9_IRQ_RESERVED315     (IMX9_IRQ_EXT + 289) /* GPUMIX Job Interrupt */
#define IMX9_IRQ_RESERVED316     (IMX9_IRQ_EXT + 290) /* GPUMIX MMU Interrupt */
#define IMX9_IRQ_RESERVED317     (IMX9_IRQ_EXT + 291) /* Reserved INTERRUPT */
#define IMX9_IRQ_RESERVED318     (IMX9_IRQ_EXT + 292) /* Reserved interrupt */
#define IMX9_IRQ_RESERVED319     (IMX9_IRQ_EXT + 293) /* Reserved interrupt */
#define IMX9_IRQ_RESERVED320     (IMX9_IRQ_EXT + 294) /* Reserved interrupt */
#define IMX9_IRQ_RESERVED321     (IMX9_IRQ_EXT + 295) /* Reserved interrupt */
#define IMX9_IRQ_RESERVED322     (IMX9_IRQ_EXT + 296) /* Reserved interrupt */
#define IMX9_IRQ_RESERVED323     (IMX9_IRQ_EXT + 297) /* Reserved interrupt */
#define IMX9_IRQ_RESERVED324     (IMX9_IRQ_EXT + 298) /* Reserved interrupt */
#define IMX9_IRQ_RESERVED325     (IMX9_IRQ_EXT + 299) /* Reserved interrupt */
#define IMX9_IRQ_RESERVED326     (IMX9_IRQ_EXT + 300) /* Reserved interrupt */
#define IMX9_IRQ_RESERVED327     (IMX9_IRQ_EXT + 301) /* Reserved interrupt */
#define IMX9_IRQ_RESERVED328     (IMX9_IRQ_EXT + 302) /* Reserved interrupt */
#define IMX9_IRQ_RESERVED329     (IMX9_IRQ_EXT + 303) /* Reserved interrupt */
#define IMX9_IRQ_RESERVED330     (IMX9_IRQ_EXT + 304) /* NETC iEPRC PCI INT */
#define IMX9_IRQ_RESERVED331     (IMX9_IRQ_EXT + 305) /* NETC iEPRC PCI INT */
#define IMX9_IRQ_RESERVED332     (IMX9_IRQ_EXT + 306) /* PCIe Controller 1 INTA */
#define IMX9_IRQ_RESERVED333     (IMX9_IRQ_EXT + 307) /* PCIe Controller 1 INTB */
#define IMX9_IRQ_RESERVED334     (IMX9_IRQ_EXT + 308) /* PCIe Controller 1 INTC */
#define IMX9_IRQ_RESERVED335     (IMX9_IRQ_EXT + 309) /* PCIe Controller 1 INTD */
#define IMX9_IRQ_RESERVED336     (IMX9_IRQ_EXT + 310) /* PCIe interrupts */
#define IMX9_IRQ_RESERVED337     (IMX9_IRQ_EXT + 311) /* PCIe Controller EDMA channel interrupt */
#define IMX9_IRQ_RESERVED338     (IMX9_IRQ_EXT + 312) /* PCIe Controller 1 INTA */
#define IMX9_IRQ_RESERVED339     (IMX9_IRQ_EXT + 313) /* PCIe Controller 1 INTB */
#define IMX9_IRQ_RESERVED340     (IMX9_IRQ_EXT + 314) /* PCIe Controller 1 INTC */
#define IMX9_IRQ_RESERVED341     (IMX9_IRQ_EXT + 315) /* PCIe Controller 1 INTD */
#define IMX9_IRQ_RESERVED342     (IMX9_IRQ_EXT + 316) /* PCIe miscellaneous interrupts */
#define IMX9_IRQ_RESERVED343     (IMX9_IRQ_EXT + 317) /* PCIe Controller EDMA channel interrupt */
#define IMX9_IRQ_RESERVED344     (IMX9_IRQ_EXT + 318) /* Wakeup interrupt from CLKREQ#, WAKEUP#, BEACON_DET */
#define IMX9_IRQ_RESERVED345     (IMX9_IRQ_EXT + 319) /* NPUMIX Functional interrupt */
#define IMX9_IRQ_RESERVED346     (IMX9_IRQ_EXT + 320) /* DISPLAYMIX Real-time traffic TBU: Fault Handling RAS Interrupt for a contained error */
#define IMX9_IRQ_RESERVED347     (IMX9_IRQ_EXT + 321) /* DISPLAYMIX Real-time traffic TBU: Error Handling RAS Interrupt for an uncontained error */
#define IMX9_IRQ_RESERVED348     (IMX9_IRQ_EXT + 322) /* DISPLAYMIX Real-time traffic TBU: Critical Error Interrupt for an uncontainable error */
#define IMX9_IRQ_RESERVED349     (IMX9_IRQ_EXT + 323) /* DISPLAYMIX Real-time traffic TBU: PMU Interrupt */
#define IMX9_IRQ_RESERVED350     (IMX9_IRQ_EXT + 324) /* TCU Event queue, secure interrupt */
#define IMX9_IRQ_RESERVED351     (IMX9_IRQ_EXT + 325) /* TCU Event queue, non-secure interrupt */
#define IMX9_IRQ_RESERVED352     (IMX9_IRQ_EXT + 326) /* TCU SYNC complete, non-secure interrupt */
#define IMX9_IRQ_RESERVED353     (IMX9_IRQ_EXT + 327) /* TCU SYNC complete, secure interrupt */
#define IMX9_IRQ_RESERVED354     (IMX9_IRQ_EXT + 328) /* TCU global non-secure interrupt */
#define IMX9_IRQ_RESERVED355     (IMX9_IRQ_EXT + 329) /* TCU global secure interrupt */
#define IMX9_IRQ_RESERVED356     (IMX9_IRQ_EXT + 330) /* TCU fault handling RAS interrupt for a contained error */
#define IMX9_IRQ_RESERVED357     (IMX9_IRQ_EXT + 331) /* TCU error recovery RAS interrupt for an uncontained error */
#define IMX9_IRQ_RESERVED358     (IMX9_IRQ_EXT + 332) /* TCU critical error interrupt, for an uncontainable uncorrected error */
#define IMX9_IRQ_RESERVED359     (IMX9_IRQ_EXT + 333) /* TCU PMU interrupt */
#define IMX9_IRQ_RESERVED360     (IMX9_IRQ_EXT + 334) /* TCU Page Request Interface */
#define IMX9_IRQ_RESERVED361     (IMX9_IRQ_EXT + 335) /* SRC GPC Low Power Handshake Gasket interrupt request for system management */
#define IMX9_IRQ_RESERVED362     (IMX9_IRQ_EXT + 336) /* CAMERAMIX MU Ored of all */
#define IMX9_IRQ_RESERVED363     (IMX9_IRQ_EXT + 337) /* CAMERAMIX MU Ored of all */
#define IMX9_IRQ_RESERVED364     (IMX9_IRQ_EXT + 338) /* CAMERAMIX MU Ored of all */
#define IMX9_IRQ_RESERVED365     (IMX9_IRQ_EXT + 339) /* CAMERAMIX MU Ored of all */
#define IMX9_IRQ_RESERVED366     (IMX9_IRQ_EXT + 340) /* CAMERAMIX MU Ored of all */
#define IMX9_IRQ_RESERVED367     (IMX9_IRQ_EXT + 341) /* CAMERAMIX MU Ored of all */
#define IMX9_IRQ_RESERVED368     (IMX9_IRQ_EXT + 342) /* CAMERAMIX MU Ored of all */
#define IMX9_IRQ_RESERVED369     (IMX9_IRQ_EXT + 343) /* CAMERAMIX MU Ored of all */
#define IMX9_IRQ_RESERVED370     (IMX9_IRQ_EXT + 344) /* CAMERAMIX ISI interrupt Channel 1 */
#define IMX9_IRQ_RESERVED371     (IMX9_IRQ_EXT + 345) /* CAMERAMIX ISI interrupt Channel 2 */
#define IMX9_IRQ_RESERVED372     (IMX9_IRQ_EXT + 346) /* CAMERAMIX ISI interrupt Channel 3 */
#define IMX9_IRQ_RESERVED373     (IMX9_IRQ_EXT + 347) /* CAMERAMIX ISI interrupt Channel 4 */
#define IMX9_IRQ_RESERVED374     (IMX9_IRQ_EXT + 348) /* CAMERAMIX ISI interrupt Channel 5 */
#define IMX9_IRQ_RESERVED375     (IMX9_IRQ_EXT + 349) /* CAMERAMIX ISI interrupt Channel 6 */
#define IMX9_IRQ_RESERVED376     (IMX9_IRQ_EXT + 350) /* CAMERAMIX ISI interrupt Channel 7 */
#define IMX9_IRQ_DMA5_4_ERROR    (IMX9_IRQ_EXT + 351) /* CAMERAMIX EDMA error interrupt */
#define IMX9_IRQ_DMA5_4_0_1      (IMX9_IRQ_EXT + 352) /* CAMERAMIX EDMA channel 0 interrupt */
#define IMX9_IRQ_DMA5_4_2_3      (IMX9_IRQ_EXT + 353) /* CAMERAMIX EDMA channel 2 interrupt */
#define IMX9_IRQ_DMA5_4_4_5      (IMX9_IRQ_EXT + 354) /* CAMERAMIX EDMA channel 4 interrupt */
#define IMX9_IRQ_DMA5_4_6_7      (IMX9_IRQ_EXT + 355) /* CAMERAMIX EDMA channel 6 interrupt */
#define IMX9_IRQ_DMA5_4_8_9      (IMX9_IRQ_EXT + 356) /* CAMERAMIX EDMA channel 8 interrupt */
#define IMX9_IRQ_DMA5_4_10_11    (IMX9_IRQ_EXT + 357) /* CAMERAMIX EDMA channel 10 interrupt */
#define IMX9_IRQ_DMA5_4_12_13    (IMX9_IRQ_EXT + 358) /* CAMERAMIX EDMA channel 12 interrupt */
#define IMX9_IRQ_DMA5_4_14_15    (IMX9_IRQ_EXT + 359) /* CAMERAMIX EDMA channel 14 interrupt */
#define IMX9_IRQ_DMA5_4_16_17    (IMX9_IRQ_EXT + 360) /* CAMERAMIX EDMA channel 16 interrupt */
#define IMX9_IRQ_DMA5_4_18_19    (IMX9_IRQ_EXT + 361) /* CAMERAMIX EDMA channel 18 interrupt */
#define IMX9_IRQ_DMA5_4_20_21    (IMX9_IRQ_EXT + 362) /* CAMERAMIX EDMA channel 20 interrupt */
#define IMX9_IRQ_DMA5_4_22_23    (IMX9_IRQ_EXT + 363) /* CAMERAMIX EDMA channel 22 interrupt */
#define IMX9_IRQ_DMA5_4_24_25    (IMX9_IRQ_EXT + 364) /* CAMERAMIX EDMA channel 24 interrupt */
#define IMX9_IRQ_DMA5_4_26_27    (IMX9_IRQ_EXT + 365) /* CAMERAMIX EDMA channel 26 interrupt */
#define IMX9_IRQ_DMA5_4_28_29    (IMX9_IRQ_EXT + 366) /* CAMERAMIX EDMA channel 28 interrupt */
#define IMX9_IRQ_DMA5_4_30_31    (IMX9_IRQ_EXT + 367) /* CAMERAMIX EDMA channel 30 interrupt */
#define IMX9_IRQ_RESERVED394     (IMX9_IRQ_EXT + 368) /* CAMERAMIX CSI Formatting Unit 1: Buffer overflow */
#define IMX9_IRQ_RESERVED395     (IMX9_IRQ_EXT + 369) /* CAMERAMIX CSI Formatting Unit 1: Interlaced Error */
#define IMX9_IRQ_RESERVED396     (IMX9_IRQ_EXT + 370) /* CAMERAMIX CSI Formatting Unit 1: Pixel Data Type Error */
#define IMX9_IRQ_RESERVED397     (IMX9_IRQ_EXT + 371) /* CAMERAMIX CSI Formatting Unit 2: Buffer overflow */
#define IMX9_IRQ_RESERVED398     (IMX9_IRQ_EXT + 372) /* CAMERAMIX CSI Formatting Unit 2: Interlaced Error */
#define IMX9_IRQ_RESERVED399     (IMX9_IRQ_EXT + 373) /* CAMERAMIX CSI Formatting Unit 2: Pixel Data Type Error */
#define IMX9_IRQ_RESERVED400     (IMX9_IRQ_EXT + 374) /* CAMERAMIX CSI1 */
#define IMX9_IRQ_RESERVED401     (IMX9_IRQ_EXT + 375) /* CAMERAMIX CSI2 */

/* Total amount of entries in system vector table */

#define NR_IRQS                  (IMX9_IRQ_EXT + 376)

/* Cores are at 100h offset from each other (affinity 1) */

#define MPID_TO_CORE(mpid)              (((mpid) >> MPIDR_AFF1_SHIFT) & MPIDR_AFFLVL_MASK)

#endif /* __ARCH_ARM64_INCLUDE_IMX9_IMX95_IRQ_H */
