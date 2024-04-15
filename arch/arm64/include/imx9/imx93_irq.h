/****************************************************************************
 * arch/arm64/include/imx9/imx93_irq.h
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

#ifndef __ARCH_ARM64_INCLUDE_IMX9_IMX93_IRQ_H
#define __ARCH_ARM64_INCLUDE_IMX9_IMX93_IRQ_H

#define IMX9_IRQ_RESERVED32             (IMX9_IRQ_EXT + 0)     /* Exception condition notification while boot */
#define IMX9_IRQ_RESERVED33             (IMX9_IRQ_EXT + 1)     /* DAP interrupt */
#define IMX9_IRQ_RESERVED34             (IMX9_IRQ_EXT + 2)     /* Reserved interrupt */
#define IMX9_IRQ_RESERVED35             (IMX9_IRQ_EXT + 3)     /* CTI trigger outputs from CM33 platform */
#define IMX9_IRQ_RESERVED36             (IMX9_IRQ_EXT + 4)     /* CTI trigger outputs from CA55 platform */
#define IMX9_IRQ_RESERVED37             (IMX9_IRQ_EXT + 5)     /* Performance Unit Interrupts from CA55 platform */
#define IMX9_IRQ_RESERVED38             (IMX9_IRQ_EXT + 6)     /* ECC error from CA55 platform cache */
#define IMX9_IRQ_RESERVED39             (IMX9_IRQ_EXT + 7)     /* 1-bit or 2-bit ECC or Parity error from CA55 platform cache */
#define IMX9_IRQ_CAN1                   (IMX9_IRQ_EXT + 8)     /* CAN1 interrupt */
#define IMX9_IRQ_CAN1_ERROR             (IMX9_IRQ_EXT + 9)     /* CAN1 error interrupt */
#define IMX9_IRQ_GPIO1_0                (IMX9_IRQ_EXT + 10)    /* General Purpose Input/Output 1 interrupt 0 */
#define IMX9_IRQ_GPIO1_1                (IMX9_IRQ_EXT + 11)    /* General Purpose Input/Output 1 interrupt 1 */
#define IMX9_IRQ_I3C1                   (IMX9_IRQ_EXT + 12)    /* Improved Inter-Integrated Circuit 1 interrupt */
#define IMX9_IRQ_LPI2C1                 (IMX9_IRQ_EXT + 13)    /* Low Power Inter-Integrated Circuit module 1 */
#define IMX9_IRQ_LPI2C2                 (IMX9_IRQ_EXT + 14)    /* Low Power Inter-Integrated Circuit module 2 */
#define IMX9_IRQ_LPIT1                  (IMX9_IRQ_EXT + 15)    /* Low Power Periodic Interrupt Timer 1 */
#define IMX9_IRQ_LPSPI1                 (IMX9_IRQ_EXT + 16)    /* Low Power Serial Peripheral Interface 1 */
#define IMX9_IRQ_LPSPI2                 (IMX9_IRQ_EXT + 17)    /* Low Power Serial Peripheral Interface 2 */
#define IMX9_IRQ_LPTMR1                 (IMX9_IRQ_EXT + 18)    /* Low Power Timer 1 */
#define IMX9_IRQ_LPUART1                (IMX9_IRQ_EXT + 19)    /* Low Power UART 1 */
#define IMX9_IRQ_LPUART2                (IMX9_IRQ_EXT + 20)    /* Low Power UART 2 */
#define IMX9_IRQ_MU1_A                  (IMX9_IRQ_EXT + 21)    /* Messaging Unit 1 - Side A (to communicate with M7 core) */
#define IMX9_IRQ_MU1_B                  (IMX9_IRQ_EXT + 22)    /* Messaging Unit 1 - Side B (to communicate with M33 core) */
#define IMX9_IRQ_MU2_A                  (IMX9_IRQ_EXT + 23)    /* Messaging Unit 2 - Side A (to communicate with M7 core) */
#define IMX9_IRQ_MU2_B                  (IMX9_IRQ_EXT + 24)    /* Messaging Unit 2 - Side B (to communicate with A55 core) */
#define IMX9_IRQ_RESERVED57             (IMX9_IRQ_EXT + 25)    /* Reserved interrupt */
#define IMX9_IRQ_RESERVED58             (IMX9_IRQ_EXT + 26)    /* Reserved interrupt */
#define IMX9_IRQ_RESERVED59             (IMX9_IRQ_EXT + 27)    /* Reserved interrupt */
#define IMX9_IRQ_RESERVED60             (IMX9_IRQ_EXT + 28)    /* Edgelock Trust MUA RX full interrupt */
#define IMX9_IRQ_RESERVED61             (IMX9_IRQ_EXT + 29)    /* Edgelock Trust MUA TX empty interrupt */
#define IMX9_IRQ_RESERVED62             (IMX9_IRQ_EXT + 30)    /* Edgelock Apps Core MUA RX full interrupt */
#define IMX9_IRQ_RESERVED63             (IMX9_IRQ_EXT + 31)    /* Edgelock Apps Core MUA TX empty interrupt */
#define IMX9_IRQ_RESERVED64             (IMX9_IRQ_EXT + 32)    /* Edgelock Realtime Core MUA RX full interrupt */
#define IMX9_IRQ_RESERVED65             (IMX9_IRQ_EXT + 33)    /* Edgelock Realtime Core MUA TX empty interrupt */
#define IMX9_IRQ_RESERVED66             (IMX9_IRQ_EXT + 34)    /* Edgelock secure interrupt */
#define IMX9_IRQ_RESERVED67             (IMX9_IRQ_EXT + 35)    /* Edgelock non-secure interrupt */
#define IMX9_IRQ_TPM1                   (IMX9_IRQ_EXT + 36)    /* Timer PWM module 1 */
#define IMX9_IRQ_TPM2                   (IMX9_IRQ_EXT + 37)    /* Timer PWM module 2 */
#define IMX9_IRQ_WDOG1                  (IMX9_IRQ_EXT + 38)    /* Watchdog 1 Interrupt */
#define IMX9_IRQ_WDOG2                  (IMX9_IRQ_EXT + 39)    /* Watchdog 2 Interrupt */
#define IMX9_IRQ_TRDC                   (IMX9_IRQ_EXT + 40)    /* AONMIX TRDC transfer error interrupt */
#define IMX9_IRQ_RESERVED73             (IMX9_IRQ_EXT + 41)    /* Reserved interrupt */
#define IMX9_IRQ_RESERVED74             (IMX9_IRQ_EXT + 42)    /* Reserved interrupt */
#define IMX9_IRQ_RESERVED75             (IMX9_IRQ_EXT + 43)    /* Reserved interrupt */
#define IMX9_IRQ_RESERVED76             (IMX9_IRQ_EXT + 44)    /* Reserved interrupt */
#define IMX9_IRQ_SAI1                   (IMX9_IRQ_EXT + 45)    /* Serial Audio Interface 1 */
#define IMX9_IRQ_RESERVED78             (IMX9_IRQ_EXT + 46)    /* M33 PS Tag/Data Parity Error */
#define IMX9_IRQ_RESERVED79             (IMX9_IRQ_EXT + 47)    /* M33 TCM ECC interrupt */
#define IMX9_IRQ_RESERVED80             (IMX9_IRQ_EXT + 48)    /* M33 TCM Error interrupt */
#define IMX9_IRQ_RESERVED81             (IMX9_IRQ_EXT + 49)    /* Reserved interrupt */
#define IMX9_IRQ_RESERVED82             (IMX9_IRQ_EXT + 50)    /* Reserved interrupt */
#define IMX9_IRQ_CAN2                   (IMX9_IRQ_EXT + 51)    /* CAN2 interrupt */
#define IMX9_IRQ_CAN2_ERROR             (IMX9_IRQ_EXT + 52)    /* CAN2 error interrupt */
#define IMX9_IRQ_FLEXIO1                (IMX9_IRQ_EXT + 53)    /* Flexible IO 1 interrupt */
#define IMX9_IRQ_FLEXIO2                (IMX9_IRQ_EXT + 54)    /* Flexible IO 2 interrupt */
#define IMX9_IRQ_FLEXSPI1               (IMX9_IRQ_EXT + 55)    /* FlexSPI controller interface interrupt 1 */
#define IMX9_IRQ_RESERVED88             (IMX9_IRQ_EXT + 56)    /* Reserved interrupt */
#define IMX9_IRQ_GPIO2_0                (IMX9_IRQ_EXT + 57)    /* General Purpose Input/Output 2 interrupt 0 */
#define IMX9_IRQ_GPIO2_1                (IMX9_IRQ_EXT + 58)    /* General Purpose Input/Output 2 interrupt 1 */
#define IMX9_IRQ_GPIO3_0                (IMX9_IRQ_EXT + 59)    /* General Purpose Input/Output 3 interrupt 0 */
#define IMX9_IRQ_GPIO3_1                (IMX9_IRQ_EXT + 60)    /* General Purpose Input/Output 3 interrupt 1 */
#define IMX9_IRQ_I3C2                   (IMX9_IRQ_EXT + 61)    /* Improved Inter-Integrated Circuit 2 interrupt */
#define IMX9_IRQ_LPI2C3                 (IMX9_IRQ_EXT + 62)    /* Low Power Inter-Integrated Circuit module 3 */
#define IMX9_IRQ_LPI2C4                 (IMX9_IRQ_EXT + 63)    /* Low Power Inter-Integrated Circuit module 4 */
#define IMX9_IRQ_LPIT2                  (IMX9_IRQ_EXT + 64)    /* Low Power Periodic Interrupt Timer 2 */
#define IMX9_IRQ_LPSPI3                 (IMX9_IRQ_EXT + 65)    /* Low Power Serial Peripheral Interface 3 */
#define IMX9_IRQ_LPSPI4                 (IMX9_IRQ_EXT + 66)    /* Low Power Serial Peripheral Interface 4 */
#define IMX9_IRQ_LPTMR2                 (IMX9_IRQ_EXT + 67)    /* Low Power Timer 2 */
#define IMX9_IRQ_LPUART3                (IMX9_IRQ_EXT + 68)    /* Low Power UART 3 */
#define IMX9_IRQ_LPUART4                (IMX9_IRQ_EXT + 69)    /* Low Power UART 4 */
#define IMX9_IRQ_LPUART5                (IMX9_IRQ_EXT + 70)    /* Low Power UART 5 */
#define IMX9_IRQ_LPUART6                (IMX9_IRQ_EXT + 71)    /* Low Power UART 6 */
#define IMX9_IRQ_RESERVED104            (IMX9_IRQ_EXT + 72)    /* MTR Master error interrupt */
#define IMX9_IRQ_RESERVED105            (IMX9_IRQ_EXT + 73)    /* BBNSM Non-Secure interrupt */
#define IMX9_IRQ_RESERVED106            (IMX9_IRQ_EXT + 74)    /* System Counter compare interrupt */
#define IMX9_IRQ_TPM3                   (IMX9_IRQ_EXT + 75)    /* Timer PWM module 3 */
#define IMX9_IRQ_TPM4                   (IMX9_IRQ_EXT + 76)    /* Timer PWM module 4 */
#define IMX9_IRQ_TPM5                   (IMX9_IRQ_EXT + 77)    /* Timer PWM module 5 */
#define IMX9_IRQ_TPM6                   (IMX9_IRQ_EXT + 78)    /* Timer PWM module 6 */
#define IMX9_IRQ_WDOG3                  (IMX9_IRQ_EXT + 79)    /* Watchdog 3 Interrupt */
#define IMX9_IRQ_WDOG4                  (IMX9_IRQ_EXT + 80)    /* Watchdog 4 Interrupt */
#define IMX9_IRQ_WDOG5                  (IMX9_IRQ_EXT + 81)    /* Watchdog 5 Interrupt */
#define IMX9_IRQ_RESERVED114            (IMX9_IRQ_EXT + 82)    /* WAKEUPMIX TRDC transfer error interrupt */
#define IMX9_IRQ_TEMPMON                (IMX9_IRQ_EXT + 83)    /* TempSensor interrupt */
#define IMX9_IRQ_RESERVED116            (IMX9_IRQ_EXT + 84)    /* Reserved interrupt */
#define IMX9_IRQ_RESERVED117            (IMX9_IRQ_EXT + 85)    /* Reserved interrupt */
#define IMX9_IRQ_USDHC1                 (IMX9_IRQ_EXT + 86)    /* ultra Secure Digital Host Controller interrupt 1 */
#define IMX9_IRQ_USDHC2                 (IMX9_IRQ_EXT + 87)    /* ultra Secure Digital Host Controller interrupt 2 */
#define IMX9_IRQ_RESERVED120            (IMX9_IRQ_EXT + 88)    /* MEGAMIX TRDC transfer error interrupt */
#define IMX9_IRQ_RESERVED121            (IMX9_IRQ_EXT + 89)    /* NIC_WRAPPER TRDC transfer error interrupt */
#define IMX9_IRQ_RESERVED122            (IMX9_IRQ_EXT + 90)    /* DRAM controller Performance Monitor Interrupt */
#define IMX9_IRQ_RESERVED123            (IMX9_IRQ_EXT + 91)    /* DRAM controller Critical Interrupt */
#define IMX9_IRQ_RESERVED124            (IMX9_IRQ_EXT + 92)    /* DRAM Phy Critical Interrupt */
#define IMX9_IRQ_RESERVED125            (IMX9_IRQ_EXT + 93)    /* Reserved interrupt */
#define IMX9_IRQ_DMA3_ERROR             (IMX9_IRQ_EXT + 94)    /* eDMA1 error interrupt */
#define IMX9_IRQ_DMA3_0                 (IMX9_IRQ_EXT + 95)    /* eDMA1 channel 0 interrupt */
#define IMX9_IRQ_DMA3_1                 (IMX9_IRQ_EXT + 96)    /* eDMA1 channel 1 interrupt */
#define IMX9_IRQ_DMA3_2                 (IMX9_IRQ_EXT + 97)    /* eDMA1 channel 2 interrupt */
#define IMX9_IRQ_DMA3_3                 (IMX9_IRQ_EXT + 98)    /* eDMA1 channel 3 interrupt */
#define IMX9_IRQ_DMA3_4                 (IMX9_IRQ_EXT + 99)    /* eDMA1 channel 4 interrupt */
#define IMX9_IRQ_DMA3_5                 (IMX9_IRQ_EXT + 100)   /* eDMA1 channel 5 interrupt */
#define IMX9_IRQ_DMA3_6                 (IMX9_IRQ_EXT + 101)   /* eDMA1 channel 6 interrupt */
#define IMX9_IRQ_DMA3_7                 (IMX9_IRQ_EXT + 102)   /* eDMA1 channel 7 interrupt */
#define IMX9_IRQ_DMA3_8                 (IMX9_IRQ_EXT + 103)   /* eDMA1 channel 8 interrupt */
#define IMX9_IRQ_DMA3_9                 (IMX9_IRQ_EXT + 104)   /* eDMA1 channel 9 interrupt */
#define IMX9_IRQ_DMA3_10                (IMX9_IRQ_EXT + 105)   /* eDMA1 channel 10 interrupt */
#define IMX9_IRQ_DMA3_11                (IMX9_IRQ_EXT + 106)   /* eDMA1 channel 11 interrupt */
#define IMX9_IRQ_DMA3_12                (IMX9_IRQ_EXT + 107)   /* eDMA1 channel 12 interrupt */
#define IMX9_IRQ_DMA3_13                (IMX9_IRQ_EXT + 108)   /* eDMA1 channel 13 interrupt */
#define IMX9_IRQ_DMA3_14                (IMX9_IRQ_EXT + 109)   /* eDMA1 channel 14 interrupt */
#define IMX9_IRQ_DMA3_15                (IMX9_IRQ_EXT + 110)   /* eDMA1 channel 15 interrupt */
#define IMX9_IRQ_DMA3_16                (IMX9_IRQ_EXT + 111)   /* eDMA1 channel 16 interrupt */
#define IMX9_IRQ_DMA3_17                (IMX9_IRQ_EXT + 112)   /* eDMA1 channel 17 interrupt */
#define IMX9_IRQ_DMA3_18                (IMX9_IRQ_EXT + 113)   /* eDMA1 channel 18 interrupt */
#define IMX9_IRQ_DMA3_19                (IMX9_IRQ_EXT + 114)   /* eDMA1 channel 19 interrupt */
#define IMX9_IRQ_DMA3_20                (IMX9_IRQ_EXT + 115)   /* eDMA1 channel 20 interrupt */
#define IMX9_IRQ_DMA3_21                (IMX9_IRQ_EXT + 116)   /* eDMA1 channel 21 interrupt */
#define IMX9_IRQ_DMA3_22                (IMX9_IRQ_EXT + 117)   /* eDMA1 channel 22 interrupt */
#define IMX9_IRQ_DMA3_23                (IMX9_IRQ_EXT + 118)   /* eDMA1 channel 23 interrupt */
#define IMX9_IRQ_DMA3_24                (IMX9_IRQ_EXT + 119)   /* eDMA1 channel 24 interrupt */
#define IMX9_IRQ_DMA3_25                (IMX9_IRQ_EXT + 120)   /* eDMA1 channel 25 interrupt */
#define IMX9_IRQ_DMA3_26                (IMX9_IRQ_EXT + 121)   /* eDMA1 channel 26 interrupt */
#define IMX9_IRQ_DMA3_27                (IMX9_IRQ_EXT + 122)   /* eDMA1 channel 27 interrupt */
#define IMX9_IRQ_DMA3_28                (IMX9_IRQ_EXT + 123)   /* eDMA1 channel 28 interrupt */
#define IMX9_IRQ_DMA3_29                (IMX9_IRQ_EXT + 124)   /* eDMA1 channel 29 interrupt */
#define IMX9_IRQ_DMA3_30                (IMX9_IRQ_EXT + 125)   /* eDMA1 channel 30 interrupt */
#define IMX9_IRQ_RESERVED158            (IMX9_IRQ_EXT + 126)   /* Reserved interrupt */
#define IMX9_IRQ_DMA4_ERROR             (IMX9_IRQ_EXT + 127)   /* eDMA2 error interrupt */
#define IMX9_IRQ_DMA4_0_1               (IMX9_IRQ_EXT + 128)   /* eDMA2 channel 0/1 interrupt */
#define IMX9_IRQ_DMA4_2_3               (IMX9_IRQ_EXT + 129)   /* eDMA2 channel 2/3 interrupt */
#define IMX9_IRQ_DMA4_4_5               (IMX9_IRQ_EXT + 130)   /* eDMA2 channel 4/5 interrupt */
#define IMX9_IRQ_DMA4_6_7               (IMX9_IRQ_EXT + 131)   /* eDMA2 channel 6/7 interrupt */
#define IMX9_IRQ_DMA4_8_9               (IMX9_IRQ_EXT + 132)   /* eDMA2 channel 8/9 interrupt */
#define IMX9_IRQ_DMA4_10_11             (IMX9_IRQ_EXT + 133)   /* eDMA2 channel 10/11 interrupt */
#define IMX9_IRQ_DMA4_12_13             (IMX9_IRQ_EXT + 134)   /* eDMA2 channel 12/13 interrupt */
#define IMX9_IRQ_DMA4_14_15             (IMX9_IRQ_EXT + 135)   /* eDMA2 channel 14/15 interrupt */
#define IMX9_IRQ_DMA4_16_17             (IMX9_IRQ_EXT + 136)   /* eDMA2 channel 16/17 interrupt */
#define IMX9_IRQ_DMA4_18_19             (IMX9_IRQ_EXT + 137)   /* eDMA2 channel 18/19 interrupt */
#define IMX9_IRQ_DMA4_20_21             (IMX9_IRQ_EXT + 138)   /* eDMA2 channel 20/21 interrupt */
#define IMX9_IRQ_DMA4_22_23             (IMX9_IRQ_EXT + 139)   /* eDMA2 channel 22/23 interrupt */
#define IMX9_IRQ_DMA4_24_25             (IMX9_IRQ_EXT + 140)   /* eDMA2 channel 24/25 interrupt */
#define IMX9_IRQ_DMA4_26_27             (IMX9_IRQ_EXT + 141)   /* eDMA2 channel 26/27 interrupt */
#define IMX9_IRQ_DMA4_28_29             (IMX9_IRQ_EXT + 142)   /* eDMA2 channel 28/29 interrupt */
#define IMX9_IRQ_DMA4_30_31             (IMX9_IRQ_EXT + 143)   /* eDMA2 channel 30/31 interrupt */
#define IMX9_IRQ_DMA4_32_33             (IMX9_IRQ_EXT + 144)   /* eDMA2 channel 32/33 interrupt */
#define IMX9_IRQ_DMA4_34_35             (IMX9_IRQ_EXT + 145)   /* eDMA2 channel 34/35 interrupt */
#define IMX9_IRQ_DMA4_36_37             (IMX9_IRQ_EXT + 146)   /* eDMA2 channel 36/37 interrupt */
#define IMX9_IRQ_DMA4_38_39             (IMX9_IRQ_EXT + 147)   /* eDMA2 channel 38/39 interrupt */
#define IMX9_IRQ_DMA4_40_41             (IMX9_IRQ_EXT + 148)   /* eDMA2 channel 40/41 interrupt */
#define IMX9_IRQ_DMA4_42_43             (IMX9_IRQ_EXT + 149)   /* eDMA2 channel 42/43 interrupt */
#define IMX9_IRQ_DMA4_44_45             (IMX9_IRQ_EXT + 150)   /* eDMA2 channel 44/45 interrupt */
#define IMX9_IRQ_DMA4_46_47             (IMX9_IRQ_EXT + 151)   /* eDMA2 channel 46/47 interrupt */
#define IMX9_IRQ_DMA4_48_49             (IMX9_IRQ_EXT + 152)   /* eDMA2 channel 48/49 interrupt */
#define IMX9_IRQ_DMA4_50_51             (IMX9_IRQ_EXT + 153)   /* eDMA2 channel 50/51 interrupt */
#define IMX9_IRQ_DMA4_52_53             (IMX9_IRQ_EXT + 154)   /* eDMA2 channel 52/53 interrupt */
#define IMX9_IRQ_DMA4_54_55             (IMX9_IRQ_EXT + 155)   /* eDMA2 channel 54/55 interrupt */
#define IMX9_IRQ_DMA4_56_57             (IMX9_IRQ_EXT + 156)   /* eDMA2 channel 56/57 interrupt */
#define IMX9_IRQ_DMA4_58_59             (IMX9_IRQ_EXT + 157)   /* eDMA2 channel 58/59 interrupt */
#define IMX9_IRQ_DMA4_60_61             (IMX9_IRQ_EXT + 158)   /* eDMA2 channel 60/61 interrupt */
#define IMX9_IRQ_DMA4_62_63             (IMX9_IRQ_EXT + 159)   /* eDMA2 channel 62/63 interrupt */
#define IMX9_IRQ_RESERVED192            (IMX9_IRQ_EXT + 160)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED193            (IMX9_IRQ_EXT + 161)   /* Edgelock Group 1 reset source */
#define IMX9_IRQ_RESERVED194            (IMX9_IRQ_EXT + 162)   /* Edgelock Group 2 reset source */
#define IMX9_IRQ_RESERVED195            (IMX9_IRQ_EXT + 163)   /* Edgelock Group 2 reset source */
#define IMX9_IRQ_RESERVED196            (IMX9_IRQ_EXT + 164)   /* JTAGSW DAP MDM-AP SRC reset source */
#define IMX9_IRQ_RESERVED197            (IMX9_IRQ_EXT + 165)   /* JTAGC SRC reset source */
#define IMX9_IRQ_RESERVED198            (IMX9_IRQ_EXT + 166)   /* CM33 SYSREQRST SRC reset source */
#define IMX9_IRQ_RESERVED199            (IMX9_IRQ_EXT + 167)   /* CM33 LOCKUP SRC reset source */
#define IMX9_IRQ_RESERVED200            (IMX9_IRQ_EXT + 168)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED201            (IMX9_IRQ_EXT + 169)   /* Reserved interrupt */
#define IMX9_IRQ_SAI2                   (IMX9_IRQ_EXT + 170)   /* Serial Audio Interface 2 */
#define IMX9_IRQ_SAI3                   (IMX9_IRQ_EXT + 171)   /* Serial Audio Interface 3 */
#define IMX9_IRQ_ISI                    (IMX9_IRQ_EXT + 172)   /* ISI interrupt */
#define IMX9_IRQ_RESERVED205            (IMX9_IRQ_EXT + 173)   /* PXP interrupt 0 */
#define IMX9_IRQ_RESERVED206            (IMX9_IRQ_EXT + 174)   /* PXP interrupt 1 */
#define IMX9_IRQ_CSI                    (IMX9_IRQ_EXT + 175)   /* CSI interrupt */
#define IMX9_IRQ_RESERVED208            (IMX9_IRQ_EXT + 176)   /* LCDIF Sync Interrupt */
#define IMX9_IRQ_DSI                    (IMX9_IRQ_EXT + 177)   /* MIPI DSI Interrupt Request */
#define IMX9_IRQ_RESERVED210            (IMX9_IRQ_EXT + 178)   /* Machine learning processor interrupt */
#define IMX9_IRQ_ENET_MAC0_RX_TX_D ONE1 (IMX9_IRQ_EXT + 179)   /* MAC 0 Receive/ Trasmit Frame/ Buffer Done */
#define IMX9_IRQ_ENET_MAC0_RX_TX_D ONE2 (IMX9_IRQ_EXT + 180)   /* MAC 0 Receive/ Trasmit Frame/ Buffer Done */
#define IMX9_IRQ_ENET                   (IMX9_IRQ_EXT + 181)   /* MAC 0 IRQ */
#define IMX9_IRQ_ENET_1588              (IMX9_IRQ_EXT + 182)   /* MAC 0 1588 Timer Interrupt - synchronous */
#define IMX9_IRQ_ENET_QOS_PMT           (IMX9_IRQ_EXT + 183)   /* ENET QOS PMT interrupt */
#define IMX9_IRQ_ENET_QOS               (IMX9_IRQ_EXT + 184)   /* ENET QOS interrupt */
#define IMX9_IRQ_RESERVED217            (IMX9_IRQ_EXT + 185)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED218            (IMX9_IRQ_EXT + 186)   /* Reserved interrupt */
#define IMX9_IRQ_USB1                   (IMX9_IRQ_EXT + 187)   /* USB-1 Wake-up Interrupt */
#define IMX9_IRQ_USB2                   (IMX9_IRQ_EXT + 188)   /* USB-2 Wake-up Interrupt */
#define IMX9_IRQ_GPIO4_0                (IMX9_IRQ_EXT + 189)   /* General Purpose Input/Output 4 interrupt 0 */
#define IMX9_IRQ_GPIO4_1                (IMX9_IRQ_EXT + 190)   /* General Purpose Input/Output 4 interrupt 1 */
#define IMX9_IRQ_LPSPI5                 (IMX9_IRQ_EXT + 191)   /* Low Power Serial Peripheral Interface 5 */
#define IMX9_IRQ_LPSPI6                 (IMX9_IRQ_EXT + 192)   /* Low Power Serial Peripheral Interface 6 */
#define IMX9_IRQ_LPSPI7                 (IMX9_IRQ_EXT + 193)   /* Low Power Serial Peripheral Interface 7 */
#define IMX9_IRQ_LPSPI8                 (IMX9_IRQ_EXT + 194)   /* Low Power Serial Peripheral Interface 8 */
#define IMX9_IRQ_LPI2C5                 (IMX9_IRQ_EXT + 195)   /* Low Power Inter-Integrated Circuit module 5 */
#define IMX9_IRQ_LPI2C6                 (IMX9_IRQ_EXT + 196)   /* Low Power Inter-Integrated Circuit module 6 */
#define IMX9_IRQ_LPI2C7                 (IMX9_IRQ_EXT + 197)   /* Low Power Inter-Integrated Circuit module 7 */
#define IMX9_IRQ_LPI2C8                 (IMX9_IRQ_EXT + 198)   /* Low Power Inter-Integrated Circuit module 8 */
#define IMX9_IRQ_PDM_HWVAD_ERROR        (IMX9_IRQ_EXT + 199)   /* PDM interrupt */
#define IMX9_IRQ_PDM_HWVAD_EVENT        (IMX9_IRQ_EXT + 200)   /* PDM interrupt */
#define IMX9_IRQ_PDM_ERROR              (IMX9_IRQ_EXT + 201)   /* PDM interrupt */
#define IMX9_IRQ_PDM_EVENT              (IMX9_IRQ_EXT + 202)   /* PDM interrupt */
#define IMX9_IRQ_RESERVED235            (IMX9_IRQ_EXT + 203)   /* AUDIO XCVR interrupt */
#define IMX9_IRQ_RESERVED236            (IMX9_IRQ_EXT + 204)   /* AUDIO XCVR interrupt */
#define IMX9_IRQ_USDHC3                 (IMX9_IRQ_EXT + 205)   /* ultra Secure Digital Host Controller interrupt 3 */
#define IMX9_IRQ_RESERVED238            (IMX9_IRQ_EXT + 206)   /* OCRAM MECC interrupt */
#define IMX9_IRQ_RESERVED239            (IMX9_IRQ_EXT + 207)   /* OCRAM MECC interrupt */
#define IMX9_IRQ_RESERVED240            (IMX9_IRQ_EXT + 208)   /* HSIOMIX TRDC transfer error interrupt */
#define IMX9_IRQ_RESERVED241            (IMX9_IRQ_EXT + 209)   /* MEDIAMIX TRDC transfer error interrupt */
#define IMX9_IRQ_LPUART7                (IMX9_IRQ_EXT + 210)   /* Low Power UART 7 */
#define IMX9_IRQ_LPUART8                (IMX9_IRQ_EXT + 211)   /* Low Power UART 8 */
#define IMX9_IRQ_RESERVED244            (IMX9_IRQ_EXT + 212)   /* CM33 MCM interrupt */
#define IMX9_IRQ_RESERVED245            (IMX9_IRQ_EXT + 213)   /* SFA interrupt */
#define IMX9_IRQ_RESERVED246            (IMX9_IRQ_EXT + 214)   /* GIC600 INTERRUPT */
#define IMX9_IRQ_RESERVED247            (IMX9_IRQ_EXT + 215)   /* GIC600 INTERRUPT */
#define IMX9_IRQ_RESERVED248            (IMX9_IRQ_EXT + 216)   /* GIC600 INTERRUPT */
#define IMX9_IRQ_RESERVED249            (IMX9_IRQ_EXT + 217)   /* ADC interrupt */
#define IMX9_IRQ_RESERVED250            (IMX9_IRQ_EXT + 218)   /* ADC interrupt */
#define IMX9_IRQ_RESERVED251            (IMX9_IRQ_EXT + 219)   /* ADC interrupt */
#define IMX9_IRQ_RESERVED252            (IMX9_IRQ_EXT + 220)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED253            (IMX9_IRQ_EXT + 221)   /* I3C1 wakeup irq after double sync */
#define IMX9_IRQ_RESERVED254            (IMX9_IRQ_EXT + 222)   /* I3C2 wakeup irq after double sync */
#define IMX9_IRQ_RESERVED255            (IMX9_IRQ_EXT + 223)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED256            (IMX9_IRQ_EXT + 224)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED257            (IMX9_IRQ_EXT + 225)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED258            (IMX9_IRQ_EXT + 226)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED259            (IMX9_IRQ_EXT + 227)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED260            (IMX9_IRQ_EXT + 228)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED261            (IMX9_IRQ_EXT + 229)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED262            (IMX9_IRQ_EXT + 230)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED263            (IMX9_IRQ_EXT + 231)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED264            (IMX9_IRQ_EXT + 232)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED265            (IMX9_IRQ_EXT + 233)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED266            (IMX9_IRQ_EXT + 234)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED267            (IMX9_IRQ_EXT + 235)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED268            (IMX9_IRQ_EXT + 236)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED269            (IMX9_IRQ_EXT + 237)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED270            (IMX9_IRQ_EXT + 238)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED271            (IMX9_IRQ_EXT + 239)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED272            (IMX9_IRQ_EXT + 240)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED273            (IMX9_IRQ_EXT + 241)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED274            (IMX9_IRQ_EXT + 242)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED275            (IMX9_IRQ_EXT + 243)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED276            (IMX9_IRQ_EXT + 244)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED277            (IMX9_IRQ_EXT + 245)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED278            (IMX9_IRQ_EXT + 246)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED279            (IMX9_IRQ_EXT + 247)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED280            (IMX9_IRQ_EXT + 248)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED281            (IMX9_IRQ_EXT + 249)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED282            (IMX9_IRQ_EXT + 250)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED283            (IMX9_IRQ_EXT + 251)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED284            (IMX9_IRQ_EXT + 252)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED285            (IMX9_IRQ_EXT + 253)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED286            (IMX9_IRQ_EXT + 254)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED287            (IMX9_IRQ_EXT + 255)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED288            (IMX9_IRQ_EXT + 256)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED289            (IMX9_IRQ_EXT + 257)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED290            (IMX9_IRQ_EXT + 258)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED291            (IMX9_IRQ_EXT + 259)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED292            (IMX9_IRQ_EXT + 260)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED293            (IMX9_IRQ_EXT + 261)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED294            (IMX9_IRQ_EXT + 262)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED295            (IMX9_IRQ_EXT + 263)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED296            (IMX9_IRQ_EXT + 264)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED297            (IMX9_IRQ_EXT + 265)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED298            (IMX9_IRQ_EXT + 266)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED299            (IMX9_IRQ_EXT + 267)   /* Reserved interrupt */
#define IMX9_IRQ_RESERVED300            (IMX9_IRQ_EXT + 268)   /* ADC Asynchronous Interrupt */

/* Total amount of entries in system vector table */

#define NR_IRQS                         (301)

#endif /* __ARCH_ARM64_INCLUDE_IMX9_IMX93_IRQ_H */
