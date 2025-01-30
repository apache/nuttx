/****************************************************************************
 * arch/arm/include/mcx-nxxx/n236_irq.h
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

#ifndef ARCH_ARM_INCLUDE_MCX_NXXX_N236_IRQ_H
#define ARCH_ARM_INCLUDE_MCX_NXXX_N236_IRQ_H

#define NXXX_IRQ_OR                     (NXXX_IRQ_EXTINT + 0)                 /**< OR IRQ */
#define NXXX_IRQ_EDMA_0_CH0             (NXXX_IRQ_EXTINT + 1)                 /**< eDMA_0_CH0 error or transfer complete */
#define NXXX_IRQ_EDMA_0_CH1             (NXXX_IRQ_EXTINT + 2)                 /**< eDMA_0_CH1 error or transfer complete */
#define NXXX_IRQ_EDMA_0_CH2             (NXXX_IRQ_EXTINT + 3)                 /**< eDMA_0_CH2 error or transfer complete */
#define NXXX_IRQ_EDMA_0_CH3             (NXXX_IRQ_EXTINT + 4)                 /**< eDMA_0_CH3 error or transfer complete */
#define NXXX_IRQ_EDMA_0_CH4             (NXXX_IRQ_EXTINT + 5)                 /**< eDMA_0_CH4 error or transfer complete */
#define NXXX_IRQ_EDMA_0_CH5             (NXXX_IRQ_EXTINT + 6)                 /**< eDMA_0_CH5 error or transfer complete */
#define NXXX_IRQ_EDMA_0_CH6             (NXXX_IRQ_EXTINT + 7)                 /**< eDMA_0_CH6 error or transfer complete */
#define NXXX_IRQ_EDMA_0_CH7             (NXXX_IRQ_EXTINT + 8)                 /**< eDMA_0_CH7 error or transfer complete */
#define NXXX_IRQ_EDMA_0_CH8             (NXXX_IRQ_EXTINT + 9)                 /**< eDMA_0_CH8 error or transfer complete */
#define NXXX_IRQ_EDMA_0_CH9             (NXXX_IRQ_EXTINT + 10)                /**< eDMA_0_CH9 error or transfer complete */
#define NXXX_IRQ_EDMA_0_CH10            (NXXX_IRQ_EXTINT + 11)                /**< eDMA_0_CH10 error or transfer complete */
#define NXXX_IRQ_EDMA_0_CH11            (NXXX_IRQ_EXTINT + 12)                /**< eDMA_0_CH11 error or transfer complete */
#define NXXX_IRQ_EDMA_0_CH12            (NXXX_IRQ_EXTINT + 13)                /**< eDMA_0_CH12 error or transfer complete */
#define NXXX_IRQ_EDMA_0_CH13            (NXXX_IRQ_EXTINT + 14)                /**< eDMA_0_CH13 error or transfer complete */
#define NXXX_IRQ_EDMA_0_CH14            (NXXX_IRQ_EXTINT + 15)                /**< eDMA_0_CH14 error or transfer complete */
#define NXXX_IRQ_EDMA_0_CH15            (NXXX_IRQ_EXTINT + 16)                /**< eDMA_0_CH15 error or transfer complete */
#define NXXX_IRQ_GPIO00                 (NXXX_IRQ_EXTINT + 17)                /**< GPIO0 interrupt 0 */
#define NXXX_IRQ_GPIO01                 (NXXX_IRQ_EXTINT + 18)                /**< GPIO0 interrupt 1 */
#define NXXX_IRQ_GPIO10                 (NXXX_IRQ_EXTINT + 19)                /**< GPIO1 interrupt 0 */
#define NXXX_IRQ_GPIO11                 (NXXX_IRQ_EXTINT + 20)                /**< GPIO1 interrupt 1 */
#define NXXX_IRQ_GPIO20                 (NXXX_IRQ_EXTINT + 21)                /**< GPIO2 interrupt 0 */
#define NXXX_IRQ_GPIO21                 (NXXX_IRQ_EXTINT + 22)                /**< GPIO2 interrupt 1 */
#define NXXX_IRQ_GPIO30                 (NXXX_IRQ_EXTINT + 23)                /**< GPIO3 interrupt 0 */
#define NXXX_IRQ_GPIO31                 (NXXX_IRQ_EXTINT + 24)                /**< GPIO3 interrupt 1 */
#define NXXX_IRQ_GPIO40                 (NXXX_IRQ_EXTINT + 25)                /**< GPIO4 interrupt 0 */
#define NXXX_IRQ_GPIO41                 (NXXX_IRQ_EXTINT + 26)                /**< GPIO4 interrupt 1 */
#define NXXX_IRQ_GPIO50                 (NXXX_IRQ_EXTINT + 27)                /**< GPIO5 interrupt 0 */
#define NXXX_IRQ_GPIO51                 (NXXX_IRQ_EXTINT + 28)                /**< GPIO5 interrupt 1 */
#define NXXX_IRQ_UTICK0                 (NXXX_IRQ_EXTINT + 29)                /**< Micro-Tick Timer interrupt */
#define NXXX_IRQ_MRT0                   (NXXX_IRQ_EXTINT + 30)                /**< Multi-Rate Timer interrupt */
#define NXXX_IRQ_CTIMER0                (NXXX_IRQ_EXTINT + 31)                /**< Standard counter/timer 0 interrupt */
#define NXXX_IRQ_CTIMER1                (NXXX_IRQ_EXTINT + 32)                /**< Standard counter/timer 1 interrupt */
#define NXXX_IRQ_RESERVED49             (NXXX_IRQ_EXTINT + 33)                /**< Reserved interrupt */
#define NXXX_IRQ_CTIMER2                (NXXX_IRQ_EXTINT + 34)                /**< Standard counter/timer 2 interrupt */
#define NXXX_IRQ_LP_FLEXCOMM0           (NXXX_IRQ_EXTINT + 35)                /**< LP_FLEXCOMM0 (NXXX_IRQ_EXTINT + LPSPI interrupt or LPI2C interrupt or LPUART Receive/Transmit interrupt) */
#define NXXX_IRQ_LP_FLEXCOMM1           (NXXX_IRQ_EXTINT + 36)                /**< LP_FLEXCOMM1 (NXXX_IRQ_EXTINT + LPSPI interrupt or LPI2C interrupt or LPUART Receive/Transmit interrupt) */
#define NXXX_IRQ_LP_FLEXCOMM2           (NXXX_IRQ_EXTINT + 37)                /**< LP_FLEXCOMM2 (NXXX_IRQ_EXTINT + LPSPI interrupt or LPI2C interrupt or LPUART Receive/Transmit interrupt) */
#define NXXX_IRQ_LP_FLEXCOMM3           (NXXX_IRQ_EXTINT + 38)                /**< LP_FLEXCOMM3 (NXXX_IRQ_EXTINT + LPSPI interrupt or LPI2C interrupt or LPUART Receive/Transmit interrupt) */
#define NXXX_IRQ_LP_FLEXCOMM4           (NXXX_IRQ_EXTINT + 39)                /**< LP_FLEXCOMM4 (NXXX_IRQ_EXTINT + LPSPI interrupt or LPI2C interrupt or LPUART Receive/Transmit interrupt) */
#define NXXX_IRQ_LP_FLEXCOMM5           (NXXX_IRQ_EXTINT + 40)                /**< LP_FLEXCOMM5 (NXXX_IRQ_EXTINT + LPSPI interrupt or LPI2C interrupt or LPUART Receive/Transmit interrupt) */
#define NXXX_IRQ_LP_FLEXCOMM6           (NXXX_IRQ_EXTINT + 41)                /**< LP_FLEXCOMM6 (NXXX_IRQ_EXTINT + LPSPI interrupt or LPI2C interrupt or LPUART Receive/Transmit interrupt) */
#define NXXX_IRQ_LP_FLEXCOMM7           (NXXX_IRQ_EXTINT + 42)                /**< LP_FLEXCOMM7 (NXXX_IRQ_EXTINT + LPSPI interrupt or LPI2C interrupt or LPUART Receive/Transmit interrupt) */
#define NXXX_IRQ_RESERVED59             (NXXX_IRQ_EXTINT + 43)                /**< Reserved interrupt */
#define NXXX_IRQ_RESERVED60             (NXXX_IRQ_EXTINT + 44)                /**< Reserved interrupt */
#define NXXX_IRQ_ADC0                   (NXXX_IRQ_EXTINT + 45)                /**< Analog-to-Digital Converter 0 - General Purpose interrupt */
#define NXXX_IRQ_ADC1                   (NXXX_IRQ_EXTINT + 46)                /**< Analog-to-Digital Converter 1 - General Purpose interrupt */
#define NXXX_IRQ_PINT0                  (NXXX_IRQ_EXTINT + 47)                /**< Pin Interrupt Pattern Match Interrupt */
#define NXXX_IRQ_PDM_EVENT              (NXXX_IRQ_EXTINT + 48)                /**< Microphone Interface interrupt  */
#define NXXX_IRQ_RESERVED65             (NXXX_IRQ_EXTINT + 49)                /**< Reserved interrupt */
#define NXXX_IRQ_RESERVED66             (NXXX_IRQ_EXTINT + 50)                /**< Reserved interrupt */
#define NXXX_IRQ_USB0_DCD               (NXXX_IRQ_EXTINT + 51)                /**< Universal Serial Bus - Device Charge Detect interrupt */
#define NXXX_IRQ_RTC                    (NXXX_IRQ_EXTINT + 52)                /**< RTC Subsystem interrupt (NXXX_IRQ_EXTINT + RTC interrupt or Wake timer interrupt) */
#define NXXX_IRQ_SMARTDMA               (NXXX_IRQ_EXTINT + 53)                /**< SmartDMA_IRQ */
#define NXXX_IRQ_RESERVED70             (NXXX_IRQ_EXTINT + 54)                /**< Reserved interrupt */
#define NXXX_IRQ_CTIMER3                (NXXX_IRQ_EXTINT + 55)                /**< Standard counter/timer 3 interrupt */
#define NXXX_IRQ_CTIMER4                (NXXX_IRQ_EXTINT + 56)                /**< Standard counter/timer 4 interrupt */
#define NXXX_IRQ_OS_EVENT               (NXXX_IRQ_EXTINT + 57)                /**< OS event timer interrupt */
#define NXXX_IRQ_RESERVED74             (NXXX_IRQ_EXTINT + 58)                /**< Reserved interrupt */
#define NXXX_IRQ_SAI0                   (NXXX_IRQ_EXTINT + 59)                /**< Serial Audio Interface 0 interrupt */
#define NXXX_IRQ_SAI1                   (NXXX_IRQ_EXTINT + 60)                /**< Serial Audio Interface 1 interrupt */
#define NXXX_IRQ_RESERVED77             (NXXX_IRQ_EXTINT + 61)                /**< Reserved interrupt */
#define NXXX_IRQ_CAN0                   (NXXX_IRQ_EXTINT + 62)                /**< Controller Area Network 0 interrupt */
#define NXXX_IRQ_CAN1                   (NXXX_IRQ_EXTINT + 63)                /**< Controller Area Network 1 interrupt */
#define NXXX_IRQ_RESERVED80             (NXXX_IRQ_EXTINT + 64)                /**< Reserved interrupt */
#define NXXX_IRQ_RESERVED81             (NXXX_IRQ_EXTINT + 65)                /**< Reserved interrupt */
#define NXXX_IRQ_USB1_HS_PHY            (NXXX_IRQ_EXTINT + 66)                /**< USBHS DCD or USBHS Phy interrupt */
#define NXXX_IRQ_USB1_HS                (NXXX_IRQ_EXTINT + 67)                /**< USB High Speed OTG Controller interrupt  */
#define NXXX_IRQ_SEC_HYPERVISOR_CALL    (NXXX_IRQ_EXTINT + 68)                /**< AHB Secure Controller hypervisor call interrupt */
#define NXXX_IRQ_RESERVED85             (NXXX_IRQ_EXTINT + 69)                /**< Reserved interrupt */
#define NXXX_IRQ_RESERVED86             (NXXX_IRQ_EXTINT + 70)                /**< Reserved interrupt */
#define NXXX_IRQ_FREQME                 (NXXX_IRQ_EXTINT + 71)                /**< Frequency Measurement interrupt */
#define NXXX_IRQ_SEC_VIO                (NXXX_IRQ_EXTINT + 72)                /**< Secure violation interrupt (NXXX_IRQ_EXTINT + Memory Block Checker interrupt or secure AHB matrix violation interrupt) */
#define NXXX_IRQ_ELS                    (NXXX_IRQ_EXTINT + 73)                /**< ELS interrupt */
#define NXXX_IRQ_PKC                    (NXXX_IRQ_EXTINT + 74)                /**< PKC interrupt */
#define NXXX_IRQ_PUF                    (NXXX_IRQ_EXTINT + 75)                /**< Physical Unclonable Function interrupt */
#define NXXX_IRQ_RESERVED92             (NXXX_IRQ_EXTINT + 76)                /**< Reserved interrupt */
#define NXXX_IRQ_EDMA_1_CH0             (NXXX_IRQ_EXTINT + 77)                /**< eDMA_1_CH0 error or transfer complete */
#define NXXX_IRQ_EDMA_1_CH1             (NXXX_IRQ_EXTINT + 78)                /**< eDMA_1_CH1 error or transfer complete */
#define NXXX_IRQ_EDMA_1_CH2             (NXXX_IRQ_EXTINT + 79)                /**< eDMA_1_CH2 error or transfer complete */
#define NXXX_IRQ_EDMA_1_CH3             (NXXX_IRQ_EXTINT + 80)                /**< eDMA_1_CH3 error or transfer complete */
#define NXXX_IRQ_EDMA_1_CH4             (NXXX_IRQ_EXTINT + 81)                /**< eDMA_1_CH4 error or transfer complete */
#define NXXX_IRQ_EDMA_1_CH5             (NXXX_IRQ_EXTINT + 82)                /**< eDMA_1_CH5 error or transfer complete */
#define NXXX_IRQ_EDMA_1_CH6             (NXXX_IRQ_EXTINT + 83)                /**< eDMA_1_CH6 error or transfer complete */
#define NXXX_IRQ_EDMA_1_CH7             (NXXX_IRQ_EXTINT + 84)                /**< eDMA_1_CH7 error or transfer complete */
#define NXXX_IRQ_RESERVED101            (NXXX_IRQ_EXTINT + 85)                /**< Reserved interrupt */
#define NXXX_IRQ_RESERVED102            (NXXX_IRQ_EXTINT + 86)                /**< Reserved interrupt */
#define NXXX_IRQ_RESERVED103            (NXXX_IRQ_EXTINT + 87)                /**< Reserved interrupt */
#define NXXX_IRQ_RESERVED104            (NXXX_IRQ_EXTINT + 88)                /**< Reserved interrupt */
#define NXXX_IRQ_RESERVED105            (NXXX_IRQ_EXTINT + 89)                /**< Reserved interrupt */
#define NXXX_IRQ_RESERVED106            (NXXX_IRQ_EXTINT + 90)                /**< Reserved interrupt */
#define NXXX_IRQ_RESERVED107            (NXXX_IRQ_EXTINT + 91)                /**< Reserved interrupt */
#define NXXX_IRQ_RESERVED108            (NXXX_IRQ_EXTINT + 92)                /**< Reserved interrupt */
#define NXXX_IRQ_CDOG0                  (NXXX_IRQ_EXTINT + 93)                /**< Code Watchdog Timer 0 interrupt */
#define NXXX_IRQ_CDOG1                  (NXXX_IRQ_EXTINT + 94)                /**< Code Watchdog Timer 1 interrupt */
#define NXXX_IRQ_I3C0                   (NXXX_IRQ_EXTINT + 95)                /**< Improved Inter Integrated Circuit interrupt 0 */
#define NXXX_IRQ_I3C1                   (NXXX_IRQ_EXTINT + 96)                /**< Improved Inter Integrated Circuit interrupt 1 */
#define NXXX_IRQ_RESERVED113            (NXXX_IRQ_EXTINT + 97)                /**< Reserved interrupt */
#define NXXX_IRQ_GDET                   (NXXX_IRQ_EXTINT + 98)                /**< Digital Glitch Detect 0 interrupt  or Digital Glitch Detect 1 interrupt */
#define NXXX_IRQ_VBAT0                  (NXXX_IRQ_EXTINT + 99)                /**< VBAT interrupt( VBAT interrupt or digital tamper interrupt) */
#define NXXX_IRQ_EWM0                   (NXXX_IRQ_EXTINT + 100)               /**< External Watchdog Monitor interrupt */
#define NXXX_IRQ_RESERVED117            (NXXX_IRQ_EXTINT + 101)               /**< Reserved interrupt */
#define NXXX_IRQ_RESERVED118            (NXXX_IRQ_EXTINT + 102)               /**< Reserved interrupt */
#define NXXX_IRQ_RESERVED119            (NXXX_IRQ_EXTINT + 103)               /**< Reserved interrupt */
#define NXXX_IRQ_RESERVED120            (NXXX_IRQ_EXTINT + 104)               /**< Reserved interrupt */
#define NXXX_IRQ_FLEXIO                 (NXXX_IRQ_EXTINT + 105)               /**< Flexible Input/Output interrupt */
#define NXXX_IRQ_RESERVED122            (NXXX_IRQ_EXTINT + 106)               /**< Reserved interrupt */
#define NXXX_IRQ_RESERVED123            (NXXX_IRQ_EXTINT + 107)               /**< Reserved interrupt */
#define NXXX_IRQ_RESERVED124            (NXXX_IRQ_EXTINT + 108)               /**< Reserved interrupt */
#define NXXX_IRQ_HSCMP0                 (NXXX_IRQ_EXTINT + 109)               /**< High-Speed comparator0 interrupt */
#define NXXX_IRQ_HSCMP1                 (NXXX_IRQ_EXTINT + 110)               /**< High-Speed comparator1 interrupt */
#define NXXX_IRQ_RESERVED127            (NXXX_IRQ_EXTINT + 111)               /**< Reserved interrupt */
#define NXXX_IRQ_FLEXPWM0_RELOAD_ERROR  (NXXX_IRQ_EXTINT + 112)               /**< FlexPWM0_reload_error interrupt */
#define NXXX_IRQ_FLEXPWM0_FAULT         (NXXX_IRQ_EXTINT + 113)               /**< FlexPWM0_fault interrupt */
#define NXXX_IRQ_FLEXPWM0_SUBMODULE0    (NXXX_IRQ_EXTINT + 114)               /**< FlexPWM0 Submodule 0 capture/compare/reload interrupt */
#define NXXX_IRQ_FLEXPWM0_SUBMODULE1    (NXXX_IRQ_EXTINT + 115)               /**< FlexPWM0 Submodule 1 capture/compare/reload interrupt */
#define NXXX_IRQ_FLEXPWM0_SUBMODULE2    (NXXX_IRQ_EXTINT + 116)               /**< FlexPWM0 Submodule 2 capture/compare/reload interrupt */
#define NXXX_IRQ_FLEXPWM0_SUBMODULE3    (NXXX_IRQ_EXTINT + 117)               /**< FlexPWM0 Submodule 3 capture/compare/reload interrupt */
#define NXXX_IRQ_FLEXPWM1_RELOAD_ERROR  (NXXX_IRQ_EXTINT + 118)               /**< FlexPWM1_reload_error interrupt */
#define NXXX_IRQ_FLEXPWM1_FAULT         (NXXX_IRQ_EXTINT + 119)               /**< FlexPWM1_fault interrupt */
#define NXXX_IRQ_FLEXPWM1_SUBMODULE0    (NXXX_IRQ_EXTINT + 120)               /**< FlexPWM1 Submodule 0 capture/compare/reload interrupt */
#define NXXX_IRQ_FLEXPWM1_SUBMODULE1    (NXXX_IRQ_EXTINT + 121)               /**< FlexPWM1 Submodule 1 capture/compare/reload interrupt */
#define NXXX_IRQ_FLEXPWM1_SUBMODULE2    (NXXX_IRQ_EXTINT + 122)               /**< FlexPWM1 Submodule 2 capture/compare/reload interrupt */
#define NXXX_IRQ_FLEXPWM1_SUBMODULE3    (NXXX_IRQ_EXTINT + 123)               /**< FlexPWM1 Submodule 3 capture/compare/reload interrupt */
#define NXXX_IRQ_QDC0_COMPARE           (NXXX_IRQ_EXTINT + 124)               /**< QDC0_Compare interrupt */
#define NXXX_IRQ_QDC0_HOME              (NXXX_IRQ_EXTINT + 125)               /**< QDC0_Home interrupt */
#define NXXX_IRQ_QDC0_WDG_SAB           (NXXX_IRQ_EXTINT + 126)               /**< QDC0_WDG_IRQ/SAB interrupt */
#define NXXX_IRQ_QDC0_IDX               (NXXX_IRQ_EXTINT + 127)               /**< QDC0_IDX interrupt */
#define NXXX_IRQ_QDC1_COMPARE           (NXXX_IRQ_EXTINT + 128)               /**< QDC1_Compare interrupt */
#define NXXX_IRQ_QDC1_HOME              (NXXX_IRQ_EXTINT + 129)               /**< QDC1_Home interrupt */
#define NXXX_IRQ_QDC1_WDG_SAB           (NXXX_IRQ_EXTINT + 130)               /**< QDC1_WDG_IRQ/SAB interrupt */
#define NXXX_IRQ_QDC1_IDX               (NXXX_IRQ_EXTINT + 131)               /**< QDC1_IDX interrupt */
#define NXXX_IRQ_ITRC0                  (NXXX_IRQ_EXTINT + 132)               /**< Intrusion and Tamper Response Controller interrupt */
#define NXXX_IRQ_RESERVED149            (NXXX_IRQ_EXTINT + 133)               /**< Reserved interrupt */
#define NXXX_IRQ_ELS_ERR                (NXXX_IRQ_EXTINT + 134)               /**< ELS error interrupt */
#define NXXX_IRQ_PKC_ERR                (NXXX_IRQ_EXTINT + 135)               /**< PKC error interrupt */
#define NXXX_IRQ_ERM_SINGLE_BIT_ERROR   (NXXX_IRQ_EXTINT + 136)               /**< ERM Single Bit error interrupt */
#define NXXX_IRQ_ERM_MULTI_BIT_ERROR    (NXXX_IRQ_EXTINT + 137)               /**< ERM Multi Bit error interrupt */
#define NXXX_IRQ_FMU0                   (NXXX_IRQ_EXTINT + 138)               /**< Flash Management Unit interrupt */
#define NXXX_IRQ_RESERVED155            (NXXX_IRQ_EXTINT + 139)               /**< Reserved interrupt */
#define NXXX_IRQ_RESERVED156            (NXXX_IRQ_EXTINT + 140)               /**< Reserved interrupt */
#define NXXX_IRQ_RESERVED157            (NXXX_IRQ_EXTINT + 141)               /**< Reserved interrupt */
#define NXXX_IRQ_RESERVED158            (NXXX_IRQ_EXTINT + 142)               /**< Reserved interrupt */
#define NXXX_IRQ_LPTMR0                 (NXXX_IRQ_EXTINT + 143)               /**< Low Power Timer 0 interrupt */
#define NXXX_IRQ_LPTMR1                 (NXXX_IRQ_EXTINT + 144)               /**< Low Power Timer 1 interrupt */
#define NXXX_IRQ_SCG                    (NXXX_IRQ_EXTINT + 145)               /**< System Clock Generator interrupt */
#define NXXX_IRQ_SPC                    (NXXX_IRQ_EXTINT + 146)               /**< System Power Controller interrupt */
#define NXXX_IRQ_WUU                    (NXXX_IRQ_EXTINT + 147)               /**< Wake Up Unit interrupt */
#define NXXX_IRQ_PORT_EFT               (NXXX_IRQ_EXTINT + 148)               /**< PORT0~5 EFT interrupt */
#define NXXX_IRQ_RESERVED165            (NXXX_IRQ_EXTINT + 149)               /**< Reserved interrupt */
#define NXXX_IRQ_RESERVED166            (NXXX_IRQ_EXTINT + 150)               /**< Reserved interrupt */
#define NXXX_IRQ_RESERVED167            (NXXX_IRQ_EXTINT + 151)               /**< Reserved interrupt */
#define NXXX_IRQ_WWDT0                  (NXXX_IRQ_EXTINT + 152)               /**< Windowed Watchdog Timer 0 interrupt */
#define NXXX_IRQ_WWDT1                  (NXXX_IRQ_EXTINT + 153)               /**< Windowed Watchdog Timer 1 interrupt */
#define NXXX_IRQ_CMC0                   (NXXX_IRQ_EXTINT + 154)               /**< Core Mode Controller interrupt */
#define NXXX_IRQ_RESERVED171            (NXXX_IRQ_EXTINT + 155)               /**< Reserved interrupt */

#define NXXX_IRQ_NEXTINT                (156)

/* Total amount of entries in system vector table */

#define NR_IRQS                         (NXXX_IRQ_EXTINT + NXXX_IRQ_NEXTINT)

#endif /* ARCH_ARM_INCLUDE_MCX_NXXX_N236_IRQ_H */
