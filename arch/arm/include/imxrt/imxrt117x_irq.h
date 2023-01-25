/****************************************************************************
 * arch/arm/include/imxrt/imxrt117x_irq.h
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

/* This file should never be included directly but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_IMXRT_IMXRT117X_IRQ_H
#define __ARCH_ARM_INCLUDE_IMXRT_IMXRT117X_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* External interrupts (priority levels >= 256) *****************************/

#define IMXRT_IRQ_EDMA0_16       (IMXRT_IRQ_EXTINT + 0)   /* eDMA Channel 0/16 Transfer Complete */
#define IMXRT_IRQ_EDMA1_17       (IMXRT_IRQ_EXTINT + 1)   /* eDMA Channel 1/17 Transfer Complete */
#define IMXRT_IRQ_EDMA2_18       (IMXRT_IRQ_EXTINT + 2)   /* eDMA Channel 2/18 Transfer Complete */
#define IMXRT_IRQ_EDMA3_19       (IMXRT_IRQ_EXTINT + 3)   /* eDMA Channel 3/19 Transfer Complete */
#define IMXRT_IRQ_EDMA4_20       (IMXRT_IRQ_EXTINT + 4)   /* eDMA Channel 4/20 Transfer Complete */
#define IMXRT_IRQ_EDMA5_21       (IMXRT_IRQ_EXTINT + 5)   /* eDMA Channel 5/21 Transfer Complete */
#define IMXRT_IRQ_EDMA6_22       (IMXRT_IRQ_EXTINT + 6)   /* eDMA Channel 6/22 Transfer Complete */
#define IMXRT_IRQ_EDMA7_23       (IMXRT_IRQ_EXTINT + 7)   /* eDMA Channel 7/23 Transfer Complete */
#define IMXRT_IRQ_EDMA8_24       (IMXRT_IRQ_EXTINT + 8)   /* eDMA Channel 8/24 Transfer Complete */
#define IMXRT_IRQ_EDMA9_25       (IMXRT_IRQ_EXTINT + 9)   /* eDMA Channel 9/25 Transfer Complete */
#define IMXRT_IRQ_EDMA10_26      (IMXRT_IRQ_EXTINT + 10)  /* eDMA Channel 10/26 Transfer Complete */
#define IMXRT_IRQ_EDMA11_27      (IMXRT_IRQ_EXTINT + 11)  /* eDMA Channel 11/27 Transfer Complete */
#define IMXRT_IRQ_EDMA12_28      (IMXRT_IRQ_EXTINT + 12)  /* eDMA Channel 12/28 Transfer Complete */
#define IMXRT_IRQ_EDMA13_29      (IMXRT_IRQ_EXTINT + 13)  /* eDMA Channel 13/29 Transfer Complete */
#define IMXRT_IRQ_EDMA14_30      (IMXRT_IRQ_EXTINT + 14)  /* eDMA Channel 14/30 Transfer Complete */
#define IMXRT_IRQ_EDMA15_31      (IMXRT_IRQ_EXTINT + 15)  /* eDMA Channel 15/31 Transfer Complete */
#define IMXRT_IRQ_EDMA_ERROR     (IMXRT_IRQ_EXTINT + 16)  /* Error Interrupt, Channels 0-15 / 16-31 */
#define IMXRT_IRQ_CM70           (IMXRT_IRQ_EXTINT + 17)  /* CTI trigger outputs (internal: CTIIRQ[0]) */
#define IMXRT_IRQ_CM71           (IMXRT_IRQ_EXTINT + 18)  /* CTI trigger outputs (internal: CTIIRQ[1]) */
#define IMXRT_IRQ_CM7CP          (IMXRT_IRQ_EXTINT + 19)  /* Core Platform exception IRQ */
#define IMXRT_IRQ_LPUART1        (IMXRT_IRQ_EXTINT + 20)  /* UART1 TX/RX interrupt */
#define IMXRT_IRQ_LPUART2        (IMXRT_IRQ_EXTINT + 21)  /* UART2 TX/RX interrupt */
#define IMXRT_IRQ_LPUART3        (IMXRT_IRQ_EXTINT + 22)  /* UART3 TX/RX interrupt */
#define IMXRT_IRQ_LPUART4        (IMXRT_IRQ_EXTINT + 23)  /* UART4 TX/RX interrupt */
#define IMXRT_IRQ_LPUART5        (IMXRT_IRQ_EXTINT + 24)  /* UART5 TX/RX interrupt */
#define IMXRT_IRQ_LPUART6        (IMXRT_IRQ_EXTINT + 25)  /* UART6 TX/RX interrupt */
#define IMXRT_IRQ_LPUART7        (IMXRT_IRQ_EXTINT + 26)  /* UART7 TX/RX interrupt */
#define IMXRT_IRQ_LPUART8        (IMXRT_IRQ_EXTINT + 27)  /* UART8 TX/RX interrupt */
#define IMXRT_IRQ_LPUART9        (IMXRT_IRQ_EXTINT + 28)  /* UART9 TX/RX interrupt */
#define IMXRT_IRQ_LPUART10       (IMXRT_IRQ_EXTINT + 29)  /* UART10 TX/RX interrupt */
#define IMXRT_IRQ_LPUART11       (IMXRT_IRQ_EXTINT + 30)  /* UART11 TX/RX interrupt */
#define IMXRT_IRQ_LPUART12       (IMXRT_IRQ_EXTINT + 31)  /* UART12 TX/RX interrupt */
#define IMXRT_IRQ_LPI2C1         (IMXRT_IRQ_EXTINT + 32)  /* LPI2C1 interrupt */
#define IMXRT_IRQ_LPI2C2         (IMXRT_IRQ_EXTINT + 33)  /* LPI2C2 interrupt */
#define IMXRT_IRQ_LPI2C3         (IMXRT_IRQ_EXTINT + 34)  /* LPI2C3 interrupt */
#define IMXRT_IRQ_LPI2C4         (IMXRT_IRQ_EXTINT + 35)  /* LPI2C4 interrupt */
#define IMXRT_IRQ_LPI2C5         (IMXRT_IRQ_EXTINT + 36)  /* LPI2C5 interrupt */
#define IMXRT_IRQ_LPI2C6         (IMXRT_IRQ_EXTINT + 37)  /* LPI2C6 interrupt */
#define IMXRT_IRQ_LPSPI1         (IMXRT_IRQ_EXTINT + 38)  /* LPSPI1 interrupt */
#define IMXRT_IRQ_LPSPI2         (IMXRT_IRQ_EXTINT + 39)  /* LPSPI2 interrupt */
#define IMXRT_IRQ_LPSPI3         (IMXRT_IRQ_EXTINT + 40)  /* LPSPI3 interrupt */
#define IMXRT_IRQ_LPSPI4         (IMXRT_IRQ_EXTINT + 41)  /* LPSPI4 interrupt */
#define IMXRT_IRQ_LPSPI5         (IMXRT_IRQ_EXTINT + 42)  /* LPSPI5 interrupt */
#define IMXRT_IRQ_LPSPI6         (IMXRT_IRQ_EXTINT + 43)  /* LPSPI6 interrupt */
#define IMXRT_IRQ_CAN1           (IMXRT_IRQ_EXTINT + 44)  /* CAN1 interrupt */
#define IMXRT_IRQ_CAN1ERR        (IMXRT_IRQ_EXTINT + 45)  /* CAN1 error interrupt */
#define IMXRT_IRQ_CAN2           (IMXRT_IRQ_EXTINT + 46)  /* CAN2 interrupt */
#define IMXRT_IRQ_CAN2ERR        (IMXRT_IRQ_EXTINT + 47)  /* CAN2 error interrupt */
#define IMXRT_IRQ_CAN3           (IMXRT_IRQ_EXTINT + 48)  /* CAN3 interrupt */
#define IMXRT_IRQ_CAN3ERR        (IMXRT_IRQ_EXTINT + 49)  /* CAN3 error interrupt */
#define IMXRT_IRQ_CM7FR          (IMXRT_IRQ_EXTINT + 50)  /* FlexRAM address fault */
#define IMXRT_IRQ_KPP            (IMXRT_IRQ_EXTINT + 51)  /* Keypad interrupt */
#define IMXRT_IRQ_RESERVED52     (IMXRT_IRQ_EXTINT + 52)  /* Reserved */
#define IMXRT_IRQ_GPRIRQ         (IMXRT_IRQ_EXTINT + 53)  /* Notify cores on exception while boot */
#define IMXRT_IRQ_ELCDIF         (IMXRT_IRQ_EXTINT + 54)  /* eLCDIF interrupt */
#define IMXRT_IRQ_LCDIFV2        (IMXRT_IRQ_EXTINT + 55)  /* LCDIFv2 interrupt */
#define IMXRT_IRQ_CSI            (IMXRT_IRQ_EXTINT + 56)  /* CSI interrupt */
#define IMXRT_IRQ_PXP            (IMXRT_IRQ_EXTINT + 57)  /* PXP interrupt */
#define IMXRT_IRQ_MIPICSI        (IMXRT_IRQ_EXTINT + 58)  /* MIPI CSI interrupt */
#define IMXRT_IRQ_MIPIDSI        (IMXRT_IRQ_EXTINT + 59)  /* MIPI DSI interrupt */
#define IMXRT_IRQ_GPU2D          (IMXRT_IRQ_EXTINT + 60)  /* GPU2D interrupt */
#define IMXRT_IRQ_GPIO6_0_15     (IMXRT_IRQ_EXTINT + 61)  /* GPIO6 channel 0-15 interrupt */
#define IMXRT_IRQ_GPIO6_16_31    (IMXRT_IRQ_EXTINT + 62)  /* GPIO6 channel 6-31 interrupt */
#define IMXRT_IRQ_DAC            (IMXRT_IRQ_EXTINT + 63)  /* DAC interrupt */
#define IMXRT_IRQ_KEYMGR         (IMXRT_IRQ_EXTINT + 64)  /* PUF interrupt */
#define IMXRT_IRQ_WDOG2          (IMXRT_IRQ_EXTINT + 65)  /* Watchdog 2 Timer reset */
#define IMXRT_IRQ_SNVS           (IMXRT_IRQ_EXTINT + 66)  /* SNVS Functional Interrupt */
#define IMXRT_IRQ_SNVSSEC        (IMXRT_IRQ_EXTINT + 67)  /* SNVS Security Interrupt */
#define IMXRT_IRQ_SNVSSB         (IMXRT_IRQ_EXTINT + 68)  /* ON-OFF short button press */
#define IMXRT_IRQ_CAAMJQ0        (IMXRT_IRQ_EXTINT + 69)  /* CAAM interrupt queue for JQ0 */
#define IMXRT_IRQ_CAAMJQ1        (IMXRT_IRQ_EXTINT + 70)  /* CAAM interrupt queue for JQ1 */
#define IMXRT_IRQ_CAAMJQ2        (IMXRT_IRQ_EXTINT + 71)  /* CAAM interrupt queue for JQ2 */
#define IMXRT_IRQ_CAAMJQ3        (IMXRT_IRQ_EXTINT + 72)  /* CAAM interrupt queue for JQ3 */
#define IMXRT_IRQ_CAAMERR        (IMXRT_IRQ_EXTINT + 73)  /* CAAM interrupt queue for recoverable error */
#define IMXRT_IRQ_CAAMRTIC       (IMXRT_IRQ_EXTINT + 74)  /* CAAM interrupt for RTIC */
#define IMXRT_IRQ_CDOG           (IMXRT_IRQ_EXTINT + 75)  /* CDOG interrupt */
#define IMXRT_IRQ_SAI1           (IMXRT_IRQ_EXTINT + 76)  /* SAI1 interrupt */
#define IMXRT_IRQ_SAI2           (IMXRT_IRQ_EXTINT + 77)  /* SAI2 interrupt */
#define IMXRT_IRQ_SAI3RX         (IMXRT_IRQ_EXTINT + 78)  /* SAI3 RX interrupt */
#define IMXRT_IRQ_SAI3TX         (IMXRT_IRQ_EXTINT + 79)  /* SAI3 TX interrupt */
#define IMXRT_IRQ_SAI4RX         (IMXRT_IRQ_EXTINT + 80)  /* SAI4 RX interrupt */
#define IMXRT_IRQ_SAI4TX         (IMXRT_IRQ_EXTINT + 81)  /* SAI4 TX interrupt */
#define IMXRT_IRQ_SPDIF          (IMXRT_IRQ_EXTINT + 82)  /* SPDIF interrupt */
#define IMXRT_IRQ_TEMP           (IMXRT_IRQ_EXTINT + 83)  /* TMPSNS interrupt */
#define IMXRT_IRQ_TEMPLOHI       (IMXRT_IRQ_EXTINT + 84)  /* TempSensor low / Tempsensor high */
#define IMXRT_IRQ_TEMPPANIC      (IMXRT_IRQ_EXTINT + 85)  /* TempSensor panic */
#define IMXRT_IRQ_PMU1P8         (IMXRT_IRQ_EXTINT + 86)  /* LPSR 1p8 brownout interrupt */
#define IMXRT_IRQ_PMU1P0         (IMXRT_IRQ_EXTINT + 87)  /* LPSR 1p0 brownout interrupt */
#define IMXRT_IRQ_LPADC1         (IMXRT_IRQ_EXTINT + 88)  /* ADC1 interrupt */
#define IMXRT_IRQ_LPADC2         (IMXRT_IRQ_EXTINT + 89)  /* ADC2 interrupt */
#define IMXRT_IRQ_USBPHY1        (IMXRT_IRQ_EXTINT + 90)  /* USB1 wakeup interrupt */
#define IMXRT_IRQ_USBPHY2        (IMXRT_IRQ_EXTINT + 91)  /* USB2 wakeup interrupt */
#define IMXRT_IRQ_RDC            (IMXRT_IRQ_EXTINT + 92)  /* RDC interrupt */
#define IMXRT_IRQ_GPIO13         (IMXRT_IRQ_EXTINT + 93)  /* GPIO13 interrupt */
#define IMXRT_IRQ_RESERVED94     (IMXRT_IRQ_EXTINT + 94)  /* Reserved */
#define IMXRT_IRQ_VIDEOMUX1      (IMXRT_IRQ_EXTINT + 95)  /* DCIC1 interrupt */
#define IMXRT_IRQ_VIDEOMUX2      (IMXRT_IRQ_EXTINT + 96)  /* DCIC2 interrupt */
#define IMXRT_IRQ_ASRC           (IMXRT_IRQ_EXTINT + 97)  /* ASRC interrupt */
#define IMXRT_IRQ_CM7FRECC       (IMXRT_IRQ_EXTINT + 98)  /* FlexRAM ECC fatal interrupt */
#define IMXRT_IRQ_CM7GPIO2_3     (IMXRT_IRQ_EXTINT + 99)  /* CM7_GPIO2 & CM7_GPIO3 interrupt */
#define IMXRT_IRQ_GPIO1_0_15     (IMXRT_IRQ_EXTINT + 100) /* GPIO1 INT0-15 interrupt */
#define IMXRT_IRQ_GPIO1_16_31    (IMXRT_IRQ_EXTINT + 101) /* GPIO1 INT16-31 interrupt */
#define IMXRT_IRQ_GPIO2_0_15     (IMXRT_IRQ_EXTINT + 102) /* GPIO2 INT0-15 interrupt */
#define IMXRT_IRQ_GPIO2_16_31    (IMXRT_IRQ_EXTINT + 103) /* GPIO2 INT16-31 interrupt */
#define IMXRT_IRQ_GPIO3_0_15     (IMXRT_IRQ_EXTINT + 104) /* GPIO3 INT0-15 interrupt */
#define IMXRT_IRQ_GPIO3_16_31    (IMXRT_IRQ_EXTINT + 105) /* GPIO3 INT16-31 interrupt */
#define IMXRT_IRQ_GPIO4_0_15     (IMXRT_IRQ_EXTINT + 106) /* GPIO4 INT0-15 interrupt */
#define IMXRT_IRQ_GPIO4_16_31    (IMXRT_IRQ_EXTINT + 107) /* GPIO4 INT16-31 interrupt */
#define IMXRT_IRQ_GPIO5_0_15     (IMXRT_IRQ_EXTINT + 108) /* GPIO5 INT0-15 interrupt */
#define IMXRT_IRQ_GPIO5_16_31    (IMXRT_IRQ_EXTINT + 109) /* GPIO5 INT16-31 interrupt */
#define IMXRT_IRQ_FLEXIO1        (IMXRT_IRQ_EXTINT + 110) /* IPI compare interrupt */
#define IMXRT_IRQ_FLEXIO2        (IMXRT_IRQ_EXTINT + 111) /* IPI compare interrupt */
#define IMXRT_IRQ_WDOG1          (IMXRT_IRQ_EXTINT + 112) /* Watchdog 1 Timer reset */
#define IMXRT_IRQ_WDOG3          (IMXRT_IRQ_EXTINT + 113) /* Watchdog 3 Timer reset */
#define IMXRT_IRQ_EWM            (IMXRT_IRQ_EXTINT + 114) /* EWM interrupt */
#define IMXRT_IRQ_OCOTP1         (IMXRT_IRQ_EXTINT + 115) /* Read FUSE error interrupt */
#define IMXRT_IRQ_OCOTP2         (IMXRT_IRQ_EXTINT + 116) /* Read FUSE done interrupt */
#define IMXRT_IRQ_GPC            (IMXRT_IRQ_EXTINT + 117) /* GPC interrupt */
#define IMXRT_IRQ_MU             (IMXRT_IRQ_EXTINT + 118) /* MU interrupt */
#define IMXRT_IRQ_GPT1           (IMXRT_IRQ_EXTINT + 119) /* GPT1 interrupt */
#define IMXRT_IRQ_GPT2           (IMXRT_IRQ_EXTINT + 120) /* GPT2 interrupt */
#define IMXRT_IRQ_GPT3           (IMXRT_IRQ_EXTINT + 121) /* GPT3 interrupt */
#define IMXRT_IRQ_GPT4           (IMXRT_IRQ_EXTINT + 122) /* GPT4 interrupt */
#define IMXRT_IRQ_GPT5           (IMXRT_IRQ_EXTINT + 123) /* GPT5 interrupt */
#define IMXRT_IRQ_GPT6           (IMXRT_IRQ_EXTINT + 124) /* GPT6 interrupt */
#define IMXRT_IRQ_FLEXPWM1_0     (IMXRT_IRQ_EXTINT + 125) /* FlexPWM1 capture/compare/reload 0 interrupt */
#define IMXRT_IRQ_FLEXPWM1_1     (IMXRT_IRQ_EXTINT + 126) /* FlexPWM1 capture/compare/reload 1 interrupt */
#define IMXRT_IRQ_FLEXPWM1_2     (IMXRT_IRQ_EXTINT + 127) /* FlexPWM1 capture/compare/reload 2 interrupt */
#define IMXRT_IRQ_FLEXPWM1_3     (IMXRT_IRQ_EXTINT + 128) /* FlexPWM1 capture/compare/reload 3 interrupt */
#define IMXRT_IRQ_FLEXPWM1_F     (IMXRT_IRQ_EXTINT + 129) /* FlexPWM1 fault interrupt */
#define IMXRT_IRQ_FLEXSPI1       (IMXRT_IRQ_EXTINT + 130) /* FlexSPI1 interrupt */
#define IMXRT_IRQ_FLEXSPI2       (IMXRT_IRQ_EXTINT + 131) /* FlexSPI2 interrupt */
#define IMXRT_IRQ_SEMC           (IMXRT_IRQ_EXTINT + 132) /* SEMC interrupt */
#define IMXRT_IRQ_USDHC1         (IMXRT_IRQ_EXTINT + 133) /* USDHC1 interrupt */
#define IMXRT_IRQ_USDHC2         (IMXRT_IRQ_EXTINT + 134) /* USDHC2 interrupt */
#define IMXRT_IRQ_USBOTG2        (IMXRT_IRQ_EXTINT + 135) /* USB OTG2 interrupt */
#define IMXRT_IRQ_USBOTG1        (IMXRT_IRQ_EXTINT + 136) /* USB OTG1 interrupt */
#define IMXRT_IRQ_ENET           (IMXRT_IRQ_EXTINT + 137) /* ENET MAC 0 interrupt */
#define IMXRT_IRQ_ENET1588       (IMXRT_IRQ_EXTINT + 138) /* ENET MAC 0 1588 Timer Interrupt */
#define IMXRT_IRQ_ENET2_1        (IMXRT_IRQ_EXTINT + 139) /* ENET_1G MAC 0 interrupt */
#define IMXRT_IRQ_ENET2_2        (IMXRT_IRQ_EXTINT + 140) /* ENET_1G MAC 0 interrupt */
#define IMXRT_IRQ_ENET2_3        (IMXRT_IRQ_EXTINT + 141) /* ENET_1G MAC 0 interrupt */
#define IMXRT_IRQ_ENET2_1588     (IMXRT_IRQ_EXTINT + 142) /* ENET_1G MAC 0 1588 Timer Interrupt */
#define IMXRT_IRQ_XBAR1_0_1      (IMXRT_IRQ_EXTINT + 143) /* XBAR1 interrupt 0/1 */
#define IMXRT_IRQ_XBAR1_2_3      (IMXRT_IRQ_EXTINT + 144) /* XBAR1 interrupt 2/3 */
#define IMXRT_IRQ_ADCETC_0       (IMXRT_IRQ_EXTINT + 145) /* ADC_ETC interrupt 0 */
#define IMXRT_IRQ_ADCETC_1       (IMXRT_IRQ_EXTINT + 146) /* ADC_ETC interrupt 1 */
#define IMXRT_IRQ_ADCETC_2       (IMXRT_IRQ_EXTINT + 147) /* ADC_ETC interrupt 2 */
#define IMXRT_IRQ_ADCETC_3       (IMXRT_IRQ_EXTINT + 148) /* ADC_ETC interrupt 3 */
#define IMXRT_IRQ_ADCETC_ERR     (IMXRT_IRQ_EXTINT + 149) /* ADC_ETC error interrupt */
#define IMXRT_IRQ_RESERVED150    (IMXRT_IRQ_EXTINT + 150) /* Reserved */
#define IMXRT_IRQ_RESERVED151    (IMXRT_IRQ_EXTINT + 151) /* Reserved */
#define IMXRT_IRQ_RESERVED152    (IMXRT_IRQ_EXTINT + 152) /* Reserved */
#define IMXRT_IRQ_RESERVED153    (IMXRT_IRQ_EXTINT + 153) /* Reserved */
#define IMXRT_IRQ_RESERVED154    (IMXRT_IRQ_EXTINT + 154) /* Reserved */
#define IMXRT_IRQ_PIT1           (IMXRT_IRQ_EXTINT + 155) /* PIT1 interrupt */
#define IMXRT_IRQ_PIT2           (IMXRT_IRQ_EXTINT + 156) /* PIT2 interrupt */
#define IMXRT_IRQ_ACMP1          (IMXRT_IRQ_EXTINT + 157) /* ACMP1 interrupt */
#define IMXRT_IRQ_ACMP2          (IMXRT_IRQ_EXTINT + 158) /* ACMP2 interrupt */
#define IMXRT_IRQ_ACMP3          (IMXRT_IRQ_EXTINT + 159) /* ACMP3 interrupt */
#define IMXRT_IRQ_ACMP4          (IMXRT_IRQ_EXTINT + 160) /* ACMP4 interrupt */
#define IMXRT_IRQ_RESERVED161    (IMXRT_IRQ_EXTINT + 161) /* Reserved */
#define IMXRT_IRQ_RESERVED162    (IMXRT_IRQ_EXTINT + 162) /* Reserved */
#define IMXRT_IRQ_RESERVED163    (IMXRT_IRQ_EXTINT + 163) /* Reserved */
#define IMXRT_IRQ_RESERVED164    (IMXRT_IRQ_EXTINT + 164) /* Reserved */
#define IMXRT_IRQ_QDC1           (IMXRT_IRQ_EXTINT + 165) /* QDC1 interrupt */
#define IMXRT_IRQ_QDC2           (IMXRT_IRQ_EXTINT + 166) /* QDC2 interrupt */
#define IMXRT_IRQ_QDC3           (IMXRT_IRQ_EXTINT + 167) /* QDC3 interrupt */
#define IMXRT_IRQ_QDC4           (IMXRT_IRQ_EXTINT + 168) /* QDC4 interrupt */
#define IMXRT_IRQ_RESERVED169    (IMXRT_IRQ_EXTINT + 169) /* Reserved */
#define IMXRT_IRQ_RESERVED170    (IMXRT_IRQ_EXTINT + 170) /* Reserved */
#define IMXRT_IRQ_QTIMER1        (IMXRT_IRQ_EXTINT + 171) /* QTIMER1 timer 0-3 interrupt */
#define IMXRT_IRQ_QTIMER2        (IMXRT_IRQ_EXTINT + 172) /* QTIMER2 timer 0-3 interrupt */
#define IMXRT_IRQ_QTIMER3        (IMXRT_IRQ_EXTINT + 173) /* QTIMER3 timer 0-3 interrupt */
#define IMXRT_IRQ_QTIMER4        (IMXRT_IRQ_EXTINT + 174) /* QTIMER4 timer 0-3 interrupt */
#define IMXRT_IRQ_SEMA4_CP0      (IMXRT_IRQ_EXTINT + 175) /* SEMA4 CP0 interrupt */
#define IMXRT_IRQ_SEMA4_CP1      (IMXRT_IRQ_EXTINT + 176) /* SEMA4 CP1 interrupt */
#define IMXRT_IRQ_FLEXPWM2_0     (IMXRT_IRQ_EXTINT + 177) /* FlexPWM2 capture/compare/reload 0 interrupt */
#define IMXRT_IRQ_FLEXPWM2_1     (IMXRT_IRQ_EXTINT + 178) /* FlexPWM2 capture/compare/reload 1 interrupt */
#define IMXRT_IRQ_FLEXPWM2_2     (IMXRT_IRQ_EXTINT + 179) /* FlexPWM2 capture/compare/reload 1 interrupt */
#define IMXRT_IRQ_FLEXPWM2_3     (IMXRT_IRQ_EXTINT + 180) /* FlexPWM2 capture/compare/reload 3 interrupt */
#define IMXRT_IRQ_FLEXPWM2_F     (IMXRT_IRQ_EXTINT + 181) /* FlexPWM2 fault interrupt */
#define IMXRT_IRQ_FLEXPWM3_0     (IMXRT_IRQ_EXTINT + 182) /* FlexPWM3 capture/compare/reload 0 interrupt */
#define IMXRT_IRQ_FLEXPWM3_1     (IMXRT_IRQ_EXTINT + 183) /* FlexPWM3 capture/compare/reload 1 interrupt */
#define IMXRT_IRQ_FLEXPWM3_2     (IMXRT_IRQ_EXTINT + 184) /* FlexPWM3 capture/compare/reload 1 interrupt */
#define IMXRT_IRQ_FLEXPWM3_3     (IMXRT_IRQ_EXTINT + 185) /* FlexPWM3 capture/compare/reload 3 interrupt */
#define IMXRT_IRQ_FLEXPWM3_F     (IMXRT_IRQ_EXTINT + 186) /* FlexPWM3 fault interrupt */
#define IMXRT_IRQ_FLEXPWM4_0     (IMXRT_IRQ_EXTINT + 187) /* FlexPWM4 capture/compare/reload 0 interrupt */
#define IMXRT_IRQ_FLEXPWM4_1     (IMXRT_IRQ_EXTINT + 188) /* FlexPWM4 capture/compare/reload 1 interrupt */
#define IMXRT_IRQ_FLEXPWM4_2     (IMXRT_IRQ_EXTINT + 189) /* FlexPWM4 capture/compare/reload 1 interrupt */
#define IMXRT_IRQ_FLEXPWM4_3     (IMXRT_IRQ_EXTINT + 190) /* FlexPWM4 capture/compare/reload 3 interrupt */
#define IMXRT_IRQ_FLEXPWM4_F     (IMXRT_IRQ_EXTINT + 191) /* FlexPWM4 fault interrupt */
#define IMXRT_IRQ_RESERVED192    (IMXRT_IRQ_EXTINT + 192) /* Reserved */
#define IMXRT_IRQ_RESERVED193    (IMXRT_IRQ_EXTINT + 193) /* Reserved */
#define IMXRT_IRQ_RESERVED194    (IMXRT_IRQ_EXTINT + 194) /* Reserved */
#define IMXRT_IRQ_RESERVED195    (IMXRT_IRQ_EXTINT + 195) /* Reserved */
#define IMXRT_IRQ_RESERVED196    (IMXRT_IRQ_EXTINT + 196) /* Reserved */
#define IMXRT_IRQ_RESERVED197    (IMXRT_IRQ_EXTINT + 197) /* Reserved */
#define IMXRT_IRQ_RESERVED198    (IMXRT_IRQ_EXTINT + 198) /* Reserved */
#define IMXRT_IRQ_RESERVED199    (IMXRT_IRQ_EXTINT + 199) /* Reserved */
#define IMXRT_IRQ_PDM_1          (IMXRT_IRQ_EXTINT + 200) /* PDM HWVAD event interrupt */
#define IMXRT_IRQ_PDM_2          (IMXRT_IRQ_EXTINT + 201) /* PDM HWVAD error interrupt */
#define IMXRT_IRQ_PDM_3          (IMXRT_IRQ_EXTINT + 202) /* PDM filter interrupt */
#define IMXRT_IRQ_PDM_4          (IMXRT_IRQ_EXTINT + 203) /* PDM error interrupt */
#define IMXRT_IRQ_EMVSIM1        (IMXRT_IRQ_EXTINT + 204) /* EMVSIM1 interrupt */
#define IMXRT_IRQ_EMVSIM2        (IMXRT_IRQ_EXTINT + 205) /* EMVSIM2 interrupt */
#define IMXRT_IRQ_MECC1          (IMXRT_IRQ_EXTINT + 206) /* MECC1 interrupt */
#define IMXRT_IRQ_MECC1F         (IMXRT_IRQ_EXTINT + 207) /* MECC1 fatal interrupt */
#define IMXRT_IRQ_MECC2          (IMXRT_IRQ_EXTINT + 208) /* MECC2 interrupt */
#define IMXRT_IRQ_MECC2F         (IMXRT_IRQ_EXTINT + 209) /* MECC2 fatal interrupt */
#define IMXRT_IRQ_XECC_FLEXSPI1  (IMXRT_IRQ_EXTINT + 210) /* XECC_FLEXSPI1 interrupt */
#define IMXRT_IRQ_XECC_FLEXSPI1F (IMXRT_IRQ_EXTINT + 211) /* XECC_FLEXSPI1 fatal interrupt */
#define IMXRT_IRQ_XECC_FLEXSPI2  (IMXRT_IRQ_EXTINT + 212) /* XECC_FLEXSPI2 interrupt */
#define IMXRT_IRQ_XECC_FLEXSPI2F (IMXRT_IRQ_EXTINT + 213) /* XECC_FLEXSPI2 fatal interrupt */
#define IMXRT_IRQ_XECC_SEMC      (IMXRT_IRQ_EXTINT + 214) /* XECC_SEMC interrupt */
#define IMXRT_IRQ_XECC_SEMCF     (IMXRT_IRQ_EXTINT + 215) /* XECC_SEMC fatal interrupt */
#define IMXRT_IRQ_ENET3          (IMXRT_IRQ_EXTINT + 216) /* ENET_QOS interrupt */
#define IMXRT_IRQ_ENET3_PMT      (IMXRT_IRQ_EXTINT + 217) /* ENET_QOS PMT interrupt */

#define IMXRT_IRQ_NEXTINT        (218)

/* GPIO second level interrupt **********************************************/

#define IMXRT_GPIO_IRQ_FIRST     (IMXRT_IRQ_EXTINT + IMXRT_IRQ_NEXTINT)
#define _IMXRT_GPIO1_0_15_BASE   IMXRT_GPIO_IRQ_FIRST

/* TO DO: Check if this really works for all GPIO banks on i.MX RT117X
 * Also, not all banks have 32 pins.
 * CM7 GPIO2 and GPIO3 are not added yet.
 */

#ifdef CONFIG_IMXRT_GPIO1_0_15_IRQ
#  define IMXRT_IRQ_GPIO1_0      (_IMXRT_GPIO1_0_15_BASE + 0)   /* GPIO1 pin 0 interrupt */
#  define IMXRT_IRQ_GPIO1_1      (_IMXRT_GPIO1_0_15_BASE + 1)   /* GPIO1 pin 1 interrupt */
#  define IMXRT_IRQ_GPIO1_2      (_IMXRT_GPIO1_0_15_BASE + 2)   /* GPIO1 pin 2 interrupt */
#  define IMXRT_IRQ_GPIO1_3      (_IMXRT_GPIO1_0_15_BASE + 3)   /* GPIO1 pin 3 interrupt */
#  define IMXRT_IRQ_GPIO1_4      (_IMXRT_GPIO1_0_15_BASE + 4)   /* GPIO1 pin 4 interrupt */
#  define IMXRT_IRQ_GPIO1_5      (_IMXRT_GPIO1_0_15_BASE + 5)   /* GPIO1 pin 5 interrupt */
#  define IMXRT_IRQ_GPIO1_6      (_IMXRT_GPIO1_0_15_BASE + 6)   /* GPIO1 pin 6 interrupt */
#  define IMXRT_IRQ_GPIO1_7      (_IMXRT_GPIO1_0_15_BASE + 7)   /* GPIO1 pin 7 interrupt */
#  define IMXRT_IRQ_GPIO1_8      (_IMXRT_GPIO1_0_15_BASE + 8)   /* GPIO1 pin 8 interrupt */
#  define IMXRT_IRQ_GPIO1_9      (_IMXRT_GPIO1_0_15_BASE + 9)   /* GPIO1 pin 9 interrupt */
#  define IMXRT_IRQ_GPIO1_10     (_IMXRT_GPIO1_0_15_BASE + 10)  /* GPIO1 pin 10 interrupt */
#  define IMXRT_IRQ_GPIO1_11     (_IMXRT_GPIO1_0_15_BASE + 11)  /* GPIO1 pin 11 interrupt */
#  define IMXRT_IRQ_GPIO1_12     (_IMXRT_GPIO1_0_15_BASE + 12)  /* GPIO1 pin 12 interrupt */
#  define IMXRT_IRQ_GPIO1_13     (_IMXRT_GPIO1_0_15_BASE + 13)  /* GPIO1 pin 13 interrupt */
#  define IMXRT_IRQ_GPIO1_14     (_IMXRT_GPIO1_0_15_BASE + 14)  /* GPIO1 pin 14 interrupt */
#  define IMXRT_IRQ_GPIO1_15     (_IMXRT_GPIO1_0_15_BASE + 15)  /* GPIO1 pin 15 interrupt */

#  define _IMXRT_GPIO1_0_15_NIRQS 16
#  define _IMXRT_GPIO1_16_31_BASE (_IMXRT_GPIO1_0_15_BASE + _IMXRT_GPIO1_0_15_NIRQS)
#else
#  define _IMXRT_GPIO1_0_15_NIRQS 0
#  define _IMXRT_GPIO1_16_31_BASE _IMXRT_GPIO1_0_15_BASE
#endif

#ifdef CONFIG_IMXRT_GPIO1_16_31_IRQ
#  define IMXRT_IRQ_GPIO1_16     (_IMXRT_GPIO1_16_31_BASE + 0)   /* GPIO1 pin 16 interrupt */
#  define IMXRT_IRQ_GPIO1_17     (_IMXRT_GPIO1_16_31_BASE + 1)   /* GPIO1 pin 17 interrupt */
#  define IMXRT_IRQ_GPIO1_18     (_IMXRT_GPIO1_16_31_BASE + 2)   /* GPIO1 pin 18 interrupt */
#  define IMXRT_IRQ_GPIO1_19     (_IMXRT_GPIO1_16_31_BASE + 3)   /* GPIO1 pin 19 interrupt */
#  define IMXRT_IRQ_GPIO1_20     (_IMXRT_GPIO1_16_31_BASE + 4)   /* GPIO1 pin 10 interrupt */
#  define IMXRT_IRQ_GPIO1_21     (_IMXRT_GPIO1_16_31_BASE + 5)   /* GPIO1 pin 21 interrupt */
#  define IMXRT_IRQ_GPIO1_22     (_IMXRT_GPIO1_16_31_BASE + 6)   /* GPIO1 pin 22 interrupt */
#  define IMXRT_IRQ_GPIO1_23     (_IMXRT_GPIO1_16_31_BASE + 7)   /* GPIO1 pin 23 interrupt */
#  define IMXRT_IRQ_GPIO1_24     (_IMXRT_GPIO1_16_31_BASE + 8)   /* GPIO1 pin 24 interrupt */
#  define IMXRT_IRQ_GPIO1_25     (_IMXRT_GPIO1_16_31_BASE + 9)   /* GPIO1 pin 25 interrupt */
#  define IMXRT_IRQ_GPIO1_26     (_IMXRT_GPIO1_16_31_BASE + 10)  /* GPIO1 pin 26 interrupt */
#  define IMXRT_IRQ_GPIO1_27     (_IMXRT_GPIO1_16_31_BASE + 11)  /* GPIO1 pin 27 interrupt */
#  define IMXRT_IRQ_GPIO1_28     (_IMXRT_GPIO1_16_31_BASE + 12)  /* GPIO1 pin 28 interrupt */
#  define IMXRT_IRQ_GPIO1_29     (_IMXRT_GPIO1_16_31_BASE + 13)  /* GPIO1 pin 29 interrupt */
#  define IMXRT_IRQ_GPIO1_30     (_IMXRT_GPIO1_16_31_BASE + 14)  /* GPIO1 pin 30 interrupt */
#  define IMXRT_IRQ_GPIO1_31     (_IMXRT_GPIO1_16_31_BASE + 15)  /* GPIO1 pin 31 interrupt */

#  define _IMXRT_GPIO1_16_31_NIRQS 16
#  define _IMXRT_GPIO2_0_15_BASE (_IMXRT_GPIO1_16_31_BASE + _IMXRT_GPIO1_16_31_NIRQS)
#  define IMXRT_GPIO1_NIRQS      (_IMXRT_GPIO1_0_15_NIRQS + _IMXRT_GPIO1_16_31_NIRQS)
#else
#  define _IMXRT_GPIO2_0_15_BASE _IMXRT_GPIO1_16_31_BASE
#  define IMXRT_GPIO1_NIRQS      _IMXRT_GPIO1_0_15_NIRQS
#endif

#ifdef CONFIG_IMXRT_GPIO2_0_15_IRQ
#  define IMXRT_IRQ_GPIO2_0      (_IMXRT_GPIO2_0_15_BASE + 0)   /* GPIO2 pin 0 interrupt */
#  define IMXRT_IRQ_GPIO2_1      (_IMXRT_GPIO2_0_15_BASE + 1)   /* GPIO2 pin 1 interrupt */
#  define IMXRT_IRQ_GPIO2_2      (_IMXRT_GPIO2_0_15_BASE + 2)   /* GPIO2 pin 2 interrupt */
#  define IMXRT_IRQ_GPIO2_3      (_IMXRT_GPIO2_0_15_BASE + 3)   /* GPIO2 pin 3 interrupt */
#  define IMXRT_IRQ_GPIO2_4      (_IMXRT_GPIO2_0_15_BASE + 4)   /* GPIO2 pin 4 interrupt */
#  define IMXRT_IRQ_GPIO2_5      (_IMXRT_GPIO2_0_15_BASE + 5)   /* GPIO2 pin 5 interrupt */
#  define IMXRT_IRQ_GPIO2_6      (_IMXRT_GPIO2_0_15_BASE + 6)   /* GPIO2 pin 6 interrupt */
#  define IMXRT_IRQ_GPIO2_7      (_IMXRT_GPIO2_0_15_BASE + 7)   /* GPIO2 pin 7 interrupt */
#  define IMXRT_IRQ_GPIO2_8      (_IMXRT_GPIO2_0_15_BASE + 8)   /* GPIO2 pin 8 interrupt */
#  define IMXRT_IRQ_GPIO2_9      (_IMXRT_GPIO2_0_15_BASE + 9)   /* GPIO2 pin 9 interrupt */
#  define IMXRT_IRQ_GPIO2_10     (_IMXRT_GPIO2_0_15_BASE + 10)  /* GPIO2 pin 10 interrupt */
#  define IMXRT_IRQ_GPIO2_11     (_IMXRT_GPIO2_0_15_BASE + 11)  /* GPIO2 pin 11 interrupt */
#  define IMXRT_IRQ_GPIO2_12     (_IMXRT_GPIO2_0_15_BASE + 12)  /* GPIO2 pin 12 interrupt */
#  define IMXRT_IRQ_GPIO2_13     (_IMXRT_GPIO2_0_15_BASE + 13)  /* GPIO2 pin 13 interrupt */
#  define IMXRT_IRQ_GPIO2_14     (_IMXRT_GPIO2_0_15_BASE + 14)  /* GPIO2 pin 14 interrupt */
#  define IMXRT_IRQ_GPIO2_15     (_IMXRT_GPIO2_0_15_BASE + 15)  /* GPIO2 pin 15 interrupt */

#  define _IMXRT_GPIO2_0_15_NIRQS 16
#  define _IMXRT_GPIO2_16_31_BASE (_IMXRT_GPIO2_0_15_BASE + _IMXRT_GPIO2_0_15_NIRQS)
#else
#  define _IMXRT_GPIO2_0_15_NIRQS 0
#  define _IMXRT_GPIO2_16_31_BASE _IMXRT_GPIO2_0_15_BASE
#endif

#ifdef CONFIG_IMXRT_GPIO2_16_31_IRQ
#  define IMXRT_IRQ_GPIO2_16     (_IMXRT_GPIO2_16_31_BASE + 0)   /* GPIO2 pin 16 interrupt */
#  define IMXRT_IRQ_GPIO2_17     (_IMXRT_GPIO2_16_31_BASE + 1)   /* GPIO2 pin 17 interrupt */
#  define IMXRT_IRQ_GPIO2_18     (_IMXRT_GPIO2_16_31_BASE + 2)   /* GPIO2 pin 18 interrupt */
#  define IMXRT_IRQ_GPIO2_19     (_IMXRT_GPIO2_16_31_BASE + 3)   /* GPIO2 pin 19 interrupt */
#  define IMXRT_IRQ_GPIO2_20     (_IMXRT_GPIO2_16_31_BASE + 4)   /* GPIO2 pin 20 interrupt */
#  define IMXRT_IRQ_GPIO2_21     (_IMXRT_GPIO2_16_31_BASE + 5)   /* GPIO2 pin 21 interrupt */
#  define IMXRT_IRQ_GPIO2_22     (_IMXRT_GPIO2_16_31_BASE + 6)   /* GPIO2 pin 22 interrupt */
#  define IMXRT_IRQ_GPIO2_23     (_IMXRT_GPIO2_16_31_BASE + 7)   /* GPIO2 pin 23 interrupt */
#  define IMXRT_IRQ_GPIO2_24     (_IMXRT_GPIO2_16_31_BASE + 8)   /* GPIO2 pin 24 interrupt */
#  define IMXRT_IRQ_GPIO2_25     (_IMXRT_GPIO2_16_31_BASE + 9)   /* GPIO2 pin 25 interrupt */
#  define IMXRT_IRQ_GPIO2_26     (_IMXRT_GPIO2_16_31_BASE + 10)  /* GPIO2 pin 26 interrupt */
#  define IMXRT_IRQ_GPIO2_27     (_IMXRT_GPIO2_16_31_BASE + 11)  /* GPIO2 pin 27 interrupt */
#  define IMXRT_IRQ_GPIO2_28     (_IMXRT_GPIO2_16_31_BASE + 12)  /* GPIO2 pin 28 interrupt */
#  define IMXRT_IRQ_GPIO2_29     (_IMXRT_GPIO2_16_31_BASE + 13)  /* GPIO2 pin 29 interrupt */
#  define IMXRT_IRQ_GPIO2_30     (_IMXRT_GPIO2_16_31_BASE + 14)  /* GPIO2 pin 30 interrupt */
#  define IMXRT_IRQ_GPIO2_31     (_IMXRT_GPIO2_16_31_BASE + 15)  /* GPIO2 pin 31 interrupt */

#  define _IMXRT_GPIO2_16_31_NIRQS 16
#  define _IMXRT_GPIO3_0_15_BASE (_IMXRT_GPIO2_16_31_BASE + _IMXRT_GPIO2_16_31_NIRQS)
#  define IMXRT_GPIO2_NIRQS      (_IMXRT_GPIO2_0_15_NIRQS + _IMXRT_GPIO2_16_31_NIRQS)
#else
#  define _IMXRT_GPIO3_0_15_BASE _IMXRT_GPIO2_16_31_BASE
#  define IMXRT_GPIO2_NIRQS      _IMXRT_GPIO2_0_15_NIRQS
#endif

#ifdef CONFIG_IMXRT_GPIO3_0_15_IRQ
#  define IMXRT_IRQ_GPIO3_0      (_IMXRT_GPIO3_0_15_BASE + 0)   /* GPIO3 pin 0 interrupt */
#  define IMXRT_IRQ_GPIO3_1      (_IMXRT_GPIO3_0_15_BASE + 1)   /* GPIO3 pin 1 interrupt */
#  define IMXRT_IRQ_GPIO3_2      (_IMXRT_GPIO3_0_15_BASE + 2)   /* GPIO3 pin 2 interrupt */
#  define IMXRT_IRQ_GPIO3_3      (_IMXRT_GPIO3_0_15_BASE + 3)   /* GPIO3 pin 3 interrupt */
#  define IMXRT_IRQ_GPIO3_4      (_IMXRT_GPIO3_0_15_BASE + 4)   /* GPIO3 pin 4 interrupt */
#  define IMXRT_IRQ_GPIO3_5      (_IMXRT_GPIO3_0_15_BASE + 5)   /* GPIO3 pin 5 interrupt */
#  define IMXRT_IRQ_GPIO3_6      (_IMXRT_GPIO3_0_15_BASE + 6)   /* GPIO3 pin 6 interrupt */
#  define IMXRT_IRQ_GPIO3_7      (_IMXRT_GPIO3_0_15_BASE + 7)   /* GPIO3 pin 7 interrupt */
#  define IMXRT_IRQ_GPIO3_8      (_IMXRT_GPIO3_0_15_BASE + 8)   /* GPIO3 pin 8 interrupt */
#  define IMXRT_IRQ_GPIO3_9      (_IMXRT_GPIO3_0_15_BASE + 9)   /* GPIO3 pin 9 interrupt */
#  define IMXRT_IRQ_GPIO3_10     (_IMXRT_GPIO3_0_15_BASE + 10)  /* GPIO3 pin 10 interrupt */
#  define IMXRT_IRQ_GPIO3_11     (_IMXRT_GPIO3_0_15_BASE + 11)  /* GPIO3 pin 11 interrupt */
#  define IMXRT_IRQ_GPIO3_12     (_IMXRT_GPIO3_0_15_BASE + 12)  /* GPIO3 pin 12 interrupt */
#  define IMXRT_IRQ_GPIO3_13     (_IMXRT_GPIO3_0_15_BASE + 13)  /* GPIO3 pin 13 interrupt */
#  define IMXRT_IRQ_GPIO3_14     (_IMXRT_GPIO3_0_15_BASE + 14)  /* GPIO3 pin 14 interrupt */
#  define IMXRT_IRQ_GPIO3_15     (_IMXRT_GPIO3_0_15_BASE + 15)  /* GPIO3 pin 15 interrupt */

#  define _IMXRT_GPIO3_0_15_NIRQS 16
#  define _IMXRT_GPIO3_16_31_BASE (_IMXRT_GPIO3_0_15_BASE + _IMXRT_GPIO3_0_15_NIRQS)
#else
#  define _IMXRT_GPIO3_0_15_NIRQS 0
#  define _IMXRT_GPIO3_16_31_BASE _IMXRT_GPIO3_0_15_BASE
#endif

#ifdef CONFIG_IMXRT_GPIO3_16_31_IRQ
#  define IMXRT_IRQ_GPIO3_16     (_IMXRT_GPIO3_16_31_BASE + 0)   /* GPIO3 pin 16 interrupt */
#  define IMXRT_IRQ_GPIO3_17     (_IMXRT_GPIO3_16_31_BASE + 1)   /* GPIO3 pin 17 interrupt */
#  define IMXRT_IRQ_GPIO3_18     (_IMXRT_GPIO3_16_31_BASE + 2)   /* GPIO3 pin 18 interrupt */
#  define IMXRT_IRQ_GPIO3_19     (_IMXRT_GPIO3_16_31_BASE + 3)   /* GPIO3 pin 19 interrupt */
#  define IMXRT_IRQ_GPIO3_20     (_IMXRT_GPIO3_16_31_BASE + 4)   /* GPIO3 pin 20 interrupt */
#  define IMXRT_IRQ_GPIO3_21     (_IMXRT_GPIO3_16_31_BASE + 5)   /* GPIO3 pin 21 interrupt */
#  define IMXRT_IRQ_GPIO3_22     (_IMXRT_GPIO3_16_31_BASE + 6)   /* GPIO3 pin 22 interrupt */
#  define IMXRT_IRQ_GPIO3_23     (_IMXRT_GPIO3_16_31_BASE + 7)   /* GPIO3 pin 23 interrupt */
#  define IMXRT_IRQ_GPIO3_24     (_IMXRT_GPIO3_16_31_BASE + 8)   /* GPIO3 pin 24 interrupt */
#  define IMXRT_IRQ_GPIO3_25     (_IMXRT_GPIO3_16_31_BASE + 9)   /* GPIO3 pin 25 interrupt */
#  define IMXRT_IRQ_GPIO3_26     (_IMXRT_GPIO3_16_31_BASE + 10)  /* GPIO3 pin 26 interrupt */
#  define IMXRT_IRQ_GPIO3_27     (_IMXRT_GPIO3_16_31_BASE + 11)  /* GPIO3 pin 27 interrupt */
#  define IMXRT_IRQ_GPIO3_28     (_IMXRT_GPIO3_16_31_BASE + 12)  /* GPIO3 pin 28 interrupt */
#  define IMXRT_IRQ_GPIO3_29     (_IMXRT_GPIO3_16_31_BASE + 13)  /* GPIO3 pin 29 interrupt */
#  define IMXRT_IRQ_GPIO3_30     (_IMXRT_GPIO3_16_31_BASE + 14)  /* GPIO3 pin 30 interrupt */
#  define IMXRT_IRQ_GPIO3_31     (_IMXRT_GPIO3_16_31_BASE + 15)  /* GPIO3 pin 31 interrupt */

#  define _IMXRT_GPIO3_16_31_NIRQS 16
#  define _IMXRT_GPIO4_0_15_BASE (_IMXRT_GPIO3_16_31_BASE + _IMXRT_GPIO3_16_31_NIRQS)
#  define IMXRT_GPIO3_NIRQS      (_IMXRT_GPIO3_0_15_NIRQS + _IMXRT_GPIO3_16_31_NIRQS)
#else
#  define _IMXRT_GPIO4_0_15_BASE _IMXRT_GPIO3_16_31_BASE
#  define IMXRT_GPIO3_NIRQS      _IMXRT_GPIO3_0_15_NIRQS
#endif

#ifdef CONFIG_IMXRT_GPIO4_0_15_IRQ
#  define IMXRT_IRQ_GPIO4_0      (_IMXRT_GPIO4_0_15_BASE + 0)   /* GPIO4 pin 0 interrupt */
#  define IMXRT_IRQ_GPIO4_1      (_IMXRT_GPIO4_0_15_BASE + 1)   /* GPIO4 pin 1 interrupt */
#  define IMXRT_IRQ_GPIO4_2      (_IMXRT_GPIO4_0_15_BASE + 2)   /* GPIO4 pin 2 interrupt */
#  define IMXRT_IRQ_GPIO4_3      (_IMXRT_GPIO4_0_15_BASE + 3)   /* GPIO4 pin 3 interrupt */
#  define IMXRT_IRQ_GPIO4_4      (_IMXRT_GPIO4_0_15_BASE + 4)   /* GPIO4 pin 4 interrupt */
#  define IMXRT_IRQ_GPIO4_5      (_IMXRT_GPIO4_0_15_BASE + 5)   /* GPIO4 pin 5 interrupt */
#  define IMXRT_IRQ_GPIO4_6      (_IMXRT_GPIO4_0_15_BASE + 6)   /* GPIO4 pin 6 interrupt */
#  define IMXRT_IRQ_GPIO4_7      (_IMXRT_GPIO4_0_15_BASE + 7)   /* GPIO4 pin 7 interrupt */
#  define IMXRT_IRQ_GPIO4_8      (_IMXRT_GPIO4_0_15_BASE + 8)   /* GPIO4 pin 8 interrupt */
#  define IMXRT_IRQ_GPIO4_9      (_IMXRT_GPIO4_0_15_BASE + 9)   /* GPIO4 pin 9 interrupt */
#  define IMXRT_IRQ_GPIO4_10     (_IMXRT_GPIO4_0_15_BASE + 10)  /* GPIO4 pin 10 interrupt */
#  define IMXRT_IRQ_GPIO4_11     (_IMXRT_GPIO4_0_15_BASE + 11)  /* GPIO4 pin 11 interrupt */
#  define IMXRT_IRQ_GPIO4_12     (_IMXRT_GPIO4_0_15_BASE + 12)  /* GPIO4 pin 12 interrupt */
#  define IMXRT_IRQ_GPIO4_13     (_IMXRT_GPIO4_0_15_BASE + 13)  /* GPIO4 pin 13 interrupt */
#  define IMXRT_IRQ_GPIO4_14     (_IMXRT_GPIO4_0_15_BASE + 14)  /* GPIO4 pin 14 interrupt */
#  define IMXRT_IRQ_GPIO4_15     (_IMXRT_GPIO4_0_15_BASE + 15)  /* GPIO4 pin 15 interrupt */

#  define _IMXRT_GPIO4_0_15_NIRQS 16
#  define _IMXRT_GPIO4_16_31_BASE (_IMXRT_GPIO4_0_15_BASE + _IMXRT_GPIO4_0_15_NIRQS)
#else
#  define _IMXRT_GPIO4_0_15_NIRQS 0
#  define _IMXRT_GPIO4_16_31_BASE _IMXRT_GPIO4_0_15_BASE
#endif

#ifdef CONFIG_IMXRT_GPIO4_16_31_IRQ
#  define IMXRT_IRQ_GPIO4_16     (_IMXRT_GPIO4_16_31_BASE + 0)   /* GPIO4 pin 16 interrupt */
#  define IMXRT_IRQ_GPIO4_17     (_IMXRT_GPIO4_16_31_BASE + 1)   /* GPIO4 pin 17 interrupt */
#  define IMXRT_IRQ_GPIO4_18     (_IMXRT_GPIO4_16_31_BASE + 2)   /* GPIO4 pin 18 interrupt */
#  define IMXRT_IRQ_GPIO4_19     (_IMXRT_GPIO4_16_31_BASE + 3)   /* GPIO4 pin 19 interrupt */
#  define IMXRT_IRQ_GPIO4_20     (_IMXRT_GPIO4_16_31_BASE + 4)   /* GPIO4 pin 20 interrupt */
#  define IMXRT_IRQ_GPIO4_21     (_IMXRT_GPIO4_16_31_BASE + 5)   /* GPIO4 pin 21 interrupt */
#  define IMXRT_IRQ_GPIO4_22     (_IMXRT_GPIO4_16_31_BASE + 6)   /* GPIO4 pin 22 interrupt */
#  define IMXRT_IRQ_GPIO4_23     (_IMXRT_GPIO4_16_31_BASE + 7)   /* GPIO4 pin 23 interrupt */
#  define IMXRT_IRQ_GPIO4_24     (_IMXRT_GPIO4_16_31_BASE + 8)   /* GPIO4 pin 24 interrupt */
#  define IMXRT_IRQ_GPIO4_25     (_IMXRT_GPIO4_16_31_BASE + 9)   /* GPIO4 pin 25 interrupt */
#  define IMXRT_IRQ_GPIO4_26     (_IMXRT_GPIO4_16_31_BASE + 10)  /* GPIO4 pin 26 interrupt */
#  define IMXRT_IRQ_GPIO4_27     (_IMXRT_GPIO4_16_31_BASE + 11)  /* GPIO4 pin 27 interrupt */
#  define IMXRT_IRQ_GPIO4_28     (_IMXRT_GPIO4_16_31_BASE + 12)  /* GPIO4 pin 28 interrupt */
#  define IMXRT_IRQ_GPIO4_29     (_IMXRT_GPIO4_16_31_BASE + 13)  /* GPIO4 pin 29 interrupt */
#  define IMXRT_IRQ_GPIO4_30     (_IMXRT_GPIO4_16_31_BASE + 14)  /* GPIO4 pin 30 interrupt */
#  define IMXRT_IRQ_GPIO4_31     (_IMXRT_GPIO4_16_31_BASE + 15)  /* GPIO4 pin 31 interrupt */

#  define _IMXRT_GPIO4_16_31_NIRQS 16
#  define _IMXRT_GPIO5_0_15_BASE (_IMXRT_GPIO4_16_31_BASE + _IMXRT_GPIO4_16_31_NIRQS)
#  define IMXRT_GPIO4_NIRQS      (_IMXRT_GPIO4_0_15_NIRQS + _IMXRT_GPIO4_16_31_NIRQS)
#else
#  define _IMXRT_GPIO5_0_15_BASE _IMXRT_GPIO4_16_31_BASE
#  define IMXRT_GPIO4_NIRQS      _IMXRT_GPIO4_0_15_NIRQS
#endif

#ifdef CONFIG_IMXRT_GPIO5_0_15_IRQ
#  define IMXRT_IRQ_GPIO5_0      (_IMXRT_GPIO5_0_15_BASE + 0)   /* GPIO5 pin 0 interrupt */
#  define IMXRT_IRQ_GPIO5_1      (_IMXRT_GPIO5_0_15_BASE + 1)   /* GPIO5 pin 1 interrupt */
#  define IMXRT_IRQ_GPIO5_2      (_IMXRT_GPIO5_0_15_BASE + 2)   /* GPIO5 pin 2 interrupt */
#  define IMXRT_IRQ_GPIO5_3      (_IMXRT_GPIO5_0_15_BASE + 3)   /* GPIO5 pin 3 interrupt */
#  define IMXRT_IRQ_GPIO5_4      (_IMXRT_GPIO5_0_15_BASE + 4)   /* GPIO5 pin 4 interrupt */
#  define IMXRT_IRQ_GPIO5_5      (_IMXRT_GPIO5_0_15_BASE + 5)   /* GPIO5 pin 5 interrupt */
#  define IMXRT_IRQ_GPIO5_6      (_IMXRT_GPIO5_0_15_BASE + 6)   /* GPIO5 pin 6 interrupt */
#  define IMXRT_IRQ_GPIO5_7      (_IMXRT_GPIO5_0_15_BASE + 7)   /* GPIO5 pin 7 interrupt */
#  define IMXRT_IRQ_GPIO5_8      (_IMXRT_GPIO5_0_15_BASE + 8)   /* GPIO5 pin 8 interrupt */
#  define IMXRT_IRQ_GPIO5_9      (_IMXRT_GPIO5_0_15_BASE + 9)   /* GPIO5 pin 9 interrupt */
#  define IMXRT_IRQ_GPIO5_10     (_IMXRT_GPIO5_0_15_BASE + 10)  /* GPIO5 pin 10 interrupt */
#  define IMXRT_IRQ_GPIO5_11     (_IMXRT_GPIO5_0_15_BASE + 11)  /* GPIO5 pin 11 interrupt */
#  define IMXRT_IRQ_GPIO5_12     (_IMXRT_GPIO5_0_15_BASE + 12)  /* GPIO5 pin 12 interrupt */
#  define IMXRT_IRQ_GPIO5_13     (_IMXRT_GPIO5_0_15_BASE + 13)  /* GPIO5 pin 13 interrupt */
#  define IMXRT_IRQ_GPIO5_14     (_IMXRT_GPIO5_0_15_BASE + 14)  /* GPIO5 pin 14 interrupt */
#  define IMXRT_IRQ_GPIO5_15     (_IMXRT_GPIO5_0_15_BASE + 15)  /* GPIO5 pin 15 interrupt */

#  define _IMXRT_GPIO5_0_15_NIRQS 16
#  define _IMXRT_GPIO5_16_31_BASE (_IMXRT_GPIO5_0_15_BASE + _IMXRT_GPIO5_0_15_NIRQS)
#else
#  define _IMXRT_GPIO5_0_15_NIRQS 0
#  define _IMXRT_GPIO5_16_31_BASE _IMXRT_GPIO5_0_15_BASE
#endif

#ifdef CONFIG_IMXRT_GPIO5_16_31_IRQ
#  define IMXRT_IRQ_GPIO5_16     (_IMXRT_GPIO5_16_31_BASE + 0)   /* GPIO5 pin 16 interrupt */
#  define IMXRT_IRQ_GPIO5_17     (_IMXRT_GPIO5_16_31_BASE + 1)   /* GPIO5 pin 17 interrupt */
#  define IMXRT_IRQ_GPIO5_18     (_IMXRT_GPIO5_16_31_BASE + 2)   /* GPIO5 pin 18 interrupt */
#  define IMXRT_IRQ_GPIO5_19     (_IMXRT_GPIO5_16_31_BASE + 3)   /* GPIO5 pin 19 interrupt */
#  define IMXRT_IRQ_GPIO5_20     (_IMXRT_GPIO5_16_31_BASE + 4)   /* GPIO5 pin 20 interrupt */
#  define IMXRT_IRQ_GPIO5_21     (_IMXRT_GPIO5_16_31_BASE + 5)   /* GPIO5 pin 21 interrupt */
#  define IMXRT_IRQ_GPIO5_22     (_IMXRT_GPIO5_16_31_BASE + 6)   /* GPIO5 pin 22 interrupt */
#  define IMXRT_IRQ_GPIO5_23     (_IMXRT_GPIO5_16_31_BASE + 7)   /* GPIO5 pin 23 interrupt */
#  define IMXRT_IRQ_GPIO5_24     (_IMXRT_GPIO5_16_31_BASE + 8)   /* GPIO5 pin 24 interrupt */
#  define IMXRT_IRQ_GPIO5_25     (_IMXRT_GPIO5_16_31_BASE + 9)   /* GPIO5 pin 25 interrupt */
#  define IMXRT_IRQ_GPIO5_26     (_IMXRT_GPIO5_16_31_BASE + 10)  /* GPIO5 pin 26 interrupt */
#  define IMXRT_IRQ_GPIO5_27     (_IMXRT_GPIO5_16_31_BASE + 11)  /* GPIO5 pin 27 interrupt */
#  define IMXRT_IRQ_GPIO5_28     (_IMXRT_GPIO5_16_31_BASE + 12)  /* GPIO5 pin 28 interrupt */
#  define IMXRT_IRQ_GPIO5_29     (_IMXRT_GPIO5_16_31_BASE + 13)  /* GPIO5 pin 29 interrupt */
#  define IMXRT_IRQ_GPIO5_30     (_IMXRT_GPIO5_16_31_BASE + 14)  /* GPIO5 pin 30 interrupt */
#  define IMXRT_IRQ_GPIO5_31     (_IMXRT_GPIO5_16_31_BASE + 15)  /* GPIO5 pin 31 interrupt */

#  define _IMXRT_GPIO5_16_31_NIRQS 16
#  define _IMXRT_GPIO6_0_15_BASE (_IMXRT_GPIO5_16_31_BASE + _IMXRT_GPIO5_16_31_NIRQS)
#  define IMXRT_GPIO5_NIRQS      (_IMXRT_GPIO5_0_15_NIRQS + _IMXRT_GPIO5_16_31_NIRQS)
#else
#  define _IMXRT_GPIO6_0_15_BASE _IMXRT_GPIO5_16_31_BASE
#  define IMXRT_GPIO5_NIRQS      _IMXRT_GPIO5_0_15_NIRQS
#endif

#ifdef CONFIG_IMXRT_GPIO6_0_15_IRQ
#  define IMXRT_IRQ_GPIO6_0      (_IMXRT_GPIO6_0_15_BASE + 0)   /* GPIO6 pin 0 interrupt */
#  define IMXRT_IRQ_GPIO6_1      (_IMXRT_GPIO6_0_15_BASE + 1)   /* GPIO6 pin 1 interrupt */
#  define IMXRT_IRQ_GPIO6_2      (_IMXRT_GPIO6_0_15_BASE + 2)   /* GPIO6 pin 2 interrupt */
#  define IMXRT_IRQ_GPIO6_3      (_IMXRT_GPIO6_0_15_BASE + 3)   /* GPIO6 pin 3 interrupt */
#  define IMXRT_IRQ_GPIO6_4      (_IMXRT_GPIO6_0_15_BASE + 4)   /* GPIO6 pin 4 interrupt */
#  define IMXRT_IRQ_GPIO6_5      (_IMXRT_GPIO6_0_15_BASE + 5)   /* GPIO6 pin 5 interrupt */
#  define IMXRT_IRQ_GPIO6_6      (_IMXRT_GPIO6_0_15_BASE + 6)   /* GPIO6 pin 6 interrupt */
#  define IMXRT_IRQ_GPIO6_7      (_IMXRT_GPIO6_0_15_BASE + 7)   /* GPIO6 pin 7 interrupt */
#  define IMXRT_IRQ_GPIO6_8      (_IMXRT_GPIO6_0_15_BASE + 8)   /* GPIO6 pin 8 interrupt */
#  define IMXRT_IRQ_GPIO6_9      (_IMXRT_GPIO6_0_15_BASE + 9)   /* GPIO6 pin 9 interrupt */
#  define IMXRT_IRQ_GPIO6_10     (_IMXRT_GPIO6_0_15_BASE + 10)  /* GPIO6 pin 10 interrupt */
#  define IMXRT_IRQ_GPIO6_11     (_IMXRT_GPIO6_0_15_BASE + 11)  /* GPIO6 pin 11 interrupt */
#  define IMXRT_IRQ_GPIO6_12     (_IMXRT_GPIO6_0_15_BASE + 12)  /* GPIO6 pin 12 interrupt */
#  define IMXRT_IRQ_GPIO6_13     (_IMXRT_GPIO6_0_15_BASE + 13)  /* GPIO6 pin 13 interrupt */
#  define IMXRT_IRQ_GPIO6_14     (_IMXRT_GPIO6_0_15_BASE + 14)  /* GPIO6 pin 14 interrupt */
#  define IMXRT_IRQ_GPIO6_15     (_IMXRT_GPIO6_0_15_BASE + 15)  /* GPIO6 pin 15 interrupt */

#  define _IMXRT_GPIO6_0_15_NIRQS 16
#  define _IMXRT_GPIO6_16_31_BASE (_IMXRT_GPIO6_0_15_BASE + _IMXRT_GPIO6_0_15_NIRQS)
#else
#  define _IMXRT_GPIO6_0_15_NIRQS 0
#  define _IMXRT_GPIO6_16_31_BASE _IMXRT_GPIO6_0_15_BASE
#endif

#ifdef CONFIG_IMXRT_GPIO6_16_31_IRQ
#  define IMXRT_IRQ_GPIO6_16     (_IMXRT_GPIO6_16_31_BASE + 0)   /* GPIO6 pin 16 interrupt */
#  define IMXRT_IRQ_GPIO6_17     (_IMXRT_GPIO6_16_31_BASE + 1)   /* GPIO6 pin 17 interrupt */
#  define IMXRT_IRQ_GPIO6_18     (_IMXRT_GPIO6_16_31_BASE + 2)   /* GPIO6 pin 18 interrupt */
#  define IMXRT_IRQ_GPIO6_19     (_IMXRT_GPIO6_16_31_BASE + 3)   /* GPIO6 pin 19 interrupt */
#  define IMXRT_IRQ_GPIO6_20     (_IMXRT_GPIO6_16_31_BASE + 4)   /* GPIO6 pin 20 interrupt */
#  define IMXRT_IRQ_GPIO6_21     (_IMXRT_GPIO6_16_31_BASE + 5)   /* GPIO6 pin 21 interrupt */
#  define IMXRT_IRQ_GPIO6_22     (_IMXRT_GPIO6_16_31_BASE + 6)   /* GPIO6 pin 22 interrupt */
#  define IMXRT_IRQ_GPIO6_23     (_IMXRT_GPIO6_16_31_BASE + 7)   /* GPIO6 pin 23 interrupt */
#  define IMXRT_IRQ_GPIO6_24     (_IMXRT_GPIO6_16_31_BASE + 8)   /* GPIO6 pin 24 interrupt */
#  define IMXRT_IRQ_GPIO6_25     (_IMXRT_GPIO6_16_31_BASE + 9)   /* GPIO6 pin 25 interrupt */
#  define IMXRT_IRQ_GPIO6_26     (_IMXRT_GPIO6_16_31_BASE + 10)  /* GPIO6 pin 26 interrupt */
#  define IMXRT_IRQ_GPIO6_27     (_IMXRT_GPIO6_16_31_BASE + 11)  /* GPIO6 pin 27 interrupt */
#  define IMXRT_IRQ_GPIO6_28     (_IMXRT_GPIO6_16_31_BASE + 12)  /* GPIO6 pin 28 interrupt */
#  define IMXRT_IRQ_GPIO6_29     (_IMXRT_GPIO6_16_31_BASE + 13)  /* GPIO6 pin 29 interrupt */
#  define IMXRT_IRQ_GPIO6_30     (_IMXRT_GPIO6_16_31_BASE + 14)  /* GPIO6 pin 30 interrupt */
#  define IMXRT_IRQ_GPIO6_31     (_IMXRT_GPIO6_16_31_BASE + 15)  /* GPIO6 pin 31 interrupt */

#  define _IMXRT_GPIO6_16_31_NIRQS 16
#  define _IMXRT_GPIO13_BASE     (_IMXRT_GPIO6_16_31_BASE + _IMXRT_GPIO6_16_31_NIRQS)
#  define IMXRT_GPIO6_NIRQS      (_IMXRT_GPIO6_0_15_NIRQS + _IMXRT_GPIO6_16_31_NIRQS)
#else
#  define _IMXRT_GPIO13_BASE     _IMXRT_GPIO6_16_31_BASE
#  define IMXRT_GPIO6_NIRQS      _IMXRT_GPIO6_0_15_NIRQS
#endif

#if defined(CONFIG_IMXRT_GPIO7_0_15_IRQ) || defined(CONFIG_IMXRT_GPIO7_16_31_IRQ)
#  warning GPIO7 has no IRQ support on i.MX RT117X
#endif

#if defined(CONFIG_IMXRT_GPIO8_0_15_IRQ) || defined(CONFIG_IMXRT_GPIO8_16_31_IRQ)
#  warning GPIO8 has no IRQ support on i.MX RT117X
#endif

#if defined(CONFIG_IMXRT_GPIO9_0_15_IRQ) || defined(CONFIG_IMXRT_GPIO9_16_31_IRQ)
#  warning GPIO9 has no IRQ support on i.MX RT117X
#endif

#ifdef CONFIG_IMXRT_GPIO13_IRQ
#  define IMXRT_IRQ_GPIO13_0     (_IMXRT_GPIO13_BASE + 0)   /* GPIO13 pin 0 interrupt */
#  define IMXRT_IRQ_GPIO13_1     (_IMXRT_GPIO13_BASE + 1)   /* GPIO13 pin 1 interrupt */
#  define IMXRT_IRQ_GPIO13_2     (_IMXRT_GPIO13_BASE + 2)   /* GPIO13 pin 2 interrupt */
#  define IMXRT_IRQ_GPIO13_3     (_IMXRT_GPIO13_BASE + 3)   /* GPIO13 pin 3 interrupt */
#  define IMXRT_IRQ_GPIO13_4     (_IMXRT_GPIO13_BASE + 4)   /* GPIO13 pin 4 interrupt */
#  define IMXRT_IRQ_GPIO13_5     (_IMXRT_GPIO13_BASE + 5)   /* GPIO13 pin 5 interrupt */
#  define IMXRT_IRQ_GPIO13_6     (_IMXRT_GPIO13_BASE + 6)   /* GPIO13 pin 6 interrupt */
#  define IMXRT_IRQ_GPIO13_7     (_IMXRT_GPIO13_BASE + 7)   /* GPIO13 pin 7 interrupt */
#  define IMXRT_IRQ_GPIO13_8     (_IMXRT_GPIO13_BASE + 8)   /* GPIO13 pin 8 interrupt */
#  define IMXRT_IRQ_GPIO13_9     (_IMXRT_GPIO13_BASE + 9)   /* GPIO13 pin 9 interrupt */
#  define IMXRT_IRQ_GPIO13_10    (_IMXRT_GPIO13_BASE + 10)  /* GPIO13 pin 10 interrupt */
#  define IMXRT_IRQ_GPIO13_11    (_IMXRT_GPIO13_BASE + 11)  /* GPIO13 pin 11 interrupt */
#  define IMXRT_IRQ_GPIO13_12    (_IMXRT_GPIO13_BASE + 12)  /* GPIO13 pin 12 interrupt */
#  define IMXRT_IRQ_GPIO13_13    (_IMXRT_GPIO13_BASE + 13)  /* GPIO13 pin 13 interrupt */
#  define IMXRT_IRQ_GPIO13_14    (_IMXRT_GPIO13_BASE + 14)  /* GPIO13 pin 14 interrupt */
#  define IMXRT_IRQ_GPIO13_15    (_IMXRT_GPIO13_BASE + 15)  /* GPIO13 pin 15 interrupt */
#  define IMXRT_IRQ_GPIO13_16    (_IMXRT_GPIO13_BASE + 16)  /* GPIO13 pin 16 interrupt */
#  define IMXRT_IRQ_GPIO13_17    (_IMXRT_GPIO13_BASE + 17)  /* GPIO13 pin 17 interrupt */
#  define IMXRT_IRQ_GPIO13_18    (_IMXRT_GPIO13_BASE + 18)  /* GPIO13 pin 18 interrupt */
#  define IMXRT_IRQ_GPIO13_19    (_IMXRT_GPIO13_BASE + 19)  /* GPIO13 pin 19 interrupt */
#  define IMXRT_IRQ_GPIO13_20    (_IMXRT_GPIO13_BASE + 20)  /* GPIO13 pin 20 interrupt */
#  define IMXRT_IRQ_GPIO13_21    (_IMXRT_GPIO13_BASE + 21)  /* GPIO13 pin 21 interrupt */
#  define IMXRT_IRQ_GPIO13_22    (_IMXRT_GPIO13_BASE + 22)  /* GPIO13 pin 22 interrupt */
#  define IMXRT_IRQ_GPIO13_23    (_IMXRT_GPIO13_BASE + 23)  /* GPIO13 pin 23 interrupt */
#  define IMXRT_IRQ_GPIO13_24    (_IMXRT_GPIO13_BASE + 24)  /* GPIO13 pin 24 interrupt */
#  define IMXRT_IRQ_GPIO13_25    (_IMXRT_GPIO13_BASE + 25)  /* GPIO13 pin 25 interrupt */
#  define IMXRT_IRQ_GPIO13_26    (_IMXRT_GPIO13_BASE + 26)  /* GPIO13 pin 26 interrupt */
#  define IMXRT_IRQ_GPIO13_27    (_IMXRT_GPIO13_BASE + 27)  /* GPIO13 pin 27 interrupt */
#  define IMXRT_IRQ_GPIO13_28    (_IMXRT_GPIO13_BASE + 28)  /* GPIO13 pin 28 interrupt */
#  define IMXRT_IRQ_GPIO13_29    (_IMXRT_GPIO13_BASE + 29)  /* GPIO13 pin 29 interrupt */
#  define IMXRT_IRQ_GPIO13_30    (_IMXRT_GPIO13_BASE + 30)  /* GPIO13 pin 30 interrupt */
#  define IMXRT_IRQ_GPIO13_31    (_IMXRT_GPIO13_BASE + 31)  /* GPIO13 pin 31 interrupt */

#  define IMXRT_GPIO13_NIRQS     32
#else
#  define IMXRT_GPIO13_NIRQS     0
#endif

#define IMXRT_GPIO_NIRQS         (IMXRT_GPIO1_NIRQS + IMXRT_GPIO2_NIRQS + \
                                  IMXRT_GPIO3_NIRQS + IMXRT_GPIO4_NIRQS + \
                                  IMXRT_GPIO5_NIRQS + IMXRT_GPIO6_NIRQS + \
                                  IMXRT_GPIO13_NIRQS)
#define IMXRT_GPIO_IRQ_LAST      (_IMXRT_GPIO1_0_15_BASE + IMXRT_GPIO_NIRQS)

/* Total number of IRQ numbers **********************************************/

#define NR_IRQS                  (IMXRT_IRQ_EXTINT + IMXRT_IRQ_NEXTINT + IMXRT_GPIO_NIRQS)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_IMXRT_IMXRT117X_IRQ_H */
