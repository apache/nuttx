/****************************************************************************************
 * arch/arm/include/imxrt/imxrt106x_irq.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
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
 ****************************************************************************************/

/* This file should never be included directly but, rather, only indirectly through
 * nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_IMXRT_IMXRT106X_IRQ_H
#define __ARCH_ARM_INCLUDE_IMXRT_IMXRT106X_IRQ_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* External interrupts (priority levels >= 256) *****************************************/

#define IMXRT_IRQ_EDMA0_16     (IMXRT_IRQ_EXTINT + 0)   /* eDMA Channel 0/16 Transfer Complete */
#define IMXRT_IRQ_EDMA1_17     (IMXRT_IRQ_EXTINT + 1)   /* eDMA Channel 1/17 Transfer Complete */
#define IMXRT_IRQ_EDMA2_18     (IMXRT_IRQ_EXTINT + 2)   /* eDMA Channel 2/18 Transfer Complete */
#define IMXRT_IRQ_EDMA3_19     (IMXRT_IRQ_EXTINT + 3)   /* eDMA Channel 3/19 Transfer Complete */
#define IMXRT_IRQ_EDMA4_20     (IMXRT_IRQ_EXTINT + 4)   /* eDMA Channel 4/20 Transfer Complete */
#define IMXRT_IRQ_EDMA5_21     (IMXRT_IRQ_EXTINT + 5)   /* eDMA Channel 5/21 Transfer Complete */
#define IMXRT_IRQ_EDMA6_22     (IMXRT_IRQ_EXTINT + 6)   /* eDMA Channel 6/22 Transfer Complete */
#define IMXRT_IRQ_EDMA7_23     (IMXRT_IRQ_EXTINT + 7)   /* eDMA Channel 7/23 Transfer Complete */
#define IMXRT_IRQ_EDMA8_24     (IMXRT_IRQ_EXTINT + 8)   /* eDMA Channel 8/24 Transfer Complete */
#define IMXRT_IRQ_EDMA9_25     (IMXRT_IRQ_EXTINT + 9)   /* eDMA Channel 9/25 Transfer Complete */
#define IMXRT_IRQ_EDMA10_26    (IMXRT_IRQ_EXTINT + 10)  /* eDMA Channel 10/26 Transfer Complete */
#define IMXRT_IRQ_EDMA11_27    (IMXRT_IRQ_EXTINT + 11)  /* eDMA Channel 11/27 Transfer Complete */
#define IMXRT_IRQ_EDMA12_28    (IMXRT_IRQ_EXTINT + 12)  /* eDMA Channel 12/28 Transfer Complete */
#define IMXRT_IRQ_EDMA13_29    (IMXRT_IRQ_EXTINT + 13)  /* eDMA Channel 13/29 Transfer Complete */
#define IMXRT_IRQ_EDMA14_30    (IMXRT_IRQ_EXTINT + 14)  /* eDMA Channel 14/30 Transfer Complete */
#define IMXRT_IRQ_EDMA15_31    (IMXRT_IRQ_EXTINT + 15)  /* eDMA Channel 15/31 Transfer Complete */
#define IMXRT_IRQ_EDMA_ERROR   (IMXRT_IRQ_EXTINT + 16)  /* Error Interrupt, Channels 0-15 / 16-31 */
#define IMXRT_IRQ_CM70         (IMXRT_IRQ_EXTINT + 17)  /* CTI trigger outputs (internal: CTIIRQ[0]) */
#define IMXRT_IRQ_CM71         (IMXRT_IRQ_EXTINT + 18)  /* CTI trigger outputs (internal: CTIIRQ[1]) */
#define IMXRT_IRQ_CM7CP        (IMXRT_IRQ_EXTINT + 19)  /* CorePlatform exception IRQ */
#define IMXRT_IRQ_LPUART1      (IMXRT_IRQ_EXTINT + 20)  /* UART1 TX/RX interrupt */
#define IMXRT_IRQ_LPUART2      (IMXRT_IRQ_EXTINT + 21)  /* UART2 TX/RX interrupt */
#define IMXRT_IRQ_LPUART3      (IMXRT_IRQ_EXTINT + 22)  /* UART3 TX/RX interrupt */
#define IMXRT_IRQ_LPUART4      (IMXRT_IRQ_EXTINT + 23)  /* UART4 TX/RX interrupt */
#define IMXRT_IRQ_LPUART5      (IMXRT_IRQ_EXTINT + 24)  /* UART5 TX/RX interrupt */
#define IMXRT_IRQ_LPUART6      (IMXRT_IRQ_EXTINT + 25)  /* UART6 TX/RX interrupt */
#define IMXRT_IRQ_LPUART7      (IMXRT_IRQ_EXTINT + 26)  /* UART7 TX/RX interrupt */
#define IMXRT_IRQ_LPUART8      (IMXRT_IRQ_EXTINT + 27)  /* UART8 TX/RX interrupt */
#define IMXRT_IRQ_LPI2C1       (IMXRT_IRQ_EXTINT + 28)  /* I2C1 Interrupt */
#define IMXRT_IRQ_LPI2C2       (IMXRT_IRQ_EXTINT + 29)  /* I2C2 Interrupt */
#define IMXRT_IRQ_LPI2C3       (IMXRT_IRQ_EXTINT + 30)  /* I2C3 Interrupt */
#define IMXRT_IRQ_LPI2C4       (IMXRT_IRQ_EXTINT + 31)  /* I2C4 Interrupt */
#define IMXRT_IRQ_LPSPI1       (IMXRT_IRQ_EXTINT + 32)  /* LPSPI1 interrupt */
#define IMXRT_IRQ_LPSPI2       (IMXRT_IRQ_EXTINT + 33)  /* LPSPI2 interrupt */
#define IMXRT_IRQ_LPSPI3       (IMXRT_IRQ_EXTINT + 34)  /* LPSPI3 interrupt */
#define IMXRT_IRQ_LPSPI4       (IMXRT_IRQ_EXTINT + 35)  /* LPSPI4 interrupt */
#define IMXRT_IRQ_CAN1         (IMXRT_IRQ_EXTINT + 36)  /* CAN1 interrupt */
#define IMXRT_IRQ_CAN2         (IMXRT_IRQ_EXTINT + 37)  /* CAN2 interrupt */
#define IMXRT_IRQ_CM7FR        (IMXRT_IRQ_EXTINT + 38)  /* FlexRAM address fault */
#define IMXRT_IRQ_KPP          (IMXRT_IRQ_EXTINT + 39)  /* Keypad Interrupt */
#define IMXRT_IRQ_TSCDIG       (IMXRT_IRQ_EXTINT + 40)  /* TSC interrupt */
#define IMXRT_IRQ_GPRIRQ       (IMXRT_IRQ_EXTINT + 41)  /* Notify cores on exception while boot */
#define IMXRT_IRQ_LCDIF        (IMXRT_IRQ_EXTINT + 42)  /* LCDIF Sync Interrupt */
#define IMXRT_IRQ_CSI          (IMXRT_IRQ_EXTINT + 43)  /* CSI interrupt */
#define IMXRT_IRQ_PXP          (IMXRT_IRQ_EXTINT + 44)  /* PXP interrupt */
#define IMXRT_IRQ_WDOG2        (IMXRT_IRQ_EXTINT + 45)  /* Watchdog Timer reset */
#define IMXRT_IRQ_SNVS         (IMXRT_IRQ_EXTINT + 46)  /* SNVS Functional Interrupt */
#define IMXRT_IRQ_SNVSSEC      (IMXRT_IRQ_EXTINT + 47)  /* SNVS Security Interrupt */
#define IMXRT_IRQ_SNVSSB       (IMXRT_IRQ_EXTINT + 48)  /* ON-OFF short button press */
#define IMXRT_IRQ_CSU          (IMXRT_IRQ_EXTINT + 49)  /* CSU Interrupt Request 1 */
#define IMXRT_IRQ_DCP          (IMXRT_IRQ_EXTINT + 50)  /* DCP channel/CRC interrupts (channel != 0) */
#define IMXRT_IRQ_DCP0         (IMXRT_IRQ_EXTINT + 51)  /* DCP channel 0 interrupt */
#define IMXRT_IRQ_RESERVED52   (IMXRT_IRQ_EXTINT + 52)  /* Reserved */
#define IMXRT_IRQ_TRNG         (IMXRT_IRQ_EXTINT + 53)  /* TRNG Interrupt */
#define IMXRT_IRQ_SJC          (IMXRT_IRQ_EXTINT + 54)  /* SJC Interrupt from General Purpose register */
#define IMXRT_IRQ_BEE          (IMXRT_IRQ_EXTINT + 55)  /* BEE IRQ */
#define IMXRT_IRQ_SAI1         (IMXRT_IRQ_EXTINT + 56)  /* SAI1 interrupt */
#define IMXRT_IRQ_SAI2         (IMXRT_IRQ_EXTINT + 57)  /* SAI2 interrupt */
#define IMXRT_IRQ_SAI3RX       (IMXRT_IRQ_EXTINT + 58)  /* SAI3 RX interrupt */
#define IMXRT_IRQ_SAI3TX       (IMXRT_IRQ_EXTINT + 59)  /* SAI3 TX interrupt */
#define IMXRT_IRQ_SPDIF        (IMXRT_IRQ_EXTINT + 60)  /* SPDIF interrupt */
#define IMXRT_IRQ_PMU          (IMXRT_IRQ_EXTINT + 61)  /* Brown-out event 1.1, 2.5 or 3.0 regulators */
#define IMXRT_IRQ_RESERVED62   (IMXRT_IRQ_EXTINT + 62)  /* Reserved */
#define IMXRT_IRQ_TEMP         (IMXRT_IRQ_EXTINT + 63)  /* Temperature Monitor */
#define IMXRT_IRQ_TEMPPANIC    (IMXRT_IRQ_EXTINT + 64)  /* TempSensor panic */
#define IMXRT_IRQ_USBPHY0      (IMXRT_IRQ_EXTINT + 65)  /* USBPHY (UTMI0) interrupt */
#define IMXRT_IRQ_USBPHY1      (IMXRT_IRQ_EXTINT + 66)  /* USBPHY (UTMI1) interrupt */
#define IMXRT_IRQ_ADC1         (IMXRT_IRQ_EXTINT + 67)  /* ADC1 interrupt */
#define IMXRT_IRQ_ADC2         (IMXRT_IRQ_EXTINT + 68)  /* ADC2 interrupt */
#define IMXRT_IRQ_DCDC         (IMXRT_IRQ_EXTINT + 69)  /* DCDC interrupt */
#define IMXRT_IRQ_RESERVED70   (IMXRT_IRQ_EXTINT + 70)  /* Reserved */
#define IMXRT_IRQ_RESERVED71   (IMXRT_IRQ_EXTINT + 71)  /* Reserved */
#define IMXRT_IRQ_GPIO1_0      (IMXRT_IRQ_EXTINT + 72)  /* GPIO1 INT0 interrupt */
#define IMXRT_IRQ_GPIO1_1      (IMXRT_IRQ_EXTINT + 73)  /* GPIO1 INT1 interrupt */
#define IMXRT_IRQ_GPIO1_2      (IMXRT_IRQ_EXTINT + 74)  /* GPIO1 INT2 interrupt */
#define IMXRT_IRQ_GPIO1_3      (IMXRT_IRQ_EXTINT + 75)  /* GPIO1 INT3 interrupt */
#define IMXRT_IRQ_GPIO1_4      (IMXRT_IRQ_EXTINT + 76)  /* GPIO1 INT4 interrupt */
#define IMXRT_IRQ_GPIO1_5      (IMXRT_IRQ_EXTINT + 77)  /* GPIO1 INT5 interrupt */
#define IMXRT_IRQ_GPIO1_6      (IMXRT_IRQ_EXTINT + 78)  /* GPIO1 INT6 interrupt */
#define IMXRT_IRQ_GPIO1_7      (IMXRT_IRQ_EXTINT + 79)  /* GPIO1 INT7 interrupt */
#define IMXRT_IRQ_GPIO1_0_15   (IMXRT_IRQ_EXTINT + 80)  /* GPIO1 INT0-15 interrupt */
#define IMXRT_IRQ_GPIO1_16_31  (IMXRT_IRQ_EXTINT + 81)  /* GPIO1 INT16-31 interrupt */
#define IMXRT_IRQ_GPIO2_0_15   (IMXRT_IRQ_EXTINT + 82)  /* GPIO2 INT0-15 interrupt */
#define IMXRT_IRQ_GPIO2_16_31  (IMXRT_IRQ_EXTINT + 83)  /* GPIO2 INT16-31 interrupt */
#define IMXRT_IRQ_GPIO3_0_15   (IMXRT_IRQ_EXTINT + 84)  /* GPIO3 INT0-15 interrupt */
#define IMXRT_IRQ_GPIO3_16_31  (IMXRT_IRQ_EXTINT + 85)  /* GPIO3 INT16-31 interrupt */
#define IMXRT_IRQ_GPIO4_0_15   (IMXRT_IRQ_EXTINT + 86)  /* GPIO4 INT0-15 interrupt */
#define IMXRT_IRQ_GPIO4_16_31  (IMXRT_IRQ_EXTINT + 87)  /* GPIO4 INT16-31 interrupt */
#define IMXRT_IRQ_GPIO5_0_15   (IMXRT_IRQ_EXTINT + 88)  /* GPIO5 INT0-15 interrupt */
#define IMXRT_IRQ_GPIO5_16_31  (IMXRT_IRQ_EXTINT + 89)  /* GPIO5 INT16-31 interrupt */
#define IMXRT_IRQ_FLEXIO1      (IMXRT_IRQ_EXTINT + 90)  /* IPI compare interrupt */
#define IMXRT_IRQ_FLEXIO2      (IMXRT_IRQ_EXTINT + 91)  /* IPI compare interrupt */
#define IMXRT_IRQ_WDOG1        (IMXRT_IRQ_EXTINT + 92)  /* Watchdog Timer reset */
#define IMXRT_IRQ_RTWDOG       (IMXRT_IRQ_EXTINT + 93)  /* Watchdog Timer reset */
#define IMXRT_IRQ_EWM          (IMXRT_IRQ_EXTINT + 94)  /* EWM interrupt */
#define IMXRT_IRQ_CCM_1        (IMXRT_IRQ_EXTINT + 95)  /* CCM interrupt 1 */
#define IMXRT_IRQ_CCM_2        (IMXRT_IRQ_EXTINT + 96)  /* CCM interrupt 2 */
#define IMXRT_IRQ_GPC          (IMXRT_IRQ_EXTINT + 97)  /* GPC interrupt 1 */
#define IMXRT_IRQ_SRC          (IMXRT_IRQ_EXTINT + 98)  /* SRC interrupt */
#define IMXRT_IRQ_RESERVED99   (IMXRT_IRQ_EXTINT + 99)  /* Reserved */
#define IMXRT_IRQ_GPT1         (IMXRT_IRQ_EXTINT + 100) /* GPT1 interrupt */
#define IMXRT_IRQ_GPT2         (IMXRT_IRQ_EXTINT + 101) /*  GPT2 interrupt */
#define IMXRT_IRQ_FLEXPWM1_0   (IMXRT_IRQ_EXTINT + 102) /* FLEXPWM1 capture/compare/reload 0 interrupt */
#define IMXRT_IRQ_FLEXPWM1_1   (IMXRT_IRQ_EXTINT + 103) /* FLEXPWM1 capture/compare/reload 1 interrupt */
#define IMXRT_IRQ_FLEXPWM1_2   (IMXRT_IRQ_EXTINT + 104) /* FLEXPWM1 capture/compare/reload 2 interrupt */
#define IMXRT_IRQ_FLEXPWM1_3   (IMXRT_IRQ_EXTINT + 105) /* FLEXPWM1 capture/compare/reload 3 interrupt */
#define IMXRT_IRQ_FLEXPWM1_F   (IMXRT_IRQ_EXTINT + 106) /* FLEXPWM1 fault interrupt */
#define IMXRT_IRQ_FLEXSPI2     (IMXRT_IRQ_EXTINT + 107) /* FlexSPI2 interrupt */
#define IMXRT_IRQ_FLEXSPI      (IMXRT_IRQ_EXTINT + 108) /* FlexSPI interrupt */
#define IMXRT_IRQ_SEMC         (IMXRT_IRQ_EXTINT + 109) /* SEMC interrupt */
#define IMXRT_IRQ_USDHC1       (IMXRT_IRQ_EXTINT + 110) /* USDHC1 interrupt */
#define IMXRT_IRQ_USDHC2       (IMXRT_IRQ_EXTINT + 111) /* USDHC2 interrupt */
#define IMXRT_IRQ_USBOTG2      (IMXRT_IRQ_EXTINT + 112) /* USBO2 USB OTG2 interrupt */
#define IMXRT_IRQ_USBOTG1      (IMXRT_IRQ_EXTINT + 113) /* USBO2 USB OTG1 interrupt */
#define IMXRT_IRQ_ENET         (IMXRT_IRQ_EXTINT + 114) /* ENET MAC 0 interrupt */
#define IMXRT_IRQ_ENET1588     (IMXRT_IRQ_EXTINT + 115) /* ENET MAC 0 1588 Timer Interrupt */
#define IMXRT_IRQ_XBAR1_0_1    (IMXRT_IRQ_EXTINT + 116) /* XBAR1 interrupt 0/1 */
#define IMXRT_IRQ_XBAR1_2_3    (IMXRT_IRQ_EXTINT + 117) /* XBAR1 interrupt 2/3 */
#define IMXRT_IRQ_ADCETC_0     (IMXRT_IRQ_EXTINT + 118) /* ADC_ETC interrupt 0 */
#define IMXRT_IRQ_ADCETC_1     (IMXRT_IRQ_EXTINT + 119) /* ADC_ETC interrupt 1 */
#define IMXRT_IRQ_ADCETC_2     (IMXRT_IRQ_EXTINT + 120) /* ADC_ETC interrupt 2 */
#define IMXRT_IRQ_ADCETC_ERR   (IMXRT_IRQ_EXTINT + 121) /* ADC_ETC error interrupt */
#define IMXRT_IRQ_PIT          (IMXRT_IRQ_EXTINT + 122) /* PIT interrupt */
#define IMXRT_IRQ_ACMP1        (IMXRT_IRQ_EXTINT + 123) /* ACMP1 interrupt */
#define IMXRT_IRQ_ACMP2        (IMXRT_IRQ_EXTINT + 124) /* ACMP2 interrupt */
#define IMXRT_IRQ_ACMP3        (IMXRT_IRQ_EXTINT + 125) /* ACMP3 interrupt */
#define IMXRT_IRQ_ACMP4        (IMXRT_IRQ_EXTINT + 126) /* ACMP4 interrupt */
#define IMXRT_IRQ_RESERVED127  (IMXRT_IRQ_EXTINT + 127) /* Reserved */
#define IMXRT_IRQ_RESERVED128  (IMXRT_IRQ_EXTINT + 128) /* Reserved  */
#define IMXRT_IRQ_ENC1         (IMXRT_IRQ_EXTINT + 129) /* ENC1 interrupt */
#define IMXRT_IRQ_ENC2         (IMXRT_IRQ_EXTINT + 130) /* ENC2 interrupt */
#define IMXRT_IRQ_ENC3         (IMXRT_IRQ_EXTINT + 131) /* ENC3 interrupt */
#define IMXRT_IRQ_ENC4         (IMXRT_IRQ_EXTINT + 132) /* ENC4 interrupt */
#define IMXRT_IRQ_QTIMER1      (IMXRT_IRQ_EXTINT + 133) /* QTIMER1 timer 0-3 interrupt */
#define IMXRT_IRQ_QTIMER2      (IMXRT_IRQ_EXTINT + 134) /* QTIMER2 timer 0-3 interrupt */
#define IMXRT_IRQ_QTIMER3      (IMXRT_IRQ_EXTINT + 135) /* QTIMER3 timer 0-3 interrupt */
#define IMXRT_IRQ_QTIMER4      (IMXRT_IRQ_EXTINT + 136) /* QTIMER4 timer 0-3 interrupt */
#define IMXRT_IRQ_FLEXPWM2_0   (IMXRT_IRQ_EXTINT + 137) /* LEXPWM2 capture/compare/reload 0 interrupt */
#define IMXRT_IRQ_FLEXPWM2_1   (IMXRT_IRQ_EXTINT + 138) /* LEXPWM2 capture/compare/reload 1 interrupt */
#define IMXRT_IRQ_FLEXPWM2_2   (IMXRT_IRQ_EXTINT + 139) /* LEXPWM2 capture/compare/reload 1 interrupt */
#define IMXRT_IRQ_FLEXPWM2_3   (IMXRT_IRQ_EXTINT + 140) /* LEXPWM2 capture/compare/reload 3 interrupt */
#define IMXRT_IRQ_FLEXPWM2_F   (IMXRT_IRQ_EXTINT + 141) /* LEXPWM2 fault interrupt */
#define IMXRT_IRQ_FLEXPWM3_0   (IMXRT_IRQ_EXTINT + 142) /* LEXPWM3 capture/compare/reload 0 interrupt */
#define IMXRT_IRQ_FLEXPWM3_1   (IMXRT_IRQ_EXTINT + 143) /* LEXPWM3 capture/compare/reload 1 interrupt */
#define IMXRT_IRQ_FLEXPWM3_2   (IMXRT_IRQ_EXTINT + 144) /* LEXPWM3 capture/compare/reload 2 interrupt */
#define IMXRT_IRQ_FLEXPWM3_F   (IMXRT_IRQ_EXTINT + 146) /* LEXPWM3 fault interrupt */
#define IMXRT_IRQ_FLEXPWM4_0   (IMXRT_IRQ_EXTINT + 147) /* LEXPWM4 capture/compare/reload 0 interrupt */
#define IMXRT_IRQ_FLEXPWM4_1   (IMXRT_IRQ_EXTINT + 148) /* LEXPWM4 capture/compare/reload 1 interrupt */
#define IMXRT_IRQ_FLEXPWM4_2   (IMXRT_IRQ_EXTINT + 149) /* LEXPWM4 capture/compare/reload 2 interrupt */
#define IMXRT_IRQ_FLEXPWM4_3   (IMXRT_IRQ_EXTINT + 150) /* LEXPWM4 capture/compare/reload 3 interrupt */
#define IMXRT_IRQ_FLEXPWM4_F   (IMXRT_IRQ_EXTINT + 151) /* LEXPWM4 fault interrupt */
#define IMXRT_IRQ_ENET2        (IMXRT_IRQ_EXTINT + 152) /* ENET2 MAC0 interrupt */
#define IMXRT_IRQ_ENET2_1588   (IMXRT_IRQ_EXTINT + 153) /* ENET2 MAC 0 1588 Timer Interrupt */
#define IMXRT_IRQ_CAN3         (IMXRT_IRQ_EXTINT + 154) /* CAN3 interrupt */
#define IMXRT_IRQ_RESERVED155  (IMXRT_IRQ_EXTINT + 155) /* Reserved */
#define IMXRT_IRQ_FLEXIO3      (IMXRT_IRQ_EXTINT + 156)  /* IPI compare interrupt */
#define IMXRT_IRQ_GPIO_6789    (IMXRT_IRQ_EXTINT + 157) /* GPIO {6789} or'ed Interrupt */
#define IMXRT_IRQ_RESERVED158  (IMXRT_IRQ_EXTINT + 158) /* Reserved */
#define IMXRT_IRQ_RESERVED159  (IMXRT_IRQ_EXTINT + 159) /* Reserved */

#define IMXRT_IRQ_NEXTINT      160

/* GPIO second level interrupt **********************************************************/

#define IMXRT_GPIO_IRQ_FIRST   (IMXRT_IRQ_EXTINT + IMXRT_IRQ_NEXTINT)
#define _IMXRT_GPIO1_0_15_BASE IMXRT_GPIO_IRQ_FIRST

#ifdef CONFIG_IMXRT_GPIO1_0_15_IRQ
  /* GPIO1 has dedicated interrupts for pins 0-7
   * REVISIT:  I am assuming that you really cannot use the dedicated and the multiplex
   * interrupts concurrently.
   */

#  define IMXRT_IRQ_GPIO1_0    (_IMXRT_GPIO1_0_15_BASE + 0)   /* GPIO1 pin 0 interrupt */
#  define IMXRT_IRQ_GPIO1_1    (_IMXRT_GPIO1_0_15_BASE + 1)   /* GPIO1 pin 1 interrupt */
#  define IMXRT_IRQ_GPIO1_2    (_IMXRT_GPIO1_0_15_BASE + 2)   /* GPIO1 pin 2 interrupt */
#  define IMXRT_IRQ_GPIO1_3    (_IMXRT_GPIO1_0_15_BASE + 3)   /* GPIO1 pin 3 interrupt */
#  define IMXRT_IRQ_GPIO1_4    (_IMXRT_GPIO1_0_15_BASE + 4)   /* GPIO1 pin 4 interrupt */
#  define IMXRT_IRQ_GPIO1_5    (_IMXRT_GPIO1_0_15_BASE + 5)   /* GPIO1 pin 5 interrupt */
#  define IMXRT_IRQ_GPIO1_6    (_IMXRT_GPIO1_0_15_BASE + 6)   /* GPIO1 pin 6 interrupt */
#  define IMXRT_IRQ_GPIO1_7    (_IMXRT_GPIO1_0_15_BASE + 7)   /* GPIO1 pin 7 interrupt */
#  define IMXRT_IRQ_GPIO1_8    (_IMXRT_GPIO1_0_15_BASE + 8)   /* GPIO1 pin 8 interrupt */
#  define IMXRT_IRQ_GPIO1_9    (_IMXRT_GPIO1_0_15_BASE + 9)   /* GPIO1 pin 9 interrupt */
#  define IMXRT_IRQ_GPIO1_10   (_IMXRT_GPIO1_0_15_BASE + 10)  /* GPIO1 pin 10 interrupt */
#  define IMXRT_IRQ_GPIO1_11   (_IMXRT_GPIO1_0_15_BASE + 11)  /* GPIO1 pin 11 interrupt */
#  define IMXRT_IRQ_GPIO1_12   (_IMXRT_GPIO1_0_15_BASE + 12)  /* GPIO1 pin 12 interrupt */
#  define IMXRT_IRQ_GPIO1_13   (_IMXRT_GPIO1_0_15_BASE + 13)  /* GPIO1 pin 13 interrupt */
#  define IMXRT_IRQ_GPIO1_14   (_IMXRT_GPIO1_0_15_BASE + 14)  /* GPIO1 pin 14 interrupt */
#  define IMXRT_IRQ_GPIO1_15   (_IMXRT_GPIO1_0_15_BASE + 15)  /* GPIO1 pin 15 interrupt */

#  define _IMXRT_GPIO1_8_15_NIRQS 16
#  define _IMXRT_GPIO1_16_31_BASE (_IMXRT_GPIO1_0_15_BASE + _IMXRT_GPIO1_8_15_NIRQS)
#else
#  define _IMXRT_GPIO1_8_15_NIRQS 0
#  define _IMXRT_GPIO1_16_31_BASE _IMXRT_GPIO1_0_15_BASE
#endif

#ifdef CONFIG_IMXRT_GPIO1_16_31_IRQ
#  define IMXRT_IRQ_GPIO1_16   (_IMXRT_GPIO1_16_31_BASE + 0)  /* GPIO1 pin 16 interrupt */
#  define IMXRT_IRQ_GPIO1_17   (_IMXRT_GPIO1_16_31_BASE + 1)  /* GPIO1 pin 17 interrupt */
#  define IMXRT_IRQ_GPIO1_18   (_IMXRT_GPIO1_16_31_BASE + 2)  /* GPIO1 pin 18 interrupt */
#  define IMXRT_IRQ_GPIO1_19   (_IMXRT_GPIO1_16_31_BASE + 3)  /* GPIO1 pin 19 interrupt */
#  define IMXRT_IRQ_GPIO1_20   (_IMXRT_GPIO1_16_31_BASE + 4)  /* GPIO1 pin 10 interrupt */
#  define IMXRT_IRQ_GPIO1_21   (_IMXRT_GPIO1_16_31_BASE + 5)  /* GPIO1 pin 21 interrupt */
#  define IMXRT_IRQ_GPIO1_22   (_IMXRT_GPIO1_16_31_BASE + 6)  /* GPIO1 pin 22 interrupt */
#  define IMXRT_IRQ_GPIO1_23   (_IMXRT_GPIO1_16_31_BASE + 7)  /* GPIO1 pin 23 interrupt */
#  define IMXRT_IRQ_GPIO1_24   (_IMXRT_GPIO1_16_31_BASE + 8)  /* GPIO1 pin 24 interrupt */
#  define IMXRT_IRQ_GPIO1_25   (_IMXRT_GPIO1_16_31_BASE + 9)  /* GPIO1 pin 25 interrupt */
#  define IMXRT_IRQ_GPIO1_26   (_IMXRT_GPIO1_16_31_BASE + 10) /* GPIO1 pin 26 interrupt */
#  define IMXRT_IRQ_GPIO1_27   (_IMXRT_GPIO1_16_31_BASE + 11) /* GPIO1 pin 27 interrupt */
#  define IMXRT_IRQ_GPIO1_28   (_IMXRT_GPIO1_16_31_BASE + 12) /* GPIO1 pin 28 interrupt */
#  define IMXRT_IRQ_GPIO1_29   (_IMXRT_GPIO1_16_31_BASE + 13) /* GPIO1 pin 29 interrupt */
#  define IMXRT_IRQ_GPIO1_30   (_IMXRT_GPIO1_16_31_BASE + 14) /* GPIO1 pin 30 interrupt */
#  define IMXRT_IRQ_GPIO1_31   (_IMXRT_GPIO1_16_31_BASE + 15) /* GPIO1 pin 31 interrupt */

#  define _IMXRT_GPIO1_16_31_NIRQS 16
#  define _IMXRT_GPIO2_0_15_BASE (_IMXRT_GPIO1_16_31_BASE + _IMXRT_GPIO1_16_31_NIRQS)
#  define IMXRT_GPIO1_NIRQS    (_IMXRT_GPIO1_8_15_NIRQS + _IMXRT_GPIO1_16_31_NIRQS)
#else
#  define _IMXRT_GPIO2_0_15_BASE _IMXRT_GPIO1_16_31_BASE
#  define IMXRT_GPIO1_NIRQS    _IMXRT_GPIO1_8_15_NIRQS
#endif

#ifdef CONFIG_IMXRT_GPIO2_0_15_IRQ
#  define IMXRT_IRQ_GPIO2_0    (_IMXRT_GPIO2_0_15_BASE + 0)   /* GPIO2 pin 0 interrupt */
#  define IMXRT_IRQ_GPIO2_1    (_IMXRT_GPIO2_0_15_BASE + 1)   /* GPIO2 pin 1 interrupt */
#  define IMXRT_IRQ_GPIO2_2    (_IMXRT_GPIO2_0_15_BASE + 2)   /* GPIO2 pin 2 interrupt */
#  define IMXRT_IRQ_GPIO2_3    (_IMXRT_GPIO2_0_15_BASE + 3)   /* GPIO2 pin 3 interrupt */
#  define IMXRT_IRQ_GPIO2_4    (_IMXRT_GPIO2_0_15_BASE + 4)   /* GPIO2 pin 4 interrupt */
#  define IMXRT_IRQ_GPIO2_5    (_IMXRT_GPIO2_0_15_BASE + 5)   /* GPIO2 pin 5 interrupt */
#  define IMXRT_IRQ_GPIO2_6    (_IMXRT_GPIO2_0_15_BASE + 6)   /* GPIO2 pin 6 interrupt */
#  define IMXRT_IRQ_GPIO2_7    (_IMXRT_GPIO2_0_15_BASE + 7)   /* GPIO2 pin 7 interrupt */
#  define IMXRT_IRQ_GPIO2_8    (_IMXRT_GPIO2_0_15_BASE + 8)   /* GPIO2 pin 8 interrupt */
#  define IMXRT_IRQ_GPIO2_9    (_IMXRT_GPIO2_0_15_BASE + 9)   /* GPIO2 pin 9 interrupt */
#  define IMXRT_IRQ_GPIO2_10   (_IMXRT_GPIO2_0_15_BASE + 10)  /* GPIO2 pin 10 interrupt */
#  define IMXRT_IRQ_GPIO2_11   (_IMXRT_GPIO2_0_15_BASE + 11)  /* GPIO2 pin 11 interrupt */
#  define IMXRT_IRQ_GPIO2_12   (_IMXRT_GPIO2_0_15_BASE + 12)  /* GPIO2 pin 12 interrupt */
#  define IMXRT_IRQ_GPIO2_13   (_IMXRT_GPIO2_0_15_BASE + 13)  /* GPIO2 pin 13 interrupt */
#  define IMXRT_IRQ_GPIO2_14   (_IMXRT_GPIO2_0_15_BASE + 14)  /* GPIO2 pin 14 interrupt */
#  define IMXRT_IRQ_GPIO2_15   (_IMXRT_GPIO2_0_15_BASE + 15)  /* GPIO2 pin 15 interrupt */

#  define _IMXRT_GPIO2_0_15_NIRQS 16
#  define _IMXRT_GPIO2_16_31_BASE (_IMXRT_GPIO2_0_15_BASE + _IMXRT_GPIO2_0_15_NIRQS)
#else
#  define _IMXRT_GPIO2_0_15_NIRQS 0
#  define _IMXRT_GPIO2_16_31_BASE _IMXRT_GPIO2_0_15_BASE
#endif

#ifdef CONFIG_IMXRT_GPIO2_16_31_IRQ
#  define IMXRT_IRQ_GPIO2_16   (_IMXRT_GPIO2_16_31_BASE + 0)  /* GPIO2 pin 16 interrupt */
#  define IMXRT_IRQ_GPIO2_17   (_IMXRT_GPIO2_16_31_BASE + 1)  /* GPIO2 pin 17 interrupt */
#  define IMXRT_IRQ_GPIO2_18   (_IMXRT_GPIO2_16_31_BASE + 2)  /* GPIO2 pin 18 interrupt */
#  define IMXRT_IRQ_GPIO2_19   (_IMXRT_GPIO2_16_31_BASE + 3)  /* GPIO2 pin 19 interrupt */
#  define IMXRT_IRQ_GPIO2_20   (_IMXRT_GPIO2_16_31_BASE + 4)  /* GPIO2 pin 20 interrupt */
#  define IMXRT_IRQ_GPIO2_21   (_IMXRT_GPIO2_16_31_BASE + 5)  /* GPIO2 pin 21 interrupt */
#  define IMXRT_IRQ_GPIO2_22   (_IMXRT_GPIO2_16_31_BASE + 6)  /* GPIO2 pin 22 interrupt */
#  define IMXRT_IRQ_GPIO2_23   (_IMXRT_GPIO2_16_31_BASE + 7)  /* GPIO2 pin 23 interrupt */
#  define IMXRT_IRQ_GPIO2_24   (_IMXRT_GPIO2_16_31_BASE + 8)  /* GPIO2 pin 24 interrupt */
#  define IMXRT_IRQ_GPIO2_25   (_IMXRT_GPIO2_16_31_BASE + 9)  /* GPIO2 pin 25 interrupt */
#  define IMXRT_IRQ_GPIO2_26   (_IMXRT_GPIO2_16_31_BASE + 10) /* GPIO2 pin 26 interrupt */
#  define IMXRT_IRQ_GPIO2_27   (_IMXRT_GPIO2_16_31_BASE + 11) /* GPIO2 pin 27 interrupt */
#  define IMXRT_IRQ_GPIO2_28   (_IMXRT_GPIO2_16_31_BASE + 12) /* GPIO2 pin 28 interrupt */
#  define IMXRT_IRQ_GPIO2_29   (_IMXRT_GPIO2_16_31_BASE + 13) /* GPIO2 pin 29 interrupt */
#  define IMXRT_IRQ_GPIO2_30   (_IMXRT_GPIO2_16_31_BASE + 14) /* GPIO2 pin 30 interrupt */
#  define IMXRT_IRQ_GPIO2_31   (_IMXRT_GPIO2_16_31_BASE + 15) /* GPIO2 pin 31 interrupt */

#  define _IMXRT_GPIO2_16_31_NIRQS 16
#  define _IMXRT_GPIO3_0_15_BASE (_IMXRT_GPIO2_16_31_BASE + _IMXRT_GPIO2_16_31_NIRQS)
#  define IMXRT_GPIO2_NIRQS    (_IMXRT_GPIO2_0_15_NIRQS + _IMXRT_GPIO2_16_31_NIRQS)
#else
#  define _IMXRT_GPIO3_0_15_BASE _IMXRT_GPIO2_16_31_BASE
#  define IMXRT_GPIO2_NIRQS    _IMXRT_GPIO2_0_15_NIRQS
#endif

#ifdef CONFIG_IMXRT_GPIO3_0_15_IRQ
#  define IMXRT_IRQ_GPIO3_0    (_IMXRT_GPIO3_0_15_BASE + 0)   /* GPIO3 pin 0 interrupt */
#  define IMXRT_IRQ_GPIO3_1    (_IMXRT_GPIO3_0_15_BASE + 1)   /* GPIO3 pin 1 interrupt */
#  define IMXRT_IRQ_GPIO3_2    (_IMXRT_GPIO3_0_15_BASE + 2)   /* GPIO3 pin 2 interrupt */
#  define IMXRT_IRQ_GPIO3_3    (_IMXRT_GPIO3_0_15_BASE + 3)   /* GPIO3 pin 3 interrupt */
#  define IMXRT_IRQ_GPIO3_4    (_IMXRT_GPIO3_0_15_BASE + 4)   /* GPIO3 pin 4 interrupt */
#  define IMXRT_IRQ_GPIO3_5    (_IMXRT_GPIO3_0_15_BASE + 5)   /* GPIO3 pin 5 interrupt */
#  define IMXRT_IRQ_GPIO3_6    (_IMXRT_GPIO3_0_15_BASE + 6)   /* GPIO3 pin 6 interrupt */
#  define IMXRT_IRQ_GPIO3_7    (_IMXRT_GPIO3_0_15_BASE + 7)   /* GPIO3 pin 7 interrupt */
#  define IMXRT_IRQ_GPIO3_8    (_IMXRT_GPIO3_0_15_BASE + 8)   /* GPIO3 pin 8 interrupt */
#  define IMXRT_IRQ_GPIO3_9    (_IMXRT_GPIO3_0_15_BASE + 9)   /* GPIO3 pin 9 interrupt */
#  define IMXRT_IRQ_GPIO3_10   (_IMXRT_GPIO3_0_15_BASE + 10)  /* GPIO3 pin 10 interrupt */
#  define IMXRT_IRQ_GPIO3_11   (_IMXRT_GPIO3_0_15_BASE + 11)  /* GPIO3 pin 11 interrupt */
#  define IMXRT_IRQ_GPIO3_12   (_IMXRT_GPIO3_0_15_BASE + 12)  /* GPIO3 pin 12 interrupt */
#  define IMXRT_IRQ_GPIO3_13   (_IMXRT_GPIO3_0_15_BASE + 13)  /* GPIO3 pin 13 interrupt */
#  define IMXRT_IRQ_GPIO3_14   (_IMXRT_GPIO3_0_15_BASE + 14)  /* GPIO3 pin 14 interrupt */
#  define IMXRT_IRQ_GPIO3_15   (_IMXRT_GPIO3_0_15_BASE + 15)  /* GPIO3 pin 15 interrupt */

#  define _IMXRT_GPIO3_0_15_NIRQS 16
#  define _IMXRT_GPIO3_16_31_BASE (_IMXRT_GPIO3_0_15_BASE + _IMXRT_GPIO3_0_15_NIRQS)
#else
#  define _IMXRT_GPIO3_0_15_NIRQS 0
#  define _IMXRT_GPIO3_16_31_BASE _IMXRT_GPIO3_0_15_BASE
#endif

#ifdef CONFIG_IMXRT_GPIO3_16_31_IRQ
#  define IMXRT_IRQ_GPIO3_16   (_IMXRT_GPIO3_16_31_BASE + 0)  /* GPIO3 pin 16 interrupt */
#  define IMXRT_IRQ_GPIO3_17   (_IMXRT_GPIO3_16_31_BASE + 1)  /* GPIO3 pin 17 interrupt */
#  define IMXRT_IRQ_GPIO3_18   (_IMXRT_GPIO3_16_31_BASE + 2)  /* GPIO3 pin 18 interrupt */
#  define IMXRT_IRQ_GPIO3_19   (_IMXRT_GPIO3_16_31_BASE + 3)  /* GPIO3 pin 19 interrupt */
#  define IMXRT_IRQ_GPIO3_20   (_IMXRT_GPIO3_16_31_BASE + 4)  /* GPIO3 pin 20 interrupt */
#  define IMXRT_IRQ_GPIO3_21   (_IMXRT_GPIO3_16_31_BASE + 5)  /* GPIO3 pin 21 interrupt */
#  define IMXRT_IRQ_GPIO3_22   (_IMXRT_GPIO3_16_31_BASE + 6)  /* GPIO3 pin 22 interrupt */
#  define IMXRT_IRQ_GPIO3_23   (_IMXRT_GPIO3_16_31_BASE + 7)  /* GPIO3 pin 23 interrupt */
#  define IMXRT_IRQ_GPIO3_24   (_IMXRT_GPIO3_16_31_BASE + 8)  /* GPIO3 pin 24 interrupt */
#  define IMXRT_IRQ_GPIO3_25   (_IMXRT_GPIO3_16_31_BASE + 9)  /* GPIO3 pin 25 interrupt */
#  define IMXRT_IRQ_GPIO3_26   (_IMXRT_GPIO3_16_31_BASE + 10) /* GPIO3 pin 26 interrupt */
#  define IMXRT_IRQ_GPIO3_27   (_IMXRT_GPIO3_16_31_BASE + 11) /* GPIO3 pin 27 interrupt */
#  define IMXRT_IRQ_GPIO3_28   (_IMXRT_GPIO3_16_31_BASE + 12) /* GPIO3 pin 28 interrupt */
#  define IMXRT_IRQ_GPIO3_29   (_IMXRT_GPIO3_16_31_BASE + 13) /* GPIO3 pin 29 interrupt */
#  define IMXRT_IRQ_GPIO3_30   (_IMXRT_GPIO3_16_31_BASE + 14) /* GPIO3 pin 30 interrupt */
#  define IMXRT_IRQ_GPIO3_31   (_IMXRT_GPIO3_16_31_BASE + 15) /* GPIO3 pin 31 interrupt */

#  define _IMXRT_GPIO3_16_31_NIRQS 16
#  define _IMXRT_GPIO4_0_15_BASE (_IMXRT_GPIO3_16_31_BASE + _IMXRT_GPIO3_16_31_NIRQS)
#  define IMXRT_GPIO3_NIRQS    (_IMXRT_GPIO3_0_15_NIRQS + _IMXRT_GPIO3_16_31_NIRQS)
#else
#  define _IMXRT_GPIO4_0_15_BASE _IMXRT_GPIO3_16_31_BASE
#  define IMXRT_GPIO3_NIRQS    _IMXRT_GPIO3_0_15_NIRQS
#endif

#ifdef CONFIG_IMXRT_GPIO4_0_15_IRQ
#  define IMXRT_IRQ_GPIO4_0    (_IMXRT_GPIO4_0_15_BASE + 0)   /* GPIO4 pin 0 interrupt */
#  define IMXRT_IRQ_GPIO4_1    (_IMXRT_GPIO4_0_15_BASE + 1)   /* GPIO4 pin 1 interrupt */
#  define IMXRT_IRQ_GPIO4_2    (_IMXRT_GPIO4_0_15_BASE + 2)   /* GPIO4 pin 2 interrupt */
#  define IMXRT_IRQ_GPIO4_3    (_IMXRT_GPIO4_0_15_BASE + 3)   /* GPIO4 pin 3 interrupt */
#  define IMXRT_IRQ_GPIO4_4    (_IMXRT_GPIO4_0_15_BASE + 4)   /* GPIO4 pin 4 interrupt */
#  define IMXRT_IRQ_GPIO4_5    (_IMXRT_GPIO4_0_15_BASE + 5)   /* GPIO4 pin 5 interrupt */
#  define IMXRT_IRQ_GPIO4_6    (_IMXRT_GPIO4_0_15_BASE + 6)   /* GPIO4 pin 6 interrupt */
#  define IMXRT_IRQ_GPIO4_7    (_IMXRT_GPIO4_0_15_BASE + 7)   /* GPIO4 pin 7 interrupt */
#  define IMXRT_IRQ_GPIO4_8    (_IMXRT_GPIO4_0_15_BASE + 8)   /* GPIO4 pin 8 interrupt */
#  define IMXRT_IRQ_GPIO4_9    (_IMXRT_GPIO4_0_15_BASE + 9)   /* GPIO4 pin 9 interrupt */
#  define IMXRT_IRQ_GPIO4_10   (_IMXRT_GPIO4_0_15_BASE + 10)  /* GPIO4 pin 10 interrupt */
#  define IMXRT_IRQ_GPIO4_11   (_IMXRT_GPIO4_0_15_BASE + 11)  /* GPIO4 pin 11 interrupt */
#  define IMXRT_IRQ_GPIO4_12   (_IMXRT_GPIO4_0_15_BASE + 12)  /* GPIO4 pin 12 interrupt */
#  define IMXRT_IRQ_GPIO4_13   (_IMXRT_GPIO4_0_15_BASE + 13)  /* GPIO4 pin 13 interrupt */
#  define IMXRT_IRQ_GPIO4_14   (_IMXRT_GPIO4_0_15_BASE + 14)  /* GPIO4 pin 14 interrupt */
#  define IMXRT_IRQ_GPIO4_15   (_IMXRT_GPIO4_0_15_BASE + 15)  /* GPIO4 pin 15 interrupt */

#  define _IMXRT_GPIO4_0_15_NIRQS 16
#  define _IMXRT_GPIO4_16_31_BASE (_IMXRT_GPIO4_0_15_BASE + _IMXRT_GPIO4_0_15_NIRQS)
#else
#  define _IMXRT_GPIO4_0_15_NIRQS 0
#  define _IMXRT_GPIO4_16_31_BASE _IMXRT_GPIO4_0_15_BASE
#endif

#ifdef CONFIG_IMXRT_GPIO4_16_31_IRQ
#  define IMXRT_IRQ_GPIO4_16   (_IMXRT_GPIO4_16_31_BASE + 0)  /* GPIO4 pin 16 interrupt */
#  define IMXRT_IRQ_GPIO4_17   (_IMXRT_GPIO4_16_31_BASE + 1)  /* GPIO4 pin 17 interrupt */
#  define IMXRT_IRQ_GPIO4_18   (_IMXRT_GPIO4_16_31_BASE + 2)  /* GPIO4 pin 18 interrupt */
#  define IMXRT_IRQ_GPIO4_19   (_IMXRT_GPIO4_16_31_BASE + 3)  /* GPIO4 pin 19 interrupt */
#  define IMXRT_IRQ_GPIO4_20   (_IMXRT_GPIO4_16_31_BASE + 4)  /* GPIO4 pin 20 interrupt */
#  define IMXRT_IRQ_GPIO4_21   (_IMXRT_GPIO4_16_31_BASE + 5)  /* GPIO4 pin 21 interrupt */
#  define IMXRT_IRQ_GPIO4_22   (_IMXRT_GPIO4_16_31_BASE + 6)  /* GPIO4 pin 22 interrupt */
#  define IMXRT_IRQ_GPIO4_23   (_IMXRT_GPIO4_16_31_BASE + 7)  /* GPIO4 pin 23 interrupt */
#  define IMXRT_IRQ_GPIO4_24   (_IMXRT_GPIO4_16_31_BASE + 8)  /* GPIO4 pin 24 interrupt */
#  define IMXRT_IRQ_GPIO4_25   (_IMXRT_GPIO4_16_31_BASE + 9)  /* GPIO4 pin 25 interrupt */
#  define IMXRT_IRQ_GPIO4_26   (_IMXRT_GPIO4_16_31_BASE + 10) /* GPIO4 pin 26 interrupt */
#  define IMXRT_IRQ_GPIO4_27   (_IMXRT_GPIO4_16_31_BASE + 11) /* GPIO4 pin 27 interrupt */
#  define IMXRT_IRQ_GPIO4_28   (_IMXRT_GPIO4_16_31_BASE + 12) /* GPIO4 pin 28 interrupt */
#  define IMXRT_IRQ_GPIO4_29   (_IMXRT_GPIO4_16_31_BASE + 13) /* GPIO4 pin 29 interrupt */
#  define IMXRT_IRQ_GPIO4_30   (_IMXRT_GPIO4_16_31_BASE + 14) /* GPIO4 pin 30 interrupt */
#  define IMXRT_IRQ_GPIO4_31   (_IMXRT_GPIO4_16_31_BASE + 15) /* GPIO4 pin 31 interrupt */

#  define _IMXRT_GPIO4_16_31_NIRQS 16
#  define _IMXRT_GPIO5_0_15_BASE (_IMXRT_GPIO4_16_31_BASE + _IMXRT_GPIO4_16_31_NIRQS)
#  define IMXRT_GPIO4_NIRQS    (_IMXRT_GPIO4_0_15_NIRQS + _IMXRT_GPIO4_16_31_NIRQS)
#else
#  define _IMXRT_GPIO5_0_15_BASE _IMXRT_GPIO4_16_31_BASE
#  define IMXRT_GPIO4_NIRQS    _IMXRT_GPIO4_0_15_NIRQS
#endif

#ifdef CONFIG_IMXRT_GPIO5_0_15_IRQ
#  define IMXRT_IRQ_GPIO5_0    (_IMXRT_GPIO5_0_15_BASE + 0)   /* GPIO5 pin 0 interrupt */
#  define IMXRT_IRQ_GPIO5_1    (_IMXRT_GPIO5_0_15_BASE + 1)   /* GPIO5 pin 1 interrupt */
#  define IMXRT_IRQ_GPIO5_2    (_IMXRT_GPIO5_0_15_BASE + 2)   /* GPIO5 pin 2 interrupt */
#  define IMXRT_IRQ_GPIO5_3    (_IMXRT_GPIO5_0_15_BASE + 3)   /* GPIO5 pin 3 interrupt */
#  define IMXRT_IRQ_GPIO5_4    (_IMXRT_GPIO5_0_15_BASE + 4)   /* GPIO5 pin 4 interrupt */
#  define IMXRT_IRQ_GPIO5_5    (_IMXRT_GPIO5_0_15_BASE + 5)   /* GPIO5 pin 5 interrupt */
#  define IMXRT_IRQ_GPIO5_6    (_IMXRT_GPIO5_0_15_BASE + 6)   /* GPIO5 pin 6 interrupt */
#  define IMXRT_IRQ_GPIO5_7    (_IMXRT_GPIO5_0_15_BASE + 7)   /* GPIO5 pin 7 interrupt */
#  define IMXRT_IRQ_GPIO5_8    (_IMXRT_GPIO5_0_15_BASE + 8)   /* GPIO5 pin 8 interrupt */
#  define IMXRT_IRQ_GPIO5_9    (_IMXRT_GPIO5_0_15_BASE + 9)   /* GPIO5 pin 9 interrupt */
#  define IMXRT_IRQ_GPIO5_10   (_IMXRT_GPIO5_0_15_BASE + 10)  /* GPIO5 pin 10 interrupt */
#  define IMXRT_IRQ_GPIO5_11   (_IMXRT_GPIO5_0_15_BASE + 11)  /* GPIO5 pin 11 interrupt */
#  define IMXRT_IRQ_GPIO5_12   (_IMXRT_GPIO5_0_15_BASE + 12)  /* GPIO5 pin 12 interrupt */
#  define IMXRT_IRQ_GPIO5_13   (_IMXRT_GPIO5_0_15_BASE + 13)  /* GPIO5 pin 13 interrupt */
#  define IMXRT_IRQ_GPIO5_14   (_IMXRT_GPIO5_0_15_BASE + 14)  /* GPIO5 pin 14 interrupt */
#  define IMXRT_IRQ_GPIO5_15   (_IMXRT_GPIO5_0_15_BASE + 15)  /* GPIO5 pin 15 interrupt */

#  define _IMXRT_GPIO5_0_15_NIRQS 16
#  define _IMXRT_GPIO5_16_31_BASE (_IMXRT_GPIO5_0_15_BASE + _IMXRT_GPIO5_0_15_NIRQS)
#else
#  define _IMXRT_GPIO5_0_15_NIRQS 0
#  define _IMXRT_GPIO5_16_31_BASE _IMXRT_GPIO5_0_15_BASE
#endif

#ifdef CONFIG_IMXRT_GPIO5_16_31_IRQ
#  define IMXRT_IRQ_GPIO5_16   (_IMXRT_GPIO5_16_31_BASE + 0)  /* GPIO5 pin 16 interrupt */
#  define IMXRT_IRQ_GPIO5_17   (_IMXRT_GPIO5_16_31_BASE + 1)  /* GPIO5 pin 17 interrupt */
#  define IMXRT_IRQ_GPIO5_18   (_IMXRT_GPIO5_16_31_BASE + 2)  /* GPIO5 pin 18 interrupt */
#  define IMXRT_IRQ_GPIO5_19   (_IMXRT_GPIO5_16_31_BASE + 3)  /* GPIO5 pin 19 interrupt */
#  define IMXRT_IRQ_GPIO5_20   (_IMXRT_GPIO5_16_31_BASE + 4)  /* GPIO5 pin 20 interrupt */
#  define IMXRT_IRQ_GPIO5_21   (_IMXRT_GPIO5_16_31_BASE + 5)  /* GPIO5 pin 21 interrupt */
#  define IMXRT_IRQ_GPIO5_22   (_IMXRT_GPIO5_16_31_BASE + 6)  /* GPIO5 pin 22 interrupt */
#  define IMXRT_IRQ_GPIO5_23   (_IMXRT_GPIO5_16_31_BASE + 7)  /* GPIO5 pin 23 interrupt */
#  define IMXRT_IRQ_GPIO5_24   (_IMXRT_GPIO5_16_31_BASE + 8)  /* GPIO5 pin 24 interrupt */
#  define IMXRT_IRQ_GPIO5_25   (_IMXRT_GPIO5_16_31_BASE + 9)  /* GPIO5 pin 25 interrupt */
#  define IMXRT_IRQ_GPIO5_26   (_IMXRT_GPIO5_16_31_BASE + 10) /* GPIO5 pin 26 interrupt */
#  define IMXRT_IRQ_GPIO5_27   (_IMXRT_GPIO5_16_31_BASE + 11) /* GPIO5 pin 27 interrupt */
#  define IMXRT_IRQ_GPIO5_28   (_IMXRT_GPIO5_16_31_BASE + 12) /* GPIO5 pin 28 interrupt */
#  define IMXRT_IRQ_GPIO5_29   (_IMXRT_GPIO5_16_31_BASE + 13) /* GPIO5 pin 29 interrupt */
#  define IMXRT_IRQ_GPIO5_30   (_IMXRT_GPIO5_16_31_BASE + 14) /* GPIO5 pin 30 interrupt */
#  define IMXRT_IRQ_GPIO5_31   (_IMXRT_GPIO5_16_31_BASE + 15) /* GPIO5 pin 31 interrupt */

#  define _IMXRT_GPIO5_16_31_NIRQS 16
#  define _IMXRT_GPIO6_0_15_BASE (_IMXRT_GPIO5_16_31_BASE + _IMXRT_GPIO5_16_31_NIRQS)
#  define IMXRT_GPIO5_NIRQS    (_IMXRT_GPIO5_0_15_NIRQS + _IMXRT_GPIO5_16_31_NIRQS)
#else
#  define _IMXRT_GPIO6_0_15_BASE _IMXRT_GPIO5_16_31_BASE
#  define IMXRT_GPIO5_NIRQS    _IMXRT_GPIO5_0_15_NIRQS
#endif

#ifdef CONFIG_IMXRT_GPIO6_0_15_IRQ
#  define IMXRT_IRQ_GPIO6_0    (_IMXRT_GPIO6_0_15_BASE + 0)   /* GPIO6 pin 0 interrupt */
#  define IMXRT_IRQ_GPIO6_1    (_IMXRT_GPIO6_0_15_BASE + 1)   /* GPIO6 pin 1 interrupt */
#  define IMXRT_IRQ_GPIO6_2    (_IMXRT_GPIO6_0_15_BASE + 2)   /* GPIO6 pin 2 interrupt */
#  define IMXRT_IRQ_GPIO6_3    (_IMXRT_GPIO6_0_15_BASE + 3)   /* GPIO6 pin 3 interrupt */
#  define IMXRT_IRQ_GPIO6_4    (_IMXRT_GPIO6_0_15_BASE + 4)   /* GPIO6 pin 4 interrupt */
#  define IMXRT_IRQ_GPIO6_5    (_IMXRT_GPIO6_0_15_BASE + 5)   /* GPIO6 pin 5 interrupt */
#  define IMXRT_IRQ_GPIO6_6    (_IMXRT_GPIO6_0_15_BASE + 6)   /* GPIO6 pin 6 interrupt */
#  define IMXRT_IRQ_GPIO6_7    (_IMXRT_GPIO6_0_15_BASE + 7)   /* GPIO6 pin 7 interrupt */
#  define IMXRT_IRQ_GPIO6_8    (_IMXRT_GPIO6_0_15_BASE + 8)   /* GPIO6 pin 8 interrupt */
#  define IMXRT_IRQ_GPIO6_9    (_IMXRT_GPIO6_0_15_BASE + 9)   /* GPIO6 pin 9 interrupt */
#  define IMXRT_IRQ_GPIO6_10   (_IMXRT_GPIO6_0_15_BASE + 10)  /* GPIO6 pin 10 interrupt */
#  define IMXRT_IRQ_GPIO6_11   (_IMXRT_GPIO6_0_15_BASE + 11)  /* GPIO6 pin 11 interrupt */
#  define IMXRT_IRQ_GPIO6_12   (_IMXRT_GPIO6_0_15_BASE + 12)  /* GPIO6 pin 12 interrupt */
#  define IMXRT_IRQ_GPIO6_13   (_IMXRT_GPIO6_0_15_BASE + 13)  /* GPIO6 pin 13 interrupt */
#  define IMXRT_IRQ_GPIO6_14   (_IMXRT_GPIO6_0_15_BASE + 14)  /* GPIO6 pin 14 interrupt */
#  define IMXRT_IRQ_GPIO6_15   (_IMXRT_GPIO6_0_15_BASE + 15)  /* GPIO6 pin 15 interrupt */

#  define _IMXRT_GPIO6_0_15_NIRQS 16
#  define _IMXRT_GPIO6_16_31_BASE (_IMXRT_GPIO6_0_15_BASE + _IMXRT_GPIO6_0_15_NIRQS)
#else
#  define _IMXRT_GPIO6_0_15_NIRQS 0
#  define _IMXRT_GPIO6_16_31_BASE _IMXRT_GPIO6_0_15_BASE
#endif

#ifdef CONFIG_IMXRT_GPIO6_16_31_IRQ
#  define IMXRT_IRQ_GPIO6_16   (_IMXRT_GPIO6_16_31_BASE + 0)  /* GPIO6 pin 16 interrupt */
#  define IMXRT_IRQ_GPIO6_17   (_IMXRT_GPIO6_16_31_BASE + 1)  /* GPIO6 pin 17 interrupt */
#  define IMXRT_IRQ_GPIO6_18   (_IMXRT_GPIO6_16_31_BASE + 2)  /* GPIO6 pin 18 interrupt */
#  define IMXRT_IRQ_GPIO6_19   (_IMXRT_GPIO6_16_31_BASE + 3)  /* GPIO6 pin 19 interrupt */
#  define IMXRT_IRQ_GPIO6_20   (_IMXRT_GPIO6_16_31_BASE + 4)  /* GPIO6 pin 20 interrupt */
#  define IMXRT_IRQ_GPIO6_21   (_IMXRT_GPIO6_16_31_BASE + 5)  /* GPIO6 pin 21 interrupt */
#  define IMXRT_IRQ_GPIO6_22   (_IMXRT_GPIO6_16_31_BASE + 6)  /* GPIO6 pin 22 interrupt */
#  define IMXRT_IRQ_GPIO6_23   (_IMXRT_GPIO6_16_31_BASE + 7)  /* GPIO6 pin 23 interrupt */
#  define IMXRT_IRQ_GPIO6_24   (_IMXRT_GPIO6_16_31_BASE + 8)  /* GPIO6 pin 24 interrupt */
#  define IMXRT_IRQ_GPIO6_25   (_IMXRT_GPIO6_16_31_BASE + 9)  /* GPIO6 pin 25 interrupt */
#  define IMXRT_IRQ_GPIO6_26   (_IMXRT_GPIO6_16_31_BASE + 10) /* GPIO6 pin 26 interrupt */
#  define IMXRT_IRQ_GPIO6_27   (_IMXRT_GPIO6_16_31_BASE + 11) /* GPIO6 pin 27 interrupt */
#  define IMXRT_IRQ_GPIO6_28   (_IMXRT_GPIO6_16_31_BASE + 12) /* GPIO6 pin 28 interrupt */
#  define IMXRT_IRQ_GPIO6_29   (_IMXRT_GPIO6_16_31_BASE + 13) /* GPIO6 pin 29 interrupt */
#  define IMXRT_IRQ_GPIO6_30   (_IMXRT_GPIO6_16_31_BASE + 14) /* GPIO6 pin 30 interrupt */
#  define IMXRT_IRQ_GPIO6_31   (_IMXRT_GPIO6_16_31_BASE + 15) /* GPIO6 pin 31 interrupt */

#  define _IMXRT_GPIO6_16_31_NIRQS 16
#  define _IMXRT_GPIO7_0_15_BASE (_IMXRT_GPIO6_16_31_BASE + _IMXRT_GPIO6_16_31_NIRQS)
#  define IMXRT_GPIO6_NIRQS    (_IMXRT_GPIO6_0_15_NIRQS + _IMXRT_GPIO6_16_31_NIRQS)
#else
#  define _IMXRT_GPIO7_0_15_BASE _IMXRT_GPIO6_16_31_BASE
#  define IMXRT_GPIO6_NIRQS    _IMXRT_GPIO6_0_15_NIRQS
#endif

#ifdef CONFIG_IMXRT_GPIO7_0_15_IRQ
#  define IMXRT_IRQ_GPIO7_0    (_IMXRT_GPIO7_0_15_BASE + 0)   /* GPIO7 pin 0 interrupt */
#  define IMXRT_IRQ_GPIO7_1    (_IMXRT_GPIO7_0_15_BASE + 1)   /* GPIO7 pin 1 interrupt */
#  define IMXRT_IRQ_GPIO7_2    (_IMXRT_GPIO7_0_15_BASE + 2)   /* GPIO7 pin 2 interrupt */
#  define IMXRT_IRQ_GPIO7_3    (_IMXRT_GPIO7_0_15_BASE + 3)   /* GPIO7 pin 3 interrupt */
#  define IMXRT_IRQ_GPIO7_4    (_IMXRT_GPIO7_0_15_BASE + 4)   /* GPIO7 pin 4 interrupt */
#  define IMXRT_IRQ_GPIO7_5    (_IMXRT_GPIO7_0_15_BASE + 5)   /* GPIO7 pin 5 interrupt */
#  define IMXRT_IRQ_GPIO7_6    (_IMXRT_GPIO7_0_15_BASE + 6)   /* GPIO7 pin 6 interrupt */
#  define IMXRT_IRQ_GPIO7_7    (_IMXRT_GPIO7_0_15_BASE + 7)   /* GPIO7 pin 7 interrupt */
#  define IMXRT_IRQ_GPIO7_8    (_IMXRT_GPIO7_0_15_BASE + 8)   /* GPIO7 pin 8 interrupt */
#  define IMXRT_IRQ_GPIO7_9    (_IMXRT_GPIO7_0_15_BASE + 9)   /* GPIO7 pin 9 interrupt */
#  define IMXRT_IRQ_GPIO7_10   (_IMXRT_GPIO7_0_15_BASE + 10)  /* GPIO7 pin 10 interrupt */
#  define IMXRT_IRQ_GPIO7_11   (_IMXRT_GPIO7_0_15_BASE + 11)  /* GPIO7 pin 11 interrupt */
#  define IMXRT_IRQ_GPIO7_12   (_IMXRT_GPIO7_0_15_BASE + 12)  /* GPIO7 pin 12 interrupt */
#  define IMXRT_IRQ_GPIO7_13   (_IMXRT_GPIO7_0_15_BASE + 13)  /* GPIO7 pin 13 interrupt */
#  define IMXRT_IRQ_GPIO7_14   (_IMXRT_GPIO7_0_15_BASE + 14)  /* GPIO7 pin 14 interrupt */
#  define IMXRT_IRQ_GPIO7_15   (_IMXRT_GPIO7_0_15_BASE + 15)  /* GPIO7 pin 15 interrupt */

#  define _IMXRT_GPIO7_0_15_NIRQS 16
#  define _IMXRT_GPIO7_16_31_BASE (_IMXRT_GPIO7_0_15_BASE + _IMXRT_GPIO7_0_15_NIRQS)
#else
#  define _IMXRT_GPIO7_0_15_NIRQS 0
#  define _IMXRT_GPIO7_16_31_BASE _IMXRT_GPIO7_0_15_BASE
#endif

#ifdef CONFIG_IMXRT_GPIO7_16_31_IRQ
#  define IMXRT_IRQ_GPIO7_16   (_IMXRT_GPIO7_16_31_BASE + 0)  /* GPIO7 pin 16 interrupt */
#  define IMXRT_IRQ_GPIO7_17   (_IMXRT_GPIO7_16_31_BASE + 1)  /* GPIO7 pin 17 interrupt */
#  define IMXRT_IRQ_GPIO7_18   (_IMXRT_GPIO7_16_31_BASE + 2)  /* GPIO7 pin 18 interrupt */
#  define IMXRT_IRQ_GPIO7_19   (_IMXRT_GPIO7_16_31_BASE + 3)  /* GPIO7 pin 19 interrupt */
#  define IMXRT_IRQ_GPIO7_20   (_IMXRT_GPIO7_16_31_BASE + 4)  /* GPIO7 pin 20 interrupt */
#  define IMXRT_IRQ_GPIO7_21   (_IMXRT_GPIO7_16_31_BASE + 5)  /* GPIO7 pin 21 interrupt */
#  define IMXRT_IRQ_GPIO7_22   (_IMXRT_GPIO7_16_31_BASE + 6)  /* GPIO7 pin 22 interrupt */
#  define IMXRT_IRQ_GPIO7_23   (_IMXRT_GPIO7_16_31_BASE + 7)  /* GPIO7 pin 23 interrupt */
#  define IMXRT_IRQ_GPIO7_24   (_IMXRT_GPIO7_16_31_BASE + 8)  /* GPIO7 pin 24 interrupt */
#  define IMXRT_IRQ_GPIO7_25   (_IMXRT_GPIO7_16_31_BASE + 9)  /* GPIO7 pin 25 interrupt */
#  define IMXRT_IRQ_GPIO7_26   (_IMXRT_GPIO7_16_31_BASE + 10) /* GPIO7 pin 26 interrupt */
#  define IMXRT_IRQ_GPIO7_27   (_IMXRT_GPIO7_16_31_BASE + 11) /* GPIO7 pin 27 interrupt */
#  define IMXRT_IRQ_GPIO7_28   (_IMXRT_GPIO7_16_31_BASE + 12) /* GPIO7 pin 28 interrupt */
#  define IMXRT_IRQ_GPIO7_29   (_IMXRT_GPIO7_16_31_BASE + 13) /* GPIO7 pin 29 interrupt */
#  define IMXRT_IRQ_GPIO7_30   (_IMXRT_GPIO7_16_31_BASE + 14) /* GPIO7 pin 30 interrupt */
#  define IMXRT_IRQ_GPIO7_31   (_IMXRT_GPIO7_16_31_BASE + 15) /* GPIO7 pin 31 interrupt */

#  define _IMXRT_GPIO7_16_31_NIRQS 16
#  define _IMXRT_GPIO8_0_15_BASE (_IMXRT_GPIO7_16_31_BASE + _IMXRT_GPIO7_16_31_NIRQS)
#  define IMXRT_GPIO7_NIRQS    (_IMXRT_GPIO7_0_15_NIRQS + _IMXRT_GPIO7_16_31_NIRQS)
#else
#  define _IMXRT_GPIO8_0_15_BASE _IMXRT_GPIO7_16_31_BASE
#  define IMXRT_GPIO7_NIRQS    _IMXRT_GPIO7_0_15_NIRQS
#endif

#ifdef CONFIG_IMXRT_GPIO8_0_15_IRQ
#  define IMXRT_IRQ_GPIO8_0    (_IMXRT_GPIO8_0_15_BASE + 0)   /* GPIO8 pin 0 interrupt */
#  define IMXRT_IRQ_GPIO8_1    (_IMXRT_GPIO8_0_15_BASE + 1)   /* GPIO8 pin 1 interrupt */
#  define IMXRT_IRQ_GPIO8_2    (_IMXRT_GPIO8_0_15_BASE + 2)   /* GPIO8 pin 2 interrupt */
#  define IMXRT_IRQ_GPIO8_3    (_IMXRT_GPIO8_0_15_BASE + 3)   /* GPIO8 pin 3 interrupt */
#  define IMXRT_IRQ_GPIO8_4    (_IMXRT_GPIO8_0_15_BASE + 4)   /* GPIO8 pin 4 interrupt */
#  define IMXRT_IRQ_GPIO8_5    (_IMXRT_GPIO8_0_15_BASE + 5)   /* GPIO8 pin 5 interrupt */
#  define IMXRT_IRQ_GPIO8_6    (_IMXRT_GPIO8_0_15_BASE + 6)   /* GPIO8 pin 6 interrupt */
#  define IMXRT_IRQ_GPIO8_7    (_IMXRT_GPIO8_0_15_BASE + 7)   /* GPIO8 pin 7 interrupt */
#  define IMXRT_IRQ_GPIO8_8    (_IMXRT_GPIO8_0_15_BASE + 8)   /* GPIO8 pin 8 interrupt */
#  define IMXRT_IRQ_GPIO8_9    (_IMXRT_GPIO8_0_15_BASE + 9)   /* GPIO8 pin 9 interrupt */
#  define IMXRT_IRQ_GPIO8_10   (_IMXRT_GPIO8_0_15_BASE + 10)  /* GPIO8 pin 10 interrupt */
#  define IMXRT_IRQ_GPIO8_11   (_IMXRT_GPIO8_0_15_BASE + 11)  /* GPIO8 pin 11 interrupt */
#  define IMXRT_IRQ_GPIO8_12   (_IMXRT_GPIO8_0_15_BASE + 12)  /* GPIO8 pin 12 interrupt */
#  define IMXRT_IRQ_GPIO8_13   (_IMXRT_GPIO8_0_15_BASE + 13)  /* GPIO8 pin 13 interrupt */
#  define IMXRT_IRQ_GPIO8_14   (_IMXRT_GPIO8_0_15_BASE + 14)  /* GPIO8 pin 14 interrupt */
#  define IMXRT_IRQ_GPIO8_15   (_IMXRT_GPIO8_0_15_BASE + 15)  /* GPIO8 pin 15 interrupt */

#  define _IMXRT_GPIO8_0_15_NIRQS 16
#  define _IMXRT_GPIO8_16_31_BASE (_IMXRT_GPIO8_0_15_BASE + _IMXRT_GPIO8_0_15_NIRQS)
#else
#  define _IMXRT_GPIO8_0_15_NIRQS 0
#  define _IMXRT_GPIO8_16_31_BASE _IMXRT_GPIO8_0_15_BASE
#endif

#ifdef CONFIG_IMXRT_GPIO8_16_31_IRQ
#  define IMXRT_IRQ_GPIO8_16   (_IMXRT_GPIO8_16_31_BASE + 0)  /* GPIO8 pin 16 interrupt */
#  define IMXRT_IRQ_GPIO8_17   (_IMXRT_GPIO8_16_31_BASE + 1)  /* GPIO8 pin 17 interrupt */
#  define IMXRT_IRQ_GPIO8_18   (_IMXRT_GPIO8_16_31_BASE + 2)  /* GPIO8 pin 18 interrupt */
#  define IMXRT_IRQ_GPIO8_19   (_IMXRT_GPIO8_16_31_BASE + 3)  /* GPIO8 pin 19 interrupt */
#  define IMXRT_IRQ_GPIO8_20   (_IMXRT_GPIO8_16_31_BASE + 4)  /* GPIO8 pin 20 interrupt */
#  define IMXRT_IRQ_GPIO8_21   (_IMXRT_GPIO8_16_31_BASE + 5)  /* GPIO8 pin 21 interrupt */
#  define IMXRT_IRQ_GPIO8_22   (_IMXRT_GPIO8_16_31_BASE + 6)  /* GPIO8 pin 22 interrupt */
#  define IMXRT_IRQ_GPIO8_23   (_IMXRT_GPIO8_16_31_BASE + 7)  /* GPIO8 pin 23 interrupt */
#  define IMXRT_IRQ_GPIO8_24   (_IMXRT_GPIO8_16_31_BASE + 8)  /* GPIO8 pin 24 interrupt */
#  define IMXRT_IRQ_GPIO8_25   (_IMXRT_GPIO8_16_31_BASE + 9)  /* GPIO8 pin 25 interrupt */
#  define IMXRT_IRQ_GPIO8_26   (_IMXRT_GPIO8_16_31_BASE + 10) /* GPIO8 pin 26 interrupt */
#  define IMXRT_IRQ_GPIO8_27   (_IMXRT_GPIO8_16_31_BASE + 11) /* GPIO8 pin 27 interrupt */
#  define IMXRT_IRQ_GPIO8_28   (_IMXRT_GPIO8_16_31_BASE + 12) /* GPIO8 pin 28 interrupt */
#  define IMXRT_IRQ_GPIO8_29   (_IMXRT_GPIO8_16_31_BASE + 13) /* GPIO8 pin 29 interrupt */
#  define IMXRT_IRQ_GPIO8_30   (_IMXRT_GPIO8_16_31_BASE + 14) /* GPIO8 pin 30 interrupt */
#  define IMXRT_IRQ_GPIO8_31   (_IMXRT_GPIO8_16_31_BASE + 15) /* GPIO8 pin 31 interrupt */

#  define _IMXRT_GPIO8_16_31_NIRQS 16
#  define _IMXRT_GPIO9_0_15_BASE (_IMXRT_GPIO8_16_31_BASE + _IMXRT_GPIO8_16_31_NIRQS)
#  define IMXRT_GPIO8_NIRQS    (_IMXRT_GPIO8_0_15_NIRQS + _IMXRT_GPIO8_16_31_NIRQS)
#else
#  define _IMXRT_GPIO9_0_15_BASE _IMXRT_GPIO8_16_31_BASE
#  define IMXRT_GPIO8_NIRQS    _IMXRT_GPIO8_0_15_NIRQS
#endif

#ifdef CONFIG_IMXRT_GPIO9_0_15_IRQ
#  define IMXRT_IRQ_GPIO9_0    (_IMXRT_GPIO9_0_15_BASE + 0)   /* GPIO9 pin 0 interrupt */
#  define IMXRT_IRQ_GPIO9_1    (_IMXRT_GPIO9_0_15_BASE + 1)   /* GPIO9 pin 1 interrupt */
#  define IMXRT_IRQ_GPIO9_2    (_IMXRT_GPIO9_0_15_BASE + 2)   /* GPIO9 pin 2 interrupt */
#  define IMXRT_IRQ_GPIO9_3    (_IMXRT_GPIO9_0_15_BASE + 3)   /* GPIO9 pin 3 interrupt */
#  define IMXRT_IRQ_GPIO9_4    (_IMXRT_GPIO9_0_15_BASE + 4)   /* GPIO9 pin 4 interrupt */
#  define IMXRT_IRQ_GPIO9_5    (_IMXRT_GPIO9_0_15_BASE + 5)   /* GPIO9 pin 5 interrupt */
#  define IMXRT_IRQ_GPIO9_6    (_IMXRT_GPIO9_0_15_BASE + 6)   /* GPIO9 pin 6 interrupt */
#  define IMXRT_IRQ_GPIO9_7    (_IMXRT_GPIO9_0_15_BASE + 7)   /* GPIO9 pin 7 interrupt */
#  define IMXRT_IRQ_GPIO9_8    (_IMXRT_GPIO9_0_15_BASE + 8)   /* GPIO9 pin 8 interrupt */
#  define IMXRT_IRQ_GPIO9_9    (_IMXRT_GPIO9_0_15_BASE + 9)   /* GPIO9 pin 9 interrupt */
#  define IMXRT_IRQ_GPIO9_10   (_IMXRT_GPIO9_0_15_BASE + 10)  /* GPIO9 pin 10 interrupt */
#  define IMXRT_IRQ_GPIO9_11   (_IMXRT_GPIO9_0_15_BASE + 11)  /* GPIO9 pin 11 interrupt */
#  define IMXRT_IRQ_GPIO9_12   (_IMXRT_GPIO9_0_15_BASE + 12)  /* GPIO9 pin 12 interrupt */
#  define IMXRT_IRQ_GPIO9_13   (_IMXRT_GPIO9_0_15_BASE + 13)  /* GPIO9 pin 13 interrupt */
#  define IMXRT_IRQ_GPIO9_14   (_IMXRT_GPIO9_0_15_BASE + 14)  /* GPIO9 pin 14 interrupt */
#  define IMXRT_IRQ_GPIO9_15   (_IMXRT_GPIO9_0_15_BASE + 15)  /* GPIO9 pin 15 interrupt */

#  define _IMXRT_GPIO9_0_15_NIRQS 16
#  define _IMXRT_GPIO9_16_31_BASE (_IMXRT_GPIO9_0_15_BASE + _IMXRT_GPIO9_0_15_NIRQS)
#else
#  define _IMXRT_GPIO9_0_15_NIRQS 0
#  define _IMXRT_GPIO9_16_31_BASE _IMXRT_GPIO9_0_15_BASE
#endif

#ifdef CONFIG_IMXRT_GPIO9_16_31_IRQ
#  define IMXRT_IRQ_GPIO9_16   (_IMXRT_GPIO9_16_31_BASE + 0)  /* GPIO9 pin 16 interrupt */
#  define IMXRT_IRQ_GPIO9_17   (_IMXRT_GPIO9_16_31_BASE + 1)  /* GPIO9 pin 17 interrupt */
#  define IMXRT_IRQ_GPIO9_18   (_IMXRT_GPIO9_16_31_BASE + 2)  /* GPIO9 pin 18 interrupt */
#  define IMXRT_IRQ_GPIO9_19   (_IMXRT_GPIO9_16_31_BASE + 3)  /* GPIO9 pin 19 interrupt */
#  define IMXRT_IRQ_GPIO9_20   (_IMXRT_GPIO9_16_31_BASE + 4)  /* GPIO9 pin 20 interrupt */
#  define IMXRT_IRQ_GPIO9_21   (_IMXRT_GPIO9_16_31_BASE + 5)  /* GPIO9 pin 21 interrupt */
#  define IMXRT_IRQ_GPIO9_22   (_IMXRT_GPIO9_16_31_BASE + 6)  /* GPIO9 pin 22 interrupt */
#  define IMXRT_IRQ_GPIO9_23   (_IMXRT_GPIO9_16_31_BASE + 7)  /* GPIO9 pin 23 interrupt */
#  define IMXRT_IRQ_GPIO9_24   (_IMXRT_GPIO9_16_31_BASE + 8)  /* GPIO9 pin 24 interrupt */
#  define IMXRT_IRQ_GPIO9_25   (_IMXRT_GPIO9_16_31_BASE + 9)  /* GPIO9 pin 25 interrupt */
#  define IMXRT_IRQ_GPIO9_26   (_IMXRT_GPIO9_16_31_BASE + 10) /* GPIO9 pin 26 interrupt */
#  define IMXRT_IRQ_GPIO9_27   (_IMXRT_GPIO9_16_31_BASE + 11) /* GPIO9 pin 27 interrupt */
#  define IMXRT_IRQ_GPIO9_28   (_IMXRT_GPIO9_16_31_BASE + 12) /* GPIO9 pin 28 interrupt */
#  define IMXRT_IRQ_GPIO9_29   (_IMXRT_GPIO9_16_31_BASE + 13) /* GPIO9 pin 29 interrupt */
#  define IMXRT_IRQ_GPIO9_30   (_IMXRT_GPIO9_16_31_BASE + 14) /* GPIO9 pin 30 interrupt */
#  define IMXRT_IRQ_GPIO9_31   (_IMXRT_GPIO9_16_31_BASE + 15) /* GPIO9 pin 31 interrupt */

#  define _IMXRT_GPIO9_16_31_NIRQS 16
#  define IMXRT_GPIO9_NIRQS    (_IMXRT_GPIO9_0_15_NIRQS + _IMXRT_GPIO9_16_31_NIRQS)
#else
#  define IMXRT_GPIO9_NIRQS    _IMXRT_GPIO9_0_15_NIRQS
#endif

#define IMXRT_GPIO_NIRQS       (IMXRT_GPIO1_NIRQS + IMXRT_GPIO2_NIRQS + \
                                IMXRT_GPIO3_NIRQS + IMXRT_GPIO4_NIRQS + \
                                IMXRT_GPIO5_NIRQS + IMXRT_GPIO6_NIRQS + \
                                IMXRT_GPIO7_NIRQS + IMXRT_GPIO9_NIRQS + \
                                IMXRT_GPIO9_NIRQS )
#define IMXRT_GPIO_IRQ_LAST    (_IMXRT_GPIO1_0_15_BASE + IMXRT_GPIO_NIRQS)

/* Total number of IRQ numbers **********************************************************/

#define NR_IRQS                (IMXRT_IRQ_EXTINT + IMXRT_IRQ_NEXTINT + IMXRT_GPIO_NIRQS)

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Inline functions
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Function Prototypes
 ****************************************************************************************/

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

#endif /* __ARCH_ARM_INCLUDE_IMXRT_IMXRT106X_IRQ_H */
