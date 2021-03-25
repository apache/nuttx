/****************************************************************************
 * arch/arm/src/kl/hardware/kl_sim.h
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

#ifndef __ARCH_ARM_SRC_KL_HARDWARE_KL_SIM_H
#define __ARCH_ARM_SRC_KL_HARDWARE_KL_SIM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

/* Relative to KL_SIMLP_BASE */

#define KL_SIM_SOPT1_OFFSET          0x0000 /* System Options Register 1 */
#define KL_SIM_SOPT1CFG_OFFSET       0x0004 /* SOPT1 Configuration Register */

/* Relative to KL_SIM_BASE */

#define KL_SIM_SOPT2_OFFSET          0x0004 /* System Options Register 2 */
#define KL_SIM_SOPT4_OFFSET          0x000c /* System Options Register 4 */
#define KL_SIM_SOPT5_OFFSET          0x0010 /* System Options Register 5 */
#define KL_SIM_SOPT7_OFFSET          0x0018 /* System Options Register 7 */
#define KL_SIM_SDID_OFFSET           0x0024 /* System Device Identification Register */
#define KL_SIM_SCGC4_OFFSET          0x0034 /* System Clock Gating Control Register 4 */
#define KL_SIM_SCGC5_OFFSET          0x0038 /* System Clock Gating Control Register 5 */
#define KL_SIM_SCGC6_OFFSET          0x003c /* System Clock Gating Control Register 6 */
#define KL_SIM_SCGC7_OFFSET          0x0040 /* System Clock Gating Control Register 7 */
#define KL_SIM_CLKDIV1_OFFSET        0x0044 /* System Clock Divider Register 1 */
#define KL_SIM_FCFG1_OFFSET          0x004c /* Flash Configuration Register 1 */
#define KL_SIM_FCFG2_OFFSET          0x0050 /* Flash Configuration Register 2 */
#define KL_SIM_UIDMH_OFFSET          0x0058 /* Unique Identification Register Mid-High */
#define KL_SIM_UIDML_OFFSET          0x005c /* Unique Identification Register Mid Low */
#define KL_SIM_UIDL_OFFSET           0x0060 /* Unique Identification Register Low */
#define KL_SIM_COPC_OFFSET           0x0100 /* COP Control Register */
#define KL_SIM_SRVCOP_OFFSET         0x0104 /* Service COP Register */

/* Register Addresses *******************************************************/

/* NOTE:
 * The SIM_SOPT1 register is located at a different base address than the
 * other SIM registers.
 */

#define KL_SIM_SOPT1                 (KL_SIMLP_BASE+KL_SIM_SOPT1_OFFSET)
#define KL_SIM_SOPT1CFG              (KL_SIMLP_BASE+KL_SIM_SOPT1CFG_OFFSET)

#define KL_SIM_SOPT2                 (KL_SIM_BASE+KL_SIM_SOPT2_OFFSET)
#define KL_SIM_SOPT4                 (KL_SIM_BASE+KL_SIM_SOPT4_OFFSET)
#define KL_SIM_SOPT5                 (KL_SIM_BASE+KL_SIM_SOPT5_OFFSET)
#define KL_SIM_SOPT7                 (KL_SIM_BASE+KL_SIM_SOPT7_OFFSET)
#define KL_SIM_SDID                  (KL_SIM_BASE+KL_SIM_SDID_OFFSET)
#define KL_SIM_SCGC4                 (KL_SIM_BASE+KL_SIM_SCGC4_OFFSET)
#define KL_SIM_SCGC5                 (KL_SIM_BASE+KL_SIM_SCGC5_OFFSET)
#define KL_SIM_SCGC6                 (KL_SIM_BASE+KL_SIM_SCGC6_OFFSET)
#define KL_SIM_SCGC7                 (KL_SIM_BASE+KL_SIM_SCGC7_OFFSET)
#define KL_SIM_CLKDIV1               (KL_SIM_BASE+KL_SIM_CLKDIV1_OFFSET)
#define KL_SIM_FCFG1                 (KL_SIM_BASE+KL_SIM_FCFG1_OFFSET)
#define KL_SIM_FCFG2                 (KL_SIM_BASE+KL_SIM_FCFG2_OFFSET)
#define KL_SIM_UIDMH                 (KL_SIM_BASE+KL_SIM_UIDMH_OFFSET)
#define KL_SIM_UIDML                 (KL_SIM_BASE+KL_SIM_UIDML_OFFSET)
#define KL_SIM_UIDL                  (KL_SIM_BASE+KL_SIM_UIDL_OFFSET)
#define KL_SIM_COPC                  (KL_SIM_BASE+KL_SIM_COPC_OFFSET)
#define KL_SIM_SRVCOP                (KL_SIM_BASE+KL_SIM_SRVCOP_OFFSET)

/* Register Bit Definitions *************************************************/

/* System Options Register 1 */

                                               /* Bits 0-17: Reserved */
#define SIM_SOPT1_OSC32KSEL_SHIFT    (18)      /* Bit 18-19: 32K oscillator clock select */
#define SIM_SOPT1_OSC32KSEL_MASK     (3 << SIM_SOPT1_OSC32KSEL_SHIFT)
#  define SIM_SOPT1_OSC32KSEL_SYS    (1 << SIM_SOPT1_OSC32KSEL_SHIFT) /* System oscillator (OSC32KCLK) */
#  define SIM_SOPT1_OSC32KSEL_RTC    (2 << SIM_SOPT1_OSC32KSEL_SHIFT) /* RTC_CLKIN */
#  define SIM_SOPT1_OSC32KSEL_LPO    (3 << SIM_SOPT1_OSC32KSEL_SHIFT) /* LPO 1kHz */

                                               /* Bits 20-28: Reserved */
#define SIM_SOPT1_USBSTBY            (1 << 29) /* Bit 29: USB voltage regulator in
                                                *         standby mode (VLPR and VLPW modes) */
#define SIM_SOPT1_USBSSTBY           (1 << 30) /* Bit 30: USB voltage regulator in standb
                                                *         mode (Stop, VLPS, LLS and VLLS modes) */
#define SIM_SOPT1_USBREGEN           (1 << 31) /* Bit 31: USB voltage regulator enable */

/* SOPT1 Configuration Register */

#define SIM_SOPT1CFG_URWE            (1 << 24) /* Bit 24: USB voltage regulator enable write enable */
#define SIM_SOPT1CFG_UVSWE           (1 << 25) /* Bit 25: USB voltage regulator VLP standby write enable */
#define SIM_SOPT1CFG_USSWE           (1 << 26) /* Bit 26: USB voltage regulator stop standby write enable */

/* To be provided */

/* System Options Register 2 */

                                               /* Bits 0-3: Reserved */
#define SIM_SOPT2_RTCCLKOUTSEL       (1 << 4)  /* Bit 0:  MCG clock select */
#define SIM_SOPT2_CLKOUTSEL_SHIFT    (5)       /* Bits 5-7: RTC clock out select */
#define SIM_SOPT2_CLKOUTSEL_MASK     (3 << SIM_SOPT2_CLKOUTSEL_SHIFT)
#  define SIM_SOPT2_CLKOUTSEL_BUSCLK   (2 << SIM_SOPT2_CLKOUTSEL_SHIFT) /* Bus clock */
#  define SIM_SOPT2_CLKOUTSEL_LPO      (3 << SIM_SOPT2_CLKOUTSEL_SHIFT) /* LPO clock (1 kHz) */
#  define SIM_SOPT2_CLKOUTSEL_MCGIRCLK (4 << SIM_SOPT2_CLKOUTSEL_SHIFT) /* MCGIRCLK */
#  define SIM_SOPT2_CLKOUTSEL_OSCERCLK (6 << SIM_SOPT2_CLKOUTSEL_SHIFT) /* OSCERCLK */

#define SIM_SOPT2_CMTUARTPAD         (1 << 11) /* Bit 11: CMT/UART pad drive strength */
#define SIM_SOPT2_TRACECLKSEL        (1 << 12) /* Bit 12: Debug trace clock select */
                                               /* Bits 13-15: Reserved */
#define SIM_SOPT2_PLLFLLSEL          (1 << 16) /* Bit 16: PLL/FLL clock select */
                                               /* Bit 17: Reserved */
#define SIM_SOPT2_USBSRC             (1 << 18) /* Bit 18: USB clock source select */
                                               /* Bits 19-23: Reserved */
#define SIM_SOPT2_TPMSRC_SHIFT       (24)      /* Bits 24-25: I2S master clock source select */
#define SIM_SOPT2_TPMSRC_MASK        (3 << SIM_SOPT2_TPMSRC_SHIFT)
#  define SIM_SOPT2_TPMSRC_CLKDIS    (0 << SIM_SOPT2_TPMSRC_SHIFT) /* Core/system clock / I2S fractional divider*/
#  define SIM_SOPT2_TPMSRC_MCGCLK    (1 << SIM_SOPT2_TPMSRC_SHIFT) /* MCGFLLCLK clock or MCGPLLCLK/2 */
#  define SIM_SOPT2_TPMSRC_OCSERCLK  (2 << SIM_SOPT2_TPMSRC_SHIFT) /* OSCERCLK clock */
#  define SIM_SOPT2_TPMSRC_MCGIRCLK  (3 << SIM_SOPT2_TPMSRC_SHIFT) /* MCGIRCLK clock */

#define SIM_SOPT2_UART0SRC_SHIFT     (26)      /* Bits 26-27: UART0 clock source select */
#define SIM_SOPT2_UART0SRC_MASK      (3 << SIM_SOPT2_UART0SRC_SHIFT)
#  define SIM_SOPT2_UART0SRC_DIS      (0 << SIM_SOPT2_UART0SRC_SHIFT) /* Clock disabled */
#  define SIM_SOPT2_UART0SRC_MCGCLK   (1 << SIM_SOPT2_UART0SRC_SHIFT) /* MCGFLLCLK clock or MCGPLLCLK/2 clock */
#  define SIM_SOPT2_UART0SRC_OSCERCLK (2 << SIM_SOPT2_UART0SRC_SHIFT) /* OSCERCLK clock */
#  define SIM_SOPT2_UART0SRC_MCGIRCLK (3 << SIM_SOPT2_UART0SRC_SHIFT) /* MCGIRCLK clock */

                                               /* Bits 28-31: Reserved */

/* System Options Register 4 */

                                               /* Bits 0-17: Reserved */
#define SIM_SOPT4_TPM1CH0SRC         (1 << 18) /* Bit 18: TPM1 channel 0 input capture source select */
                                               /* Bit 19: Reserved */
#define SIM_SOPT4_TPM2CH0SRC         (1 << 20) /* Bit 20: TPM2 channel 0 input capture source select */
                                               /* Bits 21-23: Reserved */
#define SIM_SOPT4_TPM0CLKSEL         (1 << 24) /* Bit 24:  TPM0CLKSEL */
#define SIM_SOPT4_TPM1CLKSEL         (1 << 25) /* Bit 25:  TPM1 External Clock Pin Select */
#define SIM_SOPT4_TPM2CLKSEL         (1 << 26) /* Bit 26:  TPM2 External Clock Pin Select */
                                               /* Bits 27-31: Reserved */

/* System Options Register 5 */

#define SIM_SOPT5_UART0TXSRC_SHIFT   (0)       /* Bits 0-1: UART 0 transmit data source select */
#define SIM_SOPT5_UART0TXSRC_MASK    (3 << SIM_SOPT5_UART0TXSRC_SHIFT)
#  define SIM_SOPT5_UART0TXSRC_TX    (0 << SIM_SOPT5_UART0TXSRC_SHIFT) /* UART0_TX pin */
#  define SIM_SOPT5_UART0TXSRC_TPM1  (1 << SIM_SOPT5_UART0TXSRC_SHIFT) /* UART0_TX modulated with TPM1 ch0 output */
#  define SIM_SOPT5_UART0TXSRC_TPM2  (2 << SIM_SOPT5_UART0TXSRC_SHIFT) /* UART0_TX modulated with TPM2 ch0 output */

#define SIM_SOPT5_UART0RXSRC         (1 << 2)  /* Bit 2: UART 0 receive data source select */
                                               /* Bit 3: Reserved */
#define SIM_SOPT5_UART1TXSRC_SHIFT   (4)       /* Bits 4-5: UART 1 transmit data source select */
#define SIM_SOPT5_UART1TXSRC_MASK    (3 << SIM_SOPT5_UART1TXSRC_SHIFT)
#  define SIM_SOPT5_UART1TXSRC_TX    (0 << SIM_SOPT5_UART1TXSRC_SHIFT) /* UART1_TX pin */
#  define SIM_SOPT5_UART1TXSRC_TPM1  (1 << SIM_SOPT5_UART1TXSRC_SHIFT) /* UART1_TX modulated with TPM1 ch0 output */
#  define SIM_SOPT5_UART1TXSRC_TPM2  (2 << SIM_SOPT5_UART1TXSRC_SHIFT) /* UART1_TX modulated with TPM2 ch0 output */

#define SIM_SOPT5_UART1RXSRC         (1 << 6)  /* Bit 6: UART 1 receive data source select */
#define SIM_SOPT5_UART0ODE           (1 << 16) /* Bit 16: UART0 Open Drain Enable */
#define SIM_SOPT5_UART1ODE           (1 << 17) /* Bit 17: UART1 Open Drain Enable */
#define SIM_SOPT5_UART2ODE           (1 << 18) /* Bit 18: UART2 Open Drain Enable */
                                               /* Bits 20-31: Reserved */

/* System Options Register 7 */

#define SIM_SOPT7_ADC0TRGSEL_SHIFT   (0)       /* Bits 0-3: ADC0 trigger select */
#define SIM_SOPT7_ADC0TRGSEL_MASK    (15 << SIM_SOPT7_ADC0TRGSEL_SHIFT)
#  define SIM_SOPT7_ADC0TRGSEL_EXT   (0 << SIM_SOPT7_ADC0TRGSEL_SHIFT)  /* External trigger pin input (EXTRG_IN)*/
#  define SIM_SOPT7_ADC0TRGSEL_CMP0  (1 << SIM_SOPT7_ADC0TRGSEL_SHIFT)  /* CMP0 output */
#  define SIM_SOPT7_ADC0TRGSEL_PIT0  (4 << SIM_SOPT7_ADC0TRGSEL_SHIFT)  /* PIT trigger 0 */
#  define SIM_SOPT7_ADC0TRGSEL_PIT1  (5 << SIM_SOPT7_ADC0TRGSEL_SHIFT)  /* PIT trigger 1 */
#  define SIM_SOPT7_ADC0TRGSEL_TPM0  (8 << SIM_SOPT7_ADC0TRGSEL_SHIFT)  /* TPM0 overflow */
#  define SIM_SOPT7_ADC0TRGSEL_TPM1  (9 << SIM_SOPT7_ADC0TRGSEL_SHIFT)  /* TPM1 overflow */
#  define SIM_SOPT7_ADC0TRGSEL_TPM2  (10 << SIM_SOPT7_ADC0TRGSEL_SHIFT) /* TPM2 overflow */
#  define SIM_SOPT7_ADC0TRGSEL_ALARM (12 << SIM_SOPT7_ADC0TRGSEL_SHIFT) /* RTC alarm */
#  define SIM_SOPT7_ADC0TRGSEL_SECS  (13 << SIM_SOPT7_ADC0TRGSEL_SHIFT) /* RTC seconds */
#  define SIM_SOPT7_ADC0TRGSEL_LPTMR (14 << SIM_SOPT7_ADC0TRGSEL_SHIFT) /* LPTMR0 trigger */

#define SIM_SOPT7_ADC0PRETRGSEL      (1 << 4)  /* Bit 4:  ADC0 pretrigger select */
                                               /* Bits 5-6: Reserved */
#define SIM_SOPT7_ADC0ALTTRGEN       (1 << 7)  /* Bit 7:  ADC0 alternate trigger enable */
                                               /* Bits 8-31: Reserved */

/* System Device Identification Register */

#define SIM_SDID_PINID_SHIFT         (0)       /* Bits 0-3: Pincount identification */
#define SIM_SDID_PINID_MASK          (15 << SIM_SDID_PINID_SHIFT)
#  define SIM_SDID_PINID_16PIN       (0 << SIM_SDID_PINID_SHIFT)  /* 16-pin */
#  define SIM_SDID_PINID_24PIN       (1 << SIM_SDID_PINID_SHIFT)  /* 24-pin */
#  define SIM_SDID_PINID_32PIN       (2 << SIM_SDID_PINID_SHIFT)  /* 32-pin */
#  define SIM_SDID_PINID_48PIN       (4 << SIM_SDID_PINID_SHIFT)  /* 48-pin */
#  define SIM_SDID_PINID_64PIN       (5 << SIM_SDID_PINID_SHIFT)  /* 64-pin */
#  define SIM_SDID_PINID_80PIN       (6 << SIM_SDID_PINID_SHIFT)  /* 80-pin */
#  define SIM_SDID_PINID_100PIN      (8 << SIM_SDID_PINID_SHIFT)  /* 100-pin */

                                               /* Bits 406: Reserved */
#define SIM_SDID_DIEID_SHIFT         (7)       /* Bits 7-1: Device die number */
#define SIM_SDID_DIEID_MASK          (15 << SIM_SDID_DIEID_SHIFT)
#define SIM_SDID_REVID_SHIFT         (12)      /* Bits 12-15: Device revision number */
#define SIM_SDID_REVID_MASK          (15 << SIM_SDID_REVID_SHIFT)
#define SIM_SDID_SRAMSIZE_SHIFT      (16)      /* Bits 16-19: System SRAM Size */
#define SIM_SDID_SRAMSIZE_MASK       (15 << SIM_SDID_SRAMSIZE_SHIFT)
#  define SIM_SDID_SRAMSIZE_p5KB     (0 << SIM_SDID_SRAMSIZE_SHIFT) /* 0.5 KB */
#  define SIM_SDID_SRAMSIZE_1KB      (1 << SIM_SDID_SRAMSIZE_SHIFT) /* 1 KB */
#  define SIM_SDID_SRAMSIZE_2KB      (2 << SIM_SDID_SRAMSIZE_SHIFT) /* 2 KB */
#  define SIM_SDID_SRAMSIZE_4KB      (3 << SIM_SDID_SRAMSIZE_SHIFT) /* 4 KB */
#  define SIM_SDID_SRAMSIZE_8KB      (4 << SIM_SDID_SRAMSIZE_SHIFT) /* 8 KB */
#  define SIM_SDID_SRAMSIZE_16KB     (5 << SIM_SDID_SRAMSIZE_SHIFT) /* 16 KB */
#  define SIM_SDID_SRAMSIZE_32KB     (6 << SIM_SDID_SRAMSIZE_SHIFT) /* 32 KB */
#  define SIM_SDID_SRAMSIZE_64KB     (7 << SIM_SDID_SRAMSIZE_SHIFT) /* 64 KB */

#define SIM_SDID_SERIESID_SHIFT      (10)      /* Bits 20-23: Kinetis Series ID */
#define SIM_SDID_SERIESID_MASK       (15 << SIM_SDID_SERIESID_SHIFT)
#  define SIM_SDID_SERIESID_KL       (1 << SIM_SDID_SERIESID_SHIFT) /* KL family */

#define SIM_SDID_SUBFAMID_SHIFT      (24)      /* Bits 24-27: Kinetis Sub-Family ID */
#define SIM_SDID_SUBFAMID_MASK       (15 << SIM_SDID_SUBFAMID_SHIFT)
#  define SIM_SDID_SUBFAMID_KLX2     (2 << SIM_SDID_SUBFAMID_SHIFT) /* KLx2 Subfamily (low end) */
#  define SIM_SDID_SUBFAMID_KLX4     (4 << SIM_SDID_SUBFAMID_SHIFT) /* KLx4 Subfamily (basic analog) */
#  define SIM_SDID_SUBFAMID_KLX5     (5 << SIM_SDID_SUBFAMID_SHIFT) /* KLx5 Subfamily (advanced analog) */
#  define SIM_SDID_SUBFAMID_KLX6     (6 << SIM_SDID_SUBFAMID_SHIFT) /* KL3x KLx6 Subfamily (advanced analog with I2S) */

#define SIM_SDID_FAMID_SHIFT         (28)     /* Bits 28-31: Kinetis family ID */
#define SIM_SDID_FAMID_MASK          (15 << SIM_SDID_FAMID_SHIFT)
#  define SIM_SDID_FAMID_KL0         (0 << SIM_SDID_FAMID_SHIFT) /* KL0x Family (low end) */
#  define SIM_SDID_FAMID_KL1         (1 << SIM_SDID_FAMID_SHIFT) /* KL1x Family (basic) */
#  define SIM_SDID_FAMID_KL2         (2 << SIM_SDID_FAMID_SHIFT) /* KL2x Family (USB) */
#  define SIM_SDID_FAMID_KL3         (3 << SIM_SDID_FAMID_SHIFT) /* KL3x Family (Segment LCD) */
#  define SIM_SDID_FAMID_KL4         (4 << SIM_SDID_FAMID_SHIFT) /* KL4x Family (USB and Segment LCD) */

/* System Clock Gating Control Register 4 */

                                               /* Bits 0-5: Reserved */
#define SIM_SCGC4_I2C0               (1 << 6)  /* Bit 6:  I2C0 Clock Gate Control */
#define SIM_SCGC4_I2C1               (1 << 7)  /* Bit 7:  I2C1 Clock Gate Control */
                                               /* Bits 8-9: Reserved */
#define SIM_SCGC4_UART0              (1 << 10) /* Bit 10: UART0 Clock Gate Control */
#define SIM_SCGC4_UART1              (1 << 11) /* Bit 11: UART1 Clock Gate Control */
#define SIM_SCGC4_UART2              (1 << 12) /* Bit 12: UART2 Clock Gate Control */
                                               /* Bits 13-17: Reserved */
#define SIM_SCGC4_USBOTG             (1 << 18) /* Bit 18: USB Clock Gate Control */
#define SIM_SCGC4_CMP                (1 << 19) /* Bit 19: Comparator Clock Gate Control */
                                               /* Bits 20-21: Reserved */
#define SIM_SCGC4_SPI0               (1 << 22) /* Bit 22: SPI0 Clock Gate Control */
#define SIM_SCGC4_SPI1               (1 << 23) /* Bit 23: SPI1 Clock Gate Control */
                                               /* Bits 24-31: Reserved */

/* System Clock Gating Control Register 5 */

#define SIM_SCGC5_LPTIMER            (1 << 0)  /* Bit 0:  Low Power Timer Clock Gate Control */
                                               /* Bits 1-4: Reserved */
#define SIM_SCGC5_TSI                (1 << 5)  /* Bit 5:  TSI Clock Gate Control */
                                               /* Bits 6-8: Reserved */
#define SIM_SCGC5_PORTA              (1 << 9)  /* Bit 9:  Port A Clock Gate Control */
#define SIM_SCGC5_PORTB              (1 << 10) /* Bit 10: Port B Clock Gate Control */
#define SIM_SCGC5_PORTC              (1 << 11) /* Bit 11: Port C Clock Gate Control */
#define SIM_SCGC5_PORTD              (1 << 12) /* Bit 12: Port D Clock Gate Control */
#define SIM_SCGC5_PORTE              (1 << 13) /* Bit 13: Port E Clock Gate Control */
                                               /* Bits 14-31: Reserved */

/* System Clock Gating Control Register 6 */

#define SIM_SCGC6_FTFL               (1 << 0)  /* Bit 0:  Flash Memory Clock Gate Control */
#define SIM_SCGC6_DMAMUX             (1 << 1)  /* Bit 1:  DMA Mux Clock Gate Control */
                                               /* Bits 2-22: Reserved */
#define SIM_SCGC6_PIT                (1 << 23) /* Bit 23: PIT Clock Gate Control */
#define SIM_SCGC6_TPM0               (1 << 24) /* Bit 24: TPM0 Clock Gate Control */
#define SIM_SCGC6_TPM1               (1 << 25) /* Bit 25: TPM1 Clock Gate Control */
#define SIM_SCGC6_TPM2               (1 << 26) /* Bit 25: TPM2 Clock Gate Control */
#define SIM_SCGC6_ADC0               (1 << 27) /* Bit 27: ADC0 Clock Gate Control */
                                               /* Bit 28: Reserved */
#define SIM_SCGC6_RTC                (1 << 29) /* Bit 29: RTC Clock Gate Control */
                                               /* Bit 30: Reserved */
#define SIM_SCGC6_DAC0               (1 << 31) /* Bit 29: DAC0 Clock Gate Control */

/* System Clock Gating Control Register 7 */

                                               /* Bits 0-7: Reserved */
#define SIM_SCGC7_DMA                (1 << 8)  /* Bit 8:  DMA Clock Gate Control */
                                               /* Bits 9-31: Reserved */

/* System Clock Divider Register 1 */

                                               /* Bits 0-15: Reserved */
#define SIM_CLKDIV1_OUTDIV4_SHIFT    (16)      /* Bits 16-18: Clock 4 output divider value */
#define SIM_CLKDIV1_OUTDIV4_MASK     (7 << SIM_CLKDIV1_OUTDIV4_SHIFT)
#  define SIM_CLKDIV1_OUTDIV4(n)     (((n)-1) << SIM_CLKDIV1_OUTDIV4_SHIFT) /* Divide by n, n=1..16 */

#  define SIM_CLKDIV1_OUTDIV4_1      (0 << SIM_CLKDIV1_OUTDIV4_SHIFT)  /* Divide by 1 */
#  define SIM_CLKDIV1_OUTDIV4_2      (1 << SIM_CLKDIV1_OUTDIV4_SHIFT)  /* Divide by 2 */
#  define SIM_CLKDIV1_OUTDIV4_3      (2 << SIM_CLKDIV1_OUTDIV4_SHIFT)  /* Divide by 3 */
#  define SIM_CLKDIV1_OUTDIV4_4      (3 << SIM_CLKDIV1_OUTDIV4_SHIFT)  /* Divide by 4 */
#  define SIM_CLKDIV1_OUTDIV4_5      (4 << SIM_CLKDIV1_OUTDIV4_SHIFT)  /* Divide by 5 */
#  define SIM_CLKDIV1_OUTDIV4_6      (5 << SIM_CLKDIV1_OUTDIV4_SHIFT)  /* Divide by 6 */
#  define SIM_CLKDIV1_OUTDIV4_7      (6 << SIM_CLKDIV1_OUTDIV4_SHIFT)  /* Divide by 7 */
#  define SIM_CLKDIV1_OUTDIV4_8      (7 << SIM_CLKDIV1_OUTDIV4_SHIFT)  /* Divide by 8 */

                                               /* Bits 19-27: Reserved */
#define SIM_CLKDIV1_OUTDIV1_SHIFT    (28)      /* Bits 28-31: Clock 1 output divider value */
#define SIM_CLKDIV1_OUTDIV1_MASK     (15 << SIM_CLKDIV1_OUTDIV1_SHIFT)
#  define SIM_CLKDIV1_OUTDIV1(n)     (((n)-1) << SIM_CLKDIV1_OUTDIV1_SHIFT) /* Divide by n, n=1..16 */

#  define SIM_CLKDIV1_OUTDIV1_1      (0 << SIM_CLKDIV1_OUTDIV1_SHIFT)  /* Divide by 1 */
#  define SIM_CLKDIV1_OUTDIV1_2      (1 << SIM_CLKDIV1_OUTDIV1_SHIFT)  /* Divide by 2 */
#  define SIM_CLKDIV1_OUTDIV1_3      (2 << SIM_CLKDIV1_OUTDIV1_SHIFT)  /* Divide by 3 */
#  define SIM_CLKDIV1_OUTDIV1_4      (3 << SIM_CLKDIV1_OUTDIV1_SHIFT)  /* Divide by 4 */
#  define SIM_CLKDIV1_OUTDIV1_5      (4 << SIM_CLKDIV1_OUTDIV1_SHIFT)  /* Divide by 5 */
#  define SIM_CLKDIV1_OUTDIV1_6      (5 << SIM_CLKDIV1_OUTDIV1_SHIFT)  /* Divide by 6 */
#  define SIM_CLKDIV1_OUTDIV1_7      (6 << SIM_CLKDIV1_OUTDIV1_SHIFT)  /* Divide by 7 */
#  define SIM_CLKDIV1_OUTDIV1_8      (7 << SIM_CLKDIV1_OUTDIV1_SHIFT)  /* Divide by 8 */
#  define SIM_CLKDIV1_OUTDIV1_9      (8 << SIM_CLKDIV1_OUTDIV1_SHIFT)  /* Divide by 9 */
#  define SIM_CLKDIV1_OUTDIV1_10     (9 << SIM_CLKDIV1_OUTDIV1_SHIFT)  /* Divide by 10 */
#  define SIM_CLKDIV1_OUTDIV1_11     (10 << SIM_CLKDIV1_OUTDIV1_SHIFT) /* Divide by 11 */
#  define SIM_CLKDIV1_OUTDIV1_12     (11 << SIM_CLKDIV1_OUTDIV1_SHIFT) /* Divide by 12 */
#  define SIM_CLKDIV1_OUTDIV1_13     (12 << SIM_CLKDIV1_OUTDIV1_SHIFT) /* Divide by 13 */
#  define SIM_CLKDIV1_OUTDIV1_14     (13 << SIM_CLKDIV1_OUTDIV1_SHIFT) /* Divide by 14 */
#  define SIM_CLKDIV1_OUTDIV1_15     (14 << SIM_CLKDIV1_OUTDIV1_SHIFT) /* Divide by 15 */
#  define SIM_CLKDIV1_OUTDIV1_16     (15 << SIM_CLKDIV1_OUTDIV1_SHIFT) /* Divide by 16 */

/* Flash Configuration Register 1 */

#define SIM_FCFG1_FLASHDIS           (1 << 0)  /* Bit 0: Flash Disable */
#define SIM_FCFG1_FLASHDOZE          (1 << 1)  /* Bit 1: Flash Doze */
                                               /* Bits 2-23: Reserved */
#define SIM_FCFG1_PFSIZE_SHIFT       (24)      /* Bits 24-27: Program flash size */
#define SIM_FCFG1_PFSIZE_MASK        (15 << SIM_FCFG1_PFSIZE_SHIFT)
#  define SIM_FCFG1_PFSIZE_8KB       (0 << SIM_FCFG1_PFSIZE_SHIFT)  /* 8 KB of program flash memory,
                                                                     * 0.25 KB protection region */
#  define SIM_FCFG1_PFSIZE_16KB      (1 << SIM_FCFG1_PFSIZE_SHIFT)  /* 16 KB of program flash memory,
                                                                     * 0.5 KB protection region */
#  define SIM_FCFG1_PFSIZE_32KB      (3 << SIM_FCFG1_PFSIZE_SHIFT)  /* 32 KB of program flash memory,
                                                                     * 1 KB protection region */
#  define SIM_FCFG1_PFSIZE_64KB      (5 << SIM_FCFG1_PFSIZE_SHIFT)  /* 64 KB of program flash memory,
                                                                     * 2 KB protection region */
#  define SIM_FCFG1_PFSIZE_128KB     (7 << SIM_FCFG1_PFSIZE_SHIFT)  /* 128 KB of program flash memory,
                                                                     * 4 KB protection region */
#  define SIM_FCFG1_PFSIZE_256KB     (9 << SIM_FCFG1_PFSIZE_SHIFT)  /* 256 KB of program flash memory,
                                                                     * 8 KB protection region */

                                               /* Bits 28-31: Reserved */

/* Flash Configuration Register 2 */

                                               /* Bits 0-15: Reserved */
#define SIM_FCFG2_MAXADDR0_SHIFT     (24)      /* Bits 24-30: Max address block */
#define SIM_FCFG2_MAXADDR0_MASK      (0x7f << SIM_FCFG2_MAXADDR0_SHIFT)
                                               /* Bit 31: Reserved */

/* Unique Identification Register High. 16-bit Unique Identification. */

/* Unique Identification Register Mid-High. 32-bit Unique Identification. */

/* Unique Identification Register Mid Low. 32-bit Unique Identification. */

/* Unique Identification Register Low. 32-bit Unique Identification. */

/* COP Control Register */

#define SIM_COPC_COPW                (1 << 0)  /* Bit 2:  COP windowed mode */
#define SIM_COPC_COPCLKS             (1 << 1)  /* Bit 1:  COP Clock Select */
#define SIM_COPC_COPT_SHIFT          (2)       /* Bits 2-3: COP Watchdog Timeout */
#define SIM_COPC_COPT_MASK           (3 << SIM_COPC_COPT_SHIFT)
#  define SIM_COPC_COPT_DISABLED     (0 << SIM_COPC_COPT_SHIFT) /* COP disabled */
#  define SIM_COPC_COPT_TO13         (1 << SIM_COPC_COPT_SHIFT) /* COP timeout after 2^5 LPO cycles or
                                                                 * 2^13 bus clock cycles */
#  define SIM_COPC_COPT_TO16         (2 << SIM_COPC_COPT_SHIFT) /* COP timeout after 2^8 LPO cycles or
                                                                 * 2^16 bus clock cycles */
#  define SIM_COPC_COPT_TO18         (3 << SIM_COPC_COPT_SHIFT) /* COP timeout after 2^10 LPO cycles or
                                                                 * 2^18 bus clock cycles */

/* Service COP Register. 8-bit value. */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/
#endif /* __ARCH_ARM_SRC_KL_HARDWARE_KL_SIM_H */
