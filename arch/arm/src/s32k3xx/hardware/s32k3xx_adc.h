/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_adc.h
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

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_ADC_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/s32k3xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ADC Register Offsets *****************************************************/

#define S32K3XX_ADC_MCR_OFFSET            (0x0000) /* Main Configuration Register (MCR) */
#define S32K3XX_ADC_MSR_OFFSET            (0x0004) /* Main Status Register (MSR) */
#define S32K3XX_ADC_ISR_OFFSET            (0x0010) /* Interrupt Status Register (ISR) */
#define S32K3XX_ADC_CEOCFR0_OFFSET        (0x0014) /* Channel End Of Conversation Flag Register for Precision Inputs (CEOCFR0) */
#define S32K3XX_ADC_CEOCFR1_OFFSET        (0x0018) /* Channel End Of Conversation Flag Register for Standard Inputs (CEOCFR1) */
#define S32K3XX_ADC_CEOCFR2_OFFSET        (0x001c) /* Channel End Of Conversation Flag Register for External Inputs (CEOCFR2) */
#define S32K3XX_ADC_IMR_OFFSET            (0x0020) /* Interrupt Mask Register (IMR) */
#define S32K3XX_ADC_CIMR0_OFFSET          (0x0024) /* EOC Interrupt Enable Register for Precision Inputs (CIMR0) */
#define S32K3XX_ADC_CIMR1_OFFSET          (0x0028) /* EOC Interrupt Enable Register for Standard Inputs (CIMR1) */
#define S32K3XX_ADC_CIMR2_OFFSET          (0x002c) /* EOC Interrupt Enable Register for External Inputs (CIMR2) */
#define S32K3XX_ADC_WTISR_OFFSET          (0x0030) /* Analog Watchdog Threshold Interrupt Status Register (WTISR) */
#define S32K3XX_ADC_WTIMR_OFFSET          (0x0034) /* Analog Watchdog Threshold Interrupt Enable Register (WTIMR) */
#define S32K3XX_ADC_DMAE_OFFSET           (0x0040) /* Direct Memory Access Configuration Register (DMAE) */
#define S32K3XX_ADC_DMAR0_OFFSET          (0x0044) /* DMA Request Enable Register for Precision Inputs (DMAR0) */
#define S32K3XX_ADC_DMAR1_OFFSET          (0x0048) /* DMA Request Enable Register for Standard Inputs (DMAR1) */
#define S32K3XX_ADC_DMAR2_OFFSET          (0x004c) /* DMA Request Enable Register for External Inputs (DMAR2) */
#define S32K3XX_ADC_THRHLR0_OFFSET        (0x0060) /* Analog Watchdog Threshold Values Register 0 (THRHLR0) */
#define S32K3XX_ADC_THRHLR1_OFFSET        (0x0064) /* Analog Watchdog Threshold Values Register 1 (THRHLR1) */
#define S32K3XX_ADC_THRHLR2_OFFSET        (0x0068) /* Analog Watchdog Threshold Values Register 2 (THRHLR2) */
#define S32K3XX_ADC_THRHLR3_OFFSET        (0x006c) /* Analog Watchdog Threshold Values Register 3 (THRHLR3) */
#define S32K3XX_ADC_PSCR_OFFSET           (0x0080) /* Presampling Control Register (PSCR) */
#define S32K3XX_ADC_PSR0_OFFSET           (0x0084) /* Presampling Enable Register for Precision Inputs (PSR0) */
#define S32K3XX_ADC_PSR1_OFFSET           (0x0088) /* Presampling Enable Register for Standard Inputs (PSR1) */
#define S32K3XX_ADC_PSR2_OFFSET           (0x008c) /* Presampling Enable Register for External Inputs (PSR2) */
#define S32K3XX_ADC_CTR0_OFFSET           (0x0094) /* Conversion Timing Register for Precision Inputs (CTR0) */
#define S32K3XX_ADC_CTR1_OFFSET           (0x0098) /* Conversion Timing Register for Standard Inputs (CTR1) */
#define S32K3XX_ADC_CTR2_OFFSET           (0x009c) /* Conversion Timing Register for External Inputs (CTR2) */
#define S32K3XX_ADC_NCMR0_OFFSET          (0x00a4) /* Normal Conversion Enable Register for Precision Inputs (NCMR0) */
#define S32K3XX_ADC_NCMR1_OFFSET          (0x00a8) /* Normal Conversion Enable Register for Standard Inputs (NCMR1) */
#define S32K3XX_ADC_NCMR2_OFFSET          (0x00ac) /* Normal Conversion Enable Register for External Inputs (NCMR2) */
#define S32K3XX_ADC_JCMR0_OFFSET          (0x00b4) /* Injected Conversion Enable Register for Precision Inputs (JCMR0) */
#define S32K3XX_ADC_JCMR1_OFFSET          (0x00b8) /* Injected Conversion Enable Register for Standard Inputs (JCMR1) */
#define S32K3XX_ADC_JCMR2_OFFSET          (0x00bc) /* Injected Conversion Enable Register for External Inputs (JCMR2) */
#define S32K3XX_ADC_DSDR_OFFSET           (0x00c4) /* Delay Start of Data Conversion Register (DSDR) */
#define S32K3XX_ADC_PDEDR_OFFSET          (0x00c8) /* Power Down Exit Delay Register (PDEDR) */

#define S32K3XX_ADC_PCDR_OFFSET(n)        (0x0100 + ((n) << 2))
#define S32K3XX_ADC_PCDR0_OFFSET          (0x0100) /* Precision Input 0 Conversion Data Register (PCDR0) */
#define S32K3XX_ADC_PCDR1_OFFSET          (0x0104) /* Precision Input 1 Conversion Data Register (PCDR1) */
#define S32K3XX_ADC_PCDR2_OFFSET          (0x0108) /* Precision Input 2 Conversion Data Register (PCDR2) */
#define S32K3XX_ADC_PCDR3_OFFSET          (0x010c) /* Precision Input 3 Conversion Data Register (PCDR3) */
#define S32K3XX_ADC_PCDR4_OFFSET          (0x0110) /* Precision Input 4 Conversion Data Register (PCDR4) */
#define S32K3XX_ADC_PCDR5_OFFSET          (0x0114) /* Precision Input 5 Conversion Data Register (PCDR5) */
#define S32K3XX_ADC_PCDR6_OFFSET          (0x0118) /* Precision Input 6 Conversion Data Register (PCDR6) */
#define S32K3XX_ADC_PCDR7_OFFSET          (0x011c) /* Precision Input 7 Conversion Data Register (PCDR7) */

#define S32K3XX_ADC_ICDR_OFFSET(n)        (0x0180 + ((n) << 2))
#define S32K3XX_ADC_ICDR0_OFFSET          (0x0180) /* Standard Input 0 Conversion Data Register (ICDR0) */
#define S32K3XX_ADC_ICDR1_OFFSET          (0x0184) /* Standard Input 1 Conversion Data Register (ICDR1) */
#define S32K3XX_ADC_ICDR2_OFFSET          (0x0188) /* Standard Input 2 Conversion Data Register (ICDR2) */
#define S32K3XX_ADC_ICDR3_OFFSET          (0x018c) /* Standard Input 3 Conversion Data Register (ICDR3) */
#define S32K3XX_ADC_ICDR4_OFFSET          (0x0190) /* Standard Input 4 Conversion Data Register (ICDR4) */
#define S32K3XX_ADC_ICDR5_OFFSET          (0x0194) /* Standard Input 5 Conversion Data Register (ICDR5) */
#define S32K3XX_ADC_ICDR6_OFFSET          (0x0198) /* Standard Input 6 Conversion Data Register (ICDR6) */
#define S32K3XX_ADC_ICDR7_OFFSET          (0x019c) /* Standard Input 7 Conversion Data Register (ICDR7) */
#define S32K3XX_ADC_ICDR8_OFFSET          (0x01a0) /* Standard Input 8 Conversion Data Register (ICDR8) */
#define S32K3XX_ADC_ICDR9_OFFSET          (0x01a4) /* Standard Input 9 Conversion Data Register (ICDR9) */
#define S32K3XX_ADC_ICDR10_OFFSET         (0x01a8) /* Standard Input 10 Conversion Data Register (ICDR10) */
#define S32K3XX_ADC_ICDR11_OFFSET         (0x01ac) /* Standard Input 11 Conversion Data Register (ICDR11) */
#define S32K3XX_ADC_ICDR12_OFFSET         (0x01b0) /* Standard Input 12 Conversion Data Register (ICDR12) */
#define S32K3XX_ADC_ICDR13_OFFSET         (0x01b4) /* Standard Input 13 Conversion Data Register (ICDR13) */
#define S32K3XX_ADC_ICDR14_OFFSET         (0x01b8) /* Standard Input 14 Conversion Data Register (ICDR14) */
#define S32K3XX_ADC_ICDR15_OFFSET         (0x01bc) /* Standard Input 15 Conversion Data Register (ICDR15) */
#define S32K3XX_ADC_ICDR16_OFFSET         (0x01c0) /* Standard Input 16 Conversion Data Register (ICDR16) */
#define S32K3XX_ADC_ICDR17_OFFSET         (0x01c4) /* Standard Input 17 Conversion Data Register (ICDR17) */
#define S32K3XX_ADC_ICDR18_OFFSET         (0x01c8) /* Standard Input 18 Conversion Data Register (ICDR18) */
#define S32K3XX_ADC_ICDR19_OFFSET         (0x01cc) /* Standard Input 19 Conversion Data Register (ICDR19) */
#define S32K3XX_ADC_ICDR20_OFFSET         (0x01d0) /* Standard Input 20 Conversion Data Register (ICDR20) */
#define S32K3XX_ADC_ICDR21_OFFSET         (0x01d4) /* Standard Input 21 Conversion Data Register (ICDR21) */
#define S32K3XX_ADC_ICDR22_OFFSET         (0x01d8) /* Standard Input 22 Conversion Data Register (ICDR22) */
#define S32K3XX_ADC_ICDR23_OFFSET         (0x01dc) /* Standard Input 23 Conversion Data Register (ICDR23) */

#define S32K3XX_ADC_ECDR_OFFSET(n)        (0x0200 + ((n) << 2))
#define S32K3XX_ADC_ECDR0_OFFSET          (0x0200) /* External Input 0 Conversion Data Register (ECDR0) */
#define S32K3XX_ADC_ECDR1_OFFSET          (0x0204) /* External Input 1 Conversion Data Register (ECDR1) */
#define S32K3XX_ADC_ECDR2_OFFSET          (0x0208) /* External Input 2 Conversion Data Register (ECDR2) */
#define S32K3XX_ADC_ECDR3_OFFSET          (0x020c) /* External Input 3 Conversion Data Register (ECDR3) */
#define S32K3XX_ADC_ECDR4_OFFSET          (0x0210) /* External Input 4 Conversion Data Register (ECDR4) */
#define S32K3XX_ADC_ECDR5_OFFSET          (0x0214) /* External Input 5 Conversion Data Register (ECDR5) */
#define S32K3XX_ADC_ECDR6_OFFSET          (0x0218) /* External Input 6 Conversion Data Register (ECDR6) */
#define S32K3XX_ADC_ECDR7_OFFSET          (0x021c) /* External Input 7 Conversion Data Register (ECDR7) */
#define S32K3XX_ADC_ECDR8_OFFSET          (0x0220) /* External Input 8 Conversion Data Register (ECDR8) */
#define S32K3XX_ADC_ECDR9_OFFSET          (0x0224) /* External Input 9 Conversion Data Register (ECDR9) */
#define S32K3XX_ADC_ECDR10_OFFSET         (0x0228) /* External Input 10 Conversion Data Register (ECDR10) */
#define S32K3XX_ADC_ECDR11_OFFSET         (0x022c) /* External Input 11 Conversion Data Register (ECDR11) */
#define S32K3XX_ADC_ECDR12_OFFSET         (0x0230) /* External Input 12 Conversion Data Register (ECDR12) */
#define S32K3XX_ADC_ECDR13_OFFSET         (0x0234) /* External Input 13 Conversion Data Register (ECDR13) */
#define S32K3XX_ADC_ECDR14_OFFSET         (0x0238) /* External Input 14 Conversion Data Register (ECDR14) */
#define S32K3XX_ADC_ECDR15_OFFSET         (0x023c) /* External Input 15 Conversion Data Register (ECDR15) */
#define S32K3XX_ADC_ECDR16_OFFSET         (0x0240) /* External Input 16 Conversion Data Register (ECDR16) */
#define S32K3XX_ADC_ECDR17_OFFSET         (0x0244) /* External Input 17 Conversion Data Register (ECDR17) */
#define S32K3XX_ADC_ECDR18_OFFSET         (0x0248) /* External Input 18 Conversion Data Register (ECDR18) */
#define S32K3XX_ADC_ECDR19_OFFSET         (0x024c) /* External Input 19 Conversion Data Register (ECDR19) */
#define S32K3XX_ADC_ECDR20_OFFSET         (0x0250) /* External Input 20 Conversion Data Register (ECDR20) */
#define S32K3XX_ADC_ECDR21_OFFSET         (0x0254) /* External Input 21 Conversion Data Register (ECDR21) */
#define S32K3XX_ADC_ECDR22_OFFSET         (0x0258) /* External Input 22 Conversion Data Register (ECDR22) */
#define S32K3XX_ADC_ECDR23_OFFSET         (0x025c) /* External Input 23 Conversion Data Register (ECDR23) */
#define S32K3XX_ADC_ECDR24_OFFSET         (0x0260) /* External Input 24 Conversion Data Register (ECDR24) */
#define S32K3XX_ADC_ECDR25_OFFSET         (0x0264) /* External Input 25 Conversion Data Register (ECDR25) */
#define S32K3XX_ADC_ECDR26_OFFSET         (0x0268) /* External Input 26 Conversion Data Register (ECDR26) */
#define S32K3XX_ADC_ECDR27_OFFSET         (0x026c) /* External Input 27 Conversion Data Register (ECDR27) */
#define S32K3XX_ADC_ECDR28_OFFSET         (0x0270) /* External Input 28 Conversion Data Register (ECDR28) */
#define S32K3XX_ADC_ECDR29_OFFSET         (0x0274) /* External Input 29 Conversion Data Register (ECDR29) */
#define S32K3XX_ADC_ECDR30_OFFSET         (0x0278) /* External Input 30 Conversion Data Register (ECDR30) */
#define S32K3XX_ADC_ECDR31_OFFSET         (0x027c) /* External Input 31 Conversion Data Register (ECDR31) */

#define S32K3XX_ADC_CWSELRPI0_OFFSET      (0x02b0) /* Channel Analog Watchdog Select Register for Precision Inputs 0 (CWSELRPI0) */
#define S32K3XX_ADC_CWSELRPI1_OFFSET      (0x02b4) /* Channel Analog Watchdog Select Register for Precision Inputs 1 (CWSELRPI1) */
#define S32K3XX_ADC_CWSELRSI0_OFFSET      (0x02c0) /* Channel Analog Watchdog Select Register for Standard Inputs 0 (CWSELRSI0) */
#define S32K3XX_ADC_CWSELRSI1_OFFSET      (0x02c4) /* Channel Analog Watchdog Select Register for Standard Inputs 1 (CWSELRSI1) */
#define S32K3XX_ADC_CWSELRSI2_OFFSET      (0x02c8) /* Channel Analog Watchdog Select Register for Standard Inputs 2 (CWSELRSI2) */
#define S32K3XX_ADC_CWSELREI0_OFFSET      (0x02d0) /* Channel Analog Watchdog Select Register for External Inputs 0 (CWSELREI0) */
#define S32K3XX_ADC_CWSELREI1_OFFSET      (0x02d4) /* Channel Analog Watchdog Select Register for External Inputs 1 (CWSELREI1) */
#define S32K3XX_ADC_CWSELREI2_OFFSET      (0x02d8) /* Channel Analog Watchdog Select Register for External Inputs 2 (CWSELREI2) */
#define S32K3XX_ADC_CWSELREI3_OFFSET      (0x02dc) /* Channel Analog Watchdog Select Register for External Inputs 3 (CWSELREI3) */
#define S32K3XX_ADC_CWENR0_OFFSET         (0x02e0) /* Channel Watchdog Enable Register for Precision Inputs (CWENR0) */
#define S32K3XX_ADC_CWENR1_OFFSET         (0x02e4) /* Channel Watchdog Enable Register for Standard Inputs (CWENR1) */
#define S32K3XX_ADC_CWENR2_OFFSET         (0x02e8) /* Channel Watchdog Enable Register for External Inputs (CWENR2) */
#define S32K3XX_ADC_AWORR0_OFFSET         (0x02f0) /* Analog Watchdog Out of Range Register for Precision Inputs (AWORR0) */
#define S32K3XX_ADC_AWORR1_OFFSET         (0x02f4) /* Analog Watchdog Out of Range Register for Standard Inputs (AWORR1) */
#define S32K3XX_ADC_AWORR2_OFFSET         (0x02f8) /* Analog Watchdog Out of Range Register for External Inputs (AWORR2) */
#define S32K3XX_ADC_STCR1_OFFSET          (0x0340) /* Self-Test Configuration Register 1 (STCR1) */
#define S32K3XX_ADC_STCR2_OFFSET          (0x0344) /* Self-Test Configuration Register 2 (STCR2) */
#define S32K3XX_ADC_STCR3_OFFSET          (0x0348) /* Self-Test Configuration Register 3 (STCR3) */
#define S32K3XX_ADC_STBRR_OFFSET          (0x034c) /* Self-Test Baud Rate Register (STBRR) */
#define S32K3XX_ADC_STSR1_OFFSET          (0x0350) /* Self-Test Status Register 1 (STSR1) */
#define S32K3XX_ADC_STSR2_OFFSET          (0x0354) /* Self-Test Status Register 2 (STSR2) */
#define S32K3XX_ADC_STSR3_OFFSET          (0x0358) /* Self-Test Status Register 3 (STSR3) */
#define S32K3XX_ADC_STSR4_OFFSET          (0x035c) /* Self-Test Status Register 4 (STSR4) */
#define S32K3XX_ADC_STDR1_OFFSET          (0x0370) /* Self-Test Conversion Data Register 1 (STDR1) */
#define S32K3XX_ADC_STAW0R_OFFSET         (0x0380) /* Self-Test Analog Watchdog S0 Register (STAW0R) */
#define S32K3XX_ADC_STAW1R_OFFSET         (0x0388) /* Self-Test Analog Watchdog S1 Register (STAW1R) */
#define S32K3XX_ADC_STAW2R_OFFSET         (0x038c) /* Self-Test Analog Watchdog S2 Register (STAW2R) */
#define S32K3XX_ADC_STAW4R_OFFSET         (0x0394) /* Self-Test Analog Watchdog C0 Register (STAW4R) */
#define S32K3XX_ADC_STAW5R_OFFSET         (0x0398) /* Self-Test Analog Watchdog C Register (STAW5R) */
#define S32K3XX_ADC_AMSIO_OFFSET          (0x039c) /* Analog Miscellaneous In/Out Register (AMSIO) */
#define S32K3XX_ADC_CALBISTREG_OFFSET     (0x03a0) /* Control and Calibration Status Register (CALBISTREG) */
#define S32K3XX_ADC_OFSGNUSR_OFFSET       (0x03a8) /* Offset and Gain User Register (OFSGNUSR) */
#define S32K3XX_ADC_CAL2_OFFSET           (0x03b4) /* Calibration Value 2 (CAL2) */

/* ADC Register Addresses ***************************************************/

/* ADC0 */

#define S32K3XX_ADC0_MCR                  (S32K3XX_ADC0_BASE + S32K3XX_ADC_MCR_OFFSET)
#define S32K3XX_ADC0_MSR                  (S32K3XX_ADC0_BASE + S32K3XX_ADC_MSR_OFFSET)
#define S32K3XX_ADC0_ISR                  (S32K3XX_ADC0_BASE + S32K3XX_ADC_ISR_OFFSET)
#define S32K3XX_ADC0_CEOCFR0              (S32K3XX_ADC0_BASE + S32K3XX_ADC_CEOCFR0_OFFSET)
#define S32K3XX_ADC0_CEOCFR1              (S32K3XX_ADC0_BASE + S32K3XX_ADC_CEOCFR1_OFFSET)
#define S32K3XX_ADC0_CEOCFR2              (S32K3XX_ADC0_BASE + S32K3XX_ADC_CEOCFR2_OFFSET)
#define S32K3XX_ADC0_IMR                  (S32K3XX_ADC0_BASE + S32K3XX_ADC_IMR_OFFSET)
#define S32K3XX_ADC0_CIMR0                (S32K3XX_ADC0_BASE + S32K3XX_ADC_CIMR0_OFFSET)
#define S32K3XX_ADC0_CIMR1                (S32K3XX_ADC0_BASE + S32K3XX_ADC_CIMR1_OFFSET)
#define S32K3XX_ADC0_CIMR2                (S32K3XX_ADC0_BASE + S32K3XX_ADC_CIMR2_OFFSET)
#define S32K3XX_ADC0_WTISR                (S32K3XX_ADC0_BASE + S32K3XX_ADC_WTISR_OFFSET)
#define S32K3XX_ADC0_WTIMR                (S32K3XX_ADC0_BASE + S32K3XX_ADC_WTIMR_OFFSET)
#define S32K3XX_ADC0_DMAE                 (S32K3XX_ADC0_BASE + S32K3XX_ADC_DMAE_OFFSET)
#define S32K3XX_ADC0_DMAR0                (S32K3XX_ADC0_BASE + S32K3XX_ADC_DMAR0_OFFSET)
#define S32K3XX_ADC0_DMAR1                (S32K3XX_ADC0_BASE + S32K3XX_ADC_DMAR1_OFFSET)
#define S32K3XX_ADC0_DMAR2                (S32K3XX_ADC0_BASE + S32K3XX_ADC_DMAR2_OFFSET)
#define S32K3XX_ADC0_THRHLR0              (S32K3XX_ADC0_BASE + S32K3XX_ADC_THRHLR0_OFFSET)
#define S32K3XX_ADC0_THRHLR1              (S32K3XX_ADC0_BASE + S32K3XX_ADC_THRHLR1_OFFSET)
#define S32K3XX_ADC0_THRHLR2              (S32K3XX_ADC0_BASE + S32K3XX_ADC_THRHLR2_OFFSET)
#define S32K3XX_ADC0_THRHLR3              (S32K3XX_ADC0_BASE + S32K3XX_ADC_THRHLR3_OFFSET)
#define S32K3XX_ADC0_PSCR                 (S32K3XX_ADC0_BASE + S32K3XX_ADC_PSCR_OFFSET)
#define S32K3XX_ADC0_PSR0                 (S32K3XX_ADC0_BASE + S32K3XX_ADC_PSR0_OFFSET)
#define S32K3XX_ADC0_PSR1                 (S32K3XX_ADC0_BASE + S32K3XX_ADC_PSR1_OFFSET)
#define S32K3XX_ADC0_PSR2                 (S32K3XX_ADC0_BASE + S32K3XX_ADC_PSR2_OFFSET)
#define S32K3XX_ADC0_CTR0                 (S32K3XX_ADC0_BASE + S32K3XX_ADC_CTR0_OFFSET)
#define S32K3XX_ADC0_CTR1                 (S32K3XX_ADC0_BASE + S32K3XX_ADC_CTR1_OFFSET)
#define S32K3XX_ADC0_CTR2                 (S32K3XX_ADC0_BASE + S32K3XX_ADC_CTR2_OFFSET)
#define S32K3XX_ADC0_NCMR0                (S32K3XX_ADC0_BASE + S32K3XX_ADC_NCMR0_OFFSET)
#define S32K3XX_ADC0_NCMR1                (S32K3XX_ADC0_BASE + S32K3XX_ADC_NCMR1_OFFSET)
#define S32K3XX_ADC0_NCMR2                (S32K3XX_ADC0_BASE + S32K3XX_ADC_NCMR2_OFFSET)
#define S32K3XX_ADC0_JCMR0                (S32K3XX_ADC0_BASE + S32K3XX_ADC_JCMR0_OFFSET)
#define S32K3XX_ADC0_JCMR1                (S32K3XX_ADC0_BASE + S32K3XX_ADC_JCMR1_OFFSET)
#define S32K3XX_ADC0_JCMR2                (S32K3XX_ADC0_BASE + S32K3XX_ADC_JCMR2_OFFSET)
#define S32K3XX_ADC0_DSDR                 (S32K3XX_ADC0_BASE + S32K3XX_ADC_DSDR_OFFSET)
#define S32K3XX_ADC0_PDEDR                (S32K3XX_ADC0_BASE + S32K3XX_ADC_PDEDR_OFFSET)

#define S32K3XX_ADC0_PCDR(n)              (S32K3XX_ADC0_BASE + S32K3XX_ADC_PCDR_OFFSET(n))
#define S32K3XX_ADC0_PCDR0                (S32K3XX_ADC0_BASE + S32K3XX_ADC_PCDR0_OFFSET)
#define S32K3XX_ADC0_PCDR1                (S32K3XX_ADC0_BASE + S32K3XX_ADC_PCDR1_OFFSET)
#define S32K3XX_ADC0_PCDR2                (S32K3XX_ADC0_BASE + S32K3XX_ADC_PCDR2_OFFSET)
#define S32K3XX_ADC0_PCDR3                (S32K3XX_ADC0_BASE + S32K3XX_ADC_PCDR3_OFFSET)
#define S32K3XX_ADC0_PCDR4                (S32K3XX_ADC0_BASE + S32K3XX_ADC_PCDR4_OFFSET)
#define S32K3XX_ADC0_PCDR5                (S32K3XX_ADC0_BASE + S32K3XX_ADC_PCDR5_OFFSET)
#define S32K3XX_ADC0_PCDR6                (S32K3XX_ADC0_BASE + S32K3XX_ADC_PCDR6_OFFSET)
#define S32K3XX_ADC0_PCDR7                (S32K3XX_ADC0_BASE + S32K3XX_ADC_PCDR7_OFFSET)

#define S32K3XX_ADC0_ICDR(n)              (S32K3XX_ADC0_BASE + S32K3XX_ADC_ICDR_OFFSET(n))
#define S32K3XX_ADC0_ICDR0                (S32K3XX_ADC0_BASE + S32K3XX_ADC_ICDR0_OFFSET)
#define S32K3XX_ADC0_ICDR1                (S32K3XX_ADC0_BASE + S32K3XX_ADC_ICDR1_OFFSET)
#define S32K3XX_ADC0_ICDR2                (S32K3XX_ADC0_BASE + S32K3XX_ADC_ICDR2_OFFSET)
#define S32K3XX_ADC0_ICDR3                (S32K3XX_ADC0_BASE + S32K3XX_ADC_ICDR3_OFFSET)
#define S32K3XX_ADC0_ICDR4                (S32K3XX_ADC0_BASE + S32K3XX_ADC_ICDR4_OFFSET)
#define S32K3XX_ADC0_ICDR5                (S32K3XX_ADC0_BASE + S32K3XX_ADC_ICDR5_OFFSET)
#define S32K3XX_ADC0_ICDR6                (S32K3XX_ADC0_BASE + S32K3XX_ADC_ICDR6_OFFSET)
#define S32K3XX_ADC0_ICDR7                (S32K3XX_ADC0_BASE + S32K3XX_ADC_ICDR7_OFFSET)
#define S32K3XX_ADC0_ICDR8                (S32K3XX_ADC0_BASE + S32K3XX_ADC_ICDR8_OFFSET)
#define S32K3XX_ADC0_ICDR9                (S32K3XX_ADC0_BASE + S32K3XX_ADC_ICDR9_OFFSET)
#define S32K3XX_ADC0_ICDR10               (S32K3XX_ADC0_BASE + S32K3XX_ADC_ICDR10_OFFSET)
#define S32K3XX_ADC0_ICDR11               (S32K3XX_ADC0_BASE + S32K3XX_ADC_ICDR11_OFFSET)
#define S32K3XX_ADC0_ICDR12               (S32K3XX_ADC0_BASE + S32K3XX_ADC_ICDR12_OFFSET)
#define S32K3XX_ADC0_ICDR13               (S32K3XX_ADC0_BASE + S32K3XX_ADC_ICDR13_OFFSET)
#define S32K3XX_ADC0_ICDR14               (S32K3XX_ADC0_BASE + S32K3XX_ADC_ICDR14_OFFSET)
#define S32K3XX_ADC0_ICDR15               (S32K3XX_ADC0_BASE + S32K3XX_ADC_ICDR15_OFFSET)
#define S32K3XX_ADC0_ICDR16               (S32K3XX_ADC0_BASE + S32K3XX_ADC_ICDR16_OFFSET)
#define S32K3XX_ADC0_ICDR17               (S32K3XX_ADC0_BASE + S32K3XX_ADC_ICDR17_OFFSET)
#define S32K3XX_ADC0_ICDR18               (S32K3XX_ADC0_BASE + S32K3XX_ADC_ICDR18_OFFSET)
#define S32K3XX_ADC0_ICDR19               (S32K3XX_ADC0_BASE + S32K3XX_ADC_ICDR19_OFFSET)
#define S32K3XX_ADC0_ICDR20               (S32K3XX_ADC0_BASE + S32K3XX_ADC_ICDR20_OFFSET)
#define S32K3XX_ADC0_ICDR21               (S32K3XX_ADC0_BASE + S32K3XX_ADC_ICDR21_OFFSET)
#define S32K3XX_ADC0_ICDR22               (S32K3XX_ADC0_BASE + S32K3XX_ADC_ICDR22_OFFSET)
#define S32K3XX_ADC0_ICDR23               (S32K3XX_ADC0_BASE + S32K3XX_ADC_ICDR23_OFFSET)

#define S32K3XX_ADC0_ECDR(n)              (S32K3XX_ADC0_BASE + S32K3XX_ADC_ECDR_OFFSET(n))
#define S32K3XX_ADC0_ECDR0                (S32K3XX_ADC0_BASE + S32K3XX_ADC_ECDR0_OFFSET)
#define S32K3XX_ADC0_ECDR1                (S32K3XX_ADC0_BASE + S32K3XX_ADC_ECDR1_OFFSET)
#define S32K3XX_ADC0_ECDR2                (S32K3XX_ADC0_BASE + S32K3XX_ADC_ECDR2_OFFSET)
#define S32K3XX_ADC0_ECDR3                (S32K3XX_ADC0_BASE + S32K3XX_ADC_ECDR3_OFFSET)
#define S32K3XX_ADC0_ECDR4                (S32K3XX_ADC0_BASE + S32K3XX_ADC_ECDR4_OFFSET)
#define S32K3XX_ADC0_ECDR5                (S32K3XX_ADC0_BASE + S32K3XX_ADC_ECDR5_OFFSET)
#define S32K3XX_ADC0_ECDR6                (S32K3XX_ADC0_BASE + S32K3XX_ADC_ECDR6_OFFSET)
#define S32K3XX_ADC0_ECDR7                (S32K3XX_ADC0_BASE + S32K3XX_ADC_ECDR7_OFFSET)
#define S32K3XX_ADC0_ECDR8                (S32K3XX_ADC0_BASE + S32K3XX_ADC_ECDR8_OFFSET)
#define S32K3XX_ADC0_ECDR9                (S32K3XX_ADC0_BASE + S32K3XX_ADC_ECDR9_OFFSET)
#define S32K3XX_ADC0_ECDR10               (S32K3XX_ADC0_BASE + S32K3XX_ADC_ECDR10_OFFSET)
#define S32K3XX_ADC0_ECDR11               (S32K3XX_ADC0_BASE + S32K3XX_ADC_ECDR11_OFFSET)
#define S32K3XX_ADC0_ECDR12               (S32K3XX_ADC0_BASE + S32K3XX_ADC_ECDR12_OFFSET)
#define S32K3XX_ADC0_ECDR13               (S32K3XX_ADC0_BASE + S32K3XX_ADC_ECDR13_OFFSET)
#define S32K3XX_ADC0_ECDR14               (S32K3XX_ADC0_BASE + S32K3XX_ADC_ECDR14_OFFSET)
#define S32K3XX_ADC0_ECDR15               (S32K3XX_ADC0_BASE + S32K3XX_ADC_ECDR15_OFFSET)
#define S32K3XX_ADC0_ECDR16               (S32K3XX_ADC0_BASE + S32K3XX_ADC_ECDR16_OFFSET)
#define S32K3XX_ADC0_ECDR17               (S32K3XX_ADC0_BASE + S32K3XX_ADC_ECDR17_OFFSET)
#define S32K3XX_ADC0_ECDR18               (S32K3XX_ADC0_BASE + S32K3XX_ADC_ECDR18_OFFSET)
#define S32K3XX_ADC0_ECDR19               (S32K3XX_ADC0_BASE + S32K3XX_ADC_ECDR19_OFFSET)
#define S32K3XX_ADC0_ECDR20               (S32K3XX_ADC0_BASE + S32K3XX_ADC_ECDR20_OFFSET)
#define S32K3XX_ADC0_ECDR21               (S32K3XX_ADC0_BASE + S32K3XX_ADC_ECDR21_OFFSET)
#define S32K3XX_ADC0_ECDR22               (S32K3XX_ADC0_BASE + S32K3XX_ADC_ECDR22_OFFSET)
#define S32K3XX_ADC0_ECDR23               (S32K3XX_ADC0_BASE + S32K3XX_ADC_ECDR23_OFFSET)
#define S32K3XX_ADC0_ECDR24               (S32K3XX_ADC0_BASE + S32K3XX_ADC_ECDR24_OFFSET)
#define S32K3XX_ADC0_ECDR25               (S32K3XX_ADC0_BASE + S32K3XX_ADC_ECDR25_OFFSET)
#define S32K3XX_ADC0_ECDR26               (S32K3XX_ADC0_BASE + S32K3XX_ADC_ECDR26_OFFSET)
#define S32K3XX_ADC0_ECDR27               (S32K3XX_ADC0_BASE + S32K3XX_ADC_ECDR27_OFFSET)
#define S32K3XX_ADC0_ECDR28               (S32K3XX_ADC0_BASE + S32K3XX_ADC_ECDR28_OFFSET)
#define S32K3XX_ADC0_ECDR29               (S32K3XX_ADC0_BASE + S32K3XX_ADC_ECDR29_OFFSET)
#define S32K3XX_ADC0_ECDR30               (S32K3XX_ADC0_BASE + S32K3XX_ADC_ECDR30_OFFSET)
#define S32K3XX_ADC0_ECDR31               (S32K3XX_ADC0_BASE + S32K3XX_ADC_ECDR31_OFFSET)

#define S32K3XX_ADC0_CWSELRPI0            (S32K3XX_ADC0_BASE + S32K3XX_ADC_CWSELRPI0_OFFSET)
#define S32K3XX_ADC0_CWSELRPI1            (S32K3XX_ADC0_BASE + S32K3XX_ADC_CWSELRPI1_OFFSET)
#define S32K3XX_ADC0_CWSELRSI0            (S32K3XX_ADC0_BASE + S32K3XX_ADC_CWSELRSI0_OFFSET)
#define S32K3XX_ADC0_CWSELRSI1            (S32K3XX_ADC0_BASE + S32K3XX_ADC_CWSELRSI1_OFFSET)
#define S32K3XX_ADC0_CWSELRSI2            (S32K3XX_ADC0_BASE + S32K3XX_ADC_CWSELRSI2_OFFSET)
#define S32K3XX_ADC0_CWSELREI0            (S32K3XX_ADC0_BASE + S32K3XX_ADC_CWSELREI0_OFFSET)
#define S32K3XX_ADC0_CWSELREI1            (S32K3XX_ADC0_BASE + S32K3XX_ADC_CWSELREI1_OFFSET)
#define S32K3XX_ADC0_CWSELREI2            (S32K3XX_ADC0_BASE + S32K3XX_ADC_CWSELREI2_OFFSET)
#define S32K3XX_ADC0_CWSELREI3            (S32K3XX_ADC0_BASE + S32K3XX_ADC_CWSELREI3_OFFSET)
#define S32K3XX_ADC0_CWENR0               (S32K3XX_ADC0_BASE + S32K3XX_ADC_CWENR0_OFFSET)
#define S32K3XX_ADC0_CWENR1               (S32K3XX_ADC0_BASE + S32K3XX_ADC_CWENR1_OFFSET)
#define S32K3XX_ADC0_CWENR2               (S32K3XX_ADC0_BASE + S32K3XX_ADC_CWENR2_OFFSET)
#define S32K3XX_ADC0_AWORR0               (S32K3XX_ADC0_BASE + S32K3XX_ADC_AWORR0_OFFSET)
#define S32K3XX_ADC0_AWORR1               (S32K3XX_ADC0_BASE + S32K3XX_ADC_AWORR1_OFFSET)
#define S32K3XX_ADC0_AWORR2               (S32K3XX_ADC0_BASE + S32K3XX_ADC_AWORR2_OFFSET)
#define S32K3XX_ADC0_STCR1                (S32K3XX_ADC0_BASE + S32K3XX_ADC_STCR1_OFFSET)
#define S32K3XX_ADC0_STCR2                (S32K3XX_ADC0_BASE + S32K3XX_ADC_STCR2_OFFSET)
#define S32K3XX_ADC0_STCR3                (S32K3XX_ADC0_BASE + S32K3XX_ADC_STCR3_OFFSET)
#define S32K3XX_ADC0_STBRR                (S32K3XX_ADC0_BASE + S32K3XX_ADC_STBRR_OFFSET)
#define S32K3XX_ADC0_STSR1                (S32K3XX_ADC0_BASE + S32K3XX_ADC_STSR1_OFFSET)
#define S32K3XX_ADC0_STSR2                (S32K3XX_ADC0_BASE + S32K3XX_ADC_STSR2_OFFSET)
#define S32K3XX_ADC0_STSR3                (S32K3XX_ADC0_BASE + S32K3XX_ADC_STSR3_OFFSET)
#define S32K3XX_ADC0_STSR4                (S32K3XX_ADC0_BASE + S32K3XX_ADC_STSR4_OFFSET)
#define S32K3XX_ADC0_STDR1                (S32K3XX_ADC0_BASE + S32K3XX_ADC_STDR1_OFFSET)
#define S32K3XX_ADC0_STAW0R               (S32K3XX_ADC0_BASE + S32K3XX_ADC_STAW0R_OFFSET)
#define S32K3XX_ADC0_STAW1R               (S32K3XX_ADC0_BASE + S32K3XX_ADC_STAW1R_OFFSET)
#define S32K3XX_ADC0_STAW2R               (S32K3XX_ADC0_BASE + S32K3XX_ADC_STAW2R_OFFSET)
#define S32K3XX_ADC0_STAW4R               (S32K3XX_ADC0_BASE + S32K3XX_ADC_STAW4R_OFFSET)
#define S32K3XX_ADC0_STAW5R               (S32K3XX_ADC0_BASE + S32K3XX_ADC_STAW5R_OFFSET)
#define S32K3XX_ADC0_AMSIO                (S32K3XX_ADC0_BASE + S32K3XX_ADC_AMSIO_OFFSET )
#define S32K3XX_ADC0_CALBISTREG           (S32K3XX_ADC0_BASE + S32K3XX_ADC_CALBISTREG_OFFSET)
#define S32K3XX_ADC0_OFSGNUSR             (S32K3XX_ADC0_BASE + S32K3XX_ADC_OFSGNUSR_OFFSET)
#define S32K3XX_ADC0_CAL2                 (S32K3XX_ADC0_BASE + S32K3XX_ADC_CAL2_OFFSET)

/* ADC1 */

#define S32K3XX_ADC1_MCR                  (S32K3XX_ADC1_BASE + S32K3XX_ADC_MCR_OFFSET)
#define S32K3XX_ADC1_MSR                  (S32K3XX_ADC1_BASE + S32K3XX_ADC_MSR_OFFSET)
#define S32K3XX_ADC1_ISR                  (S32K3XX_ADC1_BASE + S32K3XX_ADC_ISR_OFFSET)
#define S32K3XX_ADC1_CEOCFR0              (S32K3XX_ADC1_BASE + S32K3XX_ADC_CEOCFR0_OFFSET)
#define S32K3XX_ADC1_CEOCFR1              (S32K3XX_ADC1_BASE + S32K3XX_ADC_CEOCFR1_OFFSET)
#define S32K3XX_ADC1_CEOCFR2              (S32K3XX_ADC1_BASE + S32K3XX_ADC_CEOCFR2_OFFSET)
#define S32K3XX_ADC1_IMR                  (S32K3XX_ADC1_BASE + S32K3XX_ADC_IMR_OFFSET)
#define S32K3XX_ADC1_CIMR0                (S32K3XX_ADC1_BASE + S32K3XX_ADC_CIMR0_OFFSET)
#define S32K3XX_ADC1_CIMR1                (S32K3XX_ADC1_BASE + S32K3XX_ADC_CIMR1_OFFSET)
#define S32K3XX_ADC1_CIMR2                (S32K3XX_ADC1_BASE + S32K3XX_ADC_CIMR2_OFFSET)
#define S32K3XX_ADC1_WTISR                (S32K3XX_ADC1_BASE + S32K3XX_ADC_WTISR_OFFSET)
#define S32K3XX_ADC1_WTIMR                (S32K3XX_ADC1_BASE + S32K3XX_ADC_WTIMR_OFFSET)
#define S32K3XX_ADC1_DMAE                 (S32K3XX_ADC1_BASE + S32K3XX_ADC_DMAE_OFFSET)
#define S32K3XX_ADC1_DMAR0                (S32K3XX_ADC1_BASE + S32K3XX_ADC_DMAR0_OFFSET)
#define S32K3XX_ADC1_DMAR1                (S32K3XX_ADC1_BASE + S32K3XX_ADC_DMAR1_OFFSET)
#define S32K3XX_ADC1_DMAR2                (S32K3XX_ADC1_BASE + S32K3XX_ADC_DMAR2_OFFSET)
#define S32K3XX_ADC1_THRHLR0              (S32K3XX_ADC1_BASE + S32K3XX_ADC_THRHLR0_OFFSET)
#define S32K3XX_ADC1_THRHLR1              (S32K3XX_ADC1_BASE + S32K3XX_ADC_THRHLR1_OFFSET)
#define S32K3XX_ADC1_THRHLR2              (S32K3XX_ADC1_BASE + S32K3XX_ADC_THRHLR2_OFFSET)
#define S32K3XX_ADC1_THRHLR3              (S32K3XX_ADC1_BASE + S32K3XX_ADC_THRHLR3_OFFSET)
#define S32K3XX_ADC1_PSCR                 (S32K3XX_ADC1_BASE + S32K3XX_ADC_PSCR_OFFSET)
#define S32K3XX_ADC1_PSR0                 (S32K3XX_ADC1_BASE + S32K3XX_ADC_PSR0_OFFSET)
#define S32K3XX_ADC1_PSR1                 (S32K3XX_ADC1_BASE + S32K3XX_ADC_PSR1_OFFSET)
#define S32K3XX_ADC1_PSR2                 (S32K3XX_ADC1_BASE + S32K3XX_ADC_PSR2_OFFSET)
#define S32K3XX_ADC1_CTR0                 (S32K3XX_ADC1_BASE + S32K3XX_ADC_CTR0_OFFSET)
#define S32K3XX_ADC1_CTR1                 (S32K3XX_ADC1_BASE + S32K3XX_ADC_CTR1_OFFSET)
#define S32K3XX_ADC1_CTR2                 (S32K3XX_ADC1_BASE + S32K3XX_ADC_CTR2_OFFSET)
#define S32K3XX_ADC1_NCMR0                (S32K3XX_ADC1_BASE + S32K3XX_ADC_NCMR0_OFFSET)
#define S32K3XX_ADC1_NCMR1                (S32K3XX_ADC1_BASE + S32K3XX_ADC_NCMR1_OFFSET)
#define S32K3XX_ADC1_NCMR2                (S32K3XX_ADC1_BASE + S32K3XX_ADC_NCMR2_OFFSET)
#define S32K3XX_ADC1_JCMR0                (S32K3XX_ADC1_BASE + S32K3XX_ADC_JCMR0_OFFSET)
#define S32K3XX_ADC1_JCMR1                (S32K3XX_ADC1_BASE + S32K3XX_ADC_JCMR1_OFFSET)
#define S32K3XX_ADC1_JCMR2                (S32K3XX_ADC1_BASE + S32K3XX_ADC_JCMR2_OFFSET)
#define S32K3XX_ADC1_DSDR                 (S32K3XX_ADC1_BASE + S32K3XX_ADC_DSDR_OFFSET)
#define S32K3XX_ADC1_PDEDR                (S32K3XX_ADC1_BASE + S32K3XX_ADC_PDEDR_OFFSET)

#define S32K3XX_ADC1_PCDR(n)              (S32K3XX_ADC1_BASE + S32K3XX_ADC_PCDR_OFFSET(n))
#define S32K3XX_ADC1_PCDR0                (S32K3XX_ADC1_BASE + S32K3XX_ADC_PCDR0_OFFSET)
#define S32K3XX_ADC1_PCDR1                (S32K3XX_ADC1_BASE + S32K3XX_ADC_PCDR1_OFFSET)
#define S32K3XX_ADC1_PCDR2                (S32K3XX_ADC1_BASE + S32K3XX_ADC_PCDR2_OFFSET)
#define S32K3XX_ADC1_PCDR3                (S32K3XX_ADC1_BASE + S32K3XX_ADC_PCDR3_OFFSET)
#define S32K3XX_ADC1_PCDR4                (S32K3XX_ADC1_BASE + S32K3XX_ADC_PCDR4_OFFSET)
#define S32K3XX_ADC1_PCDR5                (S32K3XX_ADC1_BASE + S32K3XX_ADC_PCDR5_OFFSET)
#define S32K3XX_ADC1_PCDR6                (S32K3XX_ADC1_BASE + S32K3XX_ADC_PCDR6_OFFSET)
#define S32K3XX_ADC1_PCDR7                (S32K3XX_ADC1_BASE + S32K3XX_ADC_PCDR7_OFFSET)

#define S32K3XX_ADC1_ICDR(n)              (S32K3XX_ADC1_BASE + S32K3XX_ADC_ICDR_OFFSET(n))
#define S32K3XX_ADC1_ICDR0                (S32K3XX_ADC1_BASE + S32K3XX_ADC_ICDR0_OFFSET)
#define S32K3XX_ADC1_ICDR1                (S32K3XX_ADC1_BASE + S32K3XX_ADC_ICDR1_OFFSET)
#define S32K3XX_ADC1_ICDR2                (S32K3XX_ADC1_BASE + S32K3XX_ADC_ICDR2_OFFSET)
#define S32K3XX_ADC1_ICDR3                (S32K3XX_ADC1_BASE + S32K3XX_ADC_ICDR3_OFFSET)
#define S32K3XX_ADC1_ICDR4                (S32K3XX_ADC1_BASE + S32K3XX_ADC_ICDR4_OFFSET)
#define S32K3XX_ADC1_ICDR5                (S32K3XX_ADC1_BASE + S32K3XX_ADC_ICDR5_OFFSET)
#define S32K3XX_ADC1_ICDR6                (S32K3XX_ADC1_BASE + S32K3XX_ADC_ICDR6_OFFSET)
#define S32K3XX_ADC1_ICDR7                (S32K3XX_ADC1_BASE + S32K3XX_ADC_ICDR7_OFFSET)
#define S32K3XX_ADC1_ICDR8                (S32K3XX_ADC1_BASE + S32K3XX_ADC_ICDR8_OFFSET)
#define S32K3XX_ADC1_ICDR9                (S32K3XX_ADC1_BASE + S32K3XX_ADC_ICDR9_OFFSET)
#define S32K3XX_ADC1_ICDR10               (S32K3XX_ADC1_BASE + S32K3XX_ADC_ICDR10_OFFSET)
#define S32K3XX_ADC1_ICDR11               (S32K3XX_ADC1_BASE + S32K3XX_ADC_ICDR11_OFFSET)
#define S32K3XX_ADC1_ICDR12               (S32K3XX_ADC1_BASE + S32K3XX_ADC_ICDR12_OFFSET)
#define S32K3XX_ADC1_ICDR13               (S32K3XX_ADC1_BASE + S32K3XX_ADC_ICDR13_OFFSET)
#define S32K3XX_ADC1_ICDR14               (S32K3XX_ADC1_BASE + S32K3XX_ADC_ICDR14_OFFSET)
#define S32K3XX_ADC1_ICDR15               (S32K3XX_ADC1_BASE + S32K3XX_ADC_ICDR15_OFFSET)
#define S32K3XX_ADC1_ICDR16               (S32K3XX_ADC1_BASE + S32K3XX_ADC_ICDR16_OFFSET)
#define S32K3XX_ADC1_ICDR17               (S32K3XX_ADC1_BASE + S32K3XX_ADC_ICDR17_OFFSET)
#define S32K3XX_ADC1_ICDR18               (S32K3XX_ADC1_BASE + S32K3XX_ADC_ICDR18_OFFSET)
#define S32K3XX_ADC1_ICDR19               (S32K3XX_ADC1_BASE + S32K3XX_ADC_ICDR19_OFFSET)
#define S32K3XX_ADC1_ICDR20               (S32K3XX_ADC1_BASE + S32K3XX_ADC_ICDR20_OFFSET)
#define S32K3XX_ADC1_ICDR21               (S32K3XX_ADC1_BASE + S32K3XX_ADC_ICDR21_OFFSET)
#define S32K3XX_ADC1_ICDR22               (S32K3XX_ADC1_BASE + S32K3XX_ADC_ICDR22_OFFSET)
#define S32K3XX_ADC1_ICDR23               (S32K3XX_ADC1_BASE + S32K3XX_ADC_ICDR23_OFFSET)

#define S32K3XX_ADC1_ECDR(n)              (S32K3XX_ADC1_BASE + S32K3XX_ADC_ECDR_OFFSET(n))
#define S32K3XX_ADC1_ECDR0                (S32K3XX_ADC1_BASE + S32K3XX_ADC_ECDR0_OFFSET)
#define S32K3XX_ADC1_ECDR1                (S32K3XX_ADC1_BASE + S32K3XX_ADC_ECDR1_OFFSET)
#define S32K3XX_ADC1_ECDR2                (S32K3XX_ADC1_BASE + S32K3XX_ADC_ECDR2_OFFSET)
#define S32K3XX_ADC1_ECDR3                (S32K3XX_ADC1_BASE + S32K3XX_ADC_ECDR3_OFFSET)
#define S32K3XX_ADC1_ECDR4                (S32K3XX_ADC1_BASE + S32K3XX_ADC_ECDR4_OFFSET)
#define S32K3XX_ADC1_ECDR5                (S32K3XX_ADC1_BASE + S32K3XX_ADC_ECDR5_OFFSET)
#define S32K3XX_ADC1_ECDR6                (S32K3XX_ADC1_BASE + S32K3XX_ADC_ECDR6_OFFSET)
#define S32K3XX_ADC1_ECDR7                (S32K3XX_ADC1_BASE + S32K3XX_ADC_ECDR7_OFFSET)
#define S32K3XX_ADC1_ECDR8                (S32K3XX_ADC1_BASE + S32K3XX_ADC_ECDR8_OFFSET)
#define S32K3XX_ADC1_ECDR9                (S32K3XX_ADC1_BASE + S32K3XX_ADC_ECDR9_OFFSET)
#define S32K3XX_ADC1_ECDR10               (S32K3XX_ADC1_BASE + S32K3XX_ADC_ECDR10_OFFSET)
#define S32K3XX_ADC1_ECDR11               (S32K3XX_ADC1_BASE + S32K3XX_ADC_ECDR11_OFFSET)
#define S32K3XX_ADC1_ECDR12               (S32K3XX_ADC1_BASE + S32K3XX_ADC_ECDR12_OFFSET)
#define S32K3XX_ADC1_ECDR13               (S32K3XX_ADC1_BASE + S32K3XX_ADC_ECDR13_OFFSET)
#define S32K3XX_ADC1_ECDR14               (S32K3XX_ADC1_BASE + S32K3XX_ADC_ECDR14_OFFSET)
#define S32K3XX_ADC1_ECDR15               (S32K3XX_ADC1_BASE + S32K3XX_ADC_ECDR15_OFFSET)
#define S32K3XX_ADC1_ECDR16               (S32K3XX_ADC1_BASE + S32K3XX_ADC_ECDR16_OFFSET)
#define S32K3XX_ADC1_ECDR17               (S32K3XX_ADC1_BASE + S32K3XX_ADC_ECDR17_OFFSET)
#define S32K3XX_ADC1_ECDR18               (S32K3XX_ADC1_BASE + S32K3XX_ADC_ECDR18_OFFSET)
#define S32K3XX_ADC1_ECDR19               (S32K3XX_ADC1_BASE + S32K3XX_ADC_ECDR19_OFFSET)
#define S32K3XX_ADC1_ECDR20               (S32K3XX_ADC1_BASE + S32K3XX_ADC_ECDR20_OFFSET)
#define S32K3XX_ADC1_ECDR21               (S32K3XX_ADC1_BASE + S32K3XX_ADC_ECDR21_OFFSET)
#define S32K3XX_ADC1_ECDR22               (S32K3XX_ADC1_BASE + S32K3XX_ADC_ECDR22_OFFSET)
#define S32K3XX_ADC1_ECDR23               (S32K3XX_ADC1_BASE + S32K3XX_ADC_ECDR23_OFFSET)
#define S32K3XX_ADC1_ECDR24               (S32K3XX_ADC1_BASE + S32K3XX_ADC_ECDR24_OFFSET)
#define S32K3XX_ADC1_ECDR25               (S32K3XX_ADC1_BASE + S32K3XX_ADC_ECDR25_OFFSET)
#define S32K3XX_ADC1_ECDR26               (S32K3XX_ADC1_BASE + S32K3XX_ADC_ECDR26_OFFSET)
#define S32K3XX_ADC1_ECDR27               (S32K3XX_ADC1_BASE + S32K3XX_ADC_ECDR27_OFFSET)
#define S32K3XX_ADC1_ECDR28               (S32K3XX_ADC1_BASE + S32K3XX_ADC_ECDR28_OFFSET)
#define S32K3XX_ADC1_ECDR29               (S32K3XX_ADC1_BASE + S32K3XX_ADC_ECDR29_OFFSET)
#define S32K3XX_ADC1_ECDR30               (S32K3XX_ADC1_BASE + S32K3XX_ADC_ECDR30_OFFSET)
#define S32K3XX_ADC1_ECDR31               (S32K3XX_ADC1_BASE + S32K3XX_ADC_ECDR31_OFFSET)

#define S32K3XX_ADC1_CWSELRPI0            (S32K3XX_ADC1_BASE + S32K3XX_ADC_CWSELRPI0_OFFSET)
#define S32K3XX_ADC1_CWSELRPI1            (S32K3XX_ADC1_BASE + S32K3XX_ADC_CWSELRPI1_OFFSET)
#define S32K3XX_ADC1_CWSELRSI0            (S32K3XX_ADC1_BASE + S32K3XX_ADC_CWSELRSI0_OFFSET)
#define S32K3XX_ADC1_CWSELRSI1            (S32K3XX_ADC1_BASE + S32K3XX_ADC_CWSELRSI1_OFFSET)
#define S32K3XX_ADC1_CWSELRSI2            (S32K3XX_ADC1_BASE + S32K3XX_ADC_CWSELRSI2_OFFSET)
#define S32K3XX_ADC1_CWSELREI0            (S32K3XX_ADC1_BASE + S32K3XX_ADC_CWSELREI0_OFFSET)
#define S32K3XX_ADC1_CWSELREI1            (S32K3XX_ADC1_BASE + S32K3XX_ADC_CWSELREI1_OFFSET)
#define S32K3XX_ADC1_CWSELREI2            (S32K3XX_ADC1_BASE + S32K3XX_ADC_CWSELREI2_OFFSET)
#define S32K3XX_ADC1_CWSELREI3            (S32K3XX_ADC1_BASE + S32K3XX_ADC_CWSELREI3_OFFSET)
#define S32K3XX_ADC1_CWENR0               (S32K3XX_ADC1_BASE + S32K3XX_ADC_CWENR0_OFFSET)
#define S32K3XX_ADC1_CWENR1               (S32K3XX_ADC1_BASE + S32K3XX_ADC_CWENR1_OFFSET)
#define S32K3XX_ADC1_CWENR2               (S32K3XX_ADC1_BASE + S32K3XX_ADC_CWENR2_OFFSET)
#define S32K3XX_ADC1_AWORR0               (S32K3XX_ADC1_BASE + S32K3XX_ADC_AWORR0_OFFSET)
#define S32K3XX_ADC1_AWORR1               (S32K3XX_ADC1_BASE + S32K3XX_ADC_AWORR1_OFFSET)
#define S32K3XX_ADC1_AWORR2               (S32K3XX_ADC1_BASE + S32K3XX_ADC_AWORR2_OFFSET)
#define S32K3XX_ADC1_STCR1                (S32K3XX_ADC1_BASE + S32K3XX_ADC_STCR1_OFFSET)
#define S32K3XX_ADC1_STCR2                (S32K3XX_ADC1_BASE + S32K3XX_ADC_STCR2_OFFSET)
#define S32K3XX_ADC1_STCR3                (S32K3XX_ADC1_BASE + S32K3XX_ADC_STCR3_OFFSET)
#define S32K3XX_ADC1_STBRR                (S32K3XX_ADC1_BASE + S32K3XX_ADC_STBRR_OFFSET)
#define S32K3XX_ADC1_STSR1                (S32K3XX_ADC1_BASE + S32K3XX_ADC_STSR1_OFFSET)
#define S32K3XX_ADC1_STSR2                (S32K3XX_ADC1_BASE + S32K3XX_ADC_STSR2_OFFSET)
#define S32K3XX_ADC1_STSR3                (S32K3XX_ADC1_BASE + S32K3XX_ADC_STSR3_OFFSET)
#define S32K3XX_ADC1_STSR4                (S32K3XX_ADC1_BASE + S32K3XX_ADC_STSR4_OFFSET)
#define S32K3XX_ADC1_STDR1                (S32K3XX_ADC1_BASE + S32K3XX_ADC_STDR1_OFFSET)
#define S32K3XX_ADC1_STAW0R               (S32K3XX_ADC1_BASE + S32K3XX_ADC_STAW0R_OFFSET)
#define S32K3XX_ADC1_STAW1R               (S32K3XX_ADC1_BASE + S32K3XX_ADC_STAW1R_OFFSET)
#define S32K3XX_ADC1_STAW2R               (S32K3XX_ADC1_BASE + S32K3XX_ADC_STAW2R_OFFSET)
#define S32K3XX_ADC1_STAW4R               (S32K3XX_ADC1_BASE + S32K3XX_ADC_STAW4R_OFFSET)
#define S32K3XX_ADC1_STAW5R               (S32K3XX_ADC1_BASE + S32K3XX_ADC_STAW5R_OFFSET)
#define S32K3XX_ADC1_AMSIO                (S32K3XX_ADC1_BASE + S32K3XX_ADC_AMSIO_OFFSET )
#define S32K3XX_ADC1_CALBISTREG           (S32K3XX_ADC1_BASE + S32K3XX_ADC_CALBISTREG_OFFSET)
#define S32K3XX_ADC1_OFSGNUSR             (S32K3XX_ADC1_BASE + S32K3XX_ADC_OFSGNUSR_OFFSET)
#define S32K3XX_ADC1_CAL2                 (S32K3XX_ADC1_BASE + S32K3XX_ADC_CAL2_OFFSET)

/* ADC2 */

#define S32K3XX_ADC2_MCR                  (S32K3XX_ADC2_BASE + S32K3XX_ADC_MCR_OFFSET)
#define S32K3XX_ADC2_MSR                  (S32K3XX_ADC2_BASE + S32K3XX_ADC_MSR_OFFSET)
#define S32K3XX_ADC2_ISR                  (S32K3XX_ADC2_BASE + S32K3XX_ADC_ISR_OFFSET)
#define S32K3XX_ADC2_CEOCFR0              (S32K3XX_ADC2_BASE + S32K3XX_ADC_CEOCFR0_OFFSET)
#define S32K3XX_ADC2_CEOCFR1              (S32K3XX_ADC2_BASE + S32K3XX_ADC_CEOCFR1_OFFSET)
#define S32K3XX_ADC2_CEOCFR2              (S32K3XX_ADC2_BASE + S32K3XX_ADC_CEOCFR2_OFFSET)
#define S32K3XX_ADC2_IMR                  (S32K3XX_ADC2_BASE + S32K3XX_ADC_IMR_OFFSET)
#define S32K3XX_ADC2_CIMR0                (S32K3XX_ADC2_BASE + S32K3XX_ADC_CIMR0_OFFSET)
#define S32K3XX_ADC2_CIMR1                (S32K3XX_ADC2_BASE + S32K3XX_ADC_CIMR1_OFFSET)
#define S32K3XX_ADC2_CIMR2                (S32K3XX_ADC2_BASE + S32K3XX_ADC_CIMR2_OFFSET)
#define S32K3XX_ADC2_WTISR                (S32K3XX_ADC2_BASE + S32K3XX_ADC_WTISR_OFFSET)
#define S32K3XX_ADC2_WTIMR                (S32K3XX_ADC2_BASE + S32K3XX_ADC_WTIMR_OFFSET)
#define S32K3XX_ADC2_DMAE                 (S32K3XX_ADC2_BASE + S32K3XX_ADC_DMAE_OFFSET)
#define S32K3XX_ADC2_DMAR0                (S32K3XX_ADC2_BASE + S32K3XX_ADC_DMAR0_OFFSET)
#define S32K3XX_ADC2_DMAR1                (S32K3XX_ADC2_BASE + S32K3XX_ADC_DMAR1_OFFSET)
#define S32K3XX_ADC2_DMAR2                (S32K3XX_ADC2_BASE + S32K3XX_ADC_DMAR2_OFFSET)
#define S32K3XX_ADC2_THRHLR0              (S32K3XX_ADC2_BASE + S32K3XX_ADC_THRHLR0_OFFSET)
#define S32K3XX_ADC2_THRHLR1              (S32K3XX_ADC2_BASE + S32K3XX_ADC_THRHLR1_OFFSET)
#define S32K3XX_ADC2_THRHLR2              (S32K3XX_ADC2_BASE + S32K3XX_ADC_THRHLR2_OFFSET)
#define S32K3XX_ADC2_THRHLR3              (S32K3XX_ADC2_BASE + S32K3XX_ADC_THRHLR3_OFFSET)
#define S32K3XX_ADC2_PSCR                 (S32K3XX_ADC2_BASE + S32K3XX_ADC_PSCR_OFFSET)
#define S32K3XX_ADC2_PSR0                 (S32K3XX_ADC2_BASE + S32K3XX_ADC_PSR0_OFFSET)
#define S32K3XX_ADC2_PSR1                 (S32K3XX_ADC2_BASE + S32K3XX_ADC_PSR1_OFFSET)
#define S32K3XX_ADC2_PSR2                 (S32K3XX_ADC2_BASE + S32K3XX_ADC_PSR2_OFFSET)
#define S32K3XX_ADC2_CTR0                 (S32K3XX_ADC2_BASE + S32K3XX_ADC_CTR0_OFFSET)
#define S32K3XX_ADC2_CTR1                 (S32K3XX_ADC2_BASE + S32K3XX_ADC_CTR1_OFFSET)
#define S32K3XX_ADC2_CTR2                 (S32K3XX_ADC2_BASE + S32K3XX_ADC_CTR2_OFFSET)
#define S32K3XX_ADC2_NCMR0                (S32K3XX_ADC2_BASE + S32K3XX_ADC_NCMR0_OFFSET)
#define S32K3XX_ADC2_NCMR1                (S32K3XX_ADC2_BASE + S32K3XX_ADC_NCMR1_OFFSET)
#define S32K3XX_ADC2_NCMR2                (S32K3XX_ADC2_BASE + S32K3XX_ADC_NCMR2_OFFSET)
#define S32K3XX_ADC2_JCMR0                (S32K3XX_ADC2_BASE + S32K3XX_ADC_JCMR0_OFFSET)
#define S32K3XX_ADC2_JCMR1                (S32K3XX_ADC2_BASE + S32K3XX_ADC_JCMR1_OFFSET)
#define S32K3XX_ADC2_JCMR2                (S32K3XX_ADC2_BASE + S32K3XX_ADC_JCMR2_OFFSET)
#define S32K3XX_ADC2_DSDR                 (S32K3XX_ADC2_BASE + S32K3XX_ADC_DSDR_OFFSET)
#define S32K3XX_ADC2_PDEDR                (S32K3XX_ADC2_BASE + S32K3XX_ADC_PDEDR_OFFSET)

#define S32K3XX_ADC2_PCDR(n)              (S32K3XX_ADC2_BASE + S32K3XX_ADC_PCDR_OFFSET(n))
#define S32K3XX_ADC2_PCDR0                (S32K3XX_ADC2_BASE + S32K3XX_ADC_PCDR0_OFFSET)
#define S32K3XX_ADC2_PCDR1                (S32K3XX_ADC2_BASE + S32K3XX_ADC_PCDR1_OFFSET)
#define S32K3XX_ADC2_PCDR2                (S32K3XX_ADC2_BASE + S32K3XX_ADC_PCDR2_OFFSET)
#define S32K3XX_ADC2_PCDR3                (S32K3XX_ADC2_BASE + S32K3XX_ADC_PCDR3_OFFSET)
#define S32K3XX_ADC2_PCDR4                (S32K3XX_ADC2_BASE + S32K3XX_ADC_PCDR4_OFFSET)
#define S32K3XX_ADC2_PCDR5                (S32K3XX_ADC2_BASE + S32K3XX_ADC_PCDR5_OFFSET)
#define S32K3XX_ADC2_PCDR6                (S32K3XX_ADC2_BASE + S32K3XX_ADC_PCDR6_OFFSET)
#define S32K3XX_ADC2_PCDR7                (S32K3XX_ADC2_BASE + S32K3XX_ADC_PCDR7_OFFSET)

#define S32K3XX_ADC2_ICDR(n)              (S32K3XX_ADC2_BASE + S32K3XX_ADC_ICDR_OFFSET(n))
#define S32K3XX_ADC2_ICDR0                (S32K3XX_ADC2_BASE + S32K3XX_ADC_ICDR0_OFFSET)
#define S32K3XX_ADC2_ICDR1                (S32K3XX_ADC2_BASE + S32K3XX_ADC_ICDR1_OFFSET)
#define S32K3XX_ADC2_ICDR2                (S32K3XX_ADC2_BASE + S32K3XX_ADC_ICDR2_OFFSET)
#define S32K3XX_ADC2_ICDR3                (S32K3XX_ADC2_BASE + S32K3XX_ADC_ICDR3_OFFSET)
#define S32K3XX_ADC2_ICDR4                (S32K3XX_ADC2_BASE + S32K3XX_ADC_ICDR4_OFFSET)
#define S32K3XX_ADC2_ICDR5                (S32K3XX_ADC2_BASE + S32K3XX_ADC_ICDR5_OFFSET)
#define S32K3XX_ADC2_ICDR6                (S32K3XX_ADC2_BASE + S32K3XX_ADC_ICDR6_OFFSET)
#define S32K3XX_ADC2_ICDR7                (S32K3XX_ADC2_BASE + S32K3XX_ADC_ICDR7_OFFSET)
#define S32K3XX_ADC2_ICDR8                (S32K3XX_ADC2_BASE + S32K3XX_ADC_ICDR8_OFFSET)
#define S32K3XX_ADC2_ICDR9                (S32K3XX_ADC2_BASE + S32K3XX_ADC_ICDR9_OFFSET)
#define S32K3XX_ADC2_ICDR10               (S32K3XX_ADC2_BASE + S32K3XX_ADC_ICDR10_OFFSET)
#define S32K3XX_ADC2_ICDR11               (S32K3XX_ADC2_BASE + S32K3XX_ADC_ICDR11_OFFSET)
#define S32K3XX_ADC2_ICDR12               (S32K3XX_ADC2_BASE + S32K3XX_ADC_ICDR12_OFFSET)
#define S32K3XX_ADC2_ICDR13               (S32K3XX_ADC2_BASE + S32K3XX_ADC_ICDR13_OFFSET)
#define S32K3XX_ADC2_ICDR14               (S32K3XX_ADC2_BASE + S32K3XX_ADC_ICDR14_OFFSET)
#define S32K3XX_ADC2_ICDR15               (S32K3XX_ADC2_BASE + S32K3XX_ADC_ICDR15_OFFSET)
#define S32K3XX_ADC2_ICDR16               (S32K3XX_ADC2_BASE + S32K3XX_ADC_ICDR16_OFFSET)
#define S32K3XX_ADC2_ICDR17               (S32K3XX_ADC2_BASE + S32K3XX_ADC_ICDR17_OFFSET)
#define S32K3XX_ADC2_ICDR18               (S32K3XX_ADC2_BASE + S32K3XX_ADC_ICDR18_OFFSET)
#define S32K3XX_ADC2_ICDR19               (S32K3XX_ADC2_BASE + S32K3XX_ADC_ICDR19_OFFSET)
#define S32K3XX_ADC2_ICDR20               (S32K3XX_ADC2_BASE + S32K3XX_ADC_ICDR20_OFFSET)
#define S32K3XX_ADC2_ICDR21               (S32K3XX_ADC2_BASE + S32K3XX_ADC_ICDR21_OFFSET)
#define S32K3XX_ADC2_ICDR22               (S32K3XX_ADC2_BASE + S32K3XX_ADC_ICDR22_OFFSET)
#define S32K3XX_ADC2_ICDR23               (S32K3XX_ADC2_BASE + S32K3XX_ADC_ICDR23_OFFSET)

#define S32K3XX_ADC2_ECDR(n)              (S32K3XX_ADC2_BASE + S32K3XX_ADC_ECDR_OFFSET(n))
#define S32K3XX_ADC2_ECDR0                (S32K3XX_ADC2_BASE + S32K3XX_ADC_ECDR0_OFFSET)
#define S32K3XX_ADC2_ECDR1                (S32K3XX_ADC2_BASE + S32K3XX_ADC_ECDR1_OFFSET)
#define S32K3XX_ADC2_ECDR2                (S32K3XX_ADC2_BASE + S32K3XX_ADC_ECDR2_OFFSET)
#define S32K3XX_ADC2_ECDR3                (S32K3XX_ADC2_BASE + S32K3XX_ADC_ECDR3_OFFSET)
#define S32K3XX_ADC2_ECDR4                (S32K3XX_ADC2_BASE + S32K3XX_ADC_ECDR4_OFFSET)
#define S32K3XX_ADC2_ECDR5                (S32K3XX_ADC2_BASE + S32K3XX_ADC_ECDR5_OFFSET)
#define S32K3XX_ADC2_ECDR6                (S32K3XX_ADC2_BASE + S32K3XX_ADC_ECDR6_OFFSET)
#define S32K3XX_ADC2_ECDR7                (S32K3XX_ADC2_BASE + S32K3XX_ADC_ECDR7_OFFSET)
#define S32K3XX_ADC2_ECDR8                (S32K3XX_ADC2_BASE + S32K3XX_ADC_ECDR8_OFFSET)
#define S32K3XX_ADC2_ECDR9                (S32K3XX_ADC2_BASE + S32K3XX_ADC_ECDR9_OFFSET)
#define S32K3XX_ADC2_ECDR10               (S32K3XX_ADC2_BASE + S32K3XX_ADC_ECDR10_OFFSET)
#define S32K3XX_ADC2_ECDR11               (S32K3XX_ADC2_BASE + S32K3XX_ADC_ECDR11_OFFSET)
#define S32K3XX_ADC2_ECDR12               (S32K3XX_ADC2_BASE + S32K3XX_ADC_ECDR12_OFFSET)
#define S32K3XX_ADC2_ECDR13               (S32K3XX_ADC2_BASE + S32K3XX_ADC_ECDR13_OFFSET)
#define S32K3XX_ADC2_ECDR14               (S32K3XX_ADC2_BASE + S32K3XX_ADC_ECDR14_OFFSET)
#define S32K3XX_ADC2_ECDR15               (S32K3XX_ADC2_BASE + S32K3XX_ADC_ECDR15_OFFSET)
#define S32K3XX_ADC2_ECDR16               (S32K3XX_ADC2_BASE + S32K3XX_ADC_ECDR16_OFFSET)
#define S32K3XX_ADC2_ECDR17               (S32K3XX_ADC2_BASE + S32K3XX_ADC_ECDR17_OFFSET)
#define S32K3XX_ADC2_ECDR18               (S32K3XX_ADC2_BASE + S32K3XX_ADC_ECDR18_OFFSET)
#define S32K3XX_ADC2_ECDR19               (S32K3XX_ADC2_BASE + S32K3XX_ADC_ECDR19_OFFSET)
#define S32K3XX_ADC2_ECDR20               (S32K3XX_ADC2_BASE + S32K3XX_ADC_ECDR20_OFFSET)
#define S32K3XX_ADC2_ECDR21               (S32K3XX_ADC2_BASE + S32K3XX_ADC_ECDR21_OFFSET)
#define S32K3XX_ADC2_ECDR22               (S32K3XX_ADC2_BASE + S32K3XX_ADC_ECDR22_OFFSET)
#define S32K3XX_ADC2_ECDR23               (S32K3XX_ADC2_BASE + S32K3XX_ADC_ECDR23_OFFSET)
#define S32K3XX_ADC2_ECDR24               (S32K3XX_ADC2_BASE + S32K3XX_ADC_ECDR24_OFFSET)
#define S32K3XX_ADC2_ECDR25               (S32K3XX_ADC2_BASE + S32K3XX_ADC_ECDR25_OFFSET)
#define S32K3XX_ADC2_ECDR26               (S32K3XX_ADC2_BASE + S32K3XX_ADC_ECDR26_OFFSET)
#define S32K3XX_ADC2_ECDR27               (S32K3XX_ADC2_BASE + S32K3XX_ADC_ECDR27_OFFSET)
#define S32K3XX_ADC2_ECDR28               (S32K3XX_ADC2_BASE + S32K3XX_ADC_ECDR28_OFFSET)
#define S32K3XX_ADC2_ECDR29               (S32K3XX_ADC2_BASE + S32K3XX_ADC_ECDR29_OFFSET)
#define S32K3XX_ADC2_ECDR30               (S32K3XX_ADC2_BASE + S32K3XX_ADC_ECDR30_OFFSET)
#define S32K3XX_ADC2_ECDR31               (S32K3XX_ADC2_BASE + S32K3XX_ADC_ECDR31_OFFSET)

#define S32K3XX_ADC2_CWSELRPI0            (S32K3XX_ADC2_BASE + S32K3XX_ADC_CWSELRPI0_OFFSET)
#define S32K3XX_ADC2_CWSELRPI1            (S32K3XX_ADC2_BASE + S32K3XX_ADC_CWSELRPI1_OFFSET)
#define S32K3XX_ADC2_CWSELRSI0            (S32K3XX_ADC2_BASE + S32K3XX_ADC_CWSELRSI0_OFFSET)
#define S32K3XX_ADC2_CWSELRSI1            (S32K3XX_ADC2_BASE + S32K3XX_ADC_CWSELRSI1_OFFSET)
#define S32K3XX_ADC2_CWSELRSI2            (S32K3XX_ADC2_BASE + S32K3XX_ADC_CWSELRSI2_OFFSET)
#define S32K3XX_ADC2_CWSELREI0            (S32K3XX_ADC2_BASE + S32K3XX_ADC_CWSELREI0_OFFSET)
#define S32K3XX_ADC2_CWSELREI1            (S32K3XX_ADC2_BASE + S32K3XX_ADC_CWSELREI1_OFFSET)
#define S32K3XX_ADC2_CWSELREI2            (S32K3XX_ADC2_BASE + S32K3XX_ADC_CWSELREI2_OFFSET)
#define S32K3XX_ADC2_CWSELREI3            (S32K3XX_ADC2_BASE + S32K3XX_ADC_CWSELREI3_OFFSET)
#define S32K3XX_ADC2_CWENR0               (S32K3XX_ADC2_BASE + S32K3XX_ADC_CWENR0_OFFSET)
#define S32K3XX_ADC2_CWENR1               (S32K3XX_ADC2_BASE + S32K3XX_ADC_CWENR1_OFFSET)
#define S32K3XX_ADC2_CWENR2               (S32K3XX_ADC2_BASE + S32K3XX_ADC_CWENR2_OFFSET)
#define S32K3XX_ADC2_AWORR0               (S32K3XX_ADC2_BASE + S32K3XX_ADC_AWORR0_OFFSET)
#define S32K3XX_ADC2_AWORR1               (S32K3XX_ADC2_BASE + S32K3XX_ADC_AWORR1_OFFSET)
#define S32K3XX_ADC2_AWORR2               (S32K3XX_ADC2_BASE + S32K3XX_ADC_AWORR2_OFFSET)
#define S32K3XX_ADC2_STCR1                (S32K3XX_ADC2_BASE + S32K3XX_ADC_STCR1_OFFSET)
#define S32K3XX_ADC2_STCR2                (S32K3XX_ADC2_BASE + S32K3XX_ADC_STCR2_OFFSET)
#define S32K3XX_ADC2_STCR3                (S32K3XX_ADC2_BASE + S32K3XX_ADC_STCR3_OFFSET)
#define S32K3XX_ADC2_STBRR                (S32K3XX_ADC2_BASE + S32K3XX_ADC_STBRR_OFFSET)
#define S32K3XX_ADC2_STSR1                (S32K3XX_ADC2_BASE + S32K3XX_ADC_STSR1_OFFSET)
#define S32K3XX_ADC2_STSR2                (S32K3XX_ADC2_BASE + S32K3XX_ADC_STSR2_OFFSET)
#define S32K3XX_ADC2_STSR3                (S32K3XX_ADC2_BASE + S32K3XX_ADC_STSR3_OFFSET)
#define S32K3XX_ADC2_STSR4                (S32K3XX_ADC2_BASE + S32K3XX_ADC_STSR4_OFFSET)
#define S32K3XX_ADC2_STDR1                (S32K3XX_ADC2_BASE + S32K3XX_ADC_STDR1_OFFSET)
#define S32K3XX_ADC2_STAW0R               (S32K3XX_ADC2_BASE + S32K3XX_ADC_STAW0R_OFFSET)
#define S32K3XX_ADC2_STAW1R               (S32K3XX_ADC2_BASE + S32K3XX_ADC_STAW1R_OFFSET)
#define S32K3XX_ADC2_STAW2R               (S32K3XX_ADC2_BASE + S32K3XX_ADC_STAW2R_OFFSET)
#define S32K3XX_ADC2_STAW4R               (S32K3XX_ADC2_BASE + S32K3XX_ADC_STAW4R_OFFSET)
#define S32K3XX_ADC2_STAW5R               (S32K3XX_ADC2_BASE + S32K3XX_ADC_STAW5R_OFFSET)
#define S32K3XX_ADC2_AMSIO                (S32K3XX_ADC2_BASE + S32K3XX_ADC_AMSIO_OFFSET )
#define S32K3XX_ADC2_CALBISTREG           (S32K3XX_ADC2_BASE + S32K3XX_ADC_CALBISTREG_OFFSET)
#define S32K3XX_ADC2_OFSGNUSR             (S32K3XX_ADC2_BASE + S32K3XX_ADC_OFSGNUSR_OFFSET)
#define S32K3XX_ADC2_CAL2                 (S32K3XX_ADC2_BASE + S32K3XX_ADC_CAL2_OFFSET)

/* ADC Register Bitfield Definitions ****************************************/

/* Main Configuration Register (MCR) */

#define ADC_MCR_PWDN                      (1 << 0)  /* Bit 0: Power Down (PWDN) */
#define ADC_MCR_ADCLKSEL_SHIFT            (1)       /* Bits 1-2: Conversion Clock (ADC_clk) Frequency Selection (ADCLKSEL) */
#define ADC_MCR_ADCLKSEL_MASK             (0x03 << ADC_MCR_ADCLKSEL_SHIFT)
#  define ADC_MCR_ADCLKSEL_DIV1           (0x00 << ADC_MCR_ADCLKSEL_SHIFT) /* (Module Clock Frequency)/1 */
#  define ADC_MCR_ADCLKSEL_DIV2           (0x01 << ADC_MCR_ADCLKSEL_SHIFT) /* (Module Clock Frequency)/2 */
#  define ADC_MCR_ADCLKSEL_DIV4           (0x02 << ADC_MCR_ADCLKSEL_SHIFT) /* (Module Clock Frequency)/4 */
#  define ADC_MCR_ADCLKSEL_DIV8           (0x03 << ADC_MCR_ADCLKSEL_SHIFT) /* (Module Clock Frequency)/8 */

                                                    /* Bits 3-4: Reserved */
#define ADC_MCR_ACKO                      (1 << 5)  /* Bit 5: Auto Clock Off (ACKO) */
#define ADC_MCR_ABORT                     (1 << 6)  /* Bit 6: Abort Conversion (ABORT) */
#define ADC_MCR_ABORTCHAIN                (1 << 7)  /* Bit 7: Abort Chain (ABORTCHAIN) */
                                                    /* Bit 8: Reserved */
#define ADC_MCR_AVGS_SHIFT                (9)       /* Bits 9-10: Averaging Select (AVGS) */
#define ADC_MCR_AVGS_MASK                 (0x03 << ADC_MCR_AVGS_SHIFT)
#  define ADC_MCR_AVGS_4CONV              (0x00 << ADC_MCR_AVGS_SHIFT) /*  4 Conversions */
#  define ADC_MCR_AVGS_8CONV              (0x01 << ADC_MCR_AVGS_SHIFT) /*  8 Conversions */
#  define ADC_MCR_AVGS_16CONV             (0x02 << ADC_MCR_AVGS_SHIFT) /* 16 Conversions */
#  define ADC_MCR_AVGS_32CONV             (0x03 << ADC_MCR_AVGS_SHIFT) /* 32 Conversions */

#define ADC_MCR_AVGEN                     (1 << 11) /* Bit 11: Averaging Enable (AVGEN) */
                                                    /* Bits 12-14: Reserved */
#define ADC_MCR_STCL                      (1 << 15) /* Bit 15: Self-Test Configuration Lock (STCL) */
#define ADC_MCR_BCTU_MODE                 (1 << 16) /* Bit 16: Body Cross Trigger Unit Mode Select (BCTU_MODE) */
#define ADC_MCR_BCTUEN                    (1 << 17) /* Bit 17: Body Cross Trigger Unit Enable (BCTUEN) */
                                                    /* Bit 18-19: Reserved */
#define ADC_MCR_JSTART                    (1 << 20) /* Bit 20: Injected Start (JSTART) */
#define ADC_MCR_JEDGE                     (1 << 21) /* Bit 21: Injected Trigger Edge Selection (JEDGE) */
#define ADC_MCR_JTRGEN                    (1 << 22) /* Bit 22: Injection Trigger Enable (JTRGEN) */
                                                    /* Bit 23: Reserved */
#define ADC_MCR_NSTART                    (1 << 24) /* Bit 24: Start Normal Conversion (NSTART) */
#define ADC_MCR_XSTRTEN                   (1 << 25) /* Bit 25: Auxiliary External Start Enable (XSTRTEN) */
#define ADC_MCR_EDGE                      (1 << 26) /* Bit 26: External Trigger Edge Selection (EDGE) */
#define ADC_MCR_TRGEN                     (1 << 27) /* Bit 27: External Trigger Enable (TRGEN) */
                                                    /* Bit 28: Reserved */
#define ADC_MCR_MODE                      (1 << 29) /* Bit 29: Normal Conversion Mode (MODE) */
#define ADC_MCR_WLSIDE                    (1 << 30) /* Bit 30: Write Left-Aligned (WLSIDE) */
#define ADC_MCR_OWREN                     (1 << 31) /* Bit 31: Overwrite Enable (OWREN) */

/* Main Status Register (MSR) */

#define ADC_MSR_ADCSTATUS_SHIFT           (0)       /* Bits 0-2: ADC State (ADCSTATUS) */
#define ADC_MSR_ADCSTATUS_MASK            (0x07 << ADC_MSR_ADCSTATUS_SHIFT)
#  define ADC_MSR_ADCSTATUS_IDLE          (0x00 << ADC_MSR_ADCSTATUS_SHIFT) /* Idle */
#  define ADC_MSR_ADCSTATUS_PWRDOWN       (0x01 << ADC_MSR_ADCSTATUS_SHIFT) /* Power Down */
#  define ADC_MSR_ADCSTATUS_WAIT          (0x02 << ADC_MSR_ADCSTATUS_SHIFT) /* Wait */
#  define ADC_MSR_ADCSTATUS_CALIB         (0x03 << ADC_MSR_ADCSTATUS_SHIFT) /* Calibrate */
#  define ADC_MSR_ADCSTATUS_CONV          (0x04 << ADC_MSR_ADCSTATUS_SHIFT) /* Convert */
#  define ADC_MSR_ADCSTATUS_DONE          (0x06 << ADC_MSR_ADCSTATUS_SHIFT) /* Done */

                                                    /* Bits 3-4: Reserved */
#define ADC_MSR_ADCO                      (1 << 5)  /* Bit 5: Auto Clock-Off On (ACKO) */
                                                    /* Bits 6-8: Reserved */
#define ADC_MSR_CHADDR_SHIFT              (9)       /* Bits 9-15: Input Under Measure (CHADDR) */
#define ADC_MSR_CHADDR_MASK               (0x7f << ADC_MSR_CHADDR_SHIFT)
#define ADC_MSR_BCTUSTART                 (1 << 16) /* Bit 16: BCTU Conversion Started (BCTUSTART) */
                                                    /* Bit 17: Reserved */
#define ADC_MSR_SELF_TEST_S               (1 << 18) /* Bit 18: Indicates whether an ongoing conversion is for self-test (SELF_TEST_S) */
                                                    /* Bit 19: Reserved */
#define ADC_MSR_JSTART                    (1 << 20) /* Bit 20: Injected Conversion Started (JSTART) */
                                                    /* Bits 21-22: Reserved */
#define ADC_MSR_JABORT                    (1 << 23) /* Bit 23: Injected Conversion Aborted (JABORT) */
#define ADC_MSR_NSTART                    (1 << 24) /* Bit 24: Normal Conversion Started (NSTART) */
                                                    /* Bits 25-30: Reserved */
#define ADC_MSR_CALIBRTD                  (1 << 31) /* Bit 31: Calibration Status (CALIBRTD) */

/* Interrupt Status Register (ISR) */

#define ADC_ISR_ECH                       (1 << 0)  /* Bit 0: End of Chain Conversion (ECH) */
#define ADC_ISR_EOC                       (1 << 1)  /* Bit 1: End of Conversion (EOC) */
#define ADC_ISR_JECH                      (1 << 2)  /* Bit 2: End of Injected Chain Conversion (JECH) */
#define ADC_ISR_JEOC                      (1 << 3)  /* Bit 3: End of Injected Conversion (JEOC) */
#define ADC_ISR_EOBCTU                    (1 << 4)  /* Bit 4: End of BCTU Conversion (EOBCTU) */
                                                    /* Bits 5-31: Reserved */

/* Channel End Of Conversation Flag Register for Precision Inputs (CEOCFR0) */

#define ADC_CEOCFR0_PIEOCF(n)             (1 << (n)) /* Bits 0-7: Precision Input End of Conversion Flag n (PIEOCFn) */
                                                     /* Bits 8-31: Reserved */

/* Channel End Of Conversation Flag Register for Standard Inputs (CEOCFR1) */

#define ADC_CEOCFR1_SIEOCF(n)             (1 << (n)) /* Bits 0-23: Standard Input End of Conversion Flag n (SIEOCFn) */
                                                     /* Bits 24-31: Reserved */

/* Channel End Of Conversation Flag Register for External Inputs (CEOCFR2) */

#define ADC_CEOCFR2_EIEOCF(n)             (1 << (n)) /* Bits 0-31: External Input End of Conversion Flag n (EIEOCFn) */

/* Interrupt Mask Register (IMR) */

#define ADC_IMR_MSKECH                    (1 << 0)  /* Bit 0: ECH Interrupt Flag Enable (MSKECH) */
#define ADC_IMR_MSKEOC                    (1 << 1)  /* Bit 1: EOC Interrupt Flag Enable (MSKEOC) */
#define ADC_IMR_MSKJECH                   (1 << 2)  /* Bit 2: JECH Interrupt Flag Enable (MSKJECH) */
#define ADC_IMR_MSKJEOC                   (1 << 3)  /* Bit 3: JEOC Interrupt Flag Enable (MSKJEOC) */
#define ADC_IMR_MSKEOBCTU                 (1 << 4)  /* Bit 4: EOBCTU Interrupt Flag Enable (MSKEOBCTU) */
                                                    /* Bits 5-31: Reserved */

/* EOC Interrupt Enable Register for Precision Inputs (CIMR0) */

#define ADC_CIMR0_PIEOCIEN(n)             (1 << (n)) /* Bits 0-7: Precision Input EOC Interrupt Enable n (PIEOCIEN) */
                                                     /* Bits 8-31: Reserved */

/* EOC Interrupt Enable Register for Standard Inputs (CIMR1) */

#define ADC_CIMR1_SIEOCIEN(n)             (1 << (n)) /* Bits 0-23: Standard Input EOC Interrupt Enable n (SIEOCIEN) */
                                                     /* Bits 24-31: Reserved */

/* EOC Interrupt Enable Register for External Inputs (CIMR2) */

#define ADC_CIMR2_EIEOCIEN(n)             (1 << (n)) /* Bits 0-31: External Input EOC Interrupt Enable n (EIEOCIEN) */

/* Analog Watchdog Threshold Interrupt Status Register (WTISR) */

#define ADC_WTISR_LAWIF1                  (1 << 0)  /* Bit 0: Low Analog Watchdog Interrupt Flag Enable 1 (LAWIF1) */
#define ADC_WTISR_HAWIF1                  (1 << 1)  /* Bit 1: High Analog Watchdog Interrupt Flag Enable 1 (HAWIF1) */
#define ADC_WTISR_LAWIF2                  (1 << 2)  /* Bit 2: Low Analog Watchdog Interrupt Flag Enable 2 (LAWIF2) */
#define ADC_WTISR_HAWIF2                  (1 << 3)  /* Bit 3: High Analog Watchdog Interrupt Flag Enable 2 (HAWIF2) */
#define ADC_WTISR_LAWIF3                  (1 << 4)  /* Bit 4: Low Analog Watchdog Interrupt Flag Enable 3 (LAWIF3) */
#define ADC_WTISR_HAWIF3                  (1 << 5)  /* Bit 5: High Analog Watchdog Interrupt Flag Enable 3 (HAWIF3) */
#define ADC_WTISR_LAWIF4                  (1 << 6)  /* Bit 6: Low Analog Watchdog Interrupt Flag Enable 4 (LAWIF4) */
#define ADC_WTISR_HAWIF4                  (1 << 7)  /* Bit 7: High Analog Watchdog Interrupt Flag Enable 4 (HAWIF4) */
#define ADC_WTISR_LAWIF5                  (1 << 8)  /* Bit 8: Low Analog Watchdog Interrupt Flag Enable 5 (LAWIF5) */
#define ADC_WTISR_HAWIF5                  (1 << 9)  /* Bit 9: High Analog Watchdog Interrupt Flag Enable 5 (HAWIF5) */
#define ADC_WTISR_LAWIF6                  (1 << 10) /* Bit 10: Low Analog Watchdog Interrupt Flag Enable 6 (LAWIF6) */
#define ADC_WTISR_HAWIF6                  (1 << 11) /* Bit 11: High Analog Watchdog Interrupt Flag Enable 6 (HAWIF6) */
#define ADC_WTISR_LAWIF7                  (1 << 12) /* Bit 12: Low Analog Watchdog Interrupt Flag Enable 7 (LAWIF7) */
#define ADC_WTISR_HAWIF7                  (1 << 13) /* Bit 13: High Analog Watchdog Interrupt Flag Enable 7 (HAWIF7) */
#define ADC_WTISR_LAWIF8                  (1 << 14) /* Bit 14: Low Analog Watchdog Interrupt Flag Enable 8 (LAWIF8) */
#define ADC_WTISR_HAWIF8                  (1 << 15) /* Bit 15: High Analog Watchdog Interrupt Flag Enable 8 (HAWIF8) */
#define ADC_WTISR_LAWIF9                  (1 << 16) /* Bit 16: Low Analog Watchdog Interrupt Flag Enable 9 (LAWIF9) */
#define ADC_WTISR_HAWIF9                  (1 << 17) /* Bit 17: High Analog Watchdog Interrupt Flag Enable 9 (HAWIF9) */
#define ADC_WTISR_LAWIF10                 (1 << 18) /* Bit 18: Low Analog Watchdog Interrupt Flag Enable 10 (LAWIF10) */
#define ADC_WTISR_HAWIF10                 (1 << 19) /* Bit 19: High Analog Watchdog Interrupt Flag Enable 10 (HAWIF10) */
#define ADC_WTISR_LAWIF11                 (1 << 20) /* Bit 20: Low Analog Watchdog Interrupt Flag Enable 11 (LAWIF11) */
#define ADC_WTISR_HAWIF11                 (1 << 21) /* Bit 21: High Analog Watchdog Interrupt Flag Enable 11 (HAWIF11) */
#define ADC_WTISR_LAWIF12                 (1 << 22) /* Bit 22: Low Analog Watchdog Interrupt Flag Enable 12 (LAWIF12) */
#define ADC_WTISR_HAWIF12                 (1 << 23) /* Bit 23: High Analog Watchdog Interrupt Flag Enable 12 (HAWIF12) */
#define ADC_WTISR_LAWIF13                 (1 << 24) /* Bit 24: Low Analog Watchdog Interrupt Flag Enable 13 (LAWIF13) */
#define ADC_WTISR_HAWIF13                 (1 << 25) /* Bit 25: High Analog Watchdog Interrupt Flag Enable 13 (HAWIF13) */
#define ADC_WTISR_LAWIF14                 (1 << 26) /* Bit 26: Low Analog Watchdog Interrupt Flag Enable 14 (LAWIF14) */
#define ADC_WTISR_HAWIF14                 (1 << 27) /* Bit 27: High Analog Watchdog Interrupt Flag Enable 14 (HAWIF14) */
#define ADC_WTISR_LAWIF15                 (1 << 28) /* Bit 28: Low Analog Watchdog Interrupt Flag Enable 15 (LAWIF15) */
#define ADC_WTISR_HAWIF15                 (1 << 29) /* Bit 29: High Analog Watchdog Interrupt Flag Enable 15 (HAWIF15) */
#define ADC_WTISR_LAWIF16                 (1 << 30) /* Bit 30: Low Analog Watchdog Interrupt Flag Enable 16 (LAWIF16) */
#define ADC_WTISR_HAWIF16                 (1 << 31) /* Bit 31: High Analog Watchdog Interrupt Flag Enable 16 (HAWIF16) */

/* Analog Watchdog Threshold Interrupt Enable Register (WTIMR) */

#define ADC_WTIMR_LAWIFEN1                (1 << 0)  /* Bit 0: Low Analog Watchdog Interrupt Flag Enable 1 (LAWIFEN1) */
#define ADC_WTIMR_HDWIFEN1                (1 << 1)  /* Bit 1: High Data Watchdog Interrupt Flag Enable 1 (HDWIFEN1) */
#define ADC_WTIMR_LAWIFEN2                (1 << 2)  /* Bit 2: Low Analog Watchdog Interrupt Flag Enable 2 (LAWIFEN2) */
#define ADC_WTIMR_HDWIFEN2                (1 << 3)  /* Bit 3: High Data Watchdog Interrupt Flag Enable 2 (HDWIFEN2) */
#define ADC_WTIMR_LAWIFEN3                (1 << 4)  /* Bit 4: Low Analog Watchdog Interrupt Flag Enable 3 (LAWIFEN3) */
#define ADC_WTIMR_HDWIFEN3                (1 << 5)  /* Bit 5: High Data Watchdog Interrupt Flag Enable 3 (HDWIFEN3) */
#define ADC_WTIMR_LAWIFEN4                (1 << 6)  /* Bit 6: Low Analog Watchdog Interrupt Flag Enable 4 (LAWIFEN4) */
#define ADC_WTIMR_HDWIFEN4                (1 << 7)  /* Bit 7: High Data Watchdog Interrupt Flag Enable 4 (HDWIFEN4) */
                                                    /* Bits 8-31: Reserved */

/* Direct Memory Access Configuration Register (DMAE) */

#define ADC_DMAE_DMAEN                    (1 << 0)  /* Bit 0: DMA Enable (DMAEN) */
#define ADC_DMAE_DCLR                     (1 << 1)  /* Bit 1: DMA Clear Request (DCLR) */
                                                    /* Bits 2-31: Reserved */

/* DMA Request Enable Register for Precision Inputs (DMAR0) */

#define ADC_DMAR0_PIDMAREN(n)             (1 << (n)) /* Bits 0-7: Precision Input DMA Request Enable n (PIDMARENn) */
                                                     /* Bits 8-31: Reserved */

/* DMA Request Enable Register for Standard Inputs (DMAR1) */

#define ADC_DMAR1_SIDMAREN(n)             (1 << (n)) /* Bits 0-23: Standard Input DMA Request Enable n (SIDMARENn) */
                                                     /* Bits 24-31: Reserved */

/* DMA Request Enable Register for External Inputs (DMAR2) */

#define ADC_DMAR2_EIDMAREN(n)             (1 << (n)) /* Bits 0-31: External Input DMA Request Enable n (EIDMARENn) */

/* Analog Watchdog Threshold Values Register n (THRHLRn) */

#define ADC_THRHLR_THRL_SHIFT             (0)       /* Bits 0-14: Low Threshold Value (THRL) */
#define ADC_THRHLR_THRL_MASK              (0x7fff << ADC_THRHLR_THRL_SHIFT)
                                                    /* Bit 15: Reserved */
#define ADC_THRHLR_THRH_SHIFT             (16)      /* Bits 16-30: High Threshold Value (THRH) */
#define ADC_THRHLR_THRH_MASK              (0x7fff << ADC_THRHLR_THRH_SHIFT)
                                                    /* Bit 31: Reserved */

/* Presampling Control Register (PSCR) */

#define ADC_PSCR_PRECONV                  (1 << 0)  /* Bit 0: Convert Presampled Value (PRECONV) */
#define ADC_PSCR_PREVAL0                  (1 << 1)  /* Bit 1: Presampling Voltage Select for Precision Inputs (PREVAL0) */
                                                    /* Bit 2: Reserved */
#define ADC_PSCR_PREVAL1                  (1 << 3)  /* Bit 3: Presampling Voltage Select for Standard Inputs (PREVAL1) */
                                                    /* Bit 4: Reserved */
#define ADC_PSCR_PREVAL2                  (1 << 5)  /* Bit 5: Presampling Voltage Select for External Inputs (PREVAL2) */
                                                    /* Bits 6-31: Reserved */

/* Presampling Enable Register for Precision Inputs (PSR0) */

#define ADC_PSR0_PRES(n)                  (1 << (n)) /* Bits 0-7: Presampling Enable n (PRESn) */
                                                     /* Bits 8-31: Reserved */

/* Presampling Enable Register for Standard Inputs (PSR1) */

#define ADC_PSR1_PRES(n)                  (1 << (n)) /* Bits 0-23: Presampling Enable n (PRESn) */
                                                     /* Bits 24-31: Reserved */

/* Presampling Enable Register for External Inputs (PSR2) */

#define ADC_PSR2_PRES(n)                  (1 << (n)) /* Bits 0-31: Presampling Enable n (PRESn) */

/* Conversion Timing Register for Precision Inputs (CTR0) */

#define ADC_CTR0_INPSAMP_SHIFT            (0)        /* Bits 0-7: Input Sample Cycles (INPSAMP) */
#define ADC_CTR0_INPSAMP_MASK             (0xff << ADC_CTR0_INPSAMP_SHIFT)
                                                     /* Bits 8-31: Reserved */

/* Conversion Timing Register for Standard Inputs (CTR1) */

#define ADC_CTR1_INPSAMP_SHIFT            (0)        /* Bits 0-7: Input Sample Cycles (INPSAMP) */
#define ADC_CTR1_INPSAMP_MASK             (0xff << ADC_CTR1_INPSAMP_SHIFT)
                                                     /* Bits 8-31: Reserved */

/* Conversion Timing Register for External Inputs (CTR2) */

#define ADC_CTR2_INPSAMP_SHIFT            (0)        /* Bits 0-7: Input Sample Cycles (INPSAMP) */
#define ADC_CTR2_INPSAMP_MASK             (0xff << ADC_CTR2_INPSAMP_SHIFT)
                                                     /* Bits 8-31: Reserved */

/* Normal Conversion Enable Register for Precision Inputs (NCMR0) */

#define ADC_NMCR0_CH(n)                   (1 << (n)) /* Bits 0-7: Precision Input n to be Converted (CHn) */
                                                     /* Bits 8-31: Reserved */

/* Normal Conversion Enable Register for Standard Inputs (NCMR1) */

#define ADC_NMCR1_CH(n)                   (1 << (n)) /* Bits 0-23: Standard Input n to be Converted (CHn) */
                                                     /* Bits 24-31: Reserved */

/* Normal Conversion Enable Register for External Inputs (NCMR2) */

#define ADC_NMCR2_CH(n)                   (1 << (n)) /* Bits 0-31: External Input n to be Converted (CHn) */

/* Injected Conversion Enable Register for Precision Inputs (JCMR0) */

#define ADC_JMCR0_CH(n)                   (1 << (n)) /* Bits 0-7: Precision Input to be Converted (CHn) */
                                                     /* Bits 8-31: Reserved */

/* Injected Conversion Enable Register for Standard Inputs (JCMR1) */

#define ADC_JMCR1_CH(n)                   (1 << (n)) /* Bits 0-23: Standard Input to be Converted (CHn) */
                                                     /* Bits 24-31: Reserved */

/* Injected Conversion Enable Register for External Inputs (JCMR2) */

#define ADC_JMCR2_CH(n)                   (1 << (n)) /* Bits 0-31: External Input to be Converted (CHn) */

/* Delay Start of Data Conversion Register (DSDR) */

#define ADC_DSDR_DSD_SHIFT                (0)       /* Bits 0-15: Delay (DSD) */
#define ASD_DSDR_DSD_MASK                 (0xff << ADC_DSDR_DSD_SHIFT)
                                                    /* Bits 16-31: Reserved */

/* Power Down Exit Delay Register (PDEDR) */

#define ADC_PDEDR_PDED_SHIFT              (0)       /* Bits 0-7: Delay (PDED) */
#define ASD_PDEDR_PDED_MASK               (0x0f << ADC_PDEDR_PDED_SHIFT)
                                                    /* Bits 8-31: Reserved */

/* Precision Input n Conversion Data Register (PCDRn) */

#define ADC_PCDR_CDATA_SHIFT              (0)       /* Bits 0-15: Conversion Data (CDATA) */
#define ADC_PCDR_CDATA_MASK               (0xff << ADC_PCDR_CDATA_SHIFT)
#define ADC_PCDR_RESULT_SHIFT             (16)      /* Bits 16-17: Conversion Data Type (RESULT) */
#define ADC_PCDR_RESULT_MASK              (0x03 << ADC_PCDR_RESULT_SHIFT)
#  define ADC_PCDR_RESULT_NORM            (0x00 << ADC_PCDR_RESULT_MASK) /* Normal Trigger */
#  define ADC_PCDR_RESULT_INJ             (0x01 << ADC_PCDR_RESULT_MASK) /* Injected Trigger */
#  define ADC_PCDR_RESULT_BCTU            (0x02 << ADC_PCDR_RESULT_MASK) /* BCTU Trigger */

#define ADC_PCDR_OVERW                    (1 << 18) /* Bit 18: Overwrite Status Flag (OVERW) */
#define ADC_PCDR_VALID                    (1 << 19) /* Bit 19: Conversion Data Available (VALID) */
                                                    /* Bits 20-31: Reserved */

/* Standard Input n Conversion Data Register (ICDRn) */

#define ADC_ICDR_CDATA_SHIFT              (0)       /* Bits 0-15: Conversion Data (CDATA) */
#define ADC_ICDR_CDATA_MASK               (0xff << ADC_ICDR_CDATA_SHIFT)
#define ADC_ICDR_RESULT_SHIFT             (16)      /* Bits 16-17: Conversion Data Type (RESULT) */
#define ADC_ICDR_RESULT_MASK              (0x03 << ADC_ICDR_RESULT_SHIFT)
#  define ADC_ICDR_RESULT_NORM            (0x00 << ADC_ICDR_RESULT_MASK) /* Normal Trigger */
#  define ADC_ICDR_RESULT_INJ             (0x01 << ADC_ICDR_RESULT_MASK) /* Injected Trigger */
#  define ADC_ICDR_RESULT_BCTU            (0x02 << ADC_ICDR_RESULT_MASK) /* BCTU Trigger */

#define ADC_ICDR_OVERW                    (1 << 18) /* Bit 18: Overwrite Status Flag (OVERW) */
#define ADC_ICDR_VALID                    (1 << 19) /* Bit 19: Conversion Data Available (VALID) */
                                                    /* Bits 20-31: Reserved */

/* External Input n Conversion Data Register (ECDRn) */

#define ADC_ECDR_CDATA_SHIFT              (0)       /* Bits 0-15: Conversion Data (CDATA) */
#define ADC_ECDR_CDATA_MASK               (0xff << ADC_ECDR_CDATA_SHIFT)
#define ADC_ECDR_RESULT_SHIFT             (16)      /* Bits 16-17: Conversion Data Type (RESULT) */
#define ADC_ECDR_RESULT_MASK              (0x03 << ADC_ECDR_RESULT_SHIFT)
#  define ADC_ECDR_RESULT_NORM            (0x00 << ADC_ECDR_RESULT_SHIFT) /* Normal Trigger */
#  define ADC_ECDR_RESULT_INJ             (0x01 << ADC_ECDR_RESULT_SHIFT) /* Injected Trigger */
#  define ADC_ECDR_RESULT_BCTU            (0x02 << ADC_ECDR_RESULT_SHIFT) /* BCTU Trigger */

#define ADC_ECDR_OVERW                    (1 << 18) /* Bit 18: Overwrite Status Flag (OVERW) */
#define ADC_ECDR_VALID                    (1 << 19) /* Bit 19: Conversion Data Available (VALID) */
                                                    /* Bits 20-31: Reserved */

/* Channel Analog Watchdog Select Register for Precision Inputs 0
 * (CWSELRPI0)
 */

#define ADC_CWSELRPI0_WSEL_SI0_0_SHIFT    (0)       /* Bits 0-1: Analog Watchdog Selection (WSEL_SI0_0) */
#define ADC_CWSELRPI0_WSEL_SI0_0_MASK     (0x03 << ADC_CWSELRPI0_WSEL_SI0_0_SHIFT)
                                                    /* Bits 2-3: Reserved */
#define ADC_CWSELRPI0_WSEL_SI0_1_SHIFT    (4)       /* Bits 4-5: Analog Watchdog Selection (WSEL_SI0_1) */
#define ADC_CWSELRPI0_WSEL_SI0_1_MASK     (0x03 << ADC_CWSELRPI0_WSEL_SI0_1_SHIFT)
                                                    /* Bits 6-7: Reserved */
#define ADC_CWSELRPI0_WSEL_SI0_2_SHIFT    (8)       /* Bits 8-9: Analog Watchdog Selection (WSEL_SI0_2) */
#define ADC_CWSELRPI0_WSEL_SI0_2_MASK     (0x03 << ADC_CWSELRPI0_WSEL_SI0_2_SHIFT)
                                                    /* Bits 10-11: Reserved */
#define ADC_CWSELRPI0_WSEL_SI0_3_SHIFT    (12)      /* Bits 12-13: Analog Watchdog Selection (WSEL_SI0_3) */
#define ADC_CWSELRPI0_WSEL_SI0_3_MASK     (0x03 << ADC_CWSELRPI0_WSEL_SI0_3_SHIFT)
                                                    /* Bits 14-15: Reserved */
#define ADC_CWSELRPI0_WSEL_SI0_4_SHIFT    (16)      /* Bits 16-17: Analog Watchdog Selection (WSEL_SI0_4) */
#define ADC_CWSELRPI0_WSEL_SI0_4_MASK     (0x03 << ADC_CWSELRPI0_WSEL_SI0_4_SHIFT)
                                                    /* Bits 18-19: Reserved */
#define ADC_CWSELRPI0_WSEL_SI0_5_SHIFT    (20)      /* Bits 20-21: Analog Watchdog Selection (WSEL_SI0_5) */
#define ADC_CWSELRPI0_WSEL_SI0_5_MASK     (0x03 << ADC_CWSELRPI0_WSEL_SI0_5_SHIFT)
                                                    /* Bits 22-23: Reserved */
#define ADC_CWSELRPI0_WSEL_SI0_6_SHIFT    (24)      /* Bits 24-25: Analog Watchdog Selection (WSEL_SI0_6) */
#define ADC_CWSELRPI0_WSEL_SI0_6_MASK     (0x03 << ADC_CWSELRPI0_WSEL_SI0_6_SHIFT)
                                                    /* Bits 26-27: Reserved */
#define ADC_CWSELRPI0_WSEL_SI0_7_SHIFT    (28)      /* Bits 28-29: Analog Watchdog Selection (WSEL_SI0_7) */
#define ADC_CWSELRPI0_WSEL_SI0_7_MASK     (0x03 << ADC_CWSELRPI0_WSEL_SI0_7_SHIFT)
                                                    /* Bits 30-31: Reserved */

/* Channel Analog Watchdog Select Register for Precision Inputs 1
 * (CWSELRPI1)
 */

                                                    /* Bits 0-31: Reserved */

/* Channel Analog Watchdog Select Register for Standard Inputs 0
 * (CWSELRSI0)
 */

#define ADC_CWSELRSI0_WSEL_SI0_0_SHIFT    (0)       /* Bits 0-1: Analog Watchdog Selection (WSEL_SI0_0) */
#define ADC_CWSELRSI0_WSEL_SI0_0_MASK     (0x03 << ADC_CWSELRSI0_WSEL_SI0_0_SHIFT)
                                                    /* Bits 2-3: Reserved */
#define ADC_CWSELRSI0_WSEL_SI1_0_SHIFT    (4)       /* Bits 4-5: Analog Watchdog Selection (WSEL_SI1_0) */
#define ADC_CWSELRSI0_WSEL_SI1_0_MASK     (0x03 << ADC_CWSELRSI0_WSEL_SI1_0_SHIFT)
                                                    /* Bits 6-7: Reserved */
#define ADC_CWSELRSI0_WSEL_SI2_0_SHIFT    (8)       /* Bits 8-9: Analog Watchdog Selection (WSEL_SI2_0) */
#define ADC_CWSELRSI0_WSEL_SI2_0_MASK     (0x03 << ADC_CWSELRSI0_WSEL_SI2_0_SHIFT)
                                                    /* Bits 10-11: Reserved */
#define ADC_CWSELRSI0_WSEL_SI3_0_SHIFT    (12)      /* Bits 12-13: Analog Watchdog Selection (WSEL_SI3_0) */
#define ADC_CWSELRSI0_WSEL_SI3_0_MASK     (0x03 << ADC_CWSELRSI0_WSEL_SI3_0_SHIFT)
                                                    /* Bits 14-15: Reserved */
#define ADC_CWSELRSI0_WSEL_SI4_0_SHIFT    (16)      /* Bits 16-17: Analog Watchdog Selection (WSEL_SI4_0) */
#define ADC_CWSELRSI0_WSEL_SI4_0_MASK     (0x03 << ADC_CWSELRSI0_WSEL_SI4_0_SHIFT)
                                                    /* Bits 18-19: Reserved */
#define ADC_CWSELRSI0_WSEL_SI5_0_SHIFT    (20)      /* Bits 20-21: Analog Watchdog Selection (WSEL_SI5_0) */
#define ADC_CWSELRSI0_WSEL_SI5_0_MASK     (0x03 << ADC_CWSELRSI0_WSEL_SI5_0_SHIFT)
                                                    /* Bits 22-23: Reserved */
#define ADC_CWSELRSI0_WSEL_SI6_0_SHIFT    (24)      /* Bits 24-25: Analog Watchdog Selection (WSEL_SI6_0) */
#define ADC_CWSELRSI0_WSEL_SI6_0_MASK     (0x03 << ADC_CWSELRSI0_WSEL_SI6_0_SHIFT)
                                                    /* Bits 26-27: Reserved */
#define ADC_CWSELRSI0_WSEL_SI7_0_SHIFT    (28)      /* Bits 28-29: Analog Watchdog Selection (WSEL_SI7_0) */
#define ADC_CWSELRSI0_WSEL_SI7_0_MASK     (0x03 << ADC_CWSELRSI0_WSEL_SI7_0_SHIFT)
                                                    /* Bits 30-31: Reserved */

/* Channel Analog Watchdog Select Register for Standard Inputs 1
 * (CWSELRSI1)
 */

#define ADC_CWSELRSI1_WSEL_SI0_1_SHIFT    (0)       /* Bits 0-1: Analog Watchdog Selection (WSEL_SI0_1) */
#define ADC_CWSELRSI1_WSEL_SI0_1_MASK     (0x03 << ADC_CWSELRSI1_WSEL_SI0_1_SHIFT)
                                                    /* Bits 2-3: Reserved */
#define ADC_CWSELRSI1_WSEL_SI1_1_SHIFT    (4)       /* Bits 4-5: Analog Watchdog Selection (WSEL_SI1_1) */
#define ADC_CWSELRSI1_WSEL_SI1_1_MASK     (0x03 << ADC_CWSELRSI1_WSEL_SI1_1_SHIFT)
                                                    /* Bits 6-7: Reserved */
#define ADC_CWSELRSI1_WSEL_SI2_1_SHIFT    (8)       /* Bits 8-9: Analog Watchdog Selection (WSEL_SI2_1) */
#define ADC_CWSELRSI1_WSEL_SI2_1_MASK     (0x03 << ADC_CWSELRSI1_WSEL_SI2_1_SHIFT)
                                                    /* Bits 10-11: Reserved */
#define ADC_CWSELRSI1_WSEL_SI3_1_SHIFT    (12)      /* Bits 12-13: Analog Watchdog Selection (WSEL_SI3_1) */
#define ADC_CWSELRSI1_WSEL_SI3_1_MASK     (0x03 << ADC_CWSELRSI1_WSEL_SI3_1_SHIFT)
                                                    /* Bits 14-15: Reserved */
#define ADC_CWSELRSI1_WSEL_SI4_1_SHIFT    (16)      /* Bits 16-17: Analog Watchdog Selection (WSEL_SI4_1) */
#define ADC_CWSELRSI1_WSEL_SI4_1_MASK     (0x03 << ADC_CWSELRSI1_WSEL_SI4_1_SHIFT)
                                                    /* Bits 18-19: Reserved */
#define ADC_CWSELRSI1_WSEL_SI5_1_SHIFT    (20)      /* Bits 20-21: Analog Watchdog Selection (WSEL_SI5_1) */
#define ADC_CWSELRSI1_WSEL_SI5_1_MASK     (0x03 << ADC_CWSELRSI1_WSEL_SI5_1_SHIFT)
                                                    /* Bits 22-23: Reserved */
#define ADC_CWSELRSI1_WSEL_SI6_1_SHIFT    (24)      /* Bits 24-25: Analog Watchdog Selection (WSEL_SI6_1) */
#define ADC_CWSELRSI1_WSEL_SI6_1_MASK     (0x03 << ADC_CWSELRSI1_WSEL_SI6_1_SHIFT)
                                                    /* Bits 26-27: Reserved */
#define ADC_CWSELRSI1_WSEL_SI7_1_SHIFT    (28)      /* Bits 28-29: Analog Watchdog Selection (WSEL_SI7_1) */
#define ADC_CWSELRSI1_WSEL_SI7_1_MASK     (0x03 << ADC_CWSELRSI1_WSEL_SI7_1_SHIFT)

/* Channel Analog Watchdog Select Register for Standard Inputs 2
 * (CWSELRSI2)
 */

#define ADC_CWSELRSI2_WSEL_SI0_2_SHIFT    (0)       /* Bits 0-1: Analog Watchdog Selection (WSEL_SI0_2) */
#define ADC_CWSELRSI2_WSEL_SI0_2_MASK     (0x03 << ADC_CWSELRSI2_WSEL_SI0_2_SHIFT)
                                                    /* Bits 2-3: Reserved */
#define ADC_CWSELRSI2_WSEL_SI1_2_SHIFT    (4)       /* Bits 4-5: Analog Watchdog Selection (WSEL_SI1_2) */
#define ADC_CWSELRSI2_WSEL_SI1_2_MASK     (0x03 << ADC_CWSELRSI2_WSEL_SI1_2_SHIFT)
                                                    /* Bits 6-7: Reserved */
#define ADC_CWSELRSI2_WSEL_SI2_2_SHIFT    (8)       /* Bits 8-9: Analog Watchdog Selection (WSEL_SI2_2) */
#define ADC_CWSELRSI2_WSEL_SI2_2_MASK     (0x03 << ADC_CWSELRSI2_WSEL_SI2_2_SHIFT)
                                                    /* Bits 10-11: Reserved */
#define ADC_CWSELRSI2_WSEL_SI3_2_SHIFT    (12)      /* Bits 12-13: Analog Watchdog Selection (WSEL_SI3_2) */
#define ADC_CWSELRSI2_WSEL_SI3_2_MASK     (0x03 << ADC_CWSELRSI2_WSEL_SI3_2_SHIFT)
                                                    /* Bits 14-15: Reserved */
#define ADC_CWSELRSI2_WSEL_SI4_2_SHIFT    (16)      /* Bits 16-17: Analog Watchdog Selection (WSEL_SI4_2) */
#define ADC_CWSELRSI2_WSEL_SI4_2_MASK     (0x03 << ADC_CWSELRSI2_WSEL_SI4_2_SHIFT)
                                                    /* Bits 18-19: Reserved */
#define ADC_CWSELRSI2_WSEL_SI5_2_SHIFT    (20)      /* Bits 20-21: Analog Watchdog Selection (WSEL_SI5_2) */
#define ADC_CWSELRSI2_WSEL_SI5_2_MASK     (0x03 << ADC_CWSELRSI2_WSEL_SI5_2_SHIFT)
                                                    /* Bits 22-23: Reserved */
#define ADC_CWSELRSI2_WSEL_SI6_2_SHIFT    (24)      /* Bits 24-25: Analog Watchdog Selection (WSEL_SI6_2) */
#define ADC_CWSELRSI2_WSEL_SI6_2_MASK     (0x03 << ADC_CWSELRSI2_WSEL_SI6_2_SHIFT)
                                                    /* Bits 26-27: Reserved */
#define ADC_CWSELRSI2_WSEL_SI7_2_SHIFT    (28)      /* Bits 28-29: Analog Watchdog Selection (WSEL_SI7_2) */
#define ADC_CWSELRSI2_WSEL_SI7_2_MASK     (0x03 << ADC_CWSELRSI2_WSEL_SI7_2_SHIFT)

/* Channel Analog Watchdog Select Register for External Inputs 0
 * (CWSELREI0)
 */

#define ADC_CWSELREI0_WSEL_SI0_0_SHIFT    (0)       /* Bits 0-1: Analog Watchdog Selection (WSEL_SI0_0) */
#define ADC_CWSELREI0_WSEL_SI0_0_MASK     (0x03 << ADC_CWSELREI0_WSEL_SI0_0_SHIFT)
                                                    /* Bits 2-3: Reserved */
#define ADC_CWSELREI0_WSEL_SI0_1_SHIFT    (4)       /* Bits 4-5: Analog Watchdog Selection (WSEL_SI0_1) */
#define ADC_CWSELREI0_WSEL_SI0_1_MASK     (0x03 << ADC_CWSELREI0_WSEL_SI0_1_SHIFT)
                                                    /* Bits 6-7: Reserved */
#define ADC_CWSELREI0_WSEL_SI0_2_SHIFT    (8)       /* Bits 8-9: Analog Watchdog Selection (WSEL_SI0_2) */
#define ADC_CWSELREI0_WSEL_SI0_2_MASK     (0x03 << ADC_CWSELREI0_WSEL_SI0_2_SHIFT)
                                                    /* Bits 10-11: Reserved */
#define ADC_CWSELREI0_WSEL_SI0_3_SHIFT    (12)      /* Bits 12-13: Analog Watchdog Selection (WSEL_SI0_3) */
#define ADC_CWSELREI0_WSEL_SI0_3_MASK     (0x03 << ADC_CWSELREI0_WSEL_SI0_3_SHIFT)
                                                    /* Bits 14-15: Reserved */
#define ADC_CWSELREI0_WSEL_SI0_4_SHIFT    (16)      /* Bits 16-17: Analog Watchdog Selection (WSEL_SI0_4) */
#define ADC_CWSELREI0_WSEL_SI0_4_MASK     (0x03 << ADC_CWSELREI0_WSEL_SI0_4_SHIFT)
                                                    /* Bits 18-19: Reserved */
#define ADC_CWSELREI0_WSEL_SI0_5_SHIFT    (20)      /* Bits 20-21: Analog Watchdog Selection (WSEL_SI0_5) */
#define ADC_CWSELREI0_WSEL_SI0_5_MASK     (0x03 << ADC_CWSELREI0_WSEL_SI0_5_SHIFT)
                                                    /* Bits 22-23: Reserved */
#define ADC_CWSELREI0_WSEL_SI0_6_SHIFT    (24)      /* Bits 24-25: Analog Watchdog Selection (WSEL_SI0_6) */
#define ADC_CWSELREI0_WSEL_SI0_6_MASK     (0x03 << ADC_CWSELREI0_WSEL_SI0_6_SHIFT)
                                                    /* Bits 26-27: Reserved */
#define ADC_CWSELREI0_WSEL_SI0_7_SHIFT    (28)      /* Bits 28-29: Analog Watchdog Selection (WSEL_SI0_7) */
#define ADC_CWSELREI0_WSEL_SI0_7_MASK     (0x03 << ADC_CWSELREI0_WSEL_SI0_7_SHIFT)
                                                    /* Bits 30-31: Reserved */

/* Channel Analog Watchdog Select Register for External Inputs 1
 * (CWSELREI1)
 */

#define ADC_CWSELREI1_WSEL_SI1_8_SHIFT    (0)       /* Bits 0-1: Analog Watchdog Selection (WSEL_SI1_8) */
#define ADC_CWSELREI1_WSEL_SI1_8_MASK     (0x03 << ADC_CWSELREI1_WSEL_SI1_8_SHIFT)
                                                    /* Bits 2-3: Reserved */
#define ADC_CWSELREI1_WSEL_SI1_9_SHIFT    (4)       /* Bits 4-5: Analog Watchdog Selection (WSEL_SI1_9) */
#define ADC_CWSELREI1_WSEL_SI1_9_MASK     (0x03 << ADC_CWSELREI1_WSEL_SI1_9_SHIFT)
                                                    /* Bits 6-7: Reserved */
#define ADC_CWSELREI1_WSEL_SI1_10_SHIFT   (8)       /* Bits 8-9: Analog Watchdog Selection (WSEL_SI1_10) */
#define ADC_CWSELREI1_WSEL_SI1_10_MASK    (0x03 << ADC_CWSELREI1_WSEL_SI1_10_SHIFT)
                                                    /* Bits 10-11: Reserved */
#define ADC_CWSELREI1_WSEL_SI1_11_SHIFT   (12)      /* Bits 12-13: Analog Watchdog Selection (WSEL_SI1_11) */
#define ADC_CWSELREI1_WSEL_SI1_11_MASK    (0x03 << ADC_CWSELREI1_WSEL_SI1_11_SHIFT)
                                                    /* Bits 14-15: Reserved */
#define ADC_CWSELREI1_WSEL_SI1_12_SHIFT   (16)      /* Bits 16-17: Analog Watchdog Selection (WSEL_SI1_12) */
#define ADC_CWSELREI1_WSEL_SI1_12_MASK    (0x03 << ADC_CWSELREI1_WSEL_SI1_12_SHIFT)
                                                    /* Bits 18-19: Reserved */
#define ADC_CWSELREI1_WSEL_SI1_13_SHIFT   (20)      /* Bits 20-21: Analog Watchdog Selection (WSEL_SI1_13) */
#define ADC_CWSELREI1_WSEL_SI1_13_MASK    (0x03 << ADC_CWSELREI1_WSEL_SI1_13_SHIFT)
                                                    /* Bits 22-23: Reserved */
#define ADC_CWSELREI1_WSEL_SI1_14_SHIFT   (24)      /* Bits 24-25: Analog Watchdog Selection (WSEL_SI1_14) */
#define ADC_CWSELREI1_WSEL_SI1_14_MASK    (0x03 << ADC_CWSELREI1_WSEL_SI1_14_SHIFT)
                                                    /* Bits 26-27: Reserved */
#define ADC_CWSELREI1_WSEL_SI1_15_SHIFT   (28)      /* Bits 28-29: Analog Watchdog Selection (WSEL_SI1_15) */
#define ADC_CWSELREI1_WSEL_SI1_15_MASK    (0x03 << ADC_CWSELREI1_WSEL_SI1_15_SHIFT)
                                                    /* Bits 30-31: Reserved */

/* Channel Analog Watchdog Select Register for External Inputs 2
 * (CWSELREI2)
 */

#define ADC_CWSELREI2_WSEL_SI2_16_SHIFT   (0)       /* Bits 0-1: Analog Watchdog Selection (WSEL_SI2_16) */
#define ADC_CWSELREI2_WSEL_SI2_16_MASK    (0x03 << ADC_CWSELREI2_WSEL_SI2_16_SHIFT)
                                                    /* Bits 2-3: Reserved */
#define ADC_CWSELREI2_WSEL_SI2_17_SHIFT   (4)       /* Bits 4-5: Analog Watchdog Selection (WSEL_SI2_17) */
#define ADC_CWSELREI2_WSEL_SI2_17_MASK    (0x03 << ADC_CWSELREI2_WSEL_SI2_17_SHIFT)
                                                    /* Bits 6-7: Reserved */
#define ADC_CWSELREI2_WSEL_SI2_18_SHIFT   (8)       /* Bits 8-9: Analog Watchdog Selection (WSEL_SI2_18) */
#define ADC_CWSELREI2_WSEL_SI2_18_MASK    (0x03 << ADC_CWSELREI2_WSEL_SI2_18_SHIFT)
                                                    /* Bits 10-11: Reserved */
#define ADC_CWSELREI2_WSEL_SI2_19_SHIFT   (12)      /* Bits 12-13: Analog Watchdog Selection (WSEL_SI2_19) */
#define ADC_CWSELREI2_WSEL_SI2_19_MASK    (0x03 << ADC_CWSELREI2_WSEL_SI2_19_SHIFT)
                                                    /* Bits 14-15: Reserved */
#define ADC_CWSELREI2_WSEL_SI2_20_SHIFT   (16)      /* Bits 16-17: Analog Watchdog Selection (WSEL_SI2_20) */
#define ADC_CWSELREI2_WSEL_SI2_20_MASK    (0x03 << ADC_CWSELREI2_WSEL_SI2_20_SHIFT)
                                                    /* Bits 18-19: Reserved */
#define ADC_CWSELREI2_WSEL_SI2_21_SHIFT   (20)      /* Bits 20-21: Analog Watchdog Selection (WSEL_SI2_21) */
#define ADC_CWSELREI2_WSEL_SI2_21_MASK    (0x03 << ADC_CWSELREI2_WSEL_SI2_21_SHIFT)
                                                    /* Bits 22-23: Reserved */
#define ADC_CWSELREI2_WSEL_SI2_22_SHIFT   (24)      /* Bits 24-25: Analog Watchdog Selection (WSEL_SI2_22) */
#define ADC_CWSELREI2_WSEL_SI2_22_MASK    (0x03 << ADC_CWSELREI2_WSEL_SI2_22_SHIFT)
                                                    /* Bits 26-27: Reserved */
#define ADC_CWSELREI2_WSEL_SI2_23_SHIFT   (28)      /* Bits 28-29: Analog Watchdog Selection (WSEL_SI2_23) */
#define ADC_CWSELREI2_WSEL_SI2_23_MASK    (0x03 << ADC_CWSELREI2_WSEL_SI2_23_SHIFT)
                                                    /* Bits 30-31: Reserved */

/* Channel Analog Watchdog Select Register for External Inputs 3
 * (CWSELREI3)
 */

#define ADC_CWSELREI3_WSEL_SI3_24_SHIFT   (0)       /* Bits 0-1: Analog Watchdog Selection (WSEL_SI3_24) */
#define ADC_CWSELREI3_WSEL_SI3_24_MASK    (0x03 << ADC_CWSELREI3_WSEL_SI3_24_SHIFT)
                                                    /* Bits 2-3: Reserved */
#define ADC_CWSELREI3_WSEL_SI3_25_SHIFT   (4)       /* Bits 4-5: Analog Watchdog Selection (WSEL_SI3_25) */
#define ADC_CWSELREI3_WSEL_SI3_25_MASK    (0x03 << ADC_CWSELREI3_WSEL_SI3_25_SHIFT)
                                                    /* Bits 6-7: Reserved */
#define ADC_CWSELREI3_WSEL_SI3_26_SHIFT   (8)       /* Bits 8-9: Analog Watchdog Selection (WSEL_SI3_26) */
#define ADC_CWSELREI3_WSEL_SI3_26_MASK    (0x03 << ADC_CWSELREI3_WSEL_SI3_26_SHIFT)
                                                    /* Bits 10-11: Reserved */
#define ADC_CWSELREI3_WSEL_SI3_27_SHIFT   (12)      /* Bits 12-13: Analog Watchdog Selection (WSEL_SI3_27) */
#define ADC_CWSELREI3_WSEL_SI3_27_MASK    (0x03 << ADC_CWSELREI3_WSEL_SI3_27_SHIFT)
                                                    /* Bits 14-15: Reserved */
#define ADC_CWSELREI3_WSEL_SI3_28_SHIFT   (16)      /* Bits 16-17: Analog Watchdog Selection (WSEL_SI3_28) */
#define ADC_CWSELREI3_WSEL_SI3_28_MASK    (0x03 << ADC_CWSELREI3_WSEL_SI3_28_SHIFT)
                                                    /* Bits 18-19: Reserved */
#define ADC_CWSELREI3_WSEL_SI3_29_SHIFT   (20)      /* Bits 20-21: Analog Watchdog Selection (WSEL_SI3_29) */
#define ADC_CWSELREI3_WSEL_SI3_29_MASK    (0x03 << ADC_CWSELREI3_WSEL_SI3_29_SHIFT)
                                                    /* Bits 22-23: Reserved */
#define ADC_CWSELREI3_WSEL_SI3_30_SHIFT   (24)      /* Bits 24-25: Analog Watchdog Selection (WSEL_SI3_30) */
#define ADC_CWSELREI3_WSEL_SI3_30_MASK    (0x03 << ADC_CWSELREI3_WSEL_SI3_30_SHIFT)
                                                    /* Bits 26-27: Reserved */
#define ADC_CWSELREI3_WSEL_SI3_31_SHIFT   (28)      /* Bits 28-29: Analog Watchdog Selection (WSEL_SI3_31) */
#define ADC_CWSELREI3_WSEL_SI3_31_MASK    (0x03 << ADC_CWSELREI3_WSEL_SI3_31_SHIFT)
                                                    /* Bits 30-31: Reserved */

/* Channel Watchdog Enable Register for Precision Inputs (CWENR0) */

#define ADC_CWENR0_CWEN(n)                (1 << (n)) /* Bits 0-7: Channel Analog Watchdog Enable for Precision Inputs (CWENn) */
                                                     /* Bits 8-31: Reserved */

/* Channel Watchdog Enable Register for Standard Inputs (CWENR1) */

#define ADC_CWENR1_CWEN(n)                (1 << (n)) /* Bits 0-23: Channel Analog Watchdog Enable for Standard Inputs (CWENn) */
                                                     /* Bits 24-31: Reserved */

/* Channel Watchdog Enable Register for External Inputs (CWENR2) */

#define ADC_CWENR2_CWEN(n)                (1 << (n)) /* Bits 0-31: Channel Analog Watchdog Enable for External Inputs (CWENn) */

/* Analog Watchdog Out of Range Register for Precision Inputs (AWORR0) */

#define ADC_AWORR0_CWEN(n)                (1 << (n)) /* Bits 0-7: Analog Watchdog Out of Range for Precision Inputs (AWOR_CHn) */
                                                     /* Bits 8-31: Reserved */

/* Analog Watchdog Out of Range Register for Standard Inputs (AWORR1) */

#define ADC_AWORR1_CWEN(n)                (1 << (n)) /* Bits 0-23: Analog Watchdog Out of Range for Standard Inputs (AWOR_CHn) */
                                                     /* Bits 24-31: Reserved */

/* Analog Watchdog Out of Range Register for External Inputs (AWORR2) */

#define ADC_AWORR2_CWEN(n)                (1 << (n)) /* Bits 0-31: Analog Watchdog Out of Range for External Inputs (AWOR_CHn) */

/* Self-Test Configuration Register 1 (STCR1) */

                                                    /* Bits 0-7: Reserved */
#define ADC_STCR1_INPSAMP_S_SHIFT         (8)       /* Bits 8-15: Input Sampling Time Algorithm S (INPSAMP_S) */
#define ADC_STCR1_INPSAMP_S_MASK          (0xff << ADC_STCR1_INPSAMP_S_SHIFT)
                                                    /* Bits 16-23: Reserved */
#define ADC_STCR1_INPSAMP_C_SHIFT         (24)      /* Bits 24-31: Input Sampling Time Algorithm C (INPSAMP_C) */
#define ADC_STCR1_INPSAMP_C_MASK          (0xff << ADC_STCR1_INPSAMP_C_SHIFT)

/* Self-Test Configuration Register 2 (STCR2) */

#define ADC_STCR2_FMA_S                   (1 << 0)  /* Bit 0: Fault Mapping Algorithm S (FMA_S) */
                                                    /* Bit 1: Reserved */
#define ADC_STCR2_FMA_C                   (1 << 2)  /* Bit 2: Fault Mapping Algorithm C (FMA_C) */
#define ADC_STCR2_FMA_WDTERR              (1 << 3)  /* Bit 3: Fault Mapping Self-Test Watchdog Timer Error (FMA_WDTERR) */
#define ADC_STCR2_FMA_WDSERR              (1 << 4)  /* Bit 4: Fault Mapping Self-Test Watchdog Sequence Error (FMA_WDSERR) */
                                                    /* Bits 5-6: Reserved */
#define ADC_STCR2_EN                      (1 << 7)  /* Bit 7: Self-Test Enable (EN) */
                                                    /* Bits 8-10: Reserved */
#define ADC_STCR2_MSKERR_S0               (1 << 11) /* Bit 11: Mask Error Interrupt Algorithm S0 (MSKERR_S0) */
#define ADC_STCR2_MSKERR_S1               (1 << 12) /* Bit 12: Mask Error Interrupt Algorithm S1 (MSKERR_S1) */
#define ADC_STCR2_MSKERR_S2               (1 << 13) /* Bit 13: Mask Error Interrupt Algorithm S2 (MSKERR_S2) */
                                                    /* Bit 14: Reserved */
#define ADC_STCR2_MSKERR_C                (1 << 15) /* Bit 15: Mask Error Interrupt Algorithm C (MSKERR_C) */
#define ADC_STCR2_MSKWDG_EOA_S            (1 << 16) /* Bit 16: Mask Error Interrupt End Of Algorithm S (MSKWDG_EOA_S) */
                                                    /* Bit 17: Reserved */
#define ADC_STCR2_MSKWDG_EOA_C            (1 << 18) /* Bit 18: Mask Error Interrupt End Of Algorithm C (MSKWDG_EOA_C) */
                                                    /* Bits 19-22: Reserved */
#define ADC_STCR2_MSKST_EOC               (1 << 23) /* Bit 23: Mask Interrupt Self-Test End Of Conversion (MSKST_EOC) */
                                                    /* Bit 24: Reserved */
#define ADC_STCR2_MSKWDTERR               (1 << 25) /* Bit 25: Mask Interrupt Self-Test Watchdog Timer Error (MSKWDTERR) */
#define ADC_STCR2_SERR                    (1 << 26) /* Bit 26: Self-Test Error Injection (SERR) */
#define ADC_STCR2_MSKWDSERR               (1 << 27) /* Bit 27: Mask Interrupt Self-Test Watchdog Sequence Error (MSKWDSERR) */
                                                    /* Bits 28-31: Reserved */

/* Self-Test Configuration Register 3 (STCR3) */

#define ADC_STCR3_MSTEP_SHIFT             (0)       /* Bits 0-4: Algorithm Step (MSTEP) */
#define ADC_STCR3_MSTEP_MASK              (0x1f << ADC_STCR3_MSTEP_SHIFT)
                                                    /* Bits 5-7: Reserved */
#define ADC_STCR3_ALG_SHIFT               (8)       /* Bits 8-9: Algorithm Selection (ALG) */
#define ADC_STCR3_ALG_MASK                (0x03 << ADC_STCR3_ALG_SHIFT)
#  define ADC_STCR3_ALG_OSOM_ALGS_SS      (0x00 << ADC_STCR3_ALG_SHIFT) /* One-Shot Operation Mode: Algorithm S (Single Step = MSTEP) */
#  define ADC_STCR3_ALG_OSOM_ALGC_SS      (0x02 << ADC_STCR3_ALG_SHIFT) /* One-Shot Operation Mode: Algorithm C (Single Step = MSTEP) */
#  define ADC_STCR3_ALG_OSOM_ALGS         (0x03 << ADC_STCR3_ALG_SHIFT) /* One-Shot Operation Mode: Algorithm S (Default) */
#  define ADC_STCR3_ALG_CCM_ALGS          (0x00 << ADC_STCR3_ALG_SHIFT) /* Continuous Conversion Mode: Algorithm S */
#  define ADC_STCR3_ALG_CCM_ALGC          (0x02 << ADC_STCR3_ALG_SHIFT) /* Continuous Conversion Mode: Algorithm C */
#  define ADC_STCR3_ALG_CCM_ALGSS         (0x03 << ADC_STCR3_ALG_SHIFT) /* Continuous Conversion Mode: Algorithm S + Algorithm C (Default) */

                                                    /* Bits 10-31: Reserved */

/* Self-Test Baud Rate Register (STBRR) */

#define ADC_STBRR_BR_SHIFT                (0)       /* Bits 0-7: Baud Rate (BR) */
#define ADC_STBRR_BR_MASK                 (0xff << ADC_STBRR_BR_SHIFT)
                                                    /* Bits 8-15: Reserved */
#define ADC_STBRR_WDT_SHIFT               (16)      /* Bits 16-18: Self-Test Watchdog Timer (WDT) */
#define ADC_STBRR_WDT_MASK                (0x07 << ADC_STBRR_WDT_SHIFT)
#  define ADC_STBRR_WDT_8KCYC             (0x00 << ADC_STBRR_WDT_SHIFT) /*     8,192 Conversion Clock Cycles */
#  define ADC_STBRR_WDT_39KCYC            (0x01 << ADC_STBRR_WDT_SHIFT) /*    39,936 Conversion Clock Cycles */
#  define ADC_STBRR_WDT_78KCYC            (0x02 << ADC_STBRR_WDT_SHIFT) /*    79,872 Conversion Clock Cycles */
#  define ADC_STBRR_WDT_156KCYC           (0x03 << ADC_STBRR_WDT_SHIFT) /*   159,744 Conversion Clock Cycles */
#  define ADC_STBRR_WDT_391KCYC           (0x04 << ADC_STBRR_WDT_SHIFT) /*   400,384 Conversion Clock Cycles */
#  define ADC_STBRR_WDT_781KCYC           (0x05 << ADC_STBRR_WDT_SHIFT) /*   799,744 Conversion Clock Cycles */
#  define ADC_STBRR_WDT_1562KCYC          (0x06 << ADC_STBRR_WDT_SHIFT) /* 1,599,488 Conversion Clock Cycles */
#  define ADC_STBRR_WDT_3906KCYC          (0x07 << ADC_STBRR_WDT_SHIFT) /* 3,999,744 Conversion Clock Cycles */

                                                    /* Bits 19-31: Reserved */

/* Self-Test Status Register 1 (STSR1) */

                                                    /* Bits 0-4: Reserved */
#define ADC_STSR1_STEP_C_SHIFT            (5)       /* Bits 5-9: Step Of Algorithm C (STEP_C) */
#define ADC_STSR1_STEP_C_MASK             (0x1f << ADC_STSR1_STEP_C_SHIFT)
                                                    /* Bit 10: Reserved */
#define ADC_STSR1_ERR_S0                  (1 << 11) /* Bit 11: Error Algorithm S Step 0 (ERR_S0) */
#define ADC_STSR1_ERR_S1                  (1 << 12) /* Bit 12: Error Algorithm S Step 1 (ERR_S1) */
#define ADC_STSR1_ERR_S2                  (1 << 13) /* Bit 13: Error Algorithm S Step 2 (ERR_S2) */
                                                    /* Bit 14: Reserved */
#define ADC_STSR1_ERR_C                   (1 << 15) /* Bit 15: Error Algorithm C (ERR_C) */
#define ADC_STSR1_WDG_EOA_S               (1 << 16) /* Bit 16: Self-Test Watchdog End Of Algorithm S (WDG_EOA_S) */
                                                    /* Bit 17: Reserved */
#define ADC_STSR1_WDG_EOA_C               (1 << 18) /* Bit 18: Self-Test Watchdog End Of Algorithm C (WDG_EOA_C) */
                                                    /* Bit 19-22: Reserved */
#define ADC_STSR1_ST_EOC                  (1 << 23) /* Bit 23: Self-Test End Of Conversion (ST_EOC) */
#define ADC_STSR1_OVERWR                  (1 << 24) /* Bit 24: Self-Test Error Status Overwrite (OVERWR) */
#define ADC_STSR1_WDTERR                  (1 << 25) /* Bit 25: Self-Test Watchdog Timer Error (WDTERR) */
                                                    /* Bit 26: Reserved */
#define ADC_STSR1_WDSERR                  (1 << 27) /* Bit 27: Self-Test Watchdog Sequence Error (WDSERR) */
                                                    /* Bit 28-31: Reserved */

/* Self-Test Status Register 2 (STSR2) */

#define ADC_STSR2_DATA0_SHIFT             (0)       /* Bits 0-14: Conversion Data ERR_S1 (DATA0) */
#define ADC_STSR2_DATA0_MASK              (0x7fff << ADC_STSR2_DATA0_SHIFT)
                                                    /* Bits 15-31: Reserved */

/* Self-Test Status Register 3 (STSR3) */

#define ADC_STSR3_DATA0_SHIFT             (0)       /* Bits 0-14: Conversion Data ERR_S0 (DATA0) */
#define ADC_STSR3_DATA0_MASK              (0x7fff << ADC_STSR3_DATA0_SHIFT)
                                                    /* Bit 15: Reserved */
#define ADC_STSR3_DATA1_SHIFT             (16)      /* Bits 16-30: Conversion Data ERR_S2 (DATA1) */
#define ADC_STSR3_DATA1_MASK              (0x7fff << ADC_STSR3_DATA1_SHIFT)
                                                    /* Bit 31: Reserved */

/* Self-Test Status Register 4 (STSR4) */

                                                    /* Bits 0-15: Reserved */
#define ADC_STSR4_DATA1_SHIFT             (16)      /* Bits 16-30: Conversion Data ERR_C (DATA1) */
#define ADC_STSR4_DATA1_MASK              (0x7fff << ADC_STSR4_DATA1_SHIFT)
                                                    /* Bit 31: Reserved */

/* Self-Test Conversion Data Register 1 (STDR1) */

#define ADC_STDR1_TCDATA_SHIFT            (0)       /* Bits 0-14: Test Channel Conversion Data (TCDATA) */
#define ADC_STDR1_TCDATA_MASK             (0x7fff << ADC_STDR1_TCDATA_SHIFT)
                                                    /* Bits 15-17: Reserved */
#define ADC_STDR1_OVERWR                  (1 << 18) /* Bit 18: Conversion Data Overwrite Status (OVERWR) */
#define ADC_STDR1_VALID                   (1 << 19) /* Bit 19: Valid Conversion Data (VALID) */
                                                    /* Bits 20-31: Reserved */

/* Self-Test Analog Watchdog S0 Register (STAW0R) */

#define ADC_STAW0R_THRL_SHIFT             (0)       /* Bits 0-14: Lower Threshold Value (THRL) */
#define ADC_STAW0R_THRL_MASK              (0x7fff << ADC_STAW0R_THRL_SHIFT)
                                                    /* Bit 15: Reserved */
#define ADC_STAW0R_THRH_SHIFT             (16)      /* Bits 16-29: Higher Threshold Value (THRH) */
#define ADC_STAW0R_THRH_MASK              (0x3fff << ADC_STAW0R_THRH_SHIFT)
#define ADC_STAW0R_WDTE                   (1 << 30) /* Bit 30: Self-Test Watchdog Timer Enable (WDTE) */
#define ADC_STAW0R_AWDE                   (1 << 31) /* Bit 31: Self-Test Watchdog Enable (AWDE) */

/* Self-Test Analog Watchdog S1 Register (STAW1R) */

#define ADC_STAW1R_THRL_SHIFT             (0)       /* Bits 0-14: Lower Threshold Value (THRL) */
#define ADC_STAW1R_THRL_MASK              (0x7fff << ADC_STAW1R_THRL_SHIFT)
                                                    /* Bits 15-30: Reserved */
#define ADC_STAW1R_AWDE                   (1 << 31) /* Bit 31: Self-Test Watchdog Enable (AWDE) */

/* Self-Test Analog Watchdog S2 Register (STAW2R) */

#define ADC_STAW2R_THRL_SHIFT             (0)       /* Bits 0-14: Lower Threshold Value (THRL) */
#define ADC_STAW2R_THRL_MASK              (0x7fff << ADC_STAW2R_THRL_SHIFT)
                                                    /* Bits 15-30: Reserved */
#define ADC_STAW2R_AWDE                   (1 << 31) /* Bit 31: Self-Test Watchdog Enable (AWDE) */

/* Self-Test Analog Watchdog C0 Register (STAW4R) */

#define ADC_STAW4R_THRL_SHIFT             (0)       /* Bits 0-14: Lower Threshold Value (THRL) */
#define ADC_STAW4R_THRL_MASK              (0x7fff << ADC_STAW4R_THRL_SHIFT)
                                                    /* Bit 15: Reserved */
#define ADC_STAW4R_THRH_SHIFT             (16)      /* Bits 16-29: Higher Threshold Value (THRH) */
#define ADC_STAW4R_THRH_MASK              (0x3fff << ADC_STAW4R_THRH_SHIFT)
#define ADC_STAW4R_WDTE                   (1 << 30) /* Bit 30: Self-Test Watchdog Timer Enable (WDTE) */
#define ADC_STAW4R_AWDE                   (1 << 31) /* Bit 31: Self-Test Watchdog Enable (AWDE) */

/* Self-Test Analog Watchdog C Register (STAW5R) */

#define ADC_STAW5R_THRL_SHIFT             (0)       /* Bits 0-14: Lower Threshold Value (THRL) */
#define ADC_STAW5R_THRL_MASK              (0x7fff << ADC_STAW5R_THRL_SHIFT)
                                                    /* Bit 15: Reserved */
#define ADC_STAW5R_THRH_SHIFT             (16)      /* Bits 16-30: Higher Threshold Value (THRH) */
#define ADC_STAW5R_THRH_MASK              (0x7fff << ADC_STAW5R_THRH_SHIFT)
                                                    /* Bit 31: Reserved */

/* Analog Miscellaneous In/Out Register (AMSIO) */

                                                    /* Bits 0-16: Reserved */
#define ADC_AMSIO_HSEN_SHIFT              (17)      /* Bits 17-18: High-Speed Enable (HSEN) */
#define ADC_AMSIO_HSEN_MASK               (0x03 << ADC_AMSIO_HSEN_SHIFT)
#  define ADC_AMSIO_HSEN_NS               (0x00 << ADC_AMSIO_HSEN_SHIFT) /* Normal Conversion Speed */
#  define ADC_AMSIO_HSEN_HS               (0x03 << ADC_AMSIO_HSEN_SHIFT) /* High-Speed Conversion */

                                                    /* Bits 19-31: Reserved */

/* Control and Calibration Status Register (CALBISTREG) */

#define ADC_CALBISTREG_TEST_EN            (1 << 0)  /* Bit 0: Calibration Enable (TEST_EN) */
                                                    /* Bits 1-2: Reserved */
#define ADC_CALBISTREG_TEST_FAIL          (1 << 3)  /* Bit 3: Calibration Status (TEST_FAIL) */
#define ADC_CALBISTREG_AVG_EN             (1 << 4)  /* Bit 4: Calibration Averaging Enable (AVG_EN) */
#define ADC_CALBISTREG_NR_SMPL_SHIFT      (5)       /* Bits 5-6: Calibration Averaging Number (NR_SMPL) */
#define ADC_CALBISTREG_NR_SMPL_MASK       (0x03 << ADC_CALBISTREG_NR_SMPL_SHIFT)
#  define ADC_CALBISTREG_NR_SMPL_4SMPL    (0x00 << ADC_CALBISTREG_NR_SMPL_SHIFT) /*  4 Samples */
#  define ADC_CALBISTREG_NR_SMPL_8SMPL    (0x01 << ADC_CALBISTREG_NR_SMPL_SHIFT) /*  8 Samples */
#  define ADC_CALBISTREG_NR_SMPL_16SMPL   (0x02 << ADC_CALBISTREG_NR_SMPL_SHIFT) /* 16 Samples */
#  define ADC_CALBISTREG_NR_SMPL_32SMPL   (0x03 << ADC_CALBISTREG_NR_SMPL_SHIFT) /* 32 Samples */

                                                    /* Bits 7-13: Reserved */
#define ADC_CALBISTREG_CALSTFUL           (1 << 14) /* Bit 14: Calibration and Self-Test Full Range Comparison (CALSTFUL) */
#define ADC_CALBISTREG_C_T_BUSY           (1 << 15) /* Bit 15: Calibration Busy (C_T_BUSY) */
                                                    /* Bits 16-26: Reserved */
#define ADC_CALBISTREG_TSAMP_SHIFT        (27)      /* Bits 27-28: Sample Period in Calibration (TSAMP) */
#define ADC_CALBISTREG_TSAMP_MASK         (0x03 << ADC_CALBISTREG_TSAMP_SHIFT)
#  define ADC_CALBISTREG_TSAMP_22CYC      (0x00 << ADC_CALBISTREG_TSAMP_SHIFT) /* 22 Conversion Clock Cycles */
#  define ADC_CALBISTREG_TSAMP_8CYC       (0x01 << ADC_CALBISTREG_TSAMP_SHIFT) /*  8 Conversion Clock Cycles */
#  define ADC_CALBISTREG_TSAMP_16CYC      (0x02 << ADC_CALBISTREG_TSAMP_SHIFT) /* 16 Conversion Clock Cycles */
#  define ADC_CALBISTREG_TSAMP_32CYC      (0x03 << ADC_CALBISTREG_TSAMP_SHIFT) /* 32 Conversion Clock Cycles */

#define ADC_CALBISTREG_RESN_SHIFT         (29)      /* Bits 29-31: Conversion Resolution (RESN) */
#define ADC_CALBISTREG_RESN_MASK          (0x07 << ADC_CALBISTREG_RESN_SHIFT)
#  define ADC_CALBISTREG_RESN_14BIT       (0x00 << ADC_CALBISTREG_RESN_SHIFT) /* 14-Bit Resolution */
#  define ADC_CALBISTREG_RESN_12BIT       (0x01 << ADC_CALBISTREG_RESN_SHIFT) /* 12-Bit Resolution */
#  define ADC_CALBISTREG_RESN_10BIT       (0x02 << ADC_CALBISTREG_RESN_SHIFT) /* 10-Bit Resolution */
#  define ADC_CALBISTREG_RESN_8BIT        (0x03 << ADC_CALBISTREG_RESN_SHIFT) /*  8-Bit Resolution */

/* Offset and Gain User Register (OFSGNUSR) */

#define ADC_OFSGNUSR_OFFSET_USER_SHIFT    (0)       /* Bits 0-7: Offset User (OFFSET_USER) */
#define ADC_OFSGNUSR_OFFSET_USER_MASK     (0xff << ADC_OFSGNUSR_OFFSET_USER_SHIFT)
                                                    /* Bits 8-15: Reserved */
#define ADC_OFSGNUSR_GAIN_USER_SHIFT      (0)       /* Bits 16-25: Gain User (GAIN_USER) */
#define ADC_OFSGNUSR_GAIN_USER_MASK       (0x03ff << ADC_OFSGNUSR_GAIN_USER_SHIFT)
                                                    /* Bits 26-31: Reserved */

/* Calibration Value 2 (CAL2) */

                                                    /* Bits 0-14: Reserved */
#define ADC_CAL2_ENX                      (1 << 15) /* Bit 15: Enable X (ENX) */
                                                    /* Bits 16-31: Reserved */

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_ADC_H */
