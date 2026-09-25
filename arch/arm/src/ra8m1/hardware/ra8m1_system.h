/****************************************************************************
 * arch/arm/src/ra8m1/hardware/ra8m1_system.h
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

#ifndef __ARCH_ARM_SRC_RA8M1_HARDWARE_RA8M1_SYSTEM_H
#define __ARCH_ARM_SRC_RA8M1_HARDWARE_RA8M1_SYSTEM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/ra8m1_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define R_SYSTEM_SBYCR_OFFSET               0x000c  /* Standby Control Register (8-bits) */
#define R_SYSTEM_SSCR2_OFFSET               0x000e  /* Software Standby Control Register 2 (8-bits) */
#define R_SYSTEM_FLSCR_OFFSET               0x0010  /* Flash Standby Control Register (8-bits) */
#define R_SYSTEM_SCKDIVCR_OFFSET            0x0020  /* System Clock Division Control Register (32-bits) */
#define R_SYSTEM_SCKDIVCR2_OFFSET           0x0024  /* System Clock Division Control Register 2 (8-bits) */
#define R_SYSTEM_SCKSCR_OFFSET              0x0026  /* System Clock Source Control Register (8-bits) */
#define R_SYSTEM_PLLCCR_OFFSET              0x0028  /* PLL Clock Control Register (16-bits) */
#define R_SYSTEM_PLLCR_OFFSET               0x002a  /* PLL Control Register (8-bits) */
#define R_SYSTEM_BCKCR_OFFSET               0x0030  /* External Bus Clock Control Register (8-bits) */
#define R_SYSTEM_MOSCCR_OFFSET              0x0032  /* Main Clock Oscillator Control Register (8-bits) */
#define R_SYSTEM_HOCOCR_OFFSET              0x0036  /* High-Speed On-Chip Oscillator Control Register (8-bits) */
#define R_SYSTEM_MOCOCR_OFFSET              0x0038  /* Middle-Speed On-Chip Oscillator Control Register (8-bits) */
#define R_SYSTEM_FLLCR1_OFFSET              0x0039  /* FLL Control Register 1 (8-bits) */
#define R_SYSTEM_FLLCR2_OFFSET              0x003a  /* FLL Control Register 2 (16-bits) */
#define R_SYSTEM_OSCSF_OFFSET               0x003c  /* Oscillation Stabilization Flag Register (8-bits) */
#define R_SYSTEM_CKOCR_OFFSET               0x003e  /* Clock Out Control Register (8-bits) */
#define R_SYSTEM_TRCKCR_OFFSET              0x003f  /* Trace Clock Control Register (8-bits) */
#define R_SYSTEM_OSTDCR_OFFSET              0x0040  /* Oscillation Stop Detection Control Register (8-bits) */
#define R_SYSTEM_OSTDSR_OFFSET              0x0041  /* Oscillation Stop Detection Status Register (8-bits) */
#define R_SYSTEM_OSCMONR_OFFSET             0x0043  /* Oscillator Monitor Register (8-bits) */
#define R_SYSTEM_PLL2CCR_OFFSET             0x0048  /* PLL2 Clock Control Register (16-bits) */
#define R_SYSTEM_PLL2CR_OFFSET              0x004a  /* PLL2 Control Register (8-bits) */
#define R_SYSTEM_PLLCCR2_OFFSET             0x004c  /* PLL Clock Control Register 2 (16-bits) */
#define R_SYSTEM_PLL2CCR2_OFFSET            0x004e  /* PLL2 Clock Control Register 2 (16-bits) */
#define R_SYSTEM_EBCKOCR_OFFSET             0x0052  /* External Bus Clock Output Control Register (8-bits) */
#define R_SYSTEM_SDCKOCR_OFFSET             0x0053  /* SDRAM Clock Output Control Register (8-bits) */
#define R_SYSTEM_SCICKDIVCR_OFFSET          0x0054  /* SCI clock Division control register (8-bits) */
#define R_SYSTEM_SCICKCR_OFFSET             0x0055  /* SCI clock control register (8-bits) */
#define R_SYSTEM_SPICKDIVCR_OFFSET          0x0056  /* SPI clock Division control register (8-bits) */
#define R_SYSTEM_SPICKCR_OFFSET             0x0057  /* SPI clock control register (8-bits) */
#define R_SYSTEM_ADCCKDIVCR_OFFSET          0x005a  /* ADC clock Division control register (8-bits) */
#define R_SYSTEM_ADCCKCR_OFFSET             0x005b  /* ADC clock control register (8-bits) */
#define R_SYSTEM_GPTCKDIVCR_OFFSET          0x005c  /* GPT clock Division control register (8-bits) */
#define R_SYSTEM_GPTCKCR_OFFSET             0x005d  /* GPT clock control register (8-bits) */
#define R_SYSTEM_LCDCKDIVCR_OFFSET          0x005e  /* LCD clock Division control register (8-bits) */
#define R_SYSTEM_LCDCKCR_OFFSET             0x005f  /* LCD clock control register (8-bits) */
#define R_SYSTEM_MOCOUTCR_OFFSET            0x0061  /* MOCO User Trimming Control Register (8-bits) */
#define R_SYSTEM_HOCOUTCR_OFFSET            0x0062  /* HOCO User Trimming Control Register (8-bits) */
#define R_SYSTEM_USBCKDIVCR_OFFSET          0x006c  /* USB clock Division control register (8-bits) */
#define R_SYSTEM_OCTACKDIVCR_OFFSET         0x006d  /* Octal-SPI clock Division control register (8-bits) */
#define R_SYSTEM_CANFDCKDIVCR_OFFSET        0x006e  /* CANFD Core clock Division control register (8-bits) */
#define R_SYSTEM_USB60CKDIVCR_OFFSET        0x006f  /* USB60 clock Division control register (8-bits) */
#define R_SYSTEM_I3CCKDIVCR_OFFSET          0x0070  /* I3C clock Division control register (8-bits) */
#define R_SYSTEM_USBCKCR_OFFSET             0x0074  /* USB clock control register (8-bits) */
#define R_SYSTEM_OCTACKCR_OFFSET            0x0075  /* Octal-SPI clock control register (8-bits) */
#define R_SYSTEM_CANFDCKCR_OFFSET           0x0076  /* CANFD Core clock control register (8-bits) */
#define R_SYSTEM_USB60CKCR_OFFSET           0x0077  /* USB60 clock control register (8-bits) */
#define R_SYSTEM_I3CCKCR_OFFSET             0x0078  /* I3C clock control register (8-bits) */
#define R_SYSTEM_MOSCSCR_OFFSET             0x007c  /* Main Clock Oscillator Standby Control Register (8-bits) */
#define R_SYSTEM_HOCOSCR_OFFSET             0x007d  /* High-Speed On-Chip Oscillator Standby Control Register (8-bits) */
#define R_SYSTEM_OPCCR_OFFSET               0x00a0  /* Operating Power Control Register (8-bits) */
#define R_SYSTEM_MOSCWTCR_OFFSET            0x00a2  /* Main Clock Oscillator Wait Control Register (8-bits) */
#define R_SYSTEM_RSTSR1_OFFSET              0x00c0  /* Reset Status Register 1 (32-bits) */
#define R_SYSTEM_SYRACCR_OFFSET             0x00cc  /* System Register Access Control Register (8-bits) */
#define R_SYSTEM_PVDCR1_OFFSET              0x00e0  /* Voltage Monitor %s Circuit Control Register 1 (8-bits) */
#define R_SYSTEM_PVDSR_OFFSET               0x00e1  /* Voltage Monitor %s Circuit Status Register (8-bits) */
#define R_SYSTEM_CRVSYSCR_OFFSET            0x00f0  /* Clock Recovery System Control Register (8-bits) */
#define R_SYSTEM_PDCTRGD_OFFSET             0x0110  /* Graphics Power Domain Control Register (8-bits) */
#define R_SYSTEM_PDRAMSCR0_OFFSET           0x0140  /* SRAM power domain Standby Control Register 0 (16-bits) */
#define R_SYSTEM_PDRAMSCR1_OFFSET           0x0142  /* SRAM power domain Standby Control Register 1 (8-bits) */
#define R_SYSTEM_VBRSABAR_OFFSET            0x03b0  /* VBATT Backup Register Security Attribute Boundary Address Register (16-bits) */
#define R_SYSTEM_VBRPABARS_OFFSET           0x03b4  /* VBATT Backup Register Privilege Attribute Boundary Address Register for Secure Region (16-bits) */
#define R_SYSTEM_VBRPABARNS_OFFSET          0x03b8  /* VBATT Backup Register Privilege Attribute Boundary Address Register for Non-secure Region (16-bits) */
#define R_SYSTEM_CGFSAR_OFFSET              0x03c0  /* Clock Generation Function Security Attribute Register (32-bits) */
#define R_SYSTEM_RSTSAR_OFFSET              0x03c4  /* Reset Security Attribution Register (32-bits) */
#define R_SYSTEM_LPMSAR_OFFSET              0x03c8  /* Low Power Mode Security Attribution Register (32-bits) */
#define R_SYSTEM_PVDSAR_OFFSET              0x03cc  /* Programmable Voltage Detection Security Attribution Register (32-bits) */
#define R_SYSTEM_BBFSAR_OFFSET              0x03d0  /* Battery Backup Function Security Attribute Register (32-bits) */
#define R_SYSTEM_PGCSAR_OFFSET              0x03d8  /* Power Gating Control Security Attribution Register (32-bits) */
#define R_SYSTEM_DPFSAR_OFFSET              0x03e0  /* Deep Standby Interrupt Factor Security Attribution Register (32-bits) */
#define R_SYSTEM_RSCSAR_OFFSET              0x03e4  /* RAM Standby Control Security Attribution Register (32-bits) */
#define R_SYSTEM_PRCR_S_OFFSET              0x03fa  /* Protect Register for Secure Register (16-bits) */
#define R_SYSTEM_PRCR_NS_OFFSET             0x03fe  /* Protect Register for Non-secure Register (16-bits) */
#define R_SYSTEM_LOCOCR_OFFSET              0x0400  /* Low-Speed On-Chip Oscillator Control Register (8-bits) */
#define R_SYSTEM_LOCOUTCR_OFFSET            0x0402  /* LOCO User Trimming Control Register (8-bits) */
#define R_SYSTEM_DPSBYCR_OFFSET             0x0a00  /* Deep Standby Control Register (8-bits) */
#define R_SYSTEM_DPSWCR_OFFSET              0x0a04  /* Deep Standby Wait Control Register (8-bits) */
#define R_SYSTEM_DPSIER0_OFFSET             0x0a08  /* Deep Standby Interrupt Enable Register 0 (8-bits) */
#define R_SYSTEM_DPSIER1_OFFSET             0x0a0c  /* Deep Standby Interrupt Enable Register 1 (8-bits) */
#define R_SYSTEM_DPSIER2_OFFSET             0x0a10  /* Deep Standby Interrupt Enable Register 2 (8-bits) */
#define R_SYSTEM_DPSIER3_OFFSET             0x0a14  /* Deep Standby Interrupt Enable Register 3 (8-bits) */
#define R_SYSTEM_DPSIFR0_OFFSET             0x0a18  /* Deep Standby Interrupt Flag Register 0 (8-bits) */
#define R_SYSTEM_DPSIFR1_OFFSET             0x0a1c  /* Deep Standby Interrupt Flag Register 1 (8-bits) */
#define R_SYSTEM_DPSIFR2_OFFSET             0x0a20  /* Deep Standby Interrupt Flag Register 2 (8-bits) */
#define R_SYSTEM_DPSIFR3_OFFSET             0x0a24  /* Deep Standby Interrupt Flag Register 3 (8-bits) */
#define R_SYSTEM_DPSIEGR0_OFFSET            0x0a28  /* Deep Standby Interrupt Edge Register 0 (8-bits) */
#define R_SYSTEM_DPSIEGR1_OFFSET            0x0a2c  /* Deep Standby Interrupt Edge Register 1 (8-bits) */
#define R_SYSTEM_DPSIEGR2_OFFSET            0x0a30  /* Deep Standby Interrupt Edge Register 2 (8-bits) */
#define R_SYSTEM_SYOCDCR_OFFSET             0x0a38  /* System Control OCD Control Register (8-bits) */
#define R_SYSTEM_RSTSR0_OFFSET              0x0a40  /* Reset Status Register 0 (8-bits) */
#define R_SYSTEM_RSTSR2_OFFSET              0x0a44  /* Reset Status Register 2 (8-bits) */
#define R_SYSTEM_RSTSR3_OFFSET              0x0a48  /* Reset Status Register 3 (8-bits) */
#define R_SYSTEM_MOMCR_OFFSET               0x0a50  /* Main Clock Oscillator Mode Oscillation Control Register (8-bits) */
#define R_SYSTEM_FWEPROR_OFFSET             0x0a54  /* Flash Write Erase Protect Register (8-bits) */
#define R_SYSTEM_PVD1CMPCR_OFFSET           0x0a58  /* Voltage Monitor 1 Comparator Control Register (8-bits) */
#define R_SYSTEM_PVD2CMPCR_OFFSET           0x0a5c  /* Voltage Monitor 2 Comparator Control Register (8-bits) */
#define R_SYSTEM_PVDCR0_OFFSET              0x0a70  /* Voltage Monitor %s Circuit Control Register 0 (8-bits) */
#define R_SYSTEM_VBATTMNSELR_OFFSET         0x0a84  /* Battery Backup Voltage Monitor Function Select Register (8-bits) */
#define R_SYSTEM_VBTBPCR1_OFFSET            0x0a88  /* VBATT Battery Power Supply Control Register 1 (8-bits) */
#define R_SYSTEM_LPSCR_OFFSET               0x0a90  /* Low Power State Control Register (8-bits) */
#define R_SYSTEM_SSCR1_OFFSET               0x0a98  /* Software Standby Control Register 1 (8-bits) */
#define R_SYSTEM_LVOCR_OFFSET               0x0ab0  /* Low Power State Control Register (8-bits) */
#define R_SYSTEM_SYRSTMSK0_OFFSET           0x0ad0  /* System Reset Mask Control Register0 (8-bits) */
#define R_SYSTEM_SYRSTMSK1_OFFSET           0x0ad4  /* System Reset Mask Control Register1 (8-bits) */
#define R_SYSTEM_SYRSTMSK2_OFFSET           0x0ad8  /* System Reset Mask Control Register2 (8-bits) */
#define R_SYSTEM_PLL1LDOCR_OFFSET           0x0b04  /* PLL1-LDO Control Register (8-bits) */
#define R_SYSTEM_PLL2LDOCR_OFFSET           0x0b08  /* PLL2-LDO Control Register (8-bits) */
#define R_SYSTEM_HOCOLDOCR_OFFSET           0x0b0c  /* HOCO-LDO Control Register (8-bits) */
#define R_SYSTEM_MOMCR2_OFFSET              0x0b10  /* Main Clock Oscillator Mode Control Register 2 (8-bits) */
#define R_SYSTEM_SOSCCR_OFFSET              0x0c00  /* Sub-clock oscillator control register (8-bits) */
#define R_SYSTEM_SOMCR_OFFSET               0x0c01  /* Sub Clock Oscillator Mode Control Register (8-bits) */
#define R_SYSTEM_VBTBER_OFFSET              0x0c40  /* VBATT Backup Enable Register (8-bits) */
#define R_SYSTEM_VBTBPCR2_OFFSET            0x0c45  /* VBATT Battery Power Supply Control Register 2 (8-bits) */
#define R_SYSTEM_VBTBPSR_OFFSET             0x0c46  /* VBATT Battery Power Supply Status Register (8-bits) */
#define R_SYSTEM_VBTADSR_OFFSET             0x0c48  /* VBATT Tamper detection Status Register (8-bits) */
#define R_SYSTEM_VBTADCR1_OFFSET            0x0c49  /* VBATT Tamper detection Control Register 1 (8-bits) */
#define R_SYSTEM_VBTADCR2_OFFSET            0x0c4a  /* VBATT Tamper detection Control Register 2 (8-bits) */
#define R_SYSTEM_VBTICTLR_OFFSET            0x0c4c  /* VBATT Input Control Register (8-bits) */
#define R_SYSTEM_VBTICTLR2_OFFSET           0x0c4d  /* VBATT Input Control Register 2 (8-bits) */
#define R_SYSTEM_VBTIMONR_OFFSET            0x0c4e  /* VBATT Input Monitor Register (8-bits) */
#define R_SYSTEM_VBTBKR_OFFSET              0x0d00  /* VBATT Backup Register %s (8-bits) */

/* Register Addresses *******************************************************/

/* SYSTEM Registers */

#define R_SYSTEM_SBYCR                     (R_SYSTEM_BASE + R_SYSTEM_SBYCR_OFFSET)
#define R_SYSTEM_SSCR2                     (R_SYSTEM_BASE + R_SYSTEM_SSCR2_OFFSET)
#define R_SYSTEM_FLSCR                     (R_SYSTEM_BASE + R_SYSTEM_FLSCR_OFFSET)
#define R_SYSTEM_SCKDIVCR                  (R_SYSTEM_BASE + R_SYSTEM_SCKDIVCR_OFFSET)
#define R_SYSTEM_SCKDIVCR2                 (R_SYSTEM_BASE + R_SYSTEM_SCKDIVCR2_OFFSET)
#define R_SYSTEM_SCKSCR                    (R_SYSTEM_BASE + R_SYSTEM_SCKSCR_OFFSET)
#define R_SYSTEM_PLLCCR                    (R_SYSTEM_BASE + R_SYSTEM_PLLCCR_OFFSET)
#define R_SYSTEM_PLLCR                     (R_SYSTEM_BASE + R_SYSTEM_PLLCR_OFFSET)
#define R_SYSTEM_BCKCR                     (R_SYSTEM_BASE + R_SYSTEM_BCKCR_OFFSET)
#define R_SYSTEM_MOSCCR                    (R_SYSTEM_BASE + R_SYSTEM_MOSCCR_OFFSET)
#define R_SYSTEM_HOCOCR                    (R_SYSTEM_BASE + R_SYSTEM_HOCOCR_OFFSET)
#define R_SYSTEM_MOCOCR                    (R_SYSTEM_BASE + R_SYSTEM_MOCOCR_OFFSET)
#define R_SYSTEM_FLLCR1                    (R_SYSTEM_BASE + R_SYSTEM_FLLCR1_OFFSET)
#define R_SYSTEM_FLLCR2                    (R_SYSTEM_BASE + R_SYSTEM_FLLCR2_OFFSET)
#define R_SYSTEM_OSCSF                     (R_SYSTEM_BASE + R_SYSTEM_OSCSF_OFFSET)
#define R_SYSTEM_CKOCR                     (R_SYSTEM_BASE + R_SYSTEM_CKOCR_OFFSET)
#define R_SYSTEM_TRCKCR                    (R_SYSTEM_BASE + R_SYSTEM_TRCKCR_OFFSET)
#define R_SYSTEM_OSTDCR                    (R_SYSTEM_BASE + R_SYSTEM_OSTDCR_OFFSET)
#define R_SYSTEM_OSTDSR                    (R_SYSTEM_BASE + R_SYSTEM_OSTDSR_OFFSET)
#define R_SYSTEM_OSCMONR                   (R_SYSTEM_BASE + R_SYSTEM_OSCMONR_OFFSET)
#define R_SYSTEM_PLL2CCR                   (R_SYSTEM_BASE + R_SYSTEM_PLL2CCR_OFFSET)
#define R_SYSTEM_PLL2CR                    (R_SYSTEM_BASE + R_SYSTEM_PLL2CR_OFFSET)
#define R_SYSTEM_PLLCCR2                   (R_SYSTEM_BASE + R_SYSTEM_PLLCCR2_OFFSET)
#define R_SYSTEM_PLL2CCR2                  (R_SYSTEM_BASE + R_SYSTEM_PLL2CCR2_OFFSET)
#define R_SYSTEM_EBCKOCR                   (R_SYSTEM_BASE + R_SYSTEM_EBCKOCR_OFFSET)
#define R_SYSTEM_SDCKOCR                   (R_SYSTEM_BASE + R_SYSTEM_SDCKOCR_OFFSET)
#define R_SYSTEM_SCICKDIVCR                (R_SYSTEM_BASE + R_SYSTEM_SCICKDIVCR_OFFSET)
#define R_SYSTEM_SCICKCR                   (R_SYSTEM_BASE + R_SYSTEM_SCICKCR_OFFSET)
#define R_SYSTEM_SPICKDIVCR                (R_SYSTEM_BASE + R_SYSTEM_SPICKDIVCR_OFFSET)
#define R_SYSTEM_SPICKCR                   (R_SYSTEM_BASE + R_SYSTEM_SPICKCR_OFFSET)
#define R_SYSTEM_ADCCKDIVCR                (R_SYSTEM_BASE + R_SYSTEM_ADCCKDIVCR_OFFSET)
#define R_SYSTEM_ADCCKCR                   (R_SYSTEM_BASE + R_SYSTEM_ADCCKCR_OFFSET)
#define R_SYSTEM_GPTCKDIVCR                (R_SYSTEM_BASE + R_SYSTEM_GPTCKDIVCR_OFFSET)
#define R_SYSTEM_GPTCKCR                   (R_SYSTEM_BASE + R_SYSTEM_GPTCKCR_OFFSET)
#define R_SYSTEM_LCDCKDIVCR                (R_SYSTEM_BASE + R_SYSTEM_LCDCKDIVCR_OFFSET)
#define R_SYSTEM_LCDCKCR                   (R_SYSTEM_BASE + R_SYSTEM_LCDCKCR_OFFSET)
#define R_SYSTEM_MOCOUTCR                  (R_SYSTEM_BASE + R_SYSTEM_MOCOUTCR_OFFSET)
#define R_SYSTEM_HOCOUTCR                  (R_SYSTEM_BASE + R_SYSTEM_HOCOUTCR_OFFSET)
#define R_SYSTEM_USBCKDIVCR                (R_SYSTEM_BASE + R_SYSTEM_USBCKDIVCR_OFFSET)
#define R_SYSTEM_OCTACKDIVCR               (R_SYSTEM_BASE + R_SYSTEM_OCTACKDIVCR_OFFSET)
#define R_SYSTEM_CANFDCKDIVCR              (R_SYSTEM_BASE + R_SYSTEM_CANFDCKDIVCR_OFFSET)
#define R_SYSTEM_USB60CKDIVCR              (R_SYSTEM_BASE + R_SYSTEM_USB60CKDIVCR_OFFSET)
#define R_SYSTEM_I3CCKDIVCR                (R_SYSTEM_BASE + R_SYSTEM_I3CCKDIVCR_OFFSET)
#define R_SYSTEM_USBCKCR                   (R_SYSTEM_BASE + R_SYSTEM_USBCKCR_OFFSET)
#define R_SYSTEM_OCTACKCR                  (R_SYSTEM_BASE + R_SYSTEM_OCTACKCR_OFFSET)
#define R_SYSTEM_CANFDCKCR                 (R_SYSTEM_BASE + R_SYSTEM_CANFDCKCR_OFFSET)
#define R_SYSTEM_USB60CKCR                 (R_SYSTEM_BASE + R_SYSTEM_USB60CKCR_OFFSET)
#define R_SYSTEM_I3CCKCR                   (R_SYSTEM_BASE + R_SYSTEM_I3CCKCR_OFFSET)
#define R_SYSTEM_MOSCSCR                   (R_SYSTEM_BASE + R_SYSTEM_MOSCSCR_OFFSET)
#define R_SYSTEM_HOCOSCR                   (R_SYSTEM_BASE + R_SYSTEM_HOCOSCR_OFFSET)
#define R_SYSTEM_OPCCR                     (R_SYSTEM_BASE + R_SYSTEM_OPCCR_OFFSET)
#define R_SYSTEM_MOSCWTCR                  (R_SYSTEM_BASE + R_SYSTEM_MOSCWTCR_OFFSET)
#define R_SYSTEM_RSTSR1                    (R_SYSTEM_BASE + R_SYSTEM_RSTSR1_OFFSET)
#define R_SYSTEM_SYRACCR                   (R_SYSTEM_BASE + R_SYSTEM_SYRACCR_OFFSET)
#define R_SYSTEM_PVDCR1(p)                 (R_SYSTEM_BASE + R_SYSTEM_PVDCR1_OFFSET + (p)*0x0002)
#define R_SYSTEM_PVDSR(p)                  (R_SYSTEM_BASE + R_SYSTEM_PVDSR_OFFSET + (p)*0x0002)
#define R_SYSTEM_CRVSYSCR                  (R_SYSTEM_BASE + R_SYSTEM_CRVSYSCR_OFFSET)
#define R_SYSTEM_PDCTRGD                   (R_SYSTEM_BASE + R_SYSTEM_PDCTRGD_OFFSET)
#define R_SYSTEM_PDRAMSCR0                 (R_SYSTEM_BASE + R_SYSTEM_PDRAMSCR0_OFFSET)
#define R_SYSTEM_PDRAMSCR1                 (R_SYSTEM_BASE + R_SYSTEM_PDRAMSCR1_OFFSET)
#define R_SYSTEM_VBRSABAR                  (R_SYSTEM_BASE + R_SYSTEM_VBRSABAR_OFFSET)
#define R_SYSTEM_VBRPABARS                 (R_SYSTEM_BASE + R_SYSTEM_VBRPABARS_OFFSET)
#define R_SYSTEM_VBRPABARNS                (R_SYSTEM_BASE + R_SYSTEM_VBRPABARNS_OFFSET)
#define R_SYSTEM_CGFSAR                    (R_SYSTEM_BASE + R_SYSTEM_CGFSAR_OFFSET)
#define R_SYSTEM_RSTSAR                    (R_SYSTEM_BASE + R_SYSTEM_RSTSAR_OFFSET)
#define R_SYSTEM_LPMSAR                    (R_SYSTEM_BASE + R_SYSTEM_LPMSAR_OFFSET)
#define R_SYSTEM_PVDSAR                    (R_SYSTEM_BASE + R_SYSTEM_PVDSAR_OFFSET)
#define R_SYSTEM_BBFSAR                    (R_SYSTEM_BASE + R_SYSTEM_BBFSAR_OFFSET)
#define R_SYSTEM_PGCSAR                    (R_SYSTEM_BASE + R_SYSTEM_PGCSAR_OFFSET)
#define R_SYSTEM_DPFSAR                    (R_SYSTEM_BASE + R_SYSTEM_DPFSAR_OFFSET)
#define R_SYSTEM_RSCSAR                    (R_SYSTEM_BASE + R_SYSTEM_RSCSAR_OFFSET)
#define R_SYSTEM_PRCR_S                    (R_SYSTEM_BASE + R_SYSTEM_PRCR_S_OFFSET)
#define R_SYSTEM_PRCR_NS                   (R_SYSTEM_BASE + R_SYSTEM_PRCR_NS_OFFSET)
#define R_SYSTEM_LOCOCR                    (R_SYSTEM_BASE + R_SYSTEM_LOCOCR_OFFSET)
#define R_SYSTEM_LOCOUTCR                  (R_SYSTEM_BASE + R_SYSTEM_LOCOUTCR_OFFSET)
#define R_SYSTEM_DPSBYCR                   (R_SYSTEM_BASE + R_SYSTEM_DPSBYCR_OFFSET)
#define R_SYSTEM_DPSWCR                    (R_SYSTEM_BASE + R_SYSTEM_DPSWCR_OFFSET)
#define R_SYSTEM_DPSIER0                   (R_SYSTEM_BASE + R_SYSTEM_DPSIER0_OFFSET)
#define R_SYSTEM_DPSIER1                   (R_SYSTEM_BASE + R_SYSTEM_DPSIER1_OFFSET)
#define R_SYSTEM_DPSIER2                   (R_SYSTEM_BASE + R_SYSTEM_DPSIER2_OFFSET)
#define R_SYSTEM_DPSIER3                   (R_SYSTEM_BASE + R_SYSTEM_DPSIER3_OFFSET)
#define R_SYSTEM_DPSIFR0                   (R_SYSTEM_BASE + R_SYSTEM_DPSIFR0_OFFSET)
#define R_SYSTEM_DPSIFR1                   (R_SYSTEM_BASE + R_SYSTEM_DPSIFR1_OFFSET)
#define R_SYSTEM_DPSIFR2                   (R_SYSTEM_BASE + R_SYSTEM_DPSIFR2_OFFSET)
#define R_SYSTEM_DPSIFR3                   (R_SYSTEM_BASE + R_SYSTEM_DPSIFR3_OFFSET)
#define R_SYSTEM_DPSIEGR0                  (R_SYSTEM_BASE + R_SYSTEM_DPSIEGR0_OFFSET)
#define R_SYSTEM_DPSIEGR1                  (R_SYSTEM_BASE + R_SYSTEM_DPSIEGR1_OFFSET)
#define R_SYSTEM_DPSIEGR2                  (R_SYSTEM_BASE + R_SYSTEM_DPSIEGR2_OFFSET)
#define R_SYSTEM_SYOCDCR                   (R_SYSTEM_BASE + R_SYSTEM_SYOCDCR_OFFSET)
#define R_SYSTEM_RSTSR0                    (R_SYSTEM_BASE + R_SYSTEM_RSTSR0_OFFSET)
#define R_SYSTEM_RSTSR2                    (R_SYSTEM_BASE + R_SYSTEM_RSTSR2_OFFSET)
#define R_SYSTEM_RSTSR3                    (R_SYSTEM_BASE + R_SYSTEM_RSTSR3_OFFSET)
#define R_SYSTEM_MOMCR                     (R_SYSTEM_BASE + R_SYSTEM_MOMCR_OFFSET)
#define R_SYSTEM_FWEPROR                   (R_SYSTEM_BASE + R_SYSTEM_FWEPROR_OFFSET)
#define R_SYSTEM_PVD1CMPCR                 (R_SYSTEM_BASE + R_SYSTEM_PVD1CMPCR_OFFSET)
#define R_SYSTEM_PVD2CMPCR                 (R_SYSTEM_BASE + R_SYSTEM_PVD2CMPCR_OFFSET)
#define R_SYSTEM_PVDCR0(p)                 (R_SYSTEM_BASE + R_SYSTEM_PVDCR0_OFFSET + (p)*0x0004)
#define R_SYSTEM_VBATTMNSELR               (R_SYSTEM_BASE + R_SYSTEM_VBATTMNSELR_OFFSET)
#define R_SYSTEM_VBTBPCR1                  (R_SYSTEM_BASE + R_SYSTEM_VBTBPCR1_OFFSET)
#define R_SYSTEM_LPSCR                     (R_SYSTEM_BASE + R_SYSTEM_LPSCR_OFFSET)
#define R_SYSTEM_SSCR1                     (R_SYSTEM_BASE + R_SYSTEM_SSCR1_OFFSET)
#define R_SYSTEM_LVOCR                     (R_SYSTEM_BASE + R_SYSTEM_LVOCR_OFFSET)
#define R_SYSTEM_SYRSTMSK0                 (R_SYSTEM_BASE + R_SYSTEM_SYRSTMSK0_OFFSET)
#define R_SYSTEM_SYRSTMSK1                 (R_SYSTEM_BASE + R_SYSTEM_SYRSTMSK1_OFFSET)
#define R_SYSTEM_SYRSTMSK2                 (R_SYSTEM_BASE + R_SYSTEM_SYRSTMSK2_OFFSET)
#define R_SYSTEM_PLL1LDOCR                 (R_SYSTEM_BASE + R_SYSTEM_PLL1LDOCR_OFFSET)
#define R_SYSTEM_PLL2LDOCR                 (R_SYSTEM_BASE + R_SYSTEM_PLL2LDOCR_OFFSET)
#define R_SYSTEM_HOCOLDOCR                 (R_SYSTEM_BASE + R_SYSTEM_HOCOLDOCR_OFFSET)
#define R_SYSTEM_MOMCR2                    (R_SYSTEM_BASE + R_SYSTEM_MOMCR2_OFFSET)
#define R_SYSTEM_SOSCCR                    (R_SYSTEM_BASE + R_SYSTEM_SOSCCR_OFFSET)
#define R_SYSTEM_SOMCR                     (R_SYSTEM_BASE + R_SYSTEM_SOMCR_OFFSET)
#define R_SYSTEM_VBTBER                    (R_SYSTEM_BASE + R_SYSTEM_VBTBER_OFFSET)
#define R_SYSTEM_VBTBPCR2                  (R_SYSTEM_BASE + R_SYSTEM_VBTBPCR2_OFFSET)
#define R_SYSTEM_VBTBPSR                   (R_SYSTEM_BASE + R_SYSTEM_VBTBPSR_OFFSET)
#define R_SYSTEM_VBTADSR                   (R_SYSTEM_BASE + R_SYSTEM_VBTADSR_OFFSET)
#define R_SYSTEM_VBTADCR1                  (R_SYSTEM_BASE + R_SYSTEM_VBTADCR1_OFFSET)
#define R_SYSTEM_VBTADCR2                  (R_SYSTEM_BASE + R_SYSTEM_VBTADCR2_OFFSET)
#define R_SYSTEM_VBTICTLR                  (R_SYSTEM_BASE + R_SYSTEM_VBTICTLR_OFFSET)
#define R_SYSTEM_VBTICTLR2                 (R_SYSTEM_BASE + R_SYSTEM_VBTICTLR2_OFFSET)
#define R_SYSTEM_VBTIMONR                  (R_SYSTEM_BASE + R_SYSTEM_VBTIMONR_OFFSET)
#define R_SYSTEM_VBTBKR(p)                 (R_SYSTEM_BASE + R_SYSTEM_VBTBKR_OFFSET + (p)*0x0001)

/* Register Bitfield Definitions ********************************************/

/* Standby Control Register (8-bits) ****************************************/

#define R_SYSTEM_SBYCR_OPE (1 <<  6)  /* 40: Output Port Enable */

/* Software Standby Control Register 2 (8-bits) *****************************/

#define R_SYSTEM_SSCR2_SS1RSF (1 <<  0)  /* 01: Software Standby 1 regulator status flag */

/* Flash Standby Control Register (8-bits) **********************************/

#define R_SYSTEM_FLSCR_FLSWCF (1 <<  0)  /* 01: Flash Stabilization wait completion flag */

/* System Clock Division Control Register (32-bits) *************************/

#define R_SYSTEM_SCKDIVCR_PCKD_SHIFT (0)
#define R_SYSTEM_SCKDIVCR_PCKD_MASK (0xf)
#  define R_SYSTEM_SCKDIVCR_PCKD_V1_VALUE_AFTER_RESET (0 << R_SYSTEM_SCKDIVCR_PCKD_SHIFT)  /* /1 (value after reset) */
#  define R_SYSTEM_SCKDIVCR_PCKD_DIV_2 (1 << R_SYSTEM_SCKDIVCR_PCKD_SHIFT)                 /* /2 */
#  define R_SYSTEM_SCKDIVCR_PCKD_DIV_4 (2 << R_SYSTEM_SCKDIVCR_PCKD_SHIFT)                 /* /4 */
#  define R_SYSTEM_SCKDIVCR_PCKD_DIV_8 (3 << R_SYSTEM_SCKDIVCR_PCKD_SHIFT)                 /* /8 */
#  define R_SYSTEM_SCKDIVCR_PCKD_DIV_16 (4 << R_SYSTEM_SCKDIVCR_PCKD_SHIFT)                /* /16 */
#  define R_SYSTEM_SCKDIVCR_PCKD_DIV_32 (5 << R_SYSTEM_SCKDIVCR_PCKD_SHIFT)                /* /32 */
#  define R_SYSTEM_SCKDIVCR_PCKD_DIV_64 (6 << R_SYSTEM_SCKDIVCR_PCKD_SHIFT)                /* /64 */
#  define R_SYSTEM_SCKDIVCR_PCKD_DIV_3 (8 << R_SYSTEM_SCKDIVCR_PCKD_SHIFT)                 /* /3 */
#  define R_SYSTEM_SCKDIVCR_PCKD_DIV_6 (9 << R_SYSTEM_SCKDIVCR_PCKD_SHIFT)                 /* /6 */
#  define R_SYSTEM_SCKDIVCR_PCKD_DIV_12 (10 << R_SYSTEM_SCKDIVCR_PCKD_SHIFT)               /* /12 */
#define R_SYSTEM_SCKDIVCR_PCKC_SHIFT (4)
#define R_SYSTEM_SCKDIVCR_PCKC_MASK (0xf)
#  define R_SYSTEM_SCKDIVCR_PCKC_V1_VALUE_AFTER_RESET (0 << R_SYSTEM_SCKDIVCR_PCKC_SHIFT)  /* /1 (value after reset) */
#  define R_SYSTEM_SCKDIVCR_PCKC_DIV_2 (1 << R_SYSTEM_SCKDIVCR_PCKC_SHIFT)                 /* /2 */
#  define R_SYSTEM_SCKDIVCR_PCKC_DIV_4 (2 << R_SYSTEM_SCKDIVCR_PCKC_SHIFT)                 /* /4 */
#  define R_SYSTEM_SCKDIVCR_PCKC_DIV_8 (3 << R_SYSTEM_SCKDIVCR_PCKC_SHIFT)                 /* /8 */
#  define R_SYSTEM_SCKDIVCR_PCKC_DIV_16 (4 << R_SYSTEM_SCKDIVCR_PCKC_SHIFT)                /* /16 */
#  define R_SYSTEM_SCKDIVCR_PCKC_DIV_32 (5 << R_SYSTEM_SCKDIVCR_PCKC_SHIFT)                /* /32 */
#  define R_SYSTEM_SCKDIVCR_PCKC_DIV_64 (6 << R_SYSTEM_SCKDIVCR_PCKC_SHIFT)                /* /64 */
#  define R_SYSTEM_SCKDIVCR_PCKC_DIV_3 (8 << R_SYSTEM_SCKDIVCR_PCKC_SHIFT)                 /* /3 */
#  define R_SYSTEM_SCKDIVCR_PCKC_DIV_6 (9 << R_SYSTEM_SCKDIVCR_PCKC_SHIFT)                 /* /6 */
#  define R_SYSTEM_SCKDIVCR_PCKC_DIV_12 (10 << R_SYSTEM_SCKDIVCR_PCKC_SHIFT)               /* /12 */
#define R_SYSTEM_SCKDIVCR_PCKB_SHIFT (8)
#define R_SYSTEM_SCKDIVCR_PCKB_MASK (0xf)
#  define R_SYSTEM_SCKDIVCR_PCKB_V1_VALUE_AFTER_RESET (0 << R_SYSTEM_SCKDIVCR_PCKB_SHIFT)  /* /1 (value after reset) */
#  define R_SYSTEM_SCKDIVCR_PCKB_DIV_2 (1 << R_SYSTEM_SCKDIVCR_PCKB_SHIFT)                 /* /2 */
#  define R_SYSTEM_SCKDIVCR_PCKB_DIV_4 (2 << R_SYSTEM_SCKDIVCR_PCKB_SHIFT)                 /* /4 */
#  define R_SYSTEM_SCKDIVCR_PCKB_DIV_8 (3 << R_SYSTEM_SCKDIVCR_PCKB_SHIFT)                 /* /8 */
#  define R_SYSTEM_SCKDIVCR_PCKB_DIV_16 (4 << R_SYSTEM_SCKDIVCR_PCKB_SHIFT)                /* /16 */
#  define R_SYSTEM_SCKDIVCR_PCKB_DIV_32 (5 << R_SYSTEM_SCKDIVCR_PCKB_SHIFT)                /* /32 */
#  define R_SYSTEM_SCKDIVCR_PCKB_DIV_64 (6 << R_SYSTEM_SCKDIVCR_PCKB_SHIFT)                /* /64 */
#  define R_SYSTEM_SCKDIVCR_PCKB_DIV_3 (8 << R_SYSTEM_SCKDIVCR_PCKB_SHIFT)                 /* /3 */
#  define R_SYSTEM_SCKDIVCR_PCKB_DIV_6 (9 << R_SYSTEM_SCKDIVCR_PCKB_SHIFT)                 /* /6 */
#  define R_SYSTEM_SCKDIVCR_PCKB_DIV_12 (10 << R_SYSTEM_SCKDIVCR_PCKB_SHIFT)               /* /12 */
#define R_SYSTEM_SCKDIVCR_PCKA_SHIFT (12)
#define R_SYSTEM_SCKDIVCR_PCKA_MASK (0xf)
#  define R_SYSTEM_SCKDIVCR_PCKA_V1_VALUE_AFTER_RESET (0 << R_SYSTEM_SCKDIVCR_PCKA_SHIFT)  /* /1 (value after reset) */
#  define R_SYSTEM_SCKDIVCR_PCKA_DIV_2 (1 << R_SYSTEM_SCKDIVCR_PCKA_SHIFT)                 /* /2 */
#  define R_SYSTEM_SCKDIVCR_PCKA_DIV_4 (2 << R_SYSTEM_SCKDIVCR_PCKA_SHIFT)                 /* /4 */
#  define R_SYSTEM_SCKDIVCR_PCKA_DIV_8 (3 << R_SYSTEM_SCKDIVCR_PCKA_SHIFT)                 /* /8 */
#  define R_SYSTEM_SCKDIVCR_PCKA_DIV_16 (4 << R_SYSTEM_SCKDIVCR_PCKA_SHIFT)                /* /16 */
#  define R_SYSTEM_SCKDIVCR_PCKA_DIV_32 (5 << R_SYSTEM_SCKDIVCR_PCKA_SHIFT)                /* /32 */
#  define R_SYSTEM_SCKDIVCR_PCKA_DIV_64 (6 << R_SYSTEM_SCKDIVCR_PCKA_SHIFT)                /* /64 */
#  define R_SYSTEM_SCKDIVCR_PCKA_DIV_3 (8 << R_SYSTEM_SCKDIVCR_PCKA_SHIFT)                 /* /3 */
#  define R_SYSTEM_SCKDIVCR_PCKA_DIV_6 (9 << R_SYSTEM_SCKDIVCR_PCKA_SHIFT)                 /* /6 */
#  define R_SYSTEM_SCKDIVCR_PCKA_DIV_12 (10 << R_SYSTEM_SCKDIVCR_PCKA_SHIFT)               /* /12 */
#define R_SYSTEM_SCKDIVCR_BCK_SHIFT (16)
#define R_SYSTEM_SCKDIVCR_BCK_MASK (0xf)
#  define R_SYSTEM_SCKDIVCR_BCK_V1_VALUE_AFTER_RESET (0 << R_SYSTEM_SCKDIVCR_BCK_SHIFT)    /* /1 (value after reset) */
#  define R_SYSTEM_SCKDIVCR_BCK_DIV_2 (1 << R_SYSTEM_SCKDIVCR_BCK_SHIFT)                   /* /2 */
#  define R_SYSTEM_SCKDIVCR_BCK_DIV_4 (2 << R_SYSTEM_SCKDIVCR_BCK_SHIFT)                   /* /4 */
#  define R_SYSTEM_SCKDIVCR_BCK_DIV_8 (3 << R_SYSTEM_SCKDIVCR_BCK_SHIFT)                   /* /8 */
#  define R_SYSTEM_SCKDIVCR_BCK_DIV_16 (4 << R_SYSTEM_SCKDIVCR_BCK_SHIFT)                  /* /16 */
#  define R_SYSTEM_SCKDIVCR_BCK_DIV_32 (5 << R_SYSTEM_SCKDIVCR_BCK_SHIFT)                  /* /32 */
#  define R_SYSTEM_SCKDIVCR_BCK_DIV_64 (6 << R_SYSTEM_SCKDIVCR_BCK_SHIFT)                  /* /64 */
#  define R_SYSTEM_SCKDIVCR_BCK_DIV_3 (8 << R_SYSTEM_SCKDIVCR_BCK_SHIFT)                   /* /3 */
#  define R_SYSTEM_SCKDIVCR_BCK_DIV_6 (9 << R_SYSTEM_SCKDIVCR_BCK_SHIFT)                   /* /6 */
#  define R_SYSTEM_SCKDIVCR_BCK_DIV_12 (10 << R_SYSTEM_SCKDIVCR_BCK_SHIFT)                 /* /12 */
#define R_SYSTEM_SCKDIVCR_PCKE_SHIFT (20)
#define R_SYSTEM_SCKDIVCR_PCKE_MASK (0xf)
#  define R_SYSTEM_SCKDIVCR_PCKE_V1_VALUE_AFTER_RESET (0 << R_SYSTEM_SCKDIVCR_PCKE_SHIFT)  /* /1 (value after reset) */
#  define R_SYSTEM_SCKDIVCR_PCKE_DIV_2 (1 << R_SYSTEM_SCKDIVCR_PCKE_SHIFT)                 /* /2 */
#  define R_SYSTEM_SCKDIVCR_PCKE_DIV_4 (2 << R_SYSTEM_SCKDIVCR_PCKE_SHIFT)                 /* /4 */
#  define R_SYSTEM_SCKDIVCR_PCKE_DIV_8 (3 << R_SYSTEM_SCKDIVCR_PCKE_SHIFT)                 /* /8 */
#  define R_SYSTEM_SCKDIVCR_PCKE_DIV_16 (4 << R_SYSTEM_SCKDIVCR_PCKE_SHIFT)                /* /16 */
#  define R_SYSTEM_SCKDIVCR_PCKE_DIV_32 (5 << R_SYSTEM_SCKDIVCR_PCKE_SHIFT)                /* /32 */
#  define R_SYSTEM_SCKDIVCR_PCKE_DIV_64 (6 << R_SYSTEM_SCKDIVCR_PCKE_SHIFT)                /* /64 */
#  define R_SYSTEM_SCKDIVCR_PCKE_DIV_3 (8 << R_SYSTEM_SCKDIVCR_PCKE_SHIFT)                 /* /3 */
#  define R_SYSTEM_SCKDIVCR_PCKE_DIV_6 (9 << R_SYSTEM_SCKDIVCR_PCKE_SHIFT)                 /* /6 */
#  define R_SYSTEM_SCKDIVCR_PCKE_DIV_12 (10 << R_SYSTEM_SCKDIVCR_PCKE_SHIFT)               /* /12 */
#define R_SYSTEM_SCKDIVCR_ICK_SHIFT (24)
#define R_SYSTEM_SCKDIVCR_ICK_MASK (0xf)
#  define R_SYSTEM_SCKDIVCR_ICK_V1_VALUE_AFTER_RESET (0 << R_SYSTEM_SCKDIVCR_ICK_SHIFT)    /* /1 (value after reset) */
#  define R_SYSTEM_SCKDIVCR_ICK_DIV_2 (1 << R_SYSTEM_SCKDIVCR_ICK_SHIFT)                   /* /2 */
#  define R_SYSTEM_SCKDIVCR_ICK_DIV_4 (2 << R_SYSTEM_SCKDIVCR_ICK_SHIFT)                   /* /4 */
#  define R_SYSTEM_SCKDIVCR_ICK_DIV_8 (3 << R_SYSTEM_SCKDIVCR_ICK_SHIFT)                   /* /8 */
#  define R_SYSTEM_SCKDIVCR_ICK_DIV_16 (4 << R_SYSTEM_SCKDIVCR_ICK_SHIFT)                  /* /16 */
#  define R_SYSTEM_SCKDIVCR_ICK_DIV_32 (5 << R_SYSTEM_SCKDIVCR_ICK_SHIFT)                  /* /32 */
#  define R_SYSTEM_SCKDIVCR_ICK_DIV_64 (6 << R_SYSTEM_SCKDIVCR_ICK_SHIFT)                  /* /64 */
#  define R_SYSTEM_SCKDIVCR_ICK_DIV_3 (8 << R_SYSTEM_SCKDIVCR_ICK_SHIFT)                   /* /3 */
#  define R_SYSTEM_SCKDIVCR_ICK_DIV_6 (9 << R_SYSTEM_SCKDIVCR_ICK_SHIFT)                   /* /6 */
#  define R_SYSTEM_SCKDIVCR_ICK_DIV_12 (10 << R_SYSTEM_SCKDIVCR_ICK_SHIFT)                 /* /12 */
#define R_SYSTEM_SCKDIVCR_FCK_SHIFT (28)
#define R_SYSTEM_SCKDIVCR_FCK_MASK (0xf)
#  define R_SYSTEM_SCKDIVCR_FCK_V1_VALUE_AFTER_RESET (0 << R_SYSTEM_SCKDIVCR_FCK_SHIFT)    /* /1 (value after reset) */
#  define R_SYSTEM_SCKDIVCR_FCK_DIV_2 (1 << R_SYSTEM_SCKDIVCR_FCK_SHIFT)                   /* /2 */
#  define R_SYSTEM_SCKDIVCR_FCK_DIV_4 (2 << R_SYSTEM_SCKDIVCR_FCK_SHIFT)                   /* /4 */
#  define R_SYSTEM_SCKDIVCR_FCK_DIV_8 (3 << R_SYSTEM_SCKDIVCR_FCK_SHIFT)                   /* /8 */
#  define R_SYSTEM_SCKDIVCR_FCK_DIV_16 (4 << R_SYSTEM_SCKDIVCR_FCK_SHIFT)                  /* /16 */
#  define R_SYSTEM_SCKDIVCR_FCK_DIV_32 (5 << R_SYSTEM_SCKDIVCR_FCK_SHIFT)                  /* /32 */
#  define R_SYSTEM_SCKDIVCR_FCK_DIV_64 (6 << R_SYSTEM_SCKDIVCR_FCK_SHIFT)                  /* /64 */
#  define R_SYSTEM_SCKDIVCR_FCK_DIV_3 (8 << R_SYSTEM_SCKDIVCR_FCK_SHIFT)                   /* /3 */
#  define R_SYSTEM_SCKDIVCR_FCK_DIV_6 (9 << R_SYSTEM_SCKDIVCR_FCK_SHIFT)                   /* /6 */
#  define R_SYSTEM_SCKDIVCR_FCK_DIV_12 (10 << R_SYSTEM_SCKDIVCR_FCK_SHIFT)                 /* /12 */

/* System Clock Division Control Register 2 (8-bits) ************************/

#define R_SYSTEM_SCKDIVCR2_CPUCK_SHIFT (0)
#define R_SYSTEM_SCKDIVCR2_CPUCK_MASK (0xf)
#  define R_SYSTEM_SCKDIVCR2_CPUCK_V1_VALUE_AFTER_RESET (0 << R_SYSTEM_SCKDIVCR2_CPUCK_SHIFT)  /* /1 (value after reset) */
#  define R_SYSTEM_SCKDIVCR2_CPUCK_DIV_2 (1 << R_SYSTEM_SCKDIVCR2_CPUCK_SHIFT)                 /* /2 */
#  define R_SYSTEM_SCKDIVCR2_CPUCK_DIV_4 (2 << R_SYSTEM_SCKDIVCR2_CPUCK_SHIFT)                 /* /4 */
#  define R_SYSTEM_SCKDIVCR2_CPUCK_DIV_8 (3 << R_SYSTEM_SCKDIVCR2_CPUCK_SHIFT)                 /* /8 */
#  define R_SYSTEM_SCKDIVCR2_CPUCK_DIV_16 (4 << R_SYSTEM_SCKDIVCR2_CPUCK_SHIFT)                /* /16 */
#  define R_SYSTEM_SCKDIVCR2_CPUCK_DIV_32 (5 << R_SYSTEM_SCKDIVCR2_CPUCK_SHIFT)                /* /32 */
#  define R_SYSTEM_SCKDIVCR2_CPUCK_DIV_64 (6 << R_SYSTEM_SCKDIVCR2_CPUCK_SHIFT)                /* /64 */
#  define R_SYSTEM_SCKDIVCR2_CPUCK_DIV_3 (8 << R_SYSTEM_SCKDIVCR2_CPUCK_SHIFT)                 /* /3 */
#  define R_SYSTEM_SCKDIVCR2_CPUCK_DIV_6 (9 << R_SYSTEM_SCKDIVCR2_CPUCK_SHIFT)                 /* /6 */
#  define R_SYSTEM_SCKDIVCR2_CPUCK_DIV_12 (10 << R_SYSTEM_SCKDIVCR2_CPUCK_SHIFT)               /* /12 */

/* System Clock Source Control Register (8-bits) ****************************/

#define R_SYSTEM_SCKSCR_CKSEL_SHIFT (0)
#define R_SYSTEM_SCKSCR_CKSEL_MASK (0x7)
#  define R_SYSTEM_SCKSCR_CKSEL_HOCO (0 << R_SYSTEM_SCKSCR_CKSEL_SHIFT)                   /* HOCO */
#  define R_SYSTEM_SCKSCR_CKSEL_MOCO (1 << R_SYSTEM_SCKSCR_CKSEL_SHIFT)                   /* MOCO */
#  define R_SYSTEM_SCKSCR_CKSEL_MAIN_CLOCK_OSCILLATOR (3 << R_SYSTEM_SCKSCR_CKSEL_SHIFT)  /* Main clock oscillator */
#  define R_SYSTEM_SCKSCR_CKSEL_SUB_CLOCK_OSCILLATOR (4 << R_SYSTEM_SCKSCR_CKSEL_SHIFT)   /* Sub-clock oscillator */
#  define R_SYSTEM_SCKSCR_CKSEL_PLL (5 << R_SYSTEM_SCKSCR_CKSEL_SHIFT)                    /* PLL */

/* PLL Clock Control Register (16-bits) *************************************/

#define R_SYSTEM_PLLCCR_PLIDIV_SHIFT (0)
#define R_SYSTEM_PLLCCR_PLIDIV_MASK (0x3)
#  define R_SYSTEM_PLLCCR_PLIDIV_DIV_1 (0 << R_SYSTEM_PLLCCR_PLIDIV_SHIFT)                        /* /1 */
#  define R_SYSTEM_PLLCCR_PLIDIV_DIV_2 (1 << R_SYSTEM_PLLCCR_PLIDIV_SHIFT)                        /* /2 */
#  define R_SYSTEM_PLLCCR_PLIDIV_DIV_3 (2 << R_SYSTEM_PLLCCR_PLIDIV_SHIFT)                        /* /3 */
#  define R_SYSTEM_PLLCCR_PLIDIV_SETTING_PROHIBITED (3 << R_SYSTEM_PLLCCR_PLIDIV_SHIFT)           /* Setting prohibited */
#define R_SYSTEM_PLLCCR_PLSRCSEL (1 <<  4)                                                        /* 10: PLL1 Clock Source Select */
#define R_SYSTEM_PLLCCR_PLLMULNF_SHIFT (6)
#define R_SYSTEM_PLLCCR_PLLMULNF_MASK (0x3)
#  define R_SYSTEM_PLLCCR_PLLMULNF_V0_00_VALUE_AFTER_RESET (0 << R_SYSTEM_PLLCCR_PLLMULNF_SHIFT)  /* 0.00 (Value after reset) */
#  define R_SYSTEM_PLLCCR_PLLMULNF_V0_33_1_3 (1 << R_SYSTEM_PLLCCR_PLLMULNF_SHIFT)                /* 0.33 (1/3) */
#  define R_SYSTEM_PLLCCR_PLLMULNF_V0_66_2_3 (2 << R_SYSTEM_PLLCCR_PLLMULNF_SHIFT)                /* 0.66 (2/3) */
#  define R_SYSTEM_PLLCCR_PLLMULNF_V0_50_1_2 (3 << R_SYSTEM_PLLCCR_PLLMULNF_SHIFT)                /* 0.50 (1/2) */
#define R_SYSTEM_PLLCCR_PLLMUL_SHIFT (8)
#define R_SYSTEM_PLLCCR_PLLMUL_MASK (0xff)
#  define R_SYSTEM_PLLCCR_PLLMUL_V26_VALUE_AFTER_RESET (25 << R_SYSTEM_PLLCCR_PLLMUL_SHIFT)       /* ×26 (Value after reset) */
#  define R_SYSTEM_PLLCCR_PLLMUL_V27 (26 << R_SYSTEM_PLLCCR_PLLMUL_SHIFT)                         /* ×27 */
#  define R_SYSTEM_PLLCCR_PLLMUL_V28 (27 << R_SYSTEM_PLLCCR_PLLMUL_SHIFT)                         /* ×28 */
#  define R_SYSTEM_PLLCCR_PLLMUL_V89 (88 << R_SYSTEM_PLLCCR_PLLMUL_SHIFT)                         /* ×89 */
#  define R_SYSTEM_PLLCCR_PLLMUL_V90 (89 << R_SYSTEM_PLLCCR_PLLMUL_SHIFT)                         /* ×90 */
#  define R_SYSTEM_PLLCCR_PLLMUL_V91 (90 << R_SYSTEM_PLLCCR_PLLMUL_SHIFT)                         /* ×91 */
#  define R_SYSTEM_PLLCCR_PLLMUL_V179 (178 << R_SYSTEM_PLLCCR_PLLMUL_SHIFT)                       /* ×179 */
#  define R_SYSTEM_PLLCCR_PLLMUL_V180 (179 << R_SYSTEM_PLLCCR_PLLMUL_SHIFT)                       /* ×180 */

/* PLL Control Register (8-bits) ********************************************/

#define R_SYSTEM_PLLCR_PLLSTP (1 <<  0)  /* 01: PLL1 Stop Control */

/* External Bus Clock Control Register (8-bits) *****************************/

#define R_SYSTEM_BCKCR_BCLKDIV (1 <<  0)  /* 01: BCLK Pin Output Select */

/* Main Clock Oscillator Control Register (8-bits) **************************/

#define R_SYSTEM_MOSCCR_MOSTP (1 <<  0)  /* 01: Main Clock Oscillator Stop */

/* High-Speed On-Chip Oscillator Control Register (8-bits) ******************/

#define R_SYSTEM_HOCOCR_HCSTP (1 <<  0)  /* 01: HOCO Stop */

/* Middle-Speed On-Chip Oscillator Control Register (8-bits) ****************/

#define R_SYSTEM_MOCOCR_MCSTP (1 <<  0)  /* 01: MOCO Stop */

/* FLL Control Register 1 (8-bits) ******************************************/

#define R_SYSTEM_FLLCR1_FLLEN (1 <<  0)  /* 01: FLL Enable */

/* FLL Control Register 2 (16-bits) *****************************************/

#define R_SYSTEM_FLLCR2_FLLCNTL_SHIFT (0)
#define R_SYSTEM_FLLCR2_FLLCNTL_MASK (0x7ff)

/* Oscillation Stabilization Flag Register (8-bits) *************************/

#define R_SYSTEM_OSCSF_HOCOSF (1 <<  0)  /* 01: HOCO Clock Oscillation Stabilization FlagNOTE: The HOCOSF bit value after a reset is 1 when the OFS1.HOCOEN bit is 0. It is 0 when the OFS1.HOCOEN bit is 1. */
#define R_SYSTEM_OSCSF_MOSCSF (1 <<  3)  /* 08: Main Clock Oscillation Stabilization Flag */
#define R_SYSTEM_OSCSF_PLLSF (1 <<  5)   /* 20: PLL1 Clock Oscillation Stabilization Flag */
#define R_SYSTEM_OSCSF_PLL2SF (1 <<  6)  /* 40: PLL2 Clock Oscillation Stabilization Flag */

/* Clock Out Control Register (8-bits) **************************************/

#define R_SYSTEM_CKOCR_CKODIV_SHIFT (4)
#define R_SYSTEM_CKOCR_CKODIV_MASK (0x7)
#  define R_SYSTEM_CKOCR_CKODIV_DIV_1 (0 << R_SYSTEM_CKOCR_CKODIV_SHIFT)    /* /1 */
#  define R_SYSTEM_CKOCR_CKODIV_DIV_2 (1 << R_SYSTEM_CKOCR_CKODIV_SHIFT)    /* /2 */
#  define R_SYSTEM_CKOCR_CKODIV_DIV_4 (2 << R_SYSTEM_CKOCR_CKODIV_SHIFT)    /* /4 */
#  define R_SYSTEM_CKOCR_CKODIV_DIV_8 (3 << R_SYSTEM_CKOCR_CKODIV_SHIFT)    /* /8 */
#  define R_SYSTEM_CKOCR_CKODIV_DIV_16 (4 << R_SYSTEM_CKOCR_CKODIV_SHIFT)   /* /16 */
#  define R_SYSTEM_CKOCR_CKODIV_DIV_32 (5 << R_SYSTEM_CKOCR_CKODIV_SHIFT)   /* /32 */
#  define R_SYSTEM_CKOCR_CKODIV_DIV_64 (6 << R_SYSTEM_CKOCR_CKODIV_SHIFT)   /* /64 */
#  define R_SYSTEM_CKOCR_CKODIV_DIV_128 (7 << R_SYSTEM_CKOCR_CKODIV_SHIFT)  /* /128 */
#define R_SYSTEM_CKOCR_CKOEN (1 <<  7)                                      /* 80: Clock out enable */

/* Trace Clock Control Register (8-bits) ************************************/

#define R_SYSTEM_TRCKCR_TRCK_SHIFT (0)
#define R_SYSTEM_TRCKCR_TRCK_MASK (0xf)
#define R_SYSTEM_TRCKCR_TRCKSEL (1 <<  4)  /* 10: Trace Clock source select */
#define R_SYSTEM_TRCKCR_TRCKEN (1 <<  7)   /* 80: Trace Clock operating Enable */

/* Oscillation Stop Detection Control Register (8-bits) *********************/

#define R_SYSTEM_OSTDCR_OSTDIE (1 <<  0)  /* 01: Oscillation Stop Detection Interrupt Enable */
#define R_SYSTEM_OSTDCR_OSTDE (1 <<  7)   /* 80: Oscillation Stop Detection Function Enable */

/* Oscillation Stop Detection Status Register (8-bits) **********************/

#define R_SYSTEM_OSTDSR_OSTDF (1 <<  0)  /* 01: Oscillation Stop Detection Flag */

/* Oscillator Monitor Register (8-bits) *************************************/

#define R_SYSTEM_OSCMONR_MOCOMON (1 <<  1)  /* 02: MOCO operation monitor */
#define R_SYSTEM_OSCMONR_LOCOMON (1 <<  2)  /* 04: LOCO operation monitor */

/* PLL2 Clock Control Register (16-bits) ************************************/

#define R_SYSTEM_PLL2CCR_PL2IDIV_SHIFT (0)
#define R_SYSTEM_PLL2CCR_PL2IDIV_MASK (0x3)
#  define R_SYSTEM_PLL2CCR_PL2IDIV_V1_VALUE_AFTER_RESET (0 << R_SYSTEM_PLL2CCR_PL2IDIV_SHIFT)         /* /1 (Value after reset) */
#  define R_SYSTEM_PLL2CCR_PL2IDIV_DIV_2 (1 << R_SYSTEM_PLL2CCR_PL2IDIV_SHIFT)                        /* /2 */
#  define R_SYSTEM_PLL2CCR_PL2IDIV_DIV_3 (2 << R_SYSTEM_PLL2CCR_PL2IDIV_SHIFT)                        /* /3 */
#  define R_SYSTEM_PLL2CCR_PL2IDIV_DIV_4 (3 << R_SYSTEM_PLL2CCR_PL2IDIV_SHIFT)                        /* /4 */
#define R_SYSTEM_PLL2CCR_PL2SRCSEL (1 <<  4)                                                          /* 10: PLL Clock Source Select */
#define R_SYSTEM_PLL2CCR_PLL2MULNF_SHIFT (6)
#define R_SYSTEM_PLL2CCR_PLL2MULNF_MASK (0x3)
#  define R_SYSTEM_PLL2CCR_PLL2MULNF_V0_00_VALUE_AFTER_RESET (0 << R_SYSTEM_PLL2CCR_PLL2MULNF_SHIFT)  /* 0.00 (Value after reset) */
#  define R_SYSTEM_PLL2CCR_PLL2MULNF_V0_33_1_3 (1 << R_SYSTEM_PLL2CCR_PLL2MULNF_SHIFT)                /* 0.33 (1/3) */
#  define R_SYSTEM_PLL2CCR_PLL2MULNF_V0_66_2_3 (2 << R_SYSTEM_PLL2CCR_PLL2MULNF_SHIFT)                /* 0.66 (2/3) */
#  define R_SYSTEM_PLL2CCR_PLL2MULNF_V0_50_1_2 (3 << R_SYSTEM_PLL2CCR_PLL2MULNF_SHIFT)                /* 0.50 (1/2) */
#define R_SYSTEM_PLL2CCR_PLL2MUL_SHIFT (8)
#define R_SYSTEM_PLL2CCR_PLL2MUL_MASK (0xff)
#  define R_SYSTEM_PLL2CCR_PLL2MUL_V26_VALUE_AFTER_RESET (25 << R_SYSTEM_PLL2CCR_PLL2MUL_SHIFT)       /* ×26 (Value after reset) */
#  define R_SYSTEM_PLL2CCR_PLL2MUL_V27 (26 << R_SYSTEM_PLL2CCR_PLL2MUL_SHIFT)                         /* ×27 */
#  define R_SYSTEM_PLL2CCR_PLL2MUL_V28 (27 << R_SYSTEM_PLL2CCR_PLL2MUL_SHIFT)                         /* ×28 */
#  define R_SYSTEM_PLL2CCR_PLL2MUL_V89 (88 << R_SYSTEM_PLL2CCR_PLL2MUL_SHIFT)                         /* ×89 */
#  define R_SYSTEM_PLL2CCR_PLL2MUL_V90 (89 << R_SYSTEM_PLL2CCR_PLL2MUL_SHIFT)                         /* ×90 */
#  define R_SYSTEM_PLL2CCR_PLL2MUL_V91 (90 << R_SYSTEM_PLL2CCR_PLL2MUL_SHIFT)                         /* ×91 */
#  define R_SYSTEM_PLL2CCR_PLL2MUL_V179 (178 << R_SYSTEM_PLL2CCR_PLL2MUL_SHIFT)                       /* ×179 */
#  define R_SYSTEM_PLL2CCR_PLL2MUL_V180 (179 << R_SYSTEM_PLL2CCR_PLL2MUL_SHIFT)                       /* ×180 */

/* PLL2 Control Register (8-bits) *******************************************/

#define R_SYSTEM_PLL2CR_PLL2STP (1 <<  0)  /* 01: PLL2 Stop Control */

/* PLL Clock Control Register 2 (16-bits) ***********************************/

#define R_SYSTEM_PLLCCR2_PLODIVP_SHIFT (0)
#define R_SYSTEM_PLLCCR2_PLODIVP_MASK (0xf)
#  define R_SYSTEM_PLLCCR2_PLODIVP_V1 (0 << R_SYSTEM_PLLCCR2_PLODIVP_SHIFT)              /* ×1 */
#  define R_SYSTEM_PLLCCR2_PLODIVP_V1_2 (1 << R_SYSTEM_PLLCCR2_PLODIVP_SHIFT)            /* ×1 / 2 */
#  define R_SYSTEM_PLLCCR2_PLODIVP_PROHIBITED_2 (2 << R_SYSTEM_PLLCCR2_PLODIVP_SHIFT)    /* prohibited */
#  define R_SYSTEM_PLLCCR2_PLODIVP_V1_4 (3 << R_SYSTEM_PLLCCR2_PLODIVP_SHIFT)            /* ×1 / 4 */
#  define R_SYSTEM_PLLCCR2_PLODIVP_PROHIBITED_4 (4 << R_SYSTEM_PLLCCR2_PLODIVP_SHIFT)    /* prohibited */
#  define R_SYSTEM_PLLCCR2_PLODIVP_V0101 (5 << R_SYSTEM_PLLCCR2_PLODIVP_SHIFT)           /* ×1 / 6 (Value after reset) */
#  define R_SYSTEM_PLLCCR2_PLODIVP_PROHIBITED_6 (6 << R_SYSTEM_PLLCCR2_PLODIVP_SHIFT)    /* prohibited */
#  define R_SYSTEM_PLLCCR2_PLODIVP_V1_8 (7 << R_SYSTEM_PLLCCR2_PLODIVP_SHIFT)            /* ×1 / 8 */
#  define R_SYSTEM_PLLCCR2_PLODIVP_PROHIBITED_8 (8 << R_SYSTEM_PLLCCR2_PLODIVP_SHIFT)    /* prohibited */
#  define R_SYSTEM_PLLCCR2_PLODIVP_PROHIBITED_9 (9 << R_SYSTEM_PLLCCR2_PLODIVP_SHIFT)    /* prohibited */
#  define R_SYSTEM_PLLCCR2_PLODIVP_PROHIBITED_10 (10 << R_SYSTEM_PLLCCR2_PLODIVP_SHIFT)  /* prohibited */
#  define R_SYSTEM_PLLCCR2_PLODIVP_PROHIBITED_11 (11 << R_SYSTEM_PLLCCR2_PLODIVP_SHIFT)  /* prohibited */
#  define R_SYSTEM_PLLCCR2_PLODIVP_PROHIBITED_12 (12 << R_SYSTEM_PLLCCR2_PLODIVP_SHIFT)  /* prohibited */
#  define R_SYSTEM_PLLCCR2_PLODIVP_PROHIBITED_13 (13 << R_SYSTEM_PLLCCR2_PLODIVP_SHIFT)  /* prohibited */
#  define R_SYSTEM_PLLCCR2_PLODIVP_PROHIBITED_14 (14 << R_SYSTEM_PLLCCR2_PLODIVP_SHIFT)  /* prohibited */
#  define R_SYSTEM_PLLCCR2_PLODIVP_V1_16 (15 << R_SYSTEM_PLLCCR2_PLODIVP_SHIFT)          /* ×1 / 16 */
#define R_SYSTEM_PLLCCR2_PLODIVQ_SHIFT (4)
#define R_SYSTEM_PLLCCR2_PLODIVQ_MASK (0xf)
#  define R_SYSTEM_PLLCCR2_PLODIVQ_PROHIBITED_0 (0 << R_SYSTEM_PLLCCR2_PLODIVQ_SHIFT)    /* prohibited */
#  define R_SYSTEM_PLLCCR2_PLODIVQ_V1_2 (1 << R_SYSTEM_PLLCCR2_PLODIVQ_SHIFT)            /* ×1 / 2 */
#  define R_SYSTEM_PLLCCR2_PLODIVQ_V1_3 (2 << R_SYSTEM_PLLCCR2_PLODIVQ_SHIFT)            /* ×1 / 3 */
#  define R_SYSTEM_PLLCCR2_PLODIVQ_V1_4 (3 << R_SYSTEM_PLLCCR2_PLODIVQ_SHIFT)            /* ×1 / 4 */
#  define R_SYSTEM_PLLCCR2_PLODIVQ_V1_5 (4 << R_SYSTEM_PLLCCR2_PLODIVQ_SHIFT)            /* ×1 / 5 */
#  define R_SYSTEM_PLLCCR2_PLODIVQ_V0101 (5 << R_SYSTEM_PLLCCR2_PLODIVQ_SHIFT)           /* ×1 / 6 (Value after reset) */
#  define R_SYSTEM_PLLCCR2_PLODIVQ_PROHIBITED_6 (6 << R_SYSTEM_PLLCCR2_PLODIVQ_SHIFT)    /* prohibited */
#  define R_SYSTEM_PLLCCR2_PLODIVQ_V1_8 (7 << R_SYSTEM_PLLCCR2_PLODIVQ_SHIFT)            /* ×1 / 8 */
#  define R_SYSTEM_PLLCCR2_PLODIVQ_V1_9 (8 << R_SYSTEM_PLLCCR2_PLODIVQ_SHIFT)            /* ×1 / 9 */
#  define R_SYSTEM_PLLCCR2_PLODIVQ_PROHIBITED_9 (9 << R_SYSTEM_PLLCCR2_PLODIVQ_SHIFT)    /* prohibited */
#  define R_SYSTEM_PLLCCR2_PLODIVQ_PROHIBITED_10 (10 << R_SYSTEM_PLLCCR2_PLODIVQ_SHIFT)  /* prohibited */
#  define R_SYSTEM_PLLCCR2_PLODIVQ_PROHIBITED_11 (11 << R_SYSTEM_PLLCCR2_PLODIVQ_SHIFT)  /* prohibited */
#  define R_SYSTEM_PLLCCR2_PLODIVQ_PROHIBITED_12 (12 << R_SYSTEM_PLLCCR2_PLODIVQ_SHIFT)  /* prohibited */
#  define R_SYSTEM_PLLCCR2_PLODIVQ_PROHIBITED_13 (13 << R_SYSTEM_PLLCCR2_PLODIVQ_SHIFT)  /* prohibited */
#  define R_SYSTEM_PLLCCR2_PLODIVQ_PROHIBITED_14 (14 << R_SYSTEM_PLLCCR2_PLODIVQ_SHIFT)  /* prohibited */
#  define R_SYSTEM_PLLCCR2_PLODIVQ_PROHIBITED_15 (15 << R_SYSTEM_PLLCCR2_PLODIVQ_SHIFT)  /* prohibited */
#define R_SYSTEM_PLLCCR2_PLODIVR_SHIFT (8)
#define R_SYSTEM_PLLCCR2_PLODIVR_MASK (0xf)
#  define R_SYSTEM_PLLCCR2_PLODIVR_PROHIBITED_0 (0 << R_SYSTEM_PLLCCR2_PLODIVR_SHIFT)    /* prohibited */
#  define R_SYSTEM_PLLCCR2_PLODIVR_V1_2 (1 << R_SYSTEM_PLLCCR2_PLODIVR_SHIFT)            /* ×1 / 2 */
#  define R_SYSTEM_PLLCCR2_PLODIVR_V1_3 (2 << R_SYSTEM_PLLCCR2_PLODIVR_SHIFT)            /* ×1 / 3 */
#  define R_SYSTEM_PLLCCR2_PLODIVR_V1_4 (3 << R_SYSTEM_PLLCCR2_PLODIVR_SHIFT)            /* ×1 / 4 */
#  define R_SYSTEM_PLLCCR2_PLODIVR_V1_5 (4 << R_SYSTEM_PLLCCR2_PLODIVR_SHIFT)            /* ×1 / 5 */
#  define R_SYSTEM_PLLCCR2_PLODIVR_V0101 (5 << R_SYSTEM_PLLCCR2_PLODIVR_SHIFT)           /* ×1 / 6 (Value after reset) */
#  define R_SYSTEM_PLLCCR2_PLODIVR_PROHIBITED_6 (6 << R_SYSTEM_PLLCCR2_PLODIVR_SHIFT)    /* prohibited */
#  define R_SYSTEM_PLLCCR2_PLODIVR_V1_8 (7 << R_SYSTEM_PLLCCR2_PLODIVR_SHIFT)            /* ×1 / 8 */
#  define R_SYSTEM_PLLCCR2_PLODIVR_V1_9 (8 << R_SYSTEM_PLLCCR2_PLODIVR_SHIFT)            /* ×1 / 9 */
#  define R_SYSTEM_PLLCCR2_PLODIVR_PROHIBITED_9 (9 << R_SYSTEM_PLLCCR2_PLODIVR_SHIFT)    /* prohibited */
#  define R_SYSTEM_PLLCCR2_PLODIVR_PROHIBITED_10 (10 << R_SYSTEM_PLLCCR2_PLODIVR_SHIFT)  /* prohibited */
#  define R_SYSTEM_PLLCCR2_PLODIVR_PROHIBITED_11 (11 << R_SYSTEM_PLLCCR2_PLODIVR_SHIFT)  /* prohibited */
#  define R_SYSTEM_PLLCCR2_PLODIVR_PROHIBITED_12 (12 << R_SYSTEM_PLLCCR2_PLODIVR_SHIFT)  /* prohibited */
#  define R_SYSTEM_PLLCCR2_PLODIVR_PROHIBITED_13 (13 << R_SYSTEM_PLLCCR2_PLODIVR_SHIFT)  /* prohibited */
#  define R_SYSTEM_PLLCCR2_PLODIVR_PROHIBITED_14 (14 << R_SYSTEM_PLLCCR2_PLODIVR_SHIFT)  /* prohibited */
#  define R_SYSTEM_PLLCCR2_PLODIVR_PROHIBITED_15 (15 << R_SYSTEM_PLLCCR2_PLODIVR_SHIFT)  /* prohibited */

/* PLL2 Clock Control Register 2 (16-bits) **********************************/

#define R_SYSTEM_PLL2CCR2_PL2ODIVP_SHIFT (0)
#define R_SYSTEM_PLL2CCR2_PL2ODIVP_MASK (0xf)
#  define R_SYSTEM_PLL2CCR2_PL2ODIVP_V1 (0 << R_SYSTEM_PLL2CCR2_PL2ODIVP_SHIFT)              /* ×1 */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVP_V1_2 (1 << R_SYSTEM_PLL2CCR2_PL2ODIVP_SHIFT)            /* ×1 / 2 */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVP_PROHIBITED_2 (2 << R_SYSTEM_PLL2CCR2_PL2ODIVP_SHIFT)    /* prohibited */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVP_V1_4 (3 << R_SYSTEM_PLL2CCR2_PL2ODIVP_SHIFT)            /* ×1 / 4 */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVP_PROHIBITED_4 (4 << R_SYSTEM_PLL2CCR2_PL2ODIVP_SHIFT)    /* prohibited */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVP_V0101 (5 << R_SYSTEM_PLL2CCR2_PL2ODIVP_SHIFT)           /* ×1 / 6 (Value after reset) */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVP_PROHIBITED_6 (6 << R_SYSTEM_PLL2CCR2_PL2ODIVP_SHIFT)    /* prohibited */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVP_V1_8 (7 << R_SYSTEM_PLL2CCR2_PL2ODIVP_SHIFT)            /* ×1 / 8 */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVP_PROHIBITED_8 (8 << R_SYSTEM_PLL2CCR2_PL2ODIVP_SHIFT)    /* prohibited */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVP_PROHIBITED_9 (9 << R_SYSTEM_PLL2CCR2_PL2ODIVP_SHIFT)    /* prohibited */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVP_PROHIBITED_10 (10 << R_SYSTEM_PLL2CCR2_PL2ODIVP_SHIFT)  /* prohibited */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVP_PROHIBITED_11 (11 << R_SYSTEM_PLL2CCR2_PL2ODIVP_SHIFT)  /* prohibited */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVP_PROHIBITED_12 (12 << R_SYSTEM_PLL2CCR2_PL2ODIVP_SHIFT)  /* prohibited */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVP_PROHIBITED_13 (13 << R_SYSTEM_PLL2CCR2_PL2ODIVP_SHIFT)  /* prohibited */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVP_PROHIBITED_14 (14 << R_SYSTEM_PLL2CCR2_PL2ODIVP_SHIFT)  /* prohibited */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVP_V1_16 (15 << R_SYSTEM_PLL2CCR2_PL2ODIVP_SHIFT)          /* ×1 / 16 */
#define R_SYSTEM_PLL2CCR2_PL2ODIVQ_SHIFT (4)
#define R_SYSTEM_PLL2CCR2_PL2ODIVQ_MASK (0xf)
#  define R_SYSTEM_PLL2CCR2_PL2ODIVQ_PROHIBITED_0 (0 << R_SYSTEM_PLL2CCR2_PL2ODIVQ_SHIFT)    /* prohibited */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVQ_V1_2 (1 << R_SYSTEM_PLL2CCR2_PL2ODIVQ_SHIFT)            /* ×1 / 2 */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVQ_V1_3 (2 << R_SYSTEM_PLL2CCR2_PL2ODIVQ_SHIFT)            /* ×1 / 3 */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVQ_V1_4 (3 << R_SYSTEM_PLL2CCR2_PL2ODIVQ_SHIFT)            /* ×1 / 4 */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVQ_V1_5 (4 << R_SYSTEM_PLL2CCR2_PL2ODIVQ_SHIFT)            /* ×1 / 5 */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVQ_V0101 (5 << R_SYSTEM_PLL2CCR2_PL2ODIVQ_SHIFT)           /* ×1 / 6 (Value after reset) */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVQ_PROHIBITED_6 (6 << R_SYSTEM_PLL2CCR2_PL2ODIVQ_SHIFT)    /* prohibited */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVQ_V1_8 (7 << R_SYSTEM_PLL2CCR2_PL2ODIVQ_SHIFT)            /* ×1 / 8 */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVQ_V1_9 (8 << R_SYSTEM_PLL2CCR2_PL2ODIVQ_SHIFT)            /* ×1 / 9 */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVQ_PROHIBITED_9 (9 << R_SYSTEM_PLL2CCR2_PL2ODIVQ_SHIFT)    /* prohibited */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVQ_PROHIBITED_10 (10 << R_SYSTEM_PLL2CCR2_PL2ODIVQ_SHIFT)  /* prohibited */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVQ_PROHIBITED_11 (11 << R_SYSTEM_PLL2CCR2_PL2ODIVQ_SHIFT)  /* prohibited */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVQ_PROHIBITED_12 (12 << R_SYSTEM_PLL2CCR2_PL2ODIVQ_SHIFT)  /* prohibited */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVQ_PROHIBITED_13 (13 << R_SYSTEM_PLL2CCR2_PL2ODIVQ_SHIFT)  /* prohibited */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVQ_PROHIBITED_14 (14 << R_SYSTEM_PLL2CCR2_PL2ODIVQ_SHIFT)  /* prohibited */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVQ_PROHIBITED_15 (15 << R_SYSTEM_PLL2CCR2_PL2ODIVQ_SHIFT)  /* prohibited */
#define R_SYSTEM_PLL2CCR2_PL2ODIVR_SHIFT (8)
#define R_SYSTEM_PLL2CCR2_PL2ODIVR_MASK (0xf)
#  define R_SYSTEM_PLL2CCR2_PL2ODIVR_PROHIBITED_0 (0 << R_SYSTEM_PLL2CCR2_PL2ODIVR_SHIFT)    /* prohibited */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVR_V1_2 (1 << R_SYSTEM_PLL2CCR2_PL2ODIVR_SHIFT)            /* ×1 / 2 */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVR_V1_3 (2 << R_SYSTEM_PLL2CCR2_PL2ODIVR_SHIFT)            /* ×1 / 3 */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVR_V1_4 (3 << R_SYSTEM_PLL2CCR2_PL2ODIVR_SHIFT)            /* ×1 / 4 */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVR_V1_5 (4 << R_SYSTEM_PLL2CCR2_PL2ODIVR_SHIFT)            /* ×1 / 5 */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVR_V0101 (5 << R_SYSTEM_PLL2CCR2_PL2ODIVR_SHIFT)           /* ×1 / 6 (Value after reset) */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVR_PROHIBITED_6 (6 << R_SYSTEM_PLL2CCR2_PL2ODIVR_SHIFT)    /* prohibited */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVR_V1_8 (7 << R_SYSTEM_PLL2CCR2_PL2ODIVR_SHIFT)            /* ×1 / 8 */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVR_V1_9 (8 << R_SYSTEM_PLL2CCR2_PL2ODIVR_SHIFT)            /* ×1 / 9 */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVR_PROHIBITED_9 (9 << R_SYSTEM_PLL2CCR2_PL2ODIVR_SHIFT)    /* prohibited */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVR_PROHIBITED_10 (10 << R_SYSTEM_PLL2CCR2_PL2ODIVR_SHIFT)  /* prohibited */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVR_PROHIBITED_11 (11 << R_SYSTEM_PLL2CCR2_PL2ODIVR_SHIFT)  /* prohibited */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVR_PROHIBITED_12 (12 << R_SYSTEM_PLL2CCR2_PL2ODIVR_SHIFT)  /* prohibited */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVR_PROHIBITED_13 (13 << R_SYSTEM_PLL2CCR2_PL2ODIVR_SHIFT)  /* prohibited */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVR_PROHIBITED_14 (14 << R_SYSTEM_PLL2CCR2_PL2ODIVR_SHIFT)  /* prohibited */
#  define R_SYSTEM_PLL2CCR2_PL2ODIVR_PROHIBITED_15 (15 << R_SYSTEM_PLL2CCR2_PL2ODIVR_SHIFT)  /* prohibited */

/* External Bus Clock Output Control Register (8-bits) **********************/

#define R_SYSTEM_EBCKOCR_EBCKOEN (1 <<  0)  /* 01: BCLK Pin Output Control */

/* SDRAM Clock Output Control Register (8-bits) *****************************/

#define R_SYSTEM_SDCKOCR_SDCKOEN (1 <<  0)  /* 01: SDCLK Pin Output Control */

/* SCI clock Division control register (8-bits) *****************************/

#define R_SYSTEM_SCICKDIVCR_CKDIV_SHIFT (0)
#define R_SYSTEM_SCICKDIVCR_CKDIV_MASK (0x7)
#  define R_SYSTEM_SCICKDIVCR_CKDIV_V1_VALUE_AFTER_RESET (0 << R_SYSTEM_SCICKDIVCR_CKDIV_SHIFT)  /* /1 (value after reset) */
#  define R_SYSTEM_SCICKDIVCR_CKDIV_DIV_2 (1 << R_SYSTEM_SCICKDIVCR_CKDIV_SHIFT)                 /* /2 */
#  define R_SYSTEM_SCICKDIVCR_CKDIV_DIV_4 (2 << R_SYSTEM_SCICKDIVCR_CKDIV_SHIFT)                 /* /4 */
#  define R_SYSTEM_SCICKDIVCR_CKDIV_DIV_6 (3 << R_SYSTEM_SCICKDIVCR_CKDIV_SHIFT)                 /* /6 */
#  define R_SYSTEM_SCICKDIVCR_CKDIV_DIV_8 (4 << R_SYSTEM_SCICKDIVCR_CKDIV_SHIFT)                 /* /8 */
#  define R_SYSTEM_SCICKDIVCR_CKDIV_DIV_3 (5 << R_SYSTEM_SCICKDIVCR_CKDIV_SHIFT)                 /* /3 */
#  define R_SYSTEM_SCICKDIVCR_CKDIV_DIV_5 (6 << R_SYSTEM_SCICKDIVCR_CKDIV_SHIFT)                 /* /5 */

/* SCI clock control register (8-bits) **************************************/

#define R_SYSTEM_SCICKCR_CKSEL_SHIFT (0)
#define R_SYSTEM_SCICKCR_CKSEL_MASK (0xf)
#  define R_SYSTEM_SCICKCR_CKSEL_HOCO (0 << R_SYSTEM_SCICKCR_CKSEL_SHIFT)                    /* HOCO */
#  define R_SYSTEM_SCICKCR_CKSEL_MOCO_VALUE_AFTER_RESET (1 << R_SYSTEM_SCICKCR_CKSEL_SHIFT)  /* MOCO (value after reset) */
#  define R_SYSTEM_SCICKCR_CKSEL_LOCO (2 << R_SYSTEM_SCICKCR_CKSEL_SHIFT)                    /* LOCO */
#  define R_SYSTEM_SCICKCR_CKSEL_MAIN_CLOCK_OSCILLATOR (3 << R_SYSTEM_SCICKCR_CKSEL_SHIFT)   /* Main clock oscillator */
#  define R_SYSTEM_SCICKCR_CKSEL_SUB_CLOCK_OSCILLATOR (4 << R_SYSTEM_SCICKCR_CKSEL_SHIFT)    /* Sub-clock oscillator */
#  define R_SYSTEM_SCICKCR_CKSEL_PLL1P (5 << R_SYSTEM_SCICKCR_CKSEL_SHIFT)                   /* PLL1P */
#  define R_SYSTEM_SCICKCR_CKSEL_PLL2P (6 << R_SYSTEM_SCICKCR_CKSEL_SHIFT)                   /* PLL2P */
#  define R_SYSTEM_SCICKCR_CKSEL_PLL1Q (7 << R_SYSTEM_SCICKCR_CKSEL_SHIFT)                   /* PLL1Q */
#  define R_SYSTEM_SCICKCR_CKSEL_PLL1R (8 << R_SYSTEM_SCICKCR_CKSEL_SHIFT)                   /* PLL1R */
#  define R_SYSTEM_SCICKCR_CKSEL_PLL2Q (9 << R_SYSTEM_SCICKCR_CKSEL_SHIFT)                   /* PLL2Q */
#  define R_SYSTEM_SCICKCR_CKSEL_PLL2R (10 << R_SYSTEM_SCICKCR_CKSEL_SHIFT)                  /* PLL2R */
#define R_SYSTEM_SCICKCR_CKSREQ (1 <<  6)                                                    /* 40: Clock Switching Request */
#define R_SYSTEM_SCICKCR_CKSRDY (1 <<  7)                                                    /* 80: Clock Switching Ready state flag */

/* SPI clock Division control register (8-bits) *****************************/

#define R_SYSTEM_SPICKDIVCR_CKDIV_SHIFT (0)
#define R_SYSTEM_SPICKDIVCR_CKDIV_MASK (0x7)
#  define R_SYSTEM_SPICKDIVCR_CKDIV_V1_VALUE_AFTER_RESET (0 << R_SYSTEM_SPICKDIVCR_CKDIV_SHIFT)  /* /1 (value after reset) */
#  define R_SYSTEM_SPICKDIVCR_CKDIV_DIV_2 (1 << R_SYSTEM_SPICKDIVCR_CKDIV_SHIFT)                 /* /2 */
#  define R_SYSTEM_SPICKDIVCR_CKDIV_DIV_4 (2 << R_SYSTEM_SPICKDIVCR_CKDIV_SHIFT)                 /* /4 */
#  define R_SYSTEM_SPICKDIVCR_CKDIV_DIV_6 (3 << R_SYSTEM_SPICKDIVCR_CKDIV_SHIFT)                 /* /6 */
#  define R_SYSTEM_SPICKDIVCR_CKDIV_DIV_8 (4 << R_SYSTEM_SPICKDIVCR_CKDIV_SHIFT)                 /* /8 */
#  define R_SYSTEM_SPICKDIVCR_CKDIV_DIV_3 (5 << R_SYSTEM_SPICKDIVCR_CKDIV_SHIFT)                 /* /3 */
#  define R_SYSTEM_SPICKDIVCR_CKDIV_DIV_5 (6 << R_SYSTEM_SPICKDIVCR_CKDIV_SHIFT)                 /* /5 */

/* SPI clock control register (8-bits) **************************************/

#define R_SYSTEM_SPICKCR_CKSEL_SHIFT (0)
#define R_SYSTEM_SPICKCR_CKSEL_MASK (0xf)
#  define R_SYSTEM_SPICKCR_CKSEL_HOCO (0 << R_SYSTEM_SPICKCR_CKSEL_SHIFT)                    /* HOCO */
#  define R_SYSTEM_SPICKCR_CKSEL_MOCO_VALUE_AFTER_RESET (1 << R_SYSTEM_SPICKCR_CKSEL_SHIFT)  /* MOCO (value after reset) */
#  define R_SYSTEM_SPICKCR_CKSEL_LOCO (2 << R_SYSTEM_SPICKCR_CKSEL_SHIFT)                    /* LOCO */
#  define R_SYSTEM_SPICKCR_CKSEL_MAIN_CLOCK_OSCILLATOR (3 << R_SYSTEM_SPICKCR_CKSEL_SHIFT)   /* Main clock oscillator */
#  define R_SYSTEM_SPICKCR_CKSEL_SUB_CLOCK_OSCILLATOR (4 << R_SYSTEM_SPICKCR_CKSEL_SHIFT)    /* Sub-clock oscillator */
#  define R_SYSTEM_SPICKCR_CKSEL_PLL1P (5 << R_SYSTEM_SPICKCR_CKSEL_SHIFT)                   /* PLL1P */
#  define R_SYSTEM_SPICKCR_CKSEL_PLL2P (6 << R_SYSTEM_SPICKCR_CKSEL_SHIFT)                   /* PLL2P */
#  define R_SYSTEM_SPICKCR_CKSEL_PLL1Q (7 << R_SYSTEM_SPICKCR_CKSEL_SHIFT)                   /* PLL1Q */
#  define R_SYSTEM_SPICKCR_CKSEL_PLL1R (8 << R_SYSTEM_SPICKCR_CKSEL_SHIFT)                   /* PLL1R */
#  define R_SYSTEM_SPICKCR_CKSEL_PLL2Q (9 << R_SYSTEM_SPICKCR_CKSEL_SHIFT)                   /* PLL2Q */
#  define R_SYSTEM_SPICKCR_CKSEL_PLL2R (10 << R_SYSTEM_SPICKCR_CKSEL_SHIFT)                  /* PLL2R */
#define R_SYSTEM_SPICKCR_CKSREQ (1 <<  6)                                                    /* 40: Clock Switching Request */
#define R_SYSTEM_SPICKCR_CKSRDY (1 <<  7)                                                    /* 80: Clock Switching Ready state flag */

/* ADC clock Division control register (8-bits) *****************************/

#define R_SYSTEM_ADCCKDIVCR_CKDIV_SHIFT (0)
#define R_SYSTEM_ADCCKDIVCR_CKDIV_MASK (0x7)
#  define R_SYSTEM_ADCCKDIVCR_CKDIV_V1_VALUE_AFTER_RESET (0 << R_SYSTEM_ADCCKDIVCR_CKDIV_SHIFT)  /* /1 (value after reset) */
#  define R_SYSTEM_ADCCKDIVCR_CKDIV_DIV_2 (1 << R_SYSTEM_ADCCKDIVCR_CKDIV_SHIFT)                 /* /2 */
#  define R_SYSTEM_ADCCKDIVCR_CKDIV_DIV_4 (2 << R_SYSTEM_ADCCKDIVCR_CKDIV_SHIFT)                 /* /4 */
#  define R_SYSTEM_ADCCKDIVCR_CKDIV_DIV_6 (3 << R_SYSTEM_ADCCKDIVCR_CKDIV_SHIFT)                 /* /6 */
#  define R_SYSTEM_ADCCKDIVCR_CKDIV_DIV_8 (4 << R_SYSTEM_ADCCKDIVCR_CKDIV_SHIFT)                 /* /8 */
#  define R_SYSTEM_ADCCKDIVCR_CKDIV_DIV_3 (5 << R_SYSTEM_ADCCKDIVCR_CKDIV_SHIFT)                 /* /3 */
#  define R_SYSTEM_ADCCKDIVCR_CKDIV_DIV_5 (6 << R_SYSTEM_ADCCKDIVCR_CKDIV_SHIFT)                 /* /5 */

/* ADC clock control register (8-bits) **************************************/

#define R_SYSTEM_ADCCKCR_CKSEL_SHIFT (0)
#define R_SYSTEM_ADCCKCR_CKSEL_MASK (0xf)
#  define R_SYSTEM_ADCCKCR_CKSEL_HOCO (0 << R_SYSTEM_ADCCKCR_CKSEL_SHIFT)                    /* HOCO */
#  define R_SYSTEM_ADCCKCR_CKSEL_MOCO_VALUE_AFTER_RESET (1 << R_SYSTEM_ADCCKCR_CKSEL_SHIFT)  /* MOCO (value after reset) */
#  define R_SYSTEM_ADCCKCR_CKSEL_LOCO (2 << R_SYSTEM_ADCCKCR_CKSEL_SHIFT)                    /* LOCO */
#  define R_SYSTEM_ADCCKCR_CKSEL_MAIN_CLOCK_OSCILLATOR (3 << R_SYSTEM_ADCCKCR_CKSEL_SHIFT)   /* Main clock oscillator */
#  define R_SYSTEM_ADCCKCR_CKSEL_SUB_CLOCK_OSCILLATOR (4 << R_SYSTEM_ADCCKCR_CKSEL_SHIFT)    /* Sub-clock oscillator */
#  define R_SYSTEM_ADCCKCR_CKSEL_PLL1P (5 << R_SYSTEM_ADCCKCR_CKSEL_SHIFT)                   /* PLL1P */
#  define R_SYSTEM_ADCCKCR_CKSEL_PLL2P (6 << R_SYSTEM_ADCCKCR_CKSEL_SHIFT)                   /* PLL2P */
#  define R_SYSTEM_ADCCKCR_CKSEL_PLL1Q (7 << R_SYSTEM_ADCCKCR_CKSEL_SHIFT)                   /* PLL1Q */
#  define R_SYSTEM_ADCCKCR_CKSEL_PLL1R (8 << R_SYSTEM_ADCCKCR_CKSEL_SHIFT)                   /* PLL1R */
#  define R_SYSTEM_ADCCKCR_CKSEL_PLL2Q (9 << R_SYSTEM_ADCCKCR_CKSEL_SHIFT)                   /* PLL2Q */
#  define R_SYSTEM_ADCCKCR_CKSEL_PLL2R (10 << R_SYSTEM_ADCCKCR_CKSEL_SHIFT)                  /* PLL2R */
#define R_SYSTEM_ADCCKCR_CKSREQ (1 <<  6)                                                    /* 40: Clock Switching Request */
#define R_SYSTEM_ADCCKCR_CKSRDY (1 <<  7)                                                    /* 80: Clock Switching Ready state flag */

/* GPT clock Division control register (8-bits) *****************************/

#define R_SYSTEM_GPTCKDIVCR_CKDIV_SHIFT (0)
#define R_SYSTEM_GPTCKDIVCR_CKDIV_MASK (0x7)
#  define R_SYSTEM_GPTCKDIVCR_CKDIV_V1_VALUE_AFTER_RESET (0 << R_SYSTEM_GPTCKDIVCR_CKDIV_SHIFT)  /* /1 (value after reset) */
#  define R_SYSTEM_GPTCKDIVCR_CKDIV_DIV_2 (1 << R_SYSTEM_GPTCKDIVCR_CKDIV_SHIFT)                 /* /2 */
#  define R_SYSTEM_GPTCKDIVCR_CKDIV_DIV_4 (2 << R_SYSTEM_GPTCKDIVCR_CKDIV_SHIFT)                 /* /4 */
#  define R_SYSTEM_GPTCKDIVCR_CKDIV_DIV_6 (3 << R_SYSTEM_GPTCKDIVCR_CKDIV_SHIFT)                 /* /6 */
#  define R_SYSTEM_GPTCKDIVCR_CKDIV_DIV_8 (4 << R_SYSTEM_GPTCKDIVCR_CKDIV_SHIFT)                 /* /8 */
#  define R_SYSTEM_GPTCKDIVCR_CKDIV_DIV_3 (5 << R_SYSTEM_GPTCKDIVCR_CKDIV_SHIFT)                 /* /3 */
#  define R_SYSTEM_GPTCKDIVCR_CKDIV_DIV_5 (6 << R_SYSTEM_GPTCKDIVCR_CKDIV_SHIFT)                 /* /5 */

/* GPT clock control register (8-bits) **************************************/

#define R_SYSTEM_GPTCKCR_CKSEL_SHIFT (0)
#define R_SYSTEM_GPTCKCR_CKSEL_MASK (0xf)
#  define R_SYSTEM_GPTCKCR_CKSEL_HOCO (0 << R_SYSTEM_GPTCKCR_CKSEL_SHIFT)                    /* HOCO */
#  define R_SYSTEM_GPTCKCR_CKSEL_MOCO_VALUE_AFTER_RESET (1 << R_SYSTEM_GPTCKCR_CKSEL_SHIFT)  /* MOCO (value after reset) */
#  define R_SYSTEM_GPTCKCR_CKSEL_LOCO (2 << R_SYSTEM_GPTCKCR_CKSEL_SHIFT)                    /* LOCO */
#  define R_SYSTEM_GPTCKCR_CKSEL_MAIN_CLOCK_OSCILLATOR (3 << R_SYSTEM_GPTCKCR_CKSEL_SHIFT)   /* Main clock oscillator */
#  define R_SYSTEM_GPTCKCR_CKSEL_SUB_CLOCK_OSCILLATOR (4 << R_SYSTEM_GPTCKCR_CKSEL_SHIFT)    /* Sub-clock oscillator */
#  define R_SYSTEM_GPTCKCR_CKSEL_PLL1P (5 << R_SYSTEM_GPTCKCR_CKSEL_SHIFT)                   /* PLL1P */
#  define R_SYSTEM_GPTCKCR_CKSEL_PLL2P (6 << R_SYSTEM_GPTCKCR_CKSEL_SHIFT)                   /* PLL2P */
#  define R_SYSTEM_GPTCKCR_CKSEL_PLL1Q (7 << R_SYSTEM_GPTCKCR_CKSEL_SHIFT)                   /* PLL1Q */
#  define R_SYSTEM_GPTCKCR_CKSEL_PLL1R (8 << R_SYSTEM_GPTCKCR_CKSEL_SHIFT)                   /* PLL1R */
#  define R_SYSTEM_GPTCKCR_CKSEL_PLL2Q (9 << R_SYSTEM_GPTCKCR_CKSEL_SHIFT)                   /* PLL2Q */
#  define R_SYSTEM_GPTCKCR_CKSEL_PLL2R (10 << R_SYSTEM_GPTCKCR_CKSEL_SHIFT)                  /* PLL2R */
#define R_SYSTEM_GPTCKCR_CKSREQ (1 <<  6)                                                    /* 40: Clock Switching Request */
#define R_SYSTEM_GPTCKCR_CKSRDY (1 <<  7)                                                    /* 80: Clock Switching Ready state flag */

/* LCD clock Division control register (8-bits) *****************************/

#define R_SYSTEM_LCDCKDIVCR_CKDIV_SHIFT (0)
#define R_SYSTEM_LCDCKDIVCR_CKDIV_MASK (0x7)
#  define R_SYSTEM_LCDCKDIVCR_CKDIV_V1_VALUE_AFTER_RESET (0 << R_SYSTEM_LCDCKDIVCR_CKDIV_SHIFT)  /* /1 (value after reset) */
#  define R_SYSTEM_LCDCKDIVCR_CKDIV_DIV_2 (1 << R_SYSTEM_LCDCKDIVCR_CKDIV_SHIFT)                 /* /2 */
#  define R_SYSTEM_LCDCKDIVCR_CKDIV_DIV_4 (2 << R_SYSTEM_LCDCKDIVCR_CKDIV_SHIFT)                 /* /4 */
#  define R_SYSTEM_LCDCKDIVCR_CKDIV_DIV_6 (3 << R_SYSTEM_LCDCKDIVCR_CKDIV_SHIFT)                 /* /6 */
#  define R_SYSTEM_LCDCKDIVCR_CKDIV_DIV_8 (4 << R_SYSTEM_LCDCKDIVCR_CKDIV_SHIFT)                 /* /8 */
#  define R_SYSTEM_LCDCKDIVCR_CKDIV_DIV_3 (5 << R_SYSTEM_LCDCKDIVCR_CKDIV_SHIFT)                 /* /3 */
#  define R_SYSTEM_LCDCKDIVCR_CKDIV_DIV_5 (6 << R_SYSTEM_LCDCKDIVCR_CKDIV_SHIFT)                 /* /5 */

/* LCD clock control register (8-bits) **************************************/

#define R_SYSTEM_LCDCKCR_CKSEL_SHIFT (0)
#define R_SYSTEM_LCDCKCR_CKSEL_MASK (0xf)
#  define R_SYSTEM_LCDCKCR_CKSEL_HOCO (0 << R_SYSTEM_LCDCKCR_CKSEL_SHIFT)                    /* HOCO */
#  define R_SYSTEM_LCDCKCR_CKSEL_MOCO_VALUE_AFTER_RESET (1 << R_SYSTEM_LCDCKCR_CKSEL_SHIFT)  /* MOCO (value after reset) */
#  define R_SYSTEM_LCDCKCR_CKSEL_LOCO (2 << R_SYSTEM_LCDCKCR_CKSEL_SHIFT)                    /* LOCO */
#  define R_SYSTEM_LCDCKCR_CKSEL_MAIN_CLOCK_OSCILLATOR (3 << R_SYSTEM_LCDCKCR_CKSEL_SHIFT)   /* Main clock oscillator */
#  define R_SYSTEM_LCDCKCR_CKSEL_SUB_CLOCK_OSCILLATOR (4 << R_SYSTEM_LCDCKCR_CKSEL_SHIFT)    /* Sub-clock oscillator */
#  define R_SYSTEM_LCDCKCR_CKSEL_PLL1P (5 << R_SYSTEM_LCDCKCR_CKSEL_SHIFT)                   /* PLL1P */
#  define R_SYSTEM_LCDCKCR_CKSEL_PLL2P (6 << R_SYSTEM_LCDCKCR_CKSEL_SHIFT)                   /* PLL2P */
#  define R_SYSTEM_LCDCKCR_CKSEL_PLL1Q (7 << R_SYSTEM_LCDCKCR_CKSEL_SHIFT)                   /* PLL1Q */
#  define R_SYSTEM_LCDCKCR_CKSEL_PLL1R (8 << R_SYSTEM_LCDCKCR_CKSEL_SHIFT)                   /* PLL1R */
#  define R_SYSTEM_LCDCKCR_CKSEL_PLL2Q (9 << R_SYSTEM_LCDCKCR_CKSEL_SHIFT)                   /* PLL2Q */
#  define R_SYSTEM_LCDCKCR_CKSEL_PLL2R (10 << R_SYSTEM_LCDCKCR_CKSEL_SHIFT)                  /* PLL2R */
#define R_SYSTEM_LCDCKCR_CKSREQ (1 <<  6)                                                    /* 40: Clock Switching Request */
#define R_SYSTEM_LCDCKCR_CKSRDY (1 <<  7)                                                    /* 80: Clock Switching Ready state flag */

/* MOCO User Trimming Control Register (8-bits) *****************************/

#define R_SYSTEM_MOCOUTCR_MOCOUTRM_SHIFT (0)
#define R_SYSTEM_MOCOUTCR_MOCOUTRM_MASK (0xff)
#  define R_SYSTEM_MOCOUTCR_MOCOUTRM_M128 (128 << R_SYSTEM_MOCOUTCR_MOCOUTRM_SHIFT)       /* -128 */
#  define R_SYSTEM_MOCOUTCR_MOCOUTRM_M127 (129 << R_SYSTEM_MOCOUTCR_MOCOUTRM_SHIFT)       /* -127 */
#  define R_SYSTEM_MOCOUTCR_MOCOUTRM_M126 (130 << R_SYSTEM_MOCOUTCR_MOCOUTRM_SHIFT)       /* -126 */
#  define R_SYSTEM_MOCOUTCR_MOCOUTRM_M1 (255 << R_SYSTEM_MOCOUTCR_MOCOUTRM_SHIFT)         /* -1 */
#  define R_SYSTEM_MOCOUTCR_MOCOUTRM_CENTER_CODE (0 << R_SYSTEM_MOCOUTCR_MOCOUTRM_SHIFT)  /* Center Code */
#  define R_SYSTEM_MOCOUTCR_MOCOUTRM_P1 (1 << R_SYSTEM_MOCOUTCR_MOCOUTRM_SHIFT)           /* +1 */
#  define R_SYSTEM_MOCOUTCR_MOCOUTRM_P125 (125 << R_SYSTEM_MOCOUTCR_MOCOUTRM_SHIFT)       /* +125 */
#  define R_SYSTEM_MOCOUTCR_MOCOUTRM_P126 (126 << R_SYSTEM_MOCOUTCR_MOCOUTRM_SHIFT)       /* +126 */
#  define R_SYSTEM_MOCOUTCR_MOCOUTRM_P127 (127 << R_SYSTEM_MOCOUTCR_MOCOUTRM_SHIFT)       /* +127 */

/* HOCO User Trimming Control Register (8-bits) *****************************/

#define R_SYSTEM_HOCOUTCR_HOCOUTRM_SHIFT (0)
#define R_SYSTEM_HOCOUTCR_HOCOUTRM_MASK (0xff)
#  define R_SYSTEM_HOCOUTCR_HOCOUTRM_M128 (128 << R_SYSTEM_HOCOUTCR_HOCOUTRM_SHIFT)       /* -128 */
#  define R_SYSTEM_HOCOUTCR_HOCOUTRM_M127 (129 << R_SYSTEM_HOCOUTCR_HOCOUTRM_SHIFT)       /* -127 */
#  define R_SYSTEM_HOCOUTCR_HOCOUTRM_M126 (130 << R_SYSTEM_HOCOUTCR_HOCOUTRM_SHIFT)       /* -126 */
#  define R_SYSTEM_HOCOUTCR_HOCOUTRM_M1 (255 << R_SYSTEM_HOCOUTCR_HOCOUTRM_SHIFT)         /* -1 */
#  define R_SYSTEM_HOCOUTCR_HOCOUTRM_CENTER_CODE (0 << R_SYSTEM_HOCOUTCR_HOCOUTRM_SHIFT)  /* Center Code */
#  define R_SYSTEM_HOCOUTCR_HOCOUTRM_P1 (1 << R_SYSTEM_HOCOUTCR_HOCOUTRM_SHIFT)           /* +1 */
#  define R_SYSTEM_HOCOUTCR_HOCOUTRM_P125 (125 << R_SYSTEM_HOCOUTCR_HOCOUTRM_SHIFT)       /* +125 */
#  define R_SYSTEM_HOCOUTCR_HOCOUTRM_P126 (126 << R_SYSTEM_HOCOUTCR_HOCOUTRM_SHIFT)       /* +126 */
#  define R_SYSTEM_HOCOUTCR_HOCOUTRM_P127 (127 << R_SYSTEM_HOCOUTCR_HOCOUTRM_SHIFT)       /* +127 */

/* USB clock Division control register (8-bits) *****************************/

#define R_SYSTEM_USBCKDIVCR_USBCKDIV_SHIFT (0)
#define R_SYSTEM_USBCKDIVCR_USBCKDIV_MASK (0x7)
#  define R_SYSTEM_USBCKDIVCR_USBCKDIV_V1_VALUE_AFTER_RESET (0 << R_SYSTEM_USBCKDIVCR_USBCKDIV_SHIFT)  /* /1 (value after reset) */
#  define R_SYSTEM_USBCKDIVCR_USBCKDIV_DIV_2 (1 << R_SYSTEM_USBCKDIVCR_USBCKDIV_SHIFT)                 /* /2 */
#  define R_SYSTEM_USBCKDIVCR_USBCKDIV_DIV_4 (2 << R_SYSTEM_USBCKDIVCR_USBCKDIV_SHIFT)                 /* /4 */
#  define R_SYSTEM_USBCKDIVCR_USBCKDIV_DIV_6 (3 << R_SYSTEM_USBCKDIVCR_USBCKDIV_SHIFT)                 /* /6 */
#  define R_SYSTEM_USBCKDIVCR_USBCKDIV_DIV_8 (4 << R_SYSTEM_USBCKDIVCR_USBCKDIV_SHIFT)                 /* /8 */
#  define R_SYSTEM_USBCKDIVCR_USBCKDIV_DIV_3 (5 << R_SYSTEM_USBCKDIVCR_USBCKDIV_SHIFT)                 /* /3 */
#  define R_SYSTEM_USBCKDIVCR_USBCKDIV_DIV_5 (6 << R_SYSTEM_USBCKDIVCR_USBCKDIV_SHIFT)                 /* /5 */

/* Octal-SPI clock Division control register (8-bits) ***********************/

#define R_SYSTEM_OCTACKDIVCR_OCTACKDIV_SHIFT (0)
#define R_SYSTEM_OCTACKDIVCR_OCTACKDIV_MASK (0x7)
#  define R_SYSTEM_OCTACKDIVCR_OCTACKDIV_V1_VALUE_AFTER_RESET (0 << R_SYSTEM_OCTACKDIVCR_OCTACKDIV_SHIFT)  /* /1 (value after reset) */
#  define R_SYSTEM_OCTACKDIVCR_OCTACKDIV_DIV_2 (1 << R_SYSTEM_OCTACKDIVCR_OCTACKDIV_SHIFT)                 /* /2 */
#  define R_SYSTEM_OCTACKDIVCR_OCTACKDIV_DIV_4 (2 << R_SYSTEM_OCTACKDIVCR_OCTACKDIV_SHIFT)                 /* /4 */
#  define R_SYSTEM_OCTACKDIVCR_OCTACKDIV_DIV_6 (3 << R_SYSTEM_OCTACKDIVCR_OCTACKDIV_SHIFT)                 /* /6 */
#  define R_SYSTEM_OCTACKDIVCR_OCTACKDIV_DIV_8 (4 << R_SYSTEM_OCTACKDIVCR_OCTACKDIV_SHIFT)                 /* /8 */
#  define R_SYSTEM_OCTACKDIVCR_OCTACKDIV_DIV_3 (5 << R_SYSTEM_OCTACKDIVCR_OCTACKDIV_SHIFT)                 /* /3 */
#  define R_SYSTEM_OCTACKDIVCR_OCTACKDIV_DIV_5 (6 << R_SYSTEM_OCTACKDIVCR_OCTACKDIV_SHIFT)                 /* /5 */

/* CANFD Core clock Division control register (8-bits) **********************/

#define R_SYSTEM_CANFDCKDIVCR_CANFDCKDIV_SHIFT (0)
#define R_SYSTEM_CANFDCKDIVCR_CANFDCKDIV_MASK (0x7)
#  define R_SYSTEM_CANFDCKDIVCR_CANFDCKDIV_V1_VALUE_AFTER_RESET (0 << R_SYSTEM_CANFDCKDIVCR_CANFDCKDIV_SHIFT)  /* /1 (value after reset) */
#  define R_SYSTEM_CANFDCKDIVCR_CANFDCKDIV_DIV_2 (1 << R_SYSTEM_CANFDCKDIVCR_CANFDCKDIV_SHIFT)                 /* /2 */
#  define R_SYSTEM_CANFDCKDIVCR_CANFDCKDIV_DIV_4 (2 << R_SYSTEM_CANFDCKDIVCR_CANFDCKDIV_SHIFT)                 /* /4 */
#  define R_SYSTEM_CANFDCKDIVCR_CANFDCKDIV_DIV_6 (3 << R_SYSTEM_CANFDCKDIVCR_CANFDCKDIV_SHIFT)                 /* /6 */
#  define R_SYSTEM_CANFDCKDIVCR_CANFDCKDIV_DIV_8 (4 << R_SYSTEM_CANFDCKDIVCR_CANFDCKDIV_SHIFT)                 /* /8 */
#  define R_SYSTEM_CANFDCKDIVCR_CANFDCKDIV_DIV_3 (5 << R_SYSTEM_CANFDCKDIVCR_CANFDCKDIV_SHIFT)                 /* /3 */
#  define R_SYSTEM_CANFDCKDIVCR_CANFDCKDIV_DIV_5 (6 << R_SYSTEM_CANFDCKDIVCR_CANFDCKDIV_SHIFT)                 /* /5 */

/* USB60 clock Division control register (8-bits) ***************************/

#define R_SYSTEM_USB60CKDIVCR_USB60CKDIV_SHIFT (0)
#define R_SYSTEM_USB60CKDIVCR_USB60CKDIV_MASK (0x7)
#  define R_SYSTEM_USB60CKDIVCR_USB60CKDIV_V1_VALUE_AFTER_RESET (0 << R_SYSTEM_USB60CKDIVCR_USB60CKDIV_SHIFT)  /* /1 (value after reset) */
#  define R_SYSTEM_USB60CKDIVCR_USB60CKDIV_DIV_2 (1 << R_SYSTEM_USB60CKDIVCR_USB60CKDIV_SHIFT)                 /* /2 */
#  define R_SYSTEM_USB60CKDIVCR_USB60CKDIV_DIV_4 (2 << R_SYSTEM_USB60CKDIVCR_USB60CKDIV_SHIFT)                 /* /4 */
#  define R_SYSTEM_USB60CKDIVCR_USB60CKDIV_DIV_6 (3 << R_SYSTEM_USB60CKDIVCR_USB60CKDIV_SHIFT)                 /* /6 */
#  define R_SYSTEM_USB60CKDIVCR_USB60CKDIV_DIV_8 (4 << R_SYSTEM_USB60CKDIVCR_USB60CKDIV_SHIFT)                 /* /8 */
#  define R_SYSTEM_USB60CKDIVCR_USB60CKDIV_DIV_3 (5 << R_SYSTEM_USB60CKDIVCR_USB60CKDIV_SHIFT)                 /* /3 */
#  define R_SYSTEM_USB60CKDIVCR_USB60CKDIV_DIV_5 (6 << R_SYSTEM_USB60CKDIVCR_USB60CKDIV_SHIFT)                 /* /5 */

/* I3C clock Division control register (8-bits) *****************************/

#define R_SYSTEM_I3CCKDIVCR_I3CCKDIV_SHIFT (0)
#define R_SYSTEM_I3CCKDIVCR_I3CCKDIV_MASK (0x7)
#  define R_SYSTEM_I3CCKDIVCR_I3CCKDIV_V1_VALUE_AFTER_RESET (0 << R_SYSTEM_I3CCKDIVCR_I3CCKDIV_SHIFT)  /* /1 (value after reset) */
#  define R_SYSTEM_I3CCKDIVCR_I3CCKDIV_DIV_2 (1 << R_SYSTEM_I3CCKDIVCR_I3CCKDIV_SHIFT)                 /* /2 */
#  define R_SYSTEM_I3CCKDIVCR_I3CCKDIV_DIV_4 (2 << R_SYSTEM_I3CCKDIVCR_I3CCKDIV_SHIFT)                 /* /4 */
#  define R_SYSTEM_I3CCKDIVCR_I3CCKDIV_DIV_6 (3 << R_SYSTEM_I3CCKDIVCR_I3CCKDIV_SHIFT)                 /* /6 */
#  define R_SYSTEM_I3CCKDIVCR_I3CCKDIV_DIV_8 (4 << R_SYSTEM_I3CCKDIVCR_I3CCKDIV_SHIFT)                 /* /8 */
#  define R_SYSTEM_I3CCKDIVCR_I3CCKDIV_DIV_3 (5 << R_SYSTEM_I3CCKDIVCR_I3CCKDIV_SHIFT)                 /* /3 */
#  define R_SYSTEM_I3CCKDIVCR_I3CCKDIV_DIV_5 (6 << R_SYSTEM_I3CCKDIVCR_I3CCKDIV_SHIFT)                 /* /5 */

/* USB clock control register (8-bits) **************************************/

#define R_SYSTEM_USBCKCR_USBCKSEL_SHIFT (0)
#define R_SYSTEM_USBCKCR_USBCKSEL_MASK (0xf)
#  define R_SYSTEM_USBCKCR_USBCKSEL_HOCO (0 << R_SYSTEM_USBCKCR_USBCKSEL_SHIFT)                    /* HOCO */
#  define R_SYSTEM_USBCKCR_USBCKSEL_MOCO_VALUE_AFTER_RESET (1 << R_SYSTEM_USBCKCR_USBCKSEL_SHIFT)  /* MOCO (value after reset) */
#  define R_SYSTEM_USBCKCR_USBCKSEL_LOCO (2 << R_SYSTEM_USBCKCR_USBCKSEL_SHIFT)                    /* LOCO */
#  define R_SYSTEM_USBCKCR_USBCKSEL_MAIN_CLOCK_OSCILLATOR (3 << R_SYSTEM_USBCKCR_USBCKSEL_SHIFT)   /* Main clock oscillator */
#  define R_SYSTEM_USBCKCR_USBCKSEL_SUB_CLOCK_OSCILLATOR (4 << R_SYSTEM_USBCKCR_USBCKSEL_SHIFT)    /* Sub-clock oscillator */
#  define R_SYSTEM_USBCKCR_USBCKSEL_PLL1P (5 << R_SYSTEM_USBCKCR_USBCKSEL_SHIFT)                   /* PLL1P */
#  define R_SYSTEM_USBCKCR_USBCKSEL_PLL2P (6 << R_SYSTEM_USBCKCR_USBCKSEL_SHIFT)                   /* PLL2P */
#  define R_SYSTEM_USBCKCR_USBCKSEL_PLL1Q (7 << R_SYSTEM_USBCKCR_USBCKSEL_SHIFT)                   /* PLL1Q */
#  define R_SYSTEM_USBCKCR_USBCKSEL_PLL1R (8 << R_SYSTEM_USBCKCR_USBCKSEL_SHIFT)                   /* PLL1R */
#  define R_SYSTEM_USBCKCR_USBCKSEL_PLL2Q (9 << R_SYSTEM_USBCKCR_USBCKSEL_SHIFT)                   /* PLL2Q */
#  define R_SYSTEM_USBCKCR_USBCKSEL_PLL2R (10 << R_SYSTEM_USBCKCR_USBCKSEL_SHIFT)                  /* PLL2R */
#define R_SYSTEM_USBCKCR_USBCKSREQ (1 <<  6)                                                       /* 40: USB clock (USBCLK) Switching Request */
#define R_SYSTEM_USBCKCR_USBCKSRDY (1 <<  7)                                                       /* 80: USB clock (USBCLK) Switching Ready state flag */

/* Octal-SPI clock control register (8-bits) ********************************/

#define R_SYSTEM_OCTACKCR_OCTACKSEL_SHIFT (0)
#define R_SYSTEM_OCTACKCR_OCTACKSEL_MASK (0xf)
#  define R_SYSTEM_OCTACKCR_OCTACKSEL_HOCO (0 << R_SYSTEM_OCTACKCR_OCTACKSEL_SHIFT)                    /* HOCO */
#  define R_SYSTEM_OCTACKCR_OCTACKSEL_MOCO_VALUE_AFTER_RESET (1 << R_SYSTEM_OCTACKCR_OCTACKSEL_SHIFT)  /* MOCO (value after reset) */
#  define R_SYSTEM_OCTACKCR_OCTACKSEL_LOCO (2 << R_SYSTEM_OCTACKCR_OCTACKSEL_SHIFT)                    /* LOCO */
#  define R_SYSTEM_OCTACKCR_OCTACKSEL_MAIN_CLOCK_OSCILLATOR (3 << R_SYSTEM_OCTACKCR_OCTACKSEL_SHIFT)   /* Main clock oscillator */
#  define R_SYSTEM_OCTACKCR_OCTACKSEL_SUB_CLOCK_OSCILLATOR (4 << R_SYSTEM_OCTACKCR_OCTACKSEL_SHIFT)    /* Sub-clock oscillator */
#  define R_SYSTEM_OCTACKCR_OCTACKSEL_PLL1P (5 << R_SYSTEM_OCTACKCR_OCTACKSEL_SHIFT)                   /* PLL1P */
#  define R_SYSTEM_OCTACKCR_OCTACKSEL_PLL2P (6 << R_SYSTEM_OCTACKCR_OCTACKSEL_SHIFT)                   /* PLL2P */
#  define R_SYSTEM_OCTACKCR_OCTACKSEL_PLL1Q (7 << R_SYSTEM_OCTACKCR_OCTACKSEL_SHIFT)                   /* PLL1Q */
#  define R_SYSTEM_OCTACKCR_OCTACKSEL_PLL1R (8 << R_SYSTEM_OCTACKCR_OCTACKSEL_SHIFT)                   /* PLL1R */
#  define R_SYSTEM_OCTACKCR_OCTACKSEL_PLL2Q (9 << R_SYSTEM_OCTACKCR_OCTACKSEL_SHIFT)                   /* PLL2Q */
#  define R_SYSTEM_OCTACKCR_OCTACKSEL_PLL2R (10 << R_SYSTEM_OCTACKCR_OCTACKSEL_SHIFT)                  /* PLL2R */
#define R_SYSTEM_OCTACKCR_OCTACKSREQ (1 <<  6)                                                         /* 40: Octal-SPI clock (OCTACLK) Switching Request */
#define R_SYSTEM_OCTACKCR_OCTACKSRDY (1 <<  7)                                                         /* 80: Octal-SPI clock (OCTACLK) Switching Ready state flag */

/* CANFD Core clock control register (8-bits) *******************************/

#define R_SYSTEM_CANFDCKCR_CANFDCKSEL_SHIFT (0)
#define R_SYSTEM_CANFDCKCR_CANFDCKSEL_MASK (0xf)
#  define R_SYSTEM_CANFDCKCR_CANFDCKSEL_HOCO (0 << R_SYSTEM_CANFDCKCR_CANFDCKSEL_SHIFT)                    /* HOCO */
#  define R_SYSTEM_CANFDCKCR_CANFDCKSEL_MOCO_VALUE_AFTER_RESET (1 << R_SYSTEM_CANFDCKCR_CANFDCKSEL_SHIFT)  /* MOCO (value after reset) */
#  define R_SYSTEM_CANFDCKCR_CANFDCKSEL_LOCO (2 << R_SYSTEM_CANFDCKCR_CANFDCKSEL_SHIFT)                    /* LOCO */
#  define R_SYSTEM_CANFDCKCR_CANFDCKSEL_MAIN_CLOCK_OSCILLATOR (3 << R_SYSTEM_CANFDCKCR_CANFDCKSEL_SHIFT)   /* Main clock oscillator */
#  define R_SYSTEM_CANFDCKCR_CANFDCKSEL_SUB_CLOCK_OSCILLATOR (4 << R_SYSTEM_CANFDCKCR_CANFDCKSEL_SHIFT)    /* Sub-clock oscillator */
#  define R_SYSTEM_CANFDCKCR_CANFDCKSEL_PLL1P (5 << R_SYSTEM_CANFDCKCR_CANFDCKSEL_SHIFT)                   /* PLL1P */
#  define R_SYSTEM_CANFDCKCR_CANFDCKSEL_PLL2P (6 << R_SYSTEM_CANFDCKCR_CANFDCKSEL_SHIFT)                   /* PLL2P */
#  define R_SYSTEM_CANFDCKCR_CANFDCKSEL_PLL1Q (7 << R_SYSTEM_CANFDCKCR_CANFDCKSEL_SHIFT)                   /* PLL1Q */
#  define R_SYSTEM_CANFDCKCR_CANFDCKSEL_PLL1R (8 << R_SYSTEM_CANFDCKCR_CANFDCKSEL_SHIFT)                   /* PLL1R */
#  define R_SYSTEM_CANFDCKCR_CANFDCKSEL_PLL2Q (9 << R_SYSTEM_CANFDCKCR_CANFDCKSEL_SHIFT)                   /* PLL2Q */
#  define R_SYSTEM_CANFDCKCR_CANFDCKSEL_PLL2R (10 << R_SYSTEM_CANFDCKCR_CANFDCKSEL_SHIFT)                  /* PLL2R */
#define R_SYSTEM_CANFDCKCR_CANFDCKSREQ (1 <<  6)                                                           /* 40: CANFD Core clock (CANFDCLK) Switching Request */
#define R_SYSTEM_CANFDCKCR_CANFDCKSRDY (1 <<  7)                                                           /* 80: CANFD Core clock (CANFDCLK) Switching Ready state flag */

/* USB60 clock control register (8-bits) ************************************/

#define R_SYSTEM_USB60CKCR_USB60CKSEL_SHIFT (0)
#define R_SYSTEM_USB60CKCR_USB60CKSEL_MASK (0xf)
#  define R_SYSTEM_USB60CKCR_USB60CKSEL_HOCO (0 << R_SYSTEM_USB60CKCR_USB60CKSEL_SHIFT)                    /* HOCO */
#  define R_SYSTEM_USB60CKCR_USB60CKSEL_MOCO_VALUE_AFTER_RESET (1 << R_SYSTEM_USB60CKCR_USB60CKSEL_SHIFT)  /* MOCO (value after reset) */
#  define R_SYSTEM_USB60CKCR_USB60CKSEL_LOCO (2 << R_SYSTEM_USB60CKCR_USB60CKSEL_SHIFT)                    /* LOCO */
#  define R_SYSTEM_USB60CKCR_USB60CKSEL_MAIN_CLOCK_OSCILLATOR (3 << R_SYSTEM_USB60CKCR_USB60CKSEL_SHIFT)   /* Main clock oscillator */
#  define R_SYSTEM_USB60CKCR_USB60CKSEL_SUB_CLOCK_OSCILLATOR (4 << R_SYSTEM_USB60CKCR_USB60CKSEL_SHIFT)    /* Sub-clock oscillator */
#  define R_SYSTEM_USB60CKCR_USB60CKSEL_PLL1P (5 << R_SYSTEM_USB60CKCR_USB60CKSEL_SHIFT)                   /* PLL1P */
#  define R_SYSTEM_USB60CKCR_USB60CKSEL_PLL2P (6 << R_SYSTEM_USB60CKCR_USB60CKSEL_SHIFT)                   /* PLL2P */
#  define R_SYSTEM_USB60CKCR_USB60CKSEL_PLL1Q (7 << R_SYSTEM_USB60CKCR_USB60CKSEL_SHIFT)                   /* PLL1Q */
#  define R_SYSTEM_USB60CKCR_USB60CKSEL_PLL1R (8 << R_SYSTEM_USB60CKCR_USB60CKSEL_SHIFT)                   /* PLL1R */
#  define R_SYSTEM_USB60CKCR_USB60CKSEL_PLL2Q (9 << R_SYSTEM_USB60CKCR_USB60CKSEL_SHIFT)                   /* PLL2Q */
#  define R_SYSTEM_USB60CKCR_USB60CKSEL_PLL2R (10 << R_SYSTEM_USB60CKCR_USB60CKSEL_SHIFT)                  /* PLL2R */
#define R_SYSTEM_USB60CKCR_USB60CKSREQ (1 <<  6)                                                           /* 40: USB clock (USB60CLK) Switching Request */
#define R_SYSTEM_USB60CKCR_USB60CKSRDY (1 <<  7)                                                           /* 80: USB clock (USB60CLK) Switching Ready state flag */

/* I3C clock control register (8-bits) **************************************/

#define R_SYSTEM_I3CCKCR_I3CCKSEL_SHIFT (0)
#define R_SYSTEM_I3CCKCR_I3CCKSEL_MASK (0xf)
#  define R_SYSTEM_I3CCKCR_I3CCKSEL_HOCO (0 << R_SYSTEM_I3CCKCR_I3CCKSEL_SHIFT)                    /* HOCO */
#  define R_SYSTEM_I3CCKCR_I3CCKSEL_MOCO_VALUE_AFTER_RESET (1 << R_SYSTEM_I3CCKCR_I3CCKSEL_SHIFT)  /* MOCO (value after reset) */
#  define R_SYSTEM_I3CCKCR_I3CCKSEL_LOCO (2 << R_SYSTEM_I3CCKCR_I3CCKSEL_SHIFT)                    /* LOCO */
#  define R_SYSTEM_I3CCKCR_I3CCKSEL_MAIN_CLOCK_OSCILLATOR (3 << R_SYSTEM_I3CCKCR_I3CCKSEL_SHIFT)   /* Main clock oscillator */
#  define R_SYSTEM_I3CCKCR_I3CCKSEL_SUB_CLOCK_OSCILLATOR (4 << R_SYSTEM_I3CCKCR_I3CCKSEL_SHIFT)    /* Sub-clock oscillator */
#  define R_SYSTEM_I3CCKCR_I3CCKSEL_PLL1P (5 << R_SYSTEM_I3CCKCR_I3CCKSEL_SHIFT)                   /* PLL1P */
#  define R_SYSTEM_I3CCKCR_I3CCKSEL_PLL2P (6 << R_SYSTEM_I3CCKCR_I3CCKSEL_SHIFT)                   /* PLL2P */
#  define R_SYSTEM_I3CCKCR_I3CCKSEL_PLL1Q (7 << R_SYSTEM_I3CCKCR_I3CCKSEL_SHIFT)                   /* PLL1Q */
#  define R_SYSTEM_I3CCKCR_I3CCKSEL_PLL1R (8 << R_SYSTEM_I3CCKCR_I3CCKSEL_SHIFT)                   /* PLL1R */
#  define R_SYSTEM_I3CCKCR_I3CCKSEL_PLL2Q (9 << R_SYSTEM_I3CCKCR_I3CCKSEL_SHIFT)                   /* PLL2Q */
#  define R_SYSTEM_I3CCKCR_I3CCKSEL_PLL2R (10 << R_SYSTEM_I3CCKCR_I3CCKSEL_SHIFT)                  /* PLL2R */
#define R_SYSTEM_I3CCKCR_I3CCKREQ (1 <<  6)                                                        /* 40: I3C clock (I3CCLK) Switching Request */
#define R_SYSTEM_I3CCKCR_I3CCKSRDY (1 <<  7)                                                       /* 80: I3C clock (I3CCLK) Switching Ready state flag */

/* Main Clock Oscillator Standby Control Register (8-bits) ******************/

#define R_SYSTEM_MOSCSCR_MOSCSOKP (1 <<  0)  /* 01: Main Clock Oscillator Standby Oscillation Keep select */

/* High-Speed On-Chip Oscillator Standby Control Register (8-bits) **********/

#define R_SYSTEM_HOCOSCR_HOCOSOKP (1 <<  0)  /* 01: HOCO Standby Oscillation Keep select */

/* Operating Power Control Register (8-bits) ********************************/

#define R_SYSTEM_OPCCR_OPCM_SHIFT (0)
#define R_SYSTEM_OPCCR_OPCM_MASK (0x3)
#  define R_SYSTEM_OPCCR_OPCM_HIGH_SPEED_MODE (0 << R_SYSTEM_OPCCR_OPCM_SHIFT)  /* High-speed mode */
#  define R_SYSTEM_OPCCR_OPCM_PROHIBITED_1 (1 << R_SYSTEM_OPCCR_OPCM_SHIFT)     /* Prohibited */
#  define R_SYSTEM_OPCCR_OPCM_PROHIBITED_2 (2 << R_SYSTEM_OPCCR_OPCM_SHIFT)     /* Prohibited */
#  define R_SYSTEM_OPCCR_OPCM_LOW_SPEED_MODE (3 << R_SYSTEM_OPCCR_OPCM_SHIFT)   /* Low-speed mode */
#define R_SYSTEM_OPCCR_OPCMTSF (1 <<  4)                                        /* 10: Operating Power Control Mode Transition Status Flag */

/* Main Clock Oscillator Wait Control Register (8-bits) *********************/

#define R_SYSTEM_MOSCWTCR_MSTS_SHIFT (0)
#define R_SYSTEM_MOSCWTCR_MSTS_MASK (0xf)
#  define R_SYSTEM_MOSCWTCR_MSTS_V0000 (0 << R_SYSTEM_MOSCWTCR_MSTS_SHIFT)  /* Wait time= 11.4us (3 cycles) / 15.3us (4 cycles) */
#  define R_SYSTEM_MOSCWTCR_MSTS_V0001 (1 << R_SYSTEM_MOSCWTCR_MSTS_SHIFT)  /* Wait time= 133.5us (35 cycles) / 137.3us (36 cycles) */
#  define R_SYSTEM_MOSCWTCR_MSTS_V0010 (2 << R_SYSTEM_MOSCWTCR_MSTS_SHIFT)  /* Wait time= 255.6us (67 cycles) / 259.4us (68 cycles) */
#  define R_SYSTEM_MOSCWTCR_MSTS_V0011 (3 << R_SYSTEM_MOSCWTCR_MSTS_SHIFT)  /* Wait time= 499.7us (131 cycles) / 503.5us (132 cycles) */
#  define R_SYSTEM_MOSCWTCR_MSTS_V0100 (4 << R_SYSTEM_MOSCWTCR_MSTS_SHIFT)  /* Wait time= 988.0us (259 cycles) / 991.8us (260 cycles) */
#  define R_SYSTEM_MOSCWTCR_MSTS_V0101 (5 << R_SYSTEM_MOSCWTCR_MSTS_SHIFT)  /* Wait time= 2086.6us (547 cycles) (value after reset) / 2090.5us (548 cycles) (value after reset) */
#  define R_SYSTEM_MOSCWTCR_MSTS_V0110 (6 << R_SYSTEM_MOSCWTCR_MSTS_SHIFT)  /* Wait time= 4039.8us (1059 cycles) / 4043.6us (1060 cycles) */
#  define R_SYSTEM_MOSCWTCR_MSTS_V0111 (7 << R_SYSTEM_MOSCWTCR_MSTS_SHIFT)  /* Wait time= 8190.2us (2147 cycles) / 8194.0us (2148 cycles) */
#  define R_SYSTEM_MOSCWTCR_MSTS_V1000 (8 << R_SYSTEM_MOSCWTCR_MSTS_SHIFT)  /* Wait time= 16368.9us (4291 cycles) / 16372.7us (4292 cycles) */
#  define R_SYSTEM_MOSCWTCR_MSTS_V1001 (9 << R_SYSTEM_MOSCWTCR_MSTS_SHIFT)  /* Wait time= 31139.4us (8163 cycles) / 31143.2us (8164 cycles) */

/* Reset Status Register 1 (32-bits) ****************************************/

#define R_SYSTEM_RSTSR1_IWDTRF (1 <<  0)  /* 01: Independent Watchdog Timer Reset Detect FlagNOTE: Writable only to clear the flag. Confirm the value is 1 and then write 0. */
#define R_SYSTEM_RSTSR1_WDT0RF (1 <<  1)  /* 02: Watchdog Timer0 Reset Detect FlagNOTE: Writable only to clear the flag. Confirm the value is 1 and then write 0. */
#define R_SYSTEM_RSTSR1_SWRF (1 <<  2)    /* 04: Software Reset Detect FlagNOTE: Writable only to clear the flag. Confirm the value is 1 and then write 0. */
#define R_SYSTEM_RSTSR1_CLU0RF (1 <<  4)  /* 10: CPU0 Lockup Reset Detect FlagNOTE: Writable only to clear the flag. Confirm the value is 1 and then write 0. */
#define R_SYSTEM_RSTSR1_LM0RF (1 <<  5)   /* 20: Local memory 0 error Reset Detect FlagNOTE: Writable only to clear the flag. Confirm the value is 1 and then write 0. */
#define R_SYSTEM_RSTSR1_BUSRF (1 << 10)   /* 400: Bus error Reset Detect FlagNOTE: Writable only to clear the flag. Confirm the value is 1 and then write 0. */
#define R_SYSTEM_RSTSR1_CMRF (1 << 14)    /* 4000: Common memory error Reset Detect FlagNOTE: Writable only to clear the flag. Confirm the value is 1 and then write 0. */
#define R_SYSTEM_RSTSR1_WDT1RF (1 << 17)  /* 20000: Watchdog Timer1 Reset Detect FlagNOTE: Writable only to clear the flag. Confirm the value is 1 and then write 0. */
#define R_SYSTEM_RSTSR1_LM1RF (1 << 21)   /* 200000: Local memory 1 error Reset Detect FlagNOTE: Writable only to clear the flag. Confirm the value is 1 and then write 0. */
#define R_SYSTEM_RSTSR1_NWRF (1 << 22)    /* 400000: Network Reset Detect FlagNOTE: Writable only to clear the flag. Confirm the value is 1 and then write 0. */

/* System Register Access Control Register (8-bits) *************************/

#define R_SYSTEM_SYRACCR_BUSY (1 <<  0)  /* 01: Access Ready monitor */

/* Voltage Monitor %s Circuit Control Register 1 (8-bits) *******************/

#define R_SYSTEM_PVDCR1_SIZE 2
#define R_SYSTEM_PVDCR1_IDTSEL_SHIFT (0)
#define R_SYSTEM_PVDCR1_IDTSEL_MASK (0x3)
#  define R_SYSTEM_PVDCR1_IDTSEL_V00 (0 << R_SYSTEM_PVDCR1_IDTSEL_SHIFT)                  /* Generate when VCC>=Vdet (rise) is detected */
#  define R_SYSTEM_PVDCR1_IDTSEL_V01 (1 << R_SYSTEM_PVDCR1_IDTSEL_SHIFT)                  /* Generate when VCC<Vdet (fall) is detected */
#  define R_SYSTEM_PVDCR1_IDTSEL_V10 (2 << R_SYSTEM_PVDCR1_IDTSEL_SHIFT)                  /* Generate when fall and rise are detected */
#  define R_SYSTEM_PVDCR1_IDTSEL_SETTINGS_PROHIBITED (3 << R_SYSTEM_PVDCR1_IDTSEL_SHIFT)  /* Settings prohibited */
#define R_SYSTEM_PVDCR1_IRQSEL (1 <<  2)                                                  /* 04: Voltage Monitor Interrupt Type Select */

/* Voltage Monitor %s Circuit Status Register (8-bits) **********************/

#define R_SYSTEM_PVDSR_SIZE 2
#define R_SYSTEM_PVDSR_DET (1 <<  0)  /* 01: Voltage Monitor Voltage Change Detection Flag NOTE: Only 0 can be written to this bit. After writing 0 to this bit, it takes 2 system clock cycles for the bit to be read as 0. */
#define R_SYSTEM_PVDSR_MON (1 <<  1)  /* 02: Voltage Monitor Signal Monitor Flag */

/* Clock Recovery System Control Register (8-bits) **************************/

#define R_SYSTEM_CRVSYSCR_CRVEN (1 <<  0)  /* 01: Clock Recovery Enable */

/* Graphics Power Domain Control Register (8-bits) **************************/

#define R_SYSTEM_PDCTRGD_PDDE (1 <<  0)    /* 01: Power control enable */
#define R_SYSTEM_PDCTRGD_PDCSF (1 <<  6)   /* 40: Power control status flag */
#define R_SYSTEM_PDCTRGD_PDPGSF (1 <<  7)  /* 80: Power gating status flag */

/* VBATT Backup Register Security Attribute Boundary Address Register
 * (16-bits)
 */

#define R_SYSTEM_VBRSABAR_SABA_SHIFT (0)
#define R_SYSTEM_VBRSABAR_SABA_MASK (0xffff)

/* VBATT Backup Register Privilege Attribute Boundary Address Register for
 * Secure Region (16-bits)
 */

#define R_SYSTEM_VBRPABARS_PABAS_SHIFT (0)
#define R_SYSTEM_VBRPABARS_PABAS_MASK (0xffff)

/* VBATT Backup Register Privilege Attribute Boundary Address Register for
 * Non-secure Region (16-bits)
 */

#define R_SYSTEM_VBRPABARNS_PABANS_SHIFT (0)
#define R_SYSTEM_VBRPABARNS_PABANS_MASK (0xffff)

/* Clock Generation Function Security Attribute Register (32-bits) **********/

#define R_SYSTEM_CGFSAR_NONSEC00 (1 <<  0)  /* 01: Non-secure Attribute bit 0 */
#define R_SYSTEM_CGFSAR_NONSEC02 (1 <<  2)  /* 04: Non-secure Attribute bit 2 */
#define R_SYSTEM_CGFSAR_NONSEC03 (1 <<  3)  /* 08: Non-secure Attribute bit 3 */
#define R_SYSTEM_CGFSAR_NONSEC04 (1 <<  4)  /* 10: Non-secure Attribute bit 4 */
#define R_SYSTEM_CGFSAR_NONSEC05 (1 <<  5)  /* 20: Non-secure Attribute bit 5 */
#define R_SYSTEM_CGFSAR_NONSEC06 (1 <<  6)  /* 40: Non-secure Attribute bit 6 */
#define R_SYSTEM_CGFSAR_NONSEC07 (1 <<  7)  /* 80: Non-secure Attribute bit 7 */
#define R_SYSTEM_CGFSAR_NONSEC08 (1 <<  8)  /* 100: Non-secure Attribute bit 8 */
#define R_SYSTEM_CGFSAR_NONSEC09 (1 <<  9)  /* 200: Non-secure Attribute bit 9 */
#define R_SYSTEM_CGFSAR_NONSEC11 (1 << 11)  /* 800: Non-secure Attribute bit 11 */
#define R_SYSTEM_CGFSAR_NONSEC12 (1 << 12)  /* 1000: Non-secure Attribute bit 12 */
#define R_SYSTEM_CGFSAR_NONSEC13 (1 << 13)  /* 2000: Non-secure Attribute bit 13 */
#define R_SYSTEM_CGFSAR_NONSEC16 (1 << 16)  /* 10000: Non-secure Attribute bit 16 */
#define R_SYSTEM_CGFSAR_NONSEC17 (1 << 17)  /* 20000: Non-secure Attribute bit 17 */
#define R_SYSTEM_CGFSAR_NONSEC18 (1 << 18)  /* 40000: Non-secure Attribute bit 18 */
#define R_SYSTEM_CGFSAR_NONSEC19 (1 << 19)  /* 80000: Non-secure Attribute bit 19 */
#define R_SYSTEM_CGFSAR_NONSEC20 (1 << 20)  /* 100000: Non-secure Attribute bit 20 */
#define R_SYSTEM_CGFSAR_NONSEC21 (1 << 21)  /* 200000: Non-secure Attribute bit 21 */
#define R_SYSTEM_CGFSAR_NONSEC22 (1 << 22)  /* 400000: Non-secure Attribute bit 22 */
#define R_SYSTEM_CGFSAR_NONSEC24 (1 << 24)  /* 1000000: Non-secure Attribute bit 24 */
#define R_SYSTEM_CGFSAR_NONSEC25 (1 << 25)  /* 2000000: Non-secure Attribute bit 25 */
#define R_SYSTEM_CGFSAR_NONSEC26 (1 << 26)  /* 4000000: Non-secure Attribute bit 26 */

/* Reset Security Attribution Register (32-bits) ****************************/

#define R_SYSTEM_RSTSAR_NONSEC0 (1 <<  0)  /* 01: Non-secure Attribute bit 0 */
#define R_SYSTEM_RSTSAR_NONSEC1 (1 <<  1)  /* 02: Non-secure Attribute bit 1 */
#define R_SYSTEM_RSTSAR_NONSEC2 (1 <<  2)  /* 04: Non-secure Attribute bit 2 */
#define R_SYSTEM_RSTSAR_NONSEC3 (1 <<  3)  /* 08: Non-secure Attribute bit 3 */

/* Low Power Mode Security Attribution Register (32-bits) *******************/

#define R_SYSTEM_LPMSAR_NONSEC0 (1 <<  0)   /* 01: Non-secure Attribute bit 00 */
#define R_SYSTEM_LPMSAR_NONSEC1 (1 <<  1)   /* 02: Non-secure Attribute bit 01 */
#define R_SYSTEM_LPMSAR_NONSEC2 (1 <<  2)   /* 04: Non-secure Attribute bit 02 */
#define R_SYSTEM_LPMSAR_NONSEC3 (1 <<  3)   /* 08: Non-secure Attribute bit 03 */
#define R_SYSTEM_LPMSAR_NONSEC8 (1 <<  8)   /* 100: Non-secure Attribute bit 08 */
#define R_SYSTEM_LPMSAR_NONSEC16 (1 << 16)  /* 10000: Non-secure Attribute bit 16 */
#define R_SYSTEM_LPMSAR_NONSEC17 (1 << 17)  /* 20000: Non-secure Attribute bit 17 */
#define R_SYSTEM_LPMSAR_NONSEC18 (1 << 18)  /* 40000: Non-secure Attribute bit 18 */
#define R_SYSTEM_LPMSAR_NONSEC19 (1 << 19)  /* 80000: Non-secure Attribute bit 19 */
#define R_SYSTEM_LPMSAR_NONSEC21 (1 << 21)  /* 200000: Non-secure Attribute bit 21 */

/* Programmable Voltage Detection Security Attribution Register (32-bits) ***/

#define R_SYSTEM_PVDSAR_NONSEC0 (1 <<  0)  /* 01: Non-secure Attribute bit 0 */
#define R_SYSTEM_PVDSAR_NONSEC1 (1 <<  1)  /* 02: Non-secure Attribute bit 1 */

/* Battery Backup Function Security Attribute Register (32-bits) ************/

#define R_SYSTEM_BBFSAR_NONSEC0 (1 <<  0)  /* 01: Non-secure Attribute bit 0 */
#define R_SYSTEM_BBFSAR_NONSEC1 (1 <<  1)  /* 02: Non-secure Attribute bit 1 */
#define R_SYSTEM_BBFSAR_NONSEC2 (1 <<  2)  /* 04: Non-secure Attribute bit 2 */
#define R_SYSTEM_BBFSAR_NONSEC3 (1 <<  3)  /* 08: Non-secure Attribute bit 3 */
#define R_SYSTEM_BBFSAR_NONSEC4 (1 <<  4)  /* 10: Non-secure Attribute bit 4 */

/* Power Gating Control Security Attribution Register (32-bits) *************/

#define R_SYSTEM_PGCSAR_NONSEC1 (1 <<  1)  /* 02: Non-secure Attribute bit 01 */
#define R_SYSTEM_PGCSAR_NONSEC2 (1 <<  2)  /* 04: Non-secure Attribute bit 02 */

/* Deep Standby Interrupt Factor Security Attribution Register (32-bits) ****/

#define R_SYSTEM_DPFSAR_DPFSA16 (1 << 16)  /* 10000: Deep Standby Interrupt Factor Security Attribute bit 16 */
#define R_SYSTEM_DPFSAR_DPFSA17 (1 << 17)  /* 20000: Deep Standby Interrupt Factor Security Attribute bit 17 */
#define R_SYSTEM_DPFSAR_DPFSA18 (1 << 18)  /* 40000: Deep Standby Interrupt Factor Security Attribute bit 18 */
#define R_SYSTEM_DPFSAR_DPFSA19 (1 << 19)  /* 80000: Deep Standby Interrupt Factor Security Attribute bit 19 */
#define R_SYSTEM_DPFSAR_DPFSA20 (1 << 20)  /* 100000: Deep Standby Interrupt Factor Security Attribute bit 20 */
#define R_SYSTEM_DPFSAR_DPFSA24 (1 << 24)  /* 1000000: Deep Standby Interrupt Factor Security Attribute bit 24 */
#define R_SYSTEM_DPFSAR_DPFSA25 (1 << 25)  /* 2000000: Deep Standby Interrupt Factor Security Attribute bit 25 */
#define R_SYSTEM_DPFSAR_DPFSA26 (1 << 26)  /* 4000000: Deep Standby Interrupt Factor Security Attribute bit 26 */
#define R_SYSTEM_DPFSAR_DPFSA27 (1 << 27)  /* 8000000: Deep Standby Interrupt Factor Security Attribute bit 27 */
#define R_SYSTEM_DPFSAR_DPFSA29 (1 << 29)  /* 20000000: Deep Standby Interrupt Factor Security Attribute bit 29 */
#define R_SYSTEM_DPFSAR_DPFSA31 (1 << 31)  /* 80000000: Deep Standby Interrupt Factor Security Attribute bit 31 */

/* RAM Standby Control Security Attribution Register (32-bits) **************/

#define R_SYSTEM_RSCSAR_RSCSA0 (1 <<  0)   /* 01: RAM Standby Control Security Attribute bit 00 */
#define R_SYSTEM_RSCSAR_RSCSA1 (1 <<  1)   /* 02: RAM Standby Control Security Attribute bit 01 */
#define R_SYSTEM_RSCSAR_RSCSA2 (1 <<  2)   /* 04: RAM Standby Control Security Attribute bit 02 */
#define R_SYSTEM_RSCSAR_RSCSA3 (1 <<  3)   /* 08: RAM Standby Control Security Attribute bit 03 */
#define R_SYSTEM_RSCSAR_RSCSA4 (1 <<  4)   /* 10: RAM Standby Control Security Attribute bit 04 */
#define R_SYSTEM_RSCSAR_RSCSA5 (1 <<  5)   /* 20: RAM Standby Control Security Attribute bit 05 */
#define R_SYSTEM_RSCSAR_RSCSA6 (1 <<  6)   /* 40: RAM Standby Control Security Attribute bit 06 */
#define R_SYSTEM_RSCSAR_RSCSA7 (1 <<  7)   /* 80: RAM Standby Control Security Attribute bit 07 */
#define R_SYSTEM_RSCSAR_RSCSA8 (1 <<  8)   /* 100: RAM Standby Control Security Attribute bit 08 */
#define R_SYSTEM_RSCSAR_RSCSA9 (1 <<  9)   /* 200: RAM Standby Control Security Attribute bit 09 */
#define R_SYSTEM_RSCSAR_RSCSA10 (1 << 10)  /* 400: RAM Standby Control Security Attribute bit 10 */
#define R_SYSTEM_RSCSAR_RSCSA11 (1 << 11)  /* 800: RAM Standby Control Security Attribute bit 11 */
#define R_SYSTEM_RSCSAR_RSCSA12 (1 << 12)  /* 1000: RAM Standby Control Security Attribute bit 12 */
#define R_SYSTEM_RSCSAR_RSCSA13 (1 << 13)  /* 2000: RAM Standby Control Security Attribute bit 13 */
#define R_SYSTEM_RSCSAR_RSCSA14 (1 << 14)  /* 4000: RAM Standby Control Security Attribute bit 14 */
#define R_SYSTEM_RSCSAR_RSCSA16 (1 << 16)  /* 10000: RAM Standby Control Security Attribute bit 16 */
#define R_SYSTEM_RSCSAR_RSCSA17 (1 << 17)  /* 20000: RAM Standby Control Security Attribute bit 17 */

/* Protect Register for Secure Register (16-bits) ***************************/

#define R_SYSTEM_PRCR_S_PRC0 (1 <<  0)                                      /* 01: Enables writing to the registers related to the clock generation circuit. */
#define R_SYSTEM_PRCR_S_PRC1 (1 <<  1)                                      /* 02: Enables writing to the registers related to the operating modes, the low power modes, and the battery backup function. */
#define R_SYSTEM_PRCR_S_PRC3 (1 <<  3)                                      /* 08: Enables writing to the registers related to the PVD. */
#define R_SYSTEM_PRCR_S_PRC4 (1 <<  4)                                      /* 10: Enables writing to the registers related to the security and privilege setting registers. */
#define R_SYSTEM_PRCR_S_PRC5 (1 <<  5)                                      /* 20: Enables writing to the registers related the reset control. */
#define R_SYSTEM_PRCR_S_PRKEY_SHIFT (8)
#define R_SYSTEM_PRCR_S_PRKEY_MASK (0xff)
#  define R_SYSTEM_PRCR_S_PRKEY_V0XA5 (165 << R_SYSTEM_PRCR_S_PRKEY_SHIFT)  /* Enables writing to the PRCR_S register. */

/* Protect Register for Non-secure Register (16-bits) ***********************/

#define R_SYSTEM_PRCR_NS_PRC0 (1 <<  0)                                       /* 01: Enables writing to the registers related to the clock generation circuit. */
#define R_SYSTEM_PRCR_NS_PRC1 (1 <<  1)                                       /* 02: Enables writing to the registers related to the operating modes, the low power modes, and the battery backup function. */
#define R_SYSTEM_PRCR_NS_PRC3 (1 <<  3)                                       /* 08: Enables writing to the registers related to the PVD. */
#define R_SYSTEM_PRCR_NS_PRC4 (1 <<  4)                                       /* 10: Enables writing to the registers related to the privilege setting registers. */
#define R_SYSTEM_PRCR_NS_PRKEY_SHIFT (8)
#define R_SYSTEM_PRCR_NS_PRKEY_MASK (0xff)
#  define R_SYSTEM_PRCR_NS_PRKEY_V0XA5 (165 << R_SYSTEM_PRCR_NS_PRKEY_SHIFT)  /* Enables writing to the PRCR_NS register. */

/* Low-Speed On-Chip Oscillator Control Register (8-bits) *******************/

#define R_SYSTEM_LOCOCR_LCSTP (1 <<  0)  /* 01: LOCO Stop */

/* LOCO User Trimming Control Register (8-bits) *****************************/

#define R_SYSTEM_LOCOUTCR_LOCOUTRM_SHIFT (0)
#define R_SYSTEM_LOCOUTCR_LOCOUTRM_MASK (0xff)
#  define R_SYSTEM_LOCOUTCR_LOCOUTRM_M128 (128 << R_SYSTEM_LOCOUTCR_LOCOUTRM_SHIFT)       /* -128 */
#  define R_SYSTEM_LOCOUTCR_LOCOUTRM_M127 (129 << R_SYSTEM_LOCOUTCR_LOCOUTRM_SHIFT)       /* -127 */
#  define R_SYSTEM_LOCOUTCR_LOCOUTRM_M126 (130 << R_SYSTEM_LOCOUTCR_LOCOUTRM_SHIFT)       /* -126 */
#  define R_SYSTEM_LOCOUTCR_LOCOUTRM_M1 (255 << R_SYSTEM_LOCOUTCR_LOCOUTRM_SHIFT)         /* -1 */
#  define R_SYSTEM_LOCOUTCR_LOCOUTRM_CENTER_CODE (0 << R_SYSTEM_LOCOUTCR_LOCOUTRM_SHIFT)  /* Center Code */
#  define R_SYSTEM_LOCOUTCR_LOCOUTRM_P1 (1 << R_SYSTEM_LOCOUTCR_LOCOUTRM_SHIFT)           /* +1 */
#  define R_SYSTEM_LOCOUTCR_LOCOUTRM_P125 (125 << R_SYSTEM_LOCOUTCR_LOCOUTRM_SHIFT)       /* +125 */
#  define R_SYSTEM_LOCOUTCR_LOCOUTRM_P126 (126 << R_SYSTEM_LOCOUTCR_LOCOUTRM_SHIFT)       /* +126 */
#  define R_SYSTEM_LOCOUTCR_LOCOUTRM_P127 (127 << R_SYSTEM_LOCOUTCR_LOCOUTRM_SHIFT)       /* +127 */

/* Deep Standby Control Register (8-bits) ***********************************/

#define R_SYSTEM_DPSBYCR_DCSSMODE (1 <<  2)  /* 04: DCDC SSMODE */
#define R_SYSTEM_DPSBYCR_SRKEEP (1 <<  4)    /* 10: Standby RAM Retention */
#define R_SYSTEM_DPSBYCR_IOKEEP (1 <<  6)    /* 40: I/O Port Retention */

/* Deep Standby Wait Control Register (8-bits) ******************************/

#define R_SYSTEM_DPSWCR_WTSTS_SHIFT (0)
#define R_SYSTEM_DPSWCR_WTSTS_MASK (0xff)
#  define R_SYSTEM_DPSWCR_WTSTS_V0X0B (11 << R_SYSTEM_DPSWCR_WTSTS_SHIFT)   /* Wait cycle for fast recovery */
#  define R_SYSTEM_DPSWCR_WTSTS_V0X9A (154 << R_SYSTEM_DPSWCR_WTSTS_SHIFT)  /* Wait cycle for slow recovery */

/* Deep Standby Interrupt Enable Register 0 (8-bits) ************************/

#define R_SYSTEM_DPSIER0_DIRQ0E (1 <<  0)  /* 01: IRQ0-DS Pin Enable */
#define R_SYSTEM_DPSIER0_DIRQ1E (1 <<  1)  /* 02: IRQ1-DS Pin Enable */
#define R_SYSTEM_DPSIER0_DIRQ2E (1 <<  2)  /* 04: IRQ2-DS Pin Enable */
#define R_SYSTEM_DPSIER0_DIRQ3E (1 <<  3)  /* 08: IRQ3-DS Pin Enable */
#define R_SYSTEM_DPSIER0_DIRQ4E (1 <<  4)  /* 10: IRQ4-DS Pin Enable */
#define R_SYSTEM_DPSIER0_DIRQ5E (1 <<  5)  /* 20: IRQ5-DS Pin Enable */
#define R_SYSTEM_DPSIER0_DIRQ6E (1 <<  6)  /* 40: IRQ6-DS Pin Enable */
#define R_SYSTEM_DPSIER0_DIRQ7E (1 <<  7)  /* 80: IRQ7-DS Pin Enable */

/* Deep Standby Interrupt Enable Register 1 (8-bits) ************************/

#define R_SYSTEM_DPSIER1_DIRQ8E (1 <<  0)   /* 01: IRQ8-DS Pin Enable */
#define R_SYSTEM_DPSIER1_DIRQ9E (1 <<  1)   /* 02: IRQ9-DS Pin Enable */
#define R_SYSTEM_DPSIER1_DIRQ10E (1 <<  2)  /* 04: IRQ10-DS Pin Enable */
#define R_SYSTEM_DPSIER1_DIRQ11E (1 <<  3)  /* 08: IRQ11-DS Pin Enable */
#define R_SYSTEM_DPSIER1_DIRQ12E (1 <<  4)  /* 10: IRQ12-DS Pin Enable */
#define R_SYSTEM_DPSIER1_DIRQ13E (1 <<  5)  /* 20: IRQ13-DS Pin Enable */
#define R_SYSTEM_DPSIER1_DIRQ14E (1 <<  6)  /* 40: IRQ14-DS Pin Enable */
#define R_SYSTEM_DPSIER1_DIRQ15E (1 <<  7)  /* 80: IRQ15-DS Pin Enable */

/* Deep Standby Interrupt Enable Register 2 (8-bits) ************************/

#define R_SYSTEM_DPSIER2_DPVD1IE (1 <<  0)   /* 01: PVD1 Deep Standby Cancel Signal Enable */
#define R_SYSTEM_DPSIER2_DPVD2IE (1 <<  1)   /* 02: PVD2 Deep Standby Cancel Signal Enable */
#define R_SYSTEM_DPSIER2_DTRTCIIE (1 <<  2)  /* 04: RTC Interval interrupt Deep Standby Cancel Signal Enable */
#define R_SYSTEM_DPSIER2_DRTCAIE (1 <<  3)   /* 08: RTC Alarm interrupt Deep Standby Cancel Signal Enable */
#define R_SYSTEM_DPSIER2_DNMIE (1 <<  4)     /* 10: NMI Pin Enable */

/* Deep Standby Interrupt Enable Register 3 (8-bits) ************************/

#define R_SYSTEM_DPSIER3_DUSBFSIE (1 <<  0)    /* 01: USBFS Suspend/Resume Deep Standby Cancel Signal Enable */
#define R_SYSTEM_DPSIER3_DUSBHSIE (1 <<  1)    /* 02: USBHS Suspend/Resume Deep Standby Cancel Signal Enable */
#define R_SYSTEM_DPSIER3_DULPT0IE (1 <<  2)    /* 04: ULPT0 Overflow Deep Standby Cancel Signal Enable */
#define R_SYSTEM_DPSIER3_DULPT1IE (1 <<  3)    /* 08: ULPT1 Overflow Deep Standby Cancel Signal Enable */
#define R_SYSTEM_DPSIER3_DIWDTIE (1 <<  5)     /* 20: IWDT Overflow Deep Standby Cancel Signal Enable */
#define R_SYSTEM_DPSIER3_DVBATTADIE (1 <<  7)  /* 80: VBATT Tamper Detection Deep Standby Cancel Signal Enable */

/* Deep Standby Interrupt Flag Register 0 (8-bits) **************************/

#define R_SYSTEM_DPSIFR0_DIRQ0F (1 <<  0)  /* 01: IRQ0-DS Pin Deep Standby Cancel Flag */
#define R_SYSTEM_DPSIFR0_DIRQ1F (1 <<  1)  /* 02: IRQ1-DS Pin Deep Standby Cancel Flag */
#define R_SYSTEM_DPSIFR0_DIRQ2F (1 <<  2)  /* 04: IRQ2-DS Pin Deep Standby Cancel Flag */
#define R_SYSTEM_DPSIFR0_DIRQ3F (1 <<  3)  /* 08: IRQ3-DS Pin Deep Standby Cancel Flag */
#define R_SYSTEM_DPSIFR0_DIRQ4F (1 <<  4)  /* 10: IRQ4-DS Pin Deep Standby Cancel Flag */
#define R_SYSTEM_DPSIFR0_DIRQ5F (1 <<  5)  /* 20: IRQ5-DS Pin Deep Standby Cancel Flag */
#define R_SYSTEM_DPSIFR0_DIRQ6F (1 <<  6)  /* 40: IRQ6-DS Pin Deep Standby Cancel Flag */
#define R_SYSTEM_DPSIFR0_DIRQ7F (1 <<  7)  /* 80: IRQ7-DS Pin Deep Standby Cancel Flag */

/* Deep Standby Interrupt Flag Register 1 (8-bits) **************************/

#define R_SYSTEM_DPSIFR1_DIRQ8F (1 <<  0)   /* 01: IRQ8-DS Pin Deep Standby Cancel Flag */
#define R_SYSTEM_DPSIFR1_DIRQ9F (1 <<  1)   /* 02: IRQ9-DS Pin Deep Standby Cancel Flag */
#define R_SYSTEM_DPSIFR1_DIRQ10F (1 <<  2)  /* 04: IRQ10-DS Pin Deep Standby Cancel Flag */
#define R_SYSTEM_DPSIFR1_DIRQ11F (1 <<  3)  /* 08: IRQ11-DS Pin Deep Standby Cancel Flag */
#define R_SYSTEM_DPSIFR1_DIRQ12F (1 <<  4)  /* 10: IRQ12-DS Pin Deep Standby Cancel Flag */
#define R_SYSTEM_DPSIFR1_DIRQ13F (1 <<  5)  /* 20: IRQ13-DS Pin Deep Standby Cancel Flag */
#define R_SYSTEM_DPSIFR1_DIRQ14F (1 <<  6)  /* 40: IRQ14-DS Pin Deep Standby Cancel Flag */
#define R_SYSTEM_DPSIFR1_DIRQ15F (1 <<  7)  /* 80: IRQ15-DS Pin Deep Standby Cancel Flag */

/* Deep Standby Interrupt Flag Register 2 (8-bits) **************************/

#define R_SYSTEM_DPSIFR2_DPVD1IF (1 <<  0)   /* 01: PVD1 Deep Standby Cancel Flag */
#define R_SYSTEM_DPSIFR2_DPVD2IF (1 <<  1)   /* 02: PVD2 Deep Standby Cancel Flag */
#define R_SYSTEM_DPSIFR2_DTRTCIIF (1 <<  2)  /* 04: RTC Interval interrupt Deep Standby Cancel Flag */
#define R_SYSTEM_DPSIFR2_DRTCAIF (1 <<  3)   /* 08: RTC Alarm interrupt Deep Standby Cancel Flag */
#define R_SYSTEM_DPSIFR2_DNMIF (1 <<  4)     /* 10: NMI Pin Deep Standby Cancel Flag */

/* Deep Standby Interrupt Flag Register 3 (8-bits) **************************/

#define R_SYSTEM_DPSIFR3_DUSBFSIF (1 <<  0)    /* 01: USBFS Suspend/Resume Deep Standby Cancel Flag */
#define R_SYSTEM_DPSIFR3_DUSBHSIF (1 <<  1)    /* 02: USBHS Suspend/Resume Deep Standby Cancel Flag */
#define R_SYSTEM_DPSIFR3_DULPT0IF (1 <<  2)    /* 04: ULPT0 Overflow Deep Standby Cancel Flag */
#define R_SYSTEM_DPSIFR3_DULPT1IF (1 <<  3)    /* 08: ULPT1 Overflow Deep Standby Cancel Flag */
#define R_SYSTEM_DPSIFR3_DIWDTIF (1 <<  5)     /* 20: IWDT Overflow Deep Standby Cancel Flag */
#define R_SYSTEM_DPSIFR3_DVBATTADIF (1 <<  7)  /* 80: VBATT Tamper Detection Deep Standby Cancel Flag */

/* Deep Standby Interrupt Edge Register 0 (8-bits) **************************/

#define R_SYSTEM_DPSIEGR0_DIRQ0EG (1 <<  0)  /* 01: IRQ0-DS Pin Edge Select */
#define R_SYSTEM_DPSIEGR0_DIRQ1EG (1 <<  1)  /* 02: IRQ1-DS Pin Edge Select */
#define R_SYSTEM_DPSIEGR0_DIRQ2EG (1 <<  2)  /* 04: IRQ2-DS Pin Edge Select */
#define R_SYSTEM_DPSIEGR0_DIRQ3EG (1 <<  3)  /* 08: IRQ3-DS Pin Edge Select */
#define R_SYSTEM_DPSIEGR0_DIRQ4EG (1 <<  4)  /* 10: IRQ4-DS Pin Edge Select */
#define R_SYSTEM_DPSIEGR0_DIRQ5EG (1 <<  5)  /* 20: IRQ5-DS Pin Edge Select */
#define R_SYSTEM_DPSIEGR0_DIRQ6EG (1 <<  6)  /* 40: IRQ6-DS Pin Edge Select */
#define R_SYSTEM_DPSIEGR0_DIRQ7EG (1 <<  7)  /* 80: IRQ7-DS Pin Edge Select */

/* Deep Standby Interrupt Edge Register 1 (8-bits) **************************/

#define R_SYSTEM_DPSIEGR1_DIRQ8EG (1 <<  0)   /* 01: IRQ8-DS Pin Edge Select */
#define R_SYSTEM_DPSIEGR1_DIRQ9EG (1 <<  1)   /* 02: IRQ9-DS Pin Edge Select */
#define R_SYSTEM_DPSIEGR1_DIRQ10EG (1 <<  2)  /* 04: IRQ10-DS Pin Edge Select */
#define R_SYSTEM_DPSIEGR1_DIRQ11EG (1 <<  3)  /* 08: IRQ11-DS Pin Edge Select */
#define R_SYSTEM_DPSIEGR1_DIRQ12EG (1 <<  4)  /* 10: IRQ12-DS Pin Edge Select */
#define R_SYSTEM_DPSIEGR1_DIRQ13EG (1 <<  5)  /* 20: IRQ13-DS Pin Edge Select */
#define R_SYSTEM_DPSIEGR1_DIRQ14EG (1 <<  6)  /* 40: IRQ14-DS Pin Edge Select */
#define R_SYSTEM_DPSIEGR1_DIRQ15EG (1 <<  7)  /* 80: IRQ15-DS Pin Edge Select */

/* Deep Standby Interrupt Edge Register 2 (8-bits) **************************/

#define R_SYSTEM_DPSIEGR2_DPVD1EG (1 <<  0)  /* 01: PVD1 Edge Select */
#define R_SYSTEM_DPSIEGR2_DPVD2EG (1 <<  1)  /* 02: PVD2 Edge Select */
#define R_SYSTEM_DPSIEGR2_DNMIEG (1 <<  4)   /* 10: NMI Pin Edge Select */

/* System Control OCD Control Register (8-bits) *****************************/

#define R_SYSTEM_SYOCDCR_DOCDF (1 <<  0)  /* 01: Deep Standby OCD flag */
#define R_SYSTEM_SYOCDCR_DBGEN (1 <<  7)  /* 80: Debugger Enable bit */

/* Reset Status Register 0 (8-bits) *****************************************/

#define R_SYSTEM_RSTSR0_PORF (1 <<  0)     /* 01: Power-On Reset Detect FlagNOTE: Writable only to clear the flag. Confirm the value is 1 and then write 0. */
#define R_SYSTEM_RSTSR0_PVD0RF (1 <<  1)   /* 02: Voltage Monitor 0 Reset Detect FlagNOTE: Writable only to clear the flag. Confirm the value is 1 and then write 0. */
#define R_SYSTEM_RSTSR0_PVD1RF (1 <<  2)   /* 04: Voltage Monitor 1 Reset Detect FlagNOTE: Writable only to clear the flag. Confirm the value is 1 and then write 0. */
#define R_SYSTEM_RSTSR0_PVD2RF (1 <<  3)   /* 08: Voltage Monitor 2 Reset Detect FlagNOTE: Writable only to clear the flag. Confirm the value is 1 and then write 0. */
#define R_SYSTEM_RSTSR0_PVD3RF (1 <<  4)   /* 10: Voltage Monitor 3 Reset Detect FlagNOTE: Writable only to clear the flag. Confirm the value is 1 and then write 0. */
#define R_SYSTEM_RSTSR0_PVD4RF (1 <<  5)   /* 20: Voltage Monitor 4 Reset Detect FlagNOTE: Writable only to clear the flag. Confirm the value is 1 and then write 0. */
#define R_SYSTEM_RSTSR0_PVD5RF (1 <<  6)   /* 40: Voltage Monitor 5 Reset Detect FlagNOTE: Writable only to clear the flag. Confirm the value is 1 and then write 0. */
#define R_SYSTEM_RSTSR0_DPSRSTF (1 <<  7)  /* 80: Deep Software Standby Reset FlagNOTE: Writable only to clear the flag. Confirm the value is 1 and then write 0. */

/* Reset Status Register 2 (8-bits) *****************************************/

#define R_SYSTEM_RSTSR2_CWSF (1 <<  0)  /* 01: Cold/Warm Start Determination Flag */

/* Reset Status Register 3 (8-bits) *****************************************/

#define R_SYSTEM_RSTSR3_OCPRF (1 <<  4)  /* 10: Overcurrent protection reset Detect Flag */

/* Main Clock Oscillator Mode Oscillation Control Register (8-bits) *********/

#define R_SYSTEM_MOMCR_MODRV0_SHIFT (1)
#define R_SYSTEM_MOMCR_MODRV0_MASK (0x7)
#  define R_SYSTEM_MOMCR_MODRV0_V8MHZ (0 << R_SYSTEM_MOMCR_MODRV0_SHIFT)              /* 8MHz */
#  define R_SYSTEM_MOMCR_MODRV0_V8_1MHZ_TO_16MHZ (1 << R_SYSTEM_MOMCR_MODRV0_SHIFT)   /* 8.1MHz to 16MHz */
#  define R_SYSTEM_MOMCR_MODRV0_V16_1MHZ_TO_20MHZ (2 << R_SYSTEM_MOMCR_MODRV0_SHIFT)  /* 16.1MHz to 20MHz */
#  define R_SYSTEM_MOMCR_MODRV0_V20_1MHZ_TO_26MHZ (3 << R_SYSTEM_MOMCR_MODRV0_SHIFT)  /* 20.1MHz to 26MHz */
#  define R_SYSTEM_MOMCR_MODRV0_V48MHZ_4 (4 << R_SYSTEM_MOMCR_MODRV0_SHIFT)           /* 48MHz */
#  define R_SYSTEM_MOMCR_MODRV0_V101 (5 << R_SYSTEM_MOMCR_MODRV0_SHIFT)               /* 48MHz (value after reset) */
#  define R_SYSTEM_MOMCR_MODRV0_V48MHZ_6 (6 << R_SYSTEM_MOMCR_MODRV0_SHIFT)           /* 48MHz */
#  define R_SYSTEM_MOMCR_MODRV0_V48MHZ_7 (7 << R_SYSTEM_MOMCR_MODRV0_SHIFT)           /* 48MHz */
#define R_SYSTEM_MOMCR_AGCEN (1 <<  4)                                                /* 10: Auto Gain Control Enable */
#define R_SYSTEM_MOMCR_MOSEL (1 <<  6)                                                /* 40: Main Clock Oscillator Switching */

/* Flash Write Erase Protect Register (8-bits) ******************************/

#define R_SYSTEM_FWEPROR_FLWE_SHIFT (0)
#define R_SYSTEM_FWEPROR_FLWE_MASK (0x3)
#  define R_SYSTEM_FWEPROR_FLWE_V00 (0 << R_SYSTEM_FWEPROR_FLWE_SHIFT)  /* Prohibits programming and erasure of the code flash, data flash or blank checking. */
#  define R_SYSTEM_FWEPROR_FLWE_V01 (1 << R_SYSTEM_FWEPROR_FLWE_SHIFT)  /* Permits programming and erasure of the code flash, data flash or blank checking. */
#  define R_SYSTEM_FWEPROR_FLWE_V10 (2 << R_SYSTEM_FWEPROR_FLWE_SHIFT)  /* Prohibits programming and erasure of the code flash, data flash or blank checking. */
#  define R_SYSTEM_FWEPROR_FLWE_V11 (3 << R_SYSTEM_FWEPROR_FLWE_SHIFT)  /* Prohibits programming and erasure of the code flash, data flash or blank checking. */

/* Voltage Monitor 1 Comparator Control Register (8-bits) *******************/

#define R_SYSTEM_PVD1CMPCR_PVDLVL_SHIFT (0)
#define R_SYSTEM_PVD1CMPCR_PVDLVL_MASK (0x1f)
#  define R_SYSTEM_PVD1CMPCR_PVDLVL_V4_29V_VDETM_0 (0 << R_SYSTEM_PVD1CMPCR_PVDLVL_SHIFT)    /* 4.29V (Vdetm_0) */
#  define R_SYSTEM_PVD1CMPCR_PVDLVL_V4_16V_VDETM_1 (1 << R_SYSTEM_PVD1CMPCR_PVDLVL_SHIFT)    /* 4.16V (Vdetm_1) */
#  define R_SYSTEM_PVD1CMPCR_PVDLVL_V4_03V_VDETM_2 (2 << R_SYSTEM_PVD1CMPCR_PVDLVL_SHIFT)    /* 4.03V (Vdetm_2) */
#  define R_SYSTEM_PVD1CMPCR_PVDLVL_V3_86V_VDETM_3 (3 << R_SYSTEM_PVD1CMPCR_PVDLVL_SHIFT)    /* 3.86V (Vdetm_3) */
#  define R_SYSTEM_PVD1CMPCR_PVDLVL_V3_14V_VDETM_4 (4 << R_SYSTEM_PVD1CMPCR_PVDLVL_SHIFT)    /* 3.14V (Vdetm_4) */
#  define R_SYSTEM_PVD1CMPCR_PVDLVL_V3_10V_VDETM_5 (5 << R_SYSTEM_PVD1CMPCR_PVDLVL_SHIFT)    /* 3.10V (Vdetm_5) */
#  define R_SYSTEM_PVD1CMPCR_PVDLVL_V3_08V_VDETM_6 (6 << R_SYSTEM_PVD1CMPCR_PVDLVL_SHIFT)    /* 3.08V (Vdetm_6) */
#  define R_SYSTEM_PVD1CMPCR_PVDLVL_V2_85V_VDETM_7 (7 << R_SYSTEM_PVD1CMPCR_PVDLVL_SHIFT)    /* 2.85V (Vdetm_7) */
#  define R_SYSTEM_PVD1CMPCR_PVDLVL_V2_83V_VDETM_8 (8 << R_SYSTEM_PVD1CMPCR_PVDLVL_SHIFT)    /* 2.83V (Vdetm_8) */
#  define R_SYSTEM_PVD1CMPCR_PVDLVL_V2_80V_VDETM_9 (9 << R_SYSTEM_PVD1CMPCR_PVDLVL_SHIFT)    /* 2.80V (Vdetm_9) */
#  define R_SYSTEM_PVD1CMPCR_PVDLVL_V2_62V_VDETM_10 (10 << R_SYSTEM_PVD1CMPCR_PVDLVL_SHIFT)  /* 2.62V (Vdetm_10) */
#  define R_SYSTEM_PVD1CMPCR_PVDLVL_V2_33V_VDETM_11 (11 << R_SYSTEM_PVD1CMPCR_PVDLVL_SHIFT)  /* 2.33V (Vdetm_11) */
#  define R_SYSTEM_PVD1CMPCR_PVDLVL_V1_90V_VDETM_12 (12 << R_SYSTEM_PVD1CMPCR_PVDLVL_SHIFT)  /* 1.90V (Vdetm_12) */
#  define R_SYSTEM_PVD1CMPCR_PVDLVL_V1_86V_VDETM_13 (13 << R_SYSTEM_PVD1CMPCR_PVDLVL_SHIFT)  /* 1.86V (Vdetm_13) */
#  define R_SYSTEM_PVD1CMPCR_PVDLVL_V1_74V_VDETM_14 (14 << R_SYSTEM_PVD1CMPCR_PVDLVL_SHIFT)  /* 1.74V (Vdetm_14) */
#  define R_SYSTEM_PVD1CMPCR_PVDLVL_V1_71V_VDETM_15 (15 << R_SYSTEM_PVD1CMPCR_PVDLVL_SHIFT)  /* 1.71V (Vdetm_15) */
#define R_SYSTEM_PVD1CMPCR_PVDE (1 <<  7)                                                    /* 80: Voltage Detection 1 Enable */

/* Voltage Monitor 2 Comparator Control Register (8-bits) *******************/

#define R_SYSTEM_PVD2CMPCR_PVDLVL_SHIFT (0)
#define R_SYSTEM_PVD2CMPCR_PVDLVL_MASK (0x1f)
#  define R_SYSTEM_PVD2CMPCR_PVDLVL_V4_29V_VDETM_0 (0 << R_SYSTEM_PVD2CMPCR_PVDLVL_SHIFT)    /* 4.29V (Vdetm_0) */
#  define R_SYSTEM_PVD2CMPCR_PVDLVL_V4_16V_VDETM_1 (1 << R_SYSTEM_PVD2CMPCR_PVDLVL_SHIFT)    /* 4.16V (Vdetm_1) */
#  define R_SYSTEM_PVD2CMPCR_PVDLVL_V4_03V_VDETM_2 (2 << R_SYSTEM_PVD2CMPCR_PVDLVL_SHIFT)    /* 4.03V (Vdetm_2) */
#  define R_SYSTEM_PVD2CMPCR_PVDLVL_V3_86V_VDETM_3 (3 << R_SYSTEM_PVD2CMPCR_PVDLVL_SHIFT)    /* 3.86V (Vdetm_3) */
#  define R_SYSTEM_PVD2CMPCR_PVDLVL_V3_14V_VDETM_4 (4 << R_SYSTEM_PVD2CMPCR_PVDLVL_SHIFT)    /* 3.14V (Vdetm_4) */
#  define R_SYSTEM_PVD2CMPCR_PVDLVL_V3_10V_VDETM_5 (5 << R_SYSTEM_PVD2CMPCR_PVDLVL_SHIFT)    /* 3.10V (Vdetm_5) */
#  define R_SYSTEM_PVD2CMPCR_PVDLVL_V3_08V_VDETM_6 (6 << R_SYSTEM_PVD2CMPCR_PVDLVL_SHIFT)    /* 3.08V (Vdetm_6) */
#  define R_SYSTEM_PVD2CMPCR_PVDLVL_V2_85V_VDETM_7 (7 << R_SYSTEM_PVD2CMPCR_PVDLVL_SHIFT)    /* 2.85V (Vdetm_7) */
#  define R_SYSTEM_PVD2CMPCR_PVDLVL_V2_83V_VDETM_8 (8 << R_SYSTEM_PVD2CMPCR_PVDLVL_SHIFT)    /* 2.83V (Vdetm_8) */
#  define R_SYSTEM_PVD2CMPCR_PVDLVL_V2_80V_VDETM_9 (9 << R_SYSTEM_PVD2CMPCR_PVDLVL_SHIFT)    /* 2.80V (Vdetm_9) */
#  define R_SYSTEM_PVD2CMPCR_PVDLVL_V2_62V_VDETM_10 (10 << R_SYSTEM_PVD2CMPCR_PVDLVL_SHIFT)  /* 2.62V (Vdetm_10) */
#  define R_SYSTEM_PVD2CMPCR_PVDLVL_V2_33V_VDETM_11 (11 << R_SYSTEM_PVD2CMPCR_PVDLVL_SHIFT)  /* 2.33V (Vdetm_11) */
#  define R_SYSTEM_PVD2CMPCR_PVDLVL_V1_90V_VDETM_12 (12 << R_SYSTEM_PVD2CMPCR_PVDLVL_SHIFT)  /* 1.90V (Vdetm_12) */
#  define R_SYSTEM_PVD2CMPCR_PVDLVL_V1_86V_VDETM_13 (13 << R_SYSTEM_PVD2CMPCR_PVDLVL_SHIFT)  /* 1.86V (Vdetm_13) */
#  define R_SYSTEM_PVD2CMPCR_PVDLVL_V1_74V_VDETM_14 (14 << R_SYSTEM_PVD2CMPCR_PVDLVL_SHIFT)  /* 1.74V (Vdetm_14) */
#  define R_SYSTEM_PVD2CMPCR_PVDLVL_V1_71V_VDETM_15 (15 << R_SYSTEM_PVD2CMPCR_PVDLVL_SHIFT)  /* 1.71V (Vdetm_15) */
#define R_SYSTEM_PVD2CMPCR_PVDE (1 <<  7)                                                    /* 80: Voltage Detection 2 Enable */

/* Voltage Monitor %s Circuit Control Register 0 (8-bits) *******************/

#define R_SYSTEM_PVDCR0_SIZE 2
#define R_SYSTEM_PVDCR0_RIE (1 <<  0)                                                    /* 01: Voltage Monitor Interrupt/Reset Enable */
#define R_SYSTEM_PVDCR0_DFDIS (1 <<  1)                                                  /* 02: Voltage Monitor Digital Filter Disable Mode Select */
#define R_SYSTEM_PVDCR0_CMPE (1 <<  2)                                                   /* 04: Voltage Monitor Circuit Comparison Result Output Enable */
#define R_SYSTEM_PVDCR0_FSAMP_SHIFT (4)
#define R_SYSTEM_PVDCR0_FSAMP_MASK (0x3)
#  define R_SYSTEM_PVDCR0_FSAMP_V1_2_LOCO_FREQUENCY (0 << R_SYSTEM_PVDCR0_FSAMP_SHIFT)   /* 1/2 LOCO frequency */
#  define R_SYSTEM_PVDCR0_FSAMP_V1_4_LOCO_FREQUENCY (1 << R_SYSTEM_PVDCR0_FSAMP_SHIFT)   /* 1/4 LOCO frequency */
#  define R_SYSTEM_PVDCR0_FSAMP_V1_8_LOCO_FREQUENCY (2 << R_SYSTEM_PVDCR0_FSAMP_SHIFT)   /* 1/8 LOCO frequency */
#  define R_SYSTEM_PVDCR0_FSAMP_V1_16_LOCO_FREQUENCY (3 << R_SYSTEM_PVDCR0_FSAMP_SHIFT)  /* 1/16 LOCO frequency */
#define R_SYSTEM_PVDCR0_RI (1 <<  6)                                                     /* 40: Voltage Monitor Circuit Mode Select */
#define R_SYSTEM_PVDCR0_RN (1 <<  7)                                                     /* 80: Voltage Monitor Reset Negate Select */

/* Battery Backup Voltage Monitor Function Select Register (8-bits) *********/

#define R_SYSTEM_VBATTMNSELR_VBATTMNSEL (1 <<  0)  /* 01: VBATT Voltage Monitor Function Select Bit */

/* VBATT Battery Power Supply Control Register 1 (8-bits) *******************/

#define R_SYSTEM_VBTBPCR1_BPWSWSTP (1 <<  0)  /* 01: Battery Power Supply Switch Stop */

/* Low Power State Control Register (8-bits) ********************************/

#define R_SYSTEM_LPSCR_LPMD_SHIFT (0)
#define R_SYSTEM_LPSCR_LPMD_MASK (0xf)
#  define R_SYSTEM_LPSCR_LPMD_SYSTEM_ACTIVE (0 << R_SYSTEM_LPSCR_LPMD_SHIFT)            /* System Active */
#  define R_SYSTEM_LPSCR_LPMD_PROHIBITED_RESERVED_1 (1 << R_SYSTEM_LPSCR_LPMD_SHIFT)    /* Prohibited. (reserved) */
#  define R_SYSTEM_LPSCR_LPMD_V2H (2 << R_SYSTEM_LPSCR_LPMD_SHIFT)                      /* Prohibited. (reserved) for CPU0 Deep Sleep */
#  define R_SYSTEM_LPSCR_LPMD_V3H (3 << R_SYSTEM_LPSCR_LPMD_SHIFT)                      /* Prohibited. (reserved) CPU0 Power Gating */
#  define R_SYSTEM_LPSCR_LPMD_SOFTWARE_STANDBY_MODE_1 (4 << R_SYSTEM_LPSCR_LPMD_SHIFT)  /* Software Standby mode 1 */
#  define R_SYSTEM_LPSCR_LPMD_SOFTWARE_STANDBY_MODE_2 (5 << R_SYSTEM_LPSCR_LPMD_SHIFT)  /* Software Standby mode 2 */
#  define R_SYSTEM_LPSCR_LPMD_V6H (6 << R_SYSTEM_LPSCR_LPMD_SHIFT)                      /* Prohibited. (reserved for Software Standby mode 3) */
#  define R_SYSTEM_LPSCR_LPMD_PROHIBITED_RESERVED_7 (7 << R_SYSTEM_LPSCR_LPMD_SHIFT)    /* Prohibited. (reserved) */
#  define R_SYSTEM_LPSCR_LPMD_V8H (8 << R_SYSTEM_LPSCR_LPMD_SHIFT)                      /* Deep Software Standby mode 1 */
#  define R_SYSTEM_LPSCR_LPMD_V9H (9 << R_SYSTEM_LPSCR_LPMD_SHIFT)                      /* Deep Software Standby mode 2 */
#  define R_SYSTEM_LPSCR_LPMD_AH (10 << R_SYSTEM_LPSCR_LPMD_SHIFT)                      /* Deep Software Standby mode 3 */

/* Software Standby Control Register 1 (8-bits) *****************************/

#define R_SYSTEM_SSCR1_SS1FR (1 <<  0)  /* 01: Software Standby 1 Fast Return */

/* Low Power State Control Register (8-bits) ********************************/

#define R_SYSTEM_LVOCR_LVO0E (1 <<  0)  /* 01: Low Voltage Operation 0 Enable */
#define R_SYSTEM_LVOCR_LVO1E (1 <<  1)  /* 02: Low Voltage Operation 1 Enable */

/* System Reset Mask Control Register0 (8-bits) *****************************/

#define R_SYSTEM_SYRSTMSK0_IWDTMASK (1 <<  0)   /* 01: Independent watchdog timer Reset Mask */
#define R_SYSTEM_SYRSTMSK0_WDT0MASK (1 <<  1)   /* 02: CPU0 Watchdog timer Reset Mask */
#define R_SYSTEM_SYRSTMSK0_SWMASK (1 <<  2)     /* 04: Software Reset Mask */
#define R_SYSTEM_SYRSTMSK0_CLUP0MASK (1 <<  4)  /* 10: CPU0 Lockup Reset Mask */
#define R_SYSTEM_SYRSTMSK0_LM0MASK (1 <<  5)    /* 20: Local memory 0 error Reset Mask */
#define R_SYSTEM_SYRSTMSK0_CMMASK (1 <<  6)     /* 40: Common memory error Reset Mask */
#define R_SYSTEM_SYRSTMSK0_BUSMASK (1 <<  7)    /* 80: BUS error Reset Mask */

/* System Reset Mask Control Register1 (8-bits) *****************************/

#define R_SYSTEM_SYRSTMSK1_LM1MASK (1 <<  5)  /* 20: Local memory 1 error Reset Mask */
#define R_SYSTEM_SYRSTMSK1_NWMASK (1 <<  7)   /* 80: Network Reset Mask */

/* System Reset Mask Control Register2 (8-bits) *****************************/

#define R_SYSTEM_SYRSTMSK2_PVD1MASK (1 <<  0)  /* 01: Voltage Monitor 1 Reset Mask */
#define R_SYSTEM_SYRSTMSK2_PVD2MASK (1 <<  1)  /* 02: Voltage Monitor 2 Reset Mask */
#define R_SYSTEM_SYRSTMSK2_PVD3MASK (1 <<  2)  /* 04: Voltage Monitor 3 Reset Mask */
#define R_SYSTEM_SYRSTMSK2_PVD4MASK (1 <<  3)  /* 08: Voltage Monitor 4 Reset Mask */
#define R_SYSTEM_SYRSTMSK2_PVD5MASK (1 <<  4)  /* 10: Voltage Monitor 5 Reset Mask */

/* PLL1-LDO Control Register (8-bits) ***************************************/

#define R_SYSTEM_PLL1LDOCR_LDOSTP (1 <<  0)  /* 01: LDO Stop */
#define R_SYSTEM_PLL1LDOCR_SKEEP (1 <<  1)   /* 02: STBY Keep */

/* PLL2-LDO Control Register (8-bits) ***************************************/

#define R_SYSTEM_PLL2LDOCR_LDOSTP (1 <<  0)  /* 01: LDO Stop */
#define R_SYSTEM_PLL2LDOCR_SKEEP (1 <<  1)   /* 02: STBY Keep */

/* HOCO-LDO Control Register (8-bits) ***************************************/

#define R_SYSTEM_HOCOLDOCR_LDOSTP (1 <<  0)  /* 01: LDO Stop */
#define R_SYSTEM_HOCOLDOCR_SKEEP (1 <<  1)   /* 02: STBY Keep */

/* Main Clock Oscillator Mode Control Register 2 (8-bits) *******************/

#define R_SYSTEM_MOMCR2_MOMODE (1 <<  0)  /* 01: Main Clock Oscillator Mode Select */

/* Sub-clock oscillator control register (8-bits) ***************************/

#define R_SYSTEM_SOSCCR_SOSTP (1 <<  0)  /* 01: Sub-Clock Oscillator Stop */

/* Sub Clock Oscillator Mode Control Register (8-bits) **********************/

#define R_SYSTEM_SOMCR_SODRV_SHIFT (0)
#define R_SYSTEM_SOMCR_SODRV_MASK (0x3)
#  define R_SYSTEM_SOMCR_SODRV_V00 (0 << R_SYSTEM_SOMCR_SODRV_SHIFT)                   /* :Standard(12.5pf) (value after reset) */
#  define R_SYSTEM_SOMCR_SODRV_LOW_POWER_MODE_1_9PF (1 << R_SYSTEM_SOMCR_SODRV_SHIFT)  /* Low power mode 1 (9pf) */
#  define R_SYSTEM_SOMCR_SODRV_LOW_POWER_MODE_2_7PF (2 << R_SYSTEM_SOMCR_SODRV_SHIFT)  /* Low power mode 2 (7pf) */
#  define R_SYSTEM_SOMCR_SODRV_LOW_POWER_MODE_3_4PF (3 << R_SYSTEM_SOMCR_SODRV_SHIFT)  /* Low power mode 3 (4pf) */
#define R_SYSTEM_SOMCR_SOSEL (1 <<  6)                                                 /* 40: Sub Clock Oscillator Switching */

/* VBATT Backup Enable Register (8-bits) ************************************/

#define R_SYSTEM_VBTBER_VBAE (1 <<  3)  /* 08: VBATT backup register access enable bit */

/* VBATT Battery Power Supply Control Register 2 (8-bits) *******************/

#define R_SYSTEM_VBTBPCR2_VDETLVL_SHIFT (0)
#define R_SYSTEM_VBTBPCR2_VDETLVL_MASK (0x7)
#  define R_SYSTEM_VBTBPCR2_VDETLVL_V2_8V (0 << R_SYSTEM_VBTBPCR2_VDETLVL_SHIFT)             /* 2.8V */
#  define R_SYSTEM_VBTBPCR2_VDETLVL_V2_53V (1 << R_SYSTEM_VBTBPCR2_VDETLVL_SHIFT)            /* 2.53V */
#  define R_SYSTEM_VBTBPCR2_VDETLVL_V2_10V (2 << R_SYSTEM_VBTBPCR2_VDETLVL_SHIFT)            /* 2.10V */
#  define R_SYSTEM_VBTBPCR2_VDETLVL_V1_95V (3 << R_SYSTEM_VBTBPCR2_VDETLVL_SHIFT)            /* 1.95V */
#  define R_SYSTEM_VBTBPCR2_VDETLVL_V1_85V (4 << R_SYSTEM_VBTBPCR2_VDETLVL_SHIFT)            /* 1.85V */
#  define R_SYSTEM_VBTBPCR2_VDETLVL_V1_75V (5 << R_SYSTEM_VBTBPCR2_VDETLVL_SHIFT)            /* 1.75V */
#  define R_SYSTEM_VBTBPCR2_VDETLVL_V1_65V (6 << R_SYSTEM_VBTBPCR2_VDETLVL_SHIFT)            /* 1.65V */
#  define R_SYSTEM_VBTBPCR2_VDETLVL_PROHIBITED_1_55V (7 << R_SYSTEM_VBTBPCR2_VDETLVL_SHIFT)  /* prohibited (1.55V) */
#define R_SYSTEM_VBTBPCR2_VDETE (1 <<  4)                                                    /* 10: Voltage drop detection enable */

/* VBATT Battery Power Supply Status Register (8-bits) **********************/

#define R_SYSTEM_VBTBPSR_VBPORF (1 <<  0)  /* 01: VBATT_POR Flag */
#define R_SYSTEM_VBTBPSR_VBPORM (1 <<  4)  /* 10: VBATT_POR Monitor */
#define R_SYSTEM_VBTBPSR_BPWSWM (1 <<  5)  /* 20: Battery Power Supply Switch Status Monitor */

/* VBATT Tamper detection Status Register (8-bits) **************************/

#define R_SYSTEM_VBTADSR_VBTADF0 (1 <<  0)  /* 01: VBATT Tamper Detection flag 0 */
#define R_SYSTEM_VBTADSR_VBTADF1 (1 <<  1)  /* 02: VBATT Tamper Detection flag 1 */
#define R_SYSTEM_VBTADSR_VBTADF2 (1 <<  2)  /* 04: VBATT Tamper Detection flag 2 */

/* VBATT Tamper detection Control Register 1 (8-bits) ***********************/

#define R_SYSTEM_VBTADCR1_VBTADIE0 (1 <<  0)   /* 01: VBATT Tamper Detection Interrupt Enable 0 */
#define R_SYSTEM_VBTADCR1_VBTADIE1 (1 <<  1)   /* 02: VBATT Tamper Detection Interrupt Enable 1 */
#define R_SYSTEM_VBTADCR1_VBTADIE2 (1 <<  2)   /* 04: VBATT Tamper Detection Interrupt Enable 2 */
#define R_SYSTEM_VBTADCR1_VBTADCLE0 (1 <<  4)  /* 10: VBATT Tamper Detection Backup Register Clear Enable 0 */
#define R_SYSTEM_VBTADCR1_VBTADCLE1 (1 <<  5)  /* 20: VBATT Tamper Detection Backup Register Clear Enable 1 */
#define R_SYSTEM_VBTADCR1_VBTADCLE2 (1 <<  6)  /* 40: VBATT Tamper Detection Backup Register Clear Enable 2 */

/* VBATT Tamper detection Control Register 2 (8-bits) ***********************/

#define R_SYSTEM_VBTADCR2_VBRTCES0 (1 <<  0)  /* 01: VBATT RTC Time Capture Event Source Select 0 */
#define R_SYSTEM_VBTADCR2_VBRTCES1 (1 <<  1)  /* 02: VBATT RTC Time Capture Event Source Select 1 */
#define R_SYSTEM_VBTADCR2_VBRTCES2 (1 <<  2)  /* 04: VBATT RTC Time Capture Event Source Select 2 */

/* VBATT Input Control Register (8-bits) ************************************/

#define R_SYSTEM_VBTICTLR_VCH0INEN (1 <<  0)  /* 01: RTCIC0 Input Enable */
#define R_SYSTEM_VBTICTLR_VCH1INEN (1 <<  1)  /* 02: RTCIC1 Input Enable */
#define R_SYSTEM_VBTICTLR_VCH2INEN (1 <<  2)  /* 04: RTCIC2 Input Enable */

/* VBATT Input Control Register 2 (8-bits) **********************************/

#define R_SYSTEM_VBTICTLR2_VCH0NCE (1 <<  0)  /* 01: VBATT CH0 Input Noise Canceler Enable */
#define R_SYSTEM_VBTICTLR2_VCH1NCE (1 <<  1)  /* 02: VBATT CH1 Input Noise Canceler Enable */
#define R_SYSTEM_VBTICTLR2_VCH2NCE (1 <<  2)  /* 04: VBATT CH2 Input Noise Canceler Enable */
#define R_SYSTEM_VBTICTLR2_VCH0EG (1 <<  4)   /* 10: VBATT CH0 Input Edge Select */
#define R_SYSTEM_VBTICTLR2_VCH1EG (1 <<  5)   /* 20: VBATT CH1 Input Edge Select */
#define R_SYSTEM_VBTICTLR2_VCH2EG (1 <<  6)   /* 40: VBATT CH2 Input Edge Select */

/* VBATT Input Monitor Register (8-bits) ************************************/

#define R_SYSTEM_VBTIMONR_VCH0MON (1 <<  0)  /* 01: VBATT CH0 Input monitor */
#define R_SYSTEM_VBTIMONR_VCH1MON (1 <<  1)  /* 02: VBATT CH1 Input monitor */
#define R_SYSTEM_VBTIMONR_VCH2MON (1 <<  2)  /* 04: VBATT CH2 Input monitor */

/* VBATT Backup Register %s (8-bits) ****************************************/

#define R_SYSTEM_VBTBKR_SIZE 128
#define R_SYSTEM_VBTBKR_VBTBKR_SHIFT (0)
#define R_SYSTEM_VBTBKR_VBTBKR_MASK (0xff)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_RA8M1_HARDWARE_RA8M1_SYSTEM_H */
