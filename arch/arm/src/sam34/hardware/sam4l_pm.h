/************************************************************************************
 * arch/arm/src/sam34/hardware/sam4l_pm.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This file is derived from nuttx/arch/avr/src/at32uc3/at32uc3_pm.h.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAM34_HARDWARE_SAM4L_PM_H
#define __ARCH_ARM_SRC_SAM34_HARDWARE_SAM4L_PM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define SAM_PM_MCCTRL_OFFSET       0x0000 /* Main Clock Control Register */
#define SAM_PM_CPUSEL_OFFSET       0x0004 /* CPU Clock Select Register */
#define SAM_PM_PBASEL_OFFSET       0x000c /* PBA Clock Select Register */
#define SAM_PM_PBBSEL_OFFSET       0x0010 /* PBB Clock Select Register */
#define SAM_PM_PBCSEL_OFFSET       0x0014 /* PBC Clock Select Register */
#define SAM_PM_PBDSEL_OFFSET       0x0018 /* PBD Clock Select Register */
#define SAM_PM_CPUMASK_OFFSET      0x0020 /* CPU Mask Register */
#define SAM_PM_HSBMASK_OFFSET      0x0024 /* HSB Mask Register */
#define SAM_PM_PBAMASK_OFFSET      0x0028 /* PBA Mask Register */
#define SAM_PM_PBBMASK_OFFSET      0x002c /* PBB Mask Register */
#define SAM_PM_PBCMASK_OFFSET      0x0030 /* PBC Mask Register */
#define SAM_PM_PBDMASK_OFFSET      0x0034 /* PBD Mask Register */
#define SAM_PM_PBADIVMASK_OFFSET   0x0040 /* PBA Divided Mask */
#define SAM_PM_CFDCTRL_OFFSET      0x0054 /* Clock Failure Detector Control */
#define SAM_PM_UNLOCK_OFFSET       0x0058 /*  Unlock Register */
#define SAM_PM_IER_OFFSET          0x00c0 /* Interrupt Enable Register */
#define SAM_PM_IDR_OFFSET          0x00c4 /* Interrupt Disable Register */
#define SAM_PM_IMR_OFFSET          0x00c8 /* Interrupt Mask Register */
#define SAM_PM_ISR_OFFSET          0x00cc /* Interrupt Status Register */
#define SAM_PM_ICR_OFFSET          0x00d0 /* Interrupt Clear Register */
#define SAM_PM_SR_OFFSET           0x00d4 /* Status Register Register */
#define SAM_PM_PPCR_OFFSET         0x0160 /* Peripheral Power Control Register */
#define SAM_PM_RCAUSE_OFFSET       0x0180 /* Reset Cause Register */
#define SAM_PM_WCAUSE_OFFSET       0x0184 /* Wake Cause Register */
#define SAM_PM_AWEN_OFFSET         0x0188 /* Asynchronous Wake Up Enable Register */
#define SAM_PM_PROTCTRL_OFFSET     0x018c /* Protection Control Register */
#define SAM_PM_FASTSLEEP_OFFSET    0x0194 /* Fast Sleep Register */
#define SAM_PM_CONFIG_OFFSET       0x03f8 /* Configuration Register */
#define SAM_PM_VERSION_OFFSET      0x03fc /* Version Register */

/* Register Addresses ***************************************************************/

#define SAM_PM_MCCTRL              (SAM_PM_BASE+SAM_PM_MCCTRL_OFFSET)
#define SAM_PM_CPUSEL              (SAM_PM_BASE+SAM_PM_CPUSEL_OFFSET)
#define SAM_PM_PBASEL              (SAM_PM_BASE+SAM_PM_PBASEL_OFFSET)
#define SAM_PM_PBBSEL              (SAM_PM_BASE+SAM_PM_PBBSEL_OFFSET)
#define SAM_PM_PBCSEL              (SAM_PM_BASE+SAM_PM_PBCSEL_OFFSET)
#define SAM_PM_PBDSEL              (SAM_PM_BASE+SAM_PM_PBDSEL_OFFSET)
#define SAM_PM_CPUMASK             (SAM_PM_BASE+SAM_PM_CPUMASK_OFFSET)
#define SAM_PM_HSBMASK             (SAM_PM_BASE+SAM_PM_HSBMASK_OFFSET)
#define SAM_PM_PBAMASK             (SAM_PM_BASE+SAM_PM_PBAMASK_OFFSET)
#define SAM_PM_PBBMASK             (SAM_PM_BASE+SAM_PM_PBBMASK_OFFSET)
#define SAM_PM_PBCMASK             (SAM_PM_BASE+SAM_PM_PBCMASK_OFFSET)
#define SAM_PM_PBDMASK             (SAM_PM_BASE+SAM_PM_PBDMASK_OFFSET)
#define SAM_PM_PBADIVMASK          (SAM_PM_BASE+SAM_PM_PBADIVMASK_OFFSET)
#define SAM_PM_CFDCTRL             (SAM_PM_BASE+SAM_PM_CFDCTRL_OFFSET)
#define SAM_PM_UNLOCK              (SAM_PM_BASE+SAM_PM_UNLOCK_OFFSET)
#define SAM_PM_IER                 (SAM_PM_BASE+SAM_PM_IER_OFFSET)
#define SAM_PM_IDR                 (SAM_PM_BASE+SAM_PM_IDR_OFFSET)
#define SAM_PM_IMR                 (SAM_PM_BASE+SAM_PM_IMR_OFFSET)
#define SAM_PM_ISR                 (SAM_PM_BASE+SAM_PM_ISR_OFFSET)
#define SAM_PM_ICR                 (SAM_PM_BASE+SAM_PM_ICR_OFFSET)
#define SAM_PM_SR                  (SAM_PM_BASE+SAM_PM_SR_OFFSET)
#define SAM_PM_PPCR                (SAM_PM_BASE+SAM_PM_PPCR_OFFSET)
#define SAM_PM_RCAUSE              (SAM_PM_BASE+SAM_PM_RCAUSE_OFFSET)
#define SAM_PM_WCAUSE              (SAM_PM_BASE+SAM_PM_WCAUSE_OFFSET)
#define SAM_PM_AWEN                (SAM_PM_BASE+SAM_PM_AWEN_OFFSET)
#define SAM_PM_PROTCTRL            (SAM_PM_BASE+SAM_PM_PROTCTRL_OFFSET)
#define SAM_PM_FASTSLEEP           (SAM_PM_BASE+SAM_PM_FASTSLEEP_OFFSET)
#define SAM_PM_CONFIG              (SAM_PM_BASE+SAM_PM_CONFIG_OFFSET)
#define SAM_PM_VERSION             (SAM_PM_BASE+SAM_PM_VERSION_OFFSET)

/* Register Bit-field Definitions ***************************************************/

/* Main Clock Control Register Bit-field Definitions */

#define PM_MCCTRL_MCSEL_SHIFT      (0)       /* Bits 0-2: Main Clock Select */
#define PM_MCCTRL_MCSEL_MASK       (7 << PM_MCCTRL_MCSEL_SHIFT)
#  define PM_MCCTRL_MCSEL_RCSYS    (0 << PM_MCCTRL_MCSEL_SHIFT) /* System RC oscillator */
#  define PM_MCCTRL_MCSEL_OSC0     (1 << PM_MCCTRL_MCSEL_SHIFT) /* Oscillator0 */
#  define PM_MCCTRL_MCSEL_PLL      (2 << PM_MCCTRL_MCSEL_SHIFT) /* PLL */
#  define PM_MCCTRL_MCSEL_DFLL     (3 << PM_MCCTRL_MCSEL_SHIFT) /* DFLL */
#  define PM_MCCTRL_MCSEL_RC80M    (4 << PM_MCCTRL_MCSEL_SHIFT) /* 80MHz RC oscillator */
#  define PM_MCCTRL_MCSEL_RCFAST   (5 << PM_MCCTRL_MCSEL_SHIFT) /* 4/8/12 MHz RC oscillator */
#  define PM_MCCTRL_MCSEL_RC1M     (6 << PM_MCCTRL_MCSEL_SHIFT) /* 1 MHz RC oscillator */

/* CPU Clock Select Register Bit-field Definitions */

#define PM_CPUSEL_SHIFT            (0)       /* Bits 0-2: CPU Clock Select */
#define PM_CPUSEL_MASK             (7 << PM_CPUSEL_SHIFT)
#  define PM_CPUSEL(n)             ((n) << PM_CPUSEL_SHIFT)
#define PM_CPUSEL_DIV              (1 << 7)  /* Bit 7:  CPU Division */

/* PBA/PBB/PBC/PBD Clock Select Register Bit-field Definitions */

#define PM_PBSEL_SHIFT             (0)       /* Bits 0-2: PBx Clock Select */
#define PM_PBSEL_MASK              (7 << PM_PBSEL_SHIFT)
#  define PM_PBSEL(n)              ((n) << PM_PBSEL_SHIFT)
#define PM_PBSEL_DIV               (1 << 7)  /* Bit 7: PBx Division */

/* CPU Mask Register Bit-field Definitions */

#define PM_CPUMASK_OCD             (1 << 0)  /* Bit 0:  On-Chip Debug */

/* HSB Mask Register Bit-field Definitions */

#define PM_HSBMASK_PDCA            (1 << 0)  /* Bit 0:  PDCA */
#define PM_HSBMASK_FLASHCALW       (1 << 1)  /* Bit 1:  FLASHCALW */
#define PM_HSBMASK_HRAMC1          (1 << 2)  /* Bit 2:  HRAMC1 (picoCache RAM) */
#define PM_HSBMASK_USBC            (1 << 3)  /* Bit 3:  USBC */
#define PM_HSBMASK_CRCCU           (1 << 4)  /* Bit 4:  CRCCU */
#define PM_HSBMASK_APBA            (1 << 5)  /* Bit 5:  APBA bridge */
#define PM_HSBMASK_APBB            (1 << 6)  /* Bit 5:  APBB bridge */
#define PM_HSBMASK_APBC            (1 << 7)  /* Bit 5:  APBC bridge */
#define PM_HSBMASK_APBD            (1 << 8)  /* Bit 5:  APBD bridge */
#define PM_HSBMASK_AESA            (1 << 9)  /* Bit 5:  AESA */

/* PBA Mask Register Bit-field Definitions */

#define PM_PBAMASK_IISC            (1 << 0)  /* Bit 0:  IISC */
#define PM_PBAMASK_SPI             (1 << 1)  /* Bit 1:  SPI */
#define PM_PBAMASK_TC0             (1 << 2)  /* Bit 2:  TC0 */
#define PM_PBAMASK_TC1             (1 << 3)  /* Bit 3:  TC1 */
#define PM_PBAMASK_TWIM0           (1 << 4)  /* Bit 4:  TWIM0 */
#define PM_PBAMASK_TWIS0           (1 << 5)  /* Bit 5:  TWIS0 */
#define PM_PBAMASK_TWIM1           (1 << 6)  /* Bit 6:  TWIM1 */
#define PM_PBAMASK_TWIS1           (1 << 7)  /* Bit 7:  TWIS1 */
#define PM_PBAMASK_USART0          (1 << 8)  /* Bit 8:  USART0 */
#define PM_PBAMASK_USART1          (1 << 9)  /* Bit 9:  USART1 */
#define PM_PBAMASK_USART2          (1 << 10) /* Bit 10: USART2 */
#define PM_PBAMASK_USART3          (1 << 11) /* Bit 11: USART3 */
#define PM_PBAMASK_ADCIFE          (1 << 12) /* Bit 12: ADCIFE */
#define PM_PBAMASK_DACC            (1 << 13) /* Bit 13: DACC */
#define PM_PBAMASK_ACIFC           (1 << 14) /* Bit 14: ACIFC */
#define PM_PBAMASK_GLOC            (1 << 15) /* Bit 15: GLOC */
#define PM_PBAMASK_ABDACB          (1 << 16) /* Bit 16: ABDACB */
#define PM_PBAMASK_TRNG            (1 << 17) /* Bit 17: TRNG */
#define PM_PBAMASK_PARC            (1 << 18) /* Bit 18: PARC */
#define PM_PBAMASK_CATB            (1 << 19) /* Bit 19: CATB */
#define PM_PBAMASK_TWIM2           (1 << 21) /* Bit 21: TWIM2 */
#define PM_PBAMASK_TWIM3           (1 << 22) /* Bit 22: TWIM3 */
#define PM_PBAMASK_LCDCA           (1 << 23) /* Bit 23: LCDCA*/

/* These are the PBMA peripherals that use divided clocks enabled in the
 * PBADIVMASK register.
 */

#define PM_PBAMASK_TIMERS (PM_PBAMASK_TC0 | PM_PBAMASK_TC1)
#define PM_PBAMASK_UARTS  (PM_PBAMASK_USART0 | PM_PBAMASK_USART1 | \
                           PM_PBAMASK_USART2 | PM_PBAMASK_USART3)

/* PBB Mask Register Bit-field Definitions */

#define PM_PBBMASK_FLASHCALW       (1 << 0)  /* Bit 0:  FLASHCALW */
#define PM_PBBMASK_HRAMC1          (1 << 1)  /* Bit 1:  HRAMC1 */
#define PM_PBBMASK_HMATRIX         (1 << 2)  /* Bit 2:  HMATRIX */
#define PM_PBBMASK_PDCA            (1 << 3)  /* Bit 3:  PDCA */
#define PM_PBBMASK_CRCCU           (1 << 4)  /* Bit 4:  CRCCU */
#define PM_PBBMASK_USBC            (1 << 5)  /* Bit 5:  USBC */
#define PM_PBBMASK_PEVC            (1 << 6)  /* Bit 6:  PEVC */

/* PBC Mask Register Bit-field Definitions */

#define PM_PBCMASK_PM              (1 << 0)  /* Bit 0:  PM */
#define PM_PBCMASK_CHIPID          (1 << 1)  /* Bit 1:  CHIPID */
#define PM_PBCMASK_SCIF            (1 << 2)  /* Bit 2:  SCIF */
#define PM_PBCMASK_FREQM           (1 << 3)  /* Bit 3:  FREQM */
#define PM_PBCMASK_GPIO            (1 << 4)  /* Bit 4:  GPIO */

/* PBD Mask Register Bit-field Definitions */

#define PM_PBDMASK_BPM             (1 << 0)  /* Bit 0:  BPM */
#define PM_PBDMASK_BSCIF           (1 << 1)  /* Bit 1:  BSCIF */
#define PM_PBDMASK_AST             (1 << 2)  /* Bit 2:  AST */
#define PM_PBDMASK_WDT             (1 << 3)  /* Bit 3:  WDT */
#define PM_PBDMASK_EIC             (1 << 4)  /* Bit 4:  EIC */
#define PM_PBDMASK_PICOUART        (1 << 5)  /* Bit 5:  PICOUART */

/* PBA Divided Mask */

#define PM_PBADIVMASK_TIMER_CLOCK2 (1 << 0) /* Bit 0: TIMER_CLOCK2 (TC0-1) */
#define PM_PBADIVMASK_CLK_USART    (1 << 2) /* Bit 2: CLK_USART/DIV (USART0-3) */
#define PM_PBADIVMASK_TIMER_CLOCK3 (1 << 2) /* Bit 2: TIMER_CLOCK3 (TC0-1) */
#define PM_PBADIVMASK_TIMER_CLOCK4 (1 << 4) /* Bit 4: TIMER_CLOCK4 (TC0-1) */
#define PM_PBADIVMASK_TIMER_CLOCK5 (1 << 6) /* Bit 5: TIMER_CLOCK5 (TC0-1) */

#define PM_PBADIVMASK_TIMER_CLOCKS \
  (PM_PBADIVMASK_TIMER_CLOCK2 | PM_PBADIVMASK_TIMER_CLOCK3 | \
   PM_PBADIVMASK_TIMER_CLOCK4 | PM_PBADIVMASK_TIMER_CLOCK5)

/* Clock Failure Detector Control */

#define PM_CFDCTRL_CFDEN           (1 << 0)  /* Bit 0:  Clock Failure Detection Enable */
#define PM_CFDCTRL_SFV             (1 << 31) /* Bit 31: Store Final Value */

/* Unlock Register */

#define PM_UNLOCK_ADDR_SHIFT       (0)       /* Bits 0-9: Unlock Address */
#define PM_UNLOCK_ADDR_MASK        (0x3ff << PM_UNLOCK_ADDR_SHIFT)
#  define PM_UNLOCK_ADDR(n)        ((n) << PM_UNLOCK_ADDR_SHIFT)
#define PM_UNLOCK_KEY_SHIFT        (24)      /* Bits 24-31: Unlock Key */
#define PM_UNLOCK_KEY_MASK         (0xff << PM_UNLOCK_KEY_SHIFT)
#  define PM_UNLOCK_KEY(n)         ((n) << PM_UNLOCK_KEY_SHIFT)

/* Interrupt Enable Register Bit-field Definitions */
/* Interrupt Disable Register Bit-field Definitions */
/* Interrupt Mask Register Bit-field Definitions */
/* Interrupt Status Register Bit-field Definitions */
/* Interrupt Clear Register Bit-field Definitions */
/* Status Register Register */

#define PM_INT_CFD                (1 << 0)  /* Bit 0:  CFD */
#define PM_INT_CKRDY              (1 << 5)  /* Bit 5:  CKRDY */
#define PM_INT_WAKE               (1 << 8)  /* Bit 8:  WAKE */

/* Peripheral Power Control Register */

#define PM_PPCR_RSTPUN            (1 << 0)  /* Bit 0:  Reset Pullup */
#define PM_PPCR_CATBRCMASK        (1 << 1)  /* Bit 1:  CAT Request Clock Mask */
#define PM_PPCR_ACIFCRCMASK       (1 << 2)  /* Bit 2:  ACIFC Request Clock Mask */
#define PM_PPCR_ASTRCMASK         (1 << 3)  /* Bit 3:  AST Request Clock Mask */
#define PM_PPCR_TWIS0RCMASK       (1 << 4)  /* Bit 4:  TWIS0 Request Clock Mask */
#define PM_PPCR_TWIS1RCMASK       (1 << 5)  /* Bit 5:  TWIS1 Request Clock Mask */
#define PM_PPCR_PEVCRCMASK        (1 << 6)  /* Bit 6:  PEVC Request Clock Mask */
#define PM_PPCR_ADCIFERCMASK      (1 << 7)  /* Bit 7:  ADCIFE Request Clock Mask */
#define PM_PPCR_VREGRCMASK        (1 << 8)  /* Bit 8:  VREG Request Clock Mask */
#define PM_PPCR_FWBGREF           (1 << 9)  /* Bit 9:  Flash Wait BGREF */
#define PM_PPCR_FWBOD18           (1 << 10) /* Bit 10: Flash Wait BOD18 */

/* Reset Cause Register */

#define PM_RCAUSE_POR                 (1 << 0)  /* Bit 0:  Power-on Reset */
#define PM_RCAUSE_BOD                 (1 << 1)  /* Bit 1:  Brown-out Reset */
#define PM_RCAUSE_EXT                 (1 << 2)  /* Bit 2:  External Reset Pin */
#define PM_RCAUSE_WDT                 (1 << 3)  /* Bit 3:  Watchdog Reset */
#define PM_RCAUSE_BKUP                (1 << 6)  /* Bit 6:  Backup reset */
#define PM_RCAUSE_OCDRST              (1 << 8)  /* Bit 8:  OCD Reset */
#define PM_RCAUSE_POR33               (1 << 10) /* Bit 10: Power-on 3.3v Reset */
#define PM_RCAUSE_BOD33               (1 << 13) /* Bit 13: Brown-out 3.3v Reset */

/* Wake Cause Register */

#define PM_WCAUSE_TWIS0               (1 << 0)  /* Bit 0:  0 TWI Slave 0 */
#define PM_WCAUSE_TWIS1               (1 << 1)  /* Bit 1:  1 TWI Slave 1 */
#define PM_WCAUSE_USBC                (1 << 2)  /* Bit 2:  2 USBC */
#define PM_WCAUSE_PSOK                (1 << 3)  /* Bit 3:  3 PSOK */
#define PM_WCAUSE_BOD18               (1 << 4)  /* Bit 4:  4 BOD18 IRQ */
#define PM_WCAUSE_BOD33               (1 << 5)  /* Bit 5:  5 BOD33 IRQ */
#define PM_WCAUSE_PICOUART            (1 << 6)  /* Bit 6:  6 PICOUART */
#define PM_WCAUSE_LCDCA               (1 << 7)  /* Bit 7:  7 LCDCA */
#define PM_WCAUSE_EIC                 (1 << 16) /* Bit 16: 16 EIC */
#define PM_WCAUSE_AST                 (1 << 17) /* Bit 17: 17 AST */

/* Asynchronous Wake Up Enable Register Bit-field Definitions */

#define PM_AWEN_TWIS0                 (1 << 0)  /* Bit 0:  TWI Slave 0 */
#define PM_AWEN_TWIS1                 (1 << 1)  /* Bit 1:  TWI Slave 1 */
#define PM_AWEN_USBC                  (1 << 2)  /* Bit 2:  USBC */
#define PM_AWEN_PSOK                  (1 << 3)  /* Bit 3:  PSOK */
#define PM_AWEN_BOD18                 (1 << 4)  /* Bit 4:  BOD18 IRQ */
#define PM_AWEN_BOD33                 (1 << 5)  /* Bit 5:  BOD33 IRQ */
#define PM_AWEN_PICOUART              (1 << 6)  /* Bit 6:  PICOUART */
#define PM_AWEN_LCDCA                 (1 << 7)  /* Bit 7:  LCDCA */

/* Protection Control Register */

/* Fast Sleep Register */
#define PM_FASTSLEEP_
#define PM_FASTSLEEP_OSC              (1 << 0)  /* Bit 0:  Oscillator */
#define PM_FASTSLEEP_PLL              (1 << 0)  /* Bit 0:  PLL */
#define PM_FASTSLEEP_FASTRCOSC_SHIFT  (0)       /* Bits 0-9: FASTRCOSC */
#define PM_FASTSLEEP_FASTRCOSC_MASK   (31 << PM_FASTSLEEP_FASTRCOSC_SHIFT)
#  define PM_FASTSLEEP_RC80           (1 << PM_FASTSLEEP_FASTRCOSC_SHIFT)
#  define PM_FASTSLEEP_RCFAST         (2 << PM_FASTSLEEP_FASTRCOSC_SHIFT)
#  define PM_FASTSLEEP_RC1M           (4 << PM_FASTSLEEP_FASTRCOSC_SHIFT)
#define PM_FASTSLEEP_DFLL             (1 << 0)  /* Bit 0:  DFLL */

/* Configuration Register */

#define PM_CONFIG_PBA                 (1 << 0)  /* Bit 0:  APBA Implemented */
#define PM_CONFIG_PBB                 (1 << 1)  /* Bit 1:  APBB Implemented */
#define PM_CONFIG_PBC                 (1 << 2)  /* Bit 2:  APBC Implemented */
#define PM_CONFIG_PBD                 (1 << 3)  /* Bit 3:  APBD Implemented */
#define PM_CONFIG_HSBPEVC             (1 << 7)  /* Bit 7:  HSB PEVC Clock Implemented */

/* Version Register */

#define PM_VERSION_SHIFT              (0)        /* Bits 0-11: Version Number */
#define PM_VERSION_MASK               (0xfff << PM_VERSION_VERSION_SHIFT)
#define PM_VERSION_VARIANT_SHIFT      (16)       /* Bits 16-19: Variant Number */
#define PM_VERSION_VARIANT_MASK       (15 << PM_VERSION_VARIANT_SHIFT)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM34_HARDWARE_SAM4L_PM_H */
