/****************************************************************************
 * arch/hc/src/m9s12/m9s12_crg.h
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

#ifndef __ARCH_HC_SRC_M9S12_M9S12_CRG_H
#define __ARCH_HC_SRC_M9S12_M9S12_CRG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CRG Module Register Offsets */

#define HCS12_CRG_SYNR_OFFSET   (HCS12_CRG_BASE+0x00) /* CRG Synthesizer Register */
#define HCS12_CRG_REFDV_OFFSET  (HCS12_CRG_BASE+0x01) /* CRG Reference Divider Register */
#define HCS12_CRG_CTFLG_OFFSET  (HCS12_CRG_BASE+0x02) /* CRG Test Flags Register */
#define HCS12_CRG_CRGFLG_OFFSET (HCS12_CRG_BASE+0x03) /* CRG Flags Register */
#define HCS12_CRG_CRGINT_OFFSET (HCS12_CRG_BASE+0x04) /* CRG Interrupt Enable Register */
#define HCS12_CRG_CLKSEL_OFFSET (HCS12_CRG_BASE+0x05) /* CRG Clock Select Register */
#define HCS12_CRG_PLLCTL_OFFSET (HCS12_CRG_BASE+0x06) /* CRG PLL Control Register */
#define HCS12_CRG_RTICTL_OFFSET (HCS12_CRG_BASE+0x07) /* CRG RTI Control Register */
#define HCS12_CRG_COPCTL_OFFSET (HCS12_CRG_BASE+0x08) /* CRG COP Control Register */
#define HCS12_CRG_FORBYP_OFFSET (HCS12_CRG_BASE+0x09) /* CRG Force and Bypass Test Register */
#define HCS12_CRG_CTCTL_OFFSET  (HCS12_CRG_BASE+0x0a) /* CRG Test Control Register */
#define HCS12_CRG_ARMCOP_OFFSET (HCS12_CRG_BASE+0x0b) /* CRG COP Arm/Timer Reset */

/* CRG Module Register Addresses */

#define HCS12_CRG_SYNR          (HCS12_REG_BASE+HCS12_CRG_SYNR_OFFSET)
#define HCS12_CRG_REFDV         (HCS12_REG_BASE+HCS12_CRG_REFDV_OFFSET)
#define HCS12_CRG_CTFLG         (HCS12_REG_BASE+HCS12_CRG_CTFLG_OFFSET)
#define HCS12_CRG_CRGFLG        (HCS12_REG_BASE+HCS12_CRG_CRGFLG_OFFSET)
#define HCS12_CRG_CRGINT        (HCS12_REG_BASE+HCS12_CRG_CRGINT_OFFSET)
#define HCS12_CRG_CLKSEL        (HCS12_REG_BASE+HCS12_CRG_CLKSEL_OFFSET)
#define HCS12_CRG_PLLCTL        (HCS12_REG_BASE+HCS12_CRG_PLLCTL_OFFSET)
#define HCS12_CRG_RTICTL        (HCS12_REG_BASE+HCS12_CRG_RTICTL_OFFSET)
#define HCS12_CRG_COPCTL        (HCS12_REG_BASE+HCS12_CRG_COPCTL_OFFSET)
#define HCS12_CRG_FORBYP        (HCS12_REG_BASE+HCS12_CRG_FORBYP_OFFSET)
#define HCS12_CRG_CTCTL         (HCS12_REG_BASE+HCS12_CRG_CTCTL_OFFSET)
#define HCS12_CRG_ARMCOP        (HCS12_REG_BASE+HCS12_CRG_ARMCOP_OFFSET)

/* CRG Module Register Bit Definitions */

#define CRG_SYNR_SHIFT          (0)  /* Bits 0-5: CRG synthesizer value */
#define CRG_SYNR_MASK           (0x3f << CRG_SYNR_SHIFT)

#define CRG_REFDV_SHIFT         (0)  /* Bit 0-3: Reference divider */
#define CRG_REFDV_MASK          (15 << CRG_REFDV_SHIFT)

#define CRG_CRGFLG_SCM          (1 << 0) /* Bit 0: Self-Clock Mode Status Bit */
#define CRG_CRGFLG_SCMIF        (1 << 1) /* Bit 1: Self-Clock Mode Interrupt Flag */
#define CRG_CRGFLG_TRACK        (1 << 2) /* Bit 2: Track Status Bit */
#define CRG_CRGFLG_LOCK         (1 << 3) /* Bit 3: Lock Status Bit */
#define CRG_CRGFLG_LOCKIF       (1 << 4) /* Bit 4: PLL Lock Interrupt Flag */
#define CRG_CRGFLG_LVRF         (1 << 5) /* Bit 5: Low Voltage Reset Flag */
#define CRG_CRGFLG_PORF         (1 << 6) /* Bit 6: Power-on Reset Flag */
#define CRG_CRGFLG_RTIF         (1 << 7) /* Bit 7: Real-Time Interrupt Flag */

#define CRG_CRGINT_SCMIE        (1 << 1) /* Bit 1: Self-Clock Mode Status Bit */
#define CRG_CRGINT_LOCKIE       (1 << 4) /* Bit 4: Lock Interrupt Enable Bit */
#define CRG_CRGINT_RTIE         (1 << 7) /* Bit 7: Lock Interrupt Enable Bit */

#define CRG_CLKSEL_COPWAI       (1 << 0) /* Bit 0: COP stops in Wait Mode Bit */
#define CRG_CLKSEL_RTIWAI       (1 << 1) /* Bit 1: RTI stops in Wait Mode Bit */
#define CRG_CLKSEL_CWAI         (1 << 2) /* Bit 2: Core stops in Wait Mode Bit */
#define CRG_CLKSEL_PLLWAI       (1 << 3) /* Bit 3: PLL stops in Wait Mode Bit */
#define CRG_CLKSEL_ROAWAI       (1 << 4) /* Bit 4: Reduced Oscillator Amplitude in Wait Mode Bit */
#define CRG_CLKSEL_SYSWAI       (1 << 5) /* Bit 5: System clocks stop in wait mode bit */
#define CRG_CLKSEL_PSTP         (1 << 6) /* Bit 6: Pseudo-Stop Bit */
#define CRG_CLKSEL_PLLSEL       (1 << 7) /* Bit 7: PLL Select Bit */

#define CRG_PLLCTL_SCME         (1 << 0) /* Bit 0: Self-Clock Mode Enable Bit */
#define CRG_PLLCTL_PCE          (1 << 1) /* Bit 1: COP Enable during Pseudo-Stop Bit */
#define CRG_PLLCTL_PRE          (1 << 2) /* Bit 2: RTI Enable during Pseudo-Stop Bit */
#define CRG_PLLCTL_ACQ          (1 << 4) /* Bit 4: Acquisition Bit */
#define CRG_PLLCTL_AUTO         (1 << 5) /* Bit 5: Automatic Bandwidth Control Bit */
#define CRG_PLLCTL_PLLON        (1 << 6) /* Bit 6: Phase Lock Loop On Bit */
#define CRG_PLLCTL_CME          (1 << 7) /* Bit 7: Clock Monitor Enable Bit */

#define CRG_RTICTL_MODCNT_SHIFT (0)      /* Bits 0-3: Real-Time Interrupt Modulus Counter Select Bits */
#define CRG_RTICTL_MODCNT_MASK  (15 << CRG_RTICTL_MODCNT_SHIFT)
#define CRG_RTICTL_PRER_SHIFT   (4)      /* Bits 4-6: Real-Time Interrupt Prescale Rate Select Bits */
#define CRG_RTICTL_PRER_MASK    (7 << CRG_RTICTL_PRE_SHIFT)

#define CRG_COPCTL_CR_SHIFT     (0)      /* Bits 0-2: COP Watchdog Timer Rate select */
#define CRG_COPCTL_CR_MASK      (7 << CRG_COPCTL_CR_SHIFT)
#define CRG_COPCTL_RSBCK        (1 << 6) /* Bit 6: COP and RTI stop in Active BDM mode B */
#define CRG_COPCTL_WCOP         (1 << 7) /* Bit 7: Window COP Mode Bit */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_HC_SRC_M9S12_M9S12_CRG_H */
