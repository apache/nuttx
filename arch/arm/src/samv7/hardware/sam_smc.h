/****************************************************************************************
 * arch/arm/src/samv7/hardware/sam_smc.h
 * Static Memory Controller (SMC) definitions for the SAMV71
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_SMC_H
#define __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_SMC_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>
#include <arch/samv7/chip.h>

#include "hardware/sam_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* SMC register offsets *****************************************************************/

#define SAM_SMCCS_OFFSET(n)            ((n) << 4)
#  define SAM_SMCCS0_OFFSET            0x0000 /* SMC CS0 offset */
#  define SAM_SMCCS1_OFFSET            0x0010 /* SMC CS1 offset */
#  define SAM_SMCCS2_OFFSET            0x0020 /* SMC CS2 offset */
#  define SAM_SMCCS3_OFFSET            0x0030 /* SMC CS3 offset */

#define SAM_SMCCS_SETUP_OFFSET         0x0000 /* SMC Setup Register */
#define SAM_SMCCS_PULSE_OFFSET         0x0004 /* SMC Pulse Register */
#define SAM_SMCCS_CYCLE_OFFSET         0x0008 /* SMC Cycle Register */
#define SAM_SMCCS_MODE_OFFSET          0x000c /* SMC Mode Register */

#define SAM_SMC_OCMS_OFFSET            0x0080 /* SMC OCMS Mode Register */
#define SAM_SMC_KEY1_OFFSET            0x0084 /* SMC KEY1 Register */
#define SAM_SMC_KEY2_OFFSET            0x0088 /* SMC KEY2 Register */
#define SAM_SMC_WPCR_OFFSET            0x00e4 /* Write Protection Control Register */
#define SAM_SMC_WPSR_OFFSET            0x00e8 /* Write Protection Status Register */

/* SMC register addresses ***************************************************************/

#define SAM_SMCCS_BASE(n)              (SAM_SMC_BASE+SAM_SMCCS_OFFSET(n))
#  define SAM_SMC_CS0_BASE             (SAM_SMC_BASE+SAM_SMCCS0_OFFSET)
#  define SAM_SMC_CS1_BASE             (SAM_SMC_BASE+SAM_SMCCS1_OFFSET)
#  define SAM_SMC_CS2_BASE             (SAM_SMC_BASE+SAM_SMCCS2_OFFSET)
#  define SAM_SMC_CS3_BASE             (SAM_SMC_BASE+SAM_SMCCS3_OFFSET)

#define SAM_SMCCS_SETUP(n)             (SAM_SMCCS_BASE(n)+SAM_SMCCS_SETUP_OFFSET)
#define SAM_SMCCS_PULSE(n)             (SAM_SMCCS_BASE(n)+SAM_SMCCS_PULSE_OFFSET)
#define SAM_SMCCS_CYCLE(n)             (SAM_SMCCS_BASE(n)+SAM_SMCCS_CYCLE_OFFSET)
#define SAM_SMCCS_MODE(n)              (SAM_SMCCS_BASE(n)+SAM_SMCCS_MODE_OFFSET)

#define SAM_SMCCS0_SETUP               (SAM_SMC_CS0_BASE+SAM_SMCCS_SETUP_OFFSET)
#define SAM_SMCCS0_PULSE               (SAM_SMC_CS0_BASE+SAM_SMCCS_PULSE_OFFSET)
#define SAM_SMCCS0_CYCLE               (SAM_SMC_CS0_BASE+SAM_SMCCS_CYCLE_OFFSET)
#define SAM_SMCCS0_MODE                (SAM_SMC_CS0_BASE+SAM_SMCCS_MODE_OFFSET)

#define SAM_SMCCS1_SETUP               (SAM_SMC_CS1_BASE+SAM_SMCCS_SETUP_OFFSET)
#define SAM_SMCCS1_PULSE               (SAM_SMC_CS1_BASE+SAM_SMCCS_PULSE_OFFSET)
#define SAM_SMCCS1_CYCLE               (SAM_SMC_CS1_BASE+SAM_SMCCS_CYCLE_OFFSET)
#define SAM_SMCCS1_MODE                (SAM_SMC_CS1_BASE+SAM_SMCCS_MODE_OFFSET)

#define SAM_SMCCS2_SETUP               (SAM_SMC_CS2_BASE+SAM_SMCCS_SETUP_OFFSET)
#define SAM_SMCCS2_PULSE               (SAM_SMC_CS2_BASE+SAM_SMCCS_PULSE_OFFSET)
#define SAM_SMCCS2_CYCLE               (SAM_SMC_CS2_BASE+SAM_SMCCS_CYCLE_OFFSET)
#define SAM_SMCCS2_MODE                (SAM_SMC_CS2_BASE+SAM_SMCCS_MODE_OFFSET)

#define SAM_SMCCS3_SETUP               (SAM_SMC_CS3_BASE+SAM_SMCCS_SETUP_OFFSET)
#define SAM_SMCCS3_PULSE               (SAM_SMC_CS3_BASE+SAM_SMCCS_PULSE_OFFSET)
#define SAM_SMCCS3_CYCLE               (SAM_SMC_CS3_BASE+SAM_SMCCS_CYCLE_OFFSET)
#define SAM_SMCCS3_MODE                (SAM_SMC_CS3_BASE+SAM_SMCCS_MODE_OFFSET)

#define SAM_SMC_OCMS                   (SAM_SMC_BASE+SAM_SMC_OCMS_OFFSET)
#define SAM_SMC_KEY1                   (SAM_SMC_BASE+SAM_SMC_KEY1_OFFSET)
#define SAM_SMC_KEY2                   (SAM_SMC_BASE+SAM_SMC_KEY2_OFFSET)
#define SAM_SMC_WPCR                   (SAM_SMC_BASE+SAM_SMC_WPCR_OFFSET)
#define SAM_SMC_WPSR                   (SAM_SMC_BASE+SAM_SMC_WPSR_OFFSET)

/* SMC register bit definitions *********************************************************/

/* SMC Setup Register */

#define SMCCS_SETUP_NWESETUP_SHIFT     (0)       /* Bits 0-5: NWE Setup length */
#define SMCCS_SETUP_NWESETUP_MASK      (63 << SMCCS_SETUP_NWESETUP_SHIFT)
#  define SMCCS_SETUP_NWESETUP(n)      ((n) << SMCCS_SETUP_NWESETUP_SHIFT)
#define SMCCS_SETUP_NCSWRSETUP_SHIFT   (8)       /* Bits 8-13: NCS Setup length in Write access */
#define SMCCS_SETUP_NCSWRSETUP_MASK    (63 << SMCCS_SETUP_NCSWRSETUP_SHIFT)
#  define SMCCS_SETUP_NCSWRSETUP(n)    ((n) << SMCCS_SETUP_NCSWRSETUP_SHIFT)
#define SMCCS_SETUP_NRDSETUP_SHIFT     (16)      /* Bits 16-21: NRD Setup length */
#define SMCCS_SETUP_NRDSETUP_MASK      (63 << SMCCS_SETUP_NRDSETUP_SHIFT)
#  define SMCCS_SETUP_NRDSETUP(n)      ((n) << SMCCS_SETUP_NRDSETUP_SHIFT)
#define SMCCS_SETUP_NCSRDSETUP_SHIFT   (24)      /* Bits 24-29: NCS Setup length in Read access */
#define SMCCS_SETUP_NCSRDSETUP_MASK    (63 << SMCCS_SETUP_NCSRDSETUP_SHIFT)
#  define SMCCS_SETUP_NCSRDSETUP(n)    ((n) << SMCCS_SETUP_NCSRDSETUP_SHIFT)

/* SMC Pulse Register */

#define SMCCS_PULSE_NWEPULSE_SHIFT     (0)       /* Bits 0-6: NWE Pulse Length */
#define SMCCS_PULSE_NWEPULSE_MASK      (127 << SMCCS_PULSE_NWEPULSE_SHIFT)
#  define SMCCS_PULSE_NWEPULSE(n)      ((n) << SMCCS_PULSE_NWEPULSE_SHIFT)
#define SMCCS_PULSE_NCSWRPULSE_SHIFT   (8)       /* Bits 8-14: NCS Pulse Length in WRITE Access */
#define SMCCS_PULSE_NCSWRPULSE_MASK    (127 << SMCCS_PULSE_NCSWRPULSE_SHIFT)
#  define SMCCS_PULSE_NCSWRPULSE(n)    ((n) << SMCCS_PULSE_NCSWRPULSE_SHIFT)
#define SMCCS_PULSE_NRDPULSE_SHIFT     (16)      /* Bits 16-22: NRD Pulse Length */
#define SMCCS_PULSE_NRDPULSE_MASK      (127 << SMCCS_PULSE_NRDPULSE_SHIFT)
#  define SMCCS_PULSE_NRDPULSE(n)      ((n) << SMCCS_PULSE_NRDPULSE_SHIFT)
#define SMCCS_PULSE_NCSRDPULSE_SHIFT   (24)      /* Bits 24-30: NCS Pulse Length in READ Access */
#define SMCCS_PULSE_NCSRDPULSE_MASK    (127 << SMCCS_PULSE_NCSRDPULSE_SHIFT)
#  define SMCCS_PULSE_NCSRDPULSE(n)    ((n) << SMCCS_PULSE_NCSRDPULSE_SHIFT)

/* SMC Cycle Register */

#define SMCCS_CYCLE_NWECYCLE_SHIFT     (0)       /* Bits 0-8: Total Write Cycle Length */
#define SMCCS_CYCLE_NWECYCLE_MASK      (0x1ff << SMCCS_CYCLE_NWECYCLE_SHIFT)
#  define SMCCS_CYCLE_NWECYCLE(n)      ((n) << SMCCS_CYCLE_NWECYCLE_SHIFT)
#define SMCCS_CYCLE_NRDCYCLE_SHIFT     (16)      /* Bits 16-24: Total Read Cycle Length */
#define SMCCS_CYCLE_NRDCYCLE_MASK      (0x1ff << SMCCS_CYCLE_NRDCYCLE_SHIFT)
#  define SMCCS_CYCLE_NRDCYCLE(n)      ((n) << SMCCS_CYCLE_NRDCYCLE_SHIFT)

/* SMC Mode Register */

#define SMCCS_MODE_READMODE            (1 << 0)  /* Bit 0: Read mode */
#define SMCCS_MODE_WRITEMODE           (1 << 1)  /* Bit 1: Write mode */
#define SMCCS_MODE_EXNWMODE_SHIFT      (4)       /* Bits 4-5: NWAIT Mode */
#define SMCCS_MODE_EXNWMODE_MASK       (3 << SMCCS_MODE_EXNWMODE_SHIFT)
#  define SMCCS_EXNWMODE_DISABLED      (0 << SMCCS_MODE_EXNWMODE_SHIFT)
#  define SMCCS_EXNWMODE_FROZEN        (2 << SMCCS_MODE_EXNWMODE_SHIFT)
#  define SMCCS_EXNWMODE_READY         (3 << SMCCS_MODE_EXNWMODE_SHIFT)
#define SMCCS_MODE_BAT                 (1 << 8)  /* Bit 8:  Byte Access Type */
#define SMCCS_MODE_DBW_MASK            (1 << 12) /* Bit 12: Data Bus Width */
#  define SMCCS_MODE_DBW_8BIT          (0 << 12) /*         0=8-bit data bus */
#  define SMCCS_MODE_DBW_16BIT         (1 << 12) /*         1=16-bit data bus */
#define SMCCS_MODE_TDFCYCLES_SHIFT     (16)      /* Bits 16-19: Data Float Time */
#define SMCCS_MODE_TDFCYCLES_MASK      (15 << SMCCS_MODE_TDFCYCLES_SHIFT)
#  define SMCCS_MODE_TDFCYCLES(n)      ((uint32_t)(n) << SMCCS_MODE_TDFCYCLES_SHIFT)
#define SMCCS_MODE_TDFMODE             (1 << 20) /* Bit 20: TDF Optimization */
#define SMCCS_MODE_PMEN                (1 << 24) /* Bit 24: Page Mode Enabled */
#define SMCCS_MODE_PS_SHIFT            (28) /* Bits 28-29: Page Size */
#define SMCCS_MODE_PS_MASK             (3 << SMCCS_MODE_PS_SHIFT)
#  define SMCCS_MODE_PS_SIZE_4BYTES    (0 << SMCCS_MODE_PS_SHIFT) /* 4 bytes */
#  define SMCCS_MODE_PS_SIZE_8BYTES    (1 << SMCCS_MODE_PS_SHIFT) /* 8 bytes */
#  define SMCCS_MODE_PS_SIZE_16BYTES   (2 << SMCCS_MODE_PS_SHIFT) /* 16 bytes */
#  define SMCCS_MODE_PS_SIZE_32BYTES   (3 << SMCCS_MODE_PS_SHIFT) /* 32 bytes */

/* SMC OCMS Mode Register */

#define SMC_OCMS_SMSE                  (1 << 0)  /* Bit 0:  Static Memory Controller Scrambling Enable */
#define SMC_OCMS_CSSE(n)               (1 << ((n)+16)) /* Chip Select (n=0-3) Scrambling Enable */
#  define SMC_OCMS_CS0SE               (1 << 16) /* Bit 16: Chip Select 0 Scrambling Enable */
#  define SMC_OCMS_CS1SE               (1 << 17) /* Bit 17: Chip Select 1 Scrambling Enable */
#  define SMC_OCMS_CS2SE               (1 << 18) /* Bit 18: Chip Select 2 Scrambling Enable */
#  define SMC_OCMS_CS3SE               (1 << 19) /* Bit 19: Chip Select 3 Scrambling Enable */

/* SMC KEY1/2 Registers (32-bit data) */

/* SMC Write Protect Mode Register */

#define SMC_WPCR_WPPEN                 (1 << 0)  /* Bit 0:  Write Protection Enable */
#define SMC_WPCR_WPKEY_SHIFT           (8)       /* Bits 8-31: Write Protection KEY password */
#define SMC_WPCR_WPKEY_MASK            (0x00ffffff << SMC_WPCR_WPKEY_SHIFT)
#  define SMC_WPCR_WPKEY               (0x00534d43 << SMC_WPCR_WPKEY_SHIFT)

/* SMC Write Protection Status */

#define SMC_WPSR_WPVS                  (1 << 0)  /* Bit 0:  Write Protect Violation Source */
#define SMC_WPSR_WPVSRC_SHIFT          (8)       /* Bits 8-23: Write Protection Violation Source */
#define SMC_WPSR_WPVSRC_MASK           (0xffff << SMC_WPSR_WPVSRC_SHIFT)

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_SMC_H */
