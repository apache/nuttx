/****************************************************************************************
 * arch/arm/src/sam34/chip/sam4l_bpm.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_SAM34_CHIP_SAM4L_BPM_H
#define __ARCH_ARM_SRC_SAM34_CHIP_SAM4L_BPM_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/sam_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* BPM register offsets ****************************************************************/

#define SAM_BPM_IER_OFFSET           0x0000 /* Interrupt Enable Register */
#define SAM_BPM_IDR_OFFSET           0x0004 /* Interrupt Disable Register */
#define SAM_BPM_IMR_OFFSET           0x0008 /* Interrupt Mask Register */
#define SAM_BPM_ISR_OFFSET           0x000c /* Interrupt Status Register */
#define SAM_BPM_ICR_OFFSET           0x0010 /* Interrupt Clear Register */
#define SAM_BPM_SR_OFFSET            0x0014 /* Status Register */
#define SAM_BPM_UNLOCK_OFFSET        0x0018 /* Unlock Register */
#define SAM_BPM_PMCON_OFFSET         0x001c /* Power Mode Control Register */
#define SAM_BPM_BKUPWCAUSE_OFFSET    0x0028 /* Backup Wake up Cause Register */
#define SAM_BPM_BKUPWEN_OFFSET       0x002c /* Backup Wake up Enable Register */
#define SAM_BPM_BKUPPMUX_OFFSET      0x0030 /* Backup Pin Muxing Register */
#define SAM_BPM_IORET_OFFSET         0x0034 /* Input Output Retention Register */
#define SAM_BPM_VERSION_OFFSET       0x00fc /* Version Register */

/* BPM register adresses ***************************************************************/

#define SAM_BPM_IER                  (SAM_BPM_BASE+SAM_BPM_IER_OFFSET)
#define SAM_BPM_IDR                  (SAM_BPM_BASE+SAM_BPM_IDR_OFFSET)
#define SAM_BPM_IMR                  (SAM_BPM_BASE+SAM_BPM_IMR_OFFSET)
#define SAM_BPM_ISR                  (SAM_BPM_BASE+SAM_BPM_ISR_OFFSET)
#define SAM_BPM_ICR                  (SAM_BPM_BASE+SAM_BPM_ICR_OFFSET)
#define SAM_BPM_SR                   (SAM_BPM_BASE+SAM_BPM_SR_OFFSET)
#define SAM_BPM_UNLOCK               (SAM_BPM_BASE+SAM_BPM_UNLOCK_OFFSET)
#define SAM_BPM_PMCON                (SAM_BPM_BASE+SAM_BPM_PMCON_OFFSET)
#define SAM_BPM_BKUPWCAUSE           (SAM_BPM_BASE+SAM_BPM_BKUPWCAUSE_OFFSET)
#define SAM_BPM_BKUPWEN              (SAM_BPM_BASE+SAM_BPM_BKUPWEN_OFFSET)
#define SAM_BPM_BKUPPMUX             (SAM_BPM_BASE+SAM_BPM_BKUPPMUX_OFFSET)
#define SAM_BPM_IORET                (SAM_BPM_BASE+SAM_BPM_IORET_OFFSET)
#define SAM_BPM_VERSION              (SAM_BPM_BASE+SAM_BPM_VERSION_OFFSET)

/* BPM register bit definitions ********************************************************/

/* Interrupt Enable Register */
/* Interrupt Disable Register */
/* Interrupt Mask Register */
/* Interrupt Status Register */
/* Interrupt Clear Register */
/* Status Register */

#define BPM_INT_PSOK                 (1 << 0)  /* Bit 0:  Power Scaling OK */
#define BPM_INT_AE                   (1 << 31) /* Bit 31: Access Error */

/* Unlock Register */

#define BPM_UNLOCK_ADDR_SHIFT        (0)       /* Bits 0-9: Unlock Address */
#define BPM_UNLOCK_ADDR_MASK         (0x3ff << BPM_UNLOCK_ADDR_SHIFT)
#  define BPM_UNLOCK_ADDR(n)         ((n) << BPM_UNLOCK_ADDR_SHIFT)
#define BPM_UNLOCK_KEY_SHIFT         (24)      /* Bits 24-31: Unlock Key */
#define BPM_UNLOCK_KEY_MASK          (0xff << BPM_UNLOCK_KEY_SHIFT)
#  define BPM_UNLOCK_KEY(n)          ((n) << BPM_UNLOCK_KEY_SHIFT)

/* Power Mode Control Register */

#define BPM_PMCON_PS_SHIFT           (0)       /* Bits 0-1: Power Scaling Configuration Value */
#define BPM_PMCON_PS_MASK            (3 << BPM_PMCON_PS_SHIFT)
#  define BPM_PMCON_PS0              (0 << BPM_PMCON_PS_SHIFT)
#  define BPM_PMCON_PS1              (1 << BPM_PMCON_PS_SHIFT)
#  define BPM_PMCON_PS2              (2 << BPM_PMCON_PS_SHIFT)
#define BPM_PMCON_PSCREQ             (1 << 2)  /* Bit 2:  Power Scaling Change Request */
#define BPM_PMCON_PSCM               (1 << 3)  /* Bit 3:  Power Scaling Change Mode */
#define BPM_PMCON_BKUP               (1 << 8)  /* Bit 8:  BACKUP Mode */
#define BPM_PMCON_RET                (1 << 9)  /* Bit 9:  RETENTION Mode */
#define BPM_PMCON_SLEEP_SHIFT        (12)      /* Bits 12-13: SLEEP mode Configuration */
#define BPM_PMCON_SLEEP_MASK         (3 << BPM_PMCON_SLEEP_SHIFT)
# define BPM_PMCON_SLEEP_SLEEP0      (0 << BPM_PMCON_SLEEP_SHIFT) /* CPU clock stopped */
# define BPM_PMCON_SLEEP_SLEEP1      (1 << BPM_PMCON_SLEEP_SHIFT) /* CPU+AHB clocks stopped */
# define BPM_PMCON_SLEEP_SLEEP2      (2 << BPM_PMCON_SLEEP_SHIFT) /* CPU+AHB+PB+GCLK clocks stopped */
# define BPM_PMCON_SLEEP_SLEEP3      (3 << BPM_PMCON_SLEEP_SHIFT) /* CPU+AHB+PB+GCLK+sources stopped */
#define BPM_PMCON_CK32S              (1 << 16) /* Bit 16: 32kHz-1kHz Clock Source Selection */
#define BPM_PMCON_FASTWKUP           (1 << 24) /* Bit 24: Fast Wakeup */

/* Backup Wake up Cause Register */

#define BPM_BKUPWCAUSE_EIC           (1 << 0)  /* Bit 0: EIC */
#define BPM_BKUPWCAUSE_AST           (1 << 1)  /* Bit 1: AST */
#define BPM_BKUPWCAUSE_WDT           (1 << 2)  /* Bit 2: WDT interrupt */
#define BPM_BKUPWCAUSE_BOD33         (1 << 3)  /* Bit 3: BOD33 interrupt */
#define BPM_BKUPWCAUSE_BOD18         (1 << 4)  /* Bit 4: BOD18 interrupt */
#define BPM_BKUPWCAUSE_PICOUART      (1 << 5)  /* Bit 5: PICOUART interrupt */

/* Backup Wake up Enable Register */

#define BPM_BKUPWEN_EICEN            (1 << 0)  /* Bit 0: EIC */
#define BPM_BKUPWEN_ASTEN            (1 << 1)  /* Bit 1: AST */
#define BPM_BKUPWEN_WDTEN            (1 << 2)  /* Bit 2: WDT interrupt */
#define BPM_BKUPWEN_BOD33EN          (1 << 3)  /* Bit 3: BOD33 interrupt */
#define BPM_BKUPWEN_BOD18EN          (1 << 4)  /* Bit 4: BOD18 interrupt */
#define BPM_BKUPWEN_PICOUARTEN       (1 << 5)  /* Bit 5: PICOUART interrupt */

/* Backup Pin Muxing Register */

#define BPM_BKUPPMUX_EIC0            (1 << 0)  /* Bit 0: PB01 EIC[0] */
#define BPM_BKUPPMUX_EIC1            (1 << 1)  /* Bit 1: PA06 EIC[1] */
#define BPM_BKUPPMUX_EIC2            (1 << 2)  /* Bit 2: PA04 EIC[2] */
#define BPM_BKUPPMUX_EIC3            (1 << 3)  /* Bit 3: PA05 EIC[3] */
#define BPM_BKUPPMUX_EIC4            (1 << 4)  /* Bit 4: PA07 EIC[4] */
#define BPM_BKUPPMUX_EIC5            (1 << 5)  /* Bit 5: PC03 EIC[5] */
#define BPM_BKUPPMUX_EIC6            (1 << 6)  /* Bit 6: PC04 EIC[6] */
#define BPM_BKUPPMUX_EIC7            (1 << 7)  /* Bit 7: PC05 EIC[7] */
#define BPM_BKUPPMUX_EIC8            (1 << 8)  /* Bit 8: PC06 EIC[8] */

/* Input Output Retention Register */

#define BPM_IORET_RET                (1 << 0)  /* Bit 0: : Retention on I/O lines after wakeup */

/* Version Register */

#define BPM_VERSION_SHIFT            (0)        /* Bits 0-11: Version Number */
#define BPM_VERSION_MASK             (0xfff << BPM_VERSION_VERSION_SHIFT)
#define BPM_VERSION_VARIANT_SHIFT    (16)       /* Bits 16-19: Variant Number */
#define BPM_VERSION_VARIANT_MASK     (15 << BPM_VERSION_VARIANT_SHIFT)

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM34_CHIP_SAM4L_BPM_H */
