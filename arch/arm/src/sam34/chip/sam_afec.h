/****************************************************************************************
 * arch/arm/src/sam34/chip/sam_afec.h
 * Analog-Front-End Controller (AFEC) definitions for the SAM4E
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_SAM34_CHIP_SAM_AFEC_H
#define __ARCH_ARM_SRC_SAM34_CHIP_SAM_AFEC_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/sam_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* AFEC register offsets ****************************************************************/

#define SAM_AFEC_CR_OFFSET           0x0000 /* Control Register */
#define SAM_AFEC_MR_OFFSET           0x0004 /* Mode Register */
#define SAM_AFEC_EMR_OFFSET          0x0008 /* Extended Mode Register */
#define SAM_AFEC_SEQ1R_OFFSET        0x000c /* Channel Sequence 1 Register */
#define SAM_AFEC_SEQ2R_OFFSET        0x0010 /* Channel Sequence 2 Register */
#define SAM_AFEC_CHER_OFFSET         0x0014 /* Channel Enable Register */
#define SAM_AFEC_CHDR_OFFSET         0x0018 /* Channel Disable Register */
#define SAM_AFEC_CHSR_OFFSET         0x001c /* Channel Status Register */
#define SAM_AFEC_LCDR_OFFSET         0x0020 /* Last Converted Data Register */
#define SAM_AFEC_IER_OFFSET          0x0024 /* Interrupt Enable Register */
#define SAM_AFEC_IDR_OFFSET          0x0028 /* Interrupt Disable Register */
#define SAM_AFEC_IMR_OFFSET          0x002c /* Interrupt Mask Register */
#define SAM_AFEC_ISR_OFFSET          0x0030 /* Interrupt Status Register */
                                            /* 0x0034-0x0040 Reserved */
                                            /* 0x0044-0x0048 Reserved */
#define SAM_AFEC_OVER_OFFSET         0x004c /* Overrun Status Register */
#define SAM_AFEC_CWR_OFFSET          0x0050 /* Compare Window Register */
#define SAM_AFEC_CGR_OFFSET          0x0054 /* Channel Gain Register */
#define SAM_AFEC_CDOR_OFFSET         0x005c /* Channel Calibration DC Offset Register */
#define SAM_AFEC_DIFFR_OFFSET        0x0060 /* Channel Differential Register */
#define SAM_AFEC_CSELR_OFFSET        0x0064 /* Channel Register Selection */
#define SAM_AFEC_CDR_OFFSET          0x0068 /* Channel Data Register */
#define SAM_AFEC_COCR_OFFSET         0x006c /* Channel Offset Compensation Register */
#define SAM_AFEC_TEMPMR_OFFSET       0x0070 /* Temperature Sensor Mode Register */
#define SAM_AFEC_TEMPCWR_OFFSET      0x0074 /* Temperature Compare Window Register */
#define SAM_AFEC_ACR_OFFSET          0x0094 /* Analog Control Register */
                                            /* 0x0098-0x00ac Reserved */
                                            /* 0x00c4-0x00e0 Reserved */
#define SAM_AFEC_WPMR_OFFSET         0x00e4 /* Write Protect Mode Register */
#define SAM_AFEC_WPSR_OFFSET         0x00e8 /* Write Protect Status Register */
                                            /* 0x00ec-0x00f8 Reserved */
                                            /* 0x0fc Reserved */
                                            /* 0x0100-0x0124 Reserved for PDC */

/* AFEC register addresses **************************************************************/

#define SAM_AFEC0_CR                 (SAM_AFEC0_BASE+SAM_AFEC_CR_OFFSET)
#define SAM_AFEC0_MR                 (SAM_AFEC0_BASE+SAM_AFEC_MR_OFFSET)
#define SAM_AFEC0_EMR                (SAM_AFEC0_BASE+SAM_AFEC_EMR_OFFSET)
#define SAM_AFEC0_SEQ1R              (SAM_AFEC0_BASE+SAM_AFEC_SEQ1R_OFFSET)
#define SAM_AFEC0_SEQ2R              (SAM_AFEC0_BASE+SAM_AFEC_SEQ2R_OFFSET)
#define SAM_AFEC0_CHER               (SAM_AFEC0_BASE+SAM_AFEC_CHER_OFFSET)
#define SAM_AFEC0_CHDR               (SAM_AFEC0_BASE+SAM_AFEC_CHDR_OFFSET)
#define SAM_AFEC0_CHSR               (SAM_AFEC0_BASE+SAM_AFEC_CHSR_OFFSET)
#define SAM_AFEC0_LCDR               (SAM_AFEC0_BASE+SAM_AFEC_LCDR_OFFSET)
#define SAM_AFEC0_IER                (SAM_AFEC0_BASE+SAM_AFEC_IER_OFFSET)
#define SAM_AFEC0_IDR                (SAM_AFEC0_BASE+SAM_AFEC_IDR_OFFSET)
#define SAM_AFEC0_IMR                (SAM_AFEC0_BASE+SAM_AFEC_IMR_OFFSET)
#define SAM_AFEC0_ISR                (SAM_AFEC0_BASE+SAM_AFEC_ISR_OFFSET)
#define SAM_AFEC0_OVER               (SAM_AFEC0_BASE+SAM_AFEC_OVER_OFFSET)
#define SAM_AFEC0_CWR                (SAM_AFEC0_BASE+SAM_AFEC_CWR_OFFSET)
#define SAM_AFEC0_CGR                (SAM_AFEC0_BASE+SAM_AFEC_CGR_OFFSET)
#define SAM_AFEC0_CDOR               (SAM_AFEC0_BASE+SAM_AFEC_CDOR_OFFSET)
#define SAM_AFEC0_DIFFR              (SAM_AFEC0_BASE+SAM_AFEC_DIFFR_OFFSET)
#define SAM_AFEC0_CSELR              (SAM_AFEC0_BASE+SAM_AFEC_CSELR_OFFSET)
#define SAM_AFEC0_CDR                (SAM_AFEC0_BASE+SAM_AFEC_CDR_OFFSET)
#define SAM_AFEC0_COCR               (SAM_AFEC0_BASE+SAM_AFEC_COCR_OFFSET)
#define SAM_AFEC0_TEMPMR             (SAM_AFEC0_BASE+SAM_AFEC_TEMPMR_OFFSET)
#define SAM_AFEC0_TEMPCWR            (SAM_AFEC0_BASE+SAM_AFEC_TEMPCWR_OFFSET)
#define SAM_AFEC0_ACR                (SAM_AFEC0_BASE+SAM_AFEC_ACR_OFFSET)
#define SAM_AFEC0_WPMR               (SAM_AFEC0_BASE+SAM_AFEC_WPMR_OFFSET)
#define SAM_AFEC0_WPSR               (SAM_AFEC0_BASE+SAM_AFEC_WPSR_OFFSET)

#define SAM_AFEC1_CR                 (SAM_AFEC1_BASE+SAM_AFEC_CR_OFFSET)
#define SAM_AFEC1_MR                 (SAM_AFEC1_BASE+SAM_AFEC_MR_OFFSET)
#define SAM_AFEC1_EMR                (SAM_AFEC1_BASE+SAM_AFEC_EMR_OFFSET)
#define SAM_AFEC1_SEQ1R              (SAM_AFEC1_BASE+SAM_AFEC_SEQ1R_OFFSET)
#define SAM_AFEC1_SEQ2R              (SAM_AFEC1_BASE+SAM_AFEC_SEQ2R_OFFSET)
#define SAM_AFEC1_CHER               (SAM_AFEC1_BASE+SAM_AFEC_CHER_OFFSET)
#define SAM_AFEC1_CHDR               (SAM_AFEC1_BASE+SAM_AFEC_CHDR_OFFSET)
#define SAM_AFEC1_CHSR               (SAM_AFEC1_BASE+SAM_AFEC_CHSR_OFFSET)
#define SAM_AFEC1_LCDR               (SAM_AFEC1_BASE+SAM_AFEC_LCDR_OFFSET)
#define SAM_AFEC1_IER                (SAM_AFEC1_BASE+SAM_AFEC_IER_OFFSET)
#define SAM_AFEC1_IDR                (SAM_AFEC1_BASE+SAM_AFEC_IDR_OFFSET)
#define SAM_AFEC1_IMR                (SAM_AFEC1_BASE+SAM_AFEC_IMR_OFFSET)
#define SAM_AFEC1_ISR                (SAM_AFEC1_BASE+SAM_AFEC_ISR_OFFSET)
#define SAM_AFEC1_OVER               (SAM_AFEC1_BASE+SAM_AFEC_OVER_OFFSET)
#define SAM_AFEC1_CWR                (SAM_AFEC1_BASE+SAM_AFEC_CWR_OFFSET)
#define SAM_AFEC1_CGR                (SAM_AFEC1_BASE+SAM_AFEC_CGR_OFFSET)
#define SAM_AFEC1_CDOR               (SAM_AFEC1_BASE+SAM_AFEC_CDOR_OFFSET)
#define SAM_AFEC1_DIFFR              (SAM_AFEC1_BASE+SAM_AFEC_DIFFR_OFFSET)
#define SAM_AFEC1_CSELR              (SAM_AFEC1_BASE+SAM_AFEC_CSELR_OFFSET)
#define SAM_AFEC1_CDR                (SAM_AFEC1_BASE+SAM_AFEC_CDR_OFFSET)
#define SAM_AFEC1_COCR               (SAM_AFEC1_BASE+SAM_AFEC_COCR_OFFSET)
#define SAM_AFEC1_TEMPMR             (SAM_AFEC1_BASE+SAM_AFEC_TEMPMR_OFFSET)
#define SAM_AFEC1_TEMPCWR            (SAM_AFEC1_BASE+SAM_AFEC_TEMPCWR_OFFSET)
#define SAM_AFEC1_ACR                (SAM_AFEC1_BASE+SAM_AFEC_ACR_OFFSET)
#define SAM_AFEC1_WPMR               (SAM_AFEC1_BASE+SAM_AFEC_WPMR_OFFSET)
#define SAM_AFEC1_WPSR               (SAM_AFEC1_BASE+SAM_AFEC_WPSR_OFFSET)

/* AFEC register bit definitions *******************************************************/

/* Control Register */
#define AFEC_CR_
/* Mode Register */
#define AFEC_MR_
/* Extended Mode Register */
#define AFEC_EMR_
/* Channel Sequence 1 Register */
#define AFEC_SEQ1R_
/* Channel Sequence 2 Register */
#define AFEC_SEQ2R_
/* Channel Enable Register */
#define AFEC_CHER_
/* Channel Disable Register */
#define AFEC_CHDR_
/* Channel Status Register */
#define AFEC_CHSR_
/* Last Converted Data Register */
#define AFEC_LCDR_
/* Interrupt Enable Register */
#define AFEC_IER_
/* Interrupt Disable Register */
#define AFEC_IDR_
/* Interrupt Mask Register */
#define AFEC_IMR_
/* Interrupt Status Register */
#define AFEC_ISR_
/* Overrun Status Register */
#define AFEC_OVER_
/* Compare Window Register */
#define AFEC_CWR_
/* Channel Gain Register */
#define AFEC_CGR_
/* Channel Calibration DC Offset Register */
#define AFEC_CDOR_
/* Channel Differential Register */
#define AFEC_DIFFR_
/* Channel Register Selection */
#define AFEC_CSELR_
/* Channel Data Register */
#define AFEC_CDR_
/* Channel Offset Compensation Register */
#define AFEC_COCR_
/* Temperature Sensor Mode Register */
#define AFEC_TEMPMR_
/* Temperature Compare Window Register */
#define AFEC_TEMPCWR_
/* Analog Control Register */
#define AFEC_ACR_
/* Write Protect Mode Register */
#define AFEC_WPMR_
/* Write Protect Status Register */
#define AFEC_WPSR_

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM34_CHIP_SAM_AFEC_H */
