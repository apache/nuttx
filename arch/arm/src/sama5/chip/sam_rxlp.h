/************************************************************************************************
 * arch/arm/src/sama5/chip/sam_RXLP.h
 * Low Power Asynchronous Receiver (RXLP) definitions for the SAMA5D3
 * and SAMAD4
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
 ************************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMA5_CHIP_SAM_RXLP_H
#define __ARCH_ARM_SRC_SAMA5_CHIP_SAM_RXLP_H

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/sam_memorymap.h"

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/

/* RXLP register offsets ************************************************************************/

#define SAM_RXLP_CR_OFFSET           0x0000 /* Control Register */
#define SAM_RXLP_MR_OFFSET           0x0004 /* Mode Register */
                                            /* 0x0008-0x0014: Reserved */
#define SAM_RXLP_RHR_OFFSET          0x0018 /* Receive Holding Register */
                                            /* 0x001c: Reserved */
#define SAM_RXLP_BRGR_OFFSET         0x0020 /* Baud Rate Generator Register */
#define SAM_RXLP_CMPR_OFFSET         0x0024 /* Comparison Register */
                                            /* 0x0024-0x00e0: Reserved */
#define SAM_RXLP_WPMR_OFFSET         0x00e4 /* Write Protect Mode Register (4) */
                                            /* 0x00e8-0xfc: Reserved */

/* RXLP register adresses ***********************************************************************/

#define SAM_RXLP0_CR                 (SAM_RXLP_VBASE+SAM_RXLP_CR_OFFSET)
#define SAM_RXLP0_MR                 (SAM_RXLP_VBASE+SAM_RXLP_MR_OFFSET)
#define SAM_RXLP0_RHR                (SAM_RXLP_VBASE+SAM_RXLP_RHR_OFFSET)
#define SAM_RXLP0_BRGR               (SAM_RXLP_VBASE+SAM_RXLP_BRGR_OFFSET)
#define SAM_RXLP0_CMPR               (SAM_RXLP_VBASE+SAM_RXLP_CMPR_OFFSET)
#define SAM_RXLP0_WPMR               (SAM_RXLP_VBASE+SAM_RXLP_WPMR_OFFSET)

/* RXLP register bit definitions ****************************************************************/

/* RXLP Control Register */

#define RXLP_CR_RSTRX                (1 << 2)  /* Bit 2:  Reset Receiver (Common) */
#define RXLP_CR_RXEN                 (1 << 4)  /* Bit 4:  Receiver Enable (Common) */
#define RXLP_CR_RXDIS                (1 << 5)  /* Bit 5:  Receiver Disable (Common) */

/* RXLP Mode Register */

#define RXLP_MR_FILTER               (1 << 4)  /* Bit 4: Receiver Digital Filter */
#define RXLP_MR_PAR_SHIFT            (9)       /* Bits 9-11: Parity Type (2) */
#define RXLP_MR_PAR_MASK             (7 << RXLP_MR_PAR_SHIFT)
#  define RXLP_MR_PAR_EVEN           (0 << RXLP_MR_PAR_SHIFT) /* Even parity */
#  define RXLP_MR_PAR_ODD            (1 << RXLP_MR_PAR_SHIFT) /* Odd parity */
#  define RXLP_MR_PAR_SPACE          (2 << RXLP_MR_PAR_SHIFT) /* Space: parity forced to 0 */
#  define RXLP_MR_PAR_MARK           (3 << RXLP_MR_PAR_SHIFT) /* Mark: parity forced to 1 */
#  define RXLP_MR_PAR_NONE           (4 << RXLP_MR_PAR_SHIFT) /* No parity */

/* RXLP Receiver Holding Register */

#define RXLP_RHR_RXCHR_SHIFT         (0)       /* Bits 0-7: Received Character (RXLP only) */
#define RXLP_RHR_RXCHR_MASK          (3 << RXLP_RHR_RXCHR_SHIFT)
#  define RXLP_RHR_RXCHR(n)          ((uint32_t)(n) << RXLP_RHR_RXCHR_SHIFT)

/* RXLP Baud Rate Generator Register */

#define RXLP_BRGR_CD_SHIFT           (0)       /* Bits 0-15: Clock Divisor (Common) */
#define RXLP_BRGR_CD_MASK            (0xffff << RXLP_BRGR_CD_SHIFT)

/* Comparison Register */

#define RXLP_CMPR_VAL1_SHIFT         (0)       /* Bits 0-7: First Comparison Value for Received Character */
#define RXLP_CMPR_VAL1_MASK          (0xff << RXLP_CMPR_VAL1_SHIFT)
#  define RXLP_CMPR_VAL1(n)          ((uint32_t)(n) << RXLP_CMPR_VAL1_SHIFT)
#define RXLP_CMPR_VAL2_SHIFT         (16)      /* Bits 16-23: Second Comparison Value for Received Character */
#define RXLP_CMPR_VAL2_MASK          (0xff << RXLP_CMPR_VAL2_SHIFT)
#  define RXLP_CMPR_VAL2(n)          ((uint32_t)(n) << RXLP_CMPR_VAL2_SHIFT)

/* RXLP Write Protect Mode Register */

#define RXLP_WPMR_WPEN               (1 << 0)  /* Bit 0: Write Protect Enable */
#define RXLP_WPMR_WPKEY_SHIFT        (8)       /* Bits 8-31: Write Protect KEY */
#define RXLP_WPMR_WPKEY_MASK         (0x00ffffff << RXLP_WPMR_WPKEY_SHIFT)
#  define RXLP_WPMR_WPKEY            (0x0052584c << RXLP_WPMR_WPKEY_SHIFT) /* "RXL" */

/************************************************************************************************
 * Public Types
 ************************************************************************************************/

/************************************************************************************************
 * Public Data
 ************************************************************************************************/

/************************************************************************************************
 * Public Functions
 ************************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMA5_CHIP_SAM_RXLP_H */
