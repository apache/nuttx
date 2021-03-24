/****************************************************************************
 * arch/arm/src/sama5/hardware/sam_rxlp.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_RXLP_H
#define __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_RXLP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RXLP register offsets ****************************************************/

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

/* RXLP register addresses **************************************************/

#define SAM_RXLP0_CR                 (SAM_RXLP_VBASE+SAM_RXLP_CR_OFFSET)
#define SAM_RXLP0_MR                 (SAM_RXLP_VBASE+SAM_RXLP_MR_OFFSET)
#define SAM_RXLP0_RHR                (SAM_RXLP_VBASE+SAM_RXLP_RHR_OFFSET)
#define SAM_RXLP0_BRGR               (SAM_RXLP_VBASE+SAM_RXLP_BRGR_OFFSET)
#define SAM_RXLP0_CMPR               (SAM_RXLP_VBASE+SAM_RXLP_CMPR_OFFSET)
#define SAM_RXLP0_WPMR               (SAM_RXLP_VBASE+SAM_RXLP_WPMR_OFFSET)

/* RXLP register bit definitions ********************************************/

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

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_RXLP_H */
