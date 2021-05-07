/****************************************************************************
 * arch/arm/src/sam34/hardware/sam_pdc.h
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

#ifndef __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_PDC_H
#define __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_PDC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PDC register offsets *****************************************************/

#define SAM_PDC_RPR_OFFSET           0x100 /* Receive Pointer Register */
#define SAM_PDC_RCR_OFFSET           0x104 /* Receive Counter Register */
#define SAM_PDC_TPR_OFFSET           0x108 /* Transmit Pointer Register */
#define SAM_PDC_TCR_OFFSET           0x10c /* Transmit Counter Register */
#define SAM_PDC_RNPR_OFFSET          0x110 /* Receive Next Pointer Register */
#define SAM_PDC_RNCR_OFFSET          0x114 /* Receive Next Counter Register */
#define SAM_PDC_TNPR_OFFSET          0x118 /* Transmit Next Pointer Register */
#define SAM_PDC_TNCR_OFFSET          0x11c /* Transmit Next Counter Register */
#define SAM_PDC_PTCR_OFFSET          0x120 /* Transfer Control Register */
#define SAM_PDC_PTSR_OFFSET          0x124 /* Transfer Status Register */

/* PDC register addresses ***************************************************/

/* These 10 registers are mapped in the peripheral memory space at the same
 * offset.
 */

/* PDC register bit definitions *********************************************/

/* Receive Pointer Register -- 32-bit address value */

/* Receive Counter Register -- 16-bit counter value */

#define PDC_RCR_RXCTR_SHIFT          (0)      /* Bits 0-15:  Receive Counter Register */
#define PDC_RCR_RXCTR_MASK           (0xffff << PDC_RCR_RXCTR_SHIFT)

/* Transmit Pointer Register -- 32-bit address value */

/* Transmit Counter Register -- 16-bit counter value */

#define PDC_TCR_TXCTR_SHIFT          (0)      /* Bits 0-15:  Transmit Counter Register */
#define PDC_TCR_TXCTR_MASK           (0xffff << PDC_TCR_TXCTR_SHIFT)

/* Receive Next Pointer Register -- 32-bit address value */

/* Receive Next Counter Register -- 16-bit counter value */

#define PDC_RNCR_RXNCTR_SHIFT        (0)      /* Bits 0-15:  Receive Next Counter */
#define PDC_RNCR_RXNCTR_MASK         (0xffff << PDC_RNCR_RXNCTR_SHIFT)

/* Transmit Next Pointer Register -- 32-bit address value */

/* Transmit Next Counter Register -- 16-bit counter value */

#define PDC_TNCR_TXNCTR_SHIFT        (0)      /* Bits 0-15:   Transmit Counter Next */
#define PDC_TNCR_TXNCTR_MASK         (0xffff << PDC_TNCR_TXNCTR_SHIFT)

/* Transfer Control Register */

#define PDC_PTCR_RXTEN               (1 << 0)  /* Bit 0:  Receiver Transfer Enable */
#define PDC_PTCR_RXTDIS              (1 << 1)  /* Bit 1:  Receiver Transfer Disable */
#define PDC_PTCR_TXTEN               (1 << 8)  /* Bit 8:  Transmitter Transfer Enable */
#define PDC_PTCR_TXTDIS              (1 << 9)  /* Bit 9:  Transmitter Transfer Disable */

/* Transfer Status Register */

#define PDC_PTSR_RXTEN               (1 << 0)  /* Bit 0:  Receiver Transfer Enable */
#define PDC_PTSR_TXTEN               (1 << 8)  /* Bit 8:  Transmitter Transfer Enable */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_PDC_H */
