/****************************************************************************
 * arch/arm/src/samv7/hardware/sam_trng.h
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

#ifndef __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_TRNG_H
#define __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_TRNG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TRNG Register Offsets ****************************************************/

#define SAM_TRNG_CR_OFFSET        0x0000 /* Control Register */
#define SAM_TRNG_IER_OFFSET       0x0010 /* Interrupt Enable Register */
#define SAM_TRNG_IDR_OFFSET       0x0014 /* Interrupt Disable Register */
#define SAM_TRNG_IMR_OFFSET       0x0018 /* Interrupt Mask Register */
#define SAM_TRNG_ISR_OFFSET       0x001c /* Interrupt Status Register */
#define SAM_TRNG_ODATA_OFFSET     0x0050 /* Output Data Register */

/* TRNG Register Addresses **************************************************/

#define SAM_TRNG_CR               (SAM_TRNG_BASE+SAM_TRNG_CR_OFFSET)
#define SAM_TRNG_IER              (SAM_TRNG_BASE+SAM_TRNG_IER_OFFSET)
#define SAM_TRNG_IDR              (SAM_TRNG_BASE+SAM_TRNG_IDR_OFFSET)
#define SAM_TRNG_IMR              (SAM_TRNG_BASE+SAM_TRNG_IMR_OFFSET)
#define SAM_TRNG_ISR              (SAM_TRNG_BASE+SAM_TRNG_ISR_OFFSET)
#define SAM_TRNG_ODATA            (SAM_TRNG_BASE+SAM_TRNG_ODATA_OFFSET)

/* TRNG Register Bit Definitions ********************************************/

/* Control Register */

#define TRNG_CR_ENABLE            (1 << 0)  /* Bit 0:  1=Enables the TRNG */
#  define TRNG_CR_DISABLE         (0)       /* Bit 0:  0=Disables the TRNG */
#define TRNG_CR_KEY_SHIFT         (8)       /* Bits 8-31: Security key */
#define TRNG_CR_KEY_MASK          (0xffffff << TRNG_CR_KEY_SHIFT)
# define TRNG_CR_KEY              (0x524e47 << TRNG_CR_KEY_SHIFT) /* RNG in ASCII */

/* Interrupt Enable Register, Interrupt Disable Register,
 * Interrupt Mask Register, and Interrupt Status Register
 */

#define TRNG_INT_DATRDY           (1 << 0)  /* Bit 0:  Data ready */

/* Output Data Register (32-bit output data) */

#endif /* __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_TRNG_H */
