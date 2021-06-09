/****************************************************************************
 * arch/arm/src/stm32/hardware/stm32g4xxxx_cordic.h
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

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32G4XXXX_CORDIC_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32G4XXXX_CORDIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_CORDIC_CSR_OFFSET     0x0000    /* CORDIC control/status register */
#define STM32_CORDIC_WDATA_OFFSET   0x0004    /* CORDIC argument register */
#define STM32_CORDIC_RDATA_OFFSET   0x0008    /* CORDIC result register */

/* Register Addresses *******************************************************/

#define STM32_CORDIC_CSR            (STM32_CORDIC_BASE+STM32_CORDIC_CSR_OFFSET)
#define STM32_CORDIC_WDATA          (STM32_CORDIC_BASE+STM32_CORDIC_WDATA_OFFSET)
#define STM32_CORDIC_RDATA          (STM32_CORDIC_BASE+STM32_CORDIC_RDATA_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* CORDIC control and status register */

#define CORDIC_CSR_FUNC_SHIFT       (0)                          /* Bits 0-3: Function */
#define CORDIC_CSR_FUNC_MASK        (7 << CORDIC_CSR_FUNC_SHIFT)
#  define CORDIC_CSR_FUNC_COS       (0 << CORDIC_CSR_FUNC_SHIFT) /* Cosine */
#  define CORDIC_CSR_FUNC_SIN       (1 << CORDIC_CSR_FUNC_SHIFT) /* Sine */
#  define CORDIC_CSR_FUNC_PHASE     (2 << CORDIC_CSR_FUNC_SHIFT) /* Phase */
#  define CORDIC_CSR_FUNC_MOD       (3 << CORDIC_CSR_FUNC_SHIFT) /* Modulus */
#  define CORDIC_CSR_FUNC_ARCTAN    (4 << CORDIC_CSR_FUNC_SHIFT) /* Arctangent */
#  define CORDIC_CSR_FUNC_HCOS      (5 << CORDIC_CSR_FUNC_SHIFT) /* Hyperbolic cosine */
#  define CORDIC_CSR_FUNC_HSIN      (6 << CORDIC_CSR_FUNC_SHIFT) /* Hyperbolic sine */
#  define CORDIC_CSR_FUNC_HARCTAN   (7 << CORDIC_CSR_FUNC_SHIFT) /* Hyperbolic arctangent */
#  define CORDIC_CSR_FUNC_LN        (8 << CORDIC_CSR_FUNC_SHIFT) /* Natural logarithm */
#  define CORDIC_CSR_FUNC_SQRT      (9 << CORDIC_CSR_FUNC_SHIFT) /* Square root */
#define CORDIC_CSR_PRECISION_SHIFT  (4)                          /* Bits 4-7: Precision */
#define CORDIC_CSR_PRECISION_MASK   (7 << CORDIC_CSR_PRECISION_SHIFT)
#define CORDIC_CSR_SCALE_SHIFT      (8)                          /* Bits 8-10: Scale */
#define CORDIC_CSR_SCALE_MASK       (3 << CORDIC_CSR_SCALE_SHIFT)
#define CORDIC_CSR_IEN              (1 << 16)                    /* Bit 16: Enable interrupt */
#define CORDIC_CSR_DMAREN           (1 << 17)                    /* Bit 17: Enable DMA read channel */
#define CORDIC_CSR_DMAWEN           (1 << 18)                    /* Bit 18: Enable DMA write channel */
#define CORDIC_CSR_NRES             (1 << 19)                    /* Bit 19: Number of results in the CORDIC_RDATA register */
#define CORDIC_CSR_NARGS            (1 << 20)                    /* Bit 20: Number of arguments expected by the CORDIC_WDATA register */
#define CORDIC_CSR_RESSIZE          (1 << 21)                    /* Bit 21: Width of output data */
#define CORDIC_CSR_ARGSIZE          (1 << 22)                    /* Bit 22: Width of input data */
#define CORDIC_CSR_RRDY             (1 << 31)                    /* Bit 31: Result ready flag */

#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32G4XXXX_CORDIC_H */
