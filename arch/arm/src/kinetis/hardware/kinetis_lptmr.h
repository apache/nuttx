/****************************************************************************
 * arch/arm/src/kinetis/hardware/kinetis_lptmr.h
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

#ifndef __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_LPTMR_H
#define __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_LPTMR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define KINETIS_LPTMR_CSR_OFFSET    0x0000 /* Low Power Timer Control Status Register */
#define KINETIS_LPTMR_PSR_OFFSET    0x0004 /* Low Power Timer Prescale Register */
#define KINETIS_LPTMR_CMR_OFFSET    0x0008 /* Low Power Timer Compare Register */
#define KINETIS_LPTMR_CNR_OFFSET    0x000c /* Low Power Timer Counter Register */

/* Register Addresses *******************************************************/

#define KINETIS_LPTMR0_CSR          (KINETIS_LPTMR0_BASE+KINETIS_LPTMR_CSR_OFFSET)
#define KINETIS_LPTMR0_PSR          (KINETIS_LPTMR0_BASE+KINETIS_LPTMR_PSR_OFFSET)
#define KINETIS_LPTMR0_CMR          (KINETIS_LPTMR0_BASE+KINETIS_LPTMR_CMR_OFFSET)
#define KINETIS_LPTMR0_CNR          (KINETIS_LPTMR0_BASE+KINETIS_LPTMR_CNR_OFFSET)

#define KINETIS_LPTMR1_CSR          (KINETIS_LPTMR1_BASE+KINETIS_LPTMR_CSR_OFFSET)
#define KINETIS_LPTMR1_PSR          (KINETIS_LPTMR1_BASE+KINETIS_LPTMR_PSR_OFFSET)
#define KINETIS_LPTMR1_CMR          (KINETIS_LPTMR1_BASE+KINETIS_LPTMR_CMR_OFFSET)
#define KINETIS_LPTMR1_CNR          (KINETIS_LPTMR1_BASE+KINETIS_LPTMR_CNR_OFFSET)

/* Register Bit Definitions *************************************************/

/* Low Power Timer Control Status Register (32-bit) */

#define LPTMR_CSR_TEN               (1 << 0)  /* Bit 0:  Timer Enable */
#define LPTMR_CSR_TMS               (1 << 1)  /* Bit 1:  Timer Mode Select */
#define LPTMR_CSR_TFC               (1 << 2)  /* Bit 2:  Timer Free Running Counter */
#define LPTMR_CSR_TPP               (1 << 3)  /* Bit 3:  Timer Pin Polarity */
#define LPTMR_CSR_TPS_SHIFT         (4)       /* Bits 4-5: Timer Pin Select */
#define LPTMR_CSR_TPS_MASK          (3 << LPTMR_CSR_TPS_SHIFT)
#  define LPTMR_CSR_TPS_INPUT0      (0 << LPTMR_CSR_TPS_SHIFT) /* Pulse counter input 0 selected */
#  define LPTMR_CSR_TPS_INPUT1      (1 << LPTMR_CSR_TPS_SHIFT) /* Pulse counter input 1 selected */
#  define LPTMR_CSR_TPS_INPUT2      (2 << LPTMR_CSR_TPS_SHIFT) /* Pulse counter input 2 selected */
#  define LPTMR_CSR_TPS_INPUT3      (3 << LPTMR_CSR_TPS_SHIFT) /* Pulse counter input 3 selected */

#define LPTMR_CSR_TIE               (1 << 6)  /* Bit 6:  Timer Interrupt Enable */
#define LPTMR_CSR_TCF               (1 << 7)  /* Bit 7:  Timer Compare Flag */

/* Low Power Timer Prescale Register (32-bit) */

#define LPTMR_PSR_PCS_SHIFT         (0)       /* Bits 0-1: Prescaler Clock Select */
#define LPTMR_PSR_PCS_MASK          (3 << LPTMR_PSR_PCS_SHIFT)
#  define LPTMR_PSR_PCS_CLOCK       (0 << LPTMR_PSR_PCS_SHIFT) /* Prescaler/glitch filter clock 0 */
#  define LPTMR_PSR_PCS_CLOCK       (1 << LPTMR_PSR_PCS_SHIFT) /* Prescaler/glitch filter clock 1 */
#  define LPTMR_PSR_PCS_CLOCK       (2 << LPTMR_PSR_PCS_SHIFT) /* Prescaler/glitch filter clock 2 */
#  define LPTMR_PSR_PCS_CLOCK       (3 << LPTMR_PSR_PCS_SHIFT) /* Prescaler/glitch filter clock 3 */

#define LPTMR_PSR_PBYP              (1 << 2)  /* Bit 2:  Prescaler Bypass */
#define LPTMR_PSR_PRESCALE_SHIFT    (6)       /* Bits 3-6: Prescale Value */

#define LPTMR_PSR_PRESCALE_MASK     (15 << LPTMR_PSR_PRESCALE_SHIFT) /* Prescale divider: Glitch filter after: */
#  define LPTMR_PSR_PRESCALE_DIV2   (0 << LPTMR_PSR_PRESCALE_SHIFT)  /* Divider=2         N/S */
#  define LPTMR_PSR_PRESCALE_DIV4   (1 << LPTMR_PSR_PRESCALE_SHIFT)  /* Divider=4         2 edges */
#  define LPTMR_PSR_PRESCALE_DIV8   (2 << LPTMR_PSR_PRESCALE_SHIFT)  /* Divider=8         4 edges */
#  define LPTMR_PSR_PRESCALE_DIV16  (3 << LPTMR_PSR_PRESCALE_SHIFT)  /* Divider=16        8 edges */
#  define LPTMR_PSR_PRESCALE_DIV32  (4 << LPTMR_PSR_PRESCALE_SHIFT)  /* Divider=32        16 edges */
#  define LPTMR_PSR_PRESCALE_DIV64  (5 << LPTMR_PSR_PRESCALE_SHIFT)  /* Divider=64        32 edges */
#  define LPTMR_PSR_PRESCALE_DIV128 (6 << LPTMR_PSR_PRESCALE_SHIFT)  /* Divider=128       64 edges */
#  define LPTMR_PSR_PRESCALE_DIV256 (7 << LPTMR_PSR_PRESCALE_SHIFT)  /* Divider=256       128 edges */
#  define LPTMR_PSR_PRESCALE_DIV512 (8 << LPTMR_PSR_PRESCALE_SHIFT)  /* Divider=512       256 edges */
#  define LPTMR_PSR_PRESCALE_DIV1K  (9 << LPTMR_PSR_PRESCALE_SHIFT)  /* Divider=1024      512 edges */
#  define LPTMR_PSR_PRESCALE_DIV2K  (10 << LPTMR_PSR_PRESCALE_SHIFT) /* Divider=2048      1024 edges */
#  define LPTMR_PSR_PRESCALE_DIV4K  (11 << LPTMR_PSR_PRESCALE_SHIFT) /* Divider=4096      2048 edges */
#  define LPTMR_PSR_PRESCALE_DIV8K  (12 << LPTMR_PSR_PRESCALE_SHIFT) /* Divider=8192      4096 edges */
#  define LPTMR_PSR_PRESCALE_DIV16K (13 << LPTMR_PSR_PRESCALE_SHIFT) /* Divider=16384     8192 edges */
#  define LPTMR_PSR_PRESCALE_DIV32K (14 << LPTMR_PSR_PRESCALE_SHIFT) /* Divider=32768     16384 edges */
#  define LPTMR_PSR_PRESCALE_DIV64K (15 << LPTMR_PSR_PRESCALE_SHIFT) /* Divider=65536     32768 edges */

                                              /* Bits 7-31: Reserved */

/* Low Power Timer Compare Register */

#define LPTMR_CMR_SHIFT             (0)       /* Bits 0-15: Compare Value */
#define LPTMR_CMR_MASK              (0xffff << LPTMR_CMR_COMPARE_SHIFT)
                                              /* Bits 16-31: Reserved */

/* Low Power Timer Counter Register */

#define LPTMR_CNR_SHIFT             (0)       /* Bits 0-15: Counter Value */
#define LPTMR_CNR_MASK              (0xffff << LPTMR_CNR_COMPARE_SHIFT)
                                              /* Bits 16-31: Reserved */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_LPTMR_H */
