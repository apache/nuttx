/****************************************************************************
 * arch/arm/src/stm32n6/hardware/stm32n6xxx_syscfg.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_SYSCFG_H
#define __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_SYSCFG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/stm32n6xxx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_SYSCFG_INITSVTORCR_OFFSET  0x010 /* Secure vector table base address register */
#define STM32_SYSCFG_VDDIO2CCCR_OFFSET   0x054 /* VDDIO2 compensation cell control register */
#define STM32_SYSCFG_VDDIO3CCCR_OFFSET   0x05c /* VDDIO3 compensation cell control register */
#define STM32_SYSCFG_VDDCCCR_OFFSET      0x064 /* VDD compensation cell control register */

/* Register Addresses *******************************************************/

#define STM32_SYSCFG_INITSVTORCR         (STM32_SYSCFG_BASE + STM32_SYSCFG_INITSVTORCR_OFFSET)
#define STM32_SYSCFG_VDDIO2CCCR          (STM32_SYSCFG_BASE + STM32_SYSCFG_VDDIO2CCCR_OFFSET)
#define STM32_SYSCFG_VDDIO3CCCR          (STM32_SYSCFG_BASE + STM32_SYSCFG_VDDIO3CCCR_OFFSET)
#define STM32_SYSCFG_VDDCCCR             (STM32_SYSCFG_BASE + STM32_SYSCFG_VDDCCCR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Compensation cell control register (VDDxCCCR / VDDIOxCCCR).  The four
 * banks (VDD, VDDIO2, VDDIO3, VDDIO4, VDDIO5) share an identical layout, so
 * the port refers to them with a single set of CCCR_* macros.
 */

#define SYSCFG_CCCR_CS              (1 << 9)  /* Bit 9:  Compensation code source select */
#define SYSCFG_CCCR_EN              (1 << 8)  /* Bit 8:  Enable compensation cell */
#define SYSCFG_CCCR_RAPSRC_SHIFT    (4)       /* Bits 7-4: Manual PMOS compensation code */
#define SYSCFG_CCCR_RAPSRC_MASK     (0xf << SYSCFG_CCCR_RAPSRC_SHIFT)
#define SYSCFG_CCCR_RANSRC_SHIFT    (0)       /* Bits 3-0: Manual NMOS compensation code */
#define SYSCFG_CCCR_RANSRC_MASK     (0xf << SYSCFG_CCCR_RANSRC_SHIFT)

/* ES0620 I/O compensation mitigation: write 0x00000287 (EN=0, CS=1,
 * RAPSRC=0x8, RANSRC=0x7) to VDDCCCR and every VDDIOxCCCR before
 * enabling high-speed pads.  This disables the cell and overrides the
 * broken defaults (RAPSRC=0x7, RANSRC=0x8) that deform output edges.
 */

#define SYSCFG_CCCR_ES0620_MANUAL   (SYSCFG_CCCR_CS | \
                                     (8 << SYSCFG_CCCR_RAPSRC_SHIFT) | \
                                     (7 << SYSCFG_CCCR_RANSRC_SHIFT))

#endif /* __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_SYSCFG_H */
