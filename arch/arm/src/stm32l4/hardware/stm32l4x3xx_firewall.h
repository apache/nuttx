/****************************************************************************
 * arch/arm/src/stm32l4/hardware/stm32l4x3xx_firewall.h
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

#ifndef __ARCH_ARM_SRC_STM32L4_HARDWARE_STM32L4X3XX_FIREWALL_H
#define __ARCH_ARM_SRC_STM32L4_HARDWARE_STM32L4X3XX_FIREWALL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32L4_FIREWALL_CSSA_OFFSET    0x0000
#define STM32L4_FIREWALL_CSL_OFFSET     0x0004
#define STM32L4_FIREWALL_NVDSSA_OFFSET  0x0008
#define STM32L4_FIREWALL_NVDSL_OFFSET   0x000c
#define STM32L4_FIREWALL_VDSSA_OFFSET   0x0010
#define STM32L4_FIREWALL_VDSL_OFFSET    0x0014
#define STM32L4_FIREWALL_CR_OFFSET      0x0020

/* Register Addresses *******************************************************/

#define STM32L4_FIREWALL_CSSA           (STM32L4_FIREWALL_BASE+STM32L4_FIREWALL_CSSA_OFFSET)
#define STM32L4_FIREWALL_CSL            (STM32L4_FIREWALL_BASE+STM32L4_FIREWALL_CSL_OFFSET)
#define STM32L4_FIREWALL_NVDSSA         (STM32L4_FIREWALL_BASE+STM32L4_FIREWALL_NVDSSA_OFFSET)
#define STM32L4_FIREWALL_NVDSL          (STM32L4_FIREWALL_BASE+STM32L4_FIREWALL_NVDSL_OFFSET)
#define STM32L4_FIREWALL_VDSSA          (STM32L4_FIREWALL_BASE+STM32L4_FIREWALL_VDSSA_OFFSET)
#define STM32L4_FIREWALL_VDSL           (STM32L4_FIREWALL_BASE+STM32L4_FIREWALL_VDSL_OFFSET)
#define STM32L4_FIREWALL_CR             (STM32L4_FIREWALL_BASE+STM32L4_FIREWALL_CR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Code Segment Start Address */
#define FIREWALL_CSSADD_SHIFT           8
#define FIREWALL_CSSADD_MASK            (0xffff << FIREWALL_CSSADD_SHIFT)

/* Code Segment Length */
#define FIREWALL_CSSLENG_SHIFT          8
#define FIREWALL_CSSLENG_MASK           (0x3fff << FIREWALL_CSSLENG_SHIFT)

/* Non-volatile Data Segment Start Address */
#define FIREWALL_NVDSADD_SHIFT          8
#define FIREWALL_NVDSADD_MASK           (0xffff << FIREWALL_NVDSADD_SHIFT)

/* Non-volatile Data Segment Length */
#define FIREWALL_NVDSLENG_SHIFT         8
#define FIREWALL_NVDSLENG_MASK          (0x3fff << FIREWALL_NVDSLENG_SHIFT)

/* Volatile Data Segment Start Address */
#define FIREWALL_VDSADD_SHIFT           6
#define FIREWALL_VDSADD_MASK            (0x07ff << FIREWALL_VDSADD_SHIFT)

/* Volatile Data Segment Length */
#define FIREWALL_VDSLENG_SHIFT          6
#define FIREWALL_VDSLENG_MASK           (0x07ff << FIREWALL_VDSLENG_SHIFT)

/* Configuration Register */
#define FIREWALL_CR_FPA                 (1 << 0) /* Bit 0: Firewall prearm */
#define FIREWALL_CR_VDS                 (1 << 1) /* Bit 1: Volatile data shared */
#define FIREWALL_CR_VDE                 (1 << 2) /* Bit 2: Volatile data execution */

#endif /* __ARCH_ARM_SRC_STM32L4_HARDWARE_STM32L4X3XX_FIREWALL_H */
