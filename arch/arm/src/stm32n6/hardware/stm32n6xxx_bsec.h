/****************************************************************************
 * arch/arm/src/stm32n6/hardware/stm32n6xxx_bsec.h
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

#ifndef __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_BSEC_H
#define __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_BSEC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/stm32n6xxx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_BSEC_DBGCR_OFFSET      0xe8c /* Debug control register */
#define STM32_BSEC_AP_UNLOCK_OFFSET  0xe90 /* Debug access port unlock register */
#define STM32_BSEC_HDPLSR_OFFSET     0xe94 /* Hide protection level status register */

/* Register Addresses *******************************************************/

#define STM32_BSEC_DBGCR      (STM32_BSEC_BASE + STM32_BSEC_DBGCR_OFFSET)
#define STM32_BSEC_AP_UNLOCK  (STM32_BSEC_BASE + STM32_BSEC_AP_UNLOCK_OFFSET)
#define STM32_BSEC_HDPLSR     (STM32_BSEC_BASE + STM32_BSEC_HDPLSR_OFFSET)

/* Register Bitfield Definitions ********************************************/

#define BSEC_DBGCR_UNLOCK          (0xb4 << 8)   /* Non-secure debug authorization */
#define BSEC_DBGCR_AUTH_HDPL_SHIFT (16)          /* Bits 16-23: Debug protection level */
#define BSEC_DBGCR_AUTH_SEC        (0xb4u << 24) /* Secure debug authorization */
#define BSEC_AP_UNLOCK_UNLOCK      0xb4          /* Unlock the debug access port */
#define BSEC_HDPLSR_HDPL_MASK      0xff          /* Bits 0-7: Current protection level */

#endif /* __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_BSEC_H */
