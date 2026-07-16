/****************************************************************************
 * arch/arm/src/stm32h7/hardware/stm32h7xxxx_crc.h
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

#ifndef __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7XXXX_CRC_H
#define __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7XXXX_CRC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CRC register offsets *****************************************************/

#define STM32_CRC_DR_OFFSET                0x0000 /* Data Register */
#define STM32_CRC_IDR_OFFSET               0x0004 /* Data Input Register */
#define STM32_CRC_CR_OFFSET                0x0008 /* Control Register */
#define STM32_CRC_INIT_OFFSET              0x0010 /* Init Value Register */
#define STM32_CRC_POL_OFFSET               0x0014 /* Polynomial Register */

/* CRC register addresses ***************************************************/

#define STM32_CRC_DR           (STM32_CRC_BASE + STM32_CRC_DR_OFFSET)
#define STM32_CRC_IDR          (STM32_CRC_BASE + STM32_CRC_IDR_OFFSET)
#define STM32_CRC_CR           (STM32_CRC_BASE + STM32_CRC_CR_OFFSET)
#define STM32_CRC_INIT         (STM32_CRC_BASE + STM32_CRC_INIT_OFFSET)
#define STM32_CRC_POL          (STM32_CRC_BASE + STM32_CRC_POL_OFFSET)

/* CRC register bit definitions *********************************************/

/* CRC CR register */

#define CRC_CR_REVOUT_SHIFT       7
#define CRC_CR_REV_OUT_MASK       (1 << CRC_CR_REVOUT_SHIFT)
#define CRC_CR_REV_OUT_NONE       (0 << CRC_CR_REVOUT_SHIFT)
#define CRC_CR_REV_OUT            (1 << CRC_CR_REVOUT_SHIFT)
#define CRC_CR_REV_IN_SHIFT       5
#define CRC_CR_REV_IN_MASK        (0x3 << CRC_CR_REV_IN_SHIFT)
#define CRC_CR_REV_IN_NONE        (0x0 << CRC_CR_REV_IN_SHIFT)
#define CRC_CR_REV_IN_BYTE        (0x1 << CRC_CR_REV_IN_SHIFT)
#define CRC_CR_REV_IN_HALFWORD    (0x2 << CRC_CR_REV_IN_SHIFT)
#define CRC_CR_REV_IN_WORD        (0x3 << CRC_CR_REV_IN_SHIFT)
#define CRC_CR_POLYSIZE_SHIFT     3
#define CRC_CR_POLYSIZE_MASK      (0x3 << CRC_CR_POLYSIZE_SHIFT)
#define CRC_CR_POLYSIZE_32BIT     (0x0 << CRC_CR_POLYSIZE_SHIFT)
#define CRC_CR_POLYSIZE_16BIT     (0x1 << CRC_CR_POLYSIZE_SHIFT)
#define CRC_CR_POLYSIZE_8BIT      (0x2 << CRC_CR_POLYSIZE_SHIFT)
#define CRC_CR_POLYSIZE_7BIT      (0x3 << CRC_CR_POLYSIZE_SHIFT)
#define CRC_CR_RESET_SHIFT        0
#define CRC_CR_RESET              (1 << CRC_CR_RESET_SHIFT)

#endif /* __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7XXXX_CRC_H */
