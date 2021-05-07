/****************************************************************************
 * arch/arm/src/stm32f0l0g0/hardware/stm32_crc.h
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

#ifndef __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32_CRC_H
#define __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32_CRC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_CRC_DR_OFFSET     0x0000  /* Data register */
#define STM32_CRC_IDR_OFFSET    0x0004  /* Independent Data register */
#define STM32_CRC_CR_OFFSET     0x0008  /* Control register */
#define STM32_CRC_INIT_OFFSET   0x0010  /* Initial CRC value register */
#define STM32_CRC_POL_OFFSET    0x0014  /* CRC polynomial register */

/* Register Addresses *******************************************************/

#define STM32_CRC_DR            (STM32_CRC_BASE + STM32_CRC_DR_OFFSET)
#define STM32_CRC_IDR           (STM32_CRC_BASE + STM32_CRC_IDR_OFFSET)
#define STM32_CRC_CR            (STM32_CRC_BASE + STM32_CRC_CR_OFFSET)
#define STM32_CRC_INIT          (STM32_CRC_BASE + STM32_CRC_INIT_OFFSET)
#define STM32_CRC_POL           (STM32_CRC_BASE + STM32_CRC_POL_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* CRC independent data register */

#define CRC_IDR_MASK            0xff      /* These bits as a temporary location for one byte, not affected by RESET bit of CR */

/* CRC control register */

#define CRC_CR_RESET            (1 << 0)  /* This bit reset the CRC calculation unit and load CRC_DR with value of CRC_INIT */
#define CRC_CR_POLYSIZE_SHIFT   3         /* Bits 3-4: Polynomial size (for STM32F07x and STM32F09x) */
#define CRC_CR_POLYSIZE_MASK    (3 << CRC_CR_POLYSIZE_SHIFT)
#  define CRC_CR_POLYSIZE_32    (0 << CRC_CR_POLYSIZE_SHIFT) /* 00: 32 bit polynomial */
#  define CRC_CR_POLYSIZE_16    (1 << CRC_CR_POLYSIZE_SHIFT) /* 01: 16 bit polynomial */
#  define CRC_CR_POLYSIZE_8     (2 << CRC_CR_POLYSIZE_SHIFT) /* 10: 8 bit polynomial */
#  define CRC_CR_POLYSIZE_7     (3 << CRC_CR_POLYSIZE_SHIFT) /* 10: 8 bit polynomial */

#define CRC_CR_REVIN_SHIFT      5         /* Bits 5-6: These bits ontrol the reversal of the bit order of the input data */
#define CRC_CR_REVIN_MASK       (3 << CRC_CR_REVIN_SHIFT)
#  define CRC_CR_REVIN_NONE     (0 << CRC_CR_REVIN_SHIFT) /* 00: bit order is not affected */
#  define CRC_CR_REVIN_BYTE     (1 << CRC_CR_REVIN_SHIFT) /* 01: reversal done by byte */
#  define CRC_CR_REVIN_HWORD    (2 << CRC_CR_REVIN_SHIFT) /* 10: reversal done by half-word */
#  define CRC_CR_REVIN_WORD     (3 << CRC_CR_REVIN_SHIFT) /* 11: reversal done by word */

#define CRC_CR_REVOUT           (1 << 7)  /* This bit controls the reversal of the bit order of the output data */

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32_CRC_H */
