/************************************************************************************
 * arch/arm/src/stm32f0l0g0/hardware/stm32_crc.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Alan Carvalho de Assis <acassis@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32_CRC_H
#define __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32_CRC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define STM32_CRC_DR_OFFSET     0x0000  /* Data register */
#define STM32_CRC_IDR_OFFSET    0x0004  /* Independent Data register */
#define STM32_CRC_CR_OFFSET     0x0008  /* Control register */
#define STM32_CRC_INIT_OFFSET   0x0010  /* Initial CRC value register */
#define STM32_CRC_POL_OFFSET    0x0014  /* CRC polynomial register */

/* Register Addresses ***************************************************************/

#define STM32_CRC_DR            (STM32_CRC_BASE + STM32_CRC_DR_OFFSET)
#define STM32_CRC_IDR           (STM32_CRC_BASE + STM32_CRC_IDR_OFFSET)
#define STM32_CRC_CR            (STM32_CRC_BASE + STM32_CRC_CR_OFFSET)
#define STM32_CRC_INIT          (STM32_CRC_BASE + STM32_CRC_INIT_OFFSET)
#define STM32_CRC_POL           (STM32_CRC_BASE + STM32_CRC_POL_OFFSET)

/* Register Bitfield Definitions ****************************************************/

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

#endif /* __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32_CRC_H */
