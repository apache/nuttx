/****************************************************************************
 * arch/arm/src/efm32/efm32_bitband.h
 *
 *   Copyright (C) 2015 Pierre-noel Bouteville . All rights reserved.
 *   Authors: Pierre-noel Bouteville <pnb990@gmail.com>
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_EFM32_EFM32_BITBAND_H
#define __ARCH_ARM_SRC_EFM32_EFM32_BITBAND_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/efm32_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/


/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(CONFIG_EFM32_BITBAND)
inline void bitband_set_peripheral(uint32_t addr, uint32_t bit, uint32_t val);
inline uint32_t bitband_get_peripheral(uint32_t addr, uint32_t bit);
inline void bitband_set_sram(uint32_t addr, uint32_t bit, uint32_t val);
inline uint32_t bitband_get_sram(uint32_t addr, uint32_t bit);

#else

#   define bitband_set_peripheral(addr,bit,val)\
    modifyreg32(addr,~(1<<bit),(1<<bit))

#   define bitband_get_peripheral(addr,bit) (((getreg32(addr)) >> bit) & 1)

#   define bitband_set_sram(add,bit,val)\
    modifyreg32(addr,~(1<<bit),(1<<bit))

#   define bitband_get_sram(addr,bit) (((getreg32(addr)) >> bit) & 1)

#endif

#endif /* __ARCH_ARM_SRC_EFM32_EFM32_BITBAND_H */
