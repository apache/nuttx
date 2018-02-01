/****************************************************************************
 * arch/arm/src/efm32/efm32_bitband.c
 *
 *   Copyright (C) 2014 Bouteville Pierre-Noel. All rights reserved.
 *   Authors: Bouteville Pierre-Noel <pnb990@gmail.com>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "stdint.h"

#include "efm32_bitband.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_EFM32_BITBAND)

#ifndef EFM32_BITBAND_PER_BASE
#   error "EFM32_BITBAND_PER_BASE not declared bitband may be not supported?"
#endif

#ifndef EFM32_BITBAND_RAM_BASE
#   error "EFM32_BITBAND_RAM_BASE not declared bitband may be not supported?"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bitband_set_peripheral
 *
 * Description:
 *   Perform bit-band write operation on peripheral memory location.
 *
 *   Bit-banding provides atomic read-modify-write cycle for single bit
 *   modification. Please refer to the reference manual for further details
 *   about bit-banding.
 *
 * Note
 *   This function is only atomic on cores which fully support bitbanding.
 *
 * Input Parameters:
 *   addr     Peripheral address location to modify bit in.
 *   bit      Bit position to modify, 0-31.
 *   val      Value to set bit to, 0 or 1.
 *
 ****************************************************************************/

inline void bitband_set_peripheral(uint32_t addr, uint32_t bit, uint32_t val)
{
  uint32_t regval;
  regval = EFM32_BITBAND_PER_BASE + ((addr-EFM32_PER_MEM_BASE)*32) + (bit*4);

  *((volatile uint32_t *)regval) = (uint32_t)val;
}

/****************************************************************************
 * Name: bitband_get_peripheral
 *
 * Description:
 *   Perform bit-band operation on peripheral memory location.
 *
 *   This function reads a single bit from the peripheral bit-band alias region.
 *   Bit-banding provides atomic read-modify-write cycle for single bit
 *   modification. Please refer to the reference manual for further details
 *   about bit-banding.
 *
 * Note
 *   This function is only atomic on cores which fully support bitbanding.
 *
 * Input Parameters:
 *   addr     Peripheral address location to read.
 *   bit      Bit position to modify, 0-31.
 *
 * Returned Value:
 *   Return bit value read, 0 or 1.
 *
 ****************************************************************************/

inline uint32_t bitband_get_peripheral(uint32_t addr, uint32_t bit)
{
  uint32_t regval;
  regval = EFM32_BITBAND_PER_BASE + ((addr-EFM32_PER_MEM_BASE)*32) + (bit*4);

  return *((volatile uint32_t *)regval);
}

/****************************************************************************
 * Name: bitband_set_sram
 *
 * Description:
 *   Perform bit-band write operation on SRAM memory location.
 *
 *   Bit-banding provides atomic read-modify-write cycle for single bit
 *   modification. Please refer to the reference manual for further details
 *   about bit-banding.
 *
 * Note
 *   This function is only atomic on cores which fully support bitbanding.
 *
 * Input Parameters:
 *   addr     SRAM address location to modify bit in.
 *   bit      Bit position to modify, 0-31.
 *   val      Value to set bit to, 0 or 1.
 *
 ****************************************************************************/

inline void bitband_set_sram(uint32_t addr, uint32_t bit, uint32_t val)
{
  uint32_t regval;
  regval = EFM32_BITBAND_RAM_BASE + ((addr-EFM32_RAM_MEM_BASE)*32) + (bit*4);

  *((volatile uint32_t *)regval) = (uint32_t)val;
}

/****************************************************************************
 * Name: bitband_get_sram
 *
 * Description::
 *   Perform bit-band operation on SRAM memory location.
 *
 *   This function reads a single bit from the RAM bit-band alias region.
 *   Bit-banding provides atomic read-modify-write cycle for single bit
 *   modification. Please refer to the reference manual for further details
 *   about bit-banding.
 *
 * Note
 *   This function is only atomic on cores which fully support bitbanding.
 *
 * Input Parameters:
 *   addr     Peripheral address location to read.
 *   bit      Bit position to modify, 0-31.
 *
 * Returned Value:
 *   Return bit value read, 0 or 1.
 *
 ****************************************************************************/

inline uint32_t bitband_get_sram(uint32_t addr, uint32_t bit)
{
  uint32_t regval;
  regval = EFM32_BITBAND_RAM_BASE + ((addr-EFM32_RAM_MEM_BASE)*32) + (bit*4);

  return *((volatile uint32_t *)regval);
}
#endif
