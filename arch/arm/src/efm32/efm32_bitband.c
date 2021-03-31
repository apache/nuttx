/****************************************************************************
 * arch/arm/src/efm32/efm32_bitband.c
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

inline void bitband_set_peripheral(uint32_t addr,
                                   uint32_t bit, uint32_t val)
{
  uint32_t regval;
  regval = EFM32_BITBAND_PER_BASE +
          ((addr - EFM32_PER_MEM_BASE) * 32) +
           (bit * 4);

  *((volatile uint32_t *)regval) = (uint32_t)val;
}

/****************************************************************************
 * Name: bitband_get_peripheral
 *
 * Description:
 *   Perform bit-band operation on peripheral memory location.
 *
 *   This function reads a single bit from the peripheral bit-band alias
 *   region.
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
  regval = EFM32_BITBAND_PER_BASE +
          ((addr - EFM32_PER_MEM_BASE) * 32) +
           (bit * 4);

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
  regval = EFM32_BITBAND_RAM_BASE +
           ((addr - EFM32_RAM_MEM_BASE) * 32) +
            (bit * 4);

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
  regval = EFM32_BITBAND_RAM_BASE +
           ((addr - EFM32_RAM_MEM_BASE) * 32) +
            (bit * 4);

  return *((volatile uint32_t *)regval);
}
#endif
