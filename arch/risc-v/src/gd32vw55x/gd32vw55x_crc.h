/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gd32vw55x_crc.h
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

#ifndef __ARCH_RISCV_SRC_GD32VW55X_GD32VW55X_CRC_H
#define __ARCH_RISCV_SRC_GD32VW55X_GD32VW55X_CRC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stddef.h>

#include "hardware/gd32vw55x_crc.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: gd32_crc_initialize
 *
 * Description:
 *   Enable the clock of the CRC calculation unit and reset the data
 *   register.  This must be called once before any of the calculation
 *   functions below.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_crc_initialize(void);

/****************************************************************************
 * Name: gd32_crc_reset
 *
 * Description:
 *   Reset the CRC data register to its initial value (0xffffffff).
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_crc_reset(void);

/****************************************************************************
 * Name: gd32_crc32_calculate
 *
 * Description:
 *   Compute the CRC-32 of an array of 32-bit words.  The data register is
 *   reset before the calculation, so this is a one-shot operation.
 *
 * Input Parameters:
 *   data - Pointer to an array of 32-bit words
 *   len  - Number of 32-bit words in the array
 *
 * Returned Value:
 *   The 32-bit CRC of the given data.
 *
 ****************************************************************************/

uint32_t gd32_crc32_calculate(const uint32_t *data, size_t len);

/****************************************************************************
 * Name: gd32_crc32_accumulate
 *
 * Description:
 *   Feed an array of 32-bit words into the CRC unit without resetting the
 *   data register first, accumulating on top of the previous result.
 *
 * Input Parameters:
 *   data - Pointer to an array of 32-bit words
 *   len  - Number of 32-bit words in the array
 *
 * Returned Value:
 *   The 32-bit CRC accumulated so far.
 *
 ****************************************************************************/

uint32_t gd32_crc32_accumulate(const uint32_t *data, size_t len);

/****************************************************************************
 * Name: gd32_crc_fdata_read / gd32_crc_fdata_write
 *
 * Description:
 *   Read and write the 8-bit general purpose free data register.  This
 *   register is not used by the CRC calculation itself.
 *
 ****************************************************************************/

uint8_t gd32_crc_fdata_read(void);
void gd32_crc_fdata_write(uint8_t fdata);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_GD32VW55X_GD32VW55X_CRC_H */
