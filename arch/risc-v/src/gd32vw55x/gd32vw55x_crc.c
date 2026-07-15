/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gd32vw55x_crc.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stddef.h>

#include <nuttx/mutex.h>

#include "riscv_internal.h"
#include "hardware/gd32vw55x_crc.h"
#include "hardware/gd32vw55x_rcu.h"
#include "gd32vw55x_crc.h"

#ifdef CONFIG_GD32VW55X_CRC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The GD32VW55x CRC unit implements the CRC-32/MPEG-2 algorithm:
 *
 *   Polynomial : 0x04c11db7
 *   Initial    : 0xffffffff
 *   Input      : 32-bit words, MSB first, not reflected
 *   Output     : not reflected, not inverted
 *
 * Neither the polynomial nor the initial value is programmable.  Data is
 * fed one 32-bit word at a time by writing the CRC_DATA register; the
 * result of the calculation is obtained by reading the same register back.
 *
 * There is a single CRC unit in the chip, so the calculation functions
 * serialize access with a mutex.  They must be called from a task context
 * (not from an interrupt handler).
 */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mutex_t g_crc_lock = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_crc_initialize
 *
 * Description:
 *   Enable the clock of the CRC calculation unit and reset the data
 *   register.
 *
 ****************************************************************************/

void gd32_crc_initialize(void)
{
  uint32_t regval;

  /* Enable the CRC peripheral clock (AHB1) */

  regval  = getreg32(GD32VW55X_RCU_AHB1EN);
  regval |= RCU_AHB1EN_CRCEN;
  putreg32(regval, GD32VW55X_RCU_AHB1EN);

  /* Reset the data register to 0xffffffff */

  putreg32(CRC_CTL_RST, GD32VW55X_CRC_CTL);
}

/****************************************************************************
 * Name: gd32_crc_reset
 *
 * Description:
 *   Reset the CRC data register to its initial value (0xffffffff).
 *
 ****************************************************************************/

void gd32_crc_reset(void)
{
  putreg32(CRC_CTL_RST, GD32VW55X_CRC_CTL);
}

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

uint32_t gd32_crc32_accumulate(const uint32_t *data, size_t len)
{
  uint32_t crc;
  size_t i;
  int ret;

  ret = nxmutex_lock(&g_crc_lock);
  if (ret < 0)
    {
      return 0;
    }

  for (i = 0; i < len; i++)
    {
      putreg32(data[i], GD32VW55X_CRC_DATA);
    }

  crc = getreg32(GD32VW55X_CRC_DATA);

  nxmutex_unlock(&g_crc_lock);
  return crc;
}

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

uint32_t gd32_crc32_calculate(const uint32_t *data, size_t len)
{
  uint32_t crc;
  size_t i;
  int ret;

  ret = nxmutex_lock(&g_crc_lock);
  if (ret < 0)
    {
      return 0;
    }

  /* Start from the initial value 0xffffffff */

  putreg32(CRC_CTL_RST, GD32VW55X_CRC_CTL);

  for (i = 0; i < len; i++)
    {
      putreg32(data[i], GD32VW55X_CRC_DATA);
    }

  crc = getreg32(GD32VW55X_CRC_DATA);

  nxmutex_unlock(&g_crc_lock);
  return crc;
}

/****************************************************************************
 * Name: gd32_crc_fdata_read
 *
 * Description:
 *   Read the 8-bit general purpose free data register.
 *
 ****************************************************************************/

uint8_t gd32_crc_fdata_read(void)
{
  return (uint8_t)(getreg32(GD32VW55X_CRC_FDATA) & CRC_FDATA_MASK);
}

/****************************************************************************
 * Name: gd32_crc_fdata_write
 *
 * Description:
 *   Write the 8-bit general purpose free data register.
 *
 ****************************************************************************/

void gd32_crc_fdata_write(uint8_t fdata)
{
  putreg32((uint32_t)fdata, GD32VW55X_CRC_FDATA);
}

#endif /* CONFIG_GD32VW55X_CRC */
