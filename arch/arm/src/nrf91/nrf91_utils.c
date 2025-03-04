/****************************************************************************
 * arch/arm/src/nrf91/nrf91_utils.c
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

#include <arch/irq.h>

#include "nvic.h"
#include "arm_internal.h"
#include "nrf91_irq.h"
#include "hardware/nrf91_utils.h"
#include "hardware/nrf91_memorymap.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf91_clrpend
 *
 * Description:
 *   Clear a pending interrupt at the NVIC.  This does not seem to be
 *   required for most interrupts.
 *
 *   This function is logically a part of nrf91_irq.c, but I will keep it in
 *   a separate file so that it will not increase the footprint on NRF91
 *   platforms that do not need this function.
 *
 ****************************************************************************/

void nrf91_clrpend(int irq)
{
  /* Check for external interrupt */

  if (irq >= NRF91_IRQ_EXTINT)
    {
      if (irq < (NRF91_IRQ_EXTINT + 32))
        {
          putreg32(1 << (irq - NRF91_IRQ_EXTINT), NVIC_IRQ0_31_CLRPEND);
        }
      else if (irq < NRF91_IRQ_NIRQS)
        {
          putreg32(1 << (irq - NRF91_IRQ_EXTINT - 32),
                   NVIC_IRQ32_63_CLRPEND);
        }
    }
}

/****************************************************************************
 * Name: nrf91_easydma_valid
 *
 * Description:
 *   Validate if easyDMA transfer is possible.
 *
 ****************************************************************************/

bool nrf91_easydma_valid(uint32_t addr)
{
#ifdef CONFIG_DEBUG_FEATURES
  /* EasyDMA cannot access flash memory */

  if (addr >= NRF91_FLASH_BASE && addr < NRF91_SRAM_BASE)
    {
      return false;
    }
#endif

  return true;
}
