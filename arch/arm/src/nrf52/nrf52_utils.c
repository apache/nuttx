/****************************************************************************
 * arch/arm/src/nrf52/nrf52_utils.c
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
#include "nrf52_irq.h"
#include "hardware/nrf52_utils.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_clrpend
 *
 * Description:
 *   Clear a pending interrupt at the NVIC.  This does not seem to be
 *   required for most interrupts.
 *
 *   This function is logically a part of nrf52_irq.c, but I will keep it in
 *   a separate file so that it will not increase the footprint on NRF52
 *   platforms that do not need this function.
 *
 ****************************************************************************/

void nrf52_clrpend(int irq)
{
  /* Check for external interrupt */

  if (irq >= NRF52_IRQ_EXTINT)
    {
      if (irq < (NRF52_IRQ_EXTINT + 32))
        {
          putreg32(1 << (irq - NRF52_IRQ_EXTINT), NVIC_IRQ0_31_CLRPEND);
        }
      else if (irq < NRF52_IRQ_NIRQS)
        {
          putreg32(1 << (irq - NRF52_IRQ_EXTINT - 32),
                   NVIC_IRQ32_63_CLRPEND);
        }
    }
}
