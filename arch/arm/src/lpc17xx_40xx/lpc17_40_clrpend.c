/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/lpc17_40_clrpend.c
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
#include "lpc17_40_clrpend.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_40_clrpend
 *
 * Description:
 *   Clear a pending interrupt at the NVIC.  This does not seem to be
 *   required for most interrupts.  Don't know why... but the LPC1766
 *   Ethernet EMAC interrupt definitely needs it!
 *
 *   I keep it in a separate file so that it will not increase the footprint
 *   on LPC17xx/LPC40xx platforms that do not need this function.
 *
 ****************************************************************************/

void lpc17_40_clrpend(int irq)
{
  /* Check for external interrupt */

  if (irq >= LPC17_40_IRQ_EXTINT)
    {
      if (irq < (LPC17_40_IRQ_EXTINT + 32))
        {
          putreg32(1 << (irq - LPC17_40_IRQ_EXTINT), NVIC_IRQ0_31_CLRPEND);
        }
      else if (irq < LPC17_40_IRQ_NIRQS)
        {
          putreg32(1 << (irq - LPC17_40_IRQ_EXTINT - 32),
                   NVIC_IRQ32_63_CLRPEND);
        }
    }
}
