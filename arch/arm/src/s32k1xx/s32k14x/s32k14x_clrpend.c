/****************************************************************************
 * arch/arm/src/s32k1xx/s32k14x/s32k14x_clrpend.c
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
#include "s32k14x_irq.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k14x_clrpend
 *
 * Description:
 *   Clear a pending interrupt at the NVIC.  This does not seem to be
 *   required for most interrupts.  Don't know why...
 *   but the S32K14x Ethernet EMAC interrupt definitely needs it!
 *
 *   This function is logically a part of s32k14x_irq.c, but I will keep it
 *   in a separate file so that it will not increase the footprint on S32K14x
 *   platforms that do not need this function.
 *
 ****************************************************************************/

void s32k14x_clrpend(int irq)
{
  /* Check for external interrupt */

  if (irq >= S32K1XX_IRQ_INTERRUPT)
    {
      irq -= S32K1XX_IRQ_INTERRUPT;
      if (irq < 32)
        {
          putreg32(1 << irq , NVIC_IRQ0_31_CLRPEND);
        }
      else if (irq < 64)
        {
          putreg32(1 << (irq - 32), NVIC_IRQ32_63_CLRPEND);
        }
      else if (irq < 96)
        {
          putreg32(1 << (irq - 64), NVIC_IRQ64_95_CLRPEND);
        }
      else if (irq < 128)
        {
          putreg32(1 << (irq - 96), NVIC_IRQ96_127_CLRPEND);
        }
      else if (irq < S32K1XX_IRQ_NIRQS)
        {
          putreg32(1 << (irq - 128), NVIC_IRQ128_159_CLRPEND);
        }
    }
}
