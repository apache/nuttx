/****************************************************************************
 * arch/arm/src/pp502x/pp502x_decodeirq.c
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
#include <assert.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/debug.h>

#include "chip.h"
#include "arm_internal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pp502x_lowestbit
 ****************************************************************************/

#ifndef CONFIG_SUPPRESS_INTERRUPTS
static int pp502x_lowestbit(uint32_t status)
{
  int bit;

  for (bit = 0; bit < 32; bit++)
    {
      if ((status & (1u << bit)) != 0)
        {
          break;
        }
    }

  return bit;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_decodeirq
 *
 * Description:
 *   Deliver the lowest numbered pending and enabled source, normal bank
 *   first.  The high bank cascade bit is not a source of its own.
 *
 ****************************************************************************/

uint32_t *arm_decodeirq(uint32_t *regs)
{
#ifdef CONFIG_SUPPRESS_INTERRUPTS
  err("ERROR: Unexpected IRQ\n");
  PANIC();
#else
  uint32_t status;
  int irq;

  status = getreg32(PP502X_INTC_CPU_STAT) &
           getreg32(PP502X_INTC_CPU_ENSTAT) & ~PP502X_INTC_HI_CASCADE;

  if (status != 0)
    {
      irq = pp502x_lowestbit(status);
    }
  else
    {
      status = getreg32(PP502X_INTC_HI_CPU_STAT) &
               getreg32(PP502X_INTC_HI_CPU_ENSTAT);
      if (status == 0)
        {
          return regs;
        }

      irq = PP502X_IRQ_HIBASE + pp502x_lowestbit(status);
    }

  regs = arm_doirq(irq, regs);
#endif

  return regs;
}
