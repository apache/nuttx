/****************************************************************************
 * arch/tricore/src/common/tricore_irq.c
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
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/bits.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include "tricore_irq.h"
#include "tricore_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

static inline uint32_t tricore_srcr_read(int irq)
{
  return getreg32(TRICORE_IR_GET_SRC(irq));
}

static inline void tricore_srcr_write(int irq, uint32_t val)
{
  putreg32(val, TRICORE_IR_GET_SRC(irq));
}

void up_irq_enable(void)
{
  TRICORE_IRQ_ENABLE();
}

void up_irqinitialize(void)
{
  TRICORE_IRQ_ENABLE();
}

void up_disable_irq(int irq)
{
  uint32_t regval = tricore_srcr_read(irq);

  regval &= ~SRCR_SRE;
  tricore_srcr_write(irq, regval);
}

void up_enable_irq(int irq)
{
  uint32_t regval = tricore_srcr_read(irq);

  regval |= SRCR_CLRR | SRCR_IOVCLR;

#if defined(CONFIG_ARCH_CHIP_FAMILY_TC3X)
  regval |= SRCR_SWSCLR;
#endif

  regval &= ~SRCR_SRPN_MASK;
  regval |= ((uint32_t)irq << SRCR_SRPN_SHIFT) & SRCR_SRPN_MASK;

  regval &= ~SRCR_TOS_MASK;
  regval |= (TRICORE_DEFAULT_IR_TOS << SRCR_TOS_SHIFT) & SRCR_TOS_MASK;

  regval |= SRCR_SRE;
  tricore_srcr_write(irq, regval);
}

#ifdef CONFIG_ARCH_HAVE_IRQTRIGGER
void up_trigger_irq(int irq, cpu_set_t cpuset)
{
  uint32_t regval;

  (void)cpuset;

  regval = tricore_srcr_read(irq);
  regval |= SRCR_SETR;
  tricore_srcr_write(irq, regval);
}

#endif /* CONFIG_ARCH_HAVE_IRQTRIGGER */

void up_affinity_irq(int irq, cpu_set_t cpuset)
{
  uint32_t regval;

  up_disable_irq(irq);

  regval = tricore_srcr_read(irq);
  regval &= ~SRCR_TOS_MASK;
  regval |= ((uint32_t)cpuset << SRCR_TOS_SHIFT) & SRCR_TOS_MASK;
  tricore_srcr_write(irq, regval);

  up_enable_irq(irq);
}

int up_prioritize_irq(int irq, int priority)
{
  uint32_t regval;

  regval = tricore_srcr_read(irq);
  regval &= ~SRCR_SRPN_MASK;
  regval |= ((uint32_t)priority << SRCR_SRPN_SHIFT) & SRCR_SRPN_MASK;
  tricore_srcr_write(irq, regval);

  return 0;
}
