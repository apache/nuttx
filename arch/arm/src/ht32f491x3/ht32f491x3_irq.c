/****************************************************************************
 * arch/arm/src/ht32f491x3/ht32f491x3_irq.c
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

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <arch/irq.h>
#include <arch/armv7-m/nvicpri.h>

#include "nvic.h"
#ifdef CONFIG_ARCH_RAMVECTORS
#  include "ram_vectors.h"
#endif
#include "arm_internal.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DEFPRIORITY32 \
  (NVIC_SYSH_PRIORITY_DEFAULT << 24 | \
   NVIC_SYSH_PRIORITY_DEFAULT << 16 | \
   NVIC_SYSH_PRIORITY_DEFAULT << 8  | \
   NVIC_SYSH_PRIORITY_DEFAULT)

#define NVIC_ENA_OFFSET    (0)
#define NVIC_CLRENA_OFFSET (NVIC_IRQ0_31_CLEAR - NVIC_IRQ0_31_ENABLE)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
static int ht32_nmi(int irq, FAR void *context, FAR void *arg)
{
  UNUSED(irq);
  UNUSED(context);
  UNUSED(arg);

  up_irq_save();
  _err("PANIC!!! NMI received\n");
  PANIC();
  return 0;
}

static int ht32_pendsv(int irq, FAR void *context, FAR void *arg)
{
  UNUSED(irq);
  UNUSED(context);
  UNUSED(arg);

  up_irq_save();
  _err("PANIC!!! PendSV received\n");
  PANIC();
  return 0;
}

static int ht32_dbgmonitor(int irq, FAR void *context, FAR void *arg)
{
  UNUSED(irq);
  UNUSED(context);
  UNUSED(arg);

  up_irq_save();
  _err("PANIC!!! Debug Monitor received\n");
  PANIC();
  return 0;
}

static int ht32_reserved(int irq, FAR void *context, FAR void *arg)
{
  UNUSED(irq);
  UNUSED(context);
  UNUSED(arg);

  up_irq_save();
  _err("PANIC!!! Reserved interrupt\n");
  PANIC();
  return 0;
}
#endif

static inline void ht32_prioritize_syscall(int priority)
{
  uint32_t regval;

  regval  = getreg32(NVIC_SYSH8_11_PRIORITY);
  regval &= ~NVIC_SYSH_PRIORITY_PR11_MASK;
  regval |= (priority << NVIC_SYSH_PRIORITY_PR11_SHIFT);
  putreg32(regval, NVIC_SYSH8_11_PRIORITY);
}

static int ht32_irqinfo(int irq, uintptr_t *regaddr, uint32_t *bit,
                        uintptr_t offset)
{
  int n;

  DEBUGASSERT(irq >= HT32_IRQ_NMI && irq < NR_IRQS);

  if (irq >= HT32_IRQ_FIRST)
    {
      n        = irq - HT32_IRQ_FIRST;
      *regaddr = NVIC_IRQ_ENABLE(n) + offset;
      *bit     = (uint32_t)1 << (n & 0x1f);
    }
  else
    {
      *regaddr = NVIC_SYSHCON;
      if (irq == HT32_IRQ_MEMFAULT)
        {
          *bit = NVIC_SYSHCON_MEMFAULTENA;
        }
      else if (irq == HT32_IRQ_BUSFAULT)
        {
          *bit = NVIC_SYSHCON_BUSFAULTENA;
        }
      else if (irq == HT32_IRQ_USAGEFAULT)
        {
          *bit = NVIC_SYSHCON_USGFAULTENA;
        }
      else if (irq == HT32_IRQ_SYSTICK)
        {
          *regaddr = NVIC_SYSTICK_CTRL;
          *bit = NVIC_SYSTICK_CTRL_ENABLE;
        }
      else
        {
          return -EINVAL;
        }
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_irqinitialize(void)
{
  uint32_t regaddr;
  int num_priority_registers;
  int i;

  for (i = 0; i < NR_IRQS - HT32_IRQ_FIRST; i += 32)
    {
      putreg32(0xffffffff, NVIC_IRQ_CLEAR(i));
    }

#if defined(__ICCARM__)
  putreg32((uint32_t)__vector_table, NVIC_VECTAB);
#else
  putreg32((uint32_t)_vectors, NVIC_VECTAB);
#endif

#ifdef CONFIG_ARCH_RAMVECTORS
  arm_ramvec_initialize();
#endif

  putreg32(DEFPRIORITY32, NVIC_SYSH4_7_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_SYSH8_11_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_SYSH12_15_PRIORITY);

  num_priority_registers = (getreg32(NVIC_ICTR) + 1) * 8;
  regaddr = NVIC_IRQ0_3_PRIORITY;

  while (num_priority_registers-- > 0)
    {
      putreg32(DEFPRIORITY32, regaddr);
      regaddr += 4;
    }

  irq_attach(HT32_IRQ_SVCALL, arm_svcall, NULL);
  irq_attach(HT32_IRQ_HARDFAULT, arm_hardfault, NULL);
  ht32_prioritize_syscall(NVIC_SYSH_SVCALL_PRIORITY);

#ifdef CONFIG_ARM_MPU
  irq_attach(HT32_IRQ_MEMFAULT, arm_memfault, NULL);
  up_enable_irq(HT32_IRQ_MEMFAULT);
#endif

#ifdef CONFIG_DEBUG_FEATURES
  irq_attach(HT32_IRQ_NMI, ht32_nmi, NULL);
#ifndef CONFIG_ARM_MPU
  irq_attach(HT32_IRQ_MEMFAULT, arm_memfault, NULL);
#endif
  irq_attach(HT32_IRQ_BUSFAULT, arm_busfault, NULL);
  irq_attach(HT32_IRQ_USAGEFAULT, arm_usagefault, NULL);
  irq_attach(HT32_IRQ_PENDSV, ht32_pendsv, NULL);
  irq_attach(HT32_IRQ_DBGMONITOR, ht32_dbgmonitor, NULL);
  irq_attach(HT32_IRQ_RESERVED, ht32_reserved, NULL);
#endif

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  arm_color_intstack();
  up_irq_enable();
#endif
}

void up_disable_irq(int irq)
{
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t bit;

  if (ht32_irqinfo(irq, &regaddr, &bit, NVIC_CLRENA_OFFSET) < 0)
    {
      return;
    }

  if (irq >= HT32_IRQ_FIRST)
    {
      putreg32(bit, regaddr);
    }
  else
    {
      regval  = getreg32(regaddr);
      regval &= ~bit;
      putreg32(regval, regaddr);
    }
}

void up_enable_irq(int irq)
{
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t bit;

  if (ht32_irqinfo(irq, &regaddr, &bit, NVIC_ENA_OFFSET) < 0)
    {
      return;
    }

  if (irq >= HT32_IRQ_FIRST)
    {
      putreg32(bit, regaddr);
    }
  else
    {
      regval  = getreg32(regaddr);
      regval |= bit;
      putreg32(regval, regaddr);
    }
}

void arm_ack_irq(int irq)
{
  UNUSED(irq);
}

#ifdef CONFIG_ARCH_IRQPRIO
int up_prioritize_irq(int irq, int priority)
{
  uint32_t regaddr;
  uint32_t regval;
  int shift;

  DEBUGASSERT(irq >= HT32_IRQ_MEMFAULT && irq < NR_IRQS &&
              (unsigned)priority <= NVIC_SYSH_PRIORITY_MIN);

  if (irq < HT32_IRQ_FIRST)
    {
      regaddr = NVIC_SYSH_PRIORITY(irq);
      irq    -= 4;
    }
  else
    {
      irq    -= HT32_IRQ_FIRST;
      regaddr = NVIC_IRQ_PRIORITY(irq);
    }

  regval      = getreg32(regaddr);
  shift       = ((irq & 3) << 3);
  regval     &= ~(0xff << shift);
  regval     |= ((uint32_t)priority << shift);
  putreg32(regval, regaddr);

  return 0;
}
#endif
