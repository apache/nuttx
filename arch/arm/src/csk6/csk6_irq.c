/****************************************************************************
 * arch/arm/src/csk6/csk6_irq.c
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
#include <nuttx/arch.h>
#include <debug.h>
#include <arch/csk6/irq.h>

#include "arm_internal.h"
#include "nvic.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NVIC_ENA_OFFSET    (0)
#define NVIC_CLRENA_OFFSET (NVIC_IRQ0_31_CLEAR - NVIC_IRQ0_31_ENABLE)

#define DEFPRIORITY32      (NVIC_SYSH_PRIORITY_DEFAULT << 24 | \
                            NVIC_SYSH_PRIORITY_DEFAULT << 16 | \
                            NVIC_SYSH_PRIORITY_DEFAULT << 8  | \
                            NVIC_SYSH_PRIORITY_DEFAULT)

/* Size of the interrupt stack allocation */

#define INTSTACK_ALLOC     (CONFIG_SMP_NCPUS * INTSTACK_SIZE)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
static int csk6_nmi(int irq, void *context, void *arg)
{
  up_irq_save();
  _err("PANIC!!! NMI received\n");
  PANIC();
  return 0;
}

static int csk6_pendsv(int irq, void *context, void *arg)
{
  up_irq_save();
  _err("PANIC!!! PendSV received\n");
  PANIC();
  return 0;
}

static int csk6_reserved(int irq, void *context, void *arg)
{
  up_irq_save();
  _err("PANIC!!! Reserved interrupt\n");
  PANIC();
  return 0;
}
#endif

/****************************************************************************
 * Name: csk6_prioritize_syscall
 *
 * Description:
 *   Set the priority of an exception.  This function may be needed
 *   internally even if support for prioritized interrupts is not enabled.
 *
 ****************************************************************************/

static inline void csk6_prioritize_syscall(int priority)
{
  uint32_t regval;

  /* SVCALL is system handler 11 */

  regval  = getreg32(NVIC_SYSH8_11_PRIORITY);
  regval &= ~NVIC_SYSH_PRIORITY_PR11_MASK;
  regval |= (priority << NVIC_SYSH_PRIORITY_PR11_SHIFT);
  putreg32(regval, NVIC_SYSH8_11_PRIORITY);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 *
 * Description:
 *   This function is called by up_initialize() during the bring-up of the
 *   system.  It is the responsibility of this function to but the interrupt
 *   subsystem into the working and ready state.
 *
 ****************************************************************************/

void up_irqinitialize(void)
{
  uint32_t regaddr;
  int num_priority_registers;
  int i;

  /* Disable all interrupts */

  for (i = 0; i < NR_IRQS - CSK6_IRQ_FIRST; i += 32)
    {
      putreg32(0xffffffff, NVIC_IRQ_CLEAR(i));
    }

  putreg32((uint32_t)_vectors, NVIC_VECTAB);

#ifdef CONFIG_ARCH_RAMVECTORS
  /* If CONFIG_ARCH_RAMVECTORS is defined, then we are using a RAM-based
   * vector table that requires special initialization.
   */

  up_ramvec_initialize();
#endif

  /* Set all interrupts (and exceptions) to the default priority */

  putreg32(DEFPRIORITY32, NVIC_SYSH4_7_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_SYSH8_11_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_SYSH12_15_PRIORITY);

  /* The NVIC ICTR register (bits 0-4) holds the number of of interrupt
   * lines that the NVIC supports:
   *
   *  0 -> 32 interrupt lines,  8 priority registers
   *  1 -> 64 "       " "   ", 16 priority registers
   *  2 -> 96 "       " "   ", 32 priority registers
   *  ...
   */

  num_priority_registers = (getreg32(NVIC_ICTR) + 1) * 8;

  /* Now set all of the interrupt lines to the default priority */

  regaddr = NVIC_IRQ0_3_PRIORITY;
  while (num_priority_registers--)
    {
      putreg32(DEFPRIORITY32, regaddr);
      regaddr += 4;
    }

  /* Attach the SVCall and Hard Fault exception handlers.  The SVCall
   * exception is used for performing context switches; The Hard Fault
   * must also be caught because a SVCall may show up as a Hard Fault
   * under certain conditions.
   */

  irq_attach(CSK6_IRQ_SVCALL, arm_svcall, NULL);
  irq_attach(CSK6_IRQ_HARDFAULT, arm_hardfault, NULL);

  /* Set the priority of the SVCall interrupt */

#ifdef CONFIG_ARCH_IRQPRIO

  /* up_prioritize_irq(CSK6_IRQ_PENDSV, NVIC_SYSH_PRIORITY_MIN); */

#endif
  csk6_prioritize_syscall(NVIC_SYSH_SVCALL_PRIORITY);

  /* If the MPU is enabled, then attach and enable the Memory Management
   * Fault handler.
   */

#ifdef CONFIG_ARM_MPU
  irq_attach(CSK6_IRQ_MEMFAULT, arm_memfault, NULL);
  up_enable_irq(CSK6_IRQ_MEMFAULT);
#endif

  /* Attach all other processor exceptions (except reset and sys tick) */

#ifdef CONFIG_DEBUG_FEATURES
  irq_attach(CSK6_IRQ_NMI, csk6_nmi, NULL);
#ifndef CONFIG_ARM_MPU
  irq_attach(CSK6_IRQ_MEMFAULT, arm_memfault, NULL);
#endif
  irq_attach(CSK6_IRQ_BUSFAULT, arm_busfault, NULL);
  irq_attach(CSK6_IRQ_USAGEFAULT, arm_usagefault, NULL);
  irq_attach(CSK6_IRQ_PENDSV, csk6_pendsv, NULL);
  arm_enable_dbgmonitor();
  irq_attach(CSK6_IRQ_DBGMONITOR, arm_dbgmonitor, NULL);
  irq_attach(CSK6_IRQ_RESERVED, csk6_reserved, NULL);
#endif

#ifndef CONFIG_SUPPRESS_INTERRUPTS

  /* And finally, enable interrupts */

  up_irq_enable();
#endif
}

static int csk6_irqinfo(int irq, uintptr_t *regaddr, uint32_t *bit,
                             uintptr_t offset)
{
  int n;

  DEBUGASSERT(irq >= CSK6_IRQ_NMI && irq < NR_IRQS);

  /* Check for external interrupt */

  if (irq >= CSK6_IRQ_FIRST)
    {
      n        = irq - CSK6_IRQ_FIRST;
      *regaddr = NVIC_IRQ_ENABLE(n) + offset;
      *bit     = (uint32_t)0x1 << (n & 0x1f);
    }

  /* Handle processor exceptions.  Only a few can be disabled */

  else
    {
      *regaddr = NVIC_SYSHCON;
      if (irq == CSK6_IRQ_MEMFAULT)
        {
          *bit = NVIC_SYSHCON_MEMFAULTENA;
        }
      else if (irq == CSK6_IRQ_BUSFAULT)
        {
          *bit = NVIC_SYSHCON_BUSFAULTENA;
        }
      else if (irq == CSK6_IRQ_USAGEFAULT)
        {
          *bit = NVIC_SYSHCON_USGFAULTENA;
        }
      else if (irq == CSK6_IRQ_SYSTICK)
        {
          *regaddr = NVIC_SYSTICK_CTRL;
          *bit = NVIC_SYSTICK_CTRL_ENABLE;
        }
      else
        {
          return -EINVAL; /* Invalid or unsupported exception */
        }
    }

  return OK;
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t bit;

  if (csk6_irqinfo(irq, &regaddr, &bit, NVIC_CLRENA_OFFSET) == 0)
    {
      /* Modify the appropriate bit in the register to disable the interrupt.
       * For normal interrupts, we need to set the bit in the associated
       * Interrupt Clear Enable register.  For other exceptions, we need to
       * clear the bit in the System Handler Control and State Register.
       */

      if (irq >= CSK6_IRQ_FIRST)
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
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t bit;

  if (csk6_irqinfo(irq, &regaddr, &bit, NVIC_ENA_OFFSET) == 0)
    {
      /* Modify the appropriate bit in the register to enable the interrupt.
       * For normal interrupts, we need to set the bit in the associated
       * Interrupt Set Enable register.  For other exceptions, we need to
       * set the bit in the System Handler Control and State Register.
       */

      if (irq >= CSK6_IRQ_FIRST)
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
}

/****************************************************************************
 * Name: arm_ack_irq
 ****************************************************************************/

void arm_ack_irq(int irq)
{
}
