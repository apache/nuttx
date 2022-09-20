/****************************************************************************
 * arch/arm/src/rtl8720c/ameba_nvic.c
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
#ifdef CONFIG_ARCH_CORTEXM33
#include <assert.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <string.h>
#include "sched/sched.h"
#include "chip.h"
#include "nvic.h"
#include "ram_vectors.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ROUND_DOWN(v, q)          (((v) / (q)) * (q))

/* Get a 32-bit version of the default priority */

#define NVIC_PRIORITY_DEFAULT32   (NVIC_SYSH_PRIORITY_DEFAULT << 24 | \
                                   NVIC_SYSH_PRIORITY_DEFAULT << 16 | \
                                   NVIC_SYSH_PRIORITY_DEFAULT << 8  | \
                                   NVIC_SYSH_PRIORITY_DEFAULT)
/* Given the address of a NVIC ENABLE register, this is the offset to
 * the corresponding NVIC CLEAR register.
 */
#define NVIC_ENABLE_OFFSET        (NVIC_IRQ0_31_ENABLE - NVIC_IRQ0_31_ENABLE)
#define NVIC_CLEAR_OFFSET         (NVIC_IRQ0_31_CLEAR  - NVIC_IRQ0_31_ENABLE)

/****************************************************************************
 * Private Function Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int (* __vectors[NR_IRQS - NVIC_IRQ_FIRST])(void);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nvic_irqinfo
 *
 * Description:
 *   Given an IRQ number, provide the register and bit setting to enable or
 *   disable the irq.
 *
 ****************************************************************************/

static int nvic_irqinfo(int irq, uintptr_t *regaddr, uint32_t *bit,
                        uintptr_t offset)
{
  DEBUGASSERT(irq >= NVIC_IRQ_MEMFAULT && irq < NR_IRQS);

  /* Check for external interrupt */

  if (irq >= NVIC_IRQ_FIRST)
    {
      irq      = irq - NVIC_IRQ_FIRST;
      *regaddr = NVIC_IRQ_ENABLE(irq) + offset;
      *bit     = (uint32_t)1 << (irq & 0x1f);
    }

  /* Handle processor exceptions.  Only a few can be disabled */

  else
    {
      *regaddr = NVIC_SYSHCON;
      if (irq == NVIC_IRQ_MEMFAULT)
        {
          *bit = NVIC_SYSHCON_MEMFAULTENA;
        }

      else if (irq == NVIC_IRQ_BUSFAULT)
        {
          *bit = NVIC_SYSHCON_BUSFAULTENA;
        }

      else if (irq == NVIC_IRQ_USAGEFAULT)
        {
          *bit = NVIC_SYSHCON_USGFAULTENA;
        }

      else if (irq == NVIC_IRQ_DBGMONITOR)
        {
          *regaddr = NVIC_DEMCR;
          *bit = NVIC_DEMCR_MONEN;
        }

      else if (irq == NVIC_IRQ_SYSTICK)
        {
          *regaddr = NVIC_SYSTICK_CTRL;
          *bit = NVIC_SYSTICK_CTRL_TICKINT;
        }

      else
        {
          return ERROR; /* Invalid or unsupported exception */
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void weak_function up_wic_initialize(void)
{
}

void weak_function up_wic_enable_irq(int irq)
{
}

void weak_function up_wic_disable_irq(int irq)
{
}

/****************************************************************************
 * Name: arm_ack_irq
 *
 * Description:
 *   Acknowledge the IRQ
 *
 ****************************************************************************/

void arm_ack_irq(int irq)
{
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   This function implements disabling of the device specified by 'irq'
 *   at the interrupt controller level if supported by the architecture
 *   (up_irq_save() supports the global level, the device level is hardware
 *   specific).
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  uintptr_t regaddr;
  uint32_t bit;
  if (nvic_irqinfo(irq, &regaddr, &bit, NVIC_CLEAR_OFFSET) == 0)
    {
      /* Modify the appropriate bit in the register to disable the interrupt.
       * For normal interrupts, we need to set the bit in the associated
       * Interrupt Clear Enable register.  For other exceptions, we need to
       * clear the bit in the System Handler Control and State Register.
       */

      if (irq >= NVIC_IRQ_FIRST)
        {
          putreg32(bit, regaddr);
        }

      else
        {
          modifyreg32(regaddr, bit, 0);
        }

      up_wic_disable_irq(irq);
    }
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   On many architectures, there are three levels of interrupt enabling: (1)
 *   at the global level, (2) at the level of the interrupt controller,
 *   and (3) at the device level.  In order to receive interrupts, they
 *   must be enabled at all three levels.
 *
 *   This function implements enabling of the device specified by 'irq'
 *   at the interrupt controller level if supported by the architecture
 *   (up_irq_restore() supports the global level, the device level is
 *   hardware specific).
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  uintptr_t regaddr;
  uint32_t bit;
  if (nvic_irqinfo(irq, &regaddr, &bit, NVIC_ENABLE_OFFSET) == 0)
    {
      /* Modify the appropriate bit in the register to enable the interrupt.
       * For normal interrupts, we need to set the bit in the associated
       * Interrupt Set Enable register.  For other exceptions, we need to
       * set the bit in the System Handler Control and State Register.
       */

      if (irq >= NVIC_IRQ_FIRST)
        {
          putreg32(bit, regaddr);
        }

      else
        {
          modifyreg32(regaddr, 0, bit);
        }

      up_wic_enable_irq(irq);
    }
}

/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 *   Set the priority of an IRQ.
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 ****************************************************************************/

int up_prioritize_irq(int irq, int priority)
{
  uint32_t regaddr;
  int shift;
  DEBUGASSERT(irq >= NVIC_IRQ_MEMFAULT && irq < NR_IRQS &&
              priority >= NVIC_SYSH_PRIORITY_MAX &&
              priority <= NVIC_SYSH_PRIORITY_MIN);
  if (irq < NVIC_IRQ_FIRST)
    {
      /* NVIC_SYSH_PRIORITY() maps {0..15} to one of three priority
       * registers (0-3 are invalid)
       */

      regaddr = NVIC_SYSH_PRIORITY(irq);
      irq    -= NVIC_IRQ_MEMFAULT;
    }

  else
    {
      /* NVIC_IRQ_PRIORITY() maps {0..} to one of many priority registers */

      irq    -= NVIC_IRQ_FIRST;
      regaddr = NVIC_IRQ_PRIORITY(irq);
    }

  shift = (irq & 3) << 3;
  modifyreg32(regaddr, 0xff << shift, priority << shift);
  return OK;
}

/****************************************************************************
 * Name: _up_doirq
 ****************************************************************************/

int _up_doirq(int irq, void *context, void *arg)
{
  if (irq < NVIC_IRQ_FIRST)
    {
      return ERROR;
    }

  __vectors[irq - NVIC_IRQ_FIRST]();
  return OK;
}

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  int i;

  /* Disable all interrupts */

  for (i = 0; i < NR_IRQS - NVIC_IRQ_FIRST; i += 32)
    {
      putreg32(0xffffffff, NVIC_IRQ_CLEAR(i));
    }

  /* Restore the NVIC vector location to local */

  memcpy(&__vectors, (void *) * (volatile uint32_t *)(NVIC_VECTAB)
         + NVIC_IRQ_FIRST * sizeof(uint32_t), sizeof(__vectors));

  /* Set the NVIC vector location in case _vectors not equal zero. */

  putreg32((uint32_t)_vectors, NVIC_VECTAB);
  /* Now bits[7-5] are available in each 8bits,
   * Take bits[7-6] as group priority, take bit[5] as subpriorities.
   */

  modifyreg32(NVIC_AIRCR, NVIC_AIRCR_VECTKEY_MASK | NVIC_AIRCR_PRIGROUP_MASK,
              NVIC_AIRCR_VECTKEY | (0x5 << NVIC_AIRCR_PRIGROUP_SHIFT));

  /* Set all interrupts (and exceptions) to the default priority */

  putreg32(NVIC_PRIORITY_DEFAULT32, NVIC_SYSH4_7_PRIORITY);
  putreg32(NVIC_PRIORITY_DEFAULT32, NVIC_SYSH8_11_PRIORITY);
  putreg32(NVIC_PRIORITY_DEFAULT32, NVIC_SYSH12_15_PRIORITY);
  for (i = 0; i < NR_IRQS - NVIC_IRQ_FIRST; i += 4)
    {
      putreg32(NVIC_PRIORITY_DEFAULT32, NVIC_IRQ_PRIORITY(i));
    }

  up_wic_initialize();
  /* Attach the SVCall and Hard Fault exception handlers.  The SVCall
   * exception is used for performing context switches; The Hard Fault
   * must also be caught because a SVCall may show up as a Hard Fault
   * under certain conditions.
   */

  irq_attach(NVIC_IRQ_SVCALL, arm_svcall, NULL);
  up_prioritize_irq(NVIC_IRQ_SVCALL, NVIC_SYSH_SVCALL_PRIORITY);
  irq_attach(NVIC_IRQ_HARDFAULT, arm_hardfault, NULL);

  /* Attach and enable the Memory Management Fault handler */

  irq_attach(NVIC_IRQ_MEMFAULT, arm_memfault, NULL);
  up_enable_irq(NVIC_IRQ_MEMFAULT);

  /* Attach and enable the external interrupts */

  for (i = NVIC_IRQ_FIRST; i < NR_IRQS; i++)
    {
      irq_attach(i, _up_doirq, NULL);
    }

  up_disable_irq(NVIC_IRQ_BUSFAULT);
  up_disable_irq(NVIC_IRQ_USAGEFAULT);

  /* And finally, enable interrupts */

  up_irq_enable();
}

void up_irq_attach_workaround(void)
{
  extern void exception_common(void);
  __vectors[NVIC_IRQ_WLAN - NVIC_IRQ_FIRST] =
    (void *) *((uint32_t *) *(volatile uint32_t *)NVIC_VECTAB +
    NVIC_IRQ_WLAN);
  *((uint32_t *) * (volatile uint32_t *)NVIC_VECTAB + NVIC_IRQ_WLAN) =
    (unsigned)exception_common;
  up_prioritize_irq(NVIC_IRQ_WLAN, NVIC_SYSH_PRIORITY_DEFAULT);
}

#endif
