/****************************************************************************
 * arch/misoc/src/lm32/lm32_irq.c
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
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/irq.h>

#include "chip.h"
#include "lm32.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  /* Attach the software interrupt */

  irq_attach(LM32_IRQ_SWINT, lm32_swint, NULL);

  /* Enable interrupts */

  irq_setie(1);
}

/****************************************************************************
 * Name: up_irq_save
 *
 * Description:
 *   Return the current interrupt enable state and disable all interrupts.
 *
 ****************************************************************************/

irqstate_t up_irq_save(void)
{
  irqstate_t flags;

  /* Get the previous value of IE */

  flags = irq_getie();

  /* Disable interrupts and return the previous interrupt state */

  irq_setie(0);
  return flags;
}

/****************************************************************************
 * Name: up_irq_restore
 *
 * Description:
 *   Restore saved interrupt state
 *
 ****************************************************************************/

void up_irq_restore(irqstate_t flags)
{
  /* Restore the interrupt state returned by up_save_irq() */

  irq_setie(flags);
}

/****************************************************************************
 * Name: up_irq_enable
 *
 * Description:
 *   Return the current interrupt enable state and enable all interrupts
 *
 ****************************************************************************/

irqstate_t up_irq_enable(void)
{
  irqstate_t flags;

  /* Get the previous value of IE */

  flags = irq_getie();

  /* Enable interrupts and return the previous interrupt state */

  irq_setie(1);
  return flags;
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
  irqstate_t flags;

  DEBUGASSERT(irq >= 0 && irq < NR_IRQS);

  /* Ignore any attempt to disable software interrupts */

  if (irq < LM32_NINTERRUPTS)
    {
      /* Disable interrupts by clearing the bit that corresponds to the irq */

      flags  = irq_getmask();
      flags &= ~(1 << irq);
      irq_setmask(flags);
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
  irqstate_t flags;
  DEBUGASSERT(irq >= 0 && irq < NR_IRQS);

  /* Ignore any attempt to enable software interrupts */

  if (irq < LM32_NINTERRUPTS)
    {
      /* Enable interrupts by setting the bit that corresponds to the irq */

      flags  = irq_getmask();
      flags |= (1 << irq);
      irq_setmask(flags);
    }
}
