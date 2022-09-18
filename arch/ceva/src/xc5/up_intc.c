/****************************************************************************
 * arch/ceva/src/xc5/up_intc.c
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
#include <nuttx/irq.h>

#include "cpm.h"
#include "up_internal.h"
#include "vintc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
  if (irq >= IRQ_VINT_FIRST)
    {
      /* Forward to the secondary interrupt controller */

      up_vintc_disable_irq(irq);
    }
  else if (irq >= IRQ_TRAP0)
    {
      switch (irq)
        {
        case IRQ_TRAP0:
          __asm__ __volatile__("rst #0x01, imaskt");
          break;
        case IRQ_TRAP1:
          __asm__ __volatile__("rst #0x02, imaskt");
          break;
        case IRQ_TRAP2:
          __asm__ __volatile__("rst #0x04, imaskt");
          break;
        case IRQ_TRAP3:
          __asm__ __volatile__("rst #0x08, imaskt");
          break;
        }
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
  /* Note: All INTx is enabled by REG_MODA_DEFAULT */

  if (irq >= IRQ_VINT_FIRST)
    {
      /* Forward to the secondary interrupt controller */

      up_vintc_enable_irq(irq);
    }
  else if (irq >= IRQ_TRAP0)
    {
      switch (irq)
        {
        case IRQ_TRAP0:
          __asm__ __volatile__("set #0x01, imaskt");
          break;
        case IRQ_TRAP1:
          __asm__ __volatile__("set #0x02, imaskt");
          break;
        case IRQ_TRAP2:
          __asm__ __volatile__("set #0x04, imaskt");
          break;
        case IRQ_TRAP3:
          __asm__ __volatile__("set #0x08, imaskt");
          break;
        }
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

#ifdef CONFIG_ARCH_IRQPRIO
int up_prioritize_irq(int irq, int priority)
{
  int ret = -EINVAL;

  if (irq >= IRQ_VINT_FIRST)
    {
      /* Forward to the secondary interrupt controller */

      ret = up_vintc_prioritize_irq(irq, priority);
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: up_trigger_irq
 *
 * Description:
 *   Trigger an IRQ by software. May not be supported by all architectures.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_HAVE_IRQTRIGGER
void up_trigger_irq(int irq, cpu_set_t cpuset)
{
  if (irq >= IRQ_VINT_FIRST)
    {
      /* Forward to the secondary interrupt controller */

      up_vintc_trigger_irq(irq);
    }
}
#endif /* CONFIG_ARCH_HAVE_IRQTRIGGER */

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  /* Initialize the secondary interrupt controller */

  up_vintc_initialize();

  /* Attach and enable SVCall exception handler */

  irq_attach(IRQ_TRAP0, up_svcall, NULL);
  up_enable_irq(IRQ_TRAP0);

  /* Attach and enable Hard Fault exception handler */

#if CONFIG_ARCH_HARDFAULT_IRQ >= 0
  irq_attach(CONFIG_ARCH_HARDFAULT_IRQ, up_hardfault, NULL);
  up_enable_irq(CONFIG_ARCH_HARDFAULT_IRQ);
#endif

  /* And finally, enable interrupts */

  up_irq_enable();
}
