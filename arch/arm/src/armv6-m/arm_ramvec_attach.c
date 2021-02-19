/****************************************************************************
 * arch/arm/src/armv6-m/arm_ramvec_attach.c
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

#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "ram_vectors.h"

#ifdef CONFIG_ARCH_RAMVECTORS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Common exception entrypoint */

void exception_common(void);

/****************************************************************************
 * Name: arm_ramvec_attach
 *
 * Description:
 *   Configure the ram vector table so that IRQ number 'irq' will be
 *   dispatched by hardware to 'vector'
 *
 ****************************************************************************/

int arm_ramvec_attach(int irq, up_vector_t vector)
{
  int ret = -EINVAL;

  irqinfo("%s IRQ%d\n", vector ? "Attaching" : "Detaching", irq);

  if ((unsigned)irq < ARMV6M_VECTAB_SIZE)
    {
      irqstate_t flags;

      /* If the new vector is NULL, then the vector is being detached. In
       * this case, disable the itnerrupt and direct any interrupts to the
       * common exception handler.
       */

      flags = enter_critical_section();
      if (vector == NULL)
        {
          /* Disable the interrupt if we can before detaching it.  We might
           * not be able to do this for all interrupts.
           */

          up_disable_irq(irq);

          /* Detaching the vector really means re-attaching it to the
           * common exception handler.
           */

           vector = exception_common;
        }

      /* Save the new vector in the vector table */

      g_ram_vectors[irq] = vector;
      leave_critical_section(flags);
      ret = OK;
    }

  return ret;
}

#endif /* CONFIG_ARCH_RAMVECTORS */
