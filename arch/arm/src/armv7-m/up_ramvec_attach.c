/****************************************************************************
 * arch/arm/irq/up_ramvec_attach.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
 * Name: up_ramvec_attach
 *
 * Description:
 *   Configure the ram vector table so that IRQ number 'irq' will be
 *   dispatched by hardware to 'vector'
 *
 ****************************************************************************/

int up_ramvec_attach(int irq, up_vector_t vector)
{
  int ret = -EINVAL;

  irqinfo("%s IRQ%d\n", vector ? "Attaching" : "Detaching", irq);

  if ((unsigned)irq < ARMV7M_VECTAB_SIZE)
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

#endif /* !CONFIG_ARCH_RAMVECTORS */
