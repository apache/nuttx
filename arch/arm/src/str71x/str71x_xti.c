/****************************************************************************
 * arch/arm/src/str71x/str71x_xti.c
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
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "str71x.h"

#ifdef CONFIG_STR71X_XTI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct xtiregs_s
{
  uint32_t mr; /* Mask register */
  uint32_t tr; /* Trigger polarity register */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

static const struct xtiregs_s g_xtiregs[2] =
{
  { STR71X_XTI_MRL, STR71X_XTI_TRL },
  { STR71X_XTI_MRH, STR71X_XTI_TRH }
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: str71x_xtiinterrupt
 *
 * Description:
 *   Dispatch an XTI interrupt.
 *
 ****************************************************************************/

static int str71x_xtiinterrupt(int irq, void *context, void *arg)
{
  uint16_t enabled = (uint16_t)getreg8(STR71X_XTI_MRH) << 8 |
                     (uint16_t)getreg8(STR71X_XTI_MRL);
  uint16_t pending = (uint16_t)getreg8(STR71X_XTI_PRH) << 8 |
                     (uint16_t)getreg8(STR71X_XTI_PRL);
  uint16_t mask;

  /* Dispatch the interrupts, the actions performed by the interrupt
   * handlers should clear the interrupt at the external source of the
   * interrupt.  We need to clear the interrupts at the source before
   * clearing the pending interrupts (see below).
   */

  pending &= enabled;

  for (irq = STR71X_IRQ_FIRSTXTI, mask = 0x0001;
       irq < NR_IRQS && pending != 0;
       irq++, mask <<= 1)
    {
      /* Is this interrupt pending? */

      if ((pending & mask) != 0)
        {
          /* Deliver the IRQ */

          irq_dispatch(irq, context);
          pending &= ~mask;
        }
    }

  /* Clear the pending interrupts.  This should be safe: "it is necessary to
   * clear at least one pending bit: this operation allows a rising edge to
   * be generated on the internal line (if there is at least one more pending
   * bit set and not masked) and so to set the interrupt controller pending
   * bit again.
   */

  putreg8(0, STR71X_XTI_PRH);
  putreg8(0, STR71X_XTI_PRL);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: str71x_xtiinitialize
 *
 * Description:
 *   Configure XTI for operation.
 *   Note that the lines are not used as wake-up sources in this
 *   implementation.  Some extensions would be required for that
 *   capability.
 *
 ****************************************************************************/

int str71x_xtiinitialize(void)
{
  int ret;

  /* Mask all interrupts by setting XTI MRH/L to zero */

  putreg8(0, STR71X_XTI_MRH);
  putreg8(0, STR71X_XTI_MRL);

  /* Clear any pending interrupts by setting XTI PRH/L to zero */

  putreg8(0, STR71X_XTI_PRH);
  putreg8(0, STR71X_XTI_PRL);

  /* Attach the XTI interrupt */

  ret = irq_attach(STR71X_IRQ_XTI, str71x_xtiinterrupt, NULL);
  if (ret == OK)
    {
      /* Enable the XTI interrupt at the XTI */

      putreg8(STR71X_XTICTRL_ID1S, STR71X_XTI_CTRL);

      /* And enable the XTI interrupt at the interrupt controller */

      up_enable_irq(STR71X_IRQ_XTI);
    }

  return ret;
}

/****************************************************************************
 * Name: str71x_xticonfig
 *
 * Description:
 *   Configure an external line to provide interrupts.
 *
 ****************************************************************************/

int str71x_xticonfig(int irq, bool rising)
{
  uint8_t regval;
  int bit;
  int ndx;
  int ret = -EINVAL;

  /* Configure one of the 16 lines as an interrupt source */

  if (irq >= STR71X_IRQ_FIRSTXTI && irq <= NR_IRQS)
    {
      /* Make sure that the interrupt is disabled */

      str71x_disable_xtiirq(irq);

      /* Decide if we use the lower or upper register */

      bit = irq - STR71X_IRQ_FIRSTXTI;
      ndx = 0;
      if (bit > 7)
        {
          /* Select the high register */

          bit  -= 8;
          ndx  = 1;
        }

      /* Set the rising or trailing edge */

      regval = getreg8(g_xtiregs[ndx].tr);
      if (rising)
        {
          regval |= (1 << bit);
        }
      else
        {
          regval &= ~(1 << bit);
        }

      putreg8(regval, g_xtiregs[ndx].tr);

      /* Return success */

      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: str71x_enable_xtiirq
 *
 * Description:
 *   Enable an external interrupt.
 *
 ****************************************************************************/

void str71x_enable_xtiirq(int irq)
{
  uint8_t regval;
  int bit;
  int ndx;

  /* Enable the external interrupt */

  if (irq >= STR71X_IRQ_FIRSTXTI && irq <= NR_IRQS)
    {
      /* Decide if we use the lower or upper register */

      bit = irq - STR71X_IRQ_FIRSTXTI;
      ndx = 0;
      if (bit > 7)
        {
          /* Select the high register */

          bit  -= 8;
          ndx  = 1;
        }

      /* Enable the interrupt be setting the corresponding mask bit
       * the XTI_MRL/H register.
       */

      regval = getreg8(g_xtiregs[ndx].mr);
      regval |= (1 << bit);
      putreg8(regval, g_xtiregs[ndx].mr);
    }
}

/****************************************************************************
 * Name: str71x_disable_xtiirq
 *
 * Description:
 *   Disable an external interrupt.
 *
 ****************************************************************************/

void str71x_disable_xtiirq(int irq)
{
  uint8_t regval;
  int bit;
  int ndx;

  /* Disable the external interrupt */

  if (irq >= STR71X_IRQ_FIRSTXTI && irq <= NR_IRQS)
    {
      /* Decide if we use the lower or upper register */

      bit = irq - STR71X_IRQ_FIRSTXTI;
      ndx = 0;
      if (bit > 7)
        {
          /* Select the high register */

          bit  -= 8;
          ndx  = 1;
        }

      /* Disable the interrupt be clearing the corresponding mask bit
       * the XTI_MRL/H register.
       */

      regval  = getreg8(g_xtiregs[ndx].mr);
      regval &= ~(1 << bit);
      putreg8(regval, g_xtiregs[ndx].mr);
    }
}

#endif /* CONFIG_STR71X_XTI */
