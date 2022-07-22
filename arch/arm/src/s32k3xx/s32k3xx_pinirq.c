/****************************************************************************
 * arch/arm/src/s32k3xx/s32k3xx_pinirq.c
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

/* Copyright 2022 NXP */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/board/board.h>
#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "arm_internal.h"

#include <s32k3xx_pin.h>
#include <hardware/s32k3xx_siul2.h>
#include <hardware/s32k3xx_wkpu.h>

#ifdef CONFIG_S32K3XX_GPIOIRQ

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define EIRQ_IMCR_FIRST  528 /* First EIRQ IMCR index */
#define EIRQ_IMCR_LAST   559 /* Last EIRQ IMCR index */
#define WKPU_SRC_OFFSET  4   /* Wakeup Source 0-3 are internal sources */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct s32k3xx_pinirq_s
{
  xcpt_t handler;
  void *arg;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Interrupt vectors.  To keep the memory usage at a minimum, the logic may
 * be configured per module.
 */

#ifdef CONFIG_S32K3XX_EIRQINTS
static struct s32k3xx_pinirq_s g_eirqisrs[32];
#endif
#ifdef CONFIG_S32K3XX_WKPUINTS
static struct s32k3xx_pinirq_s g_wkpuisrs[60];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k3xx_eirqinterrupt
 *
 * Description:
 *   External Interrupt (EIRQ) interrupt handling.
 *
 ****************************************************************************/

#ifdef CONFIG_S32K3XX_EIRQINTS
static int s32k3xx_eirqinterrupt(int irq, void *context, void *arg)
{
  uint32_t disr0;
  uint32_t direr0;
  uint32_t eif;
  int i;

  /* Find interrupt flags for all enabled interrupts */

  disr0  = getreg32(S32K3XX_SIUL2_DISR0);
  direr0 = getreg32(S32K3XX_SIUL2_DIRER0);

  eif = disr0 & direr0;

  /* Examine each EIRQ channel */

  for (i = 0; (i < 32) && (eif != 0); i++)
    {
      uint32_t bit = (1 << i);
      if ((eif & bit) != 0)
        {
          if (g_eirqisrs[i].handler != NULL)
            {
              xcpt_t handler  = g_eirqisrs[i].handler;
              void  *argument = g_eirqisrs[i].arg;

              /* There is a registered interrupt handler... invoke it */

              handler(irq, context, argument);
            }

          /* Writing a one to the DISR0 register will clear the pending
           * interrupt.
           */

          eif &= ~bit;
          putreg32((1 << i), S32K3XX_SIUL2_DISR0);
        }
    }

  return OK;
}
#endif /* CONFIG_S32K3XX_EIRQINTS */

/****************************************************************************
 * Name: s32k3xx_wkpuinterrupt
 *
 * Description:
 *   Wakeup Unit (WKPU) interrupt handling.
 *
 ****************************************************************************/

#ifdef CONFIG_S32K3XX_WKPUINTS
static int s32k3xx_wkpuinterrupt(int irq, void *context, void *arg)
{
  uint32_t wisr;
  uint32_t irer;
  uint32_t wisr_64;
  uint32_t irer_64;
  uint64_t eif;
  int i;

  /* Find interrupt flags for all enabled interrupts */

  wisr    = getreg32(S32K3XX_WKPU_WISR);
  irer    = getreg32(S32K3XX_WKPU_IRER);
  wisr_64 = getreg32(S32K3XX_WKPU_WISR_64);
  irer_64 = getreg32(S32K3XX_WKPU_IRER_64);

  eif = (wisr & irer) | (((uint64_t) (wisr_64 & irer_64)) << 32);

  /* Examine each WKPU source */

  for (i = WKPU_SRC_OFFSET; (i < 64) && (eif != 0); i++)
    {
      uint64_t bit = (1ULL << i);
      if ((eif & bit) != 0)
        {
          unsigned int index = i - WKPU_SRC_OFFSET; /* g_wkpuisrs only contains IRQ handlers for external WKPU sources */

          if (g_wkpuisrs[index].handler != NULL)
            {
              xcpt_t handler  = g_wkpuisrs[index].handler;
              void  *argument = g_wkpuisrs[index].arg;

              /* There is a registered interrupt handler... invoke it */

              handler(irq, context, argument);
            }

          /* Writing a one to the WISR register will clear the pending
           * interrupt.
           */

          eif &= ~bit;
          if (i < 32)
            {
              putreg32((1 << i), S32K3XX_WKPU_WISR);
            }
          else
            {
              putreg32((1 << (i - 32)), S32K3XX_WKPU_WISR_64);
            }
        }
    }

  return OK;
}
#endif /* CONFIG_S32K3XX_WKPUINTS */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k3xx_pinirq_initialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   GPIO pins.
 *
 ****************************************************************************/

void s32k3xx_pinirq_initialize(void)
{
#ifdef CONFIG_S32K3XX_EIRQINTS
  irq_attach(S32K3XX_IRQ_SIUL2_VEC0, s32k3xx_eirqinterrupt, NULL);
  putreg8(0xff, S32K3XX_SIUL2_DISR0_IRQ0);
  up_enable_irq(S32K3XX_IRQ_SIUL2_VEC0);

  irq_attach(S32K3XX_IRQ_SIUL2_VEC1, s32k3xx_eirqinterrupt, NULL);
  putreg8(0xff, S32K3XX_SIUL2_DISR0_IRQ1);
  up_enable_irq(S32K3XX_IRQ_SIUL2_VEC1);

  irq_attach(S32K3XX_IRQ_SIUL2_VEC2, s32k3xx_eirqinterrupt, NULL);
  putreg8(0xff, S32K3XX_SIUL2_DISR0_IRQ2);
  up_enable_irq(S32K3XX_IRQ_SIUL2_VEC2);

  irq_attach(S32K3XX_IRQ_SIUL2_VEC3, s32k3xx_eirqinterrupt, NULL);
  putreg8(0xff, S32K3XX_SIUL2_DISR0_IRQ3);
  up_enable_irq(S32K3XX_IRQ_SIUL2_VEC3);
#endif /* CONFIG_S32K3XX_EIRQINTS */

#ifdef CONFIG_S32K3XX_WKPUINTS
  irq_attach(S32K3XX_IRQ_WKPU, s32k3xx_wkpuinterrupt, NULL);
  putreg32(0xffffffff, S32K3XX_WKPU_WISR);
  putreg32(0xffffffff, S32K3XX_WKPU_WISR_64);
  up_enable_irq(S32K3XX_IRQ_WKPU);
#endif /* CONFIG_S32K3XX_WKPUINTS */
}

/****************************************************************************
 * Name: s32k3xx_pinirqattach
 *
 * Description:
 *   Attach a pin interrupt handler.  The normal initialization sequence is:
 *
 *   1. Call s32k3xx_pinconfig() to configure the interrupting pin (pin
 *      interrupts will be disabled).
 *   2. Call s32k3xx_pinirqattach() to attach the pin interrupt handling
 *      function.
 *   3. Call s32k3xx_pinirqenable() to enable interrupts on the pin.
 *
 * Input Parameters:
 *   pinset - Pin configuration
 *   pinisr - Pin interrupt service routine
 *   arg    - An argument that will be provided to the interrupt service
 *            routine.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int s32k3xx_pinirqattach(uint32_t pinset, xcpt_t pinisr, void *arg)
{
  unsigned int index;
  irqstate_t flags;

#ifdef CONFIG_S32K3XX_EIRQINTS
  unsigned int imcr = (pinset & _IMCR_MASK) >> _IMCR_SHIFT;

  if (((pinset & _PIN_INPUT_MODE_MASK) != PIN_INPUT_MODE_WKPU) && \
      (imcr >= (EIRQ_IMCR_FIRST - 512)) && (imcr <= (EIRQ_IMCR_LAST - 512)))
    {
      /* Calculate the EIRQ index from the IMCR number */

      index = imcr - (EIRQ_IMCR_FIRST - 512);

      /* Attach the interrupt handler */

      flags = enter_critical_section();

      g_eirqisrs[index].handler = pinisr;
      g_eirqisrs[index].arg     = arg;

      leave_critical_section(flags);
      return OK;
    }
#endif /* CONFIG_S32K3XX_EIRQINTS */

#ifdef CONFIG_S32K3XX_WKPUINTS
  if ((pinset & _PIN_INPUT_MODE_MASK) == PIN_INPUT_MODE_WKPU)
    {
      /* Get the WPKU index based on pinset */

      index = (pinset & _WKPU_MASK) >> _WKPU_SHIFT; /* Don't add offset */

      /* Attach the interrupt handler */

      flags = enter_critical_section();

      g_wkpuisrs[index].handler = pinisr;
      g_wkpuisrs[index].arg     = arg;

      leave_critical_section(flags);
      return OK;
    }
#endif /* CONFIG_S32K3XX_WKPUINTS */

  return -EINVAL;
}

/****************************************************************************
 * Name: s32k3xx_pinirqenable
 *
 * Description:
 *   Enable the interrupt for specified pin IRQ
 *
 ****************************************************************************/

void s32k3xx_pinirqenable(uint32_t pinset)
{
  unsigned int index;

#ifdef CONFIG_S32K3XX_EIRQINTS
  unsigned int imcr = (pinset & _IMCR_MASK) >> _IMCR_SHIFT;

  if (((pinset & _PIN_INPUT_MODE_MASK) != PIN_INPUT_MODE_WKPU) && \
      (imcr >= (EIRQ_IMCR_FIRST - 512)) && (imcr <= (EIRQ_IMCR_LAST - 512)))
    {
      index = imcr - (EIRQ_IMCR_FIRST - 512);

      switch (pinset & _PIN_INT_MASK)
        {
          case PIN_INT_RISING:
            {
              modifyreg32(S32K3XX_SIUL2_IREER0, 0, 1 << index); /* Enable rising-edge triggered events */
              modifyreg32(S32K3XX_SIUL2_IFEER0, 1 << index, 0); /* Disable falling-edge triggered events */
            }
            break;

          case PIN_INT_FALLING:
            {
              modifyreg32(S32K3XX_SIUL2_IREER0, 0, 1 << index); /* Enable falling-edge triggered events */
              modifyreg32(S32K3XX_SIUL2_IFEER0, 1 << index, 0); /* Disable rising-edge triggered events */
            }
            break;

          case PIN_INT_BOTH:
            {
              modifyreg32(S32K3XX_SIUL2_IREER0, 0, 1 << index); /* Enable rising-edge triggered events */
              modifyreg32(S32K3XX_SIUL2_IFEER0, 0, 1 << index); /* Enable falling-edge triggered events */
            }
            break;

          default:
            break;
        }

      /* Enable interrupt requests */

      modifyreg32(S32K3XX_SIUL2_DIRER0, 0, 1 << index);
    }
#endif /* CONFIG_S32K3XX_EIRQINTS */

#ifdef CONFIG_S32K3XX_WKPUINTS
  if ((pinset & _PIN_INPUT_MODE_MASK) == PIN_INPUT_MODE_WKPU)
    {
      /* Get the WPKU index based on pinset */

      index = ((pinset & _WKPU_MASK) >> _WKPU_SHIFT) + WKPU_SRC_OFFSET;

      if (index < 32)
        {
          switch (pinset & _PIN_INT_MASK)
            {
              case PIN_INT_RISING:
                {
                  modifyreg32(S32K3XX_WKPU_WIREER, 0, 1 << index); /* Enable rising-edge triggered events */
                  modifyreg32(S32K3XX_WKPU_WIFEER, 1 << index, 0); /* Disable falling-edge triggered events */
                }
                break;

              case PIN_INT_FALLING:
                {
                  modifyreg32(S32K3XX_WKPU_WIFEER, 0, 1 << index); /* Enable falling-edge triggered events */
                  modifyreg32(S32K3XX_WKPU_WIREER, 1 << index, 0); /* Disable rising-edge triggered events */
                }
                break;

              case PIN_INT_BOTH:
                {
                  modifyreg32(S32K3XX_WKPU_WIREER, 0, 1 << index); /* Enable rising-edge triggered events */
                  modifyreg32(S32K3XX_WKPU_WIFEER, 0, 1 << index); /* Enable falling-edge triggered events */
                }
                break;

              default:
                break;
            }

          /* Enable analog glitch filter */

          modifyreg32(S32K3XX_WKPU_WIFER, 0, 1 << index);

          /* Enable interrupt requests */

          modifyreg32(S32K3XX_WKPU_IRER, 0, 1 << index);
        }
      else
        {
          switch (pinset & _PIN_INT_MASK)
            {
              case PIN_INT_RISING:
                {
                  modifyreg32(S32K3XX_WKPU_WIREER_64, 0, 1 << (index - 32)); /* Enable rising-edge triggered events */
                  modifyreg32(S32K3XX_WKPU_WIFEER_64, 1 << (index - 32), 0); /* Disable falling-edge triggered events */
                }
                break;

              case PIN_INT_FALLING:
                {
                  modifyreg32(S32K3XX_WKPU_WIFEER_64, 0, 1 << (index - 32)); /* Enable falling-edge triggered events */
                  modifyreg32(S32K3XX_WKPU_WIREER_64, 1 << (index - 32), 0); /* Disable rising-edge triggered events */
                }
                break;

              case PIN_INT_BOTH:
                {
                  modifyreg32(S32K3XX_WKPU_WIREER_64, 0, 1 << (index - 32)); /* Enable rising-edge triggered events */
                  modifyreg32(S32K3XX_WKPU_WIFEER_64, 0, 1 << (index - 32)); /* Enable falling-edge triggered events */
                }
                break;

            default:
                break;
            }

          /* Enable analog glitch filter */

          modifyreg32(S32K3XX_WKPU_WIFER_64, 0, 1 << (index - 32));

          /* Enable interrupt requests */

          modifyreg32(S32K3XX_WKPU_IRER_64, 0, 1 << (index - 32));
        }
    }
#endif /* CONFIG_S32K3XX_WKPUINTS */
}

/****************************************************************************
 * Name: s32k3xx_pinirqdisable
 *
 * Description:
 *   Disable the interrupt for specified pin
 *
 ****************************************************************************/

void s32k3xx_pinirqdisable(uint32_t pinset)
{
  unsigned int index;

#ifdef CONFIG_S32K3XX_EIRQINTS
  unsigned int imcr = (pinset & _IMCR_MASK) >> _IMCR_SHIFT;

  if (((pinset & _PIN_INPUT_MODE_MASK) != PIN_INPUT_MODE_WKPU) && \
      (imcr >= (EIRQ_IMCR_FIRST - 512)) && (imcr <= (EIRQ_IMCR_LAST - 512)))
    {
      index = imcr - (EIRQ_IMCR_FIRST - 512);

      modifyreg32(S32K3XX_SIUL2_DIRER0, 1 << index, 0); /* Disable interrupt requests */
      modifyreg32(S32K3XX_SIUL2_IREER0, 1 << index, 0); /* Disable rising-edge triggered events */
      modifyreg32(S32K3XX_SIUL2_IFEER0, 1 << index, 0); /* Disable falling-edge triggered events */
    }
#endif /* CONFIG_S32K3XX_EIRQINTS */

#ifdef CONFIG_S32K3XX_WKPUINTS
  if ((pinset & _PIN_INPUT_MODE_MASK) == PIN_INPUT_MODE_WKPU)
    {
      /* Get the WPKU index based on pinset */

      index = ((pinset & _WKPU_MASK) >> _WKPU_SHIFT) + WKPU_SRC_OFFSET;

      if (index < 32)
        {
          modifyreg32(S32K3XX_WKPU_IRER,   1 << index, 0); /* Disable interrupt requests */
          modifyreg32(S32K3XX_WKPU_WIREER, 1 << index, 0); /* Disable rising-edge triggered events */
          modifyreg32(S32K3XX_WKPU_WIFEER, 1 << index, 0); /* Disable falling-edge triggered events */
          modifyreg32(S32K3XX_WKPU_WIFER,  1 << index, 0); /* Disable analog glitch filter */
        }
      else
        {
          modifyreg32(S32K3XX_WKPU_IRER_64,   1 << (index - 32), 0); /* Disable interrupt requests */
          modifyreg32(S32K3XX_WKPU_WIREER_64, 1 << (index - 32), 0); /* Disable rising-edge triggered events */
          modifyreg32(S32K3XX_WKPU_WIFEER_64, 1 << (index - 32), 0); /* Disable falling-edge triggered events */
          modifyreg32(S32K3XX_WKPU_WIFER_64,  1 << (index - 32), 0); /* Disable analog glitch filter */
        }
    }
#endif /* CONFIG_S32K3XX_WKPUINTS */
}
#endif /* CONFIG_S32K3XX_GPIOIRQ */
