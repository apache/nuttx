/****************************************************************************
 * arch/avr/src/at32uc3/at32uc3_irq.c
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
#include "at32uc3_config.h"

#include <stdint.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/irq.h>

#include "up_internal.h"
#include "at32uc3.h"

#include "chip.h"
#include "at32uc3_intc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* These symbols are exported from up_exceptions.S:
 */

extern uint32_t vectortab;
extern uint32_t avr32_int0;
extern uint32_t avr32_int1;
extern uint32_t avr32_int2;
extern uint32_t avr32_int3;

/* The provide interrupt handling offsets relative to the EVBA
 * address (which should be vectortab).
 */

#define AVR32_INT0_RADDR ((uint32_t)&avr32_int0 - (uint32_t)&vectortab)
#define AVR32_INT1_RADDR ((uint32_t)&avr32_int1 - (uint32_t)&vectortab)
#define AVR32_INT2_RADDR ((uint32_t)&avr32_int2 - (uint32_t)&vectortab)
#define AVR32_INT3_RADDR ((uint32_t)&avr32_int3 - (uint32_t)&vectortab)

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile uint32_t *g_current_regs;

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct irq_groups_s
{
  uint8_t baseirq;  /* IRQ number associated with bit 0 */
  uint8_t nirqs;    /* Number of IRQs in this group */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This table maps groups into (1) the base IRQ number for the group and (2)
 * the number of valid interrupts in the group.
 */

static const struct irq_groups_s g_grpirqs[AVR32_IRQ_NGROUPS] =
{
  {AVR32_IRQ_BASEIRQGRP0,  AVR32_IRQ_NREQGRP0 }, /* Group  0 */
  {AVR32_IRQ_BASEIRQGRP1,  AVR32_IRQ_NREQGRP1 }, /* Group  1 */
  {AVR32_IRQ_BASEIRQGRP2,  AVR32_IRQ_NREQGRP2 }, /* Group  2 */
  {AVR32_IRQ_BASEIRQGRP3,  AVR32_IRQ_NREQGRP3 }, /* Group  3 */
  {AVR32_IRQ_BASEIRQGRP4,  AVR32_IRQ_NREQGRP4 }, /* Group  4 */
  {AVR32_IRQ_BASEIRQGRP5,  AVR32_IRQ_NREQGRP5 }, /* Group  5 */
  {AVR32_IRQ_BASEIRQGRP6,  AVR32_IRQ_NREQGRP6 }, /* Group  6 */
  {AVR32_IRQ_BASEIRQGRP7,  AVR32_IRQ_NREQGRP7 }, /* Group  7 */
  {AVR32_IRQ_BASEIRQGRP8,  AVR32_IRQ_NREQGRP8 }, /* Group  8 */
  {AVR32_IRQ_BASEIRQGRP9,  AVR32_IRQ_NREQGRP9 }, /* Group  9 */
  {AVR32_IRQ_BASEIRQGRP10, AVR32_IRQ_NREQGRP10}, /* Group 10 */
  {AVR32_IRQ_BASEIRQGRP11, AVR32_IRQ_NREQGRP11}, /* Group 11 */
  {AVR32_IRQ_BASEIRQGRP12, AVR32_IRQ_NREQGRP12}, /* Group 12 */
  {AVR32_IRQ_BASEIRQGRP13, AVR32_IRQ_NREQGRP13}, /* Group 13 */
  {AVR32_IRQ_BASEIRQGRP14, AVR32_IRQ_NREQGRP14}, /* Group 14 */
  {AVR32_IRQ_BASEIRQGRP15, AVR32_IRQ_NREQGRP15}, /* Group 15 */
  {AVR32_IRQ_BASEIRQGRP16, AVR32_IRQ_NREQGRP16}, /* Group 16 */
  {AVR32_IRQ_BASEIRQGRP17, AVR32_IRQ_NREQGRP17}, /* Group 17 */
  {AVR32_IRQ_BASEIRQGRP18, AVR32_IRQ_NREQGRP18}, /* Group 18 */
};

/* The following table provides the value of the IPR register to
 * use to assign a group to different interrupt priorities.
 */

#if 0 /* REVISIT -- Can we come up with a way to statically initialize? */
static const uint32_t g_ipr[AVR32_IRQ_INTPRIOS] =
{
  ((AVR32_INT0_RADDR << INTC_IPR_AUTOVECTOR_SHIFT) | INTC_IPR_INTLEVEL_INT0),
  ((AVR32_INT1_RADDR << INTC_IPR_AUTOVECTOR_SHIFT) | INTC_IPR_INTLEVEL_INT1),
  ((AVR32_INT2_RADDR << INTC_IPR_AUTOVECTOR_SHIFT) | INTC_IPR_INTLEVEL_INT2),
  ((AVR32_INT3_RADDR << INTC_IPR_AUTOVECTOR_SHIFT) | INTC_IPR_INTLEVEL_INT3),
};
#else
static uint32_t g_ipr[AVR32_IRQ_INTPRIOS];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_getgrp
 ****************************************************************************/

static int up_getgrp(unsigned int irq)
{
  int i;

  if (irq >= AVR32_IRQ_BASEIRQGRP0)
    {
      for (i = 0; i < AVR32_IRQ_NGROUPS; i++)
        {
          if (irq < g_grpirqs[i].baseirq + g_grpirqs[i].nirqs)
            {
              return i;
            }
        }
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: avr32_xcptn
 *
 * Description:
 *   Handlers for unexpected exceptions.  All are fatal error conditions.
 *
 ****************************************************************************/

static int avr32_xcptn(int irq, FAR void *context, FAR void *arg)
{
  up_irq_save();
  _alert("PANIC!!! Exception IRQ: %d\n", irq);
  PANIC();
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  int group;
  int irq;

  /* Initialize the table that provides the value of the IPR register to
   * use to assign a group to different interrupt priorities.
   */

#if 1 /* REVISIT -- Can we come up with a way to statically initialize? */
  g_ipr[0] = ((AVR32_INT0_RADDR << INTC_IPR_AUTOVECTOR_SHIFT) |
               INTC_IPR_INTLEVEL_INT0);
  g_ipr[1] = ((AVR32_INT1_RADDR << INTC_IPR_AUTOVECTOR_SHIFT) |
               INTC_IPR_INTLEVEL_INT1);
  g_ipr[2] = ((AVR32_INT2_RADDR << INTC_IPR_AUTOVECTOR_SHIFT) |
               INTC_IPR_INTLEVEL_INT2);
  g_ipr[3] = ((AVR32_INT3_RADDR << INTC_IPR_AUTOVECTOR_SHIFT) |
               INTC_IPR_INTLEVEL_INT3);
#endif

  /* Set the interrupt group priority to a default value.  All are linked to
   * interrupt priority level 0 and to interrupt vector INT0.
   */

  for (group = 0; group < AVR32_IRQ_MAXGROUPS; group++)
    {
      putreg32(g_ipr[0], AVR32_INTC_IPR(group));
    }

  /* currents_regs is non-NULL only while processing an interrupt */

  g_current_regs = NULL;

  /* Attach the exception handlers */

  for (irq = 0; irq < AVR32_IRQ_NEVENTS; irq++)
    {
      irq_attach(irq, avr32_xcptn, NULL);
    }

  /* Initialize GPIO interrupt facilities */

#ifdef CONFIG_AVR32_GPIOIRQ
#ifdef CONFIG_HAVE_WEAKFUNCTIONS
  if (gpio_irqinitialize != NULL)
#endif
    {
      gpio_irqinitialize();
    }
#endif

  /* And finally, enable interrupts */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  up_irq_restore(0);
#endif
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
  if (priority >= 0 && priority < AVR32_IRQ_INTPRIOS)
    {
      int group = up_getgrp(irq);
      if (group >= 0)
        {
          putreg32(g_ipr[priority], AVR32_INTC_IPR(group));
          return OK;
        }
    }

  return -EINVAL;
}
#endif

/****************************************************************************
 * Name: avr32_intirqno
 *
 * Description:
 *   Return the highest priority pending INTn interrupt (hwere n=level).
 *   This is called directly from interrupt handling logic.  This should be
 *   save since the UC3B will save all C scratch/volatile registers (and
 *   this function should not alter the perserved/static registers).
 *
 ****************************************************************************/

unsigned int avr32_intirqno(unsigned int level)
{
  /* Get the group that caused the interrupt: "ICRn identifies the group with
   * the highest priority that has a pending interrupt of level n. This value
   * is only defined when at least one interrupt of level n is pending.
   */

  uint32_t group = getreg32(AVR32_INTC_ICR(level)) & INTC_ICR_CAUSE_MASK;
  if (group < AVR32_IRQ_NGROUPS)
    {
      /* Now get the set of pending interrupt requests for this group.
       * Note that we may get spurious interrupts due to races conditions
       */

      uint32_t irr = getreg32(AVR32_INTC_IRR(group));
      unsigned irq  = g_grpirqs[group].baseirq;
      uint32_t mask = 1;
      int i;

      /* Check each interrupt source for this group */

      for (i = 0; i < g_grpirqs[group].nirqs; i++)
        {
          /* Is there an interrupt pending? */

          if ((irr & mask) != 0)
            {
              /* Yes.. return its IRQ number */

              return irq;
            }

          /* No.. this is interrupt is not pending */

          irq++;
          mask <<= 1;
        }

      _err("ERROR: Spurious interrupt: group=%d IRR=%08x\n", group, irr);
      return -ENODEV;
    }

  _err("ERROR: Bad group: %d\n", group);
  return AVR32_IRQ_BADVECTOR;
}
