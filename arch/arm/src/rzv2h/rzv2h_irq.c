/****************************************************************************
 * arch/arm/src/rzv2h/rzv2h_irq.c
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
#include <nuttx/irq.h>
#include <arch/barriers.h>
#include <sched.h>
#include <stdint.h>
#include <errno.h>
#include <syslog.h>
#include "arm_internal.h"
#include "sctlr.h"
#include "gic.h"
#include "rzv2h_irq.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RZV2H INTC INTSEL register layout
 *
 * Base address: 0x10400140 (CR8 INTC INTSEL, CR8 INTC base + 0x140).
 *
 * Each INTSEL register covers 3 SPI interrupt lines.
 * The register index for a given IRQ number is:
 *   reg_index = (irq - 353) / 3
 *   register address = 0x10400140 + reg_index * 4
 *
 * Within each register, the 3 fields are packed as:
 *   Bits  9: 0  - SPIK_SEL for IRQ (353 + 3*reg_index + 0)
 *   Bits 19:10  - SPIK_SEL for IRQ (353 + 3*reg_index + 1)
 *   Bits 29:20  - SPIK_SEL for IRQ (353 + 3*reg_index + 2)
 */

#define RZ_INTC_INTSEL_BASE            (0x10400140) /* CR8 INTC INTSEL: CR8 INTC base + 0x140 */
#define OFFSET(y)                      ((y) - RZV2H_INTSEL_FIRST)
#define REG_INTSEL_ADDR(y)             (RZ_INTC_INTSEL_BASE + ((OFFSET(y) / 3) * 4))
#define REG_INTSEL_READ(y)             getreg32(REG_INTSEL_ADDR(y))
#define REG_INTSEL_WRITE(y, v)         putreg32((v), REG_INTSEL_ADDR(y))
#define REG_INTSEL_SPIK_SEL_SHIFT(y)   ((OFFSET(y) % 3) * 10)
#define REG_INTSEL_SPIK_SEL_MASK(y)    (0x3ffu << REG_INTSEL_SPIK_SEL_SHIFT(y))

/* RZV2H_INTSEL_FIRST/LAST/COUNT (SEL pool bounds) are defined in
 * rzv2h_irq.h, shared with the default-SEL macros used by peripheral
 * drivers.
 */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Event currently connected to each SEL line (index = SEL number).
 * RZV2H_IRQSEL_NONE marks a free line (never used or freed via
 * rzv2h_intsel_disconnect_event()).  Initialized by
 * rzv2h_intsel_initialize() since RZV2H_IRQSEL_NONE is not zero and
 * cannot rely on default BSS init.
 */

static rzv2h_irqsel_t g_intsel_event[RZV2H_INTSEL_COUNT];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rzv2h_intsel_initialize
 *
 * Description:
 *   Initialize the SEL line event-tracking table: mark all SEL0-SEL126
 *   lines as unconnected.  Must be called exactly once, before any call
 *   to rzv2h_intsel_connect_event() or rzv2h_intsel_disconnect_event().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void rzv2h_intsel_initialize(void)
{
  int i;

  for (i = 0; i < RZV2H_INTSEL_COUNT; i++)
    {
      g_intsel_event[i] = RZV2H_IRQSEL_NONE;
    }
}

/****************************************************************************
 * Name: rzv2h_intsel_connect_event
 *
 * Description:
 *   Connect a peripheral event to a specific, caller-chosen SEL line
 *   (SEL0-SEL126) via the INTSEL registers.  The CR8 core's INTC uses
 *   INTSEL registers (at base 0x10400140, specific to the CR8 INTC
 *   instance) to select which peripheral event maps to each SPI
 *   interrupt line.  Each INTSEL register packs 3 event selectors at
 *   10 bits each.
 *
 * Input Parameters:
 *   irq      - IRQ number to program (rzv2h_irqn_t, e.g.
 *              RZV2H_IRQ_SEL0 = 353).  Must be in [RZV2H_INTSEL_FIRST,
 *              RZV2H_INTSEL_LAST].
 *   event    - Event selector value (rzv2h_irqsel_t, the peripheral
 *              event to map to 'irq').  These values are fixed per
 *              peripheral/channel.
 *   irq_detect_type - GIC detect type for 'irq'
 *              (BSP_GIC_SPI_DETECT_LEVEL or BSP_GIC_SPI_DETECT_EDGE,
 *              see bsp_irq_gic.h).  Recorded into FSP's
 *              g_gic_detect_type[] table so that R_BSP_IrqCfg()
 *              (called from *_Open()) applies the correct GIC detect
 *              type for this SEL line automatically, on this and every
 *              future Open(), without the caller having to call
 *              up_set_irq_type() itself.
 *
 * Returned Value:
 *   OK (0) on success.  Returns -EINVAL if 'irq' is outside the SEL0-
 *   SEL126 range, or -EBUSY if 'irq' is already connected to a
 *   different event.
 *
 ****************************************************************************/

int rzv2h_intsel_connect_event(rzv2h_irqn_t irq, rzv2h_irqsel_t event,
                          uint8_t irq_detect_type)
{
  irqstate_t flags;
  uint32_t regval;
  int sel;

  if ((int)irq < RZV2H_INTSEL_FIRST || (int)irq > RZV2H_INTSEL_LAST)
    {
      return -EINVAL;
    }

  sel = (int)irq - RZV2H_INTSEL_FIRST;

  flags = enter_critical_section();

  if (g_intsel_event[sel] != RZV2H_IRQSEL_NONE &&
      g_intsel_event[sel] != event)
    {
      leave_critical_section(flags);
      syslog(LOG_ERR,
             "INTC: SEL%d (IRQ %d) already connected to event %d, "
             "cannot connect event %d\n",
             sel, (int)irq, (int)g_intsel_event[sel], (int)event);
      return -EBUSY;
    }

  g_intsel_event[sel] = event;

  /* Program INTSEL register: connect this IRQ to the given event */

  regval = REG_INTSEL_READ(irq);
  regval &= ~REG_INTSEL_SPIK_SEL_MASK(irq);
  regval |= ((uint32_t)event << REG_INTSEL_SPIK_SEL_SHIFT(irq))
             & REG_INTSEL_SPIK_SEL_MASK(irq);
  REG_INTSEL_WRITE(irq, regval);

  /* Recorded into FSP's g_gic_detect_type[] table to
   * apply the correct trigger type automatically.
   */

  g_gic_detect_type[RZV2H_FSP_TO_GIC_IRQ(irq)] = irq_detect_type;

  leave_critical_section(flags);

  syslog(LOG_INFO, "INTC: connected SEL%d (IRQ %d) to event %d\n",
         sel, (int)irq, (int)event);

  return OK;
}

/****************************************************************************
 * Name: rzv2h_intsel_disconnect_event
 *
 * Description:
 *   Release a SEL line previously connected by
 *   rzv2h_intsel_connect_event().  Clears the INTSEL register field
 *   for the line and marks it unconnected.
 *   Callers should invoke this from their shutdown/deinit path
 *   to avoid leaking stale event-tracking entries across repeated
 *   connect/disconnect cycles.
 *
 * Input Parameters:
 *   irq - IRQ number previously passed to
 *         rzv2h_intsel_connect_event() (e.g. RZV2H_IRQ_SEL0 = 353).
 *
 * Returned Value:
 *   OK (0) on success.  Returns -EINVAL if 'irq' is outside the SEL0-
 *   SEL126 range or is not currently connected.
 *
 ****************************************************************************/

int rzv2h_intsel_disconnect_event(rzv2h_irqn_t irq)
{
  irqstate_t flags;
  uint32_t regval;
  int sel;

  if ((int)irq < RZV2H_INTSEL_FIRST || (int)irq > RZV2H_INTSEL_LAST)
    {
      return -EINVAL;
    }

  sel = (int)irq - RZV2H_INTSEL_FIRST;

  flags = enter_critical_section();

  if (g_intsel_event[sel] == RZV2H_IRQSEL_NONE)
    {
      leave_critical_section(flags);
      return -EINVAL;
    }

  regval = REG_INTSEL_READ(irq);
  regval &= ~REG_INTSEL_SPIK_SEL_MASK(irq);
  REG_INTSEL_WRITE(irq, regval);

  g_intsel_event[sel] = RZV2H_IRQSEL_NONE;

  leave_critical_section(flags);

  syslog(LOG_INFO, "INTC: freed SEL%d (IRQ %d)\n", sel, (int)irq);

  return 0;
}

/****************************************************************************
 * Name: rzv2h_interrupt_common_handler
 *
 * Description:
 *   Common wrapper for nested FSP interrupt handlers.  Records entry
 *   into the FSP interrupt context stack, invokes 'fsp_isr', then
 *   records exit.
 *
 * Input Parameters:
 *   fsp_irq - FSP IRQ number of the interrupt being serviced, pushed
 *             onto the nested-interrupt context stack so FSP can look
 *             up the currently active IRQ.
 *   fsp_isr - FSP ISR function to invoke while 'fsp_irq' is recorded
 *             as active.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void rzv2h_interrupt_common_handler(rzv2h_irqn_t fsp_irq,
                                     void (*fsp_isr)(void))
{
  DEBUGASSERT(g_current_interrupt_pointer < RZV2H_BSP_PRV_INTERRUPTABLE_NUM);

  g_current_interrupt_num[g_current_interrupt_pointer++] =
    (uint16_t)fsp_irq;

  UP_DMB();

  fsp_isr();

  g_current_interrupt_pointer--;
}

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  /* Initialize the Generic Interrupt Controller (GIC) for CPU0.
   * In AMP mode, we want arm_gic0_initialize to be called only once.
   */

  if (sched_getcpu() == 0)
    {
      arm_gic0_initialize();  /* Initialization unique to CPU0 */
    }

  arm_gic_initialize();   /* Initialization common to all CPUs */

  /* Reset the SEL-line event-tracking table so every SEL0-SEL126 line
   * starts out marked unconnected, ready for peripheral drivers to
   * connect their events via rzv2h_intsel_connect_event().
   */

  rzv2h_intsel_initialize();

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  /* And finally, enable interrupts */

  arm_color_intstack();
  up_irq_enable();
#endif
}
