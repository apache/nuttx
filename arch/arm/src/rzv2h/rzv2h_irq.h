/****************************************************************************
 * arch/arm/src/rzv2h/rzv2h_irq.h
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

#ifndef __ARCH_ARM_SRC_RZV2H_RZV2H_IRQ_H
#define __ARCH_ARM_SRC_RZV2H_RZV2H_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "bsp_api.h"
#include "rzv2h_irq_id.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* FSP numbers peripheral interrupts relative to the first SPI.  NuttX uses
 * the absolute GIC interrupt ID, including the 16 SGIs and 16 PPIs.
 */

#define RZV2H_FSP_TO_GIC_IRQ(irq) \
  ((int)(irq) + BSP_CORTEX_VECTOR_TABLE_ENTRIES)

/* SEL pool bounds: RZV2H_IRQ_SEL0 to RZV2H_IRQ_SEL126, 127 lines total.
 * 'rzv2h_irqsel_t' (peripheral event selectors) and 'rzv2h_irqn_t'
 * (GIC/FSP interrupt numbers, including
 * RZV2H_IRQ_SEL0..RZV2H_IRQ_SEL126) are declared in rzv2h_irq_id.h.
 */

#define RZV2H_INTSEL_FIRST           RZV2H_IRQ_SEL0    /* 353 */
#define RZV2H_INTSEL_LAST            RZV2H_IRQ_SEL126  /* 479 */
#define RZV2H_INTSEL_COUNT           (RZV2H_INTSEL_LAST - RZV2H_INTSEL_FIRST + 1)

#define RZV2H_BSP_PRV_INTERRUPTABLE_NUM      (32U)

/****************************************************************************
 * Public Function Prototypes
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

void rzv2h_intsel_initialize(void);

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
                          uint8_t irq_detect_type);

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

int rzv2h_intsel_disconnect_event(rzv2h_irqn_t irq);

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
                                     void (*fsp_isr)(void));

#endif /* __ARCH_ARM_SRC_RZV2H_RZV2H_IRQ_H */
