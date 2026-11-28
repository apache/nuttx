/****************************************************************************
 * arch/risc-v/src/spacemit-k1x-rcpu/spacemit_k1x_rcpu_irq.c
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

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <arch/irq.h>
#include <arch/barriers.h>

#include "riscv_internal.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Value written to clicintctl for every source.
 *
 * clicintctl packs the interrupt level in the high bits and the priority in
 * the low bits.  The split point is not architectural: it is set by
 * CFG.NLBITS and can differ between Nuclei implementations.  Writing all
 * ones selects the maximum of both fields whatever the split is, which is
 * what we want for every source this port uses.
 */

#define ECLIC_INTCTL_MAX            0xff

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Base addresses discovered from the core at start up.  Zero means "not
 * discovered yet".
 */

static uintptr_t g_eclic_base;
static uintptr_t g_systimer_base;

/* Number of interrupt sources reported by the ECLIC, cached for the
 * accessors below.
 */

static unsigned int g_eclic_nirqs;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spacemit_k1x_rcpu_discover_iregion
 *
 * Description:
 *   Locate the Nuclei IRegion and derive the ECLIC and timer addresses.
 *
 *   The Nuclei core does not fix these at a compile time address.  It
 *   reports its capabilities in CSR_MCFG_INFO and, when IREGION_EXIST is
 *   set, the base of the IRegion in CSR_MIRGB_INFO.  The ECLIC and the
 *   machine timer then live at fixed offsets inside that region.
 *
 *   K1 does report an IRegion.  The fallback below exists for cores that do
 *   not, and is the value the Nuclei documentation gives for that case; it
 *   is not an address that was observed on K1.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spacemit_k1x_rcpu_discover_iregion(void)
{
  uint32_t mcfg;
  uintptr_t iregion;

  mcfg = READ_CSR(CSR_MCFG_INFO);

  if ((mcfg & SPACEMIT_K1X_RCPU_MCFG_INFO_IREGION_EXIST) != 0)
    {
      /* CSR_MIRGB_INFO reports the base aligned down to 1 KB */

      iregion = (uintptr_t)(READ_CSR(CSR_MIRGB_INFO) &
                            SPACEMIT_K1X_RCPU_IREGION_ALIGN_MASK);

      g_eclic_base    = iregion + SPACEMIT_K1X_RCPU_IREGION_ECLIC_OFFSET;
      g_systimer_base = iregion + SPACEMIT_K1X_RCPU_IREGION_TIMER_OFFSET;
    }
  else
    {
      g_eclic_base    = SPACEMIT_K1X_RCPU_ECLIC_BASE_FALLBACK;
      g_systimer_base = SPACEMIT_K1X_RCPU_SYSTIMER_BASE_FALLBACK;
    }

  /* Cache the source count while the information is at hand.  NUMINT is the
   * id of the highest source, so the count is one more than that.
   */

  g_eclic_nirqs = (getreg32(SPACEMIT_K1X_RCPU_ECLIC_INFO(g_eclic_base)) &
                   SPACEMIT_K1X_RCPU_ECLIC_INFO_NUMINT_MASK) + 1;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spacemit_k1x_rcpu_eclic_base
 *
 * Description:
 *   Return the ECLIC base address, discovering it on first use.
 *
 ****************************************************************************/

static void spacemit_k1x_rcpu_iregion(void)
{
  if (g_eclic_base == 0)
    {
      spacemit_k1x_rcpu_discover_iregion();
    }
}

static uintptr_t spacemit_k1x_rcpu_eclic_base(void)
{
  spacemit_k1x_rcpu_iregion();
  return g_eclic_base;
}

/****************************************************************************
 * Name: spacemit_k1x_rcpu_systimer_base
 *
 * Description:
 *   Return the base address of the Nuclei machine timer, discovering it on
 *   first use.
 *
 ****************************************************************************/

uintptr_t spacemit_k1x_rcpu_systimer_base(void)
{
  spacemit_k1x_rcpu_iregion();
  return g_systimer_base;
}

/****************************************************************************
 * Name: spacemit_k1x_rcpu_eclic_num_irqs
 *
 * Description:
 *   Return the number of interrupt sources reported by the ECLIC.
 *
 ****************************************************************************/

static unsigned int spacemit_k1x_rcpu_eclic_num_irqs(void)
{
  spacemit_k1x_rcpu_iregion();
  return g_eclic_nirqs;
}

/****************************************************************************
 * Name: up_irqinitialize
 *
 * Description:
 *   Put the ECLIC into a known state and attach the common exception
 *   handlers.  Called once during start up with interrupts disabled.
 *
 ****************************************************************************/

void up_irqinitialize(void)
{
  uintptr_t base;
  uint32_t info;
  uint8_t cfg;
  unsigned int nlbits;
  unsigned int nirqs;
  unsigned int i;

  up_irq_save();

  base  = spacemit_k1x_rcpu_eclic_base();
  info  = getreg32(SPACEMIT_K1X_RCPU_ECLIC_INFO(base));
  nirqs = spacemit_k1x_rcpu_eclic_num_irqs();

  /* The width of the level field is a property of the part, not of this
   * driver, and the vendor firmware does exactly the same thing: it reads it
   * from ECLIC->INFO at run time rather than carrying a constant.
   */

  nlbits = (info & SPACEMIT_K1X_RCPU_ECLIC_INFO_CTLBITS_MASK) >>
           SPACEMIT_K1X_RCPU_ECLIC_INFO_CTLBITS_SHIFT;

  /* Read-modify-write, because CFG has other bits that must be preserved */

  cfg = getreg8(SPACEMIT_K1X_RCPU_ECLIC_CFG(base));
  cfg = (cfg & ~SPACEMIT_K1X_RCPU_ECLIC_CFG_NLBITS_MASK) |
        (uint8_t)(nlbits << SPACEMIT_K1X_RCPU_ECLIC_CFG_NLBITS_SHIFT);
  putreg8(cfg, SPACEMIT_K1X_RCPU_ECLIC_CFG(base));

  /* Threshold zero: every interrupt whose level is non-zero is taken */

  putreg8(0, SPACEMIT_K1X_RCPU_ECLIC_MTH(base));

  /* Every source: not enabled, not pending, level triggered, highest level
   * and priority.  Level triggering is the default the ECLIC resets with.
   */

  for (i = 0; i < nirqs; i++)
    {
      putreg8(0, SPACEMIT_K1X_RCPU_ECLIC_INTIE(base, i));
      putreg8(0, SPACEMIT_K1X_RCPU_ECLIC_INTIP(base, i));
      putreg8(SPACEMIT_K1X_RCPU_ECLIC_INTATTR_TRIG_LEVEL,
              SPACEMIT_K1X_RCPU_ECLIC_INTATTR(base, i));
      putreg8(ECLIC_INTCTL_MAX,
              SPACEMIT_K1X_RCPU_ECLIC_INTCTL(base, i));
    }

  /* Report a part whose source count does not match what this port was
   * built for.  Not fatal: the loop above sized itself from the hardware, so
   * only the NR_IRQS derived tables are short.  Louder than a silent
   * mismatch.
   */

  if (nirqs != CONFIG_SPACEMIT_K1X_RCPU_ECLIC_NUM_IRQS)
    {
      _err("ERROR: ECLIC reports %u sources, configured %u\n",
           nirqs, CONFIG_SPACEMIT_K1X_RCPU_ECLIC_NUM_IRQS);
    }

#if defined(CONFIG_STACK_COLORATION) && CONFIG_ARCH_INTERRUPTSTACK > 15
  /* Colorize the interrupt stack for debug purposes */

  riscv_stack_color(g_intstackalloc, CONFIG_ARCH_INTERRUPTSTACK & ~15);
#endif

  /* Attach the common interrupt handler */

  riscv_exception_attach();

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  /* And finally, enable interrupts */

  up_irq_enable();
#endif
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the interrupt associated with the IRQ number.  In CLIC mode the
 *   per-source gate is the ECLIC INTIE bit; the mie CSR does not carry
 *   external sources at all.
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  uintptr_t base = spacemit_k1x_rcpu_eclic_base();
  int extirq = irq - RISCV_IRQ_ASYNC;

  DEBUGASSERT(extirq >= 0 &&
              extirq < (int)spacemit_k1x_rcpu_eclic_num_irqs());

  putreg8(0, SPACEMIT_K1X_RCPU_ECLIC_INTIE(base, extirq));
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the interrupt associated with the IRQ number.
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  uintptr_t base = spacemit_k1x_rcpu_eclic_base();
  int extirq = irq - RISCV_IRQ_ASYNC;

  DEBUGASSERT(extirq >= 0 &&
              extirq < (int)spacemit_k1x_rcpu_eclic_num_irqs());

  putreg8(SPACEMIT_K1X_RCPU_ECLIC_INTIE_IE,
          SPACEMIT_K1X_RCPU_ECLIC_INTIE(base, extirq));
}

/****************************************************************************
 * Name: riscv_ack_irq
 *
 * Description:
 *   Acknowledge an interrupt.  In CLIC mode the identity arrives in
 *   mcause.EXCCODE and there is no claim/complete register, so nothing has
 *   to be read back.  A level triggered source stays pending until whatever
 *   raised it is serviced by its own driver, but an edge triggered source
 *   has to have its pending bit cleared here.
 *
 ****************************************************************************/

void riscv_ack_irq(int irq)
{
  uintptr_t base = spacemit_k1x_rcpu_eclic_base();
  int extirq = irq - RISCV_IRQ_ASYNC;
  uint8_t attr;

  if (extirq < 0 || extirq >= (int)spacemit_k1x_rcpu_eclic_num_irqs())
    {
      return;
    }

  attr = getreg8(SPACEMIT_K1X_RCPU_ECLIC_INTATTR(base, extirq));

  if ((attr & SPACEMIT_K1X_RCPU_ECLIC_INTATTR_TRIG_MASK) !=
      SPACEMIT_K1X_RCPU_ECLIC_INTATTR_TRIG_LEVEL)
    {
      putreg8(0, SPACEMIT_K1X_RCPU_ECLIC_INTIP(base, extirq));
    }
}

/****************************************************************************
 * Name: up_irq_enable
 *
 * Description:
 *   Enable interrupts globally and return the previous machine status.
 *
 ****************************************************************************/

irqstate_t up_irq_enable(void)
{
  return READ_AND_SET_CSR(CSR_MSTATUS, MSTATUS_MIE);
}
