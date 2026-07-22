/****************************************************************************
 * arch/arm/src/rm57/rm57_irq.c
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

/* Adapted from tms570_irq.c, but using the RM57L843 VIM register layout
 * (see hardware/rm57_vim.h), which differs from TMS570's VIM/VIM-Parity
 * split (RM57's VIM has 4 groups of 32 channels plus on-chip ECC
 * registers, not present on TMS570LS04x).
 *
 * ESM (Error Signaling Module) interrupt attach/enable is deferred along
 * with the rest of ESM support.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/debug.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/irq.h>

#include "arm_internal.h"
#include "hardware/rm57l843_memorymap.h"
#include "hardware/rm57_vim.h"
#include "rm57_irq.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_error_handler
 ****************************************************************************/

static void rm57_error_handler(void)
{
  PANIC();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 *
 * Description:
 *   Only the indexed interrupt mode is supported here: after the CPU
 *   takes an IRQ/FIQ exception, it branches to the vector at 0x18/0x1c.
 *   The main handler reads the IRQINDEX/FIQINDEX register to determine
 *   which channel fired. See tms570_irq.c for a more detailed
 *   description of the VIM's three interrupt handling modes; only mode
 *   1 (indexed) is used here, matching the sibling TMS570 port.
 *
 ****************************************************************************/

void up_irqinitialize(void)
{
  uintptr_t *vimram;
  int i;

  /* Initialize VIM RAM vectors with a default error handler. These
   * vectors are not used by the indexed-mode interrupt handling below,
   * but are populated as a safety net for the fall-back vector logic.
   */

  vimram = (uintptr_t *)RM57_VIMRAM_BASE;
  for (i = 0; i < (RM57_IRQ_NCHANNELS + 1); i++)
    {
      *vimram++ = (uintptr_t)rm57_error_handler;
    }

  putreg32((uint32_t)rm57_error_handler, RM57_VIM_FBVECADDR);

  /* Assign all channels to IRQ (not FIQ) */

  putreg32(0, RM57_VIM_FIRQPR0);
  putreg32(0, RM57_VIM_FIRQPR1);
  putreg32(0, RM57_VIM_FIRQPR2);
  putreg32(0, RM57_VIM_FIRQPR3);

  /* Disable all interrupts (channels 0/1 - ESM high / phantom - are left
   * alone, matching TMS570's convention of never disabling channel 0/1
   * through the mask registers)
   */

  putreg32(0xfffffffc, RM57_VIM_REQMASKCLR0);
  putreg32(0xffffffff, RM57_VIM_REQMASKCLR1);
  putreg32(0xffffffff, RM57_VIM_REQMASKCLR2);
  putreg32(0xffffffff, RM57_VIM_REQMASKCLR3);

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  /* And finally, enable interrupts globally */

  arm_color_intstack();
  up_irq_enable();
#endif
}

/****************************************************************************
 * Name: arm_decodeirq
 *
 * Description:
 *   This function is called from the IRQ vector handler in arm_vectors.S.
 *   At this point, the interrupt has been taken and the registers have
 *   been saved on the stack.  This function simply needs to determine
 *   the irq number of the interrupt and then to call arm_doirq to
 *   dispatch the interrupt.
 *
 ****************************************************************************/

uint32_t *arm_decodeirq(uint32_t *regs)
{
  int vector;

  /* Get the interrupting vector number from the IRQINDEX register. Zero,
   * the "phantom" vector, is returned when no channel is pending.
   */

  vector = getreg32(RM57_VIM_IRQINDEX) & VIM_IRQINDEX_MASK;
  if (vector > 0)
    {
      /* Dispatch the interrupt. NOTE that the IRQ number is the vector
       * number offset by one to skip over the "phantom" vector.
       */

      regs = arm_doirq(vector - 1, regs);
    }

  return regs;
}

/****************************************************************************
 * Name: arm_decodefiq
 *
 * Description:
 *   This function is called from the FIQ vector handler in arm_vectors.S.
 *
 ****************************************************************************/

#ifdef CONFIG_ARMV7R_HAVE_DECODEFIQ
uint32_t *arm_decodefiq(uint32_t *regs)
{
  int vector;

  vector = getreg32(RM57_VIM_FIQINDEX) & VIM_FIQINDEX_MASK;
  if (vector > 0)
    {
      regs = arm_doirq(vector - 1, regs);
    }

  return regs;
}
#endif

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the IRQ or FIQ specified by 'channel'
 *
 ****************************************************************************/

void up_disable_irq(int channel)
{
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t bitmask;
  unsigned int regndx;

  DEBUGASSERT(channel >= 0 && channel < RM57_IRQ_NCHANNELS);

  regndx  = VIM_REGNDX(channel);
  channel = VIM_REGBIT(channel);
  bitmask = (1 << channel);

  /* Disable the IRQ/FIQ by setting the corresponding REQMASKCLR bit. */

  regaddr = RM57_VIM_REQMASKCLR(regndx);
  regval  = getreg32(regaddr);
  regval |= bitmask;
  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the IRQ specified by 'channel'
 *
 ****************************************************************************/

void up_enable_irq(int channel)
{
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t bitmask;
  unsigned int regndx;

  DEBUGASSERT(channel >= 0 && channel < RM57_IRQ_NCHANNELS);

  regndx  = VIM_REGNDX(channel);
  channel = VIM_REGBIT(channel);
  bitmask = (1 << channel);

#ifdef CONFIG_ARMV7R_HAVE_DECODEFIQ
  /* Select IRQ (vs FIQ) by clearing the corresponding FIRQPR bit */

  regaddr = RM57_VIM_FIRQPR(regndx);
  regval  = getreg32(regaddr);
  regval &= ~bitmask;
  putreg32(regval, regaddr);
#endif

  /* Enable the IRQ by setting the corresponding REQMASKSET bit. */

  regaddr = RM57_VIM_REQMASKSET(regndx);
  regval  = getreg32(regaddr);
  regval |= bitmask;
  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: up_enable_fiq
 *
 * Description:
 *   Enable the FIQ specified by 'channel'
 *
 ****************************************************************************/

#ifdef CONFIG_ARMV7R_HAVE_DECODEFIQ
void up_enable_fiq(int channel)
{
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t bitmask;
  unsigned int regndx;

  DEBUGASSERT(channel >= 0 && channel < RM57_IRQ_NCHANNELS);

  regndx  = VIM_REGNDX(channel);
  channel = VIM_REGBIT(channel);
  bitmask = (1 << channel);

  /* Select FIQ (vs IRQ) by setting the corresponding FIRQPR bit */

  regaddr = RM57_VIM_FIRQPR(regndx);
  regval  = getreg32(regaddr);
  regval |= bitmask;
  putreg32(regval, regaddr);

  /* Enable the FIQ by setting the corresponding REQMASKSET bit. */

  regaddr = RM57_VIM_REQMASKSET(regndx);
  regval  = getreg32(regaddr);
  regval |= bitmask;
  putreg32(regval, regaddr);
}
#endif

/****************************************************************************
 * Name: arm_ack_irq
 *
 * Description:
 *   Acknowledge the IRQ
 *
 ****************************************************************************/

void arm_ack_irq(int irq)
{
}
