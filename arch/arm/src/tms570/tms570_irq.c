/****************************************************************************
 * arch/arm/src/tms570/tms570_irq.c
 *
 *   Copyright (C) 2015-2016 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/irq.h>

#include "up_arch.h"
#include "up_internal.h"

#include "chip/tms570_vim.h"
#include "tms570_gio.h"
#include "tms570_esm.h"
#include "tms570_irq.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_current_regs[] holds a references to the current interrupt level
 * register storage structure.  If is non-NULL only during interrupt
 * processing.  Access to g_current_regs[] must be through the macro
 * CURRENT_REGS for portability.
 */

volatile uint32_t *g_current_regs[1];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tms570_error_handler
 ****************************************************************************/

static void tms570_error_handler(void)
{
  PANIC();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize

 * The device supports three different possibilities for software to handle
 * interrupts:
 *
 *   1. Index interrupts mode (compatible with TMS470R1x legacy code),
 *   2. Register vectored interrupts (automatically provide vector address
 *      to application)
 *   3. Hardware vectored interrupts (automatically dispatch to ISR, IRQ
 *      only)
 *
 * Only the indexed mode is supported here: After the interrupt is received
 * by the CPU, the CPU branches to 0x18 (IRQ) or 0x1C (FIQ) to execute the
 * main ISR. The main ISR routine reads the offset register (IRQINDEX,
 * FIQINDEX) to determine the source of the interrupt.
 *
 * To use mode 2), it would only be necessary to initialize the VIM_RAM.
 * To use mode 3), it would be necessary to initialize the VIM_RAM and also
 * to set the vector enable (VE) bit in the CP15 R1 register.  This bit is
 * zero on reset so that the default state after reset is backward
 * compatible to earlier ARM CPU.
 *
 ****************************************************************************/

#define TMS570_VIM_FIRQPR3 (TMS570_VIM_BASE+0x001c)
#define TMS570_VIM_REQENASET3 (TMS570_VIM_BASE+0x003c)
#define TMS570_VIM_REQENACLR3 (TMS570_VIM_BASE+0x004c)

void up_irqinitialize(void)
{
  FAR uintptr_t *vimram;
  int i;

  /* Initialize VIM RAM vectors.  These vectors are not used in the current
   * interrupt handler logic.
   */

  vimram = (FAR uintptr_t *)TMS570_VIMRAM_BASE;
  for (i = 0; i < (TMS570_IRQ_NCHANNELS + 1); i++)
    {
      *vimram++ = (uintptr_t)tms570_error_handler;
    }

  /* Set Fall-Back Address Parity Error Register (also not used) */

  putreg32((uint32_t)tms570_error_handler, TMS570_VIM_FBPARERR);

  /* Assign all interrupt requests to the VIM channel of the same value.
   * NOTE: Nothing need be done.  That is the power-on default mapping.
   */

  /* Assign all channels to IRQs */

  putreg32(0, TMS570_VIM_FIRQPR0);
  putreg32(0, TMS570_VIM_FIRQPR1);
  putreg32(0, TMS570_VIM_FIRQPR2);
#ifdef TMS570_VIM_FIRQPR3
  putreg32(0, TMS570_VIM_FIRQPR3);
#endif

  /* Disable all interrupts */

  putreg32(0xfffffffc, TMS570_VIM_REQENACLR0);
  putreg32(0xffffffff, TMS570_VIM_REQENACLR1);
  putreg32(0xffffffff, TMS570_VIM_REQENACLR2);
#ifdef TMS570_VIM_REQENACLR3
  putreg32(0xffffffff, TMS570_VIM_REQENACLR3);
#endif

  /* currents_regs is non-NULL only while processing an interrupt */

  CURRENT_REGS = NULL;

#ifdef CONFIG_ARMV7R_HAVE_DECODEFIQ
  /* By default, interrupt CHAN0 is mapped to ESM (Error Signal Module)
   * high level interrupt and CHAN1 is reserved for other NMI. For safety
   * reasons, these two channels are mapped to FIQ only and can NOT be
   * disabled through ENABLE registers.
   */

#endif

#ifndef CONFIG_SUPPRESS_INTERRUPTS
#ifdef CONFIG_TMS570_GIO_IRQ
  /* Initialize logic to support a second level of interrupt decoding for
   * GIO pins.
   */

  tms570_gioirq_initialize();
#endif

  /* Attach and enable ESM interrupts.  The high level interrupt is really
   * an NMI.
   */

  (void)irq_attach(TMS570_REQ_ESMHIGH, tms570_esm_interrupt, NULL);
  (void)irq_attach(TMS570_REQ_ESMLO, tms570_esm_interrupt, NULL);
  up_enable_irq(TMS570_REQ_ESMHIGH);
  up_enable_irq(TMS570_REQ_ESMLO);

  /* And finally, enable interrupts globally */

  up_irq_enable();
#endif
}

/****************************************************************************
 * Name: arm_decodeirq
 *
 * Description:
 *   This function is called from the IRQ vector handler in arm_vectors.S.
 *   At this point, the interrupt has been taken and the registers have
 *   been saved on the stack.  This function simply needs to determine the
 *   the irq number of the interrupt and then to call arm_doirq to dispatch
 *   the interrupt.
 *
 * Input Parameters:
 *   regs - A pointer to the register save area on the stack.
 *
 ****************************************************************************/

uint32_t *arm_decodeirq(uint32_t *regs)
{
  int vector;

  /* Check for a VRAM parity error.
   *
   * REVISIT: This is not to critical in this implementation since VIM RAM
   * is not used.
   */

  /* Get the interrupting vector number from the IRQINDEX register.  Zero,
   * the "phantom" vector will returned.
   */

  vector = getreg32(TMS570_VIM_IRQINDEX) & VIM_IRQINDEX_MASK;
  if (vector > 0)
    {
      /* Dispatch the interrupt.  NOTE that the IRQ number is the vector
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
 *   At this point, the interrupt has been taken and the registers have
 *   been saved on the stack.  This function simply needs to determine the
 *   the irq number of the interrupt and then to call arm_doirq to dispatch
 *   the interrupt.
 *
 *  Input Parameters:
 *   regs - A pointer to the register save area on the stack.
 *
 ****************************************************************************/

#ifdef CONFIG_ARMV7R_HAVE_DECODEFIQ
uint32_t *arm_decodefiq(FAR uint32_t *regs)
{
  int vector;

  /* Check for a VRAM parity error.
   *
   * REVISIT: This is not to critical in this implementation since VIM RAM
   * is not used.
   */

  /* Get the interrupting vector number from the FIQINDEX register.  Zero,
   * the "phantom" vector will returned.
   */

  vector = getreg32(TMS570_VIM_FIQINDEX) & VIM_FIQINDEX_MASK;
  if (vector > 0)
    {
      /* Dispatch the interrupt.  NOTE that the IRQ number is the vector
       * number offset by one to skip over the "phantom" vector.
       */

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

  DEBUGASSERT(channel >= 0 && channel < TMS570_IRQ_NCHANNELS);

  /* Offset to account for the "phantom" vector */

  regndx   = VIM_REGNDX(channel);
  channel  = VIM_REGBIT(channel);
  bitmask  = (1 << channel);

  /* Disable the IRQ/FIQ by setting the corresponding REQENACLR bit. */

  regaddr  =  TMS570_VIM_REQENACLR(regndx);
  regval   = getreg32(regaddr);
  regval  |= bitmask;
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

  DEBUGASSERT(channel >= 0 && channel < TMS570_IRQ_NCHANNELS);

  /* Offset to account for the "phantom" vector */

  regndx  = VIM_REGNDX(channel);
  channel = VIM_REGBIT(channel);
  bitmask = (1 << channel);

#ifdef CONFIG_ARMV7R_HAVE_DECODEFIQ
  /* Select IRQ (vs FIQ) by clearing the corresponding FIRQPR bit */

  regaddr  = TMS570_VIM_FIRQPR(regndx);
  regval   = getreg32(regaddr);
  regval  &= ~bitmask;
  putreg32(regval, regaddr);
#endif

  /* Enable the IRQ by setting the corresponding REQENASET bit. */

  regaddr  = TMS570_VIM_REQENASET(regndx);
  regval   = getreg32(regaddr);
  regval  |= bitmask;
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

  DEBUGASSERT(channel >= 0 && channel < TMS570_IRQ_NCHANNELS);

  /* Offset to account for the "phantom" vector */

  regndx  = VIM_REGNDX(channel);
  channel = VIM_REGBIT(channel);
  bitmask = (1 << channel);

  /* Select FIQ (vs IRQ) by setting the corresponding FIRQPR bit */

  regaddr  = TMS570_VIM_FIRQPR(regndx);
  regval   = getreg32(regaddr);
  regval  &= ~bitmask;
  putreg32(regval, regaddr);

  /* Enable the FIQ by setting the corresponding REQENASET bit. */

  regaddr  = TMS570_VIM_REQENASET(regndx);
  regval   = getreg32(regaddr);
  regval  |= bitmask;
  putreg32(regval, regaddr);
}
#endif

/****************************************************************************
 * Name: up_ack_irq
 *
 * Description:
 *   Acknowledge the IRQ
 *
 ****************************************************************************/

void up_ack_irq(int irq)
{
}
