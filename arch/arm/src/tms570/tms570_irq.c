/****************************************************************************
 * arch/arm/src/tms570/tms570_irq.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/irq.h>

#include "up_arch.h"
#include "up_internal.h"

#include "chip/tms570_vim.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* This is the address of current interrupt saved state data.  Used for
 * context switching.  Only value during interrupt handling.
 */

volatile uint32_t *current_regs;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  /* Disable all interrupts. */
#warning Missing logic

  /* Colorize the interrupt stack for debug purposes */

#if defined(CONFIG_STACK_COLORATION) && CONFIG_ARCH_INTERRUPTSTACK > 3
  {
    size_t intstack_size = (CONFIG_ARCH_INTERRUPTSTACK & ~3);
    up_stack_color((FAR void *)((uintptr_t)&g_intstackbase - intstack_size),
                   intstack_size);
  }
#endif

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

  current_regs = NULL;

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  /* Initialize logic to support a second level of interrupt decoding for
   * GPIO pins.
   */

#ifdef CONFIG_TMS570_GPIO_IRQ
  tms570_gpioirqinitialize();
#endif

  /* And finally, enable interrupts */

  irqenable();
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
 *  Input parameters:
 *   regs - A pointer to the register save area on the stack.
 *
 ****************************************************************************/

uint32_t *arm_decodeirq(uint32_t *regs)
{
#warning Missing Logic
  return 0;
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
 *  Input parameters:
 *   regs - A pointer to the register save area on the stack.
 *
 ****************************************************************************/

#ifdef CONFIG_ARMV7R_HAVE_DECODEFIQ
uint32_t *arm_decodefiq(FAR uint32_t *regs)
{
#warning Missing Logic
  return 0;
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

  DEBUGASSERT(channel >= 0 && channel < TMS570_IRQ_NCHANNELS)

  /* Offset to account for the "phantom" vector */

  channel++;
  regndx   = VIM_REGNDX(channel);
  channel  = VIM_REGBIT(channel);
  bitmask  = (1 << channel);

  /* Disable the IRQ/FIQ by setting the corresponding REQENACLR bit. */

  regaddr  =  TMS570_VIM_REQENACLR(regndx);
  regval   = getreg32(regaddr);
  regaddr |= bitmask;
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

  DEBUGASSERT(channel >= 0 && channel < TMS570_IRQ_NCHANNELS)

  /* Offset to account for the "phantom" vector */

  channel++;
  regndx  = VIM_REGNDX(channel);
  channel = VIM_REGBIT(channel);
  bitmask = (1 << channel);

#ifdef CONFIG_ARMV7R_HAVE_DECODEFIQ
  /* Select IRQ (vs FIQ) by clearing the corresponding FIRQPR bit */

  regaddr  = TMS570_VIM_FIRQPR(regndx);
  regval   = getreg32(regaddr);
  regaddr &= ~bitmask;
  putreg32(regval, regaddr);
#endif

  /* Enable the IRQ by setting the corresponding REQENASET bit. */

  regaddr  = TMS570_VIM_REQENASET(regndx);
  regval   = getreg32(regaddr);
  regaddr |= bitmask;
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

  DEBUGASSERT(channel >= 0 && channel < TMS570_IRQ_NCHANNELS)

  /* Offset to account for the "phantom" vector */

  channel++;
  regndx  = VIM_REGNDX(channel);
  channel = VIM_REGBIT(channel);
  bitmask = (1 << channel);

  /* Select FIQ (vs IRQ) by setting the corresponding FIRQPR bit */

  regaddr  = TMS570_VIM_FIRQPR(regndx);
  regval   = getreg32(regaddr);
  regaddr &= ~bitmask;
  putreg32(regval, regaddr);

  /* Enable the FIQ by setting the corresponding REQENASET bit. */

  regaddr  = TMS570_VIM_REQENASET(regndx);
  regval   = getreg32(regaddr);
  regaddr |= bitmask;
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
#warning Missing logic
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
int up_prioritize_irq(int channel, int priority)
{
#warning Missing logic
}
#endif
