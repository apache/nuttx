/****************************************************************************
 * arch/arm/src/sama5/sam_irq.c
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

#include <stdint.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/irq.h>

#include "up_arch.h"
#include "os_internal.h"
#include "up_internal.h"

#ifdef CONFIG_SAMA5_PIO_IRQ
#  include "sam_pio.h"
#endif

#include "chip/sam_aic.h"
#include "chip/sam_matrix.h"
#include "chip/sam_aximx.h"

#include "sam_irq.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private types
 ****************************************************************************/

typedef uint32_t *(*doirq_t)(int irq, uint32_t *regs);

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile uint32_t *current_regs;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_srctype[SCRTYPE_NTYPES] =
{
 0, 0, 1, 1, 2, 3
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_dumpaic
 *
 * Description:
 *   Dump some interesting AIC registers
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_IRQ)
static void sam_dumpaic(const char *msg, int irq)
{
  irqstate_t flags;

  flags = irqsave();
  lldbg("AIC (%s, irq=%d):\n", msg, irq);

  /* Select the register set associated with this irq */

  putreg32(irq, SAM_AIC_SSR);

  /* Then dump all of the (readable) register contents */

  lldbg("  SSR: %08x  SMR: %08x  SVR: %08x  IVR: %08x\n",
        getreg32(SAM_AIC_SSR),  getreg32(SAM_AIC_SMR),
        getreg32(SAM_AIC_SVR),  getreg32(SAM_AIC_IVR));
  lldbg("  FVR: %08x ISR: %08x\n",
        getreg32(SAM_AIC_FVR),  getreg32(SAM_AIC_ISR));
  lldbg("  IPR: %08x       %08x       %08x       %08x\n",
        getreg32(SAM_AIC_IPR0), getreg32(SAM_AIC_IPR1),
        getreg32(SAM_AIC_IPR2), getreg32(SAM_AIC_IPR3));
  lldbg("  IMR: %08x CISR: %08x  SPU: %08x FFSR: %08x\n",
        getreg32(SAM_AIC_IMR),  getreg32(SAM_AIC_CISR),
        getreg32(SAM_AIC_SPU),  getreg32(SAM_AIC_FFSR));
  lldbg("  DCR: %08x WPMR: %08x WPMR: %08x\n",
        getreg32(SAM_AIC_DCR),  getreg32(SAM_AIC_WPMR),
        getreg32(SAM_AIC_WPMR));
  irqrestore(flags);
}
#else
#  define sam_dumpaic(msg, irq)
#endif

/****************************************************************************
 * Name: sam_spurious
 *
 * Description:
 *   Spurious interrupt handler.
 *
 *   Paragraph 17.8.6, Spurious Interrupt: "The Advanced Interrupt Controller
 *   features protection against spurious interrupts. A spurious interrupt is
 *   defined as being the assertion of an interrupt source long enough for the
 *   AIC to assert the nIRQ, but no longer present when AIC_IVR is read. This
 *   is most prone to occur when:
 *
 *     o An external interrupt source is programmed in level-sensitive mode
 *       and an active level occurs for only a short time.
 *     o An internal interrupt source is programmed in level sensitive and
 *       the output signal of the corresponding embedded peripheral is
 *       activated for a short time. (As in the case for the Watchdog.)
 *     o An interrupt occurs just a few cycles before the software begins to
 *       mask it, thus resulting in a pulse on the interrupt source.
 *
 *   "The AIC detects a spurious interrupt at the time the AIC_IVR is read
 *   while no enabled interrupt source is pending. When this happens, the AIC
 *   returns the value stored by the programmer in AIC_SPU (Spurious Vector
 *   Register). The programmer must store the address of a spurious interrupt
 *   handler in AIC_SPU as part of the application, to enable an as fast as
 *   possible return to the normal execution flow. This handler writes in
 *   AIC_EOICR and performs a return from interrupt."
 *
 ****************************************************************************/

static uint32_t *sam_spurious(int irq, uint32_t *regs)
{
  /* This is probably irrevelant since true vectored interrupts are not used
   * in this implementation.  The value of AIC_IVR is ignored.
   */

#if defined(CONFIG_DEBUG_IRQ)
  lldbg("Spurious interrupt: IRQ %d\n", irq);
#endif
  return regs;
}

/****************************************************************************
 * Name: sam_fiqhandler
 *
 * Description:
 *   Default FIQ interrupt handler.
 *
 ****************************************************************************/

static uint32_t *sam_fiqhandler(int irq, uint32_t *regs)
{
  /* This is probably irrevelant since FIQs are not used in this
   * implementation.
   */

#if defined(CONFIG_DEBUG_IRQ) || defined(CONFIG_ARCH_STACKDUMP)
  lldbg("FIQ?: IRQ: %d\n");
#endif
  PANIC();
  return regs; /* Won't get here */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 *
 * Description:
 *   This function is called by up_initialize() during the bring-up of the
 *   system.  It is the responsibility of this function to but the interrupt
 *   subsystem into the working and ready state.
 *
 ****************************************************************************/

void up_irqinitialize(void)
{
  int i;

  /* The following operations need to be atomic, but since this function is
   * called early in the intialization sequence, we expect to have exclusive
   * access to the AIC.
   */

  /* Colorize the interrupt stack for debug purposes */

#if defined(CONFIG_DEBUG_STACK) && CONFIG_ARCH_INTERRUPTSTACK > 3
  {
    size_t intstack_size = (CONFIG_ARCH_INTERRUPTSTACK & ~3);
    up_stack_color((FAR void *)((uintptr_t)&g_intstackbase - intstack_size),
                   intstack_size);
  }
#endif

  /* Unprotect SMR, SVR, SPU and DCR register */

  putreg32(AIC_WPMR_WPKEY, SAM_AIC_WPMR);

  /* Configure the FIQ and the IRQs. */

  for (i = 0; i < SAM_IRQ_NINT; i++)
    {
      /* Select the interrupt registers */

      putreg32(i, SAM_AIC_SSR);

      /* Disable the interrupt */

      putreg32(AIC_IDCR_INTD, SAM_AIC_IDCR);

      /* Set the (unused) FIQ/IRQ handler */

      if (i == SAM_PID_FIQ)
        {
          putreg32((uint32_t)sam_fiqhandler, SAM_AIC_SVR);
        }
      else
        {
          putreg32((uint32_t)arm_doirq, SAM_AIC_SVR);
        }

      /* Set the default interrupt priority */

      putreg32(SAM_DEFAULT_PRIOR, SAM_AIC_SMR);

      /* Clear any pending interrupt */

      putreg32(AIC_ICCR_INTCLR, SAM_AIC_ICCR);
    }

  /* Set the (unused) spurious interrupt handler */

  putreg32((uint32_t)sam_spurious, SAM_AIC_SPU);

  /* Perform 8 interrupt acknowledgements by writing any value to the
   * EOICR register.
   */

  for (i = 0; i < 8 ; i++)
    {
      putreg32(AIC_EOICR_ENDIT, SAM_AIC_EOICR);
    }

  /* Restore protection and the interrupt state */

  putreg32(AIC_WPMR_WPKEY | AIC_WPMR_WPEN, SAM_AIC_WPMR);

#if defined(CONFIG_ARCH_LOWVECTORS) && defined(CONFIG_SAMA5_BOOT_ISRAM)
  /* Disable MATRIX write protection */

#if 0 /* Disabled on reset */
  putreg32(MATRIX_WPMR_WPKEY, SAM_MATRIX_WPMR);
#endif

  /* Set remap state 0 if we are running from internal SRAM.  If we booted
   * into NOR FLASH, then the first level bootloader should have already
   * provided this mapping for us.
   *
   * This is done late in the boot sequence.  Any exceptions taken before
   * this point in time will be handled by the ROM code, not by the NuttX
   * interrupt since which was, up to this point, uninitialized.
   *
   *   Boot state:    ROM is seen at address 0x00000000
   *   Remap State 0: SRAM is seen at address 0x00000000 (through AHB slave
   *                  interface) instead of ROM.
   *   Remap State 1: HEBI is seen at address 0x00000000 (through AHB slave
   *                  interface) instead of ROM for external boot.
   *
   * Here we are assuming that vectors reside in the lower end of ISRAM.
   * Hmmm... this probably does not matter since we will map a page to
   * address 0x0000:0000 in that case anyway.
   */

  putreg32(MATRIX_MRCR_RCB0, SAM_MATRIX_MRCR);   /* Enable remap */
  putreg32(AXIMX_REMAP_REMAP0, SAM_AXIMX_REMAP); /* Remap SRAM */

  /* Restore MATRIX write protection */

#if 0 /* Disabled on reset */
  putreg32(MATRIX_WPMR_WPKEY | MATRIX_WPMR_WPEN, SAM_MATRIX_WPMR);
#endif

  /* It might be wise to flush the instruction cache here */
#endif

  /* currents_regs is non-NULL only while processing an interrupt */

  current_regs = NULL;

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  /* Initialize logic to support a second level of interrupt decoding for
   * PIO pins.
   */

#ifdef CONFIG_SAMA5_PIO_IRQ
  sam_pioirqinitialize();
#endif

  /* And finally, enable interrupts */

  (void)irqenable();
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
 *  Input paramters:
 *   regs - A pointer to the register save area on the stack.
 *
 ****************************************************************************/

uint32_t *arm_decodeirq(uint32_t *regs)
{
  uint32_t irqid;
  uint32_t ivr;

 /* Paragraph 17.8.5 Protect Mode: "The Protect Mode permits reading the
  *   Interrupt Vector Register without performing the associated automatic
  *   operations. ... Writing PROT in AIC_DCR (Debug Control Register) at 0x1
  *   enables the Protect Mode.
  *
  *  "When the Protect Mode is enabled, the AIC performs interrupt stacking
  *    only when a write access is performed on the AIC_IVR. Therefore, the
  *    Interrupt Service Routines must write (arbitrary data) to the AIC_IVR
  *    just after reading it. The new context of the AIC, including the value
  *    of the Interrupt Status Register (AIC_ISR), is updated with the current
  *    interrupt only when AIC_IVR is written. ..."
  *
  *  "To summarize, in normal operating mode, the read of AIC_IVR performs the
  *   following operations within the AIC:
  *
  *   1. Calculates active interrupt (higher than current or spurious).
  *   2. Determines and returns the vector of the active interrupt.
  *   3. Memorizes the interrupt.
  *   4. Pushes the current priority level onto the internal stack.
  *   5. Acknowledges the interrupt.
  *
  * "However, while the Protect Mode is activated, only operations 1 to 3 are
  *  performed when AIC_IVR is read.  Operations 4 and 5 are only performed by
  *  the AIC when AIC_IVR is written.
  *
  * "Software that has been written and debugged using the Protect Mode runs
  *  correctly in Normal Mode without modification. However, in Normal Mode the
  *  AIC_IVR write has no effect and can be removed to optimize the code.
  */

  /* Write in the IVR to support Protect Mode */

  ivr = getreg32(SAM_AIC_IVR);
  putreg32(ivr, SAM_AIC_IVR);

  /* Get the IRQ number from the interrrupt status register.  NOTE that the
   * IRQ number is the same is the peripheral ID (PID).
   */

  irqid = getreg32(SAM_AIC_ISR) & AIC_ISR_MASK;

  /* Dispatch the interrupt */

  regs = ((doirq_t)ivr)((int)irqid, regs);

  /* Acknowledge interrupt */

  putreg32(AIC_EOICR_ENDIT, SAM_AIC_EOICR);
  return regs;
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  irqstate_t flags;

  if (irq < SAM_IRQ_NINT)
    {
      /* These operations must be atomic */

      flags = irqsave();

      /* Select the register set associated with this irq */

      putreg32(irq, SAM_AIC_SSR);

      /* Disable the interrupt */

      putreg32(AIC_IDCR_INTD, SAM_AIC_IDCR);
      sam_dumpaic("disable", irq);
      irqrestore(flags);
    }
#ifdef CONFIG_SAMA5_PIO_IRQ
  else
    {
      /* Maybe it is a (derived) PIO IRQ */

      sam_pioirqdisable(irq);
    }
#endif
  sam_dumpaic("disable", irq);
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  irqstate_t flags;

  if (irq < SAM_IRQ_NINT)
    {
      /* These operations must be atomic */

      flags = irqsave();

      /* Select the register set associated with this irq */

      putreg32(irq, SAM_AIC_SSR);

      /* Enable the interrupt */

      putreg32(AIC_IECR_INTEN, SAM_AIC_IECR);
      sam_dumpaic("enable", irq);
      irqrestore(flags);
    }
#ifdef CONFIG_SAMA5_PIO_IRQ
  else
    {
      /* Maybe it is a (derived) PIO IRQ */

      sam_pioirqenable(irq);
    }
#endif
}

/****************************************************************************
 * Name: up_maskack_irq
 *
 * Description:
 *   Mask the IRQ and acknowledge it
 *
 ****************************************************************************/

void up_maskack_irq(int irq)
{
  up_disable_irq(irq);
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
  irqstate_t flags;
  uint32_t regval;

  DEBUGASSERT(irq < SAM_IRQ_NINT && (unsigned)priority <= AIC_SMR_PRIOR_MASK);
  if (irq < SAM_IRQ_NINT)
    {
      /* These operations must be atomic */

      flags = irqsave();

      /* Select the register set associated with this irq */

      putreg32(irq, SAM_AIC_SSR);

      /* Unprotect and write the SMR register */

      putreg32(AIC_WPMR_WPKEY, SAM_AIC_WPMR);

      /* Set the new priority, preserving the current srctype */

      regval  = getreg32(SAM_AIC_SMR);
      regval &= ~AIC_SMR_PRIOR_MASK;
      regval |= (uint32_t)priority << AIC_SMR_PRIOR_SHIFT;
      putreg32(regval, SAM_AIC_SMR);

      /* Restore protection and the interrupt state */

      putreg32(AIC_WPMR_WPKEY | AIC_WPMR_WPEN, SAM_AIC_WPMR);
      sam_dumpaic("prioritize", irq);
      irqrestore(flags);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: sam_irq_srctype
 *
 * Description:
 *   irq     - Identifies the IRQ source to be configured
 *   srctype - IRQ source configuration
 *
 ****************************************************************************/

void sam_irq_srctype(int irq, enum sam_srctype_e srctype)
{
  irqstate_t flags;
  uint32_t regval;

  DEBUGASSERT(irq < SAM_IRQ_NINT && (unsigned)srctype < SCRTYPE_NTYPES);

  /* These operations must be atomic */

  flags = irqsave();

  /* Select the register set associated with this irq */

  putreg32(irq, SAM_AIC_SSR);

  /* Unprotect and write the SMR register */

  putreg32(AIC_WPMR_WPKEY, SAM_AIC_WPMR);

  /* Set the new srctype, preserving the current priority */

  regval  = getreg32(SAM_AIC_SMR);
  regval &= ~AIC_SMR_SRCTYPE_MASK;
  regval |= (uint32_t)g_srctype[srctype] << AIC_SMR_SRCTYPE_SHIFT;
  putreg32(regval, SAM_AIC_SMR);

  /* Restore protection and the interrupt state */

  putreg32(AIC_WPMR_WPKEY | AIC_WPMR_WPEN, SAM_AIC_WPMR);
  sam_dumpaic("srctype", irq);
  irqrestore(flags);
}

