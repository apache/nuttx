/****************************************************************************
 * arch/arm/src/armv7-a/arm_gic.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <stdint.h>
#include <errno.h>

#include <arch/irq.h>

#include "up_arch.h"
#include "up_internal.h"
#include "gic.h"

#ifdef CONFIG_ARMV7A_HAVE_GIC

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_gic_initialize
 *
 * Description:
 *   Perform basic GIC initialization for the current CPU
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void arm_gic_initialize(void)
{
  unsigned int nlines;
  unsigned int irq;
  uint32_t regval;
  uint32_t field;
#ifdef CONFIG_SMP
  int cpu;

  /* Which CPU are we initializing */

  cpu = up_cpu_index();
#endif

  /* Get the number of interrupt lines. */

  regval = getreg32(GIC_ICDICTR);
  field  = (regval & GIC_ICDICTR_ITLINES_MASK) >> GIC_ICDICTR_ITLINES_SHIFT;
  nlines = (field + 1) << 5;

  /* Initialize SPIs.  The following should be done only by CPU0. */

#ifdef CONFIG_SMP
  if (cpu == 0)
#endif
    {
      /* A processor in Secure State sets:
       *
       * 1. Which interrupts are non-secure (ICDISR).
       *    REVISIT: Which bit state corresponds to secure?
       * 2. Trigger mode of the SPI (ICDICFR). All fields set to 11->Edge
       *    sensitive.
       * 3. Innterrupt Clear-Enable (ICDICER)
       * 3. Priority of the SPI using the priority set register (ICDIPR).
       *    Priority values are 8-bit unsigned binary. A GIC supports a
       *    minimum of 16 and a maximum of 256 priority levels. Here all
       *    are set to the middle priority 128 (0x80).
       * 4. Target that receives the SPI interrupt (ICDIPTR).  Set all to
       *    CPU0.
       */

      /* Registers with 1-bit per interrupt */

      for (irq = GIC_IRQ_SPI; irq < nlines; irq += 32)
        {
          putreg32(0x00000000, GIC_ICDISR(irq));   /* SPIs secure */
          putreg32(0xffffffff, GIC_ICDICFR(irq));  /* SPIs edge triggered */
          putreg32(0xffffffff, GIC_ICDICER(irq));  /* SPIs disabled */
        }

      /* Registers with 8-bits per interrupt */

      for (irq = GIC_IRQ_SPI; irq < nlines; irq += 8)
        {
          putreg32(0x80808080, GIC_ICDIPR(irq));   /* SPI priority */
          putreg32(0x01010101, GIC_ICDIPTR(irq));  /* SPI on CPU0 */
        }
    }

  /* The remaining steps need to be done by all CPUs */

  /* Initialize SGIs and PPIs.  NOTE: A processor in non-secure state cannot
   * program its interrupt security registers and must get a secure processor
   * to program the registers.
   */

  /* Registers with 1-bit per interrupt */

  putreg32(0x00000000, GIC_ICDISR(0));      /* SGIs and PPIs secure */
  putreg32(0xf8000000, GIC_ICDICER(0));     /* PPIs disabled */

  /* Registers with 8-bits per interrupt */

  putreg32(0x80808080, GIC_ICDIPR(0));         /* SGI[3:0] priority */
  putreg32(0x80808080, GIC_ICDIPR(4));         /* SGI[4:7] priority */
  putreg32(0x80808080, GIC_ICDIPR(8));         /* SGI[8:11] priority */
  putreg32(0x80808080, GIC_ICDIPR(12));        /* SGI[12:15] priority */
  putreg32(0x80000000, GIC_ICDIPR(24));        /* PPI[0] priority */
  putreg32(0x80808080, GIC_ICDIPR(28));        /* PPI[1:4] priority */

#if defined(CONFIG_ARCH_TRUSTZONE_SECURE) || defined(CONFIG_ARCH_TRUSTZONE_BOTH)
  /* Set FIQn=1 if secure interrupts are to signal using nfiq_c.
   *
   * NOTE:  Only for processors that operate in secure state.
   * REVISIT: Do I need to do this?
   */

#endif

#ifdef CONFIG_ARCH_TRUSTZONE_BOTH
  /* Program the AckCtl bit to select the required interrupt acknowledge
   * behavior.
   *
   * NOTE: Only for processors that operate in both secure and non-secure
   * state.
   */

#  warning Missing logic

  /* Program the SBPR bit to select the required binary pointer behavior.
   *
   * NOTE: Only for processors that operate in both secure and non-secure
   * state.
   */

#  warning Missing logic
#endif

#if defined(CONFIG_ARCH_TRUSTZONE_SECURE) || defined(CONFIG_ARCH_TRUSTZONE_BOTH)
  /* Set EnableS=1 to enable CPU interface to signal secure interrupts.
   *
   * NOTE:  Only for processors that operate in secure mostatede.
   */

#  warning Missing logic
#endif

#if defined(CONFIG_ARCH_TRUSTZONE_NONSECURE) || defined(CONFIG_ARCH_TRUSTZONE_BOTH)
  /* Set EnableNS=1 to enable the CPU to signal non-secure interrupts.
   *
   * NOTE:  Only for processors that operate in non-secure state.
   * REVISIT: Initial implementation operates only in secure state.
   */

#  warning Missing logic
#endif

  /* Set the binary point register.
   *
   * Priority values are 8-bit unsigned binary.  The binary point is a 3-bit
   * field; the value n (n=0-6) specifies that bits (n+1) through bit 7 are
   * used in the comparison for interrupt pre-emption.  A GIC supports a
   * minimum of 16 and a maximum of 256 priority levels so not all binary
   * point settings may be meaningul. The special value n=7 (GIC_ICCBPR_NOPREMPT)
   * disables pre-emption.  We disable all pre-emption here to prevent nesting
   * of interrupt handling.
   */

  putreg32(GIC_ICCBPR_NOPREMPT, GIC_ICCBPR);

#ifdef CONFIG_ARCH_TRUSTZONE_BOTH
  /* If the processor operates in both security states and SBPR=0, then it
   * must switch to the other security state and repeat the programming of
   * the binary point register so that the binary point will be programmed
   * for interrupts in both security states.
   */

#  warning Missing logic
#endif

  /* Enable the distributor by setting the the Enable bit in the enable
   * register.
   */

  putreg32(GIC_ICCICR_ENABLE, GIC_ICCICR);

#ifdef CONFIG_ARCH_TRUSTZONE_BOTH
  /* A processor in the secure state must then switch to the non-secure
   * a repeat setting of the enable bit in the enable register.  This
   * enables distributor to respond to interrupt in both security states.
   * REVISIT: Initial implementation operates only in secure state.
   */

#  warning Missing logic
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
  uint32_t regval;
  int irq;

  /* Read the interrupt acknowledge register and get the interrupt ID */

  regval = getreg32(GIC_ICCIAR);
  irq    = (regval & GIC_ICCIAR_INTID_MASK) >> GIC_ICCIAR_INTID_SHIFT;

  /* Ignore spurions IRQs.  ICCIAR will report 1023 if there is no pending
   * interrupt.
   */

  DEBUGASSERT(irg < NR_IRQS || irq == 1023);
  if (irq < NR_IRQS)
    {
      /* Dispatch the interrupt */

      regs = arm_doirq(irq, regs);
    }

  /* Write to the end-of-interrupt register */

  putreg32(regval, GIC_ICCEOIR);
  return regs;
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   On many architectures, there are three levels of interrupt enabling: (1)
 *   at the global level, (2) at the level of the interrupt controller,
 *   and (3) at the device level.  In order to receive interrupts, they
 *   must be enabled at all three levels.
 *
 *   This function implements enabling of the device specified by 'irq'
 *   at the interrupt controller level if supported by the architecture
 *   (up_irq_restore() supports the global level, the device level is hardware
 *   specific).
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  /* Ignore invalid interrupt IDs.  Also, in the Cortex-A9 MPCore, SGIs are
   * always enabled. The corresponding bits in the ICDISERn are read as
   * one, write ignored.
   */

  if (irq > GIC_IRQ_SGI15 && irq < NR_IRQS)
    {
      uintptr_t regaddr;

      /* Write '1' to the corresponding bit in the distributor Interrupt
       * Set-Enable Register (ICDISER)
       */

      regaddr = GIC_ICDISER(irq);
      putreg32(GIC_ICDISER_INT(irq), regaddr);
    }
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   This function implements disabling of the device specified by 'irq'
 *   at the interrupt controller level if supported by the architecture
 *   (up_irq_save() supports the global level, the device level is hardware
 *   specific).
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  /* Ignore invalid interrupt IDs.  Also, in the Cortex-A9 MPCore, SGIs are
   * always enabled. The corresponding bits in the ICDISERn are read as
   * one, write ignored.
   */

  if (irq > GIC_IRQ_SGI15 && irq < NR_IRQS)
    {
      uintptr_t regaddr;

      /* Write '1' to the corresponding bit in the distributor Interrupt
       * Clear-Enable Register (ICDISER)
       */

      regaddr = GIC_ICDICER(irq);
      putreg32(GIC_ICDICER_INT(irq), regaddr);
    }
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

int up_prioritize_irq(int irq, int priority)
{
  DEBUGASSERT(irq >= 0 && irq < NR_IRQS && priority >= 0 && priority <= 255);

  /* Ignore invalid interrupt IDs */

  if (irq >= 0 && irq < NR_IRQS)
    {
      uintptr_t regaddr;
      uint32_t regval;

      /* Write the new priority to the corresponding field in the in the
       * distributor Interrupt Priority Register (GIC_ICDIPR).
       */

      regaddr  = GIC_ICDIPR(irq);
      regval   = getreg32(regaddr);
      regval  &= ~GIC_ICDIPR_ID_MASK(irq);
      regval  |= GIC_ICDIPR_ID(irq, priority);
      putreg32(regval, regaddr);

      return OK;
    }

  return -EINVAL;
}

#endif /* CONFIG_ARMV7A_HAVE_GIC */
