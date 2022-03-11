/****************************************************************************
 * arch/arm/src/armv7-a/arm_gicv2.c
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

#include <sys/types.h>
#include <stdint.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <arch/irq.h>

#include "arm_internal.h"
#include "gic.h"

#ifdef CONFIG_ARMV7A_HAVE_GICv2

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_gic0_initialize
 *
 * Description:
 *   Perform common, one-time GIC initialization on CPU0 only.  Both
 *   arm_gic0_initialize() must be called on CPU0; arm_gic_initialize() must
 *   be called for all CPUs.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void arm_gic0_initialize(void)
{
  unsigned int nlines = arm_gic_nlines();
  unsigned int irq;

  arm_gic_dump("Entry arm_gic0_initialize", true, 0);

  /* Initialize SPIs.  The following should be done only by CPU0. */

  /* A processor in Secure State sets:
   *
   * 1. Which interrupts are non-secure (ICDISR).  All set to zero (group
   *    0).
   * 2. Trigger mode of the SPI (ICDICFR). All fields set to 0b01->Level
   *    sensitive, 1-N model.
   * 3. Interrupt Clear-Enable (ICDICER)
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
      putreg32(0x00000000, GIC_ICDISR(irq));   /* SPIs group 0 */
      putreg32(0xffffffff, GIC_ICDICER(irq));  /* SPIs disabled */
    }

  /* Registers with 2-bits per interrupt */

  for (irq = GIC_IRQ_SPI; irq < nlines; irq += 16)
    {
      putreg32(0x55555555, GIC_ICDICFR(irq));  /* SPIs level sensitive */
    }

  /* Registers with 8-bits per interrupt */

  for (irq = GIC_IRQ_SPI; irq < nlines; irq += 4)
    {
      putreg32(0x80808080, GIC_ICDIPR(irq));   /* SPI priority */
      putreg32(0x01010101, GIC_ICDIPTR(irq));  /* SPI on CPU0 */
    }

#ifdef CONFIG_SMP
  /* Attach SGI interrupt handlers. This attaches the handler to all CPUs. */

  DEBUGVERIFY(irq_attach(GIC_IRQ_SGI1, arm_start_handler, NULL));
  DEBUGVERIFY(irq_attach(GIC_IRQ_SGI2, arm_pause_handler, NULL));
#endif

  arm_gic_dump("Exit arm_gic0_initialize", true, 0);
}

/****************************************************************************
 * Name: arm_gic_initialize
 *
 * Description:
 *   Perform common GIC initialization for the current CPU (all CPUs)
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
  uint32_t iccicr;
  uint32_t icddcr;

  arm_gic_dump("Entry arm_gic_initialize", true, 0);

  /* Initialize PPIs.  The following steps need to be done by all CPUs */

  /* Initialize SGIs and PPIs.  NOTE: A processor in non-secure state cannot
   * program its interrupt security registers and must get a secure processor
   * to program the registers.
   */

  /* Registers with 1-bit per interrupt */

  putreg32(0x00000000, GIC_ICDISR(0));      /* SGIs and PPIs secure */
  putreg32(0xf8000000, GIC_ICDICER(0));     /* PPIs disabled */

  /* Registers with 8-bits per interrupt */

  putreg32(0x80808080, GIC_ICDIPR(0));      /* SGI[3:0] priority */
  putreg32(0x80808080, GIC_ICDIPR(4));      /* SGI[4:7] priority */
  putreg32(0x80808080, GIC_ICDIPR(8));      /* SGI[8:11] priority */
  putreg32(0x80808080, GIC_ICDIPR(12));     /* SGI[12:15] priority */
  putreg32(0x80000000, GIC_ICDIPR(24));     /* PPI[0] priority */
  putreg32(0x80808080, GIC_ICDIPR(28));     /* PPI[1:4] priority */

  /* Set the binary point register.
   *
   * Priority values are 8-bit unsigned binary.  The binary point is a 3-bit
   * field; the value n (n=0-6) specifies that bits (n+1) through bit 7 are
   * used in the comparison for interrupt pre-emption.  A GIC supports a
   * minimum of 16 and a maximum of 256 priority levels so not all binary
   * point settings may be meaningul. The special value n=7
   * (GIC_ICCBPR_NOPREMPT) disables pre-emption.  We disable all pre-emption
   * here to prevent nesting of interrupt handling.
   */

  putreg32(GIC_ICCBPR_NOPREMPT, GIC_ICCBPR);

  /* Program the idle priority in the PMR */

  putreg32(GIC_ICCPMR_MASK, GIC_ICCPMR);

  /* Configure the  CPU Interface Control Register */

  iccicr  = getreg32(GIC_ICCICR);

#if defined(CONFIG_ARCH_TRUSTZONE_SECURE) || defined(CONFIG_ARCH_TRUSTZONE_BOTH)
  /* Clear secure state ICCICR bits to be configured below */

  iccicr &= ~(GIC_ICCICRS_FIQEN | GIC_ICCICRS_ACKTCTL | GIC_ICCICRS_CBPR |
              GIC_ICCICRS_EOIMODES | GIC_ICCICRS_EOIMODENS |
              GIC_ICCICRS_ENABLEGRP0 | GIC_ICCICRS_ENABLEGRP1 |
              GIC_ICCICRS_FIQBYPDISGRP0 | GIC_ICCICRS_IRQBYPDISGRP0 |
              GIC_ICCICRS_FIQBYPDISGRP1 | GIC_ICCICRS_IRQBYPDISGRP1);

#elif defined(CONFIG_ARCH_TRUSTZONE_NONSECURE)
  /* Clear non-secure state ICCICR bits to be configured below */

  iccicr &= ~(GIC_ICCICRS_EOIMODENS | GIC_ICCICRU_ENABLEGRP1 |
              GIC_ICCICRU_FIQBYPDISGRP1 | GIC_ICCICRU_IRQBYPDISGRP1);

#endif

#if defined(CONFIG_ARCH_TRUSTZONE_SECURE)
  /* Set FIQn=1 if secure interrupts are to signal using nfiq_c.
   *
   * NOTE:  Only for processors that operate in secure state.
   * REVISIT: Do I need to do this?
   */

  /* iccicr |= GIC_ICCICRS_FIQEN; */

#elif defined(CONFIG_ARCH_TRUSTZONE_BOTH)
  /* Set FIQn=1 if secure interrupts are to signal using nfiq_c.
   *
   * NOTE:  Only for processors that operate in secure state.
   * REVISIT: Do I need to do this?
   */

  iccicr |= GIC_ICCICRS_FIQEN;
#endif

#if defined(CONFIG_ARCH_TRUSTZONE_SECURE)
  /* Program the AckCtl bit to select the required interrupt acknowledge
   * behavior.
   *
   * NOTE: Only for processors that operate in both secure and non-secure
   * state.
   * REVISIT: This is here only for superstituous reasons.  I don't think
   * I need this setting in this configuration.
   */

  iccicr |= GIC_ICCICRS_ACKTCTL;

#elif defined(CONFIG_ARCH_TRUSTZONE_BOTH)
  /* Program the AckCtl bit to select the required interrupt acknowledge
   * behavior.
   *
   * NOTE: Only for processors that operate in both secure and non-secure
   * state.
   */

  iccicr |= GIC_ICCICRS_ACKTCTL;

  /* Program the SBPR bit to select the required binary pointer behavior.
   *
   * NOTE: Only for processors that operate in both secure and non-secure
   * state.
   */

  iccicr |= GIC_ICCICRS_CBPR;
#endif

#if defined(CONFIG_ARCH_TRUSTZONE_SECURE) || defined(CONFIG_ARCH_TRUSTZONE_BOTH)
  /* Set EnableS=1 to enable CPU interface to signal secure interrupts.
   *
   * NOTE:  Only for processors that operate in secure state.
   */

  iccicr |= GIC_ICCICRS_EOIMODES;
#endif

#if defined(CONFIG_ARCH_TRUSTZONE_NONSECURE)
  /* Set EnableNS=1 to enable the CPU to signal non-secure interrupts.
   *
   * NOTE:  Only for processors that operate in non-secure state.
   */

  iccicr |= GIC_ICCICRS_EOIMODENS;

#elif defined(CONFIG_ARCH_TRUSTZONE_BOTH)
  /* Set EnableNS=1 to enable the CPU to signal non-secure interrupts.
   *
   * NOTE:  Only for processors that operate in non-secure state.
   */

  iccicr |= GIC_ICCICRU_EOIMODENS;
#endif

 #ifdef CONFIG_ARCH_TRUSTZONE_BOTH
  /* If the processor operates in both security states and SBPR=0, then it
   * must switch to the other security state and repeat the programming of
   * the binary point register so that the binary point will be programmed
   * for interrupts in both security states.
   */

#  warning Missing logic
#endif

#if !defined(CONFIG_ARCH_HAVE_TRUSTZONE)
  /* Enable the distributor by setting the Enable bit in the enable
   * register (no security extensions).
   */

  iccicr |= GIC_ICCICR_ENABLE;
  icddcr  = GIC_ICDDCR_ENABLE;

#elif defined(CONFIG_ARCH_TRUSTZONE_SECURE)
  /* Enable the Group 0 interrupts, FIQEn and disable Group 0/1
   * bypass.
   */

#if 0 /* REVISIT -- I don't know why this needs to be like this */
  iccicr |= (GIC_ICCICRS_ENABLEGRP0 | GIC_ICCICRS_FIQBYPDISGRP0 |
             GIC_ICCICRS_IRQBYPDISGRP0 | GIC_ICCICRS_FIQBYPDISGRP1 |
             GIC_ICCICRS_IRQBYPDISGRP1);
#else
  iccicr |= (GIC_ICCICRS_ENABLEGRP0 | GIC_ICCICRS_ENABLEGRP1 |
             GIC_ICCICRS_FIQBYPDISGRP0 | GIC_ICCICRS_IRQBYPDISGRP0 |
             GIC_ICCICRS_FIQBYPDISGRP1 | GIC_ICCICRS_IRQBYPDISGRP1);
#endif
  icddcr  = GIC_ICDDCR_ENABLEGRP0;

#elif defined(CONFIG_ARCH_TRUSTZONE_BOTH)
  /* Enable the Group 0/1 interrupts, FIQEn and disable Group 0/1
   * bypass.
   */

  iccicr |= (GIC_ICCICRS_ENABLEGRP0 | GIC_ICCICRS_ENABLEGRP1 |
             GIC_ICCICRS_FIQBYPDISGRP0 | GIC_ICCICRS_IRQBYPDISGRP0 |
             GIC_ICCICRS_FIQBYPDISGRP1 | GIC_ICCICRS_IRQBYPDISGRP1);
  icddcr  = (GIC_ICDDCR_ENABLEGRP0 | GIC_ICDDCR_ENABLEGRP1);

#else /* defined(CONFIG_ARCH_TRUSTZONE_NONSECURE) */
  /* Enable the Group 1 interrupts and disable Group 1 bypass. */

  iccicr |= (GIC_ICCICRU_ENABLEGRP1 | GIC_ICCICRU_FIQBYPDISGRP1 |
             GIC_ICCICRU_IRQBYPDISGRP1);
  icddcr  = GIC_ICDDCR_ENABLE;

#endif

  /* Write the final ICCICR value to enable the GIC. */

  putreg32(iccicr, GIC_ICCICR);

#ifdef CONFIG_ARCH_TRUSTZONE_BOTH
  /* A processor in the secure state must then switch to the non-secure
   * a repeat setting of the enable bit in the enable register.  This
   * enables distributor to respond to interrupt in both security states.
   * REVISIT: Initial implementation operates only in secure state.
   */

#  warning Missing logic
#endif

  /* Write the ICDDCR value to enable the forwarding of interrupt by the
   * distributor.
   */

  putreg32(icddcr, GIC_ICDDCR);
  arm_gic_dump("Exit arm_gic_initialize", true, 0);
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
 *  Input Parameters:
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

  DEBUGASSERT(irq < NR_IRQS || irq == 1023);
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
 *   (up_irq_restore() supports the global level, the device level is
 *   hardware specific).
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

      arm_gic_dump("Exit up_enable_irq", false, irq);
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

      arm_gic_dump("Exit up_disable_irq", false, irq);
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

      arm_gic_dump("Exit up_prioritize_irq", false, irq);
      return OK;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: arm_gic_irq_trigger
 *
 * Description:
 *   Set the trigger type for the specified IRQ source and the current CPU.
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 * Input Parameters:
 *   irq - The interrupt request to modify.
 *   edge - False: Active HIGH level sensitive, True: Rising edge sensitive
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

int arm_gic_irq_trigger(int irq, bool edge)
{
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t intcfg;

  if (irq > GIC_IRQ_SGI15 && irq < NR_IRQS)
    {
      /* Get the address of the Interrupt Configuration Register for this
       * irq.
       */

      regaddr = GIC_ICDICFR(irq);

      /* Get the new Interrupt configuration bit setting */

      intcfg = (edge ? (INT_ICDICFR_EDGE | INT_ICDICFR_1N) : INT_ICDICFR_1N);

      /* Write the correct interrupt trigger to the Interrupt Configuration
       * Register.
       */

      regval  = getreg32(regaddr);
      regval &= ~GIC_ICDICFR_ID_MASK(irq);
      regval |= GIC_ICDICFR_ID(irq, intcfg);
      putreg32(regval, regaddr);

      return OK;
    }

  return -EINVAL;
}

#endif /* CONFIG_ARMV7A_HAVE_GICv2 */
