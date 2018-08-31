/****************************************************************************
 * arch/arm/src/samd5e5/sam_eic.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Matt Thompson <matt@extent3d.com>
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
#include <assert.h>
#include <debug.h>

#include "up_arch.h"
#include "sam_gclk.h"
#include "sam_periphclks.h"
#include "sam_port.h"
#include "sam_eic.h"

#include <arch/board/board.h>

#ifdef CONFIG_SAMD5E5_EIC

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_eic_syncwait
 *
 * Description:
 *   Wait until the ENABLE synchronization is complete
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_eic_syncwait(void)
{
  while ((getreg8(SAM_EIC_SYNCBUSY) & EIC_SYNCBUSY_ENABLE) != 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_eic_dumpregs
 *
 * Description:
 *   Dump all external interrupt controller (EIC) registers
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_eic_dumpregs(void)
{
  irqinfo("EIC:\n");
  irqinfo("       CTRLA:  %02x\n", getreg8(SAM_EIC_CTRLA));
  irqinfo("     NMICTRL:  %02x\n", getreg8(SAM_EIC_NMICTRL));
  irqinfo("     NMIFLAG:  %04x\n", getreg16(SAM_EIC_NMIFLAG));
  irqinfo("    SYNCBUSY:  %08x\n", getreg32(SAM_EIC_SYNCBUSY));
  irqinfo("      EVCTRL:  %08x\n", getreg32(SAM_EIC_EVCTRL));
  irqinfo("    INTENCLR:  %08x\n", getreg32(SAM_EIC_INTENCLR));
  irqinfo("    INTENSET:  %08x\n", getreg32(SAM_EIC_INTENSET));
  irqinfo("     INTFLAG:  %08x\n", getreg32(SAM_EIC_INTFLAG));
  irqinfo("      ASYNCH:  %08x\n", getreg32(SAM_EIC_ASYNCH));
  irqinfo("     CONFIG0:  %08x\n", getreg32(SAM_EIC_CONFIG0));
  irqinfo("     CONFIG1:  %08x\n", getreg32(SAM_EIC_CONFIG1));
  irqinfo("   DEBOUNCEN:  %08x\n", getreg32(SAM_EIC_DEBOUNCEN));
  irqinfo("  DPRESCALER:  %08x\n", getreg32(SAM_EIC_DPRESCALER));
  irqinfo("    PINSTATE:  %08x\n", getreg32(SAM_EIC_PINSTATE));
}

/****************************************************************************
 * Name: sam_eic_initialize
 *
 * Description:
 *   Initialize the EIC.  Called one timer during system bring-up.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

int sam_eic_initialize(void)
{
  uintptr_t regaddr;
  uint32_t regval;

  /* Configure the EIC APB clock */

  sam_apb_eic_enableperiph();

  /* Use the selected GCLK_EIC.  Some optional functions need a peripheral
   * clock, which can either be a generic clock (GCLK_EIC, for wider
   * frequency selection) or a Ultra Low Power 32KHz clock (CLK_ULP32K, for
   * highest power efficiency). One of the clock sources must be configured
   * and enabled before using the peripheral.
   */

  regaddr = SAM_GCLK_PCHCTRL(GCLK_CHAN_EIC);
  regval  = GCLK_PCHCTRL_GEN(BOARD_GCLK_EIC) | GCLK_PCHCTRL_CHEN;
  putreg32(regval, regaddr);

  /* Enable the EIC, selecting clocking via the GCLK_EIC  */

  putreg8(EIC_CTRLA_ENABLE | EIC_CTRLA_ENABLE, SAM_EIC_CTRLA);
  sam_eic_syncwait();

  sam_eic_dumpregs();
  return OK;
}

/****************************************************************************
 * Name: sam_eic_configure
 *
 * Description:
 *   Configure the interrupt edge sensitivity in CONFIGn register of the
 *   EIC.  The interrupt will be enabled at the EIC (but not at the NVIC).
 *
 * Input Parameters:
 *   eirq    - Pin to be configured (0..15)
 *   pinset  - Configuration of the pin
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

int sam_eic_configure(uint8_t eirq, port_pinset_t pinset)
{
  uint32_t reg;
  uint32_t val;
  uint32_t config;

  /* Determine which of the CONFIG[0:1] registers to write to */

  if (eirq < 8)
    {
      reg = SAM_EIC_CONFIG0;

      val = EIC_CONFIG0_SENSE_BOTH(eirq);
      if ((pinset & PORT_INT_RISING) != 0)
        {
          val = EIC_CONFIG0_SENSE_RISE(eirq);
        }

      if ((pinset & PORT_INT_FALLING) != 0)
        {
          val = EIC_CONFIG0_SENSE_FALL(eirq);
        }

      val |= EIC_CONFIG0_FILTEN(eirq);
    }
  else /* if (eirq < 16) */
    {
      reg = SAM_EIC_CONFIG1;

      val = EIC_CONFIG1_SENSE_BOTH(eirq);
      if ((pinset & PORT_INT_RISING) != 0)
        {
          val = EIC_CONFIG1_SENSE_RISE(eirq);
        }

      if ((pinset & PORT_INT_FALLING) != 0)
        {
          val = EIC_CONFIG1_SENSE_FALL(eirq);
        }

      val |= EIC_CONFIG1_FILTEN(eirq);
    }

  /* Write the new config to the CONFIGn register */

  config  = getreg32(reg);
  config |= val;
  putreg32(config, reg);

  /* Enable interrupt generation for this pin */

  putreg32(EIC_EXTINT(eirq), SAM_EIC_INTENSET);

  sam_eic_dumpregs();
  return OK;
}

/****************************************************************************
 * Name: sam_eic_irq_ack
 *
 * Description:
 *   Acknowledge receipt of an external interrupt.
 *
 * Input Parameters:
 *   irq - SAM_IRQ_EXTINTn IRQ to be acknowledged, n=0-15
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

int sam_eic_irq_ack(int irq)
{
  int eirq = irq - SAM_IRQ_EXTINT0;

  putreg32(EIC_EXTINT(eirq), SAM_EIC_INTENCLR);
  return OK;
}

#endif /* CONFIG_SAMD5E5_EIC */
