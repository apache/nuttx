/****************************************************************************
 * arch/arm/src/samd2l2/sam_eic.c
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

/* References:
 *   1. "Microchip SAM D21E / SAM D21G / SAM D21J Datasheet"
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#include "arm_internal.h"
#include "sam_config.h"

#include "sam_pm.h"
#include "sam_gclk.h"
#include "sam_periphclks.h"
#include "sam_eic.h"
#include "sam_port.h"

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/irq.h>

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int sam_eic_isr(int irq, void *context, void *arg)
{
  uint32_t intflag;
  int bit;

  /* Get the pending interrupt flag register */

  intflag = getreg32(SAM_EIC_INTFLAG);

  /* Dispatch the IRQ to the SAM_IRQ_EXTINTn handlers */

  for (bit = 0; bit < SAM_IRQ_NEXTINTS; bit++)
    {
      if (intflag >> bit & 0x1)
        {
          irq_dispatch(SAM_IRQ_EXTINT0 + bit, context);
        }
    }

  /* Clear the pending interrupt flags */

  putreg32(EIC_EXTINT_ALL, SAM_EIC_INTFLAG);

  return 0;
}

static void sam_eic_syncwait(void)
{
  while ((getreg8(SAM_EIC_STATUS) & EIC_STATUS_SYNCBUSY) != 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void sam_eic_dumpregs(void)
{
  irqinfo("EIC:\n");
  irqinfo("  CTRLA:    %02x\n", getreg8(SAM_EIC_CTRLA));
  irqinfo("  STATUS:   %02x\n", getreg8(SAM_EIC_STATUS));
  irqinfo("  NMICTRL:  %02x\n", getreg8(SAM_EIC_NMICTRL));
  irqinfo("  NMIFLAG:  %02x\n", getreg8(SAM_EIC_NMIFLAG));
  irqinfo("  EVCTRL:   %08x\n", getreg32(SAM_EIC_EVCTRL));
  irqinfo("  INTENCLR: %08x\n", getreg32(SAM_EIC_INTENCLR));
  irqinfo("  INTENSET: %08x\n", getreg32(SAM_EIC_INTENSET));
  irqinfo("  INTFLAG:  %08x\n", getreg32(SAM_EIC_INTFLAG));
  irqinfo("  WAKEUP:   %08x\n", getreg32(SAM_EIC_WAKEUP));
  irqinfo("  CONFIG0:  %08x\n", getreg32(SAM_EIC_CONFIG0));
  irqinfo("  CONFIG1:  %08x\n", getreg32(SAM_EIC_CONFIG1));
  irqinfo("  CONFIG2:  %08x\n", getreg32(SAM_EIC_CONFIG2));
}

/****************************************************************************
 * Name: sam_eic_initialize
 *
 * Description:
 *   Initialize the external interrupt controller (EIC).
 *
 * Input Parameters:
 *   gclkgen - GCLK Generator
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

int sam_eic_initialize(uint8_t gclkgen)
{
  uint16_t regval;

  sam_eic_enableperiph();

  regval = GCLK_CLKCTRL_ID_EIC | GCLK_CLKCTRL_GEN(gclkgen) |
           GCLK_CLKCTRL_CLKEN;
  putreg16(regval, SAM_GCLK_CLKCTRL);

  putreg8(EIC_CTRLA_ENABLE, SAM_EIC_CTRLA);
  sam_eic_syncwait();

  irq_attach(SAM_IRQ_EIC, sam_eic_isr, NULL);

  sam_eic_dumpregs();

  up_enable_irq(SAM_IRQ_EIC);

  return OK;
}

/****************************************************************************
 * Name: sam_eic_initialize
 *
 * Description:
 *   Enable a external interrupt.
 *
 * Input Parameters:
 *   irq - SAM_IRQ_EXTINTn IRQ to be enabled
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

int sam_eic_irq_enable(int irq)
{
  uint32_t config;
  int eirq = irq - SAM_IRQ_EXTINT0;

  config  = getreg32(SAM_EIC_CONFIG0);
  config |= EIC_CONFIG0_FILTEN(eirq) | EIC_CONFIG0_SENSE_FALL(eirq);
  putreg32(config, SAM_EIC_CONFIG0);

  putreg32(EIC_EXTINT(eirq), SAM_EIC_INTENSET);
  sam_eic_dumpregs();
  return OK;
}

/****************************************************************************
 * Name: sam_eic_config
 *
 * Description:
 *   Configure the interrupt edge sensitivity in CONFIGn register of the EIC
 *
 * Input Parameters:
 *   eirq    - Pin to be configured
 *   pinset  - Configuration of the pin
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

int sam_eic_config(uint8_t eirq, port_pinset_t pinset)
{
  uint32_t reg;
  uint32_t val;
  uint32_t config;

  /* Determine which of the CONFIG[0:2] registers to write to */

  if (eirq < 8)
    {
      reg = SAM_EIC_CONFIG0;

      val = EIC_CONFIG0_SENSE_BOTH(eirq);
      if (pinset & PORT_INT_RISING)
        {
          val = EIC_CONFIG0_SENSE_RISE(eirq);
        }

      if (pinset & PORT_INT_FALLING)
        {
          val = EIC_CONFIG0_SENSE_FALL(eirq);
        }

      val |= EIC_CONFIG0_FILTEN(eirq);
    }
  else if (eirq < 16)
    {
      reg = SAM_EIC_CONFIG1;

      val = EIC_CONFIG1_SENSE_BOTH(eirq);
      if (pinset & PORT_INT_RISING)
        {
          val = EIC_CONFIG1_SENSE_RISE(eirq);
        }

      if (pinset & PORT_INT_FALLING)
        {
          val = EIC_CONFIG1_SENSE_FALL(eirq);
        }

      val |= EIC_CONFIG1_FILTEN(eirq);
    }
  else
    {
      reg = SAM_EIC_CONFIG2;

      val = EIC_CONFIG2_SENSE_BOTH(eirq);
      if (pinset & PORT_INT_RISING)
        {
          val = EIC_CONFIG2_SENSE_RISE(eirq);
        }

      if (pinset & PORT_INT_FALLING)
        {
          val = EIC_CONFIG2_SENSE_FALL(eirq);
        }

      val |= EIC_CONFIG2_FILTEN(eirq);
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
