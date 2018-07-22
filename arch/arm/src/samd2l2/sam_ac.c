/****************************************************************************
 * arch/arm/src/samd2l2/sam_ac.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Matt Thompson <matt@extent3d.com>
 *
 * References:
 *   1. "Microchip SAM D21E / SAM D21G / SAM D21J Datasheet"
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
#include <stdbool.h>
#include <assert.h>
#include <errno.h>

#include "up_arch.h"

#include "sam_config.h"

#include "sam_pm.h"
#include "sam_gclk.h"
#include "sam_periphclks.h"
#include "sam_ac.h"
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

static int sam_ac_isr(int irq, FAR void *context, FAR void *arg)
{
  return OK;
}

static void sam_ac_syncwait(void)
{
  while ((getreg8(SAM_AC_STATUSB) & AC_STATUSB_SYNCBUSY) != 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_ac_initialize
 *
 * Description:
 *   Initialize the Analog Comparator (AC).
 *
 * Input Parameters:
 *   gclkgen - GCLK Generator
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

int sam_ac_initialize(uint8_t gclkgen)
{
  uint16_t regval;

  sam_ac_enableperiph();

  /* The Analog Comparators use two GCLKs */

  /* Enable comparator digital GCLK which provides the sampling rate */

  regval = GCLK_CLKCTRL_ID_ACDIG | GCLK_CLKCTRL_GEN(gclkgen) | GCLK_CLKCTRL_CLKEN;
  putreg16(regval, SAM_GCLK_CLKCTRL);

  /* Enable comparator analog GCLK */

  regval = GCLK_CLKCTRL_ID_ACANA | GCLK_CLKCTRL_GEN(gclkgen) | GCLK_CLKCTRL_CLKEN;
  putreg16(regval, SAM_GCLK_CLKCTRL);

  putreg8(AC_CTRLA_ENABLE, SAM_AC_CTRLA);
  sam_ac_syncwait();

  irq_attach(SAM_IRQ_AC, sam_ac_isr, NULL);

  up_enable_irq(SAM_IRQ_AC);

  return OK;
}

int sam_ac_config(uint8_t channel, uint32_t compctrl)
{
  switch(channel)
    {
      case 0:
        putreg32(compctrl, SAM_AC_COMPCTRL0);
        break;
      case 1:
        putreg32(compctrl, SAM_AC_COMPCTRL1);
        break;
      default:
        return -ENODEV;
    }

  sam_ac_syncwait();

  return OK;
}

int sam_ac_enable(uint8_t channel, bool enable)
{
  uint32_t regval;
  switch(channel)
    {
      case 0:
        regval = getreg32(SAM_AC_COMPCTRL0);
        if(enable == true)
          regval |= AC_COMPCTRL_ENABLE;
        else
          regval &= ~AC_COMPCTRL_ENABLE;
        putreg32(regval, SAM_AC_COMPCTRL0);
        break;
      case 1:
        regval = getreg32(SAM_AC_COMPCTRL1);
        if(enable == true)
          regval |= AC_COMPCTRL_ENABLE;
        else
          regval &= ~AC_COMPCTRL_ENABLE;
        putreg32(regval, SAM_AC_COMPCTRL1);
        break;
      default:
        return -ENODEV;
    }

  sam_ac_syncwait();

  return OK;
}
