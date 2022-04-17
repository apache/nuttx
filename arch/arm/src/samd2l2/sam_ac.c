/****************************************************************************
 * arch/arm/src/samd2l2/sam_ac.c
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
#include <errno.h>

#include "arm_internal.h"
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

static int sam_ac_isr(int irq, void *context, void *arg)
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

  regval = GCLK_CLKCTRL_ID_ACDIG |
           GCLK_CLKCTRL_GEN(gclkgen) |
           GCLK_CLKCTRL_CLKEN;
  putreg16(regval, SAM_GCLK_CLKCTRL);

  /* Enable comparator analog GCLK */

  regval = GCLK_CLKCTRL_ID_ACANA |
           GCLK_CLKCTRL_GEN(gclkgen) |
           GCLK_CLKCTRL_CLKEN;
  putreg16(regval, SAM_GCLK_CLKCTRL);

  putreg8(AC_CTRLA_ENABLE, SAM_AC_CTRLA);
  sam_ac_syncwait();

  irq_attach(SAM_IRQ_AC, sam_ac_isr, NULL);

  up_enable_irq(SAM_IRQ_AC);

  return OK;
}

int sam_ac_config(uint8_t channel, uint32_t compctrl)
{
  switch (channel)
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
  switch (channel)
    {
      case 0:
        regval = getreg32(SAM_AC_COMPCTRL0);
        if (enable == true)
          {
            regval |= AC_COMPCTRL_ENABLE;
          }
        else
          {
            regval &= ~AC_COMPCTRL_ENABLE;
          }

        putreg32(regval, SAM_AC_COMPCTRL0);
        break;

      case 1:
        regval = getreg32(SAM_AC_COMPCTRL1);
        if (enable == true)
          {
            regval |= AC_COMPCTRL_ENABLE;
          }
        else
          {
            regval &= ~AC_COMPCTRL_ENABLE;
          }

        putreg32(regval, SAM_AC_COMPCTRL1);
        break;

      default:
        return -ENODEV;
    }

  sam_ac_syncwait();

  return OK;
}
