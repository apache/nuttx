/****************************************************************************
 * arch/arm/src/stm32n6/stm32_otg_phyif.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdint.h>

#include <nuttx/arch.h>

#include "arm_internal.h"
#include "stm32_otg_phyif.h"
#include "hardware/stm32n6xxx_pwr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* VDD33USB is a board supply: give it up to 100 ms to be detected */

#define USB33RDY_TIMEOUT_US  100000

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_otg_phyinit
 *
 * Description:
 *   Power up and configure the embedded high speed PHY of the selected
 *   USB OTG HS instance.
 *   The OTG and PHY clocks are already enabled by the RCC set-up.  The
 *   sequence follows the STM32CubeN6 USB examples: mark VDD33USB valid,
 *   hold the core and the PHY in reset, program the PHY reference clock in
 *   the PHY controller, then release the PHY and the core.
 *
 ****************************************************************************/

int stm32_otg_phyinit(void)
{
  uint32_t regval;
  int timeout;

  /* Enable the VDD33USB monitor, wait until the supply is ready and mark it
   * valid to remove the USB33 isolation.
   */

  modifyreg32(STM32_PWR_SVMCR3, 0, PWR_SVMCR3_USB33VMEN);
  for (timeout = USB33RDY_TIMEOUT_US;
       (getreg32(STM32_PWR_SVMCR3) & PWR_SVMCR3_USB33RDY) == 0;
       timeout--)
    {
      if (timeout == 0)
        {
          return -ETIMEDOUT;
        }

      up_udelay(1);
    }

  modifyreg32(STM32_PWR_SVMCR3, 0, PWR_SVMCR3_USB33SV);

  /* Hold the core and the PHY in reset, release the PHY controller */

  putreg32(STM32_OTGHS_RST | STM32_OTGHS_PHYRST | STM32_OTGHS_PHYCTLRST,
           STM32_RCC_AHB5RSTSR);
  putreg32(STM32_OTGHS_PHYCTLRST, STM32_RCC_AHB5RSTCR);
  up_mdelay(1);

  /* Reference clock frequency, OTG block of the PHY disabled */

  regval  = getreg32(STM32_OTGHS_PHYC_CR);
  regval &= ~USBPHYC_CR_FSEL_MASK;
  regval |= STM32_OTGHS_PHY_FSEL | USBPHYC_CR_OTGDISABLE0 | USBPHYC_CR_CMN |
            USBPHYC_CR_RETENABLEN1;
  putreg32(regval, STM32_OTGHS_PHYC_CR);

  /* Release the PHY, then the core */

  putreg32(STM32_OTGHS_PHYRST, STM32_RCC_AHB5RSTCR);
  up_mdelay(1);
  putreg32(STM32_OTGHS_RST, STM32_RCC_AHB5RSTCR);

  return OK;
}
