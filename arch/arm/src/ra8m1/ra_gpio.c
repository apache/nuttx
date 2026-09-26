/****************************************************************************
 * arch/arm/src/ra8m1/ra_gpio.c
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

#include <stdint.h>
#include <time.h>
#include <errno.h>

#include <nuttx/debug.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "ra_gpio.h"
#include "hardware/ra8m1_pinmap.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 *   cfgset.cfg is written to the pin's PmnPFS register in a single 32-bit
 *   access: PMR/PSEL/PCR/NCODR/DSCR select the pin's function and drive
 *   characteristics, while PDR/PODR set its initial direction/output level
 *   -- PmnPFS.PDR and PmnPFS.PODR are the same physical bits as
 *   PORTm.PCNTR1's PDR/PODR (RA8M1 User's Manual 19.2.5), so no separate
 *   PCNTR1 write is needed here.
 *
 ****************************************************************************/

void ra_configgpio(gpio_pinset_t cfgset)
{
  uint8_t regval;

  regval = R_PFS_PWPR_PFSWE;
  putreg8(0, R_PFS_PWPR);
  putreg8(regval, R_PFS_PWPR);

  putreg32(cfgset.cfg, R_PFS(cfgset.port, cfgset.pin));

  regval = R_PFS_PWPR_B0WI;
  putreg8(0, R_PFS_PWPR);
  putreg8(regval, R_PFS_PWPR);
}

/****************************************************************************
 * Name: ra_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void ra_gpiowrite(gpio_pinset_t pinset, bool value)
{
  if (value)
    {
      putreg16((1 << pinset.pin), R_PORT_POSR(pinset.port));
    }
  else
    {
      putreg16((1 << pinset.pin), R_PORT_PORR(pinset.port));
    }
}

/****************************************************************************
 * Name: ra_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool ra_gpioread(gpio_pinset_t pinset)
{
  return (getreg16(R_PORT_PIDR(pinset.port)) &
    (uint16_t)(0x01 << pinset.pin));
}
