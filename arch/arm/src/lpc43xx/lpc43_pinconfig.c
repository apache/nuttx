/****************************************************************************
 * arch/arm/src/lpc43xx/lpc43_pinconfig.c
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

#include <arch/board/board.h>
#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <errno.h>

#include "arm_internal.h"
#include "lpc43_pinconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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
 * Name: lpc43_pin_config
 *
 * Description:
 *   Configure a pin based on bit-encoded description of the pin.
 *
 * Input Parameters:
 *   20-bit encoded value describing the pin.
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

int lpc43_pin_config(uint32_t pinconf)
{
  unsigned int pinset = ((pinconf & PINCONF_PINS_MASK) >>
                                    PINCONF_PINS_SHIFT);
  unsigned int pin    = ((pinconf & PINCONF_PIN_MASK) >>
                                    PINCONF_PIN_SHIFT);
  unsigned int func   = ((pinconf & PINCONF_FUNC_MASK) >>
                                    PINCONF_FUNC_SHIFT);
  uintptr_t regaddr;
  uint32_t  regval;

  /* Set up common pin configurations */

  regval = (func << SCU_PIN_MODE_SHIFT);

  /* Enable/disable pull-down resistor */

  if (PINCONF_IS_PULLDOWN(pinconf))
    {
      regval |= SCU_PIN_EPD;  /* Set bit to enable */
    }

  if (!PINCONF_IS_PULLUP(pinconf))
    {
      regval |= SCU_PIN_EPUN; /* Set bit to disable */
    }

  /* Enable/disable input buffering */

  if (PINCONF_INBUFFER_ENABLED(pinconf))
    {
      regval |= SCU_PIN_EZI; /* Set bit to enable */
    }

  /* Enable/disable glitch filtering */

  if (!PINCONF_GLITCH_ENABLE(pinconf))
    {
      regval |= SCU_PIN_ZIF; /* Set bit to disable */
    }

  /* Only normal and high speed pins support the slew rate setting */

  if (PINCONF_IS_SLEW_FAST(pinconf))
    {
      regval |= SCU_NDPIN_EHS; /* 0=slow; 1=fast */
    }

  /* Only high drive pins suppose drive strength */

  switch (pinconf & PINCONF_DRIVE_MASK)
    {
      default:
      case PINCONF_DRIVE_NORMAL: /* Normal-drive: 4 mA drive strength (or not high drive pin) */
        regval |= SCU_HDPIN_EHD_NORMAL;
        break;

      case PINCONF_DRIVE_MEDIUM: /* Medium-drive: 8 mA drive strength */
        regval |= SCU_HDPIN_EHD_MEDIUM;
        break;

      case PINCONF_DRIVE_HIGH: /* High-drive: 14 mA drive strength */
        regval |= SCU_HDPIN_EHD_HIGH;
        break;

      case PINCONF_DRIVE_ULTRA: /* Ultra high-drive: 20 mA drive strength */
        regval |= SCU_HDPIN_EHD_ULTRA;
        break;
    }

  /* Get the address of the pin configuration register and save the new
   * pin configuration.
   */

  regaddr =  LPC43_SCU_SFSP(pinset, pin);
  putreg32(regval, regaddr);

  return OK;
}
