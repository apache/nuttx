/****************************************************************************
 * arch/arm/src/s32k3xx/s32k3xx_pin.c
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

/* Copyright 2022 NXP */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/board/board.h>

#include <assert.h>
#include <errno.h>

#include <nuttx/arch.h>

#include "arm_internal.h"

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include "s32k3xx_pin.h"
#include "hardware/s32k3xx_siul2.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k3xx_pinconfig
 *
 * Description:
 *   Configure a pin based on a bit-encoded description of the pin.
 *   NOTE that DMA/interrupts are disabled at the initial PIN configuration.
 *
 ****************************************************************************/

int s32k3xx_pinconfig(uint32_t cfgset)
{
  unsigned int port;
  unsigned int pin;
  unsigned int input_mode;
  unsigned int imcr;
  unsigned int output_mode;
  uint32_t     regval;

  /* Get the port and pin number */

  port = (cfgset & _PIN_PORT_MASK) >> _PIN_PORT_SHIFT;
  pin  = (cfgset & _PIN_MASK)      >> _PIN_SHIFT;

  DEBUGASSERT(port < S32K3XX_NPORTS);
  DEBUGASSERT(pin < S32K3XX_NPINS);
  if ((port >= S32K3XX_NPORTS) && (pin >= S32K3XX_NPINS))
    {
      return -EINVAL; /* Invalid port or pin number */
    }

  /* Get the desired input mode */

  input_mode = (cfgset & _PIN_INPUT_MODE_MASK) >> _PIN_INPUT_MODE_SHIFT;

  /* Get IMCR number (may also be the WKPU number) */

  imcr = (cfgset & _IMCR_MASK) >> _IMCR_SHIFT;

  /* Check if a valid input mode was defined */

  if (input_mode <= _PIN_INPUT_MODE_ALT11)
    {
      /* The desired input mode should only be written to the IMCR register
       * if it does not disable the input signal.  Otherwise writing the SSS
       * to the IMCR register could cause unwanted behavior (i.e. disable an
       * input signal which might have been assigned to another pin).
       *
       * To completely disable the input signal and make sure it is not
       * assigned to any pins, change the IMCR register directly.
       *
       * TO DO: Create a mapping of input signals to IMCR registers and
       * maybe create a function to deassign and disable the signal.
       */

      if (input_mode != _PIN_INPUT_MODE_DIS)
        {
          putreg32(SIUL2_IMCR_SSS(input_mode), S32K3XX_SIUL2_IMCR(imcr));
        }
      else
        {
          /* Else we just ignore the selection by doing nothing... */
        }
    }
  else if (input_mode == _PIN_INPUT_MODE_WKPU)
    {
      /* WKPU interrupt mode */

      /* WKPU number is stored in IMCR field */
    }
  else
    {
      return -EINVAL; /* Undefined input mode! */
    }

  /* Get the desired output mode */

  output_mode = (cfgset & _PIN_OUTPUT_MODE_MASK) >> _PIN_OUTPUT_MODE_SHIFT;

  /* Prepare the value that will be written to the MSCR register */

  regval = 0;

  /* Source Signal Select */

  regval |= SIUL2_MSCR_SSS(output_mode);

  /* Drive Strength Enable */

  if ((cfgset & _PIN_OUTPUT_DRIVE_MASK) == PIN_OUTPUT_HIGHDRIVE)
    {
      regval |= SIUL2_MSCR_DSE;
    }

  /* Pull Enable & Pull Select */

  if ((cfgset & _PIN_INPUT_PULLENABLE) == _PIN_INPUT_PULLENABLE)
    {
      if ((cfgset & _PIN_INPUT_PULL_MASK) == PIN_INPUT_PULLDOWN)
        {
          regval |= SIUL2_MSCR_PUE; /* Pull-Down */
        }
      else
        {
          regval |= (SIUL2_MSCR_PUE | SIUL2_MSCR_PUS); /* Pull-Up */
        }
    }

  /* Slew Rate Control */

  if ((cfgset & _PIN_OUTPUT_SLEW_MASK) == PIN_OUTPUT_SLOWSLEW)
    {
      regval |= SIUL2_MSCR_SRC;
    }

  /* Input Buffer Enable (IBE) */

  regval |= SIUL2_MSCR_IBE;

  /* Output Buffer Enable (OBE) */

  if ((cfgset & _PIN_OUTPUT_BUFFER_MASK) == PIN_OUTPUT_BUFFERENA)
    {
      regval |= SIUL2_MSCR_OBE;
    }

  /* Write to the corresponding MSCR for this pin */

  putreg32(regval, S32K3XX_SIUL2_MSCR((port << 5) + pin));

  /* For GPIO outputs: Set the initial value */

  if ((output_mode == _PIN_MODE_GPIO) &&
     ((cfgset & _PIN_OUTPUT_BUFFER_MASK) == PIN_OUTPUT_BUFFERENA))
    {
      s32k3xx_gpiowrite(cfgset, ((cfgset & GPIO_OUTPUT_ONE) != 0));
    }

  return OK;
}
