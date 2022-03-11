/****************************************************************************
 * arch/arm/src/imx1/imx_gpio.c
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

#include "chip.h"
#include "arm_internal.h"
#include "imx_gpio.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxgpio_configoutput
 ****************************************************************************/

void imxgpio_configoutput(int port, int bit, int value)
{
  imxgpio_configinput(port, bit);            /* Same as input except: */
  imxgpio_dirout(port, bit);                 /* Output */

  if (value)
    {
      imxgpio_setoutput(port, bit);          /* Set output = 1 */
    }
  else
    {
      imxgpio_clroutput(port, bit);          /* Set output = 0 */
    }
}

/****************************************************************************
 * Name: imxgpio_configinput
 ****************************************************************************/

void imxgpio_configinput(int port, int bit)
{
  imxgpio_pullupdisable(port, bit);          /* No pullup */
  imxgpio_dirin(port, bit);                  /* Input */
  imxgpio_gpiofunc(port, bit);               /* Use as GPIO */
  imxgpio_primaryperipheralfunc(port, bit);  /* Not necessary */
  imxgpio_ocrain(port, bit);                 /* Output AIN */
  imxgpio_aoutgpio(port, bit);               /* AOUT input is GPIO */
  imxgpio_boutgpio(port, bit);               /* BOUT input is GPIO */
}

/****************************************************************************
 * Name: imxgpio_configpfoutput
 ****************************************************************************/

void imxgpio_configpfoutput(int port, int bit)
{
  imxgpio_configinput(port, bit);            /* Same as input except: */
  imxgpio_peripheralfunc(port, bit);         /*   Use as peripheral */
  imxgpio_primaryperipheralfunc(port, bit);  /*   Primary function */
  imxgpio_dirout(port, bit);                 /*   Make output */
}

/****************************************************************************
 * Name: imxgpio_configpfinput
 ****************************************************************************/

void imxgpio_configpfinput(int port, int bit)
{
  imxgpio_configinput(port, bit);            /* Same as input except: */
  imxgpio_peripheralfunc(port, bit);         /*   Use as peripheral */
  imxgpio_primaryperipheralfunc(port, bit);  /*   Primary function */
}
