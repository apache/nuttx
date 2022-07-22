/****************************************************************************
 * arch/arm/src/s32k3xx/s32k3xx_pingpio.c
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

#include <assert.h>
#include <errno.h>

#include "arm_internal.h"
#include "hardware/s32k3xx_siul2.h"
#include "s32k3xx_pin.h"

#include <arch/board/board.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k3xx_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void s32k3xx_gpiowrite(uint32_t pinset, bool value)
{
  unsigned int port;
  unsigned int pin;

  DEBUGASSERT((pinset & _PIN_OUTPUT_MODE_MASK)   == PIN_MODE_GPIO);
  DEBUGASSERT((pinset & _PIN_OUTPUT_BUFFER_MASK) == PIN_OUTPUT_BUFFERENA);

  /* Get the port and pin number */

  port = (pinset & _PIN_PORT_MASK) >> _PIN_PORT_SHIFT;
  pin  = (pinset & _PIN_MASK)      >> _PIN_SHIFT;

  DEBUGASSERT(port < S32K3XX_NPORTS);
  DEBUGASSERT(pin < S32K3XX_NPINS);
  if ((port < S32K3XX_NPORTS) && (pin < S32K3XX_NPINS))
    {
      /* Set or clear the output */

      putreg8(value, S32K3XX_SIUL2_GPDO((port << 5) + pin));
    }
}

/****************************************************************************
 * Name: s32k3xx_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool s32k3xx_gpioread(uint32_t pinset)
{
  unsigned int port;
  unsigned int pin;
  bool         ret = false;

  /* Get the port and pin number */

  port = (pinset & _PIN_PORT_MASK) >> _PIN_PORT_SHIFT;
  pin  = (pinset & _PIN_MASK)      >> _PIN_SHIFT;

  DEBUGASSERT(port < S32K3XX_NPORTS);
  DEBUGASSERT(pin < S32K3XX_NPINS);
  if ((port < S32K3XX_NPORTS) && (pin < S32K3XX_NPINS))
    {
      ret = getreg8(S32K3XX_SIUL2_GPDI((port << 5) + pin));
    }

  return ret;
}
