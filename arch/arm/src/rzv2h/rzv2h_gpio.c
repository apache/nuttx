/****************************************************************************
 * arch/arm/src/rzv2h/rzv2h_gpio.c
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

#include <stdint.h>
#include <time.h>
#include <errno.h>

#include <nuttx/debug.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "rzv2h_fsp_err.h"
#include "rzv2h_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rzv2h_configgpio
 *
 * Description:
 *   Configure a GPIO pin using the bit-encoded pin configuration supplied
 *   by the caller.
 ****************************************************************************/

int rzv2h_configgpio(gpio_pinset_t pinset)
{
  fsp_err_t err;

  err = R_IOPORT_PinCfg(NULL, pinset.port_pin, pinset.cfg);
  return rzv2h_fsp_err_to_errno(err);
}

/****************************************************************************
 * Name: rzv2h_gpiowrite
 *
 * Description:
 *   Write a logic high or low value to the selected GPIO pin.
 ****************************************************************************/

int rzv2h_gpiowrite(gpio_pinset_t pinset, bool value)
{
  fsp_err_t err;
  bsp_io_level_t level = value ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW;

  err = R_IOPORT_PinWrite(NULL, pinset.port_pin, level);
  return rzv2h_fsp_err_to_errno(err);
}

/****************************************************************************
 * Name: rzv2h_gpioread
 *
 * Description:
 *   Read the current logic level from the selected GPIO pin.
 ****************************************************************************/

bool rzv2h_gpioread(gpio_pinset_t pinset)
{
  bsp_io_level_t level;
  fsp_err_t err;

  err = R_IOPORT_PinRead(NULL, pinset.port_pin, &level);
  if (err != FSP_SUCCESS)
    {
      return false;
    }

  return level == BSP_IO_LEVEL_HIGH;
}
