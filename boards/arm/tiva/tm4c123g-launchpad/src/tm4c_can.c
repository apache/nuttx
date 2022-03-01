/****************************************************************************
 * boards/arm/tiva/tm4c123g-launchpad/src/tm4c_can.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2020 Matthew Trescott
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Based heavily on stm32_can.c from the boards directory.
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

#include <errno.h>
#include <debug.h>

#include <nuttx/can/can.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"

#include "tiva_can.h"
#include "tm4c123g-launchpad.h"

#include "tiva_enableclks.h"
#include "tiva_gpio.h"
#include "hardware/tiva_pinmap.h"

#ifdef CONFIG_CAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tm4c_can_setup
 *
 * Description:
 *  Initialize CAN and register the CAN device
 *
 ****************************************************************************/

int tm4c_can_setup(void)
{
#ifdef CONFIG_TIVA_CAN
  int ret;

#  ifdef CONFIG_TIVA_CAN0
  tiva_can0_enableclk();

  ret = tiva_configgpio(GPIO_CAN0_RX);

  if (ret < 0)
    {
      goto configgpio_error;
    }

  ret = tiva_configgpio(GPIO_CAN0_TX);

  if (ret < 0)
    {
      goto configgpio_error;
    }

  /* Call tiva_can_initialize() to get an instance of CAN interface 0
   * and register it.
   */

  ret = tiva_can_initialize("/dev/can0", 0);
  if (ret < 0)
    {
      canerr("ERROR:  Failed to get/register CAN interface 0\n");
      return ret;
    }
#  endif /* CONFIG_TIVA_CAN0 */

#  ifdef CONFIG_TIVA_CAN1
  tiva_can1_enableclk();

  ret = tiva_configgpio(GPIO_CAN1_RX);

  if (ret < 0)
    {
      goto configgpio_error;
    }

  ret = tiva_configgpio(GPIO_CAN1_TX);

  if (ret < 0)
    {
      goto configgpio_error;
    }

  /* Call tiva_can_initialize() to get an instance of CAN interface 1
   * and register it.
   */

  ret = tiva_can_initialize("/dev/can1", 1);
  if (ret < 0)
    {
      canerr("ERROR:  Failed to get/register CAN interface 1\n");
      return ret;
    }
#  endif /* CONFIG_TIVA_CAN1 */

  return OK;

configgpio_error:
  canerr("ERROR: failed to configure CAN GPIO pin.\n");
  return ret;
#else
  return -ENODEV;
#endif
}

#endif /* CONFIG_CAN */

