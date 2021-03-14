/****************************************************************************
 * boards/arm/stm32f7/nucleo-144/src/stm32_can.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 *   Imported by Roberto Bucher <roberto.bucher@supsi.ch>
 *   and adapted from original stm32l4/nucleo-l476rg/src
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

/* Added by: Ben vd Veen (DisruptiveNL) -- www.nuttx.nl */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/can/can.h>

#include "stm32_can.h"
#include "nucleo-144.h"

#ifdef CONFIG_CAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* #ifdef CONFIG_STM32F7_CAN1 */
/* #  define CAN_PORT 1 */
/* #endif */

#ifdef CONFIG_STM32F7_CAN1
#  define CAN_PORT 1
#else
#  define CAN_PORT 2
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32f7_can_setup
 *
 * Description:
 *  Initialize CAN and register the CAN device
 *
 ****************************************************************************/

int stm32f7_can_setup(void)
{
#if defined(CONFIG_STM32F7_CAN1)
  struct can_dev_s *can;
  int ret;

  /* Call stm32f7can_initialize() to get an instance of the CAN interface */

  can = stm32_caninitialize(CAN_PORT);
  if (can == NULL)
    {
      canerr("ERROR: Failed to get CAN interface\n");
      return -ENODEV;
    }

  /* Register the CAN driver at "/dev/can0" */

  ret = can_register("/dev/can0", can);
  if (ret < 0)
    {
      canerr("ERROR: can_register failed: %d\n", ret);
      return ret;
    }
  return OK;
#endif

#if defined(CONFIG_STM32F7_CAN2)
  struct can_dev_s *can;
  int ret;

  /* Call stm32f7can_initialize() to get an instance of the CAN interface */

  can = stm32_caninitialize(CAN_PORT);
  if (can == NULL)
    {
      canerr("ERROR: Failed to get CAN interface\n");
      return -ENODEV;
    }

  /* Register the CAN driver at "/dev/can1" */

  ret = can_register("/dev/can1", can);
  if (ret < 0)
    {
      canerr("ERROR: can_register failed: %d\n", ret);
      return ret;
    }
  return OK; 
#else
  return -ENODEV;
#endif
}

#endif /* CONFIG_CAN */
