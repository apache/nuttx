/************************************************************************************
 * boards/zkit-arm-1769/src/lpc17_40_can.c
 *
 *   Copyright (C) 2013 Zilogic Systems. All rights reserved.
 *   Author: Raashid Muhammed <code@zilogic.com>
 *
 *   Based on boards/olimex-lpc1766stk/src/lpc17_40_can.c
 *
 *   Copyright (C) 2012, 2016, 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/can/can.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_arch.h"

#include "lpc17_40_can.h"
#include "lx_cpu.h"

#ifdef CONFIG_CAN

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Configuration ********************************************************************/

#define CAN_PORT1 1
#define CAN_PORT2 2

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: lx_cpu_can_setup
 *
 * Description:
 *  Initialize CAN and register the CAN device
 *
 ************************************************************************************/

int lx_cpu_can_setup(void)
{
#if defined(CONFIG_LPC17_40_CAN1) || defined(CONFIG_LPC17_40_CAN2)
  struct can_dev_s *can;
  int ret;

#ifdef CONFIG_LPC17_40_CAN1
  /* Call lpc17_40_caninitialize() to get an instance of the CAN1 interface */

  can = lpc17_40_caninitialize(CAN_PORT1);
  if (can == NULL)
    {
      canerr("ERROR:  Failed to get CAN1 interface\n");
      return -ENODEV;
    }

  /* Register the CAN1 driver at "/dev/can0" */

  ret = can_register("/dev/can0", can);
  if (ret < 0)
    {
      canerr("ERROR: CAN1 register failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_LPC17_40_CAN2
  /* Call lpc17_40_caninitialize() to get an instance of the CAN2 interface */

  can = lpc17_40_caninitialize(CAN_PORT2);
  if (can == NULL)
    {
      canerr("ERROR:  Failed to get CAN2 interface\n");
      return -ENODEV;
    }

  /* Register the CAN2 driver at "/dev/can1" */

#ifndef CONFIG_LPC17_40_CAN1
  ret = can_register("/dev/can0", can);
#else
  ret = can_register("/dev/can1", can);
#endif
  if (ret < 0)
    {
      canerr("ERROR: CAN2 register failed: %d\n", ret);
      return ret;
    }
#endif

  return OK;
#else
  return -ENODEV;
#endif
}

#endif /* CONFIG_CAN */
