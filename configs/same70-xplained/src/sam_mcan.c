/************************************************************************************
 * configs/same70-xplainedk/src/sam_mcan.c
 *
 *   Copyright (C) 2015-2016 Gregory Nutt. All rights reserved.
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

#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/can/can.h>
#include <arch/board/board.h>

#include "sam_mcan.h"
#include "same70-xplained.h"

#ifdef CONFIG_SAMV7_MCAN

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/

#if defined(CONFIG_SAMV7_MCAN0) && defined(CONFIG_SAMV7_MCAN1)
#  warning "Both MCAN0 and MCAN1 are enabled.  Assuming only CAN1."
#  undef CONFIG_SAMV7_MCAN0
#endif

#ifdef CONFIG_SAMV7_MCAN0
#  define CAN_PORT 0
#else
#  define CAN_PORT 1
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: sam_can_setup
 *
 * Description:
 *  Initialize CAN and register the CAN device
 *
 ************************************************************************************/

int sam_can_setup(void)
{
#if defined(CONFIG_SAMV7_MCAN0) || defined(CONFIG_SAMV7_MCAN1)
  struct can_dev_s *can;
  int ret;

  /* Call stm32_caninitialize() to get an instance of the CAN interface */

  can = sam_mcan_initialize(CAN_PORT);
  if (can == NULL)
    {
      canerr("ERROR:  Failed to get CAN interface\n");
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
#else
  return -ENODEV;
#endif
}

#endif /* CONFIG_SAMV7_MCAN */
