/************************************************************************************
 * configs/zkit-arm-1769/src/lpc17_can.c
 *
 *   Copyright (C) 2013 Zilogic Systems. All rights reserved.
 *   Author: Raashid Muhammed <code@zilogic.com>
 *
 *   Based on configs/olimex-lpc1766stk/src/lpc17_can.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

#include <nuttx/can.h>
#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"

#include "lpc17_can.h"
#include "zkitarm_internal.h"

#if defined(CONFIG_CAN) && (defined(CONFIG_LPC17_CAN1) || defined(CONFIG_LPC17_CAN2))

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/
#define CAN_PORT1 1
#define CAN_PORT2 2


/* Debug ***************************************************************************/
/* Non-standard debug that may be enabled just for testing CAN */

#ifdef CONFIG_DEBUG_CAN
#  define candbg    dbg
#  define canvdbg   vdbg
#  define canlldbg  lldbg
#  define canllvdbg llvdbg
#else
#  define candbg(x...)
#  define canvdbg(x...)
#  define canlldbg(x...)
#  define canllvdbg(x...)
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: can_devinit
 *
 * Description:
 *   All LPC17 architectures must provide the following interface to work with
 *   examples/can.
 *
 ************************************************************************************/

int can_devinit(void)
{
  static bool initialized = false;
  struct can_dev_s *can;
  int ret;

  /* Check if we have already initialized */

  if (!initialized)
    {
#ifdef CONFIG_LPC17_CAN1
      /* Call lpc17_caninitialize() to get an instance of the CAN1 interface */

      can = lpc17_caninitialize(CAN_PORT1);
      if (can == NULL)
        {
          candbg("ERROR:  Failed to get CAN1 interface\n");
          return -ENODEV;
        }

      /* Register the CAN1 driver at "/dev/can0" */

      ret = can_register("/dev/can0", can);
      if (ret < 0)
        {
          candbg("ERROR: CAN1 register failed: %d\n", ret);
          return ret;
        }
#endif

#ifdef CONFIG_LPC17_CAN2
      /* Call lpc17_caninitialize() to get an instance of the CAN2 interface */

      can = lpc17_caninitialize(CAN_PORT2);
      if (can == NULL)
        {
          candbg("ERROR:  Failed to get CAN2 interface\n");
          return -ENODEV;
        }

      /* Register the CAN2 driver at "/dev/can1" */

      ret = can_register("/dev/can1", can);
      if (ret < 0)
        {
          candbg("ERROR: CAN2 register failed: %d\n", ret);
          return ret;
        }
#endif

      /* Now we are initialized */

      initialized = true;
    }

  return OK;
}

#endif /* CONFIG_CAN && (CONFIG_LPC17_CAN1 || CONFIG_LPC17_CAN2) */
