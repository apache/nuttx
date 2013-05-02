/************************************************************************************
 * configs/zkit-arm-1769/src/up_dac.c
 * arch/arm/src/board/up_dac.c
 *
 *   Copyright (C) 2013 Zilogic Systems. All rights reserved.
 *   Author: Kannan <code@nuttx.org>
 *
 * Based on configs/stm3220g-eval/src/up_dac.c
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

#include <nuttx/analog/dac.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"

#include "zkitarm_internal.h"
#include "lpc17_dac.h"

#ifdef CONFIG_DAC

/************************************************************************************
 * Name: dac_devinit
 *
 * Description:
 *   All LPC17xx architectures must provide the following interface to work with
 *   examples/diag.
 *
 ************************************************************************************/

int dac_devinit(void)
{
  static bool initialized = false;
  struct dac_dev_s *dac;
  int ret;

  if (!initialized)
  {
    /* Call lpc17_dacinitialize() to get an instance of the dac interface */

    dac = lpc17_dacinitialize();
    if (dac == NULL)
      {
        adbg("ERROR: Failed to get dac interface\n");
        return -ENODEV;
      }

    ret = dac_register("/dev/dac0", dac);
    if (ret < 0)
      {
        adbg("dac_register failed: %d\n", ret);
        return ret;
      }

    initialized = true;
  }

  return OK;
}

#endif /* CONFIG_DAC */
