/****************************************************************************
 * arch/arm/src/am335x/am335x_sysclk.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <errno.h>

#include "arm_arch.h"
#include "hardware/am335x_scm.h"
#include "am335x_sysclk.h"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am335x_get_sysclk
 *
 * Description:
 *   Return the sysclk frequency
 *
 ****************************************************************************/

int32_t am335x_get_sysclk(void)
{
  uint32_t regval;

  /* Read the input clock freq from the control module. */

  regval = getreg32(AM335X_SCM_CTRL_STATUS);

  /* Return the frequency of the configured crystal */

  switch (regval & SCM_CTRL_STATUS_SYSBOOT1_MASK)
    {
      case SCM_CTRL_STATUS_SYSBOOT1_19p2MHZ: /* 19.2Mhz */
        return 19200000;

      case SCM_CTRL_STATUS_SYSBOOT1_24MHZ: /* 24Mhz */
        return 24000000;

      case SCM_CTRL_STATUS_SYSBOOT1_25MHZ: /* 25Mhz */
        return 25000000;

      case SCM_CTRL_STATUS_SYSBOOT1_26MHZ: /* 26Mhz */
        return 26000000;
    }

  return -EINVAL;  /* Should never happen */
}
