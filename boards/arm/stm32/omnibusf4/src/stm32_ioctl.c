/****************************************************************************
 * boards/arm/stm32/omnibusf4/src/stm32_appinit.c
 *
 *   Copyright (C) 2019 Bill Gatliff. All rights reserved.
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Bill Gatliff <bgat@billgatliff.com>
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

#include <sys/types.h>
#include <stdint.h>
#include <errno.h>

#include <nuttx/board.h>

#include "omnibusf4.h"

#ifdef CONFIG_BOARDCTL_IOCTL

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_ioctl
 *
 * Description:
 *   The "landing site" for much of the boardctl() interface. Generic board-
 *   control functions invoked via ioctl() get routed through here.
 *
 *   Since we don't do anything unusual at the moment, this function
 *   accomplishes nothing except avoid a missing-function linker error if
 *   CONFIG_BOARDCTL_IOCTL is selected.
 *
 * Input Parameters:
 *   cmd - IOCTL command being requested.
 *   arg - Arguments for the IOCTL.
 *
 * Returned Value:
 *   we don't yet support any boardctl IOCTLs.  This function always returns
 *  -ENOTTY which is the standard IOCTL return value when a command is not
 *  supported
 *
 ****************************************************************************/

int board_ioctl(unsigned int cmd, uintptr_t arg)
{
  switch (cmd)
    {
      default:
        return -ENOTTY;
    }

  return OK;
}

#endif /* CONFIG_BOARDCTL_IOCTL */
