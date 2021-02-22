/****************************************************************************
 * boards/arm/tms570/tms570ls31x-usb-kit/src/tms570_bringup.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

#include <stdbool.h>
#include <stdio.h>
#include <syslog.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>
#include "tms570ls31x_usb_kit.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Checking needed by MMC/SDCard */

#ifdef CONFIG_NSH_MMCSDMINOR
#  define MMCSD_MINOR CONFIG_NSH_MMCSDMINOR
#else
#  define MMCSD_MINOR 0
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tms570_bringup
 *
 * Description:
 *   Bring up simulated board features
 *
 ****************************************************************************/

int tms570_bringup(void)
{
  int ret = OK;

#ifdef CONFIG_MMCSD
  tms570_spidev_initialize();
  ret = tms570_mmcsd_initialize(MMCSD_MINOR);
  if (ret < 0)
    {
      serr("Failed to initialize SD slot %d: %d\n", ret);
      return ret;
    }
#endif

  UNUSED(ret);
  return OK;
}
