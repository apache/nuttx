/****************************************************************************
 * config/tm4c123g-launchpad/src/tm4c_timer.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 *   With modifications from Calvin Maguranis
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

#include <debug.h>

#include "tiva_timer.h"
#include "tm4c123g-launchpad.h"

#ifdef CONFIG_TIVA_TIMER

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_TIVA_TIMER32_PERIODIC
#  error CONFIG_TIVA_TIMER32_PERIODIC is not defined
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_timer_initialize
 *
 * Description:
 *   Configure the timer driver for the timer example application.
 *
 ****************************************************************************/

int tiva_timer_initialize(void)
{
  static bool initialized = false;
  int ret = OK;

  /* Check if we have already initialized */

  if (!initialized)
    {
      struct tiva_gptm32config_s timer_config;
      timer_config.cmn.gptm      = 0;
      timer_config.cmn.mode      = TIMER32_MODE_PERIODIC;
      timer_config.cmn.alternate = false;

      timer_config.config.flags   = TIMER_FLAG_COUNTUP;
      timer_config.config.handler = 0;
      timer_config.config.arg     = 0;

      ret = tiva_timer_initialize(CONFIG_EXAMPLE_TIMER_DEVNAME, &timer_config);
      if (ret < 0)
      {
        syslog(LOG_ERR, "ERROR: Failed to register timer driver: %d\n", ret);
      }

      /* now we are initialized */

      initialized = true;
    }

  return ret;
}
#endif /* CONFIG_TIVA_TIMER */
