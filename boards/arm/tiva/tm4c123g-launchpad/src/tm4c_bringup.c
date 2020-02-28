/****************************************************************************
 * boards/arm/tiva/tm4c123g-launchpad/src/tm4c_bringup.c
 *
 *   Copyright (C) 2014, 2016 Gregory Nutt. All rights reserved.
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

#include <syslog.h>

#include "tm4c123g-launchpad.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tm4c_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int tm4c_bringup(void)
{
  int ret = OK;

#ifdef CONFIG_TIVA_ADC
  /* Initialize ADC and register the ADC driver. */

  ret = tm4c_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: tm4c_adc_setup failed: %d\n", ret);
    }
#endif

#ifdef HAVE_AT24
  /* Initialize the AT24 driver */

  ret = tm4c_at24_automount(AT24_MINOR);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: tm4c_at24_automount failed: %d\n", ret);
      return ret;
    }
#endif /* HAVE_AT24 */

#ifdef CONFIG_TIVA_TIMER
  /* Initialize the timer driver */

  ret = tiva_timer_configure();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: tiva_timer_configure failed: %d\n", ret);
      return ret;
    }
#endif /* CONFIG_TIVA_TIMER */

#ifdef CONFIG_CAN_MCP2515
  /* Configure and initialize the MCP2515 CAN device */

  ret = tiva_mcp2515initialize("/dev/can0");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_mcp2515initialize() failed: %d\n", ret);
      return ret;
    }
#endif

  UNUSED(ret);
  return ret;
}
