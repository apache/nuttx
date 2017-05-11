/****************************************************************************
 * config/mbed/src/lpc17_appinit.c
 *
 *   Copyright (C) 2010, 2016 Gregory Nutt. All rights reserved.
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

#include <stdio.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/mmcsd.h>

#include "mbed.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* PORT and SLOT number probably depend on the board configuration */

#ifdef CONFIG_ARCH_BOARD_MBED
#  define NSH_HAVEUSBDEV 1
#else
#  error "Unrecognized board"
#  undef NSH_HAVEUSBDEV
#endif

/* Can't support USB features if USB is not enabled */

#ifndef CONFIG_USBDEV
#  undef NSH_HAVEUSBDEV
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initalization logic and the
 *         matching application logic.  The value cold be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int board_app_initialize(uintptr_t arg)
{
  int ret;

#ifdef CONFIG_PWM
  /* Initialize PWM and register the PWM device. */

  ret = mbed_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: mbed_pwm_setup() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_ADC
  /* Initialize ADC and register the ADC driver. */

  ret = mbed_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: mbed_adc_setup failed: %d\n", ret);
    }
#endif

  UNUSED(ret);
  return OK;
}
