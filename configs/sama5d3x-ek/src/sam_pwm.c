/************************************************************************************
 * configs/sama5d3x-ek/src/sam_pwm.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/pwm.h>
#include <arch/board/board.h>

#include "sam_pwm.h"
#include "sama5d3x-ek.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/
/* PWM.  There are no dedicated PWM output pins available to the user for PWM
 * testing.  Care must be taken because all PWM output pins conflict with some other
 * usage of the pin by other devices:
 *
 *    -----+---+---+----+--------------------
 *     PWM  PIN PER PIO  CONFLICTS
 *    -----+---+---+----+--------------------
 *     PWM0 FI   B  PC28   SPI1, ISI
 *          H    B  PB0    GMAC
 *               B  PA20   LCDC, ISI
 *          L    B  PB1    GMAC
 *               B  PA21   LCDC, ISI
 *    -----+---+---+----+--------------------
 *     PWM1 FI   B  PC31   HDMI
 *          H    B  PB4    GMAC
 *               B  PA22   LCDC, ISI
 *          L    B  PB5    GMAC
 *               B  PE31   ISI, HDMI
 *               B  PA23   LCDC, ISI
 *    -----+---+---+----+--------------------
 *     PWM2 FI   B  PC29   UART0, ISI, HDMI
 *          H    C  PD5    HSMCI0
 *               B  PB8    GMAC
 *          L    C  PD6    HSMCI0
 *               B  PB9    GMAC
 *    -----+---+---+----+--------------------
 *     PWM3 FI   C  PD16   SPI0, Audio
 *          H    C  PD7    HSMCI0
 *               B  PB12   GMAC
 *          L    C  PD8    HSMCI0
 *               B  PB13   GMAC
 *    -----+---+---+----+--------------------
 */

#ifndef CONFIG_SAMA5D3xEK_CHANNEL
#  if defined(CONFIG_SAMA5_PWM_CHAN0)
#    warning Assuming PWM channel 0
#    define CONFIG_SAMA5D3xEK_CHANNEL 0
#  elif defined(CONFIG_SAMA5_PWM_CHAN1)
#    warning Assuming PWM channel 1
#    define CONFIG_SAMA5D3xEK_CHANNEL 1
#  elif defined(CONFIG_SAMA5_PWM_CHAN2)
#    warning Assuming PWM channel 2
#    define CONFIG_SAMA5D3xEK_CHANNEL 2
#  elif defined(CONFIG_SAMA5_PWM_CHAN3)
#    warning Assuming PWM channel 3
#    define CONFIG_SAMA5D3xEK_CHANNEL 3
#  endif
#endif

#if defined(CONFIG_PWM) && defined(CONFIG_SAMA5_PWM)

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: pwm_devinit
 *
 * Description:
 *   All SAMA5 architectures must provide the following interface to work with
 *   examples/pwm.
 *
 ************************************************************************************/

int pwm_devinit(void)
{
  static bool initialized = false;
  struct pwm_lowerhalf_s *pwm;
  int ret;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* Call sam_pwminitialize() to get an instance of the PWM interface */

      pwm = sam_pwminitialize(CONFIG_SAMA5D3xEK_CHANNEL);
      if (!pwm)
        {
          dbg("Failed to get the SAMA5 PWM lower half\n");
          return -ENODEV;
        }

      /* Register the PWM driver at "/dev/pwm0" */

      ret = pwm_register("/dev/pwm0", pwm);
      if (ret < 0)
        {
          adbg("pwm_register failed: %d\n", ret);
          return ret;
        }

      /* Now we are initialized */

      initialized = true;
    }

  return OK;
}

#endif /* CONFIG_PWM */
