/****************************************************************************
 * boards/arm/stm32/photon/src/stm32_rgbled.c
 *
 *   Copyright (C) 2018 Verge Inc. All rights reserved.
 *   Author: Anthony Merlino <anthony@vergeaero.com>
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

#include <errno.h>
#include <debug.h>
#include <fcntl.h>
#include <stdio.h>

#include <nuttx/fs/fs.h>
#include <nuttx/timers/pwm.h>
#include <nuttx/leds/rgbled.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_arch.h"
#include "stm32_pwm.h"
#include "photon.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ********************************************************************/

#define HAVE_RGBLED 1

#ifndef CONFIG_PWM
#  undef HAVE_RGBLED
#endif

#ifndef CONFIG_STM32_TIM2
#  undef HAVE_RGBLED
#endif

#ifndef CONFIG_PWM_MULTICHAN
#  undef HAVE_RGBLED
#endif

#ifndef CONFIG_STM32_TIM2_PWM
#  undef HAVE_RGBLED
#endif

#ifndef CONFIG_STM32_TIM2_CHANNEL2
#  undef HAVE_PWM
#endif

#ifndef CONFIG_STM32_TIM2_CHANNEL3
#  undef HAVE_PWM
#endif

#ifndef CONFIG_STM32_TIM2_CHANNEL4
#  undef HAVE_PWM
#endif

#ifdef HAVE_RGBLED

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_rgbled_setup
 *
 * Description:
 *   Initial for support of a connected RGB LED using PWM.
 *
 ****************************************************************************/

int stm32_rgbled_setup(void)
{
  static bool initialized = false;
  struct pwm_lowerhalf_s    *ledr;
  struct pwm_lowerhalf_s    *ledg;
  struct pwm_lowerhalf_s    *ledb;
  int ret;
  int fd;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* Call stm32_pwminitialize() to get an instance of the PWM interface */

      ledr = stm32_pwminitialize(RGBLED_RPWMTIMER);
      if (!ledr)
        {
          lederr("ERROR: Failed to get the STM32 PWM lower half to LEDR\n");
          return -ENODEV;
        }

      ledr->ops->setup(ledr);

      /* Call stm32_pwminitialize() to get an instance of the PWM interface */

      ledg = stm32_pwminitialize(RGBLED_GPWMTIMER);
      if (!ledg)
        {
          lederr("ERROR: Failed to get the STM32 PWM lower half to LEDG\n");
          return -ENODEV;
        }

      ledg->ops->setup(ledg);

      /* Call stm32_pwminitialize() to get an instance of the PWM interface */

      ledb = stm32_pwminitialize(RGBLED_BPWMTIMER);
      if (!ledb)
        {
          lederr("ERROR: Failed to get the STM32 PWM lower half to LEDB\n");
          return -ENODEV;
        }

      ledb->ops->setup(ledb);

      /* Register the RGB LED diver at "/dev/rgbled0" */

#ifdef CONFIG_PWM_MULTICHAN
      ret = rgbled_register("/dev/rgbled0", ledr, ledg, ledb,
                            RGBLED_RPWMCHANNEL, RGBLED_GPWMCHANNEL,
                            RGBLED_BPWMCHANNEL);
#else
      ret = rgbled_register("/dev/rgbled0", ledr, ledg, ledb);
#endif
      if (ret < 0)
        {
          lederr("ERROR: rgbled_register failed: %d\n", ret);
          return ret;
        }

      fd = nx_open("/dev/rgbled0", O_WRONLY);
      if (fd < 0)
        {
          lederr("ERROR: open failed: %d\n", fd);
          return ret;
        }

      /* Initialize led off */

      nx_write(fd, "#000000", 8);
      close(fd);

      /* Now we are initialized */

      initialized = true;
    }

  return OK;
}

#else
#  error "HAVE_RGBLED is undefined"
#endif /* HAVE_RGBLED */
