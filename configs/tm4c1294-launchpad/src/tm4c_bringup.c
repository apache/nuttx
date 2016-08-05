/****************************************************************************
 * config/tm4c1294-launchpad/src/tm4c_bringup.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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
#include <stdint.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/i2c/i2c_master.h>
#include <arch/board/board.h>

#include "tiva_i2c.h"
#include "tiva_pwm.h"
#include "tm4c1294-launchpad.h"

#define PWM_PATH_FMT        "/dev/pwm%d"
#define PWM_PATH_FMTLEN     (10)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DK_TM4C129X_TIMER
#  define HAVE_TIMER
#endif

#ifdef CONFIG_TM4C1294_LAUNCHPAD_PWM
#  define HAVE_PWM
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tm4c_i2c_register
 *
 * Description:
 *   Register one I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#ifdef HAVE_I2CTOOL
static void tm4c_i2c_register(int bus)
{
  FAR struct i2c_master_s *i2c;
  int ret;

  i2c = tiva_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      _err("ERROR: Failed to get I2C%d interface\n", bus);
    }
  else
    {
      ret = i2c_register(i2c, bus);
      if (ret < 0)
        {
          _err("ERROR: Failed to register I2C%d driver: %d\n", bus, ret);
          tiva_i2cbus_uninitialize(i2c);
        }
    }
}
#endif

/****************************************************************************
 * Name: tm4c_i2ctool
 *
 * Description:
 *   Register I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#ifdef HAVE_I2CTOOL
static void tm4c_i2ctool(void)
{
#ifdef CONFIG_TIVA_I2C0
  tm4c_i2c_register(0);
#endif
#ifdef CONFIG_TIVA_I2C1
  tm4c_i2c_register(1);
#endif
#ifdef CONFIG_TIVA_I2C2
  tm4c_i2c_register(2);
#endif
#ifdef CONFIG_TIVA_I2C3
  tm4c_i2c_register(3);
#endif
#ifdef CONFIG_TIVA_I2C4
  tm4c_i2c_register(4);
#endif
#ifdef CONFIG_TIVA_I2C5
  tm4c_i2c_register(5);
#endif
#ifdef CONFIG_TIVA_I2C6
  tm4c_i2c_register(6);
#endif
#ifdef CONFIG_TIVA_I2C7
  tm4c_i2c_register(7);
#endif
#ifdef CONFIG_TIVA_I2C8
  tm4c_i2c_register(8);
#endif
#ifdef CONFIG_TIVA_I2C9
  tm4c_i2c_register(9);
#endif
}
#else
#  define tm4c_i2ctool()
#endif

/****************************************************************************
* Name: tm4c_pwm_register
*
* Description:
*   Register a PWM dev file with the upper_level PWM driver.
*
* Input Parameters:
*   channel - A number identifying the PWM channel use.
*
* Returned Value:
*   None.
*
****************************************************************************/

void tm4c_pwm_register(int channel)
{
  FAR struct pwm_lowerhalf_s *dev;
  int ret;
  char pwm_path[PWM_PATH_FMTLEN];

  dev = tiva_pwm_initialize(channel);
  if (dev == NULL)
    {
      pwmerr("ERROR: Failed to get PWM%d interface\n", channel);
    }
  else
    {
      snprintf(pwm_path, PWM_PATH_FMTLEN, PWM_PATH_FMT, channel);
      ret = pwm_register(pwm_path, dev);
      if (ret < 0)
        {
          pwmerr("ERROR: Failed to register PWM%d driver: %d\n",
                 channel, ret);
        }
    }
}

/****************************************************************************
 * Name: tm4c_pwm
 *
 * Description:
 *   Register PWM drivers for the PWM tool.
 *
 ****************************************************************************/

#ifdef HAVE_PWM
static void tm4c_pwm(void)
{
#ifdef CONFIG_TIVA_PWM0_CHAN0
  tm4c_pwm_register(0);
#endif
#ifdef CONFIG_TIVA_PWM0_CHAN1
  tm4c_pwm_register(1);
#endif
#ifdef CONFIG_TIVA_PWM0_CHAN2
  tm4c_pwm_register(2);
#endif
#ifdef CONFIG_TIVA_PWM0_CHAN3
  tm4c_pwm_register(3);
#endif
#ifdef CONFIG_TIVA_PWM0_CHAN4
  tm4c_pwm_register(4);
#endif
#ifdef CONFIG_TIVA_PWM0_CHAN5
  tm4c_pwm_register(5);
#endif
#ifdef CONFIG_TIVA_PWM0_CHAN6
  tm4c_pwm_register(6);
#endif
#ifdef CONFIG_TIVA_PWM0_CHAN7
  tm4c_pwm_register(7);
#endif
}
#endif # HAVE_PWM

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
#ifdef HAVE_TIMER
  int ret;
#endif

  /* Register I2C drivers on behalf of the I2C tool */

  tm4c_i2ctool();

#ifdef HAVE_PWM
  /* Register PWM drivers */

  tm4c_pwm();
#endif

#ifdef HAVE_TIMER
  /* Initialize the timer driver */

  ret = tiva_timer_configure();
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize timer driver: %d\n", ret);
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: board_pwm_setup
 *
 * Description:
 *   No implementation for now, it's called by PWM tool via boardctl().
 *   See include/nuttx/board.h
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero on Success.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARDCTL_PWMTEST
int board_pwm_setup(void)
{
  return OK;
}
#endif

