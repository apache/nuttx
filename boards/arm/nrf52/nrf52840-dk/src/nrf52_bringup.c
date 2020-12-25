/****************************************************************************
 * boards/arm/nrf52/nrf52840-dk/src/nrf52_bringup.c
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

#include <sys/types.h>
#include <syslog.h>

#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

#include "nrf52840-dk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NRF52_TIMER (0)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_i2c_register
 *
 * Description:
 *   Register one I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
static void nrf52_i2c_register(int bus)
{
  struct i2c_master_s *i2c;
  int ret;

  i2c = nrf52_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get I2C%d interface\n", bus);
    }
  else
    {
      ret = i2c_register(i2c, bus);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register I2C%d driver: %d\n",
                 bus, ret);
          nrf52_i2cbus_uninitialize(i2c);
        }
    }
}
#endif

/****************************************************************************
 * Name: nrf52_i2ctool
 *
 * Description:
 *   Register I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
static void nrf52_i2ctool(void)
{
#ifdef CONFIG_NRF52_I2C0
  nrf52_i2c_register(0);
#endif
#ifdef CONFIG_NRF52_I2C1
  nrf52_i2c_register(1);
#endif
}
#endif

/****************************************************************************
 * Name: nrf52_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int nrf52_bringup(void)
{
  int ret;

#ifdef CONFIG_USERLED
  /* Register the LED driver */

  ret = userled_lower_initialize(CONFIG_EXAMPLES_LEDS_DEVPATH);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: userled_lower_initialize() failed: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_SENSORS_LSM6DSL
  ret = nrf52_lsm6dsl_initialize("/dev/lsm6dsl0");
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize LSM6DSL driver: %d\n",
             ret);
    }
#endif /* CONFIG_SENSORS_LSM6DSL */

#ifdef CONFIG_SENSORS_LSM303AGR
  ret = nrf52_lsm303agr_initialize("/dev/lsm303agr0");
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize LSM303AGR driver: %d\n",
             ret);
    }
#endif /* CONFIG_SENSORS_LSM303AGR */

#ifdef CONFIG_SENSORS_HTS221
  ret = nrf52_hts221_initialize("/dev/hts2210");
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize HTS221 driver: %d\n",
             ret);
    }
#endif /* CONFIG_SENSORS_HTS221 */

#ifdef CONFIG_LPWAN_SX127X
  ret = nrf52_lpwaninitialize();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize wireless driver: %d\n",
             ret);
    }
#endif /* CONFIG_LPWAN_SX127X */

#if defined(CONFIG_TIMER) && defined(CONFIG_NRF52_TIMER)
  /* Configure TIMER driver */

  ret = nrf52_timer_driver_setup("/dev/timer0", NRF52_TIMER);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize timer driver: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_PWM
  /* Configure PWM driver */

  ret = nrf52_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize PWM driver: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_ADC
  /* Configure ADC driver */

  ret = nrf52_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize ADC driver: %d\n",
             ret);
    }
#endif

  UNUSED(ret);
  return OK;
}
