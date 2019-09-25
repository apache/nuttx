/****************************************************************************
 * boards/arm/stm32/stm32f4discovery/src/stm32_ds1307.c
 *
 *   Copyright (C) 2019 Alan Carvalho de Assis. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
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

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/timers/rtc.h>
#include <nuttx/timers/ds3231.h>

#include "stm32.h"
#include "stm32_i2c.h"
#include "stm32f4discovery.h"

#if defined(CONFIG_I2C) && defined(CONFIG_RTC_DS1307)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DS1307_I2C_ADDR    0x6f /* DS1307 I2C Address */
#define DS1307_I2C_BUS     1    /* DS1307 is on I2C1 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_ds1307_init
 *
 * Description:
 *   Initialize and configure the DS1307 RTC
 *
 ****************************************************************************/

int stm32_ds1307_init(void)
{
  FAR struct i2c_master_s *i2c;
  static bool initialized = false;
  int ret;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* No.. Get the I2C bus driver */

      rtcinfo("Initialize I2C%d\n", DS1307_I2C_BUS);
      i2c = stm32_i2cbus_initialize(DS1307_I2C_BUS);
      if (!i2c)
        {
          rtcerr("ERROR: Failed to initialize I2C%d\n", DS1307_I2C_BUS);
          return -ENODEV;
        }

      /* Now bind the I2C interface to the DS1307 RTC driver */

      rtcinfo("Bind the DS1307 RTC driver to I2C%d\n", DS1307_I2C_BUS);
      ret = dsxxxx_rtc_initialize(i2c);
      if (ret < 0)
        {
          rtcerr("ERROR: Failed to bind I2C%d to the DS1307 RTC driver\n",
                 DS1307_I2C_BUS);
          return -ENODEV;
        }

#ifdef CONFIG_I2C_DRIVER
      /* Register the I2C to get the "nsh> i2c bus" command working */

      ret = i2c_register(i2c, DS1307_I2C_BUS);
      if (ret < 0)
        {
          rtcerr("ERROR: Failed to register I2C%d driver: %d\n", bus, ret);
          return -ENODEV;
        }
#endif

      /* Synchronize the system time to the RTC time */

      clock_synchronize();

      /* Now we are initialized */

      initialized = true;
    }

  return OK;
}

#endif /* CONFIG_I2C && CONFIG_RTC_DS1307 */
