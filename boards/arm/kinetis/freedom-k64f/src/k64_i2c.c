/****************************************************************************
 * boards/arm/kinetis/freedom-k64f/src/k64_i2c.c
 *
 *   Copyright (C) 2020 Philippe Coval. All rights reserved.
 *   Author:  Philippe Coval <rzr@users.sf.net>
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

#include "kinetis_i2c.h"

#if defined(CONFIG_KINETIS_I2C0)

# if defined(CONFIG_SENSORS_FXOS8700CQ)
#  include "nuttx/sensors/fxos8700cq.h"
# endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_KINETIS_I2C0
FAR struct i2c_master_s * g_i2c0_dev;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: k64_i2cdev_initialize
 *
 * Description:
 *   Called to configure I2C
 *
 ****************************************************************************/

int k64_i2cdev_initialize(void)
{
  int ret = OK;

#ifdef CONFIG_KINETIS_I2C0
  g_i2c0_dev = kinetis_i2cbus_initialize(0);
  if (g_i2c0_dev == NULL)
    {
      syslog(LOG_ERR, "ERROR: kinetis_i2cbus_initialize(0) failed: %d\n",
             ret);
      ret = -ENODEV;
    }
  else
    {
#ifdef CONFIG_I2C_DRIVER
      ret = i2c_register(g_i2c0_dev, 0);
#if defined(CONFIG_SENSORS_FXOS8700CQ)
      fxos8700cq_register("/dev/accel0", g_i2c0_dev);
#endif
#endif
    }
#endif

  return ret;
}

#endif /* CONFIG_KINETIS_I2C0 */
