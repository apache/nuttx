/****************************************************************************
 * configs/lpcxpresso-lpc54628/src/lpc54_i2ctool.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#include <nuttx/i2c/i2c_master.h>

#include "lpc54_config.h"
#include "lpc54_i2c_master.h"
#include "lpcxpresso-lpc54628.h"

#ifdef HAVE_I2CTOOL

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_i2c_register
 *
 * Description:
 *   Register one I2C drivers for the I2C tool.
 *
 ****************************************************************************/

static void lpc54_i2c_register(int bus, int ndx)
{
  FAR struct i2c_master_s *i2c;
  int ret;

  i2c = lpc54_i2c_handle(bus, ndx);
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
          lpc54_i2cbus_uninitialize(i2c);
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_i2ctool
 *
 * Description:
 *   Register I2C drivers for the I2C tool.
 *
 ****************************************************************************/

void lpc54_i2ctool(void)
{
#ifdef CONFIG_LPC54_I2C0_MASTER
  lpc54_i2c_register(0, I2C0NDX);
#endif
#ifdef CONFIG_LPC54_I2C1_MASTER
  lpc54_i2c_register(1, I2C1NDX);
#endif
#ifdef CONFIG_LPC54_I2C2_MASTER
  lpc54_i2c_register(2, I2C2NDX);
#endif
#ifdef CONFIG_LPC54_I2C3_MASTER
  lpc54_i2c_register(3, I2C3NDX);
#endif
#ifdef CONFIG_LPC54_I2C4_MASTER
  lpc54_i2c_register(4, I2C4NDX);
#endif
#ifdef CONFIG_LPC54_I2C5_MASTER
  lpc54_i2c_register(5, I2C5NDX);
#endif
#ifdef CONFIG_LPC54_I2C6_MASTER
  lpc54_i2c_register(6, I2C6NDX);
#endif
#ifdef CONFIG_LPC54_I2C7_MASTER
  lpc54_i2c_register(7, I2C7NDX);
#endif
#ifdef CONFIG_LPC54_I2C8_MASTER
  lpc54_i2c_register(8, I2C8NDX);
#endif
#ifdef CONFIG_LPC54_I2C9_MASTER
  lpc54_i2c_register(9, I2C9NDX);
#endif
}

#endif /* HAVE_I2CTOOL */
