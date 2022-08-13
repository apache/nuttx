/****************************************************************************
 * boards/arm/lpc54xx/lpcxpresso-lpc54628/src/lpc54_i2ctool.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
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
  struct i2c_master_s *i2c;
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
