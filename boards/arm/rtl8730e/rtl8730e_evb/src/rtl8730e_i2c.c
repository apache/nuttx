/****************************************************************************
 * boards/arm/rtl8730e/rtl8730e_evb/src/rtl8730e_i2c.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <sys/param.h>
#include <syslog.h>

#include "ameba_gpio.h"
#include "ameba_i2c.h"
#include "rtl8730e_evb.h"

#ifdef CONFIG_AMEBA_I2C

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* One entry per I2C bus exposed to NuttX at /dev/i2cN.  The SCL/SDA pads are
 * examples used by the `i2c` config (system/i2c i2ctool) -- adjust them to
 * match your board's wiring.  Unlike the GPIO or UART crossbars, an RTL8730E
 * pad reaches exactly one I2C controller, so a pad can only be swapped for
 * another from the same controller's set; see i2c_index_get() in the SDK's
 * hal/src/i2c_api.c.  Pads the audio codec drives must not be used.
 */

struct rtl8730e_i2c_s
{
  int     bus;                  /* Controller index (AMEBA_I2C0..AMEBA_I2C2) */
  uint8_t sclpin;               /* SCL pad (AMEBA_PA()/AMEBA_PB() encoding) */
  uint8_t sdapin;               /* SDA pad (AMEBA_PA()/AMEBA_PB() encoding) */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rtl8730e_i2c_s g_i2c_buses[] =
{
  {
    AMEBA_I2C0, AMEBA_PA(10), AMEBA_PA(9)
  },
  {
    AMEBA_I2C1, AMEBA_PA(4), AMEBA_PA(3)
  },
  {
    AMEBA_I2C2, AMEBA_PB(11), AMEBA_PB(10)
  },
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rtl8730e_i2c_initialize
 *
 * Description:
 *   Register the board's I2C master buses at /dev/i2cN.
 *
 ****************************************************************************/

int rtl8730e_i2c_initialize(void)
{
  int ret;
  int i;

  for (i = 0; i < (int)nitems(g_i2c_buses); i++)
    {
      ret = ameba_i2c_register(g_i2c_buses[i].bus, g_i2c_buses[i].sclpin,
                               g_i2c_buses[i].sdapin);
      if (ret < 0)
        {
          syslog(LOG_ERR,
                 "ERROR: ameba_i2c_register(/dev/i2c%d) failed: %d\n",
                 g_i2c_buses[i].bus, ret);
          return ret;
        }
    }

  return OK;
}

#endif /* CONFIG_AMEBA_I2C */
