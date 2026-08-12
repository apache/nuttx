/****************************************************************************
 * boards/arm/rtl8720f/rtl8720f_evb/src/rtl8720f_i2c.c
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
#include "rtl8720f_rtl8720f_evb.h"

#ifdef CONFIG_AMEBA_I2C

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* One entry per I2C bus exposed to NuttX at /dev/i2cN.  The SCL/SDA pads are
 * examples used by the `i2c` config (system/i2c i2ctool) -- any pad can be
 * routed to an I2C controller through the pin mux, so adjust them to match
 * your board's wiring.  I2C0 is routed to PA22/PA23 and I2C1 to PA24/PA25,
 * free general-purpose pads on this board.  RTL8720F drives all GPIO through
 * a single port A controller, so pads use the AMEBA_PA() encoding.  Note the
 * I2C bus is open-drain: fit external pull-ups on SCL/SDA (the on-chip
 * pull-ups are weak, and probe loading such as a logic-analyzer clip can
 * keep the line from rising).
 */

struct rtl8720f_i2c_s
{
  int     bus;                  /* Controller index (AMEBA_I2C0/AMEBA_I2C1) */
  uint8_t sclpin;               /* SCL pad (AMEBA_PA() encoding) */
  uint8_t sdapin;               /* SDA pad (AMEBA_PA() encoding) */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rtl8720f_i2c_s g_i2c_buses[] =
{
  {
    AMEBA_I2C0, AMEBA_PA(22), AMEBA_PA(23)
  },
  {
    AMEBA_I2C1, AMEBA_PA(24), AMEBA_PA(25)
  },
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rtl8720f_i2c_initialize
 *
 * Description:
 *   Register the board's I2C master buses at /dev/i2cN.
 *
 ****************************************************************************/

int rtl8720f_i2c_initialize(void)
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
