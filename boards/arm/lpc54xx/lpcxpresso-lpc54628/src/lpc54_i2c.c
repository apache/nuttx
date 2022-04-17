/****************************************************************************
 * boards/arm/lpc54xx/lpcxpresso-lpc54628/src/lpc54_i2c.c
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

#if defined(HAVE_I2CTOOL) || defined(HAVE_FT5x06)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct i2c_master_s *g_i2c_handle[NI2C];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_i2c_handle
 *
 * Description:
 *   Create (or reuse) an I2C handle
 *
 ****************************************************************************/

struct i2c_master_s *lpc54_i2c_handle(int bus, int ndx)
{
  struct i2c_master_s *i2c = g_i2c_handle[ndx];

  if (i2c == NULL)
    {
      i2c = lpc54_i2cbus_initialize(bus);
      if (i2c == NULL)
        {
          syslog(LOG_ERR, "ERROR: Failed to get I2C%d interface\n", bus);
        }
      else
        {
          g_i2c_handle[ndx] = i2c;
        }
    }

  return i2c;
}

/****************************************************************************
 * Name: lpc54_i2c_free
 *
 * Description:
 *   Free an I2C handle created by lpc54_i2c_handle
 *
 ****************************************************************************/

void lpc54_i2c_free(int ndx)
{
  if (g_i2c_handle[ndx] != NULL)
    {
      lpc54_i2cbus_uninitialize(g_i2c_handle[ndx]);
      g_i2c_handle[ndx] = NULL;
    }
}

#endif /* HAVE_I2CTOOL || HAVE_FT5x06*/
