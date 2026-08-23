/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-touch-lcd7/src/esp32s3_board_ioexpander.c
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

#include <errno.h>
#include <syslog.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/ioexpander/ch422g.h>

#include "esp32s3_i2c.h"
#include "esp32s3-touch-lcd7.h"

#ifdef CONFIG_IOEXPANDER_CH422G

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct ch422g_config_s g_ch422g_config =
{
  .frequency = 400000,
};

/* The expander is shared, so it is brought up once and handed out. */

static FAR struct ioexpander_dev_s *g_ioe;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_ioexpander_initialize
 *
 * Description:
 *   Bring up the CH422G I/O expander, if it has not been brought up
 *   already, and return it.  Several parts of the board hang off the
 *   expander, so the first caller initialises it and the rest share it.
 *
 * Returned Value:
 *   The expander instance on success, NULL on failure.
 *
 ****************************************************************************/

FAR struct ioexpander_dev_s *board_ioexpander_initialize(void)
{
  FAR struct i2c_master_s *i2c;

  if (g_ioe != NULL)
    {
      return g_ioe;
    }

  i2c = esp32s3_i2cbus_initialize(BOARD_I2C_BUS);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize I2C%d\n", BOARD_I2C_BUS);
      return NULL;
    }

  g_ioe = ch422g_initialize(i2c, &g_ch422g_config);
  if (g_ioe == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize CH422G\n");
      esp32s3_i2cbus_uninitialize(i2c);
      return NULL;
    }

  return g_ioe;
}

#endif /* CONFIG_IOEXPANDER_CH422G */
