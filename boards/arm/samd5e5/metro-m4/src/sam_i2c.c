/****************************************************************************
 * boards/arm/samd5e5/metro-m4/src/sam_i2c.c
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

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/i2c/i2c_master.h>

#include "arm_internal.h"
#include "chip.h"
#include "metro-m4.h"
#include "sam_config.h"
#include "sam_i2c_master.h"
#include <arch/board/board.h>

#if defined(SAMD5E5_HAVE_I2C5_MASTER)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: metro_m4_i2cdev_initialize
 *
 * Description:
 *   Called to configure I2C
 *
 ****************************************************************************/

int metro_m4_i2cdev_initialize(void)
{
    int ret = OK;

  #ifdef SAMD5E5_HAVE_I2C5_MASTER
      g_i2c5_dev = sam_i2c_master_initialize(5);

      if (!g_i2c5_dev)
        {
          syslog(LOG_ERR, "ERROR: metro_m4_i2cdev_initialize() failed: %d\n",
                ret);
          ret = -ENODEV;
        }
      else
        {
          #ifdef CONFIG_I2C_DRIVER
                ret = i2c_register(g_i2c5_dev, 5);
          #endif
        }
  #endif

  return ret;
}

#endif /* SAMD5E5_HAVE_I2C5_MASTER */
