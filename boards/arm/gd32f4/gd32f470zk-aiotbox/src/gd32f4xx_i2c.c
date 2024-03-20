/****************************************************************************
 * boards/arm/gd32f4/gd32f470zk-aiotbox/src/gd32f4xx_i2c.c
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
#include<nuttx/config.h>
#include<stdbool.h>
#include<stdio.h>
#include<errno.h>
#include<debug.h>

#include<nuttx/i2c/i2c_master.h>

#include"gd32f4xx.h"
#include "gd32f470z_aiotbox.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_I2C
void gd32_i2c_initialize(void)
{
  FAR struct i2c_master_s *i2c;
  int ret;
  i2cinfo("Initialize I2c\n");

#ifdef CONFIG_GD32F4_I2C0
  i2c = gd32_i2cbus_initialize(0);

  if (i2c == NULL)
    {
      i2cerr("init i2c0 faild.\n");
      return;
    }
  else
    {
      ret = i2c_register(i2c, 0);

      if (ret < 0)
        {
          i2cerr("registering i2c0 faild.\n");
        }
      else
        {
          i2cinfo("registering i2c0 successed.\n");
        }
    }

  i2c = NULL;
#endif
#ifdef CONFIG_GD32F4_I2C1
  i2c = gd32_i2cbus_initialize(1);

  if (i2c == NULL)
    {
      i2cerr("init i2c1 faild.\n");
      return;
    }
  else
    {
      ret = i2c_register(i2c, 1);

      if (ret < 0)
        {
          i2cerr("registering i2c1 faild.\n");
        }
      else
        {
          i2cinfo("registering i2c1 successed.\n");
        }
    }

#endif
}

#endif