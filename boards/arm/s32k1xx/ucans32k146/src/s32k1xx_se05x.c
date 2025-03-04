/****************************************************************************
 * boards/arm/s32k1xx/ucans32k146/src/s32k1xx_se05x.c
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

/* Copyright 2023 NXP */

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/compiler.h>
#include <nuttx/config.h>

#include "ucans32k146.h"
#include <errno.h>
#include <nuttx/crypto/se05x.h>
#include <s32k1xx_lpi2c.h>
#include <s32k1xx_pin.h>

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

static bool board_se05x_enable(bool state);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct se05x_config_s se05x_config =
{
    .address = 0x48,
    .frequency = 400000,
    .set_enable_pin = board_se05x_enable,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool board_se05x_enable(bool state)
{
  s32k1xx_gpiowrite(GPIO_SE050_EN, state);
  return true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int weak_function s32k1xx_se05x_initialize()
{
  int ret;

  struct i2c_master_s *lpi2c0 = s32k1xx_i2cbus_initialize(0);
  ret = lpi2c0 == NULL ? -EPERM : 0;

  if (ret == 0)
    {
      ret = se05x_register("/dev/se05x", lpi2c0, &se05x_config);
    }

  return ret;
}
