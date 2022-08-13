/****************************************************************************
 * boards/arm/tiva/dk-tm4c129x/src/tm4c_bringup.c
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

#include <debug.h>

#include <nuttx/i2c/i2c_master.h>
#include <arch/board/board.h>

#include "tiva_i2c.h"
#include "dk-tm4c129x.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_I2C) && defined(CONFIG_LM75_I2C) && defined(CONFIG_TIVA_I2C6)
#  define HAVE_TMP100
#endif

#ifdef CONFIG_DK_TM4C129X_TIMER
#  define HAVE_TIMER
#endif

#ifdef CONFIG_SYSTEM_LM75_DEVNAME
#  define TMP100_DEVNAME CONFIG_SYSTEM_LM75_DEVNAME
#else
#  define TMP100_DEVNAME "/dev/temp"
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tm4c_i2c_register
 *
 * Description:
 *   Register one I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#ifdef HAVE_I2CTOOL
static void tm4c_i2c_register(int bus)
{
  struct i2c_master_s *i2c;
  int ret;

  i2c = tiva_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      _err("ERROR: Failed to get I2C%d interface\n", bus);
    }
  else
    {
      ret = i2c_register(i2c, bus);
      if (ret < 0)
        {
          _err("ERROR: Failed to register I2C%d driver: %d\n", bus, ret);
          tiva_i2cbus_uninitialize(i2c);
        }
    }
}
#endif

/****************************************************************************
 * Name: tm4c_i2ctool
 *
 * Description:
 *   Register I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#ifdef HAVE_I2CTOOL
static void tm4c_i2ctool(void)
{
#ifdef CONFIG_TIVA_I2C0
  tm4c_i2c_register(0);
#endif
#ifdef CONFIG_TIVA_I2C1
  tm4c_i2c_register(1);
#endif
#ifdef CONFIG_TIVA_I2C2
  tm4c_i2c_register(2);
#endif
#ifdef CONFIG_TIVA_I2C3
  tm4c_i2c_register(3);
#endif
#ifdef CONFIG_TIVA_I2C4
  tm4c_i2c_register(4);
#endif
#ifdef CONFIG_TIVA_I2C5
  tm4c_i2c_register(5);
#endif
#ifdef CONFIG_TIVA_I2C6
  tm4c_i2c_register(6);
#endif
#ifdef CONFIG_TIVA_I2C7
  tm4c_i2c_register(7);
#endif
#ifdef CONFIG_TIVA_I2C8
  tm4c_i2c_register(8);
#endif
#ifdef CONFIG_TIVA_I2C9
  tm4c_i2c_register(9);
#endif
}
#else
#  define tm4c_i2ctool()
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tm4c_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int tm4c_bringup(void)
{
#if defined(HAVE_TMP100) || defined(HAVE_TIMER)
  int ret;
#endif

  /* Register I2C drivers on behalf of the I2C tool */

  tm4c_i2ctool();

#ifdef HAVE_TMP100
  /* Initialize and register the TMP-100 Temperature Sensor driver. */

  ret = tiva_tmp100_initialize(TMP100_DEVNAME);
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize TMP100 driver: %d\n", ret);
    }
#endif

#ifdef HAVE_TIMER
  /* Initialize the timer driver */

  ret = tiva_timer_configure();
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize timer driver: %d\n", ret);
    }
#endif

  return OK;
}
