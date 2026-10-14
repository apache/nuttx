/****************************************************************************
 * boards/arm/ra8m1/ek-ra8m1/src/ra8m1_bringup.c
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

#include <stdio.h>
#include <syslog.h>

#include <nuttx/board.h>
#include <nuttx/fs/fs.h>
#include <nuttx/leds/userled.h>

#ifdef CONFIG_I2C
#  include <nuttx/i2c/i2c_master.h>
#endif

#include <arch/board/board.h>

#include "ek_ra8m1.h"

#ifdef CONFIG_RA_GPT_TIMER
#  include "ra_gpt.h"
#endif

#if defined(CONFIG_RA_IIC0) || defined(CONFIG_RA_IIC1)
#  include "ra_iic_i2c.h"
#endif

#if defined(CONFIG_RA_SCI0_I2C) || defined(CONFIG_RA_SCI1_I2C) || \
    defined(CONFIG_RA_SCI2_I2C) || defined(CONFIG_RA_SCI3_I2C) || \
    defined(CONFIG_RA_SCI4_I2C) || defined(CONFIG_RA_SCI9_I2C)
#  include "ra_sci_i2c.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef HAVE_LEDS

#if !defined(CONFIG_ARCH_LEDS) && defined(CONFIG_USERLED_LOWER)
#  define HAVE_LEDS 1
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8m1_i2c_register
 *
 * Description:
 *   Register an already-initialized i2c_master_s interface (from either
 *   the dedicated IIC peripheral or an SCI_B channel in Simple IIC mode)
 *   as /dev/i2c<devno>, e.g. for apps/system/i2c (i2ctool) to use.  devno
 *   is this board's own choice, independent of the backend's internal
 *   channel number -- see ra8m1_bringup()'s own comment.
 *
 ****************************************************************************/

#if defined(CONFIG_RA_IIC0) || defined(CONFIG_RA_IIC1) || \
    defined(CONFIG_RA_SCI0_I2C) || defined(CONFIG_RA_SCI1_I2C) || \
    defined(CONFIG_RA_SCI2_I2C) || defined(CONFIG_RA_SCI3_I2C) || \
    defined(CONFIG_RA_SCI4_I2C) || defined(CONFIG_RA_SCI9_I2C)
static void ra8m1_i2c_register(FAR struct i2c_master_s *i2c, int devno)
{
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get I2C%d interface\n", devno);
      return;
    }

#ifdef CONFIG_SYSTEM_I2CTOOL
  int ret = i2c_register(i2c, devno);

  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register I2C%d driver: %d\n",
             devno, ret);
    }
#endif
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8m1_bringup
 *
 * Description:
 *   Perform board-specific initialization.
 *
 ****************************************************************************/

int ra8m1_bringup(void)
{
  int ret = OK;

#ifdef HAVE_LEDS
  board_userled_initialize();

  /* Register the LED driver */

  ret = userled_lower_initialize(LED_DRIVER_PATH);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_RA_GPT0_GPT
  /* GPT0 is a 32-bit timer */

  ret = ra_gpt_timer_initialize("/dev/timer0", 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: GPT0 timer failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_RA_GPT9_GPT
  /* GPT9 is a 16-bit timer */

  ret = ra_gpt_timer_initialize("/dev/timer1", 9);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: GPT9 timer failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_DEV_GPIO
  ret = ra8m1_gpio_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: ra8m1_gpio_initialize() failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_RA_IIC0) || defined(CONFIG_RA_IIC1) || \
    defined(CONFIG_RA_SCI0_I2C) || defined(CONFIG_RA_SCI1_I2C) || \
    defined(CONFIG_RA_SCI2_I2C) || defined(CONFIG_RA_SCI3_I2C) || \
    defined(CONFIG_RA_SCI4_I2C) || defined(CONFIG_RA_SCI9_I2C)
  /* /dev/i2cN numbers are this board's own choice, assigned here to
   * whichever channels this build actually enables -- not a fixed,
   * chip-wide table reserving a slot for every possible channel whether
   * built or not (see the NuttX convention surveyed across nrf52/cxd56xx:
   * a second I2C-capable peripheral type always gets its own independent
   * init call, with the board deciding its /dev/i2cN number directly).
   * A build with only CONFIG_RA_SCI2_I2C set, for example, gets
   * /dev/i2c0 for it, not /dev/i2c4.
   */

  int i2cbus = 0;
#endif

#ifdef CONFIG_RA_IIC0
  ra8m1_i2c_register(ra_iic_i2cbus_initialize(0), i2cbus++);
#endif

#ifdef CONFIG_RA_IIC1
  ra8m1_i2c_register(ra_iic_i2cbus_initialize(1), i2cbus++);
#endif

#ifdef CONFIG_RA_SCI0_I2C
  ra8m1_i2c_register(ra_sci_i2cbus_initialize(0), i2cbus++);
#endif

#ifdef CONFIG_RA_SCI1_I2C
  ra8m1_i2c_register(ra_sci_i2cbus_initialize(1), i2cbus++);
#endif

#ifdef CONFIG_RA_SCI2_I2C
  ra8m1_i2c_register(ra_sci_i2cbus_initialize(2), i2cbus++);
#endif

#ifdef CONFIG_RA_SCI3_I2C
  ra8m1_i2c_register(ra_sci_i2cbus_initialize(3), i2cbus++);
#endif

#ifdef CONFIG_RA_SCI4_I2C
  ra8m1_i2c_register(ra_sci_i2cbus_initialize(4), i2cbus++);
#endif

#ifdef CONFIG_RA_SCI9_I2C
  ra8m1_i2c_register(ra_sci_i2cbus_initialize(9), i2cbus++);
#endif

  return ret;
}
