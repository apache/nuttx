/****************************************************************************
 * boards/arm/rzv2h/rzv2h-evk/src/rzv2h_bringup.c
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

#include <sys/types.h>
#include <sys/mount.h>
#include <syslog.h>

#include <arch/barriers.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/leds/userled.h>

#include "rzv2h-evk.h"

#if defined(CONFIG_I2C) && defined(CONFIG_RZV2H_RIIC_I2C)
#include <nuttx/i2c/i2c_master.h>
#include "rzv2h_riic_i2c.h"
#endif

#if defined(CONFIG_I2C) && defined(CONFIG_RZV2H_SCI_I2C)
#include <nuttx/i2c/i2c_master.h>
#include "rzv2h_sci_i2c.h"
#endif

#if defined(CONFIG_RZV2H_GPT_PWM)
#include "rzv2h_pwm.h"
#endif

#ifdef CONFIG_RZV2H_CANFD
#include <stdio.h>
#ifndef CONFIG_RZV2H_CANFD_SOCKET
#include <nuttx/can/can.h>
#endif
#include "rzv2h_canfd.h"
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rzv2h_riic_i2c_initialize
 ****************************************************************************/

#if defined(CONFIG_I2C) && defined(CONFIG_RZV2H_RIIC_I2C)
static void rzv2h_riic_i2c_register(int bus)
{
  struct i2c_master_s *i2c;
  int ret;

  i2c = rzv2h_riic_i2cbus_initialize(bus);
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
          rzv2h_riic_i2cbus_uninitialize(i2c);
        }
    }
}

static void rzv2h_riic_i2c_initialize(void)
{
#ifdef CONFIG_RZV2H_RIIC0
  rzv2h_riic_i2c_register(0);
#endif
#ifdef CONFIG_RZV2H_RIIC1
  rzv2h_riic_i2c_register(1);
#endif
#ifdef CONFIG_RZV2H_RIIC2
  rzv2h_riic_i2c_register(2);
#endif
#ifdef CONFIG_RZV2H_RIIC3
  rzv2h_riic_i2c_register(3);
#endif
#ifdef CONFIG_RZV2H_RIIC4
  rzv2h_riic_i2c_register(4);
#endif
#ifdef CONFIG_RZV2H_RIIC5
  rzv2h_riic_i2c_register(5);
#endif
#ifdef CONFIG_RZV2H_RIIC6
  rzv2h_riic_i2c_register(6);
#endif
#ifdef CONFIG_RZV2H_RIIC7
  rzv2h_riic_i2c_register(7);
#endif
#ifdef CONFIG_RZV2H_RIIC8
  rzv2h_riic_i2c_register(8);
#endif
}
#endif

/****************************************************************************
 * Name: rzv2h_sci_i2c_initialize
 *
 * Description:
 *   Initialize the SCI channels used in simple I2C mode and register
 *   SCI channel n as /dev/i2c<CONFIG_RZV2H_SCI_I2C_BUS_OFFSET + n>.
 *
 ****************************************************************************/

#if defined(CONFIG_I2C) && defined(CONFIG_RZV2H_SCI_I2C)
static void rzv2h_sci_i2c_register(int channel)
{
  struct i2c_master_s *i2c;

  i2c = rzv2h_sci_i2cbus_initialize(channel);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get SCI%d I2C interface\n",
             channel);
    }
#ifdef CONFIG_I2C_DRIVER
  else
    {
      int bus = RZV2H_SCI_I2C_BUS(channel);
      int ret;

      ret = i2c_register(i2c, bus);
      if (ret < 0)
        {
          syslog(LOG_ERR,
                 "ERROR: Failed to register SCI%d I2C as I2C%d: %d\n",
                 channel, bus, ret);
          rzv2h_sci_i2cbus_uninitialize(i2c);
        }
    }
#endif
}

static void rzv2h_sci_i2c_initialize(void)
{
#ifdef CONFIG_RZV2H_SCI_I2C0
  rzv2h_sci_i2c_register(0);
#endif
#ifdef CONFIG_RZV2H_SCI_I2C1
  rzv2h_sci_i2c_register(1);
#endif
#ifdef CONFIG_RZV2H_SCI_I2C2
  rzv2h_sci_i2c_register(2);
#endif
#ifdef CONFIG_RZV2H_SCI_I2C3
  rzv2h_sci_i2c_register(3);
#endif
#ifdef CONFIG_RZV2H_SCI_I2C4
  rzv2h_sci_i2c_register(4);
#endif
#ifdef CONFIG_RZV2H_SCI_I2C5
  rzv2h_sci_i2c_register(5);
#endif
#ifdef CONFIG_RZV2H_SCI_I2C6
  rzv2h_sci_i2c_register(6);
#endif
#ifdef CONFIG_RZV2H_SCI_I2C7
  rzv2h_sci_i2c_register(7);
#endif
#ifdef CONFIG_RZV2H_SCI_I2C8
  rzv2h_sci_i2c_register(8);
#endif
#ifdef CONFIG_RZV2H_SCI_I2C9
  rzv2h_sci_i2c_register(9);
#endif
}
#endif

/****************************************************************************
 * Name: rzv2h_canfd_register
 ****************************************************************************/

#ifdef CONFIG_RZV2H_CANFD
static void rzv2h_canfd_register(int channel)
{
#ifdef CONFIG_RZV2H_CANFD_SOCKET
  int ret;

  /* SocketCAN: network interface can<channel> */

  ret = rzv2h_canfd_sock_initialize(channel);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register can%d: %d\n",
             channel, ret);
    }
#else
  struct can_dev_s *dev;
  char path[16];
  int ret;

  dev = rzv2h_canfd_initialize(channel);
  if (dev == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to init CANFD%d\n", channel);
      return;
    }

  snprintf(path, sizeof(path), "/dev/can%d", channel);
  ret = can_register(path, dev);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register %s: %d\n", path, ret);
    }
#endif
}

/****************************************************************************
 * Name: rzv2h_canfd_register_all
 ****************************************************************************/

static void rzv2h_canfd_register_all(void)
{
#ifdef CONFIG_RZV2H_CANFD0
  rzv2h_canfd_register(0);
#endif
#ifdef CONFIG_RZV2H_CANFD1
  rzv2h_canfd_register(1);
#endif
#ifdef CONFIG_RZV2H_CANFD2
  rzv2h_canfd_register(2);
#endif
#ifdef CONFIG_RZV2H_CANFD3
  rzv2h_canfd_register(3);
#endif
#ifdef CONFIG_RZV2H_CANFD4
  rzv2h_canfd_register(4);
#endif
#ifdef CONFIG_RZV2H_CANFD5
  rzv2h_canfd_register(5);
#endif
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rzv2h_bringup
 ****************************************************************************/

int rzv2h_bringup(void)
{
  int ret = OK;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = mount(NULL, "/proc", "procfs", 0, NULL);
  UP_DSB();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#if defined(CONFIG_DEV_GPIO)
  ret = rzv2h_gpio_initialize();
  UP_DSB();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: rzv2h_gpio_initialize() failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_RZV2H_QSPI
  ret = rzv2h_qspi_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: rzv2h_qspi_initialize() failed: %d\n", ret);
      return ret;
    }
#endif

#if !defined(CONFIG_ARCH_LEDS) && defined(CONFIG_USERLED_LOWER)
  /* Register the user LED driver.  The generic lower half calls
   * board_userled_initialize() before registering /dev/userleds.
   */

  ret = userled_lower_initialize(LED_DRIVER_PATH);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
      return ret;
    }
#endif

#if defined(CONFIG_I2C) && defined(CONFIG_RZV2H_RIIC_I2C)
  rzv2h_riic_i2c_initialize();
#endif

#if defined(CONFIG_I2C) && defined(CONFIG_RZV2H_SCI_I2C)
  rzv2h_sci_i2c_initialize();
#endif

#ifdef CONFIG_RZV2H_GPT_PWM
  /* Initialize PWM and register the PWM device */

  ret = rzv2h_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: rzv2h_pwm_setup() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_RZV2H_CANFD
  rzv2h_canfd_register_all();
#endif

  return OK;
}
