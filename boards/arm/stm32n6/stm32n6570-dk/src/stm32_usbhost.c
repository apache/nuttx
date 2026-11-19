/****************************************************************************
 * boards/arm/stm32n6/stm32n6570-dk/src/stm32_usbhost.c
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
#include <stdbool.h>
#include <stdint.h>
#include <syslog.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/usb/usbhost.h>

#include "stm32_gpio.h"
#include "stm32_i2c.h"
#include "stm32_otg.h"
#include "stm32_usbhost.h"
#include "stm32n6570-dk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_USBHOST_WAITER
#  error "USB host support needs CONFIG_USBHOST_WAITER"
#endif

#if !defined(CONFIG_STM32N6_OTGHS_USB2) && !defined(CONFIG_STM32_I2C2)
#  error "USB1 host support needs CONFIG_STM32_I2C2 for the TCPP03"
#endif

/* TCPP03 Type-C port protection, register 0 (the only writable one) */

#define TCPP03_REG0              0x00
#define TCPP03_REG0_GDP_CLOSED   (1 << 2)  /* VBUS provider switch closed */
#define TCPP03_REG0_GDC_OPEN     (1 << 3)  /* VBUS consumer switch open */
#define TCPP03_REG0_MODE_NORMAL  (1 << 4)  /* Normal power mode */

#define TCPP03_I2C_FREQUENCY     100000

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifndef CONFIG_STM32N6_OTGHS_USB2
/****************************************************************************
 * Name: stm32_tcpp03_write
 *
 * Description:
 *   Write register 0 of the TCPP03 on I2C2.
 *
 ****************************************************************************/

static int stm32_tcpp03_write(uint8_t value)
{
  struct i2c_master_s *i2c;
  struct i2c_config_s config;
  uint8_t buffer[2];
  int ret;

  i2c = stm32_i2cbus_initialize(TCPP03_I2C_BUS);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  config.frequency = TCPP03_I2C_FREQUENCY;
  config.address   = TCPP03_I2C_ADDR;
  config.addrlen   = 7;

  buffer[0] = TCPP03_REG0;
  buffer[1] = value;

  ret = i2c_write(i2c, &config, buffer, sizeof(buffer));
  stm32_i2cbus_uninitialize(i2c);
  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_usbhost_initialize
 *
 * Description:
 *   Initialize the USB OTG HS host controller and start the thread that
 *   waits for device connections.
 *
 ****************************************************************************/

int stm32_usbhost_initialize(void)
{
  struct usbhost_connection_s *conn;

#ifdef CONFIG_STM32N6_OTGHS_USB2
  /* USB2 Type-A connector: VBUS off until the host driver turns it on */

  stm32_configgpio(GPIO_USB2_PWR_EN);
#else
  /* Enable the TCPP03.  It powers up in hibernate mode with VBUS off. */

  stm32_configgpio(GPIO_TCPP03_EN);
#endif

  conn = stm32_otghshost_initialize(0);
  if (conn == NULL)
    {
      return -ENODEV;
    }

  return usbhost_waiter_initialize(conn);
}

/****************************************************************************
 * Name: stm32_usbhost_vbusdrive
 *
 * Description:
 *   Drive VBUS on the USB2 Type-A connector with its power switch, or on
 *   the USB1 Type-C connector (CN18).  There the TCPP03 switches the 5 V
 *   supply to VBUS through its provider gate driver; it is set up over I2C2
 *   with the consumer switch open.  The CC lines are not driven: the
 *   device is expected to draw VBUS without a Type-C attach handshake.
 *
 ****************************************************************************/

void stm32_usbhost_vbusdrive(int iface, bool enable)
{
#ifdef CONFIG_STM32N6_OTGHS_USB2
  UNUSED(iface);

  stm32_gpiowrite(GPIO_USB2_PWR_EN, enable);
#else
  int ret;

  UNUSED(iface);

  if (enable)
    {
      ret = stm32_tcpp03_write(TCPP03_REG0_MODE_NORMAL |
                               TCPP03_REG0_GDC_OPEN |
                               TCPP03_REG0_GDP_CLOSED);
    }
  else
    {
      ret = stm32_tcpp03_write(TCPP03_REG0_MODE_NORMAL |
                               TCPP03_REG0_GDC_OPEN);
    }

  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: TCPP03 VBUS %s failed: %d\n",
             enable ? "on" : "off", ret);
    }
#endif
}
