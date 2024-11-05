/****************************************************************************
 * drivers/wireless/bluetooth/bt_driver.c
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
#include <nuttx/wireless/bluetooth/bt_driver.h>

#ifdef CONFIG_UART_BTH4
#  include <nuttx/serial/uart_bth4.h>
#endif

#ifdef CONFIG_BLUETOOTH_BRIDGE
#  include <nuttx/wireless/bluetooth/bt_bridge.h>
#endif

#ifdef CONFIG_BLUETOOTH_SLIP
#  include <nuttx/wireless/bluetooth/bt_slip.h>
#endif

#include <stdio.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int bt_driver_register_internal(FAR struct bt_driver_s *driver,
                                       FAR const char *prefix, int id)
{
#ifdef CONFIG_UART_BTH4
  char name[32];

  snprintf(name, sizeof(name), prefix, id);
  return uart_bth4_register(name, driver);
#elif defined(CONFIG_NET_BLUETOOTH)
  return bt_netdev_register(driver);
#else
  return -ENOSYS;
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bt_driver_register_with_id
 *
 * Description:
 *   Register bluetooth driver.
 *
 * Input Parameters:
 *   driver - an instance of the bt_driver_s interface
 *   id     - bluetooth device id
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

int bt_driver_register_with_id(FAR struct bt_driver_s *driver, int id)
{
#ifdef CONFIG_BLUETOOTH_BRIDGE
  FAR struct bt_driver_s *btdrv;
  FAR struct bt_driver_s *bledrv;
#endif
  int ret;

#ifdef CONFIG_BLUETOOTH_SLIP
  driver = bt_slip_register(driver);
  if (driver == NULL)
    {
      return -ENOMEM;
    }
#endif

#ifdef CONFIG_BLUETOOTH_BRIDGE
  ret = bt_bridge_register(driver, &btdrv, &bledrv);
  if (ret < 0)
    {
      return ret;
    }

  ret = bt_driver_register_internal(btdrv, "/dev/ttyBT%d", id);
  if (ret < 0)
    {
      return ret;
    }

  ret = bt_driver_register_internal(bledrv, "/dev/ttyBLE%d", id);
  if (ret < 0)
    {
      return ret;
    }

#else
  ret = bt_driver_register_internal(driver, "/dev/ttyHCI%d", id);
  if (ret < 0)
    {
      return ret;
    }
#endif

  return ret;
}
