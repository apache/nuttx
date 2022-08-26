/****************************************************************************
 * include/nuttx/wireless/bluetooth/bt_bridge.h
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

#ifndef __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_BRIDGE_H
#define __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_BRIDGE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/wireless/bluetooth/bt_driver.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: bt_bridge_register
 *
 * Description:
 *   Register the Bluetooth BT/BLE dual mode bridge driver
 *
 ****************************************************************************/

int bt_bridge_register(FAR struct bt_driver_s *hcidrv,
                       FAR struct bt_driver_s **btdrv,
                       FAR struct bt_driver_s **bledrv);

#endif /* __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_BRIDGE_H */
