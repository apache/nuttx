/****************************************************************************
 * include/nuttx/wireless/bluetooth/bt_slip.h
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

#ifndef __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_SLIP_H
#define __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_SLIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/wireless/bluetooth/bt_driver.h>

/****************************************************************************
 * Name: bt_slip_register
 *
 * Description:
 *   SLIP-HCI initialize function, the h5 hci protocol would call
 *   this function.
 *
 * Parameters:
 *   drv - SLIP-HCI low bt driver
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

FAR struct bt_driver_s *bt_slip_register(FAR struct bt_driver_s *drv);

#endif /* __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_SLIPHCI_H */
