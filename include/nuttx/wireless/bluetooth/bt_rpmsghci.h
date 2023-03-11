/****************************************************************************
 * include/nuttx/wireless/bluetooth/bt_rpmsghci.h
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

#ifndef __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_RPMSGHCI_H
#define __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_RPMSGHCI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/wireless/bluetooth/bt_driver.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_BLUETOOTH_RPMSG_SERVER

/****************************************************************************
 * Name: rpmsghci_server_init
 *
 * Description:
 *   Rpmsg-HCI server initialize function, the server cpu should call
 *   this function.
 *
 * Parameters:
 *   name - RPMSG-HCI server name
 *   bt   - BT device handler
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

int rpmshci_server_init(FAR const char *name, FAR struct bt_driver_s *dev);
#endif

#ifdef CONFIG_BLUETOOTH_RPMSG

/****************************************************************************
 * Name: rpmsghci_register
 *
 * Description:
 *   Rpmsg-HCI client initialize function, the client cpu should call
 *   this function in the board initialize process.
 *
 * Parameters:
 *   cpuname  - the server cpu name
 *   name     - the HCI device you want to access in the remote cpu
 *
 * Returned Values:
 *    A non-NULL handle is returned on success;
 *    A NULL is returned on any failure.
 *
 ****************************************************************************/

FAR struct bt_driver_s *rpmsghci_register(FAR const char *remotecpu,
                                          FAR const char *name);
#endif

#endif /* __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_RPMSGHCI_H */
