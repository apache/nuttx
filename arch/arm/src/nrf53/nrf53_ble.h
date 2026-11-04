/****************************************************************************
 * arch/arm/src/nrf53/nrf53_ble.h
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

#ifndef __ARCH_ARM_SRC_NRF53_NRF53_BLE_H
#define __ARCH_ARM_SRC_NRF53_NRF53_BLE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: nrf53_ble_initialize
 *
 * Description:
 *   Initialize the nRF53 radio lower half and register the BLE Link Layer
 *   as a Bluetooth HCI driver.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int nrf53_ble_initialize(void);

#ifdef CONFIG_BLUETOOTH_RPMSG_SERVER
/****************************************************************************
 * Name: nrf53_ble_rpmsghci_server_initialize
 *
 * Description:
 *   Initialize the nRF53 radio lower half and expose the BLE Link Layer
 *   through an RPMSG HCI server.
 *
 * Input Parameters:
 *   name - RPMSG HCI server name
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int nrf53_ble_rpmsghci_server_initialize(FAR const char *name);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_NRF53_NRF53_BLE_H */
