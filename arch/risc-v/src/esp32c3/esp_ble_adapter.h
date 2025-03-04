/****************************************************************************
 * arch/risc-v/src/esp32c3/esp_ble_adapter.h
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

#ifndef __ARCH_RISCV_SRC_ESP32C3_ESP_BLE_ADAPTER_H
#define __ARCH_RISCV_SRC_ESP32C3_ESP_BLE_ADAPTER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

#include "esp_bt.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp_bt_controller_init
 *
 * Description:
 *   Init  BT controller.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int esp_bt_controller_init(void);

/****************************************************************************
 * Name: esp_bt_controller_deinit
 *
 * Description:
 *   Deinit BT controller.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise, -1 (ERROR) is returned.
 *
 ****************************************************************************/

int esp_bt_controller_deinit(void);

/****************************************************************************
 * Name: esp_bt_controller_enable
 *
 * Description:
 *   Enable BT controller.
 *
 * Input Parameters:
 *   mode - the mode(BLE/BT/BTDM) to enable. For compatible of API, retain
 *   this argument. This mode must be equal as the mode in "cfg" of
 *   esp_bt_controller_init().
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int esp_bt_controller_enable(esp_bt_mode_t mode);

/****************************************************************************
 * Name: esp_bt_controller_disable
 *
 * Description:
 *   Disable BT controller.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int esp_bt_controller_disable(void);

/****************************************************************************
 * Name: esp_bt_controller_get_status
 *
 * Description:
 *   Enable BT controller.
 *
 * Input Parameters:
 *   mode - the mode(BLE/BT/BTDM) to enable. For compatible of API, retain
 *   this argument. This mode must be equal as the mode in "cfg" of
 *   esp_bt_controller_init().
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

esp_bt_controller_status_t esp_bt_controller_get_status(void);

/****************************************************************************
 * Name: esp_vhci_host_check_send_available
 *
 * Description:
 *   Check if the host can send packet to controller or not.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   True if returned on success. Otherwise, false is returned.
 *
 ****************************************************************************/

bool esp_vhci_host_check_send_available(void);

/****************************************************************************
 * Name: esp_vhci_host_send_packet
 *
 * Description:
 *   Host send packet to controller.
 *
 * Input Parameters:
 *   data - the packet pointer
 *   len  - the packet length
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp_vhci_host_send_packet(uint8_t *data, uint16_t len);

/****************************************************************************
 * Name: esp_vhci_register_callback
 *
 * Description:
 *   Register the vhci reference callback.
 *
 * Input Parameters:
 *   callback - struct defined by vhci_host_callback structure.
 *
 * Returned Value:
 *   status - success or fail
 *
 ****************************************************************************/

int esp_vhci_register_callback(const esp_vhci_host_callback_t *callback);

#endif /* __ARCH_RISCV_SRC_ESP32C3_ESP_BLE_ADAPTER_H */
