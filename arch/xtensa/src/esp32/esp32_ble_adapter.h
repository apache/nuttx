/****************************************************************************
 * arch/xtensa/src/esp32/esp32_ble_adapter.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32_ESP32_BLE_ADAPTER_H
#define __ARCH_XTENSA_SRC_ESP32_ESP32_BLE_ADAPTER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

#include "esp_bt.h"

/* Bluetooth system and controller config */

#define BTDM_CFG_BT_DATA_RELEASE            (1 << 0)
#define BTDM_CFG_HCI_UART                   (1 << 1)
#define BTDM_CFG_CONTROLLER_RUN_APP_CPU     (1 << 2)
#define BTDM_CFG_SCAN_DUPLICATE_OPTIONS     (1 << 3)
#define BTDM_CFG_SEND_ADV_RESERVED_SIZE     (1 << 4)
#define BTDM_CFG_BLE_FULL_SCAN_SUPPORTED    (1 << 5)

/* Bluetooth memory regions */

#define SOC_MEM_BT_DATA_START               0x3ffae6e0
#define SOC_MEM_BT_DATA_END                 0x3ffaff10
#define SOC_MEM_BT_EM_START                 0x3ffb0000
#define SOC_MEM_BT_EM_END                   0x3ffb7cd8
#define SOC_MEM_BT_EM_BTDM0_START           0x3ffb0000
#define SOC_MEM_BT_EM_BTDM0_END             0x3ffb09a8
#define SOC_MEM_BT_EM_BLE_START             0x3ffb09a8
#define SOC_MEM_BT_EM_BLE_END               0x3ffb1ddc
#define SOC_MEM_BT_EM_BTDM1_START           0x3ffb1ddc
#define SOC_MEM_BT_EM_BTDM1_END             0x3ffb2730
#define SOC_MEM_BT_EM_BREDR_START           0x3ffb2730
#define SOC_MEM_BT_EM_BREDR_NO_SYNC_END     0x3ffb6388  /* Not calculate with synchronize connection support */
#define SOC_MEM_BT_EM_BREDR_END             0x3ffb7cd8  /* Calculate with synchronize connection support */
#define SOC_MEM_BT_EM_SYNC0_START           0x3ffb6388
#define SOC_MEM_BT_EM_SYNC0_END             0x3ffb6bf8
#define SOC_MEM_BT_EM_SYNC1_START           0x3ffb6bf8
#define SOC_MEM_BT_EM_SYNC1_END             0x3ffb7468
#define SOC_MEM_BT_EM_SYNC2_START           0x3ffb7468
#define SOC_MEM_BT_EM_SYNC2_END             0x3ffb7cd8
#define SOC_MEM_BT_BSS_START                0x3ffb8000
#define SOC_MEM_BT_BSS_END                  0x3ffb9a20
#define SOC_MEM_BT_MISC_START               0x3ffbdb28
#define SOC_MEM_BT_MISC_END                 0x3ffbdb5c

#define SOC_MEM_BT_EM_PER_SYNC_SIZE         0x870

#define SOC_MEM_BT_EM_BREDR_REAL_END        (SOC_MEM_BT_EM_BREDR_NO_SYNC_END + CONFIG_BTDM_CTRL_BR_EDR_MAX_SYNC_CONN_EFF * SOC_MEM_BT_EM_PER_SYNC_SIZE)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_bt_controller_init
 *
 * Description:
 *    Init  BT controller.
 *
 * Input Parameters:
 *    None
 *
 * Returned Value:
 *    None
 *
 ****************************************************************************/

int esp32_bt_controller_init(void);

/****************************************************************************
 * Name: esp32_bt_controller_deinit
 *
 * Description:
 *    Deinit  BT controller.
 * Input Parameters:
 *    cfg -  Initial configuration of BT controller.
 *
 * Returned Value:
 *    None
 *
 ****************************************************************************/

int esp32_bt_controller_deinit(void);

/****************************************************************************
 * Name: esp32_bt_controller_enable
 *
 * Description:
 *    disable  BT controller.
 * Input Parameters:
 *    None
 *
 * Returned Value:
 *    None
 *
 ****************************************************************************/

int esp32_bt_controller_enable(esp_bt_mode_t mode);

/****************************************************************************
 * Name: esp32_bt_controller_disable
 *
 * Description:
 *    disable  BT controller.
 * Input Parameters:
 *    None
 *
 * Returned Value:
 *    None
 *
 ****************************************************************************/

int esp32_bt_controller_disable(void);

/****************************************************************************
 * Name: esp32_bt_controller_enable
 *
 * Description:
 *    Enable  BT controller.
 * Input Parameters:
 *    None
 *
 * Returned Value:
 *    None
 *
 ****************************************************************************/

esp_bt_controller_status_t esp32_bt_controller_get_status(void);

/****************************************************************************
 * Name: esp32_vhci_host_check_send_available
 *
 * Description:
 * used for check actively if the host can send packet to controller or not.
 * Input Parameters:
 *  None
 *
 * Returned Value:
 *   bool - true or false
 *
 ****************************************************************************/

bool esp32_vhci_host_check_send_available(void);

/****************************************************************************
 * Name: esp32_vhci_host_send_packet
 *
 * Description:
 *    host send packet to controller.
 * Input Parameters:
 *  data - the packet point
 *  len - the packet length
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32_vhci_host_send_packet(uint8_t *data, uint16_t len);

/****************************************************************************
 * Name: esp32_vhci_register_callback
 *
 * Description:
 *    register the vhci reference callback.
 * Input Parameters:
 *  callback - struct defined by vhci_host_callback structure.
 *
 * Returned Value:
 *   status - success or fail
 *
 ****************************************************************************/

int esp32_vhci_register_callback(
                            const esp_vhci_host_callback_t *callback);

#endif /* __ARCH_XTENSA_SRC_ESP32_ESP32_BLE_ADAPTER_H */
