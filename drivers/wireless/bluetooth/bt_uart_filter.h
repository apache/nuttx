/****************************************************************************
 * drivers/wireless/bluetooth/bt_uart_filter.h
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

#ifndef __DRIVER_WIRELESS_BLUETOOTH_BT_UART_FILTER_H
#define __DRIVER_WIRELESS_BLUETOOTH_BT_UART_FILTER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BT_UART_FILTER_CONN_COUNT   4
#define BT_UART_FILTER_OPCODE_COUNT 16

#define BT_UART_FILTER_TYPE_BT      0
#define BT_UART_FILTER_TYPE_BLE     1
#define BT_UART_FILTER_TYPE_COUNT   2

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct bt_uart_filter_s
{
  int      type;
  uint16_t opcode[BT_UART_FILTER_OPCODE_COUNT];
  uint16_t handle[BT_UART_FILTER_CONN_COUNT];
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void bt_uart_filter_init(FAR struct bt_uart_filter_s *filter, int type);
bool bt_uart_filter_forward_send(FAR struct bt_uart_filter_s *filter,
                                 FAR char *buffer, size_t buflen);
bool bt_uart_filter_forward_recv(FAR struct bt_uart_filter_s *filter,
                                 FAR char *buffer, size_t buflen);

#endif /* __DRIVER_WIRELESS_BLUETOOTH_BT_UART_FILTER_H */
