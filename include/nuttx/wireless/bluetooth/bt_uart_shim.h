/****************************************************************************
 * include/nuttx/wireless/bluetooth/bt_uart_shim.h
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

#ifndef __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_UART_SHIM_H
#define __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_UART_SHIM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/wireless/bluetooth/bt_uart.h>

#ifdef CONFIG_BLUETOOTH_UART_SHIM

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

FAR struct btuart_lowerhalf_s *btuart_shim_getdevice(FAR const char *path);

#endif /* CONFIG_BLUETOOTH_UART_SHIM */
#endif /* __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_UART_SHIM_H */
