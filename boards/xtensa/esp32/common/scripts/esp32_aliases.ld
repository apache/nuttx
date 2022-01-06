/****************************************************************************
 * boards/xtensa/esp32/common/scripts/esp32_aliases.ld
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

#include <nuttx/config.h>

/* Lower-case aliases for symbols not compliant to nxstyle */

/* Bluetooth needs symbol alias, to be removed after IDF rename it */

#ifdef CONFIG_ESP32_BLE
  api_vhci_host_check_send_available = API_vhci_host_check_send_available;
  api_vhci_host_send_packet = API_vhci_host_send_packet;
  api_vhci_host_register_callback = API_vhci_host_register_callback;
#endif
