/****************************************************************************
 * boards/xtensa/esp32s3/common/scripts/esp32s3_aliases.ld
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

#include <nuttx/config.h>

/* Bluetooth needs symbol alias, to be removed after IDF rename it */

#ifdef CONFIG_ESPRESSIF_BLE
  api_vhci_host_check_send_available = API_vhci_host_check_send_available;
  api_vhci_host_send_packet = API_vhci_host_send_packet;
  api_vhci_host_register_callback = API_vhci_host_register_callback;
  scan_stack_enable_adv_flow_ctrl_vs_cmd = scan_stack_enableAdvFlowCtrlVsCmd;
  adv_stack_enable_clear_legacy_adv_vs_cmd = adv_stack_enableClearLegacyAdvVsCmd;
  adv_filter_stack_enable_dup_exc_list_vs_cmd = advFilter_stack_enableDupExcListVsCmd;
  chan_sel_stack_enable_set_csa_vs_cmd = chanSel_stack_enableSetCsaVsCmd;
#endif

PROVIDE( cache_writeback_addr = Cache_WriteBack_Addr );
