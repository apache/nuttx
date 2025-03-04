/****************************************************************************
 * boards/risc-v/esp32h2/common/scripts/esp32h2_aliases.ld
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

cache_set_idrom_mmu_size = Cache_Set_IDROM_MMU_Size;
cache_resume_icache = Cache_Resume_ICache;
cache_suspend_icache = Cache_Suspend_ICache;
cache_invalidate_icache_all = Cache_Invalidate_ICache_All;

#ifdef CONFIG_ESPRESSIF_BLE

/* Lower-case aliases for BLE library symbols not compliant to nxstyle */

api_vhci_host_check_send_available = API_vhci_host_check_send_available;
api_vhci_host_send_packet = API_vhci_host_send_packet;
api_vhci_host_register_callback = API_vhci_host_register_callback;

#endif
