# ##############################################################################
# wireless/ble_ll/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more contributor
# license agreements.  See the NOTICE file distributed with this work for
# additional information regarding copyright ownership.  The ASF licenses this
# file to you under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License.  You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations under
# the License.
#

if(CONFIG_WIRELESS_BLE_LL)
  set(SRCS
      ble_ll.c
      ble_ll_driver.c
      ble_ll_hci.c
      ble_ll_hci_ev.c
      ble_ll_hci_supp_cmd.c
      ble_ll_hw.c
      ble_ll_iob.c
      ble_ll_pdu.c
      ble_ll_rand.c
      ble_ll_resolv.c
      ble_ll_rfmgmt.c
      ble_ll_sched.c
      ble_ll_tmr.c
      ble_ll_utils.c
      ble_ll_whitelist.c)

  if(CONFIG_BLE_LL_ROLE_BROADCASTER)
    list(APPEND SRCS ble_ll_adv.c)
  endif()

  if(CONFIG_BLE_LL_ROLE_OBSERVER)
    list(APPEND SRCS ble_ll_scan.c)
  endif()

  if(CONFIG_BLE_LL_ROLE_CENTRAL OR CONFIG_BLE_LL_ROLE_PERIPHERAL)
    list(APPEND SRCS ble_ll_conn.c ble_ll_conn_hci.c ble_ll_ctrl.c)
  endif()

  target_sources(wireless PRIVATE ${SRCS})
endif()
