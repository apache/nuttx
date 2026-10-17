############################################################################
# net/bt/mesh/CMakeLists.txt
#
# Copyright (c) ByteDance Ltd. and/or its affiliates.
#
# Author: chao an <anchao.archer@bytedance.com>
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

############################################################################
# net/bt/mesh/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
#
############################################################################

if(CONFIG_BT_MESH)
  set(BT_MESH_SRCS
    ${CMAKE_CURRENT_LIST_DIR}/main.c
    ${CMAKE_CURRENT_LIST_DIR}/cfg.c
    ${CMAKE_CURRENT_LIST_DIR}/adv.c
    ${CMAKE_CURRENT_LIST_DIR}/beacon.c
    ${CMAKE_CURRENT_LIST_DIR}/net.c
    ${CMAKE_CURRENT_LIST_DIR}/subnet.c
    ${CMAKE_CURRENT_LIST_DIR}/app_keys.c
    ${CMAKE_CURRENT_LIST_DIR}/heartbeat.c
    ${CMAKE_CURRENT_LIST_DIR}/crypto.c
    ${CMAKE_CURRENT_LIST_DIR}/crypto_psa.c
    ${CMAKE_CURRENT_LIST_DIR}/access.c
    ${CMAKE_CURRENT_LIST_DIR}/msg.c
    ${CMAKE_CURRENT_LIST_DIR}/cfg_srv.c
    ${CMAKE_CURRENT_LIST_DIR}/health_srv.c
    ${CMAKE_CURRENT_LIST_DIR}/va.c
    ${CMAKE_CURRENT_LIST_DIR}/transport.c
  )

  if(CONFIG_BT_MESH_ADV_LEGACY)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/adv_legacy.c)
  endif()
  if(CONFIG_BT_MESH_ADV_EXT)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/adv_ext.c)
  endif()
  if(CONFIG_BT_SETTINGS)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/settings.c)
  endif()
  if(CONFIG_BT_MESH_RPL_STORAGE_MODE_SETTINGS)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/rpl.c)
  endif()
  if(CONFIG_BT_MESH_LOW_POWER)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/lpn.c)
  endif()
  if(CONFIG_BT_MESH_FRIEND)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/friend.c)
  endif()
  if(CONFIG_BT_MESH_PROV)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/prov.c)
  endif()
  if(CONFIG_BT_MESH_PROVISIONEE)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/provisionee.c)
  endif()
  if(CONFIG_BT_MESH_PROVISIONER)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/provisioner.c)
  endif()
  if(CONFIG_BT_MESH_PB_ADV)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/pb_adv.c)
  endif()
  if(CONFIG_BT_MESH_PB_GATT_COMMON)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/pb_gatt.c)
  endif()
  if(CONFIG_BT_MESH_PB_GATT)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/pb_gatt_srv.c)
  endif()
  if(CONFIG_BT_MESH_PB_GATT_CLIENT)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/pb_gatt_cli.c)
  endif()
  if(CONFIG_BT_MESH_GATT_CLIENT)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/gatt_cli.c)
  endif()
  if(CONFIG_BT_MESH_PROXY_CLIENT)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/proxy_cli.c)
  endif()
  if(CONFIG_BT_MESH_GATT_PROXY)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/proxy_srv.c)
  endif()
  if(CONFIG_BT_MESH_GATT)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/proxy_msg.c)
  endif()
  if(CONFIG_BT_MESH_CFG_CLI)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/cfg_cli.c)
  endif()
  if(CONFIG_BT_MESH_HEALTH_CLI)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/health_cli.c)
  endif()
  if(CONFIG_BT_MESH_SAR_CFG_CLI)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/sar_cfg_cli.c)
  endif()
  if(CONFIG_BT_MESH_SAR_CFG_SRV)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/sar_cfg_srv.c)
  endif()
  if(CONFIG_BT_MESH_SAR_CFG)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/sar_cfg.c)
  endif()
  if(CONFIG_BT_MESH_OP_AGG_SRV)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/op_agg_srv.c)
  endif()
  if(CONFIG_BT_MESH_OP_AGG_CLI)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/op_agg_cli.c)
  endif()
  if(CONFIG_BT_MESH_OP_AGG)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/op_agg.c)
  endif()
  if(CONFIG_BT_MESH_LARGE_COMP_DATA_SRV)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/large_comp_data_srv.c)
  endif()
  if(CONFIG_BT_MESH_LARGE_COMP_DATA_CLI)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/large_comp_data_cli.c)
  endif()
  if(CONFIG_BT_MESH_PRIV_BEACON_SRV)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/priv_beacon_srv.c)
  endif()
  if(CONFIG_BT_MESH_PRIV_BEACON_CLI)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/priv_beacon_cli.c)
  endif()
  if(CONFIG_BT_MESH_SELF_TEST)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/test.c)
  endif()
  if(CONFIG_BT_MESH_CDB)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/cdb.c)
  endif()
  if(CONFIG_BT_MESH_BLOB_SRV)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/blob_srv.c)
  endif()
  if(CONFIG_BT_MESH_BLOB_CLI)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/blob_cli.c)
  endif()
  if(CONFIG_BT_MESH_BLOB_IO_FLASH)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/blob_io_flash.c)
  endif()
  if(CONFIG_BT_MESH_DFU_CLI)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/dfu_cli.c)
  endif()
  if(CONFIG_BT_MESH_DFU_SRV)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/dfu_srv.c)
  endif()
  if(CONFIG_BT_MESH_DFD_SRV)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/dfd_srv.c)
  endif()
  if(CONFIG_BT_MESH_DFU_SLOTS)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/dfu_slot.c)
  endif()
  if(CONFIG_BT_MESH_DFU_METADATA)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/dfu_metadata.c)
  endif()
  if(CONFIG_BT_MESH_RPR_CLI)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/rpr_cli.c)
  endif()
  if(CONFIG_BT_MESH_RPR_SRV)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/rpr_srv.c)
  endif()
  if(CONFIG_BT_MESH_OD_PRIV_PROXY_CLI)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/od_priv_proxy_cli.c)
  endif()
  if(CONFIG_BT_MESH_OD_PRIV_PROXY_SRV)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/od_priv_proxy_srv.c ${CMAKE_CURRENT_LIST_DIR}/sol_pdu_rpl_srv.c)
  endif()
  if(CONFIG_BT_MESH_SOL_PDU_RPL_CLI)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/sol_pdu_rpl_cli.c)
  endif()
  if(CONFIG_BT_MESH_BRG_CFG_CLI)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/brg_cfg_cli.c)
  endif()
  if(CONFIG_BT_MESH_BRG_CFG_SRV)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/brg_cfg_srv.c ${CMAKE_CURRENT_LIST_DIR}/brg_cfg.c)
  endif()
  if(CONFIG_BT_MESH_SOLICITATION)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/solicitation.c)
  endif()
  if(CONFIG_BT_MESH_STATISTIC)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/statistic.c)
  endif()
  if(CONFIG_BT_MESH_ACCESS_DELAYABLE_MSG)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/delayable_msg.c)
  endif()
  if(CONFIG_BT_TESTING)
    list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/testing.c)
  endif()

  if(CONFIG_BT_MESH_SHELL)
    list(APPEND BT_MESH_SRCS
      ${CMAKE_CURRENT_LIST_DIR}/shell/shell.c
      ${CMAKE_CURRENT_LIST_DIR}/shell/utils.c
    )
    if(CONFIG_BT_MESH_SHELL_HEALTH_CLI)
      list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/health.c)
    endif()
    if(CONFIG_BT_MESH_SHELL_CFG_CLI)
      list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/cfg.c)
    endif()
    if(CONFIG_BT_MESH_SHELL_LARGE_COMP_DATA_CLI)
      list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/large_comp_data.c)
    endif()
    if(CONFIG_BT_MESH_SHELL_OP_AGG_CLI)
      list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/op_agg.c)
    endif()
    if(CONFIG_BT_MESH_SHELL_PRIV_BEACON_CLI)
      list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/priv_beacon.c)
    endif()
    if(CONFIG_BT_MESH_SHELL_RPR_CLI)
      list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/rpr.c)
    endif()
    if(CONFIG_BT_MESH_SHELL_SAR_CFG_CLI)
      list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/sar.c)
    endif()
    if(CONFIG_BT_MESH_SHELL_BLOB_CLI OR CONFIG_BT_MESH_SHELL_BLOB_SRV OR CONFIG_BT_MESH_SHELL_BLOB_IO_FLASH)
      list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/blob.c)
    endif()
    if(CONFIG_BT_MESH_SHELL_DFU_SLOT OR CONFIG_BT_MESH_SHELL_DFU_METADATA OR CONFIG_BT_MESH_SHELL_DFU_CLI OR CONFIG_BT_MESH_SHELL_DFU_SRV)
      list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/dfu.c)
    endif()
    if(CONFIG_BT_MESH_SHELL_DFD_SRV)
      list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/dfd.c)
    endif()
    if(CONFIG_BT_MESH_OD_PRIV_PROXY_CLI)
      list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/od_priv_proxy.c)
    endif()
    if(CONFIG_BT_MESH_SOL_PDU_RPL_CLI)
      list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/sol_pdu_rpl.c)
    endif()
    if(CONFIG_BT_MESH_SHELL_BRG_CFG_CLI)
      list(APPEND BT_MESH_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/brg_cfg.c)
    endif()
  endif()

  target_sources(net PRIVATE ${BT_MESH_SRCS})
  target_include_directories(net PRIVATE ${CMAKE_CURRENT_LIST_DIR})
endif()
