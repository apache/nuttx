############################################################################
# net/bt/services/CMakeLists.txt
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
# net/bt/services/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
#
############################################################################

if(CONFIG_BT)
  set(BT_SERVICE_SRCS)

  if(CONFIG_BT_ANS)
    list(APPEND BT_SERVICE_SRCS ${CMAKE_CURRENT_LIST_DIR}/ans.c)
  endif()
  if(CONFIG_BT_DIS)
    list(APPEND BT_SERVICE_SRCS ${CMAKE_CURRENT_LIST_DIR}/dis.c)
  endif()
  if(CONFIG_BT_GAP_SVC)
    list(APPEND BT_SERVICE_SRCS ${CMAKE_CURRENT_LIST_DIR}/gap_svc.c)
  endif()
  if(CONFIG_BT_CTS)
    list(APPEND BT_SERVICE_SRCS ${CMAKE_CURRENT_LIST_DIR}/cts.c)
  endif()
  if(CONFIG_BT_ETS)
    list(APPEND BT_SERVICE_SRCS ${CMAKE_CURRENT_LIST_DIR}/ets.c)
  endif()
  if(CONFIG_BT_HRS)
    list(APPEND BT_SERVICE_SRCS ${CMAKE_CURRENT_LIST_DIR}/hrs.c)
  endif()
  if(CONFIG_BT_TPS)
    list(APPEND BT_SERVICE_SRCS ${CMAKE_CURRENT_LIST_DIR}/tps.c)
  endif()

  if(CONFIG_BT_BAS)
    list(APPEND BT_SERVICE_SRCS ${CMAKE_CURRENT_LIST_DIR}/bas/bas.c)
  endif()
  if(CONFIG_BT_BAS_BLS)
    list(APPEND BT_SERVICE_SRCS ${CMAKE_CURRENT_LIST_DIR}/bas/bas_bls.c)
  endif()
  if(CONFIG_BT_BAS_BCS)
    list(APPEND BT_SERVICE_SRCS ${CMAKE_CURRENT_LIST_DIR}/bas/bas_bcs.c)
  endif()

  if(CONFIG_BT_IAS)
    list(APPEND BT_SERVICE_SRCS ${CMAKE_CURRENT_LIST_DIR}/ias/ias.c)
  endif()
  if(CONFIG_BT_IAS_CLIENT)
    list(APPEND BT_SERVICE_SRCS ${CMAKE_CURRENT_LIST_DIR}/ias/ias_client.c)
  endif()
  if(CONFIG_BT_SHELL AND (CONFIG_BT_IAS OR CONFIG_BT_IAS_CLIENT))
    if(CONFIG_BT_IAS)
      list(APPEND BT_SERVICE_SRCS ${CMAKE_CURRENT_LIST_DIR}/ias/shell/ias.c)
    endif()
    if(CONFIG_BT_IAS_CLIENT)
      list(APPEND BT_SERVICE_SRCS ${CMAKE_CURRENT_LIST_DIR}/ias/shell/ias_client.c)
    endif()
  endif()

  if(CONFIG_BT_ZEPHYR_NUS)
    list(APPEND BT_SERVICE_SRCS
      ${CMAKE_CURRENT_LIST_DIR}/nus/nus.c
      ${CMAKE_CURRENT_LIST_DIR}/nus/nus_inst.c
    )
    if(CONFIG_BT_ZEPHYR_NUS_AUTO_START_BLUETOOTH)
      list(APPEND BT_SERVICE_SRCS ${CMAKE_CURRENT_LIST_DIR}/nus/bt_nus_auto_start_bt.c)
    endif()
  endif()

  if(CONFIG_BT_OTS)
    list(APPEND BT_SERVICE_SRCS
      ${CMAKE_CURRENT_LIST_DIR}/ots/ots.c
      ${CMAKE_CURRENT_LIST_DIR}/ots/ots_l2cap.c
      ${CMAKE_CURRENT_LIST_DIR}/ots/ots_obj_manager.c
      ${CMAKE_CURRENT_LIST_DIR}/ots/ots_oacp.c
      ${CMAKE_CURRENT_LIST_DIR}/ots/ots_olcp.c
    )
    if(CONFIG_BT_OTS_DIR_LIST_OBJ)
      list(APPEND BT_SERVICE_SRCS ${CMAKE_CURRENT_LIST_DIR}/ots/ots_dir_list.c)
    endif()
  endif()

  if(CONFIG_BT_OTS_CLIENT)
    list(APPEND BT_SERVICE_SRCS ${CMAKE_CURRENT_LIST_DIR}/ots/ots_client.c)
    if(NOT CONFIG_BT_OTS)
      list(APPEND BT_SERVICE_SRCS ${CMAKE_CURRENT_LIST_DIR}/ots/ots_l2cap.c)
    endif()
  endif()

  if(BT_SERVICE_SRCS)
    target_sources(net PRIVATE ${BT_SERVICE_SRCS})
  endif()

  target_include_directories(net PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/bas
    ${CMAKE_CURRENT_LIST_DIR}/ias
    ${CMAKE_CURRENT_LIST_DIR}/nus
    ${CMAKE_CURRENT_LIST_DIR}/ots
  )
endif()
