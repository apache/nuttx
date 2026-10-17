############################################################################
# net/bt/common/CMakeLists.txt
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
# net/bt/common/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
#
############################################################################

if(CONFIG_BT)
  set(BT_COMMON_SRCS
    ${CMAKE_CURRENT_LIST_DIR}/addr.c
    ${CMAKE_CURRENT_LIST_DIR}/dummy.c
    ${CMAKE_CURRENT_LIST_DIR}/bt_str.c
    ${CMAKE_CURRENT_LIST_DIR}/hci_pkt.c
    ${CMAKE_CURRENT_LIST_DIR}/hci_lockstep.c
  )

  if(CONFIG_BT_HCI_SET_PUBLIC_ADDR)
    list(APPEND BT_COMMON_SRCS ${CMAKE_CURRENT_LIST_DIR}/hci_public_addr.c)
  endif()

  if(CONFIG_BT_RPA)
    list(APPEND BT_COMMON_SRCS ${CMAKE_CURRENT_LIST_DIR}/rpa.c)
  endif()

  if(CONFIG_BT_PRIVATE_SHELL)
    list(APPEND BT_COMMON_SRCS ${CMAKE_CURRENT_LIST_DIR}/bt_shell_private.c)
  endif()

  target_sources(net PRIVATE ${BT_COMMON_SRCS})
endif()
