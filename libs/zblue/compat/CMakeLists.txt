# ##############################################################################
# libs/zblue/compat/CMakeLists.txt
#
# Author: chao an <magicdcps@hotmail.com>
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
# ##############################################################################

target_sources(
  zblue
  PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src/kernel/queue.c
          ${CMAKE_CURRENT_LIST_DIR}/src/kernel/timeout.c
          ${CMAKE_CURRENT_LIST_DIR}/src/kernel/mem_slab.c
          ${CMAKE_CURRENT_LIST_DIR}/src/net_buf/net_buf.c
          ${CMAKE_CURRENT_LIST_DIR}/src/net_buf/net_buf_simple.c
          ${CMAKE_CURRENT_LIST_DIR}/src/sections/defines.c
          ${CMAKE_CURRENT_LIST_DIR}/src/subsys/fs/fs.c
          ${CMAKE_CURRENT_LIST_DIR}/src/drivers/bluetooth/hci/hci_driver.c)
