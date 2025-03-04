# ##############################################################################
# fs/driver/CMakeLists.txt
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

set(SRCS fs_registerdriver.c fs_unregisterdriver.c fs_registerpipedriver.c
         fs_unregisterpipedriver.c)

# Don't built-in block driver support if there are no mountpoints

if(NOT CONFIG_DISABLE_MOUNTPOINT)
  list(
    APPEND
    SRCS
    fs_registerblockdriver.c
    fs_unregisterblockdriver.c
    fs_findblockdriver.c
    fs_openblockdriver.c
    fs_closeblockdriver.c
    fs_blockpartition.c
    fs_findmtddriver.c
    fs_blockmerge.c
    fs_closemtddriver.c)

  if(CONFIG_MTD)
    list(APPEND SRCS fs_registermtddriver.c fs_unregistermtddriver.c
         fs_mtdproxy.c)

    if(CONFIG_MTD_PARTITION)
      list(APPEND SRCS fs_mtdpartition.c)
    endif()
  endif()

  if(CONFIG_BCH)
    if(NOT CONFIG_DISABLE_PSEUDOFS_OPERATIONS)
      list(APPEND SRCS fs_blockproxy.c)
    endif()
  endif()
endif()

target_sources(fs PRIVATE ${SRCS})
