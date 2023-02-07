# ##############################################################################
# fs/littlefs/CMakeLists.txt
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

if(CONFIG_FS_LITTLEFS)
  if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/littlefs)
    set(LITTLEFS_VERSION 2.4.0)

    FetchContent_Declare(
      littlefs
      URL https://github.com/ARMmbed/littlefs/archive/v${LITTLEFS_VERSION}.tar.gz
          SOURCE_DIR
          ${CMAKE_CURRENT_LIST_DIR}/littlefs)
    FetchContent_MakeAvailable(littlefs)
  endif()

  target_compile_definitions(
    fs
    PRIVATE -DLFS_TRACE=finfo -DLFS_DEBUG=finfo -DLFS_WARN=fwarn
            -DLFS_ERROR=ferr -DLFS_ASSERT=DEBUGASSERT
            -DLFS_CONFIG=${CMAKE_CURRENT_LIST_DIR}/lfs_vfs.h)

  target_sources(fs PRIVATE lfs_vfs.c ${CMAKE_CURRENT_LIST_DIR}/littlefs/lfs.c
                            ${CMAKE_CURRENT_LIST_DIR}/littlefs/lfs_util.c)

  target_include_directories(fs PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
endif()
