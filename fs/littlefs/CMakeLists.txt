# ##############################################################################
# fs/littlefs/CMakeLists.txt
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

if(CONFIG_FS_LITTLEFS)
  if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/littlefs)
    if(CONFIG_FS_LITTLEFS_LOCAL_PATCHES)
      FetchContent_Declare(
        littlefs
        URL https://github.com/ARMmbed/littlefs/archive/${CONFIG_FS_LITTLEFS_VERSION}.tar.gz
            SOURCE_DIR
            ${CMAKE_CURRENT_LIST_DIR}/littlefs
            BINARY_DIR
            ${CMAKE_BINARY_DIR}/fs/littlefs/littlefs
        PATCH_COMMAND
          patch -p2 -d ${CMAKE_CURRENT_LIST_DIR} <
          ${CMAKE_CURRENT_LIST_DIR}/lfs_util.patch && patch -p2 -d
          ${CMAKE_CURRENT_LIST_DIR} <
          ${CMAKE_CURRENT_LIST_DIR}/lfs_getpath.patch && patch -p2 -d
          ${CMAKE_CURRENT_LIST_DIR} <
          ${CMAKE_CURRENT_LIST_DIR}/lfs_getsetattr.patch)
    else()
      FetchContent_Declare(
        littlefs
        URL https://github.com/ARMmbed/littlefs/archive/${CONFIG_FS_LITTLEFS_VERSION}.tar.gz
            SOURCE_DIR
            ${CMAKE_CURRENT_LIST_DIR}/littlefs
            BINARY_DIR
            ${CMAKE_BINARY_DIR}/fs/littlefs/littlefs)
    endif()
    FetchContent_MakeAvailable(littlefs)
  endif()

  target_compile_definitions(
    fs
    PRIVATE
      $<$<NOT:$<BOOL:${CONFIG_FS_LITTLEFS_HAS_LFS_DEFINES}>>:-DLFS_TRACE=finfo>
      $<$<NOT:$<BOOL:${CONFIG_FS_LITTLEFS_HAS_LFS_DEFINES}>>:-DLFS_DEBUG=finfo>
      $<$<NOT:$<BOOL:${CONFIG_FS_LITTLEFS_HAS_LFS_DEFINES}>>:-DLFS_WARN=fwarn>
      $<$<NOT:$<BOOL:${CONFIG_FS_LITTLEFS_HAS_LFS_DEFINES}>>:-DLFS_ERROR=ferr>
      $<$<NOT:$<BOOL:${CONFIG_FS_LITTLEFS_HAS_LFS_DEFINES}>>:-DLFS_ASSERT=DEBUGASSERT>
      $<$<NOT:$<BOOL:${CONFIG_FS_LITTLEFS_HAS_LFS_DEFINES}>>:-DLFS_CONFIG=${CMAKE_CURRENT_LIST_DIR}/lfs_vfs.h>
      $<$<BOOL:${CONFIG_FS_LITTLEFS_HAS_LFS_DEFINES}>:-DLFS_DEFINES=${CMAKE_CURRENT_LIST_DIR}/lfs_nuttx_defines.h>
      -DLFS_NAME_MAX=${CONFIG_FS_LITTLEFS_NAME_MAX}
      -DLFS_FILE_MAX=${CONFIG_FS_LITTLEFS_FILE_MAX}
      -DLFS_ATTR_MAX=${CONFIG_FS_LITTLEFS_ATTR_MAX}
      $<$<BOOL:${CONFIG_FS_LITTLEFS_MULTI_VERSION}>:-DLFS_MULTIVERSION>)

  # Note: the littlefs upstream doesn't seems to care -Wshadow cf.
  # https://github.com/littlefs-project/littlefs/pull/873
  target_compile_options(fs PRIVATE -Wno-shadow)

  target_sources(fs PRIVATE lfs_vfs.c ${CMAKE_CURRENT_LIST_DIR}/littlefs/lfs.c
                            ${CMAKE_CURRENT_LIST_DIR}/littlefs/lfs_util.c)

  target_include_directories(fs PRIVATE ${CMAKE_CURRENT_LIST_DIR})

  if(CONFIG_TRACE_LITTLEFS_FS)
    set_source_files_properties(lfs_vfs.c DIRECTORY ..
                                PROPERTIES COMPILE_FLAGS -finstrument-functions)
  endif()
endif()
