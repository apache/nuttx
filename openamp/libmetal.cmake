# ##############################################################################
# openamp/libmetal.cmake
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
if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/libmetal)
  FetchContent_Declare(
    libmetal
    DOWNLOAD_NAME "libmetal-main.zip"
    DOWNLOAD_DIR ${CMAKE_CURRENT_LIST_DIR}
    URL "https://github.com/OpenAMP/libmetal/archive/v${LIBMETAL_VERSION}.zip"
        SOURCE_DIR
        ${CMAKE_CURRENT_LIST_DIR}/libmetal
        BINARY_DIR
        ${NUTTX_BINARY_DIR}/openamp/libmetal
        CONFIGURE_COMMAND
        ""
        BUILD_COMMAND
        ""
        INSTALL_COMMAND
        ""
    PATCH_COMMAND
      patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
      ${CMAKE_CURRENT_LIST_DIR}/0001-libmetal-cmake-set-HAVE_STDATOMIC_H-default-true-in-.patch
      && patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
      ${CMAKE_CURRENT_LIST_DIR}/0002-libmetal-atomic-enable-64-bit-atomic-by-toolchain-bu.patch
      && patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
      ${CMAKE_CURRENT_LIST_DIR}/0003-mutex-change-the-libmetal-nuttx-mutex-to-recursive-m.patch
    DOWNLOAD_NO_PROGRESS true
    TIMEOUT 30)

  FetchContent_GetProperties(libmetal)

  if(NOT libmetal_target_POPULATED)
    FetchContent_Populate(libmetal)
  endif()
endif()

if("${CONFIG_ARCH}" STREQUAL "sim")
  set(LIBMETAL_ARCH x86_64)
elseif("${CONFIG_ARCH}" STREQUAL "risc-v")
  set(LIBMETAL_ARCH riscv)
elseif("${CONFIG_ARCH}" STREQUAL "arm64")
  set(LIBMETAL_ARCH aarch64)
else()
  set(LIBMETAL_ARCH ${CONFIG_ARCH})
endif()

set(PROJECT_SYSTEM nuttx)
set(CMAKE_SYSTEM_PROCESSOR ${LIBMETAL_ARCH})
set(MACHINE ${CONFIG_ARCH})
set(CMAKE_SYSTEM_NAME NuttX)
set(WITH_DOC OFF)

if(CONFIG_OPENAMP_CACHE)
  add_compile_definitions(METAL_CACHE)
endif()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/libmetal
                 ${CMAKE_CURRENT_BINARY_DIR}/libmetal EXCLUDE_FROM_ALL)

nuttx_create_symlink(${CMAKE_CURRENT_BINARY_DIR}/libmetal/lib/include/metal
                     ${NUTTX_BINARY_DIR}/include/metal)

nuttx_add_external_library(metal-static MODE KERNEL)

# decorate lib metal output object file name in TASKING. TASKING archive tool
# `artc` DO NOT support obj files with the same name in different directories.
# We workaround this by adding a prefix to the object file name in BINARY dir
if(CONFIG_ARCH_TOOLCHAIN_TASKING)
  get_target_property(METAL_SOURCES metal-static SOURCES)
  set_target_properties(metal-static PROPERTIES SOURCES "")
  foreach(SRC_FILE IN LISTS METAL_SOURCES)
    get_filename_component(ABS_SRC_FILE "${SRC_FILE}" ABSOLUTE)
    file(RELATIVE_PATH REL_SRC_FILE "${CMAKE_SOURCE_DIR}" "${ABS_SRC_FILE}")
    get_filename_component(DIR_PART "${REL_SRC_FILE}" DIRECTORY)
    get_filename_component(FILE_PART "${REL_SRC_FILE}" NAME)
    string(REPLACE "/" "_" DIR_PREFIX "${DIR_PART}")
    if(DIR_PREFIX)
      set(NEW_OBJ_NAME "${DIR_PREFIX}_${FILE_PART}")
    else()
      set(NEW_OBJ_NAME "${FILE_PART}")
    endif()
    configure_file(${CMAKE_CURRENT_LIST_DIR}/libmetal/lib/${SRC_FILE}
                   ${CMAKE_CURRENT_BINARY_DIR}/${NEW_OBJ_NAME} COPYONLY)
    target_sources(metal-static
                   PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/${NEW_OBJ_NAME})
  endforeach()
endif()
