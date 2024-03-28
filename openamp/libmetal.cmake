# ##############################################################################
# openamp/libmetal.cmake
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
    DOWNLOAD_NAME "libmetal-v${OPENAMP_VERSION}.zip"
    DOWNLOAD_DIR ${CMAKE_CURRENT_LIST_DIR}
    URL "https://github.com/OpenAMP/libmetal/archive/v${OPENAMP_VERSION}.zip"
        SOURCE_DIR
        ${CMAKE_CURRENT_LIST_DIR}/libmetal
        BINARY_DIR
        ${CMAKE_BINARY_DIR}/openamp/libmetal
        CONFIGURE_COMMAND
        ""
        BUILD_COMMAND
        ""
        INSTALL_COMMAND
        ""
    PATCH_COMMAND
      patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
      ${CMAKE_CURRENT_LIST_DIR}/0001-libmetal-add-metal_list_for_each_safe-support.patch
      && patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
      ${CMAKE_CURRENT_LIST_DIR}/0002-libmetal-nuttx-io.c-align-access-when-read-write-siz.patch
      && patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
      ${CMAKE_CURRENT_LIST_DIR}/0003-libmetal-nuttx-io.c-Fix-void-pointer-arithmetic-in-a.patch
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

set(PROJECT_VERSION_MAJOR 0)
set(PROJECT_VERSION_MINOR 1)
set(PROJECT_VERSION_PATCH 0)
set(PROJECT_VERSION 0.1.0)
set(PROJECT_SYSTEM nuttx)
set(PROJECT_PROCESSOR ${LIBMETAL_ARCH})
set(PROJECT_MACHINE ${CONFIG_ARCH_CHIP})
set(PROJECT_SYSTEM_UPPER nuttx)
set(PROJECT_PROCESSOR_UPPER ${LIBMETAL_ARCH})
set(PROJECT_MACHINE_UPPER ${CONFIG_ARCH_CHIP})

set(headers)
file(
  GLOB headers
  LIST_DIRECTORIES false
  RELATIVE ${CMAKE_CURRENT_LIST_DIR}/libmetal/lib
  ${CMAKE_CURRENT_LIST_DIR}/libmetal/lib/*.h)
foreach(header ${headers})
  configure_file(${CMAKE_CURRENT_LIST_DIR}/libmetal/lib/${header}
                 ${CMAKE_BINARY_DIR}/include/metal/${header})
endforeach()

set(headers)
file(
  GLOB headers
  LIST_DIRECTORIES false
  RELATIVE ${CMAKE_CURRENT_LIST_DIR}/libmetal/lib/system/nuttx
  ${CMAKE_CURRENT_LIST_DIR}/libmetal/lib/system/nuttx/*.h)
foreach(header ${headers})
  configure_file(${CMAKE_CURRENT_LIST_DIR}/libmetal/lib/system/nuttx/${header}
                 ${CMAKE_BINARY_DIR}/include/metal/system/nuttx/${header})
endforeach()

set(headers)
file(
  GLOB headers
  LIST_DIRECTORIES false
  RELATIVE ${CMAKE_CURRENT_LIST_DIR}/libmetal/lib/processor/${LIBMETAL_ARCH}
  ${CMAKE_CURRENT_LIST_DIR}/libmetal/lib/processor/${LIBMETAL_ARCH}/*.h)
foreach(header ${headers})
  configure_file(
    ${CMAKE_CURRENT_LIST_DIR}/libmetal/lib/processor/${LIBMETAL_ARCH}/${header}
    ${CMAKE_BINARY_DIR}/include/metal/processor/${LIBMETAL_ARCH}/${header})
endforeach()

set(headers)
file(
  GLOB headers
  LIST_DIRECTORIES false
  RELATIVE ${CMAKE_CURRENT_LIST_DIR}/libmetal/lib/compiler/gcc
  ${CMAKE_CURRENT_LIST_DIR}/libmetal/lib/compiler/gcc/*.h)
foreach(header ${headers})
  configure_file(${CMAKE_CURRENT_LIST_DIR}/libmetal/lib/compiler/gcc/${header}
                 ${CMAKE_BINARY_DIR}/include/metal/compiler/gcc/${header})
endforeach()

nuttx_add_kernel_library(lib_metal)

target_sources(
  lib_metal
  PRIVATE libmetal/lib/system/nuttx/condition.c
          libmetal/lib/system/nuttx/device.c
          libmetal/lib/system/nuttx/init.c
          libmetal/lib/system/nuttx/io.c
          libmetal/lib/system/nuttx/irq.c
          libmetal/lib/system/nuttx/shmem.c
          libmetal/lib/system/nuttx/time.c
          libmetal/lib/device.c
          libmetal/lib/dma.c
          libmetal/lib/init.c
          libmetal/lib/io.c
          libmetal/lib/irq.c
          libmetal/lib/log.c
          libmetal/lib/shmem.c
          libmetal/lib/version.c)

target_compile_definitions(lib_metal PRIVATE METAL_INTERNAL)
