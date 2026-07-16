# ##############################################################################
# arch/arm/src/common/ameba/cmake/ameba_sdk.cmake
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

# Resolve the ameba-rtos SDK checkout + asdk toolchain dir for the CMake build,
# and sanity-check the compiler.  The SDK + asdk toolchain are provisioned by `.
# tools/ameba/env.sh` before cmake, which puts the compiler on PATH for the
# standard probe -- this is why no per-chip hook is needed in the shared arch
# Toolchain.cmake.  Included by the arch chip CMakeLists BEFORE it builds the
# SDK-relative source/include lists, so AMEBA_SDK is available to them.

include_guard(GLOBAL)

set(_ameba_common_dir ${CMAKE_CURRENT_LIST_DIR}/..)

# AMEBA_SDK: prefer the environment (set by env.sh / an external checkout), fall
# back to the shared in-tree checkout.
if(DEFINED ENV{AMEBA_SDK} AND EXISTS "$ENV{AMEBA_SDK}/component/soc")
  set(AMEBA_SDK $ENV{AMEBA_SDK})
elseif(EXISTS "${_ameba_common_dir}/ameba-rtos/component/soc")
  set(AMEBA_SDK ${_ameba_common_dir}/ameba-rtos)
else()
  message(
    FATAL_ERROR
      "ameba-rtos SDK not found.  Source the build environment first:\n"
      "  . tools/ameba/env.sh\n" "then re-run cmake.")
endif()
get_filename_component(AMEBA_SDK ${AMEBA_SDK} ABSOLUTE)

# asdk install root (only needed by the packaging step); matches env.sh.
if(DEFINED ENV{AMEBA_TOOLCHAIN_DIR} AND NOT "$ENV{AMEBA_TOOLCHAIN_DIR}"
                                        STREQUAL "")
  set(AMEBA_TOOLCHAIN_DIR $ENV{AMEBA_TOOLCHAIN_DIR})
else()
  set(AMEBA_TOOLCHAIN_DIR $ENV{HOME}/rtk-toolchain)
endif()

# The compiler must be the SDK-pinned asdk toolchain (put on PATH by env.sh); a
# stray arm-none-eabi-gcc would not match the SDK's prebuilt archives.
if(NOT "${CMAKE_C_COMPILER}" MATCHES "${AMEBA_TOOLCHAIN_DIR}")
  message(
    FATAL_ERROR
      "C compiler '${CMAKE_C_COMPILER}' is not the ameba asdk toolchain.\n"
      "Source the build environment before configuring:\n"
      "  . tools/ameba/env.sh ${CONFIG_ARCH_BOARD}")
endif()

# ...and it must be the asdk version THIS IC pins (RTL8720F -> 12.3.1, RTL8721Dx
# -> 10.3.1).  Both versions share the same arm-none-eabi triple and differ only
# by directory, so a PATH left over from another board (env.sh not re-sourced)
# would silently build NuttX with the wrong compiler.  Catch it at configure
# time and say exactly how to fix it.
execute_process(
  COMMAND sh ${_ameba_common_dir}/tools/ameba_asdk_version.sh ${AMEBA_SDK}
          ${AMEBA_SOC_NAME}
  OUTPUT_VARIABLE _ameba_want_ver
  OUTPUT_STRIP_TRAILING_WHITESPACE)
if(_ameba_want_ver AND NOT "${CMAKE_C_COMPILER}" MATCHES
                       "asdk-${_ameba_want_ver}-")
  message(
    FATAL_ERROR
      "compiler '${CMAKE_C_COMPILER}'\n"
      "  is not asdk-${_ameba_want_ver}, which ${CONFIG_ARCH_CHIP} requires.  The "
      "environment is likely still set for another board.  Re-source it for "
      "this board (a fresh build dir per board):\n"
      "  . tools/ameba/env.sh ${CONFIG_ARCH_BOARD}\n"
      "  cmake -B build-${CONFIG_ARCH_BOARD} -DBOARD_CONFIG=${CONFIG_ARCH_BOARD}:<config> -GNinja"
  )
endif()
