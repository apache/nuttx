# ##############################################################################
# tools/esp32/mcuboot_toolchain_esp32.cmake
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

set(CMAKE_SYSTEM_NAME Generic)

set(CMAKE_C_COMPILER xtensa-esp32-elf-gcc)
set(CMAKE_CXX_COMPILER xtensa-esp32-elf-g++)
set(CMAKE_ASM_COMPILER xtensa-esp32-elf-gcc)
set(_CMAKE_TOOLCHAIN_PREFIX xtensa-esp32-elf-)

set(CMAKE_C_FLAGS
    "${UNIQ_CMAKE_C_FLAGS}"
    CACHE STRING "C Compiler Base Flags" FORCE)
set(CMAKE_CXX_FLAGS
    "${UNIQ_CMAKE_CXX_FLAGS}"
    CACHE STRING "C++ Compiler Base Flags" FORCE)
set(CMAKE_ASM_FLAGS
    "${UNIQ_CMAKE_ASM_FLAGS}"
    CACHE STRING "ASM Compiler Base Flags" FORCE)

set(CMAKE_EXE_LINKER_FLAGS
    "-Wl,--gc-sections"
    CACHE STRING "Linker Base Flags")
