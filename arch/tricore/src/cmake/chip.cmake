# ##############################################################################
# arch/tricore/src/cmake/chip.cmake
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

set(PLATFORM_FLAGS)

if(CONFIG_ARCH_CHIP_FAMILY_TC3X)
  if(CONFIG_TRICORE_TOOLCHAIN_LLVM)
    list(APPEND PLATFORM_FLAGS -march=tc162 -mcpu=tc3xx)
    list(APPEND PLATFORM_FLAGS -Wno-error=implicit-function-declaration)
  else()
    list(APPEND PLATFORM_FLAGS -mtc162)
  endif()
elseif(CONFIG_ARCH_CHIP_FAMILY_TC4X)
  if(CONFIG_TRICORE_TOOLCHAIN_LLVM)
    list(APPEND PLATFORM_FLAGS -march=tc18 -mcpu=tc4xx)
    list(APPEND PLATFORM_FLAGS -Wno-error=implicit-function-declaration)
  else()
    list(APPEND PLATFORM_FLAGS -mtc18)
  endif()
endif()

add_compile_options(${PLATFORM_FLAGS})
add_link_options(${PLATFORM_FLAGS})
