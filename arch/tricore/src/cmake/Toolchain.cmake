# ##############################################################################
# arch/tricore/src/cmake/Toolchain.cmake
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

# Toolchain

# The compiler checks read this file again in a separate project, pass the
# options that pick the toolchain and its flags on to them.

list(
  APPEND
  CMAKE_TRY_COMPILE_PLATFORM_VARIABLES
  CONFIG_TRICORE_TOOLCHAIN_LLVM
  CONFIG_TRICORE_TOOLCHAIN_TASKING
  CONFIG_ARCH_CHIP_FAMILY_TC3X
  CONFIG_ARCH_CHIP_FAMILY_TC4X
  CONFIG_TC3X_CHIP_TC375
  CONFIG_TC4X_CHIP_TC4D7)

if(CONFIG_TRICORE_TOOLCHAIN_LLVM)
  include(${CMAKE_CURRENT_LIST_DIR}/clang.cmake)
elseif(CONFIG_TRICORE_TOOLCHAIN_TASKING)
  include(${CMAKE_CURRENT_LIST_DIR}/tasking.cmake)
else()
  include(${CMAKE_CURRENT_LIST_DIR}/gcc.cmake)
endif()

if(CONFIG_DEBUG_SYMBOLS)
  add_compile_options(${CONFIG_DEBUG_SYMBOLS_LEVEL})
endif()
