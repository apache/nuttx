# ##############################################################################
# arch/arm64/src/cmake/Toolchain.cmake
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
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_C_COMPILER_FORCED TRUE)
set(CMAKE_CXX_COMPILER_FORCED TRUE)
set(OPTION_MARCH_FEATURE)

if(CONFIG_ARCH_HAVE_AE)
  set(OPTION_MARCH_FEATURE ae)
endif()

if(CONFIG_ARCH_ARMV8A)
  add_compile_options(-march=armv8-a)
elseif(CONFIG_ARCH_ARMV8R)
  if(CONFIG_ARCH_FPU)
    add_compile_options(-march=armv8-r)
  else()
    add_compile_options(-march=armv8-r+nofp)
  endif()
elseif(CONFIG_ARCH_CORTEX_A53)
  set(LLVM_CPUTYPE cortex-a53${OPTION_MARCH_FEATURE})
  add_compile_options(-mcpu=cortex-a53${OPTION_MARCH_FEATURE})
elseif(CONFIG_ARCH_CORTEX_A57)
  set(LLVM_CPUTYPE cortex-a57${OPTION_MARCH_FEATURE})
  add_compile_options(-mcpu=cortex-a57${OPTION_MARCH_FEATURE})
elseif(CONFIG_ARCH_CORTEX_A72)
  set(LLVM_CPUTYPE cortex-a72${OPTION_MARCH_FEATURE})
  add_compile_options(-mcpu=cortex-a72${OPTION_MARCH_FEATURE})
elseif(CONFIG_ARCH_CORTEX_R82)
  set(LLVM_CPUTYPE cortex-r82${OPTION_MARCH_FEATURE})
  add_compile_options(-mcpu=cortex-r82${OPTION_MARCH_FEATURE})
endif()

# include the toolchain specific cmake file

if(CONFIG_ARCH_TOOLCHAIN_CLANG)
  include(clang)
elseif(CONFIG_ARCH_TOOLCHAIN_GCC) # gcc
  include(gcc)
endif()
