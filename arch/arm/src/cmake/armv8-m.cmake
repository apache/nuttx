# ##############################################################################
# arch/arm/src/cmake/armv8-m.cmake
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

# LLVM Configuration
if(CONFIG_ARCH_CORTEXM23)
  set(LLVM_ARCHTYPE thumbv8m.base)
  set(LLVM_CPUTYPE cortex-m23)
elseif(CONFIG_ARCH_CORTEXM33)
  set(LLVM_ARCHTYPE thumbv8m.main)
  set(LLVM_CPUTYPE cortex-m33)
elseif(CONFIG_ARCH_CORTEXM35P)
  set(LLVM_ARCHTYPE thumbv8m.main)
  set(LLVM_CPUTYPE cortex-m35p)
elseif(CONFIG_ARCH_CORTEXM55)
  set(LLVM_ARCHTYPE thumbv8.1m.main)
  set(LLVM_CPUTYPE cortex-m55)
elseif(CONFIG_ARCH_CORTEXM85)
  set(LLVM_ARCHTYPE thumbv8.1m.main)
  set(LLVM_CPUTYPE cortex-m85)
endif()

# Set ABI type based on FPU configuration
if(CONFIG_ARCH_FPU)
  set(LLVM_ABITYPE eabihf)
else()
  set(LLVM_ABITYPE eabi)
endif()

if(CONFIG_ARM_DSP)
  set(EXTCPUFLAGS +dsp)
endif()

if(CONFIG_ARM_TOOLCHAIN_CLANG)
  if(CONFIG_ARCH_CORTEXM23)
    set(TOOLCHAIN_CLANG_CONFIG armv8m.main_soft_nofp)
  elseif(CONFIG_ARCH_CORTEXM33)
    if(CONFIG_ARCH_FPU)
      set(TOOLCHAIN_CLANG_CONFIG armv8m.main_hard_fp)
    else()
      set(TOOLCHAIN_CLANG_CONFIG armv8m.main_soft_nofp)
    endif()
  elseif(CONFIG_ARCH_CORTEXM35P)
    if(CONFIG_ARCH_FPU)
      set(TOOLCHAIN_CLANG_CONFIG armv8m.main_hard_fp)
    else()
      set(TOOLCHAIN_CLANG_CONFIG armv8m.main_soft_nofp)
    endif()
  elseif(CONFIG_ARCH_CORTEXM55)
    if(CONFIG_ARCH_FPU)
      set(TOOLCHAIN_CLANG_CONFIG armv8.1m.main_hard_fp)
    else()
      set(TOOLCHAIN_CLANG_CONFIG armv8.1m.main_soft_nofp_nomve)
    endif()
  elseif(CONFIG_ARCH_CORTEXM85)
    if(CONFIG_ARCH_FPU)
      set(TOOLCHAIN_CLANG_CONFIG armv8.1m.main_hard_fp)
    else()
      set(TOOLCHAIN_CLANG_CONFIG armv8.1m.main_soft_nofp_nomve)
    endif()
  endif()
endif()

if(CONFIG_ARCH_CORTEXM23)
  list(APPEND PLATFORM_FLAGS -mtune=cortex-m23 -march=armv8-m.main
       -mfloat-abi=soft)
elseif(CONFIG_ARCH_CORTEXM33)
  list(APPEND PLATFORM_FLAGS -mtune=cortex-m33
       -march=armv8-m.main${EXTCPUFLAGS})
  if(CONFIG_ARCH_FPU)
    list(APPEND PLATFORM_FLAGS -mfpu=fpv5-sp-d16)
  endif()
elseif(CONFIG_ARCH_CORTEXM35P)
  list(APPEND PLATFORM_FLAGS -mtune=cortex-m35p
       -march=armv8-m.main${EXTCPUFLAGS})
  if(CONFIG_ARCH_FPU)
    list(APPEND PLATFORM_FLAGS -mfpu=fpv5-sp-d16)
  endif()
elseif(CONFIG_ARCH_CORTEXM55)
  list(APPEND PLATFORM_FLAGS -mtune=cortex-m55)
  if(CONFIG_ARM_HAVE_MVE)
    list(APPEND PLATFORM_FLAGS -march=armv8.1-m.main+mve.fp+fp.dp)
  else()
    list(APPEND PLATFORM_FLAGS -march=armv8.1-m.main${EXTCPUFLAGS})
  endif()
  if(CONFIG_ARCH_FPU)
    list(APPEND PLATFORM_FLAGS -mfpu=fpv5-d16)
  endif()
endif()

if(CONFIG_ARCH_FPU)
  if(CONFIG_ARM_FPU_ABI_SOFT)
    list(APPEND PLATFORM_FLAGS -mfloat-abi=softfp)
  else()
    list(APPEND PLATFORM_FLAGS -mfloat-abi=hard)
  endif()
else()
  list(APPEND PLATFORM_FLAGS -mfloat-abi=soft)
endif()

if(CONFIG_ARMV8M_STACKCHECK)
  list(APPEND PLATFORM_FLAGS -finstrument-functions -ffixed-r10)
endif()

if(CONFIG_ARCH_INSTRUMENT_ALL AND NOT CONFIG_ARMV8M_STACKCHECK)
  list(APPEND PLATFORM_FLAGS -finstrument-functions)
endif()

if(CONFIG_ARMV8M_CMSE)
  list(APPEND PLATFORM_FLAGS -mcmse)
endif()

add_compile_options(${PLATFORM_FLAGS})
