# ##############################################################################
# arch/arm/src/cmake/armv7-m_ghs.cmake
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

if(CONFIG_ARCH_CORTEXM4)
  list(APPEND PLATFORM_FLAGS -cpu=cortexm4)
  if(CONFIG_ARCH_FPU)
    if(CONFIG_ARCH_DPFPU)
      list(APPEND PLATFORM_FLAGS -fpu=vfpv3)
    else()
      list(APPEND PLATFORM_FLAGS -fpu=vfpv3_d16)
    endif()
  endif()
elseif(CONFIG_ARCH_CORTEXM7)
  list(APPEND PLATFORM_FLAGS -cpu=cortexm7)
  if(CONFIG_ARCH_FPU)
    if(CONFIG_ARCH_DPFPU)
      list(APPEND PLATFORM_FLAGS -fpu=vfpv3)
    else()
      list(APPEND PLATFORM_FLAGS -fpu=vfpv3_d16)
    endif()
  endif()
else()
  list(APPEND PLATFORM_FLAGS -mtune=cortex-m3 -march=armv7-m -mfloat-abi=soft)
endif()

if(CONFIG_ARCH_FPU)
  if(CONFIG_ARM_FPU_ABI_SOFT)
    list(APPEND PLATFORM_FLAGS -fsoft)
  elseif(CONFIG_ARCH_DPFPU)
    list(APPEND PLATFORM_FLAGS -fhard)
  else()
    list(APPEND PLATFORM_FLAGS -fsingle)
  endif()
else()
  list(APPEND PLATFORM_FLAGS -fsoft)
endif()

if(CONFIG_ARMV7M_STACKCHECK)
  list(APPEND PLATFORM_FLAGS -finstrument-functions -ffixed-r10)
endif()

if(CONFIG_ARCH_INSTRUMENT_ALL AND NOT CONFIG_ARMV7M_STACKCHECK)
  list(APPEND PLATFORM_FLAGS -finstrument-functions)
endif()

add_compile_options(${PLATFORM_FLAGS})
