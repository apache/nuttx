# ##############################################################################
# arch/arm/src/cmake/armv8-m.cmake
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

if(CONFIG_ARM_DSP)
  set(EXTCPUFLAGS +dsp)
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
