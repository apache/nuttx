# ##############################################################################
# arch/arm/src/cmake/armv7-a.cmake
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

if(CONFIG_ARCH_CORTEXA5)
  list(APPEND PLATFORM_FLAGS -mcpu=cortex-a5)
elseif(CONFIG_ARCH_CORTEXA7)
  list(APPEND PLATFORM_FLAGS -mcpu=cortex-a7)
elseif(CONFIG_ARCH_CORTEXA8)
  list(APPEND PLATFORM_FLAGS -mcpu=cortex-a8)
elseif(CONFIG_ARCH_CORTEXA9)
  list(APPEND PLATFORM_FLAGS -mcpu=cortex-a9)
endif()

if(NOT CONFIG_ARM_DPFPU32)
  set(ARCHFPUD16 -d16)
endif()

# Cortex-A5  | -mfpu=vfpv4-fp16 | -mfpu=vfpv4-d16-fp16 | -mfpu=neon-fp16
# Cortex-A7  | -mfpu=vfpv4      | -mfpu=vfpv4-d16      | -mfpu=neon-vfpv4
# Cortex-A8  | -mfpu=vfpv3      |                      | -mfpu=neon (alias for
# neon-vfpv3) Cortex-A9  | -mfpu=vfpv3-fp16 | -mfpu=vfpv3-d16-fp16 |
# -mfpu=neon-fp16 Cortex-A15 | -mfpu=vfpv4      |                      |
# -mfpu=neon-vfpv4

if(CONFIG_ARCH_FPU)
  if(CONFIG_ARM_FPU_ABI_SOFT)
    list(APPEND PLATFORM_FLAGS -mfloat-abi=softfp)
  else()
    list(APPEND PLATFORM_FLAGS -mfloat-abi=hard)
  endif()

  if(CONFIG_ARM_NEON)
    set(ARCHNEON neon-)
  endif()
  if(CONFIG_ARCH_CORTEXA8)
    set(ARCHFPU vfpv3)
  elseif(CONFIG_ARCH_CORTEXA9)
    set(ARCHFPU vfpv3)
  else()
    set(ARCHFPU vfpv4)
  endif()

  list(APPEND PLATFORM_FLAGS -mfpu=${ARCHNEON}${ARCHFPU}${ARCHFPUD16})

else()
  list(APPEND PLATFORM_FLAGS -mfloat-abi=soft)
endif()

if(CONFIG_ARCH_INSTRUMENT_ALL AND NOT CONFIG_ARMV8M_STACKCHECK)
  list(APPEND PLATFORM_FLAGS -finstrument-functions)
endif()

add_compile_options(${PLATFORM_FLAGS})
