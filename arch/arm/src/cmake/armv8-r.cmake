# ##############################################################################
# arch/arm/src/cmake/armv8-r.cmake
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

if(CONFIG_ARCH_CORTEXR52)
  list(APPEND PLATFORM_FLAGS -mcpu=cortex-r52)
endif()

if(CONFIG_ARCH_FPU)

  if(CONFIG_ARM_NEON)
    list(APPEND PLATFORM_FLAGS -mfpu=neon-fp-armv8)
  else()
    list(APPEND PLATFORM_FLAGS -mfpu=fp-armv8)
  endif()
  if(CONFIG_ARM_FPU_ABI_SOFT)
    list(APPEND PLATFORM_FLAGS -mfloat-abi=softfp)
  else()
    list(APPEND PLATFORM_FLAGS -mfloat-abi=hard)
  endif()
else()
  list(APPEND PLATFORM_FLAGS -mfloat-abi=soft)
endif()

add_compile_options(${PLATFORM_FLAGS})
