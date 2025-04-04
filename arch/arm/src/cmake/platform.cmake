# ##############################################################################
# arch/arm/src/cmake/platform.cmake
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

# configure target processor

if(CONFIG_ARCH_CORTEXM0)
  set(CMAKE_SYSTEM_PROCESSOR cortex-m0)
elseif(CONFIG_ARCH_CORTEXM3)
  set(CMAKE_SYSTEM_PROCESSOR cortex-m3)
elseif(CONFIG_ARCH_CORTEXM4)
  set(CMAKE_SYSTEM_PROCESSOR cortex-m4)
elseif(CONFIG_ARCH_CORTEXM7)
  set(CMAKE_SYSTEM_PROCESSOR cortex-m7)
elseif(CONFIG_ARCH_CORTEXM23)
  set(CMAKE_SYSTEM_PROCESSOR cortex-m23)
elseif(CONFIG_ARCH_CORTEXM33)
  set(CMAKE_SYSTEM_PROCESSOR cortex-m33)
elseif(CONFIG_ARCH_CORTEXM35P)
  set(CMAKE_SYSTEM_PROCESSOR cortex-m35p)
elseif(CONFIG_ARCH_CORTEXM55)
  set(CMAKE_SYSTEM_PROCESSOR cortex-m55)
elseif(CONFIG_ARCH_CORTEXA5)
  set(CMAKE_SYSTEM_PROCESSOR cortex-a5)
elseif(CONFIG_ARCH_CORTEXA7)
  set(CMAKE_SYSTEM_PROCESSOR cortex-a7)
elseif(CONFIG_ARCH_CORTEXA8)
  set(CMAKE_SYSTEM_PROCESSOR cortex-a8)
elseif(CONFIG_ARCH_CORTEXA9)
  set(CMAKE_SYSTEM_PROCESSOR cortex-a9)
elseif(CONFIG_ARCH_CORTEXR4)
  set(CMAKE_SYSTEM_PROCESSOR cortex-r4)
elseif(CONFIG_ARCH_CORTEXR5)
  set(CMAKE_SYSTEM_PROCESSOR cortex-r5)
elseif(CONFIG_ARCH_CORTEXR7)
  set(CMAKE_SYSTEM_PROCESSOR cortex-r7)
elseif(CONFIG_ARCH_CORTEXR52)
  set(CMAKE_SYSTEM_PROCESSOR cortex-r8)
else()
  message(FATAL_ERROR "CMAKE_SYSTEM_PROCESSOR not set")
endif()

get_directory_property(TOOLCHAIN_DIR_FLAGS DIRECTORY ${CMAKE_SOURCE_DIR}
                                                     COMPILE_OPTIONS)

set(NUTTX_EXTRA_FLAGS "")
foreach(FLAG ${TOOLCHAIN_DIR_FLAGS})
  if(NOT FLAG MATCHES "^\\$<.*>$")
    list(APPEND NUTTX_EXTRA_FLAGS ${FLAG})
  else()
    string(REGEX MATCH "\\$<\\$<COMPILE_LANGUAGE:C>:(.*)>" matched ${FLAG})
    if(matched)
      list(APPEND NUTTX_EXTRA_FLAGS ${CMAKE_MATCH_1})
    endif()
  endif()
endforeach()

separate_arguments(CMAKE_C_FLAG_ARGS NATIVE_COMMAND ${CMAKE_C_FLAGS})

nuttx_find_toolchain_lib()

if(NOT CONFIG_LIBM)
  nuttx_find_toolchain_lib(libm.a)
endif()

if(CONFIG_LIBSUPCXX_TOOLCHAIN)
  nuttx_find_toolchain_lib(libsupc++.a)
endif()

if(CONFIG_COVERAGE_TOOLCHAIN)
  nuttx_find_toolchain_lib(libgcov.a)
endif()
