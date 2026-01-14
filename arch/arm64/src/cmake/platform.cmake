# ##############################################################################
# arch/arm64/src/cmake/platform.cmake
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

get_directory_property(TOOLCHAIN_DIR_FLAGS DIRECTORY ${CMAKE_SOURCE_DIR}
                                                     COMPILE_OPTIONS)

set(CMAKE_C_FLAG_ARGS)
foreach(FLAG ${TOOLCHAIN_DIR_FLAGS})
  if(NOT FLAG MATCHES "^\\$<.*>$")
    list(APPEND CMAKE_C_FLAG_ARGS ${FLAG})
  else()
    string(REGEX MATCH "\\$<\\$<COMPILE_LANGUAGE:C>:(.*)>" matched ${FLAG})
    if(matched)
      list(APPEND CMAKE_C_FLAG_ARGS ${CMAKE_MATCH_1})
    endif()
  endif()
endforeach()

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
