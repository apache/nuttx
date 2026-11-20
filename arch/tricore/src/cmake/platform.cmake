# ##############################################################################
# arch/tricore/src/cmake/platform.cmake
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

nuttx_collect_extra_flags(EXCLUDE "-nostdlib")

separate_arguments(CMAKE_C_FLAG_ARGS NATIVE_COMMAND ${CMAKE_C_FLAGS})

if(CONFIG_TRICORE_TOOLCHAIN_GNU)
  nuttx_find_toolchain_lib()
  if(CONFIG_LIBM_TOOLCHAIN)
    nuttx_find_toolchain_lib(libm.a)
  endif()
  if(CONFIG_LIBSUPCXX_TOOLCHAIN)
    nuttx_find_toolchain_lib(libsupc++.a)
  endif()
  if(CONFIG_LIBCXXTOOLCHAIN)
    nuttx_find_toolchain_lib(libstdc++.a)
  endif()
  if(CONFIG_LIBMINIABI)
    list(APPEND CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES ${NUTTX_DIR}/include/cxx)
  endif()
  if(CONFIG_COVERAGE_TOOLCHAIN)
    nuttx_find_toolchain_lib(libgcov.a)
  endif()

  set(PREPROCESS ${CMAKE_C_COMPILER} ${CMAKE_C_FLAG_ARGS} -E -P -x c)
endif()

if(CONFIG_TRICORE_TOOLCHAIN_TASKING)
  find_program(TASKING_COMPILER_PATH ctc)
  get_filename_component(C_COMPILER_DIR ${TASKING_COMPILER_PATH} DIRECTORY)
  set(PREPROCESS
      ${TASKING_COMPILER_PATH} ${CMAKE_C_FLAG_ARGS} -E
      -I${TOOLCHAIN_PREFIX}/../include.lsl -I${C_COMPILER_DIR}/../include.lsl)
  if(CONFIG_LIBM_TOOLCHAIN)
    nuttx_find_toolchain_lib(libm.a)
  endif()
endif()
