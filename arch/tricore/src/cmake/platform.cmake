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

get_directory_property(TOOLCHAIN_DIR_FLAGS DIRECTORY ${NUTTX_DIR}
                                                     COMPILE_OPTIONS)

set(NUTTX_EXTRA_FLAGS "")
foreach(FLAG ${TOOLCHAIN_DIR_FLAGS})
  if(NOT FLAG MATCHES "^\\$<.*>$")
    if(NOT FLAG MATCHES "-nostdlib")
      list(APPEND NUTTX_EXTRA_FLAGS ${FLAG})
    endif()
  else()
    string(REGEX MATCH "\\$<\\$<COMPILE_LANGUAGE:C>:(.*)>" matched ${FLAG})
    if(matched)
      list(APPEND NUTTX_EXTRA_FLAGS ${CMAKE_MATCH_1})
    endif()
  endif()
endforeach()

if(NUTTX_EXTRA_FLAGS)
  list(REMOVE_DUPLICATES NUTTX_EXTRA_FLAGS)
endif()

separate_arguments(CMAKE_C_FLAG_ARGS NATIVE_COMMAND ${CMAKE_C_FLAGS})

if(CONFIG_TRICORE_TOOLCHAIN_GNUC)
  nuttx_find_toolchain_lib()
  if(CONFIG_LIBM_TOOLCHAIN)
    nuttx_find_toolchain_lib(libm.a)
  endif()
  if(CONFIG_LIBSUPCXX)
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
endif()

if(CONFIG_TRICORE_TOOLCHAIN_TASKING)
  find_program(TASKING_CTC ctc)
  get_filename_component(TASKING_BIN_DIR ${TASKING_CTC} DIRECTORY)
  set(PREPROCESS ${TASKING_CTC} -Ep -I${TASKING_BIN_DIR}/../include.lsl)

  # astc has no C preprocessor, the .asm files go through ctc first for the
  # headers and options.  rfh and the VCON0 write of the TC1.8 path need the
  # hypervisor mode.

  set(TASKING_AFLAGS -Wa--no-warnings=201)
  if(CONFIG_TRICORE_TC18)
    list(APPEND TASKING_AFLAGS -Wa--user-mode=hypervisor)
  endif()

  string(JOIN " " TASKING_ASM_FLAGS ${PLATFORM_FLAGS} ${TASKING_AFLAGS})
  set(CMAKE_DEPFILE_FLAGS_ASM "")
  set(CMAKE_ASM_COMPILE_OBJECT
      "${TASKING_CTC} -Ep -D__ASSEMBLY__ <DEFINES> <INCLUDES> <SOURCE> -o <OBJECT>.src"
      "<CMAKE_ASM_COMPILER> -c ${TASKING_ASM_FLAGS} <OBJECT>.src -o <OBJECT>")
else()
  set(PREPROCESS ${CMAKE_C_COMPILER} ${CMAKE_C_FLAG_ARGS} -E -P -x c)
endif()
