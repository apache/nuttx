# ##############################################################################
# arch/arm64/src/cmake/platform.cmake
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

execute_process(
  COMMAND ${CMAKE_C_COMPILER} ${CMAKE_C_FLAG_ARGS} ${NUTTX_EXTRA_FLAGS}
          --print-libgcc-file-name
  OUTPUT_STRIP_TRAILING_WHITESPACE
  OUTPUT_VARIABLE extra_library)
list(APPEND EXTRA_LIB ${extra_library})
if(NOT CONFIG_LIBM)
  execute_process(
    COMMAND ${CMAKE_C_COMPILER} ${CMAKE_C_FLAG_ARGS} ${NUTTX_EXTRA_FLAGS}
            --print-file-name=libm.a
    OUTPUT_STRIP_TRAILING_WHITESPACE
    OUTPUT_VARIABLE extra_library)
  list(APPEND EXTRA_LIB ${extra_library})
endif()
if(CONFIG_LIBSUPCXX_TOOLCHAIN)
  execute_process(
    COMMAND ${CMAKE_C_COMPILER} ${CMAKE_C_FLAG_ARGS} ${NUTTX_EXTRA_FLAGS}
            --print-file-name=libsupc++.a
    OUTPUT_STRIP_TRAILING_WHITESPACE
    OUTPUT_VARIABLE extra_library)
  list(APPEND EXTRA_LIB ${extra_library})
endif()
if(CONFIG_COVERAGE_TOOLCHAIN)
  execute_process(
    COMMAND ${CMAKE_C_COMPILER} ${CMAKE_C_FLAG_ARGS} ${NUTTX_EXTRA_FLAGS}
            --print-file-name=libgcov.a
    OUTPUT_STRIP_TRAILING_WHITESPACE
    OUTPUT_VARIABLE extra_library)
  list(APPEND EXTRA_LIB ${extra_library})
endif()

nuttx_add_extra_library(${EXTRA_LIB})

set(PREPROCESS ${CMAKE_C_COMPILER} ${CMAKE_C_FLAG_ARGS} -E -P -x c)
