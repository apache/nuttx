# ##############################################################################
# arch/x86_64/src/cmake/platform.cmake
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

if(CONFIG_LIBM_TOOLCHAIN)
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

if(CONFIG_CXX_EXCEPTION)
  execute_process(
    COMMAND ${CMAKE_C_COMPILER} ${CMAKE_C_FLAG_ARGS} ${NUTTX_EXTRA_FLAGS}
            --print-file-name=libgcc_eh.a
    OUTPUT_STRIP_TRAILING_WHITESPACE
    OUTPUT_VARIABLE extra_library)
  list(APPEND EXTRA_LIB ${extra_library})
endif()

nuttx_add_extra_library(${EXTRA_LIB})

set(PREPROCESS ${CMAKE_C_COMPILER} ${CMAKE_C_FLAG_ARGS} -E -P -x c)

if(CONFIG_ARCH_MULTIBOOT1)
  message(STATUS "Generating: nuttx.mb1 in ELF32/multiboot1")
  if(CONFIG_ALLSYMS)
    set(FINAL_NUTTX_ELF "${CMAKE_BINARY_DIR}/final_nuttx")
  else()
    set(FINAL_NUTTX_ELF "${CMAKE_BINARY_DIR}/nuttx")
  endif()
  set(NUTTX_ELF "${CMAKE_BINARY_DIR}/nuttx")
  set(NUTTX_BIN "${NUTTX_ELF}.bin")
  set(NUTTX_REALMODE_BIN "${NUTTX_ELF}_realmode.bin")
  set(NUTTX_MB1 "${NUTTX_ELF}.mb1")
  add_custom_command(
    OUTPUT ${NUTTX_BIN} ${NUTTX_REALMODE_BIN}
    COMMAND ${CMAKE_OBJCOPY} -R .realmode -R .note.* -O binary
            ${FINAL_NUTTX_ELF} ${NUTTX_BIN}
    COMMAND ${CMAKE_OBJCOPY} -j .realmode -O binary ${FINAL_NUTTX_ELF}
            ${NUTTX_REALMODE_BIN}
    DEPENDS ${FINAL_NUTTX_ELF}
    COMMENT "Generating binary and realmode segments from nuttx ELF")
  add_custom_command(
    OUTPUT ${NUTTX_MB1}
    COMMAND
      ${CMAKE_C_COMPILER} -m32 -no-pie -nostdlib -DNUTTX_BIN='"${NUTTX_BIN}"'
      -DNUTTX_REALMODE_BIN='"${NUTTX_REALMODE_BIN}"'
      ${CMAKE_SOURCE_DIR}/arch/x86_64/src/common/multiboot1.S -T
      ${CMAKE_SOURCE_DIR}/arch/x86_64/src/common/multiboot1.ld -o ${NUTTX_MB1}
    DEPENDS ${NUTTX_BIN} ${NUTTX_REALMODE_BIN}
    COMMENT "Building nuttx.mb1 multiboot1 image")
  add_custom_target(multiboot1 ALL DEPENDS ${NUTTX_MB1})
endif()
