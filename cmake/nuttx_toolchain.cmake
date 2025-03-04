# ##############################################################################
# cmake/nuttx_toolchain.cmake
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

# This file is used to set the parts that need common settings in the Toolchain
# file. Such as preprocessing process, link command, toolchain library method
# search. If the manual of the newly supported toolchain is different, you can
# override these methods in the toolchain

if("${CMAKE_C_COMPILER_ID}" STREQUAL "GNU")
  if(CMAKE_C_COMPILER_VERSION VERSION_GREATER 4.9)
    # force color for gcc > 4.9
    add_compile_options(-fdiagnostics-color=always)
  endif()
endif()

# Support CMake to define additional configuration options

if(EXTRA_FLAGS)
  add_compile_options(${EXTRA_FLAGS})
endif()

# ~~~
# nuttx_generate_preprocess_target
#
# Description:
#   because different toolchains have different preprocessing instructions,
#   we define the COMMON preprocessing target here.
#
# Prototype:
#  nuttx_generate_preprocess_target(
#    SOURCE_FILE
#    ${single_source_file}
#    TARGET_FILE
#    ${single_target_output}
#    DEPENDS
#    ${option_depned_target})
# ~~~

#
if(NOT NUTTX_TOOLCHAIN_PREPROCESS_DEFINED)
  function(nuttx_generate_preprocess_target)

    # parse arguments into variables

    nuttx_parse_function_args(
      FUNC
      nuttx_generate_preprocess_target
      ONE_VALUE
      SOURCE_FILE
      TARGET_FILE
      MULTI_VALUE
      DEPENDS
      REQUIRED
      SOURCE_FILE
      TARGET_FILE
      ARGN
      ${ARGN})

    add_custom_command(
      OUTPUT ${TARGET_FILE}
      COMMAND ${PREPROCESS} -I${CMAKE_BINARY_DIR}/include
              -I${NUTTX_CHIP_ABS_DIR} ${SOURCE_FILE} > ${TARGET_FILE}
      DEPENDS ${SOURCE_FILE} ${DEPENDS})

  endfunction()
endif()

# ~~~
# nuttx_find_toolchain_lib
#
# Description:
#   this is general function for finding toolchain libraries.
#
# Prototype:
#  nuttx_find_toolchain_lib(${single_toolchain_lib})
# ~~~

if(NOT NUTTX_FIND_TOOLCHAIN_LIB_DEFINED)
  function(nuttx_find_toolchain_lib)
    if(NOT ARGN)
      execute_process(
        COMMAND ${CMAKE_C_COMPILER} ${CMAKE_C_FLAG_ARGS} ${NUTTX_EXTRA_FLAGS}
                --print-libgcc-file-name
        OUTPUT_STRIP_TRAILING_WHITESPACE
        OUTPUT_VARIABLE extra_lib_path)
    else()
      execute_process(
        COMMAND ${CMAKE_C_COMPILER} ${CMAKE_C_FLAG_ARGS} ${NUTTX_EXTRA_FLAGS}
                --print-file-name=${ARGN}
        OUTPUT_STRIP_TRAILING_WHITESPACE
        OUTPUT_VARIABLE extra_lib_path)
    endif()
    nuttx_add_extra_library(${extra_lib_path})
  endfunction()
endif()
