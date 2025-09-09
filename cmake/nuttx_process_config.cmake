# ##############################################################################
# cmake/nuttx_process_config.cmake
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

# save preprocess defconfig as orig by default
set(NUTTX_ORIG_DEFCONFIG ${NUTTX_DEFCONFIG})

function(process_config OUTPUT INPUT TREE_FILE)
  set(options)
  set(oneValueArgs)
  set(multiValueArgs INCLUDE_PATHS)
  cmake_parse_arguments(PARSE_ARGV 2 PROCESS_INCLUDES "${options}"
                        "${oneValueArgs}" "${multiValueArgs}")

  set(include_args "")
  foreach(path IN LISTS PROCESS_INCLUDES_INCLUDE_PATHS)
    list(APPEND include_args "${path}")
  endforeach()

  if(TREE_FILE)
    set(TREE_OPTION --tree)
  endif()
  message(STATUS "Processing includes: ${INPUT} → ${OUTPUT}")
  execute_process(
    COMMAND
      ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/process_config.py
      preprocess ${OUTPUT} ${INPUT} ${include_args} ${TREE_OPTION} ${TREE_FILE}
    RESULT_VARIABLE result
    OUTPUT_VARIABLE out
    ERROR_VARIABLE err)

  if(NOT result EQUAL 0)
    message(FATAL_ERROR "Failed to process includes:\n${err}")
  endif()
endfunction()

# fetch defconfig content
file(READ "${NUTTX_DEFCONFIG}" FILE_CONTENTS)
string(FIND "${FILE_CONTENTS}" "#include" INCLUDE_FOUND)

if(NOT EXISTS ${CMAKE_BINARY_DIR}/.defconfig.processed)
  set(TREE_FILE ${CMAKE_BINARY_DIR}/config_tree.json)
else()
  set(TREE_FILE ${CMAKE_BINARY_DIR}/config_tree_dirty.json)
endif()
# Should we preprocess defconfig?
if(INCLUDE_FOUND GREATER -1)
  get_filename_component(NUTTX_DEFCONFIG_DIR "${NUTTX_DEFCONFIG}" DIRECTORY)
  process_config(
    ${CMAKE_BINARY_DIR}/.defconfig.processed
    ${NUTTX_DEFCONFIG}
    ${TREE_FILE}
    INCLUDE_PATHS
    ${NUTTX_DEFCONFIG_DIR}/../../common/configs
    ${NUTTX_DEFCONFIG_DIR}/../common
    ${NUTTX_DEFCONFIG_DIR}
    ${NUTTX_DIR}/../apps
    ${NUTTX_DIR}/../nuttx-apps)
  set(NUTTX_DEFCONFIG ${CMAKE_BINARY_DIR}/.defconfig.processed)
endif()
