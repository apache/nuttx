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

function(process_config OUTPUT INPUT)
  set(options)
  set(oneValueArgs)
  set(multiValueArgs INCLUDE_PATHS)
  cmake_parse_arguments(PARSE_ARGV 2 PROCESS_INCLUDES "${options}"
                        "${oneValueArgs}" "${multiValueArgs}")

  set(include_args "")
  foreach(path IN LISTS PROCESS_INCLUDES_INCLUDE_PATHS)
    list(APPEND include_args "${path}")
  endforeach()

  message(STATUS "Processing includes: ${INPUT} -> ${OUTPUT}")
  execute_process(
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/process_config.py
            ${OUTPUT} ${INPUT} ${include_args}
    RESULT_VARIABLE result
    OUTPUT_VARIABLE out
    ERROR_VARIABLE err)

  if(NOT result EQUAL 0)
    message(FATAL_ERROR "Failed to process includes:\n${err}")
  endif()
endfunction()
