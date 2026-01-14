# ##############################################################################
# cmake/nuttx_add_subdirectory.cmake
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

function(nuttx_add_subdirectory)
  # Parse arguments: EXCLUDE can be a list of directories
  set(options)
  set(oneValueArgs)
  set(multiValueArgs EXCLUDE)
  cmake_parse_arguments(NUTTX "${options}" "${oneValueArgs}"
                        "${multiValueArgs}" ${ARGN})

  # Find all subdirs that have a CMakeLists.txt
  file(
    GLOB subdir
    LIST_DIRECTORIES false
    RELATIVE ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/*/CMakeLists.txt)

  foreach(dir ${subdir})
    get_filename_component(dir ${dir} DIRECTORY)

    # Skip excluded directories
    list(FIND NUTTX_EXCLUDE ${dir} _skip_index)
    if(_skip_index GREATER -1)
      message(STATUS "nuttx_add_subdirectory: Skipping ${dir}")
      continue()
    endif()

    add_subdirectory(${dir})
  endforeach()
endfunction()
