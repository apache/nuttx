# ##############################################################################
# cmake/nuttx_add_dependencies.cmake
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

# global dependency maintenance target
add_custom_target(nuttx_dep_map)

include(nuttx_parse_function_args)

# ~~~
# nuttx_add_dependencies
#
# Description:
#   Wrapper of cmake add_dependencies.
#   At the same time, It also allows adding the header file path
#   exported by the dependent library under NUTTX_APPS_BINDIR/include
#
# Usage:
#   nuttx_add_dependencies( TARGET <string> DEPENDS <list>)
#
# Parameters:
#   TARGET            : target needs to add dependencie
#   DEPENDS           : targets which depends on
# ~~~

function(nuttx_add_dependencies)

  # parse arguments into variables

  nuttx_parse_function_args(
    FUNC
    nuttx_add_dependencies
    ONE_VALUE
    TARGET
    MULTI_VALUE
    DEPENDS
    REQUIRED
    TARGET
    DEPENDS
    ARGN
    ${ARGN})

  set_property(
    TARGET nuttx_dep_map
    APPEND
    PROPERTY ALL_PROCESS_TARGET ${TARGET})

  foreach(dep ${DEPENDS})
    set_property(
      TARGET nuttx_dep_map
      APPEND
      PROPERTY ${TARGET} ${dep})
  endforeach()
endfunction()

# After collecting all dependency mappings, process all targets uniformly
function(process_all_target_dependencies)
  # get all target need add deps
  get_property(
    all_process_target
    TARGET nuttx_dep_map
    PROPERTY ALL_PROCESS_TARGET)
  foreach(target ${all_process_target})
    if(TARGET ${target})
      get_target_property(NO_COMPILABLE_TARGET ${target} NO_COMPILABLE_TARGET)
      if(NOT NO_COMPILABLE_TARGET)
        # treat `target` as mapping key
        get_property(
          all_deps
          TARGET nuttx_dep_map
          PROPERTY ${target})
        foreach(dep ${all_deps})
          if(TARGET ${dep})
            # ensure build time order
            add_dependencies(${target} ${dep})
            # inherit public properties
            nuttx_link_libraries(${target} ${dep})
            # compatible with previous export headers
            target_include_directories(
              ${target} PRIVATE ${NUTTX_APPS_BINDIR}/include/${dep})
          endif()
        endforeach()
      endif()
    endif()
  endforeach()
endfunction()
