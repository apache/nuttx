# ##############################################################################
# cmake/nuttx_add_dependencies.cmake
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

  foreach(dep ${DEPENDS})
    # add dependencies
    add_dependencies(${TARGET} ${dep})
    # add export include directory
    target_include_directories(${TARGET}
                               PRIVATE ${NUTTX_APPS_BINDIR}/include/${dep})
  endforeach()
endfunction()
