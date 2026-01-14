# ##############################################################################
# cmake/nuttx_parse_function_args.cmake
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

# =============================================================================
#
# Defined functions in this file
#
# utility functions
#
# * nuttx_parse_function_args
#

include(CMakeParseArguments)

# ~~~
# nuttx_parse_function_args
#
# This function simplifies usage of the cmake_parse_arguments module. It is
# intended to be called by other functions.
#
# Usage:
#   nuttx_parse_function_args( FUNC <name> [ OPTIONS <list> ]
#    [ ONE_VALUE <list> ] [ MULTI_VALUE <list> ] REQUIRED <list> ARGN <ARGN>)
#
# Input:
#   FUNC        : the name of the calling function
#   OPTIONS     : boolean flags
#   ONE_VALUE   : single value variables
#   MULTI_VALUE : multi value variables
#   REQUIRED    : required arguments
#   ARGN        : the function input arguments, typically ${ARGN}
#
# Output:
#   The function arguments corresponding to the following are set:
#     ${OPTIONS}, ${ONE_VALUE}, ${MULTI_VALUE}
#
# Example:
#  function test()
#    nuttx_parse_function_args(FUNC TEST ONE_VALUE NAME MULTI_VALUE
#                              LIST REQUIRED NAME LIST ARGN ${ARGN})
#    message(STATUS "name: ${NAME}")
#    message(STATUS "list: ${LIST}")
#  endfunction()
#
# test(NAME "hello" LIST a b c)
#
# OUTPUT: name: hello list: a b c
# ~~~
function(nuttx_parse_function_args)

  cmake_parse_arguments(IN "" "FUNC"
                        "OPTIONS;ONE_VALUE;MULTI_VALUE;REQUIRED;ARGN" "${ARGN}")
  cmake_parse_arguments(OUT "${IN_OPTIONS}" "${IN_ONE_VALUE}"
                        "${IN_MULTI_VALUE}" "${IN_ARGN}")

  if(OUT_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "${IN_NAME}: unparsed ${OUT_UNPARSED_ARGUMENTS}")
  endif()

  foreach(arg ${IN_REQUIRED})
    if(NOT OUT_${arg})
      if(NOT "${OUT_${arg}}" STREQUAL "0")
        message(
          FATAL_ERROR "${IN_NAME} requires argument ${arg}\nARGN: ${IN_ARGN}")
      endif()
    endif()
  endforeach()

  foreach(arg ${IN_OPTIONS} ${IN_ONE_VALUE} ${IN_MULTI_VALUE})
    set(${arg}
        ${OUT_${arg}}
        PARENT_SCOPE)
  endforeach()

endfunction()
