# ##############################################################################
# cmake/nuttx_remove_compile_options.cmake
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

# ~~~
# nuttx_remove_compile_options
#
# Description:
#  Customize to remove certain options
#  in the default Arch ToolchainFile.
#
#   For example, a customized chip board uses a customized tool chain, which has compilation configuration conflicts with default.
#   You can use this macro to remove it in the global compile options.
#   This function is similar to the reverse operation of `add_compile_options()`
#
# Usage:
#   nuttx_remove_compile_options(ARGNS)
#
# Parameters:
#   ARGNS                : options regex that needs to be deleted
#
# Example:
#   nuttx_remove_compile_options(-march -mabi)
#
#   befor: CFLAGS = -O2 -g -march=rv32if -mabi=ilp32f -mcpu=e907fp
#   after: CFLAGS = -O2 -g -mcpu=e907fp
#
# ~~~

function(nuttx_remove_compile_options)
  get_property(
    CURRENT_OPTIONS
    DIRECTORY
    PROPERTY COMPILE_OPTIONS)

  set_property(DIRECTORY PROPERTY COMPILE_OPTIONS)

  # Remove flags starting with regex
  foreach(CURRENT_OPTION_VARIABLE ${CURRENT_OPTIONS})
    set(MATCHED_FLAG)
    foreach(regex ${ARGN})
      string(REGEX MATCH "^${regex}" MATCHED ${CURRENT_OPTION_VARIABLE})
      if(MATCHED)
        string(CONCAT MATCHED_FLAG ${MATCHED_FLAG}${MATCHED})
      endif()
    endforeach()
    if(NOT MATCHED_FLAG)
      add_compile_options(${CURRENT_OPTION_VARIABLE})
    endif()
  endforeach()
endfunction()
