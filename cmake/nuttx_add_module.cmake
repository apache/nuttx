# ##############################################################################
# cmake/nuttx_add_module.cmake
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

function(nuttx_add_module target)
  # Use ELF capable toolchain, by building manually and overwriting the non-elf
  # output
  if(CMAKE_C_ELF_COMPILER)
    add_library(${target} ${ARGN})

    add_custom_command(
      TARGET ${target}
      POST_BUILD
      COMMAND
        ${CMAKE_C_ELF_COMPILER}
        $<TARGET_PROPERTY:nuttx,NUTTX_ELF_MODULE_LINK_OPTIONS>
        $<TARGET_FILE:${target}> -o ${target}
      COMMAND_EXPAND_LISTS)
  else()
    add_executable(${target} ${ARGN})
    target_link_options(
      ${target} PRIVATE
      $<GENEX_EVAL:$<TARGET_PROPERTY:nuttx,NUTTX_ELF_MODULE_LINK_OPTIONS>>)
  endif()

  set_property(TARGET ${target} PROPERTY ELF_BINARY
                                         ${CMAKE_CURRENT_BINARY_DIR}/${target})

  nuttx_add_library_internal(${target})

  target_compile_options(
    ${target}
    PRIVATE
      $<GENEX_EVAL:$<TARGET_PROPERTY:nuttx,NUTTX_ELF_MODULE_COMPILE_OPTIONS>>)
  target_compile_definitions(
    ${target}
    PRIVATE $<GENEX_EVAL:$<TARGET_PROPERTY:nuttx,NUTTX_ELF_MODULE_DEFINITIONS>>
            $<GENEX_EVAL:$<TARGET_PROPERTY:nuttx,NUTTX_KERNEL_DEFINITIONS>>)

  install(TARGETS ${target})
endfunction()
