# ##############################################################################
# cmake/nuttx_generate_outputs.cmake
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

function(nuttx_generate_outputs target)
  if(CONFIG_INTELHEX_BINARY)
    add_custom_command(
      OUTPUT ${target}.hex
      COMMAND ${CMAKE_OBJCOPY} -O ihex ${target} ${target}.hex
      DEPENDS ${target})
    add_custom_target(${target}-hex ALL DEPENDS ${target}.hex)
    file(APPEND ${CMAKE_BINARY_DIR}/nuttx.manifest "${target}.hex\n")
  endif()

  if(CONFIG_MOTOROLA_SREC)
    add_custom_command(
      OUTPUT ${target}.srec
      COMMAND ${CMAKE_OBJCOPY} -O srec ${target} ${target}.srec
      DEPENDS ${target})
    add_custom_target(${target}-srec ALL DEPENDS ${target}.srec)
    file(APPEND ${CMAKE_BINARY_DIR}/nuttx.manifest "${target}.srec\n")
  endif()

  if(CONFIG_RAW_BINARY)
    add_custom_command(
      OUTPUT ${target}.bin
      COMMAND ${CMAKE_OBJCOPY} -O binary ${target} ${target}.bin
      DEPENDS ${target})
    add_custom_target(${target}-bin ALL DEPENDS ${target}.bin)
    file(APPEND ${CMAKE_BINARY_DIR}/nuttx.manifest "${target}.bin\n")
  endif()
endfunction(nuttx_generate_outputs)
