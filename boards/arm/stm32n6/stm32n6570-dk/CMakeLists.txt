# ##############################################################################
# boards/arm/stm32n6/stm32n6570-dk/CMakeLists.txt
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

add_subdirectory(src)

if(CONFIG_RAW_BINARY)
  set(OPENOCD
      openocd
      CACHE STRING "OpenOCD executable")
  set(OPENOCD_TARGET
      target/stm32n6x.cfg
      CACHE STRING "OpenOCD target script")

  add_custom_target(
    flash
    COMMAND
      ${OPENOCD} -f interface/stlink.cfg -c "transport select swd" -f
      ${OPENOCD_TARGET} -f ${CMAKE_CURRENT_LIST_DIR}/scripts/flash.cfg -c
      "stm32n6_program nuttx.bin"
    WORKING_DIRECTORY ${NUTTX_BINARY_DIR}
    USES_TERMINAL VERBATIM)
  add_dependencies(flash nuttx-bin)
endif()
