# ##############################################################################
# boards/arm/rp2040/raspberrypi-pico-w/CMakeLists.txt
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

add_custom_target(
  nuttx_post_build
  DEPENDS nuttx
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  COMMENT "Regenerate nuttx.uf2")

# The uf2 command to convert ELF/BIN to UF2
add_custom_command(
  TARGET nuttx_post_build
  POST_BUILD
  COMMAND picotool ARGS uf2 convert --quiet -t elf nuttx nuttx.uf2
  COMMAND_EXPAND_LISTS
  COMMAND ${CMAKE_COMMAND} -E echo "nuttx.uf2" >>
          ${CMAKE_BINARY_DIR}/nuttx.manifest
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  COMMENT "Regenerate nuttx.uf2")
