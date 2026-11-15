# ##############################################################################
# arch/risc-v/src/sbi/rustsbi/CMakeLists.txt
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

add_custom_target(
  rustsbi ALL
  COMMAND
    ${CMAKE_COMMAND} -E env MAKEFLAGS=
    CARGO_TARGET_DIR=${CMAKE_CURRENT_BINARY_DIR}/target cargo prototyper build
    --target ${CONFIG_RUSTSBI_TARGET} --config-file ${CONFIG_RUSTSBI_CONFIG}
    dynamic
  COMMAND
    ${CMAKE_COMMAND} -E copy
    ${CMAKE_CURRENT_BINARY_DIR}/target/${CONFIG_RUSTSBI_TARGET}/release/rustsbi-prototyper-dynamic.elf
    ${NUTTX_BINARY_DIR}/nuttx-rustsbi.elf
  WORKING_DIRECTORY ${NUTTX_DIR}/${CONFIG_RUSTSBI_DIR}
  DEPENDS nuttx
  BYPRODUCTS ${NUTTX_BINARY_DIR}/nuttx-rustsbi.elf
  VERBATIM)

if(EXISTS ${NUTTX_BINARY_DIR}/nuttx.manifest)
  file(STRINGS ${NUTTX_BINARY_DIR}/nuttx.manifest rustsbi_manifest_entry
       REGEX "^nuttx-rustsbi\\.elf$")
endif()
if(NOT rustsbi_manifest_entry)
  file(APPEND ${NUTTX_BINARY_DIR}/nuttx.manifest "nuttx-rustsbi.elf\n")
endif()
