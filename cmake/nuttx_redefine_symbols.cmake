# ##############################################################################
# cmake/nuttx_redefine_symbols.cmake
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

set(REDEFINE_INPUT_FILE ${NUTTX_DIR}/arch/sim/src/nuttx-names.in)
set(REDEFINE_OUTPUT_FILE ${CMAKE_BINARY_DIR}/nuttx-names.dat)

add_custom_command(
  OUTPUT ${REDEFINE_OUTPUT_FILE}
  COMMAND ${CMAKE_C_COMPILER} -E -P -x c -I${CMAKE_BINARY_DIR}/include
          ${REDEFINE_INPUT_FILE} > ${REDEFINE_OUTPUT_FILE}
  DEPENDS nuttx_context ${REDEFINE_INPUT_FILE})

add_custom_target(sim_redefine_symbols DEPENDS ${REDEFINE_OUTPUT_FILE})
