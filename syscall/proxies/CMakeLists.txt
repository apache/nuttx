# ##############################################################################
# syscall/proxies/CMakeLists.txt
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
nuttx_add_system_library(proxies)

list(TRANSFORM SYSCALLS PREPEND "${CMAKE_CURRENT_BINARY_DIR}/PROXY_")

# generate files
add_custom_command(
  OUTPUT ${SYSCALLS}
  COMMAND ${CMAKE_BINARY_DIR}/bin/mksyscall -p
          ${CMAKE_CURRENT_LIST_DIR}/../syscall.csv
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  DEPENDS mksyscall)

# add sources to target
target_sources(proxies PRIVATE ${SYSCALLS})
