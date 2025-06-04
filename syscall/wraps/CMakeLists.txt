# ##############################################################################
# syscall/wraps/CMakeLists.txt
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
nuttx_add_kernel_library(wraps)

list(TRANSFORM SYSCALLS PREPEND "${CMAKE_CURRENT_BINARY_DIR}/WRAP_")

# generate files
add_custom_command(
  OUTPUT ${SYSCALLS}
  COMMAND ${NUTTX_BINARY_DIR}/bin_host/mksyscall -w
          ${CMAKE_CURRENT_LIST_DIR}/../syscall.csv
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  DEPENDS mksyscall)

# On sim the final link also contains the host objects, which must keep calling
# the host libc. The wraps would have to be applied to nuttx.rel only, so skip
# them there.

if(NOT CONFIG_ARCH_SIM)
  set(WRAP_LD_CMD "${CMAKE_BINARY_DIR}/syscall_wraps.ldcmd")

  add_custom_command(
    OUTPUT ${WRAP_LD_CMD}
    COMMAND
      ${PREPROCESS} -I${CMAKE_BINARY_DIR}/include -I${NUTTX_DIR}/include
      -I${NUTTX_CHIP_ABS_DIR} ${CMAKE_CURRENT_LIST_DIR}/../syscall_wraps.h >
      ${WRAP_LD_CMD}tmp
    COMMAND
      bash -c
      "sed -e '1,/WRAPOPTSTARTS/d' -e '/^#/d' -e 's/--wrap /-Wl,--wrap=/g' ${WRAP_LD_CMD}tmp > ${WRAP_LD_CMD}"
    DEPENDS ${CMAKE_CURRENT_LIST_DIR}/../syscall_wraps.h
    VERBATIM)

  add_custom_target(wrap_ld_cmd DEPENDS ${WRAP_LD_CMD})
  add_dependencies(nuttx wrap_ld_cmd)

  target_link_options(nuttx PRIVATE "@${WRAP_LD_CMD}")
endif()

# add sources to target
target_sources(wraps PRIVATE ${SYSCALLS})
