# ##############################################################################
# boards/x86_64/qemu/qemu-intel64/src/CMakeLists.txt
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

set(SRCS qemu_boot.c qemu_bringup.c qemu_net.c qemu_boardinit.c)

if(CONFIG_BOARDCTL_RESET)
  list(APPEND SRCS qemu_reset.c)
endif()

if(CONFIG_BUILD_KERNEL)
  # Pack the standalone application binaries into the ROMFS image registered by
  # qemu_boardinit.c. The application targets are not known yet when this
  # directory is configured, so the dependency is resolved at generation time
  # from the nuttx target property. The nuttx_add_romfs helper cannot be used
  # here: its library is added to apps_post, which the application targets
  # depend on (cycle).
  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/romfs_boot.c
    COMMAND genromfs -f romfs.img -d ${NUTTX_BINARY_DIR}/bin -V NuttXBootVol
    COMMAND xxd -i romfs.img romfs_boot.c
    DEPENDS $<TARGET_PROPERTY:nuttx,NUTTX_LOADABLE_APPS>
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  # the board target is defined in a parent directory: anchor the generated
  # source with a local target so the build rule is emitted
  add_custom_target(board_romfs
                    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/romfs_boot.c)
  add_dependencies(board board_romfs)
  list(APPEND SRCS ${CMAKE_CURRENT_BINARY_DIR}/romfs_boot.c)
elseif(CONFIG_BOARD_LATE_INITIALIZE)
  # empty ROMFS placeholder, see Makefile
  list(APPEND SRCS romfs_stub.c)
endif()

target_sources(board PRIVATE ${SRCS})

set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/qemu.ld")
