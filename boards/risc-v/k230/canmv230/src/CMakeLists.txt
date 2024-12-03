# ##############################################################################
# boards/risc-v/k230/canmv230/src/CMakeLists.txt
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

set(SRCS canmv_init.c)

if(CONFIG_BUILD_KERNEL)
  if(EXISTS ${CMAKE_BINARY_DIR}/romfs_boot.c)
    list(APPEND SRCS ${CMAKE_BINARY_DIR}/romfs_boot.c)
  else()
    list(APPEND SRCS romfs_stub.c)
  endif()
endif()

target_sources(board PRIVATE ${SRCS})

if(CONFIG_ARCH_CHIP_K230)
  if(CONFIG_BUILD_KERNEL)
    if(CONFIG_NUTTSBI)
      set(LDFILE ld-nuttsbi.script)
    else()
      set(LDFILE ld-kernel.script)
    endif()
  elseif(CONFIG_BUILD_PROTECTED)
    set(LDFILE ld-protected.script)
  else()
    set(LDFILE ld-flat.script)
  endif()
endif()

set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/${LDFILE}")
