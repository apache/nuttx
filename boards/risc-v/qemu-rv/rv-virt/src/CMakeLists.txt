# ##############################################################################
# boards/risc-v/qemu-rv/rv-virt/src/CMakeLists.txt
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

set(SRCS qemu_rv_appinit.c)

target_sources(board PRIVATE ${SRCS})

if(CONFIG_ARCH_CHIP_QEMU_RV)

  if(CONFIG_BUILD_KERNEL)
    if(CONFIG_ARCH_CHIP_QEMU_RV64)
      set(LDFILE ld-kernel64.script)
    else()
      set(LDFILE ld-kernel32.script)
    endif()
  else()
    set(LDFILE ld.script)
  endif()
endif()

set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/${LDFILE}")
