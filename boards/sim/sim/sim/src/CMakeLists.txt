# ##############################################################################
# boards/sim/sim/sim/src/CMakeLists.txt
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

# CSRCS = dummy.c

if(CONFIG_BOARD_LATE_INITIALIZE)
  list(APPEND SRCS sim_boot.c)
endif()

if(CONFIG_BOARDCTL)
  list(APPEND SRCS sim_appinit.c)
endif()

if(CONFIG_BOARD_LATE_INITIALIZE)
  list(APPEND SRCS sim_bringup.c)
  if(CONFIG_LIBC_ZONEINFO_ROMFS)
    list(APPEND SRCS sim_zoneinfo.c)
  endif()
elseif(CONFIG_BOARDCTL)
  list(APPEND SRCS sim_bringup.c)
  if(CONFIG_LIBC_ZONEINFO_ROMFS)
    list(APPEND SRCS sim_zoneinfo.c)
  endif()
endif()

if(CONFIG_EXAMPLES_GPIO)
  if(CONFIG_GPIO_LOWER_HALF)
    list(APPEND SRCS sim_ioexpander.c)
  else()
    list(APPEND SRCS sim_gpio.c)
  endif()
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS sim_buttons.c)
endif()

if(CONFIG_MOTOR_FOC_DUMMY)
  list(APPEND SRCS sim_foc.c)
endif()

if(CONFIG_USBDEV_COMPOSITE)
  list(APPEND SRCS sim_composite.c)
endif()

target_sources(board PRIVATE ${SRCS})

set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/gnu-elf.ld")

if(CONFIG_ETC_ROMFS)
  nuttx_add_romfs(
    NAME
    etc
    MOUNTPOINT
    etc
    RCSRCS
    etc/init.d/rcS
    etc/init.d/rc.sysinit
    RCRAWS
    etc/group
    etc/passwd
    PATH
    ${CMAKE_CURRENT_BINARY_DIR}/etc)

  target_link_libraries(board PRIVATE romfs_etc)
endif()
