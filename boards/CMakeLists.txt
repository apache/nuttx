# ##############################################################################
# boards/CMakeLists.txt
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

# Determine if there is a Kconfig file for any custom board configuration

# TODO: implement if("${CONFIG_ARCH_BOARD_CUSTOM}" STREQUAL "y") CUSTOM_DIR =
# $(patsubst "%",%,$(CONFIG_ARCH_BOARD_CUSTOM_DIR))
# if("${CONFIG_ARCH_BOARD_CUSTOM_DIR_RELPATH}" STREQUAL "y") CUSTOM_KPATH =
# $(TOPDIR)$(DELIM)$(CUSTOM_DIR)$(DELIM)Kconfig else CUSTOM_KPATH =
# $(CUSTOM_DIR)$(DELIM)Kconfig endif CUSTOM_KCONFIG = $(if $(wildcard
# $(CUSTOM_KPATH)),y,) else CUSTOM_KCONFIG = endif

# ifeq ($(CUSTOM_KCONFIG),y) BOARD_KCONFIG = $(CUSTOM_KPATH) else BOARD_KCONFIG
# = $(TOPDIR)$(DELIM)configs$(DELIM)dummy$(DELIM)dummy_kconfig endif

file(WRITE ${CMAKE_BINARY_DIR}/dummy.c "")
nuttx_add_kernel_library(board ${CMAKE_BINARY_DIR}/dummy.c)

# boardctl support

if(CONFIG_BOARDCTL)
  target_sources(board PRIVATE boardctl.c)
endif()

# obtain include directories exported by libarch
target_include_directories(board
                           PRIVATE $<TARGET_PROPERTY:arch,INCLUDE_DIRECTORIES>)

# add contents of <arch>, <soc> and <board> directories, adding common/ subdirs
# if found at each arch and soc level

if(EXISTS ${NUTTX_BOARD_ABS_DIR}/../common/CMakeLists.txt)
  add_subdirectory("${NUTTX_BOARD_ABS_DIR}/../common" EXCLUDE_FROM_ALL
                   exclude_common)

  # Create link ARCH_SRC_BOARD_BOARD_SYMLINK

  file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/arch/${CONFIG_ARCH}/src/board)
  file(CREATE_LINK ${NUTTX_BOARD_ABS_DIR}/src
       ${CMAKE_BINARY_DIR}/arch/${CONFIG_ARCH}/src/board/board SYMBOLIC)
endif()

if(EXISTS ${NUTTX_BOARD_ABS_DIR}/../drivers/CMakeLists.txt)
  add_subdirectory("${NUTTX_BOARD_ABS_DIR}/../drivers" EXCLUDE_FROM_ALL
                   exclude_drivers)
endif()

add_subdirectory(${NUTTX_BOARD_ABS_DIR} EXCLUDE_FROM_ALL exclude_board)
