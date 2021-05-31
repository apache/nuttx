############################################################################
# configs/CMakeLists.txt
#
#   Copyright (C) 2015-2016 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

# Determine if there is a Kconfig file for any custom board configuration

# TODO: implement
#if("${CONFIG_ARCH_BOARD_CUSTOM}" STREQUAL "y")
  #CUSTOM_DIR = $(patsubst "%",%,$(CONFIG_ARCH_BOARD_CUSTOM_DIR))
#  if("${CONFIG_ARCH_BOARD_CUSTOM_DIR_RELPATH}" STREQUAL "y")
    #CUSTOM_KPATH = $(TOPDIR)$(DELIM)$(CUSTOM_DIR)$(DELIM)Kconfig
#  else
#    CUSTOM_KPATH = $(CUSTOM_DIR)$(DELIM)Kconfig
#  endif
#  CUSTOM_KCONFIG = $(if $(wildcard $(CUSTOM_KPATH)),y,)
#else
#  CUSTOM_KCONFIG =
#endif

#ifeq ($(CUSTOM_KCONFIG),y)
#  BOARD_KCONFIG = $(CUSTOM_KPATH)
#else
#  BOARD_KCONFIG = $(TOPDIR)$(DELIM)configs$(DELIM)dummy$(DELIM)dummy_kconfig
#endif

file(WRITE ${CMAKE_BINARY_DIR}/empty.c "")
nuttx_add_kernel_library(board ${CMAKE_BINARY_DIR}/empty.c)

# boardctl support

if(CONFIG_LIB_BOARDCTL)
	target_sources(board PRIVATE boardctl.c)
endif()

# obtain include directories exported by libarch
target_include_directories(board PRIVATE $<TARGET_PROPERTY:arch,INCLUDE_DIRECTORIES>)
  
# add contents of <arch>, <soc> and <board> directories, adding common/ subdirs
# if found at each arch and soc level

add_subdirectory(${CONFIG_ARCH})
if (EXISTS ${CMAKE_CURRENT_LIST_DIR}/${CONFIG_ARCH}/common/CMakeLists.txt)
  add_subdirectory(${CONFIG_ARCH}/common)
endif()

add_subdirectory(${CONFIG_ARCH}/${CONFIG_ARCH_CHIP})
if (EXISTS ${CMAKE_CURRENT_LIST_DIR}/${CONFIG_ARCH}/${CONFIG_ARCH_CHIP}/common/CMakeLists.txt)
  add_subdirectory(${CONFIG_ARCH}/${CONFIG_ARCH_CHIP}/common)
endif()

get_filename_component(BOARD_NAME ${NUTTX_BOARD} NAME)
add_subdirectory(${CONFIG_ARCH}/${CONFIG_ARCH_CHIP}/${BOARD_NAME})
