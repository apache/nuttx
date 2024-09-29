# ##############################################################################
# arch/CMakeLists.txt
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
# Declare arch library. In contrast to other user/kernel pairs built in KERNEL
# mode, on arch/ the "user" portion uses a different set of sources than the
# kernel portion. To keep things simple the naming k<lib> is not used. Instead,
# the "user" portion is named "arch_interface".

nuttx_add_kernel_library(arch)
target_include_directories(arch PRIVATE ${CMAKE_SOURCE_DIR}/sched)

if(CONFIG_BUILD_PROTECTED)
  nuttx_add_system_library(arch_interface)
  file(TOUCH "${CMAKE_CURRENT_BINARY_DIR}/empty.c")
  target_sources(arch_interface PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/empty.c")
  target_include_directories(arch_interface PRIVATE ${CMAKE_SOURCE_DIR}/sched)
endif()

# TODO: move this higher up ifeq ($(CONFIG_SCHED_INSTRUMENTATION_SYSCALL),y)
# EXTRALINKCMDS += @$(TOPDIR)/syscall/syscall_wraps.ldcmd endif

# include corresponding arch subdirectory
add_subdirectory(${CONFIG_ARCH})
