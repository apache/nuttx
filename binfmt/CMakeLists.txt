# ##############################################################################
# binfmt/CMakeLists.txt
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

set(SRCS)

nuttx_add_kernel_library(binfmt)

nuttx_add_subdirectory()

list(
  APPEND
  SRCS
  binfmt_globals.c
  binfmt_initialize.c
  binfmt_register.c
  binfmt_unregister.c
  binfmt_loadmodule.c
  binfmt_unloadmodule.c
  binfmt_execmodule.c
  binfmt_exec.c
  binfmt_copyargv.c
  binfmt_copyactions.c
  binfmt_dumpmodule.c
  binfmt_coredump.c)

if(CONFIG_BINFMT_LOADABLE)
  list(APPEND SRCS binfmt_exit.c)
endif()

if(CONFIG_LIBC_EXECFUNCS)
  list(APPEND SRCS binfmt_execsymtab.c)
endif()

if(CONFIG_ELF)
  list(APPEND SRCS elf.c)
endif()

if(CONFIG_NXFLAT)
  list(APPEND SRCS nxflat.c)
endif()

# Builtin application interfaces

if(CONFIG_BUILTIN)
  list(APPEND SRCS builtin.c)
endif()

target_sources(binfmt PRIVATE ${SRCS})
target_include_directories(binfmt PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(binfmt PRIVATE ${CMAKE_SOURCE_DIR}/sched)
