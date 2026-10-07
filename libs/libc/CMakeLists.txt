# ##############################################################################
# libs/libc/CMakeLists.txt
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
nuttx_add_kernel_library(c SPLIT)

nuttx_add_subdirectory()

target_include_directories(c PRIVATE ${CMAKE_CURRENT_LIST_DIR})

# Generate the system symbol tables from the C library, math library and system
# call lists, as libs/libc/Makefile does.  The lists are sorted by name.

if(CONFIG_EXECFUNCS_SYSTEM_SYMTAB OR CONFIG_LIBC_ELF_SYSTEM_SYMTAB)
  nuttx_build_host_target(mksymtab ${NUTTX_DIR}/tools/mksymtab.c)

  set(SYMTAB_CSVS
      ${CMAKE_CURRENT_LIST_DIR}/libc.csv ${NUTTX_DIR}/libs/libm/libm.csv
      ${NUTTX_DIR}/syscall/syscall.csv)
  set(SYMTAB_CSV ${CMAKE_CURRENT_BINARY_DIR}/system_symtab.csv)

  add_custom_command(
    OUTPUT ${SYMTAB_CSV}
    COMMAND cat ${SYMTAB_CSVS} | LC_ALL=C sort > ${SYMTAB_CSV}
    DEPENDS ${SYMTAB_CSVS})

  if(CONFIG_EXECFUNCS_SYSTEM_SYMTAB)
    add_custom_command(
      OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/exec_symtab.c
      COMMAND
        ${NUTTX_BINARY_DIR}/bin_host/mksymtab system_symtab.csv exec_symtab.c
        ${CONFIG_EXECFUNCS_SYMTAB_ARRAY} ${CONFIG_EXECFUNCS_NSYMBOLS_VAR}
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
      DEPENDS mksymtab ${SYMTAB_CSV})
    target_sources(c PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/exec_symtab.c)
  endif()

  if(CONFIG_LIBC_ELF_SYSTEM_SYMTAB)
    add_custom_command(
      OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/elf_sys_symtab.c
      COMMAND
        ${NUTTX_BINARY_DIR}/bin_host/mksymtab system_symtab.csv elf_sys_symtab.c
        ${CONFIG_LIBC_ELF_SYMTAB_ARRAY} ${CONFIG_LIBC_ELF_NSYMBOLS_VAR}
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
      DEPENDS mksymtab ${SYMTAB_CSV})
    target_sources(c PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/elf_sys_symtab.c)
  endif()
endif()
