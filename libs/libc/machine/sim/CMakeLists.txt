############################################################################
# libs/libc/machine/sim/Make.defs
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

# XXX ELF relocations are not actually sim-dependent.
# TODO: We should share the code with eg. ../x86/arch_elf.c.

set(SRCS)

if (X86_64)
  if (CONFIG_SIM_M32)
    if (CONFIG_LIBC_ARCH_ELF)
      list(APPEND SRCS arch_elf.c)
    endif()
    if (CONFIG_ARCH_SETJMP_H)
      list(APPEND SRCS arch_setjmp.S)
    endif()
  else()
    if (CONFIG_LIBC_ARCH_ELF)
      list(APPEND SRCS arch_elf64.c)
    endif()
    if (CONFIG_ARCH_SETJMP_H)
      list(APPEND SRCS arch_setjmp_x86_64.S)
    endif()
  endif()
elseif (X86)
  if (CONFIG_LIBC_ARCH_ELF)
    list(APPEND SRCS arch_elf.c)
  endif()
  if (CONFIG_ARCH_SETJMP_H)
    list(APPEND SRCS arch_setjmp.S)
  endif()
elseif (ARM)
  if (CONFIG_ARCH_SETJMP_H)
    list(APPEND SRCS arch_setjmp_arm.S)
  endif()
endif()

target_sources(c PRIVATE ${SRCS})
