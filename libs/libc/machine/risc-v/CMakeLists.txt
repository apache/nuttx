# ##############################################################################
# libs/libc/machine/risc-v/CMakeLists.txt
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

if(CONFIG_RISCV_MEMCPY)
  list(APPEND SRCS arch_memcpy.S)
endif()

if(CONFIG_RISCV_MEMSET)
  list(APPEND SRCS arch_memset.S)
endif()

if(CONFIG_RISCV_STRCMP)
  list(APPEND SRCS arch_strcmp.S)
endif()

if(CONFIG_RISCV_MEMCHR)
  list(APPEND SRCS arch_memchr.c)
endif()

if(CONFIG_RISCV_MEMCMP)
  list(APPEND SRCS arch_memcmp.c)
endif()

if(CONFIG_RISCV_MEMMOVE)
  list(APPEND SRCS arch_memmove.c)
endif()

if(CONFIG_RISCV_STRCPY)
  list(APPEND SRCS arch_strcpy.c)
endif()

if(CONFIG_RISCV_STRLCPY)
  list(APPEND SRCS arch_strlcpy.c)
endif()

if(CONFIG_RISCV_STRNCMP)
  list(APPEND SRCS arch_strncmp.c)
endif()

if(CONFIG_RISCV_STRNLEN)
  list(APPEND SRCS arch_strnlen.c)
endif()

if(CONFIG_ARCH_SETJMP_H)
  list(APPEND SRCS arch_setjmp.S)
endif()

if(CONFIG_LIBC_ARCH_ELF)
  list(APPEND SRCS arch_elf.c)
endif()

target_sources(c PRIVATE ${SRCS})
