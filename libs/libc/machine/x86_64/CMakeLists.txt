# ##############################################################################
# libs/libc/machine/x86_64/CMakeLists.txt
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

set(SRCS)

if(CONFIG_LIBC_ARCH_ELF)
  list(APPEND SRCS arch_elf64.c)
endif()

if(CONFIG_ARCH_SETJMP_H)
  list(APPEND SRCS arch_setjmp_x86_64.S)
endif()

if(CONFIG_X86_64_MEMCMP)
  if(CONFIG_ARCH_X86_64_AVX2)
    list(APPEND SRCS arch_memcmp_avx2.S)
  else()
    list(APPEND SRCS arch_memcmp.S)
  endif()
endif()

if(CONFIG_X86_64_MEMMOVE)
  if(CONFIG_ARCH_X86_64_AVX2)
    list(APPEND SRCS arch_memmove_avx2.S)
  else()
    list(APPEND SRCS arch_memmove.S)
  endif()
endif()

if(CONFIG_X86_64_MEMSET)
  if(CONFIG_ARCH_X86_64_AVX2)
    list(APPEND SRCS arch_memset_avx2.S)
  else()
    list(APPEND SRCS arch_memset.S)
  endif()
endif()

if(CONFIG_X86_64_STPCPY)
  if(CONFIG_ARCH_X86_64_AVX2)
    list(APPEND SRCS arch_stpcpy_avx2.S)
  else()
    list(APPEND SRCS arch_stpcpy.S)
  endif()
endif()

if(CONFIG_X86_64_STPNCPY)
  if(CONFIG_ARCH_X86_64_AVX2)
    list(APPEND SRCS arch_stpncpy_avx2.S)
  else()
    list(APPEND SRCS arch_stpncpy.S)
  endif()
endif()

if(CONFIG_X86_64_STRCAT)
  if(CONFIG_ARCH_X86_64_AVX2)
    list(APPEND SRCS arch_strcat_avx2.S)
  else()
    list(APPEND SRCS arch_strcat.S)
  endif()
endif()

if(CONFIG_X86_64_STRCMP)
  if(CONFIG_ARCH_X86_64_AVX2)
    list(APPEND SRCS arch_strcmp_avx2.S)
  else()
    list(APPEND SRCS arch_strcmp.S)
  endif()
endif()

if(CONFIG_X86_64_STRCPY)
  if(CONFIG_ARCH_X86_64_AVX2)
    list(APPEND SRCS arch_strcpy_avx2.S)
  else()
    list(APPEND SRCS arch_strcpy.S)
  endif()
endif()

if(CONFIG_X86_64_STRLEN)
  if(CONFIG_ARCH_X86_64_AVX2)
    list(APPEND SRCS arch_strlen_avx2.S)
  else()
    list(APPEND SRCS arch_strlen.S)
  endif()
endif()

if(CONFIG_X86_64_STRNCPY)
  if(CONFIG_ARCH_X86_64_AVX2)
    list(APPEND SRCS arch_strncpy_avx2.S)
  else()
    list(APPEND SRCS arch_strncpy.S)
  endif()
endif()

if(CONFIG_X86_64_STRNCMP)
  if(CONFIG_ARCH_X86_64_AVX2)
    list(APPEND SRCS arch_strncmp_avx2.S)
  else()
    list(APPEND SRCS arch_strncmp.S)
  endif()
endif()

# Scan functions: AVX2 when enabled, else SSE2 (x86_64 baseline). No legacy
# non-SIMD asm; generic C libc is the final fallback when the switch is off.

if(CONFIG_X86_64_STRCHR)
  if(CONFIG_ARCH_X86_64_AVX2)
    list(APPEND SRCS arch_strchr_avx2.S)
  else()
    list(APPEND SRCS arch_strchr.S)
  endif()
endif()

if(CONFIG_X86_64_STRCHRNUL)
  if(CONFIG_ARCH_X86_64_AVX2)
    list(APPEND SRCS arch_strchrnul_avx2.S)
  else()
    list(APPEND SRCS arch_strchrnul.S)
  endif()
endif()

if(CONFIG_X86_64_STRRCHR)
  if(CONFIG_ARCH_X86_64_AVX2)
    list(APPEND SRCS arch_strrchr_avx2.S)
  else()
    list(APPEND SRCS arch_strrchr.S)
  endif()
endif()

if(CONFIG_X86_64_STRNLEN)
  if(CONFIG_ARCH_X86_64_AVX2)
    list(APPEND SRCS arch_strnlen_avx2.S)
  else()
    list(APPEND SRCS arch_strnlen.S)
  endif()
endif()

if(CONFIG_X86_64_MEMCHR)
  if(CONFIG_ARCH_X86_64_AVX2)
    list(APPEND SRCS arch_memchr_avx2.S)
  else()
    list(APPEND SRCS arch_memchr.S)
  endif()
endif()

if(CONFIG_PROFILE_MINI)
  list(APPEND SRCS gnu/mcount.S)
endif()

target_sources(c PRIVATE ${SRCS})
