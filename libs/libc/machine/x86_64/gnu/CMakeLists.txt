# ##############################################################################
# libs/libc/machine/X86_64/gnu/CMakeLists.txt
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

if(CONFIG_X86_64_MEMCMP)
  list(APPEND SRCS arch_memcmp.S)
endif()

if(CONFIG_X86_64_MEMMOVE)
  list(APPEND SRCS arch_memmove.S)
endif()

if(CONFIG_X86_64_MEMSET)
  if(CONFIG_ARCH_X86_64_AVX)
    list(APPEND SRCS arch_memset_avx2.S)
  else()
    list(APPEND SRCS arch_memset_sse2.S)
  endif()
endif()

if(CONFIG_X86_64_STPCPY)
  list(APPEND SRCS arch_stpcpy.S)
endif()

if(CONFIG_X86_64_STPNCPY)
  list(APPEND SRCS arch_stpncpy.S)
endif()

if(CONFIG_X86_64_STRCAT)
  list(APPEND SRCS arch_strcat.S)
endif()

if(CONFIG_X86_64_STRCMP)
  list(APPEND SRCS arch_strcmp.S)
endif()

if(CONFIG_X86_64_STRCPY)
  list(APPEND SRCS arch_strcpy.S)
endif()

if(CONFIG_X86_64_STRLEN)
  list(APPEND SRCS arch_strlen.S)
endif()

if(CONFIG_X86_64_STRNCPY)
  list(APPEND SRCS arch_strncpy.S)
endif()

if(CONFIG_X86_64_STRNCMP)
  list(APPEND SRCS arch_strncmp.S)
endif()

target_sources(c PRIVATE ${SRCS})
