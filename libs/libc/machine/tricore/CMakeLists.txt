# ##############################################################################
# libs/libc/machine/tricore/CMakeLists.txt
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

if(CONFIG_LIBC_ATOMIC_ARCH)
  list(APPEND SRCS arch_atomic.c)
endif()

if(CONFIG_TRICORE_TOOLCHAIN_TASKING)
  set(TRICORE_SUBDIR tasking)
else()
  set(TRICORE_SUBDIR gnu)
endif()

if(CONFIG_ARCH_SETJMP_H)
  list(APPEND SRCS ${TRICORE_SUBDIR}/arch_setjmp.${ASMEXT})
endif()

if(CONFIG_LIBC_ARCH_MEMCPY)
  list(APPEND SRCS ${TRICORE_SUBDIR}/arch_memcpy.${ASMEXT})
endif()

if(CONFIG_LIBC_ARCH_MEMSET)
  list(APPEND SRCS ${TRICORE_SUBDIR}/arch_memset.${ASMEXT})
endif()

if(CONFIG_LIBC_ARCH_MEMMOVE)
  list(APPEND SRCS ${TRICORE_SUBDIR}/arch_memmove.${ASMEXT})
endif()

if(CONFIG_LIBC_ARCH_MEMCMP)
  list(APPEND SRCS ${TRICORE_SUBDIR}/arch_memcmp.${ASMEXT})
endif()

if(CONFIG_LIBC_ARCH_MEMCHR)
  list(APPEND SRCS ${TRICORE_SUBDIR}/arch_memchr.${ASMEXT})
endif()

if(CONFIG_LIBC_ARCH_STRCPY)
  list(APPEND SRCS ${TRICORE_SUBDIR}/arch_strcpy.${ASMEXT})
endif()

if(CONFIG_LIBC_ARCH_STRCMP)
  list(APPEND SRCS ${TRICORE_SUBDIR}/arch_strcmp.${ASMEXT})
endif()

if(CONFIG_LIBC_ARCH_STRLEN)
  list(APPEND SRCS ${TRICORE_SUBDIR}/arch_strlen.${ASMEXT})
endif()

if(CONFIG_LIBC_ARCH_STRCHR)
  list(APPEND SRCS ${TRICORE_SUBDIR}/arch_strchr.${ASMEXT})
endif()

if(CONFIG_LIBC_ARCH_STRCHRNUL)
  list(APPEND SRCS ${TRICORE_SUBDIR}/arch_strchrnul.${ASMEXT})
endif()

if(CONFIG_LIBC_ARCH_STRNCMP)
  list(APPEND SRCS ${TRICORE_SUBDIR}/arch_strncmp.${ASMEXT})
endif()

if(CONFIG_LIBC_ARCH_STRNLEN)
  list(APPEND SRCS ${TRICORE_SUBDIR}/arch_strnlen.${ASMEXT})
endif()

if(CONFIG_LIBC_ARCH_STRRCHR)
  list(APPEND SRCS ${TRICORE_SUBDIR}/arch_strrchr.${ASMEXT})
endif()

if(CONFIG_LIBC_ARCH_STRNCPY)
  list(APPEND SRCS ${TRICORE_SUBDIR}/arch_strncpy.${ASMEXT})
endif()

if(CONFIG_LIBC_ARCH_STPCPY)
  list(APPEND SRCS ${TRICORE_SUBDIR}/arch_stpcpy.${ASMEXT})
endif()

target_include_directories(c PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_sources(c PRIVATE ${SRCS})
