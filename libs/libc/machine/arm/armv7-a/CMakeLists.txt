# ##############################################################################
# libs/libc/machine/arm/armv7-a/CMakeLists.txt
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

if(CONFIG_ARCH_TOOLCHAIN_GNU)
  set(ARCH_TOOLCHAIN_DIR gnu)
endif()

if(CONFIG_ARMV7A_MEMCHR)
  list(APPEND SRCS ${ARCH_TOOLCHAIN_DIR}/arch_memchr.S)
endif()

if(CONFIG_ARMV7A_MEMCPY)
  list(APPEND SRCS ${ARCH_TOOLCHAIN_DIR}/arch_memcpy.S)
endif()

if(CONFIG_ARMV7A_MEMMOVE)
  list(APPEND SRCS ${ARCH_TOOLCHAIN_DIR}/arch_memmove.S)
endif()

if(CONFIG_ARMV7A_MEMSET)
  list(APPEND SRCS ${ARCH_TOOLCHAIN_DIR}/arch_memset.S)
endif()

if(CONFIG_ARMV7A_STRCMP)
  list(APPEND SRCS ${ARCH_TOOLCHAIN_DIR}/arch_strcmp.S)
endif()

if(CONFIG_ARMV7A_STRLEN)
  list(APPEND SRCS ${ARCH_TOOLCHAIN_DIR}/arch_strlen.S)
endif()

if(CONFIG_LIBC_ARCH_ELF)
  list(APPEND SRCS arch_elf.c)
endif()

target_sources(c PRIVATE ${SRCS})
