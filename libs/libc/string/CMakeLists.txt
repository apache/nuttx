# ##############################################################################
# libs/libc/string/CMakeLists.txt
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

# Add the string C files to the build

set(SRCS
    lib_ffs.c
    lib_ffsl.c
    lib_ffsll.c
    lib_fls.c
    lib_flsl.c
    lib_flsll.c
    lib_isbasedigit.c
    lib_memccpy.c
    lib_memrchr.c
    lib_memmem.c
    lib_popcount.c
    lib_popcountl.c
    lib_popcountll.c
    lib_skipspace.c
    lib_stpcpy.c
    lib_stpncpy.c
    lib_strcasecmp.c
    lib_strcat.c
    lib_strcspn.c
    lib_strchrnul.c
    lib_strdup.c
    lib_strerror.c
    lib_strncasecmp.c
    lib_strncat.c
    lib_strncmp.c
    lib_strndup.c
    lib_strcasestr.c
    lib_strpbrk.c
    lib_strrchr.c
    lib_strspn.c
    lib_strstr.c
    lib_strtok.c
    lib_strtokr.c
    lib_strsep.c
    lib_strerrorr.c
    lib_explicit_bzero.c
    lib_strsignal.c
    lib_timingsafe_bcmp.c
    lib_index.c
    lib_rindex.c)

if(NOT CONFIG_LIBC_ARCH_MEMCHR)
  list(APPEND SRCS lib_memchr.c)
endif()

if(NOT CONFIG_LIBC_ARCH_MEMCMP)
  list(APPEND SRCS lib_memcmp.c)
endif()

if(NOT CONFIG_LIBC_ARCH_MEMCPY)
  if(CONFIG_MEMCPY_VIK)
    list(APPEND SRCS lib_vikmemcpy.c)
  else()
    list(APPEND SRCS lib_memcpy.c)
  endif()
endif()

if(NOT CONFIG_LIBC_ARCH_MEMMOVE)
  list(APPEND SRCS lib_memmove.c)
endif()

if(NOT CONFIG_LIBC_ARCH_MEMSET)
  list(APPEND SRCS lib_memset.c)
endif()

if(NOT CONFIG_LIBC_ARCH_STRCHR)
  list(APPEND SRCS lib_strchr.c)
endif()

if(NOT CONFIG_LIBC_ARCH_STRCMP)
  list(APPEND SRCS lib_strcmp.c)
endif()

if(NOT CONFIG_LIBC_ARCH_STRCPY)
  list(APPEND SRCS lib_strcpy.c)
endif()

if(NOT CONFIG_LIBC_ARCH_STRLCAT)
  list(APPEND SRCS lib_strlcat.c)
endif()

if(NOT CONFIG_LIBC_ARCH_STRLCPY)
  list(APPEND SRCS lib_strlcpy.c)
endif()

if(NOT CONFIG_LIBC_ARCH_STRLEN)
  list(APPEND SRCS lib_strlen.c)
endif()

if(NOT CONFIG_LIBC_ARCH_STRNCPY)
  list(APPEND SRCS lib_strncpy.c)
endif()

if(NOT CONFIG_LIBC_ARCH_STRNLEN)
  list(APPEND SRCS lib_strnlen.c)
endif()

if(CONFIG_LIBC_LOCALE)
  list(APPEND SRCS lib_strcoll.c lib_strxfrm.c)
endif()

target_sources(c PRIVATE ${SRCS})
