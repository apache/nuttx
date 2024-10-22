# ##############################################################################
# libs/libc/string/CMakeLists.txt
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

# Add the string C files to the build

set(SRCS
    lib_ffs.c
    lib_ffsl.c
    lib_ffsll.c
    lib_fls.c
    lib_flsl.c
    lib_flsll.c
    lib_isbasedigit.c
    lib_memmem.c
    lib_popcount.c
    lib_popcountl.c
    lib_popcountll.c
    lib_skipspace.c
    lib_strcasecmp.c
    lib_strcspn.c
    lib_strdup.c
    lib_strerror.c
    lib_strncasecmp.c
    lib_strncat.c
    lib_strndup.c
    lib_strcasestr.c
    lib_strpbrk.c
    lib_strspn.c
    lib_strstr.c
    lib_strtok.c
    lib_strtokr.c
    lib_strsep.c
    lib_strerrorr.c
    lib_explicit_bzero.c
    lib_strsignal.c
    lib_index.c
    lib_rindex.c
    lib_timingsafe_bcmp.c
    lib_strverscmp.c
    lib_mempcpy.c
    lib_rawmemchr.c
    lib_memmove.c
    lib_memset.c
    lib_strlcat.c
    lib_strlcpy.c
    lib_strnlen.c)

if(CONFIG_MEMCPY_VIK)
  list(APPEND SRCS lib_vikmemcpy.c)
elseif(CONFIG_LIBC_STRING_OPTIMIZE)
  list(APPEND SRCS lib_bsdmemcpy.c)
else()
  list(APPEND SRCS lib_memcpy.c)
endif()

if(CONFIG_LIBC_LOCALE)
  list(APPEND SRCS lib_strcoll.c lib_strxfrm.c)
endif()

if(CONFIG_LIBC_STRING_OPTIMIZE)
  list(
    APPEND
    SRCS
    lib_bsdmemccpy.c
    lib_bsdmemcmp.c
    lib_bsdmemrchr.c
    lib_bsdstpncpy.c
    lib_bsdstrchr.c
    lib_bsdstrcmp.c
    lib_bsdstrlen.c
    lib_bsdstrncpy.c
    lib_bsdmemchr.c
    lib_bsdstpcpy.c
    lib_bsdstrcat.c
    lib_bsdstrchrnul.c
    lib_bsdstrcpy.c
    lib_bsdstrncmp.c
    lib_bsdstrrchr.c)
else()
  list(
    APPEND
    SRCS
    lib_memccpy.c
    lib_memcmp.c
    lib_memrchr.c
    lib_stpncpy.c
    lib_strchr.c
    lib_strcmp.c
    lib_strlen.c
    lib_strncpy.c
    lib_memchr.c
    lib_stpcpy.c
    lib_strcat.c
    lib_strchrnul.c
    lib_strcpy.c
    lib_strncmp.c
    lib_strrchr.c)
endif()

target_sources(c PRIVATE ${SRCS})
