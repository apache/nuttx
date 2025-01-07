# ##############################################################################
# libs/libc/misc/CMakeLists.txt
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
# ##############################################################################

# Add the internal C files to the build

list(
  APPEND
  SRCS
  lib_bitmap.c
  lib_circbuf.c
  lib_mknod.c
  lib_umask.c
  lib_utsname.c
  lib_getrandom.c
  lib_xorshift128.c
  lib_tea_encrypt.c
  lib_tea_decrypt.c
  lib_cxx_initialize.c
  lib_idr.c
  lib_impure.c
  lib_memfd.c
  lib_mutex.c
  lib_fchmodat.c
  lib_fstatat.c
  lib_getfullpath.c
  lib_openat.c
  lib_mkdirat.c
  lib_utimensat.c
  lib_mallopt.c
  lib_getnprocs.c)

if(CONFIG_LIBC_TEMPBUFFER)
  list(APPEND SRCS lib_tempbuffer.c)
endif()

# Support for platforms that do not have long long types

list(
  APPEND
  SRCS
  lib_umul32.c
  lib_umul64.c
  lib_umul32x64.c
  lib_uadd32x64.c
  lib_uadd64.c
  lib_usub64x32.c
  lib_usub64.c)

if(CONFIG_PIPES)
  list(APPEND SRCS lib_mkfifo.c)
endif()

# Add the miscellaneous C files to the build

list(
  APPEND
  SRCS
  lib_dumpbuffer.c
  lib_dumpvbuffer.c
  lib_fnmatch.c
  lib_debug.c
  lib_crc64.c
  lib_crc32.c
  lib_crc16.c
  lib_crc16ccitt.c
  lib_crc8.c
  lib_crc8ccitt.c
  lib_crc8rohc.c
  lib_crc8table.c
  lib_glob.c
  lib_backtrace.c
  lib_ftok.c
  lib_err.c
  lib_instrument.c)

# Keyboard driver encoder/decoder

if(CONFIG_LIBC_KBDCODEC)
  list(APPEND SRCS lib_kbdencode.c lib_kbddecode.c)
endif()

# SLCD driver encoder/decoder

if(CONFIG_LIBC_SLCDCODEC)
  list(APPEND SRCS lib_slcdencode.c lib_slcddecode.c)
endif()

# Environment search path support

if(CONFIG_LIBC_ENVPATH)
  list(APPEND SRCS lib_envpath.c)
endif()

if(CONFIG_FDSAN)
  list(APPEND SRCS lib_fdsan.c)
endif()

if(CONFIG_FDCHECK)
  list(APPEND SRCS lib_fdcheck.c)
endif()

if(NOT CONFIG_LIBC_UNAME_DISABLE_TIMESTAMP)
  add_custom_target(
    always_rebuild_lib_utsname
    COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_SOURCE_DIR}/lib_utsname.c)

  add_dependencies(c always_rebuild_lib_utsname)
endif()

target_sources(c PRIVATE ${SRCS})
