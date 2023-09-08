# ##############################################################################
# libs/libc/stream/CMakeLists.txt
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
list(
  APPEND
  SRCS
  lib_meminstream.c
  lib_memoutstream.c
  lib_memsistream.c
  lib_memsostream.c
  lib_lowoutstream.c
  lib_rawinstream.c
  lib_rawoutstream.c
  lib_rawsistream.c
  lib_rawsostream.c
  lib_zeroinstream.c
  lib_nullinstream.c
  lib_nulloutstream.c
  lib_mtdoutstream.c
  lib_libnoflush.c
  lib_libsnoflush.c
  lib_syslogstream.c
  lib_syslograwstream.c
  lib_bufferedoutstream.c
  lib_hexdumpstream.c)

if(CONFIG_FILE_STREAM)
  list(APPEND SRCS lib_stdinstream.c lib_stdoutstream.c lib_stdsistream.c
       lib_stdsostream.c)
endif()

if(CONFIG_LIBC_LZF)
  list(APPEND SRCS lib_lzfcompress.c)
endif()

if(NOT CONFIG_DISABLE_MOUNTPOINT)
  list(APPEND SRCS lib_blkoutstream.c)
endif()

target_sources(c PRIVATE ${SRCS})
