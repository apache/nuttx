# ##############################################################################
# libs/libc/stdio/CMakeLists.txt
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

# This first group of C files do not depend on having C streams.

set(SRCS
    lib_fileno.c
    lib_printf.c
    lib_sprintf.c
    lib_asprintf.c
    lib_snprintf.c
    lib_libsprintf.c
    lib_vsprintf.c
    lib_vasprintf.c
    lib_vsnprintf.c
    lib_dprintf.c
    lib_vdprintf.c
    lib_vprintf.c
    lib_perror.c
    lib_putchar.c
    lib_getchar.c
    lib_puts.c
    lib_gets_s.c
    lib_gets.c
    lib_libdgets.c
    lib_sscanf.c
    lib_vsscanf.c
    lib_libvscanf.c
    lib_libvsprintf.c
    lib_remove.c
    lib_tempnam.c
    lib_tmpnam.c
    lib_ultoa_invert.c
    lib_renameat.c
    lib_putwchar.c
    lib_libbsprintf.c)

if(CONFIG_LIBC_FLOATINGPOINT)
  list(APPEND SRCS lib_dtoa_engine.c lib_dtoa_data.c)
endif()

# The remaining sources files depend upon C streams

if(CONFIG_FILE_STREAM)
  list(
    APPEND
    SRCS
    lib_fopen.c
    lib_freopen.c
    lib_fclose.c
    lib_fread.c
    lib_libfread_unlocked.c
    lib_fseek.c
    lib_fseeko.c
    lib_ftell.c
    lib_ftello.c
    lib_fsetpos.c
    lib_getdelim.c
    lib_fgetpos.c
    lib_getc.c
    lib_fgetc.c
    lib_fgets.c
    lib_libfgets.c
    lib_fwrite.c
    lib_libfwrite.c
    lib_fflush.c
    lib_libflushall.c
    lib_libfflush.c
    lib_rdflush_unlocked.c
    lib_wrflush_unlocked.c
    lib_putc.c
    lib_fputc.c
    lib_fputs.c
    lib_ungetc.c
    lib_fprintf.c
    lib_vfprintf.c
    lib_feof.c
    lib_ferror.c
    lib_rewind.c
    lib_clearerr.c
    lib_scanf.c
    lib_vscanf.c
    lib_fscanf.c
    lib_vfscanf.c
    lib_tmpfile.c
    lib_setbuf.c
    lib_setvbuf.c
    lib_libfilelock.c
    lib_libgetstreams.c
    lib_setbuffer.c
    lib_fputwc.c
    lib_putwc.c
    lib_fputws.c
    lib_fopencookie.c
    lib_fmemopen.c
    lib_open_memstream.c
    lib_fgetwc.c
    lib_getwc.c
    lib_ungetwc.c)
endif()

target_sources(c PRIVATE ${SRCS})
