############################################################################
# libs/libc/stdio/Make.defs
#
#   Copyright (C) 2011-2014 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

# This first group of C files do not depend on having C streams.

set(SRCS
  lib_fileno.c lib_printf.c lib_sprintf.c lib_asprintf.c
  lib_snprintf.c lib_libsprintf.c lib_vsprintf.c lib_vasprintf.c
  lib_vsnprintf.c lib_dprintf.c lib_vdprintf.c
  lib_sscanf.c lib_vsscanf.c lib_libvscanf.c
  lib_tempnam.c lib_tmpnam.c)

if (CONFIG_LIBC_PRINT_LEGACY)
  list(APPEND SRCS legacy_libvsprintf.c)

  if (CONFIG_LIBC_FLOATINGPOINT)
    list(APPEND SRCS legacy_dtoa.c)
  endif()
else()
  list(APPEND SRCS lib_libvsprintf.c lib_ultoa_invert.c)
  if (CONFIG_LIBC_FLOATINGPOINT)
    list(APPEND SRCS lib_dtoa_engine.c lib_dtoa_data.c)
  endif()
endif()

# The remaining sources files depend upon C streams

if (CONFIG_FILE_STREAM)
  list(APPEND SRCS
    lib_fopen.c lib_freopen.c lib_fclose.c lib_fread.c lib_libfread.c
    lib_fseek.c lib_fseeko.c lib_ftell.c lib_ftello.c lib_fsetpos.c
    lib_getdelim.c lib_fgetpos.c lib_getc.c lib_getchar.c lib_fgetc.c
    lib_fgets.c lib_gets_s.c lib_gets.c lib_libfgets.c lib_fwrite.c
    lib_libfwrite.c lib_fflush.c lib_libflushall.c lib_libfflush.c
    lib_rdflush.c lib_wrflush.c lib_putc.c lib_putchar.c lib_fputc.c
    lib_puts.c lib_fputs.c lib_ungetc.c lib_vprintf.c lib_fprintf.c
    lib_vfprintf.c lib_libstream.c lib_libfilesem.c
    lib_perror.c lib_feof.c lib_ferror.c
    lib_remove.c lib_rewind.c lib_clearerr.c
    lib_scanf.c lib_vscanf.c lib_fscanf.c lib_vfscanf.c lib_tmpfile.c
    lib_setbuf.c lib_setvbuf.c)
endif()

target_sources(c PRIVATE ${SRCS})
