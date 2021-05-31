############################################################################
# libs/libc/misc/Make.defs
#
#   Copyright (C) 2011-2012, 2014, 2016, 2018 Gregory Nutt. All rights
#     reserved.
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

# Add the internal C files to the build
list(APPEND SRCS
  lib_mknod.c lib_umask.c lib_utsname.c lib_getrandom.c
  lib_xorshift128.c lib_tea_encrypt.c lib_tea_decrypt.c
  lib_cxx_initialize.c lib_impure.c)

# Add the internal C files to the build

list(APPEND SRCS
  lib_cxx_initialize.c lib_umask.c lib_utsname.c
  lib_xorshift128.c lib_tea_encrypt.c lib_tea_decrypt.c)

if (NOT CONFIG_STDIO_DISABLE_BUFFERING)
#  list(APPEND SRCS lib_filesem.c)
endif()

# Support for platforms that do not have long long types

list(APPEND SRCS
  lib_umul32.c lib_umul64.c lib_umul32x64.c
  lib_uadd32x64.c lib_uadd64.c lib_usub64x32.c lib_usub64.c)

# Add C files that depend on file OR socket descriptors

#list(APPEND SRCS lib_sendfile.c)

if (CONFIG_FILE_STREAM)
#  list(APPEND SRCS lib_streamsem.c)
endif()

if (CONFIG_PIPES)
  list(APPEND SRCS lib_mkfifo.c)
endif()

# Add the miscellaneous C files to the build

list(APPEND SRCS
  lib_crc64.c lib_crc32.c lib_crc16.c lib_crc8.c lib_crc8ccitt.c
  lib_dumpbuffer.c lib_dumpvbuffer.c lib_debug.c)

# Keyboard driver encoder/decoder

if (CONFIG_LIB_KBDCODEC)
  list(APPEND SRCS lib_kbdencode.c lib_kbddecode.c)
endif()

# SLCD driver encoder/decoder

if (CONFIG_LIB_SLCDCODEC)
  list(APPEND SRCS lib_slcdencode.c lib_slcddecode.c)
endif()

# Environment search path support

if (CONFIG_LIB_ENVPATH)
  list(APPEND SRCS lib_envpath.c)
endif()

target_sources(c PRIVATE ${SRCS})
