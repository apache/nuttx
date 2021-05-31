############################################################################
# libs/libc/string/CMakeLists.txt
#
#   Copyright (C) 2011-2012, 2014, 2016-2018 Gregory Nutt. All rights reserved.
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

set(SRCS
  lib_ffs.c lib_ffsl.c lib_ffsll.c lib_fls.c lib_flsl.c
  lib_flsll.c lib_isbasedigit.c lib_memset.c lib_memchr.c
  lib_memccpy.c lib_memcmp.c lib_memmove.c lib_memrchr.c
  lib_popcount.c lib_popcountl.c lib_popcountll.c
  lib_skipspace.c lib_stpcpy.c lib_stpncpy.c lib_strcasecmp.c
  lib_strcat.c lib_strchr.c lib_strcpy.c lib_strcmp.c lib_strcspn.c
  lib_strdup.c lib_strerror.c lib_strlen.c lib_strnlen.c
  lib_strncasecmp.c lib_strncat.c lib_strncmp.c lib_strncpy.c
  lib_strndup.c lib_strcasestr.c lib_strpbrk.c lib_strrchr.c
  lib_strspn.c lib_strstr.c lib_strtok.c lib_strtokr.c
  lib_strsep.c lib_strerrorr.c lib_explicit_bzero.c lib_strsignal.c
  lib_index.c lib_rindex.c lib_strlcpy.c)

if (NOT CONFIG_LIBC_ARCH_MEMCPY)
  if (CONFIG_MEMCPY_VIK)
    list(APPEND SRCS lib_vikmemcpy.c)
  else()
    list(APPEND SRCS lib_memcpy.c)
  endif()
endif()

if (CONFIG_LIBC_LOCALE)
  list(APPEND SRCS lib_strcoll.c lib_strxfrm.c)
endif()

target_sources(c PRIVATE ${SRCS})
