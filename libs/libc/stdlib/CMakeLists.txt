############################################################################
# libs/libc/stdlib/CMakeLists.txt
#
#   Copyright (C) 2012, 2015-2016 Gregory Nutt. All rights reserved.
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

set (SRCS
  lib_abs.c lib_abort.c lib_atof.c lib_atoi.c lib_atexit.c
  lib_atol.c lib_atoll.c lib_div.c lib_ldiv.c lib_lldiv.c
  lib_itoa.c lib_labs.c lib_llabs.c lib_realpath.c lib_bsearch.c
  lib_rand.c lib_qsort.c lib_srand.c lib_strtol.c lib_exit.c
  lib_strtoll.c lib_strtoul.c lib_strtoull.c lib_strtod.c lib_strtof.c
  lib_strtold.c lib_checkbase.c lib_mktemp.c lib_mkstemp.c lib_mkdtemp.c)

if (CONFIG_LIBC_WCHAR)
  list(APPEND SRCS
    lib_mblen.c lib_mbtowc.c lib_wctomb.c
    lib_mbstowcs.c lib_wcstombs.c)
endif()

if (CONFIG_PSEUDOTERM_SUSV1)
  list(APPEND SRCS lib_ptsname.c lib_ptsnamer.c)
endif()

if (CONFIG_PSEUDOTERM)
  list(APPEND SRCS lib_unlockpt.c)
endif()

target_sources(c PRIVATE ${SRCS})
