############################################################################
# libs/libc/wchar/Make.defs
#
#   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

if("${CONFIG_LIBC_WCHAR}" STREQUAL "y")

	set(CSRCS)

	# Add the internal C files to the build
	list(APPEND CSRCS
		lib_wcscmp.c lib_wcslen.c lib_wmemchr.c lib_wmemcmp.c lib_wmemcpy.c
		lib_wmemmove.c lib_wmemset.c lib_btowc.c lib_mbrtowc.c lib_wctob.c
		lib_wcslcpy.c lib_wcsxfrm.c lib_wcrtomb.c lib_wcsftime.c
		lib_wcscoll.c lib_wcstol.c lib_wcstoll.c lib_wcstoul.c
		lib_wcstoull.c lib_wcstold.c lib_wcstof.c lib_wcstod.c
		lib_swprintf.c lib_mbsnrtowcs.c lib_wcsnrtombs.c
		lib_mbrlen.c lib_mbsrtowcs.c
	)
	
	# Add the wchar directory to the build
	set_parent_srcs(${CSRCS})

endif()

