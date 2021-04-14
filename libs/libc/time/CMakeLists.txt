############################################################################
# libs/libc/time/Make.defs
#
#   Copyright (C) 2011-2012, 2014-2015, 2018 Gregory Nutt. All rights
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

# Add the time C files to the build

set(CSRCS) 

if(NOT "${CONFIG_DISABLE_SIGNALS}" STREQUAL "y")
	list(APPEND CSRCS lib_nanosleep.c)
endif()

if("${CONFIG_LIBC_LOCALTIME}" STREQUAL "y")
	list(APPEND CSRCS
		lib_localtime.c
		lib_asctime.c
		lib_asctimer.c
		lib_ctime.c
		lib_ctimer.c
	)
else()
	list(APPEND CSRCS
		lib_mktime.c
		lib_gmtime.c
		lib_gmtimer.c
	)

	if("${CONFIG_TIME_EXTENDED}" STREQUAL "y")
		list(APPEND CSRCS
			lib_dayofweek.c
			lib_asctime.c
			lib_asctimer.c
			lib_ctime.c
			lib_ctimer.c
		)
	endif()
endif()

# Add the time directory to the build
list(APPEND CSRCS
	lib_strftime.c
	lib_calendar2utc.c
	lib_daysbeforemonth.c
	lib_gettimeofday.c
	lib_isleapyear.c
	lib_settimeofday.c
	lib_time.c
	lib_difftime.c
)
set_parent_srcs(${CSRCS})
