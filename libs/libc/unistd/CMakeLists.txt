############################################################################
# libs/libc/unistd/CMakeLists.txt
#
#   Copyright (C) 2011-2012, 2017 Gregory Nutt. All rights reserved.
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

set(CSRCS)

# Add the unistd directory to the build
list(APPEND CSRCS
	lib_access.c lib_daemon.c lib_swab.c lib_sysconf.c
  lib_getopt_common.c lib_getopt.c lib_getopt_long.c
  lib_getopt_longonly.c lib_getoptvars.c lib_getoptargp.c
  lib_getopterrp.c lib_getoptindp.c lib_getoptoptp.c
)

if(NOT "${CONFIG_DISABLE_ENVIRON}" STREQUAL "y")
	list(APPEND CSRCS lib_chdir.c lib_getcwd.c)
endif()

if("${CONFIG_LIBC_EXECFUNCS}" STREQUAL "y")
	list(APPEND CSRCS lib_execl.c)
endif()

if(NOT "${CONFIG_DISABLE_MOUNTPOINTS}" STREQUAL "y")
	list(APPEND CSRCS lib_truncate.c)
endif()

if("${CONFIG_PIPES}" STREQUAL "y")
	list(APPEND CSRCS lib_pipe.c)
endif()

if(NOT "${CONFIG_DISABLE_SIGNALS}" STREQUAL "y")
	list(APPEND CSRCS lib_sleep.c lib_usleep.c)
endif()

list(APPEND CSRCS lib_gethostname.c lib_sethostname.c)

set_parent_srcs(${CSRCS})
