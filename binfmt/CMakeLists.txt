############################################################################
# binfmt/CMakeLists.txt
#
#   Copyright (C) 2007-2009, 2012-2016, 2018 Gregory Nutt. All rights
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

set(SRCS)
set(CSRCS)

if("${CONFIG_ELF}" STREQUAL "y")
	list(APPEND CSRCS elf.c)
	add_subdirectory(libelf)
endif()

if("${CONFIG_NXFLAT}" STREQUAL "y")
	add_subdirectory(libnxflat)
endif()

if("${CONFIG_BINFMT_PCODE}" STREQUAL "y")
	list(APPEND CSRCS pcode.c)
	add_subdirectory(libpcode)
endif()

if("${CONFIG_BINFMT_LOADABLE}" STREQUAL "y")
	list(APPEND CSRCS binfmt_exit.c)
endif()

if("${CONFIG_LIBC_EXECFUNCS}" STREQUAL "y")
	list(APPEND CSRCS binfmt_execsymtab.c)
endif()

nuttx_add_library(binfmt
	binfmt_copyargv.c
	binfmt_dumpmodule.c
	binfmt_exec.c
	binfmt_execmodule.c
	binfmt_globals.c
	binfmt_initialize.c
	binfmt_loadmodule.c
	binfmt_register.c
	binfmt_unloadmodule.c
	binfmt_unregister.c
	${CSRCS}
	${SRCS}
)

target_include_directories(binfmt INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
