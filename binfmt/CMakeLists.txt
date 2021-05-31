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

nuttx_add_kernel_library(binfmt)

add_subdirectory(libelf)
add_subdirectory(libnxflat)

list(APPEND SRCS
  binfmt_globals.c binfmt_initialize.c binfmt_register.c binfmt_unregister.c
  binfmt_loadmodule.c binfmt_unloadmodule.c binfmt_execmodule.c
  binfmt_exec.c binfmt_copyargv.c binfmt_dumpmodule.c)

if (CONFIG_BINFMT_LOADABLE)
  list(APPEND SRCS binfmt_exit.c)
endif()

if (CONFIG_LIBC_EXECFUNCS)
  list(APPEND SRCS binfmt_execsymtab.c)
endif()

if (CONFIG_ELF)
  list(APPEND SRCS elf.c)
endif()

if (CONFIG_NXFLAT)
  list(APPEND SRCS nxflat.c)
endif()

# Builtin application interfaces

if (CONFIG_BUILTIN)
  list(APPEND SRCS builtin.c)
endif()

target_sources(binfmt PRIVATE ${SRCS})
target_include_directories(binfmt PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
