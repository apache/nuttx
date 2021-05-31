############################################################################
# sched/group/CMakeLists.txt
#
#   Copyright (C) 2014, 2018 Gregory Nutt. All rights reserved.
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

list(APPEND SRCS
  group_create.c group_join.c group_leave.c group_find.c
  group_setupstreams.c group_setupidlefiles.c group_setuptaskfiles.c
  group_foreachchild.c group_killchildren.c group_signal.c)

if (CONFIG_SCHED_HAVE_PARENT)
  if (CONFIG_SCHED_CHILD_STATUS)
    list(APPEND SRCS group_childstatus.c)
  endif()
else()
  if (CONFIG_SCHED_WAITPID)
    list(APPEND SRCS group_waiter.c)
  endif()
endif()

if (CONFIG_SCHED_USER_IDENTITY)
  list(APPEND SRCS group_setuid.c group_setgid.c group_getuid.c group_getgid.c)
endif()

if (CONFIG_ARCH_ADDRENV)
  list(APPEND SRCS group_addrenv.c)
endif()

if (CONFIG_SIG_SIGSTOP_ACTION)
  list(APPEND SRCS group_suspendchildren.c group_continue.c)
endif()

if (CONFIG_BINFMT_LOADABLE)
  list(APPEND SRCS group_exitinfo.c)
endif()

if (NOT CONFIG_BUILD_FLAT)
  list(APPEND SRCS group_malloc.c group_zalloc.c group_free.c)
endif()

if (NOT CONFIG_TLS_NELEM EQUAL 0)
#  list(APPEND SRCS group_tlsalloc.c group_tlsfree.c)
endif()

if (NOT CONFIG_BUILD_KERNEL)
#  list(APPEND SRCS group_taskdata.c)
endif()

target_sources(sched PRIVATE ${SRCS})
