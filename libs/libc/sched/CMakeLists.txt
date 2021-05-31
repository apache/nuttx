############################################################################
# libs/libc/sched/Make.defs
#
#   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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

set(SRCS sched_getprioritymax.c sched_getprioritymin.c
		clock_timespec_subtract.c clock_time2ticks.c
		clock_timespec_add.c clock_ticks2time.c sched_dumpstack.c
		sched_backtrace.c)

if (NOT CONFIG_CANCELLATION_POINTS)
  list(APPEND SRCS task_setcanceltype.c task_testcancel.c)
endif()

if (NOT CONFIG_SMP)
  list(APPEND SRCS sched_cpucount.c)
endif()

if (NOT CONFIG_BUILD_KERNEL)
  list(APPEND SRCS task_startup.c)
endif()

if (CONFIG_CXX_INITIALIZE_SINIT)
  list(APPEND SRCS cxx_initialize_sinit.c)
endif()

if (CONFIG_CXX_INITIALIZE_MACHO)
  list(APPEND SRCS cxx_initialize_macho.c)
endif()

target_sources(c PRIVATE ${SRCS})
