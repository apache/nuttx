############################################################################
# sched/task/CMakeLists.txt
#
#   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

if("${CONFIG_ARCH_HAVE_VFORK}" STREQUAL "y")
	if("${CONFIG_SCHED_WAITPID}" STREQUAL "y")
		list(APPEND CSRCS task_vfork.c)
	endif()
endif()

if(NOT "${CONFIG_BUILD_KERNEL}" STREQUAL "y")
	list(APPEND CSRCS task_spawn.c)
endif()

if("${CONFIG_CANCELLATION_POINTS}" STREQUAL "y")
	list(APPEND CSRCS
		task_setcanceltype.c
		task_testcancel.c
	)
endif()

if(NOT "${CONFIG_BINFMT_DISABLE}" STREQUAL "y")
	if("${CONFIG_LIBC_EXECFUNCS}" STREQUAL "y")
		list(APPEND CSRCS
			task_execv.c
			task_posixspawn.c
		)
	endif()
endif()

if("${CONFIG_SCHED_STARTHOOK}" STREQUAL "y")
	list(APPEND CSRCS task_starthook.c)
endif()

if("${CONFIG_SCHED_ATEXIT}" STREQUAL "y")
	list(APPEND CSRCS task_atexit.c)
endif()

if("${CONFIG_SCHED_ONEXIT}" STREQUAL "y")
	list(APPEND CSRCS task_onexit.c)
endif()

if("${CONFIG_CANCELLATION_POINTS}" STREQUAL "y")
	list(APPEND CSRCS task_cancelpt.c)
endif()

if("${CONFIG_SCHED_HAVE_PARENT}" STREQUAL "y")
	list(APPEND CSRCS task_reparent.c)
endif()

# Include task build support

list(APPEND CSRCS
	exit.c
	task_activate.c
	task_create.c
	task_delete.c
	task_exit.c
	task_exithook.c
	task_getgroup.c
	task_getpid.c
	task_init.c
	task_prctl.c
	task_recover.c
	task_restart.c
	task_setcancelstate.c
	task_setup.c
	task_spawnparms.c
	task_start.c
	task_terminate.c
)
set_parent_srcs(${CSRCS})
