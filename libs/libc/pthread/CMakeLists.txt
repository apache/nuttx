############################################################################
# libs/libc/pthread/Make.defs
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

if(NOT "${CONFIG_DISABLE_PTHREAD}" STREQUAL "y")

	# Add the pthread C files to the build
	set(CSRCS)

	if("${CONFIG_SMP}" STREQUAL "y")
		list(APPEND CSRCS
			pthread_attr_getaffinity.c
			pthread_attr_setaffinity.c
		)
	endif()
	
	if("${CONFIG_BUILD_PROTECTED}" STREQUAL "y")
		list(APPEND CSRCS pthread_startup.c)
	endif()
	
	# Add the pthread directory to the build
	list(APPEND CSRCS
		pthread_attr_destroy.c
		pthread_attr_getinheritsched.c
		pthread_attr_getschedparam.c
		pthread_attr_getschedpolicy.c
		pthread_attr_getstack.c
		pthread_attr_getstacksize.c
		pthread_attr_init.c
		pthread_attr_setinheritsched.c
		pthread_attr_setschedparam.c
		pthread_attr_setschedpolicy.c
		pthread_attr_setstack.c
		pthread_attr_setstacksize.c
		pthread_barrierattr_destroy.c
		pthread_barrierattr_getpshared.c
		pthread_barrierattr_init.c
		pthread_barrierattr_setpshared.c
		pthread_barrierdestroy.c
		pthread_barrierinit.c
		pthread_barrierwait.c
		pthread_condattr_destroy.c
		pthread_condattr_init.c
		pthread_conddestroy.c
		pthread_condinit.c
		pthread_mutexattr_destroy.c
		pthread_mutexattr_getprotocol.c
		pthread_mutexattr_getpshared.c
		pthread_mutexattr_getrobust.c
		pthread_mutexattr_gettype.c
		pthread_mutexattr_init.c
		pthread_mutexattr_setprotocol.c
		pthread_mutexattr_setpshared.c
		pthread_mutexattr_setrobust.c
		pthread_mutexattr_settype.c
		pthread_once.c
		pthread_rwlock.c
		pthread_rwlock_rdlock.c
		pthread_rwlock_wrlock.c
		pthread_setcancelstate.c
		pthread_setcanceltype.c
		pthread_testcancel.c
		pthread_yield.c
	)
	set_parent_srcs(${CSRCS})

endif() # CONFIG_DISABLE_PTHREAD
