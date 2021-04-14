############################################################################
# sched/signal/Make.defs
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

if(NOT "${CONFIG_DISABLE_SIGNALS}" STREQUAL "y")

	set(CSRCS) 

	if("${CONFIG_SIG_DEFAULT}" STREQUAL "y")
		list(APPEND CSRCS sig_default.c)
	endif()

	if("${CONFIG_SIG_EVTHREAD}" STREQUAL "y")
		list(APPEND CSRCS sig_evthread.c)
	endif()

	if(NOT "${CONFIG_DISABLE_POLL}" STREQUAL "y")
		list(APPEND CSRCS
			sig_ppoll.c
			sig_pselect.c
		)
	endif()

	# Include signal build support
	
	list(APPEND CSRCS
		sig_action.c
		sig_allocpendingsigaction.c
		sig_cleanup.c
		sig_deliver.c
		sig_dispatch.c
		sig_findaction.c
		sig_initialize.c
		sig_kill.c
		sig_lowest.c
		sig_nanosleep.c
		sig_notification.c
		sig_pause.c
		sig_pending.c
		sig_procmask.c
		sig_queue.c
		sig_releasependingsigaction.c
		sig_releasependingsignal.c
		sig_removependingsignal.c
		sig_sleep.c
		sig_suspend.c
		sig_timedwait.c
		sig_unmaskpendingsignal.c
		sig_usleep.c
		sig_waitinfo.c
	)
	set_parent_srcs(${CSRCS})

endif()
