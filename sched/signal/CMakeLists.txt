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

set(SRCS)

if(CONFIG_SIG_DEFAULT)
  list(APPEND SRCS sig_default.c)
endif()

list(APPEND SRCS
  sig_initialize.c
  sig_action.c sig_procmask.c sig_pending.c sig_suspend.c
  sig_kill.c sig_queue.c sig_waitinfo.c sig_timedwait.c
  sig_findaction.c sig_allocpendingsigaction.c
  sig_releasependingsigaction.c sig_unmaskpendingsignal.c
  sig_removependingsignal.c sig_releasependingsignal.c sig_lowest.c
  sig_notification.c sig_cleanup.c sig_dispatch.c sig_deliver.c
  sig_pause.c sig_nanosleep.c sig_usleep.c sig_sleep.c
  sig_ppoll.c sig_pselect.c)

target_sources(sched PRIVATE ${SRCS})
