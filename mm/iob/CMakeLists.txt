############################################################################
# mm/iob/Make.defs
#
#   Copyright (C) 2014, 2017-2018 Gregory Nutt. All rights reserved.
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

if(CONFIG_MM_IOB)

	set(SRCS)

  list(APPEND SRCS
    iob_add_queue.c iob_alloc.c iob_alloc_qentry.c iob_clone.c
    iob_concat.c iob_copyin.c iob_copyout.c iob_contig.c iob_free.c
    iob_free_chain.c iob_free_qentry.c iob_free_queue.c
    iob_initialize.c iob_pack.c iob_peek_queue.c iob_remove_queue.c
    iob_statistics.c iob_trimhead.c iob_trimhead_queue.c iob_trimtail.c
    iob_navail.c)
	
	if(CONFIG_IOB_NOTIFIER)
		list(APPEND SRCS iob_notifier.c)
	endif()
	
	if(CONFIG_DEBUG_FEATURES)
		list(APPEND SRCS iob_dump.c)
	endif()
	
  target_sources(mm PRIVATE ${SRCS})

endif()
