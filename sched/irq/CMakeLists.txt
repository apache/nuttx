############################################################################
# sched/irq/Make.defs
#
#   Copyright (C) 2014, 2016, 2018 Gregory Nutt. All rights reserved.
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

if("${CONFIG_SMP}" STREQUAL "y")
	if("${CONFIG_SPINLOCK_IRQ}" STREQUAL "y")
		if("${CONFIG_ARCH_GLOBAL_IRQDISABLE}" STREQUAL "y")
			list(APPEND CSRCS irq_spinlock.c)
		endif()
	endif()
endif()

if("${CONFIG_IRQCOUNT}" STREQUAL "y")
	list(APPEND CSRCS irq_csection.c)
endif()

if("${CONFIG_SCHED_IRQMONITOR}" STREQUAL "y")
	list(APPEND CSRCS irq_foreach.c)
	if("${CONFIG_FS_PROCFS}" STREQUAL "y")
		list(APPEND CSRCS irq_procfs.c)
	endif()
endif()

if("${CONFIG_IRQCHAIN}" STREQUAL "y")
	list(APPEND CSRCS irq_chain.c)
endif()

list(APPEND CSRCS
	irq_initialize.c
	irq_attach.c
	irq_dispatch.c
	irq_unexpectedisr.c
)
set_parent_srcs(${CSRCS})
