############################################################################
# libs/libnx/nxmu/Make.defs
#
#   Copyright (C) 2013, 2017 Gregory Nutt. All rights reserved.
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

# All of user-space hooks must be built in for the multi-user NX
# implementation so that they are available to the application in the kernel
# build (single use mode cannot be used with the kernel build)

if (CONFIG_NX)
  set(SRCS
    nxmu_sendserver.c nx_connect.c nx_disconnect.c
    nx_eventhandler.c nx_eventnotify.c nxmu_semtake.c
    nx_block.c nx_synch.c
    nx_kbdchin.c nx_kbdin.c nx_mousein.c
    nx_releasebkgd.c nx_requestbkgd.c nx_setbgcolor.c
    nxmu_sendwindow.c nx_closewindow.c nx_constructwindow.c
    nx_bitmap.c nx_fill.c nx_filltrapezoid.c nx_getposition.c
    nx_getrectangle.c nx_lower.c nx_modal.c nx_move.c nx_openwindow.c
    nx_raise.c nx_redrawreq.c nx_setpixel.c nx_setposition.c
    nx_setsize.c nx_setvisibility.c)

  if (CONFIG_NX_HWCURSOR)
    list(APPEND SRCS nx_cursor.c)
  elseif (CONFIG_NX_SWCURSOR)
    list(APPEND SRCS nx_cursor.c)
  endif()

  target_sources(nx PRIVATE ${SRCS})
endif()
