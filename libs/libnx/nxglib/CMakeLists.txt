############################################################################
# libs/libnx/nxglib/Make.defs
#
#   Copyright (C) 2013-2014, 2017 Gregory Nutt. All rights reserved.
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

# Expose NXGL interfaces to applications

if(CONFIG_NXGLIB)

  # Files needed by both NX and NXFONTS
	set(SRCS nxglib_rgbblend.c)
	
	# Files needed only by NX
	
	if(CONFIG_NX)
    list(APPEND SRCS
      nxgl_area2rect.c nxglib_circlepts.c nxglib_circletraps.c
      nxglib_intersecting.c nxglib_nonintersecting.c nxglib_nullrect.c
      nxgl_rect2area.c nxglib_rectadd.c nxglib_rectcopy.c nxglib_rectinside.c
      nxglib_rectintersect.c nxglib_rectoffset.c nxglib_rectoverlap.c
      nxglib_rectsize.c nxglib_rectunion.c nxglib_rgb2yuv.c
      nxglib_runcopy.c nxglib_runoffset.c nxglib_splitline.c
      nxglib_trapcopy.c nxglib_trapoffset.c nxglib_vectoradd.c
      nxglib_vectsubtract.c nxglib_yuv2rgb.c)

		if (NOT CONFIG_NX_NPLANES EQUAL 1)
			list(APPEND SRCS nxglib_colorcmp.c nxglib_colorcopy.c)
		endif()
	endif()
	
  target_sources(nx PRIVATE ${SRCS})
endif()
