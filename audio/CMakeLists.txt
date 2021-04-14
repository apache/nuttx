############################################################################
# audio/CMakeLists.txt
#
#   Copyright (C) 2013 Ken Pettit. All rights reserved.
#   Author: Ken Pettit <pettitkd@gmail.com>
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

if("${CONFIG_AUDIO}" STREQUAL "y")

	set(CSRCS)
	list(APPEND CSRCS audio.c)
	
	if("${CONFIG_AUDIO_COMP}" STREQUAL "y")
		list(APPEND CSRCS audio_comp.c)
	endif()
	
	# Include support for various drivers.  Each Make.defs file will add its
	# files to the source file list, add its DEPPATH info, and will add
	# the appropriate paths to the VPATH variable
	
	if("${CONFIG_AUDIO_FORMAT_PCM}" STREQUAL "y")
		list(APPEND CSRCS pcm_decode.c)
	endif()
	
	if(CSRCS)
		nuttx_add_library(audio ${CSRCS})
	endif()

endif()
