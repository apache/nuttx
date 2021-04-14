############################################################################
# libs/libc/misc/Make.defs
#
#   Copyright (C) 2011-2012, 2014, 2016, 2018 Gregory Nutt. All rights
#     reserved.
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

# Add the internal C files to the build
set(CSRCS)

if(NOT "${CONFIG_STDIO_DISABLE_BUFFERING}" STREQUAL "y")
	list(APPEND CSRCS lib_filesem.c)
endif()

# Add C files that depend on file OR socket descriptors
if(NOT "${CONFIG_NFILE_DESCRIPTORS}" STREQUAL "0")

	list(APPEND CSRCS lib_sendfile.c)
	
	if(NOT "${CONFIG_NFILE_STREAMS}" STREQUAL "0")
		list(APPEND CSRCS lib_streamsem.c)
	endif()

	if("${CONFIG_LIBC_IOCTL_VARIADIC}" STREQUAL "y")
		list(APPEND CSRCS lib_ioctl.c)
	endif()

	if("${CONFIG_PIPES}" STREQUAL "y")
		list(APPEND CSRCS lib_mkfifo.c)
	endif()

else() # CONFIG_NFILE_DESCRIPTORS > 0
	if(NOT "${CONFIG_NSOCKET_DESCRIPTORS}" STREQUAL "0")
	
		list(APPEND CSRCS lib_sendfile.c)
		
			if(NOT "${CONFIG_NFILE_STREAMS}" STREQUAL "0")
				list(APPEND CSRCS lib_streamsem.c)
			endif()
	
		if("${CONFIG_LIBC_IOCTL_VARIADIC}" STREQUAL "y")
			list(APPEND CSRCS lib_ioctl.c)
		endif()

	endif() # CONFIG_NSOCKET_DESCRIPTORS > 0
endif() # CONFIG_NFILE_DESCRIPTORS > 0

# Add the miscellaneous C files to the build
if("${CONFIG_DEBUG_FEATURES}" STREQUAL "y")
	list(APPEND CSRCS lib_debug.c)
endif()

# Keyboard driver encoder/decoder
if("${CONFIG_LIB_KBDCODEC}" STREQUAL "y")
	list(APPEND CSRCS
		lib_kbdencode.c
		lib_kbddecode.c
	)
endif()

# SLCD driver encoder/decoder
if("${CONFIG_LIB_SLCDCODEC}" STREQUAL "y")
	list(APPEND CSRCS
		lib_slcdencode.c
		lib_slcddecode.c
	)
endif()

# Environment search path support
if("${CONFIG_LIB_ENVPATH}" STREQUAL "y")
	list(APPEND CSRCS lib_envpath.c)
endif()

# Add the misc directory to the build
list(APPEND CSRCS
	lib_crc16.c
	lib_crc32.c
	lib_crc64.c
	lib_crc8.c
	lib_crc8ccitt.c
	lib_dumpbuffer.c
	lib_match.c
	lib_stream.c
	lib_tea_decrypt.c
	lib_tea_encrypt.c
	lib_uadd32x64.c
	lib_uadd64.c
	lib_umul32.c
	lib_umul32x64.c
	lib_umul64.c
	lib_usub64.c
	lib_usub64x32.c
	lib_utsname.c
	lib_xorshift128.c
)
set_parent_srcs(${CSRCS})
