############################################################################
# tools/ProtectedLibs.mk
#
#   Copyright (C) 2007-2012, 2014, 2016-2018 Gregory Nutt. All rights
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

# NUTTXLIBS is the list of NuttX libraries that is passed to the
#   processor-specific Makefile to build the final NuttX target.
#   Libraries in FSDIRS are excluded if file descriptor support
#   is disabled.
# USERLIBS is the list of libraries used to build the final user-space
#   application
# EXPORTLIBS is the list of libraries that should be exported by
#   'make export' is

NUTTXLIBS = staging$(DELIM)libsched$(LIBEXT)
USERLIBS =

# Driver support.  Generally depends on file descriptor support but there
# are some components in the drivers directory that are needed even if file
# descriptors are not supported.

NUTTXLIBS += staging$(DELIM)libdrivers$(LIBEXT)

# Add libraries for board support

NUTTXLIBS += staging$(DELIM)libconfigs$(LIBEXT)

# Add libraries for syscall support.  The C library will be needed by
# both the kernel- and user-space builds.  For now, the memory manager (mm)
# is placed in user space (only).

NUTTXLIBS += staging$(DELIM)libstubs$(LIBEXT) staging$(DELIM)libkc$(LIBEXT)
NUTTXLIBS += staging$(DELIM)libkmm$(LIBEXT) staging$(DELIM)libkarch$(LIBEXT)
USERLIBS  += staging$(DELIM)libproxies$(LIBEXT) staging$(DELIM)libuc$(LIBEXT)
USERLIBS  += staging$(DELIM)libumm$(LIBEXT) staging$(DELIM)libuarch$(LIBEXT)

# Add libraries for two pass build support.  The special directory pass1
# may be populated so that application generated logic can be included into
# the kernel build

ifeq ($(CONFIG_BUILD_2PASS),y)
NUTTXLIBS += staging$(DELIM)libpass1$(LIBEXT)
endif

# Add libraries for C++ support.  CXX, CXXFLAGS, and COMPILEXX must
# be defined in Make.defs for this to work!

ifeq ($(CONFIG_HAVE_CXX),y)
USERLIBS += staging$(DELIM)$(LIBXX)$(LIBEXT)
endif

# Add library for application support.

ifneq ($(APPDIR),)
USERLIBS += staging$(DELIM)libapps$(LIBEXT)
endif

# Add libraries for network support

ifeq ($(CONFIG_NET),y)
NUTTXLIBS += staging$(DELIM)libnet$(LIBEXT)
endif

# Add libraries for Crypto API support

ifeq ($(CONFIG_CRYPTO),y)
NUTTXLIBS += staging$(DELIM)libcrypto$(LIBEXT)
endif

# Add libraries for file system support

ifeq ($(CONFIG_NFILE_DESCRIPTORS),0)
ifneq ($(CONFIG_NSOCKET_DESCRIPTORS),0)
NUTTXLIBS += staging$(DELIM)libfs$(LIBEXT)
endif
else
NUTTXLIBS += staging$(DELIM)libfs$(LIBEXT) staging$(DELIM)libbinfmt$(LIBEXT)
endif

# Add libraries for the NX graphics sub-system

ifeq ($(CONFIG_NX),y)
NUTTXLIBS += staging$(DELIM)libgraphics$(LIBEXT)
NUTTXLIBS += staging$(DELIM)libknx$(LIBEXT)
USERLIBS  += staging$(DELIM)libunx$(LIBEXT)
else ifeq ($(CONFIG_NXFONTS),y)
NUTTXLIBS += staging$(DELIM)libknx$(LIBEXT)
USERLIBS  += staging$(DELIM)libunx$(LIBEXT)
endif

# Add libraries for the Audio sub-system

ifeq ($(CONFIG_AUDIO),y)
NUTTXLIBS += staging$(DELIM)libaudio$(LIBEXT)
endif

# Add libraries for the Wireless sub-system

ifeq ($(CONFIG_WIRELESS),y)
NUTTXLIBS += staging$(DELIM)libwireless$(LIBEXT)
endif

# Add C++ library

ifeq ($(CONFIG_HAVE_CXX),y)
NUTTXLIBS += staging$(DELIM)$(LIBXX)$(LIBEXT)
endif

# Add DSP library

ifeq ($(CONFIG_LIBDSP),y)
NUTTXLIBS += staging$(DELIM)libdsp$(LIBEXT)
endif

# Export only the user libraries

EXPORTLIBS = $(USERLIBS)
