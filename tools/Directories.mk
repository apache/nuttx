############################################################################
# tools/Directories.mk
#
#   Copyright (C) 2007-2012, 2014, 2016-2019 Gregory Nutt. All rights
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

# Lists of build directories.
#
# CONTEXTDIRS include directories that have special, one-time pre-build
#   requirements.  Normally this includes things like auto-generation of
#   configuration specific files or creation of configurable symbolic links
# CLEANDIRS are the directories that the clean target will executed in.
#   These are all directories that we know about.
# CCLEANDIRS are directories that the clean_context target will execute in.
#   The clean_context target "undoes" the actions of the context target.
#   Only directories known to require cleaning are included.
# KERNDEPDIRS are the directories in which we will build target dependencies.
#   If NuttX and applications are built separately (CONFIG_BUILD_PROTECTED or
#   CONFIG_BUILD_KERNEL), then this holds only the directories containing
#   kernel files.
# USERDEPDIRS. If NuttX and applications are built separately (CONFIG_BUILD_PROTECTED),
#   then this holds only the directories containing user files. If
#   CONFIG_BUILD_KERNEL is selected, then applications are not build at all.

CLEANDIRS :=
CCLEANDIRS := boards $(APPDIR) graphics $(ARCH_SRC)
KERNDEPDIRS :=
USERDEPDIRS :=

# In the protected build, the applications in the apps/ directory will be
# into the userspace; in the flat build, the applications will be built into
# the kernel space.  But in the kernel build, the applications will not be
# built at all by this Makefile.

ifeq ($(CONFIG_BUILD_PROTECTED),y)
USERDEPDIRS += $(APPDIR)
else ifneq ($(CONFIG_BUILD_KERNEL),y)
KERNDEPDIRS += $(APPDIR)
else
CLEANDIRS += $(APPDIR)
endif

KERNDEPDIRS += sched drivers boards $(ARCH_SRC)
KERNDEPDIRS += fs binfmt

ifeq ($(EXTERNALDIR),external)
  KERNDEPDIRS += external
endif

CONTEXTDIRS = boards fs $(APPDIR) $(ARCH_SRC)
CLEANDIRS += pass1

ifeq ($(CONFIG_BUILD_FLAT),y)

KERNDEPDIRS += libs$(DELIM)libc mm
ifeq ($(CONFIG_HAVE_CXX),y)
KERNDEPDIRS += libs$(DELIM)libxx
else
CLEANDIRS += libs$(DELIM)libxx
endif

else

USERDEPDIRS += libs$(DELIM)libc mm
ifeq ($(CONFIG_HAVE_CXX),y)
USERDEPDIRS += libs$(DELIM)libxx
else
CLEANDIRS += libs$(DELIM)libxx
endif

endif

ifeq ($(CONFIG_LIB_SYSCALL),y)
CONTEXTDIRS += syscall
USERDEPDIRS += syscall
else
ifeq ($(CONFIG_SCHED_INSTRUMENTATION_SYSCALL),y)
CONTEXTDIRS += syscall
USERDEPDIRS += syscall
else
CLEANDIRS += syscall
endif
endif

ifeq ($(CONFIG_LIB_ZONEINFO_ROMFS),y)
CONTEXTDIRS += libs$(DELIM)libc
endif

ifeq ($(CONFIG_NX),y)
KERNDEPDIRS += graphics
CONTEXTDIRS += graphics
else
CLEANDIRS += graphics
endif

ifeq ($(CONFIG_NXFONTS),y)
ifeq ($(CONFIG_BUILD_FLAT),y)
KERNDEPDIRS += libs$(DELIM)libnx
else
USERDEPDIRS += libs$(DELIM)libnx
endif
CONTEXTDIRS += libs$(DELIM)libnx
else
CLEANDIRS += libs$(DELIM)libnx
endif

ifeq ($(CONFIG_AUDIO),y)
KERNDEPDIRS += audio
else
CLEANDIRS += audio
endif

ifeq ($(CONFIG_VIDEO),y)
KERNDEPDIRS += video
else
CLEANDIRS += video
endif

ifeq ($(CONFIG_WIRELESS),y)
KERNDEPDIRS += wireless
else
CLEANDIRS += wireless
endif

ifeq ($(CONFIG_LIBDSP),y)
KERNDEPDIRS += libs$(DELIM)libdsp
else
CLEANDIRS += libs$(DELIM)libdsp
endif

# Add networking directories to KERNDEPDIRS and CLEANDIRS

ifeq ($(CONFIG_NET),y)
KERNDEPDIRS += net
else
CLEANDIRS += net
endif

ifeq ($(CONFIG_CRYPTO),y)
KERNDEPDIRS += crypto
else
CLEANDIRS += crypto
endif

ifeq ($(CONFIG_OPENAMP),y)
KERNDEPDIRS += openamp
else
CLEANDIRS += openamp
endif

CLEANDIRS += $(KERNDEPDIRS) $(USERDEPDIRS)
